// This file is part of the DGF-SDK.
//
// Copyright (C) 2025 Advanced Micro Devices, Inc.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// clang-format off
#include "../include/DGFBaker.h"

#include "../source/Cluster.h"
#include "../source/ClusterBuilder.h"
#include "../source/StripGeneration.h"
#include "../source/Quantization.h"
#include "../source/HPG24BlockPacker.h"
#include "../source/SAHBlockPacker.h"
#include "../source/DebugOutput.h"

namespace DGFBaker
{
    static std::vector<uint32_t> GenerateVertexTable( 
        const std::vector<uint8_t>& blocksOut, 
        const std::vector<TriangleRemapInfo>& triangleRemap, 
        const BakerMesh::IndexReader& indexReader )
    {
        std::vector<uint32_t> vertexTable;
        size_t triBase = 0;
        for (size_t i = 0; i < blocksOut.size(); i += DGF::BLOCK_SIZE)
        {
            const uint8_t* block = blocksOut.data() + i;
            size_t numTris = DGF::DecodeTriangleCount(block);

            // read original indices for each triangle in the block, in block order
            uint32_t vertIndices[3*DGF::MAX_TRIS];
            uint32_t triIndices[DGF::MAX_TRIS];
            for (size_t t = 0; t < numTris; t++)
                triIndices[t] = triangleRemap[triBase + t].InputPrimIndex;                
            
            indexReader(vertIndices, triIndices, numTris);
            
            // rotate them to match block winding
            for (size_t t = 0; t < numTris; t++)
            {
                uint32_t idx[3];
                for (size_t k = 0; k < 3; k++)
                    idx[k] = vertIndices[3 * t + k];
                for (size_t k = 0; k < 3; k++)
                    vertIndices[3 * t + k] = idx[triangleRemap[triBase + t].IndexRotation[k]];
            }

            // build table of unique verts sorted by first use
            uint32_t uniqueIndices[DGF::MAX_VERTS];
            size_t numUniqueIndices = 0;
            for (size_t v = 0; v < 3 * numTris; v++)
            {
                bool seen = false;
                for (size_t k = 0; k < numUniqueIndices; k++)
                {
                    if (uniqueIndices[k] == vertIndices[v]) 
                    {
                        seen = true;
                        break;
                    }
                }
                if (!seen)
                    uniqueIndices[numUniqueIndices++] = vertIndices[v];                
            }

            vertexTable.insert(vertexTable.end(), &uniqueIndices[0], &uniqueIndices[numUniqueIndices]);
            triBase += numTris;
        }

        return std::move(vertexTable);
    }


    BakerOutput Baker::BakeDGF( const BakerMesh& bakerMesh )
    {
        DGF::Timer totalTime;
        DGF::PerformanceData perfData;

        std::vector<DGFBaker::TriangleRemapInfo> remapOut;
        std::vector<int32_t> ommIndicesOut;

        size_t maxVerts = std::min( m_config.clusterMaxVerts, DGF::MAX_UBLAS_VERTS );
        size_t maxFaces = std::min( m_config.clusterMaxFaces, DGF::MAX_UBLAS_TRIS );

        std::vector<DGF::Cluster> clusters;
        
        {
            DGF::Timer t;
            DGF::BuildClustersBinSAH(bakerMesh, clusters, maxVerts, maxFaces);
            if (m_config.validateClusters)
                DGF::ValidateClusters(clusters, bakerMesh, maxVerts, maxFaces);
            perfData.Clustering += t.Tick();
        }

        int8_t exponent;
        auto expInfo = DGF::ChooseQuantizationExponent(clusters, m_config.targetBitWidth);
        exponent = std::max(expInfo.minExp, expInfo.targetExp);

        std::vector<uint32_t> clusterTable;
        if (m_config.generateClusterTable)
            clusterTable.reserve(clusters.size());

        switch (m_config.packer)
        {
        case DGFBaker::PackerMode::HPG24:
            DGF::PackBlocksHPG24(clusters, perfData, exponent, m_config);
            break;

        default:
        case DGFBaker::PackerMode::SAH:
            DGF::PackBlocksSAH(clusters, perfData, exponent, m_config);
            break;
        }
        
        // collect DGF blocks from clusters into consecutive array, and generate cluster table if requested
        std::vector<uint8_t> dgfBlocksOut;
        for (DGF::Cluster& cluster : clusters)
        {
            if (m_config.generateClusterTable)
                clusterTable.push_back((uint32_t) (dgfBlocksOut.size() / DGF::BLOCK_SIZE) );

            dgfBlocksOut.insert(dgfBlocksOut.end(), cluster.dgfBlocks.begin(), cluster.dgfBlocks.end());

            // copy the remapping information, replace cluster-local indices with top-level ones
            if (m_config.generateTriangleRemap || m_config.generateVertexTable)
            {
                for (const DGF::TriangleRemap& localRemap : cluster.localRemap)
                {
                    DGFBaker::TriangleRemapInfo info;
                    info.InputPrimIndex = cluster.PrimIDs[localRemap.InputPrimID];
                    for (size_t i = 0; i < 3; i++)
                        info.IndexRotation[i] = localRemap.IndexRotation[i];

                    remapOut.push_back(info);
                }
            }

            ommIndicesOut.insert(ommIndicesOut.end(), cluster.outputOMMIndices.begin(), cluster.outputOMMIndices.end());
        }

        // generate block vertex table if requested
        std::vector<uint32_t> vertexTable;
        if (m_config.generateVertexTable)
        {
            DGF::Timer t;
            vertexTable = std::move(GenerateVertexTable(dgfBlocksOut, remapOut, bakerMesh.m_IndexReader));
            perfData.GenerateVertexTable += t.Tick();
        }

        // report perf data if requested
        perfData.TotalTime = totalTime.Tick();
        if (m_config.printPerfData && m_config.outputStream)
            DGF::PrintPerfData(*m_config.outputStream, perfData, m_config );
        
        return BakerOutput( std::move(dgfBlocksOut), std::move(remapOut), std::move(ommIndicesOut), std::move(vertexTable), std::move(clusterTable) );
    }

    DecodedMesh DecodeDGF(const BakerOutput& output)
    {
        return DecodeDGF(output.dgfBlocks.data(), output.dgfBlocks.size(), output.blockOMMIndices.data());
    }


    DecodedMesh DecodeDGF(const uint8_t* blocks, size_t sizeInBytes, const int32_t* pOMMIndices)    
    {
        const auto dgfBlocks = std::span<const uint8_t>(blocks, sizeInBytes);
        size_t numBlocks = dgfBlocks.size() / DGF::BLOCK_SIZE;

        // read the total vertex and triangle count
        size_t totalTris=0;
        size_t totalVerts=0;
        for ( size_t i=0; i<numBlocks; i++ )
        {
            totalTris  += DGF::DecodeTriangleCount( dgfBlocks.data() + i*DGF::BLOCK_SIZE );
            totalVerts += DGF::DecodeVertexCount( dgfBlocks.data() + i*DGF::BLOCK_SIZE );
        }
         
        // allocate memory
        std::vector<uint32_t> ib( totalTris * 3 );
        std::vector<float> vb( totalVerts * 3 );
        std::vector<TriangleAttributes> attribs( totalTris );
        std::vector<int8_t> quant( totalVerts );
        std::vector<uint32_t> triIndex(totalTris);

        // decode the blocks
        uint32_t vertexBase = 0;
        size_t ommIndexBase = 0;
        for ( size_t block=0; block<numBlocks; block++ )
        {
            const uint8_t* pBlock = dgfBlocks.data() + block * DGF::BLOCK_SIZE ;
            
            DGF::MetaData meta;
            DGF::DecodeMetaData( &meta, pBlock );

            uint8_t tmpIndices[DGF::MAX_INDICES];
            DGF::DecodeTriangleList( tmpIndices, pBlock );

            uint8_t opaqueFlags[DGF::MAX_TRIS];
            uint32_t geomID[DGF::MAX_TRIS];
            DGF::DecodeGeomIDs( geomID, opaqueFlags, pBlock );
            
            DGF::OffsetVert offsetVerts[DGF::MAX_VERTS];
            DGF::FloatVert floatVerts[DGF::MAX_VERTS];
            DGF::DecodeOffsetVerts( meta.numVerts, offsetVerts, pBlock );
            DGF::ConvertOffsetsToFloat( meta.numVerts, floatVerts, offsetVerts, meta );
            
            uint8_t triangleOMMIndices[DGF::MAX_TRIS];
            DGF::DecodeTriangleOMMIndices( triangleOMMIndices, pBlock );
            
            // copy indices
            for (size_t i = 0; i < 3*meta.numTris; i++)
                ib[3*meta.primIDBase+i] = tmpIndices[i] + vertexBase;

            // copy vertices
            for (size_t i = 0; i < meta.numVerts; i++)
            {
                for( size_t k=0; k<3; k++ )
                    vb[3*(vertexBase + i) + k] = floatVerts[i].xyz[k];
            }

            // build per-vertex quantization factor array
            for( size_t i=0; i<meta.numVerts; i++ )
                quant[vertexBase+i] = static_cast<int8_t>(((int)meta.exponent) - DGF::EXPONENT_BIAS);

            // copy triangle IDs
            for( uint32_t i=0; i<meta.numTris; i++ )
                triIndex[ meta.primIDBase + i ] = meta.primIDBase + i;

            // copy triangle attributes
            for (size_t i = 0; i < meta.numTris; i++) 
            {        
                auto& attribsOut = attribs[meta.primIDBase + i];
                uint8_t ommDescriptorIndex = triangleOMMIndices[i];
                attribsOut.geomID = geomID[i];
                attribsOut.opaque = opaqueFlags[i];
                attribsOut.hasOMM = meta.numOMMDescriptors > 0;
                if( attribsOut.hasOMM )
                    attribsOut.ommIndex = (int32_t) pOMMIndices[ommIndexBase + ommDescriptorIndex];
            }

            ommIndexBase += meta.numOMMDescriptors;
            vertexBase += meta.numVerts;
        }

        return DecodedMesh( std::move(vb), std::move(ib), std::move(quant), std::move(attribs), std::move(triIndex) );
    }


    bool Validator::ValidateDGF( const BakerMesh& input, const BakerOutput& output )
    {
        const std::vector<uint8_t>& dgfBlocks = output.dgfBlocks;
        const std::vector<TriangleRemapInfo>& remap = output.triangleRemap;

        if ( dgfBlocks.size() % DGF::BLOCK_SIZE != 0 )
        {
            m_errorStream << "Input DGF block array is not a multiple of block size!\n";
            return false;
        }
        if (remap.size() > input.GetTriangleCount())
        {
            m_errorStream << "Remap table is too large!\n";
            return false;
        }
        
        DecodedMesh decoded = DecodeDGF( output );
        
        size_t numInputTris = input.GetTriangleCount();
        size_t numOutputTris = decoded.GetTriangleCount();

        std::vector<bool> inputTriSeen( numInputTris );
        
        for ( size_t i=0; i<numOutputTris; i++ )
        {
            const TriangleRemapInfo& triRemap = remap[i];

            if (decoded.GetPrimIDs()[i] != i)
            {
                m_errorStream << "Output prim IDs are misnumbered!\n";
                return false;
            }

            for (size_t j = 0; j < 3; j++)
            {
                // verify that index rotation values are sane
                size_t idx = triRemap.IndexRotation[j];
                if (idx >= 3)
                {
                    m_errorStream << "Output triangle " << i << " bad index rotation value in remap table\n";
                    return false;
                }
            
                // verify that all 3 vertices are present in the rotation table
                // and that the vertex order is a valid winding-preserving rotation
                size_t expected_next = (idx+1)%3;
                size_t actual_next   = triRemap.IndexRotation[(j+1)%3];
                if (expected_next != actual_next)
                {
                    m_errorStream << "Output triangle " << i << " bad index rotation value in remap table\n";
                    return false;
                }
            }

            size_t inputIndex = triRemap.InputPrimIndex;
            
            // verify that remap prim indices are sane
            if (inputIndex >= input.GetTriangleCount())
            {
                m_errorStream << "Ouptut triangle " << i << " bad prim index in remap table\n";
                return false;
            }

            // verify that triangles are referenced only once
            if ( inputTriSeen[inputIndex] )
            {
                m_errorStream << "Input triangle " << inputIndex << " seen more than once in output remap table\n";
                return false;
            }

            inputTriSeen[inputIndex] = true;

             // verify that attributes match
            TriangleAttributes inputAttribs;
            input.ReadAttributes( &inputAttribs, triRemap.InputPrimIndex );

            TriangleAttributes outputAttribs = decoded.GetAttributeBuffer()[i];
            if (outputAttribs.geomID != inputAttribs.geomID ||
                outputAttribs.hasOMM != inputAttribs.hasOMM ||
                outputAttribs.opaque != inputAttribs.opaque ||
                (outputAttribs.ommIndex != inputAttribs.ommIndex && inputAttribs.hasOMM))
            {
                m_errorStream << "Output triangle: " << i << " attribute mismatch\n";
                return false;
            }

            // read the index buffer and rotate the indices
            uint32_t originalIndices[3];
            input.ReadIndices( originalIndices, triRemap.InputPrimIndex );
            uint32_t rotatedIndices[3];
            for( size_t j=0; j<3; j++ )
                rotatedIndices[j] = originalIndices[ triRemap.IndexRotation[j] ];

            for( size_t j=0; j<3; j++ )
            {
                // verify that the output VB/IB is identical to the input VB indexed with the remapped IB           
                uint32_t inputIndex  = rotatedIndices[j];
                uint32_t outputIndex = decoded.GetIndexBuffer()[3*i+j];
                
                // check for out of range indices
                if (outputIndex >= decoded.GetVertexCount())
                {
                    m_errorStream << "Output triangle: " << i << " topology index out of bounds!\n";
                    return false;
                }

                // verify that the output VB/IB produces the same verts as the input VB indexed with the remapped IB     
                //   and quantized according to what we found in the blocks      
                DGF::float3 inVert, outVert;
                input.ReadVertex( &inVert.x, inputIndex );
                for( size_t k=0; k <3; k++ )
                    outVert[k] = decoded.GetVertexBuffer()[ 3*outputIndex + k ];

                // compare quantized results
                int8_t exponent = decoded.GetVertexQuantizationFactorBuffer()[outputIndex];
                DGF::int3 Qin  = DGF::QuantizeVert( inVert, exponent );
                DGF::int3 QOut = DGF::QuantizeVert( outVert, exponent );
                for (size_t k = 0; k < 3; k++)
                {
                    if( Qin[k] != QOut[k] )
                    {
                        m_errorStream << "Vertex mismatch, output triangle: " << i << "\n"
                            << " Input position: " << triRemap.InputPrimIndex << "\n"
                            << " Input: "   << inVert[0] << "," << inVert[1]  << "," << inVert[2] << "\n"
                            << " Result: " << outVert[0] << "," << outVert[1] << "," << outVert[2] << "\n"
                            << " Exp: " << (int) exponent << "\n";
                        return false;
                    }
                }
            }
        }

        // if any triangles were not seen, verify that they are degenerates
        for (size_t i = 0; i < numInputTris; i++)
        {
            if (!inputTriSeen[i])
            {
                uint32_t indices[3];
                input.ReadIndices( indices, (uint32_t) i );
                
                uint32_t a = indices[0];
                uint32_t b = indices[1];
                uint32_t c = indices[2];

                if (!(a == b || b == c || a == c))
                {
                    m_errorStream << "Input triangle " << i << " missing from output\n";
                    return false;
                }                    
            }
        }

        return true;

    }

    PackerMode PackerModeFromString(const char* str)
    {
        if (strcmp(str, "HPG24") == 0)
        {
            return PackerMode::HPG24;
        }
        else if (strcmp(str, "SAH") == 0)
        {
            return PackerMode::SAH;
        }
        else
        {
            return PackerMode::DEFAULT;
        }
    }

    const char* PackerModeToString(PackerMode mode)
    {
        switch (mode)
        {
        case PackerMode::HPG24:     return "HPG24";
        case PackerMode::SAH:       return "SAH";
        default:                    return "????";
        }
    }
    

    std::vector<uint32_t> GenerateDGFBlockMap(const uint8_t* pBlocks, size_t numBlocks, size_t totalTris, size_t groupSize)
    {
        DGF_ASSERT( groupSize % 32 == 0 && groupSize >= 32 );

        std::vector<uint32_t> blockMap;
        blockMap.reserve(1 + (groupSize / 32));

        uint32_t blockIndex = 0;
        uint32_t positionInBlock = 0;
        
        for (size_t i = 0; i < totalTris; i += groupSize) 
        {
            DGF_ASSERT(blockIndex < (1 << 24));
            DGF_ASSERT(positionInBlock < DGF::MAX_TRIS);

            const uint8_t* pBlock = pBlocks + blockIndex * DGF::BLOCK_SIZE;
            uint32_t       numTris = (uint32_t)DGF::DecodeTriangleCount(pBlock);

            // store the block index and start position for the first triangle in this group
            uint32_t entryX = (blockIndex)+(positionInBlock << 24);
            blockMap.push_back(entryX);

            // reserve space for the end-of-block bits
            for (size_t j = 0; j < groupSize; j += 32)
                blockMap.push_back(0);

            // search for block boundaries within the group
            uint32_t* eobBits = &blockMap[blockMap.size() - (groupSize / 32)];
            for (size_t j = 0; j < groupSize; j++ ) 
            {                
                if (positionInBlock == (numTris - 1)) 
                {
                    // move to next block, and set this triangle's 'end-of-block' bit
                    blockIndex++;
                    positionInBlock = 0;
                    eobBits[j / 32] |= (1 << (j % 32));

                    // stop early if we're out of blocks... remaining bits don't matter
                    if (blockIndex == numBlocks)
                        break;

                    // read triangle count for next block
                    numTris = (uint32_t)DGF::DecodeTriangleCount(pBlocks + blockIndex * DGF::BLOCK_SIZE);
                }
                else 
                {
                    // stay in the same block
                    positionInBlock++;
                }                
            }
        }
        
        return std::move(blockMap);
    }

}
