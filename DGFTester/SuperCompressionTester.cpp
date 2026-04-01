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

#include <DGF.h>
#include <DGFBaker.h>
#include <DGFS.h>
#include "CommandLine.h"

#include <chrono>
#include <fstream>

bool CompareDGFs(const uint8_t* a, const uint8_t* b, const int32_t* pBlockAOMMIndices, const int32_t* pDecodedOMMIndices )
{
    DGF::MetaData metaA;
    DGF::DecodeMetaData(&metaA, a);
    DGF::MetaData metaB;
    DGF::DecodeMetaData(&metaB, b);

    if (metaA.numVerts != metaB.numVerts )
        return false;
    if (metaA.numTris != metaB.numTris )
        return false;

    DGF::OffsetVert ovA[DGF::MAX_VERTS];
    DGF::OffsetVert ovB[DGF::MAX_VERTS];
    DGF::FloatVert fvA[DGF::MAX_VERTS];
    DGF::FloatVert fvB[DGF::MAX_VERTS];
    DGF::DecodeOffsetVerts(metaA.numVerts, ovA, a);
    DGF::ConvertOffsetsToFloat(metaA.numVerts, fvA, ovA, metaA);
    DGF::DecodeOffsetVerts(metaB.numVerts, ovB, b);
    DGF::ConvertOffsetsToFloat(metaB.numVerts, fvB, ovB, metaB);

    for (size_t i = 0; i < metaA.numVerts; i++)
        for (size_t j = 0; j < 3; j++)
            if (fvA[i].xyz[j] != fvB[i].xyz[j])
                return false;

    DGF::TriControlValues ctrlA[DGF::MAX_TRIS];
    DGF::TriControlValues ctrlB[DGF::MAX_TRIS];
    uint8_t idxA[DGF::MAX_INDICES];
    uint8_t idxB[DGF::MAX_INDICES];
    DGF::DecodeTopology(ctrlA, idxA, a);
    DGF::DecodeTopology(ctrlB, idxB, b);
    size_t idx = 0;
    for (size_t t = 0; t < metaA.numTris; t++)
    {
        if (ctrlA[t] != ctrlB[t])
            return false;

        if (idxA[idx] != idxB[idx])
            return false;
        
        idx++;
        if (ctrlA[t] == DGF::TC_RESTART)
        {
            if (idxA[idx] != idxB[idx])
                return false;
            idx++;
            if (idxA[idx] != idxB[idx])
                return false;
            idx++;
        }
    }

    uint32_t geomA[DGF::MAX_TRIS];
    uint32_t geomB[DGF::MAX_TRIS];
    uint8_t opa[DGF::MAX_TRIS];
    uint8_t opb[DGF::MAX_TRIS];
    DGF::DecodeGeomIDs(geomA, opa, a);
    DGF::DecodeGeomIDs(geomB, opb, b);
    for (size_t t = 0; t < metaA.numTris; t++)
    {
        if (geomA[t] != geomB[t])
            return false;
        if (opa[t] != opb[t])
            return false;
    }

    if (metaA.numOMMDescriptors != metaB.numOMMDescriptors)
        return false;

    if (metaA.numOMMDescriptors > 0)
    {
        uint8_t paletteIndexA[DGF::MAX_TRIS];
        uint8_t paletteIndexB[DGF::MAX_TRIS];
        DGF::DecodeTriangleOMMIndices(paletteIndexA, a);
        DGF::DecodeTriangleOMMIndices(paletteIndexB, b);

        // reconstruct an the OMM palette for block B.  
        //  We do not want the test to fail if the blocks reference the same OMMs but with a different order
        int32_t blockBPalette[DGF::MAX_OMM_DESCRIPTORS];
        uint8_t entryValid[DGF::MAX_OMM_DESCRIPTORS] = { 0 };
        for (size_t i = 0; i < metaB.numTris; i++)
        {
            // check whether two triangles with the same palette index have
            //  different OMM indices in the output.  That's a bug
            uint8_t palettePosition = paletteIndexB[i];
            int32_t descriptor = pDecodedOMMIndices[i];
            if (entryValid[palettePosition])
            {
                if (blockBPalette[palettePosition] != descriptor)
                    return false;
            }
            else
            {
                blockBPalette[palettePosition] = descriptor;
                entryValid[palettePosition] = 1;
            }
        }

        // compare each triangle's OMM with the corresponding one from block A, and ensure they match
        for (size_t i = 0; i < metaA.numTris; i++)
        {
            int32_t expectedIndex = pBlockAOMMIndices[paletteIndexA[i]];
            int32_t actualIndex = blockBPalette[paletteIndexB[i]];
            if (expectedIndex != actualIndex)
                return false;
        }
    }

    // compare userdata
    if (metaA.haveUserData != metaB.haveUserData)
        return false;

    if (metaA.haveUserData)
    {
        uint32_t userDataA;
        uint32_t userDataB;
        DGF::ReadUserData(a, &userDataA, 0, DGF::MAX_USERDATA_SIZE);
        DGF::ReadUserData(a, &userDataB, 0, DGF::MAX_USERDATA_SIZE);
        if (userDataA != userDataB)
            return false;
    }

    // compare primID base fields
    if (metaA.primIDBase != metaB.primIDBase)
        return false;

    return true;
}

bool CompareMeshes(const DGFBaker::DecodedMesh& a, const DGFBaker::DecodedMesh& b)
{
    if (a.GetTriangleCount() != b.GetTriangleCount())
    {
        return false;
    }

    size_t numTris = a.GetTriangleCount();
    for (size_t i = 0; i < numTris; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            uint32_t idxa = a.GetIndexBuffer()[3 * i + j];
            uint32_t idxb = b.GetIndexBuffer()[3 * i + j];           
            const float* vtxa = a.GetVertexBuffer() + 3 * idxa;
            const float* vtxb = b.GetVertexBuffer() + 3 * idxb;
            for (size_t k = 0; k < 3; k++)
            {
                if (vtxa[k] != vtxb[k])
                    return false;
            }
        }

        auto attribA = a.GetAttributeBuffer()[i];
        auto attribB = b.GetAttributeBuffer()[i];
        if (attribA != attribB)
            return false;
    }
    return true;
}

inline uint64_t ReadBits(const uint8_t* bytes, size_t start, size_t len)
{
    DGF_ASSERT(len <= 48);

    size_t firstByte = (start) / 8;
    size_t lastByte = (start + (len - 1)) / 8;
    size_t numBytes = 1 + lastByte - firstByte;

    uint64_t dst = 0;
    for (size_t i = 0; i < numBytes; i++)
    {
        uint64_t currByte = bytes[firstByte + i];
        dst |= (currByte << (8 * i));
    }

    return (dst >> (start % 8)) & ((1ull << len) - 1);
}


static void ConvertCompressed1ToFloat(float* output, const void* compressed1, size_t numVerts)
{
    const uint32_t* header = (const uint32_t*)compressed1;

    size_t xSize = (header[1] & 0xf) + 1;
    size_t ySize = ((header[1]>>4) & 0xf) + 1;
    size_t zSize = (header[2] & 0xf) + 1;
    uint32_t exponent = header[0] & 0xff;
    int32_t AnchorX = ((int32_t)header[0]) >> 8;
    int32_t AnchorY = ((int32_t)header[1]) >> 8;
    int32_t AnchorZ = ((int32_t)header[2]) >> 8;
    
    float scale = std::ldexp(1.0f, exponent - DGF::EXPONENT_BIAS);

    uint32_t xmask = (1u << xSize) - 1;
    uint32_t ymask = (1u << ySize) - 1;
    uint32_t zmask = (1u << zSize) - 1;

    size_t vertexSize = xSize + ySize + zSize;
    const uint8_t* packedOffsets = (const uint8_t*)(header + 3);
    size_t bitPos = 0;
    for (size_t i = 0; i < numVerts; i++)
    {
        uint64_t bits = ReadBits(packedOffsets, bitPos, vertexSize);
        bitPos += vertexSize;

        int32_t x = bits & xmask;
        int32_t y = (bits >> xSize) & ymask;
        int32_t z = (bits >> (xSize + ySize)) & zmask;

        output[3*i+0] = (AnchorX + x) * scale;
        output[3*i+1] = (AnchorY + y) * scale;
        output[3*i+2] = (AnchorZ + z) * scale;
    }

}

DGFBaker::DecodedMesh CreateDecodedMeshFromMeshletBuffers(
    const DGFS::MeshletDecoder::DecodeInfo& info,
    const DGFS::MeshletDecoder::DecodeBuffers& buffers,
    uint8_t bytesPerIndex)
{
    std::vector<float> vb(3 * info.numVertices);
    std::vector<uint32_t> ib(3 * info.numTriangles);
    std::vector<DGFBaker::TriangleAttributes> attribs(info.numTriangles);

    // copy vertices
    if( info.vertexFormat == DGFS::MeshletDecoder::VF_FLOAT3 )
    {
        const float* vertexBuffer = (const float*)buffers.vertexBuffer;
        for (size_t v = 0; v < info.numVertices; v++)
            for (size_t k = 0; k < 3; k++)
                vb[3 * v + k] = vertexBuffer[3 * v + k];
    }
    else
    {
        ConvertCompressed1ToFloat(vb.data(), buffers.vertexBuffer, info.numVertices);
    }

    // copy indices based on bytesPerIndex
    for (size_t t = 0; t < info.numTriangles; t++)
    {
        for (size_t k = 0; k < 3; k++)
        {
            uint32_t index;
            switch (bytesPerIndex)
            {
            case 1:
                index = buffers.indexBuffer[3 * t + k];
                break;
            case 2:
                index = ((const uint16_t*)buffers.indexBuffer)[3 * t + k];
                break;
            case 4:
                index = ((const uint32_t*)buffers.indexBuffer)[3 * t + k];
                break;
            default:
                index = buffers.indexBuffer[3 * t + k];
                break;
            }
            ib[3 * t + k] = index;
        }
    }

    // copy triangle attributes
    for( size_t t=0; t<info.numTriangles; t++ )
    {
        uint32_t geomIDAndFlags = buffers.geomIDAndFlagsBase;
        if (buffers.geomIDAndFlagsBuffer != nullptr)
        {
            if (buffers.geomIDAndFlagsIndexBuffer != nullptr)
                geomIDAndFlags = buffers.geomIDAndFlagsBuffer[buffers.geomIDAndFlagsIndexBuffer[t]];
            else
                geomIDAndFlags = buffers.geomIDAndFlagsBuffer[t];
        }

        attribs[t].geomID = geomIDAndFlags & 0x00FFFFFF;
        attribs[t].opaque = (geomIDAndFlags & DGFS::GEOMETRY_FLAG_OPAQUE) != 0;
        if (buffers.ommIndexBuffer != nullptr )
        {
            attribs[t].ommIndex = buffers.ommIndexBuffer[t];
            attribs[t].hasOMM = true;
        }
    }

    return DGFBaker::DecodedMesh(std::move(vb), std::move(ib), {}, std::move(attribs), {});
}

template< class TDecoder >
void DecodeClusters(
    const std::vector< std::vector<uint8_t> >& streams, 
    std::vector<typename TDecoder::DecodeBuffers>& buffers, 
    const TDecoder& decoder )
{    
    size_t numStreams = streams.size();
    for (size_t i = 0; i < numStreams; i++)
        decoder.Decode(buffers[i], streams[i].data(), streams[i].size());
}

template< class TDecoder >
void SetupDecodeBuffers(
    const std::vector< std::vector<uint8_t> >& streams,
    std::vector<typename TDecoder::DecodeBuffers>& buffers, 
    const TDecoder& decoder,
    std::vector<uint8_t>& bufferMemory )
{
    size_t numStreams = streams.size();
    size_t allocationSize = 0;
    for (size_t i = 0; i < numStreams; i++)
    {
        auto decodeInfo = decoder.GetDecodeInfo(streams[i].data(), streams[i].size());
        allocationSize += decodeInfo.totalBufferSize;
    }

    bufferMemory.resize(allocationSize);

    size_t offset = 0;
    for (size_t i = 0; i < numStreams; i++)
    {
        auto decodeInfo = decoder.GetDecodeInfo(streams[i].data(), streams[i].size());
        decoder.BindBufferMemory(buffers[i], &bufferMemory[offset], decodeInfo);
        DGF_ASSERT(offset + decodeInfo.totalBufferSize <= bufferMemory.size());
        offset += decodeInfo.totalBufferSize;
    }
}

template< class TDecoder >
struct DecodeOutput
{
    std::vector<uint8_t> bufferMemory;
    std::vector<typename TDecoder::DecodeBuffers> buffers;
    double setupTime;
    double encodeTime;
};

template< class TDecoder >
DecodeOutput<TDecoder> Decode( const std::vector< std::vector<uint8_t> >& streams, const TDecoder& decoder)
{
    DecodeOutput<TDecoder> output;
    output.buffers.resize(streams.size());

    auto startTime = std::chrono::high_resolution_clock::now();
    SetupDecodeBuffers(streams, output.buffers, decoder, output.bufferMemory);
    auto setupTime = std::chrono::high_resolution_clock::now();
    DecodeClusters(streams, output.buffers, decoder);
    auto endTime = std::chrono::high_resolution_clock::now();

    double setupMS  = std::chrono::duration<double>(setupTime - startTime).count();
    double encodeMS = std::chrono::duration<double>(endTime - setupTime).count();

    output.setupTime = setupMS;
    output.encodeTime = encodeMS;

    return output;
}

template< class TDecodeOutput >
void PrintStatistics(const TDecodeOutput& output, size_t numTris)
{
    printf("    Buffer setup time: %.2f ms\n", output.setupTime);
    printf("    Decode time: %.2f ms\n", output.encodeTime);
    printf("    Decoded size: %.2f MB (%.2f B/Tri)\n",
        output.bufferMemory.size() / (1024.0 * 1024.0),
        output.bufferMemory.size() / (double)numTris);
}

int TestDGFS(const CommandLine& cmdLine, const DGFBaker::BakerMesh& mesh)
{
    size_t numTris = mesh.GetTriangleCount();
    
    DGFBaker::Config cfg = cmdLine.bakerConfig;
    DGFBaker::Baker baker(cfg);

    printf("Building initial DGF blocks\n");    
   
    auto dgfStart = std::chrono::high_resolution_clock::now();
    std::vector<DGFBaker::BakerMeshCluster> clusters = baker.BuildClusters(mesh);
    DGFBaker::ClusteredBakerMesh clusteredMesh = DGFBaker::ClusteredBakerMesh(mesh, clusters);
    DGFBaker::BakerOutputPreclustered output = baker.BakeDGFPreclustered(clusteredMesh);    
    auto dgfEnd = std::chrono::high_resolution_clock::now();
    
    double dgfBakerTime = std::chrono::duration<double>(dgfEnd - dgfStart).count();
    printf("    Build time: %.2f ms\n", 1000.0 * dgfBakerTime);

    // select single cluster for debugging
    if (cmdLine.dgfsSingleCluster)
    {
        auto cluster = std::move(output.clusters[cmdLine.dgfsSingleClusterNumber]);
        output.clusters.clear();
        output.clusters.push_back(std::move(cluster));
    }

    // measure dgf block size
    size_t initialBlockSize = 0;
    for (auto& cluster : output.clusters)
        initialBlockSize += cluster.dgfBlocks.size();

    printf("    Initial DGF size: %zu bytes (%.2f MB, %.2f B/Tri)\n", 
        initialBlockSize, initialBlockSize / (1024.0 * 1024.0), initialBlockSize / (double)numTris);

    size_t numClusters = output.clusters.size();

    // randomize primID base fields for testing
    if (cmdLine.dgfsRandomPrimID)
    {
        printf("Randomizing primID bases\n");
        for (size_t i = 0; i < numClusters; i++)
        {
            auto& cluster = output.clusters[i];

            for (size_t b = 0; b < cluster.dgfBlocks.size(); b += DGF::BLOCK_SIZE)
            {
                uint8_t* block = &cluster.dgfBlocks[b];
                uint32_t primBase = rand() & 0x07ffffff;
                uint32_t* dwords = (uint32_t*)block;
                dwords[4] |= primBase;
            }
        }
    }

    // randomize userdata fields for testing
    if (cfg.enableUserData && cmdLine.dgfsRandomUserData)
    {
        printf("Randomizing userdata fields\n");
        for (size_t i = 0; i < numClusters; i++)
        {
            auto& cluster = output.clusters[i];
            for (size_t b = 0; b < cluster.dgfBlocks.size(); b += DGF::BLOCK_SIZE)
            {
                uint8_t* block = &cluster.dgfBlocks[b];
                uint32_t userData = rand() & 0x07ffffff;
                DGF::WriteUserData(block, &userData, 0, 4);
            }
        }
    }

    printf("Encoding DGFS streams:\n");

    DGFS::Encoder::Config encoderConfig;
    encoderConfig.allowInternalThreading = cmdLine.bakerConfig.allowInternalThreading;
    DGFS::Encoder encoder(encoderConfig);
    
    auto encodeStart = std::chrono::high_resolution_clock::now();
    std::vector< std::vector<uint8_t> > dgfsStreams = encoder.EncodeClusters(output);
    auto encodeEnd = std::chrono::high_resolution_clock::now();

    size_t dgfsSize = 0;
    for (auto& stream : dgfsStreams)
        dgfsSize += stream.size();

    printf("    Encode time: %.2f ms\n", 1000.0 * std::chrono::duration<double>(encodeEnd - encodeStart).count());
    printf("    DGFS size: %zu bytes (%.2f MB, %.2f B/Tri)\n", dgfsSize, dgfsSize / (1024.0 * 1024.0), dgfsSize / (double)numTris);

    DGFS::MeshletDecoder::Config meshletDecoderConfig;
    meshletDecoderConfig.bytesPerIndex = cmdLine.dgfsBytesPerIndex;
    meshletDecoderConfig.indexedGeomIDs = cmdLine.dgfsIndexedGeomIDs;
    
    if (cmdLine.dgfsCompressed1Vertices)
        meshletDecoderConfig.vertexFormat = DGFS::MeshletDecoder::VF_COMPRESSED1;

    DGFS::MeshletDecoder meshletDecoder(meshletDecoderConfig);
    DGFS::DGFDecoder dgfDecoder;

    printf("Using bytesPerIndex: %d\n", cmdLine.dgfsBytesPerIndex);

    printf("Meshlet decode:\n");
    auto meshletOutput = Decode(dgfsStreams, meshletDecoder);
    PrintStatistics(meshletOutput,numTris);

    printf("DGF decode:\n");
    auto dgfOutput     = Decode(dgfsStreams, dgfDecoder);
    PrintStatistics(dgfOutput, numTris);

    if (cmdLine.validate)
    {
        printf("Validating results...");
        for (size_t i = 0; i < meshletOutput.buffers.size(); i++)
        {
            auto& dgfs = dgfsStreams[i];
            auto decodeInfo = meshletDecoder.GetDecodeInfo(dgfs.data(), dgfs.size());

            const uint8_t* blocks = output.clusters[i].dgfBlocks.data();
            size_t numBlocks = output.clusters[i].dgfBlocks.size() / DGF::BLOCK_SIZE;

            const int32_t* pBlockOMMIndices = nullptr;
            if (!output.clusters[i].blockOMMIndices.empty())
                pBlockOMMIndices = output.clusters[i].blockOMMIndices.data();

            // test meshlet decode
            {
                auto dgfsMesh = CreateDecodedMeshFromMeshletBuffers(decodeInfo, meshletOutput.buffers[i], cmdLine.dgfsBytesPerIndex);
                auto dgfMesh = DGFBaker::DecodeDGF(blocks, 128 * numBlocks, pBlockOMMIndices);

                if (!CompareMeshes(dgfsMesh, dgfMesh))
                {
                    printf("\nMeshlet decoder is broken!  Cluster number: %zu\n", i);
                    return 1;
                }
            }

            // test DGF decode
            {
                auto decodeInfo = dgfDecoder.GetDecodeInfo(dgfs.data(), dgfs.size());

                if (decodeInfo.numBlocks != numBlocks)
                {
                    printf("\nWrong decoded block size!?!");
                    return 1;
                }

                size_t descriptorOffset = 0;
                size_t triOffset = 0;
                for (size_t b = 0; b < numBlocks; b++)
                {
                    const uint8_t* blockA = blocks + b * DGF::BLOCK_SIZE;
                    const uint8_t* blockB = dgfOutput.buffers[i].blocksBuffer + b * DGF::BLOCK_SIZE;

                    if (!CompareDGFs(blockA, blockB, pBlockOMMIndices + descriptorOffset, dgfOutput.buffers[i].ommIndexBuffer + triOffset))
                    {
                        printf("\nDGF block mismatch:  Cluster: %zu  Block: %zu\n", i, b);
                        return 1;
                    }

                    descriptorOffset += DGF::DecodeOMMDescriptorCount(blockA);
                    triOffset += DGF::DecodeTriangleCount(blockA);
                }
            }
        }
        printf("ok\n");
    }

    if (cmdLine.binDumpFile)
    {
        printf("Dumping...\n");

        std::ofstream file;
        file.open(cmdLine.binDumpFile, std::ios::binary);
        if (file.fail())
        {
            printf("Error opening: %s\n", cmdLine.binDumpFile);
            return false;
        }

        // write number of streams and start positions in a header
        uint32_t numStreams = (uint32_t)dgfsStreams.size();
        file << numStreams;

        uint32_t offset=0;
        for (auto& stream : dgfsStreams)
        {
            file << offset;
            offset += (uint32_t)stream.size();
        }


        // write streams
        for (auto& stream : dgfsStreams)
            file.write((const char*)stream.data(), stream.size());

        if (!file.good())
        {
            printf("Error writing: %s\n", cmdLine.binDumpFile);
            return 1;
        }

        file.close();

        printf("Wrote: %s\n", cmdLine.binDumpFile);
    }

    return 0;
}
