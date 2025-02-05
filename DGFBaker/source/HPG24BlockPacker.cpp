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
#include "HPG24BlockPacker.h"
#include "StripGeneration.h"
#include "MaterialCompression.h"

#include "StripGeneration.h"
#include "Quantization.h"

#include "DGFBaker.h"

namespace DGF
{
    struct TmpBlock
    {
        std::bitset<MAX_UBLAS_VERTS> vertsInBlock;
        uint8_t vertexClusterToBlock[MAX_UBLAS_VERTS];   // index of each cluster vert in 'blockIndexBuffer' index space
        uint8_t vertexBlockToCluster[DGF::MAX_VERTS];   // index of each cluster vert in 'blockIndexBuffer' index space
        uint8_t blockIndexBuffer[3 * MAX_TRIS]; // block-local triangle indices
        
        int3 minVertex = int3( INT32_MAX,  INT32_MAX,  INT32_MAX);
        int3 maxVertex = int3(-INT32_MAX, -INT32_MAX, -INT32_MAX);
        size_t numTris  = 0;
        size_t numVerts = 0;

        uint32_t geomIDPayloads[DGF::MAX_GEOMID_BITS]; // unique payloads
        uint32_t geomIDDiffMask = 0; // mask of bits which are not the same across geomIDs
        size_t numGeomIDPayloads = 0;
        uint8_t triangleGeomIDIndices[DGF::MAX_TRIS]; // payload index for each block triangle

        uint8_t triangleIndices[DGF::MAX_TRIS]; // index of each block tri in the cluster
        TriangleRemap triangleRemap[DGF::MAX_TRIS];

        uint8_t x_bits;
        uint8_t y_bits;
        uint8_t z_bits;

        int32_t ommDescriptors[DGF::MAX_OMM_DESCRIPTORS];
        uint8_t triangleOMMIndices[DGF::MAX_TRIS];
        size_t numOMMDescriptors = 0;

    };

    void ExportBlock( BlockDesc& outputBlock, 
                      const TmpBlock& block, 
                      std::span<const int3> clusterVertices, 
                      TriangleRemap* remapOut, 
                      const ClusterAdjacencyGraph& clusterGraph,
                      const uint8_t forcedBlockWidth[3] )
    {        
        outputBlock.anchor   = block.minVertex;
        outputBlock.numVerts = block.numVerts;
        outputBlock.x_bits = block.x_bits;
        outputBlock.y_bits = block.y_bits;
        outputBlock.z_bits = block.z_bits;
               
        // build strips for the last time
        BlockAdjacencyGraph blockGraph;
        blockGraph.Build(clusterGraph, block.triangleIndices, block.numTris);
        GenerateStrips(outputBlock.strips, blockGraph, remapOut, block.blockIndexBuffer, block.numTris);

        // renumber the indices in the output strip, and copy vertices into their first-use position as we go
        uint8_t firstUse[DGF::MAX_VERTS];
        std::bitset<DGF::MAX_VERTS> vertSeen;
        size_t vertexCount = 0;
        for (size_t i = 0; i < outputBlock.strips.numIndices; i++)
        {
            uint8_t idx = outputBlock.strips.indexBuffer[i];
            if (!vertSeen.test(idx))
            {
                // track first-use position for each vertex
                size_t vertPosition = vertexCount++;
                firstUse[idx] = (uint8_t)vertPosition;
                vertSeen.set(idx);

                // copy each vertex on first use
                uint8_t clusterIndex = block.vertexBlockToCluster[idx];
                int3 delta = clusterVertices[clusterIndex] - block.minVertex;
                outputBlock.blockVerts[vertPosition].xyz[0] = (uint16_t) delta.x;
                outputBlock.blockVerts[vertPosition].xyz[1] = (uint16_t) delta.y;
                outputBlock.blockVerts[vertPosition].xyz[2] = (uint16_t) delta.z;
            }

            outputBlock.strips.indexBuffer[i] = firstUse[idx];
        }

        // If offset bit-width has been forced, resulting in data loss,
        //   then clamp the value.  
        // 
        // This prevents the encoder from asserting during round-trip validation.
        //   This may cause cracking, but that's the user's problem for the moment
        //
        for (size_t k = 0; k < 3; k++)
        {
            size_t w = forcedBlockWidth[k];
            if (w > 0)
            {
                w = std::min(w, DGF::MAX_VERTEX_COMPONENT_BITS);
                uint16_t max = (uint16_t)((1 << w) - 1);
                for (size_t i = 0; i < vertexCount; i++)
                {
                    uint16_t v = outputBlock.blockVerts[i].xyz[k];
                    outputBlock.blockVerts[i].xyz[k] = std::min(v, max);
                }
            }
        }

        for (size_t i = 0; i < block.numTris; i++)
        {
            // index of the triangle in the block order which landed at position 0 in the strip
            uint32_t blockTriIndex = remapOut[i].InputPrimID;

            // copy geomID information in strip order
            uint32_t payload = block.geomIDPayloads[block.triangleGeomIDIndices[blockTriIndex]];
            outputBlock.triangleGeomIDs[i] = payload >> 1;
            outputBlock.triangleOpaqueFlags[i] = payload & 1;

            // replace the remap index with this triangle's position in the original cluster
            uint8_t clusterTriIndex = block.triangleIndices[remapOut[i].InputPrimID];
            remapOut[i].InputPrimID = clusterTriIndex;
        }

        // OMM stuff
        outputBlock.numOMMDescriptors = block.numOMMDescriptors;
        for (size_t d = 0; d < block.numOMMDescriptors; d++)
            outputBlock.ommDescriptors[d] = block.ommDescriptors[d];
        memcpy(outputBlock.triangleOMMIndices, block.triangleOMMIndices, block.numTris);
 
    }

    bool AddTriangle(TmpBlock& block,
                     std::span<const int3>  clusterVertices,
                     const uint8_t* indices,
                     uint32_t geomIDPayload,
                     uint8_t triangleIndex,
                     size_t triLimit, size_t vertLimit,
                     const ClusterAdjacencyGraph& clusterGraph,
                     const uint8_t forcedVertexWidth[3],
                     bool userData, 
                     bool haveOMM,
                     int32_t ommDescriptor
                  )
    {
        if (block.numTris == triLimit)
            return false;

        int3 minVert = block.minVertex;
        int3 maxVert = block.maxVertex;
        uint8_t numVerts = (uint8_t) block.numVerts;
        uint8_t numTris  = (uint8_t) block.numTris;
        uint8_t triIndexInBlock = numTris++;

        // add vertices and indices to the block
        for (size_t i = 0; i < 3; i++)
        {
            uint8_t idx = indices[i];
            if (!block.vertsInBlock.test(idx))
            {
                if (numVerts == vertLimit)
                    return false;

                minVert = min(minVert, clusterVertices[idx]);
                maxVert = max(maxVert, clusterVertices[idx]);
                block.vertexClusterToBlock[idx] = numVerts;
                block.vertexBlockToCluster[numVerts] = idx;
                block.vertsInBlock.set(idx);
                idx = numVerts++;
            }
            else
            {
                idx = block.vertexClusterToBlock[idx];
            }

            block.blockIndexBuffer[3 * triIndexInBlock + i] = idx;
        }
        block.triangleIndices[triIndexInBlock] = triangleIndex;

        uint8_t x_bits = (uint8_t) BitsNeededUnsigned(maxVert.x - minVert.x);
        uint8_t y_bits = (uint8_t) BitsNeededUnsigned(maxVert.y - minVert.y);
        uint8_t z_bits = (uint8_t) BitsNeededUnsigned(maxVert.z - minVert.z);

        if (x_bits > DGF::MAX_VERTEX_COMPONENT_BITS || y_bits > DGF::MAX_VERTEX_COMPONENT_BITS || z_bits > DGF::MAX_VERTEX_COMPONENT_BITS) {
            if (numTris > 1) {
                return false;
            }
            // note: truncating vertex component bits here causes validation to fail later.
            // this only happens for individual triangles which are too big (in terms of vertex position bits)
            // to fit in a single block.
            x_bits = std::min<uint8_t>(x_bits, DGF::MAX_VERTEX_COMPONENT_BITS);
            y_bits = std::min<uint8_t>(y_bits, DGF::MAX_VERTEX_COMPONENT_BITS);
            z_bits = std::min<uint8_t>(z_bits, DGF::MAX_VERTEX_COMPONENT_BITS);
        }

        // apply forced offset widths
        if (forcedVertexWidth[0] > 0)
            x_bits = std::min(forcedVertexWidth[0], (uint8_t)DGF::MAX_VERTEX_COMPONENT_BITS);
        if (forcedVertexWidth[1] > 0)
            y_bits = std::min(forcedVertexWidth[1], (uint8_t)DGF::MAX_VERTEX_COMPONENT_BITS);
        if (forcedVertexWidth[2] > 0)
            z_bits = std::min(forcedVertexWidth[2], (uint8_t)DGF::MAX_VERTEX_COMPONENT_BITS);

        size_t bits_per_vertex = CalculateVertexBits(std::max<size_t>(1, x_bits) + std::max<size_t>(1, y_bits) + std::max<size_t>(1, z_bits));
        size_t vertex_bits = bits_per_vertex * numVerts;

        // align vertex bits to byte boundary
        if (vertex_bits % 8)
            vertex_bits += 8 - (vertex_bits % 8);

        // stop early if verts don't fit
        if (vertex_bits > DGF::MAX_FRONT_BUFFER_BIT_SIZE)
            return false;

        // see if this is a new geomID payload
        bool newGeomID = true;
        for (size_t i = 0; i < block.numGeomIDPayloads; i++)
        {
            if (block.geomIDPayloads[i] == geomIDPayload)
            {
                block.triangleGeomIDIndices[triIndexInBlock] = (uint8_t) i;
                newGeomID = false;
                break;
            }
        }

        size_t numGeomIDPayloads = block.numGeomIDPayloads + ((newGeomID) ? 1 : 0);
        uint32_t diffMask = block.geomIDDiffMask;
        if (newGeomID)
        {
            // fail the block if we're at capacity
            if (numGeomIDPayloads > DGF::MAX_GEOM_IDS)
                return false;

            // add new payload to the block
            block.geomIDPayloads[block.numGeomIDPayloads] = geomIDPayload;
            block.triangleGeomIDIndices[triIndexInBlock] = (uint8_t) block.numGeomIDPayloads;

            // compute running mask of payload bits which differ
            diffMask |= (block.geomIDPayloads[0] ^ geomIDPayload);
        }

        if (haveOMM)
        {
            // search for this descriptor in the descriptor list
            uint8_t descriptorIndex = (uint8_t) block.numOMMDescriptors;
            for (size_t d = 0; d < block.numOMMDescriptors; d++)
            {
                if (block.ommDescriptors[d] == ommDescriptor)
                {
                    descriptorIndex = (uint8_t) d;
                    break;
                }
            }

            // store a new descriptor
            if (descriptorIndex == block.numOMMDescriptors)
            {
                if (block.numOMMDescriptors == DGF::MAX_OMM_DESCRIPTORS)
                    return false;
                block.ommDescriptors[block.numOMMDescriptors++] = ommDescriptor;
            }

            block.triangleOMMIndices[triIndexInBlock] = descriptorIndex;
        }

        // determine compressed geomID size
        size_t geomPaletteSize = GetMaterialIdsBitSize(numGeomIDPayloads, numTris, block.geomIDPayloads[0], diffMask);

        // determine omm palette size
        size_t ommPaletteSize = DGF::ComputeOMMPaletteBitSize(block.numOMMDescriptors, numTris);

        // make sure verts and IDs all fit
        size_t frontBufferSize = vertex_bits + ommPaletteSize + geomPaletteSize;
        if (frontBufferSize > DGF::MAX_FRONT_BUFFER_BIT_SIZE)
            return false;
        
        // build strips to make sure they fit
        BlockAdjacencyGraph blockGraph; 
        blockGraph.Build(clusterGraph, block.triangleIndices, numTris);
        DGFStripInfo tmpStrip;
        DGF::TriangleRemap tmpRemap[DGF::MAX_VERTS];
        GenerateStrips(tmpStrip, blockGraph, tmpRemap, block.blockIndexBuffer, numTris);

        // does everything fit?
        auto topologySizes = tmpStrip.PackedSize();
        if (topologySizes.indexSize > DGF::MAX_INDEX_BUFFER_BIT_SIZE)
            return false;

        size_t userDataByteSize = userData ? DGF::MAX_USERDATA_SIZE : 0;
        size_t BIT_BUDGET = 8 * (DGF::BLOCK_SIZE - DGF::HEADER_SIZE - userDataByteSize);
        if (frontBufferSize + topologySizes.Total() > BIT_BUDGET)
            return false;

        // keep this triangle in the block
        block.numTris = numTris;
        block.numVerts = numVerts;
        block.numGeomIDPayloads = numGeomIDPayloads;
        block.geomIDDiffMask = diffMask;
        block.minVertex = minVert;
        block.maxVertex = maxVert;
        block.x_bits = x_bits;
        block.y_bits = y_bits;
        block.z_bits = z_bits;
        return true;
    }

    // adds triangles to blocks one by one
    void PackTriangles(std::vector<BlockDesc>& blocksOut,
        std::span<TriangleRemap> remapOut,
        PerformanceData& perfData,
        std::span<const int3>     clusterVertices,
        std::span<const uint8_t>  clusterIndices,
        std::span<const DGFBaker::TriangleAttributes> clusterAttributes,
        size_t triLimit, size_t vertLimit, const uint8_t forcedVertexWidth[3], bool userData )
    {
        size_t numTris = clusterIndices.size() / 3;
        DGF_ASSERT(numTris > 0);

        std::bitset<MAX_UBLAS_TRIS> usedTris;
        size_t numUsed = 0;

        // build cluster adjacency graph
        ClusterAdjacencyGraph clusterGraph;
        clusterGraph.Build(clusterIndices.data(), numTris);

        // build geomID payloads
        uint32_t clusterGeomIDPayloads[MAX_UBLAS_TRIS];
        for (size_t i = 0; i < numTris; i++)
        {
            uint32_t geomID = clusterAttributes.empty() ? 0 : clusterAttributes[i].geomID;
            uint32_t opaque = clusterAttributes.empty() ? 1 : clusterAttributes[i].opaque;
            clusterGeomIDPayloads[i] = (geomID << 1) + (opaque & 1);
        }

        do
        {
            // pick first available triangle as initial candidate
            size_t candidate = 0;
            while (candidate < numTris && usedTris[candidate])
                candidate++;

            size_t primOffset = numUsed;
            TmpBlock block;
            while (numUsed < numTris && block.numTris < DGF::MAX_TRIS)
            {
                // see if we can pack
                Timer t;
                bool success = AddTriangle(block, clusterVertices, clusterIndices.data() + 3 * candidate, 
                    clusterGeomIDPayloads[candidate], (uint8_t) candidate, triLimit, vertLimit, clusterGraph, forcedVertexWidth, 
                    userData, clusterAttributes[candidate].hasOMM, clusterAttributes[candidate].ommIndex );

                perfData.AddTriangle += t.Tick();

                // block is full?
                if (!success)
                    break;

                // we can keep this triangle.
                numUsed++;
                if (numUsed == numTris)
                    break;

                // mark triangle as used
                usedTris.set(candidate);

                // choose a new candidate. favor triangles which share the most verts with those already in the block
                t.Tick();

                size_t bestCandidate = numTris;
                size_t bestVertCount = 0;
                for (size_t i = 0; i < numTris; i++)
                {
                    if (usedTris[i])
                        continue;

                    size_t numSharedVerts = 0;
                    for (size_t v = 0; v < 3; v++)
                        if (block.vertsInBlock[clusterIndices[3 * i + v]])
                            numSharedVerts++;

                    if (bestCandidate == numTris || numSharedVerts > bestVertCount)
                    {
                        bestCandidate = i;
                        bestVertCount = numSharedVerts;
                    }
                }
                candidate = bestCandidate;
                perfData.PickBestCandidate += t.Tick();
            }

            // finalize the block
            DGF::Timer t;
            
            auto& blockDesc = blocksOut.emplace_back();
            blockDesc.primBase = primOffset;
            ExportBlock(blockDesc, block, clusterVertices, &remapOut[primOffset], clusterGraph, forcedVertexWidth);

            perfData.ExportBlock += t.Tick();
            
        } while (numUsed < numTris);
    }

    void BuildBlocks(std::vector<BlockDesc>& blocksOut, std::span<TriangleRemap> remapOut, PerformanceData& perfData,
                     std::span<const int3> clusterVertices, int8_t exponent, std::span<const uint8_t> clusterIndices,
                     std::span<const DGFBaker::TriangleAttributes> clusterAttributes,
                     size_t triLimit, size_t vertLimit, const uint8_t forcedVertexWidth[3], bool enableUserData )
    {
        Timer t;

        PackTriangles(blocksOut, remapOut, perfData, clusterVertices, clusterIndices, clusterAttributes, triLimit, vertLimit, forcedVertexWidth, enableUserData);

        for (auto& block: blocksOut)
            block.exponent = exponent;

        perfData.BuildBlocks += t.Tick();
    }


    static void EncodeBlock( uint8_t* blockData, const BlockDesc& desc, int8_t exponent, bool userData, bool validate )
    {
        const uint32_t* triangleGeomIDs = desc.triangleGeomIDs;
        const uint8_t* triangleOpaqueFlags = desc.triangleOpaqueFlags;
        const DGF::OffsetVert* relativeVerts = desc.blockVerts;

        size_t xBits = std::max<size_t>(1, desc.x_bits);
        size_t yBits = std::max<size_t>(1, desc.y_bits);
        size_t zBits = std::max<size_t>(1, desc.z_bits);

        DGF_ASSERT(xBits <= DGF::MAX_VERTEX_COMPONENT_BITS);
        DGF_ASSERT(yBits <= DGF::MAX_VERTEX_COMPONENT_BITS);
        DGF_ASSERT(zBits <= DGF::MAX_VERTEX_COMPONENT_BITS);

        // force vertex size to a multiple of bit alignment
        size_t totalBits = xBits + yBits + zBits;
        size_t roundedBits = CalculateVertexBits(totalBits);
        size_t extraBits = roundedBits - totalBits;

        while (extraBits)
        {
            if (xBits < DGF::MAX_VERTEX_COMPONENT_BITS)
            {
                xBits++;
                extraBits--;
                if (extraBits == 0)
                    break;
            }
            if (yBits < DGF::MAX_VERTEX_COMPONENT_BITS)
            {
                yBits++;
                extraBits--;
                if (extraBits == 0)
                    break;
            }
            if (zBits < DGF::MAX_VERTEX_COMPONENT_BITS)
            {
                zBits++;
                extraBits--;
                if (extraBits == 0)
                    break;
            }
        }

        DGF_ASSERT(xBits <= DGF::MAX_VERTEX_COMPONENT_BITS);
        DGF_ASSERT(yBits <= DGF::MAX_VERTEX_COMPONENT_BITS);
        DGF_ASSERT(zBits <= DGF::MAX_VERTEX_COMPONENT_BITS);
        DGF_ASSERT(((xBits + yBits + zBits) % DGF::VERTEX_BIT_ALIGNMENT) == 0);

        DGF::EncoderInput input;
        input.exponent = desc.exponent + DGF::EXPONENT_BIAS; // TODO: Use biased throughout
        input.anchorX = desc.anchor.x;
        input.anchorY = desc.anchor.y;
        input.anchorZ = desc.anchor.z;
        input.indexBuffer = desc.strips.indexBuffer;
        input.numIndices = (uint8_t) desc.strips.numIndices;
        input.numTris    = (uint8_t) desc.strips.numTriangles;
        input.numVerts   = (uint8_t) desc.numVerts;
        input.primIDBase = (uint32_t) desc.primBase;
        input.triangleGeomIDs = triangleGeomIDs;
        input.triangleOpaqueFlags = triangleOpaqueFlags;
        input.triControl = desc.strips.triControl;
        input.verts = relativeVerts;
        input.xBits = (uint8_t) xBits;
        input.yBits = (uint8_t) yBits;
        input.zBits = (uint8_t) zBits;
        input.numOMMDescriptors = (uint8_t) desc.numOMMDescriptors;
        input.triangleOMMDescriptorIndices = desc.triangleOMMIndices;
        input.userDataSize = userData ? DGF::MAX_USERDATA_SIZE : 0;
        input.roundTripValidation = validate;

        DGF::Encode(blockData, input);
    }


    void EncodeBlocks(Cluster& cluster, PerformanceData& perfData, const std::vector<BlockDesc>& blockDescs, bool userData, bool validate )
    {
        Timer t;
        cluster.dgfBlocks.resize(blockDescs.size() * DGF::BLOCK_SIZE);
      
        for (size_t i = 0; i < blockDescs.size(); i++)
        {
            const BlockDesc& blockDesc = blockDescs[i];

            EncodeBlock(cluster.dgfBlocks.data() + DGF::BLOCK_SIZE * i, blockDesc, blockDesc.exponent, userData, validate );

            for (size_t d = 0; d < blockDesc.numOMMDescriptors; d++)
                cluster.outputOMMIndices.push_back(blockDesc.ommDescriptors[d]);            
        }
        perfData.EncodeBlocks += t.Tick();
        perfData.NumBlocks += blockDescs.size();
    }



    // High-precision morton code
    struct MortC
    {
        uint64_t lo;
        uint64_t hi;
        inline bool operator<(const MortC& rhs) const
        {
            const MortC& a = *this;
            const MortC& b = rhs;
            if (a.hi != b.hi)
                return a.hi < b.hi;
            else
                return a.lo < b.lo;
        }

        static MortC Morton(uint32_t x, uint32_t y, uint32_t z)
        {
            // TODO: optimize me
            MortC m = { 0, 0 };
            for (size_t i = 0; i < 32; i++)
            {
                uint64_t xbit = (x >> i) & 1;
                uint64_t ybit = (y >> i) & 1;
                uint64_t zbit = (z >> i) & 1;
                uint64_t triple = xbit + (ybit << 1) + (zbit << 2);

                if (i <= 20)
                    m.lo |= (triple << (3 * i));
                else
                    m.hi |= (triple << (3 * (i - 20)));
            }
            return m;
        }
    };

    struct MortKey
    {
        MortC mort;
        uint32_t idx;
        bool operator<(const MortKey& mk) const { return mort < mk.mort; }
    };

    static void MortonSort( DGF::Cluster& cluster )
    {
        AABB box;
        for (float3& vert : cluster.Vertices)
            box.Expand(vert);

        // we use the diagonal length when computing morton codes so that
        //  distances are not distorted if the mesh has a non-square aspect ratio
        float3 diagonal = box.Max - box.Min;
        float diag = std::max(diagonal.x, std::max(diagonal.y, diagonal.z));

        size_t numPrims = cluster.Indices.size() / 3;
        std::vector<MortKey> mort;
        mort.reserve(numPrims);
        for (size_t triIndex = 0; triIndex < numPrims; triIndex++)
        {
            float3 Centroid(0, 0, 0);
            for (size_t j = 0; j < 3; j++)
                Centroid = Centroid + cluster.Vertices[cluster.Indices[3 * triIndex + j]];

            float x = Centroid.x / 3;
            float y = Centroid.y / 3;
            float z = Centroid.z / 3;
            x = (x - box.Min.x) / diag;
            y = (y - box.Min.y) / diag;
            z = (z - box.Min.z) / diag;
            uint32_t ix = (uint32_t)(((double)x) * (double)(0xffffffff));
            uint32_t iy = (uint32_t)(((double)y) * (double)(0xffffffff));
            uint32_t iz = (uint32_t)(((double)z) * (double)(0xffffffff));

            MortKey m;
            m.mort = MortC::Morton(ix, iy, iz);
            m.idx = (uint32_t)(triIndex);
            mort.push_back(m);
        }

        std::sort(mort.begin(), mort.end());

        std::vector<uint32_t> inputPrimIDs = cluster.PrimIDs;
        std::vector<uint8_t> inputVertexIndices = cluster.Indices;
        for (size_t i = 0; i < numPrims; i++)
        {
            for (size_t j = 0; j < 3; j++)
                cluster.Indices[3 * i + j] = inputVertexIndices[3 * mort[i].idx + j];

            cluster.PrimIDs[i] = inputPrimIDs[mort[i].idx];
        }
        if (!cluster.Attributes.empty())
        {
            auto inputAttribs = cluster.Attributes;
            for (size_t i = 0; i < numPrims; i++)
                cluster.Attributes[i] = inputAttribs[mort[i].idx];
        }
    }


    void PackBlocksHPG24( std::vector<DGF::Cluster>& clusters, PerformanceData& perfData, int8_t exponent, const DGFBaker::Config& config )
    {
        DGF::Timer t;

        std::vector<DGF::BlockDesc> blocks;
        std::vector<DGF::int3> quantizedVerts;

        size_t clusterPrimIDPrefix = 0;
        for (DGF::Cluster& cluster : clusters)
        {
            size_t numTris = cluster.Indices.size() / 3;
            size_t numVerts = cluster.Vertices.size();

            quantizedVerts.resize(numVerts);

            MortonSort(cluster);

            {
                DGF::Timer t;
                DGF::QuantizeVerts(quantizedVerts, cluster.Vertices, exponent);
                perfData.QuantizeVertices += t.Tick();
            }

            cluster.localRemap.resize(numTris);
            DGF::BuildBlocks(blocks, cluster.localRemap, perfData, quantizedVerts, exponent, cluster.Indices, cluster.Attributes,
                config.blockMaxTris, config.blockMaxVerts, config.blockForcedOffsetWidth, config.enableUserData);

            // offset the primitive IDs for this cluster
            for (DGF::BlockDesc& block : blocks)
                block.primBase += clusterPrimIDPrefix;

            DGF::EncodeBlocks(cluster, perfData, blocks, config.enableUserData, config.encoderRoundTripValidation );

            clusterPrimIDPrefix += numTris;

            blocks.clear();
        }

        perfData.PackTriangles += t.Tick();
    }
}