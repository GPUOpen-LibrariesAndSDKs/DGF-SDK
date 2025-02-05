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


#include "SAHBlockPacker.h"

#include "MaterialCompression.h"
#include "Quantization.h"

#include "SIMD.h"

namespace DGF
{
    static constexpr size_t MAX_CLUSTER_VERTS = DGFBaker::MAX_CLUSTER_VERTICES;
    static constexpr size_t MAX_CLUSTER_TRIS = DGFBaker::MAX_CLUSTER_FACES;
    
    static_assert(MAX_CLUSTER_VERTS <= 256 && MAX_CLUSTER_TRIS <= 256, "More index bits please!");

    struct TempBlock
    {
        int8_t   exponent;
        uint8_t  firstTri;
        uint8_t  triCount;
    };

    static void AdjustExponent( int8_t& exponent, int3& vertMin, int3& vertMax, int3* blockVerts, size_t numVerts )
    {
        uint32_t diffMask = 0;
        for (size_t i = 0; i < numVerts; i++)
        {
            const int3& v = blockVerts[i];
            diffMask |= (v.x | v.y | v.z);
        }

        int shift = std::countr_zero(diffMask);
        if (shift)
        {
            int adjustedExp = exponent + shift;
            if (adjustedExp <= DGF::EXPONENT_MAX - DGF::EXPONENT_BIAS)
            {
                exponent = static_cast<int8_t>(adjustedExp);
                vertMin = vertMin >> shift;
                vertMax = vertMax >> shift;
                for (size_t i = 0; i < numVerts; i++)
                    blockVerts[i] = blockVerts[i] >> shift;
            }
        }
    }

    struct SortKeys
    {
        uint8_t keys[3]; // triangle's position in an axis-sorted cluter list
        uint8_t idx;     // index of triangle in cluster
    };

    static bool BuildBlock(
        uint32_t primIDBase,
        int8_t exponent,
        size_t numTris,
        const SortKeys* sortKeys,
        std::span<const int3> verts,
        const DGFBaker::TriangleAttributes* clusterAttributes,
        const DGFBaker::Config& config,
        ClusterAdjacencyGraph& clusterGraph,
        Cluster* cluster,
        PerformanceData& perfData)
    {
        // too many tris to fit in block?
        if (numTris > DGF::MAX_TRIS || numTris > config.blockMaxTris)
            return false;

        // scan vertices
        std::bitset<DGF::MAX_CLUSTER_VERTS> vertSeen;
        uint8_t clusterIndexToBlockIndex[DGF::MAX_CLUSTER_VERTS];
        uint8_t blockLocalIndices[DGF::MAX_TRIS * 3];
        size_t numVerts = 0;
        int3 vertMin = int3(S24_MAX, S24_MAX, S24_MAX);
        int3 vertMax = int3(S24_MIN, S24_MIN, S24_MIN);
        int3 blockLocalVerts[DGF::MAX_VERTS];
        for (size_t i = 0; i < numTris; i++)
        {
            size_t triIndex = sortKeys[i].idx;
            for (size_t j = 0; j < 3; j++)
            {
                uint8_t idx = cluster->Indices[3*triIndex+j];
                if (!vertSeen.test(idx))
                {
                    // too many verts to fit in block?
                    if (numVerts == DGF::MAX_VERTS)
                        return false;

                    // add new vertex to the block and update block AABB
                    int3 vert = verts[idx];
                    blockLocalVerts[numVerts] = vert;
                    vertMin = min(vertMin, vert);
                    vertMax = max(vertMax, vert);
                    vertSeen.set(idx);

                    // map block vertices to their positions in the cluster
                    clusterIndexToBlockIndex[idx] = (uint8_t)numVerts;
                    numVerts++;
                }

                blockLocalIndices[3 * i + j] = clusterIndexToBlockIndex[idx];
            }
        }

        if (numVerts >= config.blockMaxVerts)
            return false;

        if (config.enableExponentAdjust)
            AdjustExponent(exponent, vertMin, vertMax, blockLocalVerts, numVerts);


        uint8_t x_bits = (uint8_t)BitsNeededUnsigned(vertMax.x - vertMin.x);
        uint8_t y_bits = (uint8_t)BitsNeededUnsigned(vertMax.y - vertMin.y);
        uint8_t z_bits = (uint8_t)BitsNeededUnsigned(vertMax.z - vertMin.z);
        if (x_bits > DGF::MAX_VERTEX_COMPONENT_BITS || y_bits > DGF::MAX_VERTEX_COMPONENT_BITS || z_bits > DGF::MAX_VERTEX_COMPONENT_BITS)
        {
            // exponent clamping should prevent this
            DGF_ASSERT(false);
        }

        // apply forced offset widths
        auto forceX = config.blockForcedOffsetWidth[0];
        auto forceY = config.blockForcedOffsetWidth[1];
        auto forceZ = config.blockForcedOffsetWidth[2];
        if (forceX > 0)
            x_bits = std::min(forceX, (uint8_t)DGF::MAX_VERTEX_COMPONENT_BITS);
        if (forceY > 0)
            y_bits = std::min(forceY, (uint8_t)DGF::MAX_VERTEX_COMPONENT_BITS);
        if (forceZ > 0)
            z_bits = std::min(forceZ, (uint8_t)DGF::MAX_VERTEX_COMPONENT_BITS);

        size_t bits_per_vertex = CalculateVertexBits(std::max<size_t>(1, x_bits) + std::max<size_t>(1, y_bits) + std::max<size_t>(1, z_bits));
        size_t vertex_bits = bits_per_vertex * numVerts;

        // align vertex bits to byte boundary
        if (vertex_bits % 8)
            vertex_bits += 8 - (vertex_bits % 8);

        // stop early if verts don't fit
        if (vertex_bits > DGF::MAX_FRONT_BUFFER_BIT_SIZE)
            return false;

        // scan triangle attributes
        uint32_t geomIDPayloads[DGF::MAX_GEOM_IDS];
        uint32_t numGeomIDPayloads = 0;
        uint32_t geomIDPayloadDiffMask = 0;        
        int32_t ommDescriptors[DGF::MAX_OMM_DESCRIPTORS];
        uint8_t triOMMIndices[DGF::MAX_INDICES];
        size_t numOMMDescriptors = 0;
        size_t numOMMIndices = 0;

        for (size_t i = 0; i < numTris; i++)
        {
            size_t triIndex  = sortKeys[i].idx;
            const DGFBaker::TriangleAttributes& attributes = clusterAttributes[triIndex];
            uint32_t payload = (attributes.geomID<<1) + attributes.opaque;
            
            bool newGeomID = true;
            for (size_t p = 0; p < numGeomIDPayloads; p++)
            {
                if (geomIDPayloads[p] == payload)
                {
                    newGeomID = false;
                    break;
                }
            }
            if (newGeomID)
            {
                if (numGeomIDPayloads == DGF::MAX_GEOM_IDS)
                    return false; // doesn't fit

                geomIDPayloads[numGeomIDPayloads++] = payload;
                geomIDPayloadDiffMask |= geomIDPayloads[0] ^ payload;
            }

            if (attributes.hasOMM)
            {
                // search for matching OMM descriptor already in block
                int32_t ommDescriptor = attributes.ommIndex;
                size_t descriptorIndex = numOMMDescriptors;
                for (size_t d = 0; d < numOMMDescriptors; d++)
                {
                    if (ommDescriptors[d] == ommDescriptor)
                    {
                        descriptorIndex = d;
                        break;
                    }
                }

                // not found, add new one
                if (descriptorIndex == numOMMDescriptors)
                {
                    if (numOMMDescriptors == DGF::MAX_OMM_DESCRIPTORS)
                        return false;
                    ommDescriptors[numOMMDescriptors++] = ommDescriptor;
                }

                // record per-triangle OMM indices
                triOMMIndices[numOMMIndices++] = (uint8_t) descriptorIndex;
            }
        }

        DGF_ASSERT(numOMMIndices == 0 || numOMMIndices == numTris); // can't mix OMM and non-OMM

        // determine compressed geomID and OMM palette size
        size_t geomPaletteSize = GetMaterialIdsBitSize(numGeomIDPayloads, numTris, geomIDPayloads[0], geomIDPayloadDiffMask);
        size_t ommPaletteSize = DGF::ComputeOMMPaletteBitSize(numOMMDescriptors, numOMMIndices);

        // make sure verts and IDs all fit
        size_t frontBufferSize = vertex_bits + ommPaletteSize + geomPaletteSize;
        if (frontBufferSize > DGF::MAX_FRONT_BUFFER_BIT_SIZE)
            return false;

        // build strips 
        DGFStripInfo strip;
        DGF::TriangleRemap remap[DGF::MAX_TRIS];
        {
            DGF::Timer t;
            
            uint8_t triangleIndices[DGF::MAX_TRIS];
            for (size_t i = 0; i < numTris; i++)
                triangleIndices[i] = sortKeys[i].idx;
            
            BlockAdjacencyGraph blockGraph;
            blockGraph.Build(clusterGraph, triangleIndices, numTris);
            GenerateStrips(strip, blockGraph, remap, blockLocalIndices, numTris);
            perfData.GenerateStrips += t.Tick();
        }

        // does everything fit?
        auto topologySizes = strip.PackedSize();
        if (topologySizes.indexSize > DGF::MAX_INDEX_BUFFER_BIT_SIZE)
            return false;

        size_t userDataByteSize = config.enableUserData ? DGF::MAX_USERDATA_SIZE : 0;
        size_t BIT_BUDGET = 8 * (DGF::BLOCK_SIZE - DGF::HEADER_SIZE - userDataByteSize);
        if (frontBufferSize + topologySizes.Total() > BIT_BUDGET)
            return false;

        //////////////////////////////////////////////////////////////////
        // If we reach this point, we will construct a block
        //////////////////////////////////////////////////////////////////

        // Construct offset vertices in strip order (by first use)
        DGF::OffsetVert relativeVerts[DGF::MAX_VERTS];
        {
            uint8_t firstUseIndex[DGF::MAX_VERTS];
            std::bitset<DGF::MAX_VERTS> vertUsed;
            size_t vertexCount = 0;
            for (size_t i = 0; i < strip.numIndices; i++)
            {
                uint8_t idx = strip.indexBuffer[i];
                if (!vertUsed.test(idx))
                {
                    // track first-use position for each vertex
                    size_t vertPosition = vertexCount++;
                    firstUseIndex[idx] = (uint8_t)vertPosition;
                    vertUsed.set(idx);

                    // copy each vertex on first use
                    int3 delta = blockLocalVerts[idx] - vertMin;
                    relativeVerts[vertPosition].xyz[0] = (uint16_t)delta.x;
                    relativeVerts[vertPosition].xyz[1] = (uint16_t)delta.y;
                    relativeVerts[vertPosition].xyz[2] = (uint16_t)delta.z;
                }

                strip.indexBuffer[i] = firstUseIndex[idx];
            }
        }

        // Construct triangle geomID arrays in strip order, and re-number the indices in the 'Remap' structures
        uint32_t triangleGeomIDs[DGF::MAX_TRIS];
        uint8_t triangleOpaqueFlags[DGF::MAX_TRIS];
        for (size_t i = 0; i < numTris; i++)
        {
            uint32_t triIndexInBlock   = remap[i].InputPrimID;
            uint32_t triIndexInCluster = sortKeys[triIndexInBlock].idx;
            triangleGeomIDs[i]     = clusterAttributes[triIndexInCluster].geomID;
            triangleOpaqueFlags[i] = clusterAttributes[triIndexInCluster].opaque;

            remap[i].InputPrimID = triIndexInCluster;
        }

        size_t xBits = std::max<size_t>(1, x_bits);
        size_t yBits = std::max<size_t>(1, y_bits);
        size_t zBits = std::max<size_t>(1, z_bits);

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


        uint8_t dgfBlock[DGF::BLOCK_SIZE];
        {
            DGF::Timer t;
            DGF::EncoderInput input;
            input.exponent = exponent + DGF::EXPONENT_BIAS; // TODO: Use biased throughout
            input.anchorX = vertMin.x;
            input.anchorY = vertMin.y;
            input.anchorZ = vertMin.z;
            input.indexBuffer = strip.indexBuffer;
            input.numIndices = (uint8_t)strip.numIndices;
            input.numTris = (uint8_t)strip.numTriangles;
            input.numVerts = (uint8_t)numVerts;
            input.primIDBase = primIDBase;
            input.triangleGeomIDs = triangleGeomIDs;
            input.triangleOpaqueFlags = triangleOpaqueFlags;
            input.triControl = strip.triControl;
            input.verts = relativeVerts;
            input.xBits = (uint8_t)xBits;
            input.yBits = (uint8_t)yBits;
            input.zBits = (uint8_t)zBits;
            input.numOMMDescriptors = (uint8_t) numOMMDescriptors;
            input.triangleOMMDescriptorIndices = triOMMIndices;
            input.userDataSize = (uint8_t)userDataByteSize;
            input.roundTripValidation = config.encoderRoundTripValidation;

            DGF::Encode(dgfBlock, input);
            perfData.EncodeBlocks += t.Tick();
        }

        cluster->dgfBlocks.insert(cluster->dgfBlocks.end(), &dgfBlock[0], &dgfBlock[DGF::BLOCK_SIZE]);
        cluster->outputOMMIndices.insert(cluster->outputOMMIndices.end(), &ommDescriptors[0], &ommDescriptors[numOMMDescriptors]);
        if( config.generateTriangleRemap || config.generateVertexTable )
            cluster->localRemap.insert(cluster->localRemap.end(), &remap[0], &remap[numTris]);

        return true;
    }

    static void PreprocessTris( 
        const Cluster& cluster, 
        SIMDBox* boxes, 
        SortKeys* sortKeys,
        DGFBaker::TriangleAttributes attributes[MAX_CLUSTER_TRIS] )
    {
        size_t numTris = cluster.Indices.size() / 3;
        DGF_ASSERT(numTris <= MAX_CLUSTER_TRIS);

        // compute triangle and cluster bounding boxes
        SIMDBox centroidBounds;
        for (size_t i = 0; i < numTris; i++)
        {
            attributes[i] = cluster.Attributes.empty() ? DGFBaker::TriangleAttributes() : cluster.Attributes[i];
            
            SIMDBox tmpBox;
            for (size_t j = 0; j < 3; j++)
            {
                uint8_t idx = cluster.Indices[3 * i + j];
                tmpBox.Expand(cluster.Vertices[idx]);
            }

            boxes[i] = tmpBox;
            centroidBounds.Expand(tmpBox.Centroid());
            sortKeys[i].idx = (uint8_t) i;
        }

        // compute sorted triangle lists on each axis
        for (size_t axis = 0; axis < 3; axis++)
        {
            float cmax = centroidBounds.Max(axis);
            float cmin = centroidBounds.Min(axis);
            float scale = (65535.0f / (cmax-cmin));
            
            // use quantized centroids for faster sorting
            uint32_t keys[MAX_CLUSTER_TRIS];
            for (size_t i = 0; i < numTris; i++)
            {
                float c = (boxes[i].Max(axis) + boxes[i].Min(axis)) * 0.5f;
                size_t cq = RoundToInt((c-cmin) * scale);
                keys[i] = (uint32_t) ( (cq << 16) + i );
            }

            RadixSort<uint32_t, MAX_CLUSTER_TRIS>(keys, numTris, 0xffff,
                [](const uint32_t key) {
                    return key >> 16;
                });
                
            for (size_t i = 0; i < numTris; i++)
                sortKeys[keys[i] & 0xff].keys[axis] = (uint8_t) i;
        }
    }


    static void SplitBlock(TempBlock& left, TempBlock& right, const SIMDBox* boxes, SortKeys* clusterKeys )
    {
        TempBlock input = left;

        size_t numTris  = input.triCount;
        size_t firstTri = input.firstTri;
        SortKeys* keys = &clusterKeys[firstTri];

        // SAH split
        // sweep the axis lists.. left-to-right and right-to-left
        float bestCost = std::numeric_limits<float>::infinity();
        size_t bestAxis = 0;
        size_t bestSplit = 0;

        float leftCosts[DGF::MAX_CLUSTER_TRIS];

        for (size_t axis = 0; axis < 3; axis++)
        {
            RadixSort<SortKeys, MAX_CLUSTER_TRIS>(keys, numTris, 0xff, [axis](SortKeys key) {return key.keys[axis]; });

            // left-sweep calculates left-side surface-area and stores in 'leftCosts'
            SIMDBox tmpBox = {};
            for (size_t i = 0; i < numTris; i++)
            {
                size_t idx = keys[i].idx;
                tmpBox.Expand(boxes[idx]);
                leftCosts[i] = tmpBox.HalfArea() * (i + 1);
            }

            // special case for degenerate geometry
            //  split in the middle to create a balanced tree
            if (tmpBox.HalfArea() == 0)
            {
                bestCost = 0;
                bestAxis = 0;
                bestSplit = numTris / 2;
                break;
            }

            // right-sweep calculates right-side surface area, calculates combined cost,
            //  and chooses best split decision
            tmpBox = boxes[keys[numTris-1].idx];
            for (size_t i = 1; i < numTris; i++)
            {
                size_t j = (numTris - i) - 1;

                float cost = leftCosts[j] + i * tmpBox.HalfArea();
                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestAxis = axis;
                    bestSplit = j;
                }

                size_t idx = keys[j].idx;
                tmpBox.Expand(boxes[idx]);
            }
        }

        RadixSort<SortKeys, MAX_CLUSTER_TRIS>(keys, numTris, 0xff, [bestAxis](SortKeys key) {return key.keys[bestAxis]; });

        size_t numLeft = 1 + bestSplit;
        size_t numRight = numTris - numLeft;

        left.exponent = input.exponent;
        left.firstTri = input.firstTri;
        left.triCount = (uint8_t)numLeft;

        right.exponent = input.exponent;
        right.firstTri = input.firstTri + (uint8_t)numLeft;
        right.triCount = (uint8_t)numRight;
    }


    void PackCluster(uint32_t primIDBase, DGF::Cluster& cluster, PerformanceData& perfData, int8_t exponent, const DGFBaker::Config& config)
    {
        size_t clusterSize = cluster.Indices.size() / 3;
        DGF_ASSERT(clusterSize <= MAX_CLUSTER_TRIS);

        SIMDBox triBoxes[MAX_CLUSTER_TRIS];
        SortKeys sortKeys[MAX_CLUSTER_TRIS];
        DGFBaker::TriangleAttributes clusterAttributes [DGF::MAX_CLUSTER_TRIS] ;
        PreprocessTris(cluster, triBoxes, sortKeys, clusterAttributes);

        ClusterAdjacencyGraph clusterGraph;
        {
            DGF::Timer t;
            clusterGraph.Build(cluster.Indices.data(), clusterSize);
            perfData.BuildClusterGraph += t.Tick();
        }

        std::vector<TempBlock> tmpBlocks;
        TempBlock tb;
        tb.exponent = exponent;
        tb.firstTri = 0;
        tb.triCount = (uint8_t)clusterSize;
        tmpBlocks.push_back(tb);

        std::vector<int3> verts;
        {
            DGF::Timer t;
            verts.resize(cluster.Vertices.size());
            DGF::QuantizeVerts(verts, cluster.Vertices, exponent);
            perfData.QuantizeVertices += t.Tick();
        }

        size_t i = 0;
        uint32_t numEncodedTris = 0;
        while (i < tmpBlocks.size())
        {
            int8_t exponent = tmpBlocks[i].exponent;
            size_t numTris = tmpBlocks[i].triCount;
            SortKeys* keys = &sortKeys[tmpBlocks[i].firstTri];

            if (BuildBlock(primIDBase + numEncodedTris, exponent, numTris, keys, verts, clusterAttributes, config, clusterGraph, &cluster, perfData))
            {
                numEncodedTris += (uint32_t) numTris;
                i++;
                continue;
            }
            else
            {
                DGF::Timer t;
                tmpBlocks.emplace_back();
                SplitBlock(tmpBlocks[i], tmpBlocks.back(), triBoxes, sortKeys );
                perfData.SplitBlock += t.Tick();
            }
        }
    }


    void PackBlocksSAH(std::vector<DGF::Cluster>& clusters, PerformanceData& perfData, int8_t exponent, const DGFBaker::Config& config)
    {
        DGF::Timer t;

        uint32_t primIDBase = 0;
        for (DGF::Cluster& cluster : clusters)
        {
            size_t numTris = cluster.Indices.size() / 3;

            PackCluster(primIDBase, cluster, perfData, exponent, config);
            perfData.NumBlocks += cluster.dgfBlocks.size()/DGF::BLOCK_SIZE;

            primIDBase += (uint32_t) numTris;
        }

        perfData.PackTriangles += t.Tick();
    }



}