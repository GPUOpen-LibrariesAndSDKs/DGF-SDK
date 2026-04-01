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

#include "DGFS.h"
#include <DGF.h>

#include "Utils.h"
#include "Threading.h"

namespace DGFS
{
    constexpr uint32_t DEFAULT_GEOMID = 0x1; // opaque=1 and GeomID=0

    using namespace DGFBaker;

    struct IndexPaletteState
    {
        size_t numPaletteEntries = 0;
        size_t numBlockIndices = 0;
        size_t firstSize = 0;
        size_t deltaSize = 0;
        uint32_t clusterPalette[MAX_CLUSTER_FACES];
        uint8_t blockEntryIndices[MAX_CLUSTER_BLOCKS]; // index of each block's elements in cluster palette
    
        bool sharedLSBs = false; // LSBs are all the same and omitted
        bool sharedLSBValue = false; // value of common LSB
    };


    struct EncoderState
    {
        const uint8_t* pInputBlocks = nullptr;
        const int32_t* pInputOMMIndices = nullptr;

        struct Vert
        {
            int32_t pos[3];
            uint16_t refCount; // number of references to this vertex
            uint16_t index;    // position in the unshared or shared vertex array
        };

        uint8_t exponent = DGF::EXPONENT_MAX;
        size_t numVerts = 0;
        size_t numSharedVerts = 0;  // number of verts used more than once
        size_t numUniqueVerts = 0;  // number of verts used exactly once

        int3 vertexPositions[MAX_CLUSTER_VERTICES];
        uint8_t vertexRefCounts[MAX_CLUSTER_VERTICES];
        uint8_t vertexSortedIndices[MAX_CLUSTER_VERTICES]; // index of each vertex in the final VB

        // for each block vertex, which of the cluster verts is it
        uint8_t blockVertToClusterVert[3 * MAX_CLUSTER_FACES];
        std::bitset<3 * MAX_CLUSTER_FACES> blockVertIsShared; // for each block vertex, is it shared by multiple blocks
        size_t numBlockVerts = 0; // sum of vertex counts across blocks

        size_t numBlocks = 0;
        size_t xBytes = 0;
        size_t yBytes = 0;
        size_t zBytes = 0;

        size_t blockDataByteSize = 0;

        // references to duplicated vertices.  These are indexed from the back of the vertex array
        size_t numBlockSharedVertIndices = 0;
        uint8_t blockSharedVertIndices[3 * MAX_CLUSTER_FACES];

        uint32_t primIDBase = 0;

        IndexPaletteState geomPalette;
        bool needGeomIDPalette = false;

        IndexPaletteState ommPalette;
        bool needOMMPalette = false;

        uint8_t blockSharedVertIndexSizes[MAX_CLUSTER_BLOCKS];

        std::bitset<MAX_CLUSTER_BLOCKS> blockStoresPrimIDBase;
        uint8_t blockUserDataEncodedSize[MAX_CLUSTER_BLOCKS];

        uint3 offsetWidths;
    };

    static void IncreaseClusterPrecision(EncoderState& state, uint8_t blockExponent)
    {
        int shift = state.exponent - blockExponent;
        for (size_t v = 0; v < state.numVerts; v++)
        {
            state.vertexPositions[v] = state.vertexPositions[v] << shift;
        }
        state.exponent = blockExponent;
    }

    static void AddVerticesToEncoderState(EncoderState& state, const uint8_t* pBlock)
    {
        DGF::MetaData meta;
        DGF::DecodeMetaData(&meta, pBlock);

        // We need to store all vertices at the highest precision
        //    if we find a higher precision DGF block, adjust previous verts to match this block
        //
        //  NOTE:  Data loss can occur here if a non-standard encoder did something unusual:
        //          E.g.  a high precision cluster near the origin and low precision very far away                
        //         If that happens we'll need some alternate mode that stores per-block exponents
        //          the DGFBaker encoder does not do this.
        //
        if (meta.exponent < state.exponent)
            IncreaseClusterPrecision(state, meta.exponent);

        // if a block's precision is lower than current cluster precision, promote vertices to higher precision
        uint32_t vertexShift = (meta.exponent > state.exponent) ? (meta.exponent - state.exponent) : 0;

        // extract block vertices and add to the cluster
        DGF::OffsetVert ov[DGF::MAX_VERTS];
        DGF::DecodeOffsetVerts(meta.numVerts, ov, pBlock);
        for (size_t v = 0; v < meta.numVerts; v++)
        {
            // expand the vertex onto the 24b DGF grid
            int3 vert(
                (meta.anchorX + ov[v].xyz[0]) << vertexShift,
                (meta.anchorY + ov[v].xyz[1]) << vertexShift,
                (meta.anchorZ + ov[v].xyz[2]) << vertexShift
            );

            // insert vert into vertex set
            size_t idx = state.numVerts;
            for (size_t i = 0; i < state.numVerts; i++)
            {
                // search for an existing copy
                if (state.vertexPositions[i] == vert)
                {
                    idx = i;
                    break;
                }
            }

            if (idx == state.numVerts)
            {
                // brand new vertex
                DGF_ASSERT(state.numVerts < MAX_CLUSTER_VERTICES);
                state.vertexRefCounts[idx] = 1;
                state.vertexPositions[idx] = vert;
                state.numVerts++;
            }
            else
            {
                // repeated vertex
                state.vertexRefCounts[idx]++;
            }

            // note each block vert's position in unique vertex set
            state.blockVertToClusterVert[state.numBlockVerts++] = (uint8_t)idx;
        }
    }

    struct AABB3i
    {
        int3 Min;
        int3 Max;
    };

    static AABB3i ComputeVertexAABB(int3* verts, size_t numVerts)
    {
        int3 bbMin(S24_MAX, S24_MAX, S24_MAX);
        int3 bbMax(S24_MIN, S24_MIN, S24_MIN);
        for (size_t i = 0; i < numVerts; i++)
        {
            bbMin = min(bbMin, verts[i]);
            bbMax = max(bbMax, verts[i]);
        }
        return { bbMin, bbMax };
    }
    static uint3 ComputeVertexOffsetWidths(AABB3i box)
    {
        uint3 offsetWidths;
        offsetWidths.x = (uint32_t) BitsNeededUnsigned(box.Max.x - box.Min.x);
        offsetWidths.y = (uint32_t) BitsNeededUnsigned(box.Max.y - box.Min.y);
        offsetWidths.z = (uint32_t) BitsNeededUnsigned(box.Max.z - box.Min.z);
        offsetWidths.x = std::max(offsetWidths.x, 1u);
        offsetWidths.y = std::max(offsetWidths.y, 1u);
        offsetWidths.z = std::max(offsetWidths.z, 1u);
        return offsetWidths;
    }

    static void PostProcessVerts(EncoderState& state)
    {
        int3 tmpVerts[MAX_CLUSTER_VERTICES];
        for (size_t v = 0; v < state.numVerts; v++)
        {
            // classify each vertex as 'shared' or 'unshared'
            // compute each vert's index in its respective set, and count the number of each
            size_t index;
            if (state.vertexRefCounts[v] > 1)
            {
                // index shared vertices from the back
                index = state.numSharedVerts++;
                index = (state.numVerts - 1) - index;
            }
            else
            {

                index = state.numUniqueVerts++;
            }

            state.vertexSortedIndices[v] = (uint8_t)index;

            // as we do this, create a re-ordered array of vertices.  Unique vertices go first in block order
            //  shared vertices go last
            tmpVerts[index] = state.vertexPositions[v];
        }

        // compute vertex offset widths
        AABB3i vertexBox   = ComputeVertexAABB(tmpVerts, state.numVerts);
        state.offsetWidths = ComputeVertexOffsetWidths(vertexBox);
        
        // first vertex is stored as a zig-zag encoded anchor
        for (size_t k = 0; k < 3; k++)
            state.vertexPositions[0][k] = ZigZagEncode(tmpVerts[0][k]);

        // vertices 1...N are stored as zig-zag encoded deltas from the anchor
        uint32_t maxDelta[3] = { 0 };
        for (size_t v = 1; v < state.numVerts; v++)
        {
            int3 vertex = tmpVerts[v];
            int3 delta = vertex - tmpVerts[v - 1];

            for (size_t k = 0; k < 3; k++) {
                uint32_t zz = ZigZagEncode(delta[k]);
                maxDelta[k] = std::max(maxDelta[k], zz);
                state.vertexPositions[v][k] = (int32_t)zz;
            }
        }

        state.xBytes = BytesNeededUnsigned(maxDelta[0]);
        state.yBytes = BytesNeededUnsigned(maxDelta[1]);
        state.zBytes = BytesNeededUnsigned(maxDelta[2]);
        DGF_ASSERT(state.xBytes <= 3);
        DGF_ASSERT(state.yBytes <= 3);
        DGF_ASSERT(state.zBytes <= 3);
    }

    static size_t ComputePrimIDBaseByteSize(uint32_t idBase)
    {
        size_t idBaseBits = BitsNeededUnsigned(idBase);
        DGF_ASSERT(idBaseBits <= DGF::MAX_PRIMID_BITS);
        static_assert(DGF::MAX_PRIMID_BITS <= 30);
        size_t idBaseBytes = RoundUpPow2(idBaseBits+2, 8) / 8;        
        return idBaseBytes;
    }

    static void AnalyzePrimIDBaseFields(EncoderState& state)
    {
        size_t totalTris = 0;
        for (size_t b = 0; b < state.numBlocks; b++)
        {
            const uint8_t* pBlock = state.pInputBlocks + (b * DGF::BLOCK_SIZE);

            uint32_t idBase = (uint32_t) DGF::DecodePrimIDBase(pBlock);
            size_t numTris  = DGF::DecodeTriangleCount(pBlock);
            
            // store ID base field if and only if it differs from a running predictor
            //  We predict the next primIDBase value by adding the triangle count to the previous one
            bool mispredicted = (idBase != totalTris);
            state.blockStoresPrimIDBase[b] = mispredicted;

            // account for size of stored ID base.  We use a varint scheme 
            //     which stores 2 bits to indicate 1,2,3, or 4 bytes.  
            // This is always non-negative compression, because there are spare bits in the IDBase field
            static_assert(DGF::MAX_PRIMID_BITS <= 30);
            if (mispredicted)
            {
                state.blockDataByteSize += ComputePrimIDBaseByteSize(idBase);

                // each time we mispredict, reset the incrementing counter
                totalTris = idBase;
            }

            totalTris += numTris;
        }
    }

    static size_t WritePrimIDBase( ByteSpan span, size_t bitPos, uint32_t base )
    {
        DGF_ASSERT((bitPos % 8) == 0);
        size_t bytePos = bitPos / 8;
        size_t byteStart = bytePos;

        size_t bitSize  = BitsNeededUnsigned(base);
        DGF_ASSERT(bitSize < 30);
        size_t byteSize = ComputePrimIDBaseByteSize(base);

        uint32_t tmpBase = (uint32_t)((base << 2) | (byteSize - 1));
        do
        {
            span[bytePos++] = (uint8_t)tmpBase;
            tmpBase = tmpBase >> 8;
        } while (tmpBase);
       

        DGF_ASSERT(bytePos - byteStart == byteSize);
        return 8 * bytePos;
    }

    static ConstByteSpan ReadPrimIDBase(uint32_t* dataOut, ConstByteSpan span)
    {
        size_t byteSize = (span[0] & 0x3) + 1;
        if (dataOut)
        {
            uint32_t base = 0;
            for (size_t bytePos = 0; bytePos < byteSize; bytePos++)
            {
                uint32_t byte = span[bytePos];
                base |= (byte << (8 * bytePos));
            }

            *dataOut = base >> 2;
        }
        return IncrementByteSpan(span, byteSize);
    }

    static void AddBlockToIndexPalette( 
        IndexPaletteState& state, 
        uint32_t blockPalette[], 
        uint8_t blockIndices[],
        size_t paletteSize,
        size_t numTris)
    {
        uint8_t paletteIndexInCluster[DGF::MAX_GEOM_IDS];
        for (size_t i = 0; i < paletteSize; i++)
        {
            uint32_t id = blockPalette[i];
            size_t entryIndex = state.numPaletteEntries;
            for (size_t e = 0; e < state.numPaletteEntries; e++)
            {
                if (state.clusterPalette[e] == id)
                {
                    entryIndex = e;
                    break;
                }
            }
            DGF_ASSERT(entryIndex < MAX_CLUSTER_FACES);

            if (entryIndex == state.numPaletteEntries)
            {
                state.clusterPalette[entryIndex] = id;;
                state.numPaletteEntries++;
            }

            paletteIndexInCluster[i] = uint_truncate<uint8_t>(entryIndex);
        }

        size_t blockTriCounter = state.numBlockIndices;
        for (size_t t = 0; t < numTris; t++)
            state.blockEntryIndices[blockTriCounter + t] = paletteIndexInCluster[blockIndices[t]];
        state.numBlockIndices += numTris;
    }

    static void PostProcessIndexPalette( IndexPaletteState& ps )
    {
        size_t numEntries = ps.numPaletteEntries;
        DGF_ASSERT(numEntries > 0);
        
        // sort palette entries
        uint32_t sortedPalette[MAX_CLUSTER_FACES];
        for (size_t i = 0; i < numEntries; i++)
            sortedPalette[i] = ps.clusterPalette[i];

        std::sort(&sortedPalette[0], &sortedPalette[numEntries]);

        uint8_t sortedIndex[MAX_CLUSTER_FACES];
        for (size_t i = 0; i < ps.numPaletteEntries; i++)
        {
            // build table mapping original entries to sorted entries
            auto location = std::lower_bound(&sortedPalette[0], &sortedPalette[numEntries], ps.clusterPalette[i]);
            sortedIndex[i] = (uint8_t)(location - &sortedPalette[0]);      
        }

        // fix the indices
        for (size_t i = 0; i < ps.numBlockIndices; i++)
            ps.blockEntryIndices[i] = sortedIndex[ps.blockEntryIndices[i]];

        // check for the common case where LSBs all match
        uint32_t diffMask = 0;
        uint32_t entry0 = ps.clusterPalette[0];
        for (size_t i = 1; i < ps.numPaletteEntries; i++)
            diffMask |= (entry0 ^ ps.clusterPalette[i]);

        if ((diffMask & 1) == 1)
        {
            // if all LSBs are the same, shift them out
            ps.sharedLSBs = true;
            ps.sharedLSBValue = (entry0 & 1);
            for (size_t i = 0; i < ps.numPaletteEntries; i++)
                sortedPalette[i] >>= 1;                
        }

        ps.firstSize = BytesNeededUnsigned(sortedPalette[0]);
        ps.clusterPalette[0] = sortedPalette[0];

        // turn entries into deltas
        ps.deltaSize = 1;
        for (size_t i = 1; i < ps.numPaletteEntries; i++)
        {
            uint32_t delta = sortedPalette[i] - sortedPalette[i - 1];
            ps.deltaSize = std::max(ps.deltaSize, BytesNeededUnsigned(delta));
            ps.clusterPalette[i] = delta;
        }

    }

    static void DeDuplicateGeomIDs(EncoderState& state)
    {
        IndexPaletteState& ps = state.geomPalette;

        size_t blockTriCounter = 0;
        for (size_t b = 0; b < state.numBlocks; b++)
        {
            const uint8_t* pBlock = state.pInputBlocks + (b * DGF::BLOCK_SIZE);

            size_t numTris = DGF::DecodeTriangleCount(pBlock);

            uint32_t geomPalette[DGF::MAX_GEOM_IDS];
            uint8_t geomIDIndices[DGF::MAX_TRIS];
            size_t paletteSize = DGF::DecodeGeomIDPalette(geomPalette, geomIDIndices, pBlock);

            AddBlockToIndexPalette(ps, geomPalette, geomIDIndices, paletteSize, numTris);
        }

        PostProcessIndexPalette(ps);

        // determine if we need a palette in the stream
        state.needGeomIDPalette       = ps.numPaletteEntries > 1 || (ps.numPaletteEntries == 1 && ps.clusterPalette[0] != DEFAULT_GEOMID);
    }

    static void DeDuplicateOMMIDs(EncoderState& state)
    {
        if (state.pInputOMMIndices == nullptr)
        {
            state.needOMMPalette = false;
            return;
        }

        size_t blockOMMOffset = 0;
        for (size_t b = 0; b < state.numBlocks; b++)
        {
            const uint8_t* pBlock = state.pInputBlocks + (b * DGF::BLOCK_SIZE);

            size_t numTris = DGF::DecodeTriangleCount(pBlock);
            size_t numOMMDescriptors = DGF::DecodeOMMDescriptorCount(pBlock);

            uint8_t triIndexInLocalPalette[DGF::MAX_TRIS];
            DGF::DecodeTriangleOMMIndices(triIndexInLocalPalette, pBlock);

            uint32_t descriptors[DGF::MAX_OMM_DESCRIPTORS];
            for (size_t d = 0; d < numOMMDescriptors; d++)
            {
                int32_t ommIndex = state.pInputOMMIndices[blockOMMOffset + d];
                descriptors[d] = ZigZagEncode(ommIndex);
            }

            AddBlockToIndexPalette(state.ommPalette, descriptors, triIndexInLocalPalette, numOMMDescriptors, numTris);

            blockOMMOffset += numOMMDescriptors;
        }

        PostProcessIndexPalette(state.ommPalette);

        // don't allocate an OMM palette if the app passed us OMM indices but 
        //  none of the cluster's blocks have any
        state.needOMMPalette = state.ommPalette.numPaletteEntries > 0;
    }

    static void AnalyzeBlockTopology(EncoderState& state)
    {
        size_t blockVertBase = 0;
        for (size_t block = 0; block < state.numBlocks; block++)
        {
            const uint8_t* pBlock = state.pInputBlocks + block * DGF::BLOCK_SIZE;
            DGF::MetaData meta;
            DGF::DecodeMetaData(&meta, pBlock);

            DGF::TriControlValues ctrl[DGF::MAX_TRIS];
            uint8_t stripIndices[DGF::MAX_INDICES];
            DGF::DecodeTopology(ctrl, stripIndices, pBlock);

            // count indices
            size_t numIndices = 0;
            for (size_t i = 1; i < meta.numTris; i++)
                numIndices += (ctrl[i] == DGF::TC_RESTART ? 3 : 1);

            // scan strip and count 'is-first' markers, re-use buffer values, and shared-vert indices
            std::bitset<DGF::MAX_VERTS> vertexUsed;

            size_t reuseBufferBitSize = 0;
            size_t maxSharedVertIndex = 0;
            size_t numSharedVerts = 0;
            for (size_t i = 0; i < numIndices + 3; i++)
            {
                size_t localIndex = stripIndices[i];
                bool isFirst = !vertexUsed.test(localIndex);
                if (!isFirst)
                {
                    // re-used vertex
                    reuseBufferBitSize += meta.reuseBufferIndexSize;
                }
                else
                {
                    // new vertex
                    size_t blockVertIndex = blockVertBase + localIndex;
                    size_t indexInCluster = state.blockVertToClusterVert[blockVertIndex];
                    bool isSharedVert = state.vertexRefCounts[indexInCluster] > 1;
                    if (isSharedVert)
                    {
                        size_t idx = state.vertexSortedIndices[indexInCluster];
                        idx = (state.numVerts - 1) - idx;
                        state.blockSharedVertIndices[state.numBlockSharedVertIndices++] = (uint8_t)idx;
                        state.blockVertIsShared.set(blockVertIndex);
                        maxSharedVertIndex = std::max(maxSharedVertIndex, idx);
                        numSharedVerts++;
                    }

                    vertexUsed.set(localIndex);
                }
            }

            size_t sharedVertIndexSize = BitsNeededUnsigned(maxSharedVertIndex);
            state.blockSharedVertIndexSizes[block] = uint_truncate<uint8_t>(sharedVertIndexSize);

            size_t vertexTableBitSize = meta.numVerts + sharedVertIndexSize * numSharedVerts;
            size_t vertexTableSize = RoundUpPow2(vertexTableBitSize, 8) / 8;

            size_t topologyBitSize = numIndices + 2 * (meta.numTris - 1) + reuseBufferBitSize;
            size_t topologySize = RoundUpPow2(topologyBitSize, 8) / 8;
            state.blockDataByteSize += vertexTableSize + topologySize;

            blockVertBase += meta.numVerts;
        }
    }

    static void ComputeBlockIndexPaletteSizes(EncoderState& state, IndexPaletteState& palette )
    {
        size_t indexSize = BitsNeededUnsigned(palette.numPaletteEntries - 1);
        size_t blockTriBase = 0;
        for (size_t block = 0; block < state.numBlocks; block++)
        {
            size_t numTris = DGF::DecodeTriangleCount(state.pInputBlocks + block * DGF::BLOCK_SIZE);

            // count number of indices used by this block
            std::bitset<MAX_CLUSTER_FACES> idUsed;
            size_t numUsedIDs = 0;
            for (size_t t = 0; t < numTris; t++)
            {
                size_t entryIndex = palette.blockEntryIndices[blockTriBase + t];
                if (!idUsed.test(entryIndex))
                {
                    idUsed.set(entryIndex);
                    numUsedIDs++;
                }
            }

            if (indexSize > 0)
            {
                // compute size of this block's squished palette
                size_t blockPaletteIndexSize = BitsNeededUnsigned(numUsedIDs - 1);
                size_t blockPaletteBitSize = 8 + // number of palette entries
                    (numUsedIDs * indexSize) +  // per-ID indices into cluster-level palette
                    (numTris * blockPaletteIndexSize); // per-tri palette entry indices

                // add the palette to the block data size
                state.blockDataByteSize += BitSizeToByteSize(blockPaletteBitSize);
            }

            blockTriBase += numTris;
        }
    }


    struct Header
    {
        uint32_t numVerts : 8; // 1-256

        uint32_t xBytes : 2; // 0-3 bytes
        uint32_t yBytes : 2; // 0-3 bytes
        uint32_t zBytes : 2; // 0-3 bytes
        uint32_t haveGeomPalette : 1; // if set, a 'geom palette' is present after the block descriptors
        uint32_t haveOMMPalette : 1; // if set, an 'omm descriptor palette' is present

        uint32_t exponent : 8;

        uint32_t vertexOffsetSize : 6; // total size of vertex offsets (bits), for COMPRESSED1 decoding
        uint32_t unused           : 2;
    };


    struct BlockDescriptor
    {
        uint16_t numTris             : 6; // 1-64
        uint16_t indexSize           : 2; // reuse buffer index size: 3,4,5,6 bits
        uint16_t sharedVertIndexSize : 4; // 0-8 bits
        uint16_t havePrimIDBase      : 1; // if set, an explicit primID base field is stored for this block
        uint16_t encodedUserDataSize : 2; // size of stored user-data field. Encodes {0,1,2,4} bytes.  
        uint16_t last                : 1;
    };

    struct IndexPaletteHeader
    {
        uint16_t numIDs : 9;    // 1-256
        uint16_t firstSize : 3; // size of first element (bytes) (0-4)
        uint16_t deltaSize : 2; // size of deltas, (bytes) (1-4)
        uint16_t sharedLSBs     : 1;      // if set, all LSBs are the same and not encoded
        uint16_t sharedLSBValue : 1;      // value of common LSBs
    };

    static_assert(sizeof(IndexPaletteHeader) == 2);
    static_assert(sizeof(BlockDescriptor) == 2);
    static_assert(sizeof(Header) == 4);

    static uint32_t ReadMultiByteDWORD(ConstByteSpan span, size_t size)
    {
        uint32_t data = 0;
        for (size_t i = 0; i < size; i++)
            data |= (span[i] << (8 * i));
        return data;
    }

    static void WriteMultiByteDWORD(ByteSpan span, size_t bytePos, uint32_t data, size_t size)
    {
        for (size_t i = 0; i < size; i++)
        {
            size_t byte = (data >> (8 * i)) & 0xFF;
            span[bytePos + i] = (uint8_t)byte;
        }
    }

    static size_t ComputeClusterIndexPaletteSize(const IndexPaletteState& state)
    {
        size_t numIDs = state.numPaletteEntries;
        DGF_ASSERT(numIDs > 0);
        return (numIDs - 1) * state.deltaSize + state.firstSize + sizeof(IndexPaletteHeader);
    }

    static size_t ComputeClusterIndexPaletteSize(const IndexPaletteHeader* header)
    {
        size_t numIDs = header->numIDs;
        DGF_ASSERT(numIDs > 0);
        size_t firstSize = header->firstSize ;
        size_t deltaSize = header->deltaSize + 1;
        return (numIDs - 1) * firstSize + deltaSize + sizeof(IndexPaletteHeader);
    }

    static size_t ComputeBlockDescriptorSize(size_t numBlocks)
    {
        return sizeof(BlockDescriptor) * numBlocks;
    }

    static size_t ComputeVertexBufferSize(size_t n, size_t x, size_t y, size_t z, size_t anchorSize)
    {
        return (n - 1) * (x + y + z) + anchorSize;
    }

    static size_t ComputeAnchorSize(const EncoderState& state)
    {
        // the first vertex is the zig-zag encoded anchor
        // figure out how many bytes we need to represent it
        // add 1 for a header byte that encodes its byte size
        size_t xSize = BytesNeededUnsigned(state.vertexPositions[0].x);
        size_t ySize = BytesNeededUnsigned(state.vertexPositions[0].y);
        size_t zSize = BytesNeededUnsigned(state.vertexPositions[0].z);
        return 1 + xSize + ySize + zSize;
    }

    static size_t ComputeStreamSize(const EncoderState& state)
    {
        size_t headerSize = sizeof(Header);
        size_t blockDescriptorSize = ComputeBlockDescriptorSize(state.numBlocks);
        size_t anchorSize = ComputeAnchorSize(state);
        size_t vertSize = ComputeVertexBufferSize(state.numVerts, state.xBytes, state.yBytes, state.zBytes, anchorSize);
        size_t geomSize = 0;
        if (state.needGeomIDPalette)
            geomSize = ComputeClusterIndexPaletteSize(state.geomPalette);

        size_t ommSize = 0;
        if (state.needOMMPalette)
            ommSize = ComputeClusterIndexPaletteSize(state.ommPalette);
        
        return headerSize + blockDescriptorSize + vertSize + geomSize + state.blockDataByteSize + ommSize;
    }

    static void WriteHeader(ByteSpan outputStream, const EncoderState& state)
    {
        Header* header = ByteSpanCast<Header>(outputStream, 0);

        uint32_t ax = (uint32_t)state.vertexPositions[0][0];
        uint32_t ay = (uint32_t)state.vertexPositions[0][1];
        uint32_t az = (uint32_t)state.vertexPositions[0][2];
        DGF_ASSERT(BitsNeededUnsigned(ax) <= 24);
        DGF_ASSERT(BitsNeededUnsigned(ay) <= 24);
        DGF_ASSERT(BitsNeededUnsigned(az) <= 24);

        header->exponent = state.exponent;
        header->numVerts = state.numVerts - 1;
        header->xBytes = state.xBytes;
        header->yBytes = state.yBytes;
        header->zBytes = state.zBytes;
        header->haveGeomPalette  = state.needGeomIDPalette;
        header->haveOMMPalette   = state.needOMMPalette;
        header->vertexOffsetSize = state.offsetWidths.x + state.offsetWidths.y + state.offsetWidths.z;
        
        DGF_ASSERT(header->vertexOffsetSize <= 48);
    }

    static size_t WriteBlockDescriptors(ByteSpan outputStream, const EncoderState& state, size_t bitPos)
    {
        DGF_ASSERT((bitPos % 8) == 0);

        size_t byteOffs = bitPos / 8;
        BlockDescriptor* blockDesc = ByteSpanCast<BlockDescriptor>(outputStream, byteOffs, state.numBlocks);

        for (size_t i = 0; i < state.numBlocks; i++)
        {
            DGF::MetaData meta;
            DGF::DecodeMetaData(&meta, state.pInputBlocks + DGF::BLOCK_SIZE * i);
            blockDesc[i].numTris = meta.numTris - 1;
            blockDesc[i].last = (i == state.numBlocks - 1) ? 1 : 0;
            blockDesc[i].indexSize = meta.reuseBufferIndexSize - 3;
            blockDesc[i].sharedVertIndexSize = state.blockSharedVertIndexSizes[i];
            blockDesc[i].havePrimIDBase      = state.blockStoresPrimIDBase[i];
            blockDesc[i].encodedUserDataSize = state.blockUserDataEncodedSize[i];

            DGF_ASSERT((blockDesc[i].numTris + 1) > 0);
        }

        return 8 * (byteOffs + ComputeBlockDescriptorSize(state.numBlocks));
    }

    static size_t WriteClusterIndexPalette(ByteSpan outputStream, const IndexPaletteState& state, size_t bitPos)
    {
        DGF_ASSERT((bitPos % 8) == 0);
        size_t bytePos = bitPos / 8;

        DGF_ASSERT(state.deltaSize > 0 && state.deltaSize <= 4);
        DGF_ASSERT(state.firstSize >= 0 && state.firstSize <= 4);

        IndexPaletteHeader* header = ByteSpanCast<IndexPaletteHeader>(outputStream, bytePos);
        header->deltaSize = state.deltaSize-1;
        header->firstSize = state.firstSize;
        header->numIDs = state.numPaletteEntries;
        header->sharedLSBs = state.sharedLSBs;
        header->sharedLSBValue = state.sharedLSBValue;
        
        bytePos += sizeof(IndexPaletteHeader);
        
        WriteMultiByteDWORD(outputStream, bytePos, state.clusterPalette[0], state.firstSize);
        bytePos += state.firstSize;

        for (size_t i = 1; i < state.numPaletteEntries; i++) 
        {
            WriteMultiByteDWORD(outputStream, bytePos, state.clusterPalette[i], state.deltaSize);
            bytePos += state.deltaSize;
        }
        
        return 8 * bytePos;
    }

    static size_t WriteAnchorVertex(ByteSpan outputStream, const EncoderState& state, size_t bytePos)
    {
        uint32_t x = state.vertexPositions[0].x;
        uint32_t y = state.vertexPositions[0].y;
        uint32_t z = state.vertexPositions[0].z;
        size_t xSize = BytesNeededUnsigned(x);
        size_t ySize = BytesNeededUnsigned(y);
        size_t zSize = BytesNeededUnsigned(z);
        
        outputStream[bytePos++] = uint_truncate<uint8_t>((xSize | (ySize << 2) | (zSize << 4)));
        
        WriteMultiByteDWORD(outputStream, bytePos, x, xSize); 
        bytePos += xSize;
        
        WriteMultiByteDWORD(outputStream, bytePos, y, ySize);
        bytePos += ySize;

        WriteMultiByteDWORD(outputStream, bytePos, z, zSize);
        bytePos += zSize;

        return bytePos;
    }

    static size_t WriteVertexBuffer(ByteSpan outputStream, const EncoderState& state, size_t bitPos)
    {
        size_t channelBytes[3] = {
            state.xBytes,
            state.yBytes,
            state.zBytes
        };

        uint8_t vertexBytes[MAX_CLUSTER_VERTICES][9];
        for (size_t i = 1; i < state.numVerts; i++)
        {
            uint32_t x = state.vertexPositions[i][0];
            uint32_t y = state.vertexPositions[i][1];
            uint32_t z = state.vertexPositions[i][2];

            vertexBytes[i][0] = (x >> 16);
            vertexBytes[i][1] = (y >> 16);
            vertexBytes[i][2] = (z >> 16);
            vertexBytes[i][3] = (x >> 8);
            vertexBytes[i][4] = (y >> 8);
            vertexBytes[i][5] = (z >> 8);
            vertexBytes[i][6] = (x & 255);
            vertexBytes[i][7] = (y & 255);
            vertexBytes[i][8] = (z & 255);
        }

        DGF_ASSERT(bitPos % 8 == 0);
        size_t byteOffs = bitPos / 8;

        byteOffs = WriteAnchorVertex(outputStream, state, byteOffs);

        // high bytes for x,y,z
        for (size_t k = 0; k < 3; k++)
        {
            if (channelBytes[k] > 2)
            {
                for (size_t i = 1; i < state.numVerts; i++)
                {
                    outputStream[byteOffs++] = vertexBytes[i][k];
                }
            }
        }


        // middle bytes for x,y,z
        for (size_t k = 0; k < 3; k++)
        {
            if (channelBytes[k] > 1)
            {
                for (size_t i = 1; i < state.numVerts; i++)
                {
                    outputStream[byteOffs++] = vertexBytes[i][k + 3];
                }
            }
        }

        // low bytes for x,y,z
        for (size_t k = 0; k < 3; k++)
        {
            if (channelBytes[k] > 0)
            {
                for (size_t i = 1; i < state.numVerts; i++)
                {
                    outputStream[byteOffs++] = vertexBytes[i][k + 6];
                }
            }
        }

        size_t bitPosStart = bitPos;
        size_t bitPosEnd = 8 * byteOffs;

        size_t streamSize = (bitPosEnd - bitPosStart) / 8;
        size_t anchorSize = ComputeAnchorSize(state);
        size_t estimate   = ComputeVertexBufferSize(state.numVerts, channelBytes[0], channelBytes[1], channelBytes[2], anchorSize );
        DGF_ASSERT(streamSize == estimate);
        return bitPosEnd;
    }

    struct BlockTopology
    {
        DGF::TriControlValues ctrl[DGF::MAX_TRIS];
        std::bitset<DGF::MAX_INDICES> isFirst;
        uint8_t reuseBuffer[DGF::MAX_INDICES];
        size_t numTris;
        size_t numIndices;
        size_t numReuseIndices;
        size_t reuseIndexSize;
    };

    static void GetBlockTopology(BlockTopology& topo, const uint8_t* block)
    {
        topo.numIndices = 0;
        topo.numTris = DGF::DecodeTriangleCount(block);
        topo.reuseIndexSize = 3;
        topo.numReuseIndices = 0;

        uint8_t tmpIndices[DGF::MAX_INDICES];
        DGF::DecodeTopology(topo.ctrl, tmpIndices, block);

        std::bitset<DGF::MAX_VERTS> vertSeen;
        for (size_t i = 0; i < topo.numTris; i++)
        {
            size_t ni = topo.ctrl[i] == DGF::TC_RESTART ? 3 : 1;
            for (size_t j = 0; j < ni; j++)
            {
                size_t indexPos = topo.numIndices++;
                uint8_t idx = tmpIndices[indexPos];
                if (vertSeen[idx])
                {
                    topo.reuseBuffer[topo.numReuseIndices++] = idx;
                    topo.reuseIndexSize = std::max(topo.reuseIndexSize, BitsNeededUnsigned(idx));
                    topo.isFirst[indexPos] = false;
                }
                else
                {
                    vertSeen[idx] = true;
                    topo.isFirst[indexPos] = true;
                }
            }
        }
    }

    static size_t WriteControlBits(
        ByteSpan outputStream,
        size_t bitPos,
        const BlockTopology& topo)
    {
        size_t bitStart = bitPos;

        // write block strip control
        for (size_t t = 1; t < topo.numTris; t++)
        {
            auto c = topo.ctrl[t];
            WriteBits(outputStream, bitPos, 2, c); bitPos += 2;
        }
        return bitPos - bitStart;
    }

    static size_t WriteIsFirstBits(
        ByteSpan outputStream,
        size_t bitPos,
        const BlockTopology& topo)
    {
        size_t bitStart = bitPos;

        // determine how many leading and trailing bits to skip
        size_t numLeads = 3;
        size_t numTrails = 0;

        // write is-first bits
        size_t numStored = topo.numIndices - (numLeads + numTrails);
        for (size_t i = 0; i < numStored; i++)
            WriteBit(outputStream, bitPos++, topo.isFirst[i + numLeads]);

        return bitPos - bitStart;
    }

    static size_t WriteReuseBuffer(
        ByteSpan outputStream,
        size_t bitPos,
        const BlockTopology& topo)
    {
        // write reuse buffer
        for (size_t i = 0; i < topo.numReuseIndices; i++)
        {
            WriteBits(outputStream, bitPos, topo.reuseIndexSize, topo.reuseBuffer[i]);
            bitPos += topo.reuseIndexSize;
        }

        return topo.numReuseIndices * topo.reuseIndexSize;
    }

    static size_t WriteBlockTopology(
        ByteSpan outputStream,
        size_t bitPos,
        const uint8_t* block
    )
    {
        BlockTopology topo;
        GetBlockTopology(topo, block);

        size_t bitStart = bitPos;
        bitPos += WriteControlBits(outputStream, bitPos, topo);
        bitPos += WriteIsFirstBits(outputStream, bitPos, topo);
        bitPos += WriteReuseBuffer(outputStream, bitPos, topo);

        return RoundUpPow2(bitPos, 8);
    }

    static size_t WriteBlockVertexTable(
        ByteSpan outputStream,
        size_t bitPos,
        EncoderState& state,
        size_t sharedVertIndexSize,
        size_t numVerts)
    {
        // write per-block vertex data
        size_t sharedVertCount = 0;
        for (size_t v = 0; v < numVerts; v++)
        {
            bool isShared = state.blockVertIsShared.test(state.numBlockVerts++);
            WriteBit(outputStream, bitPos + v, isShared);
            if (isShared)
            {
                size_t pos = bitPos + numVerts + sharedVertCount * sharedVertIndexSize;
                size_t idx = state.blockSharedVertIndices[state.numBlockSharedVertIndices++];
                WriteBits(outputStream, pos, sharedVertIndexSize, idx);
                sharedVertCount++;
            }
        }

        bitPos += sharedVertCount * sharedVertIndexSize + numVerts;
        bitPos = RoundUpPow2(bitPos, 8);
        return bitPos;
    }

    static size_t WriteBlockIndexPalette(
        ByteSpan outputStream,
        size_t bitPos,
        const IndexPaletteState& state,
        size_t numTris,
        size_t blockTriOffset)
    {
        // build block-local geomID palette
        std::bitset<MAX_CLUSTER_FACES> clusterGeomIDUsed;
        size_t numLocalIDs = 0;
        uint8_t localIDIndexInCluster[DGF::MAX_GEOM_IDS]; // index of each local geomID in cluster
        uint8_t triIndexInLocalPalette[DGF::MAX_TRIS];
        uint8_t clusterIndexToBlock[MAX_CLUSTER_FACES]; // for each cluster geomID, where is it in this block
        for (size_t t = 0; t < numTris; t++)
        {
            uint8_t clusterIdx = state.blockEntryIndices[blockTriOffset + t];
            if (!clusterGeomIDUsed.test(clusterIdx))
            {
                // first time seeing this ID, assign it a local position
                clusterGeomIDUsed.set(clusterIdx);
                localIDIndexInCluster[numLocalIDs] = clusterIdx;
                clusterIndexToBlock[clusterIdx] = uint_truncate<uint8_t>(numLocalIDs++);
            }

            triIndexInLocalPalette[t] = clusterIndexToBlock[clusterIdx];
        }

        // for the special case where all cluster GeomIDs are used, we can skip the local index table
        //  and index the cluster table directly.  In order to do that we need to deal with the case
        //  where the local geomIDs are in a different order than in the cluster palette
        if (numLocalIDs == state.numPaletteEntries)
        {
            for (size_t i = 0; i < numTris; i++)
            {
                // force the local indices to match the cluster-level indices
                uint8_t localIdx = triIndexInLocalPalette[i];
                uint8_t paletteIndex = localIDIndexInCluster[localIdx];
                triIndexInLocalPalette[i] = paletteIndex;
            }
        }

        // write the number of IDs in the block
        WriteBits(outputStream, bitPos, 8, numLocalIDs);
        bitPos += 8;

        if (numLocalIDs < state.numPaletteEntries)
        {
            size_t geomIDIndexSize = BitsNeededUnsigned(state.numPaletteEntries - 1);

            // for each geomID in this block, write out its position in the cluster palette
            for (size_t id = 0; id < numLocalIDs; id++)
            {
                WriteBits(outputStream, bitPos, geomIDIndexSize, localIDIndexInCluster[id]);
                DGF_ASSERT(localIDIndexInCluster[id] < state.numPaletteEntries);
                bitPos += geomIDIndexSize;
            }
        }

        // for each triangle, write the local index of its geomID
        size_t localIndexSize = BitsNeededUnsigned(numLocalIDs - 1);
        if (localIndexSize > 0)
        {
            for (size_t t = 0; t < numTris; t++)
            {
                WriteBits(outputStream, bitPos, localIndexSize, triIndexInLocalPalette[t]);
                bitPos += localIndexSize;
            }
        }

        bitPos = RoundUpPow2(bitPos, 8);
        return bitPos;
    }

    static void AnalyzeBlockUserData(EncoderState& state)
    {
        for (size_t b = 0; b < state.numBlocks; b++)
        {
            const uint8_t* pBlock = state.pInputBlocks + b * DGF::BLOCK_SIZE;

            if (DGF::BlockHasUserData(pBlock))
            {
                uint32_t userData;
                DGF::ReadUserData(pBlock, &userData, 0, sizeof(uint32_t));

                size_t size = RoundUpPow2(BitsNeededUnsigned(userData), 8) / 8;
                
                switch (size)
                {
                case 0: 
                case 1:
                    state.blockDataByteSize++;
                    state.blockUserDataEncodedSize[b] = 1;
                    break;
                case 2:
                    state.blockDataByteSize += 2;
                    state.blockUserDataEncodedSize[b] = 2;
                    break;
                case 3:
                case 4:
                    state.blockDataByteSize += 4;
                    state.blockUserDataEncodedSize[b] = 3;
                    break;
                default:
                    DGF_ASSERT(false);
                }
            }
            else
            {
                state.blockUserDataEncodedSize[b] = 0;
            }
        }

    }

    static size_t DecodeUserDataSize(size_t encodedSize)
    {
        static const size_t lut[] = { 0,1,2,4 };
        return lut[encodedSize&3];
    }

    static ConstByteSpan ReadUserData(uint32_t* userData, ConstByteSpan span, size_t size)
    {
        if (userData)
            *userData = ReadMultiByteDWORD(span, size);
        return IncrementByteSpan(span, size);
    }

    static size_t WriteUserData( ByteSpan span, size_t bitPos, uint32_t data )
    {
        DGF_ASSERT((bitPos % 8) == 0);

        size_t size = RoundUpPow2(BitsNeededUnsigned(data), 8) / 8;
        if (size == 0)
            size = 1;
        if (size == 3)
            size = 4;
        
        size_t bytePos = bitPos / 8;
        WriteMultiByteDWORD(span, bytePos, data, size);
        
        return bitPos + 8 * size;
    }

    std::vector<std::vector<uint8_t>> DGFS::Encoder::EncodeClusters(const DGFBaker::BakerOutput& output)
    {
        DGF_ASSERT(!output.clusterTable.empty()); 

        size_t numClusters = output.clusterTable.size();
        std::vector<const uint8_t*> clusterBlockPtrs(numClusters);
        std::vector<size_t> clusterNumBlocks(numClusters);
        std::vector<const int32_t*> clusterOMMPtrs(numClusters);

        // scan the cluster table and generate per-cluster block and omm descriptor sub-arrays
        size_t totalBlocks = output.dgfBlocks.size() / DGF::BLOCK_SIZE;
        size_t ommDescriptorOffset = 0;
        const uint8_t* blockPtr = output.dgfBlocks.data();
        const int32_t* ommPtr = (output.blockOMMIndices.empty()) ? nullptr : output.blockOMMIndices.data();
        for (size_t i = 0; i < numClusters; i++)
        {
            size_t firstBlock = output.clusterTable[i];
            size_t numBlocks;
            if (i < numClusters - 1)
                numBlocks = output.clusterTable[i + 1] - output.clusterTable[i];
            else
                numBlocks = totalBlocks - firstBlock;

            clusterNumBlocks[i] = numBlocks;
            clusterBlockPtrs[i] = output.dgfBlocks.data() + firstBlock * DGF::BLOCK_SIZE;
            clusterOMMPtrs[i] = ommPtr;
            
            if (ommPtr)
            {
                // advance omm descriptor pointer            
                for (size_t b = 0; b < numBlocks; b++)
                {
                    const uint8_t* block = clusterBlockPtrs[i] + b * DGF::BLOCK_SIZE;
                    ommPtr += DGF::DecodeOMMDescriptorCount(block);
                }                
            }
        }

        std::vector< std::vector<uint8_t> > streams(numClusters);

        auto _Encode =
            [&clusterBlockPtrs,&clusterNumBlocks,&clusterOMMPtrs,this,&streams](size_t idx)
            {
                const uint8_t* pBlocks = clusterBlockPtrs[idx];
                size_t numBlocks = clusterNumBlocks[idx];
                const int32_t* pOMMIndices = clusterOMMPtrs[idx];
                streams[idx] = EncodeCluster(pBlocks, numBlocks, pOMMIndices);
            };

        if (m_config.allowInternalThreading)
        {
            Threading::ParallelFor(numClusters, _Encode);
        }
        else
        {
            for (size_t i = 0; i < numClusters; i++)
                _Encode(i);
        }

        return streams;

    }

    std::vector<std::vector<uint8_t>> DGFS::Encoder::EncodeClusters(const DGFBaker::BakerOutputPreclustered& clusters)
    {
        size_t numClusters = clusters.clusters.size();
        std::vector< std::vector<uint8_t> > streams(numClusters);

        auto _Encode = 
            [&clusters,this,&streams](size_t idx)
            {   
                auto& cluster = clusters.clusters[idx];
                const uint8_t* pBlocks = cluster.dgfBlocks.data();
                size_t numBlocks = cluster.dgfBlocks.size() / DGF::BLOCK_SIZE;
                
                const int32_t* pOMMIndices = nullptr;
                if (!cluster.blockOMMIndices.empty())
                    pOMMIndices = cluster.blockOMMIndices.data();

                streams[idx] = EncodeCluster(pBlocks,numBlocks, pOMMIndices);
            };

        if (m_config.allowInternalThreading)
        {
            Threading::ParallelFor(numClusters, _Encode);
        }
        else
        {
            for (size_t i = 0; i < numClusters; i++)
                _Encode(i);
        }

        return streams;
    }

    std::vector<uint8_t> DGFS::Encoder::EncodeCluster(const uint8_t* pDGFBlocks, size_t numBlocks, const int32_t* pOMMIndices)
    {
        EncoderState state;
        state.pInputBlocks = pDGFBlocks;
        state.numBlocks = numBlocks;
        state.pInputOMMIndices = pOMMIndices;

        // decode block vertices and choose a common exponent
        //  for the cluster.  
        for (size_t block = 0; block < numBlocks; block++)
        {
            const uint8_t* pBlock = pDGFBlocks + block * DGF::BLOCK_SIZE;
            AddVerticesToEncoderState(state,pBlock);
        }

        // mark all verts as either shared or unshared
        //   sort vertices, compute zig-zag encoding and vertex bit widths
        PostProcessVerts(state);

        // determine which primID base fields to store
        AnalyzePrimIDBaseFields(state);

        // Inspect the user-data fields
        AnalyzeBlockUserData(state);

        // de-duplicate geometry and OMM IDs and determine if we need to store a palette
        DeDuplicateGeomIDs(state);
        DeDuplicateOMMIDs(state);
        
        // if we need to store a palette, figure out how large the block-level palettes are
        if (state.needGeomIDPalette)
            ComputeBlockIndexPaletteSizes(state, state.geomPalette);
        
        if (state.needOMMPalette)
            ComputeBlockIndexPaletteSizes(state, state.ommPalette);

        // compute size of block vertex tables and topology
        AnalyzeBlockTopology(state);

        // allocate the output stream
        size_t streamSize = ComputeStreamSize(state);
        std::vector<uint8_t> stream(streamSize);
        ByteSpan outputSpan = MakeByteSpan(stream.data(),stream.size());

        size_t bitPos = 8 * sizeof(Header);

        WriteHeader(outputSpan, state);
        bitPos = WriteBlockDescriptors(outputSpan, state, bitPos);

        if (state.needGeomIDPalette)
            bitPos = WriteClusterIndexPalette(outputSpan, state.geomPalette, bitPos);

        if (state.needOMMPalette)
            bitPos = WriteClusterIndexPalette(outputSpan, state.ommPalette, bitPos);
        
        bitPos = WriteVertexBuffer(outputSpan, state, bitPos);
        
        size_t blockDataStart = bitPos;
        size_t blockTriangleOffset = 0;
        state.numBlockVerts = 0;
        state.numBlockSharedVertIndices = 0;
        for (size_t b = 0; b < numBlocks; b++)
        {
            const uint8_t* pBlock = pDGFBlocks + b * DGF::BLOCK_SIZE;

            DGF::MetaData meta;
            DGF::DecodeMetaData(&meta, pBlock);

            if (state.blockStoresPrimIDBase[b])
            {
                uint32_t primIDBase = (uint32_t)DGF::DecodePrimIDBase(pBlock);
                bitPos = WritePrimIDBase(outputSpan, bitPos, primIDBase);
            }

            if (state.blockUserDataEncodedSize[b])
            {
                uint32_t userData;
                DGF::ReadUserData(pBlock, &userData, 0, sizeof(uint32_t));                
                bitPos = WriteUserData(outputSpan, bitPos, userData);
            }

            bitPos = WriteBlockTopology(outputSpan, bitPos, pBlock);           
            bitPos = WriteBlockVertexTable(outputSpan, bitPos, state, state.blockSharedVertIndexSizes[b], meta.numVerts);

            if (state.geomPalette.numPaletteEntries > 1)
                bitPos = WriteBlockIndexPalette(outputSpan, bitPos, state.geomPalette, meta.numTris, blockTriangleOffset);
            
            if (state.needOMMPalette)
                bitPos = WriteBlockIndexPalette(outputSpan, bitPos, state.ommPalette, meta.numTris, blockTriangleOffset);
            
            blockTriangleOffset += meta.numTris;
        }

        stream.resize(RoundUpPow2(bitPos, 8) / 8);
        return stream;
    }

    static ConstByteSpan DecodeClusterIndexPalette(size_t& paletteSize, uint32_t palette[], ConstByteSpan span)
    {
        const IndexPaletteHeader* header = ByteSpanCast<IndexPaletteHeader>(span);
        span = IncrementByteSpan(span, sizeof(IndexPaletteHeader));

        size_t firstSize = header->firstSize;
        size_t deltaSize = header->deltaSize+1;

        size_t numIDs = header->numIDs;
        palette[0] = ReadMultiByteDWORD(span, firstSize);
        span = IncrementByteSpan(span, firstSize);

        for (size_t i = 1; i < numIDs; i++)
        {
            uint32_t delta = ReadMultiByteDWORD(span, deltaSize);
            palette[i] = palette[i - 1] + delta;
            span = IncrementByteSpan(span, deltaSize);
        }

        // handle omitted LSBs
        if (header->sharedLSBs)
        {
            uint32_t lsb = header->sharedLSBValue;
            for (size_t i = 0; i < numIDs; i++)
                palette[i] = (palette[i] << 1) | lsb;
        }
        
        paletteSize = numIDs;
        return span;
    }

    static uint32_t DecodeFirstClusterIndexPaletteEntry( ConstByteSpan span )
    {
        const IndexPaletteHeader* header = ByteSpanCast<IndexPaletteHeader>(span);
        span = IncrementByteSpan(span, sizeof(IndexPaletteHeader));

        uint32_t entry = ReadMultiByteDWORD(span, header->firstSize);
        if (header->sharedLSBs)
            entry = (entry << 1) | (header->sharedLSBValue);

        return entry;
    }

    static uint32_t ReadGeomIDAndFlagsBaseFromClusterPalette( ConstByteSpan paletteSpan )
    {
        const IndexPaletteHeader* geomHeader = ByteSpanCast<IndexPaletteHeader>(paletteSpan);
        uint32_t firstEntry = DecodeFirstClusterIndexPaletteEntry(paletteSpan);

        // convert from DGF style to CLAS style
        uint32_t id     = firstEntry >> 1;
        uint32_t opaque = (firstEntry & 1) ? GEOMETRY_FLAG_OPAQUE : 0;
        return id + opaque;
    }

    MeshletDecoder::DecodeInfo MeshletDecoder::GetDecodeInfo(const uint8_t* stream, size_t streamSize) const
    {
        ConstByteSpan span = MakeByteSpan(stream, streamSize);

        const Header* header = ByteSpanCast<Header>(span,0);
        size_t byteOffset = sizeof(Header);

        MeshletDecoder::DecodeInfo info;
        info.numVertices = header->numVerts + 1;
        info.haveOMMIndices = header->haveOMMPalette;
        info.vertexFormat = m_config.vertexFormat;
       
        // scan block descriptors and count triangles
        info.numTriangles = 0;
        bool lastBlock = false;
        do
        {
            const BlockDescriptor* block = ByteSpanCast<BlockDescriptor>(span, byteOffset);
            info.numTriangles += (block->numTris + 1);
            lastBlock = block->last;
            byteOffset += sizeof(BlockDescriptor);
        } while (!lastBlock);

        
        // handle geomID palette
        if (header->haveGeomPalette)
        {
            ConstByteSpan geomPaletteSpan = IncrementByteSpan(span, byteOffset);
            const IndexPaletteHeader* geomHeader = ByteSpanCast<IndexPaletteHeader>(geomPaletteSpan);
            info.numGeomIDAndFlagsValues = geomHeader->numIDs;

            // skip geomID palette data
            byteOffset += ComputeClusterIndexPaletteSize(geomHeader);
        }
        else
        {
            // implicit geomID
            info.numGeomIDAndFlagsValues = 1;
        }

        DGF_ASSERT(m_config.bytesPerIndex == 1 || m_config.bytesPerIndex == 2 || m_config.bytesPerIndex == 4);

        size_t vbSize = 0;
        if (m_config.vertexFormat == VF_COMPRESSED1)
        {
            size_t offsetWidth = header->vertexOffsetSize;
            size_t offsetsSize = RoundUpPow2(offsetWidth * info.numVertices, 32) / 8;
            vbSize = 12 + offsetsSize;

            info.vertexPositionBitSize = offsetWidth;
        }
        else
        {
            vbSize = info.numVertices * 3 * sizeof(float);

            info.vertexPositionBitSize = 8 * 3 * sizeof(float);
        }

        size_t ibSize = info.numTriangles * 3 * m_config.bytesPerIndex;

        size_t ommIndexSize = 0;
        if (info.haveOMMIndices)
            ommIndexSize = info.numTriangles * sizeof(int32_t);

        size_t geomIDSize = 0;
        size_t geomIDIndexBufferSize = 0;
        if (info.numGeomIDAndFlagsValues > 1)
        {
            if (m_config.indexedGeomIDs)
            {
                geomIDSize = info.numGeomIDAndFlagsValues * sizeof(uint32_t);
                geomIDIndexBufferSize = info.numTriangles;
            }
            else
            {
                geomIDSize = info.numTriangles * sizeof(uint32_t);
                geomIDIndexBufferSize = 0;
            }
        }

        info.vertexBufferSize = vbSize;
        info.indexBufferSize = ibSize;
        info.geomIDAndFlagsBufferSize = geomIDSize;
        info.geomIDAndFlagsIndexBufferSize = geomIDIndexBufferSize;
        info.ommIndexBufferSize = ommIndexSize;
        info.totalBufferSize = vbSize + ibSize + ommIndexSize + geomIDSize + geomIDIndexBufferSize; 

        return info;
    }

    void MeshletDecoder::BindBufferMemory(DecodeBuffers& buffers, void* allocation, const MeshletDecoder::DecodeInfo& info) const
    {
        uint8_t* pBytes = (uint8_t*)allocation;
        buffers.vertexBuffer = (float*)pBytes;
        pBytes += info.vertexBufferSize;

        if (info.haveOMMIndices)
        {
            buffers.ommIndexBuffer = (int32_t*)pBytes;
            pBytes += info.numTriangles * sizeof(int32_t);
        }
        else
        {
            buffers.ommIndexBuffer = nullptr;
        }

        if (info.numGeomIDAndFlagsValues > 1)
        {
            if (m_config.indexedGeomIDs)
            {
                buffers.geomIDAndFlagsBuffer = (uint32_t*)pBytes;
                pBytes += info.numGeomIDAndFlagsValues * sizeof(uint32_t);
                
                buffers.geomIDAndFlagsIndexBuffer = pBytes;
                pBytes += info.numTriangles;
            }
            else
            {
                buffers.geomIDAndFlagsBuffer = (uint32_t*)pBytes;
                pBytes += info.numTriangles * sizeof(uint32_t);

                buffers.geomIDAndFlagsIndexBuffer = nullptr;
            }
        }
        else
        {
            buffers.geomIDAndFlagsBuffer = nullptr;
            buffers.geomIDAndFlagsIndexBuffer = nullptr;
        }

        buffers.indexBuffer = pBytes;
    }

    static ConstByteSpan ReadAnchorVertex(int3& anchor, ConstByteSpan span)
    {
        uint8_t headerByte = span[0];
        size_t xSize = headerByte & 3;
        size_t ySize = (headerByte >> 2) & 3;
        size_t zSize = (headerByte >> 4) & 3;
        span = IncrementByteSpan(span, 1);
        
        uint32_t zzX = ReadMultiByteDWORD(span, xSize);
        span = IncrementByteSpan(span, xSize);
        
        uint32_t zzY = ReadMultiByteDWORD(span, ySize);
        span = IncrementByteSpan(span, ySize);
        
        uint32_t zzZ = ReadMultiByteDWORD(span, zSize);
        span = IncrementByteSpan(span, zSize);

        anchor.x = ZigZagDecode(zzX);
        anchor.y = ZigZagDecode(zzY);
        anchor.z = ZigZagDecode(zzZ);
        return span;
    }

    static ConstByteSpan DecodeVertexBufferToFloat( float* vertices, ConstByteSpan span, const Header* header )
    {
        size_t channelBytes[3] = {
            header->xBytes,
            header->yBytes,
            header->zBytes
        };

        size_t numVerts = header->numVerts + 1;

        int3 anchor;
        span = ReadAnchorVertex(anchor, span);

        // high bytes (x,y,z)
        // middle bytes (x,y,z)
        // low bytes (x,y,z)
        uint32_t zigzag[MAX_CLUSTER_VERTICES][3] = {};
        size_t byteOffs = 0;
        for (int byte = 2; byte >= 0; byte--)
        {
            for (size_t k = 0; k < 3; k++)
            {
                if (channelBytes[k] > byte)
                {
                    for (size_t v = 1; v < numVerts; v++)
                    {
                        uint32_t bits = span[byteOffs++];
                        zigzag[v][k] |= (bits << (8 * byte));
                    }
                }
            }
        }

        float scale = std::ldexp(1.0f, header->exponent - DGF::EXPONENT_BIAS);

        // first vertex is the anchor
        vertices[0] = anchor.x * scale;
        vertices[1] = anchor.y * scale;
        vertices[2] = anchor.z * scale;

        // remaining vertices are deltas from one another
        int3 prevVert = anchor;
        for (size_t v = 1; v < numVerts; v++)
        {
            int3 delta;
            delta.x = ZigZagDecode(zigzag[v][0]);
            delta.y = ZigZagDecode(zigzag[v][1]);
            delta.z = ZigZagDecode(zigzag[v][2]);
          
            int3 vert = prevVert + delta;
            vertices[3 * v+0] = vert.x * scale;
            vertices[3 * v+1] = vert.y * scale;
            vertices[3 * v+2] = vert.z * scale;

            prevVert = vert;
        }

        return IncrementByteSpan(span, byteOffs);
    }

    static ConstByteSpan DecodeVertexBufferToS24(int3* vertices, ConstByteSpan span, const Header* header)
    {
        size_t channelBytes[3] = {
           header->xBytes,
           header->yBytes,
           header->zBytes
        };

        size_t numVerts = header->numVerts + 1;

        int3 anchor;
        span = ReadAnchorVertex(anchor, span);

        // high bytes (x,y,z)
        // middle bytes (x,y,z)
        // low bytes (x,y,z)
        uint32_t zigzag[MAX_CLUSTER_VERTICES][3] = {};
        size_t byteOffs = 0;
        for (int byte = 2; byte >= 0; byte--)
        {
            for (size_t k = 0; k < 3; k++)
            {
                if (channelBytes[k] > byte)
                {
                    for (size_t v = 1; v < numVerts; v++)
                    {
                        uint32_t bits = span[byteOffs++];
                        zigzag[v][k] |= (bits << (8 * byte));
                    }
                }
            }
        }

        // first vertex is the anchor
        vertices[0] = anchor;

        // remaining vertices are deltas from one another
        int3 prevVert = anchor;
        for (size_t v = 1; v < numVerts; v++)
        {
            int3 delta;
            delta.x = ZigZagDecode(zigzag[v][0]);
            delta.y = ZigZagDecode(zigzag[v][1]);
            delta.z = ZigZagDecode(zigzag[v][2]);

            int3 vert = prevVert + delta;
            vertices[v] = vert;
            prevVert = vert;
        }

        return IncrementByteSpan(span, byteOffs);
    }

    static ConstByteSpan DecodeVertexBufferToCompressed1(uint8_t* vertices, ConstByteSpan span, const Header* header)
    {
        // Decode to S24 first
        int3 tmpVerts[MAX_CLUSTER_VERTICES];
        size_t numVerts = header->numVerts + 1;
        span = DecodeVertexBufferToS24(tmpVerts, span, header);
    
        // compute anchor and offsets from S24 vertices
        AABB3i vertexBox = ComputeVertexAABB(tmpVerts, numVerts);
        uint3 offsetWidths = ComputeVertexOffsetWidths(vertexBox);
        int3 anchor = vertexBox.Min;

        // Bit-pack the vertices.   This involves read/modify/write operations
        //   So we use a temporary array in case a user is decoding into a write-combined buffer

        const constexpr size_t COMPRESSED1_MAX_SIZE = RoundUpPow2(6 * MAX_CLUSTER_VERTICES,8);
        uint64_t packedVerts[COMPRESSED1_MAX_SIZE/8+1] = { 0 };

        size_t vertexWidth = offsetWidths.x + offsetWidths.y + offsetWidths.z;
        size_t yShift = offsetWidths.x;
        size_t zShift = offsetWidths.x + offsetWidths.y;
        size_t bitPos = 0;
        for (size_t i = 0; i < numVerts; i++)
        {
            int3 delta = tmpVerts[i] - anchor;
            uint64_t packed = delta.x;
            packed += (((uint64_t)delta.y) << yShift);
            packed += (((uint64_t)delta.z) << zShift);

            size_t wordPos = bitPos / 64;
            size_t loShift = bitPos % 64;
            size_t hiShift = 64 - loShift;
            uint64_t lo = packed << loShift;
            uint64_t hi = (hiShift > vertexWidth) ? 0 : (packed >> hiShift);
            packedVerts[wordPos + 0] |= lo;
            packedVerts[wordPos + 1] |= hi;

            bitPos += vertexWidth;
        }
       
        size_t packedVertsSize = RoundUpPow2(bitPos, 32) / 8;
        
        // assemble the COMPRESSED1 header, and copy everything into the output location
        uint32_t header0 = header->exponent | (anchor.x << 8);
        uint32_t header1 = (offsetWidths.x - 1) | ((offsetWidths.y - 1) << 4) | (anchor.y << 8);
        uint32_t header2 = (offsetWidths.z - 1) | (anchor.z << 8);
        memcpy(vertices + 0, &header0, 4);
        memcpy(vertices + 4, &header1, 4);
        memcpy(vertices + 8, &header2, 4);
        memcpy(vertices + 12, packedVerts, packedVertsSize);

        return span;
    }

    static ConstByteSpan DecodeBlockTopology( BlockTopology& topo, ConstByteSpan span, const BlockDescriptor& desc )
    {        
        size_t numTris = desc.numTris + 1;        
        topo.numIndices = 3;
        topo.numTris = numTris;
        topo.reuseIndexSize = desc.indexSize + 3;
        topo.numReuseIndices = 0;

        topo.ctrl[0] = DGF::TC_RESTART;
        for (size_t i = 0; i < numTris-1; i++)
        {
            uint8_t ctrl = (span[i / 4] >> (2 * (i % 4))) & 3;
            topo.ctrl[i + 1] = (DGF::TriControlValues)ctrl;
            topo.numIndices += 1 + 2 * (ctrl == DGF::TC_RESTART);
        }

        size_t bitPos = 2 * (numTris - 1);
        topo.isFirst[0] = 1;
        topo.isFirst[1] = 1;
        topo.isFirst[2] = 1;
        for (size_t i = 0; i < topo.numIndices - 3; i++)
        {
            bool bit = ReadBit(span, bitPos + i);
            topo.isFirst[i + 3] = bit;
            topo.numReuseIndices += (bit) ? 0 : 1;
        }
        
        bitPos += (topo.numIndices - 3);
        for (size_t i = 0; i < topo.numReuseIndices; i++)
        {
            topo.reuseBuffer[i] = uint_truncate<uint8_t>(ReadBits(span, bitPos, topo.reuseIndexSize));
            bitPos += topo.reuseIndexSize;
        }

        bitPos = RoundUpPow2(bitPos, 8);
        return IncrementByteSpan(span, bitPos / 8);
    }

    static ConstByteSpan DecodeBlockVertexTable(
        uint8_t* table, 
        ConstByteSpan span, 
        size_t numVerts, 
        size_t& uniqueVertCounter, 
        size_t sharedVertIndexSize,
        size_t clusterNumVerts )
    {
        // 1b/vertex indicating shared or non-shared
        // immediately followed by shared-vert indices
        size_t reuseOffset = numVerts;
        for (size_t i = 0; i < numVerts; i++)
        {
            bool isShared = ReadBit(span, i);
            if (isShared)
            {
                size_t index = ReadBits(span, reuseOffset, sharedVertIndexSize);
                reuseOffset += sharedVertIndexSize;
                index =  ((clusterNumVerts - 1) - index);
                table[i] = uint_truncate<uint8_t>(index);
            }
            else
            {
                table[i] = uint_truncate<uint8_t>(uniqueVertCounter++);
            }
        }

        size_t tableBitSize = RoundUpPow2(reuseOffset,8);
        return IncrementByteSpan(span, tableBitSize/8);
    }

    static size_t CountBlocks(ConstByteSpan span)
    {
        size_t spanOffset = 0;
        while (1)
        {
            const BlockDescriptor* desc = ByteSpanCast<BlockDescriptor>(span,spanOffset);
            spanOffset += sizeof(BlockDescriptor);
            if (desc->last)
                break;
        }

        return spanOffset / sizeof(BlockDescriptor);
    }

    static void ResolveClusterTopology(const uint8_t* vertexTable, const BlockTopology& topo, uint8_t* outputIndices, uint8_t bpi )
    {
        uint8_t stripIndices[DGF::MAX_INDICES];
        size_t counter = 0;
        size_t reuseCounter = 0;
        for (size_t i = 0; i < topo.numIndices; i++)
        {
            size_t idx;
            if (topo.isFirst[i])
                idx = counter++;
            else
                idx = topo.reuseBuffer[reuseCounter++];            

            idx = vertexTable[idx];
            stripIndices[i] = (uint8_t)idx;
        }

        switch (bpi)
        {
        default:
            DGF_ASSERT(false);
            break;
        case 1:
            {
                DGF::ConvertTopologyToTriangleList(outputIndices, topo.ctrl, stripIndices, topo.numTris);
            }
            break;
        case 2:
        case 4:
            {
                uint8_t tmpIndices[DGF::MAX_INDICES];
                DGF::ConvertTopologyToTriangleList(tmpIndices, topo.ctrl, stripIndices, topo.numTris);
                if (bpi == 2)
                {
                    uint16_t* pWideOutputIndices = (uint16_t*)outputIndices;
                    for (size_t i = 0; i < 3 * topo.numTris; i++)
                        pWideOutputIndices[i] = (uint16_t)(tmpIndices[i]);                    
                }
                else
                {
                    uint32_t* pWideOutputIndices = (uint32_t*)outputIndices;
                    for (size_t i = 0; i < 3 * topo.numTris; i++)
                        pWideOutputIndices[i] = (uint32_t)(tmpIndices[i]);
                }
            }
            break;
        }

    }

    static ConstByteSpan DecodeBlockIndexPalette(
        uint8_t blockPaletteIndexInCluster[],
        uint8_t triangleLocalEntryIndices[],
        ConstByteSpan span,
        size_t numTris,
        size_t clusterPaletteSize,
        size_t* numEntriesOut = nullptr
        )
    {
        size_t numEntries = ReadBits(span, 0, 8);
        DGF_ASSERT(numEntries > 0);

        if (numEntriesOut)
            *numEntriesOut = numEntries;

        size_t bitPos = 8;
        if (numEntries == clusterPaletteSize)
        {
            // if entire cluster palette is referenced, we do not store a local indirection table
            for (size_t i = 0; i < numEntries; i++)
                blockPaletteIndexInCluster[i] = (uint8_t)i;
        }        
        else
        {
            size_t paletteIndexSize = BitsNeededUnsigned(clusterPaletteSize - 1);
            for (size_t i = 0; i < numEntries; i++)
            {
                size_t index = ReadBits(span, bitPos, paletteIndexSize);
                blockPaletteIndexInCluster[i] = uint_truncate<uint8_t>(index);
                bitPos += paletteIndexSize;
            }
        }
        
        size_t triangleIndexSize = BitsNeededUnsigned(numEntries - 1);
        DGF_ASSERT(triangleIndexSize <= 8);

        for (size_t i = 0; i < numTris; i++)
        {
            uint8_t localIndex = (uint8_t)ReadBits(span, bitPos, triangleIndexSize);
            triangleLocalEntryIndices[i] = localIndex;
            bitPos += triangleIndexSize;
        }

        bitPos = RoundUpPow2(bitPos, 8);
        return IncrementByteSpan(span, bitPos / 8);
    }


    void MeshletDecoder::Decode(DecodeBuffers& buffers, const uint8_t* stream, size_t streamSize) const
    {
        ConstByteSpan span = MakeByteSpan(stream, streamSize);
        const Header* header = ByteSpanCast<Header>(span);

        span = IncrementByteSpan(span, sizeof(Header));

        // count blocks
        size_t numBlocks = CountBlocks(span);
        const BlockDescriptor* blockDescriptors = ByteSpanCast<BlockDescriptor>(span, 0, numBlocks);

        span = IncrementByteSpan(span, ComputeBlockDescriptorSize(numBlocks));

        uint32_t clusterGeomPalette[MAX_CLUSTER_FACES];
        size_t clusterGeomIDCount = 0;
        if (header->haveGeomPalette)
        {
            // decode geomID palette
            span = DecodeClusterIndexPalette(clusterGeomIDCount, clusterGeomPalette, span);

            // translate geomIDs from DGF-style (flag in LSB) to CLAS style (flags in MSB)
            for (size_t i = 0; i < clusterGeomIDCount; i++)
            {
                uint32_t it = clusterGeomPalette[i];
                uint32_t id = it >> 1;
                uint32_t opaque = it & 1;
                clusterGeomPalette[i] = id | (opaque * DGFS::GEOMETRY_FLAG_OPAQUE);
            }

            // set geomIDAndFlagsBase - only meaningful when there's exactly 1 entry
            if (clusterGeomIDCount == 1)
                buffers.geomIDAndFlagsBase = clusterGeomPalette[0];
            else
                buffers.geomIDAndFlagsBase = 0;
            
            // set maximum geometry index:  We rely on the fact that the palette entries are sorted
            buffers.maxGeomIndexValue = clusterGeomPalette[clusterGeomIDCount - 1] & 0x00FFFFFF;
            
            // in indexed geomID mode, write unique indices to output array
            if (m_config.indexedGeomIDs && clusterGeomIDCount > 1)
            {
                for (size_t i = 0; i < clusterGeomIDCount; i++)
                    buffers.geomIDAndFlagsBuffer[i] = clusterGeomPalette[i];
            }
        }
        else
        {
            // implicit geomID - use default opaque value
            buffers.geomIDAndFlagsBase = GEOMETRY_FLAG_OPAQUE;
            buffers.maxGeomIndexValue = 0;
        }

        int32_t clusterOMMPalette[MAX_CLUSTER_FACES];
        size_t clusterOMMIndexCount = 0;
        if (header->haveOMMPalette)
        {
            // decode cluster OMM palette
            uint32_t tmpPalette[MAX_CLUSTER_FACES];
            span = DecodeClusterIndexPalette(clusterOMMIndexCount, tmpPalette, span);

            // omm indices (which are signed) are zig-zag encoded in the stream to improve compressibility
            for (size_t i = 0; i < clusterOMMIndexCount; i++)
                clusterOMMPalette[i] = ZigZagDecode(tmpPalette[i]);            
        }

        // decode vertex buffer
        if (m_config.vertexFormat == VF_COMPRESSED1)
        {
            span = DecodeVertexBufferToCompressed1((uint8_t*)buffers.vertexBuffer, span, header);
        }
        else
        {
            span = DecodeVertexBufferToFloat((float*)buffers.vertexBuffer, span, header);
        }

        // decode per-block data stream
        size_t triangleOffset = 0;
        size_t uniqueVertCounter = 0;
        size_t clusterNumVerts = header->numVerts + 1;
        uint8_t* outputIndexBuffer = buffers.indexBuffer;
        uint32_t* outputGeomIDBuffer = buffers.geomIDAndFlagsBuffer;
        int32_t* outputOMMIndexBuffer = buffers.ommIndexBuffer;
        uint8_t* outputGeomIDIndexBuffer = buffers.geomIDAndFlagsIndexBuffer;

        for (size_t block = 0; block < numBlocks; block++)
        {
            size_t blockNumTris   = blockDescriptors[block].numTris + 1;
            size_t sharedVertIndexSize = blockDescriptors[block].sharedVertIndexSize;

            // skip primIDBase field if present
            if (blockDescriptors[block].havePrimIDBase)
                span = ReadPrimIDBase(nullptr, span);

            // skip a stored user-data field if present
            size_t userDataSize = DecodeUserDataSize(blockDescriptors[block].encodedUserDataSize);
            if (userDataSize)
                span = ReadUserData(nullptr, span, userDataSize);

            // topology
            BlockTopology topo;
            span = DecodeBlockTopology(topo, span, blockDescriptors[block]);
            size_t numVerts = topo.numIndices - topo.numReuseIndices;
           
            // vertex table
            uint8_t vertexTable[DGF::MAX_VERTS];
            span = DecodeBlockVertexTable(vertexTable, span, numVerts, uniqueVertCounter, sharedVertIndexSize, clusterNumVerts );

            // translate topology from strip to list, and from block index space to cluster index space
            ResolveClusterTopology(vertexTable, topo, outputIndexBuffer, m_config.bytesPerIndex);
            outputIndexBuffer += blockNumTris * 3 * m_config.bytesPerIndex;

            if (header->haveGeomPalette && clusterGeomIDCount > 1)
            {
                // decode block-level geomID palette
                uint8_t blockGeomPalette[DGF::MAX_GEOM_IDS];
                uint8_t triangleIndexInBlockPalette[DGF::MAX_TRIS];
                span = DecodeBlockIndexPalette(blockGeomPalette, triangleIndexInBlockPalette, span, topo.numTris, clusterGeomIDCount);

                if (m_config.indexedGeomIDs)
                {
                    // write geomID indices into output array
                    for (size_t t = 0; t < blockNumTris; t++)
                    {
                        uint8_t idx = triangleIndexInBlockPalette[t];
                        outputGeomIDIndexBuffer[t] = blockGeomPalette[idx];
                    }

                    outputGeomIDIndexBuffer += blockNumTris;
                }
                else
                {
                    // gather geomIDs and write to output array
                    for (size_t t = 0; t < blockNumTris; t++)
                    {
                        uint8_t idx = triangleIndexInBlockPalette[t];
                        outputGeomIDBuffer[t] = clusterGeomPalette[blockGeomPalette[idx]];
                    }

                    outputGeomIDBuffer += blockNumTris;
                }
            }

            if (header->haveOMMPalette)
            {
                // decode block-level OMM palette
                uint8_t blockOMMPalette[DGF::MAX_GEOM_IDS];
                uint8_t triangleIndexInBlockPalette[DGF::MAX_TRIS];
                span = DecodeBlockIndexPalette(blockOMMPalette, triangleIndexInBlockPalette, span, topo.numTris, clusterOMMIndexCount);

                // scatter omm IDs into output array
                for (size_t t = 0; t < blockNumTris; t++)
                {
                    uint8_t idx = triangleIndexInBlockPalette[t];
                    outputOMMIndexBuffer[t] = clusterOMMPalette[blockOMMPalette[idx]];
                }
                outputOMMIndexBuffer += blockNumTris;
            }
        }       
    }

    DGFDecoder::DecodeInfo DGFDecoder::GetDecodeInfo(const uint8_t* stream, size_t streamSize) const
    {
        ConstByteSpan span = MakeByteSpan(stream, streamSize);

        const Header* header = ByteSpanCast<Header>(span);
        
        DGFDecoder::DecodeInfo info;
        info.haveOMMIndices = header->haveOMMPalette;
        info.numBlocks = 0;
        info.numTriangles = 0;

        size_t offset = sizeof(Header);

        // parse block descriptors and count triangles and blocks
        info.numTriangles = 0;
        info.numBlocks = 0;
        bool lastBlock = false;
        do
        {
            const BlockDescriptor* block = ByteSpanCast<BlockDescriptor>(span, offset);

            info.numTriangles += (block->numTris + 1);
            info.numBlocks++;
            lastBlock = block->last;
            offset += sizeof(BlockDescriptor);

        } while (!lastBlock);

        size_t blockBufferSize = info.numBlocks * DGF::BLOCK_SIZE;
        size_t ommIndicesSize = 0;
        if (info.haveOMMIndices)
            ommIndicesSize = sizeof(int32_t) * info.numTriangles;

        info.blockBufferSize      = blockBufferSize;
        info.ommIndicesBufferSize = ommIndicesSize;
        info.totalBufferSize      = blockBufferSize + ommIndicesSize;
        return info;
    }

    void DGFDecoder::BindBufferMemory(DecodeBuffers& buffers, void* allocation, const DecodeInfo& info) const
    {
        uint8_t* alloc = (uint8_t*)allocation;
        buffers.blocksBuffer = alloc;
        alloc += info.numBlocks * DGF::BLOCK_SIZE;
        if (info.haveOMMIndices)
        {
            buffers.ommIndexBuffer = (int32_t*)alloc;
        }
        else
        {
            buffers.ommIndexBuffer = nullptr;
        }
    }

    static void AdjustBlockVertices(const int3 blockVerts[DGF::MAX_VERTS], DGF::OffsetVert* ov, DGF::EncoderInput& input)
    {
        int32_t minX = DGF::ANCHOR_MAX;
        int32_t minY = DGF::ANCHOR_MAX;
        int32_t minZ = DGF::ANCHOR_MAX;
        int32_t maxX = DGF::ANCHOR_MIN;
        int32_t maxY = DGF::ANCHOR_MIN;
        int32_t maxZ = DGF::ANCHOR_MIN;
        uint32_t tz = 0;

        for (size_t v = 0; v < input.numVerts; v++)
        {
            int32_t x = blockVerts[v][0];
            int32_t y = blockVerts[v][1];
            int32_t z = blockVerts[v][2];

            // compute block-level AABB
            maxX = (maxX < x) ? x : maxX;
            maxY = (maxY < y) ? y : maxY;
            maxZ = (maxZ < z) ? z : maxZ;
            minX = (minX < x) ? minX : x;
            minY = (minY < y) ? minY : y;
            minZ = (minZ < z) ? minZ : z;

            // compute trailing zero mask for exponent adjustment
            tz |= (x | y | z);
        }

        // adjust exponent to avoid storing trailing zeros
        int expShift = std::countr_zero(tz);
        if (expShift)
        {
            int adjustedExp = input.exponent + expShift;
            if (adjustedExp > DGF::EXPONENT_MAX)
                expShift = 0;

            input.exponent = input.exponent + expShift;
        }

        minX >>= expShift;
        minY >>= expShift;
        minZ >>= expShift;
        maxX >>= expShift;
        maxY >>= expShift;
        maxZ >>= expShift;

        // re-compute offsets for new anchor
        input.anchorX = minX;
        input.anchorY = minY;
        input.anchorZ = minZ;
        for (size_t v = 0; v < input.numVerts; v++)
        {
            ov[v].xyz[0] = (blockVerts[v][0] >> expShift) - minX;
            ov[v].xyz[1] = (blockVerts[v][1] >> expShift) - minY;
            ov[v].xyz[2] = (blockVerts[v][2] >> expShift) - minZ;
        }

        // adjust bit-widths to meet dgf requirements

        uint8_t bpv[3];
        bpv[0] = (uint8_t) BitsNeededUnsigned(maxX - minX);
        bpv[1] = (uint8_t) BitsNeededUnsigned(maxY - minY);
        bpv[2] = (uint8_t) BitsNeededUnsigned(maxZ - minZ);
        DGF::AdjustVertexWidths(bpv, bpv);

        input.xBits = bpv[0];
        input.yBits = bpv[1];
        input.zBits = bpv[2];
    }

    static void ReEncodeDGFBlock( 
        uint8_t* pBlockOut,
        uint8_t exponent,
        uint32_t primIDBase,
        bool haveUserData,
        uint32_t userDataValue,
        const BlockTopology& topo,
        const uint8_t vertexTable[DGF::MAX_VERTS],
        const int3 vertices[MAX_CLUSTER_VERTICES],
        const uint32_t geomIDs[DGF::MAX_GEOM_IDS],
        const uint8_t geomIDIndices[DGF::MAX_TRIS],
        const uint8_t ommIndices[DGF::MAX_TRIS],
        size_t geomPaletteSize,
        size_t ommPaletteSize )
    {
        // TODO: This implementation decodes everything and
        //  calls the DGF::Encode() method, which is rather naive.
        //  Replace this with something that writes the block directly

        // pick block vertices from the cluster vertex buffer
        size_t numVerts = topo.numIndices - topo.numReuseIndices;
        int3 blockVertices[DGF::MAX_VERTS];
        for (size_t v = 0; v < numVerts; v++)
            blockVertices[v] = vertices[vertexTable[v]];

        DGF::OffsetVert ov[DGF::MAX_VERTS];

        // expand the strip index buffer
        // TODO: Shouldn't have to!  But it's what the encode function wants
        uint8_t stripIndices[DGF::MAX_INDICES];
        size_t vertexCounter = 0;
        size_t reuseCounter = 0;
        for (size_t i = 0; i < topo.numIndices; i++)
        {
            if (topo.isFirst[i])
                stripIndices[i] = (uint8_t) (vertexCounter++);
            else
                stripIndices[i] = topo.reuseBuffer[reuseCounter++];
        }

        // expand geom palette into per triangle IDs and opaque flags
        // TODO: Shouldn't have to!  But its what the encode function wants
        uint32_t triangleGeomIDs[DGF::MAX_TRIS];
        uint8_t triangleOpaqueFlags[DGF::MAX_TRIS];
        for (size_t i = 0; i < topo.numTris; i++)
        {
            uint32_t id = geomIDs[geomIDIndices[i]];
            triangleGeomIDs[i] = id >> 1;
            triangleOpaqueFlags[i] = id & 1;
        }

        DGF::EncoderInput input;
        input.exponent = exponent;
        input.numIndices        = uint_truncate<uint8_t>(topo.numIndices);
        input.numOMMDescriptors = uint_truncate<uint8_t>(ommPaletteSize);
        input.numTris           = uint_truncate<uint8_t>(topo.numTris);
        input.numVerts          = uint_truncate<uint8_t>(numVerts);
        input.primIDBase = primIDBase;
        input.roundTripValidation = false;
        input.userDataSize = haveUserData ? 4 : 0;
        input.verts = ov;
        input.triangleGeomIDs = triangleGeomIDs;
        input.triangleOpaqueFlags = triangleOpaqueFlags;
        input.triControl = topo.ctrl;
        input.indexBuffer = stripIndices;
        input.triangleOMMDescriptorIndices = ommIndices;

        AdjustBlockVertices(blockVertices, ov, input);

        DGF::Encode(pBlockOut, input);

        if (haveUserData)
            DGF::WriteUserData(pBlockOut, &userDataValue, 0, sizeof(uint32_t));
    }

    void DGFDecoder::Decode(DecodeBuffers& buffers, const uint8_t* stream, size_t streamSize) const
    {
        buffers.maxGeomIndexValue = 0;
        buffers.maxPrimIndexValue = 0;
        buffers.totalBlockTris  = 0;
        buffers.totalBlockVerts = 0;

        ConstByteSpan span = MakeByteSpan(stream, streamSize);
        const Header* header = ByteSpanCast<Header>(span);

        span = IncrementByteSpan(span, sizeof(Header));

        // count blocks
        size_t numBlocks = CountBlocks(span);
        const BlockDescriptor* blockDescriptors = ByteSpanCast<BlockDescriptor>(span, 0, numBlocks);

        span = IncrementByteSpan(span, ComputeBlockDescriptorSize(numBlocks));

        uint32_t clusterGeomPalette[MAX_CLUSTER_FACES];
        size_t clusterGeomIDCount = 0;
        if (header->haveGeomPalette)
        {
            span = DecodeClusterIndexPalette(clusterGeomIDCount, clusterGeomPalette, span);
            buffers.maxGeomIndexValue = (clusterGeomPalette[clusterGeomIDCount - 1] >> 1);
        }
        
        int32_t clusterOMMPalette[MAX_CLUSTER_FACES];
        size_t clusterOMMIndexCount = 0;
        if (header->haveOMMPalette)
        {
            uint32_t tmpPalette[MAX_CLUSTER_FACES];
            span = DecodeClusterIndexPalette(clusterOMMIndexCount, tmpPalette, span);
            
            for (size_t i = 0; i < clusterOMMIndexCount; i++)
                clusterOMMPalette[i] = ZigZagDecode(tmpPalette[i]);
        }
        
        // decode vertex buffer to S24
        int3 vertexBuffer[MAX_CLUSTER_VERTICES];
        span = DecodeVertexBufferToS24(vertexBuffer, span, header);

        size_t triangleOffset = 0;
        size_t uniqueVertCounter = 0;
        size_t clusterNumVerts = header->numVerts + 1;
        int32_t* outputOMMIndexBuffer = buffers.ommIndexBuffer;

        uint32_t defaultGeomID = DEFAULT_GEOMID;
        if (clusterGeomIDCount == 1)
            defaultGeomID = clusterGeomPalette[0];

        uint32_t primIDBase = 0;
        for (size_t blockIndex = 0; blockIndex < numBlocks; blockIndex++)
        {
            uint32_t blockNumTris = blockDescriptors[blockIndex].numTris + 1;
            size_t sharedVertIndexSize = blockDescriptors[blockIndex].sharedVertIndexSize;

            // read a stored primID base field if present
            //  blocks which don't have a stored ID base use an incrementing triangle count as a predictor
            if (blockDescriptors[blockIndex].havePrimIDBase)
                span = ReadPrimIDBase(&primIDBase, span);

            // read a stored user-data field if present
            size_t userDataSize = DecodeUserDataSize(blockDescriptors[blockIndex].encodedUserDataSize);
            uint32_t userData   = 0;
            if (userDataSize)
                span = ReadUserData(&userData, span, userDataSize);
           
            // topology
            BlockTopology topo;
            span = DecodeBlockTopology(topo, span, blockDescriptors[blockIndex]);
            size_t numVerts = topo.numIndices - topo.numReuseIndices;

            // vertex table
            uint8_t vertexTable[DGF::MAX_VERTS];
            span = DecodeBlockVertexTable(vertexTable, span, numVerts, uniqueVertCounter, sharedVertIndexSize, clusterNumVerts);

            // GeomID palette
            uint32_t blockGeomIDPalette[DGF::MAX_GEOM_IDS];
            uint8_t triangleGeomIDIndices[DGF::MAX_TRIS];
            size_t geomIDPaletteSize = 0;
            if (header->haveGeomPalette && clusterGeomIDCount > 1)
            {
                // decode block-level geomID palette
                uint8_t localPalette[DGF::MAX_GEOM_IDS];
                span = DecodeBlockIndexPalette(localPalette, triangleGeomIDIndices, span, topo.numTris, clusterGeomIDCount, &geomIDPaletteSize );

                for (size_t i = 0; i < geomIDPaletteSize; i++)
                    blockGeomIDPalette[i] = clusterGeomPalette[localPalette[i]];
            }
            else
            {
                // use default geomID
                blockGeomIDPalette[0] = defaultGeomID;
                geomIDPaletteSize = 1;
                memset(triangleGeomIDIndices, 0, blockNumTris);
            }

            // OMM indices
            size_t numOMMDescriptors = 0;
            uint8_t triangleOMMDescriptorIndices[DGF::MAX_TRIS];
            if (header->haveOMMPalette)
            {
                // decode block-level OMM palette
                uint8_t blockPalette[DGF::MAX_OMM_DESCRIPTORS];
                span = DecodeBlockIndexPalette(blockPalette, triangleOMMDescriptorIndices, span, topo.numTris, clusterOMMIndexCount, &numOMMDescriptors);

                // scatter OMM indices into output array
                for (size_t t = 0; t < blockNumTris; t++)
                {
                    uint8_t idx = triangleOMMDescriptorIndices[t];
                    DGF_ASSERT(idx < numOMMDescriptors);
                    idx = blockPalette[idx];
                    DGF_ASSERT(idx < clusterOMMIndexCount);
                    outputOMMIndexBuffer[t] = clusterOMMPalette[idx];
                }

                outputOMMIndexBuffer += blockNumTris;
            }

            uint8_t* pBlockOut = buffers.blocksBuffer + blockIndex * DGF::BLOCK_SIZE;
            ReEncodeDGFBlock(pBlockOut,
                header->exponent,
                primIDBase,
                userDataSize > 0,
                userData,
                topo,vertexTable,vertexBuffer,
                blockGeomIDPalette, triangleGeomIDIndices, 
                triangleOMMDescriptorIndices,geomIDPaletteSize, numOMMDescriptors );

            buffers.totalBlockTris  += blockNumTris;
            buffers.totalBlockVerts += (uint16_t) numVerts;
            buffers.maxPrimIndexValue = std::max(buffers.maxPrimIndexValue, primIDBase + (blockNumTris - 1));

            primIDBase += blockNumTris;
        }
    }
}
