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
#include "DGF.h"

#include <bitset>
#include <cassert>
#include <cmath>
#include <tuple>
#include <cstring>

namespace DGF
{
    struct BlockHeader
    {
        // DWORD 0
        uint32_t header_byte   : 8;  // must be 0x6
        uint32_t bits_per_index: 2;  // Encodes 3,4,5,6
        uint32_t num_vertices  : 6;  // Number of vertices
        uint32_t num_triangles : 6;  // 1-64 (add 1 when decoding)  
        uint32_t geom_id_meta  : 10;
  
        // DWORD 1
        uint32_t exponent       : 8;  // Float32 scale (exponent-only) with bias 127.
                                      // Values 1-232 are supported.
        int32_t  x_anchor       : 24;
 
        // DWORD 2
        uint32_t x_bits        : 4;  // 1-16 (XBits + YBits + ZBits must be a multiple of 4 )
        uint32_t y_bits        : 4;  // 1-16 (add 1 when decoding for x,y,z bits)
        int32_t y_anchor       : 24;
  
        // DWORD 3
        uint32_t z_bits               : 4;  // 1-16
        uint32_t omm_descriptor_count : 3; // 0-7
        uint32_t geom_id_mode         : 1;  // 0 = Constant Mode  1= Palette Mode
        int32_t z_anchor              : 24;
 
        // DWORD 4
        uint32_t prim_id_base   : 29;
        uint32_t have_user_data : 1;  // If set, first dword following the header is user meta-data.  If clear, first dword contains vertex data
        uint32_t unused         : 2; // MBZ
 
        // 108B of variable-length data segments follow.
    };
    static_assert(sizeof(BlockHeader) == HEADER_SIZE, "Incorrect structure size");
    
    static uint64_t ReadBits(const uint8_t* bytes, size_t start, size_t len)
    {
        DGF_ASSERT(len <= 48);

        size_t firstByte = (start) / 8;
        size_t lastByte  = (start + (len-1)) / 8;
        size_t numBytes = 1 + lastByte - firstByte;
        
        uint64_t dst = 0;
        for (size_t i = 0; i < numBytes; i++)
        {
            uint64_t currByte = bytes[firstByte+i];
            dst |= (currByte << (8*i));       
        }

        return (dst >> (start%8)) & ((1ull << len) - 1);
    }

    static void WriteBits(uint8_t* bytes, size_t start, size_t len, uint64_t value)
    {
        DGF_ASSERT(len <= 48);

        uint64_t mask = ((1ull << len) - 1);
        size_t offset = start % 8;
        value = value << offset;
        mask = mask << offset;

        bytes += (start / 8);
        while (mask)
        {
            size_t currByte = *bytes;
            currByte = (currByte & ~mask) | (value & mask);
            *bytes = (uint8_t)currByte;
            mask  = mask >> 8;
            value = value >> 8;
            bytes++;
        }
    }

    static uint32_t lzcount(uint32_t bits)
    {
#ifdef _MSC_VER
        unsigned long out;
        char result = _BitScanReverse(&out, bits);
        return result ? (31 - out) : 32;
#elif __GNUC__
        return bits ? __builtin_clz(bits) : 32;
#else
        static_assert(false, "ScanReverse not ported to this platform.");
#endif
    }

    static size_t BitsNeededUnsigned(uint32_t bits)
    {
        size_t lz = 32 - lzcount(bits);
        return lz;
    }

    static size_t RoundUpPow2(size_t n, size_t m)
    {
        return (n + (m - 1)) & ~(m - 1);
    }

    static size_t GetOMMIndexSize(size_t numDescriptors)
    {
        switch (numDescriptors)
        {
        case 0:
        case 1:
            return 0;
        case 2:
            return 1;
        case 3:
        case 4:
            return 2;
        case 5:
        case 6:
        case 7:
            return 3;
        default:
            DGF_ASSERT(false);
            return 0;
        }
    }

    static size_t GetOMMIndexSize(const uint8_t* block)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        return GetOMMIndexSize(header->omm_descriptor_count);    
    }

    static size_t GetOMMHotPatchSectionDwordSize(const uint8_t* block)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        size_t haveOMM            = header->omm_descriptor_count ? 1 : 0;
        size_t size               = 2 * haveOMM + header->omm_descriptor_count; 
        return size;
    }

    static size_t DecodeGeomIDPaletteIDCount(const BlockHeader* header)
    {
        return (header->geom_id_meta >> 5) + 1;
    }

    static size_t DecodeGeomIDPalettePrefixSize(const BlockHeader* header)
    {
        return (header->geom_id_meta) & 0x1f;
    }

    void DecodeMetaData(MetaData* meta, const uint8_t* block)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        meta->anchorX             = header->x_anchor;
        meta->anchorY             = header->y_anchor;
        meta->anchorZ             = header->z_anchor;
        meta->exponent            = header->exponent;
        meta->numTris             = header->num_triangles + 1;
        meta->primIDBase          = header->prim_id_base;
        meta->xBits               = header->x_bits + 1;
        meta->yBits               = header->y_bits + 1;
        meta->zBits               = header->z_bits + 1;
        meta->headerByte          = header->header_byte;
        meta->numVerts            = header->num_vertices + 1;
        meta->haveUserData        = header->have_user_data;
        if (header->geom_id_mode == 0)
        {
            meta->geomIDMode = GeomIDModes::GEOMID_CONSTANT;
            meta->geomIDPrefix = header->geom_id_meta;
            meta->geomIDPrefixBitSize = 24;
            meta->numGeomIDs = 1;
        }
        else
        {
            size_t numIDs             = DecodeGeomIDPaletteIDCount(header);
            size_t prefixBitSize      = DecodeGeomIDPalettePrefixSize(header);
            meta->geomIDMode          = GeomIDModes::GEOMID_PALETTE;
            meta->numGeomIDs          = (uint8_t)numIDs;
            meta->geomIDPrefixBitSize = (uint8_t)prefixBitSize;
            meta->geomIDPrefix        = (uint32_t) ReadBits( block + sizeof(BlockHeader), 0, prefixBitSize);
        }

        meta->numOMMDescriptors     = header->omm_descriptor_count;
        meta->numOMMHotPatchDwords  = (uint8_t) GetOMMHotPatchSectionDwordSize(block);
    }

    size_t DecodeTriangleCount(const uint8_t* block)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        return header->num_triangles+1;
    }
    size_t DecodeVertexCount(const uint8_t* block)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        return header->num_vertices+1;
    }

    size_t DecodePrimIDBase(const uint8_t* block)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        return header->prim_id_base;
    }

    void ConvertTopologyToTriangleList( uint8_t* pIndexBuffer, const TriControlValues* control, const uint8_t* indices, size_t numTris )
    {
        // Convert from strip form to list form
        size_t indexPos = 0;
        size_t prev[3] = {0, 0, 0};
        size_t prevPrev[3] = {0, 0, 0};
        for (size_t i = 0; i < numTris; i++)
        {
            DGF::TriControlValues ctrl = control[i];
            size_t v[3] = {0, 0, 0};
            switch (ctrl)
            {
            case TriControlValues::TC_RESTART:
                v[0] = indexPos++;
                v[1] = indexPos++;
                v[2] = indexPos++;
                break;

            case TriControlValues::TC_EDGE1:
                // triangle dangles from second edge (1,2) of predecessor
                v[0] = prev[2];
                v[1] = prev[1];
                v[2] = indexPos++;
                break;

            case TriControlValues::TC_EDGE2:
                // triangle dangles from third edge (2,0) of predecessor
                v[0] = prev[0];
                v[1] = prev[2];
                v[2] = indexPos++;
                break;

            case TriControlValues::TC_BACKTRACK:
                // triangle dangles from opposite edge of predecessor's predecessor
                DGF_ASSERT(i >= 2);
                switch (control[i - 1])
                {
                case TriControlValues::TC_EDGE1:
                    // triangle dangles from third edge (2,0) of predecessor's predecessor
                    v[0] = prevPrev[0];
                    v[1] = prevPrev[2];
                    v[2] = indexPos++;
                    break;

                case TriControlValues::TC_EDGE2:
                    // triangle dangles from second edge (1,2) of predecessor
                    v[0] = prevPrev[2];
                    v[1] = prevPrev[1];
                    v[2] = indexPos++;
                    break;

                default:
                    DGF_ASSERT(false);
                    break;
                }
            }

            for (size_t j = 0; j < 3; j++)
            {
                pIndexBuffer[3 * i + j] = indices[v[j]];
                prevPrev[j]  = prev[j];
                prev[j]      = v[j];
            }
        }
    }
    
    // Decodes triangle indices.. returns number of vertices
    size_t DecodeTriangleList(uint8_t* pIndexBuffer, const uint8_t* block)
    {
        // decode control values and count indices
        DGF::TriControlValues controlValues[DGF::MAX_TRIS];
        uint8_t indexBuffer[3 * DGF::MAX_TRIS];

        size_t numVerts = DecodeTopology(controlValues, indexBuffer, block);
        size_t numTris  = DecodeTriangleCount(block);
        ConvertTopologyToTriangleList( pIndexBuffer, controlValues, indexBuffer, numTris );
        return numVerts;
    }

    static size_t DecodeNumVerts(const uint8_t* block)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        return header->num_vertices+1;
    }

    static size_t DecodeBitsPerVertex(const uint8_t* block)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);        
        size_t xbits = header->x_bits+1;
        size_t ybits = header->y_bits+1;
        size_t zbits = header->z_bits+1;
        return (xbits+ybits+zbits);
    }


    size_t ComputeOMMPaletteBitSize(size_t numDescriptors, size_t numTris)
    {
        if (numDescriptors == 0)
            return 0;

        size_t numHotPatchDwords = 2 + numDescriptors;
        size_t ommIndexSize = GetOMMIndexSize(numTris);
        return 32 * numHotPatchDwords + RoundUpPow2(ommIndexSize * numTris, 8);
    }

    static size_t GetOMMPaletteBitSize(const uint8_t* block)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        return ComputeOMMPaletteBitSize(header->omm_descriptor_count, header->num_triangles + 1);
    }

    static size_t GetVertexDataBitSize(const uint8_t* block)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        size_t numVerts = DecodeNumVerts(block);
        size_t bpv = DecodeBitsPerVertex(block);
        return RoundUpPow2( numVerts*bpv, 8 );
    }

    static size_t GetGeomPaletteBitSize(const uint8_t* block)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);

        size_t geomIDPaletteBitSize=0;
        if (header->geom_id_mode)
        {
            size_t numIDs         = (header->geom_id_meta >>5) + 1;
            size_t prefixBitSize  = (header->geom_id_meta & 0x1f);
            size_t payloadBitSize = (DGF::MAX_GEOMID_BITS+1) - prefixBitSize;
            size_t indexBitSize   = BitsNeededUnsigned((uint32_t)numIDs - 1);
            size_t numTris        = header->num_triangles+1;

            geomIDPaletteBitSize = RoundUpPow2( numIDs*payloadBitSize + numTris*indexBitSize + prefixBitSize, 8 );
        }
        return geomIDPaletteBitSize;
    }

    static size_t GetFrontBufferBitSize(const uint8_t* block, size_t numVerts)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        size_t vtxBits = header->x_bits + header->y_bits + header->z_bits + 3;
        vtxBits *= numVerts;
        vtxBits = RoundUpPow2(vtxBits,8);

        return vtxBits + GetGeomPaletteBitSize(block) + GetOMMPaletteBitSize(block);
    }

    static size_t DecodeBitsPerIndex( const uint8_t* block )
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        return header->bits_per_index + 3;
    }

    static size_t GetUserDataBitSize(const uint8_t* block)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        return 32 * header->have_user_data;
    }

    static uint8_t* GetUserDataPointer( uint8_t* block )
    {
        return block + sizeof(BlockHeader);
    }
    static const uint8_t* GetUserDataPointer(const uint8_t* block)
    {
        return GetUserDataPointer(const_cast<uint8_t*>(block));
    }

    size_t DecodeTopology(TriControlValues* control, uint8_t* indexBuffer, const uint8_t* block)
    {
        size_t numTris = DecodeTriangleCount(block);        
        size_t numVerts = DecodeNumVerts(block);

        // decode control values and count indices
        // first triangle is implicitly TC_RESTART, and does not store indices
        control[0]  = TriControlValues::TC_RESTART;
        size_t numStoredIndices = 0;
        for (size_t i = 1; i < numTris; i++)
        {
            // remaining triangles reference 3 indices on restart, 1 index otherwise
            size_t ctrl = ReadBits(block, 1024 - 2*i, 2);
            
            DGF_ASSERT( ctrl == TriControlValues::TC_RESTART ||
                        ctrl == TriControlValues::TC_EDGE1 ||
                        ctrl == TriControlValues::TC_EDGE2 ||
                        ctrl == TriControlValues::TC_BACKTRACK);

            if (ctrl == TriControlValues::TC_RESTART)
                numStoredIndices += 3;
            else
                numStoredIndices++;

            control[i] = static_cast<DGF::TriControlValues>(ctrl);
        }

        // "is-first" bits are immediately after the control bits (back to front)
        size_t isFirstBitPos = 1024 - 2*(numTris-1) - 1;

        // index buffer is immediately after the front buffer (front to back)
        size_t indexBitPos  = 8*HEADER_SIZE + GetUserDataBitSize(block) + GetFrontBufferBitSize(block,numVerts);

        // decode the index buffer
        indexBuffer[0] = 0; // first 3 indices are implicit, not stored
        indexBuffer[1] = 1;
        indexBuffer[2] = 2;
        size_t vertexCounter = 3;
        size_t bitsPerIndex  = 2;

        bitsPerIndex = DecodeBitsPerIndex(block);

        for (size_t i = 0; i < numStoredIndices; i++)
        {
            bool isFirst = ReadBits( block, isFirstBitPos-i, 1 );

            size_t indexValue;
            if ( isFirst )
            {
                // first reference to each vertex is omitted
                indexValue   = vertexCounter++;
            }
            else
            {
                // second and subsequent refs to each vertex are stored tightly packed
                //  using the minimal number of bits
                indexValue = ReadBits(block, indexBitPos, bitsPerIndex);
                indexBitPos += bitsPerIndex;
            }

            indexBuffer[i+3] = (uint8_t) indexValue;
        }

        return numVerts;
    }

    void DecodeGeomIDs( uint32_t* pIDs, uint8_t* pOpaqueFlags, const uint8_t* block )
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        size_t numTris            = header->num_triangles + 1;
        DGF_ASSERT(numTris <= DGF::MAX_TRIS);

        if (header->geom_id_mode == 0)
        {
            // constant mode
            uint32_t constantID = header->geom_id_meta>>1;
            uint8_t constantOpaque = (header->geom_id_meta&1);
            for (size_t i = 0; i < numTris; i++)
            {
                pIDs[i] = constantID;
                pOpaqueFlags[i] = constantOpaque;
            }
        }
        else
        {
            // palette mode
            size_t numIDs         = DecodeGeomIDPaletteIDCount(header);
            size_t prefixBitSize  = DecodeGeomIDPalettePrefixSize(header);
            size_t payloadBitSize = (DGF::MAX_GEOMID_BITS+1) - prefixBitSize;
            size_t indexBitSize   = BitsNeededUnsigned((uint32_t)numIDs - 1);

            DGF_ASSERT( numIDs <= DGF::MAX_GEOM_IDS && numIDs > 0 );
            DGF_ASSERT( prefixBitSize <= DGF::MAX_GEOMID_BITS );
            DGF_ASSERT( payloadBitSize <= DGF::MAX_GEOMID_BITS );

            size_t ommSize      = GetOMMPaletteBitSize(block)/8;
            size_t vertSize     = GetVertexDataBitSize(block)/8;
            size_t userDataSize = GetUserDataBitSize(block) / 8;

            const uint8_t* paletteBits = block + sizeof(BlockHeader) + ommSize + vertSize + userDataSize;

            uint32_t geomIDPrefix = (uint32_t) ReadBits(paletteBits, 0, prefixBitSize );
            geomIDPrefix = geomIDPrefix << payloadBitSize;

            size_t indexOffset   = prefixBitSize;
            size_t payloadOffset = indexOffset + numTris*indexBitSize;

            for (size_t i = 0; i < numTris; i++)
            {
                size_t payloadIndex      = ReadBits(paletteBits, indexOffset, indexBitSize);
                size_t payloadPos        = payloadOffset + payloadIndex*payloadBitSize;
                uint32_t geomIDPayload   = (uint32_t) ReadBits(paletteBits, payloadPos, payloadBitSize);
                uint32_t reconstructedID = geomIDPayload | geomIDPrefix;

                pIDs[i]         = reconstructedID >> 1;
                pOpaqueFlags[i] = reconstructedID & 1;

                indexOffset += indexBitSize;
            }
        }
    }

    void DecodeOffsetVerts(size_t numVerts, OffsetVert* pVerts, const uint8_t* block)
    {
        DGF_ASSERT(numVerts <= DGF::MAX_VERTS);

        size_t userDataSize = GetUserDataBitSize(block);

        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        const uint8_t* vertexData = reinterpret_cast<const uint8_t*>(block) + sizeof(BlockHeader) + userDataSize/8;

        size_t xbits = header->x_bits + 1;
        size_t ybits = header->y_bits + 1;
        size_t zbits = header->z_bits + 1;

        size_t bpv = xbits + ybits + zbits;
        DGF_ASSERT(bpv % DGF::VERTEX_BIT_ALIGNMENT == 0);

        for (size_t i = 0; i < numVerts; i++)
        {
            size_t bitOffset = i * bpv;
            size_t vertex    = ReadBits(vertexData, bitOffset, bpv);

            size_t x = vertex;
            size_t y = x >> xbits;
            size_t z = y >> ybits;
           
            x = x & ((1ull << xbits) - 1);
            y = y & ((1ull << ybits) - 1);
            z = z & ((1ull << zbits) - 1);

            DGF_ASSERT(x <= (1 << xbits) - 1);
            DGF_ASSERT(y <= (1 << ybits) - 1);
            DGF_ASSERT(z <= (1 << zbits) - 1);

            pVerts[i].xyz[0] = static_cast<uint16_t>(x);
            pVerts[i].xyz[1] = static_cast<uint16_t>(y);
            pVerts[i].xyz[2] = static_cast<uint16_t>(z);
        }
    }

    FloatVert DecodeAnchor(const MetaData& meta)
    {
        float scale = std::ldexp(1.0f, meta.exponent - EXPONENT_BIAS);
        return FloatVert {static_cast<float>(meta.anchorX) * scale, static_cast<float>(meta.anchorY) * scale, static_cast<float>(meta.anchorZ) * scale};
    }

    void ConvertOffsetsToFloat(size_t numVerts, FloatVert* pVerts, const OffsetVert* pOffsets, const MetaData& meta)
    {
        DGF_ASSERT(numVerts <= DGF::MAX_VERTS);

        float scale = std::ldexp(1.0f, meta.exponent - EXPONENT_BIAS);
        for (size_t i = 0; i < numVerts; i++)
        {
            pVerts[i].xyz[0] = static_cast<float>(pOffsets[i].xyz[0] + meta.anchorX) * scale;
            pVerts[i].xyz[1] = static_cast<float>(pOffsets[i].xyz[1] + meta.anchorY) * scale;
            pVerts[i].xyz[2] = static_cast<float>(pOffsets[i].xyz[2] + meta.anchorZ) * scale;
        }
    }

    void DecodeTriangleOMMIndices(uint8_t* pOMMIndices, const uint8_t* block)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        size_t numTris            = header->num_triangles + 1;
        size_t ommIndexSize       = GetOMMIndexSize(block);
        if (ommIndexSize == 0)
        {
            // no OMM descriptors, fill in zeros
            for (size_t i = 0; i < numTris; i++)
                pOMMIndices[i] = 0;
        }
        else
        {
            // skip the "hot-patch" section and vertex data
            size_t ommPaletteHotPatchSize = 4*GetOMMHotPatchSectionDwordSize(block);
            size_t vertexDataSize = GetVertexDataBitSize(block)/8;
            size_t userDataSize = GetUserDataBitSize(block) / 8;

            // read the indices
            const uint8_t* ommIndicesPos  = block + sizeof(BlockHeader) + ommPaletteHotPatchSize + vertexDataSize + userDataSize;
            for (size_t i = 0; i < numTris; i++)
                pOMMIndices[i] = (uint8_t) ReadBits(ommIndicesPos, i * ommIndexSize, ommIndexSize);
        }
    }

    size_t ReadOMMHotPatchSection(uint32_t* pDwords, const uint8_t* block)
    {
        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        size_t numDwords = GetOMMHotPatchSectionDwordSize(block);
                    
        size_t vertexDataSize = GetVertexDataBitSize(block)/8;
        size_t userDataSize = GetUserDataBitSize(block) / 8;

        const uint32_t* hotPatch = reinterpret_cast<const uint32_t*>( block + sizeof(BlockHeader) +  vertexDataSize + userDataSize) ;
        for( size_t i=0; i<numDwords; i++ )
            pDwords[i] = hotPatch[i];
        return numDwords;
    }

    static void WriteStripControlBits( uint8_t* blockPtr, size_t numTriangles, const DGF::TriControlValues* control )
    {
        DGF_ASSERT(control[0] == TriControlValues::TC_RESTART);
     
        control++; // first triangle is implicitly 'RESTART' and not stored
        numTriangles--;

        // write 2-bit pairs into the block from back to front, in reverse order
        blockPtr += DGF::BLOCK_SIZE - 1;
        size_t n4 = 4 * (numTriangles / 4);
        for( size_t i=0; i<n4; i+= 4 )
        {
            size_t c0 = control[i+0];
            size_t c1 = control[i+1];
            size_t c2 = control[i+2];
            size_t c3 = control[i+3];
            size_t ctrl = (c0 << 6) | (c1 << 4) | (c2 << 2) | c3;
            *(blockPtr--) = (uint8_t)ctrl;
        }

        // handle the remainder
        if (numTriangles % 4)
        {
            uint8_t ctrl = 0;
            size_t shift = 6;
            for (size_t i = n4; i < numTriangles; i++)
            {
                size_t c = control[i];
                ctrl |= (c << shift);
                shift -= 2;
            }
            *blockPtr = ctrl;
        }
    }

    static size_t EncodeTopology(uint8_t* blockPtr, size_t indexBitPos, const EncoderInput& input, size_t bitsPerIndex )
    {
        const TriControlValues* control = input.triControl;
        size_t numTriangles             = input.numTris;
        const uint8_t* indexBuffer      = input.indexBuffer;
        size_t numIndices               = input.numIndices;

        if ( numTriangles == 0 )
            return 0;

        DGF_ASSERT( numTriangles <= MAX_TRIS );
        DGF_ASSERT( numIndices <= 3 * MAX_TRIS );
        DGF_ASSERT( numIndices >= 3 );
        DGF_ASSERT( indexBuffer[0] == 0 && indexBuffer[1] == 1 && indexBuffer[2] == 2 );
        
        WriteStripControlBits(blockPtr, numTriangles, control);
      
        // "is-first" bits are immediately after the control bits (back to front)
        size_t isFirstBitPos = 1024 - 2*(numTriangles-1) - 1;
        size_t indexStart = indexBitPos;

        static_assert(MAX_VERTS <= 64, "More bits please!");
        uint64_t vertexSeen = 0x7; // first 3 indices are always 0,1,2, and so not stored
        size_t vertexCounter = 3; 
        for (size_t i = 3; i < numIndices; i++) // first 3 indices are always 0,1,2, always 'isFirst=1' and so not stored
        {
            size_t idx = indexBuffer[i];
            if (vertexSeen & (1ull << idx))
            {
                // repeated index

                // The 'isFirst' bit is already zero because we zero-initialized the block

                // write the index in the index buffer
                WriteBits(blockPtr, indexBitPos, bitsPerIndex, idx);
                indexBitPos += bitsPerIndex;
            }
            else
            {
                DGF_ASSERT(idx == vertexCounter); // vertices must be ordered by first use
                vertexSeen |= (1ull << idx);
                vertexCounter++;

                // write the 'isFirst' bit for this index
                size_t pos = isFirstBitPos - (i - 3);
                blockPtr[pos / 8] |= (1 << (pos % 8));
            }
        }
    
        size_t indexSize    = indexBitPos - indexStart;
        size_t isFirstSize  = numIndices-3;
        size_t controlSize  = 2*(numTriangles-1);
        size_t topologySize = indexSize + isFirstSize + controlSize;

        DGF_ASSERT( indexBitPos <= isFirstBitPos + (numIndices-3) ); // verify that index buffer doesn't collide with 'isFirst' bits
        DGF_ASSERT( indexSize <= MAX_INDEX_BUFFER_BIT_SIZE );

        return topologySize;
    }

    static void EncodeHeader(BlockHeader* header, const EncoderInput& input, size_t bitsPerIndex )
    {
        DGF_ASSERT( input.anchorX >= ANCHOR_MIN && input.anchorX <= ANCHOR_MAX );
        DGF_ASSERT( input.anchorY >= ANCHOR_MIN && input.anchorY <= ANCHOR_MAX );
        DGF_ASSERT( input.anchorZ >= ANCHOR_MIN && input.anchorZ <= ANCHOR_MAX );
        DGF_ASSERT( input.exponent >= EXPONENT_MIN && input.exponent <= EXPONENT_MAX );
        DGF_ASSERT( input.numTris <= MAX_TRIS && input.numTris > 0 );
        DGF_ASSERT( (input.primIDBase + input.numTris - 1) <= MAX_PRIMID );
        DGF_ASSERT( (input.xBits + input.yBits + input.zBits) % VERTEX_BIT_ALIGNMENT == 0 );
        DGF_ASSERT(input.numOMMDescriptors <= MAX_OMM_DESCRIPTORS);
        DGF_ASSERT( bitsPerIndex <= MAX_INDEX_BITS && bitsPerIndex >= MIN_INDEX_BITS );
        DGF_ASSERT( input.numVerts <= MAX_VERTS && input.numVerts > 0 );
        DGF_ASSERT(input.userDataSize <= MAX_USERDATA_SIZE);

        header->header_byte          = 0x6;
        header->x_anchor             = input.anchorX;
        header->y_anchor             = input.anchorY;
        header->z_anchor             = input.anchorZ;
        header->exponent             = input.exponent;
        header->prim_id_base         = input.primIDBase;
        header->num_triangles        = input.numTris - 1;
        header->x_bits               = input.xBits - 1;
        header->y_bits               = input.yBits - 1;
        header->z_bits               = input.zBits - 1;
        header->num_vertices         = input.numVerts - 1;
        header->bits_per_index       = bitsPerIndex-3;
        header->omm_descriptor_count = input.numOMMDescriptors;
        header->have_user_data       = input.userDataSize ? 1 : 0;

    }

    static size_t EncodeOMMPalette( uint8_t* block, size_t bitPos, const EncoderInput& input )
    {
        if (input.numOMMDescriptors == 0)
            return 0;

        size_t startPos = bitPos;

        // clear hot-patched section
        WriteBits(block, bitPos, 64, 0);
        bitPos += 64;
        size_t descriptorCount = input.numOMMDescriptors;
        for (size_t i = 0; i < descriptorCount; i++ )
        {
            WriteBits(block, bitPos, 32, 0);
            bitPos += 32;
        }

        // write pre-computed section
        size_t indexSize = descriptorCount / 2;
        for (size_t i = 0; i < input.numTris; i++)
        {
            size_t idx = input.triangleOMMDescriptorIndices[i];
            DGF_ASSERT(idx < (1ull << indexSize));
            WriteBits(block, bitPos, indexSize, idx);
            bitPos += indexSize;
        }

        // clear pad bits
        size_t bitSize = bitPos - startPos;
        if (bitSize % 8)
        {
            size_t numPadBits = 8 - (bitSize % 8);
            WriteBits(block, bitPos, numPadBits, 0);
            bitSize += numPadBits;
        }

        return bitSize;
    }

    static size_t EncodeGeomIDs(uint8_t* block, size_t bitPos, const EncoderInput& input )
    {
        BlockHeader* header = reinterpret_cast<BlockHeader*>(block);

        const uint32_t* triangleGeomIDs = input.triangleGeomIDs;
        size_t numTris                  = input.numTris;

        uint32_t payloads[MAX_TRIS] = {0};
        uint32_t indices[MAX_TRIS];
        uint32_t numPayloads = 0;

        // deduplicate GeomID/OpaqueFlag pairs.  opaque flag goes in LSB
        for (size_t i = 0; i < numTris; i++)
        {
            uint32_t idx = numPayloads;
            uint32_t geomID = triangleGeomIDs[i];
            uint32_t opaque = input.triangleOpaqueFlags[i];
            
            DGF_ASSERT( geomID <= MAX_GEOMID );

            uint32_t payload = (geomID << 1) + (opaque & 1);
            for (uint32_t j = 0; j < numPayloads; j++)
            {
                if (payloads[j] == payload)
                {
                    idx = j;
                    break;
                }
            }

            indices[i] = idx;
            if (idx == numPayloads)
                payloads[numPayloads++] = payload;
        }

        DGF_ASSERT( numPayloads <= MAX_GEOM_IDS );

        // check if we can use constant-id mode
        if (numPayloads == 1 && (payloads[0]>>1) <= MAX_CONSTANT_GEOMID )
        {
            header->geom_id_meta = payloads[0];
            header->geom_id_mode = 0;
            return 0;
        }

        // need to use palette mode

        // determine prefix
        uint32_t diff = 0;
        for (size_t i = 1; i < numPayloads; i++)
            diff |= payloads[i] ^ payloads[0];

        size_t prefixSize  = lzcount(diff << 7); // force a 25b total size
        size_t payloadSize = 25 - prefixSize;
        size_t indexSize   = BitsNeededUnsigned(numPayloads - 1);

        header->geom_id_mode = 1;
        header->geom_id_meta = (prefixSize) | ((numPayloads-1) << 5);

        // store prefix
        size_t startPos = bitPos;
        WriteBits( block, bitPos, prefixSize, payloads[0] >> payloadSize );
        bitPos += prefixSize;

        // store indices
        if (indexSize > 0)
        {
            for (size_t i = 0; i < numTris; i++)
            {
                WriteBits(block, bitPos, indexSize, indices[i]);
                bitPos += indexSize;
            }
        }

        // store payloads
        if (payloadSize > 0)
        {
            for (size_t i = 0; i < numPayloads; i++)
            {
                WriteBits(block, bitPos, payloadSize, payloads[i]);
                bitPos += payloadSize;
            }
        }

        // clear pad bits
        size_t bitSize = bitPos - startPos;
        if (bitSize % 8)
        {
            size_t numPadBits = 8 - (bitSize % 8);
            WriteBits(block, bitPos, numPadBits, 0);
            bitSize += numPadBits;
        }

        return bitSize;
    }

    static size_t EncodeVertices(uint8_t* blockData, size_t vertexBitPos, const EncoderInput& input )
    {
        DGF_ASSERT(input.numVerts * (input.xBits + input.yBits + input.zBits) <= MAX_FRONT_BUFFER_BIT_SIZE);

        size_t startPos   = vertexBitPos;
        size_t xbits      = input.xBits;
        size_t xybits     = input.yBits + input.xBits;
        size_t vertexBits = xybits + input.zBits;
        for (size_t i = 0; i < input.numVerts; i++)
        {
            uint64_t x = input.verts[i].xyz[0];
            uint64_t y = input.verts[i].xyz[1];
            uint64_t z = input.verts[i].xyz[2];
            uint64_t vert = x | (y << xbits) | (z << xybits);
            WriteBits(blockData, vertexBitPos, vertexBits, vert);
            vertexBitPos += vertexBits;           
        }

        return RoundUpPow2( vertexBitPos - startPos, 8 );
    }

    void ComputeTopologySizes(size_t* pIsFirstBitSize,
                            size_t* pControlBitSize,
                            size_t* pRepeatIndexBitSize,
                            const TriControlValues* control,
                            const uint8_t* indices,
                            size_t numTris)
    {
        *pControlBitSize = (numTris-1)*2; // 2b/tri.  first tri is always 'RESTART'
        
        size_t numIndices = 0;
        for( size_t i=0; i<numTris; i++ )
            numIndices += (control[i] == TriControlValues::TC_RESTART) ? 3 : 1;

        *pIsFirstBitSize = numIndices-3; // 1b/index.  first 3 are always 0,1,2

        // compute bits per index and count non-first indices
        size_t numRepeatIndices = 0;
        uint8_t maxRepeatIndex = (1<<MIN_INDEX_BITS)-1;

        static_assert(MAX_VERTS <= 64, "More bits please!");
        uint64_t VertSeen = 0;
        for (size_t i = 0; i < numIndices; i++)
        {
            uint8_t idx = indices[i];
            if (VertSeen & (1ull<<idx))
            {
                maxRepeatIndex = std::max(idx, maxRepeatIndex);
                numRepeatIndices++;
            }
            else
            {
                VertSeen |= (1ull << idx);
            }
        }

        size_t bitsPerIndex = BitsNeededUnsigned(maxRepeatIndex);
        *pRepeatIndexBitSize = bitsPerIndex * numRepeatIndices;
    }

    void ReadUserData( const uint8_t* block, void* output, size_t offsetToRead, size_t sizeToRead )
    {
        size_t userDataSize = GetUserDataBitSize(block) / 8;
        DGF_ASSERT(offsetToRead + sizeToRead <= userDataSize);

        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        const uint8_t* userData = GetUserDataPointer(block);
        memcpy(output, userData + offsetToRead, sizeToRead);
    }

    void WriteUserData( uint8_t* block, const void* input, size_t offsetToWrite, size_t sizeToWrite )
    {
        size_t userDataSize = GetUserDataBitSize(block) / 8;
        DGF_ASSERT(offsetToWrite + sizeToWrite <= userDataSize);

        const BlockHeader* header = reinterpret_cast<const BlockHeader*>(block);
        uint8_t* userData = GetUserDataPointer(block);
        memcpy(userData + offsetToWrite, input, sizeToWrite);
    }

    static size_t ComputeIndexSize(const EncoderInput& input)
    {
        static_assert(MAX_VERTS <= 64, "More bits please!");
        uint8_t maxRepeatIndex = (1 << MIN_INDEX_BITS) - 1;
        uint64_t VertSeen = 0;
        for (size_t i = 0; i < input.numIndices; i++)
        {
            uint8_t idx = input.indexBuffer[i];
            if (VertSeen & (1ull << idx))
                maxRepeatIndex = std::max(maxRepeatIndex, idx);
            else
                VertSeen |= (1ull << idx);            
        }
        return BitsNeededUnsigned(maxRepeatIndex);
    }

    void Encode(uint8_t* block, const EncoderInput& input)
    {
        memset(block, 0, BLOCK_SIZE);

        size_t bitsPerIndex = ComputeIndexSize(input);

        // header
        BlockHeader* header = reinterpret_cast<BlockHeader*>(block);
        EncodeHeader(header, input, bitsPerIndex);
        size_t headerBitSize = 8 * sizeof(BlockHeader);

        // userdata
        size_t userDataBitSize = input.userDataSize ? 32 : 0;

        // vertex data (optional)
        size_t vertexBitPos = headerBitSize + userDataBitSize;
        size_t vertexBitSize = EncodeVertices(block, vertexBitPos, input);

        // omm palette (optional)
        size_t ommPaletteBitPos = headerBitSize + userDataBitSize + vertexBitSize;
        size_t ommPaletteBitSize = EncodeOMMPalette(block, ommPaletteBitPos, input);

        // geomID palette (optional)
        size_t geomIDBitPos = headerBitSize + userDataBitSize + vertexBitSize + ommPaletteBitSize;
        size_t geomPaletteBitSize = EncodeGeomIDs(block, geomIDBitPos, input);

        // topology
        size_t indexBitPos = headerBitSize + userDataBitSize + vertexBitSize + geomPaletteBitSize + ommPaletteBitSize;
        size_t topologyBitSize = EncodeTopology(block, indexBitPos, input, bitsPerIndex);

        size_t frontBufferBitSize = ommPaletteBitSize + geomPaletteBitSize + vertexBitSize;
        DGF_ASSERT(frontBufferBitSize <= DGF::MAX_FRONT_BUFFER_BIT_SIZE);

        DGF_ASSERT(headerBitSize + userDataBitSize + frontBufferBitSize + topologyBitSize <= BLOCK_SIZE * 8);

        if (input.roundTripValidation)
        {
            // round-trip verification
            TriControlValues ctrlResult[MAX_TRIS];
            uint8_t indexResult[3 * MAX_TRIS];
            size_t numVerts = DecodeTopology(ctrlResult, indexResult, block);

            DGF_ASSERT(numVerts == input.numVerts);

            for (size_t i = 0; i < input.numTris; i++)
                DGF_ASSERT(ctrlResult[i] == input.triControl[i]);

            for (size_t i = 0; i < input.numIndices; i++)
            {
                DGF_ASSERT(indexResult[i] == input.indexBuffer[i]);
                DGF_ASSERT(indexResult[i] < numVerts);
            }

            OffsetVert vertResult[MAX_VERTS];
            DecodeOffsetVerts(input.numVerts, vertResult, block);
            for (size_t i = 0; i < numVerts; i++)
                for (size_t j = 0; j < 3; j++)
                    DGF_ASSERT(vertResult[i].xyz[j] == input.verts[i].xyz[j]);

            uint32_t geomIDResult[MAX_TRIS];
            uint8_t opaqueFlagResult[MAX_TRIS];
            DecodeGeomIDs(geomIDResult, opaqueFlagResult, block);
            for (size_t i = 0; i < input.numTris; i++)
            {
                DGF_ASSERT(geomIDResult[i] == input.triangleGeomIDs[i]);
                DGF_ASSERT(opaqueFlagResult[i] == (input.triangleOpaqueFlags[i] & 1));
            }

            uint8_t ommIndices[MAX_TRIS];
            DecodeTriangleOMMIndices(ommIndices, block);
            for (size_t i = 0; i < input.numTris; i++)
            {
                if (input.numOMMDescriptors)
                {
                    DGF_ASSERT(ommIndices[i] == input.triangleOMMDescriptorIndices[i]);
                }
                else
                {
                    DGF_ASSERT(ommIndices[i] == 0);
                }
            }
        }
    }
} // namespace DGF


#ifndef DGF_NO_ASSERTS

#include <cstdio>
#include <cstdlib>

namespace DGF
{
    static pfnAssertDelegate g_UserAssertDelegate = nullptr;

    bool CallAssertDelegate( const char* File, int Line, const char* Condition )
    {
        if (g_UserAssertDelegate != nullptr) 
        {
            return g_UserAssertDelegate(File,Line,Condition);
        }
        else
        {
            fprintf(stderr,"%s:(%d) :: DGF ASSERT FAILED: %s\n",File,Line,Condition);
            return true;
        }
    }   

    void SetAssertDelegate(pfnAssertDelegate filter)
    {
        g_UserAssertDelegate = filter;
    }
}

#endif
