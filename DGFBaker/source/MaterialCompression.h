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
#pragma once

#include <array>

namespace DGF
{
    struct CompressedMaterialMetadata
    {
        bool paletteMode = false;
        union {
            uint32_t constant_id : 10 = 0;
            struct {
                uint32_t payloadCount : 5; // 1-32
                uint32_t prefixSize   : 5; // 1-32
            };
        };
    };
    struct CompressedMaterialBuffer
    {
        CompressedMaterialMetadata meta;
        size_t numBits = 0;
        std::array<uint8_t, DGF::BLOCK_SIZE - DGF::HEADER_SIZE> bits;
    };

    
    inline void DecompressMaterialIds(const uint8_t* bits, const CompressedMaterialMetadata& info, 
            const std::span<uint32_t> dstGeomID, 
            const std::span<uint8_t> dstOpaqueFlag )
    {
        if (info.paletteMode == 0)
        {
            std::fill_n(dstGeomID.data(), dstGeomID.size(), info.constant_id>>1 );
            std::fill_n(dstOpaqueFlag.data(),dstOpaqueFlag.size(),info.constant_id&1);            
            return;
        }

        size_t prefixSize  = info.prefixSize;
        size_t payloadSize = 25 - prefixSize;
        size_t indexSize   = BitsNeededUnsigned(info.payloadCount);

        uint32_t prefix = (uint32_t) ReadBits(bits, 0, prefixSize) << payloadSize;

        if (info.payloadCount == 0)
        {           
            std::fill_n(dstGeomID.data(), dstGeomID.size(), prefix>>1 );
            std::fill_n(dstOpaqueFlag.data(),dstOpaqueFlag.size(),prefix&1);
            return;
        }

        size_t numTris = dstGeomID.size();

        std::array<uint32_t, 64> payloads;

        size_t payloadOffset = prefixSize + indexSize*numTris;
        size_t indexOffset = prefixSize;

         // read payloads
        if (payloadSize > 0)
        {
            for (size_t i = 0; i < info.payloadCount + 1; i++)
            {
                payloads[i] = (uint32_t) ReadBits(bits, payloadOffset, payloadSize);
                payloads[i] |= prefix;
                payloadOffset += payloadSize;
            }
        }

        // read indices
        if (indexSize > 0)
        {
            for (size_t i = 0; i < numTris; i++)
            {
                size_t index = ReadBits(bits, indexOffset, indexSize);
                dstGeomID[i]      = payloads[index] >> 1;
                dstOpaqueFlag[i]  = payloads[index] &  1;
                indexOffset += indexSize;
            }
        }

       
    }


    inline bool TryPackMaterialIds(CompressedMaterialBuffer& dst, const std::span<const uint32_t> materialIds, const std::span<uint8_t> opaqueFlags ) {
        std::array<uint32_t, DGF::MAX_TRIS> payloads;
        std::array<uint32_t, DGF::MAX_TRIS> indices;
        uint32_t numPayloads = 0;

        // deduplicate materialIds
        for (size_t i = 0; i < materialIds.size(); i++)
        {
            uint32_t payload = (materialIds[i]<<1) + (opaqueFlags[i]&1);
            size_t idx = numPayloads;
            for (size_t j = 0; j < numPayloads; j++) {
                if (payloads[j] == payload) {
                    idx = j;
                    break;
                }
            }
            indices[i] = (uint32_t)idx;
            if (idx == numPayloads)
                payloads[numPayloads++] = payload;
        }

        // check if we can use constant-id mode
        if (numPayloads == 1 && BitsNeededUnsigned(payloads[0]) <= 10) {
            dst.meta.paletteMode = false;
            dst.meta.constant_id = payloads[0];
            return true;
        }

        if (numPayloads > 32)
            return false; // cannot store payload count in 5 bits

        dst.meta.paletteMode = true;
        dst.meta.payloadCount = numPayloads-1;
        
        // determine prefix
        uint32_t diff = 0;
        for (size_t i = 1; i < numPayloads; i++)
            diff |= payloads[i] ^ payloads[0];

        size_t prefixSize = std::countl_zero(diff<<7); // force a 25b total size
        size_t payloadSize = 25 - prefixSize;
        size_t indexSize = BitsNeededUnsigned(dst.meta.payloadCount);

        dst.meta.prefixSize = prefixSize;

        dst.numBits = 0;

        // store prefix
        WriteBits(dst.bits.data(), dst.numBits, prefixSize, payloads[0] >> payloadSize);
        dst.numBits += prefixSize;

        // store indices
        if (indexSize > 0) {
            for (size_t i = 0; i < materialIds.size(); i++) {
                WriteBits(dst.bits.data(), dst.numBits, indexSize, indices[i]);
                dst.numBits += indexSize;
            }
        }

        // store payloads
        if (payloadSize > 0)
        {
            for (size_t i = 0; i < numPayloads; i++)
            {
                WriteBits(dst.bits.data(), dst.numBits, payloadSize, payloads[i]);
                dst.numBits += payloadSize;
            }
        }
     
        // pad to byte boundary
        if( dst.numBits % 8 )
            dst.numBits += (8 - (dst.numBits % 8));

        return true;
    }


    inline size_t GetMaterialIdsBitSize(size_t numPayloads, size_t numTris, uint32_t firstPayload, uint32_t sharedBitsMask)
    {        
        // determine compressed geomID size
        bool paletteMode = (numPayloads > 1 || BitsNeededUnsigned(firstPayload) > 10);
        if ( paletteMode )
        {                 
            size_t prefixSize = std::countl_zero(sharedBitsMask)-7;
            size_t payloadSize = 25 - prefixSize;
            size_t indexSize = BitsNeededUnsigned((uint32_t)numPayloads - 1);

            size_t size = prefixSize + (numPayloads * payloadSize) + (numTris * indexSize);
            size += (8 - (size % 8)); // align to byte boundary
            return size;
        }
        else
        {
            return 0;
        }
    }

}