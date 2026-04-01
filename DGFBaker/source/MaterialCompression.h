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

namespace DGFBaker
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