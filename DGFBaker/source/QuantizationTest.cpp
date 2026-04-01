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

#include "Quantization.h"
#include "SIMD.h"

namespace DGFBaker
{
    void ComputeAndValidateExponent(float f)
    {
        int e0 = (int)std::ceil(std::log2(f < 0.0f ? (double)f / S24_MIN : (double)f / S24_MAX));
        int e1 = f > 0.0f ? ComputeQuantizationExponentPositive(f, 23) : ComputeQuantizationExponentNegative(f);

        float f1 = f * Exp2(-e0);
        float f2 = f * Exp2(-e1);
        int v1 = Quantize(f, (int8_t)e0);
        int v2 = Quantize(f, (int8_t)e1);

        if (f >= 0.0f)
        {
            DGF_ASSERT(v1 > S24_MAX / 2 && v1 <= S24_MAX);
            DGF_ASSERT(v2 > S24_MAX / 2 && v2 <= S24_MAX);
        }
        else
        {
            DGF_ASSERT(v1 <= S24_MIN / 2 && v1 >= S24_MIN);
            DGF_ASSERT(v2 <= S24_MIN / 2 && v2 >= S24_MIN);
        }
        DGF_ASSERT((v1 << 8) >> 8 == v1);
        DGF_ASSERT((v2 << 8) >> 8 == v2);

        if (f <= 0.0f)
            return;

        for (int targetBitWidth = 2; targetBitWidth <= 16; targetBitWidth++)
        {
            double objectEdge = f;
            double imax = (1 << (targetBitWidth - 1)) - 1;
            int e0 = (int)std::ceil(std::log2(objectEdge / imax));
            int e1 = ComputeQuantizationExponentPositive(f, targetBitWidth - 1);

            // This only fails for targetBitWidth=2 and f=1.00000012, but e1 is smaller and still correct.
            //DGF_ASSERT(e0 == e1);

            int bitsNeeded0 = (int)BitsNeededSigned(Quantize(f, (int8_t)e0));
            int bitsNeeded1 = (int)BitsNeededSigned(Quantize(f, (int8_t)e1));
            DGF_ASSERT(bitsNeeded0 == targetBitWidth);
            DGF_ASSERT(bitsNeeded1 == targetBitWidth);
        }
    }

    void QuantizationSanityChecks()
    {
        union
        {
            float f;
            uint32_t u;
        };

        // Test 3 positive values surrounding 1.0f
        f = 1.0f;
        u--;
        ComputeAndValidateExponent(f);
        u++;
        ComputeAndValidateExponent(f);
        u++;
        ComputeAndValidateExponent(f);

        // Test 3 negative values surrounding -1.0f
        f = -1.0f;
        u--;
        ComputeAndValidateExponent(f);
        u++;
        ComputeAndValidateExponent(f);
        u++;
        ComputeAndValidateExponent(f);
    }

} // namespace DGFBaker
