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
#include "Quantization.h"
#include "SIMD.h"

namespace DGF
{
    int32_t Quantize(float value, int8_t exponent)
    {
        float scale = ldexpf(1.0f, -exponent); 
        return RoundToInt(value * scale);        
    }

    float DeQuantize(int32_t value, int8_t exponent)
    {
        float scale = ldexpf(1.0f, exponent); 
        return value * scale;
    }

    void QuantizeVerts(const std::span<float3> verticesOut, int8_t exponent)
    {
        // NOTE:  Not using helper classes here, because a scalar loop
        //   is easier for compiler auto-vectorization to get right
        float* pFloat = (float*)verticesOut.data();
        static_assert(sizeof(float3) == 3 * sizeof(float), "oops");

        for (size_t i = 0; i < 3 * verticesOut.size(); i++)
        {
            pFloat[i] = DeQuantize(Quantize(pFloat[i], exponent), exponent);
        }
    }

    void QuantizeVerts(const std::span<int3> verticesOut, const std::span<const float3> verticesIn, int8_t Exp)
    {
        float* pFloat = (float*)verticesIn.data();
        int32_t* pInt = (int32_t*)verticesOut.data();
        static_assert(sizeof(float3) == 3 * sizeof(float), "oops");
        static_assert(sizeof(int3) == 3 * sizeof(int32_t), "oops");

        float scale = ldexpf(1.0f, -Exp);
        size_t n = 3 * verticesIn.size();
        size_t n4 = 4 * (n / 4);
        
        SIMDVecf vscale(scale);
        for (size_t i = 0; i < n4; i += 4)
        {
            SIMDVecf scaled  = SIMDVecf::AlignedLoad(pFloat+i) * vscale;
            SIMDVeci rounded = RoundToInt(scaled);
            rounded.AlignedStore(pInt + i);
        }

        for (size_t i = n4; i<n; i++)
        {
            pInt[i] = RoundToInt( pFloat[i] * scale );
        }
    }

    void DeQuantizeVerts(const std::span<float3> verticesOut, const std::span<const int3> verticesIn, int8_t Exp)
    {
        float* pFloat = (float*)verticesOut.data();
        int32_t* pInt = (int32_t*)verticesIn.data();
        static_assert(sizeof(float3) == 3 * sizeof(float), "oops");
        static_assert(sizeof(int3) == 3 * sizeof(int32_t), "oops");

        float scale = ldexpf(1.0f, Exp);
        for (size_t i = 0; i < 3 * verticesIn.size(); i++)
            pFloat[i] = ((float)pInt[i]) * scale;        
    }

    ExponentInfo ChooseQuantizationExponent(const std::span<Cluster> clusters, size_t targetBits)
    {
        // find object-level AABB, and 
        //  length of largest AABB edge of any cluster
        double maxClusterEdge = 0;
        AABB objectBox;
        for (Cluster& cluster : clusters)
        {
            AABB box;
            for( float3& vert : cluster.Vertices )
                box.Expand(vert);

            float clusterEdge = box.LongEdgeLength();
            if( maxClusterEdge < clusterEdge )
                maxClusterEdge = clusterEdge;

            objectBox.Expand(box);
        }

        // compute the minimum exponent which prevents offset overflow
        //   We use the length of the cluster box to avoid having to place anchors at specific places in order to have enough range
        int8_t minExp = (int8_t) std::ceil( std::log2(maxClusterEdge) - 16 );

        // Adjust the minimum exponent to avoid anchor overflow
        double minCoord = std::min( objectBox.Min.x, std::min( objectBox.Min.y, objectBox.Min.z));
        double maxCoord = std::max( objectBox.Max.x, std::max( objectBox.Max.y, objectBox.Max.z));
        double Amin = S24_MIN;
        double Amax = S24_MAX;
        if (minCoord < 0)
        {
            minExp = std::max( minExp, (int8_t)std::ceil( std::log2( minCoord / S24_MIN ) ) );
        }
        if (maxCoord > 0)
        {
            minExp = std::max( minExp, (int8_t)std::ceil( std::log2( maxCoord / S24_MAX ) ) );
        }

        // choose initial exponent based on object AABB length
        double objectEdge = objectBox.LongEdgeLength();
        double imax = (1<<(targetBits-1))-1;
        int8_t targetExp = (int8_t) std::ceil( std::log2(objectEdge/imax) );

        return { minExp, targetExp };
    }

    int3 QuantizeVert( float3 v, int8_t exp )
    {
        return int3(Quantize(v.x, exp), Quantize(v.y, exp), Quantize(v.z, exp));
    }

    float3 DeQuantizeVert(int3 v, int8_t exp)
    {
        return float3(DeQuantize(v.x, exp), DeQuantize(v.y, exp), DeQuantize(v.z, exp));
    }
}