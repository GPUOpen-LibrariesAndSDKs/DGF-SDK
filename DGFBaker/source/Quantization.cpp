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

namespace DGFBaker
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
        float* pFloat = (float*)verticesOut.data();
        static_assert(sizeof(float3) == 3 * sizeof(float), "oops");

        for (size_t i = 0; i < 3 * verticesOut.size(); i++)
        {
            pFloat[i] = DeQuantize(Quantize(pFloat[i], exponent), exponent);
        }
    }

    void QuantizeVerts(const std::span<int3> verticesOut, const float3* verticesIn, int8_t Exp)
    {
        float* pFloat = (float*)verticesIn;
        int32_t* pInt = (int32_t*)verticesOut.data();
        static_assert(sizeof(float3) == 3 * sizeof(float), "oops"); // this code assumes tightly packed vecotrs
        static_assert(sizeof(int3) == 3 * sizeof(int32_t), "oops");

        float scale = ldexpf(1.0f, -Exp);
        size_t n = 3 * verticesOut.size();
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

    int8_t ChooseQuantizationExponent( const ClusteredMesh& mesh, const Config& config )
    {
        // find object-level AABB, and 
        //  length of largest AABB edge of any cluster
        double3 maxClusterSize = double3(0,0,0);
        size_t numClusters = mesh.GetNumClusters();
        AABB objectBox;
        for (size_t i = 0; i < numClusters; i++)
        {
            float3 verts[DGFBaker::MAX_CLUSTER_VERTICES];
            size_t numVerts = mesh.ReadClusterVertices(i, (float*)verts, DGFBaker::MAX_CLUSTER_VERTICES);

            AABB box;
            for (size_t i = 0; i < numVerts; i++)
                box.Expand(verts[i]);

            double3 diag = double3(
                box.Max.x - box.Min.x, 
                box.Max.y - box.Min.y, 
                box.Max.z - box.Min.z);

            if (config.quantizeForAnimation)
            {
                // for animation, bound the offsets based on the bounding box diagonal
                //   to allow for rotation, and apply a fudge factor for deformation
                double diagLen  = std::sqrt(diag.x * diag.x + diag.y * diag.y + diag.z * diag.z);                
                diagLen *= config.clusterDeformationPadding;

                double maxSize  = std::max(diagLen, maxClusterSize.x);
                maxClusterSize = double3(maxSize, maxSize, maxSize);
            }
            else
            {
                // for static geometry:  bound the offsets based on the x,y,z extents
                maxClusterSize = max(diag, maxClusterSize);
            }

            objectBox.Expand(box);            
        }

        // combine the provided animation extents with the object level bounding box
        //  This ensures that the exponent clamping will prevent anchor overflow for any animated position
        if (config.quantizeForAnimation)
        {
            DGFBaker::AnimationExtents extents = mesh.GetAnimationExtents();
            if (extents.Valid())
            {
                AABB extentsAABB(
                    float3(extents.Min[0], extents.Min[1], extents.Min[2]),
                    float3(extents.Max[0], extents.Max[1], extents.Max[2]));
                objectBox.Expand(extentsAABB);
            }
        }

        // determine the assigned bit widths for each axis.
        //  if assigned bit-width is zero, quantize based on the maximum bit width
        //   in this case the encoder chooses the bit width
        size_t xbits = config.blockForcedOffsetWidth[0];
        size_t ybits = config.blockForcedOffsetWidth[1];
        size_t zbits = config.blockForcedOffsetWidth[2];
        xbits = xbits ? xbits : DGF::MAX_VERTEX_COMPONENT_BITS;
        ybits = ybits ? ybits : DGF::MAX_VERTEX_COMPONENT_BITS;
        zbits = zbits ? zbits : DGF::MAX_VERTEX_COMPONENT_BITS;

        // compute the minimum exponent which prevents offset overflow
        int8_t minExpX = (int8_t)std::ceil( std::log2((double)maxClusterSize.x) - xbits );
        int8_t minExpY = (int8_t)std::ceil( std::log2((double)maxClusterSize.y) - ybits );
        int8_t minExpZ = (int8_t)std::ceil( std::log2((double)maxClusterSize.z) - zbits );
        int8_t minExp = std::max(minExpX, std::max(minExpY, minExpZ));

        // Adjust the minimum exponent to avoid anchor overflow
        double minCoord = std::min(objectBox.Min.x, std::min(objectBox.Min.y, objectBox.Min.z));
        double maxCoord = std::max(objectBox.Max.x, std::max(objectBox.Max.y, objectBox.Max.z));
        double Amin = S24_MIN;
        double Amax = S24_MAX;
        if (minCoord < 0)
        {
            minExp = std::max(minExp, (int8_t)std::ceil(std::log2(minCoord / S24_MIN)));
        }
        if (maxCoord > 0)
        {
            minExp = std::max(minExp, (int8_t)std::ceil(std::log2(maxCoord / S24_MAX)));
        }

        // choose target exponent based on object AABB length
        double objectEdge = objectBox.LongEdgeLength();
        double imax = (1 << (config.targetBitWidth - 1)) - 1;
        int8_t targetExp = (int8_t)std::ceil(std::log2(objectEdge / imax));

        // clamp the target exponent to the minimum exponent
        return std::max(minExp, targetExp);
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