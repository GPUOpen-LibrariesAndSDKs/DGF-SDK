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

#include "Utils.h"
#include "Cluster.h"
#include <span>

namespace DGFBaker
{

    int32_t Quantize(float value, int8_t exponent);
    float DeQuantize(int32_t value, int8_t exponent);
    void QuantizeVerts(const std::span<float3> vertices, int8_t exponent);
    void QuantizeVerts(int3* verticesOut, const float3* verticesIn, size_t numVertices, int8_t Exp);
    void DeQuantizeVerts(const std::span<float3> verticesOut, const std::span<const int3> quantizedVertices, int8_t exponent);


    int8_t ChooseQuantizationExponent(const DGFBaker::ClusteredMesh& mesh, const DGFBaker::Config& config);

    // Helper functions to determine the appropriate quantization exponent for a given float value and target precision
    int8_t ComputeQuantizationExponentPositive(float value, size_t precision = 23);
    int8_t ComputeQuantizationExponentNegative(float value); // implicitly precision=23, only used for anchors
    
    int3 QuantizeVert(float3 v, int8_t exp);
    float3 DeQuantizeVert(int3 v, int8_t exp);

    void QuantizationSanityChecks();

} // namespace DGF