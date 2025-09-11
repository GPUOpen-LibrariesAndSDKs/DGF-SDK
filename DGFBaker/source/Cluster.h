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

#pragma once
// clang-format off

#include "Utils.h"
#include "StripGeneration.h" 
#include "DGFBaker.h"

namespace DGFBaker
{
    typedef BakerMeshCluster Cluster;

    struct PackClusterInputs
    {
        size_t numVerts;
        size_t numTris;
        const float3* Verts;
        const uint8_t* Indices;
        const DGFBaker::TriangleAttributes* Attributes;
        int8_t exponent;
    };

    struct PackClusterResult
    {
        std::vector<uint8_t> dgfBlocks;
        std::vector<LocalTriangleRemap> localRemap;
        std::vector<int32_t> outputOMMIndices;
    };

}