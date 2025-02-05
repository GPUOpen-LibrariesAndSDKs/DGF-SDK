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
#include "StripGeneration.h"
#include "MaterialCompression.h"
#include "Cluster.h"

namespace DGFBaker
{
    struct Config;
}

namespace DGF
{

    static constexpr size_t MAX_UBLAS_VERTS = 256;
    static constexpr size_t MAX_UBLAS_TRIS  = 256;

    struct BlockDesc
    {
        DGF::OffsetVert blockVerts[DGF::MAX_VERTS];
        DGFStripInfo strips;
        uint32_t triangleGeomIDs[DGF::MAX_TRIS];
        uint8_t triangleOpaqueFlags[DGF::MAX_TRIS];
        size_t numVerts;
        int3 anchor;
        size_t primBase;
        int8_t exponent;
        size_t x_bits;
        size_t y_bits;
        size_t z_bits;

        int32_t ommDescriptors[DGF::MAX_OMM_DESCRIPTORS];
        uint8_t triangleOMMIndices[DGF::MAX_TRIS];
        size_t numOMMDescriptors = 0;
    };

    void BuildBlocks(
        std::vector<BlockDesc>& blocksOut, std::span<TriangleRemap> remapOut, PerformanceData& perfData,        
        std::span<const int3> clusterVertices, int8_t exponent, std::span<const uint8_t> clusterIndices,
        std::span<const uint32_t> clusterMaterialIDs, std::span<const uint8_t> clusterOpaqueFlags,
        size_t triLimit, size_t vertLimit, uint8_t forcedOffsetWidth[3], bool enableUserData
    );

    void EncodeBlocks(
        std::vector<uint8_t>& bytesOut, PerformanceData& perfData,   
        const std::vector<BlockDesc>& blocksOut,
        bool userData 
    );

    void PackBlocksHPG24(std::vector<DGF::Cluster>& clusters, PerformanceData& perfData, int8_t exponent, const DGFBaker::Config& config);
}