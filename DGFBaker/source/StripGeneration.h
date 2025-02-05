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

#include <cassert>

#include "AdjacencyGraph.h"

namespace DGF
{
    struct TriangleRemap
    {
        bool operator==(const TriangleRemap& remap) const
        {
            if (InputPrimID != remap.InputPrimID)
                return false;
            if (IndexRotation != remap.IndexRotation)
                return false;
            return true;
        }

        // Index of this output triangle in the input triangle ordering
        uint8_t InputPrimID;
        // for each index in the output triangle,  which of the 3 input vertices is it (0,1,2)
        std::array<uint8_t, 3> IndexRotation;
    };

    struct DGFStripInfo
    {
        size_t numTriangles=0;
        size_t numIndices=0;
        DGF::TriControlValues triControl[DGF::MAX_TRIS];
        uint8_t indexBuffer[DGF::MAX_TRIS*3];

        bool operator==(const DGFStripInfo& other) const
        {
            if (numTriangles != other.numTriangles)
                return false;
            if (numIndices != other.numIndices)
                return false;
            for (size_t t = 0; t < numTriangles; t++)
                if (triControl[t] != other.triControl[t])
                    return false;
            for (size_t i = 0; i < numIndices; i++)
                if (indexBuffer[i] != other.indexBuffer[i])
                    return false;
            return true;
        }

        DGFStripInfo()
        {

        }

        inline std::span<uint8_t> GetIndices()
        {
            return { indexBuffer, numIndices };
        }

        struct Sizes
        {
            size_t controlBitSize;
            size_t isFirstSize;
            size_t indexSize;
            size_t Total() const { return controlBitSize + isFirstSize + indexSize; };
        };

        Sizes PackedSize() const
        {
            size_t controlBitSize = 2 * (numTriangles - 1); // first triangle is always a restart
            size_t isFirstTotal   = numIndices - 3;
            size_t indexTotal  = 0;
            
            {
                size_t bitsPerIndex  = 3;

                std::bitset<DGF::MAX_VERTS> vertexSeen;
                uint8_t stripPosition[256];
                vertexSeen.set(indexBuffer[0]); // first 3 indices are implicit
                vertexSeen.set(indexBuffer[1]);
                vertexSeen.set(indexBuffer[2]);
                stripPosition[indexBuffer[0]] = 0; // first-use position for each vertex 
                stripPosition[indexBuffer[1]] = 1;
                stripPosition[indexBuffer[2]] = 2;

                size_t repeatCount = 0;
                size_t vertexCount = 3;
                for (size_t i = 3; i < numIndices; i++)
                {
                    auto idx = indexBuffer[i];
                    if (!vertexSeen.test(idx))
                    {
                        vertexSeen.set(idx);
                        stripPosition[idx] = (uint8_t) vertexCount++;
                    }
                    else
                    {
                        bitsPerIndex = std::max( bitsPerIndex, BitsNeededUnsigned(stripPosition[idx]));
                        repeatCount++;
                    }
                }

                indexTotal += repeatCount * bitsPerIndex;
            }

            Sizes sizes;
            sizes.controlBitSize = controlBitSize;
            sizes.indexSize = indexTotal;
            sizes.isFirstSize = isFirstTotal;

            return sizes;
        }
    };
}


namespace DGF
{
    constexpr bool ENABLE_BACKTRACK = true;

    void GenerateStrips(DGFStripInfo& strips, BlockAdjacencyGraph& graph, TriangleRemap* remapOut, const uint8_t* indices, size_t numTris);
}