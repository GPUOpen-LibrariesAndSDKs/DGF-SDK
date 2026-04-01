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

#include "DGFBaker.h"

#include <cstdint>
#include <vector>

namespace DGFS
{
    // Maximum number of triangles across all DGF blocks used to encode a cluster
    constexpr size_t MAX_CLUSTER_FACES    = 256;

    // Maximum number of unique vertex positions across all DGF blocks used to encode a cluster
    constexpr size_t MAX_CLUSTER_VERTICES = 256;

    constexpr size_t MAX_CLUSTER_BLOCKS = MAX_CLUSTER_FACES;

    class Encoder
    {
    public:

        struct Config
        {
            bool allowInternalThreading = true;
        };

        Encoder() {}
        
        Encoder(const Config& config) : m_config(config)
        {
        }

        std::vector<std::vector<uint8_t>> EncodeClusters(const DGFBaker::BakerOutputPreclustered& clusters);

        std::vector<std::vector<uint8_t>> EncodeClusters(const DGFBaker::BakerOutput& output);

        std::vector<uint8_t> EncodeCluster(const uint8_t* pDGFBlocks, size_t numBlocks, const int32_t* pBlockOMMIndices );

    private:
        Config m_config;
    };


    class MeshletDecoder
    {
    public:

        enum VertexFormat
        {
            VF_FLOAT3,
            VF_COMPRESSED1
        };

        struct Config
        {
            uint8_t bytesPerIndex = 1;
            bool indexedGeomIDs = false;
            VertexFormat vertexFormat = VF_FLOAT3;
        };

        struct DecodeInfo
        {             
            uint16_t numVertices;            
            uint16_t numTriangles;
            uint16_t numGeomIDAndFlagsValues;
            
            bool haveOMMIndices;

            size_t totalBufferSize;
            size_t vertexBufferSize;
            size_t indexBufferSize;
            size_t geomIDAndFlagsBufferSize;
            size_t ommIndexBufferSize;
            size_t geomIDAndFlagsIndexBufferSize;
            
            VertexFormat vertexFormat;
            size_t vertexPositionBitSize;
        };

        struct DecodeBuffers
        {
            // vertex buffer.  3 floats per vertex, or COMPRESSED1
            void*    vertexBuffer = nullptr;           

            // 3 vertex indices per triangle
            uint8_t*  indexBuffer = nullptr;

            // one entry for each unique 'geomIDAndFlags' value
            //   used only if DecodeInfo::numGeomIDAndFlagsValues > 1
            uint32_t* geomIDAndFlagsBuffer = nullptr;
            
            // one OMM Index for each triangle.  Used only if 'DecodeInfo::haveOMMIndices' is true
            int32_t*  ommIndexBuffer = nullptr;

            // Per-triangle index into the 'geomIDAndFlagsBuffer'
            //  Used only if 'Config::indexedGeomIDs is true 
            //     and 'DecodeInfo::numGeomIDAndFlagsValues' is > 1
            uint8_t* geomIDAndFlagsIndexBuffer = nullptr;

            // if DecodeInfo::numGeomIDAndFlagsValues <= 1, this value is used for all triangles
            //  if DecodeInfo::numGeomIDAndFlagsValues > 1, this value is zero, and the contents of the 'geomIDAndFlagsBuffer' are used instead
            uint32_t geomIDAndFlagsBase = 0;
            
            // maximum geometry index value
            uint32_t maxGeomIndexValue = 0;
        };

        MeshletDecoder() : m_config()
        {
        }

        MeshletDecoder(Config config) : m_config(config)
        {
        }

        DecodeInfo GetDecodeInfo(const uint8_t* stream, size_t streamSize) const;
        
        void BindBufferMemory(DecodeBuffers& buffers, void* allocation, const DecodeInfo& info) const;

        void Decode(DecodeBuffers& buffers, const uint8_t* stream, size_t streamSize) const;

    private:
        Config m_config;
    };



    class DGFDecoder
    {
    public:

        struct DecodeInfo
        {
            uint16_t numBlocks;
            uint16_t numTriangles;
            bool haveOMMIndices;

            size_t totalBufferSize;
            size_t blockBufferSize;
            size_t ommIndicesBufferSize;
        };

        struct DecodeBuffers
        {
            uint8_t* blocksBuffer = nullptr;

            // one OMM Index for each triangle.  Used only if 'DecodeInfo::haveOMMIndices' is set
            int32_t* ommIndexBuffer = nullptr;

            uint32_t maxGeomIndexValue = 0;
            uint32_t maxPrimIndexValue = 0;
            uint16_t totalBlockVerts   = 0;
            uint16_t totalBlockTris    = 0;
        };

        DecodeInfo GetDecodeInfo(const uint8_t* stream, size_t streamSize) const;

        void BindBufferMemory(DecodeBuffers& buffers, void* allocation, const DecodeInfo& info) const;

        void Decode(DecodeBuffers& buffers, const uint8_t* stream, size_t streamSize) const;

    };

    constexpr uint32_t GEOMETRY_FLAG_OPAQUE = 0x04000000;

}
