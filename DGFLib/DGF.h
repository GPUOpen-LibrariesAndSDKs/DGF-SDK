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

#include <cstddef>
#include <cstdint>

namespace DGF
{
    constexpr size_t MAX_TRIS        = 64;
    constexpr size_t MAX_VERTS       = 64;
    constexpr size_t MAX_GEOM_IDS    = 32;
    constexpr size_t MAX_GEOMID_BITS = 24;
    constexpr size_t VERTEX_BIT_ALIGNMENT = 4;
    constexpr size_t HEADER_SIZE    = 20;

    // The 'front buffer' contains the vertex data, OMM palette, and GeomID palettes
    constexpr size_t MAX_FRONT_BUFFER_BIT_SIZE = 96 * 8;
    
    // The maximum size of the reuse buffer, after encoding
    constexpr size_t MAX_INDEX_BUFFER_BIT_SIZE = 24 * 8;

    constexpr size_t MIN_INDEX_BITS = 3;    
    constexpr size_t MAX_INDEX_BITS = 6;
    constexpr size_t MAX_INDICES    = 3 * MAX_TRIS;

    constexpr size_t MAX_VERTEX_COMPONENT_BITS  = 16;
    constexpr size_t BLOCK_SIZE                 = 128;
    
    constexpr int32_t ANCHOR_MIN = ~(0x7fffff);
    constexpr int32_t ANCHOR_MAX = 0x7fffff;

    constexpr size_t EXPONENT_BIAS = 127;
    constexpr size_t EXPONENT_MIN  = 1;
    constexpr size_t EXPONENT_MAX  = 232;
    
    constexpr size_t MAX_GEOMID          = ((1 << MAX_GEOMID_BITS) - 1);
    constexpr size_t MAX_CONSTANT_GEOMID = ((1 << 9) - 1);
    constexpr size_t MAX_PRIMID          = ((1 << 29) - 1);

    constexpr size_t MAX_OMM_DESCRIPTORS    = 7;
    constexpr size_t MAX_OMM_HOTPATCH_DWORDS  = 2 + MAX_OMM_DESCRIPTORS;

    constexpr size_t MAX_USERDATA_SIZE = 4;

    struct OffsetVert
    {
        uint16_t xyz[3];
    };
    struct FloatVert
    {
        float xyz[3];
    };

    enum GeomIDModes : uint8_t 
    {
        GEOMID_CONSTANT,
        GEOMID_PALETTE
    };

    enum TriControlValues : uint8_t
    {
        TC_RESTART   = 0,
        TC_EDGE1     = 1,
        TC_EDGE2     = 2,
        TC_BACKTRACK = 3
    };

    struct MetaData
    {
        int32_t  anchorX;
        int32_t  anchorY;
        int32_t  anchorZ;
        uint32_t geomIDPrefix;
        uint32_t primIDBase;
        
        uint8_t exponent;
        uint8_t xBits;
        uint8_t yBits;
        uint8_t zBits;
        uint8_t numTris;
        uint8_t numVerts;
        
        GeomIDModes geomIDMode;
        uint8_t numGeomIDs;
        uint8_t geomIDPrefixBitSize;
        uint8_t headerByte;

        uint8_t numOMMDescriptors; 
        uint8_t numOMMHotPatchDwords;
        uint8_t haveUserData;
    };

    void DecodeMetaData(MetaData* meta, const uint8_t* block);

    size_t DecodeTriangleCount( const uint8_t* block );
    size_t DecodePrimIDBase( const uint8_t* block );
    size_t DecodeVertexCount( const uint8_t* block );

    // Decodes triangle indices.. returns number of vertices
    size_t DecodeTriangleList(uint8_t* pIndexBuffer, const uint8_t* block);
    
    // Convert unpacked control values + index buffer into indexed triangle list
    void ConvertTopologyToTriangleList(uint8_t* pIndexBuffer, const TriControlValues* control, const uint8_t* indices, size_t numTris);

    // Unpack strip control values and index buffer
    //   returns number of vertices
    size_t DecodeTopology( TriControlValues* control, uint8_t* indices, const uint8_t* block );

    // Decodes per-triangle OMM descriptor indices
    void DecodeTriangleOMMIndices(uint8_t* pOMMIndices, const uint8_t* block);

    // Reads data from the OMM hot-patch section
    size_t ReadOMMHotPatchSection( uint32_t* pHotPatchDwords, const uint8_t* block );

    // Decodes per-triangle geomIDs and opacity flags
    void DecodeGeomIDs(uint32_t* pIDs, uint8_t* pOpaqueFlags, const uint8_t* block);

    // Convert the block anchor to a floating-point vertex
    FloatVert DecodeAnchor(const MetaData& meta);

    // Extract a range of fixed-point vertices from a block
    void DecodeOffsetVerts(size_t numVerts, OffsetVert* pVerts, const uint8_t* block);

    // Convert a range of fixed-point vertices to floating-point
    void ConvertOffsetsToFloat(size_t numVerts, FloatVert* pVerts, const OffsetVert* pOffsets, const MetaData& meta);

    // Compute the size of an encoded mesh topology
    void ComputeTopologySizes( size_t* pIsFirstBitSize, 
                               size_t* pControlBitSize, 
                               size_t* pRepeatIndexBitSize, 
                               const TriControlValues* control,
                               const uint8_t* indices,
                               size_t numTris );
   
    // Compute the size of an OMM palette
    size_t ComputeOMMPaletteBitSize(size_t numDescriptors, size_t numTris);

    // Reads user-data from an encoded block
    void ReadUserData( const uint8_t* block, void* output, size_t offsetToRead, size_t sizeToRead );

    // Inserts user-defined data into a previously encoded block
    void WriteUserData( uint8_t* block, const void* userData, size_t offsetToWrite, size_t sizeToWrite );

    struct EncoderInput
    {
        int32_t anchorX;        // Fixed-point anchor positions (must fit in 24b)
        int32_t anchorY;
        int32_t anchorZ;
        uint32_t primIDBase;    // Triangle i's primitive ID is: primIDBase+i

        uint8_t exponent;       // Biased exponent for float->Fixed conversion
        uint8_t xBits;          // Number of bits required to encode the offset verts
        uint8_t yBits;
        uint8_t zBits;
        uint8_t numTris;        
        uint8_t numVerts;
        uint8_t numIndices;     // Number of entries in the strip index buffer
        
        uint8_t numOMMDescriptors;
        uint8_t userDataSize;           // User-data size in bytes 
        uint8_t roundTripValidation;

        const OffsetVert* verts;                // Fixed-point, anchor-relative vertex positions
        const TriControlValues* triControl;     // Triangle strip control bits
        const uint8_t* indexBuffer;             // Triangle strip index buffer
        const uint32_t* triangleGeomIDs;  
        const uint8_t* triangleOpaqueFlags; // only the LSB is used
        const uint8_t* triangleOMMDescriptorIndices;  // Index of each triangle's descriptor in the OMM palette

    };

    // Assembles a DGF block from encoding parameters and geometry data
    void Encode(uint8_t* block, const EncoderInput& input);
       
} // namespace DGF  


// Hookable assertion mechanism.  DGFLib clients may register their own assertion mechansism  

#ifndef DGF_NO_ASSERTS

    namespace DGF
    {
        bool CallAssertDelegate( const char* File, int Line, const char* Condition );

        typedef bool (*pfnAssertDelegate)( const char* File, int Line, const char* Condition );
        void SetAssertDelegate( pfnAssertDelegate filter );

    } // namespace DGF

    #ifdef _MSC_VER
        #define DGF_DEBUG_BREAK __debugbreak()
    #else
        #include <signal.h>
        #define DGF_DEBUG_BREAK raise(SIGTRAP)
    #endif

    #define DGF_ASSERT(Condition)                                               \
        do                                                                      \
        {                                                                       \
            if ((Condition) == false)                                           \
            {                                                                   \
                if (::DGF::CallAssertDelegate(__FILE__, __LINE__, #Condition))  \
                {                                                               \
                    DGF_DEBUG_BREAK;                                            \
                }                                                               \
            }                                                                   \
        } while (false)                                                         

#else
    #define DGF_ASSERT(x) 
#endif

