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

#include <cstdint>
#include <vector>
#include <functional>
#include <ostream>

namespace DGFBaker
{  
    constexpr size_t MAX_CLUSTER_FACES = 256;
    constexpr size_t MAX_CLUSTER_VERTICES = 256;

    struct TriangleAttributes
    {
        uint32_t geomID : 24 = 0;
        uint32_t opaque : 1 = 1;
        uint32_t hasOMM : 1 = 0;
        int32_t  ommIndex   = -1;

        inline bool operator==(const TriangleAttributes& rhs) const  = default;        
        inline bool operator!=(const TriangleAttributes& rhs) const  = default;
        inline bool operator<(const TriangleAttributes& rhs) const   = default;
        inline auto operator<=>(const TriangleAttributes& rhs) const = default; 
    };
     
    class BakerMesh
    {
    public:

        // Reads a set of vertices by index (3 floats per vertex)
        typedef std::function<void(float*, const uint32_t* pVertIndices, size_t numVertices)> VertexReader;

        // Reads a range of triangle indices (3 indices per tri)
        typedef std::function<void(uint32_t*, const uint32_t* pTriIndices, size_t numTris)> IndexReader;

        // Reads a range of triangle attributes (1 per tri) for a set of indexed triangles
        typedef std::function<void(TriangleAttributes*, const uint32_t* pTriIndices, size_t numTris )> AttributeReader;

        BakerMesh(const float* vb, const uint32_t* ib, size_t numVerts, size_t numTris)
            : BakerMesh( DefaultVertexReader(vb), DefaultIndexReader(ib), NullAttributeReader(), numVerts, numTris )
        {

        }

        BakerMesh(const float* vb, const uint32_t* ib, const TriangleAttributes* attribs, size_t numVerts, size_t numTris)
            : BakerMesh( DefaultVertexReader(vb), DefaultIndexReader(ib), DefaultAttributeReader(attribs), numVerts, numTris )
        {

        }

        BakerMesh(VertexReader vertReader, IndexReader idxReader, size_t numVerts, size_t numTris)
            : m_VertexReader(vertReader), m_IndexReader(idxReader), m_AttributeReader(NullAttributeReader()), m_numVerts(numVerts), m_numTris(numTris)
        {
        }

        BakerMesh( VertexReader vertReader, IndexReader idxReader, AttributeReader attribReader, size_t numVerts, size_t numTris)
            : m_VertexReader(vertReader), m_IndexReader(idxReader), m_AttributeReader(attribReader), m_numVerts(numVerts), m_numTris(numTris)
        {
        }

        size_t GetTriangleCount() const { return m_numTris; };
        size_t GetVertexCount() const { return m_numVerts; }
        bool HasTriangleAttributes() const { return (bool)m_AttributeReader; }
        
        void ReadIndices(uint32_t indices[3], uint32_t triIndex) const
        {
            m_IndexReader(indices,&triIndex, 1);
        }
        void ReadIndices(uint32_t* indices, const uint32_t* triIndex, size_t numTris) const
        {
            m_IndexReader(indices, triIndex, numTris);
        }

        void ReadVertex(float vert[3], uint32_t index) const
        {
            m_VertexReader(vert,&index,1);
        }

        void ReadVertices(float* vert, const uint32_t* indices, size_t numVerts ) const
        {
            m_VertexReader(vert, indices, numVerts);
        }

        void ReadAttributes(TriangleAttributes* attribs, uint32_t index) const
        {
            m_AttributeReader(attribs,&index,1);
        }
        void ReadAttributes(TriangleAttributes* attribs, const uint32_t* indices, size_t numTris ) const
        {
            m_AttributeReader(attribs,indices,numTris);
        }


        static VertexReader DefaultVertexReader(const float* vb)
        {
            return
                [vb](float* out, const uint32_t* ib, size_t numVerts)
                {
                    for (size_t i = 0; i < numVerts; i++)
                    {
                        size_t idx = ib[i];
                        for (size_t j = 0; j < 3; j++)
                            out[3 * i + j] = vb[3 * idx + j];
                    }
                };
        }

        static IndexReader DefaultIndexReader(const uint32_t* ib)
        {
            return
                [ib](uint32_t* out, const uint32_t* triIndices, size_t numTris)
                {
                    for (size_t i = 0; i < numTris; i++)
                    {
                        size_t idx = triIndices[i];
                        for (size_t j = 0; j < 3; j++)
                            out[3 * i + j] = ib[3 * idx + j];
                    }
                };
        }

        static AttributeReader NullAttributeReader()
        {
            return ConstantAttributeReader(TriangleAttributes());
        }

        static AttributeReader DefaultAttributeReader(const TriangleAttributes* attribs)
        {
            return
                [attribs](TriangleAttributes* out, const uint32_t* triIndices, size_t numTris)
                {
                    for (size_t i = 0; i < numTris; i++)
                        out[i] = attribs[triIndices[i]];
                };
        }

        static AttributeReader ConstantAttributeReader(TriangleAttributes attribs)
        {
            return
                [attribs](TriangleAttributes* out, const uint32_t* triIndices, size_t numTris)
                {
                    for (size_t i = 0; i < numTris; i++)
                        out[i] = attribs;
                };
        }

    private:
        friend class Baker;

        VertexReader m_VertexReader;
        IndexReader m_IndexReader;
        AttributeReader m_AttributeReader;
        size_t m_numVerts = 0;
        size_t m_numTris = 0;            
    };

    struct TriangleRemapInfo
    {
        uint32_t InputPrimIndex;    // For each output triangle, which input triangle was it?
        uint8_t IndexRotation[3];   // For each vertex of this triangle, which input vertex was it (0,1, or 2)
    };

    enum class PackerMode
    {
        HPG24,
        SAH,
        DEFAULT = SAH
    };
    
    PackerMode PackerModeFromString(const char* str);
    const char* PackerModeToString(PackerMode mode);

    struct Config
    {
        size_t clusterMaxFaces = 128;
        size_t clusterMaxVerts = 256;
        size_t targetBitWidth = 16;     // target signed precision used to select quantization exponent
        size_t blockMaxTris  = 64;
        size_t blockMaxVerts = 64;
        uint8_t blockForcedOffsetWidth[3] = { 0,0,0 }; // if non-zero, bit-width of x,y,z offsets are forced to these values
                                                       //    if the vertex data needs more precision, data loss may occur.
                                                       // If the sum of the widths is not a multiple of 4, x,y,z are incremented until it is

        std::ostream* outputStream = nullptr;
        bool printPerfData = false;                     
        bool validateClusters = false;                 // Enable sanity-checks for debugging cluster formation
        bool generateVertexTable = false;              // Generate the vertex remapping table
        bool generateClusterTable = false;             // Generate the cluster table
        bool generateTriangleRemap = false;            // Generate the triangle remapping table
        bool enableUserData = false;                   // Reserve space for a 'user-data' field in the blocks
        bool encoderRoundTripValidation = false;       // Enable sanity-checks for debugging DGFLib encoder
        PackerMode packer = PackerMode::DEFAULT;

        bool enableExponentAdjust  = true;           // Allow block exponents to vary to improve compression for pre-quantized input
    };
    
    class BakerOutput
    {
    public:

        BakerOutput(
            std::vector<uint8_t> dgfBlocksIn,
            std::vector<TriangleRemapInfo> remapIn,
            std::vector<int32_t> ommIndicesIn,
            std::vector<uint32_t> vertexTable,
            std::vector<uint32_t> clusterTable )
            : dgfBlocks(std::move(dgfBlocksIn)),
              triangleRemap(std::move(remapIn)),
              blockOMMIndices(std::move(ommIndicesIn)),
              vertexTable(std::move(vertexTable)),
              clusterTable(std::move(clusterTable))
        {
        }

        // DGF blocks as raw bytes
        std::vector<uint8_t> dgfBlocks;                 

        // Table mapping each encoded triangle to its original input triangle
        //  There is one entry per triangle in block order
        std::vector<TriangleRemapInfo> triangleRemap;   

        // Array of OMM indices referenced by each block, in block order
        //  There is one entry per OMM descriptor.  Size of table is the sum of OMM descriptor counts across DGF blocks
        std::vector<int32_t> blockOMMIndices;           

        // Table mapping DGF block vertices to input vertices.  
        //    Each block has one entry per vertex which indexes into the original input vertex buffer
        //    Size of the table is the sum of vertex counts across all output blocks
        std::vector<uint32_t> vertexTable;

        // Table giving the index of the first DGF block for each cluster
        //    Size of table is the number of clusters
        std::vector<uint32_t> clusterTable;
    };

    class Baker
    {
    public:
        
        Baker(const Config& config)
            : m_config(config) 
        {
        }

        BakerOutput BakeDGF( const BakerMesh& mesh );

    protected:
        Config m_config;
    };


    // Decoding support
    
    class DecodedMesh
    {
    public:

        DecodedMesh(
            std::vector<float> vb, 
            std::vector<uint32_t> ib, 
            std::vector<int8_t> quant, 
            std::vector<TriangleAttributes> attribs,
            std::vector<uint32_t> triIndex )
            : m_vb(std::move(vb)), 
              m_ib(std::move(ib)), 
              m_attribs(std::move(attribs)), 
              m_quantFactor(std::move(quant)),
              m_triIndex( std::move(triIndex) )
        {
        };

        size_t GetVertexCount() const { return m_vb.size()/3; };
        size_t GetTriangleCount() const { return m_ib.size()/3; };
        const float* GetVertexBuffer() const { return m_vb.data(); }
        const uint32_t* GetIndexBuffer() const { return m_ib.data(); }
        const TriangleAttributes* GetAttributeBuffer() const { return m_attribs.data(); }
        const int8_t* GetVertexQuantizationFactorBuffer() const { return m_quantFactor.data(); }
        const uint32_t* GetPrimIDs() const { return m_triIndex.data(); }

    protected:

        std::vector<float> m_vb;                    // Decoded vertex positions (may have duplicates)
        std::vector<uint32_t> m_ib;                 // Decoded index buffer
        std::vector<TriangleAttributes> m_attribs;  // Decoded triangle attributes
        std::vector<int8_t> m_quantFactor;          // Signed exponent for each vertex (from its block), for re-quantizing input data for comparison
        std::vector<uint32_t> m_triIndex;           // PrimID for each triangle (from its block)
    };
    
    DecodedMesh DecodeDGF( const BakerOutput& output );
    DecodedMesh DecodeDGF( const uint8_t* blocks, size_t sizeInBytes, const int32_t* pOMMIndices );

    // Validation
    class Validator
    {
    public:
        Validator(std::ostream& errorStream)
            : m_errorStream(errorStream)
        {
        };

        bool ValidateDGF( const BakerMesh& input, const BakerOutput& output );

    protected:

        std::ostream& m_errorStream;
    };


    // Generate a block-map data structure for mapping of primitive IDs to their block locations
    //  The input is an array of DGF blocks with ascending, tightly-packed primitive IDs
    std::vector<uint32_t> GenerateDGFBlockMap( const uint8_t* pBlocks, size_t numBlocks, size_t totalTris, size_t groupSize );

} 
