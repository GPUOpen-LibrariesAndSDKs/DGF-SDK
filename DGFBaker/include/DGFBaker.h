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
#include <iostream>
#include <cstring>
#include <limits>

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
     
    class AnimationExtents
    {
    public:

        AnimationExtents()
        {
            for (size_t i = 0; i < 3; i++)
            {
                Min[i] = std::numeric_limits<float>::max();
                Max[i] = std::numeric_limits<float>::min();
            }
        }
        AnimationExtents(const AnimationExtents& rhs) = default;
        AnimationExtents(AnimationExtents&& rhs) = default;
        AnimationExtents& operator=(const AnimationExtents& rhs) = default;
        AnimationExtents& operator=(AnimationExtents&& rhs) = default;

        AnimationExtents(float xmin, float ymin, float zmin,
                         float xmax, float ymax, float zmax)
        {
            Set(xmin, ymin, zmin, xmax, ymax, zmax);
        }

        bool Valid() const { 
            return Min[0] <= Max[0] && Min[1] <= Max[1] && Min[2] <= Max[2]; 
        }
        
        void Set(float xmin, float ymin, float zmin,
                 float xmax, float ymax, float zmax)
        {
            Min[0] = xmin;
            Min[1] = ymin;
            Min[2] = zmin;
            Max[0] = xmax;
            Max[1] = ymax;
            Max[2] = zmax;
        }

        float Min[3];
        float Max[3];
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

        BakerMesh(const float* vb, const uint8_t* ib, const TriangleAttributes* attribs, size_t numVerts, size_t numTris)
            : BakerMesh(DefaultVertexReader(vb), DefaultIndexReader(ib), DefaultAttributeReader(attribs), numVerts, numTris)
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

        template< class Index_T >
        static IndexReader DefaultIndexReader(const Index_T* ib)
        {
            return
                [ib](uint32_t* out, const uint32_t* triIndices, size_t numTris)
                {
                    for (size_t i = 0; i < numTris; i++)
                    {
                        size_t idx = triIndices[i];
                        for (size_t j = 0; j < 3; j++)
                            out[3 * i + j] = (uint32_t) ib[3 * idx + j];
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

        void SetAnimationExtents(AnimationExtents extents)
        {
            m_extents = extents;
        }

        AnimationExtents GetAnimationExtents() const
        {
            return m_extents;
        }

    private:
        friend class Baker;

        VertexReader m_VertexReader;
        IndexReader m_IndexReader;
        AttributeReader m_AttributeReader;
        size_t m_numVerts = 0;
        size_t m_numTris = 0;        

        AnimationExtents m_extents;
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

    enum class QuantizationMode
    {
        TARGET_BIT_WIDTH,    // use 'targetBitWidth' field
        EXPLICIT_EXPONENT,   // use 'quantizationExponent' field
        DEFAULT = TARGET_BIT_WIDTH
    };

    struct Config
    {
        size_t clusterMaxFaces = 128;
        size_t clusterMaxVerts = 256;
         
        size_t blockMaxTris  = 64;
        size_t blockMaxVerts = 64;
        uint8_t blockForcedOffsetWidth[3] = { 0,0,0 }; // if any element is non-zero, bit-width of x,y,z offsets are forced to these values
                                                       //    The widths will be adjusted to a valid DGF bit allocation.
                                                       //    if the vertex data needs more precision, data loss may occur.
                                                       // Set all values to zero to allow the encoder to choose the offset widths
        std::ostream* outputStream = nullptr;
        bool printPerfData = false;                     
        bool validateClusters = false;                 // Enable sanity-checks for debugging cluster formation
        bool generateVertexTable = false;              // Generate the vertex remapping table
        bool generateClusterTable = false;             // Generate the cluster table
        bool generateTriangleRemap = false;            // Generate the triangle remapping table
        bool enableUserData = false;                   // Reserve space for a 'user-data' field in the blocks
        bool encoderRoundTripValidation = false;       // Enable sanity-checks for debugging DGFLib encoder        
        PackerMode packer = PackerMode::DEFAULT;

        QuantizationMode quantizationMode = QuantizationMode::DEFAULT;

        // Controls for QuantizationMode::TARGET_BIT_WIDTH

        size_t targetBitWidth = 16;               // target signed precision used to select quantization exponent based on geometry AABB
        bool quantizeForAnimation = false;       // if true, quantization is based on the length of the cluster diagonal
        float clusterDeformationPadding = 1.0f;  // Multiplier for cluster AABB extents during quantization, to account for deformation
                                                 //  This is best viewed as a percentage (e.g. 1.05f is 5%). Ignored when 'quantizeForAnimation' is false
                                                 

        // Control for QuantizationMode::EXPLICIT_EXPONENT
        // Scale vertices by 2^Exp and round results to integer
        //   Out of range results will wrap
        int8_t quantizationExponent = -15; 

        bool enableExponentAdjust  = true;           // Allow block exponents to vary to improve compression for pre-quantized input

        bool weldVertices = false; // Allow the encoder to combine different input vertices with identical positions 
                                   

        bool allowInternalThreading = false;

        // If enabled, vertices with different input positions which quantize to the same 
        //   output position will be combined, and any resulting degenerate triangles removed
        // 
        //  Vertices with the same input position will not be combined, since it is assumed that they indicate a vertex attribute boundary
        //   
        bool postQuantizationWelding = false;

        size_t sahBlockSize = 1;
    };
    
    class BakerOutput
    {
    public:

        BakerOutput(){}

        BakerOutput(
            std::vector<uint8_t> dgfBlocksIn,
            std::vector<TriangleRemapInfo> remapIn,
            std::vector<int32_t> ommIndicesIn,
            std::vector<uint32_t> vertexTable,
            std::vector<uint32_t> clusterTable, 
            int8_t quantFactor )
            : dgfBlocks(std::move(dgfBlocksIn)),
              triangleRemap(std::move(remapIn)),
              blockOMMIndices(std::move(ommIndicesIn)),
              vertexTable(std::move(vertexTable)),
              clusterTable(std::move(clusterTable)),
              quantizationFactor(quantFactor)
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
        // 
        //  If vertex welding is enabled, the block vertices may be mapped to a different vertex with the same position
        std::vector<uint32_t> vertexTable;

        // Table giving the index of the first DGF block for each cluster
        //    Size of table is the number of clusters
        std::vector<uint32_t> clusterTable;

        // Signed vertex quantization factor which was selected
        int8_t quantizationFactor = 0;
    };


    class ClusteredMesh
    {
    public:

        // Reads cluster vertices into an array of 3xNumVerts floats.   
        //   Args are:   cluster index, output pointer, max vert count
        //   Returns the number of vertices
        typedef std::function<size_t(size_t, float*, size_t)> ClusterVertexReader;
        
        // Reads cluster connectivity into an array of 3*NumTris indices.   
        //   Args are:   cluster index, output pointer, max triangle count
        //   Returns the number of triangles
        typedef std::function<size_t(size_t, uint8_t*, size_t)> ClusterIndexReader;

        // Reads triangle attributes. 
        //  Args are:  cluster index, output ptr, max triangle count
        //  Returns the number of triangles
        typedef std::function<size_t(size_t, DGFBaker::TriangleAttributes*,size_t)> ClusterAttributeReader;

        ClusteredMesh(
            size_t numClusters,
            ClusterVertexReader vertexReader,
            ClusterIndexReader indexReader,
            ClusterAttributeReader readAttributes ,
            AnimationExtents extents = {}
        )
            : m_readClusterVertices(vertexReader),
              m_readClusterIndices(indexReader),
              m_readClusterAttributes(readAttributes),  
              m_numClusters(numClusters),
              m_animExtents(extents)
        {
        }

        size_t GetNumClusters() const { return m_numClusters; }

        size_t ReadClusterVertices(size_t clusterIndex, float* output, size_t maxVerts) const
        {
            return m_readClusterVertices(clusterIndex, output, maxVerts);
        }

        size_t ReadClusterIndices(size_t clusterIndex, uint8_t* output, size_t maxTris) const
        {
            return m_readClusterIndices(clusterIndex, output, maxTris);
        }

        size_t ReadClusterTriangleAttributes(size_t clusterIndex, DGFBaker::TriangleAttributes* output, size_t maxTris) const
        {
            return m_readClusterAttributes(clusterIndex, output, maxTris);
        }

        size_t ReadClusterTriangles(size_t clusterIndex, uint8_t* output, DGFBaker::TriangleAttributes* attribs, size_t maxTris) const
        {
            m_readClusterIndices(clusterIndex, output, maxTris);
            return m_readClusterAttributes(clusterIndex, attribs, maxTris);
        }
        
        void SetAnimationExtents(AnimationExtents extents)
        {
            m_animExtents = extents;
        }

        AnimationExtents GetAnimationExtents() const
        {
            return m_animExtents;
        }

    private:

        ClusterVertexReader m_readClusterVertices;
        ClusterIndexReader m_readClusterIndices;
        ClusterAttributeReader m_readClusterAttributes;
        size_t m_numClusters;
        AnimationExtents m_animExtents;
    };

    class BakerOutputPreclustered
    {
    public:

        struct OutputCluster
        {
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
            std::vector<uint8_t> vertexTable;
        };

        BakerOutputPreclustered() = default;
        BakerOutputPreclustered(std::vector<OutputCluster> outputClusters, int8_t quantFactor)
            : clusters(std::move(outputClusters)),
            quantizationFactor(quantFactor)
        {
        }

        std::vector<OutputCluster> clusters;
        int8_t quantizationFactor;
    };
   

    struct BakerMeshCluster
    {
        std::vector<uint32_t> VertexIndices;  // index of each vert in the input mesh
        std::vector<uint8_t>  Topology;       // index of each triangle vert in 'VertexIndices' (3 per triangle)
        std::vector<uint32_t> PrimIDs;        // index of each triangle in the input mesh
    };

    class ClusteredBakerMesh : public ClusteredMesh
    {
    public:

        ClusteredBakerMesh(const BakerMesh& mesh, std::vector<BakerMeshCluster> clusters)
            : ClusteredMesh(clusters.size(),
                [this](size_t cluster, float* verts, size_t maxVerts) 
                { 
                    return VertexReader(cluster, verts, maxVerts); 
                },
                [this](size_t cluster, uint8_t* indices, size_t maxTris) 
                { 
                    return IndexReader(cluster,indices,maxTris); 
                },
                [this](size_t cluster, TriangleAttributes* attribs, size_t maxTris) 
                { 
                    return AttributeReader(cluster, attribs, maxTris); 
                },
                mesh.GetAnimationExtents()),
            m_mesh(mesh),
            m_clusters(std::move(clusters))
        {
        }

    private:

        size_t VertexReader(size_t clusterID, float* verts, size_t maxVerts)
        {
            const auto& cluster = m_clusters[clusterID];
            size_t numVerts = cluster.VertexIndices.size();
            if (numVerts > maxVerts)
                numVerts = maxVerts;

            m_mesh.ReadVertices(verts, cluster.VertexIndices.data(), numVerts);
            return numVerts;
        }

        size_t IndexReader(size_t clusterID, uint8_t* indices, size_t maxTris)
        {
            const auto& cluster = m_clusters[clusterID];
            size_t numTris = cluster.Topology.size()/3;
            if (numTris > maxTris)
                numTris = maxTris;
            memcpy(indices, cluster.Topology.data(), numTris*3);
            return numTris;
        }

        size_t AttributeReader(size_t clusterID, DGFBaker::TriangleAttributes* attribs, size_t maxTris)
        {
            const auto& cluster = m_clusters[clusterID];
            size_t numTris = cluster.PrimIDs.size();
            if (numTris > maxTris)
                numTris = maxTris;
            if (m_mesh.HasTriangleAttributes())
                m_mesh.ReadAttributes(attribs, cluster.PrimIDs.data(), numTris);
            return numTris;
        }

        BakerMesh m_mesh;
        std::vector<BakerMeshCluster> m_clusters;
    };

    // The Baker

    class Baker
    {
    public:
        Baker() : m_config() {};

        Baker(const Config& config);

        std::vector<BakerMeshCluster> BuildClusters( const BakerMesh& mesh );

        BakerOutput BakeDGF( const BakerMesh& mesh );

        BakerOutputPreclustered BakeDGFPreclustered( const ClusteredMesh& mesh );

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
        Validator(std::ostream& errorStream, bool vertexWelding=false)
            : m_errorStream(errorStream), m_vertexWelding(vertexWelding), m_postQuantWelding(false)
        {
        };

        Validator(const DGFBaker::Config& config)
            : m_errorStream(config.outputStream != nullptr ? *config.outputStream : std::cerr), 
              m_vertexWelding(config.weldVertices),
              m_postQuantWelding(config.postQuantizationWelding)
        {
        };

        bool ValidateDGF( const BakerMesh& input, const BakerOutput& output );
        bool ValidateDGFPreclustered(const ClusteredMesh& input, const BakerOutputPreclustered& output);
        bool ValidateDecodedMesh( const BakerMesh& input, const DecodedMesh& result, const std::vector<TriangleRemapInfo>& remap, int8_t quantizationFactor );

    protected:

        bool ValidateVertexTable(
            const BakerMesh& input,
            const std::vector<uint8_t>& dgfBlocks,
            const std::vector<DGFBaker::TriangleRemapInfo>& triangleRemap,
            const std::vector<uint32_t>& vertexTable );

        std::ostream& m_errorStream;
        bool m_vertexWelding;
        bool m_postQuantWelding;
    };


    // Generate a block-map data structure for mapping of primitive IDs to their block locations
    //  The input is an array of DGF blocks with ascending, tightly-packed primitive IDs
    std::vector<uint32_t> GenerateDGFBlockMap( const uint8_t* pBlocks, size_t numBlocks, size_t totalTris, size_t groupSize );

} 
