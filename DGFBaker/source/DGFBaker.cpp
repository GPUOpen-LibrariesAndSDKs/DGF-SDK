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
#include "../include/DGFBaker.h"

#include "../source/Cluster.h"
#include "../source/ClusterBuilder.h"
#include "../source/StripGeneration.h"
#include "../source/Quantization.h"
#include "../source/HPG24BlockPacker.h"
#include "../source/SAHBlockPacker.h"
#include "../source/DebugOutput.h"
#include "../source/Threading.h"

namespace DGFBaker
{

    static std::vector<uint32_t> GenerateVertexTable( 
        const uint8_t* blocks, size_t numBlocks,
        const LocalTriangleRemap* triangleRemap,
        const uint32_t* clusterPrimIDs,
        const BakerMesh& bakerMesh )
    {
        std::vector<uint32_t> vertexTable;
        size_t triBase = 0;
        for (size_t i = 0; i < numBlocks; i++)
        {
            const uint8_t* block = blocks + i*DGF::BLOCK_SIZE;
            size_t numTris = DGF::DecodeTriangleCount(block);

            // read original indices for each triangle in the block, in block order
            uint32_t vertIndices[3*DGF::MAX_TRIS];
            uint32_t triIndices[DGF::MAX_TRIS];
            for (size_t t = 0; t < numTris; t++)
                triIndices[t] = clusterPrimIDs[triangleRemap[triBase + t].InputPrimID];                
            
            bakerMesh.ReadIndices(vertIndices, triIndices, numTris);
            
            // rotate original indices to match block vertex order
            for (size_t t = 0; t < numTris; t++)
            {
                uint32_t idx[3];
                for (size_t k = 0; k < 3; k++)
                    idx[k] = vertIndices[3 * t + k];
                for (size_t k = 0; k < 3; k++)
                    vertIndices[3 * t + k] = idx[triangleRemap[triBase + t].IndexRotation[k]];
            }

            // decode the local topology from the block
            uint8_t localIndices[DGF::MAX_INDICES];
            size_t numVerts = DGF::DecodeTriangleList(localIndices, block);

            // iterate the block index buffer and find the corresponding global index for each local vertex
            uint64_t vertSeen = 0;
            uint32_t uniqueIndices[DGF::MAX_VERTS];
            size_t numUniqueIndices = 0;
            for (size_t v = 0; v < 3 * numTris; v++)
            {
                uint8_t localIndex = localIndices[v];
                uint64_t bit = (1ull << localIndex);

                bool firstUse = (vertSeen & bit) == 0;
                if (firstUse)
                {
                    uint32_t globalIndex = vertIndices[v];
                    uniqueIndices[numUniqueIndices++] = globalIndex;
                    vertSeen = vertSeen | bit;
                    if (numUniqueIndices == numVerts)
                        break;
                }
            }

            vertexTable.insert(vertexTable.end(), &uniqueIndices[0], &uniqueIndices[numUniqueIndices]);
            triBase += numTris;
        }

        return vertexTable;
    }
    


    static std::vector<uint8_t> GenerateClusterVertexTable(
        const std::vector<uint8_t>& blocksOut,
        const std::vector<LocalTriangleRemap>& localRemap,
        const uint8_t* inputIndices)
    {
        std::vector<uint8_t> vertexTable;
        size_t numBlocks = blocksOut.size() / DGF::BLOCK_SIZE;
        for (size_t b = 0; b < numBlocks; b++)
        {
            const uint8_t* block = blocksOut.data() + b*DGF::BLOCK_SIZE;
            size_t numTris = DGF::DecodeTriangleCount(block);
            size_t triBase = DGF::DecodePrimIDBase(block);
        
            // reconstruct the input index buffer for the block-ordered tris
            uint8_t vertIndices[DGF::MAX_TRIS * 3];
            for (size_t t = 0; t < numTris; t++)
            {
                for (size_t j = 0; j < 3; j++)
                {
                    size_t originalTriIndex  = localRemap[triBase+t].InputPrimID;
                    size_t originalVertIndex = localRemap[triBase+t].IndexRotation[j];
                    vertIndices[3 * t + j] = inputIndices[3 * originalTriIndex + originalVertIndex];
                }
            }

            // build table of unique verts from the block index buffer, ordered by first use
            // decode the local topology from the block
            uint8_t localIndices[DGF::MAX_INDICES];
            size_t numVerts = DGF::DecodeTriangleList(localIndices, block);

            // iterate the block index buffer and find the corresponding global index for each local vertex
            uint64_t vertSeen = 0;
            uint32_t uniqueIndices[DGF::MAX_VERTS];
            size_t numUniqueIndices = 0;
            for (size_t v = 0; v < 3 * numTris; v++)
            {
                uint8_t localIndex = localIndices[v];
                uint64_t bit = (1ull << localIndex);

                bool firstUse = (vertSeen & bit) == 0;
                if (firstUse)
                {
                    uint32_t globalIndex = vertIndices[v];
                    uniqueIndices[numUniqueIndices++] = globalIndex;
                    vertSeen = vertSeen | bit;
                    if (numUniqueIndices == numVerts)
                        break;
                }
            }

            vertexTable.insert(vertexTable.end(), &uniqueIndices[0], &uniqueIndices[numUniqueIndices]);
        }

        return vertexTable;
    }

    static int8_t PickExponent(const DGFBaker::Config& config, const ClusteredMesh& mesh)
    {
        if (config.quantizationMode == QuantizationMode::EXPLICIT_EXPONENT)
        {
            return config.quantizationExponent;
        }
        else if (config.quantizationMode == QuantizationMode::TARGET_BIT_WIDTH)
        {
            return ChooseQuantizationExponent(mesh,config);
        }
        else
        {
            if (config.outputStream)
                *config.outputStream << "Invalid quantization mode!";
            
            // pick something semi-plausible and carry on
            return -15;
        }
    }

    static int8_t DoExponentSelection(const DGFBaker::Config& config, const ClusteredMesh& mesh, PerformanceData& pd)
    {
        Timer t;
        int8_t exp = PickExponent(config, mesh);
        pd.ExponentSelect = t.Tick();
        return exp;
    }

    std::vector<BakerMeshCluster> Baker::BuildClusters(const BakerMesh& bakerMesh)
    {
        size_t maxVerts = m_config.clusterMaxVerts;
        size_t maxFaces = m_config.clusterMaxFaces;
        DGF_ASSERT(maxFaces <= DGFBaker::MAX_CLUSTER_FACES);
        DGF_ASSERT(maxVerts <= DGFBaker::MAX_CLUSTER_VERTICES);

        size_t sahBlockSize = m_config.sahBlockSize;
        std::vector<Cluster> clusters;
        if (m_config.allowInternalThreading)
            BuildClustersBinSAH_Parallel(bakerMesh, clusters, maxVerts, maxFaces, m_config.weldVertices, sahBlockSize);
        else
            BuildClustersBinSAH(bakerMesh, clusters, maxVerts, maxFaces, m_config.weldVertices, sahBlockSize);

        if (m_config.validateClusters)
            ValidateClusters(clusters, bakerMesh, maxVerts, maxFaces);
        return clusters;
    }

    static ClusteredMesh CreateClusteredMesh( 
        const std::vector<Cluster>& clusters,
        const BakerMesh& bakerMesh )
    {
        auto _VertReader =
            [&bakerMesh, &clusters](size_t clusterIndex, float* output, size_t maxVerts)
            {
                const Cluster& cluster = clusters[clusterIndex];
                size_t numVerts = cluster.VertexIndices.size();
                numVerts = std::min(numVerts, maxVerts);
                bakerMesh.ReadVertices(output, cluster.VertexIndices.data(), numVerts);
                return numVerts;
            };

        auto _IndexReader =
            [&clusters](size_t clusterIndex, uint8_t* output, size_t maxTris)
            {
                const Cluster& cluster = clusters[clusterIndex];
                size_t numTris = cluster.PrimIDs.size();
                numTris = std::min(maxTris, numTris);
                memcpy(output, cluster.Topology.data(), 3 * numTris);
                return numTris;
            };

        auto _AttribReader =
            [&clusters,&bakerMesh](size_t clusterIndex, TriangleAttributes* attribs, size_t maxTris)
            {
                const Cluster& cluster = clusters[clusterIndex];
                size_t numTris = cluster.PrimIDs.size();
                numTris = std::min(maxTris, numTris);
                if (bakerMesh.HasTriangleAttributes())
                    bakerMesh.ReadAttributes(attribs, cluster.PrimIDs.data(), numTris);
                else
                {
                    for (size_t i = 0; i < numTris; i++)
                        attribs[i] = TriangleAttributes();
                }
                return numTris;
            };

        size_t numClusters = clusters.size();
        return ClusteredMesh(numClusters, _VertReader, _IndexReader, _AttribReader);
    }

    Baker::Baker(const Config& config)
        : m_config(config)
    {
        m_config.clusterMaxVerts = std::min(m_config.clusterMaxVerts, DGFBaker::MAX_CLUSTER_VERTICES);
        m_config.clusterMaxFaces = std::min(m_config.clusterMaxFaces, DGFBaker::MAX_CLUSTER_FACES);

        if (m_config.clusterMaxFaces != config.clusterMaxFaces)
            if (m_config.outputStream)
                (*m_config.outputStream) << "clusterMaxFaces adjusted to " << (int) m_config.clusterMaxFaces << "\n";

        if (m_config.clusterMaxVerts != config.clusterMaxVerts)
            if (m_config.outputStream)
                (*m_config.outputStream) << "clusterMaxVerts adjusted to " << (int) m_config.clusterMaxVerts << "\n";

        uint8_t x_bits = m_config.blockForcedOffsetWidth[0];
        uint8_t y_bits = m_config.blockForcedOffsetWidth[1];
        uint8_t z_bits = m_config.blockForcedOffsetWidth[2];
        size_t maxBits = std::max(x_bits, std::max(y_bits, z_bits));
        if (maxBits > 0)
        {
            DGF::AdjustVertexWidths(m_config.blockForcedOffsetWidth, config.blockForcedOffsetWidth);
            if (x_bits != m_config.blockForcedOffsetWidth[0] ||
                y_bits != m_config.blockForcedOffsetWidth[1] ||
                z_bits != m_config.blockForcedOffsetWidth[2])
            {
                if (m_config.outputStream)
                    (*m_config.outputStream) << "forced vertex widths adjusted to: "
                        << (int)m_config.blockForcedOffsetWidth[0] << ","
                        << (int)m_config.blockForcedOffsetWidth[1] << ","
                        << (int)m_config.blockForcedOffsetWidth[2] << "\n";                   
            }
        }

        // clamp SAH block size to prevent division by zero
        if (m_config.sahBlockSize < 1)
        {
            m_config.sahBlockSize = 1; 
            if (m_config.outputStream)
                (*m_config.outputStream) << "SAH block size of 0 is illegal.  Overriding to 1\n";
        }
    }

    static size_t RemoveIndexDegens( 
        uint8_t* degenMap,
        size_t numTris, 
        uint8_t* indices )
    {
        size_t n = 0;
        for (size_t i = 0; i < numTris; i++)
        {
            uint8_t a = indices[3 * i + 0];
            uint8_t b = indices[3 * i + 1];
            uint8_t c = indices[3 * i + 2];
            if (a == b || b == c || a == c)
                continue;

            degenMap[n] = (uint8_t)(i);
            indices[3 * n + 0] = a;
            indices[3 * n + 1] = b;
            indices[3 * n + 2] = c;
            n++;
        }
        return n;       
    }

    template< class CompareFN >
    static size_t MergeClusterVertices(
        uint8_t* vertexRemap,
        uint8_t* uniqueVerts,
        const uint8_t* indices,
        CompareFN IndexCompare,
        size_t numVerts,
        size_t numTris)    
    {
        DGF_ASSERT(numVerts <= 256);

        // map each vertex onto itself
        for (size_t i = 0; i < numVerts; i++)
            vertexRemap[i] = (uint8_t)i;

        // enumerate mesh edges
        // for each edge:
        //     if edge is collapsible
        //         - delete higher numbered vert, map onto lower vert
        //            this is effectively an edge collapse
        //
        for (size_t t = 0; t < numTris; t++)
        {
            for (size_t k = 0; k < 3; k++)
            {
                // iterate mesh edges
                uint8_t idxA = indices[3 * t + k];
                uint8_t idxB = indices[3 * t + (k + 1) % 3];

                // apply earlier vertex replacements
                while (vertexRemap[idxA] != idxA)
                    idxA = vertexRemap[idxA];
                while (vertexRemap[idxB] != idxB)
                    idxB = vertexRemap[idxB];

                if (idxA != idxB && IndexCompare(idxA,idxB))
                {
                    if (idxA > idxB)
                        std::swap(idxA, idxB);
                    vertexRemap[idxB] = idxA;
                }
            }
        }

        // flatten chains of vertex replacements until all replaced verts are mapped onto one non-replaced one
        for (size_t i = 0; i < numVerts; i++)
        {
            uint8_t target = vertexRemap[i];
            if (target != i)
            {
                do
                {
                    target = vertexRemap[target];
                } while (target != vertexRemap[target]);
                vertexRemap[i] = target;
            }
        }

        // compact the vertices and note new position of each input vertex
        size_t numOutputVerts = 0;
        for (size_t i = 0; i < numVerts; i++)
        {
            size_t targetVert = vertexRemap[i];
            if (targetVert == i)
            {
                // keep verts which are mapped onto themselves
                // and record the new positions of these vertices
                vertexRemap[i] = (uint8_t)numOutputVerts;
                uniqueVerts[numOutputVerts++] = (uint8_t)i;

            }
            else
            {
                // record the new position of the vertex we were mapped onto
                vertexRemap[i] = vertexRemap[targetVert];
            }
        }

        return numOutputVerts;
    }

    
    static size_t WeldVerticesPostQuant(
        uint8_t* vertexRemap,
        uint8_t* uniqueVerts,
        const uint8_t* indices,
        const float3* V,
        const int3* Q,
        size_t numVerts,
        size_t numTris )
    {
        auto _CompareQuantizedVerts = 
            [V, Q](uint8_t idxA, uint8_t idxB) 
            { 
                return Q[idxA] == Q[idxB] && V[idxA] != V[idxB]; 
            };

        return MergeClusterVertices(vertexRemap, uniqueVerts, indices, _CompareQuantizedVerts, numVerts, numTris);
    }

    static size_t WeldVerticesPreQuant(
        uint8_t* vertexRemap,
        uint8_t* uniqueVerts,
        const uint8_t* indices,
        const float3* V,
        size_t numVerts,
        size_t numTris
    )
    {
        auto _CompareInputVerts =
            [V](uint8_t idxA, uint8_t idxB)
            {
                return V[idxA] == V[idxB];
            };

        return MergeClusterVertices(vertexRemap, uniqueVerts, indices, _CompareInputVerts, numVerts, numTris);
    }

    static void DoPostQuantWeld(Cluster& cluster, int8_t exponent, const DGFBaker::BakerMesh& bakerMesh)
    {
        size_t numInputTris = cluster.PrimIDs.size();
        size_t numInputVerts = cluster.VertexIndices.size();

        float3 V[MAX_CLUSTER_VERTICES];
        bakerMesh.ReadVertices((float*)V, cluster.VertexIndices.data(), cluster.VertexIndices.size());

        int3 Q[MAX_CLUSTER_VERTICES];
        QuantizeVerts(Q, V, numInputVerts, exponent);

        uint8_t vertexRemap[MAX_CLUSTER_VERTICES];
        uint8_t uniqueVerts[MAX_CLUSTER_VERTICES];
        size_t numOutputVerts = WeldVerticesPostQuant(
            vertexRemap,
            uniqueVerts,
            cluster.Topology.data(),
            V, Q, numInputVerts, numInputTris);

        // renumber indices
        for (size_t i = 0; i < cluster.Topology.size(); i++)
            cluster.Topology[i] = vertexRemap[cluster.Topology[i]];

        // delete resulting degens
        uint8_t degenMap[MAX_CLUSTER_FACES];
        size_t numOutputTris = RemoveIndexDegens(degenMap, numInputTris, cluster.Topology.data());

        // compact vertex and prim ID lists
        for (size_t i = 0; i < numOutputTris; i++)
            cluster.PrimIDs[i] = cluster.PrimIDs[degenMap[i]];
        for (size_t i = 0; i < numOutputVerts; i++)
            cluster.VertexIndices[i] = cluster.VertexIndices[uniqueVerts[i]];

        // shrink the cluster
        cluster.PrimIDs.resize(numOutputTris);
        cluster.Topology.resize(3 * numOutputTris);
        cluster.VertexIndices.resize(numOutputVerts);
    }


    static void PackBlocks(
        std::vector<BakerMeshCluster>& clusters,
        const DGFBaker::BakerMesh& bakerMesh,
        const DGFBaker::Config& config,
        std::vector<uint32_t>& clusterTable,
        std::vector<uint32_t>& vertexTable,
        std::vector<uint8_t>& dgfBlocksOut,
        std::vector<DGFBaker::TriangleRemapInfo>& remapOut,
        std::vector<int32_t>& ommIndicesOut,
        int8_t exponent,
        PerformanceData& perfData
    )
    {
        Timer packTimer;

        size_t numClusters = clusters.size();
        if (config.generateClusterTable)
            clusterTable.resize(numClusters);

        if (config.postQuantizationWelding)
        {
            auto _RemeshCluster = [&clusters, &bakerMesh, exponent](size_t clusterIndex)
                {
                    Cluster& cluster = clusters[clusterIndex];
                    DoPostQuantWeld(cluster, exponent, bakerMesh);
                };

            if (config.allowInternalThreading)
            {
                Threading::ParallelFor(numClusters,
                    _RemeshCluster
                );
            }
            else
            {
                for (size_t i = 0; i < numClusters; i++)
                    _RemeshCluster(i);
            }
        }


        std::vector<uint32_t> clusterPrimIDBase(numClusters);
        {
            uint32_t primIDBase = 0;
            for (size_t i = 0; i < numClusters; i++)
            {
                clusterPrimIDBase[i] = primIDBase;
                primIDBase += (uint32_t)clusters[i].PrimIDs.size();
            }
            if (config.generateTriangleRemap)
                remapOut.resize(primIDBase);
        }

        std::vector<PackClusterResult> packResults(numClusters);
        
        std::vector<std::vector<uint32_t>> vertexTables;
        if (config.generateVertexTable)
            vertexTables.resize(numClusters);


        auto _ProcessCluster = [&](size_t clusterIndex)
            {
                Cluster& cluster = clusters[clusterIndex];

                DGFBaker::TriangleAttributes attributes[DGFBaker::MAX_CLUSTER_FACES];
                if (bakerMesh.HasTriangleAttributes())
                    bakerMesh.ReadAttributes(attributes, cluster.PrimIDs.data(), cluster.PrimIDs.size());

                float3 vertices[DGFBaker::MAX_CLUSTER_VERTICES];
                bakerMesh.ReadVertices((float*)vertices, cluster.VertexIndices.data(), cluster.VertexIndices.size());

                int3 QuantizedVerts[DGFBaker::MAX_CLUSTER_VERTICES];
                QuantizeVerts(QuantizedVerts, vertices, cluster.VertexIndices.size(), exponent);

                uint32_t primIDBase = clusterPrimIDBase[clusterIndex];

                if (cluster.PrimIDs.size() == 0)
                    return;

                PackClusterInputs clusterData;
                clusterData.Attributes = attributes;
                clusterData.Indices = cluster.Topology.data();
                clusterData.Verts = vertices;
                clusterData.numTris = cluster.PrimIDs.size();
                clusterData.numVerts = cluster.VertexIndices.size();;
                clusterData.exponent = exponent;
                clusterData.primIDBase = primIDBase;
                clusterData.QuantizedVerts = QuantizedVerts;

                // pack the cluster
                switch (config.packer)
                {
                case DGFBaker::PackerMode::HPG24:
                    packResults[clusterIndex] = PackClusterHPG24(clusterData, config);
                    break;
                case DGFBaker::PackerMode::SAH:
                    packResults[clusterIndex] = PackClusterSAH(clusterData, config);
                    break;
                }

                // copy the remapping information, replace cluster-local indices with top-level ones
                PackClusterResult& packResult = packResults[clusterIndex];
                if (config.generateTriangleRemap)
                {
                    size_t primIndex = primIDBase;
                    for (const LocalTriangleRemap& localRemap : packResult.localRemap)
                    {
                        DGFBaker::TriangleRemapInfo info;
                        info.InputPrimIndex = cluster.PrimIDs[localRemap.InputPrimID];
                        for (size_t i = 0; i < 3; i++)
                            info.IndexRotation[i] = localRemap.IndexRotation[i];

                        remapOut[primIndex++] = info;
                    }
                }

                // produce vertex remapping table if requested
                if (config.generateVertexTable)
                {
                    auto table = GenerateVertexTable(
                        packResult.dgfBlocks.data(), packResult.dgfBlocks.size() / DGF::BLOCK_SIZE,
                        packResult.localRemap.data(), cluster.PrimIDs.data(), bakerMesh);

                    vertexTables[clusterIndex] = std::move(table);
                }
            };

        if (config.allowInternalThreading)
        {
            Threading::ParallelFor(numClusters,
                _ProcessCluster
            );
        }
        else
        {
            for (size_t i = 0; i < numClusters; i++)
                _ProcessCluster(i);
        }

        perfData.TrianglePacking = packTimer.Tick();

        Timer cleanupTimer;

        // pre-allocate output arrays
        size_t dgfBlockSize = 0;
        size_t ommIndicesSize = 0;
        size_t vertexTableSize = 0;
        for (size_t clusterIndex = 0; clusterIndex < numClusters; clusterIndex++)
        {
            // update cluster table
            if (config.generateClusterTable)
                clusterTable[clusterIndex] = (uint32_t)(dgfBlockSize / DGF::BLOCK_SIZE);
            
            // compute size of consolidated output arrays
            dgfBlockSize += packResults[clusterIndex].dgfBlocks.size();
            ommIndicesSize += packResults[clusterIndex].outputOMMIndices.size();

            if(config.generateVertexTable)
                vertexTableSize += vertexTables[clusterIndex].size();
        }

        dgfBlocksOut.reserve(dgfBlockSize);
        ommIndicesOut.reserve(ommIndicesSize);
        vertexTable.reserve(vertexTableSize);

        // combine cluster-level arrays into one output array
        size_t numPrims = 0;
        for (size_t clusterIndex = 0; clusterIndex < numClusters; clusterIndex++)
        {
            Cluster& cluster = clusters[clusterIndex];

            PackClusterResult& packResult = packResults[clusterIndex];

            // concatenate blocks
            dgfBlocksOut.insert(dgfBlocksOut.end(), packResult.dgfBlocks.begin(), packResult.dgfBlocks.end());
            
            // store OMM indices
            ommIndicesOut.insert(ommIndicesOut.end(), packResult.outputOMMIndices.begin(), packResult.outputOMMIndices.end());

            // concatenate vertex tables
            if (config.generateVertexTable)
                vertexTable.insert(vertexTable.end(), vertexTables[clusterIndex].begin(), vertexTables[clusterIndex].end());            
        }

        perfData.Cleanup = cleanupTimer.Tick();
    }


    BakerOutput Baker::BakeDGF( const BakerMesh& bakerMesh )
    {
        Timer totalTime;
        PerformanceData perfData;

        std::vector<DGFBaker::TriangleRemapInfo> remapOut;
        std::vector<int32_t> ommIndicesOut;
        std::vector<uint8_t> dgfBlocksOut;
        std::vector<BakerMeshCluster> clusters;

        {
            Timer t;
            clusters = BuildClusters(bakerMesh);
            perfData.Clustering += t.Tick();
        }

        // corner case:  all triangles are index-degenerate
        if( clusters.empty() )
            return BakerOutput();

        std::vector<uint32_t> clusterTable;
        std::vector<uint32_t> vertexTable;

        auto clusteredMesh = CreateClusteredMesh(clusters, bakerMesh);

        int8_t exponent = DoExponentSelection(m_config, clusteredMesh,perfData);

        PackBlocks(clusters, bakerMesh, m_config, clusterTable, vertexTable, dgfBlocksOut, remapOut, ommIndicesOut, exponent,perfData);


        // report perf data if requested
        perfData.TotalTime = totalTime.Tick();
        if (m_config.printPerfData && m_config.outputStream)
            PrintPerfData(*m_config.outputStream, perfData, m_config );
        
        return BakerOutput( std::move(dgfBlocksOut), std::move(remapOut), std::move(ommIndicesOut), std::move(vertexTable), std::move(clusterTable), exponent );
    }

    DGFBaker::BakerOutputPreclustered Baker::BakeDGFPreclustered(const ClusteredMesh& mesh)
    {
        Timer totalTime;
        PerformanceData perfData;

        size_t numClusters = mesh.GetNumClusters();
        int8_t exponent = DoExponentSelection(m_config, mesh, perfData);

        Timer packingTime;

        std::vector<DGFBaker::BakerOutputPreclustered::OutputCluster> outputClusters;
        outputClusters.resize(numClusters);

        auto _DoPacking = 
            [mesh,&outputClusters,exponent,this](size_t idx)
            {
                DGFBaker::TriangleAttributes attributes[DGFBaker::MAX_CLUSTER_FACES];
                uint8_t indices[DGFBaker::MAX_CLUSTER_FACES * 3];
                size_t numTris = mesh.ReadClusterTriangles(idx, indices, attributes, DGFBaker::MAX_CLUSTER_FACES);

                float3 vertices[DGFBaker::MAX_CLUSTER_VERTICES];
                size_t numVerts = mesh.ReadClusterVertices(idx, (float*)vertices, DGFBaker::MAX_CLUSTER_VERTICES);

                DGF_ASSERT(numTris <= MAX_CLUSTER_FACES);
                DGF_ASSERT(numVerts <= MAX_CLUSTER_VERTICES);

                // combine vertices with the same input position
                if (m_config.weldVertices)
                {
                    uint8_t vertexRemap[MAX_CLUSTER_VERTICES];
                    uint8_t uniqueVerts[MAX_CLUSTER_VERTICES];
                    numVerts = WeldVerticesPreQuant(vertexRemap, uniqueVerts, indices, vertices, numVerts, numTris);

                    for (size_t i = 0; i < numVerts; i++)
                        vertices[i] = vertices[uniqueVerts[i]];

                    for (size_t i = 0; i < numTris * 3; i++)
                        indices[i] = vertexRemap[indices[i]];
                }

                
                auto& outputCluster = outputClusters[idx];

                int3 QuantizedVerts[MAX_CLUSTER_VERTICES];
                QuantizeVerts(QuantizedVerts, vertices, numVerts, exponent);

                if (m_config.postQuantizationWelding)
                {
                    // weld vertices again after quantization
                    uint8_t vertexRemap[MAX_CLUSTER_VERTICES];
                    uint8_t uniqueVerts[MAX_CLUSTER_VERTICES];
                    numVerts = WeldVerticesPostQuant(vertexRemap, uniqueVerts, indices, vertices, QuantizedVerts, numVerts, numTris);

                    for (size_t i = 0; i < numVerts; i++)
                    {
                        size_t inputIndex = uniqueVerts[i];
                        vertices[i] = vertices[inputIndex];
                        QuantizedVerts[i] = QuantizedVerts[inputIndex];
                    }

                    for (size_t i = 0; i < numTris * 3; i++)
                        indices[i] = vertexRemap[indices[i]];
                }

                // scrub degens.
                //  If we're welding vertices, then this is a necessary cleanup step.
                //  If we're not, we still need to do it since we don't know whether or not the cluster builder did it for us
                //   (the user may have used a clusterizer that leaves them in)
                uint8_t degenMap[MAX_CLUSTER_FACES];
                numTris = RemoveIndexDegens(degenMap, numTris, indices);

                // remove degens from attribute array
                for (size_t i = 0; i < numTris; i++)
                    attributes[i] = attributes[degenMap[i]];

                // skip packing for clusters where all triangles are index-degenerate
                if (numTris == 0)
                    return;

                PackClusterInputs clusterData;
                clusterData.Attributes     = attributes;
                clusterData.Indices        = indices;
                clusterData.Verts          = vertices;
                clusterData.numTris        = numTris;
                clusterData.numVerts       = numVerts;
                clusterData.exponent       = exponent;
                clusterData.QuantizedVerts = QuantizedVerts;
                clusterData.primIDBase     = 0;

                
                // pack the cluster
                PackClusterResult packResult;
                switch (m_config.packer)
                {
                case DGFBaker::PackerMode::HPG24:
                    packResult = DGFBaker::PackClusterHPG24(clusterData, m_config);
                    break;
                case DGFBaker::PackerMode::SAH:
                    packResult = DGFBaker::PackClusterSAH(clusterData, m_config);
                    break;
                }

                outputCluster.dgfBlocks = std::move(packResult.dgfBlocks);
                outputCluster.blockOMMIndices = std::move(packResult.outputOMMIndices);

                if (m_config.generateTriangleRemap)
                {
                    // translate 'remap' structures 
                    outputCluster.triangleRemap.reserve(packResult.localRemap.size());
                    for (auto& localRemap : packResult.localRemap)
                    {
                        DGFBaker::TriangleRemapInfo outputRemap;
                        outputRemap.InputPrimIndex = degenMap[localRemap.InputPrimID];
                        for (size_t j = 0; j < 3; j++)
                            outputRemap.IndexRotation[j] = localRemap.IndexRotation[j];
                        outputCluster.triangleRemap.push_back(outputRemap);
                    }
                }

                // build vertex table remapping table
                if (m_config.generateVertexTable)
                {
                    // if we re-numbered the cluster vertices we need to get the original vertex numberings back
                    //    in order to generate a correct vertex table.
                    //  To do that, we re-read the original index buffer and eliminate any degens that we removed
                    //    before the blocks were encoded
                    if (m_config.weldVertices || m_config.postQuantizationWelding)
                    {
                        mesh.ReadClusterIndices(idx, indices, DGFBaker::MAX_CLUSTER_FACES);
                        for (size_t i = 0; i < numTris; i++)
                        {
                            size_t inputTriIndex = degenMap[i];
                            for (size_t j = 0; j < 3; j++)
                                indices[3 * i + j] = indices[3 * inputTriIndex + j];
                        }
                    }
                    outputCluster.vertexTable = GenerateClusterVertexTable(outputCluster.dgfBlocks, packResult.localRemap, indices);
                }
            };

        if (m_config.allowInternalThreading)
            Threading::ParallelFor(numClusters, _DoPacking);
        else
        {
            for (size_t idx = 0; idx < numClusters; idx++)
                _DoPacking(idx);            
        }

        perfData.TrianglePacking = packingTime.Tick();

        // report perf data if requested        
        perfData.TotalTime = totalTime.Tick();       
        if (m_config.printPerfData && m_config.outputStream)
            PrintPerfData(*m_config.outputStream, perfData, m_config);

        return BakerOutputPreclustered(std::move(outputClusters),exponent);
    }

    DecodedMesh DecodeDGF(const BakerOutput& output)
    {
        return DecodeDGF(output.dgfBlocks.data(), output.dgfBlocks.size(), output.blockOMMIndices.data());
    }

    DecodedMesh DecodeDGF(const uint8_t* blocks, size_t sizeInBytes, const int32_t* pOMMIndices)    
    {
        const auto dgfBlocks = std::span<const uint8_t>(blocks, sizeInBytes);
        size_t numBlocks = dgfBlocks.size() / DGF::BLOCK_SIZE;

        // read the total vertex and triangle count
        size_t totalTris=0;
        size_t totalVerts=0;
        for ( size_t i=0; i<numBlocks; i++ )
        {
            totalTris  += DGF::DecodeTriangleCount( dgfBlocks.data() + i*DGF::BLOCK_SIZE );
            totalVerts += DGF::DecodeVertexCount( dgfBlocks.data() + i*DGF::BLOCK_SIZE );
        }
         
        // allocate memory
        std::vector<uint32_t> ib( totalTris * 3 );
        std::vector<float> vb( totalVerts * 3 );
        std::vector<TriangleAttributes> attribs( totalTris );
        std::vector<int8_t> quant( totalVerts );
        std::vector<uint32_t> triIndex(totalTris);

        // decode the blocks
        uint32_t vertexBase = 0;
        size_t ommIndexBase = 0;
        size_t primBase = 0;
        for ( size_t block=0; block<numBlocks; block++ )
        {
            const uint8_t* pBlock = dgfBlocks.data() + block * DGF::BLOCK_SIZE ;
            
            DGF::MetaData meta;
            DGF::DecodeMetaData( &meta, pBlock );

            uint8_t tmpIndices[DGF::MAX_INDICES];
            DGF::DecodeTriangleList( tmpIndices, pBlock );

            uint8_t opaqueFlags[DGF::MAX_TRIS];
            uint32_t geomID[DGF::MAX_TRIS];
            DGF::DecodeGeomIDs( geomID, opaqueFlags, pBlock );
            
            DGF::OffsetVert offsetVerts[DGF::MAX_VERTS];
            DGF::FloatVert floatVerts[DGF::MAX_VERTS];
            DGF::DecodeOffsetVerts( meta.numVerts, offsetVerts, pBlock );
            DGF::ConvertOffsetsToFloat( meta.numVerts, floatVerts, offsetVerts, meta );
            
            uint8_t triangleOMMIndices[DGF::MAX_TRIS];
            DGF::DecodeTriangleOMMIndices( triangleOMMIndices, pBlock );
            
            // copy indices
            for (size_t i = 0; i < 3*meta.numTris; i++)
                ib[3*primBase+i] = tmpIndices[i] + vertexBase;

            // copy vertices
            for (size_t i = 0; i < meta.numVerts; i++)
            {
                for( size_t k=0; k<3; k++ )
                    vb[3*(vertexBase + i) + k] = floatVerts[i].xyz[k];
            }

            // build per-vertex quantization factor array
            for( size_t i=0; i<meta.numVerts; i++ )
                quant[vertexBase+i] = static_cast<int8_t>(((int)meta.exponent) - DGF::EXPONENT_BIAS);

            // copy triangle IDs
            for( uint32_t i=0; i<meta.numTris; i++ )
                triIndex[primBase + i ] = meta.primIDBase + i;

            // copy triangle attributes
            for (size_t i = 0; i < meta.numTris; i++) 
            {        
                auto& attribsOut = attribs[primBase + i];
                uint8_t ommDescriptorIndex = triangleOMMIndices[i];
                attribsOut.geomID = geomID[i];
                attribsOut.opaque = opaqueFlags[i];
                attribsOut.hasOMM = meta.numOMMDescriptors > 0;
                if( attribsOut.hasOMM )
                    attribsOut.ommIndex = (int32_t) pOMMIndices[ommIndexBase + ommDescriptorIndex];
            }

            ommIndexBase += meta.numOMMDescriptors;
            vertexBase += meta.numVerts;
            primBase += meta.numTris;
        }

        return DecodedMesh( std::move(vb), std::move(ib), std::move(quant), std::move(attribs), std::move(triIndex) );
    }

    bool Validator::ValidateDGF( const BakerMesh& input, const BakerOutput& output )
    {
        const std::vector<uint8_t>& dgfBlocks = output.dgfBlocks;
        const std::vector<TriangleRemapInfo>& remap = output.triangleRemap;

        if ( dgfBlocks.size() % DGF::BLOCK_SIZE != 0 )
        {
            m_errorStream << "Input DGF block array is not a multiple of block size!\n";
            return false;
        }

        DecodedMesh decoded = DecodeDGF( output );
        
        if (!ValidateDecodedMesh(input, decoded, remap, output.quantizationFactor ))
            return false;

        return ValidateVertexTable(input, output.dgfBlocks, output.triangleRemap, output.vertexTable);
    }

    bool Validator::ValidateDGFPreclustered(const ClusteredMesh& input, const BakerOutputPreclustered& output)
    {
        if (input.GetNumClusters() != output.clusters.size())
        {
            m_errorStream << "Cluster count is wrong!\n";
            return false;
        }

        size_t numClusters = input.GetNumClusters();
        for (size_t c = 0; c < numClusters; c++)
        {
            // read the input cluster
            float3 verts[DGFBaker::MAX_CLUSTER_VERTICES];
            uint8_t indices[DGFBaker::MAX_CLUSTER_FACES * 3];
            DGFBaker::TriangleAttributes attribs[DGFBaker::MAX_CLUSTER_FACES];
            size_t numVerts = input.ReadClusterVertices(c, (float*)verts, DGFBaker::MAX_CLUSTER_VERTICES);
            size_t numTris = input.ReadClusterTriangles(c, indices, attribs, DGFBaker::MAX_CLUSTER_FACES);
         
            DGFBaker::BakerMesh clusterMesh((float*)verts, indices, attribs, numVerts, numTris);

            // decode the output
            const uint8_t* blocks = output.clusters[c].dgfBlocks.data();
            size_t blockSize = output.clusters[c].dgfBlocks.size();
            DGFBaker::DecodedMesh decoded = DecodeDGF(blocks, blockSize, output.clusters[c].blockOMMIndices.data());

            // call the base validator
            if (!ValidateDecodedMesh(clusterMesh, decoded, output.clusters[c].triangleRemap, output.quantizationFactor))
            {
                m_errorStream << "Errors found in cluster " << c << "\n";
                return false;
            }

            // validate vertex table
            std::vector<uint32_t> dwordTable;
            for (uint8_t idx : output.clusters[c].vertexTable)
                dwordTable.push_back(idx);

            if (!ValidateVertexTable(clusterMesh, output.clusters[c].dgfBlocks, output.clusters[c].triangleRemap, dwordTable))
            {
                m_errorStream << "Vertex table error in cluster " << c << "\n";
                return false;
            }            
        }

        return true;
    }

    bool Validator::ValidateDecodedMesh( const BakerMesh& input, const DecodedMesh& decoded, const std::vector<TriangleRemapInfo>& remap, int8_t quantizationFactor )
    {
        if (remap.size() > input.GetTriangleCount())
        {
            m_errorStream << "Remap table is too large!\n";
            return false;
        }

        size_t numInputTris = input.GetTriangleCount();
        size_t numOutputTris = decoded.GetTriangleCount();

        std::vector<bool> inputTriSeen(numInputTris);

        for (size_t i = 0; i < numOutputTris; i++)
        {
            const TriangleRemapInfo& triRemap = remap[i];

            if (decoded.GetPrimIDs()[i] != i)
            {
                m_errorStream << "Output prim IDs are misnumbered!\n";
                return false;
            }

            for (size_t j = 0; j < 3; j++)
            {
                // verify that index rotation values are sane
                size_t idx = triRemap.IndexRotation[j];
                if (idx >= 3)
                {
                    m_errorStream << "Output triangle " << i << " bad index rotation value in remap table\n";
                    return false;
                }

                // verify that all 3 vertices are present in the rotation table
                // and that the vertex order is a valid winding-preserving rotation
                size_t expected_next = (idx + 1) % 3;
                size_t actual_next = triRemap.IndexRotation[(j + 1) % 3];
                if (expected_next != actual_next)
                {
                    m_errorStream << "Output triangle " << i << " bad index rotation value in remap table\n";
                    return false;
                }
            }

            size_t inputIndex = triRemap.InputPrimIndex;

            // verify that remap prim indices are sane
            if (inputIndex >= input.GetTriangleCount())
            {
                m_errorStream << "Ouptut triangle " << i << " bad prim index in remap table\n";
                return false;
            }

            // verify that triangles are referenced only once
            if (inputTriSeen[inputIndex])
            {
                m_errorStream << "Input triangle " << inputIndex << " seen more than once in output remap table\n";
                return false;
            }

            inputTriSeen[inputIndex] = true;

            // verify that attributes match
            TriangleAttributes inputAttribs;
            input.ReadAttributes(&inputAttribs, triRemap.InputPrimIndex);

            TriangleAttributes outputAttribs = decoded.GetAttributeBuffer()[i];
            if (outputAttribs.geomID != inputAttribs.geomID ||
                outputAttribs.hasOMM != inputAttribs.hasOMM ||
                outputAttribs.opaque != inputAttribs.opaque ||
                (outputAttribs.ommIndex != inputAttribs.ommIndex && inputAttribs.hasOMM))
            {
                m_errorStream << "Output triangle: " << i << " attribute mismatch\n";
                return false;
            }

            // read the index buffer and rotate the indices
            uint32_t originalIndices[3];
            input.ReadIndices(originalIndices, triRemap.InputPrimIndex);
            uint32_t rotatedIndices[3];
            for (size_t j = 0; j < 3; j++)
                rotatedIndices[j] = originalIndices[triRemap.IndexRotation[j]];

            for (size_t j = 0; j < 3; j++)
            {
                // verify that the output VB/IB is identical to the input VB indexed with the remapped IB           
                uint32_t inputIndex = rotatedIndices[j];
                uint32_t outputIndex = decoded.GetIndexBuffer()[3 * i + j];

                // check for out of range indices
                if (outputIndex >= decoded.GetVertexCount())
                {
                    m_errorStream << "Output triangle: " << i << " topology index out of bounds!\n";
                    return false;
                }

                // verify that the output VB/IB produces the same verts as the input VB indexed with the remapped IB     
                //   and quantized according to what we found in the blocks      
                float3 inVert, outVert;
                input.ReadVertex(&inVert.x, inputIndex);
                for (size_t k = 0; k < 3; k++)
                    outVert[k] = decoded.GetVertexBuffer()[3 * outputIndex + k];

                // compare quantized results
                int8_t exponent = decoded.GetVertexQuantizationFactorBuffer()[outputIndex];
                int3 Qin = QuantizeVert(inVert, exponent);
                int3 QOut = QuantizeVert(outVert, exponent);
                for (size_t k = 0; k < 3; k++)
                {
                    if (Qin[k] != QOut[k])
                    {
                        m_errorStream << "Vertex mismatch, output triangle: " << i << "\n"
                            << " Input position: " << triRemap.InputPrimIndex << "\n"
                            << " Input: " << inVert[0] << "," << inVert[1] << "," << inVert[2] << "\n"
                            << " Result: " << outVert[0] << "," << outVert[1] << "," << outVert[2] << "\n"
                            << " Exp: " << (int)exponent << "\n";
                        return false;
                    }
                }
            }

            // make sure that post-quant degen filtering works as advertised
            // do not perform this check unless vertex welding is also enabled
            //   because there are corner cases around split vertices that are difficult to validate
            //   on a per-triangle basis in isolation
            if (m_postQuantWelding && m_vertexWelding)
            {
                float3 inVerts[3];
                input.ReadVertices((float*)inVerts, rotatedIndices, 3);

                int3 QVerts[3];
                for (size_t j = 0; j < 3; j++)
                {
                    uint32_t outputIndex = decoded.GetIndexBuffer()[3 * i + j];
                    int8_t exponent = decoded.GetVertexQuantizationFactorBuffer()[outputIndex];
                    QVerts[j] = QuantizeVert(inVerts[j], exponent);
                }

                bool postQuantDegen =
                    QVerts[0] == QVerts[1] ||
                    QVerts[1] == QVerts[2] ||
                    QVerts[2] == QVerts[0];
                
                if (postQuantDegen)
                {
                    m_errorStream << "Post-quant degen triangle not removed." 
                          << " Input idx: " << inputIndex 
                          << " Output idx: " << i << "\n";
                    return false;
                }
            }
        }

        // if any triangles were not seen, verify that they are degenerates
        for (size_t i = 0; i < numInputTris; i++)
        {
            if (!inputTriSeen[i])
            {
                uint32_t indices[3];
                input.ReadIndices(indices, (uint32_t)i);

                uint32_t a = indices[0];
                uint32_t b = indices[1];
                uint32_t c = indices[2];

                bool isIndexDegen = (a == b || b == c || a == c);
                if (!isIndexDegen)
                {
                    static_assert(sizeof(float3) == 3 * sizeof(float), "uh-oh");
                    
                    float3 verts[3];
                    input.ReadVertices( &verts[0].x, indices, 3);

                    // check if we legally removed a position degen because of vertex welding
                    bool isVertexDegen = (verts[0] == verts[1]) || (verts[1] == verts[2]) || (verts[2] == verts[0]);
                    if (!isVertexDegen || (isVertexDegen && !m_vertexWelding))
                    {
                        // check if we legally removed a post-quantization degen
                        // NOTE:  We use the highest-precision exponent in the model for this check
                        //    This is correct for the current encoder behavior, but if we ever start using variable quantization factors,
                        //      then we need more information to figure this out
                        bool postQuantDegenRemovalValid = false;
                        if (m_postQuantWelding)
                        {
                            int3 Q[3];
                            QuantizeVerts(Q, verts, 3, quantizationFactor);

                            if ((Q[0] == Q[1]) || (Q[1] == Q[2]) || (Q[2] == Q[0]))
                            {
                                postQuantDegenRemovalValid = true;
                            }
                        }
                        
                        if (!postQuantDegenRemovalValid)
                        {
                            m_errorStream << "Input triangle " << i << " missing from output\n";
                            return false;
                        }
                    }
                }
            }
        }

        return true;
    }

    bool Validator::ValidateVertexTable(
        const BakerMesh& input, 
        const std::vector<uint8_t>& dgfBlocks,
        const std::vector<DGFBaker::TriangleRemapInfo>& triangleRemap,
        const std::vector<uint32_t>& vertexTable )
    {
        if (triangleRemap.empty() || vertexTable.empty())
            return true;

        const DGFBaker::TriangleRemapInfo* pRemap = triangleRemap.data();
        const uint32_t* pVertexTable = vertexTable.data();

        size_t totalBlockVerts = 0;
        size_t numBlocks = dgfBlocks.size() / DGF::BLOCK_SIZE;
        for (size_t b = 0; b < numBlocks; b++)
        {
            const uint8_t* pBlock = dgfBlocks.data() + DGF::BLOCK_SIZE * b;
            size_t numVerts = DGF::DecodeVertexCount(pBlock);
            size_t numTris = DGF::DecodeTriangleCount(pBlock);

            // decode triangle connectivity
            uint8_t indices[DGF::MAX_INDICES];
            DGF::DecodeTriangleList(indices, pBlock);

            size_t maxVertexIndex = input.GetVertexCount();

            DGF::MetaData meta;
            DGF::DecodeMetaData(&meta, pBlock);
            int8_t quantExponent = static_cast<int8_t>(meta.exponent - DGF::EXPONENT_BIAS);

            for (size_t i = 0; i < numTris; i++)
            {
                // read original triangle indices
                uint32_t originalIndices[3];
                input.ReadIndices(originalIndices, pRemap[i].InputPrimIndex);

                // rotate out to block vertex order and compare with original indices
                for (size_t j = 0; j < 3; j++)
                {
                    uint32_t vertexTableIndex = pVertexTable[indices[3 * i + j]];
                    if (vertexTableIndex > maxVertexIndex)
                    {
                        m_errorStream << "Block " << b << " Triangle " << i << " Vertex " << j << " vertex table entry is out of range\n";
                        return false;
                    }

                    uint32_t originalIndex = originalIndices[pRemap[i].IndexRotation[j]];
                    if ( originalIndex != vertexTableIndex)
                    {
                        if (m_vertexWelding || m_postQuantWelding)
                        {
                            float3 A;
                            float3 B;
                            input.ReadVertex((float*)&A, originalIndex);
                            input.ReadVertex((float*)&B, vertexTableIndex);
                            static_assert(sizeof(float3) == 3 * sizeof(float));

                            // for vertex welding, indices are allowed to mismatch
                            // as long as input positions are identical
                            bool inputVertsMatch = (A == B);
                            if( m_vertexWelding && inputVertsMatch )
                                continue;

                            // for post-quant degen removal, indices are allowed to mismatch
                            //  if the quantized positions are the same and the input positions aren't
                            int3 Qa = QuantizeVert(A, quantExponent);
                            int3 Qb = QuantizeVert(B, quantExponent);
                            bool quantVertsMatch = (Qa == Qb);
                            if (m_postQuantWelding && (quantVertsMatch && !inputVertsMatch))
                                continue;
                        }
                      
                        m_errorStream << "Block " << b << " Triangle " << i << " Vertex " << j << " vertex table entry is wrong\n";
                        return false;                        
                    }
                }
            }

            totalBlockVerts += numVerts;
            pVertexTable += numVerts;
            pRemap += numTris;

            if (totalBlockVerts > vertexTable.size())
            {
                m_errorStream << "Vertex table is too small";
                return false;
            }
        }

        if (totalBlockVerts < vertexTable.size())
        {
            m_errorStream << "Vertex table is too large";
            return false;
        }

        return true;
    }

    PackerMode PackerModeFromString(const char* str)
    {
        if (strcmp(str, "HPG24") == 0)
        {
            return PackerMode::HPG24;
        }
        else if (strcmp(str, "SAH") == 0)
        {
            return PackerMode::SAH;
        }
        else
        {
            return PackerMode::DEFAULT;
        }
    }

    const char* PackerModeToString(PackerMode mode)
    {
        switch (mode)
        {
        case PackerMode::HPG24:     return "HPG24";
        case PackerMode::SAH:       return "SAH";
        default:                    return "????";
        }
    }
    

    std::vector<uint32_t> GenerateDGFBlockMap(const uint8_t* pBlocks, size_t numBlocks, size_t totalTris, size_t groupSize)
    {
        DGF_ASSERT( groupSize % 32 == 0 && groupSize >= 32 );

        std::vector<uint32_t> blockMap;
        blockMap.reserve(1 + (groupSize / 32));

        uint32_t blockIndex = 0;
        uint32_t positionInBlock = 0;
        
        for (size_t i = 0; i < totalTris; i += groupSize) 
        {
            DGF_ASSERT(blockIndex < (1 << 24));
            DGF_ASSERT(positionInBlock < DGF::MAX_TRIS);

            const uint8_t* pBlock = pBlocks + blockIndex * DGF::BLOCK_SIZE;
            uint32_t       numTris = (uint32_t)DGF::DecodeTriangleCount(pBlock);

            // store the block index and start position for the first triangle in this group
            uint32_t entryX = (blockIndex)+(positionInBlock << 24);
            blockMap.push_back(entryX);

            // reserve space for the end-of-block bits
            for (size_t j = 0; j < groupSize; j += 32)
                blockMap.push_back(0);

            // search for block boundaries within the group
            uint32_t* eobBits = &blockMap[blockMap.size() - (groupSize / 32)];
            for (size_t j = 0; j < groupSize; j++ ) 
            {                
                if (positionInBlock == (numTris - 1)) 
                {
                    // move to next block, and set this triangle's 'end-of-block' bit
                    blockIndex++;
                    positionInBlock = 0;
                    eobBits[j / 32] |= (1 << (j % 32));

                    // stop early if we're out of blocks... remaining bits don't matter
                    if (blockIndex == numBlocks)
                        break;

                    // read triangle count for next block
                    numTris = (uint32_t)DGF::DecodeTriangleCount(pBlocks + blockIndex * DGF::BLOCK_SIZE);
                }
                else 
                {
                    // stay in the same block
                    positionInBlock++;
                }                
            }
        }
        
        return blockMap;
    }

}
