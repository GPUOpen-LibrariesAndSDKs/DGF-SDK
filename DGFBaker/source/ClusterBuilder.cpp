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

#include "ClusterBuilder.h"
// clang-format off

#include <span>
#include <algorithm>
#include <unordered_map>

#include "SIMD.h"

#include "DGFBaker.h"

namespace DGF
{

struct alignas(16) PrimRef
{
    float3 bbmin;
    uint32_t triangleIndex;
    float3 bbmax;
    uint8_t binIndex[3];
    uint8_t _pad;

    SIMDBox LoadBox()
    {
        return SIMDBox(bbmin, bbmax);
    }
};
static_assert(sizeof(PrimRef) == 32, "what??");


static void SortIndices( uint64_t* indices, size_t numIndices, size_t maxVertexIndex )
{
    RadixSort<uint64_t, 3 * DGFBaker::MAX_CLUSTER_FACES>(indices, numIndices, maxVertexIndex,
        [](uint64_t i)
        {
            return i >> 32;
        }
    );
}

static Cluster BuildCluster(PrimRef* primRefs, size_t numTris, const DGFBaker::BakerMesh& mesh)
{
    DGF_ASSERT(numTris <= DGFBaker::MAX_CLUSTER_FACES);

    static_assert(DGFBaker::MAX_CLUSTER_FACES <= 256 && DGFBaker::MAX_CLUSTER_VERTICES <= 256);

    // copy prim IDs
    Cluster outputCluster;
    outputCluster.PrimIDs.resize(numTris);
    outputCluster.Indices.resize(3 * numTris);
    for (size_t i = 0; i < numTris; i++)
        outputCluster.PrimIDs[i] = primRefs[i].triangleIndex;
    
    // read triangle indices
    uint32_t meshIndices[3 * DGFBaker::MAX_CLUSTER_FACES];
    mesh.ReadIndices(meshIndices, outputCluster.PrimIDs.data(), outputCluster.PrimIDs.size());

    uint32_t maxIndex = 0;
    uint64_t vertexKeys[3 * DGFBaker::MAX_CLUSTER_FACES];
    for (size_t i = 0; i < numTris; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            uint32_t idx = meshIndices[3 * i + j];
            maxIndex = std::max(idx, maxIndex);
            vertexKeys[3 * i + j] = ((uint64_t)(idx) << 32ull) + (i << 8) + j;
        }
    }

    // sort vertices to find unique ones
    SortIndices(vertexKeys, 3 * numTris, maxIndex);

    // build the local vertex and index buffer 
    size_t numIndices = 3 * numTris;
    for (size_t v = 0; v < numIndices; )
    {
        // copy the next unique vertex
        size_t v0 = v;
        size_t oldIndex = vertexKeys[v0] >> 32;
        size_t newIndex = outputCluster.Vertices.size();
        
        float3 vert;
        mesh.ReadVertex(&vert.x, (uint32_t) oldIndex);
        outputCluster.Vertices.push_back(vert);

        do
        {
            // scan run of references to this vertex and populate corresponding index buffer positions
            size_t vk = vertexKeys[v] & 0xffff;
            size_t i = (vk >> 8);
            size_t j = (vk & 3);
            outputCluster.Indices[3 * i + j] = (uint8_t)newIndex;
            v++;

        } while (v < numIndices && (vertexKeys[v] >> 32) == oldIndex);
    }

    return std::move(outputCluster);
}

static void PostProcessCluster( std::vector<Cluster>& clusters, Cluster& cluster, const DGFBaker::BakerMesh& mesh )
{
    if (!mesh.HasTriangleAttributes())
    {
        clusters.push_back(std::move(cluster));
    }
    else
    {
        DGFBaker::TriangleAttributes attribs[DGFBaker::MAX_CLUSTER_FACES];
        size_t numTris = cluster.Indices.size() / 3;
        mesh.ReadAttributes(attribs, cluster.PrimIDs.data(), cluster.PrimIDs.size());

        bool haveOMM = false;
        bool haveNonOMM = false;
        for (size_t i = 0; i < numTris; i++)
        {
            haveOMM = attribs[i].hasOMM && haveOMM;
            haveNonOMM = !attribs[i].hasOMM && haveNonOMM;
            if (haveOMM && haveNonOMM)
                break;
        }
    
        // split clusters which have a mixture of OMM and non-OMM triangles
        if( haveOMM && haveNonOMM )
        {
            Cluster ommCluster;
            Cluster nonOMMCluster;
            std::bitset<DGF::MAX_VERTS> OMMVerts;
            std::bitset<DGF::MAX_VERTS> nonOMMVerts;
            uint8_t ommVertIndex[DGF::MAX_VERTS];
            uint8_t nonOMMVertIndex[DGF::MAX_VERTS];
            uint8_t numOMMVerts = 0;
            uint8_t numNonOMMVerts = 0;
            for (size_t i = 0; i < numTris; i++)
            {
                if (attribs[i].hasOMM)
                {
                    for (size_t j = 0; j < 3; j++)
                    {
                        uint8_t idx = cluster.Indices[3 * i + j];
                        if (!OMMVerts.test(idx))
                        {
                            ommVertIndex[idx] = numOMMVerts++;
                            OMMVerts.set(idx);
                            ommCluster.Vertices.push_back(cluster.Vertices[idx]);
                        }
                        ommCluster.Indices.push_back(ommVertIndex[idx]);
                    }
                    ommCluster.PrimIDs.push_back(cluster.PrimIDs[i]);
                    ommCluster.Attributes.push_back(attribs[i]);
                }
                else
                {
                    for (size_t j = 0; j < 3; j++)
                    {
                        uint8_t idx = cluster.Indices[3 * i + j];
                        if (!nonOMMVerts.test(idx))
                        {
                            nonOMMVertIndex[idx] = numNonOMMVerts++;
                            nonOMMVerts.set(idx);
                            nonOMMCluster.Vertices.push_back(cluster.Vertices[idx]);
                        }
                        nonOMMCluster.Indices.push_back(nonOMMVertIndex[idx]);
                    }
                    nonOMMCluster.PrimIDs.push_back(cluster.PrimIDs[i]);
                    nonOMMCluster.Attributes.push_back(attribs[i]);
                }
            }

            DGF_ASSERT(ommCluster.PrimIDs.size() > 0 && nonOMMCluster.PrimIDs.size() > 0);  
            clusters.push_back(std::move(ommCluster));
            clusters.push_back(std::move(nonOMMCluster));
        }
        else
        {
            // all triangles have the same 'OMM-ness'
            cluster.Attributes.insert(cluster.Attributes.end(), &attribs[0], &attribs[numTris]);
            clusters.push_back(std::move(cluster));
        }
    }
}


void BuildClustersBinSAH(const DGFBaker::BakerMesh& mesh, std::vector<Cluster>& clusters, size_t maxVerts, size_t maxFaces)
{
    constexpr size_t NUM_BINS = 256;
    size_t numTris = mesh.GetTriangleCount();

    static_assert(NUM_BINS <= 256);

    std::vector< PrimRef > primRefs;

    // construct triangle AABBs and primID list
    AABB centroidBounds;
    primRefs.reserve(numTris);
    for (size_t i = 0; i < numTris; i++)
    {
        uint32_t idx[3];
        mesh.ReadIndices(idx, (uint32_t) i);

        // filter degenerates
        uint32_t a = idx[0];
        uint32_t b = idx[1];
        uint32_t c = idx[2];
        if (a == b || b == c || a == c)
            continue;

        float vertices[9];
        mesh.ReadVertices( vertices, idx, 3);
        static_assert(sizeof(float3) == 3 * sizeof(float), "The compiler did something unexpected");

        float3 v0 = float3(vertices[0], vertices[1], vertices[2]);
        float3 v1 = float3(vertices[3], vertices[4], vertices[5]);
        float3 v2 = float3(vertices[6], vertices[7], vertices[8]);
        float3 bbmin = min(v0, min(v1, v2));
        float3 bbmax = max(v0, max(v1, v2));
        primRefs.push_back({ bbmin,(uint32_t)i,bbmax,{0},0 });
        centroidBounds.Expand((bbmin + bbmax) * 0.5f);
    }

    numTris = primRefs.size();

    struct Subtree
    {
        AABB centroidBounds;
        uint32_t startPrim;
        uint32_t numPrims;
    };

    std::vector<Subtree> subtrees;
    Subtree root;
    root.centroidBounds = centroidBounds;
    root.startPrim = 0;
    root.numPrims = (uint32_t)numTris;
    subtrees.push_back(root);

    std::vector<Subtree> pendingClusters;

    constexpr float inf = std::numeric_limits<float>::infinity();

    while (!subtrees.empty())
    {
        Subtree tree = subtrees.back(); subtrees.pop_back();
        uint32_t numPrims = tree.numPrims;
        if (numPrims <= maxFaces)
        {
            // if face count is below the threshold, try to build a cluster
            Cluster cluster = BuildCluster(&primRefs[tree.startPrim], tree.numPrims, mesh);
            if (cluster.Vertices.size() <= maxVerts)
            {
                // keep the cluster if it meets the vertex limit
                //  otherwise, we need to split more
                PostProcessCluster(clusters, cluster, mesh);
                continue;
            }
        }

        PrimRef* subtreeRefs = primRefs.data() + tree.startPrim;
        centroidBounds = tree.centroidBounds;

        // special case where all triangles have identical centroids
        // binned-sah will fail to partition things in this case, so split prims in the middle
        float3 centroidDiag = centroidBounds.Max - centroidBounds.Min;
        if (centroidDiag.x == 0 && centroidDiag.y == 0 && centroidDiag.z == 0)
        {
            uint32_t numLeft = numPrims / 2;
            uint32_t numRight = numPrims - numLeft;
            DGF_ASSERT(numLeft > 0 && numRight > 0);
            subtrees.push_back({ centroidBounds, tree.startPrim, numLeft });
            subtrees.push_back({ centroidBounds, tree.startPrim + numLeft, numRight });
            continue;
        }

        SIMDVecf centroidScale = ((float)(NUM_BINS)) / (centroidDiag);
        SIMDVecf centroidBBMin = centroidBounds.Min;

        // initialize bins
        SIMDBox binBounds[3][NUM_BINS];
        uint32_t binCount[3][NUM_BINS] = { {0} };

        // place primitives into bins by centroid
        for (size_t i = 0; i < numPrims; i++)
        {
            PrimRef& ref = subtreeRefs[i];
            SIMDBox refBox = ref.LoadBox();
            SIMDVecf centroid = refBox.Centroid();
            SIMDVecf binIndex = (centroid - centroidBBMin) * centroidScale;
            binIndex = min(binIndex, (NUM_BINS - 1.0f));
            SIMDVeci binIndices = SIMDVeci(binIndex);
            uint32_t bx = binIndices.X();
            uint32_t by = binIndices.Y();
            uint32_t bz = binIndices.Z();

            binBounds[0][bx].Expand(refBox);
            binBounds[1][by].Expand(refBox);
            binBounds[2][bz].Expand(refBox);
            binCount[0][bx]++;
            binCount[1][by]++;
            binCount[2][bz]++;

            ref.binIndex[0] = bx;
            ref.binIndex[1] = by;
            ref.binIndex[2] = bz;
        }

        // select split
        uint32_t bestAxis = 0;
        uint32_t bestBin = NUM_BINS / 2;
        float bestSAH = inf;
        for (uint32_t a = 0; a < 3; a++)
        {
            // sweep left/right and compute per-bin LR area and counts
            SIMDBox leftBox = binBounds[a][0];
            SIMDBox rightBox = binBounds[a][NUM_BINS - 1];
            uint32_t count = binCount[a][0];

            float leftArea[NUM_BINS];
            float rightArea[NUM_BINS];
            uint32_t leftCount[NUM_BINS];
            for (size_t i = 1; i < NUM_BINS; i++)
            {
                leftArea[i - 1] = leftBox.HalfArea();
                rightArea[NUM_BINS - i - 1] = rightBox.HalfArea();
                leftCount[i - 1] = count;
                leftBox.Expand(binBounds[a][i]);
                rightBox.Expand(binBounds[a][NUM_BINS - i - 1]);
                count += binCount[a][i];
            }

            DGF_ASSERT(count == numPrims);

            // chboose the best bin to split on.. right-most bin is not considered
            for (uint32_t i = 0; i < NUM_BINS - 1; i++)
            {
                float al = leftArea[i];
                float ar = rightArea[i];
                uint32_t cl = leftCount[i];
                uint32_t cr = numPrims - cl;

                float sah = al * cl + ar * cr;
                if (sah < bestSAH)
                {
                    bestSAH = sah;
                    bestBin = i;
                    bestAxis = a;
                }
            }
        }

        // partition prim-refs
        uint32_t front = 0;
        uint32_t back = numPrims;
        SIMDBox leftCentroidBounds;
        SIMDBox rightCentroidBounds;
        for (size_t i = 0; i < numPrims; i++)
        {
            PrimRef& ref = subtreeRefs[front];
            SIMDBox refBox = ref.LoadBox();
            SIMDVecf centroid = refBox.Centroid();
            if (ref.binIndex[bestAxis] <= bestBin)
            {
                leftCentroidBounds.Expand(centroid);
                front++;
            }
            else
            {
                rightCentroidBounds.Expand(centroid);
                std::swap(subtreeRefs[front], subtreeRefs[--back]);
            }
        }

        uint32_t numLeft = front;
        uint32_t numRight = numPrims - front;
        DGF_ASSERT(numLeft > 0);
        DGF_ASSERT(numRight > 0);
        DGF_ASSERT(numLeft + numRight == numPrims);

        subtrees.push_back({ leftCentroidBounds.ToAABB(),  tree.startPrim, numLeft });
        subtrees.push_back({ rightCentroidBounds.ToAABB(), tree.startPrim + numLeft, numRight });
    }
}

bool ValidateClusters(const std::vector<Cluster>& clusters, const DGFBaker::BakerMesh& mesh, size_t maxVerts, size_t maxFaces )
{
    struct Triangle
    {
        bool operator!=(const Triangle& rhs) const {
            return v[0] != rhs.v[0] || v[1] != rhs.v[1] || v[2] != rhs.v[2] || ID != rhs.ID ||
                attribs != rhs.attribs;
        };
        bool operator<(const Triangle& rhs) const
        {
            if( ID != rhs.ID )
                return ID < rhs.ID;
            if (attribs != rhs.attribs)
                return attribs < rhs.attribs;

            for( size_t i=0; i<3; i++ )
                if( v[i] != rhs.v[i] )
                    return v[i] < rhs.v[i];

            return false;
        };

        float3 v[3];
        uint32_t ID;
        DGFBaker::TriangleAttributes attribs;
    };
    size_t numTris = mesh.GetTriangleCount();
    std::vector<Triangle> inputTris;
    inputTris.reserve(numTris);

    std::vector<Triangle> clusterTris;
    clusterTris.reserve(numTris);

    for (size_t i = 0; i < numTris; i++)
    {
        uint32_t idx[3];
        mesh.ReadIndices(idx, (uint32_t) i);

        DGFBaker::TriangleAttributes attribs;
        if (mesh.HasTriangleAttributes())
            mesh.ReadAttributes(&attribs, (uint32_t)i);

        float vertices[9];
        mesh.ReadVertices(vertices, idx, 3);

        inputTris.push_back(
            {
                {
                    { vertices[0], vertices[1], vertices[2] },
                    { vertices[3], vertices[4], vertices[5] },
                    { vertices[6], vertices[7], vertices[8] }
                },
                (uint32_t)(i) ,
                attribs
            } );
    }

    for (const auto& cluster : clusters)
    {
        if( cluster.Vertices.size() > maxVerts )
            return false;
        if (cluster.Indices.size() > maxFaces * 3)
            return false;

        for (size_t i = 0; i < cluster.Indices.size(); i += 3)
        {
            auto attributes = cluster.Attributes.empty() ? DGFBaker::TriangleAttributes() : cluster.Attributes[i / 3];
            clusterTris.push_back(
                {
                    {
                        cluster.Vertices[cluster.Indices[i + 0]],
                        cluster.Vertices[cluster.Indices[i + 1]],
                        cluster.Vertices[cluster.Indices[i + 2]]
                    },
                 cluster.PrimIDs[i/3] ,
                attributes
                }
            );
        }
    }

    if( inputTris.size() != clusterTris.size() )
        return false;

    std::sort( clusterTris.begin(), clusterTris.end() );
    std::sort( inputTris.begin(), inputTris.end() );

    for (size_t i = 0; i < clusterTris.size(); i++)
    {
        if (inputTris[i] != clusterTris[i])
        {
            return false;
        }
    }

    return true;
}


} // namespace DGF
