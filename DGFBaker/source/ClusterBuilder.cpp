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

#include "Threading.h"

namespace DGFBaker
{

struct alignas(16) PrimRef
{
    float3 bbmin;
    uint32_t triangleIndex;
    float3 bbmax;
    uint8_t binIndex[3];
    uint8_t invalid;

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

static void WeldVertices(Cluster& cluster, const uint16_t* tmpIndices, const DGFBaker::BakerMesh& mesh )
{
    // sort and de-duplicate vertices by position
    struct TmpVert
    {
        float3 v;
        uint32_t oldIndexInCluster;
        uint32_t indexInMesh;
        inline bool operator<(const TmpVert& rhs) const { return v < rhs.v;}
    };

    TmpVert tmpVerts[3*DGFBaker::MAX_CLUSTER_FACES];
    size_t numVerts = cluster.VertexIndices.size();
    for (size_t i = 0; i < numVerts; i++)
    {
        tmpVerts[i].oldIndexInCluster = (uint32_t)i;
        tmpVerts[i].indexInMesh = cluster.VertexIndices[i];
        mesh.ReadVertex((float*)&tmpVerts[i].v, tmpVerts[i].indexInMesh);
    }

    std::sort(&tmpVerts[0], &tmpVerts[numVerts]);

    // map all duplicated verts onto the same original vert
    std::array<uint8_t, 3*DGFBaker::MAX_CLUSTER_FACES> indexRemap;
    size_t n = 0;
    size_t i = 0;
    do
    {
        // store a new unique vertex
        size_t newIndex = n;
        cluster.VertexIndices[n++] = tmpVerts[i].indexInMesh;

        // map all copies of this vertex onto the stored one
        size_t i0 = i;
        do
        {
            indexRemap[tmpVerts[i++].oldIndexInCluster] = (uint8_t)newIndex;
        } while (i < numVerts && !(tmpVerts[i0] < tmpVerts[i]));
        
    } while (i < numVerts);

    cluster.VertexIndices.resize(n);

    size_t numIndices = cluster.Topology.size();
    for (size_t i = 0; i < numIndices; i++)
        cluster.Topology[i] = indexRemap[tmpIndices[i]];

}

static Cluster BuildCluster(PrimRef* primRefs, size_t numTris, const DGFBaker::BakerMesh& mesh, bool weldVerts)
{
    DGF_ASSERT(numTris <= DGFBaker::MAX_CLUSTER_FACES);

    static_assert(DGFBaker::MAX_CLUSTER_FACES <= 256 && DGFBaker::MAX_CLUSTER_VERTICES <= 256);

    // copy prim IDs
    Cluster outputCluster;
    outputCluster.PrimIDs.resize(numTris);
    outputCluster.Topology.resize(3 * numTris);
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

    if (weldVerts)
    {
        // build the vertex buffer, and a temporary index buffer for welding
        size_t numIndices = 3 * numTris;
        uint16_t tmpIndices[3 * DGFBaker::MAX_CLUSTER_FACES];

        for (size_t v = 0; v < numIndices; )
        {
            // copy the next unique vertex
            size_t v0 = v;
            size_t oldIndex = vertexKeys[v0] >> 32;
            size_t newIndex = outputCluster.VertexIndices.size();

            outputCluster.VertexIndices.push_back((uint32_t)oldIndex);

            do
            {
                // scan run of references to this vertex and populate corresponding index buffer positions
                size_t vk = vertexKeys[v] & 0xffff;
                size_t i = (vk >> 8);
                size_t j = (vk & 3);
                tmpIndices[3 * i + j] = (uint16_t)newIndex;
                v++;

            } while (v < numIndices && (vertexKeys[v] >> 32) == oldIndex);
        }

        // combine verts with unique positions
        WeldVertices(outputCluster, tmpIndices, mesh);
    }
    else
    {
        // build the local vertex and index buffer 
        size_t numIndices = 3 * numTris;
        for (size_t v = 0; v < numIndices; )
        {
            // copy the next unique vertex
            size_t v0 = v;
            size_t oldIndex = vertexKeys[v0] >> 32;
            size_t newIndex = outputCluster.VertexIndices.size();

            outputCluster.VertexIndices.push_back((uint32_t)oldIndex);

            do
            {
                // scan run of references to this vertex and populate corresponding index buffer positions
                size_t vk = vertexKeys[v] & 0xffff;
                size_t i = (vk >> 8);
                size_t j = (vk & 3);
                outputCluster.Topology[3 * i + j] = (uint8_t)newIndex;
                v++;

            } while (v < numIndices && (vertexKeys[v] >> 32) == oldIndex);
        }
    }

    return outputCluster;
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
        size_t numTris = cluster.Topology.size() / 3;
        mesh.ReadAttributes(attribs, cluster.PrimIDs.data(), cluster.PrimIDs.size());

        bool haveOMM = false;
        bool haveNonOMM = false;
        for (size_t i = 0; i < numTris; i++)
        {
            haveOMM    = haveOMM    ||  attribs[i].hasOMM;
            haveNonOMM = haveNonOMM || !attribs[i].hasOMM;
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
                        uint8_t idx = cluster.Topology[3 * i + j];
                        if (!OMMVerts.test(idx))
                        {
                            ommVertIndex[idx] = numOMMVerts++;
                            OMMVerts.set(idx);
                            ommCluster.VertexIndices.push_back(cluster.VertexIndices[idx]);
                        }
                        ommCluster.Topology.push_back(ommVertIndex[idx]);
                    }
                    ommCluster.PrimIDs.push_back(cluster.PrimIDs[i]);
                }
                else
                {
                    for (size_t j = 0; j < 3; j++)
                    {
                        uint8_t idx = cluster.Topology[3 * i + j];
                        if (!nonOMMVerts.test(idx))
                        {
                            nonOMMVertIndex[idx] = numNonOMMVerts++;
                            nonOMMVerts.set(idx);
                            nonOMMCluster.VertexIndices.push_back(cluster.VertexIndices[idx]);
                        }
                        nonOMMCluster.Topology.push_back(nonOMMVertIndex[idx]);
                    }
                    nonOMMCluster.PrimIDs.push_back(cluster.PrimIDs[i]);
                }
            }

            DGF_ASSERT(ommCluster.PrimIDs.size() > 0 && nonOMMCluster.PrimIDs.size() > 0);  
            clusters.push_back(std::move(ommCluster));
            clusters.push_back(std::move(nonOMMCluster));
        }
        else
        {
            // all triangles have the same 'OMM-ness'
            clusters.push_back(std::move(cluster));
        }
    }
}

struct Split
{
    uint32_t bestAxis ;
    uint32_t bestBin;
    float bestSAH;
};

template< int NUM_BINS >
struct Bins
{
    SIMDBox binBounds[3][NUM_BINS];
    uint32_t binCount[3][NUM_BINS] = { {0} };

    void InsertPrimRef(PrimRef& ref, SIMDVecf centroidScale, SIMDVecf centroidBBMin )
    {
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

    void MergeWith(const Bins<NUM_BINS>& rhs)
    {
        for (size_t i = 0; i < NUM_BINS; i++)
        {
            binBounds[0][i].Expand(rhs.binBounds[0][i]);
            binBounds[1][i].Expand(rhs.binBounds[1][i]);
            binBounds[2][i].Expand(rhs.binBounds[2][i]);
        }
        for (size_t i = 0; i < NUM_BINS; i++)
        {
            binCount[0][i] += rhs.binCount[0][i];
            binCount[1][i] += rhs.binCount[1][i];
            binCount[2][i] += rhs.binCount[2][i];
        }
    }

    Split SelectSplit( uint32_t numPrims, size_t sahBlockSize )
    {
        uint32_t bestAxis = 0;
        uint32_t bestBin = NUM_BINS / 2;
        float bestSAH = std::numeric_limits<float>::infinity();
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

                float sah = static_cast<float>(ceil(cl / sahBlockSize) * al + ceil(cr / sahBlockSize) * ar);

                if (sah < bestSAH)
                {
                    bestSAH = sah;
                    bestBin = i;
                    bestAxis = a;
                }
            }
        }

        return { bestAxis,bestBin,bestSAH };
    }
};


struct Subtree
{
    AABB centroidBounds;    
    PrimRef* primRefs;
    uint32_t numPrims;
};

static void BuildSubtreeSequential( 
    const DGFBaker::BakerMesh& mesh, 
    size_t maxFaces, 
    size_t maxVerts, 
    bool weldVerts,
    size_t sahBlockSize,
    Subtree tree, 
    std::vector<Cluster>& clusters )
{
    std::vector<Subtree> subtrees;
    subtrees.push_back(tree);

    while (!subtrees.empty())
    {
        Subtree tree = subtrees.back(); subtrees.pop_back();
        PrimRef* primRefs = tree.primRefs;
        Subtree left, right;
        constexpr size_t NUM_BINS = 256;
        static_assert(NUM_BINS <= 256);

        uint32_t numPrims = tree.numPrims;
        if (numPrims <= maxFaces)
        {
            // if face count is below the threshold, try to build a cluster
            Cluster cluster = BuildCluster(primRefs, tree.numPrims, mesh, weldVerts);
            if (cluster.VertexIndices.size() <= maxVerts)
            {
                // keep the cluster if it meets the vertex limit
                //  otherwise, we need to split more
                PostProcessCluster(clusters, cluster, mesh);
                continue;
            }
        }

        PrimRef* subtreeRefs = primRefs;
        AABB centroidBounds = tree.centroidBounds;

        // special case where all triangles have identical centroids
        // binned-sah will fail to partition things in this case, so split prims in the middle
        float3 centroidDiag = centroidBounds.Max - centroidBounds.Min;
        if (centroidDiag.x == 0 && centroidDiag.y == 0 && centroidDiag.z == 0)
        {
            uint32_t numLeft = numPrims / 2;
            uint32_t numRight = numPrims - numLeft;
            DGF_ASSERT(numLeft > 0 && numRight > 0);
            subtrees.push_back({ centroidBounds, primRefs, numLeft });
            subtrees.push_back({ centroidBounds, primRefs + numLeft, numRight });
            continue;
        }

        SIMDVecf centroidScale = ((float)(NUM_BINS)) / (centroidDiag);
        SIMDVecf centroidBBMin = centroidBounds.Min;

        // initialize bins
        Bins<NUM_BINS> bins;

        // place primitives into bins by centroid
        for (size_t i = 0; i < numPrims; i++)
            bins.InsertPrimRef(subtreeRefs[i], centroidScale, centroidBBMin);

        // select split       
        Split split = bins.SelectSplit(numPrims, sahBlockSize);

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
            if (ref.binIndex[split.bestAxis] <= split.bestBin)
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
        subtrees.push_back({ leftCentroidBounds.ToAABB(),  primRefs, numLeft });
        subtrees.push_back({ rightCentroidBounds.ToAABB(), primRefs+ numLeft, numRight });
    }
}


void BuildClustersBinSAH(const DGFBaker::BakerMesh& mesh, std::vector<Cluster>& clusters, size_t maxVerts, size_t maxFaces, bool weldVerts, size_t sahBlockSize)
{
    size_t numTris = mesh.GetTriangleCount();
    
    std::vector< PrimRef > primRefs;

    // construct triangle AABBs and primID list
    AABB centroidBounds;
    primRefs.reserve(numTris);
    for (size_t i = 0; i < numTris; i++)
    {
        uint32_t idx[3];
        mesh.ReadIndices(idx, (uint32_t) i);

        // filter index-degenerates
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

        // filter position degenerates if vertex welding is requested
        //   they will turn into index-degenerates
        if (weldVerts && ((v0 ==v1) || (v1==v2) || (v2 == v0)) )
            continue;
        
        float3 bbmin = min(v0, min(v1, v2));
        float3 bbmax = max(v0, max(v1, v2));
        primRefs.push_back({ bbmin,(uint32_t)i,bbmax,{0},0 });
        centroidBounds.Expand((bbmin + bbmax) * 0.5f);
    }

    if (primRefs.empty())
        return;

    std::vector<Subtree> subtrees;
    Subtree root;
    root.centroidBounds = centroidBounds;
    root.primRefs = primRefs.data();
    root.numPrims = (uint32_t)primRefs.size();
   
    BuildSubtreeSequential(mesh, maxFaces, maxVerts, weldVerts, sahBlockSize, root, clusters);
}



class ParallelSubtree
{
public:
    Subtree tree;
    std::vector<Cluster> clusters;
    ParallelSubtree* children = nullptr;
};

static void BuildSubtreeParallel(
    const DGFBaker::BakerMesh& mesh,
    ParallelSubtree* root,
    size_t maxVerts,
    size_t maxFaces,
    bool weldVerts,
    size_t sahBlockSize)
{
    Subtree & tree = root->tree;
    uint32_t numPrims = tree.numPrims;
    PrimRef* primRefs = tree.primRefs;
    
    constexpr size_t SERIAL_THRESHOLD = 32768;
    constexpr size_t NUM_BINS = 256;

    if (numPrims <= maxFaces)
    {
        // if face count is below the threshold, try to build a cluster
        Cluster cluster = BuildCluster(primRefs, tree.numPrims, mesh, weldVerts);
        if (cluster.VertexIndices.size() <= maxVerts)
        {
            // keep the cluster if it meets the vertex limit
            //  otherwise, we need to split more
            PostProcessCluster(root->clusters, cluster, mesh);
            return;
        }
    }
    else if (numPrims <= SERIAL_THRESHOLD)
    {
        BuildSubtreeSequential(mesh, maxFaces, maxVerts, weldVerts, sahBlockSize, tree, root->clusters);
        return;
    }

    AABB centroidBounds = tree.centroidBounds;

    // special case where all triangles have identical centroids
    // binned-sah will fail to partition things in this case, so split prims in the middle
    float3 centroidDiag = centroidBounds.Max - centroidBounds.Min;
    if (centroidDiag.x == 0 && centroidDiag.y == 0 && centroidDiag.z == 0)
    {
        BuildSubtreeSequential(mesh, maxFaces, maxVerts, weldVerts, sahBlockSize, tree, root->clusters);
        return;
    }

    // choose a split
    Split split;
    {        
        SIMDVecf centroidScale = ((float)(NUM_BINS)) / (centroidDiag);
        SIMDVecf centroidBBMin = centroidBounds.Min;

        Bins<NUM_BINS> bins = Threading::ParallelReduce(
            tree.numPrims,
            [tree, centroidScale, centroidBBMin](size_t begin, size_t end)
            {
                Bins<NUM_BINS> bins;
                for (size_t i = begin; i < end; i++)
                    bins.InsertPrimRef(tree.primRefs[i], centroidScale, centroidBBMin);
                return bins;
            },
            [](Bins<NUM_BINS> a, Bins<NUM_BINS> b)
            {
                a.MergeWith(b);
                return a;
            }
        );

        // merge bins and select split
        split = bins.SelectSplit(tree.numPrims, sahBlockSize);
    }

    {
        // partition prim-refs and compute l/r centroid bounds at the same time
        // it is faster to do this serially.  A parallel partition on the primref array might be faster still,
        // but it would be difficult to ensure identical results between threaded and non-threaded paths
        uint32_t front = 0;
        uint32_t back = numPrims;
        SIMDBox leftCentroidBounds;
        SIMDBox rightCentroidBounds;
        for (size_t i = 0; i < numPrims; i++)
        {
            PrimRef& ref = tree.primRefs[front];
            SIMDBox refBox = ref.LoadBox();
            SIMDVecf centroid = refBox.Centroid();
            if (ref.binIndex[split.bestAxis] <= split.bestBin)
            {
                leftCentroidBounds.Expand(centroid);
                front++;
            }
            else
            {
                rightCentroidBounds.Expand(centroid);
                std::swap(tree.primRefs[front], tree.primRefs[--back]);
            }
        }

        uint32_t numLeft = front;
        uint32_t numRight = numPrims - front;
        DGF_ASSERT(numLeft > 0);
        DGF_ASSERT(numRight > 0);
        DGF_ASSERT(numLeft + numRight == numPrims);
        Subtree left  = { leftCentroidBounds.ToAABB(),  primRefs, numLeft };
        Subtree right = { rightCentroidBounds.ToAABB(), primRefs + numLeft, numRight };

        // go wide on subtrees (parallel for)
        root->children = new ParallelSubtree[2];
        root->children[0].tree = left;
        root->children[1].tree = right;

        Threading::ParallelFor(2,
            [root,&mesh,maxVerts,maxFaces,weldVerts,sahBlockSize](size_t childIndex)
            {
                BuildSubtreeParallel(mesh, &root->children[childIndex], maxVerts, maxFaces, weldVerts, sahBlockSize);
            }
        );
    }

    
}


void BuildClustersBinSAH_Parallel(
    const DGFBaker::BakerMesh& mesh,
    std::vector<Cluster>& clusters,
    size_t maxVerts,
    size_t maxFaces,
    bool weldVertices,
    size_t sahBlockSize)
{
    // TODO: preprocess primrefs in parallel
    size_t numPrims = mesh.GetTriangleCount();

    std::vector<PrimRef> primRefs(numPrims);
    
    AABB centroidBounds;

    centroidBounds = Threading::ParallelReduce( numPrims,
        [&primRefs,&mesh, weldVertices](size_t start, size_t end)
        {
            AABB centroidBounds;
            for (size_t i = start; i < end; i++)
            {
                primRefs[i].invalid = 1; // mark primref invalid

                uint32_t idx[3];
                mesh.ReadIndices(idx, (uint32_t)i);

                // filter index-degenerates
                uint32_t a = idx[0];
                uint32_t b = idx[1];
                uint32_t c = idx[2];
                if (a == b || b == c || a == c)
                    continue;

                float vertices[9];
                mesh.ReadVertices(vertices, idx, 3);
                static_assert(sizeof(float3) == 3 * sizeof(float), "The compiler did something unexpected");
                float3 v0 = float3(vertices[0], vertices[1], vertices[2]);
                float3 v1 = float3(vertices[3], vertices[4], vertices[5]);
                float3 v2 = float3(vertices[6], vertices[7], vertices[8]);

                // filter position degenerates if vertex welding is requested
                //   they will turn into index-degenerates
                if (weldVertices && ((v0 == v1) || (v1 == v2) || (v2 == v0)))
                    continue;

                float3 bbmin = min(v0, min(v1, v2));
                float3 bbmax = max(v0, max(v1, v2));
                primRefs[i] = { bbmin,(uint32_t)i,bbmax,{0},0 };

                centroidBounds.Expand((bbmin + bbmax) * 0.5f);
            }
            return centroidBounds;
        },
        [](AABB a, AABB b)
        {
            a.Expand(b);
            return a;
        }
    );

    // compact primref array sequentially
    size_t numValidPrims = 0;
    for (size_t i = 0; i < numPrims; i++)
    {
        if (primRefs[i].invalid == 0)
            primRefs[numValidPrims++] = primRefs[i];        
    }
    primRefs.resize(numValidPrims);
    
    // allocate parallel subtree
    ParallelSubtree root;
    root.tree.centroidBounds = centroidBounds;
    root.tree.numPrims       = (uint32_t) primRefs.size();
    root.tree.primRefs       = primRefs.data();

    BuildSubtreeParallel(mesh,&root,maxVerts,maxFaces,weldVertices, sahBlockSize);

    // walk parallel tree in-order and export the clusters in a deterministic order
    std::vector<ParallelSubtree*> stack;
    stack.push_back(&root);

    do
    {
        ParallelSubtree* t = stack.back();
        stack.pop_back();
        
        while (t->clusters.empty())
        {
            DGF_ASSERT(t->children != nullptr);
            stack.push_back(&t->children[1]);
            t = &t->children[0];
        }

        clusters.insert(clusters.end(),
            std::make_move_iterator(t->clusters.begin()),
            std::make_move_iterator(t->clusters.end()));

    } while (!stack.empty());


    // deallocate subtrees
    if (root.children)
    {
        stack.push_back(root.children);
        do
        {
            ParallelSubtree* t = stack.back();
            stack.pop_back();

            if( t[0].children )
                stack.push_back(t[0].children);
            if ( t[1].children )
                stack.push_back(t[1].children);
            delete[] t;

        } while (!stack.empty());
    }
}


bool ValidateClusters(const std::vector<Cluster>& clusters, const DGFBaker::BakerMesh& mesh, size_t maxVerts, size_t maxFaces )
{
    struct Triangle
    {
        bool operator!=(const Triangle& rhs) const {
            return v[0] != rhs.v[0] || v[1] != rhs.v[1] || v[2] != rhs.v[2] || ID != rhs.ID;
        };
        bool operator<(const Triangle& rhs) const
        {
            if( ID != rhs.ID )
                return ID < rhs.ID;
            
            for( size_t i=0; i<3; i++ )
                if( v[i] != rhs.v[i] )
                    return v[i] < rhs.v[i];

            return false;
        };

        float3 v[3];
        uint32_t ID;
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
                (uint32_t)(i) 
            } );
    }

    for (const auto& cluster : clusters)
    {
        if( cluster.VertexIndices.size() > maxVerts )
            return false;
        if (cluster.Topology.size() > maxFaces * 3)
            return false;

        for (size_t i = 0; i < cluster.Topology.size(); i += 3)
        {
            uint32_t triIndices[] = {
                cluster.VertexIndices[cluster.Topology[i + 0]],
                cluster.VertexIndices[cluster.Topology[i + 1]],
                cluster.VertexIndices[cluster.Topology[i + 2]]
            };

            float vertices[9];
            mesh.ReadVertices(vertices, triIndices, 3);
            
            clusterTris.push_back(
                {
                    {
                        { vertices[0], vertices[1], vertices[2] },
                        { vertices[3], vertices[4], vertices[5] },
                        { vertices[6], vertices[7], vertices[8] }
                    },
                 cluster.PrimIDs[i/3] 
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
