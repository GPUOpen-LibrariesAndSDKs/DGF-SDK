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

#include <cstdint>
#include <bit>
#include <bitset>
#include "DGF.h"

// The graph code is perf-sensitive enough that we allow opting out of asserts
#define GRAPH_ASSERT(x)
//#define GRAPH_ASSERT(x) DGF_ASSERT(x)

namespace DGF
{

    class ClusterAdjacencyGraph
    {
    public:
        static constexpr size_t MAX_NODES = 256;
        
        size_t CountEdges( uint8_t candidate, const std::bitset<MAX_NODES>& nodesInBlock )
        {
            GRAPH_ASSERT(candidate < m_totalNodes);
            auto& node = m_nodes[candidate];
            size_t edgeCount = 0;
            size_t edgeMask = node.edgeValid;
            while (edgeMask)
            {
                size_t edgeIdx = std::countr_zero(edgeMask);
                if (nodesInBlock.test(node.edges[edgeIdx].target))
                    edgeCount++;
                edgeMask ^= (1ull << edgeIdx);
            }
            return edgeCount;
        }

        void Build(const uint8_t* vertexIndices, size_t numTris)
        {
            // initialize graph and nodes
            m_totalNodes = (uint8_t)numTris;
            for (size_t i = 0; i < numTris; i++)
            {
                m_nodes[i].edgeValid = 0;
            }

            struct HalfEdge
            {
                uint16_t SortKey; // the smallest(LSB) and largest (MSB) vertex indices
                uint8_t FaceIndex;
                uint8_t EdgeIndexInFace;
            };
            HalfEdge edges[3 * MAX_NODES];

            static_assert(sizeof(HalfEdge) == 4, "No compiler.. no!");
            
            // build half edges
            size_t numHalfEdges = 0;
            for (size_t i = 0; i < numTris; i++)
            {
                for (size_t k = 0; k < 3; k++)
                {
                    uint8_t i0 = vertexIndices[3 * i + k];
                    uint8_t i1 = vertexIndices[3 * i + (k + 1) % 3];
                    size_t V0 = std::min(i0, i1);
                    size_t V1 = std::max(i0, i1);

                    HalfEdge e;
                    e.FaceIndex = static_cast<uint8_t>(i);
                    e.EdgeIndexInFace = (uint8_t)k;
                    e.SortKey = (uint16_t)( (V0 << 8) + V1);
                    edges[numHalfEdges++] = e;

                    GRAPH_ASSERT(V0 != V1); // XXX: Degens don't work
                }
            }

            // sort half-edges by index pair and fuse into full-edges
            RadixSort<HalfEdge, 3*MAX_NODES>(edges, numHalfEdges, 0xffff, [](HalfEdge e) {return e.SortKey; });

            size_t numFullEdges = 0;
            size_t e = 0;
            do
            {
                // find end of run of like edges
                size_t e0 = e;
                do
                {
                    e++;
                } while (e < numHalfEdges && edges[e0].SortKey == edges[e].SortKey);

                // form pairs as long as we have two half-edges available
                //  looping will naturally handle the non-manifold case
                //
                //  In the event of a non-manifold, edges with more than 2 triangles
                //    will be duplicated once for each pair of triangles.  This will limit
                //    the number of mesh paths available to the strip generator.. but this is the price
                //    one pays for being silly and using non-manifolds
                //
                size_t run_length = e - e0;
                while (run_length > 1)
                {
                    HalfEdge a = edges[e0];
                    e0++;
                    HalfEdge b = edges[e0];
                    e0++;

                    // insert an edge into the nodes for each pair of adjacent faces
                    size_t F0 = a.FaceIndex;
                    size_t F1 = b.FaceIndex;
                    size_t F0Idx = a.EdgeIndexInFace;
                    size_t F1Idx = b.EdgeIndexInFace;

                    auto& node0 = m_nodes[F0];
                    auto& node1 = m_nodes[F1];
                    size_t F0Bit = 1ull << F0Idx;
                    size_t F1Bit = 1ull << F1Idx;
                    GRAPH_ASSERT((node0.edgeValid & F0Bit) == 0);
                    GRAPH_ASSERT((node1.edgeValid & F1Bit) == 0);

                    node0.edgeValid |= F0Bit;
                    node1.edgeValid |= F1Bit;

                    auto& edge0 = node0.edges[F0Idx];
                    edge0.target = (uint8_t) F1;
                    edge0.srcIndex = F0Idx;
                    edge0.dstIndex = F1Idx;

                    auto& edge1 = node1.edges[F1Idx];
                    edge1.target = (uint8_t) F0;
                    edge1.srcIndex = F1Idx;
                    edge1.dstIndex = F0Idx;

                    run_length -= 2;
                }

            } while (e < numHalfEdges);

            // in non-manifold cases involving double-wound triangles, it's possible to have
            //   two edges with the same target.   Detect these and fix them
            //   otherwise the strip builder will get confused and try to add them twice
            for (size_t i = 0; i < numTris; i++)
            {
                auto& node = m_nodes[i];
                for (size_t e = 0; e < 3; e++)
                {
                    if (!(node.edgeValid & (1 << e)))
                        continue;

                    for (size_t e1 = 0; e1 < 3; e1++)
                    {
                        if (e1 == e)
                            continue;
                        if (!(node.edgeValid & (1 << e1)))
                            continue;

                        if (node.edges[e].target == node.edges[e1].target) {
                            node.edgeValid ^= (1 << e1);
                        }
                    }
                }
            }
        }

    private:
        friend class BlockAdjacencyGraph;
        struct Edge
        {
            uint8_t target;
            uint8_t srcIndex : 2; // index of this edge in source node's edge list
            uint8_t dstIndex : 2; // index of this edge in sink node's edge list
        };

        struct Node
        {
            uint8_t edgeValid;
            Edge edges[3];
        };

        
        uint8_t m_totalNodes;
        Node m_nodes[MAX_NODES];
    };


    class BlockAdjacencyGraph
    {
    public:

        void Build(const ClusterAdjacencyGraph& g, const uint8_t* triangleIndices, size_t numTris)
        {
            uint8_t triRemap[ClusterAdjacencyGraph::MAX_NODES];
            std::bitset < ClusterAdjacencyGraph::MAX_NODES> triSeen;

            for (size_t i = 0; i < numTris; i++)
            {
                uint8_t inputIndex = triangleIndices[i];
                GRAPH_ASSERT(inputIndex < g.m_totalNodes);
                auto& srcNode = g.m_nodes[inputIndex];

                m_nodes[i].valence = 0;
                m_nodes[i].edgeValid = 0;

                size_t edgeMask = srcNode.edgeValid;
                while (edgeMask)
                {
                    // iterate the edges and see if we have added the target triangle
                    //  if we have, insert this edge into the edge set for both nodes
                    size_t edgeIndex = std::countr_zero(edgeMask);
                    size_t target = srcNode.edges[edgeIndex].target;
                    size_t srcIndex = srcNode.edges[edgeIndex].srcIndex;
                    size_t dstIndex = srcNode.edges[edgeIndex].dstIndex;
                    if (triSeen.test(target))
                    {
                        uint8_t remappedIndex = triRemap[target];
                        m_nodes[i].edgeValid |= (1 << edgeIndex);
                        m_nodes[i].edges[edgeIndex].target = remappedIndex;
                        m_nodes[i].edges[edgeIndex].dstIndex = dstIndex;
                        m_nodes[i].edges[edgeIndex].srcIndex = srcIndex;
                        m_nodes[i].valence++;

                        m_nodes[remappedIndex].edgeValid |= (1 << dstIndex);
                        m_nodes[remappedIndex].edges[dstIndex].target = (uint8_t) i;
                        m_nodes[remappedIndex].edges[dstIndex].dstIndex = srcIndex;
                        m_nodes[remappedIndex].edges[dstIndex].srcIndex = dstIndex;
                        m_nodes[remappedIndex].valence++;
                    }
                    edgeMask ^= (1ull << edgeIndex);
                }

                triSeen.set(inputIndex);
                triRemap[inputIndex] = (uint8_t) i;
            }

            m_nodeValid = (numTris == 64) ? ~0 : ((1ull << numTris) - 1);          
        }

        bool IsEmpty() const { return m_nodeValid == 0; }

        uint8_t GetValence(size_t node) const
        {
            GRAPH_ASSERT(m_nodeValid & (1ull << node));
            return m_nodes[node].valence;
        }

        uint8_t PickStartingNode() const
        {
            // scan nodes and find one with minimum valence
            uint64_t nodeMask = m_nodeValid;
            size_t bestValence = 4;
            size_t bestNode = 0;
            while (nodeMask)
            {
                size_t idx = std::countr_zero(nodeMask);
                size_t valence = GetValence(idx);
                if (valence < bestValence)
                {
                    bestValence = valence;
                    bestNode = idx;
                }

                nodeMask ^= (1ull << idx);
            }
            return (uint8_t) bestNode;
        }

        void DeleteNode(uint8_t nodeIndex)
        {
            // mark node invalid
            GRAPH_ASSERT(m_nodeValid & (1ull << nodeIndex));
            m_nodeValid ^= (1ull << nodeIndex);

            // invalidate edges in neighboring nodes
            Node& node = m_nodes[nodeIndex];
            for (size_t i = 0; i < 3; i++)
            {
                if (node.edgeValid & (1 << i))
                {
                    auto dstEdgeIndex = node.edges[i].dstIndex;
                    auto neighborIdx = node.edges[i].target;
                    auto& neighbor = m_nodes[neighborIdx];

                    GRAPH_ASSERT(m_nodeValid & (1ull << neighborIdx));
                    GRAPH_ASSERT(neighbor.valence > 0);
                    GRAPH_ASSERT(neighbor.edges[dstEdgeIndex].target == nodeIndex);
                    GRAPH_ASSERT(neighbor.edgeValid & (1 << dstEdgeIndex));

                    neighbor.valence--;
                    neighbor.edgeValid ^= (1 << dstEdgeIndex);
                }
            }
        }

        struct EdgeInfo
        {
            uint8_t srcNode;
            uint8_t dstNode;
            uint8_t srcEdgeIndex; // index of the edge in source's edge list
            uint8_t dstEdgeIndex; // index of the edge in target's edge list
        };

        EdgeInfo PickStartingEdge(uint8_t nodeIdx) const
        {
            GRAPH_ASSERT(m_nodeValid & (1ull << nodeIdx));

            // choose edge with minimum valence
            size_t bestValence = 4;
            size_t bestEdge = 0;
            auto& node = m_nodes[nodeIdx];
            for (size_t i = 0; i < 3; i++)
            {
                if (node.edgeValid & (1 << i))
                {
                    size_t valence = GetValence(node.edges[i].target);
                    if (valence < bestValence)
                    {
                        bestEdge = i;
                        bestValence = valence;
                    }
                }
            }

            EdgeInfo edgeInfo;
            edgeInfo.srcNode = nodeIdx;
            edgeInfo.srcEdgeIndex = (uint8_t) bestEdge;
            edgeInfo.dstEdgeIndex = node.edges[bestEdge].dstIndex;
            edgeInfo.dstNode = node.edges[bestEdge].target;
            return edgeInfo;
        }

        bool PickNextEdge(uint8_t nodeIndex, EdgeInfo* next, EdgeInfo* backtrack)
        {
            GRAPH_ASSERT(m_nodeValid & (1ull << nodeIndex));

            auto& node = m_nodes[nodeIndex];
            size_t valence = GetValence(nodeIndex);
            GRAPH_ASSERT(valence > 0 && valence <= 2);
            GRAPH_ASSERT(valence == std::popcount(node.edgeValid));

            if (valence == 2)
            {
                // two edges... choose the one whose target has minimum valence
                //    keep the other one for backtracking
                size_t edgeMask = node.edgeValid;
                size_t e0 = std::countr_zero(edgeMask);
                size_t e1 = std::countr_zero(edgeMask ^ (1ull << e0));
                size_t v0 = GetValence(node.edges[e0].target);
                size_t v1 = GetValence(node.edges[e1].target);
                if (v1 < v0)
                    std::swap(e0, e1);
                
                next->dstNode      = node.edges[e0].target;
                next->dstEdgeIndex = node.edges[e0].dstIndex;
                next->srcEdgeIndex = node.edges[e0].srcIndex;
                next->srcNode      = nodeIndex;

                // if we backtrack, we want the strip builder to remove the target of the first edge 
                backtrack->dstNode = node.edges[e1].target;
                backtrack->dstEdgeIndex = node.edges[e1].dstIndex;
                backtrack->srcEdgeIndex = node.edges[e1].srcIndex;
                backtrack->srcNode = node.edges[e0].target; 
                return true;
            }
            else
            {
                // only one edge
                size_t validEdge = std::countr_zero(node.edgeValid);
                next->dstNode = node.edges[validEdge].target;
                next->dstEdgeIndex = node.edges[validEdge].dstIndex;
                next->srcEdgeIndex = node.edges[validEdge].srcIndex;
                next->srcNode = nodeIndex;
                return false;
            }

        }

    private:

        struct Edge
        {
            uint8_t target;
            uint8_t srcIndex : 2; // index of this edge in source node's edge list
            uint8_t dstIndex : 2; // index of this edge in sink node's edge list
        };

        struct Node
        {
            uint8_t valence;
            uint8_t edgeValid;
            Edge edges[3];
        };

        uint64_t m_nodeValid;
        Node m_nodes[DGF::MAX_TRIS];
        static_assert(DGF::MAX_TRIS <= 64, "More bits please!");

    };


}