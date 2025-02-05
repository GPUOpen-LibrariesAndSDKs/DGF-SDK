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
#include "StripGeneration.h"


//#define STRIP_BUILD_TRACE(...) printf(__VA_ARGS__)
#define STRIP_BUILD_TRACE(...)

namespace DGF
{
    struct TmpStrip
    {
        struct TmpTri
        {
            uint8_t rotatedIndices[3];
            uint8_t inputIndex;
            uint8_t rotation;
        };

        void AddTriangle(const uint8_t* indices, uint8_t rotation, uint8_t inputIndex )
        {
            auto& tri = tris[numTris++];
            tri.inputIndex = inputIndex;
            tri.rotation = rotation;
            for (size_t i = 0; i < 3; i++)
                tri.rotatedIndices[i] = indices[3*inputIndex + ((i + tri.rotation) % 3)];

        }

        TmpTri tris[MAX_TRIS];
        size_t numTris = 0;
    };


    static void EncodeStrip( DGFStripInfo& strips, TriangleRemap* remapOut, const TmpStrip& tmpStrip )
    {
        size_t numTris = tmpStrip.numTris; 
        strips.numTriangles = tmpStrip.numTris;

        // Scan the strip and set control values.  
        strips.triControl[0] = DGF::TriControlValues::TC_RESTART;
        for (size_t i = 1; i < numTris; i++)
        {
            uint8_t p0 = tmpStrip.tris[i - 1].rotatedIndices[0];
            uint8_t p1 = tmpStrip.tris[i - 1].rotatedIndices[1];
            uint8_t p2 = tmpStrip.tris[i - 1].rotatedIndices[2];

            uint8_t v0 = tmpStrip.tris[i].rotatedIndices[0];
            uint8_t v1 = tmpStrip.tris[i].rotatedIndices[1];

            // 0----1
            //   \ |
            //    \|
            //     2
            if (p1 == v1 && p2 == v0)
            {
                // triangle dangles from second edge (1,2)
                strips.triControl[i] = DGF::TriControlValues::TC_EDGE1;
            }
            else if (p2 == v1 && p0 == v0)
            {
                // triangle dangles from third edge (2,0)
                strips.triControl[i] = DGF::TriControlValues::TC_EDGE2;
            }
            else if (i >= 2)
            {
                // try backtracking
                uint8_t pp0 = tmpStrip.tris[i - 2].rotatedIndices[0];
                uint8_t pp1 = tmpStrip.tris[i - 2].rotatedIndices[1];
                uint8_t pp2 = tmpStrip.tris[i - 2].rotatedIndices[2];

                // check for triangle dangling from third edge (2,0) of predecessor's predecessor
                if (strips.triControl[i - 1] == DGF::TriControlValues::TC_EDGE1 &&
                    pp2 == v1 && pp0 == v0 )
                {
                    strips.triControl[i] = DGF::TriControlValues::TC_BACKTRACK;
                }
                // check for triangle dangling from second edge (1,2) of predecessor's predecessor
                else if (strips.triControl[i - 1] == DGF::TriControlValues::TC_EDGE2 &&
                    pp1 == v1 && pp2 == v0)
                {
                    strips.triControl[i] = DGF::TriControlValues::TC_BACKTRACK;
                }
                else
                {
                    strips.triControl[i] = DGF::TriControlValues::TC_RESTART;
                }
            }
            else
            {
                strips.triControl[i] = DGF::TriControlValues::TC_RESTART;
            }
        }

        // build the index array and fill the remap tables
        strips.numIndices = 0;
        for (size_t i = 0; i < numTris; i++)
        {
            if (strips.triControl[i] == TriControlValues::TC_RESTART)
            {
                auto idx0 = tmpStrip.tris[i].rotatedIndices[0];
                auto idx1 = tmpStrip.tris[i].rotatedIndices[1];
                auto idx2 = tmpStrip.tris[i].rotatedIndices[2];

                strips.indexBuffer[strips.numIndices++] = idx0;
                strips.indexBuffer[strips.numIndices++] = idx1;
                strips.indexBuffer[strips.numIndices++] = idx2;
            }
            else
            {
                strips.indexBuffer[strips.numIndices++] = tmpStrip.tris[i].rotatedIndices[2];
            }

            remapOut[i].InputPrimID = tmpStrip.tris[i].inputIndex;
            remapOut[i].IndexRotation[0] = tmpStrip.tris[i].rotation;
            remapOut[i].IndexRotation[1] = (tmpStrip.tris[i].rotation + 1) % 3;
            remapOut[i].IndexRotation[2] = (tmpStrip.tris[i].rotation + 2) % 3;
        }
    }



    void GenerateStrips(DGFStripInfo& strips, BlockAdjacencyGraph& graph,  TriangleRemap* remapOut, const uint8_t* indices, size_t numTris )
    {
        TmpStrip tmpStrip;

        STRIP_BUILD_TRACE("\n\n");

        // iterate until all triangles are processed
        while (!graph.IsEmpty())
        {
            // pick a triangle to start the walk
            uint8_t srcNode = graph.PickStartingNode();
            STRIP_BUILD_TRACE("restart at: %u\n",srcNode);

            // if this is a disconnected triangle, add it and pick another
            if (graph.GetValence(srcNode) == 0)
            {
                graph.DeleteNode(srcNode);
                tmpStrip.AddTriangle(indices, 0, srcNode);
                continue;
            }

            // choose a minimum-valence edge to walk across
            auto edgeInfo  = graph.PickStartingEdge(srcNode);
            
            // add first triangle to the strip.  If we are walking across its first edge, rotate it one position            
            tmpStrip.AddTriangle(indices, edgeInfo.srcEdgeIndex == 0 ? 1 : 0, srcNode);

            bool canBacktrack = false;
            BlockAdjacencyGraph::EdgeInfo backtrackEdge;
            while (1)
            {                
                // remove the node we just came from
                graph.DeleteNode(edgeInfo.srcNode);

                // add the next triangle to the strip                    
                // rotate destination triangle so that the edge we're walking across is the zero'th edge
                uint8_t dstTri = edgeInfo.dstNode ;
                tmpStrip.AddTriangle(indices, edgeInfo.dstEdgeIndex, dstTri);
                
                if (graph.GetValence(edgeInfo.dstNode) == 0)
                {
                    // we got stuck and ran out of adjacent triangles

                    // see if we're able to backtrack
                    if (ENABLE_BACKTRACK && canBacktrack)
                    {
                        // successful backtrack.. continue strip, but prevent doing it twice
                        STRIP_BUILD_TRACE("backtrack\n");
                        canBacktrack = false;
                        edgeInfo = backtrackEdge;
                        continue;
                    }
                    else
                    {
                        // restart the strip
                        // remove the dead-end triangle from the graph
                        graph.DeleteNode(edgeInfo.dstNode);
                        break;
                    }
                }
                else
                {
                    // there's at least one neighbor that we could walk to
                    //  if there are two, retain the other for backtracking
                    canBacktrack = graph.PickNextEdge(edgeInfo.dstNode, &edgeInfo, &backtrackEdge);
                }
            }
        }

        EncodeStrip(strips, remapOut, tmpStrip);
    }
}