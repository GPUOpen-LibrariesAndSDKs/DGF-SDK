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

#ifndef DGF_BVH_BUILD_DECODE_HLSL
#define DGF_BVH_BUILD_DECODE_HLSL

#include "DGFDecompression.hlsl"

#define RS \
"SRV(t0),UAV(u0),UAV(u1)"

ByteAddressBuffer  DGFBuffer     : register(t0);
RWByteAddressBuffer VertexBuffer : register(u0);
RWByteAddressBuffer IndexBuffer  : register(u1);

[RootSignature(RS)]
[numthreads(64,1,1)]
void CS_main( uint3 groupID : SV_GroupID, uint3 gtid : SV_GroupThreadID )
{
    const uint blockId = groupID.x;
    DGFBlockInfo s = DGFInit(DGFBuffer, blockId);
    
    uint vertexBase   = s.header.userData;
    uint triangleBase = s.header.primIDBase;
    
    uint vertexIndex   = min(gtid.x, s.header.numVerts - 1);
    uint triangleIndex = min(gtid.x, s.header.numTriangles - 1);
    
    uint3 TriangleIndices = DGFGetTriangle_BitScan_Wave(s, triangleIndex);
    TriangleIndices.xyz += vertexBase.xxx;
    
    float3 Vertex = DGFGetVertex(s, vertexIndex);
        
    uint vertexAddress = 12 * (vertexBase + vertexIndex);
    uint triangleAddress = 12 * (triangleBase + triangleIndex);
    
    VertexBuffer.Store3(vertexAddress, asuint(Vertex));
    IndexBuffer.Store3(triangleAddress, TriangleIndices);
}

#endif // DGF_BVH_BUILD_DECODE_HLSL