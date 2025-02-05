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

#include "MeshShaderUtil.hlsl"
#include "PixelShader.hlsl"
#include "DGFDecompression.hlsl"

#define RS_MESH_SHADER \
"RootConstants(num32BitConstants=56, b1),"\
"RootConstants(num32BitConstants=2, b0),"\
"SRV(t0),"\
"SRV(t1),"\
"SRV(t2)"\

struct MeshShaderConstStruct
{
    uint DGFBlockStart;
};

ConstantBuffer<MeshShaderConstStruct> MeshShaderConst : register(b0);
ByteAddressBuffer DGFBuffer : register(t0);
StructuredBuffer<uint> SurfaceNormals : register(t1);
StructuredBuffer<uint> VertexNormals : register(t2);

 
[RootSignature(RS_MESH_SHADER)]
[NumThreads(MS_NUM_THREADS, 1, 1)]
[OutputTopology("triangle")]
void MS_main(
    uint gtid : SV_GroupThreadID,
    uint gid : SV_GroupID,
    out indices uint3 tris[MS_MAX_TRIS],
    out vertices PixelShaderInput verts[MS_MAX_VERTS],
    out primitives PrimitiveAttributes outprim[MS_MAX_TRIS]
)
{
    const uint blockId = MeshShaderConst.DGFBlockStart + gid;
    DGFBlockInfo s = DGFInit(DGFBuffer, blockId);
    
    SetMeshOutputCounts(CLAMP_MAX_VERTS(s.header.numVerts), CLAMP_MAX_TRIS(s.header.numTriangles));
    
    if (gtid < s.header.numTriangles)
    {
        tris[CLAMP_TRI_IDX(gtid)] = CLAMP_VERT_IDX_UINT3(DGFGetTriangle_BitScan_Lane(s, gtid));
        outprim[CLAMP_TRI_IDX(gtid)].geomID = DGFGetGeomIDAndOpacity(s, gtid).x;
        
        if (RasterConsts.displayMode == 2)
        {
            float4 normal = float4(UnpackFloat3x10(SurfaceNormals.Load(s.header.primIDBase + gtid)), 0.0f);
            outprim[CLAMP_TRI_IDX(gtid)].normal = normalize(mul(RasterConsts.normalMatrix, normal));
        }
    }
    
    if (gtid < s.header.numVerts)
    {
        const float4 p = float4(DGFGetVertex(s, gtid), 1.0f);
        verts[CLAMP_VERT_IDX(gtid)].position = mul(RasterConsts.projectionMatrix, p);
        verts[CLAMP_VERT_IDX(gtid)].worldSpacePosition = mul(RasterConsts.modelWorldMatrix, p).xyz;
        verts[CLAMP_VERT_IDX(gtid)].meshletIndex = gid;
        
        if (RasterConsts.displayMode == 3)
        {
            uint vertexOffset = s.header.userData;
            float4 normal = float4(UnpackFloat3x10(VertexNormals.Load(vertexOffset + gtid)), 0.0f);
            verts[CLAMP_VERT_IDX(gtid)].normal = normalize(mul(RasterConsts.normalMatrix, normal));
        }
    }
}