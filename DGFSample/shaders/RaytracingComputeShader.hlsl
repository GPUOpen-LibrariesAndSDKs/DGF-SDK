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

#ifndef RAYTRACING_COMPUTE_SHADER_HLSL
#define RAYTRACING_COMPUTE_SHADER_HLSL

#include "MeshShaderUtil.hlsl"
#include "DGFDecompression.hlsl"

#define RS \
"DescriptorTable(UAV(u0, numDescriptors = 1)),"\
"RootConstants(num32BitConstants=33, b0),"\
"SRV(t0),"\
"SRV(t1),"\
"SRV(t2),"\
"SRV(t3),"\
"SRV(t4)"

static const float3 MeshletColors[] =
{
    float3(255.0f / 255.0f, 105.0f / 255.0f, 97.0f / 255.0f),
    float3(255.0f / 255.0f, 180.0f / 255.0f, 128.0f / 255.0f),
    float3(248.0f / 255.0f, 243.0f / 255.0f, 141.0f / 255.0f),
    float3(66.0f / 255.0f, 214.0f / 255.0f, 164.0f / 255.0f),
    float3(8.0f / 255.0f, 202.0f / 255.0f, 209.0f / 255.0f),
    float3(89.0f / 255.0f, 173.0f / 255.0f, 246.0f / 255.0f),
    float3(157.0f / 255.0f, 148.0f / 255.0f, 255.0f / 255.0f),
    float3(199.0f / 255.0f, 128.0f / 255.0f, 232.0f / 255.0f),
    
    0.75f * float3(255.0f / 255.0f, 105.0f / 255.0f, 97.0f / 255.0f),
    0.75f * float3(255.0f / 255.0f, 180.0f / 255.0f, 128.0f / 255.0f),
    0.75f * float3(248.0f / 255.0f, 243.0f / 255.0f, 141.0f / 255.0f),
    0.75f * float3(66.0f / 255.0f, 214.0f / 255.0f, 164.0f / 255.0f),
    0.75f * float3(8.0f / 255.0f, 202.0f / 255.0f, 209.0f / 255.0f),
    0.75f * float3(89.0f / 255.0f, 173.0f / 255.0f, 246.0f / 255.0f),
    0.75f * float3(157.0f / 255.0f, 148.0f / 255.0f, 255.0f / 255.0f),
    0.75f * float3(199.0f / 255.0f, 128.0f / 255.0f, 232.0f / 255.0f),
};

struct RTConstants
{
    float4x4 viewProjMatrix;
    float4x4 normalMatrix;
    uint displayMode;
};
    
ConstantBuffer<RTConstants> Constants   : register(b0);
RWTexture2D<unorm float4> RenderTarget  : register(u0);
RaytracingAccelerationStructure tlas    : register(t0);
ByteAddressBuffer dgfBlockMap           : register(t1);
StructuredBuffer<uint> SurfaceNormals   : register(t2);
StructuredBuffer<uint> VertexNormals    : register(t3);
ByteAddressBuffer dgfBlockBuffer        : register(t4);


[RootSignature(RS)]
[numthreads(8, 8, 1)]
void CS_main(uint3 tid : SV_DispatchThreadID)
{
    uint2 dims;
    RenderTarget.GetDimensions(dims.x, dims.y);
    
    if (tid.x < dims.x && tid.y < dims.y)
    {
        float2 uv = (tid.xy + 0.5f) / float2(dims.xy);
        uv = float2(2, -2) * uv + float2(-1, 1);
        float4 V0 = mul(Constants.viewProjMatrix, float4(uv.xy, 0, 1));
        float4 V1 = mul(Constants.viewProjMatrix, float4(uv.xy, 1, 1));
        float3 Origin    = V0.xyz / V0.w;
        float3 Direction = ((V1.xyz / V1.w) - Origin.xyz);
        
        RayDesc rd;
        rd.Origin = Origin;
        rd.Direction = Direction;
        rd.TMin = 0;
        rd.TMax = 1;
        
        RayQuery <RAY_FLAG_FORCE_OPAQUE|RAY_FLAG_SKIP_PROCEDURAL_PRIMITIVES> rq;
        rq.TraceRayInline(tlas, 0, 0xff, rd);
        rq.Proceed();
        
        float4 color = float4(0.3, 0, 0, 1);
        if (rq.CommittedStatus() == COMMITTED_TRIANGLE_HIT)
        {           
            uint primIndex = rq.CommittedPrimitiveIndex();
            uint2 blockInfo = DGFLookupBlockMap(dgfBlockMap,primIndex);
            uint dgfBlockIndex = blockInfo.x;
            uint triangleIndexInBlock = blockInfo.y;
            
            DGFBlockInfo dgfBlock = DGFLoadBlockInfo(dgfBlockBuffer, dgfBlockIndex);
            
            float3 shadingNormal = 0;
            if (Constants.displayMode == 2)
            {
                // original mesh face normals
                float4 normal     = float4(UnpackFloat3x10(SurfaceNormals.Load(primIndex)), 0.0f);
                float4 faceNormal = normalize(mul(Constants.normalMatrix, normal));
                shadingNormal = faceNormal.xyz;
            }
            else if (Constants.displayMode == 3)
            {
                // original mesh vertex normals
                uint3 localIndices    = DGFGetTriangle_BitScan_Lane(dgfBlock, triangleIndexInBlock);
                uint3 vertexIndices   = localIndices + dgfBlock.header.userData.xxx;
                
                float3 nx = UnpackFloat3x10(VertexNormals.Load(vertexIndices.x));
                float3 ny = UnpackFloat3x10(VertexNormals.Load(vertexIndices.y));
                float3 nz = UnpackFloat3x10(VertexNormals.Load(vertexIndices.z));
                float2 bary = rq.CommittedTriangleBarycentrics();
                float3 normal = nx * (1 - bary.x - bary.y) + ny * bary.x + nz * bary.y;
                shadingNormal = normalize(mul(Constants.normalMatrix, float4(normal, 0)).xyz);
            }
            else
            { 
                // compressed triangle normal
                uint3 localIndices = DGFGetTriangle_BitScan_Lane(dgfBlock, triangleIndexInBlock);
                float3 V0 = DGFGetVertex(dgfBlock, localIndices.x);
                float3 V1 = DGFGetVertex(dgfBlock, localIndices.y);
                float3 V2 = DGFGetVertex(dgfBlock, localIndices.z);                
                float3 N = cross(V1 - V0, V2 - V0);
                shadingNormal = normalize(mul(Constants.normalMatrix, float4(N, 0)).xyz);
            }          
            
            color.xyz = saturate(-dot(shadingNormal, normalize(Direction)));
            
            if (Constants.displayMode == 1)
            {
                // show meshlet colors
                color.xyz *= MeshletColors[dgfBlockIndex & 15];
            }
            else if (Constants.displayMode == 4)
            {
                // show geomIDs
                uint geomID = DGFGetGeomIDAndOpacity(dgfBlock, triangleIndexInBlock).x;
                color.xyz *= MeshletColors[geomID & 15];
            }
        }
        
        RenderTarget[tid.xy] = color;
    }    
}

#endif // RAYTRACING_COMPUTE_SHADER_HLSL