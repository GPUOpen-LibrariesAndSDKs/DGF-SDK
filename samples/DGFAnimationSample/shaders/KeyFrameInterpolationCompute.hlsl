// Copyright (c) 2024 Advanced Micro Devices, Inc.All rights reserved.

#include "MeshShaderUtil.hlsl"

#define RS_COMPUTE_SHADER \
"CBV(b0),"\
"SRV(t0),"\
"SRV(t1),"\
"SRV(t2),"\
"SRV(t3),"\
"UAV(u0),"\
"UAV(u1)"

struct ComputeConstStruct
{
    float InterpolationFactor;
    uint VertexCount;
};

[[vk::binding(0, 0)]]
ConstantBuffer<ComputeConstStruct> ComputeConst : register(b0);

[[vk::binding(1, 0)]]
StructuredBuffer<float3> KeyFramePositions : register(t0);

[[vk::binding(2, 0)]]
StructuredBuffer<uint> KeyFrameNormals : register(t1);

[[vk::binding(3, 0)]]
StructuredBuffer<float3> NextKeyFramePositions : register(t2);

[[vk::binding(4, 0)]]
StructuredBuffer<uint> NextKeyFrameNormals : register(t3);

[[vk::binding(5, 0)]]
RWStructuredBuffer<float3> Positions : register(u0);

[[vk::binding(6, 0)]]
RWStructuredBuffer<uint> Normals : register(u1);

[RootSignature(RS_COMPUTE_SHADER)]
[NumThreads(64, 1, 1)]
void CS_main(int3 DTid : SV_DispatchThreadID)
{
    if (DTid.x < ComputeConst.VertexCount)
    {
        const float3 v0 = KeyFramePositions.Load(DTid.x);
        const float3 v1 = NextKeyFramePositions.Load(DTid.x);
        
        Positions[DTid.x] = lerp(v0, v1, ComputeConst.InterpolationFactor);
    
        const float3 n0 = UnpackFloat3x10(KeyFrameNormals.Load(DTid.x));
        const float3 n1 = UnpackFloat3x10(NextKeyFrameNormals.Load(DTid.x));
        
        Normals[DTid.x] = PackFloat3x10(lerp(n0, n1, ComputeConst.InterpolationFactor));
    }
}
