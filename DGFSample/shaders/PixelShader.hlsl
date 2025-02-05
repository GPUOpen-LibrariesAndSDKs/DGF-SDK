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

#ifndef PIXEL_SHADER_HLSL
#define PIXEL_SHADER_HLSL

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
    
    0.75f*float3(255.0f / 255.0f, 105.0f / 255.0f, 97.0f / 255.0f),
    0.75f*float3(255.0f / 255.0f, 180.0f / 255.0f, 128.0f / 255.0f),
    0.75f*float3(248.0f / 255.0f, 243.0f / 255.0f, 141.0f / 255.0f),
    0.75f*float3(66.0f / 255.0f, 214.0f / 255.0f, 164.0f / 255.0f),
    0.75f*float3(8.0f / 255.0f, 202.0f / 255.0f, 209.0f / 255.0f),
    0.75f*float3(89.0f / 255.0f, 173.0f / 255.0f, 246.0f / 255.0f),
    0.75f*float3(157.0f / 255.0f, 148.0f / 255.0f, 255.0f / 255.0f),
    0.75f*float3(199.0f / 255.0f, 128.0f / 255.0f, 232.0f / 255.0f),
};

struct PixelShaderInput
{
    float4 position : SV_Position;
    float3 worldSpacePosition : POSITION;
    int meshletIndex : TEXCOORD;
    float4 normal : COLOR0;
};

struct PrimitiveAttributes
{
    float4 normal : COLOR1;
    uint geomID   : COLOR2;
};

struct RasterConstStruct
{
    float4x4 projectionMatrix;
    float4x4 modelWorldMatrix;
    float4x4 normalMatrix;
    float4 cameraPosition;
    uint displayMode;
};

ConstantBuffer<RasterConstStruct> RasterConsts : register(b1);

float4 PS_main(PixelShaderInput input, PrimitiveAttributes primInput) : SV_Target
{
    float3 color;
    
    float3 V = normalize(RasterConsts.cameraPosition.xyz - input.worldSpacePosition);
    
    if (RasterConsts.displayMode == 2)
    {
        // face normal
        float3 N = primInput.normal.xyz;
        color = saturate(dot(N, V));
    }
    else if (RasterConsts.displayMode == 3)
    {
        // vertex normal
        float3 N = normalize(input.normal.xyz);
        color = saturate(dot(N, V));
    }
    else 
    {
        // compressed face normal
        float3 dx = ddx(input.worldSpacePosition);
        float3 dy = ddy(input.worldSpacePosition);
        float3 N = normalize(cross(dx, dy));
        color = saturate(-dot(N, V));
    }
    
    if (RasterConsts.displayMode == 1)
    {
        // meshlet colors
        color *= MeshletColors[input.meshletIndex & 15];
    }
    else if (RasterConsts.displayMode == 4)
    {
        // geomID colors
        color *= MeshletColors[primInput.geomID & 15];        
    }
       
    return float4(color, 1);
}

#endif // PIXEL_SHADER_HLSL