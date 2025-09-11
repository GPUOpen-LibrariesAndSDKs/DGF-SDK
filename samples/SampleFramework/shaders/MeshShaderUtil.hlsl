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

#ifndef MESH_SHADER_UTIL_HLSL
#define MESH_SHADER_UTIL_HLSL

#ifdef SAFE_MESH_SHADER

#define CLAMP_VERT_IDX(x__) clamp((x__), 0, MS_MAX_VERTS - 1)
#define CLAMP_VERT_IDX_UINT3(x__) (uint3(CLAMP_VERT_IDX((x__).x), CLAMP_VERT_IDX((x__).y), CLAMP_VERT_IDX((x__).z)))
#define CLAMP_TRI_IDX(x__) clamp((x__), 0, MS_MAX_TRIS - 1)
#define CLAMP_MAX_VERTS(x__) clamp((x__), 0, MS_MAX_VERTS)
#define CLAMP_MAX_TRIS(x__) clamp((x__), 0, MS_MAX_TRIS)

#else

#define CLAMP_VERT_IDX(x__) x__
#define CLAMP_VERT_IDX_UINT3(x__) x__
#define CLAMP_TRI_IDX(x__) x__
#define CLAMP_MAX_VERTS(x__) x__
#define CLAMP_MAX_TRIS(x__) x__

#endif

float3 UnpackFloat3x10(uint packedNormal)
{
    // Extract 10 bits for each component
    float x = (float) (packedNormal & 0x3FF);
    float y = (float) ((packedNormal >> 10) & 0x3FF);
    float z = (float) ((packedNormal >> 20) & 0x3FF);

    // Map back from [0, 1023] to [-1, 1]
    x = x / 1023.0f * 2.0f - 1.0f;
    y = y / 1023.0f * 2.0f - 1.0f;
    z = z / 1023.0f * 2.0f - 1.0f;

    // Normalize the vector to account for any minor precision loss
    return normalize(float3(x, y, z));
}

uint PackFloat3x10(float3 normal)
{
    // Convert from [-1, 1] to [0, 1023] (for 10 bits)
    uint x = (uint) ((normal.x * 0.5f + 0.5f) * 1023.0f + 0.5f);
    uint y = (uint) ((normal.y * 0.5f + 0.5f) * 1023.0f + 0.5f);
    uint z = (uint) ((normal.z * 0.5f + 0.5f) * 1023.0f + 0.5f);

        // Pack the three 10-bit values into a single 32-bit integer
    return (x & 0x3FF) | ((y & 0x3FF) << 10) | ((z & 0x3FF) << 20);
}


#endif // MESH_SHADER_UTIL_HLSL