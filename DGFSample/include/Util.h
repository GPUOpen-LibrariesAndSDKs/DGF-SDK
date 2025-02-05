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

#include <vector>
#include <filesystem>
#include <fstream>
#include "stdafx.h"
#include <DGF.h>
#include "DGFBaker.h"

#include "DXFramework.h"

typedef unsigned int uint;

struct float3 {
    float x;
    float y;
    float z;

    inline float3 operator+(const float3& rhs) const
    {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }
    inline float3 operator-(const float3& rhs) const
    {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }
    inline float3 operator*(const float s) const
    {
        return {x * s, y * s, z * s};
    }

    inline float3 operator+=(const float3& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    float3() : x(0), y(0), z(0){};

    float3(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}

    bool operator==(const float3& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

inline float3 normalize(float3 v)
{
    const auto l = v.x * v.x + v.y *v.y + v.z * v.z;
    const auto li = 1.0f / sqrtf(l);
    if (isfinite(li))
    {
        const auto x = v.x * li;
        const auto y = v.y * li;
        const auto z = v.z * li;
        return float3(x, y, z);
    }
    else
    {
        return float3(0, 0, 1);
    }
}

inline float3 cross(float3 a, float3 b)
{
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

struct float4 {
    float x;
    float y;
    float z;
    float w;
};

struct float4x4 {
    float4x4()
    {
        SetZero();
    }

    void SetIdentity()
    {
        SetZero();
        m00 = m11 = m22 = m33 = 1.0f;
    }

    void SetZero()
    {
        for (int i = 0; i < _countof(d); i++) {
            d[i] = 0.0f;
        }
    }

    void SetProj(float aspect, float fovYDegrees, float nearPlaneDistance, float farPlaneDistance)
    {
        SetZero();
        constexpr auto degressToRad = 3.1415926f / 180.0f;
        const auto     t            = tanf(fovYDegrees * 0.5f * degressToRad);
        const auto     n            = nearPlaneDistance;
        const auto     f            = farPlaneDistance;
        m00                         = 1.0f / (t * aspect);
        m11                         = 1.0f / t;
        m22                         = f / (f - n);
        m32                         = 1.0;
        m23                         = -(f * n) / (f - n);
    }

    void SetTranslate(const float3& t)
    {
        SetIdentity();
        m03 = t.x;
        m13 = t.y;
        m23 = t.z;
    }

    void SetRotateX(float angleDegrees)
    {
        SetIdentity();
        constexpr auto degressToRad = 3.1415926f / 180.0f;
        const auto     c            = cosf(angleDegrees * degressToRad);
        const auto     s            = sinf(angleDegrees * degressToRad);
        m11                         = c;
        m12                         = -s;
        m21                         = s;
        m22                         = c;
    }

    void SetRotateY(float angleDegrees)
    {
        SetIdentity();
        constexpr auto degressToRad = 3.1415926f / 180.0f;
        const auto     c            = cosf(angleDegrees * degressToRad);
        const auto     s            = sinf(angleDegrees * degressToRad);
        m20                         = c;
        m22                         = s;
        m00                         = -s;
        m02                         = c;
    }

    void SetRotateZ(float angleDegrees)
    {
        SetIdentity();
        constexpr auto degressToRad = 3.1415926f / 180.0f;
        const auto     c            = cosf(angleDegrees * degressToRad);
        const auto     s            = sinf(angleDegrees * degressToRad);
        m00                         = c;
        m01                         = -s;
        m10                         = s;
        m11                         = c;
    }

    const float& operator()(int row, int col) const
    {
        return d[col * 4 + row];
    }

    float& operator()(int row, int col)
    {
        return d[col * 4 + row];
    }

    union {
        struct {
            float m00, m10, m20, m30;
            float m01, m11, m21, m31;
            float m02, m12, m22, m32;
            float m03, m13, m23, m33;
        };
        struct {
            float4 col0;
            float4 col1;
            float4 col2;
            float4 col3;
        };
        float d[16];
    };
};

struct uint3 {
    uint x;
    uint y;
    uint z;
    uint3(){};
    uint3(uint xx, uint yy, uint zz) : x(xx), y(yy), z(zz) {}
};

float4x4 operator*(const float4x4& l, const float4x4& r);

struct uint2 {
    uint x;
    uint y;
    uint2(){};
    uint2(uint xx, uint yy) : x(xx), y(yy) {}
};

struct Mesh {
    std::vector<float>    vertices;        // 3 floats per vertex
    std::vector<uint32_t> surfaceNormals;  // 10 bits per channel
    std::vector<uint32_t> vertexNormals;   // 10 bits per channel
    std::vector<uint32_t> indices;
    std::vector<uint32_t> materialIDs;  // material ID per face
    size_t                numMaterials = 0;

    size_t GetMeshSizeInBytes() const;

    size_t GetNumVertices() const
    {
        return vertices.size() / 3;
    }

    size_t GetNumTriangles() const
    {
        return indices.size() / 3;
    }
};

class DGFMesh {
public:
    explicit DGFMesh() {}
    explicit DGFMesh(const std::filesystem::path pathToFile);
    explicit DGFMesh(const std::vector<uint8_t>&  dgfBlocks,
                     const std::vector<uint32_t>& m_surfaceNormals = {},
                     const std::vector<uint32_t>& m_vertexNormals  = {});

    bool UploadGpuResources(ID3D12Device*              device,
                            ID3D12CommandQueue*        cmdQueue,
                            ID3D12CommandAllocator*    cmdAlloc,
                            ID3D12GraphicsCommandList* cmdList);

    const std::vector<uint8_t>& GetDGFBlocks() const;
    const std::vector<uint2>&   GetDispatchplan() const;

    size_t                    GetNumDGFBlocks() const;
    D3D12_GPU_VIRTUAL_ADDRESS GetDGFBlockBuffer() const;
    D3D12_GPU_VIRTUAL_ADDRESS GetSurfaceNormalsBuffer() const;
    D3D12_GPU_VIRTUAL_ADDRESS GetVertexNormalsBuffer() const;
    D3D12_GPU_VIRTUAL_ADDRESS GetIDToBlockMapVirtualAddress() const;
    float4x4                  GetNormalizationTransformation() const;

    bool HasSurfaceNormals() const;
    bool HasVertexNormals() const;

    size_t GetNumBlockVertices() const;
    size_t GetNumBlockTriangles() const;

private:
    void LoadBinary(const std::filesystem::path pathToFile);
    void CreateReferenceMesh();
    void PopulateVertexOffsets();
    void Update();
    void BuildIDToBlockMap();

    Microsoft::WRL::ComPtr<ID3D12Resource> m_idToBlockMapResource;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_blockResource;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_surfaceNormalsResource;
    Microsoft::WRL::ComPtr<ID3D12Resource> m_vertexNormalsResource;

    std::vector<uint2> m_dispatchPlan;

    std::vector<uint8_t>  m_blocks;
    std::vector<uint32_t> m_surfaceNormals;
    std::vector<uint32_t> m_vertexNormals;
    std::vector<uint32_t> m_idToBlockMap;

    std::vector<float>    m_referencePositions;
    std::vector<uint32_t> m_referenceTriangles;

    size_t m_sumBlockTris  = 0;
    size_t m_sumBlockVerts = 0;
};

Mesh ParsePlyFile(const std::filesystem::path inputFile);
Mesh ParseObjFile(const std::filesystem::path inputFile, bool discardMaterials);

std::wstring OpenFileDialog();

DGFBaker::BakerMesh CreateDGFBakerMesh(const Mesh& mesh);

std::vector<float> ComputeScaledSurfaceNormals(const uint* const  indices,
                                         const size_t       numIndices,
                                         const float* const vertexPositions,
                                         const size_t       numVertices);

void NormalizeScaledSurfaceNormals(float* const surfaceNormal, const size_t numSurfaces);

std::vector<float> ComputeVertexNormals(const uint* const  indices,
                                        const size_t       numIndices,
                                        const float* const vertexPositions,
                                        const size_t       numVertices,
                                        const float* const surfaceNormals = nullptr);

std::vector<uint32_t> PackFloat3x10(const float* const unpackedFloats, const size_t numFloats);