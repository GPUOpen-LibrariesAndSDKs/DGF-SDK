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

#include <DGF.h>
#include "DGFBaker.h"

#include "api.h"
#include <assert.h>

#include <array>


#define STRING(x) #x
#define TOSTRING(x) STRING(x)
#define PING std::cout << __FILE__ << " (" << __LINE__ << "): " << __FUNCTION__ << std::endl
#define PRINT(x) std::cout << STRING(x) << " = " << (x) << std::endl
#define PRINT2(x,y) std::cout << STRING(x) << " = " << (x) << ", " << STRING(y) << " = " << (y) << std::endl
#define PRINT3(x,y,z) std::cout << STRING(x) << " = " << (x) << ", " << STRING(y) << " = " << (y) << ", " << STRING(z) << " = " << (z) << std::endl
#define PRINT4(x,y,z,w) std::cout << STRING(x) << " = " << (x) << ", " << STRING(y) << " = " << (y) << ", " << STRING(z) << " = " << (z) << ", " << STRING(w) << " = " << (w) << std::endl
#define PRINT5(x,y,z,w,a) std::cout << STRING(x) << " = " << (x) << ", " << STRING(y) << " = " << (y) << ", " << STRING(z) << " = " << (z) << ", " << STRING(w) << " = " << (w) << ", " << STRING(a) << " = " << (a) << std::endl
#define PRINT6(x,y,z,w,a,b) std::cout << STRING(x) << " = " << (x) << ", " << STRING(y) << " = " << (y) << ", " << STRING(z) << " = " << (z) << ", " << STRING(w) << " = " << (w) << ", " << STRING(a) << " = " << (a) << ", " << STRING(b) << " = " << (b) << std::endl

#define UPRINT(x) std::cout << STRING(x) << " = " << (x) << std::endl
#define UPRINT2(x,y) std::cout << STRING(x) << " = " << (x) << ", " << STRING(y) << " = " << (y) << std::endl
#define UPRINT3(x,y,z) std::cout << STRING(x) << " = " << (x) << ", " << STRING(y) << " = " << (y) << ", " << STRING(z) << " = " << (z) << std::endl
#define UPRINT4(x,y,z,w) std::cout << STRING(x) << " = " << (x) << ", " << STRING(y) << " = " << (y) << ", " << STRING(z) << " = " << (z) << ", " << STRING(w) << " = " << (w) << std::endl
#define UPRINT5(x,y,z,w,a) std::cout << STRING(x) << " = " << (x) << ", " << STRING(y) << " = " << (y) << ", " << STRING(z) << " = " << (z) << ", " << STRING(w) << " = " << (w) << ", " << STRING(a) << " = " << (a) << std::endl
#define UPRINT6(x,y,z,w,a,b) std::cout << STRING(x) << " = " << (x) << ", " << STRING(y) << " = " << (y) << ", " << STRING(z) << " = " << (z) << ", " << STRING(w) << " = " << (w) << ", " << STRING(a) << " = " << (a) << ", " << STRING(b) << " = " << (b) << std::endl

#define THROW_RUNTIME_ERROR(x){ \
    throw std::runtime_error(x);\
}                               

#define THROW_IF_FALSE(cond){\
    if(!(cond))\
        throw std::runtime_error(#cond);\
}

#define FATAL(x)   THROW_RUNTIME_ERROR(x)
#define WARNING(x) { std::cerr << "Warning: " << x << std::endl << std::flush; }

#define NOT_IMPLEMENTED FATAL(std::string(__FUNCTION__) + " not implemented")

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

    inline float3 operator*(const float3& rhs) const
    {
        return { x * rhs.x, y * rhs.y, z * rhs.z };
    }

    inline float3 operator/(const float3& rhs) const
    {
        return { x / rhs.x, y / rhs.y, z / rhs.z };
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

    float3(float v) : x(v), y(v), z(v) {}

    bool operator==(const float3& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }

    __forceinline static const float3 min(const float3& a, const float3& b) {
        return float3(std::min(a.x,b.x), std::min(a.y, b.y), std::min(a.z, b.z));
    }

    __forceinline static const float3 max(const float3& a, const float3& b) {
        return float3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
    }

    friend std::ostream& operator<<(std::ostream& os, const float3& v) {
        return os << "(" << v.x << "," << v.y << "," << v.z << ")";
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

inline float dot(float3 a, float3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z; ;
}


struct float2 {
    float x;
    float y;

    inline float2 operator+(const float2& rhs) const
    {
        return { x + rhs.x, y + rhs.y};
    }
    inline float2 operator-(const float2& rhs) const
    {
        return { x - rhs.x, y - rhs.y};
    }

    inline float2 operator*(const float2& rhs) const
    {
        return { x * rhs.x, y * rhs.y};
    }

    inline float2 operator/(const float2& rhs) const
    {
        return { x / rhs.x, y / rhs.y};
    }

    inline float2 operator*(const float s) const
    {
        return { x * s, y * s};
    }

    inline float2 operator+=(const float2& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    float2() : x(0), y(0) {};

    float2(float xx, float yy) : x(xx), y(yy) {}

    float2(float v) : x(v), y(v) {}

    bool operator==(const float2& other) const
    {
        return x == other.x && y == other.y;
    }

    __forceinline static const float2 min(const float2& a, const float2& b) {
        return float2(std::min(a.x, b.x), std::min(a.y, b.y));
    }

    __forceinline static const float2 max(const float2& a, const float2& b) {
        return float2(std::max(a.x, b.x), std::max(a.y, b.y));
    }

};

__forceinline float dot(float2 a, float2 b)
{
    return a.x * b.x + a.y * b.y;
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

    void SetScale(float x, float y, float z)
    {
        SetIdentity();
        m00 = x;
        m11 = y;
        m22 = z;
    }

    void SetRotationFromQuaternion(float w, float x, float y, float z)
    {
        SetIdentity();

        d[0] = 1 - 2 * (y * y + z * z);
        d[1] = 2 * (x * y + z * w);
        d[2] = 2 * (x * z - y * w);

        d[4] = 2 * (x * y - z * w);
        d[5] = 1 - 2 * (x * x + z * z);
        d[6] = 2 * (y * z + x * w);

        d[8] = 2 * (x * z + y * w);
        d[9] = 2 * (y * z - x * w);
        d[10] = 1 - 2 * (x * x + y * y);

    }

    const float& operator()(const size_t row, const size_t col) const
    {
        return d[col * 4 + row];
    }

    float& operator()(const size_t row, const size_t col)
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
    struct DispatchPlanEntry
    {
        uint32_t blockStart;
        uint32_t workgroupCount;
        uint32_t constantBufferOffset;
    };

    explicit DGFMesh() {}
    explicit DGFMesh(const DGFBaker::BakerOutput& bakerOutput, const Mesh& originalMesh);
    explicit DGFMesh(const std::filesystem::path pathToFile);


    bool UploadGpuResources(api::Device* device);

    const std::vector<uint8_t>& GetDGFBlocks() const;
    const std::vector<DispatchPlanEntry>&   GetDispatchplan() const;

    size_t                    GetNumDGFBlocks() const;
    float4x4 GetNormalizationTransformation() const;

    api::Buffer* GetDGFBlockBuffer() const { return m_blockResource.Get(); }
    api::Buffer* GetSurfaceNormalsBuffer() const { return m_surfaceNormalsResource.Get(); }
    api::Buffer* GetVertexNormalsBuffer() const { return m_vertexNormalsResource.Get(); }
    api::Buffer* GetIDToBlockMapVirtualAddress() const { return m_idToBlockMapResource.Get(); }
    api::Buffer* GetVertexTableBuffer() const { return m_vertexTableResource.Get(); } 
    api::Buffer* GetDispatchPlanConstantBuffer() const { return m_dispatchPlanResource.Get(); }

    uint32_t GetMaxGeometryIndex() const { return m_maxGeomIndex; }
    uint32_t GetMaxPrimitiveIndex() const { return m_maxPrimIndex; }

    bool HasSurfaceNormals() const;
    bool HasVertexNormals() const;

    size_t GetNumBlockVertices() const;
    size_t GetNumBlockTriangles() const;

    api::DGFBLASDesc GetDGFBLASDesc() const;

private:
    void LoadBinary(const std::filesystem::path pathToFile);
    void CreateReferenceMesh();
    void PopulateVertexOffsets();
    void Update();
    void BuildIDToBlockMap();

    Microsoft::WRL::ComPtr<api::Buffer> m_idToBlockMapResource;
    Microsoft::WRL::ComPtr<api::Buffer> m_blockResource;
    Microsoft::WRL::ComPtr<api::Buffer> m_surfaceNormalsResource;
    Microsoft::WRL::ComPtr<api::Buffer> m_vertexNormalsResource;
    Microsoft::WRL::ComPtr<api::Buffer> m_vertexTableResource;
    Microsoft::WRL::ComPtr<api::Buffer> m_dispatchPlanResource;
    std::vector<DispatchPlanEntry> m_dispatchPlan;

    std::vector<uint8_t>  m_blocks;
    std::vector<uint32_t> m_surfaceNormals;
    std::vector<uint32_t> m_vertexNormals;
    std::vector<uint32_t> m_idToBlockMap;
    std::vector<uint32_t> m_vertexTable;

    std::vector<float>    m_referencePositions;
    std::vector<uint32_t> m_referenceTriangles;

    size_t m_sumBlockTris  = 0;
    size_t m_sumBlockVerts = 0;
    uint32_t m_maxPrimIndex = 0;
    uint32_t m_maxGeomIndex = 0;
};

Mesh ParsePlyFile(const std::filesystem::path inputFile);
Mesh ParseObjFile(const std::filesystem::path inputFile, bool discardMaterials);

std::wstring OpenFileDialog( const wchar_t* fileType = L"Mesh Files", const wchar_t* fileExtensions = L"*.ply;*.obj");

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


class FreeList
{
public:
    uint32_t Alloc()
    {
        if (m_freeList.empty())
        {
            return m_indexCount++;
        }
        else
        {
            auto ret = m_freeList.back();
            m_freeList.pop_back();
            return ret;
        }
    }

    void Free(uint32_t index)
    {
        assert(index < m_indexCount);
        m_freeList.push_back(index);
    }

private:
    uint32_t m_indexCount = 0;
    std::vector<uint32_t> m_freeList;
};


template <class T>
void RemapTriangleAttribute(const std::vector<T>& input,
    std::vector<T>& output,
    const std::vector<DGFBaker::TriangleRemapInfo>& remap,
    const size_t                                    nChannels)
{
    assert(input.size() == output.size());
    for (size_t i = 0; i < remap.size(); ++i) {
        for (size_t j = 0; j < nChannels; ++j) {
            output[nChannels * i + j] = input[nChannels * remap[i].InputPrimIndex + j];
        }
    }
}

template <class T>
void RemapVertexAttribute(const std::vector<T>& input,
    std::vector<T>& output,
    const DGFBaker::BakerOutput& bakerOutput,
    const size_t           nChannels)
{
    const std::vector<uint32_t>& vertexTable = bakerOutput.vertexTable;
    const std::vector<uint8_t>& dgfBlocks = bakerOutput.dgfBlocks;

    uint32_t vertexOffset = 0;

    for (size_t i = 0; i < dgfBlocks.size(); i += DGF::BLOCK_SIZE) {
        const uint8_t* pBlock = dgfBlocks.data() + i;
        size_t   numVerts = DGF::DecodeVertexCount(pBlock);

        // build the duplicated attribute array.
        for (size_t j = 0; j < numVerts; j++) {
            uint32_t inputIndex = vertexTable[vertexOffset++];
            for (size_t j = 0; j < nChannels; ++j) {
                output.push_back(input[nChannels * inputIndex + j]);
            }
        }
    }
}


enum class BinaryByteUnit : std::uint32_t {
    // Automatically choose the next binary unit.
    Auto = 0,
    Byte = 1,
    KibiByte = 2,
    MebiByte = 3,
    GibiByte = 4,
    TebiByte = 5,
    Count,
    Default = Auto
};


void PrintByteSizeInBinaryUnits(std::ostream& ostream,
    const std::uint64_t value,
    BinaryByteUnit      binaryUnit = BinaryByteUnit::Auto);

Mesh LoadMesh(const std::filesystem::path& pathToFile);


struct int3 {
    int x;
    int y;
    int z;
    int3() : x(0), y(0), z(0) {};
    int3(int xx, int yy, int zz) : x(xx), y(yy), z(zz) {}

    int3 operator-(const int3& other) const
    {
        return int3(x - other.x, y - other.y, z - other.z);
    }

    bool operator<(const int3& other) const
    {
        return x < other.x && y < other.y && z < other.z;
    }
};

float3 min(const float3& a, const float3& b);
float3 max(const float3& a, const float3& b);
int3   min(const int3& a, const int3& b);
int3   max(const int3& a, const int3& b);


//float2 min(const float2& a, const float2& b);
//float2 max(const float2& a, const float2& b);


std::wstring OpenDirectoryDialog();

struct AABB {
    float3 Min;
    float3 Max;

    void Init()
    {
        constexpr float inf = std::numeric_limits<float>::infinity();
        Min = { inf, inf, inf };
        Max = { -inf, -inf, -inf };
    }

    AABB()
    {
        Init();
    };

    AABB(const float3 &min, const float3 &max) : Min(min), Max(max) {}

    AABB(const AABB& rhs) = default;
    AABB& operator=(const AABB& rhs) = default;

    void Expand(const float3 &pt)
    {
        Min = min(pt, Min);
        Max = max(pt, Max);
    }

    void Expand(const AABB &box)
    {
        Min = min(box.Min, Min);
        Max = max(box.Max, Max);
    }

    float HalfArea() const
    {
        float3 d = Max - Min;
        return d.x * (d.y + d.z) + d.y * d.z;
    }

    bool Contains(const AABB &box) const
    {
        return Min.x <= box.Min.x && Min.y <= box.Min.y && Min.z <= box.Min.z && Max.x >= box.Max.x &&
            Max.y >= box.Max.y && Max.z >= box.Max.z;
    }

    float3 Diagonal() const
    {
        return Max - Min;
    }


    float LongEdgeLength() const
    {
        float3 d = Max - Min;
        return std::max(d.x, std::max(d.y, d.z));
    }

    float3 Centroid() const
    {
        return (Max + Min) * 0.5f;
    }

    float SurfaceArea() const
    {
        const float3 d = Diagonal();
        return 2.0f * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    bool IsValid() const
    {
        return (Min.x <= Max.x) && (Min.y <= Max.y) && (Min.z <= Max.z);
    }

    float maxDiagDim() const
    {
        const float3 d = Diagonal();
        return std::max(std::max(d.x, d.y), d.z);
    }

    __forceinline friend std::ostream& operator<<(std::ostream& os, const AABB& p);


    __forceinline static const AABB Intersect(const AABB& a, const AABB& b) {
        float3 lower(max(a.Min, b.Min));
        float3 upper(min(a.Max, b.Max));
        return AABB(lower, upper);
    }

};

__forceinline std::ostream& operator<<(std::ostream& os, const AABB& b) {
    os << "[" << b.Min.x << ", " << b.Min.y << "," << b.Min.z << "] - ";
    os << "[" << b.Max.x << ", " << b.Max.y << "," << b.Max.z << "]";
    return os;
}

struct AABB2f
{
    float2 Min;
    float2 Max;

    void Init()
    {
        Max = float2(-FLT_MAX, -FLT_MAX);
        Min = float2(FLT_MAX, FLT_MAX);
    }

    AABB2f()
    {
        Init();
    }

    AABB2f(const float2& lower, const float2& upper) : Min(lower), Max(upper)
    {
    }

    bool IsValid() const
    {
        return (Min.x <= Max.x) && (Min.y <= Max.y);
    }

    void Union(float2 p)
    {
        Min.x = std::min(Min.x, p.x);
        Min.y = std::min(Min.y, p.y);
        Max.x = std::max(Max.x, p.x);
        Max.y = std::max(Max.y, p.y);
    }

    void Extend(float2 p)
    {
        Union(p);
    }

    void Union(AABB2f b)
    {
        Union(b.Min);
        Union(b.Max);
    }

    void Extend(AABB2f b)
    {
        Union(b);
    }

    float2 Diagonal() const
    {
        return Max - Min;
    }

    float MaxDiagDim() const
    {
        const float2 d = Diagonal();
        return std::max(d.x, d.y);
    }

    float2 Centroid() const
    {
        return (Max + Min) * 0.5f;
    }

    float2 Centroid2() const
    {
        return Max + Min;
    }

    float SurfaceArea() const
    {
        const float2 d = Diagonal();
        return d.x * d.y;
    }

    __forceinline static const AABB2f Intersect(const AABB2f& a, const AABB2f& b) {
        float2 lower(float2::max(a.Min, b.Min));
        float2 upper(float2::min(a.Max, b.Max));
        return AABB2f(lower, upper);
    }
    __forceinline friend std::ostream& operator<<(std::ostream& os, const AABB2f& p);

};

__forceinline std::ostream& operator<<(std::ostream& os, const AABB2f& b) {
    os << "[" << b.Min.x << ", " << b.Min.y << "] - ";
    os << "[" << b.Max.x << ", " << b.Max.y << "]";
    return os;
}



std::wstring GetExecutablePath();


std::string wstring_to_string(const std::wstring& wstr);


constexpr inline size_t RoundUpPow2(size_t n, size_t m)
{
    return (n + (m - 1)) & ~(m - 1);
}
