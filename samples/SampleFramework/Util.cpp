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


#define NOMINMAX

#include "Util.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include "miniply.h"
#include <shobjidl.h>
#include <shlobj.h>

namespace {
    std::vector<std::vector<uint>> MapTriangleIndexToVertex(const uint* const indices,
                                                            const size_t      numTriangles,
                                                            const size_t      numVertices)
    {
        std::vector<std::vector<uint>> indexMapping(numVertices);
        for (size_t i = 0; i < numVertices; i++) {
            indexMapping[i].reserve(10);
        }

        uint3 const* const triangles = reinterpret_cast<uint3 const* const>(indices);

        for (size_t i = 0; i < numTriangles; i++) {
            const auto& triangle      = triangles[i];
            uint        triangleIndex = static_cast<uint>(i);

            assert(triangle.x < numVertices);
            assert(triangle.y < numVertices);
            assert(triangle.z < numVertices);

            indexMapping[triangle.x].push_back(triangleIndex);
            indexMapping[triangle.y].push_back(triangleIndex);
            indexMapping[triangle.z].push_back(triangleIndex);
        }

        return indexMapping;
    }

    uint PackFloat3x10(const float3 normal)
    {
        // Convert from [-1, 1] to [0, 1023] (for 10 bits)
        uint x = (uint)((normal.x * 0.5f + 0.5f) * 1023.0f + 0.5f);
        uint y = (uint)((normal.y * 0.5f + 0.5f) * 1023.0f + 0.5f);
        uint z = (uint)((normal.z * 0.5f + 0.5f) * 1023.0f + 0.5f);

        // Pack the three 10-bit values into a single 32-bit integer
        return (x & 0x3FF) | ((y & 0x3FF) << 10) | ((z & 0x3FF) << 20);
    }

    template <class T>
    void UploadBuffer(api::Device* device,
        api::CommandList* cmdList,
        ComPtr<api::Buffer>& resource,
        const std::vector<T>& data, bool uav=false)
    {
        if (data.size() == 0)
            return;
        if (uav)
            resource = device->CreateUAVBuffer(data.size() * sizeof(T));
        else
            resource = device->CreateBuffer(data.size() * sizeof(T));
        cmdList->InitializeBuffer(resource.Get(), data.data(), data.size() * sizeof(T));
    }

    std::vector<uint> Factorization(const uint n)
    {
        std::vector<uint> result;

        uint f  = 2;
        uint nl = n;

        while (f * f <= nl) {
            if (nl % f == 0) {
                result.emplace_back(f);
                nl /= f;
            } else {
                f++;
            }
        }
        if (nl != 1) {
            result.emplace_back(nl);
        }
        return result;
    }

    uint2 DispatchGrid(const uint numGroups, const uint maxThreadsPerDimension, const uint maxThreadsPerThreadGroups)
    {
        if (numGroups >= maxThreadsPerThreadGroups) {
            return uint2(0, 0);
        } else if (numGroups <= maxThreadsPerDimension) {
            return uint2(numGroups, 1);
        } else {
            const auto factors = Factorization(numGroups);
            uint       i       = 0;
            uint2      result(1, 1);
            while (i < factors.size() && result.x * factors[i] <= maxThreadsPerDimension) {
                result.x *= factors[i++];
            }
            while (i < factors.size() && result.y * factors[i] <= maxThreadsPerDimension) {
                result.y *= factors[i++];
            }
            return result;
        }
    }

    std::vector<DGFMesh::DispatchPlanEntry> ComputeDispatchPlan(const uint numGroups, const uint maxGroupsPerDimension)
    {
        std::vector<DGFMesh::DispatchPlanEntry> plan;
        for (uint i = 0; i < numGroups;) {
            const auto groupSize = std::min<uint>(numGroups - i, maxGroupsPerDimension);
            plan.emplace_back(i, groupSize, (uint32_t)(api::CONSTANT_BUFFER_ALIGNMENT * plan.size()));
            i += groupSize;
        }
        return plan;
    }

    float4x4 GetNormalizationTransformation(float const* const positions3D, size_t numberOfPositions)
    {
        float3 const* const positions = reinterpret_cast<float3 const* const>(positions3D);
        float3              lower(
            std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        float3 upper(
            -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
        for (int i = 0; i < numberOfPositions; i++) {
            lower.x = std::min(lower.x, positions[i].x);
            lower.y = std::min(lower.y, positions[i].y);
            lower.z = std::min(lower.z, positions[i].z);
            if (positions[i].x != std::numeric_limits<float>::infinity()) {
                upper.x = std::max(upper.x, positions[i].x);
            }
            if (positions[i].y != std::numeric_limits<float>::infinity()) {
                upper.y = std::max(upper.y, positions[i].y);
            }
            if (positions[i].z != std::numeric_limits<float>::infinity()) {
                upper.z = std::max(upper.z, positions[i].z);
            }
        }
        const auto diff      = upper - lower;
        const auto invLength = 1.0f / std::max(std::max(diff.x, diff.y), diff.z);
        float4x4   result;
        result.m00 = result.m11 = result.m22 = invLength;
        result.m03                           = -(lower.x + upper.x) * invLength * 0.5f;
        result.m13                           = -(lower.y + upper.y) * invLength * 0.5f;
        result.m23                           = -(lower.z + upper.z) * invLength * 0.5f;
        result.m33                           = 1.0f;
        return result;
    }

    float4x4 GetNormalizationTransformation(const std::vector<float>& positions)
    {
        return ::GetNormalizationTransformation(reinterpret_cast<float const* const>(positions.data()),
                                                positions.size() / 3);
    }

}  // namespace

float4x4 operator*(const float4x4& l, const float4x4& r)
{
    float4x4 ret;
    for (size_t i = 0; i < 4; i++)
    {
        for (size_t j = 0; j < 4; j++)
        {
            float x = l(i, 0) * r(0,j);
            for (size_t k = 1; k < 4; k++)
                x += l(i, k) * r(k, j);
            ret.d[4 * i + j] = x;
        }
    }
    return ret;
}

DGFMesh::DGFMesh(const std::filesystem::path pathToFile)
{
    LoadBinary(pathToFile);
    Update();
}

DGFMesh::DGFMesh(const DGFBaker::BakerOutput& bakerOutput, const Mesh& originalMesh)
    : m_blocks(bakerOutput.dgfBlocks),
      m_vertexTable(bakerOutput.vertexTable)
{
    if (!originalMesh.surfaceNormals.empty()) {
        m_surfaceNormals.resize(originalMesh.surfaceNormals.size());
        ::RemapTriangleAttribute(originalMesh.surfaceNormals, m_surfaceNormals, bakerOutput.triangleRemap, 1);
    }

    if (!originalMesh.vertexNormals.empty()) {
        m_vertexNormals.reserve(2 * originalMesh.vertexNormals.size());
        RemapVertexAttribute(originalMesh.vertexNormals, m_vertexNormals, bakerOutput, 1);
    }

    Update();
}

void DGFMesh::Update()
{
    m_dispatchPlan =
        ComputeDispatchPlan(static_cast<uint>(GetNumDGFBlocks()), api::MAX_THREAD_GROUP_DIM);
    CreateReferenceMesh();
    BuildIDToBlockMap();
    PopulateVertexOffsets();
}

void DGFMesh::LoadBinary(const std::filesystem::path pathToFile)
{
    std::ifstream file;
    file.open(pathToFile, std::ios::binary | std::ios::ate);
    if (file.fail()) {
        throw std::runtime_error(std::format("Error loading DGF file \"{}\".", pathToFile.string()));
    }
    size_t filesize = file.tellg();
    m_blocks.resize(filesize);
    file.seekg(0, std::ios::beg);
    file.read(reinterpret_cast<char*>(m_blocks.data()), filesize);
    file.close();
    if (!file.good()) {
        throw std::runtime_error(std::format("Error loading DGF file \"{}\".", pathToFile.string()));
    }
}

api::DGFBLASDesc DGFMesh::GetDGFBLASDesc() const
{
    api::DGFBLASDesc desc;
    desc.blocks = GetDGFBlockBuffer();
    desc.blocksByteOffset = 0;
    desc.maxGeometryIndex = GetMaxGeometryIndex();
    desc.maxPrimitiveIndex = GetMaxPrimitiveIndex();
    desc.numBlocks = (uint32_t)GetNumDGFBlocks();
    desc.numTriangles = (uint32_t)GetNumBlockTriangles();
    desc.numVertices = (uint32_t)GetNumBlockVertices();
    return desc;
}

const std::vector<uint8_t>& DGFMesh::GetDGFBlocks() const
{
    return m_blocks;
}

size_t DGFMesh::GetNumDGFBlocks() const
{
    return m_blocks.size() / DGF::BLOCK_SIZE;
}

const std::vector<DGFMesh::DispatchPlanEntry>& DGFMesh::GetDispatchplan() const
{
    return m_dispatchPlan;
}

float4x4 DGFMesh::GetNormalizationTransformation() const
{
    return ::GetNormalizationTransformation(m_referencePositions);
}

size_t DGFMesh::GetNumBlockVertices() const
{
    return m_sumBlockVerts;
};

size_t DGFMesh::GetNumBlockTriangles() const
{
    return m_sumBlockTris;
};

bool DGFMesh::UploadGpuResources(api::Device* device)
{

    api::CommandList* cmdList = device->BeginCommandList();

    UploadBuffer(device, cmdList, m_vertexNormalsResource, m_vertexNormals);
    UploadBuffer(device, cmdList, m_idToBlockMapResource, m_idToBlockMap);
    UploadBuffer(device, cmdList, m_blockResource, m_blocks, true );
    UploadBuffer(device, cmdList, m_surfaceNormalsResource, m_surfaceNormals);
    UploadBuffer(device, cmdList, m_vertexTableResource, m_vertexTable);

    struct alignas(api::CONSTANT_BUFFER_ALIGNMENT) DispatchPlanEntry
    {
        uint blockStart;
    };
    std::vector<DispatchPlanEntry> dispatchPlanEntries(m_dispatchPlan.size());
    for (size_t i = 0; i < m_dispatchPlan.size(); i++)
        dispatchPlanEntries[i].blockStart = m_dispatchPlan[i].blockStart;
    
    UploadBuffer(device, cmdList, m_dispatchPlanResource, dispatchPlanEntries);

    device->SubmitCommandList(cmdList);
    device->WaitForGPUIdle(); // drain GPU so we can delete the upload buffers

    return true;
}

void DGFMesh::CreateReferenceMesh()
{
    m_sumBlockTris  = 0;
    m_sumBlockVerts = 0;
    m_maxPrimIndex = 0;
    m_maxGeomIndex = 0;

    const auto numBlocks = GetNumDGFBlocks();
    for (size_t block = 0; block < numBlocks; block++) {
        const uint8_t* pBlock = m_blocks.data() + block * DGF::BLOCK_SIZE;

        DGF::MetaData meta;
        DGF::DecodeMetaData(&meta, pBlock);

        uint8_t tmpIndices[DGF::MAX_INDICES];
        DGF::DecodeTriangleList(tmpIndices, pBlock);

        uint8_t  opaqueFlags[DGF::MAX_TRIS];
        uint32_t geomID[DGF::MAX_TRIS];
        DGF::DecodeGeomIDs(geomID, opaqueFlags, pBlock);

        for( size_t i=0; i<meta.numTris; i++ )
            m_maxGeomIndex = std::max(m_maxGeomIndex, geomID[i]);

        m_maxPrimIndex = std::max(m_maxPrimIndex, meta.primIDBase + (meta.numTris - 1));

        DGF::OffsetVert offsetVerts[DGF::MAX_VERTS];
        DGF::FloatVert  floatVerts[DGF::MAX_VERTS];
        DGF::DecodeOffsetVerts(meta.numVerts, offsetVerts, pBlock);
        DGF::ConvertOffsetsToFloat(meta.numVerts, floatVerts, offsetVerts, meta);

        uint8_t triangleOMMIndices[DGF::MAX_TRIS];
        DGF::DecodeTriangleOMMIndices(triangleOMMIndices, pBlock);

        for (size_t i = 0; i < meta.numTris; i++) {
            m_referenceTriangles.emplace_back(tmpIndices[3 * i + 0]);
            m_referenceTriangles.emplace_back(tmpIndices[3 * i + 1]);
            m_referenceTriangles.emplace_back(tmpIndices[3 * i + 2]);
        }
        for (size_t i = 0; i < meta.numVerts; i++) {
            m_referencePositions.emplace_back(floatVerts[i].xyz[0]);
            m_referencePositions.emplace_back(floatVerts[i].xyz[1]);
            m_referencePositions.emplace_back(floatVerts[i].xyz[2]);
        }

        m_sumBlockTris += meta.numTris;
        m_sumBlockVerts += meta.numVerts;
    }
}

void DGFMesh::BuildIDToBlockMap()
{
    m_idToBlockMap              = DGFBaker::GenerateDGFBlockMap(m_blocks.data(), m_blocks.size()/DGF::BLOCK_SIZE, m_sumBlockTris, 32);
}

void DGFMesh::PopulateVertexOffsets()
{
    uint32_t   offset    = 0;
    const auto numBlocks = m_blocks.size() / DGF::BLOCK_SIZE;
    for (size_t block = 0; block < numBlocks; block++) {
        uint8_t* pBlock = m_blocks.data() + block * DGF::BLOCK_SIZE;

        // insert per-block vertex offsets into the UserData field
        DGF::WriteUserData(pBlock, &offset, 0, sizeof(offset));
        offset += (uint32_t) DGF::DecodeVertexCount(pBlock);
    }
}

bool DGFMesh::HasSurfaceNormals() const
{
    return !m_surfaceNormals.empty();
}

bool DGFMesh::HasVertexNormals() const
{
    return !m_vertexNormals.empty();
}

Mesh ParseObjFile(const std::filesystem::path inputFile, bool discardMaterials)
{
    tinyobj::attrib_t                attrib;
    std::vector<tinyobj::shape_t>    shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;

    std::filesystem::path materialBaseDir       = inputFile.parent_path();
    auto                  materialBaseDirString = materialBaseDir.generic_string();

    bool ret = tinyobj::LoadObj(
        &attrib, &shapes, &materials, &warn, &err, inputFile.generic_string().c_str(), materialBaseDirString.c_str(), false);

    if (discardMaterials) {
        materials.clear();
        printf("Discarded materials from obj\n");
    }

    if (!warn.empty()) {
        printf(warn.c_str());
    }

    if (!err.empty()) {
        printf(err.c_str());
    }

    if (!ret) {
        throw std::runtime_error("Loading Obj file failed");
    }

    Mesh mesh;
    
    // eliminate stuff we don't use here
    attrib.colors.clear();
    attrib.texcoords.clear();
    attrib.skin_weights.clear();
    attrib.texcoord_ws.clear();
    attrib.vertex_weights.clear();

    std::unordered_map<uint64_t, uint32_t> vertexMap;
    uint32_t                               numMergedVertices = 0;

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            
            bool needsTriangulation = (fv == 4);

			std::vector<uint32_t> indices;

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                
                uint64_t vertexKey = (uint32_t) idx.vertex_index;
                vertexKey          = (vertexKey << 32) + (uint32_t)(idx.normal_index);
                if (vertexMap.find(vertexKey) == vertexMap.end())
                {
                    // create new vertex
                    uint32_t mergedVertexIndex = numMergedVertices++;
                    vertexMap[vertexKey] = mergedVertexIndex;
                }
                indices.push_back(vertexMap[vertexKey]);
            }

            // per-face material
            int materialID = shapes[s].mesh.material_ids[f];
            if (materialID == -1 || discardMaterials)
                materialID = 0;  // no material... use first material
            mesh.materialIDs.push_back(materialID);
            
            if (needsTriangulation) {
				mesh.indices.push_back(indices[0]);
				mesh.indices.push_back(indices[1]);
				mesh.indices.push_back(indices[2]);

				mesh.indices.push_back(indices[0]);
				mesh.indices.push_back(indices[2]);
				mesh.indices.push_back(indices[3]);

				mesh.materialIDs.push_back(materialID);

			}
			else {
				mesh.indices.insert(mesh.indices.end(), indices.begin(), indices.end());
			}

			index_offset += fv;
        }
    }

    // fuse vertices and normals into unique verts
    mesh.vertices.resize(3 * numMergedVertices);
    
    std::vector<float> uncompactedNormals;    
    const bool hasNormals = !attrib.normals.empty();
    if (hasNormals)
    {
        uncompactedNormals.resize(3 * numMergedVertices);
    }

    for (const auto& pair : vertexMap) {
        int32_t  vertex_index = (pair.first >> 32);
        int32_t  normal_index = (pair.first & 0xffffffff);
        uint32_t mergedIndex  = pair.second;
        if (vertex_index >= 0) {
            mesh.vertices[3 * mergedIndex + 0] = attrib.vertices[3 * vertex_index + 0];
            mesh.vertices[3 * mergedIndex + 1] = attrib.vertices[3 * vertex_index + 1];
            mesh.vertices[3 * mergedIndex + 2] = attrib.vertices[3 * vertex_index + 2];
        } else {
            mesh.vertices[3 * mergedIndex + 0] = 0.0f;
            mesh.vertices[3 * mergedIndex + 1] = 0.0f;
            mesh.vertices[3 * mergedIndex + 2] = 1.0f;
        }
        if (hasNormals) {
            if (normal_index >= 0) {
                uncompactedNormals[3 * mergedIndex + 0] = attrib.normals[3 * normal_index + 0];
                uncompactedNormals[3 * mergedIndex + 1] = attrib.normals[3 * normal_index + 1];
                uncompactedNormals[3 * mergedIndex + 2] = attrib.normals[3 * normal_index + 2];
            } else {
                uncompactedNormals[3 * mergedIndex + 0] = 0.0f;
                uncompactedNormals[3 * mergedIndex + 1] = 0.0f;
                uncompactedNormals[3 * mergedIndex + 2] = 1.0f;
            }
        }
    }

    mesh.numMaterials = materials.size();

    // Compact the normals
    if (!uncompactedNormals.empty()) {
        assert(uncompactedNormals.size() % 3 == 0);

        mesh.vertexNormals.reserve(uncompactedNormals.size() / 3);

        for (size_t i = 0; i < uncompactedNormals.size(); i += 3) {
            mesh.vertexNormals.emplace_back(PackFloat3x10(
                normalize(float3(uncompactedNormals[i + 0], uncompactedNormals[i + 1], uncompactedNormals[i + 2]))));
        }
    }

    return mesh;
}

Mesh ParsePlyFile(const std::filesystem::path inputFile)
{
    miniply::PLYReader reader(inputFile.generic_string().c_str());
    if (!reader.valid()) {
        throw std::runtime_error("PLY reader is invalid");
    }

    uint32_t           indexes[3];
    bool               gotVerts = false;
    Mesh               mesh;
    std::vector<float> uncompactedNormals;

    while (reader.has_element()) {
        if (reader.element_is(miniply::kPLYVertexElement) && reader.load_element() && reader.find_pos(indexes)) {
            size_t numVerts = reader.num_rows();
            mesh.vertices.resize(3 * numVerts);
            reader.extract_properties(indexes, 3, miniply::PLYPropertyType::Float, mesh.vertices.data());
            gotVerts = true;

            if (reader.find_normal(indexes)) {
                uncompactedNormals.resize(3 * numVerts);
                reader.extract_properties(indexes, 3, miniply::PLYPropertyType::Float, uncompactedNormals.data());
            }
        } else if (reader.element_is(miniply::kPLYFaceElement) && reader.load_element() && reader.find_indices(indexes))
        {
            bool polys = reader.requires_triangulation(indexes[0]);
            if (polys && !gotVerts) {
                fprintf(stderr, "Error: need vertex positions to triangulate faces.\n");
                break;
            }
            if (polys) {
                size_t num_indices = reader.num_triangles(indexes[0]) * 3;
                mesh.indices.resize(num_indices);
                reader.extract_triangles(indexes[0],
                                         mesh.vertices.data(),
                                         static_cast<uint32_t>(mesh.vertices.size()) / 3,
                                         miniply::PLYPropertyType::Int,
                                         mesh.indices.data());
            } else {
                size_t num_indices = reader.num_rows() * 3;
                mesh.indices.resize(num_indices);
                reader.extract_list_property(indexes[0], miniply::PLYPropertyType::Int, mesh.indices.data());
            }
        }

        reader.next_element();
    }

    if (mesh.vertices.empty() || mesh.indices.empty()) {
        throw std::runtime_error("PLY file does not have verts and faces");
    }

    // Compact the normals
    if (!uncompactedNormals.empty()) {
        assert(uncompactedNormals.size() % 3 == 0);

        mesh.vertexNormals.reserve(uncompactedNormals.size() / 3);

        for (size_t i = 0; i < uncompactedNormals.size(); i += 3) {
            mesh.vertexNormals.emplace_back(PackFloat3x10(
                normalize(float3(uncompactedNormals[i + 0], uncompactedNormals[i + 1], uncompactedNormals[i + 2]))));
        }
    }
    return mesh;
}

inline std::string HrToString(HRESULT hr)
{
    char s_str[64] = {};
    sprintf_s(s_str, "HRESULT of 0x%08X", static_cast<UINT>(hr));
    return std::string(s_str);
}

class HrException : public std::runtime_error 
{
public:
    HrException(HRESULT hr) : std::runtime_error(HrToString(hr)), m_hr(hr) {}
    HRESULT Error() const
    {
        return m_hr;
    }

private:
    const HRESULT m_hr;
};
inline void ThrowIfFailed(HRESULT hr)
{
    if (FAILED(hr)) {
        throw HrException(hr);
    }
}

std::wstring OpenFileDialog( const wchar_t* fileType, const wchar_t* extensions )
{
    if (FAILED(CoInitializeEx(NULL, COINIT_MULTITHREADED | COINIT_DISABLE_OLE1DDE))) {
        return L"";
    }

    PWSTR pszFilePath;
    try {
        ComPtr<IFileOpenDialog> pFileOpen;
        ComPtr<IShellItem>      pItem;
        COMDLG_FILTERSPEC       ComDlgFS[2] = {{fileType,extensions}, {L"All Files", L"*.*"}};

        ThrowIfFailed(CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_ALL, IID_PPV_ARGS(&pFileOpen)));
        ThrowIfFailed(pFileOpen->SetFileTypes(2, ComDlgFS));
        ThrowIfFailed(pFileOpen->Show(NULL));
        ThrowIfFailed(pFileOpen->GetResult(&pItem));
        ThrowIfFailed(pItem->GetDisplayName(SIGDN_FILESYSPATH, &pszFilePath));

    } catch (...) {
        CoUninitialize();
        return L"";
    }
    CoUninitialize();
    return pszFilePath;
}

DGFBaker::BakerMesh CreateDGFBakerMesh(const Mesh& mesh)
{
    // Provide a function for the baker to retrieve mesh vertices
    auto _VertexReader = [&mesh](float* vertices, const uint32_t* vertexIndices, size_t numVerts) {
        for (size_t i = 0; i < numVerts; i++) {
            size_t idx = vertexIndices[i];
            for (size_t j = 0; j < 3; j++) vertices[3 * i + j] = mesh.vertices[3 * idx + j];
        }
    };

    // Provide a function for the baker to retrieve triangle indices
    auto _IndexReader = [&mesh](uint32_t* indices, const uint32_t* triangleIndices, size_t numTris) {
        for (size_t i = 0; i < numTris; i++) {
            size_t triIndex = triangleIndices[i];
            for (size_t j = 0; j < 3; j++) indices[3 * i + j] = mesh.indices[3 * triIndex + j];
        }
    };

    // Provide a function for the baker to retrieve triangle attributes
    auto _AttributeReader = [&mesh](DGFBaker::TriangleAttributes* out, const uint32_t* triIndices, size_t numTris) {
        if (mesh.materialIDs.empty()) {
            DGFBaker::TriangleAttributes defaultAttribs;
            for (size_t i = 0; i < numTris; i++) out[i] = defaultAttribs;
        } else {
            for (size_t i = 0; i < numTris; i++) {
                DGFBaker::TriangleAttributes attribs;
                attribs.geomID   = mesh.materialIDs[triIndices[i]];
                attribs.opaque   = true;
                attribs.hasOMM   = false;
                attribs.ommIndex = 0;
                out[i]           = attribs;
            }
        }
    };

    size_t numVerts = mesh.vertices.size() / 3;
    size_t numTris  = mesh.indices.size() / 3;
    return DGFBaker::BakerMesh(_VertexReader, _IndexReader, _AttributeReader, numVerts, numTris);
}

size_t Mesh::GetMeshSizeInBytes() const
{
    return vertices.size() * sizeof(vertices[0]) + indices.size() * sizeof(indices[0]) +
           materialIDs.size() * sizeof(materialIDs[0]) + vertexNormals.size() * sizeof(vertexNormals[0]) +
           surfaceNormals.size() * sizeof(surfaceNormals[0]);
}

std::vector<float> ComputeScaledSurfaceNormals(const uint* const  indices,
                                               const size_t       numIndices,
                                               const float* const vertexPositions,
                                               const size_t       numVertices)
{
    assert(numIndices % 3 == 0);
    const auto numTriangles = numIndices / 3;

    float3 const* const positions = reinterpret_cast<float3 const* const>(vertexPositions);
    uint3 const* const  triangles = reinterpret_cast<uint3 const* const>(indices);

    std::vector<float> normals;
    normals.reserve(3 * numTriangles);

    for (size_t i = 0; i < numTriangles; ++i) {
        const auto triangle = triangles[i];

        const float3 v0 = positions[triangle.y] - positions[triangle.x];
        const float3 v1 = positions[triangle.z] - positions[triangle.x];

        const auto normal = cross(v0, v1);

        normals.emplace_back(normal.x);
        normals.emplace_back(normal.y);
        normals.emplace_back(normal.z);
    }
    assert(normals.size() == 3 * numTriangles);
    return normals;
}

void NormalizeScaledSurfaceNormals(float* const surfaceNormal, const size_t numSurfaces) 
{
    assert(surfaceNormal != nullptr);
    assert(numSurfaces != 0);
    
    auto surfaceNormalFloat3 = reinterpret_cast<float3* const>(surfaceNormal);    
    for (size_t i = 0; i < numSurfaces; ++i) {
        
        surfaceNormalFloat3[i] = normalize(surfaceNormalFloat3[i]);        
    }
}


std::vector<float> ComputeVertexNormals(const uint* const  indices,
                                        const size_t       numIndices,
                                        const float* const vertexPositions,
                                        const size_t       numVertices,
                                        const float* const surfaceNormals)
{
    assert(numIndices % 3 == 0);
    const auto numTriangles = numIndices / 3;

    std::vector<float> surfaceNormalVector;
    float3 const*      surfaceNormalPointer = nullptr;

    if (surfaceNormals) {
        surfaceNormalPointer = reinterpret_cast<float3 const*>(surfaceNormals);
    } else {
        surfaceNormalVector  = ComputeScaledSurfaceNormals(indices, numIndices, vertexPositions, numVertices);
        surfaceNormalPointer = reinterpret_cast<float3 const*>(surfaceNormalVector.data());
    }

    const auto vertexIndexToTriangleMap = MapTriangleIndexToVertex(indices, numTriangles, numVertices);

    std::vector<float> vertexNormals;
    vertexNormals.reserve(3 * numVertices);

    for (size_t i = 0; i < numVertices; ++i) {
        const auto triangleIndices = vertexIndexToTriangleMap[i];
        float3     vertexNormal;
        for (const auto& triangleIndex : triangleIndices) {
            vertexNormal += surfaceNormalPointer[triangleIndex];
        }
        vertexNormal = normalize(vertexNormal);
        vertexNormals.emplace_back(vertexNormal.x);
        vertexNormals.emplace_back(vertexNormal.y);
        vertexNormals.emplace_back(vertexNormal.z);
    }
    assert(vertexNormals.size() == 3 * numVertices);

    return vertexNormals;
}

std::vector<uint32_t> PackFloat3x10(const float* const unpackedFloat3s, const size_t numFloat3s)
{
    std::vector<uint32_t> packed;
    packed.reserve(numFloat3s);
    float3 const* const float3Pointer = reinterpret_cast<float3 const* const>(unpackedFloat3s);

    for (size_t i = 0; i < numFloat3s; ++i) {
        packed.push_back(PackFloat3x10(float3Pointer[i]));
    }
    assert(packed.size() == numFloat3s);
    return packed;
}

static std::uint32_t CountBinaryUnitMultiples(std::uint64_t value)
{
    value /= 1024;
    return (value > 0) ? 1 + CountBinaryUnitMultiples(value) : 0;
}

void PrintByteSizeInBinaryUnits(std::ostream& ostream,
    const std::uint64_t value,
    BinaryByteUnit      binaryUnit)
{
    // Compute the byte size in binary units.
    double byteSize = static_cast<double>(value);

    // Automatically compute the next binary unit from value.
    if (binaryUnit == BinaryByteUnit::Auto) {
        // Count the number of multiples of 1024.
        const auto multipleCount = CountBinaryUnitMultiples(value);
        // Set the binary unit enum based on the multiple count. Add 1 as enum value 0
        // represents the auto mode.
        binaryUnit = static_cast<BinaryByteUnit>(std::min(
            multipleCount + 1, static_cast<std::underlying_type_t<BinaryByteUnit>>(BinaryByteUnit::Count) - 1));
    }

    // Compute the binary unit size based on the user input.
    for (std::int32_t count = static_cast<std::underlying_type_t<BinaryByteUnit>>(binaryUnit) - 1; count > 0;
        --count)
    {
        byteSize /= 1024.f;
    }

    const char* unitStr = "!HUGE!";
    switch (binaryUnit)
    {
    case BinaryByteUnit::Byte:
        unitStr = "B";
        break;
    case BinaryByteUnit::KibiByte:
        unitStr = "KiB";
        break;
    case BinaryByteUnit::MebiByte:
        unitStr = "MiB";
        break;
    case BinaryByteUnit::GibiByte:
        unitStr = "GiB";
        break;
    case BinaryByteUnit::TebiByte:
        unitStr = "TiB";
        break;
    }

    std::array<char, 20> buffer;
    std::snprintf(buffer.data(), buffer.size(), "%3.3f", byteSize);
    ostream << std::setw(7) << buffer.data() << " " << unitStr;
}


Mesh LoadMesh(const std::filesystem::path& pathToFile)
{
    Mesh mesh;
    if (pathToFile.extension() == ".ply") {
        mesh = std::move(ParsePlyFile(pathToFile));
    }
    else if (pathToFile.extension() == ".obj") {
        mesh = std::move(ParseObjFile(pathToFile, false));
    }
    else {
        throw std::runtime_error("Unknown file type");
    }

    if (mesh.surfaceNormals.empty()) {
        auto unpackedNormals = ComputeScaledSurfaceNormals(
            mesh.indices.data(), mesh.indices.size(), mesh.vertices.data(), mesh.vertices.size() / 3);
        NormalizeScaledSurfaceNormals(unpackedNormals.data(), unpackedNormals.size() / 3);
        mesh.surfaceNormals = std::move(PackFloat3x10(unpackedNormals.data(), unpackedNormals.size() / 3));
    }

    if (mesh.vertexNormals.empty()) {
        const auto unpackedNormals = ComputeVertexNormals(
            mesh.indices.data(), mesh.indices.size(), mesh.vertices.data(), mesh.vertices.size() / 3);

        mesh.vertexNormals = std::move(PackFloat3x10(unpackedNormals.data(), unpackedNormals.size() / 3));
    }
    return mesh;
}


std::wstring OpenDirectoryDialog()
{
    BROWSEINFO bi = { 0 };
    bi.lpszTitle = L"Select a Folder";
    bi.ulFlags = BIF_USENEWUI;  // Use new UI style for modern look
    bi.hwndOwner = NULL;

    // Open the folder picker dialog
    PIDLIST_ABSOLUTE pidl = SHBrowseForFolder(&bi);
    if (pidl != nullptr) {
        // Get the folder path from the selected item
        wchar_t path[MAX_PATH];
        if (SHGetPathFromIDList(pidl, path)) {
            // Return the path as a wide string
            return std::wstring(path);
        }
        CoTaskMemFree(pidl);
    }
    return L"";  // Return an empty string if no folder was selected
}


float3 min(const float3& a, const float3& b)
{
    return float3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
}

float3 max(const float3& a, const float3& b)
{
    return float3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
}

int3 min(const int3& a, const int3& b)
{
    return int3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
}

int3 max(const int3& a, const int3& b)
{
    return int3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
}

std::wstring GetExecutablePath()
{
	TCHAR path[MAX_PATH];

	if (!SUCCEEDED(GetModuleFileName(NULL, path, MAX_PATH))) {
		throw std::runtime_error("Could not retrieve current executable path");
	}
	std::filesystem::path temp(path);
	return temp.parent_path();
}


std::string wstring_to_string(const std::wstring& wide)
{
    int count = WideCharToMultiByte(CP_ACP, 0, wide.c_str(), (int)wide.length(), NULL, 0, NULL, NULL);
    std::string ret(count, 0);
    WideCharToMultiByte(CP_ACP, 0, wide.c_str(), -1, &ret[0], count, NULL, NULL);
    return ret;
}