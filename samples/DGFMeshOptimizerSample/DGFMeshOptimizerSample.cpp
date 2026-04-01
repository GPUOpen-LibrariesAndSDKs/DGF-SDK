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

// DGFMeshOptimizerSample
// 
// This sample demonstrates how to use meshoptimizer to build meshlets (clusters)
// and then use DGFBaker::BakeDGFPreclustered to generate DGF blocks.
//

#define _CRT_SECURE_NO_WARNINGS

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include "miniply.h"
#include "meshoptimizer.h"

#include <DGF.h>
#include <DGFBaker.h>

#include <iostream>
#include <vector>
#include <chrono>
#include <filesystem>
#include <cstring>
#include <algorithm>
#include <cmath>

//-----------------------------------------------------------------------------
// Mesh data structure
//-----------------------------------------------------------------------------
struct Mesh
{
    std::vector<float> vertices;    // 3 floats per vertex
    std::vector<uint32_t> indices;  // 3 indices per triangle
};

//-----------------------------------------------------------------------------
// Model loading functions (from DGFTester)
//-----------------------------------------------------------------------------
bool ParseOBJFile(const char* inputFile, Mesh& mesh)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;

    std::filesystem::path inputPath(inputFile);
    std::filesystem::path materialBaseDir = inputPath.parent_path();
    auto materialBaseDirString = materialBaseDir.generic_string();

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, inputFile, materialBaseDirString.c_str());

    if (!warn.empty()) {
        std::cout << warn << std::endl;
    }

    if (!err.empty()) {
        std::cerr << err << std::endl;
    }

    if (!ret) {
        return false;
    }

    mesh.vertices.reserve(attrib.vertices.size());

    // eliminate stuff we don't use here
    attrib.normals.clear();
    attrib.colors.clear();
    attrib.texcoords.clear();
    attrib.skin_weights.clear();
    attrib.texcoord_ws.clear();
    attrib.vertex_weights.clear();

    // Loop over vertices
    mesh.vertices.insert(mesh.vertices.end(), attrib.vertices.begin(), attrib.vertices.end());

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                const tinyobj::index_t& idx = shapes[s].mesh.indices[index_offset + v];
                mesh.indices.push_back(idx.vertex_index);
            }
            index_offset += fv;
        }
    }

    return true;
}

bool ParsePlyFile(const char* inputFile, Mesh& mesh)
{
    miniply::PLYReader reader(inputFile);
    if (!reader.valid()) {
        return false;
    }

    uint32_t indexes[3];
    bool gotVerts = false, gotFaces = false;

    while (reader.has_element() && (!gotVerts || !gotFaces)) {
        if (reader.element_is(miniply::kPLYVertexElement) && reader.load_element() && reader.find_pos(indexes)) {
            size_t numVerts = reader.num_rows();
            mesh.vertices.resize(3 * numVerts);
            reader.extract_properties(indexes, 3, miniply::PLYPropertyType::Float, mesh.vertices.data());
            gotVerts = true;
        }
        else if (reader.element_is(miniply::kPLYFaceElement) && reader.load_element() && reader.find_indices(indexes)) {
            bool polys = reader.requires_triangulation(indexes[0]);
            if (polys && !gotVerts) {
                fprintf(stderr, "Error: need vertex positions to triangulate faces.\n");
                break;
            }
            if (polys) {
                size_t num_indices = reader.num_triangles(indexes[0]) * 3;
                mesh.indices.resize(num_indices);
                reader.extract_triangles(indexes[0], mesh.vertices.data(), (uint32_t)mesh.vertices.size() / 3, miniply::PLYPropertyType::Int, mesh.indices.data());
            }
            else {
                size_t num_indices = reader.num_rows() * 3;
                mesh.indices.resize(num_indices);
                reader.extract_list_property(indexes[0], miniply::PLYPropertyType::Int, mesh.indices.data());
            }
            gotFaces = true;
        }
        if (gotVerts && gotFaces) {
            break;
        }
        reader.next_element();
    }

    if (!gotVerts || !gotFaces) {
        return false;
    }
    return true;
}

const char* GetExtension(const char* file)
{
    const char* end = file + strlen(file);
    while (end != file && *end != '.')
        --end;
    return end;
}

//-----------------------------------------------------------------------------
// Meshlet data structure (output from meshoptimizer)
//-----------------------------------------------------------------------------
struct MeshletData
{
    std::vector<meshopt_Meshlet> meshlets;
    std::vector<unsigned int> meshlet_vertices;
    std::vector<unsigned char> meshlet_triangles;
};

//-----------------------------------------------------------------------------
// Build meshlets using meshoptimizer
//-----------------------------------------------------------------------------
MeshletData BuildMeshlets(const Mesh& mesh, size_t maxVertices, size_t maxTriangles, size_t minTriangles, float fillWeight)
{
    MeshletData data;

    size_t indexCount = mesh.indices.size();
    size_t vertexCount = mesh.vertices.size() / 3;

    // Get maximum possible meshlet count
    size_t maxMeshlets = meshopt_buildMeshletsBound(indexCount, maxVertices, maxTriangles);

    // Allocate output buffers
    data.meshlets.resize(maxMeshlets);
    data.meshlet_vertices.resize(maxMeshlets * maxVertices);
    data.meshlet_triangles.resize(maxMeshlets * maxTriangles * 3);

    // Build meshlets using spatial algorithm
    size_t meshletCount = meshopt_buildMeshletsSpatial(
        data.meshlets.data(),
        data.meshlet_vertices.data(),
        data.meshlet_triangles.data(),
        mesh.indices.data(),
        indexCount,
        mesh.vertices.data(),
        vertexCount,
        sizeof(float) * 3,  // vertex stride
        maxVertices,
        minTriangles,
        maxTriangles,
        fillWeight
    );

    // Resize to actual count
    data.meshlets.resize(meshletCount);

    return data;
}

//-----------------------------------------------------------------------------
// Create a ClusteredMesh adapter for DGFBaker from meshoptimizer output
//-----------------------------------------------------------------------------
class MeshletClusteredMesh : public DGFBaker::ClusteredMesh
{
public:
    MeshletClusteredMesh(const Mesh& mesh, const MeshletData& meshletData)
        : ClusteredMesh(
            meshletData.meshlets.size(),
            [this](size_t cluster, float* verts, size_t maxVerts) { return ReadClusterVerts(cluster, verts, maxVerts); },
            [this](size_t cluster, uint8_t* indices, size_t maxTris) { return ReadClusterIndices(cluster, indices, maxTris); },
            [this](size_t cluster, DGFBaker::TriangleAttributes* attribs, size_t maxTris) { return ReadClusterAttribs(cluster, attribs, maxTris); }
        ),
        m_mesh(mesh),
        m_meshletData(meshletData)
    {
    }

private:
    size_t ReadClusterVerts(size_t clusterID, float* verts, size_t maxVerts)
    {
        const meshopt_Meshlet& meshlet = m_meshletData.meshlets[clusterID];
        size_t numVerts = std::min((size_t)meshlet.vertex_count, maxVerts);

        for (size_t i = 0; i < numVerts; i++)
        {
            unsigned int vertexIndex = m_meshletData.meshlet_vertices[meshlet.vertex_offset + i];
            verts[i * 3 + 0] = m_mesh.vertices[vertexIndex * 3 + 0];
            verts[i * 3 + 1] = m_mesh.vertices[vertexIndex * 3 + 1];
            verts[i * 3 + 2] = m_mesh.vertices[vertexIndex * 3 + 2];
        }

        return numVerts;
    }

    size_t ReadClusterIndices(size_t clusterID, uint8_t* indices, size_t maxTris)
    {
        const meshopt_Meshlet& meshlet = m_meshletData.meshlets[clusterID];
        size_t numTris = std::min((size_t)meshlet.triangle_count, maxTris);

        // Copy triangle indices (already in local format from meshoptimizer)
        memcpy(indices, &m_meshletData.meshlet_triangles[meshlet.triangle_offset], numTris * 3);

        return numTris;
    }

    size_t ReadClusterAttribs(size_t clusterID, DGFBaker::TriangleAttributes* attribs, size_t maxTris)
    {
        const meshopt_Meshlet& meshlet = m_meshletData.meshlets[clusterID];
        size_t numTris = std::min((size_t)meshlet.triangle_count, maxTris);

        // Default attributes for all triangles
        for (size_t i = 0; i < numTris; i++)
        {
            attribs[i] = DGFBaker::TriangleAttributes();
        }

        return numTris;
    }

    const Mesh& m_mesh;
    const MeshletData& m_meshletData;
};

//-----------------------------------------------------------------------------
// SAH calculation (from DGFTester)
//-----------------------------------------------------------------------------
float Area(float min[3], float max[3])
{
    float dx = max[0] - min[0];
    float dy = max[1] - min[1];
    float dz = max[2] - min[2];
    return dx * dy + dy * dz + dx * dz;
}

float AnalyzeBlockSAH(const DGFBaker::BakerOutputPreclustered& output, const Mesh& mesh)
{
    // Compute mesh bounding box
    float meshMin[3] = { INFINITY, INFINITY, INFINITY };
    float meshMax[3] = { -INFINITY, -INFINITY, -INFINITY };
    for (size_t i = 0; i < mesh.vertices.size(); i += 3)
    {
        for (size_t j = 0; j < 3; j++)
        {
            meshMin[j] = std::min(mesh.vertices[i + j], meshMin[j]);
            meshMax[j] = std::max(mesh.vertices[i + j], meshMax[j]);
        }
    }

    DGF::OffsetVert ov[DGF::MAX_VERTS];
    DGF::FloatVert fv[DGF::MAX_VERTS];
    float rootArea = 1.0f / Area(meshMin, meshMax);
    float sahSum = 0;

    // Iterate over all clusters and their blocks
    for (const auto& cluster : output.clusters)
    {
        size_t numBlocks = cluster.dgfBlocks.size() / DGF::BLOCK_SIZE;
        for (size_t i = 0; i < numBlocks; i++)
        {
            const uint8_t* block = cluster.dgfBlocks.data() + DGF::BLOCK_SIZE * i;
            DGF::MetaData meta;
            DGF::DecodeMetaData(&meta, block);
            DGF::DecodeOffsetVerts(meta.numVerts, ov, block);
            DGF::ConvertOffsetsToFloat(meta.numVerts, fv, ov, meta);

            float blockMin[3] = { INFINITY, INFINITY, INFINITY };
            float blockMax[3] = { -INFINITY, -INFINITY, -INFINITY };
            for (size_t v = 0; v < meta.numVerts; v++)
            {
                for (size_t j = 0; j < 3; j++)
                {
                    blockMin[j] = std::min(fv[v].xyz[j], blockMin[j]);
                    blockMax[j] = std::max(fv[v].xyz[j], blockMax[j]);
                }
            }

            sahSum += Area(blockMin, blockMax) * rootArea;
        }
    }

    return sahSum;
}

//-----------------------------------------------------------------------------
// Print statistics (similar to DGFTester)
//-----------------------------------------------------------------------------
void PrintStatistics(
    const Mesh& mesh,
    const MeshletData& meshletData,
    const DGFBaker::BakerOutputPreclustered& output,
    double meshletBuildTime,
    double dgfBakeTime)
{
    size_t numVerts = mesh.vertices.size() / 3;
    size_t numTriangles = mesh.indices.size() / 3;
    size_t inputByteSize = 4 * (mesh.vertices.size() + mesh.indices.size());
    size_t numClusters = meshletData.meshlets.size();

    printf("\n=== Input Model ===\n");
    printf("Vertices:   %zu\n", numVerts);
    printf("Triangles:  %zu\n", numTriangles);
    printf("Input size: %.2f MB\n", inputByteSize / (1024.0 * 1024.0));

    printf("\n=== Meshlet Building (meshoptimizer) ===\n");
    printf("Meshlets (clusters): %zu\n", numClusters);
    printf("Build time: %.2f ms\n", meshletBuildTime * 1000);

    // Compute meshlet statistics
    size_t minTrisPerMeshlet = SIZE_MAX;
    size_t maxTrisPerMeshlet = 0;
    size_t totalMeshletTris = 0;
    size_t minVertsPerMeshlet = SIZE_MAX;
    size_t maxVertsPerMeshlet = 0;
    size_t totalMeshletVerts = 0;

    for (const auto& meshlet : meshletData.meshlets)
    {
        minTrisPerMeshlet = std::min(minTrisPerMeshlet, (size_t)meshlet.triangle_count);
        maxTrisPerMeshlet = std::max(maxTrisPerMeshlet, (size_t)meshlet.triangle_count);
        totalMeshletTris += meshlet.triangle_count;

        minVertsPerMeshlet = std::min(minVertsPerMeshlet, (size_t)meshlet.vertex_count);
        maxVertsPerMeshlet = std::max(maxVertsPerMeshlet, (size_t)meshlet.vertex_count);
        totalMeshletVerts += meshlet.vertex_count;
    }

    printf("Tris per meshlet:  Min: %zu  Max: %zu  Mean: %.2f\n",
        minTrisPerMeshlet, maxTrisPerMeshlet, (double)totalMeshletTris / numClusters);
    printf("Verts per meshlet: Min: %zu  Max: %zu  Mean: %.2f\n",
        minVertsPerMeshlet, maxVertsPerMeshlet, (double)totalMeshletVerts / numClusters);

    printf("\n=== DGF Baking ===\n");
    printf("Bake time: %.2f ms\n", dgfBakeTime * 1000);

    // Count total blocks
    size_t totalBlocks = 0;
    size_t totalBlockBytes = 0;
    size_t totalBlockVerts = 0;
    size_t totalBlockTris = 0;

    for (const auto& cluster : output.clusters)
    {
        size_t clusterBlocks = cluster.dgfBlocks.size() / DGF::BLOCK_SIZE;
        totalBlocks += clusterBlocks;
        totalBlockBytes += cluster.dgfBlocks.size();
        totalBlockVerts += cluster.vertexTable.size();

        // Count triangles from blocks
        for (size_t b = 0; b < clusterBlocks; b++)
        {
            const uint8_t* block = cluster.dgfBlocks.data() + b * DGF::BLOCK_SIZE;
            totalBlockTris += DGF::DecodeTriangleCount(block);
        }
    }

    // Compute blocks per cluster statistics
    size_t minBlocksPerCluster = SIZE_MAX;
    size_t maxBlocksPerCluster = 0;
    for (const auto& cluster : output.clusters)
    {
        size_t clusterBlocks = cluster.dgfBlocks.size() / DGF::BLOCK_SIZE;
        minBlocksPerCluster = std::min(minBlocksPerCluster, clusterBlocks);
        maxBlocksPerCluster = std::max(maxBlocksPerCluster, clusterBlocks);
    }

    printf("Total DGF blocks: %zu\n", totalBlocks);
    printf("Blocks per cluster: Min: %zu  Max: %zu  Mean: %.2f\n",
        minBlocksPerCluster, maxBlocksPerCluster, (double)totalBlocks / numClusters);

    double bytesPerTri = (double)totalBlockBytes / numTriangles;
    double compressionRatio = (double)inputByteSize / totalBlockBytes;

    printf("\n=== Compression ===\n");
    printf("DGF size: %.2f MB\n", totalBlockBytes / (1024.0 * 1024.0));
    printf("Bytes per triangle: %.4f\n", bytesPerTri);
    printf("Compression ratio: %.2f:1\n", compressionRatio);
    printf("Block vertices: %zu (%.2f V/Tri)\n", totalBlockVerts, (double)totalBlockVerts / totalBlockTris);
    printf("Tris per block: %.2f\n", (double)totalBlockTris / totalBlocks);
    printf("Verts per block: %.2f\n", (double)totalBlockVerts / totalBlocks);
    printf("Vertex duplication factor: %.2f\n", (double)totalBlockVerts / numVerts);

    // SAH calculation
    float sah = AnalyzeBlockSAH(output, mesh);
    printf("SAH: %f\n", sah);
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    printf("DGFMeshOptimizerSample - Using meshoptimizer for clustering\n\n");

    if (argc < 2)
    {
        printf("Usage: %s <model.obj|model.ply> [options]\n", argv[0]);
        printf("\nOptions:\n");
        printf("  --max-vertices N    Maximum vertices per meshlet (default: 256)\n");
        printf("  --max-triangles N   Maximum triangles per meshlet (default: 128)\n");
        printf("  --min-triangles N   Minimum triangles per meshlet (default: 1)\n");
        printf("  --fill-weight F     Fill weight for meshlet building (default: 0.5)\n");
        printf("  --target-bits N     Target bit width for quantization (default: 16)\n");
        return 1;
    }

    const char* inputFile = argv[1];
    size_t maxVertices = 256;
    size_t maxTriangles = 128;
    size_t minTriangles = 1;
    float fillWeight = 0.5f;
    size_t targetBitWidth = 16;

    // Parse command line options
    for (int i = 2; i < argc; i++)
    {
        if (strcmp(argv[i], "--max-vertices") == 0 && i + 1 < argc)
        {
            maxVertices = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--max-triangles") == 0 && i + 1 < argc)
        {
            maxTriangles = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--min-triangles") == 0 && i + 1 < argc)
        {
            minTriangles = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--fill-weight") == 0 && i + 1 < argc)
        {
            fillWeight = (float)atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--target-bits") == 0 && i + 1 < argc)
        {
            targetBitWidth = atoi(argv[++i]);
        }
    }

    printf("Configuration:\n");
    printf("  Max vertices per meshlet:  %zu\n", maxVertices);
    printf("  Max triangles per meshlet: %zu\n", maxTriangles);
    printf("  Min triangles per meshlet: %zu\n", minTriangles);
    printf("  Fill weight:               %.2f\n", fillWeight);
    printf("  Target bit width:          %zu\n", targetBitWidth);

    // Load the mesh
    const char* extension = GetExtension(inputFile);
    Mesh mesh;

    printf("\nLoading: %s\n", inputFile);

    if (_stricmp(extension, ".obj") == 0)
    {
        if (!ParseOBJFile(inputFile, mesh))
        {
            fprintf(stderr, "Error loading OBJ file\n");
            return 1;
        }
    }
    else if (_stricmp(extension, ".ply") == 0)
    {
        if (!ParsePlyFile(inputFile, mesh))
        {
            fprintf(stderr, "Error loading PLY file\n");
            return 1;
        }
    }
    else
    {
        fprintf(stderr, "Unknown file extension: %s\n", extension);
        return 1;
    }

    // Build meshlets using meshoptimizer
    printf("Building meshlets with meshoptimizer...\n");
    auto meshletStartTime = std::chrono::high_resolution_clock::now();
    MeshletData meshletData = BuildMeshlets(mesh, maxVertices, maxTriangles, minTriangles, fillWeight);
    auto meshletEndTime = std::chrono::high_resolution_clock::now();
    double meshletBuildTime = std::chrono::duration<double>(meshletEndTime - meshletStartTime).count();

    printf("Created %zu meshlets\n", meshletData.meshlets.size());

    // Create ClusteredMesh adapter for DGFBaker
    MeshletClusteredMesh clusteredMesh(mesh, meshletData);

    // Configure DGFBaker
    DGFBaker::Config bakerConfig;
    bakerConfig.targetBitWidth = targetBitWidth;
    bakerConfig.quantizationMode = DGFBaker::QuantizationMode::TARGET_BIT_WIDTH;
    bakerConfig.packer = DGFBaker::PackerMode::SAH;
    bakerConfig.outputStream = &std::cout;
    bakerConfig.generateVertexTable = true;

    // Bake DGF blocks
    printf("Baking DGF blocks...\n");
    DGFBaker::Baker baker(bakerConfig);

    auto bakeStartTime = std::chrono::high_resolution_clock::now();
    DGFBaker::BakerOutputPreclustered output = baker.BakeDGFPreclustered(clusteredMesh);
    auto bakeEndTime = std::chrono::high_resolution_clock::now();
    double dgfBakeTime = std::chrono::duration<double>(bakeEndTime - bakeStartTime).count();

    // Print statistics
    PrintStatistics(mesh, meshletData, output, meshletBuildTime, dgfBakeTime);

    printf("\nDone!\n");
    return 0;
}
