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


#define _CRT_SECURE_NO_WARNINGS // fopen() is not that bad

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include <iostream>
#include "miniply.h"

#include <DGF.h>
#include <DGFBaker.h>
#include <chrono>
#include <filesystem>

struct Mesh
{
    std::vector<float> vertices; // 3 floats per vertex
    std::vector<uint32_t> indices;
    std::vector<uint32_t> materialIDs; // material ID per face
    size_t numMaterials = 0;
};

bool ParseOBJFile( const char* inputFile, Mesh& mesh, bool discardMaterials )
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;

    std::filesystem::path inputPath(inputFile);
    std::filesystem::path materialBaseDir = inputPath.parent_path();
    auto materialBaseDirString = materialBaseDir.generic_string();

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, inputFile,materialBaseDirString.c_str() );

    if (discardMaterials) {
        materials.clear();
        printf("Discarded materials from obj\n");
    }

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
    for (size_t v = 0; v < attrib.vertices.size(); v++)
        mesh.vertices.push_back(attrib.vertices[v]);

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
                
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                mesh.indices.push_back(idx.vertex_index);
            }
            index_offset += fv;

            // per-face material
            int materialID = shapes[s].mesh.material_ids[f];
            if (materialID == -1 || discardMaterials )
                materialID = 0;  // no material... use first material
            mesh.materialIDs.push_back(materialID);
        }
    }

    mesh.numMaterials = materials.size();
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
                reader.extract_triangles(indexes[0], mesh.vertices.data(), (uint32_t) mesh.vertices.size()/3, miniply::PLYPropertyType::Int, mesh.indices.data());
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


DGFBaker::BakerMesh CreateDGFBakerMesh( const Mesh& mesh )
{
    // Provide a function for the baker to retrieve mesh vertices
    auto _VertexReader =
        [&mesh](float* vertices, const uint32_t* vertexIndices, size_t numVerts)
        {     
            for (size_t i = 0; i < numVerts; i++)
            {
                size_t idx = vertexIndices[i];
                for (size_t j = 0; j < 3; j++)
                    vertices[3 * i + j] = mesh.vertices[3 * idx + j];
            }   
        };

    // Provide a function for the baker to retrieve triangle indices
    auto _IndexReader =
        [&mesh](uint32_t* indices, const uint32_t* triangleIndices, size_t numTris)
        {
            for (size_t i = 0; i < numTris; i++)
            {
                size_t triIndex = triangleIndices[i];
                for (size_t j = 0; j < 3; j++)
                    indices[3 * i + j] = mesh.indices[3 * triIndex + j];
            }
        };

    // Provide a function for the baker to retrieve triangle attributes
    auto _AttributeReader = 
        [&mesh](DGFBaker::TriangleAttributes* out, const uint32_t* triIndices, size_t numTris)
        {
            if (mesh.materialIDs.empty())
            {
                DGFBaker::TriangleAttributes defaultAttribs;
                for (size_t i = 0; i < numTris; i++)
                    out[i] = defaultAttribs;
            }
            else
            {
                for (size_t i = 0; i < numTris; i++)
                {
                    DGFBaker::TriangleAttributes attribs;
                    attribs.geomID = mesh.materialIDs[triIndices[i]];
                    attribs.opaque = true;
                    attribs.hasOMM = false;
                    attribs.ommIndex = 0;
                    out[i] = attribs;
                }
            }
        };

    size_t numVerts = mesh.vertices.size() / 3;
    size_t numTris = mesh.indices.size() / 3;
    return DGFBaker::BakerMesh(_VertexReader, _IndexReader, _AttributeReader, numVerts, numTris);
}

struct CommandLine
{
    const char* inFile;
    const char* objDumpFile = nullptr;
    const char* binDumpFile = nullptr;
    const char* statsFile = nullptr;
    DGFBaker::Config bakerConfig;
    bool validate = true;
    bool discardMaterials = false;
    bool measureError = false;
};

bool ParseCommandLine( CommandLine& cmd, int argc, char** argv)
{
    if (argc < 2)
    {
        printf("USAGE: %s <infile> [OPTIONS]\n", argv[0]);
        printf("    <infile> is a wavefront obj or ply file\n");
        printf("    OPTIONS are:\n");
        printf("         --cluster-max-faces <uint> \n");
        printf("         --cluster-max-vertices <uint> \n");
        printf("         --target-bits <uint> \n");
        printf("         --print-perf \n");
        printf("         --skip-validation \n");
        printf("         --dump-obj <path>\n");
        printf("         --dump-bin <path>\n");
        printf("         --write-stats <path>\n");
        printf("         --discard-materials\n");
        printf("         --measure-error\n");
        printf("         --forced-offset-width <uint> <uint> <uint>\n");
        printf("         --user-data\n");
        printf("         --packer {HPG24|SAH}\n");
        return false;
    }

    cmd.inFile = argv[1];
    
    DGFBaker::Config& bakerConfig = cmd.bakerConfig;
    bakerConfig.clusterMaxFaces = 128;
    bakerConfig.clusterMaxVerts = 256;
    bakerConfig.targetBitWidth = 16;
    bakerConfig.outputStream = &std::cout;
    bakerConfig.generateVertexTable = true;
    bakerConfig.generateClusterTable = true;
    bakerConfig.packer = DGFBaker::PackerMode::DEFAULT;
    bakerConfig.encoderRoundTripValidation = true;
    bakerConfig.generateTriangleRemap = true;

    for (int i = 2; i < argc; )
    {
        const char* optionName = argv[i++];        
        if (strcmp(optionName, "--cluster-max-faces") == 0)
        {
            if (i != argc)
                bakerConfig.clusterMaxFaces = atoi(argv[i++]);            
        }
        else if (strcmp(optionName, "--cluster-max-vertices") == 0)
        {
            if (i != argc)
                bakerConfig.clusterMaxVerts = atoi(argv[i++]);
        }
        else if (strcmp(optionName, "--target-bits") == 0)
        {
            if (i != argc)
                bakerConfig.targetBitWidth = atoi(argv[i++]);
        }   
        else if (strcmp(optionName, "--print-perf") == 0)
        {
            bakerConfig.printPerfData = true;
        }
        else if (strcmp(optionName, "--skip-validation") == 0)
        {
            cmd.validate = false;
            cmd.bakerConfig.encoderRoundTripValidation = false;;
        }
        else if (strcmp(optionName, "--discard-materials") == 0)
        {
            cmd.discardMaterials = true;
        }
        else if (strcmp(optionName, "--dump-obj") == 0)
        {
            if (i != argc)
                cmd.objDumpFile = argv[i++];          
        }
        else if (strcmp(optionName, "--dump-bin") == 0)
        {
            if (i != argc)
                cmd.binDumpFile = argv[i++];
        }
        else if (strcmp(optionName, "--write-stats") == 0)
        {
            if (i != argc)
                cmd.statsFile = argv[i++];
        }
        else if (strcmp(optionName, "--measure-error") == 0)
        {
            cmd.measureError = true;
        }
        else if (strcmp(optionName, "--forced-offset-width") == 0)
        {
            if (i+3 <= argc)
            {
                cmd.bakerConfig.blockForcedOffsetWidth[0] = atoi(argv[i++]);
                cmd.bakerConfig.blockForcedOffsetWidth[1] = atoi(argv[i++]);
                cmd.bakerConfig.blockForcedOffsetWidth[2] = atoi(argv[i++]);
            }
        }
        else if (strcmp(optionName, "--user-data") == 0)
        {
            cmd.bakerConfig.enableUserData = true;
        }
        else if (strcmp(optionName, "--packer") == 0)
        {
            if (i != argc)
            {
                const char* packerName = argv[i++];
                cmd.bakerConfig.packer = DGFBaker::PackerModeFromString(packerName);
            }
        }
        else
        {
            printf("Spurious commandline argument: %s\n", optionName);
        }
    }

    return true;
}

bool DumpOBJ( const char* fileName, const DGFBaker::DecodedMesh& mesh )
{
    std::ofstream file;
    file.open(fileName);
    if (file.fail())
        return false;

    size_t numVerts = mesh.GetVertexCount();
    size_t numTris = mesh.GetTriangleCount();
    const float* floatVerts = mesh.GetVertexBuffer();
    const uint32_t* indices = mesh.GetIndexBuffer();

    for (size_t vert = 0; vert < numVerts; vert++) {
        file << "v ";
        for (size_t i = 0; i < 3; i++) {
            file << floatVerts[3*vert+i] << " ";
        }
        file << "\n";
    }

    file << "g mesh" << "\n";
    for (size_t i = 0; i <numTris; i++) {
        file << "f " << indices[3*i] + 1 << " " << indices[3*i + 1] + 1 << " " << indices[3*i + 2] + 1 << "\n";
    }
    file.close();
    return file.good();
}

bool DumpBIN(const char* fileName, const DGFBaker::BakerOutput& bakerOutput)
{
    std::ofstream file;
    file.open(fileName, std::ios::binary);
    if (file.fail())
    {
        return false;
    }
    file.write(reinterpret_cast<const char*>(bakerOutput.dgfBlocks.data()), bakerOutput.dgfBlocks.size());
    file.close();
    return file.good();
}


const char* GetExtension(const char* file)
{
    const char* end = file + strlen(file);
    while (end != file && *end != '.')
        --end;
    return end;
}

struct Errors
{
    double average;
    double max;
};

Errors AnalyzeError(const DGFBaker::BakerOutput& output, const Mesh& input)
{
    // compute length of mesh diagonal
    double min[3], max[3];
    for (size_t k = 0; k < 3; k++)
    {
        min[k] = FLT_MAX;
        max[k] = -FLT_MAX;
    }
    for (size_t v = 0; v < input.vertices.size(); v+= 3)
    {
        for (size_t k = 0; k < 3; k++)
        {
            double p = input.vertices[v + k];
            min[k] = std::min(min[k], p);
            max[k] = std::max(max[k], p);
        }        
    }
    double diag = 0;
    for (size_t k = 0; k < 3; k++)
        diag += (max[k] - min[k]) * (max[k] - min[k]);
    diag = sqrt(diag);

    DGFBaker::DecodedMesh decoded = DGFBaker::DecodeDGF(output);

    double maxError = 0;
    double errorSum = 0;
    size_t numVerts = output.vertexTable.size();
    for (size_t v = 0; v < numVerts; v++)
    {
        size_t idx = output.vertexTable[v];
        double distance = 0;
        for (size_t k = 0; k < 3; k++)
        {
            double decodeP = decoded.GetVertexBuffer()[3 * v + k];
            double inputP = input.vertices[3 * idx + k];
            double delta = decodeP - inputP;
            distance += delta * delta;
        }
        double error = sqrt(distance)/diag;
        maxError = std::max(error, maxError);
        errorSum += error;
    }

    double avgError = errorSum / numVerts;
    return { avgError,maxError };
}

size_t CountPaletteBlocks( const DGFBaker::BakerOutput& output )
{
    size_t palettes = 0;
    for (size_t i = 0; i < output.dgfBlocks.size(); i += 128)
    {
        DGF::MetaData meta;
        DGF::DecodeMetaData(&meta, output.dgfBlocks.data() + i);
        if (meta.geomIDMode == DGF::GEOMID_PALETTE)
            palettes++;
    }
    return palettes;
}

float Area(float min[3], float max[3])
{
    float dx = max[0] - min[0];
    float dy = max[1] - min[1];
    float dz = max[2] - min[2];
    return dx * dy + dy * dz + dx * dz;
}

float AnalyzeBlockSAH( const uint8_t* blocks, size_t numBlocks, const Mesh& mesh )
{
    float meshMin[3] = { INFINITY,INFINITY,INFINITY };
    float meshMax[3] = { -INFINITY,-INFINITY,-INFINITY };
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
    for (size_t i = 0; i < numBlocks; i++)
    {
        const uint8_t* block = blocks + DGF::BLOCK_SIZE * i;
        DGF::MetaData meta;
        DGF::DecodeMetaData(&meta, block);
        DGF::DecodeOffsetVerts(meta.numVerts, ov, block);
        DGF::ConvertOffsetsToFloat(meta.numVerts, fv, ov, meta);
        
        float blockMin[3] = { INFINITY,INFINITY,INFINITY };
        float blockMax[3] = { -INFINITY,-INFINITY,-INFINITY };
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

    return sahSum;
}


int main(int argc, char* argv[])
{    
    CommandLine cmdLine;
    if (!ParseCommandLine(cmdLine, argc, argv))
    {
        return 1;
    }

    const DGFBaker::Config& bakerConfig = cmdLine.bakerConfig;
    const char* inFile  = cmdLine.inFile;
    
    printf("DGFBaker configuration: \n");
    printf("    Cluster face limit: %zu\n", bakerConfig.clusterMaxFaces);
    printf("    Cluster vert limit: %zu\n", bakerConfig.clusterMaxVerts);
    printf("    Bit width         : %zu\n", bakerConfig.targetBitWidth);    
    printf("    Packer            : %s\n",  DGFBaker::PackerModeToString(bakerConfig.packer));

    // load the input
    const char* Extension = GetExtension(inFile);
    Mesh mesh;
    if (_stricmp(Extension, ".obj") == 0)
    {
        printf("Loading obj file: %s\n", inFile);
        if (!ParseOBJFile(inFile, mesh, cmdLine.discardMaterials ))
        {
            return 1;
        }
    }
    else if (_stricmp(Extension, ".ply") == 0)
    {
        printf("Loading ply file: %s\n", inFile);
        if (!ParsePlyFile(inFile, mesh))
        {
            return 1;
        }
    }
    else
    {
        printf("File extension is: %s.. what is this?\n", Extension);
        return 1;
    }

    // create a "baker mesh" from the input data
    size_t numTriangles = mesh.indices.size() / 3;
    size_t numVerts = mesh.vertices.size() / 3;
    size_t inputByteSize = 4 * (mesh.vertices.size() + mesh.indices.size());
    size_t numMaterials = mesh.numMaterials;
    printf("Model size: %zu verts %zu tris  %zu materials\n", numVerts, numTriangles, numMaterials);
    printf("Input size: %.2fMB\n", inputByteSize / (1024.0 * 1024.0));
    DGFBaker::BakerMesh bakerMesh = CreateDGFBakerMesh(mesh);
    
    // bake it
    printf("Baking DGF blocks...\n");
    DGFBaker::Baker baker(bakerConfig);
    
    auto startTime = std::chrono::high_resolution_clock::now();
    DGFBaker::BakerOutput dgfOutput = baker.BakeDGF(bakerMesh);
    auto endTime = std::chrono::high_resolution_clock::now();
    double executionTime = std::chrono::duration<double>(endTime - startTime).count();

    // print the size
    size_t blockByteSize = dgfOutput.dgfBlocks.size();
    size_t numBlocks = blockByteSize / 128;
    double dgfBytesPerTri = (blockByteSize) / (double)numTriangles;
    size_t numPaletteBlocks = CountPaletteBlocks(dgfOutput);
    size_t numBlockVerts = dgfOutput.vertexTable.size();
    size_t numBlockTris = dgfOutput.triangleRemap.size();

    printf("%zu DGF blocks (%.2f%% palette)\n", numBlocks, numPaletteBlocks*100.0/(double)numBlocks );
    printf("%.4f B/tri  (%.2f:1)\n", dgfBytesPerTri, inputByteSize / (double)blockByteSize);
    printf("%zu block vertices, %zu block triangles (%.2f V/Tri)\n", numBlockVerts, numBlockTris, (double)numBlockVerts/(double)numBlockTris);
    printf("%.2f Tris/Block\n", numBlockTris / (double)numBlocks);
    printf("%.2f Verts/Block\n", numVerts / (double)numBlocks);
    printf("Vertex duplication factor: %.2f\n", (double)numBlockVerts / (double)numVerts);
    printf("Vertex duplication cost per byte: %.2fB/Tri\n", (numBlockVerts - numVerts)/(double)numTriangles );

    float sah = AnalyzeBlockSAH(dgfOutput.dgfBlocks.data(), numBlocks, mesh);
    printf("SAH is: %f\n", sah);

    // check it
    if (cmdLine.validate)
    {
        printf("Validating DGF blocks...\n");
        DGFBaker::Validator val(std::cout);
        if (!val.ValidateDGF(bakerMesh, dgfOutput))
        {
            return 1;
        }
    }

    // dump it
    if (cmdLine.objDumpFile)
    {
        printf("Decoding and dumping...\n");
        DGFBaker::DecodedMesh decoded = DGFBaker::DecodeDGF(dgfOutput);
        if (!DumpOBJ(cmdLine.objDumpFile, decoded))
        {
            printf("Error writing: %s\n", cmdLine.objDumpFile);
            return 1;
        }
        printf("Wrote: %s\n", cmdLine.objDumpFile);
    }

    // dump it
    if (cmdLine.binDumpFile)
    {
        printf("Dumping...\n");
        
        if (!DumpBIN(cmdLine.binDumpFile, dgfOutput))
        {
            printf("Error writing: %s\n", cmdLine.binDumpFile);
            return 1;
        }
        printf("Wrote: %s\n", cmdLine.binDumpFile);                
    }


    Errors error = { 0 };
    if (cmdLine.measureError)
    {
        error = AnalyzeError(dgfOutput, mesh);
        printf("Max Error: %f  Average Error: %f\n",error.max, error.average );
    }

    if (cmdLine.statsFile)
    {
        printf("Writing stats to %s\n", cmdLine.statsFile);
        FILE* fp = fopen(cmdLine.statsFile, "w");
        fprintf(fp, "%s,",  cmdLine.inFile);
        fprintf(fp, "%zu,", cmdLine.bakerConfig.targetBitWidth);
        fprintf(fp, "%zu,", cmdLine.bakerConfig.clusterMaxFaces);
        fprintf(fp, "%zu,", numVerts);
        fprintf(fp, "%zu,", numTriangles);
        fprintf(fp, "%zu,", numMaterials);
        fprintf(fp, "%zu,", numBlocks);
        fprintf(fp, "%f,",  executionTime);
        fprintf(fp, "%zu,",  dgfOutput.vertexTable.size());
        fprintf(fp, "%zu,",  dgfOutput.clusterTable.size());
        fprintf(fp, "%f,",   dgfBytesPerTri);
        fprintf(fp, "%f,", error.average);
        fprintf(fp, "%f,", error.max);
        fprintf(fp, "%zu,", numPaletteBlocks);
        fprintf(fp, "%f,", sah);
        fprintf(fp, "\n");
        fclose(fp);
    }

    printf("Clean exit\n");
    return 0;
}