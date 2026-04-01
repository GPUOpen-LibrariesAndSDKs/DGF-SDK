// This file is part of the DGF-SDK.
//
// Copyright (C) 2026 Advanced Micro Devices, Inc.
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

// clang-format off

#include "CommandLine.h"

bool ParseCommandLine(CommandLine& cmd, int argc, char** argv)
{
    if (argc < 2)
    {
        printf("USAGE: %s <infile> [OPTIONS]\n", argv[0]);
        printf("    <infile> is a wavefront obj or ply file\n");
        printf("    OPTIONS are:\n");
        printf("         --cluster-max-faces <uint> \n");
        printf("         --cluster-max-vertices <uint> \n");
        printf("         --target-bits <uint> \n");
        printf("         --exponent <int> \n");
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
        printf("         --weld-vertices\n");
        printf("         --test-preclustering\n");
        printf("         --quantize-for-animation\n");
        printf("         --deformation-padding <float>\n");
        printf("         --generate-omm-ids\n");
        printf("         --random-omm-ids <num_omms>\n");
        printf("         --post-quant-welding\n");
        printf("         --sah-block-size <block_size>\n");
        printf("         --test-dgfs\n");
        printf("         --dgfs-random-primid\n");
        printf("         --dgfs-random-user-data\n");
        printf("         --dgfs-bytes-per-index <1|2|4>\n");
        printf("         --dgfs-geomid-indexing\n");
        printf("         --dgfs-compressed1-vertices\n");
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
    bakerConfig.weldVertices = false;
    bakerConfig.allowInternalThreading = true;

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
            bakerConfig.quantizationMode = DGFBaker::QuantizationMode::TARGET_BIT_WIDTH;
        }
        else if (strcmp(optionName, "--exponent") == 0)
        {
            if (i != argc)
                bakerConfig.quantizationExponent = atoi(argv[i++]);
            bakerConfig.quantizationMode = DGFBaker::QuantizationMode::EXPLICIT_EXPONENT;
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
            if (i + 3 <= argc)
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
        else if (strcmp(optionName, "--weld-vertices") == 0)
        {
            cmd.bakerConfig.weldVertices = true;
        }
        else if (strcmp(optionName, "--test-preclustering") == 0)
        {
            cmd.testPreclusterPath = true;
        }
        else if (strcmp(optionName, "--quantize-for-animation") == 0)
        {
            cmd.bakerConfig.quantizeForAnimation = true;
        }
        else if (strcmp(optionName, "--deformation-padding") == 0)
        {
            if (i != argc)
            {
                float padding = (float)std::atof(argv[i++]);
                cmd.bakerConfig.clusterDeformationPadding = padding;
            }
        }
        else if (strcmp(optionName, "--generate-omm-ids") == 0)
        {
            cmd.generateOMMIDs = true;
        }
        else if (strcmp(optionName, "--randomize-omm-ids") == 0)
        {
            cmd.randomOMMIDs = true;
            if (i != argc)
            {
                int numOMMs = (int)std::atoi(argv[i++]);
                if (numOMMs == 0)
                    numOMMs = 1;
                cmd.numRandomOMMs = numOMMs;
            }
        }
        else if (strcmp(optionName, "--no-threading") == 0)
        {
            cmd.bakerConfig.allowInternalThreading = false;
        }
        else if (strcmp(optionName, "--post-quant-welding") == 0)
        {
            cmd.bakerConfig.postQuantizationWelding = true;
        }
        else if (strcmp(optionName, "--sah-block-size") == 0)
        {
            cmd.bakerConfig.sahBlockSize = atoi(argv[i++]);
        }
        else if (strcmp(optionName, "--test-dgfs") == 0)
        {
            cmd.testDGFS = true;
        }
        else if (strcmp(optionName, "--dgfs-random-primid") == 0)
        {
            cmd.dgfsRandomPrimID = true;
        }
        else if (strcmp(optionName, "--dgfs-random-user-data") == 0)
        {
            cmd.dgfsRandomUserData = true;
        }
        else if (strcmp(optionName, "--dgfs-bytes-per-index") == 0)
        {
            if (i != argc)
            {
                int bpi = atoi(argv[i++]);
                if (bpi == 1 || bpi == 2 || bpi == 4)
                    cmd.dgfsBytesPerIndex = (uint8_t)bpi;
                else
                    printf("Invalid --dgfs-bytes-per-index value: %d (must be 1, 2, or 4)\n", bpi);
            }
        }
        else if (strcmp(optionName, "--dgfs-geomid-indexing") == 0)
        {
            cmd.dgfsIndexedGeomIDs = true;
        }
        else if (strcmp(optionName, "--dgfs-compressed1-vertices") == 0)
        {
            cmd.dgfsCompressed1Vertices = true;
        }
        else if (strcmp(optionName, "--dgfs-single-cluster") == 0)
        {
            if (i != argc)
            {
                int clusterNumber = atoi(argv[i++]);
                cmd.dgfsSingleClusterNumber = clusterNumber;
                cmd.dgfsSingleCluster = true;
            }
        }
        else
        {
            printf("Spurious commandline argument: %s\n", optionName);
        }
    }

    return true;
}
