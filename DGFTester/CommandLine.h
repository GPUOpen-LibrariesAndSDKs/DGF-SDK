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

#pragma once
// clang-format off


#include <DGFBaker.h>

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
    bool testPreclusterPath = false;
    bool testDGFS = false;

    bool randomOMMIDs = false;
    bool generateOMMIDs = false;
    int numRandomOMMs = 1;

    bool dgfsRandomPrimID = false;
    bool dgfsRandomUserData = false;
    uint8_t dgfsBytesPerIndex = 1;
    bool dgfsIndexedGeomIDs = false;
    bool dgfsCompressed1Vertices = false;

    bool dgfsSingleCluster = false;
    uint32_t dgfsSingleClusterNumber = 0;
};

bool ParseCommandLine(CommandLine& cmd, int argc, char** argv);
