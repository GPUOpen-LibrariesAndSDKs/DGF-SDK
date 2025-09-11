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

#ifndef _DGF_BVH_BUILDER_H_
#define _DGF_BVH_BUILDER_H_
#pragma once

#include "Util.h"
#include "api.h"

class DGFBVHBuilder 
{
public:
    
    struct PrebuildInfo
    {
        UINT bvhSize;
        UINT bvhBuildScratchSize;
        UINT decodeScratchSize;
    };

    DGFBVHBuilder();

    void Init(api::Device* device, ShaderCompiler& compiler, uint32_t flags);

    PrebuildInfo GetPrebuildInfo(const DGFMesh& mesh, bool allowNativeDGF) const;

    void Build( 
        api::Buffer* decodeScratch,
        size_t decodeOffset,
        api::Buffer* buildScratch,
        api::RTAS* destination,
        const DGFMesh& mesh,
        api::CommandList* cmdList,
        bool allowNativeDGF,
        api::TimerHeap* timers = nullptr,
        size_t timerOffset = 0
        );

private:

    ComPtr<api::ComputePipeline> m_decodeShader;
    ComPtr<api::Device> m_device;
    uint32_t m_flags;
};


#endif