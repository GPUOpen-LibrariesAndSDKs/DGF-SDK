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

#include "DXFramework.h"
#include "Util.h"

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

    void Init(ID3D12Device5* device, dx::ShaderCompiler* compiler);

    PrebuildInfo GetPrebuildInfo(const DGFMesh& mesh) const;

    void Build( 
        ID3D12Resource* decodeScratch,
        size_t decodeOffset,
        D3D12_GPU_VIRTUAL_ADDRESS buildScratch,
        D3D12_GPU_VIRTUAL_ADDRESS destination,
        const DGFMesh& mesh,
        ID3D12GraphicsCommandList4* cmdList
        );

private:

    dx::ComputeShader m_decodeShader;
    ComPtr<ID3D12Device5> m_device;
};


#endif