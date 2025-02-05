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
#define NOMINMAX
#include "d3dx12.h"
#include <dxcapi.h>
using Microsoft::WRL::ComPtr;

namespace dx {
    class ShaderBlob {
    public:
        ShaderBlob(ComPtr<IDxcBlob> blob) : m_blob(std::move(blob)){};
        const void* GetBufferPointer() const
        {
            return m_blob->GetBufferPointer();
        };
        size_t GetBufferSize() const
        {
            return m_blob->GetBufferSize();
        };

    private:
        ComPtr<IDxcBlob> m_blob;
    };

    class ShaderCompiler {
    public:
        ShaderCompiler() {}

        ShaderBlob Compile(const WCHAR*           file,
                           const WCHAR*           entrypoint,
                           const WCHAR*           profile,
                           std::vector<DxcDefine> defines = {});

    private:
        std::wstring               m_shaderRoot = L"./shaders/";
        ComPtr<IDxcCompiler3>      m_compiler;
        ComPtr<IDxcUtils>          m_utils;
        ComPtr<IDxcIncludeHandler> m_includes;
    };

    class RenderingPipeline {
    public:
        enum class RenderMode : int32_t { DGFSerial = 0, DGFBitScanWave = 1, DGFBitScanLane = 2, Raytracing = 3 };

        RenderingPipeline(RenderMode mode, const std::wstring shader) : m_shaderPath(shader), m_renderMode(mode) {}

        void       Init(ID3D12Device2* device, dx::ShaderCompiler* compiler, DXGI_FORMAT rendertargetFormat);
        RenderMode GetMode() const;

        ComPtr<ID3D12PipelineState> GetPipelineState();
        ComPtr<ID3D12RootSignature> GetRootSignature();

    private:
        const std::wstring m_shaderPath;
        const RenderMode   m_renderMode;

        ComPtr<ID3D12RootSignature> m_rootSignature;
        ComPtr<ID3D12PipelineState> m_pipelineState;
    };

    class ComputeShader {
    public:
        ComputeShader(const std::wstring shader, const std::wstring entrypoint = L"CS_main") : m_shaderPath(shader), m_entryPoint(entrypoint) {}

        void Init(ID3D12Device* device, dx::ShaderCompiler* compiler );
        
        ComPtr<ID3D12PipelineState> GetPipelineState() 
        { 
            return m_Shader; 
        }

        ComPtr<ID3D12RootSignature> GetRootSignature()
        {
            return m_RootSig;
        }
    private:
        std::wstring                m_shaderPath;
        std::wstring                m_entryPoint;
        ComPtr<ID3D12PipelineState> m_Shader;
        ComPtr<ID3D12RootSignature> m_RootSig;
    };
}  // namespace dx