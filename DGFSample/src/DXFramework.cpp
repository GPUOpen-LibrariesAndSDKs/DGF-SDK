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


#include "DXFramework.h"
#include <sstream>
#include <d3dcompiler.h>

#define THROW_ON_FAILURE(x)                                     \
    {                                                           \
        HRESULT hr = x;                                         \
        if (!SUCCEEDED(hr)) {                                   \
            const char*       call = #x;                        \
            std::stringstream stream;                           \
            stream << "Failure: " << call << " HRESULT=" << hr; \
            throw std::runtime_error(stream.str());             \
        }                                                       \
    }

#define THROW_RUNTIME_ERROR(x)       \
    {                                \
        throw std::runtime_error(x); \
    }
namespace {
    const static auto                   SIXTY_FOUR      = std::to_wstring(64);
    const static std::vector<DxcDefine> g_shaderDefines = {DxcDefine({L"MS_NUM_THREADS", SIXTY_FOUR.c_str()}),
                                                           DxcDefine({L"MS_MAX_TRIS", SIXTY_FOUR.c_str()}),
                                                           DxcDefine({L"MS_MAX_VERTS", SIXTY_FOUR.c_str()})};

}  // namespace

namespace dx {
    ShaderBlob ShaderCompiler::Compile(const WCHAR*           file,
                                       const WCHAR*           entrypoint,
                                       const WCHAR*           profile,
                                       std::vector<DxcDefine> defines)
    {
        if (m_compiler == nullptr) {
            THROW_ON_FAILURE(DxcCreateInstance(CLSID_DxcCompiler, IID_PPV_ARGS(&m_compiler)));
            THROW_ON_FAILURE(DxcCreateInstance(CLSID_DxcUtils, IID_PPV_ARGS(&m_utils)));
            THROW_ON_FAILURE(m_utils->CreateDefaultIncludeHandler(&m_includes));
        }

        std::wstring fullPath = m_shaderRoot + file;

        ComPtr<IDxcBlobEncoding> blob;
        THROW_ON_FAILURE(m_utils->LoadFile(fullPath.c_str(), nullptr, blob.GetAddressOf()));

        DxcBuffer sourceBuffer;
        sourceBuffer.Ptr      = blob->GetBufferPointer();
        sourceBuffer.Size     = blob->GetBufferSize();
        sourceBuffer.Encoding = 0;

        std::vector<LPCWSTR> arguments;
        arguments.push_back(L"-I");
        arguments.push_back(m_shaderRoot.c_str());

        // Treat warnings as errors
        arguments.push_back(L"-WX");

        // Make sure that optimization is on.
        arguments.push_back(L"-O3");

        ComPtr<IDxcUtils> dxcutils;
        THROW_ON_FAILURE(DxcCreateInstance(CLSID_DxcUtils, IID_PPV_ARGS(&dxcutils)));
        ComPtr<IDxcCompilerArgs> compilerArguments;
        dxcutils->BuildArguments(file,
                                 entrypoint,
                                 profile,
                                 arguments.data(),
                                 (UINT32)arguments.size(),
                                 defines.data(),
                                 (UINT32)defines.size(),
                                 &compilerArguments);
        ComPtr<IDxcResult> pResult;
        THROW_ON_FAILURE(m_compiler->Compile(&sourceBuffer,
                                             compilerArguments->GetArguments(),
                                             compilerArguments->GetCount(),
                                             m_includes.Get(),
                                             IID_PPV_ARGS(&pResult)));

        HRESULT status;
        THROW_ON_FAILURE(pResult->GetStatus(&status));
        if (FAILED(status)) {
            ComPtr<IDxcBlobEncoding> errorBlob;
            ComPtr<IDxcBlobUtf8>     errorBlobUtf8;
            THROW_ON_FAILURE(pResult->GetErrorBuffer(&errorBlob));
            THROW_ON_FAILURE(m_utils->GetBlobAsUtf8(errorBlob.Get(), &errorBlobUtf8));
            char str[10240];
            sprintf_s(str,
                      sizeof(str),
                      "*****HLSL COMPILE ERRORS*****\nError while compiling %S\n%s\n",
                      fullPath.c_str(),
                      errorBlobUtf8->GetStringPointer());

            OutputDebugStringA(str);
            THROW_RUNTIME_ERROR("Shader compilation errors");
        }

        ComPtr<IDxcBlob> resultBlob;
        THROW_ON_FAILURE(pResult->GetResult(&resultBlob));
        return ShaderBlob(resultBlob);
    }

    void RenderingPipeline::Init(ID3D12Device2*      device,
                                 dx::ShaderCompiler* compiler,
                                 const DXGI_FORMAT   rendertargetFormat)
    {
        auto meshShaderBlob  = compiler->Compile(m_shaderPath.c_str(), L"MS_main", L"ms_6_6", g_shaderDefines);
        auto pixelShaderBlob = compiler->Compile(m_shaderPath.c_str(), L"PS_main", L"ps_6_6", g_shaderDefines);

        // Pull root signature from the precompiled mesh shader.
        THROW_ON_FAILURE(device->CreateRootSignature(
            0, meshShaderBlob.GetBufferPointer(), meshShaderBlob.GetBufferSize(), IID_PPV_ARGS(&m_rootSignature)));

        D3DX12_MESH_SHADER_PIPELINE_STATE_DESC psoDesc = {};
        psoDesc.pRootSignature                         = m_rootSignature.Get();
        psoDesc.MS                                     = {.pShaderBytecode = meshShaderBlob.GetBufferPointer(),
                                                          .BytecodeLength  = meshShaderBlob.GetBufferSize()};
        psoDesc.PS                                     = {.pShaderBytecode = pixelShaderBlob.GetBufferPointer(),
                                                          .BytecodeLength  = pixelShaderBlob.GetBufferSize()};
        psoDesc.RasterizerState                        = {.FillMode              = D3D12_FILL_MODE_SOLID,
                                                          .CullMode              = D3D12_CULL_MODE_NONE,
                                                          .FrontCounterClockwise = FALSE,
                                                          .DepthBias             = D3D12_DEFAULT_DEPTH_BIAS,
                                                          .DepthBiasClamp        = D3D12_DEFAULT_DEPTH_BIAS_CLAMP,
                                                          .SlopeScaledDepthBias  = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS,
                                                          .DepthClipEnable       = TRUE,
                                                          .MultisampleEnable     = FALSE,
                                                          .AntialiasedLineEnable = FALSE,
                                                          .ForcedSampleCount     = 0,
                                                          .ConservativeRaster    = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF};
        psoDesc.BlendState.AlphaToCoverageEnable       = FALSE;
        psoDesc.BlendState.IndependentBlendEnable      = FALSE;
        for (UINT i = 0; i < D3D12_SIMULTANEOUS_RENDER_TARGET_COUNT; i++) {
            const D3D12_RENDER_TARGET_BLEND_DESC rtbs = {.BlendEnable           = FALSE,
                                                         .LogicOpEnable         = FALSE,
                                                         .SrcBlend              = D3D12_BLEND_ONE,
                                                         .DestBlend             = D3D12_BLEND_ONE,
                                                         .BlendOp               = D3D12_BLEND_OP_ADD,
                                                         .SrcBlendAlpha         = D3D12_BLEND_ONE,
                                                         .DestBlendAlpha        = D3D12_BLEND_ONE,
                                                         .BlendOpAlpha          = D3D12_BLEND_OP_ADD,
                                                         .LogicOp               = D3D12_LOGIC_OP_NOOP,
                                                         .RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL};
            psoDesc.BlendState.RenderTarget[i]        = rtbs;
        }
        psoDesc.DepthStencilState     = {.DepthEnable    = TRUE,
                                         .DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL,
                                         .DepthFunc      = D3D12_COMPARISON_FUNC_LESS,
                                         .StencilEnable  = FALSE};
        psoDesc.SampleMask            = UINT_MAX;
        psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        psoDesc.NumRenderTargets      = 1;
        psoDesc.SampleDesc            = {.Count = 1, .Quality = 0};

        psoDesc.RTVFormats[0] = rendertargetFormat;
        psoDesc.DSVFormat     = DXGI_FORMAT_D32_FLOAT;

        auto psoStream = CD3DX12_PIPELINE_MESH_STATE_STREAM(psoDesc);

        D3D12_PIPELINE_STATE_STREAM_DESC streamDesc;
        streamDesc.pPipelineStateSubobjectStream = &psoStream;
        streamDesc.SizeInBytes                   = sizeof(psoStream);

        THROW_ON_FAILURE(device->CreatePipelineState(&streamDesc, IID_PPV_ARGS(&m_pipelineState)));
    }

    RenderingPipeline::RenderMode RenderingPipeline::GetMode() const
    {
        return m_renderMode;
    }

    ComPtr<ID3D12PipelineState> RenderingPipeline::GetPipelineState()
    {
        return m_pipelineState;
    }

    ComPtr<ID3D12RootSignature> RenderingPipeline::GetRootSignature()
    {
        return m_rootSignature;
    }


    void ComputeShader::Init(ID3D12Device* device, dx::ShaderCompiler* compiler)
    {
        auto Blob       = compiler->Compile(m_shaderPath.c_str(), m_entryPoint.c_str(), L"cs_6_6", g_shaderDefines);
        auto             blob     = Blob.GetBufferPointer();
        auto             blobSize = Blob.GetBufferSize();
        ComPtr<ID3DBlob> rsBlob;
        D3DGetBlobPart(blob, blobSize, D3D_BLOB_ROOT_SIGNATURE, 0, &rsBlob);

        THROW_ON_FAILURE(device->CreateRootSignature(
            0, rsBlob->GetBufferPointer(), rsBlob->GetBufferSize(), IID_PPV_ARGS(&m_RootSig)));

        D3D12_COMPUTE_PIPELINE_STATE_DESC psoDesc;
        psoDesc.CachedPSO.CachedBlobSizeInBytes = 0;
        psoDesc.CachedPSO.pCachedBlob           = nullptr;
        psoDesc.NodeMask                        = 0;
        psoDesc.pRootSignature                  = m_RootSig.Get();
        psoDesc.Flags                           = D3D12_PIPELINE_STATE_FLAG_NONE;
        psoDesc.CS.BytecodeLength               = blobSize;
        psoDesc.CS.pShaderBytecode              = blob;

        THROW_ON_FAILURE(device->CreateComputePipelineState(&psoDesc, IID_PPV_ARGS(&m_Shader)));
    }

}  // namespace dx