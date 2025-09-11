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

#include "ShaderCompiler.h"
#include <stdexcept>
#include <sstream>

#include "Util.h"

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



#define DXIL_FOURCC(ch0, ch1, ch2, ch3)                                        \
  ((uint32_t)(uint8_t)(ch0) | (uint32_t)(uint8_t)(ch1) << 8 |                  \
   (uint32_t)(uint8_t)(ch2) << 16 | (uint32_t)(uint8_t)(ch3) << 24)

constexpr UINT32 DFCC_DXIL = DXIL_FOURCC('D', 'X', 'I', 'L');

ShaderCompiler::ShaderCompiler()
{
    m_shaderRoot = GetExecutablePath() + L"\\shaders\\";
}

ShaderBlob ShaderCompiler::Compile(const WCHAR* file,
    const WCHAR* entrypoint,
    const WCHAR* profile,
    bool SPIRV,
    std::span<const DxcDefine> defines)
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
    sourceBuffer.Ptr = blob->GetBufferPointer();
    sourceBuffer.Size = blob->GetBufferSize();
    sourceBuffer.Encoding = 0;

    std::vector<LPCWSTR> arguments;

    // Treat warnings as errors
    arguments.push_back(L"-WX");
    arguments.push_back(L"-Wno-ignored-attributes");

    // Make sure that optimization is on.
    arguments.push_back(L"-O3");

    if (SPIRV)
    {
        arguments.push_back(L"-spirv");
        arguments.push_back(L"-fspv-target-env=vulkan1.3");
        arguments.push_back(L"-fspv-extension=SPV_EXT_mesh_shader");
        arguments.push_back(L"-fspv-extension=SPV_KHR_ray_query");
        arguments.push_back(L"-fvk-use-scalar-layout");
    }

    // set the include path
    arguments.push_back(L"-I");
    arguments.push_back(m_shaderRoot.c_str());

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

ComPtr<ID3D12ShaderReflection> ShaderBlob::ReflectShader()
{
    ComPtr<IDxcContainerReflection> pReflection;
    UINT32 shaderIdx;
    THROW_ON_FAILURE(DxcCreateInstance(CLSID_DxcContainerReflection, IID_PPV_ARGS(&pReflection)));

    THROW_ON_FAILURE(pReflection->Load(m_blob.Get()));

    THROW_ON_FAILURE(
        pReflection->FindFirstPartKind(DFCC_DXIL, &shaderIdx));

    ComPtr<ID3D12ShaderReflection> reflection;
    THROW_ON_FAILURE(pReflection->GetPartReflection(
        shaderIdx, IID_PPV_ARGS(&reflection)));

    return reflection;
}