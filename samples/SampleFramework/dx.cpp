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

#include "dx.h"

#include <d3d12.h>
#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx12.h"
#include "imgui_internal.h"

#include <sstream>
#include <d3dcompiler.h>

extern "C" {
    __declspec(dllexport) extern const UINT D3D12SDKVersion = AGILITY_SDK_VERSION;
}  // or later
extern "C" {
    __declspec(dllexport) extern const char* D3D12SDKPath = ".\\D3D12\\";
}


namespace dx
{
    constexpr size_t MAX_DESCRIPTORS = 63356;

    static D3D12_GPU_VIRTUAL_ADDRESS GetBufferAddress(api::Buffer* buffer, size_t offset)
    {
        return static_cast<dx::Buffer*>(buffer)->GetGPUVirtualAddress() + offset;
    }


    static ID3D12GraphicsCommandList6* GetCommandList(api::CommandList* cmdList)
    {
        return static_cast<dx::CommandList*>(cmdList)->Get();
    }

    static D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS TranslateRTASFlags(uint32_t Flags)
    {
        UINT d3d = 0;
        if (Flags & api::RTAS_PREFER_FAST_BUILD)
            d3d |= D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_BUILD;
        if (Flags & api::RTAS_PREFER_FAST_TRACE)
            d3d |= D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;

        return (D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAGS)d3d;
    }

    static bool FindRTASBinding(std::span<D3D12_SHADER_INPUT_BIND_DESC> bindings, D3D12_ROOT_DESCRIPTOR rd )
    {
        for (auto& it : bindings)
        {
            if (it.BindPoint == rd.ShaderRegister && it.Space == rd.RegisterSpace)
                return true;
        }
        return false;
    }

    static void ValidateRootSignature(ShaderBlob& blob, std::span<const api::RootParameterType> rp)
    {
        ComPtr<ID3DBlob> rsBlob;
        D3DGetBlobPart(blob.GetBufferPointer(), blob.GetBufferSize(), D3D_BLOB_ROOT_SIGNATURE, 0, &rsBlob);

        ComPtr<ID3D12RootSignatureDeserializer> deserializer;
        ThrowIfFailed(D3D12CreateRootSignatureDeserializer(rsBlob->GetBufferPointer(), rsBlob->GetBufferSize(), IID_PPV_ARGS(&deserializer)));
        
        ComPtr<ID3D12ShaderReflection> reflection = blob.ReflectShader();
        
        D3D12_SHADER_DESC shaderDesc;
        reflection->GetDesc(&shaderDesc);

        std::vector<D3D12_SHADER_INPUT_BIND_DESC> rtasBindings;
        for (size_t i = 0; i < shaderDesc.BoundResources; i++)
        {
            D3D12_SHADER_INPUT_BIND_DESC sibd;
            reflection->GetResourceBindingDesc((UINT)i, &sibd);
            if (sibd.Type == D3D_SIT_RTACCELERATIONSTRUCTURE)
                rtasBindings.push_back(sibd);            
        }


        const D3D12_ROOT_SIGNATURE_DESC* desc = deserializer->GetRootSignatureDesc();
        if (desc->NumParameters != rp.size())
            THROW_RUNTIME_ERROR("Root signature size mismatch");

        for (size_t i = 0; i < desc->NumParameters; i++)
        {
            switch (rp[i])
            {
            case api::RootParameterType::BUFFER_SRV:
                THROW_IF_FALSE(desc->pParameters[i].ParameterType == D3D12_ROOT_PARAMETER_TYPE_SRV);        
                break;

            case api::RootParameterType::BUFFER_UAV:
                THROW_IF_FALSE(desc->pParameters[i].ParameterType == D3D12_ROOT_PARAMETER_TYPE_UAV);
                break;

            case api::RootParameterType::CONSTANT_BUFFER:
                THROW_IF_FALSE(desc->pParameters[i].ParameterType == D3D12_ROOT_PARAMETER_TYPE_CBV);
                break;

            case api::RootParameterType::IMAGE_UAV:
                THROW_IF_FALSE(desc->pParameters[i].ParameterType == D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE);
                THROW_IF_FALSE(desc->pParameters[i].DescriptorTable.NumDescriptorRanges == 1);
                THROW_IF_FALSE(desc->pParameters[i].DescriptorTable.pDescriptorRanges[0].RangeType == D3D12_DESCRIPTOR_RANGE_TYPE_UAV )
                break;

            case api::RootParameterType::RTAS:
                THROW_IF_FALSE(desc->pParameters[i].ParameterType == D3D12_ROOT_PARAMETER_TYPE_SRV);

                // verify that the root parameter is bound to an rtas and not something else
                THROW_IF_FALSE(FindRTASBinding(rtasBindings, desc->pParameters[i].Descriptor));
                break;
            }
        }
    }

    static void GetHardwareAdapter(IDXGIFactory1* pFactory,
        IDXGIAdapter1** ppAdapter,
        bool            requestHighPerformanceAdapter)
    {
        *ppAdapter = nullptr;

        ComPtr<IDXGIAdapter1> adapter;

        ComPtr<IDXGIFactory6> factory6;
        if (SUCCEEDED(pFactory->QueryInterface(IID_PPV_ARGS(&factory6))))
        {
            for (UINT adapterIndex = 0;
                DXGI_ERROR_NOT_FOUND != factory6->EnumAdapterByGpuPreference(adapterIndex,
                    requestHighPerformanceAdapter == true
                    ? DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE
                    : DXGI_GPU_PREFERENCE_UNSPECIFIED,
                    IID_PPV_ARGS(&adapter));
                ++adapterIndex)
            {
                DXGI_ADAPTER_DESC1 desc;
                adapter->GetDesc1(&desc);

                if (desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) {
                    // Don't select the Basic Render Driver adapter.
                    // If you want a software adapter, pass in "/warp" on the command line.
                    continue;
                }

                // Check to see whether the adapter supports Direct3D 12, but don't create the
                // actual device yet.
                if (SUCCEEDED(D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_11_0, _uuidof(ID3D12Device), nullptr))) {
                    break;
                }
            }
        }
        else
        {
            for (UINT adapterIndex = 0; DXGI_ERROR_NOT_FOUND != pFactory->EnumAdapters1(adapterIndex, &adapter);
                ++adapterIndex)
            {
                DXGI_ADAPTER_DESC1 desc;
                adapter->GetDesc1(&desc);

                if (desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) {
                    // Don't select the Basic Render Driver adapter.
                    // If you want a software adapter, pass in "/warp" on the command line.
                    continue;
                }

                // Check to see whether the adapter supports Direct3D 12, but don't create the
                // actual device yet.
                if (SUCCEEDED(D3D12CreateDevice(adapter.Get(), D3D_FEATURE_LEVEL_11_0, _uuidof(ID3D12Device), nullptr))) {
                    break;
                }
            }
        }

        *ppAdapter = adapter.Detach();
    }

    DynamicBuffer::DynamicBuffer(ComPtr<ID3D12Resource> renameChain[api::FRAME_COUNT])
        : Buffer(renameChain[0])
    {
        for (size_t i = 0; i < api::FRAME_COUNT; i++)
            m_renameChain[i] = renameChain[i];        
    }

    void* DynamicBuffer::Map(size_t frameIndex)
    {
        void* pData;
        m_renameChain[frameIndex]->Map(0, nullptr, &pData);

        // redirect the base buffer pointer at the current buffer
        //   This ensures that the most recent buffer is used whenever the buffer is bound on a command list
        //    The application must unmap buffers in the same frame as the 'Map()', but commands can be recorded before unmapping is done
        m_resource = m_renameChain[frameIndex];

        return pData;
    }
    void DynamicBuffer::Unmap(size_t frameIndex, size_t bytesWritten)
    {
        D3D12_RANGE writeRange;
        writeRange.Begin = 0;
        writeRange.End = bytesWritten;
        m_renameChain[frameIndex]->Unmap(0, &writeRange);       
    }

    void DynamicBuffer::Write(const void* contents, size_t size, size_t frameIndex)
    {
        D3D12_RANGE writeRange;
        writeRange.Begin = 0;
        writeRange.End = size;

        void* pData;
        m_renameChain[frameIndex]->Map(0, nullptr, &pData);
        memcpy(pData, contents, size);
        m_renameChain[frameIndex]->Unmap(0, &writeRange);
        
        // redirect the base buffer pointer at the current buffer
        //   This ensures that the most recent buffer is used whenever the buffer is bound on a command list
        m_resource = m_renameChain[frameIndex];
    }

    void CommandList::BeginRenderPass(api::Viewport vp, api::ScissorRect rect,
        api::Texture* renderTarget, api::Texture* zbuffer,
        uint32_t clearFlags, const float clearColor[4], float clearDepth )
    {
        D3D12_VIEWPORT d3dvp;
        d3dvp.Height = vp.Height;
        d3dvp.Width = vp.Width;
        d3dvp.TopLeftX = vp.TopLeftX;
        d3dvp.TopLeftY = vp.TopLeftY;
        d3dvp.MinDepth = vp.MinDepth;
        d3dvp.MaxDepth = vp.MaxDepth;
        
        D3D12_RECT d3drect;
        d3drect.left = rect.left;
        d3drect.right = rect.right;
        d3drect.top = rect.top;
        d3drect.bottom = rect.bottom;
                
        dx::RenderTargetTexture* rt = static_cast<dx::RenderTargetTexture*>(renderTarget);
        dx::DepthStencilTexture* ds = static_cast<dx::DepthStencilTexture*>(zbuffer);

        D3D12_CPU_DESCRIPTOR_HANDLE rtv = rt->m_heap->GetCPUDescriptorHandleForHeapStart();
        rtv.ptr += rt->m_heapOffset;

        D3D12_CPU_DESCRIPTOR_HANDLE dsv = ds->m_heap->GetCPUDescriptorHandleForHeapStart();
        
        auto cmdList = Get();
        cmdList->RSSetViewports(1, &d3dvp);
        cmdList->RSSetScissorRects(1, &d3drect);
        cmdList->OMSetRenderTargets(1, &rtv, false, &dsv);

        if( clearFlags & api::CLEAR_COLOR )
            cmdList->ClearRenderTargetView(rtv, clearColor, 0, nullptr);

        if( clearFlags & api::CLEAR_DEPTH )
        {
            cmdList->ClearDepthStencilView(
                dsv,
                D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL,
                1.0f,
                0, 0, nullptr);
        }
    }

    void CommandList::EndRenderPass()
    {
        // no-op
    }

    void CommandList::SetPipelineState(api::RenderingPipeline* pso)
    {
        dx::RenderingPipeline* dxpso = static_cast<dx::RenderingPipeline*>(pso);
        Get()->SetGraphicsRootSignature(dxpso->m_rs.Get());
        Get()->SetPipelineState(dxpso->m_shader.Get());
    }

    void CommandList::SetGraphicsRootSRV(size_t paramIndex, api::Buffer* buffer, size_t offset)
    {        
        Get()->SetGraphicsRootShaderResourceView((UINT)paramIndex, GetBufferAddress(buffer,offset));
    }

    void CommandList::SetGraphicsRootCBV(size_t paramIndex, api::Buffer* buffer, size_t offset)
    {
        Get()->SetGraphicsRootConstantBufferView((UINT)paramIndex, GetBufferAddress(buffer, offset));
    }

    void CommandList::DispatchMesh(uint32_t x, uint32_t y, uint32_t z)
    {
        Get()->DispatchMesh(x, y, z);
    }

    void CommandList::SetPipelineState(api::ComputePipeline* pso)
    {
        dx::ComputePipeline* dxpso = static_cast<dx::ComputePipeline*>(pso);
        Get()->SetComputeRootSignature(dxpso->m_rs.Get());
        Get()->SetPipelineState(dxpso->m_shader.Get());
    }

    void CommandList::SetComputeRootCBV(size_t paramIndex, api::Buffer* buffer, size_t offset)
    {
        Get()->SetComputeRootConstantBufferView((UINT)paramIndex, GetBufferAddress(buffer, offset));
    }

    void CommandList::SetComputeRootSRV(size_t paramIndex, api::RTAS* tlas)
    {
        Get()->SetComputeRootShaderResourceView((UINT)paramIndex, tlas->GetRTASReference());
    }

    void CommandList::SetComputeRootSRV(size_t paramIndex, api::Buffer* buffer, size_t offset )
    {
        Get()->SetComputeRootShaderResourceView((UINT)paramIndex, GetBufferAddress(buffer, offset));
    }

    void CommandList::SetComputeRootUAV(size_t paramIndex, api::Buffer* buffer, size_t offset )
    {
        Get()->SetComputeRootUnorderedAccessView((UINT)paramIndex, GetBufferAddress(buffer, offset));
    }

    void CommandList::SetComputeRootUAV(size_t paramIndex, api::Texture* uav)
    {
        dx::Texture* dxTexture = static_cast<dx::Texture*>(uav);        
        Get()->SetComputeRootDescriptorTable((UINT)paramIndex, dxTexture->m_uavDescriptor);
    }

    void CommandList::Dispatch(uint32_t X, uint32_t Y, uint32_t Z)
    {
        Get()->Dispatch(X, Y, Z);
    }

    void CommandList::BuildRTAS(const api::RTASDesc& desc)
    {
        D3D12_RAYTRACING_GEOMETRY_DESC geo;
        D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC bd;
        bd.DestAccelerationStructureData = desc.Destination->GetRTASReference();
        bd.ScratchAccelerationStructureData = GetBufferAddress(desc.Scratch, desc.ScratchByteOffset);
        bd.SourceAccelerationStructureData = 0;
        bd.Inputs.Flags = TranslateRTASFlags(desc.Flags);
        bd.Inputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
       
        if (desc.Type == api::RTASType::BOTTOM_LEVEL)
        {
            bd.Inputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
            bd.Inputs.pGeometryDescs = &geo;
            bd.Inputs.NumDescs = 1;
           
            geo.Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;
            geo.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
            geo.Triangles.IndexBuffer = GetBufferAddress(desc.BLAS.IndexBuffer, desc.BLAS.IndexByteOffset);
            geo.Triangles.IndexCount = desc.BLAS.NumIndices;
            geo.Triangles.Transform3x4 = 0;
            geo.Triangles.VertexBuffer.StartAddress = GetBufferAddress(desc.BLAS.VertexBuffer, desc.BLAS.VertexByteOffset);
            geo.Triangles.VertexBuffer.StrideInBytes = desc.BLAS.VertexStride;
            geo.Triangles.VertexCount = desc.BLAS.NumVertices;
            geo.Triangles.IndexFormat = DXGI_FORMAT_R32_UINT;
            geo.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;
        }
        else if(desc.Type == api::RTASType::TOP_LEVEL)
        {
            bd.Inputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
            bd.Inputs.NumDescs = desc.TLAS.NumInstances;
            bd.Inputs.InstanceDescs = GetBufferAddress(desc.TLAS.InstanceBuffer, desc.TLAS.InstancesByteOffset);           
        }
        else
        {
            THROW_RUNTIME_ERROR("Not implemented");
        }

        m_list->BuildRaytracingAccelerationStructure(&bd, 0, nullptr);
    }

    void CommandList::UAVBarrier()
    {
        auto barrier = CD3DX12_RESOURCE_BARRIER::UAV(nullptr);
        m_list->ResourceBarrier(1, &barrier);
    }

    static D3D12_RESOURCE_STATES TranslateBufferState(api::BufferState state)
    {
        switch (state)
        {
        case api::BufferState::SRV:
            return D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
        case api::BufferState::UAV:
            return D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        default:
            THROW_RUNTIME_ERROR("Bad resource state!");
        }
    }

    void CommandList::BufferBarrier(api::Buffer* buffer, api::BufferState from, api::BufferState to)
    {
        dx::Buffer* dxBuffer = static_cast<dx::Buffer*>(buffer);
        auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(
            dxBuffer->GetResource(), 
            TranslateBufferState(from), 
            TranslateBufferState(to));
        m_list->ResourceBarrier(1, &barrier);
    }

    static D3D12_RESOURCE_STATES TranslateTextureState(api::TextureState state)
    {
        switch (state)
        {
        case api::TextureState::COPY_DEST:
            return D3D12_RESOURCE_STATE_COPY_DEST;
        case api::TextureState::COPY_SOURCE:
            return D3D12_RESOURCE_STATE_COPY_SOURCE;
        case api::TextureState::PRESENT:
            return D3D12_RESOURCE_STATE_PRESENT;
        case api::TextureState::RENDER:
            return D3D12_RESOURCE_STATE_RENDER_TARGET;
        case api::TextureState::UAV:
            return D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
        default:
            THROW_RUNTIME_ERROR("bad resource state!");
        }
    }

    void CommandList::TextureBarrier(api::Texture* texture, api::TextureState from, api::TextureState to)
    {
        dx::Texture* dxTexture = static_cast<dx::Texture*>(texture);
        auto barrier = CD3DX12_RESOURCE_BARRIER::Transition(dxTexture->GetResource(), TranslateTextureState(from), TranslateTextureState(to));
        m_list->ResourceBarrier(1, &barrier);
    }

    void CommandList::CopyTexture(api::Texture* dst, api::Texture* src)
    {
        ID3D12Resource* dst12 = static_cast<dx::Texture*>(dst)->GetResource();
        ID3D12Resource* src12 = static_cast<dx::Texture*>(src)->GetResource();
        m_list->CopyResource(dst12, src12);
    }

    void CommandList::InitializeBuffer(api::Buffer* buffer, const void* contents, size_t size)
    {
        ComPtr<ID3D12Resource> upload;
        auto dafaultResourceDesc = CD3DX12_RESOURCE_DESC::Buffer(size, D3D12_RESOURCE_FLAG_NONE);
        auto uploadHeap = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD);
        ThrowIfFailed(m_device->CreateCommittedResource(&uploadHeap,
            D3D12_HEAP_FLAG_NONE,
            &dafaultResourceDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&upload)));

        void* pMapped;
        ThrowIfFailed(upload->Map(0, nullptr, &pMapped));
        memcpy(pMapped, contents, size);
        upload->Unmap(0, nullptr);

        dx::Buffer* dest = static_cast<dx::Buffer*>(buffer);
        m_list->CopyResource(dest->GetResource(), upload.Get());

        // keep the upload buffer alive until this command list retires
        m_uploadBuffers.push_back(std::move(upload));
    }

    void CommandList::ResolveTimerHeap(api::TimerHeap* heap)
    {
        dx::TimerHeap* dxheap = static_cast<dx::TimerHeap*>(heap);
        m_list->ResolveQueryData(
            dxheap->m_queryHeap.Get(), 
            D3D12_QUERY_TYPE_TIMESTAMP, 
            0, (UINT)dxheap->m_numTimers, 
            dxheap->m_readbackBuffer.Get(), 0);
    }

    void CommandList::RecordTimer(api::TimerHeap* heap, size_t timerIndex)
    {
        dx::TimerHeap* dxheap = static_cast<dx::TimerHeap*>(heap);
        m_list->EndQuery(dxheap->m_queryHeap.Get(), D3D12_QUERY_TYPE_TIMESTAMP, (UINT)timerIndex);
    }

    void TimerHeap::ReadTimers(double* time, size_t start, size_t count)
    {        
        if (start >= m_numTimers || (start + count) > m_numTimers)
            THROW_RUNTIME_ERROR("Out of range query read");

        uint64_t* pTimestamps;
        m_readbackBuffer->Map(0, nullptr, (void**)&pTimestamps );
        for (size_t i = 0; i < count; i++)
            time[i] = pTimestamps[start + i] * m_gpuTickDuration;
        m_readbackBuffer->Unmap(0, nullptr);
    }

    Texture::~Texture()
    {
        m_device->DeallocateDescriptor(m_uavDescriptor);
    }

    Device::Device(HWND hwnd, uint32_t width, uint32_t height )
    {
        UINT dxgiFactoryFlags = 0;

#if defined(_DEBUG)
        // Enable the debug layer (requires the Graphics Tools "optional feature").
        // NOTE: Enabling the debug layer after device creation will invalidate the active device.
        {
            ComPtr<ID3D12Debug> debugController;
            if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debugController)))) {
                debugController->EnableDebugLayer();

                // Enable additional debug layers.
                dxgiFactoryFlags |= DXGI_CREATE_FACTORY_DEBUG;
            }
        }
#endif

        ComPtr<IDXGIFactory4> factory;
        ThrowIfFailed(CreateDXGIFactory2(dxgiFactoryFlags, IID_PPV_ARGS(&factory)));

        ComPtr<IDXGIAdapter1> hardwareAdapter;
        GetHardwareAdapter(factory.Get(), &hardwareAdapter, true);

        ThrowIfFailed(D3D12CreateDevice(hardwareAdapter.Get(), D3D_FEATURE_LEVEL_12_2, IID_PPV_ARGS(&m_device)));

        D3D12_FEATURE_DATA_SHADER_MODEL shaderModel = { D3D_SHADER_MODEL_6_7 };
        if (FAILED(m_device->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL, &shaderModel, sizeof(shaderModel))) ||
            (shaderModel.HighestShaderModel < D3D_SHADER_MODEL_6_7))
        {
            OutputDebugStringA("ERROR: Shader Model 6.5 is not supported\n");
            throw std::exception("Shader Model 6.5 is not supported");
        }

        D3D12_FEATURE_DATA_D3D12_OPTIONS7 features = {};
        if (FAILED(m_device->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS7, &features, sizeof(features))) ||
            (features.MeshShaderTier == D3D12_MESH_SHADER_TIER_NOT_SUPPORTED))
        {
            OutputDebugStringA("ERROR: Mesh Shaders aren't supported!\n");
            throw std::exception("Mesh Shaders aren't supported!");
        }

        // Describe and create the command queue.
        D3D12_COMMAND_QUEUE_DESC queueDesc = {};
        queueDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
        queueDesc.NodeMask = 0;
        queueDesc.Priority = D3D12_COMMAND_QUEUE_PRIORITY_NORMAL;
        queueDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;

        ThrowIfFailed(m_device->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&m_commandQueue)));

        // Describe and create the swap chain.
        DXGI_SWAP_CHAIN_DESC1 swapChainDesc = {};
        swapChainDesc.BufferCount = api::FRAME_COUNT;
        swapChainDesc.Width = width;
        swapChainDesc.Height = height;
        swapChainDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
        swapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
        swapChainDesc.SampleDesc.Count = 1;
        swapChainDesc.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING;

        ComPtr<IDXGISwapChain3> swapChain;
        ThrowIfFailed(factory->CreateSwapChainForHwnd(
            m_commandQueue.Get(),  // Swap chain needs the queue so that it can force a flush on it.
            hwnd,
            &swapChainDesc,
            nullptr,
            nullptr,
            (IDXGISwapChain1**)swapChain.GetAddressOf()));

        // This sample does not support fullscreen transitions.
        ThrowIfFailed(factory->MakeWindowAssociation(hwnd, DXGI_MWA_NO_ALT_ENTER));

        ThrowIfFailed(swapChain.As(&m_swapChain));
        m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();

        // Create descriptor heaps.
        {
            // Describe and create a render target view (RTV) descriptor heap.
            D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc = {};
            rtvHeapDesc.NumDescriptors = api::FRAME_COUNT;
            rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
            rtvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
            ThrowIfFailed(m_device->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&m_rtvHeap)));

            m_rtvDescriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

        
            D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
            srvHeapDesc.NumDescriptors = MAX_DESCRIPTORS;
            srvHeapDesc.Type  = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
            srvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
            ThrowIfFailed(m_device->CreateDescriptorHeap(&srvHeapDesc, IID_PPV_ARGS(&m_srvHeap)));

            m_srvDescriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
        }

        // Create frame resources.
        {
            CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(m_rtvHeap->GetCPUDescriptorHandleForHeapStart());

            // Create a RTV and a command allocator for each frame.
            for (UINT n = 0; n < api::FRAME_COUNT; n++) {
                
                ComPtr<ID3D12Resource> bbResource;
                ThrowIfFailed(m_swapChain->GetBuffer(n, IID_PPV_ARGS(&bbResource)));
                m_device->CreateRenderTargetView(bbResource.Get(), nullptr, rtvHandle);
                rtvHandle.Offset(1, m_rtvDescriptorSize);

                m_frames[n].backbuffer = new RenderTargetTexture( bbResource, m_rtvHeap, n*m_rtvDescriptorSize, this );
                ThrowIfFailed(m_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT,
                    IID_PPV_ARGS(&m_frames[n].allocator)));

                m_frames[n].fenceValue = 0;
            }
        }

        // Create synchronization objects and wait until assets have been uploaded to the GPU.
        {
            ThrowIfFailed(m_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&m_fence)));
            
            // Create an event handle to use for frame synchronization.
            m_fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
            if (m_fenceEvent == nullptr) {
                ThrowIfFailed(HRESULT_FROM_WIN32(GetLastError()));
            }
        }

        // allocate a descriptor for IMGUI
        Descriptor d = AllocateDescriptor();

        ImGui_ImplDX12_InitInfo initInfo = {};
        initInfo.Device = m_device.Get();
        initInfo.CommandQueue = m_commandQueue.Get();
        initInfo.NumFramesInFlight = api::FRAME_COUNT;
        initInfo.RTVFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
        initInfo.DSVFormat = DXGI_FORMAT_D32_FLOAT;
        initInfo.SrvDescriptorHeap = m_srvHeap.Get();
        initInfo.LegacySingleSrvCpuDescriptor = d.cpu;
        initInfo.LegacySingleSrvGpuDescriptor = d.gpu;

        ImGui_ImplDX12_Init(&initInfo);
    }

    dx::CommandList* Device::BeginCommandList()
    {
        ComPtr<ID3D12GraphicsCommandList6> d3dlist;
        m_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, m_frames[m_frameIndex].allocator.Get(), nullptr, IID_PPV_ARGS(&d3dlist));
        dx::CommandList* list = new dx::CommandList(d3dlist,m_device);
        
        ID3D12DescriptorHeap* heap = m_srvHeap.Get();
        d3dlist->SetDescriptorHeaps(1, &heap);

        m_frames[m_frameIndex].commandLists.push_back(list);
        return list;
    }

    void Device::SubmitCommandList(api::CommandList* commandList)
    {
        auto list = GetCommandList(commandList);
        ThrowIfFailed(list->Close());
        m_commandQueue->ExecuteCommandLists(1, (ID3D12CommandList**)&list);
    }

    dx::RenderingPipeline* Device::CreateRenderingPipeline(
        ShaderCompiler& compiler,
        const WCHAR* path,
        std::span<const api::RootParameterType> rootSig,
        std::span<const DxcDefine> defines )
    {

        auto meshShaderBlob  = compiler.Compile(path, L"MS_main", L"ms_6_6", false, defines);
        auto pixelShaderBlob = compiler.Compile(path, L"PS_main", L"ps_6_6", false, defines);

        ComPtr<ID3D12RootSignature> rootSignature;

        ComPtr<ID3DBlob> rsBlob;
        D3DGetBlobPart(meshShaderBlob.GetBufferPointer(), meshShaderBlob.GetBufferSize(), D3D_BLOB_ROOT_SIGNATURE, 0, &rsBlob);

        // Pull root signature from the precompiled mesh shader.
        ThrowIfFailed(m_device->CreateRootSignature(
            0, meshShaderBlob.GetBufferPointer(), meshShaderBlob.GetBufferSize(), IID_PPV_ARGS(&rootSignature)));

        ValidateRootSignature(meshShaderBlob, rootSig);


        D3DX12_MESH_SHADER_PIPELINE_STATE_DESC psoDesc = {};
        psoDesc.pRootSignature = rootSignature.Get();
        psoDesc.MS = { .pShaderBytecode = meshShaderBlob.GetBufferPointer(),
                                                          .BytecodeLength = meshShaderBlob.GetBufferSize() };
        psoDesc.PS = { .pShaderBytecode = pixelShaderBlob.GetBufferPointer(),
                                                          .BytecodeLength = pixelShaderBlob.GetBufferSize() };
        psoDesc.RasterizerState = { .FillMode = D3D12_FILL_MODE_SOLID,
                                                          .CullMode = D3D12_CULL_MODE_NONE,
                                                          .FrontCounterClockwise = FALSE,
                                                          .DepthBias = D3D12_DEFAULT_DEPTH_BIAS,
                                                          .DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP,
                                                          .SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS,
                                                          .DepthClipEnable = TRUE,
                                                          .MultisampleEnable = FALSE,
                                                          .AntialiasedLineEnable = FALSE,
                                                          .ForcedSampleCount = 0,
                                                          .ConservativeRaster = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF };
        psoDesc.BlendState.AlphaToCoverageEnable = FALSE;
        psoDesc.BlendState.IndependentBlendEnable = FALSE;
        for (UINT i = 0; i < D3D12_SIMULTANEOUS_RENDER_TARGET_COUNT; i++) {
            const D3D12_RENDER_TARGET_BLEND_DESC rtbs = { .BlendEnable = FALSE,
                                                         .LogicOpEnable = FALSE,
                                                         .SrcBlend = D3D12_BLEND_ONE,
                                                         .DestBlend = D3D12_BLEND_ONE,
                                                         .BlendOp = D3D12_BLEND_OP_ADD,
                                                         .SrcBlendAlpha = D3D12_BLEND_ONE,
                                                         .DestBlendAlpha = D3D12_BLEND_ONE,
                                                         .BlendOpAlpha = D3D12_BLEND_OP_ADD,
                                                         .LogicOp = D3D12_LOGIC_OP_NOOP,
                                                         .RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL };
            psoDesc.BlendState.RenderTarget[i] = rtbs;
        }
        psoDesc.DepthStencilState = { .DepthEnable = TRUE,
                                         .DepthWriteMask = D3D12_DEPTH_WRITE_MASK_ALL,
                                         .DepthFunc = D3D12_COMPARISON_FUNC_LESS,
                                         .StencilEnable = FALSE };
        psoDesc.SampleMask = UINT_MAX;
        psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        psoDesc.NumRenderTargets = 1;
        psoDesc.SampleDesc = { .Count = 1, .Quality = 0 };

        psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
        psoDesc.DSVFormat = DXGI_FORMAT_D32_FLOAT;

        auto psoStream = CD3DX12_PIPELINE_MESH_STATE_STREAM(psoDesc);

        D3D12_PIPELINE_STATE_STREAM_DESC streamDesc;
        streamDesc.pPipelineStateSubobjectStream = &psoStream;
        streamDesc.SizeInBytes = sizeof(psoStream);

        ComPtr<ID3D12PipelineState> pipelineState;
        ThrowIfFailed(m_device->CreatePipelineState(&streamDesc, IID_PPV_ARGS(&pipelineState)));

        return new RenderingPipeline(rootSignature, pipelineState);
    }

    dx::ComputePipeline* Device::CreateComputePipeline(
        ShaderCompiler& compiler,
        const WCHAR* path,
        std::span<const api::RootParameterType> rootSigTypes,
        std::span<const DxcDefine> defines ,
        const WCHAR* entrypoint) 
    {
        auto Blob = compiler.Compile(path,  entrypoint, L"cs_6_6", false, defines);
        auto             blob = Blob.GetBufferPointer();
        auto             blobSize = Blob.GetBufferSize();
        ComPtr<ID3DBlob> rsBlob;
        D3DGetBlobPart(blob, blobSize, D3D_BLOB_ROOT_SIGNATURE, 0, &rsBlob);

        ComPtr<ID3D12RootSignature> rootSig;
        ThrowIfFailed(m_device->CreateRootSignature(
            0, rsBlob->GetBufferPointer(), rsBlob->GetBufferSize(), IID_PPV_ARGS(&rootSig)));

        ValidateRootSignature(Blob, rootSigTypes);

        D3D12_COMPUTE_PIPELINE_STATE_DESC psoDesc;
        psoDesc.CachedPSO.CachedBlobSizeInBytes = 0;
        psoDesc.CachedPSO.pCachedBlob = nullptr;
        psoDesc.NodeMask = 0;
        psoDesc.pRootSignature = rootSig.Get();
        psoDesc.Flags = D3D12_PIPELINE_STATE_FLAG_NONE;
        psoDesc.CS.BytecodeLength = blobSize;
        psoDesc.CS.pShaderBytecode = blob;

        ComPtr<ID3D12PipelineState> shader;
        ThrowIfFailed(m_device->CreateComputePipelineState(&psoDesc, IID_PPV_ARGS(&shader)));
        return new ComputePipeline(rootSig, shader);
    }

    dx::Texture* Device::CreateDepthStencilBuffer(uint32_t width, uint32_t height)
    {
        D3D12_DEPTH_STENCIL_VIEW_DESC depthStencilDesc = {};
        depthStencilDesc.Format = DXGI_FORMAT_D32_FLOAT;
        depthStencilDesc.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
        depthStencilDesc.Flags = D3D12_DSV_FLAG_NONE;

        D3D12_CLEAR_VALUE depthOptimizedClearValue = {};
        depthOptimizedClearValue.Format = DXGI_FORMAT_D32_FLOAT;
        depthOptimizedClearValue.DepthStencil.Depth = 1.0f;
        depthOptimizedClearValue.DepthStencil.Stencil = 0;

        const CD3DX12_HEAP_PROPERTIES depthStencilHeapProps(D3D12_HEAP_TYPE_DEFAULT);
        const CD3DX12_RESOURCE_DESC   depthStencilTextureDesc = CD3DX12_RESOURCE_DESC::Tex2D(
            DXGI_FORMAT_D32_FLOAT, width, height, 1, 0, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL);

        ComPtr<ID3D12Resource> depthStencil;
        ThrowIfFailed(m_device->CreateCommittedResource(&depthStencilHeapProps,
            D3D12_HEAP_FLAG_NONE,
            &depthStencilTextureDesc,
            D3D12_RESOURCE_STATE_DEPTH_WRITE,
            &depthOptimizedClearValue,
            IID_PPV_ARGS(&depthStencil)));

        ComPtr<ID3D12DescriptorHeap> dsvHeap;
        D3D12_DESCRIPTOR_HEAP_DESC dsvHeapDesc = {};
        dsvHeapDesc.NumDescriptors = 1;
        dsvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
        dsvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
        ThrowIfFailed(m_device->CreateDescriptorHeap(&dsvHeapDesc, IID_PPV_ARGS(&dsvHeap)));
        
        m_device->CreateDepthStencilView(
            depthStencil.Get(), &depthStencilDesc, dsvHeap->GetCPUDescriptorHandleForHeapStart());

        return new DepthStencilTexture(depthStencil,dsvHeap, this);
    }

    dx::Texture* Device::CreateUAVTexture(uint32_t width, uint32_t height)
    {
        const CD3DX12_HEAP_PROPERTIES rtHeapProps(D3D12_HEAP_TYPE_DEFAULT);
        const CD3DX12_RESOURCE_DESC   rtTextureDesc = CD3DX12_RESOURCE_DESC::Tex2D(
            DXGI_FORMAT_R8G8B8A8_UNORM, width, height, 1, 1, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

        ComPtr<ID3D12Resource> texture;
        ThrowIfFailed(m_device->CreateCommittedResource(&rtHeapProps,
            D3D12_HEAP_FLAG_NONE,
            &rtTextureDesc,
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
            nullptr,
            IID_PPV_ARGS(&texture)));

        D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc;
        uavDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        uavDesc.ViewDimension = D3D12_UAV_DIMENSION_TEXTURE2D;
        uavDesc.Texture2D.MipSlice = 0;
        uavDesc.Texture2D.PlaneSlice = 0;

        Descriptor d = AllocateDescriptor();

        m_device->CreateUnorderedAccessView(texture.Get(), nullptr, &uavDesc, d.cpu);

        return new Texture(texture,this,d.gpu);
    }

    dx::RTASBuffer* Device::CreateRTASBuffer(size_t size)
    {
        auto defaultHeap = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);

        auto blasDesc = CD3DX12_RESOURCE_DESC::Buffer(size, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

        ComPtr<ID3D12Resource> buffer;
        ThrowIfFailed(m_device->CreateCommittedResource(&defaultHeap,
            D3D12_HEAP_FLAG_NONE,
            &blasDesc,
            D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE,
            nullptr,
            IID_PPV_ARGS(&buffer)));

        return new RTASBuffer(buffer);
    }

    dx::RTAS* Device::CreateRTAS(api::RTASBuffer* buffer, size_t offset, size_t size)
    {
        dx::RTASBuffer* dxBuffer = static_cast<dx::RTASBuffer*>(buffer);
        return new RTAS(dxBuffer, offset);
    }

    dx::Buffer* Device::CreateUAVBuffer(size_t size)
    {
        ComPtr<ID3D12Resource> buffer;
        auto dafaultResourceDesc = CD3DX12_RESOURCE_DESC::Buffer(size, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
        auto uploadHeap = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
        ThrowIfFailed(m_device->CreateCommittedResource(&uploadHeap,
            D3D12_HEAP_FLAG_NONE,
            &dafaultResourceDesc,
            D3D12_RESOURCE_STATE_COMMON,
            nullptr,
            IID_PPV_ARGS(&buffer)));
        return new dx::Buffer(buffer);
    }

    dx::Buffer* Device::CreateDynamicBuffer(size_t size)
    {
        auto dafaultResourceDesc = CD3DX12_RESOURCE_DESC::Buffer(size, D3D12_RESOURCE_FLAG_NONE);
        auto uploadHeap = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD);
        
        ComPtr<ID3D12Resource> renameChain[api::FRAME_COUNT];
        for (size_t i = 0; i < api::FRAME_COUNT; i++)
        {
            ThrowIfFailed(m_device->CreateCommittedResource(&uploadHeap,
                D3D12_HEAP_FLAG_NONE,
                &dafaultResourceDesc,
                D3D12_RESOURCE_STATE_GENERIC_READ,
                nullptr,
                IID_PPV_ARGS(&renameChain[i])));
        }

        return new dx::DynamicBuffer(renameChain);
    }

    dx::Buffer* Device::CreateBuffer(size_t size)
    {
        ComPtr<ID3D12Resource> buffer;
        auto dafaultResourceDesc = CD3DX12_RESOURCE_DESC::Buffer(size, D3D12_RESOURCE_FLAG_NONE);
        auto heapDesc = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);
        ThrowIfFailed(m_device->CreateCommittedResource(&heapDesc,
            D3D12_HEAP_FLAG_NONE,
            &dafaultResourceDesc,
            D3D12_RESOURCE_STATE_COMMON,
            nullptr,
            IID_PPV_ARGS(&buffer)));
        return new dx::Buffer(buffer);
    }

    api::RTASPrebuildInfo Device::GetRTASPrebuildInfo(const api::RTASDesc& desc) 
    {
        D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS brtasi;
        brtasi.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
        brtasi.Flags = TranslateRTASFlags(desc.Flags);

        D3D12_RAYTRACING_GEOMETRY_DESC geom;
        if (desc.Type == api::RTASType::BOTTOM_LEVEL)
        {
            brtasi.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;
            brtasi.pGeometryDescs = &geom;
            brtasi.NumDescs = 1;
            geom.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
            geom.Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;
            geom.Triangles.IndexCount = desc.BLAS.NumIndices;
            geom.Triangles.IndexFormat = DXGI_FORMAT_R32_UINT;
            geom.Triangles.Transform3x4 = 0;
            geom.Triangles.VertexCount = desc.BLAS.NumVertices;
            geom.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT;
            geom.Triangles.VertexBuffer.StrideInBytes = desc.BLAS.VertexStride;
            geom.Triangles.VertexBuffer.StartAddress = 0;
            geom.Triangles.IndexBuffer = 0;
        }
        else if( desc.Type == api::RTASType::TOP_LEVEL )
        {
            brtasi.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
            brtasi.NumDescs = desc.TLAS.NumInstances;
            brtasi.InstanceDescs = 0;
        }
        else
        {
            THROW_RUNTIME_ERROR("Not implemented");
        }

        D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO pbi;
        m_device->GetRaytracingAccelerationStructurePrebuildInfo(&brtasi, &pbi);
        
        api::RTASPrebuildInfo apipbi;
        apipbi.buildScratchSize = pbi.ScratchDataSizeInBytes;
        apipbi.rtasSize = pbi.ResultDataMaxSizeInBytes;
        apipbi.updateScratchSize = pbi.UpdateScratchDataSizeInBytes;
        return apipbi;
    }


    void Device::PresentAndAdvanceFrame()
    {
        // Present the frame.
        ThrowIfFailed(m_swapChain->Present(0, DXGI_PRESENT_ALLOW_TEARING));

        // Schedule a Signal command in the queue.
        const UINT64 currentFenceValue = m_frames[m_frameIndex].fenceValue;
        ThrowIfFailed(m_commandQueue->Signal(m_fence.Get(), currentFenceValue));

        // Update the frame index.
        m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();

        // If the next frame is not ready to be rendered yet, wait until it is ready.
        if (m_fence->GetCompletedValue() < m_frames[m_frameIndex].fenceValue) {
            ThrowIfFailed(m_fence->SetEventOnCompletion(m_frames[m_frameIndex].fenceValue, m_fenceEvent));
            WaitForSingleObject(m_fenceEvent, INFINITE);
        }

        // reset the command allocator and reap command lists
        ThrowIfFailed(m_frames[m_frameIndex].allocator->Reset());
        for (api::CommandList* list : m_frames[m_frameIndex].commandLists)
            delete list;
        m_frames[m_frameIndex].commandLists.clear();
        
        for (auto& callback : m_frames[m_frameIndex].syncCallbacks)
            callback();
        m_frames[m_frameIndex].syncCallbacks.clear();

        // Set the fence value for the next frame.
        m_frames[m_frameIndex].fenceValue = currentFenceValue + 1;
    }

    void Device::WaitForGPUIdle()
    {
        // Schedule a Signal command in the queue.
        ThrowIfFailed(m_commandQueue->Signal(m_fence.Get(), m_frames[m_frameIndex].fenceValue));

        // Wait until the fence has been processed.
        ThrowIfFailed(m_fence->SetEventOnCompletion(m_frames[m_frameIndex].fenceValue, m_fenceEvent));
        WaitForSingleObject(m_fenceEvent, INFINITE);

        // Increment the fence value for the current frame.
        m_frames[m_frameIndex].fenceValue++;

        // all frames have now retired.. reap all command lists
        for (size_t i = 0; i < api::FRAME_COUNT; i++)
        {
            for (dx::CommandList* cl : m_frames[i].commandLists)
                delete cl;
            m_frames[i].commandLists.clear();

            for (auto& callback : m_frames[i].syncCallbacks)
                callback();
            m_frames[i].syncCallbacks.clear();
        }
    }

    Device::~Device()
    {
        ImGui_ImplDX12_Shutdown();
    }

    void Device::IMGUI_NewFrame()
    {
        ImGui_ImplDX12_NewFrame();
    }

    void Device::IMGUI_RenderDrawData(api::CommandList* cmdList)
    {
        ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), GetCommandList(cmdList));
    }

    Device::Descriptor Device::AllocateDescriptor()
    {
        uint32_t descriptorIndex = m_descriptorList.Alloc();
        if (descriptorIndex >= MAX_DESCRIPTORS)
            throw std::runtime_error("Out of descriptors");

        Descriptor d;
        d.cpu = m_srvHeap->GetCPUDescriptorHandleForHeapStart();
        d.gpu = m_srvHeap->GetGPUDescriptorHandleForHeapStart();
        d.cpu.ptr += descriptorIndex * m_srvDescriptorSize;
        d.gpu.ptr += descriptorIndex * m_srvDescriptorSize;        
        return d;
    }

    void Device::DeallocateDescriptor(D3D12_GPU_DESCRIPTOR_HANDLE handle)
    {
        D3D12_GPU_DESCRIPTOR_HANDLE heapBase = m_srvHeap->GetGPUDescriptorHandleForHeapStart();
        if (handle.ptr >= heapBase.ptr)
        {
            uint32_t idx = (uint32_t)((handle.ptr - heapBase.ptr) / m_srvDescriptorSize);
            m_descriptorList.Free(idx);
        }
    }


    dx::TimerHeap* Device::CreateTimerHeap( size_t numTimers )
    {
        const D3D12_QUERY_HEAP_DESC queryHeapDesc = {
        D3D12_QUERY_HEAP_TYPE_TIMESTAMP, static_cast<UINT>(numTimers), 0 };

        ComPtr<ID3D12QueryHeap> queryHeap;
        ThrowIfFailed(m_device->CreateQueryHeap(&queryHeapDesc, IID_PPV_ARGS(&queryHeap)));

        auto resourceDesc = CD3DX12_RESOURCE_DESC::Buffer(numTimers * sizeof(uint64_t), D3D12_RESOURCE_FLAG_NONE);
        auto defaultHeap = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_READBACK);

        ComPtr<ID3D12Resource> readback;
        ThrowIfFailed(m_device->CreateCommittedResource(&defaultHeap,
            D3D12_HEAP_FLAG_NONE,
            &resourceDesc,
            D3D12_RESOURCE_STATE_COPY_DEST,
            nullptr,
            IID_PPV_ARGS(&readback)));

        std::uint64_t timestampFrequency;
        ThrowIfFailed(m_commandQueue->GetTimestampFrequency(&timestampFrequency));
        double gpuTickDuration = 1.0 / timestampFrequency;

        return new dx::TimerHeap(queryHeap, readback, gpuTickDuration, numTimers);
    }

    void Device::WriteDynamicBuffer(api::Buffer* buffer, const void* contents, size_t size )
    {
        dx::DynamicBuffer* dxBuffer = static_cast<dx::DynamicBuffer*>(buffer);
        dxBuffer->Write(contents, size, m_frameIndex);
    }
    
    void* Device::MapDynamicBuffer( api::Buffer* buffer)
    {
        dx::DynamicBuffer* dxBuffer = static_cast<dx::DynamicBuffer*>(buffer);
        return dxBuffer->Map(m_frameIndex);
    }
    
    void Device::UnmapDynamicBuffer(api::Buffer* buffer, size_t bytesWritten)
    {
        dx::DynamicBuffer* dxBuffer = static_cast<dx::DynamicBuffer*>(buffer);
        dxBuffer->Unmap(m_frameIndex, bytesWritten);
    }

    void Device::ScheduleFrameSyncCallback(api::FrameSyncCallback cb)
    {
        m_frames[m_frameIndex].syncCallbacks.push_back(std::move(cb));
    }

    dx::Buffer* Device::CreateUploadBuffer(size_t size)
    {
        ComPtr<ID3D12Resource> upload;
        auto dafaultResourceDesc = CD3DX12_RESOURCE_DESC::Buffer(size, D3D12_RESOURCE_FLAG_NONE);
        auto uploadHeap = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD);
        ThrowIfFailed(m_device->CreateCommittedResource(&uploadHeap,
            D3D12_HEAP_FLAG_NONE,
            &dafaultResourceDesc,
            D3D12_RESOURCE_STATE_GENERIC_READ,
            nullptr,
            IID_PPV_ARGS(&upload)));

        return new dx::Buffer(std::move(upload));
    }

    void* Device::MapBuffer(api::Buffer* buffer, size_t readSize)
    {
        dx::Buffer* dxBuffer = static_cast<dx::Buffer*>(buffer);
        return dxBuffer->Map(readSize);
    }

    void Device::UnmapBuffer(api::Buffer* buffer, size_t writeSize)
    {
        dx::Buffer* dxBuffer = static_cast<dx::Buffer*>(buffer);
        dxBuffer->Unmap(writeSize);
    }


    Device* CreateDevice(HWND hwnd, HINSTANCE hi, uint32_t width, uint32_t height) 
    {
        return new Device(hwnd,width,height);        
    }

}
