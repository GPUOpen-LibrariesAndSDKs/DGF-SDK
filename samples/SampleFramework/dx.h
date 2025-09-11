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

#include "api.h"
#include "d3dx12.h"
#include "DXSampleHelper.h"

#include <dxgi1_6.h>
#include "Util.h"

namespace dx
{
    class Device;

    class Buffer : public api::Buffer
    {
    public:
        D3D12_GPU_VIRTUAL_ADDRESS GetGPUVirtualAddress() { return m_resource->GetGPUVirtualAddress(); }
        ID3D12Resource* GetResource() { return m_resource.Get(); }
        
    protected:
        friend class Device;
        Buffer(ComPtr<ID3D12Resource> resource) : m_resource(resource) {}

        ComPtr<ID3D12Resource> m_resource;
    };

    class DynamicBuffer : public dx::Buffer
    {
    public:

    private:
        friend class Device;
        DynamicBuffer(ComPtr<ID3D12Resource> renameChain[api::FRAME_COUNT]);

        void Write(const void* contents, size_t size, size_t frameIndex);

        ComPtr<ID3D12Resource> m_renameChain[api::FRAME_COUNT];
    };

    class Texture : public api::Texture
    {
    public:
        ID3D12Resource* GetResource() { return m_resource.Get(); }
    protected:
        friend class Device;
        friend class CommandList;
        Texture(ComPtr<ID3D12Resource> resource, Device* device) : m_resource(resource), m_device(device) {}

        Texture(ComPtr<ID3D12Resource> resource, Device* device, D3D12_GPU_DESCRIPTOR_HANDLE uav) : m_device(device), m_resource(resource), m_uavDescriptor(uav)
        {
        }

        virtual ~Texture();

        ComPtr<ID3D12Resource> m_resource;
        D3D12_GPU_DESCRIPTOR_HANDLE m_uavDescriptor = { 0 };
        Device* m_device;
    };

    class DepthStencilTexture : public dx::Texture
    {
    public:
    private:
        friend class Device;
        friend class CommandList;
        DepthStencilTexture(ComPtr<ID3D12Resource> resource, ComPtr<ID3D12DescriptorHeap> heap, Device* device)
            : dx::Texture(resource,device),
              m_heap(heap)
        {
        }
        ComPtr<ID3D12DescriptorHeap> m_heap;
    };

    class RenderTargetTexture : public dx::Texture
    {
    public:
    private:
        friend class Device;
        friend class CommandList;
        RenderTargetTexture(ComPtr<ID3D12Resource> resource, ComPtr<ID3D12DescriptorHeap> heap, uint32_t offset, Device* device )
            : dx::Texture(resource,device),
              m_heap(heap),
              m_heapOffset(offset)
        {
        }
        ComPtr<ID3D12DescriptorHeap> m_heap;
        uint32_t m_heapOffset;
    };

    class TimerHeap : public api::TimerHeap
    {
    public:
        virtual void ReadTimers(double* time, size_t start, size_t count) override;

    private:
        friend class Device;
        friend class CommandList;

        TimerHeap(ComPtr<ID3D12QueryHeap> heap, ComPtr<ID3D12Resource> readback, double tickDuration, size_t numTimers)
            : m_queryHeap(std::move(heap)), 
              m_readbackBuffer(std::move(readback)), 
              m_gpuTickDuration(tickDuration), 
              m_numTimers(numTimers)
        {
        }
        
        ComPtr<ID3D12QueryHeap> m_queryHeap;
        ComPtr<ID3D12Resource> m_readbackBuffer;
        double m_gpuTickDuration;
        size_t m_numTimers;
    };

    class CommandList : public api::CommandList
    {
    public:
        ID3D12GraphicsCommandList6* Get() { return m_list.Get(); }

        virtual void SetPipelineState(api::RenderingPipeline* pso) override;
        virtual void SetGraphicsRootSRV(size_t paramIndex, api::Buffer* buffer, size_t offset ) override;
        virtual void SetGraphicsRootCBV(size_t paramIndex, api::Buffer* buffer, size_t offset) override;
        virtual void DispatchMesh(uint32_t x, uint32_t y, uint32_t z) override;

        virtual void SetPipelineState(api::ComputePipeline* pso) override;
        virtual void SetComputeRootCBV(size_t paramIndex, api::Buffer* buffer, size_t offset) override;
        virtual void SetComputeRootSRV(size_t paramIndex, api::RTAS* tlas) override;
        virtual void SetComputeRootSRV(size_t paramIndex, api::Buffer* buffer, size_t offset ) override;
        virtual void SetComputeRootUAV(size_t paramIndex, api::Buffer* buffer, size_t offset ) override;
        virtual void SetComputeRootUAV(size_t paramIndex, api::Texture* uav) override;
        virtual void Dispatch(uint32_t X, uint32_t Y, uint32_t Z) override;

        virtual void BuildRTAS(const api::RTASDesc& desc) override;

        virtual void UAVBarrier() override;

        virtual void BufferBarrier(api::Buffer* buffer, api::BufferState from, api::BufferState to) override;
        virtual void TextureBarrier(api::Texture* texture, api::TextureState from, api::TextureState to) override;
        virtual void CopyTexture(api::Texture* dst, api::Texture* src) override;

        virtual void BeginRenderPass(api::Viewport vp, api::ScissorRect rect, 
                                    api::Texture* backBuffer, api::Texture* zBuffer,
                                    uint32_t clearFlags, const float clearColor[4], float clearDepth ) override;
        virtual void EndRenderPass() override;

        virtual void InitializeBuffer(api::Buffer* buffer, const void* contents, size_t size) override;

        virtual void ResolveTimerHeap(api::TimerHeap* heap) override;
        virtual void RecordTimer(api::TimerHeap* heap, size_t timer) override;

    private:
        friend class Device;
        
        CommandList(ComPtr<ID3D12GraphicsCommandList6> list, ComPtr<ID3D12Device> device) : m_list(list), m_device(device)
        {
        }
        ~CommandList() = default;

        ComPtr<ID3D12GraphicsCommandList6> m_list;
        ComPtr<ID3D12Device> m_device;

        std::vector<ComPtr<ID3D12Resource>> m_uploadBuffers;
    };

    class ComputePipeline : public api::ComputePipeline
    {
    public:

    private:
        friend class Device;
        friend class CommandList;
        ComputePipeline(ComPtr<ID3D12RootSignature> rs, ComPtr<ID3D12PipelineState> shader)
            : m_rs(rs), m_shader(shader)
        {
        }
        ComPtr<ID3D12RootSignature> m_rs;
        ComPtr<ID3D12PipelineState> m_shader;
    };

    class RenderingPipeline : public api::RenderingPipeline
    {
    public:
    private:
        friend class Device;
        friend class CommandList;
        RenderingPipeline(ComPtr<ID3D12RootSignature> rs, ComPtr<ID3D12PipelineState> shader)
            : m_rs(rs), m_shader(shader)
        {
        }

        ComPtr<ID3D12RootSignature> m_rs;
        ComPtr<ID3D12PipelineState> m_shader;
    };

    class RTAS : public api::RTAS 
    {
    public:
        virtual api::GPUVA GetGPUVirtualAddress() override { return m_resource->GetGPUVirtualAddress(); }
        virtual api::GPUVA GetRTASReference() override { return m_resource->GetGPUVirtualAddress(); }

    private:
        friend class Device;
        RTAS(ComPtr<ID3D12Resource> buffer)
            : m_resource(buffer)
        {
        }
        ComPtr<ID3D12Resource> m_resource;
    };

    class Device : public api::Device
    {
    public:

        // get the next render target in the swap chain
        virtual dx::Texture* GetCurrentBackbuffer() override {
            return m_frames[m_frameIndex].backbuffer.Get();
        }

        // start a command list
        virtual dx::CommandList* BeginCommandList() override ;

        // submit a batch of command lists
        virtual void SubmitCommandList(api::CommandList* commandList) override;

        virtual api::RTASPrebuildInfo GetRTASPrebuildInfo(const api::RTASDesc& desc) override;

        // Presents and advances to the next frame
        virtual void PresentAndAdvanceFrame() override;

        virtual void WaitForGPUIdle() override;

        virtual dx::ComputePipeline* CreateComputePipeline(
            ShaderCompiler& compiler,
            const WCHAR* path,
            std::span<const api::RootParameterType> rootParams,
            std::span<const DxcDefine> defines = {},
            const WCHAR* entrypoint = L"CS_main") override;

        virtual dx::RenderingPipeline* CreateRenderingPipeline(
            ShaderCompiler& compiler,
            const WCHAR* path,
            std::span<const api::RootParameterType> rootParams,
            std::span<const DxcDefine> defines = {}) override;

        virtual dx::Texture* CreateDepthStencilBuffer(uint32_t width, uint32_t height) override;

        virtual dx::Texture* CreateUAVTexture(uint32_t width, uint32_t height) override;

        virtual dx::RTAS* CreateRTAS(size_t size) override;

        virtual dx::Buffer* CreateUAVBuffer(size_t size) override;
        virtual dx::Buffer* CreateDynamicBuffer(size_t size) override;
        virtual dx::Buffer* CreateBuffer(size_t size) override;

        virtual void IMGUI_NewFrame() override;

        virtual void IMGUI_RenderDrawData(api::CommandList* cmdList) override;

        virtual dx::TimerHeap* CreateTimerHeap(size_t numTimers) override;
        
        virtual void WriteDynamicBuffer(api::Buffer* buffer, const void* contents, size_t size) override;

        virtual std::wstring GetAPIName() override { return L"D3D12"; }

    private:
        friend Device* CreateDevice(HWND hwnd, HINSTANCE hInstance, uint32_t width, uint32_t height);
        friend class Texture;

        Device(HWND hwnd, uint32_t width, uint32_t height);
        ~Device();

        struct Descriptor
        {
            D3D12_GPU_DESCRIPTOR_HANDLE gpu;
            D3D12_CPU_DESCRIPTOR_HANDLE cpu;
        };
        Descriptor AllocateDescriptor();
        void DeallocateDescriptor(D3D12_GPU_DESCRIPTOR_HANDLE handle);

        struct FrameState
        {
            ComPtr<Texture> backbuffer;
            ComPtr<ID3D12CommandAllocator> allocator;
            std::vector< dx::CommandList* > commandLists;
            UINT64 fenceValue;
        };


        // Pipeline objects.
        ComPtr<IDXGISwapChain3>        m_swapChain;
        ComPtr<ID3D12Device5>          m_device;
        ComPtr<ID3D12CommandQueue>     m_commandQueue;
        ComPtr<ID3D12DescriptorHeap>   m_rtvHeap;
        ComPtr<ID3D12DescriptorHeap>   m_srvHeap;

        FrameState m_frames[api::FRAME_COUNT];

        UINT m_rtvDescriptorSize;
        UINT m_dsvDescriptorSize;
        UINT m_srvDescriptorSize;

        // Synchronization objects.
        UINT                m_frameIndex;
        UINT                m_frameCounter;
        HANDLE              m_fenceEvent;
        ComPtr<ID3D12Fence> m_fence;

        FreeList m_descriptorList;

    };


    Device* CreateDevice(HWND hwnd, HINSTANCE hInstance, uint32_t width, uint32_t height);
}