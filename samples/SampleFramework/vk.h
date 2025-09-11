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
#include <Windows.h>
#include <assert.h>

#define VK_USE_PLATFORM_WIN32_KHR 1
#include "volk.h"

#include "vk_mem_alloc.h"

#define CHECK_IF_FAILED(result) { auto ret = result; assert(ret==VK_SUCCESS); }

namespace vk
{
    class Device;

    struct VMABuffer
    {
        VkBuffer buffer;
        VmaAllocation allocation;
    };

    class Buffer : public api::Buffer
    {
    public:
        VkDeviceAddress GetGPUVirtualAddress()
        {
            VkBufferDeviceAddressInfo info = { VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
            info.buffer = m_buffer;
            return vkGetBufferDeviceAddress(m_device, &info);
        }

        VkBuffer GetResource() { return m_buffer; }

        VmaAllocator GetAllocator() { return m_allocator; }
        VmaAllocation GetAllocation() { return m_allocation; }

    protected:
        friend class Device;

        Buffer(VkBuffer buffer, VkDevice device, VmaAllocator allocator, VmaAllocation allocation) : m_buffer(buffer), m_device(device), m_allocator(allocator), m_allocation(allocation) {}

        VkBuffer m_buffer;
        VkDevice m_device;
        VmaAllocator m_allocator;
        VmaAllocation m_allocation;
    };
    
    class DynamicBuffer : public vk::Buffer
    {
    public:

    private:
        friend class Device;
        DynamicBuffer(VkDevice device, VmaAllocator allocator, VMABuffer renameChain[api::FRAME_COUNT]);

        void Write(const void* contents, size_t size, size_t frameIndex);

        VMABuffer m_renameChain[api::FRAME_COUNT];
    };
	
    class Texture : public api::Texture
    {
    public:
        VkImage GetResource() { return m_image; }
        VkImageView GetImageView() { return m_imageView; }
        VkImageAspectFlags GetAspectMask() { return m_aspectMask; }
        VkExtent3D GetSize() { return m_size; }
    protected:
        friend class Device;
        friend class CommandList;

        Texture(VkImage image, VkImageView imageView, Device* device, VkImageAspectFlags aspectMask, VkExtent3D size) : m_image(image), m_imageView(imageView),  m_device(device), m_aspectMask(aspectMask), m_size(size){}

        virtual ~Texture();

        VkImage m_image;
        VkImageView m_imageView;
        VkImageAspectFlags m_aspectMask;
        VkExtent3D m_size;
        Device* m_device;
    };

    class DepthStencilTexture : public vk::Texture
    {
    public:
    private:
        friend class Device;
        friend class CommandList;

        DepthStencilTexture(VkImage image, VkImageView imageView, Device* device, VkExtent2D size)
            : vk::Texture(image, imageView, device, VK_IMAGE_ASPECT_DEPTH_BIT, { size.width, size.height, 0 }),
            m_size(size), m_imageView(imageView)
        {
        }
        VkImageView GetImageView() { return m_imageView; };

        VkExtent2D m_size;
        VkImageView m_imageView;
    };

    class RenderTargetTexture : public vk::Texture
    {
    public:
    private:
        friend class Device;
        friend class CommandList;
        VkImageView GetImageView() { return m_imageView; };

        RenderTargetTexture(VkImage image, VkImageView imageView, Device* device, VkExtent2D size)
            : vk::Texture(image, imageView, device, VK_IMAGE_ASPECT_COLOR_BIT, {size.width, size.height, 0 }),
            m_size(size), m_imageView(imageView)
        {
        }

        VkExtent2D m_size;
        VkImageView m_imageView;
    };

    class TimerHeap : public api::TimerHeap
    {
    public:
        virtual void ReadTimers(double* time, size_t start, size_t count) override;

    private:
        friend class Device;
        friend class CommandList;

        TimerHeap(VkQueryPool queryPool, VmaAllocator allocator, VmaAllocation readbackAllocation, VkBuffer readbackBuffer, double tickDuration, size_t numTimers)
            : m_queryPool(queryPool),
            m_allocator(allocator),
            m_readbackAllocation(readbackAllocation),
            m_readbackBuffer(readbackBuffer),
            m_gpuTickDuration(tickDuration),
            m_numTimers(numTimers)
        {
        }

        VkQueryPool m_queryPool;
        VmaAllocator m_allocator;
        VmaAllocation m_readbackAllocation;
        VkBuffer m_readbackBuffer;
        double m_gpuTickDuration;
        size_t m_numTimers;
    };

    class CommandList : public api::CommandList
    {
    public:
        VkCommandBuffer Get() { return m_commandBuffer; }

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
        
        virtual void InitializeBuffer(api::Buffer* buffer, const void* contents, size_t size) override;

        virtual void ResolveTimerHeap(api::TimerHeap* heap) override;
        virtual void RecordTimer(api::TimerHeap* heap, size_t timer) override;


        virtual void BeginRenderPass(api::Viewport vp, api::ScissorRect rect,
            api::Texture* backBuffer, api::Texture* zBuffer,
            uint32_t clearFlags, const float clearColor[4], float clearDepth) override;

        virtual void EndRenderPass() override;

    private:
        friend class Device;
        
        CommandList(
            VkCommandBuffer commandBuffer, 
            VkDevice device, 
            VmaAllocator vmaAllocator, 
            VkCommandPool commandPool,
            Device* frameworkDevice ) : m_commandBuffer(commandBuffer), m_device(device), m_vmaAllocator(vmaAllocator), m_commandPool(commandPool), m_frameworkDevice(frameworkDevice)
        {
        }
        ~CommandList()
        {
            for (VMABuffer buf : m_uploadBuffers)
            {
                vmaDestroyBuffer(m_vmaAllocator, buf.buffer, buf.allocation);
            }
            
            vkFreeCommandBuffers(m_device, m_commandPool, 1, &m_commandBuffer);
        }

        VkCommandBuffer m_commandBuffer;
        VkCommandPool m_commandPool;
        VkDevice m_device;
        VmaAllocator m_vmaAllocator;
        Device* m_frameworkDevice;

        VkDescriptorSetLayout m_currentSetLayout;
        VkPipelineLayout m_currentPipelineLayout;
        VkDescriptorSet m_currentDescriptorSet;
        uint32_t m_currentDescriptorCount = 0;

        std::vector<VMABuffer> m_uploadBuffers;
    };

    class ComputePipeline : public api::ComputePipeline
    {
    public:

    private:
        friend class Device;
        friend class CommandList;

        ComputePipeline(
            VkPipeline pipeline, 
            VkPipelineLayout layout, 
            VkDescriptorSet descriptor_set,
            uint32_t descriptor_count,
            VkDescriptorSetLayout set_layout ) 
            : m_pipeline(pipeline), 
              m_layout(layout), 
              m_descriptor_set(descriptor_set), 
              m_descriptor_count(descriptor_count), 
              m_descriptor_set_layout(set_layout)
        {
        }

        VkPipeline m_pipeline;
        VkPipelineLayout m_layout;
        VkDescriptorSet m_descriptor_set;
        VkDescriptorSetLayout m_descriptor_set_layout;
        uint32_t m_descriptor_count;
    };

    class RenderingPipeline : public api::RenderingPipeline
    {
    public:
    private:
        friend class Device;
        friend class CommandList;

        RenderingPipeline(
            VkPipeline pipeline, 
            VkPipelineLayout layout, 
            VkDescriptorSet descriptor_set,
            uint32_t descriptor_count,
            VkDescriptorSetLayout set_layout ) 
            : m_pipeline(pipeline), 
              m_layout(layout), 
              m_descriptor_set(descriptor_set), 
              m_descriptor_count(descriptor_count),
              m_descriptor_set_layout(set_layout)
        {
        }

        VkPipeline m_pipeline;
        VkPipelineLayout m_layout;
        VkDescriptorSet m_descriptor_set;
        VkDescriptorSetLayout m_descriptor_set_layout;
        uint32_t m_descriptor_count;
    };

    class RTAS : public api::RTAS 
    {
    public:
        virtual api::GPUVA GetGPUVirtualAddress() override
        {
            VkBufferDeviceAddressInfo info = { VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
            info.buffer = m_buffer;
            return vkGetBufferDeviceAddress(m_device, &info);
        }

        virtual api::GPUVA GetRTASReference() override
        {
            VkAccelerationStructureDeviceAddressInfoKHR info = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR };
            info.accelerationStructure = m_as;
            return vkGetAccelerationStructureDeviceAddressKHR(m_device, &info);
        }

        VkAccelerationStructureKHR GetASHandle()
        {
            return m_as;
        }

    private:
        friend class Device;

        RTAS(VkBuffer buffer, VmaAllocator allocator, VmaAllocation allocation, VkDevice device, VkAccelerationStructureKHR as) : m_buffer(buffer), m_allocator(allocator), m_allocation(allocation), m_device(device), m_as(as)
        {
        }

        ~RTAS()
        {
            vmaDestroyBuffer(m_allocator, m_buffer, m_allocation);
        }

        VkAccelerationStructureKHR m_as;
        VkBuffer m_buffer;
        VkDevice m_device;
        VmaAllocator m_allocator;
        VmaAllocation m_allocation;
    };
    class Device : public api::Device
    {
    public:

        // get the next render target in the swap chain
        virtual vk::Texture* GetCurrentBackbuffer() override {
            return m_swapchain_textures[m_backbufferIndex].Get();
        }

        // start a command list
        virtual vk::CommandList* BeginCommandList() override;

        // submit a batch of command lists
        virtual void SubmitCommandList(api::CommandList* commandList) override;

        virtual api::RTASPrebuildInfo GetRTASPrebuildInfo(const api::RTASDesc& desc) override;

        // Presents and advances to the next frame
        virtual void PresentAndAdvanceFrame() override;

        virtual void WaitForGPUIdle() override;

        virtual vk::ComputePipeline* CreateComputePipeline(
            ShaderCompiler& compiler,
            const WCHAR* path,
            std::span<const api::RootParameterType> rootSig,
            std::span<const DxcDefine> defines = {},
            const WCHAR* entrypoint = L"CS_main") override;

        virtual vk::RenderingPipeline* CreateRenderingPipeline(
            ShaderCompiler& compiler,
            const WCHAR* path,
            std::span<const api::RootParameterType> rootSig,
            std::span<const DxcDefine> defines = {}) override;

        virtual vk::Texture* CreateDepthStencilBuffer(uint32_t width, uint32_t height) override;

        virtual vk::Texture* CreateUAVTexture(uint32_t width, uint32_t height) override;

        virtual vk::RTAS* CreateRTAS(size_t size) override;

        virtual vk::Buffer* CreateUAVBuffer(size_t size) override;

        virtual vk::Buffer* CreateDynamicBuffer(size_t size) override;
        virtual vk::Buffer* CreateBuffer(size_t size) override;

        virtual void IMGUI_NewFrame() override;

        virtual void IMGUI_RenderDrawData(api::CommandList* cmdList) override;

        virtual vk::TimerHeap* CreateTimerHeap(size_t numTimers) override;
        
        virtual void WriteDynamicBuffer(api::Buffer* buffer, const void* contents, size_t size) override;

        virtual std::wstring GetAPIName() override { return L"Vulkan"; }


    private:
        friend Device* CreateDevice(HWND hwnd, HINSTANCE hInstance, uint32_t width, uint32_t height);

        Device(HWND hwnd, HINSTANCE hInstance, uint32_t width, uint32_t height);
        ~Device();

        virtual bool SupportsNativeDGF() { return m_nativeDGF; }

        struct FrameState
        {
            std::vector<vk::CommandList*> commandLists;
            VkSemaphore imageAquired;
            VkSemaphore renderComplete;
            VkFence     renderCompleteFence;
            UINT64 fenceValue;
            VkDescriptorPool descriptorPool;
        };

        friend class CommandList;
        VkDescriptorSet AllocateDispatchDescriptors(size_t count, VkDescriptorSetLayout setLayout);

        void WaitForImageReady();

        VkSwapchainKHR m_swapchain;
        std::vector<VkImage> m_swapchain_images;
        std::vector<VkImageView> m_swapchain_image_views;
        std::vector<ComPtr<Texture>> m_swapchain_textures;
        UINT m_backbufferIndex = 0;

        
        VkSemaphore m_timelineSemaphore;
        uint64_t m_timelineSemaphoreValue = 0;
        
        VkDevice m_device;
        VkCommandPool m_commandPool;
        VkDescriptorPool m_descriptorPool; 
        VkQueue m_queue;
        VmaAllocator m_vmaAllocator;

        FrameState m_frames[api::FRAME_COUNT];
        UINT m_frameIndex = 0;

        float m_timestampPeriod = 0.0;

        bool m_nativeDGF = false;
    };

    Device* CreateDevice(HWND hwnd, HINSTANCE hInstance, uint32_t width, uint32_t height);
}
