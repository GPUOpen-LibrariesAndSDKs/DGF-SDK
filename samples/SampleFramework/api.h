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

#include <functional>
#include <cstdint>
#include <span>
#include "ShaderCompiler.h"

namespace api
{
    constexpr size_t FRAME_COUNT = 3;
    constexpr size_t MAX_THREAD_GROUP_DIM = 65535;
    constexpr size_t CONSTANT_BUFFER_ALIGNMENT = 256;
    constexpr size_t MAX_DISPATCHES_PER_FRAME = 128;
    constexpr size_t MAX_DESCRIPTORS_PER_DISPATCH = 32;
    constexpr size_t MAX_PIPELINES = 256;
    constexpr size_t RTAS_ALIGNMENT = 256;


    typedef uint64_t GPUVA;

    class RefCount
    {
    public:

        uint32_t AddRef() 
        { 
            return ++m_refCount; 
        }

        uint32_t Release() 
        {
            uint32_t ret = --m_refCount;
            if (!ret)
                delete this;
            return ret;
        }

    protected:
        virtual ~RefCount() {};
    private:
        uint32_t m_refCount = 0;
    };

    struct Viewport
    {
        Viewport (float topLeftX,
            float topLeftY,
            float width,
            float height,
            float minDepth = 0.0f,
            float maxDepth = 1.0f) noexcept
        {
            TopLeftX = topLeftX;
            TopLeftY = topLeftY;
            Width = width;
            Height = height;
            MinDepth = minDepth;
            MaxDepth = maxDepth;
        }
        float TopLeftX;
        float TopLeftY;
        float Width;
        float Height;
        float MinDepth;
        float MaxDepth;
    };

    struct ScissorRect
    {
        ScissorRect(uint32_t Left, uint32_t Top, uint32_t Right, uint32_t Bottom) noexcept
        {
            left = Left;
            top = Top;
            right = Right;
            bottom = Bottom;
        }

        uint32_t left;
        uint32_t top;
        uint32_t right;
        uint32_t bottom;
    };

    struct InstanceDesc
    {
        float Transform[3][4];
        UINT InstanceID : 24;
        UINT InstanceMask : 8;
        UINT InstanceContributionToHitGroupIndex : 24;
        UINT Flags : 8;
        GPUVA AccelerationStructure;
    };

    class Texture : public RefCount
    {
    public:

    };

    class Buffer : public RefCount 
    {
    public:
    };

    class DynamicBuffer : public Buffer
    {
    public:
    };


    struct RTASPrebuildInfo
    {
        size_t rtasSize;
        size_t buildScratchSize;
        size_t updateScratchSize;
    };

    enum class RTASType
    {
        TOP_LEVEL,
        BOTTOM_LEVEL,
        DGF_BOTTOM_LEVEL
    };

    class RTASBuffer : public RefCount
    {
    public:

    };

    class RTAS : public RefCount
    {
    public:
        virtual GPUVA GetRTASReference() = 0;
    };


    enum RTASFlags : uint32_t
    {
        RTAS_PREFER_FAST_TRACE = 1<<0,
        RTAS_PREFER_FAST_BUILD = 1<<1
    };

    struct BLASDesc
    {
        // Vertex format is implicitly R32G32B32_FLOAT
        // Index format is implicity R32_UINT
        api::Buffer* VertexBuffer;
        api::Buffer* IndexBuffer;
        uint32_t VertexByteOffset;
        uint32_t IndexByteOffset;
        uint32_t NumVertices;
        uint32_t NumIndices;
        uint32_t VertexStride;
    };

    struct DGFBLASDesc
    {
        api::Buffer* blocks;
        size_t blocksByteOffset;
        uint32_t numBlocks;
        uint32_t numTriangles;
        uint32_t numVertices;
        uint32_t maxPrimitiveIndex;
        uint32_t maxGeometryIndex;
    };

    struct TLASDesc
    {
        api::Buffer* InstanceBuffer;
        uint32_t InstancesByteOffset;
        uint32_t NumInstances;
    };

    struct RTASDesc
    {
        RTASType Type;
        api::RTAS* Destination;
        api::Buffer* Scratch;
        uint32_t ScratchByteOffset;
        uint32_t Flags;
        union
        {
            BLASDesc    BLAS;
            DGFBLASDesc DGFBLAS;
            TLASDesc    TLAS;
        };
    };

   
    class ComputePipeline : public RefCount
    {
    public:

    };

    class RenderingPipeline : public RefCount
    {
    public:

    };


    enum class TextureState
    {
        UAV,
        RENDER,
        PRESENT,
        COPY_DEST,
        COPY_SOURCE,
    };

    enum class BufferState
    {
        UAV,
        SRV
    };

    class TimerHeap : public RefCount
    {
    public:

        virtual void ReadTimers(double* time, size_t first, size_t count) = 0;
    };

    enum ClearFlags
    {
        CLEAR_DEPTH = 1<<0,
        CLEAR_COLOR = 1<<1,
        CLEAR_ALL = CLEAR_DEPTH | CLEAR_COLOR
    };

    enum class RootParameterType
    {
        CONSTANT_BUFFER,
        BUFFER_SRV,
        BUFFER_UAV,
        IMAGE_UAV,
        RTAS
    };

    // NOTE: command list objects are not reference-counted.  They are implicitly owned by 
    //  the device, and are cleaned up after execution
    class CommandList
    {
    public:
        virtual ~CommandList() {};

        virtual void SetPipelineState(RenderingPipeline* pso) = 0;
        virtual void SetGraphicsRootSRV(size_t paramIndex, Buffer* buffer, size_t offset) = 0;
        virtual void SetGraphicsRootCBV(size_t paramIndex, Buffer* buffer, size_t offset) = 0;
        virtual void DispatchMesh(uint32_t x, uint32_t y, uint32_t z) = 0;

        virtual void SetPipelineState(ComputePipeline* pso) = 0;
        virtual void SetComputeRootCBV(size_t paramIndex, Buffer* buffer, size_t offset ) = 0;
        virtual void SetComputeRootSRV(size_t paramIndex, RTAS* tlas) = 0;
        virtual void SetComputeRootSRV(size_t paramIndex, Buffer* buffer, size_t offset ) = 0;
        virtual void SetComputeRootUAV(size_t paramIndex, Buffer* buffer, size_t offset ) = 0;
        virtual void SetComputeRootUAV(size_t paramIndex, Texture* uav ) = 0;
        virtual void Dispatch(uint32_t X, uint32_t Y, uint32_t Z) = 0;
        
        virtual void BuildRTAS(const RTASDesc& desc) = 0;

        virtual void UAVBarrier() = 0;

        virtual void BufferBarrier(Buffer* buffer, BufferState from, BufferState to) = 0;

        virtual void TextureBarrier(Texture* texture, TextureState from, TextureState to) = 0;
        virtual void CopyTexture(Texture* dst, Texture* src) = 0;


        virtual void BeginRenderPass(api::Viewport vp, api::ScissorRect rect,
            api::Texture* backBuffer, api::Texture* zBuffer,
            uint32_t clearFlags, const float clearColor[4], float clearDepth) = 0;

        virtual void EndRenderPass() = 0;

        virtual void InitializeBuffer(Buffer* buffer, const void* contents, size_t size ) = 0;

        virtual void ResolveTimerHeap(TimerHeap* heap) = 0;

        virtual void RecordTimer(TimerHeap* heap, size_t timerIndex) = 0;
    };

    typedef std::function<void()> FrameSyncCallback;

 
    class Device : public RefCount
    {
    public:

        // get the next render target in the swap chain
        virtual Texture* GetCurrentBackbuffer() = 0;

        // start a command list
        virtual CommandList* BeginCommandList() = 0;

        // submit a batch of command lists
        virtual void SubmitCommandList(CommandList* commandList) = 0;

        virtual RTASPrebuildInfo GetRTASPrebuildInfo(const RTASDesc& desc) = 0;

        virtual void ScheduleFrameSyncCallback(FrameSyncCallback cb) = 0;

        // Presents and advances to the next frame
        virtual void PresentAndAdvanceFrame() = 0;

        virtual void WaitForGPUIdle() = 0;

        virtual ComputePipeline* CreateComputePipeline(
            ShaderCompiler& compiler, 
            const WCHAR* path, 
            std::span<const RootParameterType> rootParams,
            std::span<const DxcDefine> defines = {},
            const WCHAR* entrypoint = L"CS_main") = 0;

        virtual RenderingPipeline* CreateRenderingPipeline(
            ShaderCompiler& compiler, 
            const WCHAR* path, 
            std::span<const RootParameterType> rootParams,
            std::span<const DxcDefine> defines = {} ) = 0;

        virtual Texture* CreateDepthStencilBuffer( uint32_t width, uint32_t height ) = 0;

        virtual Texture* CreateUAVTexture(uint32_t width, uint32_t height) = 0;

        virtual RTASBuffer* CreateRTASBuffer(size_t size) = 0;
        virtual RTAS* CreateRTAS(RTASBuffer* buffer, size_t offset, size_t size) = 0;

        virtual Buffer* CreateUAVBuffer(size_t size) = 0;
        virtual Buffer* CreateDynamicBuffer(size_t size) = 0;
        virtual Buffer* CreateBuffer(size_t size) = 0;
        
        // TODO: Deprecate the "Dynamic buffer" concept.. replace with upload/readback buffers and explicit frame sync
        //   Less API code this way
        virtual void* MapDynamicBuffer(Buffer* buffer) = 0;
        virtual void UnmapDynamicBuffer(Buffer* buffer, size_t byteWritten) = 0;
        virtual void WriteDynamicBuffer(Buffer* buffer, const void* bytes, size_t size) = 0;

        virtual Buffer* CreateUploadBuffer(size_t size) = 0;;
        virtual void* MapBuffer(Buffer* buffer, size_t readRangeSize) = 0;
        virtual void UnmapBuffer(Buffer* buffer, size_t writeRangeSize) = 0;

        virtual void IMGUI_NewFrame() = 0;
        
        virtual void IMGUI_RenderDrawData( api::CommandList* cmdList ) = 0;

        virtual bool SupportsNativeDGF() { return false;  }

        virtual TimerHeap* CreateTimerHeap( size_t numTimers ) = 0;

        virtual std::wstring GetAPIName() = 0;
    };


}
