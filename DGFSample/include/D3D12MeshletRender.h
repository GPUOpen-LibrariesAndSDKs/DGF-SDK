//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#pragma once

#include "DXSample.h"
#include "StepTimer.h"
#include "SimpleCamera.h"
#include "DXFramework.h"
#include "Util.h"
#include "DGFBaker.h"
#include "DGFBVHBuilder.h"

using namespace DirectX;

// Note that while ComPtr is used to manage the lifetime of resources on the CPU,
// it has no understanding of the lifetime of resources on the GPU. Apps must account
// for the GPU lifetime of resources to avoid destroying objects that may still be
// referenced by the GPU.
// An example of this can be found in the class method: OnDestroy().
using Microsoft::WRL::ComPtr;

class D3D12MeshletRender : public DXSample {
public:
    D3D12MeshletRender(UINT width, UINT height, std::wstring name);
    ~D3D12MeshletRender();
    virtual void OnInit();
    virtual void OnUpdate();
    virtual void OnRender();
    virtual void OnDestroy();
    virtual void OnKeyDown(UINT8 key);
    virtual void OnKeyUp(UINT8 key);

private:
    static const UINT FrameCount = 2;

    enum class DisplayMode : uint {
        Triangles      = 0,
        Meshlets       = 1,
        SurfaceNormals = 2,
        VertexNormals  = 3,
        GeomIDs        = 4
    };

    struct UIData {
        dx::RenderingPipeline::RenderMode renderMode          = dx::RenderingPipeline::RenderMode::DGFBitScanWave;
        std::wstring                      meshFile            = L"";
        bool                              needsMeshChange     = false;
        bool                              bakingMeshInitiated = false;
        DGFBaker::Config                  bakerConfig         = {.validateClusters      = true,
                                                                 .generateVertexTable   = true,
                                                                 .generateTriangleRemap = true,
                                                                 .enableUserData        = true};

        DisplayMode displayMode = DisplayMode::Meshlets;
    } m_uiData;

    // Pipeline objects.
    CD3DX12_VIEWPORT               m_viewport;
    CD3DX12_RECT                   m_scissorRect;
    ComPtr<IDXGISwapChain3>        m_swapChain;
    ComPtr<ID3D12Device5>          m_device;
    ComPtr<ID3D12Resource>         m_renderTargets[FrameCount];
    ComPtr<ID3D12Resource>         m_depthStencil;
    ComPtr<ID3D12CommandAllocator> m_commandAllocators[FrameCount];
    ComPtr<ID3D12CommandQueue>     m_commandQueue;
    ComPtr<ID3D12DescriptorHeap>   m_rtvHeap;
    ComPtr<ID3D12DescriptorHeap>   m_dsvHeap;
    ComPtr<ID3D12DescriptorHeap>   m_srvHeap;

    UINT m_rtvDescriptorSize;
    UINT m_dsvDescriptorSize;
    UINT m_srvDescriptorSize;

    ComPtr<ID3D12GraphicsCommandList6> m_commandList;
    UINT8*                             m_cbvDataBegin;

    StepTimer    m_timer;
    SimpleCamera m_camera;

    // Synchronization objects.
    UINT                m_frameIndex;
    UINT                m_frameCounter;
    HANDLE              m_fenceEvent;
    ComPtr<ID3D12Fence> m_fence;
    UINT64              m_fenceValues[FrameCount];

    dx::ShaderCompiler       m_compiler;
    std::unique_ptr<DGFMesh> m_dgfModel;
    std::unique_ptr<Mesh>    m_originalModel;
    XMMATRIX                 m_modelMatrix;

    std::vector<dx::RenderingPipeline> m_pipelines = {
        dx::RenderingPipeline(dx::RenderingPipeline::RenderMode::DGFSerial, L"DGFSerialMeshShader.hlsl"),
        dx::RenderingPipeline(dx::RenderingPipeline::RenderMode::DGFBitScanWave, L"DGFBitScanWaveMeshShader.hlsl"),
        dx::RenderingPipeline(dx::RenderingPipeline::RenderMode::DGFBitScanLane, L"DGFBitScanLaneMeshShader.hlsl")};

    void LoadPipeline();
    void LoadAssets();
    void PopulateCommandList();
    void MoveToNextFrame();
    void WaitForGpu();
    void RenderUI();
    void RenderRasterized();
    void RenderRaytraced();

    dx::RenderingPipeline* GetPipeline(const dx::RenderingPipeline::RenderMode mode);

    bool ChangeMesh(const std::filesystem::path& pathToFile);

    ComPtr<ID3D12Resource>      m_raytracingBackBuffer;
    dx::ComputeShader           m_raytracingComputeShader;
    D3D12_GPU_DESCRIPTOR_HANDLE m_raytracingUavDescriptorHandle;
    DGFBVHBuilder               m_dgfBvhBuilder;
    ComPtr<ID3D12Resource>      m_asScratchResource;
    ComPtr<ID3D12Resource>      m_blas;
    ComPtr<ID3D12Resource>      m_tlas;
    ComPtr<ID3D12Resource>      m_rtasDecodeBuffer;
    ComPtr<ID3D12Resource>      m_instanceDescBuffer;
    bool                        m_needBVHUpdate = true;
    std::vector<uint32_t>       m_idToBlockMap;

    void AllocateBVHResources();
    void BuildBVH();
};
