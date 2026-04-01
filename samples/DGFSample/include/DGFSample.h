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

#include "Sample.h"
#include "StepTimer.h"
#include "SimpleCamera.h"
#include "api.h"
#include "Util.h"
#include "DGFBaker.h"
#include "DGFBVHBuilder.h"

using namespace DirectX;

using Microsoft::WRL::ComPtr;

class DGFSample : public Sample {
public:
    DGFSample(UINT width, UINT height, std::wstring name);
    ~DGFSample();
    virtual void OnInit(api::Device* device) override;
    virtual void OnUpdate() override;
    virtual void OnRender() override;
    virtual void OnDestroy() override;
    virtual void OnKeyDown(UINT8 key) override;
    virtual void OnKeyUp(UINT8 key) override;

private:
    static const UINT FrameCount = 2;

    enum class DisplayMode : uint {
        Triangles      = 0,
        Meshlets       = 1,
        SurfaceNormals = 2,
        VertexNormals  = 3,
        GeomIDs        = 4
    };

    enum class RenderMode : int32_t { DGFSerial = 0, DGFBitScanWave = 1, DGFBitScanLane = 2, Raytracing = 3 };


    struct UIData {
        RenderMode renderMode          = RenderMode::DGFBitScanWave;
        std::wstring                      meshFile            = L"";
        bool                              needsMeshChange     = false;
        bool                              bakingMeshInitiated = false;
        DGFBaker::Config                  bakerConfig         = {.validateClusters      = true,
                                                                 .generateVertexTable   = true,
                                                                 .generateTriangleRemap = true,
                                                                 .enableUserData        = true,
                                                                 .allowInternalThreading = true};

        DisplayMode displayMode = DisplayMode::Meshlets;
        bool        useNativeDGF = false;
    } m_uiData;

    struct RasterConsts {
        XMMATRIX modelViewProjectionMatrix;
        XMMATRIX modelWorldMatrix;
        XMMATRIX normalMatrix;
        XMVECTOR cameraPosition;
        uint     displayMode;
    };

    struct RTConstants {
        XMMATRIX viewProjInverse;
        XMMATRIX normalMatrix;
        uint     displayMode;
    };

    // Pipeline objects.
    api::Viewport               m_viewport;
    api::ScissorRect            m_scissorRect;

    StepTimer       m_timer;
    SimpleCamera    m_camera;
    ShaderCompiler  m_compiler;

    std::unique_ptr<DGFMesh> m_dgfModel;
    std::unique_ptr<Mesh>    m_originalModel;
    XMMATRIX                 m_modelMatrix;
    UINT                     m_frameCounter = 0;

    ComPtr<api::ComputePipeline>                m_raytracingComputeShader;

    void LoadPipeline();
    void LoadAssets();
    api::CommandList* PopulateCommandList();
    void RenderUI(api::CommandList* cmdList);
    api::CommandList* RenderRasterized(api::Texture* backBuffer);
    api::CommandList* RenderRaytraced(api::Texture* backBuffer);

    ComPtr<api::RenderingPipeline> m_serialRenderPipeline;
    ComPtr<api::RenderingPipeline> m_bitscanWaveRenderPipeline;
    ComPtr<api::RenderingPipeline> m_bitscanLaneRenderPipeline;

    api::RenderingPipeline* GetPipeline(const RenderMode mode);

    bool ChangeMesh(const std::filesystem::path& pathToFile);

    DGFBVHBuilder m_dgfBvhBuilder;
    
    ComPtr<api::Texture>    m_zbuffer;
    ComPtr<api::Texture>    m_raytracingBackBuffer;
    ComPtr<api::Buffer>     m_asScratchResource;
    ComPtr<api::RTASBuffer> m_rtasBuffer;
    ComPtr<api::RTAS>       m_blas;
    ComPtr<api::RTAS>       m_tlas;
    size_t                  m_blasOffset = 0;
    size_t                  m_tlasOffset = 0;
    ComPtr<api::Buffer>  m_rtasDecodeBuffer;
    ComPtr<api::Buffer>  m_instanceDescBuffer;
    ComPtr<api::Buffer>  m_rasterConstantBuffer;
    ComPtr<api::Buffer>  m_rtConstantBuffer;

    bool                        m_needBVHUpdate = true;
    std::vector<uint32_t>       m_idToBlockMap;

    void AllocateBVHResources();
    void BuildBVH();
};
