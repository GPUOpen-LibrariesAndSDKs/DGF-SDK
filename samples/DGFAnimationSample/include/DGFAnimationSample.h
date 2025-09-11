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

#include "StepTimer.h"
#include "SimpleCamera.h"
#include "Util.h"
#include "DGFBaker.h"
#include "Animation.h"

#include "DGFBVHBuilder.h"

#include "Sample.h"

using namespace DirectX;

// Note that while ComPtr is used to manage the lifetime of resources on the CPU,
// it has no understanding of the lifetime of resources on the GPU. Apps must account
// for the GPU lifetime of resources to avoid destroying objects that may still be
// referenced by the GPU.
// An example of this can be found in the class method: OnDestroy().
using Microsoft::WRL::ComPtr;

class DGFAnimationSample : public Sample {
public:
    DGFAnimationSample(UINT width, UINT height, std::wstring name);
    ~DGFAnimationSample();
    virtual void OnInit(api::Device* device);
    virtual void OnUpdate();
    virtual void OnRender();
    virtual void OnDestroy();
    virtual void OnKeyDown(UINT8 key);
    virtual void OnKeyUp(UINT8 key);

private:
    static const UINT FrameCount = 2;

    enum class DisplayMode : uint { Triangles = 0, Meshlets = 1, SurfaceNormals = 2, VertexNormals = 3, GeomIDs = 4 };
    enum class BVHBuildMode : uint { Rebuild = 0, Update = 1 };

    struct Timings {
        size_t count = 6;

        double interpolationTime  = 0.0f;
        double requantizationTime = 0.0f;
        double decodeDGFTime      = 0.0f;
        double blasBuildTime      = 0.0f;
        double tlasBuildTime      = 0.0f;
        double rayTracingTime     = 0.0f;
    };

    Timings m_timings;
    Timings m_averageTimings;
    double  m_timingElapsed = 0;
    size_t  m_timingFrames  = 0;

    struct FixedOffset {
        std::string displayName;
        uint8_t     value[3];
    };
    const std::vector<FixedOffset> m_offsetCombos = {
        {"48b (16-16-16)", {16, 16, 16}},        
        {"36b (12-12-12)", {12, 12, 12}},
        {"32b (11-11-10)", {11, 11, 10}},
        {"24b (8-8-8)",    {8,   8,  8}}
    };

    struct UIData {
        std::wstring keyframeDirectory = GetExecutablePath() + L"\\data\\wooddoll";

        bool needsDirectoryChange = false;
        bool bakingMeshInitiated  = false;

        Animation::Config animationConfig = {};
        DGFBaker::Config  bakerConfig     = {.blockForcedOffsetWidth = {16, 16, 16},
                                             .validateClusters       = true,
                                             .generateVertexTable    = true,
                                             .generateTriangleRemap  = true,
                                             .enableUserData         = true,
                                             .targetBitWidth         = 24,
                                             .quantizeForAnimation   = true,      // use cluster diagonal instead of AABB extents for quantization
                                             .clusterDeformationPadding = 1.03f,  // fudge-factor for non-rigid animation
                                             .enableExponentAdjust   = false};

        DisplayMode  displayMode  = DisplayMode::VertexNormals;
        BVHBuildMode bvhBuildMode = BVHBuildMode::Rebuild;
        bool         useNativeDGF = false;
    } m_uiData;

    struct RTConstants {
        XMMATRIX viewProjInverse;
        XMMATRIX normalMatrix;
        uint     displayMode;
    };

    struct InterpolationConstants {
        float InterpolationFactor;
        uint  VertexCount;
    };

    StepTimer        m_timer;
    SimpleCamera     m_camera;
    api::Viewport    m_viewport;
    api::ScissorRect m_scissorRect;

    ShaderCompiler m_compiler;

    std::unique_ptr<DGFMesh> m_dgfMesh;
    std::vector<Mesh>        m_meshes;
    UINT                     m_frameCounter = 0;

    void ClearRenderTarget(api::CommandList* cmdList, api::Texture* backBuffer);
    void LoadPipeline();
    void RenderUI(api::CommandList* commandList);

    void InterpolationPass(api::CommandList* cmdList);
    void ReQuantizationPass(api::CommandList* cmdList);

    void BuildKeyframeBuffers();
    bool ChangeDirectory();

    void BuildBVH(api::CommandList* cmdList);
    void AllocateBVHResources();
    void RenderRaytraced(api::CommandList* cmdList, api::Texture* backBuffer);
    void ResolveTimings();

    std::vector<ComPtr<api::Buffer>> m_keyFrameVertices;
    std::vector<ComPtr<api::Buffer>> m_keyFrameFaceNormals;

    api::Buffer* GetCurrentKeyframeVertices();
    api::Buffer* GetNextKeyframeVertices();
    api::Buffer* GetCurrentKeyframeNormals();

    std::unique_ptr<Animation::AnimationHelper> m_animationHelper;
    Animation::KeyFrameInfo                     m_keyFrameInfo;
    ComPtr<api::Texture>                        m_zbuffer;
    ComPtr<api::Texture>                        m_raytracingBackBuffer;
    ComPtr<api::Buffer>                         m_asScratchResource;

    ComPtr<api::RTAS>   m_blas;
    ComPtr<api::RTAS>   m_tlas;
    ComPtr<api::Buffer> m_rtasDecodeBuffer;
    ComPtr<api::Buffer> m_instanceDescBuffer;
    ComPtr<api::Buffer> m_rtConstantsBuffer;
    ComPtr<api::Buffer> m_interpolationConstantBuffer;
    DGFBVHBuilder       m_dgfBvhBuilder;

    ComPtr<api::ComputePipeline> m_interpolationShader;
    ComPtr<api::ComputePipeline> m_raytracingComputeShader;
    ComPtr<api::ComputePipeline> m_requantizationShader;
    ComPtr<api::Buffer>          m_interpolatedVerticesBuffer;
    ComPtr<api::TimerHeap>       m_timerHeap;

    bool m_needBVHRealloc = false;
};
