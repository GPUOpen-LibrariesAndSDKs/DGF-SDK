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

#include "DGFSample.h"

#include <array>

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx12.h"
#include "imgui_internal.h"

namespace {

    DGFMesh BakeMesh(const Mesh& mesh, const DGFBaker::Config& config)
    {
        DGFBaker::BakerMesh   bakerMesh = CreateDGFBakerMesh(mesh);
        DGFBaker::Baker       baker(config);
        DGFBaker::BakerOutput dgfOutput = baker.BakeDGF(bakerMesh);

        return DGFMesh(dgfOutput, mesh);
    }

}  // namespace

DGFSample::DGFSample(UINT width, UINT height, std::wstring name)
    : Sample(width, height, name),
      m_viewport(0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height)),
      m_scissorRect(0, 0, static_cast<LONG>(width), static_cast<LONG>(height)),
      m_compiler()
{
    m_uiData.bakerConfig.enableUserData = true;
}

DGFSample::~DGFSample() {}

void DGFSample::OnInit(api::Device* device)
{
    m_uiData.useNativeDGF = device->SupportsNativeDGF();
    m_camera.Init({0.0f, 0.0f, 2.0f});
    m_camera.SetMoveSpeed(1.0f);

    m_device = device;

    LoadPipeline();
    LoadAssets();
}

// Load the rendering pipeline dependencies.
void DGFSample::LoadPipeline()
{
    const static auto                   SIXTY_FOUR      = std::to_wstring(64);
    const static std::vector<DxcDefine> g_shaderDefines = {DxcDefine({L"MS_NUM_THREADS", SIXTY_FOUR.c_str()}),
                                                           DxcDefine({L"MS_MAX_TRIS", SIXTY_FOUR.c_str()}),
                                                           DxcDefine({L"MS_MAX_VERTS", SIXTY_FOUR.c_str()})};

    auto rtRootSig = {
        api::RootParameterType::IMAGE_UAV,
        api::RootParameterType::CONSTANT_BUFFER,        
        api::RootParameterType::RTAS,
        api::RootParameterType::BUFFER_SRV,
        api::RootParameterType::BUFFER_SRV,
        api::RootParameterType::BUFFER_SRV,
        api::RootParameterType::BUFFER_SRV
    };

    m_raytracingComputeShader = m_device->CreateComputePipeline(m_compiler,L"RaytracingComputeShader.hlsl",rtRootSig, g_shaderDefines);

    auto rasterRootSig = {
        api::RootParameterType::CONSTANT_BUFFER,
        api::RootParameterType::CONSTANT_BUFFER,
        api::RootParameterType::BUFFER_SRV,
        api::RootParameterType::BUFFER_SRV,
        api::RootParameterType::BUFFER_SRV
    };

    m_serialRenderPipeline = m_device->CreateRenderingPipeline(m_compiler, L"DGFSerialMeshShader.hlsl", rasterRootSig,g_shaderDefines);
    m_bitscanWaveRenderPipeline = m_device->CreateRenderingPipeline(m_compiler, L"DGFBitScanWaveMeshShader.hlsl",rasterRootSig,g_shaderDefines);
    m_bitscanLaneRenderPipeline = m_device->CreateRenderingPipeline(m_compiler, L"DGFBitScanLaneMeshShader.hlsl",rasterRootSig,g_shaderDefines);

    m_zbuffer              = m_device->CreateDepthStencilBuffer(m_width, m_height);
    m_raytracingBackBuffer = m_device->CreateUAVTexture(m_width, m_height);
    m_rtConstantBuffer     = m_device->CreateDynamicBuffer(sizeof(RTConstants));
    m_rasterConstantBuffer = m_device->CreateDynamicBuffer(sizeof(RasterConsts));
}

// Load the sample assets.
void DGFSample::RenderUI(api::CommandList* cmdList)
{
    m_device->IMGUI_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();

    {
        ImGui::Begin("DGF Decompression Sample Options");

        if (ImGui::Button("Open File")) {
            m_uiData.meshFile            = OpenFileDialog();
            m_uiData.bakingMeshInitiated = true;
        } 
        
        if (m_uiData.renderMode == RenderMode::Raytracing) {            
            ImGui::BeginDisabled(!m_device->SupportsNativeDGF());  

            if (ImGui::Checkbox("Native DGF", &m_uiData.useNativeDGF)) {
                m_needBVHUpdate = true;
            }            

            ImGui::EndDisabled();            
        }

        ImGui::Separator();

        ImGui::Text("Baker parameters");

        ImGui::SliderInt("Target Bit Width", reinterpret_cast<int*>(&m_uiData.bakerConfig.targetBitWidth), 1, 24);

        if (ImGui::Button("Bake")) {
            m_uiData.bakingMeshInitiated = true;
        }
        if (m_uiData.bakingMeshInitiated) {
            if (m_uiData.meshFile.empty()) {
                ImGui::Text("Please open a Mesh before baking");
            } else {
                const std::string message = "BAKING IN PROGRESS";
                ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(m_viewport.Width, m_viewport.Height), ImGuiCond_Always);
                ImGui::SetNextWindowBgAlpha(0.6f);
                if (ImGui::Begin("modal_dialog_bg",
                                 nullptr,
                                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoInputs |
                                     ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoTitleBar |
                                     ImGuiWindowFlags_NoResize))
                {
                    ImGui::End();
                }

                // We can not rely on ImGui auto window sizing, as this will only be available
                // in the next frame and this window will only be shown for one frame

                // calculate text size
                const auto textSize = ImGui::CalcTextSize(message.c_str());
                // get window padding
                const auto windowPadding = ImGui::GetStyle().WindowPadding;

                // window size = text size + padding
                const auto windowSize = ImVec2(textSize.x + (2 * windowPadding.x), textSize.y + (2 * windowPadding.y));

                // Position window in center of frame
                ImGui::SetNextWindowPos(
                    ImVec2(m_viewport.Width / 2, m_viewport.Height / 2), ImGuiCond_Always, ImVec2(0.5, 0.5));
                ImGui::SetNextWindowSize(windowSize, ImGuiCond_Always);
                if (ImGui::Begin("modal_dialog",
                                 nullptr,
                                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoInputs |
                                     ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoTitleBar |
                                     ImGuiWindowFlags_NoResize))
                {
                    ImGui::Text("%s", message.c_str());
                    ImGui::End();
                }
                m_uiData.needsMeshChange     = true;
                m_uiData.bakingMeshInitiated = false;
            }
        }

        ImGui::Separator();

        ImGui::Text("Mesh Statistics");

        std::string dgfBlockSizeStr        = "0 B";
        std::string dgfVertexNormalSizeStr = "0 B";

        std::string originalMeshSizeStr      = "0 B";
        std::string originalNormalsSizeStr   = "0 B";
        std::string attributeOverheadSizeStr = "0 B";

        std::string totalSizeCompressed   = "0 B";
        std::string totalSizeUncompressed = "0 B";
        size_t      numTris               = 0;
        size_t      compressedSize        = 0;
        size_t      uncompressedSize      = 0;

        if (m_dgfModel && m_originalModel) {
            numTris = m_originalModel->GetNumTriangles();
            {
                std::stringstream sstream;
                PrintByteSizeInBinaryUnits(sstream, m_dgfModel->GetNumDGFBlocks() * DGF::BLOCK_SIZE);
                dgfBlockSizeStr = sstream.str();
            }

            {
                std::stringstream sstream;
                PrintByteSizeInBinaryUnits(sstream, m_dgfModel->GetNumBlockVertices() * sizeof(uint32_t));
                dgfVertexNormalSizeStr = sstream.str();
            }

            {
                size_t inputMeshSize = 12 * m_originalModel->GetNumVertices() + 12 * m_originalModel->GetNumTriangles();
                std::stringstream sstream;
                PrintByteSizeInBinaryUnits(sstream, inputMeshSize);
                originalMeshSizeStr = sstream.str();
            }

            {
                size_t            originalNormalsSize = 4 * m_originalModel->GetNumVertices();
                std::stringstream sstream;
                PrintByteSizeInBinaryUnits(sstream, originalNormalsSize);
                originalNormalsSizeStr = sstream.str();
            }
            {
                compressedSize =
                    m_dgfModel->GetNumDGFBlocks() * DGF::BLOCK_SIZE + 4 * m_dgfModel->GetNumBlockVertices();
                std::stringstream sstream;
                PrintByteSizeInBinaryUnits(sstream, compressedSize);
                totalSizeCompressed = sstream.str();
            }
            {
                uncompressedSize = 12 * m_originalModel->GetNumVertices() + 12 * m_originalModel->GetNumTriangles() +
                                   4 * m_originalModel->GetNumVertices();
                std::stringstream sstream;
                PrintByteSizeInBinaryUnits(sstream, uncompressedSize);
                totalSizeUncompressed = sstream.str();
            }

            {
                std::stringstream sstream;

                size_t size = 0;
                if (m_dgfModel->HasVertexNormals()) {
                    size += (m_dgfModel->GetNumBlockVertices() - m_originalModel->GetNumVertices()) * sizeof(uint32_t);
                }

                PrintByteSizeInBinaryUnits(sstream, size);

                attributeOverheadSizeStr = sstream.str();
            }
        }

        if (numTris > 0) {
            ImGui::Text("Triangle Count : %.2fM", numTris / 1000000.0);
            ImGui::Text("Compressed: %s\n", totalSizeCompressed.c_str());
            ImGui::Text("Uncompressed: %s (%.2fx)\n\n",
                        totalSizeUncompressed.c_str(),
                        (double)uncompressedSize / (double)compressedSize);
            ImGui::Text("\tDGF Mesh Size : %s", dgfBlockSizeStr.c_str());
            ImGui::Text("\tInput Mesh Size : %s", originalMeshSizeStr.c_str());
            ImGui::Text("\tDGF Normals Size : %s", dgfVertexNormalSizeStr.c_str());
            ImGui::Text("\tInput Normals Size : %s", originalNormalsSizeStr.c_str());
            ImGui::Text("\t\tDuplication Overhead : %s", attributeOverheadSizeStr.c_str());
        }

        ImGui::Separator();
        ImGui::Text("Rendering method");

        if (ImGui::RadioButton("DGF Bit Scan Decompression (Wave)", m_uiData.renderMode == RenderMode::DGFBitScanWave))
        {
            m_uiData.renderMode = RenderMode::DGFBitScanWave;
            m_timer.ResetElapsedTime();
        }

        if (ImGui::RadioButton("DGF Bit Scan Decompression (Lane)", m_uiData.renderMode == RenderMode::DGFBitScanLane))
        {
            m_uiData.renderMode = RenderMode::DGFBitScanLane;
            m_timer.ResetElapsedTime();
        }

        if (ImGui::RadioButton("Raytracing", m_uiData.renderMode == RenderMode::Raytracing)) {
            m_uiData.renderMode = RenderMode::Raytracing;
            m_timer.ResetElapsedTime();
        }

        if (ImGui::RadioButton("DGF Serial Decompression", m_uiData.renderMode == RenderMode::DGFSerial)) {
            m_uiData.renderMode = RenderMode::DGFSerial;
            m_timer.ResetElapsedTime();
        }

        ImGui::Separator();
        ImGui::Text("Display mode");

        if (ImGui::RadioButton("Compressed Face Normal", m_uiData.displayMode == DisplayMode::Triangles)) {
            m_uiData.displayMode = DisplayMode::Triangles;
        }
        if (ImGui::RadioButton("DGF Blocks", m_uiData.displayMode == DisplayMode::Meshlets)) {
            m_uiData.displayMode = DisplayMode::Meshlets;
        }
        if (ImGui::RadioButton("GeomIDs", m_uiData.displayMode == DisplayMode::GeomIDs)) {
            m_uiData.displayMode = DisplayMode::GeomIDs;
        }

        if (m_dgfModel && m_dgfModel->HasSurfaceNormals()) {
            if (ImGui::RadioButton("Original Face Normals", m_uiData.displayMode == DisplayMode::SurfaceNormals)) {
                m_uiData.displayMode = DisplayMode::SurfaceNormals;
            }
        } else {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.4f, 0.4f, 1.f));
            ImGui::RadioButton("Original Face Normals", false);
            ImGui::PopStyleColor();
        }

        if (m_dgfModel && m_dgfModel->HasVertexNormals()) {
            if (ImGui::RadioButton("Original Vertex Normals", m_uiData.displayMode == DisplayMode::VertexNormals)) {
                m_uiData.displayMode = DisplayMode::VertexNormals;
            }
        } else {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.4f, 0.4f, 1.f));
            ImGui::RadioButton("Original Vertex Normals", false);
            ImGui::PopStyleColor();
        }

        ImGui::Separator();

        if (ImGui::Button("Reset Camera")) {
            m_camera.Reset();
        }
        ImGui::End();
    }
    ImGui::Render();

    m_device->IMGUI_RenderDrawData(cmdList);
}

api::RenderingPipeline* DGFSample::GetPipeline(const RenderMode mode)
{
    switch (mode) {
    case RenderMode::DGFSerial:
        return m_serialRenderPipeline.Get();
    case RenderMode::DGFBitScanLane:
        return m_bitscanLaneRenderPipeline.Get();
    case RenderMode::DGFBitScanWave:
        return m_bitscanWaveRenderPipeline.Get();
    }

    return nullptr;
}

void DGFSample::LoadAssets()
{
    m_dgfBvhBuilder.Init(m_device.Get(), m_compiler, api::RTAS_PREFER_FAST_TRACE);
}

// Update frame-based values.
void DGFSample::OnUpdate()
{
    m_timer.Tick(NULL);
    if (m_frameCounter++ % 30 == 0) {
        // Update window text with FPS value.
        wchar_t fps[256];
        swprintf_s(
            fps, L"%ufps, %.2fms per frame", m_timer.GetFramesPerSecond(), 1e3f / (float)m_timer.GetFramesPerSecond());
        SetCustomWindowText(fps);
    }
    m_camera.Update(static_cast<float>(m_timer.GetElapsedSeconds()));

    if (m_uiData.needsMeshChange) {
        ChangeMesh(m_uiData.meshFile);
    }
}

// Render the scene.
void DGFSample::OnRender()
{
    auto cmdList = PopulateCommandList();
    m_device->SubmitCommandList(cmdList);
    m_device->PresentAndAdvanceFrame();
}

void DGFSample::OnDestroy() {}

void DGFSample::OnKeyDown(UINT8 key)
{
    m_camera.OnKeyDown(key);
}

void DGFSample::OnKeyUp(UINT8 key)
{
    m_camera.OnKeyUp(key);
}

bool DGFSample::ChangeMesh(const std::filesystem::path& pathToFile)
{
    Mesh newMesh;

    try {
        if (pathToFile.extension() == ".ply" || pathToFile.extension() == ".obj") {
            newMesh = LoadMesh(pathToFile);
        } else {
            throw std::runtime_error("Unknown file type");
        }

    } catch (...) {
        m_uiData.needsMeshChange = false;
        return false;
    }

    m_originalModel = std::make_unique<Mesh>(newMesh);

    DGFMesh newDGFMesh = BakeMesh(newMesh, m_uiData.bakerConfig);

    m_dgfModel = std::make_unique<DGFMesh>(newDGFMesh);
    m_dgfModel->UploadGpuResources(m_device.Get());

    auto model      = m_dgfModel->GetNormalizationTransformation();
    m_modelMatrix   = XMMATRIX(reinterpret_cast<float*>(&model));
    m_needBVHUpdate = true;

    m_uiData.needsMeshChange = false;
    m_timer.ResetElapsedTime();

    return true;
}

api::CommandList* DGFSample::RenderRasterized(api::Texture* backBuffer)
{
    auto pipeline = GetPipeline(m_uiData.renderMode);
    if (!pipeline) {
        throw std::exception("Unknown pipeline");
    }

    api::CommandList* cmdList = m_device->BeginCommandList();
    cmdList->SetPipelineState(pipeline);
    
    cmdList->TextureBarrier(backBuffer, api::TextureState::PRESENT, api::TextureState::RENDER);

    const float clearColor[] = {0.0f, 0.2f, 0.4f, 1.0f};

    const float clearDepth   = 1.0f;
    cmdList->BeginRenderPass(
        m_viewport, m_scissorRect, backBuffer, m_zbuffer.Get(), api::CLEAR_ALL, clearColor, clearDepth);
    
    if (m_dgfModel) {
        cmdList->SetGraphicsRootSRV(2, m_dgfModel->GetDGFBlockBuffer(), 0);
        if (m_dgfModel->HasSurfaceNormals()) {
            cmdList->SetGraphicsRootSRV(3, m_dgfModel->GetSurfaceNormalsBuffer(), 0);
        }
        if (m_dgfModel->HasVertexNormals()) {
            cmdList->SetGraphicsRootSRV(4, m_dgfModel->GetVertexNormalsBuffer(), 0);
        }

        XMMATRIX view = m_camera.GetViewMatrix();
        XMMATRIX proj = m_camera.GetProjectionMatrix(XM_PI / 3.0f, m_aspectRatio, 0.01f, 10000.0f);



        RasterConsts rasterConsts = {m_modelMatrix * view * proj,
                                     m_modelMatrix,
                                     XMMatrixTranspose(XMMatrixInverse(nullptr, m_modelMatrix)),
                                     m_camera.GetCameraPosition(),
                                     static_cast<uint>(m_uiData.displayMode)};

        m_device->WriteDynamicBuffer(m_rasterConstantBuffer.Get(), &rasterConsts, sizeof(RasterConsts));
        cmdList->SetGraphicsRootCBV(0, m_rasterConstantBuffer.Get(), 0);
        
        for (const auto& dispatchPlan : m_dgfModel->GetDispatchplan()) {
            cmdList->SetGraphicsRootCBV(1,m_dgfModel->GetDispatchPlanConstantBuffer(),dispatchPlan.constantBufferOffset);
            cmdList->DispatchMesh(dispatchPlan.workgroupCount, 1, 1);
        }
    }

    return cmdList;
}

api::CommandList* DGFSample::RenderRaytraced(api::Texture* backBuffer)
{
    if (m_needBVHUpdate) {
        AllocateBVHResources();
        BuildBVH();
        m_needBVHUpdate = false;
    }

    api::CommandList* cmdList = m_device->BeginCommandList();

    XMMATRIX view        = m_camera.GetViewMatrix();
    XMMATRIX proj        = m_camera.GetProjectionMatrix(XM_PI / 3.0f, m_aspectRatio, 0.01f, 10000.0f);
    XMMATRIX viewProj    = m_modelMatrix * view * proj;
    XMMATRIX viewProjInv = XMMatrixInverse(nullptr, viewProj);

    RTConstants consts;
    consts.viewProjInverse = viewProjInv;
    consts.displayMode     = (uint)m_uiData.displayMode;
    consts.normalMatrix    = XMMatrixTranspose(XMMatrixInverse(nullptr, m_modelMatrix));
    m_device->WriteDynamicBuffer(m_rtConstantBuffer.Get(), &consts, sizeof(consts));

    cmdList->SetPipelineState(m_raytracingComputeShader.Get());

    cmdList->SetComputeRootUAV(0, m_raytracingBackBuffer.Get()); 
    cmdList->SetComputeRootCBV(1, m_rtConstantBuffer.Get(), 0);
    cmdList->SetComputeRootSRV(2, m_tlas.Get());
    cmdList->SetComputeRootSRV(3, m_dgfModel->GetIDToBlockMapVirtualAddress(),0);

    cmdList->SetComputeRootSRV(4, m_dgfModel->GetSurfaceNormalsBuffer(), 0);
    cmdList->SetComputeRootSRV(5, m_dgfModel->GetVertexNormalsBuffer(), 0);
    cmdList->SetComputeRootSRV(6, m_dgfModel->GetDGFBlockBuffer(), 0);

    uint XGroups = ((UINT)m_viewport.Width + 7) / 8;
    uint YGroups = ((UINT)m_viewport.Height + 7) / 8;
    cmdList->Dispatch(XGroups, YGroups, 1);

    cmdList->TextureBarrier(m_raytracingBackBuffer.Get(), api::TextureState::UAV, api::TextureState::COPY_SOURCE);
    cmdList->TextureBarrier(backBuffer, api::TextureState::PRESENT, api::TextureState::COPY_DEST);

    cmdList->CopyTexture(backBuffer, m_raytracingBackBuffer.Get());

    cmdList->TextureBarrier(m_raytracingBackBuffer.Get(), api::TextureState::COPY_SOURCE, api::TextureState::UAV);
    cmdList->TextureBarrier(backBuffer, api::TextureState::COPY_DEST, api::TextureState::RENDER);

    cmdList->BeginRenderPass(m_viewport, m_scissorRect, backBuffer, m_zbuffer.Get(), api::CLEAR_DEPTH, nullptr, 1.0f);
    
    return cmdList;
}

void DGFSample::AllocateBVHResources()
{
    DGFBVHBuilder::PrebuildInfo dgfPBI = m_dgfBvhBuilder.GetPrebuildInfo(*m_dgfModel, m_uiData.useNativeDGF);

    api::RTASDesc tlasDesc;
    tlasDesc.Type                = api::RTASType::TOP_LEVEL;
    tlasDesc.TLAS.InstanceBuffer = 0;
    tlasDesc.TLAS.NumInstances   = 1;
    tlasDesc.Flags               = 0;

    api::RTASPrebuildInfo tlasPBI = m_device->GetRTASPrebuildInfo(tlasDesc);

    size_t scratchResourceSize = std::max(dgfPBI.bvhBuildScratchSize, (UINT)tlasPBI.buildScratchSize);

    m_tlas = m_device->CreateRTAS(tlasPBI.rtasSize);
    m_blas = m_device->CreateRTAS(dgfPBI.bvhSize);

    m_asScratchResource  = m_device->CreateUAVBuffer(scratchResourceSize);
    m_rtasDecodeBuffer   = m_device->CreateUAVBuffer(dgfPBI.decodeScratchSize);
    m_instanceDescBuffer = m_device->CreateDynamicBuffer(sizeof(api::InstanceDesc));
}

api::CommandList* DGFSample::PopulateCommandList()
{
    api::CommandList* cmdList    = nullptr;
    api::Texture*     backBuffer = m_device->GetCurrentBackbuffer();
    auto              mode       = m_uiData.renderMode;
    if (mode == RenderMode::Raytracing) {
        cmdList = RenderRaytraced(backBuffer);
    } else {
        cmdList = RenderRasterized(backBuffer);
    }

    RenderUI(cmdList);
    cmdList->EndRenderPass();

    cmdList->TextureBarrier(backBuffer, api::TextureState::RENDER, api::TextureState::PRESENT);

    return cmdList;
}

void DGFSample::BuildBVH()
{
    api::CommandList* cmdList = m_device->BeginCommandList();

    // build the blas
    m_dgfBvhBuilder.Build(m_rtasDecodeBuffer.Get(), 0, m_asScratchResource.Get(), m_blas.Get(), *m_dgfModel, cmdList, m_uiData.useNativeDGF);

    cmdList->UAVBarrier();

    // build the tlas
    api::InstanceDesc instanceDesc;
    instanceDesc.InstanceContributionToHitGroupIndex = 0;
    instanceDesc.Flags                               = 0;
    instanceDesc.InstanceContributionToHitGroupIndex = 0;
    instanceDesc.InstanceMask                        = 0xff;
    instanceDesc.AccelerationStructure               = m_blas->GetRTASReference();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            instanceDesc.Transform[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    m_device->WriteDynamicBuffer(m_instanceDescBuffer.Get(), &instanceDesc, sizeof(instanceDesc));

    api::RTASDesc tlasDesc;
    tlasDesc.Type                     = api::RTASType::TOP_LEVEL;
    tlasDesc.Destination              = m_tlas.Get();
    tlasDesc.Scratch                  = m_asScratchResource.Get();
    tlasDesc.ScratchByteOffset        = 0;
    tlasDesc.TLAS.InstanceBuffer      = m_instanceDescBuffer.Get();
    tlasDesc.TLAS.InstancesByteOffset = 0;
    tlasDesc.TLAS.NumInstances        = 1;
    tlasDesc.Flags                    = 0;

    cmdList->BuildRTAS(tlasDesc);

    m_device->SubmitCommandList(cmdList);
    m_device->WaitForGPUIdle();
}