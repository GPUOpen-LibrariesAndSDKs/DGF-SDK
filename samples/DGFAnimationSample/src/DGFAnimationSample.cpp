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

#include "DGFAnimationSample.h"
#include "DGF.h"

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx12.h"
#include "imgui_internal.h"

#include <array>
#include <algorithm>

namespace {
    DGFBaker::BakerOutput BakeMesh(const Mesh& mesh, const DGFBaker::Config& config, AABB vertexBox )
    {
        DGFBaker::BakerMesh bakerMesh = CreateDGFBakerMesh(mesh);
        DGFBaker::Baker     baker(config);

        DGFBaker::AnimationExtents extents(
            vertexBox.Min.x, vertexBox.Min.y, vertexBox.Min.z, 
            vertexBox.Max.x, vertexBox.Max.y, vertexBox.Max.z);

        bakerMesh.SetAnimationExtents(extents);

        return baker.BakeDGF(bakerMesh);
    }
}  // namespace

DGFAnimationSample::DGFAnimationSample(UINT width, UINT height, std::wstring name)
    : Sample(width, height, name),
      m_viewport(0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height)),
      m_scissorRect(0, 0, static_cast<LONG>(width), static_cast<LONG>(height))
{
}

DGFAnimationSample::~DGFAnimationSample() {}

void DGFAnimationSample::OnInit(api::Device* device)
{
    m_uiData.useNativeDGF = m_device->SupportsNativeDGF();
    m_device              = device;

    m_camera.Init({0.0f, 0.0f, 2.0f});
    m_camera.SetMoveSpeed(1.0f);

    LoadPipeline();

    if (!m_uiData.keyframeDirectory.empty()) {
        ChangeDirectory();
    }
}

// Load the rendering pipeline dependencies.
void DGFAnimationSample::LoadPipeline()
{
    m_zbuffer              = m_device->CreateDepthStencilBuffer(m_width, m_height);
    m_raytracingBackBuffer = m_device->CreateUAVTexture(m_width, m_height);

    m_dgfBvhBuilder.Init(m_device.Get(), m_compiler, api::RTAS_PREFER_FAST_TRACE);

    auto rtRootSig = {api::RootParameterType::IMAGE_UAV,
                      api::RootParameterType::CONSTANT_BUFFER,
                      api::RootParameterType::RTAS,
                      api::RootParameterType::BUFFER_SRV,
                      api::RootParameterType::BUFFER_SRV,
                      api::RootParameterType::BUFFER_SRV,
                      api::RootParameterType::BUFFER_SRV,
                      api::RootParameterType::BUFFER_SRV};

    m_raytracingComputeShader = m_device->CreateComputePipeline(m_compiler, L"RaytracingComputeShader.hlsl", rtRootSig);

    auto interpolationRootSig = {
        api::RootParameterType::CONSTANT_BUFFER,
        api::RootParameterType::BUFFER_SRV,
        api::RootParameterType::BUFFER_SRV,
        api::RootParameterType::BUFFER_SRV,
        api::RootParameterType::BUFFER_SRV,
        api::RootParameterType::BUFFER_UAV,
        api::RootParameterType::BUFFER_UAV,
    };

    m_interpolationShader =
        m_device->CreateComputePipeline(m_compiler, L"KeyFrameInterpolationCompute.hlsl", interpolationRootSig);

    auto quantizationRootSig = {
        api::RootParameterType::CONSTANT_BUFFER,
        api::RootParameterType::BUFFER_SRV,
        api::RootParameterType::BUFFER_SRV,
        api::RootParameterType::BUFFER_UAV,
    };

    m_requantizationShader = m_device->CreateComputePipeline(m_compiler, L"Requantization.hlsl", quantizationRootSig);

    m_timerHeap = m_device->CreateTimerHeap(2 * m_timings.count);
}

void DGFAnimationSample::BuildKeyframeBuffers()
{
    if (m_meshes.empty()) {
        return;
    }

    api::CommandList* cmdList = m_device->BeginCommandList();

    size_t positionBufferSize   = m_meshes[0].vertices.size() * sizeof(float);
    size_t normalBufferSize     = m_meshes[0].vertexNormals.size() * sizeof(uint32_t);
    size_t keyframeBufferSize   = positionBufferSize + normalBufferSize;
    size_t faceNormalBufferSize = m_meshes[0].surfaceNormals.size() * sizeof(uint32_t);

    std::vector<uint8_t> scratch(keyframeBufferSize);

    for (size_t i = 0; i < m_meshes.size(); i++) {
        if (m_meshes[i].vertices.size() != m_meshes[0].vertices.size() ||
            m_meshes[i].vertexNormals.size() != m_meshes[0].vertexNormals.size() ||
            m_meshes[i].surfaceNormals.size() != m_meshes[0].surfaceNormals.size())
        {
            THROW_RUNTIME_ERROR("Keyframe meshes have mismatched vertex or triangle counts");
        }

        api::Buffer* vertices    = m_device->CreateBuffer(keyframeBufferSize);
        api::Buffer* faceNormals = m_device->CreateBuffer(faceNormalBufferSize);

        Mesh& currentMesh = m_meshes[i];
        memcpy(scratch.data(), currentMesh.vertices.data(), positionBufferSize);
        memcpy(scratch.data() + positionBufferSize, currentMesh.vertexNormals.data(), normalBufferSize);

        cmdList->InitializeBuffer(vertices, scratch.data(), scratch.size());
        cmdList->InitializeBuffer(faceNormals, m_meshes[i].surfaceNormals.data(), faceNormalBufferSize);

        m_keyFrameVertices.push_back(vertices);
        m_keyFrameFaceNormals.push_back(faceNormals);
    }

    m_device->SubmitCommandList(cmdList);
    m_device->WaitForGPUIdle();
}

bool DGFAnimationSample::ChangeDirectory()
{
    if (m_uiData.keyframeDirectory == L"") {
        return false;
    }
    m_device->WaitForGPUIdle();
    m_keyFrameVertices.clear();
    m_keyFrameFaceNormals.clear();

    m_meshes.clear();

    std::unordered_set<std::string> loadableFileExtensions = {".obj", ".ply"};

    for (const auto& entry : std::filesystem::directory_iterator(m_uiData.keyframeDirectory)) {
        if (!std::filesystem::is_regular_file(entry)) {
            continue;
        }

        auto filePath = entry.path();

        const auto isLoadable =
            loadableFileExtensions.find(filePath.extension().string()) != loadableFileExtensions.end();

        if (isLoadable) {
            m_meshes.push_back(LoadMesh(filePath));
        }
    }

    if (m_meshes.empty()) {
        return false;
    }

    AABB vertexBox;

    std::vector<float3> animationVertices;
    animationVertices.reserve(m_meshes.size() * m_meshes.front().vertices.size() / 3);

    for (const auto& mesh : m_meshes) {
        for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
            float3 v = {mesh.vertices[i], mesh.vertices[i + 1], mesh.vertices[i + 2]};
            vertexBox.Expand(v);
        }
    }

    const Mesh&           firstMesh   = m_meshes.front();
    DGFBaker::BakerOutput bakerOutput = BakeMesh(m_meshes.front(), m_uiData.bakerConfig, vertexBox);

    m_dgfMesh = std::make_unique<DGFMesh>(bakerOutput, firstMesh);
    m_dgfMesh->UploadGpuResources(m_device.Get());

    for (auto& mesh : m_meshes) {
        auto tmp = mesh.surfaceNormals;
        RemapTriangleAttribute(tmp, mesh.surfaceNormals, bakerOutput.triangleRemap, 1);
    }

    BuildKeyframeBuffers();

    const size_t positionSize    = m_meshes.front().vertices.size() * sizeof(float);
    const size_t normalsSize     = m_meshes.front().vertexNormals.size() * sizeof(uint32_t);
    const size_t bufferSize      = positionSize + normalsSize;
    m_interpolatedVerticesBuffer = m_device->CreateUAVBuffer(bufferSize);

    m_animationHelper = std::make_unique<Animation::AnimationHelper>(m_meshes.size(), m_uiData.animationConfig);

    AllocateBVHResources();

    m_interpolationConstantBuffer = m_device->CreateDynamicBuffer(sizeof(InterpolationConstants));
    m_rtConstantsBuffer           = m_device->CreateDynamicBuffer(sizeof(RTConstants));

    m_uiData.needsDirectoryChange = false;
    m_timer.ResetElapsedTime();
    return true;
}
// Load the sample assets.
void DGFAnimationSample::RenderUI(api::CommandList* commandList)
{
    m_device->IMGUI_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("DGF Decompression Sample Options");

    if (ImGui::Button("Open Directory")) {
        m_uiData.keyframeDirectory   = OpenDirectoryDialog();
        m_uiData.bakingMeshInitiated = true;
    }

    ImGui::BeginDisabled(!m_device->SupportsNativeDGF());
    if (ImGui::Checkbox("Native DGF", &m_uiData.useNativeDGF)) {
        m_needBVHRealloc = true;
    }
    ImGui::EndDisabled();

    ImGui::Separator();

    if (ImGui::TreeNodeEx("Mesh Statistics", ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_Framed)) {
        if (!m_meshes.empty()) {
            const auto numTriangles = m_meshes.front().GetNumTriangles();
            const auto numVertices  = m_meshes.front().GetNumVertices();
            if (numTriangles > 100000) {
                ImGui::Text("Triangle Count : %.2fM", numTriangles / 1000000.0);
            } else {
                ImGui::Text("Triangle Count : %i", numTriangles);
            }

            if (numVertices > 100000) {
                ImGui::Text("Vertex Count : %.2fM", numVertices / 1000000.0);
            } else {
                ImGui::Text("Vertex Count : %i", numVertices);
            }
            std::string formattedSize       = "0 B";
            std::string formattedDGFDensity = "0 B";
            {
                size_t            size = m_meshes.front().GetMeshSizeInBytes();
                std::stringstream sstream;
                PrintByteSizeInBinaryUnits(sstream, size);
                formattedSize = sstream.str();
            }

            ImGui::Text("Mesh Size : %s", formattedSize.c_str());

            ImGui::Text("DGF Density : %.2f (Bytes/Tri)",
                        (float)m_dgfMesh->GetDGFBlocks().size() / m_dgfMesh->GetNumBlockTriangles());
        }
        ImGui::TreePop();
    }
    ImGui::Separator();

    if (ImGui::TreeNodeEx("Timings", ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_Framed)) {
        if (m_uiData.animationConfig.linearInterpolation) {
            ImGui::Text("%.4f ms: Vertex Interpolation", 1000 * m_timings.interpolationTime);
        } else {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.4f, 0.4f, 1.f));
            ImGui::Text("0.0000 ms: Vertex Interpolation");
            ImGui::PopStyleColor();
        }
        ImGui::Text("%.4f ms: Requantization", 1000 * m_timings.requantizationTime);
        if (m_uiData.useNativeDGF == false)
            ImGui::Text("%.4f ms: Decode DGF", 1000 * m_timings.decodeDGFTime);
        ImGui::Text("%.4f ms: BLAS Build", 1000 * m_timings.blasBuildTime);
        ImGui::Text("%.4f ms: TLAS Build", 1000 * m_timings.tlasBuildTime);
        ImGui::Text("%.4f ms: Ray Tracing", 1000 * m_timings.rayTracingTime);
        ImGui::TreePop();
    }

    ImGui::Separator();

    if (ImGui::TreeNodeEx("Baker parameters", ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_Framed)) {
        ImGui::Text("Baker parameters");
        
        static std::string current_item = m_offsetCombos.front().displayName;
        if (ImGui::BeginCombo("Vertex Bit Width", current_item.c_str())) {
            for (const auto [name, value] : m_offsetCombos) {
                bool is_selected = (current_item == name);
                if (ImGui::Selectable(name.c_str(), is_selected)) {
                    current_item                                   = name;
                    m_uiData.bakerConfig.blockForcedOffsetWidth[0] = value[0];
                    m_uiData.bakerConfig.blockForcedOffsetWidth[1] = value[1];
                    m_uiData.bakerConfig.blockForcedOffsetWidth[2] = value[2];
                }
            }
            ImGui::EndCombo();
        }

        if (ImGui::Button("Bake")) {
            m_uiData.bakingMeshInitiated = true;
        }
        ImGui::TreePop();
    }

    if (ImGui::TreeNodeEx("Display mode", ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_Framed)) {
        if (ImGui::RadioButton("Compressed Face Normal", m_uiData.displayMode == DisplayMode::Triangles)) {
            m_uiData.displayMode = DisplayMode::Triangles;
        }
        if (ImGui::RadioButton("DGF Blocks", m_uiData.displayMode == DisplayMode::Meshlets)) {
            m_uiData.displayMode = DisplayMode::Meshlets;
        }
        if (ImGui::RadioButton("GeomIDs", m_uiData.displayMode == DisplayMode::GeomIDs)) {
            m_uiData.displayMode = DisplayMode::GeomIDs;
        }

        if (m_dgfMesh && m_dgfMesh->HasSurfaceNormals()) {
            if (ImGui::RadioButton("Original Face Normals", m_uiData.displayMode == DisplayMode::SurfaceNormals)) {
                m_uiData.displayMode = DisplayMode::SurfaceNormals;
            }
        } else {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.4f, 0.4f, 1.f));
            ImGui::RadioButton("Original Face Normals", false);
            ImGui::PopStyleColor();
        }

        if (m_dgfMesh && m_dgfMesh->HasVertexNormals()) {
            if (ImGui::RadioButton("Original Vertex Normals", m_uiData.displayMode == DisplayMode::VertexNormals)) {
                m_uiData.displayMode = DisplayMode::VertexNormals;
            }
        } else {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.4f, 0.4f, 1.f));
            ImGui::RadioButton("Original Vertex Normals", false);
            ImGui::PopStyleColor();
        }
        ImGui::TreePop();
    }

    if (ImGui::TreeNodeEx("Animation Settings", ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_Framed)) {
        if (m_animationHelper) {
            if (ImGui::SliderFloat("Playback Speed", &m_uiData.animationConfig.playbackRate, 0.0f, 10.0f)) {
                m_animationHelper->SetPlaybackRate(m_uiData.animationConfig.playbackRate);
                if (!m_uiData.animationConfig.looping) {
                    m_animationHelper->Reset();
                }
            }

            if (ImGui::Checkbox("Looping", &m_uiData.animationConfig.looping)) {
                m_animationHelper->SetLoop(m_uiData.animationConfig.looping);
            }

            if (ImGui::Button(" Reset ")) {
                m_animationHelper->Reset();
            }
            ImGui::SameLine();
            if (m_animationHelper->IsEnabled()) {
                if (ImGui::Button("   Pause  ")) {
                    m_animationHelper->Pause();
                }
            } else {
                if (ImGui::Button(" Continue ")) {
                    m_animationHelper->Begin();
                }
            }
        }
        ImGui::Checkbox("Frame Interpolation", &m_uiData.animationConfig.linearInterpolation);

        ImGui::TreePop();
    }

    if (ImGui::TreeNodeEx("Camera", ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_Framed)) {
        if (ImGui::Button("Reset Camera")) {
            m_camera.Reset();
        }
        ImGui::TreePop();
    }

    if (m_uiData.bakingMeshInitiated) {
        const std::string message = "BAKING IN PROGRESS";
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(m_viewport.Width, m_viewport.Height), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.6f);
        if (ImGui::Begin("modal_dialog_bg",
                         nullptr,
                         ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoScrollbar |
                             ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize))
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
                         ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoScrollbar |
                             ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize))
        {
            ImGui::Text("%s", message.c_str());
            ImGui::End();
        }
        m_uiData.needsDirectoryChange = true;
        m_uiData.bakingMeshInitiated  = false;
    }

    ImGui::End();
    ImGui::Render();
    m_device->IMGUI_RenderDrawData(commandList);
}

void DGFAnimationSample::ResolveTimings()
{
    std::vector<double> timings(2 * m_timings.count);
    m_timerHeap->ReadTimers(timings.data(), 0, 2 * m_timings.count);

    m_averageTimings.interpolationTime += timings[1] - timings[0];
    m_averageTimings.requantizationTime += timings[3] - timings[2];
    m_averageTimings.decodeDGFTime += timings[5] - timings[4];
    m_averageTimings.blasBuildTime += timings[7] - timings[6];
    m_averageTimings.tlasBuildTime += timings[9] - timings[8];
    m_averageTimings.rayTracingTime += timings[11] - timings[10];

    m_timingElapsed += m_timer.GetElapsedSeconds();
    m_timingFrames++;
    if (m_timingElapsed >= 0.25) {
        double scale                 = 1.0 / m_timingFrames;
        m_timingFrames               = 0;
        m_timingElapsed              = 0;
        m_timings.interpolationTime  = m_averageTimings.interpolationTime * scale;
        m_timings.requantizationTime = m_averageTimings.requantizationTime * scale;
        m_timings.decodeDGFTime      = m_averageTimings.decodeDGFTime * scale;
        m_timings.blasBuildTime      = m_averageTimings.blasBuildTime * scale;
        m_timings.tlasBuildTime      = m_averageTimings.tlasBuildTime * scale;
        m_timings.rayTracingTime     = m_averageTimings.rayTracingTime * scale;
        m_averageTimings             = {};
    }
}

// Update frame-based values.
void DGFAnimationSample::OnUpdate()
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

    if (m_uiData.needsDirectoryChange) {
        ChangeDirectory();
    }

    if (m_animationHelper) {
        m_keyFrameInfo = m_animationHelper->GetKeyFrameInfo();
    }
}

// Render the scene.
void DGFAnimationSample::OnRender()
{
    if (m_needBVHRealloc) {
        m_device->WaitForGPUIdle();
        AllocateBVHResources();
        m_needBVHRealloc = false;
    }

    api::CommandList* cmdList    = m_device->BeginCommandList();
    api::Texture*     backBuffer = m_device->GetCurrentBackbuffer();

    if (m_dgfMesh) {
        if (m_uiData.animationConfig.linearInterpolation) {
            InterpolationPass(cmdList);
        }

        ReQuantizationPass(cmdList);
        BuildBVH(cmdList);

        RenderRaytraced(cmdList, backBuffer);

    } else {
        ClearRenderTarget(cmdList, backBuffer);
    }

    RenderUI(cmdList);

    cmdList->EndRenderPass();

    cmdList->TextureBarrier(backBuffer, api::TextureState::RENDER, api::TextureState::PRESENT);

    m_device->SubmitCommandList(cmdList);
    m_device->PresentAndAdvanceFrame();

    ResolveTimings();
}

void DGFAnimationSample::OnDestroy() {}

void DGFAnimationSample::OnKeyDown(UINT8 key)
{
    m_camera.OnKeyDown(key);
}

void DGFAnimationSample::OnKeyUp(UINT8 key)
{
    m_camera.OnKeyUp(key);
}

api::Buffer* DGFAnimationSample::GetCurrentKeyframeVertices()
{
    return m_keyFrameVertices[m_keyFrameInfo.currentIndex].Get();
}
api::Buffer* DGFAnimationSample::GetCurrentKeyframeNormals()
{
    return m_keyFrameFaceNormals[m_keyFrameInfo.currentIndex].Get();
}
api::Buffer* DGFAnimationSample::GetNextKeyframeVertices()
{
    return m_keyFrameVertices[m_keyFrameInfo.nextIndex].Get();
}

void DGFAnimationSample::ClearRenderTarget(api::CommandList* cmdList, api::Texture* backBuffer)
{
    const float clearColor[] = {0.0f, 0.2f, 0.4f, 1.0f};
    cmdList->BeginRenderPass(m_viewport, m_scissorRect, backBuffer, m_zbuffer.Get(), api::CLEAR_ALL, clearColor, 1.0f);
}

void DGFAnimationSample::InterpolationPass(api::CommandList* cmdList)
{
    cmdList->SetPipelineState(m_interpolationShader.Get());
    cmdList->BufferBarrier(m_interpolatedVerticesBuffer.Get(), api::BufferState::SRV, api::BufferState::UAV);

    api::Buffer* keyFrameVerts     = GetCurrentKeyframeVertices();
    api::Buffer* nextKeyFrameVerts = GetNextKeyframeVertices();
    const size_t vertexBufferSize  = sizeof(float) * m_meshes.front().vertices.size();
    cmdList->SetComputeRootSRV(1, keyFrameVerts, 0);
    cmdList->SetComputeRootSRV(2, keyFrameVerts, vertexBufferSize);
    cmdList->SetComputeRootSRV(3, nextKeyFrameVerts, 0);
    cmdList->SetComputeRootSRV(4, nextKeyFrameVerts, vertexBufferSize);

    cmdList->SetComputeRootUAV(5, m_interpolatedVerticesBuffer.Get(), 0);
    cmdList->SetComputeRootUAV(6, m_interpolatedVerticesBuffer.Get(), vertexBufferSize);

    InterpolationConstants constants;
    constants.InterpolationFactor = static_cast<float>(m_keyFrameInfo.interpolationFactor);
    constants.VertexCount         = static_cast<uint>(m_meshes.front().GetNumVertices());
    m_device->WriteDynamicBuffer(m_interpolationConstantBuffer.Get(), &constants, sizeof(constants));

    cmdList->RecordTimer(m_timerHeap.Get(), 0);
    cmdList->SetComputeRootCBV(0, m_interpolationConstantBuffer.Get(), 0);
    cmdList->Dispatch(static_cast<UINT>(std::ceil(constants.VertexCount / 64.0f)), 1, 1);
    cmdList->BufferBarrier(m_interpolatedVerticesBuffer.Get(), api::BufferState::UAV, api::BufferState::SRV);
    cmdList->RecordTimer(m_timerHeap.Get(), 1);
}

void DGFAnimationSample::ReQuantizationPass(api::CommandList* cmdList)
{
    cmdList->SetPipelineState(m_requantizationShader.Get());

    api::Buffer* blockBuffer = m_dgfMesh->GetDGFBlockBuffer();
    cmdList->BufferBarrier(blockBuffer, api::BufferState::SRV, api::BufferState::UAV);

    if (m_uiData.animationConfig.linearInterpolation) {
        cmdList->SetComputeRootSRV(1, m_interpolatedVerticesBuffer.Get(), 0);
    } else {
        cmdList->SetComputeRootSRV(1, GetCurrentKeyframeVertices(), 0);
    }

    cmdList->SetComputeRootSRV(2, m_dgfMesh->GetVertexTableBuffer(), 0);
    cmdList->SetComputeRootUAV(3, blockBuffer, 0);

    cmdList->RecordTimer(m_timerHeap.Get(), 2);
    for (const auto& dispatchPlan : m_dgfMesh->GetDispatchplan()) {
        cmdList->SetComputeRootCBV(0, m_dgfMesh->GetDispatchPlanConstantBuffer(), dispatchPlan.constantBufferOffset);
        cmdList->Dispatch(dispatchPlan.workgroupCount, 1, 1);
    }

    cmdList->BufferBarrier(blockBuffer, api::BufferState::UAV, api::BufferState::SRV);
    cmdList->RecordTimer(m_timerHeap.Get(), 3);
}