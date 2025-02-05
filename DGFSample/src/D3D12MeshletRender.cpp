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

#include "stdafx.h"
#include "D3D12MeshletRender.h"

#include "imgui.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_dx12.h"
#include "imgui_internal.h"

#include <array>

namespace {

    enum class BinaryByteUnit : std::uint32_t {
        // Automatically choose the next binary unit.
        Auto     = 0,
        Byte     = 1,
        KibiByte = 2,
        MebiByte = 3,
        GibiByte = 4,
        TebiByte = 5,
        Count,
        Default = Auto
    };

    const std::unordered_map<BinaryByteUnit, std::string> binaryUnitMap = {{BinaryByteUnit::Byte, "B"},
                                                                           {BinaryByteUnit::KibiByte, "KiB"},
                                                                           {BinaryByteUnit::MebiByte, "MiB"},
                                                                           {BinaryByteUnit::GibiByte, "GiB"},
                                                                           {BinaryByteUnit::TebiByte, "TiB"}};

    std::uint32_t CountBinaryUnitMultiples(std::uint64_t value)
    {
        value /= 1024;
        return (value > 0) ? 1 + CountBinaryUnitMultiples(value) : 0;
    }

    void PrintByteSizeInBinaryUnits(std::ostream&       ostream,
                                    const std::uint64_t value,
                                    BinaryByteUnit      binaryUnit = BinaryByteUnit::Auto)
    {
        // Compute the byte size in binary units.
        double byteSize = static_cast<double>(value);

        // Automatically compute the next binary unit from value.
        if (binaryUnit == BinaryByteUnit::Auto) {
            // Count the number of multiples of 1024.
            const auto multipleCount = CountBinaryUnitMultiples(value);
            // Set the binary unit enum based on the multiple count. Add 1 as enum value 0
            // represents the auto mode.
            binaryUnit = static_cast<BinaryByteUnit>(std::min(
                multipleCount + 1, static_cast<std::underlying_type_t<BinaryByteUnit>>(BinaryByteUnit::Count) - 1));
        }

        // Compute the binary unit size based on the user input.
        for (std::int32_t count = static_cast<std::underlying_type_t<BinaryByteUnit>>(binaryUnit) - 1; count > 0;
             --count)
        {
            byteSize /= 1024.f;
        }

        std::string          unitStr = binaryUnitMap.at(binaryUnit);
        std::array<char, 20> buffer;
        std::snprintf(buffer.data(), buffer.size(), "%3.3f", byteSize);
        ostream << std::setw(7) << buffer.data() << " " << unitStr;
    }

    template <class T>
    void RemapTriangleAttribute(const std::vector<T>&                           input,
                                std::vector<T>&                                 output,
                                const std::vector<DGFBaker::TriangleRemapInfo>& remap,
                                const size_t                                    nChannels)
    {
        assert(input.size() == output.size());
        for (size_t i = 0; i < remap.size(); ++i) {
            for (size_t j = 0; j < nChannels; ++j) {
                output[nChannels * i + j] = input[nChannels * remap[i].InputPrimIndex + j];
            }
        }
    }

    template <class T>
    void RemapVertexAttribute(const std::vector<T>&  input,
                              std::vector<T>&        output,
                              DGFBaker::BakerOutput& bakerOutput,
                              const size_t           nChannels)
    {
        const std::vector<uint32_t>& vertexTable = bakerOutput.vertexTable;
        std::vector<uint8_t>&        dgfBlocks   = bakerOutput.dgfBlocks;

        uint32_t vertexOffset = 0;

        for (size_t i = 0; i < dgfBlocks.size(); i += DGF::BLOCK_SIZE) {
            uint8_t* pBlock   = dgfBlocks.data() + i;
            size_t   numVerts = DGF::DecodeVertexCount(pBlock);

            // build the duplicated attribute array.
            for (size_t j = 0; j < numVerts; j++) {
                uint32_t inputIndex = vertexTable[vertexOffset++];
                for (size_t j = 0; j < nChannels; ++j) {
                    output.push_back(input[nChannels * inputIndex + j]);
                }
            }
        }
    }

    DGFMesh BakeMesh(const Mesh& mesh, const DGFBaker::Config& config)
    {
        DGFBaker::BakerMesh   bakerMesh = CreateDGFBakerMesh(mesh);
        DGFBaker::Baker       baker(config);
        DGFBaker::BakerOutput dgfOutput = baker.BakeDGF(bakerMesh);

        std::vector<uint32_t> remappedSurfaceNormals;
        if (!mesh.surfaceNormals.empty()) {
            remappedSurfaceNormals.resize(mesh.surfaceNormals.size());
            RemapTriangleAttribute(mesh.surfaceNormals, remappedSurfaceNormals, dgfOutput.triangleRemap, 1);
        }

        std::vector<uint32_t> remappedVertexNormals;
        if (!mesh.vertexNormals.empty()) {
            remappedVertexNormals.reserve(2 * mesh.vertexNormals.size());
            RemapVertexAttribute(mesh.vertexNormals, remappedVertexNormals, dgfOutput, 1);
        }

        return DGFMesh(dgfOutput.dgfBlocks, remappedSurfaceNormals, remappedVertexNormals);
    }

    Mesh LoadMesh(const std::filesystem::path& pathToFile)
    {
        Mesh mesh;
        if (pathToFile.extension() == ".ply") {
            mesh = ParsePlyFile(pathToFile);
        } else if (pathToFile.extension() == ".obj") {
            mesh = ParseObjFile(pathToFile, false);
        } else {
            throw std::runtime_error("Unknown file type");
        }

        if (mesh.surfaceNormals.empty()) {
            auto unpackedNormals = ComputeScaledSurfaceNormals(
                mesh.indices.data(), mesh.indices.size(), mesh.vertices.data(), mesh.vertices.size() / 3);
            NormalizeScaledSurfaceNormals(unpackedNormals.data(), unpackedNormals.size() / 3);
            mesh.surfaceNormals = PackFloat3x10(unpackedNormals.data(), unpackedNormals.size() / 3);
        }

        if (mesh.vertexNormals.empty()) {
            const auto unpackedNormals = ComputeVertexNormals(
                mesh.indices.data(), mesh.indices.size(), mesh.vertices.data(), mesh.vertices.size() / 3);

            mesh.vertexNormals = PackFloat3x10(unpackedNormals.data(), unpackedNormals.size() / 3);
        }
        return mesh;
    }
}  // namespace

D3D12MeshletRender::D3D12MeshletRender(UINT width, UINT height, std::wstring name)
    : DXSample(width, height, name),
      m_viewport(0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height)),
      m_scissorRect(0, 0, static_cast<LONG>(width), static_cast<LONG>(height)),
      m_rtvDescriptorSize(0),
      m_dsvDescriptorSize(0),
      m_srvDescriptorSize(0),
      m_cbvDataBegin(nullptr),
      m_frameIndex(0),
      m_frameCounter(0),
      m_fenceEvent{},
      m_fenceValues{},
      m_compiler(),
      m_raytracingComputeShader(L"RaytracingComputeShader.hlsl")
{
    m_uiData.bakerConfig.enableUserData = true;
}

D3D12MeshletRender::~D3D12MeshletRender()
{
    ImGui_ImplDX12_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
}

void D3D12MeshletRender::OnInit()
{
    m_camera.Init({0.0f, 0.0f, 2.0f});
    m_camera.SetMoveSpeed(1.0f);

    LoadPipeline();
    LoadAssets();

    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplWin32_Init(Win32Application::GetHwnd());
    ImGui_ImplDX12_Init(m_device.Get(),
                        FrameCount,
                        DXGI_FORMAT_R8G8B8A8_UNORM,
                        m_srvHeap.Get(),
                        m_srvHeap->GetCPUDescriptorHandleForHeapStart(),
                        m_srvHeap->GetGPUDescriptorHandleForHeapStart());
}

// Load the rendering pipeline dependencies.
void D3D12MeshletRender::LoadPipeline()
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

    if (m_useWarpDevice) {
        ComPtr<IDXGIAdapter> warpAdapter;
        ThrowIfFailed(factory->EnumWarpAdapter(IID_PPV_ARGS(&warpAdapter)));

        ThrowIfFailed(D3D12CreateDevice(warpAdapter.Get(), D3D_FEATURE_LEVEL_12_2, IID_PPV_ARGS(&m_device)));
    } else {
        ComPtr<IDXGIAdapter1> hardwareAdapter;
        GetHardwareAdapter(factory.Get(), &hardwareAdapter, true);

        ThrowIfFailed(D3D12CreateDevice(hardwareAdapter.Get(), D3D_FEATURE_LEVEL_12_2, IID_PPV_ARGS(&m_device)));
    }

    D3D12_FEATURE_DATA_SHADER_MODEL shaderModel = {D3D_SHADER_MODEL_6_7};
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
    queueDesc.Flags                    = D3D12_COMMAND_QUEUE_FLAG_NONE;
    queueDesc.NodeMask                 = 0;
    queueDesc.Priority                 = D3D12_COMMAND_QUEUE_PRIORITY_NORMAL;
    queueDesc.Type                     = D3D12_COMMAND_LIST_TYPE_DIRECT;

    ThrowIfFailed(m_device->CreateCommandQueue(&queueDesc, IID_PPV_ARGS(&m_commandQueue)));

    // Describe and create the swap chain.
    DXGI_SWAP_CHAIN_DESC1 swapChainDesc = {};
    swapChainDesc.BufferCount           = FrameCount;
    swapChainDesc.Width                 = m_width;
    swapChainDesc.Height                = m_height;
    swapChainDesc.Format                = DXGI_FORMAT_R8G8B8A8_UNORM;
    swapChainDesc.BufferUsage           = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    swapChainDesc.SwapEffect            = DXGI_SWAP_EFFECT_FLIP_DISCARD;
    swapChainDesc.SampleDesc.Count      = 1;
    swapChainDesc.Flags                 = DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING;

    ComPtr<IDXGISwapChain3> swapChain;
    ThrowIfFailed(factory->CreateSwapChainForHwnd(
        m_commandQueue.Get(),  // Swap chain needs the queue so that it can force a flush on it.
        Win32Application::GetHwnd(),
        &swapChainDesc,
        nullptr,
        nullptr,
        (IDXGISwapChain1**)swapChain.GetAddressOf()));

    // This sample does not support fullscreen transitions.
    ThrowIfFailed(factory->MakeWindowAssociation(Win32Application::GetHwnd(), DXGI_MWA_NO_ALT_ENTER));

    ThrowIfFailed(swapChain.As(&m_swapChain));
    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();

    // Create descriptor heaps.
    {
        // Describe and create a render target view (RTV) descriptor heap.
        D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc = {};
        rtvHeapDesc.NumDescriptors             = FrameCount;
        rtvHeapDesc.Type                       = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        rtvHeapDesc.Flags                      = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
        ThrowIfFailed(m_device->CreateDescriptorHeap(&rtvHeapDesc, IID_PPV_ARGS(&m_rtvHeap)));

        m_rtvDescriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

        // Describe and create a render target view (RTV) descriptor heap.
        D3D12_DESCRIPTOR_HEAP_DESC dsvHeapDesc = {};
        dsvHeapDesc.NumDescriptors             = 1;
        dsvHeapDesc.Type                       = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
        dsvHeapDesc.Flags                      = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
        ThrowIfFailed(m_device->CreateDescriptorHeap(&dsvHeapDesc, IID_PPV_ARGS(&m_dsvHeap)));

        m_dsvDescriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_DSV);

        D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
        srvHeapDesc.NumDescriptors             = 2;
        srvHeapDesc.Type                       = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
        srvHeapDesc.Flags                      = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        ThrowIfFailed(m_device->CreateDescriptorHeap(&srvHeapDesc, IID_PPV_ARGS(&m_srvHeap)));

        m_srvDescriptorSize = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
    }

    // Create frame resources.
    {
        CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(m_rtvHeap->GetCPUDescriptorHandleForHeapStart());

        // Create a RTV and a command allocator for each frame.
        for (UINT n = 0; n < FrameCount; n++) {
            ThrowIfFailed(m_swapChain->GetBuffer(n, IID_PPV_ARGS(&m_renderTargets[n])));
            m_device->CreateRenderTargetView(m_renderTargets[n].Get(), nullptr, rtvHandle);
            rtvHandle.Offset(1, m_rtvDescriptorSize);

            ThrowIfFailed(m_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT,
                                                           IID_PPV_ARGS(&m_commandAllocators[n])));
        }
    }

    // Create the depth stencil view.
    {
        D3D12_DEPTH_STENCIL_VIEW_DESC depthStencilDesc = {};
        depthStencilDesc.Format                        = DXGI_FORMAT_D32_FLOAT;
        depthStencilDesc.ViewDimension                 = D3D12_DSV_DIMENSION_TEXTURE2D;
        depthStencilDesc.Flags                         = D3D12_DSV_FLAG_NONE;

        D3D12_CLEAR_VALUE depthOptimizedClearValue    = {};
        depthOptimizedClearValue.Format               = DXGI_FORMAT_D32_FLOAT;
        depthOptimizedClearValue.DepthStencil.Depth   = 1.0f;
        depthOptimizedClearValue.DepthStencil.Stencil = 0;

        const CD3DX12_HEAP_PROPERTIES depthStencilHeapProps(D3D12_HEAP_TYPE_DEFAULT);
        const CD3DX12_RESOURCE_DESC   depthStencilTextureDesc = CD3DX12_RESOURCE_DESC::Tex2D(
            DXGI_FORMAT_D32_FLOAT, m_width, m_height, 1, 0, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL);

        ThrowIfFailed(m_device->CreateCommittedResource(&depthStencilHeapProps,
                                                        D3D12_HEAP_FLAG_NONE,
                                                        &depthStencilTextureDesc,
                                                        D3D12_RESOURCE_STATE_DEPTH_WRITE,
                                                        &depthOptimizedClearValue,
                                                        IID_PPV_ARGS(&m_depthStencil)));

        NAME_D3D12_OBJECT(m_depthStencil);

        m_device->CreateDepthStencilView(
            m_depthStencil.Get(), &depthStencilDesc, m_dsvHeap->GetCPUDescriptorHandleForHeapStart());
    }

    // Create the raytracing backbuffer
    {
        const CD3DX12_HEAP_PROPERTIES rtHeapProps(D3D12_HEAP_TYPE_DEFAULT);
        const CD3DX12_RESOURCE_DESC   rtTextureDesc = CD3DX12_RESOURCE_DESC::Tex2D(
            DXGI_FORMAT_R8G8B8A8_UNORM, m_width, m_height, 1, 1, 1, 0, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

        ThrowIfFailed(m_device->CreateCommittedResource(&rtHeapProps,
                                                        D3D12_HEAP_FLAG_NONE,
                                                        &rtTextureDesc,
                                                        D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
                                                        nullptr,
                                                        IID_PPV_ARGS(&m_raytracingBackBuffer)));

        D3D12_UNORDERED_ACCESS_VIEW_DESC uavDesc;
        uavDesc.Format               = DXGI_FORMAT_R8G8B8A8_UNORM;
        uavDesc.ViewDimension        = D3D12_UAV_DIMENSION_TEXTURE2D;
        uavDesc.Texture2D.MipSlice   = 0;
        uavDesc.Texture2D.PlaneSlice = 0;

        auto handle = m_srvHeap->GetCPUDescriptorHandleForHeapStart();
        handle.ptr += m_srvDescriptorSize;
        m_device->CreateUnorderedAccessView(m_raytracingBackBuffer.Get(), nullptr, &uavDesc, handle);

        m_raytracingUavDescriptorHandle = m_srvHeap->GetGPUDescriptorHandleForHeapStart();
        m_raytracingUavDescriptorHandle.ptr += m_srvDescriptorSize;
    }
}

// Load the sample assets.
void D3D12MeshletRender::RenderUI()
{
    ImGui_ImplDX12_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();

    {
        ImGui::Begin("DGF Decompression Sample Options");

        if (ImGui::Button("Open File")) {
            m_uiData.meshFile            = OpenFileDialog();
            m_uiData.bakingMeshInitiated = true;
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

        std::string dgfBlockSizeStr          = "0 B";
        std::string dgfVertexNormalSizeStr   = "0 B";
        
        std::string originalMeshSizeStr      = "0 B";
        std::string originalNormalsSizeStr   = "0 B";
        std::string attributeOverheadSizeStr = "0 B";

        std::string totalSizeCompressed = "0 B";
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
                size_t originalNormalsSize = 4 * m_originalModel->GetNumVertices();
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
                uncompressedSize = 12 * m_originalModel->GetNumVertices() +
                                           12 * m_originalModel->GetNumTriangles() +
                                           4 * m_originalModel->GetNumVertices();
                std::stringstream sstream;
                PrintByteSizeInBinaryUnits(sstream, uncompressedSize);
                totalSizeUncompressed = sstream.str();
            }

            {
                std::stringstream sstream;

                size_t size = 0;
                if (m_dgfModel->HasVertexNormals()) {
                    size +=
                        (m_dgfModel->GetNumBlockVertices() - m_originalModel->GetNumVertices()) * sizeof(uint32_t);
                }

                PrintByteSizeInBinaryUnits(sstream, size);

                attributeOverheadSizeStr = sstream.str();
            }
        }

        if (numTris > 0) 
        {
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

        if (ImGui::RadioButton("DGF Bit Scan Decompression (Wave)",
                               m_uiData.renderMode == dx::RenderingPipeline::RenderMode::DGFBitScanWave))
        {
            m_uiData.renderMode = dx::RenderingPipeline::RenderMode::DGFBitScanWave;
            m_timer.ResetElapsedTime();
        }

        if (ImGui::RadioButton("DGF Bit Scan Decompression (Lane)",
                               m_uiData.renderMode == dx::RenderingPipeline::RenderMode::DGFBitScanLane))
        {
            m_uiData.renderMode = dx::RenderingPipeline::RenderMode::DGFBitScanLane;
            m_timer.ResetElapsedTime();
        }

        if (ImGui::RadioButton("Raytracing", m_uiData.renderMode == dx::RenderingPipeline::RenderMode::Raytracing)) {
            m_uiData.renderMode = dx::RenderingPipeline::RenderMode::Raytracing;
            m_timer.ResetElapsedTime();
        }

        if (ImGui::RadioButton("DGF Serial Decompression",
                               m_uiData.renderMode == dx::RenderingPipeline::RenderMode::DGFSerial))
        {
            m_uiData.renderMode = dx::RenderingPipeline::RenderMode::DGFSerial;
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

    ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), m_commandList.Get());
}

dx::RenderingPipeline* D3D12MeshletRender::GetPipeline(const dx::RenderingPipeline::RenderMode mode)
{
    for (auto& pipeline : m_pipelines) {
        if (pipeline.GetMode() == mode) {
            return &pipeline;
        }
    }
    return nullptr;
}

void D3D12MeshletRender::LoadAssets()
{
    for (auto& pipeline : m_pipelines) {
        pipeline.Init(m_device.Get(), &m_compiler, m_renderTargets[0]->GetDesc().Format);
    }

    m_raytracingComputeShader.Init(m_device.Get(), &m_compiler);

    m_dgfBvhBuilder.Init(m_device.Get(), &m_compiler);

    // Create the command list.
    ThrowIfFailed(m_device->CreateCommandList(0,
                                              D3D12_COMMAND_LIST_TYPE_DIRECT,
                                              m_commandAllocators[m_frameIndex].Get(),
                                              nullptr,
                                              IID_PPV_ARGS(&m_commandList)));

    // Command lists are created in the recording state, but there is nothing
    // to record yet. The main loop expects it to be closed, so close it now.
    ThrowIfFailed(m_commandList->Close());

    // Create synchronization objects and wait until assets have been uploaded to the GPU.
    {
        ThrowIfFailed(m_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&m_fence)));
        m_fenceValues[m_frameIndex]++;

        // Create an event handle to use for frame synchronization.
        m_fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr);
        if (m_fenceEvent == nullptr) {
            ThrowIfFailed(HRESULT_FROM_WIN32(GetLastError()));
        }

        // Wait for the command list to execute; we are reusing the same command
        // list in our main loop but for now, we just want to wait for setup to
        // complete before continuing.
        WaitForGpu();
    }
}

// Update frame-based values.
void D3D12MeshletRender::OnUpdate()
{
    m_timer.Tick(NULL);
    if (m_frameCounter++ % 30 == 0) {
        // Update window text with FPS value.
        wchar_t fps[256];
        swprintf_s(fps, L"%ufps, %.2fms per frame", m_timer.GetFramesPerSecond(), 1e3f / (float)m_timer.GetFramesPerSecond());
        SetCustomWindowText(fps);
    }
    m_camera.Update(static_cast<float>(m_timer.GetElapsedSeconds()));

    if (m_uiData.needsMeshChange) {
        ChangeMesh(m_uiData.meshFile);
    }
}

// Render the scene.
void D3D12MeshletRender::OnRender()
{
    // Record all the commands we need to render the scene into the command list.
    PopulateCommandList();

    // Execute the command list.
    ID3D12CommandList* ppCommandLists[] = {m_commandList.Get()};
    m_commandQueue->ExecuteCommandLists(_countof(ppCommandLists), ppCommandLists);

    // Present the frame.
    ThrowIfFailed(m_swapChain->Present(0, DXGI_PRESENT_ALLOW_TEARING));

    MoveToNextFrame();
}

void D3D12MeshletRender::OnDestroy()
{
    // Ensure that the GPU is no longer referencing resources that are about to be
    // cleaned up by the destructor.
    WaitForGpu();

    CloseHandle(m_fenceEvent);
}

void D3D12MeshletRender::OnKeyDown(UINT8 key)
{
    m_camera.OnKeyDown(key);
}

void D3D12MeshletRender::OnKeyUp(UINT8 key)
{
    m_camera.OnKeyUp(key);
}

bool D3D12MeshletRender::ChangeMesh(const std::filesystem::path& pathToFile)
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
    m_dgfModel->UploadGpuResources(
        m_device.Get(), m_commandQueue.Get(), m_commandAllocators[m_frameIndex].Get(), m_commandList.Get());

    auto model      = m_dgfModel->GetNormalizationTransformation();
    m_modelMatrix   = XMMATRIX(reinterpret_cast<float*>(&model));
    m_needBVHUpdate = true;

    m_uiData.needsMeshChange = false;
    m_timer.ResetElapsedTime();

    return true;
}

void D3D12MeshletRender::RenderRasterized()
{
    auto pipeline = GetPipeline(m_uiData.renderMode);

    if (!pipeline) {
        throw std::exception("Unknown pipeline");
    }

    // However, when ExecuteCommandList() is called on a particular command
    // list, that command list can then be reset at any time and must be before
    // re-recording.
    ThrowIfFailed(m_commandList->Reset(m_commandAllocators[m_frameIndex].Get(), pipeline->GetPipelineState().Get()));
    m_commandList->SetDescriptorHeaps(1, m_srvHeap.GetAddressOf());

    // Set necessary state.
    m_commandList->SetGraphicsRootSignature(pipeline->GetRootSignature().Get());
    m_commandList->RSSetViewports(1, &m_viewport);
    m_commandList->RSSetScissorRects(1, &m_scissorRect);

    // Indicate that the back buffer will be used as a render target.
    const auto toRenderTargetBarrier = CD3DX12_RESOURCE_BARRIER::Transition(
        m_renderTargets[m_frameIndex].Get(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET);
    m_commandList->ResourceBarrier(1, &toRenderTargetBarrier);

    CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(
        m_rtvHeap->GetCPUDescriptorHandleForHeapStart(), m_frameIndex, m_rtvDescriptorSize);
    CD3DX12_CPU_DESCRIPTOR_HANDLE dsvHandle(m_dsvHeap->GetCPUDescriptorHandleForHeapStart());
    m_commandList->OMSetRenderTargets(1, &rtvHandle, FALSE, &dsvHandle);

    // Record commands.
    const float clearColor[] = {0.0f, 0.2f, 0.4f, 1.0f};
    m_commandList->ClearRenderTargetView(rtvHandle, clearColor, 0, nullptr);
    m_commandList->ClearDepthStencilView(dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);
    if (m_dgfModel) {
        m_commandList->SetGraphicsRootShaderResourceView(2, m_dgfModel->GetDGFBlockBuffer());
        if (m_dgfModel->HasSurfaceNormals()) {
            m_commandList->SetGraphicsRootShaderResourceView(3, m_dgfModel->GetSurfaceNormalsBuffer());
        }
        if (m_dgfModel->HasVertexNormals()) {
            m_commandList->SetGraphicsRootShaderResourceView(4, m_dgfModel->GetVertexNormalsBuffer());
        }

        XMMATRIX view = m_camera.GetViewMatrix();
        XMMATRIX proj = m_camera.GetProjectionMatrix(XM_PI / 3.0f, m_aspectRatio, 0.01f, 10000.0f);

        struct RasterConsts {
            XMMATRIX modelViewProjectionMatrix;
            XMMATRIX modelWorldMatrix;
            XMMATRIX normalMatrix;
            XMVECTOR cameraPosition;
            uint     displayMode;
        };

        RasterConsts rasterConsts = {m_modelMatrix * view * proj,
                                     m_modelMatrix,
                                     XMMatrixTranspose(XMMatrixInverse(nullptr, m_modelMatrix)),
                                     m_camera.GetCameraPosition(),
                                     static_cast<uint>(m_uiData.displayMode)};
        m_commandList->SetGraphicsRoot32BitConstants(
            0, sizeof(RasterConsts) / 4, reinterpret_cast<const UINT*>(&rasterConsts), 0);

        for (const auto& dispatchPlan : m_dgfModel->GetDispatchplan()) {
            m_commandList->SetGraphicsRoot32BitConstants(1, 1, &dispatchPlan.x, 0);
            m_commandList->DispatchMesh(dispatchPlan.y, 1, 1);
        }
    }
}

void D3D12MeshletRender::RenderRaytraced()
{
    if (m_needBVHUpdate) {
        AllocateBVHResources();
        BuildBVH();
        m_needBVHUpdate = false;
    }

    auto pipeline = m_raytracingComputeShader.GetPipelineState();
    auto rootSig  = m_raytracingComputeShader.GetRootSignature();
    ThrowIfFailed(m_commandList->Reset(m_commandAllocators[m_frameIndex].Get(), nullptr));
    m_commandList->SetDescriptorHeaps(1, m_srvHeap.GetAddressOf());

    XMMATRIX view        = m_camera.GetViewMatrix();
    XMMATRIX proj        = m_camera.GetProjectionMatrix(XM_PI / 3.0f, m_aspectRatio, 0.01f, 10000.0f);
    XMMATRIX viewProj    = m_modelMatrix * view * proj;
    XMMATRIX viewProjInv = XMMatrixInverse(nullptr, viewProj);


    struct RTConstants
    {
        XMMATRIX viewProjInverse;
        XMMATRIX normalMatrix;
        uint     displayMode;
    };

    RTConstants consts;
    consts.viewProjInverse = viewProjInv;
    consts.displayMode     = (uint) m_uiData.displayMode;
    consts.normalMatrix    = XMMatrixTranspose(XMMatrixInverse(nullptr, m_modelMatrix));

    m_commandList->SetComputeRootSignature(rootSig.Get());
    m_commandList->SetPipelineState(pipeline.Get());
    m_commandList->SetComputeRootDescriptorTable(0, m_raytracingUavDescriptorHandle);
    m_commandList->SetComputeRoot32BitConstants(1, 33, &consts, 0);
    m_commandList->SetComputeRootShaderResourceView(2, m_tlas->GetGPUVirtualAddress());
    m_commandList->SetComputeRootShaderResourceView(3, m_dgfModel->GetIDToBlockMapVirtualAddress());
    m_commandList->SetComputeRootShaderResourceView(4, m_dgfModel->GetSurfaceNormalsBuffer());
    m_commandList->SetComputeRootShaderResourceView(5, m_dgfModel->GetVertexNormalsBuffer());
    m_commandList->SetComputeRootShaderResourceView(6, m_dgfModel->GetDGFBlockBuffer());

    uint XGroups = ((UINT)m_viewport.Width + 7) / 8;
    uint YGroups = ((UINT)m_viewport.Height + 7) / 8;
    m_commandList->Dispatch(XGroups, YGroups, 1);

    ID3D12Resource*        swapChainBuffer = m_renderTargets[m_frameIndex].Get();
    ID3D12Resource*        rtBuffer        = m_raytracingBackBuffer.Get();
    D3D12_RESOURCE_BARRIER barriers[4]     = {
        CD3DX12_RESOURCE_BARRIER::Transition(
            rtBuffer, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_STATE_COPY_SOURCE),
        CD3DX12_RESOURCE_BARRIER::Transition(
            swapChainBuffer, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_COPY_DEST),
        CD3DX12_RESOURCE_BARRIER::Transition(
            rtBuffer, D3D12_RESOURCE_STATE_COPY_SOURCE, D3D12_RESOURCE_STATE_UNORDERED_ACCESS),
        CD3DX12_RESOURCE_BARRIER::Transition(
            swapChainBuffer, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_RENDER_TARGET),
    };

    m_commandList->ResourceBarrier(2, &barriers[0]);
    m_commandList->CopyResource(swapChainBuffer, rtBuffer);
    m_commandList->ResourceBarrier(2, &barriers[2]);

    // setup render target for GUI
    m_commandList->RSSetViewports(1, &m_viewport);
    m_commandList->RSSetScissorRects(1, &m_scissorRect);

    CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(
        m_rtvHeap->GetCPUDescriptorHandleForHeapStart(), m_frameIndex, m_rtvDescriptorSize);
    CD3DX12_CPU_DESCRIPTOR_HANDLE dsvHandle(m_dsvHeap->GetCPUDescriptorHandleForHeapStart());
    m_commandList->OMSetRenderTargets(1, &rtvHandle, FALSE, &dsvHandle);

    // Record commands.
    const float clearColor[] = {0.0f, 0.2f, 0.4f, 1.0f};
    m_commandList->ClearDepthStencilView(dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);
}

void D3D12MeshletRender::AllocateBVHResources()
{
    DGFBVHBuilder::PrebuildInfo dgfPBI = m_dgfBvhBuilder.GetPrebuildInfo(*m_dgfModel);

    // Allocate the BLAS, TLAS, scratch buffer, and decode buffer
    //
    //   In a real implementation the decode buffer and scratch buffer would
    //     exist in a "transient" resource heap managed by the engine
    //

    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS tlasInputs = {};
    tlasInputs.Flags       = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;
    tlasInputs.NumDescs    = 1;
    tlasInputs.Type        = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
    tlasInputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO tlasPBI;
    m_device->GetRaytracingAccelerationStructurePrebuildInfo(&tlasInputs, &tlasPBI);

    size_t scratchResourceSize = std::max(dgfPBI.bvhBuildScratchSize, (UINT)tlasPBI.ScratchDataSizeInBytes);

    auto blasDesc = CD3DX12_RESOURCE_DESC::Buffer(dgfPBI.bvhSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
    auto tlasDesc =
        CD3DX12_RESOURCE_DESC::Buffer(tlasPBI.ResultDataMaxSizeInBytes, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
    auto scratchDesc = CD3DX12_RESOURCE_DESC::Buffer(scratchResourceSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
    auto defaultHeap = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT);

    m_blas              = nullptr;
    m_tlas              = nullptr;
    m_asScratchResource = nullptr;

    ThrowIfFailed(m_device->CreateCommittedResource(&defaultHeap,
                                                    D3D12_HEAP_FLAG_NONE,
                                                    &scratchDesc,
                                                    D3D12_RESOURCE_STATE_COMMON,
                                                    nullptr,
                                                    IID_PPV_ARGS(&m_asScratchResource)));

    ThrowIfFailed(m_device->CreateCommittedResource(&defaultHeap,
                                                    D3D12_HEAP_FLAG_NONE,
                                                    &blasDesc,
                                                    D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE,
                                                    nullptr,
                                                    IID_PPV_ARGS(&m_blas)));

    ThrowIfFailed(m_device->CreateCommittedResource(&defaultHeap,
                                                    D3D12_HEAP_FLAG_NONE,
                                                    &tlasDesc,
                                                    D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE,
                                                    nullptr,
                                                    IID_PPV_ARGS(&m_tlas)));
    auto decodeBufferDesc =
        CD3DX12_RESOURCE_DESC::Buffer(dgfPBI.decodeScratchSize, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);

    ThrowIfFailed(m_device->CreateCommittedResource(&defaultHeap,
                                                    D3D12_HEAP_FLAG_NONE,
                                                    &scratchDesc,
                                                    D3D12_RESOURCE_STATE_COMMON,
                                                    nullptr,
                                                    IID_PPV_ARGS(&m_rtasDecodeBuffer)));

    // Allocate an upload heap for the instance descriptors

    auto uploadHeap       = CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_UPLOAD);
    auto instanceDescDesc = CD3DX12_RESOURCE_DESC::Buffer(sizeof(D3D12_RAYTRACING_INSTANCE_DESC));

    ThrowIfFailed(m_device->CreateCommittedResource(&uploadHeap,
                                                    D3D12_HEAP_FLAG_NONE,
                                                    &instanceDescDesc,
                                                    D3D12_RESOURCE_STATE_GENERIC_READ,
                                                    nullptr,
                                                    IID_PPV_ARGS(&m_instanceDescBuffer)));
}

void D3D12MeshletRender::PopulateCommandList()
{
    // Command list allocators can only be reset when the associated
    // command lists have finished execution on the GPU; apps should use
    // fences to determine GPU execution progress.
    ThrowIfFailed(m_commandAllocators[m_frameIndex]->Reset());

    auto mode = m_uiData.renderMode;
    if (mode == dx::RenderingPipeline::RenderMode::Raytracing) {
        RenderRaytraced();
    } else {
        RenderRasterized();
    }

    RenderUI();

    // Indicate that the back buffer will now be used to present.
    const auto toPresentBarrier = CD3DX12_RESOURCE_BARRIER::Transition(
        m_renderTargets[m_frameIndex].Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);
    m_commandList->ResourceBarrier(1, &toPresentBarrier);

    ThrowIfFailed(m_commandList->Close());
}

// Wait for pending GPU work to complete.
void D3D12MeshletRender::WaitForGpu()
{
    // Schedule a Signal command in the queue.
    ThrowIfFailed(m_commandQueue->Signal(m_fence.Get(), m_fenceValues[m_frameIndex]));

    // Wait until the fence has been processed.
    ThrowIfFailed(m_fence->SetEventOnCompletion(m_fenceValues[m_frameIndex], m_fenceEvent));
    WaitForSingleObject(m_fenceEvent, INFINITE);

    // Increment the fence value for the current frame.
    m_fenceValues[m_frameIndex]++;
}

// Prepare to render the next frame.
void D3D12MeshletRender::MoveToNextFrame()
{
    // Schedule a Signal command in the queue.
    const UINT64 currentFenceValue = m_fenceValues[m_frameIndex];
    ThrowIfFailed(m_commandQueue->Signal(m_fence.Get(), currentFenceValue));

    // Update the frame index.
    m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();

    // If the next frame is not ready to be rendered yet, wait until it is ready.
    if (m_fence->GetCompletedValue() < m_fenceValues[m_frameIndex]) {
        ThrowIfFailed(m_fence->SetEventOnCompletion(m_fenceValues[m_frameIndex], m_fenceEvent));
        WaitForSingleObject(m_fenceEvent, INFINITE);
    }

    // Set the fence value for the next frame.
    m_fenceValues[m_frameIndex] = currentFenceValue + 1;
}

void D3D12MeshletRender::BuildBVH()
{
    ThrowIfFailed(m_commandAllocators[m_frameIndex]->Reset());
    ThrowIfFailed(m_commandList->Reset(m_commandAllocators[m_frameIndex].Get(), nullptr));

    // Build the DGF BLAS
    m_dgfBvhBuilder.Build(m_rtasDecodeBuffer.Get(),
                          0,
                          m_asScratchResource->GetGPUVirtualAddress(),
                          m_blas->GetGPUVirtualAddress(),
                          *m_dgfModel,
                          m_commandList.Get());

    const auto barrier = CD3DX12_RESOURCE_BARRIER::UAV(nullptr);
    m_commandList->ResourceBarrier(1, &barrier);

    // Build the TLAS
    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS tlasInputs = {};
    tlasInputs.DescsLayout                                          = D3D12_ELEMENTS_LAYOUT_ARRAY;
    tlasInputs.Flags         = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;
    tlasInputs.NumDescs      = 1;
    tlasInputs.Type          = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL;
    tlasInputs.InstanceDescs = m_instanceDescBuffer->GetGPUVirtualAddress();

    D3D12_RAYTRACING_INSTANCE_DESC instanceDesc;
    instanceDesc.InstanceContributionToHitGroupIndex = 0;
    instanceDesc.Flags                               = 0;
    instanceDesc.InstanceContributionToHitGroupIndex = 0;
    instanceDesc.InstanceMask                        = 0xff;
    instanceDesc.AccelerationStructure               = m_blas->GetGPUVirtualAddress();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            instanceDesc.Transform[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    void* pMappedInstanceDescs;
    ThrowIfFailed(m_instanceDescBuffer->Map(0, nullptr, &pMappedInstanceDescs));
    memcpy(pMappedInstanceDescs, &instanceDesc, sizeof(instanceDesc));
    m_instanceDescBuffer->Unmap(0, nullptr);

    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC buildDesc;
    buildDesc.DestAccelerationStructureData    = m_tlas->GetGPUVirtualAddress();
    buildDesc.ScratchAccelerationStructureData = m_asScratchResource->GetGPUVirtualAddress();
    buildDesc.SourceAccelerationStructureData  = 0;
    buildDesc.Inputs                           = tlasInputs;
    m_commandList->BuildRaytracingAccelerationStructure(&buildDesc, 0, nullptr);

    ThrowIfFailed(m_commandList->Close());

    ID3D12CommandList* ppCommandLists[] = {m_commandList.Get()};
    m_commandQueue->ExecuteCommandLists(1, ppCommandLists);

    WaitForGpu();
}