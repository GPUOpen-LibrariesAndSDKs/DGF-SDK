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


#include "DGFBVHBuilder.h"

DGFBVHBuilder::DGFBVHBuilder() : m_decodeShader(L"DGFBVHBuildDecode.hlsl") {}

void DGFBVHBuilder::Init(ID3D12Device5* device, dx::ShaderCompiler* compiler)
{
    m_decodeShader.Init(device, compiler);
    m_device = device;
}

DGFBVHBuilder::PrebuildInfo DGFBVHBuilder::GetPrebuildInfo(const DGFMesh& mesh) const
{
    D3D12_RAYTRACING_GEOMETRY_DESC geomDesc;
    geomDesc.Flags                                = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;
    geomDesc.Type                                 = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
    geomDesc.Triangles.Transform3x4               = 0;
    geomDesc.Triangles.IndexCount                 = 3 * static_cast<uint32_t>(mesh.GetNumBlockTriangles());
    geomDesc.Triangles.IndexFormat                = DXGI_FORMAT_R32_UINT;
    geomDesc.Triangles.VertexCount                = static_cast<uint32_t>(mesh.GetNumBlockVertices());
    geomDesc.Triangles.VertexFormat               = DXGI_FORMAT_R32G32B32_FLOAT;
    geomDesc.Triangles.VertexBuffer.StartAddress  = 0;
    geomDesc.Triangles.VertexBuffer.StrideInBytes = 12;

    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS blasInputs = {};
    blasInputs.DescsLayout                                          = D3D12_ELEMENTS_LAYOUT_ARRAY;
    blasInputs.Flags          = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;
    blasInputs.NumDescs       = 1;
    blasInputs.pGeometryDescs = &geomDesc;
    blasInputs.Type           = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;

    D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO blasPBI;
    m_device->GetRaytracingAccelerationStructurePrebuildInfo(&blasInputs, &blasPBI);

    PrebuildInfo pbi;
    pbi.bvhSize             = static_cast<uint32_t>(blasPBI.ResultDataMaxSizeInBytes);
    pbi.bvhBuildScratchSize = static_cast<uint32_t>(blasPBI.ScratchDataSizeInBytes);
    pbi.decodeScratchSize   = geomDesc.Triangles.VertexCount * 12 + geomDesc.Triangles.IndexCount * 4;
    return pbi;
}

void DGFBVHBuilder::Build(ID3D12Resource*             decodeScratch,
                          size_t                      decodeScratchOffset,
                          D3D12_GPU_VIRTUAL_ADDRESS   buildScratch,
                          D3D12_GPU_VIRTUAL_ADDRESS   destination,
                          const DGFMesh&              mesh,
                          ID3D12GraphicsCommandList4* cmdList)
{
    // run the decode shader first
    D3D12_GPU_VIRTUAL_ADDRESS decodeAddress   = decodeScratch->GetGPUVirtualAddress();
    D3D12_GPU_VIRTUAL_ADDRESS decodeVBAddress = decodeAddress;
    D3D12_GPU_VIRTUAL_ADDRESS decodeIBAddress = decodeAddress + 12 * mesh.GetNumBlockVertices();

    cmdList->SetPipelineState(m_decodeShader.GetPipelineState().Get());
    cmdList->SetComputeRootSignature(m_decodeShader.GetRootSignature().Get());
    cmdList->SetComputeRootShaderResourceView(0, mesh.GetDGFBlockBuffer());
    cmdList->SetComputeRootUnorderedAccessView(1, decodeVBAddress);
    cmdList->SetComputeRootUnorderedAccessView(2, decodeIBAddress);
    cmdList->Dispatch((UINT) mesh.GetNumDGFBlocks(), 1, 1);

    D3D12_RESOURCE_BARRIER barrier;
    barrier.Transition.pResource   = decodeScratch;
    barrier.Transition.Subresource = 0;
    barrier.Transition.StateAfter  = D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE;
    barrier.Transition.StateBefore = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
    barrier.Flags                  = D3D12_RESOURCE_BARRIER_FLAG_NONE;
    barrier.Type                   = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
    cmdList->ResourceBarrier(1, &barrier);

    // now do the build
    D3D12_RAYTRACING_GEOMETRY_DESC geomDesc;
    geomDesc.Flags                                = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;
    geomDesc.Type                                 = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;
    geomDesc.Triangles.Transform3x4               = 0;
    geomDesc.Triangles.IndexCount                 = static_cast<uint32_t>(3 * mesh.GetNumBlockTriangles());
    geomDesc.Triangles.IndexFormat                = DXGI_FORMAT_R32_UINT;
    geomDesc.Triangles.VertexCount                = static_cast<uint32_t>(mesh.GetNumBlockVertices());
    geomDesc.Triangles.VertexFormat               = DXGI_FORMAT_R32G32B32_FLOAT;
    geomDesc.Triangles.VertexBuffer.StartAddress  = 0;
    geomDesc.Triangles.VertexBuffer.StrideInBytes = 12;
    geomDesc.Triangles.IndexBuffer                = decodeIBAddress;
    geomDesc.Triangles.VertexBuffer.StartAddress  = decodeVBAddress;

    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS blasInputs = {};
    blasInputs.DescsLayout                                          = D3D12_ELEMENTS_LAYOUT_ARRAY;
    blasInputs.Flags          = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;
    blasInputs.NumDescs       = 1;
    blasInputs.pGeometryDescs = &geomDesc;
    blasInputs.Type           = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL;

    D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC buildDesc;
    buildDesc.DestAccelerationStructureData    = destination;
    buildDesc.ScratchAccelerationStructureData = buildScratch;
    buildDesc.SourceAccelerationStructureData  = 0;
    buildDesc.Inputs                           = blasInputs;
    cmdList->BuildRaytracingAccelerationStructure(&buildDesc, 0, nullptr);
}