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

DGFBVHBuilder::DGFBVHBuilder() {}

void DGFBVHBuilder::Init(api::Device* device, ShaderCompiler& compiler, uint32_t flags)
{
    auto rootSig = {
        api::RootParameterType::BUFFER_SRV,
        api::RootParameterType::BUFFER_UAV,
        api::RootParameterType::BUFFER_UAV,
    };

    m_decodeShader = device->CreateComputePipeline(compiler,L"DGFBVHBuildDecode.hlsl", rootSig);

    m_device = device;
    m_flags = flags;
}

DGFBVHBuilder::PrebuildInfo DGFBVHBuilder::GetPrebuildInfo(const DGFMesh& mesh, bool allowNativeDGF) const
{
    api::RTASDesc desc;
    desc.Flags = m_flags;

    size_t decodeScratchSize = 0;
    if (allowNativeDGF && m_device->SupportsNativeDGF())
    {
        desc.Type = api::RTASType::DGF_BOTTOM_LEVEL;
        desc.DGFBLAS.blocks = mesh.GetDGFBlockBuffer();
        desc.DGFBLAS.blocksByteOffset = 0;
        desc.DGFBLAS.maxGeometryIndex   = mesh.GetMaxGeometryIndex();
        desc.DGFBLAS.maxPrimitiveIndex  = mesh.GetMaxPrimitiveIndex();
        desc.DGFBLAS.numBlocks          = (uint32_t)mesh.GetNumDGFBlocks();
        desc.DGFBLAS.numTriangles       = (uint32_t)mesh.GetNumBlockTriangles();
        desc.DGFBLAS.numVertices        = (uint32_t)mesh.GetNumBlockVertices();
    }
    else
    {
        desc.Type = api::RTASType::BOTTOM_LEVEL;
        desc.BLAS.IndexBuffer = 0;
        desc.BLAS.VertexBuffer = 0;
        desc.BLAS.NumIndices = (uint32_t)(3 * mesh.GetNumBlockTriangles());
        desc.BLAS.NumVertices = (uint32_t)mesh.GetNumBlockVertices();
        desc.BLAS.VertexStride = 12;
        decodeScratchSize = desc.BLAS.NumVertices * 12 + desc.BLAS.NumIndices * 4;
    }
    

    auto rtasPBI = m_device->GetRTASPrebuildInfo(desc);

    PrebuildInfo pbi;
    pbi.bvhSize             = static_cast<uint32_t>(rtasPBI.rtasSize);
    pbi.bvhBuildScratchSize = static_cast<uint32_t>(rtasPBI.buildScratchSize);
    pbi.decodeScratchSize   = static_cast<uint32_t>(decodeScratchSize);
    return pbi;
}

void DGFBVHBuilder::Build(api::Buffer*      decodeScratch,
                          size_t            decodeOffset,
                          api::Buffer*      buildScratch,
                          api::RTAS*        destination,
                          const DGFMesh&    mesh,
                          api::CommandList* cmdList, 
                          bool allowNativeDGF,
                          api::TimerHeap* timers, size_t timerOffset )
{
    if (allowNativeDGF && m_device->SupportsNativeDGF())
    {
        if (timers)
            cmdList->RecordTimer(timers, timerOffset + 2);

        api::RTASDesc desc;
        desc.Flags = m_flags;
        desc.Type = api::RTASType::DGF_BOTTOM_LEVEL;
        desc.DGFBLAS.blocks             = mesh.GetDGFBlockBuffer();
        desc.DGFBLAS.blocksByteOffset   = 0;
        desc.DGFBLAS.maxGeometryIndex   = mesh.GetMaxGeometryIndex();
        desc.DGFBLAS.maxPrimitiveIndex  = mesh.GetMaxPrimitiveIndex();
        desc.DGFBLAS.numBlocks          = (uint32_t)mesh.GetNumDGFBlocks();
        desc.DGFBLAS.numTriangles       = (uint32_t)mesh.GetNumBlockTriangles();
        desc.DGFBLAS.numVertices        = (uint32_t)mesh.GetNumBlockVertices();
        desc.Destination       = destination;
        desc.Scratch           = buildScratch;
        desc.ScratchByteOffset = 0;

        cmdList->BuildRTAS(desc);
        cmdList->UAVBarrier();

        if (timers)
            cmdList->RecordTimer(timers, timerOffset + 3);
    }
    else
    {
        if (timers)
            cmdList->RecordTimer(timers, timerOffset + 0);

        // run the decode shader first
        size_t decodeVBOffset = 0;
        size_t decodeIBOffset = 12 * mesh.GetNumBlockVertices();
        cmdList->SetPipelineState(m_decodeShader.Get());
        cmdList->SetComputeRootSRV(0, mesh.GetDGFBlockBuffer(), 0);
        cmdList->SetComputeRootUAV(1, decodeScratch, decodeVBOffset);
        cmdList->SetComputeRootUAV(2, decodeScratch, decodeIBOffset);
        cmdList->Dispatch((UINT)mesh.GetNumDGFBlocks(), 1, 1);

        cmdList->BufferBarrier(decodeScratch, api::BufferState::UAV, api::BufferState::SRV);

        if (timers)
        {
            cmdList->RecordTimer(timers, timerOffset + 1);
            cmdList->RecordTimer(timers, timerOffset + 2);
        }

        api::RTASDesc desc;
        desc.Flags = m_flags;
        desc.Type = api::RTASType::BOTTOM_LEVEL;
        desc.BLAS.IndexBuffer = decodeScratch;
        desc.BLAS.VertexBuffer = decodeScratch;
        desc.BLAS.IndexByteOffset = (uint32_t)decodeIBOffset;
        desc.BLAS.VertexByteOffset = (uint32_t)decodeVBOffset;
        desc.BLAS.NumIndices = (uint32_t)(3 * mesh.GetNumBlockTriangles());
        desc.BLAS.NumVertices = (uint32_t)mesh.GetNumBlockVertices();
        desc.BLAS.VertexStride = 12;
        desc.Destination = destination;
        desc.Scratch = buildScratch;
        desc.ScratchByteOffset = 0;

        cmdList->BuildRTAS(desc);

        cmdList->BufferBarrier(decodeScratch, api::BufferState::SRV, api::BufferState::UAV);
        if (timers)
            cmdList->RecordTimer(timers, timerOffset + 3);

    }
}