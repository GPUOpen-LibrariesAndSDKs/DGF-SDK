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
//

#include "DGFAnimationSample.h"

void DGFAnimationSample::RenderRaytraced(api::CommandList* cmdList, api::Texture* backBuffer)
{
    cmdList->SetPipelineState(m_raytracingComputeShader.Get());

    XMMATRIX view    = m_camera.GetViewMatrix();
    XMMATRIX proj    = m_camera.GetProjectionMatrix(XM_PI / 3.0f, m_aspectRatio, 0.01f, 10000.0f);
    float4x4 model   = m_dgfMesh->GetNormalizationTransformation();
    XMMATRIX xmmodel = XMMATRIX(reinterpret_cast<float*>(&model));

    XMMATRIX viewProj    = xmmodel * view * proj;
    XMMATRIX viewProjInv = XMMatrixInverse(nullptr, viewProj);

    RTConstants consts;
    consts.viewProjInverse = viewProjInv;
    consts.displayMode     = (uint)m_uiData.displayMode;
    consts.normalMatrix    = XMMatrixTranspose(XMMatrixInverse(nullptr, xmmodel));

    m_device->WriteDynamicBuffer(m_rtConstantsBuffer.Get(), &consts, sizeof(consts));

    cmdList->SetComputeRootUAV(0, m_raytracingBackBuffer.Get());
    cmdList->SetComputeRootCBV(1, m_rtConstantsBuffer.Get(), 0);
    cmdList->SetComputeRootSRV(2, m_tlas.Get());
    cmdList->SetComputeRootSRV(3, m_dgfMesh->GetIDToBlockMapVirtualAddress(),0);
    cmdList->SetComputeRootSRV(4, GetCurrentKeyframeNormals(), 0);

    const size_t vertexBufferSize = sizeof(float) * m_meshes.front().vertices.size();
    if (m_uiData.animationConfig.linearInterpolation) {
        cmdList->SetComputeRootSRV(5, m_interpolatedVerticesBuffer.Get(), vertexBufferSize);
    } else {
        cmdList->SetComputeRootSRV(5, GetCurrentKeyframeVertices(), vertexBufferSize);
    }

    cmdList->SetComputeRootSRV(6, m_dgfMesh->GetDGFBlockBuffer(),0);
    cmdList->SetComputeRootSRV(7, m_dgfMesh->GetVertexTableBuffer(),0);

    uint XGroups = ((UINT)m_viewport.Width + 7) / 8;
    uint YGroups = ((UINT)m_viewport.Height + 7) / 8;
    cmdList->RecordTimer(m_timerHeap.Get(), 10);
    cmdList->Dispatch(XGroups, YGroups, 1);
    cmdList->TextureBarrier(m_raytracingBackBuffer.Get(), api::TextureState::UAV, api::TextureState::COPY_SOURCE);
    cmdList->RecordTimer(m_timerHeap.Get(), 11);
    
    cmdList->TextureBarrier(backBuffer, api::TextureState::PRESENT, api::TextureState::COPY_DEST);
    cmdList->CopyTexture(backBuffer, m_raytracingBackBuffer.Get());
    cmdList->TextureBarrier(m_raytracingBackBuffer.Get(), api::TextureState::COPY_SOURCE, api::TextureState::UAV);
    cmdList->TextureBarrier(backBuffer, api::TextureState::COPY_DEST, api::TextureState::RENDER);

    cmdList->ResolveTimerHeap(m_timerHeap.Get());

    cmdList->BeginRenderPass(m_viewport, m_scissorRect, backBuffer, m_zbuffer.Get(), api::CLEAR_DEPTH, nullptr, 1.0f);
}

void DGFAnimationSample::BuildBVH(api::CommandList* cmdList)
{    
    // build the blas
    m_dgfBvhBuilder.Build(m_rtasDecodeBuffer.Get(), 0, m_asScratchResource.Get(), 0, m_blas.Get(), m_dgfMesh->GetDGFBLASDesc(), cmdList, m_uiData.useNativeDGF, m_timerHeap.Get(), 4);
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

    cmdList->RecordTimer(m_timerHeap.Get(), 8);
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
    cmdList->UAVBarrier();
    cmdList->RecordTimer(m_timerHeap.Get(), 9);
}

void DGFAnimationSample::AllocateBVHResources()
{
    DGFBVHBuilder::PrebuildInfo dgfPBI = m_dgfBvhBuilder.GetPrebuildInfo(m_dgfMesh->GetDGFBLASDesc(), m_uiData.useNativeDGF);

    api::RTASDesc tlasDesc;
    tlasDesc.Type                = api::RTASType::TOP_LEVEL;
    tlasDesc.TLAS.InstanceBuffer = 0;
    tlasDesc.TLAS.NumInstances   = 1;
    tlasDesc.Flags               = 0;

    api::RTASPrebuildInfo tlasPBI = m_device->GetRTASPrebuildInfo(tlasDesc);

    size_t scratchResourceSize = std::max(dgfPBI.bvhBuildScratchSize, (UINT)tlasPBI.buildScratchSize);

    m_blasOffset = 0;
    m_tlasOffset = RoundUpPow2(dgfPBI.bvhSize, api::RTAS_ALIGNMENT);    
    size_t totalRTASSize = m_tlasOffset + tlasPBI.rtasSize;
    
    m_rtasBuffer = m_device->CreateRTASBuffer(totalRTASSize);
    m_blas = m_device->CreateRTAS(m_rtasBuffer.Get(), m_blasOffset, dgfPBI.bvhSize);
    m_tlas = m_device->CreateRTAS(m_rtasBuffer.Get(), m_tlasOffset, tlasPBI.rtasSize);

    m_asScratchResource  = m_device->CreateUAVBuffer(scratchResourceSize);
    m_rtasDecodeBuffer   = m_device->CreateUAVBuffer(dgfPBI.decodeScratchSize);
    m_instanceDescBuffer = m_device->CreateDynamicBuffer(sizeof(api::InstanceDesc));
}
