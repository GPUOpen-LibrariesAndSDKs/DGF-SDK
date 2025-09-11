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

#define NOMINMAX

#include <Windows.h>
#include "vk.h"

#define VOLK_IMPLEMENTATION
#include "volk.h"

#include "imgui.h"
#include "imgui_impl_win32.h"
#define IMGUI_IMPL_VULKAN_USE_VOLK
#include "imgui_impl_vulkan.h"

#define VMA_IMPLEMENTATION
#include "vk_mem_alloc.h"

#include "Util.h"

#include "vk_amdx_dense_geometry_format.h"

namespace vk
{
    constexpr size_t MAX_DESCRIPTORS = 63356;


    static VkDeviceAddress GetBufferAddress(api::Buffer* buffer, size_t offset)
    {
        return static_cast<vk::Buffer*>(buffer)->GetGPUVirtualAddress() + offset;
    }

    static VkCommandBuffer GetCommandList(api::CommandList* cmdList)
    {
        return static_cast<vk::CommandList*>(cmdList)->Get();
    }

    static VkBuildAccelerationStructureFlagsKHR TranslateRTASFlags(uint32_t Flags)
    {
        VkBuildAccelerationStructureFlagsKHR flagsOut = 0;
        if (Flags & api::RTAS_PREFER_FAST_BUILD)
            flagsOut |= VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR;
        if (Flags & api::RTAS_PREFER_FAST_TRACE)
            flagsOut |= VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;

        return flagsOut;
    }

    DynamicBuffer::DynamicBuffer(VkDevice device, VmaAllocator allocator, VMABuffer renameChain[api::FRAME_COUNT])
       : Buffer(renameChain[0].buffer, device, allocator, renameChain[0].allocation)
    {
        for (size_t i = 0; i < api::FRAME_COUNT; i++)
            m_renameChain[i] = renameChain[i];        
    }

    void DynamicBuffer::Write(const void* contents, size_t size, size_t frameIndex)
    {
        void* pData;
        vmaMapMemory(GetAllocator(), m_renameChain[frameIndex].allocation, &pData);
        memcpy(pData, contents, size);
        vmaUnmapMemory(GetAllocator(), m_renameChain[frameIndex].allocation);

        // NOTE: flush/invalidate should not be necessary due to use of host-coherent memory

        // redirect the base buffer pointer at the current buffer
        //   This ensures that the most recent buffer is used whenever the buffer is bound on a command list
        m_buffer = m_renameChain[frameIndex].buffer;
        m_allocation = m_renameChain[frameIndex].allocation;
    }

    void CommandList::BeginRenderPass(api::Viewport vp, api::ScissorRect rect,
        api::Texture* backBuffer, api::Texture* zBuffer,
        uint32_t clearFlags, const float clearColor[4], float clearDepth )
    {
        bool shouldClearDepth = clearFlags & api::CLEAR_DEPTH;
        bool shouldClearColor = clearFlags & api::CLEAR_COLOR;
    
        vk::RenderTargetTexture* rt = static_cast<vk::RenderTargetTexture*>(backBuffer);
        VkRenderingAttachmentInfo rai = { VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        rai.imageView = rt->GetImageView();
        rai.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
        rai.loadOp = shouldClearColor ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD;
        rai.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        if (shouldClearColor)
        {
            for (size_t i = 0; i < 4; i++)
                rai.clearValue.color.float32[i] = clearColor[i];
        }

        vk::DepthStencilTexture* ds = static_cast<vk::DepthStencilTexture*>(zBuffer);
        VkRenderingAttachmentInfo depth = { VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO };
        depth.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
        depth.imageView = ds->GetImageView();
        depth.loadOp = shouldClearDepth ? VK_ATTACHMENT_LOAD_OP_CLEAR : VK_ATTACHMENT_LOAD_OP_LOAD;
        depth.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        depth.clearValue.depthStencil.depth = clearDepth;
        depth.clearValue.depthStencil.stencil = 0;

        VkRenderingInfo ri = { VK_STRUCTURE_TYPE_RENDERING_INFO };
        VkExtent3D size = rt->GetSize();
        ri.renderArea.extent = { size.width, size.height };
        ri.layerCount = 1;
        ri.colorAttachmentCount = 1;
        ri.pColorAttachments = &rai;
        ri.pDepthAttachment = &depth;
    
        VkViewport viewport = { vp.TopLeftX, vp.TopLeftY + vp.Height, vp.Width, -vp.Height, vp.MinDepth, vp.MaxDepth };
        VkRect2D scissor;
        scissor.offset = { static_cast<int32_t>(rect.left), static_cast<int32_t>(rect.top) };
        scissor.extent = { (rect.right - rect.left) , (rect.bottom - rect.top) };

        vkCmdSetViewport(Get(), 0, 1, &viewport);
        vkCmdSetScissor(Get(), 0, 1, &scissor);
        vkCmdBeginRendering(Get(), &ri);
    }
    void CommandList::EndRenderPass()
    {
        vkCmdEndRendering(Get());
    }

    void CommandList::SetPipelineState(api::RenderingPipeline* pso)
    {
        vk::RenderingPipeline* vkpso = static_cast<vk::RenderingPipeline*>(pso);

        vkCmdBindPipeline(Get(), VK_PIPELINE_BIND_POINT_GRAPHICS, vkpso->m_pipeline);
        
        m_currentSetLayout = vkpso->m_descriptor_set_layout;
        m_currentDescriptorSet = vkpso->m_descriptor_set;
        m_currentDescriptorCount = vkpso->m_descriptor_count;
        m_currentPipelineLayout = vkpso->m_layout;
    }

    void CommandList::SetGraphicsRootSRV(size_t paramIndex, api::Buffer* buffer, size_t offset)
    {        
        vk::Buffer* dest = static_cast<vk::Buffer*>(buffer);

        VkDescriptorBufferInfo dbi;
        dbi.buffer = dest->GetResource();
        dbi.offset = offset;
        dbi.range = VK_WHOLE_SIZE;

        VkWriteDescriptorSet wds = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        wds.dstSet = m_currentDescriptorSet;
        wds.dstBinding = (UINT)paramIndex;
        wds.descriptorCount = 1;
        wds.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        wds.pBufferInfo = &dbi;

        vkUpdateDescriptorSets(m_device, 1, &wds, 0, nullptr);
    }

    void CommandList::SetGraphicsRootCBV(size_t paramIndex, api::Buffer* buffer, size_t offset)
    {
        vk::Buffer* dest = static_cast<vk::Buffer*>(buffer);

        VkDescriptorBufferInfo dbi;
        dbi.buffer = dest->GetResource();
        dbi.offset = offset;
        dbi.range = VK_WHOLE_SIZE;

        VkWriteDescriptorSet wds = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        wds.dstSet = m_currentDescriptorSet;
        wds.dstBinding = (UINT)paramIndex;
        wds.descriptorCount = 1;
        wds.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        wds.pBufferInfo = &dbi;

        vkUpdateDescriptorSets(m_device, 1, &wds, 0, nullptr);
    }

    void CommandList::DispatchMesh(uint32_t x, uint32_t y, uint32_t z)
    {
        VkDescriptorSet set = m_frameworkDevice->AllocateDispatchDescriptors(m_currentDescriptorCount, m_currentSetLayout);

        std::vector<VkCopyDescriptorSet> bindings(m_currentDescriptorCount);
        for (size_t i = 0; i < m_currentDescriptorCount; i++)
        {
            VkCopyDescriptorSet copy = { VK_STRUCTURE_TYPE_COPY_DESCRIPTOR_SET };
            copy.dstArrayElement = 0;
            copy.dstBinding = (uint32_t)i;
            copy.srcArrayElement = 0;
            copy.srcBinding = (uint32_t)i;
            copy.srcSet = m_currentDescriptorSet;
            copy.descriptorCount = 1;
            copy.dstSet = set;

            bindings[i] = copy;
        }
       
        vkUpdateDescriptorSets(m_device, 0, nullptr, (uint32_t)bindings.size(), bindings.data());

        vkCmdBindDescriptorSets(Get(), VK_PIPELINE_BIND_POINT_GRAPHICS, m_currentPipelineLayout, 0, 1, &set, 0, nullptr);
        vkCmdDrawMeshTasksEXT(Get(), x, y, z);
    }

    void CommandList::SetPipelineState(api::ComputePipeline* pso)
    {
        vk::ComputePipeline* vkpso = static_cast<vk::ComputePipeline*>(pso);

        vkCmdBindPipeline(Get(), VK_PIPELINE_BIND_POINT_COMPUTE, vkpso->m_pipeline);
        
        m_currentDescriptorSet   = vkpso->m_descriptor_set;
        m_currentDescriptorCount = vkpso->m_descriptor_count;
        m_currentPipelineLayout  = vkpso->m_layout;
        m_currentSetLayout       = vkpso->m_descriptor_set_layout;
    }

    void CommandList::SetComputeRootCBV(size_t paramIndex, api::Buffer* buffer, size_t offset)
    {
        vk::Buffer* dest = static_cast<vk::Buffer*>(buffer);

        VkDescriptorBufferInfo dbi;
        dbi.buffer = dest->GetResource();
        dbi.offset = offset;
        dbi.range = VK_WHOLE_SIZE;

        VkWriteDescriptorSet wds = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        wds.dstSet = m_currentDescriptorSet;
        wds.dstBinding = (UINT)paramIndex;
        wds.descriptorCount = 1;
        wds.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        wds.pBufferInfo = &dbi;

        vkUpdateDescriptorSets(m_device, 1, &wds, 0, nullptr);
    }

    void CommandList::SetComputeRootSRV(size_t paramIndex, api::RTAS* tlas)
    {
        vk::RTAS* dest = static_cast<vk::RTAS*>(tlas);

        const VkAccelerationStructureKHR as = dest->GetASHandle();

        VkWriteDescriptorSetAccelerationStructureKHR wdsas = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR};
        wdsas.accelerationStructureCount = 1;
        wdsas.pAccelerationStructures = &as;

        VkWriteDescriptorSet wds = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        wds.pNext = &wdsas;
        wds.dstSet = m_currentDescriptorSet;
        wds.dstBinding = (UINT)paramIndex;
        wds.descriptorCount = 1;
        wds.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;

        vkUpdateDescriptorSets(m_device, 1, &wds, 0, nullptr);
    }

    void CommandList::SetComputeRootSRV(size_t paramIndex, api::Buffer* buffer, size_t offset )
    {
        vk::Buffer* dest = static_cast<vk::Buffer*>(buffer);

        VkDescriptorBufferInfo dbi;
        dbi.buffer = dest->GetResource();
        dbi.offset = offset;
        dbi.range = VK_WHOLE_SIZE;

        VkWriteDescriptorSet wds = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        wds.dstSet = m_currentDescriptorSet;
        wds.dstBinding = (UINT)paramIndex;
        wds.descriptorCount = 1;
        wds.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        wds.pBufferInfo = &dbi;

        vkUpdateDescriptorSets(m_device, 1, &wds, 0, nullptr);
    }

    void CommandList::SetComputeRootUAV(size_t paramIndex, api::Buffer* buffer, size_t offset )
    {
        vk::Buffer* dest = static_cast<vk::Buffer*>(buffer);

        VkDescriptorBufferInfo dbi;
        dbi.buffer = dest->GetResource();
        dbi.offset = offset;
        dbi.range = VK_WHOLE_SIZE;

        VkWriteDescriptorSet wds = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        wds.dstSet = m_currentDescriptorSet;
        wds.dstBinding = (UINT)paramIndex;
        wds.descriptorCount = 1;
        wds.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        wds.pBufferInfo = &dbi;

        vkUpdateDescriptorSets(m_device, 1, &wds, 0, nullptr);
    }

    void CommandList::SetComputeRootUAV(size_t paramIndex, api::Texture* uav)
    {
        vk::Texture* dest = static_cast<vk::Texture*>(uav);

        VkDescriptorImageInfo dii;
        dii.imageView = dest->GetImageView();
        dii.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

        VkWriteDescriptorSet wds = { VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
        wds.dstSet = m_currentDescriptorSet;
        wds.dstBinding = (UINT)paramIndex;
        wds.descriptorCount = 1;
        wds.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        wds.pImageInfo = &dii;

        vkUpdateDescriptorSets(m_device, 1, &wds, 0, nullptr);
    }

    void CommandList::Dispatch(uint32_t X, uint32_t Y, uint32_t Z)
    {
        VkDescriptorSet set = m_frameworkDevice->AllocateDispatchDescriptors(m_currentDescriptorCount,m_currentSetLayout);

        std::vector<VkCopyDescriptorSet> bindings(m_currentDescriptorCount);
        for (size_t i = 0; i < m_currentDescriptorCount; i++)
        {
            VkCopyDescriptorSet copy = { VK_STRUCTURE_TYPE_COPY_DESCRIPTOR_SET };
            copy.dstArrayElement = 0;
            copy.dstBinding = (uint32_t)i;
            copy.srcArrayElement = 0;
            copy.srcBinding = (uint32_t)i;
            copy.srcSet = m_currentDescriptorSet;
            copy.descriptorCount = 1;
            copy.dstSet = set;

            bindings[i] = copy;
        }

        vkUpdateDescriptorSets(m_device, 0, nullptr, (uint32_t) bindings.size(), bindings.data());

        vkCmdBindDescriptorSets(Get(), VK_PIPELINE_BIND_POINT_COMPUTE, m_currentPipelineLayout, 0, 1, &set, 0, nullptr);

        vkCmdDispatch(Get(), X, Y, Z);
    }

    void CommandList::BuildRTAS(const api::RTASDesc& desc)
    {
        VkAccelerationStructureDenseGeometryFormatTrianglesDataAMDX ASDGFFTD = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DENSE_GEOMETRY_FORMAT_TRIANGLES_DATA_AMDX };
        VkAccelerationStructureBuildGeometryInfoKHR ASBGI = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
        VkAccelerationStructureGeometryKHR ASG = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
        ASG.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;

        ASBGI.flags = TranslateRTASFlags(desc.Flags);
        ASBGI.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
        ASBGI.geometryCount = 1;
        ASBGI.pGeometries = &ASG;
        ASBGI.scratchData.deviceAddress = GetBufferAddress(desc.Scratch, desc.ScratchByteOffset);
        ASBGI.dstAccelerationStructure = static_cast<vk::RTAS*>(desc.Destination)->GetASHandle();

        VkAccelerationStructureBuildRangeInfoKHR ASBRI = { 0 };
        std::vector<VkAccelerationStructureBuildRangeInfoKHR*> rangeInfos = { &ASBRI };

        switch (desc.Type)
        {
            case api::RTASType::TOP_LEVEL:
            {
                ASBGI.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;

                ASG.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;

                ASG.geometry.instances = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR };
                ASG.geometry.instances.arrayOfPointers = VK_FALSE;
                ASG.geometry.instances.data.deviceAddress = GetBufferAddress(desc.TLAS.InstanceBuffer, desc.TLAS.InstancesByteOffset);

                ASBRI.primitiveCount = 1;

                break;
            }
            case api::RTASType::BOTTOM_LEVEL:
            {
                ASBGI.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;

                ASG.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;

                ASG.geometry.triangles = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR };
                ASG.geometry.triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
                ASG.geometry.triangles.vertexData.deviceAddress = GetBufferAddress(desc.BLAS.VertexBuffer, desc.BLAS.VertexByteOffset);
                ASG.geometry.triangles.vertexStride = desc.BLAS.VertexStride;
                ASG.geometry.triangles.maxVertex = desc.BLAS.NumVertices - 1;
                ASG.geometry.triangles.indexType = VK_INDEX_TYPE_UINT32;
                ASG.geometry.triangles.indexData.deviceAddress = GetBufferAddress(desc.BLAS.IndexBuffer, desc.BLAS.IndexByteOffset);;

                ASBRI.primitiveCount = desc.BLAS.NumIndices / 3;

                break;
            }
            case api::RTASType::DGF_BOTTOM_LEVEL:
            {
                ASBGI.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;

                ASG.geometryType = VK_GEOMETRY_TYPE_DENSE_GEOMETRY_FORMAT_TRIANGLES_AMDX;
                ASG.pNext = &ASDGFFTD;

                ASDGFFTD.compressedData.deviceAddress = GetBufferAddress(desc.DGFBLAS.blocks, desc.DGFBLAS.blocksByteOffset);
                ASDGFFTD.dataSize = desc.DGFBLAS.numBlocks * VK_COMPRESSED_TRIANGLE_FORMAT_DGF1_BYTE_STRIDE_AMDX;
                ASDGFFTD.numTriangles = desc.DGFBLAS.numTriangles;
                ASDGFFTD.numVertices = desc.DGFBLAS.numVertices;
                ASDGFFTD.maxPrimitiveIndex = desc.DGFBLAS.maxPrimitiveIndex;
                ASDGFFTD.maxGeometryIndex = desc.DGFBLAS.maxGeometryIndex;
                ASDGFFTD.format = VK_COMPRESSED_TRIANGLE_FORMAT_DGF1_AMDX;

                ASBRI.primitiveCount = 0;

                break;
            }
            default:
            {
                assert(0);
                break;
            }
        }

        vkCmdBuildAccelerationStructuresKHR(Get(), 1, &ASBGI, rangeInfos.data());
    }

    constexpr uint32_t BUFFER_READ_FLAGS = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR |
        VK_ACCESS_SHADER_READ_BIT;
    constexpr uint32_t BUFFER_WRITE_FLAGS = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR |
        VK_ACCESS_SHADER_WRITE_BIT;

    void CommandList::UAVBarrier()
    {
        VkMemoryBarrier barrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER };
        barrier.srcAccessMask = BUFFER_READ_FLAGS | BUFFER_WRITE_FLAGS;
        barrier.dstAccessMask = BUFFER_READ_FLAGS | BUFFER_WRITE_FLAGS;
        
        VkPipelineStageFlags stageMask =
            VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT |
            VK_PIPELINE_STAGE_MESH_SHADER_BIT_EXT |
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
            VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;

        vkCmdPipelineBarrier(Get(), stageMask, stageMask, 0, 1, &barrier, 0, nullptr, 0, nullptr);
    }

    void CommandList::BufferBarrier(api::Buffer* buffer, api::BufferState from, api::BufferState to)
    {
        vk::Buffer* vkBuffer = static_cast<vk::Buffer*>(buffer);
        VkBufferMemoryBarrier barrier = { VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER };
        barrier.buffer = vkBuffer->GetResource();
        barrier.size = VK_WHOLE_SIZE;
      
        auto readFlags = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_KHR |
            VK_ACCESS_SHADER_READ_BIT;
        auto writeFlags = VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_KHR |
            VK_ACCESS_SHADER_WRITE_BIT ;

        if (from == api::BufferState::UAV && to == api::BufferState::SRV)
        {
            barrier.srcAccessMask = BUFFER_READ_FLAGS | BUFFER_WRITE_FLAGS;
            barrier.dstAccessMask = BUFFER_READ_FLAGS;
        }
        else if( from == api::BufferState::SRV && to == api::BufferState::UAV )
        {
            barrier.srcAccessMask = BUFFER_READ_FLAGS;
            barrier.dstAccessMask = BUFFER_READ_FLAGS | BUFFER_WRITE_FLAGS;
        }

        VkPipelineStageFlags stageMask =
            VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT |
            VK_PIPELINE_STAGE_MESH_SHADER_BIT_EXT |
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
            VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;

        vkCmdPipelineBarrier(Get(), stageMask, stageMask, 0, 0, nullptr, 1, &barrier, 0, nullptr);        
    }

    static VkImageLayout TranslateTextureState(api::TextureState state)
    {
        switch (state)
        {
        case api::TextureState::COPY_DEST:
            return VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        case api::TextureState::COPY_SOURCE:
            return VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        case api::TextureState::PRESENT:
            return VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        case api::TextureState::RENDER:
            return VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        case api::TextureState::UAV:
            return VK_IMAGE_LAYOUT_GENERAL;
        default:
            assert(0);
            return VK_IMAGE_LAYOUT_GENERAL;
        }
    }

    void CommandList::TextureBarrier(api::Texture* texture, api::TextureState from, api::TextureState to)
    {
        VkPipelineStageFlags srcStageMask, dstStageMask;

        VkImageMemoryBarrier imb = { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
        imb.oldLayout = TranslateTextureState(from);
        imb.newLayout = TranslateTextureState(to);
        imb.image = static_cast<vk::Texture*>(texture)->GetResource();
        imb.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        imb.subresourceRange.levelCount = 1;
        imb.subresourceRange.layerCount = 1;

        if (from == api::TextureState::RENDER && to == api::TextureState::PRESENT)
        {
            imb.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
            imb.dstAccessMask = 0;

            srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
            dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
        }
        else if (from == api::TextureState::PRESENT && to == api::TextureState::RENDER)
        {
            imb.srcAccessMask = 0;
            imb.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

            srcStageMask = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        }
        else if (from == api::TextureState::UAV && to == api::TextureState::COPY_SOURCE)
        {
            imb.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            imb.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
            dstStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
            srcStageMask = VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT | 
                           VK_PIPELINE_STAGE_MESH_SHADER_BIT_EXT | 
                           VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT;
        }
        else if (from == api::TextureState::COPY_DEST && to == api::TextureState::PRESENT)
        {
            imb.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
            imb.dstAccessMask = 0;
            srcStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
            dstStageMask = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
        }
        else if (from == api::TextureState::PRESENT && to == api::TextureState::COPY_DEST)
        {
            imb.srcAccessMask = 0;
            imb.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            srcStageMask = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            dstStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;;
        }
        else if (from == api::TextureState::COPY_SOURCE && to == api::TextureState::UAV)
        {
            imb.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
            imb.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            srcStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
            dstStageMask = VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT | VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
                VK_PIPELINE_STAGE_MESH_SHADER_BIT_EXT ;
        }
        else if (from == api::TextureState::COPY_DEST && to == api::TextureState::RENDER)
        {
            imb.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            imb.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
            srcStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
            dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        }
        else
        {
            assert(0);
        }

        vkCmdPipelineBarrier(Get(), srcStageMask, dstStageMask, 0, 0, nullptr, 0, nullptr, 1, &imb);
    }

    void CommandList::CopyTexture(api::Texture* dst, api::Texture* src)
    {
        VkOffset3D offset = { 0,0,0 };
        VkImageSubresourceLayers srl = {};
        srl.aspectMask = static_cast<vk::Texture*>(src)->GetAspectMask();
        srl.layerCount = 1;
        
        VkExtent3D extent = static_cast<vk::Texture*>(src)->GetSize();
        VkImageCopy ic = {}; 
        ic.extent = extent;
        ic.srcOffset = offset;
        ic.dstOffset = offset;
        ic.srcSubresource = srl;
        ic.dstSubresource = srl;

        vkCmdCopyImage(
            Get(),
            static_cast<vk::Texture*>(src)->GetResource(),
            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            static_cast<vk::Texture*>(dst)->GetResource(),
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            1,
            &ic);
    }

    void CommandList::InitializeBuffer(api::Buffer* buffer, const void* contents, size_t size)
    {
        VkBufferCreateInfo bufCreateInfo = { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bufCreateInfo.size = size;
        bufCreateInfo.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;

        VmaAllocationCreateInfo allocCreateInfo = {};
        allocCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocCreateInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;

        VkBuffer buf;
        VmaAllocation alloc;
        VmaAllocationInfo allocInfo;
        vmaCreateBuffer(m_vmaAllocator, &bufCreateInfo, &allocCreateInfo, &buf, &alloc, &allocInfo);

        void* pData;
        vmaMapMemory(m_vmaAllocator, alloc, &pData);
        memcpy(pData, contents, size);
        vmaUnmapMemory(m_vmaAllocator, alloc);

        VkBufferCopy2 bc2 = { VK_STRUCTURE_TYPE_BUFFER_COPY_2 };

        vk::Buffer* dest = static_cast<vk::Buffer*>(buffer);

        VkCopyBufferInfo2 cbi2 = { VK_STRUCTURE_TYPE_COPY_BUFFER_INFO_2 };
        cbi2.srcBuffer = buf;
        cbi2.dstBuffer = dest->GetResource();
        cbi2.regionCount = 1;
        cbi2.pRegions = &bc2;

        bc2.size = size;

        vkCmdCopyBuffer2(Get(), &cbi2);

        VMABuffer vmab = { .buffer = buf, .allocation = alloc };
        // keep the upload buffer alive until this command list retires
        m_uploadBuffers.push_back(vmab);
    }


    void CommandList::ResolveTimerHeap(api::TimerHeap* heap)
    {
        vk::TimerHeap* vkheap = static_cast<vk::TimerHeap*>(heap);
        vkCmdCopyQueryPoolResults(Get(), vkheap->m_queryPool, 0, (uint32_t)vkheap->m_numTimers, vkheap->m_readbackBuffer, 0, sizeof(uint64_t), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT);
        vkCmdResetQueryPool(Get(), vkheap->m_queryPool, 0, (uint32_t)vkheap->m_numTimers);
    }

    void CommandList::RecordTimer(api::TimerHeap* heap, size_t timerIndex)
    {
        vk::TimerHeap* vkheap = static_cast<vk::TimerHeap*>(heap);
        vkCmdWriteTimestamp(Get(), VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, vkheap->m_queryPool, (uint32_t)timerIndex);
    }

    void TimerHeap::ReadTimers(double* time, size_t start, size_t count)
    {
        if (start >= m_numTimers || (start + count) > m_numTimers)
            THROW_RUNTIME_ERROR("Out of range query read");

        void* pData;
        vmaMapMemory(m_allocator, m_readbackAllocation, &pData);
        uint64_t* pTimestamps = reinterpret_cast<uint64_t *>(pData);
        for (size_t i = 0; i < count; i++)
            time[i] = pTimestamps[start + i] * m_gpuTickDuration;
        vmaUnmapMemory(m_allocator, m_readbackAllocation);
    }


    Texture::~Texture()
    {
    }

    // layout transition from VK_IMAGE_LAYOUT_UNDEFINED
    void TransitionImageFromUndefined(Device* device, VkDevice vkDevice, VkQueue queue, VkCommandPool commandPool, VkImage image, VkImageLayout layout, VkImageAspectFlags aspect)
    {
        VkCommandBuffer cb;
        VkCommandBufferAllocateInfo CBAI = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
        CBAI.commandPool = commandPool;
        CBAI.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        CBAI.commandBufferCount = 1;

        CHECK_IF_FAILED(vkAllocateCommandBuffers(vkDevice, &CBAI, &cb));

        VkCommandBufferBeginInfo cbbi = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
        cbbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vkBeginCommandBuffer(cb, &cbbi);

        VkImageMemoryBarrier imb = { VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
        imb.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        imb.newLayout = layout;
        imb.image = image;
        imb.subresourceRange.aspectMask = aspect;
        imb.subresourceRange.levelCount = 1;
        imb.subresourceRange.layerCount = 1;

        vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, 1, &imb);

        vkEndCommandBuffer(cb);

        VkSubmitInfo si = { VK_STRUCTURE_TYPE_SUBMIT_INFO };
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cb;

        CHECK_IF_FAILED(vkQueueSubmit(queue, 1, &si, nullptr));

        device->WaitForGPUIdle();

        vkFreeCommandBuffers(vkDevice, commandPool, 1, &cb);
    }

    Device::Device(HWND hwnd, HINSTANCE hInstance, uint32_t width, uint32_t height )
    {
        CHECK_IF_FAILED(volkInitialize());

        // create instance
        VkInstance instance;
        {
            VkApplicationInfo AI = { VK_STRUCTURE_TYPE_APPLICATION_INFO };
            AI.pApplicationName = "Vulkan DGF Sample";
            AI.apiVersion = VK_API_VERSION_1_3;

            const std::vector<const char*> instanceExtensions{ VK_KHR_SURFACE_EXTENSION_NAME, VK_KHR_WIN32_SURFACE_EXTENSION_NAME };

            VkInstanceCreateInfo ICI = { VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
            ICI.pApplicationInfo = &AI;
            ICI.enabledExtensionCount = static_cast<uint32_t>(instanceExtensions.size());
            ICI.ppEnabledExtensionNames = instanceExtensions.data();

            CHECK_IF_FAILED(vkCreateInstance(&ICI, nullptr, &instance));

            volkLoadInstance(instance);
        }
        
        // create device
        uint32_t queueFamilyIndex = 99;
        VkPhysicalDevice physicalDevice;
        {
            uint32_t count = 1;

            CHECK_IF_FAILED(vkEnumeratePhysicalDevices(instance, &count, &physicalDevice));

            vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &count, nullptr);

            std::vector<VkQueueFamilyProperties> QFP(count);

            vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &count, QFP.data());

            for (uint32_t i = 0; i < count; i++)
            {
                if ((QFP[i].queueFlags & VK_QUEUE_GRAPHICS_BIT))
                {
                    queueFamilyIndex = i;
                    break;
                }
            }

            assert(queueFamilyIndex != 99);

            // timestamp
            if (QFP[queueFamilyIndex].timestampValidBits != 0)
            {
                VkPhysicalDeviceProperties pdp;
                vkGetPhysicalDeviceProperties(physicalDevice, &pdp);

                m_timestampPeriod = pdp.limits.timestampPeriod;
            }
            else
            {
                THROW_RUNTIME_ERROR("Missing timestamp support");
            }

            // query extensions
            {
                uint32_t count;

                vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &count, nullptr);

                std::vector<VkExtensionProperties> EP(count);
                vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &count, EP.data());

                for (uint32_t i = 0; i < count; i++)
                {
                    if (strcmp(EP[i].extensionName, VK_AMDX_DENSE_GEOMETRY_FORMAT_EXTENSION_NAME) == 0)
                    {
                        m_nativeDGF = true;
                        break;
                    }
                }
            }

            // enable extensions
            std::vector<const char*> deviceExtensions = {};

            deviceExtensions.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
            deviceExtensions.push_back(VK_KHR_RAY_QUERY_EXTENSION_NAME);
            deviceExtensions.push_back(VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME);
            deviceExtensions.push_back(VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME);
            deviceExtensions.push_back(VK_EXT_MESH_SHADER_EXTENSION_NAME);
            deviceExtensions.push_back(VK_KHR_DYNAMIC_RENDERING_EXTENSION_NAME);
            deviceExtensions.push_back(VK_KHR_MAINTENANCE_5_EXTENSION_NAME);
            deviceExtensions.push_back(VK_EXT_SCALAR_BLOCK_LAYOUT_EXTENSION_NAME);
            deviceExtensions.push_back(VK_EXT_HOST_QUERY_RESET_EXTENSION_NAME);
            if (m_nativeDGF)
                deviceExtensions.push_back(VK_AMDX_DENSE_GEOMETRY_FORMAT_EXTENSION_NAME);

            VkDeviceQueueCreateInfo QCI = { VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
            float priority = 1.0;
            QCI.queueFamilyIndex = queueFamilyIndex;
            QCI.queueCount = 1;
            QCI.pQueuePriorities = &priority;

            VkPhysicalDeviceFeatures PDF = {};
            PDF.shaderInt64 = VK_TRUE;
        
            VkPhysicalDeviceDenseGeometryFormatFeaturesAMDX PDDGFF = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DENSE_GEOMETRY_FORMAT_FEATURES_AMDX };
            PDDGFF.denseGeometryFormat = VK_TRUE;

            VkPhysicalDeviceVulkan13Features PDV13F = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES };
            PDV13F.dynamicRendering = VK_TRUE;        
            if (m_nativeDGF)
                PDV13F.pNext = &PDDGFF;

            VkPhysicalDeviceMeshShaderFeaturesEXT PDMSF = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_MESH_SHADER_FEATURES_EXT };
            PDMSF.meshShader = VK_TRUE;
            PDMSF.pNext = &PDV13F;

            VkPhysicalDeviceAccelerationStructureFeaturesKHR PDASF = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR };
            PDASF.accelerationStructure = VK_TRUE;
            PDASF.pNext = &PDMSF;

            VkPhysicalDeviceRayQueryFeaturesKHR PDRQF = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR };
            PDRQF.rayQuery = VK_TRUE;
            PDRQF.pNext = &PDASF;

            VkPhysicalDeviceTimelineSemaphoreFeatures TLSF = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES };
            TLSF.pNext = &PDRQF;
            TLSF.timelineSemaphore = VK_TRUE;

            VkPhysicalDeviceBufferAddressFeaturesEXT BDAF = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES };
            BDAF.pNext = &TLSF;
            BDAF.bufferDeviceAddress = VK_TRUE;

            VkPhysicalDeviceScalarBlockLayoutFeaturesEXT PDSBLF = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SCALAR_BLOCK_LAYOUT_FEATURES_EXT };
            PDSBLF.pNext = &BDAF;
            PDSBLF.scalarBlockLayout = VK_TRUE;

            VkPhysicalDeviceHostQueryResetFeatures PDHQRF = { VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_HOST_QUERY_RESET_FEATURES_EXT };
            PDHQRF.pNext = &PDSBLF;
            PDHQRF.hostQueryReset = VK_TRUE;

            VkDeviceCreateInfo DCI = { VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
            DCI.pNext = &PDHQRF;
            DCI.queueCreateInfoCount = 1;
            DCI.pQueueCreateInfos = &QCI;
            DCI.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
            DCI.ppEnabledExtensionNames = deviceExtensions.data();
            DCI.pEnabledFeatures = &PDF;
            
            CHECK_IF_FAILED(vkCreateDevice(physicalDevice, &DCI, nullptr, &m_device));

            volkLoadDevice(m_device);

            vkGetDeviceQueue(m_device, queueFamilyIndex, 0, &m_queue);
        }

        // create surface
        VkSurfaceKHR surface;
        {
            VkWin32SurfaceCreateInfoKHR SCI = { VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR };
            SCI.hinstance = hInstance;
            SCI.hwnd = hwnd;

            CHECK_IF_FAILED(vkCreateWin32SurfaceKHR(instance, &SCI, nullptr, &surface));
        }

        // create descriptor pool for pipeline descriptors
        {
            constexpr size_t MAX_DESCRIPTORS = api::MAX_DESCRIPTORS_PER_DISPATCH * api::MAX_PIPELINES;
            std::vector<VkDescriptorPoolSize> poolSizes;
            poolSizes.push_back({ VK_DESCRIPTOR_TYPE_SAMPLER, MAX_DESCRIPTORS });
            poolSizes.push_back({ VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, MAX_DESCRIPTORS });
            poolSizes.push_back({ VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, MAX_DESCRIPTORS });
            poolSizes.push_back({ VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, MAX_DESCRIPTORS });
            poolSizes.push_back({ VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, MAX_DESCRIPTORS });
            poolSizes.push_back({ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, MAX_DESCRIPTORS });
            poolSizes.push_back({ VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, MAX_DESCRIPTORS });

            VkDescriptorPoolCreateInfo dpci = { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
            dpci.maxSets = api::MAX_PIPELINES;
            dpci.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
            dpci.pPoolSizes = poolSizes.data();
            
            CHECK_IF_FAILED(vkCreateDescriptorPool(m_device, &dpci, nullptr, &m_descriptorPool));;
        }

        // create swapchain
        {
            VkSwapchainCreateInfoKHR SCI = { VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR };
            SCI.surface = surface;
            SCI.minImageCount = api::FRAME_COUNT;
            SCI.imageFormat = VK_FORMAT_R8G8B8A8_UNORM;
            SCI.imageColorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
            SCI.imageExtent = { width, height };
            SCI.imageArrayLayers = 1;
            SCI.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
            SCI.preTransform = VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR;
            SCI.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
            SCI.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
            SCI.clipped = VK_TRUE;

            CHECK_IF_FAILED(vkCreateSwapchainKHR(m_device, &SCI, nullptr, &m_swapchain));

            uint32_t imageCount = api::FRAME_COUNT;

            m_swapchain_images.resize(imageCount);

            CHECK_IF_FAILED(vkGetSwapchainImagesKHR(m_device, m_swapchain, &imageCount, m_swapchain_images.data()));
            assert(imageCount == api::FRAME_COUNT);

            vkGetSwapchainImagesKHR(m_device, m_swapchain, &imageCount, m_swapchain_images.data());
            assert(imageCount == api::FRAME_COUNT);

            VkImageViewCreateInfo IVCI = { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
            IVCI.viewType = VK_IMAGE_VIEW_TYPE_2D;
            IVCI.format = SCI.imageFormat;
            IVCI.components = { VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY };
            IVCI.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            IVCI.subresourceRange.levelCount = 1;
            IVCI.subresourceRange.layerCount = 1;

            m_swapchain_image_views.resize(imageCount);

            for (uint32_t i = 0; i < api::FRAME_COUNT; i++)
            {
                IVCI.image = m_swapchain_images[i];

                CHECK_IF_FAILED(vkCreateImageView(m_device, &IVCI, nullptr, &m_swapchain_image_views[i]));

                m_swapchain_textures.push_back( new RenderTargetTexture(IVCI.image, m_swapchain_image_views[i], this, {width, height}));

                std::vector<VkDescriptorPoolSize> poolSizes;
                constexpr size_t MAX_DESCRIPTORS = api::MAX_DISPATCHES_PER_FRAME * api::MAX_DESCRIPTORS_PER_DISPATCH;
                poolSizes.push_back({ VK_DESCRIPTOR_TYPE_SAMPLER, MAX_DESCRIPTORS });
                poolSizes.push_back({ VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, MAX_DESCRIPTORS });
                poolSizes.push_back({ VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, MAX_DESCRIPTORS });
                poolSizes.push_back({ VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, MAX_DESCRIPTORS });
                poolSizes.push_back({ VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, MAX_DESCRIPTORS });
                poolSizes.push_back({ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, MAX_DESCRIPTORS });
                poolSizes.push_back({ VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, MAX_DESCRIPTORS });

                VkDescriptorPoolCreateInfo dpci = { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
                dpci.maxSets = api::MAX_DISPATCHES_PER_FRAME;
                dpci.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
                dpci.pPoolSizes = poolSizes.data();

                CHECK_IF_FAILED(vkCreateDescriptorPool(m_device,&dpci,nullptr,&m_frames[i].descriptorPool))

                VkSemaphoreCreateInfo sci = { VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
                CHECK_IF_FAILED(vkCreateSemaphore(m_device, &sci, nullptr, &m_frames[i].imageAquired));
                CHECK_IF_FAILED(vkCreateSemaphore(m_device, &sci, nullptr, &m_frames[i].renderComplete));

                VkFenceCreateInfo fci = { VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
                fci.flags = VK_FENCE_CREATE_SIGNALED_BIT;
                CHECK_IF_FAILED(vkCreateFence(m_device, &fci, nullptr, &m_frames[i].renderCompleteFence));
            }
            
            VkSemaphoreCreateInfo sci = { VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };

            VkSemaphoreCreateInfo tsci = { VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
            VkSemaphoreTypeCreateInfo stci = { VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO };
            stci.initialValue = 0;
            stci.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
            tsci.pNext = &stci;
            
            CHECK_IF_FAILED(vkCreateSemaphore(m_device, &tsci, nullptr, &m_timelineSemaphore));
            m_timelineSemaphoreValue = 0;

            CHECK_IF_FAILED(vkAcquireNextImageKHR(m_device, m_swapchain, UINT64_MAX, m_frames[m_frameIndex].imageAquired, VK_NULL_HANDLE, &m_backbufferIndex));
            WaitForImageReady();
        }

        // create command pool
        {
            VkCommandPoolCreateInfo CPCI = { VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };

            CPCI.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
            CPCI.queueFamilyIndex = queueFamilyIndex;

            CHECK_IF_FAILED(vkCreateCommandPool(m_device, &CPCI, nullptr, &m_commandPool));
        }

     

        // setup vma
        {
            VmaAllocatorCreateInfo vaci = { .physicalDevice = physicalDevice, .device = m_device, .instance = instance, .vulkanApiVersion = VK_API_VERSION_1_3 };
            vaci.flags = VMA_ALLOCATOR_CREATE_BUFFER_DEVICE_ADDRESS_BIT | VMA_ALLOCATOR_CREATE_KHR_MAINTENANCE5_BIT;

            VmaVulkanFunctions vvf;
            vmaImportVulkanFunctionsFromVolk(&vaci, &vvf);

            vaci.pVulkanFunctions = &vvf;

            CHECK_IF_FAILED(vmaCreateAllocator(&vaci, &m_vmaAllocator));
        }

        // imgui setup
        {
            const VkFormat colorFormat{ VK_FORMAT_R8G8B8A8_UNORM };

            VkPipelineRenderingCreateInfoKHR prci = { VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
            prci.colorAttachmentCount = 1;
            prci.pColorAttachmentFormats = &colorFormat;
            prci.depthAttachmentFormat = VK_FORMAT_D32_SFLOAT;

            ImGui_ImplVulkan_InitInfo ii = {};
            ii.ApiVersion = VK_API_VERSION_1_3;
            ii.Instance = instance;
            ii.PhysicalDevice = physicalDevice;
            ii.Device = m_device;
            ii.QueueFamily = queueFamilyIndex;
            ii.Queue = m_queue;
            ii.DescriptorPoolSize = 50;
            ii.MinImageCount = api::FRAME_COUNT;
            ii.ImageCount = api::FRAME_COUNT;

            ii.UseDynamicRendering = true;
            ii.PipelineRenderingCreateInfo = prci;

            ImGui_ImplVulkan_Init(&ii);
        }

        // swapchain images UNDEFINED -> PRESENT_SRC
        for (uint32_t i = 0; i < api::FRAME_COUNT; i++)
        {
            TransitionImageFromUndefined(this, m_device, m_queue, m_commandPool, m_swapchain_images[i], VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_ASPECT_COLOR_BIT);
        }
    }

    vk::CommandList* Device::BeginCommandList()
    {
        VkCommandBuffer cb;
        VkCommandBufferAllocateInfo CBAI = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
        CBAI.commandPool = m_commandPool;
        CBAI.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        CBAI.commandBufferCount = 1;

        CHECK_IF_FAILED(vkAllocateCommandBuffers(m_device, &CBAI, &cb));

        vk::CommandList* list = new vk::CommandList(cb, m_device, m_vmaAllocator, m_commandPool,this);

        m_frames[m_frameIndex].commandLists.push_back(list);
        VkCommandBufferBeginInfo cbbi = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
        cbbi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        vkBeginCommandBuffer(cb, &cbbi);

        return list;
    }

    void Device::SubmitCommandList(api::CommandList* commandList)
    {
        auto cb = GetCommandList(commandList);

        // insert a global barrier to emulate D3D's implicit flush behavior
        VkMemoryBarrier barrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER };
        barrier.dstAccessMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
        barrier.srcAccessMask = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
        vkCmdPipelineBarrier(cb, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 1, &barrier, 0, nullptr, 0, nullptr);

        vkEndCommandBuffer(cb);

        VkPipelineStageFlags stage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;

        // Use a timeline semaphore to emulate D3D's implicit sync behavior
        uint64_t waitValue = m_timelineSemaphoreValue;
        uint64_t signalValue = ++m_timelineSemaphoreValue;

        VkTimelineSemaphoreSubmitInfo timelineInfo;
        timelineInfo.sType = VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO;
        timelineInfo.pNext = NULL;
        timelineInfo.waitSemaphoreValueCount = 1;
        timelineInfo.pWaitSemaphoreValues = &waitValue;
        timelineInfo.signalSemaphoreValueCount = 1;
        timelineInfo.pSignalSemaphoreValues = &signalValue;

        VkSubmitInfo si = { VK_STRUCTURE_TYPE_SUBMIT_INFO };
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cb;
        si.waitSemaphoreCount = 1;
        si.pWaitSemaphores = &m_timelineSemaphore;
        si.pWaitDstStageMask = &stage;
        si.signalSemaphoreCount = 1;
        si.pSignalSemaphores = &m_timelineSemaphore;
        si.pNext = &timelineInfo;

        CHECK_IF_FAILED(vkQueueSubmit(m_queue, 1, &si, nullptr));
    }

    static VkDescriptorSetLayout CreateDescriptorSetLayout(
        VkDevice device,
        std::span<const api::RootParameterType> rootSig
    )
    {
        assert(rootSig.size() <= api::MAX_DESCRIPTORS_PER_DISPATCH);

        std::vector<VkDescriptorSetLayoutBinding> bindings(rootSig.size());
        for (size_t i = 0; i < rootSig.size(); i++)
        {
            bindings[i].pImmutableSamplers = nullptr;
            bindings[i].stageFlags = VK_SHADER_STAGE_ALL;
            bindings[i].descriptorCount = 1;
            bindings[i].binding = (UINT)i;
            switch (rootSig[i])
            {
            case api::RootParameterType::BUFFER_SRV:
            case api::RootParameterType::BUFFER_UAV:
                bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                break;

            case api::RootParameterType::CONSTANT_BUFFER:
                bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                break;

            case api::RootParameterType::IMAGE_UAV:
                bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                break;

            case api::RootParameterType::RTAS:
                bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
                break;
            }
        }
        
        VkDescriptorSetLayout ds_layout;
        VkDescriptorSetLayoutCreateInfo dslci = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        dslci.bindingCount = static_cast<uint32_t>(bindings.size());
        dslci.pBindings = bindings.data();

        CHECK_IF_FAILED(vkCreateDescriptorSetLayout(device, &dslci, nullptr, &ds_layout));
        return ds_layout;
    }

    vk::RenderingPipeline* Device::CreateRenderingPipeline(
        ShaderCompiler& compiler,
        const WCHAR* path,
        std::span<const api::RootParameterType> rootSig,
        std::span<const DxcDefine> defines )
    {
        // compile shaders
        auto meshShaderBlob = compiler.Compile(path, L"MS_main", L"ms_6_6", true, defines);
        auto pixelShaderBlob = compiler.Compile(path, L"PS_main", L"ps_6_6", true, defines);

        // create shader modules
        VkShaderModuleCreateInfo smci = { VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
        smci.codeSize = meshShaderBlob.GetBufferSize();
        smci.pCode = static_cast<const uint32_t *>(meshShaderBlob.GetBufferPointer());

        VkShaderModule mesh_shader_module;
        CHECK_IF_FAILED(vkCreateShaderModule(m_device, &smci, nullptr, &mesh_shader_module));

        smci.codeSize = pixelShaderBlob.GetBufferSize();
        smci.pCode = static_cast<const uint32_t*>(pixelShaderBlob.GetBufferPointer());

        VkShaderModule pixel_shader_module;
        CHECK_IF_FAILED(vkCreateShaderModule(m_device, &smci, nullptr, &pixel_shader_module));

        VkDescriptorSetLayout ds_layout = CreateDescriptorSetLayout(m_device, rootSig);

        // create pipeline layout
        VkPipelineLayout layout;
        VkPipelineLayoutCreateInfo lci = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        lci.setLayoutCount = 1;
        lci.pSetLayouts = &ds_layout;
        
        CHECK_IF_FAILED(vkCreatePipelineLayout(m_device, &lci, nullptr, &layout));

        // create pipeline
        VkPipelineShaderStageCreateInfo stages[2] = {{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}, {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO} };
        stages[0].stage = VK_SHADER_STAGE_MESH_BIT_EXT;
        stages[0].module = mesh_shader_module;
        stages[0].pName = "MS_main";
        stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        stages[1].module = pixel_shader_module;
        stages[1].pName = "PS_main";

        VkPipelineViewportStateCreateInfo vpsi = { VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
        vpsi.viewportCount = 1;
        vpsi.scissorCount = 1;

        VkPipelineRasterizationStateCreateInfo rsci = { VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
        rsci.lineWidth = 1;

        VkPipelineMultisampleStateCreateInfo msci = { VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
        msci.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        VkPipelineDepthStencilStateCreateInfo dssci = { VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO };
        dssci.depthTestEnable = VK_TRUE;
        dssci.depthCompareOp = VK_COMPARE_OP_LESS;
        dssci.depthWriteEnable = VK_TRUE;

        VkPipelineColorBlendAttachmentState cbas = {};
        cbas.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

        VkPipelineColorBlendStateCreateInfo cbsci = { VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
        cbsci.attachmentCount = 1;
        cbsci.pAttachments = &cbas;

        VkDynamicState ds[2] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };

        VkPipelineDynamicStateCreateInfo dsci = { VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO };
        dsci.dynamicStateCount = 2;
        dsci.pDynamicStates = ds;

        VkFormat format = VK_FORMAT_R8G8B8A8_UNORM;
        VkPipelineRenderingCreateInfo rci = { VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
        rci.colorAttachmentCount = 1;
        rci.pColorAttachmentFormats = &format;
        rci.depthAttachmentFormat = VK_FORMAT_D32_SFLOAT;

        VkGraphicsPipelineCreateInfo gpci = { VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
        gpci.pNext = &rci;
        gpci.stageCount = 2;
        gpci.pStages = stages;
        gpci.pViewportState = &vpsi;
        gpci.pRasterizationState = &rsci;
        gpci.pMultisampleState = &msci;
        gpci.pDepthStencilState = &dssci;
        gpci.pColorBlendState = &cbsci;
        gpci.pDynamicState = &dsci;
        gpci.layout = layout;

        VkPipeline pipeline;
        CHECK_IF_FAILED(vkCreateGraphicsPipelines(m_device, VK_NULL_HANDLE, 1, &gpci, nullptr, &pipeline));

        VkDescriptorSetAllocateInfo dsai = {VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        dsai.descriptorPool = m_descriptorPool;
        dsai.descriptorSetCount = 1;
        dsai.pSetLayouts = &ds_layout;

        VkDescriptorSet descriptor_set;
        CHECK_IF_FAILED(vkAllocateDescriptorSets(m_device, &dsai, &descriptor_set));

        return new RenderingPipeline(pipeline, layout, descriptor_set, (uint32_t)rootSig.size(), ds_layout );
    }

    vk::ComputePipeline* Device::CreateComputePipeline(
        ShaderCompiler& compiler,
        const WCHAR* path,
        std::span<const api::RootParameterType> rootSig,
        std::span<const DxcDefine> defines ,
        const WCHAR* entrypoint) 
    {
        auto Blob = compiler.Compile(path, entrypoint, L"cs_6_6", true, defines);

        // create shader module
        VkShaderModuleCreateInfo smci = { VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
        smci.codeSize = Blob.GetBufferSize();
        smci.pCode = static_cast<const uint32_t*>(Blob.GetBufferPointer());

        VkShaderModule shader_module;
        CHECK_IF_FAILED(vkCreateShaderModule(m_device, &smci, nullptr, &shader_module));

        VkDescriptorSetLayout ds_layout = CreateDescriptorSetLayout(m_device, rootSig);
 
        // create pipeline layout
        VkPipelineLayout layout;
        VkPipelineLayoutCreateInfo lci = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        lci.setLayoutCount = 1;
        lci.pSetLayouts = &ds_layout;
        CHECK_IF_FAILED(vkCreatePipelineLayout(m_device, &lci, nullptr, &layout));

        std::string entrypointA = wstring_to_string(std::wstring(entrypoint));

        // create pipeline
        VkPipelineShaderStageCreateInfo pssci = { VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
        pssci.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        pssci.module = shader_module;
        pssci.pName = entrypointA.c_str();

        VkPipeline pipeline;
        VkComputePipelineCreateInfo ci = { VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
        ci.stage = pssci;
        ci.layout = layout;

        CHECK_IF_FAILED(vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &ci, nullptr, &pipeline));

        VkDescriptorSetAllocateInfo dsai = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        dsai.descriptorPool = m_descriptorPool;
        dsai.descriptorSetCount = 1;
        dsai.pSetLayouts = &ds_layout;

        VkDescriptorSet descriptor_set;
        CHECK_IF_FAILED(vkAllocateDescriptorSets(m_device, &dsai, &descriptor_set));

        return new ComputePipeline(pipeline, layout, descriptor_set, (uint32_t) rootSig.size(), ds_layout );
    }

    vk::Texture* Device::CreateDepthStencilBuffer(uint32_t width, uint32_t height)
    {
        VkImageCreateInfo ici = { VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ici.imageType = VK_IMAGE_TYPE_2D;
        ici.extent.width = width;
        ici.extent.height = height;
        ici.extent.depth = 1;
        ici.mipLevels = 1;
        ici.arrayLayers = 1;
        ici.format = VK_FORMAT_D32_SFLOAT;
        ici.tiling = VK_IMAGE_TILING_OPTIMAL;
        ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        ici.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
        ici.samples = VK_SAMPLE_COUNT_1_BIT;

        VmaAllocationCreateInfo allocCreateInfo = {};
        allocCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocCreateInfo.flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT;
        allocCreateInfo.priority = 1.0f;

        VkImage image;
        VmaAllocation alloc;
        CHECK_IF_FAILED(vmaCreateImage(m_vmaAllocator, &ici, &allocCreateInfo, &image, &alloc, nullptr));

        VkImageView image_view;

        VkImageViewCreateInfo ivci = { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        ivci.image = image;
        ivci.viewType = VK_IMAGE_VIEW_TYPE_2D;
        ivci.format = VK_FORMAT_D32_SFLOAT;
        ivci.components = { VK_COMPONENT_SWIZZLE_R , VK_COMPONENT_SWIZZLE_G, VK_COMPONENT_SWIZZLE_B, VK_COMPONENT_SWIZZLE_A };
        ivci.subresourceRange = { VK_IMAGE_ASPECT_DEPTH_BIT , 0, 1, 0, 1};
        
        CHECK_IF_FAILED(vkCreateImageView(m_device, &ivci, nullptr, &image_view));

        TransitionImageFromUndefined(this, m_device, m_queue, m_commandPool, image, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL, VK_IMAGE_ASPECT_DEPTH_BIT);

        return new DepthStencilTexture(image, image_view, this, {width, height});
    }

    vk::Texture* Device::CreateUAVTexture(uint32_t width, uint32_t height)
    {
        VkImageCreateInfo ici = { VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
        ici.imageType = VK_IMAGE_TYPE_2D;
        ici.extent.width = width;
        ici.extent.height = height;
        ici.extent.depth = 1;
        ici.mipLevels = 1;
        ici.arrayLayers = 1;
        ici.format = VK_FORMAT_R8G8B8A8_UNORM;
        ici.tiling = VK_IMAGE_TILING_OPTIMAL;
        ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        ici.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        ici.samples = VK_SAMPLE_COUNT_1_BIT;

        VmaAllocationCreateInfo allocCreateInfo = {};
        allocCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocCreateInfo.flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT;
        allocCreateInfo.priority = 1.0f;

        VkImage image;
        VmaAllocation alloc;
        CHECK_IF_FAILED(vmaCreateImage(m_vmaAllocator, &ici, &allocCreateInfo, &image, &alloc, nullptr));

        VkImageView image_view;

        VkImageViewCreateInfo ivci = { VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        ivci.image = image;
        ivci.viewType = VK_IMAGE_VIEW_TYPE_2D;
        ivci.format = VK_FORMAT_R8G8B8A8_UNORM;
        ivci.components = { VK_COMPONENT_SWIZZLE_R , VK_COMPONENT_SWIZZLE_G, VK_COMPONENT_SWIZZLE_B, VK_COMPONENT_SWIZZLE_A };
        ivci.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT , 0, 1, 0, 1 };

        CHECK_IF_FAILED(vkCreateImageView(m_device, &ivci, nullptr, &image_view));

        TransitionImageFromUndefined(this, m_device, m_queue, m_commandPool, image, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_ASPECT_COLOR_BIT);

        return new Texture(image, image_view, this, VK_IMAGE_ASPECT_COLOR_BIT, { width, height, 1});
    }

    vk::RTAS* Device::CreateRTAS(size_t size)
    {
        VkBufferCreateInfo bufCreateInfo = { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bufCreateInfo.size = size;
        bufCreateInfo.usage = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
            VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;

        VmaAllocationCreateInfo allocCreateInfo = {};
        allocCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocCreateInfo.flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT;

        VkBuffer buf;
        VmaAllocation alloc;
        VmaAllocationInfo allocInfo;
        vmaCreateBuffer(m_vmaAllocator, &bufCreateInfo, &allocCreateInfo, &buf, &alloc, &allocInfo);

        VkAccelerationStructureCreateInfoKHR asci = {VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR};
        asci.buffer = buf;
        asci.size = size;
        asci.type = VK_ACCELERATION_STRUCTURE_TYPE_GENERIC_KHR;

        VkAccelerationStructureKHR as;
        CHECK_IF_FAILED(vkCreateAccelerationStructureKHR(m_device, &asci, nullptr, &as));

        return new RTAS(buf, m_vmaAllocator, alloc, m_device, as);
    }

    constexpr VkFlags BUFFER_USAGE_GENERIC =
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT |
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
        VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
        VK_BUFFER_USAGE_TRANSFER_DST_BIT | 
        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
        ;

    vk::Buffer* Device::CreateUAVBuffer(size_t size)
    {
        VkBufferUsageFlags2CreateInfo BUF2CI = { VK_STRUCTURE_TYPE_BUFFER_USAGE_FLAGS_2_CREATE_INFO };
        BUF2CI.usage = BUFFER_USAGE_GENERIC;

        if (m_nativeDGF)
            BUF2CI.usage |= VK_BUFFER_USAGE_2_COMPRESSED_DATA_DGF1_BIT_AMDX;

        VkBufferCreateInfo bufCreateInfo = { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bufCreateInfo.pNext = &BUF2CI;
        bufCreateInfo.size = size;
        bufCreateInfo.usage = BUFFER_USAGE_GENERIC;

        VmaAllocationCreateInfo allocCreateInfo = {};
        allocCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocCreateInfo.flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT;

        VkBuffer buf;
        VmaAllocation alloc;
        VmaAllocationInfo allocInfo;
        vmaCreateBuffer(m_vmaAllocator, &bufCreateInfo, &allocCreateInfo, &buf, &alloc, &allocInfo);

        return new vk::Buffer(buf, m_device, m_vmaAllocator, alloc);
    }

    vk::Buffer* Device::CreateDynamicBuffer(size_t size)
    {
        vk::VMABuffer renameChain[api::FRAME_COUNT];

        VkBufferCreateInfo bufCreateInfo = { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bufCreateInfo.size = size;
        bufCreateInfo.usage = BUFFER_USAGE_GENERIC;

        VmaAllocationCreateInfo allocCreateInfo = {};
        allocCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocCreateInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;

        VmaAllocationInfo allocInfo;

        for (size_t i = 0; i < api::FRAME_COUNT; i++)
        {
            vmaCreateBuffer(m_vmaAllocator, &bufCreateInfo, &allocCreateInfo, &renameChain[i].buffer, &renameChain[i].allocation, &allocInfo);
        }
        
        return new vk::DynamicBuffer(m_device, m_vmaAllocator, renameChain);
    }

    vk::Buffer* Device::CreateBuffer(size_t size)
    {
        VkBufferCreateInfo bufCreateInfo = { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bufCreateInfo.size = size;
        bufCreateInfo.usage = BUFFER_USAGE_GENERIC | VK_BUFFER_USAGE_TRANSFER_DST_BIT;

        VmaAllocationCreateInfo allocCreateInfo = {};
        allocCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocCreateInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;

        VkBuffer buf;
        VmaAllocation alloc;
        VmaAllocationInfo allocInfo;
        vmaCreateBuffer(m_vmaAllocator, &bufCreateInfo, &allocCreateInfo, &buf, &alloc, &allocInfo);

        return new vk::Buffer(buf, m_device, m_vmaAllocator, alloc);
    }

    api::RTASPrebuildInfo Device::GetRTASPrebuildInfo(const api::RTASDesc& desc) 
    {
        VkAccelerationStructureDenseGeometryFormatTrianglesDataAMDX ASDGFFTD = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DENSE_GEOMETRY_FORMAT_TRIANGLES_DATA_AMDX };
        VkAccelerationStructureBuildSizesInfoKHR ASBSI = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
        VkAccelerationStructureBuildGeometryInfoKHR ASBGI = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };

        uint32_t primitive_count;

        VkAccelerationStructureGeometryKHR ASG = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
        ASG.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;

        ASBGI.flags = TranslateRTASFlags(desc.Flags);
        ASBGI.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
        ASBGI.geometryCount = 1;
        ASBGI.pGeometries = &ASG;

        api::RTASPrebuildInfo prebuildInfo;

        switch (desc.Type)
        {
            case api::RTASType::TOP_LEVEL:
            {
                ASBGI.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;

                ASG.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
                ASG.geometry.instances = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR };
                ASG.geometry.instances.arrayOfPointers = VK_FALSE;

                primitive_count = desc.TLAS.NumInstances;

                break;
            }
            case api::RTASType::BOTTOM_LEVEL:
            {
                ASBGI.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;

                ASG.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;

                ASG.geometry.triangles = { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR };
                ASG.geometry.triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
                ASG.geometry.triangles.vertexStride = desc.BLAS.VertexStride;
                ASG.geometry.triangles.maxVertex = desc.BLAS.NumVertices - 1;
                ASG.geometry.triangles.indexType = VK_INDEX_TYPE_UINT32;

                primitive_count = desc.BLAS.NumIndices / 3;

                break;
            }
            case api::RTASType::DGF_BOTTOM_LEVEL:
            {
                ASBGI.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;

                ASG.geometryType = VK_GEOMETRY_TYPE_DENSE_GEOMETRY_FORMAT_TRIANGLES_AMDX;
                ASG.pNext = &ASDGFFTD;

                ASDGFFTD.compressedData.deviceAddress = GetBufferAddress(desc.DGFBLAS.blocks, desc.DGFBLAS.blocksByteOffset);
                ASDGFFTD.dataSize = desc.DGFBLAS.numBlocks * VK_COMPRESSED_TRIANGLE_FORMAT_DGF1_BYTE_STRIDE_AMDX;
                ASDGFFTD.numTriangles = desc.DGFBLAS.numTriangles;
                ASDGFFTD.numVertices = desc.DGFBLAS.numVertices;
                ASDGFFTD.maxPrimitiveIndex = desc.DGFBLAS.maxPrimitiveIndex;
                ASDGFFTD.maxGeometryIndex = desc.DGFBLAS.maxGeometryIndex;
                ASDGFFTD.format = VK_COMPRESSED_TRIANGLE_FORMAT_DGF1_AMDX;

                primitive_count = 0;

                break;
            }
            default:
            {
                assert(0);
                break;
            }
        }

        vkGetAccelerationStructureBuildSizesKHR(
            m_device,
            VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
            &ASBGI,
            &primitive_count,
            &ASBSI);

        prebuildInfo = { ASBSI.accelerationStructureSize, ASBSI.buildScratchSize, ASBSI.updateScratchSize };

        return prebuildInfo;
    }

    void Device::WaitForImageReady()
    {
        // dummy queue submit to wait on the image ready binary semaphore and signal the queue timeline semaphore
        VkPipelineStageFlags stage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;

        uint64_t signalValue = ++m_timelineSemaphoreValue;

        VkTimelineSemaphoreSubmitInfo timelineInfo;
        timelineInfo.sType = VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO;
        timelineInfo.pNext = NULL;
        timelineInfo.waitSemaphoreValueCount = 0;
        timelineInfo.pWaitSemaphoreValues = nullptr;
        timelineInfo.signalSemaphoreValueCount = 1;
        timelineInfo.pSignalSemaphoreValues = &signalValue;

        VkSubmitInfo si = { VK_STRUCTURE_TYPE_SUBMIT_INFO };
        si.commandBufferCount = 0;
        si.pCommandBuffers = nullptr;
        si.waitSemaphoreCount = 1;
        si.pWaitSemaphores = &m_frames[m_frameIndex].imageAquired;
        si.pWaitDstStageMask = &stage;
        si.signalSemaphoreCount = 1;
        si.pSignalSemaphores = &m_timelineSemaphore;
        si.pNext = &timelineInfo;

        CHECK_IF_FAILED(vkQueueSubmit(m_queue, 1, &si, nullptr));       
    }


    void Device::PresentAndAdvanceFrame()
    {
        CHECK_IF_FAILED(vkResetFences(m_device, 1, &m_frames[m_frameIndex].renderCompleteFence));

        // HACK: Dummy submit to Synchronize with the last submitted command buffer
        //  because presentation and timeline semaphores don't mix
        {
            VkPipelineStageFlags stage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;

            uint64_t waitValue = m_timelineSemaphoreValue;

            VkTimelineSemaphoreSubmitInfo timelineInfo;
            timelineInfo.sType = VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO;
            timelineInfo.pNext = NULL;
            timelineInfo.waitSemaphoreValueCount = 1;
            timelineInfo.pWaitSemaphoreValues = &waitValue;
            timelineInfo.signalSemaphoreValueCount = 0;
            timelineInfo.pSignalSemaphoreValues = nullptr;

            VkSubmitInfo si = { VK_STRUCTURE_TYPE_SUBMIT_INFO };
            si.commandBufferCount = 0;
            si.pCommandBuffers = nullptr;
            si.waitSemaphoreCount = 1;
            si.pWaitSemaphores = &m_timelineSemaphore;
            si.pWaitDstStageMask = &stage;
            si.signalSemaphoreCount = 1;
            si.pSignalSemaphores = &m_frames[m_frameIndex].renderComplete;            
            si.pNext = &timelineInfo;

            CHECK_IF_FAILED(vkQueueSubmit(m_queue, 1, &si, m_frames[m_frameIndex].renderCompleteFence));
        }

        VkPresentInfoKHR pi = { VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
        pi.waitSemaphoreCount = 1;
        pi.pWaitSemaphores = &m_frames[m_frameIndex].renderComplete;
        pi.swapchainCount = 1;
        pi.pSwapchains = &m_swapchain;
        pi.pImageIndices = &m_backbufferIndex;       
        
        VkResult res = vkQueuePresentKHR(m_queue, &pi);
        assert(res == VK_SUCCESS || res == VK_SUBOPTIMAL_KHR);

        m_frameIndex = (m_frameIndex + 1) % api::FRAME_COUNT;
        
        // wait for previous frame fence to retire and reset for next submission        
        CHECK_IF_FAILED(vkWaitForFences(m_device, 1, &m_frames[m_frameIndex].renderCompleteFence, VK_TRUE, UINT64_MAX));
        CHECK_IF_FAILED(vkAcquireNextImageKHR(m_device, m_swapchain, UINT64_MAX, m_frames[m_frameIndex].imageAquired, VK_NULL_HANDLE, &m_backbufferIndex));
        
        // dummy command buffer to wait on image-acquire semaphore
        WaitForImageReady();

        for (api::CommandList* list : m_frames[m_frameIndex].commandLists)
            delete list;
        m_frames[m_frameIndex].commandLists.clear();
        
        CHECK_IF_FAILED(vkResetDescriptorPool(m_device, m_frames[m_frameIndex].descriptorPool, 0));
    }

    void Device::WaitForGPUIdle()
    {
        vkDeviceWaitIdle(m_device);

        // all frames have now retired.. reap all command lists
        for (size_t i = 0; i < api::FRAME_COUNT; i++)
        {
            for (vk::CommandList* cl : m_frames[i].commandLists)
                delete cl;

            m_frames[i].commandLists.clear();
            CHECK_IF_FAILED(vkResetDescriptorPool(m_device, m_frames[i].descriptorPool, 0));
        }
    }

    Device::~Device()
    {
       ImGui_ImplVulkan_Shutdown();
    }

    void Device::IMGUI_NewFrame()
    {
       ImGui_ImplVulkan_NewFrame();
    }

    void Device::IMGUI_RenderDrawData(api::CommandList* cmdList)
    {
        ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), GetCommandList(cmdList));
    }

    vk::TimerHeap* Device::CreateTimerHeap(size_t numTimers)
    {
        VkQueryPool pool;

        VkQueryPoolCreateInfo qpci = { VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
        qpci.queryCount = (uint32_t)numTimers;
        qpci.queryType = VK_QUERY_TYPE_TIMESTAMP;
        CHECK_IF_FAILED(vkCreateQueryPool(m_device, &qpci, nullptr, &pool));

        vkResetQueryPool(m_device, pool, 0, (uint32_t)numTimers);

        VkBufferCreateInfo bufCreateInfo = { VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
        bufCreateInfo.size = numTimers * sizeof(uint64_t);
        bufCreateInfo.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;

        VmaAllocationCreateInfo allocCreateInfo = {};
        allocCreateInfo.usage = VMA_MEMORY_USAGE_AUTO;
        allocCreateInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;

        VkBuffer buf;
        VmaAllocation alloc;
        VmaAllocationInfo allocInfo;
        vmaCreateBuffer(m_vmaAllocator, &bufCreateInfo, &allocCreateInfo, &buf, &alloc, &allocInfo);

        double gpuTickDuration = m_timestampPeriod / 1000000000.0;

        return new vk::TimerHeap(pool, m_vmaAllocator, alloc, buf, gpuTickDuration, numTimers);
    }

    VkDescriptorSet vk::Device::AllocateDispatchDescriptors(size_t num, VkDescriptorSetLayout setLayout)
    {
        FrameState& currentFrame = m_frames[m_frameIndex];

        VkDescriptorSetAllocateInfo alloc = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        alloc.descriptorPool = currentFrame.descriptorPool;
        alloc.descriptorSetCount = 1;
        alloc.pSetLayouts = &setLayout;

        VkDescriptorSet set;
        CHECK_IF_FAILED(vkAllocateDescriptorSets(m_device, &alloc, &set));
        return set;
    }

    void Device::WriteDynamicBuffer(api::Buffer* buffer, const void* contents, size_t size)
    {
        vk::DynamicBuffer* dyn_buffer = static_cast<vk::DynamicBuffer*>(buffer);
        dyn_buffer->Write(contents, size, m_frameIndex);
    }

    Device* CreateDevice(HWND hwnd, HINSTANCE hInstance, uint32_t width, uint32_t height)
    {
        return new Device(hwnd,hInstance,width,height);        
    }

}