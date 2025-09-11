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


#include "MeshShaderUtil.hlsl"
#include "DGFDecompression.hlsl"

#define RS_COMPUTE_SHADER \
"CBV(b0),"\
"SRV(t0),"\
"SRV(t1),"\
"UAV(u0)"

struct ConstantDataStruct
{
    uint DGFBlockStart;
};

[[vk::binding(0, 0)]]
ConstantBuffer<ConstantDataStruct> Constants : register(b0);

[[vk::binding(1, 0)]]
StructuredBuffer<float3> Positions : register(t0);

[[vk::binding(2, 0)]]
StructuredBuffer<uint> VertexTable : register(t1);

[[vk::binding(3, 0)]]
RWByteAddressBuffer DgfBlockBuffer : register(u0);


int3 Quantize(float3 vertex, float scale)
{
    return round(vertex * scale);
}

uint3 GetIndices(const uint bitAddress)
{
    const uint byteAddress = bitAddress / 8;
    const uint index0 = byteAddress / 4;

    // We use the mask to write out values
    // Therefore we can ignore any additional indices, when they are not needed
    return uint3(index0, index0 + 1, index0 + 2);
}

uint3 GetMasks(const uint bitAddress, const uint3 bitsPerChannel)
{
    const uint startBitAddress = bitAddress % 32;
    const uint vertexBitSize = bitsPerChannel.x + bitsPerChannel.y + bitsPerChannel.z;

    const bool twoWordOverflow = (startBitAddress + vertexBitSize) > 32;
    const bool threeWordOverflow = (startBitAddress + vertexBitSize) > 64;

    const uint mask0 = uint((((uint64_t) 1 << vertexBitSize) - 1) << startBitAddress);
    uint mask1 = twoWordOverflow ? uint(((uint64_t) 1 << (startBitAddress + vertexBitSize - 32)) - 1) : 0x0;
    const uint mask2 = threeWordOverflow ? uint(((uint64_t) 1 << (startBitAddress + vertexBitSize - 64)) - 1) : 0x0;

    mask1 = threeWordOverflow ? ~0x0 : mask1;

    return uint3(mask0, mask1, mask2);
}

uint3 GetValues(const uint bitAddress, const uint3 bitsPerChannel, const uint64_t value)
{
    const uint startBitAddress = bitAddress % 32;
    const uint vertexBitSize = bitsPerChannel.x + bitsPerChannel.y + bitsPerChannel.z;

    const bool twoWordOverflow = (startBitAddress + vertexBitSize) > 32;
    const bool threeWordOverflow = (startBitAddress + vertexBitSize) > 64;

    const uint value0 = uint(value << startBitAddress);
    const uint value1 = twoWordOverflow ? uint(value >> (32 - startBitAddress)) : 0x0;
    const uint value2 = threeWordOverflow ? uint(value >> (32 - startBitAddress + 32)) : 0x0;

    return uint3(value0, value1, value2);
}

groupshared uint blockData[32];

[RootSignature(RS_COMPUTE_SHADER)]
[NumThreads(32, 1, 1)]
// We can set the wave size to 32, since we fix the bit-width of vertices to 16, limiting the vertex count to 16
[WaveSize(32)]
void CS_main(
    uint gtid : SV_GroupThreadID,
    uint gid : SV_GroupID)
{
    const uint blockId = Constants.DGFBlockStart + gid;
    DGFBlockInfoRW blockInfo = DGFInitRW(DgfBlockBuffer, blockId);
    
    blockData[gtid] = DgfBlockBuffer.Load(blockInfo.blockStartOffset + 4 * gtid);
    
    GroupMemoryBarrierWithGroupSync();
    
    const uint vertexBitAddress = (gtid * blockInfo.bitsPerVertex);

    const uint3 indices = GetIndices(vertexBitAddress);
    const uint3 masks = GetMasks(vertexBitAddress, blockInfo.header.bitsPerComponent);

    // Bit location in 4 Byte increment
    const uint vertexStartIndex = blockInfo.header.bitSize / 32;
    
    if (gtid < blockInfo.header.numVerts)
    {
        // Write out 0s to clear current vertex data
        InterlockedAnd(blockData[vertexStartIndex + indices.x], ~masks.x);
        InterlockedAnd(blockData[vertexStartIndex + indices.y], ~masks.y);
        InterlockedAnd(blockData[vertexStartIndex + indices.z], ~masks.z);
        
    }
    
    GroupMemoryBarrierWithGroupSync();
    
    int3 quantizedPositions = int3(2147483647, 2147483647, 2147483647);
    
    if (gtid < blockInfo.header.numVerts)
    {
        uint vertexindex = VertexTable.Load(blockInfo.header.userData + gtid);
        float3 blockPosition = Positions.Load(vertexindex);
        int unbiasedExponent = -((blockData[1] & 0xff) - 127);
        quantizedPositions = Quantize(blockPosition, pow(2, unbiasedExponent));
    }
    int3 minimumAnchor = WaveActiveMin(quantizedPositions);
    
    if (gtid < blockInfo.header.numVerts)
    {
        int3 offsetVert = quantizedPositions - minimumAnchor;
        
        const uint64_t xbits = blockInfo.header.bitsPerComponent.x;
        const uint64_t xybits = blockInfo.header.bitsPerComponent.x + blockInfo.header.bitsPerComponent.y;
        const uint64_t codedVert = offsetVert.x | (offsetVert.y << xbits) | ((uint64_t) offsetVert.z << xybits);
        
        const uint3 values = GetValues(vertexBitAddress, blockInfo.header.bitsPerComponent, codedVert);
        
        InterlockedOr(blockData[vertexStartIndex + indices.x], values.x & masks.x);
        InterlockedOr(blockData[vertexStartIndex + indices.y], values.y & masks.y);
        InterlockedOr(blockData[vertexStartIndex + indices.z], values.z & masks.z);
    }
    
    
    // Update the anchors
    if (WaveIsFirstLane())
    {
        blockData[1] = (blockData[1] & 0x000000FF) | ((minimumAnchor.x << 8) & 0xFFFFFF00);
        blockData[2] = (blockData[2] & 0x000000FF) | ((minimumAnchor.y << 8) & 0xFFFFFF00);
        blockData[3] = (blockData[3] & 0x000000FF) | ((minimumAnchor.z << 8) & 0xFFFFFF00);
    }
    
    GroupMemoryBarrierWithGroupSync();
    
    DgfBlockBuffer.Store(blockInfo.blockStartOffset + 4 * gtid, blockData[gtid]);
    
}