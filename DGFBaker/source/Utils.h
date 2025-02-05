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

#pragma once
// clang-format off

#include <vector>
#include <cstdint>
#include <limits>
#include <cmath>
#include <chrono>
#include <bitset>
#include <ostream>
#include <bit>
#include <tuple>
#include <iomanip>
#include <cstring>

#include <array>
#include <span>
#include <algorithm>

#include "DGF.h" // DGF_ASSERT

namespace DGF 
{
    constexpr int32_t S24_MAX = 0x7FFFFF;
    constexpr int32_t S24_MIN = ~0x7FFFFF;


    inline static constexpr size_t CalculateVertexBits(size_t vertex_bits)
    {
        return DGF::VERTEX_BIT_ALIGNMENT * ((vertex_bits + DGF::VERTEX_BIT_ALIGNMENT - 1) / DGF::VERTEX_BIT_ALIGNMENT);
    }

    // linear algebra helpers

    template< class T >
    struct vec3
    {
        vec3() = default;
        vec3( const vec3<T>& ) = default;
        vec3<T>& operator=(const vec3<T>&) = default;
        
        vec3( T xx, T yy, T zz ) : x(xx), y(yy), z(zz) {}
        
        vec3<T> operator-(const vec3<T>& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; };
        vec3<T> operator+(const vec3<T>& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; };
        vec3<T> operator+(const T& s) const { return {x + s, y + s, z + s}; };        
        vec3<T> operator*(const vec3<T>& rhs) const { return {x * rhs.x, y * rhs.y, z * rhs.z}; };       
        vec3<T> operator*(const T& s) const { return {x * s, y * s, z * s}; };       
        bool operator==(const vec3<T>& rhs) const { return x == rhs.x && y == rhs.y && z == rhs.z; };
        bool operator!=(const vec3<T>& rhs) const { return !operator==(rhs); };
        bool operator<(const vec3<T>& rhs) const {
            if (x != rhs.x)
                return x < rhs.x;
            else if (y != rhs.y)
                return y < rhs.y;
            else 
                return z < rhs.z;
        };

        T& operator[](size_t idx)
        {
            DGF_ASSERT(idx < 3);
            static_assert(sizeof(vec3<T>) == 3 * sizeof(T));
            T* ptr = &x;
            return ptr[idx];
        };
        const T& operator[](size_t idx) const 
        {
            DGF_ASSERT(idx < 3);
            static_assert(sizeof(vec3<T>) == 3 * sizeof(T));
            const T* ptr = &x;
            return ptr[idx];
        };

        
        T x;
        T y;
        T z;
    };

    template< class T >
    inline vec3<T> min(vec3<T> a, vec3<T> b)
    {
        return {a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z};
    }

    template< class T> 
    inline vec3<T> min(vec3<T> a, T s)
    {
        return {a.x < s ? a.x : s, a.y < s ? a.y : s, a.z < s ? a.z : s};
    }

    template <class T>
    inline vec3<T> max(vec3<T> a, vec3<T> b)
    {
        return {a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z};
    }

    template< class T >
    inline vec3<T> operator/(T s, vec3<T> v)
    {
        return { s / v.x, s / v.y, s / v.z };
    }

    typedef vec3<float> float3;
    typedef vec3<int32_t> int3;
    typedef vec3<uint32_t> uint3;

    inline int3 operator>>(const int3& lhs, size_t shift)
    {
        return { lhs.x >> shift, lhs.y >> shift, lhs.z >> shift };
    }

    // misc. structs
    
    struct AABB
    {
        float3 Min;
        float3 Max;
        AABB()
        {
            constexpr float inf = std::numeric_limits<float>::infinity();
            Min                 = {inf, inf, inf};
            Max                 = {-inf, -inf, -inf};
        };
        AABB(float3 min, float3 max) :
            Min(min),
            Max(max)
        {
        }

        AABB(const AABB& rhs) = default;
        AABB& operator=(const AABB& rhs) = default;

        void Expand(float3 pt)
        { 
            Min = min(pt,Min);
            Max = max(pt,Max);
        }

        void Expand(AABB box)
        {
            Min = min(box.Min, Min);
            Max = max(box.Max, Max);
        }

        float HalfArea() const
        {
            float3 d = Max - Min;
            return d.x * ( d.y + d.z ) + d.y * d.z;
        }

        bool Contains(AABB box) const
        {
            return Min.x <= box.Min.x && Min.y <= box.Min.y && Min.z <= box.Min.z 
                && Max.x >= box.Max.x && Max.y >= box.Max.y && Max.z >= box.Max.z;
        }

        float LongEdgeLength() const
        {
            float3 d = Max - Min;
            return std::max(d.x, std::max(d.y, d.z));
        }
    };
    
    struct Timer
    {
        std::chrono::high_resolution_clock::time_point m_Last;

        inline Timer() : m_Last(std::chrono::high_resolution_clock::now()) {}

        // returns the time that passed since the last tick()
        inline std::chrono::nanoseconds Tick()
        {
            auto now = std::chrono::high_resolution_clock::now();
            auto dt = now - m_Last;
            m_Last = now;
            return dt;
        }
    };
    struct PerformanceData
    {
        std::chrono::nanoseconds Clustering            = std::chrono::nanoseconds(0);
        std::chrono::nanoseconds QuantizeVertices      = std::chrono::nanoseconds(0);

        std::chrono::nanoseconds PackTriangles = std::chrono::nanoseconds(0);
        std::chrono::nanoseconds PickBestCandidate = std::chrono::nanoseconds(0);
        
        std::chrono::nanoseconds TryPack = std::chrono::nanoseconds(0);
        std::chrono::nanoseconds TryPackMaterialIds    = std::chrono::nanoseconds(0);
        std::chrono::nanoseconds GenerateStrips = std::chrono::nanoseconds(0);

        std::chrono::nanoseconds BuildClusterGraph = std::chrono::nanoseconds(0);
        std::chrono::nanoseconds BuildBlocks = std::chrono::nanoseconds(0);

        std::chrono::nanoseconds AddTriangle = std::chrono::nanoseconds(0);
        std::chrono::nanoseconds ExportBlock = std::chrono::nanoseconds(0);
        
        std::chrono::nanoseconds SplitBlock = std::chrono::nanoseconds(0);


        std::chrono::nanoseconds EncodeBlocks          = std::chrono::nanoseconds(0);
        std::chrono::nanoseconds GenerateVertexTable   = std::chrono::nanoseconds(0);

        std::chrono::nanoseconds TotalTime;           
        size_t NumBlocks = 0;

    
    };

    // bit manipulation

    inline constexpr size_t BitsNeededSigned(int32_t v)
    {
        uint32_t bits = (uint32_t)(v);
        if (bits & 0x80000000)
            bits = ~bits;

        size_t lz = 32 - std::countl_zero(bits);
        return 1 + lz;
    }
    inline constexpr size_t BitsNeededUnsigned(uint32_t bits)
    {
        size_t lz = 32 - std::countl_zero(bits);
        return lz;
    }

    inline constexpr size_t ReadBits(const uint8_t* bytes, size_t start, size_t len)
    {
        DGF_ASSERT(len <= 48);

        size_t firstByte = (start) / 8;
        size_t lastByte = (start + (len - 1)) / 8;
        size_t numBytes = 1 + lastByte - firstByte;

        uint64_t dst = 0;
        for (size_t i = 0; i < numBytes; i++)
        {
            uint64_t currByte = bytes[firstByte + i];
            dst |= (currByte << (8 * i));
        }

        return (dst >> (start % 8)) & ((1ull << len) - 1);
    }

    inline constexpr void WriteBits(uint8_t* bytes, size_t start, size_t len, size_t value)
    {
        DGF_ASSERT(len <= 48);

        uint64_t mask = ((1ull << len) - 1);
        size_t offset = start % 8;
        value = value << offset;
        mask = mask << offset;

        bytes += (start / 8);
        while (mask)
        {
            size_t currByte = *bytes;
            currByte = (currByte & ~mask) | (value & mask);
            *bytes = (uint8_t)currByte;
            mask = mask >> 8;
            value = value >> 8;
            bytes++;
        }
    }


    template< class TData, int MAX_N, class TKeyFunc >
    inline void RadixSort(TData* data, size_t n, size_t maxKey, TKeyFunc key)
    {
        static_assert(MAX_N <= 65536, "More bits please");
        DGF_ASSERT(n <= MAX_N);

        uint16_t bins[256];
        TData tmp[MAX_N];

        size_t radixIterations = (64 - std::countl_zero(maxKey) + 7) / 8;
        for (size_t r = 0; r < radixIterations; r++)
        {
            size_t shift = 8 * r;

            // bin keys, transfer to tmp buffer
            memset(bins, 0, sizeof(bins));
            for (size_t i = 0; i < n; i++)
            {
                size_t bin = (key(data[i]) >> shift) & 0xFF;
                tmp[i] = data[i];
                bins[bin]++;
            }

            // bin prefix sums
            size_t offset = 0;
            for (size_t i = 0; i < 256; i++)
            {
                size_t count = bins[i];
                bins[i] = (uint16_t) offset;
                offset += count;
            }

            // scatter keys 
            for (size_t i = 0; i < n; i++)
            {
                size_t bin = (key(tmp[i]) >> shift) & 0xFF;
                data[bins[bin]++] = tmp[i];
            }
        }
    }


}
