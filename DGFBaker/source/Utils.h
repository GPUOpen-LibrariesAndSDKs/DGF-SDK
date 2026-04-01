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

namespace DGFBaker
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
    typedef vec3<double> double3;

    inline int3 operator>>(const int3& lhs, size_t shift)
    {
        return { lhs.x >> shift, lhs.y >> shift, lhs.z >> shift };
    }

    inline int3 operator<<(const int3& lhs, size_t shift)
    {
        return { lhs.x << shift, lhs.y << shift, lhs.z << shift };
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
        std::chrono::nanoseconds ExponentSelect        = std::chrono::nanoseconds(0);
        std::chrono::nanoseconds TrianglePacking       = std::chrono::nanoseconds(0);
        std::chrono::nanoseconds Cleanup               = std::chrono::nanoseconds(0);
        std::chrono::nanoseconds TotalTime;               
    };

    // bit manipulation

    constexpr inline size_t RoundUpPow2(size_t n, size_t m)
    {
        return (n + (m - 1)) & ~(m - 1);
    }

    inline constexpr size_t BitSizeToByteSize(size_t n)
    {
        return RoundUpPow2(n, 8) / 8;
    }

    inline constexpr size_t BitsNeededSigned(int32_t v)
    {
        uint32_t bits = (uint32_t)(v);
        if (bits & 0x80000000)
            bits = ~bits;

        size_t lz = 32 - std::countl_zero(bits);
        return 1 + lz;
    }
    template< class TInt >
    inline constexpr size_t BitsNeededUnsigned(TInt bits)
    {
        typedef typename std::make_unsigned<TInt>::type TUnsigned;
        size_t lz = (8*sizeof(TInt)) - std::countl_zero((TUnsigned)bits);
        return lz;
    }    

    template< class TInt >
    inline constexpr size_t BytesNeededUnsigned(TInt bits)
    {
        size_t bitSize = BitsNeededUnsigned(bits);
        return RoundUpPow2(bitSize, 8) / 8;
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

    template<typename S>
    inline typename std::make_unsigned<S>::type ZigZagEncode(S s)
    {
        static_assert(std::is_signed<S>::value, "Type must be signed");
        using U = typename std::make_unsigned<S>::type;

        const auto u2 = static_cast<U>(s) << 1;
        const auto n = static_cast<U>(s >> (std::numeric_limits<U>::digits - 1));
        return u2 ^ n;
    }

    template<typename U>
    inline typename std::make_signed<U>::type ZigZagDecode(U u)
    {
        static_assert(std::is_unsigned<U>::value, "Type must be unsigned");
        using S = typename std::make_signed<U>::type;

        const auto a = static_cast<S>(u >> 1);
        const auto n = static_cast<S>(u & 1);
        return a ^ (-n);
    }

#ifndef DGFBaker_CHECKED_TRUNCATION
#ifdef _DEBUG
#define DGFBaker_CHECKED_TRUNCATION
#endif
#endif

    template< class TTo>
    inline TTo uint_truncate(size_t src)
    {
        static_assert(std::is_unsigned<TTo>::value);
#ifdef DGFBaker_CHECKED_TRUNCATION
        DGF_ASSERT(src <= std::numeric_limits<TTo>::max());
#endif
        return static_cast<TTo>(src);
    }

#ifndef DGFBaker_CHECKED_BYTE_SPANS
#ifdef _DEBUG
#define DGFBaker_CHECKED_BYTE_SPANS
#endif
#endif

#ifdef DGFBaker_CHECKED_BYTE_SPANS
    
    typedef std::span<uint8_t> ByteSpan;
    typedef std::span<const uint8_t> ConstByteSpan;

    inline ByteSpan MakeByteSpan(uint8_t* buffer, size_t size)
    {
        return ByteSpan(buffer, size);
    }
    inline ConstByteSpan MakeByteSpan(const uint8_t* buffer, size_t size)
    {
        return ConstByteSpan(buffer, size);
    }
    inline bool IsByteSpanEmpty(ConstByteSpan span)
    {
        return span.size() == 0;
    }   
    template<class T> T* ByteSpanCast(ByteSpan span, size_t offset=0, size_t arraySize=1)
    {
        DGF_ASSERT(offset + sizeof(T)*arraySize < span.size());
        return reinterpret_cast<T*>(span.data() + offset);
    }
    template<class T> const T* ByteSpanCast(ConstByteSpan span, size_t offset=0, size_t arraySize = 1)
    {
        DGF_ASSERT(offset + sizeof(T) * arraySize < span.size());
        return reinterpret_cast<const T*>(span.data() + offset);
    }
    inline ByteSpan IncrementByteSpan(ByteSpan span, size_t incr)
    {
        return span.subspan(incr, span.size() - incr);
    }
    inline ConstByteSpan IncrementByteSpan(ConstByteSpan span, size_t incr)
    {
        return span.subspan(incr, span.size() - incr);
    }
#else
    typedef uint8_t* ByteSpan;
    typedef const uint8_t* ConstByteSpan;
    inline ByteSpan MakeByteSpan(uint8_t* buffer, size_t size)
    {
        return (size > 0) ? buffer : nullptr;
    }
    inline ConstByteSpan MakeByteSpan(const uint8_t* buffer, size_t size)
    {
        return (size > 0) ? buffer : nullptr;
    }
    inline bool IsByteSpanEmpty(ConstByteSpan span)
    {
        return span == nullptr;
    }      
    template<class T> T* ByteSpanCast(ByteSpan span, size_t offset = 0, size_t arraySize = 1)
    {
        return reinterpret_cast<T*>(span + offset);
    }
    template<class T> const T* ByteSpanCast(ConstByteSpan span, size_t offset = 0, size_t arraySize = 1)
    {
        return reinterpret_cast<const T*>(span + offset);
    }
    inline ByteSpan IncrementByteSpan(ByteSpan span, size_t incr)
    {
        return span + incr;
    }
    inline ConstByteSpan IncrementByteSpan(ConstByteSpan span, size_t incr)
    {
        return span + incr;
    }
#endif

    inline bool ReadBit(const ConstByteSpan stream, size_t index)
    {
        return (stream[index / 8] >> (index % 8)) & 1;
    }

    inline uint64_t ReadBits(const ConstByteSpan bytes, size_t start, size_t len)
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

    inline void WriteBits(ByteSpan bytes, size_t start, size_t len, uint64_t value)
    {
        DGF_ASSERT(len <= 48);

        uint64_t mask = ((1ull << len) - 1);
        size_t bitOffset = start % 8;
        value = value << bitOffset;
        mask  = mask << bitOffset;

        size_t byteOffset = (start / 8);
        while (mask)
        {
            size_t currByte = bytes[byteOffset];
            currByte = (currByte & ~mask) | (value & mask);
            bytes[byteOffset] = (uint8_t)currByte;
            mask = mask >> 8;
            value = value >> 8;
            byteOffset++;
        }
    }

    inline void WriteBit(ByteSpan bytes, size_t pos, size_t value)
    {
        uint8_t bit = (uint8_t)(value << (pos % 8));
        bytes[pos / 8] |= bit;
    }
}
