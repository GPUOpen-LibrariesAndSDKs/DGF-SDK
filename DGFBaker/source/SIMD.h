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

#include <xmmintrin.h>
#include <emmintrin.h>

#include "Utils.h"

namespace DGF
{
    class SIMDVeci;

    class SIMDVecf
    {
    public:
        SIMDVecf() {}
        SIMDVecf(const float3& v3) : m_v(_mm_setr_ps(v3.x, v3.y, v3.z, 0)) {}
        SIMDVecf(__m128 v) : m_v(v) {}
        SIMDVecf(float v) : m_v(_mm_set_ps1(v)) {}

        static SIMDVecf AlignedLoad(float* v) { return SIMDVecf(_mm_load_ps(v)); }

        SIMDVecf operator+(SIMDVecf rhs) const
        {
            return SIMDVecf(_mm_add_ps(m_v, rhs.m_v));
        }
        SIMDVecf operator-(SIMDVecf rhs) const
        {
            return SIMDVecf(_mm_sub_ps(m_v, rhs.m_v));
        }
        SIMDVecf operator*(float rhs) const
        {
            return SIMDVecf(_mm_mul_ps(m_v, _mm_set1_ps(rhs)));
        }
        SIMDVecf operator*(SIMDVecf rhs) const
        {
            return SIMDVecf(_mm_mul_ps(m_v, rhs.m_v));
        }

    private:
        friend class SIMDVeci;
        friend class SIMDBox;
        friend SIMDVeci RoundToInt(SIMDVecf);
        friend SIMDVecf min(SIMDVecf a, SIMDVecf b);
        friend SIMDVecf max(SIMDVecf a, SIMDVecf b);
        __m128 m_v;
    };

    inline SIMDVecf min(SIMDVecf a, SIMDVecf b)
    {
        return SIMDVecf(_mm_min_ps(a.m_v, b.m_v));
    }
    inline SIMDVecf max(SIMDVecf a, SIMDVecf b)
    {
        return SIMDVecf(_mm_max_ps(a.m_v, b.m_v));
    }

    class SIMDVeci
    {
    public:
        SIMDVeci(SIMDVecf v) : m_v(_mm_cvttps_epi32(v.m_v)) {}
        SIMDVeci(__m128i v) : m_v(v) {}
        inline uint32_t X() { return _mm_cvtsi128_si32(m_v); }
        inline uint32_t Y() { return _mm_cvtsi128_si32(_mm_shuffle_epi32(m_v, _MM_SHUFFLE(1, 1, 1, 1))); }
        inline uint32_t Z() { return _mm_cvtsi128_si32(_mm_shuffle_epi32(m_v, _MM_SHUFFLE(2, 2, 2, 2))); }

        inline void AlignedStore(int* addr)
        {
            _mm_store_si128((__m128i*) addr,m_v);
        }

    private:
        __m128i m_v;
    };

    inline SIMDVeci RoundToInt(SIMDVecf v)
    {
        return SIMDVeci(_mm_cvtps_epi32(v.m_v));
    }

    inline int RoundToInt(float f)
    {
        return _mm_cvtss_si32(_mm_set_ss(f));
    }

    class SIMDBox
    {
    public:
        SIMDBox()
            : m_min(_mm_set_ps1(std::numeric_limits<float>::infinity())),
            m_max(_mm_set_ps1(-std::numeric_limits<float>::infinity()))
        {
        }
        SIMDBox(SIMDVecf min, SIMDVecf max) : m_min(min.m_v), m_max(max.m_v)
        {
        }
        SIMDBox(AABB box)
            : m_min(_mm_setr_ps(box.Min.x, box.Min.y, box.Min.z, 0)),
            m_max(_mm_setr_ps(box.Max.x, box.Max.y, box.Max.z, 0))
        {
        }

        SIMDBox(float3 min, float3 max)
            : m_min(_mm_setr_ps(min.x, min.y, min.z, 0)),
            m_max(_mm_setr_ps(max.x, max.y, max.z, 0))
        {

        }
        void Expand(SIMDBox b)
        {
            m_min = _mm_min_ps(b.m_min, m_min);
            m_max = _mm_max_ps(b.m_max, m_max);
        }

        void Expand(SIMDVecf p)
        {
            m_min = _mm_min_ps(p.m_v, m_min);
            m_max = _mm_max_ps(p.m_v, m_max);
        }

        float HalfArea()
        {
            __m128 d = _mm_sub_ps(m_max, m_min);
            float dx = _mm_cvtss_f32(d);
            float dy = _mm_cvtss_f32(_mm_shuffle_ps(d, d, _MM_SHUFFLE(1, 1, 1, 1)));
            float dz = _mm_cvtss_f32(_mm_shuffle_ps(d, d, _MM_SHUFFLE(2, 2, 2, 2)));
            return dx * (dy + dz) + dy * dz;
        }

        SIMDVecf Centroid()
        {
            return _mm_mul_ps(_mm_add_ps(m_min, m_max), _mm_set1_ps(0.5f));
        }

        AABB ToAABB()
        {
            __m128 vmin = m_min;
            __m128 vmax = m_max;
            float3 min;
            min.x = _mm_cvtss_f32(vmin);
            min.y = _mm_cvtss_f32(_mm_shuffle_ps(vmin, vmin, _MM_SHUFFLE(1, 1, 1, 1)));
            min.z = _mm_cvtss_f32(_mm_shuffle_ps(vmin, vmin, _MM_SHUFFLE(2, 2, 2, 2)));
            float3 max;
            max.x = _mm_cvtss_f32(vmax);
            max.y = _mm_cvtss_f32(_mm_shuffle_ps(vmax, vmax, _MM_SHUFFLE(1, 1, 1, 1)));
            max.z = _mm_cvtss_f32(_mm_shuffle_ps(vmax, vmax, _MM_SHUFFLE(2, 2, 2, 2)));
            return AABB{ min,max };
        }

        float Min(size_t j) const
        {
            return reinterpret_cast<const float*>(&m_min)[j];
        }
        float Max(size_t j) const
        {
            return reinterpret_cast<const float*>(&m_max)[j];
        }


        __m128 m_min;
        __m128 m_max;
    };


}