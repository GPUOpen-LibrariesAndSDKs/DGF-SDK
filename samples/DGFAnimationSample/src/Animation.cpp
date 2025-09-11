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

#include "Animation.h"

using namespace Animation;

AnimationHelper::AnimationHelper(const size_t keyFrameCount, const Config& config)
    : m_config(config), m_keyFrameCount(keyFrameCount)
{
    Reset();
}

void AnimationHelper::Begin()
{
    m_startTime += (std::chrono::steady_clock::now() - m_stopTime);
    m_stopTime = {};
    m_enabled  = true;
}

void AnimationHelper::Pause()
{
    m_stopTime = std::chrono::steady_clock::now();
    m_enabled  = false;
}

void AnimationHelper::Reset()
{
    m_stopTime  = {};
    m_startTime = std::chrono::steady_clock::now();
}

void AnimationHelper::SetPlaybackRate(const float playbackRate)
{
    if (playbackRate == 0) {
        return;
    }
    m_config.playbackRate = playbackRate;
}

void AnimationHelper::SetLoop(const bool looping)
{
    m_config.looping = looping;
}

double AnimationHelper::GetInterpolationFactor() const
{
    const double index = (GetSecondsPassed() * m_keyFramesPerSecond) / m_config.playbackRate;
    return index - static_cast<int64_t>(index);
}

double AnimationHelper::GetSecondsPassed() const
{
    const auto currentTime = std::chrono::steady_clock::now();

    if (m_enabled) {
        return std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - m_startTime).count();
    } else {
        return std::chrono::duration_cast<std::chrono::duration<double>>(m_stopTime - m_startTime).count();
    }
}

int32_t AnimationHelper::GetKeyFrameSample() const
{
    return static_cast<int32_t>((GetSecondsPassed() * m_keyFramesPerSecond) / m_config.playbackRate);
}

bool AnimationHelper::IsEnabled() const
{
    return m_enabled;
}

KeyFrameInfo AnimationHelper::GetKeyFrameInfo()
{
    KeyFrameInfo info = {};

    if (m_keyFrameCount <= 0) {
        return info;
    }

    if (m_debugMode) {
        info.interpolationFactor = 0.5;
        info.currentIndex        = static_cast<KeyFrameIndex>(m_counter % m_keyFrameCount);
        info.nextIndex           = static_cast<KeyFrameIndex>((m_counter + 1) % m_keyFrameCount);
        if (m_enabled) {
            m_counter++;
        }
        return info;
    }

    const int32_t sample     = GetKeyFrameSample();
    info.interpolationFactor = GetInterpolationFactor();
    const auto    isReversed = sample < 0;
    KeyFrameIndex current, next = 0;

    if (m_config.looping) {
        current = static_cast<KeyFrameIndex>(sample % m_keyFrameCount);
        next    = static_cast<KeyFrameIndex>((sample + (isReversed ? -1 : +1)) % m_keyFrameCount);

    } else {
        if (std::abs(sample) >= m_keyFrameCount) {
            info.currentIndex        = static_cast<KeyFrameIndex>(isReversed ? 0 : m_keyFrameCount - 1);
            info.nextIndex           = info.currentIndex;
            info.interpolationFactor = 0;

            return info;
        }

        if (sample < 0) {
            current = static_cast<KeyFrameIndex>(m_keyFrameCount + sample);
            next    = (current - 1) < 0 ? 0 : current - 1;

        } else {
            current = static_cast<KeyFrameIndex>(sample);
            next    = static_cast<KeyFrameIndex>((current + 1) >= m_keyFrameCount ? m_keyFrameCount - 1 : current + 1);
        }
    }
    info.currentIndex = current;
    info.nextIndex    = next;

    return info;
}