// Copyright (c) 2024 Advanced Micro Devices, Inc.All rights reserved.
#pragma once

#include <filesystem>
#include <chrono>
#include <unordered_set>

namespace Animation {
    struct Config {
        float playbackRate        = 1.0f;
        bool  looping             = true;
        bool  linearInterpolation = true;
    };

    typedef uint32_t KeyFrameIndex;

    struct KeyFrameInfo {
        KeyFrameIndex currentIndex        = static_cast<KeyFrameIndex>(-1);
        KeyFrameIndex nextIndex           = static_cast<KeyFrameIndex>(-1);
        double        interpolationFactor = 0;
    };

    class AnimationHelper {
    public:
        AnimationHelper() {}
        AnimationHelper(const size_t keyFrameCount, const Config& config);

        void Begin();
        void Pause();
        void Reset();

        void SetPlaybackRate(float playbackRate);
        void SetLoop(bool looping);

        double GetInterpolationFactor() const;

        KeyFrameInfo GetKeyFrameInfo();
        bool         IsEnabled() const;

    private:
        double  GetSecondsPassed() const;
        int32_t GetKeyFrameSample() const;

        size_t m_keyFrameCount = 0;

        std::chrono::steady_clock::time_point m_startTime = {};
        std::chrono::steady_clock::time_point m_stopTime  = {};

        Config       m_config             = {};
        bool         m_enabled            = true;
        const size_t m_keyFramesPerSecond = 30;

        bool   m_debugMode = false;
        size_t m_counter   = 0;
    };

}  // namespace Animation