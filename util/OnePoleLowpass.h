#pragma once

#include <algorithm>
#include <cmath>

class OnePoleLowpass
{
public:
    static float AlphaFromCutoff(float cutoff_hz, float sample_rate_hz)
    {
        constexpr float two_pi = 6.28318530717958647692f;
        if (sample_rate_hz <= 0.0f)
            return 1.0f;
        const float clamped_cutoff = std::max(cutoff_hz, 0.0f);
        return 1.0f - std::exp(-two_pi * clamped_cutoff / sample_rate_hz);
    }

    float Process(float input, float alpha)
    {
        const float clamped_alpha = std::clamp(alpha, 0.0f, 1.0f);
        state_ += clamped_alpha * (input - state_);
        return state_;
    }

    void Reset(float value = 0.0f)
    {
        state_ = value;
    }

    float state() const
    {
        return state_;
    }

private:
    float state_ = 0.0f;
};
