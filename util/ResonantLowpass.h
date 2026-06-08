#pragma once

#include <algorithm>
#include <cmath>

class ResonantLowpass
{
public:
    float Process(float input, float cutoff_hz, float sample_rate_hz, float q)
    {
        constexpr float pi = 3.14159265358979323846f;
        if (sample_rate_hz <= 0.0f)
            return input;

        const float nyquist = 0.5f * sample_rate_hz;
        const float clamped_cutoff = std::clamp(cutoff_hz, 1.0f, nyquist * 0.95f);
        const float clamped_q = std::max(q, 0.1f);

        const float g = std::tan(pi * clamped_cutoff / sample_rate_hz);
        const float r = 1.0f / clamped_q;
        const float h = 1.0f / (1.0f + r * g + g * g);

        const float hp = (input - (r + g) * ic1eq_ - ic2eq_) * h;
        const float bp = g * hp + ic1eq_;
        const float lp = g * bp + ic2eq_;

        ic1eq_ = g * hp + bp;
        ic2eq_ = g * bp + lp;
        return lp;
    }

    void Reset()
    {
        ic1eq_ = 0.0f;
        ic2eq_ = 0.0f;
    }

private:
    float ic1eq_ = 0.0f;
    float ic2eq_ = 0.0f;
};
