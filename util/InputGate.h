#pragma once

#include <algorithm>
#include <cmath>

struct InputGateConfig
{
    float open_threshold = 0.012f;
    float close_threshold = 0.008f;
    float detector_attack_ms = 1.5f;
    float detector_release_ms = 35.0f;
    float hold_ms = 35.0f;
};

class InputGate
{
public:
    bool Process(float absolute_input, float sample_rate_hz, const InputGateConfig& config)
    {
        const float attack_alpha = AlphaFromMs(config.detector_attack_ms, sample_rate_hz);
        const float release_alpha = AlphaFromMs(config.detector_release_ms, sample_rate_hz);
        const float detector_alpha = absolute_input > detector_level_ ? attack_alpha : release_alpha;
        detector_level_ += detector_alpha * (absolute_input - detector_level_);

        const int hold_samples = std::max(
            1,
            static_cast<int>(std::max(config.hold_ms, 0.0f) * 0.001f * std::max(sample_rate_hz, 1.0f)));

        if (is_open_)
        {
            if (detector_level_ >= config.open_threshold)
            {
                hold_counter_samples_ = hold_samples;
            }
            else if (detector_level_ < config.close_threshold)
            {
                if (hold_counter_samples_ > 0)
                    --hold_counter_samples_;
                if (hold_counter_samples_ <= 0)
                    is_open_ = false;
            }
        }
        else if (detector_level_ > config.open_threshold)
        {
            is_open_ = true;
            hold_counter_samples_ = hold_samples;
        }

        return is_open_;
    }

    void Reset(bool open = true)
    {
        is_open_ = open;
        detector_level_ = 0.0f;
        hold_counter_samples_ = 0;
    }

    bool is_open() const
    {
        return is_open_;
    }

private:
    static float AlphaFromMs(float time_ms, float sample_rate_hz)
    {
        const float clamped_time_ms = std::max(time_ms, 0.0f);
        if (sample_rate_hz <= 0.0f)
            return 1.0f;
        const float tau_samples = std::max(clamped_time_ms * 0.001f * sample_rate_hz, 1.0f);
        return 1.0f - std::exp(-1.0f / tau_samples);
    }

    bool is_open_ = true;
    float detector_level_ = 0.0f;
    int hold_counter_samples_ = 0;
};
