#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

class PolyPitchDetector
{
public:
    static constexpr size_t max_fundamentals = 1;
    static constexpr float  two_pi           = 6.28318530717958647692f;
    static constexpr float  a_string_hz      = 110.0f;
    static constexpr float  low_e_hz         = 82.4069f;   // E2
    static constexpr float  high_e_plus_two_octaves_hz = 1318.510f;  // E6
    static constexpr int    low_e_midi       = 40;         // E2
    static constexpr int    high_e_plus_two_octaves_midi = 88;  // D#4 / bottom 24 notes
    static constexpr size_t resonator_count =
        static_cast<size_t>(high_e_plus_two_octaves_midi - low_e_midi + 1);
    static constexpr size_t a_string_index = static_cast<size_t>(45 - low_e_midi);  // A2
    static constexpr float  default_radius   = 0.9975f;
    static constexpr float  default_energy_slew = 0.012f;
    static constexpr size_t inactive_process_divisor = 4;
    static constexpr float  inactive_energy_threshold = 1.0e-4f;

    struct Analysis
    {
        std::array<float, max_fundamentals> fundamentals_hz{};
        size_t                              count = 0;
    };

    void Init(float sample_rate)
    {
        sample_rate_hz = sample_rate;
        process_phase = 0;
        for (size_t index = 0; index < resonator_count; ++index)
        {
            resonators[index].real_state = 0.0f;
            resonators[index].imaginary_state = 0.0f;
            resonators[index].energy = 0.0f;

            const int midi_note = low_e_midi + static_cast<int>(index);
            resonators[index].frequency = MidiToHz(static_cast<float>(midi_note));
        }

        SetRadius(default_radius);
        SetEnergySlew(default_energy_slew);
    }

    void SetRadius(float radius)
    {
        current_radius = radius;
        for (auto& resonator : resonators)
        {
            const float resonator_angle = two_pi * resonator.frequency / sample_rate_hz;
            resonator.rotation_cosine = current_radius * std::cos(resonator_angle);
            resonator.rotation_sine = current_radius * std::sin(resonator_angle);
        }
    }

    void SetEnergySlew(float energy_slew)
    {
        energy_slew_factor = energy_slew;
    }

    void Process(float input)
    {
        for (size_t index = 0; index < resonator_count; ++index)
        {
            auto& resonator = resonators[index];
            const bool is_active = resonator.energy >= inactive_energy_threshold;
            const bool scheduled_this_sample = ((index + process_phase) % inactive_process_divisor) == 0;

            if (is_active || scheduled_this_sample)
            {
                ProcessResonator(resonator, input);
            }
            else
            {
                // Cheap decay while skipped keeps dormant bins settling to zero.
                resonator.energy += energy_slew_factor * (0.0f - resonator.energy);
            }
        }

        process_phase = (process_phase + 1) % inactive_process_divisor;
    }

    Analysis Analyze(float threshold) const
    {
        Analysis analysis{};

        const Resonator* best_resonator = &resonators[a_string_index];

        for (const auto& resonator : resonators)
        {
            if (resonator.energy > best_resonator->energy)
                best_resonator = &resonator;
        }
        

        if (best_resonator->energy >= threshold)
        {
            analysis.count = 1;
            analysis.fundamentals_hz[0] = best_resonator->frequency;
        }

        return analysis;
    }

    float GetEnergyAt(float hz) const
    {
        const size_t index = FrequencyToIndex(hz);
        return resonators[index].energy;
    }

    float GetEnergyAtIndex(size_t index) const
    {
        return resonators[index].energy;
    }

    float GetFrequencyAtIndex(size_t index) const
    {
        return resonators[index].frequency;
    }

    float GetRadius() const { return current_radius; }

private:
    struct Resonator
    {
        float real_state = 0.0f;
        float imaginary_state = 0.0f;
        float rotation_cosine = 0.0f;
        float rotation_sine = 0.0f;
        float frequency = 0.0f;
        float energy = 0.0f;
    };

    static float MidiToHz(float midi_note)
    {
        return 440.0f * std::pow(2.0f, (midi_note - 69.0f) / 12.0f);
    }

    static float HzToMidi(float hz)
    {
        return 69.0f + 12.0f * std::log2(hz / 440.0f);
    }

    size_t FrequencyToIndex(float hz) const
    {
        const int midi_note = static_cast<int>(std::round(HzToMidi(hz)));
        const int clamped_midi = std::clamp(
            midi_note,
            low_e_midi,
            high_e_plus_two_octaves_midi);
        return static_cast<size_t>(clamped_midi - low_e_midi);
    }

    void ProcessResonator(Resonator& resonator, float input)
    {
        const float previous_real_state = resonator.real_state;
        const float previous_imaginary_state = resonator.imaginary_state;

        resonator.real_state = input
            + resonator.rotation_cosine * previous_real_state
            - resonator.rotation_sine * previous_imaginary_state;

        resonator.imaginary_state = resonator.rotation_sine * previous_real_state
            + resonator.rotation_cosine * previous_imaginary_state;

        const float instantaneous_energy = resonator.real_state * resonator.real_state
            + resonator.imaginary_state * resonator.imaginary_state;

        resonator.energy += energy_slew_factor * (instantaneous_energy - resonator.energy);
    }

    std::array<Resonator, resonator_count> resonators{};
    float sample_rate_hz = 48000.0f;
    float current_radius = default_radius;
    float energy_slew_factor = default_energy_slew;
    size_t process_phase = 0;
};