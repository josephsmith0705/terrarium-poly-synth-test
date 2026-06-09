#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>

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
    static constexpr size_t harmonic_support_count = 3;
    static constexpr size_t reverse_evidence_max_sources = 8;
    static constexpr float  default_radius   = 0.9975f;
    static constexpr float  default_energy_slew = 0.012f;
    static constexpr size_t inactive_process_divisor = 4;
    static constexpr float  inactive_energy_threshold = 1.0e-4f;
    
    // Neighbor competition tuning
    static constexpr float  neighbor_activity_threshold = 1.0e-5f;
    static constexpr float  neighbor_dominance_deadzone = 0.10f;
    static constexpr float  neighbor_transfer_max_fraction = 0.30f;
    
    // Harmonic suppression tuning
    static constexpr float  harmonic_gate_ratio_low = 0.20f;
    static constexpr float  harmonic_gate_ratio_high = 1.10f;
    static constexpr float  harmonic_support_boost_max = 1.2f;
    static constexpr float  harmonic_reverse_penalty_max = 0.85f;
    static constexpr float  harmonic_suppression_intensity_scale = 10.0f;

    // Parameters for pitch detection that come from user controls
    struct DetectorControls
    {
        float radius = default_radius;
        float neighbor_competition = 0.0f;
        float harmonic_suppression = 0.5f;
        bool bypass_harmonic_suppression = false;
    };

    struct Analysis
    {
        std::array<float, max_fundamentals> fundamentals_hz{};
        size_t                              count = 0;
    };

    void Init(float sample_rate)
    {
        sample_rate_hz = sample_rate;
        process_phase = 0;
        detector_decimation_phase_ = 0;
        analysis_counter_ = 0;
        for (size_t index = 0; index < resonator_count; ++index)
        {
            resonators[index].real_state = 0.0f;
            resonators[index].imaginary_state = 0.0f;
            resonators[index].energy = 0.0f;

            const int midi_note = low_e_midi + static_cast<int>(index);
            resonators[index].frequency = MidiToHz(static_cast<float>(midi_note));
        }

        InitializeHarmonicMaps();

        SetRadius(default_radius);
        SetEnergySlew(default_energy_slew);
    }

    float HarmonicSupportRatio(
        size_t index,
        const std::array<float, resonator_count>& energies) const
    {
        float weighted_harmonic_energy = 0.0f;
        float total_harmonic_weight = 0.0f;
        for (size_t harmonic_index = 0;
             harmonic_index < harmonic_support_count;
             ++harmonic_index)
        {
            const size_t mapped_index = harmonic_map_[index][harmonic_index];
            if (mapped_index >= resonator_count)
                continue;

            const float harmonic_weight = harmonic_weights_[harmonic_index];
            weighted_harmonic_energy += energies[mapped_index] * harmonic_weight;
            total_harmonic_weight += harmonic_weight;
        }

        if (total_harmonic_weight <= 1.0e-9f)
            return 0.0f;

        const float normalized_support = weighted_harmonic_energy / total_harmonic_weight;
        return normalized_support / (energies[index] + 1.0e-9f);
    }

    float HarmonicReverseRatio(
        size_t index,
        const std::array<float, resonator_count>& energies) const
    {
        float weighted_fundamental_energy = 0.0f;
        float total_weight = 0.0f;
        for (const auto& reverse_source : reverse_harmonic_map_[index])
        {
            if (reverse_source.fundamental_index >= resonator_count)
                continue;

            weighted_fundamental_energy +=
                energies[reverse_source.fundamental_index] * reverse_source.weight;
            total_weight += reverse_source.weight;
        }

        if (total_weight <= 1.0e-9f)
            return 0.0f;

        const float normalized_reverse_energy = weighted_fundamental_energy / total_weight;
        return normalized_reverse_energy / (energies[index] + 1.0e-9f);
    }

    void ConfigureAnalysisScheduler(size_t detector_decimation_factor, int analysis_period_samples)
    {
        detector_decimation_factor_ = std::max<size_t>(detector_decimation_factor, 1);
        analysis_period_samples_ = std::max(analysis_period_samples, 1);
        detector_decimation_phase_ = 0;
        analysis_counter_ = 0;
    }

    template <typename AnalysisCallback>
    void ProcessInputSample(float input, AnalysisCallback&& on_analysis)
    {
        detector_decimation_phase_ = (detector_decimation_phase_ + 1) % detector_decimation_factor_;
        if (detector_decimation_phase_ != 0)
            return;

        Process(input);
        ++analysis_counter_;
        if (analysis_counter_ >= analysis_period_samples_)
        {
            analysis_counter_ = 0;
            std::forward<AnalysisCallback>(on_analysis)();
        }
    }

    template <size_t max_synth_voices>
    void ProcessInputSample(
        float input,
        const DetectorControls& controls,
        std::array<size_t, max_synth_voices>& active_voice_indices_out,
        size_t& active_voice_count_out)
    {
        detector_decimation_phase_ = (detector_decimation_phase_ + 1) % detector_decimation_factor_;
        if (detector_decimation_phase_ != 0)
            return;

        Process(input);
        ++analysis_counter_;
        if (analysis_counter_ >= analysis_period_samples_)
        {
            analysis_counter_ = 0;
            UpdateVoiceAmplitudesFromDetector(
                controls,
                active_voice_indices_out,
                active_voice_count_out);
        }
    }

    template <size_t max_synth_voices>
    void UpdateVoiceAmplitudesFromDetector(
        const DetectorControls& controls,
        std::array<size_t, max_synth_voices>& active_voice_indices_out,
        size_t& active_voice_count_out) const
    {
        std::array<float, resonator_count> energies{};
        std::array<float, resonator_count> competed_energies{};
        std::array<bool, resonator_count> initially_selected{};
        std::array<float, max_synth_voices> initial_top_energies{};
        std::array<size_t, max_synth_voices> initial_top_indices{};
        std::array<float, max_synth_voices> final_top_energies{};
        std::array<size_t, max_synth_voices> final_top_indices{};
        initial_top_indices.fill(resonator_count);
        final_top_indices.fill(resonator_count);

        for (size_t resonator_index = 0;
             resonator_index < resonator_count;
             ++resonator_index)
        {
            energies[resonator_index] = resonators[resonator_index].energy;
            competed_energies[resonator_index] = energies[resonator_index];
            RankResonatorEnergy(
                energies[resonator_index],
                resonator_index,
                initial_top_energies,
                initial_top_indices);
        }

        for (size_t rank = 0; rank < max_synth_voices; ++rank)
        {
            const size_t resonator_index = initial_top_indices[rank];
            if (resonator_index >= resonator_count)
                continue;
            initially_selected[resonator_index] = true;
        }

        if (!controls.bypass_harmonic_suppression)
        {
            for (size_t resonator_index = 0;
                 resonator_index < resonator_count;
                 ++resonator_index)
            {
                if (!initially_selected[resonator_index])
                    continue;

                const float support_ratio =
                    HarmonicSupportRatio(resonator_index, energies);
                const float reverse_ratio =
                    HarmonicReverseRatio(resonator_index, energies);
                const float support_weight = std::clamp(
                    (support_ratio - harmonic_gate_ratio_low)
                        / (harmonic_gate_ratio_high - harmonic_gate_ratio_low),
                    0.0f,
                    1.0f);
                const float reverse_weight = std::clamp(
                    (reverse_ratio - harmonic_gate_ratio_low)
                        / (harmonic_gate_ratio_high - harmonic_gate_ratio_low),
                    0.0f,
                    1.0f);
                const float suppression_amount =
                    controls.harmonic_suppression * harmonic_suppression_intensity_scale;

                const float support_gain =
                    1.0f + suppression_amount * harmonic_support_boost_max * support_weight;
                const float reverse_penalty = std::max(
                    1.0f - suppression_amount * harmonic_reverse_penalty_max * reverse_weight,
                    0.1f);
                competed_energies[resonator_index] =
                    std::max(energies[resonator_index] * support_gain * reverse_penalty, 0.0f);
            }
        }

        if (controls.neighbor_competition > 0.0f)
        {
            for (size_t resonator_index = 0;
                 resonator_index + 1 < resonator_count;
                 ++resonator_index)
            {
                if (!(initially_selected[resonator_index]
                      && initially_selected[resonator_index + 1]))
                {
                    continue;
                }

                const float left_energy = competed_energies[resonator_index];
                const float right_energy = competed_energies[resonator_index + 1];
                if (left_energy < neighbor_activity_threshold
                    || right_energy < neighbor_activity_threshold)
                {
                    continue;
                }

                const float pair_energy = left_energy + right_energy;
                if (pair_energy <= 1.0e-9f)
                    continue;

                const float dominance = (left_energy - right_energy) / pair_energy;
                const float abs_dominance = std::abs(dominance);
                if (abs_dominance <= neighbor_dominance_deadzone)
                    continue;

                const float dominance_strength =
                    (abs_dominance - neighbor_dominance_deadzone)
                    / (1.0f - neighbor_dominance_deadzone);

                const float transfer_amount = controls.neighbor_competition
                                            * dominance_strength
                                            * (std::min(left_energy, right_energy)
                                               * neighbor_transfer_max_fraction);
                if (left_energy > right_energy)
                {
                    competed_energies[resonator_index] += transfer_amount;
                    competed_energies[resonator_index + 1] -= transfer_amount;
                }
                else if (right_energy > left_energy)
                {
                    competed_energies[resonator_index] -= transfer_amount;
                    competed_energies[resonator_index + 1] += transfer_amount;
                }

                competed_energies[resonator_index] = std::max(competed_energies[resonator_index], 0.0f);
                competed_energies[resonator_index + 1] = std::max(competed_energies[resonator_index + 1], 0.0f);
            }
        }

        for (size_t resonator_index = 0;
             resonator_index < resonator_count;
             ++resonator_index)
        {
            RankResonatorEnergy(
                competed_energies[resonator_index],
                resonator_index,
                final_top_energies,
                final_top_indices);
        }

        active_voice_count_out = 0;
        for (size_t rank = 0; rank < max_synth_voices; ++rank)
        {
            const size_t resonator_index = final_top_indices[rank];
            if (resonator_index >= resonator_count)
                continue;

            if (competed_energies[resonator_index] <= 0.0f)
                continue;

            active_voice_indices_out[active_voice_count_out++] = resonator_index;
        }
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
    struct ReverseEvidenceSource
    {
        size_t fundamental_index = resonator_count;
        float weight = 0.0f;
    };

    static constexpr std::array<float, harmonic_support_count> harmonic_multipliers_{
        2.0f,
        3.0f,
        4.0f,
    };

    static constexpr std::array<float, harmonic_support_count> harmonic_weights_{
        1.0f,
        0.7f,
        0.45f,
    };

    static void InsertReverseEvidenceSource(
        std::array<ReverseEvidenceSource, reverse_evidence_max_sources>& reverse_sources,
        size_t fundamental_index,
        float weight)
    {
        for (auto& reverse_source : reverse_sources)
        {
            if (reverse_source.fundamental_index < resonator_count)
                continue;

            reverse_source.fundamental_index = fundamental_index;
            reverse_source.weight = weight;
            return;
        }

        size_t weakest_index = 0;
        for (size_t source_index = 1;
             source_index < reverse_evidence_max_sources;
             ++source_index)
        {
            if (reverse_sources[source_index].weight
                < reverse_sources[weakest_index].weight)
            {
                weakest_index = source_index;
            }
        }

        if (weight > reverse_sources[weakest_index].weight)
        {
            reverse_sources[weakest_index].fundamental_index = fundamental_index;
            reverse_sources[weakest_index].weight = weight;
        }
    }

    void InitializeHarmonicMaps()
    {
        for (auto& reverse_sources : reverse_harmonic_map_)
        {
            for (auto& reverse_source : reverse_sources)
                reverse_source = {};
        }

        for (size_t index = 0; index < resonator_count; ++index)
        {
            for (size_t harmonic_index = 0;
                 harmonic_index < harmonic_support_count;
                 ++harmonic_index)
            {
                const float harmonic_frequency =
                    resonators[index].frequency * harmonic_multipliers_[harmonic_index];
                const size_t mapped_index =
                    harmonic_frequency > resonators.back().frequency
                        ? resonator_count
                        : FrequencyToIndex(harmonic_frequency);
                harmonic_map_[index][harmonic_index] = mapped_index;

                if (mapped_index >= resonator_count)
                    continue;

                InsertReverseEvidenceSource(
                    reverse_harmonic_map_[mapped_index],
                    index,
                    harmonic_weights_[harmonic_index]);
            }
        }
    }

    template <size_t max_synth_voices>
    static void RankResonatorEnergy(
        float energy,
        size_t resonator_index,
        std::array<float, max_synth_voices>& top_energies,
        std::array<size_t, max_synth_voices>& top_indices)
    {
        for (size_t rank = 0; rank < max_synth_voices; ++rank)
        {
            if (energy <= top_energies[rank])
                continue;

            for (size_t shift = max_synth_voices - 1; shift > rank; --shift)
            {
                top_energies[shift] = top_energies[shift - 1];
                top_indices[shift] = top_indices[shift - 1];
            }

            top_energies[rank] = energy;
            top_indices[rank] = resonator_index;
            return;
        }
    }

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
    std::array<std::array<size_t, harmonic_support_count>, resonator_count> harmonic_map_{};
    std::array<std::array<ReverseEvidenceSource, reverse_evidence_max_sources>, resonator_count>
        reverse_harmonic_map_{};
    float sample_rate_hz = 48000.0f;
    float current_radius = default_radius;
    float energy_slew_factor = default_energy_slew;
    size_t process_phase = 0;
    size_t detector_decimation_factor_ = 1;
    size_t detector_decimation_phase_ = 0;
    int analysis_period_samples_ = 1;
    int analysis_counter_ = 0;
};