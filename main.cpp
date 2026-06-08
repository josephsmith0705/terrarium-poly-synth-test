// Terrarium Poly Synth — Single-Resonator Trigger + Square Wave
//
// Based on: Zhou, Reiss, Mattavelli, Zoia —
//   "A Computationally Efficient Method for Polyphonic Pitch Estimation"
//
// Architecture:
//   Per-sample:        run one resonator tuned to guitar A string (110 Hz)
//   Every N samples:   check resonator energy against threshold
//   Synthesis:         output a fixed-amplitude square wave at 110 Hz

#include <algorithm>
#include <array>
#include <cmath>

#include <daisy_seed.h>

#include <util/InputGate.h>
#include <util/OnePoleLowpass.h>
#include <util/PolyPitchDetector.h>
#include <util/Terrarium.h>

namespace
{
Terrarium terrarium;

constexpr float  two_pi           = 6.28318530717958647692f;
constexpr float  input_highpass_cutoff_hz = 70.0f;
constexpr float  input_lowpass_cutoff_hz  = 380.0f;
constexpr float  energy_to_volume_curve = 1.8f;
constexpr size_t detector_decimation_factor = 2;
constexpr int    analysis_period_detector_samples = 128;
constexpr float  synth_lowpass_cutoff_min_hz = 350.0f;
constexpr float  synth_lowpass_cutoff_max_hz = 10000.0f;
constexpr float  square_amplitude = 0.85f;
constexpr float  synth_output_gain = 2.0f;
constexpr size_t max_synth_voices = 12;
constexpr size_t harmonic_support_count = 3;
constexpr size_t reverse_evidence_max_sources = 8;
constexpr float  neighbor_competition_max = 5.0f;
constexpr float  neighbor_activity_threshold = 1.0e-5f;
constexpr float  neighbor_dominance_deadzone = 0.10f;
constexpr float  neighbor_transfer_max_fraction = 0.30f;
constexpr float  output_level_min = 0.8f;
constexpr float  output_level_max = 6.0f;
constexpr float  fixed_output_level_control = 0.5f;
constexpr float  fixed_synth_blend = 1.0f;
constexpr float  harmonic_gate_ratio_low = 0.20f;
constexpr float  harmonic_gate_ratio_high = 1.10f;
constexpr float  harmonic_support_boost_max = 1.2f;
constexpr float  harmonic_reverse_penalty_max = 0.85f;
constexpr float  harmonic_suppression_intensity_scale = 10.0f;
constexpr float  input_gate_open_threshold = 0.012f;
constexpr float  input_gate_close_threshold = 0.008f;
constexpr float  input_gate_detector_attack_ms = 1.5f;
constexpr float  input_gate_detector_release_ms = 35.0f;
constexpr float  input_gate_hold_ms = 35.0f;
constexpr InputGateConfig input_gate_config{
    input_gate_open_threshold,
    input_gate_close_threshold,
    input_gate_detector_attack_ms,
    input_gate_detector_release_ms,
    input_gate_hold_ms,
};

struct Controls
{
    float radius = 0.45f;  // knob 1  resonator radius
    float lpf_cutoff_hz = 600.0f;  // knob 2  synth output LPF frequency in Hz
    float neighbor_competition = neighbor_competition_max;  // temporarily fixed to 100%
    float output_level = 2.0f;  // temporarily fixed in control loop
    float synth_blend = fixed_synth_blend;  // temporarily fixed to 100% wet
    float harmonic_suppression = 0.5f;  // knob 3  harmonic support/suppression intensity
    bool use_input_gate = false;  // toggle 2 enables input gate for synth cutoff
    bool bypass_harmonic_suppression = false;  // toggle 3 bypasses harmonic support/suppression
};

volatile bool effect_enabled = true;
Controls      controls;
float         sample_rate_hz = 48000.0f;

float Clamp01(float x) { return std::clamp(x, 0.0f, 1.0f); }

float LogInterp(float lo, float hi, float t)
{
    return lo * std::pow(hi / lo, Clamp01(t));
}

float Lerp(float lo, float hi, float t)
{
    const float clamped_t = Clamp01(t);
    return lo + (hi - lo) * clamped_t;
}

PolyPitchDetector pitch_detector;

int analysis_counter = 0;
size_t detector_decimation_phase = 0;

std::array<float, PolyPitchDetector::resonator_count> square_phases{};
std::array<float, PolyPitchDetector::resonator_count> voice_amplitudes{};
std::array<float, PolyPitchDetector::resonator_count> resonator_frequencies{};
std::array<size_t, max_synth_voices> active_voice_indices{};
size_t active_voice_count = 0;

struct ReverseEvidenceSource
{
    size_t fundamental_index = PolyPitchDetector::resonator_count;
    float weight = 0.0f;
};

std::array<std::array<size_t, harmonic_support_count>, PolyPitchDetector::resonator_count>
    harmonic_map{};
std::array<std::array<ReverseEvidenceSource, reverse_evidence_max_sources>, PolyPitchDetector::resonator_count>
    reverse_harmonic_map{};

OnePoleLowpass synth_output_lowpass;

float detector_highpass_alpha = 0.99f;
float detector_lowpass_alpha  = 0.05f;
float detector_highpass_state = 0.0f;
float detector_previous_input = 0.0f;
OnePoleLowpass detector_input_lowpass;
InputGate input_gate;

constexpr std::array<float, harmonic_support_count> harmonic_multipliers{
    2.0f,
    3.0f,
    4.0f,
};

constexpr std::array<float, harmonic_support_count> harmonic_weights{
    1.0f,
    0.7f,
    0.45f,
};

float HzToMidi(float hz)
{
    return 69.0f + 12.0f * std::log2(hz / 440.0f);
}

size_t FrequencyToResonatorIndex(float hz)
{
    const int midi_note = static_cast<int>(std::round(HzToMidi(hz)));
    const int clamped_midi = std::clamp(
        midi_note,
        PolyPitchDetector::low_e_midi,
        PolyPitchDetector::high_e_plus_two_octaves_midi);
    return static_cast<size_t>(clamped_midi - PolyPitchDetector::low_e_midi);
}

void InsertReverseEvidenceSource(
    std::array<ReverseEvidenceSource, reverse_evidence_max_sources>& reverse_sources,
    size_t fundamental_index,
    float weight)
{
    for (auto& reverse_source : reverse_sources)
    {
        if (reverse_source.fundamental_index < PolyPitchDetector::resonator_count)
            continue;

        reverse_source.fundamental_index = fundamental_index;
        reverse_source.weight = weight;
        return;
    }

    size_t weakest_index = 0;
    for (size_t source_index = 1; source_index < reverse_evidence_max_sources; ++source_index)
    {
        if (reverse_sources[source_index].weight < reverse_sources[weakest_index].weight)
            weakest_index = source_index;
    }

    if (weight > reverse_sources[weakest_index].weight)
    {
        reverse_sources[weakest_index].fundamental_index = fundamental_index;
        reverse_sources[weakest_index].weight = weight;
    }
}

void InitializeHarmonicMaps()
{
    for (auto& reverse_sources : reverse_harmonic_map)
    {
        for (auto& reverse_source : reverse_sources)
            reverse_source = {};
    }

    for (size_t index = 0; index < PolyPitchDetector::resonator_count; ++index)
    {
        for (size_t harmonic_index = 0; harmonic_index < harmonic_support_count; ++harmonic_index)
        {
            const float harmonic_frequency =
                resonator_frequencies[index] * harmonic_multipliers[harmonic_index];
            const size_t mapped_index = harmonic_frequency > resonator_frequencies.back()
                                          ? PolyPitchDetector::resonator_count
                                          : FrequencyToResonatorIndex(harmonic_frequency);
            harmonic_map[index][harmonic_index] = mapped_index;

            if (mapped_index >= PolyPitchDetector::resonator_count)
                continue;

            InsertReverseEvidenceSource(
                reverse_harmonic_map[mapped_index],
                index,
                harmonic_weights[harmonic_index]);
        }
    }
}

float HarmonicSupportRatio(size_t index, const std::array<float, PolyPitchDetector::resonator_count>& energies)
{
    float weighted_harmonic_energy = 0.0f;
    float total_harmonic_weight = 0.0f;
    for (size_t harmonic_index = 0; harmonic_index < harmonic_support_count; ++harmonic_index)
    {
        const size_t mapped_index = harmonic_map[index][harmonic_index];
        if (mapped_index >= PolyPitchDetector::resonator_count)
            continue;

        const float harmonic_weight = harmonic_weights[harmonic_index];
        weighted_harmonic_energy += energies[mapped_index] * harmonic_weight;
        total_harmonic_weight += harmonic_weight;
    }

    if (total_harmonic_weight <= 1.0e-9f)
        return 0.0f;

    const float normalized_support = weighted_harmonic_energy / total_harmonic_weight;
    return normalized_support / (energies[index] + 1.0e-9f);
}

float HarmonicReverseRatio(size_t index, const std::array<float, PolyPitchDetector::resonator_count>& energies)
{
    float weighted_fundamental_energy = 0.0f;
    float total_weight = 0.0f;
    for (const auto& reverse_source : reverse_harmonic_map[index])
    {
        if (reverse_source.fundamental_index >= PolyPitchDetector::resonator_count)
            continue;

        weighted_fundamental_energy += energies[reverse_source.fundamental_index] * reverse_source.weight;
        total_weight += reverse_source.weight;
    }

    if (total_weight <= 1.0e-9f)
        return 0.0f;

    const float normalized_reverse_energy = weighted_fundamental_energy / total_weight;
    return normalized_reverse_energy / (energies[index] + 1.0e-9f);
}

void FilterInput(float dry)
{
    detector_highpass_state = detector_highpass_alpha
                            * (detector_highpass_state + dry - detector_previous_input);
    detector_previous_input = dry;
    detector_input_lowpass.Process(detector_highpass_state, detector_lowpass_alpha);
}

void RankResonatorEnergy(
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

void UpdateVoiceAmplitudesFromDetector(float radius)
{
    std::array<float, PolyPitchDetector::resonator_count> energies{};
    std::array<float, PolyPitchDetector::resonator_count> competed_energies{};
    std::array<bool, PolyPitchDetector::resonator_count> initially_selected{};
    std::array<float, max_synth_voices> initial_top_energies{};
    std::array<size_t, max_synth_voices> initial_top_indices{};
    std::array<float, max_synth_voices> final_top_energies{};
    std::array<size_t, max_synth_voices> final_top_indices{};
    initial_top_indices.fill(PolyPitchDetector::resonator_count);
    final_top_indices.fill(PolyPitchDetector::resonator_count);

    const float neighbor_competition = controls.neighbor_competition;

    const auto amplitude_from_energy = [&](float energy) {
        const float normalized_amplitude = std::sqrt(std::max(energy, 0.0f))
                                         * (1.0f - radius);
        const float linear_amplitude = std::clamp(normalized_amplitude * 24.0f, 0.0f, 1.0f);
        const float shaped_amplitude = std::pow(linear_amplitude, energy_to_volume_curve);
        return shaped_amplitude;
    };

    for (size_t resonator_index = 0;
         resonator_index < PolyPitchDetector::resonator_count;
         ++resonator_index)
    {
        energies[resonator_index] = pitch_detector.GetEnergyAtIndex(resonator_index);
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
        if (resonator_index >= PolyPitchDetector::resonator_count)
            continue;
        initially_selected[resonator_index] = true;
    }

    if (!controls.bypass_harmonic_suppression)
    {
        for (size_t resonator_index = 0;
             resonator_index < PolyPitchDetector::resonator_count;
             ++resonator_index)
        {
            if (!initially_selected[resonator_index])
                continue;

            const float support_ratio = HarmonicSupportRatio(resonator_index, energies);
            const float reverse_ratio = HarmonicReverseRatio(resonator_index, energies);
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

    if (neighbor_competition > 0.0f)
    {
        for (size_t resonator_index = 0;
             resonator_index + 1 < PolyPitchDetector::resonator_count;
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

            const float transfer_amount = neighbor_competition
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
         resonator_index < PolyPitchDetector::resonator_count;
         ++resonator_index)
    {
        RankResonatorEnergy(
            competed_energies[resonator_index],
            resonator_index,
            final_top_energies,
            final_top_indices);
    }

    voice_amplitudes.fill(0.0f);
    active_voice_count = 0;
    for (size_t rank = 0; rank < max_synth_voices; ++rank)
    {
        const size_t resonator_index = final_top_indices[rank];
        if (resonator_index >= PolyPitchDetector::resonator_count)
            continue;

        const float amplitude = amplitude_from_energy(competed_energies[resonator_index]);
        if (amplitude <= 0.0f)
            continue;

        voice_amplitudes[resonator_index] = amplitude;
        active_voice_indices[active_voice_count++] = resonator_index;
    }
}

float RenderOscillators()
{
    float wet = 0.0f;
    for (size_t voice_index = 0; voice_index < active_voice_count; ++voice_index)
    {
        const size_t resonator_index = active_voice_indices[voice_index];
        const float voice_amplitude = voice_amplitudes[resonator_index];

        const float phase_step = two_pi * resonator_frequencies[resonator_index] / sample_rate_hz;
        square_phases[resonator_index] += phase_step;
        if (square_phases[resonator_index] >= two_pi)
            square_phases[resonator_index] -= two_pi;

        const float square = (square_phases[resonator_index] < 3.14159265f) ? 1.0f : -1.0f;
        wet += square * (voice_amplitude * square_amplitude);
    }

    return wet * (synth_output_gain * controls.output_level);
}

void ProcessAudioBlock(
    daisy::AudioHandle::InputBuffer  in,
    daisy::AudioHandle::OutputBuffer out,
    size_t size)
{
    const Controls c = controls;

    const float radius = pitch_detector.GetRadius();
    const float synth_lowpass_alpha =
        OnePoleLowpass::AlphaFromCutoff(c.lpf_cutoff_hz, sample_rate_hz);

    for (size_t i = 0; i < size; ++i)
    {
        const float dry = in[0][i];
        if (c.use_input_gate)
        {
            input_gate.Process(std::abs(dry), sample_rate_hz, input_gate_config);
        }
        else
        {
            input_gate.Reset(true);
        }

        FilterInput(dry);

        detector_decimation_phase = (detector_decimation_phase + 1) % detector_decimation_factor;
        if (detector_decimation_phase == 0)
        {
            pitch_detector.Process(detector_input_lowpass.state());

            ++analysis_counter;
            if (analysis_counter >= analysis_period_detector_samples)
            {
                analysis_counter = 0;
                UpdateVoiceAmplitudesFromDetector(radius);
            }
        }

        float wet = RenderOscillators();
        if (c.use_input_gate && !input_gate.is_open())
        {
            wet = 0.0f;
        }
        const float synth_lowpass_sample = synth_output_lowpass.Process(wet, synth_lowpass_alpha);

        const float output = effect_enabled
                   ? Lerp(dry, synth_lowpass_sample, c.synth_blend)
                   : dry;
        out[0][i] = std::clamp(output, -1.0f, 1.0f);
        out[1][i] = out[0][i];
    }
}
} 

int main()
{
    terrarium.Init(true);
    terrarium.seed.SetAudioBlockSize(1);

    sample_rate_hz = terrarium.seed.AudioSampleRate();
    pitch_detector.Init(sample_rate_hz / static_cast<float>(detector_decimation_factor));

    const float dt = 1.0f / sample_rate_hz;
    const float highpass_rc = 1.0f / (two_pi * input_highpass_cutoff_hz);
    detector_highpass_alpha = highpass_rc / (highpass_rc + dt);
    detector_lowpass_alpha = 1.0f - std::exp(-two_pi * input_lowpass_cutoff_hz / sample_rate_hz);
    detector_input_lowpass.Reset(0.0f);
    synth_output_lowpass.Reset(0.0f);
    input_gate.Reset(true);
    pitch_detector.SetEnergySlew(0.25f);

    for (size_t resonator_index = 0;
         resonator_index < PolyPitchDetector::resonator_count;
         ++resonator_index)
    {
        square_phases[resonator_index] = 0.0f;
        voice_amplitudes[resonator_index] = 0.0f;
        resonator_frequencies[resonator_index] =
            pitch_detector.GetFrequencyAtIndex(resonator_index);
    }
    InitializeHarmonicMaps();

    auto& led_effect = terrarium.leds[0];
    auto& led_mode   = terrarium.leds[1];

    terrarium.seed.StartAudio(ProcessAudioBlock);

    terrarium.Loop(250, [&]() {
        controls.radius = terrarium.knobs[0].Process();
        controls.lpf_cutoff_hz = LogInterp(
            synth_lowpass_cutoff_min_hz,
            synth_lowpass_cutoff_max_hz,
            terrarium.knobs[1].Process());
        controls.harmonic_suppression = terrarium.knobs[2].Process();
        controls.neighbor_competition = neighbor_competition_max;
        controls.output_level = LogInterp(
            output_level_min,
            output_level_max,
            fixed_output_level_control);
        controls.synth_blend = fixed_synth_blend;
        controls.use_input_gate = terrarium.toggles[1].Pressed();
        controls.bypass_harmonic_suppression = terrarium.toggles[2].Pressed();

        // Radius close to 1.0 means narrower frequency selectivity.
        pitch_detector.SetRadius(LogInterp(0.9990f, 0.99999999f, controls.radius));

        if (terrarium.stomps[0].RisingEdge())
            effect_enabled = !effect_enabled;

        led_effect.Set(effect_enabled ? 1.0f : 0.0f);
        led_mode.Set(controls.bypass_harmonic_suppression ? 1.0f : 0.15f);
    });
}
