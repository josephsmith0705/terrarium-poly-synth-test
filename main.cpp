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
constexpr float  neighbor_competition_min = 0.0f;
constexpr float  neighbor_competition_max = 5.0f;
constexpr float  neighbor_activity_threshold = 1.0e-5f;
constexpr float  neighbor_dominance_deadzone = 0.10f;
constexpr float  neighbor_transfer_max_fraction = 0.30f;
constexpr float  output_level_min = 0.8f;
constexpr float  output_level_max = 6.0f;
constexpr float  blend_min = 0.0f;
constexpr float  blend_max = 1.0f;
constexpr float  input_gate_open_threshold = 0.012f;
constexpr float  input_gate_close_threshold = 0.008f;
constexpr float  input_gate_detector_attack_ms = 1.5f;
constexpr float  input_gate_detector_release_ms = 35.0f;
constexpr float  input_gate_hold_ms = 35.0f;

// -----------------------------------------------------------------------
// Controls
// -----------------------------------------------------------------------

struct Controls
{
    float radius = 0.45f;  // knob 1  resonator radius
    float lpf_cutoff_hz = 600.0f;  // knob 2  synth output LPF frequency in Hz
    float neighbor_competition = 0.5f;  // knob 3  adjacent resonator competition
    float output_level = 2.0f;  // knob 5  master synth level
    float synth_blend = 1.0f;  // knob 6  dry/wet blend (0=dry, 1=wet)
    bool use_input_gate = false;  // toggle 2 enables input gate for synth cutoff
    bool ignore_neighbor_competition = false;  // toggle 3 bypasses neighbor competition
};

volatile bool effect_enabled = true;
Controls      controls;
float         sample_rate_hz = 48000.0f;

float Clamp01(float x) { return std::clamp(x, 0.0f, 1.0f); }

float LogLerp(float lo, float hi, float t)
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
int analysis_period_active = analysis_period_detector_samples;
size_t detector_decimation_phase = 0;

std::array<float, PolyPitchDetector::resonator_count> square_phases{};
std::array<float, PolyPitchDetector::resonator_count> voice_amplitudes{};
std::array<float, PolyPitchDetector::resonator_count> resonator_frequencies{};
std::array<size_t, max_synth_voices> active_voice_indices{};
size_t active_voice_count = 0;

float synth_lowpass_state = 0.0f;

float detector_highpass_alpha = 0.99f;
float detector_lowpass_alpha  = 0.05f;
float detector_highpass_state = 0.0f;
float detector_previous_input = 0.0f;
float detector_lowpass_state  = 0.0f;
bool input_gate_open = true;
float gate_detector_level = 0.0f;
int   gate_hold_counter_samples = 0;

void FilterInput(float dry)
{
    detector_highpass_state = detector_highpass_alpha
                            * (detector_highpass_state + dry - detector_previous_input);
    detector_previous_input = dry;
    detector_lowpass_state += detector_lowpass_alpha
                            * (detector_highpass_state - detector_lowpass_state);
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

    const float neighbor_competition = controls.ignore_neighbor_competition
                                     ? 0.0f
                                     : controls.neighbor_competition;

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

        for (size_t rank = 0; rank < max_synth_voices; ++rank)
        {
            if (energies[resonator_index] <= initial_top_energies[rank])
                continue;

            for (size_t shift = max_synth_voices - 1; shift > rank; --shift)
            {
                initial_top_energies[shift] = initial_top_energies[shift - 1];
                initial_top_indices[shift] = initial_top_indices[shift - 1];
            }

            initial_top_energies[rank] = energies[resonator_index];
            initial_top_indices[rank] = resonator_index;
            break;
        }
    }

    for (size_t rank = 0; rank < max_synth_voices; ++rank)
    {
        const size_t resonator_index = initial_top_indices[rank];
        if (resonator_index >= PolyPitchDetector::resonator_count)
            continue;
        initially_selected[resonator_index] = true;
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
        for (size_t rank = 0; rank < max_synth_voices; ++rank)
        {
            if (competed_energies[resonator_index] <= final_top_energies[rank])
                continue;

            for (size_t shift = max_synth_voices - 1; shift > rank; --shift)
            {
                final_top_energies[shift] = final_top_energies[shift - 1];
                final_top_indices[shift] = final_top_indices[shift - 1];
            }

            final_top_energies[rank] = competed_energies[resonator_index];
            final_top_indices[rank] = resonator_index;
            break;
        }
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

    if(!effect_enabled)
    {
        for (size_t i = 0; i < size; ++i)
        {
            out[0][i] = in[0][i];
            out[1][i] = in[1][i];
        }
        return;
    }

    const float radius = pitch_detector.GetRadius();
    const float synth_lowpass_alpha = 1.0f
                                    - std::exp(-two_pi * c.lpf_cutoff_hz / sample_rate_hz);
    const float gate_detector_attack_alpha = 1.0f
                                           - std::exp(-1.0f / std::max((input_gate_detector_attack_ms * 0.001f) * sample_rate_hz, 1.0f));
    const float gate_detector_release_alpha = 1.0f
                                            - std::exp(-1.0f / std::max((input_gate_detector_release_ms * 0.001f) * sample_rate_hz, 1.0f));
    const int gate_hold_samples = std::max(
        1,
        static_cast<int>(input_gate_hold_ms * 0.001f * sample_rate_hz));

    for (size_t i = 0; i < size; ++i)
    {
        const float dry = in[0][i];
        const float dry_abs = std::abs(dry);
        const float gate_alpha = dry_abs > gate_detector_level
                               ? gate_detector_attack_alpha
                               : gate_detector_release_alpha;
        gate_detector_level += gate_alpha * (dry_abs - gate_detector_level);

        if (c.use_input_gate)
        {
            if (input_gate_open)
            {
                if (gate_detector_level >= input_gate_open_threshold)
                {
                    gate_hold_counter_samples = gate_hold_samples;
                }
                else if (gate_detector_level < input_gate_close_threshold)
                {
                    if (gate_hold_counter_samples > 0)
                        --gate_hold_counter_samples;
                    if (gate_hold_counter_samples <= 0)
                        input_gate_open = false;
                }
            }
            else if (gate_detector_level > input_gate_open_threshold)
            {
                input_gate_open = true;
                gate_hold_counter_samples = gate_hold_samples;
            }
        }
        else
        {
            input_gate_open = true;
            gate_hold_counter_samples = gate_hold_samples;
        }

        FilterInput(dry);

        detector_decimation_phase = (detector_decimation_phase + 1) % detector_decimation_factor;
        if (detector_decimation_phase == 0)
        {
            pitch_detector.Process(detector_lowpass_state);

            ++analysis_counter;
            if (analysis_counter >= analysis_period_active)
            {
                analysis_counter = 0;
                UpdateVoiceAmplitudesFromDetector(radius);
            }
        }

        float wet = RenderOscillators();
        if (c.use_input_gate && !input_gate_open)
        {
            wet = 0.0f;
        }
        synth_lowpass_state += synth_lowpass_alpha * (wet - synth_lowpass_state);

        const float output = effect_enabled
                   ? Lerp(dry, synth_lowpass_state, c.synth_blend)
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

    auto& led_effect = terrarium.leds[0];
    auto& led_mode   = terrarium.leds[1];

    terrarium.seed.StartAudio(ProcessAudioBlock);

    terrarium.Loop(250, [&]() {
        controls.radius = terrarium.knobs[0].Process();
        controls.lpf_cutoff_hz = LogLerp(
            synth_lowpass_cutoff_min_hz,
            synth_lowpass_cutoff_max_hz,
            terrarium.knobs[1].Process());
        controls.neighbor_competition = Lerp(
            neighbor_competition_min,
            neighbor_competition_max,
            terrarium.knobs[2].Process());
        controls.output_level = LogLerp(
            output_level_min,
            output_level_max,
            terrarium.knobs[4].Process());
        controls.synth_blend = Lerp(
            blend_min,
            blend_max,
            terrarium.knobs[5].Process());
        controls.use_input_gate = terrarium.toggles[1].Pressed();
        controls.ignore_neighbor_competition = terrarium.toggles[2].Pressed();

        // Radius close to 1.0 means narrower frequency selectivity.
        pitch_detector.SetRadius(LogLerp(0.9990f, 0.99999999f, controls.radius));
        pitch_detector.SetMode(PolyPitchDetector::Mode::FullRange);

        if (terrarium.stomps[0].RisingEdge())
            effect_enabled = !effect_enabled;

        led_effect.Set(effect_enabled ? 1.0f : 0.0f);
        led_mode.Set(controls.ignore_neighbor_competition ? 1.0f : 0.15f);
    });
}
