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

// -----------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------

constexpr float  two_pi           = 6.28318530717958647692f;
constexpr float  detector_highpass_cutoff_hz = 70.0f;
constexpr float  detector_lowpass_cutoff_hz  = 380.0f;
constexpr float  energy_to_volume_curve = 1.8f;
constexpr float  local_selection_switch_margin_min = 0.0f;
constexpr float  local_selection_switch_margin_max = 0.35f;
constexpr float  local_selection_neighbor_bias_min = 0.7f;
constexpr float  local_selection_neighbor_bias_max = 1.4f;
constexpr float  dual_note_window_similarity_min = 0.60f;
constexpr float  dual_note_pair_balance_min = 0.72f;

// -----------------------------------------------------------------------
// Controls
// -----------------------------------------------------------------------

struct Controls
{
    float radius         = 0.45f;  // 1  resonator radius
    float threshold      = 0.40f;  // 2  detection threshold
    float analysis_frame = 0.20f;  // 3  analysis window length: 128–2048 samples
    float local_selection_switch_margin = 0.30f;  // 4  takeover margin for neighbor winner
    float energy_slew    = 0.25f;  // 5  energy smoothing speed
    float local_selection_neighbor_bias = 0.40f;  // 6  neighbor winner bias
    bool  full_bank_mode = false;  // toggle 1  full-bank output vs A-only output
    bool  fuzz_lp_dry_blend = false;  // toggle 3  fuzzed LP dry + HP synth blend
};

volatile bool effect_enabled = true;
Controls      controls;
float         sample_rate_hz = 48000.0f;

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

float Clamp01(float x) { return std::clamp(x, 0.0f, 1.0f); }

float Lerp(float a, float b, float t) { return a + (b - a) * Clamp01(t); }

float LogLerp(float lo, float hi, float t)
{
    return lo * std::pow(hi / lo, Clamp01(t));
}

// -----------------------------------------------------------------------
// Global state
// -----------------------------------------------------------------------

PolyPitchDetector pitch_detector;

int analysis_counter = 0;
int analysis_period_active = 256;

std::array<float, PolyPitchDetector::resonator_count> square_phases{};
std::array<float, PolyPitchDetector::resonator_count> voice_amplitudes{};

float split_blend_alpha = 0.03f;
float lp_dry_pre_fuzz_state = 0.0f;
float lp_dry_post_fuzz_state = 0.0f;
float lp_synth_state = 0.0f;

float detector_highpass_alpha = 0.99f;
float detector_lowpass_alpha  = 0.05f;
float detector_highpass_state = 0.0f;
float detector_previous_input = 0.0f;
float detector_lowpass_state  = 0.0f;

// -----------------------------------------------------------------------
// Audio callback
// -----------------------------------------------------------------------

void ProcessAudioBlock(
    daisy::AudioHandle::InputBuffer  in,
    daisy::AudioHandle::OutputBuffer out,
    size_t size)
{
    const Controls c = controls;

    const float threshold = Lerp(0.0000001f, 0.99f, c.threshold);
    const float local_selection_switch_margin = Lerp(
        local_selection_switch_margin_min,
        local_selection_switch_margin_max,
        c.local_selection_switch_margin);
    const float local_selection_neighbor_bias = Lerp(
        local_selection_neighbor_bias_min,
        local_selection_neighbor_bias_max,
        c.local_selection_neighbor_bias);
    constexpr float square_amplitude = 0.25f;
    const float radius = pitch_detector.GetRadius();

    for (size_t i = 0; i < size; ++i)
    {
        const float dry = in[0][i];

        // Detector-only pre-filter: keep mostly guitar fundamental range.
        detector_highpass_state = detector_highpass_alpha
                                * (detector_highpass_state + dry - detector_previous_input);
        detector_previous_input = dry;
        detector_lowpass_state += detector_lowpass_alpha
                                * (detector_highpass_state - detector_lowpass_state);

        pitch_detector.Process(detector_lowpass_state);

        ++analysis_counter;
        if (analysis_counter >= analysis_period_active)
        {
            analysis_counter = 0;
            analysis_period_active = static_cast<int>(
                LogLerp(128.0f, 2048.0f, c.analysis_frame));

            std::array<float, PolyPitchDetector::resonator_count> selected_voice_amplitudes{};

            const auto amplitude_from_energy = [&](float energy) {
                const float normalized_amplitude = std::sqrt(std::max(energy, 0.0f))
                                                 * (1.0f - radius);
                const float linear_amplitude = std::clamp(normalized_amplitude * 12.0f, 0.0f, 1.0f);
                const float shaped_amplitude = std::pow(linear_amplitude, energy_to_volume_curve);
                return square_amplitude * shaped_amplitude;
            };

            for (size_t resonator_index = 0;
                 resonator_index < PolyPitchDetector::resonator_count;
                 ++resonator_index)
            {
                const float detected_energy = pitch_detector.GetEnergyAtIndex(resonator_index);
                if (detected_energy < threshold)
                    continue;

                const float left_energy = (resonator_index > 0)
                                        ? pitch_detector.GetEnergyAtIndex(resonator_index - 1)
                                        : -1.0f;
                const float right_energy = (resonator_index + 1 < PolyPitchDetector::resonator_count)
                                         ? pitch_detector.GetEnergyAtIndex(resonator_index + 1)
                                         : -1.0f;

                size_t winner_index = resonator_index;
                float winner_score = detected_energy;

                const float left_score = left_energy * local_selection_neighbor_bias;
                if (left_score > winner_score * (1.0f + local_selection_switch_margin))
                {
                    winner_score = left_score;
                    winner_index = resonator_index - 1;
                }

                const float right_score = right_energy * local_selection_neighbor_bias;
                if (right_score > winner_score * (1.0f + local_selection_switch_margin))
                {
                    winner_index = resonator_index + 1;
                }

                const float amplitude = amplitude_from_energy(detected_energy);

                selected_voice_amplitudes[winner_index] = std::max(
                    selected_voice_amplitudes[winner_index],
                    amplitude);

                // Preserve true neighboring-note double-stops: when a 5-bin
                // window around an adjacent pair is similarly energized,
                // keep both pair notes audible instead of collapsing to one.
                if (resonator_index > 0
                    && resonator_index + 3 < PolyPitchDetector::resonator_count)
                {
                    const size_t left_pair_index = resonator_index;
                    const size_t right_pair_index = resonator_index + 1;

                    const float window_e0 = pitch_detector.GetEnergyAtIndex(left_pair_index - 1);
                    const float window_e1 = pitch_detector.GetEnergyAtIndex(left_pair_index);
                    const float window_e2 = pitch_detector.GetEnergyAtIndex(right_pair_index);
                    const float window_e3 = pitch_detector.GetEnergyAtIndex(right_pair_index + 1);
                    const float window_e4 = pitch_detector.GetEnergyAtIndex(right_pair_index + 2);

                    const float window_min = std::min(
                        std::min(window_e0, window_e1),
                        std::min(window_e2, std::min(window_e3, window_e4)));
                    const float window_max = std::max(
                        std::max(window_e0, window_e1),
                        std::max(window_e2, std::max(window_e3, window_e4)));

                    const float window_similarity = window_min / (window_max + 1.0e-9f);
                    const float pair_balance = std::min(window_e1, window_e2)
                                             / (std::max(window_e1, window_e2) + 1.0e-9f);

                    const bool likely_dual_neighbor_notes =
                        window_e1 >= threshold
                        && window_e2 >= threshold
                        && window_similarity >= dual_note_window_similarity_min
                        && pair_balance >= dual_note_pair_balance_min;

                    if (likely_dual_neighbor_notes)
                    {
                        selected_voice_amplitudes[left_pair_index] = std::max(
                            selected_voice_amplitudes[left_pair_index],
                            amplitude_from_energy(window_e1));
                        selected_voice_amplitudes[right_pair_index] = std::max(
                            selected_voice_amplitudes[right_pair_index],
                            amplitude_from_energy(window_e2));
                    }
                }
            }

            voice_amplitudes = selected_voice_amplitudes;
        }

        float wet = 0.0f;
        size_t active_voice_count = 0;
        for (size_t resonator_index = 0;
             resonator_index < PolyPitchDetector::resonator_count;
             ++resonator_index)
        {
            const float voice_amplitude = voice_amplitudes[resonator_index];
            if (voice_amplitude <= 0.0f)
                continue;

            // In A-only output mode, keep full-bank detection active but only
            // render the A-string voice so suppression behavior can be tested.
            if (!c.full_bank_mode
                && resonator_index != PolyPitchDetector::a_string_index)
                continue;

            const float frequency = pitch_detector.GetFrequencyAtIndex(resonator_index);
            const float phase_step = two_pi * frequency / sample_rate_hz;
            square_phases[resonator_index] += phase_step;
            if (square_phases[resonator_index] >= two_pi)
                square_phases[resonator_index] -= two_pi;

            const float square = (square_phases[resonator_index] < 3.14159265f) ? 1.0f : -1.0f;
            wet += square * voice_amplitude;
            ++active_voice_count;
        }

        if (active_voice_count > 0)
            wet *= 1.0f / std::sqrt(static_cast<float>(active_voice_count));

        // Dry branch: low-pass -> square fuzz -> low-pass.
        lp_dry_pre_fuzz_state += split_blend_alpha * (dry - lp_dry_pre_fuzz_state);
        const float fuzzed_dry = (lp_dry_pre_fuzz_state > 0.01f) ? 1.0f : -1.0f;
        lp_dry_post_fuzz_state += split_blend_alpha * (fuzzed_dry - lp_dry_post_fuzz_state);
        lp_synth_state += split_blend_alpha * (wet - lp_synth_state);

        if (c.fuzz_lp_dry_blend)
        {
            constexpr float fuzz_mix = 0.02f;
            const float fuzzed_lp_dry = lp_dry_post_fuzz_state;
            const float hp_synth = wet - lp_synth_state;
            wet = hp_synth + fuzzed_lp_dry * fuzz_mix;
        }

        const float output = effect_enabled ? wet : dry;
        out[0][i] = std::clamp(output, -1.0f, 1.0f);
        out[1][i] = out[0][i];
    }
}
}  // namespace

// -----------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------

int main()
{
    terrarium.Init(true);
    terrarium.seed.SetAudioBlockSize(2);

    sample_rate_hz = terrarium.seed.AudioSampleRate();
    pitch_detector.Init(sample_rate_hz);

    const float split_blend_cutoff_hz = 180.0f;
    split_blend_alpha = 1.0f - std::exp(-two_pi * split_blend_cutoff_hz / sample_rate_hz);

    const float dt = 1.0f / sample_rate_hz;
    const float highpass_rc = 1.0f / (two_pi * detector_highpass_cutoff_hz);
    detector_highpass_alpha = highpass_rc / (highpass_rc + dt);
    detector_lowpass_alpha = 1.0f - std::exp(-two_pi * detector_lowpass_cutoff_hz / sample_rate_hz);

    for (size_t resonator_index = 0;
         resonator_index < PolyPitchDetector::resonator_count;
         ++resonator_index)
    {
        square_phases[resonator_index] = 0.0f;
        voice_amplitudes[resonator_index] = 0.0f;
    }

    auto& led_effect = terrarium.leds[0];
    auto& led_mode   = terrarium.leds[1];

    terrarium.seed.StartAudio(ProcessAudioBlock);

    terrarium.Loop(250, [&]() {
        controls.radius          = terrarium.knobs[0].Process();
        controls.threshold       = terrarium.knobs[1].Process();
        controls.analysis_frame  = terrarium.knobs[2].Process();
        controls.local_selection_switch_margin = terrarium.knobs[3].Process();
        controls.energy_slew     = terrarium.knobs[4].Process();
        controls.local_selection_neighbor_bias = terrarium.knobs[5].Process();
        controls.full_bank_mode  = terrarium.toggles[0].Pressed();
        controls.fuzz_lp_dry_blend = terrarium.toggles[2].Pressed();

        // Radius close to 1.0 means narrower frequency selectivity.
        pitch_detector.SetRadius(LogLerp(0.9990f, 0.99999999f, controls.radius));
        // Higher energy slew is faster response but can be twitchier.
        pitch_detector.SetEnergySlew(Lerp(0.001f, 0.50f, controls.energy_slew));
        // Always keep detection in full-range mode. The toggle now only
        // controls whether all voiced notes are audible or A-only is audible.
        pitch_detector.SetMode(PolyPitchDetector::Mode::FullRange);

        if (terrarium.stomps[0].RisingEdge())
            effect_enabled = !effect_enabled;

        led_effect.Set(effect_enabled ? 1.0f : 0.0f);
        led_mode.Set(controls.full_bank_mode ? 1.0f : 0.15f);
    });
}
