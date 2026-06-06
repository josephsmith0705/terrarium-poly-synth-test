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

// -----------------------------------------------------------------------
// Controls
// -----------------------------------------------------------------------

struct Controls
{
    float radius         = 0.45f;  // 1  resonator radius
    float threshold      = 0.40f;  // 2  detection threshold
    float analysis_frame = 0.20f;  // 3  analysis window length: 128–2048 samples
    float neighbor_range = 1.0f;  // 4  semitone neighborhood range for suppression
    float energy_slew    = 0.25f;  // 5  energy smoothing speed
    float neighbor_threshold_boost = 0.25f;  // 6  threshold increase near active notes
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

float HarmonicConfidence(int semitone_distance)
{
    const int distance = std::abs(semitone_distance);
    if (distance == 0)
        return 0.0f;

    const int interval_class = distance % 12;
    float confidence = 0.0f;

    // High-confidence harmonic relationships.
    if (interval_class == 0)
        confidence = 1.00f;       // octave
    else if (interval_class == 7)
        confidence = 0.90f;       // fifth
    else if (interval_class == 5)
        confidence = 0.85f;       // fourth
    else if (interval_class == 4 || interval_class == 9)
        confidence = 0.45f;       // major third / major sixth

    if (confidence <= 0.0f)
        return 0.0f;

    // Confidence decays slightly as intervals get farther apart.
    const int octave_span = distance / 12;
    return confidence / (1.0f + 0.25f * static_cast<float>(octave_span));
}

float HarmonicNeighborConfidence(int semitone_distance)
{
    // Neighboring bins around strong harmonic intervals can be false positives.
    // Use a reduced confidence based on adjacent harmonic classes.
    const float left_neighbor_confidence = HarmonicConfidence(semitone_distance - 1);
    const float right_neighbor_confidence = HarmonicConfidence(semitone_distance + 1);
    return 0.45f * std::max(left_neighbor_confidence, right_neighbor_confidence);
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
    const int neighbor_range_bins = static_cast<int>(std::round(Lerp(0.0f, 20.0f, c.neighbor_range)));
    const float neighbor_threshold_boost = Lerp(0.0f, 50.0f, c.neighbor_threshold_boost);
    const float harmonic_threshold_boost = neighbor_threshold_boost * 0.75f;
    const float harmonic_neighbor_threshold_boost = harmonic_threshold_boost * 0.35f;
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

            std::array<bool, PolyPitchDetector::resonator_count> was_active{};
            for (size_t index = 0; index < PolyPitchDetector::resonator_count; ++index)
                was_active[index] = voice_amplitudes[index] > 0.0f;

            // Only strong active bins are allowed to suppress neighbors/harmonics.
            std::array<bool, PolyPitchDetector::resonator_count> is_strong_anchor{};
            float max_active_energy = 0.0f;
            for (size_t index = 0; index < PolyPitchDetector::resonator_count; ++index)
            {
                if (!was_active[index])
                    continue;
                max_active_energy = std::max(max_active_energy, pitch_detector.GetEnergyAtIndex(index));
            }

            if (max_active_energy > 0.0f)
            {
                const float anchor_energy_threshold = std::max(threshold, max_active_energy * 0.55f);
                for (size_t index = 0; index < PolyPitchDetector::resonator_count; ++index)
                {
                    if (!was_active[index])
                        continue;
                    is_strong_anchor[index] =
                        pitch_detector.GetEnergyAtIndex(index) >= anchor_energy_threshold;
                }
            }

            for (size_t resonator_index = 0;
                 resonator_index < PolyPitchDetector::resonator_count;
                 ++resonator_index)
            {
                const float detected_energy = pitch_detector.GetEnergyAtIndex(resonator_index);

                float adaptive_threshold = threshold;
                if (neighbor_range_bins > 0)
                {
                    for (size_t active_index = 0;
                         active_index < PolyPitchDetector::resonator_count;
                         ++active_index)
                    {
                        if (!is_strong_anchor[active_index])
                            continue;

                        const int semitone_distance = std::abs(
                            static_cast<int>(resonator_index) - static_cast<int>(active_index));
                        if (semitone_distance == 0 || semitone_distance > neighbor_range_bins)
                            continue;

                        const float proximity = 1.0f
                                              - static_cast<float>(semitone_distance)
                                                    / static_cast<float>(neighbor_range_bins + 1);
                        adaptive_threshold += neighbor_threshold_boost * proximity;
                    }
                }

                for (size_t active_index = 0;
                     active_index < PolyPitchDetector::resonator_count;
                     ++active_index)
                {
                    if (!is_strong_anchor[active_index] || active_index == resonator_index)
                        continue;

                    const int semitone_distance = std::abs(
                        static_cast<int>(resonator_index) - static_cast<int>(active_index));
                    const float harmonic_confidence = HarmonicConfidence(semitone_distance);
                    if (harmonic_confidence <= 0.0f)
                    {
                        const float harmonic_neighbor_confidence =
                            HarmonicNeighborConfidence(semitone_distance);
                        if (harmonic_neighbor_confidence <= 0.0f)
                            continue;

                        adaptive_threshold += harmonic_neighbor_threshold_boost
                                            * harmonic_neighbor_confidence;
                    }
                    else
                    {
                        adaptive_threshold += harmonic_threshold_boost * harmonic_confidence;
                    }
                }

                const float normalized_amplitude = std::sqrt(std::max(detected_energy, 0.0f))
                                                 * (1.0f - radius);
                const float linear_amplitude = std::clamp(normalized_amplitude * 12.0f, 0.0f, 1.0f);
                const float shaped_amplitude = std::pow(linear_amplitude, energy_to_volume_curve);
                const float amplitude_from_energy = square_amplitude * shaped_amplitude;

                voice_amplitudes[resonator_index] =
                    (detected_energy >= adaptive_threshold) ? amplitude_from_energy : 0.0f;
            }
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
        // controls.neighbor_range  = terrarium.knobs[3].Process();
        controls.energy_slew     = terrarium.knobs[4].Process();
        controls.neighbor_threshold_boost = terrarium.knobs[5].Process();
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
