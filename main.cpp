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
constexpr float  synth_lowpass_cutoff_min_hz = 120.0f;
constexpr float  synth_lowpass_cutoff_max_hz = 4800.0f;
constexpr float  square_amplitude = 0.85f;
constexpr float  triangle_amplitude = 0.85f;
constexpr float  synth_output_gain = 2.0f;

// -----------------------------------------------------------------------
// Controls
// -----------------------------------------------------------------------

struct Controls
{
    float radius = 0.45f;  // knob 1  resonator radius
    float lpf_frequency = 0.50f;  // knob 2  synth output LPF frequency
    bool  triangle_mode = false;  // toggle 2  square/triangle switch
};

volatile bool effect_enabled = true;
Controls      controls;
float         sample_rate_hz = 48000.0f;

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

float Clamp01(float x) { return std::clamp(x, 0.0f, 1.0f); }

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

float synth_lowpass_state = 0.0f;

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

    const float radius = pitch_detector.GetRadius();
    const float synth_lowpass_cutoff_hz = LogLerp(
        synth_lowpass_cutoff_min_hz,
        synth_lowpass_cutoff_max_hz,
        c.lpf_frequency);
    const float synth_lowpass_alpha = 1.0f
                                    - std::exp(-two_pi * synth_lowpass_cutoff_hz / sample_rate_hz);

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
            analysis_period_active = 256;

            std::array<float, PolyPitchDetector::resonator_count> selected_voice_amplitudes{};

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
                const float detected_energy = pitch_detector.GetEnergyAtIndex(resonator_index);
                selected_voice_amplitudes[resonator_index] = amplitude_from_energy(detected_energy);
            }

            voice_amplitudes = selected_voice_amplitudes;
        }

        float wet = 0.0f;
        for (size_t resonator_index = 0;
             resonator_index < PolyPitchDetector::resonator_count;
             ++resonator_index)
        {
            const float voice_amplitude = voice_amplitudes[resonator_index];

            const float frequency = pitch_detector.GetFrequencyAtIndex(resonator_index);
            const float phase_step = two_pi * frequency / sample_rate_hz;
            square_phases[resonator_index] += phase_step;
            if (square_phases[resonator_index] >= two_pi)
                square_phases[resonator_index] -= two_pi;

            const float square = (square_phases[resonator_index] < 3.14159265f) ? 1.0f : -1.0f;
            const float phase_norm = square_phases[resonator_index] / two_pi;
            const float triangle = 1.0f - 4.0f * std::abs(phase_norm - 0.5f);
            const float waveform = c.triangle_mode ? triangle : square;
            const float oscillator_gain = c.triangle_mode ? triangle_amplitude : square_amplitude;
            wet += waveform * (voice_amplitude * oscillator_gain);
        }

        wet *= synth_output_gain;
        synth_lowpass_state += synth_lowpass_alpha * (wet - synth_lowpass_state);

        const float output = effect_enabled ? synth_lowpass_state : dry;
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

    const float dt = 1.0f / sample_rate_hz;
    const float highpass_rc = 1.0f / (two_pi * detector_highpass_cutoff_hz);
    detector_highpass_alpha = highpass_rc / (highpass_rc + dt);
    detector_lowpass_alpha = 1.0f - std::exp(-two_pi * detector_lowpass_cutoff_hz / sample_rate_hz);
    pitch_detector.SetEnergySlew(0.25f);

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
        controls.radius = terrarium.knobs[0].Process();
        controls.lpf_frequency = terrarium.knobs[1].Process();
        controls.triangle_mode = terrarium.toggles[1].Pressed();

        // Radius close to 1.0 means narrower frequency selectivity.
        pitch_detector.SetRadius(LogLerp(0.9990f, 0.99999999f, controls.radius));
        pitch_detector.SetMode(PolyPitchDetector::Mode::FullRange);

        if (terrarium.stomps[0].RisingEdge())
            effect_enabled = !effect_enabled;

        led_effect.Set(effect_enabled ? 1.0f : 0.0f);
        led_mode.Set(controls.triangle_mode ? 1.0f : 0.15f);
    });
}
