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
constexpr float  neighbor_competition_max = 5.0f;
constexpr float  output_level_min = 0.8f;
constexpr float  output_level_max = 6.0f;
constexpr float  fixed_output_level_control = 0.5f;
constexpr float  fixed_synth_blend = 1.0f;
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

std::array<float, PolyPitchDetector::resonator_count> square_phases{};
std::array<float, PolyPitchDetector::resonator_count> resonator_frequencies{};
std::array<size_t, max_synth_voices> active_voice_indices{};
size_t active_voice_count = 0;

OnePoleLowpass synth_output_lowpass;

float detector_highpass_alpha = 0.99f;
float detector_lowpass_alpha  = 0.05f;
float detector_highpass_state = 0.0f;
float detector_previous_input = 0.0f;
OnePoleLowpass detector_input_lowpass;
InputGate input_gate;

float FilterInput(float dry)
{
    detector_highpass_state = detector_highpass_alpha
                            * (detector_highpass_state + dry - detector_previous_input);
    detector_previous_input = dry;
    return detector_input_lowpass.Process(detector_highpass_state, detector_lowpass_alpha);
}

float RenderOscillators()
{
    float wet = 0.0f;
    for (size_t voice_index = 0; voice_index < active_voice_count; ++voice_index)
    {
        const size_t resonator_index = active_voice_indices[voice_index];
        const float voice_energy = pitch_detector.GetEnergyAtIndex(resonator_index);

        const float radius = pitch_detector.GetRadius();
        const float normalized_amplitude = std::sqrt(std::max(voice_energy, 0.0f)) * (1.0f - radius);
        const float linear_amplitude = std::clamp(normalized_amplitude * 24.0f, 0.0f, 1.0f);
        const float shaped_amplitude = std::pow(linear_amplitude, 1.8f);

        const float phase_step = two_pi * resonator_frequencies[resonator_index] / sample_rate_hz;
        square_phases[resonator_index] += phase_step;
        if (square_phases[resonator_index] >= two_pi)
            square_phases[resonator_index] -= two_pi;

        const float square = (square_phases[resonator_index] < 3.14159265f) ? 1.0f : -1.0f;
        wet += square * (shaped_amplitude * square_amplitude);
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
        const PolyPitchDetector::DetectorControls detector_controls{
            .radius = radius,
            .neighbor_competition = c.neighbor_competition,
            .harmonic_suppression = c.harmonic_suppression,
            .bypass_harmonic_suppression = c.bypass_harmonic_suppression,
        };

        const float dry = in[0][i];
        if (c.use_input_gate)
        {
            input_gate.Process(std::abs(dry), sample_rate_hz, input_gate_config);
        }
        else
        {
            input_gate.Reset(true);
        }

        const float filtered_dry = FilterInput(dry);
        
        pitch_detector.ProcessInputSample(
            filtered_dry,
            detector_controls,
            active_voice_indices,
            active_voice_count);

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
    pitch_detector.ConfigureAnalysisScheduler(
        detector_decimation_factor,
        analysis_period_detector_samples);

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
        resonator_frequencies[resonator_index] =
            pitch_detector.GetFrequencyAtIndex(resonator_index);
    }

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
