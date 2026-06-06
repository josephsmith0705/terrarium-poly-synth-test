# Terrarium Poly Synth — Complex Resonator + Additive Synthesis Engine

Firmware for a polyphonic guitar-to-synth effect on Daisy Seed + PedalPCB Terrarium.

## Approach

Based on: Zhou, Reiss, Mattavelli, Zoia — *"A Computationally Efficient Method for Polyphonic Pitch Estimation"*.

**Phase 1 — Resonator bank (per sample):** 51 complex resonators tuned to every semitone from D2 to E6 run continuously:

$$z_k[n] = x[n] + r \cdot e^{j\omega_k} \cdot z_k[n-1]$$

The smoothed energy $|z_k|^2$ at each resonator builds up wherever the guitar has spectral content — including all harmonics of every string in a chord.

**Phase 2 — Fundamental detection (every N samples):** The energy profile is peak-picked and harmonic-suppressed to identify up to 4 fundamental pitches. This is the only step that uses a threshold — and it only determines *which frequencies are fundamentals*, not amplitude.

**Phase 3 — Additive synthesis (per sample):** For each detected fundamental, 8 sine-wave partials are synthesised at integer multiples of the fundamental. Each partial's amplitude is read **directly from the resonator energy at that harmonic frequency** — not from an independent threshold gate. This means:

- Partials never pop on or off — they smoothly fade as the resonator energy at each harmonic changes.
- The synth's timbral profile naturally matches the guitar's harmonic content.
- Attack and release of each harmonic follow the guitar's own dynamics at that frequency.

## Controls

### Knobs

| Knob | Function | Range |
|---|---|---|
| 1 Radius | Resonator memory — frequency resolution vs. response speed | 0.990 (≈20 ms τ) to 0.99995 (≈417 ms τ) |
| 2 Threshold | Peak-detection threshold relative to loudest resonator | 1% to 90% of peak energy |
| 3 Analysis Window | How many samples accumulate before voice update — trades latency for stability | 128 samples (2.7 ms) to 2048 samples (42 ms) |
| 4 Onset Sensitivity | Multiplier above slow average that triggers a resonator reset | Gentle to sharp |
| 5 Harmonic Tolerance | How strictly a candidate must match an integer multiple to be suppressed | 3% to 28% fractional deviation |
| 6 Crossover | LP/HP blend crossover frequency (active when Toggle 3 is up) | 80 Hz to 900 Hz |

### Toggle Switches

| Toggle | Down | Up |
|---|---|---|
| 1 Harm Suppress | All peaks output as voices | Harmonic peaks suppressed — cleaner chord tracking |
| 2 Onset Reset | Resonators accumulate freely | Reset on each note attack — faster note changes |
| 3 LP/HP Blend | Pure synth output | LP(guitar) + HP(synth) — guitar body + synth harmonics |
| 4 Carrier Blend | Pure oscillator synthesis | Blend in hard-clipped guitar for extra grit |

### Footswitches and LEDs

- **Left stomp**: bypass. LED 1 lit when active.
- **Right stomp**: unused.
- **LED 2**: tracks Radius knob (dim = fast/short memory, bright = slow/long memory).

## LP/HP Blend (Toggle 3)

When enabled, the output becomes a crossover mix:

- **LP(guitar)** — the dry guitar signal passed through a 1-pole low-pass. Preserves the pick attack, body, and fundamental pitch of the input.
- **HP(synth)** — the synthesiser output passed through the same crossover as a high-pass. Provides the clean upper harmonics and overtones from the resonator bank.

The crossover frequency (Knob 6) controls where the split happens. Around 200–400 Hz works well for standard tuning: the guitar provides the low-register anchor while the synth takes over the harmonic register above it. Both LP states are always maintained in the background so there are no clicks when toggling blend on or off.

## Building

    cmake \
        -GNinja \
        -DTOOLCHAIN_PREFIX=/usr/local \
        -DCMAKE_TOOLCHAIN_FILE=lib/libDaisy/cmake/toolchains/stm32h750xx.cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -B build .
    cmake --build build

Firmware for a guitar-to-synth effect on Daisy Seed + PedalPCB Terrarium.

## Approach

Rather than tracking pitch, this firmware uses a **spectral self-vocoder**:

1. The guitar input is driven and clipped to produce a square-wave-like carrier (rich in odd harmonics, following the guitar's pitch and chords automatically — no pitch detection needed).
2. The same input (raw or optionally driven) feeds a bank of bandpass filters as the modulator, extracting a per-band amplitude envelope.
3. The carrier is filtered through the same bank and each band is scaled by the corresponding modulator envelope.
4. Result: the guitar's harmonic energy is imprinted onto the square-wave carrier, producing a synthesiser-like texture that naturally tracks chords.

## Controls

### Knobs

| Knob | Function | Range |
|---|---|---|
| 1 Drive | Pre-gain into the carrier clip path | 1× to 24× (log) |
| 2 Clip Shape | Blends carrier from warm tanh to hard square wave | 0 = soft, 1 = square |
| 3 Bands | Number of active vocoder bands | 1 to 16 |
| 4 Attack | Per-band envelope follower attack time | ~0.5 ms to ~50 ms |
| 5 Release | Per-band envelope follower release time | ~2 ms to ~200 ms |
| 6 Band Shift | Shifts all band centres up or down within the guitar range | −1 oct to +1 oct |

### Toggle Switches

| Toggle | Down | Up |
|---|---|---|
| 1 Wide Q | Narrow bands (high selectivity) | Wide overlapping bands |
| 2 Sub Blend | Off | Add octave-down frequency divider |
| 3 Dirty Mod | Clean modulator (raw guitar envelope) | Drive applied to modulator too |
| 4 Invert Even | Normal phase per band | Even-numbered bands phase-inverted (hollow/comb texture) |

### Footswitches and LEDs

- **Left stomp**: bypass. LED 1 lit when effect is active.
- **Right stomp**: unused.
- **LED 2**: brightness tracks active band count (dim = 1 band, bright = 16 bands).

## DSP Notes

- Guitar range hard limits: **73 Hz – 1320 Hz** (drop-D open string to 24th fret high-E).
- Band centres are log-spaced within those limits. Band Shift knob moves both endpoints by the same factor, preserving the width ratio.
- Filter reconfiguration only runs when a knob affecting bands or envelope actually changes, keeping CPU overhead minimal during steady-state playing.
- At 16 bands (knob 3 maximum), per-sample cost is ~2% CPU on a 480 MHz Daisy Seed at 48 kHz.

## Building

    cmake \
        -GNinja \
        -DTOOLCHAIN_PREFIX=/usr/local \
        -DCMAKE_TOOLCHAIN_FILE=lib/libDaisy/cmake/toolchains/stm32h750xx.cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -B build .
    cmake --build build

This build is temporarily focused on tracking experiments rather than final voicing. The dry path is removed while the effect is enabled, the output filter is bypassed, and the old envelope mode control is gone so the panel can be used for tracking and band-layout tuning.

## Current DSP Focus

- Poly-only oscillator resynthesis
- Adjustable pitch-tracking band count (1–12)
- Adjustable vocoder band count (1–12)
- Adjustable global analysis range shift
- Adjustable global analysis range width
- Optional vocoder modulation from the guitar input
- **Harmonic suppression:** loudest band is treated as the candidate fundamental; any other band whose tracked frequency is an integer multiple (×2–×7, ±10%) of a louder confirmed fundamental is muted that block
- **Guitar hard cutoff:** frequency candidates outside the physical note range of a standard/drop-D guitar (73 Hz – 1320 Hz) are rejected unconditionally, regardless of band placement, so sub-harmonics and high harmonics never bleed into the output

At 100% on the relevant knobs, the firmware uses the current maximum configured band counts:

- Pitch tracking bands: 12
- Vocoder bands: 12

All frequency tracking is hard-clamped to the note range of a standard guitar (73 Hz – 1320 Hz, covering drop D up to the 24th fret of the high E string). Frequency candidates outside this range are rejected before any slew or phase update occurs.

## Controls

### Knobs

#### Knob 1 - Sensitivity
Tracking threshold. Lower values make more bands wake up on quieter material.

#### Knob 2 - Slew
Pitch-update speed. Lower values smooth detector changes more heavily; higher values chase note changes faster.

#### Knob 3 - Pitch Tracking Bands
Number of active tracking bands from 1 to 12.

#### Knob 4 - Vocoder Bands
Number of active vocoder analysis bands from 1 to 12.

#### Knob 5 - Range Shift
Moves the full tracking and vocoder range lower or higher.

#### Knob 6 - Range Width
Shrinks or expands the overall frequency span covered by the active bands.

### Toggle Switches

#### Toggle 1 - Overlap
- `Down`: narrower, stricter per-band ranges
- `Up`: wider, more overlapping per-band ranges

#### Toggle 2 - Range Focus
- `Down`: low-focus coarse shift
- `Up`: high-focus coarse shift

#### Toggle 3 - Voice Sum
- `Down`: strongest detected band dominates the output
- `Up`: all active tracking bands are summed

#### Toggle 4 - Vocoder
- `Down`: off
- `Up`: on

### Footswitches and LEDs

#### Left Stomp
Effect bypass.

#### Right Stomp
Currently unused.

#### LED 1
Effect enabled indicator.

#### LED 2
Shows analysis state. It is brightest when the vocoder is enabled; otherwise it scales with the active tracking-band count.

## Vocoder Note

The vocoder does not replace pitch tracking. It uses the guitar input as the modulation source and the synth bank as the carrier. That means the synth still gets its base pitch from the tracking bands, while the vocoder helps imprint the input's spectral profile onto the generated voices.

## Building

    cmake \
        -GNinja \
        -DTOOLCHAIN_PREFIX=/usr/local \
        -DCMAKE_TOOLCHAIN_FILE=lib/libDaisy/cmake/toolchains/stm32h750xx.cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -B build .
    cmake --build build
