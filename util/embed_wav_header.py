#!/usr/bin/env python3

import argparse
import io
from pathlib import Path
import wave


def convert_wav_bytes(input_path: Path, target_rate: int, max_seconds: float) -> bytes:
    with wave.open(str(input_path), "rb") as wav_in:
        channels = wav_in.getnchannels()
        sample_width = wav_in.getsampwidth()
        source_rate = wav_in.getframerate()
        frame_count = wav_in.getnframes()

        raw = wav_in.readframes(frame_count)

    bytes_per_frame = sample_width * channels
    total_src_frames = len(raw) // bytes_per_frame

    def read_pcm_sample(frame: int, channel: int) -> int:
        offset = frame * bytes_per_frame + channel * sample_width
        sample_bytes = raw[offset:offset + sample_width]

        if sample_width == 1:
            # 8-bit PCM WAV is unsigned.
            return (sample_bytes[0] - 128) << 8
        if sample_width == 2:
            return int.from_bytes(sample_bytes, byteorder="little", signed=True)
        if sample_width == 3:
            # Sign-extend 24-bit little-endian.
            sign = 0xFF if (sample_bytes[2] & 0x80) else 0x00
            extended = sample_bytes + bytes([sign])
            return int.from_bytes(extended, byteorder="little", signed=True) >> 8
        if sample_width == 4:
            # 32-bit PCM integer; reduce to int16-ish range.
            return int.from_bytes(sample_bytes, byteorder="little", signed=True) >> 16

        raise ValueError(f"Unsupported PCM sample width: {sample_width} bytes")
    max_target_frames = int(target_rate * max_seconds)
    ratio = source_rate / target_rate

    mono_samples = []
    target_frames = min(max_target_frames, int(total_src_frames / ratio))
    for target_index in range(target_frames):
        src_frame = int(target_index * ratio)
        if src_frame >= total_src_frames:
            break

        if channels == 1:
            mono_value = read_pcm_sample(src_frame, 0)
        else:
            channel_sum = 0
            for channel in range(channels):
                channel_sum += read_pcm_sample(src_frame, channel)
            mono_value = int(channel_sum / channels)

        mono_samples.append(max(-32768, min(32767, mono_value)))

    mono_bytes = b"".join(int(value).to_bytes(2, byteorder="little", signed=True) for value in mono_samples)

    out_buffer = io.BytesIO()
    with wave.open(out_buffer, "wb") as wav_out:
        wav_out.setnchannels(1)
        wav_out.setsampwidth(2)
        wav_out.setframerate(target_rate)
        wav_out.writeframes(mono_bytes)

    return out_buffer.getvalue()


def write_header(data: bytes, output_path: Path, symbol: str) -> None:

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="ascii") as out:
        out.write("#pragma once\n\n")
        out.write("#include <cstdint>\n\n")
        out.write("__attribute__((used, aligned(4)))\n")
        out.write(f"const unsigned char {symbol}[] = {{\n")

        bytes_per_line = 12
        for start in range(0, len(data), bytes_per_line):
            chunk = data[start:start + bytes_per_line]
            values = ", ".join(f"0x{b:02x}" for b in chunk)
            out.write(f"    {values},\n")

        out.write("};\n")
        out.write(f"const unsigned int {symbol}_len = {len(data)};\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Embed a WAV file as a C header")
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--symbol", required=True)
    parser.add_argument("--target-rate", type=int, default=12000)
    parser.add_argument("--max-seconds", type=float, default=1.8)
    args = parser.parse_args()

    wav_bytes = convert_wav_bytes(
        Path(args.input),
        target_rate=args.target_rate,
        max_seconds=args.max_seconds,
    )
    write_header(wav_bytes, Path(args.output), args.symbol)


if __name__ == "__main__":
    main()
