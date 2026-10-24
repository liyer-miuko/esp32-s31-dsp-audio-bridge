# ESP32-S31 DSP Audio Bridge

[简体中文](README.md) · [License scope](LICENSE_SCOPE.md) · [Build](docs/BUILD.md)

An experimental DIY USB/Bluetooth audio bridge with four-channel DSP and dual I²S outputs. Much of the code was written or changed by AI models, including Codex/GPT and DeepSeek, under the project owner's direction. **No guarantee is made that any function works correctly, safely, continuously, or on your hardware.**

Original project contributions are available for noncommercial purposes under PolyForm Noncommercial 1.0.0. Third-party material retains its own terms. This is source-available software, **not OSI open source**.

## Features

- Asynchronous USB UAC input up to 96 kHz / 32-bit.
- Classic Bluetooth A2DP input, preferred when both sources have audio.
- Input EQ, routing, four DSP channels, PEQ, HP/LP filters, limiter, loudness, compressor, output gain and delay.
- Two stereo I²S outputs up to 96 kHz. Shared APLL does not guarantee frame phase alignment.
- Standalone English/Chinese HTML control page with light/dark themes, Web Serial and Web Bluetooth.
- Device configuration, five presets, separate full-configuration and DSP-parameter JSON files.
- Optional core0 I²C amplifier control with basic TAS5828M/TAS5825M templates; disabled by default and not tested with physical I²C amplifier hardware.

SBC, AAC, aptX and aptX HD have implementation/history evidence, not a universal compatibility guarantee. aptX LL and LC3plus A2DP require a suitable source and remain unverified where none was available. LHDC is removed. LDAC is not enabled for negotiation by default because sustained playback failed core0 performance limits. Adaptive/Lossless are not provided. BLE is for control, not LE Audio.

## Use and safety

Use an **ESP32-S31**, not an ESP32-S3. Read [hardware wiring](docs/HARDWARE.md) before connecting a DAC or amplifier. Begin at low volume. Verify voltage levels, pins, output format, routing and amplifier coefficients. Do not use this experimental code in safety-critical systems.

Open or download [webui/dsp webui.html](webui/dsp%20webui.html). The page has no external assets; device access still requires browser/OS API support. If local-file access is restricted, use a trusted localhost environment. Read the device configuration before editing it. Importing a file does not automatically save NVS. Amplifier configuration is separate.

## Publication status

The application source snapshot is based on firmware 0.254. This repository intentionally excludes the full SDK, analyzers, captures, local credentials, historical archives and private workflow records.

**No flashable binary is published, and this subset is not claimed to reproduce 0.254 independently.** The existing binary links Helix AAC (RPSL), whose combined-work terms conflict with the intended noncommercial project license; the LDAC decoder's redistribution grant also remains unresolved. Required unresolved SDK files are identified, not silently replaced or relicensed. See [firmware release status](firmware/releases/README.md) and [third-party notices](THIRD_PARTY_NOTICES.md).

Build the standalone UI with `node webui/build.cjs`. Firmware build requirements and limitations are in [BUILD.md](docs/BUILD.md). Contributions must preserve third-party licenses and the frozen core1 source/interface/object checks.

Codec copyright, patent, trademark, certification and commercial distribution rights are separate. This repository grants no rights owned by codec vendors and provides no warranty of non-infringement. See [LICENSE](LICENSE), [LICENSE_SCOPE.md](LICENSE_SCOPE.md), [CONTRIBUTING.md](CONTRIBUTING.md) and [SECURITY.md](SECURITY.md).
