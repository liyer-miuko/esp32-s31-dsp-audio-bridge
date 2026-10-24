# Third-party notices

Third-party code retains its original license and copyright notices. The root noncommercial license does not apply to third-party rights. This inventory describes the publication snapshot, not a blanket legal clearance for a combined firmware product.

| Component | Upstream / license evidence | Treatment |
| --- | --- | --- |
| ESP-IDF v6.1 fork | [WillyBilly06/ESP32-S31-A2DP-Codecs](https://github.com/WillyBilly06/ESP32-S31-A2DP-Codecs); ESP-IDF, Broadcom/AOSP/Espressif file notices | Full SDK excluded. Only scoped, identified changes with license evidence included. Apache text in `LICENSES/Apache-2.0.txt`. |
| cJSON 1.7.19 | [DaveGamble/cJSON](https://github.com/DaveGamble/cJSON/tree/v1.7.19), MIT | Exact sources, LICENSE and original source identities included under `firmware/components/amp_json`. |
| TinyUSB 0.21.0~1 | [espressif/tinyusb](https://github.com/espressif/tinyusb), MIT | Two modified component files and license included; original component resolved separately. Descriptors retain in-file MIT notices, including Jerzy Kasenbreg's notice. |
| freeaptx | Source header Copyright 2017 Aurelien Jacobs, 2018–2020 Pali Rohár, 2021 Hunter Wardlaw; LGPL-2.1-or-later | Actual compiled `main/freeaptx.c` takes precedence over conflicting parent LICENSE. Complete library source/header/build files and COPYING are included in the small SDK overlay. Modifications retain LGPL. No combined firmware binary is distributed. |
| Helix AAC | RealNetworks copyright and RPSL/RCSL in compiled `aacdec.c` | Decoder is not redistributed in this snapshot. [RPSL sections 4.1/4.2](https://opensource.org/license/RPSL-1.0) constrain combined works. No separate RCSL grant is assumed. Current binary withheld. |
| LDAC decoder | [O2C14/libldac-dec](https://github.com/O2C14/libldac-dec), via original fork | Local source lacks an explicit redistribution license. Do not mistake AOSP's LDAC encoder license for a license to this decoder. Implementation/deltas and binary withheld pending evidence. |
| LHDC | Removed/disabled in current app; local fork contains library material | No library or implementation changes published. No 192 kHz support claimed. |
| LC3 / LC3plus wrapper | Local liblc3 LICENSE Apache-2.0; vendor wrapper notices | Wrapper changes with explicit Apache notices included. Neither source availability nor this wrapper establishes a vendor patent license or interoperability certification. |
| Espressif controller, PHY and other SDK libraries | Per-library upstream terms | Not bundled. Obtain from the pinned SDK under those terms; notices alone are not a replacement for permission. |

SDK changes retain the upstream license blocks, authors and comments. Project changes relative to the external base are listed in `sdk/manifest.json`; omitted entries are explicit. The linked artifact identity is documented for provenance only, not offered for download.

This repository publishes source and WebUI only. Users obtaining external dependencies must inspect those dependencies and any intended combination before building or redistributing. The maintainer remains responsible for license obligations attached to material distributed here; local building does not transfer that responsibility to somebody else. “Third party” does not mean “official vendor-authorized,” and no such authorization is inferred.

The names ESP32, Bluetooth, aptX, LDAC, LHDC, LC3plus and TAS belong to their respective owners. Listing them is identification, not endorsement, certification or a grant of commercial rights.
