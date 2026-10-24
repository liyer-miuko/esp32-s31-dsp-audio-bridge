# License scope / 许可范围

Required Notice: Original project contributions Copyright 2026 liyer-miuko and respective contributors.

The root LICENSE is the unmodified **PolyForm Noncommercial License 1.0.0**. It applies only to original contributions the contributors are entitled to license: original application/control code, WebUI, documentation and new standalone framework code. It applies only to rights that exist and can be granted; AI attribution does not establish exclusive ownership or third-party permission.

It does **not** relicense third-party material, erase notices or take away existing rights in upstream code. For mixed files, original upstream portions retain their original license; project-original additions use PolyForm only where upstream terms permit. Where a third-party license requires modifications to remain under that license, that requirement prevails (for example, the modified freeaptx library remains LGPL-2.1-or-later).

| Material | Terms |
| --- | --- |
| Original firmware application, WebUI and docs | PolyForm-Noncommercial-1.0.0, to the extent licensable |
| `firmware/components/amp_json/cJSON.c`, `.h`, LICENSE | MIT; upstream notices retained |
| Third-party TinyUSB-derived descriptors in `firmware/main/` | MIT notices in those files; original portions remain MIT |
| ESP-IDF / Bluedroid / AOSP portions in SDK overlay | Apache-2.0 or the original file's stated terms; preserve upstream rights and notices |
| `sdk/overlay/.../libfreeaptx/main/` | LGPL-2.1-or-later as stated in actual compiled source, including modifications; not the conflicting CC0 label in a parent directory |
| TinyUSB component overlay | MIT for upstream portions; project additions only under the root license where compatible |
| Omitted Helix AAC, LDAC, LHDC implementations | Not distributed here, not relicensed here; no grant is inferred from their being online |

This is a **noncommercial source-available** project. CC BY 3.0 permits commercial use and is not used. PolyForm is software-oriented; its own definitions of permitted personal, noncommercial and organizational uses control. Commercial use outside those permissions needs a separate grant from the relevant rights holder. The maintainer cannot grant rights belonging to third parties.

Patent, trademark and certification rights are not created by a copyright notice. Nothing in this repository promises a commercial codec license, Bluetooth qualification or regulatory certification.

To preserve these boundaries the existing mixed-license binary is withheld. A compatible replacement codec, a valid separate license, or a different distribution architecture must resolve the conflict before a binary release. A general “at your own risk” statement cannot replace those obligations.
