# Core0 I2C amplifier control

Default: disabled. The present test amplifier uses hardware mode. Do not enable
I2C until the amplifier is wired for software control. Default pins are SDA9 /
SCL10. GPIO6 is reserved in this project. Both I2S pin groups and GPIO8 remain unchanged.
Use external pull-ups compatible with the amplifier I/O voltage.

No PCM samples are read or changed here. The existing hardware-mute state
machine supplies the logical mute request: UAC **or** decoded BT PCM activity,
including all-zero PCM, with the existing start/stop delays. A connection alone
is inactive. GPIO polarity affects only GPIO8, not amplifier register values.
The new worker is core0, priority 1. I2C calls have a 5 ms timeout. The default
hardware-mode configuration creates no bus, worker, JSON buffer or profile heap.
Validated enabled profiles allocate these resources only when needed. Disabling
the last amplifier first attempts its old mute function, closes the bus and
releases the worker/model. Failed I2C is FAULT with mute UNKNOWN, without an
automatic retry loop. A new explicit profile application can recover it.

## JSON schema, version 1

The document contains exactly `version`, `bus`, and two `ports`. Bus fields:
`enabled` (boolean), `sda`, `scl`, `hz` (100000 or 400000). Port fields: `port`
(0/1, unique), `enabled`, `model` (1..23 ASCII letters/digits/underscore/hyphen),
`address` (unshifted 7-bit, 8..119), and `functions`.

Functions `startup`, `mute`, `unmute` are mandatory and nonempty. Four additional
named functions are permitted. A port has at most 96 operations in total.

| Operation | Meaning |
| --- | --- |
| `{"write":[reg,byte,...]}` | Register write, 1..24 data bytes; cannot wrap 255 |
| `{"update":[reg,mask,value]}` | Read/modify/write; value must fit mask |
| `{"check":[reg,mask,value]}` | Read and require `(read & mask) == value` |
| `{"delay_ms":5}` | Worker-only delay; max 1000 ms per operation |

Startup delay sum <=2000 ms; other function delay sum <=100 ms. Input is limited
to 8192 UTF-8 bytes, depth 12. Duplicate/unknown keys, missing fields, fractional
integers, unsafe pins, duplicate active addresses and escaped NUL are rejected.
Numbers are decimal, not strings such as `"0x4c"`. This is declarative register
data, not executable JavaScript/C. Runtime allocation failure cannot update the
active configuration. Validation temporarily needs a staging model and parser
tree; the maximum wire size is not a guarantee of available heap during decoding.

The TAS5825M/5828M templates select book/page 0, enter HiZ with DSP enabled and
soft mute, wait >=5 ms, and enter Play while still muted. The clocks are already
running when the worker starts. Mute updates **bit3** of DEVICE_CTRL2 (0x03);
bit4 is DSP reset, not a second channel mute. Reserved bits are preserved.
PPC3 board-specific coefficients, modulation, power-stage and speaker settings
are not invented by these basic templates. Both output ports can have separate
profiles and addresses on the shared bus.

References: [TAS5825M](https://www.ti.com/lit/ds/symlink/tas5825m.pdf),
[TAS5828M](https://www.ti.com/lit/ds/symlink/tas5828m.pdf), register 0x03 and
startup sequence. cJSON 1.7.19 is local MIT-licensed source in components/amp_json.

## Wire protocol

Existing AA55 / u8 length / CRC8 framing. Payload maximum 255. DSP BULK v5 and
the frozen shared configuration ABI are unchanged. UART and BLE enter the same
proto_task. The control lease, BLE generation and upload session fence apply.
Responses use `[B8, op|80, token, status, ...]`. Token is u8.

| op | Request after `[B8,op,token]` | Response data after status |
| --- | --- | --- |
| 0 | none | 55-byte complete STATUS payload (layout below) |
| 1 | offset u16 LE | total u16, offset u16, generation u32, <=243 JSON bytes |
| 2 | total u16, expected generation u32 | none; starts RAM staging only |
| 3 | offset u16, contiguous JSON bytes | none; requires same session/token |
| 4 | persist u8 (0 RAM / 1 independent amp NVS) | new generation u32 |
| 5 | port u8, function index u8 (3..6) | queued ACK; result via STATUS |
| 6 | none | remove saved amp profile only; RAM unchanged |

All integer multibyte fields are little-endian. Upload timeout is 10 seconds
since the last accepted chunk. Incomplete/stale/out-of-order uploads cannot
commit. The host lease ending aborts an incomplete upload. BEGIN is not an ACK
for successful hardware initialization. COMMIT ACK is validation/storage/queue
acceptance; the host must compare requested/applied generation, busy and each
port's state/error. A failed saved profile is retained for correction; boot uses
disabled defaults and reports an error in last_result with last_token=0.

STATUS: B8,80,token,status at 0..3; schema=1 at4; requested generation u32 at5;
processed generation u32 at9; busy at13; last job token/result at14/15; PCM active
at16; requested logical mute at17. Each port has 18 bytes from18 and36:
state u8, confirmed mute u8, error i32, successful write/read/function counts u32.
State 0 disabled /1 starting /2 ready /3 fault. Mute 2 means unknown; 0/1 means
the register command completed, not measured analog output. Saved-profile-present
is byte54. Result 0 OK /1 length /2 value /3 busy /4 stale /5 save /6 I2C /7 memory.

For concurrent UART/BLE requests the owner receives its own replies; another
transport gets BUSY. DSP request IDs generated by the new frontend skip B8.
Legacy DSP requests whose ID is B8 are still dispatched by their param envelope.
Raw DSP BULK continuations are assembled before extension response parsing.

## Storage and rollback

Amplifier data uses NVS namespace `amp_i2c`, key `profile_v1`. DSP A5/A8, DSP
presets and DSP JSON do not save, replace or reset it. Amplifier op6 deletes only
that key. Ordinary APP upgrades do not change NVS. Before downgrading to a version
without this feature, mute/disable an attached I2C amplifier explicitly; old
firmware cannot control it. Hardware-mode amplifiers retain their original path.
No core1 source, object, ABI, bootloader, partition or DSP math change is required.
