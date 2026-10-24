/*
 * DSP parameter store - ESP32-S31 audio bridge (v11, protocol layer).
 *
 * The store validates WebUI/protocol writes on core0. Accepted configurations
 * are applied at a core1 block boundary; PCM stays in the core1 output chain.
 *
 * Layout of the model mirrors the WebUI signal flow:
 *
 *   inputs[2] -> route[2][4] -> channels[4] (PEQ + HP/LP) -> master
 *     -> ports[2] (声道映射 -> 端口 HP/LP -> 端口增益 -> 延时)
 *
 * Every field is validated and clamped on write, so a hostile or buggy host
 * cannot push the DSP out of range. Floats are only ever stored/compared,
 * never used as bit patterns.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DSP_NUM_INPUTS      2
/* 4 条 DSP 通道 = 每个端口的左右两个槽：
 *   CH1 = 端口0 左(I2S0 L) / CH2 = 端口0 右(I2S0 R)
 *   CH3 = 端口1 左(I2S1 L) / CH4 = 端口1 右(I2S1 R)
 * 端口 p 取 CH(2p+1) 作左槽、CH(2p+2) 作右槽。这样"i2s0 低通 + i2s1 高通"
 * 就是各通道自己的高通/低通，端口级不再需要滤波。 */
#define DSP_NUM_CHANNELS    4
#define DSP_NUM_PORTS       2
#define DSP_MAX_BANDS       15
#define DSP_DEFAULT_BANDS   10

/* 单个 PEQ / 房间 EQ 频段的参数范围 —— **唯一权威定义**。
 * 三处必须一致：写入校验（dsp_config_set_param）、BULK 反序列化（deserialize_band）、
 * 滤波器设计（dsp_biquad_design）。前端面板 / REW 导入 / JSON 归一化 / 曲线拖拽也用同一组值。
 * 历史上这里是 ±15，而前端 JSON 归一化路径夹 ±12 → "导出再导入把 +15 悄悄变 +12"（v23 统一为 ±12）。 */
#define DSP_BAND_FREQ_MIN_HZ   20.0f
#define DSP_BAND_FREQ_MAX_HZ   20000.0f
#define DSP_BAND_GAIN_MIN_DB   (-12.0f)
#define DSP_BAND_GAIN_MAX_DB   (12.0f)
#define DSP_BAND_Q_MIN         0.4f
#define DSP_BAND_Q_MAX         10.0f

/* Serialized configuration blob (SYNC_CONFIG bulk + NVS image).
 * Version history: 1 = first protocol release (XO records were 10 bytes, a
 * serialization bug); 2 = XO records are exactly 8 bytes; 3 = port records are
 * 32 bytes and carry the port-level HP/LP crossovers; 4 = limiter section;
 * 5 = 4 DSP channels (one per port slot). A stored blob with a different
 * version is ignored and defaults are used, so a format change can never be
 * silently mis-parsed. */
#define DSP_CONFIG_MAGIC        0x44535043u   /* 'DSPC' */
#define DSP_CONFIG_VERSION      5

/* 序列化长度 = 298 + 16 × B，B = Σ输入(n_eq + n_req) + Σ通道(n_eq)。
 * 满配 B = 2×(15+15) + 4×15 = 120 → 2218 B；这里按公式推导并向上取整到 2304，
 * 避免以后再改布局时忘记同步（旧值 2048 < 2218：EQ 段数拉满会让 BULK/保存/预设
 * 全部静默失败）。WebUI 的 parseBulk 也按同一公式推进偏移。 */
#define DSP_CONFIG_BLOB_BANDS_MAX  (DSP_NUM_INPUTS * 2 * DSP_MAX_BANDS + DSP_NUM_CHANNELS * DSP_MAX_BANDS)
#define DSP_CONFIG_BLOB_MAX        (298u + 16u * DSP_CONFIG_BLOB_BANDS_MAX + 86u)   /* 2218 -> 2304 */

/* 固件版本号已搬到 **dsp_version.h**（v33 / 固件 0.32）——
 * 原因：本头文件经 dsp_engine.h 被 core1 的 dsp_engine.c 引用，
 * 版本宏留在这里会让"每次升版本"都触发 core1 重编译、基线漂移。
 * 需要版本号的文件请直接 `#include "dsp_version.h"`。 */

/* 设备端预设槽（存在 NVS 里，换浏览器/换手机也能用）。
 * WebUI 侧 PRESET_SLOTS = 5，两边必须一致（槽位掩码是 u8，天然支持 8 槽）。
 * 容量提醒：NVS 分区 0x6000 = 24 KB，5 个满配（2218 B）预设 + 主配置约占 18 KB，
 * 可用约 20 KB —— 够用但余量不大；要加到 8 槽必须先扩 NVS 分区。 */
#define DSP_PRESET_SLOTS        5

/* S31 HWLOOP 生产预算（0.122 成本模型；0.125 保留模型，区分目标与期限）。
 * 仍按 core0 填块 -> core1 DSP/端口提交的整链、96k/240帧计；不沿用旧净DSP口径。
 * 0.121 task62 既有串口四点差值折算：mono11.2us；pair最多16.35us。
 * 分别取13/19us，单位成本留约16%余量；固定567+51+(comp?155:0)保持。
 * 参考与推导：工作区 manager_task64/model.json，不修改历史 task54 模型。
 *
 * N 是界面计数。最坏布局先填38个立体声位置（输入30+端口0分频8），
 * 通道PEQ/HP/LP按CH1/2、CH3/4分别配对；每对、每组先填单侧才最贵。
 * 最多46个通道单侧位置。之后增加一个通道段，mono少一次、pair多一次：
 *   N<=38:    C(N)=19N
 *   38<N<=84: C(N)=19*38+13*(N-38)
 *   84<N<=130:C(N)=19*38+13*46+6*(N-84)=816+6N
 * 130段模型估计：关压缩2214us；开压缩2369us。130为物理界面最大计数。
 * 2250us仍是生产设计目标；压缩档的模型超目标在core0编译时提示，
 * 2500us仍是该档编译期拒绝线。模型检查不保证实际每块均在期限内。
 * 不重新标定单价、固定项或预算百分比；响度和削波恒预留，不增大缓冲。
 */
#define DSP_BLOCK_US_96K        2500u   /* 240 帧 @96 kHz */
#define DSP_BUDGET_PCT          90u     /* 只用 90% 块周期：峰值 max_us 比 avg 高约 2~6%，
                                         * 再留一点给 ISR/日志/遥测抖动（实测 max/avg ≤1.03）。
                                         * ⚠ **不许为了好看放宽这个系数**。 */

#define DSP_FIXED_BASE_US       567u    /* 全链路固定项（含最强削波预留与额外抖动） */
#define DSP_FIXED_LOUD_US       51u     /* 响度补偿：恒预留 */
#define DSP_FIXED_COMP_US       155u    /* 压缩器：只在开启时预留 */
#define DSP_PER_BIQUAD_US       13u     /* 一次mono调用，240帧 */
#define DSP_PER_PAIR_US         19u     /* 一次pair调用，两路各240帧 */
#define DSP_STEREO_SLOTS        (2u * DSP_MAX_BANDS + 8u)
#define DSP_CHANNEL_MONO_SLOTS  ((DSP_NUM_CHANNELS / 2u) * (DSP_MAX_BANDS + 8u))
#define DSP_BIQUAD_UI_MAX       (DSP_STEREO_SLOTS + DSP_NUM_CHANNELS * (DSP_MAX_BANDS + 8u))
#define DSP_PRODUCTION_BUDGET_US (DSP_BLOCK_US_96K * DSP_BUDGET_PCT / 100u)

/* 仅用于常量表达式/编译期核对，不在PCM热路径估算布局。n无副作用。 */
#define DSP_BIQUAD_WORST_US(n) \
    ((n) <= DSP_STEREO_SLOTS ? (n) * DSP_PER_PAIR_US : \
     (n) <= DSP_STEREO_SLOTS + DSP_CHANNEL_MONO_SLOTS ? \
        DSP_STEREO_SLOTS * DSP_PER_PAIR_US + ((n) - DSP_STEREO_SLOTS) * DSP_PER_BIQUAD_US : \
        DSP_STEREO_SLOTS * DSP_PER_PAIR_US + DSP_CHANNEL_MONO_SLOTS * DSP_PER_BIQUAD_US + \
        ((n) - DSP_STEREO_SLOTS - DSP_CHANNEL_MONO_SLOTS) * (DSP_PER_PAIR_US - DSP_PER_BIQUAD_US))

/* 统一准入，不针对当前摆放位置浮动。固件与WebUI同步。
 *
 * ★ 0.123两档统一到130；0.125仅明确模型与证据口径，准入值不变。
 *   既有证据：工作区handoff/evidence/verification_130ui.json，设备0.123，
 *   96k/32各30s，130段无压缩1758us、三开2026us（已剔除prep>5000样本）。
 *   此记录不验收0.124端口修补、最大效果强度、所有启动样本或长期运行。
 *   task65/66原130声明有计数错误，不再用其5.4us/段推断全拓扑成本。
 *   当前单价与固定项仍是工程估计，不用整链拟合斜率替换mono/pair成本。
 *   若实际越设计线/期限，按整链记录定位；不自动降级或靠环缓存掩盖。 */
#define DSP_LIMIT_PRODUCTION_COMP    DSP_BIQUAD_UI_MAX
#define DSP_LIMIT_PRODUCTION_NOCOMP  DSP_BIQUAD_UI_MAX

/* ---------------- DSP 精度模式（v27 新增，参数 0x80 / blob 尾部 limAt+30） ----------------
 * 0 = 高精度（历史口径 41 段）
 * 1 = **扩展（历史口径 82 段，压缩器禁用）—— v28 起的出厂默认**
 * ⚠ 上面41/82是历史数字；当前两档见上方生产模型，精度模式不改变准入。
 * 非法值一律钳到 0。
 *
 * ★ v28 把默认从"高精度"改成"扩展"，理由（用户 2026-09-16 口径）：
 *   **这个音频桥的默认形态应当是"82 段、各功能全通、唯独不支持压缩器"**。
 *   压缩器（夜间模式）对普通用户需求不大，但**功能必须保留**，所以做成
 *   "想用压缩器 ⇒ 段数上限降到 41"这个**显式**取舍，而不是让用户去挑模式。
 *   ⚠ 旧 blob（该字节为 0）会读成"高精度" —— 那是**用户自己存过**的配置，保持不动；
 *     只有 `RESET_CONFIG` / 首次烧录才落到新的扩展默认。 */
#define DSP_PRECISION_HIGH      0
#define DSP_PRECISION_EXTENDED  1
#define DSP_PRECISION_MAX       DSP_PRECISION_EXTENDED
#define DSP_PRECISION_DEFAULT   DSP_PRECISION_EXTENDED

/* ---------------- Block ids (must match the WebUI) ---------------- */
#define DSP_BLOCK_INPUT     0x01
#define DSP_BLOCK_OUTPUT    0x02
#define DSP_BLOCK_ROUTING   0x03
#define DSP_BLOCK_GLOBAL    0x04
#define DSP_BLOCK_SYSTEM    0x05

/* ---------------- Parameter ids (must match the WebUI) ---------------- */
#define DSP_P_GAIN          0x01
#define DSP_P_MUTE          0x02
#define DSP_P_PHASE         0x03
#define DSP_P_EQ_ON         0x10
#define DSP_P_EQ_FREQ       0x11
#define DSP_P_EQ_GAIN       0x12
#define DSP_P_EQ_Q          0x13
#define DSP_P_EQ_TYPE       0x14
#define DSP_P_EQ_COUNT      0x15   /* active PEQ band count (WebUI extension) */
#define DSP_P_HP_ON         0x20
#define DSP_P_HP_FREQ       0x21
#define DSP_P_HP_TYPE       0x22
#define DSP_P_HP_SLOPE      0x23
#define DSP_P_LP_ON         0x30
#define DSP_P_LP_FREQ       0x31
#define DSP_P_LP_TYPE       0x32
#define DSP_P_LP_SLOPE      0x33
#define DSP_P_ROUTE_ON      0x50
#define DSP_P_ROUTE_GAIN    0x51
#define DSP_P_ROUTE_SRC     0x52   /* 该交叉点的源声道（0 自动 / 1 L / 2 R / 3 单声道）*/
#define DSP_P_MASTER_VOL    0x60
#define DSP_P_PORT_MODE     0x61
#define DSP_P_PORT_GAIN     0x62
#define DSP_P_PORT_DELAY    0x63
#define DSP_P_PORT_FORMAT   0x64
#define DSP_P_PORT_ENABLE   0x65
/* 端口级 HP/LP（BLOCK_GLOBAL(4)、ch=0、idx=端口号）：i2s0 高通 + i2s1 低通的
 * 分频只能在端口做（通道滤波器被两个端口共用）。两端口共用同一组系数，
 * 每个槽（L/R）各自持有独立状态，所以不会产生左右串音。 */
#define DSP_P_PORT_HP_ON    0x66
#define DSP_P_PORT_HP_FREQ  0x67
#define DSP_P_PORT_HP_TYPE  0x68
#define DSP_P_PORT_HP_SLOPE 0x69
#define DSP_P_PORT_LP_ON    0x6A
#define DSP_P_PORT_LP_FREQ  0x6B
#define DSP_P_PORT_LP_TYPE  0x6C
#define DSP_P_PORT_LP_SLOPE 0x6D
#define DSP_P_REQ_ON         0x70
#define DSP_P_REQ_FREQ      0x71
#define DSP_P_REQ_GAIN      0x72
#define DSP_P_REQ_Q         0x73
#define DSP_P_REQ_TYPE      0x74
#define DSP_P_REQ_COUNT     0x75   /* active room-EQ band count (WebUI extension) */
#define DSP_P_LIMIT_ON      0x76   /* 削波保护（限幅器）开关 */
#define DSP_P_LIMIT_THRESH  0x77   /* float32 dBFS, -24..0 */
#define DSP_P_LIMIT_RELEASE 0x78   /* float32 ms, 10..1000 */
/* 响度补偿（等响）：主音量联动的高低架补偿，落在 blob 尾部预留区，不占双二阶预算 */
#define DSP_P_LOUD_ON       0x79   /* u8：0 关 / 1 开（默认关，保持出厂旁通） */
#define DSP_P_LOUD_AMOUNT   0x7A   /* float32 dB, 0..15：主音量到最低时的最大补偿量 */
#define DSP_P_LOUD_REF      0x7B   /* float32, 0.05..1：参考音量（该音量及以上补偿为 0） */
/* 压缩器（动态范围压缩 / 夜间模式）：端口增益之后、削波保护之前。
 * 阈值/压缩比/释放 三个参数 + 开关；补偿增益由固件按 |th|·(1-1/R) 自动推出，不占参数位。
 * 曲线（硬拐点）：x ≤ th 时 out = x + makeup；x > th 时 out = x/R（等价于 th + (x-th)/R + makeup）。
 * 因为 makeup = |th|·(1-1/R)，0 dBFS 输入恰好得到 0 dBFS 输出，**任何输入都不会削顶**。 */
#define DSP_P_COMP_ON       0x7C   /* u8：0 关 / 1 开（默认关，保持出厂旁通） */
#define DSP_P_COMP_TH       0x7D   /* int8 dBFS, -60..0：压缩阈值 */
#define DSP_P_COMP_RATIO    0x7E   /* u16(小端), 值=压缩比×10, 10..255 => 1.0..25.5。
                                    * ⚠ **v24 起由 u8 改为 u16**（参数编码 v2）：旧宽度"u8 + ×10"
                                    * 是全协议里唯一一处语义不自然的编码，且上限只有 20.0:1。
                                    * 宽度提到 u16 后**上限应声翻倍**，但这里刻意取 **255** 而不是
                                    * 65535 —— 因为 blob 里该字段仍是 **1 字节**，上限 >255 会让
                                    * "存 NVS → 重启 → 读回"变成**有损**（静默改掉用户的值）。
                                    * 255÷10 = 25.5:1 已覆盖压缩器的实际用量。 */
#define DSP_P_COMP_REL      0x7F   /* u16 ms, 10..2000：释放时间（攻击固定 5 ms） */
/* ★ v27 新增：DSP 精度模式（u8，1 字节；0=高精度默认 / 1=扩展）。
 * `0x80` 是参数区里**最低的空闲 id**（参数最高用到 `0x7F`）。
 * ⚠ 不要和**消息类型** `MSG_ACK = 0x80` 搞混 —— 两者是不同的命名空间
 *（参数 id 在参数帧里是 `p[4]`；消息号在 `p[0]`），不冲突。
 * 语义与取舍见上面 DSP 精度模式那一段；存储位置见 dsp_config.c 序列化的 limAt+30。 */
#define DSP_P_PRECISION_MODE 0x80
/* ---------------- ★ v36 新增：硬件控制层参数（规格 `规格_硬件控制层_Key与Mute.md` v9 §4.1） ----------------
 * 本轮（阶段 B）**只加参数与配置，不实现行为**；行为（mute 状态机）在后续轮次做。
 *
 * 三个参数（**u16 必须按 2 字节小端发，否则 status=4 ERR_LENGTH；越界一律 status=5 且原值不变**）：
 *     0x81  u16  延时 A：停止播放 → 静音延时 ms（0..2000，默认 30）
 *     0x82  u8   mute 有效电平反转（0 = 低有效静音 = 默认 / 1 = 高有效静音）
 *     0x84  u16  延时 B：开始播放 → 解除静音延时 ms（0..2000，默认 30）
 *
 * ⛔ `0x83` 与 `0x85~0x89` **没有 case 分支** ⇒ 写入落到 GLOBAL 的 `default:`，
 *    返回既有的 `DSP_STATUS_ERR_PARAM`（`status=0x02`，不静默成功、不改状态）；
 *    读侧也不上报。
 *
 * blob 尾部（原 limAt+31..37 那 7 个预留字节，**用满但不改总长、不改 blob ver**）：
 *     limAt+31..32  延时 A（u16 小端）
 *     limAt+33..34  延时 B（u16 小端）
 *     limAt+35      **保留字节**：写 0、读忽略
 *     limAt+36      mute 极性（只用 bit0）
 *     limAt+37      **尾部存在标记**（0 = 旧 blob ⇒ 全用默认；非 0 = 上面 6 B 有效）
 *
 * ⚠ 为什么需要一个"存在标记"（**实现决定，布局不变**）：
 *   延时的合法值**包含 0**，所以"这 6 B 全 0 ⇒ 视为旧 blob"这种隐式判据会把
 *   用户显式设的"两个延时都 0"在**重启后悄悄改成默认 30 ms**（有损，且用户看不出来）。
 *   压缩器当年能用"全 0 ⇒ 视为没有这一段"，是因为它的阈值/压缩比的合法范围**都不含 0**；
 *   延时没有这个性质。⇒ 用一个**显式标记**：旧 blob 该字节本来就是 0（预留字节写 0），
 *   所以**天然回落默认**、又不会与"合法的全 0 值"混淆。
 *   （规格 §4.2 的"旧 blob 全 0 ⇒ 天然回落默认"由本标记**更强地**满足。）
 *   ⚠ 保留字节 limAt+35 同样恒写 0：不新增 NVS 迁移/保存。
 */
#define DSP_P_HW_MUTE_DELAY_A   0x81
#define DSP_P_HW_MUTE_POLARITY  0x82
#define DSP_P_HW_MUTE_DELAY_B   0x84

#define DSP_HW_DELAY_MIN        0
#define DSP_HW_DELAY_MAX        2000
#define DSP_HW_DELAY_DEFAULT    30
#define DSP_HW_POLARITY_MAX     1

/* 尾部"硬件控制层字段"长度 = 6 B 数据 + 1 B 存在标记 = 7 B
 * （6 B = 延时 A 2 + 延时 B 2 + 保留字节 1 + 极性 1）；
 * **编译期静态断言**（dsp_config.c 的 `_Static_assert`）保证它不超过尾部预留的 7 B ——
 * 否则将来加字段会**静默溢出**（规格 §4.2 的硬要求）。 */
#define DSP_BLOB_HW_FIELDS_LEN     6u
#define DSP_BLOB_HW_MARKER_LEN     1u
#define DSP_BLOB_HW_TOTAL_LEN      (DSP_BLOB_HW_FIELDS_LEN + DSP_BLOB_HW_MARKER_LEN)
#define DSP_BLOB_TAIL_RESERVED_LEN 7u   /* limAt+31..37（精度模式占了 limAt+30 之后的全部余量） */
#define DSP_BLOB_HW_MARKER_VAL     0xA5u /* 存在标记的值（非 0 即可；取个显眼常数便于人工看 blob） */
/* 压缩器参数合法范围（固件与前端共用同一套数字；越界一律 status=5 拒绝） */
#define DSP_COMP_TH_MIN       (-60)
#define DSP_COMP_TH_MAX         0
#define DSP_COMP_RATIO_MIN      10     /* ×10 => 1.0:1 */
#define DSP_COMP_RATIO_MAX      255    /* ×10 => 25.5:1（= blob 单字节能无损存下的上限） */
#define DSP_COMP_REL_MIN        10    /* ms */
#define DSP_COMP_REL_MAX        2000  /* ms */

/* ---------------- 版本：**两层，别混用** ----------------
 *
 * | 版本 | 管什么 | 变了会怎样 |
 * |---|---|---|
 * | `DSP_CONFIG_VERSION`（blob `ver`，当前 **5**） | **BULK 序列化布局** | 不匹配 → 固件丢弃 NVS 旧 blob、用默认值。**代价大**，只在字段增删/改宽度时动 |
 * | `DSP_PROTO_VERSION`（参数编码，当前 **2**） | **单个参数的线上编码**（宽度/缩放/语义） | 不匹配 → 老客户端某个参数写入会 `status=4`（长度错误）被拒。**不动 NVS** |
 *
 * 为什么分开：0x7E 由 u8→u16 **只改了参数编码、没改 blob 布局**（序列化仍是 1 字节）。
 * 若为此把 blob ver 也升到 6，会让**所有用户已存的配置被整包丢弃** —— 代价与收益不成比例。
 * 所以：blob ver 保持 5，另立 PROTO_VERSION 并在 SYS_STATUS 里报出，让前端能识别。
 *
 * **改参数编码时的检查单**：① 本宏 +1；② 更新 §3.3 参数表；③ 前端 `pushAll` 的字节宽度；
 * ④ 对应 `proto_*_test.ps1` 的发送宽度；⑤ 在 §13 变更记录里写明"哪几个 id、从什么改成什么"。 */
#define DSP_CONFIG_VERSION      5      /* BULK blob 布局版本（与 dsp_config.c 的 wr_u16 一致） */
#define DSP_PROTO_VERSION       2      /* 参数编码版本：v2 = 0x7E 由 u8 改 u16 */

/* 注：v23 排查右对齐时用过的临时诊断参数 `DSP_P_SLOTDBG`(0x7D) 已删除（0x7D 让给压缩器阈值）。
 * 它的作用是"直接写 I2S 槽标志位"，结论已固化在 i2s_out_v2.c 的 slot_cfg_for() 注释里；
 * 若将来还要扫线格式，临时加回 `i2s_out_set_slot_raw()`（该函数仍保留在 i2s_out_v2.c）。 */

/* ---------------- Filter / crossover enums ---------------- */
typedef enum {
    DSP_FILTER_PEAKING = 0,
    DSP_FILTER_LOWSHELF = 1,
    DSP_FILTER_HIGHSHELF = 2,
    DSP_FILTER_LOWPASS = 3,
    DSP_FILTER_HIGHPASS = 4,
    DSP_FILTER_BANDPASS = 5,
    DSP_FILTER_NOTCH = 6,
    DSP_FILTER_ALLPASS = 7,
} dsp_filter_type_t;

typedef enum {
    DSP_XO_BUTTERWORTH = 0,
    DSP_XO_LINKWITZ_RILEY = 1,
} dsp_xo_type_t;

/* 路由交叉点的源声道选择 */
typedef enum {
    DSP_ROUTE_SRC_AUTO = 0,   /* 沿用原语义：CH1 <- 源 L，CH2 <- 源 R */
    DSP_ROUTE_SRC_L    = 1,
    DSP_ROUTE_SRC_R    = 2,
    DSP_ROUTE_SRC_MONO = 3,   /* (L + R) / 2 */
} dsp_route_src_t;

typedef enum {
    DSP_PORT_MODE_STEREO = 0,
    DSP_PORT_MODE_LEFT_ONLY = 1,
    DSP_PORT_MODE_RIGHT_ONLY = 2,
} dsp_port_mode_t;

typedef enum {
    DSP_PORT_FORMAT_I2S = 0,   /* Philips I2S: WS 提前 1 个 BCLK，MSB 在第二个边沿 */
    DSP_PORT_FORMAT_LJ  = 1,   /* 左对齐 (MSB-first)：MSB 与 WS 边沿同时 */
    DSP_PORT_FORMAT_RJ  = 2,   /* 右对齐：数据对齐到槽的 LSB */
} dsp_port_format_t;

/* ---------------- Parameter status codes (protocol ACK) ---------------- */
typedef enum {
    DSP_STATUS_OK = 0x00,
    DSP_STATUS_ERR_BLOCK = 0x01,
    DSP_STATUS_ERR_PARAM = 0x02,
    DSP_STATUS_ERR_INDEX = 0x03,
    DSP_STATUS_ERR_LENGTH = 0x04,
    DSP_STATUS_ERR_VALUE = 0x05,
    DSP_STATUS_ERR_UNSUPPORTED = 0x06,
    DSP_STATUS_ERR_LIMIT = 0x07,        /* 超出 DSP 滤波器预算（会被拒绝） */
    DSP_STATUS_ERR_PRESET = 0x08,       /* 预设槽无效 / 空 */
} dsp_status_t;

/* ---------------- Model ---------------- */
typedef struct {
    uint8_t on;
    uint8_t type;      /* dsp_filter_type_t */
    float   f;         /* Hz, 20..20000 */
    float   g;         /* dB, -15..+15 */
    float   q;         /* 0.4..10 */
} dsp_band_t;

typedef struct {
    float       gain_db;    /* -72..+12 */
    uint8_t     mute;
    uint8_t     phase;      /* 1 = invert */
    uint8_t     n_eq;
    uint8_t     n_req;
    dsp_band_t  eq[DSP_MAX_BANDS];
    dsp_band_t  req[DSP_MAX_BANDS];
} dsp_input_t;

typedef struct {
    uint8_t on;
    uint8_t src;       /* dsp_route_src_t：该交叉点取输入的哪个声道 */
    float   gain;      /* linear 0..1 */
} dsp_route_t;

typedef struct {
    uint8_t on;
    uint8_t type;      /* dsp_xo_type_t */
    uint8_t slope;     /* 1 / 2 / 4 -> 12 / 24 / 48 dB/oct */
    float   f;         /* Hz */
} dsp_xo_t;

typedef struct {
    uint8_t    n_eq;
    dsp_band_t eq[DSP_MAX_BANDS];
    dsp_xo_t   hp;
    dsp_xo_t   lp;
} dsp_channel_t;

typedef struct {
    uint8_t mode;      /* dsp_port_mode_t */
    uint8_t format;    /* dsp_port_format_t */
    uint8_t enabled;
    float   gain_db;   /* -24..+12 */
    float   delay_ms;  /* 0..20 */
    dsp_xo_t hp;       /* 端口高通：系数两槽共用，状态每槽独立 */
    dsp_xo_t lp;       /* 端口低通：分频请用 LR 型、两端口同频同阶 */
} dsp_port_t;

/* 输出限幅器（削波保护）：立体声联动，端口增益之后、延时之前 */
typedef struct {
    uint8_t on;
    float   thresh_db;    /* -24..0 dBFS */
    float   release_ms;   /* 10..1000 ms */
} dsp_limiter_t;

/* 响度补偿（等响）：两个隐藏架式双二阶，挂在主音量之后、四通道共用。
 * 补偿量随主音量下降而线性增大到 amount_db，参考音量及以上为 0 dB。 */
typedef struct {
    uint8_t on;
    float   amount_db;   /* 0..15：主音量到最低时的最大补偿量 */
    float   ref;         /* 0.05..1：参考音量（补偿为 0 的音量点） */
} dsp_loudness_t;

/* 压缩器（动态范围压缩 / 夜间模式）：立体声联动，端口增益之后、削波保护之前。
 * 攻击时间固定 5 ms（暂不做参数）；补偿增益自动 = |thresh|·(1-1/ratio)。 */
typedef struct {
    uint8_t on;
    int8_t  thresh_db;    /* -60..0 dBFS */
    uint16_t ratio_x10;   /* 值=压缩比×10, 10..255 => 1.0..25.5。
                           * ⚠ 内存里是 u16（参数编码 v2），但**序列化仍写 1 字节** ——
                           *   上限刻意取 255 就是为了让 blob 单字节能**无损**存下，
                           *   从而 blob 布局不变、blob ver 保持 5。
                           *   改上限时必须同时看 dsp_config.c 的 wr_u8（blob）。 */
    uint16_t release_ms;  /* 10..2000 ms */
} dsp_comp_t;

typedef struct {
    dsp_input_t   in[DSP_NUM_INPUTS];
    dsp_route_t   route[DSP_NUM_INPUTS][DSP_NUM_CHANNELS];
    dsp_channel_t ch[DSP_NUM_CHANNELS];
    float         master;   /* linear 0..1 */
    dsp_port_t    port[DSP_NUM_PORTS];
    dsp_limiter_t lim;
    dsp_loudness_t loud;
    dsp_comp_t    comp;
    /* ★ v27：DSP 精度模式（DSP_PRECISION_HIGH / DSP_PRECISION_EXTENDED）。
     * ⚠ **v30 起它不再影响双二阶上限**（两种模式预算相同；上限只取决于 `comp.on`）。
     * 参数 id / 默认值 / blob 布局 / 取值范围一律未变，仍然读写与上报。
     * 存在 blob 尾部**原本就预留的空闲字节** limAt+30 ⇒ blob ver 仍是 5。 */
    uint8_t       precision_mode;
    /* ★ v36：硬件控制层（**阶段 B 只存配置**；mute 状态机在阶段 C1 实现）。
     * 三个字段都进 blob 尾部（见上面参数 id 那一段的偏移账）。
     * ⚠ limAt+35 是**保留字节：写 0、读忽略** —— 长度与其它字段偏移一个字节都没动，
     *   所以历史配置不会错位、也不需要 NVS 迁移。 */
    uint16_t      hw_mute_delay_a_ms;   /* 0..2000，默认 30：停止播放 → 静音 */
    uint16_t      hw_mute_delay_b_ms;   /* 0..2000，默认 30：开始播放 → 解除静音 */
    uint8_t       hw_mute_polarity;     /* 0 = 低有效静音（默认）/ 1 = 高有效静音 */
} dsp_config_t;

/* 当前精度模式（协议层读用；**与预算无关**，见上面的注释）。 */
uint8_t dsp_config_precision_mode(void);

/* ★ v36：硬件控制层的只读访问（阶段 C1 的 mute 状态机用；只暴露存取器，不含行为）。 */
uint16_t dsp_config_hw_mute_delay_a_ms(void);
uint16_t dsp_config_hw_mute_delay_b_ms(void);
uint8_t  dsp_config_hw_mute_polarity(void);

/* 按**指定的压缩器开关**算双二阶上限（不读活动配置）。
 * ★ v30：参数语义从"精度模式"改成**"压缩器开着吗"**（0/1）。
 *   序列化/反序列化路径**必须**用这个版本、并传**候选配置自己的 `comp.on`** ——
 *   那时候选还没提交进 `s_cfg`，用 `dsp_config_biquad_limit()` 会拿当前状态去判（v27 踩过）。 */
uint32_t dsp_config_biquad_limit_for(uint8_t comp_on);

/* ---------------- API ---------------- */
esp_err_t dsp_config_init(void);                 /* defaults + NVS load + apply */
void      dsp_config_reset_defaults(void);
esp_err_t dsp_config_load(void);                 /* NVS -> RAM (keeps defaults on miss) */
esp_err_t dsp_config_save(void);                 /* RAM -> NVS */
void      dsp_config_copy(dsp_config_t *dst);    /* consistent snapshot */

/* Apply every parameter the audio engine understands today. */
void dsp_config_apply_all(void);

/* Single-parameter write from the host protocol.
 * value_len selects the encoding: 4 = float32 LE, 1 = uint8. */
dsp_status_t dsp_config_set_param(uint8_t block, uint8_t ch, uint8_t param, uint8_t idx,
                                  const uint8_t *value, size_t value_len);

/* Serialize the whole model (protocol bulk + NVS image).
 * Returns the number of bytes written, or 0 if cap is too small. */
size_t dsp_config_serialize(uint8_t *buf, size_t cap, uint32_t fs);

/* Parse a blob produced by dsp_config_serialize(). Values are clamped. */
esp_err_t dsp_config_deserialize(const uint8_t *buf, size_t len);

/* Master volume (0..1 linear) converted to a dB value clamped to +-12 dB. */
float dsp_config_master_gain_db(void);

/* Number of parameters currently wired into the audio path (diagnostics). */
uint32_t dsp_config_live_param_count(void);

/* ---- DSP 负载预算（滤波器上限保护）----
 * 双二阶总数 = 输入 PEQ + 房间 EQ + 通道 PEQ + 通道 HP/LP + 端口 HP/LP（按"最坏情况"统计
 * 两个输入源）。上限随采样率变化（块周期越长余量越多），超限的写入会被拒绝。 */
uint32_t dsp_config_count_biquads(void);
uint32_t dsp_config_biquad_limit(void);

/* ---- 设备端预设槽（NVS，namespace dspcfg，key "p0".."p3"）---- */
esp_err_t dsp_config_preset_save(uint8_t slot);    /* 当前配置 -> 槽 */
esp_err_t dsp_config_preset_load(uint8_t slot);    /* 槽 -> 当前配置（并重建引擎） */
esp_err_t dsp_config_preset_clear(uint8_t slot);
uint8_t   dsp_config_preset_mask(void);            /* bit0..bit3 = 槽是否已用 */

#ifdef __cplusplus
}
#endif
