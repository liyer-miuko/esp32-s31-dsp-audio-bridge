/*
 * 固件版本号 —— ESP32-S31 audio bridge。
 *
 * ★ v33（固件 0.32）新建本文件，目的只有一个：**把版本号从 dsp_config.h 里搬出来**。
 *
 * 为什么（实测，不是洁癖）：
 *   `dsp_engine.h` 会 include `dsp_config.h`，而 `dsp_engine.c` 是 **core1 的 DSP 热点**
 *   （`dsp_engine_process()` 在 IRAM）。版本宏只要留在 `dsp_config.h` 里，
 *   **每次升版本号都会重新编译 core1 的编译单元**，于是 core1 的每块耗时基线整体漂移
 *   —— 实测：0.31 → 0.32 期间核心热点从 2001 µs 漂到 2008~2009 µs（旁通 18 → 25 µs），
 *   之后再也无法与上一版逐点对照，而"逐点对照"是本项目唯一能证明"没把东西改坏"的手段。
 *
 * ⇒ 版本号只被**报告端**需要（`serial_proto.c` 的 SYS_STATUS、`dsp_config.c` 的启动日志），
 *   与 DSP 引擎无关。搬到独立头文件后：
 *     **升版本号 ⇒ `dsp_engine.c.obj` 的哈希不变**（这正是本轮第 1 步的验收判据）。
 *
 * 规则：固件号与工程轮次标签 vN 是**两套编号**，没有关系；
 *       同一个号不能同时指两个不同的 bin（改一次固件就必须递增一次）。
 */
#pragma once

/* 固件版本（上报给主机：SYS_STATUS 尾部 + 启动日志）。规则：工程版本 vN -> "0.N"
 * 0.30 = 段数上限拆两档（压缩器关 117 / 开 104）；
 * 0.31 = 修 P1-1 原子回滚漏洞（段数 auto-grow 没进回滚点，被拒的写会留下超限配置）。
 * 0.32 = v33 UAC 通路诊断（**只加计数器：不改音频/DSP 算法、不改任何 limit、不改 blob**）。
 *   diag 新增 `lost`（真损失帧数）/ `dsil`（非放音态静音帧，**不计损失**）/
 *   `slow`（P 超块周期次数）/ `pmax`（**窗口内**最坏 P）/ `pn` / `wait1`（等 core1）/
 *   `busy`（core0 干剩下的），后四者在放音态分窗累加。
 *   动机：`i2s_ovf`（IDF `on_send_q_ovf`）只是**生产者迟到的证据**，不是丢帧的证据。
 * 0.33 = 修 `lost` 的语义缺陷（0.32 把"每个 tick 未产出满 240 帧"都算损失 ⇒ 45 s 报出
 *   452 万帧、是名义总量的 2.6 倍，物理上不可能）+ `pmax` 改**窗口内** +
 *   **core1 结构解耦**（dsp_engine.h 不再 include dsp_audio.h、版本号搬进 dsp_version.h
 *   ⇒ 升版本不再触发 core1 重编译、不再让基线漂移）。
 *
 * ⚠ 0.34 / 0.35 = 本轮排查 `lost` 计数器异常时的中间版本（连烧了几版，**未归档**）；
 *   二者只差临时诊断代码。设备最终停在 **0.35**，交付 bin = `uac_bridge_v33_uac_diag.bin`。
 * ⚠ **`lost` 计数器尚未收口，禁止用于判据**（原因已写在 uac2_audio.c 的对应注释里）。
 *
 * 0.36 = v33 收口版（core1 结构解耦 + 基线冻结；设备交付停在 0.36）。
 * 0.37 = **v35/v36 轮：硬件控制层「阶段 B 接口层」** —— 只加**参数与配置**，不含任何行为：
 *        ① 新参数 `0x81` u16 延时 A / `0x82` u8 极性 / `0x83` u8 编码器启用 / `0x84` u16 延时 B
 *           （`0x85~0x89` u8×5 键位码**只预留 ID**）；
 *        ② blob 尾部原预留的 7 B 启用为"硬件控制层 6 B + 存在标记 1 B" ⇒ **总长 1578 B 不变、
 *           blob ver 仍为 5**，并加了**编译期静态断言**防静默溢出；
 *        ③ **不初始化任何 GPIO、不创建任何任务**（mute 状态机与编码器在阶段 C1/C2）。
 *        ⚠ 本版**不改变音频通路、不改任何 limit**（"不动 core1"这句**只对源码成立**：
 *           实测 `dsp_engine.c.obj` 因 `dsp_config.h` 的 `dsp_config_t` 加字段而**传递式重编**，
 *           obj 哈希 `ADADC1CA…` → `F3B0C0A3…`、core1 基线整体下移 **−0.4~−4.7 µs**。
 *           六个冻结源文件与 v33 归档**逐字节相同**（6/6）⇒ 偏移归因于重编，已按冻结协议
 *           重测并重新冻结，见 `工程版本\v33_uac_perf\build_artifacts\core1_baseline.json` 的 `_v36_refreeze`。）
 *
 * 0.39 = **v39 轮：修 RESET_CONFIG 的「配置传播缺口」**（结构问题，不是漏一行）：
 *        hw_mute_apply_config() 原先**只在参数写路径**里调，而 RESET / 预设载入 / NVS 载入走的是
 *        dsp_config_apply_all()（只推 DSP 引擎 + I2S 线格式）⇒ **复位后模块仍留着旧极性**
 *        （实测：RESET 之后 0xAF 仍报 pol=1、引脚停在旧极性的静音电平）。
 *        现收口成**两个面**：引擎面 dsp_engine_sync() / 控制面 dsp_config_apply_control()
 *        （由 apply_all 调用）；参数写对纯控制项**只走控制面**，保住
 *        「0x81~0x84 不重建 DSP 引擎」这条优化。dsp_config.h 里登记了「谁属于哪一面」。 * 0.38 = **v37 轮：硬件控制层「阶段 C1 —— Mute 输出」**（规格_硬件控制层_Key与Mute.md §1）：
 *        ① **GPIO8 在 `app_main()` 最早期就被驱动成静音电平**（早于 I2S 通道/DSP 任务；
 *           那时还不知道用户极性 ⇒ 先用默认低有效，配置载入后再由 `hw_mute_apply_config()` 接管）；
 *        ② `hw_mute.c` 的低优先级任务（core0，10 ms）用**无钩子**方式判定"正在播放"：
 *           `playing := pump_runs 在推进 && silence_runs 没推进`（读 0xA3/0xA4 已有的计数器，
 *           音频通路里**不加任何钩子**、不碰 core1）；
 *        ③ 两个延时（`0x81` A = 停止→静音、`0x84` B = 开始→解除静音）+ 极性 `0x82` **在此版真正生效**；
 *           **一次只有一个延时在跑**，新事件取消旧的、以最新状态为准；
 *        ④ `0x81~0x84`（以及预留的 `0x85~0x89`）**不再触发 DSP 引擎重建** —— 它们是纯控制项，
 *           重建既白花 core1 时间，又会让拖动滑块时正在播放的声音断一下；
 *        ⑤ 新增诊断帧 **`0xAF` HW_STATUS**（20 B）：静音状态/引脚实测电平/playing/极性/两个延时/
 *           跳变次数/距上次跳变的 µs ⇒ 验收不必只依赖 LA 接线（LA 仍是权威尺子）。
 *        ⚠ `P_MASTER_VOL` **不参与静音**（规格 §0.0 硬约束：静音走 GPIO，不是把 master 归零）。
 *
 * 0.40 = **v41 轮（C2 前置）：`0x80` 补 `pure_control = true;`**（登记册 **L19**）：
 *        精度模式与 DSP 引擎无关（`dsp_config_precision_mode()` **无调用者**），
 *        而 v38 的 pure_control 清扫漏了它 ⇒ 每次写 `0x80` 都白重建一次引擎
 *        （v40 隔离探针：换值、换回、以及 **同值重写** 三种写 Δ 均 +1；而 `0x81~0x84` 全 0）。
 *        ⚠ 触发路径真实：前端 `pushAll()` 每次整包同步都会按**同值**写一次 `0x80`。
 *        ⚠ 本版**只动这一个分支**：参数 id / 默认值 / blob 布局 / 音频通路**一字未改**；
 *          `dsp_engine.c` 未被触碰（obj 仍应为冻结值 —— 按冻结协议复验）。
 *        配套：`html前端\dsp webui.html:1789-1799` 那段 v27/v28 的**过期注释**同步改掉。
 *
 * 0.42 = **v44 轮（BUG-2 根因修复）：BT 闸门加回差**（`uac2_audio.c` 源策略一处）：
 *        起播门槛（ring ≥ BT_PREBUFFER_BYTES）不再兼作维持门槛 —— 旧逻辑水位随机游走
 *        瞬时跌破 5760 B 就关闸 ⇒ 泵写整块静音（`nodata`）⇒ 可听杂音/断续。
 *        实测判据：25/25 静音块台阶全部落在"当秒水位最低 < 5760 B"窗口（v44 报告）。
 *        开闸条件与真欠载记账（`bt_silence_runs`）不变；不动任何 limit/缓冲。
 *
 * 0.43 = **v44 轮第二格：BT ring 水位双向调节**（仍属 BUG-2 修复链，一次一格）：
 *        ① 起播水位 3 块(5760B) → **8 块(15360B)** —— 实测源抖动 p99 欠载游程 ≈7KB，
 *           回差修复后 ring 不再被拍合钳位，3 块起点等于零裕度；
 *        ② 新增上限 12 块(23040B) 弃旧 —— 源时钟实测 +12~34ppm（快），
 *           0.42 只有下限修复时长会话 21~70 分钟会溢出丢新，上限弃旧把延迟与溢出一起封住。
 *        仍未动任何 limit/环容量；codec 闪断回差是下一格（单独验证）。
 *
 * 0.44 = **v44 轮第三格（L38 裕度维度）：BT ring 扩容 + 水位时间基准化**：
 *        ① CONFIG_BT_PCM_RING_FRAMES 4096 → 8192 帧（+32KB 静态内存）—— P9 前提已满足：
 *           欠载实测 94 次/210s@96k、ring min=768B、ovf 账目在案（v44 报告）；
 *        ② 水位从「块/字节」改为**时间基准**（骑高 40ms / 上限 60ms 弃旧）—— 块基准裕度
 *           随采样率反比缩水（8 块 = 40ms@48k 但 20ms@96k），LDAC 源抖动毫秒级不随率变
 *           ⇒ 三档欠载梯度 96k:94 / 88.2k:81 / 48k:1 即由此来。
 *        判据：96k 欠载率降两个数量级（94 → 目标 <5/210s）；48k 不回归。
 *
 * 0.45 = **v45 轮：修 L40「BT 连接瞬间呲呲声」** —— TinyUSB audio RX 的 FIFO 写入改**原子**
 *        （espressif__tinyusb audio_device.c：整包写不进就整包丢弃，不再部分写入）。
 *        根因：BT 连接秒 core0 被挤占 → 泵延迟 → RX FIFO 打满 → 部分写入把 24 位打包流
 *        推移 n%6 字节 → 每样本字节轮转 → 线上满幅乱码噪声。改后溢出只丢整包（零可听
 *        损失、流对齐不破坏）。不改任何 limit/缓冲/优先级（P9），不动 core1 冻结件。
 *
 * 0.45 = v45 初版修复：TinyUSB audio 类 **!HWFIFO 分支**的原子写 —— 实测**无效**
 *        （S31 = DWC2 slave 模式：CFG_TUD_DWC2_DMA_ENABLE=0 ⇒ HWFIFO=1，该分支是死代码）。
 *        bin = uac_bridge_v45_l40_rx_atomic.bin（287023FB…），已烧录验证过 VER=0.45 后被 0.46 取代。
 * 0.46 = **v45 真修复**：DWC2 slave RX 真路径（dcd_dwc2.c handle_rxflvl_irq GRXSTS_RX_DATA）
 *        整包原子入环 —— tu_hwfifo_read_to_fifo 在环满时部分写入/滞留 RxFIFO ⇒ 打包字节流
 *        失步 ⇒ 每样本字节轮转 ⇒ 满幅乱码（L40 呲呲）。改后满环整包丢弃（零可听损失）。
 *
 * 0.47 = **v46 轮：L43-A「BT 连接窗预静音」** —— ACL 连接/codec 协商起 3s 强制 MUTED 窗（GPIO8），
 *        窗内跳过常规仲裁、出窗按当前 playing 重新仲裁（A/B/极性语义与 pin 真值不变式不变）。
 *        消 L43 三层瞬态听感（丢包段/边界咔哒/泵挤占簇）；触发点 a2dp_audio_int.c 三处事件挂钩。
 *
 * 0.48 = L43-A 窗长按用户拍板 3000 ms → **1500 ms**（"连接后 1.5s 再放音"，3s 太久）。
 *
 * 0.49 = L43-A 窗长按用户拍板 1500 ms → **2000 ms**（用户 2026-09-26 复拍："连上后静音窗加到 2 s"）。
 *        只改 `HW_MUTE_BT_CONNECT_QUIET_MS` 一个常数（hw_mute.h）；触发点、仲裁语义、
 *        pin 真值不变式、A/B/极性语义**一字未改**；不动 limit/缓冲/优先级（P9），不碰 core1 冻结件。
 *
 * 0.50 = **v47 轮 L44：蓝牙接入的两个现场故障**（需求：蓝牙优先 + 连接时无异常音 + 不干扰
 *        正在播放的 UAC）。**结论：1.5 s 窗无效不是窗太短，而是窗挂错了位置。**
 *        H1（异常音）——0.47/0.48 把预静音窗挂在 **ACL 连接 / codec 协商**上；而杂音出现在
 *          **输出真正交给 BT 的那一刻**（`uac2_audio.c` 的切换点）：i2s_out_reconfig() 停/起
 *          I2S 时钟（APLL 换频、DAC 掉锁）+ fork 解码器预热（L38③b 坏帧乱码直通）+ ring
 *          前几块。那一刻通常**晚于** ACL 连接（手机开流/按播放才发生）⇒ 旧窗早已过期，
 *          所以 1.5 s → 2 s 当然无效。修法：把窗挪到**真的切换**这一刻，且**只在本次连接的
 *          第一次接管**上盖（否则每次起播都吃掉窗长那么久的音乐）；断开即复位门闩。
 *          ⚠ 旧挂钩（a2dp_audio_int.c 的 4 处）**保留不动** —— 它们只可能让窗更长，不会更短。
 *        H2（长时间静音）——退回 UAC 的逻辑只写在 `!(bt_connected && bt_streaming)` 那一支里；
 *          而"A2DP 流已打开（STARTED）却一个 PCM 块都没送到"（手机开了流不推音频 / 解码停滞）
 *          **永远等不到超时** ⇒ active_source 卡在 BT、泵永久写整块数字零、UAC 正在播的内容
 *          被整段吃掉。修法：把"退回 UAC"收成**唯一出口**，判据用"BT 还活着吗"
 *          （心跳 = 泵交付过整块 PCM 或 BT 环里还有数据），超时复用既有的
 *          `CONFIG_BT_A2DP_SWITCH_BACK_MS`（**不新增任何 limit 常数**）。
 *        ⚠ 本版**不动任何缓冲/优先级/limit**（P9）、不碰 core1 冻结件、不改 `playing` 判据
 *          （保 `rig_mute_verify` 的 0xAF 契约）；只加两处状态 + 一个 diag 字段
 *          （`bt:` 行尾部 `guard=` / `idle=`，让复测可机读）。
 *        ⚠ **0.50 = 中间版（H1+H2，未烧录）** —— 随后现场证据把第三块逼了出来，见 0.51。
 *
 * 0.51 = **v47 轮 L44 收口：H3「功放跟着真实音频走」**（+ 0.50 的 H1/H2）。
 *        现场实测（`测试与抓包\串口日志\l40_v45_retest_serial.txt`，一次完整的 BT 接管）：
 *          675899 `output source switched to BT @ 96000 Hz`
 *          675954 `hw_mute: unmuted (pin GPIO8 -> 1)`      ← 功放**整段 BT 会话**都开着
 *          677788 `bt: gate=1 ring=32128 pkt=714 drop=0`    ← BT 在健康推流
 *          677767 `peaks: i2s L/R -999.00/-999.00 dBFS`     ← 但输出是**数字静音**
 *          679585 `output source switched to UAC`           ← 只有"流挂起"才回得来
 *        根因：`playing := pump 推进 && silence_runs 没推进`，而 **`silence_runs` 只在
 *          UAC 分支自增** ⇒ BT 模式下 playing **恒真** ⇒ 功放在 BT 会话里一直解除静音。
 *          ⇒ 这就是"连接时那段异常音"能听见、而 1.5 s / 2.0 s 预静音窗**治不了**的原因：
 *            窗只盖窗长那一段，功放在 BT 会话里**一直**开着。
 *        修法：新增计数器 `nonaudio_runs`（`has_data == false` 的每个 tick 都计一次，
 *          一次覆盖 UAC FIFO 空 / BT 门未开 / BT 欠载 / 源切换 / 静音窗 / codec 静音），
 *          `hw_mute` 改用它判 playing。**`silence_runs` 语义不动**（BUG-1 分流计数器，
 *          仍被 diag 与量具解析）。对 UAC 独占场景（`rig_mute_verify` 全部用例）两者
 *          **逐拍一致**：有内容 ⇒ 都不前进 ⇒ playing=1；无内容 ⇒ 都前进 ⇒ playing=0。
 *        另加 `diag-ext: nonaudio=` 一行（追加在最末，不改既有 diag 字段口径）。
 *
 * 0.52 = **v47 轮 L44 真根因：BT 闸门自锁**（本机 BT 源真机复测抓到的，fault B 的主因）。
 *        0.51 装上后做**完整实机复测**（本机 Windows 作 A2DP 源，aptX 44.1k），判据工具
 *        `工具\scripts\l44_judge.py` 在 `bt:` 行上抓到**连续 4 个采样**：
 *          771073 streaming=1 ready=1 dur=687ms  ring=3200  pkt=2645 gate=0
 *          772073 streaming=1 ready=1 dur=1687ms ring=8480  pkt=2678 gate=0
 *          773074 streaming=1 ready=1 dur=2688ms ring=13760 pkt=2711 gate=0
 *          774074 streaming=1 ready=1 dur=3688ms ring=13120 pkt=2743 gate=0
 *        数据一直在进、dur 远超 80 ms、ready=1，泵却整秒整秒写数字零 = **4 秒静音**。
 *        根因：门关着时泵的 hold 分支把环排到 **BT_PREBUFFER_MS（40 ms）**，而**开门条件**
 *          又要求环 ≥ 同一个 40 ms 目标 ⇒ 两条件互相抵消，**能否重开纯属竞态**
 *          （真机 ring 一直卡在 12288~14112 之间）。这就是"概率性静音很长时间"。
 *          且 hw_mute 的活性看门狗抓不到（环里有数据 ⇒ 判"活着"）。
 *        修法：闭门期改排到 **BT_LEVEL_HIGH_MS（60 ms）** ⇒ 水位有 40/60 ms 回差带，
 *          环必然爬过 40 ms 门槛 ⇒ **开门确定**。复用既有常数，不新增 limit（P9）。
 *        ⚠ 只改 `uac2_audio.c` 一处 drain 门槛；不动缓冲/优先级/冻结件。
 *
 * 0.53 = **v48 轮 L45：UAC 流内数据断口的"咔咔声"**（用户 2026-09-26 实听点单：
 *        "选择音乐进度后，从 1 分钟跳到 1 分 40 秒，起播的声音有咔咔声"）。
 *        现场：拖进度条 = 播放器停一下再从新位置起。shared 模式下**端点流一直开着**
 *          （`streaming` 恒 1、`have_audio` 恒 1），只是主机短暂不送数据 ⇒
 *          旧代码的淡入淡出判据（`starting`/`stopping` 都看这两个量）**两个沿都不触发**：
 *            落沿：最后一个样本**硬切 0**  ← LA 实测 Δ=−0.209 / −0.245（满幅 0.25），
 *                  而功放延时 A 要 **22 ms** 后才静音 ⇒ 台阶已经放出去了 = "咔"
 *            起沿：0 **硬切**到恢复后的第一个样本（台阶大小看相位，随机）
 *        修法（只动 `uac2_audio.c` 输入端，不碰 core1/缓冲/优先级/limit）：
 *          ① 落沿：不再写整块硬零，改为从**最后一个输出样本**（音量+DSP 之后记下的
 *             `uac_last_l/r`）线性衰减到 0，铺满本 tick 240 帧；尾巴只放一次。
 *          ② 起沿：`gap_rising`（上一 tick 无音频、这一 tick 有，且流一直开着）
 *             触发既有的 `uac_fade_start(false, …)` 淡入。
 *        ⚠ 只在**流内断口**（streaming 且不在切换中）生效；流真正的开/关仍走原
 *          starting/stopping 淡入淡出，语义不变。
 *
 * 0.54 = **v48 轮 L46：数字静音边沿去咔**（用户耳测反馈 0.53 未消除 + 补了一条关键线索：
 *        "音乐播放器的音频并不是实时都在本地，可能要等网络和本地加载完毕"）。
 *        **改判**：0.53 修的是**设备侧断口**（主机不送数据 ⇒ FIFO 干涸、`silence_runs` 前进）；
 *        而用户听到的是**另一条路径** —— 播放器等加载时**一直在送**，只是送**精确的 0**
 *        ⇒ `has_data` 恒真、`silence_runs` **不动**、功放**一次都不翻转**，
 *        设备忠实放出一个「音 → 0 → 音」的**满幅台阶**。
 *        现场实测（`captures\v54_peakcut_before.bin`，素材切点卡在波峰、已自证）：
 *          跳变 @4.034469s Δ=−0.250（落沿）、@4.634506s Δ=+0.250（起沿）、
 *          **MUTE(D3) 上升 0 次下降 0 次** ⇒ 与功放/静音窗无关，纯内容台阶。
 *        修法（只动 `uac2_audio.c` 输入端）：整块 PCM 判"是否**精确 0**" ——
 *          落沿：用上一块的**真实末样本**衰减到 0（`uac_last_l/r` 只从非零块更新）；
 *          起沿：对首部做短淡入。两者时长都用既有 `UAC_FADE_MS`，**不动功放**。
 *        ⇒ 与 L34「数字静音不静音」的已定口径**不冲突**（那说的是**功放**，本版不碰功放）；
 *          也不影响 `ctx_hygiene` 的 `playing=1` 预期语义。
 *        ⚠ 只在**精确 0** 的边沿上动作：真实音乐的安静乐段几乎不会是整块精确 0 ⇒ 分得开。
 *        ⚠ 第一版（0.54）只判"**整块**是不是全零" ⇒ **落沿漏掉**：USB 块会**跨界**，
 *          静音起点通常落在某一块**中间**（前段音频、后段 0）⇒ 那一块 `all_zero==false`，
 *          等下一块真全零时台阶**早就发出去了**（实测：起沿消掉、落沿 Δ=−0.250 仍在）。
 *        ⇒ 0.55 改为**在块内**扫"连续 ≥24 帧（0.5 ms）精确 0"的游程：
 *          落沿回头把本块内的**真实音频尾巴**淡掉（本块还没转 slot，改得动）；
 *          静音正好从块首开始时，才退回"用上一块末样本向前衰减"。
 *
 * 0.56 = **v48 轮 L46 回退**：0.54/0.55 的"数字静音边沿去咔"**未通过判据，撤掉**。
 *        判据素材 `测试与抓包\音频\l45_peakcut_gap.wav`（1 kHz → **600 ms 精确 0** → 1.5 kHz，
 *        断口切点**卡在波峰**、用 `工具\scripts\l45_verify_wav_cut.py` 自证 ±0.250 FS）；
 *        判据尺子 = LA `D0=DOUT/D1=BCLK/D2=WS/D3=MUTE` 12 s @12 MHz + `l40_la_glitch_probe.py`：
 *          · **0.53（本版行为）**：跳变 4 处，其中数字静音两沿 **Δ=−0.250 / +0.250**（满幅）；
 *            `MUTE(D3) 上升 0 次下降 0 次` ⇒ 功放全程没动，**纯内容台阶**（用户听到的"咔咔"）；
 *          · **0.54**（判"整块是否全零"）：起沿消掉 ✓、**落沿 Δ=−0.250 仍在** ✗
 *            —— USB 块会**跨界**，静音起点落在块中间 ⇒ `all_zero==false`；
 *          · **0.55**（块内扫 ≥24 帧零游程）：**反而更糟，跳变 2 → 4**，零段 600 → 594.6 ms；
 *            逐样本 dump（`工具\scripts\l45_dump_samples.py`）显示它在**正确位置之前**插进
 *            19 帧硬零 + 一段错位的衰减斜坡 ⇒ **把音频改坏了**。
 *        ⇒ **不发布未通过判据的固件**（P5/P9）。本版 = **0.53 行为**
 *          （设备侧断口的淡入/衰减尾巴保留），数字静音去咔留待独立一轮重新设计。
 *        ⚠ **必须写清楚**：L46（用户耳测的那条"选进度后咔咔"）**尚未修复**。
 *
 * 0.57 = **v48 轮 L46 重做**（用户 2026-09-26 拍板口径）：
 *        「位精确旁通（corr=1.000000）—— **唯一例外**：数字静音游程（连续精确 0 ≥ 50 ms）
 *          的起/落沿各加 ≤5 ms 去台阶斜坡，**仅改边沿邻域样本**……非边沿区域逐样本不变
 *          （`rig_bitexact_ab` 复证）」
 *        ★ 与 0.54/0.55 的根本差别 —— **落沿不回头改音乐样本**：
 *          0.55 想把"静音之前的音频尾巴"淡掉，那要么需要预知未来（≥50 ms 确认）、
 *          要么改已经发出去的样本；它实际把波形改坏了（跳变 2→4，逐样本 dump 可见
 *          正确位置之前被插进 19 帧硬零 + 错位斜坡）。
 *          本版改为**用衰减斜坡顶替游程开头那些本来就是 0 的样本**：
 *          斜坡起点 = 最后一个真实样本值 ⇒ 序列 …x, x·(n−1)/n, …, 0, 0… **天然连续**，
 *          且**一个音乐样本都没动**。所以落沿可以零延迟、不需要预见。
 *          起沿要动"回来的音乐"的头几毫秒 ⇒ **这里才严格用 50 ms 门槛**。
 *        判据：素材 `测试与抓包\音频\l45_peakcut_gap.wav`（切点卡波峰、±0.250 FS 已自证）
 *          + LA `l40_la_glitch_probe.py`；位精确用 `rig_bitexact_ab.py`（改前存参考、改后 cmp）。
 *
 * 0.58 = **v48 轮 L46 修 b/c：切进度仍咔 + 切下一曲新增咔**（用户 2026-09-26 耳测反馈）。
 *        ⚠ 用户同时给了一条硬约束：**"功放 mute 引脚没接、由供电长期高电平，
 *          改动不能动 mute 功能"** ⇒ 咔咔**只可能来自音频样本本身**，与功放无关
 *          （本轮 L46 全部改动都在 PCM 域，一行 `hw_mute` 都没碰）。
 *        两个成因，都由已有实测定位：
 *          **b) 陈旧末样本**：下一曲重新开流时，流开头有**十几 ms 精确 0**（宿主起播静音）。
 *             若留着**上一曲**的末样本，落沿斜坡会拿它去接 ⇒ 在静音里凭空插一个旧幅度台阶。
 *             修法：`l46_last_valid` 在 **`!streaming`（关流）或换源**时一并作废。
 *             ⚠ 0.53 的 `uac_last_valid` **故意不清**（它要靠关流末样本接衰减尾巴）—— 两件事。
 *          **c) 起沿门槛够不着**：播放器起播只送 **14.48 ms** 精确 0（LA 实测），
 *             50 ms 门槛根本触发不了；而且既有 `starting` 淡入会被那块**零数据块**吃掉，
 *             等真音频到达时早已用完 ⇒ 两头都不管 = 0 硬切到音频 = 满幅台阶。
 *             修法：**起沿门槛降到与落沿同一判据**（连续 8 帧精确 0）。
 *             安全性不变：判据仍是"精确 0"，音乐样本一个都不动（`rig_bitexact_ab` 复证）。
 *
 * 0.59 = **v48 轮 L46 量具版（不发布）**：只在恢复沿挂临时探针（PROBE resume），
 *        用于量清"停流→恢复"那一沿的全部状态判据（任务书 40 §11 清单）。用完即删。
 *
 * 0.69 = **v49 轮 BLE 调音（任务书 42）**：BLE GATT 透传 AA55（ble_tuning.c +
 *        serial_proto.c 双解析器/应答路由）。控制器 BR_EDR_ONLY → BTDM 双模，
 *        删除 BLE mem_release。⚠ 开 BLE 不许弄坏 A2DP = 铁律（41 §6.5），
 *        P0 A/B 未过不许接界面。
 *
 * 0.70 = **v49 轮 BLE 发现修复（未成功，仅烧录留档）**：广播携带服务 UUID 0xFF00。
 *        结构化 esp_ble_gap_config_adv_data 被 fork 的 `service_uuid_len & 0xf`
 *        校验拒绝（只收 16B 倍数）⇒ 广播整条消失，实测扫不到。已由 0.71 取代。
 *
 * 0.71 = **v49 轮 BLE 发现修复（未完成，烧录留档）**：广播改 config_adv_data_raw
 *        手拼 AD；补 30 s 周期 status 自报。status 实测 `adv=0 conn=0` ⇒
 *        raw 配置完成走 ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT，回调没接它
 *        ⇒ start_advertising 从未调用。已由 0.72 取代。
 *
 * 0.72 = **v49 轮 BLE 发现定版**：结构化/raw 两个"配置完成"事件接到同一出口。
 *        判据：l42_ble_host_probe --mode scan 可见 target 且 service_uuids 含
 *        0000ff00-…；状态自报 adv=1。
 *
 * 0.81 = L46 定位版（非修复）：core0 只读统计 USB 归一化输入与 I2S 写入口的
 *        连续零样本段，以及放音态泵唤醒间隔；不改 PCM、仲裁、静音或缓冲。
 *        判据：LA 已复现连接时的 54/132/210 帧数字零，与 observe 日志分层对照。
 *        0.73–0.80 已用于历史尝试并回滚，版本号不复用。冻结源与 obj 必须不变。
 * 0.82 = L46 单点对照候选：同引脚/时钟/DMA 参数，将输出 I2S0 改为 I2S1。
 *        理由：0.81 的 LA 零段在 USB 输入与 I2S 写入口均不存在；S31 的 BT 内部
 *        destination 仅支持 I2S0。端口冲突仍是待证假设，以重复连接 LA 为判据。
 *        保留 0.81 只读探针用于 A/B；未改蓝牙仲裁，L47 尚待手机切编码证据。
 * 0.83 = L46 通知诊断：0.82 I2S1 仍复现插零，回到 I2S0；只计数 on_sent
 *        向二值信号量投递失败的次数，比较 sent/pump 与 LA。音频写入策略不变。
 *        待验证：连接调度抖动合并通知→驱动队列积压→未填满 DMA 尾部被放弃。
 * 0.84 = L46 完成通知候选修复：二值信号量改为计数信号量，上限复用既有 6 个
 *        DMA 描述符；重配后清空旧代通知。0.83 实测连接前 notify_full=0、后=1，
 *        sent-pump 从 0 变为 1，同轮 LA 波形异常。PCM/缓冲/仲裁/DSP不改。
 *        判据：UAC 500 Hz持续、5秒后BT仅接入，重复LA无插零/跳变；并验证回退。
 * 0.85 = L46 DMA 尾部只读诊断：0.84 无通知合并仍见 132 帧插零，记录驱动
 *        写入口 rw_pos 与队列余量，核对未填满尾部是否恰为 LA 零段字节数。
 *        临时私有头只用于读固定 fork 的状态；不改 IDF、不覆盖 components 树。
 * 0.86 = COM5/LA 状态时序定位：core0 在 RAM 记录路由/门闩/切换阶段的变化，
 *        诊断任务延后输出；CFG 起止追加 esp_timer 时间戳。音频策略不变。
 *        0.85 第二次连接实测：DMA 跳尾 624 B，LA 同轮恰有 78 帧=624 B 插零。
 *        用 GPIO8 的设备时间戳与 LA 边沿对齐；有 trace 丢失则不下门闩结论。
 * 0.87 = L46 DMA 完整块写入候选：DSP/槽转换之后增加一个描述符大小的暂存区，
 *        只把填满的块交给 IDF，避免调度抖动时跳过未填尾部；不补零、不改样本。
 *        0.85/0.86 实测 624/816 B 跳尾分别对应 LA 78/102 帧插零，门闩未开。
 *        原 FIFO、BT 环、DMA 描述符/帧数和 DSP 不变；新增 1920 B，公共输出延迟
 *        最多一块（48k 为 5ms）。重配清旧暂存；超时保留已接收数据并返回接受前缀。
 *        判据：分片/阻塞实际 C 代码保序测试 + 多次连接 LA + 正常 BT/UAC 回退。
 * 0.88 = L46 原始 USB 序列定位：0.87 完整 DMA 块未再跳尾/插零，但非周期素材三次
 *        连接为通过/失败/通过，失败轮仍丢 48+6 帧。USB 读后、音量/DSP 前每 2048
 *        帧采 64 个双声道测试符号，RAM 暂存、控制任务后打印；不改任何音频策略。
 *        判据：原始输入锚点对金样本的偏移，与同轮 LA 序列偏移对照，区分 USB
 *        输入损失和后级输出损失。普通音频不产生锚点；日志丢失则标明证据不足。
 * 0.89 = L46 调度单点候选：core0 音频泵优先级 22→24，高于固定 fork 的 BT
 *        控制任务 23。0.88 原始输入/LA 同轮丢 6+78 帧，FIFO 6640/6656 B，泵
 *        间隔 9839 us；不改变缓冲、limit、路由、IRQ 或 core1。另加 ISR 收到字节
 *        计数与同步 FIFO 收支快照，区分 FIFO 丢包和到达 ISR 之前的损失。
 *        判据：非周期序列三次连接输入/LA 都保序、收支无新增损失，正常 BT 回退。
 * 0.90 = L46 USB IRQ 亲和性单点对照候选：启动临时 core1 任务调用现有 tusb_init，
 *        使固定 fork 的 USB IRQ 在 core1 注册；任务随即自删。TUD/音频泵/仲裁
 *        仍在 core0，DSP 六文件及 obj 不变，不改 IRQ flags、IDF 或任何缓冲。
 *        0.89 三次为通过/通过/丢 6 帧；失败轮原始输入/LA 同丢，FIFO 收支恒定、
 *        水位正常，指向接收回调之前的损失。以三次真实保序和正常 BT 回退为判据。
 *        必须另验 core1 的时序预算；跨核 USB 字节收支非原子，atomic=0 仅作参考。
 * 0.91 = 撤回 0.90 的 core1 USB IRQ：96k/117 段 COM5 实测 pmax 2704 us、
 *        slow/i2s_ovf 增长；48k 通过不能抵消满负载退化。恢复原 core0 初始化，
 *        项目自有链接包装器仅将 HS USB 的 LOWMED 分配明确选为 level 2；
 *        其他 IRQ 参数原样转发，IDF/TinyUSB/DSP/缓冲/limit 均不改。
 *        判据：48k 非周期输入/LA 三次保序；96k 满配只用 COM5 查 DSP 预算，
 *        本台 LA 不承担 96k 验收。仍需真实手机连接/切编码抓包。
 * 0.92 = L47 有期限的 BT 输出保留：手机切编码实测 ACL 保持，但 A2DP profile
 *        短暂关闭；0.91 在 70/90ms 立即退 UAC，LA 抓到 770/720ms 的 500Hz。
 *        仅已选中的 BT 在 profile closing/CFG 时保留输出，沿用既有 2000ms
 *        协商窗且不续期；新代 PCM 通过原 80ms/40ms 门槛后释放，真实 ACL
 *        断连或期限到期仍退 UAC，普通暂停仍按 100ms 看门狗。代号关门防止
 *        5ms CFG 被 10ms 轮询跳过；异采样率 PCM 在重配前不入 I2S。
 *        判据：手机 SBC/LDAC 双向切换各三次 LA 无 UAC500 窜入，并验暂停、
 *        断连、空闲接入与超时；core1 六文件及 obj、IDF/缓冲/limit 不变。
 * 0.93 = L47 撤销尚未提交的 UAC 回退：0.92 第六次切换，SUSPEND 到 profile
 *        关闭 123ms，100ms 看门狗已请求回退；LA 仍见 780ms UAC500（未通过）。
 *        淡出、静音等待、I2S 重配后均复核有界保留；若此时已确认重协商，
 *        保持 BT 所有权并取消旧请求，必要时恢复原 I2S 时钟。普通暂停时序不变。
 *        判据：双向切编码重复 LA 无 UAC500，取消路径与真实暂停/断连均实测。
 * 0.94 = USB Serial/JTAG 整帧原子发送：旧三次写入在 TX 拥塞时可只送出
 *        AA55/长度和 CRC，载荷丢失；现在沿用既有 BLE 帧缓冲，一次提交整帧，
 *        写入失败立即停止本次 BULK，不增加队列、堆分配或 TX 缓冲。
 *        判据：原 C 负向对照复现残帧；新 C 拥塞不发残帧、BULK 失败不续发；
 *        COM3 连续/停读回读 CRC 零、恢复配置一致，音频/core1 六文件及 obj 不变。
 * 0.95 = 移除 core0 淡入/淡出，PCM 不再逐样本乘渐变增益；先实际拉 MUTE
 *        再失效 BT 代号/关闭数据门或重配 I2S。0.94 切编码 LA 零沿比 MUTE
 *        早约 16ms；强制窗立即写 GPIO，旧待解除事件不得覆盖新强制静音。
 *        同 ACL 的 profile 关闭不清协商窗；普通暂停仅用既有 A+任务周期盖沿。
 *        保留旧 28ms BT→UAC 提交间隔以接住 123ms 迟到的重协商，不作淡化。
 *        手机暂停后持续全零约 3.4s，按用户选择保留歌曲静音段 BT 所有权；
 *        100ms 看门狗、80/40ms 起播、2000ms 协商窗、缓冲与 core1 均不改。
 *        判据：SBC/LDAC 双向各三次 LA 无 UAC500，MUTE 先于切换零沿，
 *        正常按钮暂停无窜音/反复切源，配置一致且 core1 obj 不变；听音待用户。
 * 0.96 = 0.95 硬件静音顺序候选未刷入；用户澄清测试端未接 MUTE，不以硬件窗
 *        推导“噗”声。恢复 a2dp_audio_int.c / hw_mute.c 至 0.94，仅保留 core0
 *        淡入/淡出删除和切源先写数字零；旧 28ms BT→UAC 提交间隔继续保留。
 *        判据只看 I2S DATA/时钟、COM3 和配置；不以 GPIO8 掩盖数据异常。
 *        普通手机暂停全零段保持 BT 所有权，100ms/80ms/40ms/2000ms 与缓冲不改。
 *        SBC/LDAC 各三次切换及真实按钮暂停，必须无 UAC 窜入、无新时钟异常；
 *        首批 PCM 仍需与实际听到的“噗”声对照，不预先宣称听感修复。
 * 0.97 = 0.96 增量构建被已恢复源码的旧时间戳欺骗，残留 0.95 的两个对象；
 *        实机日志出现源码已不存在的 40ms 强制窗，故 0.96 不作最终验收。
 *        重新编译 a2dp_audio_int.c / hw_mute.c 的 0.94 字节；音频核心沿用
 *        0.96 的无淡化、先数字零和 28ms 回退提交，MUTE 行为实际恢复至 0.94。
 *        判据：构建日志确认两文件重编，回读 bin 相同，实机无候选 40ms 行为；
 *        正常双向切编码 DATA/COM3 实测无 UAC 窜入，core1 冻结锚点仍同。
 * 0.98 = USB 协议日志真正零等待：0.97 LOG 虽先 try-lock，随后仍会在 TX 环满时
 *        等待 100ms，可能阻塞蓝牙解码/控制任务；LOG 沿用整帧提交但等待设为 0，
 *        失败立即终止该行后续分片。命令回复的等待、缓冲、DSP/路由策略不变。
 *        判据：实际 C 拥塞日志不等待且不发残帧，普通回读与播放切编码实测 CRC、
 *        DATA 与生产迟到计数；故意停读仍出现的 64B 传输丢失另列未结案。
 * 0.99 = 仅以 -O2 编译现有 fork 的 LDAC 解码单元 ldaclib.c；0.98 立体声
 *        连续播放 core1=100%，codec queue full 并导致 BT 欠载和 I2S 写错。
 *        不改 IDF 字节、DSP 六文件、缓冲、路由、音量或全局编译设置。
 *        判据：同源 LDAC 44.1/48k DATA 连续纯音及串口无新写错/欠载；
 *        96k 仅性能/丢包观察，禁止用本台 LA 验收 96k DATA。冻结 obj 相同。
 *        UAC 的 DSP 前样本跳号与串口重载 CRC 问题独立保留，不预先结案。
 * 0.100 = 增加 RAM 诊断开关 0xB5，默认关闭，协议会话超时自动关闭；
 *         开启后 BT/UAC 都跳过 dsp_audio_process 及 core1 往返，不写 NVS。
 *         保留 0.99 LDAC 优化，六个冻结文件不变。以同源 DATA/输入锚点
 *         和 DSP blocks 不再增长确认真正绕过，比较输入跳帧与 DSP 因果。
 * 0.101 = USB 接收优先级单点候选：仅 HS USB IRQ level 2→3，仍在 core0。
 *         0.100 同源 44.1/48k 正常/跳过 DSP 均发生输入和 DATA 同步跳号；
 *         LA 未采集时也复现，FIFO 收支差额不增长。候选以三次输入/DATA
 *         保序为判据，失败即回退；不改 IDF/DSP/缓冲或路由策略。
 * 0.102 = 撤回 0.101 USB IRQ level 3，恢复 0.100 的 level 2 字节。
 *         0.101 首轮仍在 DSP 前跳过 12 帧，未满足保序判据；该轮 LA
 *         窗口未检出跳号，不宣称优先级提高已修复。保留 0.99 LDAC
 *         -O2 与 0.100 RAM 诊断开关；UAC 输入链路故障继续在账。
 * 0.103 = DS-04A **纯观测**：既有 tud_audio_rx_done_isr 内加整数统计
 *         （包长直方图 bin=bytes>>3、完成间隔直方图、异常首次/最大时间、
 *         最小/最大/零包长），在既有 uac_ctrl diag 任务侧低频打印
 *         `usbisr:` / `usbisr-pktlen:` / `usbisr-gap:` 三条**独立新行**，
 *         不改任何既有行字段口径。无 ISR 格式化/浮点/日志、无新线程/
 *         队列/中断，不改音频行为/缓冲/limit/IRQ 级别/IDF/managed_components，
 *         六个冻结 DSP 文件与 dsp_engine.c.obj 锚点不变。
 *         ⚠ 回调在重新武装之后 ⇒ 间隔只证明服务观察有空洞，不独自
 *         证明主机没送或 DCD 根因。0.102 同源 14 s 基线实测线上删除
 *         228 帧、输入跳号 480 帧（区间内 228），旧失败复现且未修。
 * 0.104 = DS-04C **纯观测**：把 0.103 的包长/间隔直方图（340 B 累加观测）换成
 *         producer 侧**每包 Δwr 账 + 生产者锚点**（DS-04B phase A 设计）：
 *           Δwr == n_bytes_received ⇒ 整包入环；
 *           Δwr == 0 && n > 0       ⇒ **整包被拒收**（dcd_dwc2.c:942 环剩余不足时
 *                                     整包读出丢弃，但 RX_COMPLETE 仍按
 *                                     xfer->total_len 上报 ⇒ 应用层包长正常、
 *                                     usb_pkts 也照常计数，只有 wr 游标不动）；
 *           Δwr >= 2*depth          ⇒ 清环/改配置 ⇒ rebase，绝不当丢包。
 *         生产者锚点 64 符号 / 8192 接收帧、8 槽队列、窗内任一包异常即整窗作废；
 *         新行 `usb-rx-seq:` / `usb-rx-seq-summary:`（独立新行，旧行口径不动），
 *         `usbisr:` 保留 n/bytes/irq_core/gap_* 标量。无 ISR 格式化/浮点/日志、
 *         无新线程/队列/中断/新 SOF，不改音频行为/缓冲/limit/IRQ 级别/IDF/
 *         managed_components；六冻结 DSP 与 dsp_engine.c.obj 锚点不变。
 *         目的：判定"整包 6 帧丢失"发生在入环之前（主机/总线/DCD 静默丢或
 *         未武装窗口）还是环满拒收路径 —— 0.103 无法区分这两者。
 * 0.105 = 0.104 观测的**取数修正 + 两个定量探针**（仍为纯观测，不改音频行为）：
 *         * 修正 Δwr 模运算：`wr_idx` 由 `advance_index()` 维护在 [0,2*depth) 且
 *           回绕时加 `UINT16_MAX-(2*depth-1)`（tusb_fifo.c:420-431）⇒ 0.104 用
 *           `(uint16_t)(wr-prev)` 把**每次环回绕**误判成 rebase，实测
 *           `v104_marked_d0_t1` 因此 producer anchors=0 / invalid=158。
 *           改用真正的模 `2*depth` 推进量，并以 `UAC_RX_DELTA_MAX=2048`
 *           （单包推进上界）判 rebase。
 *         * 新增 `late_n` / `miss_exp`：由完成间隔推算"该间隔本应到来的微帧数-1"
 *           的累计值，作为"服务太晚 ⇒ 端点未武装 ⇒ 硬件整包吞掉"的**定量预测**，
 *           与锚点实测删除包数直接比较；≥5 ms 的间隔记入 `stop_gap`（停流，不算丢失）。
 *         * 新增 `probe_l` / `probe_r`：锚点窗首帧原始样本，供报告肉眼核对数据面。
 *         六冻结 DSP 与 `dsp_engine.c.obj` 锚点仍不变；无新线程/队列/中断/新 SOF、
 *         无 ISR 格式化/浮点/日志，不改音频行为/缓冲/limit/IRQ 级别/IDF。
 * 0.106 = 0.105 基础上加**只读 DWC2 状态探针**（仍为纯观测），用来把"缺失的整包"
 *         判给主机侧还是设备侧：
 *         * ISR 内每包读 `DSTS.SOFFN`（微帧号，HS 每 125 µs 加一；IDF 头
 *           `USB_OTGHS_DSTS_REG`/`USB_OTGHS_DSTS_SOFFN`，基址 0x20300000 与
 *           `dwc2_esp32.h:84` 的 ESP32S31 分支一致）⇒ ΔSOFFN-1 = 设备时间轴上
 *           "没有任何完成"的微帧数 = 缺失包数（与锚点删源量独立对照）。
 *         * diag 任务侧只读 `GINTSTS`（bit20 INCOMPISOIN / bit21 device 侧
 *           INCOMPISOOUT，即 IDF 的 INCOMPLP）：该位不在 DCD 的 gintmsk 里
 *           （dcd_dwc2.c:1190）⇒ 置位后粘滞、不会被清，只读即可回答"设备侧是否
 *           出现过 ISO OUT 传输未完成"。**全程只读：不写 W1C、不清中断、不改
 *           DCD/重装逻辑**。
 *         * 自检：CMODE=0(device)、ENUMSPD=0(HS)、SOFFN 需按 ≈8000/s 前进，否则
 *           本探针结论标 invalid，不许拿坏数据当证据。
 * 0.107 = 0.106 探针的**取数修正**（仍为纯观测）：
 *         * 0.106 把 SOFFN 的寄存器掩码写成 0x3FFF（正确应为 DSTS[21:8] =
 *           0x003FFC00，IDF 头 `USB_OTGHS_DSTS_SOFFN 0x3FFF`/`_S 8`）⇒ 实际只取到
 *           bits13:8（6 bit），值域被截成 0..63，wrap 每 64 微帧一次 ⇒
 *           `soffn_miss` 会把"完成被推迟"(Δ=2) 当丢包，`soffn_bad` 里混入大量
 *           wrap 事件（实测 ~121/s）。0.107 修正掩码，并改口径：
 *           **缺失包数 = ΣΔSOFFN − 样本数**（mod 2^14），对"Δ=2 后紧跟 Δ=0"天然抵消，
 *           只有真正少掉的微帧才留净 +1；另单独输出 `soffn_zero`（被推迟的完成数）、
 *           `soffn_df_sum`、`soffn_max`、`soffn_first_at`，便于报告里逐项核对。
 *         * 只在 <5 ms 的完成间隔上累计（停流/启动两边都不计，不制造假缺失）。
 * 0.108 = 再加**端点级只读探针**（仍为纯观测）：读 EP1 OUT 的 `DOEPINT`
 *         （`USB_OTGHS_DOEPINT1_REG = 0x20300b28`）：
 *         * bit11 `PKTDRPSTS` —— IDF 原文："indicates to the application that an
 *           ISOC OUT packet has been dropped. This bit does not have an associated
 *           mask bit and does not generate an interrupt." ⇒ **设备侧确实丢掉一个
 *           ISO OUT 包**的粘滞标志；DCD 的 doepmsk 只有 STUPM|XFRCM
 *           （dcd_dwc2.c:763），且只写回 XFRC/SETUP/STPKTRX/STSPHSRX/EPDISD
 *           ⇒ 该位不会被 DCD 清掉，只读即可定点归因（含 first_at 时刻）。
 *         * bit8 `OUTPKTERR` = OUT 包错误（同样不在 DCD 清除路径里）。
 *         * 全程只读：不写 W1C、不清任何位。用途：把"设备没完成的那几个微帧"
 *           判给**设备侧丢包**（PKTDRPSTS 置位）还是**主机没送**（该位始终为 0）。
 * 0.109 = **用户提议的最小通路测试版**（`main/uac_bridge_main.c` 的编译期宏
 *         `UAC_MIN_I2S_ONLY=1`；宏为 0 时与 0.108 行为完全一致）：
 *         · 不初始化 Bluedroid、不启动 BLE 调音 ⇒ **无蓝牙功能**、BT 控制器关闭；
 *         · 音频走既有 RAM 直通（`uac2_audio_set_dsp_test_bypass(true)`，与 0xB5 同一
 *           路径、不写 NVS）⇒ DSP 不在音频通路、core1 不被喂块；
 *         · UAC2 描述符/端点/环/反馈、I2S 输出、串口协议、观测代码与六冻结 DSP
 *           文件**一律不变**（冻结文件仍在构建里，只是不被调用）。
 *         目的：把"设备侧软件活动（BT/DSP/中断）是否扰动 USB 接收"这最后一扇门
 *         关掉——若最小版仍然出现同样的内容洞，则成因在主机/链路侧；
 *         若洞消失，则说明是整机软件活动与 USB 接收的相互影响。
 *         ⚠ 这是**诊断用**版本，不是产品形态；测完需把宏改回 0 重编回到正常版。
 * 0.110 = 把 `UAC_MIN_I2S_ONLY` 恢复为 **0**（回到全功能正常构建：BT/BLE + DSP 通路
 *         与 0.103a/0.108 相同，仍带 0.104–0.108 的只读观测代码）。宏本身保留在源码里，
 *         供以后复现 0.109 那次对照（把它改成 1 重编即可）。
 *         判据：0.109 与 0.108 在同一时间段内 A/B/A 实测结果一致（各 87 s 零丢帧）
 *         ⇒ 最小版不是修复；现象是随主机状态发作的，与固件版本无关。
 * 0.111 = 修正 SOFFN 只读探针：0.107 的 0x003ffc00 仍漏掉 DSTS[9:8]，
 *         微帧号被量化到 4 的倍数，可能制造或隐藏净计数差。直接使用 fork IDF
 *         的 SOFFN_M/V/S，并断言掩码为 0x003fff00；不改音频行为、缓冲或 IRQ。
 *         判据：完整 14 bit 值域与回绕向量通过、六冻结 DSP/object 不变。
 *         旧 SOFFN 数据不能据此宣称所有包到达或定位到主机；SOFFN 即使正确
 *         也只观测完成时的 USB 时间，需和连续源 DATA 及接收锚点交叉核验。
 * 0.112 = **移除实体编码器 / USB HID 调主机音量**（用户 2026-10-11 要求；只删功能，
 *         不改音频/DSP 算法、不改任何 limit、不刷机）：
 *         · 删除 `main/hw_encoder.c/.h`、`main/host_volume.c/.h` 及 CMake/初始化的全部引用；
 *           **GPIO10/11/12 不再被配置成控制输入**（app_main 里那次 early_init 一并删除）。
 *         · USB：删除 HID Consumer Control 接口（原第 3 个接口 `ITF_NUM_HID = 2`）、
 *           中断 IN 端点 `0x83`、report 描述符与全部 `tud_hid_*` 回调，
 *           `CFG_TUD_HID 1 -> 0`（应用侧不再留任何实现）。
 *           ⇒ **配置描述符重回 2 个接口（仅 AC + AS）、`wTotalLength` 减 25 B**；
 *           音频功能（Feature Unit 音量/静音、显式异步反馈、采样率/格式/包大小/FIFO 容量）
 *           **一个字节都没动**。
 *         · `dsp_config`：删除编码器启用 `0x83` 与键位动作码 `0x85~0x89` 的字段/API/case。
 *           这些 id 写入落到 GLOBAL 的 `default:` ⇒ 返回**既有的** `status=2 ERR_PARAM`
 *           （不静默成功）；**blob 总长与其它字段偏移不变** —— limAt+35 由"编码器启用"
 *           变成**保留位**（恒写 0、读入忽略），blob ver 仍为 5、**不新增 NVS 迁移/保存**。
 *         · WebUI（`html前端\dsp webui.html`）：删除编码器开关/下发/默认字段/注释与
 *           相关接线文案，只保留 MUTE 的 A/B 延时与极性；blob 解析按**固定偏移**读，
 *           删除字段不会让后面的字节错位。
 *         · 判据：`hw_encoder`/`host_volume`/`tud_hid_*`/`0x83` 应用侧零实现（源码 + ELF 符号）；
 *           配置描述符 `bNumInterfaces=2`、音频端点与 blob 保留位偏移经独立检查通过。
 *         ⚠ 本版**没有实机验证**（无 COM3/LA 权限，设备仍是 0.111）；六冻结 DSP 源文件
 *           逐字节未动，但 `dsp_config.h` 的 `dsp_config_t` 删字段会**传递式重编**
 *           `dsp_engine.c.obj` ⇒ 旧 object 锚点必然失配，按冻结协议如实记录、不掩盖。
 * 0.113 = **core1 整链 + 纯异步 UAC 交接**（用户 2026-10-12 授权重新制作 core1 基线）：
 *         · 一个 core1 worker 独占：描述符队列 → `dsp_engine_process()` → 槽宽转换
 *           （**原地**，无需额外转换/填零缓冲）→ `i2s_out_write()`（整描述符 stage）
 *           → I2S0 DMA。core0 不再等 DSP、不写 I2S、不读处理后的 PCM。
 *         · 两个固定 240×2 int32 PCM 槽 + 描述符（槽号/有效样本数/rate/source/epoch/
 *           has_data/forced_silence/真实旁通/生产开始与入队时刻）。无空槽时**不读
 *           FIFO、不覆写**，回压单独计数；短于 240 帧的合法块按 stride 计入，
 *           跨块拼装沿用 `pcm_dma_stage`，不填零补齐、不丢尾。
 *         · 所有 I2S 变更操作（init/start/reconfig/format/slot/raw/write）都在 core1
 *           执行：core0 把控制项投进**同一个队列**并等待回答，因此重配必然排在它之前
 *           已入队的 PCM 之后（不靠 2 ms 睡眠）；DMA 中断由 core1 创建通道时取得亲和性。
 *           epoch = 输出配置代次，旧 epoch 的块由 core1 明确作废并计 `stale_epoch`。
 *         · **纯 UAC 启动模式**（`UAC_PURE_MODE=1`）：不启动 BT/BLE，但 DSP 引擎、
 *           路由/端口、格式转换与 I2S 输出全部照常；0xB5 只绕引擎、不绕 core1 输出。
 *           串口参数/状态与既有诊断全部保留（BLE/BT 代码与接口保留、本轮不启动）。
 *         · MUTE 判据改为"**任一路输入正在送 PCM**"（UAC 端点有数据 OR BT 交付整块）；
 *           删除 BT 首次接管时无条件压 2 s GPIO 静音的窗口。
 *         · 新增 `c1out/c1max/c1avg` 遥测：每块 producer 准备/排队/DSP 净耗时/端口
 *           转换+提交/总墙钟，deadline_miss、90% 预算越界、回压、stale epoch、
 *           write error/partial/未交付字节、stage pending；帧数改用 2×slot 字节的
 *           **stride** 口径（旧 `n / frame_bytes` 算的是整描述符，不是帧）。
 *         · 逐源 `-O2` 只限 DSP 与 core1 输出单元（`dsp_audio.c`），不加 fast-math、
 *           不改全局优化、**不改任何 limit/缓冲/DMA 段数**。
 *         ⚠ **无实机验证**（无 COM3/LA 权限，设备仍 0.111）：新链的预算与保序、
 *           三开段数都必须由任务 14/15 实测后由经理决定新上限；`dsp_audio.c/.h` 本轮
 *           按授权重做，旧六文件冻结与 `dsp_engine.c.obj` 锚点只作**历史差异证据**。
 * 0.114 = 经理审查修复0.113：静音描述符按样本数写出，16bit原地压缩使用
 *         字节写入；生产者读/提交与所有格式/采样率/槽变更共用短供给门闩，
 *         已入队控制等待真实ACK，不遗留超时ACK。参数重建与引擎初始化改由
 *         core1执行，删除core0板载A9自检及38.4KB PCM；A9明确返回不支持。
 *         DSP计数/64bit窗口原子快照，参数重建及控制停供给另记账；MUTE活动
 *         直接看UAC ISR有效帧与BT有效解码PCM回调，歌曲数字静音仍为活动。
 *         保留BLE/core0协议入口，DSP数学与limit/缓冲不变；待实机验收后冻结。
 * 0.115 = 修正core0 c1out日志多余参数导致dm之后字段错位；undel_b按实际帧
 *         stride转换成字节。不改core1处理/算法/配置/缓冲；本轮新日志参数
 *         数量独立核查，完整UAC实收与DSP预算继续验收，尚未冻结core1。
 * 0.116 = core0同一快照c1slice初稿，CSV格式候选，未刷机。
 * 0.117 = c1slice使用现有空格键值格式与dl_us字段；A4载荷／消费语义及C1对象不变。
 *         段数／三开预算仍须完整窗口的总墙钟与零错误增量证明，尚未冻结C1。
 * 0.118 = 任务51：参数 clamp 之后，三种带增益 EQ（PEAKING／LOWSHELF／HIGHSHELF）
 *         在 g_db == 0.0f 时直接写恒等系数 b0=1、b1=b2=a1=a2=0 并返回：0 dB 这三种
 *         EQ 数学上就是恒等传递，原路径的 inv_a0 舍入与 fmadd/fmsub 残差会被双二阶
 *         递归放大（Task50 完整帧串联 0 dB EQ 最大误差 L79808／R82656 LSB，已超
 *         预声明 8192 界）。HP／LP／bandpass／notch／allpass 不走此分支，所有非零
 *         gain 公式与运算顺序一字未改；只动本函数一个分支，不改 DSP 数学／路由／
 *         端口／缓冲／limit／协议。⚠ 本轮为离线源码与小分支确认，无实机 LA 验证。
 * 0.119 = 任务55：DSP 预算改为**生产模型**，上限两档 **压缩器关 87 段 / 开 75 段**
 *         （T = 567 + 51 + (压缩器开?155:0) + 13×(N+38) µs）。567 是**整条生产链**
 *         （core0 填块 → core1 端口提交）的固定项估计，含最强削波预留与额外抖动，与
 *         旧 176 µs 的 DSP 净耗时模型口径不同，数值变大**不代表**新 DSP 数学变慢。
 *         dsp_config.h 的限宏由 MEASURED 更名 PRODUCTION（不把工程估计当完整最大布局实测）。
 *         dsp_engine.c 的旁通判断／过载统计／活动滤波计数只按**端口 0** 的 HP/LP 计
 *         （配置侧同口径；端口 1 状态与接口保留，I2S1 仍不启用）。WebUI 常量同步为
 *         75 / 87 与生产模型说明。DSP 数学、端口 1 状态、缓冲、协议/blob、CSS 均未改。
 *         ⚠ 本轮只做离线源码，另有一次最终 96k/32 段 LA **待验**（由经理另派席位）；
 *           **未做硬件验证**，不得写成通过。
 * 0.120 = S31 HWLOOP 双二阶内核候选，输入／响度／端口 L/R 及四通道成对交错，
 *         各路转置DFII浮点顺序保持；生产缓冲、75/87限值、2250us预算不变。
 *         含临时core0启动逐位／周期探针，串口初始化后执行，未影响PCM处理路径。
 *         经理修正参考块常驻系数、独立左右样本／状态、零帧与块尾检查；此时未实机。
 * 0.121 = 按用户要求停止所有测试，删除临时启动探针及调用，仅保留S31 HWLOOP
 *         mono／独立双路交错内核与core1完整链路。修正文档中的2x2／仅两通道／
 *         永久冻结等旧描述；不改递归数学、240帧缓冲、75/87限值或2250us预算。
 *         本版只有代码与反汇编审查、固定fork构建，无测试、无烧录、无LA通过声明。
 * 0.122 = 依0.121既有四点串口记录及HWLOOP配对拓扑重算最坏布局预算，
 *         mono13/pair19us，固定项及2250us不变；统一准入关压缩130/三开110。
 *         修复COMP_ON非零非1值绕过赋值前准入，拒收保持原配置。
 *         core1数学/调度/缓冲/协议布局不变；构建及离线代码健康检查，
 *         未烧录、未测速、未做LA抓包或LA重判；最大布局为工程估计待外部实收。
 * 0.123 = 两档准入统一到物理界面最大计数130（关压缩与三开同值）。依据0.122上的
 *         两次30 s最坏布局窗口实测：130段无压缩整链1588us、110段三开1620us，
 *         错误计数全0（96k/32，正式l74全帧无异常、无坏槽）；实测反推单价约
 *         5.4us/段，较本模型13/19us常数保守得多，故压缩器档也容得下满界面，
 *         前端不必再按压缩器开关切换描述口径。
 *         准入函数仍只返回常量；把"上限=模型110/111交界"的旧编译期断言改为物理
 *         上限断言（两档均等于UI_MAX；131段不存在，旧式子在统一后必然编译失败）。
 *         递归数学/240帧缓冲/任务/协议/blob布局不变。
 *         该档130只经一次30 s窗口验证，尚未做长时间soak；若以后高段数出现超期，
 *         按同一整链口径定位并回收模型，不靠环缓存掩盖。
 */
/* 0.124 = 冻结前审查：修复source16/slot24/RJ将3字节打包数据送入4字节翻转的
 *         跨样本错误。24位槽使用24位DSP输出全槽MSB，32位槽窄源的既有24位RJ
 *         补偿保持；DSP数学、环/缓冲、线程、协议与130软件准入不改。
 *         仅离线代码健康/构建，未刷机或LA验证。task66声明130实为110活动段，
 *         不把130软件门槛或旧结构判据当成三开130最大强度实收。
 */
/* 0.125 = core0预算政策显式化：保留13/19us成本模型、130准入及原期限断言；
 *         压缩档模型超2250us生产目标时给编译期提示，模型超过2500us仍拒绝构建。
 *         更正预算块旧计数/验收注释；无PCM、配置事务或core1实现变更。
 *         引用既有设备0.123满载记录，本版未烧录或重新做硬件/LA测试。
 */
#define DSP_FW_VERSION_MAJOR    0
/* 0.126 = core0 UAC/BT arbitration: early codec epochs, event-ended BT wait,
 *         arrival/delivery liveness, generation-checked source commits and
 *         counted high-water trims. A2DP starts, BLE tuning stays off.
 *         Frozen core1/SDK/config and audio buffers remain unchanged. */
/* 0.129 = core0 aptX: streaming resynchronization with Q1.31 scratch reuse,
 *         bounded decoder counters/HD RTP checks; LL uses negotiated rate.
 *         Frozen core1, arbitration, PCM rings and config ABI unchanged.
 *         Offline candidate only; no hardware/audio validation claimed. */
/* 0.130 = core0 UAC reopen: rely on TinyUSB's synchronous EP FIFO reset,
 *         remove the delayed STOP flush, and preserve new PCM on no-op FORMAT.
 *         Resume water level/timeout, PCM gains, mute, BT and core1 unchanged.
 *         Seek click cause/listening remains unverified; no fade or sample trim. */
/* 0.131 = default-off, RAM-only core0 pre-FU input step observer (0xB6).
 *         Retain 16 events; pump-boundary ARM/disarm and stable readback.
 *         No PCM edits, new audio buffer/task/ISR, NVS or core1 changes.
 *         Diagnostic candidate; does not claim a seek/noise repair. */
/* 0.132 = second, default-off tape at the RX_POST_ENQUEUE boundary (same 0xB6
 *         generation, read via [B6,4] / [B6,5,index]): scans every frame of
 *         each fully enqueued packet inside the RX callback, with bounded
 *         coverage accounting and a prev-history break on any gap. Old
 *         0/1/2/3 wire formats are kept byte-for-byte; no PCM/volume/hold/
 *         route/ring-size or core1 changes. Diagnostic candidate only; it is
 *         neither an audio fix nor a noise verdict. */
/* 0.133 = core0 UAC recovery step mitigation: after >=20 ms stereo digital
 *         silence, one large nonzero step in the first 12 ms is joined with
 *         a 1 ms additive-offset release. No FIFO, BT, FU, config or core1
 *         change; raw probes stay before conditioning. Candidate only. */
/* 0.134 = remove the rejected offset conditioner. Preserve and drain same-
 * format UAC OUT tails, then qualify new prefill; no gain/sample edits.
 * Project-managed TinyUSB pre-clear/open hooks required (registered patch).
 * Existing FIFO size, BT arbitration, FU/NVS and frozen core1 unchanged. */
/* 0.135 = retain same-format short prefills on close as well as running tails.
 * Below the start level, release complete PCM unchanged after 30 ms instead
 * of consuming/discarding it. No new buffer, PCM edits, FU/config or core1
 * change. Existing OUT hooks and finite FIFO overflow policy are retained. */
/* 0.136 = conservative core0 UAC latency step: use the existing FIFO_COUNT
 * target parameter, 3328 -> 3072 bytes. FIFO capacity, feedback smoothing/
 * clamps, prefill/retained-tail policy, PCM/FU, BT and frozen core1 unchanged.
 * No end-to-end latency or audible-pop improvement is claimed by the build. */
/* 0.137 = user-selected 2048-byte FIFO_COUNT target; replaces the unflashed
 * 3072-byte 0.136 candidate. Built-in gain quantization changes with the target;
 * FIFO capacity, PCM, prefill/tail handling and frozen core1 stay unchanged.
 * This is a host-built candidate, not a measured latency/stability result. */
/* 0.138 = core0 USB format mailbox and bus lifecycle: latest requests survive
 * a yielded output reconfigure, input FIFO boundaries stay in USB callbacks,
 * and resume restores a still-open alternate through the existing prefill.
 * Keep 2048B feedback target, PCM/FU, frozen core1 and persistent config. */
/* 0.139 = close the USB input-transaction readiness window before publication:
 * a stale notification cannot apply a new format until its FIFO/RX boundary
 * finishes; clear incompatible late ISO bytes before rearming RX. Replaces
 * the unflashed, withdrawn 0.138 build; no PCM buffer. */
/* 0.140 = Q15 fractional FIFO feedback gains, pump-owned FU gain cache,
 * and RAM-only legacy diagnostics OFF at boot. Keep PCM math/rounding, FIFO
 * target/capacity, tail policy, B6 dual tapes, NVS and frozen core1 unchanged. */
/* 0.141 = core0 codec budget: optimise aptX/LDAC units, fix sync output-capacity
 * arithmetic, bound LDAC transport/frame parsing by actual input and expose
 * bounded codec progress/wall-time evidence. Frozen core1/queues unchanged. */
/* 0.142 = split the *playback* high watermark: LDAC (vendor 0x12D / codec 0xAA)
 * at a really applied 96 kHz may queue to 70 ms before the existing 240-frame
 * trim fires, so the observed 48,128 B (~62.7 ms) bursts are absorbed instead of
 * being cut. Prebuffer 40 ms, warm-up 60 ms, the 65536 B ring, trim granularity,
 * every other codec/rate and the whole decode path are unchanged. */
/* 0.143 = retain selected BT across the SUSPEND-to-CFG gap. A 300 ms tentative
 * pause grace promotes to the existing confirmed reconfiguration wait; ordinary
 * pauses expire, and fallback rechecks the same hold under the PCM epoch lock.
 * The accepted 0.142 watermarks, decoder path and frozen core1 are unchanged. */
/* 0.144 = LDAC same-CFG quality resumes keep a finite 600 ms ownership grace
 * across both SUSPEND and START/prebuffer; other codecs keep 300 ms. Serialize
 * the BT read epoch check with its nonblocking core1 enqueue, retain the output
 * epoch ABI, and expose a coherent normalized ring ledger. Watermarks unchanged. */
/* 0.145 = BT underrun re-prebuffers instead of dribbling silence: after two
 * consecutive starved blocks the BT gate closes and the existing hold window
 * refills to BT_PREBUFFER_MS before uac_poll_source_policy reopens it. Keeps
 * the 40/60 ms hysteresis, every watermark/trim policy and frozen core1. */
/* 0.146 = BT gate opens later so start-up survives A2DP jitter: prebuffer
 * 40->60 ms and hold-window drain 60->80 ms (20 ms hysteresis preserved, the
 * 0.52 race stays fixed). Play-time trim is decoupled back to 60 ms (70 ms for
 * LDAC 96k) so the accepted 0.142 steady state is untouched. */
/* 0.147 = revert 0.146's watermark change (no measured improvement for the
 * reported "seek" artifact): gate back to 40 ms / hold drain 60 ms. The 0.145
 * suspended-tail drain, the 0.144 ownership grace and the decoupled 60/70 ms
 * play-time trim are all retained unchanged. */
/* 0.149 = rollback of 0.148 (Codex 99 disposition). 0.148's token-isolated
 * drain changed audible behaviour and regressed LDAC 990 / UAC-playing codec
 * switches, so this build restores the exact 0.147 worktree behaviour under a
 * new incremented version number. It is a rebuild, not a byte-identical 0.147
 * backup. The 0.148 source and APP are preserved under releases/rollback. */
/* 0.150 = per-codec start-up watermark. LDAC 990 kbps could not play at all
 * because the gate demanded the shared 40 ms while LDAC packets are larger and
 * further apart. LDAC now gets its own prebuffer/high pair (44.1/48k 80/100 ms,
 * 88.2k 55/70 ms, 96k 50/65 ms) and, crucially, the push-side floor/high uses
 * the very same values so the 0.52 open-never-happens race cannot return.
 * Other codecs keep 40/60 ms; play-time trim, 0.144 grace and core1 unchanged. */
/* 0.151 = PCM injection for phone-free testing (CONFIG_UAC2_BT_INJECT). USB
 * audio frames are pushed straight into the BT decoded-PCM ring while the
 * firmware pretends an A2DP source is connected/streaming, so ring level, start
 * gate, pump, core1 and trim all run the real BT path. Vendor/codec auto-cycles
 * every 15 s to exercise the per-codec watermarks. No phone needed. */
/* 0.152 = disable 0.151's PCM injection. On the bench it caused UAC/BT source
 * oscillation (state-trace flooding) and ended in an assert (MCAUSE 0xdeadc0de),
 * so injection is compiled out and the 0.150 behaviour is restored. The injection
 * code stays in the tree behind CONFIG_UAC2_BT_INJECT for a later, safer rework. */
/* 0.153 = give the BT ring real headroom and stop trimming on a 60 ms knee.
 * Ring 8192->16384 frames (65536->131072 B): 44.1k 371 ms, 48k 341 ms,
 * 88.2k 186 ms, 96k 171 ms. LDAC watermarks rise to 80/100 ms (<=48k),
 * 80/100 ms (88.2k) and 100/120 ms (96k). Play-time trim now uses the same
 * per-codec high watermark instead of the fixed 60/70 ms knee, so a jittery
 * source no longer forces periodic whole-block drops. Console baud 115200->
 * 921600 for observable logs. core1 and the 0.144 grace are untouched. */
/* 0.154 = tick-level BT trace (the answer to "why can't we see what the MCU
 * did"). Every pump tick appends an 8-byte summary (ring level in 1/100 ms,
 * gate/streaming/draining bits, delivered/silence/trim bits) to a 4096-entry
 * RAM ring = 11 s at 96k. Once per 10 s only ANOMALIES (gate flips, trim,
 * underrun) are printed, so 115200 baud finally suffices. No behaviour change. */
/* 0.155 = full revert of 0.153/0.154. Doubling the ring to 131072 B plus a
 * 32 KB tick-trace array added ~96 KB DRAM and the board entered a
 * LoadStoreError boot loop (MCAUSE=3), so ring, LDAC watermarks and play-time
 * trim are all back to the 0.150 values that were confirmed able to play LDAC
 * 990, and the tick trace is removed. 0.152's injection removal is kept. */
/* 0.156 = ring doubled ALONE and verified stable. The 0.153 crash was the
 * tick-trace code, not capacity: SOC_DRAM_HIGH(0x2F080000) - _bss_end leaves
 * ~176 KB free after the 131072 B ring. LDAC watermarks now use the enlarged
 * ring (96k 100/120 ms of a 171 ms ring) and play-time trim shares the same
 * per-codec knee instead of dropping whole blocks at 60/70 ms. */
/* 0.157 = fix the old-tail drain's two real defects (Codex 99).
 * (a) Eligibility: begin_drain now requires s_bt_played_epoch == current epoch,
 *     i.e. this generation really delivered PCM. Prebuffer that never played is
 *     dropped cleanly instead of being eaten as a phantom "old tail".
 * (b) Submittability: submit_pcm32_epoch accepts the draining epoch even though
 *     streaming is already false, so the tail is actually handed to core1 rather
 *     than read-and-discarded. Rate change still drops the tail cleanly.
 * Only normal (non-draining) submits update the played epoch. */
/* 0.158 = close Codex-99 defects (c) and (d).
 * (c) A tail shorter than one DSP block can never satisfy pop's can>=len, so
 *     the old code waited out the full 150 ms while emitting silence. Now the
 *     drain ends immediately and the residual frames are dropped cleanly.
 * (d) Underrun re-prebuffer keyed off the cumulative bt_silence_runs, so two
 *     distant underruns could trigger it and the counter's monotonic meaning
 *     was destroyed. A separate bt_silence_streak now counts consecutive
 *     starved blocks, cleared on a real delivered block and at source switch;
 *     bt_silence_runs is cumulative again. */
/* 0.159 = finish Codex-99 defect (b): START must not let the normal pop path
 * bypass the frozen drain endpoint. Two changes:
 *   · a STARTED event now ends any outstanding drain (epoch flush) *before*
 *     restoring streaming, so a new lifecycle never inherits an old tail;
 *   · pcm_ring_pop's normal branch requires !s_bt_draining, so while a tail is
 *     being drained only the drain branch (which honours s_drain_end) can read.
 * Without this, streaming returning true made the unbounded normal branch win
 * and the unsigned (s_drain_end - s_pcm_tail) could wrap. */
/* 0.160 = observability only, no behaviour change. The bt-flow line now also
 * prints drain_read (bytes read out of the frozen old interval) and drain_blk
 * (blocks the drain actually handed to core1), so a codec switch can be judged
 * as "old tail really played" vs "dropped cleanly" instead of argued about. */
/* 0.161 = make the drain ledger cumulative like the others. 0.160's
 * drain_read was still zeroed at every epoch flush and at every drain start,
 * so the very number meant to answer "how much old tail was read" was erased
 * exactly when it would be inspected. It is now monotonic, matching
 * flush_bytes and drain_blocks. Observability only. */
/* 0.162 = harden the drain read boundary. The drain branch compared an
 * unsigned (s_drain_end - s_pcm_tail); if tail ever passed end it would wrap to
 * a huge value and let the drain read *new* generation data as if it were the
 * old tail. Now the difference is signed and must be > 0 and >= len. */
/* 0.163 = fix a real mis-classification in the drain eligibility check.
 * s_bt_played_epoch and s_route_hold.epoch are both statically zero, so the
 * numeric comparison alone reported "this generation played" on the very first
 * SUSPEND after boot, letting a never-played prebuffer be drained as an old
 * tail. A separate s_bt_played_valid flag is now required, set only by a
 * successful non-draining submit and cleared on disconnect. */
/* 0.164 = measure the switch gap. Whenever bt.streaming goes false a timestamp
 * is taken; the next real delivered block turns it into bt_last_gap_ms /
 * bt_max_gap_ms, printed on the bt-flow line as gap_last/gap_max. This is the
 * direct way to separate "the silence is too long" (seek-like pause) from
 * "the silence was short but content jumped" (a seam / core1 issue). */
/* 0.165 = separate supply-side stalls from consumer-side starvation, the thing
 * that was never recorded before. packets (decoded packets received from the
 * phone) and delivered (blocks handed to core1) are now tracked independently:
 * when streaming is true but packets stop growing a starve window is opened and
 * its length is accumulated into starve_last/starve_max/starve_n (>=200 ms only).
 * Large starve numbers ⇒ the phone/ link stopped feeding; small starve with an
 * emptying ring ⇒ the consumer is too fast. Different fixes; no more guessing. */
/* 0.166 = remove dead code left from the 0.142 play-watermark experiment:
 * s_bt_play_high_ms, BT_PLAY_HIGH_MS_LDAC96/OTHER and bt_trim_high_bytes had no
 * remaining callers since 0.156 moved both trim sites to the per-codec
 * bt_trim_high_bytes_id(). No behaviour change; smaller surface to misread. */
/* 0.167 = tick-level BT trace, done safely this time. 0.154's version logged
 * from inside the pump and the board rebooted; that was almost certainly the
 * stale incremental build (0.166 proved identical edits are stable after a
 * clean rebuild), but this version removes the risk anyway: the pump only
 * appends an 8-byte summary to a 2048-entry ring (16 KB, ~5.5 s at 96k) and
 * every ESP_LOG call stays in the ctrl task. The summary prints only when the
 * gate flipped >=2 times or a trim happened in the window. */
/* 0.168 = BT actually plays from a PC source. Measured with AltA2DP on Windows:
 *   ACL up, cfg_apply=1, vid=0x12D cid=0xAA (LDAC negotiated),
 *   pkt=3014 packets arrived, but streaming stayed 0 and delivered stayed 0,
 *   with ovf=2983 because the gate never opened.
 * Cause: that driver pushes PCM without ever sending ESP_A2D_AUDIO_STATE_STARTED,
 * and the code waited for it forever. Now the first well-formed packet on a
 * connected+selected route raises streaming itself (SUSPEND/DISCONNECT still
 * clear it). Exposed as stream_autostart / autostart= in bt-flow.
 * Also removed the 16 KB tick trace (0.167): it pushed heap down to 18 KB at
 * ring 16384 and BT could not init -- ring stays 8192 for 84 KB free heap. */
/* 0.169 = unblock the 0.168 auto-start. a2dp_audio_set_output_selected() has no
 * caller anywhere, so s_route_hold.selected is always false; requiring it in
 * the auto-start condition kept autostart at 0 while pkt kept climbing
 * (measured pkt=3010, ovf=2979, delivered=0). The data packet itself is the
 * decisive evidence; connection + valid format are enough. */
/* 0.170 = relax auto-start further. After 0.169 (route_hold no longer required)
 * autostart was still 0 while pkt climbed to 4542, so the bits/channels
 * whitelist was also rejecting. Format checking is already done by
 * push_pcm_to_ring with the same values, so the duplicate gate only blocks the
 * legitimate "data arrived but format not yet published" case. Now the only
 * requirements are: link connected, not injecting, no CFG pending/failure.
 * The format seen at auto-start is exported as as_bits/as_ch in bt-flow. */
/* 0.171 = LDAC 96 kHz actually starts. 0.168-0.170 chased the wrong variable:
 * START does arrive (bt_cycle.txt 38.673 s state=1), and autostart staying 0
 * is correct because that START had already set streaming. The real blocker is
 * arithmetic: the LDAC 96k prebuffer is 100 ms = 76800 B, while the ring was
 * 8192 frames = 65536 B (85.3 ms). `level >= prebuffer` was unreachable, so the
 * gate never opened: pkt grew, ring sat at 63488 B, ovf climbed, delivered=0.
 * Ring is now 12288 frames (98304 B, 128 ms); a compile-time check in
 * a2dp_audio.h makes the threshold-vs-ring inequality explicit, and bt-policy
 * prints fw/ring_b/ldac96_prebuf_b/rate/prebuf_b/high_b every status period. */
/* 0.172 = diagnostic split of the LDAC wall time. The 0.171 PC test showed
 * 5.46 s of PCM delivered in 21.6 s (25% real time) at 96k/990 and 40% at
 * 96k/660, with 83-91% RTP sequence jumps. Add pure ldacBT_decode() and PCM
 * callback accumulators (avg/max) to ldac-stat so the next run can say which
 * half is slow; no audio path or policy change. */
/* 0.173 = two evidence-driven changes from the 0.172 timing run:
 * 1) open the BT gate in the same locked switch instead of leaving it for the
 *    10 ms ctrl poll (that poll was delayed ~100-300 ms by the diag block, so
 *    the first 96 kHz start played silence and the 100 ms fallback could fire);
 * 2) one task-runtime snapshot after 10 s of BT streaming. ldac-stat proved
 *    pure decode 10.4 ms/frame at 96k and 6.4 ms at 48k with a 0.25 ms PCM
 *    callback, while SYS_STATUS reported core0 only 27% busy and the SDK
 *    logged "codec queue full (20)" thousands of times. The snapshot decides
 *    whether A2DP_DECODER is CPU-bound or preempted/blocked. */
/* 0.174 = stop the queue-full log flood that starved uac_ctrl during LDAC.
 * 0.173 measured 6014 "Pkt dropped, codec queue full (20)" warnings in 60 s;
 * the ctrl task printed no bt:/bt-flow lines at all for that whole window,
 * although A2DP_DECODER used only 23% CPU and IDLE0 stayed at 57%. The SDK
 * now logs that condition at most once per second (counter kept), and the
 * one-shot task snapshot also prints vTaskList states. No audio policy or
 * ring/gate change in this version. */
/* 0.175 = build the libldac-dec hot code with its intended optimisations.
 * The 0.172-0.174 runs measured 5.4 ms/frame at 48k and 8.1 ms at 96k (real
 * time budget 2.667 ms), with the decoder task CPU-bound and the task WDT
 * firing at 48k. The libldac-dec CMakeLists specifies -O3/-funroll-loops/
 * -ffast-math, but the bt component compiles these two TUs itself with -O2
 * only. Apply those flags to ldacBT.c and ldaclib.c and re-measure dec_avg_us.
 * No algorithm, buffer, policy, or core1 change. */
/* 0.176 = 0.175 verified that the previous source-property placement did not
 * reach the compile command (still -O2). Move the LDAC -O3/-fast-math flags
 * into set_bluedroid_host_compile_flags() (parent directory scope of the bt
 * target) and rebuild; same audio behaviour otherwise. */
/* 0.177 = 0.176 proved source-level COMPILE_OPTIONS still did not reach the
 * LDAC compile commands. Diagnostic: -O3/-funroll-loops/-ffast-math applied to
 * the whole bt component, then measure ldac-stat dec_avg_us. This is a test
 * build; the flag will be narrowed or reverted after the measurement. */
/* 0.178 = set the LDAC source compile options in components/bt/CMakeLists.txt
 * itself (the directory that defines the bt target, before
 * idf_component_register). 0.176/0.177 proved the subdirectory/function scopes
 * did not reach the compile command and the component-wide -O3 broke unrelated
 * GATT code. This time only ldacBT.c/ldaclib.c get -O3/-fast-math. */
/* 0.179 = use COMPILE_FLAGS (string) instead of COMPILE_OPTIONS for the two
 * LDAC TUs; the latter did not appear in the compile command. Same flags,
 * same files, no other change. */
/* 0.180 = component-level -O3 (last flag wins over the target -O2) with the
 * GATT stringop/array-bounds warnings demoted. Diagnostic for decoder timing;
 * no source logic change. */
/* 0.181 = round close: keep the 0.171 ring, 0.173 gate, and 0.174 log-rate
 * fixes; revert the 0.175-0.180 compiler experiments because the per-source
 * and component -O3 flags were always followed by the target's -O2 (last flag
 * wins), while the component-wide unroll/fast-math grew the image by 260 KB
 * without a measured decode gain. Evidence for the remaining limit: 5.4 ms/
 * frame at 48k and 8.1 ms at 96k against the 2.667 ms real-time budget, the
 * task WDT firing during 48k decode, and queue-full drops. 96k/990 remains
 * CPU-limited on this decoder; no claim of continuous 96k playback. */
/* 0.182 = stage profiler for the LDAC decoder. 0.181's total dec_avg is
 * 5.4 ms/frame at 48k/990; add unpack/decode/imdct/setpcm microsecond
 * accumulators inside ldaclib_decode() and print per-stage averages in
 * ldac-stage. Diagnostic only; no behaviour change. */
/* 0.183 = function-level #pragma GCC optimize("O3","unroll-loops","fast-math") in
 * ldaclib.c/ldacBT.c. 0.180 proved the component/source CMake flags are always
 * followed by the target -O2; the pragma overrides the command line for the
 * decoder functions themselves. Diagnostic for the 2.8 ms unpack + 3.1 ms
 * IMDCT stages measured by 0.182. */
/* 0.184 = raise uac_ctrl from priority 6 to 21. Under a 990 kbps stream the
 * A2DP_DECODER task (19) stays runnable continuously; at 6 the source policy
 * never ran and the switch to BT only happened after the source stopped. 21 is
 * above decoder/BTC (19) and below HCI (22), so the 10 ms policy poll gets a
 * slot without disturbing the BT controller path. Scheduling fix only. */
/* 0.185 = final after reverting the 0.182-0.184 experiments. Behaviour is the
 * 0.181 set (ring 12288 / gate-on-switch / queue-full log rate limit) plus the
 * 0.172 decode/callback timing. Round-2 measurements kept for the record:
 * 0.182 stage profile at 96k = unpack 2.8 ms + decode 1.7 ms + imdct 3.1 ms +
 * setpcm 0.5 ms per frame; 0.183 O3 pragma gave no clean gain; 0.184 ctrl
 * priority 21 moved the switch earlier but a 96k switch/reconfig preempted the
 * decoder for ~1.6 s and dropped the stream. None shipped. */
/* 0.186 = cache-placement test for the 3.1 ms/frame IMDCT. Put the 8 IMDCT
 * tables (~6 KB) in DRAM and proc_imdct_core_ldac in IRAM, so the hot loop no
 * longer fetches tables/code from the 80 MHz DIO flash cache while the BT
 * stack streams. Same arithmetic; revert if ldac-stage does not improve. */
/* 0.187 = 0.186 IMDCT DRAM/IRAM placement measured and reverted. It lowered
 * heap to 40.7 KB and dec_avg went 8.1 -> 9.6 ms/frame at 96k (callback also
 * rose to 1.5 ms), so the 3.1 ms IMDCT is not flash-cache-bound. Behaviour is
 * back to 0.185 (ring/gate/log fixes only). */
/* 0.188 = measure A2DP_DECODER's own CPU share. The 96k dec_avg wall time
 * (8-10 ms) may be mostly preemption; the task run-time counter delta over 1 s
 * separates decoder CPU from waiting/preemption. Diagnostic only. */
/* 0.189 = per-stage task CPU. 0.188 proved total decoder CPU is 63% at 96k,
 * but the 0.182 stage numbers were wall times and could be mostly preemption.
 * Accumulate the current task's ulRunTimeCounter around unpack/decode/imdct/
 * setpcm; this says where the 5.2 ms/frame CPU actually goes. Diagnostic. */
/* 0.190 = IMDCT-only O3/unroll/fast-math via push_options/pop_options. 0.189
 * per-stage task CPU at 96k: unpack 1743 + decode 1014 + imdct 2058 + setpcm
 * 347 us/frame = 5162 us. Measure imdct_us CPU; keep only if it drops. */
/* 0.190 = IMDCT-only O3 pragma measured and reverted: imdct_us CPU stayed
 * 2058 -> 2124 us/frame at 96k. Final in-service is 0.189 (per-stage CPU
 * diagnostics, 0.181 behaviour). */
/* 0.191 = boot FPU benchmark: 100k dependent float FMAs. If this is tens of
 * ms instead of ~1.5-3 ms, the LDAC IMDCT is running through a trap/emulation
 * path and the 2x deficit is an FPU-enable problem, not algorithm cost. */
/* 0.192 = delayed FPU benchmark task (logs at 5 s, after USB-Serial/JTAG is
 * up). Distinguishes hardware FPU from trap/emulation for the LDAC IMDCT. */
/* 0.193 = FPU benchmark repeats every 5 s so a normal serial capture sees it. */
/* 0.193 FPU benchmark measured 100k dependent FMAs in 4482 us (~14 cycles/op),
 * i.e. the hardware FPU is active; the LDAC IMDCT is not trap/emulation-bound.
 * 0.189 remains the final in-service build; the bench task is not shipped. */
/* 0.194 = 48 kHz LDAC cap test: sink caps advertise only 44.1/48 kHz and the
 * config builder prefers 48. Purpose: check whether continuous playback is
 * achievable at a rate the decoder can sustain. This is a candidate, not a
 * claim that 96k/990 is fixed. */
/* 0.194 = 48 kHz cap test measured: 48k/990 dec_avg 5.47-5.51 ms/frame,
 * 4 task WDT, 24 queue-full; still ~1.3x too slow at HQ. Cap not shipped;
 * final remains 0.189. */
/* 0.195 = IMDCT micro-optimisation: 16-byte align a_work and hoist the
 * a_work[index1] loads into locals (remove duplicate loads / help ILP).
 * Measure imdct_us CPU; revert if no gain. */
/* 0.195 = IMDCT align/hoist measured: imdct_us 2058 -> 2080 us/frame, no
 * gain. Reverted; final remains 0.189. */
/* 0.196 = unpack bit-reader restrict qualifiers. unpack is 1.74 ms/frame and
 * the pointer-based bit reader may reload p_block/p_loc due to assumed
 * aliasing. Add restrict to the static helpers; measure unpack_us CPU. */
/* 0.196 = unpack restrict measured: unpack_us 1743 -> 1656 us/frame (~5%,
 * within noise), total still ~5.0 ms; not a fix. Reverted; final 0.189. */
/* 0.197 = LDAC core0 fast-kernel candidate: bounded reservoir/Huffman peek-drop,
 * fused dequantization, paired DCT butterfly stages and batched hardware PCM
 * conversion. No scheduler, ring, other codec, core1 or output-ABI change.
 * Compilation is not a throughput or continuous-playback acceptance result. */
/* 0.198 = LDAC_CORE0_FAST_V2: true N/2 positive complex FFT with
 * pre/post rotations replaces the reflected DCT butterfly graph.
 * Keep 0.197 unpack/dequant/PCM; no core1, ABI, ring, scheduling
 * or WDT changes. CPU/numeric/continuous acceptance not yet run. */
/* 0.199 = LDAC 32-bit sink/config support on top of V2 FFT kernel. The phone
 * can offer 32-bit/96k; PC AltA2DP stays 24-bit. Long-capture candidate. */
/* 0.200 = add LHDC v5 24-bit up to 96k (192k excluded), LC3plus A2DP
 * parser/SEP, keep aptX LL; LDAC V1 kernel; 48k LDAC/LC3 test base. */
/* 0.201 = core0 Classic codec registry/guard, LDAC default-off,
 * rate-sized LHDC admission (default <=48k), LC3plus packed24 and CIE fixes.
 * Framework build is not a source/performance/10-minute acceptance result. */
/* 0.214 = restore the AVRCP CT/TG role before A2DP init so Windows' classic
 * audio stack keeps the A2DP profile open, and log the A2DP disconnect reason.
 * Core0 only; codecs, PCM path, ring, DSP and core1 are unchanged. */
/* 0.215 = diagnostic: AVDT/AVCT/BTC trace at DEBUG plus local BD address log
 * to find why Windows closes AVDTP right after SBC SET_CONFIG. Not a release. */
/* 0.216 = Windows/AVRCP-compatible sink with a codec set that fits internal
 * RAM: LDAC SEP disabled (not in this project's codec requirement and its
 * ~30 KB working set aborted the BT stack), LHDC-v5 96 kHz SEP enabled, and
 * the BT PCM ring reduced 12288 -> 8192 frames (+32 KB heap) with the 96 kHz
 * non-LDAC watermark still compile-time enforced. Core1/DSP untouched. */
/* 0.217 = headless SSP pairing: NoInputNoOutput IO capability, accept
 * numeric-comparison confirmation and answer passkey requests, log auth OK.
 * Windows previously aborted A2DP right after SET_CONFIG because SSP
 * confirmation was never answered (HCI auth fail 0x05). */
/* 0.218 = diagnostic: AVDT/AVCT/BTC/AVRC/A2D trace at DEBUG to capture the
 * exact AVDTP/AVCT exchange that makes Windows close a paired A2DP link.
 * Not a release build. */
/* 0.219 = same diagnostic with trace level 5 (DEBUG) so AVDT_TRACE_DEBUG /
 * AVCT debug are actually compiled in. Not a release build. */
/* 0.220 = A2DP-only profile again (BT_A2DP_AVRCP_ROLE=n) while keeping the
 * headless SSP pairing fix and the 0.216 RAM/codec fixes. Windows closed the
 * paired link immediately after this sink opened the AVCTP channel. */
/* 0.221 = diagnostic: force avdt_cb.trace_level=5 in the fork and L2CAP
 * debug, so the AVDTP command that Windows aborts on is visible. Not release. */
/* 0.222 = diagnostic: hex-dump every AVDTP TX/RX message (fork patch) to
 * settle message direction and the exact CLOSE reason. Not a release. */
/* 0.223 = fix the A2DP role race: a local SNK never initiates AVDTP and the
 * ACP signalling deadline is 6 s, so Windows' SET_CONFIG (~2.1 s) wins. */
/* 0.224 = diagnostic: hex-dump every outgoing AVDTP signaling packet, so the
 * SEP capability responses Windows rejects can be compared with AVDTP. */
/* 0.225 = clean candidate of the 0.216-0.223 fixes with all AVDT diagnostic
 * dumps and trace levels removed again. */
/* 0.226 = SBC-only test variant: AAC decoder/SEP disabled so the PC must
 * negotiate SBC. Not a release; reverted after the SBC point. */
/* 0.227 = restore the 0.225 codec set (AAC back) after the 0.226 codec probe. */
/* 0.228 = quiet the AAC decoder's per-100-packet/per-100-frame UART logs that
 * ran inside the decode path (32-44 ms max calls, one underrun per soak). */
/* 0.229 = SBC-only probe: AAC + aptX family SEPs disabled so the PC must pick
 * SBC. Not a release; reverted after the SBC point. */
/* 0.230 = restore the 0.228 codec set after the SBC probe (SBC configured but
 * Windows never sent media; tracked for the next AVDT-trace round). */
/* 0.231 = diagnostic: SBC-only SEPs + AVDT/AVCT/BTC/A2D trace to see whether
 * Windows ever sends SET_CONFIG/OPEN/START for SBC. Not a release. */
/* 0.232 = diagnostic: SBC 48 kHz-only sink caps to test whether Windows'
 * 44.1 kHz endpoint format state is what blocks the A2DP START. Not release. */
/* 0.233 = restore the 0.230 codec set and diagnostics after the race probe;
 * the A2DP role race is documented for the next fix round. */
/* 0.234 = fix the A2DP role race: a sink ignores BTA_AV_PENDING_EVT (the
 * signalling-channel timeout) instead of starting a source-role SDP/AVDTP
 * open, so the Windows source's SET_CONFIG is never preempted. */
/* 0.235 = test variant: SBC-only SEPs on top of the 0.234 race fix, to see
 * whether the sink now stays the acceptor and actually receives SBC media. */
/* 0.236 = production restore after the 0.235 probe: full 0.230 codec set plus
 * the 0.234 sink pending-gate fix. */
/* 0.237 = SBC-only SEPs on the 0.234 race fix, with the PC Settings/endpoint
 * recovered, to obtain the SBC media point. */
/* 0.238 = SBC 48 kHz-only sink caps on the race fix, to test whether the
 * Windows 44.1 kHz endpoint format is what blocks the SBC stream. */
/* 0.239 = SBC caps restored (44.1|48) after the 0.238 48k-only link-abort test. */
/* 0.240 = aptX probe: AAC off, aptX-LL/HD off, plain aptX + SBC on, so the PC
 * must negotiate aptX for the next codec point. */
/* 0.241 = aptX HD probe: AAC and plain aptX off, aptX HD + SBC on. */
/* 0.242 = production restore: full codec set + 0.234 sink pending-gate fix. */
/* 0.243 = diagnostic: log each advertised SEID's codec_info at GET_CAP so the
 * LHDC/LC3plus SEPs can be proven over AVDTP without a phone source. */
/* 0.245 = isolated codec recovery, checked SBC/LHDC packet bounds and explicit
 * SBC/aptX config failure; LHDC core0 -O2 and worker CPU/heap evidence. */
/* 0.246: bound core0 input work, fence late decoded PCM, isolate peers/control
 * transports, enable BLE tuning; selected BT content has no latency trim. */
/* 0.247: bound aptX resync, use worker CPU for budget isolation and release
 * terminal BT failures before bounded UAC source/clock recovery. */
/* 0.248: make the actual core0 decoder worker the sole PCM-format publisher;
 * deliver public CFG notifications in STARTED too, without late-state writes. */
/* 0.249: retain worker format across same-ACL profile rebuild; use LHDC V5's
 * own two-byte media header and preserve HD/LHDC failure evidence after close. */
/* 0.250: LHDC96-only DFT-3/5 butterflies and removal of unused w2 tables;
 * exact auxiliary heap admission, transactional setup and retained timing. */
/* 0.251: keep the 96k transform/FFT/butterfly entry points out of flash
 * inlining. 0.250 is withdrawn before deployment; confirm linked IRAM too. */
/* 0.254: allow bounded BLE credit recovery without truncating configuration. */
#define DSP_FW_VERSION_MINOR    254
#if DSP_FW_VERSION_MAJOR > 255 || DSP_FW_VERSION_MINOR > 255
#error "SYS_STATUS version fields are u8; use the next major version instead of wrapping"
#endif
