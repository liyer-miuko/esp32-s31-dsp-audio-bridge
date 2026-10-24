# 控制协议

实现依据：真源 `main/serial_proto.c`、`dsp_config.c/.h`、`amp_control.c`、`ble_tuning.c`；UI 源为 `webui/app.js`、`config-codec.js`、`amp-control.js`。本快照版本/身份见仓库根 SOURCE_MANIFEST.json；二进制发布状态见 firmware/releases/README.md。

## AA55 与传输

`AA 55 | len:u8 | payload[len] | crc8`。CRC8 初值 0、多项式 0x07，覆盖 len+payload。最长 payload 255 B。

参数请求 `[msg_id,01,block,ch,param,idx,value...]`；ACK `[msg_id,80,status]`；ERROR `[msg_id,82,code,detail]`。按长度与命令上下文区分参数和短控制请求。新 DSP 消息 id 跳过 B8；老参数 envelope 仍由后端兼容。

参数 ACK：0 接受，1 block 无效，2 参数不支持，3 索引越界，4 宽度错误，5 非法值，6 功能未实现，7 超预算并回滚本包，8 预设无效/为空。ERROR 和 B8 状态码含义不同，不能混用。部分旧参数仍会夹取范围并返回成功；验收必须比较实际回读值。

| 命令/回复 | 当前字段 |
| --- | --- |
| A0 → A1 | 仅保活 |
| A2 → BULK | 首包 `[00,02,total:u16,data≤251]`，续包纯数据；续包优先于 B8/遥测识别 |
| A3 LOG | payload 去掉首字节后按完整换行拼接 bytes，再 UTF-8 解码；未完成尾巴保留 |
| A4 遥测，25 B | min/max/avg µs 在1/5/9；块数13；fill17；source18（0 UAC、1 BT）；flags19（UAC活动1、BT连接2、BT推流4）；参数编码版本20（2）；ppm float21 |
| A5 / A8 | 保存 DSP NVS / 恢复 DSP RAM 默认；不覆盖功放 NVS |
| A6 状态，35 B | CPU0/1负载1/2；CPU频率3/7；heap11、heap_total15、app_size19、flash23；版本27/28；biquads29:u16、limit31:u16；预设mask33、slots34 |
| A7 | 滤波器/系数读取；格式以源内回包为准 |
| AA/AB/AC/AD | 预设保存/载入/清除/状态，槽0..4 |
| AF HW，24 B | 版本2；mute2、pin3、playing4、极性5；延迟A6:u16/B8:u16；transitions10、since14、playing_since18。不是 DSP 错误计数 |
| B0 → B1，48 B | flags2：连接1、推流2、decoder ready4、busy8；codec_type3；rate4、channels8、fill9；duration10、level14、capacity18、min22、max26、packets30、drops34、overflows38、flushes42 |
| B5 | RAM-only DSP 引擎绕过；默认关，仍经过 core1 转换/提交。不能用于冒充活动 DSP 验收 |
| B6 | RAM-only 双边界观察与旧诊断开关；默认关。ARM ACK 仅接受，必须再确认 requested/applied/armed；当前完整布局看真源头文件，不沿用0.131长度表 |
| B8 | 独立 I²C 功放，见下文 |

B1 的 codec_type=255 不能区分不同 vendor codec，回包本身不包含 VID/CID。实际身份应使用同一配置代次的原生日志。A6 的 heap 在11，不是1。

UART/BLE 共用单控制会话，UART请求活跃时优先，竞争接口返回 BUSY。BLE FF00 服务、FF01 WRITE、FF02 NOTIFY；先订阅后写，请求按≤20 B有序分片兼容 MTU23。连接/订阅/接收溢出换代后，旧请求和未完成上传不能流入新连接。BLE 每完整 AA55 帧有500 ms发送上限，只让 proto_task 有界等待，不让 BTU/音频等待；API接受通知不等于无线送达。

## 参数表

宽度为精确字节数，小端。block：1输入、2通道DSP、3路由、4全局、5系统。端口用 idx，通道DSP用ch，路由用源ch与输出idx。

| param | 字段 | 宽度/类型 |
| --- | --- | --- |
| 01/02/03 | 输入增益/静音/反相 | 4:f32 dB / 1 / 1 |
| 10..15 | PEQ on/f/g/q/type/count | 1/4/4/4/1/1 |
| 20..23 / 30..33 | 通道 HP / LP on/f/type/slope | 各1/4/1/1 |
| 50..52 | 路由 on/gain/src | 1/4/1 |
| 60 | 主音量线性值，0为数字静音 | 4:f32 |
| 61..65 | 端口 mode/gain/delay/format/enable | 1/4/4/1/1 |
| 66..6D | 端口 HP/LP | 各1/4/1/1；UI不重复暴露旧端口滤波 |
| 70..75 | 输入房间 EQ on/f/g/q/type/count | 1/4/4/4/1/1 |
| 76/77/78 | 限幅on/threshold/release | 1/4/4 |
| 79/7A/7B | 响度on/amount/ref | 1/4/4 |
| 7C/7D/7E/7F | 压缩on/threshold/ratio×10/release | 1 / 1:int8 / 2:u16 / 2:u16 |
| 80 | 精度兼容字段，不改变冻结预算 | 1 |
| 81/84/82 | mute延迟A/B/极性 | 2:u16 / 2:u16 / 1 |
| 83/85..89 | 未实现保留 | 不产生配置字段 |

压缩比存储范围1..25.5:1，当前冻结 DSP 处理最高20:1，两者不能混称。GPIO延迟0..2000 ms，默认30 ms。PEQ增益±12 dB、Q0.4..10；图形坐标范围不是参数合法范围。

## BULK v5

头16 B：magic 0x44535043、version5:u16、preset:u16、live Fs:u32、flags:u32。

- inputs[2]：gain:f32、mute/phase/n_eq/n_req各u8，再按各计数读取16 B band。
- route[2][4]：on/src/保留2 B + gain:f32，共64 B。
- channels[4]：头8 B + n_eq×16 B + HP8 B + LP8 B。
- 尾106 B：master4 + 双端口各32 + limiter12 + loudness12 + compressor6 + 兼容保留8。
- band：f/g/q各f32，type/on/保留2 B；XO：f32频率、type/slope/on/保留1 B。

总量 `298+16×B`，B为全部输入EQ+REQ和四通道EQ的总段数。默认1578 B，满15段2218 B，buffer上限2304 B。当前C对请求0段做最小段数夹取；用实际C序列化验证的最小夹取fixture为426 B，不能把“请求0”写成“实际0段”。

从 limiter 起的偏移：on0、threshold4、release8；loud on12/amount16/ref20；comp on24/th26/ratio27:u8/release28:u16；precision30；delayA31、delayB33、保留35、极性36、存在标记37。压缩比参数线宽2 B，与 blob 内1 B不同。

首帧必须是 `4+min(total,251)`，每个续包恰为 `min(remaining,255)`。长度、版本、计数、有限浮点数和完整尾部全部校验后，UI才一次提交配置。不能夹取计数来猜游标、拼接下一条A4、应用半份配置，或把零响度提升改回默认10 dB。

JSON导入/导出属于另一接口；JSON键、设备名、单位和协议标识不翻译。左右联动只在浏览器镜像通道PEQ/HP/LP帧，不镜像源输入、路由、全局量或段数。

DSP NVS为 `dspcfg/blob`，预设 `p0..p4`；功放保存独立。硬复位后仍保留才是掉电持久化证据，单 ACK 不足。稳定BULK对比仅排除头部live Fs的[8,12)；其他改变必须解释或恢复。

## I²C 功放 B8

请求 `[B8,op,token,...]`，回复 `[B8,op|80,token,status,...]`。op：0状态，1按offset读取JSON，2BEGIN(total:u16,generation:u32)，3连续CHUNK(offset:u16,data)，4COMMIT(persist:u8)，5自定义函数(port,index3..6)，6仅清除功放保存项。

状态码：0OK、1长度、2值、3忙、4代次过期、5保存、6I²C、7内存不足。STATUS共55 B：schema4、requested generation5、applied9、busy13、last token14/result15、playing16/mute17；端口0@18、端口1@36，每项18 B含state/mute/error:i32/writes/reads/jobs；saved54。

GET回复header12 B：total@4:u16、offset@6:u16、generation@8:u32，再带≤243 B原JSON。上传不跨会话/token/代次，连续offset，10 s空闲期限。COMMIT仅接受，需等busy清零、applied及每端口状态确认实际结果。

JSON最多8192 B、深度12，严格键/重复键/整数，两个端口；默认均关闭，地址7位。操作/预算与TAS基础模板见 [AMPLIFIER_CONTROL.md](AMPLIFIER_CONTROL.md)。写失败需报告fault和mute UNKNOWN，不自动重试；没有实物不能声称已静音。NVS仅 `amp_i2c/profile_v1`。
