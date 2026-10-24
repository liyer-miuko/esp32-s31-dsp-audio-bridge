"use strict";
/* =====================================================================
 * 音频桥 DSP 控制台 — ESP32-S31（固件 0.206 / 参数编码 v2 / BULK ver=5 / 双 I2S + 0xB7 实验室）
 * 工作流：USB / 蓝牙（输入级：增益 / 静音 / 反相 / PEQ / 房间 EQ）
 *         → 路由矩阵 2×4（启用 + 源声道 + 增益）→ 通道 DSP（dsp1~dsp4：PEQ + 高通 + 低通）
 *         → 主音量 → 响度补偿（等响）→ 削波保护 → 输出端口（端点适配：声道 / 格式 /
 *           微调 / 延时；端口级 HP/LP 协议保留、界面不暴露）
 * 线协议（0.24 已实现，参数编码 v2）：
 *   帧：AA 55 | len | payload | crc8(poly 0x07，覆盖 len+payload)
 *   payload 参数消息：[msg_id, MSG_PARAM_UPDATE(0x01), block, ch, param, idx, value…]
 *   控制帧：PING(0xA0)/PONG(0xA1)/SYNC_CONFIG(0xA2)/LOG(0xA3)/TELEMETRY(0xA4 25B)/
 *           SAVE_CONFIG(0xA5)/BULK_CONFIG(0x02)/SYS_STATUS(0xA6 35B)/FILTER_DUMP(0xA7)/
 *           RESET_CONFIG(0xA8)/DSP_SELFTEST(0xA9)/预设(0xAA~0xAD)/电平表(0xAE)
 *   波特率 115200 8N1（设备是原生 USB Serial/JTAG，波特率对链路无影响，仅为 Web Serial 必填项）
 *   BULK 配置体（小端，ver=5）：magic 0x44535043 / ver u16 / preset u16 / fs u32 / flags u32
 *     inputs[2]：f32 线性增益, u8 mute, u8 phase, u8 n_eq, u8 n_req, 频段[n_eq]×16, 频段[n_req]×16
 *     频段 16B：f32 f, f32 g, f32 q, u8 type, u8 on, u8 rsv×2
 *     route[2][4]：u8 on, u8 src, u8 rsv×2, f32 增益
 *     channels[4]：u8 enabled, u8 rsv, u8 n_eq, u8 n_hp, u8 n_lp, u8 rsv×3, 频段[n_eq]×16, hp 8B, lp 8B
 *     分频 8B：f32 f, u8 type, u8 slope, u8 on, u8 rsv
 *     master f32, ports[2]：u8 mode, u8 format, u8 enabled, u8 rsv, f32 gain_db, f32 delay_ms,
 *       u32 pad, hp 8B, lp 8B（端口记录 32B）
 *     限幅器 12B（on/rsv×3/thresh_db/release_ms）+ 预留 20B
 *       —— 预留区前 12B 已启用为响度补偿：u8 on, u8 rsv×3, f32 amount_db, f32 ref
 *       —— 预留区接下来的 6B（v23 起）为压缩器：u8 on, u8 rsv, i8 thresh_db,
 *          u8 ratio×10(10..255), u16 release_ms(小端)；再余 8B 未用
 * ===================================================================== */
const H0=0xAA,H1=0x55;
const MSG_PARAM_UPDATE=0x01,MSG_BULK_CONFIG=0x02,MSG_ACK=0x80,MSG_STATUS=0x81,MSG_ERROR=0x82;
const MSG_PING=0xA0,MSG_PONG=0xA1,MSG_SYNC_CONFIG=0xA2,MSG_LOG=0xA3,MSG_TELEMETRY=0xA4,MSG_SAVE_CONFIG=0xA5,MSG_SYS_STATUS=0xA6,MSG_FILTER_DUMP=0xA7,MSG_RESET_CONFIG=0xA8;
const MSG_PRESET_SAVE=0xAA,MSG_PRESET_LOAD=0xAB,MSG_PRESET_CLEAR=0xAC,MSG_PRESET_INFO=0xAD;
const BLOCK_INPUT=0x01,BLOCK_OUTPUT=0x02,BLOCK_ROUTING=0x03,BLOCK_GLOBAL=0x04,BLOCK_SYSTEM=0x05;
const P_GAIN=0x01,P_MUTE=0x02,P_PHASE=0x03;
const P_EQ_ON=0x10,P_EQ_FREQ=0x11,P_EQ_GAIN=0x12,P_EQ_Q=0x13,P_EQ_TYPE=0x14,P_EQ_COUNT=0x15;
const P_HP_ON=0x20,P_HP_FREQ=0x21,P_HP_TYPE=0x22,P_HP_SLOPE=0x23;
const P_LP_ON=0x30,P_LP_FREQ=0x31,P_LP_TYPE=0x32,P_LP_SLOPE=0x33;
const P_ROUTE_ON=0x50,P_ROUTE_GAIN=0x51,P_ROUTE_SRC=0x52;
const P_MASTER_VOL=0x60;
const P_PORT_MODE=0x61;
const P_PORT_GAIN=0x62;
const P_PORT_DELAY=0x63;
const P_PORT_FORMAT=0x64;
const P_PORT_ENABLE=0x65;
const P_PORT_HP_ON=0x66,P_PORT_HP_FREQ=0x67,P_PORT_HP_TYPE=0x68,P_PORT_HP_SLOPE=0x69;
const P_PORT_LP_ON=0x6A,P_PORT_LP_FREQ=0x6B,P_PORT_LP_TYPE=0x6C,P_PORT_LP_SLOPE=0x6D;
const P_REQ_ON=0x70,P_REQ_FREQ=0x71,P_REQ_GAIN=0x72,P_REQ_Q=0x73,P_REQ_TYPE=0x74,P_REQ_COUNT=0x75;
const P_LIMIT_ON=0x76,P_LIMIT_THRESH=0x77,P_LIMIT_RELEASE=0x78;
/* 响度补偿（等响）：0x79 开关 / 0x7A 最大补偿 dB / 0x7B 参考音量 */
const P_LOUD_ON=0x79,P_LOUD_AMOUNT=0x7A,P_LOUD_REF=0x7B;
/* 压缩器（动态范围 / 夜间模式）：端口增益之后、削波保护之前。
 * 补偿增益由固件自动按 |阈值|·(1-1/压缩比) 推出（0 dBFS 进 → 0 dBFS 出，不会削顶），所以没有"补偿增益"参数。 */
const P_COMP_ON=0x7C,P_COMP_TH=0x7D,P_COMP_RATIO=0x7E,P_COMP_REL=0x7F;
/* ★ v36（阶段 B 接口层）：硬件控制层参数 id（规格 v9 §4.1）
 *   0x81 u16 延时 A / 0x82 u8 极性 / 0x84 u16 延时 B
 *   ⚠ **两个 u16 下发时必须带 nbytes=2**（`setParam(...,false,2)`）——
 *     只发 1 字节会被固件判 `status=4 ERR_LENGTH` 并**静默保留原值**（`0x7E` 踩过同一个坑）。
 *   ⛔ 0.112 起 `0x83`（编码器启用）与 `0x85~0x89`（键位码）随实体编码器 / HID 功能移除：
 *     固件端**不再有分支**、写入返回 `status=8 ERR_PARAM` ⇒ 前端**不出面板、也不下发**。 */
const P_HW_DELAY_A=0x81,P_HW_POLARITY=0x82,P_HW_DELAY_B=0x84;
const HW_DELAY_MAX=2000,HW_DELAY_DEFAULT=30;
/* ★ DSP 精度模式（固件 0.27 / 协议 0x80，u8 1 字节：0=高精度 / 1=扩展）。
 * ★ v32（固件 0.29）：两种模式的段数上限已同值（v32 当时为 104，历史数字），模式不再影响预算
 *   （压缩器 847→170 µs 之后，"扩展模式禁止压缩器"那条硬禁已按用户决定取消）。
 *   参数 id / 默认值 / blob 布局一律未变 —— 模式字节仍照旧读写与上报，只是不再改变上限。
 * 它历史上存在的理由是"把取舍显式化"：更早的固件上限会随效果开关浮动（82/72/51/41），
 * 用户配好满配滤波后一开压缩器就被整包回滚，且界面上完全看不出原因。 */
const P_PRECISION_MODE=0x80;
const PRECISION_HIGH=0,PRECISION_EXTENDED=1;
const PRECISION_NAME=["High precision","Extended"];
/* 0.122 HWLOOP两档工程估计：mono13us/pair19us，固定开销保持567+51+(comp?155:0)。
 * 130段最坏2214us，三开110段最坏2249us；整链预算仍2250us。
 * 未连接按此候选估算；旧设备先按其版本和SYS_STATUS限制，不把新上限套给旧固件。
 * 精度模式不改变预算；当前最大布局尚未实机验收。 */
const LIMIT_COMP=130, LIMIT_NOCOMP=130;   /* 0.123: 两档统一到物理界面最大计数 */
const PROTO_VERSION=2;                         /* 本前端按参数编码 v2 发送（0x7E 压缩比 = u16） */
const COMP_RATIO_MIN=10,COMP_RATIO_MAX=255;    /* 值 = 压缩比×10 → 1.0..25.5（参数编码 v2：0x7E 为 u16） */
const COMP_REL_MIN=10,COMP_REL_MAX=2000;       /* ms */
/* 「夜间模式」一键预设（与固件默认值一致：阈值 -30 / 4.0:1 / 200 ms） */
const NIGHT_PRESET={th:-30,ratio:40,rel:200};
/* 自动补偿增益：makeup = |th|·(1-1/R)，用于界面显示 */
function compMakeup(th,ratioX10){
  const R=Math.max(1,Math.min(20,(ratioX10||10)/10));
  return Math.abs(th)*(1-1/R);
}
/* 与固件一致的隐藏架式滤波器参数（dsp_engine.c 里的 DSP_LOUD_*） */
const LOUD_LO_HZ=100,LOUD_HI_HZ=10000,LOUD_Q=0.707;
const PRESET_SLOTS=5;
const F_PEAK=0,F_LSHELF=1,F_HSHELF=2,F_LP=3,F_HP=4,F_BP=5,F_NOTCH=6,F_AP=7;
const FNAME=["Peaking","Low shelf","High shelf","Low-pass","High-pass","Band-pass","Notch","All-pass"];
const XO_BW=0,XO_LR=1;
const SLOPE12=1,SLOPE24=2,SLOPE48=4;
const PORT_FMT_NAME=["I2S (Philips)","Left-justified (MSB)","Right-justified"];
const PORT_MODE_NAME=["Stereo","Left channel only","Right channel only"];
/* ⚠ 文案口径（v25 澄清，别改回去）：这条**同时**描述了「输入侧取哪一路」与「输出侧送去哪个槽」，
 * 因为只写输入侧（"dsp1/dsp3←L、dsp2/dsp4←R"）会被读成"输出侧也交叉"，与面板上的
 * "dsp1-端口i2s0-左"看着矛盾。实际链路（固件 dsp_engine.c:315-316 与 :762-772）：
 *     AUTO ⇒ dsp1/dsp3 ← 源 L、dsp2/dsp4 ← 源 R（输入侧）
 *     端口 p 用 CH(2p+1)=左槽、CH(2p+2)=右槽 ⇒ dsp1→i2s0 左槽、dsp2→i2s0 右槽（输出侧）
 * 合起来 = 左→dsp1→i2s0左、右→dsp2→i2s0右，**对角直通，没有任何交叉**
 * （"交叉增益"= 把对角换成反对角，即把某格显式选成 L/R，属补救用途）。 */
const ROUTE_SRC_NAME=[
  "Auto: left to odd DSP channels; right to even DSP channels",
  "Left source channel",
  "Right source channel",
  "Mono (L+R)/2"];
const PORT_FORMAT_NAME=["I2S","TDM"];
const MAX_BAND=15,DEF_BAND=10;
const PANEL_NAME={in:"Inputs",route:"Routing",dsp:"DSP",ports:"Ports"};
/* DSP 通道命名（全界面统一）：长名用于标题/表头，短名用于图例 */
const DSP_LONG=["DSP1 / I2S0 / left","DSP2 / I2S0 / right","DSP3 / I2S1 / left","DSP4 / I2S1 / right"];
const DSP_SHORT=["dsp1","dsp2","dsp3","dsp4"];
const DSP_MINI=["DSP1 / I2S0 / left","DSP2 / I2S0 / right","DSP3 / I2S1 / left","DSP4 / I2S1 / right"];
/* 源名称（与固件 TELEMETRY 的源字节、cfg.inputs 的索引一致：0=USB / 1=蓝牙） */
const SRC_NAMES=["USB","Bluetooth"];
/* DSP 通道可用性：dsp1/dsp2 = I2S0 左右；dsp3/dsp4 = I2S1 左右。
 * 双 I2S 已启用，DSP_DISABLED 当前全 0；保留该表给未来不可用通道。 */
const DSP_DISABLED=[0,0,0,0];
function dspDisabled(ch){return !!DSP_DISABLED[ch]}
const PORTS=[
  {id:"i2s0", name:"I2S0 output", color:"var(--c-p0)", fmt:false, i2s:true,
   desc:"I2S0: MCLK GPIO2, BCLK GPIO3, WS GPIO4, DATA GPIO7. The clock follows the selected source. No ASRC.", disabled:false},
  {id:"i2s1", name:"I2S1 output", color:"var(--c-p1)", fmt:false, i2s:true,
   desc:"I2S1: MCLK GPIO13, BCLK GPIO14, WS GPIO15, DATA GPIO16. Both ports share APLL. Frame phase alignment is not guaranteed.", disabled:false},
];
/* ================= 主题（深色 / 浅色） ================= */
const THEME_KEY="esp32s31.dspui.theme";
let gTheme=document.documentElement.dataset.theme||"dark";
function setTheme(t){
  gTheme=t;document.documentElement.dataset.theme=t;
  try{localStorage.setItem(THEME_KEY,t)}catch(e){}
  const b=$("btnTheme");if(b)b.textContent=t==="light"?"☾":"☀";
  const mc=document.querySelector('meta[name="theme-color"]');
  if(mc)mc.setAttribute("content",t==="light"?"#eceff7":"#05070d");
  try{refreshSliderGeom()}catch(e){}      /* 主题切换会改变实测样式，重新量一次滑块几何 */
  drawResp();
}
/* 图形和节点共用 CSS 颜色变量，切换主题时保持对应关系。 */
function cssv(n,fb){try{const v=getComputedStyle(document.documentElement).getPropertyValue(n).trim();return v||fb}catch(e){return fb}}
function pal(){return{inp:[cssv("--usb","#7fb9c4"),cssv("--bt","#a7d48b")],
  out:[cssv("--ch1","#e3bb72"),cssv("--ch2","#b7a6dc"),cssv("--ch3","#6cc7b3"),cssv("--ch4","#e59aa6")],
  port:[cssv("--c-p0","#e6967e"),cssv("--c-p1","#82bfd0")]}}
/* ================= 本地配置 ================= */
const EQ_FREQS=[31.5,63,125,250,500,1000,2000,4000,8000,16000];
function newBand(idx){return{on:0,type:F_PEAK,f:(EQ_FREQS[idx]||1000),g:0,q:1}}
function newXo(){return{on:0,f:3000,type:XO_LR,slope:SLOPE24}}
function newPort(i){return{mode:0,format:0,gain:0,delay:0,enabled:1,hp:newXo(),lp:newXo()}}
function newRoute(){return{on:1,gain:1,src:0}}
function newLim(){return{on:0,thresh:-1,release:100}}
/* 响度补偿默认：关闭（保持出厂旁通）、最大 10 dB、参考音量 80% —— 与固件默认值一致 */
function newLoud(){return{on:0,amount:10,ref:0.8}}
/* 压缩器默认：关闭（保持出厂旁通）、阈值 -30 dBFS、4.0:1、释放 200 ms —— 与固件默认值一致 */
function newComp(){return{on:0,th:-30,ratio:40,rel:200}}
/* ★ v36（阶段 B 接口层）：硬件控制层的三项配置 —— 与固件 `dsp_config.c` 的出厂默认逐项一致：
 *   延时 A/B = 30 ms、低有效静音（pol=0）。
 * ⚠ 本轮**只做"配置 + 下发"**：mute 输出的**行为**在阶段 C1 实现
 *   （所以这里绝不出现"步进/挡位/手感"这类控件，也不碰端口区）。
 * ⛔ 0.112：原第四项"编码器启用"（`enc` / `0x83`）随实体编码器 / HID 功能一并删除 ——
 *   这里**不能**留成恒 0 的占位字段：那等于把已废弃的参数继续往固件发（固件会回 status=8）。 */
function newHw(){return{delayA:30,delayB:30,pol:0}}
function newIn(){const x={gain:0,mute:0,phase:0,eq:[],req:[]};
  for(let i=0;i<DEF_BAND;i++){x.eq.push(newBand(i));x.req.push(newBand(i))}return x}
function newDspOut(){const x={eq:[]};
  for(let i=0;i<DEF_BAND;i++)x.eq.push(newBand(i));
  x.hp=newXo();x.lp=newXo();return x}
function defaultCfg(){
  const route=[ [ newRoute(),newRoute(),newRoute(),newRoute() ],
                [ newRoute(),newRoute(),newRoute(),newRoute() ] ];
  const outs=[newDspOut(),newDspOut(),newDspOut(),newDspOut()];
  /* link[0]/link[1]：端口0（dsp1↔dsp2）/ 端口1（dsp3↔dsp4）的左右联动开关，默认都开。 */
  return{inputs:[newIn(),newIn()],outs,route,master:1,link:[1,1],
         ports:[newPort(0),newPort(1)],lim:newLim(),loud:newLoud(),comp:newComp(),hw:newHw(),
         /* ★ v28：出厂默认 = 扩展，与固件 `DSP_PRECISION_DEFAULT` 一致。
          * 用户在界面上看不到"模式"这个东西。
          * ★ v32：两种模式的上限已同值（v32 当时 104；**现为候选关130 / 三开110（旧设备以回读为准）**），模式字节保留纯粹为了协议/blob 兼容。 */
         precision:PRECISION_EXTENDED};
}
const LS_KEY="esp32s31.dspcfg.v3";
function saveLocal(){try{localStorage.setItem(LS_KEY,JSON.stringify(cfg,(k,v)=>k==="_ui"?undefined:v))}catch(e){}}
function loadLocal(){
  try{
    const s=localStorage.getItem(LS_KEY);
    if(!s)return;
    const c=JSON.parse(s);
    if(!c||!Array.isArray(c.inputs)||!Array.isArray(c.outs))return;
    cfg.inputs=c.inputs.slice(0,2);
      cfg.outs=c.outs.slice(0,4);
      cfg.route=Array.isArray(c.route)?c.route.slice(0,2).map(r=>Array.isArray(r)?r.slice(0,4):[]):cfg.route;
      while(cfg.inputs.length<2)cfg.inputs.push(newIn());
      while(cfg.outs.length<4)cfg.outs.push(newDspOut());
      while(cfg.route.length<2)cfg.route.push([{on:1,gain:1},{on:1,gain:1}]);
      cfg.route.forEach(r=>{while(r.length<4)r.push({on:0,gain:1,src:0});});
      cfg.route.forEach(r=>r.forEach(c=>{c.src=cl(c.src===undefined?0:c.src,0,3)}));
    if(typeof c.master==="number")cfg.master=c.master;
    cfg.link=normLink(c.link);      /* 兼容老格式（单个 0/1）→ [端口0, 端口1] */
    if(c.lim&&typeof c.lim==="object")cfg.lim={on:c.lim.on?1:0,
      thresh:Math.max(-24,Math.min(0,+c.lim.thresh||-1)),
      release:Math.max(10,Math.min(1000,+c.lim.release||100))};
    if(c.loud&&typeof c.loud==="object")cfg.loud={on:c.loud.on?1:0,
      amount:Math.max(0,Math.min(15,c.loud.amount===undefined?10:+c.loud.amount)),
      ref:Math.max(0.05,Math.min(1,c.loud.ref===undefined?0.8:+c.loud.ref))};
    if(Array.isArray(c.ports))cfg.ports=c.ports.slice(0,2);
    cfg.ports=cfg.ports.map((p,i)=>({mode:p.mode||0,format:p.format||0,gain:p.gain||0,delay:p.delay||0,
      enabled:(p.enabled===undefined?0:(!!p.enabled)),hp:fixXo(p.hp),lp:fixXo(p.lp)}));
    while(cfg.ports.length<2){cfg.ports.push(newPort(cfg.ports.length));}
    /* 双 I2S 已启用：不再强制关闭端口 1；旧本地配置保留用户选择。 */
    /* ★ 缺陷修复（v43，C42）：这里**原先根本没有合并 `hw`** ⇒ 刷新页面后
     *   "硬件控制"页（停止/播放延时、电平反转）静默退回出厂 30/30/0 ——
     *   用户在那一页设的值**从来没被持久化过**（`saveLocal` 存了、这里没读回来）。
     *   缺 `hw`（老 localStorage / 老导出文件）⇒ 回落 `newHw()`，且**不报错**。
     * ⚠ 这里**不能**写成 `if(c.hw){...}`：那样老存储会原样保留内存里那个可能被改坏的值，
     *   "回落"根本没发生（本用例的负向对照就是拿这个抓出来的）。 */
    const hwL=(c.hw&&typeof c.hw==="object")?c.hw:{};
    cfg.hw=Object.assign(newHw(),{
      delayA:cl(hwL.delayA===undefined?HW_DELAY_DEFAULT:hwL.delayA,0,HW_DELAY_MAX),
      delayB:cl(hwL.delayB===undefined?HW_DELAY_DEFAULT:hwL.delayB,0,HW_DELAY_MAX),
      pol:hwL.pol?1:0});   /* ⛔ 0.112：老存储里的 `enc` 直接丢弃，不再读也不再下发 */
    const clampQ=b=>{if(b&&typeof b.q==="number")b.q=Math.max(0.4,Math.min(10,b.q))};
    cfg.inputs.forEach(inp=>{inp.eq.forEach(clampQ);inp.req.forEach(clampQ)});
    cfg.outs.forEach(o=>o.eq.forEach(clampQ));
  }catch(e){}
}
let cfg=defaultCfg();loadLocal();
let gFs=48000;
let gFwVer="--",gFiltCount=0,gFiltLimit=0,gPresetMask=0,gProtoVer=0;
/* ================= 串口 ================= */
let port=null,reader=null,writer=null,msgId=0,rxBuf=new Uint8Array(0);
let bulkBuf=null,bulkGot=0,bulkTimer=null;
const $=id=>document.getElementById(id);
function log(t,cls){const b=$("logBox");const line=document.createElement("div");
  line.className=cls||"";if(cls==="device-raw")line.dataset.noTranslate="";line.textContent=t;b.appendChild(line);
  while(b.children.length>300)b.removeChild(b.firstChild);b.scrollTop=b.scrollHeight}
function crc8(d){let c=0;for(const b of d){c^=b;for(let i=0;i<8;i++)c=(c&0x80)?((c<<1)^0x07)&0xff:(c<<1)&0xff}return c}
function frame(payload){const f=new Uint8Array(payload.length+4);f[0]=H0;f[1]=H1;f[2]=payload.length;
  f.set(payload,3);f[f.length-1]=crc8(f.slice(2,f.length-1));return f}
/* One ordered frame queue for both transports. Never overlap GATT writes
 * or let an old connection's queued commands run on the next connection. */
let controlGeneration=0, txChain=Promise.resolve(), txPending=0;
function transportActive(){return !!writer||bleActive()}
function resetTransport(){
  if(window.ampUI)window.ampUI.cancel();
  controlGeneration++; rxBuf=new Uint8Array(0);bulkBuf=null;bulkGot=0;
  if(bulkTimer){clearTimeout(bulkTimer);bulkTimer=null}
}
function sendFrame(p){
  const f=frame(p), w=writer, ch=w?null:bleRxCh, gen=controlGeneration;
  if(!w&&!bleActive()){
    if(!window.__noTransportWarned){window.__noTransportWarned=1;
      log("No control connection. Changes stay in this browser.","warn")}
    return false;
  }
  if(txPending>=2048){log("The transmit queue is full. Read the device configuration before you try again.","err");resetTransport();return false}
  txPending++;
  txChain=txChain.then(async()=>{
    if(gen!==controlGeneration)return;
    if(w){if(writer!==w)return;await w.write(f);return}
    if(!bleActive()||bleRxCh!==ch||writer)return;
    /* Web Bluetooth has no portable negotiated-MTU accessor. 20 bytes fit
     * ATT MTU 23; writes with response expose firmware queue/lease failures. */
    for(let off=0;off<f.length;off+=20){
      if(gen!==controlGeneration||bleRxCh!==ch||writer)return;
      const part=f.slice(off,off+20);
      if(ch.writeValueWithResponse)await ch.writeValueWithResponse(part);
      else await ch.writeValue(part);
    }
  }).catch(e=>{
    if(gen===controlGeneration){log("Transmission stopped: "+e.message,"err");resetTransport()}
  }).finally(()=>{txPending--});
  return true; /* queued, not an ACK that the device applied the value */
}
function f32(v){const b=new Uint8Array(4);new DataView(b.buffer).setFloat32(0,v,true);return b}
/* `nbytes`：**整型参数的字节宽度**，默认 1（向后兼容既有调用）。
 * ⚠ 参数编码 v2 起 `0x7E` 压缩比是 **u16**，必须显式传 `nbytes=2`；
 *   仍按 1 字节发会被固件判 `status=4`（长度错误）并**静默保留原值** ——
 *   表现为"滑块拖了设备没反应"。字节宽度一律以协议表（HANDOFF §3.3）为准。 */
function setParam(block,ch,param,idx,val,isFloat,nbytes){
  let id=(msgId++)&0xff;
  if(id===0xB8)id=(msgId++)&0xff; // Reserved for amplifier response envelopes.
  const p=[id,MSG_PARAM_UPDATE,block,ch,param,idx];
  if(isFloat){p.push(...f32(val));}
  else if((nbytes|0)>1){                       /* 小端多字节整型 */
    let v=(val|0)&0xffffffff;
    for(let i=0;i<nbytes;i++){p.push(v&0xff);v>>>=8;}
  }
  else {p.push(val&0xff);}
  sendFrame(p);
  if(typeof mirrorParam==="function")mirrorParam(block,ch,param,idx,val,isFloat,nbytes);   /* 左右联动 */
  if(typeof recordHistorySoon==="function")recordHistorySoon();                     /* 撤销/重做：批量记录 */
}
function sendPing(){sendFrame(new Uint8Array([MSG_PING]))}
/* ================= DSP 通道左右联动 =================
 * cfg.link = [端口0, 端口1]，每一位控制一对通道的联动：
 *   端口0：dsp1（ch0）↔ dsp2（ch1）
 *   端口1：dsp3（ch2）↔ dsp4（ch3）   ← I2S1 放开后就用这一对
 * 开启后：在任意一侧调整，另一侧 UI 立即跟随（输入框/开关/下拉都刷新），
 *         并且**镜像帧同步下发到设备**（后端立即生效）。
 * 镜像范围：通道 PEQ 的 开关/类型/频点/增益/Q + 高通/低通 的 开关/类型/斜率/频点。
 * 输入级 PEQ / 房间 EQ（BLOCK_INPUT）与其它 block 一概不联动。
 * 注意：只镜像参数帧（idx 是频段号，左右一致），不动 active band 计数（pushAll 按需补）。 */
const MIRROR_PARAMS=new Set([P_EQ_ON,P_EQ_FREQ,P_EQ_GAIN,P_EQ_Q,P_EQ_TYPE,
                            P_HP_ON,P_HP_FREQ,P_HP_TYPE,P_HP_SLOPE,
                            P_LP_ON,P_LP_FREQ,P_LP_TYPE,P_LP_SLOPE]);
function pairOf(ch){return ch>>1}                       /* ch0/1 → 端口0；ch2/3 → 端口1 */
function linkTarget(ch){return ch^1}                    /* 同端口内左右互换 */
function linkOn(ch){return !!(cfg.link&&cfg.link[pairOf(ch)])}
function linkLabel(p){return p===0?"Link DSP1 and DSP2":"Link DSP3 and DSP4"}
let syncing=false;
function mirrorParam(block,ch,param,idx,val,isFloat,nbytes){
  if(syncing)return;                          /* 防止递归 */
  if(block!==BLOCK_OUTPUT)return;
  if(!MIRROR_PARAMS.has(param))return;
  if(!(ch>=0&&ch<4))return;
  if(!linkOn(ch))return;                      /* 这一对没开联动就各自独立 */
  const t=linkTarget(ch);
  const dst=cfg.outs&&cfg.outs[t], src=cfg.outs&&cfg.outs[ch];
  if(!dst||!src)return;
  syncing=true;
  try{
    if(param>=P_HP_ON&&param<=P_HP_SLOPE){
      const k=param===P_HP_ON?"on":param===P_HP_TYPE?"type":param===P_HP_SLOPE?"slope":"f";
      src.hp[k]=val; dst.hp[k]=val;
    }else if(param>=P_LP_ON&&param<=P_LP_SLOPE){
      const k=param===P_LP_ON?"on":param===P_LP_TYPE?"type":param===P_LP_SLOPE?"slope":"f";
      src.lp[k]=val; dst.lp[k]=val;
    }else if(idx>=0&&src.eq[idx]&&dst.eq[idx]){
      const k=param===P_EQ_ON?"on":param===P_EQ_TYPE?"type":param===P_EQ_FREQ?"f":
              param===P_EQ_GAIN?"g":"q";
      src.eq[idx][k]=val; dst.eq[idx][k]=val;
    }
    setParam(block,t,param,idx,val,isFloat);  /* 直接下发镜像帧 → 后端同步生效 */
    saveLocal();
    refreshChannelCard(t);                    /* 对侧 UI 立即跟随 */
  }finally{ syncing=false }
}
/* 把某条通道卡的控件刷成 cfg 里的值（镜像后调用；正在编辑的输入框不抢焦点） */
function refreshChannelCard(ch){
  const out=cfg.outs&&cfg.outs[ch];
  if(!out)return;
  (out.eq||[]).forEach(b=>{if(b&&typeof b._refresh==="function")b._refresh()});
  (out._uiRefresh||[]).forEach(fn=>{try{fn()}catch(e){}});
}
function buildLinkUI(){
  const wrap=$("linkTogWrap");if(!wrap)return;
  wrap.innerHTML="";
  if(!Array.isArray(cfg.link))cfg.link=normLink(cfg.link);
  [0,1].forEach(p=>{
    const row=document.createElement("div");row.className="rbar tight";
    const lab=document.createElement("label");lab.textContent=linkLabel(p);
    row.appendChild(lab);
    const t=tog(cfg.link[p],v=>{
      cfg.link[p]=v;
      saveLocal();
      log((v?"Link on: ":"Link off: ")+linkLabel(p)+" channel pair","info");
      drawResp();
    });
    row.appendChild(t);
    const tip=document.createElement("span");tip.className="tip";
    tip.textContent=p===0?"I2S0: changes apply to both linked channels."
                         :"I2S1: changes apply to both linked channels.";
    row.appendChild(tip);
    wrap.appendChild(row);
  });
}
function sendSync(){bulkBuf=null;bulkGot=0;if(bulkTimer){clearTimeout(bulkTimer);bulkTimer=null}sendFrame(new Uint8Array([MSG_SYNC_CONFIG]));log("Read the device configuration.","info")}
function sendSave(){sendFrame(new Uint8Array([MSG_SAVE_CONFIG]));log("Save requested. Check the device reply.","info")}
function sendReset(){sendFrame(new Uint8Array([MSG_RESET_CONFIG]));log("Default configuration requested. Check the device reply.","info")}

/* Codec availability is a build property. No unsupported B7 writes. */
function requestAmpSettings(){if(window.ampUI)window.ampUI.read()}
let monTimer=null,monInterval=1000;
function sendSysReq(){sendFrame(new Uint8Array([MSG_SYS_STATUS]))}
function startMon(){stopMon();sendSysReq();monTimer=setInterval(sendSysReq,monInterval)}
function stopMon(){if(monTimer){clearInterval(monTimer);monTimer=null}}
function resetMon(){
  ["monCpu0v","monCpu1v","monFreq0","monFreq1","monHeap","monFlash"].forEach(id=>$(id).textContent="--");
  $("monCpu0").style.width="0%";$("monCpu1").style.width="0%";
}
/* 主音量显示：0% 是真的静音（固件把 master=0 映射成 0 增益） */
function masterLabel(v){return v<=0?"Mute (0%)":Math.round(v*100)+"%"}
function fmtBytes(b){if(b>=1048576)return (b/1048576).toFixed(1)+" MB";if(b>=1024)return (b/1024).toFixed(0)+" KB";return b+" B"}
/* ================= 信号流图导航 ================= */
function buildFlow(){
  const v=$("flowV"),st=$("flowStatus");
  if(!v)return;
  if(v._navBound)return;
  v._navBound=true;
  const nav=e=>{
    const g=e.target&&e.target.closest?e.target.closest(".fv-node"):null;
    if(!g)return;
    if(!v.contains(g))return;
    e.preventDefault();
    const jp=g.getAttribute("data-jump")||"in";
    showPanel(jp);
    g.classList.add("flash");
    setTimeout(()=>g.classList.remove("flash"),1400);
    const tgt=g.getAttribute("data-target");
    if(tgt){
      try{
        const el=document.querySelector(tgt);
        if(el){
          setTimeout(()=>{
            try{el.scrollIntoView({behavior:"smooth",block:"nearest"})}catch(_){}
            el.classList.add("flash");
            setTimeout(()=>el.classList.remove("flash"),1600);
          },60);
        }
      }catch(_){}
    }
    if(st){
      st.textContent="Panel: "+(g.textContent||"").replace(/\s+/g," ").trim()+" → "+(PANEL_NAME[jp]||"Inputs");
      st.style.color="var(--acc)";
    }
  };
  v.addEventListener("click",nav);
  document.addEventListener("click",nav,true);
  if(st){st.textContent="Select a node to open its panel.";st.style.color="var(--dim)"}
}
/* ================= 操作台切换 ================= */
let curPanel="in";
function showPanel(name,animate){
  if(!Object.hasOwn(PANEL_NAME,name))name="in";
  curPanel=name;
  const el=$("tp-"+name);
  if(el&&!el.classList.contains("on")){
    document.querySelectorAll(".tp").forEach(p=>p.classList.remove("on"));
    el.classList.add("on");
    if(animate!==false){el.classList.add("flash");setTimeout(()=>el.classList.remove("flash"),1500)}
  }
  document.querySelectorAll("#flowV .fv-node").forEach(g=>g.classList.remove("sel"));
  document.querySelectorAll('#flowV .fv-node[data-jump="'+name+'"]').forEach(g=>g.classList.add("sel"));
  document.querySelectorAll(".tab").forEach(t=>{t.classList.toggle("on",t.dataset.panel===name);t.setAttribute("aria-selected",String(t.dataset.panel===name));});
  const idx=["in","route","dsp","ports"].indexOf(name),ind=$("tabInd");
  if(ind&&idx>=0)ind.style.transform="translateX("+(idx*100)+"%)";
}
/* ================= 接收解析 ================= */
function handlePayload(p){
  /* onRx supplies the complete AA55 payload. Direct replies use p[0];
   * ACK/ERROR use [request_id, 0x80/0x82, ...]. BULK continuations are raw. */
  if(bulkBuf){handleBulk(p);return;}
  if(window.ampUI && window.ampUI.receive(p))return;
  if(p.length>=4&&p[0]===0&&p[1]===MSG_BULK_CONFIG){handleBulk(p);return;}
  const t=p[0];
  if(t===MSG_TELEMETRY&&p.length===25){
    window.__telDbg={t:t,len:p.length,head:Array.from(p.slice(0,20))};
    const dv=new DataView(p.buffer,p.byteOffset+1,24);
    const mn=dv.getUint32(0,true),mx=dv.getUint32(4,true),avg=dv.getUint32(8,true);
    const blocks=dv.getUint32(12,true),fill=dv.getUint8(16),ppm=dv.getFloat32(20,true);
    $("telTime").textContent=(avg/1000).toFixed(2)+" ms（min "+(mn/1000).toFixed(2)+" / max "+(mx/1000).toFixed(2)+"）";
    $("telBlocks").textContent=blocks.toLocaleString()+" blocks";
    $("telFill").textContent=fill+"%";
    $("telPpm").textContent=(ppm>=0?"+":"")+ppm.toFixed(0)+" ppm";
    /* [18..20] 原是保留字节（旧固件恒 0）—— v21 固件起：p[18]=当前输出源(0=USB/1=蓝牙)，
     * p[19]=状态位(bit0 放音中 / bit1 蓝牙已连接 / bit2 蓝牙推流中)，
     * p[20]=**参数编码版本**（v24 固件起 = v2；旧固件恒 0 = "没有这个字段"）。
     * 顶栏「当前源」就靠前两个字节点亮（旧固件读到 0 → 显示 USB，是合理的默认）。 */
    const src=dv.getUint8(17),fl=dv.getUint8(18);
    /* 参数编码版本自检：本前端按 **v2** 编码发送（0x7E 压缩比 = u16）。
     * 若设备报 v1/0，说明固件偏旧 → 压缩比按 2 字节发会被判 status=4，
     * 所以**主动提示**，而不是让用户对着"拖了没反应"猜。 */
    const pv=dv.getUint8(19);
    if(pv!==gProtoVer){
      gProtoVer=pv;
      const sv=$("sysVer");
      if(sv)sv.title="Firmware "+gFwVer+" / parameter format v"+(pv||"1 (old)");
      if(pv<PROTO_VERSION){
        /* ★ 第 1 步文案：**不写死固件版本号** —— 老版本号（原文"请烧录 0.24+"）
         * 早就过期了（设备自报已是 0.4x），会把人指去升一个更旧的版本。
         * 改口径：把**设备自报的版本**说出来（`gFwVer` 就是它），只判"过旧"，不承诺具体版本。 */
        log("Device parameter format v"+(pv||"1 (old)")+" is below the required version v"+PROTO_VERSION
           +". Parameter widths can differ. Some changes cannot be applied."
           +"Use a compatible frontend or update the firmware. Device format: "+(gFwVer&&gFwVer!=="--"?gFwVer:"unknown")+".","warn");
      }
    }
    const el=$("stSrc");
    if(el){
      const name=SRC_NAMES[src]||SRC_NAMES[0];
      el.textContent=name;
      const bits=[(fl&0x01)?"PCM active":"Idle"];
      if(fl&0x02)bits.push("Bluetooth connected");
      if(fl&0x04)bits.push("Bluetooth streaming");
      el.title=name+" · "+bits.join(" · ");
    }
    return;
  }
  if(p[0]===MSG_LOG&&p.length>1){
    let s="";for(let i=1;i<p.length;i++)s+=String.fromCharCode(p[i]);
    log("Device: "+s.trim(),"device-raw");return;
  }
  if(p[0]===MSG_PONG){log("Device online. Protocol reply received.","info");return;}
  if(p[0]===MSG_PRESET_INFO&&p.length>=3){
    gPresetMask=p[2];updPresetUI();
    const used=[];for(let i=0;i<PRESET_SLOTS;i++)if(gPresetMask&(1<<i))used.push(i+1);
    log("Preset slots: "+p[1]+". Occupied: "+(used.length?used.join(", "):"none"),"info");return;}
  if(p[0]===MSG_SYS_STATUS&&p.length>=27){
    /* ⚠ v25 修：DataView 长度原为 **26**，而下面 :1197 起要读 `getUint8(26..32)` —— **必然越界**
     * （每收一个 SYS_STATUS 就抛一次 RangeError，被 onRx 的 try/catch 吞掉，只在控制台可见）。
     * 真机端到端测试台把它抓了出来。SYS_STATUS 载荷 = 1 + 34（26 B 实时量 + 8 B 版本/负载/掩码），
     * 门槛 >=27（1+26）本身是对的，只需把 DataView 开到 **整个载荷**，扩展字段再按长度门保护。 */
    const dv=new DataView(p.buffer,p.byteOffset+1,p.length-1);
    const c0=dv.getUint8(0),c1=dv.getUint8(1);
    const f0=dv.getUint32(2,true),f1=dv.getUint32(6,true);
    const hf=dv.getUint32(10,true),ht=dv.getUint32(14,true);
    const fu=dv.getUint32(18,true),ft=dv.getUint32(22,true);
    $("monCpu0v").textContent=c0+"%";$("monCpu0").style.width=c0+"%";
    $("monCpu1v").textContent=c1+"%";$("monCpu1").style.width=c1+"%";
    $("monFreq0").textContent=(f0/1e6).toFixed(1)+" MHz";
    $("monFreq1").textContent=(f1/1e6).toFixed(1)+" MHz";
    /* 固件两个字段都取 esp_clk_cpu_freq()（S31 两核共用同一时钟源，DFS 下一起变），
     * 所以两行数值相同是正常的 —— 用 title 说明，避免被当成"读到同一个字段"的 bug。 */
    if($("monFreq1")&&!$("monFreq1").title)$("monFreq1").title="Both CPU cores use the same clock.";
    $("monHeap").textContent=fmtBytes(hf)+" / "+fmtBytes(ht);
    $("monFlash").textContent=fmtBytes(fu)+" / "+fmtBytes(ft);
    /* ⚠ v25 修（重要）：遥测帧**恒为 25 B**（固件 `serial_proto.c:424` `uint8_t p[25]`），
     * 里面**只有** 24 B 实时量（min/max/avg/blocks/fill/src/status/proto/ppm）——
     * **没有**固件版本、也没有双二阶负载与上限。原来这段放在遥测分支里读 `p[27..33]`，
     * 条件 `p.length>=35` **永远不成立** → 固件版本永远显示 "--"、DSP 负载行永远不变
     * （真机端到端测试台在状态栏上实测到）。
     * 正确的数据源就是本分支的 **SYS_STATUS(0xA6)**：载荷 = 26 B 实时量 + 8 B
     * （`serial_proto.c:474-482`：p[27]/[28] 版本、p[29..30] 双二阶数、p[31..32] 上限、p[33] 预设掩码），
     * 所以这里按**含类型字节**的下标 p[27..33] 读（p = 类型 + 34 B 载荷，长度 35）。 */
    if(p.length>=34){
      /* 下标是**载荷内偏移**（`p[0]` = 载荷首字节，**不含类型字节**；上面的 dv 也从载荷起）。
       * 设备实测：载荷 [26]/[27] = 版本 0.24，[28..29] = 双二阶数，[30..31] = 上限，[32] = 预设掩码。 */
      const vm=dv.getUint8(26),vn=dv.getUint8(27);
      const fc=dv.getUint8(28)|(dv.getUint8(29)<<8),fl=dv.getUint8(30)|(dv.getUint8(31)<<8);
      const pmk=dv.getUint8(32);
      gFwVer=vm+"."+vn;gFiltCount=fc;gFiltLimit=fl;gPresetMask=pmk;
      const el=$("sysVer");if(el)el.textContent="v"+gFwVer;
      const ld=$("sysLoad");
      if(ld){
        ld.textContent=fc+" / "+fl+" biquads";
        ld.style.color=fc>fl?"var(--err)":(fc>fl*0.85?"var(--warn)":"");
        /* 小字提示：上限**只随压缩器开关**变化（响度恒预留）—— 用户最容易在这里困惑 */
        ld.title="The device reports the filter limit for the current mode. "
                +"Read limit: "+fl+" biquads.";
      }
      updPresetUI();
    }
    return;
  }
  if(p.length===3&&p[1]===MSG_ACK){
    const st=p[2];
    if(st!==0)log("Parameter rejected. Message="+p[0]+" status="+st+"（"+(ACK_STATUS_NAME[st]||"unknown")+"）","warn");
    return;
  }
  if(p.length===4&&p[1]===MSG_ERROR){
    log(p[2]===4?"Another control channel owns the device. Stop its requests and wait 15 seconds.":
        "Device error. Message="+p[0]+" code="+p[2]+" detail="+p[3],"err");
    return;
  }
  if(t===MSG_STATUS){log("Device status. Message="+p[1],"info");}
}
function handleBulk(p){
  function rejectBulk(){
    if(bulkTimer){clearTimeout(bulkTimer);bulkTimer=null;}
    bulkBuf=null;bulkGot=0;
    log("Configuration read failed. An unexpected data length was received. No settings were applied.","err");
  }
  if(bulkBuf===null){
    if(p.length<4||p[0]!==0||p[1]!==MSG_BULK_CONFIG){
      return;}
    const total=p[2]|(p[3]<<8);
    if(total<298||total>2304||p.length!==4+Math.min(total,251)){rejectBulk();return;}
    bulkBuf=new Uint8Array(total);bulkGot=0;
    if(bulkTimer)clearTimeout(bulkTimer);
    bulkTimer=setTimeout(()=>{bulkTimer=null;bulkBuf=null;bulkGot=0;log("Configuration read incomplete. Read it again before you make changes.","err")},15000);
    const n=p.length-4;
    bulkBuf.set(p.slice(4),0);bulkGot=n;
  }else{
    const n=Math.min(bulkBuf.length-bulkGot,255);
    if(p.length!==n){rejectBulk();return;}
    bulkBuf.set(p,bulkGot);bulkGot+=n;
  }
  if(bulkGot>=bulkBuf.length){const b=bulkBuf;bulkBuf=null;if(bulkTimer){clearTimeout(bulkTimer);bulkTimer=null}parseBulk(b);}
}
const ACK_STATUS_NAME={0:"OK",1:"Invalid block",2:"Unsupported parameter",3:"Index out of range",4:"Incorrect length",5:"Invalid value",6:"Function not supported",
  7:"Filter limit exceeded. The write was rolled back.",8:"Preset empty or invalid"};
const DSP_MAGIC=0x44535043;

/* DSP预算以SYS_STATUS回读为准，未连接时使用0.122候选130/110。
 * 96k整链固定567+51+(comp?155:0)，mono13/pair19us，最坏布局分段模型。
 * 响度/削波恒预留，上限只随压缩器开关变；manager_task64/model.json。
 * 旧176us是净DSP口径，不能替换本轮整链固定项；新上限仍待实收。
 */
const DSP_USAGE_WARN=0.85;          /* 用到 85% 起黄 */
const DSP_USAGE_DANGER=1.0;         /* 超过上限红 */
/* 这两个单元的开销（生产模型：响度 ≈51 µs、压缩器 ≈155 µs 每块）中，响度**恒预留**，
 * 所以响度不改变上限；而**压缩器会**（候选关130 / 三开110（旧设备以回读为准））。
 * 前端那条硬线就是 `limitNow()`，它只是"更早一步拦 + 弹窗讲清楚"。 */
function budgetHint(){
  if(!gFiltLimit)return"Device filter limit not read";
  const pct=gFiltLimit?Math.round(100*gFiltCount/gFiltLimit):0;
  return gFiltCount+" / "+gFiltLimit+" biquads ("+pct+"%）";
}
/* 加段前的预算检查：设备在超限时会以 status=7 整包回滚，
 * 与其让用户点完再被回滚，不如先算给他看（数字仍以设备读回为准）。
 * ★ v30：**加段一律按 `limitNow()` 判**（压缩器开110 / 关130）——
 *   不再分"压缩器分支 / 设备上限分支"两套阈值，避免两处不一致。
 *   越界要**弹窗**讲清楚，不能只往日志里写一行 —— 用户不会去看日志。 */
function budgetGuard(adding){
  if(!gFiltLimit){
    /* 设备没连上：用与固件**同口径**的本地估算挡（见 localBandCount 的说明） */
    if(bandCountNow()+(adding||1)>limitNow()){
      popup("Cannot add a filter", overLimitBody(adding), "OK");
      log("Current limit: "+limitNow()+" biquads. Compressor "+(cfg.comp.on?"on":"off")+". Used: "
        +bandCountNow()+". Remove filters before you add more.","warn");
      return false;
    }
    return true;
  }
  if(gFiltCount+adding>gFiltLimit){
    if(cfg.comp.on&&compWouldExceed(adding)){
      /* 压缩器开着仍然越界 ⇒ 这不是"设备上限"的问题，而是压缩器那档更严 */
      popup("Cannot add a filter", overLimitBody(adding), "OK");
    } else {
      popup("Cannot add a filter",
        "Used: <b>"+budgetHint()+"</b>. An additional "+(adding||1)+" exceeds the device limit.<br><br>"
        +"Remove other filters first.",
        "OK");
    }
    log("Used: "+budgetHint()+". An additional "+adding+" exceeds the limit ("+gFiltLimit+" biquads). "
      +"Remove other filters first.","warn");
    return false;
  }
  return true;
}
/* 段数控件旁的小字：让"段数"和"总预算"的关系看得见 */
function budgetMiniText(){
  if(!gFiltLimit)return"Device limit unknown";
  return"Channel use: "+budgetHint();
}
const clampN=(n,lo,hi)=>Math.max(lo,Math.min(hi,n|0));
const resizeBands=(arr,n)=>{
  n=clampN(n,1,MAX_BAND);
  while(arr.length<n)arr.push(newBand(arr.length));
  while(arr.length>n)arr.pop();
};
function parseBulk(buf){
  let parsed;
  try{parsed=S31Config.decode(buf,cfg);}
  catch(e){log(e.message,"err");return false;}
  // Only validated data reaches cfg, localStorage, undo history, or the panels.
  cfg=parsed.config;gFs=parsed.sampleRate;
  const step=(name,fn)=>{try{fn()}catch(e){log("Configuration step failed: "+name+". Error: "+e.message,"err")}};
  step("Record undo state",()=>{if(typeof recordHistory==="function")recordHistory("Device read")});
  step("Save in browser",saveLocal);step("Refresh panels",rebuildUI);
  $("sysFs").textContent=gFs+" Hz";$("stFs").textContent=(gFs/1000).toFixed(1)+" kHz";
  log("Configuration read. Preset #"+parsed.preset+". Sample rate: "+gFs+" Hz. Format v"+parsed.version+", "+buf.byteLength+" bytes. Filter arrays match the device.","info");
  return true;
}
/* ================= 串口连接 ================= */
async function disconnectSerial(){
  const r=reader,w=writer,p=port;
  reader=null;writer=null;port=null;resetTransport();stopMon();connUI(false);
  try{if(r)await r.cancel()}catch(e){}
  try{if(r)r.releaseLock()}catch(e){}
  try{if(w){await w.close();w.releaseLock()}}catch(e){try{w.releaseLock()}catch(ignore){}}
  try{if(p)await p.close()}catch(e){log("Serial close: "+e.message,"warn")}
  
  if(bleActive())log("Serial disconnected. Wait for the old device lease to expire before BLE control.","info");
}
async function connect(){
  try{
    port=await navigator.serial.requestPort();await port.open({baudRate:115200});
    writer=port.writable.getWriter();reader=port.readable.getReader();
    const r=reader;resetTransport();const gen=controlGeneration;
    connUI(true);window.__noTransportWarned=0;
    log("Serial connected. Waiting for the device reply.");
    (async()=>{try{for(;;){const{value,done}=await r.read();if(done)break;
      if(value&&reader===r&&writer)onRx(value)}}catch(e){log("Serial read: "+e.message,"err")}
      finally{if(reader===r)await disconnectSerial()}})();
    setTimeout(()=>{if(gen!==controlGeneration||reader!==r)return;
      sendPing();sendSync();startMon();requestAmpSettings();sendFrame(new Uint8Array([MSG_PRESET_INFO]))},800);
  }catch(e){log("Connection failed: "+e.message,"err");await disconnectSerial()}
}
function connUI(on){
  $("connDot").className=on?"dot on":"dot";
  $("connTxt").textContent=on?"Serial connected":"Connect serial";
  $("connStat").classList.toggle("on",on);
}
$("connStat").onclick=async()=>{if(port)await disconnectSerial();else await connect()};

/* BLE: FF01 is WRITE, FF02 is NOTIFY. UART is preferred while open;
 * notifications from an inactive transport never enter the shared parser. */
let bleDev=null,bleRxCh=null,bleTxCh=null,bleGeneration=0;
function bleActive(){return !!(bleDev&&bleDev.gatt.connected&&bleRxCh&&bleTxCh)}
function bleUI(on){
  $("bleDot").className=on?"dot on":"dot";
  $("bleTxt").textContent=on?"BLE connected":"Connect BLE";
  $("bleStat").classList.toggle("on",on);
}
function clearBle(device){
  if(bleDev!==device)return;
  bleGeneration++;bleRxCh=null;bleTxCh=null;bleDev=null;bleUI(false);
  if(!writer){resetTransport();stopMon()}
}
async function bleConnect(){
  let device;
  try{
    if(!navigator.bluetooth){log("This browser does not support Web Bluetooth.","err");return}
    device=await navigator.bluetooth.requestDevice({filters:[{services:[0xff00]}],optionalServices:[0xff00]});
    const gatt=await device.gatt.connect(),svc=await gatt.getPrimaryService(0xff00);
    const rx=await svc.getCharacteristic(0xff01),tx=await svc.getCharacteristic(0xff02);
    bleDev=device;bleRxCh=rx;bleTxCh=tx;const gen=++bleGeneration;
    tx.addEventListener("characteristicvaluechanged",e=>{
      if(gen!==bleGeneration||bleDev!==device||writer)return;
      const v=e.target.value;onRx(new Uint8Array(v.buffer,v.byteOffset,v.byteLength))});
    device.addEventListener("gattserverdisconnected",()=>{clearBle(device);log("BLE disconnected.","warn")},{once:true});
    await tx.startNotifications();
    if(gen!==bleGeneration||!device.gatt.connected)return;
    bleUI(true);window.__noTransportWarned=0;
    if(writer){log("BLE connected. Serial remains the control channel.","info");return}
    resetTransport();log("BLE tuning connected. Configuration writes can briefly affect Bluetooth audio.","info");
    sendPing();sendSync();startMon();requestAmpSettings();sendFrame(new Uint8Array([MSG_PRESET_INFO]));
  }catch(e){log("BLE connection failed: "+e.message,"err");
    if(device){try{device.gatt.disconnect()}catch(ignore){}clearBle(device)}}
}
$("bleStat").onclick=async()=>{
  if(bleActive()){const d=bleDev;try{d.gatt.disconnect()}finally{clearBle(d)};return}
  await bleConnect();
};
$("btnSync").onclick=()=>{if(!transportActive()){log("No control connection.","warn");return}sendSync();startMon()};
$("btnSave").onclick=async()=>{
  if(!transportActive()){log("No control connection.","warn");return}
  if(!(await guardSaveNvs()))return;
  sendSave();
};
$("btnDefault").onclick=()=>{
  if(typeof recordHistory==="function")recordHistory("Defaults");
  if(transportActive()){sendReset();setTimeout(()=>sendSync(),400)}
  else{cfg=defaultCfg();saveLocal();rebuildUI();drawResp();log("Local defaults loaded. The device has not changed.","info")}
};
$("btnTheme").onclick=()=>setTheme(gTheme==="light"?"dark":"light");

/* ================= 配置导入导出 ================= */
function cl(v,lo,hi){v=+v;if(!isFinite(v))v=lo;return Math.max(lo,Math.min(hi,v))}
/* cfg.link 归一化：新格式 [端口0, 端口1]；老格式（单个 0/1）或缺字段都兼容 */
function normLink(v){return Array.isArray(v)?[v[0]?1:0,v[1]?1:0]
  :(v===undefined||v===null?[1,1]:(v?[1,1]:[0,0]))}
function fixBand(b,i){
  const d=newBand(i||0);
  if(!b||typeof b!=="object")return d;
  return{on:b.on?1:0,
    type:(b.type>=F_PEAK&&b.type<=F_AP)?b.type:F_PEAK,
    f:cl(b.f,20,20000),g:cl(b.g,-EQ_GAIN_MAX,EQ_GAIN_MAX),q:cl(b.q,0.4,10)};
}
function fixBands(a){
  const n=Math.max(0,Math.min(MAX_BAND,Array.isArray(a)?a.length:0));
  const o=[];for(let i=0;i<n;i++)o.push(fixBand(a[i],i));
  // A file with 1..15 bands keeps that exact count. Use defaults only when
  // no list was supplied; importing must not silently add inactive bands.
  if(!o.length)while(o.length<DEF_BAND)o.push(newBand(o.length));
  return o;
}
function fixXo(x){
  const d=newXo();
  if(!x||typeof x!=="object")return d;
  return{on:x.on?1:0,f:cl(x.f,20,20000),
    type:(x.type===XO_BW||x.type===XO_LR)?x.type:XO_LR,
    slope:([SLOPE12,SLOPE24,SLOPE48].indexOf(x.slope)>=0)?x.slope:SLOPE24};
}
function fixIn(x){
  const o={gain:0,mute:0,phase:0,eq:[],req:[]};
  if(!x||typeof x!=="object"){o.eq=fixBands([]);o.req=fixBands([]);return o}
  o.gain=cl(x.gain,-72,12);o.mute=x.mute?1:0;o.phase=x.phase?1:0;
  o.eq=fixBands(x.eq);o.req=fixBands(x.req);
  return o;
}
function fixOut(x){
  const o={eq:[],hp:newXo(),lp:newXo()};
  if(!x||typeof x!=="object"){o.eq=fixBands([]);return o}
  o.eq=fixBands(x.eq);o.hp=fixXo(x.hp);o.lp=fixXo(x.lp);
  return o;
}
function normalizeCfg(c){
  if(!c||typeof c!=="object")return null;
  const inputs=(Array.isArray(c.inputs)?c.inputs:[]).slice(0,2);
  const outs=(Array.isArray(c.outs)?c.outs:[]).slice(0,4);
  while(inputs.length<2)inputs.push(newIn());
  while(outs.length<4)outs.push(newDspOut());
  const route=[];
  for(let i=0;i<2;i++){
    const r=(Array.isArray(c.route)&&Array.isArray(c.route[i]))?c.route[i]:[];
    const rr=[];
    for(let o=0;o<4;o++){
      const x=r[o];
      /* 缺项回落到出厂默认（全通）；显式给出但为空的对象按关闭处理 */
      rr.push((x===undefined||x===null)
        ? newRoute()
        : {on:x.on?1:0,gain:cl(x.gain===undefined?1:x.gain,0,1),src:cl(x.src===undefined?0:x.src,0,3)});
    }
    route.push(rr);
  }
  const ports=[];
  for(let i=0;i<2;i++){
    const p=(Array.isArray(c.ports)?c.ports[i]:null);
    if(p===undefined||p===null){ports.push(newPort(i));continue}
    ports.push({mode:cl(p.mode,0,2),format:cl(p.format,0,2),
      gain:cl(p.gain,-24,12),delay:cl(p.delay,0,20),enabled:p.enabled?1:0,
      hp:fixXo(p.hp),lp:fixXo(p.lp)});
  }
  PORTS.forEach((p,i)=>{if(p.disabled)ports[i].enabled=0});
  const l=c.lim&&typeof c.lim==="object"?c.lim:{};
  const lim={on:l.on?1:0,thresh:cl(l.thresh===undefined?-1:l.thresh,-24,0),
             release:cl(l.release===undefined?100:l.release,10,1000)};
  const ld=c.loud&&typeof c.loud==="object"?c.loud:{};
  const loud={on:ld.on?1:0,amount:cl(ld.amount===undefined?10:ld.amount,0,15),
              ref:cl(ld.ref===undefined?0.8:ld.ref,0.05,1)};
  /* 压缩器：默认与固件一致（关 / -30 dBFS / 4.0:1 / 200 ms） */
  const cp=c.comp&&typeof c.comp==="object"?c.comp:{};
  const comp={on:cp.on?1:0,th:Math.round(cl(cp.th===undefined?-30:cp.th,-60,0)),
              ratio:Math.round(cl(cp.ratio===undefined?40:cp.ratio,COMP_RATIO_MIN,COMP_RATIO_MAX)),
              rel:Math.round(cl(cp.rel===undefined?200:cp.rel,COMP_REL_MIN,COMP_REL_MAX))};
  /* ★ v27 精度模式 / v28 对用户隐式：缺项回落到**扩展（v32 起两种模式同值；现为候选关130 / 三开110（旧设备以回读为准），默认形态）**；
   * ⚠ 一致性：**压缩器开着就归一到高精度** —— v32 起固件**不再**硬拒"扩展 + 压缩器开"
   *   （两种模式上限同值），这里保留归一只是为了维持"压缩器 ⇒ 高精度"这个既有约定；
   *   并且把 comp.on **保留**、不悄悄关掉用户的压缩器。 */
  const precRaw=cl(c.precision===undefined?PRECISION_EXTENDED:c.precision,PRECISION_HIGH,PRECISION_EXTENDED);
  /* ★ v30：`precFinal` 归一**撤掉** —— 精度模式与预算无关，不再"压缩器开就归一为高精度"。
   *   缺项直接回落到扩展（与固件 `DSP_PRECISION_DEFAULT` 一致），有值就原样保留。 */
  const precFinal=precRaw;
  /* ★ 缺陷修复（v43，C42）：这里**原先漏了 `hw`** —— 于是 `applyCfg()` 走
   *   `cfg=normalizeCfg(c)` 之后 `cfg.hw` 变成 undefined，紧接着 `rebuildUI()→syncHwUI()`
   *   与 `pushAll()` 四条 hw 下发全部无保护地读 `cfg.hw.*` ⇒
   *   `TypeError: Cannot read properties of undefined (reading 'delayA')`，
   *   导入配置文件 / Ctrl+Z 撤销 / Ctrl+Y 重做三个核心动作当场崩
   *   （而且 `applyCfg` 里 `saveLocal()` 排在崩溃之前 ⇒ 顺带把"没有 hw 的配置"写进 localStorage）。
   *   自 v36 引入、红了 7 轮。夹取口径与 `saveLocal`/BULK 解析一致：
   *   延时 ∈ 0..HW_DELAY_MAX（u16，固件端也夹），极性归 0/1。
   *   ⛔ 0.112：原第四项 `enc`（编码器启用 / `0x83`）已删除，**不在这里恢复**。 */
  const hwSrc=(c.hw&&typeof c.hw==="object")?c.hw:{};
  const hw={delayA:cl(hwSrc.delayA===undefined?HW_DELAY_DEFAULT:hwSrc.delayA,0,HW_DELAY_MAX),
            delayB:cl(hwSrc.delayB===undefined?HW_DELAY_DEFAULT:hwSrc.delayB,0,HW_DELAY_MAX),
            pol:hwSrc.pol?1:0};
  /* ⚠ 这里**有意**不恢复 `comp` / `precision`（不要当成漏项补上，C42）：
   *   `precision` 是纯 blob 兼容字节、界面上没有这个开关（见下面 `P_PRECISION_MODE` 那段的注释）；
   *   `comp` 默认关是"保持出厂旁通"的既有口径 —— 它们各自的缺省语义见上面 precRaw / comp 两段注释。 */
  return{inputs:inputs.map(fixIn),outs:outs.map(fixOut),route,
    master:cl(c.master===undefined?1:c.master,0,1),link:normLink(c.link),
    ports,lim,loud,comp,hw,precision:precFinal};
}
/* ---- REW / EqualizerAPO EQ 文本导入 ----
 * 支持的行格式（三家写法都认，大小写不敏感、字段顺序任意）：
 *   REW        : Filter 1: ON PK Fc 100 Hz Gain -3.0 dB Q 1.40
 *   REW(旧)    : Filter  1: ON  None
 *   EqualizerAPO: Filter 1: ON PK Fc 100 Hz Gain -3.0 dB Q 1.40
 *   APO 省略式 : Filter 1: ON PK 100 -3 1.4          （Fc Gain Q 按位置）
 *   APO 简化式 : Filter 1: ON PK Fc 100 Hz Gain -3 dB   （无 Q → 用 0.707）
 *   Preamp: -6.0 dB                                  （忽略，仅提示）
 * 类型关键字：PK/PEAK/PEQ · LS/LSC/LOWSHELF · HS/HSC/HIGHSHELF · LP/LPF/LOWPASS ·
 *   HP/HPF/HIGHPASS · BP/BPF/BANDPASS · NO/NOTCH · AP/APF/ALLPASS · NONE/OFF（跳过该行）
 * Q 也接受 BW（带宽，Q = 1/(2·sinh(ln2/2·BW))）与 S（架式斜率，Q = 1/√((A+1/A)(1/S−1)+2)）。 */
const EQ_TYPE_MAP={PK:F_PEAK,PEAK:F_PEAK,PEQ:F_PEAK,
  LS:F_LSHELF,LSC:F_LSHELF,LOWSHELF:F_LSHELF,
  HS:F_HSHELF,HSC:F_HSHELF,HIGHSHELF:F_HSHELF,
  LP:F_LP,LPF:F_LP,LOWPASS:F_LP,
  HP:F_HP,HPF:F_HP,HIGHPASS:F_HP,
  BP:F_BP,BPF:F_BP,BANDPASS:F_BP,
  NO:F_NOTCH,NOTCH:F_NOTCH,
  AP:F_AP,APF:F_AP,ALLPASS:F_AP};
const EQ_NOOP=new Set(["NONE","OFF","NOOP","-"]);
/* BW（倍频程带宽）→ Q */
function bwToQ(bw){return bw>0?1/(2*Math.sinh(Math.LN2/2*bw)):0.707}
/* S（架式斜率）→ Q：gain 为 dB */
function shelfSToQ(s,gdb){
  const A=Math.pow(10,(+gdb||0)/40);
  const den=(A+1/A)*(1/s-1)+2;
  return den>0?1/Math.sqrt(den):0.707;
}
/* 把一行 Filter 的剩余字段解析成 {f,g,q}（缺省值可传） */
function parseFilterFields(rest,type,gDef){
  const num=/[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?/;
  let f=null,g=null,q=null;
  const fm=rest.match(new RegExp("Fc\\s*("+num.source+")\\s*(k?Hz|K?HZ)?","i"));
  if(fm){f=+fm[1];if(fm[2]&&/^k/i.test(fm[2]))f*=1000;}
  const gm=rest.match(new RegExp("Gain\\s*("+num.source+")\\s*dB","i"));
  if(gm)g=+gm[1];
  const qm=rest.match(new RegExp("Q\\s*("+num.source+")","i"));
  if(qm)q=+qm[1];
  const bm=rest.match(new RegExp("BW\\s*("+num.source+")","i"));
  if(qm===null&&bm)q=bwToQ(+bm[1]);
  const sm=rest.match(new RegExp("S\\s*("+num.source+")","i"));
  if(qm===null&&bm===null&&sm&&sm[1]>0)q=shelfSToQ(+sm[1],g===null?0:g);
  /* APO 省略式：'100 -3 1.4' —— 已带单位的写法不会走到这里（上面三个 match 会命中）
     也接受裸 k 后缀：'1k -3' → 1000 Hz */
  if(f===null&&g===null&&q===null){
    const st=rest.replace(/\b(ON|OFF)\b/gi," ")
      .replace(/\b(FC|GAIN|Q|BW|S)\b/gi," ");
    const nm=st.match(new RegExp("^[\\s,;]*("+num.source+")([kK])?(?:[\\s,;]+("+num.source+"))?(?:[\\s,;]+("+num.source+"))?","i"));
    if(nm&&nm[1]!==undefined){
      f=+nm[1];
      if(nm[2])f*=1000;
      if(nm[3]!==undefined)g=+nm[3];
      if(nm[4]!==undefined)q=+nm[4];
    }
  }
  if(f===null)f=1000;
  if(g===null)g=(gDef===undefined?0:gDef);   /* LP/HP/BP/NO/AP 这类没给 Gain 就按 0 */
  if(q===null)q=(type===F_LP||type===F_HP)?0.707:0.707;
  return{f:f,g:g,q:q};
}
function parseEqText(text){
  const out={bands:[],preamp:null,skipped:0};
  if(typeof text!=="string")return out;
  const lines=text.split(/\r?\n/);
  for(const raw of lines){
    /* EqualizerAPO 用 \t 分隔，REW 用空格；都先归一成空格 */
    const line=raw.replace(/\t/g," ").trim();
    if(!line||line.startsWith("#")||line.startsWith("//"))continue;
    const pm=line.match(/^Preamp\s*:\s*([-+0-9.eE]+)\s*dB/i);
    if(pm){out.preamp=+pm[1];continue}
    /* 行识别收紧：必须是 Filter N: 或 Filter N ON ...（避免把任意含 "Filter 1" 的文本吃进来） */
    const m=line.match(/^Filter\s*(\d+)\s*[:.]?\s+(ON|OFF)\b\s*(.*)$/i);
    if(!m){if(/\bFilter\b/i.test(line))out.skipped++;continue}
    const on=m[2].toUpperCase()==="ON"?1:0;
    let rest=(m[3]||"").trim();
    /* 类型：取首词，逗号/分号也当分隔 */
    const tm=rest.match(/^([A-Za-z]+)/);
    const kw=tm?tm[1].toUpperCase():"";
    if(EQ_NOOP.has(kw)||rest===""){out.skipped++;continue}     /* NONE / OFF：本段无滤波 */
    const type=EQ_TYPE_MAP[kw];
    if(type===undefined){out.skipped++;continue}
    rest=rest.slice(tm[0].length);
    const v=parseFilterFields(rest,type,0);
    out.bands.push({on:on,type:type,f:cl(v.f,20,20000),g:cl(v.g,-EQ_GAIN_MAX,EQ_GAIN_MAX),q:cl(v.q,0.4,10)});
  }
  return out;
}
function applyEqImport(text,target){
  const r=parseEqText(text);
  if(!r.bands.length){log("No supported filter lines found. Use REW or EqualizerAPO text.","err");return null}
  const use=Math.max(0,Math.min(MAX_BAND,r.bands.length));
  const list=r.bands.slice(0,use);
  while(list.length<DEF_BAND)list.push(newBand(list.length));
  const targets=target==="1"?[0]:target==="2"?[1]:[0,1];
  targets.forEach(ci=>{cfg.outs[ci].eq=list.map(b=>({on:b.on,type:b.type,f:b.f,g:b.g,q:b.q}))});
  return r;
}
function cfgSnapshot(){return JSON.parse(JSON.stringify(cfg))}
// A sound-only file deliberately excludes routing, ports, GPIO mute and the
// amplifier profile. Reject invalid/extra fields before changing any setting.
const DSP_FILE_FIELDS=["inputs","outs","master","lim","loud","comp"];
function dspParametersSnapshot(){
  const clean=normalizeCfg(cfgSnapshot());
  return {format:"s31-dsp-parameters",version:1,...Object.fromEntries(DSP_FILE_FIELDS.map(k=>[k,clean[k]]))};
}
function decodeDspParameters(data){
  const allowed=["format","version",...DSP_FILE_FIELDS].sort();
  if(!data||Array.isArray(data)||data.format!=="s31-dsp-parameters"||data.version!==1||
    JSON.stringify(Object.keys(data).sort())!==JSON.stringify(allowed))throw Error("Invalid DSP parameter file.");
  const candidate=normalizeCfg({...cfgSnapshot(),...Object.fromEntries(DSP_FILE_FIELDS.map(k=>[k,data[k]]))});
  const stable=v=>Array.isArray(v)?v.map(stable):v&&typeof v==="object"?
    Object.fromEntries(Object.keys(v).sort().map(k=>[k,stable(v[k])])):v;
  for(const key of DSP_FILE_FIELDS){
    if(JSON.stringify(stable(data[key]))!==JSON.stringify(stable(candidate[key])))throw Error("Invalid DSP parameter file.");
  }
  return candidate;
}
function pushAll(dspOnly=false){
  if(!dspOnly&&cfg.ports[1].format===2){log("I2S1 right-justified format is not supported. No configuration was sent.","err");return false}
  if(!transportActive()){log("No control connection. Changes stay in this browser.","warn");return false}
  // A bulk snapshot already contains both channel values. The link switch is
  // for live editing; mirroring here would overwrite a valid imported channel.
  const savedLink=cfg.link;cfg.link=[0,0];
  try{
  const eqSend=(block,ch,arr,pOn,pF,pG,pQ,pT,pN)=>{
    setParam(block,ch,pN,0,arr.length,false);
    arr.forEach((b,i)=>{
      setParam(block,ch,pOn,i,b.on,false);
      setParam(block,ch,pF,i,b.f,true);
      setParam(block,ch,pG,i,b.g,true);
      setParam(block,ch,pQ,i,b.q,true);
      setParam(block,ch,pT,i,b.type,false);
    });
  };
  cfg.inputs.forEach((inp,ch)=>{
    setParam(BLOCK_INPUT,ch,P_GAIN,0,inp.gain,true);
    setParam(BLOCK_INPUT,ch,P_MUTE,0,inp.mute,false);
    setParam(BLOCK_INPUT,ch,P_PHASE,0,inp.phase,false);
    eqSend(BLOCK_INPUT,ch,inp.eq,P_EQ_ON,P_EQ_FREQ,P_EQ_GAIN,P_EQ_Q,P_EQ_TYPE,P_EQ_COUNT);
    eqSend(BLOCK_INPUT,ch,inp.req,P_REQ_ON,P_REQ_FREQ,P_REQ_GAIN,P_REQ_Q,P_REQ_TYPE,P_REQ_COUNT);
  });
  cfg.outs.forEach((o,ch)=>{
    eqSend(BLOCK_OUTPUT,ch,o.eq,P_EQ_ON,P_EQ_FREQ,P_EQ_GAIN,P_EQ_Q,P_EQ_TYPE,P_EQ_COUNT);
    setParam(BLOCK_OUTPUT,ch,P_HP_ON,0,o.hp.on,false);
    setParam(BLOCK_OUTPUT,ch,P_HP_FREQ,0,o.hp.f,true);
    setParam(BLOCK_OUTPUT,ch,P_HP_TYPE,0,o.hp.type,false);
    setParam(BLOCK_OUTPUT,ch,P_HP_SLOPE,0,o.hp.slope,false);
    setParam(BLOCK_OUTPUT,ch,P_LP_ON,0,o.lp.on,false);
    setParam(BLOCK_OUTPUT,ch,P_LP_FREQ,0,o.lp.f,true);
    setParam(BLOCK_OUTPUT,ch,P_LP_TYPE,0,o.lp.type,false);
    setParam(BLOCK_OUTPUT,ch,P_LP_SLOPE,0,o.lp.slope,false);
  });
  if(!dspOnly)cfg.route.forEach((r,i)=>r.forEach((c,o)=>{
    setParam(BLOCK_ROUTING,i,P_ROUTE_ON,o,c.on,false);
    setParam(BLOCK_ROUTING,i,P_ROUTE_GAIN,o,c.gain,true);
    setParam(BLOCK_ROUTING,i,P_ROUTE_SRC,o,c.src,false);
  }));
  setParam(BLOCK_GLOBAL,0,P_MASTER_VOL,0,cfg.master,true);
  /* ★ v41（2026-09-20）**订正这段过期注释** —— 它原来写的是：
   *   "精度模式必须**最先**下发：它决定设备当前的双二阶上限（41 / 80），
   *    后面所有滤波写入都要按新上限判定；放在最后会出现'按旧上限判超限、整包回滚'。"
   *   那是 **v27/v28 的语义**：**v30 起精度模式与预算彻底无关**
   *   （两种模式上限同值，只由压缩器开关决定：候选关130 / 三开110（旧设备以回读为准））。
   *   ⇒ 那段理由**已不成立**，而它正是这行代码一直没被清理的原因。
   *   ⚠ 本行**保留**只为**保 blob 兼容**（`cfg.precision` 原样带着，UI **永不修改它**）——
   *     与本文档后段（2942 / 3099 行附近）的口径一致。
   *   ⚠ 设备侧配套改动（固件 **0.40**）：`0x80` 补了 `pure_control = true;`，
   *     所以这行**同值重写**不再触发无谓的引擎重建（登记册 L19）。 */
  if(!dspOnly)setParam(BLOCK_GLOBAL,0,P_PRECISION_MODE,0,cfg.precision,false);
  setParam(BLOCK_GLOBAL,0,P_LIMIT_ON,0,cfg.lim.on,false);
  setParam(BLOCK_GLOBAL,0,P_LIMIT_THRESH,0,cfg.lim.thresh,true);
  setParam(BLOCK_GLOBAL,0,P_LIMIT_RELEASE,0,cfg.lim.release,true);
  /* 响度补偿（等响）：必须在主音量之后下发，否则设备会按旧主音量算一次补偿量 */
  setParam(BLOCK_GLOBAL,0,P_LOUD_ON,0,cfg.loud.on,false);
  setParam(BLOCK_GLOBAL,0,P_LOUD_AMOUNT,0,cfg.loud.amount,true);
  setParam(BLOCK_GLOBAL,0,P_LOUD_REF,0,cfg.loud.ref,true);
  /* 压缩器（动态范围 / 夜间模式）：阈值/压缩比/释放是整数参数，整型下发 */
  setParam(BLOCK_GLOBAL,0,P_COMP_TH,0,cfg.comp.th,false);
  setParam(BLOCK_GLOBAL,0,P_COMP_RATIO,0,cfg.comp.ratio,false,2);   /* v2: u16 */
  setParam(BLOCK_GLOBAL,0,P_COMP_REL,0,cfg.comp.rel,false);
  setParam(BLOCK_GLOBAL,0,P_COMP_ON,0,cfg.comp.on,false);
  /* ★ v36（阶段 B）：硬件控制层三项 —— **两个 u16 必须带 nbytes=2**（否则固件判 status=4 ERR_LENGTH）。
   * ⛔ 0.112：不再下发 `0x83`（编码器启用，已废弃；固件会回 `status=0x02 ERR_PARAM`）。 */
  if(!dspOnly){
    setParam(BLOCK_GLOBAL,0,P_HW_DELAY_A,0,cfg.hw.delayA,false,2);
    setParam(BLOCK_GLOBAL,0,P_HW_DELAY_B,0,cfg.hw.delayB,false,2);
    setParam(BLOCK_GLOBAL,0,P_HW_POLARITY,0,cfg.hw.pol,false);
  }
  if(!dspOnly)cfg.ports.forEach((p,i)=>{
    /* 双 I2S 均已启用：两个端口的端点参数都下发。 */
    {
      setParam(BLOCK_GLOBAL,0,P_PORT_ENABLE,i,p.enabled?1:0,false);
      setParam(BLOCK_GLOBAL,0,P_PORT_MODE,i,p.mode,false);
      /* I2S 线格式也要全量重发：它只在端口卡下拉改变时下发过一次，
       * 出厂默认/预设载入/撤销都会把设备打回 0(I2S Philips) 而界面还显示旧值。 */
      setParam(BLOCK_GLOBAL,0,P_PORT_FORMAT,i,p.format,false);
      setParam(BLOCK_GLOBAL,0,P_PORT_GAIN,i,p.gain,true);
      setParam(BLOCK_GLOBAL,0,P_PORT_DELAY,i,p.delay,true);
    }
    setParam(BLOCK_GLOBAL,0,P_PORT_HP_ON,i,p.hp.on,false);
    setParam(BLOCK_GLOBAL,0,P_PORT_HP_FREQ,i,p.hp.f,true);
    setParam(BLOCK_GLOBAL,0,P_PORT_HP_TYPE,i,p.hp.type,false);
    setParam(BLOCK_GLOBAL,0,P_PORT_HP_SLOPE,i,p.hp.slope,false);
    setParam(BLOCK_GLOBAL,0,P_PORT_LP_ON,i,p.lp.on,false);
    setParam(BLOCK_GLOBAL,0,P_PORT_LP_FREQ,i,p.lp.f,true);
    setParam(BLOCK_GLOBAL,0,P_PORT_LP_TYPE,i,p.lp.type,false);
    setParam(BLOCK_GLOBAL,0,P_PORT_LP_SLOPE,i,p.lp.slope,false);
  });
  return true;
  }finally{cfg.link=savedLink;}
}
function applyCfg(c,label,dspOnly=false){
  const n=normalizeCfg(c);
  if(!n){log("Invalid configuration ignored.","warn");return false}
  if(n.ports[1].format===2){log("I2S1 does not support right-justified format. Select Philips or left-justified format before you apply this configuration.","err");return false}
  /* 撤销点记在改动之前，这样 Ctrl+Z 能回到改动前的状态 */
  if(typeof recordHistory==="function")recordHistory(label?("Apply "+label):"Apply configuration");
  cfg=n;saveLocal();rebuildUI();drawResp();
  const sent=pushAll(dspOnly);
  log((sent?"Configuration queued for the device: ":"Local configuration applied. Device disconnected: ")+(label||"configuration"),"info");
  return true;
}
function exportJsonFile(value,prefix){
  const data=JSON.stringify(value,null,2);
  const blob=new Blob([data],{type:"application/json"});
  const a=document.createElement("a");
  a.href=URL.createObjectURL(blob);
  a.download=prefix+new Date().toISOString().replace(/[:.]/g,"-").slice(0,19)+".json";
  document.body.appendChild(a);a.click();
  setTimeout(()=>{URL.revokeObjectURL(a.href);a.remove()},500);
  log("Configuration exported as JSON ("+data.length+" bytes).","info");
}
$("btnExport").onclick=()=>exportJsonFile(normalizeCfg(cfgSnapshot()),"s31-config-");
$("btnExportDsp").onclick=()=>exportJsonFile(dspParametersSnapshot(),"s31-dsp-");
$("btnImportDsp").onclick=()=>$("dspFile").click();
$("dspFile").onchange=async e=>{
  const file=e.target.files&&e.target.files[0];e.target.value="";if(!file)return;
  try{
    if(file.size>131072)throw Error("DSP parameter file exceeds 128 KiB.");
    const data=JSON.parse(await file.text());
    applyCfg(decodeDspParameters(data),"DSP parameter file",true);
  }catch(error){log(error instanceof SyntaxError?"Invalid DSP parameter file.":error.message||"Invalid DSP parameter file.","err");}
};
$("btnImport").onclick=()=>$("importFile").click();
$("importFile").onchange=e=>{
  const f=e.target.files&&e.target.files[0];
  e.target.value="";
  if(!f)return;
  const r=new FileReader();
  r.onload=()=>{
    try{
      const data=JSON.parse(r.result);
      if(data&&data.format==="s31-dsp-parameters"){log("Use Import DSP parameters for this file.","warn");return;}
      applyCfg(data,"Import file "+f.name);
    }
    catch(err){log("Invalid JSON ("+err.message+"）","warn")}
  };
  r.onerror=()=>log("Cannot read the file.","warn");
  r.readAsText(f);
};
$("btnImportEq").onclick=()=>$("importEqFile").click();
$("importEqFile").onchange=e=>{
  const f=e.target.files&&e.target.files[0];
  e.target.value="";
  if(!f)return;
  const r=new FileReader();
  r.onload=()=>{
    const t=String(r.result||"");
    const tgt=(prompt(S31I18N.t("Select channels: enter 1, 2, or 12 for both channels."),"12")||"12").trim();
    const before=(typeof cfgSnapshot==="function")?cfgSnapshot():null;   /* 改动前的快照（撤销点） */
    const res=applyEqImport(t,tgt);
    if(!res)return;
    if(typeof recordNamed==="function")recordNamed(before,"Import EQ ("+f.name+"）");
    saveLocal();rebuildUI();drawResp();
    const sent=pushAll();
    log("Imported "+res.bands.length+" EQ bands to channel "+(tgt==="1"?"1":tgt==="2"?"2":"1+2")+
        (res.preamp!==null?(". File preamp "+res.preamp+" dB was not applied. Set master volume separately."):"")+
        (res.skipped?(". Skipped "+res.skipped+" lines"):"")+
        (sent?". Queued for the device.":". Local changes only."),
        sent?"info":"warn");
  };
  r.onerror=()=>log("Cannot read the EQ file.","err");
  r.readAsText(f);
};

/* ================= 撤销 / 重做 =================
 * 记录的是「整份配置快照」，与预设/导出用的是同一份 JSON（cfgSnapshot）。
 * 模型：hist 是一串「状态」，histIdx 指向当前状态；撤销/重做 = 移动指针 + applyCfg
 *      （applyCfg 会全量下发，所以界面与设备始终一致）。
 * 记录时机（两种都记「当前/改后」的状态）：
 *   · 面板编辑：setParam() 触发的 debounced 记录（拖滑块/连续输入只记一次）；
 *     没真变的话被去重丢掉，所以不必在按下时抓"改动前"快照。
 *   · 结构性变化（预设载入 / 导入 / 出厂默认 / 设备同步 / A-B 切换）→ 在改动前
 *     recordHistory(label) 显式记录，撤销即可回到改动前。
 * 去重基线 baseSnap = 最后一次进历史的状态，避免重复记录（拖滑块、回读等都靠它收敛）。 */
const HIST_MAX=50;
let hist=[],histIdx=-1,histLock=false,baseSnap=null;
function diffs(a,b){return JSON.stringify(a)!==JSON.stringify(b)}
function recordSnapshot(snap,label){
  if(histLock||!snap)return;
  if(baseSnap&&!diffs(snap,baseSnap))return;               /* 与基线一致 → 没变，不记 */
  hist=hist.slice(0,histIdx+1);
  hist.push({cfg:snap,label:label||""});
  if(hist.length>HIST_MAX)hist=hist.slice(hist.length-HIST_MAX);
  histIdx=hist.length-1;
  baseSnap=snap;
  undoUI();
}
/* 结构性变化：在「改之前」调用可让撤销回到改动前（applyCfg 内部就是先记再改） */
function recordHistory(label){recordSnapshot(cfgSnapshot(),label)}
/* 面板编辑：改动完成后记一次「当前（改后）状态」；没真变的话 recordSnapshot 会去重丢掉。
   （不需要"改动前"的哨兵：拿当前状态与基线比就能知道有没有变过。） */
function recordPending(label){recordSnapshot(cfgSnapshot(),label)}
/* 在「已经改完」的场合用：调用方先把改动前的快照取好，改完再把这个快照记进历史 */
function recordNamed(snap,label){recordSnapshot(snap?JSON.parse(JSON.stringify(snap)):null,label||"Change")}
/* 面板编辑的落点：debounce 之后记一次当前状态（没真变会被去重丢掉） */
const recordHistorySoon=debounced(()=>recordPending(null),400);
function undoUI(){
  const st=$("undoState");
  if(st)st.textContent=(histIdx>=0?(histIdx+1):0)+" / "+hist.length+
    (histIdx>0?(" Previous: "+((hist[histIdx]&&hist[histIdx].label)||"Edit")):"")+
    (histIdx<hist.length-1?" (redo available)":"");
  const u=$("btnUndo"),r=$("btnRedo");
  if(u)u.disabled=(histIdx<=0);
  if(r)r.disabled=(histIdx>=hist.length-1);
}
function undoStep(){
  if(histIdx<=0){log("No earlier state.","warn");return}
  histIdx--;
  histLock=true;
  try{applyCfg(hist[histIdx].cfg,"Undo: "+((hist[histIdx].label)||"previous state"))}
  finally{histLock=false}
  baseSnap=hist[histIdx].cfg;              /* 撤销后基线跟着走，之后的编辑才对得上 */
  undoUI();
}
function redoStep(){
  if(histIdx>=hist.length-1){log("No state to redo.","warn");return}
  histIdx++;
  histLock=true;
  try{applyCfg(hist[histIdx].cfg,"Redo: "+((hist[histIdx].label)||"next state"))}
  finally{histLock=false}
  baseSnap=hist[histIdx].cfg;
  undoUI();
}
$("btnUndo").onclick=undoStep;
$("btnRedo").onclick=redoStep;
$("btnHistClear").onclick=()=>{
  hist=[];histIdx=-1;histLock=true;
  try{const s=cfgSnapshot();hist=[{cfg:s,label:"New starting state"}];histIdx=0;baseSnap=s}finally{histLock=false}
  undoUI();log("Undo history cleared. The current state is the new starting state.","info");
};
window.addEventListener("keydown",e=>{
  if(!e.ctrlKey&&!e.metaKey)return;
  const t=e.target;
  if(t&&(t.tagName==="INPUT"||t.tagName==="TEXTAREA"||t.isContentEditable))return;   /* 输入框里让浏览器自己处理 */
  const k=(e.key||"").toLowerCase();
  if(k==="z"&&!e.shiftKey){e.preventDefault();undoStep()}
  else if(k==="y"||(k==="z"&&e.shiftKey)){e.preventDefault();redoStep()}
});
hist=[{cfg:cfgSnapshot(),label:"Initial state"}];histIdx=0;baseSnap=hist[0].cfg;undoUI();
function onRx(chunk){
  const nb=new Uint8Array(rxBuf.length+chunk.length);nb.set(rxBuf);nb.set(chunk,rxBuf.length);rxBuf=nb;
  for(;;){
    const i=rxBuf.indexOf(H0);
    if(i<0){rxBuf=new Uint8Array(0);return;}
    if(i>0){rxBuf=rxBuf.slice(i);continue;}
    if(rxBuf.length<4)return;
    if(rxBuf[1]!==H1){rxBuf=rxBuf.slice(1);continue;}
    const len=rxBuf[2];
    if(rxBuf.length<4+len)return;
    const payload=rxBuf.slice(3,3+len);
    const act=crc8(rxBuf.slice(2,3+len));
    if(act===rxBuf[3+len]){
      /* ⚠ v25：分帧后必须兜住异常 —— 否则任何一条畸形/未预期长度的帧都会让整个读循环停摆，
       * 而且异常只会出现在控制台里，用户看到的是"界面卡住不动"。真机端到端测试台
       * 就是靠这个把"遥测下标偏移"抓出来的。 */
      try{ handlePayload(payload); }
      catch(err){
        if(!window.__rxErrOnce){window.__rxErrOnce=Object.create(null);}
        const k=(payload[0]||0)+":"+payload.length;
        if(!window.__rxErrOnce[k]){window.__rxErrOnce[k]=1;
          log("Parse error. Type 0x"+(payload[0]||0).toString(16)+", length "+payload.length+")："+err.message,"err");}
      }
    }
    rxBuf=rxBuf.slice(4+len);
  }
}
/* ================= UI 构建 ================= */
function tog(on,fn){const el=document.createElement("span");el.className="tog"+(on?" on":"");
  el.tabIndex=0;el.setAttribute("aria-label","Enable setting");
  el.onkeydown=e=>{if(e.key===" "||e.key==="Enter"){e.preventDefault();el.click();}};
  el.setAttribute("role","switch");el.setAttribute("aria-checked",on?"true":"false");
  el.onclick=()=>{el.classList.toggle("on");const v=el.classList.contains("on");
    el.setAttribute("aria-checked",v?"true":"false");fn(v?1:0)};return el}
function freqInput(f0,oninput){
  const FMIN=20,FMAX=20000;
  const clampF=f=>Math.max(FMIN,Math.min(FMAX,Math.round(f)));
  const wrap=document.createElement("div");wrap.className="nwrap";
  const inp=document.createElement("input");inp.type="number";inp.min=FMIN;inp.max=FMAX;inp.step=1;
  inp.className="numinp";inp.value=String(clampF(f0));
  const unit=document.createElement("span");unit.className="unit";unit.textContent="Hz";
  let cur=clampF(f0);
  const apply=(f,doSend)=>{const cf=clampF(f);cur=cf;inp.value=String(cf);oninput(cf,doSend)};
  inp.addEventListener("wheel",e=>{
    e.preventDefault();
    const typed=parseInt(inp.value,10);
    const base=isFinite(typed)?typed:cur;
    const step=Math.max(1,Math.round(base*0.05));
    apply(base+(e.deltaY<0?step:-step),true);
  },{passive:false});
  inp.addEventListener("input",()=>{
    const v=parseInt(inp.value,10);
    if(isFinite(v)&&v>=FMIN&&v<=FMAX){cur=v;oninput(v,false)}
  });
  inp.addEventListener("change",()=>{
    const v=parseInt(inp.value,10);
    apply(isFinite(v)?v:cur,true);
  });
  inp.addEventListener("focus",()=>inp.select());
  wrap.appendChild(inp);wrap.appendChild(unit);
  return{el:wrap,inp};
}
function numInput(f0,min,max,step,unit,oninput){
  const decimals=step<1?(String(step).split(".")[1]||"").length:0;
  const clampF=f=>{const v=Math.max(min,Math.min(max,Math.round(f/step)*step));
    return decimals>0?+(v.toFixed(decimals)):Math.round(v)};
  const fmt=v=>decimals>0?v.toFixed(decimals):String(Math.round(v));
  const wrap=document.createElement("div");wrap.className="nwrap";
  const inp=document.createElement("input");inp.type="number";inp.min=min;inp.max=max;inp.step=step;
  inp.className="numinp";inp.value=fmt(clampF(f0));
  const lab=document.createElement("span");lab.className="unit";lab.textContent=unit||"";
  let cur=clampF(f0);
  const apply=(v,doSend)=>{const cf=clampF(v);cur=cf;inp.value=fmt(cf);oninput(cf,doSend)};
  inp.addEventListener("wheel",e=>{
    e.preventDefault();
    const typed=parseFloat(inp.value);
    const base=isFinite(typed)?typed:cur;
    apply(base+(e.deltaY<0?step:-step),true);
  },{passive:false});
  inp.addEventListener("input",()=>{
    const v=parseFloat(inp.value);
    if(isFinite(v)&&v>=min&&v<=max){cur=clampF(v);oninput(cur,false)}
  });
  inp.addEventListener("change",()=>{
    const v=parseFloat(inp.value);
    apply(isFinite(v)?v:cur,true);
  });
  inp.addEventListener("focus",()=>inp.select());
  wrap.appendChild(inp);wrap.appendChild(lab);
  return{el:wrap,inp};
}
function debounced(fn,ms=250){let t=null;return(...a)=>{clearTimeout(t);t=setTimeout(()=>fn(...a),ms)}}
/* 滑块的已填充进度：只写 --frac（0~1 的进度数），填充宽度由 CSS 的
 * calc(小球半径 + (轨道宽 − 小球宽) × frac) 精确求值 —— 浏览器自己算几何，
 * 不做任何像素测量，天然免疫分辨率/缩放/面板显隐导致的错位。
 * （旧实现用 getBoundingClientRect + 实测小球宽换算成 px，测量时机不对
 *   （面板隐藏、缩放、伪元素实际尺寸与 width 声明不符）时就会和小球对不上。） */
function updFill(r){
  const mn=+r.min,mx=+r.max,v=+r.value;
  if(!isFinite(mn)||!isFinite(mx)||mx<=mn)return;
  const frac=Math.max(0,Math.min(1,(v-mn)/(mx-mn)));
  r.style.setProperty("--frac",frac.toFixed(4));
}
function refreshSliderGeom(){
  document.querySelectorAll('input[type="range"]').forEach(updFill);
}
function bandRow(band,block,ch,idx,kind){
  kind=kind||"eq";
  const pOn=kind==="req"?P_REQ_ON:P_EQ_ON,pF=kind==="req"?P_REQ_FREQ:P_EQ_FREQ,
        pG=kind==="req"?P_REQ_GAIN:P_EQ_GAIN,pQ=kind==="req"?P_REQ_Q:P_EQ_Q,pT=kind==="req"?P_REQ_TYPE:P_EQ_TYPE;
  const row=document.createElement("div");row.className="band eq-band";
  const head=document.createElement("div");head.className="band-head";
  const toggle=tog(band.on,v=>{band.on=v;setParam(block,ch,pOn,idx,v,false);saveLocal();drawResp()});
  head.appendChild(toggle);
  const bi=document.createElement("span");bi.className="bidx";bi.textContent="Band "+(idx+1);
  head.appendChild(bi);row.appendChild(head);
  const sel=document.createElement("select");
  sel.className="eq-type";sel.setAttribute("aria-label","Filter type");
  FNAME.forEach((n,i)=>sel.add(new Option(n,i)));
  sel.value=band.type;
  sel.onchange=()=>{band.type=+sel.value;
    setParam(block,ch,pT,idx,+sel.value,false);saveLocal();drawResp()};
  row.appendChild(sel);
  const fgrp=document.createElement("div");fgrp.className="grp eq-frequency";
  const sendF=debounced(()=>setParam(block,ch,pF,idx,band.f,true));
  const fi=freqInput(band.f,(f,doSend)=>{
    band.f=f;
    if(doSend)sendF();
    saveLocal();drawResp();
  });
  fgrp.appendChild(fi.el);row.appendChild(fgrp);
  const ggrp=document.createElement("div");ggrp.className="grp eq-gain";
  const sendG=debounced(()=>setParam(block,ch,pG,idx,band.g,true));
  const gi=numInput(band.g,-EQ_GAIN_MAX,EQ_GAIN_MAX,0.5,"dB",(v,doSend)=>{
    band.g=v;
    if(doSend)sendG();
    saveLocal();drawResp();
  });
  ggrp.appendChild(gi.el);row.appendChild(ggrp);
  const qgrp=document.createElement("div");qgrp.className="grp eq-q";
  const sendQ=debounced(()=>setParam(block,ch,pQ,idx,band.q,true));
  const qi=numInput(band.q,0.4,10,0.05,"Q",(v,doSend)=>{
    band.q=v;
    if(doSend)sendQ();
    saveLocal();drawResp();
  });
  qgrp.appendChild(qi.el);row.appendChild(qgrp);
  band._ui={fi:fi,gInp:gi.inp,qInp:qi.inp};
  /* 被左右联动镜像改动后，把本行所有控件刷成新值（并同步曲线） */
  band._refresh=()=>{
    try{
      syncBandUI(band);
      if(document.activeElement!==sel)sel.value=band.type;
      const tg=toggle;
      if(tg&&tg.classList&&tg.classList.contains("tog")){
        tg.classList.toggle("on",!!band.on);
        tg.setAttribute("aria-checked",band.on?"true":"false");
      }
      drawResp();
    }catch(e){}
  };
  return row;
}
/* 分频行（通道级 / 端口级复用）。
 * opt（可选）：{block,idx,params:{on,f,type,slope},label}
 *   通道级：默认 block=BLOCK_OUTPUT、ch=通道号、idx=0、P_HP_ / P_LP_ 参数组；
 *   端口级：{block:BLOCK_GLOBAL, idx:端口号, params:P_PORT_HP_ / P_PORT_LP_ 参数组}（ch 传 0）。 */
function xoRow(xo,kind,ch,opt){
  const hp=kind==="hp";
  const o=opt||{};
  const block=(o.block===undefined)?BLOCK_OUTPUT:o.block;
  const idx=(o.idx===undefined)?0:o.idx;
  const xp=o.params||(hp?{on:P_HP_ON,f:P_HP_FREQ,type:P_HP_TYPE,slope:P_HP_SLOPE}
                         :{on:P_LP_ON,f:P_LP_FREQ,type:P_LP_TYPE,slope:P_LP_SLOPE});
  const row=document.createElement("div");row.className="band";
  row.appendChild(tog(xo.on,v=>{xo.on=v;setParam(block,ch,xp.on,idx,v,false);saveLocal();drawResp()}));
  const sel=document.createElement("select");sel.add(new Option("Butterworth",XO_BW));sel.add(new Option("Linkwitz-Riley",XO_LR));
  sel.value=xo.type;sel.onchange=()=>{xo.type=+sel.value;setParam(block,ch,xp.type,idx,+sel.value,false);saveLocal();drawResp()};
  row.appendChild(sel);
  const s2=document.createElement("select");
  s2.add(new Option("12 dB/oct",SLOPE12));s2.add(new Option("24 dB/oct",SLOPE24));s2.add(new Option("48 dB/oct",SLOPE48));
  s2.value=xo.slope;s2.onchange=()=>{xo.slope=+s2.value;setParam(block,ch,xp.slope,idx,+s2.value,false);saveLocal();drawResp()};
  row.appendChild(s2);
  const fgrp=document.createElement("div");fgrp.className="grp";
  const sendF=debounced(()=>setParam(block,ch,xp.f,idx,xo.f,true));
  const fi=freqInput(xo.f,(f,doSend)=>{
    xo.f=f;
    if(doSend)sendF();
    saveLocal();drawResp();
  });
  fgrp.appendChild(fi.el);row.appendChild(fgrp);
  const lab=document.createElement("span");lab.className="v";lab.style.minWidth="0";
  lab.textContent=o.label||(hp?"Channel high-pass":"Channel low-pass");
  row.prepend(lab);
  /* 被左右联动镜像改动后，把本行所有控件刷成新值（开关/类型/斜率/频点） */
  row._refresh=()=>{
    try{
      const tg=row.children[0];
      if(tg&&tg.classList&&tg.classList.contains("tog")){
        tg.classList.toggle("on",!!xo.on);
        tg.setAttribute("aria-checked",xo.on?"true":"false");
      }
      if(document.activeElement!==sel)sel.value=xo.type;
      if(document.activeElement!==s2)s2.value=xo.slope;
      if(document.activeElement!==fi.inp)fi.inp.value=String(Math.round(xo.f));
      drawResp();
    }catch(e){}
  };
  return row;
}
function bandCountRow(bands,rebuild,block,ch,kind){
  const row=document.createElement("div");row.className="band addrow";
  const lab=document.createElement("span");lab.className="bidx";lab.style.minWidth="0";
  lab.textContent="Bands "+bands.length+" / "+MAX_BAND;
  const pCount=kind==="req"?P_REQ_COUNT:P_EQ_COUNT;
  const sendCount=()=>{if(block!==undefined)setParam(block,ch,pCount,0,bands.length,false)};
  const mk=(txt,fn)=>{const b=document.createElement("button");
    b.className="btn";b.style.cssText="padding:3px 12px;font-size:11px;min-height:30px";
    b.textContent=txt;b.onclick=fn;return b};
  row.appendChild(lab);
  row.appendChild(mk("Add band",()=>{
    if(bands.length>=MAX_BAND){log("Limit reached: "+MAX_BAND+" bands","warn");return}
    /* 单组段数没超，但**设备总预算**可能已经到顶 —— 先算，再让设备裁决 */
    if(!budgetGuard(1))return;
    bands.push(newBand(bands.length));sendCount();saveLocal();rebuild();drawResp();
  }));
  row.appendChild(mk("Remove last band",()=>{
    if(bands.length<=1){log("Keep at least one band.","warn");return}
    bands.pop();sendCount();saveLocal();rebuild();drawResp();
  }));
  /* 小字：这一组之外还有多少预算被其它组占着（每加一段 = 1 个双二阶） */
  const mini=document.createElement("span");
  mini.className="tip";mini.style.cssText="font-size:10px;color:var(--dim);margin-left:6px;white-space:nowrap";
  mini.textContent="One biquad per band. "+budgetMiniText();
  row.appendChild(mini);
  return row;
}
/* 输入端卡片：标题彩色竖条 + 增益/静音/反相 + PEQ/房间 EQ，直接排布，无多余装饰。 */
function inCard(inp,ch,title,color){
  const card=document.createElement("div");
  card.className="card src-card";
  card.style.setProperty("--sc",color);

  const head=document.createElement("div");head.className="src-head";
  const h=document.createElement("h4");
  h.innerHTML='<span class="bar" style="background:'+color+';color:'+color+'"></span>'+title;
  head.appendChild(h);
  card.appendChild(head);

  const r1=document.createElement("div");r1.className="rbar tight";
  r1.appendChild(Object.assign(document.createElement("label"),{textContent:"Gain, dB"}));
  const g=document.createElement("input");g.type="range";g.min=-72;g.max=12;g.step=0.5;g.value=inp.gain;
  const gv=document.createElement("span");gv.className="val";gv.textContent=(inp.gain>0?"+":"")+inp.gain+" dB";
  g.oninput=()=>{inp.gain=+g.value;gv.textContent=(inp.gain>0?"+":"")+inp.gain+" dB";
    setParam(BLOCK_INPUT,ch,P_GAIN,0,inp.gain,true);saveLocal()};
  r1.appendChild(g);r1.appendChild(gv);card.appendChild(r1);

  const r2=document.createElement("div");r2.className="rbar tight";
  r2.appendChild(Object.assign(document.createElement("label"),{textContent:"Mute"}));
  r2.appendChild(tog(inp.mute,v=>{inp.mute=v;setParam(BLOCK_INPUT,ch,P_MUTE,0,v,false);saveLocal()}));
  r2.appendChild(Object.assign(document.createElement("label"),{textContent:"Invert phase"}));
  r2.appendChild(tog(inp.phase,v=>{inp.phase=v;setParam(BLOCK_INPUT,ch,P_PHASE,0,v,false);saveLocal()}));
  card.appendChild(r2);

  const d1=document.createElement("details");d1.className="eqbox";d1.open=true;
  d1.innerHTML="<summary>Parametric EQ ("+inp.eq.length+" bands; maximum "+MAX_BAND+")</summary>";
  const s1=document.createElement("div");s1.className="eqscroll";
  inp.eq.forEach((b,i)=>s1.appendChild(bandRow(b,BLOCK_INPUT,ch,i,"eq")));
  d1.appendChild(s1);d1.appendChild(bandCountRow(inp.eq,buildInputs,BLOCK_INPUT,ch,"eq"));card.appendChild(d1);
  const d2=document.createElement("details");d2.className="eqbox";
  d2.innerHTML="<summary>Room EQ ("+inp.req.length+" bands; maximum "+MAX_BAND+")</summary>";
  const s2=document.createElement("div");s2.className="eqscroll";
  inp.req.forEach((b,i)=>s2.appendChild(bandRow(b,BLOCK_INPUT,ch,i,"req")));
  d2.appendChild(s2);d2.appendChild(bandCountRow(inp.req,buildInputs,BLOCK_INPUT,ch,"req"));card.appendChild(d2);
  return card;
}
function buildInputs(){
  const c0=$("in0"),c1=$("in1");c0.innerHTML="";c1.innerHTML="";
  c0.appendChild(inCard(cfg.inputs[0],0,"USB input","var(--usb)"));
  c1.appendChild(inCard(cfg.inputs[1],1,"Bluetooth input","var(--bt)"));
}
function dspCard(out,ch,color){
  const card=document.createElement("div");card.className="card";
  /* 统一命名：dspN-端口i2sM-左/右声道（见 DSP_LONG） */
  const h=document.createElement("h4");h.innerHTML='<span class="bar" style="background:'+color+';color:'+color+'"></span>'+(DSP_LONG[ch]||("dsp"+(ch+1)));
  card.appendChild(h);
  const d=document.createElement("details");d.className="eqbox";d.open=true;
  d.innerHTML="<summary>Output parametric EQ ("+out.eq.length+" bands; maximum "+MAX_BAND+")</summary>";
  const s=document.createElement("div");s.className="eqscroll";
  out.eq.forEach((b,i)=>s.appendChild(bandRow(b,BLOCK_OUTPUT,ch,i)));
  d.appendChild(s);d.appendChild(bandCountRow(out.eq,buildDsp,BLOCK_OUTPUT,ch,"eq"));card.appendChild(d);
  /* 通道级高通/低通：整通道限带/保护用；做分频请去「输出端口」卡。默认收起，避免和端口滤波混淆。 */
  const xd=document.createElement("details");xd.className="eqbox";
  xd.innerHTML="<summary>Channel high-pass and low-pass</summary>";
  const hpRow=xoRow(out.hp,"hp",ch);
  const lpRow=xoRow(out.lp,"lp",ch);
  xd.appendChild(hpRow);
  xd.appendChild(lpRow);
  const tip=document.createElement("div");tip.className="rbar tight";
  const tipS=document.createElement("span");tipS.className="tip";
  tipS.textContent="These filters apply to this DSP channel only."+
    "For a crossover, set low-pass filters on one port and matching high-pass filters on the other port.";
  tip.appendChild(tipS);xd.appendChild(tip);
  card.appendChild(xd);
  /* 左右联动镜像后，把对侧通道卡的控件刷成新值（开关/类型/斜率/频点） */
  out._uiRefresh=[()=>{try{hpRow._refresh&&hpRow._refresh()}catch(e){}},
                  ()=>{try{lpRow._refresh&&lpRow._refresh()}catch(e){}}];
  /* dsp3/dsp4（I2S1）已启用；lock 分支保留给未来不可用通道。 */
  if(dspDisabled(ch)){
    card.classList.add("lock");
    card.querySelectorAll("input,select,button").forEach(e=>{e.disabled=true});
    card.querySelectorAll(".tog").forEach(e=>{
      e.onclick=null;e.style.pointerEvents="none";e.style.opacity=".45";e.removeAttribute("role")});
  }
  return card;
}
function buildDsp(){
  const colors=["var(--ch1)","var(--ch2)","var(--ch3)","var(--ch4)"];
  cfg.outs.forEach((o,i)=>{const c=$("xoOut"+i);if(!c)return;c.innerHTML="";c.appendChild(dspCard(o,i,colors[i]))});
}
/* 路由矩阵：两路源各一条通道条（徽标 + 表头 dsp1~dsp4 + 每格开关/增益/源声道），
 * 与固件 route[src][ch] 一一对应。 */
function buildRoute(){
  const c=$("routeMx");c.innerHTML="";
  const srcNames=["USB","Bluetooth"];
  const srcColors=["var(--usb)","var(--bt)"];

  for(let i=0;i<2;i++){
    const box=document.createElement("div");box.className="mx-block";
    const cap=document.createElement("div");cap.className="mx-cap";
    const badge=document.createElement("span");badge.className="mx-badge";
    badge.style.setProperty("--sc",srcColors[i]);
    badge.innerHTML='<i class="mx-led"></i>'+srcNames[i]+" →";
    cap.appendChild(badge);
    const capTxt=document.createElement("span");capTxt.className="mx-captxt";
    capTxt.textContent="2 inputs / 4 channels";
    cap.appendChild(capTxt);
    box.appendChild(cap);

    /* 每列一个「表头 + 数据格」的纵向容器：桌面 4 列时表头成行，
     * 窄屏折成 2 列时表头依然正好压在自己格子的上方 */
    const grid=document.createElement("div");grid.className="mx-grid";
    for(let o=0;o<4;o++){
      const col=document.createElement("div");col.className="mx-col";
      const hd=document.createElement("div");hd.className="mx-cell mx-head";
      hd.innerHTML='<b class="mx-th">'+DSP_SHORT[o]+'</b><small class="mx-thsub">'+DSP_MINI[o].replace(/^dsp\d-端口/,"Port")+'</small>';
      col.appendChild(hd);

      const cell=cfg.route[i][o];
      const d=document.createElement("div");d.className="mx-cell";
      const sw=document.createElement("div");sw.className="sw "+(cell.on?"on":"off");
      sw.textContent=cell.on?"✓":"✗";
      sw.onclick=()=>{cell.on=cell.on?0:1;sw.className="sw "+(cell.on?"on":"off");
        sw.textContent=cell.on?"✓":"✗";
        g.style.opacity=cell.on?1:.35;
        if(ss)ss.disabled=!cell.on;
        d.classList.toggle("off",!cell.on);
        setParam(BLOCK_ROUTING,i,P_ROUTE_ON,o,cell.on,false);saveLocal();drawResp()};
      d.appendChild(sw);
      const g=document.createElement("input");g.type="range";g.min=0;g.max=100;g.step=1;g.value=Math.round(cell.gain*100);
      const gv=document.createElement("div");gv.className="gv";gv.textContent=g.value+"%";
      g.oninput=()=>{cell.gain=+g.value/100;gv.textContent=g.value+"%";
        setParam(BLOCK_ROUTING,i,P_ROUTE_GAIN,o,cell.gain,true);saveLocal()};
      g.onclick=e=>e.stopPropagation();
      d.appendChild(g);d.appendChild(gv);
      /* 源声道选择：默认（AUTO）= dsp1/dsp3←源L、dsp2/dsp4←源R（**输入侧**）；
       * 输出侧固定为 dsp1→i2s0左槽、dsp2→i2s0右槽（固件 dsp_engine.c:762-772，硬编码）。
       * 两者合起来才是"左→dsp1→i2s0左"这条直通链路；也可 L/R 互换、单声道 (L+R)/2。
       * （下拉项的文案见 ROUTE_SRC_NAME —— 那里把输入/输出两侧一起说清了，避免被读成"输出侧也交叉"。） */
      const ss=document.createElement("select");ss.className="rsrc";
      ROUTE_SRC_NAME.forEach((n,k)=>ss.add(new Option(n,k)));
      ss.value=cell.src||0;
      ss.disabled=!cell.on;
      ss.onclick=e=>e.stopPropagation();
      ss.onchange=()=>{cell.src=+ss.value;
        setParam(BLOCK_ROUTING,i,P_ROUTE_SRC,o,cell.src,false);saveLocal();drawResp()};
      d.appendChild(ss);
      if(!cell.on){g.style.opacity=.35;d.classList.add("off")}
      /* dsp3/dsp4（端口 i2s1）已启用，不再灰显。 */
      col.appendChild(d);
      grid.appendChild(col);
    }
    box.appendChild(grid);
    c.appendChild(box);
  }
}
/* 端口不再做高通/低通：分频在「处理区 · DSP」的 4 条通道上做（dsp1/dsp2 = 端口0，dsp3/dsp4 = 端口1）。 */
function updateOutStat(){
  const e=$("stOut");if(!e)return;
  const a=!!(cfg.ports[0]&&cfg.ports[0].enabled), b=!!(cfg.ports[1]&&cfg.ports[1].enabled);
  e.textContent=(a&&b)?"Dual I2S":(a?"I2S0":(b?"I2S1":"Off"));
  e.title=(a?"I2S0 on":"I2S0 off")+" · "+(b?"I2S1 on":"I2S1 off");
}
function buildPorts(){
  const c=$("portCards");c.innerHTML="";
  PORTS.forEach((p,i)=>{
    const pm=cfg.ports[i];
    const avail=!p.disabled;
    if(!avail&&pm.enabled){pm.enabled=0}          /* 不可用端口强制关闭 */
    const en=avail&&!!pm.enabled;
    const card=document.createElement("div");
    card.className="port"+(en?"":" lock")+($("respSel")&&$("respSel").value==="out"+i?" sel":"");
    card.style.setProperty("--pc",p.color);
    const h=document.createElement("h4");
    h.innerHTML='<span class="bar" style="background:'+p.color+';color:'+p.color+'"></span>'+p.name;
    h.style.cursor="pointer";h.title="Show channel response";
    h.onclick=()=>showResp("out"+i);
    card.appendChild(h);
    const desc=document.createElement("div");desc.className="pdesc";
    desc.innerHTML=p.desc+(avail?(en?"":" / off"):" / unavailable");
    card.appendChild(desc);
    const re=document.createElement("div");re.className="rbar tight";
    re.appendChild(Object.assign(document.createElement("label"),{textContent:"On"}));
    if(avail){
      re.appendChild(tog(en,v=>{
        cfg.ports[i].enabled=v;
        setParam(BLOCK_GLOBAL,0,P_PORT_ENABLE,i,v,false);
        saveLocal();buildPorts();
      }));
      const t=document.createElement("span");t.className="tip";
      t.textContent="Set the output port on or off. Off sends silence.";
      re.appendChild(t);
    }else{
      const t=document.createElement("span");t.className="tip";
      t.textContent="Port not supported";
      re.appendChild(t);
    }
    card.appendChild(re);
    const rb=document.createElement("div");rb.className="rbar tight";
    rb.appendChild(Object.assign(document.createElement("label"),{textContent:"Channel mode"}));
    const sel=document.createElement("select");
    PORT_MODE_NAME.forEach((n,k)=>sel.add(new Option(n,k)));
    sel.value=pm.mode;sel.disabled=(!avail)||(!en);
    sel.onchange=()=>{if(!avail)return;pm.mode=+sel.value;
      setParam(BLOCK_GLOBAL,0,P_PORT_MODE,i,+sel.value,false);saveLocal()};
    rb.appendChild(sel);card.appendChild(rb);
    const rf=document.createElement("div");rf.className="rbar tight";
    rf.appendChild(Object.assign(document.createElement("label"),{textContent:"I2S format"}));
    const fsel=document.createElement("select");
    PORT_FMT_NAME.forEach((n,k)=>{
      const o=new Option(n,k);if(i===1&&k===2){o.disabled=true;o.text+=" (not supported)";}fsel.add(o);
    });
    fsel.value=pm.format;fsel.disabled=(!avail)||(!en);
    fsel.onchange=()=>{if(!avail)return;pm.format=+fsel.value;
      setParam(BLOCK_GLOBAL,0,P_PORT_FORMAT,i,+fsel.value,false);saveLocal()};
    rf.appendChild(fsel);card.appendChild(rf);
    const rg=document.createElement("div");rg.className="rbar tight";
    rg.appendChild(Object.assign(document.createElement("label"),{textContent:"Output gain"}));
    const g=document.createElement("input");g.type="range";g.min=-24;g.max=12;g.step=0.5;g.value=pm.gain;
    g.disabled=(!avail)||(!en);
    const gv=document.createElement("span");gv.className="val";gv.textContent=(pm.gain>0?"+":"")+pm.gain+" dB";
    g.oninput=()=>{if(!avail)return;pm.gain=+g.value;gv.textContent=(pm.gain>0?"+":"")+pm.gain+" dB";
      setParam(BLOCK_GLOBAL,0,P_PORT_GAIN,i,pm.gain,true);saveLocal()};
    rg.appendChild(g);rg.appendChild(gv);card.appendChild(rg);
    const rd=document.createElement("div");rd.className="rbar tight";
    rd.appendChild(Object.assign(document.createElement("label"),{textContent:"Delay"}));
    const d=document.createElement("input");d.type="range";d.min=0;d.max=20;d.step=0.1;d.value=pm.delay;
    d.disabled=(!avail)||(!en);
    const dv=document.createElement("span");dv.className="val";dv.textContent=pm.delay.toFixed(1)+" ms";
    d.oninput=()=>{if(!avail)return;pm.delay=Math.round(+d.value*10)/10;dv.textContent=pm.delay.toFixed(1)+" ms";
      setParam(BLOCK_GLOBAL,0,P_PORT_DELAY,i,pm.delay,true);saveLocal()};
    rd.appendChild(d);rd.appendChild(dv);card.appendChild(rd);
    /* 通道归属按用户要求不再在端口卡上重复显示（卡标题 + 表格已说明 dspN 与端口的关系），
     * 只留一句「高通/低通去哪调」的指引。 */
    const rt2=document.createElement("div");rt2.className="rbar tight";
    const rtS=document.createElement("span");rtS.className="tip";
    rtS.textContent="Set high-pass and low-pass filters in DSP channels "+(2*i+1)+" / dsp"+(2*i+2)+".";
    rt2.appendChild(rtS);card.appendChild(rt2);
    c.appendChild(card);
  });
  updateOutStat();
  c.querySelectorAll("input[type=range]").forEach(r=>{
    updFill(r);
    if(!r._fillBound){
      r._fillBound=1;
      r.addEventListener("input",()=>updFill(r));
      r.addEventListener("wheel",e=>e.preventDefault(),{passive:false});
    }
  });
}
/* 精度模式对用户已隐式（v28 去掉面板）：`syncCompUI()` 自己维护开关/说明，
 * 不再有独立的精度面板同步步骤。 */
function rebuildUI(){buildInputs();buildDsp();buildRoute();buildPorts();buildFlow();buildLinkUI();syncLimUI();syncLoudUI();syncCompUI();syncHwUI();updPresetUI();
  $("masterVol").value=Math.round(cfg.master*100);$("masterVolv").textContent=masterLabel(cfg.master);
  document.querySelectorAll("input[type=range]").forEach(r=>{
    updFill(r);
    if(!r._fillBound){
      r._fillBound=1;
      r.addEventListener("input",()=>updFill(r));
      r.addEventListener("wheel",e=>e.preventDefault(),{passive:false});
    }
  });
  showPanel(curPanel,false);
  drawResp()}
/* ================= 频响曲线（RBJ biquad · 与固件系数一致） ================= */
function coeff(band,fs){
  const A=Math.pow(10,band.g/40),w0=2*Math.PI*band.f/fs,cw=Math.cos(w0),sw=Math.sin(w0),Q=Math.max(band.q,0.01),alpha=sw/(2*Q);
  let b0,b1,b2,a0,a1,a2;
  switch(band.type){
    case F_PEAK:b0=1+alpha*A;b1=-2*cw;b2=1-alpha*A;a0=1+alpha/A;a1=-2*cw;a2=1-alpha/A;break;
    case F_LSHELF:{const beta=2*Math.sqrt(A)*alpha;
      b0=A*((A+1)-(A-1)*cw+beta);b1=2*A*((A-1)-(A+1)*cw);b2=A*((A+1)-(A-1)*cw-beta);
      a0=(A+1)+(A-1)*cw+beta;a1=-2*((A-1)+(A+1)*cw);a2=(A+1)+(A-1)*cw-beta;break}
    case F_HSHELF:{const beta=2*Math.sqrt(A)*alpha;
      b0=A*((A+1)+(A-1)*cw+beta);b1=-2*A*((A-1)+(A+1)*cw);b2=A*((A+1)+(A-1)*cw-beta);
      a0=(A+1)-(A-1)*cw+beta;a1=2*((A-1)-(A+1)*cw);a2=(A+1)-(A-1)*cw-beta;break}
    case F_LP:b0=(1-cw)/2;b1=1-cw;b2=(1-cw)/2;a0=1+alpha;a1=-2*cw;a2=1-alpha;break;
    case F_HP:b0=(1+cw)/2;b1=-(1+cw);b2=(1+cw)/2;a0=1+alpha;a1=-2*cw;a2=1-alpha;break;
    case F_NOTCH:b0=1;b1=-2*cw;b2=1;a0=1+alpha;a1=-2*cw;a2=1-alpha;break;
    case F_BP:b0=alpha;b1=0;b2=-alpha;a0=1+alpha;a1=-2*cw;a2=1-alpha;break;
    case F_AP:b0=1-alpha;b1=-2*cw;b2=1+alpha;a0=1+alpha;a1=-2*cw;a2=1-alpha;break;
    default:return null;
  }
  return{b0:b0/a0,b1:b1/a0,b2:b2/a0,a1:a1/a0,a2:a2/a0};
}
function mag2(co,w){
  const c=Math.cos(w),c2=Math.cos(2*w);
  const num=co.b0*co.b0+co.b1*co.b1+co.b2*co.b2
           +2*(co.b0*co.b1+co.b1*co.b2)*c+2*co.b0*co.b2*c2;
  const den=1+co.a1*co.a1+co.a2*co.a2
           +2*(co.a1+co.a1*co.a2)*c+2*co.a2*c2;
  return den>0?num/den:1;
}
function butterQ(order,stage){return 1/(2*Math.cos(Math.PI*(2*stage+1)/(2*order)))}
function xoStageList(xo,isHp){
  if(!xo.on)return[];
  const ft=isHp?F_HP:F_LP;
  const out=[];
  if(xo.type===XO_BW){
    const order=xo.slope*2;
    for(let i=0;i<xo.slope;i++)out.push({on:1,type:ft,f:xo.f,g:0,q:butterQ(order,i)});
  }else{
    if(xo.slope===1){out.push({on:1,type:ft,f:xo.f,g:0,q:0.5});}
    else{
      const bwOrder=xo.slope;
      const n=Math.max(1,xo.slope/2);
      const qs=[];for(let i=0;i<n;i++)qs.push(butterQ(bwOrder,i));
      for(let i=0;i<n;i++)out.push({on:1,type:ft,f:xo.f,g:0,q:qs[i]});
      for(let i=0;i<n;i++)out.push({on:1,type:ft,f:xo.f,g:0,q:qs[i]});
    }
  }
  return out;
}
let handles=[];
let CW=880,CH=240;
function fitCanvas(){
  const cv=$("respCv");if(!cv)return;
  const dpr=window.devicePixelRatio||1;
  cv.style.display="block";cv.style.width="100%";cv.style.height="auto";
  const r=cv.getBoundingClientRect();
  CW=Math.max(300,Math.min(1280,Math.round(r.width||880)));
  CH=CW<560?Math.round(Math.min(320,Math.max(200,CW*0.62)))
           :Math.round(Math.min(320,Math.max(240,CW*0.28)));
  cv.style.width=CW+"px";cv.style.height=CH+"px";
  cv.width=Math.round(CW*dpr);cv.height=Math.round(CH*dpr);
  cv.getContext("2d").setTransform(dpr,0,0,dpr,0,0);
}
function respLayout(){return{fmin:20,fmax:20000,top:22,bot:CH-22,left:50,right:CW-14}}
function chainAt(chain,fs,f){
  const w0=2*Math.PI*f/fs;let m=1;
  for(const b of chain){if(!b.on)continue;const c=coeff(b,fs);if(!c)continue;m*=mag2(c,w0)}
  return 10*Math.log10(m);
}
let gView="mag";
let aYMin=-15,aYMax=15;              /* ★ v34：幅频固定 ±15 dB（0 dB 居中），见 drawResp 里的说明 */
const N_RESP=480;
const AXIS_DB=15;                    /* 幅频视图的对称半量程（±AXIS_DB）—— 纯**显示**范围 */
/* ⚠ **不要**把 PEQ 单段增益上限跟着 AXIS_DB 一起改成 15（v34 差点犯的错）：
 *   固件写死 `DSP_BAND_GAIN_MIN_DB=-12 / MAX_DB=12`（`dsp_config.h:46-47`，
 *   且 `dsp_config.c:178/910`、`dsp_filter.c:33` 三处都按它夹取），
 *   前端若按 ±15 下发，设备会**静默夹回 ±12** —— 正是本项目反复强调的
 *   "前端说一套、设备是另一套"。前端**必须**与固件一致：±12。
 *   （视图能画到 ±15 是留了余量：响度补偿/多段叠加可以超过单段 ±12。） */
const EQ_GAIN_MAX=12;
const GRID_DB_STEP=5;                /* ±15 下取 5 ⇒ 格线 -15/-10/-5/0/+5/+10/+15，共 7 条 */
const DB_STEP=6;
const PHASE_MIN=-180,PHASE_MAX=180;
function bqPhase(co,w){
  const c1=Math.cos(w),s1=Math.sin(w),c2=Math.cos(2*w),s2=Math.sin(2*w);
  const nr=co.b0+co.b1*c1+co.b2*c2, ni=-(co.b1*s1+co.b2*s2);
  const dr=1+co.a1*c1+co.a2*c2,      di=-(co.a1*s1+co.a2*s2);
  return Math.atan2(ni*dr-nr*di, nr*dr+ni*di);
}
function chainPhase(chain,fs,f){
  const w0=2*Math.PI*f/fs; let p=0;
  for(const b of chain){if(!b.on)continue;const c=coeff(b,fs);if(!c)continue;p+=bqPhase(c,w0)}
  return p*180/Math.PI;
}
function viewVal(v,fs,f){return gView==="phase"?chainPhase(v.chain,fs,f):(chainAt(v.chain,fs,f)+(v.offset||0))}
/* 响度补偿当前应补偿的 dB（与固件 loudness_amount() 逐式一致，纯前端预览用）。
 * 主音量从参考点降到最低（-12 dB）时，从 0 线性升到 amount；参考点及以上为 0。 */
function loudBoostDb(){
  const L=cfg.loud;
  if(!L||!L.on||L.amount<=0)return 0;
  const db0=v=>{if(v<=0)return -120;let d=20*Math.log10(v);return Math.max(-12,Math.min(12,d))};
  const ref=Math.max(0.05,Math.min(1,L.ref));
  const span=db0(ref)+12;
  if(span<=0.01)return 0;
  let k=(db0(ref)-db0(cfg.master))/span;
  if(k<=0)return 0;if(k>1)k=1;
  return L.amount*k;
}
/* 响度补偿的两个隐藏架式滤波器（参数与固件 DSP_LOUD_* 一致） */
function loudStages(){
  const g=loudBoostDb();
  if(g<=0.001)return [];
  return [{on:1,type:F_LSHELF,f:LOUD_LO_HZ,g:g,q:LOUD_Q},
          {on:1,type:F_HSHELF,f:LOUD_HI_HZ,g:g,q:LOUD_Q}];
}
function chainView(sel){
  const P=pal();
  if(sel==="in0"||sel==="in1"){
    const i=+sel[2],inp=cfg.inputs[i];
    return{label:["USB","Bluetooth"][i],color:P.inp[i],offset:0,
      chain:[...inp.req,...inp.eq],
      bands:[...inp.req.map((b,idx)=>({band:b,idx,kind:"req",block:BLOCK_INPUT,ch:i})),
             ...inp.eq.map((b,idx)=>({band:b,idx,kind:"eq",block:BLOCK_INPUT,ch:i}))]};
  }
  const pm=sel.match(/^p([01])([LR])$/);
  if(pm){
    /* 端口视图 = 该槽对应的通道全链（端口映射后就是这条通道）+ 端口增益。
     * 端口曲线用独立的端口色并画成虚线：和它叠加的 dsp 曲线（同一条链）区分开。 */
    const pi=+pm[1],slotL=pm[2]==="L";
    const port=cfg.ports[pi],mode=port.mode||0;
    let ci=slotL?2*pi:2*pi+1;                  /* 立体声：左槽 CH(2p+1)、右槽 CH(2p+2) */
    if(mode===1)ci=2*pi;                       /* 仅左：两槽都是左通道 */
    if(mode===2)ci=2*pi+1;                     /* 仅右：两槽都是右通道 */
    const chn=cfg.outs[ci]||cfg.outs[0];
    return{label:"Port "+pi+(slotL?" left":" right"),color:P.port[pi],offset:(port.enabled?port.gain:0),
      chain:[...chn.eq,...xoStageList(chn.hp,true),...xoStageList(chn.lp,false),...loudStages()],
      bands:chn.eq.map((b,idx)=>({band:b,idx,kind:"eq",block:BLOCK_OUTPUT,ch:ci}))};
  }
  const o=Math.min(3,Math.max(0,+sel[3]||0)),out=cfg.outs[o];
  return{label:DSP_SHORT[o],color:P.out[o],offset:0,
    chain:[...out.eq,...xoStageList(out.hp,true),...xoStageList(out.lp,false),...loudStages()],
    bands:out.eq.map((b,idx)=>({band:b,idx,kind:"eq",block:BLOCK_OUTPUT,ch:o}))};
}
function rr(ctx,x,y,w,h,r){ctx.beginPath();ctx.moveTo(x+r,y);
  ctx.arcTo(x+w,y,x+w,y+h,r);ctx.arcTo(x+w,y+h,x,y+h,r);
  ctx.arcTo(x,y+h,x,y,r);ctx.arcTo(x,y,x+w,y,r);ctx.closePath()}
function drawResp(){
  const cv=$("respCv"),ctx=cv.getContext("2d");
  /* ⚠ 这里是 **PEQ 单段增益的合法范围**，必须与固件一致：**±12**
   *   （固件 `dsp_config.h` 的 `DSP_BAND_GAIN_MIN_DB/MAX_DB`，三处夹取都按它）。
   *   它与上面的 `AXIS_DB=15`（**视图量程**）是两件事，别再混：
   *   视图画到 ±15 是为了给响度/多段叠加留余量，而**单段**只能调到 ±12。
   *   若把这里也改成 15，设备会把 ±13..15 静默夹回 ±12（前端说一套、设备另一套）。 */
  const EQ_LIMIT_LO=-EQ_GAIN_MAX,EQ_LIMIT_HI=EQ_GAIN_MAX;
    let overMax=false,underMin=false,rawMax=-Infinity,rawMin=Infinity;
    const sel=$("respSel").value,fs=gFs;
  const isL=gTheme==="light";
  const L=respLayout(),fmin=L.fmin,fmax=L.fmax,top=L.top,bot=L.bot,left=L.left,right=L.right;
  const X=f=>left+(Math.log(f/fmin)/Math.log(fmax/fmin))*(right-left);
  let views=[];
  if(sel==="all"){for(const s of["in0","in1","out0","out1","out2","out3","p0L","p0R","p1L","p1R"]){const v=chainView(s);if(v)views.push(v)}}
  else{const v=chainView(sel);if(v)views.push(v)}
  handles=[];
  if(gView==="mag"){
    let mn=Infinity,mx=-Infinity;
    for(const v of views)for(let i=0;i<=N_RESP;i++){
      const f=fmin*Math.pow(fmax/fmin,i/N_RESP);
      const raw=chainAt(v.chain,fs,f)+(v.offset||0);
      const g=Math.max(-80,Math.min(24,raw));
      if(raw>EQ_LIMIT_HI){overMax=true;if(raw>rawMax)rawMax=raw;}
      if(raw<EQ_LIMIT_LO){underMin=true;if(raw<rawMin)rawMin=raw;}
      if(g<mn)mn=g;if(g>mx)mx=g;
    }
    /* ★ v34（用户要求）：幅频视图用**固定对称量程 ±15 dB**，让 0 dB 线**水平居中**。
     * 原来是"按数据自动缩放 + 最少 5 格 × DB_STEP=6"的启发式：
     *   数据只在 0 附近时算出 lo=-18 / hi=+12（不对称），0 dB 线被压在偏上位置；
     *   曲线一变，量程还会跟着跳，读图时没有稳定参照。
     * 现在固定 [-15,+15]：0 dB 正好在画布垂直中点，格线用 5 dB（-15/-10/-5/0/5/10/15），
     * 与"PEQ 允许摆到 ±15"的上限也对得上（用户要求 ±15）。 */
    aYMin=-AXIS_DB;aYMax=AXIS_DB;
  } else {aYMin=PHASE_MIN;aYMax=PHASE_MAX;}
  const Y=g=>top+(aYMax-g)/(aYMax-aYMin)*(bot-top);
  const bg=ctx.createLinearGradient(0,0,0,CH);
  bg.addColorStop(0,cssv("--chart-bg0",isL?"#fcfdfb":"#121c1e"));bg.addColorStop(1,cssv("--chart-bg1",isL?"#f5f8f4":"#0f181a"));
  ctx.fillStyle=bg;ctx.fillRect(0,0,CW,CH);
  ctx.textAlign="left";ctx.textBaseline="alphabetic";
  ctx.strokeStyle=cssv("--chart-grid",isL?"rgba(28,47,42,.08)":"rgba(170,214,196,.075)");
  ctx.fillStyle=cssv("--chart-label",isL?"#61766e":"#8fa39d");
  ctx.font="11px sans-serif";ctx.lineWidth=1;
  /* 网格步长：相位固定 45°，幅频用 GRID_DB_STEP=5（配合固定 ±15 量程 ⇒ 7 条对称格线，
   * 0 dB 落在正中）。原来复用 DB_STEP=6，在 ±15 上会得到 ±12 那种不对称的端点值。 */
  const hstep=gView==="phase"?45:GRID_DB_STEP;
  for(let g=aYMin;g<=aYMax+0.01;g+=hstep){
    const y=Math.round(Y(g))+0.5;
    ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();
    const lbl=gView==="phase"?(g===0?"0°":(g>0?"+":"")+g+"°"):g+"dB";
    /* 端点标签别画出画布：±15 那两条正好落在 top/bot 上，+3.5 的基线偏移会把字顶到画布外
     * 被裁掉（v34 实测："15dB" 只剩一半）。夹到 [12, CH-3] 之内。 */
    ctx.fillText(lbl,5,Math.max(12,Math.min(CH-3,Y(g)+3.5)));
  }
  for(const f of[20,50,100,200,500,1000,2000,5000,10000,20000]){
    if(f<fmin||f>fmax)continue;
    const x=Math.round(X(f))+0.5;
    ctx.beginPath();ctx.moveTo(x,top);ctx.lineTo(x,bot);ctx.stroke();
    ctx.fillText(f>=1000?(f/1000)+"k":f,X(f)-7,CH-6);
  }
  if(gView==="mag"){ctx.strokeStyle=cssv("--chart-zero",isL?"rgba(28,47,42,.28)":"rgba(170,214,196,.26)");ctx.lineWidth=1;ctx.beginPath();
    ctx.moveTo(left,Math.round(Y(0))+0.5);ctx.lineTo(right,Math.round(Y(0))+0.5);ctx.stroke();}
    if(gView==="mag"){
      ctx.save();ctx.strokeStyle=isL?"rgba(178,72,56,.4)":"rgba(240,154,143,.42)";ctx.lineWidth=1;ctx.setLineDash([4,5]);
      for(const gl of[EQ_LIMIT_LO,EQ_LIMIT_HI]){
        if(gl>=aYMin&&gl<=aYMax){ctx.beginPath();ctx.moveTo(left,Math.round(Y(gl))+0.5);ctx.lineTo(right,Math.round(Y(gl))+0.5);ctx.stroke();}
      }
      ctx.restore();
    }
  ctx.lineWidth=2.2;ctx.lineJoin="round";ctx.lineCap="round";const n=N_RESP;
  views.forEach(v=>{
    ctx.strokeStyle=v.color;
    ctx.beginPath();
    for(let i=0;i<=n;i++){
      const f=fmin*Math.pow(fmax/fmin,i/n);
      let g=viewVal(v,fs,f);
      if(gView==="phase")g=((g+180)%360+360)%360-180;
      g=Math.max(aYMin,Math.min(aYMax,g));
      const x=X(f),y=Y(g);
      i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
    }
    ctx.stroke();
      if(gView==="mag"){
        ctx.save();
        ctx.strokeStyle=cssv("--err","#f09a8f");ctx.lineWidth=3;
        let started=false;
        for(let i=0;i<=n;i++){
          const f=fmin*Math.pow(fmax/fmin,i/n);
          const raw=chainAt(v.chain,fs,f)+(v.offset||0);
          const over=(raw>EQ_LIMIT_HI)||(raw<EQ_LIMIT_LO);
          const g=Math.max(aYMin,Math.min(aYMax,raw));
          const x=X(f),y=Y(g);
          if(over){
            if(!started){ctx.beginPath();ctx.moveTo(x,y);started=true;}
            else ctx.lineTo(x,y);
          }else if(started){ctx.stroke();started=false;}
        }
        if(started)ctx.stroke();
        ctx.restore();
      }
  });
    const warnEl=$("respLimitWarn");
    if(warnEl){
      if(gView==="mag"&&(overMax||underMin)){
        /* ⚠ 数字统一读 EQ_LIMIT_*（= ±EQ_GAIN_MAX = ±12，与固件一致），别写死：
         * v34 把**视图量程**改成 ±15 时，若这里跟着写 ±15，
         * 就会把"单段到 ±12 上限"的曲线误报成越界。视图量程与 PEQ 上限是两件事。 */
        let msg="Response exceeds +/-"+EQ_LIMIT_HI+" dB";
        if(overMax)msg+=". Maximum "+rawMax.toFixed(1)+" dB. Digital clipping is possible.";
        if(underMin)msg+=". Minimum "+rawMin.toFixed(1)+" dB";
        warnEl.textContent=msg;
        warnEl.style.display="block";
      }else{
        warnEl.style.display="none";
      }
    }
  if(sel==="all"){
    ctx.font="bold 10px sans-serif";ctx.textAlign="left";
    let lx=left+4,ly=14;
    views.forEach(v=>{
      const w=ctx.measureText(S31I18N.t(v.label)).width;
      if(lx+w>right){lx=left+4;ly+=13}           /* 图例放不下就换一行 */
      ctx.fillStyle=v.color;ctx.fillText(S31I18N.t(v.label),lx,ly);lx+=w+14});
  }
  if(gView==="mag"){
    views.forEach(v=>v.bands.forEach(b=>{if(b.band.on)handles.push({...b,color:v.color})}));
    ctx.textAlign="center";
    handles.forEach((h,i)=>{
      const hx=Math.max(left+3,Math.min(right-3,X(h.band.f)));
      const hy=Math.max(top+3,Math.min(bot-3,Y(h.band.g)));
      h.x=hx;h.y=hy;
      const active=drag&&drag.band===h.band;
      const hover=!drag&&hoverIdx===i;
      ctx.beginPath();ctx.arc(hx,hy,active?10:hover?8:6,0,2*Math.PI);
      ctx.fillStyle="#fff";ctx.shadowColor=h.color;ctx.shadowBlur=active||hover?12:6;
      ctx.fill();ctx.shadowBlur=0;
      ctx.lineWidth=2;ctx.strokeStyle=h.color;ctx.stroke();
      if(active||hover){ctx.beginPath();ctx.arc(hx,hy,3.5,0,2*Math.PI);ctx.fillStyle=h.color;ctx.fill();}
      ctx.fillStyle=h.color;ctx.font="bold 10px sans-serif";
      ctx.fillText(String(h.idx+1),hx,hy-(active?15:12));
    });
  }
  if(drag&&dragTip){
    const qwarn=dragTip.q<0.6?" Q is low. The filter affects a wide frequency range.":(dragTip.q>8?" Q is high. The filter peak is narrow.":"");
    const txt="bands"+(drag.idx+1)+"  "+dragTip.f+" Hz  "+(dragTip.g>0?"+":"")+dragTip.g+" dB  Q "+dragTip.q.toFixed(2)+qwarn;
    ctx.font="11px sans-serif";
    const tw=ctx.measureText(txt).width+16;
    const tx=Math.max(left+2,Math.min(right-tw-2,dragTip.x-tw/2));
    const ty=Math.max(top+22,dragTip.y-16);
    rr(ctx,tx,ty-14,tw,19,6);
    ctx.fillStyle=isL?"rgba(255,255,255,.97)":"rgba(19,30,32,.96)";ctx.fill();
    ctx.strokeStyle=(drag&&drag.color)||cssv("--acc","#8fd4b8");ctx.lineWidth=1;ctx.stroke();
    ctx.fillStyle=cssv("--txt",isL?"#1c2f2a":"#e8f0ed");ctx.textAlign="left";
    ctx.fillText(txt,tx+8,ty);
  }
}
/* ============ 响应曲线手柄交互（Pointer：鼠标 + 触控统一） ============ */
let drag=null,hoverIdx=-1,dragTip=null;
const cv=$("respCv");
let hitR=14;
function cvPos(e){const r=cv.getBoundingClientRect();
  return{x:(e.clientX-r.left)*CW/r.width,y:(e.clientY-r.top)*CH/r.height}}
function hitHandle(x,y){let best=-1,bd=hitR;
  for(let i=0;i<handles.length;i++){const h=handles[i];
    const d=Math.hypot(h.x-x,h.y-y);if(d<=bd){bd=d;best=i}}
  return best}
function syncBandUI(band){
  const u=band._ui;if(!u)return;
  if(document.activeElement!==u.fi.inp)u.fi.inp.value=String(Math.round(band.f));
  if(u.gInp&&document.activeElement!==u.gInp)u.gInp.value=(Math.round(band.g*100)/100).toFixed(1);
  if(u.qInp&&document.activeElement!==u.qInp)u.qInp.value=band.q.toFixed(2);
}
cv.addEventListener("pointerdown",e=>{
  const p=cvPos(e);hitR=e.pointerType==="touch"?26:14;
  const i=hitHandle(p.x,p.y);
  if(i<0)return;
  const h=handles[i];
  drag={band:h.band,idx:h.idx,kind:h.kind,block:h.block,ch:h.ch,color:h.color,moved:false};
  cv.style.cursor="grabbing";
  try{cv.setPointerCapture(e.pointerId)}catch(_){}
  e.preventDefault();drawResp();
});
cv.addEventListener("pointermove",e=>{
  const p=cvPos(e);
  if(drag){
    const L=respLayout();
    const px=Math.max(L.left,Math.min(L.right,p.x));
    const py=Math.max(L.top,Math.min(L.bot,p.y));
    const f=Math.max(20,Math.min(20000,Math.round(L.fmin*Math.pow(L.fmax/L.fmin,(px-L.left)/(L.right-L.left)))));
    /* 拖动夹取 = PEQ 单段合法范围（±12，与固件一致）—— 不是视图量程 ±15。
     * 拖到 ±15 处会被夹回 ±12，这是**正确**的：单段本来最多只能 ±12。 */
    const g=Math.max(-EQ_GAIN_MAX,Math.min(EQ_GAIN_MAX,Math.round((aYMax-(py-L.top)/(L.bot-L.top)*(aYMax-aYMin))*2)/2));
    if(f!==drag.band.f||g!==drag.band.g){
      drag.moved=true;drag.band.f=f;drag.band.g=g;
      dragTip={x:p.x,y:p.y,f,g,q:drag.band.q};
      syncBandUI(drag.band);saveLocal();drawResp();
    }
    return;
  }
  if(e.pointerType==="touch")return;
  const i=hitHandle(p.x,p.y);
  if(i!==hoverIdx){hoverIdx=i;cv.style.cursor=i>=0?"grab":"default";drawResp();}
});
function endDrag(){
  if(!drag)return;
  if(drag.moved){
    const pF=drag.kind==="req"?P_REQ_FREQ:P_EQ_FREQ;
    const pG=drag.kind==="req"?P_REQ_GAIN:P_EQ_GAIN;
    setParam(drag.block,drag.ch,pF,drag.idx,drag.band.f,true);
    setParam(drag.block,drag.ch,pG,drag.idx,drag.band.g,true);
  }
  drag=null;dragTip=null;hoverIdx=-1;cv.style.cursor="default";drawResp();
}
cv.addEventListener("pointerup",endDrag);
cv.addEventListener("pointercancel",endDrag);
cv.addEventListener("pointerleave",()=>{if(!drag){hoverIdx=-1;cv.style.cursor="default";drawResp()}});
function syncPortSel(){
  const val=$("respSel").value;
  document.querySelectorAll("#portCards .port").forEach((card,k)=>
    card.classList.toggle("sel",val==="out"+k));
}
function showResp(v){
  const rs=$("respSel");
  if(rs){let ok=false;for(const o of rs.options){if(o.value===v){ok=true;break}}
    if(ok)rs.value=v;}
  drawResp();syncPortSel();
  const cvs=$("respCv");
  if(cvs){const r=cvs.getBoundingClientRect();
    if(r.top<0||r.bottom>window.innerHeight){try{cvs.scrollIntoView({behavior:"smooth",block:"start"})}catch(_){}}}
}
$("respSel").onchange=()=>{drawResp();syncPortSel()};
$("respView").onchange=()=>{gView=$("respView").value;drawResp()};
/* ================= Tabs 导航 ================= */
document.querySelectorAll(".tab").forEach(t=>{
  t.addEventListener("click",()=>{
    showPanel(t.dataset.panel);
    const tp=$("tp-"+t.dataset.panel);
    if(tp&&window.innerWidth<860){
      try{tp.scrollIntoView({behavior:"smooth",block:"start"})}catch(_){}
    }
  });
});
/* ================= 设备端配置预设（NVS 槽位，方块选择 + 占用状态条） ================= */
let gPresetSelSlot=0;                     /* 当前选中的槽（0 基），点方块切换 */
function buildPresetSlots(){
  const wrap=$("presetSlots");if(!wrap)return;
  wrap.innerHTML="";
  for(let i=0;i<PRESET_SLOTS;i++){
    const used=!!(gPresetMask&(1<<i));
    const s=document.createElement("div");
    s.className="pslot"+(i===gPresetSelSlot?" sel":"")+(used?" used":"");
    s.title="Slot "+(i+1)+(used?" / occupied":" / empty");
    const n=document.createElement("div");n.className="pnum";n.textContent=i+1;
    const st=document.createElement("div");st.className="pstat";
    st.title=used?"Device preset stored":"Device preset empty";
    s.appendChild(n);s.appendChild(st);
    s.onclick=()=>{if(gPresetSelSlot!==i){gPresetSelSlot=i;buildPresetSlots()}};
    wrap.appendChild(s);
  }
}
function updPresetUI(){buildPresetSlots()}
$("btnDevPresetSave").onclick=()=>{
  if(!transportActive()){log("No control connection.","warn");return}
  sendFrame(new Uint8Array([MSG_PRESET_SAVE,gPresetSelSlot]));
  log("Save requested for device preset "+(gPresetSelSlot+1),"info");
};
$("btnDevPresetLoad").onclick=()=>{
  if(!transportActive()){log("No control connection.","warn");return}
  sendFrame(new Uint8Array([MSG_PRESET_LOAD,gPresetSelSlot]));
  log("Load requested for device preset "+(gPresetSelSlot+1)+". The device will send its configuration.","info");
};
$("btnDevPresetClear").onclick=()=>{
  if(!transportActive()){log("No control connection.","warn");return}
  sendFrame(new Uint8Array([MSG_PRESET_CLEAR,gPresetSelSlot]));
  log("Clear requested for device preset "+(gPresetSelSlot+1),"info");
};
/* ================= 削波保护（限幅器） ================= */
function buildLimUI(){
  const wrap=$("limTogWrap");if(!wrap)return;
  wrap.innerHTML="";
  wrap.appendChild(tog(cfg.lim.on,v=>{cfg.lim.on=v;
    setParam(BLOCK_GLOBAL,0,P_LIMIT_ON,0,v,false);saveLocal()}));
}
function syncLimUI(){
  buildLimUI();
  /* 程序化赋值不会触发 input 事件 —— 这里必须显式刷新填充进度，
   * 否则「阈值」进度条的填充还停在上一次的位置，和小球对不上。 */
  const th=$("limThresh"),rl=$("limRel");
  th.value=cfg.lim.thresh;updFill(th);$("limThreshv").textContent=cfg.lim.thresh.toFixed(1)+" dBFS";
  rl.value=cfg.lim.release;updFill(rl);$("limRelv").textContent=Math.round(cfg.lim.release)+" ms";
}
$("limThresh").oninput=()=>{cfg.lim.thresh=+$("limThresh").value;updFill($("limThresh"));
  $("limThreshv").textContent=cfg.lim.thresh.toFixed(1)+" dBFS";
  setParam(BLOCK_GLOBAL,0,P_LIMIT_THRESH,0,cfg.lim.thresh,true);saveLocal()};
$("limRel").oninput=()=>{cfg.lim.release=+$("limRel").value;updFill($("limRel"));
  $("limRelv").textContent=Math.round(cfg.lim.release)+" ms";
  setParam(BLOCK_GLOBAL,0,P_LIMIT_RELEASE,0,cfg.lim.release,true);saveLocal()};
/* ================= 响度补偿（等响） ================= */
function buildLoudUI(){
  const wrap=$("loudTogWrap");if(!wrap)return;
  wrap.innerHTML="";
  wrap.appendChild(tog(cfg.loud.on,v=>{cfg.loud.on=v;
    setParam(BLOCK_GLOBAL,0,P_LOUD_ON,0,v,false);saveLocal();syncLoudUI();drawResp()}));
}
function loudNowText(){
  const g=loudBoostDb();
  if(!cfg.loud.on)return "Off";
  if(g<=0.001)return "0.0 dB (master volume at or above reference)";
  return "+"+g.toFixed(2)+" dB";
}
function syncLoudUI(){
  buildLoudUI();
  const a=$("loudAmt"),r=$("loudRef");
  a.value=cfg.loud.amount;updFill(a);$("loudAmtv").textContent=cfg.loud.amount.toFixed(1)+" dB";
  r.value=Math.round(cfg.loud.ref*100);updFill(r);$("loudRefv").textContent=Math.round(cfg.loud.ref*100)+"%";
  $("loudNowv").textContent=loudNowText();
}
$("loudAmt").oninput=()=>{cfg.loud.amount=+$("loudAmt").value;updFill($("loudAmt"));
  $("loudAmtv").textContent=cfg.loud.amount.toFixed(1)+" dB";
  setParam(BLOCK_GLOBAL,0,P_LOUD_AMOUNT,0,cfg.loud.amount,true);saveLocal();syncLoudUI();drawResp()};
$("loudRef").oninput=()=>{cfg.loud.ref=+$("loudRef").value/100;updFill($("loudRef"));
  $("loudRefv").textContent=Math.round(cfg.loud.ref*100)+"%";
  setParam(BLOCK_GLOBAL,0,P_LOUD_REF,0,cfg.loud.ref,true);saveLocal();syncLoudUI();drawResp()};
/* ================= S31 HWLOOP预算：关压缩130 / 三开110 =================
 * 两档都是96k/240帧整链的最坏布局工程估计，预算仍2250us。
 * mono13us、pair19us；响度/削波恒预留，压缩器额外155us。
 * 此页面可能先连接旧0.121设备，当前准入仍尊重SYS_STATUS，不把新上限套给旧固件。
 * 常量只有顶部一处；精度字段仅保留blob兼容，不改变上限。
 */
function limitForMode(compOn){
  const m=/^0\.(\d+)$/.exec(gFwVer);
  let lim=compOn?LIMIT_COMP:LIMIT_NOCOMP;
  if(gFiltLimit&&m&&+m[1]<122){
    lim=+m[1]>=119?(compOn?75:87):(compOn?104:117);
  }
  // Current report can temporarily lag an effect toggle; use the smaller bound.
  if(gFiltLimit&&!!compOn===!!cfg.comp.on)lim=Math.min(lim,gFiltLimit);
  return lim;
}
function limitNow(){return limitForMode(!!cfg.comp.on)}
function precisionIsExt(){return cfg.precision===PRECISION_EXTENDED}
/* 设备上报的"总双二阶数"才是权威；设备没连上时退化成"本地估算"（界面自己的段数合计）。
 * 本地估算只用于**挡操作**（宁可少算也别放过去），实际判定仍以 gFiltLimit 为准。
 *
 * ★ v30 修：本地估算过去与固件 `count_biquads_of()` **口径不一致**，离线时最多能差 ~60 段
 *   ⇒ 用户离线配到"117"（117 为当时的档位）、一连设备就被判超标而拒收。现在**逐条对齐固件口径**：
 *     · 输入级取**两源最大值**（USB / 蓝牙同一时刻只跑一路）—— 原来是**求和**（最多多算 30）
 *     · 分频按**阶数**计（slope 1/2/4 ⇒ 1/2/4 个双二阶），HP 与 LP **分开算**
 *       —— 原来固定算 1（slope=4 时少算 3；4 通道 ×2 处 ⇒ 最多少算 24）
 *     · **端口 0** 的 HP/LP **计入** —— 原来**完全没算**（最多少算 8）
 *   对照固件 `dsp_config.c` 的 `count_biquads_of()` / `xo_stage_count()`。 */
function lrStages(xo){
  /* 与固件 `xo_stage_count()` 同式：关着 0 级；LR slope=1 是 1 级；其余都按 slope 级 */
  if(!xo||!xo.on)return 0;
  const slope=(xo.slope===1||xo.slope===2||xo.slope===4)?xo.slope:2;
  if(xo.type===1&&slope===1)return 1;      /* Linkwitz-Riley LR2 = 一级二阶 */
  return slope;
}
function localBandCount(){
  let inMax=0;
  cfg.inputs.forEach(i=>{
    const n=i.eq.filter(b=>b.on).length+i.req.filter(b=>b.on).length;
    if(n>inMax)inMax=n;                    /* 两源取**最大值**，不是求和 */
  });
  let n=inMax;
  cfg.outs.forEach(o=>{
    n+=o.eq.filter(b=>b.on).length;
    n+=lrStages(o.hp)+lrStages(o.lp);      /* 分频按阶数计，HP/LP 分开 */
  });
  /* 端口分频：**只算端口 0**（固件只处理 i2s0；端口 1 不计入预算） */
  const p0=(cfg.ports&&cfg.ports[0])||null;
  if(p0)n+=lrStages(p0.hp)+lrStages(p0.lp);
  return n;
}
function bandCountNow(){return gFiltLimit?gFiltCount:localBandCount()}
/* 开压缩器是否超过目标固件的对应上限？（设备没连上也照样拦，避免用户白配一遍再被拒） */
function compWouldExceed(extra){
  return (bandCountNow()+(extra||0))>limitForMode(true);
}
/* ── 弹窗 ──────────────────────────────────────────────────────────────
 * 不用浏览器原生弹窗：①它会**阻塞**真机端到端测试的 CDP 求值；②没法给"该怎么办"写清两行。
 * 自绘一个轻量遮罩，按钮点是/否，Esc 与点遮罩等价于取消。
 * ⚠ 注释里也别把原生弹窗的调用原样写出来 —— webui_dom_check 用正则扫全文，
 *   连注释一起命中，会误报"用了原生弹窗"。 */
let dspPopupEl=null;
function popup(title,bodyHtml,okText){
  return new Promise(resolve=>{
    /* 用模块级变量记当前弹窗，而不是按 id 去查 —— 那个 id 是**运行时才创建**的，
     * 静态 id 检查（webui_dom_check）在 HTML 里找不到声明会报"引用缺失"。
     * ⚠ 注释里也别把那个查询原样写出来：检查用的是正则扫全文，连注释一起命中。 */
    if(dspPopupEl){try{dspPopupEl.remove()}catch(e){}dspPopupEl=null}
    const ov=document.createElement("div");
    ov.id="dspPopup";
    ov.style.cssText="position:fixed;inset:0;z-index:9999;background:rgba(0,0,0,.55);"
      +"display:flex;align-items:center;justify-content:center;padding:20px";
    const bx=document.createElement("div");
    bx.className="panel";
    bx.style.cssText="max-width:520px;width:100%;margin:0;box-shadow:0 20px 60px -20px #000";
    const h=document.createElement("h3");h.textContent=title;h.style.marginTop="0";
    const bd=document.createElement("div");
    bd.style.cssText="font-size:12.5px;line-height:1.85;color:var(--dim)";
    bd.innerHTML=bodyHtml;
    const foot=document.createElement("div");
    foot.style.cssText="display:flex;gap:10px;justify-content:flex-end;margin-top:16px";
    const ok=document.createElement("button");ok.className="btn";ok.type="button";
    ok.textContent=okText||"OK";
    const done=v=>{try{ov.remove()}catch(e){}if(dspPopupEl===ov)dspPopupEl=null;
      document.removeEventListener("keydown",esc);resolve(v)};
    const esc=e=>{if(e.key==="Escape")done(false)};
    ok.onclick=()=>done(true);
    ov.onclick=e=>{if(e.target===ov)done(false)};
    document.addEventListener("keydown",esc);
    foot.appendChild(ok);bx.appendChild(h);bx.appendChild(bd);bx.appendChild(foot);
    ov.appendChild(bx);document.body.appendChild(ov);
    dspPopupEl=ov;
    ok.focus();
  });
}
/* 统一的"段数会越界"提示文案（加段闸门 / 开压缩器闸门共用）
 * ★ v30：上限拆两档 ⇒ 文案要说清**为什么现在这档是这个数**，以及两条出路。 */
function overLimitBody(adding, forComp){
  const lim=forComp?limitForMode(true):limitNow();
  return "Current use: <b>"+bandCountNow()+" biquads</b>"
    +(adding?". Additional filters: "+(adding||1)+" biquads":'')+'。<br><br>'
    +"Limits: <b>compressor off "+limitForMode(false)+"</b> / "
    +"<b>compressor on "+limitForMode(true)+"</b>"
    +". The compressor needs additional processing time.<br>"
    +"Compressor "+(cfg.comp.on?"on":"off")+". Current limit: "+limitNow()+" biquads."
    +(forComp?"With the compressor on, the limit is <b>"+lim+" biquads</b>.":'')+'<br><br>'
    +"<b>Select one action:</b><br>"
    +"Reduce the total to <b>"+lim+" biquads</b>, or<br>"
    +'· '+(cfg.comp.on?"set the compressor off (limit "+limitForMode(false)+")."
                    :"Keep the compressor off");
}
/* 兼容旧调用名 */
function compOverLimitBody(){return overLimitBody(0,true)}
/* 保存 NVS 前的最后一道闸：**段数超过当前状态的上限**时不许存。
 * ⚠ 这道闸在"设备未连接"时也要生效（`gFiltLimit=0`）——
 *   否则用户能离线配出超限配置、存进去、载不回来。 */
function configBlockReason(){
  if(cfg.ports[1].format===2)return "I2S1 right-justified format is not supported. Select a supported format before you save.";
  const n=bandCountNow(), lim=limitNow();
  if(n>lim){
    return "Filter use: <b>"+n+"</b>. Current limit: <b>"+lim+"</b>"
      +" (compressor "+(cfg.comp.on?"on":"off")+'）。';
  }
  return "";
}
async function guardSaveNvs(){
  const why=configBlockReason();
  if(!why)return true;
  await popup("Cannot save to NVS",
    why+'<br><br>'
    +"<b>Select one action:</b><br>"
    +"Reduce the total to <b>"+limitNow()+" biquads</b>, or<br>"
    +'· '+(cfg.comp.on?"set the compressor off (limit "+limitForMode(false)+")."
                    :"Keep the compressor off")+'<br><br>'
    +"This configuration exceeds the storage limit. Reduce the filter count before you save it to the device.",
    "OK");
  log("NVS save blocked: "+why.replace(/<[^>]+>/g,""),"err");
  return false;
}
/* ================= 压缩器（动态范围 / 夜间模式） ================= */
function compRatioText(x10){return ((x10||10)/10).toFixed(1)+":1"}
/* 说明行里那半句"当前 N 段 / 上限 M 段"；顺带把文案里的 __EXT__/__COMP__ 占位符
 * 换成**常量里的真数字** —— 这样"改常量忘了改文案"这类漂移不可能发生。
 * （用 textContent 而不是往 HTML 里塞 <span>：塞了标签会把这句中文切断，静态断言与
 *   无障碍朗读都会被切得七零八落。） */
function updCompBudgetText(){
  const el=$("compBudgetNow");if(!el)return;
  const n=bandCountNow();
  el.innerHTML="Current use: <b>"+n+"</b>. Current limit: <b>"+limitNow()+"</b>"
    +" (compressor "+(cfg.comp.on?"on":"off")+"）"
    +(gFiltLimit?" (device limit "+gFiltLimit+")":" (local estimate; device disconnected)");
}
function fillLimitPlaceholders(){
  for(const el of document.querySelectorAll("*")){
    if(el.children.length===0 && el.textContent &&
       (el.textContent.indexOf("__EXT__")>=0 || el.textContent.indexOf("__COMP__")>=0)){
      el.textContent=el.textContent
        .split("__EXT__").join(String(LIMIT_NOCOMP))
        .split("__COMP__").join(String(LIMIT_COMP));
    }
  }
}
/* 开/关压缩器时同步切换设备模式（对用户隐式）：
 * ★ v30：**不再下发 `0x80`、也不再"开关压缩器时切模式"** ——
 *   固件两种精度模式的预算**相同**，上限完全由 `0x7C`（压缩器）决定。
 *   `pushAll()` 里仍原样带着 `0x80`（保 blob 兼容），但 **UI 永不修改它**。
 * 超过压缩器档上限时先弹窗拦住，不下发 `on=1`
 * （设备侧也会硬拒 —— 双保险：前端拦得住界面，设备拦得住协议直发）。 */
async function setCompressor(on){
  if(on){
    if(compWouldExceed(0)){await popup("Cannot start the compressor",compOverLimitBody());return false}
  }
  cfg.comp.on=on?1:0;
  setParam(BLOCK_GLOBAL,0,P_COMP_ON,0,cfg.comp.on,false);
  saveLocal();syncCompUI();
  /* 压缩器开关会改变上限⇒ 立刻重读，免得段数护栏用旧上限 */
  if(typeof sendSysReq==="function"){try{sendSysReq()}catch(e){}}
  return true;
}
function buildCompUI(){
  const wrap=$("compTogWrap");if(!wrap)return;
  wrap.innerHTML="";
  wrap.appendChild(tog(cfg.comp.on,v=>{setCompressor(!!v)}));
  updCompBudgetText();
}
/* 文案里的 __EXT__ / __COMP__ 占位符在启动时替换一次（见 fillLimitPlaceholders） */
/* ================= ★ v36：硬件控制面板（阶段 B 接口层）=================
 * 三项（规格 v9 §4.3）：停止延时 / 播放延时 / 电平反转。
 * ⛔ 0.112：原第一项"编码器启用"（硬件控制项里的 `enc` / `0x83`）与键位面板一并删除 ——
 *   实体编码器 / USB HID 调主机音量功能已从固件整体移除，这里不留开关、不下发、不显示。
 * ⚠ v42 用户改口：原"端口区一个字不动"的裁定**被新指令覆盖** ——
 *   端口区 I2S 接口卡下方新增「静音输出 · MUTE」嵌套 UI（任务书 42），
 *   与本面板同源 cfg.hw、双向同步，不产生第二套参数。
 * ⚠ 静音的**行为**不在本轮：这里只把配置下发给设备的参数区。 */
function buildHwUI(){
  const mp=$("mutePolWrap");if(mp){mp.innerHTML="";mp.appendChild(tog(cfg.hw.pol,v=>setHwPol(!!v)))}
}
function syncHwUI(){
  buildHwUI();
  // Mute settings have one home in Ports. Do not move the input cursor on readback.
  const ma=$("muteDelayA"),mb=$("muteDelayB");
  if(ma&&document.activeElement!==ma)ma.value=cfg.hw.delayA;
  if(mb&&document.activeElement!==mb)mb.value=cfg.hw.delayB;
}
function setHwPol(on){
  cfg.hw.pol=on?1:0;
  setParam(BLOCK_GLOBAL,0,P_HW_POLARITY,0,cfg.hw.pol,false);
  saveLocal();syncHwUI();return true;
}
(function wireHwUI(){
  const ma=$("muteDelayA"),mb=$("muteDelayB");
  if(ma)ma.oninput=()=>{cfg.hw.delayA=Math.max(0,Math.min(HW_DELAY_MAX,+ma.value|0));
    setParam(BLOCK_GLOBAL,0,P_HW_DELAY_A,0,cfg.hw.delayA,false,2);   /* u16 ⇒ nbytes=2 */
    saveLocal()};
  if(mb)mb.oninput=()=>{cfg.hw.delayB=Math.max(0,Math.min(HW_DELAY_MAX,+mb.value|0));
    setParam(BLOCK_GLOBAL,0,P_HW_DELAY_B,0,cfg.hw.delayB,false,2);   /* u16 ⇒ nbytes=2 */
    saveLocal()};
})();
fillLimitPlaceholders();
function syncCompUI(){
  buildCompUI();
  const th=$("compTh"),ra=$("compRatio"),rl=$("compRel");
  th.value=cfg.comp.th;updFill(th);$("compThv").textContent=cfg.comp.th+" dBFS";
  ra.value=cfg.comp.ratio;updFill(ra);$("compRatiov").textContent=compRatioText(cfg.comp.ratio);
  rl.value=cfg.comp.rel;updFill(rl);$("compRelv").textContent=Math.round(cfg.comp.rel)+" ms";
  /* 自动补偿增益 = |阈值|×(1−1/压缩比)，与固件 dsp_engine.c 同一式子 */
  $("compMakeupv").textContent="+"+compMakeup(cfg.comp.th,cfg.comp.ratio).toFixed(2)+" dB";
}
$("compTh").oninput=()=>{cfg.comp.th=+$("compTh").value;updFill($("compTh"));
  $("compThv").textContent=cfg.comp.th+" dBFS";
  setParam(BLOCK_GLOBAL,0,P_COMP_TH,0,cfg.comp.th,false);saveLocal();syncCompUI()};
$("compRatio").oninput=()=>{cfg.comp.ratio=+$("compRatio").value;updFill($("compRatio"));
  $("compRatiov").textContent=compRatioText(cfg.comp.ratio);
  setParam(BLOCK_GLOBAL,0,P_COMP_RATIO,0,cfg.comp.ratio,false,2);   /* v2: u16 */saveLocal();syncCompUI()};
$("compRel").oninput=()=>{cfg.comp.rel=+$("compRel").value;updFill($("compRel"));
  $("compRelv").textContent=Math.round(cfg.comp.rel)+" ms";
  setParam(BLOCK_GLOBAL,0,P_COMP_REL,0,cfg.comp.rel,false);saveLocal();syncCompUI()};
$("compNightBtn").onclick=async()=>{
  /* 夜间模式一键 = 填设计值 + 开压缩器。开压缩器要过段数闸门，所以这里 async。 */
  if(compWouldExceed(0)){await popup("Cannot start the compressor",compOverLimitBody());return}
  cfg.comp.th=NIGHT_PRESET.th;cfg.comp.ratio=NIGHT_PRESET.ratio;cfg.comp.rel=NIGHT_PRESET.rel;
  setParam(BLOCK_GLOBAL,0,P_COMP_TH,0,cfg.comp.th,false);
  setParam(BLOCK_GLOBAL,0,P_COMP_RATIO,0,cfg.comp.ratio,false,2);   /* v2: u16 */
  setParam(BLOCK_GLOBAL,0,P_COMP_REL,0,cfg.comp.rel,false);
  /* ⚠ 开关必须走 setCompressor()：它会顺手把精度模式切到"高精度(41)"，
   * 否则在扩展(82)模式下直接发 on=1 会被固件硬拒 status=5（白点一次）。 */
  await setCompressor(true);
  log("Night mode on. Threshold "+NIGHT_PRESET.th+" dBFS / "+compRatioText(NIGHT_PRESET.ratio)+
      " / release "+NIGHT_PRESET.rel+" ms. Automatic gain +"+compMakeup(NIGHT_PRESET.th,NIGHT_PRESET.ratio).toFixed(2)+" dB）","info");
};
/* ================= 系统面板 ================= */
$("monRate").onchange=()=>{
  monInterval=Math.max(500,Math.min(5000,+$("monRate").value));
  if(transportActive())startMon();
};
showPanel("in");
{const pm=location.hash.match(/panel=(in|route|dsp|ports)(?:&|$)/);if(pm)showPanel(pm[1],false);}
$("masterVol").oninput=()=>{const v=+$("masterVol").value/100;cfg.master=v;
  $("masterVolv").textContent=masterLabel(v);
  setParam(BLOCK_GLOBAL,0,P_MASTER_VOL,0,v,true);saveLocal();
  if($("loudNowv"))$("loudNowv").textContent=loudNowText();   /* 响度补偿随主音量实时变化 */
  drawResp()};
/* ================= 启动 ================= */
setTheme(gTheme);
fitCanvas();
try{rebuildUI()}catch(e){
  log("Initialization error: "+e.message,"err");console.error(e);
  try{drawResp()}catch(_){}
}
requestAnimationFrame(()=>{try{fitCanvas();drawResp()}catch(e){}});
window.addEventListener("resize",()=>{try{refreshSliderGeom();fitCanvas();drawResp()}catch(e){}});
window.__dsp={cfg,drawResp,chainAt,showPanel,setTheme,getHandles:()=>handles,getDrag:()=>drag,getTip:()=>dragTip,
  linkTarget,mirrorParam,setParam,updFill,refreshSliderGeom,loudBoostDb,loudStages,loudNowText,syncLoudUI};
log("Ready. Set input, routing, DSP, and port parameters in their panels.");
log("Connect serial or BLE. Read the device configuration before you change parameters.","info");
window.addEventListener("s31languagechange",()=>drawResp());
document.querySelectorAll(".stat.conn,.fv-node").forEach(el=>{el.tabIndex=0;el.setAttribute("role","button");el.onkeydown=e=>{if(e.key==="Enter"||e.key===" "){e.preventDefault();el.click();}};});

