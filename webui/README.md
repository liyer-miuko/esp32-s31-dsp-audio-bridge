# 单文件控制台

直接使用 `dsp webui.html`，无需配套 CSS、JavaScript、字体或网络下载。支持简体中文/English、明暗主题、Web Serial 和 Web Bluetooth。浏览器/系统仍需支持设备访问接口；BLE 调音不是 A2DP 音频。

读取配置后再调节。导入文件只应用到当前配置，不自动写 NVS。完整配置文件与 DSP 音效参数文件分别导入导出；功放 JSON 在输出端口单独管理。有效的左右差异与 1～15 段数量会保留，联动只用于手动修改。

开发时修改 `console.source.html`、CSS 和 JS，然后在仓库根运行 `node webui/build.cjs`。Node.js 内置工具即可，不安装 npm 依赖。打包生成完整 HTML 与 dist 副本；交付时只复制一个 HTML。

协议见 [PROTOCOL.md](../docs/PROTOCOL.md)。功放模板默认关闭，不能替代实际板卡初始化和功率/保护系数。I²S1 右对齐不受支持。硬件能力和未验证范围见根 README。
