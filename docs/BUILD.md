# 构建说明

## 当前公开包的边界

这是 0.254 的应用/WebUI 源码快照，**不是开箱即用、可独立复现的完整固件发行包**。完整 ESP-IDF、工具链和未确认分发权的解码库不上传。`sdk/manifest.json` 列出包含与省略的差异；任何省略都不能当作“与基线相同”。

历史真源 HEAD 为 `10a521223b84ab51e54417222de195e8f73032bc`。精简公开仓库使用新历史，该 HEAD 是本地来源身份，不是本仓库中可 checkout 的提交。

外部 SDK 基础来自 [ESP32-S31-A2DP-Codecs](https://github.com/WillyBilly06/ESP32-S31-A2DP-Codecs)，参考提交 `71071a783811799d2977e4e87f733acdc2220c29`，目录 `esp-idf-v6.1-codecs`。公开 SDK 仍有各自许可问题；本项目不替它的所有内容作授权承诺。差异清单逐文件保存外部基础 SHA，应用前必须匹配。

## WebUI

需要 Node.js，无 npm 依赖。仓库根目录运行：

```text
node webui/build.cjs
```

输出 `webui/dsp webui.html` 和相同的 dist 副本。发布时只需一个 HTML。没有下载、串口、蓝牙或硬件操作。

## 固件（供具备合法依赖的开发者参考）

1. 获取适合 ESP32-S31 的 ESP-IDF v6.1 fork，以及相应 RISC-V GCC 15.2 工具链、Python 环境。
2. 核对 `sdk/manifest.json` 的基础文件身份。`sdk/overlay/` 是以 SDK 根为相对路径的已发布文件；`sdk/tinyusb-overlay/` 相对于 TinyUSB 组件根。只在独立 SDK 副本中使用，先保留备份。不要覆盖其他工程共用的 SDK。
3. **先解决省略文件和依赖许可**。不能通过删除依赖、关闭 guard 或把未知库改名来冒充同一版固件。
4. 在已导出 IDF 环境中进入 `firmware`。组件管理器按 `dependencies.lock` 获取 TinyUSB `0.21.0~1`（基础提交 `7049c58a0e895acc92c6407574b05b5536eddfc8`），再核对并应用两份 TinyUSB 差异。组件管理器覆盖修改时必须重新核对。
5. 用 `tools/build_a2dp_fork.ps1` 运行源码 guard、`idf.py -B build-fork build` 和对象 guard；需要当前 IDF Python 环境中的 pyelftools。wrapper 不自动下载/替换 SDK，不改冻结清单，也不执行刷写。
6. 历史冻结对象是精确代码/数据/重定位基线。工具链或 SDK 变化可能导致对象 guard 失败；**失败不等于可以重锚或忽略**，应先检查差异。此公开精简包未重新构建，不宣称对象复现已通过。

`main/core1_frozen.json` 保留历史原始字节。公开 checker 仅将 SDK 路径解析为 `IDF_PATH`（无此变量时使用历史路径），不改变任何散列或校验条件。`SOURCE_MANIFEST.json` 记录这一移植改动。

## APP-only 与回退

当前仓库不提供可刷写二进制。以后只有解决许可、成功构建并留存身份/guard 后，才可以建立二进制 release。

现有板分区布局下 APP 地址为 `0x10000`。这不是可对任意板照抄的烧录指令。先核对目标、分区、Flash 容量和实际设备。只更新 APP，不覆盖 bootloader、分区表或顺带擦除 NVS；不要使用全片擦除。回退也要选择对应板卡和配置兼容的已知镜像，不能仅按文件名判断。

调音配置、配对记录与功放配置可能含个人信息，备份仅保存在本机，不提交仓库。
