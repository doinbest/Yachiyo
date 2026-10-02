# 代码目录

本目录统一保存项目代码、屏幕配置、软件测试与参考项目。仓库说明和协作规则分别见 [README](../README.md) 与 [AGENTS](../AGENTS.md)；正式资料在 [docs](../docs/README.md)。

| 目录 | 用途与入口 |
|---|---|
| `template/` | F407 主车固件；Keil `MDK-ARM/template.uvprojx`，Target `template` |
| [orange_pi/](orange_pi/README.md) | 香橙派最新视觉源码 `vision.py`，使用 USB CDC B2；本地更新不等于实机部署 |
| [HMI/](HMI/README.md) | 串口屏 `display.HMI` 和字库资源 |
| [物料台转盘/](物料台转盘/README.md) | 独立 F103 转盘固件及原始压缩包 |
| [upper-computer-web/](upper-computer-web/README.md) | 本地控制台；`local-car/start.cmd` 或启动快捷方式 |
| [reference/](reference/README.md) | GongXun、F1 雷达及 C# 雷达上位机参考代码 |
| `tests/` | F407 主机回归；在仓库根运行 `python Code/tests/run_firmware_tests.py` |
| `tools/` | 共用开发工具位置，目前仅保留旧缓存 |

各模块内部目录与相互邻接关系保持。构建与软件测试日志统一保存在仓库根 `.embeddedskills/`；参考工程不自动参加活动固件构建。
