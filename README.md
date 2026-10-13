# STM32F407VGT6 工训赛事搬运小车

项目包含F407主车固件、香橙派B2视觉、串口屏、本地网页控制台和独立F103物料台转盘。主车为STM32F407VGT6（LQFP100、1 MB Flash），使用STM32 HAL裸机主循环；车载ID8转盘属于主车UART5业务，与独立F103工程分别维护。

## 当前能力与验收范围

- 底盘反馈运动、航向保持、固定路线、雷达静止采集/建图/规划及同C核心的网页预演。
- 颜色/编号环静态对准、手动码垛、动态追踪与夹取、单件车载取放，以及后向机械臂两批任务调度。
- 二维码任务码、OLED/串口屏、本地控制台、参数读回及通信诊断。

这些是当前源码能力；完整两批赛事、连续带料取放和定位精度仍需实车验收。软件回归、烧录、香橙派部署及实物成功分别记录。已知问题与暂缓决定见[问题清单](docs/待解决问题.md)。

统一比赛、取放与路线的现行需求见[比赛与取放约定](docs/比赛与取放约定.md)。固件与控制台已集成；已有[10月14日软件验证与烧录记录](docs/开发记录/2026-10-14-统一DXDY与物料台一次横移.md)及[后续编号环版本记录](docs/开发记录/2026-10-14-编号环对准高度与抬升选择.md)。完整两批实车验收及后续修改的下载状态分别核对，不从历史烧录推定当前设备版本。

## 入口

| 模块 | 使用入口 |
|---|---|
| F407主车 | [主车说明](Code/template/README.md)；Keil `Code/template/MDK-ARM/template.uvprojx`，Target `template` |
| 本地控制台 | [启动与操作](Code/upper-computer-web/local-car/README.md)；双击 `Code/upper-computer-web/local-car/start.cmd` |
| 香橙派 | [视觉说明](Code/orange_pi/README.md)；本地入口 `vision.py`，实机部署状态单独核对 |
| 串口屏 | [屏幕工程](Code/HMI/README.md)；`display.HMI`及字库 |
| 独立F103转盘 | [转盘说明](Code/物料台转盘/README.md)；Target `turntable`，不套用主车下载配置 |
| 参考/测试/工具 | [代码索引](Code/README.md)，参考代码不加入主车默认构建 |

控制台需要Python 3和pyserial，缺少时执行 `python -m pip install pyserial`。访问 `http://127.0.0.1:8765/`，由用户选择实际串口，默认115200/8N1；启动、刷新和重连不启动运动。无需npm安装或云部署。

## 软件验证

在仓库根运行与改动相关的检查：

```powershell
python Code/tests/run_firmware_tests.py
node --test Code/upper-computer-web/local-car/tests/*.test.mjs
python -m unittest discover -s Code/upper-computer-web/local-car/tests -p 'test*.py'
python Code/tests/radar_wire_contract_test.py
```

主机固件测试需要Python/GCC，网页测试需要Node.js；不打开实车串口。独立F103测试见其模块说明。默认只维护、构建Keil，EIDE暂不维护。日志在 `.embeddedskills/`。

## 文档

- [协作规则](AGENTS.md)：授权、实现和验证边界。
- [硬件与运行基线](docs/硬件与运行基线.md)：接线、Emm单位、坐标及已确认条件。
- [比赛与取放约定](docs/比赛与取放约定.md)：当前任务顺序、夹持/库存、暂停恢复及路线约束。
- [协议索引](docs/上位机协议.md)：按系统、机械臂、视觉和雷达查命令与报文。
- [资料索引](docs/README.md)：模块操作、硬件手册、设计基线和历史证据。
- [开发记录](docs/开发记录/README.md)：修改原因及当时软件/实机验证；不代替现行手册。

代码入口集中于 `Code/`；`docs/`保存正式资料，`tmp/`为本地临时分析。上层代码不要求特定风格，底层Emm驱动继续保留。旧版本通过Git和[整理前文档快照](docs/开发记录/2026-10-09-文档整理前快照/README.md)追溯。
