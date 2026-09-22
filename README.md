# STM32F407VGT6 工训赛事搬运小车

本项目包含搬运小车 STM32 固件、香橙派视觉程序和本地网页控制台。主控为 STM32F407VGT6（LQFP100，1 MB Flash），主要使用 STM32 HAL 裸机主循环。

现有功能包括机械臂与夹爪、麦轮底盘、HWT101 航向验证与保持、B2 颜色搜索/对准、二维码任务码、串口屏、OLED、地图遥测和实车前四段路线测试。视觉对准和路线到点不等于完整搬运任务完成；软件回归不替代实物验收。

## 三个当前入口

| 入口 | 用途 | 启动/使用 |
|---|---|---|
| [template/](template/) | STM32 固件 | Keil 打开 `template/MDK-ARM/template.uvprojx`，Target 为 `template` |
| [orange_pi/change.py](orange_pi/change.py) | 香橙派 B2 颜色视觉 | 部署到具有对应相机、串口及运行依赖的香橙派，先核对程序内设备路径 |
| [local-car](upper-computer-web/local-car/README.md) | 八千代·巡航本地控制台 | Windows 双击 `upper-computer-web/local-car/start.cmd` |

控制台需要 Python 3、pyserial；缺少时执行 `python -m pip install pyserial`。服务启动后用桌面 Chrome/Edge 打开 `http://127.0.0.1:8765/`。无需 npm 安装、旧 React 页面或云部署。启动不会自动连接串口。

网页共享模式需从设备列表手动选择串口，无默认 COM 号；通信格式默认 115200 / 8N1，实际串口应现场核对，它与 `.embeddedskills/config.json` 的工具串口配置相互独立。准备、实车运动和停车流程详见控制台手册。

上电后保持整车静止：固件默认 Emm42 Receive 应答，自动等待 IMU 新鲜数据并执行 5 秒验证；通过后才取得航向控制资格。失败时在系统页重新验证，地图起点仍需准备跑图确认。见[本次变更记录](docs/开发记录/2026-09-22默认Receive与上电IMU验证.md)。

## 硬件与通信

| 接口 | 引脚 | 当前用途 |
|---|---|---|
| USART1 | PA9/PA10 | 无线文本控制台，ASCII/CRLF |
| USART2 | PA2/PA3 | 预留雷达 |
| USART3 | PD8/PD9 | 串口屏 |
| UART4 | PC10/PC11 | 二维码 |
| UART5 | PC12/PD2 | 步进电机共享总线 |
| USB CDC | PA11/PA12 | 香橙派 B2 视觉 |
| I2C1 | PB6/PB7 | OLED |
| I2C2 | PB10/PB11 | HWT101 Z 轴角度，SDA=PB11 |

CAN 工程配置保留，当前配置速率 875 kbit/s；实际使用前核对所有节点。SWD 调试器型号和参数以实际连接为准。

USART1 控制台、香橙派 USB B2 和 UART5 电机协议各自独立，不能混用。旧 `sys.ping#`、香橙派 CRC 帧和 B3 圆环编号实现已退出活动工程。

## 工程结构与构建

- `template/Core`：主程序、外设初始化和中断入口。
- `template/Hardware`：电机、Camera、二维码、HWT101、显示、按键及 Flash 驱动。
- `template/App`：机械臂、底盘运动/定位/路线、视觉任务、控制台与显示业务。
- `template/System`：公共时间功能；`Drivers` / `Middlewares`：第三方代码。
- `tests`：固件主机回归；控制台测试在 `upper-computer-web/local-car/tests`。

主要构建环境为 Keil MDK-ARM，备用工程在 `template/MDK-ARM/eide`，配置名同为 `template`。CubeMX 工程为 `template/template.ioc`；只有修改外设配置时才重新生成，生成前检查 USER CODE 和差异。

在工程根目录运行软件回归：

```powershell
python tests/run_firmware_tests.py
node --test upper-computer-web/local-car/tests/*.test.mjs
python -m unittest discover -s upper-computer-web/local-car/tests -p test_bridge.py
```

固件主机测试需要 Python 和 PATH 中的 GCC；网页测试需要 Node.js。上述测试使用主机模拟，不打开实车串口。日志和产物位于 `.embeddedskills/`，不提交到 Git。

## 当前文档与历史

- [控制台使用手册](upper-computer-web/local-car/README.md)
- [当前 USART1 协议](docs/上位机协议.md)
- [资料索引](docs/README.md)：硬件资料、设计基线和历史开发记录。
- [代码规范](docs/开发规范/张大头风格STM32代码规范与AI提示词.md)
- [2026-09-22 USART1 DMA 与无线输出整理](docs/开发记录/2026-09-22USART1-DMA与无线输出整理.md)
- [2026-09-21 清理记录](docs/开发记录/2026-09-21控制台与历史代码清理.md)

2026-09-11 从 `stm32_orangepi_test` 迁入现有业务，迁移事实保留在[迁移记录](docs/开发记录/2026-09-11新PCB与USB通信迁移.md)。2026-09-21 清理退出使用的代码；历史文档保留当时实验结论，旧源码可通过 Git 历史追溯。

## 协作方式

采用需求驱动的 Vibe Coding：用户提出需求、修改建议、关键取舍和实物验收；Codex 负责设计、全部编码、集成、测试和文档。允许主动提出改进和参考适合本项目的优秀开源实现，核查来源、许可证及硬件适配条件。具体授权和硬件操作边界见 [AGENTS.md](AGENTS.md)。
