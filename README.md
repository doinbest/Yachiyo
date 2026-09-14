# STM32F407VGT6 工训赛事搬运小车固件

本项目用于开发以 STM32F407VGT6 为主控的工训赛事搬运小车固件。2026-09-11 起，后续开发回到本目录，已从 `stm32_orangepi_test` 迁入现有业务，并适配新 PCB 通信引脚。

当前接线、迁移范围、USB 协议和联调方法见 [新 PCB 与 USB 通信迁移](docs/开发记录/2026-09-11新PCB与USB通信迁移.md)。香橙派配套程序为 [orange_pi/change.py](orange_pi/change.py)。

当前显示、按键、命令与验证入口见 [工程整理与 OLED 三页使用说明](docs/开发记录/2026-09-11工程整理与OLED三页.md)。OLED 使用总览、视觉、通信诊断三页；物料视觉仍处于关闭高层超时的 `Test` 模式，Z 轴默认加速度为 20。

2026-09-12 新增独立底盘速度任务、UART5 四轮同步事务与 [本地二维地图](upper-computer-web/local-car/README.md)。使用入口、参数确认和验证结果见 [底盘同步与二维地图对齐记录](docs/开发记录/2026-09-12底盘同步与二维地图对齐.md)。上电默认关闭反馈轮询和地图遥测；新底盘任务须先确认实际驱动器应答配置。

项目当前处于基础模板和功能验证阶段。开发过程以学习、理解和可验证为核心：每次只完成一个较小的功能，由项目作者负责主要代码编写，Codex 负责资料整理、原理讲解、方案讨论、代码审查和调试协助。

## 硬件与工程环境

- 主控：STM32F407VGT6
- MCU：STM32F407VGT6，LQFP100，1 MB Flash
- 配置工具：STM32CubeMX 6.17.0
- 主要构建环境：Keil MDK-ARM
- 备用构建环境：EIDE
- 调试接口：SWD，具体调试器在连接硬件后确定

当前 USART1（PA9/PA10）用于 Zigbee 控制台，USART2（PA2/PA3）预留雷达，USART3（PD8/PD9）用于串口屏，UART4（PC10/PC11）用于二维码，UART5（PC12/PD2）用于步进电机。香橙派使用原生 USB CDC（PA11/PA12），OLED 使用 I2C1（PB6/PB7），HWT101 使用 I2C2（PB10/PB11）。CAN 配置保持现状。

## 开发方式

每个功能按以下过程逐步完成：

1. 明确功能目标、硬件连接、输入、输出和预期现象。
2. 阅读 ST 官方资料、嘉立创技术文档以及所用模块的编程手册。
3. 将功能拆分为能够单独编译、观察和验证的小步骤。
4. 先用流程图、时序说明或伪代码确认思路。
5. 由项目作者编写并集成主要代码；必要时由 Codex 提供少量参考代码。
6. 审查代码后再进行编译、烧录和硬件联调。
7. 记录实际现象、问题原因和最终结论。

参考代码不会默认直接写入工程。项目作者应先理解、审查和修改，再手动复制到工程中。只有在明确要求 Codex 修改文件时，才会直接更改业务代码。

## 文档与代码原则

- 优先采用清晰、直接、容易单步验证的实现。
- 避免在单个函数中同时承担按键处理、通信、状态判断、控制输出和界面刷新等过多职责。
- 不为了“架构完整”而过早引入复杂状态机、深层封装或较长调用链。
- 所有公开的自研模块接口应逐步补充 Doxygen 风格说明。
- 第三方 HAL、CMSIS 和芯片支持文件保持原样，不进行批量注释改写。
- 文档中的明确事实应注明资料来源；尚未由文档或实验确认的内容应标为推测或待验证项。

## 工程结构

- `template/template.ioc`：STM32CubeMX 配置
- `template/Core`：启动代码、主程序和外设初始化
- `template/Drivers`：CMSIS 与 STM32 HAL 驱动
- `template/Hardware`：电机、姿态、香橙派链路、屏幕和按键驱动
- `template/App`：麦轮底盘、机械臂、视觉流程、控制台、比赛串口屏和 OLED 三页显示
- `template/System`：时间与无业务含义的公共能力
- `template/MDK-ARM/template.uvprojx`：Keil 工程
- `template/MDK-ARM/eide`：EIDE 工程
- `.embeddedskills/config.json`：构建、调试和通信工具的项目级配置
- `AGENTS.md`：Codex 在本项目中的协作规则

## 构建

使用 Keil uVision 打开 `template/MDK-ARM/template.uvprojx`，选择 `template` Target 后进行构建。

需要修改引脚、时钟或外设配置时，使用 STM32CubeMX 打开 `template/template.ioc`。重新生成代码前应检查 USER CODE 区域和工程差异，避免覆盖已经编写的内容。

构建产物、调试状态、串口日志和 CAN 日志不提交到 Git。

主机回归测试：在工程根目录运行 `python tests/run_firmware_tests.py`，需要 Python 和 PATH 中的 GCC。测试日志及可执行文件位于 `.embeddedskills/tests/`，不包含真实电机或通信硬件操作。

## 第一版比赛软件底座

本节记录历史版本：四组三位/六色任务码、香橙派 USART1 CRC16 接收、最小任务状态和统一安全停止。相关文件保留，但当前 Target 已采用迁入的机械臂、视觉和二维码业务，USB 使用 `Camera` 的 B2 协议。请以顶部的新 PCB 迁移记录为当前入口。

新增文件、协议字节、Keil Watch 变量、无电机测试和架空底盘步骤见 `docs/开发记录/第一版智能搬运车软件底座实现与使用手册.md`。

## Zigbee 网页上位机原型

仓库保留早期基于 USART2 的 Zigbee 网页上位机原型和 `docs/上位机协议.md`。新 PCB 的 Zigbee 已改为 USART1，当前 MCU 使用迁入的 `arm_console` 文本命令。历史网页协议和占位代码不能视为当前 MCU 接口说明。

网页工程位于 `upper-computer-web/`，采用浏览器 Web Serial API。请使用桌面版
Chrome 或 Edge，通过 HTTPS 发布地址或本机开发服务器访问；直接双击本地 HTML
文件不能可靠获得 Web Serial 安全上下文。

2026-09-12 本地控制台已改版为 **八千代·巡航**，新增六模块导航、公共终端及 UART4 二维码缓存查询/跟随。见 [控制台使用说明](upper-computer-web/local-car/README.md) 与 [本轮开发记录](docs/开发记录/2026-09-12八千代控制台与二维码接入.md)。二维码桥接需本轮固件；尚未实物验收。
