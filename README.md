# STM32F407VGT6 工训赛事搬运小车固件

本项目用于开发以 STM32F407VGT6 为主控的工训赛事搬运小车固件，外设与引脚配置沿用参考 VET6 工程。

项目当前处于基础模板和功能验证阶段。开发过程以学习、理解和可验证为核心：每次只完成一个较小的功能，由项目作者负责主要代码编写，Codex 负责资料整理、原理讲解、方案讨论、代码审查和调试协助。

## 硬件与工程环境

- 主控：STM32F407VGT6
- MCU：STM32F407VGT6，LQFP100，1 MB Flash
- 配置工具：STM32CubeMX 6.17.0
- 主要构建环境：Keil MDK-ARM
- 备用构建环境：EIDE
- 调试接口：SWD，具体调试器在连接硬件后确定

当前 CubeMX 工程已启用 CAN1、CAN2、TIM1、UART4、UART5、USART1、USART2、USART3 和 USART6。第一版代码将 UART5 用于 X42S/Emm_V5 电机总线、UART4 用于现有 JY61P 临时验证模块、USART1 用于香橙派可靠帧接收、USART3 用于淘晶驰屏。CAN 配置中的当前计算速率仍需分别结合时钟树、总线节点和模块手册复核。

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
- `template/App`：麦轮底盘、任务码、任务入口和安全策略
- `template/System`：时间与无业务含义的公共能力
- `template/MDK-ARM/template.uvprojx`：Keil 工程
- `template/MDK-ARM/eide`：EIDE 工程
- `.embeddedskills/config.json`：构建、调试和通信工具的项目级配置
- `AGENTS.md`：Codex 在本项目中的协作规则

## 构建

使用 Keil uVision 打开 `template/MDK-ARM/template.uvprojx`，选择 `template` Target 后进行构建。

需要修改引脚、时钟或外设配置时，使用 STM32CubeMX 打开 `template/template.ioc`。重新生成代码前应检查 USER CODE 区域和工程差异，避免覆盖已经编写的内容。

构建产物、调试状态、串口日志和 CAN 日志不提交到 Git。

## 第一版比赛软件底座

当前已加入 2027 四组三位/六色任务码、香橙派 USART1 带 CRC16 帧接收、最小任务状态和统一安全停止。自动比赛路线、X42S 到位反馈、HWT101 正式驱动和雷达尚未实现，香橙派 `START` 帧不会启动电机。

新增文件、协议字节、Keil Watch 变量、无电机测试和架空底盘步骤见 `docs/开发记录/第一版智能搬运车软件底座实现与使用手册.md`。

## Zigbee 网页上位机原型

仓库保留了基于 USART2（PA2/PA3、115200 bit/s、8N1）的 Zigbee 网页上位机原型和 `docs/上位机协议.md`。当前 `template/Hardware/usart2(zigbee - pid)` 仍是占位代码，且没有加入 Keil Target，因此不能把网页中存在命令控件理解为 MCU 命令层已经可用。

网页工程位于 `upper-computer-web/`，采用浏览器 Web Serial API。请使用桌面版
Chrome 或 Edge，通过 HTTPS 发布地址或本机开发服务器访问；直接双击本地 HTML
文件不能可靠获得 Web Serial 安全上下文。
