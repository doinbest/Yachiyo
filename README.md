# STM32F407VGT6 工训赛事搬运小车

本项目包含搬运小车 STM32 固件、香橙派视觉代码、串口屏配置、本地网页控制台及独立物料台转盘固件。主车主控为 STM32F407VGT6（LQFP100，1 MB Flash），主要使用 STM32 HAL 裸机主循环；转盘使用独立 STM32F103C8T6。

现有功能包括机械臂与夹爪、麦轮底盘、HWT101 航向验证与保持、B2 颜色搜索/对准、二维码任务码、串口屏、OLED、地图遥测和实车16段路线。视觉对准和路线到点不等于完整搬运任务完成；软件回归不替代实物验收。

LDS50C 已接入 USART2：F407 静止采一整圈、筛选建图并规划工位路线；网页“地图与规划”沿用当前二维地图，提供雷达图层、右键放大、设备参数与同一 C 算法的离线对照。扫描与计算不会自动运动，运行路线须显式启动；原实车16段路线继续可用。坐标对照、分页协议及上板顺序见[雷达融合实现与联调](docs/开发记录/2026-10-02-雷达融合实现与联调.md)。

新增静止红色单件抓取：固定姿态动作验证、底盘/X协同对准（蓝色物料）、单件抓取（红色物料），以及前四段模拟扫码联动。通过网页视觉页或`grab`命令使用；配置缺项不运动，抓取提起后Base转−180°、下降放置、松爪并提起后等待人工验收；上电自动Z/X碰撞回零、Base就近回零。详见[实现与标定](docs/开发记录/2026-09-27-单件抓取实现与标定.md)。所有视觉任务统一使用原 B2 协议。2026-10-02 用户提供的最新视觉源码已保存为 `Code/orange_pi/vision.py`，旧视觉脚本已删除；本次仅更新本地源码，未部署香橙派。用户已确认 `align` 完整验证，本轮仅分析 `pick` 与其流程对齐及漏检容错，详见[分析记录](docs/开发记录/2026-10-02-视觉源码更新与align-pick容错分析.md)。协议约定见[ B2 恢复说明](docs/开发记录/2026-09-29-B2协议兼容恢复.md)。

2026-10-02 后续修复了取放后参考缺项状态未及时更新的问题，并精简抓取终端中文摘要。普通停止和总线恢复保留配置，参考变化后使用“重新建立三轴参考”；本次未修改视觉对准和漏检恢复策略。详见[修复记录](docs/开发记录/2026-10-02-抓取参考诊断与终端精简.md)。

## 项目目录与入口

```text
模版搭建/
├─ Code/                    代码与软件工程
│  ├─ template/             F407 主车固件
│  ├─ orange_pi/            香橙派视觉代码
│  ├─ HMI/                  串口屏工程与字库
│  ├─ 物料台转盘/            独立 F103 固件与原始压缩包
│  ├─ upper-computer-web/   本地控制台
│  ├─ reference/            GongXun 与雷达参考代码
│  ├─ tests/                F407 主机回归
│  └─ tools/                共用工具，含雷达C规划离线入口
├─ docs/                    正式资料、协议与开发记录
├─ .embeddedskills/         本地配置、日志与分析产物
├─ tmp/                     临时分析文件
├─ AGENTS.md                协作规则
└─ README.md                项目入口
```

| 入口 | 用途 | 启动/使用 |
|---|---|---|
| [Code/template/](Code/template) | STM32 固件 | Keil 打开 `Code/template/MDK-ARM/template.uvprojx`，Target 为 `template` |
| [Code/orange_pi/](Code/orange_pi/README.md) | 香橙派本地视觉源码 | 最新入口 `vision.py`，使用 B2；本地源码更新与实机部署分别确认 |
| [Code/HMI/](Code/HMI/README.md) | 串口屏配置 | 用对应串口屏编辑软件打开 `Code/HMI/display.HMI`，字库跟随工程 |
| [Code/物料台转盘/](Code/物料台转盘/README.md) | 独立 F103 转盘固件 | Keil 打开 `Code/物料台转盘/turntable/MDK-ARM/turntable.uvprojx`，Target `turntable` |
| [local-car](Code/upper-computer-web/local-car/README.md) | 八千代·巡航本地控制台 | Windows 双击 `Code/upper-computer-web/local-car/start.cmd` |
| [Code/reference/](Code/reference/README.md) | 外部参考项目与验证样例 | 按目录索引阅读；不加入当前主车构建 |

全部代码统一放入 `Code/`，模块内部入口与相对结构保持；代码索引见 [Code/README](Code/README.md)。`Code/reference/GongXun2025-main/` 保留原控制、视觉、机械与报告资料；雷达两份源码快照也在 `Code/reference/`。原根目录 `物料台转盘.zip` 移至 `Code/物料台转盘/archives/`，它是旧上电直发版本，不能替代当前双按键代码。

控制台需要 Python 3、pyserial；缺少时执行 `python -m pip install pyserial`。服务启动后用桌面 Chrome/Edge 打开 `http://127.0.0.1:8765/`。无需 npm 安装、旧 React 页面或云部署。启动不会自动连接串口。

网页共享模式需从设备列表手动选择串口，无默认 COM 号；通信格式默认 115200 / 8N1，实际串口应现场核对，它与 `.embeddedskills/config.json` 的工具串口配置相互独立。准备、实车运动和停车流程详见控制台手册。

上电后保持整车静止：固件默认 Emm42 Receive 应答，自动等待 IMU 新鲜数据并执行 5 秒验证；通过后才取得航向控制资格。失败时在系统页重新验证，地图起点仍需准备跑图确认。见[本次变更记录](docs/开发记录/2026-09-22默认Receive与上电IMU验证.md)。

## 硬件与通信

| 接口 | 引脚 | 当前用途 |
|---|---|---|
| USART1 | PA9/PA10 | 无线文本控制台，ASCII/CRLF |
| USART2 | PA2/PA3 | LDS50C，921600/8N1，循环RX DMA |
| USART3 | PD8/PD9 | 串口屏 |
| UART4 | PC10/PC11 | 二维码 |
| UART5 | PC12/PD2 | 步进电机共享总线 |
| USB CDC | PA11/PA12 | 香橙派 B2 视觉 |
| I2C1 | PB6/PB7 | OLED |
| I2C2 | PB10/PB11 | HWT101 Z 轴角度，SDA=PB11 |
| TIM1_CH1 | PE9 | 270°夹爪舵机控制信号，50 Hz PWM |

CAN1/CAN2 当前停用，工程配置保留；将来启用时重新确认节点与速率。SWD 调试器型号和参数以实际连接为准。

板载 PE4/PE5 可在 OLED 页面间翻页。进入 `4/4 Servo Test` 后保持 Off，PE2 选择松开 500 µs，PE3 选择抓紧 700 µs；PE5 开启/关闭输出，PE4 退出并清零输出。USART1 `grip open`、`grip catch` 与网页“打开”“夹取”使用相同的两个实测脉宽；`grip idle` 也取松开位置。上电与仅进入页面不会驱动舵机；信号消失后是否保持力由舵机型号及供电决定。700 µs 为用户确认的完全抓紧位置，不继续增加脉宽顶住机构。

USART1 控制台、香橙派 USB B2 和 UART5 电机协议各自独立，不能混用。旧 `sys.ping#`、香橙派 CRC 帧和 B3 圆环编号实现已退出活动工程。

## 工程结构与构建

- `Code/template/Core`：主程序、外设初始化和中断入口。
- `Code/template/Hardware`：电机、Camera、二维码、HWT101、显示、按键及 Flash 驱动。
- `Code/template/App`：机械臂、底盘运动/定位/路线、视觉任务、控制台与显示业务。
- `System` 职责保留为无业务含义的公共功能，当前未使用的延时模块已删除；`Drivers` / `Middlewares`：第三方代码。
- `Code/tests`：固件主机回归；控制台测试在 `Code/upper-computer-web/local-car/tests`。

主要构建环境为 Keil MDK-ARM，备用工程在 `Code/template/MDK-ARM/eide`，配置名同为 `template`。CubeMX 工程为 `Code/template/template.ioc`；只有修改外设配置时才重新生成，生成前检查 USER CODE 和差异。

2026-10-02 精简只更新 Keil 清单；EIDE 按用户要求暂不维护，仍有已删除模块的旧路径及原有 SPI 缺项，未验证可用。张大头 Emm 与 W25Q128 驱动完整保留，详见[固件精简记录](docs/开发记录/2026-10-02-STM32主车固件精简.md)。

控制台的 `grab set` 和 `config <axis> ...` 修改 RAM 参数，受理后供后续控制使用；STM32 复位/重新上电恢复已烧录固件默认值。网页保存的路线速度只在浏览器本地保存，修改网页预填值不会改写固件默认值。当前没有这些参数的 Flash 保存与上电加载功能；保留 Flash 驱动不等于调参已经掉电保存。

在工程根目录运行软件回归：

```powershell
python Code/tests/run_firmware_tests.py
node --test Code/upper-computer-web/local-car/tests/*.test.mjs
python -m unittest discover -s Code/upper-computer-web/local-car/tests -p 'test*.py'
python Code/tests/radar_wire_contract_test.py
```

固件主机测试需要 Python 和 PATH 中的 GCC；网页测试需要 Node.js。上述测试使用主机模拟，不打开实车串口。日志集中保存到 `.embeddedskills/`，其中仅提交 `config.json`；Keil/EIDE 也会在各工程内生成被 Git 忽略的编译输出。

转盘回归独立运行 `python Code/物料台转盘/turntable/tests/run_tests.py`；HMI 使用屏幕编辑软件编译。当前 F407 的默认构建和下载参数不用于转盘。

## 当前文档与历史

- [控制台使用手册](Code/upper-computer-web/local-car/README.md)
- [当前 USART1 协议](docs/上位机协议.md)
- [资料索引](docs/README.md)：硬件资料、设计基线和历史开发记录。
- [代码规范](docs/开发规范/张大头风格STM32代码规范与AI提示词.md)
- [2026-09-22 USART1 DMA 与无线输出整理](docs/开发记录/2026-09-22USART1-DMA与无线输出整理.md)
- [2026-09-21 清理记录](docs/开发记录/2026-09-21控制台与历史代码清理.md)

2026-09-11 从 `stm32_orangepi_test` 迁入现有业务，迁移事实保留在[迁移记录](docs/开发记录/2026-09-11新PCB与USB通信迁移.md)。2026-09-21 清理退出使用的代码；历史文档保留当时实验结论，旧源码可通过 Git 历史追溯。

## 协作方式

采用需求驱动的 Vibe Coding：用户提出需求、修改建议、关键取舍和实物验收；Codex 负责设计、全部编码、集成、测试和文档。允许主动提出改进和参考适合本项目的优秀开源实现，核查来源、许可证及硬件适配条件。具体授权和硬件操作边界见 [AGENTS.md](AGENTS.md)。

## 电机总线恢复入口（2026-09-27）

误操作或通信锁定后，可在本地控制台“系统与标定”或“视觉任务 → 单件抓取”点击“一键恢复电机总线”（`bus recover`），用 `bus status` 查询结果。恢复先停止任务、逐台检查全部七台驱动器，成功后不自动续跑；底盘同步缓存仍不确定时保留底盘运动限制。详见 [电机总线恢复与手动操作检查](docs/开发记录/2026-09-27-电机总线恢复与手动操作检查.md)。


## 近期开发记录

以下记录保留修改原因、参数、当时验证和烧录状态；日期记录不自动代表当前实机版本。

| 日期 | 内容与记录 |
|---|---|
| 2026-09-29 | [控制台中文化、停车确认与通信启动](docs/开发记录/2026-09-29-控制台中文化与通信启动改进.md) |
| 2026-09-30 | [视觉对准提速依据与模型对比](docs/开发记录/2026-09-30-蓝色协同对准参数提速与对比.md)：默认 20mm/s、0.6s⁻¹、20mm/s²；模拟不能作为实车性能结论 |
| 2026-09-30 | [蓝色对准短暂漏检容忍](docs/开发记录/2026-09-30-蓝色协同对准短暂漏检容忍.md)：250ms 容忍、连续 3 次有效坐标恢复、停稳后最多等待 1500ms 重新识别；保持香橙派程序 |
| 2026-09-30 | [抓取 Z 轴默认 250RPM](docs/开发记录/2026-09-30-抓取Z轴默认250RPM.md)：按 480 命令脉冲/mm 换算为 27.777778mm/s，可使用 grab set/get 临时调速 |
| 2026-10-02 | [项目目录整理](docs/开发记录/2026-10-02-项目目录整理.md)：参考工程归档、模块入口和协作规范 |
| 2026-10-02 | [STM32 主车固件精简](docs/开发记录/2026-10-02-STM32主车固件精简.md)：Git 检查点、未使用模块与兼容接口清理、参数保存边界 |
| 2026-10-02 | [雷达融合实现与联调](docs/开发记录/2026-10-02-雷达融合实现与联调.md)：USART2、整圈建图、当前坐标规划、网页图层与同 C 核心离线对照 |
