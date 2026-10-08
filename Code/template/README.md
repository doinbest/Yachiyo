# F407主车固件

STM32F407VGT6，STM32 HAL裸机工程。入口为 `Core/Src/main.c`；业务在 `App/`，硬件驱动在 `Hardware/`，HAL/CMSIS等第三方代码在 `Drivers/`。工程规则见[AGENTS](../../AGENTS.md)，接线/单位见[硬件基线](../../docs/硬件与运行基线.md)。

## 构建与验证

Keil打开 `MDK-ARM/template.uvprojx`，Target `template`。CubeMX文件为 `template.ioc`，仅在明确的外设配置需求下重新生成。当前只维护Keil，`MDK-ARM/eide`未同步、不构建，不作为可用备用入口。

仓库根运行 `python Code/tests/run_firmware_tests.py`（需要Python和GCC）；雷达线协议检查为 `python Code/tests/radar_wire_contract_test.py`。这些是主机验证，不操作设备。烧录按实际探针及用户授权执行，之后提醒重开香橙派程序再进行视觉实测。

## 按需求定位

| 需求 | 主要源码 |
|---|---|
| 初始化/调度 | [main.c](Core/Src/main.c) |
| 控制台命令 | [arm_console.c](App/arm_console.c)、[radar_console.c](App/radar_console.c) |
| 抓取/对准/码垛 | [GrabTask.c](App/GrabTask.c)、[参数定义](App/grab_parameters.def) |
| 动态追踪计算 | [grab_tracking.c](App/grab_tracking.c) |
| 正式任务 | [contest_mission.c](App/contest_mission.c) |
| 底盘运动/路线 | [chassis_motion.c](App/chassis_motion.c)、[chassis_route_profile.c](App/chassis_route_profile.c) |
| 雷达地图/规划 | [radar_map.c](App/radar_map.c)、[radar_start.c](App/radar_start.c)；规划入口从控制台调用链定位 |
| 物料搜索/静态对准所有者 | [GrabTask.c](App/GrabTask.c)；响应采样 [MaterialVision.c](App/MaterialVision.c) |
| 车载ID8转盘 | [turntable.c](App/turntable.c) |
| UART5共享事务 | [motor_bus.c](Hardware/motor_bus.c)、[Emm_V5.c](Hardware/Emm_V5.c) |
| 屏幕业务 | [contest_screen.c](App/contest_screen.c)、[OLED](App/oled_ui.c) |

`ArmVision`和旧 `vision` 命令已退役；静态对准用 `grab align color|ring`。`material auto/ref/calib/status/stop` 保留兼容，搜索及标定由 GrabTask 拥有，MaterialVision 只提供响应采样和兼容状态。搜索保持人工确认的观察姿态，对准共用静态控制器；标定写入静态/动态共用 J。持续速度、定时保航向、MoveTo/转向由 chassis_motion 推进，mecanum_chassis 负责总线执行。源码默认值、设备RAM读回与浏览器预填分别判断；机械参数与抓取参数定义不能从旧开发记录推断。
