# F407主车固件

STM32F407VGT6，STM32 HAL裸机工程。入口为 `Core/Src/main.c`；业务在 `App/`，硬件驱动在 `Hardware/`，HAL/CMSIS等第三方代码在 `Drivers/`。工程规则见[AGENTS](../../AGENTS.md)，接线/单位见[硬件基线](../../docs/硬件与运行基线.md)。

## 构建与验证

Keil打开 `MDK-ARM/template.uvprojx`，Target `template`。CubeMX文件为 `template.ioc`，仅在明确的外设配置需求下重新生成。当前只维护Keil，`MDK-ARM/eide`未同步、不构建，不作为可用备用入口。

仓库根运行 `python Code/tests/run_firmware_tests.py`（需要Python和GCC）；雷达线协议检查为 `python Code/tests/radar_wire_contract_test.py`。这些是主机验证，不操作设备。烧录按实际探针及用户授权执行，之后提醒重开香橙派程序再进行视觉实测。

上电/复位等待1秒后自动启动 Z/X 碰撞回零、Base/ID8车仓就近回零。共享总线按应答逐条发送，先发出三轴HOME再逐轴验证，不等待前一个轴完成才启动下一个；ID8独立调度并行运行。用户已确认空爪上电时四者同时运动无机械干涉。各机构按实际反馈建立参考，全部完成后才安排 IMU 静止5秒验证。回零完成时短暂未使能仍先复查600 ms，位置稳定且无驱动故障时仅补一次使能并在1500 ms内核验。停止先中断所有仍在回零的机构，再读取退出状态；停车未确认不会伪报成功。`grab rehome` 使用并行三轴回零，已完成的上电ID8回零不重放；手动回零、状态和位置入口保留。

## 按需求定位

普通雷达扫描统一为15个有效完整圈；`radar scan lower 15` 采集并规划，`radar live map start lower` 持续更新最近15圈，`radar stop` 停止采集。下方默认情况二 `(2280,314)` 和精确Q16标定；车体起点仍为 `(2250,150)`，工位、首段接驳和取放流程保留。上方保持独立安装 `(2250,2250,270°)`，不套用下方矩阵。稳定点XY是独立结果，原始点云仅来自最后一圈诊断。

坏帧、缺段和DMA覆盖只回滚当前圈。更改标定或筛选需新15圈；阈值和路线参数可用匹配的完整稳定证据重算。CPU工作区及稳定点传输副本使用CCM，接收DMA仍在普通SRAM。操作与协议见[地图与雷达](../upper-computer-web/local-car/docs/地图与雷达.md)、[雷达协议](../../docs/协议/雷达.md)。

离线程序 `Code/tools/radar_plan_cli.c` 支持原有 `RADAR1` 历史单圈语义，以及 `RADAR2 mask circle_count`：随后为原10项参数、情况编号、4项矩阵Q16、2项平移Q16、零角0.1°，再依次为每圈点数及 `[angle_tenths distance_mm energy]`。输入至少15个有效完整圈，超过15圈按最近15圈计算；自动双起点输入为15圈。离线程序调用同一投影、稳定证据和规划C核心，不操纵串口。

| 需求 | 主要源码 |
|---|---|
| 初始化/调度 | [main.c](Core/Src/main.c) |
| 控制台命令 | [arm_console.c](App/arm_console.c)、[radar_console.c](App/radar_console.c) |
| 抓取/对准/码垛 | [GrabTask.c](App/GrabTask.c)、[参数定义](App/grab_parameters.def) |
| 动态追踪计算 | [grab_tracking.c](App/grab_tracking.c) |
| 正式任务 | [contest_mission.c](App/contest_mission.c) |
| 底盘运动/路线 | [chassis_motion.c](App/chassis_motion.c)、[chassis_route_profile.c](App/chassis_route_profile.c) |
| 雷达投影/稳定证据 | [radar_geometry.c](App/radar_geometry.c)、[radar_stability.c](App/radar_stability.c)，与离线程序共用纯C核心 |
| 雷达采集/地图/规划 | [radar_scan.c](App/radar_scan.c)、[radar_map.c](App/radar_map.c)、[radar_start.c](App/radar_start.c) |
| 静态对准/响应标定所有者 | [GrabTask.c](App/GrabTask.c)；响应采样 [grab_calibration.c](App/grab_calibration.c) |
| 车载ID8转盘 | [turntable.c](App/turntable.c) |
| UART5共享事务 | [motor_bus.c](Hardware/motor_bus.c)、[Emm_V5.c](Hardware/Emm_V5.c) |
| 屏幕业务 | [contest_screen.c](App/contest_screen.c)、[OLED](App/oled_ui.c) |

`ArmVision`和旧 `vision` 命令已退役；静态对准用 `grab align color|ring`。`MaterialVision`、后退搜索和全部 `material` 命令已退役，旧命令只返回提示。响应标定使用 `grab calib ref` / `grab calib <1..6>` / `grab calib status`，由 GrabTask 调度；`grab stop` 取消并保留旧模型。标定保持当前 Base/Z/夹爪姿态，成功写入静态/动态共用 J 的前后与 X 两列，侧向列保留。持续速度、定时保航向、MoveTo/转向由 chassis_motion 推进，mecanum_chassis 负责总线执行。源码默认值、设备RAM读回与浏览器预填分别判断；机械参数与抓取参数定义不能从旧开发记录推断。
