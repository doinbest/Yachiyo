# F407主车固件

固定X/Z任务采用余弦速度规划，默认Z抬升K150、下降与X伸缩K200；场景速度上限、绝对终点和停稳确认保留，详见[机械臂协议](../../docs/协议/机械臂与取放.md)。

2026-10-14：[ 比赛视觉航向分支 ](../../docs/开发记录/2026-10-14-比赛视觉航向分支实现与验证.md)，默认关闭；尚未标定时开启只观测。

STM32F407VGT6，STM32 HAL裸机工程。入口为 `Core/Src/main.c`；业务在 `App/`，硬件驱动在 `Hardware/`，HAL/CMSIS等第三方代码在 `Drivers/`。工程规则见[AGENTS](../../AGENTS.md)，接线/单位见[硬件基线](../../docs/硬件与运行基线.md)。

## 构建与验证

当前路线加速度可在控制台“底盘 → 底盘单段运动测试”独立验证：八方向、合距离和合速度可选，复用普通直线的半余弦K=200及短段适配。串口接口为 `chassis test <forward_mm> <left_mm> <speed_mm_s>`；从当前位置建立参考后即可测试，无需执行比赛。操作见[地图与雷达](../upper-computer-web/local-car/docs/地图与雷达.md#单独测试当前路线加速度)。

统一两批比赛调度已实现，当前需求和后续修订见[比赛与取放约定](../../docs/比赛与取放约定.md)，早期设计与计划在该文档保留导航。车仓X收回、加工与暂存第一层空爪预对准后取料放置、第二层持料码垛及阶段续接已集成；软件能力不代表整场实车通过。

PD12 / TD-8125MG雷达盘舵机由[lidar_servo.c](Hardware/lidar_servo.c)控制，TIM4_CH1、50 Hz、上电零脉宽。`lidarservo out`为实测2450 μs转出扫描，`lidarservo in`为1450 μs收回车内；保留`pulse <500..5000>`与`status`，原off接口已删除。后续出发前展开采图、完成后收回的基线见[雷达协议](../../docs/协议/雷达.md#pd12-雷达盘舵机)；比赛雷达分支由[mission_navigation.c](App/mission_navigation.c)自动展开、采集规划、收回稳定后交给比赛路线出发，独立手动操作仍保留。

Keil打开 `MDK-ARM/template.uvprojx`，Target `template`。CubeMX文件为 `template.ioc`，仅在明确的外设配置需求下重新生成。当前只维护Keil，`MDK-ARM/eide`未同步、不构建，不作为可用备用入口。

仓库根运行 `python Code/tests/run_firmware_tests.py`（需要Python和GCC）；雷达线协议检查为 `python Code/tests/radar_wire_contract_test.py`。这些是主机验证，不操作设备。烧录按实际探针及用户授权执行。用户2026-10-12确认香橙派当前程序无需在STM32烧录后重开；本地视觉源码、远端部署和实车验证分别报告。

上电/复位等待1秒后，Z/X碰撞回零与ID8就近回零并行；X/Z反馈验证完成后先开爪2500 μs，待ID8建立参考后再启动Base单圈方向回零（mode=1，负相对方向）。仅RAM调整回零模式/方向，读回核对，其余驱动参数保留。全部机构完成后安排IMU静止5秒验证；grab rehome复用该顺序，单轴回零不开爪。近似参考、短暂失能复查及停止行为见[硬件基线](../../docs/硬件与运行基线.md)和[机械臂协议](../../docs/协议/机械臂与取放.md)。

## 按需求定位

路线直线平移在 `chassis_motion.c` 中采用半余弦速度过渡，`CHASSIS_DISTANCE_COSINE_K=200` 对应名义400 ms；短段降低峰值，制动同步核算余弦积分距离与反馈延迟。驱动档位仍为0，位置与停稳确认独立。正式右下开局采用共享 `chassis_path.h` 的右通道曲线：保持180°到(2125,350)，600 mm内转至(2100,950,90°)后直行；开局上限200 mm/s，保留原前瞻、向量变化率和航向限速。第一批从右侧保持90°横移到物料台中间(1200,2100)，到点停稳后原地顺时针转90°至0°；第二批中央进站曲线保留。开局接近段尚未改为余弦控制。详见[比赛与取放约定](../../docs/比赛与取放约定.md)。

普通雷达扫描统一为15个有效完整圈；`radar scan lower 15` 采集并规划，`radar live map start lower` 持续更新最近15圈，`radar stop` 停止采集。下方默认情况二 `(2280,314)` 和精确Q16标定；车体起点为 `(2250,150,-180°)`，工位、首段接驳和取放流程保留。上方独立雷达安装为 `(2050,2250,270°)`，与右上车辆名义起点 `(2250,2250)` 分开，不套用下方矩阵。稳定点XY是独立结果，原始点云仅来自最后一圈诊断。

机构实物条件见[硬件与运行基线](../../docs/硬件与运行基线.md)：Base作业为原参考+130°、扫码为原参考+35°，车仓方向为原参考；ID8仓1/2/3默认0°/+90°/+180°，与Base及车体yaw独立。源码默认、烧录版本及设备RAM分别核对。

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

Base 回零默认启用近似参考降级继续：只放行单独 HOME 失败，连续两轮新鲜位置稳定、实际转速 0 RPM、正常使能及驱动状态后继续。`base_home_degrade=0` 可切回严格模式；状态持续报告 `base_home_degraded=1`。局部人工接受与误差影响见[机械臂协议](../../docs/协议/机械臂与取放.md#base-回零降级继续)。

## 统一比赛入口

2026-10-13增加[比赛动作连续衔接](../../docs/开发记录/2026-10-13-比赛动作连续衔接.md)：末件入仓直接结束在行驶姿态，实时X/Z避让完成后重叠Base与底盘平移，复用进站姿态；路线busy保存原目标重试，相容共线纯途经点合并。串口入口不变，主机回归与实车验证分别记录。

车仓选仓共用`Turntable_Index()`：1号仓执行ID8单圈就近回零，2/3号仓按最短方向定位。仓1按回零完成、新鲜位置稳定及零速/使能反馈确认后继续，不依赖普通位置到位标志；不清库存或重建机械臂参考。入仓、取出和比赛流程同时适用，参见[实施记录](../../docs/开发记录/2026-10-13-车仓1号采用就近回零.md)。实现与各次下载证据见开发记录，车仓实物验收另记。

`grab mission qr [speed] [turn_deg_s] [lower|upper] [fixed|radar] [lowered|raised]` 或 `grab mission fixed <code> [speed] [turn_deg_s] [lower|upper] [fixed|radar] [lowered|raised]`，省略选项默认路线200 mm/s、转向100°/s、右下、固定路线、raised（Z=0高位对准）；低位对照须显式指定lowered。两种来源都真实扫码，屏幕只显示实际码，采用码独立冻结。`grab mission pause`及`grab stop`暂停，`grab mission stop`终止，`grab mission continue`停稳后续接；终止停车未确认时不能新开比赛。完整字段、取放和恢复见[现行约定](../../docs/比赛与取放约定.md)及[协议](../../docs/协议/机械臂与取放.md)，软件、烧录和实车结果分别记录。

加工区第三件取回入仓采用ID8直接270°避让、Z提起到位后回180°，回转可与Base并行；角度随仓3配置计算，暂停保留目标。见[实施记录](../../docs/开发记录/2026-10-14-加工第三件车仓避让.md)。后续编译及DAP烧录校验已通过，实车验证另记。
