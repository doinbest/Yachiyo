# 导航结果接口

STM32 上电后自动控制 LDS-50C 完成一次扫描并规划路线。电机模块只在
`g_navigation_result.state == NAV_PATH_READY` 时读取结果。

## 读取顺序

从 `g_navigation_result.path[0]` 开始，依次执行到
`g_navigation_result.path[g_navigation_result.path_count - 1]`。一次规划的结果在复位前保持不变，固件不会自动重新扫描。

每个 `NavWaypoint` 字段含义：

- `x_mm`、`y_mm`：场地绝对坐标，单位毫米。
- `row`、`column`：5×5 网格编号，均从 1 开始。
- `distance_to_next_mm`：到下一个途径点的直线目标距离，单位毫米。
- `direction`：到下一个点的绝对方向，`NORTH/EAST/SOUTH/WEST`。
- `turn`：到达当前点后相对于上一段的动作，`START/STRAIGHT/LEFT/RIGHT/REVERSE`。

最后一点的 `distance_to_next_mm=0`、`direction=NAV_DIR_END`、
`turn=NAV_TURN_END`。电机模块必须以 `path_count` 为数组边界，不能通过坐标值猜测终点。

## 状态与错误

- `NAV_IDLE`：已初始化，尚未开始。
- `NAV_SCANNING`：正在等待并采集一整圈。
- `NAV_PLANNING`：正在生成障碍位图和路线。
- `NAV_PATH_READY`：全部字段已经稳定，可以执行。
- `NAV_ERROR`：流程停止，查看 `error`。

错误码包括雷达/DMA/命令失败、10 秒扫描超时、路径不可达和路径容量溢出。
完整一圈即使只有少量场内点，也会像配套 EXE 一样继续按“单格至少3点”规则
生成障碍位图；不会再用额外的全局点数阈值拒绝空旷或低反射场景。路径不可达时，
`failed_from_task` 与 `failed_to_task` 保存失败任务段的起止编号。

完整一圈采用与配套 EXE 上位机相同的测量包拼接规则：只要当前测量包的
起始角度小于上一测量包的起始角度，就表示跨过 0°。第一次跨零后开始正式
计数，第二次跨零后结束本圈。状态包仅作为雷达配置状态，不参与扫描分圈。
因此即使有效回波没有覆盖到高角度区域，小幅的起始角度下降也能识别为新的一圈。

## 诊断字段

- `cell_point_counts[25]`：按 R1C1、R1C2……R5C5 排列的有效点数。
- `scanned_obstacle_mask`：阈值判断并移除受保护格后的扫描障碍。R1C1
  是雷达/小车起点格，也和任务点 2、3、4、5 一样受保护，近距离车体回波不会把它封死。
- `effective_obstacle_mask`：扫描障碍与四个固定禁区的合成结果。
- `valid_scan_points`：本次完整扫描中进入有效 5×5 场地的点数。
- `rx_*`：USART2 解析器的累计接收字节、有效测量包/状态包、长度错误、
  校验错误、丢弃字节，以及数据包声明过的最大点数。
- `scan_*`：导航层实际收到的测量帧数、原始点数、首末/最小/最大起始
  角度和检测到的跨零次数。这些字段用于区分“串口没有收到数据”“数据包
  被解析器拒绝”和“收到测量包但未形成完整一圈”。

USART3 用作电脑调试输出口，不参与雷达扫描和路径规划。参数为
`115200、8 数据位、无校验、1 停止位`。STM32 的 PB10（USART3_TX）连接
USB 转 TTL 模块 RX，并与模块共地；电脑串口助手选择 ASCII 接收。

程序只在进入 `NAV_PATH_READY` 或 `NAV_ERROR` 后发送一次，格式如下：

```text
NAV,state=PATH_READY,error=NONE,valid=32,scanned=0x00000000,effective=0x00000000,path_count=33
RX,bytes=1234,measurement_packets=7,status_packets=1,bad_length=0,bad_checksum=0,discarded=0
SCAN,frames=7,raw_points=900,first_angle=3500,last_angle=50,min_angle=50,max_angle=3500,wraps=2,max_declared_points=137
RAW,hex=CEFA890000002A...
MAP,processed=1600,accepted=220,angle_reject=0,distance_reject=10,intensity_reject=0,outside=1370
RANGE,min=95,max=3100,status_flags=0x0F,unit_mm=1
CELL,index=1,row=1,column=1,count=0
...
WP,index=1,row=1,column=1,x=230,y=230,distance=558,direction=EAST,turn=START
...
END
```

报告包括 25 个 `CELL` 行以及 `path_count` 个 `WP` 行。电脑应先打开串口，
再复位 STM32；如果错过这次单次报告，重新复位即可。电机模块仍可直接在同一
MCU 内读取全局结构，不依赖 USART3。

## 软件验收记录

`RAW` 行是启动命令完成、旧 DMA 数据丢弃后，新扫描窗口中收到的前 64 个
原始字节（十六进制），用于判断雷达输出是否与协议帧头一致。

`MAP` 行把正式采集的一圈点划分为接受、角度排除、距离排除、强度排除和
场地外五类。`RANGE` 行给出该圈原始距离范围，以及状态包中的原始标志和
距离单位位；`unit_mm=1` 表示雷达上报毫米。

最终 ARM 单元测试结果：原始雷达解析及启动命令 38 项检查全部通过，导航、建图、规划和
USART3 报告 292 项检查全部通过。Keil `Radar` 目标使用 ARMCC 5.06 update 5
全量重建为 0 error、0 warning：`Code=12124`、`RO-data=2284`、
`RW-data=40`、`ZI-data=11544`，总 RAM 约 11.31 KiB，未超过
STM32F103C8Tx 的 20 KiB。

固件构建产物还执行了栈分配检查：`LdsReceiver_Poll` 和
`Navigation_Poll` 不再在 1024 字节系统栈上分配完整雷达帧；路径规划入口的
局部栈为 272 字节。板载雷达、电气连接和现场障碍物仍需按计划中的硬件验收步骤实测。
