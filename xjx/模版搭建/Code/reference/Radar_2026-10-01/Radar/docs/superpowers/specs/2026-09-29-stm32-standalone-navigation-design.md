# STM32 单机雷达建图与路径规划设计

## 目标

STM32F103C8T6 上电后脱离电脑独立运行：通过 USART2 控制 LDS-50C 雷达，采集一整圈有效测量数据，在本机生成 5×5 障碍地图，按照固定任务顺序 `1-5-4-2-3-4-2-3-1` 规划路线，并把全部途径点保存到全局结果数组中。后续电机与编码器模块只需按数组顺序读取并执行。

本阶段不实现电机 PWM、编码器闭环、姿态修正，也不修改现有上位机。

## 已有基础

- MCU：STM32F103C8T6，72 MHz，64 KB Flash，20 KB RAM。
- USART2：921600 baud，连接 LDS-50C，RX 使用 256 字节循环 DMA。
- USART3：115200 baud，当前不参与导航算法，保留给以后调试或执行机构通信。
- `lds_parser` 已支持可变分辨率帧、固定分辨率帧、状态帧、校验和与错误计数。
- 当前固件约占 5.5 KB ROM、3.8 KB RAM。

## 固定场地参数

- 场地：2400×2400 mm。
- 有效网格边界：`150、550、1000、1400、1850、2250 mm`。
- 雷达旋转中心：`X=230 mm，Y=230 mm`，位于 R1C1。
- 雷达角度定义：0° 指向场地正 Y，90° 指向场地正 X，顺时针增加。
- 筛选：角度 0°～360°，距离 100～40000 mm，能量 0～255。
- 扫描障碍阈值：同一方格至少 3 个有效点。
- 固定禁区：R2C2、R2C4、R4C2、R4C4。
- 任务点：1=雷达精确位置；2=R3C1 中心；3=R5C3 中心；4=R3C5 中心；5=R1C3 中心。
- 任务点 2～5 所在方格受保护，扫描点不能把这些格设为障碍。

## 启动与数据流

1. HAL、GPIO、USART2、USART3 和雷达 DMA 初始化。
2. 初始化导航状态和 25 个方格计数。
3. 向雷达依次发送停止、毫米单位、能量输出、滤波和启动命令。
4. 接收启动时的残缺圈，直到角度第一次从高值回绕到低值；此时清零统计并进入正式采集。
5. 从第一次回绕到第二次回绕之间的数据构成唯一一次正式扫描。
6. 每个点到达时立即完成角度插值、筛选、坐标转换、方格定位和计数，不保存完整点云。
7. 第二次回绕时冻结方格计数并停止雷达。
8. 合成固定禁区和扫描障碍，清除受保护任务格上的扫描障碍。
9. 按固定任务顺序分段规划，拼接为一条完整路径。
10. 填充全局结果数组并将状态置为 `NAV_PATH_READY`。结果保持不变，直到复位或以后显式重新扫描。

## 模块划分

### `navigation_config.h`

保存全部编译期参数：边界、雷达位置、筛选范围、障碍阈值、固定禁区、受保护方格、任务点顺序、扫描超时和路径容量。

### `scan_mapper.c/.h`

- 接收 `LdsParserResult`。
- 与上位机相同地处理固定分辨率与可变分辨率帧。
- 可变分辨率帧暂存到下一帧到达，以便计算扇区跨度。
- 检测角度回绕并管理“等待首个回绕/正式扫描/扫描完成”状态。
- 使用 0.1° 定点角度和 Q15 四分之一正弦查表计算坐标，避免接收期间反复调用软件浮点三角函数。
- 对落在有效网格外的点只忽略，不写数组外区域。
- 只维护 25 个 `uint16_t` 方格计数和有效点总数。

坐标公式与上位机一致：

```text
x = radar_x + distance × sin(angle)
y = radar_y + distance × cos(angle)
```

### `grid_map.c/.h`

- 把 25 个点数转换成扫描障碍位图。
- 合并固定禁区位图。
- 对任务点 2～5 应用保护掩码。
- 提供行列与位索引转换、方格中心坐标和阻塞查询。

### `path_planner.c/.h`

- 对 25 个方格使用固定长度数组实现确定性的 A*。
- 邻居顺序保持为北、东、南、西，只允许四邻接移动。
- 移动代价使用相邻方格中心的实际毫米距离；启发值使用到目标中心的曼哈顿毫米距离。
- 为每一对任务点单独规划；除第一段外，拼接时跳过后一段重复的首格。
- 保留路线回退造成的非相邻重复访问。
- 起点和最终返回点使用雷达精确坐标 `(230,230)`，其余点使用方格中心。
- 根据相邻方格生成北、东、南、西方向，以及起步、直行、左转、右转、掉头动作。
- 距离使用两点欧氏距离四舍五入到整数毫米。

### `navigation.c/.h`

- 负责上电自动流程和状态机。
- 调用现有 `LdsReceiver_Poll()`，读取解析完成的测量和状态帧。
- 完整扫描后调用 `LdsReceiver_Stop()`，再生成地图和路径。
- 对外提供只读的 `g_navigation_result`。

### `main.c`

- 初始化阶段调用 `Navigation_Init()` 和 `Navigation_Start()`。
- 主循环调用 `Navigation_Poll()`。
- LED：扫描时闪烁、路径完成时熄灭、错误时常亮。

## 结果数据结构

```c
typedef enum
{
  NAV_IDLE = 0,
  NAV_SCANNING,
  NAV_PLANNING,
  NAV_PATH_READY,
  NAV_ERROR
} NavState;

typedef enum
{
  NAV_ERROR_NONE = 0,
  NAV_ERROR_RADAR,
  NAV_ERROR_SCAN_TIMEOUT,
  NAV_ERROR_NOT_ENOUGH_POINTS,
  NAV_ERROR_NO_PATH,
  NAV_ERROR_PATH_OVERFLOW
} NavError;

typedef struct
{
  uint16_t x_mm;
  uint16_t y_mm;
  uint16_t distance_to_next_mm;
  uint8_t row;
  uint8_t column;
  uint8_t direction;
  uint8_t turn;
} NavWaypoint;

typedef struct
{
  NavState state;
  NavError error;
  uint16_t cell_point_counts[25];
  uint32_t scanned_obstacle_mask;
  uint32_t effective_obstacle_mask;
  uint16_t valid_scan_points;
  uint16_t path_count;
  NavWaypoint path[NAV_MAX_WAYPOINTS];
} NavigationResult;
```

固定任务序列共有 8 段。每段最多经过 25 格，去掉段连接处重复点后理论上限为 193 点，因此 `NAV_MAX_WAYPOINTS=193`。默认无额外障碍时应生成 33 个点。

最后一个路径点的 `distance_to_next_mm=0`，方向和动作使用专用结束值，电机模块以 `path_count` 判断数组终点。

## 内存与实时性

- 不保存整圈原始点云。
- 新增方格计数约 50 字节。
- 路径数组因结构体对齐预计不超过约 2.4 KB。
- 可变分辨率帧暂存约 0.8 KB。
- 正弦四分之一查表约 1.8 KB Flash，不占运行时 RAM。
- 预计总 RAM 仍显著低于 20 KB；构建后必须用 Keil map 文件复核 RW/ZI 总量。

## 错误处理

- 雷达命令或 DMA 初始化失败：`NAV_ERROR_RADAR`。
- 启动后 10 秒内未得到一整圈：`NAV_ERROR_SCAN_TIMEOUT`。
- 完整圈有效点少于 30：`NAV_ERROR_NOT_ENOUGH_POINTS`。
- 任一任务段无法连通：`NAV_ERROR_NO_PATH`，同时保存失败段的起止任务编号。
- 拼接结果超过容量：`NAV_ERROR_PATH_OVERFLOW`。
- 进入错误状态后停止雷达、保持诊断数据并点亮 LED，不自动重复扫描。

## 测试与验收

纯算法模块不依赖 HAL，并使用 `Tests` 目录中的 C 测试验证：

1. 0°、90°、180°、270°坐标转换。
2. 固定与可变分辨率角度插值。
3. 首个回绕只负责对齐，第二个回绕才结束正式扫描。
4. 网格边界点、场外点和距离筛选。
5. 三点阈值、四个固定禁区与四个受保护任务格。
6. 默认空地图生成 33 个路径点，起止坐标均为 `(230,230)`。
7. 路径包含与上位机相同的重复访问顺序。
8. 障碍绕行与不可达任务段。
9. 路径点距离、方向、转向和最终结束标记。
10. 超时、点数不足与路径容量保护。

硬件验收时通过 Keil 调试器查看 `g_navigation_result`：状态必须进入 `NAV_PATH_READY`，并可逐项检查方格计数、障碍位图、`path_count` 和所有路径点。雷达在完成一圈后必须停止发送扫描数据。

## 非目标

- 不实现电机驱动、编码器计数、运动学或 PID。
- 不在行驶期间重新扫描或动态避障。
- 不把完整原始点云保存到 RAM 或 Flash。
- 不修改 PC 上位机或让 STM32 依赖电脑启动。
