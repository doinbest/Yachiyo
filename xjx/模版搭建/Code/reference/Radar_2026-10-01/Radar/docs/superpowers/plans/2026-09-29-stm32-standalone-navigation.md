# STM32 Standalone Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 STM32F103C8T6 上电后独立控制 LDS-50C 完成一次 360° 扫描，把有效点流式转换成 5×5 障碍地图，按 `1-5-4-2-3-4-2-3-1` 规划完整路线，并把全部途径点和诊断信息保存在 `g_navigation_result` 中供后续电机模块读取。

**Architecture:** 保留现有 USART2 循环 DMA、`lds_receiver` 和 `lds_parser`，新增纯 C 的定点坐标、网格、单圈扫描和 A* 路径模块，再由一个很薄的 HAL 状态机负责上电启动、超时、停止雷达和发布最终结果。扫描时只累计 25 个格子的点数，不保存整圈点云；路径按 8 个任务段分别规划并拼接，保留任务顺序中的回退。

**Tech Stack:** STM32Cube HAL、C99/ARMCC 5、Keil µVision 5、STM32F103C8T6、USART2 921600 baud、DMA circular RX。

**Spec:** `docs/superpowers/specs/2026-09-29-stm32-standalone-navigation-design.md`

## Global Constraints

- 不修改 PC 上位机，也不让固件依赖电脑或 USART3。
- 不实现 PWM、电机、编码器、PID、运行中重规划或动态避障。
- 不在 RAM 中保存完整点云；只保存可变分辨率待定帧、25 个格计数和最终路径。
- 0°=场地正 Y，90°=场地正 X，角度顺时针增加；坐标单位统一为毫米。
- 固定禁区永远不能被任务格保护逻辑清除；保护逻辑只清除扫描障碍。
- 所有纯算法模块不得包含 HAL 头文件，便于在 `Tests` 中独立验证。
- 目标目录当前不是 Git 仓库。每个任务末尾执行差异/文件清单复核，但不创建仓库、不提交；若用户之后初始化 Git，再按任务边界补提交。

## Review Focus

以下五个不易察觉的失败模式必须由对应任务中的测试覆盖：

1. 可变分辨率帧依赖“下一帧起始角”计算角度步进，若在第二次回绕时处理顺序错误，会漏掉最后一帧或把下一圈点计入本圈——由任务 3 的跨回绕测试覆盖。
2. 点正好落在 `550/1000/1400/1850/2250` 边界时可能被分到错误方格或重复计数——由任务 2 的半开区间边界测试覆盖。
3. 保护任务格时若对最终合成位图统一清位，今后配置调整时可能错误开放固定禁区；实现必须先清扫描位图、再合并固定禁区——由任务 2 的“固定禁区优先”测试覆盖。
4. A* 同代价节点的选取顺序若不确定，路径虽可达但坐标顺序会与上位机不一致，或拼接时删掉任务序列中的合法回退——由任务 4 的精确 33 点序列测试覆盖。
5. `distance * Q15`、启发代价或路径容量计算若使用 16 位中间量会溢出，40000 mm 量程下会产生假坐标或数组越界——由任务 2 和任务 4 的极值/容量测试覆盖。

---

### Task 1: 固化导航公共类型、配置和测试入口

**Files:**

- Create: `Core/Inc/navigation_types.h`
- Create: `Core/Inc/navigation_config.h`
- Create: `Core/Src/navigation_config.c`
- Create: `Tests/test_navigation.c`
- Modify: `MDK-ARM/Radar.uvprojx`

- [x] **Step 1: 写出第一组失败测试**

在 `Tests/test_navigation.c` 沿用现有 `CHECK` 宏，先声明并测试这些不变量：

```c
CHECK(NAV_GRID_ROWS == 5U);
CHECK(NAV_GRID_COLUMNS == 5U);
CHECK(NAV_MAX_WAYPOINTS == 193U);
CHECK(g_nav_grid_x_mm[0] == 150U);
CHECK(g_nav_grid_x_mm[5] == 2250U);
CHECK(g_nav_mission_order[0] == 1U);
CHECK(g_nav_mission_order[8] == 1U);
CHECK((NAV_FIXED_OBSTACLE_MASK & NAV_CELL_MASK(2U, 2U)) != 0U);
CHECK((NAV_PROTECTED_CELL_MASK & NAV_CELL_MASK(3U, 1U)) != 0U);
```

同时为后续测试预留一个 `main()`，依次调用各模块测试函数，失败时返回非零。

- [x] **Step 2: 在测试目标中确认失败**

先把 `NavigationTests` 目标加入 `MDK-ARM/Radar.uvprojx`，只编译纯算法和 `Tests/test_navigation.c`，不链接 HAL 外设代码。执行：

```powershell
& 'C:\Keil_v5\UV4\UV4.exe' -b 'C:\Users\33418\Desktop\Stm32Cube_Code\Radar\MDK-ARM\Radar.uvprojx' -t 'NavigationTests' -j0 -o 'C:\Users\33418\Desktop\Stm32Cube_Code\Radar\MDK-ARM\navigation-tests.log'
```

Expected: 因 `navigation_types.h` / `navigation_config.h` 尚不存在而构建失败；日志中的首个错误应指向缺失接口，而不是工程路径或许可证问题。

- [x] **Step 3: 实现最小公共接口**

`navigation_types.h` 定义：

```c
typedef enum { NAV_IDLE = 0, NAV_SCANNING, NAV_PLANNING, NAV_PATH_READY, NAV_ERROR } NavState;
typedef enum {
  NAV_ERROR_NONE = 0,
  NAV_ERROR_RADAR,
  NAV_ERROR_SCAN_TIMEOUT,
  NAV_ERROR_NOT_ENOUGH_POINTS,
  NAV_ERROR_NO_PATH,
  NAV_ERROR_PATH_OVERFLOW
} NavError;
typedef enum { NAV_DIR_NORTH = 0, NAV_DIR_EAST, NAV_DIR_SOUTH, NAV_DIR_WEST, NAV_DIR_END = 0xFF } NavDirection;
typedef enum { NAV_TURN_START = 0, NAV_TURN_STRAIGHT, NAV_TURN_LEFT, NAV_TURN_RIGHT, NAV_TURN_REVERSE, NAV_TURN_END = 0xFF } NavTurn;

typedef struct {
  uint16_t x_mm;
  uint16_t y_mm;
  uint16_t distance_to_next_mm;
  uint8_t row;
  uint8_t column;
  uint8_t direction;
  uint8_t turn;
} NavWaypoint;

typedef struct {
  NavState state;
  NavError error;
  uint8_t failed_from_task;
  uint8_t failed_to_task;
  uint16_t cell_point_counts[25];
  uint32_t scanned_obstacle_mask;
  uint32_t effective_obstacle_mask;
  uint16_t valid_scan_points;
  uint16_t path_count;
  NavWaypoint path[NAV_MAX_WAYPOINTS];
} NavigationResult;
```

`navigation_config.h/.c` 保存唯一配置源：两组 6 条边界、雷达 `(230,230)`、滤波上下限、阈值 3、10 秒超时、30 个最小有效点、固定/保护掩码、任务格和任务顺序。数组放在 `const` 区，避免占 RW RAM。

- [x] **Step 4: 运行测试并确认通过**

重复 `NavigationTests` 构建，在 µVision Simulator 中运行测试目标到 `main()` 返回；Expected: `all navigation tests passed`，返回值 0。

- [x] **Step 5: 复核接口和内存布局**

确认 `sizeof(NavWaypoint)` 不大于 10 字节，`sizeof(NavigationResult)` 低于 2.5 KB，配置数组位于 RO 区；确认没有包含 `stm32f1xx_hal.h`。

- [x] **Step 6: 保存任务检查点**

记录新增文件清单和 `navigation-tests.log`。当前目录非 Git 仓库，因此不执行 commit。

### Task 2: 实现定点坐标转换和 5×5 网格语义

**Files:**

- Create: `Core/Inc/nav_math.h`
- Create: `Core/Src/nav_math.c`
- Create: `Core/Inc/nav_sine_q15.h`
- Create: `Core/Src/nav_sine_q15.c`
- Create: `Tools/generate_nav_sine_table.py`
- Create: `Core/Inc/grid_map.h`
- Create: `Core/Src/grid_map.c`
- Modify: `Tests/test_navigation.c`
- Modify: `MDK-ARM/Radar.uvprojx`

- [x] **Step 1: 写出定点数学和网格失败测试**

增加以下测试：

- 0°、90°、180°、270°在 1000 mm 距离下分别得到 `(230,1230)`、`(1230,230)`、`(230,-770)`、`(-770,230)`，负坐标必须保留为有符号值并在入格前被拒绝。
- 359.9° 和 0° 连续，无象限跳变。
- 40000 mm 极值计算不溢出 16 位中间量。
- 网格使用半开区间 `[lower, upper)`；`x=550` 属于 C2，`x=2250` 和场外负坐标不属于任何格。
- 2 个点不阻塞，3 个点阻塞。
- 保护格上的扫描障碍被清除，四个固定禁区仍全部保持阻塞；测试直接锁定“先清扫描位图、后合并固定位图”的顺序。

- [x] **Step 2: 运行测试确认红灯**

构建并在 Simulator 运行 `NavigationTests`。Expected: 链接失败于 `NavMath_PolarToCartesian`、`GridMap_FindCell`、`GridMap_BuildMasks` 等尚未实现函数。

- [x] **Step 3: 生成并实现 Q15 四分之一正弦表**

`Tools/generate_nav_sine_table.py` 生成 0.0°～90.0°、步长 0.1°的 901 个 `int16_t` 值到 `nav_sine_q15.c`。运行：

```powershell
python 'C:\Users\33418\Desktop\Stm32Cube_Code\Radar\Tools\generate_nav_sine_table.py'
```

`NavMath_SinQ15(angle_tenths)` 通过象限对称访问表；余弦调用 `sin(angle+900)`。极坐标转换必须使用 `int32_t` 乘法和对称四舍五入：

```c
int32_t x = (int32_t)NAV_RADAR_X_MM + NavMath_MulQ15(distance_mm, sin_q15);
int32_t y = (int32_t)NAV_RADAR_Y_MM + NavMath_MulQ15(distance_mm, cos_q15);
```

- [x] **Step 4: 实现网格索引和障碍位图**

公开最小接口：

```c
uint8_t GridMap_FindCell(int32_t x_mm, int32_t y_mm, uint8_t *row, uint8_t *column);
uint8_t GridMap_CellIndex(uint8_t row, uint8_t column);
void GridMap_CellCenter(uint8_t row, uint8_t column, uint16_t *x_mm, uint16_t *y_mm);
void GridMap_BuildMasks(const uint16_t counts[25], uint32_t *scanned, uint32_t *effective);
uint8_t GridMap_IsBlocked(uint32_t mask, uint8_t row, uint8_t column);
```

合成顺序必须是：

```c
scanned &= ~NAV_PROTECTED_CELL_MASK;
effective = NAV_FIXED_OBSTACLE_MASK | scanned;
```

- [x] **Step 5: 运行测试确认绿灯并检查生成文件**

重新构建并运行 `NavigationTests`。Expected: 所有数学/网格测试通过；`nav_sine_q15.c` 只有 `const` 数据，未生成可写全局表。

- [x] **Step 6: 保存任务检查点**

记录测试日志及生成器命令。当前目录非 Git 仓库，因此不执行 commit。

### Task 3: 流式完成一整圈扫描并直接累计方格

**Files:**

- Create: `Core/Inc/scan_mapper.h`
- Create: `Core/Src/scan_mapper.c`
- Modify: `Tests/test_navigation.c`
- Modify: `MDK-ARM/Radar.uvprojx`

- [x] **Step 1: 写出扫描状态机失败测试**

用人工构造的 `LdsParserResult` 覆盖：

- 固定分辨率帧按 `sector_angle_tenths/(count-1)` 展开，单点帧不除以零。
- 可变分辨率帧先暂存，下一帧到来后用顺时针角差/点数计算步进。
- 启动时的残圈不计数；第一次高角到低角回绕后清零并正式采集；第二次回绕冻结结果。
- 第二次回绕到来时，先完整结算上一圈待定帧，再标记完成，但不计入新圈首帧。
- 角度、距离、能量筛选均在坐标转换前生效。
- 网格外点增加“已解析”统计但不增加 25 格计数；有效入格点只增加一个格。
- 完成后继续推帧不会改变结果。

- [x] **Step 2: 运行测试确认失败**

构建/运行 `NavigationTests`。Expected: 缺少 `ScanMapper_*` 符号。

- [x] **Step 3: 实现显式状态和最小 RAM 上下文**

接口：

```c
typedef enum { SCAN_WAIT_FIRST_WRAP = 0, SCAN_COLLECTING, SCAN_COMPLETE } ScanMapperState;

typedef struct {
  ScanMapperState state;
  uint16_t previous_start_angle_tenths;
  uint8_t have_previous_start;
  uint8_t pending_valid;
  LdsParserResult pending_variable_frame;
  uint16_t cell_point_counts[25];
  uint16_t valid_scan_points;
} ScanMapper;

void ScanMapper_Init(ScanMapper *mapper);
void ScanMapper_PushFrame(ScanMapper *mapper, const LdsParserResult *frame);
uint8_t ScanMapper_IsComplete(const ScanMapper *mapper);
```

为防止 1614 点时 `uint16_t` 足够但未来异常流持续递增，计数和总数采用饱和加法，不允许回卷。

- [x] **Step 4: 明确回绕判定和帧处理顺序**

采用跨越大角度的回绕判定，例如前一帧起始角 `> 3000` 且当前 `< 600`；不能把普通的小幅乱序当作新圈。可变帧处理顺序写成单一辅助函数，确保“结算 pending → 判定/切换圈 → 暂存当前帧”的语义由测试锁定。

- [x] **Step 5: 运行测试确认绿灯**

Expected: 固定/可变帧、两次回绕、滤波和完成冻结全部通过；检查 `sizeof(ScanMapper)`，新增 RAM 主要来自一个待定解析帧而非整圈点云。

- [x] **Step 6: 保存任务检查点**

记录 `sizeof(ScanMapper)` 和测试日志。当前目录非 Git 仓库，因此不执行 commit。

### Task 4: 实现确定性 5×5 A* 和完整任务路线

**Files:**

- Create: `Core/Inc/path_planner.h`
- Create: `Core/Src/path_planner.c`
- Modify: `Tests/test_navigation.c`
- Modify: `MDK-ARM/Radar.uvprojx`

- [x] **Step 1: 写出路径规划失败测试**

测试必须检查完整值而不只检查“有路径”：

- 默认只有四个固定禁区时，`1-5-4-2-3-4-2-3-1` 生成正好 33 个途径点。
- 比较 33 个点的精确 `(row,column)` 顺序，确认 N、E、S、W 邻居顺序和同代价 tie-break 稳定。
- 任务 1 的开头和最终返回使用 `(230,230)`；其他点使用所在格中心。
- 每段连接处仅删除一次相邻重复点；任务序列后续的重复访问和回退必须保留。
- 添加扫描障碍后能绕行；封死任一任务段时返回失败的 `from/to` 任务编号。
- 最后一点距离为 0、方向 `NAV_DIR_END`、动作 `NAV_TURN_END`。
- 直行、左右转、掉头各有至少一个断言。
- 人工设置小容量时返回 overflow，且哨兵字节不被覆盖。
- 代价和欧氏距离使用 32 位中间量，极值平方不会以 16 位计算。

- [x] **Step 2: 运行测试确认失败**

构建/运行 `NavigationTests`。Expected: 缺少 `PathPlanner_BuildMissionPath`。

- [x] **Step 3: 实现单段确定性 A***

使用 25 元固定数组保存 `g_cost`、`f_cost`、父节点、open/closed 标记，不使用 `malloc`。同 `f` 时按较小 `h`、再按更早入队序号选择；扩展邻居固定为北、东、南、西。移动代价为相邻格中心的物理毫米差，启发为中心坐标曼哈顿距离。

- [x] **Step 4: 实现任务段拼接和途径点派生**

公开接口：

```c
uint8_t PathPlanner_BuildMissionPath(uint32_t blocked_mask,
                                     NavigationResult *result);
```

内部先生成格索引序列，再一次性填充 `NavWaypoint`。每次写数组前检查 `path_count < NAV_MAX_WAYPOINTS`。方向由当前格到下一格决定；转向由前后方向差决定；第一个点为 `START`，最后一点覆盖成结束标记。

- [x] **Step 5: 运行测试确认绿灯**

Expected: 默认精确 33 点、绕行、不可达、回退、转向和 overflow 测试全部通过。

- [x] **Step 6: 保存任务检查点**

把测试中锁定的默认 33 格序列同步写入测试注释，作为后续上位机/单片机一致性基准。当前目录非 Git 仓库，因此不执行 commit。

### Task 5: 实现可测试的导航流程和错误优先级

**Files:**

- Create: `Core/Inc/navigation.h`
- Create: `Core/Src/navigation.c`
- Modify: `Core/Inc/lds_receiver.h`
- Modify: `Core/Src/lds_receiver.c`
- Modify: `Tests/test_navigation.c`
- Modify: `MDK-ARM/Radar.uvprojx`

- [x] **Step 1: 写出状态机失败测试**

将雷达动作与时间通过小型依赖接口注入测试替身，覆盖：

- `Navigation_Start()` 清空旧结果并进入 `NAV_SCANNING`，只启动一次雷达。
- 扫描完成后只调用一次停止，随后进入 `NAV_PLANNING` 再到 `NAV_PATH_READY`。
- 10 秒超时使用无符号 tick 差值，在 `HAL_GetTick()` 回卷附近仍正确。
- 完整圈少于 30 个有效点返回 `NAV_ERROR_NOT_ENOUGH_POINTS`。
- DMA/命令错误返回 `NAV_ERROR_RADAR`。
- 无路径时保存 `failed_from_task` / `failed_to_task`。
- 停止雷达命令若失败，不覆盖此前更具体的超时、点数不足或无路径错误。
- 进入 `NAV_PATH_READY` 或 `NAV_ERROR` 后继续轮询不再修改结果或重启扫描。

- [x] **Step 2: 运行测试确认失败**

构建/运行 `NavigationTests`。Expected: 缺少 `Navigation_*` 符号。

- [x] **Step 3: 收紧接收器对外接口**

不再让导航模块直接依赖 `g_lds_parser` 的内部布局；在 `lds_receiver.h/.c` 增加：

```c
uint8_t LdsReceiver_ReadMeasurement(LdsParserResult *result);
uint8_t LdsReceiver_ReadStatus(LdsParserStatus *status);
LdsReceiverStatus LdsReceiver_GetStatus(void);
```

保留现有全局符号以避免无关破坏，但新导航代码只调用函数接口。`LdsReceiver_ConfigureAndStart()` 开始前重置命令错误，且任一命令失败立即停止后续命令发送。

- [x] **Step 4: 实现导航控制器和结果发布**

`navigation.h` 只公开：

```c
extern NavigationResult g_navigation_result;
void Navigation_Init(void);
void Navigation_Start(void);
void Navigation_Poll(void);
```

结果发布顺序为：复制 25 格计数和有效点数 → 生成位图 → 规划路线 → 最后写状态。这样调试器看到 `NAV_PATH_READY` 时所有字段已经稳定。错误处理集中到一个 helper，保证只记录第一个根因，并尽力停止雷达。

- [x] **Step 5: 运行测试并确认绿灯**

Expected: 启动、完整圈、tick 回卷、点数不足、雷达失败、无路径、停止失败优先级和终态冻结全部通过。

- [x] **Step 6: 保存任务检查点**

记录状态迁移表和测试日志。当前目录非 Git 仓库，因此不执行 commit。

### Task 6: 接入上电主循环、LED 和 Keil 固件目标

**Files:**

- Modify: `Core/Src/main.c`
- Modify: `MDK-ARM/Radar.uvprojx`
- Create: `docs/navigation-result-interface.md`

- [x] **Step 1: 先建立固件构建基线**

在接入前执行：

```powershell
& 'C:\Keil_v5\UV4\UV4.exe' -b 'C:\Users\33418\Desktop\Stm32Cube_Code\Radar\MDK-ARM\Radar.uvprojx' -t 'Radar' -j0 -o 'C:\Users\33418\Desktop\Stm32Cube_Code\Radar\MDK-ARM\radar-before-integration.log'
```

Expected: 0 errors。若现有工程本身失败，先记录基线问题，不把它误认为导航改动导致。

- [x] **Step 2: 接入上电自动流程**

在 CubeMX `USER CODE` 区域内：

```c
Navigation_Init();
Navigation_Start();
```

主循环只调用 `Navigation_Poll()` 并按状态更新 LED：扫描中按固定周期翻转，`NAV_PATH_READY` 熄灭，`NAV_ERROR` 常亮。删除原先直接根据解析包计数控制 LED 的逻辑。USART3 初始化保留但不收发导航数据。

- [x] **Step 3: 把全部新增 `.c` 加入 `Radar` 目标**

在 `Radar.uvprojx` 中加入 `navigation_config.c`、`nav_math.c`、`nav_sine_q15.c`、`grid_map.c`、`scan_mapper.c`、`path_planner.c`、`navigation.c`；确认头文件路径仍只需要 `../Core/Inc`。

- [x] **Step 4: 构建固件并消除所有警告**

```powershell
& 'C:\Keil_v5\UV4\UV4.exe' -b 'C:\Users\33418\Desktop\Stm32Cube_Code\Radar\MDK-ARM\Radar.uvprojx' -t 'Radar' -j0 -o 'C:\Users\33418\Desktop\Stm32Cube_Code\Radar\MDK-ARM\radar-navigation-build.log'
```

Expected: 0 errors、0 warnings。检查生成的 `.map`：RW+ZI 必须低于 20 KB，RO 必须低于 64 KB；路径数组、解析器和 DMA 缓冲区均能在 map 中定位。

- [x] **Step 5: 写电机模块读取说明**

`docs/navigation-result-interface.md` 说明：

- 仅在 `state == NAV_PATH_READY` 时读取。
- 从 `path[0]` 到 `path[path_count-1]` 顺序执行。
- `x_mm/y_mm` 是绝对场地坐标，`distance_to_next_mm` 是到下一个途径点的目标距离。
- `direction/turn` 的枚举含义、最后一点结束标记、错误码和失败任务段。
- 数据在复位前保持不变；本阶段不会自动重扫或发送 USART3。

- [x] **Step 6: 保存任务检查点**

保存两个构建日志和 map 内存摘要。当前目录非 Git 仓库，因此不执行 commit。

### Task 7: 最终回归与硬件验收

**Files:**

- Modify: `Tests/test_navigation.c`（仅在回归暴露缺口时）
- Modify: `docs/navigation-result-interface.md`（记录最终实测值）

- [x] **Step 1: 执行纯算法完整回归**

重建并在 Simulator 运行 `NavigationTests`。Expected: 原有 `test_lds_parser.c` 测试仍通过，新增导航测试全部通过，没有被跳过的断言。

- [x] **Step 2: 执行最终固件构建**

清理后重建 `Radar` 目标；Expected: 0 errors、0 warnings，并记录最终 Program Size 的 Code/RO/RW/ZI。

- [ ] **Step 3: 下载到板并验证一次扫描**

硬件连接 LDS-50C 到 USART2，复位 MCU。用 Keil Watch 观察：

```text
g_navigation_result.state
g_navigation_result.error
g_navigation_result.valid_scan_points
g_navigation_result.cell_point_counts
g_navigation_result.scanned_obstacle_mask
g_navigation_result.effective_obstacle_mask
g_navigation_result.path_count
g_navigation_result.path
```

Expected: 10 秒内进入 `NAV_PATH_READY`；雷达完成一圈后停止；`path_count` 在 1～193；最后一点距离为 0；没有写 USART3。

- [ ] **Step 4: 验证默认场地基准**

在没有额外扫描障碍进入有效格时，Expected: 只含四个固定禁区，路径为 33 点，首尾均 `(230,230)`。若现场墙面产生扫描障碍，则路径允许变化，但任务顺序、固定禁区和保护格语义必须不变。

- [ ] **Step 5: 验证三类故障**

- 断开雷达或制造命令失败：进入 `NAV_ERROR_RADAR`。
- 波特率错误/无完整圈：10 秒后进入 `NAV_ERROR_SCAN_TIMEOUT`。
- 有完整圈但少于 30 个有效入格点：进入 `NAV_ERROR_NOT_ENOUGH_POINTS`。

每种故障均应停止自动流程、LED 常亮、诊断字段保留且不自动重试。

- [x] **Step 6: 最终范围审计**

确认没有改动上位机目录，没有加入电机/PWM/编码器控制，没有把点云数组写入 RAM，没有改变 USART3 行为。列出所有修改文件和实测内存数据交给用户。当前目录非 Git 仓库，因此不执行 commit。
