# 上位机与 STM32 配置协议

这个协议是本项目为“把上位机配置和规划结果下载到 STM32”定义的，不是 LDS-50C 原厂协议。STM32 固件需要按本文实现对应接收端。

所有多字节整数均为小端。

## 通用帧

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 2 | 固定前缀 `A5 5A` |
| 2 | 1 | 协议版本，当前为 1 |
| 3 | 1 | 命令 |
| 4 | 2 | 序号 `sequence` |
| 6 | 2 | 负载长度 `N`，上限 4096 |
| 8 | N | 负载 |
| `8 + N` | 2 | CRC-16/CCITT-FALSE |

CRC 计算范围从版本字段开始，覆盖 `version + command + sequence + length + payload`，不含 `A5 5A` 和 CRC 本身。参数：多项式 `0x1021`，初值 `0xFFFF`，无输入/输出反转，`xorout = 0x0000`；字符串 `123456789` 的校验值为 `0x29B1`。

## 命令

| 值 | 名称 | 方向及含义 |
|---:|---|---|
| `0x10` | StageConfiguration | 上位机发送 47 字节配置，STM32 暂存但不立即生效 |
| `0x11` | ApplyConfiguration | 上位机要求原子应用已暂存配置 |
| `0x12` | SaveConfiguration | 用户点击独立按钮后，要求 STM32 保存到非易失存储 |
| `0x13` | ReadConfiguration | 上位机请求；STM32 用同命令、同序号返回当前配置 |
| `0x20` | SetPath | 上位机发送路径 |
| `0x7E` | Ack | STM32 确认成功，序号必须与请求一致 |
| `0x7F` | Nack | STM32 拒绝；负载可放 UTF-8 原因文本 |

上位机严格执行：`StageConfiguration → ApplyConfiguration → SetPath → ReadConfiguration`。每步等待同序号 ACK，默认超时 2 秒；乱序、重复、NACK、超时或读回字段不一致都会终止流程并显示错误。

“保存到 STM32 Flash”是独立操作，只发送空负载的 `SaveConfiguration` 并等待 ACK。下载和调参时不会自动写 Flash，以避免频繁擦写。

## 配置负载 v1

固定 47 字节：

| 顺序 | 类型 | 内容 |
|---:|---|---|
| 1 | `u8` | 负载版本 1 |
| 2 | `u16 × 2` | 雷达 X、Y，mm |
| 3 | `u16 × 4` | 四条内部 X 线，mm |
| 4 | `u16 × 4` | 四条内部 Y 线，mm |
| 5 | `u16 × 2` | 最小、最大角度，0.1° |
| 6 | `u16 × 2` | 最小、最大距离，mm |
| 7 | `u8 × 2` | 最小、最大能量 |
| 8 | `u16` | 单格扫描障碍点数阈值 |
| 9 | `u32` | 固定禁区位图 |
| 10 | `u32` | 手动禁区位图 |
| 11 | `u8 × 3` | 起点 row、column、anchor |
| 12 | `u8 × 3` | 终点 row、column、anchor |

25 位地图位图中，bit 索引为 `(row - 1) × 5 + (column - 1)`；row 1 位于场地下方，column 1 位于左侧。`anchor`：0 为格中心，1 为雷达原点，2 为右下启停区 1 的特殊终点。

## 路径负载 v1

```text
u8  payloadVersion = 1
u16 entryCount
repeat entryCount:
    u8  row
    u8  column
    u16 xMm
    u16 yMm
u16 segmentCount              # 必须等于 entryCount - 1
repeat segmentCount:
    u16 distanceMm
    u8  direction
    u8  turn
```

`direction`：0 北、1 东、2 南、3 西。`turn`：0 起步、1 直行、2 左转、3 右转、4 掉头。路径只允许上下左右四邻域移动，但允许重复经过同一格和原路回退。单个 v1 路径负载最多包含 409 个路径点和 408 段，编码后恰为 4091 字节，不超过通用帧的 4096 字节负载上限；`entryCount = 410` 或更大的负载必须拒绝。

STM32 应先完整验证前缀、长度、CRC、版本、范围和序号，再写入运行参数；不要在接收半帧时修改控制状态。执行运动时，仍需由底盘固件把 `distanceMm` 换算为编码器脉冲，并用闭环控制修正误差。
