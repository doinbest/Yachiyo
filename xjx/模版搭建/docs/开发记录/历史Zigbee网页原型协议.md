# 历史 Zigbee 网页原型协议（已退役）

> 2026-09-21：对应网页及旧实现已移出活动工程。以下保留当时协议设计，不是当前硬件操作说明。当前接口见[USART1 协议](../上位机协议.md)，旧实现从 Git 历史追溯。

## 历史链路与帧格式

- MCU 接口：USART2（PA2 TX、PA3 RX）
- 当前参数：115200 bit/s、8 数据位、无校验、1 停止位
- 物理链路：Zigbee 透明串口
- 编码：ASCII
- 命令结束符：`#`；固件也接受换行作为结束符
- 最大命令长度：96 字节（不含结束符）

上位机命令采用 `命令名=参数1,参数2#` 的格式。该格式参考 RoboWalker
串口绘图模块的可读变量赋值思路，扩展为适合电机控制的多参数命令。

固件响应：

- 成功接收并分发：`@ok=命令名#`
- 参数或命令错误：`@err=命令名,原因#`
- 握手：`@hello=robocar-stepper,1,115200#`
- CAN 电机返回帧：`@can=扩展帧ID,DLC,十六进制数据#`

`@ok` 只表示 MCU 已接受命令并提交给 CAN 发送层，不等价于电机已经运动到位。
要判断电机实际状态，应读取位置、速度、误差或状态标志。

## 控制命令

| 命令 | 参数 | 示例 |
| --- | --- | --- |
| `sys.ping` | 无 | `sys.ping#` |
| `stream.can` | `0/1` 关闭或开启 CAN 返回帧透传 | `stream.can=1#` |
| `motor.enable` | 地址，使能 `0/1` | `motor.enable=1,1#` |
| `motor.speed` | 地址，有符号 RPM，加速度 `0~255` | `motor.speed=1,-300,20#` |
| `motor.move` | 地址，有符号脉冲，RPM，加速度，同步标志 | `motor.move=1,3200,300,20,0#` |
| `motor.stop` | 地址 | `motor.stop=1#` |
| `motor.stop_all` | 无（广播地址 0） | `motor.stop_all#` |
| `motor.sync` | 无（广播同步启动） | `motor.sync#` |
| `motor.zero` | 地址（当前位置清零） | `motor.zero=1#` |
| `motor.home` | 地址，回零模式 `0~3`，同步标志 | `motor.home=1,0,0#` |
| `motor.home_abort` | 地址 | `motor.home_abort=1#` |
| `motor.read` | 地址，参数名 | `motor.read=1,vel#` |

可读参数名：`vbus`、`cbus`、`cpha`、`clkc`、`vel`、`cpos`、`perr`、
`flag`、`oflag`。

## 最小联调顺序

1. 电机脱离负载，确认 CAN 速率、终端电阻、供电和电机 ID。
2. 浏览器选择 Zigbee USB 串口，使用 115200 bit/s 连接。
3. 发送 `sys.ping#`，应收到 `@hello=robocar-stepper,1,115200#`。
4. 发送 `motor.enable=1,1#`，再发送低速命令 `motor.speed=1,50,20#`。
5. 确认转向和状态后发送 `motor.stop=1#`。
6. 最后再验证位置、同步和回零；回零前必须先确认机械限位及电机端参数。

当前固件先采用主循环轮询接收，便于最小化验证。若后续加入阻塞任务、持续遥测
或实测出现丢包，再将 USART2 接收升级为空闲中断/DMA，并保持本命令层接口不变。
