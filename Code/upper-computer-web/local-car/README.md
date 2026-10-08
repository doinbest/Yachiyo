# 八千代·巡航本地控制台

本地Python串口服务与浏览器操作台，共享F407 USART1无线链路；网页提供任务操作、参数读回、地图和诊断。串口命令与完成语义见[协议索引](../../../docs/上位机协议.md)。

## 启动与连接

1. 安装Python 3和pyserial；缺少时执行 `python -m pip install pyserial`。
2. 双击本目录 `start.cmd` 或上一级 `启动控制台.lnk`，也可在本目录运行 `python serve.py`。
3. 用Chrome/Edge访问 `http://127.0.0.1:8765/`，刷新串口列表，人工选择实际设备并连接，默认115200/8N1。

服务只监听回环地址，无需npm安装或云部署；不要直接双击HTML。启动不自动连串口，页面切换、刷新和重连不发送运动或雷达启动命令。快捷方式移动项目后须调整路径。退出服务前停止任务并确认反馈，再断开串口；服务终端输入 `quit`。

## 操作入口

| 页面 | 用途 | 详细说明 |
|---|---|---|
| 系统与标定 | 信息、IMU原生标定/验证、回零及故障恢复 | [连接与恢复](docs/连接与恢复.md) |
| 底盘 | 平移/转向测试、四轮反馈与路线操作 | [地图与雷达](docs/地图与雷达.md) |
| 地图与遥测 | 实车监控、F1雷达建图、同C核心离线预演 | [地图与雷达](docs/地图与雷达.md) |
| 二维码 | 最新码/跟随及四组三位任务码 | [系统协议](../../../docs/协议/系统与底盘.md) |
| 视觉任务 | 编号环、物料取放、追踪夹取、路线任务、仅识别及码垛 | [视觉任务](docs/视觉任务.md) |
| 参数调试 | 读取/应用设备RAM参数 | [视觉任务](docs/视觉任务.md) |
| 终端与日志 | 手动命令、过滤、导出及停车状态 | [连接与恢复](docs/连接与恢复.md) |

`accepted`或串口写出不代表动作完成；停车请求与反馈停稳分别显示。网页断开也不证明车辆停车。表单预填、本机保存和MCU读回不同；网页不会因重连自动写参数或续跑。

## 开发与验证

`app.mjs`组织界面，`protocol.mjs`定义命令，`parameter-schema.mjs`及相关模块管理参数；`serve.py`/`serial_bridge.py`管理共享服务及串口队列。F1雷达页读取设备冻结结果，不另写规划算法；保留的直连雷达模块只供独立诊断。

抓取参数默认值/范围由[grab_parameters.def](../../template/App/grab_parameters.def)维护，仓库根运行 `python Code/tools/sync_grab_parameters.py`生成网页元数据。不要手写第二套默认值。

仓库根运行相关检查：

```powershell
node --test Code/upper-computer-web/local-car/tests/*.test.mjs
python -m unittest discover -s Code/upper-computer-web/local-car/tests -p 'test*.py'
```

测试使用FakeSerial/内存流，不连接实车发送模拟运动。部署及实车验收按当时记录单独判断。历史变化见[开发记录](../../../docs/开发记录/README.md)和[整理前手册](../../../docs/开发记录/2026-10-09-文档整理前快照/原控制台手册.md)。
