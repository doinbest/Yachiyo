# 物料台转盘

这是独立的 STM32F103C8T6 转盘控制工程，与 F407 主车分别构建和下载。

| 位置 | 用途 |
|---|---|
| `turntable/` | 当前维护的固件工程 |
| `turntable/MDK-ARM/turntable.uvprojx` | Keil 入口，Target `turntable` |
| `turntable/README.md` | 按键、分度、电机参数、测试和历史反馈模块说明 |
| `archives/物料台转盘.zip` | 原项目根目录的旧上电直发版压缩包，入口为 `Turntable_BootInit/Process`；原内容保留 |

当前入口为双按键直接控制 `Turntable_DirectInit/Process`，不等待回包，详情以 [转盘使用说明](turntable/README.md) 顶部“当前入口”为准。压缩包缺少当前 direct 模块，不能用它覆盖或重建当前工程。

在仓库根目录运行独立主机回归：

```powershell
python Code/物料台转盘/turntable/tests/run_tests.py
```

需要 Python、GCC；模拟测试不操作电机，日志写入 `.embeddedskills/turntable/`。下载必须单独核对 F103 的实际调试器和目标参数，不套用主车 `.embeddedskills/config.json` 的 F407 配置。
