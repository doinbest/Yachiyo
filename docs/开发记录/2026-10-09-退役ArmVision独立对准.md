# 2026-10-09 退役 ArmVision 独立对准

## 授权与检查点

用户确认旧 `vision material` 与统一 `grab align` 的对准目标重复，授权删除旧独立Base/X流程及相关入口。删除前建立Git检查点 `406b416`（`chore: checkpoint before retiring ArmVision`）；此前控制台输出与冗余清理已在检查点历史中保存。旧实现仅通过Git追溯，活动工程不留备份。

## 最终行为

- 删除 `Code/template/App/ArmVision.c/.h` 和专用 `Code/tests/arm_vision_test.c`，同步Keil源文件清单和回归入口。
- 删除主循环初始化、周期推进、电机事件消费、忙状态引用和IMU计时段；移除旧标定结果、运动记录、错误输出及OLED旧状态。
- OLED概览页PE2不再启动旧Base/X对准，概览底部不再显示Run提示；PE3选择、PE4/PE5翻页以及舵机测试页PE2选择500 µs行为保留。没有将PE2改成会移动底盘的新入口。
- `vision ref/status/material/calib/stop` 全部退役，命令分发最前面返回 `ERR retired: use grab align color|ring, material ref|calib`。该提示没有摄像头、机械臂、底盘或任务取消副作用；包括旧 `vision stop` 在抓取运行与重启待执行期间均不执行停止或取消。
- 网页命令目录删除旧入口，停止/只读分类同步到现行命令；旧手工命令不会自动转成 `grab align`。

当前静态对准继续使用 `grab align color <1..6>|ring <1..3> [z_mm] [stay]`，保持颜色/编号环独立准心、码垛固定像素补偿、停车确认与缺帧恢复。对准算法、动态追踪、正式抓放与两批任务未因本次退役改变。

## MaterialVision 依赖处理

保留独立 `MaterialVision` 搜索/底盘-X对准及响应标定功能；本次未授权删除这套功能。原先借用 `ArmVision` 的观察姿态参考改为本模块的一个RAM标志：

| 旧入口 | 当前入口 | 行为 |
|---|---|---|
| `vision ref` | `material ref` | 人工确认当前安全观察姿态，不运动、不回零、不充值或清除参数 |
| `vision calib chassis <颜色>` | `material calib <颜色>` | 沿用原底盘/X响应标定算法 |
| `material auto/status/stop` | 保留 | 搜索/状态/停止范围沿用原实现 |

上电初始化参考无效；启动与标定缺参考时不启动运动。普通停止保留参考和标定，忙时确认参考被拒绝且保留原值。状态新增 `ref=0|1`，缺参考明确回复 `ref_required`。与现行 `GrabTask` 互斥通过其真实忙状态维持，不再保留旧模块占位接口。

OLED使用该模块的参考、标定与错误状态，移除已不存在的ArmVision选择分支；同时去掉MaterialVision错误名称表中过时的Home项，使名称与当前枚举一一对应。

## 验证

新增旧命令退役且不产生设备副作用的测试，旧代码运行时先确认断言失败；删除后通过以下检查：

- 18个相关C主机回归通过：`arm_console_host_test`、`material_vision_test`、`oled_ui_test`、`camera_usb_test`、`grab_task_test`、`grab_pose_test`、`grab_stack_test`、`grab_track_task_test`、`grab_track_pick_test`、`grab_command_sim_test`、`grab_transfer_test`、`grab_grace_test`、`contest_mission_test`、`chassis_motion_test`、`chassis_turn_test`、`chassis_route_test`、`mecanum_heading_test`、`radar_console_test`。
- `arm_console_test.py`、`display_schedule_test.py`、`hwt101_upload_test.py`通过。旧模块专用测试删除，相关夹具不再模拟已删除模块。
- 网页 `protocol.test.mjs`、`controls.test.mjs`、`terminal-format.test.mjs`、`console-simulation.test.mjs`共67项通过；最终 `grab.test.mjs` 21项通过，合计88项。串口服务 `test_bridge.py` 共34项通过，全部使用模拟设备。
- Keil `template` build为0错误、0警告；同步源清单后生成AXF/HEX。
- 活动固件、Keil清单与测试不再引用 `ArmVision`；源码差异按CRLF规则检查，无空白错误。未修改或构建EIDE，沿用本项目明确的暂缓约定。

执行期间同一工作区有另一组机械臂参数同步改动，本次保留它们。网页 `grab.test.mjs` 曾有一项默认值精度断言失败：生成值 `x_ppm=31.746031746`，测试期望旧舍入值 `31.746032`；来源为并行改动的 `grab-params.mjs` / `grab-parameter-metadata.mjs`。本次未调整这些参数或该断言；并行任务更新测试后重新运行，最终21/21通过。最后对更新后的共享工作区再次执行Keil构建。

本次测试日志为 `.embeddedskills/tests/retire-armvision-tests.log`、`retire-armvision-entrypoints.log`、`retire-armvision-grab-final.log`、`retire-armvision-bridge.log`；生产构建日志为 `.embeddedskills/build/template-template-build.log`，最后构建结果另存 `retire-armvision-final.json`。产物为 `Code/template/MDK-ARM/template/template.axf` 与同目录HEX，日志/产物不纳入Git。

## 实物范围

未连接串口、发送设备指令、烧录或部署香橙派。上板后核对 `help` 中已无旧入口、手工旧命令只提示退役、OLED概览PE2不启动对准，以及当前 `grab align` 的正常实车流程。软件回归不等同实车验收。
