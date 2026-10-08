# MaterialVision退役与控制台保留

修改前将未提交的控制整合及文档内容保存为Git快照 `23987b7`。本轮按[批准计划](../superpowers/plans/2026-10-09-materialvision-retirement.md)删除旧后退搜索和全部material入口，保留响应标定与现有控制台功能。

## 实现

- 删除MaterialVision.c/.h、搜索状态、兼容转换、专用错误推送和重复互锁；Keil清单改为grab_calibration.c/.h。
- GrabTask管理标定启动、轮询、反馈和停车；grab_calibration提供响应采样和只读结果。原参数保持：五个新鲜样本中位数、底盘20 mm/s前进/返回各1秒、X电机5°/8 RPM、150 ms停稳。
- 实际轮位置反馈决定前后分母，实际X角度决定X分母。成功一次提交j00/j10及换算后的j02/j12，侧向列不变；失败、USB暂缺数据、重复启动和取消不覆盖旧有效矩阵。
- `grab calib ref`仅人工确认当前姿态；`grab calib <1..6>`不安排Base/Z/夹爪动作；`grab calib status`不发送设备请求，现有忙状态和网页停止锁定期间可读。取消使用grab stop和现有停车确认。
- 网页“系统与标定”添加确认、颜色标定和状态查询控件。OK grab calib回复使用独立快照，普通任务状态继续grab state格式。切页、刷新和重连不启动或重放标定。
- camera material/ring/status/stop及独立坐标输出保留；OLED改为GrabTask任务、视觉坐标、通信及首个故障显示。底盘标定占用检查仅针对标定，保持路线期间已有机械臂操作及停稳后的人工夹爪/高度功能。

## 软件验证

| 检查 | 结果与证据 |
|---|---|
| 修改前基线 | 固件主机通过，网页485/485，Python81/81；material-retirement-baseline-*.log |
| 固件主机全量 | PASS；`.embeddedskills/material-retirement-firmware.log` |
| 标定服务与真实GrabTask联动 | 实际反馈换算、矩阵提交/保留、X边界、无Base/Z/夹爪附带动作、USB异常/恢复、重复启动、取消通过 |
| 命令分发 | 新查询在机械臂/底盘/路线/IMU/转盘/雷达/恢复/手动操作忙状态下可读；旧命令无动作；独立相机输出及人工夹爪/高度操作通过 |
| 网页全量 | 490/490；`.embeddedskills/material-retirement-web.log` |
| 网页真实表单模拟 | 新控件只发一次启动，独立解析不覆盖grab快照，停止锁定允许查询、阻止启动，重连不重放；既有环/码垛/追踪/路线/停车故障模拟通过 |
| Python串口服务 | 82/82；FakeSerial验证新查询、启动阻止与grab stop优先；`.embeddedskills/material-retirement-bridge.log` |
| Keil全量重建 | Target template，0错误/0警告；`.embeddedskills/build/template-template-rebuild.log` |

没有改动EIDE、硬件配置、香橙派源码、HMI或独立F103工程。EIDE旧MaterialVision清单按既定边界暂留，不作为当前构建入口。未连接实车、烧录或部署；软件回归不代表实车验收。

## 审查修复

独立代码审查发现普通任务区标定完成文案使用done，而固件实际发布complete；已修正并执行真实renderGrabStatus回归，先失败后通过。自查修正MCU重启后旧标定快照仍显示为当前的问题，改为历史值且不自动查询；新诊断解析兼容串口arm>提示符。这些场景均加入真实app/协议测试。采样、矩阵保留、停车与命令链路未发现其他新增问题。Keil清单121个入口均存在且无MaterialVision条目。

## 后续实车验收

在已确认的观察姿态下验证前后/X响应、回程停稳和模型对准效果；过程中取消，确认物理停车并保留旧模型。继续验证颜色/编号环对准、手动环放置、码垛调高、动态追踪夹取、车载取放与两批路线。P001、雷达配置与二维码进度问题维持原范围，不在本次修改中处理。
