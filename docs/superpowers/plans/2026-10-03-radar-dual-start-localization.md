# 雷达双起点识别与 F4 地图参考接入 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. 本轮仅交付计划，后续实施按任务顺序执行。

**Goal:** 在 F407 静止扫描后识别右上/右下起点，建立正确地图参考，使雷达规划路线从所选起点出发并返回。

**Architecture:** 完整局部点云先提取物料台、二维码装置轮廓，分别匹配两种起点布局。识别成功后复用现有定位原点和同一圈点云建图；规划及执行接收所选起点和航向。网页与 F4 共用纯 C 识别、建图和规划核心。

**Tech Stack:** STM32F407 HAL、LDS50C/USART2循环DMA、C99、Keil Target `template`；本地原生网页、Node测试、Python桥接、GCC主机测试。

**Spec:** [需求整理与设计依据](../specs/2026-10-03-radar-dual-start-design.md)。执行前同时阅读该文档。

## Global Constraints

- 保留官方 Emm_V5、W25Q128 驱动及诊断；不改 IOC、引脚、时钟、EIDE 配置，不运行 EIDE。
- 参数仍仅作用于 STM32 RAM；不增加 Flash 保存、启动加载或浏览器自动写参数。
- 短暂坏帧、单位变化和 DMA 覆盖只重采当前未完整圈，沿用10秒扫描窗口；不增加固定恢复次数耗尽规则。
- 停止及时取消剩余采集/识别，不清已有效参数、其他业务参考或旧完整地图。结果待判定不触发电机搜索运动。
- 不烧录、不复位、不向实车发指令。软件验证与后续双起点实物验收分别记录。

## Review Focus

- 扫描高度没有穿过圆盘或二维码板：使用该高度的真实支撑/底座轮廓，不强制按300 mm圆和A4线段分类。任务1/3覆盖。
- 物料台、二维码板随机移动：允许区域内仍能区分两起点，不能绑定固定物体中心。任务3覆盖。
- 二维码板从起点看近似侧向或被遮挡：弱证据不否决可靠物料台证据；两假设无法区分时保持待判定。任务3覆盖。
- 注册原点时暂缺航向反馈：保留已识别结果及旧有效参考，不发布错误新图。任务4覆盖。
- 上方起点识别正确但执行仍检查下方起点/90°：规划首末点、执行参考和航向一起接入；原固定路线保持行为。任务5覆盖。

---

## 文件职责与接口决定

| 文件 | 职责 |
|---|---|
| 新增 `Code/template/App/radar_start.c/.h` | 纯 C 点云特征提取及两起点分类；不包含 HAL、串口或运动调用 |
| 新增 `Code/template/App/radar_start_task.c/.h` | 非阻塞采集、识别、注册参考和重投影协调；不实现另一套定位器 |
| 修改 `App/radar_scan.c/.h` | 增加仅采集点云和成功定位后的重投影；保留普通扫描 |
| 修改 `App/radar_map.c/.h` | 参数化雷达规划起终点，保留旧默认入口 |
| 修改 `App/radar_console.c/.h`、`Core/Src/main.c` | 命令、分页及主循环接入；更新现有扫描占用判断 |
| 修改 `App/chassis_route.c/.h`、`App/chassis_motion.c`、`App/GrabRoute.c/.h` | 计划携带起点姿态、执行不再写死下方起点；注册失败保留锚点；既有首件交接兼容 |
| 修改 `Code/tools/radar_plan_cli.c`、`serve.py` | 主机回放复用同一 C 核心 |
| 修改网页 `radar-model.mjs`、`radar-view.mjs`、`radar-format.mjs`、`app.mjs`、`parameter-schema.mjs`、`parameter-page.mjs`、`parameter-console.mjs`、`index.html`、`styles.css` | 识别入口、结果图层、参数读回和界面接线；分页类 `RadarExchange` 位于 `radar-model.mjs` |
| 修改 Keil `MDK-ARM/template.uvprojx` | 仅加入新源文件 |

上述 `App/`、`Core/` 路径均相对 `Code/template/`；网页路径相对 `Code/upper-computer-web/local-car/`。沿用现有职责，不进行无关拆分。

识别类型定义在 `radar_start.h`，其中共享姿态类型定义在已有 `radar_map.h` 并由识别模块复用，避免循环include：

- `RadarStart_Id_t`：`RADAR_START_UNKNOWN=0`、`RADAR_START_UPPER=1`、`RADAR_START_LOWER=2`，不与工位编号混用。
- `RadarStart_Pose_t`：在 `radar_map.h` 定义 `float x_mm, y_mm, yaw_deg`，航向为地图+X逆时针。
- `RadarStart_Config_t`：车体安装三参数、两起点名义姿态、实测物料台/二维码装置可见轮廓和允许区域、经回放确定的匹配条件。
- `RadarStart_Result_t`：分析状态、起点ID、名义车辆姿态、推导雷达姿态、目标特征及两个假设得分、结果原因。姿态来源标为 `nominal_start`。
- `RadarStartTask_Status_t`：任务状态/原因、当前采集ID和原点代次、识别结果；区分识别完成、参考已建立、地图已发布。

采用有界分组和累计拟合，分批处理点云；不复制一份4096点浮点XY数组，不引入 SLAM、动态分配或通用任务框架。运行占用只维持到识别/重投影结束，期间停车和既有通信处理继续运行。

## 任务0：实施前基线

截至本计划形成时，HEAD 为 `d60658c`，工作区干净。这是分析时状态，实施时重新检查，不将它当作未来必然基线。

- [ ] 检查 `git status`、暂存区与新增文件；保存用户已授权且尚未提交的源码/文档，不强制加入忽略产物。若有未提交工作，单独提交 `chore: checkpoint before dual-start localization`；否则记录现有HEAD。
- [ ] 记录 Emm、Flash及诊断、IOC、EIDE文件 SHA256，保存至 `.embeddedskills/radar-dual-start/`。
- [ ] 运行网页、桥接、固件和雷达线协议回归，Keil构建 `template`；保存日志及 Program Size，区分原有失败。

验收：得到可追溯基线哈希、保护文件清单和原有回归结果，后续提交仅含本功能。

## 任务1：建立双起点点云样本与可见轮廓基线

**Files:** 新增 `Code/tests/fixtures/radar_start/README.md`、人工标注的双起点回放数据；新增 `Code/tests/radar_start_fixture_test.py`。实测高度、安装关系和轮廓记入同目录说明。

**Interfaces:** 回放样本使用现有 `RadarSample_t` 三元组 `angle_tenths/distance_mm/energy`。每份样本携带已知起点、雷达安装、摆放姿态、物料台/二维码位置、实际扫描高度、数据来源和是否合成，不把合成样本当实测。

- [ ] 按场地图先生成两起点、物料台及二维码允许位置变化、50 mm障碍柱的合成基线；断言样本角度单位为0.1°、距离为mm，真值坐标来自几何生成而非待测识别器。
- [ ] 将用户后续提供的实测完整圈原样纳入回放，记录该高度下实际扫到的轮廓；需要采集实车数据时单独按已确认硬件操作授权进行，不猜COM口。
- [ ] 以实测回放确定可见轮廓、测距误差容许范围和匹配差值。样本缺少实测时软件可以开发，识别率与实车可用性保持未验收，不代填高度/底座尺寸。
- [ ] 运行 `python Code/tests/radar_start_fixture_test.py`，要求样本结构/单位/来源验证通过；提交 `test: add dual-start radar replay fixtures`。

## 任务2：完整局部点云入口与同圈重投影

**Files:** 修改 `radar_scan.c/.h`；修改 `Code/tests/test_radar_scan.c` 和 `run_firmware_tests.py`；同步 Keil清单只在新增C文件接入时处理。

**Interfaces:**

- 保留 `bool RadarScan_Start(const RadarMap_Params_t *, uint32_t origin_generation)`。
- 新增 `bool RadarScan_StartLocal(void)`：只受理静止局部采集，不要求世界原点，不发布新地图。
- 保留 `RadarScan_Points(uint16_t *)`：只返回完整圈。
- 新增 `bool RadarScan_Reproject(const RadarMap_Params_t *, uint32_t origin_generation, uint32_t scan_id)`：确认当前完整圈ID，重投影并发布与新参考关联的地图；不重新开雷达或发送电机指令。

- [ ] 增加失败测试：局部采集完成后点云可读，地图ID及旧图不变；区外点仍保留；指定旧scan_id重投影被拒绝；重投影结果与逐点 `RadarMap_Add` 同值。
- [ ] 运行固件回归确认新断言在实现前失败，原有普通扫描场景仍通过。
- [ ] 实现局部采集模式、完整圈元数据与重投影。重投影用已有working_map，只有整图完成后替换published_map；沿用单点云缓存和10秒坏帧恢复。
- [ ] 回归包括 mm/cm变化、角度接缝、坏帧、DMA覆盖、超时、取消和下一圈重新采集；要求未完成圈不外露、旧完整图不被错误替换。
- [ ] 提交 `feat: capture local radar clouds before map registration`。

## 任务3：纯 C 双起点识别

**Files:** 新增 `radar_start.c/.h`、`Code/tests/test_radar_start.c`；在 `radar_map.h` 增加共享姿态类型，更新主机测试入口和Keil文件清单。

**Interfaces:**

```c
void RadarStart_Init(void);
bool RadarStart_Begin(const RadarSample_t *points, uint16_t count,
                     const RadarStart_Config_t *config);
bool RadarStart_Process(uint16_t point_budget); /* true表示本次分析已结束 */
void RadarStart_ResultGet(RadarStart_Result_t *result);
void RadarStart_Cancel(void);
```

`Begin`期间冻结配置，点云在分析结束前不能被新扫描覆盖；`Process(64)`分批推进，离线工具使用同一接口循环到结束。状态区分 analyzing、identified、ambiguous、insufficient、cancelled；坐标以 `RadarMap_Project` 同一角度定义转换。

- [ ] 先用任务1真值样本写失败测试：上方输出UPPER、下方输出LOWER；随机允许位置不改变ID；不得把小障碍柱判为物料台；跨零度分组不拆开同一目标。
- [ ] 补充测试：部分圆弧不使用均值充当圆心；10 mm量化与约30 mm扰动；板侧向、底座轮廓及遮挡；一个目标暂缺但剩余证据仍充分；两假设等价时输出ambiguous，不输出默认LOWER。
- [ ] 实现局部分组、累计拟合、形状候选与两假设得分。以任务1标注确定轮廓和容许范围，不使用未经依据的紧门槛；筛选考虑随机摆放区与场地外侧目标。
- [ ] 对完整4096点上限样本记录处理步数与工作区大小，确认没有动态分配、整圈XY副本或点云二次常驻缓存；所有预算调用最终结束。
- [ ] 运行固件回归，原parser/map/scan场景保持通过；提交 `feat: classify radar scans against both start layouts`。

## 任务4：起点参考、地图与命令接入

**Files:** 新增 `radar_start_task.c/.h`；修改 `radar_console.c/.h`、`chassis_motion.c`、`Core/Src/main.c` 和Keil清单；新增 `Code/tests/radar_start_task_test.c`，扩展 `radar_console_test.c`、`radar_wire_contract_test.py`、航向锚点相关回归。

**Interfaces:**

```c
void RadarStartTask_Init(void);
bool RadarStartTask_Start(void);
void RadarStartTask_Process(void);
void RadarStartTask_Stop(void);
bool RadarStartTask_IsBusy(void);
void RadarStartTask_StatusGet(RadarStartTask_Status_t *status);
```

控制台增加 `radar start detect`、`radar start status`、`radar start get`、`radar start set <key> <value>`；`radar stop`覆盖采集与识别。新安装键为 `lidar_forward_mm/lidar_left_mm/lidar_yaw_offset_deg`，两起点姿态键为 `start1_x_mm/start1_y_mm/start1_yaw_deg` 和 `start2_x_mm/start2_y_mm/start2_yaw_deg`。原 `radar scan/get/set/fetch/nav/task`保留。新 `@RADAR` start/landmark参数数据遵守现有分页、快照标识和单条256字节上限；新增 fetch kind 与参数页总数由同一清单生成，不继续写死10页。

- [ ] 失败测试覆盖：无地图原点时也能识别；成功后安装偏移在90°和270°下正确旋转；一次请求从接收到地图发布不会提交底盘/机械臂/ID8运动。
- [ ] 将任务2/3接入非阻塞主循环：局部采集→分批分析→充分证据后注册参考→重投影同圈。当前窗口内可重采完整圈，按总10秒计时，不能每圈重置总期限；只累计小型特征证据。
- [ ] 更新运动入口已有扫描占用为“采集或识别处理中”，覆盖底盘、抓取与ID8分度；停车仍能立即进入原停止流程。停止不调用其他业务清参数/清参考接口。
- [ ] 调整 `ChassisMotion_AnchorSet` 顺序：先取得合格航向，再替换锚点；失败测试确认原有效锚点保持。复用现有IMU资格，不触发归零/重复静止验证。
- [ ] 测试暂缺航向时保留识别结果，反馈恢复后继续注册；原有验证失败则说明原因；成功注册等待正常新反馈，不把 `feedback_valid=false` 的瞬时过渡报告为识别失败。
- [ ] 测试失败/取消保留旧图，新图/参考代次一致、上一轮点云不能作为本轮完成；识别任务主动注册原点的代次更新不会触发现有“外部原点改变”取消分支。暂未识别显示原因，不擅自选择起点或发动搜索。
- [ ] 运行固件、线协议回归并Keil构建，提交 `feat: register the detected start before publishing radar maps`。

## 任务5：双起点规划与执行

**Files:** 修改 `radar_map.c/.h`、`radar_console.c`、`chassis_route.c/.h`、`GrabRoute.c/.h`；扩展 `test_radar_map.c`、`chassis_route_test.c`、`grab_route_test.c`、`radar_start_task_test.c`。

**Interfaces:**

- 新增 `bool RadarPlan_BuildFromStart(const RadarMap_t *, const RadarStart_Pose_t *, RadarPlan_t *)`；原 `RadarPlan_Build`保留下方默认行为。姿态类型使用任务3在 `radar_map.h` 中定义的类型，不让地图核心反向依赖识别器。
- `RadarPlan_t`增加 `RadarStart_Pose_t start_pose`；`RadarPlan_BuildLeg`填写输入首点，航向保持接口原有默认语义且不作为任务执行计划。
- `ChassisRoute_PlanStart(const RadarPlan_t *, float speed, bool station_mode)`签名保留，读取plan.start_pose；原固定路线入口不变。

- [ ] 先写失败测试：UPPER计划首末为 `(2250,2250)`、LOWER为 `(2250,150)`；访问工位顺序仍 `起点→扫码→原料→粗工→暂存→原料→粗工→暂存→起点`，绕行不改变station/visit。
- [ ] 扩展计划和执行状态携带参考姿态；计划路线校验首点、所选原点/代次和对应到点反馈，不再硬编码下方起点。保持已有必要反馈与实际软件边界条件。
- [ ] 执行雷达计划时目标yaw采用所选出发航向；上下两起点分别保持标定后的向二维码台航向。原16段仍使用90°；路径折点不自动变为车头转向。
- [ ] 扩展组合路线状态传递，保留扫码visit1、首件原料visit2的现有交接范围；不自动执行剩余两批物料调度。
- [ ] 测试上方路线可受理、末点返上方，原固定路线仍拒绝错误起点；状态显示目标yaw一致；首件交接由station/visit触发；运行期间参考改变按现有实际失效原因退出。
- [ ] 运行固件和线协议回归、Keil构建，提交 `feat: execute radar plans from either start pose`。

## 任务6：网页预览、参数与共同算法回放

**Files:** 修改文件职责表所列网页模块、`serve.py`、`Code/tools/radar_plan_cli.c`；扩展 `tests/radar.test.mjs`、`radar-view.test.mjs`、`test_radar_service.py`及参数页回归。

**Interfaces:** CLI保留 `RADAR1`输入；增加 `RADAR2`局部点云+识别配置输入，输出start/landmarks及同一C地图/规划结果。网页只渲染和发起显式请求。服务新增 `/api/radar/start`仅作离线回放；串口在线识别由 `radar start detect`完成。

- [ ] 先写测试：新start/landmark分页正确关联同一快照；当前/历史结果可区分；离线CLI输出与主机C测试一致；读取/导航/切页没有TX。
- [ ] 在地图雷达区域增加“扫描并识别起点”及结果条；优先下载少量结果与特征页，不等待完整点云。画出实测轮廓、参与匹配的物料台/二维码装置与车辆名义参考，不把物体中心替代车辆停靠点。
- [ ] 参数调试“雷达与路线”增加车体安装和两个出发姿态，保持蓝/黑背景、展开卡片和一键读回；计算出的雷达世界坐标只读，与普通扫描世界坐标配置分别说明。
- [ ] 既有参数批次支持新参数分页、RAM写入和主动读回比较；unset留空、拒绝显示原因，不能用网页预填冒充设备值。
- [ ] 离线回放通过Python编译/调用相同radar_start/radar_map C核心；地图视角、返回导航、参数草稿沿用已有机制。
- [ ] 桌面、笔记本和手机实际预览，检查结果/按钮/图层与参数对齐；运行网页、桥接、雷达服务测试，提交 `feat: preview dual-start radar detection in the console`。

## 任务7：文档、完整回归与交付

**Files:** 修改 `README.md`、`AGENTS.md`、`docs/上位机协议.md`、控制台README；新增 `docs/开发记录/2026-10-03-雷达双起点识别实现与验收.md`，若实际完成日期改变则使用实际日期。历史记录保留当时事实。

- [ ] 记录自动识别/人工参考/名义起点的区别，命令、参数、状态、地图来源、双起点路线范围及停止恢复行为。
- [ ] 运行以下回归，任何失败先定位首个失败步骤并区分基线问题：

```powershell
node --test Code/upper-computer-web/local-car/tests/*.test.mjs
python -m unittest discover -s Code/upper-computer-web/local-car/tests -p test_bridge.py
python -m unittest discover -s Code/upper-computer-web/local-car/tests -p test_radar_service.py
python Code/tests/run_firmware_tests.py
python Code/tests/radar_wire_contract_test.py
```

- [ ] Keil构建Target `template`，无新增错误/警告；对比真实ROM/RAM结果，确认4096点云无重复常驻缓冲。
- [ ] 检查文档链接及最终diff，核对Emm/Flash/诊断/IOC/EIDE哈希与基线一致，保存日志、各批提交及网页预览。
- [ ] 提交 `docs: document dual-start detection and field acceptance`。

## 后续实物验收与回退

实物验收依次为：测量扫描平面及安装关系；两起点分别静止采集；允许摆放范围内改变物料台/二维码位置；加入障碍和部分遮挡；验证识别、同圈地图和目标图层；显式低速走到二维码位置；验证上方起点的扫码观察姿态及首件交接；最后验证返回各自起点。

每次分别记录起点分类正确性、观察到的轮廓、名义摆放误差、地图投影误差、路线实际通行和返回结果。合成点云通过、主机测试通过和Keil构建通过均不等于实车验收。

若需要回退，先保存后来工作，再 `git revert <对应批次提交>`。可先回退网页、再回退执行与参考接入，保留早期纯C分析模块及日志用于排查；不使用 `reset --hard`。
