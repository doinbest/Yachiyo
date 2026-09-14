# 小车指令精简与本地控制台修改计划

> **For agentic workers:** 执行时使用 `superpowers:executing-plans`，逐项实现和验证。本文件仅为计划，不代表已修改固件或网页。代码修改需遵守项目 AGENTS.md 中作者负责主要业务代码的约定。

**Goal:** 让当前小车用中文选择操作即可发送正确指令，删除失效入口和重复指令，保留视觉对准、机构标定与故障诊断能力。

**Architecture:** 固件继续使用 USART1 空格分隔 ASCII 命令；网页继续采用现有独立静态站点。用“日常操作 / 标定设置 / 诊断测试”组织命令，中文名称与线上的命令字符串分离，不重写视觉算法。固件清理、网页目录和界面、二维码选色修正分别交付。

**Tech Stack:** STM32 HAL/C、已有主机 C 回归测试、原生 HTML/CSS/JavaScript Modules、Web Serial、Node 内置测试、Python 本地静态服务器。

**Spec:** 本次用户需求、根目录 `AGENTS.md`、`docs/赛事资料/智能搬运赛题/智能搬运赛题设计与开发指导.md` 第 43–51 行、`docs/开发记录/2026-09-11工程整理与OLED三页.md`；命令实际行为以当前 `template/App/arm_console.c` 及其调用实现为准。

## 全局约束

- 修改目标为当前 `模版搭建`，不再修改历史 `stm32_orangepi_test`。
- PC 控制链路为 USART1 / Zigbee，当前配置 115200 / 8N1；串口设备由操作者选择。
- 香橙派继续使用原生 USB CDC、`orange_pi/change.py`、B2 颜色协议；PC 网页不占用这条视觉链路。
- 网页位于 `upper-computer-web/local-car/`；上层 React/Vinext 原型不是本次修改对象。
- 保持每次点击发送一条命令、CRLF、最多 63 字节和 6 个字段；不新增云服务、登录、npm 依赖、自动动作宏或循环发送。
- 保留现有二次元背景、深浅主题、本地换图、参数预览、常用按钮和日志功能。
- 不把 `accepted` 显示为到位，不把“视觉对准”显示为“完整抓取”，不把某一模块停止标成“整车急停”。
- 不修改 `.ioc`、`.uvprojx`、`.eide/eide.yml`，不改 HAL 第三方文件，不自动烧录或发送硬件命令。
- 工作区已有大量迁入文件和未提交修改；执行前检查差异，不使用 `git add .`，每个交付点只核对本任务改动。是否提交按作者当前要求执行。

## 1. 已核实的现状与设计决定

1. 当前固件已经移除 B3 的正常入口，并对旧圆环命令返回 `ERR Unsupported B3`。本次保留这个拒绝行为与测试；主要修正仍提供 B3 按钮的网页。
2. `camera material` 只发起识别和查看，不驱动机械臂；`vision material` 使用 Base/X 标定矩阵对准；`material auto` 使用 Base 回零、底盘前后移动与 X 轴修正。两套对准机构不同，不能因为名称相似就删掉其中一套。
3. `MaterialVision_CorrectionStart()` 使用固定 PID 参数，不读取 `MaterialVision_Calibration` 的响应矩阵；启动仍检查视觉参考，但不要求这份底盘响应标定。因此 `vision calib chassis` 保留为诊断测量，不再作为正常操作的前置步骤。
4. `wheel stop` 与 `chassis stop` 进入同一实现；无距离的 `chassis backward` 是固定 30 mm/s 持续后退。这两个特例可从固件删除。
5. `vision stop` 已同时停止两套视觉任务；网页常驻区保留这一按钮，`material stop` 留给单模块诊断。
6. 夹爪 open/catch/idle 当前占空比都是 7.50%。这说明实物参数尚未分开标定，不说明 open/catch 功能过时。网页去掉 idle 快捷入口，固件保留；不猜测真实舵机参数。
7. 当前固件新增 `camera status`、`imu status`、`screen status`，网页需要补齐。
8. 当前按键选色和 OLED 都读取任务码第二组；按已存赛题，第二组是位置。作为独立小步骤修正当前第一批选色；第二批流程、放置定位、Z 轴取放与完整自主比赛流程不在本轮实现范围内。

这里的精简重点是减少正常使用时需要理解的操作，而不是为了缩短 help 删除仍有价值的底层接口。

## 2. 完整命令处置表

约定：`axis` 为 base/z/x，注明 all 的接口才允许 all；`color` 为 1 红、2 黄、3 蓝、4 绿、5 黑、6 浅蓝。表中“保留”均保持线上的原有拼写。

| 现有命令 | 固件处理 | 网页层级 / 中文名称 |
| --- | --- | --- |
| `pos <axis> <degree>` | 保留，相对角度 | 日常 / 机械臂相对移动 |
| `enable <axis\|all>` | 保留 | 日常 / 机械臂使能 |
| `disable <axis\|all>` | 保留 | 标定 / 解除机械臂使能 |
| `state <axis\|all>` | 保留 | 日常 / 机械臂状态 |
| `position <axis>` | 保留 | 日常 / 读取位置（脉冲） |
| `stop <axis\|all>` | 保留 | 日常 / 机械臂停止 |
| `home <axis\|all> <mode>` | 保留 | 标定 / 执行回零 |
| `origin <axis\|all>` | 保留 | 标定 / 保存机械零点 |
| `zero <axis\|all>` | 保留 | 标定 / 当前位置计数清零 |
| `config <axis\|all>` | 保留 | 标定 / 读取运动配置 |
| `config <axis\|all> <rpm> <acc> <limit>` | 保留 | 标定 / 设置运动配置（RAM） |
| `grip open` | 保留 | 日常 / 打开夹爪 |
| `grip catch` | 保留 | 日常 / 闭合夹爪 |
| `grip idle` | 固件保留 | 删除网页目录与旧收藏；原始输入仍可手动调用 |
| `grip duty <duty>` | 保留 | 标定 / 夹爪占空比标定 |
| `chassis forward <mm>` | 保留 | 日常 / 定距前进 |
| `chassis backward <mm>` | 保留 | 日常 / 定距后退；与前进共用一个参数表单 |
| `chassis backward` | 删除无参数分支及专用速度宏 | 不提供入口；遗漏距离返回现有格式错误 |
| `chassis velocity <forward> <left> <yaw>` | 保留 | 诊断 / 底盘持续速度测试 |
| `chassis stop` | 保留 | 日常 / 底盘停止；也用于单轮测试后停止 |
| `wheel <fl\|rl\|rr\|fr> <rpm>` | 保留 | 诊断 / 单轮转向测试 |
| `wheel stop` | 删除别名 | 删除目录；已有收藏转成 `chassis stop` |
| `camera material <color>` | 保留 | 诊断 / 仅识别颜色，不运动 |
| `camera stop` | 保留 | 诊断 / 结束识别查看；不宣称停止香橙派程序 |
| `camera status` | 保留 | 诊断 / 摄像头与 USB 状态（新增网页入口） |
| `vision ref` | 保留 | 标定 / 确认视觉参考；会清除原机械臂视觉标定 |
| `vision calib material <color>` | 保留 | 标定 / Base/X 视觉标定 |
| `vision calib chassis <color>` | 保留 | 诊断 / 底盘/X 像素响应测量；不作为 PID 必需标定 |
| `vision calib <1..3>` | 保持 B3 拒绝 | 删除网页圆环标定入口 |
| `vision ring <1..3>` | 保持 B3 拒绝 | 删除网页圆环对准入口 |
| `vision material <color>` | 保留 | 日常 / 机械臂对准颜色（Base/X） |
| `vision status` | 保留 | 日常 / 机械臂视觉状态 |
| `vision stop` | 保留 | 日常 / 停止视觉任务（两套流程） |
| `material auto <color>` | 保留 | 日常 / 底盘协同对准（底盘/X）；包含 Base 回零 |
| `material status` | 保留 | 日常 / 底盘协同任务状态 |
| `material stop` | 保留 | 诊断 / 停止底盘协同任务 |
| `imu status` | 保留 | 诊断 / HWT101 航向状态（新增网页入口） |
| `screen status` | 保留 | 诊断 / 串口屏链路状态（新增网页入口） |
| `info` | 保留 | 日常 / 设备信息 |
| `help` | 保留 | 诊断 / 固件完整帮助 |

`home ... direction` 的兼容拼写不主动扩散到网页，网页沿用 `dir`；本轮不顺带删除其他解析兼容行为。底层 B3 函数和类型暂不整片删除，因为现有测试和模块仍有引用；命令不能触发它们即可。

## 3. 文件修改地图

以下路径均相对于当前工程根目录。

| 文件 | 修改责任 |
| --- | --- |
| `template/App/arm_console.c` | 删除两个特例，同步 help 和任务描述；保持既有分组函数 |
| `tests/arm_console_host_test.c` | 用实际字节输入验证旧命令不再触发动作、替代命令继续有效 |
| `tests/arm_console_test.py` | 保留 B3 拒绝、帮助分段与状态入口检查；不把文本检查当行为测试 |
| `upper-computer-web/local-car/protocol.mjs` | 当前命令目录、层级、颜色名称、真实动作说明、收藏命令解析 |
| `upper-computer-web/local-car/preferences.mjs`（新增） | 纯函数迁移旧收藏，保留背景与主题 |
| `upper-computer-web/local-car/app.mjs` | 层级筛选、参数表单、收藏迁移、查询按钮、更新项目来源文案 |
| `upper-computer-web/local-car/index.html` | 三层入口、三个常驻停止按钮、诊断原始输入、当前工程说明 |
| `upper-computer-web/local-car/styles.css` | 分层按钮和诊断折叠区样式；沿用现有主题变量 |
| `upper-computer-web/local-car/tests/protocol.test.mjs` | 删除入口、保留功能、颜色映射、层级与命令解析测试 |
| `upper-computer-web/local-car/tests/preferences.test.mjs`（新增） | 旧收藏转换、丢弃、去重、空收藏与背景保留测试 |
| `upper-computer-web/local-car/README.md` | 当前接线、分层使用、指令取舍、反馈含义和验证命令 |
| `docs/当前控制台指令.md`（新增） | 固件现行完整命令表；与历史 `#` 协议明确区分 |
| `docs/上位机协议.md` | 仅更新页首当前文档链接，保留历史正文 |
| `docs/开发记录/2026-09-11工程整理与OLED三页.md` | 更新现行指令说明，并记录选色变更 |
| `template/Core/Src/main.c` | 独立步骤：当前按键测试从 AAA 选择颜色 |
| `template/App/oled_ui.c` | 独立步骤：同一 AAA 项显示同一颜色 |
| `tests/oled_ui_test.c` | 独立步骤：使用颜色与位置明显不同的任务码回归 |

`serial.mjs`、`serve.py`、`start.cmd`、`assets/atelier.png` 复用，不计划改动。`ArmVision.c`、`MaterialVision.c`、`Camera.c`、`orange_pi/change.py` 只读核对和回归，保持现有算法及 B2 帧格式。`Arm.c` 的实物占空比另按测量结果设置，不纳入软件清理。

## 4. 任务一：精简固件命令入口

**Files:** `template/App/arm_console.c`、`tests/arm_console_host_test.c`。

**Interfaces:** 接收仍为 `ArmConsole_ReceiveData()` → `ArmConsole_Process()`；错误沿用 `ERR format`，B3 沿用 `ERR Unsupported B3`。

- [ ] 在主机测试的空闲阶段增加下面的行为断言；先运行并确认旧特例导致断言失败。

```c
before = motor_calls;
command("chassis backward\r");
assert(strstr(output, "ERR format"));
assert(motor_calls == before);

before = stop_calls;
command("wheel stop\r");
assert(strstr(output, "ERR format"));
assert(stop_calls == before);

command("chassis backward 20\r");
assert(forward_value == -20.0f);
command("chassis velocity -30 0 0\r");
assert(forward_value == -30.0f && left_value == 0.0f);
before = stop_calls;
command("chassis stop\r");
assert(stop_calls == before + 1U);
assert(strstr(output, "OK chassis stop"));
```

- [ ] 删除 `ARM_CONSOLE_BACKWARD_TEST_SPEED_MM_S` 与 TokenCount 为 2 的 backward 分支。
- [ ] 将停止分支条件收窄为以下形式，分支内既有忙检查和执行逻辑保持原样。

```c
if ((TokenCount == 2U) &&
    (strcmp(Tokens[0], "chassis") == 0) &&
    (strcmp(Tokens[1], "stop") == 0))
```

- [ ] help 删除两条旧入口；保留分段输出。将物料任务说明改为 `Base home, chassis/X align only`，将底盘标定说明标为 `response measurement`，避免暗示完整抓取。
- [ ] 运行 `python tests/run_firmware_tests.py`。旧 B3 拒绝、视觉互斥、单位换算和异步提示测试也必须通过。
- [ ] 审查只删除了命令特例，未删除 `Mecanum_Test_Stop()` 等共用函数。此任务形成独立可交付差异。

## 5. 任务二：更新网页命令目录与收藏解析

**Files:** `protocol.mjs`、`tests/protocol.test.mjs`。

**Interfaces:** 保留 `commands`、`groups`、`buildCommand(id, values)`、`frameCommand(text)`；新增 `levels`、命令的 `level` 字段，以及 `parseKnownCommand(text)`。该解析函数返回 `{id, values, wire}` 或 `null`，供收藏迁移使用。

- [ ] 增加删除入口和现行状态查询的测试，然后运行，确认当前网页测试失败。

```js
test('current catalog removes retired UI entries and exposes status', () => {
  for (const id of ['vision-calib', 'vision-ring', 'wheel-stop', 'grip-idle'])
    assert.equal(commands.some(c => c.id === id), false);
  for (const id of ['camera-status', 'imu-status', 'screen-status'])
    assert.equal(buildCommand(id, {}), id.replace('-', ' '));
  assert.equal(buildCommand('vision-material', {target:'6'}), 'vision material 6');
  assert.equal(buildCommand('material-auto', {target:'6'}), 'material auto 6');
});
```

- [ ] 删除四个网页 command ID 和不再使用的 `ring()`；保留夹爪 open/catch。更新源文件注释为当前 `template/App/arm_console.c`。
- [ ] 增加 `camera-status`、`imu-status`、`screen-status`，均无参数；按处置表更新所有名称和 note。颜色选项改为下列常量，线上编号不变。

```js
const colors = [['1','红色 · 1'], ['2','黄色 · 2'], ['3','蓝色 · 3'],
  ['4','绿色 · 4'], ['5','黑色 · 5'], ['6','浅蓝色 · 6']];
const target = () => choice('target', '目标颜色', colors);
export const levels = [['operate','日常操作'], ['setup','标定设置'],
  ['diagnostic','诊断测试']];
const setupIds = new Set(['disable','home','origin','zero','config-get',
  'config-set','grip-duty','vision-ref','vision-calib-material']);
const diagnosticIds = new Set(['velocity','wheel','camera-material','camera-stop',
  'camera-status','vision-calib-chassis','material-stop','imu-status','screen-status','help']);
const command = (id,group,label,template,fields=[],note='') => ({
  id,group,label,template,fields,note,
  level: setupIds.has(id) ? 'setup' : diagnosticIds.has(id) ? 'diagnostic' : 'operate'
});
```

这些集合放在 command 工厂和 commands 数组之前。机械臂、底盘等 group 继续表示对象；level 表示使用场景，两者不混为一个字段。

- [ ] note 至少说明：vision ref 清除标定但不执行回零；机械臂标定会运动；material auto 先回零且不含 Z/夹爪；底盘响应测量不参与当前 PID；wheel 用 chassis stop 停止；camera stop 只结束 STM32 请求/查看。
- [ ] 在 `buildCommand()` 后增加已知命令解析，用现有校验复核收藏参数。

```js
export function parseKnownCommand(text) {
  try { frameCommand(text); } catch { return null; }
  const tokens = text.trim().split(/\s+/);
  for (const c of commands) {
    const pattern = c.template.split(' ');
    if (pattern.length !== tokens.length) continue;
    const values = {};
    let matches = true;
    for (let i = 0; i < pattern.length; i++) {
      const field = /^\{(\w+)\}$/.exec(pattern[i]);
      if (field) values[field[1]] = tokens[i];
      else if (pattern[i] !== tokens[i]) { matches = false; break; }
    }
    if (!matches) continue;
    try { return {id:c.id, values, wire:buildCommand(c.id, values)}; }
    catch { /* Try the next definition, then reject. */ }
  }
  return null;
}
```

- [ ] 测试解析 `pos base 15` 成功；`chassis backward`、`vision ring 1`、`pos all 15`、`grip duty 99` 返回 null；空白规范化后仍可解析 `state  all`。
- [ ] 测试 setup/diagnostic 的 ID 对应正确层级；运行 `node --test upper-computer-web/local-car/tests/protocol.test.mjs`，保留已有边界、分帧、accepted 语义测试。修改测试标题中“old firmware”为“current USART1 console”。

## 6. 任务三：迁移已保存的常用指令

**Files:** 新增 `preferences.mjs`、`tests/preferences.test.mjs`；修改 `app.mjs` 的偏好加载。

**Interfaces:** `migratePreferences(value)` 返回 `{prefs, removed, converted}`，保留原存储键 `sky-atelier-v1`，增加内部 `schemaVersion: 2`。函数没有 DOM、串口和存储写入副作用。

- [ ] 先增加回归用例：旧收藏同时包含 B3、wheel stop、合法 pos、grip idle；应移除两项、转换一项，主题和背景不变。

```js
import test from 'node:test';
import assert from 'node:assert/strict';
import { migratePreferences } from '../preferences.mjs';
test('migrates favorites without losing appearance', () => {
  const result = migratePreferences({dark:true, background:'image-data', favorites:[
    {label:'旧圆环',wire:'vision ring 1'}, {label:'停轮',wire:'wheel stop'},
    {label:'移动',wire:'pos base 15'}, {label:'待机',wire:'grip idle'}
  ]});
  assert.equal(result.prefs.dark, true);
  assert.equal(result.prefs.background, 'image-data');
  assert.deepEqual(result.prefs.favorites.map(f => f.wire), ['chassis stop','pos base 15']);
  assert.equal(result.removed, 2);
  assert.equal(result.converted, 1);
  assert.deepEqual(migratePreferences({favorites:[]}).prefs.favorites, []);
});
```

- [ ] 最小实现如下；缺少 favorites 时由 app 使用默认收藏，用户主动清空的数组不能变回默认列表。

```js
import { commands, parseKnownCommand } from './protocol.mjs';
export function migratePreferences(value) {
  const prefs = value && typeof value === 'object' && !Array.isArray(value)
    ? {...value} : {};
  let removed = 0, converted = 0;
  if (Array.isArray(prefs.favorites)) {
    const seen = new Set();
    const favorites = [];
    for (const old of prefs.favorites) {
      const legacyStop = typeof old?.wire === 'string' && old.wire.trim() === 'wheel stop';
      const parsed = parseKnownCommand(legacyStop ? 'chassis stop' : old?.wire);
      if (!parsed || seen.has(parsed.wire) || favorites.length >= 24) { removed++; continue; }
      seen.add(parsed.wire);
      const definition = commands.find(c => c.id === parsed.id);
      favorites.push({label:definition.label, wire:parsed.wire});
      if (legacyStop) converted++;
    }
    prefs.favorites = favorites;
  } else { delete prefs.favorites; }
  prefs.schemaVersion = 2;
  return {prefs, removed, converted};
}
```

- [ ] 补测重复 chassis stop 去重、非法参数移除、损坏对象、连续迁移结果稳定、空数组保留。运行 preferences 测试。
- [ ] app 在首次渲染收藏前调用迁移，使用迁移后的 prefs；页面初始化完成后保存一次。有变化时写 SYS 日志“已清理 X 条旧常用指令，转换 Y 条停止指令”，不发送设备命令。
- [ ] 默认收藏采用设备信息、机械臂状态、打开夹爪、闭合夹爪、机械臂视觉状态、底盘协同任务状态；用户合法收藏参数照常保留。

## 7. 任务四：网页布局与交互

**Files:** `index.html`、`app.mjs`、`styles.css`。

**Interfaces:** 新增 `#levels`；使用 `command.level` 与原 `command.group` 筛选；所有按钮继续通过现有 `send()` 和 `SerialLink` 发送。

- [ ] 在模块 tabs 之前增加层级入口，默认日常。只列出当前层级有命令的模块，切层后当前模块不可用则选择第一个可用模块；只切换表单，不发送命令。

```html
<nav id="levels" class="level-tabs" aria-label="操作场景"></nav>
```

```js
let currentLevel = 'operate';
function availableGroups() {
  return groups.filter(([id]) => commands.some(c => c.level === currentLevel && c.group === id));
}
function availableCommands() {
  return commands.filter(c => c.level === currentLevel && c.group === currentGroup);
}
// 层级按钮 click 内执行：
currentLevel = button.dataset.level;
if (!availableGroups().some(([id]) => id === currentGroup))
  currentGroup = availableGroups()[0][0];
renderLevels(); renderGroups(); renderCommands();
```

`renderLevels()` 按 `levels` 创建 button，设置 dataset.level、aria-pressed 和 active；事件绑定在该函数内，button 指当前创建的按钮。`renderGroups()` 改用 availableGroups，`renderCommands()` 改用 availableCommands。当前定义保证各层级至少有一个模块。

- [ ] 常用收藏仍跨层级展示，这是作者主动保存的快捷操作；仅通过迁移剔除废弃命令，不因切页面删除合法高级收藏。
- [ ] 常驻停止区缩为三个按钮：机械臂停止 → `stop all`、底盘停止 → `chassis stop`、停止视觉任务 → `vision stop`。`material stop` 放在诊断表单；不新增“全停”命令串。
- [ ] 系统诊断模块提供新增三个状态查询。每次点击只查一次，回复显示在日志和现有最近反馈卡，不新增周期轮询或推测出的实时仪表。
- [ ] 原始输入放入“手动 ASCII 命令”折叠区，默认收起；保留现有字符/长度校验。此处允许手动诊断，不声称只允许目录命令；输入旧 B3 时由固件明确拒绝。

```html
<details class="raw-details">
  <summary>手动 ASCII 命令</summary>
  <!-- 将现有 raw-form 整体移入此处，保留原有 ID 和监听器。 -->
</details>
```

- [ ] 连接说明、页脚、启动 SYS 日志改为“模版搭建 · USART1 / Zigbee”；README 删除当前工程仍有 USB PING/INFO 测试入口的旧引导。
- [ ] 用现有主题变量增加布局，不引入组件库。保留人物位于右侧、文字区有实色底、深浅主题和换图功能。

```css
.level-tabs{display:flex;gap:8px;padding:0 25px 14px;flex-wrap:wrap}
.level-tabs button[aria-pressed="true"]{background:var(--accent);color:var(--paper)}
.raw-details summary{padding:12px 20px;color:var(--muted);cursor:pointer;font-size:13px}
@media(max-width:500px){.level-tabs{padding-inline:16px}.level-tabs button{flex:1;padding-inline:8px}}
```

- [ ] 运行 `node --check upper-computer-web/local-car/app.mjs`；在本地浏览器检查 1440×900、1024×768、390×844 的布局、浅/深主题、键盘焦点、颜色下拉、停止按钮和折叠输入。
- [ ] 浏览器加载带旧收藏的数据并刷新：旧 B3 不再出现，合法参数和背景保留；不连接硬件也可验证表单预览及所有发送按钮的断开禁用状态。
- [ ] 沿用 serial 测试验证连接不发命令、断线清理、写锁释放；不要因为 UI 修改重写已有传输层。

界面目标示意：

```text
晴空工坊                         浅/深主题  背景设置
USART1 / Zigbee   波特率   连接 / 断开
[日常操作] [标定设置] [诊断测试]
[机械臂] [夹爪] [底盘] [视觉] [系统]
操作名称 → 参数与颜色 → 实际命令预览 → 发送 / 加入常用
常用按钮                         二次元人物 / 最近设备回复
收发记录                         机械臂停止 / 底盘停止 / 视觉停止
▸ 手动 ASCII 命令
```

## 8. 任务五：独立修正当前按键测试的选色来源

**Files:** `main.c`、`oled_ui.c`、`tests/oled_ui_test.c`、开发记录。此任务可以与网页修改单独审查和回退。

**Interfaces:** 当前测试仍循环选择三个项目；明确这三个项目属于第一批 AAA。保留完整原始任务码和现有 OLED API，不增加第二批切换按钮或比赛状态机。

- [ ] 在 OLED 测试中使用 `156+123+516+231`，选择下标 1；期望完整任务码不变，颜色为 Black，不是第二组的 Yellow。

```c
OledUi_TaskCodeSet("156+123+516+231", 1);
refresh();
assert(!strcmp(rows[1], "156+123+516+231"));
assert(!strcmp(rows[2], "Item:2 Color:Black"));
```

- [ ] `Camera_ColorTaskDigitGet()` 改为读取 `QR_TaskCode[Camera_ColorDigitIndex]`；注释明确第一批 AAA。
- [ ] `OledUi_OverviewBuild()` 同步读取 `OledUi_TaskCode[OledUi_SelectedIndex]`。修订原测试中第二项颜色期望；补测下标 0/2 分别为 `Red`/`Lightblue`，与 `OledUi_ColorName()` 现有返回值一致。
- [ ] 开发记录将“第二组项目”改为“第一批 AAA 的第 1～3 项”，记录此前用位置编号选色的问题。BBB/DDD 仍保存在完整任务码中，本步骤不丢弃位置信息。
- [ ] 运行固件主机回归并编译；上板后用该示例验证 OLED、PE3 选项和 PE2 发送的 B2 目标一致，三项为 1/5/6。没有硬件实测时，只记录软件测试与编译结果。

第二批应使用 CCC，位置来源为 BBB/DDD；这些语义必须进入后续自主搬运设计，但本轮不假装已有第二批完整运行能力。也不恢复不支持的 B3 来充当位置定位。

## 9. 任务六：文档与最终验证

**Files:** 新建 `docs/当前控制台指令.md`；更新网页 README、历史协议顶部链接和当前开发记录。

- [ ] 当前指令文档收录第 2 节全部处置结果与合法参数；区分支持命令、已删除命令及 B3 拒绝，不复制历史 `sys.ping#` 清单。
- [ ] README 写明三条调试路径：
  1. 看图像：诊断 → camera material → camera status → camera stop；
  2. Base/X 对准：实物参考位置准备 → vision ref → vision calib material → vision material → vision status；
  3. 底盘/X 对准：满足参考和机构条件 → material auto → material status；不要求先执行底盘响应测量。
- [ ] 每条路径都注明动作范围；夹爪标定使用 grip duty 小步调整，open/catch 的实物值另行记录；不写入推测值。
- [ ] 网页测试从工程根目录执行以下命令，要求全部通过：

```powershell
node --test upper-computer-web/local-car/tests/protocol.test.mjs upper-computer-web/local-car/tests/preferences.test.mjs upper-computer-web/local-car/tests/serial.test.mjs
node --check upper-computer-web/local-car/app.mjs
python tests/run_firmware_tests.py
```

- [ ] 使用项目既定 Keil skill 构建 `template/MDK-ARM/template.uvprojx` 的 `template` Target；先按 skill 发现本机工具路径，不硬编码未核实的 UV4 安装目录。日志保留在 `.embeddedskills/`。编译失败先处理本次引入的第一个错误，并与修改前基线区分。
- [ ] 本地预览执行 `python upper-computer-web/local-car/serve.py --no-browser --port 8766`，打开 `http://127.0.0.1:8766/`，执行任务四的浏览器清单。若 8766 已被占用，选另一空闲端口并记录实际地址。
- [ ] 在明确硬件参数后分开验证只读查询、手动小动作、标定、两种对准及对应停止。记录实际串口、固件版本/构建产物、命令、回复和可见现象；未验证项明确记为未上板。
- [ ] 检查 `git diff --check` 和本任务文件差异；新文件也单独检查，避免因尚未跟踪而漏审。

## 10. 验收标准与执行顺序

执行顺序：固件特例清理 → 网页命令目录 → 收藏迁移 → 网页布局 → 独立选色修正 → 文档与组合验证。每个步骤先解释改动和可观察现象，再做小范围实现。

- [ ] 日常操作页面不再混入圆环、单轮、PWM、保存零点、响应测量等低频项。
- [ ] B3 快捷入口与旧收藏消失；无距离后退、wheel stop 在新固件上不产生动作。
- [ ] 两种对准方式均可选择，名称准确；三种新增状态查询均可发送。
- [ ] home/origin/zero、两种标定用途、停止范围有清楚说明。
- [ ] 刷新不恢复废弃收藏；背景、主题和合法收藏参数保留；连接与页面切换不触发动作。
- [ ] 现有二次元主题和本地启动流程可用；浏览器截图与布局检查结果写入实施记录。
- [ ] 第一批颜色读取与显示一致，不再使用 BBB 位置值作为颜色。
- [ ] 自动化检查、Keil 编译、浏览器验证、硬件验证分别记录，不能用其中一项代替另一项。

本计划不把“指令精简”扩大成重写视觉算法、改二进制控制协议或实现整套自主比赛。完成后，作者日常主要做“选模块、选动作、填参数、发送”，底层可诊断能力仍然保留。
