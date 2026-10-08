import {
  createGrid,
  filterRadarPoints,
  loadRadarDirectPreferences,
  normalizeRadarConfig,
  saveRadarDirectPreferences,
  scanToCsv,
  toFieldPoint,
} from './radar-direct-model.mjs';
import {
  formatMissionRoute,
  parseMissionRoute,
  resolveMissionPoints,
  resolvePlanningEndpoint,
} from './radar-direct-planner.mjs';
import {mountRadarDirectMap} from './radar-direct-map.mjs';

const directionText = value => ['向北', '向东', '向南', '向西'][value] || '—';
const turnText = value => ['起步', '直行', '左转', '右转', '掉头'][value] || '到达';
const clone = value => structuredClone(value);

/** Mount the F407-owned radar host page. All hardware writes are button driven. */
export function mountRadarDirectHost({
  root,
  storage = globalThis.localStorage,
  hostClient,
  connectionController,
  mapFactory = options => mountRadarDirectMap(options),
} = {}) {
  if (!root || !hostClient?.model || typeof hostClient.receive !== 'function' ||
      typeof connectionController?.listPorts !== 'function') {
    throw new TypeError('F407 radar host needs a root, hostClient and shared connectionController');
  }
  const document = root.ownerDocument || globalThis.document;
  const refs = {};
  const el = (tag, {id, cls, type, value, text, title} = {}) => {
    const node = document.createElement(tag);
    if (id) { node.id = id; refs[id] = node; }
    if (cls) node.className = cls;
    if (type) node.type = type;
    if (value !== undefined) node.value = String(value);
    if (text !== undefined) node.textContent = text;
    if (title) node.title = title;
    return node;
  };
  const button = (id, text, cls = 'button button-ghost') => el('button', {id, text, type: 'button', cls});
  const field = (parent, id, text, value, options = {}) => {
    const label = el('label', {cls: options.cls || 'radar-direct-field'});
    label.append(el('span', {text}));
    let input;
    if (options.values) {
      input = el('select', {id});
      for (const [itemValue, itemText] of options.values) input.append(el('option', {value: itemValue, text: itemText}));
    } else {
      input = el('input', {id, type: options.type || 'number', value});
      for (const name of ['min', 'max', 'step']) if (options[name] !== undefined) input.setAttribute(name, options[name]);
    }
    input.value = String(value);
    label.append(input); parent.append(label); return input;
  };
  const panelHeading = (kicker, title) => {
    const heading = el('div', {cls: 'radar-direct-heading'});
    const copy = el('div'); copy.append(el('span', {cls: 'radar-direct-kicker', text: kicker}), el('h2', {text: title}));
    heading.append(copy); return heading;
  };

  const loaded = loadRadarDirectPreferences(storage);
  let config = loaded.config;
  let connectionState = {
    state: connectionController.state || 'disconnected',
    port: connectionController.port || '',
    baudrate: 115200,
  };
  let derived = null;
  let busy = false;
  let disposed = false;
  const logs = [];

  root.className = 'radar-direct-host';
  const hero = el('header', {cls: 'radar-direct-hero'});
  const heroCopy = el('div'); heroCopy.append(el('span', {cls: 'radar-direct-kicker', text: 'F407 · LDS-50C · ONE-SCAN MAPPING'}), el('h1', {text: '雷达建图与路径规划'}), el('p', {text: 'F407 接收完整扫描圈并按 F1 规则筛选、分区和规划 5 × 5 场地路径。'}));
  const heroStatus = el('div', {cls: 'radar-direct-hero-status'}); heroStatus.append(el('span', {cls: 'radar-direct-status-dot'}), el('strong', {id: 'radar-direct-freeze', text: '等待扫描'}), el('small', {id: 'radar-direct-point-summary', text: '0 个显示点 / 0 个接收点'}));
  hero.append(heroCopy, heroStatus);

  const connection = el('section', {cls: 'radar-direct-connection glass'}); connection.append(panelHeading('SERIAL WORKFLOW', '设备与扫描'));
  const connectionFields = el('div', {cls: 'radar-direct-connection-fields'});
  const port = field(connectionFields, 'radar-direct-port', '串口', config.radarPort, {values: [['', '未选择串口']]});
  const baud = field(connectionFields, 'radar-direct-baud', '波特率', 115200, {values: [[115200, 115200]]});
  baud.disabled = true;
  const refreshPortsButton = button('radar-direct-refresh-ports', '↻', 'button button-ghost radar-direct-icon-button');
  connectionFields.append(refreshPortsButton); connection.append(connectionFields);
  const actions = el('div', {cls: 'radar-direct-actions'});
  for (const item of [
    ['radar-direct-connect', '连接', 'button button-primary'],
    ['radar-direct-disconnect', '断开'],
    ['radar-direct-start', '启动旋转'],
    ['radar-direct-stop', '停止'],
    ['radar-direct-single', '单次扫描', 'button button-accent'],
    ['radar-direct-continuous', '持续扫描', 'button button-accent'],
    ['radar-direct-read-status', '读取状态/结果'],
    ['radar-direct-save', '保存本机配置'],
  ]) actions.append(button(...item));
  connection.append(actions, el('p', {id: 'radar-direct-connection-status', cls: 'radar-direct-feedback', text: '设备未连接'}));

  const workspace = el('section', {cls: 'radar-direct-grid'});
  const controls = el('aside', {cls: 'radar-direct-controls'});
  const filterPanel = el('section', {cls: 'radar-direct-panel glass'}); filterPanel.append(panelHeading('POINT FILTER', '点云筛选'));
  const displayButtons = el('div', {cls: 'radar-direct-segments'}); displayButtons.append(button('radar-direct-show-raw', '接收原始'), button('radar-direct-show-filtered', '上位机筛选'));
  const themeButtons = el('div', {cls: 'radar-direct-segments'}); themeButtons.append(button('radar-direct-theme-dark', '黑'), button('radar-direct-theme-light', '白'));
  filterPanel.append(el('p', {cls: 'radar-direct-subtitle', text: '单次/持续扫描都由 F407 用完整圈判障和规划；120 点仅作预览，完整点云按需诊断。'}), displayButtons, themeButtons);
  const filterFields = el('div', {cls: 'radar-direct-two-fields'});
  field(filterFields, 'radar-direct-min-angle', '最小角度 °', config.filter.minAngleDeg, {min: 0, max: 360, step: 0.1});
  field(filterFields, 'radar-direct-max-angle', '最大角度 °', config.filter.maxAngleDeg, {min: 0, max: 360, step: 0.1});
  field(filterFields, 'radar-direct-min-distance', '最小距离 mm', config.filter.minDistanceMm, {min: 0, max: 65535});
  field(filterFields, 'radar-direct-max-distance', '最大距离 mm', config.filter.maxDistanceMm, {min: 0, max: 65535});
  field(filterFields, 'radar-direct-min-energy', '最小能量', config.filter.minEnergy, {min: 0, max: 255});
  field(filterFields, 'radar-direct-max-energy', '最大能量', config.filter.maxEnergy, {min: 0, max: 255});
  filterPanel.append(filterFields, button('radar-direct-apply-filter', '应用筛选（不重新扫描）', 'button button-primary radar-direct-full'));

  const mapSettings = el('section', {cls: 'radar-direct-panel glass'}); mapSettings.append(panelHeading('FIELD CALIBRATION', '场地与分区'));
  const radarFields = el('div', {cls: 'radar-direct-two-fields'});
  field(radarFields, 'radar-direct-radar-x', '雷达 X mm', config.map.radarXmm, {min: 0, max: 2400});
  field(radarFields, 'radar-direct-radar-y', '雷达 Y mm', config.map.radarYmm, {min: 0, max: 2400}); mapSettings.append(radarFields);
  for (const axis of ['x', 'y']) {
    mapSettings.append(el('p', {cls: 'radar-direct-subtitle', text: `内部${axis === 'x' ? '竖线 X' : '横线 Y'} · 也可在地图拖动`}));
    const fields = el('div', {cls: 'radar-direct-boundaries'});
    for (let index = 0; index < 4; index += 1) field(fields, `radar-direct-${axis}${index + 1}`, `${axis.toUpperCase()}${index + 1}`, config.map[`${axis}Lines`][index + 1]);
    mapSettings.append(fields);
  }
  const planningMode = field(mapSettings, 'radar-direct-planning-mode', '规划方式', config.planningMode, {values: [['mission', '编号任务路线'], ['point-to-point', '普通起点到终点']]});
  const missionRoute = field(mapSettings, 'radar-direct-mission-route', '任务顺序（编号 1～5）', formatMissionRoute(config.missionRoute), {type: 'text'});
  const endpointValues = [['radar', '启停区 2（雷达中心）'], ['goal-zone-one', '启停区 1（右下边界）']];
  for (let row = 1; row <= 5; row += 1) for (let column = 1; column <= 5; column += 1) endpointValues.push([`cell-${row}-${column}`, `R${row} · C${column}`]);
  const endpoints = el('div', {id: 'radar-direct-endpoints', cls: 'radar-direct-two-fields'});
  field(endpoints, 'radar-direct-start-cell', '起点格', 'radar', {values: endpointValues});
  field(endpoints, 'radar-direct-goal-cell', '终点格', 'goal-zone-one', {values: endpointValues}); mapSettings.append(endpoints);
  field(mapSettings, 'radar-direct-threshold', '判为扫描障碍的最少点数', config.map.obstaclePointThreshold, {min: 1, max: 4096});
  mapSettings.append(button('radar-direct-apply-map', '应用场地与规划参数', 'button button-primary radar-direct-full'), el('p', {id: 'radar-direct-settings-status', cls: 'radar-direct-feedback'}));
  controls.append(filterPanel, mapSettings);

  const mapStage = el('section', {cls: 'radar-direct-map-stage glass'}); mapStage.append(panelHeading('PHYSICAL FIELD', '2400 × 2400 mm 场地'));
  const legend = el('div', {cls: 'radar-direct-legend', text: '固定禁区　手动禁区　扫描障碍　规划路径　任务点'});
  const canvas = el('canvas', {id: 'radar-direct-map'}); canvas.setAttribute('aria-label', '四象限雷达点云与2400毫米场地地图');
  mapStage.append(legend, canvas, el('p', {id: 'radar-direct-cursor', cls: 'radar-direct-footnote', text: '拖动四条内部横线或竖线；单击普通格切换手动禁区。'}));

  const diagnostics = el('aside', {cls: 'radar-direct-diagnostics glass'}); diagnostics.append(panelHeading('LIVE DIAGNOSTICS', '数据诊断'));
  const metrics = el('div', {cls: 'radar-direct-metrics'});
  for (const [id, label] of [['raw', '原始点'], ['filtered', '筛选点'], ['blocked', '障碍格'], ['distance', '路径 mm']]) { const item = el('article'); item.append(el('strong', {id: `radar-direct-metric-${id}`, text: '0'}), el('span', {text: label})); metrics.append(item); }
  diagnostics.append(metrics);
  const counters = el('dl', {id: 'radar-direct-counters', cls: 'radar-direct-list'}); diagnostics.append(el('h3', {text: '帧解析 · 累计计数'}), counters);
  const flags = el('dl', {id: 'radar-direct-flags', cls: 'radar-direct-list'}); diagnostics.append(el('h3', {text: '雷达实际状态'}), flags);
  diagnostics.append(el('section', {id: 'radar-direct-route-summary', cls: 'radar-direct-route-summary'}));
  const tableHead = labels => { const head = el('thead'), row = el('tr'); for (const label of labels) row.append(el('th', {text: label})); head.append(row); return head; };
  const waypointTable = el('table', {cls: 'radar-direct-table'}); const waypointBody = el('tbody', {id: 'radar-direct-waypoints'}); waypointTable.append(tableHead(['序号', '方格', 'X mm', 'Y mm', '下一段', '方向', '动作']), waypointBody); diagnostics.append(el('h3', {text: '完整途径点'}), waypointTable);
  const previewTable = el('table', {cls: 'radar-direct-table'}); const previewBody = el('tbody', {id: 'radar-direct-preview'}); previewTable.append(tableHead(['角度', '距离', '能量']), previewBody); diagnostics.append(el('h3', {id: 'radar-direct-preview-title', text: '上位机筛选点预览 · 前 12 条'}), previewTable);
  diagnostics.append(button('radar-direct-csv', '导出当前原始圈 CSV', 'button button-primary radar-direct-full'), el('div', {id: 'radar-direct-logs', cls: 'radar-direct-logs'}));
  workspace.append(controls, mapStage, diagnostics); root.replaceChildren(hero, connection, workspace);

  const endpointFromValue = (value, fallback) => {
    if (value === 'radar') return {row: 1, column: 1, anchor: 'radar'};
    if (value === 'goal-zone-one') return {row: 1, column: 5, anchor: 'goal-zone-one'};
    const match = /^cell-([1-5])-([1-5])$/.exec(value);
    return match ? {row: Number(match[1]), column: Number(match[2]), anchor: 'cell'} : fallback;
  };
  const endpointValue = endpoint => endpoint.anchor === 'radar' ? 'radar' : endpoint.anchor === 'goal-zone-one' ? 'goal-zone-one' : `cell-${endpoint.row}-${endpoint.column}`;
  refs['radar-direct-start-cell'].value = endpointValue(config.start); refs['radar-direct-goal-cell'].value = endpointValue(config.goal);

  function log(text) {
    logs.push(`${new Date().toLocaleTimeString('zh-CN', {hour12: false})}　${text}`);
    if (logs.length > 30) logs.shift();
    refs['radar-direct-logs'].replaceChildren(...logs.slice(-6).reverse().map(line => el('p', {text: line})));
  }

  function setLocalConfig(next) {
    config = normalizeRadarConfig(next);
    syncInputs();
    render();
  }

  function syncInputs() {
    const values = {
      'radar-direct-min-angle': config.filter.minAngleDeg, 'radar-direct-max-angle': config.filter.maxAngleDeg,
      'radar-direct-min-distance': config.filter.minDistanceMm, 'radar-direct-max-distance': config.filter.maxDistanceMm,
      'radar-direct-min-energy': config.filter.minEnergy, 'radar-direct-max-energy': config.filter.maxEnergy,
      'radar-direct-radar-x': config.map.radarXmm, 'radar-direct-radar-y': config.map.radarYmm,
      'radar-direct-threshold': config.map.obstaclePointThreshold, 'radar-direct-planning-mode': config.planningMode,
      'radar-direct-mission-route': formatMissionRoute(config.missionRoute), 'radar-direct-start-cell': endpointValue(config.start),
      'radar-direct-goal-cell': endpointValue(config.goal), 'radar-direct-baud': 115200,
    };
    for (const axis of ['x', 'y']) for (let index = 1; index <= 4; index += 1) values[`radar-direct-${axis}${index}`] = config.map[`${axis}Lines`][index];
    for (const [id, value] of Object.entries(values)) refs[id].value = String(value);
    if (config.radarPort) refs['radar-direct-port'].value = config.radarPort;
    refs['radar-direct-endpoints'].hidden = config.planningMode !== 'point-to-point';
    missionRoute.parentElement.hidden = config.planningMode !== 'mission';
  }

  const anchorName = value => value === 1 || value === 'radar' ? 'radar' :
    value === 2 || value === 'goal' || value === 'goal-zone-one' ? 'goal-zone-one' : 'cell';
  const deviceEndpoint = endpoint => ({
    row: endpoint.row,
    column: endpoint.column,
    anchor: endpoint.anchor === 'goal-zone-one' ? 'goal' : endpoint.anchor,
  });
  const sameIdentity = (left, right) => !!left && !!right &&
    ['session', 'config', 'scan', 'map', 'plan'].every(key => left[key] === right[key]);
  let activeConfigId = null;
  function syncActiveConfig() {
    const device = hostClient.model.snapshots.config;
    if (!device || device.source !== 'active' || device.config === activeConfigId) return;
    activeConfigId = device.config;
    config = normalizeRadarConfig({
      filter: {
        minAngleDeg: device.angle_min / 10,
        maxAngleDeg: device.angle_max / 10,
        minDistanceMm: device.distance_min,
        maxDistanceMm: device.distance_max,
        minEnergy: device.energy_min,
        maxEnergy: device.energy_max,
      },
      map: {
        radarXmm: device.radar_x,
        radarYmm: device.radar_y,
        xLines: [150, device.x1, device.x2, device.x3, device.x4, 2250],
        yLines: [150, device.y1, device.y2, device.y3, device.y4, 2250],
        obstaclePointThreshold: device.threshold,
        fixedBlockedMask: device.fixed_mask,
        manualBlockedMask: device.manual_mask,
      },
      planningMode: device.planning_mode === 'point' ? 'point-to-point' : 'mission',
      missionRoute: device.mission,
      start: {...device.start, anchor: anchorName(device.start?.anchor)},
      goal: {...device.goal, anchor: anchorName(device.goal?.anchor)},
      displayMode: config.displayMode,
      theme: config.theme,
      radarPort: config.radarPort,
      radarBaudrate: config.radarBaudrate,
    });
    syncInputs();
  }

  function cellsFromDevice(grid, deviceMap, deviceConfig) {
    const fixedMask = (deviceConfig?.fixed_mask ?? grid.fixedBlockedMask) >>> 0;
    const protectedMask = (deviceConfig?.protected_mask ?? 0x00404405) >>> 0;
    const manualMask = (deviceMap?.manual_mask ?? config.map.manualBlockedMask) >>> 0;
    const scannedMask = (deviceMap?.scanned_mask ?? 0) >>> 0;
    const effectiveMask = (deviceMap?.effective_mask ?? fixedMask) >>> 0;
    return grid.cells.map(cell => {
      const bit = (1 << cell.bitIndex) >>> 0;
      const fixed = !!(fixedMask & bit), protectedCell = !!(protectedMask & bit);
      const manual = !protectedCell && !!(manualMask & bit), scanned = !protectedCell && !!(scannedMask & bit);
      return {...cell, flags: {fixed, manual, scanned}, protected: protectedCell,
        scannedPointCount: deviceMap?.counts?.[cell.bitIndex] ?? 0, blocked: !!(effectiveMask & bit)};
    });
  }

  function planFromDevice(deviceMap, devicePath) {
    const pending = !deviceMap || deviceMap.path_valid && !sameIdentity(deviceMap, devicePath);
    const points = sameIdentity(deviceMap, devicePath) ? devicePath.points || [] : [];
    const waypoints = points.map(([xmm, ymm, nextDistanceMm, row, column, direction, turn]) =>
      ({xmm, ymm, nextDistanceMm, row, column, direction, turn}));
    return {
      success: !!deviceMap?.path_valid && !!devicePath?.path_valid && waypoints.length > 0,
      waypoints,
      pending,
      totalDistanceMm: waypoints.reduce((sum, point) => sum + point.nextDistanceMm, 0),
      failureReason: pending ? (deviceMap ? '地图已收到，路径结果读取中' : '等待 F407 地图与路径') : `第 ${deviceMap.failed_from_task || '—'} 到 ${deviceMap.failed_to_task || '—'} 个任务点之间不可达`,
    };
  }

  function render() {
    if (disposed) return;
    syncActiveConfig();
    const snapshots = hostClient.model.snapshots;
    const status = hostClient.model.status;
    const continuous = status?.l === 1 || status?.l === true;
    const deviceConfig = snapshots.config?.source === 'active' ? snapshots.config : null;
    const deviceMap = snapshots.map || null;
    const devicePath = snapshots.path || null;
    const matchingCloud = sameIdentity(deviceMap || status, snapshots.cloud) ? snapshots.cloud : null;
    const preview = snapshots.preview && (!status || snapshots.preview.session === status.session) ? snapshots.preview : null;
    const sourcePoints = matchingCloud?.points || preview?.points || [];
    const rawPoints = sourcePoints;
    const filteredPoints = matchingCloud ? filterRadarPoints(rawPoints, config.filter) : rawPoints;
    const grid = createGrid(config.map);
    const cells = cellsFromDevice(grid, deviceMap, deviceConfig);
    const plan = planFromDevice(deviceMap, devicePath);
    const displayed = config.displayMode === 'raw' ? rawPoints : filteredPoints;
    const displayFieldPoints = displayed.map(point => toFieldPoint(point, grid.radar));
    const missionPoints = resolveMissionPoints(grid, grid.radar);
    derived = {rawPoints, filteredPoints, grid, cells, plan, displayedPoints: displayed, missionPoints,
      deviceMap, devicePath, matchingCloud, preview};
    root.dataset.theme = config.theme;
    refs['radar-direct-show-raw'].dataset.active = String(config.displayMode === 'raw'); refs['radar-direct-show-filtered'].dataset.active = String(config.displayMode === 'filtered');
    refs['radar-direct-theme-dark'].dataset.active = String(config.theme === 'dark'); refs['radar-direct-theme-light'].dataset.active = String(config.theme === 'light');
    const pointStart = config.planningMode === 'point-to-point' ? resolvePlanningEndpoint({grid, endpoint: config.start}) : null;
    const pointGoal = config.planningMode === 'point-to-point' ? resolvePlanningEndpoint({grid, endpoint: config.goal}) : null;
    mapRenderer.render({fieldSize: grid.fieldSize, activeMin: grid.activeMinimum, activeMax: grid.activeMaximum, xLines: grid.xLines, yLines: grid.yLines, radar: grid.radar, points: displayFieldPoints, cells, path: plan.success ? plan.waypoints.map(point => ({x: point.xmm, y: point.ymm})) : [], missionPoints: missionPoints.map(point => ({number: point.number, row: point.endpoint.row, column: point.endpoint.column, x: point.xmm, y: point.ymm})), start: pointStart, goal: pointGoal, theme: config.theme});
    refs['radar-direct-freeze'].textContent = deviceMap ? `${deviceMap.historical ? '历史 · ' : ''}设备地图已冻结` : '等待扫描';
    refs['radar-direct-freeze'].parentElement.dataset.ready = String(!!deviceMap);
    refs['radar-direct-point-summary'].textContent = deviceMap ? `${displayed.length} 个当前显示点 / F407 原始 ${deviceMap.raw_points} · 筛选 ${deviceMap.filtered_points}` : `${displayed.length} 个当前显示点 / ${status?.map ? '地图结果读取中' : '等待完整圈建图'}`;
    refs['radar-direct-metric-raw'].textContent = String(deviceMap?.raw_points ?? '—'); refs['radar-direct-metric-filtered'].textContent = String(deviceMap?.filtered_points ?? '—');
    refs['radar-direct-metric-blocked'].textContent = String(cells.filter(cell => cell.blocked).length); refs['radar-direct-metric-distance'].textContent = plan.success ? plan.totalDistanceMm.toFixed(0) : '—';
    refs['radar-direct-counters'].replaceChildren(...[['原始点', deviceMap?.raw_points ?? '—'], ['筛选点', deviceMap?.filtered_points ?? '—'], ['网格外点', deviceMap?.outside_points ?? '—'], ['分页校验', hostClient.model.lastError || (deviceMap ? '正常' : '等待结果')]].map(([name, value]) => { const row = el('div'); row.append(el('dt', {text: name}), el('dd', {text: String(value)})); return row; }));
    if(deviceMap?.outside_points > 0 && deviceMap.filtered_points - deviceMap.outside_points <= 1){
      const row=el('div');row.append(el('dt',{text:'点位提示'}),el('dd',{text:`${deviceMap.outside_points}/${deviceMap.filtered_points} 点在判障网格外（150～2250 mm），请核对位置与0°方向`}));refs['radar-direct-counters'].append(row);
    }
    const scanNames = ['空闲', '准备', '等待整圈', '采集中', '完成', '已停止', '错误'];
    const liveNames = ['准备', '等待整圈', '实时更新', '已停止', '错误'];
    refs['radar-direct-flags'].replaceChildren(...[['扫描状态', status ? scanNames[status.ss] || status.ss : '等待状态'], ['实时状态', status ? liveNames[status.ls] || status.ls : '等待状态'], ['完整点云', matchingCloud ? `${matchingCloud.points.length} 点（按需诊断）` : deviceMap ? (status?.cloud === 0 ? '本圈：未缓存完整点云' : '保留在 F407 · 不自动上传') : '未上传'], ['实时预览', preview ? `${preview.points.length} 点（仅显示）` : '尚无预览']].map(([name, value]) => { const row = el('div'); row.append(el('dt', {text: name}), el('dd', {text: String(value)})); return row; }));
    const routeSummary = refs['radar-direct-route-summary']; routeSummary.replaceChildren(el('h3', {text: `路径规划 · ${plan.success ? '可通行' : plan.pending ? '待同步' : '无可用路径'}`}), el('p', {text: plan.success ? `${plan.waypoints.length} 个坐标点，总距离 ${plan.totalDistanceMm.toFixed(0)} mm` : plan.failureReason}), ...(config.planningMode === 'mission' ? [el('strong', {text: `任务顺序 ${formatMissionRoute(config.missionRoute)}`})] : []));
    refs['radar-direct-waypoints'].replaceChildren(...plan.waypoints.map((point, index) => { const tr = el('tr'); for (const value of [index + 1, `R${point.row}C${point.column}`, point.xmm, point.ymm, point.nextDistanceMm || '—', directionText(point.direction), turnText(point.turn)]) tr.append(el('td', {text: String(value)})); return tr; }));
    refs['radar-direct-preview-title'].textContent = `${matchingCloud ? (config.displayMode === 'raw' ? '完整原始点' : '完整点云本地显示筛选') : 'F407 实时预览'} · 前 12 条`;
    refs['radar-direct-preview'].replaceChildren(...displayed.slice(0, 12).map(point => { const tr = el('tr'); for (const value of [`${point.angle_deg.toFixed(2)}°`, point.distance_mm, point.energy ?? '—']) tr.append(el('td', {text: String(value)})); return tr; }));
    refs['radar-direct-csv'].disabled = !matchingCloud;
    const connected = connectionState.state === 'connected', compatible = connected && Number(connectionState.baudrate) === 115200;
    port.disabled = connectionState.state !== 'disconnected' || busy; baud.disabled = true; refreshPortsButton.disabled = connectionState.state !== 'disconnected' || busy;
    refs['radar-direct-connect'].disabled = connected || busy || !port.value; refs['radar-direct-disconnect'].disabled = !connected || busy;
    for (const id of ['radar-direct-start', 'radar-direct-stop', 'radar-direct-single', 'radar-direct-read-status']) refs[id].disabled = !compatible || busy;
    refs['radar-direct-single'].disabled = !compatible || busy || !!status?.a || !!hostClient.pendingStart;
    refs['radar-direct-continuous'].disabled = !compatible || busy; refs['radar-direct-continuous'].textContent = continuous ? '停止持续扫描' : '持续扫描';
    refs['radar-direct-connection-status'].textContent = busy ? '正在处理…' : continuous ? `F407 实时预览中：最新 ${preview?.points.length ?? 0} 点` : compatible ? `共享串口已连接 ${connectionState.port || port.value} / 115200` : connected ? `共享串口当前为 ${connectionState.baudrate}，雷达页需断开后以 115200 重连` : '设备未连接 · 已保留最后完整设备结果';
    if(!busy&&compatible){
      if(hostClient.lastError)refs['radar-direct-connection-status'].textContent=`${hostClient.lastError} · 可点击“读取状态/结果”重试`;
      else if(status?.ss===6)refs['radar-direct-connection-status'].textContent=`F407 扫描失败：${status.reason || '未知原因'}`;
      else if(status?.a)refs['radar-direct-connection-status'].textContent=`F407 ${scanNames[status.ss] || '扫描中'} · ${status.reason || ''}`;
      else if(status?.ss===4)refs['radar-direct-connection-status'].textContent=continuous ? '持续建图 · 每圈完整原始点判障，按无线节奏更新障碍和路线' : '扫描完成 · 正常仅同步障碍格统计和路线，完整点云不自动上传';
    }
  }

  const mapRenderer = mapFactory({canvas, onBoundaryDragged(axis, index, millimetres) {
    return applyDeviceConfig({[`${axis.toLowerCase()}${index + 1}`]: millimetres}, `${axis}${index + 1} 已由设备读回为 ${millimetres} mm`);
  }, onCellClicked(row, column) {
    const cell = derived?.grid.getCell(row, column); if (!cell) return;
    const classified = derived.cells[cell.bitIndex]; if (classified.protected || classified.flags.fixed) { refs['radar-direct-settings-status'].textContent = classified.protected ? '任务点所在格受保护，不能设为障碍。' : '固定禁区不能手工取消。'; return; }
    const bit = (1 << cell.bitIndex) >>> 0, manual = (hostClient.model.snapshots.map?.manual_mask ?? config.map.manualBlockedMask) >>> 0;
    return applyDeviceConfig({manual_mask: (manual ^ bit) >>> 0}, `R${row}C${column} 手动障碍已由设备确认`);
  }});

  async function runHost(action) {
    if (busy) return;
    busy = true; render();
    try { await hostClient.command(action); log(`操作已写出：${action}`); }
    catch (error) { log(`操作失败：${error.message}`); refs['radar-direct-connection-status'].textContent = error.message; }
    finally { busy = false; render(); }
  }
  async function runConnection(action) {
    if (busy) return;
    busy = true; render();
    try {
      if (action === 'connect') await connectionController.connect(port.value, 115200);
      else await connectionController.disconnect();
    } catch (error) { log(`串口操作失败：${error.message}`); refs['radar-direct-connection-status'].textContent = error.message; }
    finally { busy = false; render(); }
  }
  async function applyDeviceConfig(values, successText) {
    if (busy) return;
    busy = true; refs['radar-direct-settings-status'].textContent = '正在写入 F407 RAM 并等待有效配置读回…'; render();
    try {
      await hostClient.applyConfig(values); syncActiveConfig(); render();
      const status=hostClient.model.status;
      refs['radar-direct-settings-status'].textContent = status?.l ? 'F407 配置已应用并读回；下一完整圈刷新障碍和路线，不重启雷达。' : status?.cloud===0 ? 'F407 配置已应用并读回；无完整点云缓存时在下次扫描生效，旧图保留。' : successText || 'F407 已应用并读回配置；地图与路径来自设备重算结果。';
    } catch (error) { refs['radar-direct-settings-status'].textContent = error.message; log(`配置未完成：${error.message}`); }
    finally { busy = false; render(); }
  }
  async function refreshPorts() {
    try {
      const result = await connectionController.listPorts(), ports = Array.isArray(result) ? result : result?.ports || [];
      const chosen = port.value || connectionState.port || config.radarPort;
      port.replaceChildren(el('option', {value: '', text: ports.length ? '请选择串口' : '未发现串口'}), ...ports.map(item => el('option', {value: item.device, text: item.description ? `${item.device} · ${item.description}` : item.device})));
      if (ports.some(item => item.device === chosen)) port.value = chosen;
      render();
    } catch (error) { log(`刷新串口失败：${error.message}`); }
  }

  refreshPortsButton.onclick = () => refreshPorts();
  port.onchange = () => render();
  refs['radar-direct-connect'].onclick = () => runConnection('connect');
  refs['radar-direct-disconnect'].onclick = () => runConnection('disconnect'); refs['radar-direct-start'].onclick = () => runHost('start'); refs['radar-direct-stop'].onclick = () => runHost('stop'); refs['radar-direct-single'].onclick = () => runHost('scan');
  refs['radar-direct-continuous'].onclick = () => runHost(hostClient.model.status?.l ? 'live-stop' : 'live-start');
  refs['radar-direct-read-status'].onclick = () => runHost('status');
  refs['radar-direct-show-raw'].onclick = () => { setLocalConfig({...config, displayMode: 'raw'}); };
  refs['radar-direct-show-filtered'].onclick = () => { setLocalConfig({...config, displayMode: 'filtered'}); };
  refs['radar-direct-theme-dark'].onclick = () => { setLocalConfig({...config, theme: 'dark'}); mapRenderer.setTheme?.('dark'); };
  refs['radar-direct-theme-light'].onclick = () => { setLocalConfig({...config, theme: 'light'}); mapRenderer.setTheme?.('light'); };
  refs['radar-direct-apply-filter'].onclick = () => {
    const values = {angle_min: Math.round(Number(refs['radar-direct-min-angle'].value) * 10), angle_max: Math.round(Number(refs['radar-direct-max-angle'].value) * 10), distance_min: Number(refs['radar-direct-min-distance'].value), distance_max: Number(refs['radar-direct-max-distance'].value), energy_min: Number(refs['radar-direct-min-energy'].value), energy_max: Number(refs['radar-direct-max-energy'].value)};
    return applyDeviceConfig(values, '筛选配置已由 F407 应用并读回；无需重新扫描。');
  };
  refs['radar-direct-apply-map'].onclick = () => {
    try {
      const parsed = parseMissionRoute(refs['radar-direct-mission-route'].value); if (planningMode.value === 'mission' && !parsed.ok) throw new Error(parsed.error);
      const start = endpointFromValue(refs['radar-direct-start-cell'].value, config.start), goal = endpointFromValue(refs['radar-direct-goal-cell'].value, config.goal);
      const values = {radar_x: Number(refs['radar-direct-radar-x'].value), radar_y: Number(refs['radar-direct-radar-y'].value), threshold: Number(refs['radar-direct-threshold'].value), planning_mode: planningMode.value === 'mission' ? 'mission' : 'point', mission: parsed.ok ? parsed.route : config.missionRoute, start: deviceEndpoint(start), goal: deviceEndpoint(goal)};
      for (const axis of ['x', 'y']) for (let index = 1; index <= 4; index += 1) values[`${axis}${index}`] = Number(refs[`radar-direct-${axis}${index}`].value);
      return applyDeviceConfig(values, '场地、障碍格与路径已由 F407 从当前冻结圈重新计算。');
    } catch (error) { refs['radar-direct-settings-status'].textContent = error.message; }
  };
  planningMode.onchange = () => { refs['radar-direct-endpoints'].hidden = planningMode.value !== 'point-to-point'; missionRoute.parentElement.hidden = planningMode.value !== 'mission'; };
  refs['radar-direct-save'].onclick = () => { const next = clone(config); next.radarPort = port.value; const result = saveRadarDirectPreferences(storage, next); if (result.ok) { config = normalizeRadarConfig(next); log('本机显示与串口选择已保存；未向 F407 发送指令'); } else log(`保存失败：${result.error}`); };
  refs['radar-direct-csv'].onclick = () => {
    const cloud = derived?.matchingCloud; if (!cloud) return; const url = URL.createObjectURL(new Blob([scanToCsv(cloud.points)], {type: 'text/csv;charset=utf-8'})); const link = el('a'); link.href = url; link.download = `lds50c-f407-scan-${cloud.scan}.csv`; link.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  };

  if (loaded.warning) log(loaded.warning);
  syncInputs(); render(); void refreshPorts();
  return {
    receive(line) { const accepted = hostClient.receive(line); if (accepted) render(); return accepted; },
    connection(snapshot) {
      connectionState = typeof snapshot === 'object' ? {...connectionState, ...snapshot} : {state: snapshot ? 'connected' : 'disconnected', port: connectionState.port, baudrate: connectionState.baudrate};
      hostClient.connection(connectionState);
      if (connectionState.port && !Array.from(port.children || []).some(option => option.value === connectionState.port)) port.append(el('option', {value: connectionState.port, text: connectionState.port}));
      if (connectionState.port) port.value = connectionState.port;
      render();
    },
    setVisible(value) { root.hidden = !value; },
    refresh() { render(); },
    dispose() { if (disposed) return; hostClient.cancelTransfer?.(); mapRenderer.dispose(); disposed = true; },
    snapshot() { return {config: clone(config), connection: clone(connectionState), deviceMap: clone(derived?.deviceMap || null), devicePath: clone(derived?.devicePath || null), scan: clone(derived?.matchingCloud || null), rawPoints: clone(derived?.rawPoints || []), filteredPoints: clone(derived?.filteredPoints || []), displayedPoints: clone(derived?.displayedPoints || []), obstacles: clone(derived?.cells || []), plan: clone(derived?.plan || null)}; },
  };
}
