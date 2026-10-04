import {RadarDirectClient} from './radar-direct-client.mjs';
import {
  classifyObstacleCells,
  createGrid,
  filterRadarPoints,
  loadRadarDirectPreferences,
  normalizeRadarConfig,
  saveRadarDirectPreferences,
  scanToCsv,
  toFieldPoint,
} from './radar-direct-model.mjs';
import {
  buildWaypointRows,
  formatMissionRoute,
  parseMissionRoute,
  planMissionRoute,
  planPointToPoint,
  resolveMissionPoints,
} from './radar-direct-planner.mjs';
import {mountRadarDirectMap} from './radar-direct-map.mjs';

const directionText = value => ({north: '向北', east: '向东', south: '向南', west: '向西'})[value] || '—';
const turnText = value => ({start: '起步', straight: '直行', left: '左转', right: '右转', reverse: '掉头'})[value] || '到达';
const clone = value => structuredClone(value);

/** Mount the phase-one USB-UART LDS50C host. All hardware writes are button driven. */
export function mountRadarDirectHost({
  root,
  api,
  storage = globalThis.localStorage,
  clientFactory = hooks => new RadarDirectClient({api, ...hooks}),
  mapFactory = options => mountRadarDirectMap(options),
} = {}) {
  if (!root || typeof api !== 'function') throw new TypeError('Direct radar host needs a root and api');
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
  let state = {connected: false, operation: 'idle', continuous: false, scan_revision: 0, parser: {}, radar_status: null};
  let scan = null;
  let derived = null;
  let busy = false;
  let disposed = false;
  const logs = [];

  root.className = 'radar-direct-host';
  const hero = el('header', {cls: 'radar-direct-hero'});
  const heroCopy = el('div'); heroCopy.append(el('span', {cls: 'radar-direct-kicker', text: 'LDS-50C · ONE-SCAN MAPPING'}), el('h1', {text: '雷达建图与路径规划'}), el('p', {text: '一次扫描冻结原始点云，实时重筛选、重分区并规划 5 × 5 场地路径。'}));
  const heroStatus = el('div', {cls: 'radar-direct-hero-status'}); heroStatus.append(el('span', {cls: 'radar-direct-status-dot'}), el('strong', {id: 'radar-direct-freeze', text: '等待扫描'}), el('small', {id: 'radar-direct-point-summary', text: '0 个显示点 / 0 个接收点'}));
  hero.append(heroCopy, heroStatus);

  const connection = el('section', {cls: 'radar-direct-connection glass'}); connection.append(panelHeading('SERIAL WORKFLOW', '设备与扫描'));
  const connectionFields = el('div', {cls: 'radar-direct-connection-fields'});
  const port = field(connectionFields, 'radar-direct-port', '串口', config.radarPort, {values: [['', '未选择串口']]});
  const baud = field(connectionFields, 'radar-direct-baud', '波特率', config.radarBaudrate, {values: [384000, 500000, 768000, 921600].map(value => [value, value])});
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
    ['radar-direct-save', '保存本机配置'],
  ]) actions.append(button(...item));
  connection.append(actions, el('p', {id: 'radar-direct-connection-status', cls: 'radar-direct-feedback', text: '设备未连接'}));

  const workspace = el('section', {cls: 'radar-direct-grid'});
  const controls = el('aside', {cls: 'radar-direct-controls'});
  const filterPanel = el('section', {cls: 'radar-direct-panel glass'}); filterPanel.append(panelHeading('POINT FILTER', '点云筛选'));
  const displayButtons = el('div', {cls: 'radar-direct-segments'}); displayButtons.append(button('radar-direct-show-raw', '接收原始'), button('radar-direct-show-filtered', '上位机筛选'));
  const themeButtons = el('div', {cls: 'radar-direct-segments'}); themeButtons.append(button('radar-direct-theme-dark', '黑'), button('radar-direct-theme-light', '白'));
  filterPanel.append(el('p', {cls: 'radar-direct-subtitle', text: '点云显示 · 同一完整扫描圈'}), displayButtons, themeButtons);
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

  function setConfig(next) {
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
      'radar-direct-goal-cell': endpointValue(config.goal), 'radar-direct-baud': config.radarBaudrate,
    };
    for (const axis of ['x', 'y']) for (let index = 1; index <= 4; index += 1) values[`radar-direct-${axis}${index}`] = config.map[`${axis}Lines`][index];
    for (const [id, value] of Object.entries(values)) refs[id].value = String(value);
    if (config.radarPort) refs['radar-direct-port'].value = config.radarPort;
    refs['radar-direct-endpoints'].hidden = config.planningMode !== 'point-to-point';
    missionRoute.parentElement.hidden = config.planningMode !== 'mission';
  }

  function render() {
    if (disposed) return;
    const rawPoints = scan?.points || [];
    const filteredPoints = filterRadarPoints(rawPoints, config.filter);
    const grid = createGrid(config.map);
    const fieldPoints = filteredPoints.map(point => toFieldPoint(point, {x: config.map.radarXmm, y: config.map.radarYmm}));
    const obstacles = classifyObstacleCells(grid, fieldPoints, config.map.obstaclePointThreshold, config.map.manualBlockedMask);
    const plan = config.planningMode === 'mission'
      ? planMissionRoute({grid, blockedMask: obstacles.blockedMask, route: config.missionRoute, radar: grid.radar})
      : planPointToPoint({grid, blockedMask: obstacles.blockedMask, start: config.start, goal: config.goal, radar: grid.radar});
    const displayed = config.displayMode === 'raw' ? rawPoints : filteredPoints;
    const displayFieldPoints = displayed.map(point => toFieldPoint(point, grid.radar));
    const missionPoints = resolveMissionPoints(grid, grid.radar);
    derived = {rawPoints, filteredPoints, grid, obstacles, plan, displayedPoints: displayed, missionPoints};
    root.dataset.theme = config.theme;
    refs['radar-direct-show-raw'].dataset.active = String(config.displayMode === 'raw'); refs['radar-direct-show-filtered'].dataset.active = String(config.displayMode === 'filtered');
    refs['radar-direct-theme-dark'].dataset.active = String(config.theme === 'dark'); refs['radar-direct-theme-light'].dataset.active = String(config.theme === 'light');
    mapRenderer.render({fieldSize: grid.fieldSize, activeMin: grid.activeMinimum, activeMax: grid.activeMaximum, xLines: grid.xLines, yLines: grid.yLines, radar: grid.radar, points: displayFieldPoints, cells: obstacles.cells, path: plan.success ? plan.waypoints.map(point => ({x: point.xmm, y: point.ymm})) : [], missionPoints: missionPoints.map(point => ({number: point.number, row: point.endpoint.row, column: point.endpoint.column, x: point.xmm, y: point.ymm})), start: config.planningMode === 'point-to-point' ? config.start : null, goal: config.planningMode === 'point-to-point' ? config.goal : null, theme: config.theme});
    refs['radar-direct-freeze'].textContent = scan ? '地图已冻结' : '等待扫描';
    refs['radar-direct-freeze'].parentElement.dataset.ready = String(!!scan);
    refs['radar-direct-point-summary'].textContent = `${displayed.length} 个显示点 / ${rawPoints.length} 个接收点`;
    refs['radar-direct-metric-raw'].textContent = String(rawPoints.length); refs['radar-direct-metric-filtered'].textContent = String(filteredPoints.length);
    refs['radar-direct-metric-blocked'].textContent = String(obstacles.cells.filter(cell => cell.blocked).length); refs['radar-direct-metric-distance'].textContent = plan.success ? plan.totalDistanceMm.toFixed(0) : '—';
    const parser = state.parser || {};
    refs['radar-direct-counters'].replaceChildren(...[['有效帧', parser.valid_frames || 0], ['校验错误', parser.checksum_errors || 0], ['长度错误', parser.length_errors || 0], ['丢弃字节', parser.discarded_bytes || 0]].map(([name, value]) => { const row = el('div'); row.append(el('dt', {text: name}), el('dd', {text: String(value)})); return row; }));
    const actual = state.radar_status;
    refs['radar-direct-flags'].replaceChildren(...[['距离单位', actual ? actual.uses_millimetres ? '毫米' : '厘米（预期毫米）' : '等待状态'], ['能量数据', actual ? actual.energy_enabled ? '开启' : '关闭（预期开启）' : '等待状态'], ['去拖点', actual ? actual.trailing_removal_enabled ? '开启' : '关闭（预期开启）' : '等待状态'], ['雷达滤波', actual ? actual.filter_enabled ? '开启' : '关闭（预期开启）' : '等待状态']].map(([name, value]) => { const row = el('div'); row.append(el('dt', {text: name}), el('dd', {text: value})); return row; }));
    const routeSummary = refs['radar-direct-route-summary']; routeSummary.replaceChildren(el('h3', {text: `路径规划 · ${plan.success ? '可通行' : '无可用路径'}`}), el('p', {text: plan.success ? `${plan.cells.length} 个展开格，${plan.waypoints.length} 个坐标点，总距离 ${plan.totalDistanceMm.toFixed(0)} mm` : plan.failureReason}), ...(config.planningMode === 'mission' ? [el('strong', {text: `任务顺序 ${formatMissionRoute(config.missionRoute)}`})] : []));
    refs['radar-direct-waypoints'].replaceChildren(...buildWaypointRows(plan).map(row => { const tr = el('tr'); for (const value of [row.sequence, row.cell, row.xmm, row.ymm, row.nextDistanceMm ?? '—', directionText(row.nextDirection), turnText(row.nextTurn)]) tr.append(el('td', {text: String(value)})); return tr; }));
    refs['radar-direct-preview-title'].textContent = `${config.displayMode === 'raw' ? '接收原始点' : '上位机筛选点'}预览 · 前 12 条`;
    refs['radar-direct-preview'].replaceChildren(...displayed.slice(0, 12).map(point => { const tr = el('tr'); for (const value of [`${point.angle_deg.toFixed(2)}°`, point.distance_mm, point.energy]) tr.append(el('td', {text: String(value)})); return tr; }));
    refs['radar-direct-csv'].disabled = !scan;
    const connected = !!state.connected, continuous = !!state.continuous || state.operation === 'continuous';
    port.disabled = connected || busy; baud.disabled = connected || busy; refreshPortsButton.disabled = connected || busy;
    refs['radar-direct-connect'].disabled = connected || busy || !port.value; refs['radar-direct-disconnect'].disabled = !connected || busy;
    for (const id of ['radar-direct-start', 'radar-direct-stop', 'radar-direct-single']) refs[id].disabled = !connected || busy || continuous;
    refs['radar-direct-continuous'].disabled = !connected || busy; refs['radar-direct-continuous'].textContent = continuous ? '停止持续扫描' : '持续扫描';
    refs['radar-direct-connection-status'].textContent = busy ? '正在处理…' : continuous ? `持续扫描中：最新 ${rawPoints.length} 个原始点` : connected ? `已连接 ${state.port || port.value} / ${state.baudrate || baud.value}` : '设备未连接 · 已保留最后完整扫描';
  }

  const mapRenderer = mapFactory({canvas, onBoundaryDragged(axis, index, millimetres) {
    try { const next = clone(config); next.map[`${axis.toLowerCase()}Lines`][index + 1] = millimetres; setConfig(next); refs['radar-direct-settings-status'].textContent = `${axis}${index + 1} 已更新为 ${millimetres} mm`; } catch (error) { refs['radar-direct-settings-status'].textContent = error.message; }
  }, onCellClicked(row, column) {
    const cell = derived?.grid.getCell(row, column); if (!cell) return;
    const classified = derived.obstacles.cells[cell.bitIndex]; if (classified.protected || classified.flags.fixed) { refs['radar-direct-settings-status'].textContent = classified.protected ? '任务点所在格受保护，不能设为障碍。' : '固定禁区不能手工取消。'; return; }
    const next = clone(config), bit = (1 << cell.bitIndex) >>> 0; next.map.manualBlockedMask = (next.map.manualBlockedMask ^ bit) >>> 0; setConfig(next);
  }});
  const client = clientFactory({
    onState(value) { state = value; render(); },
    onScan(value) { if (value?.points) { scan = value; log(`完整扫描圈 ${value.revision}：${value.points.length} 个原始点`); render(); } },
    onEvents(events) { for (const event of events) log(event.text || event.kind || '雷达事件'); },
    onError(error) { log(`本地服务：${error.message}`); refs['radar-direct-connection-status'].textContent = `本地服务暂不可用：${error.message}`; },
  });

  async function run(action, body = {}) {
    if (busy) return;
    busy = true; render();
    try { await client.command(action, body); log(`操作完成：${action}`); }
    catch (error) { log(`操作失败：${error.message}`); refs['radar-direct-connection-status'].textContent = error.message; }
    finally { busy = false; render(); }
  }
  async function refreshPorts() {
    try {
      const result = await api('radar-direct/ports');
      const chosen = port.value || config.radarPort;
      port.replaceChildren(el('option', {value: '', text: result.ports?.length ? '请选择串口' : '未发现串口'}), ...(result.ports || []).map(item => el('option', {value: item.device, text: item.description ? `${item.device} · ${item.description}` : item.device})));
      if ((result.ports || []).some(item => item.device === chosen)) port.value = chosen;
      render();
    } catch (error) { log(`刷新串口失败：${error.message}`); }
  }

  refreshPortsButton.onclick = () => refreshPorts();
  refs['radar-direct-connect'].onclick = () => run('connect', {port: port.value, baudrate: Number(baud.value)});
  refs['radar-direct-disconnect'].onclick = () => run('disconnect'); refs['radar-direct-start'].onclick = () => run('start'); refs['radar-direct-stop'].onclick = () => run('stop'); refs['radar-direct-single'].onclick = () => run('scan');
  refs['radar-direct-continuous'].onclick = () => run(state.continuous || state.operation === 'continuous' ? 'continuous-stop' : 'continuous-start');
  refs['radar-direct-show-raw'].onclick = () => { setConfig({...config, displayMode: 'raw'}); };
  refs['radar-direct-show-filtered'].onclick = () => { setConfig({...config, displayMode: 'filtered'}); };
  refs['radar-direct-theme-dark'].onclick = () => { setConfig({...config, theme: 'dark'}); mapRenderer.setTheme?.('dark'); };
  refs['radar-direct-theme-light'].onclick = () => { setConfig({...config, theme: 'light'}); mapRenderer.setTheme?.('light'); };
  refs['radar-direct-apply-filter'].onclick = () => {
    try { const next = clone(config); next.filter = {minAngleDeg: Number(refs['radar-direct-min-angle'].value), maxAngleDeg: Number(refs['radar-direct-max-angle'].value), minDistanceMm: Number(refs['radar-direct-min-distance'].value), maxDistanceMm: Number(refs['radar-direct-max-distance'].value), minEnergy: Number(refs['radar-direct-min-energy'].value), maxEnergy: Number(refs['radar-direct-max-energy'].value)}; setConfig(next); refs['radar-direct-settings-status'].textContent = '已对当前完整扫描圈重新筛选。'; } catch (error) { refs['radar-direct-settings-status'].textContent = error.message; }
  };
  refs['radar-direct-apply-map'].onclick = () => {
    try {
      const parsed = parseMissionRoute(refs['radar-direct-mission-route'].value); if (planningMode.value === 'mission' && !parsed.ok) throw new Error(parsed.error);
      const next = clone(config); next.map.radarXmm = Number(refs['radar-direct-radar-x'].value); next.map.radarYmm = Number(refs['radar-direct-radar-y'].value); next.map.obstaclePointThreshold = Number(refs['radar-direct-threshold'].value);
      next.map.xLines = [150, ...[1, 2, 3, 4].map(index => Number(refs[`radar-direct-x${index}`].value)), 2250]; next.map.yLines = [150, ...[1, 2, 3, 4].map(index => Number(refs[`radar-direct-y${index}`].value)), 2250];
      next.planningMode = planningMode.value; if (parsed.ok) next.missionRoute = parsed.route; next.start = endpointFromValue(refs['radar-direct-start-cell'].value, next.start); next.goal = endpointFromValue(refs['radar-direct-goal-cell'].value, next.goal); setConfig(next); refs['radar-direct-settings-status'].textContent = '场地、障碍格与路径已从当前冻结圈重新计算。';
    } catch (error) { refs['radar-direct-settings-status'].textContent = error.message; }
  };
  planningMode.onchange = () => { const next = clone(config); next.planningMode = planningMode.value; setConfig(next); };
  refs['radar-direct-save'].onclick = () => { const next = clone(config); next.radarPort = port.value; next.radarBaudrate = Number(baud.value); const result = saveRadarDirectPreferences(storage, next); if (result.ok) { config = normalizeRadarConfig(next); log('本机配置已保存'); } else log(`保存失败：${result.error}`); };
  refs['radar-direct-csv'].onclick = () => {
    if (!scan) return; const url = URL.createObjectURL(new Blob([scanToCsv(scan.points)], {type: 'text/csv;charset=utf-8'})); const link = el('a'); link.href = url; link.download = `lds50c-scan-${scan.revision}.csv`; link.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  };

  if (loaded.warning) log(loaded.warning);
  syncInputs(); render(); void refreshPorts(); void client.start();
  return {
    setVisible(value) { root.hidden = !value; },
    refresh() { return client.refresh(); },
    dispose() { if (disposed) return; disposed = true; client.suspend(); mapRenderer.dispose(); },
    snapshot() { return {config: clone(config), state: clone(state), scan: scan ? clone(scan) : null, rawPoints: clone(derived?.rawPoints || []), filteredPoints: clone(derived?.filteredPoints || []), obstacles: clone(derived?.obstacles || null), plan: clone(derived?.plan || null)}; },
  };
}
