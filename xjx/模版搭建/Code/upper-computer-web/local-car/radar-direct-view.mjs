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
import {installationParameters,RADAR_INSTALLATION_PRESETS,projectFrozenRadarPoint,stableFieldPoint,radarSnapshotFrame} from './radar-installation.mjs';

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
  let draftDirty=false,deviceDraft=null,frozenConfig=null,frozenKey='';
  const configurationHistory=new Map();
  let installationDraft=installationParameters('case1');
  try{const saved=JSON.parse(storage?.getItem?.('local-car-radar-installation-v1')??'null');if(saved&&Object.values(saved).every(Number.isFinite))installationDraft={...installationDraft,...saved};}catch{}
  let transferProgress='尚未读取',transferActive=false;
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
  const heroCopy = el('div'); heroCopy.append(el('span', {cls: 'radar-direct-kicker', text: 'F407 · LDS-50C · RADAR MAPPING'}), el('h1', {text: '雷达建图与诊断'}), el('p', {text: '由当前F407固件采集15个有效完整圈，输出稳定XY、障碍格和路径；实车执行请使用实车监控入口。'}));
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
    ['radar-direct-single', '采集15圈并规划', 'button button-accent'],
    ['radar-direct-continuous', '持续扫描', 'button button-accent'],
    ['radar-direct-read-status', '读取状态/结果'],
    ['radar-direct-fetch-stable', '读取稳定点'],
    ['radar-direct-fetch-cloud', '读取最后圈原始点'],
    ['radar-direct-cancel-transfer', '取消读取'],
    ['radar-direct-export', '导出完整快照'],
    ['radar-direct-save', '保存本机配置'],
  ]) actions.append(button(...item));
  connection.append(actions, el('p', {id: 'radar-direct-connection-status', cls: 'radar-direct-feedback', text: '设备未连接'}),el('p',{id:'radar-direct-transfer-status',cls:'radar-direct-feedback',text:transferProgress}));

  const workspace = el('section', {cls: 'radar-direct-grid'});
  const controls = el('aside', {cls: 'radar-direct-controls'});
  const filterPanel = el('section', {cls: 'radar-direct-panel glass'}); filterPanel.append(panelHeading('POINT FILTER', '点云筛选'));
  const displayButtons = el('div', {cls: 'radar-direct-segments'}); displayButtons.append(button('radar-direct-show-stable', '稳定XY · 默认'),button('radar-direct-show-raw', '最后圈原始'), button('radar-direct-show-filtered', '最后圈筛选'),button('radar-direct-show-preview', '实时preview120'));
  const themeButtons = el('div', {cls: 'radar-direct-segments'}); themeButtons.append(button('radar-direct-theme-dark', '黑'), button('radar-direct-theme-light', '白'));
  filterPanel.append(el('p', {cls: 'radar-direct-subtitle', text: '稳定点使用15圈窗口、10mm箱、30mm半径和3圈票。原始云仅最后一圈；preview120是独立显示预览。'}), displayButtons, themeButtons);
  field(filterPanel,'radar-direct-import','导入冻结JSON诊断','',{type:'file'}).accept='.json,application/json';
  const filterFields = el('div', {cls: 'radar-direct-two-fields'});
  field(filterFields, 'radar-direct-min-angle', '最小角度 °', config.filter.minAngleDeg, {min: 0, max: 360, step: 0.1});
  field(filterFields, 'radar-direct-max-angle', '最大角度 °', config.filter.maxAngleDeg, {min: 0, max: 360, step: 0.1});
  field(filterFields, 'radar-direct-min-distance', '最小距离 mm', config.filter.minDistanceMm, {min: 0, max: 65535});
  field(filterFields, 'radar-direct-max-distance', '最大距离 mm', config.filter.maxDistanceMm, {min: 0, max: 65535});
  field(filterFields, 'radar-direct-min-energy', '最小能量', config.filter.minEnergy, {min: 0, max: 255});
  field(filterFields, 'radar-direct-max-energy', '最大能量', config.filter.maxEnergy, {min: 0, max: 255});
  filterPanel.append(filterFields, button('radar-direct-apply-filter', '应用筛选 · 重建15圈窗口', 'button button-primary radar-direct-full'));

  const mapSettings = el('section', {cls: 'radar-direct-panel glass'}); mapSettings.append(panelHeading('FIELD CALIBRATION', '场地与分区'));
  const radarFields = el('div', {cls: 'radar-direct-two-fields'});
  field(radarFields, 'radar-direct-radar-x', '雷达 X mm', config.map.radarXmm, {min: 0, max: 2400});
  field(radarFields, 'radar-direct-radar-y', '雷达 Y mm', config.map.radarYmm, {min: 0, max: 2400}); mapSettings.append(radarFields);
  const installationFields=el('div',{cls:'radar-direct-two-fields'});
  field(installationFields,'radar-direct-installation','安装草稿','case1',{values:Object.entries(RADAR_INSTALLATION_PRESETS).map(([key,p])=>[key,p.label])});
  field(installationFields,'radar-direct-zero','0°地图朝向 / °',installationDraft.zero_tenths/10,{min:0,max:359.9,step:.1});mapSettings.append(installationFields);
  const coefficients=el('details');coefficients.append(el('summary',{text:'高级仿射系数 · Q16整数'}));
  const coefficientFields=el('div',{cls:'radar-direct-two-fields'});
  for(const [key,value]of Object.entries(installationDraft).filter(([k])=>k.endsWith('_q16')))field(coefficientFields,`radar-direct-${key}`,key,value,{min:-2147483648,max:2147483647,step:1});
  coefficients.append(coefficientFields);mapSettings.append(coefficients,el('p',{id:'radar-direct-installation-state',cls:'radar-direct-feedback',text:'编辑草稿、设备RAM和冻结采集参数分别保留。'}));
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
  const previewTable = el('table', {cls: 'radar-direct-table'}); const previewBody = el('tbody', {id: 'radar-direct-preview'}),previewHead=el('thead',{id:'radar-direct-preview-head'}); previewTable.append(previewHead, previewBody); diagnostics.append(el('h3', {id: 'radar-direct-preview-title', text: '15圈稳定XY · 前 12 条'}), previewTable);
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
  const draftFromDevice = device => normalizeRadarConfig({
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
  let activeConfigId = null;
  function syncActiveConfig() {
    const capture=hostClient.model.snapshots.capture_config;
    if(capture)configurationHistory.set(capture.config,{config:draftFromDevice(capture),wire:clone(capture)});
    const device=hostClient.model.snapshots.active_config??hostClient.model.snapshots.config;
    if (!device || device.historical || device.source !== 'active' || device.config === activeConfigId) return;
    activeConfigId = device.config;deviceDraft=draftFromDevice(device);
    configurationHistory.set(device.config,{config:clone(deviceDraft),wire:clone(device)});
    if(!draftDirty){config=deviceDraft;syncInputs();
      for(const key of Object.keys(installationDraft))if(device[key]!==undefined)installationDraft[key]=device[key];
      if(device.zero_tenths!==undefined)refs['radar-direct-zero'].value=String(device.zero_tenths/10);
      for(const key of Object.keys(installationDraft).filter(k=>k.endsWith('_q16')))refs[`radar-direct-${key}`].value=String(installationDraft[key]);
    }
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
    const deviceConfig = snapshots.active_config??(snapshots.config?.source === 'active' ? snapshots.config : null);
    const deviceMap = snapshots.map || null;
    const devicePath = snapshots.path || null;
    const matchingCloud = sameIdentity(deviceMap || status, snapshots.cloud) ? snapshots.cloud : null;
    const preview = snapshots.preview && (!status || snapshots.preview.session === status.session) ? snapshots.preview : null;
    const sourcePoints = matchingCloud?.points || preview?.points || [];
    const rawPoints = matchingCloud?.points || [];
    const mapKey=deviceMap?`${deviceMap.session}:${deviceMap.config}:${deviceMap.scan}:${deviceMap.map}:${configurationHistory.has(deviceMap.config)}`:'';
    if(deviceMap&&mapKey!==frozenKey){
      frozenKey=mapKey;frozenConfig=clone(configurationHistory.get(deviceMap.config)?.config??deviceDraft??config);
      if(deviceMap.grid){frozenConfig.map.xLines=[...deviceMap.grid.x];frozenConfig.map.yLines=[...deviceMap.grid.y];}
      if(deviceMap.pose){frozenConfig.map.radarXmm=deviceMap.pose[0];frozenConfig.map.radarYmm=deviceMap.pose[1];}
    }
    const drawingConfig=frozenConfig??deviceDraft??config;
    const filteredPoints = matchingCloud ? filterRadarPoints(rawPoints, drawingConfig.filter) : [];
    const stable=sameIdentity(deviceMap,snapshots.stable)?snapshots.stable:null;
    const grid = createGrid(drawingConfig.map);
    const frozenWire=configurationHistory.get(deviceMap?.config)?.wire;
    const cells = cellsFromDevice(grid, deviceMap, frozenWire);
    const plan = planFromDevice(deviceMap, devicePath);
    const displayed = config.displayMode === 'stable' ? stable?.points??[] : config.displayMode === 'preview'?preview?.points??[]:config.displayMode === 'raw' ? rawPoints : filteredPoints;
    const geometryFromWire=wire=>wire?.zero_tenths!==undefined?{matrix_q16:['m00_q16','m01_q16','m10_q16','m11_q16'].map(k=>wire[k]),origin_q16:[wire.tx_q16,wire.ty_q16],zero_tenths:wire.zero_tenths}:undefined;
    const pose={...deviceMap,geometry:deviceMap?.geometry??geometryFromWire(frozenWire)};
    const previewWire=configurationHistory.get(preview?.config)?.wire,displayPose=config.displayMode==='preview'?{...preview,geometry:preview?.geometry??geometryFromWire(previewWire)}:pose;
    const displayRadar=config.displayMode==='preview'&&previewWire?{x:previewWire.radar_x,y:previewWire.radar_y}:grid.radar;
    const displayFieldPoints = displayed.map(point=>{const pair=config.displayMode==='stable'?stableFieldPoint(point):displayPose.geometry?projectFrozenRadarPoint(point,displayPose):null;return pair?{x:pair[0],y:pair[1],source:point}:toFieldPoint(point,displayRadar);});
    const missionPoints = resolveMissionPoints(grid, grid.radar);
    derived = {rawPoints, filteredPoints, grid, cells, plan, displayedPoints: displayed, missionPoints,
      deviceMap, devicePath, matchingCloud, preview,stable,frozenConfig:clone(drawingConfig)};
    root.dataset.theme = config.theme;
    refs['radar-direct-show-raw'].dataset.active = String(config.displayMode === 'raw'); refs['radar-direct-show-filtered'].dataset.active = String(config.displayMode === 'filtered');
    refs['radar-direct-show-stable'].dataset.active=String(config.displayMode==='stable');refs['radar-direct-show-preview'].dataset.active=String(config.displayMode==='preview');
    refs['radar-direct-theme-dark'].dataset.active = String(config.theme === 'dark'); refs['radar-direct-theme-light'].dataset.active = String(config.theme === 'light');
    const pointStart = drawingConfig.planningMode === 'point-to-point' ? resolvePlanningEndpoint({grid, endpoint: drawingConfig.start}) : null;
    const pointGoal = drawingConfig.planningMode === 'point-to-point' ? resolvePlanningEndpoint({grid, endpoint: drawingConfig.goal}) : null;
    const frozenZero=pose.geometry?.zero_tenths/10;
    mapRenderer.render({fieldSize: grid.fieldSize, activeMin: grid.activeMinimum, activeMax: grid.activeMaximum, xLines: grid.xLines, yLines: grid.yLines, radar: {...grid.radar,zero_deg:Number.isFinite(frozenZero)?frozenZero:90}, points: displayFieldPoints, cells, path: plan.success ? plan.waypoints.map(point => ({x: point.xmm, y: point.ymm})) : [], missionPoints: missionPoints.map(point => ({number: point.number, row: point.endpoint.row, column: point.endpoint.column, x: point.xmm, y: point.ymm})), start: pointStart, goal: pointGoal, theme: config.theme});
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
    const routeSummary = refs['radar-direct-route-summary']; routeSummary.replaceChildren(el('h3', {text: `路径规划 · ${plan.success ? '可通行' : plan.pending ? '待同步' : '无可用路径'}`}), el('p', {text: plan.success ? `${plan.waypoints.length} 个坐标点，总距离 ${plan.totalDistanceMm.toFixed(0)} mm` : plan.failureReason}), ...(drawingConfig.planningMode === 'mission' ? [el('strong', {text: `冻结任务顺序 ${formatMissionRoute(drawingConfig.missionRoute)}`})] : []));
    refs['radar-direct-waypoints'].replaceChildren(...plan.waypoints.map((point, index) => { const tr = el('tr'); for (const value of [index + 1, `R${point.row}C${point.column}`, point.xmm, point.ymm, point.nextDistanceMm || '—', directionText(point.direction), turnText(point.turn)]) tr.append(el('td', {text: String(value)})); return tr; }));
    refs['radar-direct-preview-title'].textContent = `${config.displayMode==='stable'?'15圈稳定XY':config.displayMode==='preview'?'独立实时preview120':config.displayMode==='raw'?'最后圈完整原始点':'最后圈本地显示筛选'} · 前 12 条`;
    const stableTable=config.displayMode==='stable',previewHeader=el('tr');for(const label of stableTable?['X mm','Y mm','支持圈']:['角度','距离','能量'])previewHeader.append(el('th',{text:label}));previewHead.replaceChildren(previewHeader);
    refs['radar-direct-preview'].replaceChildren(...displayed.slice(0, 12).map(point => { const tr = el('tr'); for (const value of stableTable?[point.x_mm,point.y_mm,point.votes]:[`${point.angle_deg.toFixed(2)}°`, point.distance_mm, point.energy ?? '—']) tr.append(el('td', {text: String(value)})); return tr; }));
    refs['radar-direct-csv'].disabled = !matchingCloud;
    refs['radar-direct-transfer-status'].textContent=transferProgress;
    refs['radar-direct-installation-state'].textContent=`草稿${draftDirty?'待应用':'已载入'} · 设备配置 ${deviceConfig?.config??'未读回'} · 冻结配置 ${deviceMap?.config??'无'}${deviceMap?.window_count!==undefined?` · ${deviceMap.window_count}/15圈 · 稳定 ${deviceMap.stable_points} 点`:''}`;
    const connected = connectionState.state === 'connected', compatible = connected && Number(connectionState.baudrate) === 115200;
    port.disabled = connectionState.state !== 'disconnected' || busy; baud.disabled = true; refreshPortsButton.disabled = connectionState.state !== 'disconnected' || busy;
    refs['radar-direct-connect'].disabled = connected || busy || !port.value; refs['radar-direct-disconnect'].disabled = !connected || busy;
    for (const id of ['radar-direct-start', 'radar-direct-stop', 'radar-direct-single', 'radar-direct-read-status']) refs[id].disabled = !compatible || busy;
    refs['radar-direct-stop'].disabled=!compatible;
    for(const id of ['radar-direct-fetch-stable','radar-direct-fetch-cloud'])refs[id].disabled=!compatible||busy||transferActive||!(status?.scan??deviceMap?.scan);
    for(const id of ['radar-direct-apply-filter','radar-direct-apply-map'])refs[id].disabled=!compatible||busy||!!status?.a&&!continuous;
    refs['radar-direct-single'].disabled = !compatible || busy || !!status?.a || !!hostClient.pendingStart;
    refs['radar-direct-continuous'].disabled = !compatible || busy; refs['radar-direct-continuous'].textContent = continuous ? '停止持续扫描' : '持续扫描';
    refs['radar-direct-connection-status'].textContent = busy ? '正在处理…' : continuous ? 'F407 持续建图 · 初满15圈后逐圈最近15圈' : compatible ? `共享串口已连接 ${connectionState.port || port.value} / 115200` : connected ? `共享串口当前为 ${connectionState.baudrate}，雷达页需断开后以 115200 重连` : '设备未连接 · 已保留最后完整设备结果';
    if(!busy&&compatible){
      if(hostClient.lastError)refs['radar-direct-connection-status'].textContent=`${hostClient.lastError} · 可点击“读取状态/结果”重试`;
      else if(status?.ss===6)refs['radar-direct-connection-status'].textContent=`F407 扫描失败：${status.reason || '未知原因'}`;
      else if(status?.a)refs['radar-direct-connection-status'].textContent=`F407 ${scanNames[status.ss] || '扫描中'} · ${status.window_count??status.wc??0}/15有效圈 · ${status.reason || ''}`;
      else if(status?.ss===4)refs['radar-direct-connection-status'].textContent=continuous ? '持续建图 · 最近15圈稳定XY、障碍格与路线同步' : '15圈采集完成 · 同步稳定XY、障碍格与路线；最后圈原始云按需读取';
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
    const stopping=['stop','live-stop'].includes(action);if (busy&&!stopping) return;
    if(!stopping)busy = true; render();
    try { await hostClient.command(action); log(`操作已写出：${action}`); }
    catch (error) { log(`操作失败：${error.message}`); refs['radar-direct-connection-status'].textContent = error.message; }
    finally { if(!stopping)busy = false; render(); }
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
      await hostClient.applyConfig(values); draftDirty=false;activeConfigId=null;syncActiveConfig(); render();
      const status=hostClient.model.status;
      const newWindow=Object.keys(values).some(k=>/^(?:radar_|angle_|distance_|energy_|zero_|m\d\d_|t[xy]_|situation|[xy][1-4])/.test(k));
      refs['radar-direct-settings-status'].textContent = newWindow?'配置已读回 · 请采集新的15圈窗口；持续模式保持旋转，旧图保留。':successText||'配置已读回 · 由冻结稳定窗口重新规划。';
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
  const originalTransfer=hostClient.onTransfer;
  hostClient.onTransfer=event=>{originalTransfer?.(event);transferActive=event.active===true;transferProgress=event.kind==='cancel'?'读取已取消 · 完整历史保留':`${event.kind} 下载 ${event.page??0}/${event.pages??'?'}${event.active?' · 进行中':event.error?` · ${event.error}`:' · 完成'}`;render();};
  const fetchSnapshot=async kind=>{const id=hostClient.model.status?.scan??hostClient.model.snapshots.map?.scan;if(!id)return;try{await hostClient.fetch(kind,id);}catch(error){transferProgress=error.message;}render();};
  refs['radar-direct-fetch-stable'].onclick=()=>fetchSnapshot('stable');refs['radar-direct-fetch-cloud'].onclick=()=>fetchSnapshot('cloud');
  refs['radar-direct-cancel-transfer'].onclick=()=>{hostClient.cancelTransfer();transferProgress='读取已取消 · 采集状态与完整历史保留';render();};
  refs['radar-direct-export'].onclick=()=>{const data={v:2,profile:'f1_host',coordinate_frame:radarSnapshotFrame(derived?.deviceMap??{profile:'f1_host',situation:readInstallationDraft().situation}),draft:{config,installation:readInstallationDraft()},snapshots:clone(hostClient.model.snapshots)};const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));const link=el('a');link.href=url;link.download=`radar-f1-${hostClient.model.snapshots.map?.scan??'history'}.json`;link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
  refs['radar-direct-import'].onchange=async()=>{const file=refs['radar-direct-import'].files?.[0];if(!file)return;try{const value=JSON.parse(await file.text());if(value.profile!=='f1_host'||value.snapshots?.map?.counts?.length!==25||!Array.isArray(value.snapshots?.path?.points))throw new Error('需要包含完整F1地图和路径的冻结快照');hostClient.cancelTransfer?.();hostClient.model.snapshots=Object.fromEntries(Object.entries(value.snapshots).map(([key,snapshot])=>[key,{...clone(snapshot),historical:true}]));frozenKey='';log('已载入历史冻结快照 · 未发送设备命令');render();}catch(error){log(`导入失败：${error.message}`);}};
  refs['radar-direct-show-stable'].onclick=()=>setLocalConfig({...config,displayMode:'stable'});
  refs['radar-direct-show-preview'].onclick=()=>setLocalConfig({...config,displayMode:'preview'});
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
      Object.assign(values,readInstallationDraft());delete values.lidar_x_mm;delete values.lidar_y_mm;delete values.zero_deg;delete values.grid_version;
      for (const axis of ['x', 'y']) for (let index = 1; index <= 4; index += 1) values[`${axis}${index}`] = Number(refs[`radar-direct-${axis}${index}`].value);
      return applyDeviceConfig(values, '场地、障碍格与路径已由 F407 从当前冻结圈重新计算。');
    } catch (error) { refs['radar-direct-settings-status'].textContent = error.message; }
  };
  planningMode.onchange = () => { draftDirty=true;refs['radar-direct-endpoints'].hidden = planningMode.value !== 'point-to-point'; missionRoute.parentElement.hidden = planningMode.value !== 'mission'; };
  for(const id of ['radar-direct-start-cell','radar-direct-goal-cell'])refs[id].onchange=()=>{draftDirty=true;};
  function readInstallationDraft(){const values={...installationDraft,zero_tenths:Math.round(Number(refs['radar-direct-zero'].value)*10)};for(const key of Object.keys(values).filter(k=>k.endsWith('_q16')))values[key]=Number(refs[`radar-direct-${key}`].value);for(const [key,value]of Object.entries(values))if(!Number.isFinite(value)||key.endsWith('_q16')&&!Number.isInteger(value))throw new Error(`${key} 需为有效整数`);return values;}
  refs['radar-direct-installation'].onchange=()=>{installationDraft=installationParameters(refs['radar-direct-installation'].value);draftDirty=true;refs['radar-direct-radar-x'].value=String(installationDraft.lidar_x_mm);refs['radar-direct-radar-y'].value=String(installationDraft.lidar_y_mm);refs['radar-direct-zero'].value=String(installationDraft.zero_tenths/10);for(const key of Object.keys(installationDraft).filter(k=>k.endsWith('_q16')))refs[`radar-direct-${key}`].value=String(installationDraft[key]);render();};
  for(const node of root.querySelectorAll('input'))node.oninput=()=>{draftDirty=true;};
  refs['radar-direct-save'].onclick = () => { try{const next=clone(config);next.radarPort=port.value;next.filter={minAngleDeg:Number(refs['radar-direct-min-angle'].value),maxAngleDeg:Number(refs['radar-direct-max-angle'].value),minDistanceMm:Number(refs['radar-direct-min-distance'].value),maxDistanceMm:Number(refs['radar-direct-max-distance'].value),minEnergy:Number(refs['radar-direct-min-energy'].value),maxEnergy:Number(refs['radar-direct-max-energy'].value)};next.map.radarXmm=Number(refs['radar-direct-radar-x'].value);next.map.radarYmm=Number(refs['radar-direct-radar-y'].value);next.map.obstaclePointThreshold=Number(refs['radar-direct-threshold'].value);for(const axis of ['x','y'])for(let i=1;i<5;i++)next.map[`${axis}Lines`][i]=Number(refs[`radar-direct-${axis}${i}`].value);next.planningMode=planningMode.value;const parsed=parseMissionRoute(refs['radar-direct-mission-route'].value);if(next.planningMode==='mission'&&!parsed.ok)throw new Error(parsed.error);if(parsed.ok)next.missionRoute=parsed.route;next.start=endpointFromValue(refs['radar-direct-start-cell'].value,config.start);next.goal=endpointFromValue(refs['radar-direct-goal-cell'].value,config.goal);const result=saveRadarDirectPreferences(storage,next);if(result.ok){config=normalizeRadarConfig(next);installationDraft=readInstallationDraft();storage?.setItem?.('local-car-radar-installation-v1',JSON.stringify(installationDraft));log('草稿和本机设置已保存；未向 F407 发送指令');}else log(`保存失败：${result.error}`);}catch(error){log(`保存失败：${error.message}`);} };
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
    dispose() { if (disposed) return; hostClient.onTransfer=originalTransfer;hostClient.cancelTransfer?.(); mapRenderer.dispose(); disposed = true; },
    snapshot() { const installation=readInstallationDraft();return {config: {...clone(config),coordinate_frame:radarSnapshotFrame({profile:'f1_host',situation:installation.situation}),installation},installation,deviceConfig:clone(deviceDraft),frozenConfig:clone(frozenConfig),connection: clone(connectionState), deviceMap: clone(derived?.deviceMap || null), devicePath: clone(derived?.devicePath || null),stable:clone(derived?.stable||null),scan: clone(derived?.matchingCloud || null), rawPoints: clone(derived?.rawPoints || []), filteredPoints: clone(derived?.filteredPoints || []), displayedPoints: clone(derived?.displayedPoints || []), obstacles: clone(derived?.cells || []), plan: clone(derived?.plan || null)}; },
  };
}
