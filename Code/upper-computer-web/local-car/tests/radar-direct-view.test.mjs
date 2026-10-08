import assert from 'node:assert/strict';
import test from 'node:test';

import {mountRadarDirectHost} from '../radar-direct-view.mjs';

class Element {
  constructor(tag, ownerDocument) {
    this.tagName = tag.toUpperCase();
    this.ownerDocument = ownerDocument;
    this.children = [];
    this.dataset = {};
    this.style = {};
    this.attributes = {};
    this.events = {};
    this.hidden = false;
    this.disabled = false;
    this.checked = false;
    this.value = '';
    this.className = '';
    this.textContent = '';
  }
  set id(value) { this._id = value; this.ownerDocument.nodes.set(value, this); }
  get id() { return this._id || ''; }
  append(...children) { for (const child of children) { if (child == null) continue; child.parentElement = this; this.children.push(child); } }
  appendChild(child) { this.append(child); return child; }
  replaceChildren(...children) { this.children = []; this.append(...children); }
  setAttribute(key, value) { this.attributes[key] = String(value); }
  addEventListener(type, listener) { this.events[type] = listener; }
  removeEventListener(type) { delete this.events[type]; }
  async click() { return this.onclick?.({target: this, preventDefault() {}}); }
  querySelectorAll(selector) {
    const descendants = this.children.flatMap(child => [child, ...child.querySelectorAll(selector)]);
    if (selector === 'input') return descendants.filter(child => child.tagName === 'INPUT');
    return descendants;
  }
}

function surface() {
  const document = {
    nodes: new Map(),
    createElement(tag) { return new Element(tag, document); },
    getElementById(id) { return document.nodes.get(id) || null; },
  };
  return {document, root: document.createElement('section'), get: id => document.getElementById(id)};
}

const configSnapshot = (overrides = {}) => ({
  k: 'host_config', source: 'active', session: 1, config: 1, scan: 2, map: 3, plan: 4,
  width: 2400, height: 2400, x1: 550, x2: 1000, x3: 1400, x4: 1850,
  y1: 550, y2: 1000, y3: 1400, y4: 1850, radar_x: 230, radar_y: 230,
  angle_min: 0, angle_max: 3600, distance_min: 100, distance_max: 40000,
  energy_min: 0, energy_max: 255, threshold: 1, fixed_mask: 0x00050140,
  manual_mask: 0, protected_mask: 0x00404405, planning_mode: 'mission',
  mission: [1, 5, 4, 2, 3, 4, 2, 3, 1],
  start: {row: 1, column: 1, anchor: 1}, goal: {row: 1, column: 5, anchor: 2},
  complete: true, historical: false, ...overrides,
});
const mapSnapshot = (overrides = {}) => ({
  k: 'host_map', session: 1, config: 1, scan: 2, map: 3, plan: 4,
  raw_points: 1616, filtered_points: 1581, outside_points: 0,
  scanned_mask: 0, manual_mask: 0, effective_mask: 0x00050140,
  path_valid: true, failed_from_task: 0, failed_to_task: 0,
  counts: Array(25).fill(0), complete: true, historical: false, ...overrides,
});
const pathSnapshot = (overrides = {}) => ({
  k: 'host_path', session: 1, config: 1, scan: 2, map: 3, plan: 4,
  path_valid: true, points: [
    [230, 230, 545, 1, 1, 1, 0],
    [775, 230, 0, 1, 2, 255, 255],
  ], complete: true, historical: false, ...overrides,
});

function harness(ui) {
  const commands = [], applications = [], maps = [], mapOptions = [], connectionCalls = [], received = [];
  const hostClient = {
    model: {snapshots: {}, status: null, connected: false, lastError: null},
    connection(value) { this.model.connected = value?.state === 'connected' || value?.connected === true || value === true; },
    receive(line) { received.push(line); return line.startsWith('@RADAR {"v":1,"profile":"f1_host"'); },
    async command(action) { commands.push(action); },
    async applyConfig(values) { applications.push(structuredClone(values)); return this.model.snapshots.config; },
    cancelTransfer() {},
  };
  const connectionController = {
    state: 'disconnected', port: '', baudrate: 115200,
    async listPorts() { connectionCalls.push(['list']); return [{device: 'COM7', description: 'USB UART'}]; },
    async connect(port, baudrate) { connectionCalls.push(['connect', port, baudrate]); },
    async disconnect() { connectionCalls.push(['disconnect']); },
  };
  const mapFactory = options => {
    mapOptions.push(options);
    return {render: model => maps.push(model), setTheme() {}, dispose() {}};
  };
  const values = new Map();
  const storage = {getItem: key => values.get(key) ?? null, setItem: (key, value) => values.set(key, value)};
  const view = mountRadarDirectHost({root: ui.root, storage, hostClient, connectionController, mapFactory});
  return {view, commands, applications, maps, mapOptions, hostClient, connectionController, connectionCalls, storage: values, received};
}

const flush = () => new Promise(setImmediate);

test('an unavailable device map is pending, not a zero-point scan',async()=>{
  const ui=surface(),h=harness(ui);await flush();
  h.view.connection({state:'connected',port:'COM7',baudrate:115200});
  h.hostClient.model.status={session:1,config:1,scan:2,map:3,plan:4,ss:4,a:0,l:1,ls:2,reason:'complete'};
  h.view.refresh();
  assert.equal(ui.get('radar-direct-metric-raw').textContent,'—');
  assert.equal(ui.get('radar-direct-metric-filtered').textContent,'—');
  assert.match(ui.get('radar-direct-point-summary').textContent,/地图.*读取/);
});

test('result-only scan labels cloud storage honestly and status reload stays button-driven',async()=>{
  const ui=surface(),h=harness(ui);await flush();
  h.view.connection({state:'connected',port:'COM7',baudrate:115200});
  h.hostClient.model.snapshots.map=mapSnapshot();h.hostClient.model.snapshots.path=pathSnapshot();
  h.hostClient.model.status={ss:4,a:0,l:0,ls:3,reason:'complete'};h.view.refresh();
  const values=ui.get('radar-direct-flags').children.map(row=>row.children.map(node=>node.textContent).join(':')).join('|');
  assert.match(values,/保留在 F407 · 不自动上传/);assert.doesNotMatch(values,/后台读取中/);
  assert.equal(h.commands.length,0);
  await ui.get('radar-direct-read-status').click();assert.deepEqual(h.commands,['status']);
  h.hostClient.lastError='ERR radar host scan_busy_or_uart';h.view.refresh();
  assert.match(ui.get('radar-direct-connection-status').textContent,/scan_busy_or_uart/);
});

test('a map awaiting its path is pending, while an actual blocked route remains unreachable',()=>{
  const ui=surface(),h=harness(ui);
  h.hostClient.model.snapshots.map=mapSnapshot();h.view.refresh();
  assert.match(ui.get('radar-direct-route-summary').children[0].textContent,/待同步/);
  assert.match(ui.get('radar-direct-route-summary').children[1].textContent,/路径.*读取/);
  h.hostClient.model.snapshots.map=mapSnapshot({path_valid:false,failed_from_task:1,failed_to_task:5});h.view.refresh();
  assert.match(ui.get('radar-direct-route-summary').children[1].textContent,/1.*5.*不可达/);
});

test('single-scan button waits for acquisition, remains available during result download and can replace live mode',()=>{
  const ui=surface(),h=harness(ui);
  h.view.connection({state:'connected',port:'COM7',baudrate:115200});
  h.hostClient.model.status={ss:3,a:1,l:0,ls:3,reason:'collecting'};h.view.refresh();
  assert.equal(ui.get('radar-direct-single').disabled,true);
  assert.equal(ui.get('radar-direct-stop').disabled,false);
  h.hostClient.model.status={ss:4,a:0,l:0,ls:3,reason:'complete'};h.view.refresh();
  assert.equal(ui.get('radar-direct-single').disabled,false);
  h.hostClient.model.status={ss:4,a:0,l:1,ls:2,reason:'complete'};h.view.refresh();
  assert.equal(ui.get('radar-direct-single').disabled,false);
});

test('continuous mapping reports full-circle counts without claiming a full cloud is stored',()=>{
  const ui=surface(),h=harness(ui);
  h.view.connection({state:'connected',port:'COM7',baudrate:115200});
  h.hostClient.model.snapshots.map=mapSnapshot({filtered_points:1331,outside_points:1330});
  h.hostClient.model.snapshots.path=pathSnapshot();
  h.hostClient.model.status={ss:4,a:0,l:1,ls:2,cloud:0,reason:'complete'};h.view.refresh();
  assert.match(ui.get('radar-direct-connection-status').textContent,/持续建图/);
  const values=ui.get('radar-direct-flags').children.map(row=>row.children.map(n=>n.textContent).join(':')).join('|');
  assert.match(values,/未缓存完整点云/);assert.doesNotMatch(values,/保留在 F407/);
  assert.match(ui.get('radar-direct-counters').children.at(-1).children[1].textContent,/1330.*150.*2250/);
  assert.equal(ui.get('radar-direct-metric-raw').textContent,'1616');
});

test('changing continuous filters reports next-circle application without starting another scan',async()=>{
  const ui=surface(),h=harness(ui);await flush();
  h.view.connection({state:'connected',port:'COM7',baudrate:115200});
  h.hostClient.model.snapshots.config=configSnapshot();
  h.hostClient.model.status={ss:4,a:0,l:1,ls:2,cloud:0,reason:'complete'};h.view.refresh();
  await ui.get('radar-direct-apply-filter').click();
  assert.equal(h.applications.length,1);
  assert.match(ui.get('radar-direct-settings-status').textContent,/下一完整圈/);
  assert.deepEqual(h.commands,[]);
});

test('mount uses the shared serial owner at fixed 115200 without automatic radar writes', async () => {
  const ui = surface(), h = harness(ui);
  await flush();

  assert.deepEqual(h.commands, []);
  assert.deepEqual(h.applications, []);
  assert.deepEqual(h.connectionCalls, [['list']]);
  assert.equal(ui.get('radar-direct-baud').value, '115200');
  assert.equal(ui.get('radar-direct-baud').disabled, true);
  assert.ok(ui.get('radar-direct-port').children.some(option => option.value === 'COM7'));
  assert.ok(ui.get('radar-direct-single'));
  assert.ok(ui.get('radar-direct-continuous'));
  assert.ok(ui.get('radar-direct-save'));
  assert.ok(ui.get('radar-direct-csv'));

  h.view.setVisible(false);
  h.view.setVisible(true);
  await ui.get('radar-direct-refresh-ports').click();
  await ui.get('radar-direct-theme-light').click();
  await ui.get('radar-direct-save').click();
  h.view.connection({state: 'connected', port: 'COM7', baudrate: 115200});
  h.view.connection({state: 'disconnected', port: '', baudrate: 115200});
  assert.deepEqual(h.commands, []);
  assert.deepEqual(h.applications, []);
  assert.equal(h.connectionCalls.filter(call => call[0] !== 'list').length, 0);
});

test('shared connect and explicit radar controls are the only hardware mutations', async () => {
  const ui = surface(), h = harness(ui);
  await flush();
  ui.get('radar-direct-port').value = 'COM7';

  await ui.get('radar-direct-connect').click();
  h.view.connection({state: 'connected', port: 'COM7', baudrate: 115200});
  await ui.get('radar-direct-start').click();
  await ui.get('radar-direct-stop').click();
  await ui.get('radar-direct-single').click();
  await ui.get('radar-direct-continuous').click();
  h.hostClient.model.status = {l: 1, ls: 2, scan: 2, map: 3, plan: 4}; h.view.refresh();
  await ui.get('radar-direct-continuous').click();
  await ui.get('radar-direct-disconnect').click();

  assert.deepEqual(h.connectionCalls.filter(call => call[0] !== 'list'), [
    ['connect', 'COM7', 115200], ['disconnect'],
  ]);
  assert.deepEqual(h.commands, ['start', 'stop', 'scan', 'live-start', 'live-stop']);
});

test('filter and map buttons stage device configuration without replacing device results locally', async () => {
  const ui = surface(), h = harness(ui);
  h.hostClient.model.snapshots = {config: configSnapshot(), map: mapSnapshot(), path: pathSnapshot()};
  h.view.refresh();
  const mapBefore = h.view.snapshot().deviceMap;

  ui.get('radar-direct-min-distance').value = '50';
  await ui.get('radar-direct-apply-filter').click();
  ui.get('radar-direct-threshold').value = '2';
  await ui.get('radar-direct-apply-map').click();

  assert.equal(h.applications[0].distance_min, 50);
  assert.equal(h.applications[0].angle_max, 3600);
  assert.equal(h.applications[1].threshold, 2);
  assert.deepEqual(h.view.snapshot().deviceMap, mapBefore);
  assert.equal(h.view.snapshot().config.map.obstaclePointThreshold, 1);
});

test('device map and path render before cloud; preview is replaced only by a matching full cloud', () => {
  const ui = surface(), h = harness(ui);
  h.hostClient.model.status = {session: 1, config: 1, scan: 2, map: 3, plan: 4, ss: 4, ls: 3, l: 0};
  h.hostClient.model.snapshots = {
    config: configSnapshot(), map: mapSnapshot({scanned_mask: 2, effective_mask: 0x00050142, counts: [0, 4, ...Array(23).fill(0)]}),
    path: pathSnapshot(),
    preview: {k: 'host_preview', session: 1, config: 1, scan: 2, map: 3, plan: 4, points: [{angle_deg: 10, distance_mm: 500}], complete: true},
    cloud: {k: 'host_cloud', session: 1, config: 1, scan: 1, map: 2, plan: 3, points: [{angle_deg: 20, distance_mm: 600, energy: 4}], complete: true},
  };
  h.view.refresh();

  assert.deepEqual(h.maps.at(-1).path, [{x: 230, y: 230}, {x: 775, y: 230}]);
  assert.equal(h.maps.at(-1).cells[1].flags.scanned, true);
  assert.equal(h.view.snapshot().displayedPoints[0].angle_deg, 10);
  assert.equal(ui.get('radar-direct-csv').disabled, true);

  h.hostClient.model.snapshots.cloud = {k: 'host_cloud', session: 1, config: 1, scan: 2, map: 3, plan: 4, points: [
    {angle_deg: 20, distance_mm: 600, energy: 4}, {angle_deg: 30, distance_mm: 700, energy: 5},
  ], complete: true};
  h.view.refresh();
  assert.equal(h.view.snapshot().displayedPoints.length, 2);
  assert.equal(ui.get('radar-direct-csv').disabled, false);
});

test('R1C1 stays protected and disconnect preserves completed device history', async () => {
  const ui = surface(), h = harness(ui);
  h.hostClient.model.snapshots = {config: configSnapshot(), map: mapSnapshot(), path: pathSnapshot()};
  h.view.refresh();
  assert.equal(h.maps.at(-1).cells[0].protected, true);

  await h.mapOptions[0].onCellClicked(1, 1);
  assert.equal(h.applications.length, 0);
  assert.match(ui.get('radar-direct-settings-status').textContent, /受保护/);

  h.view.connection({state: 'disconnected', port: '', baudrate: 115200});
  assert.equal(h.view.snapshot().deviceMap.map, 3);
  assert.equal(h.maps.at(-1).path.length, 2);
  assert.match(ui.get('radar-direct-connection-status').textContent, /保留最后完整/);
});

test('host frames are accepted before legacy routing and disposal sends no stop', () => {
  const ui = surface(), h = harness(ui);
  const host = '@RADAR {"v":1,"profile":"f1_host","k":"host_status"}';
  assert.equal(h.view.receive(host), true);
  assert.equal(h.view.receive('OK radar ready'), false);
  h.view.dispose();
  assert.deepEqual(h.commands, []);
  assert.equal(h.connectionCalls.filter(call => call[0] !== 'list').length, 0);
});
