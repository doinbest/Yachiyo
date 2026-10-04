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

function harness(ui) {
  const commands = [], maps = [], clientInstances = [];
  const clientFactory = hooks => {
    const client = {
      started: 0,
      suspended: 0,
      async start() { this.started += 1; },
      suspend() { this.suspended += 1; },
      async refresh() {},
      async command(action, body) { commands.push({action, body}); },
      emitState(value) { hooks.onState(value); },
      emitScan(value) { hooks.onScan(value); },
      emitEvents(value) { hooks.onEvents(value); },
      emitError(value) { hooks.onError(value); },
    };
    clientInstances.push(client);
    return client;
  };
  const mapFactory = () => ({render: model => maps.push(model), setTheme() {}, dispose() {}});
  const values = new Map();
  const storage = {getItem: key => values.get(key) ?? null, setItem: (key, value) => values.set(key, value)};
  const apiCalls = [];
  const api = async path => { apiCalls.push(path); return {ports: [{device: 'COM7', description: 'USB UART'}]}; };
  const view = mountRadarDirectHost({root: ui.root, api, storage, clientFactory, mapFactory});
  return {view, commands, maps, client: clientInstances[0], storage: values, apiCalls};
}

test('mount reproduces the direct-radar workflow without STM32 controls or automatic writes', async () => {
  const ui = surface(), h = harness(ui);
  await new Promise(setImmediate);

  assert.equal(h.client.started, 1);
  assert.deepEqual(h.commands, []);
  assert.deepEqual(h.apiCalls, ['radar-direct/ports']);
  assert.ok(ui.get('radar-direct-port'));
  assert.ok(ui.get('radar-direct-connect'));
  assert.ok(ui.get('radar-direct-single'));
  assert.ok(ui.get('radar-direct-continuous'));
  assert.ok(ui.get('radar-direct-save'));
  assert.ok(ui.get('radar-direct-csv'));
  assert.doesNotMatch(ui.root.textContent + ui.root.children.map(child => child.textContent).join(' '), /STM32|Flash|下载、应用/);
});

test('hardware mutations happen only from their explicit buttons', async () => {
  const ui = surface(), h = harness(ui);
  ui.get('radar-direct-port').value = 'COM7';
  ui.get('radar-direct-baud').value = '921600';

  await ui.get('radar-direct-connect').click();
  await ui.get('radar-direct-start').click();
  await ui.get('radar-direct-stop').click();
  await ui.get('radar-direct-single').click();
  await ui.get('radar-direct-continuous').click();
  h.client.emitState({connected: true, operation: 'continuous', continuous: true, scan_revision: 0, parser: {}});
  await ui.get('radar-direct-continuous').click();
  await ui.get('radar-direct-disconnect').click();

  assert.deepEqual(h.commands, [
    {action: 'connect', body: {port: 'COM7', baudrate: 921600}},
    {action: 'start', body: {}},
    {action: 'stop', body: {}},
    {action: 'scan', body: {}},
    {action: 'continuous-start', body: {}},
    {action: 'continuous-stop', body: {}},
    {action: 'disconnect', body: {}},
  ]);
});

test('one frozen scan can be refiltered, remapped and planned without another scan', async () => {
  const ui = surface(), h = harness(ui);
  h.client.emitScan({revision: 1, received_at: '2026-10-04T00:00:00Z', points: [
    {angle_deg: 0, distance_mm: 1000, energy: 100},
    {angle_deg: 180, distance_mm: 90, energy: 100},
  ]});
  assert.equal(h.view.snapshot().rawPoints.length, 2);
  assert.equal(h.view.snapshot().filteredPoints.length, 1);

  ui.get('radar-direct-min-distance').value = '50';
  await ui.get('radar-direct-apply-filter').click();

  assert.equal(h.view.snapshot().rawPoints.length, 2);
  assert.equal(h.view.snapshot().filteredPoints.length, 2);
  assert.equal(h.commands.length, 0);
  assert.ok(h.view.snapshot().plan);
  assert.ok(h.maps.length >= 2);
});

test('point-to-point map receives finite physical coordinates for anchored endpoints', async () => {
  const ui = surface(), h = harness(ui);
  ui.get('radar-direct-planning-mode').value = 'point-to-point';

  await ui.get('radar-direct-apply-map').click();

  const map = h.maps.at(-1);
  assert.deepEqual(map.start, {xmm: 230, ymm: 230, row: 1, column: 1});
  assert.deepEqual(map.goal, {xmm: 2250, ymm: 150, row: 1, column: 5});
  assert.ok([map.start.xmm, map.start.ymm, map.goal.xmm, map.goal.ymm].every(Number.isFinite));
});

test('settings are persisted only by explicit save and last scan survives disconnect state', async () => {
  const ui = surface(), h = harness(ui);
  ui.get('radar-direct-radar-x').value = '240';
  await ui.get('radar-direct-apply-map').click();
  assert.equal(h.storage.size, 0);

  await ui.get('radar-direct-save').click();
  assert.equal(JSON.parse([...h.storage.values()][0]).map.radarXmm, 240);

  h.client.emitScan({revision: 7, points: [{angle_deg: 0, distance_mm: 500, energy: 20}]});
  h.client.emitState({connected: false, operation: 'idle', scan_revision: 7, parser: {}});
  assert.equal(h.view.snapshot().rawPoints.length, 1);

  h.view.setVisible(false);
  assert.equal(ui.root.hidden, true);
  assert.equal(h.client.suspended, 0);
  h.view.dispose();
  assert.equal(h.client.suspended, 1);
});
