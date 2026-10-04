import assert from 'node:assert/strict';
import test from 'node:test';

import {RadarDirectClient} from '../radar-direct-client.mjs';

const flush = () => new Promise(setImmediate);

test('start performs read-only refresh and publishes only a newer complete scan', async () => {
  const calls = [], states = [], scans = [], events = [];
  const api = async (path, body) => {
    calls.push({path, body});
    if (path === 'radar-direct/status') return {connected: false, scan_revision: 2, last_event_id: 4};
    if (path === 'radar-direct/scan?after=0') return {revision: 2, received_at: 'now', points: [{angle_deg: 0, distance_mm: 100, energy: 1}]};
    if (path === 'radar-direct/events?after=0') return {events: [{id: 4, kind: 'scan'}], last_event_id: 4, gap: false};
    throw new Error(`unexpected ${path}`);
  };
  const client = new RadarDirectClient({api, onState: value => states.push(value), onScan: value => scans.push(value), onEvents: value => events.push(...value), pollDelayMs: 60_000});

  await client.start();
  client.suspend();

  assert.deepEqual(calls.map(call => [call.path, call.body]), [
    ['radar-direct/status', undefined],
    ['radar-direct/scan?after=0', undefined],
    ['radar-direct/events?after=0', undefined],
  ]);
  assert.equal(states.at(-1).scan_revision, 2);
  assert.equal(scans.length, 1);
  assert.equal(scans[0].revision, 2);
  assert.deepEqual(events.map(event => event.id), [4]);
});

test('unchanged revisions do not redownload the scan', async () => {
  const calls = [];
  const api = async path => {
    calls.push(path);
    if (path === 'radar-direct/status') return {connected: true, scan_revision: 3, last_event_id: 0};
    if (path.startsWith('radar-direct/scan?')) return {revision: 3, points: [{angle_deg: 1, distance_mm: 2, energy: 3}]};
    if (path.startsWith('radar-direct/events?')) return {events: [], last_event_id: 0, gap: false};
    throw new Error(path);
  };
  const scans = [], client = new RadarDirectClient({api, onScan: scan => scans.push(scan), pollDelayMs: 60_000});

  await client.refresh();
  await client.refresh();

  assert.equal(scans.length, 1);
  assert.equal(calls.filter(path => path.startsWith('radar-direct/scan?')).length, 1);
});

test('a delayed older scan cannot replace a newer revision', async () => {
  let statusCall = 0, scanCall = 0, releaseOld;
  const old = new Promise(resolve => { releaseOld = resolve; });
  const scans = [];
  const api = async path => {
    if (path === 'radar-direct/status') return {connected: true, scan_revision: ++statusCall === 1 ? 1 : 2, last_event_id: 0};
    if (path === 'radar-direct/scan?after=0' && ++scanCall === 1) return old;
    if (path.startsWith('radar-direct/scan?')) return {revision: 2, points: [{angle_deg: 2, distance_mm: 2, energy: 2}]};
    if (path.startsWith('radar-direct/events?')) return {events: [], last_event_id: 0, gap: false};
    throw new Error(path);
  };
  const client = new RadarDirectClient({api, onScan: scan => scans.push(scan), pollDelayMs: 60_000});
  const first = client.refresh();
  await flush();
  // The second refresh is allowed to complete while the first scan request is pending.
  const second = client.refresh();
  await flush();
  releaseOld({revision: 1, points: [{angle_deg: 1, distance_mm: 1, energy: 1}]});
  await Promise.all([first, second]);

  assert.equal(scans.at(-1).revision, 2);
  assert.equal(client.scanRevision, 2);
});

test('commands are explicit POSTs and service errors remain retryable', async () => {
  const calls = [], errors = [];
  let fail = true;
  const api = async (path, body) => {
    calls.push({path, body});
    if (path === 'radar-direct/status' && fail) { fail = false; throw new Error('service unavailable'); }
    if (path === 'radar-direct/status') return {connected: false, scan_revision: 0, last_event_id: 0};
    if (path.startsWith('radar-direct/events?')) return {events: [], last_event_id: 0, gap: false};
    return {action: path.split('/').at(-1)};
  };
  const client = new RadarDirectClient({api, onError: error => errors.push(error.message), pollDelayMs: 60_000});

  await assert.rejects(client.refresh(), /unavailable/);
  assert.deepEqual(errors, ['service unavailable']);
  await client.command('connect', {port: 'COM7', baudrate: 500000});

  assert.deepEqual(calls.find(call => call.path === 'radar-direct/connect'), {
    path: 'radar-direct/connect', body: {port: 'COM7', baudrate: 500000},
  });
  assert.ok(calls.filter(call => call.path === 'radar-direct/status').length >= 2);
});

test('suspend aborts polling without sending a hardware command', async () => {
  const calls = [];
  let observedSignal;
  const api = async (path, body, signal) => {
    calls.push({path, body});
    observedSignal = signal;
    return new Promise((resolve, reject) => signal.addEventListener('abort', () => reject(Object.assign(new Error('aborted'), {name: 'AbortError'})), {once: true}));
  };
  const client = new RadarDirectClient({api, pollDelayMs: 60_000});

  const starting = client.start();
  await flush();
  client.suspend();
  await starting;

  assert.equal(observedSignal.aborted, true);
  assert.deepEqual(calls, [{path: 'radar-direct/status', body: undefined}]);
});
