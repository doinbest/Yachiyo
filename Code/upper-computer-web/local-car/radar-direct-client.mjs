const endpointFor = action => ({
  connect: 'connect',
  disconnect: 'disconnect',
  start: 'start',
  stop: 'stop',
  scan: 'scan',
  'continuous-start': 'continuous/start',
  'continuous-stop': 'continuous/stop',
})[action];

/** Read-only polling plus explicit LDS50C mutations. It never replays a command. */
export class RadarDirectClient {
  constructor({
    api,
    onState = () => {},
    onScan = () => {},
    onEvents = () => {},
    onError = () => {},
    pollDelayMs = 250,
  } = {}) {
    if (typeof api !== 'function') throw new TypeError('RadarDirectClient requires an api function');
    Object.assign(this, {api, onState, onScan, onEvents, onError, pollDelayMs});
    this.running = false;
    this.generation = 0;
    this.scanRevision = 0;
    this.statusScanRevision = 0;
    this.eventCursor = 0;
    this.controller = null;
    this.timer = null;
    this.state = null;
  }

  async start() {
    if (this.running) return;
    this.running = true;
    const generation = ++this.generation;
    await this.#cycle(generation);
    if (this.running && generation === this.generation) void this.#poll(generation);
  }

  suspend() {
    this.running = false;
    this.generation += 1;
    this.controller?.abort();
    this.controller = null;
    clearTimeout(this.timer);
    this.timer = null;
  }

  async refresh() {
    return this.#refresh(undefined);
  }

  async command(action, body = {}) {
    const endpoint = endpointFor(action);
    if (!endpoint) throw new RangeError(`Unknown radar command: ${action}`);
    const result = await this.api(`radar-direct/${endpoint}`, body);
    if (result && typeof result.connected === 'boolean') {
      this.state = result;
      this.statusScanRevision = Math.max(this.statusScanRevision, Number(result.scan_revision) || 0);
      this.onState(result);
    }
    try { await this.refresh(); } catch {}
    return result;
  }

  async #poll(generation) {
    while (this.running && generation === this.generation) {
      await new Promise(resolve => {
        this.timer = setTimeout(resolve, this.pollDelayMs);
        this.timer?.unref?.();
      });
      this.timer = null;
      if (!this.running || generation !== this.generation) return;
      await this.#cycle(generation);
    }
  }

  async #cycle(generation) {
    try {
      this.controller = new AbortController();
      await this.#refresh(this.controller.signal);
    } catch {} finally {
      if (this.running && generation === this.generation) this.controller = null;
    }
  }

  async #refresh(signal) {
    try {
      const status = await this.api('radar-direct/status', undefined, signal);
      this.state = status;
      this.statusScanRevision = Math.max(this.statusScanRevision, Number(status.scan_revision) || 0);
      this.onState(status);

      const requests = [];
      if (this.statusScanRevision > this.scanRevision) {
        const after = this.scanRevision;
        requests.push(this.api(`radar-direct/scan?after=${after}`, undefined, signal).then(scan => {
          const revision = Number(scan?.revision) || 0;
          if (scan?.points && revision >= this.statusScanRevision && revision > this.scanRevision) {
            this.scanRevision = revision;
            this.onScan(scan);
          }
        }));
      }
      requests.push(this.api(`radar-direct/events?after=${this.eventCursor}`, undefined, signal).then(batch => {
        if (batch?.gap) this.eventCursor = 0;
        const events = (batch?.events || []).filter(event => Number(event.id) > this.eventCursor);
        if (events.length) this.onEvents(events);
        this.eventCursor = Math.max(this.eventCursor, Number(batch?.last_event_id) || 0);
      }));
      await Promise.all(requests);
      return status;
    } catch (error) {
      if (error?.name !== 'AbortError') this.onError(error);
      throw error;
    }
  }
}
