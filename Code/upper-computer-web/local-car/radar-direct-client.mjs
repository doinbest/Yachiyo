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
    this.serviceInstance = null;
    this.requestSequence = 0;
    this.appliedStatusSequence = 0;
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
    const sequence = ++this.requestSequence;
    const result = await this.api(`radar-direct/${endpoint}`, body);
    if (result && typeof result.connected === 'boolean') this.#acceptStatus(result, sequence);
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
      const sequence = ++this.requestSequence;
      const status = await this.api('radar-direct/status', undefined, signal);
      const accepted = this.#acceptStatus(status, sequence);
      if (!accepted) return this.state;
      const instance = accepted.instance;

      const requests = [];
      if (this.statusScanRevision > this.scanRevision) {
        const after = this.scanRevision;
        requests.push(this.api(`radar-direct/scan?after=${after}`, undefined, signal).then(scan => {
          if (sequence < this.appliedStatusSequence || instance !== this.serviceInstance || !this.#matchesInstance(scan, instance)) return;
          const revision = Number(scan?.revision) || 0;
          if (scan?.points && revision >= this.statusScanRevision && revision > this.scanRevision) {
            this.scanRevision = revision;
            this.onScan(scan);
          }
        }));
      }
      requests.push(this.api(`radar-direct/events?after=${this.eventCursor}`, undefined, signal).then(batch => {
        if (sequence < this.appliedStatusSequence || instance !== this.serviceInstance || !this.#matchesInstance(batch, instance)) return;
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

  #acceptStatus(status, sequence) {
    if (sequence < this.appliedStatusSequence) return null;
    const scanRevision = Number(status?.scan_revision) || 0;
    const lastEventId = Number(status?.last_event_id) || 0;
    const instance = typeof status?.instance_id === 'string' && status.instance_id ? status.instance_id : null;
    const instanceChanged = this.serviceInstance !== null && instance !== null && instance !== this.serviceInstance;
    const countersRegressed = scanRevision < this.statusScanRevision || lastEventId < this.eventCursor;
    if (instanceChanged || countersRegressed) {
      this.scanRevision = 0;
      this.statusScanRevision = 0;
      this.eventCursor = 0;
    }
    if (instance !== null) this.serviceInstance = instance;
    this.appliedStatusSequence = sequence;
    this.state = status;
    this.statusScanRevision = Math.max(this.statusScanRevision, scanRevision);
    this.onState(status);
    return {instance: this.serviceInstance};
  }

  #matchesInstance(response, expected) {
    const actual = typeof response?.instance_id === 'string' && response.instance_id ? response.instance_id : null;
    return actual === null || expected === null || actual === expected;
  }
}
