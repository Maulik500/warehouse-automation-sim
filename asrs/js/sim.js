// Warehouse model: several aisles, each with one stacker crane, two racks (or one),
// an inbound and an outbound conveyor lane and their stations. Pallets carry individual boxes.
// Deterministic (seeded), no rendering dependencies — runs in Node and browser.
//
// Crane joints q are in the aisle-local frame (y = 0 on the aisle centreline);
// pallet and box positions are in world coordinates.

import { CRANE, FORK, LOAD, RACK, PD, LAYOUT, SKUS } from './config.js';
import { AxisMove } from './motion.js';
import { deflEmpty, deflLoad, deflection, hoistTargets, rackSlot, monitor } from './kinematics.js';

const AX = CRANE.axes;
const { abs, max, min } = Math;
const Z_CONV = PD.z + PD.deckLift; // pallet underside while riding on rollers

function rng32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// World position of a box centre (boxes never rotate: the crane has no rotary axis).
export const boxWorld = (p, b) => [p.x + b.lx, p.y + b.ly, p.z + b.lz];

// ====================================================================================
export class Aisle {
  constructor(wh, def) {
    this.wh = wh;
    this.id = def.id;
    this.y = def.y;
    this.rows = def.rows;
    this.rackSides = def.rows.map((r) => r[1]);

    this.slots = [];
    for (const [row, side] of def.rows)
      for (let b = 1; b <= RACK.bays; b++)
        for (let l = 1; l <= RACK.levels; l++) {
          const s = rackSlot(row, side, b, l);
          s.aisle = this; s.pallet = null; s.reserved = false; s.y = this.y + side * RACK.yStore;
          this.slots.push(s);
        }

    // stations: handling positions with the same support geometry as a rack location
    const st = (kind, side) => ({ kind, side, x: PD.x, zs: PD.z, y: this.y + side * RACK.yStore, id: `${kind.toUpperCase()}-${this.id}`, pallet: null, deck: kind === 'in' ? 1 : 0, moving: null });
    this.stIn = st('in', PD.inSide);
    this.stOut = st('out', PD.outSide);
    this.inLane = []; // pallets rolling towards the IN station, head first
    this.outLane = []; // pallets rolling from the OUT station to the handover port, head first
    this.requests = [];

    // crane: parked in front of the stations, fork centred, at pick height
    this.q = [PD.x, hoistTargets(PD.z, 0).low, 0];
    this.qd = [0, 0, 0];
    this.qdd = [0, 0, 0];
    this.moves = [null, null, null];
    this.carried = null; // pallet resting fully on the fork
    this.engaged = null; // { pallet, zs } while a transfer is in progress
    this.steps = [];
    this.stepIdx = 0;
    this.stepT0 = 0;
    this.op = null;
    this.phase = 'IDLE';
    this.nextKind = 'store';
    this.lastViolations = [];
    this.stats = { stored: 0, retrieved: 0, opTimes: [], dualTimes: [], lastDualStart: null, vPeak: [0, 0, 0], aPeak: [0, 0, 0], violations: 0, violationLog: [] };
  }

  get t() { return this.wh.t; }
  get fill() { return this.slots.filter((s) => s.pallet).length / this.slots.length; }
  get forkCentred() { return abs(this.q[2]) <= FORK.centredWin; }
  inReady() { return !!this.stIn.pallet && this.stIn.deck === 0; }
  outFree() { return !this.stOut.pallet && this.stOut.deck === 0; }
  outRoom() { return this.outLane.length + (this.stOut.pallet ? 1 : 0) < PD.laneCap; }
  monitorCtx() { return { rackSides: this.rackSides, deck: { 1: this.stIn.side > 0 ? this.stIn.deck : this.stOut.deck, '-1': this.stIn.side < 0 ? this.stIn.deck : this.stOut.deck } }; }

  place(p, s) {
    s.pallet = p; s.reserved = false;
    p.slot = s; p.x = s.x; p.y = s.y; p.z = s.zs;
  }

  // ---------- sequencing (spec §8) ----------
  pickSteps(loc, pallet) {
    const h = hoistTargets(loc.zs, pallet.mass);
    return [
      { k: 'move', x: loc.x, z: h.low, label: `TRAVEL → ${loc.id}` },
      { k: 'dwell', T: CRANE.tSettle, label: 'SETTLE' },
      { k: 'fork', to: loc.side * FORK.stroke, label: 'FORK EXTEND' },
      { k: 'do', fn: () => { this.engaged = { pallet, zs: loc.zs }; } },
      { k: 'creep', z: h.high, label: 'LIFT' },
      { k: 'do', fn: () => { this.carried = pallet; this.engaged = null; loc.pallet = null; pallet.slot = null; } },
      { k: 'fork', to: 0, label: 'FORK RETRACT' },
      { k: 'dwell', T: CRANE.tCentre, label: 'CENTRE CHECK' },
    ];
  }
  putSteps(loc, pallet) {
    const h = hoistTargets(loc.zs, pallet.mass);
    return [
      { k: 'move', x: loc.x, z: h.high, label: `TRAVEL → ${loc.id}` },
      { k: 'dwell', T: CRANE.tSettle, label: 'SETTLE' },
      { k: 'fork', to: loc.side * FORK.stroke, label: 'FORK EXTEND' },
      { k: 'do', fn: () => { this.engaged = { pallet, zs: loc.zs }; } },
      { k: 'creep', z: h.low, label: 'LOWER' },
      { k: 'do', fn: () => {
        this.carried = null; this.engaged = null;
        if (loc.kind === 'rack') this.place(pallet, loc);
        else { loc.pallet = pallet; pallet.x = loc.x; pallet.y = loc.y; pallet.z = loc.zs; }
      } },
      { k: 'fork', to: 0, label: 'FORK RETRACT' },
      { k: 'dwell', T: CRANE.tCentre, label: 'CENTRE CHECK' },
    ];
  }

  schedule() {
    const empty = this.slots.filter((s) => !s.pallet && !s.reserved);
    const full = this.slots.filter((s) => s.pallet && !s.reserved);
    this.requests = this.requests.filter((s) => s.pallet && !s.reserved);
    const wantStore = this.inReady() && empty.length > 0 && this.fill < 0.8;
    const wantRetr = full.length > 0 && this.outRoom() && (this.requests.length > 0 || this.fill > 0.3);

    let kind = null;
    if (this.requests.length && wantRetr) kind = 'retrieve';
    else if (this.nextKind === 'store' && wantStore) kind = 'store';
    else if (wantRetr && (this.nextKind === 'retrieve' || !wantStore)) kind = 'retrieve';
    else if (wantStore) kind = 'store';
    if (!kind) return false;

    const pick1 = (list) => list[Math.floor(this.wh.rand() * list.length)];
    if (kind === 'store') {
      const pallet = this.stIn.pallet, slot = pick1(empty);
      slot.reserved = true;
      this.op = { kind, slot, pallet, t0: this.t };
      this.steps = [...this.pickSteps(this.stIn, pallet), ...this.putSteps(slot, pallet), { k: 'do', fn: () => { this.stats.stored++; } }];
      this.nextKind = 'retrieve';
      this.stats.lastDualStart = this.t;
    } else {
      const slot = this.requests.shift() ?? pick1(full);
      const pallet = slot.pallet;
      slot.reserved = true;
      this.op = { kind, slot, pallet, t0: this.t };
      this.steps = [
        ...this.pickSteps(slot, pallet),
        { k: 'do', fn: () => { slot.reserved = false; } },
        { k: 'wait', until: () => this.outFree(), label: 'WAIT OUT FREE' },
        ...this.putSteps(this.stOut, pallet),
        { k: 'do', fn: () => {
          this.stats.retrieved++;
          if (this.stats.lastDualStart !== null) { this.stats.dualTimes.push(this.t - this.stats.lastDualStart); this.stats.lastDualStart = null; }
        } },
      ];
      this.nextKind = 'store';
    }
    this.stepIdx = 0;
    this.begin();
    return true;
  }

  begin() {
    const s = this.steps[this.stepIdx];
    this.stepT0 = this.t;
    if (s.k === 'move') {
      // interlock: travel and full-speed hoist are only released with the fork centred
      if (!this.forkCentred) throw new Error('interlock: move requested with fork extended');
      this.moves[0] = new AxisMove(this.q[0], s.x, AX[0], this.t);
      this.moves[1] = new AxisMove(this.q[1], s.z, AX[1], this.t);
    } else if (s.k === 'fork') {
      if (this.qd[0] !== 0 || this.qd[1] !== 0) throw new Error('interlock: fork requested while crane is moving');
      this.moves[2] = new AxisMove(this.q[2], s.to, this.carried ? { ...AX[2], ...CRANE.forkLoaded } : AX[2], this.t);
    } else if (s.k === 'creep') {
      this.moves[1] = new AxisMove(this.q[1], s.z, CRANE.hoistCreep, this.t);
    } else if (s.k === 'do') s.fn();
    if (s.label) this.phase = s.label;
  }

  stepDone() {
    const s = this.steps[this.stepIdx];
    if (s.k === 'do') return true;
    if (s.k === 'dwell') return this.t - this.stepT0 >= s.T - 1e-9;
    if (s.k === 'wait') return s.until();
    return this.moves.every((m) => m === null);
  }

  // ---------- time stepping ----------
  step(dt) {
    const t = this.t;
    for (let i = 0; i < 3; i++) {
      const m = this.moves[i];
      if (!m) { this.qd[i] = 0; this.qdd[i] = 0; continue; }
      const s = m.at(t);
      this.q[i] = s.p; this.qd[i] = s.v; this.qdd[i] = s.a;
      if (m.done(t)) { this.moves[i] = null; this.qd[i] = 0; this.qdd[i] = 0; }
      this.stats.vPeak[i] = max(this.stats.vPeak[i], abs(s.v));
      this.stats.aPeak[i] = max(this.stats.aPeak[i], abs(s.a));
    }

    while (this.op && this.stepDone()) {
      this.stepIdx++;
      if (this.stepIdx >= this.steps.length) {
        this.stats.opTimes.push(this.t - this.op.t0);
        this.op = null; this.steps = []; this.phase = 'IDLE';
      } else this.begin();
    }
    if (!this.op) this.schedule();

    this.conveyors(dt);
    this.loadPose();

    // independent safety monitor
    const bad = monitor(this.q, this.qd, this.carried, this.monitorCtx());
    if (bad.length) {
      this.stats.violations++;
      if (this.stats.violationLog.length < 20) this.stats.violationLog.push({ t: this.t, aisle: this.id, bad, q: [...this.q] });
    }
    this.lastViolations = bad;
  }

  // Lanes run along X. Inbound flows +X to the IN station, outbound flows -X to the port.
  conveyors(dt) {
    const d = PD.v * dt, dDeck = dt / PD.tDeck;
    const { stIn, stOut, inLane, outLane } = this;
    const moveDeck = (st, target) => { st.deck = target > st.deck ? min(target, st.deck + dDeck) : max(target, st.deck - dDeck); };
    const deckZ = (st) => max(st.zs, st.zs - PD.deckDrop + st.deck * (PD.deckDrop + PD.deckLift));

    // ---- inbound ----
    inLane.forEach((p, i) => { p.x = min(PD.x - (i + 1) * PD.pitch, p.x + d); });
    if (!stIn.pallet && !stIn.moving && stIn.deck === 1 && inLane.length && inLane[0].x >= PD.x - PD.pitch - 1e-9) stIn.moving = inLane.shift();
    if (stIn.moving) {
      const p = stIn.moving;
      p.x = min(PD.x, p.x + d);
      if (p.x >= PD.x) { stIn.pallet = p; stIn.moving = null; }
    }
    // deck drops under a delivered pallet and comes back up once the fork has left
    moveDeck(stIn, stIn.pallet ? 0 : this.forkCentred ? 1 : stIn.deck);
    if (stIn.pallet && this.engaged?.pallet !== stIn.pallet) stIn.pallet.z = deckZ(stIn);
    const last = inLane[inLane.length - 1];
    if (this.wh.inbound === 'auto' && inLane.length < PD.laneCap && (!last || last.x >= PD.xPort + PD.pitch)) this.wh.feedPallet(`IN-${this.id}`);

    // ---- outbound ----
    if (stOut.pallet && !stOut.moving && this.forkCentred && !this.engaged && outLane.length < PD.laneCap) stOut.moving = stOut.pallet;
    moveDeck(stOut, stOut.moving ? 1 : stOut.pallet ? stOut.deck : 0);
    if (stOut.pallet) stOut.pallet.z = deckZ(stOut);
    if (stOut.moving && stOut.deck === 1) {
      const p = stOut.moving;
      p.x -= d;
      if (p.x <= PD.x - PD.pitch) { outLane.push(p); stOut.pallet = null; stOut.moving = null; }
    }
    outLane.forEach((p, i) => { p.x = max(PD.xPort + i * PD.pitch, p.x - d); p.z = Z_CONV; });
  }

  // Share of the pallet weight currently on the fork (0 … 1), spec §6.3.
  loadFraction() {
    if (this.engaged) {
      const { pallet, zs } = this.engaged;
      const dm = deflLoad(this.q[2], pallet.mass);
      return min(1, max(0, (this.q[1] - deflEmpty(this.q[2]) - zs) / dm));
    }
    return this.carried ? 1 : 0;
  }
  // Actual height of the fork load plane at the tip, including sag.
  forkPlane() {
    const m = this.engaged?.pallet ?? this.carried;
    return this.q[1] - deflEmpty(this.q[2]) - (m ? this.loadFraction() * deflLoad(this.q[2], m.mass) : 0);
  }
  loadPose() {
    if (this.engaged) {
      const { pallet, zs } = this.engaged;
      pallet.z = max(zs, this.q[1] - deflection(this.q[2], pallet.mass));
    } else if (this.carried) {
      const p = this.carried;
      p.x = this.q[0]; p.y = this.y + this.q[2]; p.z = this.q[1] - deflection(this.q[2], p.mass);
    }
  }
}

// ====================================================================================
export class Warehouse {
  constructor({ seed = 7, fill = 0.55, consumer = 'auto', inbound = 'auto' } = {}) {
    this.rand = rng32(seed);
    this.t = 0;
    this.nextId = 1;
    this.pallets = new Set();
    this.consumer = consumer; // 'auto': built-in stand-in for the downstream cell; 'external': wait for takeBox()/releasePallet()
    this.inbound = inbound; // 'auto': pallets appear at the IN ports; 'external': only via feedPallet()
    this.listeners = {};
    this.counts = { boxesIn: 0, boxesOut: 0, palletsIn: 0, palletsOut: 0 };

    this.aisles = LAYOUT.aisles.map((def) => new Aisle(this, def));
    for (const a of this.aisles)
      for (const s of a.slots)
        if (this.rand() < fill) a.place(this.makePallet(), s);
    this.auto = this.aisles.map(() => ({ pallet: null, t: 0 })); // stand-in consumer timers
  }

  get slots() { return this.aisles.flatMap((a) => a.slots); }
  on(ev, fn) { (this.listeners[ev] ??= []).push(fn); return this; }
  emit(ev, data) { for (const fn of this.listeners[ev] ?? []) fn(data); }

  // ---------- unit loads ----------
  makePallet(skuIndex, layers) {
    const r = this.rand;
    const k = skuIndex ?? Math.floor(r() * SKUS.length), S = SKUS[k];
    const n = layers ?? 2 + Math.floor(r() * (S.maxLayers - 1));
    const id = this.nextId++;
    const p = { id, sku: S.sku, skuIndex: k, layers: n, boxes: [], x: 0, y: 0, z: 0, slot: null, h: LOAD.palletH + n * S.sz, mass: LOAD.palletMass + n * S.nx * S.ny * S.mass };
    let c = 0;
    for (let l = 0; l < n; l++)
      for (let j = 0; j < S.ny; j++)
        for (let i = 0; i < S.nx; i++)
          p.boxes.push({
            id: `${id}-${String(++c).padStart(3, '0')}`, sku: S.sku, layer: l + 1, mass: S.mass, sx: S.sx, sy: S.sy, sz: S.sz,
            lx: (i - (S.nx - 1) / 2) * S.sx, ly: (j - (S.ny - 1) / 2) * S.sy, lz: LOAD.palletH + (l + 0.5) * S.sz, // centre, relative to the pallet origin
          });
    this.pallets.add(p);
    this.counts.boxesIn += p.boxes.length;
    this.counts.palletsIn++;
    this.emit('palletCreated', p);
    return p;
  }

  // ---------- interface to the neighbouring cells (spec §13) ----------
  aisleOf(portId) {
    const a = this.aisles.find((x) => `IN-${x.id}` === portId || `OUT-${x.id}` === portId);
    if (!a) throw new Error(`unknown port ${portId}`);
    return a;
  }
  // Put a new pallet on an inbound port. Returns the pallet, or null if the port is occupied.
  feedPallet(portId, { skuIndex, layers } = {}) {
    const a = this.aisleOf(portId), last = a.inLane[a.inLane.length - 1];
    if (a.inLane.length >= PD.laneCap || (last && last.x < PD.xPort + PD.pitch - 1e-9)) return null;
    const p = this.makePallet(skuIndex, layers);
    p.x = PD.xPort; p.y = a.stIn.y; p.z = Z_CONV;
    a.inLane.push(p);
    return p;
  }
  // The pallet standing at an outbound port, or null while the port is empty.
  presented(portId) {
    const p = this.aisleOf(portId).outLane[0];
    return p && p.x <= PD.xPort + 1e-9 ? p : null;
  }
  pickable(p) {
    const top = max(0, ...p.boxes.map((b) => b.layer));
    return p.boxes.filter((b) => b.layer === top);
  }
  describeBox(p, b, top) {
    return { id: b.id, sku: b.sku, layer: b.layer, pickable: b.layer === top, mass: b.mass, size: [b.sx, b.sy, b.sz], pos: boxWorld(p, b).map((v) => +v.toFixed(4)) };
  }
  outputs() {
    return this.aisles.map((a) => {
      const p = this.presented(`OUT-${a.id}`);
      const top = p ? max(0, ...p.boxes.map((b) => b.layer)) : 0;
      return {
        port: `OUT-${a.id}`, aisle: a.id, pos: [PD.xPort, a.stOut.y, Z_CONV], queued: a.outLane.length - (p ? 1 : 0),
        pallet: p && { id: p.id, sku: p.sku, pos: [p.x, p.y, p.z], boxesLeft: p.boxes.length, boxes: p.boxes.map((b) => this.describeBox(p, b, top)) },
      };
    });
  }
  // Remove one box from the presented pallet. Only the top layer may be taken.
  // Without boxId the next top-layer box is taken. Returns the box description or null.
  takeBox(portId, boxId) {
    const p = this.presented(portId);
    if (!p || !p.boxes.length) return null;
    const cand = this.pickable(p);
    const b = boxId ? cand.find((x) => x.id === boxId) : cand[cand.length - 1];
    if (!b) return null;
    const out = this.describeBox(p, b, b.layer);
    p.boxes.splice(p.boxes.indexOf(b), 1);
    p.mass -= b.mass;
    this.counts.boxesOut++;
    this.emit('boxTaken', { port: portId, pallet: p, box: b, info: out });
    return out;
  }
  // Remove the presented pallet from the port (the empty pallet is taken away).
  releasePallet(portId, { force = false } = {}) {
    const a = this.aisleOf(portId), p = this.presented(portId);
    if (!p || (p.boxes.length && !force)) return false;
    a.outLane.shift();
    this.pallets.delete(p);
    this.counts.boxesOut += p.boxes.length;
    this.counts.palletsOut++;
    this.emit('palletReleased', { port: portId, pallet: p });
    return true;
  }
  // Ask for a stored pallet: by location id ('C-07-3'), by SKU, or any. Returns the location id or null.
  requestRetrieval({ slotId, sku } = {}) {
    const ok = (s) => s.pallet && !s.reserved && !s.aisle.requests.includes(s);
    const s = this.slots.find((x) => ok(x) && (slotId ? x.id === slotId : sku ? x.pallet.sku === sku : true));
    if (!s) return null;
    s.aisle.requests.push(s);
    return s.id;
  }
  snapshot() {
    return {
      t: +this.t.toFixed(3),
      cranes: this.aisles.map((a) => ({ aisle: a.id, phase: a.phase, q: a.q.map((v) => +v.toFixed(4)), command: a.op && { kind: a.op.kind, slot: a.op.slot.id, pallet: a.op.pallet.id } })),
      inventory: this.slots.filter((s) => s.pallet).map((s) => ({ slot: s.id, pallet: s.pallet.id, sku: s.pallet.sku, boxes: s.pallet.boxes.length, mass: s.pallet.mass })),
      outputs: this.outputs(),
      counts: { ...this.counts },
    };
  }

  // Stand-in for the downstream cell: empties each presented pallet top-down, then releases it.
  autoConsume(dt) {
    this.aisles.forEach((a, i) => {
      const port = `OUT-${a.id}`, st = this.auto[i], p = this.presented(port);
      if (p !== st.pallet) { st.pallet = p; st.t = 3.0; st.every = p ? min(0.3, 15 / max(1, p.boxes.length)) : 0; return; }
      if (!p) return;
      st.t -= dt;
      if (st.t > 0) return;
      if (p.boxes.length) { this.takeBox(port); st.t = p.boxes.length ? st.every : 2.0; }
      else this.releasePallet(port);
    });
  }

  step(dt) {
    this.t += dt;
    for (const a of this.aisles) a.step(dt);
    if (this.consumer === 'auto') this.autoConsume(dt);
  }

  get stats() {
    const sum = (f) => this.aisles.reduce((s, a) => s + f(a), 0);
    return { stored: sum((a) => a.stats.stored), retrieved: sum((a) => a.stats.retrieved), violations: sum((a) => a.stats.violations) };
  }
}
