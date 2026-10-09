// Cell model: conveyor pick stations with their sensors, drop stations, and the AMR sequencer.
// Purely kinematic, fixed time step, no rendering dependencies.

import { AMR, MAST, FORK, PALLET, SKUS, CONV, CELL, DEG } from './config.js';
import { AxisMove } from '../../asrs/js/motion.js';
import { Path, line, turn, speedCap, steerOfPath, tangentLength } from './path.js';
import * as K from './kinematics.js';

const { abs, sin, cos, min, max, PI, sqrt } = Math;

// ---------- stations and routes (spec §10) ----------
export const T_TURN = tangentLength(PI / 2);

// Base poses of a station: docked (TCP on the pallet centre) and pre-dock (D_IN before it).
export function stationPoses(st) {
  const dock = K.baseFor(st.x, st.y, st.th);
  return { dock, pre: { x: dock.x - K.D_IN * cos(st.th), y: dock.y - K.D_IN * sin(st.th), th: st.th } };
}
export const dockPath = (st) => new Path(stationPoses(st).pre, [line(K.D_IN)]);
// Every route between a pick station (heading +X) and a side station (heading ±Y) is one "L":
// `leg` starts on the pick line, turns 90° and ends at the side station's pre-dock point;
// `link` joins the start of the leg to the pick pre-dock point along the pick line.
export function routePaths(pick, side) {
  const sg = Math.sign(sin(side.th)), pre = stationPoses(side).pre, start = { x: side.x - T_TURN, y: pick.y, th: 0 };
  const stem = sg * (pre.y - (pick.y + sg * T_TURN));
  if (stem < -1e-9) throw new Error(`no room for the turn between ${pick.id} and ${side.id}`);
  return {
    leg: new Path(start, [...turn((sg * PI) / 2), line(stem)]),
    link: new Path(start, [line(stationPoses(pick).pre.x - start.x)]),
  };
}

function rng(seed) {
  let s = seed;
  return () => ((s = (s * 16807) % 2147483647) / 2147483647);
}

// ---------- unit loads ----------
function makePallet(id, rand) {
  const p = { id, kind: 'empty', sku: null, boxes: [], mass: PALLET.mass, h: PALLET.h, cgz: PALLET.h / 2, x: 0, y: 0, z: 0, yaw: 0, pitch: 0, station: null, carried: false };
  if (rand() >= CONV.pEmpty) {
    const k = Math.floor(rand() * SKUS.length), S = SKUS[k], layers = 2 + Math.floor(rand() * (S.maxLayers - 1));
    p.kind = 'filled'; p.sku = S.sku; p.skuIndex = k;
    let mz = PALLET.mass * p.cgz;
    for (let l = 0; l < layers; l++)
      for (let i = 0; i < S.nx; i++)
        for (let j = 0; j < S.ny; j++) {
          const b = { sx: S.sx, sy: S.sy, sz: S.sz, lx: (i - (S.nx - 1) / 2) * S.sx, ly: (j - (S.ny - 1) / 2) * S.sy, lz: PALLET.h + (l + 0.5) * S.sz };
          p.boxes.push(b);
          mz += S.mass * b.lz;
        }
    p.mass += p.boxes.length * S.mass;
    p.h += layers * S.sz;
    p.cgz = mz / p.mass;
  }
  return p;
}

// ---------- conveyor line with its pick station (spec §6) ----------
class Line {
  constructor(cfg, cell) {
    Object.assign(this, { id: cfg.id, kind: 'pick', x: CONV.xP, y: cfg.y, th: 0, cell });
    this.pallets = []; // index 0 = nearest the pick position
    this.locked = false; // handshake: end position stopped and reserved for the AMR
    this.tPresent = 0;
    this.tSpawn = 0;
    this.xIn = CONV.xP + CONV.cap * CONV.pitch; // where new pallets enter
  }
  // sensors
  get b1() { return this.pallets.length > 0 && abs(this.pallets[0].x - CONV.xP) < 1e-9; } // pallet present at the end stop
  get b2() { return this.pallets.length > 0 && this.pallets[0].x - CONV.xP < CONV.b2x - CONV.b1x; } // slow-down
  get b3() { return this.b1 && this.pallets[0].h > CONV.b3z; } // load above the deck -> filled
  get ready() { return this.b1 && this.tPresent >= CONV.tDebounce; }
  get presented() { return this.b1 ? this.pallets[0] : null; }
  spawn() {
    const c = this.cell, p = makePallet(++c.nPallets, c.rand);
    Object.assign(p, { x: this.xIn, y: this.y + (2 * c.rand() - 1) * CONV.maxOffsetY, z: CONV.z, station: this.id });
    this.pallets.push(p);
    c.pallets.add(p);
    return p;
  }
  step(dt) {
    const c = this.cell;
    let stop = CONV.xP;
    for (const p of this.pallets) {
      // a locked end position is not refilled: whatever is not already there waits one pitch back
      if (this.locked && p.x > CONV.xP) stop = max(stop, CONV.xP + CONV.pitch);
      p.x = max(stop, p.x - CONV.v * dt);
      stop = p.x + CONV.pitch;
    }
    this.tPresent = this.b1 ? this.tPresent + dt : 0;
    if (c.t >= this.tSpawn && this.pallets.length < CONV.cap && stop <= this.xIn + 1e-9) {
      this.spawn();
      this.tSpawn = c.t + CONV.spawn[0] + (CONV.spawn[1] - CONV.spawn[0]) * c.rand();
    }
  }
}

// ---------- AMR ----------
class Amr {
  constructor(cell) {
    this.cell = cell;
    this.at = cell.home;
    this.pose = { ...stationPoses(cell.home).pre };
    this.odo = { ...this.pose }; // dead reckoning from the wheel commands, for comparison
    Object.assign(this, { v: 0, acc: 0, k: 0, phi: 0, vs: 0, w: 0, steerRate: 0, steerAcc: 0 });
    Object.assign(this, { tilt: MAST.travel.tilt, lift: MAST.travel.lift, shift: 0, mastRate: [0, 0, 0] });
    this.wheel = { drive: 0, left: 0, right: 0 }; // rolling angles
    this.engaged = null; // { pallet, zs, station, mode } while the tines are in a pallet
    this.carried = null;
    this.phase = 'idle';
    this.job = null;
    this.path = null;
    this.act = null;
    this.gen = this.cycle();
    this.stats = { picked: 0, delivered: 0, filled: 0, empty: 0, violations: 0, dist: 0, minClear: Infinity, cycle: [] };
    this.last = { bad: [], clearance: Infinity, zone: null };
  }
  get loaded() { return !!this.carried; }

  // --- actions: { start(t), update(t), done(t) } ---
  run(path, s0, s1, base, phase) {
    if (s1 < s0) base = { ...base, v: min(base.v, AMR.vRev) };
    const lim = speedCap(path, s0, s1, base);
    let move;
    return {
      start: (t) => { move = new AxisMove(s0, s1, lim, t); this.phase = phase; this.path = path; },
      update: (t) => {
        const m = move.at(t), p = path.at(m.p), st = steerOfPath(p);
        this.pose = { x: p.x, y: p.y, th: p.th };
        Object.assign(this, { s: m.p, v: m.v, acc: m.a, k: p.k, w: p.k * m.v, phi: st.phi, vs: m.v * sqrt(1 + AMR.L * AMR.L * p.k * p.k) });
        this.steerRate = st.dphi * m.v;
        this.steerAcc = st.ddphi * m.v * m.v + st.dphi * m.a;
      },
      done: (t) => move.done(t),
    };
  }
  mast(target, mode, phase) {
    const lims = { tilt: MAST.tilt, lift: { empty: MAST.lift, loaded: MAST.liftLoaded, creep: MAST.liftCreep }[mode], shift: MAST.shift };
    const axes = ['tilt', 'lift', 'shift'];
    let moves;
    return {
      start: (t) => { this.phase = phase; moves = axes.map((a) => (a in target ? new AxisMove(this[a], target[a], lims[a], t) : null)); },
      update: (t) => moves.forEach((m, i) => {
        if (!m) return;
        const s = m.at(t);
        this[axes[i]] = s.p;
        this.mastRate[i] = s.v;
      }),
      done: (t) => moves.every((m) => !m || m.done(t)),
    };
  }
  wait(cond, phase) {
    return { start: () => { this.phase = phase; }, update: () => {}, done: () => cond() };
  }

  *travel(to) {
    const from = this.at, base = this.loaded ? AMR.drive.loaded : AMR.drive.empty;
    const toPick = to.kind === 'pick', { leg, link } = this.cell.route(toPick ? to : from, toPick ? from : to);
    if (toPick) {
      yield this.run(leg, leg.length, 0, base, `reverse out of ${from.id}`);
      yield this.run(link, 0, link.length, base, `drive to ${to.id}`);
    } else {
      yield this.run(link, link.length, 0, base, `reverse out of ${from.id}`);
      yield this.run(leg, 0, leg.length, base, `drive to ${to.id}`);
    }
    this.at = to;
  }

  // One job = pick at a conveyor end, deliver to a floor slot (filled) or the stack (empty).
  *cycle() {
    const cell = this.cell, creep = AMR.drive.creep, T = MAST.travel;
    while (true) {
      yield this.wait(() => (this.job = cell.chooseJob()), 'idle: waiting for a pallet');
      const { line: ln, dest } = this.job, t0 = cell.t;
      if (this.at !== ln) yield* this.travel(ln);

      // ---- pick (spec §8.1) ----
      yield this.wait(() => ln.ready, `${ln.id}: wait for pallet present`);
      ln.locked = true;
      const p = ln.presented, pick = K.liftTargets(CONV.z), dockP = cell.dock(ln);
      yield this.mast({ tilt: 0, lift: pick.entry, shift: p.y - ln.y }, 'empty', `${ln.id}: set fork height and side shift`);
      yield this.run(dockP, 0, K.D_IN, creep, `${ln.id}: insert forks`);
      this.engaged = { pallet: p, zs: CONV.z, station: ln, mode: 'pick' };
      yield this.mast({ lift: pick.carry }, 'creep', `${ln.id}: lift pallet`);
      yield this.run(dockP, K.D_IN, 0, creep, `${ln.id}: withdraw`);
      ln.locked = false;
      yield this.mast({ tilt: T.tilt, lift: T.lift, shift: 0 }, 'loaded', 'lower to travel pose');
      this.stats.picked++;

      // ---- deliver (spec §8.1) ----
      yield* this.travel(dest);
      const zs = dest.support(), put = K.liftTargets(zs), dockD = cell.dock(dest);
      yield this.mast({ tilt: 0, lift: put.carry }, 'loaded', `${dest.id}: raise to deposit height`);
      yield this.run(dockD, 0, K.D_IN, creep, `${dest.id}: approach`);
      Object.assign(this.engaged, { zs, station: dest, mode: 'put' });
      yield this.mast({ lift: put.entry }, 'creep', `${dest.id}: set down`);
      yield this.run(dockD, K.D_IN, 0, creep, `${dest.id}: withdraw`);
      this.engaged = null;
      yield this.mast({ tilt: T.tilt, lift: T.lift }, 'empty', 'travel pose');
      this.stats.delivered++;
      this.stats[p.kind]++;
      this.stats.cycle.push(cell.t - t0);
      this.job = null;
    }
  }

  // The pallet follows the fork once the fork top reaches the deck underside (form fit).
  updateLoad() {
    const e = this.engaged;
    if (!e) return;
    const p = e.pallet, top = K.tcpWorld(this.pose, this.tilt, this.lift, this.shift)[2];
    if (e.mode === 'pick' && top - PALLET.pocketHi > e.zs + 1e-9) {
      e.station.pallets.shift();
      Object.assign(p, { station: null, carried: true });
      this.carried = p;
      e.mode = 'hold';
    } else if (e.mode === 'put' && top - PALLET.pocketHi <= e.zs) {
      Object.assign(p, { z: e.zs, pitch: 0, carried: false, station: e.station.id });
      this.carried = null;
      e.station.accept(p, this.cell.t);
      e.mode = 'rest';
    }
    if (e.mode === 'hold' || e.mode === 'put') Object.assign(p, K.carriedPallet(this.pose, this.tilt, this.lift, this.shift));
  }

  // Load mass and centre of gravity in {B}, for the stability figures.
  load() {
    const p = this.carried;
    if (!p) return null;
    const b = K.tcpBody(this.tilt, this.lift, this.shift);
    return { mass: p.mass, x: b[0], z: p.z + p.cgz };
  }
  state() {
    return { pose: this.pose, v: this.v, phi: this.phi, vs: this.vs, steerRate: this.steerRate, tilt: this.tilt, lift: this.lift, shift: this.shift, mastRate: this.mastRate, loaded: this.loaded };
  }

  step(dt) {
    const cell = this.cell, tw0 = K.bodyTwist(this.vs, this.phi);
    if (!this.act) {
      this.act = this.gen.next().value;
      this.act.start(cell.t);
    }
    this.act.update(cell.t + dt);
    if (this.act.done(cell.t + dt)) this.act = null;
    this.updateLoad();

    // wheels and dead reckoning: both come from the wheel commands only
    const tw = K.bodyTwist(this.vs, this.phi), v = (tw0.v + tw.v) / 2, w = (tw0.w + tw.w) / 2, ws = K.loadWheelSpeeds(v, w);
    this.odo = K.stepPose(this.odo, v, w, dt);
    this.wheel.drive += (this.vs / AMR.rDrive) * dt;
    this.wheel.left += (ws.left / AMR.rLoad) * dt;
    this.wheel.right += (ws.right / AMR.rLoad) * dt;
    this.stats.dist += abs(v) * dt;
  }
}

// ---------- the cell ----------
export class Cell {
  constructor({ seed = 7, prefill = 2 } = {}) {
    this.t = 0;
    this.rand = rng(seed);
    this.nPallets = 0;
    this.pallets = new Set();
    this.lines = CONV.lines.map((c) => new Line(c, this));
    this.slots = CELL.slots.map((s) => ({
      ...s, kind: 'slot', pallet: null, reserved: false, tIn: 0,
      support: () => s.z,
      accept(p, t) { this.pallet = p; this.tIn = t; },
    }));
    this.stack = {
      ...CELL.stack, kind: 'stack', pallets: [], reserved: false, tFull: 0,
      support() { return this.z + this.pallets.length * PALLET.h; },
      accept(p, t) { this.pallets.push(p); this.tFull = t; },
    };
    this.home = { ...CELL.home, kind: 'home' };
    this.sides = [...this.slots, this.stack, this.home];
    this.frames = this.lines.map((l) => {
      const x0 = CONV.xP - PALLET.sx / 2 - CONV.endOverhang, x1 = l.xIn + PALLET.sx / 2 + 0.3;
      return { id: l.id, x0, x1, y: l.y, top: CONV.z, poly: K.rectPoly(0, l.y, 0, x0, x1, CONV.frameW / 2) };
    });
    this._routes = new Map();
    this._docks = new Map();
    this.nextSlot = 0;
    for (const l of this.lines) {
      for (let i = 0; i < prefill; i++) l.spawn().x = CONV.xP + i * CONV.pitch;
      l.tSpawn = CONV.spawn[0] * this.rand();
    }
    this.amr = new Amr(this);
  }
  route(pick, side) {
    const key = pick.id + '>' + side.id;
    if (!this._routes.has(key)) this._routes.set(key, routePaths(pick, side));
    return this._routes.get(key);
  }
  dock(st) {
    if (!this._docks.has(st.id)) this._docks.set(st.id, dockPath(st));
    return this._docks.get(st.id);
  }

  // Longest-waiting presented pallet that has somewhere to go.
  chooseJob() {
    const cand = this.lines.filter((l) => l.ready).sort((a, b) => b.tPresent - a.tPresent);
    for (const ln of cand) {
      let dest = null;
      if (ln.b3) {
        for (let i = 0; i < this.slots.length && !dest; i++) {
          const s = this.slots[(this.nextSlot + i) % this.slots.length];
          if (!s.pallet && !s.reserved) { dest = s; this.nextSlot = (this.nextSlot + i + 1) % this.slots.length; }
        }
      } else if (!this.stack.reserved && this.stack.pallets.length < this.stack.max) dest = this.stack;
      if (dest) {
        dest.reserved = true;
        return { line: ln, dest, kind: ln.b3 ? 'filled' : 'empty' };
      }
    }
    return null;
  }

  // What the safety monitor is allowed to know about the surroundings.
  env() {
    const e = this.amr.engaged, resting = (st) => (e && e.station === st && e.mode === 'rest' ? e : null);
    const stations = [
      ...this.lines.map((l) => ({ id: l.id, kind: 'pick', x: l.x, y: l.y, th: l.th, zs: CONV.z, pallet: l.presented, locked: l.locked })),
      ...this.slots.map((s) => ({ id: s.id, kind: 'slot', x: s.x, y: s.y, th: s.th, zs: s.z, pallet: s.pallet })),
      { id: this.stack.id, kind: 'stack', x: this.stack.x, y: this.stack.y, th: this.stack.th, zs: resting(this.stack)?.zs ?? this.stack.support(), pallet: resting(this.stack)?.pallet ?? null },
    ];
    const pallets = [];
    for (const p of this.pallets) {
      if (p._x !== p.x || p._y !== p.y || p._yaw !== p.yaw) { p.poly = K.palletPoly(p); p._x = p.x; p._y = p.y; p._yaw = p.yaw; }
      pallets.push(p);
    }
    return { stations, frames: this.frames, pallets };
  }

  step(dt) {
    const a = this.amr;
    for (const l of this.lines) l.step(dt);
    a.step(dt);
    this.t += dt;

    // drop stations are emptied by the downstream process once the AMR has left
    for (const s of this.slots) {
      if (s.pallet && a.engaged?.pallet !== s.pallet) {
        s.reserved = true; // occupied
        if (this.t - s.tIn >= CELL.tSlotDwell) { this.pallets.delete(s.pallet); s.pallet = null; s.reserved = false; }
      }
    }
    const k = this.stack;
    if (!a.engaged || a.engaged.station !== k) {
      if (k.pallets.length && a.job?.dest !== k) k.reserved = false;
      if (k.pallets.length >= k.max && this.t - k.tFull >= CELL.tStackDwell) {
        for (const p of k.pallets) this.pallets.delete(p);
        k.pallets = [];
        k.reserved = false;
      }
    }

    a.last = K.monitor(a.state(), this.env());
    if (a.last.bad.length) a.stats.violations++;
    a.stats.minClear = min(a.stats.minClear, a.last.clearance);
  }
}
