// Palletizing cell for the integrated warehouse (spec §4.3, §8.3, §8.4): the verified 6R robot
// (js/kinematics.js, motion laws of js/sim.js) building mixed pallets — one SKU per layer — from
// boxes that arrive through the infeed line, on a pallet that stands on a two-position chain
// conveyor. Everything here is computed in the cell frame; box and pallet poses are written in {G}.

import { ROBOT as RB, TOOL, CELL } from '../../js/config.js';
import { fk, ik, toolDown, singularity } from '../../js/kinematics.js';
import { AxisMove } from '../../asrs/js/motion.js';
import { layerPattern } from '../../mobman/js/recipes.js';
import { PLACE, PCELL, PTOOL, PALLET, SKU, G, DEG, toG, toC } from './config.js';
import { placeBoxes } from './asrs.js';
import { overlap } from './traffic.js';

const { PI, abs, min, max, hypot, sqrt, cos, sin, atan2, acos } = Math;
const { conv } = CELL;
const quint = (u) => (u <= 0 ? 0 : u >= 1 ? 1 : u * u * u * (10 - 15 * u + 6 * u * u));
const KV = 1.875, KA = 5.7735;
export function ptpTime(qa, qb, k = 1) {
  let T = 0.05;
  for (let i = 0; i < 6; i++) {
    const d = abs(qb[i] - qa[i]);
    T = max(T, (KV * d) / (k * RB.vMax[i]), sqrt((KA * d) / (k * RB.aMax[i])));
  }
  return T;
}
const linTime = (d) => max(0.05, (KV * d) / RB.vLin, sqrt((KA * d) / RB.aLin));
const VAC = PTOOL.cups * PTOOL.vacuum * PI * (PTOOL.cupDia / 2) ** 2;
const PEEL = (PTOOL.cups / 2) * PTOOL.vacuum * PI * (PTOOL.cupDia / 2) ** 2 * 2 * PTOOL.rowY;
const Q1 = [PCELL.q1Min, PCELL.q1Max];
// plan outline of a box (centre, yaw of the long side)
const outline = (x, y, yaw, sx, sy) => { const c = cos(yaw), s = sin(yaw); return [[-sx / 2, -sy / 2], [sx / 2, -sy / 2], [sx / 2, sy / 2], [-sx / 2, sy / 2]].map(([u, v]) => [x + c * u - s * v, y + s * u + c * v]); };
const BELT_POLY = [[conv.x - PCELL.beltW / 2, conv.yStart], [conv.x + PCELL.beltW / 2, conv.yStart], [conv.x + PCELL.beltW / 2, conv.yEnd], [conv.x - PCELL.beltW / 2, conv.yEnd]];
// TCP height of the place hover point: h + hoverExtra above the final box top, and high enough
// for the carried box to leave the belt end with clearance.
const hoverZ = (slot, h) => max(slot.zTop + h + PCELL.hoverExtra, conv.top + h + PCELL.beltClear);
const extent = (b, yaw) => [abs(cos(yaw)) * b.sx + abs(sin(yaw)) * b.sy, abs(sin(yaw)) * b.sx + abs(cos(yaw)) * b.sy];

// Slots of a mixed pallet in the cell frame: box top-face centre, in placing order. Per layer:
// far row first, far column first, so nothing already placed is reached over.
export function orderSlots(layers) {
  const out = [];
  let z = PALLET.h;
  layers.forEach((id, k) => {
    const S = SKU[id];
    const lay = layerPattern(S.rc, 0).map((b) => ({ layer: k, sku: id, x: PCELL.build.x - b.y, y: PCELL.build.y + b.x, zTop: z + S.h, h: S.h, yaw: b.yaw + PI / 2 }));
    lay.sort((a, b) => b.y - a.y || a.x - b.x);
    out.push(...lay);
    z += S.h;
  });
  return out;
}

// ---------- box infeed line: receiving conveyor → row stop → incline → right-angle transfer → meter belt ----------
class Infeed {
  constructor(cell) { Object.assign(this, { cell, rows: [], flight: null, queue: [], lastRelease: -1e9, reserved: null, received: 0 }); }
  get boxes() { return [...this.rows.flatMap((r) => r.boxes), ...(this.flight ? this.flight.boxes : []), ...this.queue]; }
  // B5: enough free conveyor behind the receiving end for one layer
  get destReady() { return !this.rows.some((r) => r.x + r.depth / 2 > PCELL.recv.x1 - PCELL.recv.clearLen); }
  get clearOfDock() { return !this.rows.some((r) => r.x + r.depth / 2 > PCELL.recv.x1); }
  receive(b) {
    const P = this.cell.P, [x, y] = toC(P, b.p[0], b.p[1]), yaw = b.yaw - P.psi;
    b.st = 'infeed';
    b.c = { x, y, z: PCELL.recv.z + b.h / 2, yaw };
    let row = this.rows.find((r) => abs(r.x - x) < 0.03 && !r.closed);
    if (!row) { row = { x, depth: extent(b, yaw)[0], boxes: [] }; this.rows.push(row); this.rows.sort((a, c) => a.x - c.x); }
    row.boxes.push(b);
    this.received++;
  }
  step(t, dt) {
    const R = PCELL.recv, I = PCELL.incline, M = PCELL.merge, cell = this.cell;
    // receiving conveyor: rows close up behind the row stop
    let limit = null;
    for (const r of this.rows) {
      const lo = limit === null ? R.x0 + r.depth / 2 : limit + r.depth / 2;
      const nx = max(lo, r.x - R.v * dt);
      for (const b of r.boxes) b.c.x += nx - r.x;
      r.x = nx; limit = r.x + r.depth / 2;
    }
    // the row stop lets one row go when the transfer is free
    const front = this.rows[0];
    if (front && !this.flight && front.x <= R.x0 + front.depth / 2 + 1e-9 && cell.wantsBoxes) { front.closed = true; this.flight = this.rows.shift(); }
    // incline and right-angle transfer
    const f = this.flight;
    if (f) {
      const stop = this.queue.length ? M.x + 0.75 : M.x, nx = max(stop, f.x - R.v * dt);
      for (const b of f.boxes) { b.c.x += nx - f.x; b.c.z = (b.c.x >= I.x1 ? I.z1 : b.c.x <= I.x0 ? I.z0 : I.z0 + ((I.z1 - I.z0) * (b.c.x - I.x0)) / (I.x1 - I.x0)) + b.h / 2; }
      f.x = nx;
      if (f.x <= M.x + 1e-9) { this.queue.push(...f.boxes.sort((a, b) => b.c.y - a.c.y)); this.flight = null; }
    }
    // meter belt: single file, one box at a time onto the tracking belt
    let lim = PCELL.meter.y;
    for (const b of this.queue) {
      const len = extent(b, b.c.yaw)[1];
      b.c.y = min(lim - len / 2, b.c.y + PCELL.meter.v * dt);
      lim = b.c.y - len / 2;
    }
    const q = this.queue[0];
    if (q && q.c.y >= PCELL.meter.y - extent(q, q.c.yaw)[1] / 2 - 1e-9 && t - this.lastRelease >= PCELL.meter.every && cell.gateOpen) {
      this.queue.shift(); this.lastRelease = t; cell.beltAdd(q);
    }
    for (const b of this.boxes) { b.p = toG(cell.P, b.c.x, b.c.y, b.c.z); b.yaw = b.c.yaw + cell.P.psi; }
  }
}

// ---------- the cell ----------
export class PalCell {
  constructor(i, world) {
    this.i = i; this.world = world; this.P = PLACE.pal[i]; this.id = `PAL${i + 1}`;
    this.t = 0;
    this.infeed = new Infeed(this);
    this.belt = []; // boxes on the tracking belt
    this.order = null;
    this.vac = false; this.held = null;
    this.qReady = this.ikDown([conv.x, CELL.yReady, conv.top + 0.25 + CELL.hoverPick], PI / 2, [0, 0, 0, 0, PI / 2, 0]);
    this.q = [...this.qReady]; this.qd = [0, 0, 0, 0, 0, 0]; this.qdd = [0, 0, 0, 0, 0, 0];
    this.state = 'IDLE'; this.seg = null;
    this.tcpHist = [];
    // pallet conveyor and position D
    this.pal = null; this.palState = 'NONE'; this.palX = PCELL.dock.x; this.palMove = null; this.palT = 0; this.locked = null; this.reserved = false;
    const [dx, dy] = toG(this.P, PCELL.dock.x, PCELL.dock.y);
    this.station = this.makeStation(dx, dy);
    this.stats = {
      boxes: 0, pallets: 0, missed: 0, ikFail: 0, violations: 0, firstBad: [], jv: [0, 0, 0, 0, 0, 0], ja: [0, 0, 0, 0, 0, 0],
      qLo: Array(6).fill(Infinity), qHi: Array(6).fill(-Infinity), minWrist: Infinity, minElbow: Infinity, minShoulder: Infinity,
      minSF: Infinity, minPeel: Infinity, maxTilt: 0, minCarry: Infinity, placeErr: 0, cycle: [],
    };
    this.last = { bad: [], sf: Infinity, peel: Infinity, sing: singularity(this.q), tcp: fk(this.q).tcp, carry: Infinity };
  }
  // Position D as a forklift station (pick the mixed pallet / deposit an empty one).
  makeStation(x, y) {
    const cell = this, P = this.P, xa = PCELL.dock.x - PALLET.sy / 2 - 0.03, xb = 0.6, yc = PCELL.dock.y;
    return {
      id: `D${this.i + 1}`, kind: 'pick', x, y, th: P.psi, cell,
      frames: [{ poly: [[xa, yc - 0.7], [xb, yc - 0.7], [xb, yc + 0.7], [xa, yc + 0.7]].map(([cx, cy]) => toG(P, cx, cy).slice(0, 2)), top: P.tz }],
      zs: () => P.tz, support: () => P.tz,
      get pickReady() { return cell.palState === 'D_FULL' && cell.palT >= 0.5; },
      get dropReady() { return cell.palState === 'NONE' && !cell.palMove; },
      get locked() { return cell.locked; },
      lock(who) { if (cell.locked && cell.locked !== who) return false; cell.locked = who; return true; },
      unlock(who) { if (cell.locked === who) cell.locked = null; },
      presented() { return cell.palState === 'D_FULL' || cell.palState === 'D_EMPTY' ? cell.pal : null; },
      take(p) { if (cell.pal === p) { cell.pal = null; cell.palState = 'NONE'; } },
      accept(p) { cell.pal = p; p.holder = cell.id; cell.palState = 'D_EMPTY'; cell.palT = 0; },
    };
  }
  get ready() { return this.palState === 'B_READY' && !this.order; } // an empty pallet stands at B
  get needsPallet() { return this.palState === 'NONE' && !this.reserved; }
  get wantsBoxes() { return !!this.order && this.palState === 'BUILD'; }
  get gateOpen() { return this.wantsBoxes && this.belt.filter((b) => b.st === 'belt').length <= 1; }
  get robotAtReady() { return this.state === 'IDLE' && this.q.every((v, i) => abs(v - this.qReady[i]) < 1e-6); }
  startOrder(order) {
    this.order = { ...order, slots: orderSlots(order.layers), placed: 0, t0: this.t };
    this.palState = 'BUILD';
    this.pal.order = order.id;
  }
  beltAdd(b) { b.st = 'belt'; b.c.x = conv.x; b.c.z = conv.top + b.h / 2; this.belt.push(b); }

  ikDown(p, psi, ref) {
    const a = ik(p, toolDown(psi), ref), b = ik(p, toolDown(psi + PI), ref);
    const ok = (q) => q && q[0] >= Q1[0] && q[0] <= Q1[1];
    if (!ok(a) || !ok(b)) return ok(a) ? a : ok(b) ? b : null;
    return abs(a[5] - ref[5]) <= abs(b[5] - ref[5]) ? a : b;
  }
  setQ(q, dt) {
    for (let i = 0; i < 6; i++) { const v = (q[i] - this.q[i]) / dt; this.qdd[i] = (v - this.qd[i]) / dt; this.qd[i] = v; }
    this.q = q;
  }
  moveTo(p, psi, dt) {
    const q = this.ikDown(p, psi, this.q);
    if (!q) { this.stats.ikFail++; throw new Error(`${this.id}: IK failed in ${this.state} at [${p.map((v) => v.toFixed(3))}] t=${this.t.toFixed(2)}`); }
    this.setQ(q, dt);
  }
  toolYaw() { const R = fk(this.q).R; return atan2(R[3], R[0]); }

  // ---------- pallet conveyor ----------
  stepPallet(dt) {
    const p = this.pal;
    this.palT += dt;
    if (this.palMove) {
      const m = this.palMove.at(this.t);
      this.palX = m.p;
      if (this.palMove.done(this.t)) { this.palX = this.palMove.to; this.palMove = null; this.palState = this.palState === 'TO_B' ? 'B_READY' : 'D_FULL'; this.palT = 0; }
    } else if (this.palState === 'D_EMPTY' && !this.locked && this.robotAtReady) {
      this.palMove = new AxisMove(PCELL.dock.x, PCELL.build.x, PCELL.convV, this.t - dt); this.palState = 'TO_B';
    } else if (this.palState === 'BUILD' && this.order.placed === this.order.slots.length && this.robotAtReady && !this.held) {
      this.finishPallet();
      this.palMove = new AxisMove(PCELL.build.x, PCELL.dock.x, PCELL.convV, this.t - dt); this.palState = 'TO_D';
    }
    if (p && p === this.pal) {
      const g = toG(this.P, this.palX, PCELL.build.y);
      p.x = g[0]; p.y = g[1]; p.z = this.P.tz; p.dom = 'g';
      placeBoxes(p);
    }
  }
  // The built stack becomes the load of the pallet: box poses are stored in the pallet frame.
  finishPallet() {
    const p = this.pal, o = this.order, c = cos(-p.yaw), s = sin(-p.yaw);
    const boxes = o.slots.map((sl) => sl.box);
    let mz = PALLET.mass * PALLET.h / 2, m = PALLET.mass;
    for (const b of boxes) {
      const dx = b.p[0] - p.x, dy = b.p[1] - p.y;
      b.lp = [c * dx - s * dy, s * dx + c * dy, b.p[2] - p.z]; b.lyaw = b.yaw - p.yaw; b.st = 'pallet'; b.pal = p;
      m += b.m; mz += b.m * b.lp[2];
    }
    Object.assign(p, { kind: 'mixed', boxes, layers: o.layers.length, layerSkus: [...o.layers], mass: m, cgz: mz / m, h: max(...o.slots.map((sl) => sl.zTop)), order: o.id, built: this.t, cellId: this.id });
    this.stats.pallets++;
    this.world.emit('palletBuilt', { cell: this.i, pallet: p, order: o });
    this.order = null;
  }

  // ---------- robot (motion cycle of ROBOT_SPEC §10.1, per-box heights) ----------
  planIntercept(b) {
    const lead = (conv.v * CELL.tSync) / 2, zH = conv.top + b.h + CELL.hoverPick;
    let T = 1, qh = null, yI = 0;
    for (let k = 0; k < 4; k++) {
      yI = b.c.y + conv.v * T + lead;
      qh = this.ikDown([conv.x, yI, zH], b.c.yaw, this.q);
      if (!qh) return null;
      T = ptpTime(this.q, qh);
    }
    return { T, qh, yI };
  }
  startPtp(qb, next, k = 1) { this.seg = { t: 0, T: ptpTime(this.q, qb, k), qa: [...this.q], qb, next }; this.state = next.label; }
  stepRobot(dt) {
    const s = this.seg, o = this.order;
    switch (this.state) {
      case 'IDLE': {
        this.setQ(this.q, dt);
        const b = o && o.placed < o.slots.length ? this.belt.find((x) => x.st === 'belt') : null;
        if (b) {
          const slot = o.slots[o.placed], zGrip = conv.top + b.h, zH = hoverZ(slot, b.h), zS = max(zGrip + CELL.liftPick, zH);
          const plan = this.planIntercept(b);
          const travel = conv.v * (2 * CELL.tSync + linTime(CELL.hoverPick) + TOOL.tGrip + linTime(zS - zGrip));
          if (plan && plan.yI >= conv.yA && plan.yI + travel <= conv.yB) {
            this.seg = { t: 0, T: plan.T, qa: [...this.q], qb: plan.qh, next: { label: 'PTP_PICK', box: b, zS, zGrip, zH, slot } };
            this.state = 'PTP_PICK';
            break;
          } else if (plan && plan.yI + travel > conv.yB) { b.st = 'missed'; this.stats.missed++; }
        }
        if (!this.robotAtReady && this.q.some((v, i) => abs(v - this.qReady[i]) > 1e-6)) this.startPtp(this.qReady, { label: 'PTP_READY' });
        break;
      }
      case 'PTP_READY': case 'PTP_PICK': case 'PTP_PLACE': {
        s.t += dt;
        const done = s.t >= s.T - 1e-9, u = done ? 1 : quint(s.t / s.T);
        this.setQ(done ? [...s.qb] : s.qa.map((a, i) => a + (s.qb[i] - a) * u), dt);
        if (done) this.endPtp(s.next);
        break;
      }
      case 'TRACK': this.stepTrack(dt); break;
      case 'PLACE_DOWN': case 'PLACE_UP': {
        s.t += dt;
        const u = quint(s.t / s.T);
        this.moveTo([s.slot.x, s.slot.y, s.z0 + (s.z1 - s.z0) * u], s.psi, dt);
        if (s.t >= s.T) {
          if (this.state === 'PLACE_DOWN') { this.release(s.slot); this.state = 'RELEASE'; this.seg = { ...s, t: 0 }; }
          else this.state = 'IDLE';
        }
        break;
      }
      case 'RELEASE':
        this.setQ(this.q, dt);
        s.t += dt;
        if (s.t >= TOOL.tRelease) { this.seg = { ...s, t: 0, z0: s.z1, z1: s.z0 }; this.state = 'PLACE_UP'; }
        break;
    }
  }
  endPtp(next) {
    if (next.label === 'PTP_PICK') {
      const f = fk(this.q), Td = linTime(CELL.hoverPick), Tl = linTime(next.zS - next.zGrip), ts = CELL.tSync, b = next.box;
      this.seg = {
        t: 0, ...next, yv: b.c.y, off0: f.tcp[1] - b.c.y, psi: atan2(f.R[3], f.R[0]),
        tB: ts, tC: ts + Td, tD: ts + Td + TOOL.tGrip, tE: ts + Td + TOOL.tGrip + Tl, tEnd: 2 * ts + Td + TOOL.tGrip + Tl, Td, Tl,
      };
      this.state = 'TRACK';
    } else if (next.label === 'PTP_PLACE') {
      this.seg = { t: 0, T: linTime(next.zH - next.slot.zTop), slot: next.slot, z0: next.zH, z1: next.slot.zTop, psi: next.psi };
      this.state = 'PLACE_DOWN';
    } else this.state = 'IDLE';
  }
  // Conveyor tracking: all motion is expressed relative to a point yv riding the belt.
  stepTrack(dt) {
    const s = this.seg, ts = CELL.tSync, v = conv.v, zHov = s.zGrip + CELL.hoverPick;
    s.t += dt;
    let y, z;
    if (s.t < s.tE) s.yv += v * dt;
    else if (!s.frozen) { s.yv += v * (dt - (s.t - s.tE)); s.frozen = true; }
    if (s.t < s.tB) { const k = 1 - s.t / ts; y = s.yv + s.off0 * k * k; z = zHov; }
    else if (s.t < s.tC) { y = s.yv; z = zHov + (s.zGrip - zHov) * quint((s.t - s.tB) / s.Td); }
    else if (s.t < s.tD) { y = s.yv; z = s.zGrip; }
    else if (s.t < s.tE) { y = s.yv; z = s.zGrip + (s.zS - s.zGrip) * quint((s.t - s.tD) / s.Tl); }
    else { const u = min(s.t - s.tE, ts); y = s.yv + v * u - (v * u * u) / (2 * ts); z = s.zS; }
    this.moveTo([conv.x, y, z], s.psi, dt);
    if (s.t >= s.tC && !this.held) this.grip(s.box); // vacuum on once the cups are on the box
    if (s.t >= s.tEnd) this.planPlace(s);
  }
  planPlace(s) {
    const sl = s.slot, rel = this.held.rel, k = PCELL.dyn[s.box.sku] ?? 1;
    const cands = [sl.yaw - rel, sl.yaw - rel + PI]
      .map((psi) => ({ psi, q: ik([sl.x, sl.y, s.zH], toolDown(psi), this.q) }))
      .filter((c) => c.q && c.q[0] >= Q1[0] && c.q[0] <= Q1[1]);
    if (!cands.length) { this.stats.ikFail++; throw new Error(`${this.id}: slot ${this.order.placed} (${sl.sku}, layer ${sl.layer}) unreachable`); }
    cands.sort((a, b) => ptpTime(this.q, a.q) - ptpTime(this.q, b.q));
    this.startPtp(cands[0].q, { label: 'PTP_PLACE', slot: sl, zH: s.zH, psi: cands[0].psi }, k);
  }
  grip(b) {
    const f = fk(this.q), yaw = atan2(f.R[3], f.R[0]);
    b.st = 'held';
    this.belt.splice(this.belt.indexOf(b), 1);
    this.held = { box: b, rel: b.c.yaw - yaw, yaw0: yaw, off: [b.c.x - f.tcp[0], b.c.y - f.tcp[1], b.c.z - f.tcp[2]] };
    this.vac = true; this.tcpHist = []; this.boxT0 = this.boxT0 ?? this.t;
  }
  carryHeld() {
    const hd = this.held;
    if (!hd) return;
    const f = fk(this.q), yaw = atan2(f.R[3], f.R[0]), d = yaw - hd.yaw0, c = cos(d), s = sin(d), b = hd.box;
    b.c = { x: f.tcp[0] + c * hd.off[0] - s * hd.off[1], y: f.tcp[1] + s * hd.off[0] + c * hd.off[1], z: f.tcp[2] + hd.off[2], yaw: yaw + hd.rel };
  }
  release(sl) {
    const b = this.held.box, o = this.order;
    this.carryHeld();
    this.stats.placeErr = max(this.stats.placeErr, hypot(b.c.x - sl.x, b.c.y - sl.y, b.c.z - (sl.zTop - b.h / 2)));
    b.st = 'placed'; sl.box = b;
    b.c = { x: sl.x, y: sl.y, z: sl.zTop - sl.h / 2, yaw: sl.yaw };
    b.p = toG(this.P, b.c.x, b.c.y, b.c.z); b.yaw = sl.yaw + this.P.psi;
    this.held = null; this.vac = false;
    o.placed++; this.stats.boxes++;
    if (this.lastPlaceT) this.stats.cycle.push(this.t - this.lastPlaceT);
    this.lastPlaceT = this.t;
    if ((o.placed === o.slots.length || o.slots[o.placed].layer !== sl.layer)) this.world.emit('layerPlaced', { cell: this.i, order: o, layer: sl.layer });
  }

  // ---------- measurements and the cell monitor ----------
  measure(dt) {
    const st = this.stats, L = this.last, f = fk(this.q), bad = [];
    L.tcp = f.tcp; L.sing = singularity(this.q);
    const moving = this.qd.some((v) => abs(v) > 1e-9);
    for (let i = 0; i < 6; i++) {
      st.jv[i] = max(st.jv[i], abs(this.qd[i]) / RB.vMax[i]); st.ja[i] = max(st.ja[i], abs(this.qdd[i]) / RB.aMax[i]);
      st.qLo[i] = min(st.qLo[i], this.q[i]); st.qHi[i] = max(st.qHi[i], this.q[i]);
      if (this.q[i] < RB.qMin[i] - 1e-9 || this.q[i] > RB.qMax[i] + 1e-9) bad.push(`P1 J${i + 1} outside its range`);
      if (abs(this.qd[i]) > RB.vMax[i] * 1.005) bad.push(`P2 J${i + 1} overspeed`);
    }
    if (this.q[0] < Q1[0] - 1e-9 || this.q[0] > Q1[1] + 1e-9) bad.push('P1 J1 outside its software range');
    if (moving || this.held) {
      st.minWrist = min(st.minWrist, abs(L.sing.wrist)); st.minElbow = min(st.minElbow, abs(L.sing.elbow)); st.minShoulder = min(st.minShoulder, abs(L.sing.shoulder));
      if (abs(L.sing.wrist) < PCELL.minWrist) bad.push('P5 wrist singularity margin');
      if (abs(L.sing.elbow) < PCELL.minElbow) bad.push('P5 elbow singularity margin');
      if (abs(L.sing.shoulder) < PCELL.minShoulder) bad.push('P5 shoulder singularity margin');
    }
    const tilt = acos(min(1, max(-1, -f.R[8])));
    if (this.vac) { st.maxTilt = max(st.maxTilt, tilt); if (tilt > 0.5 * DEG) bad.push('P6 tool not vertical with vacuum on'); }
    const H = this.tcpHist;
    H.push(f.tcp); if (H.length > 3) H.shift();
    L.sf = Infinity; L.peel = Infinity; L.carry = Infinity;
    if (this.held && H.length === 3) {
      const a = [0, 1, 2].map((i) => (H[2][i] - 2 * H[1][i] + H[0][i]) / (dt * dt)), ah = hypot(a[0], a[1]), az = max(a[2], 0), b = this.held.box;
      L.sf = VAC / (b.m * (G + az + ah / PTOOL.mu));
      L.peel = PEEL / (b.m * ah * (b.h / 2) + b.m * (G + az) * PTOOL.rowY);
      st.minSF = min(st.minSF, L.sf); st.minPeel = min(st.minPeel, L.peel);
      if (L.sf < PTOOL.sfMin) bad.push('P7 vacuum safety factor');
      if (L.peel < PTOOL.peelMin) bad.push('P7 peel safety factor');
    }
    // carried box: its underside against the stack and the belt, while it swings to the pallet
    if (this.held && this.state === 'PTP_PLACE') {
      const b = this.held.box, zb = b.c.z - b.h / 2, me = outline(b.c.x, b.c.y, b.c.yaw, b.sx, b.sy);
      let d = Infinity;
      if (overlap(me, BELT_POLY)) d = zb - conv.top;
      for (const sl of this.order.slots) {
        if (!sl.box || hypot(b.c.x - sl.x, b.c.y - sl.y) > 0.8) continue;
        if (overlap(me, outline(sl.x, sl.y, sl.yaw, sl.box.sx - 0.004, sl.box.sy - 0.004))) d = min(d, zb - sl.zTop);
      }
      L.carry = d; st.minCarry = min(st.minCarry, d);
      if (d < 0.03) bad.push('P10 carried box clearance');
    }
    if ((this.palMove || this.locked) && moving) bad.push('P8 robot moving while the pallet moves or a forklift is at D');
    if (this.held && !this.vac) bad.push('P7 box held without vacuum');
    L.bad = bad;
    if (bad.length) { st.violations++; if (st.firstBad.length < 12) st.firstBad.push(`${this.id} t=${this.t.toFixed(2)} ${this.state}: ${bad.join('; ')}`); }
  }

  step(dt) {
    this.t += dt;
    for (const b of this.belt) {
      b.c.y += conv.v * dt;
      if (b.c.y > conv.yEnd) { if (b.st === 'belt') this.stats.missed++; b.st = 'lost'; }
    }
    if (this.belt.some((b) => b.st === 'lost')) this.belt = this.belt.filter((b) => b.st !== 'lost');
    this.infeed.step(this.t, dt);
    this.stepPallet(dt);
    this.stepRobot(dt);
    this.carryHeld();
    const P = this.P;
    for (const b of this.belt) { b.p = toG(P, b.c.x, b.c.y, b.c.z); b.yaw = b.c.yaw + P.psi; }
    if (this.held) { const b = this.held.box; b.p = toG(P, b.c.x, b.c.y, b.c.z); b.yaw = b.c.yaw + P.psi; }
    this.measure(dt);
  }
}
