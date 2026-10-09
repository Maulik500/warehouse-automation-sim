// The integrated warehouse (spec §10): one clock for every machine and vehicle. Machines never
// call each other; they meet through station handshakes, the WES and the fleet manager.
// No rendering dependencies — runs in Node and in the browser.

import * as FK from '../../amr/js/kinematics.js';
import * as MC from '../../mobman/js/clearance.js';
import { DT, FLOOR, FLEET, PLACE, PCELL, PALLET, ROAD, ROWS, SKU, toG } from './config.js';
import { Traffic, SHAPE, rect } from './traffic.js';
const spurSweep = (sp) => Array.from({ length: Math.ceil(sp.path.length / 0.4) + 1 }, (_, i) => rect(sp.path.at(Math.min(sp.path.length, i * 0.4)), SHAPE.fork.claim));
import { makeSpur } from './roadmap.js';
import { Asrs, placeBoxes } from './asrs.js';
import { PalCell } from './palletizer.js';
import { floorSlot, stack, parking, Wrapper } from './stations.js';
import { Forklift } from './forklift.js';
import { RobotAmr } from './mobile.js';
import { Wes, Fleet } from './wes.js';

const { min, max, hypot } = Math;
function rng32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export class World {
  constructor({ seed = 7, forklifts = FLEET.forklifts, robots = FLEET.robots, receiving = true, autoOrders = true } = {}) {
    this.opts = { seed, forklifts, robots, receiving };
    this.rand = rng32(seed);
    this.t = 0; this.tickNo = 0;
    this.listeners = {};
    this.traffic = new Traffic();
    this.asrs = new Asrs({ rand: this.rand });
    this.cells = PLACE.pal.map((_, i) => new PalCell(i, this));
    this.wrap = new Wrapper(this);
    this.storage = FLOOR.storage.map(floorSlot);
    this.empties = FLOOR.empties.map(stack);
    this.receiving = FLOOR.receiving.map(floorSlot);
    this.parking = FLOOR.parking.map(parking);
    this.fleet = new Fleet(this);
    this.wes = new Wes(this);
    this.wes.auto = autoOrders;

    // forklift spurs
    for (const row of ROWS) this.asrs.lanes[row].spur = makeSpur(this.asrs.lanes[row], 'E');
    for (const c of this.cells) c.station.spur = makeSpur(c.station, 'S');
    for (const st of [this.wrap.I4, this.wrap.O4, ...this.storage, ...this.empties]) st.spur = makeSpur(st, 'N');
    for (const st of [...this.receiving, ...this.parking]) st.spur = makeSpur(st, 'S');
    for (const st of [...ROWS.map((r) => this.asrs.lanes[r]), ...this.cells.map((c) => c.station), this.wrap.I4, this.wrap.O4, ...this.storage, ...this.empties, ...this.receiving, ...this.parking]) st.spur.sweep = spurSweep(st.spur);
    this.stations = [...ROWS.map((r) => this.asrs.lanes[r]), ...this.cells.map((c) => c.station), this.wrap.I4, this.wrap.O4, ...this.storage, ...this.empties, ...this.receiving];

    // fixed equipment the robot AMRs drive past / reach over
    this.mmFrames = {};
    for (const row of ROWS) { const f = this.asrs.lanes[row].frame; this.mmFrames[row] = MC.obb(`lane ${row}`, (f.x0 + f.x1) / 2, f.y, f.top / 2, f.x1 - f.x0, 2 * f.hw, f.top); }
    this.mmFrameList = [...Object.values(this.mmFrames), ...this.cells.map((c) => {
      const R = PCELL.recv, [x, y] = toG(c.P, (R.x0 + R.x1) / 2, R.y), top = c.P.tz + R.z;
      return MC.obb(`receiving conveyor ${c.i + 1}`, x, y, top / 2, R.w + 0.08, R.x1 - R.x0, top);
    })];

    // initial pallets: empties on the stacks and one at every build position
    this.empties.forEach((st, k) => { for (let n = 0; n < FLOOR.emptyStart[k]; n++) { const p = this.fleet.emptyPallet(st.x, st.y, n * PALLET.h); p.yaw = 0; st.accept(p); } });
    for (const c of this.cells) {
      const g = toG(c.P, PCELL.build.x, PCELL.build.y), p = this.fleet.emptyPallet(g[0], g[1], c.P.tz);
      c.pal = p; p.holder = c.id; c.palX = PCELL.build.x; c.palState = 'B_READY';
    }

    this.forklifts = Array.from({ length: forklifts }, (_, k) => new Forklift(`F${k + 1}`, this, this.parking[k]));
    this.robots = Array.from({ length: robots }, (_, k) => new RobotAmr(`M${k + 1}`, this, [4, 2][k]));
    this.vehicles = [...this.forklifts.map((v) => ({ v, shape: SHAPE.fork })), ...this.robots.map((v) => ({ v, shape: SHAPE.mm }))];
    this.stats = { shipped: 0, shippedBoxes: 0, emptiesOut: 0, minGap: Infinity, gapViol: 0, lostBoxes: 0, cycle: [] };
    this.shippedLog = [];
    this.raw = { laneMovedLocked: 0, cellConvMovedLocked: 0, robotMovedForkAtD: 0, wrapMovedLocked: 0, forkZoneNoLock: 0 };
    this._loose = null;
  }
  on(ev, fn) { (this.listeners[ev] ??= []).push(fn); return this; }
  emit(ev, data) { for (const fn of this.listeners[ev] ?? []) fn(data); }

  // Every pallet that stands or travels outside the ASRS (global coordinates).
  loose() {
    if (this._loose) return this._loose;
    const out = [];
    for (const s of [...this.storage, ...this.receiving]) if (s.pallet) out.push(s.pallet);
    for (const s of this.empties) out.push(...s.pallets);
    for (const c of this.cells) if (c.pal) out.push(c.pal);
    for (const w of this.wrap.pallets) out.push(w.src);
    for (const r of ROWS) { const l = this.asrs.lanes[r]; if (l.pallet && l.pallet.dom === 'g') out.push(l.pallet); }
    for (const f of this.forklifts) if (f.carried) out.push(f.carried);
    return (this._loose = out);
  }
  looseNear(pose, r) { return this.loose().filter((p) => hypot(p.x - pose.x, p.y - pose.y) < r); }

  step(dt = DT) {
    this.t += dt; this.tickNo++; this._loose = null;
    this.asrs.step(dt);
    for (const c of this.cells) c.step(dt);
    this.wrap.step(dt);
    this.wes.step();
    for (const r of this.robots) r.step(dt);
    for (const f of this.forklifts) f.step(dt);

    // raw checks, independent of every monitor (spec §15, "Handshakes")
    const R = this.raw;
    for (const r of ROWS) { const l = this.asrs.lanes[r]; if (l.locked && l.move) R.laneMovedLocked++; }
    for (const c of this.cells) { if (c.locked && c.palMove) R.cellConvMovedLocked++; if (c.locked && c.qd.some((v) => Math.abs(v) > 1e-9)) R.robotMovedForkAtD++; }
    for (const p of this.wrap.pallets) if (p.move && ((this.wrap.i4Lock && (p.zone === -1 || p.from === -1)) || (this.wrap.o4Lock && p.zone === 7))) R.wrapMovedLocked++;
    for (const f of this.forklifts) {
      const z = f.last.zone, st = z && f.job && [f.job.pick, f.job.drop].find((s) => s.id === z);
      if (st && st.kind === 'pick' && st.locked !== f.id) R.forkZoneNoLock++;
    }
    // shipping: a stored pallet leaves the building after a dwell
    for (const s of this.storage) {
      const p = s.pallet;
      if (p && !p.carried && p.tStored !== undefined && this.t - p.tStored >= FLOOR.tShip && !s.reserved) {
        for (const b of p.boxes) this.asrs.boxes.delete(b);
        this.stats.shipped++; this.stats.shippedBoxes += p.boxes.length;
        this.shippedLog.push({ id: p.id, order: p.order, layers: p.layerSkus, boxes: p.boxes.length, h: p.h, t: this.t, built: p.built, wrapped: !!p.wrapped });
        s.pallet = null;
      }
    }
    // surplus empty pallets: a full stack is collected when the station is nearly full
    const total = this.empties.reduce((n, s) => n + s.pallets.length, 0);
    for (const s of this.empties) {
      const full = s.pallets.length >= s.max && !s.reservedIn && !s.reservedOut && total > FLOOR.emptyKeep && !this.fleet.near(s) && !this.forklifts.some((f) => f.onSpur === s);
      s.tFull = full ? (s.tFull ?? this.t) : undefined;
      if (full && this.t - s.tFull >= FLOOR.tStack) { this.stats.emptiesOut += s.pallets.length; s.pallets = []; s.tFull = undefined; break; }
    }
    // fleet monitor: no two vehicle outlines closer than ROAD.gap
    if (this.tickNo % 8 === 0) {
      const V = this.vehicles;
      for (let i = 0; i < V.length; i++)
        for (let j = i + 1; j < V.length; j++) {
          const a = V[i].v.pose, b = V[j].v.pose;
          if (hypot(a.x - b.x, a.y - b.y) > 6) continue;
          const d = FK.polyDist(rect(a, V[i].shape.out), rect(b, V[j].shape.out));
          if (d < this.stats.minGap) this.stats.minGap = d;
          if (d < ROAD.gap) this.stats.gapViol++;
        }
    }
  }

  // ---- bookkeeping for the verification ----
  boxCount() {
    let rack = 0, tracked = this.asrs.boxes.size;
    for (const p of this.asrs.pallets) if (!p.boxes) rack += p.layers * p.skuDef.perLayer;
    return { created: this.asrs.counts.boxesIn, rack, tracked, shipped: this.stats.shippedBoxes };
  }
  violations() {
    return {
      asrs: this.asrs.violations,
      cells: this.cells.reduce((s, c) => s + c.stats.violations, 0),
      wrapper: this.wrap.stats.violations,
      robots: this.robots.reduce((s, r) => s + r.stats.violations, 0),
      forklifts: this.forklifts.reduce((s, f) => s + f.stats.violations, 0),
      fleet: this.stats.gapViol,
    };
  }
}
export { DT, SKU, min, max, placeBoxes };
