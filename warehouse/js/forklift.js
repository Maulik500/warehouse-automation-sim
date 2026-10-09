// Forklift for the integrated warehouse (spec §4.5): the verified pallet AMR — kinematics, mast,
// lift targets, pick / deposit sequence and safety monitor R1–R10 of amr/js — driving on the
// forklift ring under traffic control, with jobs from the fleet manager.

import { AMR, MAST, PALLET as FP, DEG } from '../../amr/js/config.js';
import { AxisMove } from '../../asrs/js/motion.js';
import { speedCap, steerOfPath } from '../../amr/js/path.js';
import * as K from '../../amr/js/kinematics.js';
import { Drive, SHAPE, overlap } from './traffic.js';
import { forkRoute } from './roadmap.js';
import { placeBoxes } from './asrs.js';

const { abs, sin, cos, min, max, PI, sqrt } = Math;
const HALF_PI = PI / 2;

// Plan polygon of a pallet for the monitor. Pallet yaw = direction of its 1200 axis.
export function palletPoly(p) {
  if (p._x !== p.x || p._y !== p.y || p._yaw !== p.yaw) { p.poly = K.palletPoly({ x: p.x, y: p.y, yaw: p.yaw - HALF_PI }); p._x = p.x; p._y = p.y; p._yaw = p.yaw; }
  return p.poly;
}

export class Forklift {
  constructor(id, world, home) {
    this.id = id; this.world = world; this.home = home; this.at = home;
    // 0.2 % below the cap on curved runs: the cap samples the path every 5 mm and can miss the exact peak
    this.drive = new Drive(id, world.traffic, SHAPE.fork, (path, a, b, base) => {
      const c = speedCap(path, a, b, b < a ? { ...base, v: min(base.v, AMR.vRev) } : base);
      if (c.kMax > 0) c.v *= 0.998;
      return c;
    });
    // Never stop on the ring where a forklift that is on a spur (or on its way into one) has to
    // back out: it could be waiting for exactly that forklift.
    this.drive.noStop = (r) => world.forklifts.some((o) => o !== this && [o.onSpur, o.target].some((st) => st && st.spur.sweep.some((q) => overlap(q, r))));
    this.target = null;
    this.pose = { ...home.spur.pre };
    this.odo = { ...this.pose };
    this.drive.park(this.pose);
    Object.assign(this, { v: 0, acc: 0, k: 0, phi: 0, vs: 0, w: 0, steerRate: 0, steerAcc: 0 });
    Object.assign(this, { tilt: MAST.travel.tilt, lift: MAST.travel.lift, shift: 0, mastRate: [0, 0, 0] });
    this.wheel = { drive: 0, left: 0, right: 0 };
    this.engaged = null; this.carried = null; this.relYaw = 0;
    this.onSpur = home; // the station whose spur the vehicle stands on (null while on the ring)
    this.phase = 'parked'; this.job = null; this.act = null;
    this.gen = this.cycle();
    this.stats = { jobs: 0, violations: 0, firstBad: [], dist: 0, minClear: Infinity, cycle: [], vEmpty: 0, vLoaded: 0, vRev: 0, vZone: 0, steer: 0, steerRate: 0, aLat: 0, lift: [Infinity, -Infinity], mastWhileMoving: 0, zoneNoLock: 0, odoErr: 0, byKind: {} };
    this.last = { bad: [], clearance: Infinity, zone: null };
  }
  get loaded() { return !!this.carried; }
  get idle() { return !this.job; }

  // --- actions: { start(t), update(t), done(t) } ---
  run(path, s0, s1, base, phase, stops) {
    return { start: () => { this.phase = phase; this.drive.go(path, s0, s1, base, stops); }, update: () => {}, done: () => !this.drive.busy };
  }
  mast(target, mode, phase) {
    const lims = { tilt: MAST.tilt, lift: { empty: MAST.lift, loaded: MAST.liftLoaded, creep: MAST.liftCreep }[mode], shift: MAST.shift };
    const axes = ['tilt', 'lift', 'shift'];
    let moves;
    return {
      start: (t) => { this.phase = phase; moves = axes.map((a) => (a in target ? new AxisMove(this[a], target[a], lims[a], t) : null)); },
      update: (t) => moves.forEach((m, i) => { if (!m) return; const s = m.at(t); this[axes[i]] = s.p; this.mastRate[i] = s.v; }),
      done: (t) => moves.every((m) => !m || m.done(t)),
    };
  }
  wait(cond, phase) { return { start: () => { this.phase = phase; }, update: () => {}, done: () => cond() }; }

  *travel(to) {
    const from = this.at;
    if (from === to) return;
    const base = () => (this.loaded ? AMR.drive.loaded : AMR.drive.empty), sp = from.spur.path;
    yield this.run(sp, sp.length, 0, base(), `reverse out of ${from.id}`, [0]);
    this.onSpur = null;
    const r = forkRoute(from.spur, to.spur);
    this.target = to;
    yield this.run(r, 0, r.length, base(), `drive to ${to.id}`);
    this.at = to; this.onSpur = to; this.target = null;
  }

  *cycle() {
    const creep = AMR.drive.creep, T = MAST.travel, fleet = this.world.fleet;
    while (true) {
      this.avail = true;
      yield this.wait(() => (this.job = fleet.nextJob(this)) || this.at !== this.home, this.at === this.home ? 'parked' : 'idle');
      this.avail = false;
      if (!this.job) { yield* this.travel(this.home); continue; }
      const { pick, drop } = this.job, t0 = this.world.t;
      yield* this.travel(pick);

      // ---- pick (AMR spec §8.1) ----
      yield this.wait(() => pick.pickReady && pick.lock(this.id), `${pick.id}: wait for PalletReady / Locked`);
      const p = pick.presented(), zsP = pick.pickZs ? pick.pickZs() : pick.zs(), tg = K.liftTargets(zsP), dockP = pick.spur.dockPath;
      const lat = -(p.x - pick.x) * sin(pick.th) + (p.y - pick.y) * cos(pick.th);
      yield this.mast({ tilt: 0, lift: tg.entry, shift: max(MAST.shift.min, min(MAST.shift.max, lat)) }, 'empty', `${pick.id}: set fork height and side shift`);
      yield this.run(dockP, 0, K.D_IN, creep, `${pick.id}: insert forks`, [K.D_IN]);
      this.engaged = { pallet: p, zs: zsP, station: pick, mode: 'pick' };
      yield this.mast({ lift: tg.carry }, 'creep', `${pick.id}: lift pallet`);
      yield this.run(dockP, K.D_IN, 0, creep, `${pick.id}: withdraw`, [0]);
      pick.unlock(this.id);
      yield this.mast({ tilt: T.tilt, lift: T.lift, shift: 0 }, 'loaded', 'lower to travel pose');
      this.job.onPicked?.(p);
      if (this.job.beforeDrop) yield this.wait(() => this.job.beforeDrop(), `${pick.id}: wait until ${drop.id} can take the pallet`);

      // ---- deliver ----
      yield* this.travel(drop);
      yield this.wait(() => drop.dropReady && drop.lock(this.id), `${drop.id}: wait for free / Locked`);
      const zsD = drop.dropZs ? drop.dropZs() : drop.zs(), put = K.liftTargets(zsD), dockD = drop.spur.dockPath;
      yield this.mast({ tilt: 0, lift: put.carry }, 'loaded', `${drop.id}: raise to deposit height`);
      yield this.run(dockD, 0, K.D_IN, creep, `${drop.id}: approach`, [K.D_IN]);
      Object.assign(this.engaged, { zs: zsD, station: drop, mode: 'put' });
      yield this.mast({ lift: put.entry }, 'creep', `${drop.id}: set down`);
      yield this.run(dockD, K.D_IN, 0, creep, `${drop.id}: withdraw`, [0]);
      this.engaged = null;
      drop.unlock(this.id);
      yield this.mast({ tilt: T.tilt, lift: T.lift }, 'empty', 'travel pose');
      this.stats.jobs++; this.stats.cycle.push(this.world.t - t0);
      this.stats.byKind[this.job.kind] = (this.stats.byKind[this.job.kind] ?? 0) + 1;
      fleet.done(this.job, p);
      this.job = null;
    }
  }

  // The pallet follows the fork once the fork top reaches the deck underside (form fit).
  updateLoad() {
    const e = this.engaged;
    if (!e) return;
    const p = e.pallet, top = K.tcpWorld(this.pose, this.tilt, this.lift, this.shift)[2];
    if (e.mode === 'pick' && top - FP.pocketHi > e.zs + 1e-9) {
      e.station.take(p);
      this.relYaw = p.yaw - this.pose.th;
      p.carried = true; p.holder = this.id; p.dom = 'g';
      this.carried = p; e.mode = 'hold';
    } else if (e.mode === 'put' && top - FP.pocketHi <= e.zs) {
      p.z = e.zs; p.pitch = 0; p.carried = false;
      this.carried = null;
      e.station.accept(p, this.world.t);
      e.mode = 'rest';
      placeBoxes(p);
    }
    if (e.mode === 'hold' || e.mode === 'put') {
      const c = K.carriedPallet(this.pose, this.tilt, this.lift, this.shift);
      p.x = c.x; p.y = c.y; p.z = c.z; p.yaw = c.yaw + this.relYaw; p.pitch = c.pitch;
      placeBoxes(p);
    }
  }
  load() {
    const p = this.carried;
    if (!p) return null;
    return { mass: p.mass, x: K.tcpBody(this.tilt, this.lift, this.shift)[0], z: p.z + (p.cgz ?? 0.07) };
  }
  // What the safety monitor may know about the surroundings: the stations of the current job,
  // their conveyor frames, and every pallet standing near the vehicle.
  env() {
    const e = this.engaged, j = this.job, stations = [], frames = [], pallets = [];
    const add = (st, role) => {
      if (!st || stations.some((x) => x.id === st.id)) return;
      const mine = e && e.station === st;
      const zs = mine ? e.zs : role === 'pick' ? (st.pickZs ? st.pickZs() : st.zs()) : st.dropZs ? st.dropZs() : st.zs();
      const pallet = mine ? (e.mode === 'pick' || e.mode === 'rest' ? e.pallet : null) : role === 'pick' ? st.presented() : null;
      stations.push({ id: st.id, kind: st.kind, x: st.x, y: st.y, th: st.th, zs, pallet, locked: st.locked === this.id || st.kind !== 'pick' });
      for (const f of st.frames ?? []) frames.push(f);
    };
    if (j) { add(j.pick, 'pick'); add(j.drop, 'drop'); }
    for (const p of this.world.looseNear(this.pose, 5)) { if (p === this.carried) continue; pallets.push({ poly: palletPoly(p), station: p.holder, carried: false, z: p.z }); }
    if (this.carried) pallets.push({ poly: palletPoly(this.carried), station: null, carried: true, z: this.carried.z });
    return { stations, frames, pallets };
  }
  state() { return { pose: this.pose, v: this.v, phi: this.phi, vs: this.vs, steerRate: this.steerRate, tilt: this.tilt, lift: this.lift, shift: this.shift, mastRate: this.mastRate, loaded: this.loaded }; }

  step(dt) {
    const t = this.world.t, tw0 = K.bodyTwist(this.vs, this.phi), d = this.drive;
    if (!this.act) { this.act = this.gen.next().value; this.act.start(t - dt); }
    this.mastRate = [0, 0, 0];
    this.act.update(t);
    d.step(t, dt);
    this.pose = { ...d.pose }; this.v = d.v; this.acc = d.a;
    if (d.path && abs(d.v) > 0) {
      const c = d.path.at(d.s), st = steerOfPath(c);
      Object.assign(this, { k: c.k, w: c.k * d.v, phi: st.phi, vs: d.v * sqrt(1 + AMR.L * AMR.L * c.k * c.k), steerRate: st.dphi * d.v, steerAcc: st.ddphi * d.v * d.v + st.dphi * d.a });
    } else Object.assign(this, { w: 0, vs: 0, steerRate: 0, steerAcc: 0, phi: d.path && d.goal ? steerOfPath(d.path.at(d.s)).phi : 0, k: 0 });
    if (this.act.done(t)) this.act = null;
    this.updateLoad();

    // wheels and dead reckoning from the wheel commands only
    const tw = K.bodyTwist(this.vs, this.phi), v = (tw0.v + tw.v) / 2, w = (tw0.w + tw.w) / 2, ws = K.loadWheelSpeeds(v, w), S = this.stats;
    this.odo = K.stepPose(this.odo, v, w, dt);
    this.wheel.drive += (this.vs / AMR.rDrive) * dt; this.wheel.left += (ws.left / AMR.rLoad) * dt; this.wheel.right += (ws.right / AMR.rLoad) * dt;
    S.dist += abs(v) * dt;
    S.odoErr = max(S.odoErr, Math.hypot(this.odo.x - this.pose.x, this.odo.y - this.pose.y));

    const L = (this.last = K.monitor(this.state(), this.env()));
    if (L.bad.length) { S.violations++; if (S.firstBad.length < 12) S.firstBad.push(`${this.id} t=${t.toFixed(2)} ${this.phase}: ${L.bad.join('; ')}`); }
    S.minClear = min(S.minClear, L.clearance);
    if (this.v > 0) { if (this.loaded) S.vLoaded = max(S.vLoaded, this.v); else S.vEmpty = max(S.vEmpty, this.v); } else S.vRev = max(S.vRev, -this.v);
    if (L.zone) S.vZone = max(S.vZone, abs(this.v));
    S.steer = max(S.steer, abs(this.phi)); S.steerRate = max(S.steerRate, abs(this.steerRate)); S.aLat = max(S.aLat, this.v * this.v * abs(this.k));
    S.lift = [min(S.lift[0], this.lift), max(S.lift[1], this.lift)];
    if (abs(this.v) > 1e-4 && this.mastRate.some((r) => abs(r) > 1e-6)) S.mastWhileMoving++;
  }
}
export { DEG };
