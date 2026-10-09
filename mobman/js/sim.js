// Cell logic: source station, AMR sequencer, arm pick-and-place planner, on-board conveyor,
// destination conveyor and the safety monitor. Pure kinematic simulation, no rendering —
// step(dt) advances time. Spec §6, §9, §10, §12.

import { AMR, ARM, TOOL, DECK, PALLET, RECIPES, ORDERS, SRC, DST, CYCLE, DOCK, TURN, SAFE, G, DEG } from './config.js';
import { Path, line, turn, tangentLength } from '../../amr/js/path.js';
import { scurve } from '../../asrs/js/motion.js';
import * as A from './arm.js';
import * as B from './base.js';
import * as C from './clearance.js';
import { palletBoxes, deckSlots, perLayer, layerExtent } from './recipes.js';

const { PI, abs, min, max, hypot, sin, cos, sqrt, atan2, acos } = Math;

// Quintic time scaling: zero velocity and acceleration at both ends.
// Peak velocity 1.875/T, peak acceleration 5.7735/T² per unit distance.
const quint = (u) => (u <= 0 ? 0 : u >= 1 ? 1 : u * u * u * (10 - 15 * u + 6 * u * u));
const KV = 1.875, KA = 5.7735;

function mulberry32(a) {
  return () => {
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// Docked base poses (spec §10.1).
export const SRC_DOCK = { x: SRC.x, y: SRC.y - PALLET.sy / 2 - SRC.armGap - ARM.base[1], th: 0 };
export const DST_DOCK = { x: DST.x - DECK.yc, y: DST.yEnd + DST.gap + AMR.body.xf, th: -PI / 2 };

// The one route of the cell: source dock -> straight -> 90° right turn -> straight -> destination dock.
export function buildRoute() {
  const T = tangentLength(-PI / 2, TURN.R, TURN.Lc);
  return new Path(SRC_DOCK, [line(DST_DOCK.x - SRC_DOCK.x - T), ...turn(-PI / 2, TURN.R, TURN.Lc), line(SRC_DOCK.y - T - DST_DOCK.y)]);
}

// Reference joint vector: fixes the 2π-turn of J1…J5 for every IK call (spec §6.3).
export const QREF = [0, -0.6, 1.5, 0.6, PI / 2, 0];
const HOME_POSE = { x: 0, y: 0, th: 0 };
export function ikBody(pB, yawB, q6ref = 0) {
  return A.ik(B.worldToArm(HOME_POSE, pB), A.toolDown(B.yawToArm(HOME_POSE, yawB)), [...QREF.slice(0, 5), q6ref], ARM.branch);
}
export const Q_TRAVEL = ikBody(ARM.travelTcp, 0);

export function ptpTime(qa, qb) {
  let T = 0.05;
  for (let i = 0; i < 6; i++) {
    const d = abs(qb[i] - qa[i]);
    T = max(T, (KV * d) / ARM.vMax[i], sqrt((KA * d) / ARM.aMax[i]));
  }
  return T;
}

// ---- safety monitor (spec §12). Reads a state snapshot only; knows nothing about intent.
export function monitor(s) {
  const bad = [], tol = 1.002;
  for (let i = 0; i < 6; i++) {
    if (s.q[i] < ARM.qMin[i] - 1e-9 || s.q[i] > ARM.qMax[i] + 1e-9) bad.push(`M1 J${i + 1} outside its range`);
    if (abs(s.qd[i]) > ARM.vMax[i] * tol) bad.push(`M2 J${i + 1} overspeed`);
    if (abs(s.qdd[i]) > ARM.aMax[i] * 1.02) bad.push(`M2 J${i + 1} over-acceleration`);
  }
  const lim = s.zone ? AMR.drive.creep : s.loaded ? AMR.drive.loaded : AMR.drive.empty;
  if (abs(s.v) > lim.v * tol) bad.push('M3 path speed above the limit of this state');
  if (s.v < -AMR.vRev * tol) bad.push('M3 reverse speed');
  if (max(abs(s.vL), abs(s.vR)) > AMR.wheelV * tol) bad.push('M3 wheel speed');
  if (abs(s.w) > AMR.omega * tol) bad.push('M3 yaw rate');
  if (s.v * s.v * abs(s.k) > AMR.aLat * tol) bad.push('M3 lateral acceleration');
  const moving = abs(s.v) > 1e-6 || abs(s.w) > 1e-6;
  if (s.armMoving && (moving || s.docked !== 'src' || !s.srcLocked)) bad.push('M4 arm moving while the base is not docked, locked and at rest');
  if (moving && (!s.armTravel || s.vac || s.held)) bad.push('M4 base moving while the arm is not stowed and empty');
  if (s.armMoving || s.held) {
    if (abs(s.sing.elbow) < ARM.minElbow) bad.push('M5 elbow singularity margin');
    if (abs(s.sing.wrist) < ARM.minWrist) bad.push('M5 wrist singularity margin');
    if (abs(s.sing.shoulder) < ARM.minShoulder) bad.push('M5 shoulder singularity margin');
  }
  if (s.vac && s.tilt > 0.5 * DEG) bad.push('M6 tool not vertical with vacuum on');
  if (s.held && s.sf < TOOL.sfMin) bad.push('M7 vacuum safety factor');
  if (s.held && !s.vac) bad.push('M7 box held without vacuum');
  if (s.zone === 'src' && !s.srcLocked) bad.push('M8 in the source zone without Locked');
  if (s.zone === 'dst' && !s.dstReserved) bad.push('M8 in the destination zone without DestReady');
  if (abs(s.deckU) > 1e-9 && (moving || s.docked !== 'dst' || !s.dstReserved)) bad.push('M9 rollers running while not docked at the destination');
  if (s.stop < 1 - 1e-9 && (moving || s.docked !== 'dst')) bad.push('M9 front stop not raised while not docked at the destination');
  if (s.armClear < SAFE.armClear) bad.push('M10 arm clearance');
  if (s.carryClear < SAFE.armClear) bad.push('M10 carried box clearance');
  if (s.chassisClear < (s.zone === 'dst' ? DST.gap - AMR.posTol : SAFE.clear) - 1e-9) bad.push('M10 chassis clearance');
  if (s.heldMass + TOOL.mass > ARM.payload) bad.push('M11 arm payload');
  if (s.deckMass > DECK.maxMass || s.deckCount > DECK.cap) bad.push('M11 deck load');
  if (s.stab.margin < 0.10) bad.push('M12 centre of gravity too close to a tipping edge');
  return bad;
}

let NEXT_ID = 1;

export class Cell {
  constructor({ seed = 7, clearEvery = 1 } = {}) {
    NEXT_ID = 1;
    this.rng = mulberry32(seed);
    this.clearEvery = clearEvery;
    this.t = 0; this.tick = 0;
    this.path = buildRoute();
    this.S = this.path.length;
    this.sPreSrc = DOCK.standoff;
    this.sPreDst = this.S - DOCK.standoff;

    // base
    this.s = this.sPreSrc;
    this.pose = this.poseAt(this.s);
    this.v = 0; this.acc = 0; this.w = 0; this.k = 0;
    this.wheel = { L: 0, R: 0, vL: 0, vR: 0 };
    this.odo = { ...this.pose };
    this.run = null;

    // arm and gripper
    this.q = [...Q_TRAVEL]; this.qd = [0, 0, 0, 0, 0, 0]; this.qdd = [0, 0, 0, 0, 0, 0];
    this.vac = false; this.held = null; this.touch = null;
    this.seg = null; this.queue = [];
    this.tcpHist = [];

    // on-board conveyor: `stop` 1 = front stop raised
    this.deck = { u: 0, stop: 1, boxes: [] };
    // source station
    this.src = { pallet: null, b1: false, b2: false, ready: false, locked: false, tStill: 0, tGone: 0, tNew: 0, recipe: 0, served: 0 };
    // destination conveyor
    this.dst = { boxes: [], reserved: false, v: 0 };

    this.boxes = [];
    this.phase = 'WAIT_PALLET';
    this.timer = 0;
    this.order = null; this.orderIx = 0;
    this.tripStart = 0; this.boxStart = 0;
    this.stats = {
      boxes: 0, delivered: 0, trips: 0, pallets: 0, ikFail: 0, violations: 0, firstBad: [],
      boxCycle: [], tripCycle: [], loadTime: [],
      jv: [0, 0, 0, 0, 0, 0], ja: [0, 0, 0, 0, 0, 0], qLo: Array(6).fill(Infinity), qHi: Array(6).fill(-Infinity),
      minElbow: Infinity, minWrist: Infinity, minShoulder: Infinity, maxTilt: 0, minSF: Infinity, maxTcpV: 0, maxTcpA: 0,
      minArmClear: { d: Infinity }, minCarry: { d: Infinity }, minChassis: Infinity, minStab: Infinity, minBrakeTip: Infinity, minSideTip: Infinity,
      vFwd: 0, vRev: 0, vLoaded: 0, vZone: 0, aPath: 0, wheelV: 0, omega: 0, aLat: 0, alpha: 0, wheelA: 0, odoErr: 0, dist: 0,
      placeErr: 0, pickErr: 0, maxRaise: 0, maxDeckMass: 0, maxDeckCount: 0, armWhileMoving: 0, baseWhileArm: 0, zoneNoLock: 0,
    };
    this.last = { bad: [], armClear: { d: Infinity }, carry: { d: Infinity }, sf: Infinity, tilt: 0, sing: A.singularity(this.q), stab: B.stability(this.load()), tcp: this.tcpWorld(), chassis: Infinity, zone: null };
    this.spawnPallet(true);
  }

  poseAt(s) { const a = this.path.at(s); return { x: a.x, y: a.y, th: a.th }; }
  tcpWorld() { return B.armToWorld(this.pose, A.fk(this.q).tcp); }
  get docked() { return this.s < 1e-9 ? 'src' : this.s > this.S - 1e-9 ? 'dst' : null; }
  get zone() { return this.s < this.sPreSrc - 1e-6 ? 'src' : this.s > this.sPreDst + 1e-6 ? 'dst' : null; }
  get loaded() { return this.deck.boxes.length > 0; }
  load() {
    return {
      q: this.q,
      held: this.held ? { m: this.held.box.m, h: this.held.box.h } : null,
      deck: this.deck.boxes.map((b) => ({ m: b.m, x: b.lp[0], y: b.lp[1], z: b.lp[2] })),
    };
  }

  // ---------- source station ----------
  spawnPallet(atStop = false) {
    const s = this.src, rcIx = s.recipe++ % RECIPES.length, rc = RECIPES[rcIx], r = () => 2 * this.rng() - 1;
    const pal = {
      id: ++s.served, rcIx, rc, z: SRC.z, extent: layerExtent(rc),
      xStop: SRC.x + SRC.scatter * r(), y: SRC.y + SRC.scatter * r(), yaw: SRC.scatterYaw * r(),
    };
    pal.x = atStop ? pal.xStop : SRC.xIn;
    pal.boxes = palletBoxes(rc).map((b) => ({ id: NEXT_ID++, rcIx, sx: rc.l, sy: rc.w, h: rc.h, m: rc.mass, st: 'pallet', layer: b.layer, lp: [b.x, b.y, b.z], lyaw: b.yaw, p: [0, 0, 0], yaw: 0, pal }));
    pal.next = 0;
    s.pallet = pal;
    if (atStop) s.tStill = SRC.tDebounce;
    this.boxes.push(...pal.boxes);
    this.placePallet(pal);
  }
  placePallet(pal) {
    const c = cos(pal.yaw), s = sin(pal.yaw);
    for (const b of pal.boxes) {
      if (b.st !== 'pallet') continue;
      b.p = [pal.x + c * b.lp[0] - s * b.lp[1], pal.y + s * b.lp[0] + c * b.lp[1], pal.z + b.lp[2]];
      b.yaw = pal.yaw + b.lyaw;
    }
  }
  stepSource(dt) {
    const s = this.src, p = s.pallet;
    if (!p) {
      s.b1 = s.b2 = s.ready = false;
      s.tNew += dt;
      if (s.tNew >= SRC.tIn) { s.tNew = 0; this.spawnPallet(); }
      return;
    }
    if (p.x < p.xStop) {
      p.x = min(p.xStop, p.x + SRC.vIn * dt);
      s.tStill = 0;
      this.placePallet(p);
    } else s.tStill += dt;
    const lead = p.x + PALLET.sx / 2, trail = p.x - PALLET.sx / 2;
    s.b1 = lead >= SRC.b1x && trail <= SRC.b1x;
    s.b2 = p.x >= p.xStop;
    const left = p.boxes.length - p.next;
    s.ready = s.b1 && s.b2 && s.tStill >= SRC.tDebounce && left > 0;
    if (left === 0 && !s.locked) {
      s.tGone += dt;
      if (s.tGone >= SRC.tOut) { s.tGone = 0; s.pallet = null; this.stats.pallets++; }
    }
  }

  // ---------- base ----------
  startRun(sTo, lim) {
    const cap = B.speedCap(this.path, this.s, sTo, lim);
    if (sTo < this.s) cap.v = min(cap.v, AMR.vRev);
    this.run = { s0: this.s, s1: sTo, dir: sTo >= this.s ? 1 : -1, prof: scurve(abs(sTo - this.s), cap.v, cap.a, cap.j), t: 0 };
  }
  stepBase(dt) {
    const r = this.run, w0 = this.w, vL0 = this.wheel.vL, vR0 = this.wheel.vR;
    if (r) {
      r.t += dt;
      if (r.t >= r.prof.T) { this.s = r.s1; this.v = 0; this.acc = 0; this.run = null; }
      else { const a = r.prof.at(r.t); this.s = r.s0 + r.dir * a.p; this.v = r.dir * a.v; this.acc = r.dir * a.a; }
    } else { this.v = 0; this.acc = 0; }
    const at = this.path.at(this.s);
    this.pose = { x: at.x, y: at.y, th: at.th };
    this.k = at.k;
    this.w = this.v * at.k;
    const wh = B.wheelsOfTwist(this.v, this.w), W = this.wheel;
    // dead reckoning from the wheel speeds only (trapezoid over the step)
    const tw = B.twistOfWheels((vL0 + wh.vL) / 2, (vR0 + wh.vR) / 2);
    this.odo = B.stepPose(this.odo, tw.v, tw.w, dt);
    W.L += ((vL0 + wh.vL) / 2 / AMR.rWheel) * dt; W.R += ((vR0 + wh.vR) / 2 / AMR.rWheel) * dt;
    const st = this.stats;
    st.wheelA = max(st.wheelA, abs(wh.vL - vL0) / dt, abs(wh.vR - vR0) / dt);
    st.alpha = max(st.alpha, abs(this.w - w0) / dt);
    W.vL = wh.vL; W.vR = wh.vR;
    st.dist += abs(this.v) * dt;
    if (this.v > 0) st.vFwd = max(st.vFwd, this.v); else st.vRev = max(st.vRev, -this.v);
    if (this.loaded) st.vLoaded = max(st.vLoaded, abs(this.v));
    if (this.zone) st.vZone = max(st.vZone, abs(this.v));
    st.aPath = max(st.aPath, abs(this.acc));
    st.wheelV = max(st.wheelV, abs(wh.vL), abs(wh.vR));
    st.omega = max(st.omega, abs(this.w));
    st.aLat = max(st.aLat, this.v * this.v * abs(this.k));
    st.odoErr = max(st.odoErr, hypot(this.odo.x - this.pose.x, this.odo.y - this.pose.y));
  }

  // ---------- arm ----------
  // Tool-down IK for a world target. The pad is symmetric under 180° about its axis, so both
  // yaws are valid: take the one that keeps J6 closest to where it is.
  ikDown(pW, yawW, q6 = this.q[5]) {
    const pA = B.worldToArm(this.pose, pW);
    let best = null;
    for (const y of [yawW, yawW + PI]) {
      const q = A.ik(pA, A.toolDown(B.yawToArm(this.pose, y)), [...QREF.slice(0, 5), q6], ARM.branch);
      if (q && (!best || abs(q[5] - q6) < abs(best.q[5] - q6))) best = { q, yaw: y };
    }
    return best;
  }
  ikLin(pW, yawW) {
    const q = A.ik(B.worldToArm(this.pose, pW), A.toolDown(B.yawToArm(this.pose, yawW)), [...QREF.slice(0, 5), this.q[5]], ARM.branch);
    if (!q) { this.stats.ikFail++; throw new Error(`IK failed in ${this.seg?.label} at [${pW.map((v) => v.toFixed(3))}] t=${this.t.toFixed(2)}`); }
    return q;
  }
  // Duration of a straight TCP move: Cartesian limits first, then stretched until every joint
  // stays inside its speed and acceleration limit along the whole line.
  linTime(p0, p1, yaw) {
    const d = hypot(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    let T = max(0.05, (KV * d) / ARM.vLin, sqrt((KA * d) / ARM.aLin));
    const N = 48, qs = [];
    for (let i = 0; i <= N; i++) {
      const u = quint(i / N);
      const q = A.ik(B.worldToArm(this.pose, p0.map((v, k) => v + (p1[k] - v) * u)), A.toolDown(B.yawToArm(this.pose, yaw)), [...QREF.slice(0, 5), this.q[5]], ARM.branch);
      if (!q) { this.stats.ikFail++; throw new Error(`line not reachable at [${p0.map((v) => v.toFixed(3))}] -> [${p1.map((v) => v.toFixed(3))}]`); }
      qs.push(q);
    }
    let rv = 0, ra = 0;
    const h = T / N;
    for (let i = 1; i <= N; i++)
      for (let j = 0; j < 6; j++) {
        const v1 = (qs[i][j] - qs[i - 1][j]) / h;
        rv = max(rv, abs(v1) / ARM.vMax[j]);
        if (i > 1) ra = max(ra, abs(v1 - (qs[i - 1][j] - qs[i - 2][j]) / h) / h / ARM.aMax[j]);
      }
    return T * max(1, 1.03 * rv, 1.03 * sqrt(ra));
  }
  setQ(q, dt) {
    for (let i = 0; i < 6; i++) {
      const v = (q[i] - this.q[i]) / dt;
      this.qdd[i] = (v - this.qd[i]) / dt;
      this.qd[i] = v;
    }
    this.q = q;
  }
  toolYaw() {
    const R = A.mul(A.Rz(this.pose.th + ARM.yaw), A.fk(this.q).R);
    return atan2(R[3], R[0]);
  }
  beginSeg(g) {
    g.t = 0;
    if (g.kind === 'ptp') { g.qa = [...this.q]; g.T = ptpTime(g.qa, g.qb); }
    else if (g.kind === 'lin') g.T = this.linTime(g.p0, g.p1, g.yaw);
    g.start?.();
    this.seg = g;
  }
  stepArm(dt) {
    if (!this.seg && this.queue.length) this.beginSeg(this.queue.shift());
    const g = this.seg;
    if (!g) { this.setQ(this.q, dt); return; }
    g.t += dt;
    const done = g.t >= g.T - 1e-9, u = done ? 1 : quint(g.t / g.T);
    if (g.kind === 'ptp') this.setQ(done ? [...g.qb] : g.qa.map((a, i) => a + (g.qb[i] - a) * u), dt);
    else if (g.kind === 'lin') this.setQ(this.ikLin(g.p0.map((v, k) => v + (g.p1[k] - v) * u), g.yaw), dt);
    else this.setQ(this.q, dt);
    if (done) { this.seg = null; g.end?.(); }
  }
  get armMoving() { return !!this.seg && this.seg.kind !== 'dwell'; }
  get armBusy() { return !!this.seg || this.queue.length > 0; }

  // Fixed equipment: what the arm can reach, and what the chassis drives past (hooks for an integrating system).
  armFrames() { return [C.SRC_FRAME]; }
  fixedFrames() { return [C.SRC_FRAME, C.DST_FRAME]; }
  // Obstacles around the docked AMR, for planning and for the monitor. `skip` leaves one box out.
  obstacles(skip) {
    const obs = [...C.bodyObstacles(this.pose), ...this.armFrames()];
    if (this.src.pallet) obs.push(...C.palletObstacles(this.src.pallet, skip));
    for (const b of this.deck.boxes) if (b !== skip) obs.push(C.boxObb(b));
    return obs;
  }
  // A joint move between two tool-down poses above pA and pB (same TCP height at both ends).
  // Joint interpolation does not keep the TCP height, so the height is raised in 40 mm steps
  // until the whole swing keeps its planning clearance (spec §10.3). `carry`: the box in the
  // gripper, { box, rel } with rel = box yaw − tool yaw.
  planSwing(pA, yawA, pB, yawB, z0, obs, carry) {
    const q6 = this.q[5];
    for (let k = 0; k <= 15; k++) {
      const z = z0 + 0.04 * k;
      const qa = A.ik(B.worldToArm(this.pose, [pA[0], pA[1], z]), A.toolDown(B.yawToArm(this.pose, yawA)), [...QREF.slice(0, 5), q6], ARM.branch);
      if (!qa) break;
      const e = this.ikDown([pB[0], pB[1], z], yawB, qa[5]);
      if (!e) break;
      let ok = true;
      const N = 32;
      for (let i = 0; i <= N && ok; i++) {
        const q = qa.map((a, j) => a + (e.q[j] - a) * (i / N));
        if (C.armClearance(q, this.pose, obs).d < SAFE.armPlan) ok = false;
        else if (carry) {
          const f = A.fk(q), tcp = B.armToWorld(this.pose, f.tcp), R = A.mul(A.Rz(this.pose.th + ARM.yaw), f.R);
          const b = { ...carry.box, p: [tcp[0], tcp[1], tcp[2] - carry.box.h / 2], yaw: atan2(R[3], R[0]) + carry.rel };
          if (C.carryClearance(b, obs).d < SAFE.carryPlan) ok = false;
        }
      }
      if (ok) { this.stats.maxRaise = max(this.stats.maxRaise, z - z0); return { z, qa, qb: e.q, yaw: e.yaw }; }
    }
    this.stats.ikFail++;
    throw new Error(`no collision-free swing at t=${this.t.toFixed(2)} (${this.order.rc.id}, box ${this.order.done})`);
  }
  // One pick-and-place cycle (spec §10.2), planned from wherever the TCP is now.
  planBox() {
    const o = this.order, pal = this.src.pallet, box = pal.boxes[pal.next], slot = o.slots[o.done];
    const h = box.h, zGrip = box.p[2] + h / 2, zDeckTop = DECK.z + h;
    const cur = this.tcpWorld(), yawCur = this.toolYaw();
    const pSlot = B.toWorld(this.pose, [slot.x, slot.y, zDeckTop]);
    const P = (z) => [box.p[0], box.p[1], z], D = (z) => [pSlot[0], pSlot[1], z], U = (z) => [cur[0], cur[1], z];
    // empty, towards the box: the pad clears the layer and the boxes on the deck
    const s1 = this.planSwing(cur, yawCur, box.p, box.yaw, max(zGrip, zDeckTop) + CYCLE.hover, this.obstacles(this.touch));
    // with the box: its underside clears the layer it came from and the boxes on the deck.
    // The box may be set down in either of its two equivalent orientations.
    const rel = box.yaw - s1.yaw;
    const s2 = this.planSwing(box.p, s1.yaw, pSlot, this.pose.th + slot.yaw - rel, max(zGrip + h + CYCLE.lift, zDeckTop + h + CYCLE.clear), this.obstacles(box), { box, rel });
    this.boxStart = this.t;
    this.queue.push(
      { kind: 'lin', label: o.done ? 'RETREAT' : 'RISE', p0: cur, p1: U(s1.z), yaw: yawCur, end: () => { this.touch = null; } },
      { kind: 'ptp', label: 'SWING TO BOX', qb: s1.qb },
      { kind: 'lin', label: 'APPROACH', p0: P(s1.z), p1: P(zGrip), yaw: s1.yaw, start: () => { this.touch = box; } },
      { kind: 'dwell', label: 'GRIP', T: TOOL.tGrip, start: () => { this.vac = true; this.stats.pickErr = max(this.stats.pickErr, hypot(...this.tcpWorld().map((v, i) => v - P(zGrip)[i]))); }, end: () => this.grip(box) },
      { kind: 'lin', label: 'LIFT', p0: P(zGrip), p1: P(s2.z), yaw: s1.yaw },
      { kind: 'ptp', label: 'SWING TO DECK', qb: s2.qb, carry: true },
      { kind: 'lin', label: 'SET DOWN', p0: D(s2.z), p1: D(zDeckTop), yaw: s2.yaw },
      { kind: 'dwell', label: 'RELEASE', T: TOOL.tRelease, start: () => this.release(slot) },
    );
    pal.next++;
  }
  // After the last box: straight up, then to the travel pose.
  planStow() {
    const cur = this.tcpWorld(), yawCur = this.toolYaw(), tv = B.toWorld(this.pose, ARM.travelTcp);
    const s = this.planSwing(cur, yawCur, tv, this.pose.th, max(cur[2] + CYCLE.hover, ARM.travelTcp[2]), this.obstacles(this.touch));
    this.queue.push(
      { kind: 'lin', label: 'RETREAT', p0: cur, p1: [cur[0], cur[1], s.z], yaw: yawCur, end: () => { this.touch = null; } },
      { kind: 'ptp', label: 'SWING TO STOW', qb: s.qb },
      { kind: 'ptp', label: 'STOW', qb: [...Q_TRAVEL] },
    );
  }
  grip(box) {
    const tcp = this.tcpWorld(), yaw = this.toolYaw();
    box.st = 'held';
    this.held = { box, yaw0: yaw, rel: box.yaw - yaw, off: [box.p[0] - tcp[0], box.p[1] - tcp[1], box.p[2] - tcp[2]] };
    this.touch = null;
    this.tcpHist = [];
  }
  release(slot) {
    const b = this.held.box, o = this.order, want = B.toWorld(this.pose, [slot.x, slot.y, slot.z]);
    this.carryHeld();
    this.stats.placeErr = max(this.stats.placeErr, hypot(b.p[0] - want[0], b.p[1] - want[1], b.p[2] - want[2]));
    b.st = 'deck';
    b.lp = B.toBody(this.pose, b.p);
    b.lyaw = b.yaw - this.pose.th;
    this.deck.boxes.push(b);
    this.held = null; this.vac = false; this.touch = b;
    o.done++;
    this.stats.boxes++;
    this.stats.boxCycle.push(this.t - this.boxStart);
  }
  carryHeld() {
    const hd = this.held;
    if (!hd) return;
    const tcp = this.tcpWorld(), yaw = this.toolYaw(), d = yaw - hd.yaw0, c = cos(d), s = sin(d);
    hd.box.p = [tcp[0] + c * hd.off[0] - s * hd.off[1], tcp[1] + s * hd.off[0] + c * hd.off[1], tcp[2] + hd.off[2]];
    hd.box.yaw = yaw + hd.rel;
  }

  // ---------- on-board and destination conveyors ----------
  stepConveyors(dt) {
    const d = this.deck, D = this.dst, front = AMR.body.xf + DST.gap / 2;
    if (d.u !== 0) {
      for (const b of d.boxes) b.lp[0] += d.u * dt;
      for (const b of d.boxes.filter((x) => x.lp[0] > front)) {
        b.st = 'dst';
        b.p = B.toWorld(this.pose, b.lp);
        b.yaw = this.pose.th + b.lyaw;
        D.boxes.push(b);
      }
      d.boxes = d.boxes.filter((b) => b.st === 'deck');
    }
    for (const b of d.boxes) { b.p = B.toWorld(this.pose, b.lp); b.yaw = this.pose.th + b.lyaw; }
    // the destination conveyor follows the deck speed while a layer is handed over, then runs on
    D.v = this.phase === 'TRANSFER' && this.ramp === 'up' ? d.u : D.boxes.length ? DST.v : 0;
    for (const b of D.boxes) b.p[1] -= D.v * dt;
    const out = D.boxes.filter((b) => b.p[1] < DST.yEnd - DST.len + 0.3);
    if (out.length) {
      this.stats.delivered += out.length;
      D.boxes = D.boxes.filter((b) => !out.includes(b));
      this.boxes = this.boxes.filter((b) => !out.includes(b));
    }
  }
  get dstInfeedClear() { return !this.dst.boxes.some((b) => b.p[1] > DST.yEnd - DST.clearLen); }
  // Has every box left the roller bed of the AMR?
  get deckClear() {
    if (this.deck.boxes.length) return false;
    return !this.dst.boxes.some((b) => B.toBody(this.pose, b.p)[0] - max(b.sx, b.sy) / 2 < AMR.body.xf + DST.gap);
  }

  // ---------- AMR sequencer (spec §10.1) ----------
  stepSequencer(dt) {
    const d = this.deck;
    switch (this.phase) {
      case 'WAIT_PALLET':
        if (this.src.ready) { this.src.locked = true; this.startRun(0, AMR.drive.creep); this.phase = 'DOCK_SRC'; }
        break;
      case 'DOCK_SRC':
        if (!this.run) { this.phase = 'SETTLE'; this.timer = 0; }
        break;
      case 'SETTLE':
        this.timer += dt;
        if (this.timer >= CYCLE.tSettle) { this.makeOrder(); this.phase = 'LOAD'; }
        break;
      case 'LOAD':
        if (!this.armBusy) {
          if (this.order.done < this.order.n) this.planBox();
          else { this.planStow(); this.phase = 'STOW'; this.stats.loadTime.push(this.t - this.order.t0); }
        }
        break;
      case 'STOW':
        if (!this.armBusy) { this.startRun(this.sPreSrc, AMR.drive.creep); this.phase = 'UNDOCK_SRC'; }
        break;
      case 'UNDOCK_SRC':
        if (!this.run) { this.src.locked = false; this.startRun(this.sPreDst, AMR.drive.loaded); this.phase = 'TRAVEL'; }
        break;
      case 'TRAVEL':
        if (!this.run) this.phase = 'WAIT_DST';
        break;
      case 'WAIT_DST':
        if (this.dstInfeedClear) { this.dst.reserved = true; this.startRun(this.S, AMR.drive.creep); this.phase = 'DOCK_DST'; }
        break;
      case 'DOCK_DST':
        if (!this.run) { this.phase = 'STOP_DOWN'; this.timer = 0; }
        break;
      case 'STOP_DOWN':
        this.timer += dt;
        d.stop = max(0, 1 - this.timer / CYCLE.tStop);
        if (d.stop === 0) { this.phase = 'TRANSFER'; this.ramp = 'up'; }
        break;
      case 'TRANSFER':
        if (this.ramp === 'up') { d.u = min(DECK.v, d.u + DECK.a * dt); if (d.u === DECK.v && this.deckClear) this.ramp = 'down'; }
        else { d.u = max(0, d.u - DECK.a * dt); if (d.u === 0) { this.phase = 'STOP_UP'; this.timer = 0; } }
        break;
      case 'STOP_UP':
        this.timer += dt;
        d.stop = min(1, this.timer / CYCLE.tStop);
        if (d.stop === 1) { this.startRun(this.sPreDst, AMR.drive.creep); this.phase = 'UNDOCK_DST'; }
        break;
      case 'UNDOCK_DST':
        if (!this.run) { this.dst.reserved = false; this.startRun(this.sPreSrc, AMR.drive.empty); this.phase = 'RETURN'; }
        break;
      case 'RETURN':
        if (!this.run) {
          this.stats.trips++;
          this.stats.tripCycle.push(this.t - this.tripStart);
          this.tripStart = this.t;
          this.phase = 'WAIT_PALLET';
        }
        break;
    }
  }
  // How many boxes this trip takes: the requested quantity, at most one layer, at most what is left.
  makeOrder() {
    const pal = this.src.pallet, rc = pal.rc, want = ORDERS[this.orderIx++ % ORDERS.length] || perLayer(rc);
    const n = min(want, perLayer(rc), pal.boxes.length - pal.next);
    this.order = { n, done: 0, want, rc, pallet: pal.id, slots: deckSlots(rc), t0: this.t };
  }

  // ---------- measurements and the safety monitor ----------
  measure(dt) {
    const st = this.stats, L = this.last, f = A.fk(this.q);
    const tcp = B.armToWorld(this.pose, f.tcp);
    L.tcp = tcp;
    L.sing = A.singularity(this.q);
    L.tilt = acos(min(1, max(-1, -f.z[2])));
    for (let i = 0; i < 6; i++) {
      st.jv[i] = max(st.jv[i], abs(this.qd[i]) / ARM.vMax[i]);
      st.ja[i] = max(st.ja[i], abs(this.qdd[i]) / ARM.aMax[i]);
      st.qLo[i] = min(st.qLo[i], this.q[i]); st.qHi[i] = max(st.qHi[i], this.q[i]);
    }
    // TCP speed and acceleration by finite differences
    const H = this.tcpHist;
    H.push(tcp);
    if (H.length > 3) H.shift();
    let az = 0, ah = 0, sp = 0;
    if (H.length === 3) {
      const a = [0, 1, 2].map((i) => (H[2][i] - 2 * H[1][i] + H[0][i]) / (dt * dt));
      az = a[2]; ah = hypot(a[0], a[1]);
      sp = hypot(H[2][0] - H[1][0], H[2][1] - H[1][1], H[2][2] - H[1][2]) / dt;
      if (this.armBusy) { st.maxTcpV = max(st.maxTcpV, sp); st.maxTcpA = max(st.maxTcpA, hypot(...a)); }
    }
    L.sf = this.held ? B.VAC_FORCE / B.holdForce(this.held.box.m, az, ah) : Infinity;
    const active = this.armMoving || !!this.held;
    if (active) {
      st.minElbow = min(st.minElbow, abs(L.sing.elbow));
      st.minWrist = min(st.minWrist, abs(L.sing.wrist));
      st.minShoulder = min(st.minShoulder, abs(L.sing.shoulder));
    }
    if (this.vac) st.maxTilt = max(st.maxTilt, L.tilt);
    if (this.held) st.minSF = min(st.minSF, L.sf);

    // clearances (sampled every `clearEvery` steps while the arm works)
    if (this.armBusy && this.tick % this.clearEvery === 0) {
      const obs = this.obstacles(this.held?.box ?? this.touch);
      L.armClear = C.armClearance(this.q, this.pose, obs);
      if (L.armClear.d < st.minArmClear.d) st.minArmClear = { ...L.armClear, t: this.t, phase: this.seg?.label };
      if (this.held && this.seg?.carry) {
        L.carry = C.carryClearance(this.held.box, obs);
        if (L.carry.d < st.minCarry.d) st.minCarry = { ...L.carry, t: this.t };
      } else L.carry = { d: Infinity };
    } else if (!this.armBusy) { L.armClear = { d: Infinity }; L.carry = { d: Infinity }; }
    L.chassis = min(...this.fixedFrames().map((f) => C.chassisClearance(this.pose, f)));
    st.minChassis = min(st.minChassis, L.chassis);
    L.stab = B.stability(this.load());
    st.minStab = min(st.minStab, L.stab.margin);
    st.minBrakeTip = min(st.minBrakeTip, L.stab.brakeTip);
    st.minSideTip = min(st.minSideTip, L.stab.sideTip);
    const mDeck = this.deck.boxes.reduce((s, b) => s + b.m, 0);
    st.maxDeckMass = max(st.maxDeckMass, mDeck); st.maxDeckCount = max(st.maxDeckCount, this.deck.boxes.length);

    const moving = abs(this.v) > 1e-6;
    L.armTravel = this.q.every((v, i) => abs(v - Q_TRAVEL[i]) < 1e-6);
    // raw checks, independent of the monitor
    if (moving && (this.armBusy || !L.armTravel)) st.armWhileMoving++;
    if (this.armMoving && moving) st.baseWhileArm++;
    if (this.zone === 'src' && !this.src.locked) st.zoneNoLock++;

    L.zone = this.zone;
    L.bad = monitor({
      q: this.q, qd: this.qd, qdd: this.qdd, v: this.v, w: this.w, k: this.k, vL: this.wheel.vL, vR: this.wheel.vR,
      zone: this.zone, loaded: this.loaded, docked: this.docked, srcLocked: this.src.locked, dstReserved: this.dst.reserved,
      armMoving: this.armMoving, armTravel: L.armTravel, vac: this.vac, held: !!this.held, heldMass: this.held ? this.held.box.m : 0,
      sing: L.sing, tilt: L.tilt, sf: L.sf, deckU: this.deck.u, stop: this.deck.stop,
      armClear: L.armClear.d, carryClear: L.carry.d, chassisClear: L.chassis, deckMass: mDeck, deckCount: this.deck.boxes.length, stab: L.stab,
    });
    if (L.bad.length) {
      st.violations++;
      if (st.firstBad.length < 12) st.firstBad.push(`t=${this.t.toFixed(3)} ${this.phase}/${this.seg?.label ?? '-'}: ${L.bad.join('; ')}`);
    }
  }

  step(dt) {
    this.t += dt; this.tick++;
    this.stepSource(dt);
    this.stepSequencer(dt);
    this.stepBase(dt);
    this.stepArm(dt);
    this.carryHeld();
    this.stepConveyors(dt);
    this.measure(dt);
  }
}
