// Cell logic: conveyor, pick-on-the-fly, palletizing pattern and robot motion
// planning. Pure kinematic simulation, no rendering — step(dt) advances time.

import { ROBOT as RB, TOOL, BOX, CELL } from './config.js';
import { fk, ik, toolDown, mul, tr, mv } from './kinematics.js';

const { conv, pallet } = CELL;
const PI = Math.PI;

// Quintic time scaling: zero velocity and acceleration at both ends.
// Peak velocity 1.875/T, peak acceleration 5.7735/T^2 per unit distance.
const quint = (u) => (u <= 0 ? 0 : u >= 1 ? 1 : u * u * u * (10 - 15 * u + 6 * u * u));
const KV = 1.875, KA = 5.7735;

export function ptpTime(qa, qb) {
  let T = 0.05;
  for (let i = 0; i < 6; i++) {
    const d = Math.abs(qb[i] - qa[i]);
    T = Math.max(T, (KV * d) / RB.vMax[i], Math.sqrt((KA * d) / RB.aMax[i]));
  }
  return T;
}
const linTime = (d) => Math.max(0.05, (KV * d) / RB.vLin, Math.sqrt((KA * d) / RB.aLin));

const Z_GRIP = conv.top + BOX.sz; // TCP height when cups touch a box on the belt
const Z_HOVER = Z_GRIP + CELL.hoverPick;
const SLOTS = pallet.nx * pallet.ny * pallet.layers;

// Slot i -> centre of the box's top face. Far row first, far column first,
// so the robot never reaches over boxes it has already placed in the layer.
export function slot(i) {
  const per = pallet.nx * pallet.ny;
  const layer = Math.floor(i / per), k = i % per;
  const iy = pallet.ny - 1 - Math.floor(k / pallet.nx), ix = k % pallet.nx;
  return [
    pallet.x + (ix - (pallet.nx - 1) / 2) * pallet.pitchX,
    pallet.y + (iy - (pallet.ny - 1) / 2) * pallet.pitchY,
    pallet.h + (layer + 1) * BOX.sz,
  ];
}

export class Sim {
  constructor() {
    this.t = 0;
    this.boxes = [];
    this.nextId = 0;
    this.placed = 0; // boxes on the current pallet
    this.spawned = 0; // boxes released for the current pallet
    this.pallets = 0;
    this.missed = 0;
    this.spawnTimer = 0;
    this.vac = false;
    this.held = null;
    this.cycleTimes = [];
    this.lastPlaceT = null;
    this.qReady = this.ikDown([conv.x, CELL.yReady, Z_HOVER], PI / 2, [0, 0, 0, 0, PI / 2, 0]);
    this.q = [...this.qReady];
    this.qd = [0, 0, 0, 0, 0, 0];
    this.state = 'IDLE';
    this.seg = null;
    for (const y of [-1.4, -3.4]) this.spawn(y);
  }

  // Tool-down IK. The gripper is symmetric under 180 deg about its axis, so
  // take whichever yaw keeps J6 closest to where it already is.
  ikDown(p, psi, ref) {
    const a = ik(p, toolDown(psi), ref), b = ik(p, toolDown(psi + PI), ref);
    if (!a || !b) return a || b;
    return Math.abs(a[5] - ref[5]) <= Math.abs(b[5] - ref[5]) ? a : b;
  }

  spawn(y) {
    this.boxes.push({
      id: this.nextId++, state: 'belt',
      p: [conv.x, y, conv.top + BOX.sz / 2], R: [1, 0, 0, 0, 1, 0, 0, 0, 1],
    });
    this.spawned++;
  }

  // Where and when the TCP can meet a moving box: fixed-point on PTP duration.
  planIntercept(box) {
    const lead = (conv.v * CELL.tSync) / 2;
    let T = 1, qh = null, yI = 0;
    for (let k = 0; k < 4; k++) {
      yI = box.p[1] + conv.v * T + lead;
      qh = this.ikDown([conv.x, yI, Z_HOVER], PI / 2, this.q);
      if (!qh) return null;
      T = ptpTime(this.q, qh);
    }
    return { T, qh, yI };
  }

  startPtp(qb, next) {
    this.seg = { t: 0, T: ptpTime(this.q, qb), qa: [...this.q], qb, next };
    this.state = next.label;
  }

  setQ(q, dt) {
    for (let i = 0; i < 6; i++) this.qd[i] = (q[i] - this.q[i]) / dt;
    this.q = q;
  }

  // Cartesian tool-down target -> joints; a failure here is a cell design error.
  moveTo(p, psi, dt) {
    const q = this.ikDown(p, psi, this.q);
    if (!q) throw new Error(`IK failed in ${this.state} at [${p.map((v) => v.toFixed(3))}] t=${this.t.toFixed(2)}`);
    this.setQ(q, dt);
  }

  step(dt) {
    this.t += dt;
    this.stepBelt(dt);
    this.stepRobot(dt);
    if (this.held) {
      const f = fk(this.q), b = this.held;
      b.R = mul(f.R, b.Rrel);
      const o = mv(f.R, b.prel);
      b.p = [f.tcp[0] + o[0], f.tcp[1] + o[1], f.tcp[2] + o[2]];
    }
  }

  stepBelt(dt) {
    for (const b of this.boxes) if (b.state === 'belt' || b.state === 'missed') b.p[1] += conv.v * dt;
    for (const b of this.boxes) {
      if ((b.state === 'belt' || b.state === 'missed') && b.p[1] > conv.yEnd) {
        b.state = 'gone';
        this.missed++;
        this.spawned--; // a replacement must be sent for this pallet
      }
    }
    this.boxes = this.boxes.filter((b) => b.state !== 'gone');
    this.spawnTimer += dt;
    if (this.spawnTimer >= conv.spawnEvery && this.spawned < SLOTS && this.state !== 'SWAP') {
      this.spawnTimer = 0;
      this.spawn(conv.yStart + 0.3);
    }
  }

  stepRobot(dt) {
    const s = this.seg;
    switch (this.state) {
      case 'IDLE': {
        this.qd.fill(0);
        if (this.placed >= SLOTS) {
          this.state = 'SWAP';
          this.seg = { t: 0 };
          break;
        }
        const box = this.boxes.find((b) => b.state === 'belt');
        const plan = box && this.planIntercept(box);
        // belt travel consumed between hover arrival and end of desync
        const zS = this.safeZ();
        const travel = conv.v * (2 * CELL.tSync + linTime(CELL.hoverPick) + TOOL.tGrip + linTime(zS - Z_GRIP));
        if (plan && plan.yI >= conv.yA && plan.yI + travel <= conv.yB) {
          this.seg = { t: 0, T: plan.T, qa: [...this.q], qb: plan.qh, next: { label: 'PTP_PICK', box, zS } };
          this.state = 'PTP_PICK';
        } else if (plan && plan.yI + travel > conv.yB) {
          box.state = 'missed'; // cannot be tracked inside the window any more
        } else if (this.q.some((v, i) => Math.abs(v - this.qReady[i]) > 1e-6)) {
          this.startPtp(this.qReady, { label: 'PTP_READY' });
        }
        break;
      }
      case 'SWAP': {
        this.qd.fill(0);
        if (s.t === 0 && this.q.some((v, i) => Math.abs(v - this.qReady[i]) > 1e-6)) {
          this.startPtp(this.qReady, { label: 'PTP_READY', then: 'SWAP' });
          break;
        }
        s.t += dt;
        if (s.t >= CELL.swapTime) {
          this.boxes = this.boxes.filter((b) => b.state !== 'placed');
          this.placed = 0;
          this.spawned = this.boxes.length;
          this.pallets++;
          this.lastPlaceT = null;
          this.state = 'IDLE';
        }
        break;
      }
      case 'PTP_READY':
      case 'PTP_PICK':
      case 'PTP_PLACE': {
        s.t += dt;
        const u = quint(s.t / s.T);
        this.setQ(s.qa.map((a, i) => a + (s.qb[i] - a) * u), dt);
        if (s.t >= s.T) this.endPtp(s.next);
        break;
      }
      case 'TRACK': this.stepTrack(dt); break;
      case 'PLACE_DOWN':
      case 'PLACE_UP': {
        s.t += dt;
        const u = quint(s.t / s.T);
        this.moveTo([s.x, s.y, s.z0 + (s.z1 - s.z0) * u], s.psi, dt);
        if (s.t >= s.T) {
          if (this.state === 'PLACE_DOWN') {
            this.release(s);
            this.state = 'RELEASE';
            this.seg = { ...s, t: 0 };
          } else this.state = 'IDLE';
        }
        break;
      }
      case 'RELEASE': {
        this.qd.fill(0);
        s.t += dt;
        if (s.t >= TOOL.tRelease) {
          this.seg = { ...s, t: 0, z0: s.z1, z1: s.z0 };
          this.state = 'PLACE_UP';
        }
        break;
      }
    }
  }

  // TCP height that clears every box of the layer being built.
  safeZ() {
    return Math.max(Z_GRIP + CELL.liftPick, slot(this.placed)[2] + CELL.hoverPlace);
  }

  endPtp(next) {
    if (next.label === 'PTP_PICK') {
      const f = fk(this.q);
      const Td = linTime(CELL.hoverPick), Tl = linTime(next.zS - Z_GRIP), ts = CELL.tSync;
      this.seg = {
        t: 0, box: next.box, zS: next.zS, yv: next.box.p[1], off0: f.tcp[1] - next.box.p[1],
        psi: Math.atan2(f.R[3], f.R[0]),
        tB: ts, tC: ts + Td, tD: ts + Td + TOOL.tGrip, tE: ts + Td + TOOL.tGrip + Tl,
        tEnd: 2 * ts + Td + TOOL.tGrip + Tl, Td, Tl,
      };
      this.state = 'TRACK';
    } else if (next.label === 'PTP_PLACE') {
      this.seg = { t: 0, T: linTime(CELL.hoverPlace), x: next.x, y: next.y, z0: next.zH, z1: next.zH - CELL.hoverPlace, psi: next.psi };
      this.state = 'PLACE_DOWN';
    } else if (next.then === 'SWAP') {
      this.state = 'SWAP';
      this.seg = { t: 1e-9 };
    } else this.state = 'IDLE';
  }

  // Conveyor tracking: all motion is expressed relative to a point yv riding the belt.
  stepTrack(dt) {
    const s = this.seg, ts = CELL.tSync, v = conv.v;
    s.t += dt;
    let y, z;
    if (s.t < s.tE) s.yv += v * dt;
    else if (!s.frozen) { // belt reference stops exactly at tE, not at the last whole step
      s.yv += v * (dt - (s.t - s.tE));
      s.frozen = true;
    }
    if (s.t < s.tB) { // accelerate to belt speed, closing the lead distance
      const k = 1 - s.t / ts;
      y = s.yv + s.off0 * k * k;
      z = Z_HOVER;
    } else if (s.t < s.tC) { // descend while riding with the box
      y = s.yv;
      z = Z_HOVER + (Z_GRIP - Z_HOVER) * quint((s.t - s.tB) / s.Td);
    } else if (s.t < s.tD) { // vacuum on, wait for pressure
      if (!this.held) this.grip(s.box);
      y = s.yv;
      z = Z_GRIP;
    } else if (s.t < s.tE) { // lift, still at belt speed
      y = s.yv;
      z = Z_GRIP + (s.zS - Z_GRIP) * quint((s.t - s.tD) / s.Tl);
    } else { // decelerate to rest
      const u = Math.min(s.t - s.tE, ts);
      y = s.yv + v * u - (v * u * u) / (2 * ts);
      z = s.zS;
    }
    this.moveTo([conv.x, y, z], s.psi, dt);
    if (s.t >= s.tEnd) this.planPlace(s);
  }

  planPlace(s) {
    const [x, y, zTop] = slot(this.placed);
    const zH = zTop + CELL.hoverPlace;
    // The box must turn 90 deg; either direction is valid, take the shorter joint move.
    const cands = [s.psi - PI / 2, s.psi + PI / 2]
      .map((psi) => ({ psi, q: ik([x, y, zH], toolDown(psi), this.q) }))
      .filter((c) => c.q);
    if (!cands.length) throw new Error(`slot ${this.placed} unreachable`);
    cands.sort((a, b) => ptpTime(this.q, a.q) - ptpTime(this.q, b.q));
    this.startPtp(cands[0].q, { label: 'PTP_PLACE', x, y, zH, psi: cands[0].psi });
  }

  grip(box) {
    const f = fk(this.q), Rt = tr(f.R);
    box.state = 'held';
    box.Rrel = mul(Rt, box.R);
    box.prel = mv(Rt, [box.p[0] - f.tcp[0], box.p[1] - f.tcp[1], box.p[2] - f.tcp[2]]);
    this.held = box;
    this.vac = true;
  }

  release() {
    this.held.state = 'placed';
    this.held.slot = this.placed;
    this.held = null;
    this.vac = false;
    this.placed++;
    if (this.lastPlaceT !== null) this.cycleTimes.push(this.t - this.lastPlaceT);
    this.lastPlaceT = this.t;
  }
}
