// Robot AMR (spec §4.2): the verified mobile depalletizer (Cell of mobman/js/sim.js) with its
// one fixed route replaced by the warehouse loop and traffic control. Arm kinematics, the
// pick-and-place planner, the deck and the safety monitor M1–M12 are inherited unchanged.

import { Cell } from '../../mobman/js/sim.js';
import { AMR, TOOL, DECK, DST, CYCLE, G } from '../../mobman/js/config.js';
import * as B from '../../mobman/js/base.js';
import * as C from '../../mobman/js/clearance.js';
import { deckSlots, perLayer } from '../../mobman/js/recipes.js';
import { Drive, SHAPE } from './traffic.js';
import { mmRoutes, MM_LINK, MM_STANDOFF, sOfRoadY } from './roadmap.js';
import { MTOOL_PEEL } from './config.js';

const { PI, abs, min, max, hypot } = Math;
const SPIN = { v: AMR.omega, a: AMR.alpha, j: 4 * AMR.alpha };
const ROUTES = [0, 1, 2, 3].map(mmRoutes);
const QUEUE_S = [2, 4]; // waiting positions at the head of the ASRS road (distance from the head)
const PEEL_AVAIL = (TOOL.cups / 2) * TOOL.vacuum * PI * (TOOL.cupDia / 2) ** 2 * 2 * MTOOL_PEEL.rowY;

export class RobotAmr extends Cell {
  constructor(id, world, s0) {
    super({ seed: 7, clearEvery: 4 });
    this.id = id; this.world = world;
    this.drive = new Drive(id, world.traffic, SHAPE.mm, (path, a, b, base) => {
      const cap = B.speedCap(path, a, b, base);
      if (b < a) cap.v = min(cap.v, AMR.vRev);
      return cap;
    });
    this.outPath = ROUTES[0].out; // every outbound route is identical along the ASRS road
    const p = this.outPath.at(s0);
    this.pose = { x: p.x, y: p.y, th: p.th };
    this.odo = { ...this.pose };
    this.drive.park(this.pose);
    this.drive.path = this.outPath; this.drive.s = s0;
    this.dockState = null; this.zoneState = null;
    this.job = null; this.lane = null; this.route = null; this.infeed = null;
    this.phase = 'IDLE';
    this.dyn = 1;
    this.stats.minPeel = Infinity; this.stats.layers = 0; this.stats.tripT = [];
    this.last.peel = Infinity;
  }
  spawnPallet() {}
  stepSource() {}
  get docked() { return this.dockState; }
  get zone() { return this.zoneState; }
  get dstInfeedClear() { return !this.infeed || this.infeed.destReady; }
  get deckClear() { return this.deck.boxes.length === 0 && this.infeed.clearOfDock; }
  get idle() { return this.phase === 'IDLE' && !this.job; }
  armFrames() { return this.lane ? [this.world.mmFrames[this.lane.row]] : []; }
  // only the equipment within reach of the chassis outline (each frame is at most 5.6 m long)
  fixedFrames() { const p = this.pose; return this.world.mmFrameList.filter((f) => hypot(f.c[0] - p.x, f.c[1] - p.y) < 5.0); }

  assign(job) { this.job = job; }

  stepBase(dt) {
    const d = this.drive, w0 = this.w, vL0 = this.wheel.vL, vR0 = this.wheel.vR;
    d.step(this.t, dt);
    this.pose = { ...d.pose };
    this.v = d.v; this.acc = d.a; this.w = d.w ?? 0; this.k = abs(d.v) > 1e-9 ? d.k ?? 0 : 0;
    const wh = B.wheelsOfTwist(this.v, this.w), W = this.wheel, st = this.stats;
    const tw = B.twistOfWheels((vL0 + wh.vL) / 2, (vR0 + wh.vR) / 2);
    this.odo = B.stepPose(this.odo, tw.v, tw.w, dt);
    W.L += ((vL0 + wh.vL) / 2 / AMR.rWheel) * dt; W.R += ((vR0 + wh.vR) / 2 / AMR.rWheel) * dt;
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

  // S5 is carried at reduced dynamics (peel moment on the pad, spec §9.2).
  beginSeg(g) {
    super.beginSeg(g);
    if (g.kind === 'ptp' && g.carry && this.dyn < 1) g.T /= this.dyn;
  }
  makeOrder() {
    const pal = this.src.pallet, rc = pal.rc, n = min(perLayer(rc), pal.boxes.length - pal.next);
    this.order = { n, done: 0, want: n, rc, pallet: pal.id, slots: deckSlots(rc), t0: this.t };
    this.dyn = pal.sku === 'S5' ? MTOOL_PEEL.S5scale : 1;
  }

  stepSequencer(dt) {
    const d = this.deck, D = this.drive, j = this.job, creep = AMR.drive.creep;
    switch (this.phase) {
      case 'IDLE':
        if (j && !D.move) {
          D.goal = null;
          this.route = ROUTES[j.cell]; this.lane = this.world.asrs.lanes[j.row]; this.infeed = this.world.cells[j.cell].infeed;
          this.sDock = sOfRoadY(this.lane.y);
          this.tripStart = this.t;
          D.go(this.route.out, D.s, this.sDock - MM_STANDOFF, AMR.drive.empty);
          this.phase = 'TO_SRC';
        } else if (!j && !D.busy && D.s < QUEUE_S[1] - 1e-6) D.go(this.outPath, D.s, QUEUE_S[1], AMR.drive.empty, QUEUE_S.filter((h) => h > D.s + 1e-6));
        break;
      case 'TO_SRC':
        if (!D.busy) this.phase = 'WAIT_PALLET';
        break;
      case 'WAIT_PALLET':
        if (this.lane.ready && this.lane.lock(this.id)) {
          this.src.locked = true; this.src.ready = true; this.src.pallet = this.lane.pallet; this.zoneState = 'src';
          D.go(this.route.out, D.s, this.sDock, creep, [this.sDock]);
          this.phase = 'DOCK_SRC';
        }
        break;
      case 'DOCK_SRC':
        if (!D.busy) { this.dockState = 'src'; this.phase = 'SETTLE'; this.timer = 0; }
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
        if (!this.armBusy) { this.dockState = null; D.go(this.route.out, this.sDock, this.sDock + MM_STANDOFF, creep, [this.sDock + MM_STANDOFF]); this.phase = 'UNDOCK_SRC'; }
        break;
      case 'UNDOCK_SRC':
        if (!D.busy) {
          this.src.locked = false; this.src.ready = false; this.src.pallet = null; this.zoneState = null;
          this.lane.unlock(this.id);
          this.world.asrs.release(this.lane.row, this.world.wes.committed[this.lane.row] > 0);
          this.world.emit('layerLoaded', { robot: this.id, job: j });
          this.lane = null;
          D.go(this.route.out, D.s, this.route.out.length, AMR.drive.loaded);
          this.phase = 'TRAVEL';
        }
        break;
      case 'TRAVEL':
        if (!D.busy) this.phase = 'WAIT_DST';
        break;
      case 'WAIT_DST':
        if (this.infeed.destReady && !this.infeed.reserved) {
          this.infeed.reserved = this.id; this.dst.reserved = true; this.zoneState = 'dst';
          D.go(this.route.dock, 0, MM_STANDOFF, creep, [MM_STANDOFF]);
          this.phase = 'DOCK_DST';
        }
        break;
      case 'DOCK_DST':
        if (!D.busy) { this.dockState = 'dst'; this.phase = 'STOP_DOWN'; this.timer = 0; }
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
        if (d.stop === 1) { this.dockState = null; D.go(this.route.dock, MM_STANDOFF, 0, creep, [0]); this.phase = 'UNDOCK_DST'; }
        break;
      case 'UNDOCK_DST':
        if (!D.busy) {
          this.dst.reserved = false; this.infeed.reserved = null; this.zoneState = null;
          this.world.emit('layerDelivered', { robot: this.id, job: j });
          this.stats.layers++;
          D.turn(PI, SPIN); this.phase = 'SPIN_OUT';
        }
        break;
      case 'SPIN_OUT':
        if (!D.busy) { D.go(this.route.back, 0, this.route.back.length, AMR.drive.empty); this.phase = 'RETURN'; }
        break;
      case 'RETURN':
        if (!D.busy) { D.turn(-PI / 2, SPIN); this.phase = 'SPIN_TOP_1'; }
        break;
      case 'SPIN_TOP_1':
        if (!D.busy) { D.go(MM_LINK, 0, MM_LINK.length, AMR.drive.empty, [MM_LINK.length]); this.phase = 'LINK'; }
        break;
      case 'LINK':
        if (!D.busy) { D.turn(-PI / 2, SPIN); this.phase = 'SPIN_TOP_2'; }
        break;
      case 'SPIN_TOP_2':
        if (!D.busy) {
          D.path = this.outPath; D.s = 0;
          this.stats.trips++; this.stats.tripT.push(this.t - this.tripStart);
          this.job = null; this.infeed = null; this.phase = 'IDLE';
        }
        break;
    }
  }

  // The layer rolls off the deck onto the receiving conveyor of the palletizing cell.
  stepConveyors(dt) {
    const d = this.deck, front = AMR.body.xf + DST.gap / 2;
    if (d.u !== 0) {
      for (const b of d.boxes) b.lp[0] += d.u * dt;
      for (const b of d.boxes) if (b.lp[0] > front) {
        b.p = B.toWorld(this.pose, b.lp); b.yaw = this.pose.th + b.lyaw;
        this.infeed.receive(b);
      }
      d.boxes = d.boxes.filter((b) => b.st === 'deck');
    }
    for (const b of d.boxes) { b.p = B.toWorld(this.pose, b.lp); b.yaw = this.pose.th + b.lyaw; }
  }

  measure(dt) {
    super.measure(dt);
    const L = this.last, H = this.tcpHist;
    L.peel = Infinity;
    if (this.held && H.length === 3) {
      const a = [0, 1, 2].map((i) => (H[2][i] - 2 * H[1][i] + H[0][i]) / (dt * dt)), b = this.held.box;
      L.peel = PEEL_AVAIL / (b.m * hypot(a[0], a[1]) * (b.h / 2) + b.m * (G + max(a[2], 0)) * MTOOL_PEEL.rowY);
      this.stats.minPeel = min(this.stats.minPeel, L.peel);
      if (L.peel < MTOOL_PEEL.peelMin) { L.bad.push('peel safety factor'); if (L.bad.length === 1) this.stats.violations++; if (this.stats.firstBad.length < 12) this.stats.firstBad.push(`t=${this.t.toFixed(2)} peel ${L.peel.toFixed(2)}`); }
    }
  }
}
export { C as CLEAR };
