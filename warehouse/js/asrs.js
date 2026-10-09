// ASRS for the integrated warehouse (spec §4.1, §8.1): the verified crane (Aisle of
// asrs/js/sim.js) with five bidirectional row lanes instead of in / out lanes. A pallet goes out
// to its row's pick port, a layer is taken, and it comes back in on the same lane.
// Pallets inside the ASRS carry ASRS-cell coordinates (dom 'asrs'); a pallet standing at a port
// carries global coordinates (dom 'g'), because that is where vehicles meet it.

import { Aisle } from '../../asrs/js/sim.js';
import { RACK, PD, LAYOUT } from '../../asrs/js/config.js';
import { AxisMove } from '../../asrs/js/motion.js';
import { palletBoxes, layerExtent } from '../../mobman/js/recipes.js';
import { PALLET, PORT, PLACE, ROW_SKU, DEG } from './config.js';

const { max, min, abs, cos, sin, floor } = Math;
const TX = PLACE.asrs.tx, TY = PLACE.asrs.ty;
const Z_CONV = PD.z + PD.deckLift;
const LANE_LIM = { v: PORT.v, a: PORT.a, j: PORT.j };
const YAW0 = -90 * DEG; // pallet X_p (the 1200 length) points to −Y of {G} everywhere in the ASRS

let BOX_ID = 0;
// Turn a pallet's layer count into tracked boxes (pick order: top layer first).
export function materialise(p) {
  const S = p.skuDef, rc = { ...S.rc, layers: p.layers };
  p.rc = rc; p.extent = layerExtent(rc); p.next = 0;
  p.boxes = palletBoxes(rc).map((b) => ({
    id: ++BOX_ID, sku: S.id, sx: S.l, sy: S.w, h: S.h, m: S.mass, st: 'pallet', layer: b.layer,
    lp: [b.x, b.y, b.z], lyaw: b.yaw, p: [0, 0, 0], yaw: 0, pal: p,
  }));
}
export function setMass(p) {
  const n = p.boxes ? p.boxes.length : p.layers * p.skuDef.perLayer;
  p.mass = PALLET.mass + n * p.skuDef.mass;
  p.h = PALLET.h + p.layers * p.skuDef.h;
  p.cgz = p.layers ? (PALLET.mass * PALLET.h / 2 + n * p.skuDef.mass * (PALLET.h + (p.layers * p.skuDef.h) / 2)) / p.mass : PALLET.h / 2;
}
// Global pose of a pallet and of the boxes on it.
export function palletG(p) { return p.dom === 'asrs' ? [p.x + TX, p.y + TY, p.z] : [p.x, p.y, p.z]; }
export function placeBoxes(p) {
  if (!p.boxes) return;
  const [gx, gy, gz] = palletG(p), c = cos(p.yaw), s = sin(p.yaw);
  for (const b of p.boxes) {
    if (b.st !== 'pallet') continue;
    b.p = [gx + c * b.lp[0] - s * b.lp[1], gy + s * b.lp[0] + c * b.lp[1], gz + b.lp[2]];
    b.yaw = p.yaw + b.lyaw;
  }
}

// One row lane with its station and its pick port.
class Lane {
  constructor(aisle, row, st) {
    Object.assign(this, { aisle, row, st, sku: ROW_SKU[row], pallet: null, claim: null, state: 'IDLE', move: null, stop: 0, t: 0, locked: null, drift: [0, 0, 0], done: false });
    this.id = `PORT-${row}`;
    this.kind = 'pick'; // for the forklift monitor: a station that must be Locked
    this.x = TX + PORT.x; this.y = TY + st.y; this.th = 0; // pallet centre and forklift heading in {G}
    const xo = this.x - PALLET.sy / 2 - PORT.frameOver, x1 = TX + PD.x - 0.6, hw = PORT.frameW / 2;
    this.frame = { x0: xo, x1, y: this.y, hw, top: PORT.z };
    this.frames = [{ poly: [[xo, this.y - hw], [x1, this.y - hw], [x1, this.y + hw], [xo, this.y + hw]], top: PORT.z }];
    this.reserved = false;
  }
  get sys() { return this.aisle.sys; }
  zs() { return PORT.z; }
  support() { return PORT.z; }
  get atPort() { return this.state === 'PORT' || this.state === 'EMPTY'; }
  // PalletReady for the robot AMR: a pallet with boxes stands still at the port
  get ready() { return this.state === 'PORT' && this.t >= PORT.tDebounce && this.pallet.boxes.length > 0; }
  // forklift side
  get pickReady() { return this.state === 'EMPTY' && this.t >= PORT.tDebounce; }
  get dropReady() { return this.state === 'DEPOSIT' && this.st.deck === 1 && this.stop === 0; }
  presented() { return this.atPort ? this.pallet : null; }
  lock(who) { if (this.locked && this.locked !== who) return false; this.locked = who; return true; }
  unlock(who) { if (this.locked === who) this.locked = null; }
  take(p) { if (this.pallet === p) { this.pallet = null; this.state = 'CLEAR'; this.t = 0; } } // a forklift lifts the empty pallet out
  accept(p) { // a forklift has set a replenishment pallet down on the port
    this.pallet = p; p.dom = 'g'; p.holder = this.id;
    this.drift = [p.x - this.x, p.y - this.y, p.yaw - YAW0];
    this.state = 'DEPOSITED'; this.t = 0;
  }

  toPort(p) { p.dom = 'g'; p.x = this.x + this.drift[0]; p.y = this.y + this.drift[1]; p.z = Z_CONV; p.yaw = YAW0 + this.drift[2]; }
  step(dt) {
    const a = this.aisle, st = this.st, p = this.pallet, tNow = a.t;
    this.t += dt;
    const deckTo = (v) => { if (a.forkCentred && !a.engaged) st.deck = v > st.deck ? min(v, st.deck + dt / PD.tDeck) : max(v, st.deck - dt / PD.tDeck); };
    const deckZ = () => max(st.zs, st.zs - PD.deckDrop + st.deck * (PD.deckDrop + PD.deckLift));
    const stopTo = (v) => { this.stop = v > this.stop ? min(v, this.stop + dt / PORT.tStop) : max(v, this.stop - dt / PORT.tStop); };
    const roll = () => {
      const q = this.move.at(tNow), u = (PD.x - q.p) / (PD.x - PORT.x); // 0 at the station, 1 at the port
      p.x = q.p + u * this.drift[0]; p.y = st.y + u * this.drift[1]; p.z = Z_CONV; p.yaw = YAW0 + u * this.drift[2];
      return this.move.done(tNow);
    };
    switch (this.state) {
      case 'IDLE': deckTo(0); stopTo(0); break;
      case 'RETR': // the crane is bringing the pallet; it reports with `done`
        if (this.done && st.pallet === p) { this.state = 'DECK_UP'; this.done = false; }
        break;
      case 'DECK_UP':
        deckTo(1); p.z = deckZ();
        if (st.deck === 1) {
          st.pallet = null;
          const r = () => 2 * this.sys.rand() - 1;
          this.drift = [PORT.scatter * r(), PORT.scatter * r(), PORT.scatterYaw * r()];
          this.move = new AxisMove(PD.x, PORT.x, LANE_LIM, tNow); this.state = 'OUT';
        }
        break;
      case 'OUT':
        if (roll()) { this.move = null; this.toPort(p); this.state = 'STOPS_UP'; }
        break;
      case 'STOPS_UP':
        stopTo(1);
        if (this.stop === 1) { this.state = 'PORT'; this.t = 0; }
        break;
      case 'PORT': case 'EMPTY': break; // waiting for a vehicle
      case 'RETURN': // boxes left: back into the rack
        stopTo(0);
        if (this.stop === 0 && st.deck === 1) {
          p.dom = 'asrs'; p.x -= TX; p.y -= TY;
          this.move = new AxisMove(PORT.x, PD.x, LANE_LIM, tNow); this.state = 'IN';
        } else deckTo(1);
        break;
      case 'IN':
        if (roll()) { this.move = null; p.x = PD.x; p.y = st.y; p.yaw = YAW0; this.state = 'DECK_DN'; }
        break;
      case 'DECK_DN':
        deckTo(0); p.z = deckZ();
        if (st.deck === 0) { st.pallet = p; this.state = 'AT_ST'; a.cmds.push({ kind: 'store', lane: this }); }
        break;
      case 'AT_ST': case 'STORING': break;
      case 'CLEAR': // empty pallet taken away by a forklift
        if (!this.locked) { stopTo(0); deckTo(0); if (this.stop === 0 && st.deck === 0) this.state = 'IDLE'; }
        break;
      case 'DEPOSIT': deckTo(1); stopTo(0); break; // waiting for a replenishment pallet
      case 'DEPOSITED':
        if (!this.locked) { this.state = 'RETURN'; }
        break;
    }
  }
}

class RowAisle extends Aisle {
  constructor(sys, def) {
    super(sys, def);
    this.sys = sys;
    this.cmds = [];
    this.stIn.deck = 0;
    this.lanes = def.rows.map(([row, side]) => new Lane(this, row, side > 0 ? this.stIn : this.stOut));
  }
  // Crane commands: put returned pallets away first (that frees a lane), then retrieve.
  schedule() {
    const i = this.cmds.findIndex((c) => c.kind === 'store') >= 0 ? this.cmds.findIndex((c) => c.kind === 'store') : this.cmds.findIndex((c) => c.kind === 'retrieve');
    if (i < 0) return false;
    const c = this.cmds.splice(i, 1)[0], lane = c.lane;
    if (c.kind === 'store') {
      const p = lane.st.pallet, free = this.slots.filter((s) => s.row === lane.row && !s.pallet && !s.reserved);
      const slot = free[floor(this.sys.rand() * free.length)];
      slot.reserved = true;
      lane.state = 'STORING';
      this.op = { kind: 'store', slot, pallet: p, t0: this.t };
      this.steps = [...this.pickSteps(lane.st, p), ...this.putSteps(slot, p), { k: 'do', fn: () => {
        this.stats.stored++;
        for (const b of p.boxes) this.sys.boxes.delete(b);
        p.boxes = null; p.holder = slot.id; lane.pallet = null; lane.state = 'IDLE';
      } }];
    } else {
      const slot = c.slot, p = slot.pallet;
      this.op = { kind: 'retrieve', slot, pallet: p, t0: this.t };
      this.steps = [...this.pickSteps(slot, p), { k: 'do', fn: () => { slot.reserved = false; } }, ...this.putSteps(lane.st, p), { k: 'do', fn: () => { this.stats.retrieved++; lane.done = true; } }];
    }
    this.stepIdx = 0;
    this.begin();
    return true;
  }
  conveyors(dt) { for (const l of this.lanes) l.step(dt); }
}

export class Asrs {
  constructor({ rand }) {
    this.rand = rand;
    this.t = 0;
    this.inbound = 'external';
    this.nextId = 1;
    this.pallets = new Set();
    this.boxes = new Set(); // tracked boxes of pallets that are out of their rack location
    this.counts = { boxesIn: 0, palletsIn: 0, emptiesOut: 0, replenished: 0 };
    this.aisles = LAYOUT.aisles.map((def) => new RowAisle(this, def));
    this.lanes = {};
    for (const a of this.aisles) for (const l of a.lanes) this.lanes[l.row] = l;
    for (const a of this.aisles)
      for (const s of a.slots)
        if (this.rand() < PORT.fill) {
          const S = ROW_SKU[s.row], n = S.srcLayers <= 2 ? S.srcLayers : 2 + floor(this.rand() * (S.srcLayers - 1));
          a.place(this.makePallet(S, n), s);
        }
  }
  makePallet(S, layers) {
    const p = { id: `A${this.nextId++}`, kind: 'src', sku: S.id, skuDef: S, layers, boxes: null, x: 0, y: 0, z: 0, yaw: YAW0, dom: 'asrs', slot: null, holder: null };
    setMass(p);
    this.pallets.add(p);
    this.counts.boxesIn += layers * S.perLayer;
    this.counts.palletsIn++;
    return p;
  }
  rowSlots(row) { return this.lanes[row].aisle.slots.filter((s) => s.row === row); }
  fill(row) { const S = this.rowSlots(row); return S.filter((s) => s.pallet || s.reserved).length / S.length; }
  // layers of a row that are in the rack and not on their way out
  stock(row) { return this.rowSlots(row).reduce((n, s) => n + (s.pallet && !s.reserved ? s.pallet.layers : 0), 0); }
  // Order a pallet of this row to its pick port. Returns the pallet, or null if the lane is busy or the row is empty.
  retrieve(row) {
    const lane = this.lanes[row];
    if (lane.state !== 'IDLE' || lane.reserved) return null;
    const cand = this.rowSlots(row).filter((s) => s.pallet && !s.reserved && s.pallet.layers > 0);
    if (!cand.length) return null;
    const slot = cand.reduce((a, b) => (b.pallet.layers < a.pallet.layers ? b : a));
    const p = slot.pallet;
    slot.reserved = true;
    materialise(p);
    for (const b of p.boxes) this.boxes.add(b);
    lane.pallet = p; lane.state = 'RETR'; lane.done = false; p.holder = lane.id;
    lane.aisle.cmds.push({ kind: 'retrieve', lane, slot });
    return p;
  }
  // A robot AMR wants one layer of this row: use the pallet that is already at the port, or
  // order one from the rack. False while the lane is busy with something else.
  request(row, who) {
    const lane = this.lanes[row];
    if (lane.claim || lane.reserved) return false;
    if (lane.state === 'PORT' && !lane.locked && lane.pallet.boxes.length) { lane.claim = who; return true; }
    if (!this.retrieve(row)) return false;
    lane.claim = who;
    return true;
  }
  // Nobody needs the pallet that waits at the port any more: back into the rack.
  sendBack(row) {
    const lane = this.lanes[row];
    if (lane.state === 'PORT' && !lane.claim && !lane.locked) lane.state = 'RETURN';
  }
  // The robot AMR has finished at the port: boxes it took are off the pallet. `keep`: more of
  // this SKU is on order, so the pallet stays at the port instead of going back into the rack.
  release(row, keep = false) {
    const lane = this.lanes[row], p = lane.pallet;
    lane.claim = null;
    p.boxes = p.boxes.filter((b) => b.st === 'pallet');
    p.next = 0;
    p.layers = Math.round(p.boxes.length / p.skuDef.perLayer);
    p.rc = { ...p.rc, layers: p.layers };
    setMass(p);
    if (p.boxes.length) { lane.state = keep ? 'PORT' : 'RETURN'; lane.t = 0; }
    else { p.kind = 'empty'; p.sku = null; lane.state = 'EMPTY'; lane.t = 0; }
  }
  // A forklift is going to bring a replenishment pallet to this row's port.
  reserveDeposit(row) {
    const lane = this.lanes[row];
    if (lane.state !== 'IDLE' || lane.reserved) return false;
    lane.state = 'DEPOSIT';
    return true;
  }
  // every pallet that is not sitting in a rack location (for drawing and for box poses)
  active() { const out = []; for (const a of this.aisles) { if (a.carried) out.push(a.carried); for (const l of a.lanes) if (l.pallet && !out.includes(l.pallet)) out.push(l.pallet); } return out; }
  step(dt) {
    this.t += dt;
    for (const a of this.aisles) a.step(dt);
    for (const p of this.active()) placeBoxes(p);
  }
  get violations() { return this.aisles.reduce((s, a) => s + a.stats.violations, 0); }
}
export { TX as ASRS_TX, TY as ASRS_TY, YAW0 as PALLET_YAW0, Z_CONV };
