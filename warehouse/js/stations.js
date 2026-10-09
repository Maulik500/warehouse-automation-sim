// Floor stations (spec §8.6) and the wrapper adapter (spec §4.4, §8.5).
// A station is what a forklift docks at: { id, kind, x, y, th, pickZs(), dropZs(), pickReady,
// dropReady, lock(), unlock(), presented(), take(), accept(), frames }.

import { Cell as WrapCell, zoneX, ZONE_T } from '../../wrapper/js/sim.js';
import { LINE } from '../../wrapper/js/config.js';
import * as WK from '../../wrapper/js/kinematics.js';
import { PALLET, PLACE, WRAPX } from './config.js';
import { placeBoxes } from './asrs.js';

const { abs, cos, sin, ceil, max, PI } = Math;
const open = { lock() { return true; }, unlock() {}, locked: null, frames: [] };

export function floorSlot(def) {
  return {
    ...open, ...def, kind: 'slot', pallet: null, reserved: false, tIn: 0,
    zs: () => 0, pickZs: () => 0, dropZs: () => 0,
    get pickReady() { return !!this.pallet; }, get dropReady() { return !this.pallet; },
    presented() { return this.pallet; },
    take() { this.pallet = null; },
    accept(p, t) { this.pallet = p; this.tIn = t; p.holder = this.id; },
  };
}
export function stack(def) {
  return {
    ...open, ...def, kind: 'stack', pallets: [], reservedIn: 0, reservedOut: 0,
    zs() { return this.pallets.length * PALLET.h; },
    pickZs() { return (this.pallets.length - 1) * PALLET.h; }, dropZs() { return this.pallets.length * PALLET.h; },
    get pickReady() { return this.pallets.length > 0; }, get dropReady() { return this.pallets.length < this.max; },
    presented() { return this.pallets[this.pallets.length - 1] ?? null; },
    take() { this.pallets.pop(); this.reservedOut = max(0, this.reservedOut - 1); },
    accept(p) { this.pallets.push(p); p.holder = this.id; this.reservedIn = max(0, this.reservedIn - 1); },
  };
}
export const parking = (def) => ({ ...open, ...def, kind: 'home' });

// ---------- belly wrapper ----------
// The verified wrapper cell with its arrival timer replaced by a hand-over zone I4 and its
// vanishing outfeed replaced by a hand-over zone O4 (zones −1 and 7 of its line).
const N = LINE.zones.length;
export class Wrapper extends WrapCell {
  constructor(world) {
    super({ seed: 7 });
    this.world = world; this.P = PLACE.wrap;
    const W = this, P = this.P, hw = WRAPX.handFrameW / 2;
    const frame = (x) => [{ poly: [[x - 0.9, -hw], [x + 0.9, -hw], [x + 0.9, hw], [x - 0.9, hw]].map(([a, b]) => [P.tx + a, P.ty + b]), top: LINE.z }];
    this.i4Lock = null; this.o4Lock = null;
    this.I4 = {
      id: 'I4', kind: 'pick', x: P.tx + WRAPX.i4, y: P.ty, th: -PI / 2, frames: frame(WRAPX.i4), reserved: false,
      zs: () => LINE.z, get pickReady() { return false; },
      get dropReady() { return !W.at(-1); },
      get locked() { return W.i4Lock; },
      lock(who) { if (W.i4Lock && W.i4Lock !== who) return false; W.i4Lock = who; return true; },
      unlock(who) { if (W.i4Lock === who) W.i4Lock = null; },
      presented() { return null; }, take() {},
      accept(p) { W.deposit(p); },
    };
    this.O4 = {
      id: 'O4', kind: 'pick', x: P.tx + WRAPX.o4, y: P.ty, th: -PI / 2, frames: frame(WRAPX.o4), reserved: false,
      zs: () => LINE.z, get dropReady() { return false; },
      get pickReady() { const w = W.resting(N); return !!w && w.tRest >= LINE.tDebounce; },
      get locked() { return W.o4Lock; },
      lock(who) { if (W.o4Lock && W.o4Lock !== who) return false; W.o4Lock = who; return true; },
      unlock(who) { if (W.o4Lock === who) W.o4Lock = null; },
      presented() { const w = W.resting(N); return w ? w.src : null; },
      take(p) { for (const w of W.pallets) if (w.src === p) W.pallets.delete(w); },
      accept() {},
    };
  }
  spawn() {}
  exitFree() { return !this.at(N) && !this.o4Lock; } // nothing rolls onto O4 while a forklift is in it
  leave(p) { p.tRest = 0; } // stays on O4 until a forklift takes it
  // A forklift has set a mixed pallet down on I4.
  deposit(p) {
    const P = this.P, c = cos(-p.yaw), s = sin(-p.yaw);
    // box extents in the wrapper's pallet frame (X_p = conveying direction)
    const flip = abs(cos(p.yaw)) < 0.5 ? null : cos(p.yaw) < 0 ? -1 : 1;
    if (flip === null) throw new Error('pallet arrives crosswise on the wrapper line');
    const boxes = p.boxes.map((b) => {
      const a = b.lyaw, ex = abs(cos(a)) * b.sx + abs(sin(a)) * b.sy, ey = abs(sin(a)) * b.sx + abs(cos(a)) * b.sy;
      return { sx: ex, sy: ey, sz: b.h, lx: flip * b.lp[0], ly: flip * b.lp[1], lz: b.lp[2] };
    });
    const top = p.boxes.filter((b) => b.layer === undefined || true).reduce((m, b) => max(m, b.lp[2] + b.h / 2), 0);
    const topLayer = p.boxes.filter((b) => abs(b.lp[2] + b.h / 2 - top) < 1e-6);
    const w = {
      id: p.id, sku: 'MIX', boxes, H: top, Hmeas: ceil(top / LINE.heightRes - 1e-9) * LINE.heightRes,
      mass: p.mass, layerMass: topLayer.reduce((m, b) => m + b.m, 0),
      dx: p.x - (P.tx + WRAPX.i4), ey: p.y - P.ty, off: [0, 0], x: 0, v: 0, zone: -1, from: -1, move: null, tRest: 0,
      wx: 0, wy: 0, yaw: 0, wrapped: false, film: null, cols: WK.columns(boxes), src: p, yaw0: p.yaw,
    };
    void c; void s;
    w.x = zoneX(-1) + w.dx;
    this.pallets.add(w);
    p.holder = 'WRAP'; p.wrap = w;
  }
  sequence() {
    const p = this.resting(-1);
    if (p && !this.i4Lock && !this.at(0)) this.startMove(p, 0);
    super.sequence();
  }
  step(dt) {
    super.step(dt);
    const P = this.P;
    for (const w of this.pallets) {
      const p = w.src;
      p.x = P.tx + w.wx; p.y = P.ty + w.wy; p.z = LINE.z; p.yaw = w.yaw0 + w.yaw; p.dom = 'g';
      p.wrapped = w.wrapped;
      if (w.zone === N && !w.move) p.holder = 'O4';
      placeBoxes(p);
    }
  }
}
export { ZONE_T, N as ZONE_OUT };
