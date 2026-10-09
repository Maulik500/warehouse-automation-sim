// Cell model: roller conveyor line, turntable wrapper and its sequencer.
// Purely kinematic, fixed time step, no rendering dependencies.

import { PALLET, SKUS, LINE, TABLE, CARRIAGE, FILM, WRAP, CLAMP, CUT, DEG } from './config.js';
import { AxisMove } from '../../asrs/js/motion.js';
import * as K from './kinematics.js';

const { abs, min, max, PI, sqrt, ceil, floor } = Math;
const HX = PALLET.sx / 2;
const T = 3, N = LINE.zones.length; // zone index of the turntable; zone N is "gone"
export const zoneX = (i) => (i - T) * LINE.pitch;
const GAP = [TABLE.deckX + LINE.rollR, LINE.xEnd]; // |x| range without rollers on either side of the table

function rng(seed) {
  let s = seed;
  return () => ((s = (s * 16807) % 2147483647) / 2147483647);
}

// ---------- unit loads ----------
function makePallet(id, rand) {
  const k = floor(rand() * SKUS.length), S = SKUS[k], layers = S.layers[0] + floor(rand() * (S.layers[1] - S.layers[0] + 1));
  const boxes = [];
  for (let l = 0; l < layers; l++)
    for (let i = 0; i < S.nx; i++)
      for (let j = 0; j < S.ny; j++)
        boxes.push({ sx: S.sx, sy: S.sy, sz: S.sz, lx: (i - (S.nx - 1) / 2) * S.sx, ly: (j - (S.ny - 1) / 2) * S.sy, lz: PALLET.h + (l + 0.5) * S.sz });
  const H = PALLET.h + layers * S.sz;
  return {
    id, sku: S.sku, skuIndex: k, layers, boxes, H,
    Hmeas: ceil(H / LINE.heightRes - 1e-9) * LINE.heightRes, // what the light grid BH reports
    mass: PALLET.mass + boxes.length * S.mass, layerMass: S.nx * S.ny * S.mass,
    dx: (2 * rand() - 1) * LINE.stopTol, ey: (2 * rand() - 1) * LINE.guideTol, // where it stops / runs between the guides
    off: [0, 0], x: 0, v: 0, zone: -1, from: -1, move: null, tRest: 0,
    wx: 0, wy: 0, yaw: 0, wrapped: false, film: null, cols: K.columns(boxes),
  };
}

export class Cell {
  constructor({ seed = 7 } = {}) {
    this.rand = rng(seed);
    this.t = 0;
    this.pallets = new Set();
    this.nPallets = 0;
    this.tSpawn = 2;
    this.roll = new Array(N).fill(0); // roller surface travel of every zone (zone 3 = deck = q2)
    this.w = {
      phase: 'IDLE', stage: 0, label: 'waiting for a pallet', pallet: null, plan: null, t0: 0, tEnd: 0, bad: [],
      th: 0, thBase: 0, om: 0, al: 0, // q1 (th restarts at 0 every cycle; thBase + th is the total angle)
      deckV: 0, // q2 speed
      lift: K.Z_BOTTOM, liftV: 0, liftMove: null, // q3
      feed: 0, vFeed: 0, integ: 0, // q4: film paid out, pre-stretch speed
      dancer: 0, tension: FILM.F, // q5: film stored in the dancer (0 = mid position)
      vFilm: 0, arm: 0, alpha: K.homeFilmLine().alpha, free: 0, // what the load takes
      rollLen: FILM.length, rollAngle: 0,
      clampUp: 1, upT: 1, jaw: 1, jawT: 1, // g1, g2 and their targets
      slide: CUT.park, slideV: 0, slideMove: null, wire: false, // g3
      supply: true, // the film end is in the clamp
      filmOnLoad: false, released: false, feed0: 0,
    };
    this.stats = {
      wrapped: 0, violations: 0, rolls: 0, cycles: [], tLast: 0, period: [],
      om: 0, al: 0, liftV: 0, deckV: 0, vFeed: 0, aFeed: 0, dancer: 0, tension: [Infinity, -Infinity],
      alpha: [Infinity, -Infinity], arm: [Infinity, -Infinity], sf: Infinity, swept: 0, queue: 0, slideV: 0,
    };
  }

  // ---------- line ----------
  // Hooks for an integrating system: may a pallet leave the last zone, and what happens to it.
  exitFree() { return true; }
  leave(p) { this.pallets.delete(p); }
  at(i) { for (const p of this.pallets) if (p.zone === i) return p; return null; }
  resting(i) { const p = this.at(i); return p && !p.move ? p : null; }
  startMove(p, to) {
    p.move = new AxisMove(p.x, zoneX(to) + p.dx, LINE.move, this.t);
    p.from = p.zone; p.zone = to; p.tRest = 0;
  }
  get b1() { return !!this.resting(T - 1); } // pallet at the infeed stop
  get ready() { const p = this.resting(T - 1); return !!p && p.tRest >= LINE.tDebounce; }
  gapEye(sign) { for (const p of this.pallets) if (sign * p.x + HX > GAP[0] && sign * p.x - HX < GAP[1]) return true; return false; }
  get b2() { return this.gapEye(-1); } // pallet across the infeed gap
  get b3() { return !!this.resting(T); } // pallet centred on the deck
  get b4() { return this.gapEye(1); } // pallet across the outfeed gap
  get b5() { return !!this.at(T + 1); } // first outfeed zone occupied

  spawn(dt) {
    this.tSpawn -= dt;
    if (this.tSpawn > 0 || this.at(0) || this.at(-1)) return;
    const p = makePallet(++this.nPallets, this.rand);
    p.x = zoneX(-1) + p.dx;
    this.pallets.add(p);
    this.startMove(p, 0);
    this.tSpawn = LINE.spawn[0] + this.rand() * (LINE.spawn[1] - LINE.spawn[0]);
  }

  // ---------- sequencer (spec §8) ----------
  sequence() {
    const w = this.w, t = this.t;
    // line zones: a pallet moves on as soon as the zone ahead is released
    for (const i of [6, 5, 4, 1, 0]) { const p = this.resting(i); if (p && (i === N - 1 ? this.exitFree() : !this.at(i + 1))) this.startMove(p, i + 1); }
    const onT = this.resting(T), i1 = this.ready ? this.resting(T - 1) : null, rollLow = w.rollLen < FILM.minRoll;
    if (w.phase === 'IDLE') {
      if (rollLow) { w.phase = 'ROLL'; w.tEnd = t + FILM.tRollChange; }
      else if (i1) { this.startMove(i1, T); w.phase = 'TRANSFER'; }
    } else if (w.phase === 'OUT') {
      if (!this.b5) { this.startMove(onT, T + 1); if (i1 && !rollLow) this.startMove(i1, T); w.phase = 'TRANSFER'; }
    } else if (w.phase === 'TRANSFER') {
      let busy = false;
      for (const p of this.pallets) if (p.move && (p.zone === T || p.from === T)) busy = true;
      if (!busy && !this.b2 && !this.b4) { if (onT) this.startWrap(onT); else w.phase = 'IDLE'; }
    } else if (w.phase === 'ROLL') {
      if (t >= w.tEnd) { w.rollLen = FILM.length; this.stats.rolls++; w.phase = 'IDLE'; }
    } else if (w.phase === 'CLAMP') {
      if (w.clampUp === 1 && w.jaw === 1) { w.supply = true; w.slideMove = new AxisMove(CUT.park, CUT.press, CUT.move, t); w.phase = 'CUT'; w.stage = 0; }
    } else if (w.phase === 'CUT') {
      if (w.stage === 0 && w.slideMove.done(t)) { w.slide = CUT.press; w.wire = true; w.tEnd = t + CUT.tWire; w.stage = 1; }
      else if (w.stage === 1 && t >= w.tEnd) { this.cut(); w.tEnd = t + CUT.tPress; w.stage = 2; }
      else if (w.stage === 2 && t >= w.tEnd) { w.slideMove = new AxisMove(CUT.press, CUT.park, CUT.move, t); w.stage = 3; }
      else if (w.stage === 3 && w.slideMove.done(t)) { w.slideMove = null; w.slide = CUT.park; w.phase = 'OUT'; }
    }
  }
  startWrap(p) {
    const w = this.w;
    p.off = [p.dx, p.ey];
    p.film = new K.FilmWrap(p.off, K.Z_BOTTOM);
    Object.assign(w, { phase: 'WRAP', stage: 0, pallet: p, plan: K.wrapPlan(p.Hmeas), t0: this.t, released: false, filmOnLoad: true, supply: false, liftMove: null, feed0: w.feed, integ: 0 });
    w.turn = new AxisMove(0, 2 * PI * w.plan.revs, TABLE.wrap, this.t);
  }
  cut() {
    const w = this.w, p = w.pallet, s = this.stats;
    p.film.finish();
    p.wrapped = true;
    w.wire = false; w.filmOnLoad = false;
    s.wrapped++;
    s.cycles.push({ id: p.id, H: p.H, revs: w.plan.revs, t: this.t - w.t0, film: w.feed - w.feed0, laid: p.film.laid });
    if (s.tLast) s.period.push(this.t - s.tLast);
    s.tLast = this.t;
  }

  // ---------- motion ----------
  moveLine(dt) {
    const w = this.w, zv = new Array(N).fill(0);
    for (const p of this.pallets) {
      if (!p.move) { p.tRest += dt; p.v = 0; continue; }
      const q = p.move.at(this.t);
      p.x = q.p; p.v = q.v;
      for (const i of [p.from, p.zone]) { const k = min(N - 1, max(0, i)); zv[k] = max(zv[k], q.v); }
      if (p.move.done(this.t)) { p.x = p.move.to; p.v = 0; p.move = null; if (p.zone === N) this.leave(p); }
    }
    for (let i = 0; i < N; i++) this.roll[i] += zv[i] * dt;
    w.deckV = zv[T];
  }
  wrapStep() {
    const w = this.w, t = this.t;
    if (w.phase !== 'WRAP') return;
    const P = w.plan, tt = t - w.t0, th0 = w.th, z0 = w.lift, film = w.pallet.film;
    const q = w.turn.at(t);
    w.th = q.p; w.om = q.v; w.al = q.a;
    if (!w.released && tt >= P.tRelease) { w.released = true; w.jawT = 0; w.upT = 0; }
    if (w.stage === 0 && tt >= P.t1) { w.liftMove = new AxisMove(P.bottom, P.top, K.liftLimits(WRAP.overlapUp), w.t0 + P.t1); w.stage = 1; }
    if (w.stage === 1 && tt >= P.t3) { w.liftMove = new AxisMove(P.top, P.bottom, K.liftLimits(WRAP.overlapDown), w.t0 + P.t3); w.stage = 2; }
    if (w.liftMove) { const l = w.liftMove.at(t); w.lift = l.p; w.liftV = l.v; }
    film.advance(th0, z0, w.th, w.lift);
    if (w.turn.done(t)) {
      w.thBase += w.turn.to;
      Object.assign(w, { th: 0, om: 0, al: 0, liftMove: null, liftV: 0, lift: P.bottom, phase: 'CLAMP', upT: 1, jawT: 1 });
    }
  }
  gripper(dt) {
    const w = this.w;
    // pneumatic strokes at constant speed: up before closing, open before going down
    if (w.upT === 1) { if (w.clampUp < 1) w.clampUp = min(1, w.clampUp + dt / CLAMP.tLift); else if (w.jaw !== w.jawT) w.jaw = w.jawT > w.jaw ? min(1, w.jaw + dt / CLAMP.tJaw) : max(0, w.jaw - dt / CLAMP.tJaw); }
    else if (w.jaw > 0) w.jaw = max(0, w.jaw - dt / CLAMP.tJaw);
    else w.clampUp = max(0, w.clampUp - dt / CLAMP.tLift);
    if (w.filmOnLoad && w.jaw === 0 && w.pallet.film.tailHeld) w.pallet.film.release();
    if (w.slideMove) { const q = w.slideMove.at(this.t); w.slide = q.p; w.slideV = q.v; } else w.slideV = 0;
  }
  // Film demand of the load, pre-stretch drive and dancer (spec §5.2, §5.4)
  feedStep(dt) {
    const w = this.w, F = FILM.feed;
    if (w.filmOnLoad) { const l = w.pallet.film.line(w.th); w.arm = l.arm; w.alpha = l.alpha; w.free = l.len; w.vFilm = w.om * l.arm; }
    else { w.arm = 0; w.vFilm = 0; w.free = 0; w.alpha = K.homeFilmLine().alpha; }
    w.integ = w.om > 0 ? w.integ - w.dancer * dt : 0;
    const target = w.om * K.ARM_MEAN - F.kp * w.dancer + F.ki * w.integ;
    const a = min(F.aMax, max(-F.aMax, (target - w.vFeed) / F.tau));
    w.aFeed = a;
    w.vFeed = max(0, w.vFeed + a * dt);
    w.dancer += (w.vFeed - w.vFilm) * dt;
    w.tension = FILM.F - FILM.dancer.k * w.dancer;
    w.feed += w.vFeed * dt;
    w.rollLen -= (w.vFeed * dt) / FILM.lambda;
    w.rollAngle += (w.vFeed * dt) / FILM.lambda / this.rollRadius;
  }
  get rollRadius() { return sqrt(FILM.rCore ** 2 + (max(0, this.w.rollLen) * FILM.t0) / PI); }

  snapshot() {
    const w = this.w, p = this.at(T) && !this.at(T).move ? this.at(T) : null;
    return {
      th: w.th, om: w.om, al: w.al, thCycle: w.th, deckV: w.deckV, lift: w.lift, liftV: w.liftV,
      slide: w.slide, clampUp: w.clampUp, jaw: w.jaw, b2: this.b2, b4: this.b4,
      filmOnLoad: w.filmOnLoad, corners: w.filmOnLoad ? w.pallet.film.corners : 0,
      dancer: w.dancer, tension: w.tension, vFeed: w.vFeed,
      load: p ? { off: [p.dx, p.ey], sf: K.loadStability(p.cols, [p.dx, p.ey], w.om, w.al).sf } : null,
    };
  }
  check() {
    const w = this.w, s = this.stats, snap = this.snapshot();
    w.bad = K.monitor(snap);
    if (w.bad.length) s.violations++;
    w.sf = snap.load ? snap.load.sf : Infinity;
    s.om = max(s.om, w.om); s.al = max(s.al, abs(w.al)); s.liftV = max(s.liftV, abs(w.liftV)); s.deckV = max(s.deckV, w.deckV);
    s.vFeed = max(s.vFeed, w.vFeed); s.aFeed = max(s.aFeed, abs(w.aFeed)); s.dancer = max(s.dancer, abs(w.dancer)); s.slideV = max(s.slideV, abs(w.slideV));
    if (snap.load) { s.sf = min(s.sf, snap.load.sf); s.swept = max(s.swept, K.sweptRadius(snap.load.off)); }
    if (w.filmOnLoad && w.om > 0) {
      s.tension = [min(s.tension[0], w.tension), max(s.tension[1], w.tension)];
      s.alpha = [min(s.alpha[0], w.alpha), max(s.alpha[1], w.alpha)];
      s.arm = [min(s.arm[0], w.arm), max(s.arm[1], w.arm)];
    }
    let n = 0;
    for (const p of this.pallets) if (p.zone < T) n++;
    s.queue = max(s.queue, n);
  }
  label() {
    const w = this.w, P = w.plan, tt = this.t - w.t0;
    switch (w.phase) {
      case 'IDLE': return 'waiting for a pallet';
      case 'TRANSFER': return 'pallet transfer';
      case 'WRAP': return tt < P.t1 ? (w.pallet.film.corners < 1 ? 'start: pulling the film tail' : 'bottom wraps') : tt < P.t2 ? 'spiral up' : tt < P.t3 ? 'top wraps' : tt < P.t4 ? 'spiral down' : this.w.om < TABLE.wrap.v ? 'stopping at home' : 'final bottom wraps';
      case 'CLAMP': return 'clamp: up and close';
      case 'CUT': return ['cut slide in', 'hot wire', 'pressing the tail', 'cut slide out'][w.stage];
      case 'OUT': return 'waiting for the outfeed';
      case 'ROLL': return 'film roll change';
    }
  }

  step(dt) {
    this.t += dt;
    this.spawn(dt);
    this.sequence();
    this.moveLine(dt);
    this.wrapStep();
    this.gripper(dt);
    this.feedStep(dt);
    // world pose of every pallet: on the turntable it turns with q1
    for (const p of this.pallets) {
      if (p.zone === T && !p.move) { [p.wx, p.wy] = K.rot(this.w.th, [p.dx, p.ey]); p.yaw = this.w.th; }
      else { p.wx = p.x; p.wy = p.ey; p.yaw = 0; }
    }
    this.check();
  }
}

export { T as ZONE_T, DEG };
