// Traffic control (spec §7): every vehicle holds a claim — the plan area it stands on plus the
// area its current run sweeps. A run starts only when its whole sweep is free of every other
// claim; if only a first part is free, the run ends at the last permitted stopping point of that
// part. Stops are always on straight pieces. No rendering dependencies.

import { AxisMove } from '../../asrs/js/motion.js';

const { abs, min, max, cos, sin, PI, ceil } = Math;

// Outline (for the fleet monitor) and claim rectangle (outline + margin) in the body frame.
export const SHAPE = {
  fork: { out: { x0: -1.55, x1: 1.27, hw: 0.60 }, claim: { x0: -1.75, x1: 1.50, hw: 0.80 }, rSpin: 0 },
  mm: { out: { x0: -0.75, x1: 0.75, hw: 0.62 }, claim: { x0: -0.95, x1: 0.95, hw: 0.82 }, rSpin: 1.30 },
};
export function rect(pose, r) {
  const c = cos(pose.th), s = sin(pose.th);
  return [[r.x0, -r.hw], [r.x1, -r.hw], [r.x1, r.hw], [r.x0, r.hw]].map(([x, y]) => [pose.x + c * x - s * y, pose.y + s * x + c * y]);
}
export function disc(pose, r, n = 8) {
  const R = r / cos(PI / n);
  return Array.from({ length: n }, (_, i) => [pose.x + R * cos((2 * PI * (i + 0.5)) / n), pose.y + R * sin((2 * PI * (i + 0.5)) / n)]);
}
const box = (P) => {
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (const p of P) { x0 = min(x0, p[0]); x1 = max(x1, p[0]); y0 = min(y0, p[1]); y1 = max(y1, p[1]); }
  return [x0, x1, y0, y1];
};
// Do two convex polygons overlap? (separating axis test)
export function overlap(A, B) {
  for (const P of [A, B])
    for (let i = 0; i < P.length; i++) {
      const a = P[i], b = P[(i + 1) % P.length], nx = b[1] - a[1], ny = a[0] - b[0];
      let a0 = Infinity, a1 = -Infinity, b0 = Infinity, b1 = -Infinity;
      for (const p of A) { const d = p[0] * nx + p[1] * ny; a0 = min(a0, d); a1 = max(a1, d); }
      for (const p of B) { const d = p[0] * nx + p[1] * ny; b0 = min(b0, d); b1 = max(b1, d); }
      if (a1 < b0 || b1 < a0) return false;
    }
  return true;
}

export class Traffic {
  constructor() { this.claims = new Map(); this.stats = { requests: 0, refused: 0 }; }
  set(id, polys) { this.claims.set(id, polys.map((poly) => ({ poly, bb: box(poly) }))); }
  get(id) { return this.claims.get(id) ?? []; }
  // Number of leading polygons of `polys` that are free of every other vehicle's claim.
  freePrefix(id, polys) {
    this.stats.requests++;
    for (let i = 0; i < polys.length; i++) {
      const bb = box(polys[i]);
      for (const [other, list] of this.claims) {
        if (other === id) continue;
        for (const c of list) {
          if (bb[1] < c.bb[0] || c.bb[1] < bb[0] || bb[3] < c.bb[2] || c.bb[3] < bb[2]) continue;
          if (overlap(polys[i], c.poly)) { this.stats.refused++; return i; }
        }
      }
    }
    return polys.length;
  }
  // Who holds the first conflicting claim (for diagnostics).
  blocker(id, polys) {
    for (const p of polys) for (const [other, list] of this.claims) if (other !== id) for (const c of list) if (overlap(p, c.poly)) return other;
    return null;
  }
}

// Where a vehicle may stop on a path between s0 and s1: every `pitch` on straight pieces, at
// least `clear` away from any curved piece. Returned in travel order, without s0, ending with s1.
export function holdPoints(path, s0, s1, pitch = 4.0, clear = 0.3) {
  const out = [], dir = s1 >= s0 ? 1 : -1;
  for (const p of path.pieces) {
    if (p.k0 !== 0 || p.k1 !== 0) continue;
    const a = p.s0 + clear, b = p.s0 + p.len - clear;
    if (b <= a) continue;
    const n = max(1, ceil((b - a) / pitch));
    for (let i = 0; i <= n; i++) {
      const s = a + ((b - a) * i) / n;
      if ((s - s0) * dir > 0.5 && (s1 - s) * dir > 0.5) out.push(s);
    }
  }
  out.sort((x, y) => dir * (x - y));
  out.push(s1);
  return out;
}

// One vehicle driving along paths under traffic control. `capFn(path, s0, s1, base)` returns the
// limits of a run (the vehicle's own speed cap); `shape` is one of SHAPE.
const STEP = 0.4, RETRY = 0.25;
export class Drive {
  constructor(id, traffic, shape, capFn) {
    Object.assign(this, { id, traffic, shape, capFn, goal: null, move: null, spin: null, s: 0, v: 0, a: 0, path: null, waitT: 0, waited: 0, maxWait: 0, blockedBy: null, retryAt: 0, trimAt: 0 });
  }
  park(pose) { this.pose = { ...pose }; this.traffic.set(this.id, [rect(pose, this.shape.claim)]); }
  get busy() { return !!this.goal || !!this.spin; }
  // Drive from s0 to s1 on `path`. `mustStop`: extra permitted stopping points.
  go(path, s0, s1, base, stops = null) {
    this.path = path; this.s = s0;
    this.goal = { s1, base, holds: stops ?? holdPoints(path, s0, s1), dir: s1 >= s0 ? 1 : -1 };
    this.move = null; this.waitT = 0;
  }
  // Turn on the spot by `dth` (robot AMR only).
  turn(dth, lim) { this.spin = { dth, lim, th0: this.pose.th, move: null }; this.waitT = 0; }
  sweep(sA, sB) {
    const n = max(1, ceil(abs(sB - sA) / STEP)), out = [];
    for (let i = 0; i <= n; i++) { const p = this.path.at(sA + ((sB - sA) * i) / n); out.push(rect(p, this.shape.claim)); }
    return out;
  }
  step(t, dt) {
    this.v = 0; this.a = 0; this.w = 0;
    if (this.spin) return this.stepSpin(t, dt);
    const g = this.goal;
    if (!g) return;
    if (!this.move) {
      if (t < this.retryAt) { this.waitT += dt; return; }
      // sample points: a uniform grid plus every permitted stopping point up to the goal
      // a stop is not permitted where it would block another vehicle's only way out (noStop)
      const marks = [this.s, ...g.holds.filter((h) => (h - this.s) * g.dir > 1e-9 && (h === g.s1 || !this.noStop || !this.noStop(rect(this.path.at(h), this.shape.claim))))];
      const polys = [], idx = [];
      for (let k = 1; k < marks.length; k++) {
        const seg = this.sweep(marks[k - 1], marks[k]);
        if (k > 1) seg.shift();
        polys.push(...seg);
        idx.push(polys.length - 1);
      }
      const free = this.traffic.freePrefix(this.id, polys);
      let k = idx.length - 1;
      while (k >= 0 && idx[k] >= free) k--;
      if (k < 0) {
        this.blockedBy = this.traffic.blocker(this.id, polys);
        this.retryAt = t + RETRY; this.waitT += dt;
        return;
      }
      const target = marks[k + 1];
      this.claim = { polys: polys.slice(0, idx[k] + 1), s0: this.s, s1: target };
      this.traffic.set(this.id, this.claim.polys);
      this.move = new AxisMove(this.s, target, this.capFn(this.path, this.s, target, g.base), t - dt);
      this.maxWait = max(this.maxWait, this.waitT); this.waited += this.waitT; this.waitT = 0; this.blockedBy = null;
      this.trimAt = t + 0.5;
    }
    const m = this.move.at(t);
    this.s = m.p; this.v = m.v; this.a = m.a;
    const at = this.path.at(this.s);
    this.pose = { x: at.x, y: at.y, th: at.th };
    this.k = at.k; this.w = this.v * at.k;
    if (this.move.done(t)) {
      this.s = this.move.to; this.v = 0; this.a = 0; this.w = 0; this.move = null;
      const e = this.path.at(this.s);
      this.pose = { x: e.x, y: e.y, th: e.th };
      this.traffic.set(this.id, [rect(this.pose, this.shape.claim)]);
      if (abs(this.s - g.s1) < 1e-9) this.goal = null;
    } else if (t >= this.trimAt) {
      // give back what lies behind the vehicle
      const c = this.claim, n = c.polys.length, f = abs(this.s - c.s0) / max(1e-9, abs(c.s1 - c.s0));
      const keep = max(0, Math.floor(f * (n - 1)) - 1);
      this.traffic.set(this.id, c.polys.slice(keep));
      this.trimAt = t + 0.5;
    }
  }
  stepSpin(t, dt) {
    const sp = this.spin;
    if (!sp.move) {
      if (t < this.retryAt) { this.waitT += dt; return; }
      const poly = disc(this.pose, this.shape.rSpin);
      if (this.traffic.freePrefix(this.id, [poly]) < 1) { this.blockedBy = this.traffic.blocker(this.id, [poly]); this.retryAt = t + RETRY; this.waitT += dt; return; }
      this.traffic.set(this.id, [poly]);
      sp.move = new AxisMove(0, sp.dth, sp.lim, t - dt);
      this.maxWait = max(this.maxWait, this.waitT); this.waited += this.waitT; this.waitT = 0; this.blockedBy = null;
    }
    const m = sp.move.at(t);
    this.pose = { ...this.pose, th: sp.th0 + m.p };
    this.w = m.v; this.k = 0;
    if (sp.move.done(t)) {
      this.pose.th = sp.th0 + sp.dth; this.w = 0; this.spin = null;
      this.traffic.set(this.id, [rect(this.pose, this.shape.claim)]);
    }
  }
}
