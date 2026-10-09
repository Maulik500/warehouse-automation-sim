// Continuous-curvature paths for the reference point (load-axle midpoint) and the speed cap
// that keeps steer rate, steer acceleration, wheel speed and lateral acceleration in limits.
// No dependencies besides config.js.

import { AMR, TURN } from './config.js';

const { abs, sin, cos, sqrt, min, max, sign, ceil, floor } = Math;

// Quintic blend 0 -> 1 with zero first and second derivative at both ends, and its integral.
const S5 = (t) => t * t * t * (10 + t * (-15 + 6 * t));
const S5i = (t) => t * t * t * t * (2.5 + t * (-3 + t));
const S5d = (t) => 30 * t * t * (1 - t) * (1 - t);
const S5dd = (t) => 60 * t * (1 - t) * (1 - 2 * t);

// A piece is { len, k0, k1 }: curvature goes from k0 to k1 along a quintic blend.
export const line = (len) => ({ len, k0: 0, k1: 0 });
// Turn by `angle` (signed, + = left): ramp in, constant arc, ramp out. Each ramp turns Lc/(2R).
export function turn(angle, R = TURN.R, Lc = TURN.Lc) {
  const k = sign(angle) / R, arc = abs(angle) * R - Lc;
  if (arc < 0) throw new Error('turn angle too small for this radius and ramp length');
  return [{ len: Lc, k0: 0, k1: k }, { len: arc, k0: k, k1: k }, { len: Lc, k0: k, k1: 0 }];
}

const DS = 0.001;
export class Path {
  constructor(start, pieces) {
    this.pieces = [];
    let s = 0, th = start.th;
    for (const p of pieces) {
      if (p.len <= 1e-12) continue;
      this.pieces.push({ ...p, s0: s, th0: th });
      s += p.len;
      th += p.k0 * p.len + (p.k1 - p.k0) * p.len * 0.5;
    }
    this.length = s;
    // position table by Simpson's rule on the analytic heading
    const n = max(1, ceil(s / DS)), h = s / n;
    this.h = h;
    this.xs = new Float64Array(n + 1);
    this.ys = new Float64Array(n + 1);
    this.xs[0] = start.x; this.ys[0] = start.y;
    let t0 = this.heading(0);
    for (let i = 0; i < n; i++) {
      const tm = this.heading((i + 0.5) * h), t1 = this.heading((i + 1) * h);
      this.xs[i + 1] = this.xs[i] + (h / 6) * (cos(t0) + 4 * cos(tm) + cos(t1));
      this.ys[i + 1] = this.ys[i] + (h / 6) * (sin(t0) + 4 * sin(tm) + sin(t1));
      t0 = t1;
    }
  }
  piece(s) {
    const P = this.pieces;
    let i = P.length - 1;
    while (i > 0 && P[i].s0 > s) i--;
    return P[i];
  }
  heading(s) {
    const p = this.piece(s), u = min(max(s - p.s0, 0), p.len);
    return p.th0 + p.k0 * u + (p.k1 - p.k0) * p.len * S5i(u / p.len);
  }
  // Curvature and its first two derivatives with respect to arc length.
  curv(s) {
    const p = this.piece(s), t = min(max((s - p.s0) / p.len, 0), 1), d = p.k1 - p.k0;
    return { k: p.k0 + d * S5(t), dk: (d * S5d(t)) / p.len, ddk: (d * S5dd(t)) / (p.len * p.len) };
  }
  at(s) {
    s = min(max(s, 0), this.length);
    const f = s / this.h, i = min(floor(f), this.xs.length - 2), u = f - i;
    return {
      x: this.xs[i] + u * (this.xs[i + 1] - this.xs[i]),
      y: this.ys[i] + u * (this.ys[i + 1] - this.ys[i]),
      th: this.heading(s),
      ...this.curv(s),
    };
  }
  get end() { return this.at(this.length); }
}

// Steer angle φ = −atan(L·κ) and its derivatives with respect to arc length.
export function steerOfPath(c) {
  const L = AMR.L, q = 1 + L * L * c.k * c.k;
  return {
    phi: -Math.atan(L * c.k),
    dphi: (-L * c.dk) / q,
    ddphi: (-L * (c.ddk * q - 2 * L * L * c.k * c.dk * c.dk)) / (q * q),
  };
}

// Drive limits for running the path between s0 and s1: `base` {v, a, j} reduced so that
// lateral acceleration, drive wheel speed, steer rate and steer acceleration stay in limits.
export function speedCap(path, s0, s1, base) {
  let k = 0, d1 = 0, d2 = 0;
  const a = min(s0, s1), b = max(s0, s1), n = max(1, ceil((b - a) / 0.005));
  for (let i = 0; i <= n; i++) {
    const c = path.curv(a + ((b - a) * i) / n), st = steerOfPath(c);
    k = max(k, abs(c.k)); d1 = max(d1, abs(st.dphi)); d2 = max(d2, abs(st.ddphi));
  }
  let v = base.v;
  if (k > 0) v = min(v, sqrt(AMR.aLat / k), AMR.wheelV / sqrt(1 + AMR.L * AMR.L * k * k));
  if (d1 > 0) v = min(v, AMR.steer.v / d1);
  if (d2 > 0) v = min(v, sqrt(max(AMR.steer.a - d1 * base.a, 0) / d2)); // φ̈ = φ″·v² + φ′·a
  return { v, a: base.a, j: base.j, kMax: k };
}

// Tangent length of a symmetric turn: distance from the corner to where the turn starts / ends.
export function tangentLength(angle, R = TURN.R, Lc = TURN.Lc) {
  const e = new Path({ x: 0, y: 0, th: 0 }, turn(angle, R, Lc)).end;
  return abs(e.y) / abs(sin(angle));
}
