// Jerk-limited rest-to-rest motion law (7-segment S-curve). No dependencies.

const { sqrt, cbrt, min, max } = Math;

// Profile for a distance D >= 0 under limits v, a, j. Returns T and samplers.
export function scurve(D, v, a, j) {
  if (D <= 1e-12) return { T: 0, D: 0, vPeak: 0, aPeak: 0, at: () => ({ p: 0, v: 0, a: 0, j: 0 }) };

  let vp = v, ap, tj, tc, tv;
  const shape = (vv) => {
    if (vv * j < a * a) { ap = sqrt(vv * j); tj = ap / j; tc = 0; } // amax never reached
    else { ap = a; tj = a / j; tc = vv / a - tj; }
  };
  shape(vp);
  if (vp * (2 * tj + tc) <= D) tv = (D - vp * (2 * tj + tc)) / vp;
  else {
    // vmax not reached: solve the peak speed for a profile with no cruise
    tv = 0;
    vp = (a * (-a / j + sqrt((a * a) / (j * j) + (4 * D) / a))) / 2; // assumes amax reached
    if (vp * j < a * a) vp = cbrt((D * D * j) / 4); // neither reached: D = 2 v^1.5 / sqrt(j)
    shape(vp);
  }

  const durs = [tj, tc, tj, tv, tj, tc, tj];
  const jerk = [j, 0, -j, 0, -j, 0, j];
  const seg = [];
  let t = 0, p = 0, vel = 0, acc = 0;
  for (let i = 0; i < 7; i++) {
    seg.push({ t, p, v: vel, a: acc, j: jerk[i] });
    const d = durs[i];
    p += vel * d + (acc * d * d) / 2 + (jerk[i] * d * d * d) / 6;
    vel += acc * d + (jerk[i] * d * d) / 2;
    acc += jerk[i] * d;
    t += d;
  }
  const T = t;

  function at(time) {
    if (time <= 0) return { p: 0, v: 0, a: 0, j: 0 };
    if (time >= T) return { p: D, v: 0, a: 0, j: 0 };
    let i = 6;
    while (seg[i].t > time) i--;
    const s = seg[i], d = time - s.t;
    return {
      p: s.p + s.v * d + (s.a * d * d) / 2 + (s.j * d * d * d) / 6,
      v: s.v + s.a * d + (s.j * d * d) / 2,
      a: s.a + s.j * d,
      j: s.j,
    };
  }
  return { T, D, vPeak: vp, aPeak: ap, at, endError: p - D };
}

// One axis executing a point-to-point move that starts at time t0.
export class AxisMove {
  constructor(from, to, lim, t0) {
    this.from = from;
    this.to = to;
    this.sign = to >= from ? 1 : -1;
    this.t0 = t0;
    this.prof = scurve(Math.abs(to - from), lim.v, lim.a, lim.j);
    this.tEnd = t0 + this.prof.T;
  }
  at(t) {
    const s = this.prof.at(min(max(t - this.t0, 0), this.prof.T));
    return { p: t >= this.tEnd ? this.to : this.from + this.sign * s.p, v: this.sign * s.v, a: this.sign * s.a, j: this.sign * s.j };
  }
  done(t) { return t >= this.tEnd; }
}
