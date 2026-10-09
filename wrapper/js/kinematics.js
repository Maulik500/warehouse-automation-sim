// Kinematics of the turntable wrapper: film line and moment arm (WRAPPER_SPEC.md §5), the film
// laid on the load (§5.3), the wrap program (§8), stability (§7.3) and the safety monitor (§9).
// No rendering dependencies.

import { PALLET, LINE, TABLE, CARRIAGE, FILM, WRAP, CLAMP, CUT, SAFE, G } from './config.js';
import { scurve } from '../../asrs/js/motion.js';

const { abs, sin, cos, atan2, asin, sqrt, hypot, min, max, PI, ceil } = Math;

export const cross = (a, b) => a[0] * b[1] - a[1] * b[0];
export const rot = (th, p) => { const c = cos(th), s = sin(th); return [c * p[0] - s * p[1], s * p[0] + c * p[1]]; };
export const wrapPi = (a) => a - 2 * PI * Math.round(a / (2 * PI));

export const PERIMETER = 2 * (PALLET.sx + PALLET.sy);
export const ARM_MEAN = PERIMETER / (2 * PI); // mean moment arm over a revolution (§5.2)
export const Z_BOTTOM = LINE.z + PALLET.h - WRAP.palletLock + FILM.w / 2; // lowest carriage position of the program

// Corners of the load in {P}, in the order the film reaches them when the table turns
// counter-clockwise: the contact zone is fixed in {W}, so in {P} it travels clockwise.
const HX = PALLET.sx / 2, HY = PALLET.sy / 2;
export const CORNERS = [[-HX, -HY], [-HX, HY], [HX, HY], [HX, -HY]];
const FACE_START = [0, 2 * HY, 2 * HY + 2 * HX, 4 * HY + 2 * HX]; // perimeter position of each corner

// ---------- film line (§5.1) ----------
// Straight film from the exit roller to the world point C. The film passes the roller on the
// load side, so the roller centre is on the right of the directed line roller → C.
// Returns the direction angle α, unit vector u, tangent point T on the roller, the free length
// and the moment arm p = z·(C × u) of the film force about the turntable axis.
export function filmLine(C) {
  const [ex, ey] = CARRIAGE.roller, r = CARRIAGE.rExit;
  const vx = C[0] - ex, vy = C[1] - ey, L = hypot(vx, vy);
  const alpha = atan2(vy, vx) - asin(r / L);
  const u = [cos(alpha), sin(alpha)];
  return { alpha, u, T: [ex - r * u[1], ey + r * u[0]], len: sqrt(L * L - r * r), arm: cross(C, u) };
}

// Film line of the home pose for a perfectly centred load: roller → corner (+hx, −hy).
export const homeFilmLine = (off = [0, 0]) => filmLine([HX + off[0], -HY + off[1]]);
// Distance of the clamp from the film line of the home pose (positive = on the load side).
export function clampMiss(off = [0, 0], th = 0) {
  const C = rot(th, [HX + off[0], -HY + off[1]]), l = filmLine(C), P = rot(th, [CLAMP.x, CLAMP.y]);
  return cross(l.u, [P[0] - l.T[0], P[1] - l.T[1]]);
}

// ---------- film on the load (§5.3) ----------
// The film is a chain of anchors in {P}: the clamp (the tail), then every load corner it has
// been wrapped round. The last anchor is the active one: the film runs from it straight to the
// exit roller and pivots about it until the line reaches the next corner.
export class FilmWrap {
  constructor(off, z0) {
    this.off = off; // pallet centre in {T}
    this.anchors = [{ x: CLAMP.x - off[0], y: CLAMP.y - off[1], z: z0, th: 0 }];
    this.next = 0; // index into CORNERS
    this.laid = 0; // film length between the anchors
    this.tailHeld = true;
    this.done = false;
    this.version = 0;
  }
  get active() { return this.anchors[this.anchors.length - 1]; }
  get corners() { return this.anchors.length - 1; }
  world(a, th) { return rot(th, [a.x + this.off[0], a.y + this.off[1]]); }
  line(th) { return filmLine(this.world(this.active, th)); }
  // > 0 while the next corner is clear of the film line, 0 at the hand-over
  gap(th) {
    const n = CORNERS[this.next], A = this.world(this.active, th), N = this.world({ x: n[0], y: n[1] }, th);
    return cross(filmLine(A).u, [N[0] - A[0], N[1] - A[1]]);
  }
  // Table goes th0 → th1 while the carriage goes z0 → z1. Every corner the film line reaches
  // on the way becomes the active anchor. The film that lands on a face is a straight line
  // towards the carriage, so the new anchor's height is interpolated on it.
  advance(th0, z0, th1, z1) {
    let n = 0;
    while (this.gap(th1) < 0 && n < 4) {
      let lo = th0, hi = th1;
      for (let i = 0; i < 48; i++) { const m = (lo + hi) / 2; if (this.gap(m) < 0) hi = m; else lo = m; }
      const zc = th1 > th0 ? z0 + ((z1 - z0) * (hi - th0)) / (th1 - th0) : z1;
      const a = this.active, c = CORNERS[this.next], seg = hypot(c[0] - a.x, c[1] - a.y), free = this.line(hi).len;
      this.anchors.push({ x: c[0], y: c[1], z: a.z + ((zc - a.z) * seg) / free, th: hi });
      this.laid += seg;
      this.next = (this.next + 1) % 4;
      this.version++;
      n++;
    }
    return n;
  }
  // The clamp opens: the tail falls against the −Y face and is covered by the next wrap.
  release() {
    const a = this.anchors[0], c = this.anchors[1], len = hypot(c.x - a.x, c.y - a.y);
    a.x = c.x + len; a.y = c.y;
    this.tailHeld = false;
    this.version++;
  }
  // The film is cut at home: it ends on the −Y face where the brush pressed it down.
  finish() {
    const a = this.active, x = CUT.tailX - this.off[0];
    this.laid += abs(a.x - x);
    this.anchors.push({ x, y: a.y, z: a.z, th: a.th });
    this.done = true;
    this.version++;
  }
  // Path length from a fixed point on the exit roller to the tail, up to a constant.
  pathLength(th) { const l = this.line(th); return this.laid + l.len - CARRIAGE.rExit * l.alpha; }
  // Number of film layers on the load surface at perimeter position s (0 … PERIMETER, from the
  // first corner, in wrap order) and height z.
  layers(s, z) {
    let n = 0;
    for (let i = 2; i < this.anchors.length; i++) {
      const a = this.anchors[i - 1], b = this.anchors[i], u = (s - FACE_START[(i - 2) % 4]) / hypot(b.x - a.x, b.y - a.y);
      if (u >= 0 && u <= 1 && abs(z - (a.z + u * (b.z - a.z))) <= FILM.w / 2) n++;
    }
    return n;
  }
}

// ---------- wrap program (§8) ----------
export function wrapHeights(H) {
  return { bottom: Z_BOTTOM, top: max(Z_BOTTOM, LINE.z + H + WRAP.overTop - FILM.w / 2) };
}
export const pitchOf = (overlap) => (1 - overlap) * FILM.w; // film advance per revolution
export const liftLimits = (overlap) => ({ v: (pitchOf(overlap) * TABLE.wrap.v) / (2 * PI), a: CARRIAGE.lift.a, j: CARRIAGE.lift.j });

// Timing of one wrap cycle for a load of height H (pallet included). The turntable makes ONE
// rest-to-rest move of a whole number of revolutions; the carriage moves are started at fixed
// times of that move.
export function wrapPlan(H) {
  const { bottom, top } = wrapHeights(H), L = TABLE.wrap;
  const ref = scurve(1e6, L.v, L.a, L.j); // spin-up followed by an endless cruise
  const tOf = (th) => { let lo = 0, hi = th / L.v + 10; for (let i = 0; i < 60; i++) { const m = (lo + hi) / 2; if (ref.at(m).p < th) lo = m; else hi = m; } return hi; };
  const up = scurve(top - bottom, ...Object.values(liftLimits(WRAP.overlapUp)));
  const down = scurve(top - bottom, ...Object.values(liftLimits(WRAP.overlapDown)));
  const t1 = tOf(2 * PI * WRAP.nBottom), t2 = t1 + up.T;
  const t3 = tOf(ref.at(t2).p + 2 * PI * WRAP.nTop), t4 = t3 + down.T;
  const revs = ceil(ref.at(t4).p / (2 * PI) + WRAP.nFinal - 1e-9);
  const turn = scurve(2 * PI * revs, L.v, L.a, L.j);
  return { bottom, top, revs, T: turn.T, tSpin: L.v / L.a + L.a / L.j, tRelease: tOf(WRAP.releaseAt), t1, t2, t3, t4, up, down };
}

// ---------- load on the turntable (§7.3) ----------
export const sweptRadius = (off) => hypot(HX + abs(off[0]), HY + abs(off[1]));

// Unwrapped load at table speed ω and acceleration α: every box column feels the inertial
// acceleration r·√(ω⁴ + α²). Limits: sliding μ·g, tipping g·(half base along the push) / (half height).
// Box columns of a load: footprint, position in {P} and height above the pallet deck.
export function columns(boxes) {
  const cols = new Map();
  for (const b of boxes) {
    const k = b.lx + ',' + b.ly, c = cols.get(k) ?? { sx: b.sx, sy: b.sy, lx: b.lx, ly: b.ly, top: 0 };
    c.top = max(c.top, b.lz + b.sz / 2 - PALLET.h);
    cols.set(k, c);
  }
  return [...cols.values()];
}
export function loadStability(cols, off, om, al) {
  let sf = Infinity, acc = 0;
  for (const c of cols) {
    const rx = c.lx + off[0], ry = c.ly + off[1];
    const ax = om * om * rx + al * ry, ay = om * om * ry - al * rx, a = hypot(ax, ay);
    if (a < 1e-9) continue;
    const tip = (G * ((c.sx / 2) * abs(ax) + (c.sy / 2) * abs(ay))) / a / (c.top / 2);
    sf = min(sf, (PALLET.mu * G) / a, tip / a);
    acc = max(acc, a);
  }
  return { sf, acc };
}
// Film tension that would slide the top layer off the layer below.
export const topLayerHold = (layerMass) => PALLET.mu * layerMass * G;

// ---------- safety monitor (§9.2) ----------
// Reads only positions, speeds and sensor states; knows nothing about what the sequencer intends.
export function monitor(s) {
  const bad = [], L = CARRIAGE.lift, eps = 1e-6;
  const turning = abs(s.om) > eps;
  const home = !turning && abs(wrapPi(s.th)) <= TABLE.homeTol;
  if (s.lift < L.min - L.tol || s.lift > L.max + L.tol) bad.push('R1 carriage beyond its travel');
  if (s.slide < CUT.park - eps || s.slide > CUT.press + eps) bad.push('R1 cut slide beyond its travel');
  if (s.om > TABLE.wrap.v * 1.0001 || abs(s.al) > TABLE.wrap.a * 1.0001) bad.push('R2 turntable speed / acceleration');
  if (s.om < -eps) bad.push('R2 turntable turning backwards');
  if (abs(s.liftV) > L.v * 1.0001) bad.push('R2 carriage speed');
  if (abs(s.deckV) > LINE.move.v * 1.0001) bad.push('R2 deck conveyor speed');
  if (abs(s.deckV) > eps && !home) bad.push('R3 deck conveyor running off the home position');
  if (turning) {
    if (s.b2 || s.b4) bad.push('R4 rotating with a pallet across a gap');
    if (s.slide > CUT.park + eps) bad.push('R4 rotating with the cut slide extended');
    if (s.load && hypot(s.load.off[0], s.load.off[1]) > TABLE.centreTol) bad.push('R4 rotating an off-centre load');
    if (s.load && sweptRadius(s.load.off) > SAFE.rLoad) bad.push('R8 load sweeps beyond its envelope');
    if (s.load && s.load.sf < SAFE.sfLoad) bad.push('R10 load stability');
  }
  if (abs(s.liftV) > eps && s.filmOnLoad && s.om < WRAP.omLift) bad.push('R5 carriage moving without rotation');
  if (s.clampUp > eps && turning && s.thCycle > WRAP.clampDownBy) bad.push('R6 clamp raised into the film');
  if (s.filmOnLoad && s.jaw < 1 - eps && s.corners < WRAP.anchorCorners) bad.push('R6 tail released before the film is anchored');
  if (s.slide > CUT.park + eps && !(home && s.jaw > 1 - eps && s.clampUp > 1 - eps && abs(s.lift - Z_BOTTOM) <= L.tol)) bad.push('R7 cut slide extended out of sequence');
  if (abs(s.dancer) > FILM.dancer.stroke) bad.push('R9 dancer at its stop');
  if (s.tension > FILM.Fmax || (s.filmOnLoad && turning && s.tension < FILM.Fmin)) bad.push('R9 film tension');
  if (s.vFeed > FILM.feed.vMax * 1.0001) bad.push('R9 film speed');
  return bad;
}
