// Closed-form forward / inverse kinematics, Jacobian and singularity measures for the 6R
// offset-wrist arm (collaborative-robot geometry) defined in config.js. Everything here is
// expressed in the arm base frame {A}. No rendering dependencies — runs in Node and browser.
// Rotations are 3x3, row-major, flat arrays of 9. A transform is { R, p }.

import { ARM, TOOL } from './config.js';

const { d1, a2, a3, d4, d5, d6 } = ARM;
const { sin, cos, atan2, acos, asin, hypot, abs, PI } = Math;

export const mul = (A, B) => {
  const C = new Array(9);
  for (let r = 0; r < 3; r++)
    for (let c = 0; c < 3; c++)
      C[r * 3 + c] = A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c];
  return C;
};
export const tr = (A) => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
export const mv = (A, v) => [
  A[0] * v[0] + A[1] * v[1] + A[2] * v[2],
  A[3] * v[0] + A[4] * v[1] + A[5] * v[2],
  A[6] * v[0] + A[7] * v[1] + A[8] * v[2],
];
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const Rx = (t) => [1, 0, 0, 0, cos(t), -sin(t), 0, sin(t), cos(t)];
export const Ry = (t) => [cos(t), 0, sin(t), 0, 1, 0, -sin(t), 0, cos(t)];
export const Rz = (t) => [cos(t), -sin(t), 0, sin(t), cos(t), 0, 0, 0, 1];
export const I3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
export const compose = (A, B) => ({ R: mul(A.R, B.R), p: add(A.p, mv(A.R, B.p)) });
export const invert = (A) => { const Rt = tr(A.R); return { R: Rt, p: mv(Rt, A.p).map((v) => -v) }; };

// Denavit–Hartenberg rows (classic / distal: Rz(θ)·Tz(d)·Tx(a)·Rx(α)), spec §3.2.
export const DH = [
  { d: d1, a: 0, alpha: PI / 2 },
  { d: 0, a: -a2, alpha: 0 },
  { d: 0, a: -a3, alpha: 0 },
  { d: d4, a: 0, alpha: PI / 2 },
  { d: d5, a: 0, alpha: -PI / 2 },
  { d: d6, a: 0, alpha: 0 },
];
export const dhT = (row, th) => ({ R: mul(Rz(th), Rx(row.alpha)), p: [row.a * cos(th), row.a * sin(th), row.d] });

// Flange orientation for a tool pointing straight down with the tool X axis at yaw psi.
export const toolDown = (psi) => mul(Rz(psi), Rx(PI));

// Frames 0..6 in {A}; frame i is attached to link i, joint i+1 turns about its Z axis.
export function frames(q) {
  const T = [{ R: I3, p: [0, 0, 0] }];
  for (let i = 0; i < 6; i++) T.push(compose(T[i], dhT(DH[i], q[i])));
  return T;
}

export function fk(q) {
  const T = frames(q), F = T[6], z = [F.R[2], F.R[5], F.R[8]];
  return { T, R: F.R, flange: F.p, tcp: add(F.p, z.map((v) => v * TOOL.tcp)), o5: T[5].p, elbow: T[2].p, z };
}

// Geometric Jacobian of the flange origin: rows vx vy vz wx wy wz, columns q1..q6.
export function jacobian(q) {
  const T = frames(q), pe = T[6].p, J = Array.from({ length: 6 }, () => new Array(6));
  for (let i = 0; i < 6; i++) {
    const z = [T[i].R[2], T[i].R[5], T[i].R[8]], lin = cross(z, sub(pe, T[i].p));
    for (let r = 0; r < 3; r++) { J[r][i] = lin[r]; J[r + 3][i] = z[r]; }
  }
  return J;
}
export function det(M) {
  const n = M.length, A = M.map((r) => [...r]);
  let d = 1;
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (abs(A[r][c]) > abs(A[p][c])) p = r;
    if (abs(A[p][c]) < 1e-300) return 0;
    if (p !== c) { [A[p], A[c]] = [A[c], A[p]]; d = -d; }
    d *= A[c][c];
    for (let r = c + 1; r < n; r++) {
      const f = A[r][c] / A[c][c];
      for (let k = c; k < n; k++) A[r][k] -= f * A[c][k];
    }
  }
  return d;
}

// The three factors of det J (spec §7): det J = a2·a3·sin q3·sin q5·shoulder.
// `shoulder` is the signed distance of the wrist point O5 from the plane that contains the
// J1 axis and is parallel to the J2 axis — 0 when O5 is on the cylinder of radius d4.
export function singularity(q) {
  const [, q2, q3, q4] = q;
  return {
    elbow: sin(q3),
    wrist: sin(q[4]),
    shoulder: -a2 * cos(q2) - a3 * cos(q2 + q3) + d5 * sin(q2 + q3 + q4),
  };
}

const wrap = (a) => atan2(sin(a), cos(a));
// Equivalent angle (mod 2π) closest to ref, pulled back inside [lo, hi] if needed.
function near(a, ref, lo, hi) {
  a = ref + wrap(a - ref);
  while (a > hi + 1e-9) a -= 2 * PI;
  while (a < lo - 1e-9) a += 2 * PI;
  return a;
}

export const BRANCHES = [];
for (const shoulder of [1, -1]) for (const elbow of [1, -1]) for (const wrist of [1, -1]) BRANCHES.push({ shoulder, elbow, wrist });

// Inverse kinematics for a TCP position and flange orientation, both in {A}. `cfg` selects one
// of the 8 posture branches (each sign ±1), `ref` the 2π-turn of every joint. Returns the joint
// vector, or null when the pose is out of reach, singular or outside the joint limits.
export function ik(tcp, Rf, ref, cfg) {
  const z6 = [Rf[2], Rf[5], Rf[8]];
  const p6 = sub(tcp, z6.map((v) => v * TOOL.tcp));
  const p5 = sub(p6, z6.map((v) => v * d6));

  // J1: O5 lies in the plane at distance d4 from the J1 axis  ->  ρ·sin(q1 − ψ) = d4
  const rho = hypot(p5[0], p5[1]);
  if (rho < d4 + 1e-9) return null; // shoulder singularity / unreachable cylinder
  const psi = atan2(p5[1], p5[0]), al = asin(d4 / rho);
  const q1 = cfg.shoulder > 0 ? psi + al : psi + PI - al;

  // R16 = Rz(q2+q3+q4)·Ry(−q5)·Rz(q6)
  const R01 = mul(Rz(q1), Rx(PI / 2)), R16 = mul(tr(R01), Rf);
  const c5 = Math.min(1, Math.max(-1, R16[8]));
  const q5 = cfg.wrist * acos(c5), s5 = sin(q5);
  if (abs(s5) < 1e-7) return null; // wrist singularity: only q234 − q6 or q234 + q6 is defined
  const q234 = atan2(-R16[5] / s5, -R16[2] / s5);
  const q6 = atan2(-R16[7] / s5, R16[6] / s5);

  // planar 2R chain to O4, in frame 1
  const R14 = mul(Rz(q234), Rx(PI / 2));
  const p16 = mv(tr(R01), sub(p6, [0, 0, d1]));
  const p14 = sub(p16, mv(R14, [-d6 * s5, d6 * cos(q5), d5]));
  const X = -p14[0], Y = -p14[1];
  const c3 = (X * X + Y * Y - a2 * a2 - a3 * a3) / (2 * a2 * a3);
  if (c3 > 1 || c3 < -1) return null; // out of reach
  const q3 = cfg.elbow * acos(c3);
  const q2 = atan2(Y, X) - atan2(a3 * sin(q3), a2 + a3 * cos(q3));
  const q4 = q234 - q2 - q3;

  const q = [q1, q2, q3, q4, q5, q6];
  for (let i = 0; i < 6; i++) {
    q[i] = near(q[i], ref[i], ARM.qMin[i], ARM.qMax[i]);
    if (q[i] < ARM.qMin[i] - 1e-9 || q[i] > ARM.qMax[i] + 1e-9) return null;
  }
  return q;
}

// Posture branch of a joint vector (inverse of the `cfg` argument of ik).
export function branchOf(q) {
  const s = singularity(q);
  return { shoulder: s.shoulder >= 0 ? 1 : -1, elbow: sin(q[2]) >= 0 ? 1 : -1, wrist: sin(q[4]) >= 0 ? 1 : -1 };
}
