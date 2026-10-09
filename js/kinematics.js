// Closed-form forward / inverse kinematics for the 6R spherical-wrist arm
// defined in config.js. No rendering dependencies — runs in Node and browser.
// Matrices are 3x3, row-major, flat arrays of 9.

import { ROBOT as RB, TOOL } from './config.js';

const { d1, a1, a2, a3, d4, d6 } = RB;
const { sin, cos, atan2, acos, hypot, PI } = Math;

export const L3 = hypot(d4, a3); // J3 -> wrist centre distance
export const BETA = atan2(d4, a3); // angle of that line from the upper-arm axis at q3 = 0

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
export const Rx = (t) => [1, 0, 0, 0, cos(t), -sin(t), 0, sin(t), cos(t)];
export const Ry = (t) => [cos(t), 0, sin(t), 0, 1, 0, -sin(t), 0, cos(t)];
export const Rz = (t) => [cos(t), -sin(t), 0, sin(t), cos(t), 0, 0, 0, 1];

const wrap = (a) => atan2(sin(a), cos(a));
// Equivalent angle (mod 2pi) closest to ref, pulled back inside [lo, hi] if needed.
function near(a, ref, lo, hi) {
  a = ref + wrap(a - ref);
  while (a > hi) a -= 2 * PI;
  while (a < lo) a += 2 * PI;
  return a;
}

// Flange orientation for a tool pointing straight down with its X axis at yaw psi.
export const toolDown = (psi) => mul(Rz(psi), Rx(PI));

export function fk(q) {
  const [q1, q2, q3, q4, q5, q6] = q;
  const q23 = q2 + q3;
  const rho = a1 + a2 * sin(q2) + d4 * cos(q23) + a3 * sin(q23); // wrist-centre radius from J1 axis
  const zw = d1 + a2 * cos(q2) - d4 * sin(q23) + a3 * cos(q23);
  const pw = [rho * cos(q1), rho * sin(q1), zw];
  const rj3 = a1 + a2 * sin(q2);
  const j3 = [rj3 * cos(q1), rj3 * sin(q1), d1 + a2 * cos(q2)];
  // Ra: frame whose X axis is the tool approach direction
  const Ra = mul(mul(mul(mul(Rz(q1), Ry(q23)), Rx(q4)), Ry(q5)), Rx(q6));
  const ax = [Ra[0], Ra[3], Ra[6]];
  const along = (d) => [pw[0] + d * ax[0], pw[1] + d * ax[1], pw[2] + d * ax[2]];
  return { pw, j3, rho, flange: along(d6), tcp: along(d6 + TOOL.tcp), R: mul(Ra, Ry(PI / 2)) };
}

// Distances to the three singular surfaces. Each is 0 on the singularity;
// det(J) = a2 * L3 * elbow * shoulder * wrist (see spec §6).
export function singularity(q) {
  return {
    wrist: sin(q[4]),
    elbow: sin(q[2] + BETA),
    shoulder: a1 + a2 * sin(q[1]) + d4 * cos(q[1] + q[2]) + a3 * sin(q[1] + q[2]),
  };
}

// Inverse kinematics for a TCP pose. Returns joint vector or null if the pose
// is out of reach or violates a joint limit. `ref` selects the 2pi-branch of
// the continuous joints; `cfg` selects one of the 8 posture branches.
export function ik(tcp, Rf, ref, cfg = { front: true, up: true, flip: false }) {
  const Ra = mul(Rf, Ry(-PI / 2));
  const ax = [Ra[0], Ra[3], Ra[6]];
  const k = d6 + TOOL.tcp;
  const pw = [tcp[0] - k * ax[0], tcp[1] - k * ax[1], tcp[2] - k * ax[2]];

  const rxy = hypot(pw[0], pw[1]);
  if (rxy < 1e-6) return null; // shoulder singularity: q1 undefined
  let q1 = atan2(pw[1], pw[0]) + (cfg.front ? 0 : PI);
  q1 = near(q1, ref[0], RB.qMin[0], RB.qMax[0]);

  const r = (cfg.front ? rxy : -rxy) - a1;
  const s = pw[2] - d1;
  const c = (r * r + s * s - a2 * a2 - L3 * L3) / (2 * a2 * L3);
  if (c > 1 || c < -1) return null; // out of reach
  const g = (cfg.up ? 1 : -1) * acos(c);
  const q3 = near(g - BETA, ref[2], RB.qMin[2], RB.qMax[2]);
  const q2 = near(atan2(r, s) - atan2(L3 * sin(g), a2 + L3 * cos(g)), ref[1], RB.qMin[1], RB.qMax[1]);

  // Wrist: R36 = Rx(q4) Ry(q5) Rx(q6)
  const M = mul(tr(mul(Rz(q1), Ry(q2 + q3))), Ra);
  const s5 = hypot(M[3], M[6]);
  let q4, q5, q6;
  if (s5 < 1e-7) {
    // wrist singularity: only q4 + q6 is defined; hold q4
    q5 = 0;
    q4 = ref[3];
    q6 = atan2(M[7], M[4]) - q4;
  } else if (!cfg.flip) {
    q5 = atan2(s5, M[0]);
    q4 = atan2(M[3], -M[6]);
    q6 = atan2(M[1], M[2]);
  } else {
    q5 = atan2(-s5, M[0]);
    q4 = atan2(-M[3], M[6]);
    q6 = atan2(-M[1], -M[2]);
  }
  q4 = near(q4, ref[3], RB.qMin[3], RB.qMax[3]);
  q6 = near(q6, ref[5], RB.qMin[5], RB.qMax[5]);

  const q = [q1, q2, q3, q4, q5, q6];
  for (let i = 0; i < 6; i++) if (q[i] < RB.qMin[i] - 1e-9 || q[i] > RB.qMax[i] + 1e-9) return null;
  return q;
}
