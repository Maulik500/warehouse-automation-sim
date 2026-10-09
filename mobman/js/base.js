// Differential-drive base: wheel <-> body kinematics, exact pose integration, the speed cap
// along a path, frame transforms to the arm, arm capsules, and quasi-static stability.
// Spec §5, §8. No rendering dependencies.

import { AMR, ARM, DECK, TOOL, G } from './config.js';
import { Rz, mv, add, sub, tr, frames } from './arm.js';

const { abs, sin, cos, sqrt, min, max, ceil } = Math;
const b = AMR.track;

// Body twist (v forward, w = yaw rate) from the wheel rim speeds, and back. The map is linear
// with the constant determinant 1/b: it has no singularity (spec §7.2).
export const twistOfWheels = (vL, vR) => ({ v: (vR + vL) / 2, w: (vR - vL) / b });
export const wheelsOfTwist = (v, w) => ({ vL: v - (w * b) / 2, vR: v + (w * b) / 2 });
export const WHEEL_MAP_DET = 1 / b;

// Exact pose after dt at constant twist (circular arc, or a straight line for w = 0).
export function stepPose(p, v, w, dt) {
  if (abs(w) < 1e-12) return { x: p.x + v * cos(p.th) * dt, y: p.y + v * sin(p.th) * dt, th: p.th };
  const th1 = p.th + w * dt, r = v / w;
  return { x: p.x + r * (sin(th1) - sin(p.th)), y: p.y - r * (cos(th1) - cos(p.th)), th: th1 };
}

// Limits for running a path between s0 and s1: `lim` {v, a, j} with v reduced so that lateral
// acceleration, wheel rim speed, yaw rate, yaw acceleration and wheel acceleration all hold.
export function speedCap(path, s0, s1, lim) {
  let k = 0, dk = 0;
  const a0 = min(s0, s1), a1 = max(s0, s1), n = max(1, ceil((a1 - a0) / 0.005));
  for (let i = 0; i <= n; i++) {
    const c = path.curv(a0 + ((a1 - a0) * i) / n);
    k = max(k, abs(c.k)); dk = max(dk, abs(c.dk));
  }
  let v = lim.v;
  if (k > 0) v = min(v, sqrt(AMR.aLat / k), AMR.wheelV / (1 + (k * b) / 2), AMR.omega / k);
  if (dk > 0) v = min(v, sqrt(max(AMR.alpha - k * lim.a, 0) / dk), sqrt(max(AMR.wheelA - lim.a * (1 + (k * b) / 2), 0) / ((dk * b) / 2)));
  return { v, a: lim.a, j: lim.j, kMax: k };
}

// ---- frames
export const toWorld = (pose, pB) => [pose.x + cos(pose.th) * pB[0] - sin(pose.th) * pB[1], pose.y + sin(pose.th) * pB[0] + cos(pose.th) * pB[1], pB[2]];
export const toBody = (pose, pW) => {
  const dx = pW[0] - pose.x, dy = pW[1] - pose.y;
  return [cos(pose.th) * dx + sin(pose.th) * dy, -sin(pose.th) * dx + cos(pose.th) * dy, pW[2]];
};
// Arm base frame {A} in {W}.
export const armFrame = (pose) => ({ R: Rz(pose.th + ARM.yaw), p: toWorld(pose, ARM.base) });
export const armToWorld = (pose, pA) => { const F = armFrame(pose); return add(F.p, mv(F.R, pA)); };
export const worldToArm = (pose, pW) => { const F = armFrame(pose); return mv(tr(F.R), sub(pW, F.p)); };
export const yawToArm = (pose, yawW) => yawW - pose.th - ARM.yaw;

// ---- arm geometry in {A} for clearance checks and drawing: capsules { a, b, r }.
// The upper arm and forearm tubes are offset from the J1 axis plane along the J2 axis (spec §3.3).
export function armCapsules(q) {
  const T = frames(q), z1 = [T[1].R[2], T[1].R[5], T[1].R[8]];
  const off = (p, e) => [p[0] + z1[0] * e, p[1] + z1[1] * e, p[2] + z1[2] * e];
  const z6 = [T[6].R[2], T[6].R[5], T[6].R[8]];
  const tcp = [T[6].p[0] + z6[0] * TOOL.tcp, T[6].p[1] + z6[1] * TOOL.tcp, T[6].p[2] + z6[2] * TOOL.tcp];
  const [rU, rF, rW, rT] = ARM.rLink, eU = ARM.offUpper, eF = ARM.offFore;
  return [
    { name: 'shoulder', a: T[1].p, b: off(T[1].p, eU), r: rU },
    { name: 'upper arm', a: off(T[1].p, eU), b: off(T[2].p, eU), r: rU },
    { name: 'elbow', a: off(T[2].p, eU), b: off(T[2].p, eF), r: rF },
    { name: 'forearm', a: off(T[2].p, eF), b: off(T[3].p, eF), r: rF },
    { name: 'wrist 1', a: off(T[3].p, eF), b: T[4].p, r: rW },
    { name: 'wrist 2', a: T[4].p, b: T[5].p, r: rW },
    { name: 'wrist 3', a: T[5].p, b: T[6].p, r: rW },
    { name: 'tool', a: T[6].p, b: tcp, r: rT },
  ];
}

// ---- mass properties and stability (quasi-static), spec §8
// `load`: { q, held: { m, h } or null, deck: [{ m, x, y, z }] in {B} }.
export function massProps(load) {
  const parts = AMR.parts.map((p) => ({ m: p.m, c: p.cg }));
  const F = { R: Rz(ARM.yaw), p: ARM.base }, inB = (pA) => add(F.p, mv(F.R, pA));
  const C = armCapsules(load.q), mid = (c) => inB([(c.a[0] + c.b[0]) / 2, (c.a[1] + c.b[1]) / 2, (c.a[2] + c.b[2]) / 2]);
  const [mB, mU, mF, mW] = ARM.linkMass;
  parts.push({ m: mB, c: mid(C[0]) }, { m: mU, c: mid(C[1]) }, { m: mF, c: mid(C[3]) }, { m: mW, c: mid(C[5]) });
  const tcp = inB(C[7].b);
  parts.push({ m: TOOL.mass, c: [tcp[0], tcp[1], tcp[2] + TOOL.tcp - TOOL.cg] });
  if (load.held) parts.push({ m: load.held.m, c: [tcp[0], tcp[1], tcp[2] - load.held.h / 2] });
  for (const d of load.deck || []) parts.push({ m: d.m, c: [d.x, d.y, d.z] });
  let m = 0;
  const c = [0, 0, 0];
  for (const p of parts) { m += p.m; for (let i = 0; i < 3; i++) c[i] += p.m * p.c[i]; }
  return { m, cg: c.map((v) => v / m) };
}
// Support polygon: the rectangle through the four casters (the drive wheels lie inside it).
export const SUPPORT = { x: AMR.casters[0][0], y: AMR.casters[0][1] };
export function stability(load) {
  const { m, cg } = massProps(load);
  const mx = SUPPORT.x - abs(cg[0]), my = SUPPORT.y - abs(cg[1]);
  return {
    m, cg,
    margin: min(mx, my), // distance of the CG projection to the nearest tipping edge
    brakeTip: (G * mx) / cg[2], // deceleration at which the wheel load at the far end reaches 0
    sideTip: (G * my) / cg[2], // lateral acceleration, same
    slide: DECK.mu * G, // a carton on braked rollers starts to slide
  };
}

// Required vacuum holding force for a box of mass m with the tool vertical (spec §9.2).
export const holdForce = (m, az, ah) => m * (G + max(az, 0) + ah / TOOL.mu);
export const VAC_FORCE = TOOL.cups * TOOL.vacuum * Math.PI * (TOOL.cupDia / 2) ** 2;
