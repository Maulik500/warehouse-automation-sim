// Kinematics, stability and safety monitor of the tricycle pallet AMR defined in config.js.
// No rendering dependencies — runs in Node and browser.

import { AMR, MAST, FORK, PALLET, DOCK, SAFE, G } from './config.js';

const { abs, sin, cos, tan, atan, hypot, min, max, PI } = Math;
const [XT, ZT] = MAST.pivot;

export const A_TCP = MAST.aFace + FORK.loadCentre; // tilt pivot -> TCP along the fork axis
export const REACH = XT + A_TCP; // load-axle midpoint -> TCP at tilt 0
export const A_TIP = MAST.aFace + FORK.length; // tilt pivot -> fork tips
export const D_IN = DOCK.standoff + FORK.length - FORK.faceGap; // creep distance pre-dock -> docked

// ---------- chassis (spec §5.1) ----------
// The steered wheel sits BEHIND the reference point, so a positive (counter-clockwise) steer
// angle turns the vehicle clockwise: ω = −v_s·sin φ / L.
export const steerOf = (k) => -atan(AMR.L * k); // steer angle for a path curvature κ = ω/v
export const curvatureOf = (phi) => -tan(phi) / AMR.L;
export const bodyTwist = (vs, phi) => ({ v: vs * cos(phi), w: (-vs * sin(phi)) / AMR.L });
export function wheelCmd(v, k) {
  const phi = steerOf(k);
  return { phi, vs: v / cos(phi) }; // = v·√(1 + L²κ²), signed like v
}
// Rim speeds of the two load wheels for a body twist.
export const loadWheelSpeeds = (v, w) => ({ left: v - (w * AMR.track) / 2, right: v + (w * AMR.track) / 2 });
// det ∂(v, ω)/∂(v_s, φ): zero at standstill, where the steer angle stops mattering.
export const steerMapDet = (vs) => -vs / AMR.L;

// Pose after dt at constant body twist (exact circular arc).
// The chord of the arc has length v·dt·sinc(ω·dt/2) and points along the mid heading.
export function stepPose(p, v, w, dt) {
  const h = (w * dt) / 2, chord = v * dt * (abs(h) < 1e-4 ? 1 - (h * h) / 6 : sin(h) / h);
  return { x: p.x + chord * cos(p.th + h), y: p.y + chord * sin(p.th + h), th: p.th + w * dt };
}

// ---------- mast and fork (spec §5.2) ----------
// Mast frame {M}: origin on the tilt axis, x_m along the fork, z_m along the mast.
export function mastToBody(tilt, [xm, ym, zm]) {
  const c = cos(tilt), s = sin(tilt);
  return [XT + xm * c - zm * s, ym, ZT + xm * s + zm * c];
}
// TCP = point of the fork top face under the pallet centre, on the centreline between the tines.
export const tcpBody = (tilt, lift, shift) => mastToBody(tilt, [A_TCP, shift, lift - ZT]);
export const tipBody = (tilt, lift, shift) => mastToBody(tilt, [A_TIP, shift, lift - ZT]);
export function toWorld(pose, [bx, by, bz]) {
  const c = cos(pose.th), s = sin(pose.th);
  return [pose.x + c * bx - s * by, pose.y + s * bx + c * by, bz];
}
export const tcpWorld = (pose, tilt, lift, shift) => toWorld(pose, tcpBody(tilt, lift, shift));

// Inverse: lift command for a TCP height at a given tilt; base pose for a TCP position at tilt 0.
export const liftFor = (zTcp, tilt) => ZT + (zTcp - ZT - A_TCP * sin(tilt)) / cos(tilt);
export function baseFor(tx, ty, th, shift = 0) {
  return { x: tx - REACH * cos(th) + shift * sin(th), y: ty - REACH * sin(th) - shift * cos(th), th };
}
export const inRange = (v, lim, eps = 1e-9) => v >= lim.min - eps && v <= lim.max + eps;

// d(x_B, z_B of the TCP)/d(tilt, lift). det = −(lift − z_t): singular when the fork plane is
// at the height of the tilt axis, where tilting and lifting move the TCP in the same direction.
export function mastJacobian(tilt, lift) {
  const c = cos(tilt), s = sin(tilt), d = lift - ZT;
  return [[-A_TCP * s - d * c, -s], [A_TCP * c - d * s, c]];
}
// d(ẋ, ẏ, θ̇ of the TCP in {B})/d(v, ω, shift rate) at tilt 0. det = −1 everywhere.
export const planarJacobian = (shift) => [[1, -shift, 0], [0, REACH, 1], [0, 1, 0]];
export const det2 = (m) => m[0][0] * m[1][1] - m[0][1] * m[1][0];
export const det3 = (m) =>
  m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
  m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
  m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

// Pose of a carried pallet (origin = centre of its underside): rigid on the fork.
export function carriedPallet(pose, tilt, lift, shift) {
  const [x, y, z] = toWorld(pose, mastToBody(tilt, [A_TCP, shift, lift - ZT - PALLET.pocketHi]));
  return { x, y, z, yaw: pose.th, pitch: tilt };
}

// Lift commands for a transfer at a support plane zs (spec §8.2), all at tilt 0.
export const liftTargets = (zs) => ({
  entry: zs + PALLET.pocketHi - FORK.cEntry, // fork passes through the pockets of a resting pallet
  carry: zs + PALLET.pocketHi + FORK.cLift, // carried pallet passes over the support
});

// ---------- stability and load holding (spec §7) ----------
// load = { mass, x, z }: mass and centre of gravity of the load in {B}.
export function stability(load) {
  const [cx, , cz] = AMR.cg, M = AMR.mass, m = load?.mass ?? 0, lx = load?.x ?? 0, lz = load?.z ?? 0;
  const restoring = M * -cx, tipping = m * lx; // moments about the load axle, per g
  const x = (M * cx + m * lx) / (M + m), z = (M * cz + m * lz) / (M + m);
  const halfW = (AMR.track / 2) * (1 + x / AMR.L); // support triangle half-width under the combined CG
  return {
    longSF: m > 0 ? restoring / tipping : Infinity,
    brakeTip: (G * (restoring - tipping)) / (M * cz + m * lz), // forward deceleration that lifts the drive wheel
    latTip: (G * halfW) / z, // lateral acceleration that tips over a triangle edge
    cg: [x, z],
    driveWheelLoad: (-(M * cx + m * lx) / AMR.L) * G, // N on the drive wheel
  };
}
// Deceleration at which a pallet starts to slide forward off the tines.
export const slideDecel = (tilt) => G * tan(atan(FORK.mu) + tilt);

// ---------- plan geometry ----------
export function rectPoly(cx, cy, th, x0, x1, halfW) {
  const c = cos(th), s = sin(th);
  return [[x0, -halfW], [x1, -halfW], [x1, halfW], [x0, halfW]].map(([x, y]) => [cx + c * x - s * y, cy + s * x + c * y]);
}
export const bodyPoly = (pose) => rectPoly(pose.x, pose.y, pose.th, AMR.body.xr, AMR.body.xf, AMR.body.w / 2);
export const palletPoly = (p) => rectPoly(p.x, p.y, p.yaw, -PALLET.sx / 2, PALLET.sx / 2, PALLET.sy / 2);

function segDist(p, a, b) {
  const dx = b[0] - a[0], dy = b[1] - a[1];
  const t = max(0, min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)));
  return hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy);
}
function overlap(A, B) {
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
// Distance between two convex polygons, 0 if they overlap.
export function polyDist(A, B) {
  if (overlap(A, B)) return 0;
  let d = Infinity;
  for (const [P, Q] of [[A, B], [B, A]])
    for (const p of P) for (let i = 0; i < Q.length; i++) d = min(d, segDist(p, Q[i], Q[(i + 1) % Q.length]));
  return d;
}

// ---------- safety monitor (spec §9) ----------
// Reads joint positions and speeds, the pose, the load sensor and the station signals only —
// it knows nothing about the sequencer.
//   s   = { pose, v, phi, vs, steerRate, tilt, lift, shift, mastRate: [tilt, lift, shift], loaded }
//   env = { stations: [{ id, kind, x, y, th, zs, pallet, locked }], frames: [poly], pallets: [{ poly, station, carried }] }
// station.zs = support plane of the next transfer, station.pallet = pallet resting on it (or null).
// Returns { bad: [violated rules], clearance } (clearance in the plan, m).
const angDiff = (a, b) => abs(((a - b + PI) % (2 * PI) + 2 * PI) % (2 * PI) - PI);
export function monitor(s, env) {
  const bad = [], D = AMR.drive, E = 1e-6;
  const moving = abs(s.v) > 1e-4;

  if (!inRange(s.phi, AMR.steer, E) || !inRange(s.tilt, MAST.tilt, E) || !inRange(s.lift, MAST.lift, E) || !inRange(s.shift, MAST.shift, E)) bad.push('R1 joint limit');

  if (abs(s.v) > (s.loaded ? D.loaded.v : D.empty.v) + E) bad.push('R2 path speed above the limit for the load state');
  if (s.v < -AMR.vRev - E) bad.push('R2 reverse speed');
  if (abs(s.vs) > AMR.wheelV + E) bad.push('R2 drive wheel speed');
  if (abs(s.steerRate) > AMR.steer.v + E) bad.push('R2 steer rate');
  if (abs(s.mastRate[0]) > MAST.tilt.v + E || abs(s.mastRate[1]) > MAST.lift.v + E || abs(s.mastRate[2]) > MAST.shift.v + E) bad.push('R2 mast axis speed');

  if (s.v * s.v * abs(curvatureOf(s.phi)) > AMR.aLat + 1e-3) bad.push('R3 lateral acceleration');
  if (moving && s.mastRate.some((r) => abs(r) > E)) bad.push('R4 mast axis moving while the vehicle moves');

  const travelPose = abs(s.lift - MAST.travel.lift) <= MAST.lift.tol && abs(s.tilt - MAST.travel.tilt) <= MAST.tilt.tol && abs(s.shift) <= MAST.shift.tol;
  if (abs(s.v) > D.creep.v + E && !travelPose) bad.push('R5 above creep speed with the mast not in the travel pose');

  // fork zones
  const tip = toWorld(s.pose, tipBody(s.tilt, s.lift, s.shift));
  const tcp = toWorld(s.pose, tcpBody(s.tilt, s.lift, s.shift));
  let zone = null;
  for (const st of env.stations) {
    const c = cos(st.th), sn = sin(st.th);
    const u = (tip[0] - st.x) * c + (tip[1] - st.y) * sn, w = -(tip[0] - st.x) * sn + (tip[1] - st.y) * c;
    if (u <= -PALLET.sx / 2 - DOCK.zoneIn || u > PALLET.sx || abs(w) > PALLET.sy || angDiff(s.pose.th, st.th) > PI / 4) continue;
    zone = st.id;
    if (abs(s.v) > D.creep.v + E) bad.push('R6 above creep speed in a fork zone');
    if (abs(s.phi) > AMR.steer.tol || angDiff(s.pose.th, st.th) > 2 * AMR.headTol) bad.push('R6 not straight in a fork zone');
    if (abs(s.tilt) > MAST.tilt.tol) bad.push('R6 mast tilted in a fork zone');
    if (st.kind === 'pick' && !st.locked) bad.push('R8 in a pick station whose conveyor is not locked');

    const inside = u > -PALLET.sx / 2; // tips beyond the near face of the pallet position
    if (st.pallet && !s.loaded && inside) {
      // tines inside the pockets of a resting pallet
      const wp = -(st.pallet.x - st.x) * sn + (st.pallet.y - st.y) * c; // its lateral offset
      const side = PALLET.openY[0] + DOCK.latMargin + FORK.tineW / 2, far = PALLET.openY[1] - DOCK.latMargin - FORK.tineW / 2;
      const lat = abs(w - wp); // centreline offset between fork and pallet
      if (FORK.tineY - lat < side || FORK.tineY + lat > far) bad.push('R7 tine outside the fork window (lateral)');
      if (s.lift - FORK.thick < st.zs + PALLET.pocketLo + DOCK.zMargin - E) bad.push('R7 tine below the fork window');
      if (s.lift > st.zs + PALLET.pocketHi + E) bad.push('R7 tine above the fork window without load signal');
    }
    if (s.loaded) {
      const under = tcp[2] - PALLET.pocketHi;
      if (st.pallet) bad.push('R9 carried pallet in an occupied station');
      if (under < st.zs - E) bad.push('R7 carried pallet below the support plane');
      if (under > st.zs + FORK.cLift + SAFE.liftWin) bad.push('R7 carried pallet too high in the station');
    }
  }

  // plan clearance: chassis and carried pallet against conveyor frames and every other pallet
  const body = bodyPoly(s.pose);
  const mine = env.pallets.find((p) => p.carried);
  let clearance = Infinity;
  for (const f of env.frames) {
    clearance = min(clearance, polyDist(body, f.poly));
    if (mine && mine.z < f.top) clearance = min(clearance, polyDist(mine.poly, f.poly));
  }
  for (const p of env.pallets) {
    if (p.carried) continue;
    clearance = min(clearance, polyDist(body, p.poly));
    if (mine && p.station !== zone) clearance = min(clearance, polyDist(mine.poly, p.poly));
  }
  if (clearance < SAFE.clear - E) bad.push('R10 plan clearance');
  return { bad, clearance, zone };
}
