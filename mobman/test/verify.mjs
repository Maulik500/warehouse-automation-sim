// Headless verification of the mobile depalletizer: arm kinematics and singularities, recipes,
// reach, base kinematics, route, docking geometry, gripper, stability, the safety monitor and
// a long production run over every recipe. `npm run verify:mobman`

import { AMR, ARM, TOOL, DECK, PALLET, RECIPES, BOXLIM, SRC, DST, CYCLE, DOCK, TURN, SAFE, DEG, G } from '../js/config.js';
import * as A from '../js/arm.js';
import * as B from '../js/base.js';
import * as C from '../js/clearance.js';
import { layerPattern, layerExtent, stackHeight, perLayer, checkRecipe, palletBoxes, deckSlots } from '../js/recipes.js';
import { Cell, monitor, buildRoute, ikBody, Q_TRAVEL, QREF, SRC_DOCK, DST_DOCK, ptpTime } from '../js/sim.js';
import { tangentLength } from '../../amr/js/path.js';

const { abs, max, min, hypot, sin, cos, PI, sqrt, atan2 } = Math;
let fails = 0;
const check = (name, ok, detail = '') => {
  if (!ok) fails++;
  console.log(`${ok ? '  ok ' : ' FAIL'}  ${name}${detail ? '  — ' + detail : ''}`);
};
const f = (x, n = 3) => x.toFixed(n);
const e = (x) => x.toExponential(1);
const dg = (q) => q.map((v) => f(v / DEG, 2)).join(', ');
const rng = (seed) => () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
const HOME = { x: 0, y: 0, th: 0 };

// ---------- 1. arm kinematics ----------
console.log('\n1. Arm: forward / inverse kinematics');
{
  const { d1, a2, a3, d4, d5, d6 } = ARM;
  const z = A.fk([0, 0, 0, 0, 0, 0]);
  const want = [-(a2 + a3), -(d4 + d6 + TOOL.tcp), d1 - d5];
  check('zero-pose TCP', hypot(...z.tcp.map((v, i) => v - want[i])) < 1e-12, `(${z.tcp.map((v) => f(v)).join(', ')}) in {A}`);
  const up = A.fk([0, -PI / 2, 0, -PI / 2, 0, 0]);
  check('upright pose: flange on top, tool axis sideways', abs(up.flange[2] - (d1 + a2 + a3 + d5)) < 1e-12 && abs(up.flange[1] + d4 + d6) < 1e-12, `flange (${up.flange.map((v) => f(v)).join(', ')})`);
  // scene-graph chain (what main.js builds: one group per DH row) against an explicit closed form
  const r = rng(11);
  let eFk = 0;
  for (let i = 0; i < 5000; i++) {
    const q = Array.from({ length: 6 }, () => (r() * 2 - 1) * PI);
    const [q1, q2, q3, q4] = q, q23 = q2 + q3, q234 = q23 + q4;
    const z1 = [sin(q1), -cos(q1), 0], rad = [cos(q1), sin(q1), 0];
    const pr = -a2 * cos(q2) - a3 * cos(q23) + d5 * sin(q234), pz = d1 - a2 * sin(q2) - a3 * sin(q23) - d5 * cos(q234);
    const o5 = [rad[0] * pr + z1[0] * d4, rad[1] * pr + z1[1] * d4, pz];
    eFk = max(eFk, hypot(...A.fk(q).o5.map((v, k) => v - o5[k])));
  }
  check('DH chain = closed-form wrist point O5, 5000 random poses', eFk < 1e-12, e(eFk) + ' m');
  let worst = 0, n = 0, miss = 0;
  for (let i = 0; i < 20000; i++) {
    const q = [0, 1, 2, 3, 4, 5].map((k) => ARM.qMin[k] * 0.45 + r() * (ARM.qMax[k] - ARM.qMin[k]) * 0.45);
    const s = A.singularity(q);
    if (abs(s.elbow) < 0.05 || abs(s.wrist) < 0.05 || abs(s.shoulder) < 0.05) continue;
    const fkq = A.fk(q), q2 = A.ik(fkq.tcp, fkq.R, q, A.branchOf(q));
    if (!q2) { miss++; continue; }
    n++;
    worst = max(worst, ...q2.map((v, k) => abs(v - q[k])));
  }
  check(`IK(FK(q)) = q on all 8 posture branches, ${n} random poses`, miss === 0 && worst < 1e-9, `worst ${e(worst)} rad, ${miss} misses`);
  // all 8 branches of one pose are distinct and all reproduce the pose
  const tgt = A.fk([0.4, -1.1, 1.3, 0.5, 1.2, 0.3]);
  const sols = A.BRANCHES.map((b) => A.ik(tgt.tcp, tgt.R, [0, 0, 0, 0, 0, 0], b)).filter(Boolean);
  const eAll = max(...sols.map((q) => hypot(...A.fk(q).tcp.map((v, k) => v - tgt.tcp[k]))));
  check('one pose, 8 posture branches, all valid', sols.length === 8 && eAll < 1e-12, `${sols.length} solutions, worst ${e(eAll)} m`);
}

// ---------- 2. Jacobian and singularities ----------
console.log('\n2. Arm: Jacobian and singularities');
{
  const r = rng(5);
  let eDet = 0, eJ = 0;
  for (let i = 0; i < 3000; i++) {
    const q = Array.from({ length: 6 }, () => (r() * 2 - 1) * 3);
    const s = A.singularity(q), J = A.jacobian(q);
    eDet = max(eDet, abs(A.det(J) - ARM.a2 * ARM.a3 * s.elbow * s.wrist * s.shoulder));
    if (i < 300) {
      const h = 1e-6;
      for (let k = 0; k < 6; k++) {
        const qp = [...q], qm = [...q];
        qp[k] += h; qm[k] -= h;
        const p = A.fk(qp).flange, m = A.fk(qm).flange;
        for (let c = 0; c < 3; c++) eJ = max(eJ, abs((p[c] - m[c]) / (2 * h) - J[c][k]));
      }
    }
  }
  check('det J = a2·a3·sin q3·sin q5·(−a2·c2 − a3·c23 + d5·s234)', eDet < 1e-12, e(eDet));
  check('Jacobian vs finite differences', eJ < 1e-8, e(eJ));
  // each factor alone makes J singular
  const base = [0.3, -0.9, 1.2, 0.4, 1.1, 0.2];
  const el = [...base]; el[2] = 0;
  const wr = [...base]; wr[4] = 0;
  check('elbow (q3 = 0) and wrist (q5 = 0) singular poses', abs(A.det(A.jacobian(el))) < 1e-12 && abs(A.det(A.jacobian(wr))) < 1e-12, `det ${e(abs(A.det(A.jacobian(el))))}, ${e(abs(A.det(A.jacobian(wr))))}`);
  // shoulder: O5 on the cylinder of radius d4 around the J1 axis (solve the factor for q4)
  const sh = [0.3, -PI / 2, 0.1, 0, 1.1, 0.2];
  sh[3] = Math.asin((ARM.a2 * cos(sh[1]) + ARM.a3 * cos(sh[1] + sh[2])) / ARM.d5) - sh[1] - sh[2];
  const o5 = A.fk(sh).o5;
  check('shoulder singular pose: O5 at radius d4 from the J1 axis', abs(A.det(A.jacobian(sh))) < 1e-12 && abs(hypot(o5[0], o5[1]) - ARM.d4) < 1e-12, `det ${e(abs(A.det(A.jacobian(sh))))}, radius ${f(hypot(o5[0], o5[1]), 4)} m`);
  // tool pointing down fixes q5 and q2 + q3 + q4
  const q = ikBody([0.3, 1.2, 1.1], 0.4);
  check('tool down ⇒ q5 = +90°, q2 + q3 + q4 = +90°', abs(q[4] - PI / 2) < 1e-12 && abs(q[1] + q[2] + q[3] - PI / 2) < 1e-12, dg(q));
}

// ---------- 3. recipes ----------
console.log('\n3. Recipes');
for (const rc of RECIPES) {
  const bad = checkRecipe(rc, BOXLIM, TOOL), [ex, ey] = layerExtent(rc), n = perLayer(rc);
  // no two boxes of a layer overlap
  let overlap = 0;
  for (let k = 0; k < 2; k++) {
    const L = layerPattern(rc, k).map((b) => ({ ...b, s: Math.abs(Math.sin(b.yaw)) > 0.5 ? [rc.w, rc.l] : [rc.l, rc.w] }));
    for (let i = 0; i < L.length; i++) for (let j = i + 1; j < L.length; j++)
      if (abs(L[i].x - L[j].x) < (L[i].s[0] + L[j].s[0]) / 2 - 1e-9 && abs(L[i].y - L[j].y) < (L[i].s[1] + L[j].s[1]) / 2 - 1e-9) overlap++;
  }
  check(`${rc.id} ${rc.sku}`, bad.length === 0 && overlap === 0, bad.join('; ') || `${(rc.l * 1000).toFixed(0)}×${(rc.w * 1000).toFixed(0)}×${(rc.h * 1000).toFixed(0)}, ${rc.mass} kg, ${n}/layer × ${rc.layers} = ${n * rc.layers} boxes, layer ${f(ex)} × ${f(ey)}, height ${f(stackHeight(rc))} m, ${n * rc.layers * rc.mass + PALLET.mass} kg`);
}

// ---------- 4. reach ----------
console.log('\n4. Reach of every pick and place position (static)');
{
  let n = 0, miss = 0, mE = 9, mS = 9, zHi = 0, zLo = 9, rMax = 0, rMin = 9;
  const qLo = Array(6).fill(Infinity), qHi = Array(6).fill(-Infinity);
  const probe = (p, yaw) => {
    const q = ikBody(p, yaw) || ikBody(p, yaw + PI);
    n++;
    if (!q) { miss++; return; }
    const s = A.singularity(q);
    mE = min(mE, abs(s.elbow)); mS = min(mS, abs(s.shoulder));
    q.forEach((v, i) => { qLo[i] = min(qLo[i], v); qHi[i] = max(qHi[i], v); });
    zHi = max(zHi, p[2]); zLo = min(zLo, p[2]);
    const r = hypot(p[0] - ARM.base[0], p[1] - ARM.base[1]);
    rMax = max(rMax, r); rMin = min(rMin, r);
  };
  for (const rc of RECIPES) {
    const zDeck = DECK.z + rc.h;
    for (const b of palletBoxes(rc)) {
      const zTop = SRC.z + b.z + rc.h / 2;
      for (const dx of [-SRC.scatter, SRC.scatter]) for (const dy of [-SRC.scatter, SRC.scatter])
        for (const z of [zTop, zTop + rc.h + CYCLE.lift]) probe([SRC.x + b.x + dx, SRC.y + b.y + dy, z], b.yaw);
    }
    const zTopMax = SRC.z + stackHeight(rc);
    for (const s of deckSlots(rc)) for (const z of [zDeck, zTopMax + rc.h + CYCLE.lift]) probe([s.x, s.y, z], s.yaw);
  }
  check(`all ${n} probe poses reachable in the one posture branch`, miss === 0, `${miss} unreachable`);
  check('static singularity margins over the whole task space', mE >= ARM.minElbow && mS >= ARM.minShoulder, `|sin q3| ≥ ${f(mE)}, shoulder ≥ ${f(mS)} m, |sin q5| = 1`);
  console.log(`      TCP height ${f(zLo)} … ${f(zHi)} m, radius from the J1 axis ${f(rMin)} … ${f(rMax)} m`);
  console.log(`      joint range needed: ${qLo.map((v, i) => `J${i + 1} ${f(v / DEG, 0)}…${f(qHi[i] / DEG, 0)}`).join(', ')}`);
  check('travel pose: whole arm inside the chassis outline', C.armOverhang(Q_TRAVEL) < -0.02, `${(-1000 * C.armOverhang(Q_TRAVEL)).toFixed(0)} mm inside; q = ${dg(Q_TRAVEL)}`);
  const top = max(...B.armCapsules(Q_TRAVEL).flatMap((c) => [c.a[2] + c.r, c.b[2] + c.r])) + ARM.base[2];
  console.log(`      overall height in the travel pose ${f(top, 2)} m; TCP ${f(ARM.travelTcp[2] - DECK.z - BOXLIM.hMax)} m above the tallest layer on the deck`);
}

// ---------- 5. base ----------
console.log('\n5. Differential-drive base');
{
  const r = rng(3);
  let eRt = 0;
  for (let i = 0; i < 20000; i++) {
    const v = (r() * 2 - 1) * 2, w = (r() * 2 - 1) * 2, wh = B.wheelsOfTwist(v, w), t = B.twistOfWheels(wh.vL, wh.vR);
    eRt = max(eRt, abs(t.v - v), abs(t.w - w));
  }
  check('wheel speeds ↔ body twist, 20 000 random twists', eRt < 1e-12, e(eRt));
  check('wheel map determinant is constant (no drive singularity)', abs(B.WHEEL_MAP_DET - 1 / AMR.track) < 1e-15, `det = 1/b = ${f(B.WHEEL_MAP_DET)}`);
  // exact arc integration vs many small Euler steps
  let p = { ...HOME }, x = 0, y = 0, th = 0;
  const v = 0.7, w = 0.9, T = 2, n = 2e5;
  p = B.stepPose(p, v, w, T);
  for (let i = 0; i < n; i++) { const tm = th + (w * T) / n / 2; x += (v * T / n) * cos(tm); y += (v * T / n) * sin(tm); th += (w * T) / n; }
  check('exact arc pose step vs fine integration', hypot(p.x - x, p.y - y) < 1e-9 && abs(p.th - th) < 1e-9, e(hypot(p.x - x, p.y - y)) + ' m');
  const pv = B.stepPose(HOME, 0, 1, 1), one = B.wheelsOfTwist(0.5, 1);
  check('pivot about the axle midpoint: position unchanged', hypot(pv.x, pv.y) < 1e-15 && abs(pv.th - 1) < 1e-15);
  check('ICR on a wheel: that wheel stands still', abs(one.vL) < 1e-15, `v = 0.5, ω = 1 → vL ${f(one.vL)}, vR ${f(one.vR)}`);
  const path = buildRoute(), end = path.end, Tt = tangentLength(-PI / 2, TURN.R, TURN.Lc);
  check('route ends exactly on the destination dock pose', hypot(end.x - DST_DOCK.x, end.y - DST_DOCK.y) < 1e-9 && abs(end.th - DST_DOCK.th) < 1e-12, `${e(hypot(end.x - DST_DOCK.x, end.y - DST_DOCK.y))} m; length ${f(path.length)} m, T = ${f(Tt, 4)} m`);
  // non-holonomic constraint and curvature continuity along the route
  let lat = 0, jump = 0, prev = path.curv(0);
  const m = 100000, h = path.length / m;
  for (let i = 1; i <= m; i++) {
    if (i % 100 === 0) { // chord over 100 steps against the heading at its midpoint
      const a = path.at((i - 100) * h), b2 = path.at(i * h), tm = path.heading((i - 50) * h);
      lat = max(lat, abs(-(b2.x - a.x) * sin(tm) + (b2.y - a.y) * cos(tm)) / (100 * h));
    }
    const c = path.curv(i * h);
    jump = max(jump, abs(c.k - prev.k));
    prev = c;
  }
  check('no lateral motion of the reference point; curvature continuous', lat < 1e-4 && jump < 1e-3, `lateral/forward ${e(lat)}, largest κ step ${e(jump)} 1/m`);
  const straightSrc = path.length, s1 = DST_DOCK.x - SRC_DOCK.x - Tt, s3 = SRC_DOCK.y - Tt - DST_DOCK.y;
  check('both pre-dock points are on straight pieces', s1 > DOCK.standoff && s3 > DOCK.standoff, `straight ${f(s1)} m after the source, ${f(s3)} m before the destination, standoff ${DOCK.standoff} m`);
  const cap = B.speedCap(path, DOCK.standoff, straightSrc - DOCK.standoff, AMR.drive.loaded);
  console.log(`      speed cap on the route: ${f(cap.v)} m/s (κ_max ${f(cap.kMax)} 1/m): lateral ${f(sqrt(AMR.aLat / cap.kMax))}, wheel ${f(AMR.wheelV / (1 + cap.kMax * AMR.track / 2))}, yaw rate ${f(AMR.omega / cap.kMax)} m/s`);
}

// ---------- 6. docking geometry ----------
console.log('\n6. Docking geometry');
{
  const cl = B.toWorld(DST_DOCK, [AMR.body.xf, DECK.yc, DECK.z]);
  check('destination dock: deck centreline on the conveyor centreline, same height', abs(cl[0] - DST.x) < 1e-12 && abs(DECK.z - DST.z) < 1e-12, `offset ${e(abs(cl[0] - DST.x))} m, gap ${(1000 * (cl[1] - DST.yEnd)).toFixed(0)} mm`);
  check('roller beds have the same clear width', DECK.w === DST.w, `${DECK.w} m`);
  const armW = B.toWorld(SRC_DOCK, ARM.base), near = SRC.y - PALLET.sy / 2;
  check('source dock: J1 axis to the pallet', abs(near - armW[1] - SRC.armGap) < 1e-12, `${f(near - armW[1])} m to the near side, ${f(SRC.y + PALLET.sy / 2 - armW[1])} m to the far side`);
  const side = C.chassisClearance(SRC_DOCK, C.SRC_FRAME);
  check('source dock: chassis ↔ conveyor frame', side >= SAFE.clear, `${(1000 * side).toFixed(0)} mm`);
  const pre = buildRoute().at(DOCK.standoff);
  check('source pre-dock point: chassis is past the conveyor end', pre.x + AMR.body.xr > SRC.endX, `${(1000 * (pre.x + AMR.body.xr - SRC.endX)).toFixed(0)} mm`);
  check('deck holds a full pallet layer', DECK.layer.sx >= PALLET.sx && DECK.w >= PALLET.sy + 0.02 && DECK.stopX - DECK.backX >= PALLET.sx, `${f(DECK.stopX - DECK.backX)} × ${DECK.w} m between the stops and guides`);
  check('B1 beam is inside the pallet at the end stop, for every scatter', SRC.x - SRC.scatter + PALLET.sx / 2 > SRC.b1x && SRC.x + SRC.scatter - PALLET.sx / 2 < SRC.b1x, `beam ${(1000 * (SRC.x + PALLET.sx / 2 - SRC.b1x)).toFixed(0)} mm behind the leading face`);
}

// ---------- 7. gripper ----------
console.log('\n7. Vacuum gripper');
{
  const F = B.VAC_FORCE;
  check('theoretical holding force', abs(F - TOOL.cups * TOOL.vacuum * PI * TOOL.cupDia ** 2 / 4) < 1e-9, `${f(F, 0)} N`);
  const sf = F / (BOXLIM.mMax * G);
  check('static safety factor, heaviest box', sf >= 2 * TOOL.sfMin, `${f(sf, 2)}`);
  const aAllow = (F / TOOL.sfMin / BOXLIM.mMax - G) * TOOL.mu;
  console.log(`      horizontal acceleration allowed at SF ${TOOL.sfMin} with ${BOXLIM.mMax} kg: ${f(aAllow, 2)} m/s²`);
  check('arm payload: heaviest box + gripper', BOXLIM.mMax + TOOL.mass <= ARM.payload, `${BOXLIM.mMax + TOOL.mass} of ${ARM.payload} kg`);
  const small = RECIPES.reduce((a, b) => (a.l * a.w < b.l * b.w ? a : b));
  check('pad inside the smallest box top, 20 mm each side', TOOL.padX <= small.l - 0.04 + 1e-9 && TOOL.padY <= small.w - 0.04 + 1e-9, `pad ${TOOL.padX} × ${TOOL.padY} on ${small.l} × ${small.w}`);
}

// ---------- 8. stability ----------
console.log('\n8. Stability (quasi-static)');
{
  const emptyT = B.stability({ q: Q_TRAVEL, held: null, deck: [] });
  const heavy = RECIPES.reduce((a, b) => (perLayer(a) * a.mass > perLayer(b) * b.mass ? a : b));
  const deck = deckSlots(heavy).map((s) => ({ m: heavy.mass, x: s.x, y: s.y, z: s.z }));
  const fullT = B.stability({ q: Q_TRAVEL, held: null, deck });
  check('travelling, empty', emptyT.brakeTip / AMR.eStopDecel > 2 && emptyT.sideTip / AMR.aLat > 2, `${f(emptyT.m, 0)} kg, CG (${emptyT.cg.map((v) => f(v, 2)).join(', ')}); brake tip ${f(emptyT.brakeTip, 1)}, side tip ${f(emptyT.sideTip, 1)} m/s²`);
  check(`travelling, heaviest layer (${heavy.id}, ${perLayer(heavy) * heavy.mass} kg)`, fullT.brakeTip / AMR.eStopDecel > 2 && fullT.sideTip / AMR.aLat > 2, `${f(fullT.m, 0)} kg, CG (${fullT.cg.map((v) => f(v, 2)).join(', ')}); SF braking ${f(fullT.brakeTip / AMR.eStopDecel, 1)}, turning ${f(fullT.sideTip / AMR.aLat, 1)}`);
  // worst arm pose: far top corner of the pallet with the heaviest box
  let worst = { margin: 9 };
  for (const rc of RECIPES) for (const b of palletBoxes(rc)) {
    const q = ikBody([b.x, SRC.y + b.y, SRC.z + b.z + rc.h / 2 + 0.3], b.yaw);
    if (!q) continue;
    const st = B.stability({ q, held: { m: BOXLIM.mMax, h: rc.h }, deck: [] });
    if (st.margin < worst.margin) worst = st;
  }
  check('arm at full reach over the pallet with 15 kg, deck empty', worst.margin > 0.15, `CG ${f(worst.cg[1])} m off centre, ${f(worst.margin)} m inside the tipping edge`);
  check('cartons on braked rollers under emergency braking', fullT.slide / AMR.eStopDecel > 1.5, `slide at ${f(fullT.slide, 2)} m/s², SF ${f(fullT.slide / AMR.eStopDecel, 2)} (and the front stop is raised)`);
  check('cartons in a turn', fullT.slide / AMR.aLat > 2, `SF ${f(fullT.slide / AMR.aLat, 1)} (and side guides)`);
}

// ---------- 9. safety monitor ----------
console.log('\n9. Safety monitor: negative tests');
{
  const ok = {
    q: [...Q_TRAVEL], qd: [0, 0, 0, 0, 0, 0], qdd: [0, 0, 0, 0, 0, 0], v: 0, w: 0, k: 0, vL: 0, vR: 0, zone: null, loaded: false, docked: null,
    srcLocked: false, dstReserved: false, armMoving: false, armTravel: true, vac: false, held: false, heldMass: 0, sing: A.singularity(Q_TRAVEL), tilt: 0,
    sf: Infinity, deckU: 0, stop: 1, armClear: Infinity, carryClear: Infinity, chassisClear: 1, deckMass: 0, deckCount: 0, stab: { margin: 0.4 },
  };
  const work = { ...ok, docked: 'src', zone: 'src', srcLocked: true, armMoving: true, armTravel: false };
  const cases = [
    ['legal: parked', ok, null],
    ['legal: driving empty at 1.5 m/s', { ...ok, v: 1.5, vL: 1.5, vR: 1.5 }, null],
    ['legal: arm working at the source', { ...work, vac: true, held: true, heldMass: 15, sf: 2.5, armClear: 0.05, carryClear: 0.08 }, null],
    ['legal: transfer at the destination', { ...ok, docked: 'dst', zone: 'dst', dstReserved: true, deckU: 0.3, stop: 0, loaded: true, chassisClear: 0.03 }, null],
    ['J3 beyond its limit', { ...ok, q: [0, 0, 3, 0, 0, 0] }, 'M1'],
    ['J1 overspeed', { ...work, qd: [2.2, 0, 0, 0, 0, 0] }, 'M2'],
    ['loaded above 1.0 m/s', { ...ok, loaded: true, v: 1.2, vL: 1.2, vR: 1.2 }, 'M3'],
    ['creep zone above 0.2 m/s', { ...ok, zone: 'src', srcLocked: true, v: -0.4, vL: -0.4, vR: -0.4 }, 'M3'],
    ['lateral acceleration', { ...ok, v: 1.2, k: 0.6, w: 0.72, vL: 0.84, vR: 1.56 }, 'M3'],
    ['arm moving while driving', { ...work, v: 0.1, vL: 0.1, vR: 0.1 }, 'M4'],
    ['arm moving without Locked', { ...work, srcLocked: false }, 'M4'],
    ['driving with the arm not stowed', { ...ok, v: 0.5, vL: 0.5, vR: 0.5, armTravel: false }, 'M4'],
    ['driving with a box in the gripper', { ...ok, v: 0.5, vL: 0.5, vR: 0.5, vac: true, held: true, sf: 3 }, 'M4'],
    ['elbow nearly stretched', { ...work, sing: { elbow: 0.1, wrist: 1, shoulder: 0.5 } }, 'M5'],
    ['wrist near q5 = 0', { ...work, sing: { elbow: 0.8, wrist: 0.2, shoulder: 0.5 } }, 'M5'],
    ['O5 near the J1 axis cylinder', { ...work, sing: { elbow: 0.8, wrist: 1, shoulder: 0.04 } }, 'M5'],
    ['tool tilted with vacuum on', { ...work, vac: true, tilt: 2 * DEG }, 'M6'],
    ['vacuum safety factor 1.4', { ...work, vac: true, held: true, heldMass: 15, sf: 1.4 }, 'M7'],
    ['source zone without Locked', { ...ok, zone: 'src', v: -0.1, vL: -0.1, vR: -0.1 }, 'M8'],
    ['destination zone without DestReady', { ...ok, zone: 'dst', v: 0.1, vL: 0.1, vR: 0.1 }, 'M8'],
    ['rollers running while driving', { ...ok, v: 0.5, vL: 0.5, vR: 0.5, deckU: 0.3 }, 'M9'],
    ['front stop down away from the destination', { ...ok, stop: 0.5 }, 'M9'],
    ['arm 10 mm from an obstacle', { ...work, armClear: 0.01 }, 'M10'],
    ['carried box 10 mm above a box', { ...work, vac: true, held: true, sf: 3, carryClear: 0.01 }, 'M10'],
    ['chassis 20 mm from the source conveyor', { ...ok, chassisClear: 0.02 }, 'M10'],
    ['box of 18 kg', { ...work, vac: true, held: true, heldMass: 18, sf: 3 }, 'M11'],
    ['deck overloaded', { ...ok, deckMass: 160, deckCount: 8, loaded: true }, 'M11'],
    ['centre of gravity near the tipping edge', { ...ok, stab: { margin: 0.05 } }, 'M12'],
  ];
  let good = 0;
  for (const [name, s, rule] of cases) {
    const bad = monitor(s), pass = rule ? bad.some((b) => b.startsWith(rule + ' ')) : bad.length === 0;
    if (pass) good++; else check(name, false, bad.join('; ') || 'not detected');
  }
  check(`${cases.length} states classified correctly (${cases.filter((c) => !c[2]).length} legal, ${cases.filter((c) => c[2]).length} illegal)`, good === cases.length);
}

// ---------- 10. production run ----------
const HOURS = +process.env.HOURS || 3;
console.log(`\n10. Production run: ${HOURS} h at 240 Hz, seed 7, every recipe`);
{
  const cell = new Cell({ seed: 7, clearEvery: 2 }), DT = 1 / 240, st = cell.stats;
  const seenRc = new Map(), created = new Set(), orders = new Map(), perRc = new Map();
  let err = null, maxDeck = 0, lastTrip = 0, refs = {}, layerRule = 0, phaseT = {}, zMax = 0;
  try {
    for (let i = 0; i < HOURS * 3600 * 240; i++) {
      cell.step(DT);
      phaseT[cell.phase] = (phaseT[cell.phase] || 0) + DT;
      if (i % 24 === 0) {
        for (const b of cell.boxes) created.add(b.id);
        const p = cell.src.pallet;
        if (p) {
          seenRc.set(p.rc.id, (seenRc.get(p.rc.id) || 0));
          // top-down rule: no box on the pallet below a box that is gone... i.e. every remaining box is at or below the top layer in work
          const left = p.boxes.filter((b) => b.st === 'pallet');
          if (left.length) { const top = max(...left.map((b) => b.layer)); if (p.boxes.some((b) => b.st !== 'pallet' && b.layer < top)) layerRule++; }
        }
        zMax = max(zMax, cell.last.tcp[2]);
      }
      if (st.trips !== lastTrip) {
        lastTrip = st.trips;
        const o = cell.order;
        orders.set(o.n, (orders.get(o.n) || 0) + 1);
        perRc.set(o.rc.id, (perRc.get(o.rc.id) || 0) + o.n);
      }
      if (!refs.pick && cell.seg?.label === 'GRIP') refs.pick = { q: [...cell.q], tcp: [...cell.last.tcp] };
      if (!refs.place && cell.seg?.label === 'RELEASE') refs.place = { q: [...cell.q], tcp: [...cell.last.tcp] };
    }
  } catch (x) { err = x; }
  check('run completed without a planning or IK failure', !err && st.ikFail === 0, err ? err.message : `${st.ikFail} failures`);
  check('safety monitor violations', st.violations === 0, st.violations ? st.firstBad.slice(0, 3).join(' | ') : '0');
  check('arm moves only at standstill, base moves only with the arm stowed (raw checks)', st.armWhileMoving === 0 && st.baseWhileArm === 0, `${st.armWhileMoving} / ${st.baseWhileArm} steps`);
  check('source zone only with Locked (raw check)', st.zoneNoLock === 0, `${st.zoneNoLock} steps`);
  check('arm joint speeds within limits', max(...st.jv) <= 1.002, `peak / limit J1…J6: ${st.jv.map((v) => f(v, 2)).join(' · ')}`);
  check('arm joint accelerations within limits', max(...st.ja) <= 1.02, `peak / limit J1…J6: ${st.ja.map((v) => f(v, 2)).join(' · ')}`);
  check('joint range used', st.qLo.every((v, i) => v >= ARM.qMin[i]) && st.qHi.every((v, i) => v <= ARM.qMax[i]), st.qLo.map((v, i) => `J${i + 1} ${f(v / DEG, 0)}…${f(st.qHi[i] / DEG, 0)}°`).join(', '));
  check('singularity margins while the arm moves', st.minElbow >= ARM.minElbow && st.minWrist >= ARM.minWrist && st.minShoulder >= ARM.minShoulder, `elbow |sin q3| ${f(st.minElbow)}, wrist |sin q5| ${f(st.minWrist)}, shoulder ${f(st.minShoulder)} m`);
  check('tool vertical whenever the vacuum is on', st.maxTilt < 1e-6, `${e(st.maxTilt / DEG)}°`);
  check('vacuum safety factor', st.minSF >= TOOL.sfMin, `min ${f(st.minSF, 2)}; TCP peak ${f(st.maxTcpV, 2)} m/s, ${f(st.maxTcpA, 2)} m/s²`);
  check('pick and place position error', st.pickErr < 1e-9 && st.placeErr < 1e-9, `${e(st.pickErr)} / ${e(st.placeErr)} m`);
  check('arm clearance', st.minArmClear.d >= SAFE.armClear, `min ${(1000 * st.minArmClear.d).toFixed(0)} mm (${st.minArmClear.link} ↔ ${st.minArmClear.obstacle}, ${st.minArmClear.phase})`);
  check('carried box clearance during swings', st.minCarry.d >= SAFE.armClear, `min ${(1000 * st.minCarry.d).toFixed(0)} mm (over ${st.minCarry.obstacle}); swing height raised by at most ${(1000 * st.maxRaise).toFixed(0)} mm`);
  check('chassis clearance', st.minChassis >= DST.gap - 1e-9, `min ${(1000 * st.minChassis).toFixed(0)} mm (the docking gap)`);
  check('path speed limits', st.vFwd <= AMR.drive.empty.v + 1e-9 && st.vRev <= AMR.vRev + 1e-9 && st.vLoaded <= AMR.drive.loaded.v + 1e-9 && st.vZone <= AMR.drive.creep.v + 1e-9, `forward ${f(st.vFwd)}, reverse ${f(st.vRev)}, loaded ${f(st.vLoaded)}, in a zone ${f(st.vZone)} m/s`);
  check('wheel speed, yaw rate, lateral acceleration', st.wheelV <= AMR.wheelV && st.omega <= AMR.omega && st.aLat <= AMR.aLat + 1e-9, `${f(st.wheelV)} of ${AMR.wheelV} m/s, ${f(st.omega / DEG, 1)} of ${f(AMR.omega / DEG, 0)}°/s, ${f(st.aLat)} of ${AMR.aLat} m/s²`);
  check('path, wheel and yaw acceleration', st.aPath <= AMR.drive.empty.a + 1e-9 && st.wheelA <= AMR.wheelA && st.alpha <= AMR.alpha, `${f(st.aPath)} m/s², wheel ${f(st.wheelA)} of ${AMR.wheelA} m/s², yaw ${f(st.alpha / DEG, 1)} of ${f(AMR.alpha / DEG, 0)}°/s²`);
  check('dead reckoning from the wheel speeds vs the path pose', st.odoErr < 0.005, `worst ${e(st.odoErr)} m after ${f(st.dist / 1000, 2)} km`);
  check('stability during the run', st.minStab >= 0.10 && st.minBrakeTip / AMR.eStopDecel > 2, `CG ≥ ${f(st.minStab)} m inside the tipping edge; brake tip ≥ ${f(st.minBrakeTip, 1)}, side tip ≥ ${f(st.minSideTip, 1)} m/s²`);
  check('deck load', st.maxDeckMass <= DECK.maxMass && st.maxDeckCount <= DECK.cap, `max ${st.maxDeckMass} kg, ${st.maxDeckCount} boxes`);
  check('layers are taken strictly top-down', layerRule === 0, `${layerRule} samples`);
  const live = cell.boxes.length, onPal = cell.boxes.filter((b) => b.st === 'pallet').length;
  const gonePal = created.size - live - st.delivered;
  check('boxes conserved', st.boxes === st.delivered + cell.deck.boxes.length + cell.dst.boxes.length + (cell.held ? 0 : 0) && gonePal === 0, `${created.size} created = ${onPal} on the pallet + ${live - onPal} in transit + ${st.delivered} delivered`);
  check('every recipe was served', seenRc.size === RECIPES.length, `${[...perRc].map(([k, n]) => `${k}: ${n}`).join(', ')} boxes; ${st.pallets} pallets emptied`);
  const mean = (a) => a.reduce((x, y) => x + y, 0) / a.length;
  console.log(`      order sizes (boxes per trip): ${[...orders].sort((a, b) => a[0] - b[0]).map(([n, c]) => `${n}×${c}`).join(', ')}`);
  console.log(`      ${st.boxes} boxes in ${st.trips} trips = ${f(st.boxes / HOURS, 0)} boxes/h; box cycle ${f(mean(st.boxCycle), 1)} s mean (${f(min(...st.boxCycle), 1)} … ${f(max(...st.boxCycle), 1)}); trip ${f(mean(st.tripCycle), 0)} s mean (${f(min(...st.tripCycle), 0)} … ${f(max(...st.tripCycle), 0)})`);
  const tot = Object.values(phaseT).reduce((x, y) => x + y, 0);
  const grp = { 'arm loading': ['SETTLE', 'LOAD', 'STOW'], driving: ['TRAVEL', 'RETURN'], 'docking (creep)': ['DOCK_SRC', 'UNDOCK_SRC', 'DOCK_DST', 'UNDOCK_DST'], transfer: ['STOP_DOWN', 'TRANSFER', 'STOP_UP'], waiting: ['WAIT_PALLET', 'WAIT_DST'] };
  console.log(`      time split: ${Object.entries(grp).map(([k, v]) => `${k} ${f((100 * v.reduce((s, p) => s + (phaseT[p] || 0), 0)) / tot, 0)} %`).join(', ')}`);
  console.log(`      highest TCP ${f(zMax, 2)} m above the floor`);
  if (refs.pick) console.log(`      reference, first pick  (TCP ${refs.pick.tcp.map((v) => f(v)).join(', ')}): q = ${dg(refs.pick.q)}`);
  if (refs.place) console.log(`      reference, first place (TCP ${refs.place.tcp.map((v) => f(v)).join(', ')}): q = ${dg(refs.place.q)}`);
}

console.log(fails ? `\n${fails} CHECK(S) FAILED` : '\nall checks passed');
process.exit(fails ? 1 : 0);
