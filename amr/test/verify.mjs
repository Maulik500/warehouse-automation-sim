// Headless verification of the AMR model: paths, chassis and mast kinematics, routes,
// clearances, stability, the safety monitor and a long production run. `npm run verify:amr`

import { AMR, MAST, FORK, PALLET, CONV, DOCK, TURN, CELL, SAFE, DEG, G } from '../js/config.js';
import { Path, line, turn, speedCap, steerOfPath, tangentLength } from '../js/path.js';
import * as K from '../js/kinematics.js';
import { Cell, T_TURN, stationPoses, routePaths } from '../js/sim.js';

const { abs, max, min, hypot, sin, cos, PI, sqrt } = Math;
let fails = 0;
const check = (name, ok, detail = '') => {
  if (!ok) fails++;
  console.log(`${ok ? '  ok ' : ' FAIL'}  ${name}${detail ? '  — ' + detail : ''}`);
};
const f = (x, n = 3) => x.toFixed(n);
const e = (x) => x.toExponential(1);
const rng = (seed) => () => ((seed = (seed * 16807) % 2147483647) / 2147483647);

// ---------- 1. paths ----------
console.log('\n1. Continuous-curvature paths');
{
  const p = new Path({ x: 0, y: 0, th: 0 }, [line(1), ...turn(PI / 2), line(1)]);
  const end = p.end;
  check('90° turn ends at exactly 90°', abs(end.th - PI / 2) < 1e-12, e(abs(end.th - PI / 2)));
  check('turn is symmetric about the corner', abs(end.x - 1 - T_TURN) < 1e-9 && abs(end.y - 1 - T_TURN) < 1e-9, `T = ${f(T_TURN, 4)} m, turn length ${f(p.length - 2, 4)} m`);
  // independent integration of the heading with a much finer midpoint rule
  let x = 0, y = 0;
  const n = 400000, h = p.length / n;
  for (let i = 0; i < n; i++) { const th = p.heading((i + 0.5) * h); x += h * cos(th); y += h * sin(th); }
  check('position table (Simpson, 1 mm) vs fine integration', hypot(x - end.x, y - end.y) < 1e-8, e(hypot(x - end.x, y - end.y)) + ' m');
  // heading is the integral of curvature; curvature and its derivatives are continuous
  let eTh = 0, jump = [0, 0, 0], prev = p.curv(0), th = 0;
  const m = 200000, hs = p.length / m;
  for (let i = 1; i <= m; i++) {
    const c = p.curv(i * hs);
    th += (hs * (prev.k + c.k)) / 2;
    eTh = max(eTh, abs(th - p.heading(i * hs)));
    jump = [max(jump[0], abs(c.k - prev.k)), max(jump[1], abs(c.dk - prev.dk)), max(jump[2], abs(c.ddk - prev.ddk))];
    prev = c;
  }
  check('heading = ∫ curvature', eTh < 1e-9, e(eTh) + ' rad');
  check('κ, κ′, κ″ continuous along the whole path', jump[0] < 1e-4 && jump[1] < 1e-3 && jump[2] < 1e-3, `largest step between samples ${e(jump[0])} / ${e(jump[1])} / ${e(jump[2])}`);
  const cap = speedCap(p, 0, p.length, AMR.drive.empty);
  let phiMax = 0;
  for (let s = 0; s <= p.length; s += 0.001) phiMax = max(phiMax, abs(steerOfPath(p.curv(s)).phi));
  console.log(`      speed cap on a turn ${f(cap.v)} m/s; steer at κmax ${f(phiMax / DEG, 2)}°; wheel/path speed ${f(sqrt(1 + (AMR.L / TURN.R) ** 2))}`);
}

// ---------- 2. chassis kinematics ----------
console.log('\n2. Chassis kinematics');
{
  const r = rng(3);
  let eRound = 0, eCon = 0, eArc = 0, eDet = 0, signOk = true;
  for (let n = 0; n < 20000; n++) {
    const v = (2 * r() - 1) * 1.6, k = (2 * r() - 1) * 1.5;
    const { phi, vs } = K.wheelCmd(v, k), tw = K.bodyTwist(vs, phi);
    eRound = max(eRound, abs(tw.v - v), abs(tw.w - k * v), abs(K.curvatureOf(phi) - k));
    if (abs(k) > 1e-6 && Math.sign(phi) === Math.sign(k)) signOk = false;
    // constant twist for 0.5 s: exact arc vs 5000 small steps; lateral velocity stays zero
    const p0 = { x: r(), y: r(), th: 6 * r() }, T = 0.5, N = 5000;
    const p1 = K.stepPose(p0, tw.v, tw.w, T);
    let q = p0;
    for (let i = 0; i < N; i++) q = K.stepPose(q, tw.v, tw.w, T / N);
    eArc = max(eArc, hypot(q.x - p1.x, q.y - p1.y));
    const h = 1e-6, p2 = K.stepPose(p0, tw.v, tw.w, h);
    eCon = max(eCon, abs((-(p2.x - p0.x) * sin(p0.th) + (p2.y - p0.y) * cos(p0.th)) / h));
    // det of d(v, ω)/d(vs, φ) by finite differences
    const d = 1e-6, a = K.bodyTwist(vs + d, phi), b = K.bodyTwist(vs, phi + d);
    eDet = max(eDet, abs(((a.v - tw.v) * (b.w - tw.w) - (b.v - tw.v) * (a.w - tw.w)) / (d * d) - K.steerMapDet(vs)));
  }
  check('wheel commands ↔ body twist round trip, 20 000 random twists', eRound < 1e-12, e(eRound));
  check('steer left turns right (rear steer)', signOk);
  check('non-holonomic constraint: lateral speed of the reference point', eCon < 1e-5, e(eCon) + ' m/s');
  check('exact-arc pose step is consistent under subdivision', eArc < 1e-10, e(eArc) + ' m');
  check('det ∂(v, ω)/∂(v_s, φ) = −v_s / L', eDet < 1e-4, e(eDet));
  const ws = K.loadWheelSpeeds(1, 1 / TURN.R);
  check('load wheel speeds on the arc', abs(ws.left - (1 - 0.5 / TURN.R)) < 1e-12 && abs(ws.right - (1 + 0.5 / TURN.R)) < 1e-12, `inner ${f(ws.left)} / outer ${f(ws.right)} per 1 m/s`);
}

// ---------- 3. mast and fork kinematics ----------
console.log('\n3. Mast and fork kinematics');
{
  const z0 = K.tcpBody(0, 0.30, 0), zt = K.tcpBody(MAST.travel.tilt, MAST.travel.lift, 0), tip = K.tipBody(MAST.travel.tilt, MAST.travel.lift, 0);
  check('zero-pose TCP', abs(z0[0] - 0.87) < 1e-12 && abs(z0[2] - 0.30) < 1e-12, `(${f(z0[0])}, ${f(z0[1])}, ${f(z0[2])})`);
  console.log(`      travel pose: TCP (${f(zt[0])}, 0, ${f(zt[2])}), tips (${f(tip[0])}, 0, ${f(tip[2])})`);
  const r = rng(5);
  let eIk = 0, eJ = 0, eD = 0, eP = 0, eB = 0;
  for (let n = 0; n < 20000; n++) {
    const tilt = MAST.tilt.min + (MAST.tilt.max - MAST.tilt.min) * r(), lift = MAST.lift.min + 1.55 * r(), shift = (2 * r() - 1) * 0.1;
    const b = K.tcpBody(tilt, lift, shift);
    eIk = max(eIk, abs(K.liftFor(b[2], tilt) - lift), abs(b[1] - shift));
    const J = K.mastJacobian(tilt, lift), h = 1e-6, bt = K.tcpBody(tilt + h, lift, shift), bl = K.tcpBody(tilt, lift + h, shift);
    eJ = max(eJ, abs((bt[0] - b[0]) / h - J[0][0]), abs((bl[0] - b[0]) / h - J[0][1]), abs((bt[2] - b[2]) / h - J[1][0]), abs((bl[2] - b[2]) / h - J[1][1]));
    eD = max(eD, abs(K.det2(J) + (lift - MAST.pivot[1])));
    eP = max(eP, abs(K.det3(K.planarJacobian(shift)) + 1));
    // base pose for a TCP target and back
    const th = 6 * r(), tx = 10 * r(), ty = 10 * r(), base = K.baseFor(tx, ty, th, shift), w = K.tcpWorld(base, 0, lift, shift);
    eB = max(eB, hypot(w[0] - tx, w[1] - ty));
  }
  check('lift IK(FK) over 20 000 random mast poses', eIk < 1e-12, e(eIk));
  check('base pose for a TCP target: FK(IK) = target', eB < 1e-12, e(eB) + ' m');
  check('mast Jacobian vs finite differences', eJ < 1e-5, e(eJ));
  check('det J_mast = −(q4 − z_t)', eD < 1e-12, `singular at q4 = ${MAST.pivot[1]} (travel pose q4 = ${MAST.travel.lift})`);
  check('det J_planar(v, ω, q̇5) = −1', eP < 1e-12);
}

// ---------- 4. routes and stations ----------
console.log('\n4. Routes and stations');
const cell0 = new Cell();
{
  let eEnd = 0, eLink = 0, stemMin = Infinity, n = 0;
  for (const pk of cell0.lines)
    for (const sd of cell0.sides) {
      const { leg, link } = routePaths(pk, sd), pre = stationPoses(sd).pre, pp = stationPoses(pk).pre;
      eEnd = max(eEnd, hypot(leg.end.x - pre.x, leg.end.y - pre.y), abs(leg.end.th - pre.th));
      eLink = max(eLink, hypot(link.end.x - pp.x, link.end.y - pp.y));
      stemMin = min(stemMin, leg.length - (2 * TURN.Lc + (PI / 2) * TURN.R - TURN.Lc));
      n++;
    }
  check(`all ${n} L-routes end on the pre-dock pose`, eEnd < 1e-8 && eLink < 1e-9, `worst ${e(eEnd)} (leg), ${e(eLink)} (link)`);
  check('every leg has a straight run-in after the turn', stemMin > 0.5, `shortest ${f(stemMin)} m`);
  let eTcp = 0;
  for (const st of [...cell0.lines, ...cell0.slots, cell0.stack]) {
    const { dock } = stationPoses(st), w = K.tcpWorld(dock, 0, 0.3, 0);
    eTcp = max(eTcp, hypot(w[0] - st.x, w[1] - st.y));
  }
  check('docked TCP = pallet centre at every station', eTcp < 1e-12, e(eTcp) + ' m');
  for (const st of [cell0.lines[0], cell0.slots[0], cell0.stack]) {
    const { dock, pre } = stationPoses(st);
    console.log(`      ${st.id}: docked (${f(dock.x, 2)}, ${f(dock.y, 2)}), pre-dock (${f(pre.x, 2)}, ${f(pre.y, 2)})`);
  }
}

// ---------- 5. static clearances ----------
console.log('\n5. Static clearances and fork window');
{
  const xFace = CONV.xP - PALLET.sx / 2, xFrame = xFace - CONV.endOverhang, xDock = CONV.xP - K.REACH, xPre = xDock - K.D_IN;
  const c = {
    tineSideIn: FORK.tineY - FORK.tineW / 2 - PALLET.openY[0],
    tineSideOut: PALLET.openY[1] - FORK.tineY - FORK.tineW / 2,
    tineFloor: PALLET.pocketHi - FORK.cEntry - FORK.thick - PALLET.pocketLo,
    tineStop: FORK.tineY - FORK.tineW / 2 - 0.05, // end stops are 0.10 wide at y = 0, ±0.5275
    tipFar: PALLET.sx - (FORK.length - FORK.faceGap),
    tipNext: CONV.pitch - PALLET.sx + PALLET.sx - (FORK.length - FORK.faceGap),
    mastFrame: xFrame - (xDock + AMR.body.xf),
    chassisFrame: xFrame - (xDock + AMR.chassisXf),
    carriedFrame: xFrame - (xPre + K.REACH + PALLET.sx / 2),
    neighbours: CELL.slots[1].x - CELL.slots[0].x - PALLET.sy,
    mastPallet: PALLET.sx / 2 + K.REACH - AMR.body.xf - PALLET.sx,
  };
  const mm = (v) => (1000 * v).toFixed(0);
  check('tine ↔ fork-window side walls', min(c.tineSideIn, c.tineSideOut) >= 0.10, `${mm(c.tineSideIn)} / ${mm(c.tineSideOut)} mm`);
  check('tine ↔ pocket floor / ceiling at entry', c.tineFloor >= 0.02 && FORK.cEntry >= 0.02, `${mm(c.tineFloor)} / ${mm(FORK.cEntry)} mm`);
  check('tine ↔ conveyor end stops', c.tineStop >= 0.10, `${mm(c.tineStop)} mm`);
  check('side shift covers the pallet scatter', MAST.shift.max >= CONV.maxOffsetY, `±${mm(MAST.shift.max)} vs ±${mm(CONV.maxOffsetY)} mm`);
  check('fork tip stays inside the pallet', c.tipFar > 0 && c.tipNext >= 0.2, `${mm(c.tipFar)} mm short of the far face, ${mm(c.tipNext)} mm to the next pallet`);
  check('mast / chassis ↔ conveyor frame end, docked', c.mastFrame >= SAFE.clear, `${mm(c.mastFrame)} / ${mm(c.chassisFrame)} mm`);
  check('carried pallet ↔ conveyor frame end at the pre-dock point', c.carriedFrame >= 0.2, `${mm(c.carriedFrame)} mm`);
  check('carried pallet ↔ neighbouring floor pallets', c.neighbours / 2 >= 0.1, `${mm(c.neighbours)} mm each side`);
  check('mast ↔ deposited pallet, docked', c.mastPallet >= SAFE.clear, `${mm(c.mastPallet)} mm`);
  const top = K.liftTargets((CELL.stack.max - 1) * PALLET.h).carry;
  check('lift range covers every transfer', K.liftTargets(0).entry >= MAST.lift.min && top <= MAST.lift.max, `${f(K.liftTargets(0).entry)} … ${f(top)} of ${MAST.lift.min} … ${MAST.lift.max}`);
  const p = K.carriedPallet({ x: 0, y: 0, th: 0 }, MAST.travel.tilt, MAST.travel.lift, 0);
  console.log(`      carried pallet in the travel pose: underside ${f(p.z)} at the centre, ${f(p.z - (PALLET.sx / 2) * sin(p.pitch))} at the heel edge`);
}

// ---------- 6. stability and load holding ----------
console.log('\n6. Stability and load holding (rated load)');
{
  const zLoad = (lift) => lift - PALLET.pocketHi + PALLET.h + PALLET.cgMax;
  const rated = K.stability({ mass: AMR.payload, x: K.REACH, z: zLoad(MAST.travel.lift) });
  const high = K.stability({ mass: AMR.payload, x: K.REACH, z: zLoad(K.liftTargets(CONV.z).carry) });
  const un = K.stability(null);
  check('static tipping forward', rated.longSF >= 1.5, `SF ${f(rated.longSF, 2)}`);
  check('tipping forward at the emergency deceleration', rated.brakeTip / AMR.eStopDecel >= 1.5, `limit ${f(rated.brakeTip, 2)} m/s² → SF ${f(rated.brakeTip / AMR.eStopDecel, 2)}; at conveyor height ${f(high.brakeTip, 2)} m/s² → SF ${f(high.brakeTip / AMR.eStopDecel, 2)}`);
  check('tipping sideways at the lateral limit', min(rated.latTip, un.latTip) / AMR.aLat >= 2, `limit ${f(rated.latTip, 2)} loaded / ${f(un.latTip, 2)} unladen m/s² → SF ${f(min(rated.latTip, un.latTip) / AMR.aLat, 2)}`);
  const sl = K.slideDecel(MAST.travel.tilt), sl0 = K.slideDecel(0);
  check('pallet sliding off the tines', sl / AMR.eStopDecel >= 1.5 && sl0 / AMR.drive.creep.a >= 2, `tilt 4°: ${f(sl, 2)} m/s² → SF ${f(sl / AMR.eStopDecel, 2)} (e-stop), ${f(sl / AMR.drive.loaded.a, 1)} (service); tilt 0°: SF ${f(sl0 / AMR.drive.creep.a, 1)} at creep`);
  check('pallet sliding sideways in a turn', (FORK.mu * G) / AMR.aLat >= 2, `SF ${f((FORK.mu * G) / AMR.aLat, 2)}`);
  check('drive wheel keeps load with the rated payload', rated.driveWheelLoad > 0.15 * AMR.mass * G, `${f(rated.driveWheelLoad / G, 0)} kg of ${f(un.driveWheelLoad / G, 0)} kg unladen`);
}

// ---------- 7. safety monitor ----------
console.log('\n7. Safety monitor, negative tests');
{
  const ln = cell0.lines[0], off = -0.04;
  const pal = { x: CONV.xP, y: ln.y + off, yaw: 0, z: CONV.z, carried: false, station: 'P1' };
  pal.poly = K.palletPoly(pal);
  const env = (o = {}) => ({
    stations: [{ id: 'P1', kind: 'pick', x: ln.x, y: ln.y, th: 0, zs: CONV.z, pallet: 'pallet' in o ? o.pallet : pal, locked: o.locked ?? true }],
    frames: cell0.frames,
    pallets: o.pallets ?? [pal],
  });
  const free = { pose: { x: 0, y: 0, th: 0 }, v: 0, phi: 0, vs: 0, steerRate: 0, tilt: MAST.travel.tilt, lift: MAST.travel.lift, shift: 0, mastRate: [0, 0, 0], loaded: false };
  const lt = K.liftTargets(CONV.z), dock = stationPoses(ln).dock;
  const docked = { ...free, pose: dock, tilt: 0, lift: lt.entry, shift: off };
  const mine = { ...pal, carried: true, station: null, z: CONV.z + FORK.cLift };
  const cases = [
    ['legal: standing in the travel pose', free, env(), null],
    ['legal: 1.6 m/s empty, travel pose', { ...free, v: 1.6, vs: 1.6 }, env(), null],
    ['legal: docked, forks in the pockets, conveyor locked', docked, env(), null],
    ['legal: pallet just lifted off the chain', { ...docked, lift: lt.carry, loaded: true }, env({ pallet: null, pallets: [mine] }), null],
    ['steer beyond 90°', { ...free, phi: 95 * DEG }, env(), 'R1'],
    ['lift above the stroke', { ...free, lift: 1.7 }, env(), 'R1'],
    ['1.5 m/s with a load', { ...free, v: 1.5, vs: 1.5, loaded: true }, env(), 'R2'],
    ['1.2 m/s in reverse', { ...free, v: -1.2, vs: -1.2 }, env(), 'R2'],
    ['steer rate 90°/s', { ...free, v: 0.5, vs: 0.5, steerRate: 90 * DEG }, env(), 'R2'],
    ['1.5 m/s on the 1.5 m radius', { ...free, v: 1.5, vs: 1.98, phi: K.steerOf(1 / 1.5) }, env(), 'R3'],
    ['lifting while driving', { ...free, v: 0.2, vs: 0.2, mastRate: [0, 0.1, 0] }, env(), 'R4'],
    ['1.0 m/s with the fork at 0.70', { ...free, v: 1.0, vs: 1.0, lift: 0.70 }, env(), 'R5'],
    ['1.0 m/s with the mast upright', { ...free, v: 1.0, vs: 1.0, tilt: 0 }, env(), 'R5'],
    ['0.5 m/s inside the station', { ...docked, v: 0.5, vs: 0.5 }, env(), 'R6'],
    ['steering inside the station', { ...docked, v: 0.1, vs: 0.1, phi: 3 * DEG }, env(), 'R6'],
    ['heading 1° off inside the station', { ...docked, pose: { ...dock, th: 1 * DEG } }, env(), 'R6'],
    ['mast tilted inside the station', { ...docked, tilt: 2 * DEG }, env(), 'R6'],
    ['tine 50 mm too low', { ...docked, lift: lt.entry - 0.05 }, env(), 'R7'],
    ['tine above the pocket, no load signal', { ...docked, lift: lt.entry + 0.05 }, env(), 'R7'],
    ['side shift 140 mm off the pallet', { ...docked, shift: 0.10 }, env(), 'R7'],
    ['carried pallet 150 mm above the chain', { ...docked, lift: lt.carry + 0.09, loaded: true }, env({ pallet: null, pallets: [{ ...mine, z: CONV.z + 0.15 }] }), 'R7'],
    ['conveyor not locked', docked, env({ locked: false }), 'R8'],
    ['carrying into an occupied station', { ...docked, lift: lt.carry, loaded: true }, env({ pallets: [pal, mine] }), 'R9'],
    ['chassis 0.2 m too far into the conveyor', { ...docked, pose: { ...dock, x: dock.x + 0.2 } }, env(), 'R10'],
  ];
  let okAll = true;
  for (const [name, s, en, want] of cases) {
    const got = K.monitor(s, en).bad;
    const ok = want === null ? got.length === 0 : got.some((g) => g.startsWith(want + ' '));
    if (!ok) { okAll = false; console.log(`        ✗ ${name}: expected ${want ?? 'no violation'}, got [${got.join('; ')}]`); }
  }
  check(`${cases.length} states classified correctly (${cases.filter((c) => c[3]).length} illegal, ${cases.filter((c) => !c[3]).length} legal)`, okAll);
}

// ---------- 8. production run ----------
const HOURS = 4, DT = 1 / 240;
console.log(`\n8. Production run: ${HOURS} h at 240 Hz, seed 7`);
{
  const cell = new Cell({ seed: 7 }), a = cell.amr;
  const pk = { v: 0, vRev: 0, vLoaded: 0, acc: 0, jerk: 0, vs: 0, phi: 0, steerRate: 0, steerAcc: 0, aLat: 0, tilt: 0, lift: 0, shift: 0, vCreepZone: 0 };
  const rg = { lift: [9, -9], tilt: [9, -9], shift: [9, -9] };
  let odo = 0, raw = { mastWhileMoving: 0, zoneUnlocked: 0, zoneNotReady: 0, lowCarry: 0 }, dropErr = 0, latErr = 0, queueMax = 0;
  let prev = { v: 0, acc: 0, phi: 0, steerRate: 0 }, wasEngaged = null, seen = new Set(), placed = new Map();
  const rules = new Map(), steps = Math.round((HOURS * 3600) / DT);
  const refs = {};
  for (let i = 0; i < steps; i++) {
    cell.step(DT);
    const { v, acc, phi, vs, steerRate, steerAcc, tilt, lift, shift, mastRate } = a;
    pk.v = max(pk.v, v); pk.vRev = max(pk.vRev, -v); if (a.loaded) pk.vLoaded = max(pk.vLoaded, abs(v));
    pk.acc = max(pk.acc, abs(acc)); pk.vs = max(pk.vs, abs(vs)); pk.phi = max(pk.phi, abs(phi));
    pk.steerRate = max(pk.steerRate, abs(steerRate)); pk.steerAcc = max(pk.steerAcc, abs(steerAcc));
    pk.aLat = max(pk.aLat, v * v * abs(a.k));
    pk.jerk = max(pk.jerk, abs(acc - prev.acc) / DT);
    pk.tilt = max(pk.tilt, abs(mastRate[0])); pk.lift = max(pk.lift, abs(mastRate[1])); pk.shift = max(pk.shift, abs(mastRate[2]));
    for (const [n, val] of [['lift', lift], ['tilt', tilt], ['shift', shift]]) { rg[n][0] = min(rg[n][0], val); rg[n][1] = max(rg[n][1], val); }
    prev = { v, acc, phi, steerRate };
    odo = max(odo, hypot(a.odo.x - a.pose.x, a.odo.y - a.pose.y));
    for (const b of a.last.bad) rules.set(b, (rules.get(b) || 0) + 1);

    // raw checks, independent of the monitor
    if (abs(v) > 1e-6 && mastRate.some((r) => abs(r) > 1e-9)) raw.mastWhileMoving++;
    const tip = K.toWorld(a.pose, K.tipBody(tilt, lift, shift));
    for (const l of cell.lines) {
      if (abs(a.pose.th) < 0.1 && abs(tip[1] - l.y) < 0.5 && tip[0] > CONV.xP - PALLET.sx / 2 - DOCK.zoneIn) {
        pk.vCreepZone = max(pk.vCreepZone, abs(v));
        if (!l.locked) raw.zoneUnlocked++;
        if (!a.engaged && !l.ready) raw.zoneNotReady++;
      }
      queueMax = max(queueMax, l.pallets.length);
    }
    if (a.carried && abs(v) > AMR.drive.creep.v + 1e-9 && a.carried.z < 0.15) raw.lowCarry++;
    // tines centred in the pallet at the moment of engagement; pallet centred in its slot on release
    if (a.engaged && !wasEngaged) latErr = max(latErr, abs(K.tcpWorld(a.pose, tilt, lift, shift)[1] - a.engaged.pallet.y));
    if (!a.engaged && wasEngaged) {
      const st = wasEngaged.station, p = wasEngaged.pallet;
      dropErr = max(dropErr, hypot(p.x - st.x, p.y - st.y), abs(p.z - wasEngaged.zs), abs(p.pitch));
      placed.set(p.id, [st.kind, p.kind]);
    }
    wasEngaged = a.engaged;
    for (const p of cell.pallets) seen.add(p.id);
    if (a.phase.endsWith('lift pallet') && !refs.pick) refs.pick = [a.pose.x, a.pose.y, a.pose.th, a.tilt, a.lift, a.shift];
  }
  const st = a.stats, mean = st.cycle.reduce((x, y) => x + y, 0) / st.cycle.length;
  check('safety monitor violations', st.violations === 0, rules.size ? [...rules].map(([k, n]) => `${k} ×${n}`).join('; ') : '0');
  check('mast axes move only at standstill (raw check)', raw.mastWhileMoving === 0, `${raw.mastWhileMoving} steps`);
  check('fork in a pick zone only with the conveyor locked and a pallet present (raw check)', raw.zoneUnlocked === 0 && raw.zoneNotReady === 0, `${raw.zoneUnlocked} / ${raw.zoneNotReady} steps`);
  check('path speed limits', pk.v <= AMR.drive.empty.v + 1e-9 && pk.vRev <= AMR.vRev + 1e-9 && pk.vLoaded <= AMR.drive.loaded.v + 1e-9 && pk.vCreepZone <= AMR.drive.creep.v + 1e-9, `forward ${f(pk.v)}, reverse ${f(pk.vRev)}, loaded ${f(pk.vLoaded)}, in a fork zone ${f(pk.vCreepZone)} m/s`);
  check('path acceleration / jerk limits', pk.acc <= AMR.drive.empty.a + 1e-9 && pk.jerk <= AMR.drive.empty.j * 1.01, `${f(pk.acc)} m/s², ${f(pk.jerk, 2)} m/s³`);
  check('drive wheel speed', pk.vs <= AMR.wheelV, `${f(pk.vs)} of ${AMR.wheelV} m/s`);
  check('steer angle / rate / acceleration', pk.phi <= AMR.steer.max && pk.steerRate <= AMR.steer.v + 1e-9 && pk.steerAcc <= AMR.steer.a, `${f(pk.phi / DEG, 2)}°, ${f(pk.steerRate / DEG, 1)} of ${f(AMR.steer.v / DEG, 0)}°/s, ${f(pk.steerAcc / DEG, 0)} of ${f(AMR.steer.a / DEG, 0)}°/s²`);
  check('lateral acceleration', pk.aLat <= AMR.aLat, `${f(pk.aLat)} of ${AMR.aLat} m/s²`);
  check('mast axis speeds', pk.tilt <= MAST.tilt.v + 1e-9 && pk.lift <= MAST.lift.v + 1e-9 && pk.shift <= MAST.shift.v + 1e-9, `tilt ${f(pk.tilt / DEG, 2)}°/s, lift ${f(pk.lift)} m/s, shift ${f(pk.shift)} m/s`);
  check('joint ranges used', rg.lift[0] >= MAST.lift.min && rg.lift[1] <= MAST.lift.max && rg.shift[1] <= MAST.shift.max, `lift ${f(rg.lift[0])} … ${f(rg.lift[1])}, tilt ${f(rg.tilt[0] / DEG, 1)} … ${f(rg.tilt[1] / DEG, 1)}°, shift ${f(rg.shift[0])} … ${f(rg.shift[1])}`);
  check('dead reckoning from the wheel commands vs the path pose', odo < 0.005, `worst ${e(odo)} m after ${f(st.dist / 1000, 2)} km`);
  check('tines centred on the pallet at engagement (side shift)', latErr < 1e-9, e(latErr) + ' m');
  check('pallet set down on the station centre, level, on the support plane', dropErr < 1e-6, e(dropErr));
  check('plan clearance never below the limit', st.minClear >= SAFE.clear - 1e-9, `min ${(1000 * st.minClear).toFixed(0)} mm`);
  const inCell = cell.pallets.size, gone = seen.size - inCell;
  check('pallets conserved', seen.size === cell.nPallets && st.delivered === placed.size, `${cell.nPallets} created = ${inCell} in the cell + ${gone} collected`);
  check('filled pallets only in floor slots, empty pallets only on the stack', [...placed.values()].every(([sk, pk2]) => (sk === 'slot') === (pk2 === 'filled') && (sk === 'stack') === (pk2 === 'empty')) && st.filled + st.empty === st.delivered, `${st.filled} filled, ${st.empty} empty`);
  check('conveyors never overflow', queueMax <= CONV.cap, `longest queue ${queueMax} of ${CONV.cap}`);
  console.log(`      ${st.delivered} pallets delivered = ${f(st.delivered / HOURS, 1)} / h; cycle ${f(mean, 1)} s mean (${f(min(...st.cycle), 1)} … ${f(max(...st.cycle), 1)}); distance ${f(st.dist / 1000, 2)} km`);
  console.log(`      capacity if never waiting: ${f(3600 / mean, 0)} pallets / h`);
  if (refs.pick) console.log(`      reference, docked at a pick station: x ${f(refs.pick[0])}, y ${f(refs.pick[1])}, θ ${f(refs.pick[2] / DEG, 1)}°, tilt ${f(refs.pick[3] / DEG, 1)}°, lift ${f(refs.pick[4])}, shift ${f(refs.pick[5])}`);
}

console.log(fails ? `\n${fails} CHECK(S) FAILED` : '\nall checks passed');
process.exit(fails ? 1 : 0);
