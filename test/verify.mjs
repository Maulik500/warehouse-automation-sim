// Headless verification of the kinematic model and the whole palletizing run.
// Run: npm run verify
import { ROBOT as RB, TOOL, BOX, CELL, R2D } from '../js/config.js';
import { fk, ik, singularity, L3, BETA, mul, tr } from '../js/kinematics.js';
import { Sim, slot } from '../js/sim.js';

let fail = 0;
const check = (ok, msg) => { console.log(`${ok ? 'PASS' : 'FAIL'}  ${msg}`); if (!ok) fail++; };
const rnd = (lo, hi) => lo + Math.random() * (hi - lo);
const randQ = () => RB.qMin.map((lo, i) => rnd(lo, RB.qMax[i]));

// 1. Zero pose matches the spec table
{
  const f = fk([0, 0, 0, 0, 0, 0]);
  const exp = [RB.a1 + RB.d4 + RB.d6 + TOOL.tcp, 0, RB.d1 + RB.a2 + RB.a3];
  check(f.tcp.every((v, i) => Math.abs(v - exp[i]) < 1e-12), `zero-pose TCP = [${f.tcp.map((v) => v.toFixed(3))}]`);
  check(Math.abs(f.R[2] - 1) < 1e-12 && Math.abs(f.R[6] + 1) < 1e-12, 'zero-pose flange Z = world +X, flange X = world -Z');
}

// 2. IK(FK(q)) reproduces the pose for every posture branch
{
  let worst = 0, n = 0;
  for (let k = 0; k < 20000; k++) {
    const q = randQ(), f = fk(q), sg = singularity(q);
    if (Math.abs(sg.wrist) < 0.02 || Math.abs(sg.elbow) < 0.02 || Math.abs(sg.shoulder) < 0.02) continue;
    const cfg = { front: sg.shoulder > 0, up: Math.sin(q[2] + BETA) > 0, flip: q[4] < 0 };
    const qi = ik(f.tcp, f.R, q, cfg);
    if (!qi) { worst = Infinity; break; }
    const g = fk(qi);
    const e = Math.hypot(...g.tcp.map((v, i) => v - f.tcp[i])) + Math.max(...g.R.map((v, i) => Math.abs(v - f.R[i])));
    const ej = Math.max(...qi.map((v, i) => Math.abs(v - q[i])));
    worst = Math.max(worst, e, ej);
    n++;
  }
  check(worst < 1e-8, `IK/FK round trip over ${n} random poses, worst error ${worst.toExponential(2)}`);
}

// 3. det(J) factorises into the three singularity terms
{
  const det = (m) => { // Gaussian elimination, 6x6
    m = m.map((r) => [...r]); let d = 1;
    for (let i = 0; i < 6; i++) {
      let p = i; for (let r = i + 1; r < 6; r++) if (Math.abs(m[r][i]) > Math.abs(m[p][i])) p = r;
      if (p !== i) { [m[i], m[p]] = [m[p], m[i]]; d = -d; }
      d *= m[i][i]; if (m[i][i] === 0) return 0;
      for (let r = i + 1; r < 6; r++) { const f = m[r][i] / m[i][i]; for (let c = i; c < 6; c++) m[r][c] -= f * m[i][c]; }
    }
    return d;
  };
  let worst = 0;
  for (let k = 0; k < 2000; k++) {
    const q = randQ(), h = 1e-6, f0 = fk(q), J = [...Array(6)].map(() => Array(6));
    for (let j = 0; j < 6; j++) {
      const qp = [...q]; qp[j] += h; const f1 = fk(qp);
      const dR = mul(f1.R, tr(f0.R)); // ~ I + [w]x h
      const col = [...f1.pw.map((v, i) => (v - f0.pw[i]) / h), (dR[7] - dR[5]) / (2 * h), (dR[2] - dR[6]) / (2 * h), (dR[3] - dR[1]) / (2 * h)];
      for (let i = 0; i < 6; i++) J[i][j] = col[i];
    }
    const sg = singularity(q);
    worst = Math.max(worst, Math.abs(Math.abs(det(J)) - Math.abs(RB.a2 * L3 * sg.elbow * sg.shoulder * sg.wrist)));
  }
  check(worst < 1e-4, `|det J| = a2*L3*|sin(q3+beta)|*|rho|*|sin q5|, worst deviation ${worst.toExponential(2)}`);
}

// 4. Full production run: two pallets
{
  const sim = new Sim(), dt = 1 / 240;
  const st = { vr: Array(6).fill(0), ar: Array(6).fill(0), qlo: Array(6).fill(9), qhi: Array(6).fill(-9), wrist: 9, elbow: 9, shoulder: 9, clr: 9, lin: 0, ah: 0, az: 0 };
  const half = (b) => { // world AABB half extents of a box (yaw is a multiple of 90 deg at rest)
    const ax = Math.abs(b.R[0]), ay = Math.abs(b.R[1]);
    return [(BOX.sx * ax + BOX.sy * ay) / 2, (BOX.sx * Math.abs(b.R[3]) + BOX.sy * Math.abs(b.R[4])) / 2, BOX.sz / 2];
  };
  const gap = (a, b) => { // signed AABB separation
    const ha = half(a), hb = half(b);
    return Math.max(...[0, 1, 2].map((i) => Math.abs(a.p[i] - b.p[i]) - ha[i] - hb[i]));
  };
  const ptBox = (p, b, rad) => { const h = half(b); return Math.hypot(...[0, 1, 2].map((i) => Math.max(0, Math.abs(p[i] - b.p[i]) - h[i]))) - rad; };
  let err = null, prevQd = [...sim.qd], prevTcp = fk(sim.q).tcp, prevVel = [0, 0, 0], tEnd = 0;
  try {
    while (sim.pallets < 2 && sim.t < 1200) {
      sim.step(dt);
      const f = fk(sim.q), sg = singularity(sim.q);
      for (let i = 0; i < 6; i++) {
        st.vr[i] = Math.max(st.vr[i], Math.abs(sim.qd[i]) / RB.vMax[i]);
        st.ar[i] = Math.max(st.ar[i], Math.abs(sim.qd[i] - prevQd[i]) / dt / RB.aMax[i]);
        st.qlo[i] = Math.min(st.qlo[i], sim.q[i]); st.qhi[i] = Math.max(st.qhi[i], sim.q[i]);
      }
      prevQd = [...sim.qd];
      const vel = f.tcp.map((v, i) => (v - prevTcp[i]) / dt); prevTcp = f.tcp;
      st.lin = Math.max(st.lin, Math.hypot(...vel));
      if (sim.held) {
        st.ah = Math.max(st.ah, Math.hypot(vel[0] - prevVel[0], vel[1] - prevVel[1]) / dt);
        st.az = Math.max(st.az, (vel[2] - prevVel[2]) / dt);
      }
      prevVel = vel;
      st.wrist = Math.min(st.wrist, Math.abs(sg.wrist)); st.elbow = Math.min(st.elbow, Math.abs(sg.elbow)); st.shoulder = Math.min(st.shoulder, Math.abs(sg.shoulder));
      // clearance: carried box and arm vs boxes on the pallet and the conveyor body
      const placed = sim.boxes.filter((b) => b.state === 'placed');
      const convBody = { p: [CELL.conv.x, (CELL.conv.yStart + CELL.conv.yEnd) / 2, CELL.conv.top / 2], R: [1, 0, 0, 0, 1, 0, 0, 0, 1] };
      const convHalf = [CELL.conv.width / 2 + 0.05, (CELL.conv.yEnd - CELL.conv.yStart) / 2, CELL.conv.top / 2];
      const moving = sim.state.startsWith('PTP');
      if (sim.held && moving) for (const b of placed) st.clr = Math.min(st.clr, gap(sim.held, b));
      const pts = [];
      for (let k = 0; k <= 8; k++) pts.push(f.j3.map((v, i) => v + ((f.pw[i] - v) * k) / 8));
      pts.push(f.flange);
      for (const p of pts) {
        for (const b of placed) st.clr = Math.min(st.clr, ptBox(p, b, 0.17));
        st.clr = Math.min(st.clr, Math.hypot(...[0, 1, 2].map((i) => Math.max(0, Math.abs(p[i] - convBody.p[i]) - convHalf[i]))) - 0.17);
      }
      tEnd = sim.t;
    }
  } catch (e) { err = e; }
  check(!err, `run completed without IK failure${err ? ' -> ' + err.message : ''}`);
  check(sim.pallets === 2, `2 pallets (${2 * CELL.pallet.nx * CELL.pallet.ny * CELL.pallet.layers} boxes) in ${tEnd.toFixed(0)} s`);
  check(sim.missed === 0, `missed boxes: ${sim.missed}`);
  check(st.vr.every((v) => v <= 1.001), `joint speed / limit   : ${st.vr.map((v) => v.toFixed(2)).join('  ')}`);
  check(st.ar.every((v) => v <= 1.05), `joint accel / limit   : ${st.ar.map((v) => v.toFixed(2)).join('  ')}`);
  // vacuum holding force vs worst-case load on the carried box (spec §8)
  const hold = TOOL.cups * TOOL.vacuum * Math.PI * (TOOL.cupDia / 2) ** 2;
  const need = BOX.mass * (9.81 + st.az + st.ah / 0.5);
  console.log(`      peak TCP speed ${st.lin.toFixed(2)} m/s, peak carried accel: horizontal ${st.ah.toFixed(2)}, vertical ${st.az.toFixed(2)} m/s^2`);
  check(hold / need >= 2, `vacuum holding force ${hold.toFixed(0)} N vs required ${need.toFixed(0)} N -> safety factor ${(hold / need).toFixed(2)}`);
  console.log(`      joint range used [deg]: ${st.qlo.map((v, i) => `J${i + 1} ${(v * R2D).toFixed(0)}..${(st.qhi[i] * R2D).toFixed(0)}`).join('  ')}`);
  check(st.wrist > 0.25, `wrist singularity margin  min|sin q5| = ${st.wrist.toFixed(3)} (q5 >= ${(Math.asin(st.wrist) * R2D).toFixed(1)} deg from 0)`);
  check(st.elbow > 0.25, `elbow singularity margin  min|sin(q3+beta)| = ${st.elbow.toFixed(3)}`);
  check(st.shoulder > 0.3, `shoulder singularity margin  min rho = ${st.shoulder.toFixed(3)} m`);
  check(st.clr > 0.02, `min clearance (carried box / forearm vs stack and conveyor) = ${(st.clr * 1000).toFixed(0)} mm`);
  const ct = sim.cycleTimes;
  console.log(`      cycle time: mean ${(ct.reduce((a, b) => a + b, 0) / ct.length).toFixed(2)} s, min ${Math.min(...ct).toFixed(2)}, max ${Math.max(...ct).toFixed(2)}`);
  const s0 = slot(0), sN = slot(31);
  console.log(`      slot 0 top-centre [${s0.map((v) => v.toFixed(3))}], slot 31 [${sN.map((v) => v.toFixed(3))}]`);
}

console.log(fail ? `\n${fail} CHECK(S) FAILED` : '\nALL CHECKS PASSED');
process.exit(fail ? 1 : 0);
