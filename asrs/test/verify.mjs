// Headless verification of the ASRS model: motion law, kinematics, workspace,
// interlocks and a long production run. `npm run verify:asrs`

import { CRANE, FORK, LOAD, RACK, PD, SAFE, LAYOUT, SKUS, G } from '../js/config.js';
import { scurve } from '../js/motion.js';
import { fk, ik, jacobian, det3, deflection, hoistTargets, rackSlot, STATION, MARGIN, monitor, levelZ, bayX } from '../js/kinematics.js';
import { Warehouse, boxWorld } from '../js/sim.js';

const AX = CRANE.axes;
const { abs, max, min } = Math;
let fails = 0;
const check = (name, ok, detail = '') => {
  if (!ok) fails++;
  console.log(`${ok ? '  ok ' : ' FAIL'}  ${name}${detail ? '  — ' + detail : ''}`);
};
const f = (x, n = 3) => x.toFixed(n);

// ---------- 1. motion law ----------
console.log('\n1. S-curve motion law');
{
  let seed = 1;
  const r = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  let eEnd = 0, vOver = 0, aOver = 0, eDer = 0;
  for (let n = 0; n < 1000; n++) {
    const D = 10 ** (-3 + 4.4 * r()), v = 0.05 + 3 * r(), a = 0.1 + 2 * r(), j = 0.3 + 6 * r();
    const p = scurve(D, v, a, j);
    eEnd = max(eEnd, abs(p.endError));
    const N = Math.ceil(max(400, p.T / 2e-3)), h = p.T / N; // step well below the jerk time
    let prev = p.at(0), integ = 0;
    for (let i = 1; i <= N; i++) {
      const s = p.at(i * h);
      vOver = max(vOver, abs(s.v) - v);
      aOver = max(aOver, abs(s.a) - a);
      // running trapezoid integral of v must reproduce p
      integ += ((s.v + prev.v) * h) / 2;
      eDer = max(eDer, abs(integ - s.p) / D);
      prev = s;
    }
  }
  check('end position exact', eEnd < 1e-9, `worst ${eEnd.toExponential(1)} m over 1000 random profiles`);
  check('speed limit never exceeded', vOver < 1e-9, `${vOver.toExponential(1)}`);
  check('acceleration limit never exceeded', aOver < 1e-9, `${aOver.toExponential(1)}`);
  check('position is the integral of velocity', eDer < 1e-4, `worst relative error ${eDer.toExponential(1)}`);
  const full = AX.map((x) => scurve(x.max - x.min, x.v, x.a, x.j));
  console.log(`      full-stroke times: travel ${f(full[0].T, 2)} s, hoist ${f(full[1].T, 2)} s, fork(empty, 1.4 m) ${f(scurve(1.4, AX[2].v, AX[2].a, AX[2].j).T, 2)} s, fork(loaded) ${f(scurve(1.4, CRANE.forkLoaded.v, CRANE.forkLoaded.a, CRANE.forkLoaded.j).T, 2)} s, lift ${f(scurve(FORK.cLow + FORK.cHigh + FORK.deflRated, CRANE.hoistCreep.v, CRANE.hoistCreep.a, CRANE.hoistCreep.j).T, 2)} s`);
}

// ---------- 2. kinematics ----------
console.log('\n2. Kinematics');
{
  let seed = 3;
  const r = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
  let e = 0, dMin = Infinity, dMax = -Infinity, jErr = 0;
  for (let n = 0; n < 20000; n++) {
    const q = AX.map((x) => x.min + (x.max - x.min) * r());
    const m = 1000 * r();
    const back = ik(fk(q, m), m);
    e = max(e, ...q.map((v, i) => abs(v - back[i])));
    const J = jacobian(q, m), d = det3(J);
    dMin = min(dMin, d); dMax = max(dMax, d);
    const h = 1e-6;
    for (let c = 0; c < 3; c++) {
      const qp = [...q], qm = [...q];
      qp[c] += h; qm[c] -= h;
      const a = fk(qp, m), b = fk(qm, m);
      for (let rr = 0; rr < 3; rr++) jErr = max(jErr, abs((a[rr] - b[rr]) / (2 * h) - J[rr][c]));
    }
  }
  check('IK(FK(q)) = q', e < 1e-12, `worst ${e.toExponential(1)} over 20 000 random poses`);
  check('analytic Jacobian = finite differences', jErr < 1e-6, `${jErr.toExponential(1)}`);
  check('det J = -1 everywhere (no singularity)', abs(dMin + 1) < 1e-12 && abs(dMax + 1) < 1e-12, `range ${dMin} … ${dMax}`);
}

// ---------- 3. workspace and static clearances ----------
console.log('\n3. Workspace and clearances');
{
  let ok = true, n = 0, zLo = Infinity, zHi = -Infinity;
  const locs = [STATION.in, STATION.out];
  for (const s of [1, -1]) for (let b = 1; b <= RACK.bays; b++) for (let l = 1; l <= RACK.levels; l++) locs.push(rackSlot('X', s, b, l));
  for (const L of locs) {
    const h = hoistTargets(L.zs, CRANE.payload);
    zLo = min(zLo, h.low); zHi = max(zHi, h.high);
    for (const z of [h.low, h.high]) for (const y of [0, L.side * FORK.stroke]) { n++; if (!ik([L.x, y, z - deflection(y, 0)], 0)) ok = false; }
  }
  check(`all ${locs.length} handling positions of one aisle inside the joint limits`, ok, `${n} poses; hoist used ${f(zLo)} … ${f(zHi)} of ${AX[1].min} … ${AX[1].max}`);
  check('travel range covers stations and last bay', PD.x >= AX[0].min && bayX(RACK.bays) <= AX[0].max, `x ${PD.x} … ${bayX(RACK.bays)}`);
  const tolX = AX[0].tol;
  check('fork ↔ support ledge (lateral)', MARGIN.forkToLedge - tolX >= SAFE.minLateral, `${f(MARGIN.forkToLedge * 1000, 0)} mm`);
  check('pallet ↔ upright (lateral)', MARGIN.palletToUpright - tolX >= SAFE.minLateral, `${f(MARGIN.palletToUpright * 1000, 0)} mm`);
  check('pallet bearing on each ledge', MARGIN.palletBearing - tolX >= 0.05, `${f(MARGIN.palletBearing * 1000, 0)} mm`);
  check('carried load ↔ rack face while travelling', MARGIN.loadToRackFace >= 0.1, `${f(MARGIN.loadToRackFace * 1000, 0)} mm`);
  check('retracted fork ↔ rack face', MARGIN.forkToRackFace >= 0.1, `${f(MARGIN.forkToRackFace * 1000, 0)} mm`);
  const overlap = FORK.length - FORK.stroke * FORK.midRatio;
  check('telescope stage overlap at full stroke', overlap / FORK.length >= 0.4, `${f(overlap * 1000, 0)} mm = ${f((100 * overlap) / FORK.length, 0)} %`);
  const tipEnd = FORK.stroke + FORK.length / 2, palletEnd = RACK.yStore + LOAD.sy / 2;
  check('extended fork supports the full pallet depth', tipEnd >= palletEnd && FORK.stroke - FORK.length / 2 <= RACK.yStore - LOAD.sy / 2, `fork ${f(FORK.stroke - FORK.length / 2)} … ${f(tipEnd)}, pallet ${f(RACK.yStore - LOAD.sy / 2)} … ${f(palletEnd)}`);
  check('extended fork stays inside the rack depth', tipEnd <= RACK.yFace + RACK.depth, `${f(tipEnd)} ≤ ${f(RACK.yFace + RACK.depth)}`);
  // vertical budget inside a cell, rated load
  const top = FORK.cHigh + deflection(FORK.stroke, CRANE.payload) + LOAD.maxH; // highest load top above zs (fork just entering)
  const clrUp = RACK.pitchZ - RACK.ledgeLegH - top;
  const clrDown = RACK.pitchZ - LOAD.maxH - (FORK.cLow + FORK.thick + FORK.deflEmpty);
  check('max load ↔ level above during transfer', clrUp >= SAFE.minLoadClearTop, `${f(clrUp * 1000, 0)} mm`);
  check('fork underside ↔ load in the level below', clrDown >= 0.1, `${f(clrDown * 1000, 0)} mm`);
  const aSlip = FORK.mu * G, aTip = (G * (FORK.tineX + FORK.tineW / 2)) / LOAD.cgMax;
  const aWorst = max(AX[0].a, CRANE.forkLoaded.a);
  check('load does not slide on the fork (normal operation)', aSlip / aWorst >= 2, `SF ${f(aSlip / aWorst, 1)}`);
  check('load does not slide on the fork (emergency stop)', aSlip / CRANE.eStopDecel >= 1.3, `SF ${f(aSlip / CRANE.eStopDecel, 2)}`);
  check('load does not tip on the fork (emergency stop)', aTip / CRANE.eStopDecel >= 1.5, `SF ${f(aTip / CRANE.eStopDecel, 2)}`);
}

// ---------- 4. safety monitor catches bad states ----------
console.log('\n4. Safety monitor (negative tests)');
{
  const s = rackSlot('A', 1, 5, 3), h = hoistTargets(s.zs, 0);
  const load = { mass: 1000, h: 1.2 };
  const hs = hoistTargets(PD.z, 0);
  const oneSided = { rackSides: [1], deck: { 1: 0, '-1': 0 } };
  const deckUp = { rackSides: [1, -1], deck: { 1: 0.4, '-1': 0 } };
  const t = (name, q, qd, c, expect, ctx) => {
    const bad = monitor(q, qd, c, ctx);
    check(name, expect ? bad.some((b) => b.startsWith(expect)) : bad.length === 0, bad.join('; ') || 'no violation');
  };
  t('legal: fork fully extended in a cell', [s.x, h.low, 1.4], [0, 0, 0], null, null);
  t('legal: loaded fork at high position', [s.x, hoistTargets(s.zs, 1000).high, 1.4], [0, 0, 0], load, null);
  t('legal: fork in a station with the deck down', [PD.x, hs.low, 1.4], [0, 0, 0], null, null);
  t('travel with fork out', [s.x, h.low, 0.5], [0.2, 0, 0], null, 'R2');
  t('fast hoist with fork out', [s.x, h.low, 0.5], [0, 0.5, 0], null, 'R3');
  t('fork out between two bays', [s.x + 0.4, h.low, 0.6], [0, 0, 0], null, 'R4');
  t('fork out 20 mm off the bay centre', [s.x + 0.02, h.low, 0.6], [0, 0, 0], null, 'R4');
  t('fork out between two levels', [s.x, s.zs + 0.7, 0.6], [0, 0, 0], null, 'R5');
  t('fork into the support ledge height', [s.x, s.zs - 0.2, 0.6], [0, 0, 0], null, 'R5');
  t('fork lifted too far inside the cell', [s.x, s.zs + 0.15, 1.4], [0, 0, 0], null, 'R5');
  t('travel beyond the end of the aisle', [AX[0].max + 0.05, 2, 0], [0, 0, 0], null, 'R1');
  t('fork extended at the station column, wrong height', [PD.x, 3.0, -0.8], [0, 0, 0], null, 'R5');
  t('fork towards the side of aisle 3 that has no rack', [s.x, h.low, -0.6], [0, 0, 0], null, 'R4', oneSided);
  t('fork into a station whose roller deck is still up', [PD.x, hs.low, 0.6], [0, 0, 0], null, 'R7', deckUp);
}

// ---------- 5. unit loads ----------
console.log('\n5. Unit loads (boxes on pallets)');
{
  let ok = true, worstH = 0, worstM = 0;
  for (const S of SKUS) {
    const fx = S.nx * S.sx, fy = S.ny * S.sy, hh = LOAD.palletH + S.maxLayers * S.sz, m = LOAD.palletMass + S.maxLayers * S.nx * S.ny * S.mass;
    worstH = max(worstH, hh); worstM = max(worstM, m);
    if (fx > LOAD.sx + 1e-9 || fy > LOAD.sy + 1e-9 || hh > LOAD.maxH + 1e-9 || m > LOAD.maxMass) ok = false;
  }
  check('every SKU pattern fits the pallet footprint, height and mass limits', ok, `tallest ${f(worstH)} m, heaviest ${worstM} kg`);
  const wh = new Warehouse({ seed: 3 });
  let overlap = 0, outside = 0, n = 0;
  for (const p of wh.pallets) {
    for (let i = 0; i < p.boxes.length; i++) {
      const a = p.boxes[i]; n++;
      if (abs(a.lx) + a.sx / 2 > LOAD.sx / 2 + 1e-9 || abs(a.ly) + a.sy / 2 > LOAD.sy / 2 + 1e-9 || a.lz + a.sz / 2 > p.h - LOAD.palletH + LOAD.palletH + 1e-9 || a.lz - a.sz / 2 < LOAD.palletH - 1e-9) outside++;
      for (let j = i + 1; j < p.boxes.length; j++) {
        const b = p.boxes[j];
        if (abs(a.lx - b.lx) < (a.sx + b.sx) / 2 - 1e-9 && abs(a.ly - b.ly) < (a.sy + b.sy) / 2 - 1e-9 && abs(a.lz - b.lz) < (a.sz + b.sz) / 2 - 1e-9) overlap++;
      }
    }
    const m = LOAD.palletMass + p.boxes.reduce((s, b) => s + b.mass, 0);
    if (m !== p.mass) outside++;
  }
  check('boxes never overlap and stay inside the load envelope; pallet mass = sum of boxes', overlap === 0 && outside === 0, `${n} boxes on ${wh.pallets.size} pallets`);
}

// ---------- 6. production run, all aisles ----------
console.log('\n6. Production run');
const HOURS = 4, DT = 1 / 240;
{
  const wh = new Warehouse({ seed: 7 });
  const lim = AX.map((x) => ({ v: x.v, a: x.a, j: x.j }));
  const boxIds = new Set();
  wh.on('boxTaken', ({ box }) => { if (boxIds.has(box.id)) fails++; boxIds.add(box.id); });
  const R = wh.aisles.map((a) => ({
    a, fd: { v: [0, 0, 0], a: [0, 0, 0], j: [0, 0, 0] }, used: AX.map(() => [Infinity, -Infinity]),
    prevQ: [...a.q], prevV: [0, 0, 0], prevA: [0, 0, 0], minUnder: Infinity, minTop: Infinity, vL: 0, aL: 0, moveOut: 0, sunk: 0, deckBad: 0, visited: new Set(),
  }));
  let laneGap = Infinity, stackedTake = 0;
  const N = Math.round((HOURS * 3600) / DT);
  for (let n = 0; n < N; n++) {
    wh.step(DT);
    for (const r of R) {
      const a = r.a;
      for (let i = 0; i < 3; i++) {
        const v = (a.q[i] - r.prevQ[i]) / DT, ac = (v - r.prevV[i]) / DT, j = (ac - r.prevA[i]) / DT;
        r.fd.v[i] = max(r.fd.v[i], abs(v)); r.fd.a[i] = max(r.fd.a[i], abs(ac)); r.fd.j[i] = max(r.fd.j[i], abs(j));
        r.prevQ[i] = a.q[i]; r.prevV[i] = v; r.prevA[i] = ac;
        r.used[i][0] = min(r.used[i][0], a.q[i]); r.used[i][1] = max(r.used[i][1], a.q[i]);
      }
      // raw geometric facts, independent of monitor()
      const out = abs(a.q[2]) > FORK.centredWin;
      if (out && (a.qd[0] !== 0 || abs(a.qd[1]) > CRANE.hoistCreep.v + 1e-9)) r.moveOut++;
      if (abs(a.q[2]) > MARGIN.forkToRackFace && abs(a.q[0] - PD.x) < 0.01 && (a.q[2] > 0 ? a.stIn : a.stOut).deck !== 0) r.deckBad++;
      if (a.carried) { r.vL = max(r.vL, abs(a.qd[2])); r.aL = max(r.aL, abs(a.qdd[2])); }
      if (a.carried && !a.engaged) {
        const p = a.carried, yl = p.y - a.y;
        if (abs(yl) > RACK.yFace - LOAD.sy / 2) {
          const station = p.x < RACK.x1 - 1;
          const zs = station ? PD.z : levelZ(Math.round((p.z - RACK.z1) / RACK.pitchZ) + 1);
          r.minUnder = min(r.minUnder, p.z - zs);
          if (!station) r.minTop = min(r.minTop, zs + RACK.pitchZ - RACK.ledgeLegH - (p.z + p.h));
        }
      }
      if (a.engaged && a.engaged.pallet.z < a.engaged.zs - 1e-12) r.sunk++;
      if (a.op?.slot) r.visited.add(a.op.slot.id);
      if (n % 24 === 0)
        for (const lane of [[...a.inLane].reverse(), a.outLane].map((l) => l.map((p) => p.x)))
          for (let i = 1; i < lane.length; i++) laneGap = min(laneGap, abs(lane[i] - lane[i - 1]) - LOAD.sx);
    }
  }

  const mean = (a) => a.reduce((s, x) => s + x, 0) / a.length;
  for (const r of R) {
    const a = r.a, st = a.stats, tag = `aisle ${a.id} (${a.rows.map((x) => x[0]).join('+')})`;
    const okLim = [0, 1, 2].every((i) => r.fd.v[i] <= lim[i].v * 1.001 && r.fd.a[i] <= lim[i].a * 1.01 && r.fd.j[i] <= lim[i].j * 1.05);
    check(`${tag}: monitor violations`, st.violations === 0, `${st.violations}${st.violationLog[0] ? ' first: ' + JSON.stringify(st.violationLog[0]) : ''}`);
    check(`${tag}: no travel / fast hoist with fork out; station deck down whenever the fork is in it`, r.moveOut === 0 && r.deckBad === 0, `${r.moveOut} / ${r.deckBad} steps`);
    check(`${tag}: v / a / j within limits on all axes`, okLim,
      `travel ${f(r.fd.v[0])}/${f(r.fd.a[0])}/${f(r.fd.j[0], 2)}  hoist ${f(r.fd.v[1])}/${f(r.fd.a[1])}/${f(r.fd.j[1], 2)}  fork ${f(r.fd.v[2])}/${f(r.fd.a[2])}/${f(r.fd.j[2], 2)}`);
    check(`${tag}: loaded fork limits, pallet clearances`, r.vL <= CRANE.forkLoaded.v + 1e-9 && r.aL <= CRANE.forkLoaded.a + 1e-9 && r.minUnder >= FORK.cHigh - 1e-6 && r.minTop >= SAFE.minLoadClearTop && r.sunk === 0,
      `fork v ${f(r.vL)} a ${f(r.aL)}; above supports ≥ ${f(r.minUnder * 1000, 1)} mm; below level above ≥ ${f(r.minTop * 1000, 0)} mm`);
    console.log(`      ${st.stored} stored, ${st.retrieved} retrieved, ${r.visited.size}/${a.slots.length} locations visited, fill ${f(a.fill * 100, 0)} %, single ${f(mean(st.opTimes), 1)} s (${f(min(...st.opTimes), 1)}…${f(max(...st.opTimes), 1)}), dual ${f(mean(st.dualTimes), 1)} s, ${f((st.stored + st.retrieved) / HOURS, 1)} movements/h`);
  }
  check('pallets on a conveyor lane never touch', laneGap >= 0.15, `min gap ${f(laneGap * 1000, 0)} mm`);
  const inRack = wh.slots.filter((s) => s.pallet).length;
  const elsewhere = wh.aisles.reduce((s, a) => s + a.inLane.length + a.outLane.length + (a.carried ? 1 : 0) + (a.stIn.pallet || a.stIn.moving ? 1 : 0) + (a.stOut.pallet ? 1 : 0), 0);
  check('pallets conserved', wh.pallets.size === wh.counts.palletsIn - wh.counts.palletsOut && wh.pallets.size === inRack + elsewhere, `${wh.pallets.size} = ${inRack} in racks + ${elsewhere} on lanes / stations / forks`);
  const boxesNow = [...wh.pallets].reduce((s, p) => s + p.boxes.length, 0);
  check('boxes conserved', boxesNow === wh.counts.boxesIn - wh.counts.boxesOut && boxIds.size === wh.counts.boxesOut, `${wh.counts.boxesIn} created = ${boxesNow} in system + ${wh.counts.boxesOut} handed over (each id once)`);
  check('no location holds two pallets', new Set(wh.slots.filter((s) => s.pallet).map((s) => s.pallet.id)).size === inRack);
  void stackedTake;
  const st = wh.stats;
  console.log(`      total: ${f((st.stored + st.retrieved) / HOURS, 0)} pallet movements/h, ${f(wh.counts.palletsOut / HOURS, 1)} pallets/h and ${f(wh.counts.boxesOut / HOURS, 0)} boxes/h handed over at the output ports`);
}

// ---------- 7. output interface ----------
console.log('\n7. Output interface (external consumer)');
{
  const wh = new Warehouse({ seed: 11, consumer: 'external' });
  const run = (sec) => { for (let i = 0; i < sec * 240; i++) wh.step(DT); };
  run(400);
  const port = 'OUT-2', o = wh.outputs().find((x) => x.port === port);
  check('a pallet is presented at the port with its box list', !!o.pallet && o.pallet.boxes.length === o.pallet.boxesLeft && o.pallet.boxesLeft > 0, o.pallet ? `pallet #${o.pallet.id} ${o.pallet.sku}, ${o.pallet.boxesLeft} boxes at (${o.pallet.pos.map((v) => f(v, 2)).join(', ')})` : 'none');
  const p = wh.presented(port);
  const top = max(...p.boxes.map((b) => b.layer)), low = p.boxes.find((b) => b.layer < top);
  check('a box under another layer cannot be taken', !low || wh.takeBox(port, low.id) === null);
  const b0 = o.pallet.boxes.find((b) => b.pickable), got = wh.takeBox(port, b0.id);
  const bw = [PD.xPort, wh.aisles[1].stOut.y];
  check('a top-layer box is handed over with its world pose', got && got.id === b0.id && abs(got.pos[0] - bw[0]) <= LOAD.sx / 2 && abs(got.pos[1] - bw[1]) <= LOAD.sy / 2 && got.pos[2] > PD.z + LOAD.palletH, got ? `${got.id} ${got.sku} at (${got.pos.map((v) => f(v, 3)).join(', ')}) size ${got.size.join('×')}` : 'null');
  check('the same box cannot be taken twice', wh.takeBox(port, b0.id) === null);
  check('a pallet that still has boxes is not released', wh.releasePallet(port) === false);
  let layerOk = true, n = 1, prevLayer = Infinity;
  for (let g; (g = wh.takeBox(port)); n++) { if (g.layer > prevLayer) layerOk = false; prevLayer = g.layer; }
  check('default order empties the pallet top-down', layerOk && wh.presented(port).boxes.length === 0, `${n} boxes taken`);
  check('the empty pallet is released and the next one moves up', wh.releasePallet(port) === true && wh.presented(port) === null);
  run(3 * 3600);
  const blocked = wh.aisles.every((a) => a.outLane.length + (a.stOut.pallet ? 1 : 0) + (a.carried ? 1 : 0) >= PD.laneCap - 1);
  check('with nobody taking boxes the output lanes fill up and the cranes stop retrieving, without violations', blocked && wh.stats.violations === 0, `lanes ${wh.aisles.map((a) => a.outLane.length).join(' / ')} of ${PD.laneCap}, fill ${wh.aisles.map((a) => f(a.fill * 100, 0) + ' %').join(' / ')}`);
  const id = wh.requestRetrieval({ sku: SKUS[0].sku });
  check('retrieval can be requested by SKU', typeof id === 'string', `→ ${id}`);
  const snap = JSON.parse(JSON.stringify(wh.snapshot()));
  check('snapshot() is plain JSON', snap.cranes.length === LAYOUT.aisles.length && snap.outputs.length === LAYOUT.aisles.length && snap.inventory.length > 0, `${JSON.stringify(wh.snapshot()).length} bytes`);
  void boxWorld;
}

console.log(fails ? `\n${fails} CHECK(S) FAILED` : '\nALL CHECKS PASSED');
process.exit(fails ? 1 : 0);
