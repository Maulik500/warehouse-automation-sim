// Verification of the integrated warehouse (WAREHOUSE_SPEC.md §15).
//   node warehouse/test/verify.mjs            static checks + three fleet runs
//   node warehouse/test/verify.mjs --quick    static checks + one short run
//   node warehouse/test/verify.mjs --cells    also re-run the five cell suites first
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { ARM, PALLET as MP, DECK, DEG } from '../../mobman/js/config.js';
import * as A from '../../mobman/js/arm.js';
import * as B from '../../mobman/js/base.js';
import { palletBoxes, deckSlots } from '../../mobman/js/recipes.js';
import { ROBOT, CELL } from '../../js/config.js';
import { ik, toolDown, singularity } from '../../js/kinematics.js';
import * as FK from '../../amr/js/kinematics.js';
import { SKUS, SKU, PORT, PCELL, PTOOL, MIX, PLACE, ROAD, FLOOR, ROWS, PALLET, G, DT, toG } from '../js/config.js';
import { World } from '../js/sim.js';
import { RING, T, forkRoute, mmRoutes, cellDock, MM_STANDOFF } from '../js/roadmap.js';
import { orderSlots } from '../js/palletizer.js';
import { sortLayers, orderHeight } from '../js/wes.js';
import { holdPoints } from '../js/traffic.js';
import { layoutConflicts } from './layoutcheck.mjs';

const args = process.argv.slice(2), quick = args.includes('--quick');
let failed = 0;
const check = (ok, text, detail = '') => { console.log(`  ${ok ? 'ok  ' : 'FAIL'} ${text}${detail ? '  — ' + detail : ''}`); if (!ok) failed++; };
const head = (t) => console.log(`\n${t}`);
const f3 = (v) => v.toFixed(3), mm = (v) => (1000 * v).toFixed(0) + ' mm';
const { PI, abs, min, max, hypot } = Math;

// ---------- 0. the five cells on their own ----------
if (args.includes('--cells')) {
  head('Regression: the five cell suites');
  const root = fileURLToPath(new URL('../../', import.meta.url));
  for (const s of ['test/verify.mjs', 'asrs/test/verify.mjs', 'amr/test/verify.mjs', 'wrapper/test/verify.mjs', 'mobman/test/verify.mjs']) {
    let ok = true;
    try { execFileSync(process.execPath, [s], { cwd: root, stdio: 'pipe' }); } catch { ok = false; }
    check(ok, s);
  }
}

// ---------- 1. reach ----------
head('Reach');
{ // mobile arm at an ASRS port: every box of every SKU at every layer, ± scatter, grip / carry / empty-swing height (+ largest raise)
  const QREF = [0, -0.6, 1.5, 0.6, PI / 2, 0], pose = { x: 0, y: 0, th: 0 };
  let n = 0, bad = 0, minE = 9, minS = 9, zLo = 9, zHi = 0;
  const probe = (p, yaw) => {
    n++;
    for (const y of [yaw, yaw + PI]) {
      const q = A.ik(B.worldToArm(pose, p), A.toolDown(B.yawToArm(pose, y)), QREF, ARM.branch);
      if (q) { const s = A.singularity(q); minE = min(minE, abs(s.elbow)); minS = min(minS, abs(s.shoulder)); zLo = min(zLo, p[2]); zHi = max(zHi, p[2]); return; }
    }
    bad++;
  };
  for (const S of SKUS) {
    const rc = { ...S.rc, layers: S.srcLayers };
    for (const dx of [-PORT.scatter, PORT.scatter]) for (const dy of [-PORT.scatter, PORT.scatter]) for (const b of palletBoxes(rc)) {
      const top = PORT.z + b.z + rc.h / 2, p = [b.x + dx, 1.25 + b.y + dy];
      for (const raise of [0, 0.12]) { probe([...p, top], b.yaw); probe([...p, top + rc.h + 0.08 + raise], b.yaw); probe([...p, max(top, DECK.z + rc.h) + 0.12 + raise], b.yaw); }
    }
    for (const s of deckSlots(rc)) for (const z of [DECK.z + rc.h, PORT.z + MP.h + S.srcLayers * rc.h + rc.h + 0.2]) probe([s.x, s.y, z], s.yaw);
  }
  check(bad === 0, `mobile arm, pallet support at ${PORT.z} m: ${n} probe poses, ${bad} unreachable`, `TCP z ${f3(zLo)} … ${f3(zHi)}, |sin q3| ≥ ${f3(minE)}, shoulder ≥ ${f3(minS)} m`);
  check(minE >= ARM.minElbow && minS >= ARM.minShoulder, 'mobile arm singularity margins over the task space', `elbow ${f3(minE)} ≥ ${ARM.minElbow}, shoulder ${f3(minS)} ≥ ${ARM.minShoulder}`);
}
{ // palletizing robot: every slot of the worst stacks, at landing and at hover height; every box on the belt
  const ref = [PI / 2, 0, 0, 0, PI / 2, 0], cv = CELL.conv;
  let n = 0, bad = 0, minW = 9, minE = 9, minS = 9, worst = '';
  const probe = (p, tag) => {
    n++;
    const q = [0, PI / 2, PI, -PI / 2].map((psi) => ik(p, toolDown(psi), ref)).find((x) => x && x[0] >= PCELL.q1Min && x[0] <= PCELL.q1Max);
    if (!q) { bad++; return; }
    const s = singularity(q);
    minW = min(minW, abs(s.wrist)); minS = min(minS, abs(s.shoulder));
    if (abs(s.elbow) < minE) { minE = abs(s.elbow); worst = tag; }
  };
  // stacks: for every SKU as the top layer, the tallest stack that the height limit allows below it
  for (const top of SKUS) {
    const below = [];
    while (orderHeight([...below, 'S4', top.id]) <= MIX.maxBoxH + 1e-9) below.push('S4');
    for (const fill of [[], below]) {
      const layers = [...fill, top.id], slots = orderSlots(layers).filter((s) => s.layer === layers.length - 1);
      for (const s of slots) { probe([s.x, s.y, s.zTop], top.id); probe([s.x, s.y, max(s.zTop + s.h + PCELL.hoverExtra, cv.top + s.h + PCELL.beltClear)], `${top.id} on ${fill.length} layers, hover`); }
    }
  }
  for (const S of SKUS) for (const y of [cv.yA, 0, cv.yB]) for (const dz of [0, CELL.hoverPick, CELL.liftPick, 0.7]) probe([cv.x, y, cv.top + S.h + dz], `${S.id} on the belt`);
  check(bad === 0, `palletizing robot: ${n} probe poses (slots at landing and hover height, belt), ${bad} unreachable inside J1 ${(PCELL.q1Min / DEG).toFixed(0)}…${(PCELL.q1Max / DEG).toFixed(0)}°`);
  check(minW >= PCELL.minWrist && minE >= PCELL.minElbow && minS >= PCELL.minShoulder, 'palletizing robot singularity margins over the task space', `wrist ${f3(minW)}, elbow ${f3(minE)} (${worst}), shoulder ${f3(minS)} m`);
}

// ---------- 2. loads and grippers ----------
head('Loads and grippers');
{
  check(SKUS.every((S) => S.mass <= 15), 'no box heavier than 15 kg', SKUS.map((S) => `${S.id} ${S.mass}`).join(', '));
  check(SKUS.every((S) => PALLET.h + S.srcLayers * S.h <= 1.2 + 1e-9), 'every source pallet ≤ 1.20 m', SKUS.map((S) => (PALLET.h + S.srcLayers * S.h).toFixed(3)).join(', '));
  check(SKUS.every((S) => PTOOL.padX <= S.l - 0.04 + 1e-9 && PTOOL.padY <= S.w - 0.04 + 1e-9), 'palletizer pad 20 mm inside the smallest box top on every side');
  const VAC = PTOOL.cups * PTOOL.vacuum * PI * (PTOOL.cupDia / 2) ** 2, PEEL = (VAC / 2) * 2 * PTOOL.rowY, ah = 9.2, az = 3.5;
  const rows = SKUS.map((S) => { const k = PCELL.dyn[S.id]; return { id: S.id, sf: VAC / (S.mass * (G + az + ah / PTOOL.mu)), peel: PEEL / (S.mass * ah * k * (S.h / 2) + S.mass * (G + az) * PTOOL.rowY) }; });
  check(rows.every((r) => r.sf >= 2), 'palletizer suction SF at the verified worst-case acceleration', rows.map((r) => `${r.id} ${r.sf.toFixed(2)}`).join(', '));
  check(rows.every((r) => r.peel >= 1.5), 'palletizer peel SF with the per-SKU dynamics', rows.map((r) => `${r.id} ${r.peel.toFixed(2)}`).join(', '));
  const srt = sortLayers(['S5', 'S3', 'S1', 'S4', 'S2']);
  check(srt.join() === 'S1,S4,S3,S2,S5', 'layer sequence = descending layer mass', srt.join(' → '));
}

// ---------- 3. layout, transforms, roads ----------
head('Layout and roads');
const w0 = new World({ forklifts: 1, robots: 1 });
{
  const e = RING.end, s = RING.at(0);
  check(hypot(e.x - s.x, e.y - s.y) < 1e-9, 'forklift ring closes on itself', `length ${RING.length.toFixed(2)} m, error ${hypot(e.x - s.x, e.y - s.y).toExponential(1)} m`);
  let worst = 0, minRun = 9;
  const all = [...w0.stations, ...w0.parking];
  for (const st of all) { const p = st.spur.path.end; worst = max(worst, hypot(p.x - st.spur.pre.x, p.y - st.spur.pre.y)); minRun = min(minRun, st.spur.stem); }
  check(worst < 1e-9, `${all.length} spurs end on their pre-dock pose`, worst.toExponential(1) + ' m');
  check(minRun >= ROAD.runIn - 1e-9, 'straight run-in before every pre-dock point', `≥ ${f3(minRun)} m`);
  let wr = 0;
  for (const a of [w0.parking[0], w0.cells[3].station, w0.asrs.lanes.A]) for (const b of [w0.wrap.I4, w0.storage[11], w0.empties[0], w0.asrs.lanes.E, w0.cells[0].station]) { const r = forkRoute(a.spur, b.spur), p = r.end; wr = max(wr, hypot(p.x - b.spur.pre.x, p.y - b.spur.pre.y)); }
  check(wr < 1e-8, '15 forklift routes end on the pre-dock pose of their station', wr.toExponential(1) + ' m');
  let dockErr = 0;
  for (const st of w0.stations) { const d = st.spur.dockPath.end, t = FK.tcpWorld({ x: d.x, y: d.y, th: d.th }, 0, 0.3, 0); dockErr = max(dockErr, hypot(t[0] - st.x, t[1] - st.y)); }
  check(dockErr < 1e-9, 'docked forklift TCP = pallet centre at every station', dockErr.toExponential(1) + ' m');
  let me = 0, cont = true, straight = true;
  for (let i = 0; i < 4; i++) {
    const r = mmRoutes(i), d = cellDock(i);
    me = max(me, hypot(r.out.end.x - r.pre.x, r.out.end.y - r.pre.y), hypot(r.dock.end.x - d.x, r.dock.end.y - d.y), hypot(r.back.end.x - ROAD.mm.xRet, r.back.end.y - ROAD.mm.yTop));
    for (const p of [r.out, r.back, RING]) {
      for (let k = 1; k < p.pieces.length; k++) if (abs(p.pieces[k].k0 - p.pieces[k - 1].k1) > 1e-12) cont = false;
      for (const h of holdPoints(p, 0, p.length)) { const c = p.curv(h); if (c.k !== 0 || c.dk !== 0) straight = false; }
    }
  }
  check(me < 1e-8, 'robot-AMR routes end on the cell pre-dock, the dock and the head of the return lane', me.toExponential(1) + ' m');
  check(cont, 'curvature continuous along every road');
  check(straight, 'every permitted stopping point lies on a straight piece (κ = 0, κ′ = 0)');
  // stations of the specification table
  const D = w0.cells.map((c) => c.station), near = (a, b) => abs(a - b) < 1e-9;
  check(D.every((s, i) => near(s.x, [7.55, 15.75, 23.95, 32.15][i]) && near(s.y, 17.6)), 'palletizer positions D', D.map((s) => `(${s.x.toFixed(2)}, ${s.y.toFixed(2)})`).join(' '));
  check(ROWS.every((r, i) => near(w0.asrs.lanes[r].x, 41) && near(w0.asrs.lanes[r].y, [36.4, 33.6, 32.1, 29.3, 27.8][i])), 'ASRS ports A–E at x = 41.00', ROWS.map((r) => w0.asrs.lanes[r].y.toFixed(2)).join(', '));
  const cd = [0, 1, 2, 3].map(cellDock);
  check(cd.every((d, i) => near(d.x, [1.56, 9.76, 17.96, 26.16][i]) && near(d.y, 8.37) && near(d.th, PI / 2)), 'robot-AMR docks at the palletizing cells', cd.map((d) => `(${d.x.toFixed(2)}, ${d.y.toFixed(2)})`).join(' '));
  const lc = layoutConflicts(w0);
  check(lc.length === 0, 'no standing position lies inside the ring sweep or inside the entry sweep of another station', lc.slice(0, 3).join('; '));
  const M = ROAD.mm;
  check(M.xRoad - M.xRet >= 1.64 + 0.3 && M.yW - M.yE >= 1.64 + 0.3 && M.xRet - ROAD.fl.xE >= 1.62 + 0.3, 'lane spacings leave ≥ 0.30 m between claims', `${f3(M.xRoad - M.xRet)} / ${f3(M.yW - M.yE)} / ${f3(M.xRet - ROAD.fl.xE)} m`);
  // mobile arm beside the port: chassis to the lane frame, J1 axis to the pallet
  const L = w0.asrs.lanes.A;
  check(near(L.x - M.xRoad, 1.25), 'robot AMR reference 1.25 m from the pallet centre at a port (verified side-on geometry)');
  check(L.frame.x0 - (M.xRoad + 0.62) >= 0.15, 'chassis ↔ lane frame end at a port', mm(L.frame.x0 - (M.xRoad + 0.62)));
}

// ---------- 4. runs ----------
function run(forklifts, robots, hours) {
  const w = new World({ forklifts, robots }), built = [], t0 = Date.now();
  w.on('palletBuilt', ({ pallet, order }) => built.push({ id: pallet.id, order: order.id, want: [...order.layers], got: pallet.layerSkus, h: pallet.h, boxes: pallet.boxes.length, n: order.slots.length }));
  let error = null, lastShip = 0, tShip = 0, stall = 0, mark = 3600, conserved = true;
  const count = () => { const c = w.boxCount(); return c.created === c.rack + c.tracked + c.shipped; };
  try {
    while (w.t < hours * 3600) {
      w.step(DT);
      if (w.stats.shipped !== lastShip) { lastShip = w.stats.shipped; tShip = w.t; }
      stall = max(stall, w.t - tShip);
      if (w.t >= mark) { mark += 3600; conserved = conserved && count(); }
    }
  } catch (e) { error = e; }
  head(`Run: ${forklifts} forklift(s) + ${robots} robot AMR(s), ${hours} h simulated at 240 Hz (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
  check(!error, 'no planning, IK or sequencing error', error ? error.message : '');
  const v = w.violations();
  check(v.asrs === 0 && v.wrapper === 0, 'ASRS (R1–R7, 3 cranes) and wrapper (R1–R10) monitors', `${v.asrs} / ${v.wrapper} violations`);
  check(v.cells === 0, 'palletizing cell monitors (4 robots)', `${v.cells} violations` + (v.cells ? ' ' + w.cells.flatMap((c) => c.stats.firstBad).slice(0, 2).join(' | ') : ''));
  check(v.robots === 0, 'robot AMR monitors (M1–M12 + peel)', `${v.robots} violations` + (v.robots ? ' ' + w.robots.flatMap((r) => r.stats.firstBad).slice(0, 2).join(' | ') : ''));
  check(v.forklifts === 0, 'forklift monitors (R1–R10)', `${v.forklifts} violations` + (v.forklifts ? ' ' + w.forklifts.flatMap((f) => f.stats.firstBad).slice(0, 2).join(' | ') : ''));
  check(v.fleet === 0 && w.stats.minGap >= ROAD.gap, 'fleet monitor: vehicle outlines never closer than 0.30 m', `min ${f3(w.stats.minGap)} m`);
  const R = w.raw, mr = w.robots.reduce((s, r) => ({ a: s.a + r.stats.armWhileMoving, b: s.b + r.stats.baseWhileArm, z: s.z + r.stats.zoneNoLock }), { a: 0, b: 0, z: 0 });
  check(Object.values(R).every((x) => x === 0), 'raw checks: conveyor moved under a locked station / robot moved with a forklift at D / forklift in a zone without Locked', Object.values(R).join(' / '));
  check(mr.a === 0 && mr.b === 0 && mr.z === 0, 'raw checks: base moved with the arm not stowed / arm moved while the base moved / port zone without Locked', `${mr.a} / ${mr.b} / ${mr.z}`);
  check(w.forklifts.every((f) => f.stats.mastWhileMoving === 0), 'raw check: mast axis moving while a forklift moves', '0 steps');
  const missed = w.cells.reduce((s, c) => s + c.stats.missed, 0), ikf = w.cells.reduce((s, c) => s + c.stats.ikFail, 0) + w.robots.reduce((s, r) => s + r.stats.ikFail, 0);
  check(missed === 0 && ikf === 0, 'boxes missed on a tracking belt / IK failures', `${missed} / ${ikf}`);
  const bc = w.boxCount();
  check(conserved && count(), 'boxes conserved (checked every hour)', `${bc.created} created = ${bc.rack} in the rack + ${bc.tracked} in flow + ${bc.shipped} shipped`);
  const ids = new Set();
  for (const p of [...w.asrs.pallets, ...w.loose()]) ids.add(p);
  const pTot = w.asrs.counts.palletsIn + FLOOR.emptyStart.reduce((a, b) => a + b, 0) + 4;
  check(ids.size + w.stats.shipped + w.stats.emptiesOut === pTot, 'pallets conserved', `${pTot} = ${ids.size} in the building + ${w.stats.shipped} shipped + ${w.stats.emptiesOut} empties collected`);
  const okOrder = built.every((b) => b.want.join() === b.got.join() && b.boxes === b.n && b.h <= MIX.maxH + 1e-9 && sortLayers(b.want).join() === b.want.join());
  check(built.length > 0 && okOrder, `every built pallet matches its order layer by layer (${built.length} pallets)`, built.length ? `heights ${f3(min(...built.map((b) => b.h)))} … ${f3(max(...built.map((b) => b.h)))} m` : '');
  check(w.shippedLog.every((s) => s.wrapped), `every shipped pallet was wrapped (${w.stats.shipped})`);
  const maxWait = max(...[...w.robots, ...w.forklifts].map((x) => x.drive.maxWait));
  check(stall < 3600 && w.stats.shipped > 0, 'no deadlock: pallets keep leaving', `longest gap between two shipped pallets ${(stall / 60).toFixed(0)} min, longest traffic wait ${maxWait.toFixed(0)} s`);
  // margins and peaks
  const C = w.cells, pk = (f) => max(...C.map(f)), mn = (f) => min(...C.map(f));
  check(pk((c) => max(...c.stats.jv)) <= 1.005, 'palletizing robots: peak joint speed / limit', [0, 1, 2, 3, 4, 5].map((i) => pk((c) => c.stats.jv[i]).toFixed(2)).join(' · '));
  console.log(`       palletizing robots: wrist ≥ ${f3(mn((c) => c.stats.minWrist))}, elbow ≥ ${f3(mn((c) => c.stats.minElbow))}, shoulder ≥ ${f3(mn((c) => c.stats.minShoulder))} m; vacuum SF ≥ ${mn((c) => c.stats.minSF).toFixed(2)}, peel SF ≥ ${mn((c) => c.stats.minPeel).toFixed(2)}; carried box clearance ≥ ${mm(mn((c) => c.stats.minCarry))}; place error ${pk((c) => c.stats.placeErr).toExponential(1)} m; J1 ${(mn((c) => c.stats.qLo[0]) / DEG).toFixed(0)} … ${(pk((c) => c.stats.qHi[0]) / DEG).toFixed(0)}°`);
  const Rb = w.robots, rmn = (f) => min(...Rb.map(f)), rmx = (f) => max(...Rb.map(f));
  console.log(`       robot AMRs: elbow ≥ ${f3(rmn((r) => r.stats.minElbow))}, shoulder ≥ ${f3(rmn((r) => r.stats.minShoulder))} m; vacuum SF ≥ ${rmn((r) => r.stats.minSF).toFixed(2)}, peel SF ≥ ${rmn((r) => r.stats.minPeel).toFixed(2)}; arm clearance ≥ ${mm(rmn((r) => r.stats.minArmClear.d))}, carried box ≥ ${mm(rmn((r) => r.stats.minCarry.d))}; largest swing raise ${mm(rmx((r) => r.stats.maxRaise))}; v ≤ ${rmx((r) => r.stats.vFwd).toFixed(3)} m/s, loaded ≤ ${rmx((r) => r.stats.vLoaded).toFixed(3)}, in a zone ≤ ${rmx((r) => r.stats.vZone).toFixed(3)}; yaw rate ≤ ${(rmx((r) => r.stats.omega) / DEG).toFixed(1)}°/s; CG margin ≥ ${f3(rmn((r) => r.stats.minStab))} m; dead reckoning ${mm(rmx((r) => r.stats.odoErr))} over ${(Rb.reduce((s, r) => s + r.stats.dist, 0) / 1000).toFixed(1)} km`);
  const F = w.forklifts, fmx = (f) => max(...F.map(f));
  console.log(`       forklifts: v ≤ ${fmx((f) => f.stats.vEmpty).toFixed(3)} empty / ${fmx((f) => f.stats.vLoaded).toFixed(3)} loaded / ${fmx((f) => f.stats.vRev).toFixed(3)} reverse / ${fmx((f) => f.stats.vZone).toFixed(3)} in a fork zone; steer ≤ ${(fmx((f) => f.stats.steer) / DEG).toFixed(1)}°, ${(fmx((f) => f.stats.steerRate) / DEG).toFixed(1)}°/s; lateral ≤ ${fmx((f) => f.stats.aLat).toFixed(3)} m/s²; lift ${f3(min(...F.map((f) => f.stats.lift[0])))} … ${f3(fmx((f) => f.stats.lift[1]))}; plan clearance ≥ ${mm(min(...F.map((f) => f.stats.minClear)))}; dead reckoning ${mm(fmx((f) => f.stats.odoErr))} over ${(F.reduce((s, f) => s + f.stats.dist, 0) / 1000).toFixed(1)} km`);
  // throughput
  const H = w.t / 3600, layers = Rb.reduce((s, r) => s + r.stats.layers, 0), trips = Rb.flatMap((r) => r.stats.tripT), jobs = F.reduce((s, f) => s + f.stats.jobs, 0), cyc = F.flatMap((f) => f.stats.cycle);
  console.log(`       throughput: ${built.length} pallets built = ${(built.length / H).toFixed(1)} / h, ${w.stats.shipped} shipped; ${layers} layers = ${(layers / H).toFixed(1)} / h; robot trip ${(trips.reduce((a, b) => a + b, 0) / max(1, trips.length)).toFixed(0)} s mean; ${jobs} forklift jobs (${Object.entries(w.fleet.stats.done).map((e) => e.join(' ')).join(', ')}), ${(cyc.reduce((a, b) => a + b, 0) / max(1, cyc.length)).toFixed(0)} s mean; wrapper ${w.wrap.stats.wrapped} wrapped; cranes ${w.asrs.aisles.map((a) => a.stats.retrieved + a.stats.stored).join(' / ')} movements`);
  return { built: built.length / H, layers: layers / H };
}
const plan = quick ? [[3, 2, 1]] : [[3, 2, 4], [1, 1, 2], [6, 2, 2]];
const res = plan.map(([f, r, h]) => ({ f, r, ...run(f, r, h) }));
head('Throughput by fleet');
for (const r of res) console.log(`  ${r.f} forklifts + ${r.r} robot AMRs: ${r.built.toFixed(1)} mixed pallets / h, ${r.layers.toFixed(1)} layers / h`);

console.log(failed ? `\n${failed} CHECK(S) FAILED` : '\nall checks passed');
process.exit(failed ? 1 : 0);
