// Runs the integrated warehouse headless and writes a compressed recording for the presentation,
// the offline file and the video:   node warehouse/tools/record.mjs [seconds] [warmup] [fps]
// Output: warehouse/rec/clip.json.gz   (every pose of every machine, pallet and box, the KPIs and
// the event log the guided tour is built from)

import { writeFileSync, mkdirSync } from 'node:fs';
import { gzipSync } from 'node:zlib';
import { fileURLToPath } from 'node:url';
import { World } from '../js/sim.js';
import { DT, ROWS } from '../js/config.js';
import { snapshot, Recorder } from '../js/state.js';

const SECONDS = +process.argv[2] || 1800, WARMUP = +process.argv[3] || 900, FPS = +process.argv[4] || 10, FORKS = 3, ROBOTS = 2;
const w = new World({ seed: 7, forklifts: FORKS, robots: ROBOTS });
const t0 = Date.now();
while (w.t < WARMUP) w.step(DT);
const rec = new Recorder({ fps: FPS, nRobots: ROBOTS, nForks: FORKS, meta: { t0: WARMUP, seed: 7, forklifts: FORKS, robots: ROBOTS } });

// event log: every state change of every machine, plus the world's own events
const prev = new Map();
const watch = (who, what, info) => { if (prev.get(who) !== what) { prev.set(who, what); rec.event({ who, what, ...info }); } };
for (const ev of ['layerLoaded', 'layerDelivered', 'layerPlaced', 'palletBuilt', 'jobDone'])
  w.on(ev, (d) => rec.event({ who: 'world', what: ev, cell: typeof d.cell === 'number' ? d.cell : typeof d.job?.cell === 'number' ? d.job.cell : d.job?.cell?.i, robot: d.robot, order: d.order?.id ?? d.job?.order, kind: d.job?.kind, pallet: d.pallet?.id }));
const kpi = () => {
  const bc = w.boxCount(), v = w.violations();
  return {
    t: +w.t.toFixed(1), done: w.wes.stats.done, open: w.wes.orders.filter((o) => o.state === 'open').length, active: w.wes.orders.filter((o) => o.state === 'active').length,
    layers: w.robots.reduce((s, r) => s + r.stats.layers, 0), built: w.cells.reduce((s, c) => s + c.stats.pallets, 0), placed: w.cells.reduce((s, c) => s + c.stats.boxes, 0),
    wrapped: w.wrap.stats.wrapped, stored: w.fleet.stats.done.store ?? 0, shipped: w.stats.shipped, rack: bc.rack, flow: bc.tracked,
    viol: Object.values(v).reduce((a, b) => a + b, 0), gap: +w.stats.minGap.toFixed(3),
    cells: w.cells.map((c) => (c.order ? [c.order.id, c.order.placed, c.order.slots.length, c.order.layers] : null)),
  };
};
const every = Math.round(1 / (FPS * DT));
let n = 0;
while (w.t < WARMUP + SECONDS) {
  if (n++ % every === 0) {
    w.robots.forEach((m) => watch(m.id, m.phase, { row: m.job?.row, cell: m.job?.cell, sku: m.job?.sku }));
    w.forklifts.forEach((f) => watch(f.id, f.phase, { kind: f.job?.kind, pick: f.job?.pick.id, drop: f.job?.drop.id }));
    w.cells.forEach((c) => { watch(c.id, c.palState); watch(c.id + '.robot', c.state === 'TRACK' || c.state === 'PTP_PICK' ? 'PICK' : c.state === 'PTP_PLACE' || c.state === 'PLACE_DOWN' ? 'PLACE' : 'IDLE'); });
    w.asrs.aisles.forEach((a) => watch(`CR${a.id}`, a.op ? `${a.op.kind} ${a.op.slot.row ?? ''}` : 'IDLE', { slot: a.op?.slot.id }));
    ROWS.forEach((r) => watch(`PORT-${r}`, w.asrs.lanes[r].state));
    watch('WRAP', w.wrap.w.phase);
    rec.add(snapshot(w), { kpi: kpi() });
  }
  w.step(DT);
}
const json = JSON.stringify(rec.toJSON()), gz = gzipSync(json, { level: 9 });
const dir = fileURLToPath(new URL('../rec/', import.meta.url));
mkdirSync(dir, { recursive: true });
writeFileSync(dir + 'clip.json.gz', gz);
const v = w.violations();
console.log(`clip.json.gz: ${rec.frames} frames (${SECONDS} s at ${FPS} fps from t = ${WARMUP} s), ${(json.length / 1e6).toFixed(1)} MB JSON -> ${(gz.length / 1e6).toFixed(2)} MB gzip; ${rec.events.length} events; ${rec.tracks.boxes.size} boxes, ${rec.tracks.pallets.size} pallets; violations ${Object.values(v).reduce((a, b) => a + b, 0)}; ${((Date.now() - t0) / 1000).toFixed(0)} s`);
