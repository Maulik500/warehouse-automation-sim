// Writes blender/warehouse_params.json for blender/build_warehouse.py: every parameter the
// scene is built from, the roads, and a sampled trajectory of every joint of every machine and
// vehicle, every pallet and every box.   `npm run export:warehouse [-- seconds warmup fps]`
// The clip starts after a warm-up so that all stations are working.

import { writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { CRANE, FORK as AFORK, RACK, PD, LAYOUT } from '../../asrs/js/config.js';
import { AMR as FAMR, MAST, FORK as FFORK } from '../../amr/js/config.js';
import { AMR as MAMR, ARM, TOOL as MTOOL, DECK } from '../../mobman/js/config.js';
import { DH } from '../../mobman/js/arm.js';
import { ROBOT, CELL } from '../../js/config.js';
import { LINE, TABLE, CARRIAGE, CLAMP, CUT } from '../../wrapper/js/config.js';
import * as cfg from '../js/config.js';
import { World } from '../js/sim.js';
import { RING, mmRoutes, MM_LINK } from '../js/roadmap.js';
import { palletG } from '../js/asrs.js';

const SECONDS = +process.argv[2] || 240, WARMUP = +process.argv[3] || 1500, FPS = +process.argv[4] || 12, DT = cfg.DT;
const world = new World({ seed: 7, forklifts: 3, robots: 2 });
while (world.t < WARMUP) world.step(DT);
const r = (v) => +v.toFixed(4);
const line = (p, d = 0.2) => Array.from({ length: Math.ceil(p.length / d) + 1 }, (_, i) => { const q = p.at(Math.min(i * d, p.length)); return [r(q.x), r(q.y)]; });
const roads = { fork: [line(RING), ...[...world.stations, ...world.parking].flatMap((s) => [line(s.spur.path), ...(s.kind === 'home' ? [] : [line(s.spur.dockPath)])])], mm: [...[0, 1, 2, 3].flatMap((i) => { const m = mmRoutes(i); return [line(m.out), line(m.back), line(m.dock)]; }), line(MM_LINK)] };
const lanes = cfg.ROWS.map((row) => { const l = world.asrs.lanes[row]; return { row, x: l.x, y: l.y, frame: l.frame }; });

// sparse tracks [frame, x, y, z, yaw]: a key where the pose changes plus a hold key before it
const boxes = new Map(), pallets = new Map(), rack = new Map(), sub = Math.round(1 / (FPS * DT)), N = SECONDS * FPS;
function track(map, id, f, pose, make) {
  if (!map.has(id)) map.set(id, { ...make(), first: f, last: f, track: [], prev: null });
  const e = map.get(id);
  e.last = f;
  if (!e.prev || pose.some((v, i) => Math.abs(v - e.prev[i]) > 1e-5)) {
    if (e.prev && e.keyF < f - 1) e.track.push([f - 1, ...e.prev]);
    e.track.push([f, ...pose]);
    e.keyF = f;
  }
  e.prev = pose;
}
const F = { cranes: [], robots: [], mm: [], forks: [], wrap: [], stops: [] };
for (let f = 0; f <= N; f++) {
  const w = world;
  F.cranes.push(w.asrs.aisles.flatMap((a) => a.q).map(r));
  F.robots.push(w.cells.flatMap((c) => c.q).map(r));
  F.mm.push(w.robots.flatMap((m) => [m.pose.x, m.pose.y, m.pose.th, m.wheel.L, m.wheel.R, ...m.q, m.deck.stop]).map(r));
  F.forks.push(w.forklifts.flatMap((v) => [v.pose.x, v.pose.y, v.pose.th, v.phi, v.tilt, v.lift, v.shift]).map(r));
  F.wrap.push([w.wrap.w.thBase + w.wrap.w.th, w.wrap.w.lift, w.wrap.w.clampUp, w.wrap.w.jaw, w.wrap.w.slide].map(r));
  F.stops.push(cfg.ROWS.map((row) => r(w.asrs.lanes[row].stop)));
  const seen = new Set();
  for (const p of [...w.loose(), ...w.asrs.active()]) {
    if (seen.has(p)) continue;
    seen.add(p);
    const g = palletG(p);
    track(pallets, p.id, f, [g[0], g[1], g[2], p.yaw].map(r), () => ({ id: p.id }));
    const e = pallets.get(p.id);
    if (p.wrapped && e.wrapF === undefined) { e.wrapF = f; e.h = r(p.h); }
  }
  for (const b of w.asrs.boxes) track(boxes, b.id, f, [...b.p, b.yaw].map(r), () => ({ id: b.id, sku: b.sku, size: [b.sx, b.sy, b.h] }));
  for (const a of w.asrs.aisles) for (const s of a.slots) { const p = s.pallet; if (p && !p.boxes) track(rack, `${p.id}:${p.layers}:${s.id}`, f, [cfg.PLACE.asrs.tx + s.x, cfg.PLACE.asrs.ty + s.y, s.zs, 0].map(r), () => ({ sku: p.sku, h: r(p.layers * p.skuDef.h) })); }
  for (let i = 0; i < sub; i++) w.step(DT);
}
const clean = (m) => [...m.values()].map(({ prev, keyF, ...e }) => e);
const out = fileURLToPath(new URL('../blender/warehouse_params.json', import.meta.url));
const { BUILDING, PLACE, PORT, PCELL, PTOOL, ROAD, FLOOR, SKUS, PALLET, WRAPX, ROWS } = cfg;
writeFileSync(out, JSON.stringify({
  BUILDING, PLACE, PORT, PCELL, PTOOL, ROAD, FLOOR, SKUS, PALLET, WRAPX, ROWS,
  CRANE, AFORK, RACK, PD, LAYOUT, FAMR, MAST, FFORK, MAMR, ARM, MTOOL, DECK, DH, ROBOT, CELL, LINE, TABLE, CARRIAGE, CLAMP, CUT,
  roads, lanes, fps: FPS, t0: WARMUP, nFork: world.forklifts.length, nMm: world.robots.length, frames: F,
  pallets: clean(pallets), boxes: clean(boxes), rack: clean(rack),
}));
const v = world.violations();
console.log(`${out}: ${N + 1} frames (${SECONDS} s at ${FPS} fps, from t = ${WARMUP} s), ${boxes.size} boxes, ${pallets.size} pallets, ${rack.size} rack pallets; monitor violations ${Object.values(v).reduce((a, b) => a + b, 0)}`);
