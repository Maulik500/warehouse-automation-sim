// Writes blender/mobman_params.json: every parameter from config.js, the docked poses, the route
// polyline, and a sampled trajectory of the base, the six arm joints, the deck stop, the pallet
// and every box, for blender/build_mobman.py.   `npm run export:mobman`

import { writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import * as cfg from '../js/config.js';
import { DH } from '../js/arm.js';
import { Cell, SRC_DOCK, DST_DOCK, Q_TRAVEL } from '../js/sim.js';

const SECONDS = +process.argv[2] || 175, FPS = 24, DT = 1 / 240;
const cell = new Cell({ seed: 7, clearEvery: 8 });
const r = (v) => +v.toFixed(4);

const route = Array.from({ length: Math.ceil(cell.path.length / 0.1) + 1 }, (_, i) => { const q = cell.path.at(Math.min(i * 0.1, cell.path.length)); return [r(q.x), r(q.y)]; });

// frames: [x, y, θ, wheel L, wheel R, q1 … q6, front stop (1 = up), vacuum (0 / 1)]
// boxes and pallets: sparse tracks [frame, x, y, z, yaw] — a key where the pose changes, plus a
// hold key on the frame before, so linear interpolation reproduces the motion exactly.
const frames = [], boxes = new Map(), pallets = new Map(), sub = Math.round(1 / (FPS * DT)), N = SECONDS * FPS;
function track(map, id, f, pose, make) {
  if (!map.has(id)) map.set(id, { ...make(), first: f, last: f, track: [], prev: null, prevF: -1 });
  const e = map.get(id);
  e.last = f;
  if (!e.prev || pose.some((v, i) => Math.abs(v - e.prev[i]) > 1e-5)) {
    if (e.prev && e.keyF < f - 1) e.track.push([f - 1, ...e.prev]);
    e.track.push([f, ...pose]);
    e.keyF = f;
  }
  e.prev = pose;
}
for (let f = 0; f <= N; f++) {
  frames.push([cell.pose.x, cell.pose.y, cell.pose.th, cell.wheel.L, cell.wheel.R, ...cell.q, cell.deck.stop, cell.vac ? 1 : 0].map(r));
  const p = cell.src.pallet;
  if (p) track(pallets, p.id, f, [p.x, p.y, p.z, p.yaw].map(r), () => ({ id: p.id, recipe: p.rc.id }));
  for (const b of cell.boxes) track(boxes, b.id, f, [...b.p, b.yaw].map(r), () => ({ id: b.id, rc: b.rcIx, size: [b.sx, b.sy, b.h] }));
  for (let i = 0; i < sub; i++) cell.step(DT);
}
const clean = (m) => [...m.values()].map(({ prev, prevF, keyF, ...e }) => e);

const out = fileURLToPath(new URL('../blender/mobman_params.json', import.meta.url));
const { AMR, DECK, ARM, TOOL, PALLET, RECIPES, SRC, DST, DOCK } = cfg;
writeFileSync(out, JSON.stringify({ AMR, DECK, ARM, TOOL, PALLET, RECIPES, SRC, DST, DOCK, DH, SRC_DOCK, DST_DOCK, Q_TRAVEL, route, fps: FPS, frames, pallets: clean(pallets), boxes: clean(boxes) }));
console.log(`${out}: ${frames.length} frames (${SECONDS} s at ${FPS} fps), ${boxes.size} boxes, ${pallets.size} pallet(s), ${cell.stats.boxes} boxes moved, ${cell.stats.trips} trip(s)`);
