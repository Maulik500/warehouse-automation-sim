// Writes blender/amr_params.json: every parameter from config.js, the station layout, the
// route polylines, and a sampled trajectory of every AMR joint and every pallet, for
// blender/build_amr.py.   `npm run export:amr`

import { writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import * as cfg from '../js/config.js';
import { Cell, stationPoses } from '../js/sim.js';

const SECONDS = 150, FPS = 24, DT = 1 / 240;
const cell = new Cell({ seed: 7 }), a = cell.amr;
const r = (v) => +v.toFixed(4);

// routes as polylines on the floor
const poly = (p) => Array.from({ length: Math.ceil(p.length / 0.1) + 1 }, (_, i) => { const q = p.at(Math.min(i * 0.1, p.length)); return [r(q.x), r(q.y)]; });
const routes = [];
for (const l of cell.lines) {
  for (const s of cell.sides) { const rt = cell.route(l, s); routes.push(poly(rt.leg), poly(rt.link)); }
  routes.push(poly(cell.dock(l)));
}
for (const s of [...cell.slots, cell.stack]) routes.push(poly(cell.dock(s)));

// frames: [x, y, θ, steer, drive wheel, left wheel, right wheel, tilt, lift, shift]
const frames = [], pallets = new Map(), sub = Math.round(1 / (FPS * DT)), N = SECONDS * FPS;
for (let f = 0; f <= N; f++) {
  frames.push([a.pose.x, a.pose.y, a.pose.th, a.phi, a.wheel.drive, a.wheel.left, a.wheel.right, a.tilt, a.lift, a.shift].map(r));
  for (const p of cell.pallets) {
    if (!pallets.has(p.id)) pallets.set(p.id, { id: p.id, kind: p.kind, sku: p.skuIndex ?? -1, boxes: p.boxes.map((b) => [b.sx, b.sy, b.sz, r(b.lx), r(b.ly), r(b.lz)]), first: f, track: [] });
    pallets.get(p.id).track.push([p.x, p.y, p.z, p.yaw, p.pitch].map(r)); // one entry per frame from `first` until it is collected
  }
  for (let i = 0; i < sub; i++) cell.step(DT);
}

const out = fileURLToPath(new URL('../blender/amr_params.json', import.meta.url));
const { AMR, MAST, FORK, PALLET, SKUS, CONV, DOCK, CELL } = cfg;
const stations = [...cell.lines, ...cell.slots, cell.stack, cell.home].map((s) => ({ id: s.id, kind: s.kind, x: s.x, y: s.y, th: s.th, pre: stationPoses(s).pre }));
writeFileSync(out, JSON.stringify({ AMR, MAST, FORK, PALLET, SKUS, CONV, DOCK, CELL, stations, routes, fps: FPS, frames, pallets: [...pallets.values()] }));
console.log(`${out}: ${frames.length} frames (${SECONDS} s at ${FPS} fps), ${pallets.size} pallets, ${a.stats.delivered} deliveries`);
