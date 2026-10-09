// Writes blender/asrs_params.json: every parameter from config.js, the stored pallets with
// their boxes, and a sampled joint trajectory per crane, for blender/build_asrs.py.
// `npm run export:asrs`

import { writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import * as cfg from '../js/config.js';
import { Warehouse } from '../js/sim.js';

const SECONDS = 150, FPS = 24, DT = 1 / 240;
const wh = new Warehouse({ seed: 7 });
const r = (v) => +v.toFixed(4);
const stored = [...wh.pallets].map((p) => ({ id: p.id, x: r(p.x), y: r(p.y), z: r(p.z), sku: p.skuIndex, boxes: p.boxes.map((b) => [b.sx, b.sy, b.sz, r(b.lx), r(b.ly), r(b.lz)]) }));
const frames = wh.aisles.map(() => []);
const sub = Math.round(1 / (FPS * DT));
for (let f = 0; f <= SECONDS * FPS; f++) {
  wh.aisles.forEach((a, i) => frames[i].push([...a.q.map(r), r(a.q[1] - a.forkPlane())])); // q1, q2, q3 (aisle-local), tip sag
  for (let i = 0; i < sub; i++) wh.step(DT);
}

const out = fileURLToPath(new URL('../blender/asrs_params.json', import.meta.url));
const { CRANE, FORK, LOAD, RACK, PD, LAYOUT, SKUS } = cfg;
writeFileSync(out, JSON.stringify({ CRANE, FORK, LOAD, RACK, PD, LAYOUT, SKUS, fps: FPS, frames, stored }));
console.log(`${out}: ${frames.length} cranes x ${frames[0].length} frames (${SECONDS} s at ${FPS} fps), ${stored.length} stored pallets, ${stored.reduce((n, p) => n + p.boxes.length, 0)} boxes`);
