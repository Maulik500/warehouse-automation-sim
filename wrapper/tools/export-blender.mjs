// Writes blender/wrapper_params.json: every parameter from config.js and a sampled trajectory of
// every joint, every pallet, every film band and the free film span, for
// blender/build_wrapper.py.   `npm run export:wrapper`

import { writeFileSync, mkdirSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import * as cfg from '../js/config.js';
import * as K from '../js/kinematics.js';
import { Cell } from '../js/sim.js';

const SECONDS = 215, FPS = 24, DT = 1 / 240;
const cell = new Cell({ seed: 7 }), w = cell.w;
const r = (v) => +v.toFixed(4);
const { FILM, CLAMP } = cfg, h = FILM.w / 2;
const GRIP = [K.Z_BOTTOM - h, K.Z_BOTTOM - h + CLAMP.gripH]; // band of web in the jaws

// frames: [q1 total, q3, clamp g1, jaw g2, slide g3, dancer q5, film paid out q4, roll angle, roll radius, wire, roller travel of the 7 zones …]
const frames = [], span = [], pallets = new Map(), sub = Math.round(1 / (FPS * DT)), N = SECONDS * FPS;
// one film band: [first frame, last frame (−1 = stays), ax, ay, az0, az1, bx, by, bz0, bz1] in {P}, absolute heights
const band = (f, a, b, held) => [f, -1, a.x, a.y, held ? GRIP[0] : a.z - h, held ? GRIP[1] : a.z + h, b.x, b.y, b.z - h, b.z + h].map(r);
for (let f = 0; f <= N; f++) {
  frames.push([w.thBase + w.th, w.lift, w.clampUp, w.jaw, w.slide, w.dancer, w.feed, w.rollAngle, cell.rollRadius, w.wire ? 1 : 0, ...cell.roll].map(r));
  // free film: exit roller → active anchor, or → clamp while only the clamp holds it
  if (w.filmOnLoad) {
    const film = w.pallet.film, a = film.active, C = film.world(a, w.th), l = K.filmLine(C), held = film.corners === 0;
    span.push([...l.T, ...C, held ? GRIP[0] : a.z - h, held ? GRIP[1] : a.z + h].map(r));
  } else if (w.supply) {
    const C = K.rot(w.th, [CLAMP.x, CLAMP.y]);
    span.push([...K.filmLine(C).T, ...C, ...GRIP].map(r));
  } else span.push(null);
  for (const p of cell.pallets) {
    if (!pallets.has(p.id)) pallets.set(p.id, { id: p.id, sku: p.skuIndex, boxes: p.boxes.map((b) => [b.sx, b.sy, b.sz, r(b.lx), r(b.ly), r(b.lz)]), first: f, track: [], bands: [], n: 1, tail: true });
    const rec = pallets.get(p.id), film = p.film;
    rec.track.push([p.wx, p.wy, p.yaw].map(r)); // one entry per frame from `first` until it leaves the line
    if (!film) continue;
    for (; rec.n < film.anchors.length; rec.n++) rec.bands.push(band(f, film.anchors[rec.n - 1], film.anchors[rec.n], rec.n === 1 && film.tailHeld));
    if (rec.tail && !film.tailHeld && rec.bands.length) { // the clamp let go: the tail now lies on the face
      rec.tail = false;
      rec.bands[0][1] = f;
      rec.bands.push(band(f, film.anchors[0], film.anchors[1], false));
    }
  }
  for (let i = 0; i < sub; i++) cell.step(DT);
}

const dir = fileURLToPath(new URL('../blender/', import.meta.url));
mkdirSync(dir, { recursive: true });
const { PALLET, SKUS, LINE, TABLE, CARRIAGE, WRAP, CUT } = cfg;
writeFileSync(dir + 'wrapper_params.json', JSON.stringify({ PALLET, SKUS, LINE, TABLE, CARRIAGE, FILM, WRAP, CLAMP, CUT, zBottom: K.Z_BOTTOM, fps: FPS, frames, span, pallets: [...pallets.values()].map(({ n, tail, ...p }) => p) }));
console.log(`${dir}wrapper_params.json: ${frames.length} frames (${SECONDS} s at ${FPS} fps), ${pallets.size} pallets, ${cell.stats.wrapped} wrapped`);
