// What the 3D view needs, as plain data: snapshot(world) from the live simulation, and a compact
// recording of many snapshots (Recorder in Node, Player in the browser) for the presentation,
// the offline file and the video. Machine channels are dense; pallets and boxes are sparse
// tracks with a key only where linear interpolation would be off by more than 2 mm / 0.3°.

import { ROWS, PALLET, PLACE } from './config.js';
import { palletG } from './asrs.js';
import { LINE } from '../../wrapper/js/config.js';

const { abs, round, floor, min, max, PI, atan2, sin, cos } = Math;
const LAMP = { off: 0x343a44, green: 0x3ecf8e, blue: 0x4aa3ff, yellow: 0xf0b429 };
const lampIx = (c) => Math.max(0, [LAMP.off, LAMP.green, LAMP.blue, LAMP.yellow].indexOf(c));
const lampOf = (i) => [LAMP.off, LAMP.green, LAMP.blue, LAMP.yellow][round(i)] ?? LAMP.off;
const WORK = new Set(['SETTLE', 'LOAD', 'STOW', 'DOCK_SRC', 'UNDOCK_SRC', 'DOCK_DST', 'STOP_DOWN', 'TRANSFER', 'STOP_UP', 'UNDOCK_DST']);

// ---------- live ----------
let rackCache = { tick: -99, rack: null, key: 0 };
function rackOf(w) {
  if (w.tickNo - rackCache.tick < 24 && rackCache.rack) return rackCache;
  const out = [], A = PLACE.asrs;
  for (const a of w.asrs.aisles) for (const s of a.slots) { const p = s.pallet; if (p && !p.boxes) out.push([A.tx + s.x, A.ty + s.y, s.zs, p.layers * p.skuDef.h, p.sku]); }
  rackCache = { tick: w.tickNo, rack: out, key: out.length * 7919 + out.reduce((h, r) => (h * 31 + r[0] * 3 + r[1] * 5 + r[3] * 100) % 1e9, 0) };
  return rackCache;
}
export function filmTop(p) {
  if (p.wrapped) return p.z + p.h + 0.04;
  const w = p.wrap;
  if (w && w.film) return min(p.z + p.h + 0.04, w.film.active.z + 0.225 - LINE.z + p.z);
  return 0;
}
export function snapshot(w, { claims = false } = {}) {
  const r = rackOf(w), A = w.asrs;
  const s = {
    t: w.t, rack: r.rack, rackKey: r.key,
    cranes: A.aisles.map((a) => [...a.q]),
    lanes: ROWS.map((row) => { const l = A.lanes[row]; return { stop: l.stop, lamp: l.locked ? LAMP.blue : l.ready ? LAMP.green : l.state === 'EMPTY' ? LAMP.yellow : LAMP.off }; }),
    cells: w.cells.map((c) => ({ q: [...c.q], held: !!c.held, gate: c.infeed.flight ? 1 : 0, lamp: c.locked ? LAMP.blue : c.palState === 'D_FULL' ? LAMP.green : c.palState === 'BUILD' ? LAMP.yellow : LAMP.off })),
    robots: w.robots.map((m) => ({ x: m.pose.x, y: m.pose.y, th: m.pose.th, wl: m.wheel.L, wr: m.wheel.R, q: [...m.q], stop: m.deck.stop, vac: m.vac, held: !!m.held, wait: m.drive.waitT > 0.5, work: WORK.has(m.phase) })),
    forks: w.forklifts.map((f) => ({ x: f.pose.x, y: f.pose.y, th: f.pose.th, phi: f.phi, wd: f.wheel.drive, wl: f.wheel.left, wr: f.wheel.right, tilt: f.tilt, lift: f.lift, shift: f.shift, wait: f.drive.waitT > 0.5, moving: abs(f.v) > 1e-3 })),
    wrap: (() => {
      const x = w.wrap.w, P = PLACE.wrap;
      let film = null;
      if (x.filmOnLoad) { const f = x.pallet.film, ln = f.line(x.th), c = f.world(f.active, x.th); film = [P.tx + ln.T[0], P.ty + ln.T[1], x.lift, P.tx + c[0], P.ty + c[1], f.active.z]; }
      return { th: x.thBase + x.th, clampUp: x.clampUp, jaw: x.jaw, lift: x.lift, slide: x.slide, film };
    })(),
    pallets: [], boxes: [],
  };
  const seen = new Set();
  for (const p of [...w.loose(), ...A.active()]) {
    if (seen.has(p)) continue;
    seen.add(p);
    const g = palletG(p);
    s.pallets.push({ id: p.id, x: g[0], y: g[1], z: g[2], yaw: p.yaw, film: filmTop(p) });
  }
  for (const b of A.boxes) s.boxes.push({ id: b.id, sku: b.sku, x: b.p[0], y: b.p[1], z: b.p[2], yaw: b.yaw, sx: b.sx, sy: b.sy, h: b.h });
  if (claims) {
    const pts = [];
    for (const list of w.traffic.claims.values()) for (const c of list) for (let k = 0; k < c.poly.length; k++) { const a = c.poly[k], b = c.poly[(k + 1) % c.poly.length]; pts.push(a[0], a[1], 0.03, b[0], b[1], 0.03); }
    s.claims = pts;
  }
  return s;
}

// ---------- channels: what is recorded densely, with its quantum ----------
// kind 'a' = angle (interpolated the short way round), 'v' = value
function channels(nRobots, nForks) {
  const C = [];
  const ch = (name, q, kind = 'v') => C.push({ name, q, kind });
  for (let i = 0; i < 3; i++) for (let k = 0; k < 3; k++) ch(`cr${i}.${k}`, 1e-4);
  for (let i = 0; i < 4; i++) { for (let k = 0; k < 6; k++) ch(`ce${i}.q${k}`, 1e-5); ch(`ce${i}.held`, 1); ch(`ce${i}.gate`, 1); ch(`ce${i}.lamp`, 1); }
  for (let i = 0; i < nRobots; i++) { for (const n of ['x', 'y']) ch(`ro${i}.${n}`, 1e-4); ch(`ro${i}.th`, 1e-5, 'a'); ch(`ro${i}.wl`, 1e-3); ch(`ro${i}.wr`, 1e-3); for (let k = 0; k < 6; k++) ch(`ro${i}.q${k}`, 1e-5); for (const n of ['stop', 'vac', 'held', 'wait', 'work']) ch(`ro${i}.${n}`, n === 'stop' ? 1e-3 : 1); }
  for (let i = 0; i < nForks; i++) { for (const n of ['x', 'y']) ch(`fo${i}.${n}`, 1e-4); ch(`fo${i}.th`, 1e-5, 'a'); for (const n of ['phi', 'wd', 'wl', 'wr', 'tilt']) ch(`fo${i}.${n}`, 1e-4); for (const n of ['lift', 'shift']) ch(`fo${i}.${n}`, 1e-4); ch(`fo${i}.wait`, 1); ch(`fo${i}.moving`, 1); }
  for (const n of ['th', 'clampUp', 'jaw', 'lift', 'slide']) ch(`wr.${n}`, 1e-4);
  ch('wr.filmOn', 1); for (let k = 0; k < 6; k++) ch(`wr.f${k}`, 1e-4);
  for (let i = 0; i < 5; i++) { ch(`la${i}.stop`, 1e-3); ch(`la${i}.lamp`, 1); }
  return C;
}
function flatten(s) {
  const v = [];
  for (const q of s.cranes) v.push(...q);
  for (const c of s.cells) v.push(...c.q, +c.held, c.gate, lampIx(c.lamp));
  for (const r of s.robots) v.push(r.x, r.y, r.th, r.wl, r.wr, ...r.q, r.stop, +r.vac, +r.held, +r.wait, +r.work);
  for (const f of s.forks) v.push(f.x, f.y, f.th, f.phi, f.wd, f.wl, f.wr, f.tilt, f.lift, f.shift, +f.wait, +f.moving);
  const w = s.wrap;
  v.push(w.th, w.clampUp, w.jaw, w.lift, w.slide, w.film ? 1 : 0, ...(w.film ?? [0, 0, 0, 0, 0, 0]));
  for (const l of s.lanes) v.push(l.stop, lampIx(l.lamp));
  return v;
}

// ---------- recorder (Node) ----------
export class Recorder {
  constructor({ fps = 10, nRobots, nForks, meta = {} }) {
    this.fps = fps; this.C = channels(nRobots, nForks); this.nR = nRobots; this.nF = nForks; this.meta = meta;
    this.cols = this.C.map(() => []); this.frames = 0;
    this.tracks = { pallets: new Map(), boxes: new Map() };
    this.rack = { first: null, events: [], prev: null };
    this.kpi = []; this.events = [];
  }
  // greedy line simplification: keep a key only when interpolation from the last kept key would miss
  track(map, id, f, pose, make) {
    let e = map.get(id);
    if (!e) { e = { ...make(), first: f, last: f, spans: [[f, f]], keys: [[f, ...pose]], pend: null }; map.set(id, e); return; }
    if (e.last !== f - 1) { // reappeared after a gap (e.g. a pallet that was back in the rack): a new visible span
      if (e.pend) { e.keys.push(e.pend); e.pend = null; }
      e.keys.push([f, ...pose]); e.last = f; e.spans.push([f, f]); return;
    }
    e.last = f; e.spans[e.spans.length - 1][1] = f;
    const k0 = e.keys[e.keys.length - 1], cand = [f, ...pose];
    if (!e.pend) { e.pend = cand; return; }
    // would k0 -> cand reproduce pend?
    const p = e.pend, u = (p[0] - k0[0]) / (f - k0[0]);
    let ok = true;
    for (let i = 1; i <= 3 && ok; i++) if (abs(k0[i] + (cand[i] - k0[i]) * u - p[i]) > 0.002) ok = false;
    const da = (a, b) => abs(atan2(sin(a - b), cos(a - b)));
    if (ok && da(k0[4] + atan2(sin(cand[4] - k0[4]), cos(cand[4] - k0[4])) * u, p[4]) > 0.005) ok = false;
    if (!ok) e.keys.push(p);
    e.pend = cand;
  }
  add(s, extra = {}) {
    const f = this.frames++, v = flatten(s);
    v.forEach((x, i) => this.cols[i].push(round(x / this.C[i].q)));
    for (const p of s.pallets) this.track(this.tracks.pallets, p.id, f, [p.x, p.y, p.z, p.yaw], () => ({ id: p.id, film: [] }));
    for (const p of s.pallets) { const e = this.tracks.pallets.get(p.id), fz = round(p.film * 1000); if (!e.film.length || e.film[e.film.length - 1][1] !== fz) e.film.push([f, fz]); }
    for (const b of s.boxes) this.track(this.tracks.boxes, b.id, f, [b.x, b.y, b.z, b.yaw], () => ({ id: b.id, sku: b.sku, size: [b.sx, b.sy, b.h] }));
    const key = s.rackKey;
    if (this.rack.prev !== key) {
      const list = s.rack.map((r) => [round(r[0] * 1000), round(r[1] * 1000), round(r[2] * 1000), round(r[3] * 1000), r[4]]);
      if (!this.rack.first) this.rack.first = list; else this.rack.events.push([f, list]);
      this.rack.prev = key;
    }
    if (f % this.fps === 0) this.kpi.push(extra.kpi ?? null);
  }
  event(e) { this.events.push({ f: this.frames, ...e }); }
  // plain JSON; machine channels as delta-coded integers (they compress to almost nothing)
  toJSON() {
    const fin = (map) => [...map.values()].map((e) => { if (e.pend) e.keys.push(e.pend); const { pend, ...o } = e; o.keys = o.keys.map((k) => [k[0], round(k[1] * 1000), round(k[2] * 1000), round(k[3] * 1000), round(k[4] * 1e4)]); return o; });
    return {
      v: 1, fps: this.fps, frames: this.frames, nRobots: this.nR, nForks: this.nF, meta: this.meta,
      channels: this.C.map((c, i) => ({ ...c, d: this.cols[i].map((x, k) => (k ? x - this.cols[i][k - 1] : x)) })),
      pallets: fin(this.tracks.pallets), boxes: fin(this.tracks.boxes), rack: this.rack.first, rackEvents: this.rack.events, kpi: this.kpi, events: this.events,
    };
  }
}

// ---------- player (browser) ----------
const lerpA = (a, b, u) => a + atan2(sin(b - a), cos(b - a)) * u;
export class Player {
  constructor(rec) {
    this.r = rec; this.fps = rec.fps; this.T = (rec.frames - 1) / rec.fps;
    this.cols = rec.channels.map((c) => { const out = new Float64Array(c.d.length); let x = 0; c.d.forEach((d, k) => { x += d; out[k] = x * c.q; }); return out; });
    this.ang = rec.channels.map((c) => c.kind === 'a');
    const prep = (list) => list.map((e) => ({ ...e, keys: e.keys.map((k) => [k[0], k[1] / 1000, k[2] / 1000, k[3] / 1000, k[4] / 1e4]) }));
    this.pallets = prep(rec.pallets); this.boxes = prep(rec.boxes);
    this.rackAt = [[0, rec.rack.map((r) => [r[0] / 1000, r[1] / 1000, r[2] / 1000, r[3] / 1000, r[4]])], ...rec.rackEvents.map(([f, l]) => [f, l.map((r) => [r[0] / 1000, r[1] / 1000, r[2] / 1000, r[3] / 1000, r[4]])])];
    this.events = rec.events; this.kpi = rec.kpi;
  }
  static async fromGzipBase64(b64) {
    const bin = Uint8Array.from(atob(b64), (c) => c.charCodeAt(0));
    const text = await new Response(new Blob([bin]).stream().pipeThrough(new DecompressionStream('gzip'))).text();
    return new Player(JSON.parse(text));
  }
  static async fromUrl(url) {
    const res = await fetch(url);
    const text = url.endsWith('.gz') ? await new Response(res.body.pipeThrough(new DecompressionStream('gzip'))).text() : await res.text();
    return new Player(JSON.parse(text));
  }
  pose(e, fr) {
    const K = e.keys;
    if (!e.spans.some(([a, b]) => fr >= a && fr <= b + 0.999)) return null;
    let lo = 0, hi = K.length - 1;
    if (fr <= K[0][0]) return K[0];
    if (fr >= K[hi][0]) return K[hi];
    while (hi - lo > 1) { const m = (lo + hi) >> 1; if (K[m][0] <= fr) lo = m; else hi = m; }
    const a = K[lo], b = K[hi], u = (fr - a[0]) / (b[0] - a[0]);
    return [0, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u, a[3] + (b[3] - a[3]) * u, lerpA(a[4], b[4], u)];
  }
  // state at time t (s, from the start of the recording)
  at(t) {
    const fr = max(0, min(this.r.frames - 1, t * this.fps)), i0 = floor(fr), i1 = min(this.r.frames - 1, i0 + 1), u = fr - i0;
    const val = (c) => { const a = this.cols[c][i0], b = this.cols[c][i1]; return this.ang[c] ? lerpA(a, b, u) : a + (b - a) * u; };
    const near = (c) => this.cols[c][u < 0.5 ? i0 : i1];
    let c = 0;
    const take = (n, f = val) => Array.from({ length: n }, () => f(c++));
    const s = { t: this.r.meta.t0 + t, cranes: [], cells: [], robots: [], forks: [], lanes: [], pallets: [], boxes: [] };
    for (let i = 0; i < 3; i++) s.cranes.push(take(3));
    for (let i = 0; i < 4; i++) { const q = take(6); const [held, gate, lamp] = take(3, near); s.cells.push({ q, held: held > 0.5, gate, lamp: lampOf(lamp) }); }
    for (let i = 0; i < this.r.nRobots; i++) {
      const [x, y, th, wl, wr] = take(5), q = take(6), stop = val(c++), [vac, held, wait, work] = take(4, near);
      s.robots.push({ x, y, th, wl, wr, q, stop, vac: vac > 0.5, held: held > 0.5, wait: wait > 0.5, work: work > 0.5 });
    }
    for (let i = 0; i < this.r.nForks; i++) { const [x, y, th, phi, wd, wl, wr, tilt, lift, shift] = take(10), [wait, moving] = take(2, near); s.forks.push({ x, y, th, phi, wd, wl, wr, tilt, lift, shift, wait: wait > 0.5, moving: moving > 0.5 }); }
    const [th, clampUp, jaw, lift, slide] = take(5), filmOn = near(c++), film = take(6);
    s.wrap = { th, clampUp, jaw, lift, slide, film: filmOn > 0.5 ? film : null };
    for (let i = 0; i < 5; i++) { const stop = val(c++), lamp = near(c++); s.lanes.push({ stop, lamp: lampOf(lamp) }); }
    for (const e of this.pallets) {
      const p = this.pose(e, fr);
      if (!p) continue;
      let film = 0;
      for (const [f, z] of e.film) { if (f > fr) break; film = z / 1000; }
      s.pallets.push({ id: e.id, x: p[1], y: p[2], z: p[3], yaw: p[4], film });
    }
    for (const e of this.boxes) { const p = this.pose(e, fr); if (p) s.boxes.push({ id: e.id, sku: e.sku, x: p[1], y: p[2], z: p[3], yaw: p[4], sx: e.size[0], sy: e.size[1], h: e.size[2] }); }
    let rk = 0;
    for (let k = 0; k < this.rackAt.length && this.rackAt[k][0] <= fr; k++) rk = k;
    s.rack = this.rackAt[rk][1]; s.rackKey = rk;
    s.kpi = this.kpi[min(this.kpi.length - 1, floor(fr / this.fps))];
    return s;
  }
}
export { PALLET };
