// Warehouse execution (spec §10.2, §10.3): orders, layer sequencing and robot-AMR jobs (Wes);
// forklift jobs and their reservations (Fleet). Neither knows anything about joints or paths.

import { SKUS, SKU, MIX, WES, ROWS, PORT, FLOOR, PALLET } from './config.js';
import { ringDist } from './roadmap.js';
import { materialise, setMass, PALLET_YAW0 } from './asrs.js';

const { floor } = Math;
export const sortLayers = (ids) => [...ids].sort((a, b) => SKU[b].layerMass - SKU[a].layerMass || SKU[b].l - SKU[a].l);
export const orderHeight = (ids) => ids.reduce((h, id) => h + SKU[id].h, 0);

export class Wes {
  constructor(world) {
    this.world = world;
    this.orders = []; this.nextId = 1; this.auto = true;
    this.cell = world.cells.map(() => null); // per cell: { order, issued, delivered, placed, transit }
    this.committed = Object.fromEntries(ROWS.map((r) => [r, 0])); // layers promised to open orders, per row
    this.stats = { created: 0, done: 0, layers: 0, held: 0 };
    world.on('layerDelivered', ({ job }) => { const c = this.cell[job.cell]; c.delivered++; c.transit = false; });
    world.on('layerPlaced', ({ cell }) => { this.cell[cell].placed++; });
    world.on('palletBuilt', ({ cell, order }) => { const o = this.orders.find((x) => x.id === order.id); o.state = 'built'; o.tBuilt = world.t; this.cell[cell] = null; this.stats.done++; });
  }
  // layers of a row that can still be promised
  free(row) { return this.world.asrs.stock(row) - this.committed[row]; }
  validate(ids) {
    if (ids.length < MIX.minLayers || ids.length > MIX.maxLayers) return `an order has ${MIX.minLayers} … ${MIX.maxLayers} layers`;
    if (ids.some((id) => !SKU[id])) return 'unknown SKU';
    if (orderHeight(ids) > MIX.maxBoxH + 1e-9) return `stack ${orderHeight(ids).toFixed(3)} m exceeds ${MIX.maxBoxH}`;
    return null;
  }
  addOrder(ids, { front = false } = {}) {
    const bad = this.validate(ids);
    if (bad) return { error: bad };
    const o = { id: `O${String(this.nextId++).padStart(3, '0')}`, layers: sortLayers(ids), state: 'open', t: this.world.t, manual: front };
    for (const id of o.layers) this.committed[SKU[id].row]++;
    const first = this.orders.findIndex((x) => x.state === 'open');
    if (front && first >= 0) this.orders.splice(first, 0, o);
    else this.orders.push(o);
    this.stats.created++;
    return o;
  }
  generate() {
    const r = this.world.rand, n = WES.minOrderLayers + floor(r() * (WES.maxOrderLayers - WES.minOrderLayers + 1)), ids = [], need = {};
    for (let k = 0; k < n; k++) {
      const cand = SKUS.filter((S) => this.free(S.row) - (need[S.row] ?? 0) > 0 && orderHeight(ids) + S.h <= MIX.maxBoxH + 1e-9);
      if (!cand.length) break;
      const S = cand[floor(r() * cand.length)];
      ids.push(S.id); need[S.row] = (need[S.row] ?? 0) + 1;
    }
    if (ids.length >= MIX.minLayers) this.addOrder(ids);
  }
  step() {
    const w = this.world;
    if (this.auto && this.orders.filter((o) => o.state === 'open' || o.state === 'active').length < WES.openOrders) this.generate();
    // order -> first idle cell with an empty pallet at B
    for (const cell of w.cells) {
      if (!cell.ready || this.cell[cell.i]) continue;
      const o = this.orders.find((x) => x.state === 'open');
      if (!o) break;
      o.state = 'active'; o.cell = cell.i; o.tStart = w.t;
      this.cell[cell.i] = { order: o, issued: 0, delivered: 0, placed: 0, transit: false };
      cell.startOrder({ id: o.id, layers: o.layers });
    }
    // a pallet waiting at a port for a SKU nobody has on order any more goes back into the rack
    for (const row of ROWS) if (this.committed[row] <= 0) w.asrs.sendBack(row);
    // layer -> robot AMR, in stacking order, oldest order first
    const active = this.cell.filter(Boolean).sort((a, b) => a.order.tStart - b.order.tStart);
    for (const c of active) {
      if (c.issued >= c.order.layers.length || c.transit || c.issued - c.placed >= WES.layersAhead) continue;
      const robot = w.robots.filter((r) => r.idle).sort((a, b) => b.drive.s - a.drive.s)[0];
      if (!robot) break;
      const id = c.order.layers[c.issued], row = SKU[id].row;
      if (!w.asrs.request(row, robot.id)) continue;
      this.committed[row]--;
      robot.assign({ cell: c.order.cell, row, sku: id, order: c.order.id, layer: c.issued });
      c.issued++; c.transit = true; this.stats.layers++;
    }
  }
}

export class Fleet {
  constructor(world) {
    this.world = world; this.jobs = []; this.asrsBusy = false; this.tick = -1; this.nextPid = 1;
    this.stats = { created: {}, done: {} };
    this.pending = Object.fromEntries(ROWS.map((r) => [r, null])); // replenishment pallet on its way, per row
  }
  emptyPallet(x, y, z) {
    return { id: `E${this.nextPid++}`, kind: 'empty', sku: null, boxes: [], layers: 0, mass: PALLET.mass, h: PALLET.h, cgz: PALLET.h / 2, x, y, z, yaw: 0, dom: 'g', holder: null };
  }
  job(kind, pick, drop, pri, extra = {}) {
    const j = { kind, pick, drop, pri, t: this.world.t, assigned: null, ...extra };
    this.jobs.push(j);
    this.stats.created[kind] = (this.stats.created[kind] ?? 0) + 1;
    return j;
  }
  // Two forklifts never work at neighbouring stations at the same time: the one arriving would
  // stop on the ring exactly where the other has to back out (spec §7).
  near(st) {
    const hit = (o) => o && o !== st && o.kind !== 'home' && Math.hypot(o.x - st.x, o.y - st.y) < 2.0;
    return this.jobs.some((j) => hit(j.pick) || hit(j.drop)) || this.world.forklifts.some((f) => hit(f.onSpur));
  }
  scan() {
    const w = this.world, a = w.asrs;
    // 1 wrapped pallet -> storage
    if (w.wrap.O4.pickReady && !w.wrap.O4.reserved) {
      const slot = w.storage.find((s) => !s.pallet && !s.reserved && !this.near(s));
      if (slot) { slot.reserved = true; w.wrap.O4.reserved = true; this.job('store', w.wrap.O4, slot, 1); }
    }
    // 2 mixed pallet -> wrapper
    for (const c of w.cells) {
      if (c.palState === 'D_FULL' && !c.station.resPick && !w.wrap.I4.reserved) { c.station.resPick = true; w.wrap.I4.reserved = true; this.job('wrap', c.station, w.wrap.I4, 2); }
    }
    // 3 empty pallet -> palletizing cell
    for (const c of w.cells) {
      if (!c.needsPallet) continue;
      const st = w.empties.filter((s) => s.pallets.length - s.reservedOut > 0 && !this.near(s)).sort((x, y) => y.pallets.length - x.pallets.length)[0];
      if (!st) break;
      c.reserved = true; st.reservedOut++;
      this.job('empty', st, c.station, 3, { cell: c });
    }
    // 4 empty source pallet -> stack; 5 replenishment -> row port. One forklift in the ASRS apron at a time.
    if (!this.asrsBusy && !this.jobs.some((j) => j.kind === 'emptyOut')) {
      for (const row of ROWS) {
        const lane = a.lanes[row];
        if (lane.state !== 'EMPTY' || lane.reserved) continue;
        const st = w.empties.filter((s) => s.pallets.length + s.reservedIn < s.max && !this.near(s)).sort((x, y) => y.pallets.length - x.pallets.length)[0];
        if (!st) break;
        lane.reserved = true; st.reservedIn++;
        this.job('emptyOut', lane, st, 4, { lane, asrs: true });
        break;
      }
    }
    if (w.opts.receiving) {
      for (const row of ROWS) {
        if (this.pending[row] || a.fill(row) >= PORT.lowFill) continue;
        const slot = w.receiving.find((s) => !s.pallet && !s.reserved);
        if (!slot) break;
        const S = SKUS.find((x) => x.row === row), p = a.makePallet(S, S.srcLayers);
        Object.assign(p, { dom: 'g', x: slot.x, y: slot.y, z: 0, yaw: 0 });
        materialise(p); setMass(p);
        for (const b of p.boxes) a.boxes.add(b);
        slot.accept(p, w.t); a.counts.replenished++;
        this.pending[row] = { slot, pallet: p, job: null };
      }
      for (const row of ROWS) {
        const q = this.pending[row];
        if (!q || q.job || this.near(q.slot) || this.jobs.some((j) => j.kind === 'replenish')) continue;
        const lane = a.lanes[row], fleet = this;
        // the lane is reserved now (it must be free, or the forklift would wait with the pallet on
        // its forks for a job only another forklift could do); the apron only once the pallet is picked
        if (!a.reserveDeposit(row)) continue;
        lane.reserved = true;
        q.job = this.job('replenish', q.slot, lane, 5, { lane, row, beforeDrop() { if (this.go) return true; if (fleet.asrsBusy) return false; fleet.asrsBusy = true; return (this.go = true); } });
        break;
      }
    }
  }
  // Pull model: a waiting forklift asks. It gets the most urgent open job for which it is the
  // nearest available forklift along the ring.
  nextJob(f) {
    const w = this.world;
    if (this.tick !== w.tickNo) { this.tick = w.tickNo; this.scan(); }
    const open = this.jobs.filter((j) => !j.assigned).sort((x, y) => x.pri - y.pri || x.t - y.t);
    const avail = w.forklifts.filter((x) => x.avail && !x.job);
    for (const j of open) {
      let best = null, bd = Infinity;
      for (const x of avail) { const d = ringDist(x.at.spur, j.pick.spur); if (d < bd - 1e-9) { bd = d; best = x; } }
      if (best === f) {
        if (j.asrs && this.asrsBusy) continue; // the apron is in use: this job waits
        if (j.asrs) this.asrsBusy = true;
        j.assigned = f.id;
        if (j.kind === 'emptyOut') j.onPicked = () => { j.lane.reserved = false; };
        if (j.kind === 'store') j.onPicked = () => { w.wrap.O4.reserved = false; };
        if (j.kind === 'wrap') j.onPicked = () => { j.pick.resPick = false; };
        return j;
      }
    }
    return null;
  }
  done(j, p) {
    const w = this.world;
    this.jobs.splice(this.jobs.indexOf(j), 1);
    this.stats.done[j.kind] = (this.stats.done[j.kind] ?? 0) + 1;
    if (j.kind === 'store') { j.drop.reserved = false; p.tStored = w.t; }
    if (j.kind === 'wrap') w.wrap.I4.reserved = false;
    if (j.kind === 'empty') j.cell.reserved = false;
    if (j.kind === 'emptyOut') { this.asrsBusy = false; w.asrs.counts.emptiesOut++; w.asrs.pallets.delete(p); }
    if (j.kind === 'replenish') { this.asrsBusy = false; j.lane.reserved = false; this.pending[j.row] = null; }
    w.emit('jobDone', { job: j, pallet: p });
  }
}
export { FLOOR, PALLET_YAW0 };
