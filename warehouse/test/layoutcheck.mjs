// Static layout checks (spec §15, "Clearance"): where a vehicle may stand must be clear of where others sweep.
import { World } from '../js/sim.js';
import { RING } from '../js/roadmap.js';
import { SHAPE, rect, overlap } from '../js/traffic.js';
export function layoutConflicts(w) {
  const ring = [];
  for (let s = 0; s < RING.length; s += 0.2) ring.push(rect(RING.at(s), SHAPE.fork.claim));
  const all = [...w.stations, ...w.parking], bad = [];
  for (const a of all) {
    const pre = rect(a.spur.pre, SHAPE.fork.claim), dock = rect(a.spur.dock, SHAPE.fork.claim);
    if (ring.some((q) => overlap(q, pre))) bad.push(`${a.id}: pre-dock inside the ring sweep`);
    for (const b of all) {
      if (a === b || Math.hypot(a.x - b.x, a.y - b.y) < 2.0) continue; // neighbours are excluded by the dispatcher
      if (b.spur.sweep.some((q) => overlap(q, pre) || overlap(q, dock))) bad.push(`${a.id}: standing vehicle inside the entry sweep of ${b.id}`);
    }
  }
  return bad;
}
if (process.argv[1].endsWith('layoutcheck.mjs')) { const b = layoutConflicts(new World({ forklifts: 1, robots: 1 })); console.log(b.length ? b.join('; ') : 'layout: no conflicts'); }
