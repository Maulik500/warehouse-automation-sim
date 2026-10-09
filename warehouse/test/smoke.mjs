import { World, DT } from '../js/sim.js';
const T = +(process.argv[2] ?? 600), w = new World({ forklifts: +(process.argv[3] ?? 3), robots: +(process.argv[4] ?? 2) });
const t0 = Date.now();
let next = 0;
try {
  while (w.t < T) {
    w.step(DT);
    if (w.t >= next) {
      next += +(process.argv[5] ?? 60);
      console.log(`t=${w.t.toFixed(0)} robots: ${w.robots.map((r) => `${r.phase}${r.drive.blockedBy ? '<' + r.drive.blockedBy : ''}`).join(' | ')}  forks: ${w.forklifts.map((f) => `${f.phase}${f.drive.blockedBy ? '<' + f.drive.blockedBy : ''}`).join(' | ')}`);
      console.log(`     cells: ${w.cells.map((c) => `${c.palState}/${c.state}/${c.order ? c.order.placed + 'of' + c.order.slots.length : '-'}`).join(' ')}  wrap ${w.wrap.w.phase} n=${w.wrap.pallets.size}  stored ${w.storage.filter((s) => s.pallet).length} shipped ${w.stats.shipped}`);
    }
  }
} catch (e) { console.log('ERROR at t=' + w.t.toFixed(2), e.stack); }
console.log('wall', ((Date.now() - t0) / 1000).toFixed(1), 's; violations', JSON.stringify(w.violations()), 'minGap', w.stats.minGap.toFixed(3));
for (const c of w.cells) if (c.stats.firstBad.length) console.log(c.stats.firstBad.slice(0, 4));
for (const r of w.robots) if (r.stats.firstBad.length) console.log(r.id, r.stats.firstBad.slice(0, 4));
for (const f of w.forklifts) if (f.stats.firstBad.length) console.log(f.stats.firstBad.slice(0, 4));
for (const a of w.asrs.aisles) if (a.stats.violationLog.length) console.log(JSON.stringify(a.stats.violationLog.slice(0, 2)));
console.log('boxes', JSON.stringify(w.boxCount()), 'jobs', JSON.stringify(w.fleet.stats), 'orders', JSON.stringify(w.wes.stats));
console.log('lanes', Object.values(w.asrs.lanes).map((l) => `${l.row}:${l.state}${l.claim ? '/' + l.claim : ''}${l.reserved ? '/res' : ''}`).join(' '), 'empties', w.empties.map((s) => s.pallets.length).join('/'), 'collected', w.stats.emptiesOut);
console.log('robot trips', w.robots.map((r) => `${r.stats.trips} (${(r.stats.tripT.reduce((a, b) => a + b, 0) / Math.max(1, r.stats.tripT.length)).toFixed(0)} s)`).join(', '), 'fork jobs', w.forklifts.map((f) => f.stats.jobs).join('/'), 'waits', [...w.robots, ...w.forklifts].map((v) => v.drive.maxWait.toFixed(0)).join('/'));
