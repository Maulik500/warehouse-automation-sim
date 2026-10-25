// The guided tour (storyboard) is not hand-timed: every step is found in the recorded event log,
// so each caption shows the machine actually doing what the caption says, at that moment of the
// run. A step: { n, caption, s0 (sim time, s from the start of the recording), dur (video s),
// speed (sim s per video s), cam: (state, u) => { eye, tgt } in {G}, labels: [[anchor, html]],
// inset: what the engineering panel shows }.

const lerp3 = (a, b, u) => a.map((v, i) => v + (b[i] - v) * u);
const ease = (u) => (u <= 0 ? 0 : u >= 1 ? 1 : u * u * (3 - 2 * u));

export function buildTour(player, view) {
  const E = player.events, fps = player.fps, T = player.T;
  const time = (e) => e.f / fps;
  // first event matching `pred` after time `after` (seconds); null if none
  const find = (pred, after = 0) => E.find((e) => time(e) >= after && pred(e)) ?? null;
  const at = (name) => (s) => view.anchor(name, s);
  const steps = [];
  const add = (st) => { if (st && st.s0 !== undefined && st.s0 >= 0 && st.s0 < T) steps.push(st); };
  // orbit-style camera around a moving or fixed anchor
  const around = (anchorFn, off0, off1 = off0, lift = 0) => (s, u) => {
    const a = anchorFn(s) ?? [34, 24, 0], o = lerp3(off0, off1, ease(u));
    return { eye: [a[0] + o[0], a[1] + o[1], a[2] + o[2]], tgt: [a[0], a[1], a[2] - 1 + lift] };
  };
  const fixed = (eye0, tgt0, eye1 = eye0, tgt1 = tgt0) => (s, u) => ({ eye: lerp3(eye0, eye1, ease(u)), tgt: lerp3(tgt0, tgt1, ease(u)) });

  // 1. the whole building
  add({ caption: 'Integrated warehouse: ASRS, robot AMRs, four palletizing cells, belly wrapper and a forklift AMR fleet', s0: 2, dur: 11, speed: 1,
    cam: fixed([10, -24, 38], [34, 22, 0], [58, -18, 36], [32, 24, 0]),
    labels: [['port:C', '<b>ASRS</b> rows A–E · 3 stacker cranes'], ['cell:1', '<b>Palletizing</b> · 4 robot cells'], ['wrapper', '<b>Belly wrapper</b>'], ['storage', '<b>Storage</b>'], ['empties', 'Empty pallets'], ['receiving', 'Receiving']] });

  // 2. a crane fetches the pallet a robot AMR will need
  // the first robot-AMR trip whose pallet a crane fetches from the rack just before (so the story starts in the rack)
  const AISLE = { A: 0, B: 0, C: 1, D: 1, E: 2 };
  const trips = E.filter((e) => /^M\d$/.test(e.who) && e.what === 'TO_SRC' && time(e) > 5);
  const craneFor = (t) => E.find((e) => e.who === `CR${AISLE[t.row] + 1}` && e.what === `retrieve ${t.row}` && time(e) > time(t) - 90 && time(e) < time(t) + 60);
  const robotTrip = trips.find(craneFor) ?? trips[0];
  if (robotTrip) {
    const row = robotTrip.row, aisle = AISLE[row], crane = craneFor(robotTrip);
    if (crane) add({ caption: `Stacker crane ${aisle + 1} retrieves an SKU pallet from row ${row} and puts it on the row's lane`, s0: time(crane), dur: 12, speed: 2.5,
      cam: around(at(`crane:${aisle}`), [-7.0, 0.0, 1.4], [-5.0, 0.0, 0.9], 0.4), labels: [[`crane:${aisle}`, `<b>Crane ${aisle + 1}</b> · travel · hoist · telescopic fork`], [`port:${row}`, `<b>Port ${row}</b> pick position`]], inset: { kind: 'crane', i: aisle } });
    const dock = find((e) => e.who === robotTrip.who && e.what === 'DOCK_SRC', time(robotTrip));
    if (dock) add({ caption: `Robot AMR ${robotTrip.who} docks beside port ${row}: the pallet is locked, the 3D camera measures it`, s0: time(dock) - 3, dur: 9, speed: 1,
      cam: around(at(`port:${row}`), [-5.5, -5, 3.4], [-4.5, -4, 2.8]), labels: [[`robot:${robotTrip.who}`, `<b>${robotTrip.who}</b> · AMR + UR20-class arm`], [`port:${row}`, '<b>Locked</b> · pallet will not move']] });
    const load = find((e) => e.who === robotTrip.who && e.what === 'LOAD', time(robotTrip));
    if (load) add({ caption: 'The 6-axis arm takes one full layer, top layer only, tool always vertical, singularity margins monitored', s0: time(load) + 2, dur: 14, speed: 1,
      cam: around(at(`arm:${robotTrip.who}`), [-2.6, -3.2, 1.6], [-3.2, -2.4, 1.4], 0.6), labels: [[`arm:${robotTrip.who}`, 'Vacuum pad · 6 cups']], inset: { kind: 'mobile', who: robotTrip.who } });
    const trav = find((e) => e.who === robotTrip.who && e.what === 'TRAVEL', time(robotTrip));
    if (trav) add({ caption: `Loaded, ${robotTrip.who} drives to palletizer ${(trav.cell ?? 0) + 1} on its own one-way loop, continuous-curvature turns only`, s0: time(trav), dur: 12, speed: 4,
      cam: around(at(`robot:${robotTrip.who}`), [-5, 5, 3.6], [-4.5, -5, 3.6]), labels: [[`robot:${robotTrip.who}`, `<b>${robotTrip.who}</b> · loaded · ≤ 1.0 m/s`]], inset: { kind: 'mobile', who: robotTrip.who } });
    const tr = find((e) => e.who === robotTrip.who && e.what === 'TRANSFER', time(robotTrip));
    if (tr) add({ caption: 'Nose-on dock: the deck rollers hand the whole layer to the receiving conveyor', s0: time(tr) - 1, dur: 9, speed: 1,
      cam: around(at(`infeed:${tr.cell ?? trav?.cell ?? 0}`), [-4.5, -4.5, 3.2], [-3.5, -5.5, 2.6]), labels: [[`robot:${robotTrip.who}`, `<b>${robotTrip.who}</b> · deck transfer`], [`infeed:${tr.cell ?? 0}`, 'Row stop · one row at a time']] });
    const ci = tr?.cell ?? trav?.cell ?? 0;
    const pick = find((e) => e.who === `PAL${ci + 1}.robot` && e.what === 'PICK', tr ? time(tr) + 10 : 0);
    if (pick) add({ caption: `Single file at 0.25 m/s: the palletizing robot tracks the moving box, picks it and builds the layer`, s0: time(pick) - 1, dur: 16, speed: 1,
      cam: around(at(`tcp:${ci}`), [5.2, -3.6, 2.6], [4.0, 3.6, 2.8], 0.4), labels: [[`cell:${ci}`, `<b>Palletizer ${ci + 1}</b> · KR 120 R2700 class`], [`tcp:${ci}`, 'TCP']], inset: { kind: 'cell', i: ci } });
  }
  // 3. a finished pallet leaves a cell (the cell the layer above went to, if it finishes in the run)
  const last = steps.length ? steps[steps.length - 1] : null, after = last ? last.s0 + last.dur * last.speed : 0, cellOf = robotTrip?.cell;
  const full = find((e) => e.who === `PAL${(cellOf ?? 0) + 1}` && e.what === 'D_FULL', after) ?? find((e) => /^PAL\d$/.test(e.who) && e.what === 'D_FULL', after) ?? find((e) => /^PAL\d$/.test(e.who) && e.what === 'D_FULL', 0);
  let fk = null;
  if (full) {
    const ci = +full.who.slice(3) - 1;
    fk = find((e) => /^F\d$/.test(e.who) && e.kind === 'wrap' && e.pick === `D${ci + 1}` && /insert forks/.test(e.what), time(full));
    add({ caption: `Mixed pallet complete — one SKU per layer, heaviest at the bottom — it indexes to position D`, s0: time(full) - 16, dur: 9, speed: 2,
      cam: around(at(`D:${ci}`), [-5, 5.5, 4], [-4, 6, 3.5]), labels: [[`D:${ci}`, `<b>D</b> · forklift hand-over`]] });
    if (fk) add({ caption: `Forklift AMR ${fk.who} picks it: creep 0.3 m/s, forks level, steer straight, side shift centres the tines`, s0: time(fk) - 3, dur: 11, speed: 1.5,
      cam: around(at(`fork:${fk.who}`), [5, 5, 3.4], [4, 6, 3]), labels: [[`fork:${fk.who}`, `<b>${fk.who}</b> · counterbalanced AMR`]], inset: { kind: 'fork', who: fk.who } });
  }
  // 4. wrapping
  const wrap = find((e) => e.who === 'WRAP' && e.what === 'WRAP', fk ? time(fk) : full ? time(full) : 0) ?? find((e) => e.who === 'WRAP' && e.what === 'WRAP', 0);
  if (wrap) add({ caption: 'Belly wrapper: the load turns at 12 rpm while the carriage spirals up and down; clamp, cut and press are automatic', s0: time(wrap) + 1, dur: 14, speed: 3,
    cam: fixed([24.0, 39.2, 3.6], [20, 34.4, 1.0], [16.0, 39.4, 3.4], [20, 34.4, 1.0]), labels: [['wrapper', '<b>Turntable</b> · film carriage'], ['I4', 'In · I4'], ['O4', 'Out · O4']], inset: { kind: 'wrap' } });
  // 5. storage
  const store = find((e) => /^F\d$/.test(e.who) && e.kind === 'store' && /^drive to S/.test(e.what), wrap ? time(wrap) : 0) ?? find((e) => /^F\d$/.test(e.who) && e.kind === 'store' && /^drive to S/.test(e.what));
  if (store) add({ caption: 'Wrapped pallet to floor storage; it is shipped from here', s0: time(store), dur: 11, speed: 3,
    cam: around(at(`fork:${store.who}`), [-5, -5, 3.2], [4, -5.5, 3.2]), labels: [[`fork:${store.who}`, `<b>${store.who}</b> · wrapped pallet`], ['storage', '<b>Storage</b> S01–S12']], inset: { kind: 'fork', who: store.who } });
  // 6. empties and replenishment
  const tStore = store ? time(store) : 0, eo = find((e) => /^F\d$/.test(e.who) && e.kind === 'emptyOut' && /insert forks/.test(e.what), tStore) ?? find((e) => /^F\d$/.test(e.who) && e.kind === 'emptyOut' && /insert forks/.test(e.what), 0);
  if (eo) add({ caption: 'An emptied source pallet leaves the ASRS port for the empty-pallet station — one forklift in the port apron at a time', s0: time(eo) - 2, dur: 10, speed: 1.5,
    cam: around(at(`fork:${eo.who}`), [-6, -5, 4], [-5, -6, 3.5]), labels: [[`fork:${eo.who}`, `<b>${eo.who}</b> · empty pallet`]] });
  const rp = find((e) => /^F\d$/.test(e.who) && e.kind === 'replenish' && /^drive to PORT/.test(e.what), eo ? time(eo) : tStore) ?? find((e) => /^F\d$/.test(e.who) && e.kind === 'replenish' && /^drive to PORT/.test(e.what), 0);
  if (rp) add({ caption: 'Replenishment: a full SKU pallet from receiving to its row port; the crane stores it', s0: time(rp), dur: 10, speed: 4,
    cam: around(at(`fork:${rp.who}`), [-8, -6, 6], [-6, -8, 6]), labels: [[`fork:${rp.who}`, `<b>${rp.who}</b> · replenishment`]] });
  // 7. the whole flow, fast
  add({ caption: 'Every motion above comes from the verified simulation: no scripted paths, no overlaps, 0 safety-monitor violations', s0: Math.max(0, T - 240), dur: 12, speed: 15,
    cam: fixed([60, -16, 40], [32, 24, 0], [6, -20, 40], [32, 24, 0]), labels: [] });

  // chronological numbering of the process steps (the first and last shots are framing shots)
  let v = 7; // after the title card
  steps.forEach((s, i) => { s.n = i === 0 || i === steps.length - 1 ? null : i; s.v0 = v; v += s.dur; s.v1 = v; s.s1 = s.s0 + s.dur * s.speed; });
  return { steps, intro: [0, 7], results: [v, v + 12], outro: [v + 12, v + 18], end: v + 18 };
}
