// Engineering view of the integrated warehouse: the live simulation with every number. The 3D
// scene is view.js (shared with the guided presentation, present.html); this file is the HUD,
// the order panel and the fleet controls. Coordinates are those of WAREHOUSE_SPEC.md.

import * as THREE from 'three';
import { ARM } from '../../mobman/js/config.js';
import { SKUS, SKU, MIX, ROWS, PALLET, PCELL, DT, DEG } from './config.js';
import { World } from './sim.js';
import { orderHeight } from './wes.js';
import { createView } from './view.js';
import { snapshot } from './state.js';

const PI = Math.PI;
let world = new World({ seed: 7, forklifts: 3, robots: 2 });
const view = createView({ layout: world, theme: 'dark' });
const { renderer, camera, controls, roads, labels, claimLines, cellView, forkView, mmView, cranes } = view;
function reset(forklifts = 3, robots = 2) {
  world = new World({ seed: 7, forklifts, robots });
  const sel = $('pick'), keep = sel.value;
  sel.innerHTML = [...world.cells.map((c) => c.id), ...world.robots.map((r) => r.id), ...world.forklifts.map((f) => f.id), 'CRANE1', 'CRANE2', 'CRANE3', 'WRAP'].map((id) => `<option>${id}</option>`).join('');
  if ([...sel.options].some((o) => o.value === keep)) sel.value = keep;
  acc = 0;
}
const sync = () => view.apply(snapshot(world, { claims: claimLines.visible }));

// ---------- HUD ----------
const $ = (id) => document.getElementById(id);
const deg = (a) => (a / DEG).toFixed(1) + '°';
const esc = (s) => String(s).replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' })[c]);
const rows = (id, list) => { $(id).innerHTML = list.map((r) => `<tr>${r.map((c, k) => `<td class="${k === 0 ? 'k' : k === r.length - 1 ? 'r' : ''}">${c}</td>`).join('')}</tr>`).join(''); };
const hms = (t) => { const s = Math.floor(t); return `${String(Math.floor(s / 3600)).padStart(2, '0')}:${String(Math.floor(s / 60) % 60).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`; };
const bar = (name, v, lim, full, unit = '') => `<div class="m"><span>${name}</span><div class="bar${v < lim ? ' bad' : v < 1.6 * lim ? ' warn' : ''}"><i style="width:${Math.min(100, (100 * v) / full)}%"></i></div><span>${v.toFixed(3)}${unit} ≥ ${lim.toFixed(2)}</span></div>`;
const kv = (k, v) => `<div class="row"><span>${k}</span><span>${v}</span></div>`;
function detail() {
  const id = $('pick').value, w = world;
  const c = w.cells.find((x) => x.id === id), r = w.robots.find((x) => x.id === id), f = w.forklifts.find((x) => x.id === id);
  if (c) {
    const s = c.last.sing;
    return kv('Robot state', c.state) + kv('Joints q1…q6', c.q.map((q) => (q / DEG).toFixed(0)).join(' ') + '°') +
      bar('Wrist |sin q5|', Math.abs(s.wrist), PCELL.minWrist, 1) + bar('Elbow |sin(q3+β)|', Math.abs(s.elbow), PCELL.minElbow, 1) + bar('Shoulder ρ', Math.abs(s.shoulder), PCELL.minShoulder, 2.7, ' m') +
      kv('TCP (cell frame)', c.last.tcp.map((v) => v.toFixed(3)).join('  ')) + kv('Vacuum SF / peel SF', `${isFinite(c.last.sf) ? c.last.sf.toFixed(2) : '–'} / ${isFinite(c.last.peel) ? c.last.peel.toFixed(2) : '–'}`) +
      kv('Run minimum SF / peel / carry', `${c.stats.minSF.toFixed(2)} / ${c.stats.minPeel.toFixed(2)} / ${(1000 * c.stats.minCarry).toFixed(0)} mm`) + kv('Infeed rows / queue / belt', `${c.infeed.rows.length} / ${c.infeed.queue.length} / ${c.belt.length}`) + kv('Monitor', c.last.bad[0] ?? `OK (${c.stats.violations})`);
  }
  if (r) {
    const s = r.last.sing;
    return kv('Phase / arm segment', `${r.phase} / ${r.seg ? r.seg.label : '–'}`) + kv('Joints q1…q6', r.q.map((q) => (q / DEG).toFixed(0)).join(' ') + '°') +
      bar('Elbow |sin q3|', Math.abs(s.elbow), ARM.minElbow, 1) + bar('Wrist |sin q5|', Math.abs(s.wrist), ARM.minWrist, 1) + bar('Shoulder |p_r|', Math.abs(s.shoulder), ARM.minShoulder, 1.2, ' m') +
      kv('Pose x y θ', `${r.pose.x.toFixed(2)} ${r.pose.y.toFixed(2)} ${deg(r.pose.th)}`) + kv('v / ω / wheels', `${r.v.toFixed(2)} m/s / ${deg(r.w)}/s / ${r.wheel.vL.toFixed(2)} ${r.wheel.vR.toFixed(2)}`) +
      kv('Deck', `${r.deck.boxes.length} boxes · stop ${r.deck.stop === 1 ? 'up' : 'down'}`) + kv('Vacuum SF / peel SF', `${isFinite(r.last.sf) ? r.last.sf.toFixed(2) : '–'} / ${isFinite(r.last.peel) ? r.last.peel.toFixed(2) : '–'}`) +
      kv('CG to tipping edge', r.last.stab.margin.toFixed(3) + ' m') + kv('Waiting for', r.drive.blockedBy ?? '–') + kv('Monitor', r.last.bad[0] ?? `OK (${r.stats.violations})`);
  }
  if (f) return kv('Phase', esc(f.phase)) + kv('Job', f.job ? `${f.job.kind}: ${f.job.pick.id} → ${f.job.drop.id}` : '–') + kv('Pose x y θ', `${f.pose.x.toFixed(2)} ${f.pose.y.toFixed(2)} ${deg(f.pose.th)}`) +
    kv('v / drive wheel', `${f.v.toFixed(2)} / ${f.vs.toFixed(2)} m/s`) + kv('Steer q2 / rate', `${deg(f.phi)} / ${deg(f.steerRate)}/s`) + kv('Tilt q3 / lift q4 / shift q5', `${deg(f.tilt)} / ${f.lift.toFixed(3)} / ${f.shift.toFixed(3)}`) +
    kv('Load', f.carried ? `${f.carried.id} ${f.carried.kind} ${f.carried.mass.toFixed(0)} kg` : '–') + kv('Fork zone / plan clearance', `${f.last.zone ?? '–'} / ${isFinite(f.last.clearance) ? f.last.clearance.toFixed(2) + ' m' : '–'}`) + kv('Waiting for', f.drive.blockedBy ?? '–') + kv('Monitor', f.last.bad[0] ?? `OK (${f.stats.violations})`);
  if (id.startsWith('CRANE')) { const a = w.asrs.aisles[+id.slice(5) - 1]; return kv('Phase', esc(a.phase)) + kv('q1 travel / q2 hoist / q3 fork', a.q.map((v) => v.toFixed(3)).join(' / ')) + kv('Command', a.op ? `${a.op.kind} ${a.op.slot.id}` : '–') + kv('Stored / retrieved', `${a.stats.stored} / ${a.stats.retrieved}`) + kv('det J', '−1 (no singularity)') + kv('Monitor', a.lastViolations[0] ?? `OK (${a.stats.violations})`); }
  const s = w.wrap.w;
  return kv('Phase', w.wrap.label()) + kv('q1 turntable', `${deg(s.th)} · ${((s.om * 60) / (2 * PI)).toFixed(1)} rpm`) + kv('q3 carriage', s.lift.toFixed(3) + ' m') + kv('Film tension / dancer', `${s.tension.toFixed(1)} N / ${(1000 * s.dancer).toFixed(1)} mm`) + kv('Moment arm p', s.arm.toFixed(3) + ' m') + kv('Clamp / jaw / slide', `${s.clampUp.toFixed(1)} / ${s.jaw.toFixed(1)} / ${s.slide.toFixed(2)}`) + kv('Monitor', s.bad[0] ?? `OK (${w.wrap.stats.violations})`);
}
function hud() {
  const w = world, O = w.wes.orders, v = w.violations(), nv = Object.values(v).reduce((a, b) => a + b, 0), bc = w.boxCount();
  $('time').textContent = hms(w.t);
  $('kOrders').textContent = `${O.filter((o) => o.state === 'open').length} / ${O.filter((o) => o.state === 'active').length} / ${w.wes.stats.done}`;
  const lay = w.robots.reduce((s, r) => s + r.stats.layers, 0);
  $('kLayers').textContent = `${lay} · ${w.t > 300 ? ((lay * 3600) / w.t).toFixed(1) : '–'}`;
  $('kPallets').textContent = `${w.cells.reduce((s, c) => s + c.stats.pallets, 0)} / ${w.wrap.stats.wrapped} / ${w.fleet.stats.done.store ?? 0} / ${w.stats.shipped}`;
  $('kBoxes').textContent = `${bc.rack} / ${bc.tracked} / ${bc.shipped}`;
  $('kEmpty').textContent = w.empties.map((s) => s.pallets.length).join(' / ');
  $('lMon').className = 'lamp' + (nv ? ' bad' : '');
  $('mon').textContent = nv ? Object.entries(v).filter((e) => e[1]).map((e) => e.join(' ')).join(', ') : 'OK (0 violations)';
  $('kGap').textContent = isFinite(w.stats.minGap) ? w.stats.minGap.toFixed(2) + ' m' : '–';
  rows('tLanes', ROWS.map((r) => { const l = w.asrs.lanes[r], p = l.pallet; return [r, `<span class="sw" style="background:#${SKU[l.sku.id].color.toString(16)}"></span>${l.sku.id}`, l.state + (l.locked ? ` · ${l.locked}` : ''), p ? `${p.layers} lay` : '–', `${(100 * w.asrs.fill(r)).toFixed(0)} %`]; }));
  rows('tCells', w.cells.map((c) => [c.id, c.palState, c.order ? `${c.order.id} ${c.order.placed}/${c.order.slots.length}` : '–', c.state, c.stats.pallets]));
  rows('tRobots', w.robots.map((r) => [r.id, r.phase + (r.drive.waitT > 0.5 ? ' (wait)' : ''), r.job ? `${r.job.sku} → PAL${r.job.cell + 1}` : '–', `${r.stats.layers} lay`]));
  rows('tForks', w.forklifts.map((f) => [f.id, esc(f.phase) + (f.drive.waitT > 0.5 ? ' (wait)' : ''), f.stats.jobs]));
  $('wPhase').textContent = w.wrap.label();
  $('wInfo').textContent = `${w.wrap.pallets.size} on the line · ${w.wrap.stats.wrapped} wrapped`;
  $('detail').innerHTML = detail();
  rows('tOrders', O.filter((o) => o.state !== 'built').slice(0, 9).map((o) => [o.id, o.layers.map((id) => `<span class="sw" title="${id}" style="background:#${SKU[id].color.toString(16)}"></span>`).join(''), o.state === 'active' ? `PAL${o.cell + 1}` : o.manual ? 'manual' : 'open', (PALLET.h + orderHeight(o.layers)).toFixed(3) + ' m']));
  rows('tStore', [['in', w.storage.filter((s) => s.pallet).map((s) => s.id).join(' ') || '–', `${w.stats.shipped} shipped`]]);
}

// ---------- cameras (three coordinates: x, up, −y of {G}) ----------
const g3 = (x, y, z) => [x, z, -y];
const VIEWS = {
  overview: [g3(14, -26, 40), g3(34, 20, 0)], top: [g3(34, 23.9, 92), g3(34, 24, 0)],
  asrs: [g3(27, 12, 15), g3(45, 31, 3)], cells: [g3(18, -12, 16), g3(18, 13, 1)],
  wrap: [g3(20, 22, 8), g3(20, 34.6, 1)], store: [g3(18, 31, 9), g3(18, 45, 0)],
};
let camView = 'overview';
function setView(name) {
  camView = name;
  document.querySelectorAll('[data-view]').forEach((b) => b.classList.toggle('on', b.dataset.view === name));
  if (!VIEWS[name]) return;
  camera.position.set(...VIEWS[name][0]); controls.target.set(...VIEWS[name][1]);
}
const tgt = new THREE.Vector3(), off = new THREE.Vector3(-6, 6, 7);
function chase() {
  if (camView !== 'follow') return;
  const id = $('pick').value, v = world.robots.find((x) => x.id === id) ?? world.forklifts.find((x) => x.id === id) ?? world.robots[0];
  tgt.set(v.pose.x, 1.0, -v.pose.y);
  controls.target.lerp(tgt, 0.12);
  camera.position.lerp(tgt.clone().add(off), 0.06);
}

// ---------- UI ----------
let paused = false, speed = 4, acc = 0;
$('pause').onclick = (e) => { paused = !paused; e.target.textContent = paused ? 'Resume' : 'Pause'; e.target.classList.toggle('on', paused); };
document.querySelectorAll('[data-speed]').forEach((b) => (b.onclick = () => { speed = +b.dataset.speed; document.querySelectorAll('[data-speed]').forEach((x) => x.classList.toggle('on', x === b)); }));
document.querySelectorAll('[data-view]').forEach((b) => (b.onclick = () => setView(b.dataset.view)));
$('panel').onclick = (e) => { const h = !$('hud').hidden; $('hud').hidden = $('side').hidden = h; e.target.classList.toggle('on', !h); };
$('paths').onclick = (e) => { roads.visible = !roads.visible; e.target.classList.toggle('on', roads.visible); };
$('claims').onclick = (e) => { claimLines.visible = !claimLines.visible; e.target.classList.toggle('on', claimLines.visible); };
$('labels').onclick = (e) => { labels.visible = !labels.visible; e.target.classList.toggle('on', labels.visible); };
$('sF').oninput = (e) => ($('nF').textContent = e.target.value);
$('sM').oninput = (e) => ($('nM').textContent = e.target.value);
$('reset').onclick = () => reset(+$('sF').value, +$('sM').value);
$('skuHead').innerHTML = SKUS.map((s) => `<span title="${s.l}×${s.w}×${s.h} m, ${s.mass} kg, ${s.perLayer} per layer"><span class="sw" style="background:#${s.color.toString(16)}"></span>${s.id}</span>`).join('');
$('skuIn').innerHTML = SKUS.map((s) => `<input type="number" min="0" max="6" value="0" data-sku="${s.id}">`).join('');
const draft = () => [...document.querySelectorAll('[data-sku]')].flatMap((i) => Array(Math.max(0, Math.min(6, +i.value || 0))).fill(i.dataset.sku));
const showDraft = () => { const d = draft(), h = orderHeight(d); $('oHeight').textContent = `${d.length} layers · ${h.toFixed(3)} m`; $('oHeight').className = h > MIX.maxBoxH + 1e-9 ? 'badc' : 'dim'; };
document.querySelectorAll('[data-sku]').forEach((i) => (i.oninput = showDraft));
$('oAdd').onclick = () => { const r = world.wes.addOrder(draft(), { front: true }); $('msg').textContent = r.error ? r.error : `${r.id} added: ${r.layers.join(' · ')} (bottom → top)`; };
$('oAuto').onclick = (e) => { world.wes.auto = !world.wes.auto; e.target.textContent = world.wes.auto ? 'On' : 'Off'; e.target.classList.toggle('on', world.wes.auto); };

function resize() { view.resize(innerWidth, innerHeight); }
addEventListener('resize', resize);
resize();

// ---------- main loop: fixed 240 Hz simulation step ----------
let last = performance.now(), hudT = 0, err = null;
reset();
setView('overview');
// Integration handle: window.wh.world is the World (see WAREHOUSE_SPEC.md §10).
window.wh = { get world() { return world; }, setView, camera, controls, cellView, forkView, mmView, cranes, sync, reset, step: (n = 1) => { for (let i = 0; i < n; i++) world.step(DT); }, get error() { return err; } };
function frame(now) {
  const wall = Math.min((now - last) / 1000, 0.1);
  last = now;
  if (!paused && !err) {
    acc += wall * speed;
    const t0 = performance.now();
    let n = 0;
    try { while (acc >= DT && performance.now() - t0 < 28) { world.step(DT); acc -= DT; n++; } } catch (e) { err = e; console.error(e); $('mon').textContent = e.message; }
    if (acc > 0.5) acc = 0; // the machine cannot keep up with this speed: run as fast as it can
  }
  sync();
  if (now - hudT > 200) { hud(); hudT = now; }
  chase();
  controls.update();
  view.render();
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
