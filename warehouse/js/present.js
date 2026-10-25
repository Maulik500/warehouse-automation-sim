// Guided presentation over a recorded run: directed cameras, captions, pinned labels, live
// counters and an engineering panel with the joint angles and singularity margins of the machine
// in focus. Works from a server (fetches rec/clip.json.gz) or offline (recording embedded as
// window.__REC). Capture mode for the video: present.html?capture=1920x1080 → window.__seek(v).

import { createView } from './view.js';
import { Player } from './state.js';
import { buildTour } from './tour.js';
import { World } from './sim.js';
import { singularity as armSing } from '../../mobman/js/arm.js';
import { ARM } from '../../mobman/js/config.js';
import { singularity as palSing } from '../../js/kinematics.js';
import { PCELL, DEG } from './config.js';

const $ = (id) => document.getElementById(id);
const capM = location.search.match(/capture=(\d+)x(\d+)/), CAPTURE = !!capM;
if (CAPTURE) document.body.classList.add('capture');
const player = window.__REC ? await Player.fromGzipBase64(window.__REC) : await Player.fromUrl('rec/clip.json.gz');
const layout = new World({ seed: 7, forklifts: player.r.nForks, robots: player.r.nRobots });
const view = createView({ layout, theme: 'light', container: $('wrap'), capture: CAPTURE });
const tour = buildTour(player, view);
view.labels.visible = false; // the presentation uses pinned HTML labels instead of 3D text
$('load').remove();
$('introMeta').innerHTML = `Run of <b>${(player.T / 60).toFixed(0)} min</b> · ${player.r.nForks} forklift AMRs · ${player.r.nRobots} robot AMRs · seed ${player.r.meta.seed}`;

function resize() {
  const w = CAPTURE ? +capM[1] : $('wrap').clientWidth, h = CAPTURE ? +capM[2] : $('wrap').clientHeight;
  view.resize(w, h);
  if (CAPTURE) Object.assign(view.renderer.domElement.style, { width: w + 'px', height: h + 'px' });
}
addEventListener('resize', resize);
resize();

// ---------- results card: measured over the recording ----------
{
  const k0 = player.kpi.find(Boolean), k1 = player.kpi[player.kpi.length - 1], H = (k1.t - k0.t) / 3600;
  const rows = [
    ['Mixed pallets built in this run', `${k1.built - k0.built} in ${(H * 60).toFixed(0)} min = ${((k1.built - k0.built) / H).toFixed(1)} / h`],
    ['Layers delivered by 2 robot AMRs', `${((k1.layers - k0.layers) / H).toFixed(1)} / h`],
    ['Boxes palletized', `${k1.placed - k0.placed}`],
    ['Safety-monitor violations (all machines, all vehicles)', `${k1.viol}`],
    ['Closest approach between two vehicles', `${k1.gap.toFixed(2)} m (≥ 0.30 required)`],
    ['Verification suite (8 simulated h, 3 fleet sizes)', 'all checks passed'],
  ];
  $('resT').innerHTML = rows.map((r) => `<tr><td>${r[0]}</td><td>${r[1]}</td></tr>`).join('');
}

// ---------- step list and timeline ticks ----------
const numbered = tour.steps.filter((s) => s.n);
numbered.forEach((s) => {
  const li = document.createElement('li');
  li.innerHTML = `<b>${s.n}</b><span>${s.caption}</span>`;
  li.onclick = () => go(s.v0);
  $('steps').appendChild(li);
  const t = document.createElement('div');
  t.className = 'tick'; t.style.left = `${(100 * s.v0) / tour.end}%`;
  $('tl').appendChild(t);
});
$('scrub').max = tour.end;

// ---------- pinned labels ----------
const labDivs = new Map();
let showLabels = true, showEng = true, follow = true, playing = !CAPTURE, curV = 0;
function drawLabels(list, s) {
  for (const d of labDivs.values()) d.style.display = 'none';
  if (!showLabels) return;
  for (const [name, html] of list) {
    const p = view.anchor(name, s), xy = p && view.project([p[0], p[1], p[2] + 0.4]);
    if (!xy) continue;
    let d = labDivs.get(name + html);
    if (!d) { d = document.createElement('div'); d.className = 'lab'; d.innerHTML = html; $('labels').appendChild(d); labDivs.set(name + html, d); }
    d.style.display = ''; d.style.left = xy[0] + 'px'; d.style.top = xy[1] + 'px';
  }
}

// ---------- engineering panel ----------
const deg = (a) => (a / DEG).toFixed(0);
const meter = (name, v, lim, full) => `<div class="m"><span>${name}</span><div class="bar${v < lim ? ' bad' : v < 1.6 * lim ? ' warn' : ''}"><i style="width:${Math.min(100, (100 * v) / full)}%"></i></div><span class="num">${v.toFixed(2)}</span></div>`;
function engPanel(inset, s) {
  if (!inset || !showEng) return null;
  if (inset.kind === 'mobile') {
    const r = s.robots[+inset.who.slice(1) - 1]; if (!r) return null;
    const g = armSing(r.q);
    return [`${inset.who} · UR20-CLASS ARM ON AMR`, `<div class="q">q1…q6 = ${r.q.map(deg).join(' · ')}°</div>` + meter('Elbow |sin q3|', Math.abs(g.elbow), ARM.minElbow, 1) + meter('Wrist |sin q5|', Math.abs(g.wrist), ARM.minWrist, 1) + meter('Shoulder |p_r| m', Math.abs(g.shoulder), ARM.minShoulder, 1.2) + `<div class="q">Base ${r.x.toFixed(2)}, ${r.y.toFixed(2)} m · θ ${deg(r.th)}° · ${r.held ? 'box held, vacuum on' : r.vac ? 'vacuum on' : 'vacuum off'}</div>`];
  }
  if (inset.kind === 'cell') {
    const c = s.cells[inset.i], g = palSing(c.q);
    return [`PALLETIZER ${inset.i + 1} · KR 120 R2700 CLASS`, `<div class="q">q1…q6 = ${c.q.map(deg).join(' · ')}°</div>` + meter('Wrist |sin q5|', Math.abs(g.wrist), PCELL.minWrist, 1) + meter('Elbow |sin(q3+β)|', Math.abs(g.elbow), PCELL.minElbow, 1) + meter('Shoulder ρ m', Math.abs(g.shoulder), PCELL.minShoulder, 2.7) + `<div class="q">${c.held ? 'Box held · tool vertical' : 'Gripper empty'} · J1 limited to −30…+110°</div>`];
  }
  if (inset.kind === 'fork') {
    const f = s.forks[+inset.who.slice(1) - 1]; if (!f) return null;
    return [`${inset.who} · COUNTERBALANCED FORKLIFT AMR`, `<div class="q">Steer q2 ${deg(f.phi)}° · tilt q3 ${(f.tilt / DEG).toFixed(1)}° · lift q4 ${f.lift.toFixed(3)} m · side shift q5 ${(1000 * f.shift).toFixed(0)} mm</div><div class="q">Tricycle, non-holonomic: steer from path curvature, all stops on straight pieces, mast moves only at standstill</div>`];
  }
  if (inset.kind === 'crane') {
    const q = s.cranes[inset.i];
    return [`STACKER CRANE ${inset.i + 1} · PPP`, `<div class="q">Travel q1 ${q[0].toFixed(3)} m · hoist q2 ${q[1].toFixed(3)} m · fork q3 ${q[2].toFixed(3)} m</div><div class="q">det J = −1 everywhere: no singularity. Safety rules R1–R7: travel only with the fork centred, fork only into an open cell</div>`];
  }
  if (inset.kind === 'wrap') {
    const w = s.wrap;
    return ['BELLY WRAPPER · TURNTABLE', `<div class="q">Turntable q1 ${((w.th % (2 * Math.PI)) / DEG).toFixed(0)}° · carriage q3 ${w.lift.toFixed(3)} m · clamp ${w.clampUp > 0.5 ? 'up' : 'down'} · cut slide ${w.slide.toFixed(2)} m</div><div class="q">Film speed = ω · p(q1): the moment arm changes 1.9 : 1 four times per turn, absorbed by the dancer</div>`];
  }
  return null;
}

// ---------- timeline ----------
const stepAt = (v) => tour.steps.filter((s) => v >= s.v0).pop() ?? tour.steps[0];
const fmt = (v) => `${Math.floor(v / 60)}:${String(Math.floor(v % 60)).padStart(2, '0')}`;
const fade = (id, a) => { const el = $(id); el.style.opacity = a; el.style.visibility = a > 0 ? 'visible' : 'hidden'; };
function renderAt(v) {
  curV = v;
  const st = stepAt(v), u = Math.min(1, Math.max(0, (v - st.v0) / (st.v1 - st.v0))), simT = Math.min(player.T, st.s0 + (Math.min(v, st.v1) - st.v0) * st.speed);
  const s = player.at(Math.max(0, simT));
  view.apply(s);
  if (follow || CAPTURE) {
    const c = st.cam(s, u);
    view.camera.position.set(c.eye[0], c.eye[2], -c.eye[1]);
    view.controls.target.set(c.tgt[0], c.tgt[2], -c.tgt[1]);
    view.camera.lookAt(view.controls.target);
  } else view.controls.update();
  view.render();
  const inStep = v >= tour.steps[0].v0 && v < tour.results[0];
  drawLabels(inStep ? st.labels : [], s);
  $('cap').style.opacity = inStep ? 1 : 0;
  $('capN').textContent = st.n ?? ''; $('capN').style.display = st.n ? '' : 'none'; $('capT').textContent = st.caption;
  const k = s.kpi;
  if (k) {
    const sec = Math.floor(k.t);
    $('kT').textContent = `${String(Math.floor(sec / 3600)).padStart(2, '0')}:${String(Math.floor(sec / 60) % 60).padStart(2, '0')}:${String(sec % 60).padStart(2, '0')}`;
    $('kBoxes').textContent = k.placed; $('kLayers').textContent = k.layers; $('kPal').textContent = `${k.built} / ${k.wrapped}`; $('kShip').textContent = `${k.stored} / ${k.shipped}`; $('kViol').textContent = k.viol;
  }
  $('hud').style.opacity = inStep ? 1 : 0;
  const ep = inStep && engPanel(st.inset, s);
  $('eng').style.opacity = ep ? 1 : 0;
  if (ep) { $('engH').textContent = ep[0]; $('engB').innerHTML = ep[1]; }
  const span = (a, b) => Math.max(0, Math.min(1, (v - a) / 0.5, (b - v) / 0.5));
  fade('intro', v < tour.intro[1] ? Math.min(1, (tour.intro[1] - v) / 0.6) : 0);
  fade('results', span(...tour.results));
  fade('outro', v >= tour.outro[0] ? Math.min(1, (v - tour.outro[0]) / 0.5) : 0);
  if (!CAPTURE) {
    $('scrub').value = v; $('tNow').textContent = `${fmt(v)} / ${fmt(tour.end)}`;
    [...$('steps').children].forEach((li, i) => li.classList.toggle('cur', numbered[i] === st && inStep));
  }
}
function go(v) { curV = Math.max(0, Math.min(tour.end, v)); follow = true; $('free').classList.remove('on'); renderAt(curV); }
window.__seek = (v) => { renderAt(v); return v; };
window.__tour = { end: tour.end, steps: tour.steps.map((s) => ({ n: s.n, caption: s.caption, v0: s.v0, v1: s.v1 })) };
window.__ready = true;

if (!CAPTURE) {
  $('play').onclick = () => { playing = !playing; $('play').textContent = playing ? 'Pause' : 'Play'; };
  $('free').onclick = () => { follow = !follow; $('free').classList.toggle('on', !follow); };
  $('labT').onclick = () => { showLabels = !showLabels; $('labT').classList.toggle('on', showLabels); };
  $('engT').onclick = () => { showEng = !showEng; $('engT').classList.toggle('on', showEng); };
  view.controls.addEventListener('start', () => { follow = false; $('free').classList.add('on'); });
  $('scrub').oninput = () => renderAt(+$('scrub').value);
  $('prev').onclick = () => { const p = numbered.filter((s) => s.v0 < curV - 0.5).pop(); go(p ? p.v0 : 0); };
  $('next').onclick = () => { const n = numbered.find((s) => s.v0 > curV + 0.1); go(n ? n.v0 : tour.results[0]); };
  if (window.__REC) $('live').href = 'Warehouse_Live.html'; // offline: the live file sits next to this one
  let last = performance.now();
  (function loop(now) {
    requestAnimationFrame(loop);
    if (playing) { curV += Math.min(0.1, (now - last) / 1000); if (curV > tour.end) curV = 0; }
    last = now;
    renderAt(curV);
  })(performance.now());
} else renderAt(0);
