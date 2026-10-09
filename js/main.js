// three.js view of the palletizing cell. All geometry lives in `W`, a Z-up
// group, so coordinates here are identical to ROBOT_SPEC.md and sim.js.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { ROBOT as RB, TOOL, BOX, CELL, R2D } from './config.js';
import { fk, singularity } from './kinematics.js';
import { Sim } from './sim.js';

const { conv, pallet } = CELL;
const PI = Math.PI;

// ---------- renderer / scene ----------
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
document.body.prepend(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14171c);
scene.fog = new THREE.Fog(0x14171c, 14, 34);

const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 100);
camera.position.set(5.2, 3.5, -4.4);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(1.1, 0.75, -0.3);
controls.enableDamping = true;
controls.maxPolarAngle = PI / 2 - 0.02;

scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x2a2622, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 2.4);
sun.position.set(4.5, 9, 3.5);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -6, right: 6, top: 6, bottom: -6, near: 1, far: 25 });
sun.shadow.bias = -0.0004;
scene.add(sun);

const W = new THREE.Group(); // world frame {W}: Z up
W.rotation.x = -PI / 2;
scene.add(W);

// ---------- helpers (all in Z-up coordinates) ----------
const mat = (color, o = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.15, ...o });
const M = {
  body: mat(0xf26a1b, { roughness: 0.42 }),
  dark: mat(0x23272e, { roughness: 0.6, metalness: 0.4 }),
  steel: mat(0x8d96a3, { roughness: 0.35, metalness: 0.7 }),
  frame: mat(0x3a4250, { metalness: 0.5 }),
  wood: mat(0xb98a55, { roughness: 0.85, metalness: 0 }),
  card: mat(0xc49a6c, { roughness: 0.9, metalness: 0 }),
  tape: mat(0x8b6a44, { roughness: 0.5, metalness: 0 }),
  cup: mat(0x1b1d21, { roughness: 0.8, emissive: 0x000000 }),
};
function add(parent, geo, material, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(geo, material);
  m.position.set(x, y, z);
  m.castShadow = m.receiveShadow = true;
  parent.add(m);
  return m;
}
const box = (p, sx, sy, sz, m, x, y, z) => add(p, new THREE.BoxGeometry(sx, sy, sz), m, x, y, z);
// Cylinder along a named axis; rEnd is the radius at the positive end.
function cyl(p, rEnd, rStart, len, axis, m, x, y, z) {
  const g = new THREE.CylinderGeometry(rEnd, rStart, len, 40);
  if (axis === 'x') g.rotateZ(-PI / 2);
  if (axis === 'z') g.rotateX(PI / 2);
  return add(p, g, m, x, y, z);
}
const group = (parent, x = 0, y = 0, z = 0) => {
  const g = new THREE.Group();
  g.position.set(x, y, z);
  parent.add(g);
  return g;
};

// ---------- floor ----------
{
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(60, 60), mat(0x1d2127, { roughness: 0.95, metalness: 0 }));
  floor.receiveShadow = true;
  W.add(floor);
  const grid = new THREE.GridHelper(20, 40, 0x39404b, 0x272c34);
  grid.rotation.x = PI / 2;
  grid.position.z = 0.002;
  W.add(grid);
  // safety perimeter marking
  const pts = [[-1.3, -4.2], [2.7, -4.2], [2.7, 3.0], [-1.3, 3.0], [-1.3, -4.2]].map(([x, y]) => new THREE.Vector3(x, y, 0.004));
  W.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: 0xf0b429 })));
}

// ---------- robot: scene graph follows spec §3.1 exactly ----------
const J = [];
{
  const { d1, a1, a2, a3, d4, d6 } = RB;
  box(W, 0.95, 0.95, 0.03, M.dark, 0, 0, 0.015);
  cyl(W, 0.38, 0.42, 0.19, 'z', M.dark, 0, 0, 0.125);

  const j1 = group(W); // axis Z
  cyl(j1, 0.33, 0.34, 0.22, 'z', M.body, 0, 0, 0.33);
  box(j1, 0.44, 0.27, 0.42, M.body, 0.30, 0.05, 0.63);
  cyl(j1, 0.13, 0.13, 0.20, 'y', M.dark, a1, 0.28, d1);
  box(j1, 0.22, 0.24, 0.30, M.dark, -0.12, 0.02, 0.59);

  const j2 = group(j1, a1, 0, d1); // axis Y
  cyl(j2, 0.21, 0.21, 0.16, 'y', M.body, 0, -0.17, 0);
  box(j2, 0.26, 0.16, a2, M.body, 0, -0.17, a2 / 2);
  cyl(j2, 0.18, 0.18, 0.16, 'y', M.body, 0, -0.17, a2);
  cyl(j2, 0.09, 0.09, 0.03, 'y', M.dark, 0, -0.265, 0);

  const j3 = group(j2, 0, 0, a2); // axis Y
  cyl(j3, 0.17, 0.17, 0.26, 'y', M.body, 0, 0.04, 0);
  box(j3, 0.50, 0.26, 0.30, M.body, 0.03, 0.04, a3 + 0.02);
  box(j3, 0.24, 0.24, 0.24, M.dark, -0.33, 0.04, a3 + 0.02);

  const j4 = group(j3, 0, 0, a3); // axis X
  cyl(j4, 0.085, 0.12, 0.78, 'x', M.body, 0.67, 0, 0);
  box(j4, 0.06, 0.21, 0.16, M.body, 1.07, 0, 0);
  for (const s of [-1, 1]) box(j4, 0.20, 0.035, 0.16, M.body, 1.17, s * 0.0875, 0);

  const j5 = group(j4, d4, 0, 0); // axis Y
  cyl(j5, 0.07, 0.07, 0.14, 'y', M.dark, 0, 0, 0);
  box(j5, 0.09, 0.11, 0.11, M.steel, 0.07, 0, 0);

  const j6 = group(j5); // axis X
  cyl(j6, 0.07, 0.07, 0.10, 'x', M.dark, d6 - 0.05, 0, 0);

  const flange = group(j6, d6, 0, 0); // {F}: Z out of the flange
  flange.rotation.y = PI / 2;
  cyl(flange, 0.055, 0.065, 0.10, 'z', M.steel, 0, 0, 0.05);
  box(flange, TOOL.bodyX, TOOL.bodyY, 0.10, M.frame, 0, 0, 0.15);
  box(flange, 0.10, 0.08, 0.05, M.dark, 0.10, 0, 0.075);
  for (const x of [-0.12, 0, 0.12]) for (const y of [-0.07, 0.07]) {
    cyl(flange, 0.012, 0.012, 0.05, 'z', M.steel, x, y, 0.225);
    cyl(flange, TOOL.cupDia / 2, 0.018, 0.05, 'z', M.cup, x, y, TOOL.tcp - 0.025);
  }
  const tcp = group(flange, 0, 0, TOOL.tcp);
  J.push(j1, j2, j3, j4, j5, j6, flange, tcp);
}
const AX = ['z', 'y', 'y', 'x', 'y', 'x'];
const setJoints = (q) => q.forEach((v, i) => { J[i].rotation[AX[i]] = v; });
const axes = J.slice(0, 7).map((g) => { const a = new THREE.AxesHelper(0.35); a.visible = false; g.add(a); return a; });

// ---------- conveyor ----------
const beltTex = (() => {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = '#22252b'; g.fillRect(0, 0, 64, 64);
  g.fillStyle = '#31363f'; g.fillRect(0, 0, 64, 5);
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
})();
const BELT_TILE = 0.25;
const zoneMeshes = [];
{
  const len = conv.yEnd - conv.yStart, yc = (conv.yEnd + conv.yStart) / 2, w = conv.width;
  box(W, w, len, 0.08, M.frame, conv.x, yc, conv.top - 0.045);
  beltTex.repeat.set(1, len / BELT_TILE);
  const belt = add(W, new THREE.PlaneGeometry(w - 0.02, len), mat(0xffffff, { map: beltTex, roughness: 0.9, metalness: 0 }), conv.x, yc, conv.top + 0.001);
  belt.castShadow = false;
  for (const s of [-1, 1]) {
    box(W, 0.04, len, 0.13, M.steel, conv.x + s * (w / 2 + 0.02), yc, conv.top - 0.02);
    for (let y = conv.yStart + 0.2; y < conv.yEnd; y += 1.1) box(W, 0.06, 0.06, conv.top - 0.085, M.frame, conv.x + s * (w / 2 - 0.02), y, (conv.top - 0.085) / 2);
    // tracking window marker on the rail
    zoneMeshes.push(box(W, 0.012, conv.yB - conv.yA, 0.03, mat(0x3ecf8e, { emissive: 0x1c6b48 }), conv.x + s * (w / 2 + 0.046), (conv.yA + conv.yB) / 2, conv.top + 0.02));
  }
  for (const y of [conv.yStart, conv.yEnd]) cyl(W, 0.045, 0.045, w, 'x', M.steel, conv.x, y, conv.top - 0.045);
  // reject bin for boxes that were not picked
  const by = conv.yEnd + 0.45;
  box(W, 0.7, 0.7, 0.02, M.frame, conv.x, by, 0.01);
  for (const [dx, dy, sx, sy] of [[0.35, 0, 0.02, 0.7], [-0.35, 0, 0.02, 0.7], [0, 0.35, 0.7, 0.02]]) box(W, sx, sy, 0.5, M.frame, conv.x + dx, by + dy, 0.25);
}

// ---------- pallet (EUR 1200 x 800 x 144) ----------
{
  const P = group(W, pallet.x, pallet.y, 0);
  const xs = [-0.3275, 0, 0.3275], ys = [-0.5275, 0, 0.5275];
  for (const x of xs) box(P, 0.1, 1.2, 0.022, M.wood, x, 0, 0.011);
  for (const x of xs) for (const y of ys) box(P, 0.145, 0.145, 0.078, M.wood, x, y, 0.061);
  for (const y of ys) box(P, 0.8, 0.145, 0.022, M.wood, 0, y, 0.111);
  [[-0.3275, 0.145], [-0.164, 0.1], [0, 0.145], [0.164, 0.1], [0.3275, 0.145]].forEach(([x, w]) => box(P, w, 1.2, 0.022, M.wood, x, 0, 0.133));
}

// ---------- cartons ----------
const boxGeo = new THREE.BoxGeometry(BOX.sx, BOX.sy, BOX.sz);
const tapeGeo = new THREE.BoxGeometry(0.05, BOX.sy + 0.002, BOX.sz + 0.002);
const edgeGeo = new THREE.EdgesGeometry(boxGeo);
const edgeMat = new THREE.LineBasicMaterial({ color: 0x6e5235 });
const boxMeshes = new Map();
function syncBoxes(sim) {
  const alive = new Set();
  for (const b of sim.boxes) {
    alive.add(b.id);
    let g = boxMeshes.get(b.id);
    if (!g) {
      g = new THREE.Group();
      add(g, boxGeo, M.card);
      add(g, tapeGeo, M.tape).castShadow = false;
      g.add(new THREE.LineSegments(edgeGeo, edgeMat));
      g.matrixAutoUpdate = false;
      W.add(g);
      boxMeshes.set(b.id, g);
    }
    const R = b.R, p = b.p;
    g.matrix.set(R[0], R[1], R[2], p[0], R[3], R[4], R[5], p[1], R[6], R[7], R[8], p[2], 0, 0, 0, 1);
  }
  for (const [id, g] of boxMeshes) if (!alive.has(id)) { W.remove(g); boxMeshes.delete(id); }
}

// ---------- TCP trail ----------
const TRAIL = 2400;
const trailPos = new Float32Array(TRAIL * 3);
const trailGeo = new THREE.BufferGeometry();
trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPos, 3));
const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0x4cc9f0, transparent: true, opacity: 0.8 }));
trail.frustumCulled = false;
W.add(trail);
let trailN = 0;
function pushTrail(p) {
  if (trailN === TRAIL) { trailPos.copyWithin(0, 3); trailN--; }
  trailPos.set(p, trailN * 3);
  trailN++;
  trailGeo.setDrawRange(0, trailN);
  trailGeo.attributes.position.needsUpdate = true;
}

// ---------- HUD ----------
const $ = (id) => document.getElementById(id);
const jointRows = [...Array(6)].map((_, i) => {
  const row = document.createElement('div');
  row.className = 'j';
  row.innerHTML = `<span>J${i + 1}</span><span></span><div class="bar pos"><i></i></div><div class="bar"><i></i></div>`;
  $('joints').append(row);
  return { val: row.children[1], pos: row.children[2].firstChild, spd: row.children[3].firstChild };
});
const colour = (v, warn, bad) => (v < bad ? 'var(--bad)' : v < warn ? 'var(--warn)' : 'var(--ok)');
function updateHud(sim, f, tcpSpeed) {
  $('state').textContent = sim.state;
  $('vac').textContent = sim.vac ? 'ON  (-60 kPa)' : 'off';
  $('vac').style.color = sim.vac ? 'var(--ok)' : 'var(--dim)';
  const total = pallet.nx * pallet.ny * pallet.layers;
  $('count').textContent = `${sim.placed} / ${total}  · layer ${Math.min(pallet.layers, Math.floor(sim.placed / (pallet.nx * pallet.ny)) + 1)}`;
  $('tot').textContent = `${sim.pallets} / ${sim.missed}`;
  const ct = sim.cycleTimes;
  $('ct').textContent = ct.length ? `${ct.at(-1).toFixed(2)} / ${(ct.reduce((a, b) => a + b, 0) / ct.length).toFixed(2)} s` : '–';
  $('time').textContent = `${sim.t.toFixed(1)} s`;
  sim.q.forEach((q, i) => {
    const r = jointRows[i], sp = Math.abs(sim.qd[i]) / RB.vMax[i];
    r.val.textContent = `${(q * R2D).toFixed(1)}°`;
    r.pos.style.left = `calc(${((q - RB.qMin[i]) / (RB.qMax[i] - RB.qMin[i])) * 100}% - 1.5px)`;
    r.spd.style.width = `${Math.min(1, sp) * 100}%`;
    r.spd.style.background = sp > 0.98 ? 'var(--warn)' : 'var(--ok)';
  });
  const sg = singularity(sim.q);
  const set = (id, v, txt, warn, bad) => { $(id).textContent = txt; $(id).style.color = colour(Math.abs(v), warn, bad); };
  set('sw', sg.wrist, Math.abs(sg.wrist).toFixed(3), 0.4, 0.26);
  set('se', sg.elbow, Math.abs(sg.elbow).toFixed(3), 0.4, 0.14);
  set('ss', sg.shoulder, `${sg.shoulder.toFixed(3)} m`, 0.6, 0.3);
  $('tcp').textContent = f.tcp.map((v) => v.toFixed(3)).join('  ');
  $('tcpv').textContent = `${tcpSpeed.toFixed(2)} m/s`;
}

// ---------- controls ----------
let sim = new Sim(), speed = 1, paused = false;
$('pause').onclick = (e) => { paused = !paused; e.target.textContent = paused ? 'Resume' : 'Pause'; e.target.classList.toggle('on', paused); };
$('reset').onclick = () => { sim = new Sim(); trailN = 0; trailGeo.setDrawRange(0, 0); };
document.querySelectorAll('[data-speed]').forEach((b) => {
  b.onclick = () => {
    speed = +b.dataset.speed;
    document.querySelectorAll('[data-speed]').forEach((o) => o.classList.toggle('on', o === b));
  };
});
const toggle = (id, fn) => { $(id).onclick = (e) => fn(e.target.classList.toggle('on')); };
toggle('trail', (on) => { trail.visible = on; });
toggle('frames', (on) => axes.forEach((a) => { a.visible = on; }));
toggle('zones', (on) => zoneMeshes.forEach((m) => { m.visible = on; }));

function resize() {
  renderer.setSize(innerWidth, innerHeight);
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);
resize();

// ---------- main loop: fixed 240 Hz simulation step ----------
const DT = 1 / 240;
let acc = 0, last = performance.now(), prevTcp = fk(sim.q).tcp, tcpSpeed = 0;
function frame(now) {
  const real = Math.min(0.05, (now - last) / 1000);
  last = now;
  if (!paused) {
    acc += real * speed;
    let moved = 0;
    while (acc >= DT) { sim.step(DT); acc -= DT; moved += DT; }
    if (moved) {
      const p = fk(sim.q).tcp;
      tcpSpeed = Math.hypot(p[0] - prevTcp[0], p[1] - prevTcp[1], p[2] - prevTcp[2]) / moved;
      prevTcp = p;
      pushTrail(p);
      beltTex.offset.y -= (conv.v * moved) / BELT_TILE;
    }
  }
  setJoints(sim.q);
  syncBoxes(sim);
  M.cup.emissive.setHex(sim.vac ? 0x1f9d63 : 0x000000);
  updateHud(sim, fk(sim.q), paused ? 0 : tcpSpeed);
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

// Debug hook: distance between the rendered TCP and the analytic FK, in metres.
window.__cell = {
  get sim() { return sim; },
  tcpError() {
    setJoints(sim.q);
    W.updateMatrixWorld(true);
    const p = W.worldToLocal(J[7].getWorldPosition(new THREE.Vector3()));
    const f = fk(sim.q).tcp;
    return Math.hypot(p.x - f[0], p.y - f[1], p.z - f[2]);
  },
};
