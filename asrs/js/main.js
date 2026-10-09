// three.js view of the ASRS. All geometry lives in `W`, a Z-up group, so
// coordinates here are identical to ASRS_SPEC.md and sim.js.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { CRANE, FORK, LOAD, RACK, PD, LAYOUT, SKUS } from './config.js';
import { fk, jacobian, det3, bayX, levelZ } from './kinematics.js';
import { Warehouse } from './sim.js';

const PI = Math.PI;
const AX = CRANE.axes;
const xMid = (bayX(1) + bayX(RACK.bays)) / 2;
const yBack = RACK.yFace + RACK.depth;
const yAll = LAYOUT.aisles.map((a) => a.y);
const yMid = (Math.max(...yAll) + Math.min(...yAll)) / 2;

// ---------- renderer / scene ----------
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
document.body.prepend(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14171c);
scene.fog = new THREE.Fog(0x14171c, 60, 150);

const camera = new THREE.PerspectiveCamera(40, 1, 0.1, 400);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.maxPolarAngle = PI / 2 - 0.02;

scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x2a2622, 1.3));
const sun = new THREE.DirectionalLight(0xffffff, 2.1);
sun.position.set(xMid - 14, 34, -yMid + 22);
sun.target.position.set(xMid - 2, 4, -yMid);
sun.castShadow = true;
sun.shadow.mapSize.set(4096, 4096);
Object.assign(sun.shadow.camera, { left: -22, right: 22, top: 20, bottom: -20, near: 1, far: 90 });
sun.shadow.bias = -0.0006;
scene.add(sun, sun.target);

const W = new THREE.Group(); // world frame {W}: Z up
W.rotation.x = -PI / 2;
scene.add(W);

// ---------- helpers (all in Z-up coordinates) ----------
const mat = (color, o = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.2, ...o });
const M = {
  crane: mat(0xf2b01b, { roughness: 0.45 }),
  dark: mat(0x23272e, { roughness: 0.6, metalness: 0.4 }),
  steel: mat(0x9aa3b0, { roughness: 0.35, metalness: 0.7 }),
  fork: mat(0xd9dee6, { roughness: 0.3, metalness: 0.8 }),
  forkMid: mat(0x7c8694, { roughness: 0.35, metalness: 0.8 }),
  conv: mat(0x4a5361, { metalness: 0.5 }),
  rail: mat(0x6b7480, { roughness: 0.4, metalness: 0.8 }),
  upright: mat(0x2f5d9e, { roughness: 0.55, metalness: 0.35 }),
  ledge: mat(0xe0742a, { roughness: 0.5, metalness: 0.3 }),
  wood: mat(0xb98a55, { roughness: 0.85, metalness: 0 }),
  box: mat(0xffffff, { roughness: 0.88, metalness: 0 }),
};

function add(parent, geo, material, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(geo, material);
  m.position.set(x, y, z);
  m.castShadow = m.receiveShadow = true;
  parent.add(m);
  return m;
}
const box = (p, sx, sy, sz, m, x, y, z) => add(p, new THREE.BoxGeometry(sx, sy, sz), m, x, y, z);
function cylGeo(r, len, axis) {
  const g = new THREE.CylinderGeometry(r, r, len, 20);
  if (axis === 'x') g.rotateZ(-PI / 2);
  if (axis === 'z') g.rotateX(PI / 2);
  return g;
}
const cyl = (p, r, len, axis, m, x, y, z) => add(p, cylGeo(r, len, axis), m, x, y, z);
const group = (parent, x = 0, y = 0, z = 0) => {
  const g = new THREE.Group();
  g.position.set(x, y, z);
  parent.add(g);
  return g;
};
const m4 = new THREE.Matrix4();
// Many identical boxes in one draw call: list = [[sx, sy, sz, x, y, z], ...]
function boxes(parent, list, material) {
  const im = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), material, list.length);
  list.forEach(([sx, sy, sz, x, y, z], i) => im.setMatrixAt(i, m4.makeScale(sx, sy, sz).setPosition(x, y, z)));
  im.castShadow = im.receiveShadow = true;
  parent.add(im);
  return im;
}
function label(parent, text, x, y, z, color = '#d9dee6', size = 1) {
  const c = document.createElement('canvas');
  c.width = 256; c.height = 96;
  const g = c.getContext('2d');
  g.font = '700 60px ui-monospace, Consolas, monospace';
  g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillStyle = color;
  g.fillText(text, 128, 50);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthWrite: false }));
  s.position.set(x, y, z);
  s.scale.set(1.3 * size, 0.49 * size, 1);
  parent.add(s);
  return s;
}

// ---------- floor ----------
{
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(220, 220), mat(0x1d2127, { roughness: 0.95, metalness: 0 }));
  floor.receiveShadow = true;
  W.add(floor);
  const grid = new THREE.GridHelper(80, 80, 0x39404b, 0x272c34);
  grid.rotation.x = PI / 2;
  grid.position.set(xMid - 2, yMid, 0.003);
  W.add(grid);
}

// ---------- racks (spec §3.3): one group per row so a row can be cut away ----------
const xRack0 = bayX(1) - RACK.pitchX / 2, rackLen = RACK.bays * RACK.pitchX;
const rowGroups = {}; // row letter -> { g, y }
for (const A of LAYOUT.aisles)
  for (const [row, side] of A.rows) {
    const g = group(W, 0, A.y, 0);
    rowGroups[row] = { g, y: A.y + side * RACK.yStore };
    const up = [], ledge = [];
    for (let k = 0; k <= RACK.bays; k++) {
      const x = xRack0 + k * RACK.pitchX;
      for (const y of [RACK.yFace + 0.05, yBack - 0.05]) up.push([RACK.uprightW, 0.10, RACK.height, x, side * y, RACK.height / 2]);
      for (let l = 0; l <= RACK.levels; l++) up.push([0.05, RACK.depth - 0.1, 0.05, x, side * (RACK.yFace + RACK.depth / 2), RACK.z1 + l * RACK.pitchZ - 0.30]);
    }
    for (let l = 1; l <= RACK.levels + 1; l++) up.push([rackLen + RACK.uprightW, 0.05, 0.08, xRack0 + rackLen / 2, side * (yBack + 0.03), RACK.z1 + (l - 1) * RACK.pitchZ + 0.45]);
    const half = (RACK.pitchX - RACK.uprightW) / 2;
    for (let b = 1; b <= RACK.bays; b++)
      for (let l = 1; l <= RACK.levels; l++)
        for (const e of [1, -1]) {
          const x = bayX(b), z = levelZ(l), y = side * RACK.yStore;
          ledge.push([RACK.ledgeW, LOAD.sy, 0.008, x + e * (half - RACK.ledgeW / 2), y, z - 0.004]);
          ledge.push([0.008, LOAD.sy, RACK.ledgeLegH, x + e * (half - 0.004), y, z - RACK.ledgeLegH / 2]);
        }
    boxes(g, up, M.upright);
    boxes(g, ledge, M.ledge);
    label(W, `ROW ${row}`, xRack0 - 0.2, A.y + side * (RACK.yFace + RACK.depth / 2), RACK.height + 0.6);
  }

// ---------- per-aisle structure: rails, top ties, stations, conveyor lanes, crane ----------
const rollerGeo = cylGeo(0.04, LOAD.sy - 0.1, 'y');
const laneX0 = PD.xPort - 0.55, laneX1 = PD.x - LOAD.sx / 2 - 0.06;
const cranes = [];
const decks = [];
const portPads = {};

function buildCrane(parent) {
  const C = CRANE;
  const travel = group(parent); // q1 along X
  const lift = group(travel); // q2 along Z
  const forkMid = group(lift); // q3/2 along Y
  const forkTop = group(lift); // q3 along Y
  box(travel, C.chassisLen, 0.50, C.chassisH - 0.08, M.crane, 0, 0, 0.08 + (C.chassisH - 0.08) / 2);
  for (const e of [1, -1]) {
    box(travel, 0.5, C.chassisW, C.chassisH - 0.06, M.dark, e * 1.25, 0, 0.06 + (C.chassisH - 0.06) / 2);
    cyl(travel, 0.13, 0.12, 'y', M.steel, e * 1.25, 0, 0.13);
    box(travel, C.mastSx, C.mastSy, C.mastH - C.chassisH, M.crane, e * C.mastX, 0, (C.mastH + C.chassisH) / 2);
    box(travel, 0.08, C.mastSy + 0.04, C.mastH - C.chassisH - 0.6, M.steel, e * (C.mastX - C.mastSx / 2 - 0.04), 0, (C.mastH + C.chassisH) / 2);
  }
  box(travel, 2 * C.mastX + C.mastSx, C.mastSy, 0.22, M.crane, 0, 0, C.mastH - 0.11);
  box(travel, 0.3, 0.3, 0.14, M.dark, 0, 0, C.mastH + 0.07); // top guide rollers
  box(travel, 0.42, 0.7, 1.3, M.dark, -1.27, 0, C.chassisH + 0.65); // control cabinet
  box(travel, 0.42, 0.6, 0.5, M.steel, 1.27, 0, C.chassisH + 0.25); // travel drive
  cyl(travel, 0.2, 0.5, 'y', M.dark, 0.35, 0, C.mastH + 0.08); // hoist drum

  // lift carriage: origin on the fork load plane (z = q2)
  box(lift, C.carriageSx, C.carriageSy, C.carriageDrop - FORK.thick, M.crane, 0, 0, -(C.carriageDrop + FORK.thick) / 2);
  for (const e of [1, -1]) {
    box(lift, 0.06, 0.55, 1.15, M.crane, e * (C.carriageSx / 2 - 0.03), 0, -C.carriageDrop + 0.575);
    box(lift, 0.10, 0.50, 0.16, M.dark, e * (C.carriageSx / 2 + 0.03), 0, 0.85);
    box(lift, 0.10, 0.50, 0.16, M.dark, e * (C.carriageSx / 2 + 0.03), 0, -0.10);
    // telescopic fork: base on the carriage, middle and top stages slide (spec §6.1)
    box(lift, FORK.tineW, FORK.length, 0.025, M.dark, e * FORK.tineX, 0, -0.0575);
    box(forkMid, FORK.tineW - 0.03, FORK.length, 0.025, M.forkMid, e * FORK.tineX, 0, -0.0325);
    box(forkTop, FORK.tineW, FORK.length, 0.020, M.fork, e * FORK.tineX, 0, -0.010);
  }
  const ropes = [1, -1].map((e) => box(travel, 0.025, 0.025, 1, M.dark, e * (C.carriageSx / 2 - 0.03), 0, 0));
  const target = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.BoxGeometry(RACK.pitchX - RACK.uprightW, LOAD.sy + 0.06, LOAD.maxH + 0.1)),
    new THREE.LineBasicMaterial({ color: 0x3ecf8e }),
  );
  W.add(target);
  return { travel, lift, forkMid, forkTop, ropes, target };
}

for (const A of LAYOUT.aisles) {
  const g = group(W, 0, A.y, 0);
  // top ties across the aisle carry the upper guide rail
  const ties = [], sides = A.rows.map((r) => r[1]);
  for (let k = 0; k <= RACK.bays; k += 4) {
    const x = xRack0 + k * RACK.pitchX;
    ties.push([0.10, 2 * yBack, 0.12, x, 0, RACK.height + 0.06]);
    for (const s of [1, -1]) if (!sides.includes(s)) ties.push([0.10, 0.10, RACK.height, x, s * (yBack - 0.05), RACK.height / 2]);
  }
  ties.push([0.10, 2 * yBack, 0.12, -1.4, 0, RACK.height + 0.06]);
  for (const s of [1, -1]) ties.push([0.10, 0.10, RACK.height, -1.4, s * (yBack - 0.05), RACK.height / 2]);
  boxes(g, ties, M.conv);
  const x0 = -2.0, x1 = AX[0].max + 2.0;
  box(g, x1 - x0, 0.08, 0.12, M.rail, (x0 + x1) / 2, 0, RACK.height - 0.06); // top guide rail
  box(g, x1 - x0, 0.07, 0.06, M.rail, (x0 + x1) / 2, 0, 0.03); // floor rail
  for (const x of [x0, x1]) box(g, 0.25, 0.5, 0.6, M.dark, x, 0, 0.3); // end buffers
  label(W, `AISLE ${A.id}`, -1.4, A.y, RACK.height + 0.6, '#f2b01b');

  // stations and lanes (spec §3.5)
  for (const [side, kind, col] of [[PD.inSide, 'in', '#3ecf8e'], [PD.outSide, 'out', '#f0b429']]) {
    const y = side * RACK.yStore, frame = [];
    for (const e of [1, -1]) {
      frame.push([PD.ledgeW, LOAD.sy, 0.10, PD.x + e * PD.ledgeX, y, PD.z - 0.05]); // fixed support ledges
      for (const dy of [-0.5, 0.5]) frame.push([0.06, 0.06, PD.z - 0.1, PD.x + e * PD.ledgeX, y + dy, (PD.z - 0.1) / 2]);
      frame.push([laneX1 - laneX0, 0.05, 0.14, (laneX0 + laneX1) / 2, y + e * (LOAD.sy / 2 - 0.01), PD.z - 0.07]); // lane side frames
    }
    for (let x = laneX0 + 0.2; x < laneX1; x += 1.2) for (const e of [1, -1]) frame.push([0.06, 0.06, PD.z - 0.14, x, y + e * (LOAD.sy / 2 - 0.01), (PD.z - 0.14) / 2]);
    boxes(g, frame, M.conv);
    const n = Math.floor((laneX1 - laneX0 - 0.1) / 0.2), rollers = new THREE.InstancedMesh(rollerGeo, M.steel, n);
    for (let i = 0; i < n; i++) rollers.setMatrixAt(i, m4.makeTranslation(laneX0 + 0.1 + i * 0.2, y, PD.z + PD.deckLift - 0.04));
    rollers.castShadow = rollers.receiveShadow = true;
    g.add(rollers);
    // roller deck between the station ledges: origin on the roller tops
    const deck = group(g, PD.x, y, PD.z);
    for (let i = -2; i <= 2; i++) cyl(deck, 0.04, LOAD.sy - 0.1, 'y', M.steel, i * 0.12, 0, -0.04);
    box(deck, 0.58, LOAD.sy - 0.2, 0.04, M.dark, 0, 0, -0.10);
    decks.push({ deck, aisle: A.id, kind });
    // port: where the neighbouring cell hands a pallet in or takes boxes out
    const pad = new THREE.Mesh(new THREE.PlaneGeometry(LOAD.sx + 0.5, LOAD.sy + 0.4), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.16 }));
    pad.position.set(PD.xPort, A.y + y, 0.006);
    W.add(pad);
    portPads[`${kind.toUpperCase()}-${A.id}`] = pad;
    label(W, `${kind.toUpperCase()}-${A.id}`, PD.xPort - 1.3, A.y + y, 0.5, col, 0.8);
  }
  cranes.push(buildCrane(g));
}

// TCP marker and trail of the selected crane
const tcpDot = add(W, new THREE.SphereGeometry(0.035, 16, 12), new THREE.MeshBasicMaterial({ color: 0x3ecf8e }));
tcpDot.castShadow = false;
const TRAIL_N = 1500;
const trailPos = new Float32Array(TRAIL_N * 3);
const trail = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x3ecf8e, transparent: true, opacity: 0.75 }));
trail.geometry.setAttribute('position', new THREE.BufferAttribute(trailPos, 3));
trail.frustumCulled = false;
W.add(trail);
let trailLen = 0;

// ---------- unit loads: every pallet and every box is one instance ----------
const palletGeo = (() => {
  const parts = [];
  const b = (sx, sy, sz, x, y, z) => parts.push(new THREE.BoxGeometry(sx, sy, sz).translate(x, y, z));
  b(LOAD.sx, LOAD.sy, 0.022, 0, 0, LOAD.palletH - 0.011); // top deck
  for (const x of [-0.35, 0, 0.35]) b(0.10, LOAD.sy, 0.10, x, 0, 0.072); // stringers along the fork direction
  for (const y of [-0.55, 0, 0.55]) b(LOAD.sx, 0.10, 0.022, 0, y, 0.011); // bottom boards
  return mergeGeometries(parts);
})();
const CAP_P = 900, CAP_B = 60000;
const far = new THREE.Sphere(new THREE.Vector3(xMid, yMid, 6), 200);
const palletMesh = new THREE.InstancedMesh(palletGeo, M.wood, CAP_P);
const boxMesh = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), M.box, CAP_B);
for (const im of [palletMesh, boxMesh]) {
  im.castShadow = im.receiveShadow = true;
  im.frustumCulled = false;
  im.boundingSphere = far;
  im.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  W.add(im);
}
const ZERO = new THREE.Matrix4().makeScale(0, 0, 0);
const skuColor = SKUS.map((s) => new THREE.Color(s.color));
const tint = new THREE.Color();
let freeP, freeB, views, boxOwner; // free instance indices, pallet -> view, box instance -> pallet

function addPalletView(p) {
  if (!freeP.length || freeB.length < p.boxes.length) return; // pool exhausted: not drawn
  const v = { pi: freeP.pop(), bi: new Map(), x: NaN, y: 0, z: 0, hidden: false };
  for (const b of p.boxes) {
    const i = freeB.pop();
    v.bi.set(b, i);
    boxOwner[i] = p;
    // slight per-box shade so the stack reads as separate cartons
    const k = 0.9 + 0.1 * (((p.id * 31 + i * 17) % 11) / 10);
    boxMesh.setColorAt(i, tint.copy(skuColor[p.skuIndex]).multiplyScalar(k));
  }
  boxMesh.instanceColor.needsUpdate = true;
  views.set(p, v);
}
function freeBox(p, b) {
  const v = views.get(p), i = v?.bi.get(b);
  if (i === undefined) return;
  boxMesh.setMatrixAt(i, ZERO);
  boxMesh.instanceMatrix.needsUpdate = true;
  v.bi.delete(b);
  boxOwner[i] = null;
  freeB.push(i);
}
function removePalletView(p) {
  const v = views.get(p);
  if (!v) return;
  for (const b of [...v.bi.keys()]) freeBox(p, b);
  palletMesh.setMatrixAt(v.pi, ZERO);
  palletMesh.instanceMatrix.needsUpdate = true;
  freeP.push(v.pi);
  views.delete(p);
}
function drawPallet(p, v) {
  if (v.hidden) {
    palletMesh.setMatrixAt(v.pi, ZERO);
    for (const i of v.bi.values()) boxMesh.setMatrixAt(i, ZERO);
  } else {
    palletMesh.setMatrixAt(v.pi, m4.makeTranslation(p.x, p.y, p.z));
    for (const [b, i] of v.bi) boxMesh.setMatrixAt(i, m4.makeScale(b.sx - 0.006, b.sy - 0.006, b.sz - 0.004).setPosition(p.x + b.lx, p.y + b.ly, p.z + b.lz));
  }
  palletMesh.instanceMatrix.needsUpdate = boxMesh.instanceMatrix.needsUpdate = true;
}

let wh;
function reset() {
  wh = new Warehouse({ seed: 7 });
  freeP = Array.from({ length: CAP_P }, (_, i) => CAP_P - 1 - i);
  freeB = Array.from({ length: CAP_B }, (_, i) => CAP_B - 1 - i);
  views = new Map();
  boxOwner = new Array(CAP_B).fill(null);
  for (let i = 0; i < CAP_P; i++) palletMesh.setMatrixAt(i, ZERO);
  for (let i = 0; i < CAP_B; i++) { boxMesh.setMatrixAt(i, ZERO); boxMesh.setColorAt(i, tint.set(0xffffff)); }
  for (const p of wh.pallets) addPalletView(p);
  wh.on('palletCreated', addPalletView);
  wh.on('boxTaken', ({ pallet, box: b }) => freeBox(pallet, b));
  wh.on('palletReleased', ({ pallet }) => removePalletView(pallet));
  trailLen = 0;
  acc = 0;
}

// ---------- HUD ----------
const $ = (id) => document.getElementById(id);
let sel = 0; // selected crane index
const craneRows = LAYOUT.aisles.map((A, i) => {
  const row = document.createElement('div');
  row.className = 'crane';
  row.innerHTML = `<span class="lamp on"></span><b>${A.id}</b><span class="rw">${A.rows.map((r) => r[0]).join('·')}</span><span class="ph"></span><span class="cm"></span>`;
  row.onclick = () => selectCrane(i);
  $('cranes').appendChild(row);
  return { row, lamp: row.querySelector('.lamp'), ph: row.querySelector('.ph'), cm: row.querySelector('.cm') };
});
const axisRows = AX.map((a) => {
  const row = document.createElement('div');
  row.className = 'j';
  row.innerHTML = `<span>${a.id}</span><span class="nm">${a.name}</span><span class="val"></span><div class="bar pos"><i></i></div><div class="bar"><i></i></div>`;
  $('axes').appendChild(row);
  return { val: row.querySelector('.val'), pos: row.querySelectorAll('i')[0], spd: row.querySelectorAll('i')[1] };
});
const portRows = LAYOUT.aisles.map((A) => {
  const row = document.createElement('div');
  row.className = 'row';
  row.innerHTML = `<span>OUT-${A.id}</span><span></span>`;
  $('ports').appendChild(row);
  return row.lastChild;
});
const lamp = (id, on, warn = false) => { $(id).className = 'lamp ' + (on ? (warn ? 'warn' : 'on') : ''); };
const cmdText = (a) => (a.op ? `${a.op.kind === 'store' ? 'IN → ' + a.op.slot.id : a.op.slot.id + ' → OUT'}` : '');

function hud() {
  wh.aisles.forEach((a, i) => {
    const r = craneRows[i];
    r.row.classList.toggle('sel', i === sel);
    r.lamp.className = 'lamp ' + (a.lastViolations.length ? 'bad' : 'on');
    r.ph.textContent = a.phase.replace(/ → .*/, '');
    r.cm.textContent = cmdText(a);
  });
  const a = wh.aisles[sel], { q, qd } = a;
  $('selName').textContent = `Crane ${a.id}`;
  const load = a.carried ?? a.engaged?.pallet;
  $('load').textContent = load ? `#${load.id} ${load.sku} · ${load.boxes.length} boxes · ${load.mass} kg` : 'none';
  AX.forEach((ax, i) => {
    const r = axisRows[i];
    r.val.textContent = (q[i] >= 0 ? ' ' : '') + q[i].toFixed(3);
    r.pos.style.left = `calc(${(100 * (q[i] - ax.min)) / (ax.max - ax.min)}% - 1.5px)`;
    r.spd.style.width = `${(100 * Math.abs(qd[i])) / ax.v}%`;
  });
  const centred = a.forkCentred;
  lamp('lCentre', centred);
  lamp('lFork', !centred, true);
  lamp('lLoad', !!a.carried);
  lamp('lDeck', a.stIn.deck === 0 && a.stOut.deck === 0);
  const bad = a.lastViolations.length > 0;
  $('lMon').className = 'lamp ' + (bad ? 'bad' : 'on');
  $('mon').textContent = bad ? a.lastViolations[0] : `OK  (${a.stats.violations} violations)`;
  const m = load ? load.mass : 0, p = fk(q, 0);
  $('tcp').textContent = `${p[0].toFixed(3)}  ${(a.y + p[1]).toFixed(3)}  ${a.forkPlane().toFixed(3)}`;
  $('sag').textContent = `${(1000 * (q[1] - a.forkPlane())).toFixed(1)} mm · load on fork ${(100 * a.loadFraction()).toFixed(0)} %`;
  $('det').textContent = det3(jacobian(q, m)).toFixed(3);

  wh.outputs().forEach((o, i) => {
    portRows[i].textContent = o.pallet ? `#${o.pallet.id} ${o.pallet.sku} · ${o.pallet.boxesLeft} boxes left · +${o.queued} queued` : `empty · ${o.queued} on the way`;
    portPads[o.port].material.opacity = o.pallet ? 0.45 : 0.16;
  });
  const st = wh.stats, occ = wh.slots.filter((s) => s.pallet).length;
  $('count').textContent = `${st.stored} / ${st.retrieved}`;
  $('fill').textContent = `${occ} / ${wh.slots.length}`;
  $('boxes').textContent = `${wh.counts.boxesIn - wh.counts.boxesOut} in system · ${wh.counts.boxesOut} handed over`;
  $('thr').textContent = wh.t > 120 ? `${(((st.stored + st.retrieved) * 3600) / wh.t).toFixed(0)} pallets/h` : '–';
  const s = Math.floor(wh.t);
  $('time').textContent = `${String(Math.floor(s / 3600)).padStart(2, '0')}:${String(Math.floor(s / 60) % 60).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
}

// ---------- scene update from sim state ----------
let showTrail = true, cutaway = true, view = 'overview';
const hiddenRows = new Set();
function applyCutaway() {
  // in the aisle views, remove the rows standing between the camera (on the -Y side) and the aisle
  hiddenRows.clear();
  if (cutaway && (view === 'aisle' || view === 'follow')) {
    const ya = LAYOUT.aisles[sel].y;
    for (const [row, r] of Object.entries(rowGroups)) if (r.y < ya) hiddenRows.add(row);
  }
  for (const [row, r] of Object.entries(rowGroups)) r.g.visible = !hiddenRows.has(row);
  const cut = cutaway && (view === 'aisle' || view === 'follow');
  cranes.forEach((c, i) => { c.travel.visible = !(cut && LAYOUT.aisles[i].y < LAYOUT.aisles[sel].y); }); // cranes in nearer aisles
  for (const v of views.values()) v.x = NaN; // force redraw
}
function sync() {
  wh.aisles.forEach((a, i) => {
    const c = cranes[i], { q } = a, sag = q[1] - a.forkPlane();
    c.travel.position.x = q[0];
    c.lift.position.z = q[1];
    c.forkMid.position.set(0, q[2] * FORK.midRatio, -sag / 8);
    c.forkTop.position.set(0, q[2], -sag);
    const zTop = q[1] - CRANE.carriageDrop + 1.15, len = CRANE.mastH - 0.2 - zTop;
    for (const r of c.ropes) { r.scale.z = len; r.position.z = zTop + len / 2; }
    const slot = a.op?.slot;
    c.target.visible = !!slot && !hiddenRows.has(slot.row);
    if (slot) c.target.position.set(slot.x, slot.y, slot.zs + (LOAD.maxH + 0.1) / 2 - 0.02);
  });
  for (const d of decks) {
    const a = wh.aisles[d.aisle - 1], st = d.kind === 'in' ? a.stIn : a.stOut;
    d.deck.position.z = PD.z - PD.deckDrop + st.deck * (PD.deckDrop + PD.deckLift);
  }
  for (const [p, v] of views) {
    const hidden = !!p.slot && hiddenRows.has(p.slot.row);
    if (p.x === v.x && p.y === v.y && p.z === v.z && hidden === v.hidden) continue;
    v.x = p.x; v.y = p.y; v.z = p.z; v.hidden = hidden;
    drawPallet(p, v);
  }

  const a = wh.aisles[sel], tip = [a.q[0], a.y + a.q[2], a.forkPlane()];
  tcpDot.position.set(...tip);
  if (showTrail) {
    const n = Math.min(trailLen, TRAIL_N - 1);
    trailPos.copyWithin(3, 0, n * 3);
    trailPos.set(tip, 0);
    trailLen = n + 1;
    trail.geometry.setDrawRange(0, trailLen);
    trail.geometry.attributes.position.needsUpdate = true;
  }
  trail.visible = showTrail;
}

// ---------- cameras (three coordinates: x, up, -worldY) ----------
const zMid = -yMid, zNear = -Math.min(...yAll) + yBack; // near edge of the installation, seen from -Y
const VIEWS = {
  overview: () => [[-20, 17, zNear + 19], [xMid - 4.5, 4.4, zMid - 1]],
  front: () => [[-21, 8.5, zMid + 0.01], [xMid, 6.2, zMid]],
  ports: () => [[-13.5, 6.5, zNear + 5.5], [-2.2, 0.6, zMid + 0.6]],
  aisle: () => [[xMid - 9, 9, -LAYOUT.aisles[sel].y + 21], [xMid - 1, 6.0, -LAYOUT.aisles[sel].y]],
};
function setView(name) {
  view = name;
  document.querySelectorAll('[data-view]').forEach((b) => b.classList.toggle('on', b.dataset.view === name));
  applyCutaway();
  if (name === 'follow') return;
  const [p, t] = VIEWS[name]();
  camera.position.set(...p);
  controls.target.set(...t);
}
function selectCrane(i) {
  sel = i;
  trailLen = 0;
  if (view === 'aisle' || view === 'follow') setView(view);
}
const followOff = new THREE.Vector3(-5.2, 1.8, 6.4);
function followCam() {
  const a = wh.aisles[sel], t = new THREE.Vector3(a.q[0], a.q[1] + 0.5, -a.y);
  controls.target.lerp(t, 0.12);
  camera.position.lerp(t.clone().add(followOff), 0.06);
}

// ---------- UI ----------
let paused = false, speed = 1, acc = 0;
$('pause').onclick = (e) => { paused = !paused; e.target.textContent = paused ? 'Resume' : 'Pause'; e.target.classList.toggle('on', paused); };
$('reset').onclick = () => { reset(); applyCutaway(); };
document.querySelectorAll('[data-speed]').forEach((b) => (b.onclick = () => {
  speed = +b.dataset.speed;
  document.querySelectorAll('[data-speed]').forEach((x) => x.classList.toggle('on', x === b));
}));
document.querySelectorAll('[data-view]').forEach((b) => (b.onclick = () => setView(b.dataset.view)));
$('panel').onclick = (e) => { $('hud').hidden = !$('hud').hidden; e.target.classList.toggle('on', !$('hud').hidden); };
$('cut').onclick = (e) => { cutaway = !cutaway; e.target.classList.toggle('on', cutaway); applyCutaway(); };
$('trail').onclick = (e) => { showTrail = !showTrail; e.target.classList.toggle('on', showTrail); trailLen = 0; };

// click a stored load to request its retrieval
const ray = new THREE.Raycaster(), mouse = new THREE.Vector2();
let downAt = null;
renderer.domElement.addEventListener('pointerdown', (e) => { downAt = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 4) return;
  mouse.set((e.clientX / innerWidth) * 2 - 1, -(e.clientY / innerHeight) * 2 + 1);
  ray.setFromCamera(mouse, camera);
  const hit = ray.intersectObject(boxMesh, false)[0];
  const p = hit && boxOwner[hit.instanceId];
  if (!p?.slot) return;
  const id = wh.requestRetrieval({ slotId: p.slot.id });
  $('req').textContent = id ? `retrieval requested: ${id} (pallet #${p.id}, ${p.sku}, ${p.boxes.length} boxes)` : `${p.slot.id} is already on order`;
});

function resize() {
  renderer.setSize(innerWidth, innerHeight);
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);
resize();

// ---------- main loop: fixed 240 Hz simulation step ----------
const DT = 1 / 240;
let last = performance.now();
reset();
setView('overview');
// Integration handle: window.asrs.wh is the Warehouse (see ASRS_SPEC.md §13).
window.asrs = { get wh() { return wh; }, setView, selectCrane, camera, controls };

function frame(now) {
  const wall = Math.min((now - last) / 1000, 0.1);
  last = now;
  if (!paused) {
    acc += wall * speed;
    let n = 0;
    while (acc >= DT && n < 1200) { wh.step(DT); acc -= DT; n++; }
    if (n === 1200) acc = 0;
  }
  sync();
  hud();
  if (view === 'follow') followCam();
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
