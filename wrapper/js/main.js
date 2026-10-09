// three.js view of the turntable wrapper cell. All geometry lives in `W`, a Z-up group, so
// coordinates here are identical to WRAPPER_SPEC.md and sim.js.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { PALLET, SKUS, LINE, TABLE, CARRIAGE, FILM, WRAP, CLAMP, CUT, DEG, RPM } from './config.js';
import * as K from './kinematics.js';
import { Cell, zoneX } from './sim.js';

const PI = Math.PI;
const [EX, EY] = CARRIAGE.roller;

// ---------- renderer / scene ----------
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
document.body.prepend(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14171c);
scene.fog = new THREE.Fog(0x14171c, 30, 90);

const camera = new THREE.PerspectiveCamera(40, 1, 0.05, 300);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.maxPolarAngle = PI / 2 - 0.02;

scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x2a2622, 1.3));
const sun = new THREE.DirectionalLight(0xffffff, 2.1);
sun.position.set(5, 16, 11);
sun.castShadow = true;
sun.shadow.mapSize.set(4096, 4096);
Object.assign(sun.shadow.camera, { left: -10, right: 10, top: 8, bottom: -8, near: 1, far: 50 });
sun.shadow.bias = -0.0005;
scene.add(sun, sun.target);

const W = new THREE.Group(); // world frame {W}: Z up
W.rotation.x = -PI / 2;
scene.add(W);

// ---------- helpers (all in Z-up coordinates) ----------
const mat = (color, o = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.2, ...o });
const M = {
  body: mat(0x2f6fb5, { roughness: 0.45 }),
  dark: mat(0x23272e, { roughness: 0.6, metalness: 0.4 }),
  steel: mat(0x9aa3b0, { roughness: 0.35, metalness: 0.7 }),
  roller: mat(0xb8bfc9, { roughness: 0.3, metalness: 0.8 }),
  rubber: mat(0x16181c, { roughness: 0.9, metalness: 0 }),
  conv: mat(0x4a5361, { metalness: 0.5 }),
  yellow: mat(0xf2b01b, { roughness: 0.5 }),
  wood: mat(0xb98a55, { roughness: 0.85, metalness: 0 }),
  box: mat(0xffffff, { roughness: 0.88, metalness: 0 }),
  brush: mat(0x7a5a2c, { roughness: 1, metalness: 0 }),
  core: mat(0x8a6b45, { roughness: 0.9, metalness: 0 }),
  filmRoll: mat(0xdfeaf5, { roughness: 0.15, metalness: 0.1, transparent: true, opacity: 0.85 }),
  film: new THREE.MeshStandardMaterial({ color: 0xc4e0ff, roughness: 0.12, metalness: 0, transparent: true, opacity: 0.26, side: THREE.DoubleSide, depthWrite: false }),
};
function add(parent, geo, material, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(geo, material);
  m.position.set(x, y, z);
  m.castShadow = m.receiveShadow = true;
  parent.add(m);
  return m;
}
const slab = (p, x0, x1, y0, y1, z0, z1, m) => add(p, new THREE.BoxGeometry(x1 - x0, y1 - y0, z1 - z0), m, (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2);
function cylGeo(r, len, axis, seg = 28) {
  const g = new THREE.CylinderGeometry(r, r, len, seg);
  if (axis === 'x') g.rotateZ(-PI / 2);
  if (axis === 'z') g.rotateX(PI / 2);
  return g;
}
const group = (parent, x = 0, y = 0, z = 0) => {
  const g = new THREE.Group();
  g.position.set(x, y, z);
  parent.add(g);
  return g;
};
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
  s.scale.set(1.0 * size, 0.375 * size, 1);
  parent.add(s);
  return s;
}
function polyline(parent, pts, color, opacity = 1) {
  const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts.map((p) => new THREE.Vector3(...p))), new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity }));
  parent.add(l);
  return l;
}
const circle = (r, z, n = 96) => Array.from({ length: n + 1 }, (_, i) => [r * Math.cos((2 * PI * i) / n), r * Math.sin((2 * PI * i) / n), z]);

// ---------- floor ----------
{
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(160, 160), mat(0x1d2127, { roughness: 0.95, metalness: 0 }));
  floor.receiveShadow = true;
  W.add(floor);
  const grid = new THREE.GridHelper(40, 40, 0x39404b, 0x272c34);
  grid.rotation.x = PI / 2;
  grid.position.z = 0.003;
  W.add(grid);
}
const sweptGroup = group(W);
polyline(sweptGroup, circle(TABLE.rSwept, 0.006), 0xf2b01b, 0.8);
polyline(sweptGroup, circle(Math.hypot(PALLET.sx, PALLET.sy) / 2, LINE.z + 0.002), 0xef4444, 0.6);

// ---------- line conveyors (spec §3.1) ----------
const rollerGeo = mergeGeometries([cylGeo(LINE.rollR, LINE.rollLen, 'y', 20)]);
const stripeGeo = new THREE.BoxGeometry(0.014, LINE.rollLen - 0.06, 0.004).translate(0, 0, LINE.rollR);
const lineRollers = []; // { x, zone }
for (const sgn of [-1, 1]) {
  const x0 = sgn < 0 ? -LINE.xFar : LINE.xEnd, x1 = sgn < 0 ? -LINE.xEnd : LINE.xFar, hw = LINE.frameW / 2, z = LINE.z;
  for (const e of [1, -1]) {
    slab(W, x0, x1, e * hw - 0.03, e * hw + 0.03, z - 0.14, z + 0.01, M.conv); // side beams
    slab(W, x0, x1, e * (hw - 0.03) - 0.006, e * (hw - 0.03) + 0.006, z + 0.01, z + 0.09, M.steel); // side guides
    for (let x = x0 + 0.15; x < x1; x += 1.3) slab(W, x - 0.04, x + 0.04, e * hw - 0.04, e * hw + 0.04, 0, z - 0.14, M.conv); // legs
  }
  for (let x = x0 + 0.15; x < x1; x += 1.3) slab(W, x - 0.03, x + 0.03, -hw, hw, z - 0.30, z - 0.22, M.conv);
  slab(W, sgn < 0 ? x0 + 0.2 : x1 - 0.6, sgn < 0 ? x0 + 0.6 : x1 - 0.2, hw + 0.03, hw + 0.30, z - 0.32, z - 0.04, M.dark); // drive
  const n = Math.floor((x1 - x0 - 0.08) / LINE.rollPitch);
  for (let i = 0; i <= n; i++) {
    const x = sgn < 0 ? x1 - 0.04 - i * LINE.rollPitch : x0 + 0.04 + i * LINE.rollPitch;
    const zone = Math.round(x / LINE.pitch) + 3;
    lineRollers.push({ x, zone: sgn < 0 ? Math.min(2, Math.max(0, zone)) : Math.max(4, Math.min(6, zone)) });
  }
}
const rollMesh = new THREE.InstancedMesh(rollerGeo, M.roller, lineRollers.length);
const stripeMesh = new THREE.InstancedMesh(stripeGeo, M.dark, lineRollers.length);
for (const im of [rollMesh, stripeMesh]) { im.castShadow = im.receiveShadow = true; im.frustumCulled = false; im.instanceMatrix.setUsage(THREE.DynamicDrawUsage); W.add(im); }
LINE.zones.forEach((id, i) => { if (i !== 3) label(W, id, zoneX(i), 0.95, 0.25, '#8a94a3', 0.7); });

// photo-eyes: two posts and a beam across the conveyor
function eye(x, z, y = LINE.frameW / 2 + 0.07, h = z + 0.05) {
  for (const e of [1, -1]) slab(W, x - 0.02, x + 0.02, e * y - 0.02, e * y + 0.02, 0, h, M.dark);
  return polyline(W, [[x, -y, z], [x, y, z]], 0x343a44).material;
}
const EYES = {
  b1: eye(zoneX(2) + PALLET.sx / 2 - 0.05, LINE.z + 0.06),
  b2: eye(-(LINE.xEnd + TABLE.deckX + LINE.rollR) / 2 - 0.02, LINE.z + 0.06, 0.75),
  b4: eye((LINE.xEnd + TABLE.deckX + LINE.rollR) / 2 + 0.02, LINE.z + 0.06, 0.75),
  b5: eye(zoneX(4) + PALLET.sx / 2 - 0.05, LINE.z + 0.06),
};
// BH: height light grid at I1
for (const e of [1, -1]) slab(W, zoneX(2) - 0.03, zoneX(2) + 0.03, e * 0.66 - 0.03, e * 0.66 + 0.03, 0, LINE.z + PALLET.maxH + 0.15, M.yellow);
const gridBeams = Array.from({ length: 14 }, (_, i) => { const z = LINE.z + 0.2 + i * 0.14; return { z, m: polyline(W, [[zoneX(2), -0.66, z], [zoneX(2), 0.66, z]], 0x343a44, 0.7).material }; });

// ---------- turntable (spec §3.2, §3.4) ----------
add(W, cylGeo(0.55, 0.30, 'z', 40), M.dark, 0, 0, 0.17); // pedestal with the slewing ring
add(W, cylGeo(0.62, 0.04, 'z', 40), M.steel, 0, 0, 0.34);
const table = group(W); // q1 about Z
add(table, cylGeo(TABLE.rPlate, 0.04, 'z', 64), M.body, 0, 0, TABLE.plateZ - 0.02);
slab(table, TABLE.rPlate - 0.10, TABLE.rPlate - 0.01, -0.03, 0.03, TABLE.plateZ, TABLE.plateZ + 0.004, M.yellow); // index mark
for (const e of [1, -1]) slab(table, -0.74, 0.74, e * 0.50 - 0.02, e * 0.50 + 0.02, TABLE.plateZ, TABLE.z - 0.02, M.conv);
const deckRollers = [];
for (let x = -TABLE.deckX; x <= TABLE.deckX + 1e-9; x += LINE.rollPitch) {
  const len = Math.min(LINE.rollLen, 2 * Math.sqrt(0.88 ** 2 - x * x) - 0.04), g = group(table, x, 0, TABLE.z - LINE.rollR);
  add(g, cylGeo(LINE.rollR, len, 'y', 20), M.roller);
  add(g, new THREE.BoxGeometry(0.014, len - 0.06, 0.004), M.dark, 0, 0, LINE.rollR);
  deckRollers.push(g);
}
// film clamp: lift g1, jaws g2. Local X is along the film line of the home pose.
const clampBase = group(table, CLAMP.x, CLAMP.y, 0);
clampBase.rotation.z = CLAMP.yaw;
slab(clampBase, -0.04, 0.04, -0.045, 0.045, TABLE.plateZ, CLAMP.zDown - 0.135, M.dark);
const clampLift = group(clampBase);
add(clampLift, cylGeo(0.016, 0.16, 'z', 12), M.steel, 0, 0, -0.20);
slab(clampLift, -0.035, 0.035, -0.04, 0.04, -0.135, -0.115, M.steel);
const jaws = [1, -1].map((e) => { const j = group(clampLift); slab(j, -0.03, 0.03, -0.006, 0.006, -0.125, 0, M.yellow); slab(j, -0.03, 0.03, e > 0 ? -0.008 : 0.006, e > 0 ? -0.006 : 0.008, -0.11, 0, M.rubber); return j; });

// ---------- mast and film carriage (spec §3.3) ----------
{
  const m = CARRIAGE.mast;
  slab(W, m.x - m.sx / 2, m.x + m.sx / 2, m.y - m.sy / 2, m.y + m.sy / 2, 0, m.h, M.body);
  slab(W, m.x - 0.35, m.x + 0.35, m.y - 0.30, m.y + 0.25, 0, 0.03, M.dark);
  slab(W, m.x - 0.14, m.x + 0.14, m.y - 0.16, m.y + 0.16, m.h, m.h + 0.16, M.dark); // lift drive
  slab(W, m.x - 0.02, m.x + 0.02, m.y + m.sy / 2, m.y + m.sy / 2 + 0.012, 0.25, m.h - 0.05, M.steel); // guide rail
}
const carriage = group(W, EX, EY, 0); // q3 along Z; origin on the exit roller axis at the film centreline
const B = CARRIAGE.body, bx0 = B.x0 - EX, bx1 = B.x1 - EX, by0 = B.y0 - EY, by1 = B.y1 - EY, bh = B.h / 2;
slab(carriage, bx0, bx1, by0, by1, bh - 0.02, bh, M.body);
slab(carriage, bx0, bx1, by0, by1, -bh, -bh + 0.02, M.body);
slab(carriage, bx0, bx1, by0, by0 + 0.02, -bh, bh, M.body);
slab(carriage, CARRIAGE.mast.x - EX - 0.06, CARRIAGE.mast.x - EX + 0.06, CARRIAGE.mast.y + CARRIAGE.mast.sy / 2 - EY, by0, -0.12, 0.12, M.dark); // saddle on the mast
add(carriage, cylGeo(CARRIAGE.rExit, FILM.w0 + 0.04, 'z', 20), M.roller); // exit roller
const ROLL_POS = [-0.31, -0.12];
add(carriage, cylGeo(FILM.rCore, FILM.w0 + 0.03, 'z', 16), M.core, ROLL_POS[0], ROLL_POS[1], 0);
const filmRoll = add(carriage, cylGeo(1, FILM.w0, 'z', 40), M.filmRoll, ROLL_POS[0], ROLL_POS[1], 0);
const rollMark = add(carriage, new THREE.BoxGeometry(0.03, 0.006, 0.006), M.dark, 0, 0, FILM.w0 / 2 + 0.005);
const stretch = [[-0.15, -0.20], [-0.09, -0.10]].map(([x, y]) => { const g = group(carriage, x, y, 0); add(g, cylGeo(0.035, FILM.w0 + 0.02, 'z', 18), M.rubber); add(g, new THREE.BoxGeometry(0.05, 0.008, 0.006), M.yellow, 0, 0, FILM.w0 / 2 + 0.013); return g; });
const dancer = group(carriage, -0.03, -0.21, 0); // q5: arm swings about Z
slab(dancer, -0.012, 0.012, 0, FILM.dancer.arm, bh - 0.06, bh - 0.04, M.steel);
add(dancer, cylGeo(0.022, FILM.w0, 'z', 14), M.roller, 0, FILM.dancer.arm, 0);
const loadEye = add(carriage, new THREE.SphereGeometry(0.02, 12, 8), new THREE.MeshBasicMaterial({ color: 0x343a44 }), bx1 - 0.02, by1, FILM.w / 2);

// ---------- cut-and-press slide (spec §7.2) ----------
slab(W, CUT.x - 0.05, CUT.x + 0.05, -1.90, -1.80, 0, 0.74, M.dark);
slab(W, CUT.x - 0.035, CUT.x + 0.035, -2.35, CUT.park, 0.74, 0.77, M.steel); // rail
const slide = group(W, 0, CUT.park, 0); // g3 along Y; origin on the brush face
slab(slide, CUT.x - 0.02, CUT.x + 0.02, -1.02, -0.03, 0.77, 0.81, M.yellow);
slab(slide, CUT.x - CUT.brushW / 2, CUT.x + CUT.brushW / 2, -0.05, -0.025, CUT.z[0], CUT.z[1], M.dark);
slab(slide, CUT.x - CUT.brushW / 2, CUT.x + CUT.brushW / 2, -0.025, 0, CUT.z[0], CUT.z[1], M.brush);
for (const z of CUT.z) slab(slide, CUT.wireX - 0.006, CUT.x, -CUT.wireBack - 0.006, -CUT.wireBack + 0.006, z - 0.012, z + 0.012, M.dark);
slab(slide, CUT.x - 0.02, CUT.x - 0.008, -CUT.wireBack, -0.04, CUT.z[0], CUT.z[0] + 0.02, M.dark);
const wire = add(slide, cylGeo(0.003, CUT.z[1] - CUT.z[0], 'z', 8), new THREE.MeshBasicMaterial({ color: 0x70757d }), CUT.wireX, -CUT.wireBack, (CUT.z[0] + CUT.z[1]) / 2);
wire.castShadow = false;

// ---------- signal tower ----------
slab(W, -0.62, -0.58, -1.62, -1.58, 0, 2.0, M.dark);
const tower = [0xef4444, 0xf0b429, 0x3ecf8e, 0x4aa3ff].map((c, i) => ({ c, m: add(W, cylGeo(0.04, 0.09, 'z', 16), new THREE.MeshBasicMaterial({ color: 0x343a44 }), -0.60, -1.60, 2.35 - i * 0.1) }));

// ---------- film: free span, and the moment-arm construction ----------
function quadMesh(nQuads, material) {
  const g = new THREE.BufferGeometry(), idx = [];
  g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(nQuads * 12), 3));
  g.setAttribute('normal', new THREE.BufferAttribute(new Float32Array(nQuads * 12), 3));
  for (let i = 0; i < nQuads; i++) idx.push(4 * i, 4 * i + 1, 4 * i + 2, 4 * i, 4 * i + 2, 4 * i + 3);
  g.setIndex(idx);
  g.setDrawRange(0, 0);
  const m = new THREE.Mesh(g, material);
  m.frustumCulled = false;
  m.renderOrder = 2;
  return m;
}
// quad i: a(x, y, z0..z1) → b(x, y, z0..z1), normal n
function setQuad(mesh, i, ax, ay, az0, az1, bx, by, bz0, bz1, nx, ny) {
  mesh.geometry.attributes.position.array.set([ax, ay, az0, bx, by, bz0, bx, by, bz1, ax, ay, az1], 12 * i);
  mesh.geometry.attributes.normal.array.set([nx, ny, 0, nx, ny, 0, nx, ny, 0, nx, ny, 0], 12 * i);
}
function commit(mesh, n) {
  mesh.geometry.setDrawRange(0, 6 * n);
  mesh.geometry.attributes.position.needsUpdate = mesh.geometry.attributes.normal.needsUpdate = true;
}
const span = quadMesh(1, M.film.clone());
span.material.opacity = 0.42;
W.add(span);
const Z_GRIP = [K.Z_BOTTOM - FILM.w / 2, K.Z_BOTTOM - FILM.w / 2 + CLAMP.gripH]; // band of web in the jaws

const armGroup = group(W);
const armLine = polyline(armGroup, [[0, 0, 0], [0, 0, 0]], 0xff5fa2);
const filmAxis = polyline(armGroup, [[0, 0, 0], [0, 0, 0]], 0xff5fa2, 0.35);
const armDot = add(armGroup, new THREE.SphereGeometry(0.022, 12, 8), new THREE.MeshBasicMaterial({ color: 0xff5fa2 }));
armDot.castShadow = false;
for (const l of [armLine, filmAxis]) l.frustumCulled = false;
const setLine = (l, a, b) => { l.geometry.attributes.position.array.set([...a, ...b]); l.geometry.attributes.position.needsUpdate = true; };

// ---------- unit loads: every pallet and every box is one instance; the film is one mesh per pallet ----------
const palletGeo = (() => {
  const parts = [], P = PALLET;
  const b = (sx, sy, sz, x, y, z) => parts.push(new THREE.BoxGeometry(sx, sy, sz).translate(x, y, z));
  for (const [y, w] of [[-0.3275, 0.145], [-0.1638, 0.1], [0, 0.145], [0.1638, 0.1], [0.3275, 0.145]]) b(P.sx, w, 0.022, 0, y, P.h - 0.011); // deck boards
  for (const x of [-0.5275, 0, 0.5275]) b(0.145, P.sy, 0.022, x, 0, 0.111); // stringer boards
  for (const y of [-0.35, 0, 0.35]) for (const x of [-0.5275, 0, 0.5275]) b(0.145, y ? 0.10 : 0.145, 0.078, x, y, 0.061); // blocks
  for (const y of [-0.35, 0, 0.35]) b(P.sx, y ? 0.10 : 0.145, 0.022, 0, y, 0.011); // bottom boards, along the travel direction
  return mergeGeometries(parts);
})();
const CAP_P = 16, CAP_B = 1400;
const palletMesh = new THREE.InstancedMesh(palletGeo, M.wood, CAP_P);
const boxMesh = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), M.box, CAP_B);
for (const im of [palletMesh, boxMesh]) { im.castShadow = im.receiveShadow = true; im.frustumCulled = false; im.instanceMatrix.setUsage(THREE.DynamicDrawUsage); W.add(im); }
const skuColor = SKUS.map((s) => new THREE.Color(s.color));
const tint = new THREE.Color(), mP = new THREE.Matrix4(), mB = new THREE.Matrix4(), quat = new THREE.Quaternion(), v3 = new THREE.Vector3(), one = new THREE.Vector3(1, 1, 1), zAxis = new THREE.Vector3(0, 0, 1);
const films = new Map(); // pallet id -> { mesh, version }

// Film bands on a pallet, in {P} with absolute heights. Later wraps lie further out.
function buildFilm(mesh, film) {
  const A = film.anchors;
  let n = 0;
  for (let i = 1; i < A.length && n < 96; i++) {
    const a = A[i - 1], b = A[i], L = Math.hypot(b.x - a.x, b.y - a.y);
    if (L < 1e-6) continue;
    const nx = -(b.y - a.y) / L, ny = (b.x - a.x) / L, o = 0.003 + 0.0005 * i, h = FILM.w / 2;
    const held = i === 1 && film.tailHeld; // the clamp end of the tail is only as high as the jaws
    setQuad(mesh, n++, a.x + nx * o, a.y + ny * o, held ? Z_GRIP[0] : a.z - h, held ? Z_GRIP[1] : a.z + h, b.x + nx * o, b.y + ny * o, b.z - h, b.z + h, nx, ny);
  }
  commit(mesh, n);
}
function drawLoads() {
  let np = 0, nb = 0;
  const alive = new Set();
  for (const p of cell.pallets) {
    if (np >= CAP_P) break;
    mP.compose(v3.set(p.wx, p.wy, LINE.z), quat.setFromAxisAngle(zAxis, p.yaw), one);
    palletMesh.setMatrixAt(np++, mP);
    for (const b of p.boxes) {
      if (nb >= CAP_B) break;
      mB.makeScale(b.sx - 0.006, b.sy - 0.006, b.sz - 0.004).setPosition(b.lx, b.ly, b.lz).premultiply(mP);
      boxMesh.setMatrixAt(nb, mB);
      boxMesh.setColorAt(nb++, tint.copy(skuColor[p.skuIndex]).multiplyScalar(0.9 + 0.1 * (((p.id * 31 + nb * 17) % 11) / 10)));
    }
    if (p.film) {
      alive.add(p.id);
      let f = films.get(p.id);
      if (!f) { f = { mesh: quadMesh(96, M.film), version: -1 }; W.add(f.mesh); films.set(p.id, f); }
      if (f.version !== p.film.version) { buildFilm(f.mesh, p.film); f.version = p.film.version; }
      f.mesh.position.set(p.wx, p.wy, 0);
      f.mesh.rotation.z = p.yaw;
    }
  }
  for (const [id, f] of films) if (!alive.has(id)) { W.remove(f.mesh); f.mesh.geometry.dispose(); films.delete(id); }
  palletMesh.count = np;
  boxMesh.count = nb;
  palletMesh.instanceMatrix.needsUpdate = boxMesh.instanceMatrix.needsUpdate = true;
  if (boxMesh.instanceColor) boxMesh.instanceColor.needsUpdate = true;
}

let cell;
function reset() {
  cell = new Cell({ seed: 7 });
  acc = 0;
}

// ---------- HUD ----------
const $ = (id) => document.getElementById(id);
const LIFT = CARRIAGE.lift, VZ = K.liftLimits(WRAP.overlapDown).v;
const JOINTS = [
  { id: 'q1', name: 'Table', val: (w) => `${(w.om / RPM).toFixed(2)} rpm`, pos: (w) => (w.th % (2 * PI)) / (2 * PI), spd: (w) => w.om / TABLE.wrap.v },
  { id: 'q2', name: 'Deck', val: (w) => `${w.deckV.toFixed(3)} m/s`, pos: () => null, spd: (w) => w.deckV / LINE.move.v },
  { id: 'q3', name: 'Lift', val: (w) => `${w.lift.toFixed(3)} m`, pos: (w) => (w.lift - LIFT.min) / (LIFT.max - LIFT.min), spd: (w) => Math.abs(w.liftV) / VZ },
  { id: 'q4', name: 'Feed', val: (w) => `${w.vFeed.toFixed(3)} m/s`, pos: () => null, spd: (w) => w.vFeed / FILM.feed.vMax },
  { id: 'q5', name: 'Dancer', val: (w) => `${(1000 * w.dancer).toFixed(1)} mm`, pos: (w) => 0.5 + w.dancer / (2 * FILM.dancer.stroke), spd: () => 0 },
  { id: 'g1', name: 'Clamp', val: (w) => (w.clampUp === 1 ? 'up' : w.clampUp === 0 ? 'down' : 'moving'), pos: (w) => w.clampUp, spd: () => 0 },
  { id: 'g2', name: 'Jaw', val: (w) => (w.jaw === 1 ? 'closed' : w.jaw === 0 ? 'open' : 'moving'), pos: (w) => w.jaw, spd: () => 0 },
  { id: 'g3', name: 'Slide', val: (w) => `${w.slide.toFixed(3)} m`, pos: (w) => (w.slide - CUT.park) / (CUT.press - CUT.park), spd: (w) => Math.abs(w.slideV) / CUT.move.v },
];
const axisRows = JOINTS.map((j) => {
  const row = document.createElement('div');
  row.className = 'j';
  row.innerHTML = `<span>${j.id}</span><span class="nm">${j.name}</span><span class="val"></span><div class="bar pos"><i></i></div><div class="bar"><i></i></div>`;
  $('axes').appendChild(row);
  return { val: row.querySelector('.val'), pos: row.querySelectorAll('i')[0], spd: row.querySelectorAll('i')[1] };
});
const lamp = (el, on, cls = 'on') => { el.className = 'lamp ' + (on ? cls : ''); };

function hud() {
  const w = cell.w, p = w.pallet, st = cell.stats, onT = cell.at(3);
  $('phase').textContent = cell.label();
  $('load').textContent = onT ? `#${onT.id} ${onT.sku} × ${onT.layers} · ${onT.H.toFixed(3)} m · ${onT.mass} kg${onT.wrapped ? ' · wrapped' : ''}` : 'none';
  $('rev').textContent = w.phase === 'WRAP' ? `${(w.th / (2 * PI)).toFixed(2)} of ${w.plan.revs}` : '–';
  JOINTS.forEach((j, i) => {
    const r = axisRows[i], pos = j.pos(w);
    r.val.textContent = j.val(w);
    r.pos.style.display = pos === null ? 'none' : '';
    if (pos !== null) r.pos.style.left = `calc(${100 * Math.min(1, Math.max(0, pos))}% - 1.5px)`;
    r.spd.style.width = `${Math.min(100, 100 * j.spd(w))}%`;
  });
  const bad = w.bad.length > 0;
  $('lMon').className = 'lamp ' + (bad ? 'bad' : 'on');
  $('mon').textContent = bad ? w.bad[0] : `OK  (${st.violations} violations)`;

  const on = w.filmOnLoad;
  $('arm').textContent = on ? `${w.arm.toFixed(3)} m  (mean ${K.ARM_MEAN.toFixed(3)})` : '–';
  $('vf').textContent = `${w.vFilm.toFixed(3)} / ${w.vFeed.toFixed(3)} m/s`;
  $('alpha').textContent = on ? `${(w.alpha / DEG).toFixed(1)}° / ${w.free.toFixed(3)} m` : '–';
  $('laid').textContent = on ? `${p.film.corners} / ${p.film.laid.toFixed(1)} m` : '–';
  $('lag').textContent = on ? `${w.lift.toFixed(3)} / ${p.film.active.z.toFixed(3)} m` : `${w.lift.toFixed(3)} m`;
  const pitch = w.om > 0 ? (2 * PI * Math.abs(w.liftV)) / w.om : 0;
  $('pitch').textContent = pitch > 1e-6 ? `${pitch.toFixed(3)} m/rev / ${(100 * (1 - pitch / FILM.w)).toFixed(0)} %` : w.om > 0 ? '0 (level wraps)' : '–';
  $('ten').textContent = `${w.tension.toFixed(1)} N / ${(1000 * w.dancer).toFixed(1)} mm`;
  $('sf').textContent = onT && !onT.move ? `${Number.isFinite(w.sf) ? w.sf.toFixed(2) : '∞'} / ${K.sweptRadius([onT.dx, onT.ey]).toFixed(3)} m` : '–';
  $('roll').textContent = `${w.rollLen.toFixed(0)} m  (Ø${(2000 * cell.rollRadius).toFixed(0)} mm)`;

  lamp($('sB1'), cell.b1, cell.ready ? 'on' : 'warn');
  lamp($('sB2'), cell.b2, 'warn'); lamp($('sB3'), cell.b3); lamp($('sB4'), cell.b4, 'warn'); lamp($('sB5'), cell.b5, 'warn');
  lamp($('sHome'), w.om === 0 && w.th === 0);
  lamp($('sClamp'), w.clampUp === 1, 'info'); lamp($('sJaw'), w.jaw === 1, 'info');
  lamp($('sPark'), w.slide === CUT.park); lamp($('sWire'), w.wire, 'bad');
  $('count').textContent = `${st.wrapped} / ${cell.pallets.size}`;
  $('thr').textContent = `${cell.t > 300 ? ((st.wrapped * 3600) / cell.t).toFixed(0) + ' pallets/h' : '–'} / ${st.period.length ? st.period.at(-1).toFixed(1) + ' s' : '–'}`;
  const s = Math.floor(cell.t);
  $('time').textContent = `${String(Math.floor(s / 3600)).padStart(2, '0')}:${String(Math.floor(s / 60) % 60).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
}

// ---------- scene update from sim state ----------
let showArm = true, view = 'overview';
const mR = new THREE.Matrix4();
function sync() {
  const w = cell.w;
  table.rotation.z = w.th; // q1
  for (const g of deckRollers) g.rotation.y = cell.roll[3] / LINE.rollR; // q2
  lineRollers.forEach((r, i) => {
    mR.makeRotationY(cell.roll[r.zone] / LINE.rollR).setPosition(r.x, 0, LINE.z - LINE.rollR);
    rollMesh.setMatrixAt(i, mR); stripeMesh.setMatrixAt(i, mR);
  });
  rollMesh.instanceMatrix.needsUpdate = stripeMesh.instanceMatrix.needsUpdate = true;
  carriage.position.z = w.lift; // q3
  const rr = cell.rollRadius;
  filmRoll.scale.set(rr, rr, 1);
  rollMark.position.set(ROLL_POS[0] + 0.6 * rr * Math.cos(-w.rollAngle), ROLL_POS[1] + 0.6 * rr * Math.sin(-w.rollAngle), FILM.w0 / 2 + 0.005);
  rollMark.rotation.z = -w.rollAngle;
  stretch[0].rotation.z = w.feed / FILM.lambda / 0.035; // q4: the second roller runs λ times faster
  stretch[1].rotation.z = -w.feed / 0.035;
  dancer.rotation.z = -w.dancer / FILM.dancer.arm; // q5
  clampLift.position.z = CLAMP.zDown + CLAMP.stroke * w.clampUp; // g1
  jaws.forEach((j, i) => { j.position.y = (i ? -1 : 1) * (0.006 + (CLAMP.open / 2) * (1 - w.jaw)); }); // g2
  slide.position.y = w.slide; // g3
  wire.material.color.setHex(w.wire ? 0xff7a1a : 0x70757d);

  EYES.b1.color.setHex(cell.b1 ? 0xf0b429 : 0x3a4250);
  EYES.b2.color.setHex(cell.b2 ? 0xf0b429 : 0x3a4250);
  EYES.b4.color.setHex(cell.b4 ? 0xf0b429 : 0x3a4250);
  EYES.b5.color.setHex(cell.b5 ? 0xf0b429 : 0x3a4250);
  const i1 = cell.resting(2);
  for (const b of gridBeams) b.m.color.setHex(i1 && b.z < LINE.z + i1.H ? 0xf0b429 : 0x3a4250);
  const top = w.pallet && cell.at(3) === w.pallet ? LINE.z + w.pallet.H : 0;
  loadEye.material.color.setHex(w.lift + FILM.w / 2 < top ? 0xf0b429 : 0x3ecf8e);
  const lit = [w.bad.length > 0, w.phase === 'WRAP' || w.phase === 'CLAMP' || w.phase === 'CUT', w.phase === 'IDLE' || w.phase === 'TRANSFER' || w.phase === 'OUT', w.phase === 'ROLL'];
  tower.forEach((t, i) => t.m.material.color.setHex(lit[i] ? t.c : 0x343a44));
  drawLoads();

  // free film: exit roller → active anchor, or → clamp while only the clamp holds it
  const h = FILM.w / 2;
  let line = null;
  if (w.filmOnLoad) {
    const f = w.pallet.film, a = f.active, C = f.world(a, w.th);
    line = K.filmLine(C);
    const held = f.corners === 0;
    setQuad(span, 0, line.T[0], line.T[1], w.lift - h, w.lift + h, C[0], C[1], held ? Z_GRIP[0] : a.z - h, held ? Z_GRIP[1] : a.z + h, -line.u[1], line.u[0]);
    commit(span, 1);
  } else if (w.supply) {
    const C = K.rot(w.th, [CLAMP.x, CLAMP.y]), l = K.filmLine(C);
    setQuad(span, 0, l.T[0], l.T[1], w.lift - h, w.lift + h, C[0], C[1], Z_GRIP[0], Z_GRIP[1], -l.u[1], l.u[0]);
    commit(span, 1);
  } else commit(span, 0);

  // moment arm: perpendicular from the turntable axis to the film line
  armGroup.visible = showArm && !!line && w.phase === 'WRAP';
  if (armGroup.visible) {
    const d = -(line.T[0] * line.u[0] + line.T[1] * line.u[1]), foot = [line.T[0] + d * line.u[0], line.T[1] + d * line.u[1], w.lift];
    setLine(armLine, [0, 0, w.lift], foot);
    setLine(filmAxis, [line.T[0] - 0.3 * line.u[0], line.T[1] - 0.3 * line.u[1], w.lift], [line.T[0] + 2.4 * line.u[0], line.T[1] + 2.4 * line.u[1], w.lift]);
    armDot.position.set(...foot);
  }
}

// ---------- cameras (three coordinates: x, up, −worldY) ----------
const VIEWS = {
  overview: [[4.8, 3.7, 6.9], [0.4, 1.0, 0.2]],
  top: [[0, 15, 0.01], [0, 0, 0]],
  machine: [[3.4, 2.6, 4.2], [0.1, 1.15, 0.3]],
  clamp: [[1.5, 1.25, 1.75], [0.35, 0.72, 0.6]],
};
function setView(name) {
  view = name;
  document.querySelectorAll('[data-view]').forEach((b) => b.classList.toggle('on', b.dataset.view === name));
  if (!VIEWS[name]) return;
  camera.position.set(...VIEWS[name][0]);
  controls.target.set(...VIEWS[name][1]);
}
const tgt = new THREE.Vector3();
function chaseCam() {
  if (view !== 'carriage') return; // rides up and down with the carriage
  const z = cell.w.lift;
  controls.target.lerp(tgt.set(0.25, z, 0.75), 0.15);
  camera.position.lerp(tgt.set(1.9, z + 0.75, 2.5), 0.08);
}

// ---------- UI ----------
let paused = false, speed = 1, acc = 0;
$('pause').onclick = (e) => { paused = !paused; e.target.textContent = paused ? 'Resume' : 'Pause'; e.target.classList.toggle('on', paused); };
$('reset').onclick = () => reset();
document.querySelectorAll('[data-speed]').forEach((b) => (b.onclick = () => {
  speed = +b.dataset.speed;
  document.querySelectorAll('[data-speed]').forEach((x) => x.classList.toggle('on', x === b));
}));
document.querySelectorAll('[data-view]').forEach((b) => (b.onclick = () => setView(b.dataset.view)));
$('panel').onclick = (e) => { $('hud').hidden = !$('hud').hidden; e.target.classList.toggle('on', !$('hud').hidden); };
$('armBtn').onclick = (e) => { showArm = !showArm; e.target.classList.toggle('on', showArm); };
$('sweptBtn').onclick = (e) => { sweptGroup.visible = !sweptGroup.visible; e.target.classList.toggle('on', sweptGroup.visible); };

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
// Integration handle: window.wrapper.cell is the Cell (see WRAPPER_SPEC.md).
window.wrapper = { get cell() { return cell; }, setView, camera, controls, step: (n = 1) => { for (let i = 0; i < n; i++) cell.step(DT); } };

function frame(now) {
  const wall = Math.min((now - last) / 1000, 0.1);
  last = now;
  if (!paused) {
    acc += wall * speed;
    let n = 0;
    while (acc >= DT && n < 1200) { cell.step(DT); acc -= DT; n++; }
    if (n === 1200) acc = 0;
  }
  sync();
  hud();
  chaseCam();
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
