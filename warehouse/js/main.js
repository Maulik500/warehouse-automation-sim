// three.js view of the integrated warehouse. All geometry lives in `W`, a Z-up group, so
// coordinates are identical to WAREHOUSE_SPEC.md and sim.js. Every machine is built as the
// kinematic chain of its own specification; cells are groups placed by (t, ψ) of spec §6.1.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { CRANE, FORK as AFORK, RACK, PD, LAYOUT } from '../../asrs/js/config.js';
import { AMR as FAMR, MAST, FORK as FFORK } from '../../amr/js/config.js';
import { AMR as MAMR, ARM, TOOL as MTOOL, DECK } from '../../mobman/js/config.js';
import { DH } from '../../mobman/js/arm.js';
import { ROBOT, CELL } from '../../js/config.js';
import { LINE, TABLE, CARRIAGE, CLAMP, CUT } from '../../wrapper/js/config.js';
import { BUILDING, PLACE, PORT, PCELL, PTOOL, ROAD, FLOOR, SKUS, SKU, MIX, ROWS, PALLET, WRAPX, DT, DEG } from './config.js';
import { World } from './sim.js';
import { RING, mmRoutes, MM_LINK } from './roadmap.js';
import { palletG } from './asrs.js';
import { orderHeight } from './wes.js';

const PI = Math.PI;
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
document.body.prepend(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14171c);
scene.fog = new THREE.Fog(0x14171c, 90, 260);
const camera = new THREE.PerspectiveCamera(40, 1, 0.1, 600);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.maxPolarAngle = PI / 2 - 0.02;
scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x2a2622, 1.35));
const sun = new THREE.DirectionalLight(0xffffff, 2.0);
sun.position.set(10, 60, 10);
sun.target.position.set(34, 0, -24);
sun.castShadow = true;
sun.shadow.mapSize.set(4096, 4096);
Object.assign(sun.shadow.camera, { left: -46, right: 46, top: 40, bottom: -40, near: 1, far: 160 });
sun.shadow.bias = -0.0006;
scene.add(sun, sun.target);

const W = new THREE.Group(); // global frame {G}: Z up
W.rotation.x = -PI / 2;
scene.add(W);

// ---------- helpers (Z-up coordinates) ----------
const mat = (color, o = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.2, ...o });
const M = {
  yellow: mat(0xf2b01b, { roughness: 0.45 }), blue: mat(0x3d6fb5, { roughness: 0.5 }), orange: mat(0xe0742a, { roughness: 0.5, metalness: 0.3 }),
  dark: mat(0x23272e, { roughness: 0.6, metalness: 0.4 }), steel: mat(0x9aa3b0, { roughness: 0.35, metalness: 0.7 }), roller: mat(0xb9c0cb, { roughness: 0.3, metalness: 0.8 }),
  tyre: mat(0x16181c, { roughness: 0.9, metalness: 0 }), conv: mat(0x4a5361, { metalness: 0.5 }), rack: mat(0x2f5f9e, { roughness: 0.6, metalness: 0.4 }), beam: mat(0xd8792a, { roughness: 0.6, metalness: 0.3 }),
  wood: mat(0xb98a55, { roughness: 0.85, metalness: 0 }), box: mat(0xffffff, { roughness: 0.88, metalness: 0 }), arm: mat(0xd9dee6, { roughness: 0.4, metalness: 0.3 }),
  joint: mat(0x4aa3ff, { roughness: 0.4, metalness: 0.3 }), cup: mat(0x2b2f36, { roughness: 0.9, metalness: 0 }), robot: mat(0xe8641c, { roughness: 0.45, metalness: 0.25 }), red: mat(0xc0392b, { roughness: 0.5 }),
  film: new THREE.MeshStandardMaterial({ color: 0xcfe8ff, transparent: true, opacity: 0.28, roughness: 0.15, metalness: 0, depthWrite: false }),
};
function add(parent, geo, material, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(geo, material);
  m.position.set(x, y, z);
  m.castShadow = m.receiveShadow = true;
  parent.add(m);
  return m;
}
const slab = (p, x0, x1, y0, y1, z0, z1, m) => add(p, new THREE.BoxGeometry(Math.abs(x1 - x0), Math.abs(y1 - y0), Math.abs(z1 - z0)), m, (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2);
function cylGeo(r, len, axis, seg = 24) {
  const g = new THREE.CylinderGeometry(r, r, len, seg);
  if (axis === 'x') g.rotateZ(-PI / 2);
  if (axis === 'z') g.rotateX(PI / 2);
  return g;
}
function rod(parent, axis, a, b, r, material, u = 0, v = 0) {
  const m = add(parent, cylGeo(r, Math.abs(b - a), axis), material), mid = (a + b) / 2;
  if (axis === 'x') m.position.set(mid, u, v); else if (axis === 'y') m.position.set(u, mid, v); else m.position.set(u, v, mid);
  return m;
}
const group = (parent, x = 0, y = 0, z = 0) => { const g = new THREE.Group(); g.position.set(x, y, z); parent.add(g); return g; };
const labels = group(W);
function label(text, x, y, z, color = '#d9dee6', size = 1) {
  const c = document.createElement('canvas');
  c.width = 512; c.height = 96;
  const g = c.getContext('2d');
  g.font = '700 54px ui-monospace, Consolas, monospace';
  g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillStyle = color;
  g.fillText(text, 256, 50);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthWrite: false }));
  s.position.set(x, y, z); s.scale.set(4.0 * size, 0.75 * size, 1);
  labels.add(s);
  return s;
}
function polyline(parent, pts, color, opacity = 1) {
  const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts.map((p) => new THREE.Vector3(...p))), new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity }));
  parent.add(l);
  return l;
}
function pad(parent, x0, x1, y0, y1, color, opacity = 0.12) {
  const m = new THREE.Mesh(new THREE.PlaneGeometry(Math.abs(x1 - x0), Math.abs(y1 - y0)), new THREE.MeshBasicMaterial({ color, transparent: true, opacity }));
  m.position.set((x0 + x1) / 2, (y0 + y1) / 2, 0.006);
  parent.add(m);
  return m;
}
function instanced(parent, geo, material, n) {
  const im = new THREE.InstancedMesh(geo, material, n);
  im.castShadow = im.receiveShadow = true; im.frustumCulled = false; im.count = 0;
  im.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  parent.add(im);
  return im;
}
function rollers(parent, n, r, len, axis, at) {
  const im = new THREE.InstancedMesh(cylGeo(r, len, axis, 10), M.roller, n), m4 = new THREE.Matrix4();
  for (let i = 0; i < n; i++) im.setMatrixAt(i, m4.makeTranslation(...at(i)));
  im.castShadow = im.receiveShadow = true;
  parent.add(im);
  return im;
}

// ---------- floor, building, roads ----------
{
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(400, 400), mat(0x1a1d23, { roughness: 0.95, metalness: 0 }));
  floor.position.set(BUILDING.x / 2, BUILDING.y / 2, -0.002);
  floor.receiveShadow = true;
  W.add(floor);
  const hall = new THREE.Mesh(new THREE.PlaneGeometry(BUILDING.x, BUILDING.y), mat(0x23272e, { roughness: 0.9, metalness: 0 }));
  hall.position.set(BUILDING.x / 2, BUILDING.y / 2, 0);
  hall.receiveShadow = true;
  W.add(hall);
  polyline(W, [[0, 0, 0.01], [BUILDING.x, 0, 0.01], [BUILDING.x, BUILDING.y, 0.01], [0, BUILDING.y, 0.01], [0, 0, 0.01]], 0x5a6472);
}
const roads = group(W);
function drawPath(path, color, opacity = 0.75) {
  const n = Math.ceil(path.length / 0.25);
  polyline(roads, Array.from({ length: n + 1 }, (_, i) => { const a = path.at((path.length * i) / n); return [a.x, a.y, 0.012]; }), color, opacity);
}

// ---------- unit loads ----------
const palletGeo = (() => {
  const parts = [], P = PALLET;
  const b = (sx, sy, sz, x, y, z) => parts.push(new THREE.BoxGeometry(sx, sy, sz).translate(x, y, z));
  for (const [y, w] of [[-0.3275, 0.145], [-0.1638, 0.1], [0, 0.145], [0.1638, 0.1], [0.3275, 0.145]]) b(P.sx, w, 0.022, 0, y, P.h - 0.011);
  for (const x of [-0.5275, 0, 0.5275]) b(0.145, P.sy, 0.022, x, 0, 0.111);
  for (const x of [-0.5275, 0, 0.5275]) for (const y of [-0.35, 0, 0.35]) b(0.145, y ? 0.1 : 0.145, 0.078, x, y, 0.061);
  for (const y of [-0.35, 0, 0.35]) b(P.sx, y ? 0.1 : 0.145, 0.022, 0, y, 0.011);
  return mergeGeometries(parts);
})();
const unit = new THREE.BoxGeometry(1, 1, 1);
const palletIM = instanced(W, palletGeo, M.wood, 160);
const rackPalIM = instanced(W, new THREE.BoxGeometry(PALLET.sx, PALLET.sy, PALLET.h), M.wood, 640);
const rackLoadIM = instanced(W, unit, M.box, 640);
const boxIM = instanced(W, unit, M.box, 2500);
const filmIM = instanced(W, unit, M.film, 40);
filmIM.castShadow = false;
const skuColor = Object.fromEntries(SKUS.map((s) => [s.id, new THREE.Color(s.color)]));
const tint = new THREE.Color(), m4 = new THREE.Matrix4(), quat = new THREE.Quaternion(), v3 = new THREE.Vector3(), scl = new THREE.Vector3(), zAxis = new THREE.Vector3(0, 0, 1), one = new THREE.Vector3(1, 1, 1);
const setInst = (im, i, x, y, z, yaw, sx = 1, sy = 1, sz = 1) => im.setMatrixAt(i, m4.compose(v3.set(x, y, z), quat.setFromAxisAngle(zAxis, yaw), scl.set(sx, sy, sz)));

// ---------- ASRS ----------
const A = PLACE.asrs, asrsG = group(W, A.tx, A.ty, 0);
const cranes = [];
{
  const xa = RACK.x1 - RACK.pitchX / 2, xb = xa + RACK.bays * RACK.pitchX, ups = [], beams = [];
  for (const a of LAYOUT.aisles) {
    for (const [row, side] of a.rows) {
      const yf = a.y + side * RACK.yFace, yb = a.y + side * (RACK.yFace + RACK.depth);
      for (let b = 0; b <= RACK.bays; b++) for (const y of [yf + side * 0.05, yb - side * 0.05]) ups.push([xa + b * RACK.pitchX, y, RACK.height / 2]);
      for (let l = 0; l < RACK.levels; l++) for (const y of [yf + side * 0.05, yb - side * 0.05]) beams.push([(xa + xb) / 2, y, RACK.z1 + l * RACK.pitchZ - 0.06]);
      label(`ROW ${row}`, A.tx + xb + 1.6, A.ty + a.y + side * RACK.yStore, 1.2, '#7fb0ff', 0.9);
    }
    // crane (ASRS spec §3.1): aisle > travel base (q1) > lift carriage (q2) > fork middle (q3/2), fork top (q3)
    const ag = group(asrsG, 0, a.y, 0), base = group(ag), car = group(base), mid = group(car), top = group(car);
    slab(ag, -1.6, 20.6, -0.05, 0.05, 0, 0.06, M.steel); // floor rail
    slab(base, -CRANE.chassisLen / 2, CRANE.chassisLen / 2, -CRANE.chassisW / 2, CRANE.chassisW / 2, 0.06, 0.06 + CRANE.chassisH, M.yellow);
    for (const e of [1, -1]) slab(base, e * CRANE.mastX - CRANE.mastSx / 2, e * CRANE.mastX + CRANE.mastSx / 2, -CRANE.mastSy / 2, CRANE.mastSy / 2, 0.3, CRANE.mastH, M.yellow);
    slab(base, -CRANE.mastX, CRANE.mastX, -0.15, 0.15, CRANE.mastH - 0.25, CRANE.mastH, M.yellow);
    slab(car, -CRANE.carriageSx / 2, CRANE.carriageSx / 2, -CRANE.carriageSy / 2, CRANE.carriageSy / 2, -CRANE.carriageDrop, -0.07, M.dark);
    for (const e of [1, -1]) {
      const x0 = e * AFORK.tineX - AFORK.tineW / 2, x1 = e * AFORK.tineX + AFORK.tineW / 2, h = AFORK.length / 2;
      slab(car, x0, x1, -h, h, -0.07, -0.045, M.steel); slab(mid, x0, x1, -h, h, -0.045, -0.02, M.roller); slab(top, x0, x1, -h, h, -0.02, 0, M.orange);
    }
    cranes.push({ base, car, mid, top });
  }
  const upI = new THREE.InstancedMesh(new THREE.BoxGeometry(RACK.uprightW, 0.1, RACK.height), M.rack, ups.length);
  ups.forEach((p, i) => upI.setMatrixAt(i, m4.makeTranslation(...p)));
  const bmI = new THREE.InstancedMesh(new THREE.BoxGeometry(xb - xa, 0.05, 0.1), M.beam, beams.length);
  beams.forEach((p, i) => bmI.setMatrixAt(i, m4.makeTranslation(...p)));
  for (const im of [upI, bmI]) { im.castShadow = im.receiveShadow = true; asrsG.add(im); }
  label('ASRS', A.tx + 11, A.ty - 3.6, 14.6, '#7fb0ff', 2.2);
}
const laneView = {};
function buildLanes(world) {
  for (const row of ROWS) {
    const l = world.asrs.lanes[row], f = l.frame, g = group(W);
    for (const e of [1, -1]) slab(g, f.x0, f.x1 + 1.2, f.y + e * f.hw - 0.04, f.y + e * f.hw, f.top - 0.16, f.top, M.conv);
    for (let x = f.x0 + 0.2; x < f.x1 + 1.2; x += 1.2) for (const e of [1, -1]) slab(g, x - 0.03, x + 0.03, f.y + e * f.hw - 0.05, f.y + e * f.hw, 0, f.top - 0.16, M.conv);
    const n = Math.floor((f.x1 + 1.0 - f.x0) / 0.16);
    rollers(g, n, 0.04, 2 * f.hw - 0.1, 'y', (i) => [f.x0 + 0.1 + i * 0.16, f.y, f.top - 0.04]);
    const stops = [-0.5275, 0, 0.5275].map((dy) => slab(g, f.x0 + 0.005, f.x0 + 0.03, f.y + dy - 0.07, f.y + dy + 0.07, f.top - 0.09, f.top, M.orange));
    const lamp = add(g, cylGeo(0.05, 0.12, 'z', 12), new THREE.MeshBasicMaterial({ color: 0x343a44 }), f.x0 - 0.1, f.y + f.hw + 0.15, 1.3);
    slab(g, f.x0 - 0.12, f.x0 - 0.08, f.y + f.hw + 0.13, f.y + f.hw + 0.17, 0, 1.24, M.dark);
    label(`PORT ${row}`, l.x - 0.2, l.y, 2.5, '#7fb0ff', 0.55);
    laneView[row] = { stops, lamp, top: f.top };
  }
}

// ---------- palletizing robot (ROBOT_SPEC §3.1): one group per joint ----------
function buildRobot(parent) {
  const { d1, a1, a2, a3, d4, d6 } = ROBOT, J = [];
  rod(parent, 'z', 0, 0.22, 0.42, M.dark);
  J[0] = group(parent);
  rod(J[0], 'z', 0.22, d1 - 0.1, 0.33, M.robot);
  slab(J[0], -0.15, a1 + 0.2, -0.3, 0.3, d1 - 0.28, d1 + 0.12, M.robot);
  J[1] = group(J[0], a1, 0, d1);
  rod(J[1], 'y', -0.33, 0.33, 0.2, M.dark);
  slab(J[1], -0.13, 0.13, -0.17, 0.17, 0, a2, M.robot);
  J[2] = group(J[1], 0, 0, a2);
  rod(J[2], 'y', -0.28, 0.28, 0.17, M.dark);
  slab(J[2], -0.3, 0.35, -0.2, 0.2, -0.14, a3 + 0.16, M.robot);
  J[3] = group(J[2], 0, 0, a3);
  rod(J[3], 'x', 0.35, d4 - 0.12, 0.1, M.robot);
  J[4] = group(J[3], d4, 0, 0);
  rod(J[4], 'y', -0.13, 0.13, 0.11, M.dark);
  J[5] = group(J[4]);
  rod(J[5], 'x', 0.05, d6, 0.08, M.steel);
  const F = group(J[5], d6, 0, 0); // flange: Z out of the flange
  F.rotation.y = PI / 2;
  rod(F, 'z', 0, 0.25, 0.04, M.dark);
  slab(F, -PTOOL.padX / 2, PTOOL.padX / 2, -PTOOL.padY / 2, PTOOL.padY / 2, 0.25, 0.272, M.steel);
  const cups = [];
  for (let i = 0; i < 3; i++) for (const e of [-1, 1]) cups.push(rod(F, 'z', 0.272, 0.3, PTOOL.cupDia / 2 - 0.004, M.cup.clone(), (i - 1) * PTOOL.colX, e * PTOOL.rowY));
  return { J, cups };
}
const cellView = PLACE.pal.map((P, i) => {
  const g = group(W, P.tx, P.ty, P.tz), cv = CELL.conv, R = PCELL.recv, I = PCELL.incline, hw = PCELL.beltW / 2;
  g.rotation.z = P.psi;
  slab(g, -0.6, 0.6, -0.6, 0.6, -P.tz, 0, M.dark); // riser
  const robot = buildRobot(g);
  // tracking belt with the meter section
  slab(g, cv.x - hw, cv.x + hw, PCELL.merge.y - 0.45, cv.yEnd, cv.top - 0.09, cv.top - 0.005, M.dark);
  for (const e of [1, -1]) slab(g, cv.x + e * hw, cv.x + e * (hw + 0.04), PCELL.merge.y - 0.45, cv.yEnd, cv.top - 0.16, cv.top + 0.01, M.conv);
  for (let y = PCELL.merge.y; y < cv.yEnd; y += 1.2) for (const e of [1, -1]) slab(g, cv.x + e * hw, cv.x + e * (hw + 0.05), y - 0.03, y + 0.03, -P.tz, cv.top - 0.16, M.conv);
  for (const y of [cv.yA, cv.yB]) polyline(g, [[cv.x - hw - 0.1, y, cv.top + 0.012], [cv.x + hw + 0.1, y, cv.top + 0.012]], 0x3ecf8e); // tracking window
  // incline and receiving conveyor
  const inc = slab(g, I.x0, I.x1, R.y - R.w / 2, R.y + R.w / 2, -0.04, 0, M.dark);
  inc.position.set((I.x0 + I.x1) / 2, R.y, (I.z0 + I.z1) / 2 - 0.02);
  inc.rotation.y = Math.atan2(I.z0 - I.z1, I.x1 - I.x0);
  for (const e of [1, -1]) slab(g, R.x0, R.x1, R.y + e * R.w / 2, R.y + e * (R.w / 2 + 0.04), R.z - 0.12, R.z + 0.05, M.conv);
  for (let x = R.x0 + 0.1; x < R.x1; x += 0.9) for (const e of [1, -1]) slab(g, x - 0.03, x + 0.03, R.y + e * R.w / 2, R.y + e * (R.w / 2 + 0.05), -P.tz, R.z - 0.12, M.conv);
  for (const x of [I.x0 + 0.1, I.x1 - 0.1]) for (const e of [1, -1]) slab(g, x - 0.03, x + 0.03, R.y + e * (R.w / 2 - 0.02), R.y + e * (R.w / 2 + 0.03), -P.tz, (x < 3 ? I.z0 : I.z1) - 0.06, M.conv);
  rollers(g, Math.floor((R.x1 - R.x0) / 0.075), 0.025, R.w, 'y', (k) => [R.x0 + 0.04 + k * 0.075, R.y, R.z - 0.025]);
  const gate = slab(g, R.x0 - 0.02, R.x0, R.y - R.w / 2 + 0.05, R.y + R.w / 2 - 0.05, R.z, R.z + 0.05, M.orange);
  // pallet conveyor: two chain strands, positions B and D
  const x0 = PCELL.dock.x - PALLET.sy / 2 - 0.03, x1 = 0.6, yc = PCELL.build.y;
  for (const e of [1, -1]) slab(g, x0, x1, yc + e * 0.42 - 0.03, yc + e * 0.42 + 0.03, -P.tz, -0.002, M.conv);
  for (const e of [1, -1]) slab(g, x0, x1, yc + e * 0.7 - 0.03, yc + e * 0.7, -P.tz, -0.02, M.dark);
  // guarding (plan outline) and the D zone
  polyline(g, [[PCELL.fenceX, 0.75, 0.01 - P.tz], [PCELL.fenceX, 2.5, 0.01 - P.tz], [2.6, 2.5, 0.01 - P.tz], [2.6, -5.0, 0.01 - P.tz], [1.0, -5.0, 0.01 - P.tz], [1.0, -1.2, 0.01 - P.tz], [-1.2, -1.2, 0.01 - P.tz], [PCELL.fenceX, 0.75, 0.01 - P.tz]], 0xf0b429, 0.8);
  for (const [fx, fy] of [[PCELL.fenceX, 0.75], [PCELL.fenceX, 2.5], [2.6, 2.5], [2.6, -5.0], [1.0, -5.0], [-1.2, -1.2]]) slab(g, fx - 0.03, fx + 0.03, fy - 0.03, fy + 0.03, -P.tz, 1.9, M.yellow);
  const lamp = add(g, cylGeo(0.06, 0.14, 'z', 12), new THREE.MeshBasicMaterial({ color: 0x343a44 }), PCELL.fenceX, 0.75, 2.0);
  label(`PAL ${i + 1}`, P.tx + 0.6, P.ty - 0.8, 3.6, '#f2b01b', 0.9);
  return { g, robot, gate, lamp };
});

// ---------- wrapper (WRAPPER_SPEC §3.4) ----------
const WP = PLACE.wrap, wrapG = group(W, WP.tx, WP.ty, 0);
const wrapView = (() => {
  const g = wrapG, hw = LINE.frameW / 2;
  for (const s of [1, -1]) {
    const xa = s * LINE.xEnd, xb = s * LINE.xFar;
    slab(g, xa, xb, -LINE.rollLen / 2, LINE.rollLen / 2, LINE.z - 0.1, LINE.z - 0.06, M.dark);
    for (const e of [1, -1]) slab(g, xa, xb, e * hw - e * 0.04, e * hw, LINE.z - 0.16, LINE.z, M.conv);
    for (let x = Math.min(xa, xb) + 0.2; x < Math.max(xa, xb); x += 1.3) for (const e of [1, -1]) slab(g, x - 0.03, x + 0.03, e * hw - e * 0.05, e * hw, 0, LINE.z - 0.16, M.conv);
    const n = Math.floor((LINE.xFar - LINE.xEnd) / LINE.rollPitch);
    rollers(g, n, LINE.rollR, LINE.rollLen, 'y', (k) => [s * (LINE.xEnd + 0.08 + k * LINE.rollPitch), 0, LINE.z - LINE.rollR]);
  }
  for (const [x, c] of [[WRAPX.i4, 0x4aa3ff], [WRAPX.o4, 0x3ecf8e]]) pad(g, x - 0.7, x + 0.7, 0.45, 3.4, c);
  const table = group(g);
  rod(table, 'z', TABLE.plateZ - 0.08, TABLE.plateZ, TABLE.rPlate, M.steel);
  rod(g, 'z', 0, TABLE.plateZ - 0.08, 0.45, M.dark);
  rollers(table, 11, LINE.rollR, 0.9, 'y', (k) => [-TABLE.deckX + 0.16 * k, 0, TABLE.z - LINE.rollR]);
  const clampG = group(table, CLAMP.x, CLAMP.y, 0);
  clampG.rotation.z = CLAMP.yaw;
  rod(clampG, 'z', TABLE.plateZ, CLAMP.zDown - 0.1, 0.03, M.dark);
  const jaws = group(clampG);
  const jaw = [1, -1].map((e) => slab(jaws, -0.05, 0.05, e * 0.02, e * 0.035, CLAMP.zDown - 0.12, CLAMP.zDown, M.orange));
  const ms = CARRIAGE.mast, b = CARRIAGE.body;
  slab(g, ms.x - ms.sx / 2, ms.x + ms.sx / 2, ms.y - ms.sy / 2, ms.y + ms.sy / 2, 0, ms.h, M.blue);
  const car = group(g);
  slab(car, b.x0, b.x1, b.y0, b.y1, -b.h / 2, b.h / 2, M.blue);
  rod(car, 'z', -0.25, 0.25, CARRIAGE.rExit, M.roller, CARRIAGE.roller[0], CARRIAGE.roller[1]);
  rod(car, 'z', -0.25, 0.25, 0.1, M.film, -0.25, -1.2);
  slab(g, CUT.x - 0.06, CUT.x + 0.06, -1.95, -1.75, 0, 1.05, M.dark);
  const slide = group(g);
  slab(slide, CUT.x - 0.05, CUT.x + 0.05, -0.06, 0, CUT.z[0], CUT.z[1], M.red);
  slab(slide, CUT.x - 0.03, CUT.x + 0.03, -0.5, -0.06, 0.74, 0.84, M.steel);
  const film = new THREE.Line(new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3)), new THREE.LineBasicMaterial({ color: 0xcfe8ff }));
  film.frustumCulled = false;
  g.add(film);
  label('WRAPPER', WP.tx, WP.ty - 2.6, 3.4, '#4aa3ff', 1.1);
  label('I4', WP.tx + WRAPX.i4, WP.ty + 3.3, 0.6, '#4aa3ff', 0.5); label('O4', WP.tx + WRAPX.o4, WP.ty + 3.3, 0.6, '#3ecf8e', 0.5);
  return { table, jaws, jaw, car, slide, film };
})();

// ---------- floor stations ----------
for (const s of FLOOR.storage) { pad(W, s.x - 0.62, s.x + 0.62, s.y - 0.42, s.y + 0.42, 0x3ecf8e); }
for (const s of FLOOR.empties) pad(W, s.x - 0.62, s.x + 0.62, s.y - 0.42, s.y + 0.42, 0xb98a55, 0.2);
for (const s of FLOOR.receiving) pad(W, s.x - 0.62, s.x + 0.62, s.y - 0.42, s.y + 0.42, 0x4aa3ff);
for (const s of FLOOR.parking) pad(W, s.x - 0.6, s.x + 0.6, s.y - 2.4, s.y - 4.6, 0x8a94a3, 0.08);
label('STORAGE', 20.2, FLOOR.storage[0].y + 1.5, 0.5, '#3ecf8e', 1.0);
label('EMPTY PALLETS', 7.5, FLOOR.empties[0].y + 1.5, 0.5, '#b98a55', 0.8);
label('RECEIVING', 10.5, FLOOR.receiving[0].y + 1.5, 0.5, '#4aa3ff', 0.8);
label('PARKING', 24.2, FLOOR.parking[0].y - 1.0, 0.5, '#8a94a3', 0.8);

// ---------- forklift (AMR_SPEC §3.2) ----------
function wheel(parent, r, w, x, y, z) {
  const g = group(parent, x, y, z);
  add(g, cylGeo(r, w, 'y'), M.tyre);
  add(g, cylGeo(r * 0.55, w + 0.012, 'y', 16), M.steel);
  return g;
}
function buildForklift(id) {
  const body = group(W), B = FAMR.body, hw = B.w / 2;
  slab(body, B.xr, FAMR.chassisXf, -hw, hw, FAMR.groundClear, FAMR.chassisH, M.yellow);
  slab(body, B.xr, -0.55, -hw + 0.03, hw - 0.03, FAMR.chassisH, FAMR.hoodH, M.dark);
  add(body, cylGeo(0.09, 0.07, 'z', 16), M.dark, B.xr + 0.14, 0, FAMR.hoodH + 0.035);
  const beacon = add(body, cylGeo(0.05, 0.08, 'z', 12), new THREE.MeshBasicMaterial({ color: 0x5a4a1a }), -0.7, 0, FAMR.hoodH + 0.04);
  const wl = wheel(body, FAMR.rLoad, FAMR.wLoad, 0, FAMR.track / 2, FAMR.rLoad), wr = wheel(body, FAMR.rLoad, FAMR.wLoad, 0, -FAMR.track / 2, FAMR.rLoad);
  const steer = group(body, -FAMR.L, 0, FAMR.rDrive), drive = wheel(steer, FAMR.rDrive, FAMR.wDrive, 0, 0, 0);
  const mast = group(body, MAST.pivot[0], 0, MAST.pivot[1]);
  for (const e of [1, -1]) slab(mast, MAST.x0 - MAST.pivot[0], MAST.x1 - MAST.pivot[0], e * MAST.w / 2 - e * 0.09, e * MAST.w / 2, -0.25, MAST.height - MAST.pivot[1], M.dark);
  slab(mast, MAST.x0 - MAST.pivot[0], MAST.x1 - MAST.pivot[0], -MAST.w / 2, MAST.w / 2, MAST.height - MAST.pivot[1] - 0.1, MAST.height - MAST.pivot[1], M.dark);
  const car = group(mast), shift = group(car);
  slab(shift, 0.09, 0.15, -FFORK.carriageW / 2, FFORK.carriageW / 2, -0.05, 0.4, M.dark);
  for (let k = 0; k < 4; k++) slab(shift, 0.1, 0.13, -FFORK.carriageW / 2 + 0.02 + k * 0.28, -FFORK.carriageW / 2 + 0.06 + k * 0.28, 0.4, FFORK.backrestH, M.dark);
  for (const e of [1, -1]) slab(shift, MAST.aFace, MAST.aFace + FFORK.length, e * FFORK.tineY - FFORK.tineW / 2, e * FFORK.tineY + FFORK.tineW / 2, -FFORK.thick, 0, M.steel);
  const tag = label(id, 0, 0, 0, '#f2b01b', 0.45);
  return { body, wl, wr, steer, drive, mast, car, shift, beacon, tag };
}
// ---------- robot AMR (MOBMAN_SPEC §3): body > deck, pedestal > arm base {A} > DH chain ----------
function buildMobman(id) {
  const Bd = MAMR.body, hw = Bd.w / 2, body = group(W);
  slab(body, Bd.xr, Bd.xf, -hw, hw, MAMR.groundClear, MAMR.chassisH, M.blue);
  slab(body, Bd.xr + 0.02, Bd.xf - 0.02, -hw - 0.004, hw + 0.004, 0.1, 0.16, M.dark);
  const wl = wheel(body, MAMR.rWheel, MAMR.wWheel, 0, MAMR.track / 2, MAMR.rWheel), wr = wheel(body, MAMR.rWheel, MAMR.wWheel, 0, -MAMR.track / 2, MAMR.rWheel);
  const y0 = DECK.yc - DECK.w / 2, y1 = DECK.yc + DECK.w / 2;
  for (const y of [y0 - 0.04, y1]) slab(body, DECK.x0, DECK.x1, y, y + 0.04, MAMR.chassisH, DECK.z + DECK.guideH, M.conv);
  slab(body, DECK.x0, DECK.x1, y0, y1, MAMR.chassisH, DECK.z - 2 * DECK.rollerR - 0.01, M.dark);
  rollers(body, Math.floor((DECK.x1 - DECK.x0) / DECK.rollerPitch), DECK.rollerR, DECK.w, 'y', (i) => [DECK.x0 + 0.04 + i * DECK.rollerPitch, DECK.yc, DECK.z - DECK.rollerR]);
  slab(body, DECK.backX - 0.03, DECK.backX, y0, y1, DECK.z, DECK.z + DECK.stopH, M.orange);
  const stop = slab(body, DECK.stopX, DECK.stopX + 0.03, y0 + 0.05, y1 - 0.05, DECK.z - DECK.stopH, DECK.z, M.orange);
  const [bx, by, bz] = ARM.base;
  rod(body, 'z', MAMR.chassisH, bz, ARM.pedestalR, M.dark, bx, by);
  slab(body, bx - 0.17, bx + 0.17, by - 0.14, by + 0.14, MAMR.chassisH, MAMR.chassisH + 0.3, M.dark);
  const armBase = group(body, bx, by, bz);
  armBase.rotation.z = ARM.yaw;
  const J = [], F = [];
  let parent = armBase;
  for (const row of DH) { const j = group(parent), f = group(j, row.a, 0, row.d); f.rotation.x = row.alpha; J.push(j); F.push(f); parent = f; }
  const { d1, a2, a3, d4, d5, d6, offUpper: eU, offFore: eF } = ARM, [rU, rF, rW] = ARM.rLink;
  rod(J[0], 'z', 0, d1 - rU, 0.105, M.arm);
  rod(F[0], 'z', -rU, eU + rU * 0.9, rU, M.joint);
  rod(F[1], 'x', 0, a2, rU * 0.82, M.arm, 0, eU);
  rod(F[1], 'z', eF - rF, eU + rU * 0.9, rF * 1.08, M.joint);
  rod(F[2], 'x', 0, a3, rF * 0.85, M.arm, 0, eF);
  rod(F[2], 'z', eF - rF, d4 + rW, rW * 1.05, M.joint);
  rod(F[3], 'z', -rW, d5 + rW, rW, M.arm);
  rod(F[4], 'z', -rW, d6, rW, M.joint);
  rod(F[5], 'z', 0, MTOOL.tcp - 0.05, 0.028, M.dark);
  slab(F[5], -MTOOL.padX / 2, MTOOL.padX / 2, -MTOOL.padY / 2, MTOOL.padY / 2, MTOOL.tcp - 0.05, MTOOL.tcp - 0.025, M.steel);
  const cups = [];
  for (let i = 0; i < 3; i++) for (const e of [-1, 1]) cups.push(rod(F[5], 'z', MTOOL.tcp - 0.025, MTOOL.tcp, MTOOL.cupDia / 2, M.cup.clone(), (i - 1) * 0.085, e * 0.045));
  const tag = label(id, 0, 0, 0, '#7fb0ff', 0.45);
  return { body, wl, wr, stop, J, cups, tag };
}
const forkView = Array.from({ length: 6 }, (_, i) => buildForklift(`F${i + 1}`));
const mmView = Array.from({ length: 2 }, (_, i) => buildMobman(`M${i + 1}`));

// traffic claims overlay
const claimLines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xff5fa2, transparent: true, opacity: 0.7 }));
claimLines.frustumCulled = false; claimLines.visible = false;
W.add(claimLines);

// ---------- world ----------
let world, built = false;
function reset(forklifts = 3, robots = 2) {
  world = new World({ seed: 7, forklifts, robots });
  if (!built) {
    buildLanes(world);
    drawPath(RING, 0xf2b01b);
    for (const st of [...world.stations, ...world.parking]) { drawPath(st.spur.path, 0xf2b01b, 0.35); if (st.kind !== 'home') drawPath(st.spur.dockPath, 0xf2b01b, 0.35); }
    for (let i = 0; i < 4; i++) { const r = mmRoutes(i); drawPath(r.out, 0x4aa3ff, 0.6); drawPath(r.back, 0x4aa3ff, 0.6); drawPath(r.dock, 0x4aa3ff, 0.6); }
    drawPath(MM_LINK, 0x4aa3ff, 0.6);
    built = true;
  }
  forkView.forEach((v, i) => { v.body.visible = v.tag.visible = i < forklifts; });
  mmView.forEach((v, i) => { v.body.visible = v.tag.visible = i < robots; });
  const sel = $('pick'), keep = sel.value;
  sel.innerHTML = [...world.cells.map((c) => c.id), ...world.robots.map((r) => r.id), ...world.forklifts.map((f) => f.id), 'CRANE1', 'CRANE2', 'CRANE3', 'WRAP'].map((id) => `<option>${id}</option>`).join('');
  if ([...sel.options].some((o) => o.value === keep)) sel.value = keep;
  rackTick = 0; acc = 0;
}

// ---------- scene update ----------
let rackTick = 0;
function syncRack() {
  let n = 0;
  for (const a of world.asrs.aisles)
    for (const s of a.slots) {
      const p = s.pallet;
      if (!p || p.boxes) continue; // out of its location, or being handled: drawn with its boxes
      const x = A.tx + s.x, y = A.ty + s.y, hgt = p.layers * p.skuDef.h;
      setInst(rackPalIM, n, x, y, s.zs + PALLET.h / 2, PI / 2);
      setInst(rackLoadIM, n, x, y, s.zs + PALLET.h + hgt / 2, 0, 0.8, 1.2, hgt);
      rackLoadIM.setColorAt(n, skuColor[p.sku]);
      n++;
    }
  rackPalIM.count = rackLoadIM.count = n;
  rackPalIM.instanceMatrix.needsUpdate = rackLoadIM.instanceMatrix.needsUpdate = true;
  if (rackLoadIM.instanceColor) rackLoadIM.instanceColor.needsUpdate = true;
}
function sync() {
  const w = world;
  if (rackTick-- <= 0) { syncRack(); rackTick = 20; }
  // cranes
  w.asrs.aisles.forEach((a, i) => { const c = cranes[i]; c.base.position.x = a.q[0]; c.car.position.z = a.q[1]; c.mid.position.y = a.q[2] / 2; c.top.position.y = a.q[2]; });
  for (const row of ROWS) {
    const l = w.asrs.lanes[row], v = laneView[row];
    v.stops.forEach((s) => (s.position.z = v.top - 0.045 - 0.09 + 0.09 * l.stop + 0.04 * l.stop));
    v.lamp.material.color.setHex(l.locked ? 0x4aa3ff : l.ready ? 0x3ecf8e : l.state === 'EMPTY' ? 0xf0b429 : 0x343a44);
  }
  // palletizing cells
  w.cells.forEach((c, i) => {
    const v = cellView[i], ax = ['z', 'y', 'y', 'x', 'y', 'x'];
    for (let k = 0; k < 6; k++) v.robot.J[k].rotation[ax[k]] = c.q[k];
    for (const cup of v.robot.cups) cup.material.color.setHex(c.held ? 0x3ecf8e : 0x2b2f36);
    v.gate.position.z = PCELL.recv.z + (c.infeed.flight ? -0.03 : 0.025);
    v.lamp.material.color.setHex(c.locked ? 0x4aa3ff : c.palState === 'D_FULL' ? 0x3ecf8e : c.palState === 'BUILD' ? 0xf0b429 : 0x343a44);
  });
  // wrapper
  {
    const s = w.wrap.w, v = wrapView;
    v.table.rotation.z = s.th;
    v.jaws.position.z = CLAMP.stroke * s.clampUp;
    v.jaw.forEach((j, k) => (j.position.y = (k ? -1 : 1) * (0.0275 + 0.02 * (1 - s.jaw))));
    v.car.position.z = s.lift;
    v.slide.position.y = s.slide;
    v.film.visible = s.filmOnLoad;
    if (s.filmOnLoad) {
      const f = s.pallet.film, ln = f.line(s.th), c = f.world(f.active, s.th);
      v.film.geometry.attributes.position.array.set([ln.T[0], ln.T[1], s.lift, c[0], c[1], f.active.z]);
      v.film.geometry.attributes.position.needsUpdate = true;
    }
  }
  // vehicles
  w.forklifts.forEach((f, i) => {
    const v = forkView[i];
    v.body.position.set(f.pose.x, f.pose.y, 0); v.body.rotation.z = f.pose.th;
    v.steer.rotation.z = f.phi; v.drive.rotation.y = f.wheel.drive; v.wl.rotation.y = f.wheel.left; v.wr.rotation.y = f.wheel.right;
    v.mast.rotation.y = -f.tilt; v.car.position.z = f.lift - MAST.pivot[1]; v.shift.position.y = f.shift;
    v.beacon.material.color.setHex(f.drive.waitT > 0.5 ? 0xef4444 : Math.abs(f.v) > 1e-3 ? 0xf0b429 : 0x5a4a1a);
    v.tag.position.set(f.pose.x, f.pose.y, 2.6);
  });
  w.robots.forEach((r, i) => {
    const v = mmView[i];
    v.body.position.set(r.pose.x, r.pose.y, 0); v.body.rotation.z = r.pose.th;
    v.wl.rotation.y = r.wheel.L; v.wr.rotation.y = r.wheel.R;
    for (let k = 0; k < 6; k++) v.J[k].rotation.z = r.q[k];
    v.stop.position.z = DECK.z - DECK.stopH / 2 + DECK.stopH * r.deck.stop;
    for (const cup of v.cups) cup.material.color.setHex(r.held ? 0x3ecf8e : r.vac ? 0xf0b429 : 0x2b2f36);
    v.tag.position.set(r.pose.x, r.pose.y, 2.7);
  });
  // pallets outside their rack location, and every tracked box
  let n = 0, nf = 0;
  const film = (p, top) => { const z0 = p.z + PALLET.h - 0.08; if (nf < 40 && top > z0 + 0.02) setInst(filmIM, nf++, p.x, p.y, (z0 + top) / 2, p.yaw, 1.212, 0.812, top - z0); };
  for (const p of w.loose()) {
    if (n >= 160) break;
    setInst(palletIM, n++, p.x, p.y, p.z, p.yaw);
    if (p.wrapped) film(p, p.z + p.h + 0.04);
    else if (p.wrap && p.wrap.film) film(p, Math.min(p.z + p.h + 0.04, p.wrap.film.active.z + 0.225 - LINE.z + p.z));
  }
  for (const p of w.asrs.active()) { if (p.dom !== 'asrs' || n >= 160) continue; const g = palletG(p); setInst(palletIM, n++, g[0], g[1], g[2], p.yaw); }
  palletIM.count = n; palletIM.instanceMatrix.needsUpdate = true;
  filmIM.count = nf; filmIM.instanceMatrix.needsUpdate = true;
  n = 0;
  for (const b of w.asrs.boxes) {
    if (n >= 2500) break;
    setInst(boxIM, n, b.p[0], b.p[1], b.p[2], b.yaw, b.sx - 0.004, b.sy - 0.004, b.h - 0.003);
    boxIM.setColorAt(n++, tint.copy(skuColor[b.sku]).multiplyScalar(0.86 + 0.14 * (((b.id * 37) % 11) / 10)));
  }
  boxIM.count = n; boxIM.instanceMatrix.needsUpdate = true;
  if (boxIM.instanceColor) boxIM.instanceColor.needsUpdate = true;
  if (claimLines.visible) {
    const pts = [];
    for (const list of w.traffic.claims.values()) for (const c of list) for (let k = 0; k < c.poly.length; k++) { const a = c.poly[k], b = c.poly[(k + 1) % c.poly.length]; pts.push(a[0], a[1], 0.03, b[0], b[1], 0.03); }
    claimLines.geometry.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  }
}

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
let view = 'overview';
function setView(name) {
  view = name;
  document.querySelectorAll('[data-view]').forEach((b) => b.classList.toggle('on', b.dataset.view === name));
  if (!VIEWS[name]) return;
  camera.position.set(...VIEWS[name][0]); controls.target.set(...VIEWS[name][1]);
}
const tgt = new THREE.Vector3(), off = new THREE.Vector3(-6, 6, 7);
function chase() {
  if (view !== 'follow') return;
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

function resize() { renderer.setSize(innerWidth, innerHeight); camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); }
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
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
