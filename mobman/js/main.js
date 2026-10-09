// three.js view of the mobile depalletizer cell. All geometry lives in `W`, a Z-up group, so
// coordinates here are identical to MOBMAN_SPEC.md and sim.js. The arm is built as one group
// per Denavit–Hartenberg row (spec §3.3) — the scene graph IS the kinematic chain.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { AMR, ARM, TOOL, DECK, PALLET, RECIPES, SRC, DST, DOCK, DEG } from './config.js';
import * as A from './arm.js';
import { Cell, Q_TRAVEL, SRC_DOCK, DST_DOCK } from './sim.js';

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
scene.fog = new THREE.Fog(0x14171c, 40, 110);

const camera = new THREE.PerspectiveCamera(40, 1, 0.05, 300);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.maxPolarAngle = PI / 2 - 0.02;

scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x2a2622, 1.3));
const sun = new THREE.DirectionalLight(0xffffff, 2.1);
sun.position.set(-2, 20, 12);
sun.target.position.set(4, 0, 1);
sun.castShadow = true;
sun.shadow.mapSize.set(4096, 4096);
Object.assign(sun.shadow.camera, { left: -11, right: 11, top: 10, bottom: -10, near: 1, far: 60 });
sun.shadow.bias = -0.0005;
scene.add(sun, sun.target);

const W = new THREE.Group(); // world frame {W}: Z up
W.rotation.x = -PI / 2;
scene.add(W);

// ---------- helpers (all in Z-up coordinates) ----------
const mat = (color, o = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.2, ...o });
const M = {
  body: mat(0xf2b01b, { roughness: 0.45 }),
  dark: mat(0x23272e, { roughness: 0.6, metalness: 0.4 }),
  steel: mat(0x9aa3b0, { roughness: 0.35, metalness: 0.7 }),
  roller: mat(0xb9c0cb, { roughness: 0.3, metalness: 0.8 }),
  tyre: mat(0x16181c, { roughness: 0.9, metalness: 0 }),
  conv: mat(0x4a5361, { metalness: 0.5 }),
  stop: mat(0xe0742a, { roughness: 0.5, metalness: 0.3 }),
  wood: mat(0xb98a55, { roughness: 0.85, metalness: 0 }),
  box: mat(0xffffff, { roughness: 0.88, metalness: 0 }),
  arm: mat(0xd9dee6, { roughness: 0.4, metalness: 0.3 }),
  joint: mat(0x4aa3ff, { roughness: 0.4, metalness: 0.3 }),
  cup: mat(0x2b2f36, { roughness: 0.9, metalness: 0 }),
};
function add(parent, geo, material, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(geo, material);
  m.position.set(x, y, z);
  m.castShadow = m.receiveShadow = true;
  parent.add(m);
  return m;
}
// box given by its extents
const slab = (p, x0, x1, y0, y1, z0, z1, m) => add(p, new THREE.BoxGeometry(x1 - x0, y1 - y0, z1 - z0), m, (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2);
function cylGeo(r, len, axis, seg = 28) {
  const g = new THREE.CylinderGeometry(r, r, len, seg);
  if (axis === 'x') g.rotateZ(-PI / 2);
  if (axis === 'z') g.rotateX(PI / 2);
  return g;
}
// cylinder along one axis of the parent frame, from a to b on that axis, at offsets (u, v) on the other two
function rod(parent, axis, a, b, r, material, u = 0, v = 0) {
  const m = add(parent, cylGeo(r, Math.abs(b - a), axis), material);
  const mid = (a + b) / 2;
  if (axis === 'x') m.position.set(mid, u, v);
  else if (axis === 'y') m.position.set(u, mid, v);
  else m.position.set(u, v, mid);
  return m;
}
const group = (parent, x = 0, y = 0, z = 0) => {
  const g = new THREE.Group();
  g.position.set(x, y, z);
  parent.add(g);
  return g;
};
function label(parent, text, x, y, z, color = '#d9dee6', size = 1) {
  const c = document.createElement('canvas');
  c.width = 512; c.height = 96;
  const g = c.getContext('2d');
  g.font = '700 54px ui-monospace, Consolas, monospace';
  g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillStyle = color;
  g.fillText(text, 256, 50);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthWrite: false }));
  s.position.set(x, y, z);
  s.scale.set(2.0 * size, 0.375 * size, 1);
  parent.add(s);
  return s;
}
function polyline(parent, pts, color, opacity = 1) {
  const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts.map((p) => new THREE.Vector3(...p))), new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity }));
  parent.add(l);
  return l;
}
const rect = (parent, x0, x1, y0, y1, color, z = 0.006) => polyline(parent, [[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z], [x0, y0, z]], color);
function pad(parent, x0, x1, y0, y1, color, opacity = 0.1) {
  const m = new THREE.Mesh(new THREE.PlaneGeometry(x1 - x0, y1 - y0), new THREE.MeshBasicMaterial({ color, transparent: true, opacity }));
  m.position.set((x0 + x1) / 2, (y0 + y1) / 2, 0.004);
  parent.add(m);
  return m;
}
function rollers(parent, n, r, len, axis, at) {
  const im = new THREE.InstancedMesh(cylGeo(r, len, axis, 14), M.roller, n), m4 = new THREE.Matrix4();
  for (let i = 0; i < n; i++) im.setMatrixAt(i, m4.makeTranslation(...at(i)));
  im.castShadow = im.receiveShadow = true;
  parent.add(im);
  return im;
}
function axes(parent, size = 0.18) {
  const g = new THREE.AxesHelper(size);
  g.material.depthTest = false;
  g.renderOrder = 5;
  parent.add(g);
  return g;
}

// ---------- floor ----------
{
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(160, 160), mat(0x1d2127, { roughness: 0.95, metalness: 0 }));
  floor.receiveShadow = true;
  W.add(floor);
  const grid = new THREE.GridHelper(40, 40, 0x39404b, 0x272c34);
  grid.rotation.x = PI / 2;
  grid.position.set(4, 0, 0.003);
  W.add(grid);
}

// ---------- source station (spec §10.1) ----------
const srcView = (() => {
  const g = group(W), x0 = SRC.endX - SRC.len, x1 = SRC.endX, hw = SRC.frameW / 2, y = SRC.y, z = SRC.z;
  for (const e of [1, -1]) {
    slab(g, x0, x1, y + e * hw - 0.04, y + e * hw, z - 0.14, z + 0.005, M.conv); // side beams
    for (let x = x0 + 0.1; x < x1; x += 1.0) slab(g, x - 0.03, x + 0.03, y + e * hw - 0.05, y + e * hw, 0, z - 0.14, M.conv); // feet
  }
  const n = Math.floor(SRC.len / 0.12);
  rollers(g, n, 0.04, SRC.frameW - 0.1, 'y', (i) => [x0 + 0.08 + i * 0.12, y, z - 0.04]);
  slab(g, SRC.x + PALLET.sx / 2 + 0.02, x1, y - hw, y + hw, z - 0.14, z + 0.11, M.stop); // end stop
  // B1 pallet present: beam across the conveyor through the leading block row
  for (const e of [1, -1]) slab(g, SRC.b1x - 0.02, SRC.b1x + 0.02, y + e * (hw + 0.06) - 0.02, y + e * (hw + 0.06) + 0.02, 0, z + SRC.b1z + 0.05, M.dark);
  const b1 = polyline(g, [[SRC.b1x, y - hw - 0.06, z + SRC.b1z], [SRC.b1x, y + hw + 0.06, z + SRC.b1z]], 0x343a44).material;
  // signal column: green = PalletReady, blue = Locked
  slab(g, x1 + 0.08, x1 + 0.12, y + hw + 0.06, y + hw + 0.10, 0, 2.0, M.dark);
  const lampGeo = cylGeo(0.035, 0.09, 'z', 16);
  const lamps = [2.05, 2.15].map((h) => add(g, lampGeo, new THREE.MeshBasicMaterial({ color: 0x343a44 }), x1 + 0.10, y + hw + 0.08, h));
  label(W, 'SOURCE', SRC.x - 1.6, y + hw + 0.5, 0.5, '#4aa3ff');
  const zone = pad(W, AMR.body.xr, DOCK.standoff + AMR.body.xf, -AMR.body.w / 2 - 0.1, AMR.body.w / 2 + 0.05, 0x4aa3ff);
  return { b1, lamps, zone };
})();

// ---------- destination conveyor (spec §10.2) ----------
const dstView = (() => {
  const g = group(W), y1 = DST.yEnd, y0 = DST.yEnd - DST.len, hw = DST.w / 2, x = DST.x, z = DST.z;
  for (const e of [1, -1]) {
    slab(g, x + e * hw, x + e * (hw + 0.04), y0, y1, z - 0.12, z + 0.05, M.conv);
    for (let y = y0 + 0.1; y < y1; y += 1.2) slab(g, x + e * hw, x + e * (hw + 0.05), y - 0.03, y + 0.03, 0, z - 0.12, M.conv);
  }
  const n = Math.floor(DST.len / DECK.rollerPitch);
  rollers(g, n, DECK.rollerR, DST.w, 'x', (i) => [x, y1 - 0.04 - i * DECK.rollerPitch, z - DECK.rollerR]);
  for (const e of [1, -1]) slab(g, x + e * (hw + 0.07) - 0.02, x + e * (hw + 0.07) + 0.02, y1 - DST.inX - 0.02, y1 - DST.inX + 0.02, 0, z + 0.12, M.dark);
  const b5 = polyline(g, [[x - hw - 0.07, y1 - DST.inX, z + 0.07], [x + hw + 0.07, y1 - DST.inX, z + 0.07]], 0x343a44).material;
  slab(g, x + hw + 0.10, x + hw + 0.14, y1 - 0.06, y1 - 0.02, 0, 1.5, M.dark);
  const lamp = add(g, cylGeo(0.035, 0.09, 'z', 16), new THREE.MeshBasicMaterial({ color: 0x343a44 }), x + hw + 0.12, y1 - 0.04, 1.55);
  label(W, 'DESTINATION', x + 1.9, y1 - 1.2, 0.5, '#3ecf8e');
  const hb = AMR.body.w / 2;
  const zone = pad(W, DST_DOCK.x - hb - 0.05, DST_DOCK.x + hb + 0.05, DST_DOCK.y - AMR.body.xf, DST_DOCK.y + DOCK.standoff - AMR.body.xr, 0x3ecf8e);
  return { b5, lamp, zone };
})();

// ---------- AMR (spec §3): body > wheels, deck, pedestal > arm base {A} > DH chain ----------
function wheel(parent, r, w, x, y, z) {
  const g = group(parent, x, y, z);
  add(g, cylGeo(r, w, 'y'), M.tyre);
  add(g, cylGeo(r * 0.55, w + 0.012, 'y', 20), M.steel);
  slab(g, -r * 0.5, r * 0.5, -w / 2 - 0.009, w / 2 + 0.009, -0.012, 0.012, M.dark); // spoke mark, makes the rolling visible
  return g;
}
function buildAmr() {
  const Bd = AMR.body, hw = Bd.w / 2, body = group(W), xc = (DECK.x0 + DECK.x1) / 2;
  slab(body, Bd.xr, Bd.xf, -hw, hw, AMR.groundClear, AMR.chassisH, M.body);
  slab(body, Bd.xr + 0.02, Bd.xf - 0.02, -hw - 0.004, hw + 0.004, 0.10, 0.16, M.dark); // bumper strip
  for (const [x, y] of [[Bd.xf - 0.07, hw - 0.07], [Bd.xr + 0.07, -hw + 0.07]]) add(body, cylGeo(0.07, 0.06, 'z', 20), M.dark, x, y, AMR.chassisH + 0.03); // scanners
  const wl = wheel(body, AMR.rWheel, AMR.wWheel, 0, AMR.track / 2, AMR.rWheel);
  const wr = wheel(body, AMR.rWheel, AMR.wWheel, 0, -AMR.track / 2, AMR.rWheel);
  for (const [x, y] of AMR.casters) add(body, cylGeo(AMR.rCaster, 0.04, 'y', 16), M.tyre, x, y, AMR.rCaster);
  // deck: roller conveyor on a sub-frame
  const y0 = DECK.yc - DECK.w / 2, y1 = DECK.yc + DECK.w / 2;
  for (const y of [y0 - 0.04, y1]) slab(body, DECK.x0, DECK.x1, y, y + 0.04, AMR.chassisH, DECK.z + DECK.guideH, M.conv);
  slab(body, DECK.x0, DECK.x1, y0, y1, AMR.chassisH, DECK.z - 2 * DECK.rollerR - 0.01, M.dark);
  const nR = Math.floor((DECK.x1 - DECK.x0) / DECK.rollerPitch);
  rollers(body, nR, DECK.rollerR, DECK.w, 'y', (i) => [DECK.x0 + 0.04 + i * DECK.rollerPitch, DECK.yc, DECK.z - DECK.rollerR]);
  slab(body, DECK.backX - 0.03, DECK.backX, y0, y1, DECK.z, DECK.z + DECK.stopH, M.stop); // fixed rear stop
  const stop = slab(body, DECK.stopX, DECK.stopX + 0.03, y0 + 0.05, y1 - 0.05, DECK.z - DECK.stopH, DECK.z, M.stop); // pop-up front stop (c2)
  const drive = add(body, cylGeo(0.03, 0.05, 'z', 16), new THREE.MeshBasicMaterial({ color: 0x343a44 }), xc, y0 - 0.02, DECK.z + DECK.guideH + 0.03);
  const beacon = add(body, cylGeo(0.035, 0.06, 'z', 16), new THREE.MeshBasicMaterial({ color: 0xf0b429 }), Bd.xr + 0.12, hw - 0.12, AMR.chassisH + 0.03);
  // pedestal and arm base frame {A}
  const [bx, by, bz] = ARM.base;
  rod(body, 'z', AMR.chassisH, bz, ARM.pedestalR, M.dark, bx, by);
  slab(body, bx - 0.17, bx + 0.17, by - 0.14, by + 0.14, AMR.chassisH, AMR.chassisH + 0.30, M.dark); // controller cabinet
  const armBase = group(body, bx, by, bz);
  armBase.rotation.z = ARM.yaw;

  // DH chain: J[i] turns by q[i] about its Z; F[i] is frame i+1 = Tz(d)·Tx(a)·Rx(α) inside it
  const J = [], F = [];
  let parent = armBase;
  for (const row of A.DH) {
    const j = group(parent), f = group(j, row.a, 0, row.d);
    f.rotation.x = row.alpha;
    J.push(j); F.push(f);
    parent = f;
  }
  const { d1, a2, a3, d4, d5, d6, offUpper: eU, offFore: eF } = ARM, [rU, rF, rW] = ARM.rLink;
  rod(J[0], 'z', 0, d1 - rU, 0.105, M.arm); // base housing, turns with J1
  rod(F[0], 'z', -rU, eU + rU * 0.9, rU, M.joint); // shoulder (J2 axis)
  rod(F[1], 'x', 0, a2, rU * 0.82, M.arm, 0, eU); // upper arm, beside the J1 plane
  rod(F[1], 'z', eF - rF, eU + rU * 0.9, rF * 1.08, M.joint); // elbow (J3 axis)
  rod(F[2], 'x', 0, a3, rF * 0.85, M.arm, 0, eF); // forearm
  rod(F[2], 'z', eF - rF, d4 + rW, rW * 1.05, M.joint); // wrist 1 (J4 axis)
  rod(F[3], 'z', -rW, d5 + rW, rW, M.arm); // wrist 2 (J5 axis)
  rod(F[4], 'z', -rW, d6, rW, M.joint); // wrist 3 (J6 axis)
  // tool on the flange (frame 6): stem, pad, cups; TCP on the cup plane
  rod(F[5], 'z', 0, 0.012, 0.045, M.steel);
  rod(F[5], 'z', 0.012, TOOL.tcp - 0.05, 0.028, M.dark);
  slab(F[5], -TOOL.padX / 2, TOOL.padX / 2, -TOOL.padY / 2, TOOL.padY / 2, TOOL.tcp - 0.05, TOOL.tcp - 0.025, M.steel);
  const cups = [];
  for (let i = 0; i < 3; i++) for (const e of [-1, 1]) cups.push(rod(F[5], 'z', TOOL.tcp - 0.025, TOOL.tcp, TOOL.cupDia / 2, M.cup.clone(), (i - 1) * 0.085, e * 0.045));
  const tcp = add(F[5], new THREE.SphereGeometry(0.016, 16, 12), new THREE.MeshBasicMaterial({ color: 0x3ecf8e }), 0, 0, TOOL.tcp);
  tcp.castShadow = false;
  const frames = [axes(body, 0.4), axes(armBase, 0.3), ...F.map((f) => axes(f, 0.16))];
  // shoulder singularity: the cylinder of radius d4 around the J1 axis that O5 must stay out of (spec §7.1)
  const singGeo = new THREE.CylinderGeometry(d4, d4, 1.5, 40, 1, true).rotateX(PI / 2);
  const sing = new THREE.Mesh(singGeo, new THREE.MeshBasicMaterial({ color: 0xff5fa2, transparent: true, opacity: 0.07, side: THREE.DoubleSide, depthWrite: false }));
  sing.position.z = 0.25;
  armBase.add(sing);
  return { body, wl, wr, stop, drive, beacon, J, F, cups, frames, sing };
}
const R = buildAmr();
R.frames.forEach((f) => (f.visible = false));

// O5 marker and its radius line to the J1 axis: the shoulder margin made visible
const o5Dot = new THREE.Mesh(new THREE.SphereGeometry(0.022, 16, 12), new THREE.MeshBasicMaterial({ color: 0xff5fa2 }));
const o5Line = new THREE.Line(new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3)), new THREE.LineBasicMaterial({ color: 0xff5fa2 }));
o5Line.frustumCulled = false;
W.add(o5Dot, o5Line);
// ICR of the base
const icrLine = new THREE.Line(new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(new Float32Array(9), 3)), new THREE.LineBasicMaterial({ color: 0xf2b01b, transparent: true, opacity: 0.9 }));
icrLine.frustumCulled = false;
const icrDot = new THREE.Mesh(new THREE.SphereGeometry(0.05, 16, 12), new THREE.MeshBasicMaterial({ color: 0xf2b01b }));
W.add(icrLine, icrDot);

// route on the floor
const pathGroup = group(W);
function drawRoute(cell) {
  pathGroup.clear();
  const p = cell.path, n = Math.ceil(p.length / 0.05);
  polyline(pathGroup, Array.from({ length: n + 1 }, (_, i) => { const a = p.at((p.length * i) / n); return [a.x, a.y, 0.008]; }), 0xf2b01b, 0.8);
  for (const s of [cell.sPreSrc, cell.sPreDst]) { const a = p.at(s); add(pathGroup, cylGeo(0.06, 0.004, 'z', 20), new THREE.MeshBasicMaterial({ color: 0xf2b01b }), a.x, a.y, 0.008); }
}

// TCP trail
const TRAIL_N = 3000;
const trailPos = new Float32Array(TRAIL_N * 3);
const trail = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x3ecf8e, transparent: true, opacity: 0.75 }));
trail.geometry.setAttribute('position', new THREE.BufferAttribute(trailPos, 3));
trail.frustumCulled = false;
W.add(trail);
let trailLen = 0;

// ---------- unit loads: the pallet, and every box as one instance ----------
const palletGeo = (() => {
  const parts = [], P = PALLET;
  const b = (sx, sy, sz, x, y, z) => parts.push(new THREE.BoxGeometry(sx, sy, sz).translate(x, y, z));
  for (const [y, w] of [[-0.3275, 0.145], [-0.1638, 0.1], [0, 0.145], [0.1638, 0.1], [0.3275, 0.145]]) b(P.sx, w, 0.022, 0, y, P.h - 0.011); // deck boards
  for (const x of [-0.5275, 0, 0.5275]) b(0.145, P.sy, 0.022, x, 0, 0.111); // stringer boards
  for (const x of [-0.5275, 0, 0.5275]) for (const y of [-0.35, 0, 0.35]) b(0.145, y ? 0.1 : 0.145, 0.078, x, y, 0.061); // blocks
  for (const y of [-0.35, 0, 0.35]) b(P.sx, y ? 0.1 : 0.145, 0.022, 0, y, 0.011); // bottom boards
  return mergeGeometries(parts);
})();
const palletMesh = add(W, palletGeo, M.wood);
const CAP_B = 400;
const boxMesh = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), M.box, CAP_B);
boxMesh.castShadow = boxMesh.receiveShadow = true;
boxMesh.frustumCulled = false;
boxMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
W.add(boxMesh);
const rcColor = RECIPES.map((r) => new THREE.Color(r.color));
const tint = new THREE.Color(), m4 = new THREE.Matrix4(), quat = new THREE.Quaternion(), v3 = new THREE.Vector3(), scl = new THREE.Vector3(), zAxis = new THREE.Vector3(0, 0, 1);
function drawLoads() {
  const p = cell.src.pallet;
  palletMesh.visible = !!p;
  if (p) { palletMesh.position.set(p.x, p.y, p.z); palletMesh.rotation.z = p.yaw; }
  let n = 0;
  for (const b of cell.boxes) {
    if (n >= CAP_B) break;
    m4.compose(v3.set(b.p[0], b.p[1], b.p[2]), quat.setFromAxisAngle(zAxis, b.yaw), scl.set(b.sx - 0.004, b.sy - 0.004, b.h - 0.003));
    boxMesh.setMatrixAt(n, m4);
    boxMesh.setColorAt(n++, tint.copy(rcColor[b.rcIx]).multiplyScalar(0.88 + 0.12 * (((b.id * 37) % 11) / 10)));
  }
  boxMesh.count = n;
  boxMesh.instanceMatrix.needsUpdate = true;
  if (boxMesh.instanceColor) boxMesh.instanceColor.needsUpdate = true;
}

let cell;
function reset() {
  cell = new Cell({ seed: 7, clearEvery: 4 });
  drawRoute(cell);
  trailLen = 0;
  acc = 0;
}

// ---------- HUD ----------
const $ = (id) => document.getElementById(id);
const JNAMES = ['Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3'];
const axisRows = JNAMES.map((name, i) => {
  const row = document.createElement('div');
  row.className = 'j';
  row.innerHTML = `<span>J${i + 1}</span><span class="nm">${name}</span><span class="val"></span><div class="bar pos"><i></i></div><div class="bar"><i></i></div>`;
  $('axes').appendChild(row);
  return { val: row.querySelector('.val'), pos: row.querySelectorAll('i')[0], spd: row.querySelectorAll('i')[1] };
});
const lamp = (id, on, cls = 'on') => { $(id).className = 'lamp ' + (on ? cls : ''); };
const sg = (v, n) => (v >= 0 ? ' ' : '') + v.toFixed(n);
function margin(bar, txt, value, limit, full, unit) {
  const el = $(bar);
  el.firstElementChild.style.width = `${Math.min(100, (100 * value) / full)}%`;
  el.className = 'bar' + (value < limit ? ' bad' : value < 1.6 * limit ? ' warn' : '');
  $(txt).textContent = `${value.toFixed(3)}${unit} ≥ ${limit}`;
}

function hud() {
  const c = cell, L = c.last, st = c.stats;
  $('phase').textContent = c.phase.replace(/_/g, ' ');
  $('seg').textContent = c.seg ? c.seg.label : '–';
  const o = c.order;
  $('order').textContent = o ? `${o.rc.id} ${o.rc.sku} · ${o.done} / ${o.n} boxes` : '–';
  $('deck').textContent = `${c.deck.boxes.length} boxes · ${c.deck.boxes.reduce((s, b) => s + b.m, 0)} kg`;
  c.q.forEach((q, i) => {
    const r = axisRows[i];
    r.val.textContent = `${sg(q / DEG, 1)}°`;
    r.pos.style.left = `calc(${(100 * (q - ARM.qMin[i])) / (ARM.qMax[i] - ARM.qMin[i])}% - 1.5px)`;
    r.spd.style.width = `${Math.min(100, (100 * Math.abs(c.qd[i])) / ARM.vMax[i])}%`;
  });
  const s = L.sing;
  margin('bE', 'mE', Math.abs(s.elbow), ARM.minElbow, 1, '');
  margin('bW', 'mW', Math.abs(s.wrist), ARM.minWrist, 1, '');
  margin('bS', 'mS', Math.abs(s.shoulder), ARM.minShoulder, 1.2, ' m');
  $('detJ').textContent = (ARM.a2 * ARM.a3 * s.elbow * s.wrist * s.shoulder).toFixed(4);
  $('tcp').textContent = L.tcp.map((v) => v.toFixed(3)).join('  ');
  $('tool').textContent = `${(L.tilt / DEG).toFixed(3)}° / ${isFinite(L.sf) ? L.sf.toFixed(2) : '–'}`;
  const mm = (d) => (isFinite(d) ? `${(1000 * d).toFixed(0)} mm` : '–');
  $('clr').textContent = `${mm(L.armClear.d)} / ${mm(L.carry.d)}`;

  $('pose').textContent = `${c.pose.x.toFixed(3)}  ${c.pose.y.toFixed(3)}  ${(c.pose.th / DEG).toFixed(1)}°`;
  $('spd').textContent = `${sg(c.v, 3)} m/s / ${sg(c.w / DEG, 1)}°/s`;
  $('whl').textContent = `${sg(c.wheel.vL, 3)} / ${sg(c.wheel.vR, 3)} m/s`;
  $('icr').textContent = Math.abs(c.k) > 1e-4 ? `${(1 / Math.abs(c.k)).toFixed(2)} m ${c.k > 0 ? 'left' : 'right'}` : '∞ (straight)';
  $('alat').textContent = `${(c.v * c.v * Math.abs(c.k)).toFixed(2)} / ${AMR.aLat.toFixed(1)} m/s²`;
  $('stab').textContent = `${L.stab.margin.toFixed(3)} m · ${L.stab.m.toFixed(0)} kg`;
  $('odo').textContent = `${(1000 * Math.hypot(c.odo.x - c.pose.x, c.odo.y - c.pose.y)).toFixed(3)} mm`;

  lamp('lB1', c.src.b1, 'warn'); lamp('lB2', c.src.b2, 'warn'); lamp('lReady', c.src.ready); lamp('lLock', c.src.locked, 'info');
  lamp('lVac', c.vac, 'warn'); lamp('lHeld', !!c.held, 'warn'); lamp('lStow', L.armTravel); lamp('lStand', Math.abs(c.v) < 1e-6);
  lamp('lStop', c.deck.stop === 1); lamp('lRoll', c.deck.u > 0, 'warn'); lamp('lB5', c.dstInfeedClear); lamp('lDst', c.dst.reserved, 'info');
  const bad = L.bad.length > 0;
  $('lMon').className = 'lamp ' + (bad ? 'bad' : 'on');
  $('mon').textContent = bad ? L.bad[0] : `OK  (${st.violations} violations)`;

  const p = c.src.pallet;
  $('pal').textContent = p ? `#${p.id} ${p.rc.id} ${p.rc.sku} · ${p.boxes.length - p.next} of ${p.boxes.length} boxes` : 'being exchanged';
  $('count').textContent = `${st.boxes} / ${st.delivered}`;
  $('thr').textContent = `${st.trips} / ${c.t > 120 ? ((st.boxes * 3600) / c.t).toFixed(0) + ' boxes/h' : '–'}`;
  $('cyc').textContent = `${st.boxCycle.length ? st.boxCycle.at(-1).toFixed(1) + ' s' : '–'} / ${st.tripCycle.length ? st.tripCycle.at(-1).toFixed(0) + ' s' : '–'}`;
  const sec = Math.floor(c.t);
  $('time').textContent = `${String(Math.floor(sec / 3600)).padStart(2, '0')}:${String(Math.floor(sec / 60) % 60).padStart(2, '0')}:${String(sec % 60).padStart(2, '0')}`;
}

// ---------- scene update from sim state ----------
let showTrail = true, showSing = true, view = 'overview';
const wp = new THREE.Vector3();
function sync() {
  const c = cell;
  R.body.position.set(c.pose.x, c.pose.y, 0);
  R.body.rotation.z = c.pose.th;
  R.wl.rotation.y = c.wheel.L;
  R.wr.rotation.y = c.wheel.R;
  for (let i = 0; i < 6; i++) R.J[i].rotation.z = c.q[i]; // the six joints
  R.stop.position.z = DECK.z - DECK.stopH / 2 + DECK.stopH * c.deck.stop; // c2
  R.drive.material.color.setHex(c.deck.u > 0 ? 0x3ecf8e : 0x343a44);
  R.beacon.material.color.setHex(Math.abs(c.v) > 1e-4 && Math.floor(performance.now() / 250) % 2 ? 0xf0b429 : 0x5a4a1a);
  for (const cup of R.cups) cup.material.color.setHex(c.held ? 0x3ecf8e : c.vac ? 0xf0b429 : 0x2b2f36);

  srcView.b1.color.setHex(c.src.b1 ? 0xf0b429 : 0x3a4250);
  srcView.lamps[0].material.color.setHex(c.src.ready ? 0x3ecf8e : 0x343a44);
  srcView.lamps[1].material.color.setHex(c.src.locked ? 0x4aa3ff : 0x343a44);
  srcView.zone.material.opacity = c.src.locked ? 0.22 : 0.08;
  dstView.b5.color.setHex(c.dstInfeedClear ? 0x3a4250 : 0xf0b429);
  dstView.lamp.material.color.setHex(c.dst.reserved ? 0x4aa3ff : c.dstInfeedClear ? 0x3ecf8e : 0x343a44);
  dstView.zone.material.opacity = c.dst.reserved ? 0.22 : 0.08;
  drawLoads();

  // shoulder singularity cylinder, O5 and its distance to the J1 axis
  const o5 = A.fk(c.q).o5, cz = Math.cos(c.pose.th + ARM.yaw), sz = Math.sin(c.pose.th + ARM.yaw);
  const ab = [c.pose.x + Math.cos(c.pose.th) * ARM.base[0] - Math.sin(c.pose.th) * ARM.base[1], c.pose.y + Math.sin(c.pose.th) * ARM.base[0] + Math.cos(c.pose.th) * ARM.base[1]];
  const o5w = [ab[0] + cz * o5[0] - sz * o5[1], ab[1] + sz * o5[0] + cz * o5[1], ARM.base[2] + o5[2]];
  R.sing.visible = o5Dot.visible = o5Line.visible = showSing;
  o5Dot.position.set(...o5w);
  o5Line.geometry.attributes.position.array.set([ab[0], ab[1], o5w[2], ...o5w]);
  o5Line.geometry.attributes.position.needsUpdate = true;

  const on = Math.abs(c.k) > 1e-3 && Math.abs(c.v) > 1e-4;
  icrLine.visible = icrDot.visible = on && pathGroup.visible;
  if (on) {
    const co = Math.cos(c.pose.th), si = Math.sin(c.pose.th), r = 1 / c.k, h = AMR.track / 2, z = 0.02;
    const icr = [c.pose.x - si * r, c.pose.y + co * r, z];
    icrLine.geometry.attributes.position.array.set([c.pose.x - si * h, c.pose.y + co * h, z, ...icr, c.pose.x + si * h, c.pose.y - co * h, z]);
    icrLine.geometry.attributes.position.needsUpdate = true;
    icrDot.position.set(...icr);
  }

  if (showTrail && cell.armBusy) {
    const n = Math.min(trailLen, TRAIL_N - 1);
    trailPos.copyWithin(3, 0, n * 3);
    trailPos.set(c.last.tcp, 0);
    trailLen = n + 1;
    trail.geometry.setDrawRange(0, trailLen);
    trail.geometry.attributes.position.needsUpdate = true;
  }
  trail.visible = showTrail;
}

// ---------- cameras (three coordinates: x, up, −worldY) ----------
const VIEWS = {
  overview: [[-3.5, 7.5, 9.5], [4.0, 0.6, 1.2]],
  top: [[4.0, 22, 1.21], [4.0, 0, 1.2]],
  arm: [[3.4, 3.1, 2.6], [0.0, 1.15, -0.6]],
  source: [[-2.6, 2.6, 2.2], [0.1, 1.0, -0.8]],
  dest: [[11.2, 3.2, 1.2], [8.0, 0.7, 4.4]],
};
function setView(name) {
  view = name;
  document.querySelectorAll('[data-view]').forEach((b) => b.classList.toggle('on', b.dataset.view === name));
  if (!VIEWS[name]) return;
  camera.position.set(...VIEWS[name][0]);
  controls.target.set(...VIEWS[name][1]);
}
const tgt = new THREE.Vector3(), off = new THREE.Vector3(-4.2, 3.6, 5.2);
function chaseCam() {
  if (view !== 'follow') return;
  tgt.set(cell.pose.x, 1.0, -cell.pose.y);
  controls.target.lerp(tgt, 0.12);
  camera.position.lerp(tgt.clone().add(off), 0.06);
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
$('paths').onclick = (e) => { pathGroup.visible = !pathGroup.visible; e.target.classList.toggle('on', pathGroup.visible); };
$('frames').onclick = (e) => { const on = !R.frames[0].visible; R.frames.forEach((f) => (f.visible = on)); e.target.classList.toggle('on', on); };
$('sing').onclick = (e) => { showSing = !showSing; e.target.classList.toggle('on', showSing); };
$('trail').onclick = (e) => { showTrail = !showTrail; e.target.classList.toggle('on', showTrail); trailLen = 0; };

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
// Integration handle: window.mobman.cell is the Cell (see MOBMAN_SPEC.md).
window.mobman = {
  get cell() { return cell; }, setView, camera, controls, R, Q_TRAVEL, SRC_DOCK, sync,
  step: (n = 1) => { for (let i = 0; i < n; i++) cell.step(DT); },
  // world position of the rendered TCP marker, for checking the scene graph against arm.js
  tcpScene: () => { R.body.updateWorldMatrix(true, true); const p = R.F[5].localToWorld(wp.set(0, 0, TOOL.tcp)); return [p.x, -p.z, p.y]; },
};

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
