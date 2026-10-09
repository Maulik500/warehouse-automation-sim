// three.js view of the pallet AMR cell. All geometry lives in `W`, a Z-up group, so
// coordinates here are identical to AMR_SPEC.md and sim.js.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { AMR, MAST, FORK, PALLET, SKUS, CONV, DOCK, CELL, DEG } from './config.js';
import * as K from './kinematics.js';
import { Cell, stationPoses } from './sim.js';

const PI = Math.PI;
const [XT, ZT] = MAST.pivot;

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
sun.position.set(-6, 22, 12);
sun.target.position.set(3, 0, 0);
sun.castShadow = true;
sun.shadow.mapSize.set(4096, 4096);
Object.assign(sun.shadow.camera, { left: -13, right: 13, top: 12, bottom: -12, near: 1, far: 60 });
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
  fork: mat(0xd9dee6, { roughness: 0.3, metalness: 0.8 }),
  tyre: mat(0x16181c, { roughness: 0.9, metalness: 0 }),
  conv: mat(0x4a5361, { metalness: 0.5 }),
  chain: mat(0x6b7480, { roughness: 0.4, metalness: 0.8 }),
  stop: mat(0xe0742a, { roughness: 0.5, metalness: 0.3 }),
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
// box given by its extents
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
const outline = (parent, cx, cy, sx, sy, color, z = 0.006) =>
  polyline(parent, [[-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1]].map(([a, b]) => [cx + (a * sx) / 2, cy + (b * sy) / 2, z]), color);

// ---------- floor ----------
{
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(160, 160), mat(0x1d2127, { roughness: 0.95, metalness: 0 }));
  floor.receiveShadow = true;
  W.add(floor);
  const grid = new THREE.GridHelper(40, 40, 0x39404b, 0x272c34);
  grid.rotation.x = PI / 2;
  grid.position.set(3, 0, 0.003);
  W.add(grid);
}

// ---------- conveyors and pick stations (spec §6) ----------
const beam = (parent, x, y0, y1, z) => {
  for (const y of [y0, y1]) slab(parent, x - 0.02, x + 0.02, y - 0.02, y + 0.02, 0, z + 0.04, M.dark);
  const l = polyline(parent, [[x, y0, z], [x, y1, z]], 0x343a44);
  return l.material;
};
const lineViews = CONV.lines.map((cfg) => {
  const g = group(W, 0, cfg.y, 0);
  const xFace = CONV.xP - PALLET.sx / 2, x0 = xFace - CONV.endOverhang, x1 = CONV.xP + CONV.cap * CONV.pitch + PALLET.sx / 2 + 0.3;
  const hw = CONV.frameW / 2, z = CONV.z;
  for (const e of [1, -1]) {
    slab(g, x0, x1, e * hw - 0.03, e * hw + 0.03, z - 0.24, z - 0.06, M.conv); // side beams
    for (let x = x0 + 0.1; x < x1; x += 1.3) slab(g, x - 0.04, x + 0.04, e * hw - 0.04, e * hw + 0.04, 0, z - 0.24, M.conv); // legs
  }
  for (const y of CONV.chainY) {
    slab(g, x0, x1, y - CONV.chainW / 2, y + CONV.chainW / 2, z - 0.05, z, M.chain); // chain strands
    slab(g, x0, x1, y - 0.07, y + 0.07, z - 0.14, z - 0.05, M.conv);
  }
  for (let x = x0 + 0.1; x < x1; x += 1.3) slab(g, x - 0.03, x + 0.03, -hw, hw, z - 0.22, z - 0.14, M.conv); // cross ties
  for (const y of PALLET.blockY) slab(g, x0, xFace, y - 0.05, y + 0.05, z - 0.05, z + CONV.stopH, M.stop); // end stops at the blocks
  slab(g, x1 - 0.5, x1 - 0.1, hw + 0.03, hw + 0.33, z - 0.3, z - 0.02, M.dark); // drive
  const b1 = beam(g, xFace + CONV.b1x, -hw - 0.08, hw + 0.08, z + CONV.b1z);
  const b3 = beam(g, CONV.xP, -hw - 0.08, hw + 0.08, z + CONV.b3z);
  // signal column: green = PalletReady, blue = Locked
  slab(g, x0 + 0.02, x0 + 0.06, hw + 0.10, hw + 0.14, 0, 1.5, M.dark);
  const lampGeo = cylGeo(0.035, 0.09, 'z', 16);
  const lamps = [1.55, 1.65].map((h) => add(g, lampGeo, new THREE.MeshBasicMaterial({ color: 0x343a44 }), x0 + 0.04, hw + 0.12, h));
  // fork zone on the floor
  const zone = new THREE.Mesh(new THREE.PlaneGeometry(PALLET.sx + DOCK.zoneIn, PALLET.sy + 0.2), new THREE.MeshBasicMaterial({ color: 0x4aa3ff, transparent: true, opacity: 0.1 }));
  zone.position.set(CONV.xP - DOCK.zoneIn / 2, 0, 0.005);
  g.add(zone);
  label(W, cfg.id, CONV.xP - 0.3, cfg.y + hw + 0.5, 1.9, '#4aa3ff');
  return { b1, b3, lamps, zone };
});

// ---------- drop stations (spec §10.1) ----------
const slotPads = CELL.slots.map((s) => {
  outline(W, s.x, s.y, PALLET.sy + 0.2, PALLET.sx + 0.2, 0x3ecf8e);
  const pad = new THREE.Mesh(new THREE.PlaneGeometry(PALLET.sy + 0.2, PALLET.sx + 0.2), new THREE.MeshBasicMaterial({ color: 0x3ecf8e, transparent: true, opacity: 0.08 }));
  pad.position.set(s.x, s.y, 0.004);
  W.add(pad);
  label(W, s.id, s.x, s.y - 0.9, 0.35, '#3ecf8e', 0.8);
  return pad;
});
outline(W, CELL.stack.x, CELL.stack.y, PALLET.sy + 0.2, PALLET.sx + 0.2, 0xf0b429);
label(W, CELL.stack.id, CELL.stack.x, CELL.stack.y + 0.9, 0.35, '#f0b429', 0.8);
{
  const h = stationPoses({ ...CELL.home }).pre;
  outline(W, h.x, h.y + 0.585, AMR.body.w + 0.3, 2.3, 0x8a94a3); // heading −Y: the tail points to +Y
  label(W, 'HOME', h.x, h.y + 2.1, 0.35, '#8a94a3', 0.8);
}

// ---------- AMR (spec §3.2): one group per joint frame ----------
function wheel(parent, r, w, x, y, z) {
  const g = group(parent, x, y, z);
  add(g, cylGeo(r, w, 'y'), M.tyre);
  add(g, cylGeo(r * 0.55, w + 0.012, 'y', 20), M.steel);
  slab(g, -r * 0.5, r * 0.5, -w / 2 - 0.009, w / 2 + 0.009, -0.015, 0.015, M.dark); // spoke mark, makes the rolling visible
  return g;
}
function buildAmr() {
  const B = AMR.body, hw = B.w / 2, body = group(W);
  // chassis: counterweight at the rear with the drive unit under it, load-wheel arms in front
  slab(body, -1.05, -0.25, -hw, hw, AMR.groundClear + 0.02, AMR.chassisH, M.body);
  slab(body, B.xr, -1.05, -hw, hw, 0.36, AMR.chassisH, M.body);
  slab(body, B.xr, -0.85, -hw + 0.03, hw - 0.03, AMR.chassisH, AMR.hoodH, M.dark); // counterweight
  slab(body, -0.85, -0.25, -hw + 0.06, hw - 0.06, AMR.chassisH, 0.88, M.dark); // battery cover
  slab(body, -0.25, AMR.chassisXf, -0.36, 0.36, AMR.groundClear + 0.02, 0.50, M.body);
  for (const e of [1, -1]) {
    slab(body, -0.25, AMR.chassisXf, e * 0.5 - 0.06, e * 0.5 + 0.06, 0.32, 0.38, M.body); // fenders
    slab(body, -0.02, 0.02, e * 0.36, e * 0.44, 0.11, 0.19, M.steel); // stub axle
    slab(body, B.xr + 0.02, B.xr + 0.10, e * (hw - 0.14), e * (hw - 0.04), 0.40, 0.50, M.dark); // rear scanners
  }
  slab(body, -1.24, -1.16, -0.04, 0.04, AMR.hoodH, 2.30, M.dark); // lidar post
  add(body, cylGeo(0.07, 0.09, 'z'), M.steel, -1.20, 0, 2.34);
  const beacon = add(body, cylGeo(0.035, 0.06, 'z', 16), new THREE.MeshBasicMaterial({ color: 0xf0b429 }), -1.45, 0.4, AMR.hoodH + 0.03);
  const wl = wheel(body, AMR.rLoad, AMR.wLoad, 0, AMR.track / 2, AMR.rLoad);
  const wr = wheel(body, AMR.rLoad, AMR.wLoad, 0, -AMR.track / 2, AMR.rLoad);
  const steer = group(body, -AMR.L, 0, AMR.rDrive); // q2 about Z
  add(steer, cylGeo(0.16, 0.05, 'z'), M.steel, 0, 0, 0.19); // slew ring
  slab(steer, -0.05, 0.05, -0.10, -0.07, -0.02, 0.19, M.steel);
  slab(steer, -0.05, 0.05, 0.07, 0.10, -0.02, 0.19, M.steel);
  const wd = wheel(steer, AMR.rDrive, AMR.wDrive, 0, 0, 0); // q1 about Y
  const steerArrow = polyline(steer, [[-0.3, 0, -0.14], [0.45, 0, -0.14], [0.36, 0.05, -0.14], [0.45, 0, -0.14], [0.36, -0.05, -0.14]], 0xf2b01b);

  // mast: tilts about -Y through the pivot (q3)
  const mast = group(body, XT, 0, ZT);
  for (const e of [1, -1]) slab(mast, MAST.x0 - XT, MAST.x1 - XT, e * MAST.w / 2 - 0.05, e * MAST.w / 2 + 0.05, 0.10 - ZT, MAST.height - ZT, M.dark);
  slab(mast, MAST.x0 - XT, MAST.x1 - XT, -MAST.w / 2, MAST.w / 2, MAST.height - ZT - 0.08, MAST.height - ZT, M.dark);
  slab(mast, MAST.x0 - XT, MAST.x0 - XT + 0.04, -MAST.w / 2, MAST.w / 2, 0.75 - ZT, 0.83 - ZT, M.dark);
  add(mast, cylGeo(0.035, MAST.w + 0.16, 'y', 16), M.steel); // tilt axis
  add(mast, cylGeo(0.04, 1.5, 'z', 16), M.steel, MAST.x0 - XT + 0.07, 0, 0.9 - ZT); // lift cylinder
  // carriage: rides on the mast (q4); origin at the height of the fork top face
  const carriage = group(mast);
  slab(carriage, MAST.x1 - XT, MAST.x1 - XT + 0.025, -FORK.carriageW / 2, FORK.carriageW / 2, -0.04, 0.42, M.dark);
  for (const e of [1, -1]) slab(carriage, MAST.x0 - XT + 0.02, MAST.x1 - XT, e * (MAST.w / 2 - 0.09) - 0.03, e * (MAST.w / 2 - 0.09) + 0.03, 0, 0.36, M.steel); // rollers
  // side shift (q5): fork shanks, tines and backrest; origin on the fork top face
  const shift = group(carriage);
  const xf = MAST.aFace;
  slab(shift, xf - 0.045, xf - 0.02, -FORK.carriageW / 2, FORK.carriageW / 2, 0.30, 0.40, M.steel);
  slab(shift, xf - 0.045, xf - 0.02, -FORK.carriageW / 2, FORK.carriageW / 2, 0.02, 0.10, M.steel);
  for (const e of [1, -1]) {
    const y = e * FORK.tineY, h = FORK.tineW / 2;
    slab(shift, xf - FORK.thick, xf, y - h, y + h, -FORK.thick, 0.42, M.fork); // shank
    slab(shift, xf, xf + FORK.length - 0.12, y - h, y + h, -FORK.thick, 0, M.fork); // tine
    const tip = new THREE.BoxGeometry(0.12, FORK.tineW, FORK.thick); // tapered tip
    const pos = tip.attributes.position;
    for (let i = 0; i < pos.count; i++) if (pos.getX(i) > 0 && pos.getZ(i) < 0) pos.setZ(i, FORK.thick / 2 - 0.012);
    tip.computeVertexNormals();
    add(shift, tip, M.fork, xf + FORK.length - 0.06, y, -FORK.thick / 2);
    slab(shift, xf - 0.03, xf - 0.01, e * (FORK.carriageW / 2 - 0.02) - 0.02, e * (FORK.carriageW / 2 - 0.02) + 0.02, 0.40, FORK.backrestH, M.dark); // backrest
  }
  for (const zb of [0.6, 0.8, FORK.backrestH - 0.02]) slab(shift, xf - 0.03, xf - 0.01, -FORK.carriageW / 2, FORK.carriageW / 2, zb - 0.02, zb + 0.02, M.dark);
  for (const y of [-0.15, 0.15]) slab(shift, xf - 0.03, xf - 0.01, y - 0.012, y + 0.012, 0.40, FORK.backrestH, M.dark);
  const tcp = add(shift, new THREE.SphereGeometry(0.022, 16, 12), new THREE.MeshBasicMaterial({ color: 0x3ecf8e }), K.A_TCP, 0, 0);
  tcp.castShadow = false;
  return { body, wl, wr, steer, wd, steerArrow, mast, carriage, shift, beacon };
}
const R = buildAmr();

// ICR construction lines: load axle -> ICR <- drive wheel
const icrMat = new THREE.LineBasicMaterial({ color: 0xff5fa2, transparent: true, opacity: 0.9 });
const icrLine = new THREE.Line(new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(new Float32Array(9), 3)), icrMat);
icrLine.frustumCulled = false;
const icrDot = new THREE.Mesh(new THREE.SphereGeometry(0.06, 16, 12), new THREE.MeshBasicMaterial({ color: 0xff5fa2 }));
W.add(icrLine, icrDot);

// routes on the floor, the active one highlighted
const pathGroup = group(W);
let activeLine = null, activePath = null;
const sample = (p, z) => Array.from({ length: Math.ceil(p.length / 0.05) + 1 }, (_, i) => { const a = p.at(Math.min(i * 0.05, p.length)); return [a.x, a.y, z]; });
function drawRoutes(cell) {
  pathGroup.clear();
  for (const l of cell.lines) {
    for (const s of cell.sides) { const r = cell.route(l, s); polyline(pathGroup, sample(r.leg, 0.008), 0x4a5462); polyline(pathGroup, sample(r.link, 0.008), 0x4a5462); }
    polyline(pathGroup, sample(cell.dock(l), 0.008), 0x4aa3ff);
  }
  for (const s of [...cell.slots, cell.stack]) polyline(pathGroup, sample(cell.dock(s), 0.008), 0x4aa3ff);
  activeLine = activePath = null;
}

// TCP trail
const TRAIL_N = 2400;
const trailPos = new Float32Array(TRAIL_N * 3);
const trail = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x3ecf8e, transparent: true, opacity: 0.75 }));
trail.geometry.setAttribute('position', new THREE.BufferAttribute(trailPos, 3));
trail.frustumCulled = false;
W.add(trail);
let trailLen = 0;

// ---------- unit loads: every pallet and every box is one instance ----------
const palletGeo = (() => {
  const parts = [], P = PALLET;
  const b = (sx, sy, sz, x, y, z) => parts.push(new THREE.BoxGeometry(sx, sy, sz).translate(x, y, z));
  for (const [x, w] of [[-0.3275, 0.145], [-0.1638, 0.1], [0, 0.145], [0.1638, 0.1], [0.3275, 0.145]]) b(w, P.sy, 0.022, x, 0, P.h - 0.011); // deck boards
  for (const y of P.blockY) b(P.sx, P.blockLy, 0.022, 0, y, 0.111); // stringer boards
  for (const x of P.blockX) for (const y of P.blockY) b(x ? P.blockLx : 0.145, P.blockLy, 0.078, x, y, 0.061); // blocks
  for (const x of P.blockX) b(x ? P.blockLx : 0.145, P.sy, 0.022, x, 0, 0.011); // bottom boards
  return mergeGeometries(parts);
})();
const CAP_P = 64, CAP_B = 2400;
const palletMesh = new THREE.InstancedMesh(palletGeo, M.wood, CAP_P);
const boxMesh = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), M.box, CAP_B);
for (const im of [palletMesh, boxMesh]) {
  im.castShadow = im.receiveShadow = true;
  im.frustumCulled = false;
  im.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
  W.add(im);
}
const skuColor = SKUS.map((s) => new THREE.Color(s.color));
const tint = new THREE.Color(), mP = new THREE.Matrix4(), mB = new THREE.Matrix4(), eul = new THREE.Euler(), quat = new THREE.Quaternion(), v3 = new THREE.Vector3(), one = new THREE.Vector3(1, 1, 1);
function drawLoads() {
  let np = 0, nb = 0;
  for (const p of cell.pallets) {
    if (np >= CAP_P) break;
    eul.set(0, -p.pitch, p.yaw, 'ZYX'); // Rz(yaw)·Ry(−pitch): pitch is positive tips-up
    mP.compose(v3.set(p.x, p.y, p.z), quat.setFromEuler(eul), one);
    palletMesh.setMatrixAt(np++, mP);
    for (const b of p.boxes) {
      if (nb >= CAP_B) break;
      mB.makeScale(b.sx - 0.006, b.sy - 0.006, b.sz - 0.004).setPosition(b.lx, b.ly, b.lz).premultiply(mP);
      boxMesh.setMatrixAt(nb, mB);
      boxMesh.setColorAt(nb++, tint.copy(skuColor[p.skuIndex]).multiplyScalar(0.9 + 0.1 * (((p.id * 31 + nb * 17) % 11) / 10)));
    }
  }
  palletMesh.count = np;
  boxMesh.count = nb;
  palletMesh.instanceMatrix.needsUpdate = boxMesh.instanceMatrix.needsUpdate = true;
  if (boxMesh.instanceColor) boxMesh.instanceColor.needsUpdate = true;
}

let cell;
function reset() {
  cell = new Cell({ seed: 7 });
  drawRoutes(cell);
  trailLen = 0;
  acc = 0;
}

// ---------- HUD ----------
const $ = (id) => document.getElementById(id);
const JOINTS = [
  { id: 'q1', name: 'Drive', unit: 'm/s', val: (a) => a.vs, pos: () => null, spd: (a) => Math.abs(a.vs) / AMR.wheelV },
  { id: 'q2', name: 'Steer', unit: '°', val: (a) => a.phi / DEG, pos: (a) => (a.phi - AMR.steer.min) / (AMR.steer.max - AMR.steer.min), spd: (a) => Math.abs(a.steerRate) / AMR.steer.v },
  { id: 'q3', name: 'Tilt', unit: '°', val: (a) => a.tilt / DEG, pos: (a) => (a.tilt - MAST.tilt.min) / (MAST.tilt.max - MAST.tilt.min), spd: (a) => Math.abs(a.mastRate[0]) / MAST.tilt.v },
  { id: 'q4', name: 'Lift', unit: 'm', val: (a) => a.lift, pos: (a) => (a.lift - MAST.lift.min) / (MAST.lift.max - MAST.lift.min), spd: (a) => Math.abs(a.mastRate[1]) / MAST.lift.v },
  { id: 'q5', name: 'Shift', unit: 'm', val: (a) => a.shift, pos: (a) => (a.shift - MAST.shift.min) / (MAST.shift.max - MAST.shift.min), spd: (a) => Math.abs(a.mastRate[2]) / MAST.shift.v },
];
const axisRows = JOINTS.map((j) => {
  const row = document.createElement('div');
  row.className = 'j';
  row.innerHTML = `<span>${j.id}</span><span class="nm">${j.name}</span><span class="val"></span><div class="bar pos"><i></i></div><div class="bar"><i></i></div>`;
  $('axes').appendChild(row);
  return { val: row.querySelector('.val'), pos: row.querySelectorAll('i')[0], spd: row.querySelectorAll('i')[1] };
});
const lineRows = CONV.lines.map((c) => {
  const row = document.createElement('div');
  row.className = 'st';
  row.innerHTML = `<b>${c.id}</b><span class="lamp">B1</span><span class="lamp">B3</span><span class="lamp">Ready</span><span class="lamp">Locked</span>`;
  $('lines').appendChild(row);
  const q = document.createElement('div');
  q.className = 'sub';
  $('lines').appendChild(q);
  return { lamps: [...row.querySelectorAll('.lamp')], q };
});
const lamp = (el, on, cls = 'on') => { el.className = 'lamp ' + (on ? cls : ''); };
const sg = (v, n) => (v >= 0 ? ' ' : '') + v.toFixed(n);

function hud() {
  const a = cell.amr, T = MAST.travel;
  $('phase').textContent = a.phase;
  $('job').textContent = a.job ? `${a.job.line.id} → ${a.job.dest.id} (${a.job.kind})` : 'none';
  const p = a.carried ?? (a.engaged?.pallet);
  $('load').textContent = p ? `#${p.id} ${p.kind}${p.sku ? ' ' + p.sku + ' · ' + p.boxes.length + ' boxes' : ''} · ${p.mass} kg${a.carried ? '' : ' (resting)'}` : 'none';
  JOINTS.forEach((j, i) => {
    const r = axisRows[i], pos = j.pos(a);
    r.val.textContent = `${sg(j.val(a), j.unit === '°' ? 2 : 3)} ${j.unit}`;
    r.pos.style.display = pos === null ? 'none' : '';
    if (pos !== null) r.pos.style.left = `calc(${100 * pos}% - 1.5px)`;
    r.spd.style.width = `${Math.min(100, 100 * j.spd(a))}%`;
  });
  lamp($('lLoad'), a.loaded);
  lamp($('lZone'), !!a.last.zone, 'warn');
  lamp($('lTravel'), Math.abs(a.lift - T.lift) < 1e-3 && Math.abs(a.tilt - T.tilt) < 1e-3 && Math.abs(a.shift) < 1e-3);
  lamp($('lStand'), Math.abs(a.v) < 1e-4);
  const bad = a.last.bad.length > 0;
  $('lMon').className = 'lamp ' + (bad ? 'bad' : 'on');
  $('mon').textContent = bad ? a.last.bad[0] : `OK  (${a.stats.violations} violations)`;

  const tcp = K.tcpWorld(a.pose, a.tilt, a.lift, a.shift);
  $('pose').textContent = `${a.pose.x.toFixed(3)}  ${a.pose.y.toFixed(3)}  ${(a.pose.th / DEG).toFixed(1)}°`;
  $('tcp').textContent = `${tcp[0].toFixed(3)}  ${tcp[1].toFixed(3)}  ${tcp[2].toFixed(3)}`;
  $('spd').textContent = `${sg(a.v, 3)} / ${sg(a.vs, 3)} m/s`;
  $('icr').textContent = Math.abs(a.k) > 1e-4 ? `${(1 / a.k).toFixed(2)} m ${a.k > 0 ? 'left' : 'right'}` : '∞ (straight)';
  $('detS').textContent = `${K.steerMapDet(a.vs).toFixed(3)}${Math.abs(a.vs) < 1e-4 ? '  singular: standstill' : ''}`;
  const dm = K.det2(K.mastJacobian(a.tilt, a.lift));
  $('detM').textContent = `${dm.toFixed(3)}${Math.abs(dm) < 0.06 ? '  near singular' : ''}`;
  $('alat').textContent = `${(a.v * a.v * Math.abs(a.k)).toFixed(2)} / ${AMR.aLat.toFixed(1)} m/s²`;
  const stb = K.stability(a.load());
  $('tip').textContent = `${stb.brakeTip.toFixed(2)} / ${AMR.eStopDecel} m/s²  SF ${(stb.brakeTip / AMR.eStopDecel).toFixed(1)}`;
  $('clr').textContent = `${(1000 * a.last.clearance).toFixed(0)} mm  (min ${(1000 * a.stats.minClear).toFixed(0)})`;
  $('odo').textContent = `${(1000 * Math.hypot(a.odo.x - a.pose.x, a.odo.y - a.pose.y)).toFixed(3)} mm`;

  cell.lines.forEach((l, i) => {
    const r = lineRows[i];
    lamp(r.lamps[0], l.b1, 'warn'); lamp(r.lamps[1], l.b3, 'warn'); lamp(r.lamps[2], l.ready); lamp(r.lamps[3], l.locked, 'info');
    const pp = l.presented;
    r.q.textContent = `   ${pp ? `#${pp.id} ${pp.kind} ${pp.mass} kg, offset ${sg(1000 * (pp.y - l.y), 0)} mm` : 'no pallet at the end stop'} · ${l.pallets.length} on line`;
  });
  $('slots').textContent = cell.slots.map((s) => (s.pallet ? `#${s.pallet.id}` : s.reserved ? 'resv' : 'free')).join('  ');
  $('stack').textContent = `${cell.stack.pallets.length} / ${cell.stack.max}`;
  const st = a.stats;
  $('count').textContent = `${st.filled} / ${st.empty}`;
  $('thr').textContent = `${cell.t > 120 ? ((st.delivered * 3600) / cell.t).toFixed(0) + ' pallets/h' : '–'} / ${st.cycle.length ? st.cycle.at(-1).toFixed(1) + ' s' : '–'}`;
  const s = Math.floor(cell.t);
  $('time').textContent = `${String(Math.floor(s / 3600)).padStart(2, '0')}:${String(Math.floor(s / 60) % 60).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
}

// ---------- scene update from sim state ----------
let showTrail = true, showIcr = true, view = 'overview';
function sync() {
  const a = cell.amr;
  R.body.position.set(a.pose.x, a.pose.y, 0);
  R.body.rotation.z = a.pose.th;
  R.steer.rotation.z = a.phi; // q2
  R.wd.rotation.y = a.wheel.drive; // q1
  R.wl.rotation.y = a.wheel.left;
  R.wr.rotation.y = a.wheel.right;
  R.mast.rotation.y = -a.tilt; // q3: positive = back = about −Y
  R.carriage.position.z = a.lift - ZT; // q4
  R.shift.position.y = a.shift; // q5
  R.beacon.material.color.setHex(Math.abs(a.v) > 1e-4 && Math.floor(performance.now() / 250) % 2 ? 0xf0b429 : 0x5a4a1a);

  cell.lines.forEach((l, i) => {
    const v = lineViews[i];
    v.b1.color.setHex(l.b1 ? 0xf0b429 : 0x3a4250);
    v.b3.color.setHex(l.b3 ? 0xf0b429 : 0x3a4250);
    v.lamps[0].material.color.setHex(l.ready ? 0x3ecf8e : 0x343a44);
    v.lamps[1].material.color.setHex(l.locked ? 0x4aa3ff : 0x343a44);
    v.zone.material.opacity = l.locked ? 0.3 : 0.1;
  });
  cell.slots.forEach((s, i) => { slotPads[i].material.opacity = s.pallet ? 0.3 : s.reserved ? 0.18 : 0.08; });
  drawLoads();

  // ICR: on the load-axle line at lateral distance 1/κ; the drive wheel axle points at it too
  const on = showIcr && Math.abs(a.k) > 1e-3;
  icrLine.visible = icrDot.visible = on;
  if (on) {
    const c = Math.cos(a.pose.th), s = Math.sin(a.pose.th), r = 1 / a.k, z = 0.02;
    const icr = [a.pose.x - s * r, a.pose.y + c * r, z], dw = [a.pose.x - c * AMR.L, a.pose.y - s * AMR.L, z];
    icrLine.geometry.attributes.position.array.set([a.pose.x, a.pose.y, z, ...icr, ...dw]);
    icrLine.geometry.attributes.position.needsUpdate = true;
    icrDot.position.set(...icr);
  }
  if (a.path !== activePath) {
    if (activeLine) { pathGroup.remove(activeLine); activeLine.geometry.dispose(); }
    activePath = a.path;
    activeLine = activePath ? polyline(pathGroup, sample(activePath, 0.012), 0xf2b01b) : null;
  }

  const tip = K.tcpWorld(a.pose, a.tilt, a.lift, a.shift);
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

// ---------- cameras (three coordinates: x, up, −worldY) ----------
const VIEWS = {
  overview: [[-7.5, 10.5, 13.5], [3.2, 0.4, 0.3]],
  top: [[3.2, 27, 0.01], [3.2, 0, 0]],
  station: [[3.0, 2.7, -6.4], [5.3, 0.7, -1.6]],
};
function setView(name) {
  view = name;
  document.querySelectorAll('[data-view]').forEach((b) => b.classList.toggle('on', b.dataset.view === name));
  if (!VIEWS[name]) return;
  camera.position.set(...VIEWS[name][0]);
  controls.target.set(...VIEWS[name][1]);
}
const tgt = new THREE.Vector3(), off = new THREE.Vector3();
function chaseCam() {
  const a = cell.amr, c = Math.cos(a.pose.th), s = Math.sin(a.pose.th);
  if (view === 'follow') {
    tgt.set(a.pose.x, 0.8, -a.pose.y);
    off.set(-5.5, 4.2, 6.5);
    controls.target.lerp(tgt, 0.12);
    camera.position.lerp(tgt.clone().add(off), 0.06);
  } else if (view === 'fork') {
    // rides with the truck: looks at the fork from the left front quarter
    const p = (bx, by, bz) => new THREE.Vector3(a.pose.x + c * bx - s * by, bz, -(a.pose.y + s * bx + c * by));
    controls.target.lerp(p(0.95, 0, a.lift + 0.1), 0.2);
    camera.position.lerp(p(0.2, 2.6, a.lift + 0.9), 0.1);
  }
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
$('icrBtn').onclick = (e) => { showIcr = !showIcr; e.target.classList.toggle('on', showIcr); };
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
// Integration handle: window.amr.cell is the Cell (see AMR_SPEC.md).
window.amr = { get cell() { return cell; }, setView, camera, controls, step: (n = 1) => { for (let i = 0; i < n; i++) cell.step(DT); } };

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
