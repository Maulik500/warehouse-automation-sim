// The 3D scene of the integrated warehouse, independent of where its state comes from: the live
// simulation (state.js → snapshot(world)) or a recording (state.js → Player). All geometry lives
// in `W`, a Z-up group, so coordinates are identical to WAREHOUSE_SPEC.md. Every machine is the
// kinematic chain of its own specification.
//
//   const v = createView({ layout: world, theme: 'light' | 'dark' });
//   v.apply(state);  v.render();  v.anchor('robot:M1', state) → [x, y, z] in {G}

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CRANE, FORK as AFORK, RACK, LAYOUT } from '../../asrs/js/config.js';
import { AMR as FAMR, MAST, FORK as FFORK } from '../../amr/js/config.js';
import { AMR as MAMR, ARM, TOOL as MTOOL, DECK } from '../../mobman/js/config.js';
import { DH } from '../../mobman/js/arm.js';
import { ROBOT, CELL } from '../../js/config.js';
import { LINE, TABLE, CARRIAGE, CLAMP, CUT } from '../../wrapper/js/config.js';
import { BUILDING, PLACE, PCELL, PTOOL, FLOOR, SKUS, ROWS, PALLET, WRAPX } from './config.js';
import { RING, mmRoutes, MM_LINK } from './roadmap.js';

const PI = Math.PI;
const THEMES = {
  dark: { bg: 0x14171c, fogNear: 90, fogFar: 260, floor: 0x1a1d23, hall: 0x23272e, edge: 0x5a6472, hemi: [0xdfe8ff, 0x2a2622, 1.35], sun: 2.0, label: '#d9dee6', road: 0xf2b01b, mmRoad: 0x4aa3ff },
  light: { bg: 0xdfe3e1, fogNear: 110, fogFar: 300, floor: 0xb9bcb6, hall: 0xd6d8d2, edge: 0x8a948e, hemi: [0xffffff, 0x8c9088, 1.9], sun: 2.5, label: '#1d2426', road: 0xe8b81e, mmRoad: 0x2f6fb5 },
};
// products named in the specification (catalogue data where public, otherwise class-typical)
export const PRODUCTS = {
  palletizer: 'KUKA KR 120 R2700-2 class', cobot: 'Universal Robots UR20 class', asrs: 'Single-mast class stacker crane, 1000 kg',
  forklift: 'Counterbalanced pallet AMR, 1000 kg', mobile: 'Differential AMR + UR20 class arm', wrapper: 'Turntable stretch wrapper, 12 rpm',
};

export function createView({ layout, theme = 'dark', container = document.body, capture = null } = {}) {
  const TH = THEMES[theme];
  const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: !!capture });
  renderer.setPixelRatio(capture ? 1 : Math.min(devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  container.prepend(renderer.domElement);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(TH.bg);
  scene.fog = new THREE.Fog(TH.bg, TH.fogNear, TH.fogFar);
  const camera = new THREE.PerspectiveCamera(40, 16 / 9, 0.1, 600);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.maxPolarAngle = PI / 2 - 0.02;
  scene.add(new THREE.HemisphereLight(...TH.hemi));
  const sun = new THREE.DirectionalLight(0xfff6e8, TH.sun);
  sun.position.set(10, 60, 10);
  sun.target.position.set(34, 0, -24);
  sun.castShadow = true;
  sun.shadow.mapSize.set(4096, 4096);
  Object.assign(sun.shadow.camera, { left: -46, right: 46, top: 40, bottom: -40, near: 1, far: 160 });
  sun.shadow.bias = -0.0006;
  scene.add(sun, sun.target);
  if (theme === 'light') { const fill = new THREE.DirectionalLight(0xdfe8ff, 0.7); fill.position.set(60, 25, 60); scene.add(fill); }

  const W = new THREE.Group(); // global frame {G}: Z up
  W.rotation.x = -PI / 2;
  scene.add(W);

  // ---------- helpers (Z-up coordinates) ----------
  const mat = (color, o = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.2, ...o });
  const M = {
    yellow: mat(0xf2b01b, { roughness: 0.45 }), blue: mat(0x3d6fb5, { roughness: 0.5 }), orange: mat(0xe0742a, { roughness: 0.5, metalness: 0.3 }),
    dark: mat(0x23272e, { roughness: 0.6, metalness: 0.4 }), steel: mat(0x9aa3b0, { roughness: 0.35, metalness: 0.7 }), roller: mat(0xb9c0cb, { roughness: 0.3, metalness: 0.8 }),
    tyre: mat(0x16181c, { roughness: 0.9, metalness: 0 }), conv: mat(0x4a5361, { metalness: 0.5 }), rack: mat(0x2f5f9e, { roughness: 0.6, metalness: 0.4 }), beam: mat(0xd8792a, { roughness: 0.6, metalness: 0.3 }),
    wood: mat(0xb98a55, { roughness: 0.85, metalness: 0 }), box: mat(0xffffff, { roughness: 0.88, metalness: 0 }), arm: mat(0xdfe3e8, { roughness: 0.4, metalness: 0.2 }),
    joint: mat(0x3f7fd0, { roughness: 0.4, metalness: 0.3 }), cup: mat(0x2b2f36, { roughness: 0.9, metalness: 0 }), kuka: mat(0xf28c00, { roughness: 0.45, metalness: 0.25 }), red: mat(0xc0392b, { roughness: 0.5 }),
    otto: mat(0x2b2f33, { roughness: 0.6, metalness: 0.3 }), wall: mat(0xe9ebe6, { roughness: 0.9, metalness: 0 }), column: mat(0xc9ccc5, { roughness: 0.8, metalness: 0.1 }),
    film: new THREE.MeshStandardMaterial({ color: 0xcfe8ff, transparent: true, opacity: 0.32, roughness: 0.15, metalness: 0, depthWrite: false }),
  };
  const add = (parent, geo, material, x = 0, y = 0, z = 0) => { const m = new THREE.Mesh(geo, material); m.position.set(x, y, z); m.castShadow = m.receiveShadow = true; parent.add(m); return m; };
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
  function label(text, x, y, z, color = TH.label, size = 1) {
    const c = document.createElement('canvas');
    c.width = 512; c.height = 96;
    const g = c.getContext('2d');
    g.font = '700 54px Inter, Arial, sans-serif';
    g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillStyle = color;
    g.fillText(text, 256, 50);
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthWrite: false }));
    s.position.set(x, y, z); s.scale.set(4.0 * size, 0.75 * size, 1);
    labels.add(s);
    return s;
  }
  function floorText(text, x, y, size, color = '#2e4b52', rot = 0, opacity = 0.85) {
    const c = document.createElement('canvas');
    c.width = 1024; c.height = 256;
    const g = c.getContext('2d');
    g.font = 'bold 150px Arial'; g.textAlign = 'center'; g.textBaseline = 'middle'; g.fillStyle = color;
    g.fillText(text, 512, 132);
    const m = new THREE.Mesh(new THREE.PlaneGeometry(size * 4, size), new THREE.MeshBasicMaterial({ map: new THREE.CanvasTexture(c), transparent: true, opacity, depthWrite: false }));
    m.position.set(x, y, 0.015); m.rotation.z = rot;
    W.add(m);
    return m;
  }
  function polyline(parent, pts, color, opacity = 1) {
    const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts.map((p) => new THREE.Vector3(...p))), new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity }));
    parent.add(l);
    return l;
  }
  function pad(parent, x0, x1, y0, y1, color, opacity = 0.12) {
    const m = new THREE.Mesh(new THREE.PlaneGeometry(Math.abs(x1 - x0), Math.abs(y1 - y0)), new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthWrite: false }));
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
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(400, 400), mat(TH.floor, { roughness: 0.95, metalness: 0 }));
    floor.position.set(BUILDING.x / 2, BUILDING.y / 2, -0.002);
    floor.receiveShadow = true;
    W.add(floor);
    const hall = new THREE.Mesh(new THREE.PlaneGeometry(BUILDING.x, BUILDING.y), mat(TH.hall, { roughness: 0.9, metalness: 0 }));
    hall.position.set(BUILDING.x / 2, BUILDING.y / 2, 0);
    hall.receiveShadow = true;
    W.add(hall);
    polyline(W, [[0, 0, 0.01], [BUILDING.x, 0, 0.01], [BUILDING.x, BUILDING.y, 0.01], [0, BUILDING.y, 0.01], [0, 0, 0.01]], TH.edge);
    // low perimeter walls (the camera sees in) and building columns every 16 m
    const X = BUILDING.x, Y = BUILDING.y;
    slab(W, 0, X, -0.25, 0, 0, 1.2, M.wall); slab(W, 0, X, Y, Y + 0.25, 0, 1.2, M.wall);
    slab(W, -0.25, 0, 0, Y, 0, 1.2, M.wall); slab(W, X, X + 0.25, 0, Y, 0, 1.2, M.wall);
    for (let x = 0; x <= X; x += 17) for (const y of [0, Y]) slab(W, x - 0.2, x + 0.2, y - 0.2, y + 0.2, 0, 9, M.column);
  }
  const roads = group(W);
  function drawPath(path, color, opacity = 0.75) {
    const n = Math.ceil(path.length / 0.25);
    polyline(roads, Array.from({ length: n + 1 }, (_, i) => { const a = path.at((path.length * i) / n); return [a.x, a.y, 0.012]; }), color, opacity);
  }

  // ---------- unit loads ----------
  const palletGeo = (() => {
    const g = new THREE.BufferGeometry(), P = PALLET, pos = [], idx = [];
    const b = (sx, sy, sz, x, y, z) => {
      const box = new THREE.BoxGeometry(sx, sy, sz).translate(x, y, z), o = pos.length / 3;
      pos.push(...box.attributes.position.array);
      for (const i of box.index.array) idx.push(o + i);
    };
    for (const [y, w] of [[-0.3275, 0.145], [-0.1638, 0.1], [0, 0.145], [0.1638, 0.1], [0.3275, 0.145]]) b(P.sx, w, 0.022, 0, y, P.h - 0.011);
    for (const x of [-0.5275, 0, 0.5275]) b(0.145, P.sy, 0.022, x, 0, 0.111);
    for (const x of [-0.5275, 0, 0.5275]) for (const y of [-0.35, 0, 0.35]) b(0.145, y ? 0.1 : 0.145, 0.078, x, y, 0.061);
    for (const y of [-0.35, 0, 0.35]) b(P.sx, y ? 0.1 : 0.145, 0.022, 0, y, 0.011);
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setIndex(idx);
    g.computeVertexNormals();
    return g;
  })();
  const unit = new THREE.BoxGeometry(1, 1, 1);
  const palletIM = instanced(W, palletGeo, M.wood, 200);
  const rackPalIM = instanced(W, new THREE.BoxGeometry(PALLET.sx, PALLET.sy, PALLET.h), M.wood, 640);
  const rackLoadIM = instanced(W, unit, M.box, 640);
  const boxIM = instanced(W, unit, M.box, 3000);
  const filmIM = instanced(W, unit, M.film, 60);
  filmIM.castShadow = false;
  const skuColor = Object.fromEntries(SKUS.map((s) => [s.id, new THREE.Color(s.color)]));
  const tint = new THREE.Color(), m4 = new THREE.Matrix4(), quat = new THREE.Quaternion(), v3 = new THREE.Vector3(), scl = new THREE.Vector3(), zAxis = new THREE.Vector3(0, 0, 1);
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
        label(`ROW ${row}`, A.tx + xb + 1.6, A.ty + a.y + side * RACK.yStore, 1.2, theme === 'light' ? '#2f5c8f' : '#7fb0ff', 0.9);
      }
      const ag = group(asrsG, 0, a.y, 0), base = group(ag), car = group(base), mid = group(car), top = group(car);
      slab(ag, -1.6, 20.6, -0.05, 0.05, 0, 0.06, M.steel);
      slab(base, -CRANE.chassisLen / 2, CRANE.chassisLen / 2, -CRANE.chassisW / 2, CRANE.chassisW / 2, 0.06, 0.06 + CRANE.chassisH, M.yellow);
      for (const e of [1, -1]) slab(base, e * CRANE.mastX - CRANE.mastSx / 2, e * CRANE.mastX + CRANE.mastSx / 2, -CRANE.mastSy / 2, CRANE.mastSy / 2, 0.3, CRANE.mastH, M.yellow);
      slab(base, -CRANE.mastX, CRANE.mastX, -0.15, 0.15, CRANE.mastH - 0.25, CRANE.mastH, M.yellow);
      slab(car, -CRANE.carriageSx / 2, CRANE.carriageSx / 2, -CRANE.carriageSy / 2, CRANE.carriageSy / 2, -CRANE.carriageDrop, -0.07, M.dark);
      for (const e of [1, -1]) {
        const x0 = e * AFORK.tineX - AFORK.tineW / 2, x1 = e * AFORK.tineX + AFORK.tineW / 2, h = AFORK.length / 2;
        slab(car, x0, x1, -h, h, -0.07, -0.045, M.steel); slab(mid, x0, x1, -h, h, -0.045, -0.02, M.roller); slab(top, x0, x1, -h, h, -0.02, 0, M.orange);
      }
      cranes.push({ base, car, mid, top, y: a.y });
    }
    const upI = new THREE.InstancedMesh(new THREE.BoxGeometry(RACK.uprightW, 0.1, RACK.height), M.rack, ups.length);
    ups.forEach((p, i) => upI.setMatrixAt(i, m4.makeTranslation(...p)));
    const bmI = new THREE.InstancedMesh(new THREE.BoxGeometry(xb - xa, 0.05, 0.1), M.beam, beams.length);
    beams.forEach((p, i) => bmI.setMatrixAt(i, m4.makeTranslation(...p)));
    for (const im of [upI, bmI]) { im.castShadow = im.receiveShadow = true; asrsG.add(im); }
    label('ASRS', A.tx + 11, A.ty - 3.6, 14.6, theme === 'light' ? '#2f5c8f' : '#7fb0ff', 2.2);
  }
  const laneView = {};
  for (const row of ROWS) {
    const l = layout.asrs.lanes[row], f = l.frame, g = group(W);
    for (const e of [1, -1]) slab(g, f.x0, f.x1 + 1.2, f.y + e * f.hw - 0.04, f.y + e * f.hw, f.top - 0.16, f.top, M.conv);
    for (let x = f.x0 + 0.2; x < f.x1 + 1.2; x += 1.2) for (const e of [1, -1]) slab(g, x - 0.03, x + 0.03, f.y + e * f.hw - 0.05, f.y + e * f.hw, 0, f.top - 0.16, M.conv);
    rollers(g, Math.floor((f.x1 + 1.0 - f.x0) / 0.16), 0.04, 2 * f.hw - 0.1, 'y', (i) => [f.x0 + 0.1 + i * 0.16, f.y, f.top - 0.04]);
    const stops = [-0.5275, 0, 0.5275].map((dy) => slab(g, f.x0 + 0.005, f.x0 + 0.03, f.y + dy - 0.07, f.y + dy + 0.07, f.top - 0.09, f.top, M.orange));
    const lamp = add(g, cylGeo(0.05, 0.12, 'z', 12), new THREE.MeshBasicMaterial({ color: 0x343a44 }), f.x0 - 0.1, f.y + f.hw + 0.15, 1.3);
    slab(g, f.x0 - 0.12, f.x0 - 0.08, f.y + f.hw + 0.13, f.y + f.hw + 0.17, 0, 1.24, M.dark);
    label(`PORT ${row}`, l.x - 0.2, l.y, 2.5, theme === 'light' ? '#2f5c8f' : '#7fb0ff', 0.55);
    laneView[row] = { stops, lamp, top: f.top };
  }

  // ---------- palletizing robot (ROBOT_SPEC §3.1): one group per joint ----------
  function buildRobot(parent) {
    const { d1, a1, a2, a3, d4, d6 } = ROBOT, J = [];
    rod(parent, 'z', 0, 0.22, 0.42, M.dark);
    J[0] = group(parent);
    rod(J[0], 'z', 0.22, d1 - 0.1, 0.33, M.kuka);
    slab(J[0], -0.15, a1 + 0.2, -0.3, 0.3, d1 - 0.28, d1 + 0.12, M.kuka);
    J[1] = group(J[0], a1, 0, d1);
    rod(J[1], 'y', -0.33, 0.33, 0.2, M.dark);
    slab(J[1], -0.13, 0.13, -0.17, 0.17, 0, a2, M.kuka);
    J[2] = group(J[1], 0, 0, a2);
    rod(J[2], 'y', -0.28, 0.28, 0.17, M.dark);
    slab(J[2], -0.3, 0.35, -0.2, 0.2, -0.14, a3 + 0.16, M.kuka);
    J[3] = group(J[2], 0, 0, a3);
    rod(J[3], 'x', 0.35, d4 - 0.12, 0.1, M.kuka);
    J[4] = group(J[3], d4, 0, 0);
    rod(J[4], 'y', -0.13, 0.13, 0.11, M.dark);
    J[5] = group(J[4]);
    rod(J[5], 'x', 0.05, d6, 0.08, M.steel);
    const F = group(J[5], d6, 0, 0);
    F.rotation.y = PI / 2;
    rod(F, 'z', 0, 0.25, 0.04, M.dark);
    slab(F, -PTOOL.padX / 2, PTOOL.padX / 2, -PTOOL.padY / 2, PTOOL.padY / 2, 0.25, 0.272, M.steel);
    const cups = [];
    for (let i = 0; i < 3; i++) for (const e of [-1, 1]) cups.push(rod(F, 'z', 0.272, 0.3, PTOOL.cupDia / 2 - 0.004, M.cup.clone(), (i - 1) * PTOOL.colX, e * PTOOL.rowY));
    return { J, F, cups };
  }
  const cellView = PLACE.pal.map((P, i) => {
    const g = group(W, P.tx, P.ty, P.tz), cv = CELL.conv, R = PCELL.recv, I = PCELL.incline, hw = PCELL.beltW / 2;
    g.rotation.z = P.psi;
    slab(g, -0.6, 0.6, -0.6, 0.6, -P.tz, 0, M.dark);
    const robot = buildRobot(g);
    slab(g, cv.x - hw, cv.x + hw, PCELL.merge.y - 0.45, cv.yEnd, cv.top - 0.09, cv.top - 0.005, M.dark);
    for (const e of [1, -1]) slab(g, cv.x + e * hw, cv.x + e * (hw + 0.04), PCELL.merge.y - 0.45, cv.yEnd, cv.top - 0.16, cv.top + 0.01, M.conv);
    for (let y = PCELL.merge.y; y < cv.yEnd; y += 1.2) for (const e of [1, -1]) slab(g, cv.x + e * hw, cv.x + e * (hw + 0.05), y - 0.03, y + 0.03, -P.tz, cv.top - 0.16, M.conv);
    for (const y of [cv.yA, cv.yB]) polyline(g, [[cv.x - hw - 0.1, y, cv.top + 0.012], [cv.x + hw + 0.1, y, cv.top + 0.012]], 0x3ecf8e);
    const inc = slab(g, I.x0, I.x1, R.y - R.w / 2, R.y + R.w / 2, -0.04, 0, M.dark);
    inc.position.set((I.x0 + I.x1) / 2, R.y, (I.z0 + I.z1) / 2 - 0.02);
    inc.rotation.y = Math.atan2(I.z0 - I.z1, I.x1 - I.x0);
    for (const e of [1, -1]) slab(g, R.x0, R.x1, R.y + e * R.w / 2, R.y + e * (R.w / 2 + 0.04), R.z - 0.12, R.z + 0.05, M.conv);
    for (let x = R.x0 + 0.1; x < R.x1; x += 0.9) for (const e of [1, -1]) slab(g, x - 0.03, x + 0.03, R.y + e * R.w / 2, R.y + e * (R.w / 2 + 0.05), -P.tz, R.z - 0.12, M.conv);
    for (const x of [I.x0 + 0.1, I.x1 - 0.1]) for (const e of [1, -1]) slab(g, x - 0.03, x + 0.03, R.y + e * (R.w / 2 - 0.02), R.y + e * (R.w / 2 + 0.03), -P.tz, (x < 3 ? I.z0 : I.z1) - 0.06, M.conv);
    rollers(g, Math.floor((R.x1 - R.x0) / 0.075), 0.025, R.w, 'y', (k) => [R.x0 + 0.04 + k * 0.075, R.y, R.z - 0.025]);
    const gate = slab(g, R.x0 - 0.02, R.x0, R.y - R.w / 2 + 0.05, R.y + R.w / 2 - 0.05, R.z, R.z + 0.05, M.orange);
    const x0 = PCELL.dock.x - PALLET.sy / 2 - 0.03, x1 = 0.6, yc = PCELL.build.y;
    for (const e of [1, -1]) slab(g, x0, x1, yc + e * 0.42 - 0.03, yc + e * 0.42 + 0.03, -P.tz, -0.002, M.conv);
    for (const e of [1, -1]) slab(g, x0, x1, yc + e * 0.7 - 0.03, yc + e * 0.7, -P.tz, -0.02, M.dark);
    const fz = 0.01 - P.tz, fence = [[PCELL.fenceX, 0.75], [PCELL.fenceX, 2.5], [2.6, 2.5], [2.6, -5.0], [1.0, -5.0], [1.0, -1.2], [-1.2, -1.2], [PCELL.fenceX, 0.75]];
    polyline(g, fence.map(([x, y]) => [x, y, fz]), 0xf0b429, 0.8);
    for (const [fx, fy] of fence.slice(0, -1)) slab(g, fx - 0.03, fx + 0.03, fy - 0.03, fy + 0.03, -P.tz, 1.9, M.yellow);
    const lamp = add(g, cylGeo(0.06, 0.14, 'z', 12), new THREE.MeshBasicMaterial({ color: 0x343a44 }), PCELL.fenceX, 0.75, 2.0);
    label(`PAL ${i + 1}`, P.tx + 0.6, P.ty - 0.8, 3.6, theme === 'light' ? '#c8102e' : '#f2b01b', 0.9);
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
      rollers(g, Math.floor((LINE.xFar - LINE.xEnd) / LINE.rollPitch), LINE.rollR, LINE.rollLen, 'y', (k) => [s * (LINE.xEnd + 0.08 + k * LINE.rollPitch), 0, LINE.z - LINE.rollR]);
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
    const film = new THREE.Line(new THREE.BufferGeometry().setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3)), new THREE.LineBasicMaterial({ color: 0x9fd0ff }));
    film.frustumCulled = false;
    W.add(film);
    label('WRAPPER', WP.tx, WP.ty - 2.6, 3.4, theme === 'light' ? '#2f5c8f' : '#4aa3ff', 1.1);
    label('I4', WP.tx + WRAPX.i4, WP.ty + 3.3, 0.6, '#4aa3ff', 0.5); label('O4', WP.tx + WRAPX.o4, WP.ty + 3.3, 0.6, '#3ecf8e', 0.5);
    return { table, jaws, jaw, car, slide, film };
  })();

  // ---------- floor stations ----------
  const flc = theme === 'light' ? '#2e4b52' : '#8a94a3';
  for (const s of FLOOR.storage) pad(W, s.x - 0.62, s.x + 0.62, s.y - 0.42, s.y + 0.42, 0x3ecf8e, 0.25);
  for (const s of FLOOR.empties) pad(W, s.x - 0.62, s.x + 0.62, s.y - 0.42, s.y + 0.42, 0xb98a55, 0.3);
  for (const s of FLOOR.receiving) pad(W, s.x - 0.62, s.x + 0.62, s.y - 0.42, s.y + 0.42, 0x4aa3ff, 0.25);
  for (const s of FLOOR.parking) pad(W, s.x - 0.6, s.x + 0.6, s.y - 2.4, s.y - 4.6, 0x8a94a3, 0.15);
  floorText('STORAGE', 20.2, FLOOR.storage[0].y + 1.5, 1.0, flc);
  floorText('EMPTY PALLETS', 7.5, FLOOR.empties[0].y + 1.5, 0.6, flc);
  floorText('RECEIVING', 10.5, FLOOR.receiving[0].y + 1.4, 0.7, flc);
  floorText('PARKING', 24.2, FLOOR.parking[0].y - 5.2, 0.8, flc);
  floorText('PALLETIZING', 18.5, 21.4, 1.1, flc);

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
    const tag = label(id, 0, 0, 0, theme === 'light' ? '#1d2426' : '#f2b01b', 0.45);
    return { body, wl, wr, steer, drive, mast, car, shift, beacon, tag };
  }
  // ---------- robot AMR (MOBMAN_SPEC §3): body > deck, pedestal > arm base {A} > DH chain ----------
  function buildMobman(id) {
    const Bd = MAMR.body, hw = Bd.w / 2, body = group(W);
    slab(body, Bd.xr, Bd.xf, -hw, hw, MAMR.groundClear, MAMR.chassisH, M.otto);
    const band = slab(body, Bd.xr + 0.02, Bd.xf - 0.02, -hw - 0.004, hw + 0.004, 0.1, 0.14, new THREE.MeshStandardMaterial({ color: 0x46c46a, emissive: 0x46c46a, emissiveIntensity: 0.5 }));
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
    const tag = label(id, 0, 0, 0, theme === 'light' ? '#1d2426' : '#7fb0ff', 0.45);
    return { body, band, wl, wr, stop, J, F, cups, tag };
  }
  const forkView = Array.from({ length: 6 }, (_, i) => buildForklift(`F${i + 1}`));
  const mmView = Array.from({ length: 2 }, (_, i) => buildMobman(`M${i + 1}`));

  const claimLines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xff5fa2, transparent: true, opacity: 0.7 }));
  claimLines.frustumCulled = false; claimLines.visible = false;
  W.add(claimLines);

  // roads
  drawPath(RING, TH.road);
  for (const st of [...layout.stations, ...layout.parking]) { drawPath(st.spur.path, TH.road, 0.35); if (st.kind !== 'home') drawPath(st.spur.dockPath, TH.road, 0.35); }
  for (let i = 0; i < 4; i++) { const r = mmRoutes(i); drawPath(r.out, TH.mmRoad, 0.6); drawPath(r.back, TH.mmRoad, 0.6); drawPath(r.dock, TH.mmRoad, 0.6); }
  drawPath(MM_LINK, TH.mmRoad, 0.6);

  // ---------- state -> scene ----------
  const hex = (c) => (typeof c === 'number' ? c : 0x343a44);
  let rackKey = null;
  function apply(s) {
    if (s.rackKey !== rackKey) {
      rackKey = s.rackKey;
      let n = 0;
      for (const r of s.rack) {
        const [x, y, z, h, sku] = r;
        setInst(rackPalIM, n, x, y, z + PALLET.h / 2, PI / 2);
        setInst(rackLoadIM, n, x, y, z + PALLET.h + h / 2, 0, 0.8, 1.2, h);
        rackLoadIM.setColorAt(n, skuColor[sku]);
        n++;
      }
      rackPalIM.count = rackLoadIM.count = n;
      rackPalIM.instanceMatrix.needsUpdate = rackLoadIM.instanceMatrix.needsUpdate = true;
      if (rackLoadIM.instanceColor) rackLoadIM.instanceColor.needsUpdate = true;
    }
    s.cranes.forEach((q, i) => { const c = cranes[i]; c.base.position.x = q[0]; c.car.position.z = q[1]; c.mid.position.y = q[2] / 2; c.top.position.y = q[2]; });
    ROWS.forEach((row, i) => {
      const L = s.lanes[i], v = laneView[row];
      v.stops.forEach((st) => (st.position.z = v.top - 0.045 - 0.09 + 0.13 * L.stop));
      v.lamp.material.color.setHex(hex(L.lamp));
    });
    const ax = ['z', 'y', 'y', 'x', 'y', 'x'];
    s.cells.forEach((c, i) => {
      const v = cellView[i];
      for (let k = 0; k < 6; k++) v.robot.J[k].rotation[ax[k]] = c.q[k];
      for (const cup of v.robot.cups) cup.material.color.setHex(c.held ? 0x3ecf8e : 0x2b2f36);
      v.gate.position.z = PCELL.recv.z + (c.gate ? -0.03 : 0.025);
      v.lamp.material.color.setHex(hex(c.lamp));
    });
    {
      const w = s.wrap, v = wrapView;
      v.table.rotation.z = w.th;
      v.jaws.position.z = CLAMP.stroke * w.clampUp;
      v.jaw.forEach((j, k) => (j.position.y = (k ? -1 : 1) * (0.0275 + 0.02 * (1 - w.jaw))));
      v.car.position.z = w.lift;
      v.slide.position.y = w.slide;
      v.film.visible = !!w.film;
      if (w.film) { v.film.geometry.attributes.position.array.set(w.film); v.film.geometry.attributes.position.needsUpdate = true; }
    }
    forkView.forEach((v, i) => { v.body.visible = v.tag.visible = i < s.forks.length; });
    s.forks.forEach((f, i) => {
      const v = forkView[i];
      v.body.position.set(f.x, f.y, 0); v.body.rotation.z = f.th;
      v.steer.rotation.z = f.phi; v.drive.rotation.y = f.wd; v.wl.rotation.y = f.wl; v.wr.rotation.y = f.wr;
      v.mast.rotation.y = -f.tilt; v.car.position.z = f.lift - MAST.pivot[1]; v.shift.position.y = f.shift;
      v.beacon.material.color.setHex(f.wait ? 0xef4444 : f.moving ? 0xf0b429 : 0x5a4a1a);
      v.tag.position.set(f.x, f.y, 2.6);
    });
    mmView.forEach((v, i) => { v.body.visible = v.tag.visible = i < s.robots.length; });
    s.robots.forEach((r, i) => {
      const v = mmView[i];
      v.body.position.set(r.x, r.y, 0); v.body.rotation.z = r.th;
      v.wl.rotation.y = r.wl; v.wr.rotation.y = r.wr;
      for (let k = 0; k < 6; k++) v.J[k].rotation.z = r.q[k];
      v.stop.position.z = DECK.z - DECK.stopH / 2 + DECK.stopH * r.stop;
      for (const cup of v.cups) cup.material.color.setHex(r.held ? 0x3ecf8e : r.vac ? 0xf0b429 : 0x2b2f36);
      const col = r.wait ? 0xf29b1d : r.work ? 0x3aa0ff : 0x46c46a;
      v.band.material.color.setHex(col); v.band.material.emissive.setHex(col);
      v.tag.position.set(r.x, r.y, 2.7);
    });
    let n = 0, nf = 0;
    for (const p of s.pallets) {
      if (n >= 200) break;
      setInst(palletIM, n++, p.x, p.y, p.z, p.yaw);
      const z0 = p.z + PALLET.h - 0.08;
      if (p.film > z0 + 0.02 && nf < 60) setInst(filmIM, nf++, p.x, p.y, (z0 + p.film) / 2, p.yaw, 1.212, 0.812, p.film - z0);
    }
    palletIM.count = n; palletIM.instanceMatrix.needsUpdate = true;
    filmIM.count = nf; filmIM.instanceMatrix.needsUpdate = true;
    n = 0;
    for (const b of s.boxes) {
      if (n >= 3000) break;
      setInst(boxIM, n, b.x, b.y, b.z, b.yaw, b.sx - 0.004, b.sy - 0.004, b.h - 0.003);
      boxIM.setColorAt(n++, tint.copy(skuColor[b.sku]).multiplyScalar(0.86 + 0.14 * (((b.id * 37) % 11) / 10)));
    }
    boxIM.count = n; boxIM.instanceMatrix.needsUpdate = true;
    if (boxIM.instanceColor) boxIM.instanceColor.needsUpdate = true;
    if (claimLines.visible && s.claims) claimLines.geometry.setAttribute('position', new THREE.Float32BufferAttribute(s.claims, 3));
  }

  // ---------- named points for cameras and pinned labels ({G}) ----------
  const tmp = new THREE.Vector3();
  const toG = (obj, x = 0, y = 0, z = 0) => { obj.updateWorldMatrix(true, false); const p = obj.localToWorld(tmp.set(x, y, z)); return [p.x, -p.z, p.y]; };
  function anchor(name, s) {
    const [kind, id] = name.split(':');
    if (kind === 'robot') { const i = +id.slice(1) - 1, r = s.robots[i]; return r && [r.x, r.y, 2.3]; }
    if (kind === 'arm') { const i = +id.slice(1) - 1; return i < s.robots.length ? toG(mmView[i].F[5], 0, 0, MTOOL.tcp) : null; }
    if (kind === 'fork') { const i = +id.slice(1) - 1, f = s.forks[i]; return f && [f.x, f.y, 2.4]; }
    if (kind === 'cell') { const P = PLACE.pal[+id]; return [P.tx, P.ty, 3.2]; }
    if (kind === 'tcp') return toG(cellView[+id].robot.F, 0, 0, 0.3);
    if (kind === 'infeed') { const P = PLACE.pal[+id], c = Math.cos(P.psi), sn = Math.sin(P.psi), x = PCELL.recv.x0, y = PCELL.recv.y; return [P.tx + c * x - sn * y, P.ty + sn * x + c * y, 1.0]; }
    if (kind === 'D') { const st = layout.cells[+id].station; return [st.x, st.y, 1.4]; }
    if (kind === 'port') { const l = layout.asrs.lanes[id]; return [l.x, l.y, 2.0]; }
    if (kind === 'crane') { const c = cranes[+id], q = s.cranes[+id]; return [A.tx + q[0], A.ty + c.y, q[1] + 1.0]; }
    if (kind === 'wrapper') return [WP.tx, WP.ty, 2.4];
    if (kind === 'I4') return [WP.tx + WRAPX.i4, WP.ty, 1.6];
    if (kind === 'O4') return [WP.tx + WRAPX.o4, WP.ty, 1.6];
    if (kind === 'storage') return [20.2, FLOOR.storage[0].y, 1.6];
    if (kind === 'empties') return [7.5, FLOOR.empties[0].y, 1.4];
    if (kind === 'receiving') return [10.5, FLOOR.receiving[0].y, 1.4];
    return null;
  }

  function resize(w, h) { renderer.setSize(w, h, !capture); camera.aspect = w / h; camera.updateProjectionMatrix(); }
  return {
    THREE, renderer, scene, camera, controls, W, roads, labels, claimLines, cellView, forkView, mmView, cranes, apply, anchor, resize,
    render() { renderer.render(scene, camera); },
    // {G} -> screen pixels (null when behind the camera or off screen)
    project(p) {
      const v = tmp.set(p[0], p[2], -p[1]).project(camera), el = renderer.domElement;
      if (v.z > 1 || Math.abs(v.x) > 0.98 || Math.abs(v.y) > 0.95) return null;
      return [((v.x + 1) / 2) * el.clientWidth, ((1 - v.y) / 2) * el.clientHeight];
    },
  };
}
