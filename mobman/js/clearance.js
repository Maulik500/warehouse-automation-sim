// Clearance model: the arm is a chain of capsules, everything else is a box that may be turned
// about Z. Distances are sampled every 20 mm along a capsule axis. Spec §11.
// No rendering dependencies.

import { AMR, ARM, DECK, SRC, DST, PALLET } from './config.js';
import { armCapsules, armFrame, toWorld } from './base.js';
import { mv, add } from './arm.js';

const { abs, max, min, hypot, cos, sin, ceil, PI } = Math;

// Box centred at (cx, cy, cz) with full sizes (sx, sy, sz), turned by yaw about Z.
export const obb = (name, cx, cy, cz, sx, sy, sz, yaw = 0) => ({ name, c: [cx, cy, cz], h: [sx / 2, sy / 2, sz / 2], cs: cos(yaw), sn: sin(yaw) });

export function distPoint(o, p) {
  const dx = p[0] - o.c[0], dy = p[1] - o.c[1];
  const ex = max(abs(o.cs * dx + o.sn * dy) - o.h[0], 0), ey = max(abs(-o.sn * dx + o.cs * dy) - o.h[1], 0), ez = max(abs(p[2] - o.c[2]) - o.h[2], 0);
  return hypot(ex, ey, ez);
}
const inPlan = (o, p) => {
  const dx = p[0] - o.c[0], dy = p[1] - o.c[1];
  return abs(o.cs * dx + o.sn * dy) <= o.h[0] && abs(-o.sn * dx + o.cs * dy) <= o.h[1];
};

// Body-fixed obstacles for a base pose (world frame).
export function bodyObstacles(pose) {
  const B = AMR.body, th = pose.th, at = (x, y) => toWorld(pose, [x, y, 0]);
  const out = [];
  let c = at((B.xr + B.xf) / 2, 0);
  out.push(obb('chassis', c[0], c[1], AMR.chassisH / 2, B.xf - B.xr, B.w, AMR.chassisH, th));
  c = at((DECK.x0 + DECK.x1) / 2, DECK.yc);
  out.push(obb('deck', c[0], c[1], (AMR.chassisH + DECK.z) / 2, DECK.x1 - DECK.x0, DECK.w + 0.08, DECK.z - AMR.chassisH, th));
  for (const e of [1, -1]) {
    c = at((DECK.x0 + DECK.x1) / 2, DECK.yc + e * (DECK.w / 2 + 0.02));
    out.push(obb('deck guide', c[0], c[1], DECK.z + DECK.guideH / 2, DECK.x1 - DECK.x0, 0.04, DECK.guideH, th));
  }
  c = at(ARM.base[0], ARM.base[1]);
  out.push(obb('pedestal', c[0], c[1], (AMR.chassisH + ARM.base[2]) / 2, 2 * ARM.pedestalR, 2 * ARM.pedestalR, ARM.base[2] - AMR.chassisH, th));
  return out;
}

// Fixed obstacles of the cell (world frame).
export const SRC_FRAME = obb('source conveyor', SRC.endX - SRC.len / 2, SRC.y, SRC.z / 2, SRC.len, SRC.frameW, SRC.z);
export const DST_FRAME = obb('destination conveyor', DST.x, DST.yEnd - DST.len / 2, DST.z / 2, DST.w + 0.08, DST.len, DST.z);

// A pallet with its load: the pallet, one block for all complete layers under the top layer,
// and one box for every box of the top layer. `skip` leaves one box out.
export function palletObstacles(pal, skip) {
  const out = [obb('pallet', pal.x, pal.y, pal.z + PALLET.h / 2, PALLET.sx, PALLET.sy, PALLET.h, pal.yaw)];
  const left = pal.boxes.filter((b) => b.st === 'pallet');
  if (!left.length) return out;
  const top = max(...left.map((b) => b.layer));
  if (top > 0) {
    const [ex, ey] = pal.extent;
    out.push(obb('stack', pal.x, pal.y, pal.z + PALLET.h + (top * pal.rc.h) / 2, ex, ey, top * pal.rc.h, pal.yaw));
  }
  for (const b of left) if (b.layer === top && b !== skip) out.push(boxObb(b));
  return out;
}
export const boxObb = (b) => obb('box ' + b.id, b.p[0], b.p[1], b.p[2], b.sx, b.sy, b.h, b.yaw);

// Smallest distance between the arm and a list of obstacles. Returns { d, link, obstacle }.
const STEP = 0.02;
export function armClearance(q, pose, obstacles) {
  const F = armFrame(pose);
  let best = { d: Infinity, link: '', obstacle: '' };
  for (const cap of armCapsules(q)) {
    const a = add(F.p, mv(F.R, cap.a)), b = add(F.p, mv(F.R, cap.b));
    const n = max(1, ceil(hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) / STEP));
    for (let i = 0; i <= n; i++) {
      const u = i / n, p = [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u];
      for (const o of obstacles) {
        if (o.name === 'pedestal' && cap.name === 'shoulder') continue;
        const d = distPoint(o, p) - cap.r;
        if (d < best.d) best = { d, link: cap.name, obstacle: o.name };
      }
    }
  }
  return best;
}

// Vertical clearance under a carried box: its underside against the top of whatever is below.
export function carryClearance(box, obstacles) {
  let best = { d: Infinity, obstacle: '' };
  const zb = box.p[2] - box.h / 2, c = cos(box.yaw), s = sin(box.yaw);
  for (let i = -2; i <= 2; i++)
    for (let j = -2; j <= 2; j++) {
      const lx = (i / 2) * (box.sx / 2 - 0.005), ly = (j / 2) * (box.sy / 2 - 0.005);
      const p = [box.p[0] + c * lx - s * ly, box.p[1] + s * lx + c * ly];
      for (const o of obstacles) {
        if (!inPlan(o, p)) continue;
        const d = zb - (o.c[2] + o.h[2]);
        if (d < best.d) best = { d, obstacle: o.name };
      }
    }
  return best;
}

// Plan distance between the chassis outline and a fixed obstacle (both as rectangles).
export function chassisClearance(pose, o) {
  const B = AMR.body;
  let d = Infinity;
  const N = 12;
  for (let i = 0; i <= N; i++)
    for (const [x, y] of [[B.xr + ((B.xf - B.xr) * i) / N, B.w / 2], [B.xr + ((B.xf - B.xr) * i) / N, -B.w / 2], [B.xr, -B.w / 2 + (B.w * i) / N], [B.xf, -B.w / 2 + (B.w * i) / N]]) {
      const p = toWorld(pose, [x, y, o.c[2]]);
      d = min(d, distPoint(o, p));
    }
  return d;
}

// Is every point of the arm inside the chassis outline (plan view)? Returns the worst overhang.
export function armOverhang(q) {
  const B = AMR.body, cy = cos(ARM.yaw), sy = sin(ARM.yaw);
  let worst = -Infinity;
  for (const cap of armCapsules(q))
    for (const p of [cap.a, cap.b]) {
      const x = ARM.base[0] + cy * p[0] - sy * p[1], y = ARM.base[1] + sy * p[0] + cy * p[1];
      worst = max(worst, x + cap.r - B.xf, B.xr - (x - cap.r), abs(y) + cap.r - B.w / 2);
    }
  return worst;
}
export { PI };
