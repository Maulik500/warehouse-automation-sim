// Pallet recipes -> box positions. A layer is a list of { x, y, yaw } in the pallet frame {P}
// (X_p = the 1200 length, Y_p = the 800 width, origin at the centre); x, y is the box centre
// and yaw the angle of the long side of the box from X_p. Spec §4.

import { RECIPES, PALLET, DECK, DEG } from './config.js';

export function layerPattern(rc, k = 0) {
  const out = [];
  if (rc.grid) {
    const g = rc.grid, sx = g.yaw ? rc.w : rc.l, sy = g.yaw ? rc.l : rc.w;
    for (let j = 0; j < g.ny; j++)
      for (let i = 0; i < g.nx; i++)
        out.push({ x: (i - (g.nx - 1) / 2) * sx, y: (j - (g.ny - 1) / 2) * sy, yaw: g.yaw * DEG });
  } else {
    const depth = rc.rows.reduce((s, r) => s + (r.yaw ? rc.l : rc.w), 0), flip = k % 2 ? -1 : 1;
    let y = -depth / 2;
    for (const r of rc.rows) {
      const sx = r.yaw ? rc.w : rc.l, sy = r.yaw ? rc.l : rc.w;
      for (let i = 0; i < r.n; i++) out.push({ x: (i - (r.n - 1) / 2) * sx, y: flip * (y + sy / 2), yaw: r.yaw * DEG });
      y += sy;
    }
  }
  return out;
}

// Plan extents of a box at a given yaw (yaw is 0 or 90° in every recipe).
export const footprint = (rc, yaw) => (Math.abs(Math.sin(yaw)) > 0.5 ? [rc.w, rc.l] : [rc.l, rc.w]);

export function layerExtent(rc, k = 0) {
  let x = 0, y = 0;
  for (const b of layerPattern(rc, k)) {
    const [sx, sy] = footprint(rc, b.yaw);
    x = Math.max(x, Math.abs(b.x) + sx / 2);
    y = Math.max(y, Math.abs(b.y) + sy / 2);
  }
  return [2 * x, 2 * y];
}
export const stackHeight = (rc) => PALLET.h + rc.layers * rc.h;
export const perLayer = (rc) => layerPattern(rc).length;

// Every recipe must fit the pallet, the height limit, the deck and the gripper (spec §4.2).
export function checkRecipe(rc, lim, tool) {
  const bad = [];
  for (let k = 0; k < Math.min(rc.layers, 2); k++) {
    const [ex, ey] = layerExtent(rc, k);
    if (ex > PALLET.sx + 1e-9 || ey > PALLET.sy + 1e-9) bad.push(`layer ${k} is ${ex.toFixed(3)} x ${ey.toFixed(3)}: larger than the pallet`);
    if (ex > DECK.layer.sx + 1e-9 || ey > DECK.w - 0.02) bad.push(`layer ${k} does not fit the deck`);
  }
  if (stackHeight(rc) > PALLET.maxH + 1e-9) bad.push(`stack ${stackHeight(rc).toFixed(3)} m exceeds ${PALLET.maxH}`);
  if (rc.l > lim.lMax || rc.w > lim.wMax || rc.h > lim.hMax || rc.l < lim.lMin || rc.w < lim.wMin || rc.h < lim.hMin) bad.push('box size outside the gripper range');
  if (rc.mass > lim.mMax) bad.push('box too heavy');
  if (tool.padX > rc.l - 0.04 + 1e-9 || tool.padY > rc.w - 0.04 + 1e-9) bad.push('vacuum pad overhangs the box');
  if (perLayer(rc) > DECK.cap) bad.push('layer has more boxes than the deck holds');
  if (perLayer(rc) * rc.mass > DECK.maxMass) bad.push('layer heavier than the deck rating');
  return bad;
}

// Boxes of a full pallet in pick order: top layer first; inside a layer the row nearest to
// the AMR (smallest y_p) first, front (largest x_p) first.
export function palletBoxes(rc) {
  const out = [];
  for (let k = rc.layers - 1; k >= 0; k--) {
    const lay = layerPattern(rc, k).sort((a, b) => a.y - b.y || b.x - a.x);
    for (const b of lay) out.push({ ...b, z: PALLET.h + (k + 0.5) * rc.h, layer: k });
  }
  return out;
}

// Deck slots in fill order: the layer pattern laid on the deck, front row first (it leaves
// first and rests against the front stop), far side from the arm first. Coordinates in {B}.
export function deckSlots(rc) {
  return layerPattern(rc, 0)
    .map((b) => ({ x: DECK.layer.x + b.x, y: DECK.yc + b.y, z: DECK.z + rc.h / 2, yaw: b.yaw }))
    .sort((a, b) => b.x - a.x || a.y - b.y);
}

export { RECIPES };
