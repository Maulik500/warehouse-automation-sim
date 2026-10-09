// Roads (spec §6.3). Everything is built from the two verified primitives: straight pieces and
// the 90° continuous-curvature turn. The forklift ring is one closed path; a station is a spur
// off it. The robot-AMR loop is a pair of paths per palletizing cell. No rendering dependencies.

import { Path, line, turn, tangentLength } from '../../amr/js/path.js';
import * as FK from '../../amr/js/kinematics.js';
import { ROAD, PLACE, PCELL, toG } from './config.js';

const { PI, abs, cos, sin } = Math;
export const T = tangentLength(PI / 2);
const wrapPi = (a) => Math.atan2(sin(a), cos(a));

// ---------- forklift ring: counter-clockwise, starts where the south leg becomes straight ----------
const { xW, xE, yS, yN } = ROAD.fl;
const LS = xE - xW - 2 * T, LE = yN - yS - 2 * T;
const RING_PIECES = [line(LS), ...turn(PI / 2), line(LE), ...turn(PI / 2), line(LS), ...turn(PI / 2), line(LE), ...turn(PI / 2)];
export const RING = new Path({ x: xW + T, y: yS, th: 0 }, RING_PIECES);
const LT = (RING.length - 2 * LS - 2 * LE) / 4;
const LEGS = {
  S: { s0: 0, len: LS, th: 0, at: (x) => x - (xW + T), corner: (p) => ({ x: p.x, y: yS }) },
  E: { s0: LS + LT, len: LE, th: PI / 2, at: (x, y) => y - (yS + T), corner: (p) => ({ x: xE, y: p.y }) },
  N: { s0: LS + LE + 2 * LT, len: LS, th: PI, at: (x) => xE - T - x, corner: (p) => ({ x: p.x, y: yN }) },
  W: { s0: 2 * LS + LE + 3 * LT, len: LE, th: -PI / 2, at: (x, y) => yN - T - y, corner: (p) => ({ x: xW, y: p.y }) },
};
// Pieces of the ring from ring position s over a distance d (lines are cut, turns are whole).
function ringSlice(s, d) {
  const out = [], L = RING.length;
  let i = RING.pieces.findIndex((p) => s >= p.s0 - 1e-9 && s < p.s0 + p.len - 1e-9);
  if (i < 0) i = 0;
  let off = s - RING.pieces[i].s0, left = d;
  while (left > 1e-9) {
    const p = RING.pieces[i], take = Math.min(p.len - off, left);
    if (p.k0 === 0 && p.k1 === 0) out.push(line(take));
    else { if (off > 1e-9 || take < p.len - 1e-9) throw new Error('ring slice cuts a turn'); out.push({ len: p.len, k0: p.k0, k1: p.k1 }); }
    left -= take; off = 0; i = (i + 1) % RING.pieces.length;
  }
  void L;
  return out;
}

// A forklift spur: leaves the ring at ring position sJ with a 90° turn and ends on the
// station's pre-dock pose. `leg` names the ring leg it hangs on.
export function makeSpur(st, leg) {
  const G = LEGS[leg], dock = FK.baseFor(st.x, st.y, st.th), pre = { x: dock.x - FK.D_IN * cos(st.th), y: dock.y - FK.D_IN * sin(st.th), th: st.th };
  const c = G.corner(pre), sJ = G.s0 + G.at(c.x - T * cos(G.th), c.y - T * sin(G.th));
  if (sJ < G.s0 - 1e-6 || sJ + T > G.s0 + G.len + 1e-6) throw new Error(`spur ${st.id}: junction outside leg ${leg}`);
  const stem = Math.hypot(pre.x - c.x, pre.y - c.y) - T;
  if (stem < 0.4) throw new Error(`spur ${st.id}: run-in ${stem.toFixed(3)} too short`);
  const angle = wrapPi(st.th - G.th), start = RING.at(sJ);
  const pieces = [...turn(angle), line(stem)];
  const path = new Path({ x: start.x, y: start.y, th: G.th }, pieces);
  return { id: st.id, sJ, pieces, path, pre, dock, stem, dockPath: new Path(pre, [line(FK.D_IN)]) };
}
// Route from the ring end of spur A (where a forklift stands after backing out) to the
// pre-dock point of spur B: along the ring, then into B.
const ROUTES = new Map();
export function forkRoute(a, b) {
  const key = a.id + '>' + b.id;
  if (!ROUTES.has(key)) {
    const d = (b.sJ - a.sJ + RING.length) % RING.length || RING.length;
    const start = RING.at(a.sJ);
    // a route holds a 1 mm position table (≈ 1.5 MB): keep the 80 most recently built
    if (ROUTES.size >= 80) ROUTES.delete(ROUTES.keys().next().value);
    ROUTES.set(key, new Path({ x: start.x, y: start.y, th: a.path.heading(0) }, [...ringSlice(a.sJ, d), ...b.pieces]));
  }
  return ROUTES.get(key);
}
export const ringDist = (a, b) => (b.sJ - a.sJ + RING.length) % RING.length;

// ---------- robot-AMR loop ----------
const M = ROAD.mm;
export const MM_HEAD = { x: M.xRoad, y: M.yTop, th: -PI / 2 };
export const MM_STANDOFF = 1.5;
// Robot AMR dock at palletizing cell i (global pose) and its pre-dock point.
export function cellDock(i) {
  const P = PLACE.pal[i], [x, y] = toG(P, PCELL.robotDock.x, PCELL.robotDock.y);
  return { x, y, th: PCELL.robotDock.th + P.psi };
}
export function mmRoutes(i) {
  const d = cellDock(i), pre = { x: d.x, y: d.y - MM_STANDOFF, th: PI / 2 };
  const run = pre.y - (M.yW + T);
  const out = new Path(MM_HEAD, [line(M.yTop - M.yW - T), ...turn(-PI / 2), line(M.xRoad - T - (d.x + T)), ...turn(-PI / 2), line(run)]);
  const back = new Path({ x: pre.x, y: pre.y, th: -PI / 2 }, [line(pre.y - (M.yE + T)), ...turn(PI / 2), line(M.xRet - T - (d.x + T)), ...turn(PI / 2), line(M.yTop - (M.yE + T))]);
  return { out, back, dock: new Path(pre, [line(MM_STANDOFF)]), pre, dockPose: d };
}
export const MM_LINK = new Path({ x: M.xRet, y: M.yTop, th: 0 }, [line(M.xRoad - M.xRet)]);
// position on an outbound route of the dock beside ASRS row y
export const sOfRoadY = (y) => M.yTop - y;
