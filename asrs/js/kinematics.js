// Kinematics, workspace and safety monitor of the 3-axis (PPP) stacker crane
// defined in config.js. No rendering dependencies — runs in Node and browser.

import { CRANE, FORK, LOAD, RACK, PD, SAFE } from './config.js';

const { abs, sign } = Math;
const AX = CRANE.axes;

// ---------- fork deflection (spec §6.3) ----------
// Tip sag of the telescopic fork: cantilever law, cubic in extension, linear in load.
export const deflEmpty = (q3) => FORK.deflEmpty * (abs(q3) / FORK.stroke) ** 3;
export const deflLoad = (q3, mass) => FORK.deflRated * (mass / CRANE.payload) * (abs(q3) / FORK.stroke) ** 3;
export const deflection = (q3, mass = 0) => deflEmpty(q3) + deflLoad(q3, mass);

// ---------- forward / inverse kinematics (spec §5) ----------
// TCP = centre of the load plane of the top fork stage, in world coordinates.
export function fk(q, mass = 0) {
  return [q[0], q[2], q[1] - deflection(q[2], mass)];
}

// Unique solution (no posture branches). Returns null outside the joint limits.
export function ik(p, mass = 0) {
  const q = [p[0], p[2] + deflection(p[1], mass), p[1]];
  return inLimits(q) ? q : null;
}

export function inLimits(q, eps = 1e-9) {
  return q.every((v, i) => v >= AX[i].min - eps && v <= AX[i].max + eps);
}

// d(TCP)/dq, rows = world x, y, z; columns = q1, q2, q3.
export function jacobian(q, mass = 0) {
  const u = abs(q[2]) / FORK.stroke;
  const dd = sign(q[2]) * 3 * (FORK.deflEmpty + FORK.deflRated * (mass / CRANE.payload)) * u * u / FORK.stroke;
  return [
    [1, 0, 0],
    [0, 0, 1],
    [0, 1, -dd],
  ];
}
export const det3 = (m) =>
  m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
  m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
  m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

// ---------- addressing (spec §7) ----------
export const bayX = (bay) => RACK.x1 + (bay - 1) * RACK.pitchX;
export const levelZ = (level) => RACK.z1 + (level - 1) * RACK.pitchZ;

// A handling position: where the fork goes in and at which support plane.
export function rackSlot(row, side, bay, level) {
  return { kind: 'rack', row, side, bay, level, x: bayX(bay), zs: levelZ(level), id: `${row}-${String(bay).padStart(2, '0')}-${level}` };
}
export const STATION = {
  in: { kind: 'in', side: PD.inSide, x: PD.x, zs: PD.z, id: 'IN' },
  out: { kind: 'out', side: PD.outSide, x: PD.x, zs: PD.z, id: 'OUT' },
};

// Commanded hoist positions for a transfer at support plane zs (spec §8.2).
// The command is raised by the expected tip sag so the *deflected* fork is where it must be.
export function hoistTargets(zs, mass) {
  return {
    low: zs - FORK.cLow + deflEmpty(FORK.stroke), // fork passes under the resting pallet
    high: zs + FORK.cHigh + deflection(FORK.stroke, mass), // carried pallet passes over the supports
  };
}

// Static lateral margins (m) — the budget the X positioning has to live in.
export const MARGIN = {
  forkToLedge: (RACK.pitchX - RACK.uprightW) / 2 - RACK.ledgeW - (FORK.tineX + FORK.tineW / 2),
  palletToUpright: (RACK.pitchX - RACK.uprightW) / 2 - LOAD.sx / 2,
  palletBearing: LOAD.sx / 2 - ((RACK.pitchX - RACK.uprightW) / 2 - RACK.ledgeW),
  loadToRackFace: RACK.yFace - LOAD.sy / 2,
  forkToRackFace: RACK.yFace - FORK.length / 2,
};

// ---------- safety monitor (spec §9) ----------
// Evaluated from joint positions, joint speeds, the load sensors and the station deck
// switches only — it knows nothing about the sequencer. q is in the aisle-local frame.
// ctx.rackSides: sides of this aisle that have a rack; ctx.deck: roller-deck position
// (0 = down) of the station on each side. Returns a list of violated rules (empty = safe).
const OPEN = { rackSides: [1, -1], deck: { 1: 0, '-1': 0 } };
export function monitor(q, qd, carried, ctx = OPEN) {
  const bad = [];
  const mass = carried ? carried.mass : 0;
  if (!inLimits(q, 1e-6)) bad.push('R1 joint limit');

  const centred = abs(q[2]) <= FORK.centredWin;
  if (!centred) {
    if (abs(qd[0]) > 1e-6) bad.push('R2 travel moving with fork extended');
    if (abs(qd[1]) > CRANE.hoistCreep.v + 1e-6) bad.push('R3 hoist above creep speed with fork extended');
  }

  const inRack = abs(q[2]) > MARGIN.forkToRackFace; // any part of the fork beyond the rack face
  if (inRack) {
    const side = sign(q[2]);
    // which column are we in?
    const bay = Math.round((q[0] - RACK.x1) / RACK.pitchX) + 1;
    let zs = null, isRack = false;
    if (bay >= 1 && bay <= RACK.bays && abs(q[0] - bayX(bay)) <= AX[0].tol + 1e-9 && ctx.rackSides.includes(side)) {
      const lvl = Math.round((q[1] - RACK.z1) / RACK.pitchZ) + 1;
      if (lvl >= 1 && lvl <= RACK.levels) { zs = levelZ(lvl); isRack = true; }
    } else if (abs(q[0] - PD.x) <= AX[0].tol + 1e-9) {
      zs = PD.z;
      if (ctx.deck[side] > 1e-9) bad.push('R7 fork in a station whose roller deck is not down');
    }

    if (zs === null) bad.push('R4 fork extended outside a column / level');
    else {
      const plane = q[1] - deflection(q[2], mass);
      if (plane - FORK.thick < zs - SAFE.forkWinLow - 1e-9) bad.push('R5 fork below the cell window');
      if (plane > zs + SAFE.forkWinHigh + 1e-9) bad.push('R5 fork above the cell window');
      if (carried && isRack) {
        const clear = zs + RACK.pitchZ - RACK.ledgeLegH - (plane + carried.h);
        if (clear < SAFE.minLoadClearTop) bad.push('R6 load too close to the level above');
      }
    }
    if (side === 0) bad.push('R4 fork side undefined');
  }
  return bad;
}
