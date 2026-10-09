// Single source of truth for the turntable stretch wrapper ("belly wrapper"), its film carriage,
// the film clamp / cut / press unit ("gripper"), the pallet and the conveyor line.
// Mirrors WRAPPER_SPEC.md — change a number here and in the spec together.
// Units: metres, radians, seconds, kilograms, newtons. World frame {W}: right-handed, Z up,
// origin on the floor on the turntable axis, +X = conveying direction, mast on the −Y side.
// Table frame {T}: {W} rotated by q1 about +Z. Pallet frame {P}: origin = centre of the pallet
// underside, X_p = the 1200 length (conveying direction), Y_p = the 800 width.

export const G = 9.81;
export const DEG = Math.PI / 180;
export const RPM = (2 * Math.PI) / 60;

// EUR pallet, conveyed lengthwise on rollers (bottom boards run along X_p).
export const PALLET = {
  sx: 1.20, sy: 0.80, h: 0.144, mass: 25,
  maxH: 2.00, // pallet + load
  maxMass: 1000,
  mu: 0.30, // carton on carton / carton on wood, conservative
};

// Box patterns: nx along X_p, ny along Y_p, column stacked, flush with the pallet.
export const SKUS = [
  { sku: 'BX-3040', sx: 0.300, sy: 0.400, sz: 0.250, mass: 9, nx: 4, ny: 2, layers: [3, 6], color: 0xc49a6c },
  { sku: 'BX-6040', sx: 0.600, sy: 0.400, sz: 0.300, mass: 16, nx: 2, ny: 2, layers: [2, 5], color: 0xa8845c },
  { sku: 'BX-4027', sx: 0.400, sy: 0.8 / 3, sz: 0.200, mass: 6, nx: 3, ny: 3, layers: [4, 8], color: 0xd8c6a4 },
];

// Roller conveyor line. Zone i (0 … 6) = I3, I2, I1, T, O1, O2, O3 at x = (i − 3)·pitch;
// zone 3 is the turntable deck.
export const LINE = {
  z: 0.50, // top of the rollers = pallet underside
  pitch: 1.75,
  zones: ['I3', 'I2', 'I1', 'T', 'O1', 'O2', 'O3'],
  move: { v: 0.30, a: 0.40, j: 1.5 }, // one pitch, rest to rest
  xEnd: 0.95, // the line conveyors end this far from the turntable axis
  xFar: 7.90,
  frameW: 1.10, rollR: 0.04, rollPitch: 0.16, rollLen: 0.95,
  tDebounce: 0.5, // B1 must be true this long, pallet at rest, before "ready"
  stopTol: 0.010, // position scatter of a stopped pallet along X
  guideTol: 0.015, // lateral scatter between the side guides
  heightRes: 0.010, // resolution of the height light grid BH (rounds up)
  spawn: [60, 120], // s between pallets (uniform)
};

// Turntable: q1 (rotation) and q2 (roller deck on the turntable).
export const TABLE = {
  z: 0.50, plateZ: 0.40,
  rPlate: 0.90,
  rSwept: 0.92, // largest radius of anything that rotates
  deckX: 0.80, // outermost deck roller
  wrap: { v: 12 * RPM, a: 0.8, j: 2.0 }, // q1 limits: rad/s, rad/s², rad/s³
  homeTol: 0.5 * DEG, // q1 mod 2π at standstill, for transfer and for the clamp
  centreTol: 0.030, // pallet centre off the axis
  mass: 320, inertia: 110, // rotating parts, about the axis
};

// Film carriage on the mast: q3. The film leaves over the exit roller.
export const CARRIAGE = {
  roller: [0, -1.10], // exit roller axis in {W}
  rExit: 0.04,
  lift: { min: 0.70, max: 2.45, v: 0.40, a: 0.80, j: 4.0, tol: 0.002 }, // q3 = height of the film centreline
  exit: [30 * DEG, 80 * DEG], // window of the film direction α at the exit roller
  body: { x0: -0.46, x1: 0.07, y0: -1.38, y1: -1.03, h: 0.62 }, // housing, plan view; h centred on q3
  mast: { x: -0.20, y: -1.52, sx: 0.20, sy: 0.24, h: 2.95 },
};

// Film, powered pre-stretch (q4) and dancer (q5).
export const FILM = {
  w0: 0.500, // roll width
  w: 0.450, // web width on the load, after neck-down
  t0: 23e-6, density: 920,
  lambda: 3.0, // film length out / film length in = 200 % pre-stretch (fixed by the gear ratio)
  rollMass: 16, rCore: 0.043,
  minRoll: 30, // m left on the roll below which the roll is changed before the next pallet
  tRollChange: 90,
  F: 60, Fmin: 25, Fmax: 110, // web tension at the exit roller: set-point and limits
  feed: { vMax: 1.5, aMax: 4.0, tau: 0.03, kp: 12, ki: 20 }, // pre-stretch drive and its dancer controller
  dancer: { stroke: 0.060, k: 300, arm: 0.15 }, // stored film ±stroke; tension changes by k per metre stored
  mu: 0.40, // film on carton (capstan), without counting cling
};
FILM.length = FILM.rollMass / (FILM.density * FILM.w0 * FILM.t0); // unstretched film on a new roll

// Wrap program.
export const WRAP = {
  nBottom: 2, // revolutions at the bottom before the carriage starts up
  nTop: 2, // revolutions at the top
  nFinal: 1.5, // least revolutions at the bottom after the carriage is back down
  overlapUp: 0.50, overlapDown: 0.35,
  palletLock: 0.080, // lowest film edge below the pallet deck
  overTop: 0.100, // highest film edge above the load
  releaseAt: 300 * DEG, // q1 at which the clamp lets the tail go
  clampDownBy: 345 * DEG, // q1 by which the clamp must be below the film
  anchorCorners: 3, // corners the film must be wrapped round before the tail may be released
  omLift: 0.9 * 12 * RPM, // the carriage may only move above this turntable speed
};

// "Gripper", part 1: film clamp on the turntable. Position in {T}; it stands on the film line
// of the home pose (K.homeFilmLine). g1 = lift (0 down … 1 up), g2 = jaw (0 open … 1 closed).
export const CLAMP = {
  x: 0.330, y: -0.689,
  yaw: 46.9 * DEG, // jaw faces are parallel to the film at home
  stroke: 0.140, zDown: 0.544, // top of the jaws when down; up = zDown + stroke
  tLift: 0.30, tJaw: 0.15,
  open: 0.040, // jaw opening (±20 mm about the nominal film line)
  gripH: 0.120, // height of web between the jaws
  force: 250, mu: 0.60, // clamping force and jaw friction (rubber on film)
};

// "Gripper", part 2: cut-and-press slide on a fixed stand. g3 = y of the brush face in {W}.
export const CUT = {
  x: 0.48, brushW: 0.10, // brush pad centre and width along X
  wireX: 0.395, wireBack: 0.105, // hot wire: x position, and set-back behind the brush face
  park: -1.30, press: -0.40,
  move: { v: 0.80, a: 4.0, j: 40 },
  z: [0.55, 1.03], // brush and wire span
  tWire: 0.40, tPress: 0.30,
  tailX: 0.40, // where the film on the load ends after the cut (in {T}, on the −Y face)
};

export const SAFE = {
  rLoad: 0.80, // largest swept radius of a load that may be rotated
  sfLoad: 1.5, // least safety factor of the unwrapped load against sliding / tipping
  clear: 0.05, // least clearance between rotating and fixed parts
};
