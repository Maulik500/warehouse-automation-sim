// Single source of truth for the pallet AMR, its mast and fork, the pallet, the conveyor
// pick stations and the cell layout. Mirrors AMR_SPEC.md — change a number here and in the
// spec together.
// Units: metres, radians, seconds, kilograms. World frame {W}: right-handed, Z up.
// Body frame {B}: origin on the floor under the load-axle midpoint, +X = fork direction,
// +Y = left, +Z up.

export const G = 9.81;
export const DEG = Math.PI / 180;

// Joint order everywhere: q1 drive wheel, q2 steer, q3 tilt, q4 lift, q5 side shift.
export const AMR = {
  name: 'CB-AMR 1000',
  // ---- chassis: tricycle, two fixed load wheels in front, one steered drive wheel at the rear
  L: 1.30, // wheelbase: load axle -> steer axis, which is at (-L, 0) in {B}
  track: 1.00, // load wheels at y = ±track/2
  rLoad: 0.15, wLoad: 0.10,
  rDrive: 0.15, wDrive: 0.12,
  // plan outline used for clearance checks (chassis + mast, everything below conveyor height)
  body: { xr: -1.55, xf: 0.38, w: 1.10 },
  chassisXf: 0.20, chassisH: 0.72, hoodH: 1.05, groundClear: 0.08,
  mass: 2100, cg: [-0.68, 0, 0.55], // unladen, incl. battery and counterweight
  payload: 1000,
  // ---- drive: limits of the path speed v at the reference point (load-axle midpoint)
  drive: {
    empty: { v: 1.6, a: 0.6, j: 1.2 },
    loaded: { v: 1.2, a: 0.4, j: 0.8 },
    creep: { v: 0.30, a: 0.3, j: 0.6 }, // forks inside a station zone
  },
  vRev: 1.0, // reverse (counterweight leading)
  wheelV: 2.0, // drive wheel rim speed
  aLat: 1.0, // lateral acceleration v²·|κ|
  eStopDecel: 1.5,
  posTol: 0.005, headTol: 0.25 * DEG, // docking accuracy the clearance budget assumes
  // ---- steer (q2): angle of the drive wheel about +Z, 0 = straight
  steer: { min: -90 * DEG, max: 90 * DEG, v: 60 * DEG, a: 300 * DEG, tol: 0.5 * DEG },
};

// Mast: tilts about a Y axis through `pivot`; the lift moves along the (tilted) mast and the
// side shift moves the fork carriage along Y. Tilt is positive BACK (fork tips rise) = Ry(-q3).
export const MAST = {
  pivot: [0.30, 0.35], // tilt axis in {B}: x_t, z_t
  aFace: 0.15, // pivot -> fork face (front of the fork shanks), along the fork axis
  tilt: { min: -3 * DEG, max: 6 * DEG, v: 4 * DEG, a: 12 * DEG, j: 60 * DEG, tol: 0.1 * DEG },
  lift: { min: 0.05, max: 1.60, v: 0.30, a: 0.30, j: 1.0, tol: 0.002 }, // q4 = height of the fork top face at tilt 0
  liftLoaded: { v: 0.20, a: 0.25, j: 0.8 },
  liftCreep: { v: 0.05, a: 0.20, j: 1.0 }, // load transfer, forks inside a pallet
  shift: { min: -0.10, max: 0.10, v: 0.08, a: 0.30, j: 1.5, tol: 0.001 },
  travel: { lift: 0.30, tilt: 4 * DEG }, // the only mast pose allowed above creep speed
  x0: 0.24, x1: 0.38, w: 0.80, height: 2.15, // mast channels in {B} at tilt 0
};

// Load-handling device ("gripper"): two fixed tines. Form fit + gravity + tilt-back, no clamping.
export const FORK = {
  length: 0.80,
  tineW: 0.12, tineY: 0.264, // tines centred at y = ±0.264 -> overall width 0.648
  thick: 0.045,
  faceGap: 0.02, // pallet face -> fork face when fully inserted
  loadCentre: 0.42, // fork face -> TCP = faceGap + pallet depth / 2
  carriageW: 0.90, backrestH: 1.00,
  mu: 0.30, // steel on wood
  cEntry: 0.030, // fork top below the deck underside when entering / leaving a resting pallet
  cLift: 0.060, // carried pallet underside above the support when entering / leaving a station
  mass: 140, // carriage + tines, part of AMR.mass
};

// EUR pallet, 9 blocks. Pallet frame {P}: origin = centre of the underside, X_p = the 800 depth
// (fork direction when entering through the 1200 side), Y_p = the 1200 length.
export const PALLET = {
  sx: 0.80, sy: 1.20, h: 0.144, mass: 25,
  pocketLo: 0.022, // top of the bottom boards = floor of the fork pocket
  pocketHi: 0.122, // underside of the deck boards = ceiling of the fork pocket
  openY: [0.0725, 0.455], // |y| range of each fork window on the 1200 side
  blockY: [-0.5275, 0, 0.5275], blockLy: 0.145, blockX: [-0.35, 0, 0.35], blockLx: 0.10,
  maxH: 1.20, maxMass: 1000, cgMax: 0.60, // filled pallet limits (height incl. pallet, CG above the deck)
};

// Box patterns of a filled pallet: nx along X_p, ny along Y_p, flush with the pallet.
export const SKUS = [
  { sku: 'BX-4030', sx: 0.400, sy: 0.300, sz: 0.250, mass: 12, nx: 2, ny: 4, maxLayers: 4, color: 0xc49a6c },
  { sku: 'BX-4060', sx: 0.400, sy: 0.600, sz: 0.300, mass: 22, nx: 2, ny: 2, maxLayers: 3, color: 0x8fa9c4 },
  { sku: 'BX-2640', sx: 0.266, sy: 0.400, sz: 0.200, mass: 8, nx: 3, ny: 3, maxLayers: 5, color: 0x8fae8b },
];

// Pick stations: the last position of a chain conveyor. Pallets travel in -X, crosswise
// (1200 side leading), and are presented to the AMR, which enters heading +X.
export const CONV = {
  xP: 6.00, // pallet centre at the pick position
  lines: [{ id: 'P1', y: 1.60 }, { id: 'P2', y: -1.60 }],
  z: 0.610, // chain top = pallet underside
  v: 0.20, pitch: 1.00, cap: 6, // accumulation positions xP, xP + pitch, ...
  frameW: 1.40, endOverhang: 0.03, // frame ends this far before the near face of the presented pallet
  chainY: [-0.42, 0.42], chainW: 0.06,
  stopH: 0.085, // end stops stand this high above the chain, at the block positions only
  // sensors (station frame: x measured from the pallet near face, +X upstream)
  b1x: 0.06, b1z: 0.060, // B1 pallet present: beam across the conveyor through the front block row
  b2x: 0.60, // B2 slow-down
  b3z: 0.224, // B3 load height: beam above the deck -> pallet is filled
  tDebounce: 0.5, // B1 must be true this long, pallet at rest, before "ready"
  maxOffsetY: 0.06, // lateral position scatter of a presented pallet
  spawn: [120, 260], // s between pallets on a line (uniform)
  pEmpty: 0.4, // share of empty pallets
};

// Docking: the AMR stops at the pre-dock point, sets the mast, then creeps in.
export const DOCK = {
  standoff: 0.30, // fork tips before the pallet near face at the pre-dock point
  zoneIn: 0.25, // station fork zone begins this far before the pallet near face
  latMargin: 0.03, // tine vs pocket side wall, minimum
  zMargin: 0.010, // tine vs pocket floor / ceiling, minimum
};

// Continuous-curvature turn: quintic curvature ramp, constant arc, quintic ramp.
export const TURN = { R: 1.50, Lc: 1.50 };

// Drop stations. `th` is the heading of the AMR while docked.
export const CELL = {
  slots: [ // filled pallets, on the floor
    { id: 'F1', x: 0.40, y: -6.60, th: -90 * DEG, z: 0 },
    { id: 'F2', x: 1.90, y: -6.60, th: -90 * DEG, z: 0 },
    { id: 'F3', x: 3.40, y: -6.60, th: -90 * DEG, z: 0 },
  ],
  stack: { id: 'E1', x: 1.90, y: 6.60, th: 90 * DEG, z: 0, max: 6 }, // empty pallets, stacked on the floor
  home: { id: 'H', x: -1.60, y: -6.60, th: -90 * DEG }, // parking position (pre-dock point only)
  tSlotDwell: 90, // a deposited filled pallet is collected after this time
  tStackDwell: 25, // a full stack is collected after this time
};

export const SAFE = {
  clear: 0.05, // minimum plan clearance body / carried pallet vs anything else
  liftWin: 0.010, // carried pallet may be at most cLift + this above a support inside a zone
};
