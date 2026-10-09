// Single source of truth for the mobile depalletizer: differential-drive AMR, on-board roller
// conveyor, 6-axis arm, vacuum gripper, pallet recipes, stations and cell layout.
// Mirrors MOBMAN_SPEC.md — change a number here and in the spec together.
// Units: metres, radians, seconds, kilograms. World frame {W}: right-handed, Z up.
// Body frame {B}: origin on the floor under the midpoint of the drive axle, +X = forward
// (the end the on-board conveyor discharges over), +Y = left (the arm side), +Z up.

export const G = 9.81;
export const DEG = Math.PI / 180;
const deg = (a) => a.map((d) => d * DEG);

// ---- AMR base: differential drive, two driven wheels on one axle + four casters
export const AMR = {
  name: 'UL-AMR 300 D',
  track: 1.00, // b: distance between the drive wheels, at y = ±b/2
  rWheel: 0.10, wWheel: 0.06,
  casters: [[0.60, 0.46], [0.60, -0.46], [-0.60, 0.46], [-0.60, -0.46]], rCaster: 0.05,
  body: { xr: -0.75, xf: 0.75, w: 1.24 }, // plan outline of the chassis
  chassisH: 0.36, groundClear: 0.03,
  // masses and centres of gravity in {B} (stability, spec §8)
  parts: [
    { name: 'chassis, drives, battery', m: 380, cg: [0, 0, 0.17] },
    { name: 'conveyor module', m: 70, cg: [0.04, -0.14, 0.55] },
    { name: 'pedestal + controller', m: 55, cg: [0, 0.47, 0.70] },
  ],
  drive: { // limits of the path speed v of the reference point
    empty: { v: 1.5, a: 0.6, j: 1.2 },
    loaded: { v: 1.0, a: 0.4, j: 0.8 },
    creep: { v: 0.20, a: 0.2, j: 0.5 }, // inside a station zone
  },
  vRev: 1.0, // reverse (rear leading)
  wheelV: 1.8, // wheel rim speed
  wheelA: 1.2, // wheel rim acceleration
  omega: 60 * DEG, alpha: 120 * DEG, // yaw rate and yaw acceleration
  aLat: 0.6, // lateral acceleration v²·|κ|
  eStopDecel: 1.5,
  posTol: 0.010, headTol: 0.5 * DEG, // docking accuracy the clearance budget assumes
};

// ---- on-board conveyor: powered rollers along X_B, discharges over the front (+X)
export const DECK = {
  z: 0.650, // roller top
  yc: -0.14, // centreline in {B}
  w: 0.86, // clear width between the side guides
  x0: -0.70, x1: 0.75, // roller bed
  layer: { x: 0, sx: 1.20, sy: 0.80 }, // where a layer is built: centred at (x, yc)
  stopX: 0.62, stopH: 0.045, // pop-up front stop (raised while travelling and loading)
  backX: -0.62, // fixed rear stop
  guideH: 0.05,
  v: 0.30, a: 0.6, // transfer speed, ramp
  rollerPitch: 0.075, rollerR: 0.025,
  mu: 0.30, // carton on braked rollers
  cap: 16, maxMass: 130,
};

// ---- arm: 6R offset-wrist collaborative arm, 20 kg / 1.75 m class. DH in arm.js.
export const ARM = {
  name: 'CB-6R-20/1750',
  d1: 0.236, // base face -> J2 axis height
  a2: 0.862, // J2 -> J3 (upper arm)
  a3: 0.729, // J3 -> J4 (forearm)
  d4: 0.201, // lateral offset of the wrist from the J1 axis
  d5: 0.159, // J4 axis -> J6 axis
  d6: 0.154, // J5 axis -> flange face
  qMin: deg([-360, -360, -165, -360, -360, -360]),
  qMax: deg([360, 360, 165, 360, 360, 360]),
  vMax: deg([120, 120, 150, 210, 210, 210]), // rad/s
  aMax: deg([220, 220, 300, 450, 450, 450]), // rad/s²
  vLin: 1.0, aLin: 2.5, // Cartesian limits for LIN moves
  payload: 20, mass: 64,
  // where the link tubes really are: lateral offsets from the J1 axis plane, along the J2 axis
  offUpper: 0.215, offFore: 0.065,
  rLink: [0.085, 0.070, 0.060, 0.050], // capsule radii for clearance checks: upper arm, forearm, wrist, tool
  linkMass: [14, 24, 14, 12], // base + shoulder, upper arm, forearm, wrist (sum = mass)
  // mounting: {A} = {B} shifted by `base`, rotated by Rz(yaw). J1 = 0 points the arm over the front.
  base: [0, 0.47, 1.40], yaw: 180 * DEG,
  pedestalR: 0.11,
  travelTcp: [-0.40, -0.40, 1.20], // TCP in {B}, tool down, for driving: the whole arm is inside the chassis outline
  // posture branch used for everything (spec §6): shoulder −, elbow +, wrist +  ->  q5 = +90°
  branch: { shoulder: -1, elbow: 1, wrist: 1 },
  // singularity margins the monitor enforces while the arm moves (spec §7)
  minElbow: 0.25, minWrist: 0.50, minShoulder: 0.10,
};

// ---- gripper: vacuum pad for closed-top cartons, top pick only
export const TOOL = {
  tcp: 0.160, // flange face -> cup sealing plane, along flange +Z
  mass: 4.5, cg: 0.07,
  padX: 0.26, padY: 0.16, // pad footprint; tool X = long side
  cups: 6, cupDia: 0.050, vacuum: 60e3, // Pa below ambient
  mu: 0.5, // cup on carton
  tGrip: 0.25, tRelease: 0.15,
  sfMin: 2.0,
};

// ---- EUR pallet. Pallet frame {P}: origin = centre of the underside, X_p = the 1200 length
// (the conveying direction), Y_p = the 800 width.
export const PALLET = { sx: 1.20, sy: 0.80, h: 0.144, mass: 25, maxH: 1.50 };

// ---- recipes: one SKU per pallet. `l` ≥ `w` are the box footprint, `yaw` the angle of the
// long side from X_p. grid: nx × ny boxes; rows: explicit rows [{ n, yaw }] stacked along Y_p,
// mirrored in Y on every second layer (interlocked).
export const RECIPES = [
  { id: 'R1', sku: 'BX-4030', l: 0.400, w: 0.300, h: 0.250, mass: 12, layers: 5, grid: { nx: 4, ny: 2, yaw: 90 }, color: 0xc49a6c },
  { id: 'R2', sku: 'BX-6040', l: 0.600, w: 0.400, h: 0.300, mass: 15, layers: 4, grid: { nx: 2, ny: 2, yaw: 0 }, color: 0x8fa9c4 },
  { id: 'R3', sku: 'BX-4026', l: 0.400, w: 0.266, h: 0.200, mass: 8, layers: 6, grid: { nx: 3, ny: 3, yaw: 0 }, color: 0x8fae8b },
  { id: 'R4', sku: 'BX-3020', l: 0.300, w: 0.200, h: 0.150, mass: 5, layers: 9, grid: { nx: 4, ny: 4, yaw: 0 }, color: 0xc9b458 },
  { id: 'R5', sku: 'BX-4030', l: 0.400, w: 0.300, h: 0.250, mass: 12, layers: 5, rows: [{ n: 4, yaw: 90 }, { n: 3, yaw: 0 }], color: 0xc98f7a },
  { id: 'R6', sku: 'BX-6040H', l: 0.600, w: 0.400, h: 0.400, mass: 14, layers: 3, grid: { nx: 2, ny: 2, yaw: 0 }, color: 0xa58fc4 },
];
export const BOXLIM = { lMax: 0.60, wMax: 0.40, hMax: 0.40, lMin: 0.30, wMin: 0.20, hMin: 0.15, mMax: 15 };
// Order quantities drawn per trip; 0 = one full layer.
export const ORDERS = [0, 2, 3, 0, 4, 5, 8, 0];

// ---- source station: end position of a pallet roller conveyor, flow +X. Coordinates in {W}.
export const SRC = {
  x: 0, y: 1.25, // pallet centre at the end stop
  z: 0.300, // roller top = pallet underside
  armGap: 0.38, // J1 axis -> near long side of the pallet when docked
  frameW: 0.96, // outer width of the conveyor frame
  len: 4.2, // conveyor length upstream of the end stop
  endX: 0.66, // frame end (+X)
  b1x: 0.52, b1z: 0.060, // B1 pallet present: beam across the conveyor through the leading block row
  b2z: 0.060, // B2 pallet squared against the end stop (second beam, trailing block row)
  tDebounce: 0.5,
  tOut: 4, // a depleted pallet is taken away this long after the AMR has cleared the station
  tIn: 3, xIn: -2.7, vIn: 0.25, // then the next recipe enters here and is conveyed to the end stop
  scatter: 0.015, scatterYaw: 1.0 * DEG, // ± pose scatter of a presented pallet, measured by the arm camera
};

// ---- destination: roller conveyor receiving the layer over its end. Flow −Y in {W}.
export const DST = {
  x: 8.0, // conveyor centreline
  yEnd: -4.70, // receiving end (the face the AMR docks against)
  len: 5.0, w: 0.86, z: 0.650,
  gap: 0.030, // AMR front -> conveyor end when docked
  v: 0.30,
  inX: 0.45, // B5 infeed-clear photo-eye, distance from the receiving end
  clearLen: 1.45, // free length needed before a transfer may start
};

// ---- motion parameters of the pick-and-place cycle
export const CYCLE = {
  hover: 0.12, // TCP above the box top before the vertical approach / after the retreat
  lift: 0.08, // carried box bottom above the surface it leaves
  clear: 0.08, // carried box bottom above everything it passes over
  zoneIn: 0.25, // station zone begins this far before the docked pose
  tSettle: 0.3, // standstill before the arm is released
  tStop: 0.4, // pop-up stop travel time
};

export const DOCK = { standoff: 1.50 }; // pre-dock point: this far before the docked pose
export const TURN = { R: 1.50, Lc: 1.50 };

export const SAFE = {
  clear: 0.05, // chassis vs fixed equipment (plan view)
  armClear: 0.03, // monitor: arm or carried box vs anything
  armPlan: 0.06, carryPlan: 0.06, // planner: the same clearances, with margin for path sampling
};
