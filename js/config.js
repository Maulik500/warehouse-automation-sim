// Single source of truth for robot, tool and cell parameters.
// Mirrors ROBOT_SPEC.md — change a number here and in the spec together.
// Units: metres, radians, seconds, kilograms. World frame: right-handed, Z up.

export const D2R = Math.PI / 180;
export const R2D = 180 / Math.PI;
const deg = (a) => a.map((d) => d * D2R);

export const ROBOT = {
  name: 'PAL-6R-120/2700',
  // Link geometry (see spec §3)
  d1: 0.675, // floor -> J2 axis height
  a1: 0.350, // J1 axis -> J2 axis horizontal offset
  a2: 1.150, // J2 -> J3 (upper arm)
  a3: 0.041, // J3 -> J4 axis perpendicular offset
  d4: 1.200, // J3 -> wrist centre along forearm (J4 axis)
  d6: 0.215, // wrist centre -> flange face
  // Joint limits (spec §4)
  qMin: deg([-185, -60, -80, -350, -125, -350]),
  qMax: deg([185, 85, 150, 350, 125, 350]),
  vMax: deg([123, 115, 112, 179, 172, 219]), // rad/s
  aMax: deg([250, 250, 250, 450, 450, 600]), // rad/s^2
  // Cartesian limits for LIN moves
  vLin: 1.5, // m/s
  aLin: 3.5, // m/s^2
  payload: 120,
};

export const TOOL = {
  tcp: 0.300, // flange face -> suction plane, along flange +Z
  mass: 22,
  bodyX: 0.36, bodyY: 0.26, // footprint of the cup plate
  cups: 6, cupDia: 0.060, vacuum: 60e3, // Pa below ambient
  tGrip: 0.20, // vacuum build-up dwell
  tRelease: 0.15, // blow-off dwell
};

export const BOX = { sx: 0.295, sy: 0.395, sz: 0.250, mass: 12 };

export const CELL = {
  conv: {
    x: 1.75, top: 0.75, width: 0.60,
    yStart: -3.8, yEnd: 0.9, // belt runs in +Y
    v: 0.25, // m/s
    yA: -0.8, yB: 0.6, // tracking window (TCP may only follow the belt inside it)
    spawnEvery: 8.0, // s
  },
  pallet: {
    x: 0.0, y: 1.55, sx: 0.8, sy: 1.2, h: 0.144,
    pitchX: 0.4, pitchY: 0.3, nx: 2, ny: 4, layers: 4,
  },
  hoverPick: 0.15, // TCP height above box top before descending
  liftPick: 0.30, // minimum lift after grip
  hoverPlace: 0.35, // TCP height above final box top before descending (> box height)
  tSync: 0.40, // belt-speed synchronisation ramp
  yReady: -0.2, // idle hover position along the belt
  swapTime: 4.0, // pallet exchange
};
