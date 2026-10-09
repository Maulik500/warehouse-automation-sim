// Single source of truth for crane, load-handling device, rack and cell parameters.
// Mirrors ASRS_SPEC.md — change a number here and in the spec together.
// Units: metres, seconds, kilograms. World frame: right-handed, Z up, X along the aisle.

export const G = 9.81;

// Joint order everywhere: q = [q1 travel (world X), q2 hoist (world Z), q3 fork (world Y)].
export const CRANE = {
  name: 'SRM-P3-1000/13',
  axes: [
    { id: 'q1', name: 'Travel', min: 0.0, max: 19.8, v: 3.0, a: 0.5, j: 1.0, tol: 0.003 },
    { id: 'q2', name: 'Hoist', min: 0.5, max: 11.25, v: 1.0, a: 0.6, j: 1.5, tol: 0.002 },
    { id: 'q3', name: 'Fork', min: -1.4, max: 1.4, v: 1.0, a: 1.0, j: 4.0, tol: 0.002 },
  ],
  hoistCreep: { v: 0.08, a: 0.3, j: 1.5 }, // the only hoist motion allowed with the fork extended
  forkLoaded: { v: 0.6, a: 0.5, j: 2.0 }, // fork limits while carrying a load
  tSettle: 0.30, // mast-sway decay before the fork may extend
  tCentre: 0.10, // fork-centred confirmation before travel/hoist are released
  eStopDecel: 2.0, // travel emergency deceleration
  payload: 1000,
  // Structure (visual + collision envelope), relative to the fork centreline x = q1
  mastX: 0.85, mastSx: 0.30, mastSy: 0.40, mastH: 13.1,
  chassisLen: 3.0, chassisW: 0.90, chassisH: 0.28,
  carriageSx: 1.40, carriageSy: 0.90, carriageDrop: 0.19, // carriage frame spans q2-0.19 … q2-0.07
};

// Load-handling device ("gripper"): 3-stage telescopic twin fork, form-fit + gravity, no clamping.
export const FORK = {
  stroke: 1.40, // |q3| at full extension = aisle centre -> storage centre
  length: 1.25, // every stage, along Y
  tineW: 0.18, tineX: 0.16, // two tines centred at x = ±0.16 -> overall width 0.50
  thick: 0.07, // total stack height below the load plane (base 25 + middle 25 + top 20 mm)
  midRatio: 0.5, // middle stage travels q3/2 (2:1 differential drive)
  deflEmpty: 0.003, // tip sag at full stroke, own weight
  deflRated: 0.012, // additional tip sag at full stroke with rated payload
  cLow: 0.040, // fork top below the support plane when entering to pick / leaving after put
  cHigh: 0.040, // pallet underside above the support plane when entering to put / leaving after pick
  centredWin: 0.002, // |q3| below this = "fork centred" sensor true
  mu: 0.30, // steel fork / wooden pallet friction
};

export const LOAD = {
  sx: 0.80, sy: 1.20, palletH: 0.144, // EUR pallet, 1200 side into the rack
  palletMass: 25,
  maxH: 1.20, // pallet + boxes
  maxMass: 1000,
  cgMax: 0.60, // highest allowed centre of gravity above the load plane
};

export const RACK = {
  bays: 16, levels: 8, // per row
  x1: 3.30, pitchX: 1.10, // bay centre x_b = x1 + (b-1)*pitchX
  z1: 0.60, pitchZ: 1.50, // support plane z_l = z1 + (l-1)*pitchZ
  yFace: 0.75, // rack front face |y| -> clear aisle 1.50
  depth: 1.30,
  yStore: 1.40, // pallet centre |y| in a location
  uprightW: 0.10, // -> clear opening 1.00
  ledgeW: 0.18, ledgeLegH: 0.12, // pallet support angles; fork window between them = 0.64
  height: 13.4,
};

// Aisles. Each has its own crane and works in an aisle-local frame whose Y origin is the
// aisle centreline: world y = aisle.y + local y. side +1 = local +Y, side -1 = local -Y.
export const LAYOUT = {
  aislePitch: 4.30, // 2 x (0.75 + 1.30) + 0.20 back-to-back gap
  aisles: [
    { id: 1, y: 0.0, rows: [['A', +1], ['B', -1]] },
    { id: 2, y: -4.3, rows: [['C', +1], ['D', -1]] },
    { id: 3, y: -8.6, rows: [['E', +1]] }, // -Y side is free for a future row F
  ],
};

// Pick-up & deposit stations at each aisle head. A station supports the pallet on two fixed
// ledges like a rack location; a roller deck between the ledges rises 10 mm above them to
// move a pallet in or out along X and drops clear of the fork zone before the fork may enter.
export const PD = {
  x: 1.10, z: 0.60,
  inSide: +1, outSide: -1, // inbound lane in line with the +Y row, outbound lane with the -Y row
  ledgeX: 0.36, ledgeW: 0.08, // fixed supports, running along Y
  deckLift: 0.010, deckDrop: 0.20, tDeck: 1.0, // roller deck: top at z+lift (up) or z-drop (down)
  v: 0.20, // conveyor speed along X
  pitch: 1.00, // accumulation pitch along X (pallet is 0.80 long in X)
  xPort: -6.10, // inbound feed point and outbound handover point (pallet centre)
  laneCap: 7, // pallet positions per lane between the port and the station
};

// Box types. A pallet carries one SKU, stacked nx x ny per layer, flush with the pallet.
// sx is along world X (across the bay), sy along world Y (into the rack).
export const SKUS = [
  { sku: 'BX-4030', sx: 0.400, sy: 0.300, sz: 0.250, mass: 12, nx: 2, ny: 4, maxLayers: 4, color: 0xc49a6c },
  { sku: 'BX-4060', sx: 0.400, sy: 0.600, sz: 0.300, mass: 22, nx: 2, ny: 2, maxLayers: 3, color: 0x8fa9c4 },
  { sku: 'BX-2640', sx: 0.266, sy: 0.400, sz: 0.200, mass: 8, nx: 3, ny: 3, maxLayers: 5, color: 0xd9d2bd },
  { sku: 'BX-8060', sx: 0.800, sy: 0.600, sz: 0.350, mass: 35, nx: 1, ny: 2, maxLayers: 3, color: 0x8fae8b },
  { sku: 'BX-2030', sx: 0.200, sy: 0.300, sz: 0.150, mass: 4, nx: 4, ny: 4, maxLayers: 7, color: 0xc98f8f },
];

// Safety-monitor margins (see spec §9). The monitor is independent of the sequencer.
export const SAFE = {
  forkWinLow: 0.14, // fork stack may reach at most this far below a support plane while inside a cell
  forkWinHigh: 0.08, // fork load plane may rise at most this far above it
  minLoadClearTop: 0.05, // carried load top vs. the structure of the level above
  minLateral: 0.03, // fork vs. ledge, pallet vs. upright, after position tolerance
};

