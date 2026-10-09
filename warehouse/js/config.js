// Single source of truth for the integrated warehouse. Mirrors WAREHOUSE_SPEC.md — change a
// number here and in the spec together. Cell-internal numbers stay in the cells' own config.js.
// Units: metres, radians, seconds, kilograms. Global frame {G}: right-handed, Z up, origin at
// the south-west corner of the building, +X east, +Y north.

export const DEG = Math.PI / 180;
export const G = 9.81;
export const DT = 1 / 240;

export const BUILDING = { x: 68, y: 48 };
export const PALLET = { sx: 1.20, sy: 0.80, h: 0.144, mass: 25, pocketHi: 0.122 };

// ---- SKU master (spec §3.1). `rc` is a recipe in the mobile depalletizer's format
// (pattern in {P}, X_p = the 1200 length); `srcLayers` = layers of a full source pallet.
const sku = (id, row, l, w, h, mass, nx, ny, yaw, srcLayers, color) => ({
  id, row, l, w, h, mass, srcLayers, color, perLayer: nx * ny, layerMass: nx * ny * mass,
  rc: { id, sku: id, l, w, h, mass, layers: srcLayers, grid: { nx, ny, yaw }, color },
});
export const SKUS = [
  sku('S1', 'A', 0.400, 0.300, 0.250, 12, 4, 2, 90, 4, 0xc49a6c),
  sku('S2', 'B', 0.600, 0.400, 0.300, 15, 2, 2, 0, 3, 0x8fa9c4),
  sku('S3', 'C', 0.400, 0.266, 0.200, 8, 3, 3, 0, 5, 0x8fae8b),
  sku('S4', 'D', 0.300, 0.200, 0.150, 5, 4, 4, 0, 7, 0xc9b458),
  sku('S5', 'E', 0.600, 0.400, 0.400, 14, 2, 2, 0, 2, 0xa58fc4),
];
export const SKU = Object.fromEntries(SKUS.map((s) => [s.id, s]));
export const ROW_SKU = Object.fromEntries(SKUS.map((s) => [s.row, s]));

// ---- mixed pallet (spec §3.2)
export const MIX = { maxBoxH: 1.056, maxH: 1.200, minLayers: 2, maxLayers: 6 };

// ---- cell placement (spec §6.1): p_G = t + Rz(psi)·p_C
export const PLACE = {
  asrs: { tx: 45.0, ty: 35.0, tz: 0, psi: 0 },
  pal: [6.0, 14.2, 22.4, 30.6].map((tx, i) => ({ id: i + 1, tx, ty: 15.2, tz: 0.10, psi: -90 * DEG })),
  wrap: { tx: 20.0, ty: 34.6, tz: 0, psi: 0 },
};

// ---- ASRS row lanes and pick ports (spec §4.1, §8.1), in the ASRS cell frame
export const PORT = {
  x: -4.00, // pallet centre at the pick port
  z: 0.610, // lane top = pallet underside
  v: 0.20, a: 0.20, j: 0.5, // lane conveyor
  tStop: 0.5, // retractable end stops
  tDebounce: 0.5,
  scatter: 0.015, scatterYaw: 1.0 * DEG, // pose scatter of a presented pallet (measured by the arm camera)
  frameOver: 0.03, // lane frame ends this far beyond the pallet's outer face
  frameW: 0.96,
  fill: 0.55, // initial fill of every row
  lowFill: 0.50, // replenishment is ordered below this
};
export const ROWS = ['A', 'B', 'C', 'D', 'E'];

// ---- palletizing cell additions (spec §4.3, §8.3, §8.4), in the palletizer cell frame
export const PCELL = {
  build: { x: 0, y: 1.55 }, dock: { x: -2.40, y: 1.55 }, // pallet conveyor positions B and D
  convV: { v: 0.20, a: 0.20, j: 0.5 },
  q1Min: -30 * DEG, q1Max: 110 * DEG, // software range of J1
  hoverExtra: 0.10, // place hover: box height + this above the final box top …
  beltClear: 0.20, // … and at least this far (box underside) above the belt top
  minElbow: 0.35, minWrist: Math.sin(15 * DEG), minShoulder: 0.30,
  dyn: { S1: 1, S2: 0.85, S3: 1, S4: 1, S5: 0.70 }, // PTP speed / acceleration scale per SKU
  // box infeed line
  recv: { x0: 4.05, x1: 6.05, y: -4.30, z: 0.550, w: 0.86, v: 0.30, clearLen: 1.45 },
  incline: { x0: 2.05, x1: 4.05, z0: 0.750, z1: 0.550 },
  merge: { x: 1.75, y: -4.30 },
  meter: { y: -3.80, every: 8.0, v: 0.25 },
  beltW: 0.70, // tracking belt, wide enough for a 0.60 box lying across it
  robotDock: { x: 6.83, y: -4.44, th: 180 * DEG }, // robot AMR reference point, docked
  fenceX: -1.20,
};
// compact vacuum pad of the palletizing robot (spec §9.2)
export const PTOOL = { padX: 0.26, padY: 0.16, cups: 6, cupDia: 0.060, vacuum: 60e3, mu: 0.5, rowY: 0.04, colX: 0.08, mass: 14, sfMin: 2.0, peelMin: 1.5 };
export const MTOOL_PEEL = { rowY: 0.045, S5scale: 0.90, peelMin: 1.5 }; // mobile arm: pad cup rows at ±45 mm

// ---- wrapper additions (spec §4.4)
export const WRAPX = { i4: -7.00, o4: 7.00, handFrameW: 0.86 };

// ---- roads (spec §6.3)
export const ROAD = {
  fl: { xW: 3.0, xE: 35.5, yS: 23.0, yN: 40.0 }, // forklift ring, counter-clockwise
  mm: { xRoad: 39.75, xRet: 37.75, yW: 4.1, yE: 2.1, yTop: 44.0, queue: [42.0, 40.0] }, // robot AMR loop
  T: 2.275, // tangent length of the 90° turn (R 1.5, ramps 1.5)
  runIn: 1.175, // straight between a spur's turn and its pre-dock point: a waiting forklift stands clear of the tail swing of others
  blockLen: 6.0,
  apron: 2.0,
  gap: 0.30, // fleet monitor: least distance between two vehicle outlines
};

// ---- floor stations (spec §6.2): pallet centres
export const FLOOR = {
  storage: Array.from({ length: 12 }, (_, k) => ({ id: `S${String(k + 1).padStart(2, '0')}`, x: 12.0 + 1.5 * k, y: 45.4, th: 90 * DEG })),
  empties: [6.0, 7.5, 9.0].map((x, k) => ({ id: `E${k + 1}`, x, y: 45.4, th: 90 * DEG, max: 6 })),
  receiving: [9.0, 10.5, 12.0].map((x, k) => ({ id: `R${k + 1}`, x, y: 28.4, th: 90 * DEG })),
  parking: Array.from({ length: 6 }, (_, k) => ({ id: `P${k + 1}`, x: 15.5 + 3.5 * k, y: 28.4, th: 90 * DEG })),
  tShip: 600, // a stored pallet is shipped this long after deposit
  emptyStart: [6, 6, 0],
  emptyKeep: 12, tStack: 30, // above this many empties a full stack is collected after this dwell
};

// ---- WES (spec §10.2)
export const WES = { seed: 7, openOrders: 4, layersAhead: 2, minOrderLayers: 2, maxOrderLayers: 5, emptyLow: 3 };
export const FLEET = { forklifts: 3, robots: 2, maxForklifts: 6, maxRobots: 2 };

export const rot = (psi, x, y) => [Math.cos(psi) * x - Math.sin(psi) * y, Math.sin(psi) * x + Math.cos(psi) * y];
// cell frame -> global frame
export const toG = (P, x, y, z = 0) => { const [gx, gy] = rot(P.psi, x, y); return [P.tx + gx, P.ty + gy, P.tz + z]; };
export const toC = (P, x, y, z = 0) => { const [cx, cy] = rot(-P.psi, x - P.tx, y - P.ty); return [cx, cy, z - P.tz]; };
