# Integrated Warehouse Specification — ASRS, Mobile Depalletizers, 4 Palletizing Cells, Belly Wrapper, Forklift AMR Fleet, Storage

This file is the contract for the integrated simulation. It does **not** restate the five cell
specifications; it places those cells in one building, defines everything that is new between
them (layout, interfaces, traffic, orders) and lists every number of a verified cell that has to
change for the integration.

| Cell | Specification (unchanged unless §4 says otherwise) | Instances |
|---|---|---|
| ASRS, 5 rows A–E, 3 cranes | [../asrs/ASRS_SPEC.md](../asrs/ASRS_SPEC.md) | 1 |
| Mobile depalletizer (AMR + conveyor deck + 6R arm) — "robot AMR" | [../mobman/MOBMAN_SPEC.md](../mobman/MOBMAN_SPEC.md) | 1–2 |
| Palletizing robot (6R, spherical wrist) | [../ROBOT_SPEC.md](../ROBOT_SPEC.md) | 4 |
| Belly wrapper (turntable stretch wrapper) | [../wrapper/WRAPPER_SPEC.md](../wrapper/WRAPPER_SPEC.md) | 1 |
| Pallet AMR (counterbalanced fork truck) — "forklift" | [../amr/AMR_SPEC.md](../amr/AMR_SPEC.md) | 1–6 |

**How to read the numbers.** `[V]` = carried over from a verified cell specification. `[N]` = new
for the integration. Every number is mirrored in [js/config.js](js/config.js); the `[N]` values
are checked by `npm run verify:warehouse` (results in §15, files and commands in §16).

**Decisions taken with the user (2026-10-08)**

| # | Decision |
|---|---|
| D1 | The ASRS gets **five output conveyors, one per row A–E** (the cell had three, one per aisle) |
| D2 | A source pallet that still holds boxes after a pick **returns to ASRS storage**; an empty one is collected by a forklift |
| D3 | The mixed pallet is built **one SKU per layer** ("rainbow pallet") |
| D4 | **No box in the whole system weighs more than 15 kg** |
| D5 | Fleet size is **configurable in the UI** (forklifts 1–6, robot AMRs 1–2; default 3 + 2) |
| D6 | **One** belly wrapper |
| D7 | three.js is the live simulation; a Blender script rebuilds the same warehouse, rigged and keyframed from an exported trajectory |
| D8 | Orders are auto-generated from a seed, and can be added by hand in a UI panel |

---

## 1. Conventions

| Item | Convention |
|---|---|
| Units | metres, radians internally (degrees in tables), seconds, kilograms |
| Global frame `{G}` | right-handed, **Z up**, origin on the floor at the south-west corner of the building, +X east, +Y north |
| Cell frame `{C}` | each cell keeps the world frame of its own specification, untouched. It is placed by `(t_x, t_y, t_z, ψ)`: `p_G = t + Rz(ψ)·p_C`, heading `θ_G = θ_C + ψ` |
| Vehicle pose | `(x, y, θ)` of the vehicle reference point in `{G}` (forklift: under the load-axle midpoint; robot AMR: under the drive-axle midpoint) |
| Pallet | EUR 1200 × 800 × 144, 25 kg, everywhere |
| Time base | one fixed step of **1/240 s** for every machine and vehicle (the step all five cells were verified with) |
| three.js mapping | everything is built in one group rotated −90° about three's X, so `{G}`(x, y, z) → three (x, z, −y). Cell groups are children of it, placed by `(t, ψ)`. No other axis conversion exists anywhere |

---

## 2. Stations and systems

| # | Station | Contains | Material in | Material out |
|---|---|---|---|---|
| 1 | **ASRS** | racks A–E, 3 stacker cranes, 5 row lanes ending in pick ports | replenishment pallets (forklift), returning part pallets | boxes (robot AMR), empty pallets (forklift) |
| 2 | **Palletizing cell** × 4 | 6R robot, box infeed line, pallet conveyor | boxes (robot AMR), empty pallet (forklift) | mixed pallet (forklift) |
| 3 | **Belly wrapper cell** | turntable wrapper, infeed and outfeed roller line | mixed pallet (forklift) | wrapped pallet (forklift) |
| 4 | **Storage area** | 12 floor slots | wrapped pallet (forklift) | shipped (timer) |
| 5 | **Empty pallet station** | 3 floor stacks of ≤ 6 | empty pallet (forklift) | empty pallet (forklift) |
| 6 | Receiving `[N, assumption A1]` | 3 floor slots | full single-SKU pallet (timer) | replenishment pallet (forklift) |
| 7 | Parking | 6 forklift bays, 2 robot-AMR queue positions | — | — |

No two stations touch: every transfer between stations is done by a vehicle.

### 2.1 Degrees of freedom of the whole warehouse (default fleet 3 + 2)

| System | Instances | Actuated axes each | Kind | Total |
|---|---|---|---|---|
| Stacker crane | 3 | 3 (travel, hoist, fork) | PPP | 9 |
| Palletizing robot | 4 | 6 + binary vacuum | 6R spherical wrist | 24 |
| Robot AMR | 2 | 2 wheels + 6 arm joints (+ deck drive, front stop, vacuum) | (2,0) base + 6R offset wrist | 16 |
| Forklift | 3 | 5 (drive, steer, tilt, lift, side shift) | (1,1) tricycle + mast | 15 |
| Belly wrapper | 1 | 4 servo + 1 passive dancer + 3 film-gripper axes | R + P + film | 8 |
| **Servo axes** | | | | **72** |
| Conveyor drives | | 5 ASRS lanes + 10 station decks, 4 × (receiving, incline, transfer, meter, tracking belt, pallet conveyor), wrapper line | on/off or speed | ≈ 45 |

---

## 3. Unit loads

### 3.1 SKU master (D4: ≤ 15 kg) `[N]`

One master for every cell. The five boxes are the mobile depalletizer's verified recipes R1, R2,
R3, R4 and R6 (all inside its gripper range), so the arm, its pad and its deck patterns are used
exactly as verified. The 22 kg and 35 kg ASRS boxes are removed.

| SKU | Box l × w × h | Mass | Layer pattern in `{P}` (X_p = 1200) | Per layer | Layer mass | Home row | Source pallet: layers / boxes / height / gross |
|---|---|---|---|---|---|---|---|
| S1 | 0.400 × 0.300 × 0.250 | 12 kg | grid 4 × 2, yaw 90° | 8 | 96 kg | A | 4 / 32 / 1.144 / 409 kg |
| S2 | 0.600 × 0.400 × 0.300 | 15 kg | grid 2 × 2, yaw 0° | 4 | 60 kg | B | 3 / 12 / 1.044 / 205 kg |
| S3 | 0.400 × 0.266 × 0.200 | 8 kg | grid 3 × 3, yaw 0° | 9 | 72 kg | C | 5 / 45 / 1.144 / 385 kg |
| S4 | 0.300 × 0.200 × 0.150 | 5 kg | grid 4 × 4, yaw 0° | 16 | 80 kg | D | 7 / 112 / 1.194 / 585 kg |
| S5 | 0.600 × 0.400 × 0.400 | 14 kg | grid 2 × 2, yaw 0° | 4 | 56 kg | E | 2 / 8 / 0.944 / 137 kg |

- Source pallets are single-SKU, column-stacked full layers, **≤ 1.20 m including the pallet**
  (the ASRS cell height, rule R6 of the ASRS) `[V]`.
- Every layer footprint is 1.200 × 0.800 (S3: 1.200 × 0.798), so any layer fully supports any
  other layer.
- Home row = the row a SKU is stored in. With one SKU per row, "row A output" always presents S1.

### 3.2 Mixed pallet (D3) `[N]`

| Parameter | Value |
|---|---|
| Composition | 2 … 6 layers, each a full layer of one SKU |
| Height | boxes ≤ **1.056**, pallet + load ≤ **1.200** — the limit the forklift, the ASRS and the palletizer reach all share |
| Mass | ≤ 409 kg (4 × S1) — far below the 1000 kg ratings |
| Layer sequence | descending **layer mass**: S1 (96) → S4 (80) → S3 (72) → S2 (60) → S5 (56); equal SKUs adjacent |
| Centre of gravity | ≤ 0.60 above the deck `[V limit]`; always met because the heaviest layers are lowest |
| Identity | pallet id, order id, list of (layer, SKU, box ids); every box id is tracked from the rack location to the storage slot |

An order is `{ id, lines: [{ sku, layers }] }` with `Σ layers·h ≤ 1.056`. One order = one mixed
pallet = one palletizing cell from first box to pallet hand-over.

---

## 4. What changes in each verified cell

Everything not listed here stays exactly as specified and verified.

### 4.1 ASRS

| Item | Was `[V]` | Becomes `[N]` | Why |
|---|---|---|---|
| Lanes | 3 inbound + 3 outbound, one-way, 7 pallets each | **5 bidirectional row lanes**, one pallet each, in line with rows A–E | D1 + D2: a pallet goes out to the pick port and back in on the same lane, so no return loop is needed |
| Stations (x = 1.10) | IN on the `+1` side, OUT on the `−1` side | every station does both: A (aisle 1, `+1`), B (aisle 1, `−1`), C (aisle 2, `+1`), D (aisle 2, `−1`), E (aisle 3, `+1`). Aisle 3 `−1` stays empty (future row F) | D1. Crane kinematics, station geometry and rules R1–R7 are untouched |
| Port position | x = −6.10 | **x = −4.00** (lane 5.10 m, ≈ 27 s each way at 0.20 m/s) | shorter out-and-back time |
| Port lane end | open towards −X | frame ends 30 mm beyond the pallet's −X face; end stops **retractable**, at the block positions only | the same port is entered by forklift tines (deposit and pick) and stood beside by the robot AMR |
| SKUs | 5 SKUs incl. 22 and 35 kg | SKU master §3.1, one SKU per row | D4 |
| Storage assignment | random within the aisle | random within the SKU's home row | D1 |
| Commands | store / retrieve | `retrieve(row)` → present at the port; `return(row)` → roll in, re-store with the new box count; `store(row)` for a replenishment pallet set on the port | D2 |
| Stand-in consumer | empties each pallet | off | the robot AMR does the taking |
| Pallet at the port after a pick | — | **stays at the port while more of its SKU is on order**, otherwise it goes back into the rack; an empty pallet waits for a forklift | found in the first runs: with one out-and-back lane per row, sending the pallet back after every layer made the lane the bottleneck (about 3 min per layer of the same SKU) |

A row lane is a single resource: from `retrieve` until the pallet is back in a rack location (or
collected empty), that row can serve nothing else. Rows A + B share crane 1 and rows C + D share
crane 2, so their requests queue on the crane.

### 4.2 Mobile depalletizer (robot AMR)

| Item | Was `[V]` | Becomes `[N]` | Why |
|---|---|---|---|
| Source pallet support plane | 0.300 | **0.610** (ASRS lane) | D1 |
| Source load height incl. pallet | ≤ 1.50 | ≤ 1.20 | ASRS cell height |
| Box tops to be gripped | 0.594 … 1.794 | **0.904 … 1.804** | follows from the two lines above |
| Carry swing height (TCP) | ≤ 2.124 | ≤ **2.084** nominal, + swing raises | §8.2 |
| Order size per trip | 2 … 16 boxes | **exactly one full layer** of one SKU (4, 8, 9 or 16 boxes) | D3 |
| Source dock | reverse 1.50 m in, forward out | **drive-through**: creep forward the last 1.50 m into the dock, creep forward 1.50 m out | the five docks lie on one one-way road (§6.3) |
| Turning round | never | **spin turn** (pivot about the reference point) at marked nodes, only at standstill and only with an empty deck | closed loop instead of an out-and-back route |
| Destination | one conveyor end | the receiving conveyor of palletizing cell 1–4 | — |
| Fleet | 1 | 1–2, with traffic control (§7) | D5 |

Pedestal height 1.40, arm, pad, deck, deck patterns, posture branch and monitor M1–M12: unchanged.

### 4.3 Palletizing cell

| Item | Was `[V]` | Becomes `[N]` | Why |
|---|---|---|---|
| Robot mounting | on the floor | on a **0.10 m riser**: `t_z = 0.10`, so cell z = 0 is 0.10 above the floor | the pallet now stands on a conveyor; with the riser it is still at cell z = 0 and every verified pose is unchanged |
| Pallet support | floor | 2-strand low-profile chain conveyor along cell X at y = 1.55, chain top = cell z 0 (floor + 0.10), positions **B** (build, x = 0) and **D** (dock, x = −2.40); pallet crosswise (1200 along Y) | user: palletize on the outfeed conveyor |
| Box supply | one carton type, one every 8 s, created at the belt start | boxes of five SKUs released from the infeed line of §8.3, one every ≥ 8 s | integration |
| Carton | 0.295 × 0.395 × 0.250, 12 kg | SKU master §3.1 | D4 |
| Gripper | 0.36 × 0.26 plate, 6 × Ø60 | **0.26 × 0.16 plate, 6 × Ø60** (§9.2) | the old plate is larger than the top of S4 (0.30 × 0.20) and would seal on neighbouring boxes |
| TCP offset | 0.300 | 0.300 (unchanged; the plate changes, not its length) | — |
| Pattern | 2 × 4, 4 layers of one carton | per layer the pattern of that layer's SKU (§3.1), X_p along cell +Y | D3 |
| Place hover height (TCP) | 0.35 above the final box top | the higher of **h + 0.10 above the final box top** and **belt top + h + 0.20**; then straight down | with 0.35 fixed, a carried 0.40 m box would pass 50 mm *through* its own layer; and on the low layers the swing left the belt end with the box only 30 mm above it (found in the first run) |
| Tracking belt width | 0.60 | **0.70** | S2 and S5 lie across the belt (0.60) |
| Stack top | 1.144 | ≤ 1.200 | §3.2 |
| J1 software range | −185 … +185° | **−30 … +110°** | keeps the arm out of position D, where forklifts dock (§8.4) |
| Box yaw on the belt | long side along the belt | as it left the deck; the pad is turned to the box's long side at the pick, and J6 supplies the difference to the place yaw | §8.3 |
| Speed / acceleration | 100 % | per SKU: S1, S3, S4 100 %, **S2 85 %, S5 70 %** | peel moment on the compact pad (§9.2) |

### 4.4 Belly wrapper

| Item | Was `[V]` | Becomes `[N]` | Why |
|---|---|---|---|
| Line zones | I3 … O3 | add **I4** (x = −7.00) and **O4** (x = +7.00), the forklift hand-over zones | they are the entry / exit points of the wrapper specification |
| Hand-over zones | — | rollers 0.80 long, frame 0.86 wide and flush with the roller tops, no side guides; a centring funnel between I4 and I3 | the forklift mast needs 60 mm to the frame when docked from the side |
| Load | 3 single-SKU load types, 0.744 … 1.744 high | mixed pallets, 0.444 … 1.200 high | inside the verified wrap program range 0.344 … 2.000 |
| Supply / removal | timers | forklift deposits at I4 / collects at O4 | integration |
| Stability rule R10 | per SKU column | evaluated per mixed pallet (§9.5) | D3 |

### 4.5 Forklift

| Item | Was `[V]` | Becomes `[N]` | Why |
|---|---|---|---|
| Fleet | 1 | 1–6 with traffic control (§7) | D5 |
| Paths | 10 fixed L-routes | a road network built from the **same two primitives** (straight piece, 90° continuous-curvature turn R 1.5 / T 2.275) | integration |
| Stations | 2 conveyor ends, 3 floor slots, 1 stack | §6.2 | integration |
| Deposit **onto** a conveyor | not used | used at palletizer D, wrapper I4 and ASRS ports: the deposit sequence of the AMR spec §8.1 with the support plane of the conveyor | integration |
| Pick **from** a stack | not used (stack was deposit-only) | used at the empty pallet station: the pick sequence with support plane `(n − 1)·0.144` | integration |
| Job source | station sensors of its own cell | fleet manager (§10.3) | integration |

Lift targets (entry = z_s + 0.092, carry = z_s + 0.182 `[V]`):

| Station | Support plane z_s | Entry | Carry |
|---|---|---|---|
| Floor slot (storage, receiving) | 0 | 0.092 | 0.182 |
| Empty stack, n pallets after / before the move (n = 1 … 6) | (n − 1) · 0.144 | 0.092 … 0.812 | 0.182 … 0.902 |
| Palletizer position D | 0.100 | 0.192 | 0.282 |
| Wrapper I4 / O4 | 0.500 | 0.592 | 0.682 |
| ASRS port | 0.610 | 0.702 | 0.792 |

All inside the verified lift range 0.050 … 1.600.

---

## 5. Process

### 5.1 Material flow

```
              boxes, one layer per trip                 mixed pallet            wrapped pallet
  ASRS port ─────────────────────────▶ Palletizer ───────────────▶ Wrapper ───────────────▶ Storage
  (row A–E)        robot AMR           cell 1–4         forklift    I4 → O4      forklift    S01–S12
      │  ▲                                ▲
      │  │ replenishment pallet           │ empty pallet
      │  └──────────── Receiving          │
      │                 forklift          │ forklift
      └────────────────────────────▶ Empty pallet station
          empty source pallet (forklift)       E1–E3
          part pallet → back into the rack (ASRS lane + crane)
```

### 5.2 One order, step by step

| # | Actor | Action | Condition to start |
|---|---|---|---|
| 1 | WES | assign the order to a free palletizing cell; sort its layers (§3.2) | cell idle, empty pallet at B |
| 2 | WES → ASRS | `retrieve(row)` for the first layer | row lane free |
| 3 | Fleet manager | send a robot AMR to that row's dock | — |
| 4 | Crane + lane | pallet from the rack to the pick port (≈ 60 s) | — |
| 5 | Robot AMR | dock, measure the pallet, move the **top layer** onto the deck, stow | `PalletReady` ∧ `Locked` |
| 6 | ASRS | boxes left → `return(row)`; none left → pallet stays, forklift job "empty to E" | robot AMR has sent `Clear` |
| 7 | Robot AMR | drive to the cell, dock nose-on, roll the layer across | `DestReady` of that cell |
| 8 | Infeed line | take the layer apart into a single file, one box per ≥ 8 s | — |
| 9 | Palletizing robot | pick from the moving belt, place the layer | box in the tracking window |
| 10 | — | steps 2–9 for the next layer; layers must arrive in stacking order | — |
| 11 | Pallet conveyor | pallet B → D | last box released, robot retracted |
| 12 | Forklift | D → wrapper I4 | `PalletReady` at D, I4 free |
| 13 | Forklift | empty pallet from E to D; conveyor D → B | D free |
| 14 | Wrapper | I4 → … → wrap → … → O4 | wrapper spec §6 |
| 15 | Forklift | O4 → next free storage slot | `PalletReady` at O4, slot free |

Steps 2–3 for layer *k + 1* are issued while layer *k* is still travelling, so the ASRS and the
robot AMR work ahead of the palletizer.

### 5.3 Back-pressure (who waits for whom)

| If this is full / missing | Then |
|---|---|
| No storage slot free | wrapped pallets stay on the outfeed (O1 … O4), then the wrapper stops discharging |
| Wrapper infeed I4 occupied | the mixed pallet waits at D; that cell cannot take a new empty pallet |
| No empty pallet at B | the cell accepts no order |
| Receiving conveyor of a cell not clear (B5) | the robot AMR waits at the pre-dock point with its layer |
| Row lane busy | the next request for that row waits; the robot AMR waits at the road head |
| No empty stack space (3 × 6) | an empty source pallet stays on its port; that row is blocked. A full stack is collected when more than 12 empties are in the station (§8.6) |
| Empty stacks all at 0 | cells wait; the WES raises "empty pallets low" at ≤ 3 |

---

## 6. Layout

Building: x = 0 … 68, y = 0 … 48, flat floor.

```
 y
 48 ┌───────────────────────────────────────────────────────────────────────────────────────┐
    │      E1 E2 E3     S01 S02 … S12   (floor slots, y = 45.4)                  MM U-turn  │
 44 │       ▲  ▲  ▲      ▲   ▲       ▲                                         ┌──▶──┐      │
 40 │  ┌──◀─┴──┴──┴──────┴───┴───────┴────────◀──── FL-N (y = 40) ─────────◀──┐ │     ▼      │
    │  │           ▼ I4                 ▼ O4                                  │ ▲   Q1,Q2    │
 35 │  │      ═════╪═══ WRAPPER ════════╪═════  (line y = 34.6)               │ │     ▼  A ══╪═ rack A
    │  ▼                                                               FL-E   ├─┼────▶   B ══╪═ rack B
    │ FL-W                                                             x=35.5 ├─┼────▶   C ══╪═ rack C   ASRS
 28 │ x = 3    R1 R2 R3          P1 … P6     (floor, y = 28.4)              ├─┼────▶   D ══╪═ rack D
    │  │          ▲  ▲  ▲             ▲     ▲                                 ├─┼────▶   E ══╪═ rack E
 23 │  └──▶───────┴──┴──┴──── FL-S (y = 23) ──▶───────────────────────────▶───┘ │     │
    │        ▼ D         ▼ D         ▼ D         ▼ D                            │     │ MM road
 18 │       ┌──┐        ┌──┐        ┌──┐        ┌──┐                            ▲     ▼ x = 39.75
    │      PAL 1       PAL 2       PAL 3       PAL 4      (robots at y = 15.2)  │     │
  9 │    ▲           ▲           ▲           ▲          (receiving conveyors)   │     │
    │    │           │           │           │                          MM return     │
 4.1│  ◀─┴───────────┴───────────┴───────────┴────── ML-W (y = 4.1) ──────◀─────┼─────┘
 2.1│  ──▶──────────────────────────────────────── ML-E (y = 2.1) ──────▶──────┘ x = 37.75
  0 └───────────────────────────────────────────────────────────────────────────────────────┘
    0                                                                                   68  x
```

### 6.1 Cell placement `[N]`

| Cell | `t_x` | `t_y` | `t_z` | `ψ` | Notes |
|---|---|---|---|---|---|
| ASRS | 45.00 | 35.00 | 0 | 0° | racks x = 47.75 … 65.5; ports at x = 41.00 |
| Palletizer 1 | 6.00 | 15.20 | 0.10 | −90° | cell +X → south, cell +Y → east |
| Palletizer 2 | 14.20 | 15.20 | 0.10 | −90° | pitch 8.20 |
| Palletizer 3 | 22.40 | 15.20 | 0.10 | −90° | |
| Palletizer 4 | 30.60 | 15.20 | 0.10 | −90° | |
| Wrapper | 20.00 | 34.60 | 0 | 0° | line along X, mast on the south side, forklifts on the north side |

For ψ = −90°: `p_G = (t_x + y_C, t_y − x_C)`.

### 6.2 Stations in `{G}` `[N]`

Forklift: docked base = pallet centre − 0.870 along the heading, pre-dock = a further 1.080 back `[V]`.

| Station | Pallet centre (x, y) | Support z | Forklift heading | Docked base | Pre-dock base | Junction on |
|---|---|---|---|---|---|---|
| ASRS port A | (41.00, 36.40) | 0.610 | 0° | (40.13, 36.40) | (39.05, 36.40) | FL-E |
| ASRS port B | (41.00, 33.60) | 0.610 | 0° | (40.13, 33.60) | (39.05, 33.60) | FL-E |
| ASRS port C | (41.00, 32.10) | 0.610 | 0° | (40.13, 32.10) | (39.05, 32.10) | FL-E |
| ASRS port D | (41.00, 29.30) | 0.610 | 0° | (40.13, 29.30) | (39.05, 29.30) | FL-E |
| ASRS port E | (41.00, 27.80) | 0.610 | 0° | (40.13, 27.80) | (39.05, 27.80) | FL-E |
| Palletizer *i* position D | (s_i, 17.60), s = 7.55, 15.75, 23.95, 32.15 | 0.100 | −90° | (s_i, 18.47) | (s_i, 19.55) | FL-S |
| Palletizer *i* position B (build) | (s_i, 15.20) | 0.100 | — | — | — | — |
| Wrapper I4 (deposit) | (13.00, 34.60) | 0.500 | −90° | (13.00, 35.47) | (13.00, 36.55) | FL-N |
| Wrapper O4 (pick) | (27.00, 34.60) | 0.500 | −90° | (27.00, 35.47) | (27.00, 36.55) | FL-N |
| Storage S01 … S12 | (12.00 + 1.50·k, 45.40), k = 0 … 11 | 0 | +90° | (x, 44.53) | (x, 43.45) | FL-N |
| Empty stacks E1 … E3 | (6.00 / 7.50 / 9.00, 45.40) | n · 0.144 | +90° | (x, 44.53) | (x, 43.45) | FL-N |
| Receiving R1 … R3 | (9.00 / 10.50 / 12.00, 28.40) | 0 | +90° | (x, 27.53) | (x, 26.45) | FL-S |
| Parking P1 … P6 | — | — | +90° | — | (15.50 + 3.50·k, 26.45) | FL-S |

Robot AMR:

| Station | Dock pose (x, y, θ) | Zone | What is where |
|---|---|---|---|
| ASRS dock, row r | (39.75, y_r, −90°), y_r = 36.40, 33.60, 32.10, 29.30, 27.80 | y_r ± 1.50 on the road | pallet centre 1.25 to the left (east); arm J1 axis at x = 40.22 |
| Palletizer *i* infeed dock | (m_i, 8.37, +90°), m = 1.56, 9.76, 17.96, 26.16 | pre-dock (m_i, 6.87) | receiving conveyor centreline x = m_i + 0.14, receiving end at y = 9.15, gap 30 mm |
| Queue positions Q1, Q2 | (39.75, 42.00, −90°), (39.75, 40.00, −90°) | — | head of the ASRS road; parking |

### 6.3 Roads `[N]`

All roads are one-way. Every corner and every junction is the verified 90° continuous-curvature
turn (R 1.50, ramps 1.50, tangent length **T = 2.275**, κ, κ′, κ″ continuous) `[V]`.

**Forklift ring** (counter-clockwise, 96.2 m along the path):

| Leg | Line | Direction | Junctions on it |
|---|---|---|---|
| FL-S | y = 23.00, x = 3.00 … 35.50 | east | south: palletizer D at s_i; north: receiving, parking |
| FL-E | x = 35.50, y = 23.00 … 40.00 | north | east: ASRS ports A–E |
| FL-N | y = 40.00, x = 35.50 … 3.00 | west | south: wrapper I4, O4; north: storage, empty stacks |
| FL-W | x = 3.00, y = 40.00 … 23.00 | south | — |

A station is a **spur** off the ring, entered and left exactly like the verified L-route:

```
enter:  on the ring, 90° turn onto the spur, straight run-in ≥ 0.775, stop at the pre-dock point
dock:   creep 1.08 straight in, transfer, creep 1.08 straight back          (AMR spec §8.1)
leave:  REVERSE back through the same turn onto the ring, stop, drive on forwards
```

So a forklift leaving a spur ends on the ring *upstream* of the junction, facing downstream; it
needs the ring between `junction − T − 1.55` and the junction for itself during the manoeuvre.
Straight run-in: **1.175** at every spur (1.275 at the ASRS ports) — 0.40 more than the 0.775 of
the forklift cell. With 0.775 a forklift waiting at a pre-dock point stood inside the tail swing
of a forklift turning into the spur opposite, or round a ring corner next to it; three forklifts
locked each other up in the first long runs. With 1.175 the rear of a waiting forklift is 1.70 m
from the lane centreline, and `verify:warehouse` checks every standing position against the
ring sweep and against the entry sweep of every other station.

**Robot AMR loop** (clockwise, ≈ 150 m):

| Leg | Line | Direction | Notes |
|---|---|---|---|
| ASRS road | x = 39.75, y = 44.00 → 4.10 | south | five drive-through docks; Q1, Q2 at its head |
| ML-W | y = 4.10, x = 39.75 → 1.56 | west | loaded; right turn north into infeed spur *i* at corner (m_i, 4.10), run-in 0.495 |
| Infeed spur *i* | x = m_i, y = 4.10 … 8.37 | north in | creep the last 1.50 |
| Leaving a cell | reverse 1.50 to the pre-dock point, **spin 180°**, south to y = 4.375, left turn onto ML-E at (m_i + 2.275, 2.10) | | crosses ML-W |
| ML-E | y = 2.10, → x = 37.75 | east | empty |
| Return | x = 37.75, y = 2.10 → 44.00 | north | empty |
| U-turn | (37.75, 44.00) spin −90°, 2.00 east, spin −90° | | onto the head of the ASRS road |

Lane spacings: ML-W / ML-E 2.00; return lane / ASRS road 2.00; return lane / FL-E 2.25
(robot AMR 1.24 wide, forklift 1.10 wide; the forklift tail swings 1.07 outside its path in a
corner).

Where the two fleets meet: a forklift serving an ASRS port stands across both robot-AMR lanes
(its pre-dock pose covers x = 37.5 … 40.3). The **apron** of a port — both robot-AMR lanes within
y_r ± 2.0 and the port itself — is used by one forklift at a time (§7).

---

## 7. Traffic control `[N]`

| Element | Rule |
|---|---|
| Roadmap | a directed graph: nodes = junctions, corners, pre-dock points, dock poses, spin nodes, queue positions; edges = straight, 90° turn, spin — the only three primitives |
| Claims | every vehicle holds a **claim**: the plan area it stands on plus the area its current run sweeps — its outline grown by 0.20 m all round (forklift 3.25 × 1.60, robot AMR 1.90 × 1.64, a circle of r 1.30 for a spin), sampled every 0.4 m along the path. Two claims never overlap, so two outlines are never closer than 0.40 m. This replaces fixed blocks: it also covers tail swing, crossings and the two fleets meeting, without a rule for each |
| A run | a rest-to-rest S-curve along the path `[V motion law]`. **A vehicle starts a run only when the whole sweep of the run is free**, and gives the area back behind it as it goes. No vehicle ever has to brake for another: it has either not started or it has a clear road |
| Stops | only at permitted points: every ≈ 4 m on straight pieces, ≥ 0.3 m from any curve (κ = 0, κ′ = 0), so steer = 0 at every stop and reversal `[V]` |
| Run length | to the goal if the whole sweep is free; otherwise to the last permitted stopping point of the part that is free |
| Grant order | first come, first served |
| No stopping in a way out | a forklift on the ring never stops where a forklift that is on a spur, or on its way into one, has to back out. Without this rule the one arriving waits for the one leaving and the one leaving waits for the one arriving (found in the first long runs) |
| Neighbouring stations | no forklift job is given for a station while another forklift is at, or has a job for, a station within 2.0 m of it (storage slots, stacks and receiving slots are 1.50 apart) |
| ASRS apron | a forklift at a port stands across both robot-AMR lanes; **one forklift job at the ports at a time**, and it takes the apron only when its pallet is on the forks |
| Neighbouring docks | rows B/C and D/E are 1.50 apart, the robot AMR is 1.50 long: two robot AMRs never dock at B and C, or at D and E, at the same time (their claims overlap) |
| Speeds | forklift: 1.6 empty / 1.2 loaded / 0.30 creep / ≤ 1.0 reverse, 1.108 through a turn. Robot AMR: 1.5 empty / 1.0 loaded / 0.20 creep, 0.949 through a turn `[V]` |
| Spin turn (robot AMR) | at standstill, S-curve on the heading with 60°/s, 120°/s², deck empty, arm in the travel pose; wheels turn at ±ω·b/2 ≤ 0.52 m/s |
| Safety monitor | one instance per vehicle and machine, each reading only its own state `[V]`, plus one fleet monitor: no two vehicle outlines closer than 0.30 m |

Not used: free-space planning, overtaking, bidirectional lanes, coordinated arm + base motion.

---

## 8. Stations in detail `[N]`

### 8.1 ASRS pick port (one per row)

| Parameter | Value |
|---|---|
| Presented pallet | centre (41.00, y_r), underside 0.610, 1200 along Y, pose scatter ±15 mm, ±1.0° (measured by the arm camera) `[V scatter]` |
| Access | from the west only: forklift head-on, robot AMR parked across the end |
| Robot AMR position | reference point 1.25 west of the pallet centre, heading south; J1 axis 0.38 from the near pallet face, 1.18 from the far face — the verified side-on geometry `[V]` |
| Chassis ↔ lane frame end | 200 mm (≥ 150 required) |
| Signals | `PalletReady(row, sku, boxesLeft)`, `RequestEntry`, `Locked`, `Clear` `[V]`; new: `Release(row)` = return or keep-for-collection |

Sequence: `retrieve(row)` → crane puts the pallet on the row station → deck up → lane conveys to
the port → end stops up → `PalletReady` → robot AMR docks, `Locked` → one layer is taken
(`takeBox` per box, top layer only `[V]`) → `Clear` → boxes left: stops down, lane conveys back,
deck down, crane re-stores; no boxes left: stops down, forklift job.

### 8.2 Robot AMR at the port — reach

| Quantity | Verified envelope `[V]` | Needed here `[N]` |
|---|---|---|
| Grip height (box top) | 0.594 … 1.794 | 0.904 … 1.804 |
| Carry swing TCP height = top + h + 0.08 | ≤ 2.124 | S1 2.084, S2 2.034, S3 2.034, S4 2.034, S5 2.034 |
| Radius from the J1 axis | 0.344 … 1.190 | unchanged (same dock geometry, same deck) |
| Elbow margin `|sin q3|` | ≥ 0.508 | to be measured |

The upper end is 10 mm above the verified grip range and swings may be raised by up to 120 mm
`[V]`, which can exceed 2.124. **The 2504-pose reach study of the mobile depalletizer must be
re-run for a 0.610 support plane before anything else is built on it.** Fallback if it fails:
S1 source pallets with 3 layers instead of 4.

### 8.3 Palletizer box infeed line (cell frame, z in cell coordinates)

| Section | Position (cell) | Height | Function |
|---|---|---|---|
| Receiving roller conveyor | x = 4.05 … 6.05, centreline y = −4.30, clear width 0.86, flow −X | 0.550 (= floor + 0.650, the deck height) | takes the layer from the robot AMR as one block; `DestReady` = 1.45 m free `[V]` |
| Row stop | x = 4.05 | | releases one row (the boxes side by side across the width) at a time |
| Incline belt | x = 2.05 … 4.05, flow −X | 0.550 → 0.750 (5.7°) | lifts to belt height |
| Right-angle transfer | at (1.75, −4.30) | 0.750 | a row arriving side by side leaves in +Y as a single file |
| Meter belt | y = −4.60 … −3.80 on x = 1.75 | 0.750 | releases one box every ≥ 8 s |
| Tracking belt | x = 1.75, 0.70 wide, y = −3.80 … +0.90, 0.25 m/s, never stops `[V]` | 0.750 | the robot picks in the window y = −0.8 … +0.6 `[V]` |

- Robot AMR dock in the cell frame: reference (6.83, −4.44), heading 180°, front edge at 6.08.
- Box yaw on the tracking belt = deck yaw + 90°: S1 arrives long side along the belt; S2 … S5
  long side across (up to 0.60 across the 0.70 belt). The pad is turned to the long side of the
  box at the pick; J6 supplies the difference to the place yaw and uses the 180° pad symmetry
  to take the shorter turn `[V]`.
- The meter belt lets a box go only when at most one box is still on the tracking belt, so the
  robot is never offered a box it cannot reach in time.
- A row is released when the transfer is free; it waits before the transfer while boxes of the
  row before are still queuing at the meter belt.
- Layers arrive in stacking order and the line is first-in-first-out, so the robot always knows
  which box is next and where it goes.
- Capacity ahead of the robot: one layer on the receiving conveyor + the boxes in transit.

### 8.4 Palletizer pallet conveyor and position D

| Parameter | Value |
|---|---|
| Conveyor | 2-strand chain, strands at cell y = 1.55 ± 0.42, chain top = cell z 0, bidirectional, 0.20 m/s |
| Positions | B (0, 1.55) build; D (−2.40, 1.55) dock. Transfer 2.40 m ≈ 14 s |
| Frame end at D | 30 mm beyond the pallet face `[V geometry]`; **no fixed end stop** (a forklift deposits here: a carried pallet passes only 60 mm above the support) — position by conveyor encoder ±10 mm, lateral offset measured by the forklift's pocket camera and taken up by the side shift `[V]` |
| Guarding | fence at cell x = −1.20 with a muted light curtain for the pallet; robot J1 limited to −30 … +110° (D is at 147° from the robot, 2.86 m away) |
| Interlock | conveyor moves only with the robot at its ready pose; a forklift enters D's zone only with `Locked`, and `Locked` is given only while the conveyor stands |

Cycle: forklift deposits an empty pallet at D → D → B → order is built → B → D → `PalletReady`
→ forklift picks → next empty.

### 8.5 Wrapper hand-over zones

A forklift docks **from the side** (heading south), so the pallet it carries crosswise on its
forks lands lengthwise on the line — the orientation the wrapper needs `[V]`. The tines stay
inside the pallet's fork pockets (25 mm above the pocket floor, i.e. 47 mm above the rollers), so
they never go below the roller tops and need no slots.

| Zone | Use | Signals |
|---|---|---|
| I4 (−7.00 wrapper frame) | deposit; then conveyed I4 → I3 through the centring funnel | `Free` (zone empty and stopped), `Locked`, `Clear` |
| O4 (+7.00) | pick; pallet stopped against a pop-up stop at the block positions | `PalletReady`, `Locked`, `Clear` |

Forklift docking tolerance ±5 mm, ±0.25° `[V]` against the wrapper's ±15 mm centring window `[V]`.

### 8.6 Storage, empty stacks, receiving

| Area | Rule |
|---|---|
| Storage S01 … S12 | single-deep floor slots, pitch 1.50 (300 mm between pallets `[V]`); filled lowest free number first; a pallet is shipped (removed) 600 s after deposit `[assumption A2]` |
| Empty stacks E1 … E3 | ≤ 6 pallets each `[V]`; deposits go to the fullest stack that is not full, picks come from the fullest stack. Start: 6 + 6 + 0, plus one empty pallet at every position B. Source pallets empty faster than mixed pallets use them up, so with more than 12 empties in the station a full stack is collected after 30 s `[assumption A13]` |
| Parking P1 … P6 | pre-dock points only, 3.50 apart (a parked forklift must be clear of its neighbour backing out) |
| Receiving R1 … R3 | a full single-SKU pallet appears when a row falls below 50 % fill `[assumption A1]`; a forklift takes it to that row's port when the lane is free |

---

## 9. End effectors and load holding

### 9.1 Overview

| System | "Gripper" | Holds by | Actuated grip DOF | Specification |
|---|---|---|---|---|
| Stacker crane | 3-stage telescopic twin fork | form fit + gravity | none (fork stroke is the 3rd crane axis) | ASRS §6 `[V]` |
| Forklift | two fixed tines, backrest, tilt, side shift | form fit in the fork pockets + gravity + tilt-back | none | AMR §7 `[V]` |
| Robot AMR arm | vacuum pad 0.26 × 0.16, 6 × Ø50, 707 N | suction, tool vertical | 1 binary | Mobman §9 `[V]` |
| Palletizing robot | vacuum pad 0.26 × 0.16, 6 × Ø60, 1018 N | suction, tool vertical | 1 binary | §9.2 `[N]` |
| Belly wrapper | film clamp + cut-and-press slide | grips the **film**, not the load | 3 | Wrapper §7 `[V]` |
| Conveyors, deck, turntable | — | gravity + friction, stops and guides | — | — |

### 9.2 Palletizing robot gripper `[N]`

| Parameter | Value |
|---|---|
| Plate | 0.26 × 0.16 — 20 mm inside the smallest box top (0.30 × 0.20) on every side |
| Cups | 6 × Ø60 in 2 × 3, pitch 0.08 × 0.08 (rows at ±0.04 across, columns at 0, ±0.08 along) |
| Vacuum | 60 kPa → 6 × 60 kPa × π·(0.03)² = **1018 N** (same as the verified gripper) |
| TCP | (0, 0, 0.300) in the flange frame; mass 14 kg (was 22) |
| Dwell | grip 0.20 s, release 0.15 s `[V]` |

Suction, `F = m·(g + a_z + a_h/μ)` with μ = 0.5 and the verified worst case a_h = 9.2, a_z = 3.5 m/s²:

| SKU | Mass | F required | Suction SF (≥ 2) | Peel moment `m·a_h·h/2 + m·(g + a_z)·0.04` | Peel SF at 100 % (≥ 1.5) | Dynamics | Peel SF used |
|---|---|---|---|---|---|---|---|
| S4 | 5 | 159 N | 6.4 | 6.1 N·m | 6.7 | 100 % | 6.7 |
| S3 | 8 | 254 N | 4.0 | 11.6 N·m | 3.5 | 100 % | 3.5 |
| S1 | 12 | 381 N | 2.7 | 20.2 N·m | 2.0 | 100 % | 2.0 |
| S2 | 15 | 476 N | 2.1 | 28.7 N·m | 1.42 | **85 %** | 1.59 |
| S5 | 14 | 444 N | 2.3 | 33.2 N·m | 1.23 | **70 %** | 1.60 |

Peel is the box prising itself off the pad when it is accelerated sideways: the box's centre of
gravity hangs h/2 below the cups. Worst direction is across the short side of the pad, where only
one cup row (3 cups, 509 N at a 0.08 m lever = 40.7 N·m) resists. The verified cell never needed
this check because its plate was as large as its one carton; a compact pad for mixed boxes does.

The same check on the robot AMR's pad (707 N, verified peak TCP acceleration 4.89 m/s²) gives
SF 1.47 for S5 → **S5 is moved by the mobile arm at 90 % dynamics** (SF 1.6). Both monitors get
the rule: *box held ⇒ peel SF ≥ 1.5*.

### 9.3 Motion constraints the grippers impose (unchanged, collected)

| System | Constraint |
|---|---|
| Both vacuum grippers | tool axis vertical while the vacuum is on; approach and retreat along the tool axis; no lift before the grip dwell, no retreat before the release dwell; top layer only |
| Palletizing robot | place approach stroke h + 0.10; PTP moves start and end ≥ h + 0.10 above the layer being built |
| Robot AMR arm | moves only with the base docked, at rest and `Locked`; base moves only with the arm in the travel pose and the vacuum off (M4) |
| Crane fork | enters and leaves only along Y at standstill; hoist creeps while the fork is out; travel only with the fork centred (R2–R5) |
| Forklift | enters and leaves a pallet only along the fork axis, steer 0, creep speed, tilt 0; mast axes move only at standstill; travel only in the travel pose (R4–R6) |
| Wrapper | cycle ends at the home angle with the carriage down; turntable never reverses; carriage moves only at wrap speed (R2, R5–R7) |

### 9.4 Pallet orientation through the system

| Where | 1200 side runs along | Presented face | Taken by |
|---|---|---|---|
| ASRS rack and lane | `{G}` Y | 1200 face, towards −X | forklift (tines into the 1200 side `[V]`), robot AMR beside it |
| On the forklift | across the forks | — | — |
| Palletizer B and D | `{G}` X (cell Y) | 1200 face, towards +Y (north) | forklift heading south |
| Wrapper line | `{G}` X (travel direction) | 1200 face, towards +Y | forklift heading south |
| Floor slots, stacks | `{G}` X | 1200 face, towards −Y | forklift heading north |

No machine in the warehouse can rotate a pallet about Z except the wrapper turntable, which
always returns to its home angle. The orientation chain above is closed without any turn.

### 9.5 Mixed pallet on the wrapper turntable

| Check | Result |
|---|---|
| Box sliding, `μ·g / a` | SF 3.3 `[V]` — independent of the SKU |
| Tipping of one box about its own edge (worst: S5, 0.40 × 0.60 base, 0.40 high) | `g·(0.20 / 0.20) / 0.90` = SF 10.9 |
| Tipping of a full-height column of the smallest footprint (S4, 0.20 base, 1.056 high) | `g·(0.10 / 0.528) / 0.90` = SF 2.06 (≥ 1.5) — and a mixed stack has no continuous columns |
| Film pulling the top layer off, `μ·m_layer·g / F` | lightest top layer S5, 56 kg: SF 2.75 |

---

## 10. Control architecture

```
                 ┌──────────────────────────── UI: order panel, fleet sliders, HUD ───┐
                 ▼                                                                    │
   ┌───────────────────────────┐   jobs    ┌────────────────────────┐  grants  ┌──────┴───────┐
   │ WES  orders, cells, stock │──────────▶│ Fleet manager          │◀────────▶│ Traffic      │
   │      sequencing           │◀──────────│ dispatch, job states   │          │ blocks       │
   └───┬───────┬───────┬───────┘  events   └───────┬────────┬───────┘          └──────────────┘
       │       │       │                           │        │
   ┌───▼──┐ ┌──▼───┐ ┌─▼──────┐              ┌─────▼──┐ ┌───▼────────┐
   │ ASRS │ │ PAL  │ │ WRAP   │              │ Fork-  │ │ Robot AMR  │     one module per machine,
   │ (V)  │ │ ×4(V)│ │ (V)    │              │ lift(V)│ │ (V)        │     each with its own monitor
   └──────┘ └──────┘ └────────┘              └────────┘ └────────────┘
                     station handshakes (PalletReady / Locked / Clear / DestReady) between machines
```

### 10.1 Layers

| Layer | Knows | Does not know |
|---|---|---|
| Machine modules `(V)` | their own kinematics, sequence, safety monitor | anything outside their cell frame |
| Station handshakes | two neighbours' signals | orders |
| Traffic | blocks and who holds them | why a vehicle is going anywhere |
| Fleet manager | jobs, vehicle states, the roadmap | box contents, robot poses |
| WES | orders, stock per row, cell states | joint angles, paths |

A machine module never calls another machine module; they meet only through handshake signals
and through the WES / fleet manager. This is what keeps every verified module verifiable on its own.

### 10.2 WES

- Order → cell: first idle cell with an empty pallet at B.
- Layer requests: issued in stacking order, at most **2 layers ahead** per cell (one on the
  receiving conveyor / belt, one on a robot AMR).
- Row arbitration: one request per row at a time; oldest order first.
- Stock: boxes per row from the ASRS inventory; a row below 40 % fill raises a replenishment job;
  an order that needs a SKU with no stock is held.
- Order generator: seed 7; each order 2 … 5 layers drawn from the SKUs in stock, height ≤ 1.056;
  a new order whenever fewer than 4 are open. Manual orders from the UI go to the head of the queue.

### 10.3 Fleet manager

| Job | Vehicle | From → to | Priority |
|---|---|---|---|
| Wrapped pallet out | forklift | O4 → S | 1 (the wrapper line blocks first) |
| Mixed pallet | forklift | D_i → I4 | 2 |
| Empty pallet to a cell | forklift | E → D_i | 3 |
| Empty source pallet | forklift | port r → E | 4 |
| Replenishment | forklift | R → port r | 5 |
| Layer | robot AMR | port r → cell i | by order age |
| Park | either | → P / Q | lowest |

A job is created only when its source is ready **and** its destination is free and can be
reserved `[V rule]`. Dispatch: the idle vehicle with the shortest path along the directed roadmap.
A vehicle with no job parks.

### 10.4 Interfaces added to the verified modules

| Module | Call / event | Meaning |
|---|---|---|
| ASRS `Warehouse` | `retrieve(row, sku)`, `returnPallet(row)`, `store(row)`, `portState(row)` | §4.1; `outputs()`, `takeBox()`, `snapshot()` stay `[V]` |
| Robot AMR | `assign({ row, cell, sku, boxes })`, events `docked`, `loaded`, `cleared`, `delivered` | the route comes from the fleet manager; the arm cycle is unchanged |
| Palletizer | `startOrder({ layers })`, `boxArriving(box)`, events `layerDone`, `palletDone` | the box source is the infeed line |
| Wrapper `Cell` | `i4Free()`, `depositAt('I4', pallet)`, `o4Ready()`, `takeFrom('O4')` | replaces the arrival timer and the vanishing outfeed |
| Forklift | `assign({ pick, drop })`, events `picked`, `dropped` | replaces its own job selection |
| All | `step(dt)`, `monitor()`, `snapshot()` | one clock, one monitor per instance, one JSON state for the view and the Blender export |

---

## 11. Singularities and failure modes

### 11.1 Per machine (all `[V]`; listed so that nothing is missed in the integration)

| Machine | Singularity | Condition | Rule |
|---|---|---|---|
| Palletizing robot | wrist | q5 = 0 (J4 ∥ J6 collinear) | \|q5\| ≥ 15°; tool vertical keeps q5 = 90° − (q2 + q3) |
| | elbow, stretched / folded | q3 = −88.04° / +91.96° | J3 ≥ −80°; Cartesian moves keep q3 ≤ +85° |
| | shoulder | wrist centre on the J1 axis | ρ ≥ 0.30 m |
| | posture branches | 8; only front / elbow-up / no-flip is used | a branch change means crossing a singularity: never |
| Robot AMR arm | elbow | sin q3 = 0 | \|sin q3\| ≥ 0.25 |
| | wrist | sin q5 = 0 (J4 ∥ J6) | tool vertical ⇒ q5 = 90° always |
| | shoulder | wrist point on the cylinder r = 0.201 around J1 (a hole in the workspace) | \|p_r\| ≥ 0.10 m |
| | posture branches | 8; only shoulder − / elbow + / wrist + | — |
| Robot AMR base | non-holonomic | no sideways motion | docks approached straight; the arm absorbs the lateral error |
| | curvature step | κ not continuous | quintic curvature ramps |
| Forklift | non-holonomic | no sideways motion | side shift ±100 mm is the lateral DOF |
| | standstill | v_s = 0: steer angle undefined from velocities | steer from path curvature; stops on straight pieces |
| | pivot | steer = ±90° | not used (≤ 40.9°) |
| | mast: tilt ∥ lift | lift = 0.35 | reach is set by the drive, never by tilt |
| Stacker crane | none | det J = −1 everywhere | failure modes are sequence errors: R1–R7 |
| Belly wrapper | helix degenerates | carriage moving, turntable not | carriage only at ≥ 90 % wrap speed |
| | moment arm vanishes | film line through the axis | centring ±30 mm |
| | clamp rendez-vous | only at home angle, carriage down | whole-revolution moves |

### 11.2 New with the integration `[N]`

| Hazard | Where | What goes wrong | Rule |
|---|---|---|---|
| Place reach near the stretched elbow | palletizing robot, S5 as the top layer at the far corner | hover TCP at cell z 1.70, wrist radius 2.01 → `|sin(q3 + β)|` ≈ **0.52** (the verified run never went below 0.64) | allowed (limit is not reached), but the monitor floor is set to 0.35 and the WES sorts S5 lower whenever layer masses permit; verified per order before it is released |
| Hover too low for tall boxes | palletizing robot | carried box cuts through its own layer | approach stroke h + 0.10 (§4.3) |
| Pad larger than the box | palletizing robot, S4 | seals on the neighbour, lifts two boxes | compact pad (§9.2) |
| Peel | both vacuum grippers, S2 / S5 | box prised off sideways | dynamics scaling + peel rule (§9.2) |
| Mobile arm reach, raised source | robot AMR at a port | top layer or carry swing out of the verified envelope | re-run the reach study first (§8.2) |
| Spin turn with a load | robot AMR | casters flip, layer shifts | spin only with an empty deck |
| Robot swings while a forklift is at D | palletizer | collision | J1 software range + fence + `Locked` interlock (§8.4) |
| Fixed end stop where a forklift deposits | D, I4, ASRS port | carried pallet (60 mm clearance) hits an 85 mm stop | no fixed stops at deposit positions; retractable at the ports |
| Forklift mast against a conveyor frame, side docking | wrapper I4 / O4 | 60 mm short | narrow hand-over zones (§4.4) |
| Two vehicles in one place | everywhere | collision | blocks (§7) + fleet monitor |
| Forklift leaves a spur in reverse | ring | backs into a following vehicle | the junction stretch is part of the spur's block |
| Layer arrives out of stacking order | palletizer infeed | wrong SKU under the gripper | WES issues layers in order; the line is FIFO; each box id is checked at the meter belt |
| Row lane used twice | ASRS | returning pallet meets an outgoing one | one request per row |
| Pallet lost or duplicated | everywhere | — | conservation checks on every pallet id and box id (§15) |

---

## 12. Throughput (measured, §15)

| Fleet | Mixed pallets / h | Layers / h | Robot AMR trip | Forklift job |
|---|---|---|---|---|
| 1 forklift + 1 robot AMR | 2.5 | 11.0 | 326 s | 134 s |
| 3 forklifts + 2 robot AMRs | 5.7 | 20.2 | 334 s | 172 s |
| 6 forklifts + 2 robot AMRs | 5.0 | 20.5 | 338 s | 181 s |

The robot AMRs are the bottleneck: one layer trip takes about 5.5 minutes (loading 4 … 16 boxes
at about 10 s each, about 60 m loaded and 90 m empty, four creep moves, the transfer and the
turns), so two of them deliver about 20 layers per hour whatever else is added. Doubling the
forklifts from 3 to 6 does not raise the output (the difference between 5.7 and 5.0 pallets / h
is the order mix of a 2 h against a 4 h run). The four palletizing robots (8 s per box), the
wrapper (about 40 pallets / h on its own) and the cranes (about 20 movements / h used of 117)
have several times the capacity that is asked of them.

| Resource | Capacity on its own | Source |
|---|---|---|
| Crane, single command | ≈ 117 pallet movements / h per aisle | ASRS §11 `[V]` |
| Palletizing robot | 8 s per box | `[V]` |
| Forklift | ≈ 55 pallets / h in its own cell | `[V]` |
| Wrapper | ≈ 40 pallets / h | `[V]` |

---

## 13. Software structure (ADR-001, ADR-002)

**ADR-001 — Integration by composition. Status: proposed.**

| Option | Assessment |
|---|---|
| **A. Compose the five verified modules** as instances in their own cell frames, add WES, fleet manager, traffic, infeed line (chosen) | every verified number stays verified; each cell still runs and verifies alone; integration effort is in the interfaces of §10.4 |
| B. One new monolithic warehouse model | simplest data flow, but all 5 verifications are thrown away and must be rebuilt |
| C. Five browser pages synchronised by messages | no code changes, but no common clock, no shared scene, no traffic |

Consequences of A: the modules must be made instantiable (no module-level state, constructor
takes a config and a seeded random source), and the mobile depalletizer and the forklift must
accept a route instead of owning one. Their own `verify:*` suites must still pass afterwards.

**ADR-002 — Traffic by block reservation with whole-run grants. Status: proposed.**

| Option | Assessment |
|---|---|
| **A. Blocks, a run starts only when fully granted** (chosen) | every run is the verified rest-to-rest S-curve; provably no rear-end or crossing conflict; vehicles sometimes wait where a smarter controller would follow |
| B. Time-window planning (space-time reservation) | smoother, higher throughput; needs trajectories that can be re-timed mid-run, which the verified motion law does not provide |
| C. Local sensing, brake for obstacles | realistic, but needs an online jerk-limited speed controller and gives deadlocks at junctions without extra rules |

Revisit B if the fleet grows beyond about 8 vehicles.

```
warehouse/
  WAREHOUSE_SPEC.md        this file
  js/config.js             every number of this file
  js/layout.js             cell placements, stations, transforms
  js/roadmap.js            nodes, edges, blocks (built from amr/js/path.js primitives)
  js/traffic.js            block reservation, fleet monitor
  js/fleet.js              jobs, dispatch, vehicle state machines
  js/wes.js                orders, layer sequencing, stock, order generator
  js/infeed.js             receiving conveyor → single file (§8.3)
  js/sim.js                one step(dt) for everything; snapshot()
  js/main.js               three.js view, HUD, order panel, fleet sliders
  test/verify.mjs          npm run verify:warehouse (§15)
  tools/export-blender.mjs trajectory export
  blender/build_warehouse.py   rebuilds the scene in Blender from the export
  index.html
```

Reused unchanged: `asrs/js/motion.js` (S-curve), `amr/js/path.js` (turn), all kinematics modules.

---

## 14. Assumptions to confirm

| # | Assumption | Where |
|---|---|---|
| A1 | A **receiving** area and replenishment by forklift exist (not in the brief). Without it the ASRS simply runs down; it can be switched off | §2, §8.6 |
| A2 | Storage is emptied by a 600 s shipping timer | §8.6 |
| A3 | One SKU per row; five SKUs equal to the mobile depalletizer's recipes R1–R4, R6 | §3.1 |
| A4 | Row lanes are out-and-back (one pallet at a time) instead of a separate return conveyor loop | §4.1 |
| A5 | A robot AMR trip is exactly one full layer | §4.2 |
| A6 | Mixed pallet ≤ 1.20 m including the pallet (2–6 layers); the order generator makes 2–5 | §3.2 |
| A7 | The palletizing robot stands on a 0.10 m riser and the pallet conveyor is a low-profile chain conveyor with two positions; the empty pallet goes in and the full pallet comes out at the same end | §4.3, §8.4 |
| A8 | The infeed line (row stop, incline, right-angle transfer, meter belt) is modelled as positions and timing, not as roller mechanics | §8.3 |
| A9 | Forklifts hand over at the wrapper from the side, on narrow zones I4 / O4 | §4.4, §8.5 |
| A10 | Building 68 × 48 m; the layout of §6 | §6 |
| A11 | Battery, charging, people, faults and recovery are not modelled; sensors are ideal | — |
| A12 | Vehicles wait rather than follow (whole-run grants) | §7 |
| A13 | Surplus empty pallets are collected from the empty pallet station by a timer | §8.6 |

---

## 15. Verification results (`npm run verify:warehouse`)

Static checks, then three runs at 240 Hz, seed 7: 3 forklifts + 2 robot AMRs for 4 h,
1 + 1 for 2 h, 6 + 2 for 2 h. (The plan was 8 h per fleet; 8 h in total were run.)

| Check | Result |
|---|---|
| The five cell suites after the hooks of §16 were added | all pass ✔ |
| Mobile arm at a port, pallet support 0.610: 5098 probe poses (every box, every layer, ± scatter, grip / carry / empty-swing height, with the largest swing raise) | 0 unreachable; TCP height 0.800 … 2.204 m; `|sin q3|` ≥ 0.320 (limit 0.25), shoulder ≥ 0.279 m ✔ |
| Palletizing robot: 224 probe poses (every SKU as top layer of the tallest allowed stack, at landing and hover height; every box on the belt) inside J1 −30 … +110° | 0 unreachable; wrist ≥ 0.510, elbow ≥ 0.366, shoulder ≥ 1.105 m ✔ |
| Box mass, source pallet height, pad inside the smallest box top | ≤ 15 kg; ≤ 1.194 m; 20 mm ✔ |
| Palletizer suction SF at the worst-case acceleration (S1 … S5) | 2.67 · 2.14 · 4.01 · 6.42 · 2.29 ✔ |
| Palletizer peel SF with the per-SKU dynamics (S1 … S5) | 2.02 · 1.59 · 3.50 · 6.66 · 1.60 ✔ |
| Forklift ring closes; 35 spurs and 15 routes end on their pre-dock pose; docked TCP = pallet centre at every station | 3·10⁻¹³ m; ≤ 3·10⁻¹¹ m; 3·10⁻¹² m ✔ |
| Robot-AMR routes end on the cell pre-dock, the dock and the head of the return lane | 6·10⁻¹¹ m ✔ |
| Curvature continuous along every road; every permitted stopping point on a straight piece | ✔ |
| Station coordinates of §6.2 computed from the cell frames | ✔ |
| No standing position inside the ring sweep or inside the entry sweep of another station | ✔ |
| Lane spacings; chassis ↔ lane frame end at a port | 2.00 / 2.00 / 2.25 m; 200 mm ✔ |

| Check in the runs | 3 + 2, 4 h | 1 + 1, 2 h | 6 + 2, 2 h |
|---|---|---|---|
| Planning, IK or sequencing errors | 0 ✔ | 0 ✔ | 0 ✔ |
| Monitor violations: cranes / wrapper / 4 palletizing cells / robot AMRs / forklifts | 0 / 0 / 0 / 0 / 0 ✔ | 0 ✔ | 0 ✔ |
| Smallest gap between two vehicle outlines (≥ 0.30) | 0.500 m ✔ | 0.50 m ✔ | 0.500 m ✔ |
| Raw checks: conveyor moved under a locked station, robot moved with a forklift at D, forklift in a zone without `Locked`, base / arm moving together, port zone without `Locked`, mast moving while driving | 0 steps ✔ | 0 ✔ | 0 ✔ |
| Boxes missed on a tracking belt / IK failures | 0 / 0 ✔ | 0 / 0 ✔ | 0 / 0 ✔ |
| Boxes conserved (every hour) | 10 878 = 9 929 + 273 + 676 ✔ | 9 445 = 9 144 + 168 + 133 ✔ | 9 996 = 9 513 + 218 + 265 ✔ |
| Pallets conserved | 376 = 336 + 22 + 18 ✔ | 344 = 334 + 4 + 6 ✔ | 359 = 339 + 8 + 12 ✔ |
| Every built pallet matches its order layer by layer, in the sorted sequence, ≤ 1.200 m | 23 pallets ✔ | 5 ✔ | 10 ✔ |
| Every shipped pallet was wrapped | 22 ✔ | 4 ✔ | 8 ✔ |
| Longest gap between two shipped pallets / longest traffic wait | 38 min / 230 s ✔ | 50 min / 42 s ✔ | 37 min / 134 s ✔ |

Margins and peaks (worst of the three runs):

| Machine | Result |
|---|---|
| Palletizing robots | joint speed / limit 1.00 · 0.49 · 0.57 · 0.00 · 0.50 · 0.99; wrist ≥ 0.510, elbow ≥ 0.615, shoulder ≥ 1.105 m; vacuum SF ≥ 2.94; peel SF ≥ 1.96; carried box ≥ 100 mm above belt and stack; place error 1·10⁻¹⁵ m; J1 −25 … +105° |
| Robot AMRs | elbow ≥ 0.508, shoulder ≥ 0.279 m, wrist 1.000; vacuum SF ≥ 2.61; peel SF ≥ 2.00; arm clearance ≥ 50 mm, carried box ≥ 68 mm, largest swing raise 40 mm; 1.500 m/s empty, 1.000 loaded, 0.200 in a zone; yaw rate 60.0°/s; CG ≥ 0.287 m inside the tipping edge; dead reckoning < 0.5 mm over 11.5 km |
| Forklifts | 1.600 m/s empty, 1.200 loaded, 0.998 reverse, 0.300 in a fork zone; steer ≤ 40.9°, 59.9°/s; lateral 0.815 m/s²; lift 0.092 … 0.902; plan clearance ≥ 60 mm (mast ↔ conveyor frame, docked); dead reckoning < 0.5 mm over 21.2 km |

The verification found these real faults while the model was built; each is now a rule:

- the carried box left the belt end only 30 mm above it on the low layers → hover height rule of §4.3;
- returning the source pallet after every layer made the row lane the bottleneck → the pallet stays while its SKU is on order (§4.1);
- a forklift waiting at palletizer 4 stood in the tail swing of the ring corner, and forklifts at opposite spurs in each other's tail swing → longer run-in, cell pitch 8.20, static layout check (§6.3);
- a forklift stopped on the ring where another had to back out, each waiting for the other → no stopping in a way out, neighbouring stations (§7);
- the next pallet rolled onto O4 while a forklift was still in it → nothing moves onto O4 while it is locked;
- the empty pallet station filled up and blocked the ASRS ports → surplus stacks are collected (§8.6);
- with one forklift, a replenishment pallet was picked up for a lane that only another forklift job could clear → the lane is reserved before the pallet is picked, the apron after;
- the box was gripped one control step (1 mm) off centre → grip after the position update.

What this verification does not cover: everything listed as not modelled in the five cell
specifications, plus A11 (no battery, people, faults or recovery), ideal docking (no localisation
noise is injected), the infeed line as timing only (A8), and collision checking limited to the
monitors' own models (plan outlines for vehicles, capsules for the mobile arm, the carried box
for the palletizing robots) — there is no mesh-level check between machines.

---

## 16. Files and how to run

| What | Where | Command |
|---|---|---|
| Parameters (this document as code) | [js/config.js](js/config.js) | — |
| Roads: ring, spurs, robot-AMR loop | [js/roadmap.js](js/roadmap.js) | — |
| Traffic: claims, runs, permitted stops | [js/traffic.js](js/traffic.js) | — |
| ASRS with row lanes and ports (uses the verified crane) | [js/asrs.js](js/asrs.js) | — |
| Robot AMR (extends the verified mobile depalletizer) | [js/mobile.js](js/mobile.js) | — |
| Palletizing cell: infeed line, robot, pallet conveyor, cell monitor | [js/palletizer.js](js/palletizer.js) | — |
| Forklift (verified kinematics, mast sequence and monitor) | [js/forklift.js](js/forklift.js) | — |
| Floor stations and the wrapper adapter (extends the verified wrapper) | [js/stations.js](js/stations.js) | — |
| Orders, layer sequencing, forklift jobs | [js/wes.js](js/wes.js) | — |
| The world: one clock, fleet monitor, raw checks, bookkeeping | [js/sim.js](js/sim.js) | — |
| three.js view | [index.html](index.html), [js/main.js](js/main.js) | `node serve.mjs 5178` in the repository root, then open `http://localhost:5178/warehouse/index.html` |
| Verification | [test/verify.mjs](test/verify.mjs), [test/layoutcheck.mjs](test/layoutcheck.mjs) | `npm run verify:warehouse` (add `-- --quick` for one 1 h run, `-- --cells` to re-run the five cell suites first) |
| Trajectory export for Blender (240 s at 12 fps, after a 1500 s warm-up) | [tools/export-blender.mjs](tools/export-blender.mjs) → `blender/warehouse_params.json` | `npm run export:warehouse` |
| Blender scene builder (one empty per joint frame, keyframed) | [blender/build_warehouse.py](blender/build_warehouse.py) | run in Blender's Scripting tab, or `blender -b -P build_warehouse.py -- --save warehouse.blend` |
| Built Blender scene | `blender/warehouse.blend` | open in Blender 5.2 and press play |

Changes to the five cells, all behaviour-neutral for the cells themselves (their suites pass
unchanged): `asrs/js/sim.js` exports its `Aisle` class; `wrapper/js/sim.js` has two hooks
(`exitFree()`, `leave()`); `mobman/js/sim.js` has two hooks (`armFrames()`, `fixedFrames()`).

In the browser, `window.wh.world` is the running `World`; `window.wh.step(n)` advances it by `n`
steps of 1/240 s. The view shows the traffic claims with the *Claims* button, and the joints,
singularity margins and monitor of any machine under *Inspect*. The Blender scene draws a pallet
in its rack location as one block per load; rollers are static.
