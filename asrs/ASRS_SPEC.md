# ASRS Specification — Stacker Cranes, Load-Handling Device, Racks, Unit Loads and Interfaces

This file is the contract for the simulation. Every number here is mirrored in
[js/config.js](js/config.js). [js/kinematics.js](js/kinematics.js) implements §5–§9,
[js/motion.js](js/motion.js) implements §4.2, [js/sim.js](js/sim.js) implements §8 and §13, and
[test/verify.mjs](test/verify.mjs) checks all of it (`npm run verify:asrs`, results in §11).

The system is a **single-deep, unit-load (pallet) AS/RS with five rack rows A–E served by three
aisles**, each aisle with its own rail-guided twin-mast stacker crane with a telescopic fork.
Every pallet carries individually tracked **boxes**. Each aisle has an inbound and an outbound
conveyor lane; the outbound lanes end in **output ports** where a neighbouring cell (AMR,
palletizer / depalletizer) takes boxes off the presented pallet (§13).

It is not a model of any vendor's machine; dimensions, speeds and clearances are representative
values for a 1000 kg, 13 m class pallet crane.

---

## 1. Conventions

| Item | Convention |
|---|---|
| Units | metres, seconds, kilograms |
| World frame `{W}` | right-handed, **Z up**. **+X** runs down the aisles away from the stations, **+Y** points from aisle 1 into row A, origin on the floor on the centreline of aisle 1 at travel position 0 |
| Aisle frame `{A_n}` | `{W}` shifted in Y to the centreline of aisle n: `y_world = y_aisle,n + y_local`. Crane joints and all kinematics are expressed in the aisle frame, so the three cranes are identical |
| Side | `+1` = local +Y, `−1` = local −Y |
| Joint vector | `q = [q1, q2, q3]`, all prismatic, per crane |
| Location id | `row-bay-level`, e.g. `C-07-3`; bay 1 is nearest the stations, level 1 is the lowest |
| TCP | centre of the load plane (top face) of the top fork stage |
| Industry naming | crane vendors and FEM 9.x call the axes X = travel, **Y = hoist, Z = fork**. Here the world is Z-up, so hoist is world Z and fork is world Y. The joints are named by function (q1/q2/q3) to keep this unambiguous |
| three.js mapping | everything is built in a group rotated −90° about three's X, so `{W}`(x, y, z) → three (x, z, −y). No other axis conversion exists anywhere |

---

## 2. Degrees of freedom (per crane)

| Joint | Name | Type | Axis | Positive direction | What it does |
|---|---|---|---|---|---|
| q1 | Travel | P | +X | away from the stations | moves the whole crane along the floor rail |
| q2 | Hoist | P | +Z | up | moves the lift carriage on the masts |
| q3 | Fork | P | +Y (aisle frame) | towards the `+1` side | extends the telescopic fork across the aisle into a location |

**3 DOF, all translational, mutually orthogonal (a Cartesian / PPP machine).** The whole
installation has 3 cranes × 3 = 9 controlled axes; the cranes share nothing mechanically and
cannot collide (each stays in its own aisle).

What that means for everything downstream:

- The TCP can be positioned in x, y, z. Its **orientation is fixed** — the crane cannot roll,
  pitch or yaw a load. A pallet that arrives skewed stays skewed, and **boxes keep the
  orientation they were stacked in from the inbound port to the output port** (§13).
- There is no redundancy and exactly **one** joint solution per TCP position (no posture
  branches, no elbow-up/elbow-down).
- The fork has one *internal* coupled motion: the middle stage moves at half the top-stage
  travel (`q3/2`). That is a mechanical 2:1 drive, not an extra DOF.
- Load transfer is done by the hoist (a ~85 mm creep stroke) — the fork itself has no lift axis.

---

## 3. Geometry

### 3.1 Kinematic chain (what the 3D scene graph implements, per crane)

| Frame | Parent | Offset in parent [m] | Moves along |
|---|---|---|---|
| Aisle | world | (0, y_aisle, 0) | fixed |
| Travel base | aisle | (q1, 0, 0) | X |
| Lift carriage | travel base | (0, 0, q2) | Z |
| Fork middle stage | lift carriage | (0, q3/2, −δ/8) | Y |
| Fork top stage (TCP) | lift carriage | (0, q3, −δ) | Y |

`δ` is the fork tip sag of §6.3. With `δ = 0` the TCP is `(q1, y_aisle + q3, q2)` in the world.

`q1` is the X position of the fork centreline; `q2` is the height of the fork load plane above
the floor (fork retracted, unloaded).

### 3.2 Crane structure

| Parameter | Value |
|---|---|
| Masts | 2, at x = q1 ± 0.85, section 0.30 × 0.40, height 13.1 |
| Clear width between masts | 1.40 (pallet 0.80 → 0.30 each side) |
| Travel chassis | 3.00 long × 0.90 wide × 0.28 high, centred on q1 |
| Lift carriage frame | 1.40 × 0.90, spans q2 − 0.19 … q2 − 0.07 (under the fork stack) |
| Rated payload | 1000 kg |

### 3.3 Layout and racks

| Aisle | Centreline y | `+1` side | `−1` side | Locations |
|---|---|---|---|---|
| 1 | 0.00 | **Row A** (pallet centres y = +1.40) | **Row B** (y = −1.40) | 256 |
| 2 | −4.30 | **Row C** (y = −2.90) | **Row D** (y = −5.70) | 256 |
| 3 | −8.60 | **Row E** (y = −7.20) | free — reserved for a future row F | 128 |

**5 rows × 16 bays × 8 levels = 640 locations.** Rows B/C and D/E stand back to back with a
0.20 gap; aisle pitch 4.30. Aisle 3 serves one row only, so its crane must never extend the fork
into a bay on the `−1` side (§9, R4).

| Parameter | Symbol | Value |
|---|---|---|
| Bay centre | `x_b` | 3.30 + (b − 1) · 1.10 → 3.30 … 19.80 |
| Support plane (pallet underside when stored) | `z_l` | 0.60 + (l − 1) · 1.50 → 0.60 … 11.10 |
| Storage centre (aisle frame) | `y` | ±1.40 |
| Rack front face (aisle frame) | | \|y\| = 0.75 → **clear aisle 1.50** |
| Rack depth | | 1.30 (0.75 … 2.05) |
| Upright width / clear bay opening | | 0.10 / 1.00 |
| Pallet supports | | two angle ledges per location running in Y, each 0.18 wide from the upright, vertical leg 0.12 down; **fork window between them 0.64** |
| Rack height | | 13.4 |

Pallets rest on the two side ledges only — there is no front beam under the pallet, so the fork
can pass beneath it.

### 3.4 Unit load: pallet + boxes

| Parameter | Value |
|---|---|
| Pallet | EUR 800 × 1200 × 144, 25 kg, **1200 side into the rack** (800 across the bay) |
| Load envelope | flush with the pallet: 0.80 × 1.20 |
| Max height incl. pallet | 1.20 |
| Max mass | 1000 kg |
| Max centre-of-gravity height above the load plane | 0.60 |

A pallet carries **one box type (SKU)**, stacked in full layers of `nx × ny` boxes, column
stacked (no interlocking), flush with the pallet edges. `sx` is along world X, `sy` along
world Y. Inbound pallets have 2 … max layers.

| SKU | Box sx × sy × sz | Mass | Per layer | Max layers | Max boxes | Max pallet height / mass |
|---|---|---|---|---|---|---|
| BX-4030 | 0.400 × 0.300 × 0.250 | 12 kg | 2 × 4 = 8 | 4 | 32 | 1.144 m / 409 kg |
| BX-4060 | 0.400 × 0.600 × 0.300 | 22 kg | 2 × 2 = 4 | 3 | 12 | 1.044 m / 289 kg |
| BX-2640 | 0.266 × 0.400 × 0.200 | 8 kg | 3 × 3 = 9 | 5 | 45 | 1.144 m / 385 kg |
| BX-8060 | 0.800 × 0.600 × 0.350 | 35 kg | 1 × 2 = 2 | 3 | 6 | 1.194 m / 235 kg |
| BX-2030 | 0.200 × 0.300 × 0.150 | 4 kg | 4 × 4 = 16 | 7 | 112 | 1.194 m / 473 kg |

Box position on its pallet (pallet origin = centre of the pallet underside), for column `i`,
row `j`, layer `k` (all from 0):

```
lx = (i − (nx − 1)/2) · sx
ly = (j − (ny − 1)/2) · sy
lz = 0.144 + (k + 0.5) · sz            (box centre)
box centre in the world = pallet position + (lx, ly, lz)        — no rotation, ever
```

Box id = `<pallet id>-<running number>`, unique in the installation. The pallet mass used for
fork-sag compensation is 25 kg + the sum of its boxes.

### 3.5 Stations and conveyor lanes (per aisle)

| Parameter | Value |
|---|---|
| Station column | x = 1.10, one IN and one OUT station per aisle |
| IN station | `+1` side (in line with rows A / C / E), support plane z = 0.60 |
| OUT station | `−1` side (in line with rows B / D / the free strip), support plane z = 0.60 |
| Station supports | two fixed ledges along Y at x = 1.10 ± 0.36 (80 mm wide) — same fork window as a rack location |
| Roller deck | between the ledges; **up**: roller tops 10 mm above the ledges (pallet can roll in X); **down**: 200 mm below them, clear of the fork. Travel time 1.0 s |
| Lanes | roller conveyors along X from the port (x = −6.10) to the station, pallet underside at z = 0.61 |
| Lane speed / accumulation pitch / capacity | 0.20 m/s / 1.00 m / 7 pallets |
| Inbound flow | +X, port → IN station |
| Outbound flow | −X, OUT station → port |

Station sequence. **IN**: deck up → pallet rolls onto the station → deck down (pallet now rests
on the ledges) → crane picks → fork centred → deck up for the next pallet. **OUT**: deck down →
crane puts → fork centred → deck up → pallet rolls out → deck down. The fork may only be inside
a station while its deck is fully down (§9, R7), and a deck only moves while the fork is centred.

---

## 4. Joint limits and motion law

### 4.1 Limits

| Joint | Min | Max | Max speed | Max accel | Max jerk | Positioning tolerance |
|---|---|---|---|---|---|---|
| q1 Travel | 0.000 | 19.800 | 3.0 m/s | 0.5 m/s² | 1.0 m/s³ | ±3 mm |
| q2 Hoist | 0.500 | 11.250 | 1.0 m/s | 0.6 m/s² | 1.5 m/s³ | ±2 mm |
| q2 Hoist, fork extended (creep) | | | 0.08 m/s | 0.3 m/s² | 1.5 m/s³ | |
| q3 Fork, empty | −1.400 | +1.400 | 1.0 m/s | 1.0 m/s² | 4.0 m/s³ | ±2 mm |
| q3 Fork, loaded | | | 0.6 m/s | 0.5 m/s² | 2.0 m/s³ | |

Other timing: mast-sway settle before the fork may extend **0.30 s**; fork-centred confirmation
before travel/hoist are released **0.10 s**; emergency travel deceleration 2.0 m/s².

Range actually needed: travel 1.100 … 19.800, hoist 0.563 … 11.155 (rated load), fork ±1.400.

### 4.2 Motion law

Every axis move is a **jerk-limited, rest-to-rest 7-segment S-curve**
(jerk +j, 0, −j, 0, −j, 0, +j). Acceleration is continuous, so the masts and the load are not
excited by acceleration steps. For a distance `D`:

```
t_j = a/j,  t_c = v/a − t_j           (if v·j < a²:  a_peak = √(v·j), t_j = a_peak/j, t_c = 0)
D_acc = v·(2·t_j + t_c)/2             (distance to reach v)
if 2·D_acc ≤ D:   cruise for (D − 2·D_acc)/v
else (v not reached):
    v_peak = a/2 · (−a/j + √(a²/j² + 4D/a))      if that is ≥ a²/j
    v_peak = ∛(D²·j/4)                           otherwise (a not reached either)
```

Travel and hoist run **simultaneously and independently**, each on its own time-optimal
profile; the move ends when the slower one arrives (Chebyshev travel). They are not
path-synchronised, so the TCP path in the aisle plane is not a straight line — and does not need
to be, because the aisle plane is free of obstacles while the fork is centred (§9).

Full-stroke times: travel 13.10 s, hoist 12.82 s, fork 1.4 m empty 2.65 s / loaded 3.78 s,
transfer lift (rated load) 1.62 s.

---

## 5. Forward and inverse kinematics (aisle frame)

```
Forward:   x = q1
           y = q3
           z = q2 − δ(q3, m)

Inverse:   q1 = x
           q3 = y
           q2 = z + δ(y, m)          → reject if any joint is outside §4.1
```

`m` is the mass on the fork. One solution, closed form, valid everywhere inside the limits.
World position = aisle-frame position + (0, y_aisle, 0).

**Jacobian** (rows x, y, z; columns q1, q2, q3):

```
        | 1   0    0         |
J  =    | 0   0    1         |          det J = −1   for every q and every load
        | 0   1   −∂δ/∂q3    |
```

---

## 6. Load-handling device ("gripper") — telescopic twin fork

The crane does not grip. The load is held by **form fit and gravity**: the fork goes under the
pallet and lifts it. Everything a gripper spec normally contains has an equivalent here. The
crane only ever handles whole pallets; individual boxes are handled by the neighbouring cell at
the output port (§13).

### 6.1 Parameters

| Parameter | Value |
|---|---|
| Type | 3-stage telescopic fork (fixed base, middle, top), two tines, double-sided (reaches both sides of the aisle) |
| Stroke | ±1.40 (aisle centre → storage centre) |
| Stage length | 1.25 (retracted: y = −0.625 … +0.625, 125 mm inside each rack face) |
| Tines | 2 × 0.18 wide at x = ±0.16 → overall width 0.50 |
| Stack thickness below the load plane | 0.070 |
| Middle-stage ratio | 0.5 (2:1 differential drive) |
| Stage overlap at full stroke | 0.55 = 44 % of the stage length |
| Actuation DOF | 1 (q3). No clamp, no vacuum, no lift |
| Entry clearance, picking (`c_low`) | fork load plane 40 mm **below** the support plane |
| Entry clearance, putting (`c_high`) | pallet underside 40 mm **above** the support plane |
| Friction fork / pallet | μ = 0.30 (steel on wood, conservative) |

Extended, the top stage covers y = 0.775 … 2.025 and the stored pallet occupies 0.800 … 2.000,
so the pallet is supported over its full depth and the fork tip stops 25 mm short of the rack
back (2.05).

### 6.2 Sensors the sequencer relies on

| Sensor | Used for |
|---|---|
| Fork centred (\|q3\| ≤ 2 mm) | releases travel, full-speed hoist and the station decks |
| Load present on fork | chooses empty/loaded fork limits and hoist targets |
| Load overhang left / right (light barriers at the rack faces) | load must be inside ±0.60 before travel |
| Location occupied (distance sensor along Y) | no put into an occupied location, no pick from an empty one |
| Travel and hoist in position | releases the fork |
| Station deck down (per station) | releases the fork into that station |

### 6.3 Fork deflection — the one coupling between axes

An extended telescopic fork is a cantilever and sags at the tip. This is the only place where
one joint's position changes another axis's result, so it is modelled explicitly:

```
δ(q3, m) = (δ_e + δ_r · m / 1000 kg) · (|q3| / 1.40)³
δ_e = 3 mm   (own weight, full stroke)
δ_r = 12 mm  (additional, rated load, full stroke)
```

The hoist command is raised by the expected sag so that the *deflected* fork is where it must
be (§8.2). During a transfer the load moves over gradually: with the fork under a resting
pallet, raising the hoist first takes up the sag (the pallet stays on its supports while the
fork picks up 0 → 100 % of the weight), and only then does the pallet leave the supports:

```
pallet underside  z_p = max( z_l ,  q2 − δ(q3, m) )
```

### 6.4 Holding the load

| Check | Formula | Result |
|---|---|---|
| Pallet sliding on the fork, normal operation | μ·g / max(a_travel, a_fork,loaded) = 2.94 / 0.5 | **SF 5.9** |
| Pallet sliding on the fork, emergency stop | μ·g / 2.0 | **SF 1.47** |
| Load tipping about the fork edge, emergency stop | g · 0.25 / 0.60 / 2.0 | **SF 2.04** |

Boxes are not restrained individually: the stack is assumed to be stable (stretch-wrapped or
interlayer friction ≥ 0.3), which the same sliding factors cover. Box-on-box sliding and stack
toppling are not simulated (§12).

Constraints the fork puts on motion and on the load:

- Enter and leave a location **only along Y**, with travel and hoist at standstill.
- With the fork extended the hoist may only creep, and only inside the cell window (§9, R5).
- No transfer without the settle time; no travel without the centred confirmation.
- The load cannot be re-oriented: pallet skew must already be inside the lateral budget of §10.
- Loaded fork speed and acceleration are reduced (§4.1).

---

## 7. Workspace (per crane)

The reachable set is a box, `[0, 19.8] × [−1.4, 1.4] × [0.5, 11.25]`, but the **usable** set is
much smaller and has the shape of a comb:

- the *spine* is the aisle plane `q3 = 0`, where travel and hoist may move freely;
- the *teeth* are short line segments in Y — one per location plus one per station — each
  attached to the spine at one exact `(q1, q2)` and only as tall in q2 as the transfer stroke.
  Aisles 1 and 2 have 258 teeth, aisle 3 has 130 (one rack row + two stations).

Everything else in the box is solid rack. Getting from one tooth to another always means
returning to the spine first.

| Target | q1 | q3 | q2 (pick entry … pick exit, rated load) |
|---|---|---|---|
| Location `r-b-l` | `x_b` | `side · 1.40` | `z_l − 0.037` … `z_l + 0.055` |
| IN station | 1.10 | +1.40 | 0.563 … 0.655 |
| OUT station | 1.10 | −1.40 | 0.563 … 0.655 |

---

## 8. Operating cycles

### 8.1 Pick (from a location or the IN station) / Put (to a location or the OUT station)

| # | Phase | Axes moving | Pick | Put |
|---|---|---|---|---|
| 1 | Travel | q1 + q2 together | to `x`, hoist at **low** | to `x`, hoist at **high** |
| 2 | Settle | none | 0.30 s | 0.30 s |
| 3 | Fork extend | q3 | to `side·1.40`, empty limits | to `side·1.40`, loaded limits |
| 4 | Transfer | q2 (creep) | **low → high**: fork takes the pallet | **high → low**: fork sets it down |
| 5 | Fork retract | q3 | to 0, loaded limits | to 0, empty limits |
| 6 | Centre check | none | 0.10 s | 0.10 s |

### 8.2 Hoist targets for a support plane `z_l` and load mass `m`

```
low  = z_l − c_low  + δ_e                         (fork passes under the resting pallet)
high = z_l + c_high + δ_e + δ_r · m / 1000        (carried pallet passes over the supports)
```

Transfer stroke = `c_low + c_high + δ_r·m/1000` = 80 … 92 mm (82 … 86 mm for the SKUs of §3.4).

### 8.3 Commands (per aisle)

- **Store**: pick at the IN station → put at an empty, unreserved location of that aisle.
- **Retrieve**: pick at a location → wait until the OUT station is free → put there.
- The scheduler alternates store and retrieve (dual-command operation) while keeping the aisle's
  fill between 30 % and 80 %; retrieval requests (by location or by SKU, §13) take priority.
- A location is reserved from the moment a command is issued until the transfer is complete.
- **Back-pressure**: a retrieve is only started if the outbound lane has room. If nobody takes
  pallets at the output port the lane fills to 7 and the crane stops retrieving; it keeps storing
  until the aisle is 80 % full, then idles.

---

## 9. Singularities and motion constraints

### 9.1 Kinematic singularities: none

`det J = −1` everywhere (§5). Three orthogonal prismatic axes cannot line up, so a crane never
loses a direction of motion and joint speeds never blow up. The sag term changes the Jacobian
only by a shear, which cannot change the determinant. There is no pose to avoid for kinematic
reasons.

### 9.2 What goes wrong instead

A Cartesian crane fails on **sequence and tolerance**, not on singularities. These are its
equivalents, and each one is enforced:

| Hazard | Why it matters | Rule |
|---|---|---|
| Fork not centred while the crane moves | the fork or the load sweeps through the rack at up to 3 m/s | **R2** travel speed must be 0 unless \|q3\| ≤ 2 mm. **R3** hoist speed ≤ 0.08 m/s unless \|q3\| ≤ 2 mm |
| Fork extends where there is no opening | hits an upright or a ledge — or, in aisle 3, reaches into the side that has no rack | **R4** fork beyond the rack face (\|q3\| > 0.125) only with q1 within ±3 mm of the station column or of a bay centre **on a side that has a rack** |
| Fork at the wrong height inside a cell | lifts the pallet into the level above or drives the fork into the supports | **R5** inside a cell the fork stack must stay between 0.14 below and 0.08 above the support plane |
| Load too tall for the cell | strikes the ledges of the level above during the high pass | **R6** carried load top ≥ 50 mm below the structure above |
| Fork enters a station while its roller deck is up | fork hits the rollers; pallet is not resting on the ledges | **R7** fork beyond the rack face at the station column only with that station's deck fully down |
| Joint over-travel | end-stop impact | **R1** soft limits of §4.1 |
| Side change | q3 goes from +1.40 to −1.40 through 0. This is not a singularity, but the fork must *stop* being a hazard on one side before it becomes one on the other | every transfer ends with the centre check |
| Mast sway | a 13 m mast is still oscillating at the top when travel stops | settle time before R4 is evaluated; jerk-limited profiles |
| Fork sag | 15 mm at rated load is 37 % of the 40 mm entry clearance | hoist targets compensated per §8.2 |
| Missing orientation DOF | a skewed pallet cannot be straightened | load contour check at the station; lateral budget §10 |

Rules R1–R7 are implemented as a **safety monitor that reads only joint positions, joint speeds,
the load sensors and the deck switches** (`monitor()` in [js/kinematics.js](js/kinematics.js)).
It does not know what the sequencer intends, so a sequencing bug cannot hide from it. One
monitor instance runs per crane.

---

## 10. Clearance budget

| Clearance | Nominal | Consumers |
|---|---|---|
| Fork ↔ support ledge (X) | 70 mm each side | travel positioning ±3, rack erection, mast sway |
| Pallet ↔ upright (X) | 100 mm each side | same + pallet skew and box overhang |
| Pallet bearing on each ledge (X) | 80 mm | same; must stay ≥ 50 |
| Carried load ↔ rack face while travelling (Y) | 150 mm each side | load overhang, fork centring ±2 |
| Retracted fork ↔ rack face (Y) | 125 mm | — |
| Fork tip ↔ rack back (Y) | 25 mm | fork positioning ±2 |
| Fork load plane ↔ resting pallet (Z, pick entry) | 40 mm | hoist ±2, level tolerance, sag error |
| Carried pallet ↔ supports (Z, put entry) | 40 mm | same |
| Max load ↔ level above during transfer (Z) | 125 mm | — |
| Fork underside ↔ max load in the level below (Z) | 187 mm | — |
| Pallets on a lane (X) | 200 mm | accumulation pitch 1.00 − pallet 0.80 |
| Neighbouring lanes B/C and D/E (Y) | 300 mm | — |

---

## 11. Verification results (`npm run verify:asrs`)

4 h of operation simulated at 240 Hz, seed 7, initial fill 55 %, all three aisles:

| Check | Result |
|---|---|
| S-curve end position, 1000 random profiles | worst error 1·10⁻¹⁴ m ✔ |
| S-curve speed / acceleration limits | never exceeded ✔ |
| IK(FK(q)) = q, 20 000 random poses and loads | worst error 2·10⁻¹⁵ ✔ |
| Analytic Jacobian vs finite differences | 1·10⁻⁹ ✔ |
| det J | −1 … −1 ✔ |
| All 258 handling positions of an aisle reachable (1032 poses) | ✔ |
| Static clearances of §10 and stage overlap | ✔ |
| Every SKU pattern fits footprint, height and mass limits | tallest 1.194 m, heaviest 473 kg ✔ |
| Boxes never overlap, stay inside the load envelope; pallet mass = Σ boxes | 8507 boxes on 346 pallets ✔ |
| Safety monitor negative tests (11 illegal states, 3 legal) | all classified correctly ✔ |
| Safety monitor violations during the run | 0 / 0 / 0 (aisle 1 / 2 / 3) ✔ |
| Travel or fast hoist with the fork extended (raw check) | 0 steps ✔ |
| Fork inside a station with its deck not down (raw check) | 0 steps ✔ |
| Peak v / a / j, every crane | travel 2.935 / 0.500 / 1.00, hoist 1.000 / 0.600 / 1.50, fork 1.000 / 1.000 / 4.00 — all within §4.1 ✔ |
| Loaded fork peak v / a | 0.600 / 0.500 of 0.6 / 0.5 ✔ |
| Carried pallet above the supports while inside a cell | min 40.0 mm ✔ |
| Carried load top ↔ level above | min 137 mm ✔ |
| Pallets on a lane never touch | min gap 200 mm ✔ |
| Pallets conserved, no double occupancy | ✔ |
| Boxes conserved | 29 015 created = 9 094 in the system + 19 921 handed over, every id exactly once ✔ |

| Aisle | Stored | Retrieved | Locations visited | Single command | Dual cycle | Movements / h |
|---|---|---|---|---|---|---|
| 1 (A + B) | 233 | 233 | 216 / 256 | 30.8 s (21.4 … 41.9) | 61.7 s | 116.5 |
| 2 (C + D) | 234 | 235 | 213 / 256 | 30.7 s (21.4 … 41.9) | 61.3 s | 117.3 |
| 3 (E) | 235 | 235 | 124 / 128 | 30.6 s (21.4 … 41.9) | 61.1 s | 117.5 |

**Installation: 351 pallet movements / h; 175.5 pallets / h and about 4 980 boxes / h leave
through the three output ports** (box rate depends on the SKU mix).

Output interface (external consumer, §13): box list and world poses delivered, lower-layer and
repeated takes refused, a loaded pallet is not released, default order empties top-down,
back-pressure fills the lanes to 7 / 7 / 7 and stops retrieval without any violation ✔.

Travel never reaches 3.0 m/s because the longest move (18.7 m) is shorter than the 19.5 m the
profile needs to get there.

---

## 12. What the simulation does not model

- Dynamics: no drive torques, no mast bending or sway (only the settle time), no rope stretch.
  Motion is kinematic and bounded by the limits of §4.1.
- Fork sag uses the cubic law of §6.3 with assumed coefficients, not a structural model of the
  stages; the load is taken as centred.
- Boxes are rigid and stay exactly where they were stacked: no box-on-box sliding, crushing or
  stack toppling. Pallets carry one SKU in full layers only.
- Positioning is exact: tolerances appear in the budget (§10) and in the monitor windows, but no
  noise is injected.
- Sensors are ideal and there are no faults (no pallet jams, no misreads, no drive trips).
- Collision checking is the rule set R1–R7 plus the static clearance table — it is not a general
  mesh-level collision test.
- No aisle changing, no double-deep storage, no multiple load-handling devices per crane.
- Storage assignment is random within an aisle; inbound pallets are not routed between aisles.
- The neighbouring cells are not modelled. A built-in stand-in empties each presented pallet so
  the ASRS can run on its own (§13.4).

---

## 13. Interfaces to the neighbouring cells (AMR, palletizer / depalletizer)

The ASRS exchanges material at six fixed **ports** at x = −6.10. A port is a pallet position at
the outer end of a conveyor lane.

### 13.1 Port positions (world frame, pallet centre; pallet underside at z = 0.610, top deck at 0.754)

| Port | Direction | x | y | In line with |
|---|---|---|---|---|
| IN-1 | into the ASRS | −6.10 | +1.40 | row A |
| OUT-1 | **out: boxes are taken here** | −6.10 | −1.40 | row B |
| IN-2 | into the ASRS | −6.10 | −2.90 | row C |
| OUT-2 | **out** | −6.10 | −5.70 | row D |
| IN-3 | into the ASRS | −6.10 | −7.20 | row E |
| OUT-3 | **out** | −6.10 | −10.00 | free strip |

A pallet presented at an OUT port stands still, long side along Y, with free access from −X,
from above and (OUT-3) from −Y. The next pallet waits 1.00 m behind it (+X).

### 13.2 What the ASRS guarantees at an output port

- The presented pallet does not move until it is released.
- Every box is axis-aligned with the world (no rotary axis anywhere in the ASRS) and its centre
  is at `pallet position + (lx, ly, lz)` of §3.4, i.e. known without measurement.
- Boxes can only be taken **from the top layer**; within a layer any order is allowed.
- When the pallet is empty the consumer releases it; the next pallet then moves up (≈ 5 s).

### 13.3 Programming interface (`Warehouse` in [js/sim.js](js/sim.js); in the browser `window.asrs.wh`)

| Call | Returns / effect |
|---|---|
| `new Warehouse({ consumer: 'external', inbound: 'auto' })` | `consumer: 'external'` turns the stand-in off so your cell does the taking; `inbound: 'external'` stops automatic pallet supply |
| `outputs()` | one entry per OUT port: `{ port, aisle, pos, queued, pallet }`; `pallet` is `null` or `{ id, sku, pos, boxesLeft, boxes: [{ id, sku, layer, pickable, mass, size: [sx, sy, sz], pos: [x, y, z] }] }` — all positions in the world frame |
| `takeBox(port, boxId?)` | removes one top-layer box and returns its description, or `null` if it is not allowed (no pallet, unknown id, box not in the top layer). Without `boxId` the next top-layer box is taken |
| `releasePallet(port)` | removes the empty pallet; `false` while boxes remain (`{ force: true }` overrides) |
| `requestRetrieval({ slotId })` / `({ sku })` / `()` | orders a stored pallet to the OUT port of its aisle; returns the location id or `null` |
| `feedPallet(port, { skuIndex, layers })` | puts a new pallet on an IN port; `null` if the port position is occupied |
| `presented(port)` | the live pallet object at an OUT port, or `null` |
| `snapshot()` | plain JSON: crane states, inventory per location (pallet, SKU, boxes, mass), outputs, counters |
| `on('boxTaken' \| 'palletReleased' \| 'palletCreated', fn)` | event hooks |
| `step(dt)` | advances the whole installation; the view uses a fixed 1/240 s |

Example of one `outputs()` entry:

```json
{ "port": "OUT-2", "aisle": 2, "pos": [-6.1, -5.7, 0.61], "queued": 1,
  "pallet": { "id": 219, "sku": "BX-2030", "pos": [-6.1, -5.7, 0.61], "boxesLeft": 112,
    "boxes": [ { "id": "219-097", "sku": "BX-2030", "layer": 7, "pickable": true, "mass": 4,
                 "size": [0.2, 0.3, 0.15], "pos": [-6.4, -6.15, 1.729] } ] } }
```

### 13.4 Stand-in consumer (default)

With `consumer: 'auto'` each presented pallet is emptied through exactly the calls above: 3 s
after arrival one top-layer box is taken every `min(0.3 s, 15 s / boxes)`, and the empty pallet
is released 2 s after the last box. Replace it by setting `consumer: 'external'` and driving
`takeBox` / `releasePallet` from the AMR / palletizer model.
