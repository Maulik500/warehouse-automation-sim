# AMR Specification — Pallet AMR, Mast, Fork ("Gripper"), Conveyor Pick Stations and Cell

This file is the contract for the simulation. Every number here is mirrored in
[js/config.js](js/config.js). [js/kinematics.js](js/kinematics.js) implements §5, §7 and §9,
[js/path.js](js/path.js) implements §4.3 and §10.2, [js/sim.js](js/sim.js) implements §6, §8 and
§10, and [test/verify.mjs](test/verify.mjs) checks all of it (`npm run verify:amr`, results in §12).

The system is **one autonomous counterbalanced fork truck (pallet AMR) that takes pallets —
filled or empty — from the end of two conveyors and delivers them**: filled pallets to floor
slots, empty pallets to a stack. Each conveyor end has a **pallet-present sensor** and a
handshake with the AMR (§6).

It is not a model of any vendor's vehicle; dimensions, speeds and clearances are representative
values for a 1000 kg, 1.6 m lift counterbalanced AMR.

**Design decisions behind the numbers** (change them here first if your hardware differs):

| Decision | Chosen | Why |
|---|---|---|
| Vehicle type | counterbalanced fork AMR | the pallet stands on a conveyor at 0.61 m: an under-ride AMR cannot reach it, a straddle truck cannot get its legs under the conveyor |
| Drive | tricycle: 2 fixed load wheels in front, 1 steered + driven wheel at the rear | the standard fork-truck chassis; exact, simple kinematics with a well-defined set of singularities |
| "Gripper" | two fixed tines + backrest + tilt + side shift | a pallet is not gripped: it is held by form fit in its fork pockets, gravity and tilt-back (§7). The side shift is the only DOF that can correct a lateral error (§5.3) |
| Pallet entry side | the 1200 side (fork depth 800) | pallets travel crosswise on a chain conveyor, so the long side is what the conveyor end presents |

---

## 1. Conventions

| Item | Convention |
|---|---|
| Units | metres, radians internally (degrees in tables), seconds, kilograms |
| World frame `{W}` | right-handed, **Z up**, origin on the floor. Conveyors deliver in **−X**; the AMR enters a pick station heading **+X** |
| Body frame `{B}` | origin **on the floor under the midpoint of the load axle** (front axle), **+X = fork direction**, +Y = left, +Z up |
| Reference point | the origin of `{B}`. It is the only point of the chassis that can never move sideways, so every path in this document is the path of that point |
| Pose | `(x, y, θ)` of `{B}` in `{W}`, θ about +Z, 0 = facing +X |
| Mast frame `{M}` | origin on the tilt axis, `x_m` along the fork, `z_m` along the mast; equal to `{B}` shifted by the pivot position when the tilt is 0 |
| TCP `{T}` | on the **top face of the fork**, on the centreline between the tines, **0.42 m in front of the fork face** — exactly under the centre of a fully inserted pallet |
| Pallet frame `{P}` | origin = centre of the pallet underside, `X_p` = the 800 depth (fork direction), `Y_p` = the 1200 length |
| Joint vector | `q = [q1 drive, q2 steer, q3 tilt, q4 lift, q5 side shift]` |
| Path curvature | `κ = ω / v`, positive = turning left (counter-clockwise) |
| three.js mapping | everything is built in a group rotated −90° about three's X, so `{W}`(x, y, z) → three (x, z, −y). No other axis conversion exists anywhere |

---

## 2. Degrees of freedom

| Joint | Name | Type | Axis (in its parent frame) | Positive direction | What it does |
|---|---|---|---|---|---|
| q1 | Drive | R, continuous | drive wheel axle | vehicle moves towards +X_B (forks first) | rolls the steered rear wheel; the only traction |
| q2 | Steer | R | +Z through (−1.30, 0) | wheel turns counter-clockwise seen from above → **vehicle turns right** (the steered wheel is behind the reference point) | sets the instantaneous centre of rotation |
| q3 | Tilt | R | −Y through the mast pivot | mast leans **back**, fork tips rise | cradles the load against the backrest for travel |
| q4 | Lift | P | +z_m (along the mast) | up | raises the fork carriage |
| q5 | Side shift | P | +Y | carriage moves left | lateral correction of the fork, ±100 mm |

**5 actuated joints.** Plus two passive load wheels that only roll.

What that means for everything downstream:

- The chassis has a 3-DOF pose `(x, y, θ)` but only **1 instantaneous degree of mobility** (roll
  along the direction the steered wheel allows) **+ 1 degree of steerability**: it is a type
  (1, 1) wheeled robot, **non-holonomic**. It cannot move sideways, ever (§5.1).
- The TCP can be placed in `x, y, z, yaw` and `pitch` (tilt). It **cannot roll**, and yaw is not
  independent of position: a pallet that stands skewed on the conveyor can only be matched by
  driving the whole approach skewed.
- With the side shift, the *planar* TCP motion `(ẋ, ẏ, θ̇)` is fully actuated by
  `(v, ω, q̇5)` (§5.3) — but only over the ±100 mm shift stroke.
- The fork tines are **not a joint**. The load-handling device has no actuated grip DOF (§7).
- Redundancy: none in the task. Every station is docked with θ, tilt and shift prescribed, so
  there is exactly **one** joint solution per station (no posture branches); the only
  two-fold choice in the system is the drive direction (§5.1).

---

## 3. Geometry

### 3.1 Chassis

| Parameter | Symbol | Value |
|---|---|---|
| Wheelbase, load axle → steer axis | `L` | 1.30 |
| Track of the load wheels | `b` | 1.00 (wheels at y = ±0.50) |
| Load wheels | | Ø0.30 × 0.10, fixed, passive |
| Drive wheel | | Ø0.30 × 0.12, steered and driven, at (−1.30, 0) |
| Plan outline for clearance checks (chassis + mast) | | x = −1.55 … +0.38, width 1.10 |
| Chassis front / height / ground clearance | | x = +0.20 / 0.72 (hood 1.05) / 0.08 |
| Unladen mass, incl. battery and counterweight | `M` | 2100 kg |
| Unladen centre of gravity in `{B}` | | (−0.68, 0, 0.55) |
| Rated payload | | 1000 kg at the TCP (load centre 0.87 m in front of the load axle) |

Wheel loads, unladen: 1098 kg on the drive wheel, 501 kg on each load wheel.

### 3.2 Kinematic chain (what the 3D scene graph implements)

Each frame is its parent translated by `offset`, then moved by its joint.

| Frame | Parent | Offset in parent [m] | Joint motion |
|---|---|---|---|
| Body `{B}` | world | pose (x, y, 0) | `Rz(θ)` — result of q1, q2 over time |
| Steer | body | (−1.30, 0, 0.15) | `Rz(q2)` |
| Drive wheel | steer | (0, 0, 0) | `Ry(q1)` |
| Load wheel L / R | body | (0, ±0.50, 0.15) | `Ry(·)`, passive |
| Mast `{M}` | body | (0.30, 0, 0.35) | `Ry(−q3)` |
| Carriage | mast | (0, 0, q4 − 0.35) | translate along z_m |
| Side shift | carriage | (0, q5, 0) | translate along Y |
| Fork face | side shift | (0.15, 0, 0) | — |
| TCP | side shift | (0.57, 0, 0) | — |
| Fork tips | side shift | (0.95, 0, 0) | — |

The side-shift frame has its origin **on the fork top face**, so `q4` is literally the height of
the fork top face above the floor when the tilt is 0.

Zero-pose check (q3 = 0, q4 = 0.30, q5 = 0): TCP in `{B}` = **(0.870, 0, 0.300)**, tips (1.250, 0, 0.300).
Travel pose (q3 = 4°, q4 = 0.30): TCP = (0.872, 0, 0.340), tips (1.251, 0, 0.366).

### 3.3 Mast

| Parameter | Value |
|---|---|
| Tilt axis in `{B}` (`x_t`, `z_t`) | (0.30, 0.35), parallel to Y |
| Mast channels (tilt 0) | x = 0.24 … 0.38, 0.80 wide, 2.15 high |
| Fork carriage / backrest | 0.90 wide, backrest 1.00 high |
| Pivot → fork face, along the fork | 0.15 |
| Pivot → TCP, along the fork (`a`) | 0.57 |

### 3.4 Pallet (EUR 800 × 1200, nine blocks)

| Parameter | Value |
|---|---|
| Size / mass | 0.80 × 1.20 × 0.144, 25 kg |
| Fork pocket floor (top of the bottom boards) | 0.022 above the underside |
| Fork pocket ceiling (underside of the deck boards) | 0.122 above the underside → **pocket height 100 mm** |
| Fork windows on the 1200 side | \|y_p\| = 0.0725 … 0.455 → two windows of **382.5 mm**, centred at ±0.264 |
| Blocks | 145 long × 100 wide × 78 high at y_p = 0, ±0.5275 and x_p = 0, ±0.35 |
| Filled pallet limits | height ≤ 1.20 incl. pallet, mass ≤ 1000 kg, CG ≤ 0.60 above the deck |

Filled pallets in the simulation carry one box type in full layers (2 … max layers):

| SKU | Box (x_p × y_p × z) | Mass | Per layer | Max layers | Max height / mass |
|---|---|---|---|---|---|
| BX-4030 | 0.400 × 0.300 × 0.250 | 12 kg | 2 × 4 | 4 | 1.144 m / 409 kg |
| BX-4060 | 0.400 × 0.600 × 0.300 | 22 kg | 2 × 2 | 3 | 1.044 m / 289 kg |
| BX-2640 | 0.266 × 0.400 × 0.200 | 8 kg | 3 × 3 | 5 | 1.144 m / 385 kg |

Box centre on its pallet for column `i`, row `j`, layer `k` (from 0):
`(lx, ly, lz) = ((i − (nx−1)/2)·sx, (j − (ny−1)/2)·sy, 0.144 + (k + 0.5)·sz)`.

---

## 4. Joint limits and motion law

### 4.1 Limits

| Joint | Min | Max | Max speed | Max accel | Max jerk | Tolerance |
|---|---|---|---|---|---|---|
| q1 Drive wheel rim | — | — | 2.0 m/s | — | — | — |
| q2 Steer | −90° | +90° | 60°/s | 300°/s² | — | ±0.5° |
| q3 Tilt | −3° (forward) | +6° (back) | 4°/s | 12°/s² | 60°/s³ | ±0.1° |
| q4 Lift, empty | 0.050 | 1.600 | 0.30 m/s | 0.30 m/s² | 1.0 m/s³ | ±2 mm |
| q4 Lift, loaded | | | 0.20 m/s | 0.25 m/s² | 0.8 m/s³ | |
| q4 Lift, transfer (forks in a pallet) | | | 0.05 m/s | 0.20 m/s² | 1.0 m/s³ | |
| q5 Side shift | −0.100 | +0.100 | 0.08 m/s | 0.30 m/s² | 1.5 m/s³ | ±1 mm |

Limits on the path speed `v` of the reference point:

| State | v | a | j |
|---|---|---|---|
| Empty | 1.6 m/s | 0.6 m/s² | 1.2 m/s³ |
| Loaded | 1.2 m/s | 0.4 m/s² | 0.8 m/s³ |
| Creep (fork tips in a station zone) | 0.30 m/s | 0.3 m/s² | 0.6 m/s³ |
| Reverse (counterweight leading) | ≤ 1.0 m/s | as above | as above |

Lateral acceleration `v²·|κ|` ≤ 1.0 m/s². Emergency deceleration 1.5 m/s². Docking accuracy
assumed by the clearance budget: ±5 mm, ±0.25°.

Range actually used by the task: steer ±40.9°, tilt 0 … 4°, lift 0.092 … 0.902, shift ±0.060.

### 4.2 Motion law

Every move — a run along a path, or a mast axis — is a **jerk-limited, rest-to-rest 7-segment
S-curve** ([../asrs/js/motion.js](../asrs/js/motion.js), the same law as the ASRS cranes). For a
run the profile is on the arc length `s` of the path; pose, steer angle and wheel speed are
functions of `s` (§5.1), so the vehicle follows the path exactly at any speed.

Mast axes that move together start together, each on its own time-optimal profile.

### 4.3 Speed cap on a path

A run uses one S-curve with the state's `(v, a, j)`, with `v` reduced so that every limit holds
on the whole run (`speedCap()` in [js/path.js](js/path.js)). With `φ′ = dφ/ds`, `φ″ = d²φ/ds²`:

```
v ≤ √(a_lat / κ_max)                      lateral acceleration
v ≤ v_wheel / √(1 + L²·κ_max²)            drive wheel rim speed
v ≤ φ̇_max / max|φ′|                       steer rate          (φ̇ = φ′·v)
v ≤ √((φ̈_max − max|φ′|·a) / max|φ″|)     steer acceleration  (φ̈ = φ″·v² + φ′·a)
```

For the turn of §10.2 these give 1.22, 1.51, **1.11** and 1.46 m/s: every run that contains a
turn is capped at **1.108 m/s** by the steer rate; straight runs use the full state limit.

Full-stroke / typical times: creep into a station (1.08 m) 5.10 s; lift 0.30 → 0.702 2.64 s;
lift full stroke 6.47 s empty, 8.86 s loaded; load transfer (90 mm) 2.25 s; tilt 0 → 4° 1.53 s;
side shift full stroke 2.97 s.

---

## 5. Kinematics

### 5.1 Chassis

The two load wheels are fixed and roll without side slip, so the body's instantaneous centre
of rotation (ICR) must lie **on the load-axle line**. The steered wheel picks the point:

```
ICR in {B} = (0, R),    R = 1/κ = −L / tan φ           (φ = q2)

Forward (wheel commands → body twist):
    v = v_s · cos φ                    v_s = drive wheel rim speed = r_d · q̇1
    ω = −v_s · sin φ / L
    ẋ = v·cos θ,   ẏ = v·sin θ,   θ̇ = ω

Inverse (path → wheel commands):
    φ   = −atan(L · κ(s))              from the path GEOMETRY, never from atan2 of velocities
    v_s = v · √(1 + L²·κ²)             signed like v
    load wheel rim speeds:  v_L = v − ω·b/2,   v_R = v + ω·b/2

Non-holonomic constraint (always true):   −ẋ·sin θ + ẏ·cos θ = 0
```

The minus signs are the whole story of a rear-steered truck: **steer left, turn right**, and the
tail swings out — in a turn of radius R the rear wheel runs on radius √(R² + L²) and the outer
rear corner on √((R + 0.55)² + 1.55²).

Posture choice: a body twist `(v, ω)` can be produced by `(v_s, φ)` or by `(−v_s, φ ± 180°)`.
The steer range ±90° removes the second solution, so **driving backwards is done by reversing
the drive wheel, not by turning the steer through 180°**. `v_s` is signed; φ stays the same
function of the path whether the path is driven forwards or backwards.

Pose integration over a control step at constant twist is an exact circular arc
(`stepPose()`); the simulation carries a dead-reckoned pose built this way from the wheel
commands only and compares it with the path pose (§12).

### 5.2 Mast and fork (in `{B}`), with `d = q4 − z_t`, `a = 0.57`, `β = q3`

```
Forward:   x_B = x_t + a·cos β − d·sin β
           y_B = q5
           z_B = z_t + a·sin β + d·cos β
           TCP pitch = β (tips up),  TCP yaw = θ,  roll = 0 always

Inverse:   q5 = y_B
           q4 = z_t + (z_B − z_t − a·sin β) / cos β          (β is prescribed, never solved for)
           x_B is NOT free: it is set by the drive (§5.3)

World:     p_W = (x, y, 0) + Rz(θ) · p_B
```

Station docking is always done at β = 0, where the TCP is at **(0.870, q5, q4)** — reach and
height decouple completely.

A carried pallet is rigid on the fork: `{P}` = TCP frame shifted by (0, 0, −0.122).

### 5.3 Base pose for a TCP target, and Jacobians

```
Docked base pose for TCP target (x*, y*) with heading θ, β = 0:
    x = x* − 0.870·cos θ + q5·sin θ
    y = y* − 0.870·sin θ − q5·cos θ

Planar TCP twist in {B} from (v, ω, q̇5):        Mast, (x_B, z_B) from (β, q4):

    | ẋ_T |   | 1   −q5    0 |   | v   |           | −a·sin β − d·cos β    −sin β |
    | ẏ_T | = | 0   0.87   1 | · | ω   |           |  a·cos β − d·sin β     cos β |
    | θ̇   |   | 0    1     0 |   | q̇5  |

    det = −1 everywhere                            det = −d = −(q4 − 0.35)

Wheel commands → twist,  ∂(v, ω)/∂(v_s, φ):       det = −v_s / L
```

---

## 6. Conveyor pick station and pallet-present sensing

### 6.1 Station geometry

| Parameter | Value |
|---|---|
| Lines | P1 at y = +1.60, P2 at y = −1.60; pallet centre at the pick position **x = 6.00** |
| Conveyor | 2-strand chain conveyor, strands at y = ±0.42, **chain top = pallet underside z = 0.610** |
| Flow | −X at 0.20 m/s, accumulation pitch 1.00 m, 6 positions (x = 6 … 11) |
| Pallet orientation | crosswise: 1200 across the conveyor, the 1200 side faces the AMR |
| Frame | 1.40 wide; ends **30 mm before the near face** of the presented pallet (x = 5.57) |
| End stops | three stops 85 mm high **at the block positions only** (y = 0, ±0.5275) — the two fork windows stay open |
| Lateral scatter of a presented pallet | ±60 mm (measured by the AMR, corrected by the side shift) |
| Arrival | one pallet every 120 … 260 s per line, 40 % empty |

### 6.2 Station sensors

| Tag | Sensor | Where (station frame: from the pallet near face, +X upstream; z above the chain) | Signal |
|---|---|---|---|
| **B1** | **Pallet present** — retro-reflective photo-eye, beam across the conveyor | x = 0.06, z = 0.060: through the **front block row**, which is solid at three places across the pallet, so the beam is broken by any pallet, filled or empty, and not by a fork | pallet is at the end stop |
| B2 | Slow-down photo-eye | x = 0.60 | conveyor switches to creep before the stop |
| **B3** | **Load height** photo-eye | z = 0.224 (80 mm above the deck), at the pallet centre | load on the deck → **filled**; B1 ∧ ¬B3 → **empty** |
| B4 | Fork-zone light curtain across the conveyor end | x = −0.25 … 0, z = 0.05 … 0.40 | something is in the transfer zone |
| S1 | Conveyor section stopped (drive off + standstill monitor) | end section | safe to enter |

**`PalletReady` = B1 true for ≥ 0.5 s ∧ S1.** A pallet that is still rolling, bouncing off the
stop, or only passing B2 is not ready. B1 is placed 60 mm inside the pallet so that a pallet
that stopped short of the end stop (> 60 mm) does **not** read as present.

### 6.3 Handshake (station ⇄ AMR)

| # | Signal | Direction | Meaning |
|---|---|---|---|
| 1 | `PalletReady`, `LoadType` (filled / empty) | station → AMR | a job exists; the type selects the destination |
| 2 | `RequestEntry` | AMR → station | AMR is at the pre-dock point |
| 3 | `Locked` | station → AMR | end position stopped and reserved: no pallet will index into it |
| 4 | `InZone` | AMR → station | fork tips are in the transfer zone (cross-checked against B4) |
| 5 | `Clear` | AMR → station | AMR is back at the pre-dock point with the pallet; lock is released |
| 6 | (B1 false ∧ B4 false ∧ `Clear`) | station | index the next pallet to the end stop |

The AMR never enters a pick station zone without `Locked` (rule R8), and never inserts the
forks without `PalletReady`.

### 6.4 Sensors on the AMR that the sequencer relies on

| Sensor | Used for |
|---|---|
| Drive and steer encoders | odometry (§5.1), steer angle |
| Lift, tilt, side-shift position sensors | q3 … q5 |
| Localisation (lidar, natural features) | pose correction; assumed ±5 mm / ±0.25° at a dock |
| Pallet-pocket camera on the carriage | lateral offset of the pallet at the pre-dock point → side-shift target |
| Fork-tip photo-eyes (one per tine) | pocket is free before / while inserting |
| Load present (switches on the backrest + lift pressure) | **`loaded`**: selects the speed limits; the monitor uses it for the fork-window rule |
| Load weight (lift pressure) | overload check against the rated payload |
| Safety scanners front and rear | personnel protection (not simulated) |
| Slot occupied (downward distance sensor on the carriage, or a floor sensor per slot) | no deposit into an occupied slot |

---

## 7. Load-handling device ("gripper") — fixed twin fork

The AMR does not grip. The load is held by **form fit** (tines inside the pallet's fork
pockets), **gravity**, **friction** and **tilt-back against the backrest**. Everything a gripper
specification normally contains has an equivalent here.

### 7.1 Parameters

| Parameter | Value |
|---|---|
| Type | two fixed tines on a side-shifting carriage with backrest |
| Tine length / width / thickness | 0.80 / 0.12 / 0.045 |
| Tine centres | y = ±0.264 → overall width 0.648 (matches the centres of the EUR fork windows) |
| Actuation DOF | none of its own. Positioned by lift, tilt and side shift |
| Insertion | fork face stops **20 mm** before the pallet face; tips end 20 mm before the far face |
| Load centre | 0.42 from the fork face (TCP) = 0.87 in front of the load axle |
| Entry clearance (`c_entry`) | fork top **30 mm below** the pocket ceiling → 25 mm above the pocket floor |
| Carry clearance (`c_lift`) | carried pallet underside **60 mm above** the support plane |
| Friction tine / pallet | μ = 0.30 (steel on wood, conservative) |
| Grip confirmation | `loaded` sensor (§6.4); there is no grip dwell — the load transfers during the lift |

### 7.2 Holding the load and staying upright

All figures for the **rated** load (1000 kg, CG 0.60 above the deck) unless stated.

| Check | Formula | Result |
|---|---|---|
| Tipping forward, static (moments about the load axle) | `M·0.68 / (m·0.87)` | **SF 1.64** |
| Tipping forward under braking, travel pose | `a_tip = g·(M·0.68 − m·0.87) / (M·0.55 + m·z_load)` = 2.64 m/s² | **SF 1.76** vs the 1.5 m/s² emergency stop |
| Same, pallet at conveyor height (creep speed only) | 2.13 m/s² | SF 1.42 |
| Tipping sideways in a turn (3-wheel support triangle) | `a_tip = g·w(x_cg) / z_cg`, `w = b/2·(1 + x_cg/L)` = 6.3 m/s² loaded, 4.25 m/s² unladen | **SF ≥ 4.2** vs 1.0 m/s² |
| Pallet sliding forward off the tines, braking, tilt 4° | `g·tan(atan μ + β)` = 3.71 m/s² | **SF 2.47** (emergency), 9.3 (service) |
| Same, tilt 0° (inside a station) | `μ·g` = 2.94 m/s² | SF 9.8 at creep deceleration |
| Pallet sliding sideways in a turn | `μ·g / a_lat` | **SF 2.9** |
| Pallet sliding backwards | — | stopped by the backrest (20 mm away) |

`x_cg`, `z_cg` are the combined centre of gravity; unladen it lies 0.68 m behind the load axle,
with the rated load 0.18 m behind it. An **empty pallet** (25 kg) is held by exactly the same
form fit; its sliding limits are the same accelerations because they do not depend on mass.

### 7.3 Constraints the fork puts on motion

- Enter and leave a pallet **only along the fork axis**, heading aligned with the station,
  steer at 0, at creep speed (R6).
- Inside a pallet the tines must stay in the pocket: 100 mm of height for a 45 mm tine, 382 mm
  of width for a 120 mm tine (R7).
- **Tilt is 0 for every transfer.** Tilt-back is applied only with the tips outside the station
  zone, and all mast axes move only at standstill (R4).
- Above creep speed the mast must be in the travel pose: lift 0.30, tilt 4°, shift 0 (R5).
- The load cannot be re-oriented or rolled; pallet skew must fit the lateral budget (§11).
- Loaded speed, acceleration and jerk are reduced (§4.1).

---

## 8. Operating cycle

### 8.1 Pick at a conveyor end / deposit at a slot or the stack

| # | Phase | Axes moving | Pick | Deposit |
|---|---|---|---|---|
| 1 | Travel | q1 + q2 | to the pre-dock point, travel pose | same, loaded limits |
| 2 | Wait | none | `PalletReady` → `RequestEntry` → `Locked` | slot free |
| 3 | Set mast | q3, q4, q5 | tilt → 0, lift → **entry**, shift → measured pallet offset | tilt → 0, lift → **carry** |
| 4 | Insert | q1 (creep, straight) | 1.08 m: tips from 0.30 before the pallet face to full insertion | 1.08 m to the slot centre |
| 5 | Transfer | q4 (transfer speed) | **entry → carry**: fork takes the pallet | **carry → entry**: fork sets it down |
| 6 | Withdraw | q1 (creep, straight) | 1.08 m back, with the pallet; then `Clear` | 1.08 m back, empty |
| 7 | Travel pose | q3, q4, q5 | lift → 0.30, tilt → 4°, shift → 0 | lift → 0.30, tilt → 4° |

### 8.2 Lift targets for a support plane `z_s` (chain top, floor, or top of the stack)

```
entry = z_s + 0.122 − 0.030 = z_s + 0.092     (fork passes through the pockets of a resting pallet)
carry = z_s + 0.122 + 0.060 = z_s + 0.182     (carried pallet passes 60 mm over the support)
```

| Station | z_s | entry | carry |
|---|---|---|---|
| Conveyor end | 0.610 | 0.702 | 0.792 |
| Floor slot | 0 | 0.092 | 0.182 |
| Stack, n pallets already there (n = 0 … 5) | n · 0.144 | 0.092 … 0.812 | 0.182 … 0.902 |

During a transfer the pallet follows the fork only once the fork top reaches the pocket ceiling:
`pallet underside = max(z_s, fork top − 0.122)`.

### 8.3 Job selection

- A job is created when a line reports `PalletReady` **and** a destination for its `LoadType`
  is free: filled → the next free floor slot (round robin F1 → F2 → F3), empty → the stack.
- If both lines are ready the pallet that has been present longest is taken first.
- The destination is reserved from job creation until the downstream process has collected it:
  a filled pallet is collected from its slot 90 s after deposit; a full stack (6) 25 s after
  the last pallet.
- **Back-pressure**: with no free destination the AMR does not pick; the pallet stays at the end
  stop and the conveyor accumulates to 6.

---

## 9. Singularities, constraints and the safety monitor

### 9.1 Kinematic singularities

| Singularity | Condition | What goes wrong | Rule in this cell |
|---|---|---|---|
| **Non-holonomic constraint** (structural, always) | lateral velocity of the reference point ≡ 0 | a lateral position error at the dock cannot be driven out without a re-approach | the side shift (±100 mm) is the lateral DOF; `det` of §5.3 is −1 |
| **Standstill** | `v_s = 0` → det ∂(v, ω)/∂(v_s, φ) = 0 | the steer angle that produces a commanded twist is undefined (0/0); computing φ from `atan2(ω, v)` gives noise and a wheel that flips ±90° at every stop | φ is computed from path curvature, which is defined at standstill; every stop is on a straight piece, so φ = 0 at rest |
| **Pivot** | φ = ±90° → `v = 0`, ICR at the reference point | the vehicle can only rotate about the load-axle midpoint; no translation possible; the map κ → φ saturates (κ → ∞) | not used: the task steers within ±40.9° |
| **Pivot on a load wheel** | \|φ\| = atan(L / (b/2)) = 69.0° → ICR on a load wheel | that wheel stops rolling and scrubs; the other carries all the motion | outside the used range |
| **Direction reversal** | `v_s` changes sign | not a singularity, but with φ ≠ 0 the reversal would be a cusp with a steer jump | reversals happen only at rest with φ = 0 |
| **Mast: tilt ∥ lift** | `q4 = z_t = 0.35` → det = 0 | tilting and lifting both move the TCP along the mast: the TCP cannot be moved along the fork by the mast | harmless by design: reach is always set by the drive, tilt is never used for positioning. The travel pose (q4 = 0.30) sits 50 mm from it |
| **Curvature discontinuity** | κ(s) not C² | steer rate / acceleration jump to infinity at any speed > 0 | all turns use quintic curvature ramps: κ, κ′, κ″ continuous (§10.2) |

### 9.2 What goes wrong instead: sequence and tolerance

Rules R1–R10 are implemented as a **safety monitor that reads only the pose, joint positions,
joint speeds, the `loaded` sensor and the station signals** (`monitor()` in
[js/kinematics.js](js/kinematics.js)). It does not know what the sequencer intends.

| Rule | Hazard | Check |
|---|---|---|
| **R1** | joint over-travel | steer, tilt, lift, shift inside §4.1 |
| **R2** | overspeed | path speed ≤ limit of the load state; reverse ≤ 1.0; drive wheel ≤ 2.0; steer rate ≤ 60°/s; mast axis speeds |
| **R3** | tipping / load shift in a turn | `v²·|κ|` ≤ 1.0 m/s² |
| **R4** | mast moving while driving | every mast axis speed is 0 unless the vehicle stands |
| **R5** | travelling with a raised or untilted load | above creep speed only in the travel pose |
| **R6** | fork sweeping sideways in a pallet | in a fork zone: creep speed, steer 0 ± 0.5°, heading aligned ± 0.5°, tilt 0 |
| **R7** | tine hits the pallet, or lifts it unseen | tips inside a resting pallet: tine ≥ 10 mm above the pocket floor, not above the ceiling without `loaded`, ≥ 30 mm from the pocket side walls; a carried pallet in a zone is 0 … 70 mm above the support |
| **R8** | conveyor moves a pallet onto the forks | in a pick zone only with `Locked` |
| **R9** | double occupancy | no carried pallet in a station that holds a resting one |
| **R10** | collision in the plan | chassis and carried pallet ≥ 50 mm from conveyor frames and every other pallet |

A fork zone begins 0.25 m before the near face of a station's pallet position.

---

## 10. Cell layout and paths

### 10.1 Stations (all coordinates in `{W}`; `x, y` = pallet centre)

| Station | Kind | x | y | AMR heading | Support z | Docked base pose | Pre-dock base pose |
|---|---|---|---|---|---|---|---|
| P1 | pick, conveyor end | 6.00 | +1.60 | 0° | 0.610 | (5.13, 1.60) | (4.05, 1.60) |
| P2 | pick, conveyor end | 6.00 | −1.60 | 0° | 0.610 | (5.13, −1.60) | (4.05, −1.60) |
| F1 | floor slot, filled | 0.40 | −6.60 | −90° | 0 | (0.40, −5.73) | (0.40, −4.65) |
| F2 | floor slot, filled | 1.90 | −6.60 | −90° | 0 | (1.90, −5.73) | (1.90, −4.65) |
| F3 | floor slot, filled | 3.40 | −6.60 | −90° | 0 | (3.40, −5.73) | (3.40, −4.65) |
| E1 | stack of empties, ≤ 6 | 1.90 | +6.60 | +90° | n · 0.144 | (1.90, 5.73) | (1.90, 4.65) |
| H | home / parking | −1.60 | — | −90° | — | — | (−1.60, −4.65) |

Docked base = pallet centre − 0.87 along the heading; pre-dock = a further **1.08 m** back
(0.30 standoff + 0.78 insertion).

### 10.2 Paths

Every route between a pick station (heading +X) and a side station (heading ±Y) is one **L**:

```
link:  straight along the pick line, from (x_side − T, y_pick) to the pick pre-dock point
leg:   from (x_side − T, y_pick): 90° turn onto the side station's line, then straight to its pre-dock point

pick → side:   reverse along the link, then forward along the leg
side → pick:   reverse along the leg (backing through the turn), then forward along the link
```

The turn is a **continuous-curvature** curve — quintic ramp, constant arc, quintic ramp:

| Parameter | Value |
|---|---|
| Radius of the arc `R` | 1.50 (κ_max = 0.667 1/m) |
| Ramp length `L_c` | 1.50 each; `κ(u) = κ_max·(10τ³ − 15τ⁴ + 6τ⁵)`, τ = u / L_c |
| Heading change per ramp / in the arc | 28.65° / 32.70° |
| Turn length | 3.856 |
| Tangent length `T` (corner → start / end of the turn) | **2.275** |
| Steer angle at κ_max | 40.91° |
| Drive wheel speed / path speed at κ_max | 1.323 |
| Swept radii at κ_max: inner side / fork tip (outer) / rear wheel / outer rear corner | 0.95 / 2.21 / 1.98 / 2.57 |

Heading along a piece is analytic (`θ = θ₀ + κ₀·u + (κ₁ − κ₀)·L·(2.5τ⁴ − 3τ⁵ + τ⁶)`); position
is integrated from it with Simpson's rule at 1 mm.

All stops are on straight pieces with κ = 0, so the steer is at 0 at every stop and at every
direction reversal.

---

## 11. Clearance budget

| Clearance | Nominal | Consumers |
|---|---|---|
| Tine ↔ fork-window side wall (Y) | 131 mm each side | pocket camera ±5, side-shift ±1, pallet skew, docking ±5 |
| Tine ↔ pocket floor / ceiling at entry (Z) | 25 / 30 mm | lift ±2, chain / floor level, tine droop |
| Tine ↔ conveyor end stop (Y) | 154 mm | same as the window |
| Fork tip ↔ far face of the pallet (X) | 20 mm short | docking ±5 |
| Fork tip ↔ next pallet on the conveyor (X) | 220 mm | — |
| Mast ↔ conveyor frame end, docked (X) | 60 mm | docking ±5, pallet stop position |
| Chassis front / load wheels ↔ conveyor frame end (X) | 240 / 290 mm | — |
| Carried pallet ↔ conveyor frame end at the pre-dock point (X) | 250 mm | — |
| Carried pallet ↔ chain during withdrawal (Z) | 60 mm | lift ±2, tine droop under load |
| Carried pallet ↔ neighbouring floor pallets (X) | 300 mm each side | docking ±5, heading ±0.25° over 1.9 m = ±8 |
| Mast ↔ deposited pallet / stack, docked | 90 mm | — |
| Carried pallet underside ↔ floor, travel pose | 190 mm at the heel edge | — |
| Pallet skew tolerated without correction | ±1.5° → ±10 mm at the tine tips | part of the 131 mm |

---

## 12. Verification results (`npm run verify:amr`)

4 h of operation simulated at 240 Hz, seed 7:

| Check | Result |
|---|---|
| 90° turn: end heading, symmetry about the corner | exact (0 rad); T = 2.2749 m, turn length 3.8562 m ✔ |
| Path position table (Simpson, 1 mm) vs 400 000-step integration | 7·10⁻¹² m ✔ |
| Heading = ∫ curvature; κ, κ′, κ″ continuous | 6·10⁻¹¹ rad; no steps ✔ |
| Wheel commands ↔ body twist, 20 000 random twists | 1·10⁻¹⁵ ✔ |
| Steer left turns right; lateral speed of the reference point | ✔; 2·10⁻⁶ m/s (finite-difference noise) ✔ |
| det ∂(v, ω)/∂(v_s, φ) = −v_s / L vs finite differences | 2·10⁻⁹ ✔ |
| Zero-pose TCP | (0.870, 0.000, 0.300) ✔ |
| Lift IK(FK), base pose IK → FK, 20 000 random poses | 4·10⁻¹⁶, 3·10⁻¹⁵ m ✔ |
| Mast Jacobian vs finite differences; det = −(q4 − z_t); planar det = −1 | 7·10⁻⁷ ✔ |
| All 10 L-routes end on the pre-dock pose; docked TCP = pallet centre | 3·10⁻¹³ m; 0 ✔ |
| Shortest straight run-in after a turn | 0.775 m ✔ |
| Static clearances of §11, lift range 0.092 … 0.902 inside 0.05 … 1.60 | ✔ |
| Stability and load holding (§7.2) | SF 1.64 / 1.76 / 4.25 / 2.47 / 2.94 ✔ |
| Safety monitor negative tests (20 illegal states, 4 legal) | all classified correctly ✔ |
| Safety monitor violations during the run | 0 ✔ |
| Mast axis moving while the vehicle moves (raw check) | 0 steps ✔ |
| Fork in a pick zone without `Locked` / without `PalletReady` (raw check) | 0 / 0 steps ✔ |
| Peak path speed: forward / reverse / loaded / in a fork zone | 1.600 / 1.000 / 1.108 / 0.300 m/s ✔ |
| Peak path acceleration / jerk | 0.600 m/s² / 1.20 m/s³ ✔ |
| Peak drive wheel rim speed | 1.600 of 2.0 m/s ✔ |
| Peak steer angle / rate / acceleration | 40.91° / 60.0 of 60°/s / 154 of 300°/s² ✔ |
| Peak lateral acceleration | 0.818 of 1.0 m/s² ✔ |
| Peak mast speeds | tilt 4.00°/s, lift 0.300 m/s, shift 0.080 m/s ✔ |
| Joint ranges used | lift 0.092 … 0.902, tilt 0 … 4°, shift −0.060 … +0.060 ✔ |
| Dead reckoning from the wheel commands vs the path pose | worst 0.06 mm over 3.91 km ✔ |
| Tines centred on the pallet at engagement (side shift) | 0 ✔ |
| Pallet set down on the station centre, level, on the support plane | 4·10⁻¹³ ✔ |
| Minimum plan clearance | 60 mm (mast ↔ conveyor frame, docked) ✔ |
| Pallets conserved; filled only to floor slots, empty only to the stack | 154 created = 4 in the cell + 150 collected; 96 filled, 56 empty ✔ |
| Longest conveyor queue | 3 of 6 ✔ |

**152 pallets delivered = 38.0 / h, limited by pallet supply.** Cycle (job start → pallet
delivered) 65.8 s mean (61.0 … 72.0), which is a capacity of **55 pallets / h** for one AMR.

A typical cycle is about 15 s of travel on each of the two trips, 10 s of creeping per station
(in and out), and 6 … 9 s of mast motion per station.

Reference joint solution, docked at P1 with a pallet 50 mm off centre:
base (5.130, 1.600, 0°), steer 0°, tilt 0°, lift 0.702, shift +0.050.

The verification found one real fault while this model was built: locking the conveyor end
pushed the presented pallet one pitch upstream. The monitor did not see it (no rule was
broken); the raw `PalletReady` check did. That is why raw checks exist next to the monitor.

---

## 13. What the simulation does not model

- Dynamics: no drive or steer torques, no tyre slip or deflection, no mast or tine bending, no
  hydraulic compliance. Motion is kinematic and bounded by the limits of §4. The stability
  figures of §7.2 are quasi-static.
- Localisation and control error: the vehicle is exactly on its path. Tolerances appear in the
  budget (§11) and in the monitor windows, but no noise is injected. The only measured
  disturbance is the lateral position of the presented pallet.
- Pallet skew, damaged pallets, overhanging or shifted loads; boxes are rigid and never slide.
- Of the station sensors only B1 (with its debounce), B3 and the `Locked` handshake are
  simulated; B2, B4 and S1 are specified but have no separate model.
- Sensors are ideal and there are no faults (no false B1, no pallet stopping short, no lost
  handshake); the recovery sequences are specified by the rules but never exercised.
- Collision checking is the rule set R6–R10 with plan (2D) polygons for the chassis and
  pallets — not a mesh-level 3D test. People and other vehicles are not modelled.
- Path planning is the fixed L-route network of §10.2, not a free-space planner; there is one
  AMR, so no traffic management.
- Battery, charging and the conveyor's own acceleration ramps.
- The downstream process: slots and the stack are emptied by a timer.
