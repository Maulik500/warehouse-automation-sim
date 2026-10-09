# Mobile Depalletizer Specification — AMR, On-board Conveyor, 6-Axis Arm, Vacuum Gripper and Cell

This file is the contract for the simulation. Every number here is mirrored in
[js/config.js](js/config.js). [js/arm.js](js/arm.js) implements §6.2–§6.3 and §7.1,
[js/base.js](js/base.js) implements §6.1, §6.4 and §8, [js/recipes.js](js/recipes.js) implements §4,
[js/clearance.js](js/clearance.js) implements §14, [js/sim.js](js/sim.js) implements §10–§12, and
[test/verify.mjs](test/verify.mjs) checks all of it (`npm run verify:mobman`, results in §15).
[js/main.js](js/main.js) is the three.js view (`npm run serve`, then `/mobman/`), and
`npm run export:mobman` + [blender/build_mobman.py](blender/build_mobman.py) rebuild the same cell,
rigged and animated, in Blender ([blender/mobman_cell.blend](blender/mobman_cell.blend)).

The system is **one AMR that carries a roller conveyor and a 6-axis arm**. It parks beside a
pallet that stands at the end of a fixed conveyor, the arm takes boxes off the pallet and builds
them into a layer on the AMR's own conveyor, the AMR drives to a second fixed conveyor, docks
nose-on, and its conveyor rolls the boxes across.

It is not a model of any vendor's product; dimensions, limits and speeds are representative
values for a 300 kg unit-load AMR and a 20 kg / 1.75 m collaborative arm.

**Design decisions behind the numbers** (change them here first if your hardware differs):

| Decision | Chosen | Why |
|---|---|---|
| Base | differential drive, 2 driven wheels + 4 casters | given. Non-holonomic: it cannot move sideways, so every dock is approached along the vehicle's own axis (§6.1) |
| Arm | 6R **offset-wrist** arm, 20 kg / 1.75 m (cobot geometry) | given. Light enough for a battery vehicle; its three wrist axes do **not** intersect, so its kinematics and singularities differ from a spherical-wrist industrial arm (§6.3, §7.1) |
| Arm position | on a fixed 1.40 m pedestal on the **left side** of the AMR, at mid-length | the pallet is on one side of the arm and the deck on the other: nothing is farther than 1.19 m from the J1 axis (§15). With the arm at the front or rear, the far end of a 1.2 m layer is out of reach |
| Lift column (7th axis) | **not needed** | with the pedestal at 1.40 m and the pallet conveyor at 0.30 m every pick and place position of every recipe is reachable in one posture branch with elbow margin ≥ 0.51 (§15). The lift was allowed; the reach study showed it buys nothing here |
| Base repositioning | **one dock pose** per station | same reason: the whole pallet is reachable from one pose, so the arm never works from a base that has to be re-localised mid-pallet |
| Docking | **side-on** at the pallet, **nose-on** at the destination | the conveyor-to-conveyor transfer needs millimetres laterally and is done along the drive axis; the pallet side tolerates centimetres because the arm measures the pallet and corrects |
| Gripper | vacuum pad, top pick, tool always vertical | given. Closed cartons, ≤ 15 kg. A vertical tool also pins the wrist at q5 = 90°, the farthest point from the wrist singularity (§7.1) |
| Deck | powered rollers, 1.24 × 0.86 m between stops and guides, discharge over the front | one full EUR layer; end transfer, as given |
| Order size | per trip: a requested number of boxes, at most one layer | given: "2, 3, 4, 5, 8 … as required" |
| Depleted pallet | replaced by the next recipe by an external system | given: modelled as a timer (§10.1) |

---

## 1. Conventions

| Item | Convention |
|---|---|
| Units | metres, radians internally (degrees in tables), seconds, kilograms |
| World frame `{W}` | right-handed, **Z up**, origin on the floor at the AMR reference point when docked at the source. The source conveyor delivers in **+X**; the destination conveyor carries away in **−Y** |
| Body frame `{B}` | origin **on the floor under the midpoint of the drive axle**, **+X = forward** (the end the deck discharges over), **+Y = left (the arm side)**, +Z up |
| Reference point | the origin of `{B}`. It is the only point of the chassis that can never move sideways, so every path in this document is the path of that point |
| Pose | `(x, y, θ)` of `{B}` in `{W}`, θ about +Z, 0 = facing +X |
| Arm base frame `{A}` | origin on the pedestal top on the J1 axis, at (0, 0.47, 1.40) in `{B}`, **turned 180° about Z** against `{B}`: `X_A = −X_B`, `Y_A = −Y_B` |
| Joint frames | classic Denavit–Hartenberg (§3.3); joint *i* turns about `z_{i−1}`, positive by the right-hand rule |
| Flange frame `{F}` | DH frame 6: origin at the flange face centre, **+Z out of the flange** (approach axis) |
| Tool frame `{T}` | same orientation as `{F}`, origin on the cup sealing plane (TCP); tool X = long side of the pad |
| Pallet frame `{P}` | origin = centre of the pallet underside, **`X_p` = the 1200 length** (conveying direction), `Y_p` = the 800 width |
| Box | footprint `l ≥ w`, height `h`; `yaw` = angle of the long side from `X_p` (or `X_B` on the deck) |
| Joint vectors | arm `q = [q1 … q6]`; base wheels `(φ_L, φ_R)`; deck `(u, stop)` |
| Path curvature | `κ = ω / v`, positive = turning left |
| three.js mapping | everything is built in a group rotated −90° about three's X, so `{W}`(x, y, z) → three (x, z, −y). No other axis conversion exists anywhere |

---

## 2. Degrees of freedom

| # | Joint | Type | Axis | Positive direction | What it does |
|---|---|---|---|---|---|
| b1 | Left wheel | R, continuous | +Y_B through (0, +0.50) | rolls forward | traction + steering |
| b2 | Right wheel | R, continuous | +Y_B through (0, −0.50) | rolls forward | traction + steering |
| — | 4 casters | 2 × R each, passive | swivel Z + roll | — | support only |
| J1 | Base / swing | R | `z_0` = +Z | counter-clockwise from above | sweeps the arm between pallet (q1 ≈ +30…+115°) and deck (q1 ≈ −70…−110°) |
| J2 | Shoulder | R | `z_1`, horizontal | (DH) | raises / lowers the upper arm |
| J3 | Elbow | R | `z_2` ∥ `z_1` | (DH) | folds the forearm; q3 = 0 is the stretched arm |
| J4 | Wrist 1 | R | `z_3` ∥ `z_1` | (DH) | pitches the wrist in the arm plane |
| J5 | Wrist 2 | R | `z_4` ⟂ `z_3` | (DH) | swings the tool axis out of the arm plane; q5 = 0 is the wrist singularity |
| J6 | Wrist 3 / flange | R | `z_5` = tool axis | (DH) | spins the tool: the box yaw |
| g | Vacuum | binary | — | on | holds the box; no kinematic DOF |
| c1 | Deck rollers | R, continuous (one drive) | +Y_B | boxes move +X_B | transfers the layer |
| c2 | Front stop | P, binary (0.4 s) | +Z | raised | keeps the layer on the deck |

**8 servo axes (2 wheels + 6 arm joints), 1 conveyor drive, 2 binary actuators.**

What that means for everything downstream:

- The base has a 3-DOF pose `(x, y, θ)` but only **2 velocity inputs**: it is a type (2, 0) wheeled
  robot, **non-holonomic**. It can turn on the spot, but it cannot move sideways, ever (§6.1).
- J2, J3 and J4 are **parallel**: together they are a planar 3R chain in the vertical plane
  that J1 swings around. J5 and J6 then orient the tool. Because the wrist axes are offset, the
  arm cannot be split into "position with J1–J3, orientation with J4–J6" (§6.3).
- **Tool down fixes two joints.** A vertical tool axis is only possible with `q5 = ±90°` and
  `q2 + q3 + q4 = ±90°` (§6.3). The task therefore uses 4 of the arm's 6 DOF: x, y, z and yaw.
- Redundancy: none in the arm (6 joints, 6-DOF task space). The mobile manipulator as a whole is
  redundant (8 inputs), and that redundancy is **removed by rule**: the arm moves only while the
  base stands docked, the base moves only while the arm is stowed (§12, M4). There is no
  coordinated base + arm motion anywhere.
- The vacuum pad is symmetric under 180° about its axis, and a box is symmetric under 180° about
  the vertical: every pick and every place has two equivalent yaws, and the planner takes the one
  that needs less J6 travel.

---

## 3. Geometry

### 3.1 Chassis

| Parameter | Symbol | Value |
|---|---|---|
| Track of the drive wheels | `b` | 1.00 (wheels at y = ±0.50, on the X_B = 0 line) |
| Drive wheels | `r` | Ø0.20 × 0.06 |
| Casters | | Ø0.10, at (±0.60, ±0.46) |
| Plan outline | | x = −0.75 … +0.75, width 1.24 |
| Chassis height / ground clearance | | 0.36 / 0.03 |
| Support polygon used for stability | | the caster rectangle, 1.20 × 0.92 |

### 3.2 On-board conveyor ("deck")

| Parameter | Value |
|---|---|
| Roller top `z_d` | **0.650** (= destination conveyor) |
| Centreline in `{B}` | y = −0.14 (the arm pedestal takes the left 0.30 m of the vehicle) |
| Clear width between side guides | 0.86 |
| Roller bed | x = −0.70 … +0.75, pitch 75 mm, Ø50 |
| Layer area | 1.20 × 0.80, centred at (0, −0.14) |
| Fixed rear stop / pop-up front stop | x = −0.62 / x = +0.62, 45 mm high |
| Side guides | 50 mm high |
| Transfer speed / ramp | 0.30 m/s / 0.6 m/s² |
| Capacity | 16 boxes, 130 kg |
| Friction carton / braked roller | μ = 0.30 |

### 3.3 Arm

| Symbol | Value [m] | Meaning |
|---|---|---|
| `d1` | 0.236 | base face → J2 axis height |
| `a2` | 0.862 | J2 → J3 (upper arm) |
| `a3` | 0.729 | J3 → J4 (forearm) |
| `d4` | 0.201 | lateral offset of the wrist from the J1 axis plane |
| `d5` | 0.159 | J4 axis → J6 axis |
| `d6` | 0.154 | J5 axis → flange face |
| `tcp` | 0.160 | flange face → cup plane (§9) |

Denavit–Hartenberg table (classic / distal: `Rz(θ)·Tz(d)·Tx(a)·Rx(α)`), in `{A}`:

| i | θ | d [m] | a [m] | α |
|---|---|---|---|---|
| 1 | q1 | 0.236 | 0 | +90° |
| 2 | q2 | 0 | −0.862 | 0° |
| 3 | q3 | 0 | −0.729 | 0° |
| 4 | q4 | 0.201 | 0 | +90° |
| 5 | q5 | 0.159 | 0 | −90° |
| 6 | q6 | 0.154 | 0 | 0° |

The 3D scene graph is this table, one group per row, and nothing else.

Zero pose (all q = 0): the arm lies stretched and horizontal along −X_A; TCP =
(−(a2 + a3), −(d4 + d6 + tcp), d1 − d5) = **(−1.591, −0.515, 0.077)** in `{A}`.
Upright pose (q2 = q4 = −90°): flange at (0, −0.355, 1.986).

**Where the tubes really are.** The DH lines of the upper arm and forearm lie in the plane
through the J1 axis; the physical tubes are beside it, offset along the J2 axis: upper arm at
**0.215**, forearm at **0.065**, wrist at 0.201 (= d4). Clearance checks and the drawing use
these offsets; the kinematics do not need them.

Capsule radii for clearance: upper arm 0.085, forearm 0.070, wrist 0.060, tool 0.050.

### 3.4 Mounting

| Parameter | Value |
|---|---|
| `{A}` in `{B}` | origin (0, **0.47**, **1.40**), `Rz(180°)` |
| Pedestal | Ø0.22, on the chassis top, 0.15 m inside the left edge |
| J2 axis height above the floor | 1.636 |
| J1 = 0 | the arm plane points over the **front** of the AMR: the swing between pallet and deck passes over the front, where nothing stands |

---

## 4. Loads

### 4.1 Pallet and limits

| Parameter | Value |
|---|---|
| Pallet | EUR 1.20 × 0.80 × 0.144, 25 kg |
| Load footprint | never larger than the pallet |
| Height, pallet + load | **≤ 1.50** |
| Box (gripper range) | l 0.30 … 0.60, w 0.20 … 0.40, h 0.15 … 0.40, **≤ 15 kg**, closed top |
| Pallet content | one SKU per pallet; filled boxes only |

### 4.2 Recipes

A recipe is a box and a layer pattern. `grid`: `nx × ny` boxes, all at one yaw. `rows`: rows
stacked along `Y_p`, each with its own count and yaw, mirrored in Y on every second layer
(interlocked).

| Recipe | Box l × w × h | Mass | Pattern | Per layer | Layers | Boxes | Layer size | Height | Gross |
|---|---|---|---|---|---|---|---|---|---|
| R1 | 0.400 × 0.300 × 0.250 | 12 | grid 4 × 2, yaw 90° | 8 | 5 | 40 | 1.200 × 0.800 | 1.394 | 505 kg |
| R2 | 0.600 × 0.400 × 0.300 | 15 | grid 2 × 2, yaw 0° | 4 | 4 | 16 | 1.200 × 0.800 | 1.344 | 265 kg |
| R3 | 0.400 × 0.266 × 0.200 | 8 | grid 3 × 3, yaw 0° | 9 | 6 | 54 | 1.200 × 0.798 | 1.344 | 457 kg |
| R4 | 0.300 × 0.200 × 0.150 | 5 | grid 4 × 4, yaw 0° | 16 | 9 | 144 | 1.200 × 0.800 | 1.494 | 745 kg |
| R5 | 0.400 × 0.300 × 0.250 | 12 | rows: 4 at 90° + 3 at 0°, interlocked | 7 | 5 | 35 | 1.200 × 0.700 | 1.394 | 445 kg |
| R6 | 0.600 × 0.400 × 0.400 | 14 | grid 2 × 2, yaw 0° | 4 | 3 | 12 | 1.200 × 0.800 | 1.344 | 193 kg |

`checkRecipe()` rejects a recipe whose layer is larger than the pallet or the deck, whose stack is
higher than 1.50, whose box is outside the gripper range, whose layer is heavier than the deck
rating, or whose box top is not at least 20 mm larger than the pad on every side.

Box centre for column `i`, row `j`, layer `k` of a grid:
`(x, y, z) = ((i − (nx−1)/2)·s_x, (j − (ny−1)/2)·s_y, 0.144 + (k + 0.5)·h)` in `{P}`.

### 4.3 Order of work

- **Pick order**: top layer first, strictly. Inside a layer the row nearest the AMR first, then
  front to back. A lower layer is never touched while a box of the layer above remains.
- **Deck slots**: the layer pattern of the recipe laid on the deck, filled front row first (it
  leaves first and rests against the front stop), far side from the arm first.
- **Order size**: per trip a requested quantity from the list `[layer, 2, 3, layer, 4, 5, 8,
  layer]` (repeating); the trip takes `min(request, one layer, boxes left on the pallet)`.
  A request may span two layers of the pallet.

---

## 5. Limits and motion laws

### 5.1 Base

| State | v | a | j |
|---|---|---|---|
| Empty | 1.5 m/s | 0.6 m/s² | 1.2 m/s³ |
| Loaded (anything on the deck) | 1.0 m/s | 0.4 m/s² | 0.8 m/s³ |
| Creep (inside a station zone) | 0.20 m/s | 0.2 m/s² | 0.5 m/s³ |
| Reverse | ≤ 1.0 m/s | as above | as above |

| Limit | Value |
|---|---|
| Wheel rim speed / acceleration | 1.8 m/s / 1.2 m/s² |
| Yaw rate / yaw acceleration | 60°/s / 120°/s² |
| Lateral acceleration `v²·|κ|` | 0.6 m/s² |
| Emergency deceleration | 1.5 m/s² |
| Docking accuracy assumed by the clearance budget | ±10 mm, ±0.5° |

Every run is a **jerk-limited, rest-to-rest 7-segment S-curve** on the arc length of the path
([../asrs/js/motion.js](../asrs/js/motion.js)). The speed of a run is capped so that every limit
holds on the whole run (`speedCap()`), with `κ′ = dκ/ds`:

```
v ≤ √(a_lat / κ_max)                              lateral acceleration
v ≤ v_wheel / (1 + κ_max·b/2)                     outer wheel rim speed
v ≤ ω_max / κ_max                                 yaw rate
v ≤ √((α_max − κ_max·a) / max|κ′|)                yaw acceleration   (ω̇ = κ′·v² + κ·a)
v ≤ √((a_wheel − a·(1 + κ_max·b/2)) / (max|κ′|·b/2))   wheel acceleration
```

For the turn of §13 these give **0.949**, 1.350, 1.571 … m/s: every run through the turn is
capped at 0.949 m/s by lateral acceleration.

### 5.2 Arm

| Joint | Min | Max | Max speed | Max accel | Range used (§15) |
|---|---|---|---|---|---|
| J1 | −360° | +360° | 120°/s | 220°/s² | −167 … +115° |
| J2 | −360° | +360° | 120°/s | 220°/s² | −109 … +11° |
| J3 | −165° | +165° | 150°/s | 300°/s² | +40 … +149° |
| J4 | −360° | +360° | 210°/s | 450°/s² | −35 … +95° |
| J5 | −360° | +360° | 210°/s | 450°/s² | +90° (constant) |
| J6 | −360° | +360° | 210°/s | 450°/s² | −274 … +151° |

Cartesian limits for straight moves: 1.0 m/s, 2.5 m/s². Rated payload 20 kg at the flange; the
application carries at most 19.5 kg (4.5 kg gripper + 15 kg box).

- **PTP** (joint move): all six joints share one quintic time law `s(τ) = 10τ³ − 15τ⁴ + 6τ⁵`
  (zero velocity and acceleration at both ends). Duration = the slowest joint:
  `T = max_i( 1.875·|Δq_i| / v_i , √(5.7735·|Δq_i| / a_i) )`.
- **LIN** (straight TCP move): the same quintic law on path length with the Cartesian limits;
  joints come from IK at every control step. The duration is then **stretched** until every joint
  stays inside its speed and acceleration limit along the whole line (the line is sampled at 49
  points before it starts).
- **PTP keeps the tool vertical for free**: both ends have q5 = 90° and q2 + q3 + q4 = 90°, and
  all joints use the same `s(τ)`, so both stay constant along the whole move.

### 5.3 Deck

Rollers 0 → 0.30 m/s at 0.6 m/s²; front stop 0.4 s per stroke.

---

## 6. Kinematics

### 6.1 Base

```
Forward (wheel rim speeds → body twist):
    v = (v_R + v_L) / 2
    ω = (v_R − v_L) / b
    ẋ = v·cos θ,   ẏ = v·sin θ,   θ̇ = ω

Inverse (path → wheel commands):
    ω   = v · κ(s)                    κ from the path GEOMETRY, never from atan2 of velocities
    v_L = v·(1 − κ·b/2),   v_R = v·(1 + κ·b/2)

Non-holonomic constraint (always true):   −ẋ·sin θ + ẏ·cos θ = 0

ICR in {B} = (0, 1/κ):   on the axle line, always
```

The map `(v_L, v_R) → (v, ω)` is linear with the **constant determinant 1/b**: the drive itself
has no singular configuration. Special cases that are not singularities: `κ = ±2/b` puts the ICR
on a wheel (that wheel stands still, the other does all the work); `v = 0, ω ≠ 0` is a pivot
about the reference point.

Pose integration over a control step at constant twist is an exact circular arc (`stepPose()`).
The simulation carries a dead-reckoned pose built this way from the wheel speeds only and
compares it with the path pose (§15).

### 6.2 Arm, forward

With `q23 = q2 + q3`, `q234 = q2 + q3 + q4`, in `{A}`:

```
r̂ = (cos q1, sin q1, 0)          radial direction of the arm plane
ẑ1 = (sin q1, −cos q1, 0)         J2 axis = normal of the arm plane

O5 (wrist point, on the J5 and J6 axes):
    p_r = −a2·cos q2 − a3·cos q23 + d5·sin q234        along r̂
    p_z = d1 − a2·sin q2 − a3·sin q23 − d5·cos q234
    O5  = p_r·r̂ + d4·ẑ1 + p_z·ẑ

R_06   = Rz(q1)·Rx(90°) · Rz(q234)·Ry(−q5)·Rz(q6)
flange = O5 + d6·(R_06·ẑ)
TCP    = O5 + (d6 + 0.160)·(R_06·ẑ)
```

`O5` always lies in the plane at distance `d4` from the J1 axis: that is the whole geometry of an
offset wrist, and both the J1 solution and the shoulder singularity come from it.

### 6.3 Arm, inverse (closed form)

Input: TCP position `p`, flange orientation `R`, one of 8 posture branches
`(shoulder, elbow, wrist) = (±1, ±1, ±1)`, and a reference joint vector that fixes the 2π-turn.

1. `ẑ6 = R·ẑ`; flange `p6 = p − 0.160·ẑ6`; wrist point `p5 = p6 − d6·ẑ6`.
2. **J1**: `ρ = |p5.xy|`, `ψ = atan2(p5.y, p5.x)`. `ρ < d4` ⇒ no solution. Otherwise
   `q1 = ψ + asin(d4/ρ)` (shoulder +) or `q1 = ψ + 180° − asin(d4/ρ)` (shoulder −).
3. `R_16 = (Rz(q1)·Rx(90°))ᵀ · R = Rz(q234)·Ry(−q5)·Rz(q6)` — a ZYZ Euler set:
   - **J5**: `q5 = wrist · acos(R_16[2][2])`
   - `q234 = atan2(−R_16[1][2]/s5, −R_16[0][2]/s5)`
   - **J6**: `q6 = atan2(−R_16[2][1]/s5, R_16[2][0]/s5)`
   - `s5 = 0` ⇒ wrist singularity, no solution returned.
4. Wrist-1 point in frame 1: `p14 = R_01ᵀ·(p6 − d1·ẑ) − Rz(q234)·Rx(90°)·(−d6·s5, d6·c5, d5)`;
   `(X, Y) = −p14.xy`.
5. **J3**: `cos q3 = (X² + Y² − a2² − a3²) / (2·a2·a3)`; `|cos q3| > 1` ⇒ out of reach;
   `q3 = elbow · acos(…)`.
6. **J2**: `q2 = atan2(Y, X) − atan2(a3·sin q3, a2 + a3·cos q3)`. **J4**: `q4 = q234 − q2 − q3`.
7. Bring every joint to the turn nearest the reference; reject if any joint is outside §5.2.

**Tool pointing down**: `ẑ6 = −ẑ` gives `R_16[2][2] = 0`, so **q5 = ±90° and q234 = ±90°**,
whatever the position.

**The simulation uses one branch for everything: shoulder −, elbow +, wrist +**, which gives
q5 = +90°, the elbow above the wrist, and q2 between −109° and +11°. Its mirror image
(shoulder +, elbow −, wrist −) is the only other branch that reaches the whole task space; the
remaining six fail on reach or put the elbow into the deck. A branch can only be changed by
passing through the corresponding singularity, so the fixed branch also means no Cartesian move
ever approaches one.

Reference vector for the turn of J1…J5: `(0, −34.4°, 85.9°, 34.4°, 90°)`; J6 follows its own
previous value.

### 6.4 Mobile manipulator: base pose + arm

```
T_W_A(pose) = Trans(x, y, 0) · Rz(θ) · Trans(0, 0.47, 1.40) · Rz(180°)

p_W   = T_W_A · p_A                    TCP in the world
p_A   = T_W_Aᵀ · p_W                   world target into the arm frame for IK
yaw_A = yaw_W − θ − 180°               tool yaw the same way
```

The base pose is a constant while the arm moves (§12, M4), so the arm solves a fixed-base problem
at each station. The base cannot correct a lateral docking error; **the arm does**: the camera
measures the pallet pose after docking (§10.3) and every pick target is the measured box, not
the nominal one. Deck slots are fixed in `{B}`, so they need no measurement at all.

---

## 7. Singularities

### 7.1 Arm

The Jacobian determinant of the flange (linear + angular velocity) factorises exactly as

```
det J = a2 · a3 · sin q3 · sin q5 · (−a2·cos q2 − a3·cos q23 + d5·sin q234)
```

(verified against a numerically built Jacobian, deviation 9·10⁻¹⁶). The last factor is `p_r` of
§6.2. Each factor is one singularity:

| Singularity | Condition | Where it is | What goes wrong | Rule in this cell |
|---|---|---|---|---|
| **Elbow** | `sin q3 = 0` → q3 = 0° (stretched) or 180° (folded) | wrist-1 point on the reach boundary | no velocity possible along the arm; J2 / J3 speeds blow up for a small radial TCP motion | monitor: `|sin q3| ≥ 0.25` while the arm moves. Used: ≥ **0.51** (q3 = 40 … 149°). J3 limit ±165° removes the folded case mechanically |
| **Wrist** | `sin q5 = 0` → q5 = 0° / 180° | J4 and J6 axes parallel | J4 and J6 turn against each other at unbounded speed; only q4 ± q6 is defined | monitor: `|sin q5| ≥ 0.50`. With the tool vertical q5 = 90° **always**: `|sin q5|` = 1.00, the maximum possible |
| **Shoulder** | `p_r = 0` | wrist point O5 on the cylinder of radius d4 = 0.201 m around the J1 axis | q1 undefined; J1 must turn 180° instantly; inside the cylinder there is no solution at all | monitor: `|p_r| ≥ 0.10 m`. Used: ≥ **0.28 m** (the nearest deck slot is 0.34 m from the J1 axis) |

Near any of them, joint speed for a given TCP speed scales as 1/(factor), so the three factors are
monitored continuously and shown in the HUD.

What an offset wrist changes against a spherical-wrist arm: the shoulder singularity is a
**cylinder of radius d4**, not the J1 axis line, and it is a hole in the workspace; the wrist
singularity is J4 ∥ J6 (not J4 collinear with J6); and a pure rotation of the tool about the TCP
moves all six joints.

### 7.2 Base

| Singularity | Condition | What goes wrong | Rule in this cell |
|---|---|---|---|
| **Non-holonomic constraint** (structural, always) | lateral velocity of the reference point ≡ 0 | a lateral position error at a dock cannot be driven out without backing off and re-approaching | the destination dock is approached on a straight 1.6 m long; the last 1.5 m at creep speed. At the source the arm absorbs the error (§6.4) |
| **Curvature discontinuity** | κ(s) not continuous | yaw acceleration and wheel acceleration jump to infinity at any speed > 0 | the turn uses quintic curvature ramps: κ, κ′, κ″ continuous (§13.2) |
| **Standstill** | v = 0 | κ = ω/v is 0/0: a controller that derives the turn rate from velocities gets noise | κ comes from path geometry; every stop is on a straight piece (κ = 0) |
| **Direction reversal** | v changes sign | none by itself; on a curve the casters flip and disturb the heading | reversals happen only at rest on a straight piece |
| Wheel map | det = 1/b, constant | — | no singular configuration exists |

### 7.3 Mobile manipulator

With the base and the arm interlocked there is no combined singularity to manage. The one that
would exist otherwise: the arm's `x_B`-direction and the base's forward motion are the same DOF
(redundant), while `y_B` is covered by the arm alone — a coordinated controller would have to
resolve that redundancy and would lose `y_B` whenever the arm is at an elbow or shoulder
singularity. Not used here.

---

## 8. Mass and stability (quasi-static)

| Part | Mass | Centre of gravity in `{B}` |
|---|---|---|
| Chassis, drives, battery | 380 kg | (0, 0, 0.17) |
| Conveyor module | 70 kg | (0.04, −0.14, 0.55) |
| Pedestal + arm controller | 55 kg | (0, 0.47, 0.70) |
| Arm | 64 kg | from the joint angles: base 14, upper arm 24, forearm 14, wrist 12 kg at the link midpoints |
| Gripper | 4.5 kg | 0.09 above the TCP |
| Box in the gripper | ≤ 15 kg | under the TCP |
| Layer on the deck | ≤ 96 kg in the recipes (rating 130) | at the slot positions |

| Check | Formula | Result |
|---|---|---|
| Travelling empty: 574 kg, CG (−0.02, 0.02, 0.45) | brake tip `g·(0.60 − |x_cg|)/z_cg`, side tip `g·(0.46 − |y_cg|)/z_cg` | 12.6 / 9.5 m/s² |
| Travelling with the heaviest layer (R1, 96 kg): 670 kg, CG z 0.50 | same | **SF 7.7** vs the 1.5 m/s² emergency stop, **SF 15** vs 0.6 m/s² in a turn |
| Arm at full reach over the pallet with 15 kg, deck empty | CG moves 0.197 m to the arm side | still **0.263 m** inside the tipping edge |
| Worst CG position in the whole run | | ≥ 0.275 m inside the tipping edge |
| Cartons sliding on braked rollers, emergency stop | `μ·g` = 2.94 m/s² | SF 1.96 — and the front stop is raised |
| Cartons sliding in a turn | `μ·g / a_lat` | SF 4.9 — and the side guides |

The arm's own reaction forces while it accelerates are not included; the vehicle stands on its
brakes during all arm motion.

---

## 9. End effector — vacuum gripper

| Parameter | Value |
|---|---|
| Type | 6-cup vacuum pad for closed-top cartons, top pick |
| Mounting | flange face, body axis = flange +Z |
| TCP | (0, 0, 0.160) m in `{F}`, on the cup sealing plane; tool X = long side of the pad |
| Pad footprint | 0.26 × 0.16 m — 20 mm smaller on every side than the smallest box top (0.30 × 0.20), so it never seals on a neighbour |
| Mass | 4.5 kg, centre of mass 0.07 m from the flange |
| Suction cups | 6 × Ø50 mm, 2 × 3 pattern |
| Vacuum level | 60 kPa below ambient |
| Friction cup / carton | μ = 0.5 |
| Actuation DOF | 1 binary (vacuum on / blow-off); no kinematic DOF |
| Grip dwell / release dwell | 0.25 s / 0.15 s |
| Symmetry | 180° about the tool axis |
| Sensors | vacuum switch (level reached), part-present = vacuum held |

**Holding force.** Theoretical: 6 × 60 kPa × π·(0.025 m)² = **707 N**.
Required with the tool vertical: `F = m·(g + a_z + a_h/μ)`.
Static, 15 kg: 147 N → SF 4.8. Horizontal acceleration allowed at SF 2 with 15 kg: 6.9 m/s².
Measured worst case of the run (§15): **SF 2.62** (≥ 2 required).

**Constraints the gripper puts on motion**
- Approach and retreat along the tool axis only (vertical), from / to the swing height.
- Tool axis stays vertical whenever the vacuum is on — the load is carried in suction, not shear.
- No lift before the grip dwell has elapsed; no retreat before the release dwell has elapsed.
- The box is rigid on the TCP while held: box pose = TCP pose × (pose captured at grip).
- A box is picked at the centre of its top face; the 0.26 × 0.16 pad on a 0.60 × 0.40 box leaves
  the box edges unsupported — fine for a rigid carton, to be reviewed for soft or heavy-ended ones.
- Only the top layer can be picked: there is no side grip and no way to pull a box out from
  under another.

---

## 10. Stations, sensors and handshakes

### 10.1 Source: pallet conveyor end

| Parameter | Value |
|---|---|
| Pallet centre at the end stop | (0, **1.25**) in `{W}`, 1200 along X |
| Conveyor | roller conveyor, flow +X, **roller top = pallet underside z = 0.300**, frame 0.96 wide, from x = −3.54 to the end at x = 0.66 |
| Pose scatter of a presented pallet | ±15 mm in x and y, ±1.0° (measured by the arm camera) |
| AMR dock pose | (0, 0, 0°): parallel to the conveyor, 150 mm from its frame |
| J1 axis → pallet | 0.38 m to the near long side, 1.18 m to the far side |
| Depleted pallet | taken away 4 s after the AMR has cleared the station; the next recipe enters 3 s later at x = −2.7 and is conveyed to the end stop at 0.25 m/s. This stands in for the pallet-handling system |

| Tag | Sensor | Where | Signal |
|---|---|---|---|
| **B1** | **Pallet present** — retro-reflective photo-eye across the conveyor | 80 mm behind the leading pallet face, 60 mm above the rollers: through the leading block row, which is solid at three places across the pallet | a pallet is at the end stop, whatever is on it |
| B2 | Pallet against the end stop (proximity switch at the stop) | end stop | the pallet has stopped where the camera expects it |
| S1 | Conveyor section stopped | end section | safe to reach in |
| — | **3D camera on the arm pedestal** | looks down on the pallet | pallet pose (x, y, yaw) and the height of the top layer → pick targets |

**`PalletReady` = B1 ∧ B2 for ≥ 0.5 s ∧ at least one box on the pallet.**

| # | Signal | Direction | Meaning |
|---|---|---|---|
| 1 | `PalletReady`, recipe id | station → AMR | a job exists |
| 2 | `RequestEntry` | AMR → station | AMR is at the pre-dock point |
| 3 | `Locked` | station → AMR | the end position is reserved: the pallet will not be moved or exchanged |
| 4 | `Clear` | AMR → station | arm stowed, AMR back at the pre-dock point; the lock is released |
| 5 | (pallet empty ∧ `Clear`) | station | exchange the pallet |

### 10.2 Destination: roller conveyor end

| Parameter | Value |
|---|---|
| Conveyor | centreline x = **8.00**, receiving end at y = −4.70, flow −Y, 5.0 m long, clear width 0.86, **roller top 0.650**, 0.30 m/s |
| AMR dock pose | (8.14, −3.92, −90°): deck centreline on the conveyor centreline, **30 mm gap** |

| Tag | Sensor | Signal |
|---|---|---|
| B3 | Deck: box at the front stop | the layer is against the stop (travel) |
| B4 | Deck clear (photo-eye across the front edge) | the last box has left the AMR |
| **B5** | Destination infeed clear (1.45 m of free conveyor behind the receiving end) | a layer can be received |

| # | Signal | Direction | Meaning |
|---|---|---|---|
| 1 | `DestReady` (= B5) | conveyor → AMR | checked at the pre-dock point; the infeed is reserved |
| 2 | `Docked` | AMR → conveyor | standstill at the dock pose |
| 3 | front stop down, both conveyors run at the same speed | both | the layer rolls across as one block |
| 4 | B4 ∧ rollers stopped ∧ stop up | AMR | transfer done |
| 5 | `Clear` | AMR → conveyor | AMR back at the pre-dock point; the infeed is released |

### 10.3 Sensors on the AMR that the sequencer relies on

| Sensor | Used for |
|---|---|
| Wheel encoders | odometry (§6.1) |
| Localisation (lidar, natural features) | pose correction; assumed ±10 mm / ±0.5° at a dock |
| Arm joint encoders | q1 … q6 |
| 3D camera | pallet pose and top-layer height after docking |
| Vacuum switch | grip confirmed / box lost |
| B3, B4, stop position switches | deck state |
| Safety scanners front and rear | personnel protection (not simulated) |

---

## 11. Operating cycle

### 11.1 AMR sequence

| # | Phase | Axes moving | What happens |
|---|---|---|---|
| 1 | `WAIT_PALLET` | none | at the source pre-dock point; wait for `PalletReady`, get `Locked` |
| 2 | `DOCK_SRC` | wheels (creep, reverse, straight) | 1.50 m back to the dock pose |
| 3 | `SETTLE` | none | 0.3 s standstill; camera measures the pallet; the order size is fixed |
| 4 | `LOAD` | arm | one pick-and-place cycle per box (§11.2) |
| 5 | `STOW` | arm | to the travel pose |
| 6 | `UNDOCK_SRC` | wheels (creep) | 1.50 m forward; then `Clear` |
| 7 | `TRAVEL` | wheels (loaded limits) | 8.37 m through the turn to the destination pre-dock point |
| 8 | `WAIT_DST` | none | wait for `DestReady` |
| 9 | `DOCK_DST` | wheels (creep, straight) | 1.50 m to the dock pose |
| 10 | `STOP_DOWN` → `TRANSFER` → `STOP_UP` | stop, rollers | the layer rolls onto the destination conveyor |
| 11 | `UNDOCK_DST` | wheels (creep, reverse) | 1.50 m back; then `Clear` |
| 12 | `RETURN` | wheels (reverse, empty limits) | back along the same path, in reverse |

The AMR drives the route forwards loaded and backwards empty; it never turns round.

### 11.2 Arm cycle for one box

| # | Segment | Motion | Detail |
|---|---|---|---|
| 1 | `RISE` / `RETREAT` | LIN, vertical | from where the TCP is (travel pose or the last slot) to the swing height |
| 2 | `SWING TO BOX` | PTP | to the point above the box, at the same height |
| 3 | `APPROACH` | LIN, vertical | down to the box top |
| 4 | `GRIP` | dwell 0.25 s | vacuum on; the box follows the TCP from the end of the dwell |
| 5 | `LIFT` | LIN, vertical | up to the carry height |
| 6 | `SWING TO DECK` | PTP | to the point above the deck slot, at the same height |
| 7 | `SET DOWN` | LIN, vertical | down to the deck |
| 8 | `RELEASE` | dwell 0.15 s | vacuum off; the box now belongs to the deck |

After the last box: `RETREAT` (LIN up), `SWING TO STOW` (PTP), `STOW` (PTP to the travel pose).

Every straight move is vertical and every swing connects two points at one height, so the only
thing a swing can hit is what lies **under** its path.

### 11.3 Swing heights

With `z_grip` = top of the box on the pallet, `z_deck = 0.650 + h` = TCP when the box stands on
the deck:

```
empty swing (pad towards the box):   z ≥ max(z_grip, z_deck) + 0.12
carry swing (box underside = TCP − h):   z ≥ max(z_grip + h + 0.08,  z_deck + h + 0.08)
```

i.e. the pad passes 120 mm over the layer and over the boxes on the deck, and the carried box
passes 80 mm over the layer it came from and over the boxes already on the deck.

**A joint move does not keep the TCP height** — between two points at one height the TCP of this
arm dips by up to ~0.17 m. So each swing is checked before it is started: the joint path is
sampled at 33 points against the clearance model (§14) with planning clearances of 60 mm (arm)
and 60 mm (under the carried box), and its height is **raised in 40 mm steps** until it is free.
In the run the largest raise was 120 mm.

### 11.4 Travel pose

TCP at (−0.40, −0.40, 1.20) in `{B}`, tool down, yaw 0:
**q = (−126.81, −35.00, 92.74, 32.26, 90.00, −36.81)°.** The whole arm is at least 33 mm inside
the chassis outline in plan; the overall height is 2.22 m; the pad is 0.15 m above the tallest
layer the deck can carry.

---

## 12. The safety monitor

Rules M1–M12 are implemented as a monitor that reads **only the state**: joint positions and
speeds, wheel speeds, pose on the path, the vacuum and deck signals, the station handshake
signals and the clearance model (`monitor()` in [js/sim.js](js/sim.js)). It does not know what
the sequencer intends.

| Rule | Hazard | Check |
|---|---|---|
| **M1** | joint over-travel | every arm joint inside §5.2 |
| **M2** | arm overspeed | joint speed ≤ limit, joint acceleration ≤ limit |
| **M3** | base overspeed, load shift | path speed ≤ limit of the state (empty / loaded / creep); reverse ≤ 1.0; wheel ≤ 1.8 m/s; yaw rate ≤ 60°/s; `v²·|κ|` ≤ 0.6 m/s² |
| **M4** | arm and base moving together | arm moving ⇒ base at rest ∧ docked at the source ∧ `Locked`. Base moving ⇒ arm in the travel pose ∧ vacuum off ∧ nothing held |
| **M5** | singularity | while the arm moves or holds a box: `|sin q3| ≥ 0.25`, `|sin q5| ≥ 0.50`, `|p_r| ≥ 0.10 m` |
| **M6** | box peeled off the cups | vacuum on ⇒ tool axis within 0.5° of vertical |
| **M7** | box dropped | box held ⇒ vacuum on ∧ holding-force safety factor ≥ 2, computed from the measured TCP acceleration |
| **M8** | entering a station unannounced | in the source zone only with `Locked`; in the destination zone only with `DestReady` |
| **M9** | layer rolling off | rollers turning ⇒ docked at the destination ∧ at rest ∧ `DestReady`. Front stop not fully up ⇒ at rest ∧ docked at the destination |
| **M10** | collision | arm ≥ 30 mm from everything; carried box ≥ 30 mm above everything under it during a swing; chassis ≥ 50 mm from the conveyors in plan (≥ 20 mm = gap − docking tolerance, in the destination zone) |
| **M11** | overload | box + gripper ≤ 20 kg; deck ≤ 130 kg and ≤ 16 boxes |
| **M12** | tipping | centre of gravity ≥ 0.10 m inside the tipping edge |

A station zone is everything between a dock pose and its pre-dock point (1.50 m).

Next to the monitor the verification counts three things directly from the simulation state
(**raw checks**): steps in which the base moved while the arm was not stowed, steps in which the
arm moved while the base moved, and steps in the source zone without `Locked`.

---

## 13. Cell layout and path

### 13.1 Stations (all coordinates in `{W}`)

| Item | x | y | Heading | Height |
|---|---|---|---|---|
| Source pallet centre | 0.00 | 1.25 | — | pallet underside 0.300 |
| AMR docked at the source | 0.00 | 0.00 | 0° | — |
| Arm J1 axis, docked at the source | 0.00 | 0.47 | — | pedestal top 1.400 |
| Source pre-dock point | 1.50 | 0.00 | 0° | — |
| Corner of the route | 8.14 | 0.00 | — | — |
| Destination pre-dock point | 8.14 | −2.42 | −90° | — |
| AMR docked at the destination | 8.14 | −3.92 | −90° | — |
| Destination conveyor receiving end | 8.00 | −4.70 | — | roller top 0.650 |

### 13.2 The route

One path: straight 5.865 m, a 90° right turn, straight 1.645 m. Length **11.366 m**.

The turn is a **continuous-curvature** curve — quintic ramp, constant arc, quintic ramp
([../amr/js/path.js](../amr/js/path.js)):

| Parameter | Value |
|---|---|
| Radius of the arc `R` | 1.50 (κ_max = 0.667 1/m) |
| Ramp length `L_c` | 1.50 each; `κ(u) = κ_max·(10τ³ − 15τ⁴ + 6τ⁵)`, τ = u / L_c |
| Turn length | 3.856 |
| Tangent length `T` (corner → start / end of the turn) | **2.275** |
| Wheel speeds at κ_max | inner 0.667·v, outer 1.333·v |

Both pre-dock points and both dock poses are on straight pieces, so κ = 0 at every stop and at
every reversal.

---

## 14. Clearance model and budget

**Model.** The arm is eight capsules (shoulder, upper arm, elbow, forearm, wrist 1–3, tool) placed
where the tubes really are (§3.3); everything else is a box that may be turned about Z: chassis,
deck, side guides, pedestal, source conveyor frame, pallet, one block for all complete layers
under the top layer, every box of the top layer, every box on the deck. Distances are sampled
every 20 mm along each capsule. The box in contact with the pad (being approached, held, or just
released) is left out. The carried box is checked downwards: its underside against the top of
whatever is under it.

| Clearance | Nominal | Consumers |
|---|---|---|
| Chassis ↔ source conveyor frame, docked (Y) | 150 mm | docking ±10, heading ±0.5° over 1.5 m = ±13 |
| Chassis rear ↔ source conveyor end at the pre-dock point (X) | 90 mm | — |
| AMR front ↔ destination conveyor end, docked (X) | 30 mm gap | docking ±10 |
| Deck centreline ↔ destination centreline (lateral) | 0 | docking ±10, heading ±0.5°: boxes have 30 mm to each guide |
| Deck ↔ destination roller height | 0 (equal) | floor level: set the destination 5 mm lower in the real cell |
| Pad ↔ neighbouring box (plan) | 20 mm each side on the smallest box | camera ±3, arm repeatability |
| Pad over the layer / the deck boxes, empty swing | ≥ 120 mm planned | swing dip (§11.3) |
| Carried box over the layer / the deck boxes | ≥ 80 mm planned | swing dip; **59 mm** minimum measured |
| Arm ↔ anything | ≥ 60 mm planned; **41 mm** minimum measured (tool over a box on the pallet during a swing) | — |
| Layer ↔ deck stops (X) | 20 mm each end | arm placement, box tolerance |
| Layer ↔ deck guides (Y) | 30 mm each side | — |
| Travel pose: arm inside the chassis outline | 33 mm | — |

---

## 15. Verification results (`npm run verify:mobman`)

3 h of operation simulated at 240 Hz, seed 7:

| Check | Result |
|---|---|
| Zero-pose TCP / upright flange | (−1.591, −0.515, 0.077) / (0, −0.355, 1.986) ✔ |
| DH chain vs closed-form wrist point, 5000 random poses | 7·10⁻¹⁶ m ✔ |
| IK(FK(q)) on all 8 posture branches, ~18 000 random poses | worst 3·10⁻¹⁴ rad, 0 misses ✔ |
| One pose, 8 posture branches | 8 distinct valid solutions ✔ |
| det J factorisation (§7.1) | 9·10⁻¹⁶ ✔ |
| Jacobian vs finite differences | 4·10⁻¹⁰ ✔ |
| Elbow, wrist and shoulder singular poses constructed, det J = 0 | ✔ (shoulder: O5 at radius 0.2010 m) |
| Tool down ⇒ q5 = 90°, q2 + q3 + q4 = 90° | ✔ |
| All 6 recipes fit pallet, height, deck and gripper; no overlaps | ✔ |
| Static reach: 2504 probe poses (every box at every layer with ±15 mm scatter, every deck slot, each at grip and at carry height) | 0 unreachable ✔ |
| Static singularity margins over the task space | `|sin q3|` ≥ 0.575, shoulder ≥ 0.279 m, `|sin q5|` = 1 ✔ |
| Task space | TCP height 0.594 … 2.124 m, radius from the J1 axis 0.344 … 1.190 m |
| Travel pose inside the chassis outline | 33 mm ✔ |
| Wheel speeds ↔ body twist, 20 000 random twists | 2·10⁻¹⁶ ✔ |
| Exact arc pose step vs fine integration | 5·10⁻¹² m ✔ |
| Route ends on the destination dock pose | 7·10⁻¹³ m; length 11.366 m ✔ |
| Lateral velocity of the reference point; curvature continuity | 9·10⁻⁶; no steps ✔ |
| Docking geometry (§13, §14) | ✔ |
| Vacuum: 707 N, static SF 4.80; payload 19.5 of 20 kg | ✔ |
| Stability (§8) | SF 7.7 / 15 / 1.96; 0.263 m ✔ |
| Safety monitor negative tests (24 illegal states, 4 legal) | all classified correctly ✔ |
| Planning or IK failures during the run | 0 ✔ |
| Safety monitor violations during the run | 0 ✔ |
| Arm moving while the base moves / base moving with the arm not stowed (raw checks) | 0 / 0 steps ✔ |
| Source zone without `Locked` (raw check) | 0 steps ✔ |
| Peak joint speed / limit (J1…J6) | 1.00 · 0.96 · 0.67 · 0.60 · 0.00 · 1.00 ✔ |
| Peak joint accel / limit (J1…J6) | 1.00 · 0.95 · 1.00 · 0.59 · 0.00 · 1.00 ✔ |
| Joint range used | J1 −167…115°, J2 −109…11°, J3 40…149°, J4 −35…95°, J5 90°, J6 −274…151° ✔ |
| Singularity margins while the arm moves | elbow 0.508, wrist 1.000, shoulder 0.279 m ✔ |
| Tool tilt with the vacuum on | 2·10⁻⁶ ° ✔ |
| Vacuum safety factor | min 2.62 (TCP peak 2.28 m/s, 4.89 m/s²) ✔ |
| Pick / place position error | 8·10⁻¹⁶ / 1·10⁻¹⁵ m ✔ |
| Arm clearance | min 41 mm ✔ |
| Carried box clearance during swings | min 59 mm; largest swing raise 120 mm ✔ |
| Path speed: forward / reverse / loaded / in a zone | 0.949 / 0.949 / 0.949 / 0.200 m/s ✔ |
| Wheel speed / yaw rate / lateral acceleration | 1.265 of 1.8 m/s / 36.2 of 60°/s / 0.600 of 0.6 m/s² ✔ |
| Path / wheel / yaw acceleration | 0.600 m/s² / 0.781 of 1.2 m/s² / 44.7 of 120°/s² ✔ |
| Dead reckoning from the wheel speeds vs the path pose | worst 0.11 mm over 1.92 km ✔ |
| Deck load | max 96 kg, 16 boxes ✔ |
| Layers taken strictly top-down | ✔ |
| Boxes conserved | 555 created = 17 on the pallet + 4 in transit + 534 delivered ✔ |
| Every recipe served | 9 pallets emptied ✔ |

**538 boxes in 84 trips = 179 boxes / h.** Box cycle 9.6 s mean (6.3 … 14.0); trip 128 s mean
(80 … 255, depending on the order size). Time split: arm loading 50 %, driving 18 %, docking at
creep speed 28 %, transfer 4 %.

Order sizes that occurred (boxes per trip): 2, 3, 4, 5, 7, 8, 9 and 16.

Reference joint solutions (degrees), shoulder − / elbow + / wrist +:

| Pose | TCP in `{W}` | q1 | q2 | q3 | q4 | q5 | q6 |
|---|---|---|---|---|---|---|---|
| Travel | (−0.400, −0.400, 1.200) at the source dock | −126.81 | −35.00 | 92.74 | 32.26 | 90.00 | −36.81 |
| First pick, R1 top layer | (0.439, 1.044, 1.694) | 36.48 | −71.36 | 108.95 | 52.40 | 90.00 | 35.53 |
| First place, R1 front far slot | (0.450, −0.340, 0.900) | −73.47 | −17.92 | 88.42 | 19.50 | 90.00 | 106.53 |

The verification found three real faults while this model was built:
- lifting a box by a fixed 80 mm and then swinging put its underside 172 mm **into** the
  neighbouring boxes of its own layer — the carry height must be a full box height above the layer;
- with both ends of a swing at the same, sufficient height the tool still cut 50 mm into a box,
  because a joint move does not hold the TCP height — hence the checked and raised swings of §11.3;
- leaving the travel pose with a joint move straight to the first box passed 26 mm from the stack —
  hence the vertical `RISE` first.

---

## 16. What the simulation does not model

- Dynamics: no joint torques, link inertia or gearbox compliance, no payload-dependent arm
  limits, no wheel slip, no reaction of the vehicle to the arm's motion. Motion is kinematic and
  bounded by the limits of §5. The stability figures of §8 are quasi-static.
- Localisation and control error: the vehicle is exactly on its path and stops exactly on the
  dock pose. Tolerances appear in the budget (§14) and in the monitor windows, but no noise is
  injected. The only measured disturbance is the pose of the presented pallet.
- The camera is ideal: it returns the true pallet pose. Boxes are rigid, sit exactly in their
  pattern on the pallet, and never shift, crush or stick together.
- Vacuum is ideal (binary), with no leakage, porous-carton or tape-seam model.
- Collision checking is the capsule-and-box model of §14 — not a mesh-level test, and no
  self-collision check between arm links (the fixed posture branch keeps them apart). People and
  other vehicles are not modelled.
- Path planning is the one fixed route of §13.2, not a free-space planner; there is one AMR.
- Of the sensors only B1, B2 (with the debounce), B4, B5, `Locked` and `DestReady` are simulated;
  B3, S1 and the vacuum switch are specified but have no separate model. Sensors are ideal and
  there are no faults; recovery sequences are implied by the rules but never exercised.
- The box-to-box gap when the layer rolls across (roller pitch, belt transitions), the conveyor
  drives' own dynamics, battery and charging.
- Whatever happens to the layer downstream: boxes vanish at the end of the destination conveyor.
- The exchange of a depleted pallet: a timer.

---

## 17. Assumptions to confirm before integration

These were decided here because they were left open; each is one number or one line in
[js/config.js](js/config.js).

| # | Assumption | Where |
|---|---|---|
| 1 | Source pallet conveyor roller height **0.300 m**; pedestal **1.400 m**. They were chosen together from the reach study; changing one needs the other re-checked (`verify:mobman` §4) | `SRC.z`, `ARM.base` |
| 2 | The 1200 side of the pallet faces the AMR and the AMR can park 150 mm beside the conveyor end | `SRC`, §13.1 |
| 3 | Deck and destination conveyor at **0.650 m**, 0.86 m clear width | `DECK.z`, `DST` |
| 4 | Destination 11.4 m of path away, one 90° turn | `DST.x`, `DST.yEnd` |
| 5 | Box range 0.30 × 0.20 × 0.15 … 0.60 × 0.40 × 0.40 m, ≤ 15 kg, closed and rigid top | `BOXLIM` |
| 6 | The six recipes of §4.2 are examples; real recipes go into `RECIPES` in the same format | `RECIPES` |
| 7 | Order quantities per trip `[layer, 2, 3, layer, 4, 5, 8, layer]` | `ORDERS` |
| 8 | Pallet pose scatter ±15 mm, ±1° | `SRC.scatter` |
| 9 | AMR mass 505 kg without the arm; 300 kg class payload | `AMR.parts` |
| 10 | Arm and AMR limits are class-typical, not a product datasheet | `ARM`, `AMR` |
