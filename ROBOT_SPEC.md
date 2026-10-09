# Robot, Gripper and Cell Specification — Palletizing Simulation

This file is the contract for the simulation. Every number here is mirrored in
[js/config.js](js/config.js); the kinematics in [js/kinematics.js](js/kinematics.js)
implement §5–§6 exactly, and [test/verify.mjs](test/verify.mjs) checks them (`npm run verify`,
results in §11).

The robot is a **generic 6-axis articulated arm with a spherical wrist**, sized like the
120 kg / 2.7 m class used for case palletizing. It is not a model of any vendor's product;
link lengths, limits and speeds are representative values for that class.

---

## 1. Conventions

| Item | Convention |
|---|---|
| Units | metres, radians internally (degrees in tables), seconds, kilograms |
| World frame `{W}` | right-handed, **Z up**, origin on the floor on the J1 axis, **+X = robot front** at q1 = 0 |
| Rotations | right-hand rule about the stated axis; `Rx, Ry, Rz` are elementary rotations |
| Joint vector | `q = [q1 … q6]`, all revolute |
| Zero pose | upper arm vertical, forearm horizontal pointing +X, wrist straight (an "L" shape) |
| Flange frame `{F}` | origin at the flange face centre, **+Z out of the flange** (approach axis), +X pointing world −Z at the zero pose |
| Tool frame `{T}` | same orientation as `{F}`, origin at the suction plane (TCP) |
| three.js mapping | everything is built in a group rotated −90° about three's X, so `{W}`(x, y, z) → three (x, z, −y). No other axis conversion exists anywhere. |

---

## 2. Degrees of freedom

| Joint | Name | Type | Axis at zero pose (in `{W}`) | Positive direction | What it does |
|---|---|---|---|---|---|
| J1 | Base / swing | R | +Z | counter-clockwise seen from above | sweeps the whole arm around the column |
| J2 | Shoulder | R | +Y | upper arm leans **forward** (+X) | reach out / pull in |
| J3 | Elbow | R | +Y | forearm pitches **down** | raises / lowers the wrist |
| J4 | Forearm roll | R | +X (along forearm) | right-hand about forearm | rolls the wrist about the forearm |
| J5 | Wrist bend | R | +Y | tool pitches **down** | tilts the tool |
| J6 | Flange roll | R | +X (along tool) | right-hand about approach axis | spins the tool |

J1–J3 position the **wrist centre** (the point where the J4, J5 and J6 axes intersect).
J4–J6 orient the tool about that point. This decoupling is what makes the closed-form
inverse kinematics of §6 possible, and it is also why the arm has exactly three singularity
types (§7).

The gripper adds no kinematic DOF (§9). Redundancy: none — 6 joints for a 6-DOF task. For
this palletizing task the tool always points straight down, so the task itself only needs
4 DOF (x, y, z, yaw); J4 stays at 0 and J5 is fully determined by J2 + J3 (§10.2).

---

## 3. Link geometry

| Symbol | Value [m] | Meaning |
|---|---|---|
| `d1` | 0.675 | floor → J2 axis height |
| `a1` | 0.350 | J1 axis → J2 axis horizontal offset |
| `a2` | 1.150 | J2 → J3 (upper arm) |
| `a3` | 0.041 | J3 axis → J4 axis perpendicular offset |
| `d4` | 1.200 | J3 → wrist centre, along the forearm |
| `d6` | 0.215 | wrist centre → flange face |
| `L3` | 1.2007 | derived: J3 → wrist centre straight-line = √(d4² + a3²) |
| `β` | 88.043° | derived: atan2(d4, a3), angle of that line from the upper-arm axis at q3 = 0 |

### 3.1 Joint chain (what the 3D scene graph implements)

Each joint frame is the parent frame translated by `offset`, then rotated by `q` about `axis`.

| Frame | Parent | Offset in parent [m] | Rotation axis |
|---|---|---|---|
| J1 | world | (0, 0, 0) | Z |
| J2 | J1 | (a1, 0, d1) = (0.350, 0, 0.675) | Y |
| J3 | J2 | (0, 0, a2) = (0, 0, 1.150) | Y |
| J4 | J3 | (0, 0, a3) = (0, 0, 0.041) | X |
| J5 | J4 | (d4, 0, 0) = (1.200, 0, 0) | Y |
| J6 | J5 | (0, 0, 0) | X |
| Flange | J6 | (d6, 0, 0) = (0.215, 0, 0), then fixed `Ry(+90°)` | — |
| TCP | Flange | (0, 0, 0.300) | — |

Zero-pose check: TCP = (a1 + d4 + d6 + 0.300, 0, d1 + a2 + a3) = **(2.065, 0, 1.866)**.

### 3.2 Denavit–Hartenberg table (classic / distal: `Rz(θ)·Tz(d)·Tx(a)·Rx(α)`)

| i | θ | d [m] | a [m] | α |
|---|---|---|---|---|
| 1 | q1 | 0.675 | 0.350 | −90° |
| 2 | q2 − 90° | 0 | 1.150 | 0° |
| 3 | q3 | 0 | 0.041 | −90° |
| 4 | q4 | 1.200 | 0 | +90° |
| 5 | q5 | 0 | 0 | −90° |
| 6 | q6 + 180° | 0.215 | 0 | 0° |

This table reproduces the §3.1 chain to machine precision (1.8·10⁻¹⁵, 5000 random poses).

---

## 4. Joint limits

| Joint | Min | Max | Max speed | Max accel | Notes |
|---|---|---|---|---|---|
| J1 | −185° | +185° | 123°/s | 250°/s² | 10° overlap at the back |
| J2 | −60° | +85° | 115°/s | 250°/s² | 0° = upper arm vertical |
| J3 | −80° | +150° | 112°/s | 250°/s² | 0° = forearm ⟂ upper arm; lower limit keeps 8° away from the stretched elbow singularity |
| J4 | −350° | +350° | 179°/s | 450°/s² | |
| J5 | −125° | +125° | 172°/s | 450°/s² | 0° is the wrist singularity |
| J6 | −350° | +350° | 219°/s | 600°/s² | |

Cartesian limits for linear moves: 1.5 m/s, 3.5 m/s². Rated payload 120 kg at the flange;
the application carries 34 kg (22 kg gripper + 12 kg box).

Coupling constraints that real arms of this type have and that the task respects with margin:
J2 + J3 must keep the forearm clear of the upper arm and base; here J3 never exceeds +52°.

---

## 5. Forward kinematics

With `q23 = q2 + q3`:

```
ρ   = a1 + a2·sin q2 + d4·cos q23 + a3·sin q23        (wrist-centre radius from the J1 axis)
z_w = d1 + a2·cos q2 − d4·sin q23 + a3·cos q23
p_w = ( ρ·cos q1, ρ·sin q1, z_w )                     (wrist centre)

R_a = Rz(q1) · Ry(q23) · Rx(q4) · Ry(q5) · Rx(q6)     (X axis of R_a = approach direction)
R_F = R_a · Ry(+90°)                                  (flange orientation, Z = approach)

p_flange = p_w + d6 · R_a·x̂
p_TCP    = p_w + (d6 + 0.300) · R_a·x̂
```

---

## 6. Inverse kinematics (closed form)

Input: TCP position `p`, flange orientation `R_F`, a posture branch, and a reference joint
vector (to choose the 2π-turn of J1, J4, J6).

1. `R_a = R_F · Ry(−90°)`; wrist centre `p_w = p − (d6 + 0.300) · R_a·x̂`.
2. **J1**: `q1 = atan2(p_w.y, p_w.x)` (front) or `+180°` (back).
3. In the arm plane: `r = ±√(p_w.x² + p_w.y²) − a1` (+ front, − back), `s = p_w.z − d1`.
4. **J3**: `cos γ = (r² + s² − a2² − L3²) / (2·a2·L3)`. `|cos γ| > 1` ⇒ out of reach.
   `γ = +acos(…)` (elbow up) or `−acos(…)` (elbow down); `q3 = γ − β`.
5. **J2**: `q2 = atan2(r, s) − atan2(L3·sin γ, a2 + L3·cos γ)`.
6. **Wrist**: `M = (Rz(q1)·Ry(q2+q3))ᵀ · R_a = Rx(q4)·Ry(q5)·Rx(q6)`, so
   - no-flip: `q5 = atan2(√(M10² + M20²), M00)`, `q4 = atan2(M10, −M20)`, `q6 = atan2(M01, M02)`
   - flip: `q5 → −q5`, `q4 = atan2(−M10, M20)`, `q6 = atan2(−M01, −M02)`
7. Unwrap J1, J4, J6 to the turn nearest the reference, then reject the solution if any joint
   is outside §4.

**Eight posture branches** = {front, back} × {elbow up, elbow down} × {no-flip, flip}.
A branch can only be changed by passing through the corresponding singularity, so a
Cartesian path must stay in one branch. **The simulation uses front / elbow-up / no-flip
for everything**, which gives q4 = 0 and q5 > 0 for a downward-pointing tool.

---

## 7. Singularities

The Jacobian determinant (wrist-centre velocity + angular velocity) factorises as

```
|det J| = a2 · L3 · |sin(q3 + β)| · |ρ| · |sin q5|
```

(verified numerically against a finite-difference Jacobian, deviation 5·10⁻⁹). Each factor is
one singularity:

| Singularity | Condition | Where it is | What goes wrong | Rule in this cell |
|---|---|---|---|---|
| **Wrist** | `sin q5 = 0` → q5 = 0° | J4 and J6 axes collinear | J4 and J6 spin ±180° at unbounded speed for a small tool motion; only q4 + q6 is defined | keep \|q5\| ≥ 15°. With the tool vertical q5 = 90° − (q2 + q3), so the forearm must never point straight down (q2 + q3 ≠ 90°) |
| **Elbow (stretched)** | `sin(q3 + β) = 0` → q3 = −88.04° | arm fully extended, wrist on the reach boundary | no velocity possible along the arm; J2/J3 speeds blow up near the boundary | J3 lower limit −80° removes it; usable reach 2.345 m from J2 instead of 2.351 m |
| **Elbow (folded)** | q3 = +91.96° | forearm folded back onto the upper arm | same loss of radial motion | inside the mechanical range — Cartesian moves must keep q3 ≤ +85° |
| **Shoulder** | `ρ = 0` | wrist centre on the J1 axis (above the column) | q1 undefined; J1 must turn 180° instantly | keep ρ ≥ 0.30 m; never route a Cartesian path over the base |

Near any of them, joint speed for a given TCP speed scales as 1/(factor), so the margins are
monitored continuously in the simulation HUD.

Margins actually used by the task (§11): min \|sin q5\| = 0.62 (q5 ≥ 38°), min \|sin(q3 + β)\| = 0.64,
min ρ = 1.12 m.

---

## 8. Workspace

| Quantity | Value |
|---|---|
| Wrist-centre reach from the J2 axis | 0.051 m (folded) … 2.345 m (at the J3 limit) |
| Max horizontal wrist-centre radius from J1 (at shoulder height) | 2.695 m |
| TCP with tool pointing down | wrist-centre envelope shifted 0.515 m down (d6 + TCP length) |

Everything the task touches lies between 1.12 m and 2.01 m wrist-centre radius.

---

## 9. End effector — vacuum gripper

| Parameter | Value |
|---|---|
| Type | 6-cup vacuum gripper for closed-top cartons |
| Mounting | flange face, body axis = flange +Z |
| TCP | (0, 0, 0.300) m in `{F}`, on the cup sealing plane; tool X = long side of the cup plate |
| Cup plate footprint | 0.36 × 0.26 m |
| Mass | 22 kg, centre of mass ≈ 0.15 m from the flange on the tool axis |
| Suction cups | 6 × Ø60 mm, 2 × 3 pattern |
| Vacuum level | 60 kPa below ambient |
| Actuation DOF | 1 binary (vacuum on / blow-off); no kinematic DOF |
| Grip dwell | 0.20 s (vacuum build-up before lifting) |
| Release dwell | 0.15 s (blow-off before retracting) |
| Symmetry | 180° about the tool axis → two equivalent yaw angles for every pick/place |

**Holding force.** Theoretical: 6 × 60 kPa × π·(0.03 m)² = **1018 N**.
Required with friction μ = 0.5 between cup and carton:
`F = m·(g + a_z + a_h/μ)`. With the measured worst case of the run
(a_h = 9.2 m/s² during the J1 swing, a_z = 3.5 m/s²) and m = 12 kg: 380 N →
**safety factor 2.7** (≥ 2 required for a horizontal grip face).

**Constraints the gripper puts on motion**
- Approach and retract along the tool axis only (vertical) for the last 0.15 m at pick and 0.35 m at place.
- Tool axis stays vertical whenever a box is held — the load is carried in suction, not shear.
- No lift before the grip dwell has elapsed; no retract before the release dwell has elapsed.
- The box is rigidly attached to the TCP while held: box pose = TCP pose × (pose captured at grip).

---

## 10. Cell layout and task

All coordinates in `{W}`.

| Element | Definition |
|---|---|
| Robot | base on the floor at the origin, front = +X |
| Conveyor | centre line x = 1.75, belt top z = 0.75, width 0.60, runs in **+Y** from y = −3.8 to y = +0.9 at **0.25 m/s** (never stops) |
| Tracking window | y = −0.8 … +0.6: the TCP may only follow the belt inside this range |
| Carton | 0.295 × 0.395 × 0.250 m, 12 kg, arrives with the long side along the belt, one every 8 s |
| Pallet | EUR 1200 × 800 × 144 mm, centre (0.0, 1.55), long side along Y |
| Pattern | 2 × 4 per layer on a 0.4 × 0.3 m pitch (5 mm gaps), long side along X → carton is rotated 90° between pick and place; 4 layers = 32 cartons, stack top z = 1.144 |
| Fill order | per layer: far row (y = 2.00) first, far column (x = −0.2) first, so nothing already placed is reached over |

### 10.1 Motion cycle

| # | Phase | Motion type | Detail |
|---|---|---|---|
| 1 | Intercept | PTP (joint) | to a hover point 0.15 m above the predicted box position, 0.05 m downstream of it; arrival time solved by fixed-point iteration on the PTP duration |
| 2 | Synchronise | LIN in belt frame | accelerate from rest to belt speed in 0.40 s; ends exactly above the box |
| 3 | Descend | LIN in belt frame | 0.15 m down while riding with the box |
| 4 | Grip | dwell in belt frame | vacuum on, 0.20 s |
| 5 | Lift | LIN in belt frame | up to the safe height for the layer being built (≥ 0.30 m) |
| 6 | De-synchronise | LIN | decelerate from belt speed to rest in 0.40 s |
| 7 | Transfer | PTP (joint) | to the place hover point, 0.35 m above the final box top |
| 8 | Place | LIN | 0.35 m straight down |
| 9 | Release | dwell | vacuum off, 0.15 s |
| 10 | Retract | LIN | 0.35 m straight up, then next intercept or ready pose |

### 10.2 Motion rules

- **PTP**: all six joints share one quintic time law `s(τ) = 10τ³ − 15τ⁴ + 6τ⁵` (zero velocity
  and acceleration at both ends). Duration = the slowest joint:
  `T = max_i( 1.875·|Δq_i| / v_i , √(5.7735·|Δq_i| / a_i) )`.
- **LIN**: same quintic law on path length with the Cartesian limits of §4; joints come from IK
  at every control step in the fixed posture branch.
- **PTP keeps the tool vertical for free**: both ends have q4 = 0 and q2 + q3 + q5 = 90°, and
  all joints use the same `s(τ)`, so the sum stays 90° along the whole move.
- **Safe height**: PTP moves start and end with the TCP at least 0.35 m above the top of the
  layer being built, i.e. the carried carton clears every placed carton by ≥ 0.10 m.
- **Missed box**: if a box can no longer be tracked to completion inside the window it is
  skipped, runs off the belt end and a replacement is requested.

---

## 11. Verification results (`npm run verify`)

Two full pallets (64 cartons, 531 s simulated, 240 Hz):

| Check | Result |
|---|---|
| Zero-pose TCP | (2.065, 0.000, 1.866) ✔ |
| IK(FK(q)) over ~19 000 random poses, all 8 branches | worst error 1·10⁻¹² ✔ |
| det J factorisation (§7) | deviation 5·10⁻⁹ ✔ |
| IK failures during the run | 0 ✔ |
| Missed cartons | 0 ✔ |
| Peak joint speed / limit (J1…J6) | 1.00 · 0.38 · 0.62 · 0.00 · 0.35 · 0.92 ✔ |
| Peak joint accel / limit (J1…J6) | 1.00 · 0.77 · 0.92 · 0.00 · 0.57 · 0.66 ✔ |
| Joint range used | J1 −25…100°, J2 −22…36°, J3 −38…52°, J4 0°, J5 38…102°, J6 −84…100° ✔ |
| Singularity margins | wrist 0.62, elbow 0.64, shoulder 1.12 m ✔ |
| Min clearance, carried carton and forearm vs stack and conveyor | 100 mm ✔ |
| Vacuum safety factor | 2.68 ✔ |
| Cycle time | 8.0 s mean (limited by carton supply, not by the robot) |

Reference joint solutions (degrees), front / elbow-up / no-flip:

| Pose | q1 | q2 | q3 | q4 | q5 | q6 |
|---|---|---|---|---|---|---|
| Pick, box at y = 0 (TCP 1.75, 0, 1.000) | 0.00 | 11.73 | 4.00 | 0.00 | 74.26 | 90.00 |
| Ready hover (TCP 1.75, −0.2, 1.150) | −6.52 | 10.97 | −2.36 | 0.00 | 81.40 | 83.48 |
| Slot 0, layer 1 far corner (TCP −0.2, 2.0, 0.394) | 95.71 | 36.24 | 1.00 | 0.00 | 52.76 | −84.29 |
| Slot 7, layer 1 near corner (TCP 0.2, 1.1, 0.394) | 79.70 | −0.41 | 52.09 | 0.00 | 38.33 | −100.30 |
| Slot 24, layer 4 far corner (TCP −0.2, 2.0, 1.144) | 95.71 | 23.65 | −18.38 | 0.00 | 84.73 | −84.29 |
| Slot 31, layer 4 near corner (TCP 0.2, 1.1, 1.144) | 79.70 | −21.95 | 27.85 | 0.00 | 84.10 | −100.30 |

---

## 12. What the simulation does not model

- Dynamics: no torques, inertia, gearbox compliance or payload-dependent limits — motion is
  kinematic and bounded only by the speed/acceleration limits of §4.
- Collision checking is limited to the carried carton and forearm against the stack and the
  conveyor body; there is no general self-collision or link-vs-cell check.
- Vacuum is ideal (binary), with no leakage or porous-carton model.
- Tracking assumes a perfectly known, constant belt speed (no encoder noise or latency).
