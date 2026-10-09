# Belly Wrapper Specification — Turntable Stretch Wrapper, Film Carriage, Clamp-Cut-Press Unit ("Gripper"), Conveyor Line

This file is the contract for the simulation. Every number here is mirrored in
[js/config.js](js/config.js). [js/kinematics.js](js/kinematics.js) implements §5, §7.3, §8.2 and
§9, [js/sim.js](js/sim.js) implements §4.2, §5.4, §6 and §8, and
[test/verify.mjs](test/verify.mjs) checks all of it (`npm run verify:wrapper`, results in §12).

The system is **one inline turntable stretch wrapper**: a pallet of boxes arrives on a running
roller conveyor, is transferred onto a roller deck that sits on a turntable, is rotated while a
film carriage travels up and down a mast and winds pre-stretched film round its "belly" (the
four vertical faces), and leaves on the outfeed conveyor. The film is started and ended without
an operator by a clamp on the turntable and a cut-and-press slide beside it.

It is not a model of any vendor's machine; dimensions, speeds and forces are representative
values for a 1000 kg, 12 rpm, 500 mm-film automatic turntable wrapper.

**Design decisions behind the numbers** (change them here first if your hardware differs):

| Decision | Chosen | Why |
|---|---|---|
| Architecture | turntable: the **load rotates**, the film source only moves vertically | chosen by the user. Two servo axes do the whole wrapping task; the price is that the unwrapped load is spun (§7.3) |
| Pallet transfer | powered roller deck **on** the turntable, in line with the conveyors | the pallet never leaves rollers; costs one interlock: the deck only lines up at one table angle (§9.1) |
| Pallet orientation | EUR 1200 × 800, **lengthwise** | the bottom boards run along the travel direction, across the rollers |
| Rotation | one direction only, counter-clockwise from above, a **whole number of revolutions per cycle** | the film can only be wound on; the cycle must end where the clamp can reach the film |
| Film start / end | clamp on the turntable + hot wire and brush on a fixed slide | the only place where film, clamp and cutter can meet is the home pose (§7) |
| Film delivery | powered pre-stretch 200 %, dancer-controlled | the load is a rectangle, so the film speed it demands changes by 1.9 : 1 four times per revolution (§5.2) |
| Load height | light grid at the infeed stop | the carriage needs its top target before the cycle starts, because the cycle is planned as a whole (§8.2) |
| Top platen | not fitted | the loads of §3.5 are stable without it (§7.3); the conditions that would require one are stated there |

---

## 1. Conventions

| Item | Convention |
|---|---|
| Units | metres, radians internally (degrees and rpm in tables), seconds, kilograms, newtons |
| World frame `{W}` | right-handed, **Z up**, origin on the floor **on the turntable axis**. Pallets travel in **+X**. The mast stands on the **−Y** side |
| Table frame `{T}` | `{W}` rotated by `q1` about +Z. Equal to `{W}` at home |
| Pallet frame `{P}` | origin = centre of the pallet underside, `X_p` = the 1200 length, `Y_p` = the 800 width. On the turntable `{P}` = `{T}` shifted by the pallet's centring error `(dx, ey)` |
| Home pose | `q1 ≡ 0 (mod 2π)` at standstill, carriage at `Z_BOTTOM`, clamp up and closed on the film, cut slide parked |
| Joint vector | `q = [q1 turntable, q2 deck conveyor, q3 carriage lift, q4 film feed, q5 dancer]`, gripper `g = [g1 clamp lift, g2 clamp jaw, g3 cut slide]` |
| Film line | the straight, taut film from the exit roller to the load; direction angle `α` in `{W}`, measured from +X |
| Moment arm `p` | perpendicular distance from the turntable axis to the film line |
| "Load height" `H` | pallet **included** (0.144 + boxes) |
| three.js mapping | everything is built in a group rotated −90° about three's X, so `{W}`(x, y, z) → three (x, z, −y). No other axis conversion exists anywhere |

---

## 2. Degrees of freedom

| Joint | Name | Type | Axis | Positive direction | Actuation | What it does |
|---|---|---|---|---|---|---|
| q1 | Turntable | R, **continuous** | +Z through the origin | counter-clockwise seen from above | servo, encoder + home switch | turns the load; winds the film on |
| q2 | Deck conveyor | P-equivalent, continuous | `X_T` (rollers turn about `Y_T`) | pallet moves towards +X | geared motor on the turntable, fed through a slip ring | carries the pallet on and off the turntable |
| q3 | Carriage lift | P | +Z along the mast | up | servo, belt | sets the height of the film web |
| q4 | Film feed (pre-stretch) | R, continuous | roller axes ∥ Z | film paid out | servo | meters the film; two rollers geared 3 : 1 stretch it 200 % |
| q5 | Dancer | R, **passive**, sprung | ∥ Z, on the carriage | film stored | none — it is the tension sensor | absorbs the difference between film paid out and film taken |
| g1 | Clamp lift | P, 2 positions | +Z, on the turntable | up | pneumatic, through a rotary union | raises the jaws into the film web |
| g2 | Clamp jaw | P, 2 positions | normal to the film line at home | closing | pneumatic | holds the film tail |
| g3 | Cut slide | P | +Y, on a fixed stand | towards the load | pneumatic / belt axis | presses the film on the load with a brush and cuts it with a hot wire |

**5 machine joints + 3 gripper joints.** What that means for everything downstream:

- The wrapping task is to put film on a surface, so the task space is the **lateral surface of
  the load**, parametrised by perimeter position `s` and height `z`. It has 2 DOF and is driven
  by exactly two joints: `q1` (→ `s`) and `q3` (→ `z`). **The machine cannot reach the top or
  the underside of the load**, and it cannot choose the film's angle independently of its speed
  along the surface — that angle is the ratio of the two joint speeds (§5.5).
- `q1` is unbounded. Nothing winds up: air goes through a rotary union and power and signals
  for `q2`, `g1`, `g2` and the deck photo-eye go through a slip ring.
- `q2` and `q1` are **mutually exclusive**: the deck is only connected to the line at one table
  angle (§9.1).
- `q4` has no kinematic freedom of its own: over a revolution it must deliver exactly the
  perimeter of the load (§5.2). `q5` is what makes that possible at every instant.
- The film is a unilateral constraint: it can only pull. `q1` never reverses, and `q3` must not
  move unless `q1` does (§9.1).
- Redundancy: none. For a given point `(s, z)` and wrap count there is exactly one `(q1, q3)`.

---

## 3. Geometry

### 3.1 Conveyor line

| Parameter | Value |
|---|---|
| Zones | `I3, I2, I1, T, O1, O2, O3` at x = −5.25, −3.50, −1.75, **0**, +1.75, +3.50, +5.25 (pitch **1.75**) |
| Top of rollers = pallet underside | z = **0.500** |
| Line conveyors | x = −7.90 … −0.95 and +0.95 … +7.90, frame 1.10 wide |
| Rollers | Ø80, pitch 0.16, 0.95 long |
| Gap between the last line roller and the first deck roller | 0.15 centre to centre (bridged by the 1.2 m bottom boards) |
| Side guides | pallet centre within **±15 mm** of the line axis |
| Stop accuracy | **±10 mm** along X; a pallet keeps its error from zone to zone |

### 3.2 Turntable

| Parameter | Symbol | Value |
|---|---|---|
| Axis | | Z through the origin |
| Plate radius / largest rotating radius | `R_T` | 0.90 / **0.92** |
| Plate top / roller top | | 0.40 / 0.50 |
| Deck rollers | | 11 rollers at `x_T` = −0.80 … +0.80, Ø80, ≤ 0.95 long (shortened to the plate circle) |
| Rotating mass / inertia without load | | 320 kg / 110 kg·m² |
| Load inertia at rated load | | 1000 kg · (1.2² + 0.8²) / 12 = 173 kg·m² |

### 3.3 Mast and film carriage

| Parameter | Value |
|---|---|
| Exit roller axis in `{W}` (`E`) | **(0, −1.10)**, radius `r_e` = 0.04 |
| `q3` | height of the **centreline** of the film web above the floor |
| Carriage housing, plan view | x = −0.46 … +0.07, y = −1.38 … −1.03, 0.62 high, centred on `q3` |
| Mast | 0.20 × 0.24 column at (−0.20, −1.52), 2.95 high |
| Film roll | 500 mm wide, 23 µm, 16 kg → **1512 m**, Ø227 new on a Ø86 core |
| Web width on the load after neck-down (`w`) | **0.450** |
| Pre-stretch ratio (`λ`) | 3.0 (200 %): 1 m off the roll becomes 3 m on the load |
| Film direction window at the exit roller | α = 30° … 80° (used: 43.0° … 67.3°) |

### 3.4 Kinematic chain (what the 3D scene graph implements)

Each frame is its parent translated by `offset`, then moved by its joint.

| Frame | Parent | Offset in parent [m] | Joint motion |
|---|---|---|---|
| Table `{T}` | world | (0, 0, 0) | `Rz(q1)` |
| Deck roller *i* | table | (−0.80 + 0.16·i, 0, 0.46) | `Ry(q2 / 0.04)` |
| Pallet on the deck `{P}` | table | (dx, ey, 0.50) | — (rigid while `q2` is stopped) |
| Clamp lift | table | (0.330, −0.689, 0.544), yawed 46.9° | `Tz(0.140 · g1)` |
| Clamp jaws | clamp lift | ±0.020 across the film line | close to 0 with `g2` |
| Carriage | world (mast) | (0, −1.10, 0) | `Tz(q3)` |
| Exit roller | carriage | (0, 0, 0) | turns with the film |
| Pre-stretch rollers, dancer, film roll | carriage | inside the housing | `q4`, `q4 / λ`, `q5` |
| Cut slide | world (stand) | (0.48, 0, 0.79) | `Ty(g3)`; `g3` = y of the brush face |

Home-pose check: film line from `E` to the load corner (+0.600, −0.400): tangent point on the
roller (−0.0292, −1.0727), direction **α = 46.91°**, free length 0.921 m, moment arm 0.711 m.
The clamp at (0.330, −0.689) is 0.24 mm from that line.

### 3.5 Pallet and loads

| Parameter | Value |
|---|---|
| Pallet | EUR 1.20 × 0.80 × 0.144, 25 kg |
| Load limits | height ≤ 2.00 incl. pallet, mass ≤ 1000 kg, flush with the pallet (no overhang) |
| Friction box / box and box / pallet (`μ`) | 0.30 |
| Corner radius of the footprint (`R_c`) | √(0.6² + 0.4²) = **0.7211** |
| Perimeter | **4.000** |

Loads in the simulation are column-stacked full layers of one box type:

| SKU | Box (x_p × y_p × z) | Mass | Per layer | Layers | Height incl. pallet | Mass |
|---|---|---|---|---|---|---|
| BX-3040 | 0.300 × 0.400 × 0.250 | 9 kg | 4 × 2 | 3 … 6 | 0.894 … 1.644 | 241 … 457 kg |
| BX-6040 | 0.600 × 0.400 × 0.300 | 16 kg | 2 × 2 | 2 … 5 | 0.744 … 1.644 | 153 … 345 kg |
| BX-4027 | 0.400 × 0.267 × 0.200 | 6 kg | 3 × 3 | 4 … 8 | 0.944 … 1.744 | 241 … 457 kg |

Box centre on its pallet for column `i`, row `j`, layer `k` (from 0):
`(lx, ly, lz) = ((i − (nx−1)/2)·sx, (j − (ny−1)/2)·sy, 0.144 + (k + 0.5)·sz)`.

---

## 4. Joint limits and motion law

### 4.1 Limits

| Joint | Min | Max | Max speed | Max accel | Max jerk | Tolerance |
|---|---|---|---|---|---|---|
| q1 Turntable | — | — | **12 rpm** (1.257 rad/s) | 0.8 rad/s² | 2.0 rad/s³ | home ±0.5° |
| q2 Deck / line conveyors | — | — | 0.30 m/s | 0.40 m/s² | 1.5 m/s³ | stop ±10 mm |
| q3 Carriage, positioning | 0.700 | 2.450 | 0.40 m/s | 0.80 m/s² | 4.0 m/s³ | ±2 mm |
| q3 Carriage, spiral up (50 % overlap) | | | **0.0450 m/s** | 0.80 m/s² | 4.0 m/s³ | |
| q3 Carriage, spiral down (35 % overlap) | | | **0.0585 m/s** | 0.80 m/s² | 4.0 m/s³ | |
| q4 Film feed (surface speed after stretch) | 0 | — | 1.5 m/s | 4.0 m/s² | — | drive time constant 30 ms |
| q5 Dancer (film stored) | −60 mm | +60 mm | — | — | — | spring 300 N per metre stored |
| g1 Clamp lift | 0 (jaw tips at z = 0.544) | 1 (0.684) | stroke in 0.30 s | | | |
| g2 Clamp jaw | 0 (open 40 mm) | 1 (closed) | stroke in 0.15 s | | | |
| g3 Cut slide | −1.300 (park) | −0.400 (on the load face) | 0.80 m/s | 4.0 m/s² | 40 m/s³ | brush compliance ±20 mm |

`q1` is never commanded negative. Range actually used by the task: carriage 0.789 … 2.119,
film speed ≤ 0.95 m/s, dancer ±19 mm.

Film tension at the exit roller: set-point **60 N**, allowed 25 … 110 N.

### 4.2 Motion law

Every servo move — the turntable, a carriage stroke, a pallet transfer, the cut slide — is a
**jerk-limited, rest-to-rest 7-segment S-curve** ([../asrs/js/motion.js](../asrs/js/motion.js),
the same law as the ASRS cranes and the AMR).

- The turntable makes **one** move per pallet, `0 → 2π·N` with `N` a whole number (§8.2). It
  therefore stops on the home angle by construction, not by searching for it.
- The two carriage strokes are started at fixed times of that move and run on their own
  S-curve; while they cruise, `q1` is cruising too, so the pitch of the helix is constant.
- A pallet transfer is one pitch (1.75 m) for every pallet involved, on identical profiles, so
  two pallets that move together keep their distance.
- The pneumatic strokes `g1`, `g2` run at constant speed between their two end positions.

Typical times: spin-up 1.97 s (70.9° of table angle); pallet transfer 6.86 s; clamp up + close
0.45 s; cut slide stroke 1.43 s.

---

## 5. Kinematics

### 5.1 Film line

The film leaves the exit roller (centre `E`, radius `r_e`) and runs straight to the **active
anchor** `C` — the load corner (or, at the start, the clamp) it was last wrapped round. It
passes the roller on the load side, so the roller centre is on the right of the directed line.

```
v = C − E,   L = |v|
α = atan2(v_y, v_x) − asin(r_e / L)            film direction
u = (cos α, sin α)
T = E + r_e · (−sin α, cos α)                   tangent point on the roller
ℓ = √(L² − r_e²)                                free film length
p = C_x · u_y − C_y · u_x                       moment arm about the turntable axis
```

`C` is given in `{T}` and rotates with the table: `C_W = Rz(q1) · (C_P + (dx, ey))`.

### 5.2 Moment arm and film demand

The film is taut and inextensible between roller and load, so the speed at which the load
takes film is the speed of the contact point along the film line:

```
v_film = ω · p(q1)            ω = q̇1
torque of the film on the turntable = F · p
```

This is the central relation of the machine. `p` is **not constant**, because the load is a
rectangle and not a cylinder:

- While the film pivots about one corner, `p` changes smoothly.
- When the film line reaches the next corner (it then lies flat on a face), that corner
  becomes the active anchor. `p` is **continuous** there — the line is the same line — but its
  slope `dp/dq1` jumps. This happens **4 times per revolution**.
- With a centred load `p` runs between the half width **0.400** (film flat on a long face) and
  the corner radius **0.721**. With the centring tolerances: 0.385 … 0.738 → the film speed
  varies **1.92 : 1** at constant `ω`.
- Over one revolution the load takes exactly its perimeter: `∮ p dq1 = 4.000 m`, so the mean
  arm is `p̄ = perimeter / 2π = 0.6366 m` and the mean film speed at 12 rpm is 0.80 m/s.

Order of the corners. The contact zone is fixed in `{W}` (it is where the film arrives), the
load turns counter-clockwise through it, so in `{P}` the anchors are visited **clockwise**:
(−0.6, −0.4) → (−0.6, +0.4) → (+0.6, +0.4) → (+0.6, −0.4) → …  For a centred load the
hand-overs are at `q1` = 74.2°, 144.4°, 246.4°, 324.4°, then 426.4° … (the first follows the
clamp, the rest repeat with period 360°).

### 5.3 Film on the load: where the web actually lands

Film does not touch the load continuously. While it pivots about a corner **nothing new is
laid**; at the hand-over a whole face length lands at once, as a straight line from the old
anchor (height `z_k`, where the film has been stuck since it landed) towards the exit roller
(height `q3` now):

```
z_{k+1} = z_k + (q3 − z_k) · L_face / ℓ⁻         ℓ⁻ = free length just before the hand-over
```

Consequences:

- The film on each face is a **straight, slightly inclined band**, not a helix arc.
- The height of the contact point **lags** the carriage (by up to 120 mm on the way up or
  down). The top and bottom wraps of §8 exist to let it catch up; without them the top of the
  load would stay bare although the carriage had been there.
- Coverage is therefore checked on the laid bands, not on the carriage trajectory (§12).

### 5.4 Film feed and dancer

```
Film balance:    ṡ_d = v_feed − ω·p           s_d = film stored in the dancer (0 = mid position)
Tension:         F = F_set − k_d · s_d
Feed command:    v* = ω·p̄ − K_p·s_d − K_i·∫ s_d dt       (K_p = 12 /s, K_i = 20 /s²)
Drive:           v̇_feed = sat((v* − v_feed) / τ, ±4 m/s²),   v_feed ≥ 0
Roll side:       v_roll = v_feed / λ,   roll radius = √(r_core² + remaining · t / π)
```

The controller does not know `p(q1)`; it feeds forward the mean arm and lets the dancer
measure the rest. The dancer has to be large enough for the 1.92 : 1 speed variation: it uses
±19 mm of ±60 mm and the tension stays within 54 … 65 N.

### 5.5 Helix: pitch and overlap

```
pitch per revolution   P = 2π · q̇3 / ω
overlap                o = 1 − P / w
film angle on a face   γ = atan(q̇3 / (ω · p))        (0.045 / 0.80 → 3.2° on the way up)
```

`P` and `o` are set by the **ratio** of two joint speeds. If `ω` drops while `q̇3` does not,
the overlap opens; at `ω = 0` the "helix" is a vertical stripe. That is why the carriage is
only allowed to move at full turntable speed (rule R5).

### 5.6 Task Jacobian

```
| ṡ   |   | p(q1)  0 |   | q̇1 |
| ż_c | = |   0    1 | · | q̇3 |          det = p(q1)
```

`ṡ` = speed of film laid along the perimeter, `ż_c` = speed of the web centre. The Jacobian
is diagonal — the two axes are fully decoupled — and `det = p ≥ 0.385 > 0` for every load the
centring tolerance allows. See §9.1 for what would make it vanish.

---

## 6. Sensors and handshake

| Tag | Sensor | Where | Signal |
|---|---|---|---|
| **B1** | Pallet at the infeed stop (photo-eye across the conveyor) | zone I1, x = −1.75 | a pallet is waiting. `Ready` = B1 for ≥ 0.5 s with the zone stopped |
| **BH** | Load height light grid, 10 mm resolution | zone I1 | `H` of the waiting pallet, rounded up |
| **B2** | Gap eye, infeed side | across the gap at x = −0.95 … −0.84 | something bridges the line conveyor and the deck |
| **B3** | Pallet centred on the deck | on the turntable, diagonal across the deck | deck stop position reached |
| **B4** | Gap eye, outfeed side | x = +0.84 … +0.95 | as B2 |
| **B5** | First outfeed zone occupied | zone O1 | no room to discharge |
| S0 | Turntable home switch + encoder | under the plate | `q1 ≡ 0` |
| S3 | Carriage encoder, end switches | mast | `q3` |
| S5 | Dancer angle | carriage | `s_d`; at either stop → film break or film jam |
| SF | Film roll low | carriage | fewer than 30 m (unstretched) left |
| SG | Clamp up / down, jaw open / closed, slide parked / pressed | reed switches | gripper state |

Handshake with the line:

| # | Condition | Action |
|---|---|---|
| 1 | `Ready` ∧ deck empty ∧ home ∧ roll not low | transfer in: I1 → T |
| 2 | B3 ∧ ¬B2 ∧ ¬B4 ∧ deck stopped | wrap cycle may start |
| 3 | cycle finished ∧ ¬B5 | transfer out: T → O1; if `Ready`, the next pallet follows in the same move |
| 4 | a line zone is released when its pallet starts to leave | the pallet behind moves up |
| 5 | roll low ∧ deck empty | no transfer in; roll change (90 s) |

Back-pressure: with the deck occupied the infeed accumulates in I1 … I3; with I3 occupied no
pallet is released into the line. Pallets arrive every 60 … 120 s.

---

## 7. The "gripper": film clamp, cut slide — and how the load is held

Nothing on this machine grips the **load**. The pallet is held on the deck by gravity and
friction, and the boxes are held on the pallet by gravity and friction until the film takes
over (§7.3). What is gripped is the **film**: its tail must be held to start a wrap, and cut
and laid down to end one.

### 7.1 Film clamp (on the turntable) — g1, g2

| Parameter | Value |
|---|---|
| Position in `{T}` | **(0.330, −0.689)**, radius 0.764 — outside the load (y = −0.40), inside the plate |
| Orientation | jaw faces parallel to the film line of the home pose (46.9°) |
| Lift `g1` | stroke 0.140: jaw tips at z = 0.544 (down, **20 mm below** the lowest film edge 0.564) or 0.684 (up) |
| Jaw `g2` | opening 40 mm (±20 mm about the nominal film line), grips the lowest **120 mm** of the web |
| Clamping force / friction | 250 N, μ = 0.6 (rubber on film) → holds 300 N: **SF 2.7** at the 110 N tension limit |
| Stroke times | lift 0.30 s, jaw 0.15 s |

The clamp can meet the film in exactly **one configuration**: `q1 ≡ 0`, carriage at the bottom.
Its position is *derived* from the home film line of §3.4; move the mast, the exit roller or
the pallet size and the clamp has to move with it.

How far the film can miss the open jaws: pallet centring (±10, ±15 mm) and home error (±0.5°)
shift the film line at the clamp by at most **13.3 mm** of the ±20 mm opening.

### 7.2 Cut-and-press slide (fixed stand) — g3

| Parameter | Value |
|---|---|
| Brush pad | x = 0.43 … 0.53, z = 0.55 … 1.03, compliant ±20 mm |
| Hot wire | vertical, at x = 0.395, set back **105 mm** behind the brush face |
| Park / pressed | brush face at y = −1.300 / −0.400 (the −Y face of a centred load) |
| Stroke | 0.90 m in 1.43 s |
| Wire pulse / press dwell | 0.40 s / 0.30 s |

Sequence at the end of a cycle, table at home and stopped:

1. `g1` up: the open jaws rise round the film web.
2. `g2` close: the film is held on the supply side.
3. `g3` in: the brush meets the film between the clamp and the corner (+0.6, −0.4) and pushes
   it flat against the −Y face. The film now runs corner → face → brush edge (0.43, −0.40) →
   clamp. The wire, 105 mm behind the brush, arrives on that last stretch (film at y = −0.501,
   wire at −0.505).
4. Wire pulse: the film is cut. The load keeps a tail of about 0.2 m, pressed on by the brush.
   The clamp keeps the end of the supply film.
5. `g3` out.

Sequence at the start of the next cycle:

1. The table starts. The clamp pulls film off the carriage; the film runs clamp → roller.
2. At `q1` = 74° the film reaches the first corner; by 246° it is round three corners.
3. At `q1` = **300°** the jaw opens and the clamp goes down (0.45 s). It must be below the film
   before `q1` = **345°**, because at 360° the film line sweeps over it again (0.175 s to spare
   at 12 rpm).
4. The 0.97 m tail falls against the −Y face and is covered by the second wrap.

Why the tail holds when the clamp lets go: capstan friction round three 90° corners,
`e^(μ·3π/2)` = **6.6** with μ = 0.4 → 9 N of tail tension balances the 60 N web tension, and
film cling provides more than that.

### 7.3 Holding the load on the turntable

Before it is wrapped the load is a stack of loose boxes on a spinning plate. Every box column
at radius `r` feels the inertial acceleration `a = r · √(ω⁴ + α²)` (centripetal and tangential).

| Check | Formula | Result |
|---|---|---|
| Box sliding on the layer below | `μ·g / a` | worst **SF 3.3** (0.90 m/s² at the corner columns of BX-3040) |
| Column tipping about its outer edge | `g · h(d) / (H_col / 2) / a`, `h(d)` = half base in the push direction | worst **SF 2.27** (BX-3040 × 6 layers, with peak ω and peak α taken together) |
| During the production run | as above with the actual ω, α | **SF ≥ 2.63** |
| Film tension pulling the top layer off | `μ · m_layer · g / F` | **SF ≥ 2.65** (54 kg layer, 60 N) |
| Pallet sliding on the deck rollers | `μ · m · g / F` (rollers braked) | SF > 7 for the lightest pallet |
| Unbalance of an off-centre load | `m · ω² · e` | 28 N at 1000 kg, 18 mm — negligible |
| Turntable torque | inertia `(110 + 173)·0.8` + film `110·0.72` | 227 + 79 N·m |

Required: SF ≥ 1.5 (rule R10). A **top platen** (a pressure plate lowered on the load, on a
passive bearing, turning with it) is needed as soon as this cannot be met at the wrap speed:
tall columns of small boxes, slippery sacks, empty containers. It would add one prismatic and
one passive revolute DOF and is not part of this machine.

### 7.4 Constraints the gripper puts on motion

- The cycle must **end at the home angle with the carriage at the bottom** — the only place
  where clamp, film and cutter meet. The wrap program is built backwards from that (§8.2).
- The turntable must **not start** unless the jaw is closed on the film.
- The jaw must **not open** before the film is round three corners (R6).
- The clamp must be **down** between `q1` = 345° of the first revolution and the final stop (R6).
- The cut slide leaves its park position **only** with the table at home and stopped, the
  carriage at the bottom and the jaw closed, and the table does not move until it is back (R4, R7).
- The parked slide must be outside the swept circle and outside every film line (§11).

---

## 8. Operating cycle

### 8.1 Phases

| # | Phase | Axes moving | What happens | Typical time |
|---|---|---|---|---|
| 1 | Transfer in | q2 + line | I1 → T, one pitch | 6.9 s |
| 2 | Tail | q1 (spin-up), q4 | the clamp pulls the film round the first corner | 2.0 s |
| 3 | Bottom wraps | q1, q4 | **2 revolutions** at `Z_BOTTOM`; clamp releases at 300° | 9 s |
| 4 | Spiral up | q1, q3, q4 | carriage up at 0.045 m/s, 50 % overlap, 0.225 m/rev | 7 … 30 s |
| 5 | Top wraps | q1, q4 | **2 revolutions** at the top | 10 s |
| 6 | Spiral down | q1, q3, q4 | carriage down at 0.0585 m/s, 35 % overlap, 0.2925 m/rev | 6 … 23 s |
| 7 | Final wraps | q1, q4 | **≥ 1.5 revolutions** at the bottom, the last part while braking; stops at home | 8 … 12 s |
| 8 | Clamp | g1, g2 | up, close | 0.45 s |
| 9 | Cut and press | g3 | in, wire, dwell, out | 3.6 s |
| 10 | Transfer out | q2 + line | T → O1 (and I1 → T) | 6.9 s |

### 8.2 Wrap program (`wrapPlan(H)`)

```
Z_BOTTOM = 0.500 + 0.144 − 0.080 + w/2 = 0.789      lowest film edge 80 mm below the pallet deck
Z_TOP    = 0.500 + H + 0.100 − w/2                   highest film edge 100 mm above the load

θ_ref(t) = turntable angle of a spin-up followed by an endless cruise
t1 = θ_ref⁻¹(2π · 2)                                 start of the up stroke
t2 = t1 + T_up                                       end of the up stroke
t3 = θ_ref⁻¹(θ_ref(t2) + 2π · 2)                     start of the down stroke
t4 = t3 + T_down                                     carriage back at the bottom
N  = ⌈ θ_ref(t4) / 2π + 1.5 ⌉                        whole revolutions of the cycle
turntable move: 0 → 2π·N
```

All of `t1 … t4` and the clamp release lie in the cruise part of the move (the carriage is
down at least 6.5 s before braking starts), so the helix has the programmed pitch everywhere.

| Load height `H` | Z_TOP | N | Rotation time |
|---|---|---|---|
| 0.744 (lowest in §3.5) | 1.119 | 9 | 47.0 s |
| 1.144 | 1.519 | 12 | 62.0 s |
| 1.744 (highest in §3.5) | 2.119 | 17 | 87.0 s |
| 2.000 (machine limit) | 2.375 | 19 | 97.0 s |

Film per pallet: ≈ `N` · 4.0 m stretched = `N` · 1.33 m off the roll; a roll lasts about 85
pallets.

---

## 9. Singularities, constraints and the safety monitor

### 9.1 Kinematic singularities and degenerate conditions

| Singularity | Condition | What goes wrong | Rule in this machine |
|---|---|---|---|
| **Helix degenerates** | `ω = 0` with `q̇3 ≠ 0` | the pitch `2π·q̇3/ω` is infinite: the film is drawn out as a vertical rope instead of a wrap; the surface is not covered | the carriage moves only above 90 % of the wrap speed (R5); both strokes lie inside the cruise (§8.2) |
| **Moment arm vanishes** | `p = 0`: the film line passes through the turntable axis | `det` of §5.6 is 0: turning the table lays no film, the film goes slack and then wraps the wrong way. Happens only if the axis is outside the load footprint | centring tolerance ±30 mm keeps `p ≥ 0.385`; an empty or grossly misplaced pallet is not rotated (R4) |
| **Corner hand-over** (4 per revolution) | the film line lies on a face | `dp/dq1` jumps → the film **acceleration** demanded by the load steps; a film drive that follows a speed command alone would tear or slacken the web | dancer (§5.4); `p` itself is continuous, so there is no speed step |
| **Unilateral film** | `ω < 0`, or `v_feed > ω·p` for long | film can only pull: reversing unwinds it, over-feeding lets it sag into the turntable | `q1` never reverses (R2); dancer and tension limits (R9) |
| **Deck alignment** | `q1 ≢ 0 (mod 2π)` | the deck rollers are not in line with the conveyors; a pallet driven now leaves the deck sideways. (At 180° the rollers line up again but the clamp is on the wrong side and the pallet is reversed — not used.) | `q2` runs only at home and at standstill (R3); `q1` runs only with both gap eyes clear (R4) |
| **Clamp rendez-vous** | clamp, film and cutter coincide only at `q1 ≡ 0`, `q3 = Z_BOTTOM` | anywhere else the clamp closes on air and the cut film end is lost: the machine cannot start the next pallet | whole-revolution turntable move; ≥ 1.5 final revolutions at the bottom so the film contact point has come down too (§5.3) |
| **Clamp in the film path** | clamp up when the film line sweeps its position (every `q1 ≡ 0`) | the clamp is wrapped into the load | clamp down by 345° of the first revolution (R6) |
| **Contact-point lag** | carriage stroke ends, table stops | the film on the load is up to 120 mm behind the carriage; top or bottom band missing | 2 top wraps, ≥ 1.5 final wraps (§8.1) |
| **Continuous axis** | `q1` unbounded | not a singularity, but anything cabled to the turntable winds up | rotary union + slip ring; `q1` is taken mod 2π only for the home test |

What this machine does **not** have: no posture branches, no wrist or elbow singularities, no
workspace boundary inside the task (the carriage travel covers every allowed load height). Its
failure modes are **sequence and timing** errors, which is why §9.2 exists.

### 9.2 Safety monitor

Rules R1–R10 are implemented as a monitor that reads **only joint positions, joint speeds and
sensor states** (`monitor()` in [js/kinematics.js](js/kinematics.js)). It does not know what
the sequencer intends.

| Rule | Hazard | Check |
|---|---|---|
| **R1** | over-travel | carriage and cut slide inside §4.1 |
| **R2** | overspeed, reversal | `0 ≤ ω ≤ 12 rpm`, `|α| ≤ 0.8`, carriage and deck speeds |
| **R3** | pallet driven off a misaligned deck | deck conveyor speed ≠ 0 only at home, table stopped |
| **R4** | crushing between rotating and fixed parts | table turning only with B2 and B4 clear, cut slide parked, load centred within 30 mm |
| **R5** | film roped, load not covered | carriage speed ≠ 0 with film on the load only if `ω ≥ 0.9 · 12 rpm` |
| **R6** | clamp wrapped in / film lost | clamp not up while turning beyond 345°; jaw not open before the film is round 3 corners |
| **R7** | cut slide hits the load, the film or the carriage | slide off its park position only at home, stopped, jaw closed, clamp up, carriage at `Z_BOTTOM` |
| **R8** | load strikes the carriage | swept radius of the load ≤ 0.80 |
| **R9** | film break, film jam | dancer inside ±60 mm; tension ≤ 110 N, and ≥ 25 N while wrapping; film speed ≤ 1.5 m/s |
| **R10** | boxes thrown off | stability SF of the load (§7.3) ≥ 1.5 at the current ω, α |

---

## 10. Cell layout (all coordinates in `{W}`)

| Item | Position |
|---|---|
| Turntable axis | (0, 0) |
| Infeed conveyor / outfeed conveyor | x = −7.90 … −0.95 / +0.95 … +7.90, centred on y = 0 |
| Pallet entry / exit points | x = −7.00 / +7.00 |
| Exit roller of the film carriage | (0, −1.10) |
| Mast | (−0.20, −1.52) |
| Clamp (home) | (0.330, −0.689) |
| Cut slide stand | (0.48, −1.85); slide axis along Y at x = 0.48 |
| B1, BH | x = −1.75 |
| Flow | −X → +X; table turns counter-clockwise seen from above |

---

## 11. Clearance budget

| Clearance | Nominal | Consumers |
|---|---|---|
| Turntable (R 0.92) ↔ line conveyor ends (x = ±0.95) | 30 mm | bearing run-out |
| Turntable ↔ carriage housing | 110 mm | — |
| Load corner (swept ≤ 0.738) ↔ exit roller surface (1.06) | 322 mm | centring ±10 / ±15, load overhang (none allowed) |
| Load swept radius ↔ envelope (0.80) | 62 mm | same |
| Pallet waiting at I1 ↔ swept circle | 288 mm | stop ±10 |
| Pallet ↔ pallet on the line | 550 mm (≥ 532 with stop errors) | — |
| Film line ↔ open clamp jaws | ±20 mm, 13.3 used | centring, home ±0.5° |
| Clamp (down) ↔ lowest film edge | 20 mm | film sag, carriage ±2 |
| Clamp ↔ load face (Y) | 289 mm | — |
| Clamp (r = 0.764 + jaws) ↔ plate edge | > 50 mm | — |
| Clamp down ↔ film line at 360° | 0.175 s = 12.6° of table angle | valve and cylinder scatter |
| Hot wire ↔ clamp jaws (X) | 35 mm | — |
| Cut slide ↔ carriage housing (X) | 315 mm | — |
| Parked cut slide ↔ lowest film line (Y) | 160 mm | — |
| Parked cut slide ↔ swept circle | > 400 mm | — |
| Brush ↔ load face | compliant ±20 mm | lateral centring ±15 |
| Carriage travel ↔ program | 0.789 … 2.375 inside 0.700 … 2.450 | — |

---

## 12. Verification results (`npm run verify:wrapper`)

4 h of operation simulated at 240 Hz, seed 7:

| Check | Result |
|---|---|
| Film line tangent to the exit roller, through the contact point, roller on the right | 1·10⁻¹⁵ m ✔ |
| **Film speed = ω·p**: d(path length)/dq1 vs the moment arm, 3 revolutions, 5 pallet positions | 3·10⁻⁹ m ✔ |
| `∮ p dq1` = perimeter | 4.000 m, error 1·10⁻⁹ ✔ |
| Moment arm continuous at every hand-over; never vanishes | ✔; 0.385 … 0.738 m (1.92 : 1) ✔ |
| Centred load: arm from half width to corner radius | 0.4000 … 0.7211 m ✔ |
| Film direction at the exit roller | 43.0° … 67.3° inside 30° … 80° ✔ |
| Wrap program for every height 0.344 … 2.000: whole revolutions, events in the cruise, carriage in travel | 6 … 19 rev, 32 … 97 s ✔ |
| Turntable profile ends on the home angle | 4·10⁻¹⁴ rad ✔ |
| Carriage speeds give the programmed overlap | 0.2250 and 0.2925 m/rev ✔ |
| Clamp on the home film line; film inside the open jaws for all tolerances | 0.24 mm; worst 13.3 of ±20 mm ✔ |
| Clamp below the film when down, 120 mm of web when up; clamp open and down in time | 20 mm; 0.175 s to spare ✔ |
| Capstan hold of the tail; clamp holding force | ratio 6.59; SF 2.73 ✔ |
| Static clearances of §11 | ✔ |
| Hot wire meets the film when the brush is on the load | film −0.501, wire −0.505 ✔ |
| Load stability at wrap speed, all SKUs at full height | SF 2.27 / top layer vs film SF 2.65 ✔ |
| Safety monitor negative tests (20 illegal states, 5 legal) | all classified correctly ✔ |
| Safety monitor violations during the run | 0 ✔ |
| Raw checks: deck moved off home / carriage moved below wrap speed / clamp up after 345° / slide out of sequence / tail released early | 0 / 0 / 0 / 0 / 0 steps ✔ |
| Turntable stops on the home angle after every wrap | 0 rad ✔ |
| Peak turntable speed / acceleration / jerk | 12.00 rpm / 0.800 rad/s² / 2.00 rad/s³ ✔ |
| Peak carriage / deck / cut slide speed | 0.0585 / 0.300 / 0.800 m/s ✔ |
| Peak film speed / pre-stretch acceleration | 0.947 of 1.5 m/s / 1.64 of 4 m/s² ✔ |
| Dancer excursion; film tension while wrapping | ±19.1 of ±60 mm; 54.3 … 64.6 N ✔ |
| **Film balance**: film paid out = film on the load + free film, every pallet | worst 1.9 mm of 53.3 m ✔ |
| **Coverage**: film layers at every point of every load face, every pallet | 2 … 9 layers ✔ |
| Film above the top of the load / below the pallet deck | ≥ 106 mm / ≥ 80 mm ✔ |
| Largest lag of the film contact point behind the carriage | 120 mm |
| Least load stability SF during the run / largest swept radius | 2.63 / 0.736 m ✔ |
| Smallest gap between pallets on the line | 0.532 m ✔ |
| Pallets conserved | 160 created = 159 left + 1 on the line ✔ |

**159 pallets wrapped = 39.8 / h, limited by pallet supply** (one every 90 s). Rotation 9 … 17
revolutions; wrap + clamp + cut 70.6 s mean (49.3 … 89.3). Film 53.3 m stretched = 17.8 m off
the roll per pallet; one roll change in the run. Longest infeed queue 2.

The verification found one real fault while this model was built: the sequencer released the
cut slide one control step before its position had been updated, so it sat a fraction of a
micrometre off its park position for the whole of the next wrap. The monitor did not see it
(inside its tolerance); the raw check "slide off park while the table turns" did, on 1.2 million
steps. That is why raw checks exist next to the monitor.

---

## 13. What the simulation does not model

- Dynamics: no motor torques, no turntable or mast deflection, no roller slip. Motion is
  kinematic and bounded by the limits of §4. The stability figures of §7.3 are quasi-static.
- Film mechanics: the film is inextensible between the pre-stretch and the load (no
  post-stretch, no relaxation, no neck-down change with tension), it never tears, and it lands
  as flat bands. Folding of the web over the top edge and round the pallet is not drawn.
- The deflection of the film by the brush during the cut, and the short loose ends on both
  sides of the cut.
- Pallet skew on the deck (only the position errors `dx`, `ey`), overhanging or leaning loads,
  boxes shifting under film tension. Boxes are rigid and never slide.
- Sensors are ideal and there are no faults (no film break, no missed clamp, no pallet stopping
  short). The recovery sequences are specified by the rules but never exercised.
- Collision checking is the rule set R3–R8 with swept radii and the static budget of §11 —
  not a mesh-level 3D test. People and safety fencing are not modelled.
- The roll change is a 90 s pause, not an operator sequence. Top-sheet dispensing, corner
  boards and labelling are not part of the machine.
- The conveyors' own drives beyond the S-curve of §4.2; downstream removal of pallets is
  instantaneous at the end of the outfeed.

---

## 14. Files and how to run

| What | Where | Command |
|---|---|---|
| Parameters (this document as code) | [js/config.js](js/config.js) | — |
| Kinematics, wrap program, safety monitor | [js/kinematics.js](js/kinematics.js) | — |
| Cell model and sequencer | [js/sim.js](js/sim.js) | — |
| three.js view | [index.html](index.html), [js/main.js](js/main.js) | `node serve.mjs 5176` in the repository root, then open `http://localhost:5176/wrapper/` |
| Verification | [test/verify.mjs](test/verify.mjs) | `npm run verify:wrapper` |
| Trajectory export for Blender (215 s, 24 fps) | [tools/export-blender.mjs](tools/export-blender.mjs) → `blender/wrapper_params.json` | `npm run export:wrapper` |
| Blender scene builder (one empty per joint frame of §3.4, keyframed) | [blender/build_wrapper.py](blender/build_wrapper.py) | run in Blender's Scripting tab, or `blender -b -P build_wrapper.py -- --glb wrapper.glb` |
| Built Blender scene | `blender/wrapper_cell.blend` | open in Blender 5.2 and press play |

In the browser, `window.wrapper.cell` is the running `Cell`; `window.wrapper.step(n)` advances it
by `n` steps of 1/240 s. In Blender the line conveyor rollers are static; only the deck rollers
turn.
