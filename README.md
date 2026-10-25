# Warehouse automation simulations

Kinematic 3D simulations of warehouse automation equipment, written in plain JavaScript with
[three.js](https://threejs.org/). Each machine is built as the kinematic chain of its own
specification — joint limits, motion laws, singularities, gripper and safety rules — and each
has a verification suite that runs in Node.

Nothing needs to be installed to watch them: open the links below.

## Start here: the guided tour

**[▶ Guided tour of the integrated warehouse](https://maulik500.github.io/warehouse-automation-sim/warehouse/present.html)** — a 3-minute directed
walk-through with captions, labels, live counters and an engineering panel (joint angles and
singularity margins of the machine in focus). Every motion comes from a recorded run of the
verified simulation; nothing is keyframed by hand.

| Download | What it is |
|---|---|
| [Warehouse_Tour.mp4](downloads/Warehouse_Tour.mp4) | the tour as a 1080p video (50 MB) |
| [Warehouse_Tour.html](downloads/Warehouse_Tour.html) | the tour as one file — opens by double-click, works offline (4 MB) |
| [Warehouse_Live.html](downloads/Warehouse_Live.html) | the live engineering simulation as one offline file (1.4 MB) |

| Simulation | Live | Specification |
|---|---|---|
| **Integrated warehouse** — ASRS, robot AMRs, 4 palletizers, belly wrapper, forklift fleet | [live](https://maulik500.github.io/warehouse-automation-sim/warehouse/) · [tour](https://maulik500.github.io/warehouse-automation-sim/warehouse/present.html) | [WAREHOUSE_SPEC.md](warehouse/WAREHOUSE_SPEC.md) |
| Palletizing robot — 6-axis arm, vacuum gripper, pick from a moving belt | [open](https://maulik500.github.io/warehouse-automation-sim/) | [ROBOT_SPEC.md](ROBOT_SPEC.md) |
| ASRS — 3 stacker cranes, 5 rack rows, telescopic forks | [open](https://maulik500.github.io/warehouse-automation-sim/asrs/) | [ASRS_SPEC.md](asrs/ASRS_SPEC.md) |
| Forklift AMR — tricycle fork truck, mast, pallet handling | [open](https://maulik500.github.io/warehouse-automation-sim/amr/) | [AMR_SPEC.md](amr/AMR_SPEC.md) |
| Belly wrapper — turntable stretch wrapper, film clamp and cutter | [open](https://maulik500.github.io/warehouse-automation-sim/wrapper/) | [WRAPPER_SPEC.md](wrapper/WRAPPER_SPEC.md) |
| Mobile depalletizer — AMR with a 6-axis arm and an on-board conveyor | [open](https://maulik500.github.io/warehouse-automation-sim/mobman/) | [MOBMAN_SPEC.md](mobman/MOBMAN_SPEC.md) |

## The integrated warehouse

Source pallets are stored in the ASRS, one SKU per row. A robot AMR takes one layer of boxes
from the pallet presented at a row's port and carries it to one of four palletizing cells, where
a 6-axis robot builds a mixed pallet, one SKU per layer. Forklift AMRs take the finished pallet
to the belly wrapper, the wrapped pallet to storage, and keep the cells supplied with empty
pallets. Orders are generated automatically and can be added by hand; the fleet size is
adjustable (1–6 forklifts, 1–2 robot AMRs).

Measured in the verification runs: about 5–6 mixed pallets per hour with 3 forklifts and 2 robot
AMRs, with zero safety-monitor violations. The robot AMRs are the bottleneck.

## Run locally

Requires [Node.js](https://nodejs.org/) 20 or newer. There are no dependencies to install.

```bash
node serve.mjs
```

Then open `http://localhost:5173/` (palletizer), or `/asrs/index.html`, `/amr/index.html`,
`/wrapper/index.html`, `/mobman/index.html`, `/warehouse/index.html`.

## Verify

```bash
npm run verify            # palletizing robot
npm run verify:asrs
npm run verify:amr
npm run verify:wrapper
npm run verify:mobman
npm run verify:warehouse  # about 15 minutes; add `-- --quick` for a short run
```

## Presentation, offline files and video

```bash
npm run record:warehouse     # 40 min of the run, 15 fps -> warehouse/rec/clip.json.gz (2 MB)
npm run build:offline        # -> warehouse/dist/Warehouse_Tour.html, Warehouse_Live.html
npm run capture:warehouse    # -> warehouse/dist/Warehouse_Tour.mp4 (headless Chrome + ffmpeg)
```

The tour's steps are found in the recorded event log (crane retrieval, AMR dock, layer pick,
transfer, palletizing, forklift pick, wrapping, storage, empties, replenishment), so a new
recording gives a new, still accurate tour.

## Blender

Each cell can export its simulated trajectory (`npm run export:<cell>`) and has a Python script
under `<cell>/blender/` that rebuilds the scene in Blender with one empty per joint frame,
keyframed from the simulation. Built scenes are included for the wrapper, the mobile
depalletizer and the integrated warehouse (Blender 5.2).

## Scope

These are kinematic simulations: motion is bounded by speed, acceleration and jerk limits, not
by dynamics. Machines are representative of their class (the palletizing robot has the link
lengths of a KUKA KR 120 R2700 class arm, the mobile arm the geometry of a UR20 class arm); they
are not vendor models. Each specification lists what it does not model.
