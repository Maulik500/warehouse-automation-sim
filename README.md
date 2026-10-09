# Warehouse automation simulations

Kinematic 3D simulations of warehouse automation equipment, written in plain JavaScript with
[three.js](https://threejs.org/). Each machine is built as the kinematic chain of its own
specification — joint limits, motion laws, singularities, gripper and safety rules — and each
has a verification suite that runs in Node.

Nothing needs to be installed to watch them: open the links below.

| Simulation | Live | Specification |
|---|---|---|
| **Integrated warehouse** — ASRS, robot AMRs, 4 palletizers, belly wrapper, forklift fleet | [open](https://maulik500.github.io/warehouse-automation-sim/warehouse/) | [WAREHOUSE_SPEC.md](warehouse/WAREHOUSE_SPEC.md) |
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

## Blender

Each cell can export its simulated trajectory (`npm run export:<cell>`) and has a Python script
under `<cell>/blender/` that rebuilds the scene in Blender with one empty per joint frame,
keyframed from the simulation. Built scenes are included for the wrapper, the mobile
depalletizer and the integrated warehouse (Blender 5.2).

## Scope

These are kinematic simulations: motion is bounded by speed, acceleration and jerk limits, not
by dynamics. Machines are generic, representative of their class, and not models of any
vendor's product. Each specification lists what it does not model.
