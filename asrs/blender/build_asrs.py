# Builds the ASRS in Blender from asrs_params.json (written by tools/export-blender.mjs)
# and keyframes every crane joint with the simulated trajectory.
#
# Blender is Z-up and metric, exactly like ASRS_SPEC.md, so coordinates are used unchanged.
# One crane per aisle n. Joint hierarchy (spec §3.1), under an Aisle_n empty at the aisle's Y:
#   Cn_J1_travel (X) > Cn_J2_hoist (Z) > Cn_J3_fork_mid (Y, q3/2)
#                                      > Cn_J3_fork_top (Y, q3)
#
# Run inside Blender:   Scripting tab > open this file > Run Script
# or headless:          blender -b -P build_asrs.py -- --glb asrs.glb

import bpy, bmesh, json, os, sys
from mathutils import Matrix

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "asrs_params.json")) as f:
    P = json.load(f)
CRANE, FORK, LOAD, RACK, PD, LAYOUT, SKUS = (P[k] for k in ("CRANE", "FORK", "LOAD", "RACK", "PD", "LAYOUT", "SKUS"))

COL_NAME = "ASRS"
old = bpy.data.collections.get(COL_NAME)
if old:  # rebuild only what this script created
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(old)
col = bpy.data.collections.new(COL_NAME)
bpy.context.scene.collection.children.link(col)


def material(name, rgb, metallic=0.2, rough=0.6):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*rgb, 1)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = rough
    return m


MAT = {
    "crane": material("asrs_crane", (0.89, 0.44, 0.02), 0.2, 0.45),
    "dark": material("asrs_dark", (0.02, 0.02, 0.03), 0.4, 0.6),
    "fork": material("asrs_fork", (0.7, 0.72, 0.76), 0.8, 0.3),
    "upright": material("asrs_upright", (0.03, 0.11, 0.34), 0.35, 0.55),
    "ledge": material("asrs_ledge", (0.75, 0.17, 0.02), 0.3, 0.5),
    "conv": material("asrs_conveyor", (0.07, 0.09, 0.12), 0.5, 0.6),
    "wood": material("asrs_wood", (0.48, 0.25, 0.09), 0.0, 0.85),
}
for i, S in enumerate(SKUS):  # one material per box type (sRGB hex -> linear)
    c = S["color"]
    MAT["sku%d" % i] = material("asrs_" + S["sku"], tuple((((c >> s) & 255) / 255) ** 2.2 for s in (16, 8, 0)), 0.0, 0.9)


def mesh_obj(name, boxes, mat, parent=None):
    """One object made of axis-aligned boxes [(sx, sy, sz, x, y, z), ...] in its own frame."""
    bm = bmesh.new()
    for sx, sy, sz, x, y, z in boxes:
        bmesh.ops.create_cube(bm, size=1.0, matrix=Matrix.Translation((x, y, z)) @ Matrix.Diagonal((sx, sy, sz, 1.0)))
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    col.objects.link(ob)
    if parent:
        ob.parent = parent
    return ob


def empty(name, parent=None):
    ob = bpy.data.objects.new(name, None)
    ob.empty_display_size = 0.3
    col.objects.link(ob)
    if parent:
        ob.parent = parent
    return ob


bay_x = lambda b: RACK["x1"] + (b - 1) * RACK["pitchX"]
level_z = lambda l: RACK["z1"] + (l - 1) * RACK["pitchZ"]
x_rack0 = bay_x(1) - RACK["pitchX"] / 2
y_back = RACK["yFace"] + RACK["depth"]
half = (RACK["pitchX"] - RACK["uprightW"]) / 2
drop = CRANE["carriageDrop"]
tines = lambda w, t, z: [(w, FORK["length"], t, e * FORK["tineX"], 0, z) for e in (1, -1)]
joints = []

for ai, A in enumerate(LAYOUT["aisles"]):
    n = A["id"]
    aisle = empty("Aisle_%d" % n)
    aisle.location = (0, A["y"], 0)

    # ---------- racks (spec §3.3) ----------
    for row, side in A["rows"]:
        up, ledge = [], []
        for k in range(RACK["bays"] + 1):
            x = x_rack0 + k * RACK["pitchX"]
            for y in (RACK["yFace"] + 0.05, y_back - 0.05):
                up.append((RACK["uprightW"], 0.10, RACK["height"], x, side * y, RACK["height"] / 2))
            for l in range(RACK["levels"] + 1):
                up.append((0.05, RACK["depth"] - 0.1, 0.05, x, side * (RACK["yFace"] + RACK["depth"] / 2), RACK["z1"] + l * RACK["pitchZ"] - 0.30))
        for b in range(1, RACK["bays"] + 1):
            for l in range(1, RACK["levels"] + 1):
                for e in (1, -1):
                    x, z, y = bay_x(b), level_z(l), side * RACK["yStore"]
                    ledge.append((RACK["ledgeW"], LOAD["sy"], 0.008, x + e * (half - RACK["ledgeW"] / 2), y, z - 0.004))
                    ledge.append((0.008, LOAD["sy"], RACK["ledgeLegH"], x + e * (half - 0.004), y, z - RACK["ledgeLegH"] / 2))
        mesh_obj("Rack_%s_uprights" % row, up, MAT["upright"], aisle)
        mesh_obj("Rack_%s_ledges" % row, ledge, MAT["ledge"], aisle)

    # ---------- stations and conveyor lanes (spec §3.5) ----------
    x0, x1 = PD["xPort"] - 0.55, PD["x"] - LOAD["sx"] / 2 - 0.06
    for side, name in ((PD["inSide"], "IN"), (PD["outSide"], "OUT")):
        y = side * RACK["yStore"]
        parts = [(PD["ledgeW"], LOAD["sy"], 0.10, PD["x"] + e * PD["ledgeX"], y, PD["z"] - 0.05) for e in (1, -1)]
        parts += [(x1 - x0, 0.05, 0.14, (x0 + x1) / 2, y + e * (LOAD["sy"] / 2 - 0.01), PD["z"] - 0.07) for e in (1, -1)]
        parts.append((x1 - x0, LOAD["sy"] - 0.1, 0.06, (x0 + x1) / 2, y, PD["z"] + PD["deckLift"] - 0.03))  # roller bed
        mesh_obj("Lane_%s_%d" % (name, n), parts, MAT["conv"], aisle)

    # ---------- crane: one empty per joint frame ----------
    j1 = empty("C%d_J1_travel" % n, aisle)
    j2 = empty("C%d_J2_hoist" % n, j1)
    j3m = empty("C%d_J3_fork_mid" % n, j2)
    j3t = empty("C%d_J3_fork_top" % n, j2)
    C = CRANE
    base = [(C["chassisLen"], 0.50, C["chassisH"] - 0.08, 0, 0, 0.08 + (C["chassisH"] - 0.08) / 2)]
    for e in (1, -1):
        base.append((C["mastSx"], C["mastSy"], C["mastH"] - C["chassisH"], e * C["mastX"], 0, (C["mastH"] + C["chassisH"]) / 2))
    base.append((2 * C["mastX"] + C["mastSx"], C["mastSy"], 0.22, 0, 0, C["mastH"] - 0.11))
    mesh_obj("C%d_base" % n, base, MAT["crane"], j1)
    car = [(C["carriageSx"], C["carriageSy"], drop - FORK["thick"], 0, 0, -(drop + FORK["thick"]) / 2)]
    for e in (1, -1):
        car.append((0.06, 0.55, 1.15, e * (C["carriageSx"] / 2 - 0.03), 0, -drop + 0.575))
    mesh_obj("C%d_carriage" % n, car, MAT["crane"], j2)
    mesh_obj("C%d_fork_base" % n, tines(FORK["tineW"], 0.025, -0.0575), MAT["dark"], j2)
    mesh_obj("C%d_fork_mid" % n, tines(FORK["tineW"] - 0.03, 0.025, -0.0325), MAT["dark"], j3m)
    mesh_obj("C%d_fork_top" % n, tines(FORK["tineW"], 0.020, -0.010), MAT["fork"], j3t)
    joints.append((j1, j2, j3m, j3t, P["frames"][ai]))

# ---------- stored pallets and their boxes (world coordinates) ----------
mesh_obj("Pallets", [(LOAD["sx"], LOAD["sy"], LOAD["palletH"], p["x"], p["y"], p["z"] + LOAD["palletH"] / 2) for p in P["stored"]], MAT["wood"])
for i, S in enumerate(SKUS):
    bx = [(sx - 0.006, sy - 0.006, sz - 0.004, p["x"] + lx, p["y"] + ly, p["z"] + lz) for p in P["stored"] if p["sku"] == i for sx, sy, sz, lx, ly, lz in p["boxes"]]
    if bx:
        mesh_obj("Boxes_" + S["sku"], bx, MAT["sku%d" % i])

# ---------- joint animation from the simulation ----------
scene = bpy.context.scene
scene.render.fps = P["fps"]
scene.frame_start = 1
scene.frame_end = len(P["frames"][0])
for j1, j2, j3m, j3t, frames in joints:
    for i, (q1, q2, q3, sag) in enumerate(frames):
        j1.location = (q1, 0, 0)
        j2.location = (0, 0, q2)
        j3m.location = (0, q3 * FORK["midRatio"], -sag / 8)
        j3t.location = (0, q3, -sag)
        for ob in (j1, j2, j3m, j3t):
            ob.keyframe_insert("location", frame=i + 1)
scene.frame_set(1)

# ---------- optional GLB export:  blender -b -P build_asrs.py -- --glb out.glb ----------
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
if "--glb" in argv:
    path = os.path.abspath(argv[argv.index("--glb") + 1])
    bpy.ops.export_scene.gltf(filepath=path, export_format="GLB", export_yup=True, export_animations=True)
    print("exported", path)

result = {"objects": len(col.objects), "cranes": len(joints), "frames": len(P["frames"][0])}
