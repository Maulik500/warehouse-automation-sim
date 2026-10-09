# Builds the mobile depalletizer cell in Blender from mobman_params.json (written by
# tools/export-blender.mjs) and keyframes the base, the six arm joints, the deck stop, the
# pallet and every box with the simulated trajectory.
#
# Blender is Z-up and metric, exactly like MOBMAN_SPEC.md, so coordinates are used unchanged.
# Joint hierarchy (spec §2, §3.3):
#   MM_body (x, y, Rz θ) > MM_wheel_L / MM_wheel_R (Ry)
#                        > MM_arm_base {A} (fixed: ARM.base, Rz 180°)
#                          > MM_J1 (Rz q1) > MM_F1 (DH row 1: Tz d, Tx a, Rx α)
#                            > MM_J2 (Rz q2) > MM_F2 > … > MM_J6 (Rz q6) > MM_F6 = flange > MM_TCP
#                        > MM_front_stop (Z)
# The arm is one empty per Denavit–Hartenberg row: the hierarchy IS the kinematic chain.
#
# Run inside Blender:   Scripting tab > open this file > Run Script
# or headless:          blender -b -P build_mobman.py -- --glb mobman.glb

import bpy, bmesh, json, math, os, sys
from mathutils import Matrix

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else bpy.path.abspath("//")
with open(os.path.join(HERE, "mobman_params.json")) as f:
    P = json.load(f)
AMR, DECK, ARM, TOOL, PALLET, RECIPES, SRC, DST, DOCK, DH = (P[k] for k in ("AMR", "DECK", "ARM", "TOOL", "PALLET", "RECIPES", "SRC", "DST", "DOCK", "DH"))

COL_NAME = "MOBMAN_CELL"
old = bpy.data.collections.get(COL_NAME)
if old:  # rebuild only what this script created
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(old)
col = bpy.data.collections.new(COL_NAME)
bpy.context.scene.collection.children.link(col)
# sparse box tracks rely on straight segments between keys
bpy.context.preferences.edit.keyframe_new_interpolation_type = "LINEAR"


def material(name, rgb, metallic=0.2, rough=0.6):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*rgb, 1)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = rough
    return m


MAT = {
    "body": material("mm_body", (0.89, 0.44, 0.02), 0.2, 0.45),
    "dark": material("mm_dark", (0.02, 0.02, 0.03), 0.4, 0.6),
    "steel": material("mm_steel", (0.32, 0.36, 0.43), 0.7, 0.35),
    "roller": material("mm_roller", (0.48, 0.52, 0.58), 0.8, 0.3),
    "tyre": material("mm_tyre", (0.01, 0.01, 0.012), 0.0, 0.9),
    "conv": material("mm_conveyor", (0.07, 0.09, 0.12), 0.5, 0.6),
    "stop": material("mm_stop", (0.75, 0.17, 0.02), 0.3, 0.5),
    "wood": material("mm_wood", (0.48, 0.25, 0.09), 0.0, 0.85),
    "arm": material("mm_arm", (0.70, 0.73, 0.78), 0.3, 0.4),
    "joint": material("mm_joint", (0.07, 0.36, 1.0), 0.3, 0.4),
    "cup": material("mm_cup", (0.02, 0.03, 0.04), 0.0, 0.9),
    "mark": material("mm_marking", (0.89, 0.44, 0.02), 0.0, 0.8),
}
for i, S in enumerate(RECIPES):  # one material per recipe (sRGB hex -> linear)
    c = S["color"]
    MAT["rc%d" % i] = material("mm_" + S["id"], tuple((((c >> s) & 255) / 255) ** 2.2 for s in (16, 8, 0)), 0.0, 0.9)

AXIS_ROT = {"x": Matrix.Rotation(math.pi / 2, 4, "Y"), "y": Matrix.Rotation(math.pi / 2, 4, "X"), "z": Matrix.Identity(4)}


def mesh_obj(name, boxes, mat, parent=None, cyls=()):
    """One object made of boxes [(x0, x1, y0, y1, z0, z1), ...] and cylinders
    [(axis, a, b, r, u, v), ...]: along `axis` from a to b, at (u, v) on the other two axes."""
    bm = bmesh.new()
    for x0, x1, y0, y1, z0, z1 in boxes:
        bmesh.ops.create_cube(bm, size=1.0, matrix=Matrix.Translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)) @ Matrix.Diagonal((x1 - x0, y1 - y0, z1 - z0, 1.0)))
    for axis, a, b, r, u, v in cyls:
        mid = (a + b) / 2
        loc = {"x": (mid, u, v), "y": (u, mid, v), "z": (u, v, mid)}[axis]
        bmesh.ops.create_cone(bm, cap_ends=True, segments=28, radius1=r, radius2=r, depth=abs(b - a), matrix=Matrix.Translation(loc) @ AXIS_ROT[axis])
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    me.materials.append(mat)
    for poly in me.polygons:
        poly.use_smooth = bool(cyls) and not boxes
    ob = bpy.data.objects.new(name, me)
    col.objects.link(ob)
    if parent:
        ob.parent = parent
    return ob


def empty(name, parent=None, loc=(0, 0, 0), rot=(0, 0, 0), size=0.12):
    ob = bpy.data.objects.new(name, None)
    ob.empty_display_size = size
    ob.location = loc
    ob.rotation_mode = "XYZ"
    ob.rotation_euler = rot
    col.objects.link(ob)
    if parent:
        ob.parent = parent
    return ob


# ---------- source station (spec §10.1) ----------
x0, x1, hw, y, z = SRC["endX"] - SRC["len"], SRC["endX"], SRC["frameW"] / 2, SRC["y"], SRC["z"]
frame = []
for e in (1, -1):
    ya, yb = sorted((y + e * hw - e * 0.04, y + e * hw))
    frame.append((x0, x1, ya, yb, z - 0.14, z + 0.005))
    x = x0 + 0.1
    while x < x1:
        frame.append((x - 0.03, x + 0.03, ya, yb, 0, z - 0.14))
        x += 1.0
    yp = y + e * (hw + 0.06)
    frame.append((SRC["b1x"] - 0.02, SRC["b1x"] + 0.02, yp - 0.02, yp + 0.02, 0, z + SRC["b1z"] + 0.05))  # B1 posts
mesh_obj("Source_frame", frame, MAT["conv"])
mesh_obj("Source_rollers", [], MAT["roller"], cyls=[("y", y - hw + 0.05, y + hw - 0.05, 0.04, x0 + 0.08 + i * 0.12, z - 0.04) for i in range(int(SRC["len"] / 0.12))])
mesh_obj("Source_end_stop", [(SRC["x"] + PALLET["sx"] / 2 + 0.02, x1, y - hw, y + hw, z - 0.14, z + 0.11)], MAT["stop"])

# ---------- destination conveyor (spec §10.2) ----------
y1, y0, hw, x, z = DST["yEnd"], DST["yEnd"] - DST["len"], DST["w"] / 2, DST["x"], DST["z"]
frame = []
for e in (1, -1):
    xa, xb = sorted((x + e * hw, x + e * (hw + 0.04)))
    frame.append((xa, xb, y0, y1, z - 0.12, z + 0.05))
    yy = y0 + 0.1
    while yy < y1:
        frame.append((xa, xb, yy - 0.03, yy + 0.03, 0, z - 0.12))
        yy += 1.2
mesh_obj("Destination_frame", frame, MAT["conv"])
rp, rr = DECK["rollerPitch"], DECK["rollerR"]
mesh_obj("Destination_rollers", [], MAT["roller"], cyls=[("x", x - hw, x + hw, rr, y1 - 0.04 - i * rp, z - rr) for i in range(int(DST["len"] / rp))])

# ---------- route ----------
cu = bpy.data.curves.new("Route", "CURVE")
cu.dimensions = "3D"
sp = cu.splines.new("POLY")
sp.points.add(len(P["route"]) - 1)
for p, (px, py) in zip(sp.points, P["route"]):
    p.co = (px, py, 0.003, 1)
cu.bevel_depth = 0.008
ob = bpy.data.objects.new("Route", cu)
ob.data.materials.append(MAT["mark"])
col.objects.link(ob)

# ---------- AMR ----------
B = AMR["body"]
bw, ch = B["w"] / 2, AMR["chassisH"]
body = empty("MM_body", size=0.4)
mesh_obj("MM_chassis", [(B["xr"], B["xf"], -bw, bw, AMR["groundClear"], ch)], MAT["body"], body)


def wheel(name, parent, r, w, loc):
    j = empty(name, parent, loc)
    mesh_obj(name + "_tyre", [], MAT["tyre"], j, cyls=[("y", -w / 2, w / 2, r, 0, 0)])
    mesh_obj(name + "_hub", [(-r * 0.5, r * 0.5, -w / 2 - 0.009, w / 2 + 0.009, -0.012, 0.012)], MAT["steel"], j, cyls=[("y", -w / 2 - 0.006, w / 2 + 0.006, r * 0.55, 0, 0)])
    return j


wl = wheel("MM_wheel_L", body, AMR["rWheel"], AMR["wWheel"], (0, AMR["track"] / 2, AMR["rWheel"]))
wr = wheel("MM_wheel_R", body, AMR["rWheel"], AMR["wWheel"], (0, -AMR["track"] / 2, AMR["rWheel"]))
mesh_obj("MM_casters", [], MAT["tyre"], body, cyls=[("y", cy - 0.02, cy + 0.02, AMR["rCaster"], cx, AMR["rCaster"]) for cx, cy in AMR["casters"]])

dy0, dy1 = DECK["yc"] - DECK["w"] / 2, DECK["yc"] + DECK["w"] / 2
mesh_obj("MM_deck_frame", [
    (DECK["x0"], DECK["x1"], dy0 - 0.04, dy0, ch, DECK["z"] + DECK["guideH"]),
    (DECK["x0"], DECK["x1"], dy1, dy1 + 0.04, ch, DECK["z"] + DECK["guideH"]),
], MAT["conv"], body)
mesh_obj("MM_deck_bed", [(DECK["x0"], DECK["x1"], dy0, dy1, ch, DECK["z"] - 2 * rr - 0.01)], MAT["dark"], body)
mesh_obj("MM_deck_rollers", [], MAT["roller"], body, cyls=[("y", dy0, dy1, rr, DECK["x0"] + 0.04 + i * rp, DECK["z"] - rr) for i in range(int((DECK["x1"] - DECK["x0"]) / rp))])
mesh_obj("MM_rear_stop", [(DECK["backX"] - 0.03, DECK["backX"], dy0, dy1, DECK["z"], DECK["z"] + DECK["stopH"])], MAT["stop"], body)
stop = empty("MM_front_stop", body, (0, 0, 0))
mesh_obj("MM_front_stop_plate", [(DECK["stopX"], DECK["stopX"] + 0.03, dy0 + 0.05, dy1 - 0.05, DECK["z"] - DECK["stopH"], DECK["z"])], MAT["stop"], stop)

bx, by, bz = ARM["base"]
mesh_obj("MM_pedestal", [(bx - 0.17, bx + 0.17, by - 0.14, by + 0.14, ch, ch + 0.30)], MAT["dark"], body, cyls=[("z", ch, bz, ARM["pedestalR"], bx, by)])
arm_base = empty("MM_arm_base", body, (bx, by, bz), (0, 0, ARM["yaw"]), 0.25)

# DH chain: J[i] turns by q[i] about its Z; F[i] is frame i+1 = Tz(d)·Tx(a)·Rx(α) inside it
J, F = [], []
parent = arm_base
for i, row in enumerate(DH):
    j = empty("MM_J%d" % (i + 1), parent)
    f = empty("MM_F%d" % (i + 1), j, (row["a"], 0, row["d"]), (row["alpha"], 0, 0), 0.08)
    J.append(j)
    F.append(f)
    parent = f
d1, a2, a3, d4, d5, d6, eU, eF = (ARM[k] for k in ("d1", "a2", "a3", "d4", "d5", "d6", "offUpper", "offFore"))
rU, rF, rW, _ = ARM["rLink"]
mesh_obj("MM_arm_base_housing", [], MAT["arm"], J[0], cyls=[("z", 0, d1 - rU, 0.105, 0, 0)])
mesh_obj("MM_shoulder", [], MAT["joint"], F[0], cyls=[("z", -rU, eU + rU * 0.9, rU, 0, 0)])
mesh_obj("MM_upper_arm", [], MAT["arm"], F[1], cyls=[("x", 0, a2, rU * 0.82, 0, eU)])
mesh_obj("MM_elbow", [], MAT["joint"], F[1], cyls=[("z", eF - rF, eU + rU * 0.9, rF * 1.08, 0, 0)])
mesh_obj("MM_forearm", [], MAT["arm"], F[2], cyls=[("x", 0, a3, rF * 0.85, 0, eF)])
mesh_obj("MM_wrist_1", [], MAT["joint"], F[2], cyls=[("z", eF - rF, d4 + rW, rW * 1.05, 0, 0)])
mesh_obj("MM_wrist_2", [], MAT["arm"], F[3], cyls=[("z", -rW, d5 + rW, rW, 0, 0)])
mesh_obj("MM_wrist_3", [], MAT["joint"], F[4], cyls=[("z", -rW, d6, rW, 0, 0)])
tz = TOOL["tcp"]
mesh_obj("MM_gripper", [(-TOOL["padX"] / 2, TOOL["padX"] / 2, -TOOL["padY"] / 2, TOOL["padY"] / 2, tz - 0.05, tz - 0.025)], MAT["steel"], F[5], cyls=[("z", 0, 0.012, 0.045, 0, 0), ("z", 0.012, tz - 0.05, 0.028, 0, 0)])
mesh_obj("MM_cups", [], MAT["cup"], F[5], cyls=[("z", tz - 0.025, tz, TOOL["cupDia"] / 2, (i - 1) * 0.085, e * 0.045) for i in range(3) for e in (-1, 1)])
tcp = empty("MM_TCP", F[5], (0, 0, tz))
tcp.empty_display_type = "ARROWS"

# ---------- pallet(s) and boxes ----------
PL = PALLET
pal_boxes = []
for yb, w in ((-0.3275, 0.145), (-0.1638, 0.1), (0, 0.145), (0.1638, 0.1), (0.3275, 0.145)):
    pal_boxes.append((-PL["sx"] / 2, PL["sx"] / 2, yb - w / 2, yb + w / 2, PL["h"] - 0.022, PL["h"]))
for xb in (-0.5275, 0, 0.5275):
    pal_boxes.append((xb - 0.0725, xb + 0.0725, -PL["sy"] / 2, PL["sy"] / 2, 0.100, 0.122))
    for yb in (-0.35, 0, 0.35):
        w = 0.1 if yb else 0.145
        pal_boxes.append((xb - 0.0725, xb + 0.0725, yb - w / 2, yb + w / 2, 0.022, 0.100))
for yb in (-0.35, 0, 0.35):
    w = 0.1 if yb else 0.145
    pal_boxes.append((-PL["sx"] / 2, PL["sx"] / 2, yb - w / 2, yb + w / 2, 0, 0.022))

movers = []
for p in P["pallets"]:
    ob = mesh_obj("Pallet_%02d_%s" % (p["id"], p["recipe"]), pal_boxes, MAT["wood"])
    ob.rotation_mode = "XYZ"
    movers.append((ob, p))
for b in P["boxes"]:
    sx, sy, sz = b["size"]
    ob = mesh_obj("Box_%03d" % b["id"], [(-sx / 2 + 0.002, sx / 2 - 0.002, -sy / 2 + 0.002, sy / 2 - 0.002, -sz / 2 + 0.0015, sz / 2 - 0.0015)], MAT["rc%d" % b["rc"]])
    ob.rotation_mode = "XYZ"
    movers.append((ob, b))

# ---------- animation from the simulation ----------
scene = bpy.context.scene
scene.render.fps = P["fps"]
scene.frame_start = 1
scene.frame_end = len(P["frames"])


def key(ob, path, value, frame, last):
    """Set and keyframe a property, skipping frames where nothing changed."""
    k = (ob.name, path)
    if last.get(k) == value and frame < scene.frame_end:
        return
    setattr(ob, path, value)
    if k in last and last[k] != value and last.get((k, "f")) != frame - 1:
        setattr(ob, path, last[k])  # hold the old value up to the frame before the change
        ob.keyframe_insert(path, frame=frame - 1)
        setattr(ob, path, value)
    ob.keyframe_insert(path, frame=frame)
    last[k] = value
    last[(k, "f")] = frame


last = {}
for i, fr_ in enumerate(P["frames"]):
    x, y, th, wL, wR = fr_[:5]
    q, stop_up = fr_[5:11], fr_[11]
    fr = i + 1
    key(body, "location", (x, y, 0), fr, last)
    key(body, "rotation_euler", (0, 0, th), fr, last)
    key(wl, "rotation_euler", (0, wL, 0), fr, last)
    key(wr, "rotation_euler", (0, wR, 0), fr, last)
    for j, qi in zip(J, q):
        key(j, "rotation_euler", (0, 0, qi), fr, last)  # q1 … q6
    key(stop, "location", (0, 0, DECK["stopH"] * stop_up), fr, last)

HIDDEN = (0.0, 0.0, 0.0)
for ob, m in movers:
    first, end = m["first"] + 1, m["last"] + 1
    if first > 1:  # not in the cell yet
        ob.scale = HIDDEN
        ob.keyframe_insert("scale", frame=1)
        ob.keyframe_insert("scale", frame=first - 1)
    ob.scale = (1.0, 1.0, 1.0)
    ob.keyframe_insert("scale", frame=first)
    for f, x, y, z_, yaw in m["track"]:  # sparse: keys only where the pose changes
        ob.location = (x, y, z_)
        ob.rotation_euler = (0, 0, yaw)
        ob.keyframe_insert("location", frame=f + 1)
        ob.keyframe_insert("rotation_euler", frame=f + 1)
    if end < scene.frame_end:  # left the cell
        ob.keyframe_insert("scale", frame=end)
        ob.scale = HIDDEN
        ob.keyframe_insert("scale", frame=end + 1)
scene.frame_set(1)

# ---------- optional GLB export:  blender -b -P build_mobman.py -- --glb out.glb ----------
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
if "--glb" in argv:
    path = os.path.abspath(argv[argv.index("--glb") + 1])
    bpy.ops.export_scene.gltf(filepath=path, export_format="GLB", export_yup=True, export_animations=True)
    print("exported", path)

result = {"objects": len(col.objects), "frames": len(P["frames"]), "boxes": len(P["boxes"]), "pallets": len(P["pallets"])}
