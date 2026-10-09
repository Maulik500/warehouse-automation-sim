# Builds the pallet AMR cell in Blender from amr_params.json (written by tools/export-blender.mjs)
# and keyframes every AMR joint and every pallet with the simulated trajectory.
#
# Blender is Z-up and metric, exactly like AMR_SPEC.md, so coordinates are used unchanged.
# Joint hierarchy (spec §3.2):
#   AMR_body (x, y, Rz θ) > AMR_J2_steer (Rz) > AMR_J1_drive_wheel (Ry)
#                         > AMR_wheel_L / AMR_wheel_R (Ry, passive)
#                         > AMR_J3_tilt (Ry, −q3) > AMR_J4_lift (Z) > AMR_J5_shift (Y) > tines, TCP
#
# Run inside Blender:   Scripting tab > open this file > Run Script
# or headless:          blender -b -P build_amr.py -- --glb amr.glb

import bpy, bmesh, json, math, os, sys
from mathutils import Matrix

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "amr_params.json")) as f:
    P = json.load(f)
AMR, MAST, FORK, PALLET, SKUS, CONV, DOCK, CELL = (P[k] for k in ("AMR", "MAST", "FORK", "PALLET", "SKUS", "CONV", "DOCK", "CELL"))
XT, ZT = MAST["pivot"]

COL_NAME = "AMR_CELL"
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
    "body": material("amr_body", (0.89, 0.44, 0.02), 0.2, 0.45),
    "dark": material("amr_dark", (0.02, 0.02, 0.03), 0.4, 0.6),
    "steel": material("amr_steel", (0.32, 0.36, 0.43), 0.7, 0.35),
    "fork": material("amr_fork", (0.7, 0.72, 0.76), 0.8, 0.3),
    "tyre": material("amr_tyre", (0.01, 0.01, 0.012), 0.0, 0.9),
    "conv": material("amr_conveyor", (0.07, 0.09, 0.12), 0.5, 0.6),
    "stop": material("amr_stop", (0.75, 0.17, 0.02), 0.3, 0.5),
    "wood": material("amr_wood", (0.48, 0.25, 0.09), 0.0, 0.85),
    "mark": material("amr_marking", (0.05, 0.6, 0.3), 0.0, 0.8),
}
for i, S in enumerate(SKUS):  # one material per box type (sRGB hex -> linear)
    c = S["color"]
    MAT["sku%d" % i] = material("amr_" + S["sku"], tuple((((c >> s) & 255) / 255) ** 2.2 for s in (16, 8, 0)), 0.0, 0.9)


def mesh_obj(name, boxes, mat, parent=None, cyls=()):
    """One object made of boxes [(x0, x1, y0, y1, z0, z1), ...] and Y-axis cylinders [(r, w, x, y, z), ...]."""
    bm = bmesh.new()
    for x0, x1, y0, y1, z0, z1 in boxes:
        bmesh.ops.create_cube(bm, size=1.0, matrix=Matrix.Translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)) @ Matrix.Diagonal((x1 - x0, y1 - y0, z1 - z0, 1.0)))
    for r, w, x, y, z in cyls:
        bmesh.ops.create_cone(bm, cap_ends=True, segments=32, radius1=r, radius2=r, depth=w, matrix=Matrix.Translation((x, y, z)) @ Matrix.Rotation(math.pi / 2, 4, "X"))
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    col.objects.link(ob)
    if parent:
        ob.parent = parent
    return ob


def empty(name, parent=None, loc=(0, 0, 0)):
    ob = bpy.data.objects.new(name, None)
    ob.empty_display_size = 0.15
    ob.location = loc
    ob.rotation_mode = "XYZ"
    col.objects.link(ob)
    if parent:
        ob.parent = parent
    return ob


# ---------- conveyors and pick stations (spec §6) ----------
x_face = CONV["xP"] - PALLET["sx"] / 2
x0, x1 = x_face - CONV["endOverhang"], CONV["xP"] + CONV["cap"] * CONV["pitch"] + PALLET["sx"] / 2 + 0.3
hw, z = CONV["frameW"] / 2, CONV["z"]
for L in CONV["lines"]:
    y = L["y"]
    frame, chain = [], []
    for e in (1, -1):
        frame.append((x0, x1, y + e * hw - 0.03, y + e * hw + 0.03, z - 0.24, z - 0.06))
        x = x0 + 0.1
        while x < x1:
            frame.append((x - 0.04, x + 0.04, y + e * hw - 0.04, y + e * hw + 0.04, 0, z - 0.24))
            x += 1.3
    x = x0 + 0.1
    while x < x1:
        frame.append((x - 0.03, x + 0.03, y - hw, y + hw, z - 0.22, z - 0.14))
        x += 1.3
    for cy in CONV["chainY"]:
        chain.append((x0, x1, y + cy - CONV["chainW"] / 2, y + cy + CONV["chainW"] / 2, z - 0.05, z))
        frame.append((x0, x1, y + cy - 0.07, y + cy + 0.07, z - 0.14, z - 0.05))
    # sensor posts: B1 pallet present, B3 load height
    for sx, sz in ((x_face + CONV["b1x"], CONV["b1z"]), (CONV["xP"], CONV["b3z"])):
        for e in (1, -1):
            frame.append((sx - 0.02, sx + 0.02, y + e * (hw + 0.08) - 0.02, y + e * (hw + 0.08) + 0.02, 0, z + sz + 0.04))
    mesh_obj("Conveyor_%s_frame" % L["id"], frame, MAT["conv"])
    mesh_obj("Conveyor_%s_chains" % L["id"], chain, MAT["steel"])
    mesh_obj("Conveyor_%s_end_stops" % L["id"], [(x0, x_face, y + by - 0.05, y + by + 0.05, z - 0.05, z + CONV["stopH"]) for by in PALLET["blockY"]], MAT["stop"])

# ---------- floor markings: drop stations and routes ----------
marks = []
for S in CELL["slots"] + [CELL["stack"]]:
    a, b = PALLET["sy"] / 2 + 0.1, PALLET["sx"] / 2 + 0.1
    for xa, xb, ya, yb in ((-a, a, -b, -b + 0.03), (-a, a, b - 0.03, b), (-a, -a + 0.03, -b, b), (a - 0.03, a, -b, b)):
        marks.append((S["x"] + xa, S["x"] + xb, S["y"] + ya, S["y"] + yb, 0, 0.002))
mesh_obj("Station_markings", marks, MAT["mark"])
for i, pts in enumerate(P["routes"]):
    cu = bpy.data.curves.new("Route_%02d" % i, "CURVE")
    cu.dimensions = "3D"
    sp = cu.splines.new("POLY")
    sp.points.add(len(pts) - 1)
    for p, (x, y) in zip(sp.points, pts):
        p.co = (x, y, 0.003, 1)
    cu.bevel_depth = 0.008
    ob = bpy.data.objects.new("Route_%02d" % i, cu)
    ob.data.materials.append(MAT["steel"])
    col.objects.link(ob)

# ---------- AMR: one empty per joint frame ----------
B = AMR["body"]
bw = B["w"] / 2
gc, ch = AMR["groundClear"] + 0.02, AMR["chassisH"]
body = empty("AMR_body")
mesh_obj("AMR_chassis", [
    (-1.05, -0.25, -bw, bw, gc, ch),
    (B["xr"], -1.05, -bw, bw, 0.36, ch),
    (-0.25, AMR["chassisXf"], -0.36, 0.36, gc, 0.50),
    (-0.25, AMR["chassisXf"], 0.44, 0.56, 0.32, 0.38),
    (-0.25, AMR["chassisXf"], -0.56, -0.44, 0.32, 0.38),
], MAT["body"], body)
mesh_obj("AMR_counterweight", [
    (B["xr"], -0.85, -bw + 0.03, bw - 0.03, ch, AMR["hoodH"]),
    (-0.85, -0.25, -bw + 0.06, bw - 0.06, ch, 0.88),
    (-1.24, -1.16, -0.04, 0.04, AMR["hoodH"], 2.30),
    (-1.27, -1.13, -0.07, 0.07, 2.30, 2.39),
], MAT["dark"], body)


def wheel(name, parent, r, w, loc):
    j = empty(name, parent, loc)
    mesh_obj(name + "_tyre", [], MAT["tyre"], j, cyls=[(r, w, 0, 0, 0)])
    mesh_obj(name + "_hub", [(-r * 0.5, r * 0.5, -w / 2 - 0.009, w / 2 + 0.009, -0.015, 0.015)], MAT["steel"], j, cyls=[(r * 0.55, w + 0.012, 0, 0, 0)])
    return j


wl = wheel("AMR_wheel_L", body, AMR["rLoad"], AMR["wLoad"], (0, AMR["track"] / 2, AMR["rLoad"]))
wr = wheel("AMR_wheel_R", body, AMR["rLoad"], AMR["wLoad"], (0, -AMR["track"] / 2, AMR["rLoad"]))
j2 = empty("AMR_J2_steer", body, (-AMR["L"], 0, AMR["rDrive"]))
mesh_obj("AMR_steer_fork", [(-0.05, 0.05, -0.10, -0.07, -0.02, 0.19), (-0.05, 0.05, 0.07, 0.10, -0.02, 0.19), (-0.12, 0.12, -0.12, 0.12, 0.165, 0.215)], MAT["steel"], j2)
j1 = wheel("AMR_J1_drive_wheel", j2, AMR["rDrive"], AMR["wDrive"], (0, 0, 0))

j3 = empty("AMR_J3_tilt", body, (XT, 0, ZT))
mx0, mx1, mw = MAST["x0"] - XT, MAST["x1"] - XT, MAST["w"] / 2
mesh_obj("AMR_mast", [
    (mx0, mx1, mw - 0.05, mw + 0.05, 0.10 - ZT, MAST["height"] - ZT),
    (mx0, mx1, -mw - 0.05, -mw + 0.05, 0.10 - ZT, MAST["height"] - ZT),
    (mx0, mx1, -mw, mw, MAST["height"] - ZT - 0.08, MAST["height"] - ZT),
    (mx0, mx0 + 0.04, -mw, mw, 0.75 - ZT, 0.83 - ZT),
], MAT["dark"], j3)
j4 = empty("AMR_J4_lift", j3)
cw = FORK["carriageW"] / 2
mesh_obj("AMR_carriage", [(mx1, mx1 + 0.025, -cw, cw, -0.04, 0.42)], MAT["dark"], j4)
j5 = empty("AMR_J5_shift", j4)
xf, th, tw, tl = MAST["aFace"], FORK["thick"], FORK["tineW"] / 2, FORK["length"]
tines, rest = [], [(xf - 0.045, xf - 0.02, -cw, cw, 0.30, 0.40), (xf - 0.045, xf - 0.02, -cw, cw, 0.02, 0.10)]
for e in (1, -1):
    y = e * FORK["tineY"]
    tines.append((xf - th, xf, y - tw, y + tw, -th, 0.42))  # shank
    tines.append((xf, xf + tl, y - tw, y + tw, -th, 0))  # tine, top face at z = 0
    rest.append((xf - 0.03, xf - 0.01, e * (cw - 0.02) - 0.02, e * (cw - 0.02) + 0.02, 0.40, FORK["backrestH"]))
for zb in (0.6, 0.8, FORK["backrestH"] - 0.02):
    rest.append((xf - 0.03, xf - 0.01, -cw, cw, zb - 0.02, zb + 0.02))
mesh_obj("AMR_fork", tines, MAT["fork"], j5)
mesh_obj("AMR_backrest", rest, MAT["dark"], j5)
tcp = empty("AMR_TCP", j5, (xf + FORK["loadCentre"], 0, 0))
tcp.empty_display_type = "ARROWS"

# ---------- pallets (EUR, nine blocks) with their boxes ----------
PL = PALLET
pal_boxes = []
for x, w in ((-0.3275, 0.145), (-0.1638, 0.1), (0, 0.145), (0.1638, 0.1), (0.3275, 0.145)):
    pal_boxes.append((x - w / 2, x + w / 2, -PL["sy"] / 2, PL["sy"] / 2, PL["h"] - 0.022, PL["h"]))
for y in PL["blockY"]:
    pal_boxes.append((-PL["sx"] / 2, PL["sx"] / 2, y - PL["blockLy"] / 2, y + PL["blockLy"] / 2, 0.100, 0.122))
    for x in PL["blockX"]:
        w = PL["blockLx"] if x else 0.145
        pal_boxes.append((x - w / 2, x + w / 2, y - PL["blockLy"] / 2, y + PL["blockLy"] / 2, 0.022, 0.100))
for x in PL["blockX"]:
    w = PL["blockLx"] if x else 0.145
    pal_boxes.append((x - w / 2, x + w / 2, -PL["sy"] / 2, PL["sy"] / 2, 0, 0.022))

pallets = []
for p in P["pallets"]:
    ob = mesh_obj("Pallet_%03d" % p["id"], pal_boxes, MAT["wood"])
    ob.rotation_mode = "XYZ"  # Blender XYZ = Rz(yaw)·Ry(−pitch)·Rx(0), spec §5.2
    if p["boxes"]:
        mesh_obj("Pallet_%03d_boxes" % p["id"], [(lx - sx / 2 + 0.003, lx + sx / 2 - 0.003, ly - sy / 2 + 0.003, ly + sy / 2 - 0.003, lz - sz / 2 + 0.002, lz + sz / 2 - 0.002) for sx, sy, sz, lx, ly, lz in p["boxes"]], MAT["sku%d" % p["sku"]], ob)
    pallets.append((ob, p))

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
for i, (x, y, th_, steer, wd, wl_, wr_, tilt, lift, shift) in enumerate(P["frames"]):
    fr = i + 1
    key(body, "location", (x, y, 0), fr, last)
    key(body, "rotation_euler", (0, 0, th_), fr, last)
    key(j2, "rotation_euler", (0, 0, steer), fr, last)  # q2
    key(j1, "rotation_euler", (0, wd, 0), fr, last)  # q1
    key(wl, "rotation_euler", (0, wl_, 0), fr, last)
    key(wr, "rotation_euler", (0, wr_, 0), fr, last)
    key(j3, "rotation_euler", (0, -tilt, 0), fr, last)  # q3: positive = back = about −Y
    key(j4, "location", (0, 0, lift - ZT), fr, last)  # q4
    key(j5, "location", (0, shift, 0), fr, last)  # q5

HIDDEN = (0.0, 0.0, 0.0)
for ob, p in pallets:
    first, n = p["first"] + 1, len(p["track"])
    if first > 1:  # not yet on the conveyor
        key(ob, "scale", HIDDEN, 1, last)
    for i, (x, y, z_, yaw, pitch) in enumerate(p["track"]):
        fr = first + i
        key(ob, "scale", (1.0, 1.0, 1.0), fr, last)
        key(ob, "location", (x, y, z_), fr, last)
        key(ob, "rotation_euler", (0, -pitch, yaw), fr, last)
    if first + n <= scene.frame_end:  # collected by the downstream process
        key(ob, "scale", HIDDEN, first + n, last)
scene.frame_set(1)

# ---------- optional GLB export:  blender -b -P build_amr.py -- --glb out.glb ----------
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
if "--glb" in argv:
    path = os.path.abspath(argv[argv.index("--glb") + 1])
    bpy.ops.export_scene.gltf(filepath=path, export_format="GLB", export_yup=True, export_animations=True)
    print("exported", path)

result = {"objects": len(col.objects), "frames": len(P["frames"]), "pallets": len(pallets)}
