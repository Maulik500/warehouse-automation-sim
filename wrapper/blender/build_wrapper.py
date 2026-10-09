# Builds the turntable wrapper cell in Blender from wrapper_params.json (written by
# tools/export-blender.mjs) and keyframes every joint, every pallet and the film with the
# simulated trajectory.
#
# Blender is Z-up and metric, exactly like WRAPPER_SPEC.md, so coordinates are used unchanged.
# Joint hierarchy (spec §3.4):
#   WR_J1_table (Rz q1) > plate, WR_J2_deck_roller_* (Ry q2 / r)
#                       > WR_G1_clamp_lift (Tz) > WR_G2_jaw_L / WR_G2_jaw_R (Ty)
#   WR_J3_carriage (Tz q3) > housing, exit roller, WR_film_roll, WR_J4_prestretch_* (Rz), WR_J5_dancer (Rz)
#   WR_G3_cut_slide (Ty)   > rod, brush, hot wire
#   Pallet_### (x, y, Rz)  > boxes, film bands (appear when the film lands)
#   WR_film_span: the free film from the exit roller to the load, one quad with keyed vertices
# The line conveyor rollers are static meshes; only the deck rollers turn.
#
# Run inside Blender:   Scripting tab > open this file > Run Script
# or headless:          blender -b -P build_wrapper.py -- --glb wrapper.glb

import bpy, bmesh, json, math, os, sys
from mathutils import Matrix

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "wrapper_params.json")) as f:
    P = json.load(f)
PALLET, SKUS, LINE, TABLE, CARRIAGE, FILM, CLAMP, CUT = (P[k] for k in ("PALLET", "SKUS", "LINE", "TABLE", "CARRIAGE", "FILM", "CLAMP", "CUT"))
EX, EY = CARRIAGE["roller"]
Z = LINE["z"]

COL_NAME = "WRAPPER_CELL"
old = bpy.data.collections.get(COL_NAME)
if old:  # rebuild only what this script created
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(old)
col = bpy.data.collections.new(COL_NAME)
bpy.context.scene.collection.children.link(col)


def material(name, rgb, metallic=0.2, rough=0.6, alpha=1.0):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*rgb, 1)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = rough
    if alpha < 1:
        bsdf.inputs["Alpha"].default_value = alpha
        for attr, val in (("blend_method", "BLEND"), ("surface_render_method", "BLENDED")):
            try:
                setattr(m, attr, val)
            except (AttributeError, TypeError):
                pass
        m.use_backface_culling = False
    return m


MAT = {
    "body": material("wr_body", (0.03, 0.16, 0.46), 0.2, 0.45),
    "dark": material("wr_dark", (0.02, 0.02, 0.03), 0.4, 0.6),
    "steel": material("wr_steel", (0.32, 0.36, 0.43), 0.7, 0.35),
    "roller": material("wr_roller", (0.48, 0.52, 0.58), 0.8, 0.3),
    "rubber": material("wr_rubber", (0.01, 0.01, 0.012), 0.0, 0.9),
    "conv": material("wr_conveyor", (0.07, 0.09, 0.12), 0.5, 0.6),
    "yellow": material("wr_yellow", (0.89, 0.44, 0.02), 0.2, 0.5),
    "wood": material("wr_wood", (0.48, 0.25, 0.09), 0.0, 0.85),
    "brush": material("wr_brush", (0.19, 0.10, 0.03), 0.0, 1.0),
    "film": material("wr_film", (0.55, 0.75, 1.0), 0.0, 0.1, 0.28),
    "roll": material("wr_film_roll", (0.75, 0.85, 0.95), 0.1, 0.15, 0.85),
}
for i, S in enumerate(SKUS):  # one material per box type (sRGB hex -> linear)
    c = S["color"]
    MAT["sku%d" % i] = material("wr_" + S["sku"], tuple((((c >> s) & 255) / 255) ** 2.2 for s in (16, 8, 0)), 0.0, 0.9)

AXIS = {"x": Matrix.Rotation(math.pi / 2, 4, "Y"), "y": Matrix.Rotation(math.pi / 2, 4, "X"), "z": Matrix.Identity(4)}


def mesh_obj(name, boxes, mat, parent=None, cyls=()):
    """One object made of boxes [(x0, x1, y0, y1, z0, z1), ...] and cylinders [(r, length, x, y, z, axis), ...]."""
    bm = bmesh.new()
    for x0, x1, y0, y1, z0, z1 in boxes:
        bmesh.ops.create_cube(bm, size=1.0, matrix=Matrix.Translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)) @ Matrix.Diagonal((x1 - x0, y1 - y0, z1 - z0, 1.0)))
    for r, w, x, y, z, ax in cyls:
        bmesh.ops.create_cone(bm, cap_ends=True, segments=32, radius1=r, radius2=r, depth=w, matrix=Matrix.Translation((x, y, z)) @ AXIS[ax])
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    col.objects.link(ob)
    if parent:
        ob.parent = parent
    return ob


def quad_obj(name, verts, mat, parent=None):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], [(0, 1, 2, 3)])
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


# ---------- line conveyors (spec §3.1) ----------
hw, rr = LINE["frameW"] / 2, LINE["rollR"]
for name, x0, x1 in (("Infeed", -LINE["xFar"], -LINE["xEnd"]), ("Outfeed", LINE["xEnd"], LINE["xFar"])):
    frame, rollers = [], []
    for e in (1, -1):
        frame.append((x0, x1, e * hw - 0.03, e * hw + 0.03, Z - 0.14, Z + 0.01))
        frame.append((x0, x1, e * (hw - 0.03) - 0.006, e * (hw - 0.03) + 0.006, Z + 0.01, Z + 0.09))
        x = x0 + 0.15
        while x < x1:
            frame.append((x - 0.04, x + 0.04, e * hw - 0.04, e * hw + 0.04, 0, Z - 0.14))
            x += 1.3
    x = x0 + 0.15
    while x < x1:
        frame.append((x - 0.03, x + 0.03, -hw, hw, Z - 0.30, Z - 0.22))
        x += 1.3
    for i in range(int((x1 - x0 - 0.08) / LINE["rollPitch"]) + 1):
        rollers.append((rr, LINE["rollLen"], x0 + 0.04 + i * LINE["rollPitch"], 0, Z - rr, "y"))
    mesh_obj("Conveyor_%s_frame" % name, frame, MAT["conv"])
    mesh_obj("Conveyor_%s_rollers" % name, [], MAT["roller"], cyls=rollers)
x_i1 = -LINE["pitch"]
mesh_obj("Sensor_BH_light_grid", [(x_i1 - 0.03, x_i1 + 0.03, e * 0.66 - 0.03, e * 0.66 + 0.03, 0, Z + PALLET["maxH"] + 0.15) for e in (1, -1)], MAT["yellow"])
posts = []
for x, y in ((x_i1 + PALLET["sx"] / 2 - 0.05, hw + 0.07), (-0.915, 0.75), (0.915, 0.75), (LINE["pitch"] + PALLET["sx"] / 2 - 0.05, hw + 0.07)):
    posts += [(x - 0.02, x + 0.02, e * y - 0.02, e * y + 0.02, 0, Z + 0.11) for e in (1, -1)]
mesh_obj("Sensor_photo_eyes_B1_B2_B4_B5", posts, MAT["dark"])

# ---------- turntable: q1, q2, clamp g1, g2 ----------
mesh_obj("WR_pedestal", [], MAT["dark"], cyls=[(0.55, 0.30, 0, 0, 0.17, "z"), (0.62, 0.04, 0, 0, 0.34, "z")])
j1 = empty("WR_J1_table")
pz = TABLE["plateZ"]
mesh_obj("WR_plate", [], MAT["body"], j1, cyls=[(TABLE["rPlate"], 0.04, 0, 0, pz - 0.02, "z")])
mesh_obj("WR_deck_frame", [(-0.74, 0.74, e * 0.50 - 0.02, e * 0.50 + 0.02, pz, Z - 0.02) for e in (1, -1)], MAT["conv"], j1)
mesh_obj("WR_index_mark", [(TABLE["rPlate"] - 0.10, TABLE["rPlate"] - 0.01, -0.03, 0.03, pz, pz + 0.004)], MAT["yellow"], j1)
deck = []
n_deck = int(round(2 * TABLE["deckX"] / LINE["rollPitch"])) + 1
for i in range(n_deck):
    x = -TABLE["deckX"] + i * LINE["rollPitch"]
    ln = min(LINE["rollLen"], 2 * math.sqrt(0.88 ** 2 - x * x) - 0.04)
    j = empty("WR_J2_deck_roller_%02d" % i, j1, (x, 0, Z - rr))
    mesh_obj("WR_deck_roller_%02d_mesh" % i, [(-0.007, 0.007, -ln / 2 + 0.03, ln / 2 - 0.03, rr - 0.002, rr + 0.002)], MAT["roller"], j, cyls=[(rr, ln, 0, 0, 0, "y")])
    deck.append(j)
cb = empty("WR_clamp_base", j1, (CLAMP["x"], CLAMP["y"], 0))
cb.rotation_euler = (0, 0, CLAMP["yaw"])  # local X = film line of the home pose
mesh_obj("WR_clamp_cylinder", [(-0.04, 0.04, -0.045, 0.045, pz, CLAMP["zDown"] - 0.135)], MAT["dark"], cb)
g1 = empty("WR_G1_clamp_lift", cb)
mesh_obj("WR_clamp_rod", [(-0.035, 0.035, -0.04, 0.04, -0.135, -0.115)], MAT["steel"], g1, cyls=[(0.016, 0.16, 0, 0, -0.20, "z")])
jaws = []
for e, nm in ((1, "L"), (-1, "R")):
    j = empty("WR_G2_jaw_" + nm, g1)
    mesh_obj("WR_jaw_%s_mesh" % nm, [(-0.03, 0.03, -0.006, 0.006, -0.125, 0)], MAT["yellow"], j)
    jaws.append((j, e))

# ---------- mast and carriage: q3, q4, q5 ----------
m = CARRIAGE["mast"]
mesh_obj("WR_mast", [
    (m["x"] - m["sx"] / 2, m["x"] + m["sx"] / 2, m["y"] - m["sy"] / 2, m["y"] + m["sy"] / 2, 0, m["h"]),
    (m["x"] - 0.35, m["x"] + 0.35, m["y"] - 0.30, m["y"] + 0.25, 0, 0.03),
], MAT["body"])
mesh_obj("WR_lift_drive", [(m["x"] - 0.14, m["x"] + 0.14, m["y"] - 0.16, m["y"] + 0.16, m["h"], m["h"] + 0.16)], MAT["dark"])
j3 = empty("WR_J3_carriage", None, (EX, EY, 0))
B = CARRIAGE["body"]
bx0, bx1, by0, by1, bh = B["x0"] - EX, B["x1"] - EX, B["y0"] - EY, B["y1"] - EY, B["h"] / 2
mesh_obj("WR_carriage_housing", [
    (bx0, bx1, by0, by1, bh - 0.02, bh),
    (bx0, bx1, by0, by1, -bh, -bh + 0.02),
    (bx0, bx1, by0, by0 + 0.02, -bh, bh),
    (m["x"] - EX - 0.06, m["x"] - EX + 0.06, m["y"] + m["sy"] / 2 - EY, by0, -0.12, 0.12),
], MAT["body"], j3)
w0 = FILM["w0"]
mesh_obj("WR_exit_roller", [], MAT["roller"], j3, cyls=[(CARRIAGE["rExit"], w0 + 0.04, 0, 0, 0, "z")])
ROLL = (-0.31, -0.12)
mesh_obj("WR_film_core", [], MAT["wood"], j3, cyls=[(FILM["rCore"], w0 + 0.03, ROLL[0], ROLL[1], 0, "z")])
roll = mesh_obj("WR_film_roll", [(0.3, 0.6, -0.01, 0.01, w0 / 2, w0 / 2 + 0.004)], MAT["roll"], j3, cyls=[(1.0, w0, 0, 0, 0, "z")])
roll.location = (ROLL[0], ROLL[1], 0)
roll.rotation_mode = "XYZ"
stretch = []
for i, (x, y) in enumerate(((-0.15, -0.20), (-0.09, -0.10))):
    j = empty("WR_J4_prestretch_%d" % (i + 1), j3, (x, y, 0))
    mesh_obj("WR_prestretch_%d_mesh" % (i + 1), [(-0.025, 0.025, -0.004, 0.004, w0 / 2 + 0.01, w0 / 2 + 0.016)], MAT["rubber"], j, cyls=[(0.035, w0 + 0.02, 0, 0, 0, "z")])
    stretch.append(j)
arm = FILM["dancer"]["arm"]
j5 = empty("WR_J5_dancer", j3, (-0.03, -0.21, 0))
mesh_obj("WR_dancer_arm", [(-0.012, 0.012, 0, arm, bh - 0.06, bh - 0.04)], MAT["steel"], j5, cyls=[(0.022, w0, 0, arm, 0, "z")])

# ---------- cut-and-press slide: g3 ----------
cx, bw, cz0, cz1, wb, wx = CUT["x"], CUT["brushW"] / 2, CUT["z"][0], CUT["z"][1], CUT["wireBack"], CUT["wireX"]
mesh_obj("WR_cut_stand", [(cx - 0.05, cx + 0.05, -1.90, -1.80, 0, 0.74), (cx - 0.035, cx + 0.035, -2.35, CUT["park"], 0.74, 0.77)], MAT["dark"])
g3 = empty("WR_G3_cut_slide", None, (0, CUT["park"], 0))
mesh_obj("WR_cut_rod", [(cx - 0.02, cx + 0.02, -1.02, -0.03, 0.77, 0.81)], MAT["yellow"], g3)
mesh_obj("WR_cut_brush", [(cx - bw, cx + bw, -0.025, 0, cz0, cz1)], MAT["brush"], g3)
mesh_obj("WR_cut_frame", [(cx - bw, cx + bw, -0.05, -0.025, cz0, cz1)] + [(wx - 0.006, cx, -wb - 0.006, -wb + 0.006, z - 0.012, z + 0.012) for z in (cz0, cz1)], MAT["dark"], g3)
mesh_obj("WR_hot_wire", [], MAT["steel"], g3, cyls=[(0.003, cz1 - cz0, wx, -wb, (cz0 + cz1) / 2, "z")])

# ---------- pallets with their boxes and film bands ----------
PL = PALLET
sx, sy = PL["sx"], PL["sy"]
pal_boxes = []
for y, w in ((-0.3275, 0.145), (-0.1638, 0.1), (0, 0.145), (0.1638, 0.1), (0.3275, 0.145)):
    pal_boxes.append((-sx / 2, sx / 2, y - w / 2, y + w / 2, PL["h"] - 0.022, PL["h"]))
for x in (-0.5275, 0, 0.5275):
    pal_boxes.append((x - 0.0725, x + 0.0725, -sy / 2, sy / 2, 0.100, 0.122))
    for y in (-0.35, 0, 0.35):
        w = 0.10 if y else 0.145
        pal_boxes.append((x - 0.0725, x + 0.0725, y - w / 2, y + w / 2, 0.022, 0.100))
for y in (-0.35, 0, 0.35):
    w = 0.10 if y else 0.145
    pal_boxes.append((-sx / 2, sx / 2, y - w / 2, y + w / 2, 0, 0.022))

pallets, bands = [], []
for p in P["pallets"]:
    ob = mesh_obj("Pallet_%03d" % p["id"], pal_boxes, MAT["wood"])
    ob.rotation_mode = "XYZ"
    mesh_obj("Pallet_%03d_boxes" % p["id"], [(lx - bsx / 2 + 0.003, lx + bsx / 2 - 0.003, ly - bsy / 2 + 0.003, ly + bsy / 2 - 0.003, lz - bsz / 2 + 0.002, lz + bsz / 2 - 0.002) for bsx, bsy, bsz, lx, ly, lz in p["boxes"]], MAT["sku%d" % p["sku"]], ob)
    pallets.append((ob, p))
    for k, (f0, f1, ax, ay, az0, az1, bx, by, bz0, bz1) in enumerate(p["bands"]):
        L = math.hypot(bx - ax, by - ay)
        if L < 1e-6:
            continue
        o = 0.003 + 0.0005 * k  # later wraps lie further out
        nx, ny = -(by - ay) / L * o, (bx - ax) / L * o
        q = quad_obj("Pallet_%03d_film_%03d" % (p["id"], k), [(ax + nx, ay + ny, az0 - Z), (bx + nx, by + ny, bz0 - Z), (bx + nx, by + ny, bz1 - Z), (ax + nx, ay + ny, az1 - Z)], MAT["film"], ob)
        bands.append((q, int(f0), int(f1)))
span = quad_obj("WR_film_span", [(0, 0, 0)] * 4, MAT["film"])

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
half = FILM["w"] / 2
prev_span = None
for i, fr_data in enumerate(P["frames"]):
    th, lift, up, jaw, slide, dancer, feed, roll_a, roll_r, wire = fr_data[:10]
    travel = fr_data[10:]
    fr = i + 1
    key(j1, "rotation_euler", (0, 0, th), fr, last)  # q1
    for j in deck:
        key(j, "rotation_euler", (0, travel[3] / rr, 0), fr, last)  # q2
    key(j3, "location", (EX, EY, lift), fr, last)  # q3
    key(stretch[0], "rotation_euler", (0, 0, feed / FILM["lambda"] / 0.035), fr, last)  # q4, before the stretch
    key(stretch[1], "rotation_euler", (0, 0, -feed / 0.035), fr, last)  # q4, after the stretch
    key(roll, "rotation_euler", (0, 0, -roll_a), fr, last)
    key(roll, "scale", (roll_r, roll_r, 1.0), fr, last)
    key(j5, "rotation_euler", (0, 0, -dancer / arm), fr, last)  # q5
    key(g1, "location", (0, 0, CLAMP["zDown"] + CLAMP["stroke"] * up), fr, last)  # g1
    for j, e in jaws:
        key(j, "location", (0, e * (0.006 + CLAMP["open"] / 2 * (1 - jaw)), 0), fr, last)  # g2
    key(g3, "location", (0, slide, 0), fr, last)  # g3
    # free film: roller tangent point at the carriage height -> contact point on the load
    s = P["span"][i]
    co = [(EX, EY, lift)] * 4 if s is None else [(s[0], s[1], lift - half), (s[2], s[3], s[4]), (s[2], s[3], s[5]), (s[0], s[1], lift + half)]
    if co != prev_span or fr == scene.frame_end:
        for v, c in zip(span.data.vertices, co):
            v.co = c
            v.keyframe_insert("co", frame=fr)
        prev_span = co

HIDDEN = (0.0, 0.0, 0.0)
SHOWN = (1.0, 1.0, 1.0)
for ob, p in pallets:
    first, n = p["first"] + 1, len(p["track"])
    if first > 1:  # not yet on the line
        key(ob, "scale", HIDDEN, 1, last)
    for i, (x, y, yaw) in enumerate(p["track"]):
        fr = first + i
        key(ob, "scale", SHOWN, fr, last)
        key(ob, "location", (x, y, Z), fr, last)
        key(ob, "rotation_euler", (0, 0, yaw), fr, last)
    if first + n <= scene.frame_end:  # left the line
        key(ob, "scale", HIDDEN, first + n, last)
for q, f0, f1 in bands:  # a film band appears when it lands on the load
    if f0 + 1 > 1:
        key(q, "scale", HIDDEN, 1, last)
    key(q, "scale", SHOWN, f0 + 1, last)
    if f1 >= 0:
        key(q, "scale", HIDDEN, f1 + 1, last)
for ob in col.objects:  # stepped keys for appear / disappear would be ideal; constant interpolation is close enough here
    ad = ob.animation_data
    if ad and ad.action and hasattr(ad.action, "fcurves"):
        for fc in ad.action.fcurves:
            if fc.data_path == "scale" and not ob.name.startswith("WR_film_roll"):
                for kp in fc.keyframe_points:
                    kp.interpolation = "CONSTANT"
scene.frame_set(1)

# ---------- optional GLB export:  blender -b -P build_wrapper.py -- --glb out.glb ----------
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
if "--glb" in argv:
    path = os.path.abspath(argv[argv.index("--glb") + 1])
    bpy.ops.export_scene.gltf(filepath=path, export_format="GLB", export_yup=True, export_animations=True)
    print("exported", path)

result = {"objects": len(col.objects), "frames": len(P["frames"]), "pallets": len(pallets), "film_bands": len(bands)}
