# Builds the integrated warehouse in Blender from warehouse_params.json (written by
# tools/export-blender.mjs) and keyframes every joint of every machine and vehicle, every pallet
# and every box with the simulated trajectory.
#
# Blender is Z-up and metric, exactly like WAREHOUSE_SPEC.md, so global coordinates {G} are used
# unchanged. Each machine is one empty per joint frame of its own specification:
#   ASRS            > Aisle_n > CRn_travel (X q1) > CRn_hoist (Z q2) > CRn_fork_mid (Y q3/2), CRn_fork_top (Y q3)
#   PALn (t, Rz ψ)  > Rn_J1 (Rz) > Rn_J2 (Ry) > Rn_J3 (Ry) > Rn_J4 (Rx) > Rn_J5 (Ry) > Rn_J6 (Rx) > Rn_flange > Rn_TCP
#   WRAP            > WR_table (Rz q1) > WR_clamp (Z g1);  WR_carriage (Z q3);  WR_cut_slide (Y g3)
#   Fn_body (x,y,θ) > Fn_steer (Rz q2);  Fn_mast (Ry −q3) > Fn_carriage (Z q4) > Fn_shift (Y q5)
#   Mn_body (x,y,θ) > Mn_arm_base {A} > Mn_J1 (Rz) > Mn_F1 (DH row 1) > … > Mn_J6 > Mn_F6 = flange;  Mn_front_stop (Z)
#
# Run inside Blender:   Scripting tab > open this file > Run Script
# or headless:          blender -b -P build_warehouse.py -- --save warehouse.blend [--glb warehouse.glb]

import bpy, bmesh, json, math, os, sys
from mathutils import Matrix

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else bpy.path.abspath("//")
with open(os.path.join(HERE, "warehouse_params.json")) as f:
    P = json.load(f)
PI = math.pi
COL_NAME = "WAREHOUSE"
old = bpy.data.collections.get(COL_NAME)
if old:  # rebuild only what this script created
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(old)
col = bpy.data.collections.new(COL_NAME)
bpy.context.scene.collection.children.link(col)
bpy.context.preferences.edit.keyframe_new_interpolation_type = "LINEAR"


def material(name, rgb, metallic=0.2, rough=0.6, alpha=1.0):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (*rgb, 1)
    b.inputs["Metallic"].default_value = metallic
    b.inputs["Roughness"].default_value = rough
    if alpha < 1:
        b.inputs["Alpha"].default_value = alpha
        try:
            m.surface_render_method = "BLENDED"
        except Exception:
            m.blend_method = "BLEND"
    return m


MAT = {
    "yellow": material("wh_yellow", (0.89, 0.44, 0.02), 0.2, 0.45), "blue": material("wh_blue", (0.05, 0.16, 0.46), 0.2, 0.5),
    "orange": material("wh_orange", (0.75, 0.17, 0.02), 0.3, 0.5), "robot": material("wh_robot", (0.80, 0.13, 0.01), 0.25, 0.45),
    "dark": material("wh_dark", (0.02, 0.02, 0.03), 0.4, 0.6), "steel": material("wh_steel", (0.32, 0.36, 0.43), 0.7, 0.35),
    "roller": material("wh_roller", (0.48, 0.52, 0.58), 0.8, 0.3), "tyre": material("wh_tyre", (0.01, 0.01, 0.012), 0.0, 0.9),
    "conv": material("wh_conveyor", (0.07, 0.09, 0.12), 0.5, 0.6), "rack": material("wh_rack", (0.03, 0.11, 0.34), 0.4, 0.6),
    "beam": material("wh_beam", (0.68, 0.19, 0.02), 0.3, 0.6), "wood": material("wh_wood", (0.48, 0.25, 0.09), 0.0, 0.85),
    "arm": material("wh_arm", (0.70, 0.73, 0.78), 0.3, 0.4), "joint": material("wh_joint", (0.07, 0.36, 1.0), 0.3, 0.4),
    "floor": material("wh_floor", (0.015, 0.018, 0.022), 0.0, 0.9), "road": material("wh_road", (0.89, 0.44, 0.02), 0.0, 0.8),
    "road_mm": material("wh_road_mm", (0.07, 0.36, 1.0), 0.0, 0.8), "film": material("wh_film", (0.6, 0.8, 1.0), 0.0, 0.15, 0.3),
}
for S in P["SKUS"]:  # one material per SKU (sRGB hex -> linear)
    c = S["color"]
    MAT[S["id"]] = material("wh_" + S["id"], tuple((((c >> s) & 255) / 255) ** 2.2 for s in (16, 8, 0)), 0.0, 0.9)
AXIS_ROT = {"x": Matrix.Rotation(PI / 2, 4, "Y"), "y": Matrix.Rotation(PI / 2, 4, "X"), "z": Matrix.Identity(4)}


def mesh_obj(name, boxes, mat, parent=None, cyls=(), seg=20):
    """Boxes [(x0, x1, y0, y1, z0, z1)] and cylinders [(axis, a, b, r, u, v)] as one object."""
    bm = bmesh.new()
    for x0, x1, y0, y1, z0, z1 in boxes:
        bmesh.ops.create_cube(bm, size=1.0, matrix=Matrix.Translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)) @ Matrix.Diagonal((abs(x1 - x0), abs(y1 - y0), abs(z1 - z0), 1.0)))
    for axis, a, b, r, u, v in cyls:
        mid = (a + b) / 2
        loc = {"x": (mid, u, v), "y": (u, mid, v), "z": (u, v, mid)}[axis]
        bmesh.ops.create_cone(bm, cap_ends=True, segments=seg, radius1=r, radius2=r, depth=abs(b - a), matrix=Matrix.Translation(loc) @ AXIS_ROT[axis])
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    ob.rotation_mode = "XYZ"
    col.objects.link(ob)
    if parent:
        ob.parent = parent
    return ob


def empty(name, parent=None, loc=(0, 0, 0), rot=(0, 0, 0), size=0.15):
    ob = bpy.data.objects.new(name, None)
    ob.empty_display_size = size
    ob.location = loc
    ob.rotation_mode = "XYZ"
    ob.rotation_euler = rot
    col.objects.link(ob)
    if parent:
        ob.parent = parent
    return ob


def frange(a, b, step):
    out = []
    while a < b:
        out.append(a)
        a += step
    return out


B, PL = P["BUILDING"], P["PALLET"]
mesh_obj("Floor", [(0, B["x"], 0, B["y"], -0.05, 0)], MAT["floor"])


def road(name, lines, mat, w=0.05):
    bm = bmesh.new()
    for pts in lines:
        for (xa, ya), (xb, yb) in zip(pts[:-1], pts[1:]):
            dx, dy = xb - xa, yb - ya
            L = math.hypot(dx, dy)
            if L < 1e-6:
                continue
            nx, ny = -dy / L * w / 2, dx / L * w / 2
            vs = [bm.verts.new((x, y, 0.004)) for x, y in ((xa - nx, ya - ny), (xb - nx, yb - ny), (xb + nx, yb + ny), (xa + nx, ya + ny))]
            bm.faces.new(vs)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    me.materials.append(mat)
    col.objects.link(bpy.data.objects.new(name, me))


road("Roads_forklift", P["roads"]["fork"], MAT["road"])
road("Roads_robot_AMR", P["roads"]["mm"], MAT["road_mm"])

# ---------- ASRS (ASRS_SPEC §3) ----------
A, RK, CR, AF, PD = P["PLACE"]["asrs"], P["RACK"], P["CRANE"], P["AFORK"], P["PD"]
asrs = empty("ASRS", loc=(A["tx"], A["ty"], 0), size=1.0)
xa = RK["x1"] - RK["pitchX"] / 2
xb = xa + RK["bays"] * RK["pitchX"]
ups, beams, CRN = [], [], []
for a in P["LAYOUT"]["aisles"]:
    for row, side in a["rows"]:
        yf, yb = a["y"] + side * RK["yFace"], a["y"] + side * (RK["yFace"] + RK["depth"])
        for y in (yf + side * 0.05, yb - side * 0.05):
            ups += [(xa + b * RK["pitchX"] - 0.05, xa + b * RK["pitchX"] + 0.05, y - 0.05, y + 0.05, 0, RK["height"]) for b in range(RK["bays"] + 1)]
            beams += [(xa, xb, y - 0.025, y + 0.025, RK["z1"] + l * RK["pitchZ"] - 0.11, RK["z1"] + l * RK["pitchZ"] - 0.01) for l in range(RK["levels"])]
    ag = empty("Aisle_%d" % a["id"], asrs, (0, a["y"], 0))
    n = a["id"]
    mesh_obj("CR%d_rail" % n, [(-1.6, 20.6, -0.05, 0.05, 0, 0.06)], MAT["steel"], ag)
    trv = empty("CR%d_travel" % n, ag)
    mx, sx, sy = CR["mastX"], CR["mastSx"], CR["mastSy"]
    mesh_obj("CR%d_structure" % n, [(-CR["chassisLen"] / 2, CR["chassisLen"] / 2, -CR["chassisW"] / 2, CR["chassisW"] / 2, 0.06, 0.06 + CR["chassisH"]), (-mx, mx, -0.15, 0.15, CR["mastH"] - 0.25, CR["mastH"])] + [(e * mx - sx / 2, e * mx + sx / 2, -sy / 2, sy / 2, 0.3, CR["mastH"]) for e in (1, -1)], MAT["yellow"], trv)
    hst = empty("CR%d_hoist" % n, trv)
    mesh_obj("CR%d_carriage" % n, [(-CR["carriageSx"] / 2, CR["carriageSx"] / 2, -CR["carriageSy"] / 2, CR["carriageSy"] / 2, -CR["carriageDrop"], -0.07)], MAT["dark"], hst)
    h = AF["length"] / 2
    tines = lambda z0, z1: [(e * AF["tineX"] - AF["tineW"] / 2, e * AF["tineX"] + AF["tineW"] / 2, -h, h, z0, z1) for e in (1, -1)]
    mesh_obj("CR%d_fork_base" % n, tines(-0.07, -0.045), MAT["steel"], hst)
    mid = empty("CR%d_fork_mid" % n, hst)
    mesh_obj("CR%d_fork_mid_stage" % n, tines(-0.045, -0.02), MAT["roller"], mid)
    top = empty("CR%d_fork_top" % n, hst)
    mesh_obj("CR%d_fork_top_stage" % n, tines(-0.02, 0), MAT["orange"], top)
    CRN.append((trv, hst, mid, top))
mesh_obj("Rack_uprights", ups, MAT["rack"], asrs)
mesh_obj("Rack_beams", beams, MAT["beam"], asrs)
STOPS = []
for L in P["lanes"]:
    f = L["frame"]
    fr = []
    for e in (1, -1):
        ya, yb = sorted((f["y"] + e * f["hw"] - e * 0.04, f["y"] + e * f["hw"]))
        fr.append((f["x0"], f["x1"] + 1.2, ya, yb, f["top"] - 0.16, f["top"]))
        fr += [(x - 0.03, x + 0.03, ya, yb, 0, f["top"] - 0.16) for x in frange(f["x0"] + 0.2, f["x1"] + 1.2, 1.2)]
    mesh_obj("Lane_%s_frame" % L["row"], fr, MAT["conv"])
    mesh_obj("Lane_%s_rollers" % L["row"], [], MAT["roller"], cyls=[("y", f["y"] - f["hw"] + 0.05, f["y"] + f["hw"] - 0.05, 0.04, x, f["top"] - 0.04) for x in frange(f["x0"] + 0.1, f["x1"] + 1.0, 0.16)], seg=10)
    st = empty("Lane_%s_stops" % L["row"], loc=(0, 0, -0.09))
    mesh_obj("Lane_%s_stop_blocks" % L["row"], [(f["x0"] + 0.005, f["x0"] + 0.03, f["y"] + dy - 0.07, f["y"] + dy + 0.07, f["top"], f["top"] + 0.085) for dy in (-0.5275, 0, 0.5275)], MAT["orange"], st)
    STOPS.append(st)

# ---------- palletizing cells (ROBOT_SPEC §3.1 + WAREHOUSE_SPEC §8.3, §8.4) ----------
RB, CL, PC, PT = P["ROBOT"], P["CELL"], P["PCELL"], P["PTOOL"]
cv, R, I, hw = CL["conv"], PC["recv"], PC["incline"], PC["beltW"] / 2
RJ = []
for C in P["PLACE"]["pal"]:
    n, tz = C["id"], C["tz"]
    g = empty("PAL%d" % n, loc=(C["tx"], C["ty"], tz), rot=(0, 0, C["psi"]), size=0.8)
    mesh_obj("PAL%d_riser" % n, [(-0.6, 0.6, -0.6, 0.6, -tz, 0)], MAT["dark"], g)
    yc, x0 = PC["build"]["y"], PC["dock"]["x"] - PL["sy"] / 2 - 0.03
    mesh_obj("PAL%d_conveyors" % n, [(cv["x"] - hw, cv["x"] + hw, PC["merge"]["y"] - 0.45, cv["yEnd"], cv["top"] - 0.09, cv["top"] - 0.005)]
             + [(cv["x"] + e * hw, cv["x"] + e * (hw + 0.04), PC["merge"]["y"] - 0.45, cv["yEnd"], -tz, cv["top"] + 0.01) for e in (1, -1)]
             + [(R["x0"], R["x1"], R["y"] + e * R["w"] / 2, R["y"] + e * (R["w"] / 2 + 0.04), -tz, R["z"] + 0.05) for e in (1, -1)]
             + [(x0, 0.6, yc + e * 0.42 - 0.03, yc + e * 0.42 + 0.03, -tz, -0.002) for e in (1, -1)], MAT["conv"], g)
    mesh_obj("PAL%d_receiving_rollers" % n, [], MAT["roller"], g, cyls=[("y", R["y"] - R["w"] / 2, R["y"] + R["w"] / 2, 0.025, x, R["z"] - 0.025) for x in frange(R["x0"] + 0.04, R["x1"], 0.075)], seg=10)
    inc = mesh_obj("PAL%d_incline" % n, [(-(I["x1"] - I["x0"]) / 2, (I["x1"] - I["x0"]) / 2, -R["w"] / 2, R["w"] / 2, -0.04, 0)], MAT["dark"], g)
    inc.location = ((I["x0"] + I["x1"]) / 2, R["y"], (I["z0"] + I["z1"]) / 2)
    inc.rotation_euler = (0, math.atan2(I["z0"] - I["z1"], I["x1"] - I["x0"]), 0)
    d1, a1, a2, a3, d4, d6 = (RB[k] for k in ("d1", "a1", "a2", "a3", "d4", "d6"))
    mesh_obj("R%d_base" % n, [], MAT["dark"], g, cyls=[("z", 0, 0.22, 0.42, 0, 0)])
    j1 = empty("R%d_J1" % n, g)
    mesh_obj("R%d_link1" % n, [(-0.15, a1 + 0.2, -0.3, 0.3, d1 - 0.28, d1 + 0.12)], MAT["robot"], j1, cyls=[("z", 0.22, d1 - 0.1, 0.33, 0, 0)])
    j2 = empty("R%d_J2" % n, j1, (a1, 0, d1))
    mesh_obj("R%d_link2" % n, [(-0.13, 0.13, -0.17, 0.17, 0, a2)], MAT["robot"], j2, cyls=[("y", -0.33, 0.33, 0.2, 0, 0)])
    j3 = empty("R%d_J3" % n, j2, (0, 0, a2))
    mesh_obj("R%d_link3" % n, [(-0.3, 0.35, -0.2, 0.2, -0.14, a3 + 0.16)], MAT["robot"], j3, cyls=[("y", -0.28, 0.28, 0.17, 0, 0)])
    j4 = empty("R%d_J4" % n, j3, (0, 0, a3))
    mesh_obj("R%d_link4" % n, [], MAT["robot"], j4, cyls=[("x", 0.35, d4 - 0.12, 0.1, 0, 0)])
    j5 = empty("R%d_J5" % n, j4, (d4, 0, 0))
    mesh_obj("R%d_link5" % n, [], MAT["dark"], j5, cyls=[("y", -0.13, 0.13, 0.11, 0, 0)])
    j6 = empty("R%d_J6" % n, j5)
    mesh_obj("R%d_link6" % n, [], MAT["steel"], j6, cyls=[("x", 0.05, d6, 0.08, 0, 0)])
    fl = empty("R%d_flange" % n, j6, (d6, 0, 0), (0, PI / 2, 0))
    mesh_obj("R%d_gripper" % n, [(-PT["padX"] / 2, PT["padX"] / 2, -PT["padY"] / 2, PT["padY"] / 2, 0.25, 0.272)], MAT["steel"], fl,
             cyls=[("z", 0, 0.25, 0.04, 0, 0)] + [("z", 0.272, 0.3, PT["cupDia"] / 2 - 0.004, (i - 1) * PT["colX"], e * PT["rowY"]) for i in range(3) for e in (-1, 1)])
    empty("R%d_TCP" % n, fl, (0, 0, 0.3), size=0.08)
    RJ.append((j1, j2, j3, j4, j5, j6))

# ---------- wrapper (WRAPPER_SPEC §3.4) ----------
WP, LN, TB, CA, CP, CT, WX = P["PLACE"]["wrap"], P["LINE"], P["TABLE"], P["CARRIAGE"], P["CLAMP"], P["CUT"], P["WRAPX"]
wr = empty("WRAP", loc=(WP["tx"], WP["ty"], 0), size=0.8)
fr, rl, lhw = [], [], LN["frameW"] / 2
for s in (1, -1):
    xs, xe = sorted((s * LN["xEnd"], s * LN["xFar"]))
    fr.append((xs, xe, -LN["rollLen"] / 2, LN["rollLen"] / 2, LN["z"] - 0.1, LN["z"] - 0.06))
    for e in (1, -1):
        ya, yb = sorted((e * lhw - e * 0.04, e * lhw))
        fr.append((xs, xe, ya, yb, LN["z"] - 0.16, LN["z"]))
        fr += [(x - 0.03, x + 0.03, ya, yb, 0, LN["z"] - 0.16) for x in frange(xs + 0.2, xe, 1.3)]
    rl += [("y", -LN["rollLen"] / 2, LN["rollLen"] / 2, LN["rollR"], x, LN["z"] - LN["rollR"]) for x in frange(xs + 0.08, xe, LN["rollPitch"])]
mesh_obj("WR_line_frame", fr, MAT["conv"], wr)
mesh_obj("WR_line_rollers", [], MAT["roller"], wr, cyls=rl, seg=10)
mesh_obj("WR_pedestal", [], MAT["dark"], wr, cyls=[("z", 0, TB["plateZ"] - 0.08, 0.45, 0, 0)])
table = empty("WR_table", wr)
mesh_obj("WR_plate", [], MAT["steel"], table, cyls=[("z", TB["plateZ"] - 0.08, TB["plateZ"], TB["rPlate"], 0, 0)], seg=48)
mesh_obj("WR_deck_rollers", [], MAT["roller"], table, cyls=[("y", -0.45, 0.45, LN["rollR"], -TB["deckX"] + 0.16 * k, TB["z"] - LN["rollR"]) for k in range(11)], seg=10)
clamp = empty("WR_clamp", table, (CP["x"], CP["y"], 0), (0, 0, CP["yaw"]))
mesh_obj("WR_clamp_jaws", [(-0.05, 0.05, e * 0.02, e * 0.035, CP["zDown"] - 0.12, CP["zDown"]) for e in (1, -1)], MAT["orange"], clamp)
ms, bd = CA["mast"], CA["body"]
mesh_obj("WR_mast", [(ms["x"] - ms["sx"] / 2, ms["x"] + ms["sx"] / 2, ms["y"] - ms["sy"] / 2, ms["y"] + ms["sy"] / 2, 0, ms["h"])], MAT["blue"], wr)
car = empty("WR_carriage", wr)
mesh_obj("WR_carriage_housing", [(bd["x0"], bd["x1"], bd["y0"], bd["y1"], -bd["h"] / 2, bd["h"] / 2)], MAT["blue"], car, cyls=[("z", -0.25, 0.25, CA["rExit"], CA["roller"][0], CA["roller"][1])])
slide = empty("WR_cut_slide", wr)
mesh_obj("WR_brush", [(CT["x"] - 0.05, CT["x"] + 0.05, -0.06, 0, CT["z"][0], CT["z"][1]), (CT["x"] - 0.03, CT["x"] + 0.03, -0.5, -0.06, 0.74, 0.84)], MAT["orange"], slide)

# ---------- forklifts (AMR_SPEC §3.2) ----------
FA, MS, FF = P["FAMR"], P["MAST"], P["FFORK"]


def wheel(name, parent, r, w, loc):
    g = empty(name, parent, loc, size=0.05)
    mesh_obj(name + "_tyre", [(-r * 0.5, r * 0.5, -w / 2 - 0.008, w / 2 + 0.008, -0.012, 0.012)], MAT["tyre"], g, cyls=[("y", -w / 2, w / 2, r, 0, 0)])
    return g


FORKS = []
for i in range(P["nFork"]):
    n, Bd = i + 1, FA["body"]
    hw_ = Bd["w"] / 2
    body = empty("F%d_body" % n, size=0.4)
    mesh_obj("F%d_chassis" % n, [(Bd["xr"], FA["chassisXf"], -hw_, hw_, FA["groundClear"], FA["chassisH"])], MAT["yellow"], body)
    mesh_obj("F%d_hood" % n, [(Bd["xr"], -0.55, -hw_ + 0.03, hw_ - 0.03, FA["chassisH"], FA["hoodH"])], MAT["dark"], body)
    wl = wheel("F%d_wheel_L" % n, body, FA["rLoad"], FA["wLoad"], (0, FA["track"] / 2, FA["rLoad"]))
    wr_ = wheel("F%d_wheel_R" % n, body, FA["rLoad"], FA["wLoad"], (0, -FA["track"] / 2, FA["rLoad"]))
    steer = empty("F%d_steer" % n, body, (-FA["L"], 0, FA["rDrive"]))
    wheel("F%d_drive_wheel" % n, steer, FA["rDrive"], FA["wDrive"], (0, 0, 0))
    px, pz = MS["pivot"]
    mast = empty("F%d_mast" % n, body, (px, 0, pz))
    mesh_obj("F%d_mast_channels" % n, [(MS["x0"] - px, MS["x1"] - px, e * MS["w"] / 2 - e * 0.09, e * MS["w"] / 2, -0.25, MS["height"] - pz) for e in (1, -1)] + [(MS["x0"] - px, MS["x1"] - px, -MS["w"] / 2, MS["w"] / 2, MS["height"] - pz - 0.1, MS["height"] - pz)], MAT["dark"], mast)
    carr = empty("F%d_carriage" % n, mast)
    shift = empty("F%d_shift" % n, carr)
    mesh_obj("F%d_backrest" % n, [(0.09, 0.15, -FF["carriageW"] / 2, FF["carriageW"] / 2, -0.05, 0.4)] + [(0.1, 0.13, -FF["carriageW"] / 2 + 0.02 + k * 0.28, -FF["carriageW"] / 2 + 0.06 + k * 0.28, 0.4, FF["backrestH"]) for k in range(4)], MAT["dark"], shift)
    mesh_obj("F%d_tines" % n, [(MS["aFace"], MS["aFace"] + FF["length"], e * FF["tineY"] - FF["tineW"] / 2, e * FF["tineY"] + FF["tineW"] / 2, -FF["thick"], 0) for e in (1, -1)], MAT["steel"], shift)
    empty("F%d_TCP" % n, shift, (MS["aFace"] + FF["loadCentre"], 0, 0), size=0.08)
    FORKS.append((body, steer, mast, carr, shift))

# ---------- robot AMRs (MOBMAN_SPEC §3) ----------
MA, AR, MT, DK, DH = P["MAMR"], P["ARM"], P["MTOOL"], P["DECK"], P["DH"]
MMS = []
for i in range(P["nMm"]):
    n, Bd = i + 1, MA["body"]
    hw_ = Bd["w"] / 2
    body = empty("M%d_body" % n, size=0.4)
    mesh_obj("M%d_chassis" % n, [(Bd["xr"], Bd["xf"], -hw_, hw_, MA["groundClear"], MA["chassisH"])], MAT["blue"], body)
    wl = wheel("M%d_wheel_L" % n, body, MA["rWheel"], MA["wWheel"], (0, MA["track"] / 2, MA["rWheel"]))
    wr_ = wheel("M%d_wheel_R" % n, body, MA["rWheel"], MA["wWheel"], (0, -MA["track"] / 2, MA["rWheel"]))
    y0, y1 = DK["yc"] - DK["w"] / 2, DK["yc"] + DK["w"] / 2
    mesh_obj("M%d_deck" % n, [(DK["x0"], DK["x1"], y, y + 0.04, MA["chassisH"], DK["z"] + DK["guideH"]) for y in (y0 - 0.04, y1)] + [(DK["x0"], DK["x1"], y0, y1, MA["chassisH"], DK["z"] - 2 * DK["rollerR"] - 0.01), (DK["backX"] - 0.03, DK["backX"], y0, y1, DK["z"], DK["z"] + DK["stopH"])], MAT["conv"], body)
    mesh_obj("M%d_deck_rollers" % n, [], MAT["roller"], body, cyls=[("y", y0, y1, DK["rollerR"], x, DK["z"] - DK["rollerR"]) for x in frange(DK["x0"] + 0.04, DK["x1"], DK["rollerPitch"])], seg=10)
    stop = empty("M%d_front_stop" % n, body)
    mesh_obj("M%d_stop_bar" % n, [(DK["stopX"], DK["stopX"] + 0.03, y0 + 0.05, y1 - 0.05, DK["z"] - DK["stopH"], DK["z"])], MAT["orange"], stop)
    bx, by, bz = AR["base"]
    mesh_obj("M%d_pedestal" % n, [(bx - 0.17, bx + 0.17, by - 0.14, by + 0.14, MA["chassisH"], MA["chassisH"] + 0.3)], MAT["dark"], body, cyls=[("z", MA["chassisH"], bz, AR["pedestalR"], bx, by)])
    parent = empty("M%d_arm_base" % n, body, (bx, by, bz), (0, 0, AR["yaw"]))
    J, Fm = [], []
    for k, row in enumerate(DH):
        j = empty("M%d_J%d" % (n, k + 1), parent)
        f = empty("M%d_F%d" % (n, k + 1), j, (row["a"], 0, row["d"]), (row["alpha"], 0, 0), size=0.06)
        J.append(j)
        Fm.append(f)
        parent = f
    d1, a2, a3, d4, d5, d6, eU, eF = (AR[k] for k in ("d1", "a2", "a3", "d4", "d5", "d6", "offUpper", "offFore"))
    rU, rF, rW = AR["rLink"][:3]
    mesh_obj("M%d_arm_base_housing" % n, [], MAT["arm"], J[0], cyls=[("z", 0, d1 - rU, 0.105, 0, 0)])
    mesh_obj("M%d_shoulder" % n, [], MAT["joint"], Fm[0], cyls=[("z", -rU, eU + rU * 0.9, rU, 0, 0)])
    mesh_obj("M%d_upper_arm" % n, [], MAT["arm"], Fm[1], cyls=[("x", 0, a2, rU * 0.82, 0, eU)])
    mesh_obj("M%d_elbow" % n, [], MAT["joint"], Fm[1], cyls=[("z", eF - rF, eU + rU * 0.9, rF * 1.08, 0, 0)])
    mesh_obj("M%d_forearm" % n, [], MAT["arm"], Fm[2], cyls=[("x", 0, a3, rF * 0.85, 0, eF)])
    mesh_obj("M%d_wrist1" % n, [], MAT["joint"], Fm[2], cyls=[("z", eF - rF, d4 + rW, rW * 1.05, 0, 0)])
    mesh_obj("M%d_wrist2" % n, [], MAT["arm"], Fm[3], cyls=[("z", -rW, d5 + rW, rW, 0, 0)])
    mesh_obj("M%d_wrist3" % n, [], MAT["joint"], Fm[4], cyls=[("z", -rW, d6, rW, 0, 0)])
    mesh_obj("M%d_gripper" % n, [(-MT["padX"] / 2, MT["padX"] / 2, -MT["padY"] / 2, MT["padY"] / 2, MT["tcp"] - 0.05, MT["tcp"] - 0.025)], MAT["steel"], Fm[5],
             cyls=[("z", 0, MT["tcp"] - 0.05, 0.028, 0, 0)] + [("z", MT["tcp"] - 0.025, MT["tcp"], MT["cupDia"] / 2, (c - 1) * 0.085, e * 0.045) for c in range(3) for e in (-1, 1)])
    empty("M%d_TCP" % n, Fm[5], (0, 0, MT["tcp"]), size=0.08)
    MMS.append((body, wl, wr_, J, stop))

# ---------- unit loads ----------
pal_boxes = []
for y, w in ((-0.3275, 0.145), (-0.1638, 0.1), (0, 0.145), (0.1638, 0.1), (0.3275, 0.145)):
    pal_boxes.append((-PL["sx"] / 2, PL["sx"] / 2, y - w / 2, y + w / 2, PL["h"] - 0.022, PL["h"]))
for x in (-0.5275, 0, 0.5275):
    pal_boxes.append((x - 0.0725, x + 0.0725, -PL["sy"] / 2, PL["sy"] / 2, 0.1, 0.122))
    for yb, w in ((-0.35, 0.1), (0, 0.145), (0.35, 0.1)):
        pal_boxes.append((x - 0.0725, x + 0.0725, yb - w / 2, yb + w / 2, 0.022, 0.1))
for yb, w in ((-0.35, 0.1), (0, 0.145), (0.35, 0.1)):
    pal_boxes.append((-PL["sx"] / 2, PL["sx"] / 2, yb - w / 2, yb + w / 2, 0, 0.022))
movers = []
for p in P["pallets"]:
    ob = mesh_obj("Pallet_%s" % p["id"], pal_boxes, MAT["wood"])
    movers.append((ob, p))
    if "wrapF" in p:  # stretch film appears when the wrap is cut
        fm = mesh_obj("Film_%s" % p["id"], [(-0.606, 0.606, -0.406, 0.406, PL["h"] - 0.08, p["h"] + 0.04)], MAT["film"], ob)
        if p["wrapF"] > p["first"]:
            fm.scale = (0, 0, 0)
            fm.keyframe_insert("scale", frame=p["wrapF"])
            fm.scale = (1, 1, 1)
            fm.keyframe_insert("scale", frame=p["wrapF"] + 1)
for b in P["boxes"]:
    sx, sy, sz = b["size"]
    movers.append((mesh_obj("Box_%05d" % b["id"], [(-sx / 2 + 0.002, sx / 2 - 0.002, -sy / 2 + 0.002, sy / 2 - 0.002, -sz / 2 + 0.0015, sz / 2 - 0.0015)], MAT[b["sku"]]), b))
for k, rp in enumerate(P["rack"]):  # pallets in their rack locations: pallet slab + one block per load
    ob = mesh_obj("Rack_pallet_%03d" % k, [(-PL["sy"] / 2, PL["sy"] / 2, -PL["sx"] / 2, PL["sx"] / 2, 0, PL["h"])], MAT["wood"])
    mesh_obj("Rack_load_%03d" % k, [(-0.4, 0.4, -0.6, 0.6, PL["h"], PL["h"] + rp["h"])], MAT[rp["sku"]], ob)
    movers.append((ob, rp))

# ---------- animation from the simulation ----------
scene = bpy.context.scene
F = P["frames"]
NF = len(F["cranes"])
scene.render.fps = P["fps"]
scene.frame_start = 1
scene.frame_end = NF
last = {}


def key(ob, path, value, frame):
    """Set and keyframe a property, skipping frames where nothing changed."""
    k = (ob.name, path)
    if last.get(k) == value and frame < NF:
        return
    if k in last and last[k] != value and last.get((k, "f")) != frame - 1:
        setattr(ob, path, last[k])  # hold the old value up to the frame before the change
        ob.keyframe_insert(path, frame=frame - 1)
    setattr(ob, path, value)
    ob.keyframe_insert(path, frame=frame)
    last[k] = value
    last[(k, "f")] = frame


AX = ((0, 0, 1), (0, 1, 0), (0, 1, 0), (1, 0, 0), (0, 1, 0), (1, 0, 0))  # palletizing robot joint axes J1..J6
for i in range(NF):
    fr_ = i + 1
    c = F["cranes"][i]
    for k, (trv, hst, mid, top) in enumerate(CRN):
        q1, q2, q3 = c[3 * k:3 * k + 3]
        key(trv, "location", (q1, 0, 0), fr_)
        key(hst, "location", (0, 0, q2), fr_)
        key(mid, "location", (0, q3 / 2, 0), fr_)
        key(top, "location", (0, q3, 0), fr_)
    q = F["robots"][i]
    for k, J in enumerate(RJ):
        for j, ob in enumerate(J):
            a = q[6 * k + j]
            key(ob, "rotation_euler", tuple(a * u for u in AX[j]), fr_)
    m = F["mm"][i]
    for k, (body, wl, wr_, J, stop) in enumerate(MMS):
        x, y, th, wL, wR = m[12 * k:12 * k + 5]
        key(body, "location", (x, y, 0), fr_)
        key(body, "rotation_euler", (0, 0, th), fr_)
        key(wl, "rotation_euler", (0, wL, 0), fr_)
        key(wr_, "rotation_euler", (0, wR, 0), fr_)
        for j, ob in enumerate(J):
            key(ob, "rotation_euler", (0, 0, m[12 * k + 5 + j]), fr_)
        key(stop, "location", (0, 0, DK["stopH"] * (m[12 * k + 11] - 1) + DK["stopH"]), fr_)
    v = F["forks"][i]
    for k, (body, steer, mast, carr, shift) in enumerate(FORKS):
        x, y, th, phi, tilt, lift, sh = v[7 * k:7 * k + 7]
        key(body, "location", (x, y, 0), fr_)
        key(body, "rotation_euler", (0, 0, th), fr_)
        key(steer, "rotation_euler", (0, 0, phi), fr_)
        key(mast, "rotation_euler", (0, -tilt, 0), fr_)
        key(carr, "location", (0, 0, lift - MS["pivot"][1]), fr_)
        key(shift, "location", (0, sh, 0), fr_)
    th, lift, up, jaw, sl = F["wrap"][i]
    key(table, "rotation_euler", (0, 0, th), fr_)
    key(car, "location", (0, 0, lift), fr_)
    key(clamp, "location", (CP["x"], CP["y"], CP["stroke"] * up), fr_)
    key(slide, "location", (0, sl, 0), fr_)
    for k, st in enumerate(STOPS):
        key(st, "location", (0, 0, 0.09 * (F["stops"][i][k] - 1)), fr_)

HIDDEN = (0.0, 0.0, 0.0)
for ob, m in movers:
    first, end = m["first"] + 1, m["last"] + 1
    if first > 1:  # not there yet
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
    if end < NF:  # gone (shipped, stored in the rack, or changed)
        ob.keyframe_insert("scale", frame=end)
        ob.scale = HIDDEN
        ob.keyframe_insert("scale", frame=end + 1)
scene.frame_set(1)

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []

# ---------- presentation setup: sun, sky, an orbiting camera, EEVEE, H.264 ----------
def presentation(out_path, frames=None, res=(1280, 720)):
    sc = bpy.context.scene
    for o in list(col.objects):
        if o.name.startswith(("REN_", )):
            bpy.data.objects.remove(o, do_unlink=True)
    sun_d = bpy.data.lights.new("REN_sun", "SUN"); sun_d.energy = 3.5; sun_d.angle = math.radians(2)
    sun = bpy.data.objects.new("REN_sun", sun_d); col.objects.link(sun); sun.rotation_euler = (math.radians(52), 0, math.radians(35))
    w = sc.world or bpy.data.worlds.new("World"); sc.world = w; w.use_nodes = True
    bg = w.node_tree.nodes.get("Background"); bg.inputs[0].default_value = (0.74, 0.77, 0.76, 1); bg.inputs[1].default_value = 0.9
    cam_d = bpy.data.cameras.new("REN_cam"); cam_d.lens = 32
    cam = bpy.data.objects.new("REN_cam", cam_d); col.objects.link(cam); sc.camera = cam
    tgt = empty("REN_target", loc=(31.0, 23.0, 1.0))
    con = cam.constraints.new("TRACK_TO"); con.target = tgt; con.track_axis = "TRACK_NEGATIVE_Z"; con.up_axis = "UP_Y"
    f0, f1 = frames or (sc.frame_start, sc.frame_end)
    for f, ang in ((f0, -2.05), (f1, -1.05)):  # slow orbit from south-west to south-east
        cam.location = (31.0 + 40 * math.cos(ang), 23.0 + 40 * math.sin(ang), 24.0)
        cam.keyframe_insert("location", frame=f)
    sc.frame_start, sc.frame_end = f0, f1
    try:
        sc.render.engine = "BLENDER_EEVEE_NEXT"
    except Exception:
        sc.render.engine = "BLENDER_EEVEE"
    sc.render.resolution_x, sc.render.resolution_y, sc.render.resolution_percentage = res[0], res[1], 100
    sc.view_settings.view_transform = "AgX"
    # frames as JPEG (encode with ffmpeg afterwards: the video output API differs between Blender versions)
    sc.render.image_settings.file_format = "JPEG"
    sc.render.image_settings.quality = 92
    sc.render.filepath = os.path.join(out_path, "frame_")


if "--render" in argv:
    rng = argv[argv.index("--frames") + 1].split("-") if "--frames" in argv else None
    presentation(os.path.abspath(argv[argv.index("--render") + 1]), (int(rng[0]), int(rng[1])) if rng else None)
    bpy.ops.render.render(animation=True)
if "--save" in argv:
    bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(argv[argv.index("--save") + 1]))
if "--glb" in argv:
    bpy.ops.export_scene.gltf(filepath=os.path.abspath(argv[argv.index("--glb") + 1]), export_format="GLB", export_yup=True, export_animations=True)

result = {"objects": len(col.objects), "frames": NF, "boxes": len(P["boxes"]), "pallets": len(P["pallets"]), "rack": len(P["rack"])}
print(result)
