"""TwinArm-5 FreeCAD model and engineering drawings.

Runs with the FreeCAD GUI (colors and PDF export need the GUI) and closes when done:

    cad run freecad/export_parts.py                          # parts -> freecad/parts/
    "<FreeCAD>/bin/freecad.exe" freecad/build_freecad.py     # -> freecad/TwinArm5.FCStd, freecad/drawings/*.pdf
    python freecad/merge_pdfs.py                             # -> freecad/TwinArm5_Drawings.pdf

Model tree:
  TwinArm5 (App::Part)
    base_link > shoulder_link > upper_arm_link > forearm_link > wrist_link > gripper_link
                                                                 > pinion_link, finger_left_link, finger_right_link
  Each link is an App::Part; its placement is computed by an expression from the angle in the "Joints"
  spreadsheet. Change an angle in the spreadsheet and recompute (Ctrl+R) to move the arm inside FreeCAD.
  Control_Box (App::Part), Joints, Parts_List (spreadsheets), Drawing_Sources (hidden).
"""
import csv
import json
import math
import os
import re
import time
import traceback

import FreeCAD as App
import FreeCADGui as Gui
import Part

HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else \
    r"C:/Users/furka/orca/workspaces/digital-twin/lookdown/cad/freecad"
HERE = HERE.replace("\\", "/")
PARTS = HERE + "/parts"
DRAW = HERE + "/drawings"
LOGF = open(HERE + "/build_freecad.log", "w", encoding="utf-8")
TODAY = time.strftime("%Y-%m-%d")


def log(*a):
    LOGF.write(" ".join(str(x) for x in a) + "\n")
    LOGF.flush()


def rgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) / 255 for i in (0, 2, 4))


def pump(sec=0.05):
    Gui.updateGui()
    time.sleep(sec)


# ------------------------------------------------------------------------------------------
# display colors (by part type)
# ------------------------------------------------------------------------------------------
def color_for(item):
    lab, grp = item["label"], item["group"]
    if item["printed"]:
        return item["color"]
    if grp.startswith("servo_"):
        return "#17181b"
    rules = [("harness", "#1e2733"), ("cable_", "#1e2733"), ("insert_", "#c9a227"), ("bearing", "#b9bec5"),
             ("horn_screw", "#2e3136"), ("screw_", "#2e3136"), ("selftap", "#2e3136"), ("setscrew", "#2e3136"),
             ("nut_", "#9aa0a6"), ("horn", "#c4c8ce"), ("ds18b20", "#121212"), ("pad", "#1a1a1a"),
             ("rubber", "#1a1a1a"), ("esp32", "#20262e"), ("ads1115", "#1d5c8a"), ("ina226", "#6a2a7a"),
             ("xl4016", "#1d5c8a"), ("cap_", "#22337a"), ("relay", "#1a1a1a"), ("gx", "#b0b4b9"),
             ("xt60", "#d4b200"), ("fan", "#1c1c1c"), ("estop", "#c62828"), ("start", "#2e7d32"),
             ("wago", "#e8e8e8"), ("fuse", "#e0e0e0"), ("board", "#2f6b3a"), ("power_board", "#2f6b3a"),
             ("signal_board", "#2f6b3a")]
    for k, c in rules:
        if lab.startswith(k):
            return c
    return "#8a8f96"


def nice(name):
    return re.sub(r"[^A-Za-z0-9_]", "_", name)


def set_color(obj, hexc, transparency=0):
    try:
        mat = App.Material(DiffuseColor=rgb(hexc))
        mat.Transparency = transparency / 100.0
        obj.ViewObject.ShapeAppearance = [mat]
    except Exception:
        obj.ViewObject.ShapeColor = rgb(hexc)


# ------------------------------------------------------------------------------------------
# printed-part info (drawing notes)
# ------------------------------------------------------------------------------------------
PRINT_INFO = {
    "base_plate": ("Base bottom plate", "p01", "bottom face on bed", "none"),
    "base_housing": ("Base housing", "p02", "top face on bed (upside down)", "minimal"),
    "turntable": ("Turntable (with hub)", "p03", "turntable top face on bed", "none"),
    "shoulder_tower": ("Shoulder tower", "p04", "flange on bed", "minimal"),
    "upper_arm": ("Upper arm", "p05", "horn cheek on bed", "tree support"),
    "forearm": ("Forearm", "p06", "horn cheek on bed", "tree support"),
    "wrist": ("Wrist housing", "p07", "lug seating face on bed", "moderate"),
    "idler_plate_shoulder": ("Idler / cable cover (shoulder)", "p08", "outer face on bed", "none"),
    "idler_plate_elbow": ("Idler / cable cover (elbow)", "p09", "outer face on bed", "none"),
    "idler_plate_wrist": ("Idler / cable cover (wrist)", "p10", "outer face on bed", "none"),
    "gripper_palm": ("Gripper body", "p11", "cover face on bed (upside down)", "moderate"),
    "gripper_cover": ("Gripper cover", "p12", "top face on bed", "none"),
    "pinion_m15z16": ("Pinion m1.5 z16 (x=+0.1)", "p13", "bottom face on bed", "none"),
    "rack_right": ("Rack (right)", "p14", "bottom face on bed", "none"),
    "rack_left": ("Rack (left)", "p15", "bottom face on bed", "none"),
    "finger_right": ("Finger (right)", "p16", "inner (pad) face on bed", "none"),
    "finger_left": ("Finger (left)", "p17", "inner (pad) face on bed", "none"),
    "hub_collar": ("Hub retaining collar", "p18", "bottom face on bed", "none"),
    "stub_retainer": ("Shaft-end washer", "p19", "flat face on bed", "none"),
    "control_box": ("Control box body", "p20", "bottom face on bed", "none"),
    "control_box_lid": ("Control box lid", "p21", "top face on bed", "none"),
}
COLOR_NAME = {"#2b2f36": "dark anthracite", "#ff6a13": "orange"}


# ------------------------------------------------------------------------------------------
# model
# ------------------------------------------------------------------------------------------
def build_model(doc, M):
    # joint spreadsheet
    ss = doc.addObject("Spreadsheet::Sheet", "Joints")
    ss.Label = "Joints"
    pose = M["poses"]["ready"]
    rows = [("Joint", "Value", "Unit", "Range (software)", "Description")]
    rows += [("base_yaw", pose["base_yaw"], "°", "-90 … 90", "J1 base yaw (Z)"),
             ("shoulder", pose["shoulder"], "°", "-70 … 105", "J2 shoulder (Y), servo center +20°"),
             ("elbow", pose["elbow"], "°", "-60 … 105", "J3 elbow (Y), servo center +25°"),
             ("wrist_pitch", pose["wrist_pitch"], "°", "-60 … 95", "J4 wrist pitch (Y), servo center +20°"),
             ("wrist_roll", pose["wrist_roll"], "°", "-90 … 90", "J5 wrist roll (Z)"),
             ("gripper", pose["gripper"], "°", "0 … 105", "J6 pinion angle (0 = closed, 2 mm gap)")]
    for r, row in enumerate(rows, start=1):
        for c, val in zip("ABCDE", row):
            ss.set(f"{c}{r}", str(val) if not isinstance(val, str) else ("'" + val if c != "B" else val))
        if r > 1:
            ss.setAlias(f"B{r}", row[0])
    ss.set("A8", "'finger_mm")
    ss.set("B8", f"=gripper * {M['mm_per_deg']:.6f}")
    ss.setAlias("B8", "finger_mm")
    ss.set("C8", "'mm")
    ss.set("E8", "'opening per finger (pad gap = 2 + 2 × finger_mm)")
    ss.setStyle("A1:E1", "bold")
    doc.recompute()

    root = doc.addObject("App::Part", "TwinArm5")
    root.Label = "TwinArm5 (arm)"
    links = {}
    chain = [("base_link", None), ("shoulder_link", "base_link"), ("upper_arm_link", "shoulder_link"),
             ("forearm_link", "upper_arm_link"), ("wrist_link", "forearm_link"), ("gripper_link", "wrist_link"),
             ("pinion_link", "gripper_link"), ("finger_left_link", "gripper_link"),
             ("finger_right_link", "gripper_link")]
    for name, parent in chain:
        p = doc.addObject("App::Part", name)
        (links[parent] if parent else root).addObject(p)
        links[name] = p
    J = M["joints"]

    def revolute(link, joint, alias):
        ax, c = J[joint]["axis"], J[joint]["center"]
        lk = links[link]
        lk.Placement = App.Placement(App.Vector(), App.Rotation(App.Vector(*ax), 0))
        lk.setExpression(".Placement.Rotation.Angle", f"Joints.{alias}")
        if ax[1] != 0:                                    # Y axis, center (0, 0, zc)
            zc = c[2]
            lk.setExpression(".Placement.Base.x", f"-{zc} * sin(Joints.{alias})")
            lk.setExpression(".Placement.Base.z", f"{zc} * (1 - cos(Joints.{alias}))")

    revolute("shoulder_link", "base_yaw", "base_yaw")
    revolute("upper_arm_link", "shoulder", "shoulder")
    revolute("forearm_link", "elbow", "elbow")
    revolute("wrist_link", "wrist_pitch", "wrist_pitch")
    revolute("gripper_link", "wrist_roll", "wrist_roll")
    revolute("pinion_link", "gripper_pinion", "gripper")
    links["finger_left_link"].setExpression(".Placement.Base.x", "-Joints.finger_mm")
    links["finger_right_link"].setExpression(".Placement.Base.x", "Joints.finger_mm")

    used = {}

    def add_part(item, container):
        shp = Part.read(PARTS + "/" + item["file"])
        label = item["group"] if item["group"] else item["label"]
        n = used.get(label, 0)
        used[label] = n + 1
        o = doc.addObject("Part::Feature", nice(label) + (f"_{n:02d}" if n else ""))
        o.Shape = shp
        o.Label = label if n == 0 else f"{label} ({n + 1})"
        o.addProperty("App::PropertyString", "PartMaterial", "TwinArm5").PartMaterial = item["material"]
        o.addProperty("App::PropertyBool", "Printed", "TwinArm5").Printed = bool(item["printed"])
        o.addProperty("App::PropertyString", "Source", "TwinArm5").Source = "cad/src (build123d)"
        container.addObject(o)
        set_color(o, color_for(item))
        return o

    for it in M["arm"]:
        add_part(it, links[it["link"]])
    box = doc.addObject("App::Part", "Control_Box")
    box.Label = "Control box"
    box.Placement = App.Placement(App.Vector(-300, 105, 0), App.Rotation(App.Vector(0, 0, 1), -90))
    for it in M["box"]:
        add_part(it, box)
    for o in doc.Objects:                                 # internal harnesses are not translucent, they stay visible
        if o.TypeId == "Part::Feature" and o.Label.startswith("control_box_lid"):
            set_color(o, P_COLOR_ORANGE, 60)               # lid translucent so the inside shows
    doc.recompute()

    # drawing sources: hidden copies of the printed parts in assembly coordinates
    grp = doc.addObject("App::DocumentObjectGroup", "Drawing_Sources")
    srcs = {}
    for it in M["arm"] + M["box"]:
        if it["printed"] and it["label"] not in srcs:
            o = doc.addObject("Part::Feature", "src_" + nice(it["label"]))
            o.Shape = Part.read(PARTS + "/" + it["file"])
            o.Label = "source: " + it["label"]
            grp.addObject(o)
            o.Visibility = False
            srcs[it["label"]] = (o, it)
    doc.recompute()
    return root, links, box, srcs


P_COLOR_ORANGE = "#ff6a13"


# ------------------------------------------------------------------------------------------
# parts list spreadsheet
# ------------------------------------------------------------------------------------------
def build_parts_list(doc, M):
    ss = doc.addObject("Spreadsheet::Sheet", "Parts_List")
    ss.Label = "Parts list"
    rows = [("No", "Part / item", "Qty", "Material / source", "Drawing / note")]
    counts = {}
    for it in M["arm"] + M["box"]:
        if it["printed"]:
            counts[it["label"]] = counts.get(it["label"], 0) + 1
    order = sorted(counts, key=lambda k: PRINT_INFO.get(k, ("", "p99"))[1])
    for k in order:
        name, pno, _, _ = PRINT_INFO.get(k, (k, "p99", "", ""))
        col = COLOR_NAME.get(next(i["color"] for i in M["arm"] + M["box"] if i["label"] == k), "")
        rows.append((pno.upper(), name, counts[k], f"PETG, {col}", f"TA5-{pno.upper()}"))
    csv_path = os.path.join(os.path.dirname(HERE), "..", "BOM.csv")
    n = 1
    with open(csv_path, encoding="utf-8-sig") as f:
        r = csv.reader(f, delimiter=";")
        next(r)
        for row in r:
            if len(row) < 9 or not row[0] or row[0] == "Filament":
                continue
            rows.append((f"S{n:02d}", row[1][:70], row[3], row[4][:34], row[0]))
            n += 1
    for i, row in enumerate(rows, start=1):
        for c, val in zip("ABCDE", row):
            ss.set(f"{c}{i}", "'" + str(val))
    ss.setStyle("A1:E1", "bold")
    ss.setColumnWidth("A", 45)
    ss.setColumnWidth("B", 520)
    ss.setColumnWidth("C", 80)
    ss.setColumnWidth("D", 270)
    ss.setColumnWidth("E", 110)
    doc.recompute()
    return ss, len(rows)


# ------------------------------------------------------------------------------------------
# TechDraw helpers (A3 landscape, ISO 5457 title block, English labels)
# Page layout (mm): title block bottom right (x 230-410, y 10-72). Views on the left (x 20-228),
# isometric + notes top right (x 232-410, y 76-285).
# ------------------------------------------------------------------------------------------
PAGES = []
TPL = App.getResourceDir() + "Mod/TechDraw/Templates/ISO/A3_Landscape_ISO5457_advanced.svg"
W, H = 420.0, 297.0
TB_TOP = 74.0


def new_page(doc, name, texts):
    page = doc.addObject("TechDraw::DrawPage", name)
    tpl = doc.addObject("TechDraw::DrawSVGTemplate", name + "_template")
    tpl.Template = TPL
    page.Template = tpl
    et = dict(tpl.EditableTexts)
    base = {"creator": "", "approval_person": "", "legal_owner_1": "TwinArm-5 graduation project",
            "legal_owner_2": "", "legal_owner_3": "", "legal_owner_4": "", "date_of_issue": TODAY,
            "document_status": "Draft", "language_code": "EN", "general_tolerances": "ISO 2768-c",
            "revision_index": "A", "responsible_department": "", "supplementary_title_1": "",
            "supplementary_title_2": "", "part_material": ""}
    base.update(texts)
    for k, v in base.items():
        if k in et:
            et[k] = v
    tpl.EditableTexts = et
    PAGES.append(page)
    return page


def wait(views, timeout=600):
    t0 = time.time()
    while time.time() - t0 < timeout:
        pump(0.2)
        if all(len(v.getVisibleEdges()) > 0 for v in views):
            pump(0.3)
            return True
    log("  timeout:", [v.Name for v in views])
    return False


def full_circle(e):
    c = e.Curve
    return hasattr(c, "Radius") and abs(e.Length - 2 * math.pi * c.Radius) < 0.02 * e.Length


def extents(v):
    """Unscaled extents of the view and the edges that define them."""
    es = v.getVisibleEdges()
    xmin = ymin = 1e9
    xmax = ymax = -1e9
    for e in es:
        b = e.BoundBox
        xmin, xmax = min(xmin, b.XMin), max(xmax, b.XMax)
        ymin, ymax = min(ymin, b.YMin), max(ymax, b.YMax)
    return xmin, xmax, ymin, ymax


def extreme_vertex(v, es, axis, val, tol=0.3, flip=True, scale=1.0):
    """Vertex at the view's extreme point in one direction; if there is no vertex (arc/circle), places a
    cosmetic vertex at that point. Returns: (Vertex index, cosmetic vertex tag or None)."""
    vs = v.getVisibleVertexes()
    k = 0 if axis == "x" else 1
    hits = [i for i, p in enumerate(vs) if abs((p.x, p.y)[k] - val) < tol]
    if hits:
        return min(hits, key=lambda i: abs((vs[i].y, vs[i].x)[k])), None
    for e in es:
        b = e.BoundBox
        lo, hi = (b.XMin, b.XMax) if axis == "x" else (b.YMin, b.YMax)
        if abs(lo - val) < tol or abs(hi - val) < tol:
            c = getattr(e.Curve, "Center", None)
            if c is None:
                continue
            px, py = (val, c.y) if axis == "x" else (c.x, val)
            n0 = len(vs)
            tag = v.makeCosmeticVertex(App.Vector(px / scale, (-py if flip else py) / scale, 0))
            v.recompute()
            t0 = time.time()
            while len(v.getVisibleVertexes()) <= n0 and time.time() - t0 < 20:
                pump(0.1)
            if len(v.getVisibleVertexes()) > n0:
                return len(v.getVisibleVertexes()) - 1, tag
            return None, tag
    return None, None


def dim_value(d):
    for _ in range(20):
        pump(0.05)
    try:
        return float(d.getRawValue())
    except Exception:
        try:
            return float(str(d.Measurement).split()[0])
        except Exception:
            return None


def checked_distance(doc, page, v, scale, axis, lo, hi):
    """DistanceX/Y between the extreme points; if the measured value does not match the view size, retries
    with the other Y convention, and if it still does not match, leaves the dimension out."""
    want = (hi - lo) / scale          # edges are scaled, the dimension is the true value
    es = v.getVisibleEdges()
    for flip in (True, False):
        i0, t0 = extreme_vertex(v, es, axis, lo, flip=flip, scale=scale)
        i1, t1 = extreme_vertex(v, es, axis, hi, flip=flip, scale=scale)
        if i0 is None or i1 is None:
            continue
        d = doc.addObject("TechDraw::DrawViewDimension", "Dim")
        d.Type = "DistanceX" if axis == "x" else "DistanceY"
        d.References2D = [(v, f"Vertex{i0}"), (v, f"Vertex{i1}")]
        page.addView(d)
        xmin, xmax, ymin, ymax = extents(v)
        if axis == "x":
            d.X, d.Y = (xmin + xmax) / 2, ymax + 9
        else:
            d.X, d.Y = xmax + 9, (ymin + ymax) / 2
        doc.recompute()
        got = dim_value(d)
        if got is not None and abs(got - want) < 0.05 * max(want, 1.0):
            return d
        log("  dimension mismatch", v.Name, axis, round(want, 2), got, "flip", flip)
        page.removeView(d) if hasattr(page, "removeView") else None
        doc.removeObject(d.Name)
        for t in (t0, t1):
            if t:
                try:
                    v.removeCosmeticVertex(t)
                except Exception:
                    pass
        v.recompute()
        if t0 is None and t1 is None:
            break
    return None


def add_dims(doc, page, v, scale, horiz=True, vert=True, holes=3):
    """Overall dimensions (from extreme vertices; diameter if the outline is a circle) and hole diameters (with count)."""
    es = v.getVisibleEdges()
    vs = v.getVisibleVertexes()
    if not es:
        return
    xmin, xmax, ymin, ymax = extents(v)
    tol = 0.3
    # is the outline a full circle?
    outer = None
    for i, e in enumerate(es):
        b = e.BoundBox
        if full_circle(e) and abs(b.XMin - xmin) < tol and abs(b.XMax - xmax) < tol:
            outer = i
    if outer is not None:
        d = doc.addObject("TechDraw::DrawViewDimension", "Dim")
        d.Type = "Diameter"
        d.References2D = [(v, f"Edge{outer}")]
        page.addView(d)
        d.X, d.Y = xmax * 0.75, ymax + 9
    else:
        if horiz:
            checked_distance(doc, page, v, scale, "x", xmin, xmax)
        if vert:
            checked_distance(doc, page, v, scale, "y", ymin, ymax)
    if holes:
        circ = {}
        for i, e in enumerate(es):
            if i != outer and full_circle(e) and e.Curve.Radius < 0.45 * max(xmax - xmin, ymax - ymin):
                circ.setdefault(round(e.Curve.Radius, 2), []).append((i, e.Curve.Center))
        k = 0
        for r in sorted(circ, reverse=True)[:holes]:
            lst = circ[r]
            i, cen = max(lst, key=lambda t: t[1].x + t[1].y)
            d = doc.addObject("TechDraw::DrawViewDimension", "Dia")
            d.Type = "Diameter"
            d.References2D = [(v, f"Edge{i}")]
            page.addView(d)
            n = len(lst)
            d.FormatSpec = (f"{n}× " if n > 1 else "") + "⌀%.1w"
            d.X = xmax + 28 + 4 * k
            d.Y = ymax - 2 - 10 * k
            k += 1


def note(doc, page, x, y, lines, size=3.0):
    a = doc.addObject("TechDraw::DrawViewAnnotation", "Note")
    a.Text = lines
    a.TextSize = size
    page.addView(a)
    a.X, a.Y = x, y
    return a


SCALES = [5, 4, 3, 2, 1.5, 1, 1 / 1.5, 1 / 2, 1 / 2.5, 1 / 3, 1 / 4, 1 / 5, 1 / 10]
GAP = 22.0


def scale_text(s):
    if s >= 1:
        return f"{s:g} : 1"
    return f"1 : {1 / s:g}"


def pick_scale(dims):
    x, y, z = dims
    for s in SCALES:
        if (x + y) * s + GAP + 45 <= 208 and (z + y) * s + GAP + 30 <= 262:
            return s
    return SCALES[-1]


def part_page(doc, src, item, idx, total):
    lab = item["label"]
    name, pno, orient, support = PRINT_INFO.get(lab, (lab, "p99", "", ""))
    bb = src.Shape.BoundBox
    dims = (bb.XLength, bb.YLength, bb.ZLength)
    s = pick_scale(dims)
    page = new_page(doc, f"Sheet_{pno.upper()}_{nice(lab)}", {
        "title": f"TwinArm-5 — {name}", "document_type": "Part drawing",
        "drawing_number": f"TA5-{pno.upper()}", "scale": scale_text(s),
        "sheet_number": f"{idx} / {total}",
        "part_material": "PETG (FDM, 0.2 mm layer)",
        "supplementary_title_1": f"Color: {COLOR_NAME.get(item['color'], '')}",
        "supplementary_title_2": f"Print: {orient}; support: {support}"})
    pg = doc.addObject("TechDraw::DrawProjGroup", "Views")
    page.addView(pg)
    pg.Source = [src]
    pg.ProjectionType = "First angle"
    pg.ScaleType = "Custom"
    pg.Scale = s
    pg.AutoDistribute = True
    pg.spacingX = GAP
    pg.spacingY = GAP
    front = pg.addProjection("Front")
    top = pg.addProjection("Top")
    right = pg.addProjection("Right")
    x, y, z = dims
    wg, hg = (x + y) * s + GAP, (z + y) * s + GAP
    left = 18 + max(0.0, (208 - 40 - wg) / 2)
    pg.X = left + y * s + GAP + x * s / 2
    topy = 287 - 14 - max(0.0, (262 - 40 - hg) / 2)
    pg.Y = topy - z * s / 2
    iso = doc.addObject("TechDraw::DrawViewPart", "Isometric")
    page.addView(iso)
    iso.Source = [src]
    iso.Direction = App.Vector(1, -1, 0.8)
    iso.XDirection = App.Vector(1, 1, 0)
    iso.ScaleType = "Custom"
    diag = math.sqrt(x * x + y * y + z * z)
    iso.Scale = min(s, 105.0 / diag)
    iso.X, iso.Y = 322, 205
    iso.Caption = "Isometric (not to scale)"
    for o in (front, top, right, iso):
        o.SmoothVisible = False
        o.SeamVisible = False
    if lab.startswith("idler_plate") or lab in ("stub_retainer", "hub_collar"):   # enclosed inner volume: hidden lines
        for o in (front, top, right):
            o.HardHidden = True
    doc.recompute()
    wait([front, top, right, iso])
    add_dims(doc, page, front, s)
    add_dims(doc, page, top, s, horiz=False)
    add_dims(doc, page, right, s, horiz=False, vert=False, holes=2)
    vol = src.Shape.Volume
    cnt = item.get("count", 1)
    note(doc, page, 322, 108, [
        f"Qty: {cnt}    Overall size: {x:.1f} × {y:.1f} × {z:.1f} mm",
        f"Volume {vol / 1000:.1f} cm³ · filament ≈ {vol * 1.27e-3 * 0.5:.1f} g / pc",
        f"Print orientation: {orient} · support: {support}",
        "PETG, 0.2 mm layer, 0.4 mm nozzle, 4 walls, 25% gyroid",
        "M3 insert holes Ø4.0 (pressed in with a soldering iron)",
        "Check bearing seats and shaft diameter after printing",
        "Undimensioned edges: per CAD model (cad/src/parts.py)",
        "Views: first-angle projection (ISO E)",
    ], size=2.8)
    doc.recompute()
    return page


def view_offset(v, bbox_world, ax_h, ax_v, scale=1.0):
    """View coordinate = world coordinate - offset (front view: horizontal X, vertical Z)."""
    xmin, xmax, ymin, ymax = extents(v)
    lo_h, hi_h = getattr(bbox_world, ax_h + "Min"), getattr(bbox_world, ax_h + "Max")
    lo_v, hi_v = getattr(bbox_world, ax_v + "Min"), getattr(bbox_world, ax_v + "Max")
    return ((lo_h + hi_h) / 2 - (xmin + xmax) / 2 / scale, (lo_v + hi_v) / 2 - (ymin + ymax) / 2 / scale)


def global_bbox(objs):
    bb = App.BoundBox()
    for o in objs:
        if o.TypeId == "Part::Feature" and not o.Shape.isNull():
            sh = o.Shape.copy()
            sh.Placement = o.getGlobalPlacement()
            bb.add(sh.BoundBox)
    return bb


def assembly_pages(doc, root, M, srcs, total, numbers):
    page = new_page(doc, "Sheet_01_General_Assembly", {
        "title": "TwinArm-5 — General assembly (ready pose)", "document_type": "Assembly drawing",
        "drawing_number": "TA5-A01", "scale": "1 : 4", "sheet_number": f"1 / {total}",
        "part_material": "—", "supplementary_title_1": "5 axes + parallel gripper, 6 servos",
        "supplementary_title_2": "Cables routed entirely through internal channels"})
    sc = 0.25
    pg = doc.addObject("TechDraw::DrawProjGroup", "Assembly_Views")
    page.addView(pg)
    pg.Source = [root]
    pg.ProjectionType = "First angle"
    pg.ScaleType = "Custom"
    pg.Scale = sc
    pg.AutoDistribute = True
    pg.spacingX = 30
    pg.spacingY = 25
    front = pg.addProjection("Front")
    top = pg.addProjection("Top")
    right = pg.addProjection("Right")
    pg.X, pg.Y = 150, 195
    iso = doc.addObject("TechDraw::DrawViewPart", "Assembly_Isometric")
    page.addView(iso)
    iso.Source = [root]
    iso.Direction = App.Vector(1, -1, 0.8)
    iso.XDirection = App.Vector(1, 1, 0)
    iso.ScaleType = "Custom"
    iso.Scale = 0.3
    iso.X, iso.Y = 322, 190
    iso.Caption = "Isometric"
    for v in (front, top, right, iso):
        v.HardHidden = False
        v.SmoothVisible = False
        v.SeamVisible = False
    doc.recompute()
    wait([front, top, right, iso], timeout=900)
    add_dims(doc, page, front, sc, holes=0)
    # balloons (front view): part number matches the parts list
    feats = [o for o in root.OutListRecursive if o.TypeId == "Part::Feature"]
    off = view_offset(front, global_bbox(feats), "X", "Z", sc)
    xmin, xmax, ymin, ymax = extents(front)
    want = ["base_plate", "base_housing", "turntable", "shoulder_tower", "upper_arm", "idler_plate_shoulder",
            "forearm", "idler_plate_elbow", "wrist", "gripper_palm", "gripper_cover", "finger_right"]
    items = []
    for lab in want:
        o = next((f for f in feats if f.Label == lab), None)
        if o is None or lab not in numbers:
            continue
        sh = o.Shape.copy()
        sh.Placement = o.getGlobalPlacement()
        c = sh.BoundBox.Center
        items.append((lab, c.x - off[0], c.z - off[1]))
    xmin, xmax = xmin / sc, xmax / sc
    mid = (xmin + xmax) / 2
    step = 10.5 / sc
    for side in (-1, 1):
        col = sorted([t for t in items if (t[1] < mid) == (side < 0)], key=lambda t: -t[2])
        y_prev = None
        for lab, vx, vz in col:
            y = vz if y_prev is None else min(vz, y_prev - step)
            y_prev = y
            b = doc.addObject("TechDraw::DrawViewBalloon", "Balloon")
            b.SourceView = front
            b.OriginX, b.OriginY = vx, vz
            b.X = (xmin if side < 0 else xmax) + side * (14 if side < 0 else 34) / sc
            b.Y = y
            b.Text = numbers[lab]
            b.BubbleShape = "Circular"
            try:
                b.ShapeScale = 0.8
                b.EndType = "Filled dot"
            except Exception:
                pass
            page.addView(b)
    note(doc, page, 322, 100, [
        "Joint axes: J1 Z (base) · J2-J4 Y (shoulder, elbow, wrist pitch)",
        "J5 Z (wrist roll) · J6 rack-and-pinion parallel gripper",
        "Shoulder axis 136 mm above ground · upper arm 110 mm · forearm 110 mm",
        "Mechanical stops: shoulder/elbow ±111°, wrist -110/+101°",
        "Balloon numbers: parts list (TA5-L01) and part drawings",
    ], size=2.8)
    doc.recompute()

    page2 = new_page(doc, "Sheet_02_Cable_Routing_Section", {
        "title": "TwinArm-5 — Internal cable routing, section A-A", "document_type": "Assembly drawing / section",
        "drawing_number": "TA5-A02", "scale": "1 : 2.5", "sheet_number": f"2 / {total}",
        "part_material": "—", "supplementary_title_1": "Section plane: internal channel axis, y = -22.25",
        "supplementary_title_2": "Hollow shafts at the joints (6804, inner Ø13)"})
    base = doc.addObject("TechDraw::DrawViewPart", "Section_Top_View")
    page2.addView(base)
    base.Source = [root]
    base.Direction = App.Vector(0, 0, 1)
    base.ScaleType = "Custom"
    base.Scale = 0.4
    base.X, base.Y = 115, 215
    base.HardHidden = False
    base.Caption = "Top view"
    sec = doc.addObject("TechDraw::DrawViewSection", "Section_AA")
    page2.addView(sec)
    sec.BaseView = base
    sec.Source = [root]
    sec.ScaleType = "Custom"
    sec.Scale = 0.4
    sec.SectionNormal = App.Vector(0, -1, 0)
    sec.Direction = App.Vector(0, -1, 0)
    sec.SectionOrigin = App.Vector(0, -22.25, 200)
    sec.SectionSymbol = "A"
    sec.X, sec.Y = 315, 180
    doc.recompute()
    wait([base, sec], timeout=900)
    note(doc, page2, 118, 95, [
        "1. The servo cable enters the internal channel (13 × 9 mm) in the block's -Y wall via the slot at the pocket bottom.",
        "2. The channel runs inside the link along the beam and bridge; no cable is exposed.",
        "3. Pitch joints: the harness passes through the hollow shaft and crosses on the axis (twist only).",
        "4. Base and wrist roll: enclosed cable chamber, the harness winds ±90° like a clock spring.",
        "5. Only external cable: the main harness from the base's rear window to the control box.",
    ], size=2.8)
    doc.recompute()
    return page, page2


def svg_table(rows, widths, row_h=4.6, font=2.3):
    """Simple vector table (mm). First row is the header."""
    tw = sum(widths)
    th = row_h * len(rows)
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{tw}mm" height="{th}mm" viewBox="0 0 {tw} {th}">',
           f'<rect x="0" y="0" width="{tw}" height="{row_h}" fill="#e8e8e8" stroke="none"/>']
    for r in range(len(rows) + 1):
        sw = 0.35 if r in (0, 1, len(rows)) else 0.13
        out.append(f'<line x1="0" y1="{r * row_h:.2f}" x2="{tw}" y2="{r * row_h:.2f}" stroke="#000" stroke-width="{sw}"/>')
    x = 0
    for wcol in [0] + widths:
        x += wcol
        out.append(f'<line x1="{x:.2f}" y1="0" x2="{x:.2f}" y2="{th:.2f}" stroke="#000" stroke-width="0.35"/>')
    for r, row in enumerate(rows):
        x = 0
        for c, (val, wcol) in enumerate(zip(row, widths)):
            txt = str(val).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
            maxc = int(wcol / (font * 0.52))
            if len(txt) > maxc:
                txt = txt[:maxc - 1] + "…"
            wgt = "bold" if r == 0 else "normal"
            out.append(f'<text x="{x + 1.2:.2f}" y="{r * row_h + row_h * 0.72:.2f}" font-family="osifont" '
                       f'font-size="{font}" font-weight="{wgt}">{txt}</text>')
            x += wcol
    out.append("</svg>")
    return "\n".join(out)


def parts_list_pages(doc, rows, total, first_idx):
    per = 43
    hdr, body = rows[0], rows[1:]
    widths = [14, 178, 24, 92, 52]
    k = 0
    for start in range(0, len(body), per):
        k += 1
        page = new_page(doc, f"Sheet_03_Parts_List_{k}", {
            "title": f"TwinArm-5 — Parts list ({k})", "document_type": "Parts list",
            "drawing_number": f"TA5-L0{k}", "scale": "—", "sheet_number": f"{first_idx + k - 1} / {total}",
            "part_material": "—", "supplementary_title_1": "P: printed part · S: purchased",
            "supplementary_title_2": "Prices and links: BOM.md / BOM.csv"})
        chunk = [hdr] + body[start:start + per]
        sym = doc.addObject("TechDraw::DrawViewSymbol", f"List_{k}")
        sym.Symbol = svg_table(chunk, widths, row_h=4.4)
        page.addView(sym)
        th = 4.4 * len(chunk)
        sym.X, sym.Y = 27 + sum(widths) / 2, 279 - th / 2
    doc.recompute()
    return k


def list_rows(M):
    rows = [("No", "Part / item", "Qty", "Material / source", "Drawing / group")]
    counts = {}
    for it in M["arm"] + M["box"]:
        if it["printed"]:
            counts[it["label"]] = counts.get(it["label"], 0) + 1
    numbers = {}
    for k in sorted(counts, key=lambda k: PRINT_INFO.get(k, ("", "p99"))[1]):
        name, pno, _, _ = PRINT_INFO.get(k, (k, "p99", "", ""))
        col = COLOR_NAME.get(next(i["color"] for i in M["arm"] + M["box"] if i["label"] == k), "")
        rows.append((pno.upper(), name, counts[k], f"PETG, {col}", f"TA5-{pno.upper()}"))
        numbers[k] = pno.upper()
    csv_path = os.path.join(os.path.dirname(HERE), "..", "BOM.csv")
    n = 1
    with open(csv_path, encoding="utf-8-sig") as f:
        r = csv.reader(f, delimiter=";")
        next(r)
        for row in r:
            if len(row) < 9 or not row[0] or row[0] == "Filament":
                continue
            rows.append((f"S{n:02d}", row[1], row[3], row[4], row[0]))
            n += 1
    return rows, numbers, counts


# ------------------------------------------------------------------------------------------
def main():
    import TechDrawGui
    M = json.load(open(PARTS + "/manifest.json", encoding="utf-8"))
    doc = App.newDocument("TwinArm5")
    root, links, box, srcs = build_model(doc, M)
    log("model:", len(doc.Objects), "objects")
    build_parts_list(doc, M)
    rows, numbers, counts = list_rows(M)
    order = sorted(srcs, key=lambda k: PRINT_INFO.get(k, ("", "p99"))[1])
    n_list = (len(rows) - 1 + 42) // 43
    total = 2 + n_list + len(order)
    quick = os.environ.get("TA5_QUICK")
    if quick:
        order = [k for k in order if k in quick.split(",")]
    if not quick or "ASM" in quick.split(","):
        assembly_pages(doc, root, M, srcs, total, numbers)
        log("assembly pages done")
    parts_list_pages(doc, rows, total, 3)
    for i, lab in enumerate(order):
        src, it = srcs[lab]
        it = dict(it, count=counts.get(lab, 1))
        try:
            part_page(doc, src, it, 3 + n_list + i, total)
            log("drawing:", lab)
        except Exception:
            log("ERROR drawing", lab, traceback.format_exc())
    doc.recompute()
    pump(1.0)
    try:
        Gui.activateWorkbench("PartWorkbench")
        Gui.ActiveDocument.ActiveView.viewIsometric()
        Gui.SendMsgToActiveView("ViewFit")
    except Exception:
        pass
    doc.saveAs(HERE + ("/quick.FCStd" if quick else "/TwinArm5.FCStd"))
    log("saved")
    os.makedirs(DRAW, exist_ok=True)
    for f in os.listdir(DRAW):
        if f.endswith(".pdf"):
            os.remove(os.path.join(DRAW, f))
    for k, page in enumerate(PAGES, start=1):
        try:
            Gui.ActiveDocument.getObject(page.Name).show()
            for _ in range(10):
                pump(0.1)
            TechDrawGui.exportPageAsPdf(page, f"{DRAW}/{k:02d}_{page.Name}.pdf")
            log("pdf:", page.Name)
        except Exception:
            log("ERROR pdf", page.Name, traceback.format_exc())
    doc.save()
    log("done")


try:
    main()
except Exception:
    log(traceback.format_exc())
LOGF.close()
os._exit(0)
