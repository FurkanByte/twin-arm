"""For the FreeCAD assembly, writes every part (in the upright "candle" pose, in world coordinates) as BREP,
plus a manifest (part name, link, color, material, printed or not).

    cad run freecad/export_parts.py      ->  freecad/parts/*.brep, freecad/parts/manifest.json

Then FreeCAD:  freecadcmd freecad/build_freecad.py
"""
import json
import sys
from pathlib import Path

import build123d as bd  # noqa: F401

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from cad.build123d import leaves  # noqa: E402

import parts as P  # noqa: E402
import print_parts as PP  # noqa: E402
from arm import POSES, twinarm  # noqa: E402
from control_box import control_box  # noqa: E402
from harness import SLEEVE_COLOR  # noqa: E402
from lib import dims  # noqa: E402

OUT = ROOT / "freecad" / "parts"
ORANGE = {"upper_arm", "forearm", "idler_plate_shoulder", "idler_plate_elbow", "control_box_lid"}


def hexcolor(c):
    try:
        r, g, b = tuple(c)[:3]
    except TypeError:
        r, g, b = c.to_tuple()[:3]
    return "#%02x%02x%02x" % (round(r * 255), round(g * 255), round(b * 255))


def color_of(leaf, printed):
    lab = leaf.label
    if printed:
        return P.ACCENT if lab in ORANGE else P.DARK
    if lab.startswith("harness"):
        return SLEEVE_COLOR
    return hexcolor(leaf.color) if leaf.color is not None else "#9a9a9a"


def dump(tree, prefix, items):
    for i, (path, leaf) in enumerate(leaves(tree, paths=True)):
        mat = str(leaf.material)
        printed = "petg" in mat.lower()
        link = next((p for p in path if p.endswith("_link")), path[0])
        sub = path[-2] if len(path) >= 2 and not path[-2].endswith("_link") and path[-2] != path[0] else ""
        fname = f"{prefix}{len(items):03d}_{leaf.label}.brep"
        bd.export_brep(leaf, str(OUT / fname))
        items.append({"file": fname, "label": leaf.label, "group": sub, "link": link,
                      "color": color_of(leaf, printed), "material": mat, "printed": printed,
                      "volume_mm3": round(leaf.volume, 1)})


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    for f in OUT.glob("*.brep"):
        f.unlink()
    arm_items, box_items = [], []
    dump(twinarm().shape, "a", arm_items)
    dump(control_box().shape, "b", box_items)
    # individual parts in print orientation (for the drawing sheets)
    printed = []
    for m in PP.PRINTED:
        shape = m().shape
        fname = f"p_{m.__name__}.brep"
        bd.export_brep(shape, str(OUT / fname))
        printed.append({"file": fname, "name": m.__name__, "label": getattr(shape, "label", m.__name__)})
    joints = {
        "base_yaw": {"axis": [0, 0, 1], "center": [0, 0, 0]},
        "shoulder": {"axis": [0, 1, 0], "center": [0, 0, dims.Z_SHOULDER]},
        "elbow": {"axis": [0, 1, 0], "center": [0, 0, dims.Z_ELBOW]},
        "wrist_pitch": {"axis": [0, 1, 0], "center": [0, 0, dims.Z_WRIST]},
        "wrist_roll": {"axis": [0, 0, 1], "center": [0, 0, dims.Z_ROLL_HORN]},
        "gripper_pinion": {"axis": [0, 0, -1], "center": [0, 0, P.G_HORN_TOP]},
    }
    manifest = {"arm": arm_items, "box": box_items, "printed": printed, "joints": joints,
                "mm_per_deg": dims.MM_PER_DEG, "poses": POSES,
                "colors": {"dark": P.DARK, "orange": P.ACCENT}}
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=1, ensure_ascii=False), encoding="utf-8")
    print(f"arm {len(arm_items)} parts, box {len(box_items)} parts, printed {len(printed)} parts -> {OUT}")
