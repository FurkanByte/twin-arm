"""Linear static strength analysis of the critical printed parts (netgen + CalculiX).

    python simulation/fea_run.py    # builds simulation/fea_parts.py, then solves -> simulation/results/fea/<part>/

Loads: peak values from the MuJoCo simulation (simulation/dynamics.py) x 1.5 dynamic factor.
Material: PETG FDM, conservative values for the interlayer direction (weakest direction).
Candle-pose coordinates: with the arm horizontal, gravity acts along the part's +X (shoulder +90°).
"""
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "simulation" / "results" / "fea"
CAD = "cad"
PETG = {"name": "PETG FDM (interlayer)", "E_MPa": 1800, "nu": 0.38, "yield_MPa": 25.0,
        "density_t_per_mm3": 1.27e-9}
DYN = 1.5

Z_SH, L = 136.0, 110.0
Z_EL = Z_SH + L
EAR_NEAR, EAR_FAR = 14.75, -34.75          # position of the ear holes from the axis (along the link)
EAR_SPAN = (EAR_NEAR - EAR_FAR) / 1000     # m


def ids():
    """Build the FEA part documents and map each model function to its document id."""
    r = subprocess.run([CAD, "run", str(ROOT / "simulation" / "fea_parts.py")], capture_output=True,
                       text=True, encoding="utf-8", errors="replace", cwd=ROOT)
    out, name = {}, None
    for line in (r.stdout + r.stderr).splitlines():
        m = re.search(r"::(fea_\w+)", line)
        if m:
            name = m.group(1)
        m = re.match(r"\s+id:\s+(\S+)", line)
        if m and name:
            out[name] = m.group(1)
            name = None
    if not out:
        raise SystemExit("fea_parts.py produced no documents:\n" + r.stderr[-2000:])
    return out


def faces(doc):
    r = subprocess.run([CAD, "model", "solve", doc, "--json"], capture_output=True, text=True, cwd=ROOT)
    return json.loads(r.stdout)["faces"]


def pick(fs, pred, want=None):
    sel = [f["ref"] for f in fs if pred(f)]
    if want is not None and len(sel) != want:
        raise SystemExit(f"face selection found {len(sel)}, expected {want}")
    if not sel:
        raise SystemExit("face selection is empty")
    return sel


def near(c, x=None, y=None, z=None, tol=0.6):
    return all(v is None or abs(c[i] - v) <= tol for i, v in enumerate((x, y, z)))


def cyl(f):
    return f["surface"] == "cylinder"


def plane_n(f, axis, sign):
    n = f["normal"]
    return f["surface"] == "plane" and n is not None and abs(n[axis] - sign) < 1e-3


def study_upper_arm(fs):
    """Arm horizontal, 150 g payload: elbow reaction (sim: elbow 1.03 N.m, weight at the tip ~4.9 N)."""
    T = 1.03 * DYN
    W = 4.9 * DYN
    Fc = T / EAR_SPAN
    ins_near = pick(fs, lambda f: cyl(f) and abs(abs(f["center_mm"][0]) - 5) < 0.6 and abs(f["center_mm"][2] - (Z_EL + EAR_NEAR)) < 0.6, 2)
    ins_far = pick(fs, lambda f: cyl(f) and abs(abs(f["center_mm"][0]) - 5) < 0.6 and abs(f["center_mm"][2] - (Z_EL + EAR_FAR)) < 0.6, 2)
    horn_face = pick(fs, lambda f: plane_n(f, 1, -1) and abs(f["center_mm"][1] - 29.9) < 0.3)
    plate_face = pick(fs, lambda f: plane_n(f, 1, -1) and abs(f["center_mm"][1] + 29.9) < 0.3)
    return {
        "material": PETG,
        "fixtures": [{"faces": horn_face + plate_face, "type": "fixed"}],
        "loads": [
            {"faces": ins_near, "type": "force", "vector_N": [Fc + W / 2, 0, 0]},
            {"faces": ins_far, "type": "force", "vector_N": [-Fc + W / 2, 0, 0]},
        ],
        "margin": 2,
    }, f"elbow torque couple {Fc:.0f} N x {EAR_SPAN * 1000:.1f} mm + weight {W:.1f} N (+X)"


def study_shoulder_tower(fs):
    """Shoulder servo reaction 2.0 N.m (demo peak) x1.5 + arm weight ~8.3 N x1.5 (-Z)."""
    T = 2.0 * DYN
    W = 8.3 * DYN
    Fc = T / EAR_SPAN
    ins_near = pick(fs, lambda f: cyl(f) and abs(abs(f["center_mm"][0]) - 5) < 0.6 and abs(f["center_mm"][2] - (Z_SH + EAR_NEAR)) < 0.6, 2)
    ins_far = pick(fs, lambda f: cyl(f) and abs(abs(f["center_mm"][0]) - 5) < 0.6 and abs(f["center_mm"][2] - (Z_SH + EAR_FAR)) < 0.6, 2)
    # hollow shaft (Ø20/Ø13): outer surface where the 6804 inner ring sits
    idler = pick(fs, lambda f: cyl(f) and abs(f["center_mm"][2] - Z_SH) < 0.6 and abs(f["area_mm2"] - 439.8) < 2, 1)
    bottom = pick(fs, lambda f: plane_n(f, 2, -1) and abs(f["center_mm"][2] - 94.0) < 0.3)
    return {
        "material": PETG,
        "fixtures": [{"faces": bottom, "type": "fixed"}],
        "loads": [
            {"faces": ins_near, "type": "force", "vector_N": [Fc, 0, -W / 4]},
            {"faces": ins_far, "type": "force", "vector_N": [-Fc, 0, -W / 4]},
            {"faces": idler, "type": "force", "vector_N": [0, 0, -W / 2]},
        ],
        "margin": 2,
    }, f"shoulder torque couple {Fc:.0f} N + weight {W:.1f} N (-Z)"


def study_finger(fs):
    """MG90S full-torque grip: 9 N per finger x1.5, on the pad face (inner face)."""
    F = 9.0 * DYN
    holes = pick(fs, lambda f: cyl(f) and abs(f["center_mm"][1] - 17.525) < 0.6 and f["center_mm"][0] in (8.0, 14.0) or
                 (cyl(f) and abs(f["center_mm"][1] - 17.525) < 0.6 and min(abs(f["center_mm"][0] - 8), abs(f["center_mm"][0] - 14)) < 0.6))
    post_bottom = pick(fs, lambda f: plane_n(f, 2, -1) and f["area_mm2"] < 100 and f["center_mm"][0] > 4)
    inner = pick(fs, lambda f: plane_n(f, 0, -1) and abs(f["center_mm"][0] - 3) < 0.3 and f["area_mm2"] > 1000)
    return {
        "material": PETG,
        "fixtures": [{"faces": holes + post_bottom, "type": "fixed"}],
        "loads": [{"faces": inner, "type": "force", "vector_N": [F, 0, 0]}],
        "margin": 2,
    }, f"grip {F:.1f} N (+X, inner face)"


def study_pinion(fs):
    """Tangential force on one tooth: 0.216 N.m / 12 mm / 2 racks x1.5."""
    F = 0.216 / 0.012 / 2 * DYN
    bottom = pick(fs, lambda f: plane_n(f, 2, -1))
    flank = pick(fs, lambda f: f["surface"] == "plane" and f["normal"] is not None and f["normal"][0] > 0.5
                 and 11.0 < f["center_mm"][1] < 13.8 and f["center_mm"][0] > 0)
    return {
        "material": PETG,
        "fixtures": [{"faces": bottom, "type": "fixed"}],
        "loads": [{"faces": flank, "type": "force", "vector_N": [-F, 0, 0]}],
        "margin": 2,
    }, f"{F:.1f} N on the tooth flank (tangential)"


def study_idler_plate(fs):
    """6804 radial load 15 N (more than the link's weight share x1.5), plate screwed to the bridge."""
    bore = pick(fs, lambda f: cyl(f) and abs(f["center_mm"][2] - Z_SH) < 0.6 and abs(f["area_mm2"] - 703.7) < 2, 1)
    holes = pick(fs, lambda f: cyl(f) and abs(abs(f["center_mm"][0]) - 14) < 0.6 and f["area_mm2"] > 100, 2)
    return {
        "material": PETG,
        "fixtures": [{"faces": holes, "type": "fixed"}],
        "loads": [{"faces": bore, "type": "force", "vector_N": [15.0, 0, -15.0]}],
        "margin": 2,
    }, "21 N on the bearing seat (45°)"


def study_turntable(fs):
    """Arm horizontal: tipping moment ~2 N.m x1.5 as a couple on the tower screws + weight 15 N."""
    M = 2.0 * DYN
    span = 0.050
    Fc = M / span / 2
    hub = pick(fs, lambda f: cyl(f) and abs(f["center_mm"][2] - 65.0) < 13 and f["area_mm2"] > 1000)
    under = pick(fs, lambda f: plane_n(f, 2, -1) and abs(f["center_mm"][2] - 82.0) < 0.3)
    front = pick(fs, lambda f: cyl(f) and f["center_mm"][0] > 24 and f["center_mm"][2] > 85, 2)
    back = pick(fs, lambda f: cyl(f) and f["center_mm"][0] < -24 and f["center_mm"][2] > 85 and abs(f["center_mm"][0]) < 26, 2)
    return {
        "material": PETG,
        "fixtures": [{"faces": hub, "type": "fixed"}, {"faces": under, "type": "roller"}],
        "loads": [
            {"faces": front, "type": "force", "vector_N": [0, 0, -2 * Fc - 7.5]},
            {"faces": back, "type": "force", "vector_N": [0, 0, 2 * Fc - 7.5]},
        ],
        "margin": 2,
    }, f"tipping couple ±{2 * Fc:.0f} N + weight 15 N"


STUDIES = {"fea_upper_arm": study_upper_arm, "fea_shoulder_tower": study_shoulder_tower,
           "fea_finger": study_finger, "fea_pinion": study_pinion,
           "fea_idler_plate": study_idler_plate, "fea_turntable": study_turntable}

if __name__ == "__main__":
    only = sys.argv[1:]
    summary = {}
    for name, doc in ids().items():
        if only and name not in only:
            continue
        study, desc = STUDIES[name](faces(doc))
        out = OUT / name
        out.mkdir(parents=True, exist_ok=True)
        (out / "study.json").write_text(json.dumps(study, indent=1), encoding="utf-8")
        r = subprocess.run([CAD, "model", "solve", doc, "--study", str(out / "study.json"), "--out", str(out), "--json"],
                           capture_output=True, text=True, cwd=ROOT)
        try:
            res = json.loads(r.stdout)
        except json.JSONDecodeError:
            print(name, "ERROR", r.stderr[-1500:])
            continue
        mm = res.get("measurements", {})
        row = {"load": desc, "max_von_mises_MPa": mm.get("max_von_mises_MPa"),
               "safety_factor": mm.get("safety_factor"), "max_displacement_mm": mm.get("max_displacement_mm"),
               "at_mm": mm.get("max_von_mises_at_mm"),
               "findings": [f["code"] + ": " + f["message"][:140] for f in res.get("findings", [])]}
        summary[name] = row
        print(name, json.dumps(row, ensure_ascii=False))
    (OUT / "summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False), encoding="utf-8")
