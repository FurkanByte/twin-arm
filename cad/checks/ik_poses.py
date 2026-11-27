"""Computes the named poses from real targets with inverse kinematics (on the MuJoCo model).

    cad run checks/ik_poses.py

Write the output into the POSES dictionary in arm.py. Targets: a 30x30x60 mm object on the table at
x=170 mm, y=±100 mm; the gripper points down, with the pad centers on the upper half of the object.
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "checks"))

import build123d  # noqa: F401,E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from sim_dynamics import build, ik_best, key_targets, set_pose  # noqa: E402

OBJ_H = 0.06
TARGETS = {                       # TCP target (m)
    "pick_above": (0.17, -0.10, OBJ_H / 2 + 0.005 + 0.06),
    "pick":       (0.17, -0.10, OBJ_H / 2 + 0.005 + 0.012),
    "place_above": (0.17, 0.10, OBJ_H / 2 + 0.005 + 0.06),
    "place":      (0.17, 0.10, OBJ_H / 2 + 0.005 + 0.014),
}

if __name__ == "__main__":
    m, d = build(0)
    seeds = [key_targets(m, k) for k in ("rest", "ready", "home")]
    for tilt in (0, 5, 10, 15, 20, 25):          # smallest tilt common to all targets
        sols = {n: ik_best(m, seeds, t, tilts=(tilt,)) for n, t in TARGETS.items()}
        if all(v[1] < 1e-3 for v in sols.values()):
            break
    print("tilt", tilt)
    for name, (sol, err, _) in sols.items():
        set_pose(m, d, dict(key_targets(m, "home"), **sol))
        z_min = min(d.geom_xpos[g][2] - m.geom_rbound[g] for g in range(m.ngeom)
                    if m.body(m.geom_bodyid[g]).name.startswith("finger") and m.geom_contype[g])
        print(f'    "{name}": dict(' + ", ".join(f"{k}={math.degrees(v):.1f}" for k, v in sol.items()) + "),",
              f"# ik_err={err * 1000:.2f} mm, finger min z~{z_min * 1000:.0f} mm")
