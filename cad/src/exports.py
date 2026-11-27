"""Rewrites all deliverable files:

    cad run src/exports.py

- description/twinarm.urdf  URDF for ROS 2 / RViz / the digital twin (+ meshes/), 0 = candle pose
- sim/twinarm.xml           -> written by src/arm_mujoco.py (servo model, actuators)
- GLB/twinarm.glb           Web (three.js) digital-twin visual, including the "demo" animation
- STEP/twinarm.step         Full assembly
- STL/*.stl                 Printed parts in print orientation
"""
from pathlib import Path

from cad import model

from arm import twinarm
from control_box import control_box, control_box_print_body, control_box_print_lid
from print_parts import PRINTED

ROOT = Path(__file__).resolve().parents[1]

if __name__ == "__main__":
    arm = twinarm()
    model.export(arm, ROOT / "description/twinarm.urdf", zero="home")
    # sim/twinarm.xml (MuJoCo model with servo actuators): cad run src/arm_mujoco.py
    model.export(arm, ROOT / "GLB/twinarm.glb")
    model.export(arm, ROOT / "STEP/twinarm.step")
    model.export(control_box(), ROOT / "STEP/control_box.step")
    for part in PRINTED + [control_box_print_body, control_box_print_lid]:
        model.export(part(), ROOT / f"STL/{part.__name__}.stl", tolerance=0.02)
