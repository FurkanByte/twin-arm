"""Rewrites all deliverable files:

    cad run cad/exports.py

- simulation/models/urdf/twinarm.urdf  URDF for ROS 2 / RViz / the digital twin (+ meshes/), 0 = candle pose
- simulation/models/mujoco/twinarm.xml -> written by simulation/mujoco_model.py (servo model, actuators)
- build/GLB/twinarm.glb           Web (three.js) digital-twin visual, including the "demo" animation
- build/STEP/twinarm.step         Full assembly
- build/STL/*.stl                 Printed parts in print orientation
"""
from pathlib import Path

from cad import model

from arm import twinarm
from control_box import control_box, control_box_print_body, control_box_print_lid
from print_parts import PRINTED

ROOT = Path(__file__).resolve().parents[1]          # repository root

if __name__ == "__main__":
    arm = twinarm()
    model.export(arm, ROOT / "simulation/models/urdf/twinarm.urdf", zero="home")
    # simulation/models/mujoco/twinarm.xml (servo actuators): cad run simulation/mujoco_model.py
    model.export(arm, ROOT / "build/GLB/twinarm.glb")
    model.export(arm, ROOT / "build/STEP/twinarm.step")
    model.export(control_box(), ROOT / "build/STEP/control_box.step")
    for part in PRINTED + [control_box_print_body, control_box_print_lid]:
        model.export(part(), ROOT / f"build/STL/{part.__name__}.stl", tolerance=0.02)
