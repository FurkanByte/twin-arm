"""MuJoCo model of TwinArm-5: kinematics/mass from the CAD + simulation choices.

Servo model (hobby servo = DC motor + gearbox + P controller):
  - torque limit     : actuator forcerange = stall torque (6.6 V)
  - back-EMF         : joint damping = stall torque / no-load speed  (motor curve at full voltage)
  - rotor inertia    : joint armature = J_rotor * N^2 (reflected through the gear ratio)
  - gear friction    : joint frictionloss
  - controller       : position actuator, kp reaches stall torque at ~3° error
Values are estimates from datasheets + typical cored DC motor values (listed in the README).

    cad run simulation/mujoco_model.py   ->  simulation/models/mujoco/twinarm.xml (+ scene.xml)
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "cad"))

import math

import build123d  # noqa: F401  (cad build123d integration)
import mujoco
from cad.mujoco import collisions, decompose, fit, parts, save, spec

from arm import RANGES, SERVO_VEL, twinarm
from lib.dims import SERVOS, STALL_KGCM

KGCM = 0.0980665                      # kg.cm -> N.m
NO_LOAD_SPEED = {"DS3218MG": math.radians(60) / 0.16, "DS3230MG": math.radians(60) / 0.16,
                 "RDS3235": math.radians(60) / 0.12, "MG90S": math.radians(60) / 0.08}   # rad/s @6 V
ARMATURE = {"DS3218MG": 0.004, "DS3230MG": 0.005, "RDS3235": 0.006, "MG90S": 0.0009}   # kg.m^2
FRICTION = {"DS3218MG": 0.04, "DS3230MG": 0.05, "RDS3235": 0.06, "MG90S": 0.005}       # N.m
KP_ERR = math.radians(3.0)            # position error at which stall torque is reached
# Mechanical stops (`cad model ranges`, relative to the home zero): the joint cannot go beyond them;
# the servo command (ctrlrange) is limited to ±90° by the servo stroke.
MECH_STOPS = {"base_yaw": (-100, 100), "shoulder": (-111, 111), "elbow": (-111, 111),
              "wrist_pitch": (-110, 101), "wrist_roll": (-100, 100)}   # cad model ranges; rotations: cable chamber

ARM_JOINTS = list(RANGES) + ["gripper_pinion"]
SERVO_OF = {j: SERVOS["gripper" if j == "gripper_pinion" else j][0] for j in ARM_JOINTS}

# Collision: internal parts other than the main printed parts (servo bodies inside their pockets,
# bolts, bearings, probe, horn) must not generate contacts; finger pads are rubber.
NO_COLLIDE = ("bearing", "bolt", "ds18b20", "horn", "servo_", "pinion", "rack_", "turntable",
              "thrust", "idler_plate", "setscrew", "gripper_cover", "screw_", "selftap_", "insert_",
              "nut_", "harness", "hub_collar", "rubber_foot", "stub_retainer")


def twinarm_mujoco() -> mujoco.MjSpec:
    s = spec(twinarm(), zero="home")

    for path in parts(s):
        leaf = path.split("/")[-1]
        parent = path.split("/")[-2]
        if any(leaf.startswith(p) or parent.startswith(p) for p in NO_COLLIDE):
            for g in collisions(s, path):
                s.delete(g)
    for side in ("left", "right"):
        for g in fit(s, f"pad_{side}", "box"):
            g.friction = [1.0, 0.01, 0.0002]          # TPU pad
            g.condim = 4
            g.solref = [0.004, 1.0]
        for g in decompose(s, f"finger_{side}", max_hulls=4):   # foot, tongue, finger separate
            g.friction = [0.5, 0.005, 0.0001]

    for j in ARM_JOINTS:
        servo = SERVO_OF[j]
        stall = STALL_KGCM[servo] * KGCM
        jt = s.joint(j)
        jt.armature = ARMATURE[servo]
        jt.damping = [stall / NO_LOAD_SPEED[servo], 0, 0]
        jt.frictionloss = FRICTION[servo]
        a = s.add_actuator(name=f"servo_{j}", target=j, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.set_to_position(kp=stall / KP_ERR, dampratio=1.0)
        a.ctrlrange = list(jt.range)              # servo stroke (joint limit in the CAD)
        a.ctrllimited = True
        if j in MECH_STOPS:
            jt.range = [math.radians(x) for x in MECH_STOPS[j]]
        a.forcelimited = True
        a.forcerange = [-stall, stall]
        s.add_sensor(name=f"q_{j}", type=mujoco.mjtSensor.mjSENS_JOINTPOS, objtype=mujoco.mjtObj.mjOBJ_JOINT, objname=j)
        s.add_sensor(name=f"tau_{j}", type=mujoco.mjtSensor.mjSENS_ACTUATORFRC, objtype=mujoco.mjtObj.mjOBJ_ACTUATOR, objname=f"servo_{j}")
    for side in ("left", "right"):                    # rack guide friction
        jt = s.joint(f"finger_{side}")
        jt.frictionloss = 0.3                         # N
        jt.damping = [2.0, 0, 0]                      # N.s/m
        jt.armature = 0.01
    s.add_exclude(bodyname1="finger_left_link", bodyname2="finger_right_link")   # touch when closed
    for eq in s.equalities:                           # gear mesh is rigid (not soft)
        eq.solref = [0.002, 1.0]
        eq.solimp = [0.99, 0.999, 0.001, 0.5, 2]
    s.option.timestep = 0.0005
    s.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    s.option.impratio = 10
    return s


if __name__ == "__main__":
    save(twinarm_mujoco(), str(Path(__file__).resolve().parent / "models" / "mujoco" / "twinarm.xml"))
