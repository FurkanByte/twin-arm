"""TwinArm-5: 5-axis desktop robot arm + parallel gripper (digital-twin capstone project).

Joints (with ESP32 PWM channel):
  1 base_yaw        base yaw              Z axis            DS3218MG
  2 shoulder        shoulder              Y axis (+ fwd)    RDS3235
  3 elbow           elbow                 Y axis            DS3230MG
  4 wrist_pitch     wrist pitch           Y axis            DS3218MG
  5 wrist_roll      wrist roll            arm axis          DS3218MG
  6 gripper_pinion  gripper servo (MG90S): a 16-tooth involute pinion drives two racks in opposite
                    directions; finger_left/right are coupled to it ("gripper" coupling). 0 = closed.

Digital-twin feedback comes from sensors added to the servos (README): potentiometer
(differential) -> ADS1115, current -> INA226 (R010 shunt), temperature -> DS18B20 (TO-92).

Zero pose: arm upright ("candle"); the servo centre position (1500 us) is calibrated to this pose;
gripper 0 = fingers closed (2 mm between pads).
Printed parts: FDM PETG, 0.2 mm layer.
Check: cad model dfm "$ID" --process printing --technology fdm --material petg
"""
import build123d as bd
from cad import model
from cad.build123d import (
    configuration_sequence, frame, limits, mass, mechanism_state, rack_and_pinion_pair,
)

import harness as HN
import hardware as HW
import parts as P
from lib.dims import (
    FINGER_ASBUILT, FINGER_TRAVEL, MG_HORN_D, MG_HORN_T, MM_PER_DEG, SERVOS, STALL_KGCM,
    Z_BASE_HORN, Z_ELBOW, Z_ROLL_HORN, Z_SHOULDER, Z_WRIST,
)

SERVO_VEL = 6.5        # ~0.16 s/60° -> 6.5 rad/s (DS32xx @6 V)

# Joint limits (degrees) = servo stroke ±90°. The mechanical stops (`cad model ranges`) are
# wider; since the horn mounts on the 25T spline in 14.4° steps, each joint's trim of up to ±7.2°
# is kept in software (README: calibration).
# On the pitch joints the servo stroke (180°) is shifted forward: the horn is mounted so that the
# servo centre position (1500 us) matches the joint's "centre" angle below. This reaches the bend
# angles needed for top-down grasping from the table; the ends stay within 1° of the mechanical stops.
RANGES = {
    "base_yaw": (-90, 90),          # centre 0   (±135 possible with the 270° servo variant)
    "shoulder": (-70, 105),         # centre +20 (stop +111: 5° software margin)
    "elbow": (-60, 105),            # centre +25 (stop +111)
    "wrist_pitch": (-60, 95),       # centre +20 (stop +101)
    "wrist_roll": (-90, 90),        # centre 0
}
SERVO_CENTER = {"base_yaw": 0, "shoulder": 20, "elbow": 25, "wrist_pitch": 20, "wrist_roll": 0}
PINION_MAX = FINGER_TRAVEL / MM_PER_DEG          # 105°: fingers 0 -> 22 mm
PINION_ASBUILT = FINGER_ASBUILT / MM_PER_DEG     # 0


def grip_for_width(width_mm, squeeze_mm=1.0):
    """Pinion angle (degrees) giving `width_mm` between the pads; closes further by squeeze."""
    p = max(0.0, (width_mm - 2.0) / 2 - squeeze_mm / 2)
    return round(p / MM_PER_DEG, 1)


GRIP_OPEN = round(PINION_MAX - 1.0, 1)           # 104°: ~46 mm
GRIP_IDLE = grip_for_width(5.0, 0)               # idle 5 mm gap: does not press on its own stop
GRIP_HOLD_30 = grip_for_width(30.0, 1.0)         # 1.0 mm squeeze while holding a 30 mm object
                                                 # (sim: no slip up to 250 g; at 0.25 mm 1.1 N, slips)

READY = {"base_yaw": 0, "shoulder": 0, "elbow": 60, "wrist_pitch": 60, "wrist_roll": 0}

# Named poses: pick/place poses were computed by checks/ik_poses.py from the target (30x30x60 mm
# object on the table, x=170, y=∓100 mm) with a common 15° tilt; pick_above -> pick is a 60 mm
# vertical descent.
POSES = {
    "home": dict(base_yaw=0, shoulder=0, elbow=0, wrist_pitch=0, wrist_roll=0, gripper=GRIP_IDLE),
    "rest": dict(base_yaw=0, shoulder=-20, elbow=90, wrist_pitch=75, wrist_roll=0, gripper=GRIP_IDLE),
    "ready": dict(READY, gripper=GRIP_OPEN),
    "pick_above": dict(base_yaw=-30.5, shoulder=24.4, elbow=53.1, wrist_pitch=87.5, wrist_roll=-30.3, gripper=GRIP_OPEN),
    "pick": dict(base_yaw=-30.5, shoulder=24.5, elbow=78.2, wrist_pitch=62.3, wrist_roll=-30.3, gripper=GRIP_OPEN),
    "grasp": dict(base_yaw=-30.5, shoulder=24.5, elbow=78.2, wrist_pitch=62.3, wrist_roll=-30.3, gripper=GRIP_HOLD_30),
    "carry": dict(base_yaw=0, shoulder=0, elbow=60, wrist_pitch=60, wrist_roll=0, gripper=GRIP_HOLD_30),
    "place_above": dict(base_yaw=30.5, shoulder=24.4, elbow=53.1, wrist_pitch=87.5, wrist_roll=30.3, gripper=GRIP_HOLD_30),
    "place": dict(base_yaw=30.5, shoulder=24.3, elbow=77.5, wrist_pitch=63.3, wrist_roll=30.3, gripper=GRIP_HOLD_30),
    "release": dict(base_yaw=30.5, shoulder=24.3, elbow=77.5, wrist_pitch=63.3, wrist_roll=30.3, gripper=GRIP_OPEN),
    "reach": dict(base_yaw=0, shoulder=70, elbow=20, wrist_pitch=0, wrist_roll=0, gripper=GRIP_IDLE),
}
DEMO = [(0, "rest"), (2, "ready"), (4, "pick_above"), (5.5, "pick"), (6.5, "grasp"),
        (8, "pick_above"), (10, "carry"), (12, "place_above"), (13.5, "place"), (14.5, "release"),
        (16, "place_above"), (18, "ready"), (20, "reach"), (22, "rest")]


def _servo(compound, joint):
    mass(compound, SERVOS[joint][1])
    return compound


def _revolute(name, parent, child, axis, angle=0.0, rng=None):
    """Defines the joint in the candle pose (before any link moves); connecting happens later."""
    j = bd.RevoluteJoint(name, parent, axis=axis, angular_range=rng or RANGES[name])
    bd.RigidJoint(name, child, j.location)
    return lambda: j.connect_to(child.joints[name], angle=angle)


@model
def twinarm():
    # --- links (each one a sub-assembly) ------------------------------------------------
    base = bd.Compound(children=[
        P.base_plate(), P.base_housing(), P.thrust_bearing(), P.radial_bearing(),
        _servo(P.servo_vertical("servo_1_base", Z_BASE_HORN), "base_yaw"),
        P.to92("ds18b20_1", P.C - 2.3, P.C + 2.3, 10.0, 10.0 + 3.6, 20.0, 24.6),
        *HW.base_hw(), *HN.base_harness(),
    ], label="base_link")

    shoulder = bd.Compound(children=[
        P.turntable(), P.disc_horn_z(Z_BASE_HORN), *P.turntable_pins(), P.hub_collar(), P.shoulder_tower(),
        _servo(P.servo_pitch("servo_2_shoulder", Z_SHOULDER), "shoulder"),
        P.pitch_sensor("ds18b20_2", Z_SHOULDER), P.stub_retainer(Z_SHOULDER),
        *HW.shoulder_link_hw(), *HW.pitch_servo_hw(Z_SHOULDER), *HN.shoulder_harness(),
    ], label="shoulder_link")

    upper = bd.Compound(children=[
        P.upper_arm(), P.arm_horn_y(Z_SHOULDER),
        P.idler_plate("idler_plate_shoulder", Z_SHOULDER, P.ACCENT), P.idler_bearing(Z_SHOULDER),
        _servo(P.servo_pitch("servo_3_elbow", Z_ELBOW), "elbow"),
        P.pitch_sensor("ds18b20_3", Z_ELBOW), P.stub_retainer(Z_ELBOW),
        *HW.clevis_hw(Z_SHOULDER), *HW.pitch_servo_hw(Z_ELBOW),
        *HN.arm_harness("harness_upper_arm", Z_SHOULDER, Z_ELBOW, HN.R_UPPER),
    ], label="upper_arm_link")

    fore = bd.Compound(children=[
        P.forearm(), P.arm_horn_y(Z_ELBOW),
        P.idler_plate("idler_plate_elbow", Z_ELBOW, P.ACCENT), P.idler_bearing(Z_ELBOW),
        _servo(P.servo_pitch("servo_4_wrist_pitch", Z_WRIST), "wrist_pitch"),
        P.pitch_sensor("ds18b20_4", Z_WRIST), P.stub_retainer(Z_WRIST),
        *HW.clevis_hw(Z_ELBOW), *HW.pitch_servo_hw(Z_WRIST),
        *HN.arm_harness("harness_forearm", Z_ELBOW, Z_WRIST, HN.R_FORE),
    ], label="forearm_link")

    wrist = bd.Compound(children=[
        P.wrist(), P.arm_horn_y(Z_WRIST),
        P.idler_plate("idler_plate_wrist", Z_WRIST, P.DARK), P.idler_bearing(Z_WRIST),
        _servo(P.servo_vertical("servo_5_wrist_roll", Z_ROLL_HORN), "wrist_roll"),
        P.wrist_sensor(),
        *HW.clevis_hw(Z_WRIST), *HW.wrist_link_hw(), *HN.wrist_harness(),
    ], label="wrist_link")

    palm = P.gripper_palm()
    frame(palm, "tcp", bd.Pos(0, 0, P.TCP_Z))                  # tool centre point
    grip = bd.Compound(children=[
        palm, P.disc_horn_z(Z_ROLL_HORN, hole_r=1.0), P.gripper_cover(),
        _servo(P.servo_mg90s("servo_6_gripper", P.G_HORN_TOP), "gripper"),
        P.gripper_sensor(),
        *HW.gripper_link_hw(), *HN.gripper_harness(),
    ], label="gripper_link")

    pinion = bd.Compound(children=[
        P.disc_horn_z(P.G_HORN_TOP, d=MG_HORN_D, t=MG_HORN_T, pilots=P.PINION_SCREWS), P.gripper_pinion(),
        *HW.pinion_link_hw(),
    ], label="pinion_link")

    fingers = {}
    for side, s in (("left", -1), ("right", 1)):
        body, pad = P.finger(s)
        fingers[side] = bd.Compound(children=[body, pad, P.rack(s), *HW.finger_hw(s)], label=f"finger_{side}_link")

    # --- joints ---------------------------------------------------------------------------
    # All joints are first defined in the candle pose, then connected from the base to the tip.
    Y = (0, 1, 0)
    connects = [
        _revolute("base_yaw", base, shoulder, bd.Axis((0, 0, 0), (0, 0, 1))),
        _revolute("shoulder", shoulder, upper, bd.Axis((0, 0, Z_SHOULDER), Y)),
        _revolute("elbow", upper, fore, bd.Axis((0, 0, Z_ELBOW), Y)),
        _revolute("wrist_pitch", fore, wrist, bd.Axis((0, 0, Z_WRIST), Y)),
        _revolute("wrist_roll", wrist, grip, bd.Axis((0, 0, Z_ROLL_HORN), (0, 0, 1))),
        # as the pinion turns + about -Z, the right rack moves +X and the left rack -X (opening)
        _revolute("gripper_pinion", grip, pinion, bd.Axis((0, 0, P.G_HORN_TOP), (0, 0, -1)),
                  angle=PINION_ASBUILT, rng=(0, PINION_MAX)),
    ]
    for side, s in (("left", -1), ("right", 1)):
        name = f"finger_{side}"
        slide = bd.Axis((0, 0, P.COVER_Z1), (s, 0, 0))
        j = bd.LinearJoint(name, grip, axis=slide, linear_range=(0, FINGER_TRAVEL))
        bd.RigidJoint(name, fingers[side], bd.Location(slide.position))
        connects.append(lambda j=j, f=fingers[side], n=name: j.connect_to(f.joints[n], position=FINGER_ASBUILT))
    for connect in connects:
        connect()

    arm = bd.Compound(children=[base, shoulder, upper, fore, wrist, grip, pinion,
                                fingers["left"], fingers["right"]], label="twinarm")

    rack_and_pinion_pair(arm, "gripper", gripper_pinion=1.0, finger_left=MM_PER_DEG,
                         finger_right=MM_PER_DEG)

    for j in RANGES:                                   # kg.cm -> N.m
        limits(arm, j, effort=round(STALL_KGCM[SERVOS[j][0]] * 0.0981, 2), velocity=SERVO_VEL)
    limits(arm, "gripper_pinion", effort=round(STALL_KGCM[SERVOS["gripper"][0]] * 0.0981, 3),
           velocity=10.0)
    for side in ("left", "right"):                     # 0.216 N.m / 12 mm / 2 fingers ~ 9 N
        limits(arm, f"finger_{side}", effort=10.0, velocity=0.12)

    for name, values in POSES.items():
        mechanism_state(arm, name, **values)
    return configuration_sequence(arm, "demo", DEMO)


if __name__ == "__main__":
    twinarm()
