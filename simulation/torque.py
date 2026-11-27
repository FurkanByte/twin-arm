"""Static gravity torque: the load carried by each pitch joint (kg.cm), at several poses and payloads.

The digital twin runs the same model live: the ratio of measured servo load / this estimate is the
"strain" indicator.

    cad run simulation/torque.py
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "cad"))

from cad.build123d import leaves, posed  # noqa: E402

from arm import twinarm  # noqa: E402
from lib.dims import L_FOREARM, L_UPPER, SERVOS, STALL_KGCM, Z_WRIST  # noqa: E402
from parts import PETG_DENSITY, PRINT_FILL, TCP_Z  # noqa: E402

TCP_FROM_WRIST = TCP_Z - Z_WRIST   # mm, from the J4 axis to the fingers' grip point
STALL = {j: STALL_KGCM[SERVOS[j][0]] for j in ("shoulder", "elbow", "wrist_pitch")}   # @6.6 V, kg.cm
SERVO_OF = {"servo_1_base": "base_yaw", "servo_2_shoulder": "shoulder", "servo_3_elbow": "elbow",
            "servo_4_wrist_pitch": "wrist_pitch", "servo_5_wrist_roll": "wrist_roll",
            "servo_6_gripper": "gripper"}
DENSITY = {"horn": 2.7e-3, "bearing": 7.85e-3, "bolt": 7.85e-3, "probe": 7.9e-3, "pad": 1.2e-3}
CHAIN = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_link",
         "gripper_link", "pinion_link", "finger_left_link", "finger_right_link"]
DOWNSTREAM = {"shoulder": CHAIN[2:], "elbow": CHAIN[3:], "wrist_pitch": CHAIN[4:]}


def leaf_mass_g(path, leaf):
    servo = next((p for p in path if p in SERVO_OF), None)
    if servo:
        return SERVOS[SERVO_OF[servo]][1] * (0.9 if leaf.label == "body" else 0.1)
    for prefix, rho in DENSITY.items():
        if leaf.label.startswith(prefix):
            return leaf.volume * rho
    return leaf.volume * PETG_DENSITY * PRINT_FILL


def joints_x(s, e, w):
    """X position of the joints (mm), with base rotation at 0."""
    sh = 0.0
    el = sh + L_UPPER * math.sin(math.radians(s))
    wr = el + L_FOREARM * math.sin(math.radians(s + e))
    tcp = wr + TCP_FROM_WRIST * math.sin(math.radians(s + e + w))
    return {"shoulder": sh, "elbow": el, "wrist_pitch": wr}, tcp


POSES = {
    "horizontal_full_reach (90,0,0)": (90, 0, 0),
    "reach (70,30,-10)": (70, 30, -10),
    "mid (45,45,0)": (45, 45, 0),
    "ready (20,65,60)": (20, 65, 60),
}

if __name__ == "__main__":
    total = 0.0
    for name, (s, e, w) in POSES.items():
        tree = posed(twinarm(), {"base_yaw": 0, "shoulder": s, "elbow": e, "wrist_pitch": w})
        items = [(path, leaf, leaf_mass_g(path, leaf), leaf.center().X) for path, leaf in leaves(tree, paths=True)]
        if not total:
            total = sum(m for _, _, m, _ in items)
            print(f"total arm mass ~{total:.0f} g (print infill {PRINT_FILL * 100:.0f}% effective)")
        jx, tcp_x = joints_x(s, e, w)
        print(f"\n{name}: TCP x={tcp_x:.0f} mm")
        for payload in (0, 150, 250):
            row = []
            for j, links in DOWNSTREAM.items():
                t = sum(m * (x - jx[j]) for path, _, m, x in items if path[1] in links)
                t += payload * (tcp_x - jx[j])
                kgcm = abs(t) / 1000 / 10            # g*mm -> kg*cm
                row.append(f"{j} {kgcm:5.1f} kg.cm ({kgcm / STALL[j] * 100:3.0f}%)")
            print(f"  payload {payload:3d} g: " + " | ".join(row))
