"""Individual parts for FEA, in their assembly positions (candle pose): loads and fixed faces
are selected in checks/fea_run.py by face position/normal.

    cad run checks/fea_parts.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import build123d  # noqa: F401,E402
from cad import model  # noqa: E402

import parts as P  # noqa: E402
from lib.dims import Z_SHOULDER  # noqa: E402


@model(name="fea_upper_arm")
def fea_upper_arm():
    return P.upper_arm()


@model(name="fea_shoulder_tower")
def fea_shoulder_tower():
    return P.shoulder_tower()


@model(name="fea_finger")
def fea_finger():
    return P.finger(1)[0]


@model(name="fea_pinion")
def fea_pinion():
    return P.gripper_pinion()


@model(name="fea_idler_plate")
def fea_idler_plate():
    return P.idler_plate("idler_plate_shoulder", Z_SHOULDER, P.LIGHT)


@model(name="fea_turntable")
def fea_turntable():
    return P.turntable()


ALL = [fea_upper_arm, fea_shoulder_tower, fea_finger, fea_pinion, fea_idler_plate, fea_turntable]

if __name__ == "__main__":
    for m in ALL:
        m()
