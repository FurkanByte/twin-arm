"""Printed parts, one by one (for STL export and print checks). Each part is rotated into its
print orientation so it sits flat on the bed, and lowered to z=0.

FDM, PETG, 0.2 mm layer, 0.4 mm nozzle.
Check: cad model dfm "$ID" --process printing --technology fdm --material petg --build-volume 220x220x250
"""
import build123d as bd
from cad import model

import parts as P
from lib.dims import Z_ELBOW, Z_SHOULDER, Z_WRIST


def on_bed(part, rot=None):
    """Rotates the part by `rot` (bd.Rot), centres it in XY and sets it on the bed."""
    label, material, color = part.label, part.material, part.color
    shape = rot * part if rot is not None else part
    bb = shape.bounding_box()
    shape = bd.Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * shape
    shape.label, shape.material, shape.color = label, material, color
    return shape


FLIP = bd.Rot(180, 0, 0)
Y_DOWN = bd.Rot(90, 0, 0)        # -Y face on the bed
Y_UP = bd.Rot(-90, 0, 0)         # +Y face on the bed


@model
def p01_base_plate():
    return on_bed(P.base_plate())


@model
def p02_base_housing():
    return on_bed(P.base_housing(), FLIP)                    # bearing face on the bed


@model
def p03_turntable():
    return on_bed(P.turntable(), FLIP)                       # turntable top face on the bed, hub up


@model
def p04_shoulder_tower():
    return on_bed(P.shoulder_tower())                        # flange on the bed (FEA: SF 12, along layers)


@model
def p05_upper_arm():
    return on_bed(P.upper_arm(), Y_UP)                       # horn cheek on the bed (flat face)


@model
def p06_forearm():
    return on_bed(P.forearm(), Y_UP)


@model
def p07_wrist():
    return on_bed(P.wrist(), FLIP)                           # servo ear seating face on the bed (flat)


@model
def p08_idler_plate_shoulder():
    return on_bed(P.idler_plate("idler_plate_shoulder", Z_SHOULDER, P.ACCENT), Y_DOWN)


@model
def p09_idler_plate_elbow():
    return on_bed(P.idler_plate("idler_plate_elbow", Z_ELBOW, P.ACCENT), Y_DOWN)


@model
def p10_idler_plate_wrist():
    return on_bed(P.idler_plate("idler_plate_wrist", Z_WRIST, P.DARK), Y_DOWN)


@model
def p11_gripper_palm():
    return on_bed(P.gripper_palm(), FLIP)                    # cover face on the bed (least support)


@model
def p12_gripper_cover():
    return on_bed(P.gripper_cover(), FLIP)


@model
def p13_pinion():
    return on_bed(P.gripper_pinion())


@model
def p14_rack_right():
    return on_bed(P.rack(1))


@model
def p15_rack_left():
    return on_bed(P.rack(-1))


@model
def p16_finger_right():
    return on_bed(P.finger(1)[0], bd.Rot(0, -90, 0))        # inner (pad) face on the bed


@model
def p17_finger_left():
    return on_bed(P.finger(-1)[0], bd.Rot(0, 90, 0))


@model
def p18_hub_collar():
    return on_bed(P.hub_collar())


@model
def p19_stub_retainer():
    return on_bed(P.stub_retainer(0.0), Y_DOWN)               # x3 (shoulder, elbow, wrist)


PRINTED = [p01_base_plate, p02_base_housing, p03_turntable, p04_shoulder_tower, p05_upper_arm,
           p06_forearm, p07_wrist, p08_idler_plate_shoulder, p09_idler_plate_elbow,
           p10_idler_plate_wrist, p11_gripper_palm, p12_gripper_cover, p13_pinion, p14_rack_right,
           p15_rack_left, p16_finger_right, p17_finger_left, p18_hub_collar,
           p19_stub_retainer]

if __name__ == "__main__":
    for m in PRINTED:
        m()
