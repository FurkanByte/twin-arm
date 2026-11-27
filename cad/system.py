"""TwinArm-5 system: arm + control box + main cable harnesses + 12 V power supply.

The box sits behind the arm (-X), its aviation connectors facing the arm. The main harness leaving
the arm's base (harness_base) splits into three GX20-12 plugs and one GX12-4 plug. The power
supply connects to the box's side wall via XT60.

    cad run cad/system.py
"""
import build123d as bd
from bd_materials import metals, plastics
from cad import model

import control_box as CB
import harness as HN
from arm import twinarm
from control_box import control_box
from lib.dims import PLATE_T

BOX_OFFSET = (-300.0, 105.0)          # box placement (rotated -90° about Z)
PSU = (199.0, 98.0, 42.0)             # HT-1115 12 V 10 A dimensions (approximate)
PSU_AT = (-290.0, 140.0)


def box_to_world(x, y, z):
    ox, oy = BOX_OFFSET
    return (y + ox, -x + oy, z)


def plug(label, x_local, r):
    """Cable-side plug of the connector (35 mm from the box towards the arm)."""
    x0, y0, z0 = box_to_world(x_local, CB.IY + CB.W + 18.0, CB.GX_Z)
    s = bd.Solid.make_cylinder(r, 35.0, bd.Plane(origin=(x0, y0, z0), z_dir=(1, 0, 0)))
    s.label, s.material, s.color = label, plastics.nylon(), "#9aa0a6"
    return s, (x0 + 35.0, y0, z0)


@model
def twinarm_system():
    arm = twinarm().shape
    arm.label = "twinarm"
    ox, oy = BOX_OFFSET
    box = bd.Pos(ox, oy, 0) * (bd.Rot(0, 0, -90) * control_box().shape)
    box.label = "control_box_assembly"
    kids = [arm, box]
    trunk_end = (-110.0, 0.0, PLATE_T + 2.0)                      # end of harness_base
    cables = []
    for (name, x, r) in CB.GX:
        p, tip = plug(name.replace("gx", "plug_gx"), x, r + 1.0)
        kids.append(p)
        rr = 2.2 if name.startswith("gx12") else 3.5
        u = (-48.0 / 48.3735, 0.0, -6.0 / 48.3735)                    # main harness last-segment direction
        p1 = (trunk_end[0] + 8 * u[0], 0.0, trunk_end[2] + 8 * u[2])
        cables.append(([trunk_end, p1, (p1[0] - 8, tip[1] * 0.5, p1[2]), (tip[0] + 14, tip[1], p1[2]),
                        (tip[0] + 8, tip[1], tip[2]), tip], rr))
    kids.append(HN.cable("harness_trunk_to_box", cables, 3.5))
    psu = bd.Pos(PSU_AT[0] + PSU[0] / 2, PSU_AT[1] + PSU[1] / 2, PSU[2] / 2) * bd.Box(*PSU)
    psu.label, psu.material, psu.color = "psu_12v_10a", metals.aluminum(), "#b0b4b8"
    kids.append(psu)
    xt = box_to_world(-CB.W - 1.0, CB.XT60_AT[0], CB.XT60_AT[1])
    kids.append(HN.cable("cable_psu_xt60", [[(PSU_AT[0] + 150.0, PSU_AT[1], 20.0), (PSU_AT[0] + 150.0, PSU_AT[1] - 8, 20.0),
                                              (xt[0], PSU_AT[1] - 15, 20.0),
                                              (xt[0], xt[1] + 6.0, xt[2])]], 2.5, "#c62828"))
    kids.append(HN.cable("cable_mains", [[(PSU_AT[0], PSU_AT[1] + 49.0, 20.0), (PSU_AT[0] - 20.0, PSU_AT[1] + 49.0, 20.0),
                                          (PSU_AT[0] - 80.0, PSU_AT[1] + 49.0, 5.0)]],
                         3.5, "#202020"))
    return bd.Compound(children=kids, label="twinarm_system")


if __name__ == "__main__":
    twinarm_system()
