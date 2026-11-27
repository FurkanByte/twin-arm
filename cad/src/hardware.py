"""Fasteners: every screw, insert, nut, set screw and foot is modelled at its real position.
Their labels are the BOM keys (checks/bom.py), e.g. "screw_M3x8_DIN912".

Convention: `at` is the point under the screw head (seating face), `d` is the direction in which
the shank enters the material (±X/±Y/±Z unit vector). Self-tapping screw shanks are drawn with the
core diameter that seats in the pilot hole (threads are not modelled).
"""
import math

import build123d as bd
from bd_materials import metals, plastics

import parts as P
from lib.dims import (
    CLEVIS_HALF, EAR_T, HORN_Y1, HUB_Z0, IDLER_Y0, MG_EAR_T, MG_EAR_Z, MG_HORN_T, SERVO_Y0,
    Y_EAR, Z_BASE_HORN, Z_ROLL_HORN,
)
from lib.geom import zcyl

STEEL = metals.alloy_steel()
BLACK = "#2a2a2a"
BRASS = "#c8a24a"

# head dimensions: (radius, height)
HEADS = {"DIN912": lambda d: (0.85 * d + 0.2, d), "ISO7380": lambda d: (0.95 * d, 0.55 * d),
         "PAN": lambda d: (0.95 * d, 0.6 * d), "DIN933": lambda d: (0.9 * d, 0.7 * d)}

_ROT = {(0, 0, -1): (0, 0, 0), (0, 0, 1): (180, 0, 0), (0, -1, 0): (-90, 0, 0),
        (0, 1, 0): (90, 0, 0), (-1, 0, 0): (0, 90, 0), (1, 0, 0): (0, -90, 0)}


def _place(shape, at, d):
    return bd.Pos(*at) * (bd.Rot(*_ROT[tuple(int(v) for v in d)]) * shape)


def _finish(shape, label, color=BLACK, material=None):
    shape.label = label
    shape.material = material or metals.alloy_steel()
    shape.color = color
    return shape


def screw(size, length, at, d, head="DIN912", label=None, core=None):
    """Metric screw. size: e.g. 'M3'; core: core radius for a self-tapping screw."""
    dia = float(size[1:])
    hr, hh = HEADS[head](dia)
    r = core if core is not None else dia / 2
    s = bd.Pos(0, 0, -length / 2) * bd.Cylinder(r, length) + bd.Pos(0, 0, hh / 2) * bd.Cylinder(hr, hh)
    lab = label or f"screw_{size}x{length:g}_{head}"
    return _finish(_place(s, at, d), lab)


def selftap(size, length, at, d, core, head="PAN"):
    return screw(size, length, at, d, head=head, core=core, label=f"selftap_{size}x{length:g}_{head}")


def insert(size, at, d, length=None):
    """Heat-set brass insert: outer radius = hole, inner = screw diameter; top flush with the surface."""
    dia = float(size[1:])
    ro = P.M3_INSERT if size == "M3" else P.M4_INSERT
    L = length or (5.75 if size == "M3" else 8.0)
    s = bd.Pos(0, 0, -L / 2) * (bd.Cylinder(ro, L) - bd.Cylinder(dia / 2, L))
    return _finish(_place(s, at, d), f"insert_{size}x{L:g}", BRASS, metals.brass())


def setscrew(size, length, at, d, core=None):
    dia = float(size[1:])
    s = bd.Pos(0, 0, -length / 2) * bd.Cylinder(core or dia / 2, length)
    return _finish(_place(s, at, d), f"setscrew_{size}x{length:g}")


def nut(size, at, d):
    """Hex nut; `at` is the face seated on the part, the nut extends away from the material."""
    dia = float(size[1:])
    h = 0.8 * dia
    s = bd.extrude(bd.RegularPolygon(0.92 * dia, 6), h) - bd.Pos(0, 0, h / 2) * bd.Cylinder(dia / 2, h * 2)
    return _finish(_place(s, at, d), f"nut_{size}")


def rubber_foot(at, r=5.0, h=3.0):
    s = bd.Pos(0, 0, -h / 2) * bd.Cylinder(r, h)
    return _finish(bd.Pos(*at) * s, "rubber_foot_10x3", "#1a1a1a", plastics.rubber())


# =======================================================================================
# Positions (candle pose)
# =======================================================================================
def pitch_servo_hw(z):
    """Pitch servo mounting to the block: near servo ear 2x insert + M3x8, far ear 2x M3x8
    self-tapping screw; stub-end retainer 2x M2x6 self-tapping screw."""
    out = []
    top = Y_EAR + EAR_T
    for x in (-P.EW, P.EW):
        out.append(insert("M3", (x, Y_EAR, z + P.EH[0]), (0, -1, 0)))
        out.append(screw("M3", 8, (x, top, z + P.EH[0]), (0, -1, 0)))
        out.append(selftap("M3", 8, (x, top, z + P.EH[1]), (0, -1, 0), core=P.M3_PILOT - 0.05))
    for dx, dz in P.RETAINER_SCREWS:                               # stub-end retainer: 2x M2x6
        out.append(selftap("M2", 6, (dx, P.STUB_END - P.RETAINER[2], z + dz), (0, 1, 0), core=0.75))
    return out


def clevis_hw(z):
    """Clevis of the next link: 2x M3x8 to the arm horn, horn centre screw, idler plate 2x insert +
    2x M3x8."""
    out = []
    y_cb = CLEVIS_HALF - 2.0
    out.append(screw("M3", 10, (0, y_cb, z + P.ARM_HORN_SCREW), (0, -1, 0)))
    out.append(screw("M3", 8, (0, HORN_Y1, z), (0, -1, 0), label="horn_screw_M3x8"))
    zb = z + P.BRIDGE_GAP + P.BRIDGE_T / 2
    for x in (-P.PLATE_SCREW_X, P.PLATE_SCREW_X):
        out.append(insert("M3", (x, IDLER_Y0, zb), (0, 1, 0)))
        out.append(screw("M3", 20, (x, P.Y_PLATE_OUT + 3.0, zb), (0, 1, 0)))
    return out


def vertical_servo_hw(z_ear, x_axis=0.0):
    """Vertical standard servo: near servo ear 2x insert + M3x8, far ear 2x M3x8 self-tapping screw."""
    out = []
    for y in (-P.EW, P.EW):
        x = x_axis - P.EH[0]
        out.append(insert("M3", (x, y, z_ear), (0, 0, -1)))
        out.append(screw("M3", 8, (x, y, z_ear + EAR_T), (0, 0, -1)))
        out.append(selftap("M3", 8, (x_axis - P.EH[1], y, z_ear + EAR_T), (0, 0, -1), core=P.M3_PILOT - 0.05))
    return out


def base_hw():
    out = vertical_servo_hw(P.PLATE_T - P.J1_POCKET + P.EAR_Z)
    for k in range(4):
        a = math.radians(45 + 90 * k)
        x, y = 40 * math.cos(a), 40 * math.sin(a)
        out.append(insert("M3", (x, y, P.PLATE_T), (0, 0, 1)))
        out.append(screw("M3", 10, (x, y, 3.3), (0, 0, 1)))                     # from below
        out.append(screw("M5", 16, (66 * math.cos(a), 66 * math.sin(a), P.PLATE_T - 3.5), (0, 0, -1),
                         head="DIN933", label="screw_M5x16_table"))
    return out


def shoulder_link_hw():
    out = []
    zc = sum(P.COLLAR_Z) / 2                                       # hub collar set screw + nut
    s = setscrew("M3", 4, (19.0, 0, zc), (-1, 0, 0), core=P.M3_PILOT - 0.05)
    out.append(s)
    out.append(screw("M3", 8, (0, 0, Z_BASE_HORN), (0, 0, -1), label="horn_screw_M3x8"))
    for k in P.PIN_HOLES:
        a = math.radians(45 + 90 * k)
        out.append(insert("M3", (P.HORN_BOLT_R * math.cos(a), P.HORN_BOLT_R * math.sin(a), HUB_Z0), (0, 0, 1)))
    for x, y in P.TOWER_SCREWS:
        out.append(insert("M3", (x, y, P.TT_Z1), (0, 0, -1)))
        out.append(screw("M3", 10, (x, y, P.TT_Z1 + 4.5), (0, 0, -1)))
    return out


def wrist_link_hw():
    return vertical_servo_hw(P.ROLL_Z_EAR)


def gripper_link_hw():
    out = []
    H = Z_ROLL_HORN
    out.append(screw("M3", 8, (0, 0, H), (0, 0, -1), label="horn_screw_M3x8"))
    for k in range(4):                                             # flange -> J5 horn
        a = math.radians(45 + 90 * k)
        out.append(selftap("M2.5", 8, (P.HORN_BOLT_R * math.cos(a), P.HORN_BOLT_R * math.sin(a), P.G_FLOOR - 2.2),
                           (0, 0, -1), core=1.0))
    z_ear_top = P.G_FLOOR + MG_EAR_Z + MG_EAR_T                    # MG90S servo ears
    for x in P.MG_EAR_X:
        out.append(selftap("M2", 8, (x, 0, z_ear_top), (0, 0, -1), core=0.8))
    for x, y in P.COVER_SCREWS:
        out.append(insert("M3", (x, y, P.COVER_Z0), (0, 0, -1)))
        out.append(screw("M3", 8, (x, y, P.COVER_Z1 - 1.8), (0, 0, -1), head="ISO7380"))
    return out


def pinion_link_hw():
    out = [screw("M2", 4, (0, 0, P.G_HORN_TOP), (0, 0, -1), head="PAN", label="horn_screw_M2x4")]
    for x, y in P.PINION_SCREWS:
        out.append(selftap("M2", 10, (x, y, P.RACK_Z1), (0, 0, -1), core=0.8))
    return out


def finger_hw(side):
    out = []
    yc = sum(P.RACK_BODY) / 2
    for x in P.FINGER_SCREWS_X:
        s = selftap("M3", 12, (x, yc, P.FOOT_Z1 - 5), (0, 0, -1), core=P.M3_PILOT - 0.05, head="DIN912")
        s.label = "screw_M3x12_DIN912"
        out.append(s if side > 0 else _rot180(s))
    return out


def _rot180(s):
    lab, mat, col = s.label, s.material, s.color
    s2 = bd.Rot(0, 0, 180) * s
    s2.label, s2.material, s2.color = lab, mat, col
    return s2
