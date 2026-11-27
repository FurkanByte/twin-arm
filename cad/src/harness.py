"""Cable harness, routed ENTIRELY INSIDE the arm.

Each link carries its own segment (it moves with the link):
- The servo cable leaves its own pocket into the internal cable duct in the block's -Y wall; the
  duct runs through the inside of the link along the beam and the bridge.
- Pitch joints (shoulder, elbow, wrist pitch): the harness passes through the HOLLOW STUB SHAFT
  (6804, Ø13 bore). The harness segments of the upper and lower links meet end to end on the shaft
  axis, along Y; this point does not move as the joint rotates (the cable only twists, it is
  never stretched or pinched). In the lower link the harness goes from the cable chamber of the
  idler plate / cable cover into the bridge duct, and from there into the beam's internal duct.
- Base yaw: closed cable chamber under the turntable (clock spring); wrist roll: closed chamber
  inside the gripper skirt. In the chambers the harness coils through ±90° (drawn as a gap
  between the two ends).
- The main harness to the control box leaves through the window at the back of the base (system.py).

Harness diameters follow from the conductor count: per servo 2x20 AWG power + 1x26 AWG PWM +
2x30 AWG pot, plus a shared 3x26 AWG 1-Wire.
"""
import math

import build123d as bd
from bd_materials import plastics

import parts as P
from lib.dims import (
    BRIDGE_GAP, CABLE_T, CABLE_W, CABLE_Z, CHAMBER_DEPTH, PLATE_T, SERVO_Y0, Z_BASE_TOP,
    Z_ROLL_HORN, Z_SHOULDER, Z_TURNTABLE, Z_WRIST,
)

SLEEVE_COLOR = "#2b4a8c"            # braided cable sleeve
YD = (P.DUCT_Y[0] + P.DUCT_Y[1]) / 2          # internal duct axis (y)
YJ = P.STUB_END - 0.8                         # on-axis junction point (stub end, inside the retainer)
YC = P.CUP_Y[0] + 4.3                          # harness turning plane in the cover chamber (outside retainer screws)

R_BASE, R_TOWER, R_UPPER, R_FORE, R_WRIST, R_GRIP, R_J1 = 4.4, 3.9, 3.8, 3.4, 3.0, 2.0, 2.0
R_LEAD = 1.1                        # flat servo cable (equivalent)


def _poly(points, r):
    shape = None
    for a, b in zip(points, points[1:]):
        va, vb = bd.Vector(*a), bd.Vector(*b)
        L = (vb - va).length
        if L < 1e-6:
            continue
        cyl = bd.Solid.make_cylinder(r, L, bd.Plane(origin=va, z_dir=(vb - va).normalized()))
        shape = cyl if shape is None else shape + cyl
    for p in points[1:-1]:
        shape = shape + bd.Pos(*p) * bd.Sphere(r)
    return shape


def cable(label, paths, r, color=SLEEVE_COLOR):
    """paths: point lists; if an element is (points, radius), that path is drawn with that radius."""
    shape = None
    for pts in paths:
        rr = r
        if isinstance(pts, tuple) and len(pts) == 2 and isinstance(pts[1], float):
            pts, rr = pts
        s = _poly(pts, rr)
        shape = s if shape is None else shape + s
    shape.label = label
    shape.material = plastics.pe()
    shape.color = color
    return shape


def length(paths):
    tot = 0.0
    for pts in paths:
        if isinstance(pts, tuple):
            pts = pts[0]
        for a, b in zip(pts, pts[1:]):
            tot += math.dist(a, b)
    return tot


def _pitch_lead(z_axis):
    """Flat cable of the pitch servo: exits the end face, through the pocket-duct slot into the duct."""
    yc = SERVO_Y0 + CABLE_Z + CABLE_T / 2
    zc = z_axis - P.B - 3.0                          # below the cable exit, in the pocket-duct slot
    return ([(0, yc, zc), (0, YD, zc)], 0.95)


def _parent(z_axis, z_from):
    """From the upper link's internal duct to the junction point on the joint axis."""
    return [(0, YD, z_from), (0, YD, z_axis), (0, YJ, z_axis)]


def _child(z_axis):
    """In the lower link: axis point -> idler cover chamber -> cover duct -> bridge -> internal duct."""
    zb = z_axis + BRIDGE_GAP
    zm = zb + (P.BRIDGE_DUCT_Z[0] + P.BRIDGE_DUCT_Z[1]) / 2
    return [(0, YJ, z_axis), (0, YC, z_axis), (0, YC, zm), (0, YD, zm), (0, YD, zb + P.BRIDGE_T + 6)]


def shoulder_harness():
    z = Z_SHOULDER
    run = _parent(z, P.TT_Z1 + 2)[::-1]                       # from the axis down along the duct
    down = [(0, YD, P.TT_Z1 + 2), (0, YD, P.TT_Z1 - 1.0), (P.TT_HOLE[0], P.TT_HOLE[1], Z_TURNTABLE - 0.5),
            (P.TT_HOLE[0], P.TT_HOLE[1], Z_BASE_TOP + 1.0)]     # via slanted duct to cable chamber
    return [cable("harness_shoulder", [run, down, _pitch_lead(z)], R_TOWER)]


def arm_harness(label, z_prox, z_dist, r):
    run = _child(z_prox)[:-1] + _parent(z_dist, z_prox + BRIDGE_GAP + P.BRIDGE_T + 6)
    return [cable(label, [run, _pitch_lead(z_dist)], r)]


def wrist_harness():
    z = Z_WRIST
    run = _child(z)[:-1] + [(0, YD, P.ROLL_Z_EAR + 6.0)]
    zl = P.ROLL_Z_BOT - 1.5                                    # J5 cable from under the servo into the duct
    lead = ([(P.B + 1.3, 0, P.ROLL_Z_BOT + CABLE_Z - 0.1), (P.B + 1.3, 0, zl), (2.0, P.DUCT_Y[1] + 1.0, zl), (2.0, YD + 2.0, zl),
             (0, YD, zl + 2.0)], R_LEAD)
    zc = P.ROLL_Z_EAR + 9.0                                    # coil in the cable chamber (clock spring)
    coil = [(0, YD, P.ROLL_Z_EAR + 6.0)] + [
        (36.0 * math.cos(math.radians(a)), 36.0 * math.sin(math.radians(a)), zc) for a in range(270, 361, 18)]
    return [cable("harness_wrist", [run, lead, coil], R_WRIST)]


def gripper_harness():
    zc = P.G_FLOOR + 1.3
    xl = P.MG_X1 + (P.MG_CABLE_T + 0.2) / 2                    # MG90S cable in its channel
    lead = ([(xl, 0, P.G_FLOOR + 1.9), (xl, 0, zc), (P.GRIP_HOLE[0], 0, zc)], 0.6)
    down = ([(P.GRIP_HOLE[0], 0, zc), (P.GRIP_HOLE[0], 0, Z_ROLL_HORN - 1.0)], 1.2)
    return [cable("harness_gripper", [lead, down], R_GRIP)]


def base_harness():
    """Drops from the cable chamber floor into the housing, exits the rear window; the J1 cable joins."""
    hx, hy = P.CHAMBER_HOLE
    zf = Z_BASE_TOP - CHAMBER_DEPTH
    trunk = [(hx, hy, zf + 6.0), (hx, hy, 44.0), (-12, -29.5, 38), (-21.2, -21.2, 33), (-29.5, -12, 29),
             (-40, 0, 27), (-56, 0, 27), (-62, 0, PLATE_T + 8), (-110, 0, PLATE_T + 2)]
    zc = PLATE_T + R_J1 + 1.0
    zl = PLATE_T - P.J1_POCKET + CABLE_Z + 2.0
    lead = ([(P.B + 1.3, -CABLE_W / 2 - 0.5, zl), (P.B + 1.3, -12.0, zl), (P.B + 1.3, -12.0, PLATE_T + 2.5),
             (P.B + 1.3, -18.0, zc)], R_LEAD)
    j1 = ([(P.B + 1.3, -18.0, zc), (P.B + 1.3, -20.0, zc), (0, -20, zc), (-25, -14, 16), (-40, -1, 26)], R_J1)
    return [cable("harness_base", [trunk, j1, lead], R_BASE)]
