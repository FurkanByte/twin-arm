"""Robot arm parts. Each function draws its part in place, in world coordinates, in the candle pose
(all joints 0); the assembly (arm.py) connects them through the joints.

Printed parts: FDM, PETG, 0.2 mm layers, 0.4 mm nozzle.

Assembly order (each pitch joint: shoulder, elbow, wrist pitch). Cables are fully internal:
  1. Press in the ear heat-set inserts (M3x5.75) and the cover inserts on the bridge's -Y face.
  2. Slide the servo into the pocket from +Y; its flat cable passes through the slot at the pocket
     bottom into the internal duct in the block's -Y wall. Ears: near ear M3x8 into the inserts,
     far ear M3x8 self-tapping screws.
  3. Glue the DS18B20 (TO-92) into the pocket beside the servo with thermal adhesive.
  4. Route the next link's harness through the internal duct and out of the HOLLOW STUB SHAFT
     bore (Ø20/Ø13).
  5. Screw the 25T aluminum arm horn to the next link's horn cheek with M3x10; push cheek+horn
     onto the spline and tighten the center screw (M3x8) through the hole in the cheek.
  6. Push the 6804-2RS onto the stub shaft and fasten the retainer washer with 2x M2x6
     self-tapping screws.
  7. Route the harness from the idler/cable cover's chamber into the bridge duct; fit the cover
     onto the bearing's outer ring and screw it to the bridge with 2x M3x20.
"""
import math

import build123d as bd
from bd_materials import metals, plastics
from bd_materials.processes import fdm
from cad.build123d import mass

from lib.dims import (
    ARM_HORN_HOLES, ARM_HORN_HUB_D, ARM_HORN_SCREW, ARM_HORN_L, ARM_HORN_W, AXIS_FROM_END, BEAM_HALF_Y, BLOCK_Y0,
    BLOCK_Y1, BRIDGE_GAP, BRIDGE_T, CABLE_T, CABLE_W, CABLE_Z, CHEEK_R, CLEVIS_HALF, EAR_HOLE_W,
    EAR_HOLES, EAR_L, EAR_T, EAR_Z, FIT, GEAR_BACKLASH, GEAR_M, HORN_BOLT_R, HORN_D, HORN_GAP,
    HORN_T, HORN_Y0, HORN_Y1, HUB_Z0, J1_POCKET, PLATE_T,
    IDLER_BRG_D, IDLER_BRG_d, IDLER_BRG_W, IDLER_LIP, IDLER_Y0, STUB_R_OUT, STUB_R_IN, PLATE_CAP,
    DUCT_HX, CHAMBER_R, CHAMBER_DEPTH,
    LINK_HALF_X, MG_A, MG_CABLE_T, MG_CABLE_W, MG_EAR_HOLES, MG_EAR_L, MG_EAR_T, MG_EAR_Z, MG_H,
    MG_HORN_D, MG_HORN_GAP, MG_HORN_T, MG_L, MG_W, PINION_R, PINION_Z, PROFILE_SHIFT, RAD_BRG,
    RAD_BRG_Z0, SERVO_H, SERVO_L, SERVO_W, SERVO_Y0, SERVO_Y1, THRUST_BRG, THRUST_Z0, TO92,
    TOWER_FLANGE_T, TURNTABLE_T, Y_EAR, Z_BASE_HORN, Z_BASE_TOP, Z_ROLL_HORN, Z_SHOULDER,
    Z_TURNTABLE, Z_WRIST, L_UPPER,
)
from lib.geom import box, stadium_y, xcyl, ycyl, zcyl

# Color palette: dark anthracite body, light gray links, orange accent
DARK = "#2b2f36"
LIGHT = "#e6e6e2"
ACCENT = "#ff6a13"
SERVO_BLACK = "#141414"
STEEL = "#9aa0a6"
ALU = "#b8bcc2"

HX = LINK_HALF_X
A = AXIS_FROM_END                 # shaft to near end (10)
B = SERVO_L - AXIS_FROM_END       # shaft to far end (30)
C = B - SERVO_L / 2               # shaft to body center (10)
EA = EAR_L / 2 - C                # shaft to near ear tip (17.25)
EB = EAR_L / 2 + C                # shaft to far ear tip (37.25)
EH = (EAR_HOLES / 2 - C, -EAR_HOLES / 2 - C)   # ear holes, from the shaft (14.75, -34.75)
EW = EAR_HOLE_W / 2               # two holes per ear: ±5 on the short axis
BLOCK_TOP = EA + FIT + 2.5        # axis to top of the servo block (20.05)

PETG_DENSITY = 1.27e-3            # g/mm^3
PRINT_FILL = 0.5                  # 4 walls + 25% gyroid infill ~ half the solid volume

M3_CLEAR, M3_HEAD = 1.7, 3.2      # radii
M3_INSERT = 2.0                   # M3x5.75 brass heat-set insert hole (Ø4.0)
M3_INSERT_DEPTH = 7.0
M3_PILOT = 1.25                   # pilot hole for M3 self-tapping/plastic screw (Ø2.5)
M4_CLEAR, M4_INSERT = 2.15, 2.8   # M4 clearance, M4x8 insert hole (Ø5.6)


def petg(color):
    return plastics.petg(process=fdm(layer_height_mm=0.2)), color


def finish(part, label, material, color, printed=False):
    part.label = label
    part.material = material
    part.color = color
    if printed:
        mass(part, part.volume * PETG_DENSITY * PRINT_FILL)
    return part


def disc_horn_holes(r, z0, z1, x=0.0, y=0.0, which=(0, 1, 2, 3)):
    """The disc horn's 4 holes (at 45° angles), Z-axis."""
    out = None
    for k in which:
        a = math.radians(45 + 90 * k)
        h = zcyl(r, z0, z1, x=x + HORN_BOLT_R * math.cos(a), y=y + HORN_BOLT_R * math.sin(a))
        out = h if out is None else out + h
    return out


# =======================================================================================
# Purchased parts
# =======================================================================================
def _servo(parts, label, hole=None):
    body = parts[0]
    for p in parts[1:]:
        body = body + p
    if hole is not None:
        body = body - hole
    body = finish(body, "body", plastics.nylon(), SERVO_BLACK)
    return bd.Compound(children=[body], label=label)


def servo_pitch(label, z):
    """Standard servo with its axis along +Y: the body extends down from the axis (into the link),
    the ears seat on the block's +Y face, the flat cable exits the far end face (-Z). The horn is
    a separate part."""
    case = box(-SERVO_W / 2, SERVO_W / 2, SERVO_Y0, SERVO_Y1, z - B, z + A)
    case = case.fillet(1.0, case.edges().filter_by(bd.Axis.Y))
    ears = box(-SERVO_W / 2, SERVO_W / 2, Y_EAR, Y_EAR + EAR_T, z - EB, z + EA)
    for h in EH:
        for x in (-EW, EW):
            ears -= ycyl(1.7, Y_EAR - 1, Y_EAR + EAR_T + 1, x=x, z=z + h)
    boss = ycyl(3.0, SERVO_Y1, HORN_Y0, z=z)
    cable = box(-CABLE_W / 2, CABLE_W / 2, SERVO_Y0 + CABLE_Z, SERVO_Y0 + CABLE_Z + CABLE_T,
                z - B - 2.0, z - B + 0.5)
    spline_hole = ycyl(1.5, SERVO_Y1 - 6.0, HORN_Y0 + 0.1, z=z)          # horn screw (M3)
    return _servo([case, ears, boss, cable], label, spline_hole)


def servo_vertical(label, z_horn_top, x_axis=0.0, y_axis=0.0):
    """Standard servo with its axis along +Z (base, wrist roll). Long side along +X; the flat cable
    exits the +X end face."""
    z_top = z_horn_top - HORN_T - HORN_GAP
    z_bot = z_top - SERVO_H
    xa, ya = x_axis, y_axis
    case = box(xa - A, xa + B, ya - SERVO_W / 2, ya + SERVO_W / 2, z_bot, z_top)
    case = case.fillet(1.0, case.edges().filter_by(bd.Axis.Z))
    ears = box(xa - EA, xa + EB, ya - SERVO_W / 2, ya + SERVO_W / 2, z_bot + EAR_Z, z_bot + EAR_Z + EAR_T)
    for h in EH:
        for y in (-EW, EW):
            ears -= zcyl(1.7, z_bot + EAR_Z - 1, z_bot + EAR_Z + EAR_T + 1, x=xa - h, y=ya + y)
    boss = zcyl(3.0, z_top, z_top + HORN_GAP, x=xa, y=ya)
    cable = box(xa + B - 0.5, xa + B + CABLE_T, ya - CABLE_W / 2, ya + CABLE_W / 2,
                z_bot + CABLE_Z, z_bot + CABLE_Z + 4.0)
    spline_hole = zcyl(1.5, z_top - 6.0, z_top + HORN_GAP + 0.1, x=xa, y=ya)
    return _servo([case, ears, boss, cable], label, spline_hole)


def servo_mg90s(label, z_horn_top):
    """MG90S, axis +Z (x=0,y=0), long side +X; cable exits the +X end face."""
    z_top = z_horn_top - MG_HORN_T - MG_HORN_GAP
    z_bot = z_top - MG_H
    mb = MG_L - MG_A
    case = box(-MG_A, mb, -MG_W / 2, MG_W / 2, z_bot, z_top)
    ex = (MG_EAR_L - MG_L) / 2
    ears = box(-MG_A - ex, mb + ex, -MG_W / 2, MG_W / 2, z_bot + MG_EAR_Z, z_bot + MG_EAR_Z + MG_EAR_T)
    for x in MG_EAR_X:
        ears -= zcyl(0.9, z_bot, z_top, x=x)
    boss = zcyl(2.4, z_top, z_top + MG_HORN_GAP)
    cable = box(mb - 0.5, mb + MG_CABLE_T, -MG_CABLE_W / 2, MG_CABLE_W / 2, z_bot + 2.0, z_bot + 5.0)
    spline_hole = zcyl(1.0, z_top - 4.0, z_top + MG_HORN_GAP + 0.1)
    return _servo([case, ears, boss, cable], label, spline_hole)


MG_EAR_X = (MG_L / 2 - MG_A - MG_EAR_HOLES / 2, MG_L / 2 - MG_A + MG_EAR_HOLES / 2)


def arm_horn_y(z):
    """25T aluminum single-arm horn, axis +Y; in the candle pose the arm points to +Z (the bridge)."""
    y0, y1 = HORN_Y0, HORN_Y1
    hub = ycyl(ARM_HORN_HUB_D / 2, y0, y1, z=z)
    arm = box(-ARM_HORN_W / 2, ARM_HORN_W / 2, y0, y1, z, z + ARM_HORN_L - ARM_HORN_W / 2)
    tip = ycyl(ARM_HORN_W / 2, y0, y1, z=z + ARM_HORN_L - ARM_HORN_W / 2)
    h = hub + arm + tip
    h -= ycyl(1.6, y0 - 1, y1 + 1, z=z)
    for d in ARM_HORN_HOLES:
        h -= ycyl(1.5, y0 - 1, y1 + 1, z=z + d)
    return finish(h, "horn", metals.aluminum(), ALU)


def disc_horn_z(z_horn_top, x=0.0, y=0.0, d=HORN_D, t=HORN_T, label="horn", pilots=None, hole_r=1.6):
    """Disc horn supplied with the servo (base, wrist roll); holes are enlarged to Ø3.2.
    `pilots`: if a list of (x, y) is given, only Ø1.6 pilot holes at those points (MG90S horn)."""
    z0 = z_horn_top - t
    h = zcyl(d / 2, z0, z_horn_top, x=x, y=y)
    h -= zcyl(1.6 if pilots is None else 1.1, z0 - 1, z_horn_top + 1, x=x, y=y)
    if pilots is None:
        h -= disc_horn_holes(hole_r, z0 - 1, z_horn_top + 1, x=x, y=y)
    else:
        for px, py in pilots:
            h -= zcyl(0.8, z0 - 1, z_horn_top + 1, x=x + px, y=y + py)
    return finish(h, label, plastics.nylon(), "#f2f2f2")


def to92(label, x0, x1, y0, y1, z0, z1):
    """DS18B20 TO-92, bonded to the servo body (thermal adhesive)."""
    return finish(box(x0, x1, y0, y1, z0, z1), label, plastics.abs_(), "#f2c94c")   # yellow heat-shrink


def thrust_bearing():
    """51107 thrust bearing (35 x 52 x 12): carries the weight of the turntable."""
    d, D, w = THRUST_BRG
    z0 = THRUST_Z0
    ring = zcyl(D / 2, z0, z0 + w) - zcyl(d / 2 + 0.1, z0 - 1, z0 + w + 1)
    groove = zcyl(D / 2 - 2, z0 + w / 2 - 0.4, z0 + w / 2 + 0.4) - zcyl(d / 2 + 2, z0, z0 + w)
    return finish(ring - groove, "bearing_51107", metals.alloy_steel(), STEEL)


def radial_bearing():
    """6006ZZ (30 x 55 x 13): centers the turntable hub and carries the tipping moment."""
    d, D, w = RAD_BRG
    ring = zcyl(D / 2, RAD_BRG_Z0, RAD_BRG_Z0 + w) - zcyl(d / 2, RAD_BRG_Z0 - 1, RAD_BRG_Z0 + w + 1)
    return finish(ring, "bearing_6006zz", metals.alloy_steel(), STEEL)


def idler_bearing(z):
    """6804-2RS thin-section bearing: inner ring on the hollow stub shaft (upper link), outer ring
    in the idler plate (lower link)."""
    y0 = IDLER_Y0 - IDLER_BRG_W
    r = ycyl(IDLER_BRG_D / 2, y0, IDLER_Y0, z=z) - ycyl(IDLER_BRG_d / 2, y0 - 1, IDLER_Y0 + 1, z=z)
    return finish(r, "bearing_6804", metals.alloy_steel(), STEEL)


STUB_END = IDLER_Y0 - IDLER_BRG_W                 # end of the hollow stub shaft
RETAINER = (6.0, 12.0, 1.6)                       # stub retainer washer: inner r (cables), outer r, thickness
RETAINER_SCREWS = [(-8.25, 0.0), (8.25, 0.0)]     # M2 self-tapping screws (x, z offset)


def stub_retainer(z):
    """Washer at the end of the hollow stub shaft: holds the bearing inner ring, 2x M2x6
    self-tapping screws."""
    ri, ro, t = RETAINER
    w = ycyl(ro, STUB_END - t, STUB_END, z=z) - ycyl(ri, STUB_END - t - 1, STUB_END + 1, z=z)
    for dx, dz in RETAINER_SCREWS:
        w -= ycyl(1.1, STUB_END - t - 1, STUB_END + 1, x=dx, z=z + dz)
    return finish(w, "stub_retainer", *petg(DARK), printed=True)


def grub_pin(label, x, y, z0, z1):
    """M3 setscrew, coupling pin."""
    return finish(zcyl(1.5, z0, z1, x=x, y=y), label, metals.alloy_steel(), "#3c3f44")


# =======================================================================================
# Shared features: servo block, pocket, clevis
# =======================================================================================
def servo_block_solid(z, z_bottom):
    blk = box(-HX, HX, BLOCK_Y0, BLOCK_Y1, z_bottom, z + BLOCK_TOP)
    top_edges = blk.edges().filter_by(bd.Axis.Y).group_by(bd.Axis.Z)[-1]
    return blk.fillet(6.0, top_edges)


TO92_GROOVE = (SERVO_W / 2 + FIT, SERVO_W / 2 + FIT + TO92[1] + 0.7)   # x range (body +X side)
DUCT_Y = (BLOCK_Y0 + 1.5, SERVO_Y0 - FIT - 1.5)   # internal cable duct: in the servo block's -Y wall


def servo_pocket(z, y_open=CLEVIS_HALF + 1):
    """Pitch servo pocket: body + ears (inserted from +Y), 4 ear inserts, cable duct and
    -X tunnel exit, DS18B20 pocket (open to +Y)."""
    pocket = box(-SERVO_W / 2 - FIT, SERVO_W / 2 + FIT, SERVO_Y0 - FIT, y_open, z - B - FIT, z + A + FIT)
    pocket += box(-SERVO_W / 2 - FIT, SERVO_W / 2 + FIT, Y_EAR, y_open, z - EB - 1.2, z + EA + 1.2)   # screw heads overhang
    for h, r in ((EH[0], M3_INSERT), (EH[1], M3_PILOT)):        # far ear: M3 self-tapping screw
        for x in (-EW, EW):
            pocket += ycyl(r, Y_EAR - M3_INSERT_DEPTH, Y_EAR + 0.1, x=x, z=z + h)
    cw = CABLE_W / 2 + 0.1
    pocket += box(-cw, cw, SERVO_Y0 - FIT, y_open, z - B - 4.0, z - B)          # insertion channel
    pocket += box(-DUCT_HX, DUCT_HX, DUCT_Y[1] - 0.5, SERVO_Y0 + 9.0, z - B - 4.0, z - B)   # cable: pocket -> internal duct
    pocket += box(TO92_GROOVE[0], TO92_GROOVE[1], SERVO_Y0 + 9.0, y_open, z - 22.0, z - 14.0)
    return pocket


def pitch_sensor(label, z):
    x0 = SERVO_W / 2
    return to92(label, x0, x0 + TO92[1], SERVO_Y0 + 10.0, SERVO_Y0 + 10.0 + TO92[0], z - 20.3, z - 15.7)


def idler_boss_add(z):
    """Hollow stub shaft (Ø20/Ø13) + shoulder pressing on the inner ring: protrudes from the
    block's -Y face."""
    stub = ycyl(STUB_R_OUT, STUB_END, BLOCK_Y0 + 0.5, z=z)
    shoulder = ycyl(11.6, IDLER_Y0, BLOCK_Y0 + 0.5, z=z)
    return stub + shoulder


def idler_boss_cut(z, z_duct_bottom):
    """Stub bore (cables) + stub-end washer screws + internal cable duct in the block's -Y wall."""
    cut = ycyl(STUB_R_IN, STUB_END - 1, DUCT_Y[1], z=z)
    for dx, dz in RETAINER_SCREWS:
        cut += ycyl(0.8, STUB_END - 0.1, STUB_END + 6, x=dx, z=z + dz)
    cut += box(-DUCT_HX, DUCT_HX, DUCT_Y[0], DUCT_Y[1], z_duct_bottom, z + STUB_R_IN)
    return cut


def _arm_horn_outline(z, y0, y1, off):
    """XZ outline of the arm horn (offset outward by off), extruded y0..y1 along Y."""
    hub = ycyl(ARM_HORN_HUB_D / 2 + off, y0, y1, z=z)
    arm = box(-ARM_HORN_W / 2 - off, ARM_HORN_W / 2 + off, y0, y1, z, z + ARM_HORN_L - ARM_HORN_W / 2)
    tip = ycyl(ARM_HORN_W / 2 + off, y0, y1, z=z + ARM_HORN_L - ARM_HORN_W / 2)
    return hub + arm + tip


PLATE_SCREW_X = 14.0
Y_PLATE_OUT = -(CLEVIS_HALF + PLATE_CAP)          # outer face of the idler cover
CUP_Y = (Y_PLATE_OUT + 1.5, STUB_END - IDLER_LIP) # cable chamber inside the cover (y)
BRIDGE_DUCT_Z = (1.8, BRIDGE_T - 1.8)             # height of the duct in the bridge (relative to the bridge)


def clevis(z):
    """Clevis of the next link: +Y cheek screwed to the arm horn + bridge. -Y cheek is the idler plate."""
    y0, y1 = HORN_Y1, CLEVIS_HALF
    cheek = ycyl(CHEEK_R, y0, y1, z=z) + box(-HX, HX, y0, y1, z, z + BRIDGE_GAP + 1)
    cheek -= ycyl(3.2, y0 - 1, y1 + 1, z=z)                           # access to the horn center screw
    cheek -= ycyl(M3_CLEAR, y0 - 1, y1 + 1, z=z + ARM_HORN_SCREW)
    cheek -= ycyl(M3_HEAD, y1 - 2.0, y1 + 1, z=z + ARM_HORN_SCREW)
    # horn locking frame: 1.5 mm horn-shaped pocket on the cheek's inner face (torque is transferred by shape)
    rim = ycyl(CHEEK_R, y0 - 1.5, y0, z=z) + box(-HX, HX, y0 - 1.5, y0, z, z + BRIDGE_GAP + 1)
    rim -= _arm_horn_outline(z, y0 - 2.0, y0 + 0.1, 0.15)
    rim -= ycyl(3.2, y0 - 2, y0 + 1, z=z)
    cheek += rim
    zb = z + BRIDGE_GAP
    bridge = box(-HX, HX, IDLER_Y0, CLEVIS_HALF, zb, zb + BRIDGE_T)
    body = cheek + bridge
    return body - clevis_cuts(z)


def clevis_cuts(z):
    """Bridge duct (cables from the idler cover -> the link's internal duct) + cover inserts."""
    zb = z + BRIDGE_GAP
    cut = box(-DUCT_HX, DUCT_HX, IDLER_Y0 - 0.01, DUCT_Y[1], zb + BRIDGE_DUCT_Z[0], zb + BRIDGE_DUCT_Z[1])
    cut += box(-DUCT_HX, DUCT_HX, DUCT_Y[0], DUCT_Y[1], zb + BRIDGE_DUCT_Z[0], zb + BRIDGE_T + 0.01)
    for x in (-PLATE_SCREW_X, PLATE_SCREW_X):
        cut += ycyl(M3_INSERT, IDLER_Y0 - 1, IDLER_Y0 + M3_INSERT_DEPTH, x=x, z=zb + BRIDGE_T / 2)
    return cut


def idler_plate(label, z, color):
    """The clevis's -Y cheek + cable cover (single print): 6804 pocket + lip; in the outer layer, a
    chamber collecting the cables from the hollow stub shaft end and a closed duct up to the
    bridge; 2x M3x20."""
    y_out, y_in = Y_PLATE_OUT, IDLER_Y0
    zb = z + BRIDGE_GAP
    plate = ycyl(CHEEK_R, y_out, y_in, z=z) + box(-HX, HX, y_out, y_in, z, zb + BRIDGE_T)
    plate -= ycyl(IDLER_BRG_D / 2, y_in - IDLER_BRG_W, y_in + 1, z=z)          # 6804 (press fit)
    plate -= ycyl(IDLER_BRG_D / 2 - 2.5, CUP_Y[1] - 0.01, y_in - IDLER_BRG_W + 0.01, z=z)   # lip
    plate -= ycyl(13.0, CUP_Y[0], CUP_Y[1], z=z)                               # cable chamber
    plate -= box(-DUCT_HX, DUCT_HX, CUP_Y[0], CUP_Y[1], z, zb + BRIDGE_DUCT_Z[1])     # cover duct
    plate -= box(-DUCT_HX, DUCT_HX, CUP_Y[1] - 0.01, y_in + 0.01, zb + BRIDGE_DUCT_Z[0], zb + BRIDGE_DUCT_Z[1])
    for x in (-PLATE_SCREW_X, PLATE_SCREW_X):
        plate -= ycyl(M3_CLEAR, y_out - 1, y_in + 1, x=x, z=zb + BRIDGE_T / 2)
        plate -= ycyl(M3_HEAD, y_out - 1, y_out + 3.0, x=x, z=zb + BRIDGE_T / 2)
    return finish(plate, label, *petg(color), printed=True)


def tie_slots(z_center):
    """Zip-tie bridge: two slots for a cable harness running along the -X face."""
    out = None
    for dz in (-5.0, 5.0):
        s = box(-HX + 2.0, -HX + 5.0, -BEAM_HALF_Y - 1, BEAM_HALF_Y + 1, z_center + dz - 2, z_center + dz + 2)
        out = s if out is None else out + s
    return out


def beam(z0, z1):
    """Body between two joints: box beam with lightening pockets in its sides."""
    b = box(-HX, HX, -BEAM_HALF_Y, BEAM_HALF_Y, z0, z1)
    b = b.fillet(4.0, b.edges().filter_by(bd.Axis.Z))
    if z1 - z0 > 30:                                  # lightening only on +Y face (-Y has the internal duct)
        b -= stadium_y(10.0, z0 + 7, z1 - 7, BEAM_HALF_Y - 2.4, BEAM_HALF_Y + 1, x=4.0)
    return b


def arm_link(z_prox, z_dist):
    """Upper arm / forearm: proximal clevis + beam + distal servo block."""
    z_beam0 = z_prox + BRIDGE_GAP + BRIDGE_T
    z_beam1 = z_dist - 30.0
    body = clevis(z_prox) + beam(z_beam0, z_beam1) + servo_block_solid(z_dist, z_dist - EB - FIT - 3.0)
    body -= servo_pocket(z_dist)
    body += idler_boss_add(z_dist)
    body -= idler_boss_cut(z_dist, z_prox + BRIDGE_GAP + BRIDGE_DUCT_Z[0])   # internal duct runs end to end
    return body


# =======================================================================================
# Base and shoulder
# =======================================================================================
def _vertical_servo_cuts(z_bot, x_axis=0.0, y_axis=0.0, sideways=False):
    """Common cuts in a vertical standard servo's seat: cable channel + exit tunnel
    (sideways=True: tunnel exits to -Y, beside the base post)."""
    xa = x_axis
    cw = CABLE_W / 2 + FIT
    cut = box(xa + B, xa + B + CABLE_T + 0.6, y_axis - cw, y_axis + cw, z_bot - FIT, z_bot + SERVO_H + 10)
    if sideways:
        cut += box(xa + B - 0.5, xa + B + CABLE_T + 0.6, y_axis - SERVO_W / 2 - 4, y_axis + cw,
                   z_bot + CABLE_Z - 1, z_bot + CABLE_Z + 5)
    else:
        cut += box(xa + B - 0.5, xa + EB + 6.0, y_axis - cw, y_axis + cw, z_bot + CABLE_Z - 1, z_bot + CABLE_Z + 5)
    return cut


def _vertical_ear_holes(z_ear):
    """Near ear: 2x M3 inserts; far ear (beside the cable channel): 2x M3 pilot holes (self-tapping)."""
    cut = None
    for h, r in ((EH[0], M3_INSERT), (EH[1], M3_PILOT)):
        for y in (-EW, EW):
            c = zcyl(r, z_ear - M3_INSERT_DEPTH, z_ear + 1, x=-h, y=y)
            cut = c if cut is None else cut + c
    return cut


def base_plate():
    """Base bottom plate (9 mm): bolted to the table with 4x M5; the J1 servo sits in a 6 mm pocket
    and its ears are screwed to two posts (printed flat)."""
    plate = zcyl(80.0, 0.0, PLATE_T)
    plate = plate.fillet(2.0, plate.edges().group_by(bd.Axis.Z)[-1])
    z_bot = PLATE_T - J1_POCKET
    z_ear = z_bot + EAR_Z
    posts = (box(-EA - 2.5, -A - FIT, -SERVO_W / 2, SERVO_W / 2, PLATE_T, z_ear)
             + box(B + FIT, EB + 2.5, -SERVO_W / 2, SERVO_W / 2, PLATE_T, z_ear))
    body = plate + posts
    body -= box(-A - FIT, B + FIT, -SERVO_W / 2 - FIT, SERVO_W / 2 + FIT, z_bot, PLATE_T + 1)   # servo pocket
    body -= _vertical_ear_holes(z_ear)
    body -= _vertical_servo_cuts(z_bot, sideways=True)
    for k in range(4):                                            # 4x M5 table mounting
        a = math.radians(45 + 90 * k)
        x, y = 66 * math.cos(a), 66 * math.sin(a)
        body -= zcyl(2.75, -1, PLATE_T + 1, x=x, y=y)
        body -= zcyl(5.0, PLATE_T - 3.5, PLATE_T + 1, x=x, y=y)
    for k in range(4):                                            # 4x M3 housing screws (from below)
        a = math.radians(45 + 90 * k)
        body -= zcyl(M3_CLEAR, -1, PLATE_T + 1, x=40 * math.cos(a), y=40 * math.sin(a))
        body -= zcyl(M3_HEAD, -1, 3.3, x=40 * math.cos(a), y=40 * math.sin(a))
    return finish(body, "base_plate", *petg(DARK), printed=True)


DECK_Z0 = 48.0
CHAMBER_HOLE = (0.0, -36.0)       # fixed hole in the floor of the cable chamber
TT_HOLE = (0.0, -34.0)            # cable exit on the underside of the turntable


def base_housing():
    """Base housing: shell around the J1 servo + 6006ZZ seat (pressed in from below) + 51107 seat.
    Printed upside down (top face on the bed). The arm cable runs outside to the rear window,
    held with zip ties."""
    shell = zcyl(46.0, PLATE_T, Z_BASE_TOP) - zcyl(42.0, PLATE_T - 1, DECK_Z0)
    body = shell + zcyl(46.0, DECK_Z0, Z_BASE_TOP)
    for k in range(4):                                            # M3 insert bosses
        a = math.radians(45 + 90 * k)
        x, y = 40 * math.cos(a), 40 * math.sin(a)
        body += zcyl(4.5, PLATE_T, DECK_Z0, x=x, y=y) - zcyl(M3_INSERT, PLATE_T - 1, PLATE_T + M3_INSERT_DEPTH, x=x, y=y)
    d, D, w = RAD_BRG
    body -= zcyl(D / 2 + 0.05, DECK_Z0 - 1, RAD_BRG_Z0 + w)       # 6006 seat (from below)
    body -= zcyl(16.0, RAD_BRG_Z0, Z_BASE_TOP + 1)                # hub passage
    body -= zcyl(THRUST_BRG[1] / 2 + 0.3, THRUST_Z0, Z_BASE_TOP + 1)   # 51107 seat
    body -= box(-50, -36, -16, 16, PLATE_T + 8.0, 42.0)           # rear cable exit (to the enclosure)
    # cable chamber: closed ring under the turntable; the cable coils here over ±90°
    r0, r1 = CHAMBER_R
    body -= zcyl(r1, Z_BASE_TOP - CHAMBER_DEPTH, Z_BASE_TOP + 1) - zcyl(r0, Z_BASE_TOP - CHAMBER_DEPTH - 1, Z_BASE_TOP + 2)
    body -= zcyl(6.0, DECK_Z0 - 1, Z_BASE_TOP - CHAMBER_DEPTH + 0.1, x=CHAMBER_HOLE[0], y=CHAMBER_HOLE[1])
    top_edge = body.edges().filter_by(bd.GeomType.CIRCLE).group_by(bd.Axis.Z)[-1].sort_by(
        lambda e: -e.radius)[0]
    body = body.fillet(2.0, [top_edge])
    return finish(body, "base_housing", *petg(DARK), printed=True)


TT_Z1 = Z_TURNTABLE + TURNTABLE_T
TOWER_SCREWS = [(-25.0, -19.0), (25.0, -19.0), (-25.0, 3.0), (25.0, 3.0)]
PIN_HOLES = (0, 2)                # two of the disc horn holes (45°, 225°)


def turntable():
    """Turntable + hub (one piece, printed with its top face on the bed). The hub turns in the
    6006ZZ inner ring and has a locating ring that centers the 51107's upper washer. Torque from
    the J1 horn goes only through 2 pins (M3 setscrews): 1 mm axial gap, the servo shaft carries
    no moment."""
    disc = zcyl(48.0, Z_TURNTABLE, TT_Z1)
    disc = disc.fillet(2.0, disc.edges().group_by(bd.Axis.Z)[-1])
    spigot = zcyl(THRUST_BRG[0] / 2 - 0.2, Z_TURNTABLE - 3.0, Z_TURNTABLE + 0.5)
    hub = zcyl(RAD_BRG[0] / 2, HUB_Z0, Z_TURNTABLE - 2.9)
    body = disc + spigot + hub
    body -= zcyl(3.2, HUB_Z0 - 1, TT_Z1 + 1)                       # access to the J1 horn screw
    body -= disc_horn_holes(M3_INSERT, HUB_Z0 - 1, HUB_Z0 + M3_INSERT_DEPTH, which=PIN_HOLES)
    for x, y in TOWER_SCREWS:
        body -= zcyl(M3_INSERT, TT_Z1 - M3_INSERT_DEPTH, TT_Z1 + 1, x=x, y=y)
    # angled internal duct from the tower duct down to the cable chamber
    a = bd.Vector(0, (DUCT_Y[0] + DUCT_Y[1]) / 2, TT_Z1 + 0.5)
    b = bd.Vector(TT_HOLE[0], TT_HOLE[1], Z_TURNTABLE - 0.5)
    u = (b - a).normalized()
    a2, b2 = a - u * 4.0, b + u * 4.0
    body -= bd.Solid.make_cylinder(6.5, (b2 - a2).length, bd.Plane(origin=a2, z_dir=u))
    body -= box(-DUCT_HX, DUCT_HX, DUCT_Y[0], DUCT_Y[1], TT_Z1 - 4, TT_Z1 + 1)
    # skirt: closes the gap between turntable and housing (cable chamber and bearing stay inside)
    body += zcyl(48.0, Z_BASE_TOP + 0.5, Z_TURNTABLE) - zcyl(46.5, Z_BASE_TOP, Z_TURNTABLE + 0.1)
    return finish(body, "turntable", *petg(DARK), printed=True)


def turntable_pins():
    out = []
    for k in PIN_HOLES:
        a = math.radians(45 + 90 * k)
        out.append(grub_pin("setscrew_M3x10", HORN_BOLT_R * math.cos(a), HORN_BOLT_R * math.sin(a),
                            Z_BASE_HORN - HORN_T + 0.5, HUB_Z0 + 5.5))
    return out


COLLAR_Z = (HUB_Z0 + 0.3, RAD_BRG_Z0)


def hub_collar():
    """Hub retaining collar: clamped to the hub below the 6006's inner ring with an M3 setscrew
    (with nut trap); when the turntable tries to lift, it holds on to the base through the bearing."""
    z0, z1 = COLLAR_Z
    c = zcyl(19.0, z0, z1) - zcyl(RAD_BRG[0] / 2, z0 - 1, z1 + 1)   # contacts the inner ring only
    zc = (z0 + z1) / 2
    c -= xcyl(M3_PILOT, 14.0, 20.0, z=zc)                              # M3x4 setscrew (pilot hole)
    return finish(c, "hub_collar", *petg(DARK), printed=True)


def shoulder_tower():
    """Shoulder servo tower: 8 mm flange screwed to the turntable with 4x M3 (4.5 mm under the heads)."""
    flange = box(-30, 30, BLOCK_Y0, BLOCK_Y1, TT_Z1, TT_Z1 + TOWER_FLANGE_T)
    flange = flange.fillet(3.0, flange.edges().filter_by(bd.Axis.Z))
    body = flange + servo_block_solid(Z_SHOULDER, TT_Z1)
    # 45° support ribs in the block-flange corner (±X)
    for s in (-1, 1):
        rib = bd.Pos(s * (HX + 3.0), (BLOCK_Y0 + BLOCK_Y1) / 2, TT_Z1 + TOWER_FLANGE_T) * bd.Rot(0, s * 45, 0) \
            * bd.Box(6.0, BLOCK_Y1 - BLOCK_Y0, 6.0)
        body += rib & box(-30, 30, BLOCK_Y0, BLOCK_Y1, TT_Z1 + TOWER_FLANGE_T, TT_Z1 + TOWER_FLANGE_T + 6)
    body -= servo_pocket(Z_SHOULDER, y_open=BLOCK_Y1 + 1)
    for x, y in TOWER_SCREWS:
        body -= zcyl(M3_CLEAR, TT_Z1 - 1, TT_Z1 + TOWER_FLANGE_T + 1, x=x, y=y)
        body -= zcyl(M3_HEAD, TT_Z1 + 4.5, TT_Z1 + TOWER_FLANGE_T + 7, x=x, y=y)
    body += idler_boss_add(Z_SHOULDER)
    body -= idler_boss_cut(Z_SHOULDER, TT_Z1 - 1)                 # internal duct runs down to the turntable
    return finish(body, "shoulder_tower", *petg(DARK), printed=True)


def upper_arm():
    return finish(arm_link(Z_SHOULDER, Z_SHOULDER + L_UPPER), "upper_arm", *petg(ACCENT), printed=True)


def forearm():
    return finish(arm_link(Z_SHOULDER + L_UPPER, Z_WRIST), "forearm", *petg(ACCENT), printed=True)


# =======================================================================================
# Wrist
# =======================================================================================
ROLL_Z_TOP = Z_ROLL_HORN - HORN_T - HORN_GAP
ROLL_Z_BOT = ROLL_Z_TOP - SERVO_H
ROLL_Z_EAR = ROLL_Z_BOT + EAR_Z


def wrist():
    """Wrist: J4 clevis + J5 (wrist roll) servo seat. Cables run up from the bridge through the
    internal duct in the housing's -Y wall and open into the cable chamber inside the gripper
    skirt; the J5 servo cable enters the same duct from under the servo."""
    zb = Z_WRIST + BRIDGE_GAP
    housing = box(-EA - 2.5, EB + 2.5, BLOCK_Y0, HX, zb, ROLL_Z_EAR)
    housing = housing.fillet(3.0, housing.edges().filter_by(bd.Axis.Z))
    body = clevis(Z_WRIST) + housing
    body -= clevis_cuts(Z_WRIST)
    body -= box(-A - FIT, B + FIT, -SERVO_W / 2 - FIT, SERVO_W / 2 + FIT, ROLL_Z_BOT - FIT, ROLL_Z_EAR + 1)
    body -= _vertical_ear_holes(ROLL_Z_EAR)
    cw = CABLE_W / 2 + FIT
    body -= box(B, B + CABLE_T + 0.6, -cw, cw, ROLL_Z_BOT - FIT, ROLL_Z_EAR + 1)          # insertion channel
    body -= box(0, B + CABLE_T + 0.6, DUCT_Y[1] - 0.5, cw, ROLL_Z_BOT - 3.0, ROLL_Z_BOT + 0.01)   # floor channel
    body -= box(-DUCT_HX, DUCT_HX, DUCT_Y[0], DUCT_Y[1], zb + BRIDGE_DUCT_Z[0], ROLL_Z_EAR + 1)   # internal duct
    body -= box(C - 3.0, C + 3.0, SERVO_W / 2 + FIT, SERVO_W / 2 + FIT + TO92[1] + 0.7,
                ROLL_Z_BOT + 8, ROLL_Z_EAR + 1)                    # DS18B20 pocket (from above)
    return finish(body, "wrist", *petg(DARK), printed=True)


def wrist_sensor():
    y0 = SERVO_W / 2
    return to92("ds18b20_5", C - 2.3, C + 2.3, y0, y0 + TO92[1], ROLL_Z_BOT + 12, ROLL_Z_BOT + 16.6)


# =======================================================================================
# Gripper: MG90S + module 1.5 involute rack-and-pinion parallel gripper
# =======================================================================================
H = Z_ROLL_HORN
G_FLOOR = H + 6.5                                # MG90S body bottom
G_SERVO_TOP = G_FLOOR + MG_H
G_HORN_TOP = G_SERVO_TOP + MG_HORN_GAP + MG_HORN_T
RACK_Z0 = G_HORN_TOP + 0.5
RACK_Z1 = RACK_Z0 + 7.5
COVER_Z0 = RACK_Z1 + 0.3
COVER_Z1 = COVER_Z0 + 4.0
FOOT_Z0 = COVER_Z1 + 0.3
FOOT_Z1 = FOOT_Z0 + 8.0
FINGER_TIP = FOOT_Z1 + 55.0
TCP_Z = FOOT_Z1 + 35.0
RACK_PITCH_Y = PINION_R + PROFILE_SHIFT * GEAR_M         # 12.15
RACK_ROOT = RACK_PITCH_Y + 1.25 * GEAR_M                 # 14.025
RACK_TIP = RACK_PITCH_Y - GEAR_M                         # 10.65
RACK_BODY = (RACK_ROOT, RACK_ROOT + 7.0)
CHANNEL_HX, CHANNEL_HY = 40.5, RACK_BODY[1] + 0.2
DECK_HX, DECK_HY = 47.0, CHANNEL_HY + 3.5
COVER_SCREWS = [(-44.0, -15.0), (44.0, -15.0), (-44.0, 15.0), (44.0, 15.0)]
POST_X = (5.0, 17.0)
POST_Y = (RACK_BODY[0] + 0.2, RACK_BODY[1] - 0.2)
SLOT_X = (POST_X[0] - 0.3, POST_X[1] + 22.0 + 0.3)
RACK_X = (-30.0, POST_X[1])
FINGER_SCREWS_X = (8.0, 14.0)
PINION_SCREWS = [(5.0, 0.0), (-5.0, 0.0)]       # M2x10 self-tapping screws, into the MG90S horn
MG_X0, MG_X1 = -MG_A, MG_L - MG_A
LOWER = (-14.0, 26.0, -15.0, 15.0)              # gripper lower body (x0, x1, y0, y1)
SKIRT_R = (44.0, 46.0)                           # cable chamber skirt
GRIP_HOLE = (21.0, 0.0)                          # hole where the gripper cable drops into the chamber


def gripper_palm():
    """Gripper body: thin-shell lower body (MG90S seat) flanged to the wrist horn + upper deck
    with rack channels. The cover (gripper_cover) retains the racks."""
    flange = zcyl(16.0, H, H + 4)
    # cable chamber cover: skirt wrapping the top of the wrist housing (cable coils here over ±90° wrist roll)
    skirt = zcyl(SKIRT_R[1], ROLL_Z_EAR + 1.0, H + 4) - zcyl(SKIRT_R[0], ROLL_Z_EAR, H + 1.0)
    x0, x1, y0, y1 = LOWER
    lower = box(x0, x1, y0, y1, H + 4, RACK_Z0 - 3)
    lower = lower.fillet(3.0, lower.edges().filter_by(bd.Axis.Z))
    deck = box(-DECK_HX, DECK_HX, -DECK_HY, DECK_HY, RACK_Z0 - 3, COVER_Z0)
    deck = deck.fillet(3.0, deck.edges().filter_by(bd.Axis.Z))
    body = flange + lower + deck + skirt
    # lightening: windows in the sides of the lower body
    for s in (-1, 1):
        body -= box(x0 + 4, x1 - 4, s * y1 - 3.0 if s > 0 else y0 - 1, s * y1 + 1 if s > 0 else y0 + 3.0,
                    H + 9, RACK_Z0 - 7)
    body -= zcyl(3.2, H - 1, G_FLOOR)                                     # access to the J5 horn screw
    body -= disc_horn_holes(1.4, H - 1, G_FLOOR)                          # 4x M2.5 self-tapping screws
    body -= disc_horn_holes(2.6, G_FLOOR - 2.2, G_FLOOR + 0.1)             # screw head counterbore
    # MG90S pocket + ear step + cable channel/tunnel + DS18B20 pocket
    body -= box(MG_X0 - FIT, MG_X1 + FIT, -MG_W / 2 - FIT, MG_W / 2 + FIT, G_FLOOR, RACK_Z0)
    ex = (MG_EAR_L - MG_L) / 2
    z_ear = G_FLOOR + MG_EAR_Z
    body -= box(MG_X0 - ex - FIT, MG_X1 + ex + FIT, -MG_W / 2 - FIT, MG_W / 2 + FIT, z_ear, RACK_Z0)
    for x in MG_EAR_X:
        body -= zcyl(0.9, z_ear - 6, z_ear + 0.1, x=x)
    cw = MG_CABLE_W / 2 + FIT
    body -= box(MG_X1, MG_X1 + MG_CABLE_T + 0.2, -cw, cw, G_FLOOR, RACK_Z0)
    body -= box(MG_X1 - 0.5, GRIP_HOLE[0] + 3.0, -cw, cw, G_FLOOR - 0.01, G_FLOOR + 3.0)   # floor channel
    body -= zcyl(3.5, H - 1, G_FLOOR + 0.1, x=GRIP_HOLE[0], y=GRIP_HOLE[1])             # drop into chamber
    body -= box(2.5, 8.5, MG_W / 2 + FIT, MG_W / 2 + FIT + TO92[1] + 0.7, G_FLOOR + 3, RACK_Z0)
    body -= box(4.0, 7.0, MG_W / 2 + FIT, LOWER[3] - 2.0, G_FLOOR + 3, RACK_Z0 - 7)   # sensor cable slot
    body -= zcyl(MG_HORN_D / 2 + 1.0, G_SERVO_TOP, RACK_Z0 + 0.1)         # horn clearance
    body -= zcyl(PINION_R + GEAR_M * (1 + PROFILE_SHIFT) + 0.5, G_HORN_TOP - 0.5, RACK_Z0 + 0.1)
    body -= box(-CHANNEL_HX, CHANNEL_HX, -CHANNEL_HY, CHANNEL_HY, RACK_Z0, COVER_Z0 + 1)
    for x, y in COVER_SCREWS:
        body -= zcyl(M3_INSERT, COVER_Z0 - M3_INSERT_DEPTH, COVER_Z0 + 1, x=x, y=y)
    return finish(body, "gripper_palm", *petg(DARK), printed=True)


def gripper_cover():
    c = box(-DECK_HX, DECK_HX, -DECK_HY, DECK_HY, COVER_Z0, COVER_Z1)
    c = c.fillet(3.0, c.edges().filter_by(bd.Axis.Z))
    slot = box(SLOT_X[0], SLOT_X[1], POST_Y[0] - 0.15, POST_Y[1] + 0.15, COVER_Z0 - 1, COVER_Z1 + 1)
    c -= slot
    c -= bd.Rot(0, 0, 180) * slot
    c -= zcyl(8.0, COVER_Z0 - 1, COVER_Z0 + 2.0)                         # pinion screw head pocket
    for x, y in COVER_SCREWS:
        c -= zcyl(M3_CLEAR, COVER_Z0 - 1, COVER_Z1 + 1, x=x, y=y)
        c -= zcyl(M3_HEAD, COVER_Z1 - 1.8, COVER_Z1 + 1, x=x, y=y)        # ISO 7380 button head
    return finish(c, "gripper_cover", *petg(DARK), printed=True)


def _inv(a):
    return math.tan(a) - a


def gripper_pinion():
    """Module 1.5, 16-tooth INVOLUTE pinion with +0.1 profile shift (PETG). Fastened to the MG90S
    round horn with 2x M2 self-tapping screws. At position 0 one tooth points to +Y, one to -Y."""
    m, z_t, x = GEAR_M, PINION_Z, PROFILE_SHIFT
    alpha = math.radians(20)
    rp, rb = PINION_R, PINION_R * math.cos(alpha)
    r_tip, r_root = rp + m * (1 + x), rp - m * (1.25 - x)
    s = math.pi * m / 2 + 2 * x * m * math.tan(alpha) - GEAR_BACKLASH / 2
    psi_p = s / (2 * rp) + _inv(alpha)
    radii = [r_root] + [rb + (r_tip - rb) * k / 6 for k in range(7)]
    prof = []
    for r in radii:
        a_r = math.acos(min(1.0, rb / max(r, rb)))
        prof.append((r, psi_p - _inv(a_r)))
    pts = []
    step = 2 * math.pi / z_t
    for i in range(z_t):
        th = math.pi / 2 + i * step
        for r, ps in prof:
            pts.append((r * math.cos(th - ps), r * math.sin(th - ps)))
        for r, ps in reversed(prof):
            pts.append((r * math.cos(th + ps), r * math.sin(th + ps)))
    gear = bd.Pos(0, 0, G_HORN_TOP) * bd.extrude(bd.Polygon(*pts, align=None), RACK_Z1 - G_HORN_TOP)
    gear -= zcyl(2.6, G_HORN_TOP - 1, RACK_Z1 + 1)                       # access to the horn screw
    for px, py in PINION_SCREWS:
        gear -= zcyl(1.0, G_HORN_TOP - 1, RACK_Z1 + 1, x=px, y=py)
    return finish(gear, "pinion_m15z16", *petg(DARK), printed=True)


def rack(side):
    """Module 1.5 rack (20° straight flanks). side=+1 right (+Y), -1 left (rotated 180°).
    With the finger closed, the tooth space at x=0 meets a pinion tooth."""
    m = GEAR_M
    pitch = math.pi * m
    half_p = (pitch / 2 - GEAR_BACKLASH / 2) / 2
    w_tip = half_p - m * math.tan(math.radians(20))
    w_root = half_p + 1.25 * m * math.tan(math.radians(20))
    body = box(RACK_X[0], RACK_X[1], RACK_BODY[0], RACK_BODY[1], RACK_Z0, RACK_Z1)
    for k in range(math.ceil(RACK_X[0] / pitch) - 1, math.floor(RACK_X[1] / pitch) + 1):
        xc = (k + 0.5) * pitch
        if xc - w_root < RACK_X[0] or xc + w_root > RACK_X[1]:
            continue
        prof = bd.Polygon((xc - w_root, RACK_ROOT + 0.01), (xc - w_tip, RACK_TIP), (xc + w_tip, RACK_TIP),
                          (xc + w_root, RACK_ROOT + 0.01), align=None)
        body += bd.Pos(0, 0, RACK_Z0) * bd.extrude(prof, RACK_Z1 - RACK_Z0)
    yc = sum(RACK_BODY) / 2
    for x in FINGER_SCREWS_X:
        body -= zcyl(M3_PILOT, RACK_Z1 - 6, RACK_Z1 + 1, x=x, y=yc)
    if side < 0:
        body = bd.Rot(0, 0, 180) * body
    return finish(body, "rack_" + ("right" if side > 0 else "left"), *petg(DARK), printed=True)


def finger(side):
    """Finger: foot sliding on the cover with 0.3 mm clearance + 12 mm tongue (2x M3 into the rack)
    + finger + rubber pad. side=+1 right (opens toward +X), -1 left (the right one rotated 180°)."""
    foot = box(3, 20, -(POST_Y[1] + 1.7), POST_Y[1] + 1.7, FOOT_Z0, FOOT_Z1)
    post = box(POST_X[0], POST_X[1], POST_Y[0], POST_Y[1], RACK_Z1, FOOT_Z0 + 0.5)
    blade = box(3, 9, -12, 12, FOOT_Z1 - 0.5, FINGER_TIP)
    tip_edge = blade.edges().filter_by(bd.Axis.Y).group_by(bd.Axis.Z)[-1].sort_by(bd.Axis.X)[-1:]
    blade = blade.chamfer(3.5, None, tip_edge)
    body = foot + post + blade
    yc = sum(RACK_BODY) / 2
    for x in FINGER_SCREWS_X:
        body -= zcyl(M3_CLEAR, RACK_Z1 - 1, FOOT_Z1 + 1, x=x, y=yc)
        body -= zcyl(M3_HEAD, FOOT_Z1 - 5, FOOT_Z1 + 1, x=x, y=yc)   # M3x12, 5 mm into the rack
    pad = box(1, 3, -12, 12, FOOT_Z1 + 15, FINGER_TIP - 2)
    if side < 0:
        body = bd.Rot(0, 0, 180) * body
        pad = bd.Rot(0, 0, 180) * pad
    name = "right" if side > 0 else "left"
    body = finish(body, f"finger_{name}", *petg(DARK), printed=True)
    pad = finish(pad, f"pad_{name}", plastics.rubber(), "#3a3a3a")
    return body, pad


def gripper_sensor():
    y0 = MG_W / 2
    return to92("ds18b20_6", 3.2, 7.8, y0, y0 + TO92[1], G_FLOOR + 8, G_FLOOR + 12.6)
