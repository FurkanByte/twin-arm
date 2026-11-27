"""Control box: servo power distribution + digital-twin sensor electronics (red team rev. 2).

Power path: 12 V PSU -> XT60 (-X wall) -> 15 A blade fuse -> 40 A relay contact (coil latches
itself via E-stop NC + START NO) -> Wago 221 distribution -> 2x XL4016 buck converter (6.0 V) ->
(1000 uF) -> power board: INA226 per servo (R010 shunt) -> GX20-12 aviation connectors (+Y wall).
Signal board: ESP32-S3 Super Mini + 3x ADS1115 (pot differential) -> GX20 pins; DS18B20 1-Wire
-> GX12-4. E-stop and START on the lid, fan on the -X wall, vents above the XL4016s.

Module dimensions are common vendor dimensions; verify against the parts you receive.
"""
import build123d as bd
from bd_materials import plastics
from cad import model

import hardware as HW
import parts as P
from lib.geom import box, xcyl, ycyl, zcyl

IX, IY, IZ = 210.0, 120.0, 70.0     # inner dimensions
W = 3.0                             # wall / floor
Z_FLOOR = W
Z_TOP = Z_FLOOR + IZ
LID_T = 3.0
CORNERS = [(5.0, 5.0), (IX - 5.0, 5.0), (5.0, IY - 5.0), (IX - 5.0, IY - 5.0)]
BOSS_H = 7.0                        # fits the insert (5.75 mm)

# Modules: (x, y, z) dimensions and lower-left corners in inner coordinates
XL4016 = (63.0, 37.0, 30.0)
XL4016_HOLES = (57.0, 31.0)
XL_AT = [(18.0, 8.0), (18.0, 52.0)]
BOARD = (70.0, 50.0, 1.6)            # 5x7 cm perfboard
BOARD_HOLES = (64.0, 44.0)
POWER_AT = (93.0, 8.0)               # power board: 6x INA226
SIGNAL_AT = (140.0, 59.0)            # signal board: ESP32 + 3x ADS1115
INA226 = (22.0, 17.0, 9.0)
ADS1115 = (28.0, 17.0, 9.0)
ESP32 = (22.5, 18.0, 5.0)
RELAY = (38.0, 38.0, 2.0)            # socket plate; 28x28x38 relay body centred on it
RELAY_HOLES = (32.0, 32.0)
RELAY_AT = (170.0, 10.0)
FUSE = (40.0, 14.0, 14.0)
FUSE_AT = (78.0, 100.0)
WAGO = (20.2, 13.3, 18.2)
WAGO_AT = [(25.0, 100.0), (48.0, 100.0)]
CAP_D, CAP_H = 8.0, 16.0
CAPS = [(87.0, 26.0), (87.0, 70.0)]
XT60_AT = (47.0, 20.0)               # -X wall (y, z)
FAN_AT = (88.0, 40.0)                # -X wall (y, z), 40x40
GX = [("gx20_servo_a", 92.0, 10.0), ("gx20_servo_b", 122.0, 10.0), ("gx20_servo_c", 152.0, 10.0),
      ("gx12_1wire", 64.0, 6.0)]   # +Y wall (x, hole radius)
GX_Z = 40.0
ESP_Y = SIGNAL_AT[1] + 26.0
ESTOP_AT = (110.0, 33.0)             # on the lid (x, y)
START_AT = (145.0, 33.0)
SWITCH_DEPTH = 45.0                  # body depth below the lid


def _part(shape, label, material, color):
    shape.label, shape.material, shape.color = label, material, color
    return shape


def _bosses(at, size, holes):
    out = None
    x0, y0 = at
    for dx in (-1, 1):
        for dy in (-1, 1):
            x = x0 + size[0] / 2 + dx * holes[0] / 2
            y = y0 + size[1] / 2 + dy * holes[1] / 2
            b = zcyl(3.5, Z_FLOOR, Z_FLOOR + BOSS_H, x=x, y=y) - zcyl(P.M3_INSERT, Z_FLOOR, Z_FLOOR + BOSS_H + 1, x=x, y=y)
            out = b if out is None else out + b
    return out


def box_body():
    outer = box(-W, IX + W, -W, IY + W, 0, Z_TOP)
    outer = outer.fillet(4.0, outer.edges().filter_by(bd.Axis.Z))
    body = outer - box(0, IX, 0, IY, Z_FLOOR, Z_TOP + 1)
    for x, y in CORNERS:                                           # lid inserts (against the wall)
        post = box(x - 5, x + 5, y - 5, y + 5, Z_FLOOR, Z_TOP)
        body += post - zcyl(P.M3_INSERT, Z_TOP - 8, Z_TOP + 1, x=x, y=y)
    for at in XL_AT:
        body += _bosses(at, XL4016, XL4016_HOLES)
    for at in (POWER_AT, SIGNAL_AT):
        body += _bosses(at, BOARD, BOARD_HOLES)
    body += _bosses(RELAY_AT, RELAY[:2], RELAY_HOLES)
    y, z = XT60_AT
    body -= box(-W - 1, 1, y - 8.3, y + 8.3, z - 4.3, z + 4.3)      # XT60 panel cutout
    for dy in (-12.5, 12.5):
        body -= xcyl(1.6, -W - 1, 1, y=y + dy, z=z)
    y, z = FAN_AT
    body -= xcyl(19.0, -W - 1, 1, y=y, z=z)                        # 40 mm fan opening
    for dy in (-16, 16):
        for dz in (-16, 16):
            body -= xcyl(1.7, -W - 1, 1, y=y + dy, z=z + dz)
    for _, x, r in GX:                                             # aviation connectors
        body -= ycyl(r, IY - 1, IY + W + 1, x=x, z=GX_Z)
    zb = Z_FLOOR + BOSS_H + BOARD[2]                               # ESP32 USB-C opening (+X wall)
    body -= box(IX - 1, IX + W + 1, ESP_Y + ESP32[1] / 2 - 6, ESP_Y + ESP32[1] / 2 + 6, zb, zb + 9)
    for i in range(6):                                             # bottom vents (under the XL4016s)
        body -= box(24 + 9 * i, 28 + 9 * i, 14, 40, -1, Z_FLOOR + 1)
        body -= box(24 + 9 * i, 28 + 9 * i, 58, 84, -1, Z_FLOOR + 1)
    for x, y in [(15, 15), (IX - 15, 15), (15, IY - 15), (IX - 15, IY - 15)]:   # rubber foot recess
        body -= zcyl(5.0, -1, 1.0, x=x, y=y)
    return _part(body, "control_box", *P.petg(P.DARK))


def lid():
    l = box(-W, IX + W, -W, IY + W, Z_TOP, Z_TOP + LID_T)
    l = l.fillet(4.0, l.edges().filter_by(bd.Axis.Z))
    for x, y in CORNERS:
        l -= zcyl(P.M3_CLEAR, Z_TOP - 1, Z_TOP + LID_T + 1, x=x, y=y)
        l -= zcyl(P.M3_HEAD, Z_TOP + LID_T - 1.8, Z_TOP + LID_T + 1, x=x, y=y)
    for (x0, y0) in XL_AT:                                         # vents above the XL4016s
        for i in range(7):
            x = x0 + 6 + 8 * i
            l -= box(x, x + 4, y0 + 4, y0 + XL4016[1] - 4, Z_TOP - 1, Z_TOP + LID_T + 1)
    for x, y in (ESTOP_AT, START_AT):                              # 22 mm button holes
        l -= zcyl(11.2, Z_TOP - 1, Z_TOP + LID_T + 1, x=x, y=y)
    return _part(l, "control_box_lid", *P.petg(P.ACCENT))


def _module(label, size, at, color, z0):
    x, y = at
    return _part(box(x, x + size[0], y, y + size[1], z0, z0 + size[2]), label, plastics.fr4(), color)


def _pcb_module(label, size, at, color, holes, body_inset, body_h, z0):
    """PCB with holes + component body on top that keeps the holes clear + 4x M3x6 screws (into inserts)."""
    x, y = at
    pcb = box(x, x + size[0], y, y + size[1], z0, z0 + 1.6)
    cx, cy = x + size[0] / 2, y + size[1] / 2
    pts = [(cx + dx * holes[0] / 2, cy + dy * holes[1] / 2) for dx in (-1, 1) for dy in (-1, 1)]
    for hx, hy in pts:
        pcb -= zcyl(1.7, z0 - 1, z0 + 3, x=hx, y=hy)
    parts = [_part(pcb, label + "_pcb", plastics.fr4(), "#1f6f3a")]
    if body_h > 0:
        ix, iy = body_inset
        comp = box(x + ix, x + size[0] - ix, y + iy, y + size[1] - iy, z0 + 1.6, z0 + 1.6 + body_h)
        parts.append(_part(comp, label, plastics.abs_(), color))
    for hx, hy in pts:
        parts.append(HW.insert("M3", (hx, hy, z0), (0, 0, -1)))
        parts.append(HW.screw("M3", 6, (hx, hy, z0 + 1.6), (0, 0, -1)))
    return parts


def electronics():
    parts = []
    zboss = Z_FLOOR + BOSS_H
    for i, at in enumerate(XL_AT, 1):
        parts += _pcb_module(f"xl4016_{i}", XL4016, at, "#1b4f9c", XL4016_HOLES, (7.0, 7.0), XL4016[2] - 1.6, zboss)
    parts += _pcb_module("power_board", BOARD, POWER_AT, "#c9a46a", BOARD_HOLES, (0, 0), 0.0, zboss)
    parts += _pcb_module("signal_board", BOARD, SIGNAL_AT, "#c9a46a", BOARD_HOLES, (0, 0), 0.0, zboss)
    zc = zboss + BOARD[2]
    px, py = POWER_AT
    for i in range(6):                                             # 3 x 2 INA226
        col, row = i % 3, i // 3
        parts.append(_module(f"ina226_{i + 1}", INA226, (px + 1.5 + col * 23.0, py + 6 + row * 21.0), "#6a2c91", zc))
    sx, sy = SIGNAL_AT
    for i in range(3):
        parts.append(_module(f"ads1115_{i + 1}", ADS1115, (sx + 7 + (i % 2) * 31.0, sy + 7 + (i // 2) * 22.0), "#6a2c91", zc))
    parts.append(_module("esp32_s3_mini", ESP32, (sx + BOARD[0] - ESP32[0], ESP_Y), "#222222", zc))
    rel = _pcb_module("relay_40a", RELAY, RELAY_AT, "#111111", RELAY_HOLES, (5.0, 5.0), 38.0, zboss)
    rel[0].label = "relay_socket"
    rel[0].color = "#222222"
    parts += rel
    parts.append(_module("fuse_holder_15a", FUSE, FUSE_AT, "#d9a400", Z_FLOOR))
    for i, at in enumerate(WAGO_AT, 1):
        parts.append(_module(f"wago_221_{i}", WAGO, at, "#ff8c1a", Z_FLOOR))
    for i, (x, y) in enumerate(CAPS, 1):
        parts.append(_part(zcyl(CAP_D / 2, Z_FLOOR, Z_FLOOR + CAP_H, x=x, y=y), f"cap_1000uF_{i}", plastics.abs_(), "#203a8f"))
    for name, (x, y) in (("estop_np2_be102", ESTOP_AT), ("start_np2_ba31", START_AT)):
        col = "#d01f1f" if name.startswith("estop") else "#1fa03c"
        head_r = 20.0 if name.startswith("estop") else 12.0
        sw = (zcyl(11.0, Z_TOP, Z_TOP + LID_T, x=x, y=y)
              + zcyl(head_r, Z_TOP + LID_T, Z_TOP + LID_T + 12, x=x, y=y)
              + box(x - 15, x + 15, y - 15, y + 15, Z_TOP - SWITCH_DEPTH, Z_TOP))
        parts.append(_part(sw, name, plastics.abs_(), col))
    y, z = XT60_AT
    parts.append(_part(box(-W - 1, 14, y - 8, y + 8, z - 4, z + 4), "xt60_panel", plastics.nylon(), "#e5c100"))
    for dy in (-12.5, 12.5):
        parts.append(HW.screw("M3", 10, (-W, y + dy, z), (1, 0, 0)))
        parts.append(HW.nut("M3", (0, y + dy, z), (-1, 0, 0)))
    y, z = FAN_AT
    fan = box(0, 10, y - 20, y + 20, z - 20, z + 20) - xcyl(18.5, -1, 11, y=y, z=z)
    fan += xcyl(8.0, 0, 10, y=y, z=z)
    for dy in (-16, 16):
        for dz in (-16, 16):
            fan -= xcyl(1.7, -1, 11, y=y + dy, z=z + dz)
    parts.append(_part(fan, "fan_40mm", plastics.abs_(), "#111111"))
    for dy in (-16, 16):
        for dz in (-16, 16):
            parts.append(HW.screw("M3", 16, (-W, y + dy, z + dz), (1, 0, 0)))
            parts.append(HW.nut("M3", (10, y + dy, z + dz), (-1, 0, 0)))
    for name, x, r in GX:
        g = ycyl(r - 0.2, IY, IY + W, x=x, z=GX_Z) + ycyl(r + 3, IY + W, IY + W + 18, x=x, z=GX_Z)             + ycyl(r - 1.5, IY - 25, IY, x=x, z=GX_Z)
        parts.append(_part(g, name, plastics.nylon(), "#9aa0a6"))
    for x, y in CORNERS:                                           # lid: insert + button-head M3x8
        parts.append(HW.insert("M3", (x, y, Z_TOP), (0, 0, -1)))
        parts.append(HW.screw("M3", 8, (x, y, Z_TOP + LID_T - 1.8), (0, 0, -1), head="ISO7380"))
    for x, y in [(15, 15), (IX - 15, 15), (15, IY - 15), (IX - 15, IY - 15)]:
        parts.append(HW.rubber_foot((x, y, -0.5)))
    return parts


@model
def control_box():
    kids = [box_body(), lid()] + electronics()
    return bd.Compound(children=kids, label="control_box_assembly")


@model
def control_box_print_body():
    return box_body()


@model
def control_box_print_lid():
    l = lid()
    flipped = bd.Pos(0, 0, Z_TOP + LID_T) * (bd.Rot(180, 0, 0) * l)
    flipped.label, flipped.material, flipped.color = l.label, l.material, l.color
    return flipped


if __name__ == "__main__":
    control_box()
    control_box_print_body()
    control_box_print_lid()
