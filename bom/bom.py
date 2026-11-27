"""Full bill of materials: counts every part in the CAD assemblies (arm + control box), matches
them against the price catalog, adds consumables not visible in CAD, and computes the total cost.

    cad run bom/bom.py      ->  bom/BOM.md, bom/BOM.csv

Prices in TL, VAT included. "status": "in stock" = stock was seen on the product page on 2026-11-13;
"verify" = the price is an estimate or stock could not be checked.
"""
import csv
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "cad"))

import build123d  # noqa: F401,E402
from cad.build123d import leaves  # noqa: E402

from arm import twinarm  # noqa: E402
from control_box import control_box  # noqa: E402
from parts import PETG_DENSITY, PRINT_FILL  # noqa: E402

KDV = 1.20
M, D, R, RS, T = "Motorobit", "Direnc.net", "Robolink", "Robotistan", "hardware store / local"

# key: (group, item, supplier, url, pack quantity, pack price incl. VAT, status)
CAT = {
    # --- servos -------------------------------------------------------------------------
    "servo:RDS3235": ("Servos", "RDS3235 35 kg·cm digital servo (shoulder)", M, "https://www.motorobit.com/rds3235-35kg-dijital-servo-motor-180-u-servo-braket-dahil", 1, 2214.50 * KDV, "in stock"),
    "servo:DS3230MG": ("Servos", "DS3230MG 30 kg·cm (elbow)", M, "https://motorobit.com/ds3230mg-30kg-su-gecirmez-dijital-servo-motor", 1, 1648.00 * KDV, "in stock"),
    "servo:DS3218MG": ("Servos", "DS3218MG 20 kg·cm (base, wrist pitch, wrist roll)", M, "https://www.motorobit.com/ds3218mg-20kg-su-gecirmez-dijital-servo-motor", 1, 1107.25 * KDV, "in stock"),
    "servo:MG90S": ("Servos", "Tower Pro MG90S (gripper; horn + screws in the box)", M, "https://www.motorobit.com/tower-pro-mg90s-rc-servo-motor", 1, 128.75 * KDV, "in stock"),
    "horn:alu": ("Servos", "25T aluminum single-arm servo horn (holes at 20/24 mm)", M, "motorobit.com — '25T metal servo horn'", 1, 120.0, "verify"),
    "horn:included": ("Servos", "Disc horn + horn screw (from the servo box)", "with servo", "", 1, 0.0, "included"),
    # --- bearings -------------------------------------------------------------------------
    "bearing_51107": ("Bearings", "51107 thrust bearing 35×52×12", "rulmanbayisi.com", "https://www.rulmanbayisi.com/i/csk-51107", 1, 58.0, "verify (by phone)"),
    "bearing_6006zz": ("Bearings", "6006ZZ radial bearing 30×55×13", T, "local bearing shop", 1, 120.0, "verify"),
    "bearing_6804": ("Bearings", "6804-2RS thin-section bearing 20×32×7 (hollow shaft, cable pass-through; pack of 5)", "akakce.com (KWC 6804 2RS pack of 5)", "https://www.akakce.com/arama/?q=6804+2rs+rulman", 5, 298.90, "verify"),
    # --- fasteners ------------------------------------------------------------------------
    "insert_M3x5.75": ("Fasteners", "M3×5.75 brass heat-set insert (pack of 10)", R, "https://www.robolinkmarket.com/m3x575mm-pirinc-insert-10-adet", 10, 72.0, "in stock"),
    "screw_M3x8_DIN912": ("Fasteners", "M3×8 socket head DIN912 (set of 10)", R, "https://www.robolinkmarket.com/m3x8-imbus-vida-somun-ve-pul-seti-10lu-paket", 10, 63.0, "in stock"),
    "horn_screw_M3x8": ("Fasteners", "M3×8 horn center screw (DIN912; if not supplied with the horn)", R, "same set", 10, 63.0, "in stock"),
    "screw_M3x6_DIN912": ("Fasteners", "M3×6 socket head DIN912", T, "", 10, 40.0, "verify"),
    "screw_M3x10_DIN912": ("Fasteners", "M3×10 socket head DIN912", T, "", 10, 45.0, "verify"),
    "screw_M3x20_DIN912": ("Fasteners", "M3×20 socket head DIN912 (idler/cable cover → bridge)", R, "robolinkmarket.com M3×20", 10, 70.0, "verify"),
    "selftap_M2x6_PAN": ("Fasteners", "M2×6 self-tapping screw (shaft-end retainer washer)", T, "", 10, 25.0, "verify"),
    "screw_M3x12_DIN912": ("Fasteners", "M3×12 socket head DIN912 (finger → rack)", R, "robolinkmarket.com M3×12", 10, 60.0, "in stock"),
    "screw_M3x16_DIN912": ("Fasteners", "M3×16 socket head DIN912 (fan)", T, "", 10, 50.0, "verify"),
    "screw_M3x8_ISO7380": ("Fasteners", "M3×8 button head ISO7380 (covers)", T, "", 10, 50.0, "verify"),
    "selftap_M3x8_PAN": ("Fasteners", "M3×8 self-tapping screw for plastic (servo far ear)", T, "", 10, 30.0, "verify"),
    "selftap_M2.5x8_PAN": ("Fasteners", "M2.5×8 self-tapping screw (gripper flange → J5 horn)", T, "", 10, 25.0, "verify"),
    "selftap_M2x10_PAN": ("Fasteners", "M2×10 self-tapping screw (pinion → MG90S horn)", T, "", 10, 25.0, "verify"),
    "selftap_M2x8_PAN": ("Fasteners", "M2×8 self-tapping screw (MG90S ears; in the servo box)", "with servo", "", 1, 0.0, "included"),
    "horn_screw_M2x4": ("Fasteners", "MG90S horn screw (in the servo box)", "with servo", "", 1, 0.0, "included"),
    "setscrew_M3x10": ("Fasteners", "M3×10 set screw (turntable torque pin)", T, "", 10, 40.0, "verify"),
    "setscrew_M3x4": ("Fasteners", "M3×4 set screw (hub collar)", T, "", 10, 30.0, "verify"),
    "screw_M5x16_table": ("Fasteners", "M5×16 bolt/wood screw (table mounting)", R, "robolinkmarket.com M5", 10, 48.0, "in stock"),
    "nut_M3": ("Fasteners", "M3 nut (fan, XT60)", R, "from the M3 screw set", 10, 15.0, "in stock"),
    "rubber_foot_10x3": ("Fasteners", "Ø10×3 adhesive rubber foot (box)", T, "", 4, 30.0, "verify"),
    "pad": ("Mechanical", "Rubber pad (cut 24×38 from a 2 mm sheet)", T, "", 2, 50.0, "verify"),
    # --- sensors and electronics ----------------------------------------------------------
    "ds18b20": ("Electronics", "DS18B20 TO-92 temperature sensor", D, "https://www.direnc.net/ds18b20-sicaklik-sensor-entegresi-to-92", 1, 29.28, "orderable"),
    "xl4016": ("Electronics", "XL4016 300 W (8 A) adjustable step-down", D, "https://www.direnc.net/dc-dc-step-down-modul-xl4016-300w", 1, 169.38, "orderable"),
    "ina226": ("Electronics", "INA226 bidirectional current/power sensor module", D, "https://www.direnc.net/ina226-iki-yonlu-akim-guc-izleme-sensor-modulu", 1, 108.89, "orderable"),
    "ads1115": ("Electronics", "ADS1115 16-bit 4-channel ADC", D, "https://www.direnc.net/ads1115-16-bit-i2c-4-kanal-modul", 1, 145.18, "orderable"),
    "esp32_s3_mini": ("Electronics", "ESP32-S3 Super Mini", D, "https://www.direnc.net/esp32-s3-wifi-bluetooth-board", 1, 349.34, "orderable"),
    "board_pcb": ("Electronics", "Double-sided perfboard 7×9 (cut to 5×7)", M, "https://www.motorobit.com/7x9cm-epoxy-double-sided-perforated-plaque", 1, 46.36, "in stock"),
    "relay": ("Electronics", "12 V 40 A 5-pin automotive relay + socket", T, "auto electrician", 1, 150.0, "verify"),
    "fuse_holder_15a": ("Electronics", "ATO blade fuse holder + 15 A fuse", T, "auto electrician", 1, 60.0, "verify"),
    "wago_221": ("Electronics", "Wago 221-415 (5-way) connector", T, "electrical supply store", 1, 25.0, "verify"),
    "cap_1000uF": ("Electronics", "1000 µF 16 V low-ESR electrolytic", D, "direnc.net", 1, 10.0, "verify"),
    "estop_np2_be102": ("Electronics", "CHINT NP2-BE102 emergency stop (NC)", M, "https://www.motorobit.com/np2-be102-nc-metal-acil-stop-butonu", 1, 257.50 * KDV, "in stock"),
    "start_np2_ba31": ("Electronics", "NP2-BA31 green START button (NO)", T, "electrical supply store", 1, 250.0, "verify"),
    "xt60_panel": ("Electronics", "XT60 male-female (panel)", D, "https://www.direnc.net/xt60-plug-60a-li-po-konnektor", 1, 18.87, "in stock"),
    "fan_40mm": ("Electronics", "40×40×10 mm 12 V fan", T, "", 1, 60.0, "verify"),
    "gx20": ("Electronics", "GX20 12-pin aviation connector (panel + plug)", T, "electronics supply store", 1, 100.0, "verify"),
    "gx12_1wire": ("Electronics", "GX12 4-pin aviation connector (panel + plug)", T, "electronics supply store", 1, 60.0, "verify"),
}

# Consumables and external parts not modeled in CAD: (group, item, supplier, url, quantity, unit price, status)
EXTRA = [
    ("Electronics", "12 V 10 A metal-case power supply (HT-1115)", M, "https://www.motorobit.com/hightek-12v-10a-metal-kasa-adaptor-led-driver", 1, 463.50 * KDV, "in stock"),
    ("Electronics", "Grounded mains cable with plug + PSU terminal cover", T, "", 1, 80.0, "verify"),
    ("Electronics", "R010 (0.01 Ω) 2512 shunt resistor (for the INA226s)", D, "https://www.direnc.net/10mr-2515-sont-direnc", 6, 39.63, "in stock"),
    ("Electronics", "Resistor set: 10 k ×18, 330 Ω ×6, 4.7 k, 100 Ω, 100 k, 22 k", D, "", 1, 60.0, "verify"),
    ("Electronics", "100 nF ceramic capacitor ×12, 3.3 V zener, 1N4007 ×2, BC547", D, "", 1, 50.0, "verify"),
    ("Electronics", "JST-XH connector set (230 pieces)", R, "https://www.robolinkmarket.com/jst-xh-xhp-254mm-tunik-konnektor-seti", 1, 186.0, "in stock"),
    ("Electronics", "Female/male pin header strip (40-pin) ×4", D, "", 4, 10.0, "verify"),
    ("Cables", "Silicone wire 20 AWG red + black (servo power), 7 m + 7 m (internal harness runs: 0.78–1.40 m per servo, 6.5 m total)", M, "motorobit.com silicone wire", 14, 35.0, "verify"),
    ("Cables", "Silicone wire 26 AWG (PWM ×6.5 m, 1-Wire 3 wires, I²C inside the box), 5 m spool ×6", M, "", 6, 60.0, "verify"),
    ("Cables", "30 AWG wire-wrap wire (pot leads), 1 spool", D, "", 1, 90.0, "verify"),
    ("Cables", "Silicone wire 1.5 mm² (12 V input), 2 m", T, "", 2, 30.0, "verify"),
    ("Cables", "Braided cable sleeve (PET) 6 mm, 3 m (harness inside the arm) + 10 mm, 2 m (arm → box)", T, "", 1, 90.0, "verify"),
    ("Cables", "Zip ties 2.5 mm (pack of 100) + Kapton tape (chafe protection at shaft/chamber pass-throughs)", R, "", 1, 110.0, "verify"),
    ("Cables", "Heat-shrink tubing set", D, "", 1, 60.0, "verify"),
    ("Cables", "USB-C data cable 1.5 m + ferrite", T, "", 1, 80.0, "verify"),
    ("Consumables", "Thermal adhesive 5 g (DS18B20)", D, "https://www.direnc.net/termal-slikon-yapistirici-sogutucu-5gr", 1, 45.37, "in stock"),
    ("Consumables", "Solder wire 0.8 mm 100 g + flux", D, "", 1, 120.0, "verify"),
    ("Consumables", "Medium-strength threadlocker (horn screws)", T, "", 1, 80.0, "verify"),
    ("Consumables", "Double-sided foam tape (Wago, fuse holder)", T, "", 1, 40.0, "verify"),
]

PRINT_COLORS = {"#2b2f36": "dark anthracite", "#ff6a13": "orange"}
SPOOL = ("Filament", "PETG 1.75 mm 1 kg spool", "porima3d.com (Eco PETG) / Microzey", "https://www.porima3d.com", 1, 708.48, "in stock")
WASTE = 1.20                      # allowance for supports + brim + test prints

SERVO_MAP = {"servo_1_base": "DS3218MG", "servo_2_shoulder": "RDS3235", "servo_3_elbow": "DS3230MG",
             "servo_4_wrist_pitch": "DS3218MG", "servo_5_wrist_roll": "DS3218MG", "servo_6_gripper": "MG90S"}


# in the assembly the colors are switched to material colors, so the print color comes from the part source (parts.py)
PART_COLOR = {k: "orange" for k in ("upper_arm", "forearm", "idler_plate_shoulder", "idler_plate_elbow",
                                     "control_box_lid")}          # every other print: dark anthracite


def color_name(c):
    """build123d Color -> PRINT_COLORS name (nearest)."""
    try:
        r, g, b = tuple(c)[:3]
    except TypeError:
        r, g, b = c.to_tuple()[:3]
    best, dist = str(c), 1e9
    for hx, name in PRINT_COLORS.items():
        q = [int(hx[i:i + 2], 16) / 255 for i in (1, 3, 5)]
        d = (q[0] - r) ** 2 + (q[1] - g) ** 2 + (q[2] - b) ** 2
        if d < dist:
            best, dist = name, d
    return best


def tr(x, nd=2):
    """English number format: 1,234.56"""
    return f"{x:,.{nd}f}"


def key_for(path, leaf):
    lab = leaf.label
    if lab == "body" and len(path) >= 2 and path[-2] in SERVO_MAP:
        return "servo:" + SERVO_MAP[path[-2]]
    if lab == "horn":
        return "horn:alu" if "Alu" in str(leaf.material) or "alu" in str(leaf.material).lower() else "horn:included"
    for prefix in ("ds18b20", "ina226", "ads1115", "wago_221", "cap_1000uF", "gx20", "pad"):
        if lab.startswith(prefix):
            return prefix
    if lab.startswith("xl4016") and lab.endswith("_pcb"):
        return "xl4016"
    if lab.startswith("xl4016"):
        return None                                   # component body (counted with the pcb)
    if lab in ("power_board_pcb", "signal_board_pcb"):
        return "board_pcb"
    if lab == "relay_socket":
        return "relay"
    if lab == "relay_40a" or lab.startswith("harness"):
        return None                                   # harness: cable items are in EXTRA (from length)
    return lab


if __name__ == "__main__":
    counts, printed = Counter(), defaultdict(lambda: [0, 0.0, ""])
    unknown = Counter()
    for tree in (twinarm().shape, control_box().shape):
        for path, leaf in leaves(tree, paths=True):
            mat = str(leaf.material).lower()
            if "petg" in mat:
                p = printed[leaf.label]
                p[0] += 1
                p[1] += leaf.volume * PETG_DENSITY * PRINT_FILL
                p[2] = PART_COLOR.get(leaf.label, "dark anthracite")
                continue
            k = key_for(path, leaf)
            if k is None:
                continue
            if k in CAT:
                counts[k] += 1
            else:
                unknown[k] += 1

    rows = []
    for k, n in sorted(counts.items(), key=lambda kv: (CAT[kv[0]][0], kv[0])):
        grp, item, sup, url, pack, price, st = CAT[k]
        packs = math.ceil(n / pack)
        rows.append((grp, item, n, f"{packs} × pack of {pack}" if pack > 1 else str(n), sup, url, price, packs * price, st))
    for grp, item, sup, url, qty, unit, st in EXTRA:
        rows.append((grp, item, qty, str(qty), sup, url, unit, qty * unit, st))
    grams = defaultdict(float)
    for lab, (n, g, col) in printed.items():
        grams[col] += g
    for col, g in grams.items():
        spools = math.ceil(g * WASTE / 1000)
        rows.append(("Filament", f"{SPOOL[1]} — {col} (~{g * WASTE:.0f} g)", spools, str(spools), SPOOL[2], SPOOL[3], SPOOL[5], spools * SPOOL[5], SPOOL[6]))

    total = sum(r[7] for r in rows)
    verified = sum(r[7] for r in rows if r[8] in ("in stock", "included", "orderable"))
    by_group = defaultdict(float)
    for r in rows:
        by_group[r[0]] += r[7]

    out_csv = ROOT / "bom" / "BOM.csv"
    with open(out_csv, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f, delimiter=";")
        w.writerow(["Group", "Item", "CAD count/quantity", "To buy", "Supplier", "URL", "Unit/pack TL (VAT incl.)", "Total TL", "Status"])
        for r in rows:
            w.writerow([r[0], r[1], r[2], r[3], r[4], r[5], f"{r[6]:.2f}", f"{r[7]:.2f}", r[8]])
        w.writerow([])
        w.writerow(["", "TOTAL", "", "", "", "", "", f"{total:.2f}", ""])

    md = ["# TwinArm-5: Full Bill of Materials (BOM)", "",
          "This list is generated by `bom/bom.py` by **counting automatically from the CAD assemblies** "
          "(arm + control box: every screw, insert, nut, bearing, sensor and module). Cables, solder and similar "
          "consumables that are not drawn in CAD are added separately. Prices are in TL, VAT included. **in stock**: "
          "stock was seen on the product page on 2026-11-13. **verify**: the price is an estimate or stock could not "
          "be checked. **orderable**: adding to cart was enabled on the product page on 2026-11-17 (no stock count shown). "
          "**included**: comes with the servo.", ""]
    for grp in ["Servos", "Bearings", "Fasteners", "Mechanical", "Electronics", "Cables", "Consumables", "Filament"]:
        sel = [r for r in rows if r[0] == grp]
        if not sel:
            continue
        md += [f"## {grp} — {tr(by_group[grp], 0)} TL", "",
               "| Item | In CAD | To buy | Supplier | Unit/pack | Total | Status |",
               "|---|---|---|---|---|---|---|"]
        for r in sel:
            sup = f"[{r[4]}]({r[5]})" if str(r[5]).startswith("http") else r[4]
            md.append(f"| {r[1]} | {r[2]} | {r[3]} | {sup} | {tr(r[6])} | {tr(r[7])} | {r[8]} |")
        md.append("")
    md += ["## Printed parts (PETG, from CAD)", "", "| Part | Qty | Estimated filament (g) | Color |", "|---|---|---|---|"]
    for lab, (n, g, col) in sorted(printed.items()):
        md.append(f"| {lab} | {n} | {g:.0f} | {col} |")
    md += ["", f"Total filament (including supports and waste, ×{tr(WASTE, 1)}): **~{tr(sum(grams.values()) * WASTE / 1000)} kg**", ""]
    md += ["## Totals", "", "| | TL (VAT incl.) |", "|---|---|"]
    for grp, v in by_group.items():
        md.append(f"| {grp} | {tr(v, 0)} |")
    md += [f"| **Grand total** | **{tr(total, 0)}** |",
           f"| Portion with verified stock and price | {tr(verified, 0)} |",
           f"| Estimated-price portion (verify) | {tr(total - verified, 0)} |",
           f"| Suggested budget with 10% spare/shipping margin | **{tr(total * 1.10, 0)}** |", ""]
    if unknown:
        md += ["CAD parts with no catalog match: " + ", ".join(f"{k} ×{v}" for k, v in unknown.items()), ""]
    (ROOT / "bom" / "BOM.md").write_text("\n".join(md), encoding="utf-8")
    print(f"rows {len(rows)}, total {total:,.0f} TL, verified {verified:,.0f} TL")
    print("not in catalog:", dict(unknown))
    for k, n in sorted(counts.items()):
        print(f"  {n:3d}  {k}")
