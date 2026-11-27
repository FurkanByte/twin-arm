"""Shared arm dimensions (mm). All parts are drawn in the candle pose (all joints 0, arm upright):
+Z up, +X forward (the direction the arm bends), +Y along the horizontal pitch axes.

Servos (in stock at Motorobit, 2026-11-13):
  RDS3235   35 kg.cm (32 @6 V)            shoulder
  DS3230MG  30 kg.cm (~27.5 @6 V), 67 g    elbow
  DS3218MG  20 kg.cm (~20 @6 V), 60 g      base, wrist pitch, wrist roll
  MG90S     2.2 kg.cm @6 V, 13.4 g          gripper (~9 N per finger: enough to grip,
                                             won't break the printed parts)
Standard-size body 40 x 20 x 40.5 mm, with ears (2 holes per ear, 49.5 x 10 pattern), 25T spline.
Ear height, hole pattern, spline position and cable exit are generic dimensions:
MUST BE VERIFIED WITH CALIPERS once the servos arrive.
"""

# --- Standard-size servo ---------------------------------------------------------------
SERVO_L = 40.0          # long side
SERVO_W = 20.0          # short side
SERVO_H = 40.5          # body height along the output shaft
AXIS_FROM_END = 10.0    # output shaft to the near end
EAR_L = 54.5            # length including ears
EAR_T = 2.5             # ear thickness
EAR_Z = 28.0            # body bottom to ear underside
EAR_HOLES = 49.5        # ear hole spacing (long axis)
EAR_HOLE_W = 10.0       # spacing of the two holes per ear (short axis)
CABLE_W, CABLE_T = 7.0, 2.0     # flat servo cable exit (on the far end face)
CABLE_Z = 3.0           # cable exit height above the body bottom
# Pitch joints (shoulder, elbow, wrist pitch): 25T aluminum SINGLE-ARM horn (Motorobit stock),
# screwed along its arm to the next link's cheek with 2x M3 + center screw. Base and wrist roll:
# the disc horn supplied with the servo (they carry no gravity torque).
HORN_D = 25.0           # disc horn diameter (base/wrist roll)
HORN_T = 3.5
HORN_GAP = 2.0          # body top -> horn underside (shaft boss)
HORN_BOLT_R = 7.5       # disc horn screw circle radius - verify against the actual horn
ARM_HORN_HUB_D = 14.0   # arm horn hub diameter
ARM_HORN_W = 9.0        # arm width
ARM_HORN_L = 28.0       # shaft to arm tip
ARM_HORN_HOLES = (20.0, 24.0)   # hole distances from the shaft (Motorobit 25T arm) - verify
ARM_HORN_SCREW = 24.0           # single M3x8 into the cheek (holes are 4 mm apart: two heads won't fit;
                                # axis held by servo shaft + idler bearing, torque goes through one screw)
FIT = 0.3               # FDM fit clearance

# --- MG90S (gripper) -------------------------------------------------------------------
MG_L, MG_W, MG_H = 22.8, 12.2, 22.7
MG_A = 5.9              # shaft to near end
MG_EAR_L, MG_EAR_T, MG_EAR_Z = 32.3, 2.5, 15.9
MG_EAR_HOLES = 27.8     # one hole per ear (M2 self-tapping screw)
MG_HORN_GAP, MG_HORN_T, MG_HORN_D = 1.5, 2.5, 20.0   # round plastic horn (supplied with the servo)
MG_CABLE_W, MG_CABLE_T = 4.0, 1.2

# --- Temperature sensor: DS18B20 TO-92, thermal-glued to the servo body, single 1-Wire bus ---
TO92 = (4.6, 3.6, 4.6)  # width, thickness (normal to the body), height

# --- Idler bearing (6804-2RS thin section: 20 x 32 x 7) - HOLLOW STUB SHAFT, cables through the joint axis --
IDLER_BRG_D, IDLER_BRG_d, IDLER_BRG_W = 32.0, 20.0, 7.0
IDLER_LIP = 0.8                                  # lip retaining the bearing
STUB_R_OUT, STUB_R_IN = 10.0, 6.5                # hollow stub shaft: Ø20 outer, Ø13 cable bore
PLATE_CAP = 12.5                                 # idler cover cable chamber thickness
DUCT_HX = 6.5                                    # internal cable duct half-width (x), depth 9 (y)

# --- Y placement of the pitch joints ----------------------------------------------------
# The servo ears seat on the block's +Y face (Y_EAR) and are screwed there. Y_EAR is chosen so
# the horn outer face and the idler boss outer face are symmetric about the axis.
_WALL, _BOSS = 12.0, 1.5          # wall behind the servo (holds a 9 mm cable duct), shaft shoulder
Y_EAR = (EAR_Z + FIT + _WALL + _BOSS - (SERVO_H - EAR_Z + HORN_GAP + HORN_T)) / 2   # 10.65
SERVO_Y0 = Y_EAR - EAR_Z                                 # body bottom
SERVO_Y1 = SERVO_Y0 + SERVO_H                            # body top
HORN_Y0 = SERVO_Y1 + HORN_GAP
HORN_Y1 = HORN_Y0 + HORN_T                               # horn outer face (28.65)
BLOCK_Y0, BLOCK_Y1 = SERVO_Y0 - FIT - _WALL, Y_EAR        # servo block
IDLER_Y0 = BLOCK_Y0 - _BOSS                              # idler boss outer face (= -HORN_Y1)
CHEEK_T = IDLER_BRG_W + IDLER_LIP                        # cheek thickness
CLEVIS_HALF = HORN_Y1 + CHEEK_T                          # clevis outer half-width (34.45)
BEAM_HALF_Y = -BLOCK_Y0                                  # beam half-width (27.15)
LINK_HALF_X = 19.0                                       # X half-width of the arm cross-section
CHEEK_R = LINK_HALF_X                                    # clevis end radius
BRIDGE_GAP = 28.0                                        # axis to clevis bridge
BRIDGE_T = 12.0                   # bridge (cable duct runs through it)

# --- Base: J1 servo -> pinned coupling -> turntable (6006ZZ radial + 51107 thrust) ------
PLATE_T = 9.0          # base bottom plate (J1 servo sits in a 6 mm pocket)
J1_POCKET = 6.0
Z_BASE_HORN = PLATE_T - J1_POCKET + 40.5 + 2.0 + 3.5   # 49: J1 horn top
HUB_Z0 = Z_BASE_HORN + 1.0    # turntable hub underside (1 mm gap to horn: servo shaft carries no moment)
RAD_BRG = (30.0, 55.0, 13.0)      # 6006ZZ: bore, outer diameter, width
RAD_BRG_Z0 = 55.0                 # hub extends 5 mm below the bearing: retaining collar
THRUST_BRG = (35.0, 52.0, 12.0)   # 51107
Z_BASE_TOP = 72.0      # housing top
CHAMBER_R = (30.0, 42.0)          # cable chamber on top of the base (clock spring), 10 mm deep
CHAMBER_DEPTH = 10.0
THRUST_Z0 = Z_BASE_TOP - 2.0      # thrust bearing sits in a 2 mm recess
Z_TURNTABLE = THRUST_Z0 + THRUST_BRG[2]   # turntable underside (80)
TURNTABLE_T = 12.0
TOWER_FLANGE_T = 8.0
Z_SHOULDER = Z_TURNTABLE + TURNTABLE_T + 42.0   # J2 (134)
L_UPPER = 110.0        # upper arm (J2 -> J3)
L_FOREARM = 110.0      # forearm (J3 -> J4)
Z_ELBOW = Z_SHOULDER + L_UPPER
Z_WRIST = Z_ELBOW + L_FOREARM
# The J5 wrist roll servo sits on top of the wrist bridge:
Z_ROLL_HORN = Z_WRIST + BRIDGE_GAP + BRIDGE_T + SERVO_H + HORN_GAP + HORN_T   # J4 + 82

# --- Gripper (rack and pinion, module 1.5, involute, profile shift +0.1) -----------------
GEAR_M = 1.5
PINION_Z = 16
PROFILE_SHIFT = 0.1
PINION_R = PINION_Z * GEAR_M / 2               # pitch radius 12 mm
GEAR_BACKLASH = 0.15                           # mm, taken from tooth thickness
FINGER_TRAVEL = 22.0                           # per finger, mm (opening 2..46 mm)
MM_PER_DEG = 2 * 3.141592653589793 * PINION_R / 360
FINGER_ASBUILT = 0.0                           # as-built pose: fingers closed (zero)

# --- Servo selection and masses (g) ----------------------------------------------------------
SERVOS = {
    "base_yaw":    ("DS3218MG", 60.0),
    "shoulder":    ("RDS3235", 66.0),     # weight not on the datasheet: estimate
    "elbow":       ("DS3230MG", 67.0),
    "wrist_pitch": ("DS3218MG", 60.0),
    "wrist_roll":  ("DS3218MG", 60.0),
    "gripper":     ("MG90S", 13.4),
}
SUPPLY_V = 6.0         # servo rail (DS32xx max 6.8 V; margin for XL4016 drift)
STALL_KGCM = {"DS3218MG": 20.0, "DS3230MG": 27.5, "RDS3235": 32.0, "MG90S": 2.2}   # 6.0 V
