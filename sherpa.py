"""Sherpa Mini (Annex) in the SV08 coordinate system.

All parts of the Sherpa STEP are placed with ONE rigid transformation, so the relative
positions of motor, gears, drive rollers and filament axis are kept exactly.

Sherpa coordinates: filament runs along Y at (x=4.3, z=6.0), motor axis along Z,
motor mounting face at z=-7, motor at y=+13.85 above the drive shaft.
Mapping: X_sh -> X, Y_sh -> Z (motor on top), Z_sh -> -Y (motor at the back).
"""
import math

from build123d import Compound, Location, Pos, Rot, import_step

import common as cm

# Drive gear and idler gear are flipped compared to the original (180° about a transverse axis
# in their mid plane z=10). The filament groove is therefore at z=14 instead of z=6, and the whole
# Sherpa including the motor sits 8 mm further back.
FLIP_Z = 10.0
FLIPPED = {"drive_gear", "idler_gear", "needle_bearing", "set_screw_M3"}
FIL_SH = (4.3, 2 * FLIP_Z - 6.0)   # filament axis in Sherpa coordinates (x, z)
MOTOR_SH_Y = 13.85           # motor axis above the drive shaft (y_sh)
HOUSING_BOT_SH = -13.5       # bottom of the housing core (y_sh)
MOTOR_FACE_SH = -7.0

CARRIER_T = 5.0              # Goliath carrier plate below the housing
GOLIATH_LEN = 43.50 + 25.75  # top of heat sink -> nozzle tip

# Housing bottom chosen so that the nozzle is at its original height
Z_HOUSING_BOT = cm.ORIG_NOZZLE_Z + GOLIATH_LEN + CARRIER_T
LIFT = 1.0                                   # Sherpa lifted above the carrier (clears the 50t gear)
Z0 = Z_HOUSING_BOT - HOUSING_BOT_SH + LIFT   # z_sv of y_sh = 0 (drive shaft)
X0 = cm.AX_X - FIL_SH[0]
Y0 = cm.AX_Y + FIL_SH[1]                     # y_sv = Y0 - z_sh

T = Location((X0, Y0, Z0)) * Rot(90, 0, 0)   # Rx(90): y->z, z->-y


FLIP = Location((0, 0, FLIP_Z)) * Rot(180, 0, 0) * Location((0, 0, -FLIP_Z))


def to_sv(x, y, z):
    return (X0 + x, Y0 - z, Z0 + y)


# Motor raised compared to the original STEP: centre distance pinion - 50t gear 15.0 mm
# (module 0.5, 10/50 teeth) instead of 14.5 mm in the CAD model
CENTER_DIST = 15.0
MOTOR_DZ = math.sqrt(CENTER_DIST ** 2 - FIL_SH[0] ** 2) - MOTOR_SH_Y
MOTOR_AX = (to_sv(FIL_SH[0], MOTOR_SH_Y, 0)[0], to_sv(FIL_SH[0], MOTOR_SH_Y, 0)[2] + MOTOR_DZ)   # (x, z)
MOTOR_FACE_Y = Y0 - MOTOR_FACE_SH

# Parts identified by volume (labels in the STEP are partly duplicated)
_NAMES = {
    5622: "housing_core", 821: "housing_front", 952: "housing_front_k", 2517: "housing_rear",
    1593: "idler_arm_short", 2128: "idler_arm_long", 18114: "motor",
}


# Readable names for the leaves of the Sherpa STEP (label prefix -> name)
_LABELS = [
    ("shaft", "shaft"), ("0_3mm_shim", "shim_0_3"), ("sherpa_mini_gear_50t", "gear_50t"),
    ("filament_gear", "drive_gear"), ("MR85-2RS", "bearing_MR85"), ("m3_set_screw", "set_screw_M3"),
    ("idler_pin", "idler_pin"), ("idler_gear", "idler_gear"), ("needle_bearing", "needle_bearing"),
    ("insert_m3x5x4", "heat_insert_M3"), ("thumbscrew", "thumbscrew"), ("spring", "spring"),
    ("NEMA14_10t", "motor"),
]
_SCREWS = {111: "screw_M3x12_motor", 140: "screw_M3x16_housing", 83: "screw_M3x8_lid"}


def _leaves(n):
    ch = list(n.children)
    return [n] if not ch else [l for c in ch for l in _leaves(c)]


def _load():
    """List of (name, shape) per leaf of the STEP tree. The global solids come from asm.solids()
    (same order as the leaves, locations already resolved)."""
    asm = import_step(str(cm.SHERPA_STEP))
    solids = asm.solids()
    parts, i = [], 0
    for leaf in _leaves(asm):
        k = len(leaf.solids())
        group = list(solids[i:i + k])
        i += k
        lab = leaf.label.split("^")[0]
        vol = sum(s.volume for s in group)
        name = next((n for v, n in _NAMES.items() if abs(vol - v) < 2), None)
        if name is None and "screw" in lab:
            name = next((n for v, n in _SCREWS.items() if abs(vol - v) < 2), None)
        if name is None:
            name = next((n for key, n in _LABELS if lab.startswith(key)), lab)
            if name == "shaft":
                name = "drive_shaft" if vol > 300 else "idler_shaft"
        flip = FLIP if name in FLIPPED else Location()
        group = [T * flip * s for s in group]
        if name == "motor":
            group = [Pos(0, 0, MOTOR_DZ) * s for s in group]
        shape = group[0] if k == 1 else Compound(group)
        parts.append((name, shape))
    return parts


parts = _load()


def get(name):
    return [p for n, p in parts if n == name]


# Parts dropped or replaced by the conversion (the housing is new)
UNUSED = {"housing_core", "housing_rear", "housing_front", "housing_front_k", "idler_arm_long",
          "heat_insert_M3", "screw_M3x12_motor", "screw_M3x16_housing", "screw_M3x8_lid"}


def idler_insert():
    """Heat-set insert for the idler thumbscrew (sits in the housing core in the original)."""
    return next(p for n, p in parts if n == "heat_insert_M3" and abs(p.bounding_box().center().X - (X0 + 9.2)) < 1.0
                and p.bounding_box().size.X < p.bounding_box().size.Y)


if __name__ == "__main__":
    print(f"Motor axis x={MOTOR_AX[0]:.2f} z={MOTOR_AX[1]:.2f}, mounting face y={MOTOR_FACE_Y:.2f}")
    print(f"Housing bottom z={Z_HOUSING_BOT:.2f}")
    for n, p in parts:
        bb = p.bounding_box()
        print(f"{n:16s} ({bb.min.X:.1f},{bb.min.Y:.1f},{bb.min.Z:.1f})-({bb.max.X:.1f},{bb.max.Y:.1f},{bb.max.Z:.1f})")
