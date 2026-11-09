"""Sherpa Mini (Annex) im SV08-Koordinatensystem.

Alle Teile des Sherpa-STEP werden mit EINER starren Transformation platziert,
dadurch bleiben die relativen Lagen von Motor, Zahnrädern, Walzen und
Filamentachse exakt erhalten.

Sherpa-Koordinaten: Filament läuft in Y bei (x=4.3, z=6.0), Motorachse in Z,
Motor-Anlagefläche bei z=-7, Motor bei y=+13.85 über der Antriebswelle.
Abbildung: X_sh -> X, Y_sh -> Z (Motor oben), Z_sh -> -Y (Motor hinten).
"""
from build123d import Compound, Location, Pos, Rot, import_step

import basis as b

# Antriebsrad und Idler-Rad sind gegenüber dem Original umgedreht (180° um eine Querachse
# in ihrer Mittelebene z=10). Die Filamentrille liegt dadurch bei z=14 statt z=6, der ganze
# Sherpa samt Motor sitzt 8 mm weiter hinten.
FLIP_Z = 10.0
FLIPPED = {"antriebsrad", "idler_rad", "nadellager", "madenschraube_M3"}
FIL_SH = (4.3, 2 * FLIP_Z - 6.0)   # Filamentachse im Sherpa (x, z)
MOTOR_SH_Y = 13.85           # Motorachse über der Antriebswelle (y_sh)
HOUSING_BOT_SH = -13.5       # Unterkante Gehäusekern (y_sh)
MOTOR_FACE_SH = -7.0

CARRIER_T = 5.0              # Goliath-Trägerplatte unter dem Gehäuse
GOLIATH_LEN = 43.50 + 25.75  # Oberkante Kühlkörper -> Düsenspitze

# Gehäuse-Unterkante so, dass die Düse auf Originalhöhe liegt
Z_HOUSING_BOT = b.ORIG_NOZZLE_Z + GOLIATH_LEN + CARRIER_T
Z0 = Z_HOUSING_BOT - HOUSING_BOT_SH          # z_sv von y_sh = 0 (Antriebswelle)
X0 = b.AX_X - FIL_SH[0]
Y0 = b.AX_Y + FIL_SH[1]                      # y_sv = Y0 - z_sh

T = Location((X0, Y0, Z0)) * Rot(90, 0, 0)   # Rx(90): y->z, z->-y


FLIP = Location((0, 0, FLIP_Z)) * Rot(180, 0, 0) * Location((0, 0, -FLIP_Z))


def to_sv(x, y, z):
    return (X0 + x, Y0 - z, Z0 + y)


MOTOR_AX = to_sv(FIL_SH[0], MOTOR_SH_Y, 0)[0::2]   # (x, z)
MOTOR_FACE_Y = Y0 - MOTOR_FACE_SH

# Motor um die eigene Achse gedreht, von vorne gesehen im Uhrzeigersinn (Ritzel bleibt koaxial)
MOTOR_ROT = 36.0
MOTOR_TURN = Pos(MOTOR_AX[0], 0, MOTOR_AX[1]) * Rot(0, MOTOR_ROT, 0) * Pos(-MOTOR_AX[0], 0, -MOTOR_AX[1])

# Teile über Volumen identifiziert (Labels im STEP sind teils doppelt)
_NAMES = {
    5622: "housing_core", 821: "housing_front", 952: "housing_front_k", 2517: "housing_rear",
    1593: "idler_arm_short", 2128: "idler_arm_long", 18114: "motor",
}


# Sprechende Namen für die Blätter des Sherpa-STEP (Label-Anfang -> Name)
_LABELS = [
    ("shaft", "welle"), ("0_3mm_shim", "passscheibe_0_3"), ("sherpa_mini_gear_50t", "zahnrad_50z"),
    ("filament_gear", "antriebsrad"), ("MR85-2RS", "lager_MR85"), ("m3_set_screw", "madenschraube_M3"),
    ("idler_pin", "idler_stift"), ("idler_gear", "idler_rad"), ("needle_bearing", "nadellager"),
    ("insert_m3x5x4", "einschmelzmutter_M3"), ("thumbscrew", "raendelschraube"), ("spring", "feder"),
    ("NEMA14_10t", "motor"),
]
_SCREWS = {111: "schraube_M3x12_motor", 140: "schraube_M3x16_gehaeuse", 83: "schraube_M3x8_deckel"}


def _leaves(n):
    ch = list(n.children)
    return [n] if not ch else [l for c in ch for l in _leaves(c)]


def _load():
    """Liste (name, shape) je Blatt des STEP-Baums. Die globalen Volumenkörper kommen aus
    asm.solids() (gleiche Reihenfolge wie die Blätter, Lage bereits aufgelöst)."""
    asm = import_step(str(b.SHERPA_STEP))
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
            if name == "welle":
                name = "antriebswelle" if vol > 300 else "idler_welle"
        flip = FLIP if name in FLIPPED else Location()
        group = [T * flip * s for s in group]
        if name == "motor":
            group = [MOTOR_TURN * s for s in group]
        shape = group[0] if k == 1 else Compound(group)
        parts.append((name, shape))
    return parts


parts = _load()


def get(name):
    return [p for n, p in parts if n == name]


# Teile, die im Umbau entfallen bzw. ersetzt werden (neues Gehäuse folgt)
UNUSED = {"housing_core", "housing_rear", "housing_front", "housing_front_k", "idler_arm_long",
          "einschmelzmutter_M3", "schraube_M3x12_motor", "schraube_M3x16_gehaeuse", "schraube_M3x8_deckel"}

if __name__ == "__main__":
    print(f"Motorachse x={MOTOR_AX[0]:.2f} z={MOTOR_AX[1]:.2f}, Anlagefläche y={MOTOR_FACE_Y:.2f}")
    print(f"Gehäuse-Unterkante z={Z_HOUSING_BOT:.2f}")
    for n, p in parts:
        bb = p.bounding_box()
        print(f"{n:16s} ({bb.min.X:.1f},{bb.min.Y:.1f},{bb.min.Z:.1f})-({bb.max.X:.1f},{bb.max.Y:.1f},{bb.max.Z:.1f})")
