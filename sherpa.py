"""Sherpa Mini (Annex) im SV08-Koordinatensystem.

Alle Teile des Sherpa-STEP werden mit EINER starren Transformation platziert,
dadurch bleiben die relativen Lagen von Motor, Zahnrädern, Walzen und
Filamentachse exakt erhalten.

Sherpa-Koordinaten: Filament läuft in Y bei (x=4.3, z=6.0), Motorachse in Z,
Motor-Anlagefläche bei z=-7, Motor bei y=+13.85 über der Antriebswelle.
Abbildung: X_sh -> X, Y_sh -> Z (Motor oben), Z_sh -> -Y (Motor hinten).
"""
from build123d import Location, Pos, Rot, import_step

import basis as b

FIL_SH = (4.3, 6.0)          # Filamentachse im Sherpa (x, z)
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


def to_sv(x, y, z):
    return (X0 + x, Y0 - z, Z0 + y)


MOTOR_AX = to_sv(FIL_SH[0], MOTOR_SH_Y, 0)[0::2]   # (x, z)
MOTOR_FACE_Y = Y0 - MOTOR_FACE_SH

# Teile über Volumen identifiziert (Labels im STEP sind teils doppelt)
_NAMES = {
    5622: "housing_core", 821: "housing_front", 952: "housing_front_k", 2517: "housing_rear",
    1593: "idler_arm_short", 2128: "idler_arm_long", 18114: "motor",
}


def _load():
    asm = import_step(str(b.SHERPA_STEP))
    parts = []
    for s in asm.solids():
        name = next((n for v, n in _NAMES.items() if abs(s.volume - v) < 2), "teil")
        parts.append((name, T * s))
    return parts


parts = _load()


def get(name):
    return [p for n, p in parts if n == name]


# Teile, die im Umbau entfallen bzw. ersetzt werden
UNUSED = {"housing_front_k", "idler_arm_long"}

if __name__ == "__main__":
    print(f"Motorachse x={MOTOR_AX[0]:.2f} z={MOTOR_AX[1]:.2f}, Anlagefläche y={MOTOR_FACE_Y:.2f}")
    print(f"Gehäuse-Unterkante z={Z_HOUSING_BOT:.2f}")
    for n, p in parts:
        if n != "teil":
            bb = p.bounding_box()
            print(f"{n:16s} ({bb.min.X:.1f},{bb.min.Y:.1f},{bb.min.Z:.1f})-({bb.max.X:.1f},{bb.max.Y:.1f},{bb.max.Z:.1f})")
