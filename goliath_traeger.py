"""
Goliath-Träger für den Sherpa-Umbau des Sovol SV08.

Rechteckige Platte, die hinten direkt in die Hinterwand übergeht (wird in
extruder_body.py mit dem Hauptkörper verschmolzen). Das Goliath (luftgekühlt)
wird von oben mit 3x M2,5 angeschraubt (Köpfe versenkt).
Dazu: Ohr für den 2515-Hotendlüfter (auf der -X-Seite). Der 5020-Lüfter ist hier nicht mehr befestigt.

Koordinaten = Sovol-STEP-Baugruppe (mm). Düse liegt auf Originalposition.
"""
import math

from build123d import Align, Box, Cylinder, Plane, Pos, Rot, export_step, import_step

import basis as b
import sherpa as sh

MIN = (Align.CENTER, Align.CENTER, Align.MIN)

# ---------------------------------------------------------------- Parameter
AX_X, AX_Y = b.AX_X, b.AX_Y
Z_TOP = sh.Z_HOUSING_BOT              # Anlage am Gehäuse
T = sh.CARRIER_T
Z_BOT = Z_TOP - T

PTFE_D = 4.15
INLET_D, INLET_DEPTH = 4.6, 1.4       # Zentrierung auf Goliath-Einlass

GOLIATH_ROT = 135.0                   # Heizpatrone zeigt nach +X
GOLIATH_TOP = 43.50
GOLIATH_TIP = -25.75
GOL_SCREW_R = 8.0
GOL_SCREWS_DEG = [45, 135, 225, 315]
SCREW_D, HEAD_D, HEAD_DEPTH = 2.9, 5.0, 2.6     # M2,5 DIN 912

# Rechteckige Platte, hinten direkt mit der Hinterwand (Hauptkörper) verbunden
X_L, X_R = -367.548, -305.501            # volle Breite des Hauptträgers
Y_FRONT = 214.2                       # knapp hinter dem Platinenträger (y=214)
PCB_SCREW, PCB_SCREW_R = (-336.5, -153.2), 2.0
Y_WALL = 242.8747                     # install_plate.Y_STEP (reicht bis in die Wand)

# 2515-Hotendlüfter: Maße wie an der Original-(+X-)Seite, Lüfter und Ohr werden dann an der
# Filamentachse (x=AX_X) auf die -X-Seite gespiegelt
FAN_MIRROR = Plane.YZ.offset(AX_X)
FAN_FACE_X = -323.80                  # gespiegelt: Lüfterfläche bei x=-347.6
FAN_SHIFT_X = FAN_FACE_X - (-328.70)
FAN_TOP_ORIG = -165.075                # Oberkante des 2515 im SV08-Modell (ohne Kabel)
FAN_DZ = Z_BOT - FAN_TOP_ORIG          # Lüfter direkt unter dem Träger
FAN_HOLES = [(217.55, -167.57 + FAN_DZ), (237.55, -167.57 + FAN_DZ)]   # (y, z) Lochlage am Lüfter
FAN_HOLE_D, FAN_HOLE_DEPTH = 2.2, 7.0
EAR_Y = (215.05, 240.05)
EAR_X = (-329.0, FAN_FACE_X)

# 5020-Bauteillüfter: Lage wie bisher (2 mm vor), nicht mehr am Träger befestigt
FAN5020_SHIFT = (0.0, -2.0, 0.0)

# ---------------------------------------------------------------- Träger
def build():
    plate = Pos((X_L + X_R) / 2, (Y_FRONT + Y_WALL) / 2, Z_BOT) * Box(X_R - X_L, Y_WALL - Y_FRONT, T, align=MIN)

    # Filamentkanal + Zentrierung
    plate -= Pos(AX_X, AX_Y, Z_BOT - 1) * Cylinder(PTFE_D / 2, T + 2, align=MIN)
    plate -= Pos(AX_X, AX_Y, Z_BOT) * Cylinder(INLET_D / 2, INLET_DEPTH, align=MIN)
    # Goliath-Schrauben (von oben, Köpfe versenkt)
    for deg in GOL_SCREWS_DEG:
        x = AX_X + GOL_SCREW_R * math.cos(math.radians(deg))
        y = AX_Y + GOL_SCREW_R * math.sin(math.radians(deg))
        plate -= Pos(x, y, Z_BOT - 1) * Cylinder(SCREW_D / 2, T + 2, align=MIN)
        plate -= Pos(x, y, Z_TOP - HEAD_DEPTH) * Cylinder(HEAD_D / 2, 10, align=MIN)
    # Untere Platinen-Passschraube ragt von vorne in die Platte (später Gewindeeinsatz)
    plate -= Pos(PCB_SCREW[0], Y_FRONT - 1, PCB_SCREW[1]) * Rot(-90, 0, 0) * Cylinder(PCB_SCREW_R, 6.0, align=MIN)
    # Freiraum für den Goliath selbst (Kühlkörper-Oberseite liegt plan an)
    return plate


def place_goliath():
    gol = import_step(str(b.GOLIATH_STEP))
    dz = Z_BOT - GOLIATH_TOP
    return Pos(AX_X, AX_Y, dz) * Rot(0, 0, GOLIATH_ROT) * gol, GOLIATH_TIP + dz


traeger = build()
gol, nozzle_z = place_goliath()
gol_solid = gol.solids()[0]
traeger = traeger - gol_solid

if __name__ == "__main__":
    b.OUT.mkdir(exist_ok=True)
    traeger.label = "goliath_traeger"
    export_step(traeger, str(b.OUT / "goliath_traeger.step"))
    print(f"Düsenspitze z={nozzle_z:.2f} (Original {b.ORIG_NOZZLE_Z})")
    print(f"Volumen Träger: {traeger.volume:.0f} mm^3, bbox {traeger.bounding_box().size}")
