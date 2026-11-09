"""
Goliath-Träger für den Sherpa-Umbau des Sovol SV08.

Platte unter dem Sherpa-Gehäuse. Das Goliath (luftgekühlt) wird von oben mit
3x M2,5 angeschraubt (Köpfe versenkt), danach wird der Träger von unten mit
3x M3 an den Hauptkörper geschraubt (Gewindeeinsätze im Hauptkörper).
Dazu: Ohr für den 2515-Hotendlüfter, Auge für die M2,5 des 5020-Lüfters.

Koordinaten = Sovol-STEP-Baugruppe (mm). Düse liegt auf Originalposition.
"""
import math

from build123d import Align, Box, Cylinder, Pos, Rot, export_step, import_step

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
GOL_SCREWS_DEG = [0, 90, 270]
SCREW_D, HEAD_D, HEAD_DEPTH = 2.9, 5.0, 2.6     # M2,5 DIN 912

# Befestigung am Hauptkörper: M3 von unten, Kopf ISO 7380/DIN 912 Ø5,5
M3_HOLES = [(-354.05, 225.05), (-345.0, 236.0), (-328.0, 236.0)]
M3_D, M3_HEAD_D = 3.3, 6.0

# 2515-Hotendlüfter (Original-Lüfter, an +X-Seite)
FAN_FACE_X = -324.30
FAN_SHIFT_X = FAN_FACE_X - (-328.70)
FAN_DZ = 12.0                         # Lüfter mittig vor den Kühlkörper (über dem Heizblock)
FAN_HOLES = [(217.55, -167.57 + FAN_DZ), (237.55, -167.57 + FAN_DZ)]   # (y, z) Lochlage am Lüfter
FAN_HOLE_D, FAN_HOLE_DEPTH = 2.2, 7.0
EAR_Y = (215.05, 240.05)
EAR_X = (-329.0, FAN_FACE_X)

# 5020-Bauteillüfter: eine M2,5 in Y-Richtung (Originalstelle, Lüfter 1 mm vor)
FAN5020_SHIFT = (0.0, -2.0, 0.0)
FAN5020_HOLE = (-360.85, -166.95)     # (x, z)
FAN5020_BACK_Y = 214.70 + FAN5020_SHIFT[1]

# ---------------------------------------------------------------- Träger
def build():
    plate = Pos(AX_X, AX_Y, Z_BOT) * Cylinder(13.0, T, align=MIN)
    # Steg nach hinten zu den beiden hinteren M3 und nach links zur linken M3
    plate += Pos(-336.5, 233.0, Z_BOT) * Box(25.0, 12.0, T, align=MIN)
    plate += Pos(-347.0, 225.05, Z_BOT) * Box(20.0, 12.0, T, align=MIN)
    for x, y in M3_HOLES:
        plate += Pos(x, y, Z_BOT) * Cylinder(4.0, T, align=MIN)
    # Ohr für 2515 (reicht bis unter die Lüfterbohrungen)
    ear_top = Z_TOP
    ear_bot = FAN_HOLES[0][1] - 4.0
    plate += Pos(sum(EAR_X) / 2, sum(EAR_Y) / 2, ear_bot) * Box(
        EAR_X[1] - EAR_X[0], EAR_Y[1] - EAR_Y[0], ear_top - ear_bot, align=MIN)
    plate += Pos(-331.0, sum(EAR_Y) / 2, Z_BOT) * Box(6.0, EAR_Y[1] - EAR_Y[0], T, align=MIN)
    # Auge für 5020 (links vorne, hängt vom Träger nach unten)
    ex, ez = FAN5020_HOLE
    eye_y0 = FAN5020_BACK_Y
    plate += Pos(ex, eye_y0 + 3.0, ez - 4.0) * Box(7.0, 6.0, Z_TOP - (ez - 4.0), align=MIN)
    plate += Pos(-355.0, 219.0, Z_BOT) * Box(12.0, 9.0, T, align=MIN)

    # Filamentkanal + Zentrierung
    plate -= Pos(AX_X, AX_Y, Z_BOT - 1) * Cylinder(PTFE_D / 2, T + 2, align=MIN)
    plate -= Pos(AX_X, AX_Y, Z_BOT) * Cylinder(INLET_D / 2, INLET_DEPTH, align=MIN)
    # Goliath-Schrauben (von oben, Köpfe versenkt)
    for deg in GOL_SCREWS_DEG:
        x = AX_X + GOL_SCREW_R * math.cos(math.radians(deg))
        y = AX_Y + GOL_SCREW_R * math.sin(math.radians(deg))
        plate -= Pos(x, y, Z_BOT - 1) * Cylinder(SCREW_D / 2, T + 2, align=MIN)
        plate -= Pos(x, y, Z_TOP - HEAD_DEPTH) * Cylinder(HEAD_D / 2, 10, align=MIN)
    # M3 zum Hauptkörper
    for x, y in M3_HOLES:
        plate -= Pos(x, y, Z_BOT - 1) * Cylinder(M3_D / 2, T + 2, align=MIN)
    # 2515-Gewindelöcher (von +X)
    for y, z in FAN_HOLES:
        plate -= Pos(FAN_FACE_X + 0.1, y, z) * Rot(0, -90, 0) * Cylinder(
            FAN_HOLE_D / 2, FAN_HOLE_DEPTH + 0.1, align=MIN)
    # 5020-Gewindeloch (von vorne, -Y)
    plate -= Pos(ex, eye_y0 - 0.1, ez) * Rot(-90, 0, 0) * Cylinder(FAN_HOLE_D / 2, 7.0, align=MIN)
    # Freiraum zum 2515-Lüfter (liegt mit Oberkante knapp unter dem Träger)
    plate -= Pos(FAN_FACE_X, sum(EAR_Y) / 2, Z_BOT - 20) * Box(30, 40, 20 + T + 1, align=(Align.MIN, Align.CENTER, Align.MIN))
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
