"""
Goliath-Hotend-Träger für Sovol SV08 (planetarer Original-Extruder).

Ersetzt den originalen Alu-Kühlblock (JXHSV08-06022). Oben übernimmt der
Träger exakt dessen zwei Zinken, die die Nase des Extruder-Getriebegehäuses
umgreifen und mit den originalen 2x M3x16 quer verschraubt werden. Darunter
hängt das Mellow/VzBoT Goliath (luftgekühlt) an einer Platte.

Koordinaten = Koordinatensystem der Sovol-STEP-Baugruppe (mm), damit alles
direkt gegen das Original geprüft werden kann.

Aufruf:  .venv\\Scripts\\python goliath_traeger.py
Ausgabe: out/goliath_traeger.step / .stl, out/check_assembly.step
"""
from pathlib import Path

from build123d import (
    Axis, Box, Compound, Cylinder, Location, Pos, Rot, import_step,
    export_step, export_stl, Align,
)

ROOT = Path(__file__).parent
SV08_STEP = ROOT / "ext/SV08/STEP/JXHSV08-06000 挤出喷头组件（行星）hotend assembly planetary.step"
GOLIATH_STEP = ROOT / "ext/Goliath/CAD/Goliath Mockup AIR.STEP"
OUT = ROOT / "out"

# ---------------------------------------------------------------- Parameter
AX_X, AX_Y = -335.70, 225.05     # Filamentachse (aus Original-STEP)
Z_GC = -166.07                   # Unterkante Nase Getriebegehäuse
PLATE_T = 5.0                    # Plattenstärke unter dem Getriebegehäuse
PLATE_R = 12.5                   # Radius der Platte um die Achse
BLOCK_X = (-342.70, -328.70)     # Grundriss Original-Kühlblock
BLOCK_Y = (215.05, 240.05)

FAN_FACE_X = -324.30             # neue Anlagefläche 2515-Lüfter (Original -328.70)
EAR_TOP = -160.0                 # Oberkante Lüfter-Ohr
FAN_HOLES = [(217.55, -167.57), (237.55, -167.57)]  # (y, z) Original-Lage
FAN_HOLE_D = 2.2                 # M2,5 selbstschneidend in ASA
FAN_HOLE_DEPTH = 7.0

GOLIATH_ROT = 135.0              # Drehung um Z: Heizpatrone zeigt nach +X
GOLIATH_TOP = 43.50              # lokale Oberkante Goliath-Kühlkörper
GOLIATH_TIP = -25.75             # lokale Düsenspitze
GOL_SCREW_R = 8.0                # Lochkreis 4x M2,5 (Quadrat 11,32)
GOL_SCREWS_DEG = [0, 90, 270]    # bei 180 deg kollidiert der Kopf mit Zinke + M3-Kopf
SCREW_D = 2.9                    # Durchgang M2,5
HEAD_D = 5.0                     # Kopfsenkung DIN 912 M2,5 (Kopf 4,5)
HEAD_DEPTH = 2.6
PTFE_D = 4.15
INLET_D, INLET_DEPTH = 4.6, 1.4  # Zentrierung auf Goliath-Einlass

ORIG_NOZZLE_Z = -224.07

# ---------------------------------------------------------------- Laden
def load_parts(path):
    asm = import_step(str(path))
    return {c.label: c for c in asm.children}

def find(parts, key):
    hits = [p for l, p in parts.items() if key in l]
    assert hits, key
    return hits

sv08 = load_parts(SV08_STEP)
block = find(sv08, "ooling_aluminum_block")[0]

# ---------------------------------------------------------------- Träger
z_plate_bot = Z_GC - PLATE_T
big = 200
_cut = Pos(AX_X, AX_Y, Z_GC + big / 2) * Box(big, big, big)
prongs = None
for s in block.solids():
    for r in s.intersect(_cut).solids():
        prongs = r if prongs is None else prongs + r

plate = Pos(AX_X, AX_Y, z_plate_bot) * Cylinder(PLATE_R, PLATE_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
plate += Pos((BLOCK_X[0] + FAN_FACE_X) / 2, sum(BLOCK_Y) / 2, z_plate_bot) * Box(
    FAN_FACE_X - BLOCK_X[0], BLOCK_Y[1] - BLOCK_Y[0], PLATE_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
ear_x0 = BLOCK_X[1] - 0.3
ear = Pos((ear_x0 + FAN_FACE_X) / 2, sum(BLOCK_Y) / 2, Z_GC) * Box(
    FAN_FACE_X - ear_x0, BLOCK_Y[1] - BLOCK_Y[0], EAR_TOP - Z_GC, align=(Align.CENTER, Align.CENTER, Align.MIN))

traeger = prongs + plate + ear

# Filamentkanal + Zentrierung
traeger -= Pos(AX_X, AX_Y, z_plate_bot - 1) * Cylinder(PTFE_D / 2, 30, align=(Align.CENTER, Align.CENTER, Align.MIN))
traeger -= Pos(AX_X, AX_Y, z_plate_bot) * Cylinder(INLET_D / 2, INLET_DEPTH, align=(Align.CENTER, Align.CENTER, Align.MIN))

# Goliath-Schrauben (lokale Lochlage 45 deg + Drehung -> Achsen)
import math
for deg in GOL_SCREWS_DEG:
    x = AX_X + GOL_SCREW_R * math.cos(math.radians(deg))
    y = AX_Y + GOL_SCREW_R * math.sin(math.radians(deg))
    traeger -= Pos(x, y, z_plate_bot - 1) * Cylinder(SCREW_D / 2, 30, align=(Align.CENTER, Align.CENTER, Align.MIN))
    traeger -= Pos(x, y, z_plate_bot + PLATE_T - HEAD_DEPTH) * Cylinder(HEAD_D / 2, 30, align=(Align.CENTER, Align.CENTER, Align.MIN))

# 2515-Lüfter-Gewindelöcher (von +X)
for y, z in FAN_HOLES:
    traeger -= Pos(FAN_FACE_X + 0.1, y, z) * Rot(0, -90, 0) * Cylinder(
        FAN_HOLE_D / 2, FAN_HOLE_DEPTH + 0.1, align=(Align.CENTER, Align.CENTER, Align.MIN))

# Durchgang für die Spitzen der originalen M3x16 (Klemmung Getriebegehäuse)
for y in (220.55, 229.55):
    traeger -= Pos(FAN_FACE_X - 6, y, -161.07) * Rot(0, 90, 0) * Cylinder(1.7, 12)

# Passform für den Ansatz des verschobenen 2515-Lüfters
fan2515 = Pos(FAN_FACE_X - BLOCK_X[1], 0, 0) * find(sv08, "2515_pipe_cooling_fan")[0]
for s in fan2515.solids():
    traeger -= s

# ---------------------------------------------------------------- Goliath platzieren
gol = import_step(str(GOLIATH_STEP))
dz = z_plate_bot - GOLIATH_TOP
gol = Pos(AX_X, AX_Y, dz) * Rot(0, 0, GOLIATH_ROT) * gol
nozzle_z = GOLIATH_TIP + dz
DROP = ORIG_NOZZLE_Z - nozzle_z

if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    traeger.label = "goliath_traeger"
    export_step(traeger, str(OUT / "goliath_traeger.step"))
    export_stl(traeger, str(OUT / "goliath_traeger.stl"), tolerance=0.01, angular_tolerance=0.1)
    print(f"Düsenspitze neu z={nozzle_z:.2f}  (Original {ORIG_NOZZLE_Z})  -> {DROP:.2f} mm tiefer")
    print(f"Volumen Träger: {traeger.volume:.0f} mm^3, bbox {traeger.bounding_box().size}")
