"""
Schrauben und Einschmelzmuttern der Neuteile (vereinfachte Modelle für Assembly und Kollisionscheck).

Schrauben: Zylinderkopf + glatter Schaft (Ø etwas unter Nennmaß, damit Gewindebohrungen in Kaufteilen
nicht als Kollision zählen). Einschmelzmuttern: Hülse mit Innenbohrung; der Presssitz in der Bohrung ist
gewollt (check.py ignoriert Teile mit "einschmelzmutter" im Namen).
"""
import math

from build123d import Align, Cylinder, Pos, Rot

import basis as b
import gehaeuse as gh
import goliath_traeger as gtr
import sherpa as sh

MIN = (Align.CENTER, Align.CENTER, Align.MIN)

# DIN 912: (Kopf-Ø, Kopfhöhe, Schaft-Ø im Modell)
M3 = (5.5, 3.0, 2.9)
M25 = (4.5, 2.5, 2.0)                    # Schaft = Kernloch im Goliath (Ø2,08)
# Einschmelzmutter M3 für Bohrung Ø4,0 (Außen-Ø, Länge, Innen-Ø)
INSERT_M3 = (4.4, 5.0, 2.5)


def _axis(d):
    """Rotation, die die +Z-Achse eines Zylinders auf die Richtung d (Einheitsvektor einer Achse) dreht."""
    return {(1, 0, 0): Rot(0, 90, 0), (-1, 0, 0): Rot(0, -90, 0),
            (0, 1, 0): Rot(-90, 0, 0), (0, -1, 0): Rot(90, 0, 0),
            (0, 0, 1): Rot(0, 0, 0), (0, 0, -1): Rot(180, 0, 0)}[d]


def schraube(p, d, length, size=M3):
    """Schraube mit Kopfauflage im Punkt p, Schaft in Richtung d (Einheitsvektor), Länge ohne Kopf."""
    hd, hh, sd = size
    r = _axis(d)
    back = tuple(-v for v in d)
    head = Pos(*p) * _axis(back) * Cylinder(hd / 2, hh, align=MIN)
    shank = Pos(*p) * r * Cylinder(sd / 2, length, align=MIN)
    return head + shank


def einschmelzmutter(p, d, size=INSERT_M3):
    """Einschmelzmutter, beginnend an der Bohrungsöffnung p, in Richtung d in das Bauteil hinein."""
    od, ln, idm = size
    r = _axis(d)
    return Pos(*p) * r * (Cylinder(od / 2, ln, align=MIN) - Cylinder(idm / 2, ln, align=MIN))


def build():
    out = []
    yb, yw = gh.Y_BEAM_FRONT, sh.MOTOR_FACE_Y
    # Platinenhalter <-> Motorplatte: Mutter in der Stütze, Schraube von der Motorseite
    for i, (x, z) in enumerate(gh.schrauben(), 1):
        out.append((f"einschmelzmutter_stuetze_{i}", einschmelzmutter((x, yb, z), (0, -1, 0))))
        out.append((f"schraube_M3x14_stuetze_{i}", schraube((x, yw, z), (0, -1, 0), 14)))
    # Motor: 2x M3x6 von vorne durch die Motorplatte (links durch das Langloch)
    mx, mz = sh.MOTOR_AX
    for side, dx in (("rechts", gh.MOTOR_EAR_DX), ("links", -gh.MOTOR_EAR_DX)):
        out.append((f"schraube_M3x6_motor_{side}",
                    schraube((mx + dx, yw - gh.MOTOR_PLATE_T, mz), (0, 1, 0), 6)))
    # Platinen-Passschrauben: oben Mutter im Platinenhalter, unten im Goliath-Träger
    for i, (x, z) in enumerate(gh.PCB_INSERTS + [gtr.PCB_SCREW], 1):
        out.append((f"einschmelzmutter_platine_{i}", einschmelzmutter((x, gh.Y_LID_FRONT, z), (0, 1, 0))))
    # Laschen des Platinenhalters: M3x8 von oben, Einschmelzmutter von unten im Goliath-Träger
    for i, (x, y) in enumerate(gh.laschen(), 1):
        out.append((f"schraube_M3x8_lasche_{i}", schraube((x, y, gh.Z_BOT + gh.LUG_T), (0, 0, -1), 8)))
        out.append((f"einschmelzmutter_lasche_{i}", einschmelzmutter((x, y, gtr.Z_BOT), (0, 0, 1))))
    # 2515-Lüfter: 2x M3x12 von außen durch die Keile in die oberen Lüfterbohrungen
    for i, (y, z) in enumerate(gtr.FAN_HOLES, 1):
        out.append((f"schraube_M3x12_luefter_{i}", schraube((gtr.X_L, y, z), (1, 0, 0), 12, (5.5, 3.0, 2.8))))
    # Goliath: 4x M2,5x5 von oben (2,6 mm im Gewinde, Kernloch 3,4 mm tief), Kopf in der Senkung des Trägers
    for i, deg in enumerate(gtr.GOL_SCREWS_DEG, 1):
        x = b.AX_X + gtr.GOL_SCREW_R * math.cos(math.radians(deg))
        y = b.AX_Y + gtr.GOL_SCREW_R * math.sin(math.radians(deg))
        out.append((f"schraube_M2_5x5_goliath_{i}",
                    schraube((x, y, gtr.Z_TOP - gtr.HEAD_DEPTH), (0, 0, -1), 5, M25)))
    return out


teile = build()
