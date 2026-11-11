"""
Schrauben und Einschmelzmuttern der Neuteile für Assembly und Kollisionscheck.

Als Modelle dienen die Kaufteile aus den Original-STEPs: Innensechskantschrauben DIN 912 aus dem SV08
(M3x12 und M2,5x8, auf die nötige Länge gekürzt) und die Einschmelzmutter M3x5x4 aus dem Sherpa.
Der Presssitz der Muttern in ihrer Bohrung ist gewollt (check.py ignoriert "einschmelzmutter").
"""
import math
from functools import lru_cache

from build123d import Align, Box, Pos, Rot

import basis as b
import gehaeuse as gh
import goliath_traeger as gtr
import sherpa as sh

_TO_Z = [Rot(0, -90, 0), Rot(90, 0, 0), Rot(0, 0, 0)]       # Achse x / y / z -> +z


def _radius_at(shape, z):
    sl = shape & (Pos(0, 0, z) * Box(40, 40, 0.05))
    if not (sl and sl.solids()):
        return 0.0
    q = sl.bounding_box()
    return max(q.size.X, q.size.Y) / 2


def _canon(part, axis_longest=True):
    """Teil auf die z-Achse legen (Mitte im Ursprung), Achse = längste bzw. kürzeste Ausdehnung."""
    q = part.bounding_box()
    sz = [q.size.X, q.size.Y, q.size.Z]
    ax = sz.index(max(sz) if axis_longest else min(sz))
    return _TO_Z[ax] * (Pos(-q.center().X, -q.center().Y, -q.center().Z) * part), sz[ax]


@lru_cache(None)
def _screw_template(kind):
    """Schraube mit Kopfauflage bei z=0, Kopf bei z<0, Schaft entlang +z."""
    label, total, head_d = {"M3": ("hex_socket_cylindrical_head_screws", 15.0, 5.5),
                            "M2.5": ("hex_socket_cylindrical_head_screws", 10.5, 4.5)}[kind]
    part = next(p for l, p in b.sv08_parts() if label in l
                and abs(max(p.bounding_box().size.X, p.bounding_box().size.Y, p.bounding_box().size.Z) - total) < 0.05
                and abs(sorted([p.bounding_box().size.X, p.bounding_box().size.Y, p.bounding_box().size.Z])[1] - head_d) < 0.05)
    c, ln = _canon(part)
    if _radius_at(c, ln / 2 - 0.3) > _radius_at(c, -ln / 2 + 0.3):     # Kopf nach unten drehen
        c = Rot(180, 0, 0) * c
    z = -ln / 2
    while _radius_at(c, z + 0.1) > head_d / 2 - 0.4:                   # Kopfauflage suchen
        z += 0.1
    return Pos(0, 0, -(z + 0.1)) * c, ln - (z + 0.1 + ln / 2)


@lru_cache(None)
def _insert_template():
    """Einschmelzmutter M3x5x4 (Sherpa) von z=0 bis z=4."""
    c, ln = _canon(sh.idler_insert(), axis_longest=False)
    return Pos(0, 0, ln / 2) * c


def _to_dir(d):
    """Rotation, die +z auf die Richtung d dreht."""
    return {(1, 0, 0): Rot(0, 90, 0), (-1, 0, 0): Rot(0, -90, 0),
            (0, 1, 0): Rot(-90, 0, 0), (0, -1, 0): Rot(90, 0, 0),
            (0, 0, 1): Rot(0, 0, 0), (0, 0, -1): Rot(180, 0, 0)}[d]


def schraube(p, d, length, kind="M3"):
    """Innensechskantschraube mit Kopfauflage im Punkt p, Schaft in Richtung d, Schaftlänge length."""
    t, ln = _screw_template(kind)
    assert length <= ln + 0.15, (kind, length, ln)         # Kopfauflage auf 0,1 mm genau gesucht
    t = t - Pos(0, 0, length) * Box(40, 40, 40, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return Pos(*p) * _to_dir(d) * t


def einschmelzmutter(p, d):
    """Einschmelzmutter M3x5x4, beginnend an der Bohrungsöffnung p, in Richtung d in das Bauteil hinein."""
    return Pos(*p) * _to_dir(d) * _insert_template()


def build():
    out = []
    yb, yw = gh.Y_BEAM_FRONT, sh.MOTOR_FACE_Y
    # Platinenhalter <-> Motorplatte: Mutter in der Stütze, Schraube von der Motorseite
    for i, (x, z) in enumerate(gh.schrauben(), 1):
        out.append((f"einschmelzmutter_stuetze_{i}", einschmelzmutter((x, yb, z), (0, -1, 0))))
        out.append((f"schraube_M3x12_stuetze_{i}", schraube((x, yw, z), (0, -1, 0), 12)))
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
        out.append((f"schraube_M3x12_luefter_{i}", schraube((gtr.X_L, y, z), (1, 0, 0), 12)))
    # Goliath: 4x M2,5x5 von oben (2,6 mm im Gewinde, Kernloch 3,4 mm tief), Kopf in der Senkung des Trägers
    for i, deg in enumerate(gtr.GOL_SCREWS_DEG, 1):
        x = b.AX_X + gtr.GOL_SCREW_R * math.cos(math.radians(deg))
        y = b.AX_Y + gtr.GOL_SCREW_R * math.sin(math.radians(deg))
        out.append((f"schraube_M2_5x5_goliath_{i}",
                    schraube((x, y, gtr.Z_TOP - gtr.HEAD_DEPTH), (0, 0, -1), 5, "M2.5")))
    return out


teile = build()
