"""
Hauptkörper für den Sherpa-Umbau (Zwischenstand).

Oberteil der vereinfachten Install-Plate (install_plate.py), nach oben verlängert; darunter ein
Quader von der Eddy-Sonde bis zum Goliath-Träger mit allen Bohrungen, Kabelrinne und Aussparungen
für Sliding Panel und Riemenklemmen (Magnete entfallen); mit Motorvertiefung
(LDO 36STH17), Goliath-Träger und der hinteren Idler-Stiftaufnahme aus gehaeuse.py (Deckel separat).
"""
import math

from build123d import (Align, Box, Cylinder, Edge, Face, Pos, Rot, Vector, Wire, extrude,
                       export_step)

import basis as b
import gehaeuse as gh
import goliath_traeger as gtr
import sherpa as sh
import install_plate as ip

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
CLR = 0.4
MX, MZ = sh.MOTOR_AX
MOTOR_R = 18.25 + CLR
EAR_DX = 21.95                 # Abstand Laschenbogen-Mitte zur Motorachse (x)
SCREW_DX, SCREW_R = 21.92, 1.7   # Lochabstand der Motorlaschen, M3-Durchgang
EAR_R = 3.5 + CLR              # Radius der Laschenbögen am Motorhalter
Z_TOP = -101.5                 # Oberkante der Verlängerung (Motor oben bei -104.22)
Z_EXT_BOT = -146.035          # Unterkante der Verlängerung (gerade Seitenkanten der Platte)
Z_PLATE_TOP = -123.151         # Oberkante der Originalplatte


def hull_face(y):
    """Konvexe Hülle aus Motorkreis und den beiden Laschenkreisen (in der Ebene y)."""
    # äußere Tangenten zwischen Kreis (0, R) und (±d, r): Winkel des Tangentenpunkts
    a = math.asin((MOTOR_R - EAR_R) / EAR_DX)
    def pt(cx, r, ang):
        return Vector(MX + cx + r * math.sin(ang), y, MZ + r * math.cos(ang))
    # Winkel gemessen von +z aus, im Uhrzeigersinn Richtung +x
    t = a                                   # Tangentenpunkt oben rechts (Winkel ab +z)
    edges = [
        Edge.make_three_point_arc(pt(0, MOTOR_R, -t), pt(0, MOTOR_R, 0), pt(0, MOTOR_R, t)),
        Edge.make_line(pt(0, MOTOR_R, t), pt(EAR_DX, EAR_R, t)),
        Edge.make_three_point_arc(pt(EAR_DX, EAR_R, t), pt(EAR_DX, EAR_R, math.pi / 2), pt(EAR_DX, EAR_R, math.pi - t)),
        Edge.make_line(pt(EAR_DX, EAR_R, math.pi - t), pt(0, MOTOR_R, math.pi - t)),
        Edge.make_three_point_arc(pt(0, MOTOR_R, math.pi - t), pt(0, MOTOR_R, math.pi), pt(0, MOTOR_R, math.pi + t)),
        Edge.make_line(pt(0, MOTOR_R, math.pi + t), pt(-EAR_DX, EAR_R, math.pi + t)),
        Edge.make_three_point_arc(pt(-EAR_DX, EAR_R, math.pi + t), pt(-EAR_DX, EAR_R, -math.pi / 2), pt(-EAR_DX, EAR_R, -t)),
        Edge.make_line(pt(-EAR_DX, EAR_R, -t), pt(0, MOTOR_R, -t)),
    ]
    return sh.MOTOR_TURN * Face(Wire(edges))


def motor_screw_holes():
    """Durchgangslöcher für die beiden Motorschrauben (M3, durch die Platte von vorne)."""
    out = []
    for dx in (-SCREW_DX, SCREW_DX):
        p = (sh.MOTOR_TURN * Pos(MX + dx, 0, MZ)).position
        out.append(ip.y_hole(p.X, p.Z, SCREW_R, ip.Y_PAD - 1, sh.MOTOR_FACE_Y + 0.1))
    return out



# Unterteil des Hauptträgers als Quader (ersetzt die Install-Plate unterhalb des Goliath-Trägers)
X_PL, X_PR = -367.548, -305.501            # Breite wie bisher (62 mm)
Z_SPLIT = gtr.Z_TOP                        # Oberkante Goliath-Träger
Z_BLOCK_BOT = -222.2                       # Unterkante der Eddy-Sonde
# SV08-Teile, die in der Platte sitzen und eine Aussparung brauchen (Metallaufnahme am Schlitten)
POCKETS = ["xtruder_sliding_panel"]


def offset_cut(body, part, clr=CLR):
    """Teil mit Luft in X und Z ausschneiden (keine Luft in Y, damit die Rückseite plan bleibt)."""
    for d in ((0, 0, 0), (clr, 0, 0), (-clr, 0, 0), (0, 0, clr), (0, 0, -clr)):
        body -= Pos(*d) * part
    return body


def box_cut(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def quader_taschen():
    """Quaderförmige Aussparungen: Riemenklemmen seitlich bis nach außen, Luftführung bis zur Unterkante."""
    out = []
    for p in b.find("belt_lock_block"):
        q = p.bounding_box()
        left = q.center().X < (X_PL + X_PR) / 2
        x0, x1 = (X_PL - 1, q.max.X + CLR) if left else (q.min.X - CLR, X_PR + 1)
        out.append(box_cut(x0, x1, ip.Y_FRONT - 1, ip.Y_BACK + 1, q.min.Z - CLR, q.max.Z + CLR))   # durchgehend
    # Luftführung: nur der Teil, der in der Platte steckt (darüber tragen die Schrauben)
    blk = box_cut(X_PL, X_PR, ip.Y_FRONT, ip.Y_BACK, Z_BLOCK_BOT, Z_SPLIT)
    q = (b.find("air_deflector2")[0] & blk).bounding_box()
    out.append(box_cut(q.min.X - CLR, q.max.X + CLR, ip.Y_FRONT - 1, ip.Y_BACK + 1, Z_BLOCK_BOT - 1, q.max.Z + CLR))
    return out


def block():
    blk = Pos((X_PL + X_PR) / 2, (ip.Y_FRONT + ip.Y_BACK) / 2, (Z_BLOCK_BOT + Z_SPLIT) / 2) * Box(
        X_PR - X_PL, ip.Y_BACK - ip.Y_FRONT, Z_SPLIT - Z_BLOCK_BOT)
    for x, z, r in ip.DURCHGANG.values():
        blk -= ip.y_hole(x, z, r, ip.Y_FRONT - 1, ip.Y_BACK + 1)
    for key in POCKETS:
        for p in b.find(key):
            blk = offset_cut(blk, p)
    for c in quader_taschen():
        blk -= c
    # Schrauben der Nachbarteile, die in den Block ragen (Senkungen/Köpfe)
    bb = blk.bounding_box()
    for label, p in b.sv08_parts():
        if "screw" not in label or any(k in label for k in b.REMOVED_SHERPA) or ip._old_motor_screw(p):
            continue
        q = p.bounding_box()
        if (q.max.Y < bb.min.Y or q.min.Y > bb.max.Y or q.max.X < bb.min.X or q.min.X > bb.max.X
                or q.max.Z < bb.min.Z or q.min.Z > bb.max.Z):
            continue
        if any((s & blk) and sum(x.volume for x in (s & blk).solids()) > 0.01 for s in p.solids()):
            blk -= ip.stepped_cylinder(p, CLR)
    blk -= ip.RINNE                          # Kabelführung
    return blk


def build():
    # Oberteil (Install-Plate-Rest mit Motoraufnahme) entfällt, ein neuer Extruderhalter folgt
    body = block()
    # Goliath-Träger ist direkt mit der Hinterwand verbunden
    body += gtr.traeger
    # Hintere Idler-Stiftaufnahme; Freiräume für 50er-Zahnrad, Ritzel, hinteres Lager und Idler
    body += gh.scharnier()
    for c in gh.freiraum():
        if c.bounding_box().max.Y > gh.Y_LID_BACK:
            body -= c
    return max(body.clean().solids(), key=lambda s: s.volume)


body = build()

if __name__ == "__main__":
    b.OUT.mkdir(exist_ok=True)
    body.label = "extruder_body_sherpa"
    export_step(body, str(b.OUT / "extruder_body_sherpa.step"))
    print(f"Volumen {body.volume:.0f} mm^3, Flächen {len(body.faces())}")
