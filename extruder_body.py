"""
Hauptkörper für den Sherpa-Umbau (Zwischenstand).

Das Sherpa-Originalgehäuse ist entfernt, ein neues Gehäuse folgt. Bis dahin
besteht der Hauptkörper aus der vereinfachten Install-Plate (install_plate.py)
mit dem Durchbruch für den Motor (LDO 36STH17), dessen Lage aus sherpa.py kommt.
"""
import math

from build123d import (Align, Box, Cylinder, Edge, Face, Pos, Rot, Vector, Wire, extrude,
                       export_step)

import basis as b
import goliath_traeger as gtr
import sherpa as sh
import install_plate as ip

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
CLR = 0.4
MX, MZ = sh.MOTOR_AX
MOTOR_R = 18.25 + CLR
EAR_DX = 21.95                 # Abstand Laschenbogen-Mitte zur Motorachse (x)
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
    return Face(Wire(edges))



def build():
    body = ip.plate
    # Verlängerung nach oben über den Motor hinaus, nur der flache Körper (Y_STEP..Y_BACK)
    # Die Originalkontur der Oberkante (mit Schrägen) wird um dz nach oben verschoben
    dz = Z_TOP - Z_PLATE_TOP
    top = [(x, z + dz) for x, z in ((-367.548, -133.913), (-367.348, -133.513), (-353.666, -123.251),
                                    (-353.366, -123.151), (-319.683, -123.151), (-319.383, -123.251),
                                    (-305.701, -133.513), (-305.501, -133.913))]
    pts = [(-367.548, Z_EXT_BOT)] + top + [(-305.501, Z_EXT_BOT)]
    segs = [("L", *a, *c) for a, c in zip(pts, pts[1:] + pts[:1])]
    body += ip.slab(segs, ip.Y_STEP, ip.Y_BACK)
    for x, z, r in ip.MAGNETE.values():           # Magnetbohrungen durch die Verlängerung erhalten
        body -= ip.y_hole(x, z, r, ip.Y_PAD - 1, ip.Y_BACK + 1)
    # Motorvertiefung: konvexe Hülle aus Motorkörper und Laschenbögen, ab Motor-Anlagefläche nach hinten
    body -= extrude(hull_face(sh.MOTOR_FACE_Y), amount=ip.Y_BACK + 1 - sh.MOTOR_FACE_Y, dir=(0, 1, 0))
    # Ritzel + Zentrierbund des Motors vor der Anlagefläche
    body -= Pos(MX, sh.MOTOR_FACE_Y, MZ) * Rot(90, 0, 0) * Cylinder(8.0 + CLR, 8.0, align=MIN)
    # 50er-Zahnrad und hinteres Lager auf der Antriebswelle (ragen seit dem Umdrehen in die Platte)
    for n in ("zahnrad_50z", "lager_MR85"):
        for p in sh.get(n):
            pb = p.bounding_box()
            if pb.max.Y > 239.0:
                r = max(pb.size.X, pb.size.Z) / 2 + CLR + 0.2
                body -= Pos(pb.center().X, pb.min.Y - CLR, pb.center().Z) * Rot(-90, 0, 0) * Cylinder(
                    r, pb.size.Y + 2 * CLR, align=MIN)
    # Goliath-Träger ist direkt mit der Hinterwand verbunden
    body += gtr.traeger
    return max(body.clean().solids(), key=lambda s: s.volume)


body = build()

if __name__ == "__main__":
    b.OUT.mkdir(exist_ok=True)
    body.label = "extruder_body_sherpa"
    export_step(body, str(b.OUT / "extruder_body_sherpa.step"))
    print(f"Volumen {body.volume:.0f} mm^3, Flächen {len(body.faces())}")
