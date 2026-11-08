"""
Hauptkörper für den Sherpa-Umbau (Zwischenstand).

Das Sherpa-Originalgehäuse ist entfernt, ein neues Gehäuse folgt. Bis dahin
besteht der Hauptkörper aus der vereinfachten Install-Plate (install_plate.py)
mit dem Durchbruch für den Motor (LDO 36STH17), dessen Lage aus sherpa.py kommt.
"""
from build123d import Align, Cylinder, Pos, Rot, export_step, export_stl

import basis as b
import sherpa as sh
import install_plate as ip

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
CLR = 0.4
MX, MZ = sh.MOTOR_AX
MOTOR_R = 18.25 + CLR

# Für den Platinenhalter (Lage der bisherigen Sherpa-Kernvorderseite)
PCB_INSERTS = [(-356.0, -134.0), (-315.0, -143.0)]
CORE_FRONT_Y = sh.Y0 - 17.5


def build():
    body = ip.plate
    body -= Pos(MX, sh.MOTOR_FACE_Y, MZ) * Rot(-90, 0, 0) * Cylinder(MOTOR_R, 60, align=MIN)
    # Motorkontur mit Luft (Befestigungslaschen liegen auf Plattenhöhe)
    motor = sh.get("motor")[0]
    for d in ((0, 0, 0), (CLR, 0, 0), (-CLR, 0, 0), (0, CLR, 0), (0, -CLR, 0), (0, 0, CLR), (0, 0, -CLR)):
        body -= Pos(*d) * motor
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
    return max(body.clean().solids(), key=lambda s: s.volume)


body = build()

if __name__ == "__main__":
    b.OUT.mkdir(exist_ok=True)
    body.label = "extruder_body_sherpa"
    export_step(body, str(b.OUT / "extruder_body_sherpa.step"))
    export_stl(body, str(b.OUT / "extruder_body_sherpa.stl"), tolerance=0.01, angular_tolerance=0.1)
    print(f"Volumen {body.volume:.0f} mm^3, Flächen {len(body.faces())}")
