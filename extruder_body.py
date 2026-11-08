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
    return max(body.clean().solids(), key=lambda s: s.volume)


body = build()

if __name__ == "__main__":
    b.OUT.mkdir(exist_ok=True)
    body.label = "extruder_body_sherpa"
    export_step(body, str(b.OUT / "extruder_body_sherpa.step"))
    export_stl(body, str(b.OUT / "extruder_body_sherpa.stl"), tolerance=0.01, angular_tolerance=0.1)
    print(f"Volumen {body.volume:.0f} mm^3, Flächen {len(body.faces())}")
