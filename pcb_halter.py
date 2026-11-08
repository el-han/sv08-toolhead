"""
Platinenhalter für den Sherpa-Umbau.

Die originalen Platinenträger (SV05 extruder_adapter_plate top/lower) wurden mit
2x M3-Passschraube ins Getriebegehäuse geschraubt. Der Sherpa-Deckel steht
weiter vor, deshalb sitzen die Platinen jetzt PCB_SHIFT weiter vorne auf
diesem Halter. Er bietet die beiden Original-Schraubpunkte (M3-Einsätze) und
wird links und rechts neben dem Deckel mit 2x M3 an den Gehäusekern geschraubt.
Zum Öffnen des Sherpa-Deckels Platinen abnehmen.
"""
from build123d import Align, Box, Cylinder, Pos, Rot, export_step, export_stl

import basis as b
import sherpa as sh
import extruder_body as eb

MIN = (Align.CENTER, Align.CENTER, Align.MIN)

PCB_SHIFT = (0.0, -10.0, 0.0)
PCB_BACK_Y = 214.0 + PCB_SHIFT[1]                    # Rückseite lower_plate
BACK_Y = 208.75                                      # vor den Köpfen der Deckelschrauben
SCREWS = [(-330.2, -128.7), (-336.5, -153.2)]        # Original-Schraubpunkte (x, z)
X_RNG, Z_RNG = (-360.5, -310.5), (-156.5, -124.5)
INSERT_D = 4.0
M3_D, HEAD_D, HEAD_T = 3.3, 6.0, 2.0


def build():
    t = BACK_Y - PCB_BACK_Y
    h = Pos(sum(X_RNG) / 2, (PCB_BACK_Y + BACK_Y) / 2, sum(Z_RNG) / 2) * Box(
        X_RNG[1] - X_RNG[0], t, Z_RNG[1] - Z_RNG[0])
    # Abstandshalter zur Kernvorderseite
    for x, z in eb.PCB_INSERTS:
        h += Pos(x, BACK_Y - 0.01, z) * Rot(-90, 0, 0) * Cylinder(4.0, eb.CORE_FRONT_Y - BACK_Y, align=MIN)
    # Fenster in der Mitte (Gewicht, Luft), Ränder und Schraubpunkte bleiben
    h -= Pos(-345.0, (PCB_BACK_Y + BACK_Y) / 2, -141.0) * Box(14.0, 20, 20.0)
    # Schraubpunkte der Platinen: Einsätze von vorne
    for x, z in SCREWS:
        h -= Pos(x, PCB_BACK_Y - 0.01, z) * Rot(-90, 0, 0) * Cylinder(INSERT_D / 2, t + 0.02, align=MIN)
    # Befestigung am Kern: Durchgang + Kopfsenkung vorne
    for x, z in eb.PCB_INSERTS:
        h -= Pos(x, PCB_BACK_Y - 1, z) * Rot(-90, 0, 0) * Cylinder(M3_D / 2, 30, align=MIN)
        h -= Pos(x, PCB_BACK_Y - 1, z) * Rot(-90, 0, 0) * Cylinder(HEAD_D / 2, HEAD_T + 1, align=MIN)
    # Freiraum zu Sherpa-Teilen (Deckelschrauben, Idler-Arm, Rändelschraube)
    for n, p in sh.parts:
        if n in sh.UNUSED or n == "motor":
            continue
        pb = p.bounding_box()
        if pb.min.Y < eb.CORE_FRONT_Y + 0.5 and n not in ("housing_core", "housing_rear"):
            g = eb.CLR
            for d in ((0, 0, 0), (g, 0, 0), (-g, 0, 0), (0, g, 0), (0, -g, 0), (0, 0, g), (0, 0, -g)):
                try:
                    h -= Pos(*d) * p
                except ValueError:
                    print(f"  (Freiraum {n} Versatz {d} übersprungen)")
    return h.clean()


halter = build()

if __name__ == "__main__":
    halter.label = "pcb_halter"
    export_step(halter, str(b.OUT / "pcb_halter.step"))
    export_stl(halter, str(b.OUT / "pcb_halter.stl"), tolerance=0.01, angular_tolerance=0.1)
    print(f"Volumen {halter.volume:.0f} mm^3, Solids {len(halter.solids())}, Dicke {BACK_Y - PCB_BACK_Y:.2f} mm")
