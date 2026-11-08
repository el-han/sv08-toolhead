"""
Lasche für den 5020-Bauteillüfter (inkl. originaler Luftführung).

Original: Lüfter hängt mit 1x M2,5x8 am Getriebegehäuse (Gewinde dort).
Neu: Die Lasche wird an diesem Gewinde verschraubt, der Lüfter an der Lasche
– um DROP tiefer und um T nach vorne (sonst Kollision mit dem Goliath).
"""
from build123d import Box, Cylinder, Pos, Rot, export_step, export_stl, Align

import goliath_traeger as gt

T = 3.0                       # Laschenstärke = Versatz nach vorne (-Y)
HOLE_X, HOLE_Z = -360.84, -166.92
GC_FACE_Y = 214.67            # Anlagefläche Getriebegehäuse / Lüfter
WIDTH = 9.0
MARGIN = 4.5
BOSS_D, BOSS_L = 6.5, 4.0     # Gewindedom hinter der Lasche
SCREW_D, TAP_D = 2.8, 2.2     # M2,5 Durchgang / selbstschneidend
HEAD_D = 5.0

FAN_SHIFT = (0.0, -T, -gt.DROP)
z_low = HOLE_Z - gt.DROP

def ycyl(r, y0, length):
    return Rot(-90, 0, 0) * Cylinder(r, length, align=(Align.CENTER, Align.CENTER, Align.MIN))

lasche = Pos(HOLE_X, GC_FACE_Y - T, z_low - MARGIN) * Box(
    WIDTH, T, (HOLE_Z - z_low) + 2 * MARGIN, align=(Align.CENTER, Align.MIN, Align.MIN))
lasche += Pos(HOLE_X, GC_FACE_Y, z_low) * ycyl(BOSS_D / 2, 0, BOSS_L)
# oben: Durchgang zum Getriebegehäuse
lasche -= Pos(HOLE_X, GC_FACE_Y - T - 1, HOLE_Z) * ycyl(SCREW_D / 2, 0, T + 2)
# unten: Gewinde für Lüfterschraube
lasche -= Pos(HOLE_X, GC_FACE_Y - T - 1, z_low) * ycyl(TAP_D / 2, 0, T + BOSS_L + 0.5)

if __name__ == "__main__":
    gt.OUT.mkdir(exist_ok=True)
    lasche.label = "fan5020_lasche"
    export_step(lasche, str(gt.OUT / "fan5020_lasche.step"))
    export_stl(lasche, str(gt.OUT / "fan5020_lasche.stl"), tolerance=0.01, angular_tolerance=0.1)
    print("Lasche", lasche.bounding_box().size, f"{lasche.volume:.0f} mm^3")
