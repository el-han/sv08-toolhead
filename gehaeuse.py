"""
Neues Sherpa-Gehäuse: Stiftaufnahme am Hauptkörper + Platinenhalter (2 mm Platte, trägt vordere Lagerstelle und Idlerstift).

FDM-Konzept: Der Hauptkörper wird mit der Rückseite (y=Y_BACK) auf dem Bett gedruckt. Die
Stiftaufnahme wächst senkrecht nach vorne, die Bohrung entsteht als stehendes Loch.
Der Deckel wird flach mit der Innenseite (y=Y_LID_BACK) nach oben gedruckt; Lagersitz und
Stiftbohrung sind dort ebenfalls stehende Löcher. Die Platinen schrauben auf die Deckel-Vorderseite.

Bohrungsmaße wie im Original-Sherpa (gemessen im STEP):
  MR85 vorne   Ø8.2  y 218.98..221.55, davor Ø7.0 bis 218.25
  MR85 hinten  Ø8.2  y 243.50..245.85, dahinter Ø6.6 bis 246.05
  Idler-Stift  Ø3.3  vorne y 219.1..221.25 (Deckel), hinten y 235.35..238.75 (Zylinder am Hauptkörper)
  Spanneinsatz M3x5x4  Ø4.8 in X, x -333.4..-328.8 (Rändelschraube von +X durch den Arm)
"""
from functools import lru_cache

from build123d import Align, Box, Cylinder, Pos, Rot, export_step

import basis as b
import goliath_traeger as gtr
import sherpa as sh

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
CLR = 0.4

Y_LID_FRONT = 214.2                 # knapp hinter dem Platinenträger (y=214)
Y_LID_BACK = 221.55                 # Trennfuge Deckel / Rahmen (wie Original)
Y_FRAME_BACK = 242.3                # Rahmen reicht bis in die Platte (Vorderfläche 242.27)
X_L = -355.5
Z_BOT, Z_TOP = sh.Z_HOUSING_BOT, -116.5

SHAFT = (-340.0, -136.32 + sh.LIFT)           # Antriebswelle (x, z)
PIN = (-326.9, -146.12 + sh.LIFT)
IDLER = (-331.4, -136.32 + sh.LIFT)           # Idler-Welle (x, z)             # Idler-Drehachse (x, z)
IDLER_SWING = (-2.0, 0.0, 6.0, 12.0)  # Schwenkwinkel des Idlers (Grad, von der Filamentachse weg)
FIL_D, PTFE_D, PTFE_DEPTH = 2.2, 4.15, 8.0

INSERT_D, INSERT_L = 4.0, 5.5
# Verdickungen um Lagersitz, Stiftbohrung und Platineneinsatz (x, z, r)
LID_PADS = [(*SHAFT, 4.1 + 1.0), (*PIN, 1.65 + 1.0), (-330.2, -128.7, INSERT_D / 2 + 1.0)]   # 1 mm Wand
LID_T = 2.0                         # Plattenstärke Platinenhalter
PCB_RECT = (-364.5, -308.5, -161.2, -126.2)   # Umriss der Platinen (x0, x1, z0, z1)
# Hintere Stiftaufnahme auf dem Hauptkörper
PIN_BOSS_R, PIN_BOSS_Y0 = 1.65 + 1.0, 235.35   # 1 mm Wandstärke
PCB_INSERTS = [(-330.2, -128.7)]    # Platinen-Passschraube im Deckel (die untere sitzt im Träger)


def ycyl(x, z, r, y0, y1):
    return Pos(x, y0, z) * Rot(-90, 0, 0) * Cylinder(r, y1 - y0, align=MIN)


def turn(deg):
    """Drehung um die Idler-Achse; positiv = von vorne gesehen im Uhrzeigersinn (Idler nach +X)."""
    px, pz = PIN
    return Pos(px, 0, pz) * Rot(0, deg, 0) * Pos(-px, 0, -pz)


def idler_sweep():
    """Bewegte Idler-Teile über den Schwenkbereich. Luft: -2° auf der Filamentseite, ±CLR in Y.
    (Kopien werden einzeln abgezogen, ein Fuse aller Kopien braucht zu viel RAM.)"""
    # Idler-Rad (Verzahnung, 457 Flächen) und Welle als einfache Zylinder, Nadellager liegen im Rad
    ix, iz = IDLER
    parts = sh.get("idler_arm_short") + [ycyl(ix, iz, 4.75, 222.05, 236.05), ycyl(ix, iz, 1.5, 218.45, 238.45)]
    poses = [turn(deg) for deg in IDLER_SWING] + [Pos(0, CLR, 0), Pos(0, -CLR, 0)]
    return [loc * p for loc in poses for p in parts]


@lru_cache(None)
def freiraum():
    """Alles, was aus Rahmen und Deckel ausgespart wird (Liste von Körpern)."""
    sx, sz = SHAFT
    cut = [
        ycyl(sx, sz, 5.5, Y_LID_BACK - 0.1, 236.6),          # Antriebsrad, Passscheibe
        ycyl(sx, sz, 13.6, 235.6, Y_FRAME_BACK + 1.5),         # 50er-Zahnrad
        ycyl(*sh.MOTOR_AX, 4.5, 235.6, Y_FRAME_BACK + 1),      # Ritzel
        # Lagersitze
        ycyl(sx, sz, 8.2 / 2, 218.98, Y_LID_BACK + 0.3),
        ycyl(sx, sz, 7.0 / 2, 218.25, 219.0),
        ycyl(sx, sz, 8.2 / 2, 243.0, 245.85),
        ycyl(sx, sz, 6.6 / 2, 245.8, 246.05),
        # Idler-Stift
        ycyl(*PIN, 3.3 / 2, 218.7, Y_LID_BACK + 0.1),
        ycyl(*PIN, 3.3 / 2, 235.35, 238.9),
        ycyl(*PIN, 3.3 / 2, 221.0, 235.4),                     # Stift zwischen den Lagerstellen
    ]
    # Filament: PTFE-Bohrung von oben, Führung bis zum Träger
    fx, fy = b.AX_X, b.AX_Y
    cut.append(Pos(fx, fy, Z_BOT - 1) * Cylinder(FIL_D / 2, Z_TOP - Z_BOT + 2, align=MIN))
    cut.append(Pos(fx, fy, Z_TOP - PTFE_DEPTH) * Cylinder(PTFE_D / 2, PTFE_DEPTH + 1, align=MIN))
    # Spanneinsatz für die Rändelschraube (in X, von +X eingesetzt) + Schraubenkanal
    cut.append(Pos(-333.4, 228.8, -126.57 + sh.LIFT) * Rot(0, 90, 0) * Cylinder(4.8 / 2, 7.4, align=MIN))
    cut.append(Pos(-333.4, 228.8, -126.57 + sh.LIFT) * Rot(0, 90, 0) * Cylinder(3.4 / 2, 12.0, align=MIN))
    cut += idler_sweep()
    return tuple(cut)


def scharnier():
    """Hintere Idler-Stiftaufnahme: Zylinder auf dem Hauptkörper (Achse in Y) mit Steg zum Träger."""
    px, pz = PIN
    c = ycyl(px, pz, PIN_BOSS_R, PIN_BOSS_Y0, Y_FRAME_BACK)
    c += Pos(px, (PIN_BOSS_Y0 + Y_FRAME_BACK) / 2, (pz + Z_BOT) / 2) * Box(
        2 * PIN_BOSS_R, Y_FRAME_BACK - PIN_BOSS_Y0, pz - Z_BOT)
    return c


def build_deckel():
    """Platinenhalter: 2 mm Platte in Platinengröße, Verdickungen an Lager, Idlerstift und Platineneinsatz."""
    x0, x1, z0, z1 = PCB_RECT
    d = Pos((x0 + x1) / 2, Y_LID_FRONT, (z0 + z1) / 2) * Box(
        x1 - x0, LID_T, z1 - z0, align=(Align.CENTER, Align.MIN, Align.CENTER))
    for x, z, r in LID_PADS:
        d += ycyl(x, z, r, Y_LID_FRONT, Y_LID_BACK)
    # Aussparungen: Goliath-Träger (samt Lüfterohr), 2515-Lüfter und Goliath, je mit Luft
    for bb in (gtr.traeger.bounding_box(), gtr_fan_bbox(), gtr.gol.bounding_box()):
        d -= Pos(bb.center().X, Y_LID_FRONT + 5, bb.center().Z) * Box(
            bb.size.X + 2 * CLR, 20, bb.size.Z + 2 * CLR)
    for c in freiraum():
        cb = c.bounding_box()
        if cb.min.Y < Y_LID_BACK and cb.max.X > X_L and cb.max.Y > Y_LID_FRONT:
            d -= c
    for x, z in PCB_INSERTS:
        d -= ycyl(x, z, INSERT_D / 2, Y_LID_FRONT - 0.1, Y_LID_FRONT + INSERT_L)
    return max(d.clean().solids(), key=lambda s: s.volume)


def gtr_fan_bbox():
    """Lage des (verschobenen, gespiegelten) 2515-Lüfters, aus dem SV08-Modell."""
    fan = next(p for l, p in b.sv08_parts() if "2515_pipe_cooling_fan" in l)
    return (Pos(gtr.FAN_SHIFT_X, 0, gtr.FAN_DZ) * fan).mirror(gtr.FAN_MIRROR).bounding_box()


deckel = build_deckel()

if __name__ == "__main__":
    b.OUT.mkdir(exist_ok=True)
    deckel.label = "deckel"
    export_step(deckel, str(b.OUT / "deckel.step"))
    print(f"Deckel: Volumen {deckel.volume:.0f} mm^3, Flächen {len(deckel.faces())}")
