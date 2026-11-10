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
WALL = 1.5                          # Wandstärke der Zylinder
LID_PADS = [(*SHAFT, 4.1 + WALL), (*PIN, 1.65 + WALL), (-330.2, -128.7, INSERT_D / 2 + WALL)]
LID_T = 3.0                         # Plattenstärke Platinenhalter
PCB_RECT = (-364.5, -308.5, -161.2, -126.2)   # Umriss der Platinen (x0, x1, z0, z1)
# Motorplatte
MOTOR_PLATE_T = 2.0
MOTOR_EAR_DX, MOTOR_EAR_R = 21.92, 3.5   # Lochabstand der Motorlaschen, Laschenradius
MOTOR_BOSS_R = 8.0 + CLR                 # Ritzel + Zentrierbund
SLOT_HALF = 1.5                          # halbe Langlochlänge (Mittenabstand ±1.5 mm)
# Hintere Stiftaufnahme auf dem Hauptkörper
PIN_BOSS_R, PIN_BOSS_Y0 = 1.65 + WALL, 235.35
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
    """Hintere Idler-Stiftaufnahme: Zylinder (Achse in Y) vorne an der Motorplatte."""
    px, pz = PIN
    return ycyl(px, pz, PIN_BOSS_R, PIN_BOSS_Y0, sh.MOTOR_FACE_Y - MOTOR_PLATE_T + 0.01)


def build_deckel():
    """Platinenhalter: 3 mm Platte in Platinengröße, Verdickungen an Lager, Idlerstift und Platineneinsatz."""
    x0, x1, z0, z1 = PCB_RECT
    d = Pos((x0 + x1) / 2, Y_LID_FRONT, (z0 + z1) / 2) * Box(
        x1 - x0, LID_T, z1 - z0, align=(Align.CENTER, Align.MIN, Align.CENTER))
    for x, z, r in LID_PADS:
        d += ycyl(x, z, r, Y_LID_FRONT, Y_LID_BACK)
    # Halter endet an der Oberkante des Goliath-Trägers (darunter trägt der Goliath-Träger die Platine)
    d -= Pos(0, 0, Z_BOT) * Box(2000, 2000, 100, align=(Align.CENTER, Align.CENTER, Align.MAX))
    for c in freiraum():
        cb = c.bounding_box()
        if cb.min.Y < Y_LID_BACK and cb.max.X > X_L and cb.max.Y > Y_LID_FRONT:
            d -= c
    for x, z in PCB_INSERTS:
        d -= ycyl(x, z, INSERT_D / 2, Y_LID_FRONT - 0.1, Y_LID_FRONT + INSERT_L)
    return max(d.clean().solids(), key=lambda s: s.volume)


def motorplatte():
    """2 mm Platte vor der Motor-Anlagefläche bis auf den Hauptträger (eigenes Teil): Durchgang für Ritzel/Zentrierbund,
    hinterer Lagersitz (über freiraum), zwei M3-Löcher in den Motorlaschen, links (von vorne) als Langloch."""
    mx, mz = sh.MOTOR_AX
    y1 = sh.MOTOR_FACE_Y
    y0 = y1 - MOTOR_PLATE_T
    x0, x1 = mx - MOTOR_EAR_DX - MOTOR_EAR_R, mx + MOTOR_EAR_DX + MOTOR_EAR_R
    z1 = sh.get("motor")[0].bounding_box().max.Z
    p = Pos((x0 + x1) / 2, (y0 + y1) / 2, (Z_BOT + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - Z_BOT)
    # Durchgang Ritzel/Zentrierbund als Langloch: Schwenken um die rechte Schraube verschiebt die Achse
    # um SLOT_HALF * (Abstand Achse-Drehpunkt) / (Abstand der Schrauben) = SLOT_HALF / 2 senkrecht
    dz = SLOT_HALF / 2
    p -= ycyl(mx, mz - dz, MOTOR_BOSS_R, y0 - 1, y1 + 1) + ycyl(mx, mz + dz, MOTOR_BOSS_R, y0 - 1, y1 + 1)
    p -= Pos(mx, (y0 + y1) / 2, mz) * Box(2 * MOTOR_BOSS_R, MOTOR_PLATE_T + 2, 2 * dz)
    p -= ycyl(mx + MOTOR_EAR_DX, mz, 1.7, y0 - 1, y1 + 1)
    xl = mx - MOTOR_EAR_DX                              # Langloch, vertikal
    p -= ycyl(xl, mz - SLOT_HALF, 1.7, y0 - 1, y1 + 1) + ycyl(xl, mz + SLOT_HALF, 1.7, y0 - 1, y1 + 1)
    p -= Pos(xl, y0 + MOTOR_PLATE_T / 2, mz) * Box(3.4, MOTOR_PLATE_T + 2, 2 * SLOT_HALF)
    return p


def gtr_fan_bbox():
    """Lage des (verschobenen, gespiegelten) 2515-Lüfters, aus dem SV08-Modell."""
    fan = next(p for l, p in b.sv08_parts() if "2515_pipe_cooling_fan" in l)
    return (Pos(gtr.FAN_SHIFT_X, 0, gtr.FAN_DZ) * fan).mirror(gtr.FAN_MIRROR).bounding_box()


deckel = build_deckel()


def build_motorplatte():
    """Motorplatte als eigenes Teil (steht auf dem Hauptträger), mit hinterem Lagersitz und Idler-Stiftaufnahme."""
    p = motorplatte() + scharnier()
    y0 = PIN_BOSS_Y0
    for c in freiraum():
        cb = c.bounding_box()
        if cb.max.Y > y0 and cb.min.Y < sh.MOTOR_FACE_Y:
            p -= c
    return max(p.clean().solids(), key=lambda s: s.volume)


motorplatte_teil = build_motorplatte()

if __name__ == "__main__":
    b.OUT.mkdir(exist_ok=True)
    deckel.label = "deckel"
    export_step(deckel, str(b.OUT / "deckel.step"))
    print(f"Deckel: Volumen {deckel.volume:.0f} mm^3, Flächen {len(deckel.faces())}")
