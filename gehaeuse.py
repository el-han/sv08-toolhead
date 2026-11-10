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
KNOB_BOT = -126.57 + sh.LIFT - 6.0      # Unterkante des Rändelkopfs (Ø12)
POST_INSET = 8.0                    # Abstand der Stützen von den Seitenkanten
Y_BEAM_BACK = 243.25 - 3.0 - 0.8       # 0,8 mm vor dem Zahnkranz des 50er-Rads (3 mm stark)
Y_BEAM_FRONT = Y_BEAM_BACK - 2.0         # Querverbindungen 2 mm tief
Z_BEAM_TOP = SHAFT[1] + 6.3 + CLR        # obere Querverbindung über Idler-Arm und Zahnradnabe
POST = 5.0                          # Querschnitt der Eckstützen (quadratisch)
LID_T = 3.0                         # Plattenstärke Platinenhalter
X_FULL = (-367.548, -305.501)                  # volle Breite des Hauptträgers
PCB_RECT = (*X_FULL, -161.2, -126.2)           # Platinenhalter: volle Breite, Höhe der Platinen (x0, x1, z0, z1)
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
        ycyl(sx, sz, 13.6, 240.25 - CLR, Y_FRAME_BACK + 1.5),   # 50er-Zahnrad, Zahnkranz (3 mm)
        ycyl(sx, sz, 6.3 + CLR, 235.6, 240.3),                # 50er-Zahnrad, Nabe (max. Ø12,6)
        ycyl(*sh.MOTOR_AX, 4.5, 240.05 - CLR, Y_FRAME_BACK + 1),   # Ritzel (beginnt bei y=240.05)
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


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def stuetzen():
    """Vier Stützen im Platinenhalter (x-Mitte, z0, z1): unten links/rechts, oben links bündig mit der
    Oberkante des Platinenhalters, oben rechts unter dem Rändelkopf."""
    x0, x1, _, z1 = PCB_RECT
    xl = sh.MOTOR_AX[0] - MOTOR_EAR_DX         # links: mittig unter dem Langloch der Motorschraube
    xr = x1 - POST_INSET - POST / 2
    zt1 = KNOB_BOT - 0.4
    return [(xl, Z_BOT, Z_BOT + POST), (xr, Z_BOT, Z_BOT + POST),
            (xl, z1 - POST, z1), (xr, zt1 - POST, zt1)]


def querverbindungen():
    """Stützen ab Y_BEAM_FRONT bis zur Motorplatte und die waagrechten Verbindungen oben und unten."""
    yw = sh.MOTOR_FACE_Y - MOTOR_PLATE_T
    st = stuetzen()
    # links und rechts je eine durchgehende Strebe von unten bis zur oberen Querverbindung
    # die linke reicht über das Langloch der Motorschraube hinaus
    z_slot_top = sh.MOTOR_AX[1] + SLOT_HALF + 1.7
    out = [box(x - POST / 2, x + POST / 2, Y_BEAM_FRONT, yw + 0.01, Z_BOT, zt)
           for x, zt in ((st[0][0], z_slot_top + 2.0), (st[1][0], Z_BEAM_TOP + POST))]
    xl, xr = st[0][0], st[1][0]
    for za, zb in ((Z_BOT, Z_BOT + POST), (Z_BEAM_TOP, Z_BEAM_TOP + POST)):
        out.append(box(xl - POST / 2, xr + POST / 2, Y_BEAM_FRONT, Y_BEAM_BACK, za, zb))
    r = out[0]
    for o in out[1:]:
        r += o
    return r


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
    # Stützen bis zum Beginn der Querverbindungen (der Rest gehört zur Motorplatte)
    for x, za, zb in stuetzen():
        d += box(x - POST / 2, x + POST / 2, Y_LID_FRONT + LID_T - 0.01, Y_BEAM_FRONT, za, zb)
    for x, z in PCB_INSERTS:
        d -= ycyl(x, z, INSERT_D / 2, Y_LID_FRONT - 0.1, Y_LID_FRONT + INSERT_L)
    return max(d.clean().solids(), key=lambda s: s.volume)


def motorplatte():
    """2 mm Platte vor der Motor-Anlagefläche bis auf den Hauptträger (eigenes Teil): Durchgang für Ritzel/Zentrierbund,
    hinterer Lagersitz (über freiraum), zwei M3-Löcher in den Motorlaschen, links (von vorne) als Langloch."""
    mx, mz = sh.MOTOR_AX
    y1 = sh.MOTOR_FACE_Y
    y0 = y1 - MOTOR_PLATE_T
    x0, x1 = X_FULL
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


def motor_langloch(y0, y1):
    mx, mz = sh.MOTOR_AX
    xl = mx - MOTOR_EAR_DX
    return (ycyl(xl, mz - SLOT_HALF, 1.7, y0, y1) + ycyl(xl, mz + SLOT_HALF, 1.7, y0, y1)
            + Pos(xl, (y0 + y1) / 2, mz) * Box(3.4, y1 - y0, 2 * SLOT_HALF))


def gtr_fan_bbox():
    """Lage des (verschobenen, gespiegelten) 2515-Lüfters, aus dem SV08-Modell."""
    fan = next(p for l, p in b.sv08_parts() if "2515_pipe_cooling_fan" in l)
    return (Pos(gtr.FAN_SHIFT_X, 0, gtr.FAN_DZ) * fan).mirror(gtr.FAN_MIRROR).bounding_box()


deckel = build_deckel()


def build_motorplatte():
    """Motorplatte als eigenes Teil (steht auf dem Hauptträger), mit hinterem Lagersitz und Idler-Stiftaufnahme."""
    p = motorplatte() + scharnier() + querverbindungen()
    p -= motor_langloch(Y_BEAM_FRONT - 1, sh.MOTOR_FACE_Y + 1)      # Langloch auch durch die linke Strebe
    y0 = PIN_BOSS_Y0
    for c in freiraum():
        cb = c.bounding_box()
        if cb.max.Y > y0 and cb.min.Y < sh.MOTOR_FACE_Y:
            p -= c
    return max(p.clean().solids(), key=lambda s: s.volume)


motorplatte_teil = build_motorplatte()


# Filamentführungen ober- und unterhalb der Antriebsräder, Maße wie im Original-Sherpa-Gehäuse
# (dort gemessen relativ zur Antriebswelle, z-Versatz zur Wellenachse)
GUIDE_D, BORE_D = 2.1, 4.2            # Führung / Aufnahme (PTFE bzw. Bohrung)
GUIDE_WALL = 1.5
UPPER = dict(top=18.0, bore_bot=5.32, bot=2.42)     # oben: Bohrung bis 5.32, Führung bis 2.42 über der Welle
LOWER = dict(top=-2.38, guide_bot=-5.28)           # unten: Führung bis -5.28, darunter Bohrung bis zum Träger


def zcyl(r, z0, z1):
    return Pos(b.AX_X, b.AX_Y, z0) * Cylinder(r, z1 - z0, align=MIN)


def build_fuehrungen():
    sz = SHAFT[1]
    ro = BORE_D / 2 + GUIDE_WALL
    up = zcyl(ro, sz + UPPER["bot"], sz + UPPER["top"])
    up -= zcyl(GUIDE_D / 2, sz + UPPER["bot"] - 1, sz + UPPER["top"] + 1)
    up -= zcyl(BORE_D / 2, sz + UPPER["bore_bot"], sz + UPPER["top"] + 1)
    lo = zcyl(ro, Z_BOT, sz + LOWER["top"])
    lo -= zcyl(GUIDE_D / 2, Z_BOT - 1, sz + LOWER["top"] + 1)
    lo -= zcyl(BORE_D / 2, Z_BOT - 1, sz + LOWER["guide_bot"])
    # Freiräume für Antriebsrad und Idler (samt Schwenkbereich) und Platinenhalter aussparen
    for c in freiraum():
        cb = c.bounding_box()
        on_axis = abs(cb.center().X - b.AX_X) < 0.01 and abs(cb.center().Y - b.AX_Y) < 0.01
        if cb.max.Z > Z_BOT and cb.min.Z < sz + UPPER["top"] and not on_axis:
            up -= c
            lo -= c
    up -= deckel
    lo -= deckel
    return up, lo


fuehrung_oben, fuehrung_unten = build_fuehrungen()

if __name__ == "__main__":
    b.OUT.mkdir(exist_ok=True)
    deckel.label = "deckel"
    export_step(deckel, str(b.OUT / "deckel.step"))
    print(f"Deckel: Volumen {deckel.volume:.0f} mm^3, Flächen {len(deckel.faces())}")
