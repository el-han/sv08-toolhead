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
import math
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
BEARING_FRONT = 243.35                 # Vorderkante des hinteren Lagersitzes (wie Original)
# Verdickungen um Lagersitz, Stiftbohrung und Platineneinsatz (x, z, r)
WALL = 1.5                          # Wandstärke der Zylinder
LID_PADS = [(*SHAFT, 4.1 + WALL), (*PIN, 1.65 + WALL), (-330.2, -128.7 + gtr.PCB_DZ, INSERT_D / 2 + WALL)]
KNOB_BOT = -126.57 + sh.LIFT - 6.0      # Unterkante des Rändelkopfs (Ø12)
Y_BEAM_BACK = 243.25 - 3.0 - 0.8       # 0,8 mm vor dem Zahnkranz des 50er-Rads (3 mm stark)
Y_BEAM_FRONT = Y_BEAM_BACK               # Trennfuge Platinenhalter / Motorplatte in den Streben (M3x12)
LUG_W, LUG_T = 7.0, 3.0                 # Befestigungslaschen: Breite, Dicke
POST = 7.0                          # Querschnitt der Eckstützen (quadratisch)
LID_T = 3.0                         # Plattenstärke Platinenhalter
X_FULL = (-367.548, -305.501)                  # volle Breite des Hauptträgers
POST_TOP_L = -126.2 + gtr.PCB_DZ            # Oberkante der linken Stütze (bisherige Halteroberkante, bleibt)
# Platinenhalter: volle Breite, unten wie die Platinen, oben bündig mit dem Kasten um die Idler-Einschmelzmutter
PCB_RECT = (*X_FULL, -161.2 + gtr.PCB_DZ, sh.idler_insert().bounding_box().center().Z + 4.8 / 2 + 1.5)
# Motorplatte
MOTOR_PLATE_T = 2.0
MOTOR_EAR_DX, MOTOR_EAR_R = 21.92, 3.5   # Lochabstand der Motorlaschen, Laschenradius
MOTOR_BOSS_R = 8.0 + CLR                 # Ritzel + Zentrierbund
SLOT_HALF = 1.5                          # halbe Langlochlänge (Mittenabstand ±1.5 mm)
# Hintere Stiftaufnahme auf dem Hauptkörper
PIN_BOSS_R, PIN_BOSS_Y0 = 1.65 + WALL, 235.35
PCB_INSERTS = [(-330.2, -128.7 + gtr.PCB_DZ)]    # Platinen-Passschraube im Deckel (die untere sitzt im Träger)


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
def freiraum(with_sweep=True):
    """Alles, was aus Rahmen und Deckel ausgespart wird (Liste von Körpern)."""
    sx, sz = SHAFT
    cut = [
        ycyl(sx, sz, 5.5, Y_LID_BACK - 0.1, 236.6),          # Antriebsrad, Passscheibe
        ycyl(sx, sz, 13.6, 240.25 - CLR, BEARING_FRONT),        # 50er-Zahnrad, Zahnkranz (3 mm, bis y=243.25)
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
    if with_sweep:
        cut += idler_sweep()
    return tuple(cut)


def scharnier():
    """Hintere Idler-Stiftaufnahme: Zylinder (Achse in Y) vorne an der Motorplatte."""
    px, pz = PIN
    return ycyl(px, pz, PIN_BOSS_R, PIN_BOSS_Y0, sh.MOTOR_FACE_Y - MOTOR_PLATE_T + 0.01)


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _post_geo():
    x0, x1, _, _ = PCB_RECT
    xl = sh.MOTOR_AX[0] - MOTOR_EAR_DX         # links: mittig unter dem Langloch der Motorschraube
    xr = x1 - (xl - x0)                        # rechts symmetrisch zum Abstand der linken vom Rand
    return xl, xr, POST_TOP_L, KNOB_BOT - 0.4          # rechts endet die Stütze unter dem Rändelkopf


def stuetzen():
    """Zwei durchgehende Stützen im Platinenhalter (x-Mitte, z0, z1): links bis zur Oberkante des
    Platinenhalters, rechts bis unter den Rändelkopf."""
    xl, xr, zl, zr = _post_geo()
    return [(xl, Z_BOT, zl), (xr, Z_BOT, zr)]


def schrauben():
    """Vier M3-Verschraubungen (x, z): unten 1 mm über der Stützenmitte, oben mittig im oberen Stützenende."""
    xl, xr, zl, zr = _post_geo()
    zb = Z_BOT + POST / 2 + 1.0
    return [(xl, zb), (xr, zb), (xl, zl - POST / 2), (xr, zr - POST / 2)]


def querverbindungen():
    """Streben ab Y_BEAM_FRONT bis zur Motorplatte, so hoch wie die Stützen im Platinenhalter."""
    yw = sh.MOTOR_FACE_Y - MOTOR_PLATE_T
    r = None
    for x, za, zb in stuetzen():
        s_ = box(x - POST / 2, x + POST / 2, Y_BEAM_FRONT, yw + 0.01, za, zb)
        r = s_ if r is None else r + s_
    return r


def laschen():
    """Befestigungslaschen (x Lochmitte, y Lochmitte) am Fuß der Stützen, nach außen zur Seitenkante,
    mittig zwischen PCB-Platte und Motorplatte."""
    x0, x1, _, _ = PCB_RECT
    y = (Y_LID_FRONT + LID_T + sh.MOTOR_FACE_Y - MOTOR_PLATE_T) / 2
    return [(x0 + LUG_W / 2, y), (x1 - LUG_W / 2, y)]          # Halbkreis bis an die Seitenkante


def build_laschen():
    out = None
    for (xs, _, _), (xh, yh) in zip(stuetzen(), laschen()):
        lug = box(min(xs, xh), max(xs, xh), yh - LUG_W / 2, yh + LUG_W / 2, Z_BOT, Z_BOT + LUG_T)
        lug += Pos(xh, yh, Z_BOT) * Cylinder(LUG_W / 2, LUG_T, align=MIN)
        lug -= Pos(xh, yh, Z_BOT - 1) * Cylinder(1.7, LUG_T + 2, align=MIN)
        out = lug if out is None else out + lug
    return out


def build_deckel():
    """Platinenhalter: 3 mm Platte in Platinengröße, Verdickungen an Lager, Idlerstift und Platineneinsatz."""
    x0, x1, z0, z1 = PCB_RECT
    d = Pos((x0 + x1) / 2, Y_LID_FRONT, (z0 + z1) / 2) * Box(
        x1 - x0, LID_T, z1 - z0, align=(Align.CENTER, Align.MIN, Align.CENTER))
    for x, z, r in LID_PADS:
        d += ycyl(x, z, r, Y_LID_FRONT, Y_LID_BACK)
    # Halter endet an der Oberkante des Goliath-Trägers (darunter trägt der Goliath-Träger die Platine)
    d -= Pos(0, 0, Z_BOT) * Box(2000, 2000, 100, align=(Align.CENTER, Align.CENTER, Align.MAX))
    # Freiräume ohne die Idler-Kontur; am Idler werden nur vereinfachte Quader ausgespart (arm_freischnitt)
    for c in freiraum(with_sweep=False):
        cb = c.bounding_box()
        if cb.min.Y < Y_LID_BACK and cb.max.X > X_L and cb.max.Y > Y_LID_FRONT:
            d -= c
    # Stützen bis zum Beginn der Querverbindungen (der Rest gehört zur Motorplatte)
    for x, za, zb in stuetzen():
        d += box(x - POST / 2, x + POST / 2, Y_LID_FRONT + LID_T - 0.01, Y_BEAM_FRONT, za, zb)
    # Befestigungslaschen zum Hauptträger (M3 von oben, Einschmelzmutter im Goliath-Träger)
    d += build_laschen()
    # Einschmelzmuttern in den Stützen (verschraubt von der Motorseite)
    for x, z in schrauben():
        d -= ycyl(x, z, INSERT_D / 2, Y_BEAM_FRONT - INSERT_L, Y_BEAM_FRONT + 0.1)
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
    # Ring vor der Platte: Lagersitz wie im Original 2,5 mm tief (MR85 ganz versenkt)
    p += ycyl(*SHAFT, 8.2 / 2 + WALL, BEARING_FRONT, sh.MOTOR_FACE_Y - MOTOR_PLATE_T + 0.01)
    for x, z in schrauben():                                         # M3-Durchgang zu den Einschmelzmuttern
        p -= ycyl(x, z, 1.7, Y_BEAM_FRONT - 0.1, sh.MOTOR_FACE_Y + 0.1)
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
ARM_EDGE = (-330.0, -130.975)            # Punkt auf der Oberkante des Idler-Auslegers vor dem Platineneinsatz
ARM_SLOPE = 17.8                         # Steigung dieser Kante nach +X (Grad)
ARM_CLR = 0.5                            # Luft zwischen Platinenhalter und Idler-Arm
INSERT_WALL = 1.5                        # Wand um die Einschmelzmutter der Rändelschraube
GEAR_CUT_R = 4.75 + 0.75                 # Antriebs-/Idler-Rad (r4,75) mit Luft
UPPER = dict(top=18.0, bore_bot=5.32, bot=2.42)     # oben: Bohrung bis 5.32, Führung bis 2.42 über der Welle
LOWER = dict(top=-2.38, guide_bot=-5.28)           # unten: Führung bis -5.28, darunter Bohrung bis zum Träger


def zcyl(r, z0, z1):
    return Pos(b.AX_X, b.AX_Y, z0) * Cylinder(r, z1 - z0, align=MIN)


def build_fuehrungen():
    sz = SHAFT[1]
    ro = BORE_D / 2 + GUIDE_WALL
    up = zcyl(ro, sz + UPPER["bot"], sz + UPPER["top"])
    lo = zcyl(ro, Z_BOT, sz + LOWER["top"])
    # Stege nach vorne zum Platinenhalter, so breit wie das Rohr (innerhalb der Höhe des Halters)
    yb = Y_LID_FRONT + LID_T - 0.01
    z_lid_top = PCB_RECT[3]
    up += box(b.AX_X - ro, b.AX_X + ro, yb, b.AX_Y, sz + UPPER["bot"], min(sz + UPPER["top"], z_lid_top))
    lo += box(b.AX_X - ro, b.AX_X + ro, yb, b.AX_Y, Z_BOT, sz + LOWER["top"])
    # Bohrungen nach den Stegen, damit sie frei bleiben
    up -= zcyl(GUIDE_D / 2, sz + UPPER["bot"] - 1, sz + UPPER["top"] + 1)
    up -= zcyl(BORE_D / 2, sz + UPPER["bore_bot"], sz + UPPER["top"] + 1)
    lo -= zcyl(GUIDE_D / 2, Z_BOT - 1, sz + LOWER["top"] + 1)
    lo -= zcyl(BORE_D / 2, Z_BOT - 1, sz + LOWER["guide_bot"])
    # Aussparungen für Antriebsrad und Idler-Rad: zwei gleich große Kreise (Achsen in Y), symmetrisch
    # zur Filamentachse
    for x in (SHAFT[0], 2 * b.AX_X - SHAFT[0]):
        c = ycyl(x, sz, GEAR_CUT_R, b.AX_Y - 20, b.AX_Y + 20)
        up -= c
        lo -= c
    return up, lo


def spanneinsatz_halter():
    """Zylinder (Achse X) um die Einschmelzmutter der Rändelschraube, Lage wie im Original-Sherpa.
    Schließt an die obere Filamentführung an; die Mutter wird von +X eingesetzt."""
    q = sh.idler_insert().bounding_box()
    c = q.center()
    x0, x1 = q.min.X - 0.6 - INSERT_WALL, q.max.X
    h = Pos(x0, c.Y, c.Z) * Rot(0, 90, 0) * Cylinder(4.8 / 2 + INSERT_WALL, x1 - x0, align=MIN)
    h -= insert_bohrung()
    return h


def arm_freischnitt(teil):
    """Vereinfachter Freischnitt zum Idler-Arm: je Schwenkstellung der Quader um den Teil des Arms, der
    dem Bauteil nahe kommt, um ARM_CLR vergrößert und auf den Umriss des Bauteils begrenzt."""
    tb = teil.bounding_box()
    k = ARM_CLR
    zone = box(tb.min.X - k, tb.max.X + k, tb.min.Y - k, tb.max.Y + k, tb.min.Z - k, tb.max.Z + k)
    out = []
    for deg in (0.0, 6.0, 12.0):
        near = (turn(deg) * sh.get("idler_arm_short")[0]) & zone
        if not (near and near.solids()):
            continue
        ab = near.bounding_box()
        c = box(ab.min.X - ARM_CLR, ab.max.X + ARM_CLR, ab.min.Y - ARM_CLR, ab.max.Y + ARM_CLR,
                ab.min.Z - ARM_CLR, ab.max.Z + ARM_CLR)
        out.append(c & box(tb.min.X, tb.max.X, tb.min.Y, tb.max.Y, tb.min.Z, tb.max.Z))
    return out


def insert_bohrung():
    q = sh.idler_insert().bounding_box()
    c = q.center()
    return Pos(q.min.X - 0.6, c.Y, c.Z) * Rot(0, 90, 0) * Cylinder(4.8 / 2, q.max.X - q.min.X + 1.6, align=MIN)


def verstaerkung_oben():
    """Quader um die Einschmelzmutter des Idlers: von der linken Stütze bis zum Ende der Mutter,
    von der Rückseite des Platinenhalters bis hinter den Mutterhalter, in der Höhe des Mutterhalters."""
    q = sh.idler_insert().bounding_box()
    c = q.center()
    r = 4.8 / 2 + INSERT_WALL
    xl = stuetzen()[0][0] + POST / 2 - 0.01
    return box(xl, q.max.X, Y_LID_FRONT + LID_T - 0.01, c.Y + r, c.Z - r, c.Z + r)


# Filamentführungen, Spanneinsatz-Halter und Verstärkung gehören zum Platinenhalter
fuehrung_oben, fuehrung_unten = build_fuehrungen()
_d = deckel + fuehrung_oben + fuehrung_unten + spanneinsatz_halter() + verstaerkung_oben()
# vereinfachte Freischnitte zum Idler-Arm an den beiden Zylindern mit Einschmelzmuttern
# Platineneinsatz: alles unter einer schrägen Linie ARM_CLR über der Oberkante des Idler-Auslegers
# (steigt mit ARM_SLOPE nach +X an) wird vom Zylinder bis zur PCB-Wand weggeschnitten
for _x, _z, _r in LID_PADS[2:]:
    _above = Pos(ARM_EDGE[0], (Y_LID_FRONT + Y_LID_BACK) / 2, ARM_EDGE[1] + ARM_CLR) * Rot(0, -ARM_SLOPE, 0) * Box(
        40, 40, 20, align=(Align.CENTER, Align.CENTER, Align.MIN))
    _d -= box(_x - _r - 0.1, _x + _r + 0.1, Y_LID_FRONT + LID_T, Y_LID_BACK + 0.1, _z - _r - 0.1, _z + _r) - _above
for _c in arm_freischnitt(spanneinsatz_halter()):
    _d -= _c
_d -= insert_bohrung()                                               # Einschmelzmutter des Idlers
for _x, _z in PCB_INSERTS:                                           # Platineneinsatz
    _d -= ycyl(_x, _z, INSERT_D / 2, Y_LID_FRONT - 0.1, Y_LID_FRONT + INSERT_L)
_sz = SHAFT[1]
_d -= zcyl(GUIDE_D / 2, _sz + UPPER["bot"] - 1, _sz + UPPER["top"] + 1)       # Filamentkanal frei halten
_d -= zcyl(BORE_D / 2, _sz + UPPER["bore_bot"], _sz + UPPER["top"] + 1)
_d -= ycyl(*SHAFT, 7.0 / 2, Y_LID_FRONT - 1, 219.0)        # Ø7 als Verlängerung der Wellenachse nach vorne (wie Original)
deckel = max(_d.clean().solids(), key=lambda s: s.volume)

if __name__ == "__main__":
    b.OUT.mkdir(exist_ok=True)
    deckel.label = "deckel"
    export_step(deckel, str(b.OUT / "deckel.step"))
    print(f"Deckel: Volumen {deckel.volume:.0f} mm^3, Flächen {len(deckel.faces())}")
