"""
Neue extruder_install_plate für den Sherpa-Umbau – Nachbau in build123d.

Die Platte wird aus gemessenen Umrissen (install_plate_konturen.py, erzeugt
von tools/konturen_messen.py) und benannten Bohrungen neu aufgebaut. Vom
Original-STEP werden nur Koordinaten übernommen, keine Geometrie.

Schichten (y, von vorne nach hinten):
  Auflagen      Y_PAD   .. Y_FRONT   5 kleine Auflagen auf der Vorderseite
  Vorderfläche  Y_FRONT .. Y_STEP    Körperumriss um FRONT_INSET nach innen versetzt
  Körper        Y_STEP  .. Y_BACK    ein Umriss (mit den Aussparungen für das Sliding Panel)

Bewusst weggelassen gegenüber dem Original: Spritzguss-Rippen und -Schrägen,
Sitz und Bohrungen des alten Motors, Zahnrad-Aussparung, Oval-Ansatz.
Funktionstaschen für Schrauben, die in die Rückseite ragen, werden aus den
Nachbarteilen bestimmt (glatte Stufenzylinder mit Luft CLR).
"""
from build123d import (Align, Box, Compound, Cylinder, Edge, Face, Kind, Pos, Rot, Solid, Vector, Wire,
                       export_step, extrude)

import basis as b
import install_plate_konturen as K

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
CLR = 0.4
BIG = 400

# ---------------------------------------------------------------- Schichten (y)
Y_PAD = 240.2747
Y_FRONT = 242.2747
Y_STEP = 242.8747
Y_NOTCH = 245.0
Y_BACK = 250.7747
FRONT_INSET = 1.466          # Absatz der Vorderfläche gegenüber dem Körperumriss

# ---------------------------------------------------------------- Bohrungen (x, z, r)
DURCHGANG = {
    "schlitten_1": (-361.525, -201.175, 1.75),     # M3 zum Schlitten
    "schlitten_2": (-315.825, -194.675, 1.75),
    "schlitten_3": (-310.825, -158.675, 1.75),
    "sensor": (-352.525, -215.975, 1.40),          # Sensorhalter
    "luftfuehrung": (-352.525, -199.975, 2.00),    # hintere Luftführung
    "schiene_links": (-341.525, -202.175, 2.00),   # M3 von hinten (Sliding Panel)
    "schiene_rechts": (-325.525, -202.175, 2.00),
}
RINNE = Pos(-321.0, (Y_NOTCH + 255.0) / 2, -187.67) * Box(10, 255.0 - Y_NOTCH, 3.10)   # Querrinne hinten (ehem. Tasche 17)
OVAL_PADS = (-340.8, -331.0, -194.0, -187.5)               # Auflagen im Oval-Bereich entfallen


# ---------------------------------------------------------------- Hilfen
def contour_face(segs, y):
    edges = []
    for s in segs:
        if s[0] == "L":
            a, c = Vector(s[1], y, s[2]), Vector(s[3], y, s[4])
            if (a - c).length > 1e-4:
                edges.append(Edge.make_line(a, c))
        else:
            edges.append(Edge.make_three_point_arc(Vector(s[1], y, s[2]), Vector(s[3], y, s[4]),
                                                   Vector(s[5], y, s[6])))
    return Face(Wire(edges))


def slab(segs, y0, y1):
    return extrude(contour_face(segs, y0), amount=y1 - y0, dir=(0, 1, 0))


def y_hole(x, z, r, y0, y1):
    return Pos(x, y0, z) * Rot(-90, 0, 0) * Cylinder(r, y1 - y0, align=MIN)


def stepped_cylinder(part, clr, step=0.5):
    """Ersatzkörper für eine Schraube: abgestufte glatte Zylinder entlang ihrer Achse
    (Radius je Abschnitt aus Schnitten), um clr vergrößert. Keine Gewindeflächen."""
    pb = part.bounding_box()
    sz = (pb.size.X, pb.size.Y, pb.size.Z)
    ax = sz.index(max(sz))
    lo, hi = (pb.min.X, pb.min.Y, pb.min.Z)[ax], (pb.max.X, pb.max.Y, pb.max.Z)[ax]
    c = pb.center()
    rot = [Rot(0, 90, 0), Rot(-90, 0, 0), Rot(0, 0, 0)][ax]
    segs = []
    t = lo
    while t < hi - 1e-6:
        t1 = min(t + step, hi)
        dims = [BIG, BIG, BIG]
        dims[ax] = t1 - t
        ctr = [c.X, c.Y, c.Z]
        ctr[ax] = (t + t1) / 2
        r = 0.0
        for so in part.solids():
            x = so & (Pos(*ctr) * Box(*dims))
            if x and x.solids():
                xb = Compound(list(x.solids())).bounding_box()
                cross = [xb.size.X, xb.size.Y, xb.size.Z]
                del cross[ax]
                r = max(r, max(cross) / 2)
        r = round(r, 1)
        if segs and segs[-1][2] == r:
            segs[-1][1] = t1
        else:
            segs.append([t, t1, r])
        t = t1
    out = None
    for a, e, r in segs:
        if r <= 0:
            continue
        ctr = [c.X, c.Y, c.Z]
        ctr[ax] = (a + e) / 2
        cyl = Pos(*ctr) * rot * Cylinder(r + clr, e - a + 2 * clr * (a == lo or e == hi))
        out = cyl if out is None else out + cyl
    return out


def unify(solid):
    """Flächen gleicher Geometrie zusammenführen (Nähte gefüllter Taschen entfernen).
    Fällt auf das Original zurück, wenn das Ergebnis ungültig ist."""
    from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
    u = ShapeUpgrade_UnifySameDomain(solid.wrapped, True, True, True)
    u.Build()
    out = Solid(u.Shape())
    if out.is_valid and abs(out.volume - solid.volume) < 0.01:
        print(f"Flächen vereinigt: {len(solid.faces())} -> {len(out.faces())}")
        return out
    print("Flächen vereinigen ergab ungültigen Körper, unverändert")
    return solid



def build():
    body_face = contour_face(K.BODY, Y_FRONT)
    front_face = Face(body_face.outer_wire().offset_2d(-FRONT_INSET, kind=Kind.ARC))
    plate = extrude(front_face, amount=Y_STEP - Y_FRONT, dir=(0, 1, 0))
    plate += slab(K.BODY, Y_STEP - 0.01, Y_BACK)
    for pad in K.PADS:
        xs = [s[1] for s in pad]
        zs = [s[2] for s in pad]
        cx, cz = sum(xs) / len(xs), sum(zs) / len(zs)
        if OVAL_PADS[0] < cx < OVAL_PADS[1] and OVAL_PADS[2] < cz < OVAL_PADS[3]:
            continue
        if len(pad) < 3:
            continue
        plate += slab(pad, Y_PAD, Y_FRONT + 0.01)

    for x, z, r in DURCHGANG.values():
        plate -= y_hole(x, z, r, Y_PAD - 1, Y_BACK + 1)

    # Funktionstaschen: Schrauben der Nachbarteile, die in die Platte ragen
    pb = plate.bounding_box()
    names = set()
    for label, p in b.sv08_parts():
        if "screw" not in label or any(k in label for k in b.REMOVED_SHERPA):
            continue
        q = p.bounding_box()
        if (q.max.Y < pb.min.Y or q.min.Y > pb.max.Y or q.max.X < pb.min.X or q.min.X > pb.max.X
                or q.max.Z < pb.min.Z or q.min.Z > pb.max.Z):
            continue
        hit = any((s & plate) and sum(x.volume for x in (s & plate).solids()) > 0.01 for s in p.solids())
        if hit and not _old_motor_screw(p):
            plate -= stepped_cylinder(p, CLR)
            names.add(b.short(label))
    plate -= RINNE
    print("Funktionstaschen für:", ", ".join(sorted(names)))
    s = plate.clean().solids()
    return max(s, key=lambda x: x.volume)


def _old_motor_screw(p):
    c = p.bounding_box().center()
    return any(abs(c.X - x) < 1.5 and abs(c.Z - z) < 1.5 for x, z in ((-322.65, -133.35), (-361.35, -153.95)))


orig = b.find("xtruder_install_plate")[0]     # nur für Vergleiche (check.py)
ob = orig.bounding_box()
plate = build()

if __name__ == "__main__":
    b.OUT.mkdir(exist_ok=True)
    plate.label = "install_plate_sherpa"
    export_step(plate, str(b.OUT / "install_plate_sherpa.step"))
    print(f"Volumen {plate.volume:.0f} mm^3, Flächen {len(plate.faces())}, gültig {plate.is_valid}")
