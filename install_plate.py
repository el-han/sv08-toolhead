"""
Neue extruder_install_plate (JXHSV08-06008) für den Sherpa-Umbau, vereinfacht.

Aufbau:
  1. Vorderteil = Originalplatte bis zur durchgehenden Haut (y <= Y_SKIN).
     Bohrungen und Konturen der Vorderseite bleiben original, darunter die
     drei M3-Bohrungen zum Schlitten.
  2. Rückseite = Querschnitt der Haut bei Y_SKIN (Umriss + Durchgangsbohrungen),
     massiv bis zur Rückseite extrudiert. Die Spritzguss-Rippen entfallen damit
     komplett, die ganze Rückseite ist eine Fläche.
  3. Funktionstaschen: nur die Nachbarteile, die tatsächlich in die Rückseite
     ragen (Sliding Panel, Senkkopfschrauben MGN-Wagen, ...), werden mit Luft
     CLR abgezogen, dazu die Rinne in Tasche 17.
"""
from build123d import Align, Box, Compound, Cylinder, Face, GeomType, Pos, Rot, Solid, extrude, export_step, export_stl

import basis as b

Y_SKIN = 245.0
CLR = 0.4
BIG = 400

orig = b.find("xtruder_install_plate")[0]
ob = orig.bounding_box()
Y_BACK = ob.max.Y
# Bohrungen des alten Motors (Mitte und Laschen) -> zu
OLD_MOTOR_HOLES = [(-342.0, -143.7), (-322.65, -133.35), (-361.35, -153.95)]
RINNE = Pos(-321.0, 249.0, -187.67) * Box(10, 12, 3.10)    # Tasche 17, Mitte bleibt offen


def _solid(shape):
    s = shape.solids()
    return s[0] if len(s) == 1 else max(s, key=lambda x: x.volume)


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


def build():
    front_box = Pos(ob.center().X, (ob.min.Y - 1 + Y_SKIN) / 2, ob.center().Z) * Box(BIG, Y_SKIN - ob.min.Y + 1, BIG)
    front = _solid(Compound(list(orig.intersect(front_box))))

    # Querschnitt der Haut -> massive Rückseite
    sliver = Compound(list(orig.intersect(Pos(ob.center().X, Y_SKIN - 0.01, ob.center().Z) * Box(BIG, 0.02, BIG))))
    faces = [f for f in sliver.faces() if f.normal_at().Y > 0.99]
    back = None
    holes = []
    for f in faces:
        outer = Face(f.outer_wire())
        e = extrude(outer, amount=Y_BACK - f.center().Y, dir=(0, 1, 0))
        back = e if back is None else back + e
        for w in f.inner_wires():
            ed = w.edges()
            if len(ed) == 1 and ed[0].geom_type == GeomType.CIRCLE:
                c = ed[0].arc_center
                if not any(abs(c.X - x) < 1.0 and abs(c.Z - z) < 1.0 for x, z in OLD_MOTOR_HOLES):
                    holes.append((c.X, c.Z, ed[0].radius))
    for x, z, r in holes:
        back = back - Pos(x, Y_SKIN - 1, z) * Rot(-90, 0, 0) * Cylinder(r, Y_BACK - Y_SKIN + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    print(f"Durchgangsbohrungen hinten: {len(holes)}")

    # Funktionstaschen: Nachbarteile, die in die Rückseite ragen
    bb = back.bounding_box()
    cutters = []
    for label, p in b.sv08_parts():
        if "xtruder_install_plate" in label or any(k in label for k in b.REMOVED_SHERPA + ["36_step_motor"]):
            continue
        pb = p.bounding_box()
        if (pb.max.Y < bb.min.Y or pb.min.Y > bb.max.Y or pb.max.X < bb.min.X or pb.min.X > bb.max.X
                or pb.max.Z < bb.min.Z or pb.min.Z > bb.max.Z):
            continue
        def _ov(s):
            r = s & back
            return sum(x.volume for x in r.solids()) if r else 0.0
        if any(_ov(s) > 0.01 for s in p.solids()):
            cutters.append((b.short(label), p))
    for name, p in cutters:
        back = back - stepped_cylinder(p, CLR)
    back = back - RINNE
    print("Funktionstaschen aus:", ", ".join(sorted({n for n, _ in cutters})))

    plate = _solid((front + back).clean())
    return plate


plate = build()

if __name__ == "__main__":
    b.OUT.mkdir(exist_ok=True)
    plate.label = "install_plate_sherpa"
    export_step(plate, str(b.OUT / "install_plate_sherpa.step"))
    print(f"Volumen {plate.volume:.0f} mm^3 (orig {orig.volume:.0f}), Flächen {len(plate.faces())} "
          f"(orig {len(orig.faces())}), gültig {plate.is_valid}")
