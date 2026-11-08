"""
Neue extruder_install_plate (JXHSV08-06008) für den Goliath-Umbau.

Oberhalb Z_CUT bleibt die Original-Geometrie (inkl. der drei M3-Bohrungen zum
Schlitten/sliding panel, Magnettaschen, Riemenklemmen-Bohrungen). Alles unter
Z_CUT (Sensor-, Luftführungs-Befestigung) wird um DROP nach unten versetzt,
die Lücke wird mit dem Querschnitt bei Z_CUT aufgefüllt. Bohrungen zum
Schlitten, die unter Z_CUT liegen, werden an ihrer Originalstelle
wiederhergestellt ("transplantiert").
"""
from build123d import Axis, Box, Compound, Cylinder, Pos, Rot, Vector, extrude, export_step, export_stl, Align

import goliath_traeger as gt

Z_CUT = -198.0
DROP = gt.DROP
# Schlitten-Bohrungen unterhalb Z_CUT: (x, z), Achse in Y, Transplantationsradius
KEEP_HOLES = [(-361.52, -201.17, 4.6)]

orig = gt.find(gt.sv08, "xtruder_install_plate")[0]
ob = orig.bounding_box()
BIG = 400

def clip(shape, zmin, zmax):
    box = Pos(ob.center().X, ob.center().Y, (zmin + zmax) / 2) * Box(BIG, BIG, zmax - zmin)
    res = None
    for s in shape.solids():
        r = s.intersect(box)
        for x in (r.solids() if r else []):
            res = x if res is None else res + x
    return res

upper = clip(orig, Z_CUT, ob.max.Z + 1)
lower = Pos(0, 0, -DROP) * clip(orig, ob.min.Z - 1, Z_CUT)

# Querschnitt bei Z_CUT nach unten extrudieren
sliver = clip(orig, Z_CUT - 0.02, Z_CUT)
faces = [f for f in sliver.faces() if f.normal_at().Z > 0.99 and abs(f.center().Z - Z_CUT) < 1e-3]
fill = None
for f in faces:
    e = extrude(f, amount=DROP, dir=(0, 0, -1))
    fill = e if fill is None else fill + e

plate = upper + fill + lower

for x, z, r in KEEP_HOLES:
    plug = Pos(x, ob.center().Y, z) * Rot(90, 0, 0) * Cylinder(r, BIG)
    plate -= plug
    local = clip(orig, z - r - 1, z + r + 1)
    for s in local.solids():
        piece = s.intersect(plug)
        for p in (piece.solids() if piece else []):
            plate += p

plate = plate.clean()

# ---------------------------------------------------------------- Rückseite flach
# Spritzguss-Hohlräume hinter der durchgehenden Haut (y ~ 242.3..245) füllen.
# Taschen, in die andere Bauteile ragen (Motor, Schraubenköpfe), bleiben offen.
FLAT_BACK = True
Y_SKIN = 245.0
CLEARANCE_GROW = 0.4
MOTOR_CONTACT_Y = 248.27   # Anlagefläche des Extrudermotors an der Platte

def _with_clearance(shape, g):
    """Grobe Aufmaß-Hülle: Original + um +/-g verschobene Kopien."""
    out = shape
    for d in ((g, 0, 0), (-g, 0, 0), (0, g, 0), (0, -g, 0), (0, 0, g), (0, 0, -g)):
        out = out + Pos(*d) * shape
    return out

def _under(piece, part, depth=12.0, step=1.0):
    """Liegt das Teil (in +Y-Richtung) über der Tasche? Tasche schrittweise nach hinten schieben."""
    k = 0.0
    while k <= depth:
        moved = Pos(0, k, 0) * piece
        for s in part.solids():
            r = s.intersect(moved)
            if r and any(x.volume > 0.05 for x in r.solids()):
                return True
        k += step
    return False

def flatten_back(plate, neighbours, fill_around=()):
    """fill_around: Bauteile, um die herum Taschen trotzdem gefüllt werden."""
    b = plate.bounding_box()
    slab = Pos(b.center().X, Y_SKIN, b.center().Z) * Box(BIG, 0.02, BIG)
    prism = None
    for s in plate.solids():
        r = s.intersect(slab)
        for so in (r.solids() if r else []):
            for f in so.faces():
                if f.normal_at().Y > 0.99:
                    e = extrude(f, amount=b.max.Y - f.center().Y, dir=(0, 1, 0))
                    prism = e if prism is None else prism + e
    kept, skipped = [], 0
    around = [_with_clearance(s, CLEARANCE_GROW) for a in fill_around for s in a.solids()]
    fp = [a.bounding_box() for a in fill_around]
    for piece in (prism - plate).solids():
        pb = piece.bounding_box()
        for a, ab in zip(fill_around, fp):
            if pb.min.X < ab.max.X and pb.max.X > ab.min.X and pb.min.Z < ab.max.Z and pb.max.Z > ab.min.Z and \
                    _under(piece, a):
                # im Grundriss des Motors: nur bis zur Motor-Anlagefläche füllen
                clipped = piece.intersect(Pos(pb.center().X, (pb.min.Y + MOTOR_CONTACT_Y) / 2 - 0.5, pb.center().Z)
                                          * Box(BIG, MOTOR_CONTACT_Y - pb.min.Y + 1, BIG))
                piece = Compound(clipped.solids()) if clipped else Compound([])
                if not piece.solids():
                    break
        if not piece.solids():
            continue
        for a in around:
            piece = piece - a
        if not piece.solids():
            continue
        pieces = piece.solids()
        piece = pieces[0] if len(pieces) == 1 else Compound(pieces)
        pb = piece.bounding_box()
        hit = False
        for n in neighbours:
            nb = n.bounding_box()
            g = CLEARANCE_GROW
            if (pb.min.X - g > nb.max.X or pb.max.X + g < nb.min.X or pb.min.Y - g > nb.max.Y
                    or pb.max.Y + g < nb.min.Y or pb.min.Z - g > nb.max.Z or pb.max.Z + g < nb.min.Z):
                continue
            for ns in n.solids():
                r = ns.intersect(piece)
                if r and any(x.volume > 0.01 for x in r.solids()):
                    hit = True
                    break
            if hit:
                break
        if hit:
            skipped += 1
            print(f"  offen: Tasche bbox x[{pb.min.X:.1f},{pb.max.X:.1f}] y[{pb.min.Y:.1f},{pb.max.Y:.1f}] "
                  f"z[{pb.min.Z:.1f},{pb.max.Z:.1f}] V={piece.volume:.0f} <- {getattr(n, 'label', '')[-30:]}")
        else:
            kept.append(piece)
    for k in kept:
        plate += k
    print(f"Rückseite: {len(kept)} Hohlräume gefüllt, {skipped} Funktionstaschen offen gelassen")
    return plate.clean()

if FLAT_BACK:
    import check
    _all = check.placed_parts()
    _motor = [p for n, _, p in _all if "step_motor" in n]
    _neigh = [p for n, _, p in _all if "step_motor" not in n]
    plate = flatten_back(plate, _neigh, fill_around=_motor)

    # Restliche Vertiefungen im Motorbereich (Freiraum-Ränder, Tasche außen rechts)
    # bis auf Höhe der Motor-Anlagefläche auffüllen. Durchgangsbohrungen (reichen bis
    # zum Boden der Schicht) bleiben offen.
    Y_LO = 246.0
    X_RNG, Z_RNG = (-370.0, -315.0), (-164.0, -118.0)
    slab = Pos(sum(X_RNG) / 2, (Y_LO + MOTOR_CONTACT_Y) / 2, sum(Z_RNG) / 2) * \
        Box(X_RNG[1] - X_RNG[0], MOTOR_CONTACT_Y - Y_LO, Z_RNG[1] - Z_RNG[0])
    n_fill = 0
    for piece in (slab - plate).solids():
        pb = piece.bounding_box()
        if pb.min.Y > Y_LO + 0.05 and piece.volume > 0.01:
            plate += piece
            n_fill += 1
    plate = plate.clean()
    print(f"Motorbereich: {n_fill} Vertiefungen auf y={MOTOR_CONTACT_Y} aufgefüllt")

    # Weitere Taschen bis zur Rückseite füllen (Nutzerauswahl). (x-Bereich, z-Bereich, Schlitz behalten)
    Y_TOP = plate.bounding_box().max.Y
    Y_LO2 = 242.0
    SURF_FILL = [
        ((-311.5, -306.0), (-143.5, -138.0), None),            # 4
        ((-311.5, -307.0), (-197.0, -192.5), None),            # 16
        ((-366.0, -361.5), (-197.0, -192.5), None),            # 19
        ((-336.0, -325.0), (-190.5, -176.5), None),            # 14, 15
    ]
    # 17: Rechtecktasche + quer laufende Rundrinne; die Rinne (Mitte) bleibt, Rest auf Oberfläche
    P17 = Pos(-321.275, (245.27 + Y_TOP) / 2, -187.65) * Box(5.65, Y_TOP - 245.27, 5.04)
    RINNE = Pos(-321.0, 249.0, -187.67) * Box(10, 12, 3.10)
    plate += (P17 - RINNE)
    for xr, zr, keep in SURF_FILL:
        box = Pos(sum(xr) / 2, (Y_LO2 + Y_TOP) / 2, sum(zr) / 2) * Box(xr[1] - xr[0], Y_TOP - Y_LO2, zr[1] - zr[0])
        add = [p for p in (box - plate).solids()
               if p.bounding_box().min.Y > Y_LO2 + 0.05 and p.volume > 0.01]
        if keep:
            # tiefster Boden nahe keep = Schlitz; dessen Grundriss offen lassen
            cand = [f for f in plate.faces() if f.normal_at().Y > 0.99
                    and abs(f.center().X - keep[0]) < 1.5 and abs(f.center().Z - keep[1]) < 1.5
                    and f.center().Y < Y_TOP - 1]
            f = min(cand, key=lambda f: f.center().Y)
            slot = extrude(f, amount=Y_TOP - f.center().Y + 1, dir=(0, 1, 0))
            add = [s for p in add for s in (p - slot).solids()]
        for p in add:
            plate += p
        print(f"  Tasche x{xr} z{zr}: {len(add)} Stück auf Oberfläche gefüllt")
    plate = plate.clean()

if __name__ == "__main__":
    gt.OUT.mkdir(exist_ok=True)
    plate.label = "install_plate_goliath"
    export_step(plate, str(gt.OUT / "install_plate_goliath.step"))
    export_stl(plate, str(gt.OUT / "install_plate_goliath.stl"), tolerance=0.01, angular_tolerance=0.1)
    b = plate.bounding_box()
    print(f"DROP={DROP:.2f}  Volumen {plate.volume:.0f} mm^3 (orig {orig.volume:.0f}), Höhe {b.size.Z:.2f} (orig {ob.size.Z:.2f})")
