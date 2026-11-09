"""Misst die Umrisse der Original-extruder_install_plate und schreibt sie nach
install_plate_konturen.py (Daten für den Nachbau in install_plate.py).

Gemessen wird nur am Original-STEP, nichts davon wird weiterverwendet als die
Koordinaten. Aufruf: .venv/bin/python tools/konturen_messen.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from build123d import Box, Compound, GeomType, Pos  # noqa: E402

import basis as b  # noqa: E402

orig = b.find("xtruder_install_plate")[0]
ob = orig.bounding_box()


def section(y):
    sec = Compound(list(orig.intersect(Pos(ob.center().X, y, ob.center().Z) * Box(400, 0.002, 400))))
    return [f for f in sec.faces() if f.normal_at().Y > 0.99]


def segs(w):
    out = []
    for e in w.edges():
        a, c = e.start_point(), e.end_point()
        if e.geom_type == GeomType.CIRCLE and e.radius > 0.3:
            m = e.position_at(0.5)
            out.append(("A", round(a.X, 3), round(a.Z, 3), round(m.X, 3), round(m.Z, 3), round(c.X, 3), round(c.Z, 3)))
        else:   # Geraden und kleine Spline-Rundungen (Spritzguss) -> Gerade
            out.append(("L", round(a.X, 3), round(a.Z, 3), round(c.X, 3), round(c.Z, 3)))
    return out


def largest(y):
    return max(section(y), key=lambda f: f.area)


LAYERS = {
    "BODY": 245.0,    # Plattenkörper (ohne die vier Führungsnasen, die install_plate.py ergänzt)
}
lines = ['"""Gemessene Umrisse der Original-Install-Plate (x, z), erzeugt von tools/konturen_messen.py.',
         'Segmente: ("L", x0, z0, x1, z1) Gerade, ("A", x0, z0, xm, zm, x1, z1) Bogen über drei Punkte."""', ""]
for name, y in LAYERS.items():
    lines.append(f"{name} = [")
    lines += [f"    {s!r}," for s in segs(largest(y).outer_wire())]
    lines.append("]\n")
pads = section(241.0)
lines.append("# Auflagen auf der Vorderseite (y 240.28..242.27)")
lines.append("PADS = [")
for f in sorted(pads, key=lambda f: (f.center().X, f.center().Z)):
    lines.append("    [")
    lines += [f"        {s!r}," for s in segs(f.outer_wire())]
    lines.append("    ],")
lines.append("]")
out = Path(__file__).resolve().parents[1] / "install_plate_konturen.py"
out.write_text("\n".join(lines) + "\n")
print(f"{out.name}: " + ", ".join(f"{n} {len(segs(largest(y).outer_wire()))} Segmente" for n, y in LAYERS.items())
      + f", {len(pads)} Auflagen")
