"""Kollisionsprüfung, Renderbilder und Gesamt-STEP für den Sherpa-/Goliath-Umbau.

Aufruf:  .venv/bin/python check.py [--render | --nocheck] [--png]
  --render   Prüfung + Teile-STEP/STL + out/check_assembly.step
  --nocheck  wie --render, aber ohne Kollisionsprüfung
  --png      zusätzlich matplotlib-Ansichten (langsam)
"""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from build123d import Compound, Pos, export_step

import basis as b
import sherpa as sh

PCB = ["adapter_plate"]
FRONT5020 = ["5020_turbofan", "air_duct_5020", "cover_plate3", "M2_5X8"]
FAN2515 = "2515_pipe_cooling_fan"


# Entfallene SV08-Schrauben, über Lage identifiziert (x, z, Achse):
#   alte Motorschrauben (Y) und M3x12 der alten Kabelklemme (X, ragen seitlich in den Sherpa)
REMOVED_SCREWS = [(-322.65, -133.35, "Y"), (-361.35, -153.95, "Y"), (-347.2, -129.65, "X")]


def removed_screw(label, p):
    if "screw" not in label:
        return False
    bb = p.bounding_box()
    s = bb.size
    ax = "XYZ"[[s.X, s.Y, s.Z].index(max(s.X, s.Y, s.Z))]
    c = bb.center()
    return any(ax == a and abs(c.X - x) < 1.5 and abs(c.Z - z) < 1.5 for x, z, a in REMOVED_SCREWS)


def sv08_placed():
    """Verbleibende SV08-Teile an ihrer neuen Lage: (name, tag, shape)."""
    import goliath_traeger as gtr
    import pcb_halter as ph
    out = []
    for label, p in b.sv08_parts():
        z = p.bounding_box().center().Z
        if "M3x12" in label and z < -196:                 # Sensor-/Deflektorschrauben bleiben
            out.append((b.short(label), "original", p))
            continue
        if "shoulder_screws" in label and "4xM3" in label:  # Platinen-Passschrauben
            out.append((b.short(label), "Platinen vor", Pos(*ph.PCB_SHIFT) * p))
            continue
        if any(k in label for k in b.REMOVED_SHERPA):
            continue
        if removed_screw(label, p):
            continue
        if any(k in label for k in PCB):
            p, tag = Pos(*ph.PCB_SHIFT) * p, "Platinen vor"
        elif any(k in label for k in FRONT5020):
            p, tag = Pos(*gtr.FAN5020_SHIFT) * p, "5020 vor"
        elif FAN2515 in label:
            p, tag = Pos(gtr.FAN_SHIFT_X, 0, gtr.FAN_DZ) * p, "2515 verschoben"
        else:
            tag = "original"
        out.append((b.short(label), tag, p))
    return out


def sherpa_placed():
    # entfallen: Gehäuse (neues folgt), Einschmelzmuttern (V~46), Motorschrauben M3x12 (V~111),
    # Gehäuseschrauben M3x16 (V~140), Deckelschrauben M3x8 (V~83)
    drop = (46, 111, 140, 83)
    return [(f"sherpa_{n}", "Sherpa", p) for n, p in sh.parts
            if n not in sh.UNUSED and not n.startswith("housing")
            and not (n == "teil" and any(abs(p.volume - v) < 1.5 for v in drop))]


def overlap(a, c):
    ab, bb = a.bounding_box(), c.bounding_box()
    if (ab.min.X > bb.max.X or ab.max.X < bb.min.X or ab.min.Y > bb.max.Y or ab.max.Y < bb.min.Y
            or ab.min.Z > bb.max.Z or ab.max.Z < bb.min.Z):
        return 0.0
    vol = 0.0
    for sa in a.solids():
        for sb in c.solids():
            try:
                r = sa.intersect(sb)
                vol += sum(s.volume for s in r.solids()) if r else 0
            except Exception:
                return float("nan")
    return vol


def render(items, fname, views, lim=((-372, -300), (190, 262), (-250, -100))):
    fig = plt.figure(figsize=(6 * len(views), 7))
    for i, (elev, azim, title) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        for shape, color, alpha in [(s, c, a) for sh_, c, a in items for s in sh_.solids()]:
            try:
                verts, tris = shape.tessellate(0.3, 0.5)
            except Exception:
                continue
            if not tris:
                continue
            pts = [(v.X, v.Y, v.Z) for v in verts]
            ax.add_collection3d(Poly3DCollection([[pts[a], pts[b_], pts[c]] for a, b_, c in tris],
                                                 facecolors=color, alpha=alpha, shade=True,
                                                 lightsource=matplotlib.colors.LightSource(azdeg=225, altdeg=45)))
        ax.set_xlim(*lim[0]); ax.set_ylim(*lim[1]); ax.set_zlim(*lim[2])
        ax.set_box_aspect(tuple(l[1] - l[0] for l in lim))
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title)
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    fig.tight_layout()
    fig.savefig(fname, dpi=110)


if __name__ == "__main__":
    import extruder_body as eb
    import goliath_traeger as gtr
    import pcb_halter as ph
    import install_plate as ip

    sv = sv08_placed()
    sp = sherpa_placed()
    new = [("Hauptkörper", eb.body), ("Träger", gtr.traeger), ("Platinenhalter", ph.halter), ("Goliath", gtr.gol)]
    if "--nocheck" in sys.argv:
        sys.argv.append("--render")
        run_check = False
    else:
        run_check = True

    if run_check:
        # Grundlast: Überschneidungen, die schon in den Ausgangsmodellen bestehen
        base_plate = {}
        for label, p in b.sv08_parts():
            if "xtruder_install_plate" not in label:
                base_plate[id(p)] = overlap(ip.orig, p)
        housing = sh.get("housing_core") + sh.get("housing_rear")
        base_sherpa = {id(p): sum(overlap(h, p) for h in housing) for _, _, p in sp}
        bad = False
        others = [(n, t, p) for n, t, p in sv + sp]
        for oname, obj in new:
            for n, t, p in others:
                v = overlap(obj, p)
                if oname == "Hauptkörper":
                    v -= base_sherpa.get(id(p), 0)
                    if t == "original":
                        v -= next((base_plate[id(q)] for l, q in b.sv08_parts() if q is p), 0)
                if v > 0.05 or v != v:
                    bad = True
                    pb = p.bounding_box()
                    print(f"  KOLLISION {oname} <-> {n} ({t}): {v:.2f} mm^3  "
                          f"[{pb.min.X:.0f},{pb.max.X:.0f}]x[{pb.min.Y:.0f},{pb.max.Y:.0f}]x[{pb.min.Z:.0f},{pb.max.Z:.0f}]")
        for i in range(len(new)):
            for j in range(i + 1, len(new)):
                v = overlap(new[i][1], new[j][1])
                print(f"  {new[i][0]} <-> {new[j][0]}: {v:.2f} mm^3")
                if v > 0.05:
                    bad = True
        print(f"Düse z={gtr.nozzle_z:.2f} (Original {b.ORIG_NOZZLE_Z}); "
              f"Motorachse x={sh.MOTOR_AX[0]:.2f} z={sh.MOTOR_AX[1]:.2f}")
        print("Keine Kollisionen." if not bad else "Siehe Kollisionen oben.")

    if "--render" in sys.argv:
        from build123d import export_stl
        for fname, obj in (("extruder_body_sherpa", eb.body), ("goliath_traeger", gtr.traeger),
                           ("pcb_halter", ph.halter)):
            obj.label = fname
            export_step(obj, str(b.OUT / f"{fname}.step"))
            export_stl(obj, str(b.OUT / f"{fname}.stl"), tolerance=0.01, angular_tolerance=0.1)
        ctx = [(p, "#b0b0b0", 0.30) for n, t, p in sv if -260 < p.bounding_box().center().Z < -90
               and p.bounding_box().size.Y < 120]
        ctx += [(p, "#c060c0", 0.8) for n, t, p in sp]
        items = ctx + [(gtr.gol, "#d08030", 1.0), (gtr.traeger, "#2060d0", 1.0), (eb.body, "#20a060", 0.9),
                       (ph.halter, "#d0c020", 1.0)]
        if "--png" in sys.argv:
          render(items, str(b.OUT / "ansicht_baugruppe.png"),
               [(10, -90, "vorne"), (10, 0, "rechts (+X)"), (25, -130, "iso")])
          render([(eb.body, "#20a060", 1.0)], str(b.OUT / "ansicht_hauptkoerper.png"),
               [(20, -60, "vorne"), (20, 120, "hinten")])
        export_step(Compound([eb.body, gtr.traeger, ph.halter, gtr.gol] + [p for _, _, p in sv + sp]),
                    str(b.OUT / "check_assembly.step"))
        print("check_assembly.step geschrieben")
