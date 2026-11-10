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
from build123d import Box, Compound, Location, Pos, export_step

import basis as b
import sherpa as sh

FRONT5020 = ["5020_turbofan", "air_duct_5020", "cover_plate3", "M2_5X8"]
FAN2515 = "2515_pipe_cooling_fan"


# Entfallene SV08-Schrauben, über Lage identifiziert (x, z, Achse):
#   alte Motorschrauben (Y) und M3x12 der alten Kabelklemme (X, ragen seitlich in den Sherpa)
REMOVED_SCREWS = [(-322.65, -133.35, "Y"), (-361.35, -153.95, "Y"), (-347.2, -129.65, "X"),
                  (-360.85, -166.95, "Y")]   # M2,5 des 5020 (Lüfter nicht mehr am Träger befestigt)


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
    out = []
    for label, p in b.sv08_parts():
        z = p.bounding_box().center().Z
        if "M3x12" in label and z < -196:                 # Sensor-/Deflektorschrauben bleiben
            out.append((label, "original", p))
            continue
        if "shoulder_screws" in label and "4xM3" in label:  # Platinen-Passschrauben
            out.append((label, "original", p))
            continue
        if any(k in label for k in b.REMOVED_SHERPA):
            continue
        if removed_screw(label, p):
            continue
        if "proximity_switch" in label:                    # Kabel der Eddy-Sonde entfernen
            p = p - Pos(-352.5, 239.8, -182.5) * Box(3.4, 3.4, 15.9)
        if any(k in label for k in FRONT5020):
            p, tag = Pos(*gtr.FAN5020_SHIFT) * p, "5020 vor"
        elif FAN2515 in label:
            # Kabel (4 dünne Litzen nach oben) entfernen, verschieben und auf die -X-Seite spiegeln
            p = Compound([p.solids()[0], p.solids()[1] - Pos(-323.65, 234.75, -155.0) * Box(1.2, 1.6, 20.2)])
            p = (Pos(gtr.FAN_SHIFT_X, 0, gtr.FAN_DZ) * p).mirror(gtr.FAN_MIRROR)
            tag = "2515 gespiegelt"
        else:
            tag = "original"
        out.append((label, tag, p))
    return out


def sherpa_placed():
    return [(f"sherpa_{n}", "Sherpa", p) for n, p in sh.parts if n not in sh.UNUSED]


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


# ---------------------------------------------------------------- Baum für check_assembly.step
NEW_COLORS = {"Hauptkoerper": (0.15, 0.65, 0.35), "Deckel": (0.20, 0.55, 0.80), "Motorplatte": (0.85, 0.75, 0.20), "Fuehrung": (0.90, 0.30, 0.30), "Goliath_Traeger": (0.15, 0.40, 0.85)}
SHERPA_COLORS = [("motor", (0.25, 0.25, 0.28)), ("zahnrad", (0.80, 0.65, 0.25)), ("antriebsrad", (0.80, 0.65, 0.25)),
                 ("idler_rad", (0.80, 0.65, 0.25)), ("idler_arm", (0.70, 0.30, 0.75)), ("lager", (0.75, 0.75, 0.78)),
                 ("nadellager", (0.75, 0.75, 0.78)), ("welle", (0.60, 0.60, 0.62)), ("raendel", (0.10, 0.10, 0.10))]
SV08_COLORS = [("turbofan", (0.12, 0.12, 0.12)), ("cooling_fan", (0.12, 0.12, 0.12)), ("duct", (0.55, 0.75, 0.90)),
               ("deflector", (0.55, 0.75, 0.90)), ("cover_plate", (0.55, 0.75, 0.90)), ("adapter_plate", (0.20, 0.45, 0.20)),
               ("proximity", (0.85, 0.45, 0.10)), ("sliding", (0.55, 0.55, 0.60)), ("MGN", (0.70, 0.70, 0.72)),
               ("screw", (0.80, 0.80, 0.82)), ("magnet", (0.40, 0.40, 0.45))]


def _named(shape, name, rgb):
    from build123d import Color
    s = shape.moved(Location()) if hasattr(shape, "moved") else shape
    s.label = name
    s.color = Color(*rgb)
    for i, c in enumerate(getattr(s, "children", ()) or ()):   # mehrteilige Teile (Lager)
        c.color = Color(*rgb)
        c.label = f"{name}_{i + 1}"
    return s


NAME_OVERRIDES = [
    ("3x8_4xM3", "passschraube_M3x8_platinen"), ("2_5x5xM2", "passschraube_M2_5x5"),
    ("plain_washer", "unterlegscheibe"), ("proximity_switch", "induktivsensor_H2N"),
    ("MGN12H", "MGN12H_wagen"), ("2515_pipe_cooling_fan", "luefter_2515_hotend"),
    ("3010_turbofan", "luefter_3010_turbo"), ("5020_turbofan", "luefter_5020_bauteil"),
    ("ptfe_tube_quick_coupling", "ptfe_schnellverbinder"), ("quick_plug_adapter", "ptfe_klammer"),
    ("length16mm", "ptfe_schlauch_16mm"), ("inner_diameter2mm_ptfe", "ptfe_schlauch"),
]


def nice_name(label):
    """Lesbarer Name aus den teils verstümmelten Sovol-Labels."""
    import re
    a = re.sub(r"[^ -~]", "", label or "")
    low = a.lower()
    for key, name in NAME_OVERRIDES:
        if key in a:
            return name
    m = re.search(r"M\d(?:_\d)?[xX]\d+", a)
    if m and "screw" in low:
        kind = ("senkkopf" if "countersunk" in low else "passschraube" if "shoulder" in low
                else "madenschraube" if "set_screw" in low else "zylinderkopf")
        return f"schraube_{m.group(0).replace('X', 'x')}_{kind}"
    s = re.sub(r"^[A-Z0-9]+-[0-9]+-[a-z]_", "", a)
    s = re.sub(r"^SV05p", "", s)
    s = re.sub(r"^[^A-Za-z]+", "", s)
    if s.startswith("xtruder"):
        s = "e" + s
    return s.strip("_")[:40] or "teil"


def _group(name, items):
    g = Compound(children=items)
    g.label = name
    return g


def assembly_tree(eb, gtr, sv, sp):
    """Baugruppe als Baum: Neuteile / Goliath / Sherpa / SV08 – mit Namen und Farben."""
    def pick(name, table, default):
        return next((c for k, c in table if k.lower() in name.lower()), default)

    def unique(items):
        seen, out = {}, []
        for n, p, c in items:
            seen[n] = seen.get(n, 0) + 1
            out.append(_named(p, n if seen[n] == 1 else f"{n}_{seen[n]}", c))
        return out

    neu = _group("Neuteile", [_named(eb.body, "Hauptkoerper", NEW_COLORS["Hauptkoerper"]),
                              _named(gh.deckel, "Deckel", NEW_COLORS["Deckel"]),
                              _named(gh.motorplatte_teil, "Motorplatte", NEW_COLORS["Motorplatte"]),
                              _named(gh.fuehrung_oben, "Filamentfuehrung_oben", NEW_COLORS["Fuehrung"]),
                              _named(gh.fuehrung_unten, "Filamentfuehrung_unten", NEW_COLORS["Fuehrung"])])
    gol = _group("Goliath", [_named(gtr.gol, "Goliath_AIR", (0.85, 0.50, 0.15))])
    sher = _group("Sherpa", unique([(n.removeprefix("sherpa_"), p, pick(n, SHERPA_COLORS, (0.60, 0.40, 0.70)))
                                    for n, t, p in sp]))
    sv_items = unique([(nice_name(n), p, pick(n, SV08_COLORS, (0.70, 0.70, 0.70))) for n, t, p in sv])
    sv08 = _group("SV08", sv_items)
    root = _group("SV08_Sherpa_Goliath", [neu, gol, sher, sv08])
    return root


if __name__ == "__main__":
    import extruder_body as eb
    import gehaeuse as gh
    import goliath_traeger as gtr
    import install_plate as ip

    sv = sv08_placed()
    sp = sherpa_placed()
    new = [("Hauptkörper", eb.body), ("Deckel", gh.deckel), ("Motorplatte", gh.motorplatte_teil),
           ("Führung oben", gh.fuehrung_oben), ("Führung unten", gh.fuehrung_unten), ("Goliath", gtr.gol)]
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
        base_sherpa = {}
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
        for fname, obj in (("extruder_body_sherpa", eb.body), ("deckel", gh.deckel), ("motorplatte", gh.motorplatte_teil)):
            obj.label = fname
            export_step(obj, str(b.OUT / f"{fname}.step"))
        ctx = [(p, "#b0b0b0", 0.30) for n, t, p in sv if -260 < p.bounding_box().center().Z < -90
               and p.bounding_box().size.Y < 120]
        ctx += [(p, "#c060c0", 0.8) for n, t, p in sp]
        items = ctx + [(gtr.gol, "#d08030", 1.0), (eb.body, "#20a060", 0.9)]
        if "--png" in sys.argv:
          render(items, str(b.OUT / "ansicht_baugruppe.png"),
               [(10, -90, "vorne"), (10, 0, "rechts (+X)"), (25, -130, "iso")])
          render([(eb.body, "#20a060", 1.0)], str(b.OUT / "ansicht_hauptkoerper.png"),
               [(20, -60, "vorne"), (20, 120, "hinten")])
        export_step(assembly_tree(eb, gtr, sv, sp), str(b.OUT / "check_assembly.step"))
        print("check_assembly.step geschrieben")
