"""Kollisionsprüfung und Renderbilder für den Goliath-Träger."""
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from build123d import Pos, Compound, export_step

import goliath_traeger as gt

REMOVED = ["xtruder_install_plate", "ooling_aluminum_block", "eating_block", "heating_plate", "thermistor", "silicone_sock",
           "shoulder_screw", "M2_5X30", "M2_5X18", "wire_harness", "ptfe_tube"]
LOWERED = ["air_deflector2", "3010_turbofan", "proximity_switch", "M2X5"]
FRONT = ["5020_turbofan", "air_duct_5020", "cover_plate3", "M2_5X8"]   # mit Lasche verschoben
FAN2515 = "2515_pipe_cooling_fan"

def short(label):
    m = re.findall(r"[A-Za-z0-9][A-Za-z0-9_\-]{3,}", label or "")
    return (m[-1] if m else label)[:40]

def placed_parts():
    out = []
    for label, p in gt.sv08.items():
        if any(k in label for k in REMOVED):
            continue
        if "M3x12" in label and p.bounding_box().center().Z < -196:   # Sensor-/Deflektorschrauben
            p, tag = Pos(0, 0, -gt.DROP) * p, "abgesenkt"
        elif "M3x5" in label:
            p, tag = Pos(0, 0, -gt.DROP) * p, "abgesenkt"
        elif any(k in label for k in FRONT):
            import fan5020_lasche as fl
            p, tag = Pos(*fl.FAN_SHIFT) * p, "abgesenkt+vor"
        elif any(k in label for k in LOWERED):
            p, tag = Pos(0, 0, -gt.DROP) * p, "abgesenkt"
        elif FAN2515 in label:
            p, tag = Pos(gt.FAN_FACE_X - gt.BLOCK_X[1], 0, 0) * p, "verschoben"
        else:
            tag = "original"
        out.append((short(label), tag, p))
    return out

def overlap(a, b):
    ab, bb = a.bounding_box(), b.bounding_box()
    if (ab.min.X > bb.max.X or ab.max.X < bb.min.X or ab.min.Y > bb.max.Y or ab.max.Y < bb.min.Y
            or ab.min.Z > bb.max.Z or ab.max.Z < bb.min.Z):
        return 0.0
    vol = 0.0
    for sa in a.solids():
        for sb in b.solids():
            try:
                r = sa.intersect(sb)
                vol += sum(s.volume for s in r.solids()) if r else 0
            except Exception:
                vol = float("nan")
    return vol

def render(items, fname, views):
    fig = plt.figure(figsize=(6 * len(views), 7))
    for i, (elev, azim, title) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        for shape, color, alpha in [(s, c, a) for sh, c, a in items for s in sh.solids()]:
            try:
                verts, tris = shape.tessellate(0.3, 0.5)
            except AttributeError:
                try:
                    shape = shape.clean()
                    verts, tris = shape.tessellate(0.6, 1.0)
                except Exception:
                    print("  (Render: Teil übersprungen)")
                    continue
            if not tris:
                continue
            pts = [(v.X, v.Y, v.Z) for v in verts]
            ax.add_collection3d(Poly3DCollection([[pts[a], pts[b], pts[c]] for a, b, c in tris],
                                                 facecolors=color, alpha=alpha, shade=True,
                                                 lightsource=matplotlib.colors.LightSource(azdeg=225, altdeg=45)))
        ax.set_xlim(-372, -300); ax.set_ylim(190, 262); ax.set_zlim(-250, -178 + 50)
        ax.set_box_aspect((72, 72, 122))
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title)
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    fig.tight_layout()
    fig.savefig(fname, dpi=110)

if __name__ == "__main__":
    import install_plate as ip
    import fan5020_lasche as fl
    parts = placed_parts()
    print(f"Absenkung Sensor/Luftführungen: {gt.DROP:.2f} mm")
    # Grundlast: Überschneidungen, die schon im Original-Modell existieren
    base = {}
    for label, p in gt.sv08.items():
        if "xtruder_install_plate" not in label:
            base[short(label)] = base.get(short(label), 0) + overlap(ip.orig, p)
    bad = False
    for name, tag, p in parts:
        for obj_name, obj in (("Träger", gt.traeger), ("Goliath", gt.gol), ("Install-Plate", ip.plate), ("Lasche", fl.lasche)):
            v = overlap(obj, p)
            if obj_name == "Install-Plate":
                v -= base.get(name, 0)
            if v > 0.05 or v != v:
                bad = True
                print(f"  KOLLISION {obj_name} <-> {name} ({tag}): {v:.2f} mm^3")
    for a, b, n in ((gt.traeger, gt.gol, "Träger <-> Goliath"), (ip.plate, gt.gol, "Plate <-> Goliath"),
                    (ip.plate, gt.traeger, "Plate <-> Träger")):
        print(f"  {n}: {overlap(a, b):.2f} mm^3")
    print("Keine Kollisionen." if not bad else "Siehe Kollisionen oben.")

    if "--render" in sys.argv:
        ctx = [(p, "#b0b0b0", 0.35) for n, t, p in parts if p.bounding_box().max.Z > -260 and p.bounding_box().min.Z < -110]
        items = ctx + [(gt.gol, "#d08030", 1.0), (gt.traeger, "#2060d0", 1.0), (ip.plate, "#20a060", 0.9)]
        render(items, str(gt.OUT / "ansicht_baugruppe.png"),
               [(10, -90, "vorne"), (10, 0, "rechts (+X)"), (25, -130, "iso")])
        render([(gt.traeger, "#2060d0", 1.0)], str(gt.OUT / "ansicht_traeger.png"),
               [(30, -60, "oben"), (-35, -60, "unten")])
        export_step(Compound([gt.traeger, gt.gol, ip.plate, fl.lasche] +[p for _, _, p in parts]), str(gt.OUT / "check_assembly.step"))
