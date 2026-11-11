"""Collision check, render images and complete STEP for the Sherpa/Goliath conversion.

Usage:  .venv/bin/python check.py [--render | --nocheck] [--png]
  --render   check + part STEPs + out/check_assembly.step
  --nocheck  like --render, but without the collision check
  --png      additional matplotlib views (slow)
"""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from build123d import Box, Compound, Location, Pos, export_step

import common as cm
import sherpa as sh

FRONT5020 = ["5020_turbofan", "air_duct_5020", "cover_plate3", "M2_5X8"]
FAN2515 = "2515_pipe_cooling_fan"
FIL_R = 0.95                       # filament Ø1.75 with some clearance (guide Ø2.1)


# Dropped SV08 screws, identified by position (x, z, axis):
#   old motor screws (Y) and M3x12 of the old cable clamp (X, reach into the Sherpa from the side)
REMOVED_SCREWS = [(-322.65, -133.35, "Y"), (-361.35, -153.95, "Y"), (-347.2, -129.65, "X"),
                  (-360.85, -166.95, "Y")]   # M2.5 of the 5020 (fan no longer attached to the carrier)


def removed_screw(label, p):
    if "screw" not in label:
        return False
    bb = p.bounding_box()
    s = bb.size
    ax = "XYZ"[[s.X, s.Y, s.Z].index(max(s.X, s.Y, s.Z))]
    c = bb.center()
    return any(ax == a and abs(c.X - x) < 1.5 and abs(c.Z - z) < 1.5 for x, z, a in REMOVED_SCREWS)


def sv08_placed():
    """Remaining SV08 parts at their new position: (name, tag, shape)."""
    import goliath_carrier as gc
    out = []
    for label, p in cm.sv08_parts():
        z = p.bounding_box().center().Z
        if "M3x12" in label and z < -196:                 # sensor/deflector screws stay
            out.append((label, "original", p))
            continue
        if "shoulder_screws" in label and "4xM3" in label:  # PCB shoulder screws (raised with the PCB)
            out.append((label, "PCB raised", Pos(0, 0, gc.PCB_DZ) * p))
            continue
        if "adapter_plate" in label:                       # PCBs (adapter plates) raised
            out.append((label, "PCB raised", Pos(0, 0, gc.PCB_DZ) * p))
            continue
        if any(k in label for k in cm.REMOVED_SHERPA):
            continue
        if removed_screw(label, p):
            continue
        if "proximity_switch" in label:                    # remove the cable of the Eddy probe
            p = p - Pos(-352.5, 239.8, -182.5) * Box(3.4, 3.4, 15.9)
        if any(k in label for k in FRONT5020):
            p, tag = Pos(*gc.FAN5020_SHIFT) * p, "5020 forward"
        elif FAN2515 in label:
            # remove the cable (4 thin wires upwards), shift and mirror to the -X side
            p = Compound([p.solids()[0], p.solids()[1] - Pos(-323.65, 234.75, -155.0) * Box(1.2, 1.6, 20.2)])
            p = (Pos(gc.FAN_SHIFT_X, 0, gc.FAN_DZ) * p).mirror(gc.FAN_MIRROR)
            tag = "2515 mirrored"
        else:
            tag = "original"
        out.append((label, tag, p))
    return out


def sherpa_placed():
    return ([(f"sherpa_{n}", "Sherpa", p) for n, p in sh.parts if n not in sh.UNUSED]
            + [("sherpa_heat_insert_idler", "Sherpa", sh.idler_insert())])


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


# ---------------------------------------------------------------- tree for check_assembly.step
NEW_COLORS = {"MainBody": (0.15, 0.65, 0.35), "PcbHolder": (0.20, 0.55, 0.80), "MotorPlate": (0.85, 0.75, 0.20)}
SILVER, GOLD = (0.80, 0.80, 0.82), (0.85, 0.75, 0.25)   # screws / heat-set inserts
SHERPA_COLORS = [("heat_insert", GOLD), ("motor", (0.25, 0.25, 0.28)), ("gear_50t", (0.80, 0.65, 0.25)),
                 ("drive_gear", (0.80, 0.65, 0.25)), ("idler_gear", (0.80, 0.65, 0.25)), ("idler_arm", (0.70, 0.30, 0.75)),
                 ("bearing", (0.75, 0.75, 0.78)), ("shaft", (0.60, 0.60, 0.62)), ("thumbscrew", (0.10, 0.10, 0.10))]
SV08_COLORS = [("turbofan", (0.12, 0.12, 0.12)), ("cooling_fan", (0.12, 0.12, 0.12)), ("duct", (0.55, 0.75, 0.90)),
               ("deflector", (0.55, 0.75, 0.90)), ("cover_plate", (0.55, 0.75, 0.90)), ("adapter_plate", (0.20, 0.45, 0.20)),
               ("proximity", (0.85, 0.45, 0.10)), ("sliding", (0.55, 0.55, 0.60)), ("MGN", (0.70, 0.70, 0.72)),
               ("screw", (0.80, 0.80, 0.82)), ("magnet", (0.40, 0.40, 0.45))]


def _named(shape, name, rgb):
    from build123d import Color
    s = shape.moved(Location()) if hasattr(shape, "moved") else shape
    s.label = name
    s.color = Color(*rgb)
    for i, c in enumerate(getattr(s, "children", ()) or ()):   # multi-solid parts (bearings)
        c.color = Color(*rgb)
        c.label = f"{name}_{i + 1}"
    return s


NAME_OVERRIDES = [
    ("3x8_4xM3", "shoulder_screw_M3x8_pcb"), ("2_5x5xM2", "shoulder_screw_M2_5x5"),
    ("plain_washer", "washer"), ("proximity_switch", "eddy_probe"),
    ("MGN12H", "MGN12H_carriage"), ("2515_pipe_cooling_fan", "fan_2515_hotend"),
    ("3010_turbofan", "fan_3010_turbo"), ("5020_turbofan", "fan_5020_part_cooling"),
    ("ptfe_tube_quick_coupling", "ptfe_quick_coupling"), ("quick_plug_adapter", "ptfe_clip"),
    ("length16mm", "ptfe_tube_16mm"), ("inner_diameter2mm_ptfe", "ptfe_tube"),
]


def nice_name(label):
    """Readable name from the partly garbled Sovol labels."""
    import re
    a = re.sub(r"[^ -~]", "", label or "")
    low = a.lower()
    for key, name in NAME_OVERRIDES:
        if key in a:
            return name
    m = re.search(r"M\d(?:_\d)?[xX]\d+", a)
    if m and "screw" in low:
        kind = ("countersunk" if "countersunk" in low else "shoulder" if "shoulder" in low
                else "set_screw" if "set_screw" in low else "socket_head")
        return f"screw_{m.group(0).replace('X', 'x')}_{kind}"
    s = re.sub(r"^[A-Z0-9]+-[0-9]+-[a-z]_", "", a)
    s = re.sub(r"^SV05p", "", s)
    s = re.sub(r"^[^A-Za-z]+", "", s)
    if s.startswith("xtruder"):
        s = "e" + s
    return s.strip("_")[:40] or "part"


def _group(name, items):
    g = Compound(children=items)
    g.label = name
    return g


def assembly_tree(eb, gc, sv, sp):
    """Assembly as a tree: new parts / Goliath / Sherpa / fasteners / SV08, with names and colours."""
    def pick(name, table, default):
        return next((c for k, c in table if k.lower() in name.lower()), default)

    def unique(items):
        seen, out = {}, []
        for n, p, c in items:
            seen[n] = seen.get(n, 0) + 1
            out.append(_named(p, n if seen[n] == 1 else f"{n}_{seen[n]}", c))
        return out

    new = _group("NewParts", [_named(eb.body, "MainBody", NEW_COLORS["MainBody"]),
                              _named(hs.pcb_holder, "PcbHolder", NEW_COLORS["PcbHolder"]),
                              _named(hs.motor_plate, "MotorPlate", NEW_COLORS["MotorPlate"])])
    gol = _group("Goliath", [_named(gc.gol, "Goliath_AIR", (0.85, 0.50, 0.15))])
    sher = _group("Sherpa", unique([(n.removeprefix("sherpa_"), p, pick(n, SHERPA_COLORS, (0.60, 0.40, 0.70)))
                                    for n, t, p in sp]))
    sv_items = unique([(nice_name(n), p, pick(n, SV08_COLORS, (0.70, 0.70, 0.70))) for n, t, p in sv])
    sv08 = _group("SV08", sv_items)
    import fasteners as fs
    fst = _group("Fasteners", [_named(p, n, GOLD if "insert" in n else SILVER) for n, p in fs.parts])
    root = _group("SV08_Sherpa_Goliath", [new, gol, sher, fst, sv08])
    return root


if __name__ == "__main__":
    import extruder_body as eb
    import goliath_carrier as gc
    import housing as hs

    sv = sv08_placed()
    sp = sherpa_placed()
    new = [("MainBody", eb.body), ("PcbHolder", hs.pcb_holder), ("MotorPlate", hs.motor_plate), ("Goliath", gc.gol)]
    if "--nocheck" in sys.argv:
        sys.argv.append("--render")
        run_check = False
    else:
        run_check = True

    if run_check:
        bad = False
        import fasteners as fs
        others = [(n, t, p) for n, t, p in sv + sp] + [(n, "fastener", p) for n, p in fs.parts]
        for oname, obj in new:
            for n, t, p in others:
                if "heat_insert" in n:             # press fit (Ø5.0 in Ø4.8) is intended
                    continue
                if oname == "Goliath" and "screw" in n:     # thread engagement in the Goliath core hole
                    continue
                v = overlap(obj, p)
                if v > 0.05 or v != v:
                    bad = True
                    pb = p.bounding_box()
                    print(f"  COLLISION {oname} <-> {n} ({t}): {v:.2f} mm^3  "
                          f"[{pb.min.X:.0f},{pb.max.X:.0f}]x[{pb.min.Y:.0f},{pb.max.Y:.0f}]x[{pb.min.Z:.0f},{pb.max.Z:.0f}]")
        for i in range(len(new)):
            for j in range(i + 1, len(new)):
                v = overlap(new[i][1], new[j][1])
                print(f"  {new[i][0]} <-> {new[j][0]}: {v:.2f} mm^3")
                if v > 0.05:
                    bad = True
        # filament path (Ø1.75 + clearance) from the Goliath inlet to the top of the upper guide must be free
        from build123d import Cylinder, Align
        z_fil0 = gc.Z_BOT
        z_fil1 = hs.SHAFT[1] + hs.UPPER["top"]
        fil = Pos(cm.AX_X, cm.AX_Y, z_fil0) * Cylinder(FIL_R, z_fil1 - z_fil0,
                                                     align=(Align.CENTER, Align.CENTER, Align.MIN))
        for oname, obj in new:
            if oname == "Goliath":
                continue
            v = overlap(obj, fil)
            if v > 0.001:
                bad = True
                print(f"  FILAMENT PATH blocked by {oname}: {v:.3f} mm^3")
        print(f"Nozzle z={gc.nozzle_z:.2f} (original {cm.ORIG_NOZZLE_Z}); "
              f"motor axis x={sh.MOTOR_AX[0]:.2f} z={sh.MOTOR_AX[1]:.2f}")
        print("No collisions." if not bad else "See collisions above.")

    if "--render" in sys.argv:
        for fname, obj in (("extruder_body_sherpa", eb.body), ("pcb_holder", hs.pcb_holder), ("motor_plate", hs.motor_plate)):
            obj.label = fname
            export_step(obj, str(cm.OUT / f"{fname}.step"))
        ctx = [(p, "#b0b0b0", 0.30) for n, t, p in sv if -260 < p.bounding_box().center().Z < -90
               and p.bounding_box().size.Y < 120]
        ctx += [(p, "#c060c0", 0.8) for n, t, p in sp]
        items = ctx + [(gc.gol, "#d08030", 1.0), (eb.body, "#20a060", 0.9)]
        if "--png" in sys.argv:
            render(items, str(cm.OUT / "view_assembly.png"),
                   [(10, -90, "front"), (10, 0, "right (+X)"), (25, -130, "iso")])
            render([(eb.body, "#20a060", 1.0)], str(cm.OUT / "view_main_body.png"),
                   [(20, -60, "front"), (20, 120, "back")])
        export_step(assembly_tree(eb, gc, sv, sp), str(cm.OUT / "check_assembly.step"))
        print("check_assembly.step written")
