"""Complete assembly of the Sherpa/Goliath conversion: placed SV08 and Sherpa parts, new parts, fasteners,
and the named, coloured STEP tree. Used by render.py and check.py."""
import re

from build123d import Box, Color, Compound, Location, Pos

import common as cm
import extruder_body as eb
import fasteners as fs
import goliath_carrier as gc
import housing as hs
import sherpa as sh

FRONT5020 = ["5020_turbofan", "air_duct_5020", "cover_plate3", "M2_5X8"]
FAN2515 = "2515_pipe_cooling_fan"

# Dropped SV08 screws, identified by position (x, z, axis):
#   old motor screws (Y) and M3x12 of the old cable clamp (X, reach into the Sherpa from the side)
REMOVED_SCREWS = [(-322.65, -133.35, "Y"), (-361.35, -153.95, "Y"), (-347.2, -129.65, "X"),
                  (-360.85, -166.95, "Y")]   # M2.5 of the 5020 (fan no longer attached to the carrier)

# New printed parts and the Goliath (the parts checked for collisions)
NEW_PARTS = [("MainBody", eb.body), ("PcbHolder", hs.pcb_holder), ("MotorPlate", hs.motor_plate),
             ("Goliath", gc.gol)]


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


# ---------------------------------------------------------------- STEP tree
NEW_COLORS = {"MainBody": (0.15, 0.65, 0.35), "PcbHolder": (0.20, 0.55, 0.80), "MotorPlate": (0.85, 0.75, 0.20)}
SILVER, GOLD = (0.80, 0.80, 0.82), (0.85, 0.75, 0.25)   # screws / heat-set inserts
SHERPA_COLORS = [("heat_insert", GOLD), ("motor", (0.25, 0.25, 0.28)), ("gear_50t", (0.80, 0.65, 0.25)),
                 ("drive_gear", (0.80, 0.65, 0.25)), ("idler_gear", (0.80, 0.65, 0.25)), ("idler_arm", (0.70, 0.30, 0.75)),
                 ("bearing", (0.75, 0.75, 0.78)), ("shaft", (0.60, 0.60, 0.62)), ("thumbscrew", (0.10, 0.10, 0.10))]
SV08_COLORS = [("turbofan", (0.12, 0.12, 0.12)), ("cooling_fan", (0.12, 0.12, 0.12)), ("duct", (0.55, 0.75, 0.90)),
               ("deflector", (0.55, 0.75, 0.90)), ("cover_plate", (0.55, 0.75, 0.90)), ("adapter_plate", (0.20, 0.45, 0.20)),
               ("proximity", (0.85, 0.45, 0.10)), ("sliding", (0.55, 0.55, 0.60)), ("MGN", (0.70, 0.70, 0.72)),
               ("screw", SILVER), ("magnet", (0.40, 0.40, 0.45))]

NAME_OVERRIDES = [
    ("3x8_4xM3", "shoulder_screw_M3x8_pcb"), ("2_5x5xM2", "shoulder_screw_M2_5x5"),
    ("plain_washer", "washer"), ("proximity_switch", "eddy_probe"),
    ("MGN12H", "MGN12H_carriage"), ("2515_pipe_cooling_fan", "fan_2515_hotend"),
    ("3010_turbofan", "fan_3010_turbo"), ("5020_turbofan", "fan_5020_part_cooling"),
    ("ptfe_tube_quick_coupling", "ptfe_quick_coupling"), ("quick_plug_adapter", "ptfe_clip"),
    ("length16mm", "ptfe_tube_16mm"), ("inner_diameter2mm_ptfe", "ptfe_tube"),
]


def _named(shape, name, rgb):
    s = shape.moved(Location()) if hasattr(shape, "moved") else shape
    s.label = name
    s.color = Color(*rgb)
    for i, c in enumerate(getattr(s, "children", ()) or ()):   # multi-solid parts (bearings)
        c.color = Color(*rgb)
        c.label = f"{name}_{i + 1}"
    return s


def nice_name(label):
    """Readable name from the partly garbled Sovol labels."""
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


def assembly_tree(sv, sp):
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
    fst = _group("Fasteners", [_named(p, n, GOLD if "insert" in n else SILVER) for n, p in fs.parts])
    sv08 = _group("SV08", unique([(nice_name(n), p, pick(n, SV08_COLORS, (0.70, 0.70, 0.70))) for n, t, p in sv]))
    return _group("SV08_Sherpa_Goliath", [new, gol, sher, fst, sv08])
