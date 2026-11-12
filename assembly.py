"""Complete assembly of the Sherpa/Goliath conversion: placed SV08 and Sherpa parts, new parts, fasteners,
and the named, coloured STEP tree. Used by render.py and check.py."""
from __future__ import annotations

import re
from dataclasses import dataclass
from functools import cached_property

from build123d import Box, Color, Compound, Location, Pos, Shape

import common
from fasteners import FASTENERS
from goliath_carrier import GOLIATH_CARRIER
from housing import MOTOR_PLATE, PCB_HOLDER
from main_body import MAIN_BODY
from sherpa import SHERPA

RGB = tuple[float, float, float]


@dataclass(frozen=True)
class PlacedPart:
    name: str
    tag: str          # what was done to the part ("original", "PCB raised", ...)
    shape: Shape


@dataclass(frozen=True)
class RemovedScrew:
    """Dropped SV08 screw, identified by its position (x, z) and the axis it points along."""
    x: float
    z: float
    axis: str


# old motor screws (Y), M3x12 of the old cable clamp (X, reaches into the Sherpa from the side),
# M2.5 of the 5020 (fan no longer attached to the carrier)
REMOVED_SCREWS = [RemovedScrew(-322.65, -133.35, "Y"), RemovedScrew(-361.35, -153.95, "Y"),
                  RemovedScrew(-347.2, -129.65, "X"), RemovedScrew(-360.85, -166.95, "Y")]
FRONT_5020_PARTS = ["5020_turbofan", "air_duct_5020", "cover_plate3", "M2_5X8"]
FAN_2515 = "2515_pipe_cooling_fan"

# ---------------------------------------------------------------- colours and names
NEW_PART_COLORS = {"MainBody": (0.15, 0.65, 0.35), "PcbHolder": (0.20, 0.55, 0.80), "MotorPlate": (0.85, 0.75, 0.20)}
GOLIATH_COLOR = (0.85, 0.50, 0.15)
SILVER, GOLD = (0.80, 0.80, 0.82), (0.85, 0.75, 0.25)   # screws / heat-set inserts
SHERPA_COLORS = [("heat_insert", GOLD), ("motor", (0.25, 0.25, 0.28)), ("gear_50t", (0.80, 0.65, 0.25)),
                 ("drive_gear", (0.80, 0.65, 0.25)), ("idler_gear", (0.80, 0.65, 0.25)), ("idler_arm", (0.70, 0.30, 0.75)),
                 ("bearing", (0.75, 0.75, 0.78)), ("shaft", (0.60, 0.60, 0.62)), ("thumbscrew", (0.10, 0.10, 0.10))]
SHERPA_DEFAULT_COLOR = (0.60, 0.40, 0.70)
SV08_COLORS = [("turbofan", (0.12, 0.12, 0.12)), ("cooling_fan", (0.12, 0.12, 0.12)), ("duct", (0.55, 0.75, 0.90)),
               ("deflector", (0.55, 0.75, 0.90)), ("cover_plate", (0.55, 0.75, 0.90)), ("adapter_plate", (0.20, 0.45, 0.20)),
               ("proximity", (0.85, 0.45, 0.10)), ("sliding", (0.55, 0.55, 0.60)), ("MGN", (0.70, 0.70, 0.72)),
               ("screw", SILVER), ("magnet", (0.40, 0.40, 0.45))]
SV08_DEFAULT_COLOR = (0.70, 0.70, 0.70)

# label fragment -> readable name for the partly garbled Sovol labels
NAME_OVERRIDES = [
    ("3x8_4xM3", "shoulder_screw_M3x8_pcb"), ("2_5x5xM2", "shoulder_screw_M2_5x5"),
    ("plain_washer", "washer"), ("proximity_switch", "eddy_probe"),
    ("MGN12H", "MGN12H_carriage"), ("2515_pipe_cooling_fan", "fan_2515_hotend"),
    ("3010_turbofan", "fan_3010_turbo"), ("5020_turbofan", "fan_5020_part_cooling"),
    ("ptfe_tube_quick_coupling", "ptfe_quick_coupling"), ("quick_plug_adapter", "ptfe_clip"),
    ("length16mm", "ptfe_tube_16mm"), ("inner_diameter2mm_ptfe", "ptfe_tube"),
]


def readable_name(label: str) -> str:
    """Readable name from a Sovol label."""
    ascii_label = re.sub(r"[^ -~]", "", label or "")
    lower = ascii_label.lower()
    for key, name in NAME_OVERRIDES:
        if key in ascii_label:
            return name
    size = re.search(r"M\d(?:_\d)?[xX]\d+", ascii_label)
    if size and "screw" in lower:
        kind = ("countersunk" if "countersunk" in lower else "shoulder" if "shoulder" in lower
                else "set_screw" if "set_screw" in lower else "socket_head")
        return f"screw_{size.group(0).replace('X', 'x')}_{kind}"
    name = re.sub(r"^[A-Z0-9]+-[0-9]+-[a-z]_", "", ascii_label)
    name = re.sub(r"^SV05p", "", name)
    name = re.sub(r"^[^A-Za-z]+", "", name)
    if name.startswith("xtruder"):
        name = "e" + name
    return name.strip("_")[:40] or "part"


def color_for(name: str, table: list[tuple[str, RGB]], default: RGB) -> RGB:
    return next((rgb for key, rgb in table if key.lower() in name.lower()), default)


def named(shape: Shape, name: str, rgb: RGB) -> Shape:
    """Copy of the shape with label and colour (also on the children of multi-solid parts like bearings)."""
    shape = shape.moved(Location()) if hasattr(shape, "moved") else shape
    shape.label = name
    shape.color = Color(*rgb)
    for i, child in enumerate(getattr(shape, "children", ()) or ()):
        child.color = Color(*rgb)
        child.label = f"{name}_{i + 1}"
    return shape


def group(name: str, children: list[Shape]) -> Compound:
    compound = Compound(children=children)
    compound.label = name
    return compound


def with_unique_names(items: list[tuple[str, Shape, RGB]]) -> list[Shape]:
    """Number repeated names: screw, screw_2, screw_3, ..."""
    seen: dict[str, int] = {}
    shapes = []
    for name, shape, rgb in items:
        seen[name] = seen.get(name, 0) + 1
        shapes.append(named(shape, name if seen[name] == 1 else f"{name}_{seen[name]}", rgb))
    return shapes


def is_removed_screw(label: str, shape: Shape) -> bool:
    if "screw" not in label:
        return False
    box = shape.bounding_box()
    sizes = [box.size.X, box.size.Y, box.size.Z]
    axis = "XYZ"[sizes.index(max(sizes))]
    center = box.center()
    return any(axis == s.axis and abs(center.X - s.x) < 1.5 and abs(center.Z - s.z) < 1.5 for s in REMOVED_SCREWS)


class Toolhead:
    """The converted toolhead. Every part is built on first use."""

    @cached_property
    def new_parts(self) -> list[tuple[str, Shape]]:
        """New printed parts and the Goliath (the parts checked for collisions)."""
        return [("MainBody", MAIN_BODY.solid), ("PcbHolder", PCB_HOLDER.solid), ("MotorPlate", MOTOR_PLATE.solid),
                ("Goliath", GOLIATH_CARRIER.goliath)]

    @property
    def printed_parts(self) -> list[tuple[str, Shape]]:
        """(file name, shape) of the parts to print."""
        return [("extruder_body_sherpa", MAIN_BODY.solid), ("pcb_holder", PCB_HOLDER.solid),
                ("motor_plate", MOTOR_PLATE.solid)]

    @property
    def fasteners(self) -> list[tuple[str, Shape]]:
        return FASTENERS.parts

    @cached_property
    def sv08_parts(self) -> list[PlacedPart]:
        """Remaining SV08 parts at their new position."""
        pcb_raise = Pos(0, 0, GOLIATH_CARRIER.PCB_RAISE)
        parts = []
        for label, shape in common.sv08_parts():
            z = shape.bounding_box().center().Z
            if "M3x12" in label and z < -196:                       # sensor/deflector screws stay
                parts.append(PlacedPart(label, "original", shape))
                continue
            if ("shoulder_screws" in label and "4xM3" in label) or "adapter_plate" in label:
                parts.append(PlacedPart(label, "PCB raised", pcb_raise * shape))   # PCBs and their screws
                continue
            if common.is_removed_sv08_part(label) or is_removed_screw(label, shape):
                continue
            if "proximity_switch" in label:                          # remove the cable of the Eddy probe
                shape = shape - Pos(-352.5, 239.8, -182.5) * Box(3.4, 3.4, 15.9)
            if any(key in label for key in FRONT_5020_PARTS):
                parts.append(PlacedPart(label, "5020 forward", Pos(*GOLIATH_CARRIER.FAN5020_SHIFT) * shape))
            elif FAN_2515 in label:
                parts.append(PlacedPart(label, "2515 mirrored", self._mirrored_2515(shape)))
            else:
                parts.append(PlacedPart(label, "original", shape))
        return parts

    @staticmethod
    def _mirrored_2515(fan: Shape) -> Shape:
        """Remove the cable (4 thin wires upwards), move below the carrier and mirror to the -X side."""
        solids = fan.solids()
        fan = Compound([solids[0], solids[1] - Pos(-323.65, 234.75, -155.0) * Box(1.2, 1.6, 20.2)])
        carrier = GOLIATH_CARRIER
        return (Pos(carrier.FAN_SHIFT_X, 0, carrier.FAN_RAISE) * fan).mirror(carrier.FAN_MIRROR)

    @cached_property
    def sherpa_parts(self) -> list[PlacedPart]:
        return [PlacedPart(f"sherpa_{name}", "Sherpa", shape) for name, shape in SHERPA.used_parts()]

    def step_tree(self) -> Compound:
        """Assembly as a tree: new parts / Goliath / Sherpa / fasteners / SV08, with names and colours."""
        new = group("NewParts", [named(shape, name, NEW_PART_COLORS[name]) for name, shape in self.new_parts
                                 if name in NEW_PART_COLORS])
        goliath = group("Goliath", [named(GOLIATH_CARRIER.goliath, "Goliath_AIR", GOLIATH_COLOR)])
        sherpa = group("Sherpa", with_unique_names(
            [(p.name.removeprefix("sherpa_"), p.shape, color_for(p.name, SHERPA_COLORS, SHERPA_DEFAULT_COLOR))
             for p in self.sherpa_parts]))
        fasteners = group("Fasteners", [named(shape, name, GOLD if "insert" in name else SILVER)
                                        for name, shape in self.fasteners])
        sv08 = group("SV08", with_unique_names(
            [(readable_name(p.name), p.shape, color_for(p.name, SV08_COLORS, SV08_DEFAULT_COLOR))
             for p in self.sv08_parts]))
        return group("SV08_Sherpa_Goliath", [new, goliath, sherpa, fasteners, sv08])


TOOLHEAD = Toolhead()
