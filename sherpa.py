"""Sherpa Mini (Annex) placed in the SV08 coordinate system.

All parts of the Sherpa STEP are placed with ONE rigid transformation, so the relative
positions of motor, gears, drive rollers and filament axis are kept exactly.

Sherpa coordinates: filament runs along Y at (x=4.3, z=6.0), motor axis along Z,
motor mounting face at z=-7, motor at y=+13.85 above the drive shaft.
Mapping: X_sherpa -> X, Y_sherpa -> Z (motor on top), Z_sherpa -> -Y (motor at the back).

Drive gear and idler gear are flipped compared to the original (180° about a transverse axis
in their mid plane z=10). The filament groove is therefore at z=14 instead of z=6, and the whole
Sherpa including the motor sits 8 mm further back.
"""
from __future__ import annotations

import math
from functools import cached_property

from build123d import Compound, Location, Pos, Rot, Shape, import_step

import common
from geometry import XZ

# Parts identified by volume (labels in the STEP are partly duplicated)
_NAMES_BY_VOLUME = {
    5622: "housing_core", 821: "housing_front", 952: "housing_front_k", 2517: "housing_rear",
    1593: "idler_arm_short", 2128: "idler_arm_long", 18114: "motor",
}
_SCREWS_BY_VOLUME = {111: "screw_M3x12_motor", 140: "screw_M3x16_housing", 83: "screw_M3x8_lid"}
# Readable names for the leaves of the Sherpa STEP (label prefix -> name)
_NAMES_BY_LABEL = [
    ("shaft", "shaft"), ("0_3mm_shim", "shim_0_3"), ("sherpa_mini_gear_50t", "gear_50t"),
    ("filament_gear", "drive_gear"), ("MR85-2RS", "bearing_MR85"), ("m3_set_screw", "set_screw_M3"),
    ("idler_pin", "idler_pin"), ("idler_gear", "idler_gear"), ("needle_bearing", "needle_bearing"),
    ("insert_m3x5x4", "heat_insert_M3"), ("thumbscrew", "thumbscrew"), ("spring", "spring"),
    ("NEMA14_10t", "motor"),
]


class Sherpa:
    """Placement of the Sherpa Mini. Dimensions are available at once, the STEP is only loaded when
    `parts` is first used."""

    # in Sherpa coordinates
    FLIP_PLANE_Z = 10.0                       # mid plane of the flipped gears
    FLIPPED = {"drive_gear", "idler_gear", "needle_bearing", "set_screw_M3"}
    FILAMENT = (4.3, 2 * FLIP_PLANE_Z - 6.0)  # filament axis (x, z)
    MOTOR_ABOVE_SHAFT = 13.85                 # motor axis above the drive shaft (y)
    HOUSING_BOTTOM = -13.5                    # bottom of the housing core (y)
    MOTOR_FACE = -7.0                         # motor mounting face (z)

    CARRIER_THICKNESS = 5.0                   # Goliath carrier plate below the housing
    GOLIATH_LENGTH = 43.50 + 25.75            # top of heat sink -> nozzle tip
    LIFT = 1.0                                # Sherpa lifted above the carrier (clears the 50t gear)
    # Motor raised compared to the original STEP: centre distance pinion - 50t gear 15.0 mm
    # (module 0.5, 10/50 teeth) instead of 14.5 mm in the CAD model
    GEAR_CENTER_DISTANCE = 15.0

    # Parts dropped or replaced by the conversion (the housing is new)
    UNUSED = {"housing_core", "housing_rear", "housing_front", "housing_front_k", "idler_arm_long",
              "heat_insert_M3", "screw_M3x12_motor", "screw_M3x16_housing", "screw_M3x8_lid"}

    # ------------------------------------------------------------ placement
    # Housing bottom chosen so that the nozzle is at its original height
    housing_bottom_z = common.ORIGINAL_NOZZLE_Z + GOLIATH_LENGTH + CARRIER_THICKNESS
    origin = (common.FILAMENT_AXIS.x - FILAMENT[0],            # SV08 position of the Sherpa origin
              common.FILAMENT_AXIS.y + FILAMENT[1],
              housing_bottom_z - HOUSING_BOTTOM + LIFT)

    def to_sv08(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """Sherpa coordinates -> SV08 coordinates."""
        x0, y0, z0 = self.origin
        return (x0 + x, y0 - z, z0 + y)

    @property
    def placement(self) -> Location:
        return Location(self.origin) * Rot(90, 0, 0)            # Rx(90): y -> z, z -> -y

    @property
    def gear_flip(self) -> Location:
        z = self.FLIP_PLANE_Z
        return Location((0, 0, z)) * Rot(180, 0, 0) * Location((0, 0, -z))

    @property
    def motor_raise(self) -> float:
        """How far the motor is moved up to reach GEAR_CENTER_DISTANCE."""
        return math.sqrt(self.GEAR_CENTER_DISTANCE ** 2 - self.FILAMENT[0] ** 2) - self.MOTOR_ABOVE_SHAFT

    @property
    def motor_axis(self) -> XZ:
        x, _, z = self.to_sv08(self.FILAMENT[0], self.MOTOR_ABOVE_SHAFT, 0)
        return XZ(x, z + self.motor_raise)

    @property
    def motor_face_y(self) -> float:
        return self.to_sv08(0, 0, self.MOTOR_FACE)[1]

    # ------------------------------------------------------------ parts
    @cached_property
    def parts(self) -> list[tuple[str, Shape]]:
        """(name, shape) per leaf of the STEP tree, placed in SV08 coordinates."""
        assembly = import_step(str(common.SHERPA_STEP))
        solids = assembly.solids()       # same order as the leaves, locations already resolved
        parts, index = [], 0
        for leaf in _leaves(assembly):
            count = len(leaf.solids())
            group = list(solids[index:index + count])
            index += count
            name = _part_name(leaf.label.split("^")[0], sum(s.volume for s in group))
            flip = self.gear_flip if name in self.FLIPPED else Location()
            group = [self.placement * flip * solid for solid in group]
            if name == "motor":
                group = [Pos(0, 0, self.motor_raise) * solid for solid in group]
            parts.append((name, group[0] if count == 1 else Compound(group)))
        return parts

    def get(self, name: str) -> list[Shape]:
        return [shape for part_name, shape in self.parts if part_name == name]

    @cached_property
    def idler_insert(self) -> Shape:
        """Heat-set insert for the idler thumbscrew (sits in the housing core in the original)."""
        def is_thumbscrew_insert(shape):
            box = shape.bounding_box()
            return abs(box.center().X - (self.origin[0] + 9.2)) < 1.0 and box.size.X < box.size.Y
        return next(shape for shape in self.get("heat_insert_M3") if is_thumbscrew_insert(shape))

    def used_parts(self) -> list[tuple[str, Shape]]:
        """Sherpa parts that stay in the conversion, plus the thumbscrew insert."""
        return ([(name, shape) for name, shape in self.parts if name not in self.UNUSED]
                + [("heat_insert_idler", self.idler_insert)])


def _leaves(node):
    children = list(node.children)
    return [node] if not children else [leaf for child in children for leaf in _leaves(child)]


def _part_name(label: str, volume: float) -> str:
    def by_volume(table):
        return next((name for v, name in table.items() if abs(volume - v) < 2), None)

    name = by_volume(_NAMES_BY_VOLUME)
    if name is None and "screw" in label:
        name = by_volume(_SCREWS_BY_VOLUME)
    if name is None:
        name = next((n for prefix, n in _NAMES_BY_LABEL if label.startswith(prefix)), label)
        if name == "shaft":
            name = "drive_shaft" if volume > 300 else "idler_shaft"
    return name


SHERPA = Sherpa()

if __name__ == "__main__":
    print(f"Motor axis x={SHERPA.motor_axis.x:.2f} z={SHERPA.motor_axis.z:.2f}, "
          f"mounting face y={SHERPA.motor_face_y:.2f}")
    print(f"Housing bottom z={SHERPA.housing_bottom_z:.2f}")
    for name, shape in SHERPA.parts:
        b = shape.bounding_box()
        print(f"{name:16s} ({b.min.X:.1f},{b.min.Y:.1f},{b.min.Z:.1f})-({b.max.X:.1f},{b.max.Y:.1f},{b.max.Z:.1f})")
