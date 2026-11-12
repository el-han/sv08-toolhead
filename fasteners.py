"""Screws and heat-set inserts of the new parts, for the assembly and the collision check.

The models are the purchased parts from the original STEPs: socket head cap screws DIN 912 from the SV08
(M3x12 and M2.5x8, cut to the required length) and the heat-set insert M3x5x4 from the Sherpa.
The press fit of the inserts in their holes is intended (check.py ignores "heat_insert").
"""
from __future__ import annotations

from functools import cache, cached_property

from build123d import Align, Box, Pos, Rot, Shape

import common
from geometry import XZ
from goliath_carrier import FULL_WIDTH, GOLIATH_CARRIER
from housing import HOUSING
from sherpa import SHERPA

Vector3 = tuple[float, float, float]

_AXIS_TO_Z = [Rot(0, -90, 0), Rot(90, 0, 0), Rot(0, 0, 0)]       # axis x / y / z -> +z
_Z_TO_DIRECTION = {(1, 0, 0): Rot(0, 90, 0), (-1, 0, 0): Rot(0, -90, 0),
                   (0, 1, 0): Rot(-90, 0, 0), (0, -1, 0): Rot(90, 0, 0),
                   (0, 0, 1): Rot(0, 0, 0), (0, 0, -1): Rot(180, 0, 0)}
# SV08 screw models: label, total length, head diameter
_SCREW_MODELS = {"M3": ("hex_socket_cylindrical_head_screws", 15.0, 5.5),
                 "M2.5": ("hex_socket_cylindrical_head_screws", 10.5, 4.5)}

FRONT, BACK, UP, DOWN, RIGHT = (0, -1, 0), (0, 1, 0), (0, 0, 1), (0, 0, -1), (1, 0, 0)


def _radius_at(shape: Shape, z: float) -> float:
    """Radius of the shape (axis along z) in a thin slice at height z."""
    sliced = shape & (Pos(0, 0, z) * Box(40, 40, 0.05))
    if not (sliced and sliced.solids()):
        return 0.0
    box = sliced.bounding_box()
    return max(box.size.X, box.size.Y) / 2


def _onto_z_axis(part: Shape, longest: bool = True) -> tuple[Shape, float]:
    """Move the part onto the z axis (centre at the origin); its axis is the longest or shortest extent.
    Returns the moved part and its length along the axis."""
    box = part.bounding_box()
    sizes = [box.size.X, box.size.Y, box.size.Z]
    axis = sizes.index(max(sizes) if longest else min(sizes))
    center = box.center()
    return _AXIS_TO_Z[axis] * (Pos(-center.X, -center.Y, -center.Z) * part), sizes[axis]


@cache
def _screw_template(kind: str) -> tuple[Shape, float]:
    """Screw with the head bearing face at z=0, head at z<0, shank along +z. Returns (shape, shank length)."""
    label, total_length, head_diameter = _SCREW_MODELS[kind]

    def matches(shape):
        sizes = sorted([shape.bounding_box().size.X, shape.bounding_box().size.Y, shape.bounding_box().size.Z])
        return abs(sizes[2] - total_length) < 0.05 and abs(sizes[1] - head_diameter) < 0.05

    model = next(shape for name, shape in common.sv08_parts() if label in name and matches(shape))
    screw, length = _onto_z_axis(model)
    if _radius_at(screw, length / 2 - 0.3) > _radius_at(screw, -length / 2 + 0.3):     # head downwards
        screw = Rot(180, 0, 0) * screw
    z = -length / 2
    while _radius_at(screw, z + 0.1) > head_diameter / 2 - 0.4:                     # find the head bearing face
        z += 0.1
    head_face = z + 0.1
    return Pos(0, 0, -head_face) * screw, length - (head_face + length / 2)


@cache
def _insert_template() -> Shape:
    """Heat-set insert M3x5x4 (Sherpa) from z=0 to z=4."""
    insert, length = _onto_z_axis(SHERPA.idler_insert, longest=False)
    return Pos(0, 0, length / 2) * insert


def screw(head_face: Vector3, direction: Vector3, length: float, kind: str = "M3") -> Shape:
    """Socket head cap screw with its head bearing face at `head_face`, shank of `length` in `direction`."""
    template, max_length = _screw_template(kind)
    assert length <= max_length + 0.15, (kind, length, max_length)   # head face found to within 0.1 mm
    shortened = template - Pos(0, 0, length) * Box(40, 40, 40, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return Pos(*head_face) * _Z_TO_DIRECTION[direction] * shortened


def heat_insert(opening: Vector3, direction: Vector3) -> Shape:
    """Heat-set insert M3x5x4, starting at the hole opening, going in `direction` into the part."""
    return Pos(*opening) * _Z_TO_DIRECTION[direction] * _insert_template()


def _at_y(point: XZ, y: float) -> Vector3:
    return (point.x, y, point.z)


class Fasteners:
    """All screws and heat-set inserts of the new parts, as (name, shape)."""

    @cached_property
    def parts(self) -> list[tuple[str, Shape]]:
        layout, carrier = HOUSING, GOLIATH_CARRIER
        motor_face = SHERPA.motor_face_y
        parts = []
        # PCB holder <-> motor plate: insert in the post, screw from the motor side
        for i, point in enumerate(layout.post_screws, 1):
            parts.append((f"heat_insert_post_{i}", heat_insert(_at_y(point, layout.POST_PARTING_Y), FRONT)))
            parts.append((f"screw_M3x12_post_{i}", screw(_at_y(point, motor_face), FRONT, 12)))
        # motor: 2x M3x6 from the front through the motor plate (the left one through the slot)
        plate_front = motor_face - layout.MOTOR_PLATE_THICKNESS
        for side, dx in (("right", layout.MOTOR_EAR_DX), ("left", -layout.MOTOR_EAR_DX)):
            parts.append((f"screw_M3x6_motor_{side}",
                          screw(_at_y(SHERPA.motor_axis.shifted(dx=dx), plate_front), BACK, 6)))
        # PCB shoulder screws: upper insert in the PCB holder, lower one in the Goliath carrier
        for i, point in enumerate((layout.PCB_INSERT, carrier.PCB_SCREW), 1):
            parts.append((f"heat_insert_pcb_{i}", heat_insert(_at_y(point, layout.PCB_PLATE_FRONT_Y), BACK)))
        # lugs of the PCB holder: M3x8 from the top, heat-set insert from below in the Goliath carrier
        for i, hole in enumerate(layout.lug_holes, 1):
            parts.append((f"screw_M3x8_lug_{i}",
                          screw((hole.x, hole.y, layout.BOTTOM_Z + layout.LUG_THICKNESS), DOWN, 8)))
            parts.append((f"heat_insert_lug_{i}", heat_insert((hole.x, hole.y, carrier.BOTTOM_Z), UP)))
        # 2515 fan: 2x M3x12 from the outside through the wedges into the upper fan holes
        for i, (y, z) in enumerate(carrier.FAN_HOLES, 1):
            parts.append((f"screw_M3x12_fan_{i}", screw((FULL_WIDTH.start, y, z), RIGHT, 12)))
        # Goliath: 4x M2.5x5 from the top (2.6 mm in the thread, core hole 3.4 mm deep), head in the counterbore
        for i, position in enumerate(carrier.goliath_screw_positions(), 1):
            parts.append((f"screw_M2_5x5_goliath_{i}",
                          screw((position.x, position.y, carrier.TOP_Z - carrier.HEAD_DEPTH), DOWN, 5, "M2.5")))
        return parts


FASTENERS = Fasteners()
