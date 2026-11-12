"""Small value types and primitive builders shared by all part classes.

Coordinates are those of the Sovol STEP assembly (mm): X to the right, Y to the back, Z up.
Most features of the toolhead are cylinders along Y (shafts, bearings, screws through the plates),
so their position is given as a point in the XZ plane.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, NamedTuple

from build123d import Align, Box, Cylinder, Location, Pos, Rot, Shape, Solid

ALIGN_BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)     # cylinder/box starts at its position


class XZ(NamedTuple):
    """Point in the XZ plane (front view), e.g. the position of an axis along Y."""
    x: float
    z: float

    def shifted(self, dx: float = 0.0, dz: float = 0.0) -> XZ:
        return XZ(self.x + dx, self.z + dz)


class XY(NamedTuple):
    """Point in the XY plane (top view), e.g. the position of a vertical axis."""
    x: float
    y: float


class Span(NamedTuple):
    """Interval along one axis."""
    start: float
    end: float

    @property
    def length(self) -> float:
        return self.end - self.start

    @property
    def middle(self) -> float:
        return (self.start + self.end) / 2


@dataclass(frozen=True)
class Boss:
    """Round boss around a hole: centre in the XZ plane and outer radius."""
    center: XZ
    radius: float


@dataclass(frozen=True)
class Rect:
    """Rectangle in the XZ plane (front view of a plate)."""
    x: Span
    z: Span


def box_between(x: tuple[float, float], y: tuple[float, float], z: tuple[float, float]) -> Solid:
    """Axis-aligned box from its corner coordinates (each argument is (min, max))."""
    (x0, x1), (y0, y1), (z0, z1) = x, y, z
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def cylinder_along_y(axis: XZ, radius: float, y_from: float, y_to: float) -> Solid:
    """Cylinder with its axis along Y at `axis`, from y_from to y_to."""
    return Pos(axis.x, y_from, axis.z) * Rot(-90, 0, 0) * Cylinder(radius, y_to - y_from, align=ALIGN_BOTTOM)


def cylinder_along_z(axis: XY, radius: float, z_from: float, z_to: float) -> Solid:
    """Vertical cylinder at `axis`, from z_from to z_to."""
    return Pos(axis.x, axis.y, z_from) * Cylinder(radius, z_to - z_from, align=ALIGN_BOTTOM)


def cylinder_along_x(start: tuple[float, float, float], radius: float, length: float) -> Solid:
    """Cylinder along +X starting at `start` (x, y, z)."""
    return Pos(*start) * Rot(0, 90, 0) * Cylinder(radius, length, align=ALIGN_BOTTOM)


def rotation_about_y(pivot: XZ, degrees: float) -> Location:
    """Rotation about an axis along Y through `pivot`; positive = clockwise seen from the front."""
    return Pos(pivot.x, 0, pivot.z) * Rot(0, degrees, 0) * Pos(-pivot.x, 0, -pivot.z)


def fuse_all(shapes: Iterable[Shape]) -> Shape:
    """Boolean union of all shapes, in order."""
    result = None
    for shape in shapes:
        result = shape if result is None else result + shape
    return result


def largest_solid(shape: Shape) -> Solid:
    """Clean the shape and keep only its largest solid (drops slivers left by the booleans)."""
    return max(shape.clean().solids(), key=lambda solid: solid.volume)


def bounding_boxes_overlap(a: Shape, b: Shape) -> bool:
    ba, bb = a.bounding_box(), b.bounding_box()
    return not (ba.min.X > bb.max.X or ba.max.X < bb.min.X or ba.min.Y > bb.max.Y or ba.max.Y < bb.min.Y
                or ba.min.Z > bb.max.Z or ba.max.Z < bb.min.Z)
