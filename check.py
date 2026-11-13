"""Collision check of the Sherpa/Goliath conversion.

Usage:  .venv/bin/python check.py
Checks the new parts and the Goliath against all SV08 and Sherpa parts, the fasteners and each other,
and checks that the filament path is free. Exit code 1 if anything collides.
"""
import sys
from itertools import combinations

from build123d import Shape

import common
from assembly import TOOLHEAD
from geometry import bounding_boxes_overlap, cylinder_along_z
from goliath_carrier import GOLIATH_CARRIER
from housing import PCB_HOLDER
from sherpa import SHERPA

FILAMENT_RADIUS = 0.95             # filament Ø1.75 with some clearance (guide Ø2.1)
TOLERANCE = 0.05                   # mm^3 below which an overlap is ignored (touching faces)


def overlap_volume(a: Shape, b: Shape) -> float:
    """Volume of the intersection; NaN if the boolean operation fails."""
    if not bounding_boxes_overlap(a, b):
        return 0.0
    volume = 0.0
    for solid_a in a.solids():
        for solid_b in b.solids():
            try:
                common_part = solid_a.intersect(solid_b)
                volume += sum(s.volume for s in common_part.solids()) if common_part else 0
            except Exception:
                return float("nan")
    return volume


def is_collision(volume: float) -> bool:
    return volume > TOLERANCE or volume != volume       # NaN counts as a collision


def is_intended_contact(new_part: str, other: str) -> bool:
    if "heat_insert" in other:                          # press fit (Ø5.0 in Ø4)
        return True
    return new_part == "Goliath" and "screw" in other   # thread engagement in the Goliath core holes


def check() -> bool:
    ok = True
    others = ([(p.name, p.tag, p.shape) for p in TOOLHEAD.sv08_parts + TOOLHEAD.sherpa_parts]
              + [(name, "fastener", shape) for name, shape in TOOLHEAD.fasteners])
    for new_name, new_shape in TOOLHEAD.new_parts:
        for name, tag, shape in others:
            if is_intended_contact(new_name, name):
                continue
            volume = overlap_volume(new_shape, shape)
            if is_collision(volume):
                ok = False
                b = shape.bounding_box()
                print(f"  COLLISION {new_name} <-> {name} ({tag}): {volume:.2f} mm^3  "
                      f"[{b.min.X:.0f},{b.max.X:.0f}]x[{b.min.Y:.0f},{b.max.Y:.0f}]x[{b.min.Z:.0f},{b.max.Z:.0f}]")
    for (name_a, shape_a), (name_b, shape_b) in combinations(TOOLHEAD.new_parts, 2):
        volume = overlap_volume(shape_a, shape_b)
        print(f"  {name_a} <-> {name_b}: {volume:.2f} mm^3")
        if volume > TOLERANCE:
            ok = False
    # filament path from the Goliath inlet to the top of the upper guide must be free
    filament = cylinder_along_z(common.FILAMENT_AXIS, FILAMENT_RADIUS, GOLIATH_CARRIER.BOTTOM_Z,
                                PCB_HOLDER.guides.top_z)
    for name, shape in TOOLHEAD.new_parts:
        if name == "Goliath":
            continue
        volume = overlap_volume(shape, filament)
        if volume > 0.001:
            ok = False
            print(f"  FILAMENT PATH blocked by {name}: {volume:.3f} mm^3")
    print(f"Nozzle z={GOLIATH_CARRIER.nozzle_z:.2f} (original {common.ORIGINAL_NOZZLE_Z}); "
          f"motor axis x={SHERPA.motor_axis.x:.2f} z={SHERPA.motor_axis.z:.2f}")
    print("No collisions." if ok else "See collisions above.")
    return ok


if __name__ == "__main__":
    sys.exit(0 if check() else 1)
