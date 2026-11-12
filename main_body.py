"""Main body for the Sherpa conversion.

Block from the Eddy probe up to the Goliath carrier, with the width and depth of the original
extruder_install_plate, with all its holes, pockets for the screws of the neighbouring parts and box-shaped
recesses for the belt clamps (including the side flange of the sliding panel) and for the air duct.
The Goliath carrier (goliath_carrier.py) is fused with it; PCB holder and motor plate (housing.py)
are separate parts.

FDM: printed with the back side (y=BACK_Y) on the bed.
"""
from __future__ import annotations

from functools import cached_property

from build123d import Box, Compound, Cylinder, Pos, Rot, Shape, Solid

import common
from common import CLEARANCE
from geometry import XZ, bounding_boxes_overlap, box_between, cylinder_along_y, cylinder_along_z, fuse_all, largest_solid
from goliath_carrier import FULL_WIDTH, GOLIATH_CARRIER, MAIN_BODY_FRONT_Y
from housing import HEAT_INSERT_HOLE, HOUSING


class MainBody:
    # extent of the block: width and depth of the original install plate, from the Eddy probe to the carrier
    FRONT_Y, BACK_Y = MAIN_BODY_FRONT_Y, 250.7747
    TOP_Z = GOLIATH_CARRIER.TOP_Z
    BOTTOM_Z = -222.2                          # bottom of the Eddy probe

    # holes of the original install plate (axis along Y, all through): position and radius
    THROUGH_HOLES = {
        "carriage_1": (XZ(-361.525, -201.175), 1.75),     # M3 to the carriage
        "carriage_2": (XZ(-315.825, -194.675), 1.75),
        "carriage_3": (XZ(-310.825, -158.675), 1.75),
        "sensor": (XZ(-352.525, -215.975), 1.40),         # sensor holder
        "air_duct": (XZ(-352.525, -199.975), 2.00),       # rear air duct
        "rail_left": (XZ(-341.525, -202.175), 2.00),      # M3 from the back (sliding panel)
        "rail_right": (XZ(-325.525, -202.175), 2.00),
    }
    # screws of the old motor (dropped), identified by their position
    OLD_MOTOR_SCREWS = (XZ(-322.65, -133.35), XZ(-361.35, -153.95))

    @property
    def depth(self) -> tuple[float, float]:
        return (self.FRONT_Y, self.BACK_Y)

    def block_outline(self, x: tuple[float, float] = FULL_WIDTH) -> Solid:
        return box_between(x, self.depth, (self.BOTTOM_Z, self.TOP_Z))

    def is_old_motor_screw(self, shape: Shape) -> bool:
        center = shape.bounding_box().center()
        return any(abs(center.X - p.x) < 1.5 and abs(center.Z - p.z) < 1.5 for p in self.OLD_MOTOR_SCREWS)

    def pockets(self) -> list[Shape]:
        """Box-shaped recesses: belt clamps including the side flange of the sliding panel, open to the side
        and all the way through; air duct open to the bottom."""
        through_y = (self.FRONT_Y - 1, self.BACK_Y + 1)
        panel = common.find_sv08("xtruder_sliding_panel")[0]
        middle_x = FULL_WIDTH.middle
        pockets = []
        for clamp in common.find_sv08("belt_lock_block"):
            clamp_box = clamp.bounding_box()
            left = clamp_box.center().X < middle_x
            # include the side flange of the sliding panel on the same side (it sits in the plate)
            half = self.block_outline((FULL_WIDTH.start, middle_x) if left else (middle_x, FULL_WIDTH.end))
            flange = panel & half
            flange_box = Compound(list(flange.solids())).bounding_box() if flange and flange.solids() else clamp_box
            inner_x = (max(clamp_box.max.X, flange_box.max.X) + CLEARANCE if left
                       else min(clamp_box.min.X, flange_box.min.X) - CLEARANCE)
            x = (FULL_WIDTH.start - 1, inner_x) if left else (inner_x, FULL_WIDTH.end + 1)
            z = (min(clamp_box.min.Z, flange_box.min.Z) - CLEARANCE, max(clamp_box.max.Z, flange_box.max.Z) + CLEARANCE)
            pockets.append(box_between(x, through_y, z))
        # air duct: only the part that sits in the plate (the screws above it need material)
        duct = (common.find_sv08("air_deflector2")[0] & self.block_outline()).bounding_box()
        pockets.append(box_between((duct.min.X - CLEARANCE, duct.max.X + CLEARANCE), through_y,
                                   (self.BOTTOM_Z - 1, duct.max.Z + CLEARANCE)))
        return pockets

    def block(self) -> Shape:
        block = self.block_outline()
        for axis, radius in self.THROUGH_HOLES.values():
            block -= cylinder_along_y(axis, radius, self.FRONT_Y - 1, self.BACK_Y + 1)
        for pocket in self.pockets():
            block -= pocket
        # screws of neighbouring parts that reach into the block (counterbores/heads)
        for label, part in common.sv08_parts():
            if "screw" not in label or common.is_removed_sv08_part(label) or self.is_old_motor_screw(part):
                continue
            if not bounding_boxes_overlap(part, block):
                continue
            if any((s & block) and sum(x.volume for x in (s & block).solids()) > 0.01 for s in part.solids()):
                block -= stepped_cylinder(part, CLEARANCE)
        return block

    @cached_property
    def solid(self) -> Shape:
        body = self.block() + GOLIATH_CARRIER.solid      # the carrier is connected directly to the back wall
        carrier = GOLIATH_CARRIER
        for hole in HOUSING.lug_holes:                   # heat-set inserts for the lugs of the PCB holder
            body -= cylinder_along_z(hole, HEAT_INSERT_HOLE / 2, carrier.BOTTOM_Z - 1, carrier.TOP_Z + 1)
        # clearances for 50t gear, pinion, rear bearing and idler
        for cut in HOUSING.clearances:
            if cut.bounding_box().max.Y > HOUSING.PCB_PLATE_BACK_Y:
                body -= cut
        return largest_solid(body)


def stepped_cylinder(part: Shape, clearance: float, step: float = 0.5) -> Shape:
    """Substitute body for a screw: stepped smooth cylinders along its axis (radius per section from slices),
    enlarged by `clearance`. No thread faces."""
    box = part.bounding_box()
    sizes = (box.size.X, box.size.Y, box.size.Z)
    axis = sizes.index(max(sizes))
    start, end = (box.min.X, box.min.Y, box.min.Z)[axis], (box.max.X, box.max.Y, box.max.Z)[axis]
    center = box.center()
    to_axis = [Rot(0, 90, 0), Rot(-90, 0, 0), Rot(0, 0, 0)][axis]
    big = 400

    def point_on_axis(t):
        p = [center.X, center.Y, center.Z]
        p[axis] = t
        return p

    def radius_of_slice(t0, t1):
        size = [big, big, big]
        size[axis] = t1 - t0
        slab = Pos(*point_on_axis((t0 + t1) / 2)) * Box(*size)
        radius = 0.0
        for solid in part.solids():
            cut = solid & slab
            if cut and cut.solids():
                cut_box = Compound(list(cut.solids())).bounding_box()
                cross = [cut_box.size.X, cut_box.size.Y, cut_box.size.Z]
                del cross[axis]
                radius = max(radius, max(cross) / 2)
        return round(radius, 1)

    sections = []                  # [t0, t1, radius], neighbouring slices with equal radius merged
    t = start
    while t < end - 1e-6:
        t1 = min(t + step, end)
        radius = radius_of_slice(t, t1)
        if sections and sections[-1][2] == radius:
            sections[-1][1] = t1
        else:
            sections.append([t, t1, radius])
        t = t1
    return fuse_all(Pos(*point_on_axis((a + b) / 2)) * to_axis
                    * Cylinder(r + clearance, b - a + 2 * clearance * (a == start or b == end))
                    for a, b, r in sections if r > 0)


MAIN_BODY = MainBody()

if __name__ == "__main__":
    print(f"Volume {MAIN_BODY.solid.volume:.0f} mm^3, faces {len(MAIN_BODY.solid.faces())}")
