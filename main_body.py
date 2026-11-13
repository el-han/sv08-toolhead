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
from geometry import XZ, bounding_boxes_overlap, box_between, cylinder_along_x, cylinder_along_y, cylinder_along_z, fuse_all, largest_solid
from goliath_carrier import FULL_WIDTH, GOLIATH_CARRIER, MAIN_BODY_FRONT_Y
from housing import HEAT_INSERT_HOLE, HEAT_INSERT_HOLE_DEPTH, HOUSING


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
    # fan duct: strip of the right side wall below the sliding panel recess, M3 from the side into an insert in a
    # block on the front face (below the head of the carriage screw at z -197.4)
    COVER_SCREW_Z = -201.8

    # side holes for the sliding panel: in the inner walls of the belt clamp recesses, on the axes of the Ø2.5 holes
    # in the panel flanges, open to the back face (the panel slides in from the back)
    PANEL_HOLE_Y = 246.37
    PANEL_HOLE_Z = (-165.67, -187.67)
    PANEL_HOLE, PANEL_HOLE_DEPTH = 5.0, 5.0

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

    @cached_property
    def right_pocket_bottom_z(self) -> float:
        """Lower edge of the recess for the right belt clamp / sliding panel flange."""
        right = [p.bounding_box() for p in self.pockets() if p.bounding_box().max.X > FULL_WIDTH.end]
        return right[0].min.Z

    @property
    def cover_insert_x(self) -> float:
        """Outer face of the insert block (fan duct side wall + clearance outside of it)."""
        return FULL_WIDTH.end - HOUSING.COVER_SIDE_WALL - HOUSING.COVER_CLEARANCE

    @property
    def cover_insert_y(self) -> float:
        return self.FRONT_Y - HEAT_INSERT_HOLE / 2 - HOUSING.COVER_INSERT_WALL

    @property
    def cover_block_z(self) -> tuple[float, float]:
        half = HEAT_INSERT_HOLE / 2 + HOUSING.COVER_INSERT_WALL
        return (self.COVER_SCREW_Z - half, self.COVER_SCREW_Z + half)

    def cover_insert_block(self) -> Shape:
        x, wall = self.cover_insert_x, HOUSING.COVER_INSERT_WALL
        block = box_between((x - HEAT_INSERT_HOLE_DEPTH - wall, x),
                            (self.cover_insert_y - HEAT_INSERT_HOLE / 2 - wall, self.FRONT_Y + 0.01), self.cover_block_z)
        return block - cylinder_along_x((x - HEAT_INSERT_HOLE_DEPTH, self.cover_insert_y, self.COVER_SCREW_Z),
                                        HEAT_INSERT_HOLE / 2, HEAT_INSERT_HOLE_DEPTH + 0.1)

    @property
    def lug_boss_bottom_z(self) -> float:
        """Bottom of the bosses below the lug inserts: insert hole depth from the top of the carrier."""
        return GOLIATH_CARRIER.TOP_Z - HEAT_INSERT_HOLE_DEPTH

    def panel_holes(self) -> list[Shape]:
        """Ø5 x 5 side holes from the inner walls of the two side recesses inwards, open to the back."""
        radius, depth = self.PANEL_HOLE / 2, self.PANEL_HOLE_DEPTH
        holes = []
        for pocket in self.pockets()[:2]:
            box = pocket.bounding_box()
            left = box.max.X < FULL_WIDTH.middle
            wall_x = box.max.X if left else box.min.X
            x = (wall_x - 0.1, wall_x + depth) if left else (wall_x - depth, wall_x + 0.1)
            for z in self.PANEL_HOLE_Z:
                holes.append(cylinder_along_x((x[0], self.PANEL_HOLE_Y, z), radius, x[1] - x[0]))
                holes.append(box_between(x, (self.PANEL_HOLE_Y, self.BACK_Y + 1), (z - radius, z + radius)))
        return holes

    def block(self) -> Shape:
        block = self.block_outline()
        for axis, radius in self.THROUGH_HOLES.values():
            block -= cylinder_along_y(axis, radius, self.FRONT_Y - 1, self.BACK_Y + 1)
        for pocket in self.pockets():
            block -= pocket
        for hole in self.panel_holes():
            block -= hole
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
        body += self.cover_insert_block()
        carrier = GOLIATH_CARRIER
        # heat-set inserts for the lugs of the PCB holder: the carrier is thinner than the insert hole is deep, so a
        # boss below it
        for hole in HOUSING.lug_holes:
            body += cylinder_along_z(hole, HEAT_INSERT_HOLE / 2 + HOUSING.COVER_INSERT_WALL, self.lug_boss_bottom_z,
                                     carrier.BOTTOM_Z + 0.01)
            if hole.x < FULL_WIDTH.middle:               # left: close the gap to the 45° flank of the front fan wedge
                radius = HEAT_INSERT_HOLE / 2 + HOUSING.COVER_INSERT_WALL
                body += box_between((FULL_WIDTH.start, carrier.FAN_BACK_X - 0.1), (hole.y - radius - 3.0, hole.y),
                                    (self.lug_boss_bottom_z, carrier.BOTTOM_Z + 0.01))
            # exactly as deep as the insert hole (does not cut into the fan wedge below the boss)
            body -= cylinder_along_z(hole, HEAT_INSERT_HOLE / 2, self.lug_boss_bottom_z, carrier.TOP_Z + 1)
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
