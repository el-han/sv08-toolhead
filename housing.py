"""New Sherpa housing, made of two printed parts:

  PcbHolder   front: carries the front bearing, the idler pin, the filament guides and the PCBs
  MotorPlate  back: carries the motor, the rear bearing and the rear idler pin seat

Both share the dimensions of HousingLayout (axes, posts, clearances for the moving parts).

FDM concept: the main body is printed with its back side on the bed. The PCB holder is printed flat with
its inside (y=PCB_PLATE_BACK_Y) facing up; bearing seat and pin hole are vertical holes there.
The PCBs are screwed to the front of the PCB holder.

Hole dimensions as in the original Sherpa (measured in the STEP):
  MR85 front   Ø8.2  y 218.98..221.55, in front of it Ø7.0 up to 218.25
  MR85 rear    Ø8.2  y 243.35..245.85, behind it Ø6.6 up to 246.05
  idler pin    Ø3.3  front y 219.1..221.25 (PCB holder), rear y 235.35..238.75 (boss on the motor plate)
  thumbscrew insert M3x5x4  Ø4.8 along X, x -333.4..-328.8 (thumbscrew from +X through the idler arm)
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property

from build123d import Align, Box, Pos, Rot, Shape

from common import CLEARANCE, FILAMENT_AXIS
from geometry import (XY, XZ, Boss, Rect, Span, box_between, cylinder_along_x, cylinder_along_y,
                      cylinder_along_z, fuse_all, largest_solid, rotation_about_y)
from goliath_carrier import FULL_WIDTH, GOLIATH_CARRIER
from sherpa import SHERPA

HEAT_INSERT_HOLE = 4.8                  # heat-set insert M3x5x4 (as in the Sherpa): Ø4.8 hole
HEAT_INSERT_HOLE_DEPTH = 5.5
M3_CLEARANCE_RADIUS = 1.7


@dataclass(frozen=True)
class Post:
    """Square post of the PCB holder (axis along Y)."""
    x: float
    z: Span


class HousingLayout:
    """Dimensions shared by PCB holder, motor plate and main body."""

    # y planes
    PCB_PLATE_FRONT_Y = 214.2           # just behind the PCB (y=214)
    PCB_PLATE_BACK_Y = 221.55           # parting line PCB holder / frame (as in the original)
    PCB_PLATE_THICKNESS = 3.0
    FRAME_BACK_Y = 242.3                # frame reaches into the plate (front face 242.27)
    REAR_BEARING_FRONT_Y = 243.35       # front edge of the rear bearing seat (as in the original)
    LEFT_EDGE_X = -355.5                # clearances left of this do not touch the PCB holder

    # z range of the housing
    BOTTOM_Z = SHERPA.housing_bottom_z
    TOP_Z = -116.5

    # axes (x, z)
    DRIVE_SHAFT = XZ(-340.0, -136.32 + SHERPA.LIFT)
    IDLER_PIVOT = XZ(-326.9, -146.12 + SHERPA.LIFT)
    IDLER_SHAFT = XZ(-331.4, -136.32 + SHERPA.LIFT)
    IDLER_SWING_ANGLES = (-2.0, 0.0, 6.0, 12.0)       # degrees, away from the filament axis

    # filament channel
    FILAMENT_HOLE = 2.2
    PTFE_HOLE, PTFE_DEPTH = 4.15, 8.0

    # bosses on the PCB holder around bearing seat, idler pin and PCB insert
    BOSS_WALL = 1.5
    PCB_INSERT = XZ(-330.2, -128.7 + GOLIATH_CARRIER.PCB_RAISE)   # upper PCB shoulder screw (the lower one is in the carrier)
    PCB_PLATE_BOSSES = (Boss(DRIVE_SHAFT, 4.1 + BOSS_WALL), Boss(IDLER_PIVOT, 1.65 + BOSS_WALL),
                        Boss(PCB_INSERT, HEAT_INSERT_HOLE / 2 + BOSS_WALL))
    THUMBSCREW_KNOB_BOTTOM = -126.57 + SHERPA.LIFT - 6.0   # lower edge of the thumbscrew knob (Ø12)

    # posts between PCB holder and motor plate. Parting line in the posts: the M3x12 from the motor side
    # ends exactly at the end of the 4 mm heat-set insert (motor plate + strut = 8 mm)
    POST_SIZE = 7.0
    POST_PARTING_Y = SHERPA.motor_face_y - 12.0 + 4.0
    LEFT_POST_TOP = -126.2 + GOLIATH_CARRIER.PCB_RAISE    # previous top of the holder, kept

    # fan duct mounting: the side walls of the fan duct reach back past the PCB holder (which is narrower by
    # wall + clearance); one M3 from each side through the side wall into an insert in the PCB holder front wall
    COVER_SIDE_WALL = 2.0
    COVER_CLEARANCE = 0.2               # the screws pull the side walls against the holder
    COVER_SCREW_Z = -136.0
    COVER_INSERT_WALL = 1.5

    # mounting lugs (PCB holder -> main body)
    LUG_WIDTH, LUG_THICKNESS = 7.0, 3.0

    # motor plate
    MOTOR_PLATE_THICKNESS = 2.0
    MOTOR_EAR_DX = 21.92                       # x offset of the motor ear holes from the motor axis
    MOTOR_COLLAR_RADIUS = 8.0 + CLEARANCE      # pinion + centring collar
    MOTOR_SLOT_HALF = 1.5                      # half slot length (centre travel ±1.5 mm)
    REAR_PIN_BOSS = Boss(IDLER_PIVOT, 1.65 + BOSS_WALL)
    REAR_PIN_BOSS_FRONT_Y = 235.35

    @property
    def strut_back_y(self) -> float:
        """Front face of the motor plate."""
        return SHERPA.motor_face_y - self.MOTOR_PLATE_THICKNESS

    @cached_property
    def pcb_plate(self) -> Rect:
        """PCB holder plate: full width, bottom as the PCBs, top flush with the block around the idler insert."""
        insert_z = SHERPA.idler_insert.bounding_box().center().Z
        return Rect(self.pcb_plate_x, Span(-161.2 + GOLIATH_CARRIER.PCB_RAISE, insert_z + HEAT_INSERT_HOLE / 2 + 1.5))

    @property
    def pcb_plate_x(self) -> Span:
        """Full width minus the fan duct side walls and clearance."""
        inset = self.COVER_SIDE_WALL + self.COVER_CLEARANCE
        return Span(FULL_WIDTH.start + inset, FULL_WIDTH.end - inset)

    @property
    def cover_insert_y(self) -> float:
        """Axis of the fan duct inserts: insert wall in front of it, behind the PCB plate front."""
        return self.PCB_PLATE_FRONT_Y + HEAT_INSERT_HOLE / 2 + self.COVER_INSERT_WALL

    @property
    def cover_block_back_y(self) -> float:
        return self.cover_insert_y + HEAT_INSERT_HOLE / 2 + self.COVER_INSERT_WALL

    def cover_inserts(self) -> list[tuple[float, int]]:
        """(outer face x of the PCB holder, direction into the holder) left and right."""
        return [(self.pcb_plate_x.start, 1), (self.pcb_plate_x.end, -1)]

    def idler_swing(self, degrees: float):
        """Idler position; positive = clockwise seen from the front (idler towards +X)."""
        return rotation_about_y(self.IDLER_PIVOT, degrees)

    # ------------------------------------------------------------ posts, screws, lugs
    @cached_property
    def posts(self) -> tuple[Post, Post]:
        """Left post below the slot of the left motor screw, up to the top of the PCB holder; right post at the
        same distance from the edge, ending just below the thumbscrew knob."""
        left_x = SHERPA.motor_axis.x - self.MOTOR_EAR_DX
        right_x = FULL_WIDTH.end - (left_x - FULL_WIDTH.start)
        return (Post(left_x, Span(self.BOTTOM_Z, self.LEFT_POST_TOP)),
                Post(right_x, Span(self.BOTTOM_Z, self.THUMBSCREW_KNOB_BOTTOM - 0.4)))

    @cached_property
    def post_screws(self) -> list[XZ]:
        """Four M3 screws motor plate -> posts: bottom 1 mm above the post centre, top centred in the post end."""
        bottom_z = self.BOTTOM_Z + self.POST_SIZE / 2 + 1.0
        return ([XZ(post.x, bottom_z) for post in self.posts]
                + [XZ(post.x, post.z.end - self.POST_SIZE / 2) for post in self.posts])

    @cached_property
    def lug_holes(self) -> list[XY]:
        """Mounting lugs at the foot of the posts, pointing outwards to the side edge, midway between PCB plate
        and motor plate (semicircle up to the side edge)."""
        y = (self.PCB_PLATE_FRONT_Y + self.PCB_PLATE_THICKNESS + self.strut_back_y) / 2
        return [XY(FULL_WIDTH.start + self.LUG_WIDTH / 2, y), XY(FULL_WIDTH.end - self.LUG_WIDTH / 2, y)]

    def post_box(self, post: Post, y_from: float, y_to: float) -> Shape:
        half = self.POST_SIZE / 2
        return box_between((post.x - half, post.x + half), (y_from, y_to), post.z)

    # ------------------------------------------------------------ clearances
    def idler_sweep(self) -> list[Shape]:
        """Moving idler parts over the swing range. Clearance: -2° on the filament side, ±CLEARANCE in Y.
        (The copies are subtracted one by one; fusing all copies needs too much RAM.)"""
        # idler gear (toothing, 457 faces) and shaft as simple cylinders, the needle bearings are inside the gear
        moving = SHERPA.get("idler_arm_short") + [cylinder_along_y(self.IDLER_SHAFT, 4.75, 222.05, 236.05),
                                                  cylinder_along_y(self.IDLER_SHAFT, 1.5, 218.45, 238.45)]
        poses = ([self.idler_swing(degrees) for degrees in self.IDLER_SWING_ANGLES]
                 + [Pos(0, CLEARANCE, 0), Pos(0, -CLEARANCE, 0)])
        return [pose * part for pose in poses for part in moving]

    @cached_property
    def thumbscrew_insert_hole(self) -> Shape:
        box = SHERPA.idler_insert.bounding_box()
        center = box.center()
        return cylinder_along_x((box.min.X - 0.6, center.Y, center.Z), HEAT_INSERT_HOLE / 2,
                                box.max.X - box.min.X + 1.6)

    @cached_property
    def fixed_clearances(self) -> tuple[Shape, ...]:
        """Cut-outs for gears, bearings, idler pin, filament and thumbscrew (without the idler sweep)."""
        shaft, pin = self.DRIVE_SHAFT, self.IDLER_PIVOT
        cuts = [
            cylinder_along_y(shaft, 5.5, self.PCB_PLATE_BACK_Y - 0.1, 236.6),             # drive gear, shim
            cylinder_along_y(shaft, 13.6, 240.25 - CLEARANCE, self.REAR_BEARING_FRONT_Y),  # 50t gear rim (to y=243.25)
            cylinder_along_y(shaft, 6.3 + CLEARANCE, 235.6, 240.3),                       # 50t gear hub (max. Ø12.6)
            cylinder_along_y(SHERPA.motor_axis, 4.5, 240.05 - CLEARANCE, self.FRAME_BACK_Y + 1),  # pinion
            # bearing seats
            cylinder_along_y(shaft, 8.2 / 2, 218.98, self.PCB_PLATE_BACK_Y + 0.3),
            cylinder_along_y(shaft, 7.0 / 2, 218.25, 219.0),
            cylinder_along_y(shaft, 8.2 / 2, 243.0, 245.85),
            cylinder_along_y(shaft, 6.6 / 2, 245.8, 246.05),
            # idler pin: front seat, rear seat, between the two seats
            cylinder_along_y(pin, 3.3 / 2, 218.7, self.PCB_PLATE_BACK_Y + 0.1),
            cylinder_along_y(pin, 3.3 / 2, 235.35, 238.9),
            cylinder_along_y(pin, 3.3 / 2, 221.0, 235.4),
            # filament: guide down to the carrier, PTFE bore from the top
            cylinder_along_z(FILAMENT_AXIS, self.FILAMENT_HOLE / 2, self.BOTTOM_Z - 1, self.TOP_Z + 1),
            cylinder_along_z(FILAMENT_AXIS, self.PTFE_HOLE / 2, self.TOP_Z - self.PTFE_DEPTH, self.TOP_Z + 1),
            # thumbscrew insert (along X, inserted from +X) + screw channel
            cylinder_along_x((-333.4, 228.8, -126.57 + SHERPA.LIFT), HEAT_INSERT_HOLE / 2, 7.4),
            cylinder_along_x((-333.4, 228.8, -126.57 + SHERPA.LIFT), 3.4 / 2, 12.0),
        ]
        return tuple(cuts)

    @cached_property
    def clearances(self) -> tuple[Shape, ...]:
        """Everything cut out of the housing parts and the main body."""
        return self.fixed_clearances + tuple(self.idler_sweep())


class FilamentGuides:
    """Filament guides above and below the drive gears, dimensions as in the original Sherpa housing
    (z offsets measured there relative to the drive shaft). Part of the PCB holder."""

    GUIDE_HOLE, BORE = 2.1, 4.2                  # filament guide / PTFE bore
    WALL = 1.5
    GEAR_CUT_RADIUS = 4.75 + 0.75                # drive/idler gear (r4.75) with clearance
    # upper guide: bore down to 5.32, guide down to 2.42 above the shaft, top 18 above it
    UPPER_TOP, UPPER_BORE_BOTTOM, UPPER_BOTTOM = 18.0, 5.32, 2.42
    # lower guide: top 2.38 below the shaft, guide down to -5.28, below it the bore down to the carrier
    LOWER_TOP, LOWER_GUIDE_BOTTOM = -2.38, -5.28

    def __init__(self, layout: HousingLayout):
        self.layout = layout
        self.shaft_z = layout.DRIVE_SHAFT.z

    @property
    def top_z(self) -> float:
        return self.shaft_z + self.UPPER_TOP

    def channel_cuts(self) -> list[Shape]:
        """Guide hole and PTFE bore of the upper guide."""
        z = self.shaft_z
        return [cylinder_along_z(FILAMENT_AXIS, self.GUIDE_HOLE / 2, z + self.UPPER_BOTTOM - 1, z + self.UPPER_TOP + 1),
                cylinder_along_z(FILAMENT_AXIS, self.BORE / 2, z + self.UPPER_BORE_BOTTOM, z + self.UPPER_TOP + 1)]

    @cached_property
    def solids(self) -> tuple[Shape, Shape]:
        """(upper, lower) guide."""
        layout, z = self.layout, self.shaft_z
        outer = self.BORE / 2 + self.WALL
        upper = cylinder_along_z(FILAMENT_AXIS, outer, z + self.UPPER_BOTTOM, z + self.UPPER_TOP)
        lower = cylinder_along_z(FILAMENT_AXIS, outer, layout.BOTTOM_Z, z + self.LOWER_TOP)
        # webs towards the PCB holder, as wide as the tube (within the height of the holder)
        web_x = (FILAMENT_AXIS.x - outer, FILAMENT_AXIS.x + outer)
        web_y = (layout.PCB_PLATE_FRONT_Y + layout.PCB_PLATE_THICKNESS - 0.01, FILAMENT_AXIS.y)
        upper += box_between(web_x, web_y, (z + self.UPPER_BOTTOM, min(z + self.UPPER_TOP, layout.pcb_plate.z.end)))
        lower += box_between(web_x, web_y, (layout.BOTTOM_Z, z + self.LOWER_TOP))
        # bores after the webs, so they stay free
        for cut in self.channel_cuts():
            upper -= cut
        lower -= cylinder_along_z(FILAMENT_AXIS, self.GUIDE_HOLE / 2, layout.BOTTOM_Z - 1, z + self.LOWER_TOP + 1)
        lower -= cylinder_along_z(FILAMENT_AXIS, self.BORE / 2, layout.BOTTOM_Z - 1, z + self.LOWER_GUIDE_BOTTOM)
        # recesses for drive gear and idler gear: two equal circles symmetric to the filament axis
        for x in (layout.DRIVE_SHAFT.x, 2 * FILAMENT_AXIS.x - layout.DRIVE_SHAFT.x):
            gear = cylinder_along_y(XZ(x, z), self.GEAR_CUT_RADIUS, FILAMENT_AXIS.y - 20, FILAMENT_AXIS.y + 20)
            upper -= gear
            lower -= gear
        return upper, lower


class PcbHolder:
    """Front housing part: 3 mm plate with bosses, posts, lugs, filament guides and the thumbscrew insert holder."""

    ARM_EDGE = XZ(-330.0, -130.975)      # point on the top edge of the idler arm in front of the PCB insert
    ARM_SLOPE = 17.8                     # inclination of this edge towards +X (degrees)
    ARM_CLEARANCE = 0.5                  # clearance between PCB holder and idler arm
    INSERT_HOLDER_WALL = 1.5             # wall around the thumbscrew insert
    # cable holder: bar standing up on the top at the left edge, waist in the middle for a cable tie
    CABLE_BAR_WIDTH, CABLE_BAR_DEPTH, CABLE_BAR_HEIGHT = 10.0, 4.0, 14.0
    CABLE_BAR_WAIST, CABLE_BAR_WAIST_HEIGHT = 6.0, 4.0     # width / height of the waist

    def __init__(self, layout: HousingLayout):
        self.layout = layout
        self.guides = FilamentGuides(layout)

    def pcb_insert_hole(self) -> Shape:
        y0 = self.layout.PCB_PLATE_FRONT_Y
        return cylinder_along_y(self.layout.PCB_INSERT, HEAT_INSERT_HOLE / 2, y0 - 0.1, y0 + HEAT_INSERT_HOLE_DEPTH)

    def lugs(self) -> Shape:
        """Mounting lugs to the main body (M3 from the top, heat-set insert in the Goliath carrier)."""
        layout = self.layout
        half, z0, z1 = layout.LUG_WIDTH / 2, layout.BOTTOM_Z, layout.BOTTOM_Z + layout.LUG_THICKNESS
        lugs = []
        for post, hole in zip(layout.posts, layout.lug_holes):
            lug = box_between((min(post.x, hole.x), max(post.x, hole.x)), (hole.y - half, hole.y + half), (z0, z1))
            lug += cylinder_along_z(hole, half, z0, z1)
            lug -= cylinder_along_z(hole, M3_CLEARANCE_RADIUS, z0 - 1, z1 + 1)
            lugs.append(lug)
        return fuse_all(lugs)

    def plate(self) -> Shape:
        """Plate of PCB size with bosses, posts, lugs and heat-set insert holes."""
        layout = self.layout
        rect = layout.pcb_plate
        front_y, back_y = layout.PCB_PLATE_FRONT_Y, layout.PCB_PLATE_BACK_Y
        plate = Pos(rect.x.middle, front_y, rect.z.middle) * Box(
            rect.x.length, layout.PCB_PLATE_THICKNESS, rect.z.length, align=(Align.CENTER, Align.MIN, Align.CENTER))
        for boss in layout.PCB_PLATE_BOSSES:
            plate += cylinder_along_y(boss.center, boss.radius, front_y, back_y)
        # the holder ends at the top of the Goliath carrier (below it the carrier holds the PCB)
        plate -= Pos(0, 0, layout.BOTTOM_Z) * Box(2000, 2000, 100, align=(Align.CENTER, Align.CENTER, Align.MAX))
        # clearances without the idler contour; at the idler only simplified boxes are cut (arm_clearance_cuts)
        for cut in layout.fixed_clearances:
            box = cut.bounding_box()
            if box.min.Y < back_y and box.max.X > layout.LEFT_EDGE_X and box.max.Y > front_y:
                plate -= cut
        # posts up to the parting line (the rest belongs to the motor plate)
        for post in layout.posts:
            plate += layout.post_box(post, front_y + layout.PCB_PLATE_THICKNESS - 0.01, layout.POST_PARTING_Y)
        plate += self.lugs()
        # heat-set inserts in the posts (screwed from the motor side)
        parting_y = layout.POST_PARTING_Y
        for screw in layout.post_screws:
            plate -= cylinder_along_y(screw, HEAT_INSERT_HOLE / 2, parting_y - HEAT_INSERT_HOLE_DEPTH, parting_y + 0.1)
        plate -= self.pcb_insert_hole()
        return largest_solid(plate)

    def cover_insert_blocks(self) -> Shape:
        """Thickened front wall at the outer edges around the fan duct inserts, over the full height of the plate."""
        layout = self.layout
        length = HEAT_INSERT_HOLE_DEPTH + layout.COVER_INSERT_WALL
        return fuse_all([box_between(sorted((x, x + inwards * length)),
                                     (layout.PCB_PLATE_FRONT_Y, layout.cover_block_back_y),
                                     (layout.BOTTOM_Z, layout.pcb_plate.z.end))
                         for x, inwards in layout.cover_inserts()])

    def cover_insert_holes(self) -> list[Shape]:
        layout = self.layout
        return [cylinder_along_x((x - inwards * 0.1 if inwards > 0 else x - HEAT_INSERT_HOLE_DEPTH,
                                  layout.cover_insert_y, layout.COVER_SCREW_Z),
                                 HEAT_INSERT_HOLE / 2, HEAT_INSERT_HOLE_DEPTH + 0.1)
                for x, inwards in layout.cover_inserts()]

    def insert_holder(self) -> Shape:
        """Cylinder (axis along X) around the thumbscrew insert, position as in the original Sherpa.
        Joins the upper filament guide; the insert is inserted from +X."""
        box = SHERPA.idler_insert.bounding_box()
        center = box.center()
        x0 = box.min.X - 0.6 - self.INSERT_HOLDER_WALL
        holder = cylinder_along_x((x0, center.Y, center.Z), HEAT_INSERT_HOLE / 2 + self.INSERT_HOLDER_WALL,
                                  box.max.X - x0)
        return holder - self.layout.thumbscrew_insert_hole

    def insert_reinforcement(self) -> Shape:
        """Block around the thumbscrew insert: from the outer edge of the left post to the end of the insert, from the back of the
        PCB plate to behind the insert holder, at the height of the insert holder."""
        layout = self.layout
        box = SHERPA.idler_insert.bounding_box()
        center = box.center()
        radius = HEAT_INSERT_HOLE / 2 + self.INSERT_HOLDER_WALL
        left_x = layout.posts[0].x - layout.POST_SIZE / 2          # up to the outer edge of the left post
        return box_between((left_x, box.max.X),
                           (layout.PCB_PLATE_FRONT_Y + layout.PCB_PLATE_THICKNESS - 0.01, center.Y + radius),
                           (center.Z - radius, center.Z + radius))

    @property
    def cable_bar_x(self) -> Span:
        start = self.layout.pcb_plate_x.start
        return Span(start, start + self.CABLE_BAR_WIDTH)

    def cable_bar(self) -> Shape:
        """Cable holder: bar on the top at the left edge, from the front face back, with a waist in the middle."""
        layout = self.layout
        x, top = self.cable_bar_x, layout.pcb_plate.z.end
        y = (layout.PCB_PLATE_FRONT_Y, layout.PCB_PLATE_FRONT_Y + self.CABLE_BAR_DEPTH)
        bar = box_between(x, y, (top - 0.01, top + self.CABLE_BAR_HEIGHT))
        notch = (self.CABLE_BAR_WIDTH - self.CABLE_BAR_WAIST) / 2
        middle = top + self.CABLE_BAR_HEIGHT / 2
        waist_z = (middle - self.CABLE_BAR_WAIST_HEIGHT / 2, middle + self.CABLE_BAR_WAIST_HEIGHT / 2)
        for side in ((x.start - 1, x.start + notch), (x.end - notch, x.end + 1)):
            bar -= box_between(side, (y[0] - 1, y[1] + 1), waist_z)
        return bar

    def arm_clearance_cuts(self, part: Shape) -> list[Shape]:
        """Simplified clearance cut to the idler arm: per swing position the box around the part of the arm
        that comes close to `part`, enlarged by ARM_CLEARANCE and limited to the outline of `part`."""
        def grown(box, by):
            return box_between((box.min.X - by, box.max.X + by), (box.min.Y - by, box.max.Y + by),
                               (box.min.Z - by, box.max.Z + by))

        part_box = part.bounding_box()
        zone = grown(part_box, self.ARM_CLEARANCE)
        outline = grown(part_box, 0)
        arm = SHERPA.get("idler_arm_short")[0]
        cuts = []
        for degrees in (0.0, 6.0, 12.0):
            near = (self.layout.idler_swing(degrees) * arm) & zone
            if near and near.solids():
                cuts.append(grown(near.bounding_box(), self.ARM_CLEARANCE) & outline)
        return cuts

    def pcb_insert_boss_arm_cut(self) -> Shape:
        """Everything of the PCB insert boss below an inclined line ARM_CLEARANCE above the top edge of the idler
        arm (rising with ARM_SLOPE towards +X), from the boss down to the PCB plate."""
        layout = self.layout
        boss = layout.PCB_PLATE_BOSSES[2]
        above = (Pos(self.ARM_EDGE.x, (layout.PCB_PLATE_FRONT_Y + layout.PCB_PLATE_BACK_Y) / 2,
                     self.ARM_EDGE.z + self.ARM_CLEARANCE)
                 * Rot(0, -self.ARM_SLOPE, 0) * Box(40, 40, 20, align=(Align.CENTER, Align.CENTER, Align.MIN)))
        x, z, r = boss.center.x, boss.center.z, boss.radius
        boss_box = box_between((x - r - 0.1, x + r + 0.1),
                               (layout.PCB_PLATE_FRONT_Y + layout.PCB_PLATE_THICKNESS, layout.PCB_PLATE_BACK_Y + 0.1),
                               (z - r - 0.1, z + r))
        return boss_box - above

    @cached_property
    def solid(self) -> Shape:
        upper_guide, lower_guide = self.guides.solids
        holder = self.plate() + upper_guide + lower_guide + self.insert_holder() + self.insert_reinforcement()
        holder += self.cover_insert_blocks() + self.cable_bar()
        holder -= self.pcb_insert_boss_arm_cut()
        for cut in self.arm_clearance_cuts(self.insert_holder()):
            holder -= cut
        # re-cut the holes the additions may have closed
        holder -= self.layout.thumbscrew_insert_hole
        holder -= self.pcb_insert_hole()
        for hole in self.cover_insert_holes():
            holder -= hole
        for cut in self.guides.channel_cuts():
            holder -= cut
        # Ø7 as extension of the shaft axis to the front (as in the original)
        holder -= cylinder_along_y(self.layout.DRIVE_SHAFT, 7.0 / 2, self.layout.PCB_PLATE_FRONT_Y - 1, 219.0)
        return largest_solid(holder)


class MotorPlate:
    """Rear housing part: 2 mm plate in front of the motor mounting face, standing on the main body, with the
    struts to the posts, the rear bearing seat and the rear idler pin seat."""

    def __init__(self, layout: HousingLayout):
        self.layout = layout

    def plate(self) -> Shape:
        """Plate with the passage for pinion/centring collar and two M3 holes in the motor ears, the left one
        (seen from the front) as a slot."""
        layout = self.layout
        motor = SHERPA.motor_axis
        y1 = SHERPA.motor_face_y
        y0 = y1 - layout.MOTOR_PLATE_THICKNESS
        top_z = SHERPA.get("motor")[0].bounding_box().max.Z
        plate = box_between(FULL_WIDTH, (y0, y1), (layout.BOTTOM_Z, top_z))

        def slot(center: XZ, radius: float, half_length: float) -> tuple[Shape, Shape]:
            """Vertical slot through the plate: (both round ends, straight middle part)."""
            ends = (cylinder_along_y(center.shifted(dz=-half_length), radius, y0 - 1, y1 + 1)
                    + cylinder_along_y(center.shifted(dz=half_length), radius, y0 - 1, y1 + 1))
            return ends, Pos(center.x, (y0 + y1) / 2, center.z) * Box(2 * radius, layout.MOTOR_PLATE_THICKNESS + 2,
                                                                       2 * half_length)

        # pinion/centring collar: swivelling about the right screw moves the axis by
        # MOTOR_SLOT_HALF * (distance axis-pivot) / (screw distance) = MOTOR_SLOT_HALF / 2 vertically
        for cut in slot(motor, layout.MOTOR_COLLAR_RADIUS, layout.MOTOR_SLOT_HALF / 2):
            plate -= cut
        plate -= cylinder_along_y(motor.shifted(dx=layout.MOTOR_EAR_DX), M3_CLEARANCE_RADIUS, y0 - 1, y1 + 1)
        for cut in slot(motor.shifted(dx=-layout.MOTOR_EAR_DX), M3_CLEARANCE_RADIUS, layout.MOTOR_SLOT_HALF):
            plate -= cut
        return plate

    def struts(self) -> Shape:
        """Post sections from the parting line to the motor plate, as high as the posts of the PCB holder."""
        layout = self.layout
        return fuse_all(layout.post_box(post, layout.POST_PARTING_Y, layout.strut_back_y + 0.01)
                        for post in layout.posts)

    def rear_pin_boss(self) -> Shape:
        """Rear idler pin seat: cylinder at the front of the motor plate."""
        boss = self.layout.REAR_PIN_BOSS
        return cylinder_along_y(boss.center, boss.radius, self.layout.REAR_PIN_BOSS_FRONT_Y,
                                self.layout.strut_back_y + 0.01)

    @cached_property
    def solid(self) -> Shape:
        layout = self.layout
        plate = self.plate() + self.rear_pin_boss() + self.struts()
        # ring in front of the plate: bearing seat 2.5 mm deep as in the original (MR85 fully sunk)
        plate += cylinder_along_y(layout.DRIVE_SHAFT, 8.2 / 2 + layout.BOSS_WALL, layout.REAR_BEARING_FRONT_Y,
                                  layout.strut_back_y + 0.01)
        for screw in layout.post_screws:                                # M3 passage to the heat-set inserts
            plate -= cylinder_along_y(screw, M3_CLEARANCE_RADIUS, layout.POST_PARTING_Y - 0.1,
                                      SHERPA.motor_face_y + 0.1)
        for cut in layout.clearances:
            box = cut.bounding_box()
            if box.max.Y > layout.REAR_PIN_BOSS_FRONT_Y and box.min.Y < SHERPA.motor_face_y:
                plate -= cut
        return largest_solid(plate)


HOUSING = HousingLayout()
PCB_HOLDER = PcbHolder(HOUSING)
MOTOR_PLATE = MotorPlate(HOUSING)

if __name__ == "__main__":
    for name, part in (("PCB holder", PCB_HOLDER), ("motor plate", MOTOR_PLATE)):
        print(f"{name}: volume {part.solid.volume:.0f} mm^3, faces {len(part.solid.faces())}")
