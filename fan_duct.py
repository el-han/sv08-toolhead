"""Part cooling duct for the 5020 fan, combined with the new front cover (replaces the SV08 air_duct_5020,
air_deflector_cover_plate3 and the original front cover).

Simplified rebuild of the original duct:
  plenum   hollow box below the fan outlet, open towards the fan
  arms     two hollow arms left and right of the nozzle, outlet slots in their inner walls facing the nozzle
  front    wall in front of the fan, full width, with 2 mm side walls back to the PCB holder and the air intake opening; the fan is screwed to it from the front with two
           self-tapping screws ST4.8 into its Ø4.4 ear bores

The front wall ends at the top of the PCB holder. Mounting: above the Goliath carrier the side walls reach back
past the (narrower) PCB holder; one M3x6 from each side through them into an insert in the PCB holder. On the
right a narrow strip below the sliding panel recess reaches back to the main body: one more M3x6 from the side.
Coordinates = Sovol STEP assembly (mm), with the 5020 at its position 2 mm forward of the original.
"""
from __future__ import annotations

from functools import cached_property

from build123d import Shape

from common import FILAMENT_AXIS
import math

from geometry import XZ, Span, box_between, cylinder_along_x, cylinder_along_y, fuse_all, largest_solid, prism
from goliath_carrier import FULL_WIDTH, GOLIATH_CARRIER
from housing import HOUSING
from main_body import MAIN_BODY



class FanDuct:
    # 5020 fan, measured at its shifted position (GOLIATH_CARRIER.FAN5020_SHIFT)
    FAN_FRONT_Y = 192.67
    FAN_TOP_Z = -156.99
    FAN_OUTLET_X = Span(-363.0, -318.65)                      # outlet opening at the bottom of the fan
    FAN_OUTLET_Y = Span(193.87, 211.56)
    FAN_AXIS = XZ(-336.52, -185.07)                           # impeller axis (along Y)
    FAN_INTAKE = 35.0                                          # intake opening in the fan front plate (Ø34.6..35.2)
    FAN_EARS = (XZ(-317.05, -205.0), XZ(-360.9, -166.9))      # Ø4.4 bores through the fan, along Y

    # duct
    WALL = 1.2
    FRONT_WALL = 2.0
    SIDE_WALL = HOUSING.COVER_SIDE_WALL                        # left and right side walls of the front cover
    BOTTOM_Z = -220.7                                          # as the original (nozzle tip at -224.07)
    PLENUM_TOP_Z = -208.7                                      # just below the fan (fan bottom -208.27/-208.66)
    PLENUM_BACK_Y = 214.2                                      # flush with the side walls (PCB holder / carrier front)
    ARM_TOP_Z = PLENUM_TOP_Z                                   # arms flush with the plenum top
    LEFT_ARM_INNER_X, LEFT_ARM_BACK_Y = -347.2, 226.0          # inner wall (towards the nozzle), back end
    RIGHT_ARM_INNER_X, RIGHT_ARM_BACK_Y = -326.0, 232.0
    DEFLECTOR_X = -349.0                                       # air deflector: right face of its vertical wall
    DEFLECTOR_DROP = 6.0                                       # 45° wall: horizontal (= vertical) extent
    DEFLECTOR_LENGTH = 7.0                                     # length of the sharp top edge
    OUTLET_HEIGHT = 4.0                                        # slot in the inner arm wall, from the bottom
    OUTLET_FROM_Y = 216.0                                      # slots start behind the plenum
    ARM_CORNER_CHAMFER = 0.5                                   # 45° at the back outer corner, fraction of the arm width

    # fan screws: self-tapping ST4.8 (DIN 7981), clearance hole in the front wall
    SCREW_HOLE = 5.0
    MOUNT_SCREW_HOLE = 3.4                                     # M3 clearance in the side wall extensions

    @property
    def front_top_z(self) -> float:
        """The front wall ends at the top of the PCB holder."""
        return HOUSING.pcb_plate.z.end

    @property
    def plenum_x(self) -> Span:
        """The plenum reaches the side walls (no gaps)."""
        return Span(FULL_WIDTH.start + self.SIDE_WALL - 0.01, FULL_WIDTH.end - self.SIDE_WALL + 0.01)

    @property
    def front_y(self) -> float:
        return self.FAN_FRONT_Y - self.FRONT_WALL

    def arms(self):
        """(x range, back y, inner wall x, direction towards the nozzle) of both arms; on the outside the arms
        reach the outer faces of the side walls."""
        for x, back_y in ((Span(FULL_WIDTH.start, self.LEFT_ARM_INNER_X), self.LEFT_ARM_BACK_Y),
                          (Span(self.RIGHT_ARM_INNER_X, FULL_WIDTH.end), self.RIGHT_ARM_BACK_Y)):
            towards_nozzle = 1 if x.end < FILAMENT_AXIS.x else -1
            inner_x = x.end if towards_nozzle > 0 else x.start
            yield x, back_y, inner_x, towards_nozzle

    def front_chamfer(self, inset: float = 0.0) -> Shape:
        """Region in front of the 45° plane from the front edge at plenum top down to the bottom
        (over the whole plenum height); `inset` moves the plane inwards (for the cavity)."""
        y0 = self.front_y + inset * math.sqrt(2)
        top, bottom, height = self.PLENUM_TOP_Z, self.BOTTOM_Z, self.PLENUM_TOP_Z - self.BOTTOM_Z
        x0 = FULL_WIDTH.start - 1
        outline = [(x0, y0, top), (x0, y0 + height, bottom), (x0, y0 + height, bottom - 1),
                   (x0, self.front_y - 1, bottom - 1), (x0, self.front_y - 1, top)]
        return prism(outline, (1, 0, 0), FULL_WIDTH.length + 2)

    def side_chamfers(self, inset: float = 0.0) -> list[Shape]:
        """45° chamfers along the lower outer edges (left and right), as large as the front chamfer; `inset`
        moves the planes inwards (for the cavity)."""
        top, bottom, height = self.PLENUM_TOP_Z, self.BOTTOM_Z, self.PLENUM_TOP_Z - self.BOTTOM_Z
        y0 = self.front_y - 1
        length = max(self.LEFT_ARM_BACK_Y, self.RIGHT_ARM_BACK_Y) + 1 - y0
        chamfers = []
        for edge, inwards in ((FULL_WIDTH.start, 1), (FULL_WIDTH.end, -1)):
            x0 = edge + inwards * inset * math.sqrt(2)
            outline = [(x0, y0, top), (x0 + inwards * height, y0, bottom), (x0 + inwards * height, y0, bottom - 1),
                       (edge - inwards, y0, bottom - 1), (edge - inwards, y0, top)]
            chamfers.append(prism(outline, (0, 1, 0), length))
        return chamfers

    def arm_corner_chamfers(self, inset: float = 0.0) -> list[Shape]:
        """Vertical 45° chamfers at the back outer corner of the arms, where the air turns towards the
        outlets; `inset` moves the chamfer plane inwards (for the cavity)."""
        shift = inset * math.sqrt(2)
        chamfers = []
        for x, back_y, _, towards_nozzle in self.arms():
            c = self.ARM_CORNER_CHAMFER * x.length
            outer_x = x.start if towards_nozzle > 0 else x.end
            y0 = back_y - shift
            z0 = self.BOTTOM_Z - 1
            outline = [(outer_x, y0 - c, z0), (outer_x + towards_nozzle * c, y0, z0),
                       (outer_x + towards_nozzle * c, back_y + 1, z0), (outer_x - towards_nozzle, back_y + 1, z0),
                       (outer_x - towards_nozzle, y0 - c, z0)]
            # only up to the arm top: above it the right side wall continues to the main body
            chamfers.append(prism(outline, (0, 0, 1), self.ARM_TOP_Z - z0))
        return chamfers

    def outer(self) -> Shape:
        front = box_between(FULL_WIDTH, (self.front_y, self.FAN_FRONT_Y), (self.BOTTOM_Z, self.front_top_z))
        # side walls at the left and right edge, back to the PCB holder / carrier front
        sides = [box_between(x, (self.front_y, HOUSING.PCB_PLATE_FRONT_Y), (self.BOTTOM_Z, self.front_top_z))
                 for x in ((FULL_WIDTH.start, FULL_WIDTH.start + self.SIDE_WALL),
                           (FULL_WIDTH.end - self.SIDE_WALL, FULL_WIDTH.end))]
        # above the Goliath carrier the side walls reach back past the PCB holder, just over the insert blocks
        sides += [box_between(x, (HOUSING.PCB_PLATE_FRONT_Y - 0.01, HOUSING.cover_block_back_y),
                              (GOLIATH_CARRIER.TOP_Z + HOUSING.COVER_CLEARANCE, self.front_top_z))
                  for x in ((FULL_WIDTH.start, FULL_WIDTH.start + self.SIDE_WALL),
                            (FULL_WIDTH.end - self.SIDE_WALL, FULL_WIDTH.end))]
        # right: straight bar from the arm top up to the sliding panel recess, back to the main body (side screw into its insert block)
        sides.append(box_between((FULL_WIDTH.end - self.SIDE_WALL, FULL_WIDTH.end),
                                 (HOUSING.PCB_PLATE_FRONT_Y - 0.01, MAIN_BODY.FRONT_Y - HOUSING.COVER_CLEARANCE),
                                 (self.ARM_TOP_Z - 0.01, MAIN_BODY.right_pocket_bottom_z)))
        plenum = box_between(self.plenum_x, (self.front_y, self.PLENUM_BACK_Y), (self.BOTTOM_Z, self.PLENUM_TOP_Z))
        arms = [box_between(x, (self.PLENUM_BACK_Y - self.WALL, back_y), (self.BOTTOM_Z, self.ARM_TOP_Z))
                for x, back_y, _, _ in self.arms()]
        body = fuse_all([front, *sides, plenum, *arms]) - self.front_chamfer()
        for chamfer in self.side_chamfers() + self.arm_corner_chamfers():
            body -= chamfer
        return body

    def cavity(self) -> Shape:
        w = self.WALL
        plenum = box_between((self.plenum_x.start + w, self.plenum_x.end - w), (self.FAN_FRONT_Y, self.PLENUM_BACK_Y - w),
                             (self.BOTTOM_Z + w, self.PLENUM_TOP_Z - w))
        arms = [box_between((x.start + w, x.end - w), (self.PLENUM_BACK_Y - w - 0.5, back_y - w),
                            (self.BOTTOM_Z + w, self.ARM_TOP_Z - w))
                for x, back_y, _, _ in self.arms()]
        outlet = box_between(self.FAN_OUTLET_X, self.FAN_OUTLET_Y, (self.PLENUM_TOP_Z - w - 0.1, self.PLENUM_TOP_Z + 0.1))
        cavity = fuse_all([plenum, *arms]) - self.front_chamfer(inset=w)
        for chamfer in self.side_chamfers(inset=w) + self.arm_corner_chamfers(inset=w):
            cavity -= chamfer
        return cavity + outlet

    def outlet_slots(self) -> list[Shape]:
        """Slots through the inner arm walls and the bottom edge below them, facing the nozzle."""
        slots = []
        for _, back_y, inner_x, towards_nozzle in self.arms():
            x = sorted((inner_x - towards_nozzle * (self.WALL + 0.5), inner_x + towards_nozzle * 0.5))
            slots.append(box_between(x, (self.OUTLET_FROM_Y, back_y - self.WALL),
                                     (self.BOTTOM_Z - 0.5, self.BOTTOM_Z + self.OUTLET_HEIGHT)))
        return slots

    def deflector(self) -> Shape:
        """Air deflector at the front of the plenum (as in the original, about 1/3 from the left), cross-section:
        a vertical wall from the plenum top to the floor and from its sharp top edge a 45° wall sloping down to
        the right. The top edge is DEFLECTOR_LENGTH long (from the inner face of the front chamfer); behind it
        the deflector drops at 45° to the floor."""
        w, x, drop = self.WALL, self.DEFLECTOR_X, self.DEFLECTOR_DROP
        top, floor = self.PLENUM_TOP_Z - w, self.BOTTOM_Z + w - 0.01
        slope = w * math.sqrt(2)
        ridge_start = self.front_y + slope + w            # inner face of the front chamfer at the plenum top
        ridge_end = ridge_start + self.DEFLECTOR_LENGTH
        height = top - floor
        y0, y1 = self.front_y, ridge_end + height + 1
        profile = [(x - w, y0, top), (x + drop, y0, top - drop - w), (x + drop, y0, top - drop - w - slope),
                   (x, y0, top - slope - w), (x, y0, floor), (x - w, y0, floor)]
        deflector = prism(profile, (0, 1, 0), y1 - y0)
        x0, x1 = x - w - 1, x + drop + 1
        back = prism([(x0, ridge_end - 1, top + 1), (x0, ridge_end + height + 0.1, floor - 0.1),
                      (x0, y1 + 1, floor - 0.1), (x0, y1 + 1, top + 1)], (1, 0, 0), x1 - x0)
        return deflector - back - self.front_chamfer()

    @cached_property
    def solid(self) -> Shape:
        duct = self.outer() - self.cavity() + self.deflector()
        for slot in self.outlet_slots():
            duct -= slot
        # air intake of the fan through the front wall
        duct -= cylinder_along_y(self.FAN_AXIS, self.FAN_INTAKE / 2, self.front_y - 1, self.FAN_FRONT_Y + 0.1)
        for ear in self.FAN_EARS:
            duct -= cylinder_along_y(ear, self.SCREW_HOLE / 2, self.front_y - 1, self.FAN_FRONT_Y + 1)
        for x in (FULL_WIDTH.start - 1, FULL_WIDTH.end - self.SIDE_WALL - 1):
            duct -= cylinder_along_x((x, HOUSING.cover_insert_y, HOUSING.COVER_SCREW_Z), self.MOUNT_SCREW_HOLE / 2,
                                     self.SIDE_WALL + 2)
        duct -= cylinder_along_x((FULL_WIDTH.end - self.SIDE_WALL - 1, MAIN_BODY.cover_insert_y, MAIN_BODY.COVER_SCREW_Z),
                                 self.MOUNT_SCREW_HOLE / 2, self.SIDE_WALL + 2)
        return largest_solid(duct)


FAN_DUCT = FanDuct()

if __name__ == "__main__":
    print(f"Fan duct: volume {FAN_DUCT.solid.volume:.0f} mm^3, faces {len(FAN_DUCT.solid.faces())}")
