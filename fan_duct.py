"""Part cooling duct for the 5020 fan, combined with the new front cover (replaces the SV08 air_duct_5020,
air_deflector_cover_plate3 and the original front cover).

Simplified rebuild of the original duct:
  plenum   hollow box below the fan outlet, open towards the fan
  arms     two hollow arms left and right of the nozzle, outlet slots in their inner walls facing the nozzle
  front    wall in front of the fan, full width, with 2 mm side walls back to the PCB holder and the air intake opening; the fan is screwed to it from the front with two
           self-tapping screws ST4.8 into its Ø4.4 ear bores

The front wall ends at the top of the PCB holder. The mounting to the rest of the toolhead is still open.
Coordinates = Sovol STEP assembly (mm), with the 5020 at its position 2 mm forward of the original.
"""
from __future__ import annotations

from functools import cached_property

from build123d import Shape

from common import FILAMENT_AXIS
import math

from geometry import XZ, Span, box_between, cylinder_along_y, fuse_all, largest_solid, prism
from goliath_carrier import FULL_WIDTH
from housing import HOUSING



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
    SIDE_WALL = 2.0                                            # left and right side walls of the front cover
    BOTTOM_Z = -220.7                                          # as the original (nozzle tip at -224.07)
    PLENUM_TOP_Z = -208.7                                      # just below the fan (fan bottom -208.27/-208.66)
    PLENUM_BACK_Y = 214.2                                      # flush with the side walls (PCB holder / carrier front)
    ARM_TOP_Z = -211.4
    LEFT_ARM = (Span(-357.4, -347.2), 226.0)                   # x range, back end y
    RIGHT_ARM = (Span(-326.0, -315.7), 232.0)
    OUTLET_HEIGHT = 4.0                                        # slot in the inner arm wall, from the bottom
    OUTLET_FROM_Y = 216.0                                      # slots start behind the plenum
    ARM_CORNER_CHAMFER = 5.0                                   # 45° at the back outer corner of each arm

    # fan screws: self-tapping ST4.8 (DIN 7981), clearance hole in the front wall
    SCREW_HOLE = 5.0

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
        """(x range, back y, inner wall x, direction towards the nozzle) of both arms."""
        for x, back_y in (self.LEFT_ARM, self.RIGHT_ARM):
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

    def arm_corner_chamfers(self, inset: float = 0.0) -> list[Shape]:
        """Vertical 45° chamfers at the back outer corner of the arms, where the air turns towards the
        outlets; `inset` moves the chamfer plane inwards (for the cavity)."""
        c, shift = self.ARM_CORNER_CHAMFER, inset * math.sqrt(2)
        chamfers = []
        for x, back_y, _, towards_nozzle in self.arms():
            outer_x = x.start if towards_nozzle > 0 else x.end
            y0 = back_y - shift
            z0 = self.BOTTOM_Z - 1
            outline = [(outer_x, y0 - c, z0), (outer_x + towards_nozzle * c, y0, z0),
                       (outer_x + towards_nozzle * c, back_y + 1, z0), (outer_x - towards_nozzle, back_y + 1, z0),
                       (outer_x - towards_nozzle, y0 - c, z0)]
            chamfers.append(prism(outline, (0, 0, 1), self.ARM_TOP_Z - self.BOTTOM_Z + 2))
        return chamfers

    def outer(self) -> Shape:
        front = box_between(FULL_WIDTH, (self.front_y, self.FAN_FRONT_Y), (self.BOTTOM_Z, self.front_top_z))
        # side walls at the left and right edge, back to the PCB holder / carrier front
        sides = [box_between(x, (self.front_y, HOUSING.PCB_PLATE_FRONT_Y), (self.BOTTOM_Z, self.front_top_z))
                 for x in ((FULL_WIDTH.start, FULL_WIDTH.start + self.SIDE_WALL),
                           (FULL_WIDTH.end - self.SIDE_WALL, FULL_WIDTH.end))]
        plenum = box_between(self.plenum_x, (self.front_y, self.PLENUM_BACK_Y), (self.BOTTOM_Z, self.PLENUM_TOP_Z))
        arms = [box_between(x, (self.PLENUM_BACK_Y - self.WALL, back_y), (self.BOTTOM_Z, self.ARM_TOP_Z))
                for x, back_y, _, _ in self.arms()]
        body = fuse_all([front, *sides, plenum, *arms]) - self.front_chamfer()
        for chamfer in self.arm_corner_chamfers():
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
        for chamfer in self.arm_corner_chamfers(inset=w):
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

    @cached_property
    def solid(self) -> Shape:
        duct = self.outer() - self.cavity()
        for slot in self.outlet_slots():
            duct -= slot
        # air intake of the fan through the front wall
        duct -= cylinder_along_y(self.FAN_AXIS, self.FAN_INTAKE / 2, self.front_y - 1, self.FAN_FRONT_Y + 0.1)
        for ear in self.FAN_EARS:
            duct -= cylinder_along_y(ear, self.SCREW_HOLE / 2, self.front_y - 1, self.FAN_FRONT_Y + 1)
        return largest_solid(duct)


FAN_DUCT = FanDuct()

if __name__ == "__main__":
    print(f"Fan duct: volume {FAN_DUCT.solid.volume:.0f} mm^3, faces {len(FAN_DUCT.solid.faces())}")
