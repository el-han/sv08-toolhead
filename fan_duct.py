"""Part cooling duct for the 5020 fan, combined with the new front cover (replaces the SV08 air_duct_5020,
air_deflector_cover_plate3 and the original front cover).

Simplified rebuild of the original duct:
  plenum   hollow box below the fan outlet, open towards the fan
  arms     two hollow arms left and right of the nozzle, outlet slots in their inner walls facing the nozzle
  front    wall in front of the fan, full width, with the air intake opening; the fan is screwed to it from the front with two
           self-tapping screws ST4.8 into its Ø4.4 ear bores

The front wall ends at the top of the PCB holder. The mounting to the rest of the toolhead is still open.
Coordinates = Sovol STEP assembly (mm), with the 5020 at its position 2 mm forward of the original.
"""
from __future__ import annotations

from functools import cached_property

from build123d import Shape

from common import FILAMENT_AXIS
from geometry import XZ, Span, box_between, cylinder_along_y, fuse_all, largest_solid
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
    BOTTOM_Z = -220.7                                          # as the original (nozzle tip at -224.07)
    PLENUM_TOP_Z = -208.7                                      # just below the fan (fan bottom -208.27/-208.66)
    PLENUM_X = Span(-364.4, -308.7)
    PLENUM_BACK_Y = 213.7
    ARM_TOP_Z = -211.4
    LEFT_ARM = (Span(-357.4, -347.2), 226.0)                   # x range, back end y
    RIGHT_ARM = (Span(-326.0, -315.7), 232.0)
    OUTLET_HEIGHT = 4.0                                        # slot in the inner arm wall, from the bottom
    OUTLET_FROM_Y = 216.0                                      # slots start behind the plenum

    # fan screws: self-tapping ST4.8 (DIN 7981), clearance hole in the front wall
    SCREW_HOLE = 5.0

    @property
    def front_top_z(self) -> float:
        """The front wall ends at the top of the PCB holder."""
        return HOUSING.pcb_plate.z.end

    @property
    def front_y(self) -> float:
        return self.FAN_FRONT_Y - self.FRONT_WALL

    def arms(self):
        """(x range, back y, inner wall x, direction towards the nozzle) of both arms."""
        for x, back_y in (self.LEFT_ARM, self.RIGHT_ARM):
            towards_nozzle = 1 if x.end < FILAMENT_AXIS.x else -1
            inner_x = x.end if towards_nozzle > 0 else x.start
            yield x, back_y, inner_x, towards_nozzle

    def outer(self) -> Shape:
        front = box_between(FULL_WIDTH, (self.front_y, self.FAN_FRONT_Y), (self.BOTTOM_Z, self.front_top_z))
        plenum = box_between(self.PLENUM_X, (self.front_y, self.PLENUM_BACK_Y), (self.BOTTOM_Z, self.PLENUM_TOP_Z))
        arms = [box_between(x, (self.PLENUM_BACK_Y - self.WALL, back_y), (self.BOTTOM_Z, self.ARM_TOP_Z))
                for x, back_y, _, _ in self.arms()]
        return fuse_all([front, plenum, *arms])

    def cavity(self) -> Shape:
        w = self.WALL
        plenum = box_between((self.PLENUM_X.start + w, self.PLENUM_X.end - w), (self.FAN_FRONT_Y, self.PLENUM_BACK_Y - w),
                             (self.BOTTOM_Z + w, self.PLENUM_TOP_Z - w))
        arms = [box_between((x.start + w, x.end - w), (self.PLENUM_BACK_Y - w - 0.5, back_y - w),
                            (self.BOTTOM_Z + w, self.ARM_TOP_Z - w))
                for x, back_y, _, _ in self.arms()]
        outlet = box_between(self.FAN_OUTLET_X, self.FAN_OUTLET_Y, (self.PLENUM_TOP_Z - w - 0.1, self.PLENUM_TOP_Z + 0.1))
        return fuse_all([plenum, *arms, outlet])

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
