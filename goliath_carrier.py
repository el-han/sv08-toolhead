"""Goliath carrier: plate below the Sherpa housing that holds the Goliath hotend.

Rectangular plate that merges with the back wall at the rear (it is fused with the main body
in main_body.py). The Goliath (air cooled) is screwed on from the top with 4x M2.5 (heads
countersunk). Below the plate: two wedges for the 2515 hotend fan (on the -X side). The 5020
part cooling fan is no longer attached here.

Coordinates = Sovol STEP assembly (mm). The nozzle is at its original position.
"""
from __future__ import annotations

import math
from functools import cached_property

from build123d import Plane, Polyline, Pos, Rot, Shape, extrude, import_step, make_face

import common
from common import FILAMENT_AXIS
from geometry import XY, XZ, Span, box_between, cylinder_along_x, cylinder_along_y, cylinder_along_z, fuse_all
from sherpa import SHERPA

FULL_WIDTH = Span(-367.548, -305.501)      # x extent of the main body
MAIN_BODY_FRONT_Y = 242.2747               # front face of the main body (back wall)


class GoliathCarrier:
    # plate
    THICKNESS = SHERPA.CARRIER_THICKNESS
    TOP_Z = SHERPA.housing_bottom_z               # contact face with the housing
    BOTTOM_Z = TOP_Z - THICKNESS
    FRONT_Y = 214.2                               # up to the PCB (PCB holder at y=214)
    BACK_Y = 242.8747                             # reaches 0.6 mm into the main body

    # filament inlet
    PTFE_DIAMETER = 4.15
    INLET_DIAMETER, INLET_DEPTH = 4.6, 1.4        # centring on the Goliath inlet

    # Goliath
    GOLIATH_ROTATION = 180.0                      # Goliath threads below the carrier holes (45° pattern)
    GOLIATH_TOP = 43.50                           # heat sink top in the Goliath STEP
    GOLIATH_TIP = -25.75                          # nozzle tip in the Goliath STEP
    SCREW_CIRCLE_RADIUS = 8.0
    SCREW_ANGLES = (45, 135, 225, 315)
    SCREW_HOLE_DIAMETER = 2.9                     # M2.5 DIN 912
    HEAD_DIAMETER, HEAD_DEPTH = 5.0, 2.6

    # PCB raised so that the lower shoulder screw sits in the middle of the carrier
    PCB_RAISE = (BOTTOM_Z + TOP_Z) / 2 - (-153.2)
    PCB_SCREW = XZ(-336.5, -153.2 + PCB_RAISE)
    PCB_SCREW_HOLE_RADIUS = 2.0

    # 2515 hotend fan: dimensions as on the original (+X) side; fan and wedges are then mirrored
    # about the filament axis to the -X side
    FAN_MIRROR = Plane.YZ.offset(FILAMENT_AXIS.x)
    FAN_FACE_X = -323.80                          # mirrored: fan face at x=-347.6
    FAN_SHIFT_X = FAN_FACE_X - (-328.70)
    FAN_TOP_ORIGINAL = -165.075                   # top of the 2515 in the SV08 model (without cables)
    FAN_RAISE = BOTTOM_Z - FAN_TOP_ORIGINAL       # fan directly below the carrier
    FAN_HOLES = ((217.6, -167.57 + FAN_RAISE), (237.6, -167.57 + FAN_RAISE))   # (y, z) upper fan holes
    FAN_BACK_X = 2 * FILAMENT_AXIS.x - (FAN_FACE_X + 15.0)   # back side of the mirrored 2515 (outside)
    FAN_WEDGE_WALL = 1.8                          # wall around the hole in the wedge
    FAN_SCREW_HOLE = 3.4                          # M3 (fan screw cuts its thread into the Ø2.8 fan hole)

    # 5020 part cooling fan: position as before (2 mm forward), no longer attached to the carrier
    FAN5020_SHIFT = (0.0, -2.0, 0.0)

    def goliath_screw_positions(self) -> list[XY]:
        return [XY(FILAMENT_AXIS.x + self.SCREW_CIRCLE_RADIUS * math.cos(math.radians(angle)),
                   FILAMENT_AXIS.y + self.SCREW_CIRCLE_RADIUS * math.sin(math.radians(angle)))
                for angle in self.SCREW_ANGLES]

    # ------------------------------------------------------------ Goliath
    @cached_property
    def _goliath_placement(self):
        dz = self.BOTTOM_Z - self.GOLIATH_TOP
        return Pos(FILAMENT_AXIS.x, FILAMENT_AXIS.y, dz) * Rot(0, 0, self.GOLIATH_ROTATION), dz

    @cached_property
    def goliath(self) -> Shape:
        """The Goliath hotend, hanging below the carrier."""
        placement, _ = self._goliath_placement
        return placement * import_step(str(common.GOLIATH_STEP))

    @property
    def nozzle_z(self) -> float:
        return self.GOLIATH_TIP + self._goliath_placement[1]

    # ------------------------------------------------------------ plate
    def fan_wedges(self) -> Shape:
        """Two wedges below the carrier at the back side of the fan (outside), each around one upper fan hole.
        The inner edge is inclined by 45° towards the fan centre and is tangent to the wall around the hole,
        so as little of the intake area as possible is covered; on the outside the wedges reach the front
        edge or the main body."""
        hole_wall_radius = self.FAN_SCREW_HOLE / 2 + self.FAN_WEDGE_WALL
        wedges = []
        for (hole_y, hole_z), outer_y in zip(self.FAN_HOLES, (self.FRONT_Y, MAIN_BODY_FRONT_Y)):
            towards_fan = 1 if outer_y < hole_y else -1
            wedge_bottom = hole_z - hole_wall_radius
            height = self.BOTTOM_Z - wedge_bottom
            edge_y = hole_y + towards_fan * hole_wall_radius * (math.sqrt(2) - 1)   # 45° edge tangent to the wall
            outline = [(outer_y, self.BOTTOM_Z), (outer_y, wedge_bottom), (edge_y, wedge_bottom),
                       (edge_y + towards_fan * height, self.BOTTOM_Z)]
            face = make_face(Polyline(*[(FULL_WIDTH.start, y, z) for y, z in outline], close=True))
            wedges.append(extrude(face, amount=self.FAN_BACK_X - 0.1 - FULL_WIDTH.start, dir=(1, 0, 0)))  # 0.1 mm to the fan
        return fuse_all(wedges)

    @cached_property
    def solid(self) -> Shape:
        """The carrier plate with all holes, fan wedges and the Goliath cut out."""
        plate = box_between(FULL_WIDTH, (self.FRONT_Y, self.BACK_Y), (self.BOTTOM_Z, self.TOP_Z))
        # filament channel + centring
        plate -= cylinder_along_z(FILAMENT_AXIS, self.PTFE_DIAMETER / 2, self.BOTTOM_Z - 1, self.TOP_Z + 1)
        plate -= cylinder_along_z(FILAMENT_AXIS, self.INLET_DIAMETER / 2, self.BOTTOM_Z,
                                  self.BOTTOM_Z + self.INLET_DEPTH)
        # Goliath screws (from the top, heads countersunk)
        for screw in self.goliath_screw_positions():
            plate -= cylinder_along_z(screw, self.SCREW_HOLE_DIAMETER / 2, self.BOTTOM_Z - 1, self.TOP_Z + 1)
            head_bottom = self.TOP_Z - self.HEAD_DEPTH
            plate -= cylinder_along_z(screw, self.HEAD_DIAMETER / 2, head_bottom, head_bottom + 10)
        # lower PCB shoulder screw enters the plate from the front (heat-set insert)
        plate -= cylinder_along_y(self.PCB_SCREW, self.PCB_SCREW_HOLE_RADIUS, self.FRONT_Y - 1, self.FRONT_Y + 5)
        plate += self.fan_wedges()
        for hole_y, hole_z in self.FAN_HOLES:
            start_x = FULL_WIDTH.start - 1
            plate -= cylinder_along_x((start_x, hole_y, hole_z), self.FAN_SCREW_HOLE / 2,
                                      self.FAN_BACK_X - FULL_WIDTH.start + 1.1)
        return plate - self.goliath.solids()[0]


GOLIATH_CARRIER = GoliathCarrier()

if __name__ == "__main__":
    carrier = GOLIATH_CARRIER
    print(f"Nozzle tip z={carrier.nozzle_z:.2f} (original {common.ORIGINAL_NOZZLE_Z})")
    print(f"Carrier volume: {carrier.solid.volume:.0f} mm^3")
