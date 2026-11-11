"""
Goliath carrier for the Sherpa conversion of the Sovol SV08.

Rectangular plate that merges with the back wall at the rear (it is fused with the main body
in extruder_body.py). The Goliath (air cooled) is screwed on from the top with 4x M2.5 (heads
countersunk). Below the plate: two wedges for the 2515 hotend fan (on the -X side). The 5020
part cooling fan is no longer attached here.

Coordinates = Sovol STEP assembly (mm). The nozzle is at its original position.
"""
import math

from build123d import Align, Box, Cylinder, Plane, Polyline, Pos, Rot, export_step, extrude, import_step, make_face

import common as cm
import sherpa as sh

MIN = (Align.CENTER, Align.CENTER, Align.MIN)

# ---------------------------------------------------------------- parameters
AX_X, AX_Y = cm.AX_X, cm.AX_Y
Z_TOP = sh.Z_HOUSING_BOT              # contact face with the housing
T = sh.CARRIER_T
Z_BOT = Z_TOP - T

PTFE_D = 4.15
INLET_D, INLET_DEPTH = 4.6, 1.4       # centring on the Goliath inlet

GOLIATH_ROT = 180.0                   # Goliath threads below the carrier holes (45° pattern)
GOLIATH_TOP = 43.50
GOLIATH_TIP = -25.75
GOL_SCREW_R = 8.0
GOL_SCREWS_DEG = [45, 135, 225, 315]
SCREW_D, HEAD_D, HEAD_DEPTH = 2.9, 5.0, 2.6     # M2.5 DIN 912

# Rectangular plate, connected directly to the back wall (main body) at the rear
X_L, X_R = -367.548, -305.501         # full width of the main body
Y_FRONT = 214.2                       # up to the PCB (PCB holder at y=214)
# PCB raised so that the lower shoulder screw sits in the middle of the carrier
PCB_DZ = (Z_BOT + Z_TOP) / 2 - (-153.2)
PCB_SCREW, PCB_SCREW_R = (-336.5, -153.2 + PCB_DZ), 2.0
Y_WALL = 242.8747                     # reaches 0.6 mm into the main body (front face y=242.27)

# 2515 hotend fan: dimensions as on the original (+X) side; fan and wedges are then mirrored
# about the filament axis (x=AX_X) to the -X side
FAN_MIRROR = Plane.YZ.offset(AX_X)
FAN_FACE_X = -323.80                  # mirrored: fan face at x=-347.6
FAN_SHIFT_X = FAN_FACE_X - (-328.70)
FAN_TOP_ORIG = -165.075               # top of the 2515 in the SV08 model (without cables)
FAN_DZ = Z_BOT - FAN_TOP_ORIG         # fan directly below the carrier
FAN_HOLES = [(217.6, -167.57 + FAN_DZ), (237.6, -167.57 + FAN_DZ)]     # (y, z) upper fan holes
FAN_BACK_X = 2 * AX_X - (FAN_FACE_X + 15.0)   # back side of the mirrored 2515 (outside)
Y_MAIN_WALL = 242.2747                # front face of the main body
FAN_LUG_WALL = 1.8                    # wall around the hole in the wedge
FAN_LUG_HOLE = 3.4                    # M3 (fan screw, cuts its thread into the Ø2.8 fan hole)

# 5020 part cooling fan: position as before (2 mm forward), no longer attached to the carrier
FAN5020_SHIFT = (0.0, -2.0, 0.0)


# ---------------------------------------------------------------- carrier
def build():
    plate = Pos((X_L + X_R) / 2, (Y_FRONT + Y_WALL) / 2, Z_BOT) * Box(X_R - X_L, Y_WALL - Y_FRONT, T, align=MIN)

    # filament channel + centring
    plate -= Pos(AX_X, AX_Y, Z_BOT - 1) * Cylinder(PTFE_D / 2, T + 2, align=MIN)
    plate -= Pos(AX_X, AX_Y, Z_BOT) * Cylinder(INLET_D / 2, INLET_DEPTH, align=MIN)
    # Goliath screws (from the top, heads countersunk)
    for deg in GOL_SCREWS_DEG:
        x = AX_X + GOL_SCREW_R * math.cos(math.radians(deg))
        y = AX_Y + GOL_SCREW_R * math.sin(math.radians(deg))
        plate -= Pos(x, y, Z_BOT - 1) * Cylinder(SCREW_D / 2, T + 2, align=MIN)
        plate -= Pos(x, y, Z_TOP - HEAD_DEPTH) * Cylinder(HEAD_D / 2, 10, align=MIN)
    # lower PCB shoulder screw enters the plate from the front (heat-set insert)
    plate -= Pos(PCB_SCREW[0], Y_FRONT - 1, PCB_SCREW[1]) * Rot(-90, 0, 0) * Cylinder(PCB_SCREW_R, 6.0, align=MIN)
    plate += fan_lugs()
    for y, z in FAN_HOLES:
        plate -= Pos(X_L - 1, y, z) * Rot(0, 90, 0) * Cylinder(FAN_LUG_HOLE / 2, FAN_BACK_X - X_L + 1.1, align=MIN)
    return plate


def fan_lugs():
    """Two wedges below the carrier at the back side of the fan (outside), each around one upper fan hole.
    The inner edge is inclined by 45° towards the fan centre and is tangent to the wall around the hole,
    so as little of the intake area as possible is covered; on the outside the wedges reach the front
    edge or the main body."""
    out = None
    r = FAN_LUG_HOLE / 2 + FAN_LUG_WALL
    # outside up to the front edge of the carrier or the wall of the main body
    for (y, z), y_out in zip(FAN_HOLES, (Y_FRONT, Y_MAIN_WALL)):
        s = 1 if y_out < y else -1                     # direction towards the fan centre
        z_bot = z - r
        h = Z_BOT - z_bot
        y0 = y + s * r * (math.sqrt(2) - 1)            # 45° edge tangent to the wall around the hole
        pts = [(y_out, Z_BOT), (y_out, z_bot), (y0, z_bot), (y0 + s * h, Z_BOT)]
        face = make_face(Polyline(*[(X_L, py, pz) for py, pz in pts], close=True))
        lug = extrude(face, amount=FAN_BACK_X - 0.1 - X_L, dir=(1, 0, 0))   # 0.1 mm clearance to the fan
        out = lug if out is None else out + lug
    return out


def place_goliath():
    gol = import_step(str(cm.GOLIATH_STEP))
    dz = Z_BOT - GOLIATH_TOP
    return Pos(AX_X, AX_Y, dz) * Rot(0, 0, GOLIATH_ROT) * gol, GOLIATH_TIP + dz


carrier = build()
gol, nozzle_z = place_goliath()
gol_solid = gol.solids()[0]
carrier = carrier - gol_solid

if __name__ == "__main__":
    cm.OUT.mkdir(exist_ok=True)
    carrier.label = "goliath_carrier"
    export_step(carrier, str(cm.OUT / "goliath_carrier.step"))
    print(f"Nozzle tip z={nozzle_z:.2f} (original {cm.ORIG_NOZZLE_Z})")
    print(f"Carrier volume: {carrier.volume:.0f} mm^3, bbox {carrier.bounding_box().size}")
