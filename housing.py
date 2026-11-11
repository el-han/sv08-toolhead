"""
New Sherpa housing: PCB holder (front, carries the front bearing and the idler pin) and motor plate
(back, carries the motor, the rear bearing and the rear idler pin seat).

FDM concept: the main body is printed with its back side (y=Y_BACK) on the bed. The PCB holder is printed
flat with its inside (y=Y_LID_BACK) facing up; bearing seat and pin hole are vertical holes there.
The PCBs are screwed to the front of the PCB holder.

Hole dimensions as in the original Sherpa (measured in the STEP):
  MR85 front   Ø8.2  y 218.98..221.55, in front of it Ø7.0 up to 218.25
  MR85 rear    Ø8.2  y 243.35..245.85, behind it Ø6.6 up to 246.05
  idler pin    Ø3.3  front y 219.1..221.25 (PCB holder), rear y 235.35..238.75 (boss on the motor plate)
  thumbscrew insert M3x5x4  Ø4.8 along X, x -333.4..-328.8 (thumbscrew from +X through the idler arm)
"""
import math
from functools import lru_cache

from build123d import Align, Box, Cylinder, Pos, Rot, export_step

import common as cm
import goliath_carrier as gc
import sherpa as sh

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
CLR = 0.4

Y_LID_FRONT = 214.2                 # just behind the PCB (y=214)
Y_LID_BACK = 221.55                 # parting line PCB holder / frame (as in the original)
Y_FRAME_BACK = 242.3                # frame reaches into the plate (front face 242.27)
X_L = -355.5
Z_BOT, Z_TOP = sh.Z_HOUSING_BOT, -116.5

SHAFT = (-340.0, -136.32 + sh.LIFT)           # drive shaft (x, z)
PIN = (-326.9, -146.12 + sh.LIFT)             # idler pivot (x, z)
IDLER = (-331.4, -136.32 + sh.LIFT)           # idler shaft (x, z)
IDLER_SWING = (-2.0, 0.0, 6.0, 12.0)  # idler swing angles (degrees, away from the filament axis)
FIL_D, PTFE_D, PTFE_DEPTH = 2.2, 4.15, 8.0

INSERT_D, INSERT_L = 4.8, 5.5               # heat-set insert M3x5x4 (as in the Sherpa): Ø4.8 hole
BEARING_FRONT = 243.35                      # front edge of the rear bearing seat (as in the original)
# bosses around bearing seat, pin hole and PCB insert (x, z, r)
WALL = 1.5                                  # wall thickness of the bosses
LID_PADS = [(*SHAFT, 4.1 + WALL), (*PIN, 1.65 + WALL), (-330.2, -128.7 + gc.PCB_DZ, INSERT_D / 2 + WALL)]
KNOB_BOT = -126.57 + sh.LIFT - 6.0          # lower edge of the thumbscrew knob (Ø12)
Y_BEAM_BACK = 243.25 - 3.0 - 0.8            # 0.8 mm in front of the rim of the 50t gear (3 mm thick)
# parting line PCB holder / motor plate in the posts: the M3x12 from the motor side ends exactly at the
# end of the 4 mm heat-set insert (yellow part including the motor plate 8 mm)
Y_BEAM_FRONT = sh.MOTOR_FACE_Y - 12.0 + 4.0
LUG_W, LUG_T = 7.0, 3.0                     # mounting lugs: width, thickness
POST = 7.0                                  # cross-section of the posts (square)
LID_T = 3.0                                 # plate thickness of the PCB holder
X_FULL = (-367.548, -305.501)               # full width of the main body
POST_TOP_L = -126.2 + gc.PCB_DZ             # top of the left post (previous top of the holder, kept)
# PCB holder: full width, bottom as the PCBs, top flush with the block around the idler insert
PCB_RECT = (*X_FULL, -161.2 + gc.PCB_DZ, sh.idler_insert().bounding_box().center().Z + 4.8 / 2 + 1.5)
# motor plate
MOTOR_PLATE_T = 2.0
MOTOR_EAR_DX, MOTOR_EAR_R = 21.92, 3.5      # hole distance of the motor ears, ear radius
MOTOR_BOSS_R = 8.0 + CLR                    # pinion + centring collar
SLOT_HALF = 1.5                             # half slot length (centre travel ±1.5 mm)
# rear idler pin seat
PIN_BOSS_R, PIN_BOSS_Y0 = 1.65 + WALL, 235.35
PCB_INSERTS = [(-330.2, -128.7 + gc.PCB_DZ)]   # PCB shoulder screw in the PCB holder (the lower one is in the carrier)


def ycyl(x, z, r, y0, y1):
    return Pos(x, y0, z) * Rot(-90, 0, 0) * Cylinder(r, y1 - y0, align=MIN)


def turn(deg):
    """Rotation about the idler pivot; positive = clockwise seen from the front (idler towards +X)."""
    px, pz = PIN
    return Pos(px, 0, pz) * Rot(0, deg, 0) * Pos(-px, 0, -pz)


def idler_sweep():
    """Moving idler parts over the swing range. Clearance: -2° on the filament side, ±CLR in Y.
    (Copies are subtracted one by one; fusing all copies needs too much RAM.)"""
    # idler gear (toothing, 457 faces) and shaft as simple cylinders, the needle bearings are inside the gear
    ix, iz = IDLER
    parts = sh.get("idler_arm_short") + [ycyl(ix, iz, 4.75, 222.05, 236.05), ycyl(ix, iz, 1.5, 218.45, 238.45)]
    poses = [turn(deg) for deg in IDLER_SWING] + [Pos(0, CLR, 0), Pos(0, -CLR, 0)]
    return [loc * p for loc in poses for p in parts]


@lru_cache(None)
def clearances(with_sweep=True):
    """Everything cut out of the housing parts (list of solids)."""
    sx, sz = SHAFT
    cut = [
        ycyl(sx, sz, 5.5, Y_LID_BACK - 0.1, 236.6),               # drive gear, shim
        ycyl(sx, sz, 13.6, 240.25 - CLR, BEARING_FRONT),          # 50t gear, rim (3 mm, up to y=243.25)
        ycyl(sx, sz, 6.3 + CLR, 235.6, 240.3),                    # 50t gear, hub (max. Ø12.6)
        ycyl(*sh.MOTOR_AX, 4.5, 240.05 - CLR, Y_FRAME_BACK + 1),  # pinion (starts at y=240.05)
        # bearing seats
        ycyl(sx, sz, 8.2 / 2, 218.98, Y_LID_BACK + 0.3),
        ycyl(sx, sz, 7.0 / 2, 218.25, 219.0),
        ycyl(sx, sz, 8.2 / 2, 243.0, 245.85),
        ycyl(sx, sz, 6.6 / 2, 245.8, 246.05),
        # idler pin
        ycyl(*PIN, 3.3 / 2, 218.7, Y_LID_BACK + 0.1),
        ycyl(*PIN, 3.3 / 2, 235.35, 238.9),
        ycyl(*PIN, 3.3 / 2, 221.0, 235.4),                        # pin between the two seats
    ]
    # filament: PTFE bore from the top, guide down to the carrier
    fx, fy = cm.AX_X, cm.AX_Y
    cut.append(Pos(fx, fy, Z_BOT - 1) * Cylinder(FIL_D / 2, Z_TOP - Z_BOT + 2, align=MIN))
    cut.append(Pos(fx, fy, Z_TOP - PTFE_DEPTH) * Cylinder(PTFE_D / 2, PTFE_DEPTH + 1, align=MIN))
    # insert for the thumbscrew (along X, inserted from +X) + screw channel
    cut.append(Pos(-333.4, 228.8, -126.57 + sh.LIFT) * Rot(0, 90, 0) * Cylinder(4.8 / 2, 7.4, align=MIN))
    cut.append(Pos(-333.4, 228.8, -126.57 + sh.LIFT) * Rot(0, 90, 0) * Cylinder(3.4 / 2, 12.0, align=MIN))
    if with_sweep:
        cut += idler_sweep()
    return tuple(cut)


def hinge_boss():
    """Rear idler pin seat: cylinder (axis along Y) at the front of the motor plate."""
    px, pz = PIN
    return ycyl(px, pz, PIN_BOSS_R, PIN_BOSS_Y0, sh.MOTOR_FACE_Y - MOTOR_PLATE_T + 0.01)


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _post_geo():
    x0, x1, _, _ = PCB_RECT
    xl = sh.MOTOR_AX[0] - MOTOR_EAR_DX         # left: centred below the slot of the motor screw
    xr = x1 - (xl - x0)                        # right: same distance from the edge as the left one
    return xl, xr, POST_TOP_L, KNOB_BOT - 0.4  # the right post ends below the thumbscrew knob


def posts():
    """Two posts of the PCB holder (x centre, z0, z1): left up to the top of the PCB holder,
    right up to just below the thumbscrew knob."""
    xl, xr, zl, zr = _post_geo()
    return [(xl, Z_BOT, zl), (xr, Z_BOT, zr)]


def post_screws():
    """Four M3 screw points (x, z): bottom 1 mm above the post centre, top centred in the upper post end."""
    xl, xr, zl, zr = _post_geo()
    zb = Z_BOT + POST / 2 + 1.0
    return [(xl, zb), (xr, zb), (xl, zl - POST / 2), (xr, zr - POST / 2)]


def struts():
    """Post sections from Y_BEAM_FRONT to the motor plate, as high as the posts of the PCB holder."""
    yw = sh.MOTOR_FACE_Y - MOTOR_PLATE_T
    r = None
    for x, za, zb in posts():
        s_ = box(x - POST / 2, x + POST / 2, Y_BEAM_FRONT, yw + 0.01, za, zb)
        r = s_ if r is None else r + s_
    return r


def lug_holes():
    """Mounting lugs (hole centre x, y) at the foot of the posts, pointing outwards to the side edge,
    midway between PCB plate and motor plate."""
    x0, x1, _, _ = PCB_RECT
    y = (Y_LID_FRONT + LID_T + sh.MOTOR_FACE_Y - MOTOR_PLATE_T) / 2
    return [(x0 + LUG_W / 2, y), (x1 - LUG_W / 2, y)]          # semicircle up to the side edge


def build_lugs():
    out = None
    for (xs, _, _), (xh, yh) in zip(posts(), lug_holes()):
        lug = box(min(xs, xh), max(xs, xh), yh - LUG_W / 2, yh + LUG_W / 2, Z_BOT, Z_BOT + LUG_T)
        lug += Pos(xh, yh, Z_BOT) * Cylinder(LUG_W / 2, LUG_T, align=MIN)
        lug -= Pos(xh, yh, Z_BOT - 1) * Cylinder(1.7, LUG_T + 2, align=MIN)
        out = lug if out is None else out + lug
    return out


def build_pcb_holder_base():
    """PCB holder: 3 mm plate of PCB size, bosses at bearing, idler pin and PCB insert."""
    x0, x1, z0, z1 = PCB_RECT
    d = Pos((x0 + x1) / 2, Y_LID_FRONT, (z0 + z1) / 2) * Box(
        x1 - x0, LID_T, z1 - z0, align=(Align.CENTER, Align.MIN, Align.CENTER))
    for x, z, r in LID_PADS:
        d += ycyl(x, z, r, Y_LID_FRONT, Y_LID_BACK)
    # the holder ends at the top of the Goliath carrier (below it the carrier holds the PCB)
    d -= Pos(0, 0, Z_BOT) * Box(2000, 2000, 100, align=(Align.CENTER, Align.CENTER, Align.MAX))
    # clearances without the idler contour; at the idler only simplified boxes are cut (arm_clearance_cut)
    for c in clearances(with_sweep=False):
        cb = c.bounding_box()
        if cb.min.Y < Y_LID_BACK and cb.max.X > X_L and cb.max.Y > Y_LID_FRONT:
            d -= c
    # posts up to the parting line (the rest belongs to the motor plate)
    for x, za, zb in posts():
        d += box(x - POST / 2, x + POST / 2, Y_LID_FRONT + LID_T - 0.01, Y_BEAM_FRONT, za, zb)
    # mounting lugs to the main body (M3 from the top, heat-set insert in the Goliath carrier)
    d += build_lugs()
    # heat-set inserts in the posts (screwed from the motor side)
    for x, z in post_screws():
        d -= ycyl(x, z, INSERT_D / 2, Y_BEAM_FRONT - INSERT_L, Y_BEAM_FRONT + 0.1)
    for x, z in PCB_INSERTS:
        d -= ycyl(x, z, INSERT_D / 2, Y_LID_FRONT - 0.1, Y_LID_FRONT + INSERT_L)
    return max(d.clean().solids(), key=lambda s: s.volume)


def motor_plate_base():
    """2 mm plate in front of the motor mounting face, down to the main body (separate part): passage for
    pinion/centring collar, rear bearing seat (via clearances), two M3 holes in the motor ears, the left one
    (seen from the front) as a slot."""
    mx, mz = sh.MOTOR_AX
    y1 = sh.MOTOR_FACE_Y
    y0 = y1 - MOTOR_PLATE_T
    x0, x1 = X_FULL
    z1 = sh.get("motor")[0].bounding_box().max.Z
    p = Pos((x0 + x1) / 2, (y0 + y1) / 2, (Z_BOT + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - Z_BOT)
    # passage for pinion/centring collar as a slot: swivelling about the right screw moves the axis
    # by SLOT_HALF * (distance axis-pivot) / (screw distance) = SLOT_HALF / 2 vertically
    dz = SLOT_HALF / 2
    p -= ycyl(mx, mz - dz, MOTOR_BOSS_R, y0 - 1, y1 + 1) + ycyl(mx, mz + dz, MOTOR_BOSS_R, y0 - 1, y1 + 1)
    p -= Pos(mx, (y0 + y1) / 2, mz) * Box(2 * MOTOR_BOSS_R, MOTOR_PLATE_T + 2, 2 * dz)
    p -= ycyl(mx + MOTOR_EAR_DX, mz, 1.7, y0 - 1, y1 + 1)
    xl = mx - MOTOR_EAR_DX                              # vertical slot
    p -= ycyl(xl, mz - SLOT_HALF, 1.7, y0 - 1, y1 + 1) + ycyl(xl, mz + SLOT_HALF, 1.7, y0 - 1, y1 + 1)
    p -= Pos(xl, y0 + MOTOR_PLATE_T / 2, mz) * Box(3.4, MOTOR_PLATE_T + 2, 2 * SLOT_HALF)
    return p


def motor_slot(y0, y1):
    mx, mz = sh.MOTOR_AX
    xl = mx - MOTOR_EAR_DX
    return (ycyl(xl, mz - SLOT_HALF, 1.7, y0, y1) + ycyl(xl, mz + SLOT_HALF, 1.7, y0, y1)
            + Pos(xl, (y0 + y1) / 2, mz) * Box(3.4, y1 - y0, 2 * SLOT_HALF))


def fan_bbox():
    """Position of the (shifted, mirrored) 2515 fan, from the SV08 model."""
    fan = next(p for l, p in cm.sv08_parts() if "2515_pipe_cooling_fan" in l)
    return (Pos(gc.FAN_SHIFT_X, 0, gc.FAN_DZ) * fan).mirror(gc.FAN_MIRROR).bounding_box()


pcb_holder = build_pcb_holder_base()


def build_motor_plate():
    """Motor plate as a separate part (stands on the main body), with rear bearing seat and idler pin seat."""
    p = motor_plate_base() + hinge_boss() + struts()
    # ring in front of the plate: bearing seat 2.5 mm deep as in the original (MR85 fully sunk)
    p += ycyl(*SHAFT, 8.2 / 2 + WALL, BEARING_FRONT, sh.MOTOR_FACE_Y - MOTOR_PLATE_T + 0.01)
    for x, z in post_screws():                                       # M3 passage to the heat-set inserts
        p -= ycyl(x, z, 1.7, Y_BEAM_FRONT - 0.1, sh.MOTOR_FACE_Y + 0.1)
    y0 = PIN_BOSS_Y0
    for c in clearances():
        cb = c.bounding_box()
        if cb.max.Y > y0 and cb.min.Y < sh.MOTOR_FACE_Y:
            p -= c
    return max(p.clean().solids(), key=lambda s: s.volume)


motor_plate = build_motor_plate()


# Filament guides above and below the drive gears, dimensions as in the original Sherpa housing
# (measured there relative to the drive shaft, z offset from the shaft axis)
GUIDE_D, BORE_D = 2.1, 4.2               # guide / bore (PTFE resp. hole)
GUIDE_WALL = 1.5
ARM_EDGE = (-330.0, -130.975)            # point on the top edge of the idler arm in front of the PCB insert
ARM_SLOPE = 17.8                         # inclination of this edge towards +X (degrees)
ARM_CLR = 0.5                            # clearance between PCB holder and idler arm
INSERT_WALL = 1.5                        # wall around the thumbscrew insert
GEAR_CUT_R = 4.75 + 0.75                 # drive/idler gear (r4.75) with clearance
UPPER = dict(top=18.0, bore_bot=5.32, bot=2.42)     # top: bore down to 5.32, guide down to 2.42 above the shaft
LOWER = dict(top=-2.38, guide_bot=-5.28)           # bottom: guide down to -5.28, below it the bore down to the carrier


def zcyl(r, z0, z1):
    return Pos(cm.AX_X, cm.AX_Y, z0) * Cylinder(r, z1 - z0, align=MIN)


def build_filament_guides():
    sz = SHAFT[1]
    ro = BORE_D / 2 + GUIDE_WALL
    up = zcyl(ro, sz + UPPER["bot"], sz + UPPER["top"])
    lo = zcyl(ro, Z_BOT, sz + LOWER["top"])
    # webs towards the PCB holder, as wide as the tube (within the height of the holder)
    yb = Y_LID_FRONT + LID_T - 0.01
    z_lid_top = PCB_RECT[3]
    up += box(cm.AX_X - ro, cm.AX_X + ro, yb, cm.AX_Y, sz + UPPER["bot"], min(sz + UPPER["top"], z_lid_top))
    lo += box(cm.AX_X - ro, cm.AX_X + ro, yb, cm.AX_Y, Z_BOT, sz + LOWER["top"])
    # bores after the webs, so they stay free
    up -= zcyl(GUIDE_D / 2, sz + UPPER["bot"] - 1, sz + UPPER["top"] + 1)
    up -= zcyl(BORE_D / 2, sz + UPPER["bore_bot"], sz + UPPER["top"] + 1)
    lo -= zcyl(GUIDE_D / 2, Z_BOT - 1, sz + LOWER["top"] + 1)
    lo -= zcyl(BORE_D / 2, Z_BOT - 1, sz + LOWER["guide_bot"])
    # recesses for drive gear and idler gear: two equal circles (axes along Y), symmetric
    # to the filament axis
    for x in (SHAFT[0], 2 * cm.AX_X - SHAFT[0]):
        c = ycyl(x, sz, GEAR_CUT_R, cm.AX_Y - 20, cm.AX_Y + 20)
        up -= c
        lo -= c
    return up, lo


def idler_insert_holder():
    """Cylinder (axis along X) around the thumbscrew insert, position as in the original Sherpa.
    Joins the upper filament guide; the insert is inserted from +X."""
    q = sh.idler_insert().bounding_box()
    c = q.center()
    x0, x1 = q.min.X - 0.6 - INSERT_WALL, q.max.X
    h = Pos(x0, c.Y, c.Z) * Rot(0, 90, 0) * Cylinder(4.8 / 2 + INSERT_WALL, x1 - x0, align=MIN)
    h -= idler_insert_bore()
    return h


def arm_clearance_cut(part):
    """Simplified clearance cut to the idler arm: per swing position the box around the part of the arm
    that comes close to the part, enlarged by ARM_CLR and limited to the outline of the part."""
    tb = part.bounding_box()
    k = ARM_CLR
    zone = box(tb.min.X - k, tb.max.X + k, tb.min.Y - k, tb.max.Y + k, tb.min.Z - k, tb.max.Z + k)
    out = []
    for deg in (0.0, 6.0, 12.0):
        near = (turn(deg) * sh.get("idler_arm_short")[0]) & zone
        if not (near and near.solids()):
            continue
        ab = near.bounding_box()
        c = box(ab.min.X - ARM_CLR, ab.max.X + ARM_CLR, ab.min.Y - ARM_CLR, ab.max.Y + ARM_CLR,
                ab.min.Z - ARM_CLR, ab.max.Z + ARM_CLR)
        out.append(c & box(tb.min.X, tb.max.X, tb.min.Y, tb.max.Y, tb.min.Z, tb.max.Z))
    return out


def idler_insert_bore():
    q = sh.idler_insert().bounding_box()
    c = q.center()
    return Pos(q.min.X - 0.6, c.Y, c.Z) * Rot(0, 90, 0) * Cylinder(4.8 / 2, q.max.X - q.min.X + 1.6, align=MIN)


def upper_reinforcement():
    """Block around the idler insert: from the left post to the end of the insert, from the back of the
    PCB holder to behind the insert holder, at the height of the insert holder."""
    q = sh.idler_insert().bounding_box()
    c = q.center()
    r = 4.8 / 2 + INSERT_WALL
    xl = posts()[0][0] + POST / 2 - 0.01
    return box(xl, q.max.X, Y_LID_FRONT + LID_T - 0.01, c.Y + r, c.Z - r, c.Z + r)


# Filament guides, idler insert holder and reinforcement belong to the PCB holder
guide_upper, guide_lower = build_filament_guides()
_d = pcb_holder + guide_upper + guide_lower + idler_insert_holder() + upper_reinforcement()
# simplified clearance cuts to the idler arm at the two bosses with heat-set inserts
# PCB insert: everything below an inclined line ARM_CLR above the top edge of the idler arm
# (rising with ARM_SLOPE towards +X) is cut from the boss down to the PCB wall
for _x, _z, _r in LID_PADS[2:]:
    _above = Pos(ARM_EDGE[0], (Y_LID_FRONT + Y_LID_BACK) / 2, ARM_EDGE[1] + ARM_CLR) * Rot(0, -ARM_SLOPE, 0) * Box(
        40, 40, 20, align=(Align.CENTER, Align.CENTER, Align.MIN))
    _d -= box(_x - _r - 0.1, _x + _r + 0.1, Y_LID_FRONT + LID_T, Y_LID_BACK + 0.1, _z - _r - 0.1, _z + _r) - _above
for _c in arm_clearance_cut(idler_insert_holder()):
    _d -= _c
_d -= idler_insert_bore()                                            # idler heat-set insert
for _x, _z in PCB_INSERTS:                                           # PCB insert
    _d -= ycyl(_x, _z, INSERT_D / 2, Y_LID_FRONT - 0.1, Y_LID_FRONT + INSERT_L)
_sz = SHAFT[1]
_d -= zcyl(GUIDE_D / 2, _sz + UPPER["bot"] - 1, _sz + UPPER["top"] + 1)       # keep the filament channel free
_d -= zcyl(BORE_D / 2, _sz + UPPER["bore_bot"], _sz + UPPER["top"] + 1)
_d -= ycyl(*SHAFT, 7.0 / 2, Y_LID_FRONT - 1, 219.0)        # Ø7 as extension of the shaft axis to the front (as original)
pcb_holder = max(_d.clean().solids(), key=lambda s: s.volume)

if __name__ == "__main__":
    cm.OUT.mkdir(exist_ok=True)
    pcb_holder.label = "pcb_holder"
    export_step(pcb_holder, str(cm.OUT / "pcb_holder.step"))
    print(f"PCB holder: volume {pcb_holder.volume:.0f} mm^3, faces {len(pcb_holder.faces())}")
