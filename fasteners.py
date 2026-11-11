"""
Screws and heat-set inserts of the new parts, for the assembly and the collision check.

The models are the purchased parts from the original STEPs: socket head cap screws DIN 912 from the SV08
(M3x12 and M2.5x8, cut to the required length) and the heat-set insert M3x5x4 from the Sherpa.
The press fit of the inserts in their holes is intended (check.py ignores "heat_insert").
"""
import math
from functools import lru_cache

from build123d import Align, Box, Pos, Rot

import common as cm
import goliath_carrier as gc
import housing as hs
import sherpa as sh

_TO_Z = [Rot(0, -90, 0), Rot(90, 0, 0), Rot(0, 0, 0)]       # axis x / y / z -> +z


def _radius_at(shape, z):
    sl = shape & (Pos(0, 0, z) * Box(40, 40, 0.05))
    if not (sl and sl.solids()):
        return 0.0
    q = sl.bounding_box()
    return max(q.size.X, q.size.Y) / 2


def _canon(part, axis_longest=True):
    """Put the part onto the z axis (centre at the origin), axis = longest or shortest extent."""
    q = part.bounding_box()
    sz = [q.size.X, q.size.Y, q.size.Z]
    ax = sz.index(max(sz) if axis_longest else min(sz))
    return _TO_Z[ax] * (Pos(-q.center().X, -q.center().Y, -q.center().Z) * part), sz[ax]


@lru_cache(None)
def _screw_template(kind):
    """Screw with the head bearing face at z=0, head at z<0, shank along +z."""
    label, total, head_d = {"M3": ("hex_socket_cylindrical_head_screws", 15.0, 5.5),
                            "M2.5": ("hex_socket_cylindrical_head_screws", 10.5, 4.5)}[kind]
    part = next(p for l, p in cm.sv08_parts() if label in l
                and abs(max(p.bounding_box().size.X, p.bounding_box().size.Y, p.bounding_box().size.Z) - total) < 0.05
                and abs(sorted([p.bounding_box().size.X, p.bounding_box().size.Y, p.bounding_box().size.Z])[1] - head_d) < 0.05)
    c, ln = _canon(part)
    if _radius_at(c, ln / 2 - 0.3) > _radius_at(c, -ln / 2 + 0.3):     # turn the head downwards
        c = Rot(180, 0, 0) * c
    z = -ln / 2
    while _radius_at(c, z + 0.1) > head_d / 2 - 0.4:                   # find the head bearing face
        z += 0.1
    return Pos(0, 0, -(z + 0.1)) * c, ln - (z + 0.1 + ln / 2)


@lru_cache(None)
def _insert_template():
    """Heat-set insert M3x5x4 (Sherpa) from z=0 to z=4."""
    c, ln = _canon(sh.idler_insert(), axis_longest=False)
    return Pos(0, 0, ln / 2) * c


def _to_dir(d):
    """Rotation that turns +z into direction d."""
    return {(1, 0, 0): Rot(0, 90, 0), (-1, 0, 0): Rot(0, -90, 0),
            (0, 1, 0): Rot(-90, 0, 0), (0, -1, 0): Rot(90, 0, 0),
            (0, 0, 1): Rot(0, 0, 0), (0, 0, -1): Rot(180, 0, 0)}[d]


def screw(p, d, length, kind="M3"):
    """Socket head cap screw with its head bearing face at point p, shank in direction d, shank length."""
    t, ln = _screw_template(kind)
    assert length <= ln + 0.15, (kind, length, ln)         # head bearing face found to within 0.1 mm
    t = t - Pos(0, 0, length) * Box(40, 40, 40, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return Pos(*p) * _to_dir(d) * t


def heat_insert(p, d):
    """Heat-set insert M3x5x4, starting at the hole opening p, going in direction d into the part."""
    return Pos(*p) * _to_dir(d) * _insert_template()


def build():
    out = []
    yb, yw = hs.Y_BEAM_FRONT, sh.MOTOR_FACE_Y
    # PCB holder <-> motor plate: insert in the post, screw from the motor side
    for i, (x, z) in enumerate(hs.post_screws(), 1):
        out.append((f"heat_insert_post_{i}", heat_insert((x, yb, z), (0, -1, 0))))
        out.append((f"screw_M3x12_post_{i}", screw((x, yw, z), (0, -1, 0), 12)))
    # motor: 2x M3x6 from the front through the motor plate (the left one through the slot)
    mx, mz = sh.MOTOR_AX
    for side, dx in (("right", hs.MOTOR_EAR_DX), ("left", -hs.MOTOR_EAR_DX)):
        out.append((f"screw_M3x6_motor_{side}",
                    screw((mx + dx, yw - hs.MOTOR_PLATE_T, mz), (0, 1, 0), 6)))
    # PCB shoulder screws: upper insert in the PCB holder, lower one in the Goliath carrier
    for i, (x, z) in enumerate(hs.PCB_INSERTS + [gc.PCB_SCREW], 1):
        out.append((f"heat_insert_pcb_{i}", heat_insert((x, hs.Y_LID_FRONT, z), (0, 1, 0))))
    # lugs of the PCB holder: M3x8 from the top, heat-set insert from below in the Goliath carrier
    for i, (x, y) in enumerate(hs.lug_holes(), 1):
        out.append((f"screw_M3x8_lug_{i}", screw((x, y, hs.Z_BOT + hs.LUG_T), (0, 0, -1), 8)))
        out.append((f"heat_insert_lug_{i}", heat_insert((x, y, gc.Z_BOT), (0, 0, 1))))
    # 2515 fan: 2x M3x12 from the outside through the wedges into the upper fan holes
    for i, (y, z) in enumerate(gc.FAN_HOLES, 1):
        out.append((f"screw_M3x12_fan_{i}", screw((gc.X_L, y, z), (1, 0, 0), 12)))
    # Goliath: 4x M2.5x5 from the top (2.6 mm in the thread, core hole 3.4 mm deep), head in the counterbore
    for i, deg in enumerate(gc.GOL_SCREWS_DEG, 1):
        x = cm.AX_X + gc.GOL_SCREW_R * math.cos(math.radians(deg))
        y = cm.AX_Y + gc.GOL_SCREW_R * math.sin(math.radians(deg))
        out.append((f"screw_M2_5x5_goliath_{i}",
                    screw((x, y, gc.Z_TOP - gc.HEAD_DEPTH), (0, 0, -1), 5, "M2.5")))
    return out


parts = build()
