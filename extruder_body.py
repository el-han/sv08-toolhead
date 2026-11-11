"""
Main body for the Sherpa conversion.

Block from the Eddy probe up to the Goliath carrier, full width and depth of the former install plate,
with all holes, screw pockets and box-shaped recesses for the belt clamps including the side flange of the
sliding panel and for the air duct (magnets are dropped). The Goliath carrier (goliath_carrier.py) is
fused with it; the PCB holder and the motor plate (housing.py) are separate parts.
"""
import math

from build123d import (Align, Box, Compound, Cylinder, Edge, Face, Pos, Rot, Vector, Wire, extrude,
                       export_step)

import common as cm
import goliath_carrier as gc
import housing as hs
import install_plate as ip
import sherpa as sh

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
CLR = 0.4
MX, MZ = sh.MOTOR_AX
MOTOR_R = 18.25 + CLR
EAR_DX = 21.95                 # distance ear arc centre to motor axis (x)
SCREW_DX, SCREW_R = 21.92, 1.7  # hole distance of the motor ears, M3 clearance
EAR_R = 3.5 + CLR              # radius of the ear arcs on the motor flange
Z_TOP = -101.5                 # top of the extension (motor top at -104.22)
Z_EXT_BOT = -146.035           # bottom of the extension (straight side edges of the plate)
Z_PLATE_TOP = -123.151         # top of the original plate


def hull_face(y):
    """Convex hull of the motor circle and the two ear circles (in the plane y)."""
    # outer tangents between circle (0, R) and (±d, r): angle of the tangent point
    a = math.asin((MOTOR_R - EAR_R) / EAR_DX)

    def pt(cx, r, ang):
        return Vector(MX + cx + r * math.sin(ang), y, MZ + r * math.cos(ang))
    # angle measured from +z, clockwise towards +x
    t = a                                   # tangent point top right (angle from +z)
    edges = [
        Edge.make_three_point_arc(pt(0, MOTOR_R, -t), pt(0, MOTOR_R, 0), pt(0, MOTOR_R, t)),
        Edge.make_line(pt(0, MOTOR_R, t), pt(EAR_DX, EAR_R, t)),
        Edge.make_three_point_arc(pt(EAR_DX, EAR_R, t), pt(EAR_DX, EAR_R, math.pi / 2), pt(EAR_DX, EAR_R, math.pi - t)),
        Edge.make_line(pt(EAR_DX, EAR_R, math.pi - t), pt(0, MOTOR_R, math.pi - t)),
        Edge.make_three_point_arc(pt(0, MOTOR_R, math.pi - t), pt(0, MOTOR_R, math.pi), pt(0, MOTOR_R, math.pi + t)),
        Edge.make_line(pt(0, MOTOR_R, math.pi + t), pt(-EAR_DX, EAR_R, math.pi + t)),
        Edge.make_three_point_arc(pt(-EAR_DX, EAR_R, math.pi + t), pt(-EAR_DX, EAR_R, -math.pi / 2), pt(-EAR_DX, EAR_R, -t)),
        Edge.make_line(pt(-EAR_DX, EAR_R, -t), pt(0, MOTOR_R, -t)),
    ]
    return sh.MOTOR_TURN * Face(Wire(edges))


def motor_screw_holes():
    """Through holes for the two motor screws (M3, through the plate from the front)."""
    out = []
    for dx in (-SCREW_DX, SCREW_DX):
        p = (sh.MOTOR_TURN * Pos(MX + dx, 0, MZ)).position
        out.append(ip.y_hole(p.X, p.Z, SCREW_R, ip.Y_PAD - 1, sh.MOTOR_FACE_Y + 0.1))
    return out


# Lower part of the main body as a block (replaces the install plate below the Goliath carrier)
X_PL, X_PR = -367.548, -305.501            # width as before (62 mm)
Z_SPLIT = gc.Z_TOP                         # top of the Goliath carrier
Z_BLOCK_BOT = -222.2                       # bottom of the Eddy probe


def box_cut(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def box_pockets():
    """Box-shaped recesses: belt clamps including the side flange of the sliding panel, open to the side
    and all the way through, air duct open to the bottom."""
    out = []
    blk = box_cut(X_PL, X_PR, ip.Y_FRONT, ip.Y_BACK, Z_BLOCK_BOT, Z_SPLIT)
    panel = cm.find("xtruder_sliding_panel")[0]
    xm = (X_PL + X_PR) / 2
    for p in cm.find("belt_lock_block"):
        q = p.bounding_box()
        left = q.center().X < xm
        # include the side flange of the sliding panel on the same side (it sits in the plate)
        half = box_cut(X_PL, xm, ip.Y_FRONT, ip.Y_BACK, Z_BLOCK_BOT, Z_SPLIT) if left else \
            box_cut(xm, X_PR, ip.Y_FRONT, ip.Y_BACK, Z_BLOCK_BOT, Z_SPLIT)
        f = panel & half
        fb = Compound(list(f.solids())).bounding_box() if f and f.solids() else q
        xa, xb = min(q.min.X, fb.min.X), max(q.max.X, fb.max.X)
        za, zb = min(q.min.Z, fb.min.Z), max(q.max.Z, fb.max.Z)
        x0, x1 = (X_PL - 1, xb + CLR) if left else (xa - CLR, X_PR + 1)
        out.append(box_cut(x0, x1, ip.Y_FRONT - 1, ip.Y_BACK + 1, za - CLR, zb + CLR))   # all the way through
    # air duct: only the part that sits in the plate (the screws above it need material)
    q = (cm.find("air_deflector2")[0] & blk).bounding_box()
    out.append(box_cut(q.min.X - CLR, q.max.X + CLR, ip.Y_FRONT - 1, ip.Y_BACK + 1, Z_BLOCK_BOT - 1, q.max.Z + CLR))
    return out


def block():
    blk = Pos((X_PL + X_PR) / 2, (ip.Y_FRONT + ip.Y_BACK) / 2, (Z_BLOCK_BOT + Z_SPLIT) / 2) * Box(
        X_PR - X_PL, ip.Y_BACK - ip.Y_FRONT, Z_SPLIT - Z_BLOCK_BOT)
    for x, z, r in ip.THROUGH_HOLES.values():
        blk -= ip.y_hole(x, z, r, ip.Y_FRONT - 1, ip.Y_BACK + 1)
    for c in box_pockets():
        blk -= c
    # screws of neighbouring parts that reach into the block (counterbores/heads)
    bb = blk.bounding_box()
    for label, p in cm.sv08_parts():
        if "screw" not in label or any(k in label for k in cm.REMOVED_SHERPA) or ip._old_motor_screw(p):
            continue
        q = p.bounding_box()
        if (q.max.Y < bb.min.Y or q.min.Y > bb.max.Y or q.max.X < bb.min.X or q.min.X > bb.max.X
                or q.max.Z < bb.min.Z or q.min.Z > bb.max.Z):
            continue
        if any((s & blk) and sum(x.volume for x in (s & blk).solids()) > 0.01 for s in p.solids()):
            blk -= ip.stepped_cylinder(p, CLR)
    return blk


def build():
    body = block()
    # the Goliath carrier is connected directly to the back wall
    body += gc.carrier
    for x, y in hs.lug_holes():                   # heat-set inserts for the lugs of the PCB holder
        body -= Pos(x, y, gc.Z_BOT - 1) * Cylinder(hs.INSERT_D / 2, gc.T + 2, align=MIN)
    # clearances for 50t gear, pinion, rear bearing and idler
    for c in hs.clearances():
        if c.bounding_box().max.Y > hs.Y_LID_BACK:
            body -= c
    return max(body.clean().solids(), key=lambda s: s.volume)


body = build()

if __name__ == "__main__":
    cm.OUT.mkdir(exist_ok=True)
    body.label = "extruder_body_sherpa"
    export_step(body, str(cm.OUT / "extruder_body_sherpa.step"))
    print(f"Volume {body.volume:.0f} mm^3, faces {len(body.faces())}")
