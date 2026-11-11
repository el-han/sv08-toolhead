"""
Main body for the Sherpa conversion.

Block from the Eddy probe up to the Goliath carrier, with the width and depth of the original
extruder_install_plate, with all its holes, pockets for the screws of the neighbouring parts and box-shaped
recesses for the belt clamps (including the side flange of the sliding panel) and for the air duct.
The Goliath carrier (goliath_carrier.py) is fused with it; the PCB holder and the motor plate (housing.py)
are separate parts.

FDM: printed with the back side (y=Y_BACK) on the bed.
"""
from build123d import Align, Box, Compound, Cylinder, Pos, Rot, export_step

import common as cm
import goliath_carrier as gc
import housing as hs

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
CLR = 0.4
BIG = 400

# Extent of the block: width and depth of the original install plate, from the Eddy probe to the carrier
X_PL, X_PR = -367.548, -305.501            # 62 mm
Y_FRONT, Y_BACK = 242.2747, 250.7747
Z_SPLIT = gc.Z_TOP                         # top of the Goliath carrier
Z_BLOCK_BOT = -222.2                       # bottom of the Eddy probe

# Holes of the original install plate (x, z, r), all along Y and through
THROUGH_HOLES = {
    "carriage_1": (-361.525, -201.175, 1.75),     # M3 to the carriage
    "carriage_2": (-315.825, -194.675, 1.75),
    "carriage_3": (-310.825, -158.675, 1.75),
    "sensor": (-352.525, -215.975, 1.40),         # sensor holder
    "air_duct": (-352.525, -199.975, 2.00),       # rear air duct
    "rail_left": (-341.525, -202.175, 2.00),      # M3 from the back (sliding panel)
    "rail_right": (-325.525, -202.175, 2.00),
}
# screws of the old motor (dropped), identified by their position (x, z)
OLD_MOTOR_SCREWS = ((-322.65, -133.35), (-361.35, -153.95))


def box_cut(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def y_hole(x, z, r, y0, y1):
    return Pos(x, y0, z) * Rot(-90, 0, 0) * Cylinder(r, y1 - y0, align=MIN)


def stepped_cylinder(part, clr, step=0.5):
    """Substitute body for a screw: stepped smooth cylinders along its axis (radius per section
    from slices), enlarged by clr. No thread faces."""
    pb = part.bounding_box()
    sz = (pb.size.X, pb.size.Y, pb.size.Z)
    ax = sz.index(max(sz))
    lo, hi = (pb.min.X, pb.min.Y, pb.min.Z)[ax], (pb.max.X, pb.max.Y, pb.max.Z)[ax]
    c = pb.center()
    rot = [Rot(0, 90, 0), Rot(-90, 0, 0), Rot(0, 0, 0)][ax]
    segs = []
    t = lo
    while t < hi - 1e-6:
        t1 = min(t + step, hi)
        dims = [BIG, BIG, BIG]
        dims[ax] = t1 - t
        ctr = [c.X, c.Y, c.Z]
        ctr[ax] = (t + t1) / 2
        r = 0.0
        for so in part.solids():
            x = so & (Pos(*ctr) * Box(*dims))
            if x and x.solids():
                xb = Compound(list(x.solids())).bounding_box()
                cross = [xb.size.X, xb.size.Y, xb.size.Z]
                del cross[ax]
                r = max(r, max(cross) / 2)
        r = round(r, 1)
        if segs and segs[-1][2] == r:
            segs[-1][1] = t1
        else:
            segs.append([t, t1, r])
        t = t1
    out = None
    for a, e, r in segs:
        if r <= 0:
            continue
        ctr = [c.X, c.Y, c.Z]
        ctr[ax] = (a + e) / 2
        cyl = Pos(*ctr) * rot * Cylinder(r + clr, e - a + 2 * clr * (a == lo or e == hi))
        out = cyl if out is None else out + cyl
    return out


def _old_motor_screw(p):
    c = p.bounding_box().center()
    return any(abs(c.X - x) < 1.5 and abs(c.Z - z) < 1.5 for x, z in OLD_MOTOR_SCREWS)


def box_pockets():
    """Box-shaped recesses: belt clamps including the side flange of the sliding panel, open to the side
    and all the way through, air duct open to the bottom."""
    out = []
    blk = box_cut(X_PL, X_PR, Y_FRONT, Y_BACK, Z_BLOCK_BOT, Z_SPLIT)
    panel = cm.find("xtruder_sliding_panel")[0]
    xm = (X_PL + X_PR) / 2
    for p in cm.find("belt_lock_block"):
        q = p.bounding_box()
        left = q.center().X < xm
        # include the side flange of the sliding panel on the same side (it sits in the plate)
        half = box_cut(X_PL, xm, Y_FRONT, Y_BACK, Z_BLOCK_BOT, Z_SPLIT) if left else \
            box_cut(xm, X_PR, Y_FRONT, Y_BACK, Z_BLOCK_BOT, Z_SPLIT)
        f = panel & half
        fb = Compound(list(f.solids())).bounding_box() if f and f.solids() else q
        xa, xb = min(q.min.X, fb.min.X), max(q.max.X, fb.max.X)
        za, zb = min(q.min.Z, fb.min.Z), max(q.max.Z, fb.max.Z)
        x0, x1 = (X_PL - 1, xb + CLR) if left else (xa - CLR, X_PR + 1)
        out.append(box_cut(x0, x1, Y_FRONT - 1, Y_BACK + 1, za - CLR, zb + CLR))   # all the way through
    # air duct: only the part that sits in the plate (the screws above it need material)
    q = (cm.find("air_deflector2")[0] & blk).bounding_box()
    out.append(box_cut(q.min.X - CLR, q.max.X + CLR, Y_FRONT - 1, Y_BACK + 1, Z_BLOCK_BOT - 1, q.max.Z + CLR))
    return out


def block():
    blk = box_cut(X_PL, X_PR, Y_FRONT, Y_BACK, Z_BLOCK_BOT, Z_SPLIT)
    for x, z, r in THROUGH_HOLES.values():
        blk -= y_hole(x, z, r, Y_FRONT - 1, Y_BACK + 1)
    for c in box_pockets():
        blk -= c
    # screws of neighbouring parts that reach into the block (counterbores/heads)
    bb = blk.bounding_box()
    for label, p in cm.sv08_parts():
        if "screw" not in label or any(k in label for k in cm.REMOVED_SHERPA) or _old_motor_screw(p):
            continue
        q = p.bounding_box()
        if (q.max.Y < bb.min.Y or q.min.Y > bb.max.Y or q.max.X < bb.min.X or q.min.X > bb.max.X
                or q.max.Z < bb.min.Z or q.min.Z > bb.max.Z):
            continue
        if any((s & blk) and sum(x.volume for x in (s & blk).solids()) > 0.01 for s in p.solids()):
            blk -= stepped_cylinder(p, CLR)
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
