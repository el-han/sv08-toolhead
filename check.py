"""Collision check of the Sherpa/Goliath conversion.

Usage:  .venv/bin/python check.py
Checks the new parts and the Goliath against all SV08 and Sherpa parts, the fasteners and each other,
and checks that the filament path is free. Exit code 1 if anything collides.
"""
import sys

from build123d import Align, Cylinder, Pos

import assembly as asm
import common as cm
import fasteners as fs
import goliath_carrier as gc
import housing as hs
import sherpa as sh

FIL_R = 0.95                       # filament Ø1.75 with some clearance (guide Ø2.1)
TOL = 0.05                         # mm^3 below which an overlap is ignored (touching faces)


def overlap(a, c):
    ab, bb = a.bounding_box(), c.bounding_box()
    if (ab.min.X > bb.max.X or ab.max.X < bb.min.X or ab.min.Y > bb.max.Y or ab.max.Y < bb.min.Y
            or ab.min.Z > bb.max.Z or ab.max.Z < bb.min.Z):
        return 0.0
    vol = 0.0
    for sa in a.solids():
        for sb in c.solids():
            try:
                r = sa.intersect(sb)
                vol += sum(s.volume for s in r.solids()) if r else 0
            except Exception:
                return float("nan")
    return vol


def check():
    bad = False
    new = asm.NEW_PARTS
    others = ([(n, t, p) for n, t, p in asm.sv08_placed() + asm.sherpa_placed()]
              + [(n, "fastener", p) for n, p in fs.parts])
    for oname, obj in new:
        for n, t, p in others:
            if "heat_insert" in n:             # press fit (Ø5.0 in Ø4.8) is intended
                continue
            if oname == "Goliath" and "screw" in n:     # thread engagement in the Goliath core hole
                continue
            v = overlap(obj, p)
            if v > TOL or v != v:
                bad = True
                pb = p.bounding_box()
                print(f"  COLLISION {oname} <-> {n} ({t}): {v:.2f} mm^3  "
                      f"[{pb.min.X:.0f},{pb.max.X:.0f}]x[{pb.min.Y:.0f},{pb.max.Y:.0f}]x[{pb.min.Z:.0f},{pb.max.Z:.0f}]")
    for i in range(len(new)):
        for j in range(i + 1, len(new)):
            v = overlap(new[i][1], new[j][1])
            print(f"  {new[i][0]} <-> {new[j][0]}: {v:.2f} mm^3")
            if v > TOL:
                bad = True
    # filament path (Ø1.75 + clearance) from the Goliath inlet to the top of the upper guide must be free
    z0, z1 = gc.Z_BOT, hs.SHAFT[1] + hs.UPPER["top"]
    fil = Pos(cm.AX_X, cm.AX_Y, z0) * Cylinder(FIL_R, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    for oname, obj in new:
        if oname == "Goliath":
            continue
        v = overlap(obj, fil)
        if v > 0.001:
            bad = True
            print(f"  FILAMENT PATH blocked by {oname}: {v:.3f} mm^3")
    print(f"Nozzle z={gc.nozzle_z:.2f} (original {cm.ORIG_NOZZLE_Z}); "
          f"motor axis x={sh.MOTOR_AX[0]:.2f} z={sh.MOTOR_AX[1]:.2f}")
    print("No collisions." if not bad else "See collisions above.")
    return not bad


if __name__ == "__main__":
    sys.exit(0 if check() else 1)
