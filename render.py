"""Writes the printable parts and the complete assembly as STEP files to out/.

Usage:  .venv/bin/python render.py [--png]
  out/extruder_body_sherpa.step, out/pcb_holder.step, out/motor_plate.step   printable parts
  out/assembly.step                                                         named, coloured assembly tree
  --png   additional matplotlib views (slow)
"""
import sys

from build123d import export_step

import assembly as asm
import common as cm

PRINTED = [("extruder_body_sherpa", asm.eb.body), ("pcb_holder", asm.hs.pcb_holder),
           ("motor_plate", asm.hs.motor_plate)]


def png(items, fname, views, lim=((-372, -300), (190, 262), (-250, -100))):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(6 * len(views), 7))
    for i, (elev, azim, title) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        for shape, color, alpha in [(s, c, a) for sh_, c, a in items for s in sh_.solids()]:
            try:
                verts, tris = shape.tessellate(0.3, 0.5)
            except Exception:
                continue
            if not tris:
                continue
            pts = [(v.X, v.Y, v.Z) for v in verts]
            ax.add_collection3d(Poly3DCollection([[pts[a], pts[b], pts[c]] for a, b, c in tris],
                                                 facecolors=color, alpha=alpha, shade=True,
                                                 lightsource=matplotlib.colors.LightSource(azdeg=225, altdeg=45)))
        ax.set_xlim(*lim[0]); ax.set_ylim(*lim[1]); ax.set_zlim(*lim[2])
        ax.set_box_aspect(tuple(l[1] - l[0] for l in lim))
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title)
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    fig.tight_layout()
    fig.savefig(fname, dpi=110)


if __name__ == "__main__":
    cm.OUT.mkdir(exist_ok=True)
    sv, sp = asm.sv08_placed(), asm.sherpa_placed()
    for fname, obj in PRINTED:
        obj.label = fname
        export_step(obj, str(cm.OUT / f"{fname}.step"))
    export_step(asm.assembly_tree(sv, sp), str(cm.OUT / "assembly.step"))
    print("out/assembly.step written")
    if "--png" in sys.argv:
        ctx = [(p, "#b0b0b0", 0.30) for n, t, p in sv if -260 < p.bounding_box().center().Z < -90
               and p.bounding_box().size.Y < 120]
        ctx += [(p, "#c060c0", 0.8) for n, t, p in sp]
        items = ctx + [(asm.gc.gol, "#d08030", 1.0), (asm.eb.body, "#20a060", 0.9)]
        png(items, str(cm.OUT / "view_assembly.png"), [(10, -90, "front"), (10, 0, "right (+X)"), (25, -130, "iso")])
        png([(asm.eb.body, "#20a060", 1.0)], str(cm.OUT / "view_main_body.png"), [(20, -60, "front"), (20, 120, "back")])
