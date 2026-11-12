"""Writes the printable parts and the complete assembly as STEP files to out/.

Usage:  .venv/bin/python render.py [--png]
  out/extruder_body_sherpa.step, out/pcb_holder.step, out/motor_plate.step   printable parts
  out/assembly.step                                                         named, coloured assembly tree
  --png   additional matplotlib views (slow)
"""
import sys

from build123d import export_step

import common
from assembly import TOOLHEAD
from goliath_carrier import GOLIATH_CARRIER
from main_body import MAIN_BODY


def png(items, file_name, views, limits=((-372, -300), (190, 262), (-250, -100))):
    """Matplotlib views of (shape, colour, alpha) items; views are (elevation, azimuth, title)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(6 * len(views), 7))
    for i, (elev, azim, title) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        for shape, color, alpha in [(solid, c, a) for item, c, a in items for solid in item.solids()]:
            try:
                vertices, triangles = shape.tessellate(0.3, 0.5)
            except Exception:
                continue
            if not triangles:
                continue
            points = [(v.X, v.Y, v.Z) for v in vertices]
            ax.add_collection3d(Poly3DCollection([[points[a], points[b], points[c]] for a, b, c in triangles],
                                                 facecolors=color, alpha=alpha, shade=True,
                                                 lightsource=matplotlib.colors.LightSource(azdeg=225, altdeg=45)))
        ax.set_xlim(*limits[0]); ax.set_ylim(*limits[1]); ax.set_zlim(*limits[2])
        ax.set_box_aspect(tuple(high - low for low, high in limits))
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(title)
        ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    fig.tight_layout()
    fig.savefig(file_name, dpi=110)


if __name__ == "__main__":
    common.OUT.mkdir(exist_ok=True)
    for file_name, shape in TOOLHEAD.printed_parts:
        shape.label = file_name
        export_step(shape, str(common.OUT / f"{file_name}.step"))
    export_step(TOOLHEAD.step_tree(), str(common.OUT / "assembly.step"))
    print("out/assembly.step written")
    if "--png" in sys.argv:
        body = MAIN_BODY.solid
        context = [(p.shape, "#b0b0b0", 0.30) for p in TOOLHEAD.sv08_parts
                   if -260 < p.shape.bounding_box().center().Z < -90 and p.shape.bounding_box().size.Y < 120]
        context += [(p.shape, "#c060c0", 0.8) for p in TOOLHEAD.sherpa_parts]
        items = context + [(GOLIATH_CARRIER.goliath, "#d08030", 1.0), (body, "#20a060", 0.9)]
        png(items, str(common.OUT / "view_assembly.png"),
            [(10, -90, "front"), (10, 0, "right (+X)"), (25, -130, "iso")])
        png([(body, "#20a060", 1.0)], str(common.OUT / "view_main_body.png"), [(20, -60, "front"), (20, 120, "back")])
