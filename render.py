"""Writes the printable parts and the complete assembly as STEP files to out/.

Usage:  .venv/bin/python render.py [--png] [--image]
  out/extruder_body_sherpa.step, out/pcb_holder.step, out/motor_plate.step   printable parts
  out/assembly.step                                                         named, coloured assembly tree
  --png   additional matplotlib views (slow)
  --image shaded views of the assembly for the README (docs/toolhead_*.png)
"""
import sys

from build123d import export_step

import common
from assembly import TOOLHEAD, colored_parts
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


def shaded_image(items, file_name, direction, size=(1400, 1400), supersample=2, margin=0.06):
    """Orthographic, flat-shaded view of (shape, rgb) items (z-buffer rasteriser).
    `direction`: viewing direction (from the camera towards the part)."""
    import numpy as np
    from PIL import Image

    view = np.asarray(direction, float)
    view /= np.linalg.norm(view)
    right = np.cross(view, (0, 0, 1.0))
    right /= np.linalg.norm(right)
    up = np.cross(right, view)
    light = -view + 0.6 * up - 0.4 * right
    light /= np.linalg.norm(light)

    triangles, colors = [], []
    for shape, rgb in items:
        for solid in shape.solids():
            try:
                vertices, faces = solid.tessellate(0.05, 0.3)
            except Exception:
                continue
            if not faces:
                continue
            points = np.array([(v.X, v.Y, v.Z) for v in vertices])
            triangles.append(points[np.array(faces)])
            colors.append(np.tile(rgb, (len(faces), 1)))
    tri = np.concatenate(triangles)
    rgb = np.concatenate(colors)
    normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    normals /= np.linalg.norm(normals, axis=1, keepdims=True) + 1e-12
    normals[(normals @ view) > 0] *= -1                      # two-sided: towards the camera
    brightness = 0.35 + 0.65 * np.clip(normals @ light, 0, 1)
    shade = np.clip(rgb * brightness[:, None] * 255, 0, 255).astype(int)

    screen = np.stack([tri @ right, -(tri @ up)], axis=-1)
    depth = tri @ view                                       # larger = farther
    low, high = screen.reshape(-1, 2).min(axis=0), screen.reshape(-1, 2).max(axis=0)
    width, height = size[0] * supersample, size[1] * supersample
    scale = (1 - 2 * margin) * min(width / (high[0] - low[0]), height / (high[1] - low[1]))
    offset = np.array([width, height]) / 2 - (low + high) / 2 * scale
    pixels = screen * scale + offset

    zbuffer = np.full((height, width), np.inf)
    image = np.full((height, width, 3), 255, np.uint8)
    for (a, b, c), (za, zb, zc), color in zip(pixels, depth, shade):
        x0, x1 = int(max(min(a[0], b[0], c[0]), 0)), int(min(max(a[0], b[0], c[0]) + 1, width))
        y0, y1 = int(max(min(a[1], b[1], c[1]), 0)), int(min(max(a[1], b[1], c[1]) + 1, height))
        if x0 >= x1 or y0 >= y1:
            continue
        area = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        if abs(area) < 1e-9:
            continue
        px, py = np.meshgrid(np.arange(x0, x1) + 0.5, np.arange(y0, y1) + 0.5)
        w0 = ((b[0] - px) * (c[1] - py) - (b[1] - py) * (c[0] - px)) / area
        w1 = ((c[0] - px) * (a[1] - py) - (c[1] - py) * (a[0] - px)) / area
        w2 = 1 - w0 - w1
        inside = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
        z = w0 * za + w1 * zb + w2 * zc
        region = zbuffer[y0:y1, x0:x1]
        visible = inside & (z < region)
        region[visible] = z[visible]
        image[y0:y1, x0:x1][visible] = color
    Image.fromarray(image).resize(size, Image.LANCZOS).save(file_name)


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
    if "--image" in sys.argv:
        docs = common.OUT.parent / "docs"
        docs.mkdir(exist_ok=True)
        items = colored_parts()
        for name, direction in (("front", (-0.55, 1.0, -0.45)), ("back", (0.6, -1.0, -0.4))):
            shaded_image(items, str(docs / f"toolhead_{name}.png"), direction)
            print(f"docs/toolhead_{name}.png written")
