"""Depth map of the back side of a plate STEP (usage: depth_map.py plate.step out.png)."""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from build123d import import_step

plate = import_step(sys.argv[1])
polys, depth = [], []
for s in plate.solids():
    v, tr = s.tessellate(0.05, 0.1)
    P = np.array([(q.X, q.Y, q.Z) for q in v])
    for a, b, c in tr:
        t = P[[a, b, c]]
        n = np.cross(t[1] - t[0], t[2] - t[0])
        if n[1] <= 1e-9:            # only faces pointing backwards (+Y)
            continue
        polys.append(t[:, [0, 2]]); depth.append(t[:, 1].mean())
order = np.argsort(depth)
fig, ax = plt.subplots(figsize=(9, 14))
pc = PolyCollection([polys[i] for i in order], array=np.array(depth)[order], cmap="viridis", edgecolors="face")
pc.set_clim(242, 250.8)
ax.add_collection(pc); ax.autoscale(); ax.set_aspect("equal"); ax.invert_xaxis()
ax.set_title("Back side, colour = height of the face (yellow = back side 250.8, dark = deep pocket)")
fig.colorbar(pc, ax=ax, shrink=0.5, label="y [mm]")
ax.grid(True, alpha=0.3)
fig.savefig(sys.argv[2], dpi=90, bbox_inches="tight")
