import sys, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from build123d import import_step

RES = 0.15
plate = import_step(sys.argv[1])
tris = []
for s in plate.solids():
    v, tr = s.tessellate(0.03, 0.1)
    P = np.array([(q.X, q.Y, q.Z) for q in v])
    for a, b, c in tr:
        t = P[[a, b, c]]
        n = np.cross(t[1] - t[0], t[2] - t[0])
        if n[1] > 1e-9:
            tris.append(t)
T = np.array(tris)
x0, x1 = T[:, :, 0].min() - 1, T[:, :, 0].max() + 1
z0, z1 = T[:, :, 2].min() - 1, T[:, :, 2].max() + 1
W, H = int((x1 - x0) / RES) + 1, int((z1 - z0) / RES) + 1
zb = np.full((H, W), -np.inf)
for t in T:
    u = (t[:, 0] - x0) / RES; w = (t[:, 2] - z0) / RES
    i0, i1 = int(np.floor(u.min())), int(np.ceil(u.max()))
    j0, j1 = int(np.floor(w.min())), int(np.ceil(w.max()))
    I, J = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1))
    px, pz = I + 0.5, J + 0.5
    d = (w[1] - w[2]) * (u[0] - u[2]) + (u[2] - u[1]) * (w[0] - w[2])
    if abs(d) < 1e-12:
        continue
    l0 = ((w[1] - w[2]) * (px - u[2]) + (u[2] - u[1]) * (pz - w[2])) / d
    l1 = ((w[2] - w[0]) * (px - u[2]) + (u[0] - u[2]) * (pz - w[2])) / d
    l2 = 1 - l0 - l1
    m = (l0 >= -1e-6) & (l1 >= -1e-6) & (l2 >= -1e-6)
    y = l0 * t[0, 1] + l1 * t[1, 1] + l2 * t[2, 1]
    Jm, Im, ym = J[m], I[m], y[m]
    np.maximum.at(zb, (Jm, Im), ym)

top = np.nanmax(zb[np.isfinite(zb)])
rec = np.isfinite(zb) & (zb < top - 0.05)
from scipy import ndimage
lab0, n0 = ndimage.label(rec)
# große Regionen nach Tiefenstufen aufteilen (dominante Ebene vs. Rest)
lab = np.zeros_like(lab0); n = 0
for k in range(1, n0 + 1):
    m = lab0 == k
    q = np.round(zb[m] / 0.05)
    vals, cnt = np.unique(q, return_counts=True)
    dom = vals[np.argmax(cnt)]
    plane = np.zeros_like(m); plane[m] = np.abs(q - dom) <= 1
    if m.sum() * RES * RES < 200:
        plane = m
    for part in (plane, m & ~plane):
        l, c = ndimage.label(part)
        for kk in range(1, c + 1):
            n += 1; lab[l == kk] = n
regions = []
for k in range(1, n + 1):
    m = lab == k
    area = m.sum() * RES * RES
    if area < 0.5:
        lab[m] = 0; continue
    jj, ii = np.nonzero(m)
    regions.append((k, area, zb[m].min(), zb[m].max(), ii.mean(), jj.mean()))
# Nummerierung von oben nach unten, links nach rechts (Ansicht von hinten: X gespiegelt)
regions.sort(key=lambda r: (-round(r[5] * RES / 8), -r[4]))
fig, ax = plt.subplots(figsize=(11, 16))
ext = (x0, x0 + W * RES, z0, z0 + H * RES)
bg = np.where(np.isfinite(zb), 0.85, 1.0)
ax.imshow(bg, origin="lower", extent=ext, cmap="gray", vmin=0, vmax=1)
cmap = plt.get_cmap("tab20")
col = np.zeros((H, W, 4))
info = []
for idx, (k, area, ymin, ymax, ci, cj) in enumerate(regions, 1):
    col[lab == k] = cmap((idx - 1) % 20)
    jj, ii = np.nonzero(lab == k)
    # Beschriftungspunkt: Pixel der Region nahe am Schwerpunkt
    p = np.argmin((ii - ci) ** 2 + (jj - cj) ** 2)
    X, Z = x0 + (ii[p] + .5) * RES, z0 + (jj[p] + .5) * RES
    ax.annotate(str(idx), (X, Z), ha="center", va="center", fontsize=11, weight="bold",
                bbox=dict(boxstyle="circle,pad=0.15", fc="white", ec="black", lw=0.8))
    info.append((idx, area, top - ymax, top - ymin, X, Z))
ax.imshow(col, origin="lower", extent=ext)
ax.invert_xaxis(); ax.set_xlabel("x [mm]"); ax.set_ylabel("z [mm]")
ax.set_title("Rückseite (Blick von hinten): nicht auf Oberfläche erhöhte Flächen\ngrau = plan, weiß = Durchbruch/außen")
fig.savefig(sys.argv[2], dpi=110, bbox_inches="tight")
for r in info:
    print(f"{r[0]:3d}  Fläche {r[1]:7.1f} mm²  Tiefe {r[2]:.2f}..{r[3]:.2f} mm  bei x={r[4]:.1f} z={r[5]:.1f}")
