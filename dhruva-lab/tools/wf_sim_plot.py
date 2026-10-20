"""Shows what the simulated corner sweeps cleaned, scene by scene, and what they left.

    cd dhruva-lab && cargo run --release --bin sweep -- --dump /tmp/wf
    uv run --with numpy --with matplotlib tools/wf_sim_plot.py /tmp/wf --out /tmp/wf.png

It writes the overview (legend, what each seed changes, a table of every run) to --out, and
pages of panels next to it (wf_1.png, …): a row per scene, a column per seed.

Each panel shows:
- light green: the floor the brushes swept, the bench's own cells (the side brush's disc and
  the main roller's 16 cm strip along the true path; not the whole body);
- the footprint every ~10 s: the body (grey), the roller across it (dark line), the side
  brush's disc (blue);
- red: the floor left unswept along the walls, from the bench's own edge samples, the scored
  "uncleaned area";
- orange: pockets, unswept floor the path closed round away from the walls (inside a loop
  round a leg), hard to reach later;
- walls in black, glass in cyan, curtain fronts dashed magenta, the path as a thin line coloured
  by time, where SLAM believed it was dashed grey, start (o), end (square);
- each bump: x at the centre and the body's outline, red for the right bumper, blue for the
  left, black for both; each tick it reversed off a contact: a purple triangle;
- SLAM's map at the end, its occupied cells as faint violet dots: what the robot saw (glass is
  missing from it, curtains are a smear);
- each inner corner marked with its distance (cm) to the nearest swept floor.

The overview's table totals every run: the uncleaned area, the corners, time and bumps.
"""

import argparse
import glob
import json
import math
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402

CELL = 0.01
BODY_RADIUS = 0.175
# The footprint is drawn every this many dumped poses (about 10 s).
FOOTPRINT_EVERY = 110
# A bump by which bumper was pressed (older dumps do not say: "both").
BUMP_COLORS = {"left": "#0050ff", "right": "#e00000", "both": "#000000"}


def legend(fig):
    """What every mark means, once for the whole figure."""
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    line = lambda **kw: Line2D([], [], **kw)  # noqa: E731
    handles = [
        line(marker="o", color="g", ls="", ms=6, label="start"),
        line(marker="s", color="k", mfc="none", ls="", ms=6, label="end"),
        line(color="#3b8f8f", lw=2, label="true path (colour = time, dark → yellow)"),
        line(color="#555", ls="--", lw=1, label="where SLAM thought it was"),
        line(marker="o", color="#444", mfc="none", ls="", ms=10,
             label="body every ~10 s, with the main roller across it"),
        line(marker="o", color="#1f6fd0", mfc="none", ls="", ms=7, label="side brush, same moments"),
        line(marker="x", color=BUMP_COLORS["right"], ls="", ms=7,
             label="bump, right bumper (red circle = the body then)"),
        line(marker="x", color=BUMP_COLORS["left"], ls="", ms=7, label="bump, left bumper"),
        line(marker="x", color=BUMP_COLORS["both"], ls="", ms=7, label="bump, both bumpers"),
        line(marker="v", color="#c000c0", ls="", ms=6, label="reversing (each tick backing off)"),
        Patch(color="#b8e6b0", label="floor swept by the brushes"),
        Patch(color="#e0301e", label="floor left unswept along a wall"),
        Patch(color="#ff8c00", label="pocket: unswept floor the path closed round"),
        line(color="k", lw=2, label="wall (lidar sees it)"),
        line(color="c", lw=3, label="glass / overhang (lidar does not)"),
        line(color="m", ls="--", lw=2, label="curtain front"),
        line(marker=".", color="#7a5cff", ls="", ms=5, label="SLAM's map, occupied"),
        line(color="w", label="numbers: inner corner's distance to swept floor, cm"),
    ]
    fig.legend(handles=handles, loc="upper center", ncol=3, fontsize=8, frameon=False)


def cells(points, x0, y0, w, h):
    """The bench's cells (their centres) on this panel's grid."""
    grid = np.zeros((h, w), bool)
    for x, y in points:
        i, j = int((x - x0) / CELL), int((y - y0) / CELL)
        if 0 <= i < w and 0 <= j < h:
            grid[j, i] = True
    return grid


def footprint(ax, run, x, y, th):
    """The body, the main roller across the centre and the side brush's disc at a pose."""
    c, s = math.cos(th), math.sin(th)
    half, (fx, fy), r = run["main_brush_half"], run["side_brush"], run["side_brush_radius"]
    ax.add_patch(plt.Circle((x, y), BODY_RADIUS, fill=False, ec="#444", lw=0.4, zorder=4))
    ax.plot([x + half * s, x - half * s], [y - half * c, y + half * c], "-", color="#444",
            lw=1.2, zorder=4)
    ax.add_patch(plt.Circle((x + fx * c - fy * s, y + fx * s + fy * c), r, fill=False,
                            ec="#1f6fd0", lw=0.6, zorder=4))


def corners(edges):
    """Inner corners (left turns) of the edge chain, in order."""
    out = []
    for k, (a, b) in enumerate(edges):
        c, d = edges[(k + 1) % len(edges)]
        if math.hypot(c[0] - b[0], c[1] - b[1]) > 1e-3:
            continue
        u, v = (b[0] - a[0], b[1] - a[1]), (d[0] - c[0], d[1] - c[1])
        if u[0] * v[1] - u[1] * v[0] > 0:
            out.append(b)
    return out


def panel(ax, run):
    pts = [p for e in run["edges"] for p in e]
    x0, y0 = min(p[0] for p in pts) - 0.1, min(p[1] for p in pts) - 0.1
    x1, y1 = max(p[0] for p in pts) + 0.1, max(p[1] for p in pts) + 0.1
    w, h = int((x1 - x0) / CELL) + 1, int((y1 - y0) / CELL) + 1
    grid = cells(run["swept"], x0, y0, w, h)
    ax.imshow(grid, origin="lower", extent=(x0, x0 + w * CELL, y0, y0 + h * CELL),
              cmap=ListedColormap(["#f4f4f4", "#b8e6b0"]), interpolation="nearest", zorder=0)
    # The uncleaned strip along the walls: each sample a 1 cm wide bar `depth` deep.
    for x, y, nx, ny, d in run["misses"]:
        if d > 0.004:
            tx, ty = ny, -nx
            quad = [(x - tx * CELL / 2, y - ty * CELL / 2), (x + tx * CELL / 2, y + ty * CELL / 2),
                    (x + tx * CELL / 2 + nx * d, y + ty * CELL / 2 + ny * d),
                    (x - tx * CELL / 2 + nx * d, y - ty * CELL / 2 + ny * d)]
            ax.fill(*zip(*quad), color="#e0301e", lw=0, zorder=1)
    # Pockets, the unswept floor the path closed round.
    pockets = np.zeros((h, w, 4))
    pockets[cells(run["pockets"], x0, y0, w, h)] = (1.0, 0.55, 0.0, 1.0)
    ax.imshow(pockets, origin="lower", extent=(x0, x0 + w * CELL, y0, y0 + h * CELL),
              interpolation="nearest", zorder=1)
    for x, y, th in run["path"][::FOOTPRINT_EVERY]:
        footprint(ax, run, x, y, th)
    for (a, b) in run["walls"]:
        ax.plot([a[0], b[0]], [a[1], b[1]], "k-", lw=1.5, zorder=3)
    for (a, b) in run["glass"]:
        ax.plot([a[0], b[0]], [a[1], b[1]], "c-", lw=3, zorder=3)
    for (a, b) in run.get("curtains", []):
        ax.plot([a[0], b[0]], [a[1], b[1]], "m--", lw=2, zorder=3)
    if run["path"]:
        xs, ys = [p[0] for p in run["path"]], [p[1] for p in run["path"]]
        ax.scatter(xs, ys, c=range(len(xs)), cmap="viridis", s=0.3, zorder=4)
        ax.plot(xs[0], ys[0], "go", ms=5, zorder=5)
        ax.plot(xs[-1], ys[-1], "ks", ms=5, mfc="none", mew=1.2, zorder=5)
    if run.get("belief"):
        ax.plot(*list(zip(*run["belief"]))[:2], "--", color="#555", lw=0.5, zorder=4)
    if run.get("seen"):
        ax.scatter(*zip(*run["seen"]), s=1.5, c="#7a5cff", alpha=0.7, lw=0, zorder=3.5)
    if run.get("reversing"):
        ax.scatter(*zip(*run["reversing"]), s=6, c="#c000c0", marker="v", lw=0, zorder=5)
    for x, y, *side in run["bumps"]:
        color = BUMP_COLORS[side[0] if side else "both"]
        ax.add_patch(plt.Circle((x, y), BODY_RADIUS, fill=False, ec=color, lw=0.6, zorder=5))
        ax.plot(x, y, "x", color=color, ms=4, mew=1, zorder=5)
    for (cx, cy), miss in zip(corners(run["edges"]), run["corners"]):
        color = "#2a8a2a" if miss <= 0.05 else "#e0301e"
        ax.annotate(f"{miss * 100:.0f}", (cx, cy), fontsize=7, color=color, weight="bold",
                    xytext=(cx + (0.08 if cx < (x0 + x1) / 2 else -0.15),
                            cy + (0.06 if cy < (y0 + y1) / 2 else -0.12)), zorder=6)
    done = "" if run["completed"] else ", NO LAP"
    ax.set_title(f"{run['seconds']:.0f} s, {run['bump_count']} bumps{done}\n"
                 f"miss {run['missed_depth'] * 100:.1f} cm deep, "
                 f"pockets {run.get('pockets_m2', 0) * 1e4:.0f} cm², "
                 f"worst corner {max(run['corners'] or [0]) * 100:.0f} cm", fontsize=8)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.tick_params(labelsize=6)


def summary(ax, runs):
    ax.axis("off")
    rows = [(r["name"], r["seconds"], r["bump_count"], r["missed_m2"] * 1e4,
             r.get("pockets_m2", 0) * 1e4, sum(r["corners"]) * 100, max(r["corners"] or [0]) * 100,
             r["completed"]) for r in runs]
    lines = [f"{'scene':<18}{'s':>5}{'bumps':>6}{'uncl cm²':>9}{'pkt cm²':>8}{'Σcorner':>8}{'worst':>6}"]
    for name, s, b, a, p, cs, cw, done in rows:
        lines.append(f"{name:<18}{s:5.0f}{b:6d}{a:9.0f}{p:8.0f}{cs:8.0f}{cw:6.0f}"
                     f"{'' if done else ' NO LAP'}")
    lines.append("")
    lines.append(f"{'TOTAL':<18}{sum(r[1] for r in rows):5.0f}{sum(r[2] for r in rows):6d}"
                 f"{sum(r[3] for r in rows):9.0f}{sum(r[4] for r in rows):8.0f}"
                 f"{sum(r[5] for r in rows):8.0f}{max(r[6] for r in rows):6.0f}")
    ax.text(0, 1, "\n".join(lines), family="monospace", fontsize=8, va="top")


def seed_note(seed, runs):
    """What a column's seed changes, from the dumps."""
    bias = next((r["range_bias"] for r in runs if r["seed"] == seed and "range_bias" in r), None)
    if bias is None:
        return f"seed {seed}"
    lidar = ("lidar as calibrated" if abs(bias) < 1e-4 else
             f"lidar reads {abs(bias) * 100:.0f} cm {'long' if bias > 0 else 'short'}")
    return f"seed {seed}: {lidar}"


def overview(runs, seeds, out):
    """The legend, what the columns are, and the table of every run."""
    fig = plt.figure(figsize=(14, 2.6 + 0.16 * len(runs)))
    legend(fig)
    notes = ["Pages: one row per scene, one column per seed. Each seed draws its own noise "
             "(lidar, wheels, gyro) and shifts the lidar's range:"]
    notes += [f"  {seed_note(s, runs)}" for s in seeds]
    fig.text(0.03, 1 - 1.3 / fig.get_figheight(), "\n".join(notes), fontsize=9, va="top")
    ax = fig.add_axes((0.03, 0, 0.94, 1 - 2.3 / fig.get_figheight()))
    summary(ax, runs)
    fig.savefig(out, dpi=90)


def page(runs, scenes, seeds, out):
    """Panels for `scenes`, a row each, a column per seed."""
    cols = len(seeds)
    fig, axes = plt.subplots(len(scenes), cols, figsize=(4.6 * cols, 3.8 * len(scenes) + 0.4),
                             squeeze=False)
    by = {(r["scene"], r["seed"]): r for r in runs}
    for i, scene in enumerate(scenes):
        for j, seed in enumerate(seeds):
            ax = axes[i, j]
            if (scene, seed) in by:
                panel(ax, by[(scene, seed)])
            else:
                ax.axis("off")
        axes[i, 0].set_ylabel(scene, fontsize=12, weight="bold")
    fig.tight_layout(rect=(0, 0, 1, 1 - 0.4 / fig.get_figheight()))
    for j, seed in enumerate(seeds):
        box = axes[0, j].get_position()
        fig.text((box.x0 + box.x1) / 2, 1 - 0.15 / fig.get_figheight(), seed_note(seed, runs),
                 ha="center", va="top", fontsize=10, weight="bold")
    fig.savefig(out, dpi=90)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir", help="directory of <scene>_<seed>.json dumps")
    ap.add_argument("--out", required=True,
                    help="PNG: the overview; the pages go next to it as <out>_1.png, …")
    ap.add_argument("--rows", type=int, default=4, help="scenes per page")
    args = ap.parse_args()
    runs = []
    for p in sorted(glob.glob(os.path.join(args.dir, "*.json"))):
        run = json.load(open(p))
        scene, seed = os.path.basename(p)[:-5].rsplit("_", 1)
        run["scene"], run["seed"] = scene, run.get("seed", int(seed))
        run["name"] = f"{scene}_{run['seed']}"
        runs.append(run)
    scenes = list(dict.fromkeys(r["scene"] for r in runs))
    seeds = sorted({r["seed"] for r in runs})
    overview(runs, seeds, args.out)
    print("wrote", args.out)
    stem, ext = os.path.splitext(args.out)
    for k in range(math.ceil(len(scenes) / args.rows)):
        out = f"{stem}_{k + 1}{ext}"
        page(runs, scenes[k * args.rows:(k + 1) * args.rows], seeds, out)
        print("wrote", out)


if __name__ == "__main__":
    main()
