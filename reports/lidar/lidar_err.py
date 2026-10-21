"""The Delta-2D's range noise and bias in the real recordings.

    uv run -q --with numpy --with scipy --with matplotlib --with protobuf --with mcap \
        --with zstandard reports/lidar/lidar_err.py --root ..

`--root` is the project around VacuumTiger: its `captures/catalog/` (the labelled runs, with
SLAM's final trajectory), the robot's own SangamIO recordings of them (`data/`,
`captures/sweep*/`) and `tools/py/` (the recording readers). Prints the numbers and writes
`lidar_err.png` to `--out`.

A  static noise: per-beam std of range while the wheels stand still, against range.
B  bumper presses: the gap the lidar reports between the shell and the nearest thing on the
   pressed side, right after the press (0 = no bias).
C  range-dependent bias: with SLAM's final poses, each point's offset (along its beam) from the
   local surface built from other scans, against how much farther it was seen than they were.
"""

import argparse
import glob
import os
import sys

import numpy as np

LASER_X = -0.105  # dhruva-v8.toml / viomi_v8.toml: the lidar 10.5 cm behind the centre
SHELL_R = 0.175
OPTICAL = 0.025  # dhruva-v8.toml: added to every range
# Aborted before the lidar came up.
SKIP = ("sweep4_1783a9",)


def raw(root, run):
    """The robot's own SangamIO recording of a catalog run."""
    hits = glob.glob(f"{root}/data/{run}.mcap") + glob.glob(
        f"{root}/captures/sweep*/**/{run}.mcap", recursive=True)
    return hits[0] if hits else None


def load(path):
    from sangam_client import read_frames, value
    from sgio2bag import unpack

    status, scans, kin = [], [], {}
    for _, m in read_frames(path):
        g = m.sensor_group
        v = g.values
        if g.group_id == "sensor_status" and "wheel_left" in v:
            status.append((g.timestamp_us, value(v["wheel_left"]), value(v["wheel_right"]),
                           bool(value(v["bumper_left"])) if "bumper_left" in v else False,
                           bool(value(v["bumper_right"])) if "bumper_right" in v else False))
        elif g.group_id == "lidar" and "scan_packed" in v:
            scans.append((g.timestamp_us, np.array(unpack(value(v["scan_packed"]))).reshape(-1, 3)))
        elif g.group_id == "kinematics" and not kin:
            kin = {k: value(x) for k, x in v.items()}
    return np.array(status, np.float64), scans, kin


def still_mask(status, kin, hold_s=2.0):
    """Per status sample: the wheels have not moved for hold_s either side."""
    wrap = 1 << int(kin["wheel_tick_bits"])
    d = np.abs((np.diff(status[:, 1:3], axis=0) + wrap / 2) % wrap - wrap / 2).sum(axis=1)
    t = status[:, 0] / 1e6
    move_t = t[1:][d > 0]
    if len(move_t) == 0:
        return np.ones(len(t), bool)
    i = np.searchsorted(move_t, t)
    before = np.where(i > 0, t - move_t[np.maximum(i - 1, 0)], np.inf)
    after = np.where(i < len(move_t), move_t[np.minimum(i, len(move_t) - 1)] - t, np.inf)
    return (before > hold_s) & (after > hold_s)


def static_noise(status, scans, kin):
    """(mean range, std) per beam over each window of 15+ still scans."""
    still = still_mask(status, kin)
    ts = status[:, 0]
    windows, cur = [], []
    for t, sc in scans:
        if still[min(np.searchsorted(ts, t), len(ts) - 1)]:
            cur.append(sc)
        elif cur:
            windows.append(cur)
            cur = []
    if cur:
        windows.append(cur)
    rows = []
    for w in windows:
        if len(w) < 15:
            continue
        beams = {}
        for sc in w:
            for a, r, _ in sc:
                if 0.15 <= r <= 8:
                    beams.setdefault(int(round(np.degrees(a))) % 360, []).append(r)
        for rs in beams.values():
            rs = np.array(rs)
            rs = rs[np.abs(rs - np.median(rs)) < 0.05]  # drop mixed pixels and dropouts
            if len(rs) >= 10:
                rows.append((rs.mean(), rs.std()))
    return np.array(rows)


def bump_gaps(status, scans):
    """Per press: the gap (m) between the shell and the nearest return on the pressed side,
    and the side (L, R, B). Not corrected for the bumper's travel, nor for the robot moving
    between the press and the scan."""
    out = []
    press = (status[1:, 3] > status[:-1, 3]) | (status[1:, 4] > status[:-1, 4])
    scan_t = np.array([t for t, _ in scans])
    for k in np.nonzero(press)[0] + 1:
        t = status[k, 0]
        left, right = status[k, 3] > 0, status[k, 4] > 0
        j = np.searchsorted(scan_t, t + 0.12e6)  # a scan started after the stop
        if j >= len(scans) or scan_t[j] - t > 0.5e6:
            continue
        sc = scans[j][1]
        sc = sc[(sc[:, 1] >= 0.15) & (sc[:, 1] < 0.6)]
        r = sc[:, 1] + OPTICAL
        x, y = LASER_X + r * np.cos(sc[:, 0]), r * np.sin(sc[:, 0])
        bearing = np.degrees(np.arctan2(y, x))
        if left and right:
            sel = np.abs(bearing) < 45
        elif left:
            sel = (bearing > 0) & (bearing < 90)
        else:
            sel = (bearing < 0) & (bearing > -90)
        if sel.sum() < 3:
            continue
        gap = np.hypot(x[sel], y[sel]) - SHELL_R
        out.append((np.sort(gap)[1], "LR"[int(not left)] if left != right else "B"))  # 2nd nearest
    return out


def range_bias(status, scans, kin, traj_path, nq=3000, rad=0.06, min_dt=5.0):
    """Per sampled point of a still scan: (its range, the mean range of the other scans'
    points round it, its offset along its beam from their line)."""
    from scipy.spatial import cKDTree

    tr = np.genfromtxt(traj_path, delimiter=",", names=True)
    tt = tr["t_us"]
    P, R, U, T = [], [], [], []
    still = still_mask(status, kin, hold_s=0.3)
    for t, sc in scans:
        if not still[min(np.searchsorted(status[:, 0], t), len(status) - 1)]:
            continue
        i = np.searchsorted(tt, t)
        if i >= len(tt) or abs(tt[i] - t) > 20000:
            continue
        x0, y0, th = tr["x"][i], tr["y"][i], tr["theta"][i]
        sc = sc[(sc[:, 1] >= 0.15) & (sc[:, 1] <= 6)]
        a = sc[:, 0] + th
        ox, oy = x0 + LASER_X * np.cos(th), y0 + LASER_X * np.sin(th)
        u = np.stack([np.cos(a), np.sin(a)], 1)
        P.append(np.stack([ox, oy], 0) + sc[:, 1:2] * u)
        R.append(sc[:, 1])
        U.append(u)
        T.append(np.full(len(sc), t / 1e6))
    if not P:
        return np.empty((0, 3))
    P, R, U, T = (np.concatenate(v) for v in (P, R, U, T))
    tree = cKDTree(P)
    q = np.random.default_rng(0).choice(len(P), min(nq, len(P)), replace=False)
    res = []
    for k, nb in zip(q, tree.query_ball_point(P[q], rad)):
        nb = np.array(nb)
        nb = nb[np.abs(T[nb] - T[k]) > min_dt]
        if len(nb) < 15:
            continue
        c = P[nb].mean(0)
        w, v = np.linalg.eigh(np.cov((P[nb] - c).T))
        if w[0] > 0.1 * w[1] or np.sqrt(w[0]) > 0.012:  # not a thin line
            continue
        n = v[:, 0]
        cos = n @ U[k]
        if abs(cos) < 0.5:
            continue
        # Positive: the point is farther along its beam than the surface.
        res.append((R[k], R[nb].mean(), (n @ (P[k] - c)) / cos))
    return np.array(res)


def plot_static(ax, st):
    edges = np.array([0.15, 0.3, 0.5, 0.75, 1, 1.5, 2, 3, 4, 6, 8])
    rows = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        s = st[(st[:, 0] >= lo) & (st[:, 0] < hi), 1]
        if len(s) > 5:
            rows.append(((lo + hi) / 2, np.median(s), np.percentile(s, 90), len(s)))
    rows = np.array(rows)
    print("static std by range: mid, median std mm, p90 mm, beams")
    for r in rows:
        print(f"  {r[0]:.2f} m  {r[1] * 1000:.2f}  {r[2] * 1000:.2f}  {int(r[3])}")
    ax.scatter(st[:, 0], st[:, 1] * 1000, s=1, alpha=0.15, c="gray")
    ax.plot(rows[:, 0], rows[:, 1] * 1000, "o-", label="median")
    ax.plot(rows[:, 0], rows[:, 2] * 1000, "s--", label="p90")
    rr = np.linspace(0.15, 6, 50)
    ax.plot(rr, (0.0005 + 0.003 * rr) * 1000, "r:", label="mock: 0.5 mm + 3 mm/m")
    ax.set(xlabel="range (m)", ylabel="std over still scans (mm)", ylim=(0, 15),
           title="A. Random noise, robot standing still (per beam)")
    ax.legend()


def plot_gaps(ax, gaps):
    g = np.array([x for _, x, _ in gaps]) * 100
    near = g[g < 5.5]
    print(f"bump gaps: n={len(g)} median {np.median(g):.2f} cm, "
          f"IQR {np.percentile(g, 25):.2f}..{np.percentile(g, 75):.2f}")
    print(f"  on what the lidar sees (< 5.5 cm): n={len(near)} median {np.median(near):.2f} "
          f"mean {near.mean():.2f} std {near.std():.2f} cm")
    by = {}
    for name, x, _ in gaps:
        if x * 100 < 5.5:
            by.setdefault(name, []).append(x * 100)
    for name, v in by.items():
        print(f"  {name}: n={len(v)} median {np.median(v):.2f} cm")
    bins = np.arange(-3, 8, 0.25)
    off = OPTICAL * 100
    ax.hist(g - off, bins=bins, alpha=0.6, label=f"raw ranges: median {np.median(g) - off:+.2f} cm")
    ax.hist(g, bins=bins, alpha=0.6,
            label=f"+2.5 cm optical offset (dhruva-v8.toml): median {np.median(g):+.2f} cm")
    ax.axvline(0, color="k")
    ax.legend(fontsize=8)
    ax.set(xlabel="lidar's gap from shell to nearest return on the pressed side (cm)",
           ylabel="presses", title="B. At a bumper press (0 = no bias)")


def plot_range_bias(ax, rb):
    dr = rb[:, 0] - rb[:, 1]
    bins = np.arange(-3, 3.01, 0.5)
    mids, meds, q1, q3 = [], [], [], []
    for lo, hi in zip(bins[:-1], bins[1:]):
        e = rb[(dr >= lo) & (dr < hi), 2]
        if len(e) > 50:
            mids.append((lo + hi) / 2)
            meds.append(np.median(e) * 1000)
            q1.append(np.percentile(e, 25) * 1000)
            q3.append(np.percentile(e, 75) * 1000)
    k, _ = np.polyfit(dr, rb[:, 2], 1)
    print(f"range bias slope: {k * 1000:.2f} mm per m farther (n={len(rb)}), "
          f"resid std {rb[:, 2].std() * 1000:.1f} mm")
    for m, e, a, c in zip(mids, meds, q1, q3):
        print(f"  dr {m:+.2f} m: median {e:+.1f} mm (IQR {a:+.1f}..{c:+.1f})")
    ax.plot(mids, meds, "o-", label="median")
    ax.fill_between(mids, q1, q3, alpha=0.3, label="IQR")
    ax.axhline(0, color="k", lw=0.5)
    ax.set(xlabel="seen this much farther than the other scans of the same spot (m)",
           ylabel="offset along the beam from the surface (mm)",
           title="C. Range-dependent error (SLAM poses, flat walls)")
    ax.legend()


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--root", default=os.path.join(here, "../../.."),
                    help="the project around VacuumTiger (captures/, data/, tools/py/)")
    ap.add_argument("--out", default=here, help="directory for lidar_err.png")
    args = ap.parse_args()
    sys.path.insert(0, os.path.join(args.root, "tools/py"))
    cat = os.path.join(args.root, "captures/catalog")
    runs = sorted(d for d in os.listdir(cat)
                  if os.path.isdir(f"{cat}/{d}") and not any(s in d for s in SKIP))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    static, gaps, rbs = [], [], []
    for run in runs:
        path = raw(args.root, run)
        if path is None:
            print(run, "no SangamIO recording, skipped")
            continue
        status, scans, kin = load(path)
        if not len(status) or not scans:
            continue
        name = run.split("_")[1]
        st = static_noise(status, scans, kin)
        g = bump_gaps(status, scans)
        traj = f"{cat}/{run}/traj_final.csv"
        rb = range_bias(status, scans, kin, traj) if os.path.exists(traj) else np.empty((0, 3))
        slope = (f", slope {np.polyfit(rb[:, 0] - rb[:, 1], rb[:, 2], 1)[0] * 1000:+.1f} mm/m"
                 if len(rb) > 100 else "")
        print(f"{name}: {len(scans)} scans, {len(st)} still beams, {len(g)} presses, "
              f"{len(rb)} surface points{slope}", flush=True)
        if len(st):
            static.append(st)
        gaps += [(name, x, side) for x, side in g]
        if len(rb):
            rbs.append(rb)

    fig, ax = plt.subplots(1, 3, figsize=(18, 5.5))
    plot_static(ax[0], np.concatenate(static))
    plot_gaps(ax[1], gaps)
    plot_range_bias(ax[2], np.concatenate(rbs))
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "lidar_err.png"), dpi=100)


if __name__ == "__main__":
    main()
