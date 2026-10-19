# dhruva-lab

Offline SLAM lab. Replays a recording through dhruva-slam's pipeline
(`engine::pipeline::SlamPipeline`, the same code the daemon runs) one scan at a
time and records what the engine believes after every step. The SLAM code stays
in `dhruva-slam`; this crate only drives it and looks at it.

```bash
cargo build --release
./target/release/dhruva-lab -c ../dhruva-slam/dhruva-v8.toml run.mcap \
    --out out/ --reference cartographer/traj.csv
```

Outputs in `--out`:

| File | What |
| --- | --- |
| `trace.mcap` | step-by-step trace with Foxglove schemas (below) |
| `map.png` | final map, 1 m grid, SLAM path (blue), odometry path (orange), loop closures (red), start (green), end (red) |
| `traj.csv` | `t_us,ms,x,y,theta,score,keyframes,points`, the columns of DhruvaSLAM's `timing.csv` |
| `metrics.json` | run summary, and with `--reference`: ATE and heading error after the best rigid alignment (same definitions as `tools/py/slam_eval.py`) |

There is no ground truth on a real floor. The reference is another SLAM on the
same recording, e.g. Cartographer from `docker/ref-slam` in the parent project.

## Watching the trace

Open `trace.mcap` in [Foxglove](https://foxglove.dev) or
[Lichtblick](https://github.com/lichtblick-suite/lichtblick) (open source), add a
3D panel with display frame `map` and press play:

| Topic | Schema | What |
| --- | --- | --- |
| `/tf` | FrameTransforms | REP-105: `odom → base_link` (raw odometry), `map → odom` (the SLAM correction) |
| `/scan` | PointCloud in `base_link` | the scan the engine matched. Set a decay time to see the history fade |
| `/belief` | SceneUpdate | pose uncertainty (2σ ellipse) at the SLAM pose |
| `/path` | SceneUpdate | trajectory so far: SLAM (blue), odometry (orange) |
| `/graph` | SceneUpdate | pose graph, only with loop closure enabled: sequential edges (grey), loop closures (red) |
| `/map` | Grid (RGBA) | the global occupancy map when it changed, at most every `--map-every` s of data time |
| `/step` | JSON | per-scan numbers for a Plot panel: match score, keyframes, closures, rotation per step |

## The corner-sweep bench

`sweep` runs the corner sweep closed loop, faster than real time, in nine small
scenes after what the real runs met (a box, an L, the dock, a curtain, legs by a
wall, the dog's bench, a chair, a round bed, a glass fin):

```bash
cargo run --release --bin sweep -- --seeds 2 --dump /tmp/wf
uv run --with numpy --with matplotlib ../../tools/py/wf_sim_plot.py /tmp/wf --out wf.png
```

(`tools/py` is in the project folder around this repo, not in it.)

The robot model, the wire, the decode and the SLAM are the real ones: SangamIO's
mock of the v8 (`sangamio/mock_bench.toml`, as measured) is stepped at its
110 Hz, its groups go through SangamIO's wire format and dhruva-slam's decode
into the SLAM the daemon runs, and the exploration session decides on the
simulated clock, as the exploration thread would. What the bench leaves out
(SLAM latency, TCP, the lidar's boot) is listed in `src/bin/sweep/run.rs`. Each
run is scored on the true path: the floor the brushes left unswept along the
walls, the corners, time, bumps, whether the lap closed. `--climb N`
hill-climbs `[exploration.wall_follow]` on that score.
