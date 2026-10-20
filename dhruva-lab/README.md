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

`sweep` runs the corner sweep closed loop, faster than real time, in small
scenes after what the real runs met: a rooms kit (a box, an L, the dock, a
curtain, legs by a wall, the dog's bench, a chair, a round bed, a glass fin) and
a primitives kit, one obstacle each. From this folder:

```bash
cargo build --release --bin sweep
./target/release/sweep --kit rooms --seeds 3 --dump /tmp/wf      # table per scene and seed + total cost
uv run -q --with numpy --with matplotlib tools/wf_sim_plot.py /tmp/wf --out /tmp/wf.png --cols 3
```

The scenes, the metrics, the flags (one scene, `--set`, `--climb`), the
sim-to-real gaps, the lab notes and the reference render of the current code
are in [SWEEP_BENCH.md](SWEEP_BENCH.md).

## The simulated robot in the web UI

To watch the simulated v8 live instead, run the daemons on SangamIO's mock, each
in its own terminal from the repo root (ports 5555, 5557, 8080):

```bash
cd sangamio && cargo build --release --features mock --target-dir target/mock && ./target/mock/release/sangamio mock.toml
cd dhruva-slam && cargo build --release && ./target/release/dhruva-slam -c dhruva-mock.toml
cd dwar/web && npm ci && npm run build && cd .. && cargo build --release && ./target/release/dwar dwar.toml
```

Then open http://localhost:8080/. `mock.toml` loads its map from `sangamio/maps/`;
driving from the UI needs `allow_drive = true` in `dwar.toml`.
