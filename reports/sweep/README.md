# Edge-pass reports

One report per edge-following strategy on the sweep bench (`dhruva-lab`'s `sweep`,
[SWEEP_BENCH.md](../../dhruva-lab/SWEEP_BENCH.md)): what it is, its numbers, its renders and
where it falls short.

| Strategy | What | rooms | primitives | fin_thin |
| --- | --- | ---: | ---: | ---: |
| [wall_follow](wall_follow.md) | ours: one keep-the-edge-beside-the-body rule | 1563.5 | 821.8 | 48.6 |
| [stock_follow](stock_follow.md) | the stock firmware's edge pass, ported | 874.1 | 684.7 | 44.5 |
| [stock_follow_fix](stock_follow_fix.md) | the port with fixes for overhangs and range bias | **725.0** | **556.4** | **38.9** |

Cost summed over 8 seeds and every scene, lower is better (the bench's definition, §4). All
three at aa6f172.

## How these were made

From `dhruva-lab/`, per strategy:

```bash
cargo build --release --bin sweep
./target/release/sweep --kit rooms --seeds 8 --strategy stock_follow_fix > rooms.txt
python3 tools/sweep_table.py rooms.txt                  # the per-scene table in the reports
./target/release/sweep --kit rooms --seeds 3 --strategy stock_follow_fix --dump /tmp/wf
uv run -q --with numpy --with matplotlib tools/wf_sim_plot.py /tmp/wf --out rooms.png
```

- `results/<strategy>_<kit>.txt` is the 8-seed printout the tables come from.
- `img/<strategy>/` holds the renders: `<kit>.png` (legend, seeds, every run) and
  `<kit>_<n>.png` (pages, a row per scene, a column per seed) for seeds 1-3, and
  `fin_thin_a.png` / `fin_thin_b.png` for fin_thin's seeds 1-4 and 5-8.
- The PNGs are quantised to 256 colours to keep the repo small.

The seeds shift the lidar's range by 0, −1 or +1 cm in turn (seed % 3: 1 → 0, 2 → −1, 3 → +1):
"reads short" puts walls nearer than they are, so the robot keeps farther off and misses
floor; "reads long" brings it into them, so it bumps. How large the real lidar's bias is:
[../lidar](../lidar/README.md).

When a strategy changes, re-run its report and add a line to its History.
