# The real lidar's errors

How wrong is the v8's Delta-2D, and is the sweep bench's ±1 cm range bias (per seed) a fair
model of it? `lidar_err.py` reads every labelled run (the 8 in the project's
`captures/catalog`). It uses the robot's own SangamIO recordings and SLAM's final trajectory
of each, and prints `lidar_err.txt` and plots `lidar_err.png`. The recordings and their
readers (`tools/py/`) live outside this repo, so the committed txt and png are the record.

![lidar errors](lidar_err.png)

## A. Random noise (robot standing still)

Per-beam std over windows of 15+ still scans:

| range | 0.2 m | 0.4 m | 0.6 m | 0.9 m | 1.25 m | 2.5 m | 3.5 m | 5 m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| median std, mm | 0.3 | 0.4 | 0.5 | 0.8 | 1.4 | 3.6 | 4.9 | 3.0 |
| p90, mm | 1.4 | 2.4 | 3.3 | 4.5 | 4.0 | 9.9 | 15 | 17 |

Small at wall-following range: under a millimetre within 1 m. The mock's 0.5 mm + 3 mm/m
is about right up close and generous beyond.

## B. Bias at a bumper press

At each press (71 in 7 runs), take the first scan stamped after it, within 0.3 s. On the
pressed side, the nearest raw return gives the gap from the resting shell (r 17.5 cm, lidar
10.5 cm behind the centre). The second-nearest return is used, against stray points. After
a press the robot turns on the spot and backs off at a few cm/s. Turning does not change a
gap to a round shell centred on the pivot, so each beam is moved back by the wheels'
forward distance since the press. The scan is not deskewed. On the 60 presses where the
lidar sees what was hit (a raw gap under 3 cm):

- **median +0.8 cm, std 0.8 cm on raw ranges**: the wall looks 0.8 cm beyond the resting
  shell. All 71 presses give +0.9 cm, since the cut is on the outcome;
- **+3.3 cm as dhruva-slam sees it**, since `dhruva-v8.toml` adds `optical_offset` 0.025 to
  every range;
- the same left and right (+0.82 / +0.82 cm), and +0.3 to +0.9 cm per run;
- front contacts read more than side ones: +0.94 cm within 30° (n=33), +0.3-0.4 cm beyond
  (n=27). A lidar 11 cm behind the centre instead of 10.5 would do that (`dhruva-onboard.toml`
  has 11), and so would more bumper travel head-on or a shell that is not quite round.
  These bands are small.

One-off checks in a scratch variant, not in the script:
- The beam timing matters by ±0.2 cm. Taking the stamp as the start, middle or end of the
  revolution gives +0.6 to +0.8 cm from the scan after the press, and +0.4 to +1.3 cm from
  the scan before it.
- The previous version of this script gave +0.36 cm on the same 60-press cut. It used a
  scan ≥ 0.12 s after the press and no motion compensation, while the robot was already
  backing off.

Still not accounted for:
- **Bumper travel.** The shell moves in a few mm before the switch closes, so at a press the
  wall is that much inside the resting shell. The bias is the gap above plus the travel: with
  5 mm of travel, raw ranges read about 1.3 cm long. Leaving it out would make a follower
  calibrated on presses rub.
- **What was hit.** A baseboard or a chair rail the lidar sees behind the contact reads long.

Also tried once, in scratch: fitting the offset from how a straight wall bows (a radial
error δ bends it by δ(1−cos θ)). The recordings have too few still scans beside long
straight walls; most still time is at the dock. It needs a capture made for it.

## C. Range-dependent bias

Each point of a still scan is placed with SLAM's poses and compared with the line through
other scans' points of the same spot (seen ≥ 5 s apart). Its offset along its beam is taken
against how much farther it was seen. Only still scans are used: moving ones smear walls
by motion skew and pose error.

- The binned medians within ±0.5 m are −0.2 and +0.6 mm (IQR about ±2 mm), with no visible
  trend.
- A straight-line fit does not agree. The pooled slope is −14 mm/m, and the per-run slopes
  run from −22 to +218 mm/m with both signs. These fits are driven by outliers (std 6 mm),
  not by the bulk.

So this method cannot see a range-dependent error, if there is one; any such error is small
next to B's offset. Only 5 runs have still scans with a final trajectory, and the spread in
range per spot is small (most within ±0.5 m).

## What it means

Measured:
- The error near the shell is a constant offset: the same in every run and on both sides.
  Within 1 m the noise is a millimetre.
- On raw ranges, the wall at a press looks 0.8 cm beyond the resting shell. In dhruva-slam,
  which adds 2.5 cm, it looks 3.3 cm beyond.

Assumed:
- The shell's radius is 17.5 cm and the lidar sits 10.5 cm behind the centre. All the presses
  are at 0.2-0.3 m, so they cannot tell a range offset from an error in these. A 1 cm wrong
  radius looks the same as a 1 cm range offset.
- The bumper's travel, a few mm, is not measured. It adds to the offset.

So, if the geometry is right:
- Raw ranges read 1-1.3 cm long, and `optical_offset` 0.025 makes it worse instead of
  correcting it. Walls would be 3.3-3.8 cm nearer than dhruva-slam believes.
- `optical_offset` is CRL-200S reference geometry, marked unverified on the v8 in both
  configs. It is not where the lidar sits; that is `mounting_x`. It is a correction added to
  every range. These presses alone do not give its value.
- Why it seems to work anyway:
  - SLAM barely cares. A constant radial error keeps scans consistent with each other; it
    moves walls out, so a room looks up to ~7 cm wider, and bends them very slightly.
  - The sweeps lean on the bumper: 6-17 presses a sweep run.
  - stock_follow undoes the offset and works on raw ranges.
- The bench's ±1 cm per seed is the right shape, perhaps around the wrong centre.

To check, a tape test with the robot still and the lidar on:
- a flat board touching the front, then at 5 and 10 cm, then the same at the side;
- the front gives `mounting_x` + radius + offset, and the side gives radius + offset;
- with the radius taped, that pins the offset and `mounting_x`, at more than one range;
- pushing the shell in by hand until it clicks gives the travel.

## Run it

```bash
uv run -q --with numpy --with scipy --with matplotlib --with protobuf --with mcap \
    --with zstandard reports/lidar/lidar_err.py --root .. > reports/lidar/lidar_err.txt
```

from `VacuumTiger/`; `--root` is the project around it (`captures/`, `data/`, `tools/py/`).

## History

- a6a0ead: first version; a press read +2.9 cm with the offset (+0.36 cm raw), no motion
  compensation.
- b8a544a: the scan right after the press, moved back by the wheels' distance; raw
  ranges; per side, bearing and run.
