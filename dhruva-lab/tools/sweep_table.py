"""The sweep bench's printout as a Markdown table: each scene's mean over its seeds.

    ./target/release/sweep --kit rooms --seeds 8 > rooms.txt          # from dhruva-lab/
    python3 tools/sweep_table.py rooms.txt
"""

import sys
from collections import defaultdict

# The printout's columns after the scene#seed.
COLUMNS = ["lap", "s", "bumps", "miss cm", "corner", "pkt cm²", "loiter", "cost", "lap%",
           "m/min", "stray%"]
SHOWN = ["cost", "laps", "s", "bumps", "miss cm", "corner", "lap%", "m/min"]


def rows(path):
    for line in open(path):
        parts = line.split()
        if len(parts) < len(COLUMNS) + 1 or "#" not in parts[0]:
            continue
        scene = parts[0].split("#")[0]
        values = dict(zip(COLUMNS, parts[1:len(COLUMNS) + 1]))
        yield scene, values


def main(path):
    by = defaultdict(list)
    for scene, values in rows(path):
        by[scene].append(values)
    print("| scene | " + " | ".join(SHOWN) + " |")
    print("| --- | " + " | ".join("---:" for _ in SHOWN) + " |")
    total = 0.0
    for scene, runs in sorted(by.items()):
        mean = lambda k: sum(float(r[k]) for r in runs) / len(runs)
        laps = sum(r["lap"] == "yes" for r in runs)
        cost = sum(float(r["cost"]) for r in runs)
        total += cost
        cells = [f"{cost:.1f}", f"{laps}/{len(runs)}", f"{mean('s'):.0f}", f"{mean('bumps'):.1f}",
                 f"{mean('miss cm'):.2f}", f"{mean('corner'):.1f}", f"{mean('lap%'):.0f}",
                 f"{mean('m/min'):.2f}"]
        print(f"| {scene} | " + " | ".join(cells) + " |")
    print(f"| **total** | **{total:.1f}** |" + " |" * (len(SHOWN) - 1))


if __name__ == "__main__":
    main(sys.argv[1])
