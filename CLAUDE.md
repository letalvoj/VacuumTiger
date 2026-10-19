# CLAUDE.md

This is the `v8-integration` fork of VacuumTiger. It runs on a viomi.vacuum.v8 (a CRL-200S).

- **Architecture, hard safety rules, docs map:** `../CLAUDE.md`, in the project folder that holds
  this repo. Read it first.
- **Current plan:** `../docs/ROADMAP.md`.

This file only covers working inside the repo.

## Do not follow upstream's deploy

Upstream's instructions copied binaries to `/usr/sbin`, renamed AuxCtrl and used an SSH password.
`sangamio/README.md` "Deployment" still shows them. On the v8 that would write flash. We never do
that:
- `../scripts/deploy.sh` copies builds to `/tmp/v8` (RAM).
- `../scripts/robot/run_web.sh` starts the stack and restores the vendor firmware after.

## Crates

Each crate is its own Cargo project (no workspace), Rust edition 2024.

| Crate | What | v8 / dev config |
| --- | --- | --- |
| `sangamio` | HAL daemon: GD32 + lidar, protobuf on TCP 5555, control lease, dead-man, bumper interlock; `--features mock` is the simulator | `viomi_v8.toml` / `mock.toml` |
| `dhruva-slam` | SLAM daemon + library: odometry, matching, loop closure, exploration (`wall_follow` corner sweep), offline `--replay` | `dhruva-v8.toml` / `dhruva-mock.toml` |
| `dwar` | web gateway: REST + WebSocket, Preact UI in `web/`, teleop, STOP, MCAP recorder | `../scripts/robot/dwar.toml` / `dwar.toml` |
| `dhruva-lab` | offline lab: replays recordings through dhruva-slam's pipeline; metrics, floor plan, rooms; `sweep`, the corner-sweep bench on the mock | — |
| `drishti`, `protocol-mitm` | PyQt viewer; serial MITM tool (reverse engineering) | — |
| `dhruva-nav`, `vastu-slam`, `slam-test` | upstream leftovers, not used (`dhruva/` holds only a proposal doc) | — |

Wire contracts:
- `sangamio/proto/sangamio.proto`
- `dhruva-slam/proto/dhruva.proto`
- GD32 details: `sangamio/src/devices/crl200s/{COMMANDS,SENSORSTATUS}.md`

## Build and test

Run these from the crate's directory:

```bash
export PATH="/opt/homebrew/opt/rustup/bin:$HOME/.cargo/bin:$PATH"
cargo test --release                       # sangamio: also --features mock
cargo build --release --features mock --target-dir target/mock   # sangamio simulator → target/mock/release/sangamio
cargo +nightly zigbuild --release --target armv7-unknown-linux-musleabihf   # robot builds
cd dwar/web && npm run build               # before building dwar
```

Format only what you touch:
- `rustfmt --edition 2024 --config skip_children=true <files>`.
- dwar is not rustfmt-clean, so don't reformat it.

## Conventions

- Safety first in SangamIO:
  - The GD32 holds the last velocity forever and needs a heartbeat every 20–50 ms
    (`sangamio/README.md`).
  - Only one process can own `/dev/ttyS3` (GD32) and `/dev/ttyS1` (lidar). The robot scripts stop
    the vendor's AuxCtrl first.
  - Never weaken the heartbeat, the dead-man or the refusal of the dangerous MCU commands.
  - No `unwrap()` on the robot's runtime paths.
- Algorithms are tested headless and deterministically. Call strategies and the mock's physics,
  lidar and bumpers as a library. Don't run daemons in real time.
- Small, single-purpose commits, each reviewed first (see `../CLAUDE.md`). Never push without the
  owner's OK.
