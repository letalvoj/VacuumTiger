# dwar

The web gateway of VacuumTiger ("dwar", Sanskrit for gate). A separate process,
a client of SangamIO (hardware, TCP 5555) and DhruvaSLAM (SLAM, TCP 5557), that
serves a REST + WebSocket API and the web UI embedded in the binary.

```
browser / phone ──HTTP + one WebSocket──► dwar :8080 ──► sangamio :5555 (lease holder for teleop)
                                                    └──► dhruva-slam :5557 (pose, map, mapping commands)
```

It never touches hardware itself: a slow phone or a crash here can't stall the
HAL, and the UI keeps working with SLAM down. Wheels and cleaning actuators are
off unless enabled in `[control]`; the lidar and STOP always work.

## Build and run

```bash
cd dwar/web && npm ci && npm run build      # UI -> web/dist (embedded by the Rust build)
cd .. && cargo build --release              # host
cargo +nightly zigbuild --release --target armv7-unknown-linux-musleabihf   # robot (1.4 MB)
./target/release/dwar dwar.toml             # every key optional; see dwar.toml
```

UI development with hot reload against a running dwar: `DWAR=http://robot:8080 npm run dev`.

On the robot (viomi v8): `scripts/deploy.sh`, then `/tmp/v8/run_web.sh <seconds>`.
That script runs SangamIO + DhruvaSLAM + dwar instead of the vendor stack and
restores the vendor firmware afterwards.

## API (`/api/v1`)

| Endpoint | |
| --- | --- |
| `GET /robot`, `/status`, `/state`, `/capabilities` | identity, health, sensor snapshot, feature flags the UI renders from |
| `POST /stop` | zero drive, cleaning actuators off, revoke teleop (any client; 503 if SangamIO is offline) |
| `POST /lease` `{force}` / `DELETE /lease` | take / release SangamIO's control lease |
| `POST /actuators/{lidar,vacuum,main_brush,side_brush,water_pump,led}` `{enabled, speed?, state?}` | |
| `POST /slam/{start_mapping,stop_mapping,clear_map}`, `GET /maps`, `GET /maps/current.pgm`, `POST /maps/{id}/enable`, `DELETE /maps/{id}` | |
| `GET /maps/{id}/segments` | reserved for rooms (501) |
| `GET /recordings`, `POST /recordings/{start,stop}`, `GET/DELETE /recordings/{name}` | SangamIO stream as MCAP (`<UTC start>_<tag>_<random>.mcap`, protobuf with embedded schema: Foxglove, `mcap` CLI/Python) |
| `GET /ws` | the WebSocket (below) |

Replies: `{"ok":true,…}` when dwar did the thing itself (recordings, STOP),
`{"queued":true,…}` when a command went to SangamIO/DhruvaSLAM, which don't
acknowledge (DhruvaSLAM's response arrives later as a WebSocket `event`).
Errors are `{"error":"…"}` with a 4xx/5xx status.

`[http] token` enables `Authorization: Bearer <token>` (or `?token=` for the WebSocket and download links). State-changing requests and the WebSocket upgrade are refused when a browser's `Origin` names another site. `/lease` needs `allow_drive` or `allow_actuators`.

## WebSocket

One per tab. Client sends `{"type":"subscribe","topics":{"sensors":10,"scan":6,"pose":10,"map":1,"status":1,"events":1}}`
(Hz caps). Every topic is latest-wins per client: a phone on bad Wi-Fi gets
fresh data at a lower rate, never a backlog.

- Text (JSON): `hello`, `status`, `sensors`, `pose`, `map_meta`, `event`, `teleop`.
- Binary, little-endian, first byte = kind:
  - `1` scan: `u8 1 | 3×0 | u32 seq | u64 t_us | f32 x,y,theta (pose at arrival, NaN without SLAM) | u16 n | n × (u16 range·4000 m, u8 quality)`, bin i at i·2π/n CCW from forward (lidar frame; add `robot.lidar_offset_*`).
  - `2` map tile: `u8 2 | 0 | u16 tx,ty,w,h | 0 | u32 geometry | cells[w·h]`, 64×64 tiles, rows from the grid origin, 0 free / 100 occupied / 255 unknown. A client gets all tiles after a `map_meta` (geometry change), then only changed tiles.
- Teleop: `{"type":"teleop","take":true|false}` (reply `{"type":"teleop","holder":bool}`), then `{"type":"drive","linear":m/s,"angular":rad/s}` at 10–20 Hz. The rules live in `src/control.rs`: dwar zeroes the drive after `deadman_ms` of silence (its own timer task), on release and when the socket closes or stalls; SangamIO's heartbeat dead-man stays below. `{"type":"stop"}` from anyone revokes teleop: the holder must take it again.

## Later

Map/clean pages (rooms from map segmentation), a `valetudo` router module
(Valetudo's REST API so its UI and Home Assistant work against our stack), MQTT.
See `docs/WEB_PLAN.md` in the parent project.
