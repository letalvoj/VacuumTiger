// Debug page panels: upstream health, sensors with live flips, hardware
// controls, SLAM, recordings, teleop, events.

import type { ComponentChildren } from "preact";
import { useEffect, useRef, useState, useErrorBoundary } from "preact/hooks";
import { api, apiUrl, type Json, type Link } from "./link";

/** A card. */
export function Panel(props: { title: string; wide?: boolean; children: ComponentChildren }) {
  return (
    <section class={`card ${props.wide ? "wide" : ""}`}>
      <h2>{props.title}</h2>
      {props.children}
    </section>
  );
}

/** Wrap a panel at its call site: a render error shows here instead of
 *  blanking the page (the boundary only catches errors of its children). */
export function Boundary(props: { title: string; wide?: boolean; children: ComponentChildren }) {
  const [error, reset] = useErrorBoundary();
  if (!error) return <>{props.children}</>;
  return (
    <Panel title={props.title} wide={props.wide}>
      <p class="error">
        {String(error)} <button class="small" onClick={reset}>retry</button>
      </p>
    </Panel>
  );
}

/** Run API actions with an error line. Each action has its own busy key, so
 *  a slow one (lidar spin-up) never disables another (an Off button). */
function useAction() {
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState<ReadonlySet<string>>(new Set());
  const inFlight = useRef(new Set<string>());
  const run = async (key: string, f: () => Promise<unknown>) => {
    if (inFlight.current.has(key)) return;
    inFlight.current.add(key);
    setBusy(new Set(inFlight.current));
    setError(null);
    try {
      await f();
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      inFlight.current.delete(key);
      setBusy(new Set(inFlight.current));
    }
  };
  return { error, busy: (key: string) => busy.has(key), run };
}
const num = (v: unknown, digits = 1) => (typeof v === "number" ? v.toFixed(digits) : "–");

function fmt(v: unknown): string {
  if (typeof v === "number") return Number.isInteger(v) ? String(v) : v.toFixed(3);
  if (typeof v === "boolean") return v ? "1" : "0";
  if (Array.isArray(v)) return v.map(fmt).join(", ");
  if (v && typeof v === "object") return JSON.stringify(v);
  return String(v ?? "–");
}

export function Health({ status, online }: { status: Json | null; online: boolean }) {
  const s = status?.sangamio;
  const d = status?.dhruva;
  const dot = (ok: unknown) => <span class={`dot ${ok ? "ok" : "bad"}`} />;
  return (
    <Panel title="Links">
      <div class="kv">
        <span>{dot(online)} dwar</span>
        <span>{online ? `up ${status?.uptime_s ?? 0} s` : "reconnecting…"}</span>
        <span>{dot(s?.connected)} SangamIO</span>
        <span>{s?.connected ? `${num(s.sensor_hz, 0)} Hz sensors · ${num(s.scan_hz)} Hz lidar · ${s.scan_points ?? 0} pts` : "offline"}</span>
        <span>{dot(d?.connected)} DhruvaSLAM</span>
        <span>{!d?.enabled ? "disabled" : d.connected ? (d.robot_status?.state ?? "connected") : "offline"}</span>
      </div>
    </Panel>
  );
}

/** Sensor values. Discrete values (bits, counters) flash when they change. */
export function Sensors({ values }: { values: Json }) {
  const prev = useRef<Json>({});
  const changed = useRef<Record<string, number>>({});
  const [, redraw] = useState(0);
  const now = performance.now();
  for (const [k, v] of Object.entries(values)) {
    const discrete = typeof v === "boolean" || Number.isInteger(v);
    if (discrete && prev.current[k] !== undefined && prev.current[k] !== v) changed.current[k] = now;
  }
  prev.current = values;
  // Clear flashes even if telemetry stops.
  useEffect(() => {
    const t = setTimeout(() => redraw((n) => n + 1), 800);
    return () => clearTimeout(t);
  }, [values]);
  const keys = Object.keys(values).sort((a, b) => {
    const rank = (k: string) => (typeof values[k] === "boolean" ? 0 : 1);
    return rank(a) - rank(b) || a.localeCompare(b);
  });
  return (
    <Panel title="Sensors" wide>
      {keys.length === 0 && <p class="muted">No telemetry yet.</p>}
      <div class="sensors">
        {keys.map((k) => {
          const v = values[k];
          const hot = now - (changed.current[k] ?? -1e9) < 700;
          return (
            <div key={k} class={`sensor ${hot ? "flash" : ""} ${v === true ? "on" : ""}`}>
              <span class="name">{k}</span>
              <span class="val">{fmt(v)}</span>
            </div>
          );
        })}
      </div>
    </Panel>
  );
}

const ACTUATORS = [
  { id: "vacuum", label: "Vacuum" },
  { id: "main_brush", label: "Main brush" },
  { id: "side_brush", label: "Side brush" },
  { id: "water_pump", label: "Water pump" },
];

export function Controls({ caps, status }: { caps: string[] | null; status: Json | null }) {
  const { error, busy, run } = useAction();
  const [speed, setSpeed] = useState(50);
  const act = status?.actuators ?? {};
  const set = (id: string, enabled: boolean, extra: Json = {}) =>
    run(`${id}-${enabled}`, () => api(`/actuators/${id}`, "POST", { enabled, ...extra }));
  return (
    <Panel title="Hardware">
      <div class="row">
        <button disabled={busy("lidar-true")} onClick={() => set("lidar", true)}>
          Lidar on
        </button>
        <button onClick={() => set("lidar", false)}>
          Lidar off
        </button>
      </div>
      <p class="muted">Lidar spin-up takes ~20 s; STOP waits behind it meanwhile.</p>
      {caps?.includes("lease") && (
        <div class="row">
          <button disabled={busy("lease")} onClick={() => run("lease", () => api("/lease", "POST", { force: true }))}>
            Take control lease
          </button>
          <button onClick={() => run("release", () => api("/lease", "DELETE"))}>
            Release lease
          </button>
        </div>
      )}
      {caps === null ? (
        <p class="muted">loading…</p>
      ) : caps.includes("actuators") ? (
        <>
          <label class="row">
            Speed {speed}%
            <input type="range" min={0} max={100} value={speed} onInput={(e) => setSpeed(+(e.target as HTMLInputElement).value)} />
          </label>
          {ACTUATORS.map((a) => (
            <div class="row" key={a.id}>
              <span class="grow">
                {a.label} {act[a.id]?.enabled ? <b>on</b> : <span class="muted">off</span>}
              </span>
              <button disabled={busy(`${a.id}-true`)} onClick={() => set(a.id, true, { speed })}>
                On
              </button>
              <button onClick={() => set(a.id, false)}>
                Off
              </button>
            </div>
          ))}
        </>
      ) : (
        <p class="muted">Vacuum/brushes/pump disabled ([control] allow_actuators).</p>
      )}
      {error && <p class="error">{error}</p>}
    </Panel>
  );
}

/** DhruvaSLAM answers asynchronously: refresh a little later. */
const later = (f: () => unknown) => () => void setTimeout(f, 1500);

/** Local, filesystem-safe timestamp for map names: 2026-10-18_0931. */
function stamp() {
  const d = new Date();
  const p = (n: number) => String(n).padStart(2, "0");
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}`;
}

export function Slam({ status }: { status: Json | null }) {
  const { error, busy, run } = useAction();
  const [maps, setMaps] = useState<Json[]>([]);
  const d = status?.dhruva;
  const refresh = () => run("maps", async () => setMaps((await api("/maps")).maps ?? []));
  useEffect(() => {
    if (d?.connected) refresh();
  }, [d?.connected]);
  const rs = d?.robot_status;
  return (
    <Panel title="SLAM">
      {rs ? (
        <div class="kv">
          <span>state</span>
          <span>{rs.state}</span>
          <span>keyframes</span>
          <span>{rs.keyframe_count}</span>
          <span>distance</span>
          <span>{num(rs.distance_traveled_m, 2)} m</span>
          <span>area</span>
          <span>{num(rs.map_area_m2)} m²</span>
        </div>
      ) : (
        <p class="muted">{d?.connected ? "waiting for status…" : "DhruvaSLAM offline"}</p>
      )}
      <div class="row">
        <button disabled={busy("start")} onClick={() => run("start", () => api("/slam/start_mapping", "POST", { name: `web-${stamp()}` }))}>
          Start mapping
        </button>
        <button onClick={() => run("stop", () => api("/slam/stop_mapping", "POST", { save: true }).then(later(refresh)))}>
          Stop + save
        </button>
        <button disabled={busy("clear")} onClick={() => confirm("Clear the map being built?") && run("clear", () => api("/slam/clear_map", "POST"))}>
          Clear
        </button>
      </div>
      <div class="row">
        <a class="button" href={apiUrl("/maps/current.pgm")} download="map.pgm">
          Download map (PGM)
        </a>
        <button disabled={busy("maps")} onClick={refresh}>
          Refresh maps
        </button>
      </div>
      {maps.map((m) => (
        <div class="row" key={m.map_id}>
          <span class="grow">
            {m.name || m.map_id} <span class="muted">{num(m.area_m2)} m²</span> {m.active && <b>active</b>}
          </span>
          <button disabled={busy("use")} onClick={() => run("use", () => api(`/maps/${encodeURIComponent(m.map_id)}/enable`, "POST").then(later(refresh)))}>
            Use
          </button>
        </div>
      ))}
      {error && <p class="error">{error}</p>}
    </Panel>
  );
}

export function Recordings({ status }: { status: Json | null }) {
  const { error, busy, run } = useAction();
  const [files, setFiles] = useState<Json[]>([]);
  const active = status?.recording;
  const refresh = async () => setFiles((await api("/recordings")).files ?? []);
  useEffect(() => {
    run("list", refresh);
  }, [active?.name]);
  const remove = (name: string) =>
    confirm(`Delete ${name}?`) &&
    run(`del-${name}`, async () => {
      await api(`/recordings/${encodeURIComponent(name)}`, "DELETE");
      await refresh();
    });
  return (
    <Panel title="Recordings">
      <p class="muted">SangamIO stream as MCAP in RAM (/tmp); opens in Foxglove.</p>
      <div class="row">
        {active ? (
          <>
            <span class="grow">
              ● {active.name} · {active.frames} frames · {num(active.seconds, 0)} s
            </span>
            <button onClick={() => run("rec-stop", () => api("/recordings/stop", "POST"))}>
              Stop
            </button>
          </>
        ) : (
          <button disabled={busy("rec-start")} onClick={() => run("rec-start", () => api("/recordings/start", "POST", { tag: "web" }))}>
            Start recording
          </button>
        )}
      </div>
      {files.map((f) => (
        <div class="row" key={f.name}>
          <a class="grow" href={apiUrl(`/recordings/${encodeURIComponent(f.name)}`)} download={f.name}>
            {f.name}
          </a>
          <span class="muted">{(f.bytes / 1024).toFixed(0)} KiB</span>
          <button disabled={busy(`del-${f.name}`)} aria-label={`Delete ${f.name}`} onClick={() => remove(f.name)}>
            Delete
          </button>
        </div>
      ))}
      {error && <p class="error">{error}</p>}
    </Panel>
  );
}

const ARM_IDLE_MS = 60_000;
/** A teleop grant this long after a "Drive" click arms. */
const ARM_REQUEST_MS = 3_000;
/** Nothing from the server for this long while driving: the link is suspect, disarm. */
const LINK_SILENCE_MS = 1500;
/** Joystick deflection below this is zero (no creeping near the centre). */
const DEADZONE = 0.1;
const ZERO = { linear: 0, angular: 0 };

function isTextInput(t: EventTarget | null) {
  return t instanceof HTMLInputElement || t instanceof HTMLTextAreaElement || t instanceof HTMLSelectElement;
}

/**
 * Teleop: one "Drive" click takes teleop (server lease) and arms; then
 * joystick / WASD / arrows. Any loss — lease, socket, focus, visibility,
 * STOP — zeroes the command and disarms; arming always takes a click.
 */
export function Teleop(props: { link: Link | null; caps: string[] | null; holder: boolean; online: boolean; max: { linear: number; angular: number } }) {
  const { link, holder, max } = props;
  const [armed, setArmed] = useState(false);
  const [stick, setStick] = useState({ x: 0, y: 0 });
  const cmd = useRef(ZERO);
  const lastActive = useRef(Date.now());
  const pad = useRef<HTMLDivElement>(null);
  const keys = useRef(new Set<string>());

  const enabled = props.caps?.includes("teleop") ?? false;
  const live = enabled && holder && armed && props.online;

  const neutral = () => {
    cmd.current = ZERO;
    keys.current.clear();
    setStick({ x: 0, y: 0 });
  };

  // When "Drive" last asked for teleop: a grant arms only shortly after that
  // click (not after a refusal, STOP or hidden tab left the request stale).
  const armRequested = useRef(0);

  // Losing the lease or the link disarms; a fresh grant after "Drive" arms.
  useEffect(() => {
    if (!holder || !props.online) setArmed(false);
    else if (Date.now() - armRequested.current < ARM_REQUEST_MS) setArmed(true);
    armRequested.current = 0;
  }, [holder, props.online]);

  const toggleDrive = () => {
    if (armed) {
      setArmed(false);
      link?.send({ type: "teleop", take: false });
    } else if (holder) {
      setArmed(true);
    } else {
      armRequested.current = Date.now();
      link?.send({ type: "teleop", take: true });
    }
  };

  // Stream the command at 10 Hz while live; dwar zeroes on silence.
  useEffect(() => {
    if (!live) return;
    lastActive.current = Date.now();
    const t = setInterval(() => {
      if ((link?.silentMs() ?? Infinity) > LINK_SILENCE_MS) {
        setArmed(false); // server unheard: don't drive blind
        return;
      }
      const moving = cmd.current.linear !== 0 || cmd.current.angular !== 0;
      link?.send({ type: "drive", ...cmd.current }, moving);
      if (moving) lastActive.current = Date.now();
      else if (Date.now() - lastActive.current > ARM_IDLE_MS) setArmed(false);
    }, 100);
    return () => {
      clearInterval(t);
      neutral();
      link?.send({ type: "drive", ...ZERO });
    };
  }, [live, link]);

  // Hidden tab / page leaving: stop and let go, unconditionally.
  useEffect(() => {
    const leave = () => {
      if (!document.hidden) return;
      armRequested.current = 0;
      neutral();
      setArmed(false);
      link?.send({ type: "drive", ...ZERO });
      link?.send({ type: "teleop", take: false });
    };
    const pagehide = () => {
      armRequested.current = 0;
      link?.send({ type: "drive", ...ZERO });
      link?.send({ type: "teleop", take: false });
    };
    document.addEventListener("visibilitychange", leave);
    addEventListener("pagehide", pagehide);
    return () => {
      document.removeEventListener("visibilitychange", leave);
      removeEventListener("pagehide", pagehide);
    };
  }, [link]);

  // WASD / arrows while live. Window blur drops held keys (missed keyups).
  useEffect(() => {
    if (!live) return;
    const apply = () => {
      const k = keys.current;
      const f = (k.has("w") || k.has("arrowup") ? 1 : 0) - (k.has("s") || k.has("arrowdown") ? 1 : 0);
      const t = (k.has("a") || k.has("arrowleft") ? 1 : 0) - (k.has("d") || k.has("arrowright") ? 1 : 0);
      cmd.current = { linear: f * max.linear, angular: t * max.angular };
      setStick({ x: -t, y: -f });
    };
    const down = (e: KeyboardEvent) => {
      // With a modifier held, macOS never delivers the keyup (Cmd+S would stick
      // "reverse"): a modifier drops all keys and is otherwise ignored.
      if (e.metaKey || e.ctrlKey || e.altKey || e.key === "Meta") {
        neutral();
        return;
      }
      if (isTextInput(e.target) || e.repeat) return;
      const k = e.key.toLowerCase();
      if (!["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"].includes(k)) return;
      e.preventDefault();
      keys.current.add(k);
      apply();
    };
    const up = (e: KeyboardEvent) => {
      keys.current.delete(e.key.toLowerCase());
      apply();
    };
    const blur = () => neutral();
    addEventListener("keydown", down);
    addEventListener("keyup", up);
    addEventListener("blur", blur);
    return () => {
      removeEventListener("keydown", down);
      removeEventListener("keyup", up);
      removeEventListener("blur", blur);
    };
  }, [live, max.linear, max.angular]);

  const fromPointer = (e: PointerEvent) => {
    const el = pad.current;
    if (!el) return;
    const r = el.getBoundingClientRect();
    let x = ((e.clientX - r.left) / r.width) * 2 - 1;
    let y = ((e.clientY - r.top) / r.height) * 2 - 1;
    const n = Math.hypot(x, y);
    if (n > 1) [x, y] = [x / n, y / n];
    setStick({ x, y });
    cmd.current = n < DEADZONE ? ZERO : { linear: -y * max.linear, angular: -x * max.angular };
  };

  return (
    <Panel title="Teleop">
      {props.caps === null ? (
        <p class="muted">loading…</p>
      ) : !enabled ? (
        <p class="muted">Disabled in dwar config ([control] allow_drive).</p>
      ) : (
        <>
          <div class="row">
            <button class={armed ? "" : "primary"} disabled={!props.online} onClick={toggleDrive}>
              {armed ? "Stop driving" : "Drive"}
            </button>
            <span class="muted">{armed ? "drag the pad or use arrows / WASD" : `idle ${ARM_IDLE_MS / 1000} s = stops driving`}</span>
          </div>
          <div
            ref={pad}
            class={`pad ${live ? "live" : ""}`}
            role="application"
            aria-label="Drive joystick: drag to drive, release to stop; keys WASD"
            onPointerDown={(e) => {
              if (!live) return;
              e.currentTarget.setPointerCapture(e.pointerId);
              fromPointer(e);
            }}
            onPointerMove={(e) => live && e.buttons && fromPointer(e)}
            onPointerUp={neutral}
            onPointerCancel={neutral}
            onLostPointerCapture={neutral}
          >
            <div class="knob" style={{ left: `${50 + stick.x * 40}%`, top: `${50 + stick.y * 40}%` }} />
          </div>
          <p class="muted">
            Supervise the robot. ≤ {max.linear.toFixed(2)} m/s, {max.angular.toFixed(2)} rad/s · keys WASD · release = stop · Esc = STOP
          </p>
        </>
      )}
    </Panel>
  );
}

export function Events({ events }: { events: Json[] }) {
  return (
    <Panel title="Events" wide>
      <div class="events">
        {events.length === 0 && <p class="muted">Nothing yet.</p>}
        {events.map((e) => (
          <div key={e.seq} class={`event ${e.level}`}>
            <span class="muted">{num(e.t)}s</span> {e.text}
          </div>
        ))}
      </div>
    </Panel>
  );
}
