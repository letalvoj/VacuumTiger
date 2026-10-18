// App shell. Pages: Debug (now), Map/Clean and Settings later; every page
// renders from /capabilities so the UI never offers what the robot can't do.

import { render } from "preact";
import { useEffect, useRef, useState } from "preact/hooks";
import { Boundary, Controls, Events, Health, Recordings, Sensors, Slam, Teleop } from "./debug";
import { api, AuthError, Link, type Json } from "./link";
import { MapRenderer, MapView, type Robot } from "./mapview";
import "./style.css";

const MAX_EVENTS = 100;
const STOP_TIMEOUT_MS = 3000;

function App() {
  const [online, setOnline] = useState(false);
  const [status, setStatus] = useState<Json | null>(null);
  const [sensors, setSensors] = useState<Json>({});
  const [hasPose, setHasPose] = useState(false);
  const [events, setEvents] = useState<Json[]>([]);
  const [session, setSession] = useState<number | null>(null);
  const [holder, setHolder] = useState(false);
  const [robot, setRobot] = useState<Robot | null>(null);
  // null until /capabilities answers (panels show "loading", not "disabled").
  const [caps, setCaps] = useState<string[] | null>(null);
  const [stopState, setStopState] = useState<"" | "sent" | "failed">("");
  const [authError, setAuthError] = useState<string | null>(null);
  const [link, setLink] = useState<Link | null>(null);
  const map = useRef<MapRenderer | null>(null);
  const seq = useRef(0);

  /** Add an event; `dedupe` collapses repeats of client-side refusals. */
  const addEvent = (level: string, text: string, t = 0, dedupe = false) =>
    setEvents((ev) => (dedupe && ev[0]?.text === text ? ev : [{ level, text, t, seq: ++seq.current }, ...ev].slice(0, MAX_EVENTS)));

  /** Capabilities decide which panels exist: retry until they load (one
   *  retry timer at a time, cancelled on unmount). */
  const capsRetry = useRef(0);
  const loadCaps = (attempt = 0) => {
    clearTimeout(capsRetry.current);
    api("/capabilities").then(
      (c) => setCaps(c.capabilities ?? []),
      (e) => {
        if (e instanceof AuthError) return setAuthError(e.message);
        capsRetry.current = window.setTimeout(() => loadCaps(attempt + 1), Math.min(500 * 2 ** attempt, 10_000));
      },
    );
  };

  useEffect(() => {
    let sessionId: number | null = null;
    const l = new Link({
      online: (up) => {
        setOnline(up);
        if (up) return loadCaps();
        // Offline: nothing on screen may look live.
        setHolder(false);
        setHasPose(false);
        map.current?.setPose(null);
        map.current?.setScan(null);
      },
      json: (m) => {
        switch (m.type) {
          case "hello":
            sessionId = m.session;
            setSession(m.session);
            setRobot(m.robot);
            break;
          case "status":
            setStatus(m);
            setHolder(sessionId !== null && m.teleop_holder === sessionId);
            if (!m.dhruva?.connected) {
              setHasPose(false);
              map.current?.setPose(null);
            }
            break;
          case "sensors":
            setSensors(m.values ?? {});
            break;
          case "pose":
            setHasPose(true);
            map.current?.setPose({ x: m.x, y: m.y, theta: m.theta });
            break;
          case "map_meta":
            map.current?.setMeta(m);
            break;
          case "event":
            addEvent(m.level, m.text, m.t);
            break;
          case "teleop":
            setHolder(!!m.holder);
            if (m.error) addEvent("warn", m.error, 0, true);
            break;
          case "error":
            addEvent("error", m.error);
            break;
        }
      },
      scan: (s) => map.current?.setScan(s),
      tile: (t) => map.current?.addTile(t),
    });
    setLink(l);
    return () => {
      clearTimeout(capsRetry.current);
      l.close();
    };
  }, []);

  /** STOP over the socket and REST both (idempotent): a half-open socket may
   *  still look open. The button shows whether the REST STOP was confirmed. */
  const stop = () => {
    setHolder(false);
    link?.send({ type: "stop" });
    api("/stop", "POST", undefined, STOP_TIMEOUT_MS).then(
      () => flashStop("sent"),
      (e) => {
        flashStop("failed");
        addEvent("error", `STOP not confirmed: ${e.message}`);
      },
    );
  };
  const stopTimer = useRef(0);
  const flashStop = (s: "sent" | "failed") => {
    setStopState(s);
    clearTimeout(stopTimer.current);
    stopTimer.current = window.setTimeout(() => setStopState(""), s === "sent" ? 1500 : 5000);
  };

  // Escape = STOP, anywhere on the page.
  const stopRef = useRef(stop);
  stopRef.current = stop;
  useEffect(() => {
    const key = (e: KeyboardEvent) => e.key === "Escape" && stopRef.current();
    addEventListener("keydown", key);
    return () => removeEventListener("keydown", key);
  }, []);

  const ctl = status?.control ?? {};
  const max = { linear: ctl.max_linear ?? 0.2, angular: ctl.max_angular ?? 1 };

  return (
    <>
      <header>
        <h1>
          <span class="logo">◉</span> VacuumTiger <span class="muted">debug</span>
        </h1>
        {authError && <span class="error">{authError}</span>}
        <button class="danger big" onClick={stop}>
          {stopState === "sent" ? "STOP ✓" : stopState === "failed" ? "STOP FAILED" : "STOP"}
        </button>
      </header>
      <main>
        <section class="card map-card">
          <Boundary title="Map">
            <MapView robot={robot} hasPose={hasPose} bind={(r) => (map.current = r)} />
          </Boundary>
        </section>
        <Boundary title="Links">
          <Health status={status} online={online} />
        </Boundary>
        <Boundary title="Teleop">
          <Teleop link={link} caps={caps} holder={holder && session !== null} online={online} max={max} />
        </Boundary>
        <Boundary title="Hardware">
          <Controls caps={caps} status={status} />
        </Boundary>
        {caps?.includes("slam") && (
          <Boundary title="SLAM">
            <Slam status={status} />
          </Boundary>
        )}
        {caps?.includes("recordings") && (
          <Boundary title="Recordings">
            <Recordings status={status} />
          </Boundary>
        )}
        <Boundary title="Sensors" wide>
          <Sensors values={sensors} />
        </Boundary>
        <Boundary title="Events" wide>
          <Events events={events} />
        </Boundary>
      </main>
    </>
  );
}

render(<App />, document.getElementById("app")!);
