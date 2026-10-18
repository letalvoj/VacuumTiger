// Connection to dwar: REST helpers and the tab's one WebSocket.

import { decodeScan, decodeTile, FRAME_SCAN, FRAME_TILE, frameKind, type Scan, type Tile } from "./protocol";

export type Json = Record<string, any>;

const TOKEN_KEY = "dwar.token";

/** Access token from `?token=` (then stored and stripped from the URL) or storage. */
function token(): string | null {
  const params = new URLSearchParams(location.search);
  const fromUrl = params.get("token");
  try {
    if (fromUrl) {
      localStorage.setItem(TOKEN_KEY, fromUrl);
      params.delete("token");
      const rest = params.toString();
      history.replaceState(null, "", `${location.pathname}${rest ? `?${rest}` : ""}${location.hash}`);
    }
    return fromUrl ?? localStorage.getItem(TOKEN_KEY);
  } catch {
    return fromUrl;
  }
}

const TOKEN = token();

/** API paths are relative to the page, so the UI works under any prefix (e.g. a proxy's /vacuum/). */
const API = "api/v1";

/** Page-relative URL for a plain link (download) that carries the token if one is set. */
export function apiUrl(path: string): string {
  return `${API}${path}${TOKEN ? `${path.includes("?") ? "&" : "?"}token=${encodeURIComponent(TOKEN)}` : ""}`;
}

/** Generous default: a lidar enable blocks SangamIO for ~20 s. */
const API_TIMEOUT_MS = 30_000;

/** Thrown for 401: retrying won't help until the token is fixed. */
export class AuthError extends Error {}

export async function api(path: string, method = "GET", body?: unknown, timeoutMs = API_TIMEOUT_MS): Promise<Json> {
  const headers: Record<string, string> = {};
  if (TOKEN) headers.Authorization = `Bearer ${TOKEN}`;
  if (body !== undefined) headers["Content-Type"] = "application/json";
  const abort = new AbortController();
  const timer = setTimeout(() => abort.abort(), timeoutMs);
  try {
    const r = await fetch(`${API}${path}`, {
      method,
      headers,
      body: body === undefined ? undefined : JSON.stringify(body),
      signal: abort.signal,
    });
    const data = await r.json().catch(() => ({}));
    if (r.status === 401) throw new AuthError("token rejected: open the page with ?token=…");
    if (!r.ok) throw new Error(data.error ?? `${r.status} ${r.statusText}`);
    return data;
  } catch (e) {
    if (abort.signal.aborted) throw new Error(`timed out after ${timeoutMs / 1000} s`);
    throw e;
  } finally {
    clearTimeout(timer);
  }
}

export interface Handlers {
  json(msg: Json): void;
  scan(s: Scan): void;
  tile(t: Tile): void;
  online(up: boolean): void;
}

/** Topic → max Hz. The server never queues: a slow link just gets fewer updates. */
const RATES = { status: 1, sensors: 10, pose: 10, scan: 6, map: 1, events: 1 };
const HIDDEN_RATES = { status: 1, events: 1 };
/** Status arrives every second; this much silence means a dead link. */
const SILENCE_MS = 4000;
const CONNECT_TIMEOUT_MS = 5000;
/** Bytes queued in the browser beyond which drive commands are dropped. */
const MAX_BUFFERED = 256;

/** The tab's WebSocket: reconnects with backoff, detects silent links, and
 *  throttles to status-only while the tab is hidden. */
export class Link {
  private ws: WebSocket | null = null;
  private backoff = 500;
  private closed = false;
  private retry = 0;
  private lastRx = 0;
  private watchdog: number;
  private connectTimer = 0;

  constructor(private h: Handlers) {
    this.watchdog = window.setInterval(() => {
      if (this.ws?.readyState === WebSocket.OPEN && Date.now() - this.lastRx > SILENCE_MS) this.dropped(this.ws);
    }, 1000);
    document.addEventListener("visibilitychange", this.wake);
    addEventListener("online", this.wake);
    this.open();
  }

  /** Send a message. A `droppable` one (a non-zero drive command) is skipped
   *  when the socket is backed up, so a stalled link never replays a burst
   *  of motion on recovery; stops and zeroes always go out. */
  send(msg: Json, droppable = false): boolean {
    const ws = this.ws;
    if (ws?.readyState !== WebSocket.OPEN) return false;
    if (droppable && ws.bufferedAmount > MAX_BUFFERED) return false;
    ws.send(JSON.stringify(msg));
    return true;
  }

  /** Milliseconds since the server last sent anything (sensors arrive at 10 Hz). */
  silentMs(): number {
    return this.ws ? Date.now() - this.lastRx : Infinity;
  }

  close() {
    this.closed = true;
    clearTimeout(this.retry);
    clearInterval(this.watchdog);
    document.removeEventListener("visibilitychange", this.wake);
    removeEventListener("online", this.wake);
    this.ws?.close();
  }

  private subscribe() {
    this.send({ type: "subscribe", topics: document.hidden ? HIDDEN_RATES : RATES });
  }

  /** Tab visible again / network back: resubscribe, or reconnect right away. */
  private wake = () => {
    if (this.ws) this.subscribe();
    else if (!document.hidden && !this.closed) {
      clearTimeout(this.retry);
      this.backoff = 500;
      this.open();
    }
  };

  private open() {
    if (this.closed || this.ws) return;
    const url = new URL(apiUrl("/ws"), location.href);
    url.protocol = url.protocol === "https:" ? "wss:" : "ws:";
    const ws = new WebSocket(url);
    ws.binaryType = "arraybuffer";
    // A connect hanging in CONNECTING (host unreachable) gives up after 5 s.
    this.connectTimer = window.setTimeout(() => this.dropped(ws), CONNECT_TIMEOUT_MS);
    ws.onopen = () => {
      clearTimeout(this.connectTimer);
      this.backoff = 500;
      this.lastRx = Date.now();
      this.h.online(true);
      this.subscribe();
    };
    ws.onmessage = (e) => {
      this.lastRx = Date.now();
      try {
        if (typeof e.data === "string") {
          this.h.json(JSON.parse(e.data));
          return;
        }
        const b = e.data as ArrayBuffer;
        const kind = frameKind(b);
        if (kind === FRAME_SCAN) {
          const s = decodeScan(b);
          if (s) this.h.scan(s);
        } else if (kind === FRAME_TILE) {
          const t = decodeTile(b);
          if (t) this.h.tile(t);
        }
      } catch (err) {
        console.warn("dwar: bad frame", err);
      }
    };
    ws.onclose = () => this.dropped(ws);
    this.ws = ws;
  }

  /** Forget a socket now (closed, or silent: a half-open one may take ages
   *  to report close) and schedule a reconnect. */
  private dropped(ws: WebSocket) {
    if (this.ws !== ws) return;
    clearTimeout(this.connectTimer);
    this.ws = null;
    ws.onopen = ws.onclose = ws.onmessage = null;
    ws.close();
    this.h.online(false);
    if (this.closed) return;
    this.retry = window.setTimeout(() => this.open(), this.backoff);
    this.backoff = Math.min(this.backoff * 2, 5000);
  }
}
