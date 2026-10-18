// Map + live lidar on one canvas. World frame (metres, x right, y up) when
// SLAM provides a pose; otherwise robot frame with the heading pointing up
// (and no map, which lives in the world frame). Pointer Events give one code
// path for mouse, pen and touch (pan, pinch).

import { useEffect, useRef, useState } from "preact/hooks";
import type { Json } from "./link";
import type { Pose, Scan, Tile } from "./protocol";

export interface Robot {
  lidar_offset_x: number;
  lidar_offset_y: number;
  radius: number;
}

interface Meta {
  geometry: number;
  resolution: number;
  width: number;
  height: number;
  origin_x: number;
  origin_y: number;
}

interface Palette {
  grid: string;
  scan: string;
  accent: string;
  danger: string;
  ok: string;
  card: string;
  free: [number, number, number, number];
  occupied: [number, number, number, number];
}

/** Map tile size in cells (dwar map.rs TILE). */
const TILE = 64;

/** Pointer travel (CSS px) before a press counts as a pan (taps don't unfollow). */
const PAN_SLOP = 5;

/** Past scans stay on screen this long, fading out. */
const HISTORY_MS = 20_000;
/** Opacity of a fresh past scan: low, so a parked robot's ~100 overlapping
 *  scans still fade instead of stacking up to a solid wall. */
const HISTORY_ALPHA = 0.2;
/** Fading continues even when no message arrives to trigger a redraw. */
const FADE_REDRAW_MS = 250;
/** Point sizes, CSS px. */
const HISTORY_DOT = 1.5;
const LIVE_DOT = 2;

/**
 * Robot model, top view, in the robot frame (x forward, y left); angles from
 * forward, CCW; distances as fractions of the body radius. Bumper halves are
 * arcs of the front; the four cliff IRs sit under the front rim
 * (docs/HW_PROTOCOLS.md 0x01/0x03; the angles are estimates until the bits
 * are mapped on the floor, T3).
 */
const BUMPER_RING = 1.08;
const CLIFF_RING = 0.8;
const TURRET = 0.25;
const BUMPERS = [
  { key: "bumper_left", from: 0, to: Math.PI / 2 },
  { key: "bumper_right", from: -Math.PI / 2, to: 0 },
];
const CLIFFS = [
  { key: "cliff_left_side", angle: (55 * Math.PI) / 180 },
  { key: "cliff_left_front", angle: (25 * Math.PI) / 180 },
  { key: "cliff_right_front", angle: (-25 * Math.PI) / 180 },
  { key: "cliff_right_side", angle: (-55 * Math.PI) / 180 },
];
/** Drive wheels: half the track and the tyre size. */
const WHEEL = { y: 0.66, length: 0.4, width: 0.12 };
/** Below this radius (CSS px) only the body and heading are drawn. */
const DETAIL_MIN_PX = 12;
/** The robot never shrinks below this radius (CSS px). */
const BODY_MIN_PX = 5;

/** Scan points in the world frame, as [x0, y0, x1, y1, ...]. */
function scanPoints(scan: Scan, p: Pose, robot: Robot): Float32Array {
  const cos = Math.cos(p.theta);
  const sin = Math.sin(p.theta);
  const n = scan.ranges.length;
  const out = new Float32Array(2 * n);
  let k = 0;
  for (let i = 0; i < n; i++) {
    const d = scan.ranges[i];
    if (d <= 0) continue;
    const a = (i * 2 * Math.PI) / n;
    const lx = d * Math.cos(a) + robot.lidar_offset_x;
    const ly = d * Math.sin(a) + robot.lidar_offset_y;
    out[k++] = p.x + cos * lx - sin * ly;
    out[k++] = p.y + sin * lx + cos * ly;
  }
  return out.slice(0, k);
}

function rgba(css: string, alpha: number): [number, number, number, number] {
  const m = css.trim().match(/^#([0-9a-f]{6})$/i);
  const n = m ? parseInt(m[1], 16) : 0x808080;
  return [(n >> 16) & 255, (n >> 8) & 255, n & 255, alpha];
}

/** Imperative renderer; the component below owns one per canvas. */
export class MapRenderer {
  private ctx: CanvasRenderingContext2D;
  private meta: Meta | null = null;
  /** Raw occupancy (row 0 = grid origin) and its rendering (row 0 = top). */
  private cells: Uint8Array | null = null;
  private image: HTMLCanvasElement | null = null;
  private scan: Scan | null = null;
  /** When `scan` arrived (performance.now()). */
  private scanAt = 0;
  private fadeTimer = 0;
  /** Earlier scans in the world frame, oldest first (only while SLAM runs). */
  private history: { t: number; points: Float32Array }[] = [];
  private pose: Pose | null = null;
  /** Latest sensor values (bumpers, cliffs, dock) for the robot model. */
  private sensors: Json = {};
  /** View centre (m) and zoom in CSS px per metre. */
  private view = { cx: 0, cy: 0, scale: 60 };
  private pointers = new Map<number, { x: number; y: number }>();
  private pressStart: { x: number; y: number } | null = null;
  private palette!: Palette;
  private frame = 0;
  private resize: ResizeObserver;
  private scheme = matchMedia("(prefers-color-scheme: dark)");
  follow = true;
  onFollowChange: (f: boolean) => void = () => {};

  /** Robot geometry from the server's hello; scan and body wait for it. */
  private robot: Robot | null = null;

  constructor(private canvas: HTMLCanvasElement) {
    const ctx = canvas.getContext("2d");
    if (!ctx) throw new Error("no 2d context");
    this.ctx = ctx;
    this.readPalette();
    canvas.addEventListener("pointerdown", this.down);
    canvas.addEventListener("pointermove", this.move);
    canvas.addEventListener("pointerup", this.up);
    canvas.addEventListener("pointercancel", this.up);
    canvas.addEventListener("wheel", this.wheel, { passive: false });
    this.resize = new ResizeObserver(() => this.invalidate());
    this.resize.observe(canvas);
    this.scheme.addEventListener("change", this.themeChanged);
  }

  destroy() {
    cancelAnimationFrame(this.frame);
    clearTimeout(this.fadeTimer);
    this.resize.disconnect();
    this.scheme.removeEventListener("change", this.themeChanged);
  }

  /** Schedule one redraw. */
  invalidate() {
    if (!this.frame) this.frame = requestAnimationFrame(() => {
      this.frame = 0;
      this.draw();
    });
  }

  setRobot(r: Robot | null) {
    this.robot = r;
    this.invalidate();
  }

  setMeta(m: Json) {
    const meta = m as Meta;
    if (this.meta?.geometry !== meta.geometry) {
      this.cells = new Uint8Array(meta.width * meta.height).fill(255);
      this.image = document.createElement("canvas");
      this.image.width = meta.width;
      this.image.height = meta.height;
    }
    this.meta = meta;
    this.invalidate();
  }

  addTile(t: Tile) {
    const m = this.meta;
    if (!m || !this.cells || t.geometry !== m.geometry) return;
    const x0 = t.tx * TILE;
    const y0 = t.ty * TILE;
    if (x0 + t.w > m.width || y0 + t.h > m.height) return;
    for (let y = 0; y < t.h; y++) this.cells.set(t.cells.subarray(y * t.w, (y + 1) * t.w), (y0 + y) * m.width + x0);
    this.paint(x0, y0, t.w, t.h);
    this.invalidate();
  }

  setPose(p: Pose | null) {
    this.pose = p;
    if (!p) this.history = []; // world-frame points mean nothing without SLAM
    this.invalidate();
  }

  setSensors(values: Json) {
    this.sensors = values;
    this.invalidate();
  }

  setScan(s: Scan | null) {
    // The scan being replaced joins the history, aged from when it arrived.
    // Without its own pose it takes the current one (a scan interval late).
    const prev = this.scan;
    const now = performance.now();
    if (prev && this.pose && this.robot && now - this.scanAt < HISTORY_MS) {
      this.history.push({ t: this.scanAt, points: scanPoints(prev, prev.pose ?? this.pose, this.robot) });
    }
    this.scan = s;
    this.scanAt = now;
    this.invalidate();
  }

  recenter() {
    this.follow = true;
    this.onFollowChange(true);
    this.invalidate();
  }

  private themeChanged = () => {
    this.readPalette();
    if (this.meta) this.paint(0, 0, this.meta.width, this.meta.height);
    this.invalidate();
  };

  private readPalette() {
    const css = getComputedStyle(this.canvas);
    const v = (name: string, fallback: string) => css.getPropertyValue(name).trim() || fallback;
    this.palette = {
      grid: v("--grid", "#888888"),
      scan: v("--scan", "#e0352b"),
      accent: v("--accent", "#f0a020"),
      danger: v("--danger", "#d4342c"),
      ok: v("--ok", "#1f9d55"),
      card: v("--card", "#ffffff"),
      free: rgba(v("--map-free", "#e8ebf0"), 255),
      occupied: rgba(v("--map-occupied", "#1e2430"), 255),
    };
  }

  /** Render a region of the occupancy grid into the map image (rows flipped). */
  private paint(x0: number, y0: number, w: number, h: number) {
    const m = this.meta;
    const g = this.image?.getContext("2d");
    if (!m || !g || !this.cells) return;
    const img = g.createImageData(w, h);
    const { free, occupied } = this.palette;
    for (let y = 0; y < h; y++) {
      const src = (y0 + y) * m.width + x0;
      const dst = (h - 1 - y) * w * 4;
      for (let x = 0; x < w; x++) {
        const c = this.cells[src + x];
        if (c === 255) continue; // unknown: transparent
        img.data.set(c >= 50 ? occupied : free, dst + x * 4);
      }
    }
    g.putImageData(img, x0, m.height - y0 - h);
  }

  private dpr() {
    return window.devicePixelRatio || 1;
  }

  /** World (m) → canvas pixels. */
  private w2s(x: number, y: number): [number, number] {
    const { cx, cy, scale } = this.view;
    const s = scale * this.dpr();
    return [this.canvas.width / 2 + (x - cx) * s, this.canvas.height / 2 - (y - cy) * s];
  }

  private s2w(sx: number, sy: number): [number, number] {
    const { cx, cy, scale } = this.view;
    const s = scale * this.dpr();
    return [cx + (sx - this.canvas.width / 2) / s, cy - (sy - this.canvas.height / 2) / s];
  }

  private draw() {
    const dpr = this.dpr();
    const cw = Math.round(this.canvas.clientWidth * dpr);
    const ch = Math.round(this.canvas.clientHeight * dpr);
    if (this.canvas.width !== cw || this.canvas.height !== ch) {
      this.canvas.width = cw;
      this.canvas.height = ch;
    }
    const ctx = this.ctx;
    const pose = this.pose ?? { x: 0, y: 0, theta: Math.PI / 2 };
    if (this.follow) {
      this.view.cx = pose.x;
      this.view.cy = pose.y;
    }
    ctx.clearRect(0, 0, cw, ch);
    this.drawGrid();

    // The map is in the world frame: only meaningful with a SLAM pose.
    const m = this.meta;
    if (m && this.image && this.pose) {
      ctx.imageSmoothingEnabled = false;
      const [sx, sy] = this.w2s(m.origin_x, m.origin_y + m.height * m.resolution);
      const s = this.view.scale * dpr * m.resolution;
      ctx.drawImage(this.image, sx, sy, m.width * s, m.height * s);
    }

    const robot = this.robot;
    if (!robot) return;

    // Earlier scans fade out; the live one is drawn on top, placed with the
    // pose it arrived with when SLAM runs.
    ctx.fillStyle = this.palette.scan;
    const now = performance.now();
    while (this.history.length && now - this.history[0].t >= HISTORY_MS) this.history.shift();
    for (const h of this.history) {
      ctx.globalAlpha = HISTORY_ALPHA * (1 - (now - h.t) / HISTORY_MS);
      this.drawPoints(h.points, HISTORY_DOT * dpr);
    }
    ctx.globalAlpha = 1;
    if (this.history.length && !this.fadeTimer) {
      this.fadeTimer = window.setTimeout(() => {
        this.fadeTimer = 0;
        this.invalidate();
      }, FADE_REDRAW_MS);
    }
    const scan = this.scan;
    if (scan) this.drawPoints(scanPoints(scan, this.pose ? (scan.pose ?? pose) : pose, robot), LIVE_DOT * dpr);

    this.drawRobot(pose, robot);
  }

  /**
   * Top view of the robot at its pose: body, drive wheels, lidar turret,
   * heading, and the switches lit when they trigger (bumper halves, cliff IRs);
   * a green rim while it sits on the dock contacts.
   */
  private drawRobot(pose: Pose, robot: Robot) {
    const ctx = this.ctx;
    const dpr = this.dpr();
    const p = this.palette;
    // Docked, the bumper/cliff bits read whatever the dock configuration left
    // there (all set under the vendor setup), not hits: no red then.
    const docked = !!this.sensors.is_dock_connected;
    const hit = (k: string) => !docked && !!this.sensors[k];
    const [rx, ry] = this.w2s(pose.x, pose.y);
    const r = Math.max(robot.radius * this.view.scale, BODY_MIN_PX) * dpr; // px
    const perMetre = r / robot.radius;
    ctx.save();
    // Robot frame in canvas px: x forward, y left (canvas y points down).
    ctx.translate(rx, ry);
    ctx.rotate(-pose.theta);
    ctx.scale(1, -1);
    ctx.lineWidth = 2 * dpr;

    // Body, slightly translucent over the scan points it covers.
    ctx.fillStyle = p.card;
    ctx.globalAlpha = 0.85;
    ctx.beginPath();
    ctx.arc(0, 0, r, 0, 2 * Math.PI);
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.strokeStyle = docked ? p.ok : p.accent;
    ctx.stroke();

    // Heading: a notch at the front.
    ctx.fillStyle = p.accent;
    ctx.beginPath();
    ctx.moveTo(0.95 * r, 0);
    ctx.lineTo(0.6 * r, 0.18 * r);
    ctx.lineTo(0.6 * r, -0.18 * r);
    ctx.fill();

    if (r >= DETAIL_MIN_PX * dpr) {
      ctx.fillStyle = p.grid;
      for (const side of [1, -1]) {
        ctx.fillRect((-WHEEL.length / 2) * r, (side * WHEEL.y - WHEEL.width / 2) * r, WHEEL.length * r, WHEEL.width * r);
      }
      ctx.strokeStyle = p.grid;
      ctx.beginPath();
      ctx.arc(robot.lidar_offset_x * perMetre, robot.lidar_offset_y * perMetre, TURRET * r, 0, 2 * Math.PI);
      ctx.stroke();

      const dot = Math.max(2 * dpr, 0.09 * r);
      for (const c of CLIFFS) {
        ctx.fillStyle = hit(c.key) ? p.danger : p.grid;
        ctx.beginPath();
        ctx.arc(CLIFF_RING * r * Math.cos(c.angle), CLIFF_RING * r * Math.sin(c.angle), dot, 0, 2 * Math.PI);
        ctx.fill();
      }
    }

    ctx.lineWidth = Math.max(2 * dpr, 0.08 * r);
    for (const b of BUMPERS) {
      ctx.strokeStyle = hit(b.key) ? p.danger : p.accent;
      ctx.globalAlpha = hit(b.key) ? 1 : 0.35;
      ctx.beginPath();
      ctx.arc(0, 0, BUMPER_RING * r, b.from, b.to);
      ctx.stroke();
    }
    ctx.restore();
  }

  /** World points as squares; `w2s` inlined (tens of thousands per frame). */
  private drawPoints(points: Float32Array, size: number) {
    const { cx, cy, scale } = this.view;
    const s = scale * this.dpr();
    const ox = this.canvas.width / 2 - cx * s - size / 2;
    const oy = this.canvas.height / 2 + cy * s - size / 2;
    for (let i = 0; i < points.length; i += 2) {
      this.ctx.fillRect(ox + points[i] * s, oy - points[i + 1] * s, size, size);
    }
  }

  private drawGrid() {
    const ctx = this.ctx;
    const { width: cw, height: ch } = this.canvas;
    const step = this.view.scale >= 15 ? 1 : 5; // metres
    const [x0, y0] = this.s2w(0, ch);
    const [x1, y1] = this.s2w(cw, 0);
    ctx.strokeStyle = this.palette.grid;
    ctx.globalAlpha = 0.3;
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil(x0 / step) * step; x <= x1; x += step) {
      const sx = Math.round(this.w2s(x, 0)[0]) + 0.5;
      ctx.moveTo(sx, 0);
      ctx.lineTo(sx, ch);
    }
    for (let y = Math.ceil(y0 / step) * step; y <= y1; y += step) {
      const sy = Math.round(this.w2s(0, y)[1]) + 0.5;
      ctx.moveTo(0, sy);
      ctx.lineTo(cw, sy);
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  /** Event position in canvas pixels. */
  private local(e: PointerEvent | WheelEvent) {
    const r = this.canvas.getBoundingClientRect();
    const dpr = this.dpr();
    return { x: (e.clientX - r.left) * dpr, y: (e.clientY - r.top) * dpr };
  }

  private zoomAt(sx: number, sy: number, factor: number) {
    const [wx, wy] = this.s2w(sx, sy);
    this.view.scale = Math.min(500, Math.max(3, this.view.scale * factor));
    const [nx, ny] = this.s2w(sx, sy);
    if (!this.follow) {
      this.view.cx += wx - nx;
      this.view.cy += wy - ny;
    }
    this.invalidate();
  }

  private down = (e: PointerEvent) => {
    this.canvas.setPointerCapture(e.pointerId);
    const p = this.local(e);
    this.pointers.set(e.pointerId, p);
    if (this.pointers.size === 1) this.pressStart = p;
  };

  private move = (e: PointerEvent) => {
    const prev = this.pointers.get(e.pointerId);
    if (!prev) return;
    const cur = this.local(e);
    const s = this.view.scale * this.dpr();
    if (this.pointers.size === 1) {
      const start = this.pressStart;
      if (this.follow && start && Math.hypot(cur.x - start.x, cur.y - start.y) < PAN_SLOP * this.dpr()) return;
      if (this.follow) {
        this.follow = false;
        this.onFollowChange(false);
      }
      this.view.cx -= (cur.x - prev.x) / s;
      this.view.cy += (cur.y - prev.y) / s;
      this.invalidate();
    } else if (this.pointers.size === 2) {
      const other = [...this.pointers.entries()].find(([id]) => id !== e.pointerId)?.[1];
      if (other) {
        const before = Math.hypot(prev.x - other.x, prev.y - other.y);
        const after = Math.hypot(cur.x - other.x, cur.y - other.y);
        if (before > 0) this.zoomAt((cur.x + other.x) / 2, (cur.y + other.y) / 2, after / before);
      }
    }
    this.pointers.set(e.pointerId, cur);
  };

  private up = (e: PointerEvent) => {
    this.pointers.delete(e.pointerId);
    // The remaining finger (after a pinch) starts a fresh press.
    this.pressStart = this.pointers.values().next().value ?? null;
  };

  private wheel = (e: WheelEvent) => {
    e.preventDefault();
    const p = this.local(e);
    const lines = e.deltaMode === WheelEvent.DOM_DELTA_LINE ? 16 : 1;
    this.zoomAt(p.x, p.y, Math.exp(-e.deltaY * lines * 0.0015));
  };
}

export function MapView(props: { robot: Robot | null; bind: (r: MapRenderer | null) => void; hasPose: boolean }) {
  const ref = useRef<HTMLCanvasElement>(null);
  const renderer = useRef<MapRenderer | null>(null);
  const [follow, setFollow] = useState(true);

  // Created on mount, before the robot is known, so no early tile is lost.
  useEffect(() => {
    if (!ref.current) return;
    const r = new MapRenderer(ref.current);
    r.onFollowChange = setFollow;
    renderer.current = r;
    props.bind(r);
    return () => {
      props.bind(null);
      r.destroy();
    };
  }, []);

  useEffect(() => {
    renderer.current?.setRobot(props.robot);
  }, [props.robot]);

  return (
    <div class="mapview">
      <canvas ref={ref} role="img" aria-label="Map and live lidar scan" />
      <div class="map-overlay">
        <span class="badge">{props.hasPose ? "map frame" : "robot frame (no SLAM pose)"}</span>
        {!follow && (
          <button class="small" onClick={() => renderer.current?.recenter()}>
            Follow robot
          </button>
        )}
      </div>
    </div>
  );
}
