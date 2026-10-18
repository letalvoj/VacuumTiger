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
  free: [number, number, number, number];
  occupied: [number, number, number, number];
}

/** Map tile size in cells (dwar map.rs TILE). */
const TILE = 64;

/** Pointer travel (CSS px) before a press counts as a pan (taps don't unfollow). */
const PAN_SLOP = 5;

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
  private pose: Pose | null = null;
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
    this.invalidate();
  }

  setScan(s: Scan) {
    this.scan = s;
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

    // Live scan, placed with the pose it arrived with when SLAM runs.
    const scan = this.scan;
    if (scan) {
      const p = this.pose ? (scan.pose ?? pose) : pose;
      const cos = Math.cos(p.theta);
      const sin = Math.sin(p.theta);
      const n = scan.ranges.length;
      const r = 2 * dpr;
      ctx.fillStyle = this.palette.scan;
      for (let i = 0; i < n; i++) {
        const d = scan.ranges[i];
        if (d <= 0) continue;
        const a = (i * 2 * Math.PI) / n;
        const lx = d * Math.cos(a) + robot.lidar_offset_x;
        const ly = d * Math.sin(a) + robot.lidar_offset_y;
        const [sx, sy] = this.w2s(p.x + cos * lx - sin * ly, p.y + sin * lx + cos * ly);
        ctx.fillRect(sx - r / 2, sy - r / 2, r, r);
      }
    }

    // Robot body + heading.
    const [rx, ry] = this.w2s(pose.x, pose.y);
    const rr = robot.radius * this.view.scale * dpr;
    ctx.strokeStyle = this.palette.accent;
    ctx.lineWidth = 2 * dpr;
    ctx.beginPath();
    ctx.arc(rx, ry, rr, 0, 2 * Math.PI);
    ctx.moveTo(rx, ry);
    ctx.lineTo(rx + Math.cos(pose.theta) * rr, ry - Math.sin(pose.theta) * rr);
    ctx.stroke();
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
