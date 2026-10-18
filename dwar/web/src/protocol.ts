// Binary WebSocket frames from dwar (encoders: dwar/src/wire.rs; layouts in
// dwar/README.md). Decoders return null for malformed frames.

/** Lidar scan: ranges in metres (0 = no return), bin i at angle i·2π/n CCW from forward. */
export interface Scan {
  /** SLAM pose when the scan arrived; null without SLAM. */
  pose: Pose | null;
  ranges: Float32Array;
}

export interface Pose {
  x: number;
  y: number;
  theta: number;
}

/** One map tile; cells row-major from the grid origin (0 free, 100 occupied, 255 unknown). */
export interface Tile {
  tx: number;
  ty: number;
  w: number;
  h: number;
  geometry: number;
  cells: Uint8Array;
}

export const FRAME_SCAN = 1;
export const FRAME_TILE = 2;

const SCAN_HEADER = 28;
const TILE_HEADER = 16;

export function frameKind(b: ArrayBuffer): number {
  return b.byteLength > 0 ? new Uint8Array(b, 0, 1)[0] : 0;
}

export function decodeScan(b: ArrayBuffer): Scan | null {
  if (b.byteLength < SCAN_HEADER + 2) return null;
  const v = new DataView(b);
  const n = v.getUint16(SCAN_HEADER, true);
  if (b.byteLength < SCAN_HEADER + 2 + n * 3) return null;
  const ranges = new Float32Array(n);
  for (let i = 0; i < n; i++) ranges[i] = v.getUint16(SCAN_HEADER + 2 + i * 3, true) / 4000;
  const x = v.getFloat32(16, true);
  return {
    pose: Number.isNaN(x) ? null : { x, y: v.getFloat32(20, true), theta: v.getFloat32(24, true) },
    ranges,
  };
}

export function decodeTile(b: ArrayBuffer): Tile | null {
  if (b.byteLength < TILE_HEADER) return null;
  const v = new DataView(b);
  const w = v.getUint16(6, true);
  const h = v.getUint16(8, true);
  if (b.byteLength < TILE_HEADER + w * h) return null;
  return {
    tx: v.getUint16(2, true),
    ty: v.getUint16(4, true),
    w,
    h,
    geometry: v.getUint32(12, true),
    cells: new Uint8Array(b, TILE_HEADER, w * h),
  };
}
