//! Final map as a PNG: occupancy (free space tinted per room when segmented), 1 m grid,
//! odometry path (orange), SLAM path (blue), loop closures (red), start (green) and end
//! (red) markers.

use std::path::Path;

use dhruva_slam::algorithms::mapping::{CellState, OccupancyGrid, Rooms};
use dhruva_slam::engine::graph::{EdgeType, PoseGraph};
use image::{Rgb, RgbImage};
use imageproc::drawing::{draw_filled_circle_mut, draw_line_segment_mut};

use crate::PoseRow;

/// Pixels per map cell.
const SCALE: u32 = 3;
/// Margin around the known area, in cells.
const MARGIN: usize = 10;

const UNKNOWN: Rgb<u8> = Rgb([205, 205, 205]);
const FREE: Rgb<u8> = Rgb([255, 255, 255]);
const OCCUPIED: Rgb<u8> = Rgb([25, 25, 25]);
const GRID: Rgb<u8> = Rgb([190, 205, 225]);
const ODOM: Rgb<u8> = Rgb([255, 150, 30]);
const SLAM: Rgb<u8> = Rgb([40, 110, 255]);
const CLOSURE: Rgb<u8> = Rgb([230, 20, 20]);

/// Free space in a light tint per room, from `rooms` (made from `grid`).
const ROOM_TINTS: [Rgb<u8>; 6] = [
    Rgb([255, 228, 196]),
    Rgb([200, 230, 255]),
    Rgb([210, 245, 210]),
    Rgb([245, 215, 245]),
    Rgb([255, 250, 200]),
    Rgb([215, 240, 240]),
];

pub fn map_png(
    path: &Path,
    grid: &OccupancyGrid,
    rows: &[PoseRow],
    graph: &PoseGraph,
    rooms: Option<&Rooms>,
) -> Result<(), String> {
    let (w, h) = grid.dimensions();
    // Crop to the observed cells.
    let (mut x0, mut y0, mut x1, mut y1) = (w, h, 0, 0);
    for cy in 0..h {
        for cx in 0..w {
            if grid.get_state(cx, cy) != CellState::Unknown {
                (x0, y0, x1, y1) = (x0.min(cx), y0.min(cy), x1.max(cx), y1.max(cy));
            }
        }
    }
    if x0 > x1 {
        return Err("empty map".into());
    }
    let (x0, y0) = (x0.saturating_sub(MARGIN), y0.saturating_sub(MARGIN));
    let (x1, y1) = ((x1 + MARGIN).min(w - 1), (y1 + MARGIN).min(h - 1));
    let (cw, ch) = (x1 - x0 + 1, y1 - y0 + 1);
    let mut img = RgbImage::from_pixel(cw as u32 * SCALE, ch as u32 * SCALE, UNKNOWN);

    let (ox, oy) = grid.origin();
    let res = grid.resolution();
    // World (m) to image pixel; image rows grow downwards, y grows upwards.
    let to_px = |x: f32, y: f32| -> (f32, f32) {
        let cx = (x - ox) / res - x0 as f32;
        let cy = (y - oy) / res - y0 as f32;
        (cx * SCALE as f32, (ch as f32 - cy) * SCALE as f32)
    };

    for cy in y0..=y1 {
        for cx in x0..=x1 {
            let color = match grid.get_state(cx, cy) {
                CellState::Occupied => OCCUPIED,
                CellState::Free => rooms
                    .and_then(|r| r.room(cx, cy))
                    .map_or(FREE, |r| ROOM_TINTS[(r as usize - 1) % ROOM_TINTS.len()]),
                CellState::Unknown => continue,
            };
            let px = (cx - x0) as u32 * SCALE;
            let py = (ch - 1 - (cy - y0)) as u32 * SCALE;
            for dy in 0..SCALE {
                for dx in 0..SCALE {
                    img.put_pixel(px + dx, py + dy, color);
                }
            }
        }
    }

    // 1 m grid lines (under the paths, over unknown/free only).
    let (wx0, wy0) = (ox + x0 as f32 * res, oy + y0 as f32 * res);
    let (wx1, wy1) = (ox + (x1 + 1) as f32 * res, oy + (y1 + 1) as f32 * res);
    for m in (wx0.ceil() as i32)..=(wx1.floor() as i32) {
        draw_dashed(&mut img, to_px(m as f32, wy0), to_px(m as f32, wy1));
    }
    for m in (wy0.ceil() as i32)..=(wy1.floor() as i32) {
        draw_dashed(&mut img, to_px(wx0, m as f32), to_px(wx1, m as f32));
    }

    polyline(
        &mut img,
        rows.iter().map(|r| to_px(r.odom.x, r.odom.y)),
        ODOM,
    );
    polyline(
        &mut img,
        rows.iter().map(|r| to_px(r.slam.x, r.slam.y)),
        SLAM,
    );
    for e in graph
        .edges()
        .iter()
        .filter(|e| e.edge_type == EdgeType::LoopClosure)
    {
        if let (Some(a), Some(b)) = (graph.get_node(e.from), graph.get_node(e.to)) {
            draw_line_segment_mut(
                &mut img,
                to_px(a.pose.x, a.pose.y),
                to_px(b.pose.x, b.pose.y),
                CLOSURE,
            );
        }
    }
    if let (Some(first), Some(last)) = (rows.first(), rows.last()) {
        let dot = |img: &mut RgbImage, (x, y): (f32, f32), c| {
            draw_filled_circle_mut(img, (x as i32, y as i32), 6, c)
        };
        dot(
            &mut img,
            to_px(first.slam.x, first.slam.y),
            Rgb([30, 170, 60]),
        );
        dot(&mut img, to_px(last.slam.x, last.slam.y), CLOSURE);
    }
    img.save(path)
        .map_err(|e| format!("{}: {e}", path.display()))
}

fn polyline(img: &mut RgbImage, pts: impl Iterator<Item = (f32, f32)>, color: Rgb<u8>) {
    let mut prev: Option<(f32, f32)> = None;
    for p in pts {
        if let Some(q) = prev {
            draw_line_segment_mut(img, q, p, color);
        }
        prev = Some(p);
    }
}

/// A light dashed line that leaves walls visible.
fn draw_dashed(img: &mut RgbImage, (ax, ay): (f32, f32), (bx, by): (f32, f32)) {
    let n = ((bx - ax).hypot(by - ay) / 4.0) as usize;
    for i in (0..n).step_by(2) {
        let (t0, t1) = (i as f32 / n as f32, (i + 1) as f32 / n as f32);
        let (sx, sy) = (ax + (bx - ax) * t0, ay + (by - ay) * t0);
        let (ex, ey) = (ax + (bx - ax) * t1, ay + (by - ay) * t1);
        let (px, py) = (sx as u32, sy as u32);
        if px < img.width() && py < img.height() && img.get_pixel(px, py) == &OCCUPIED {
            continue;
        }
        draw_line_segment_mut(img, (sx, sy), (ex, ey), GRID);
    }
}
