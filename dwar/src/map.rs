//! The SLAM occupancy grid, split into tiles stamped with the map version in
//! which they last changed, so clients fetch only what changed.

use crate::proto::dhruva;
use serde_json::{Value, json};

/// Map tiles are this many cells square.
pub const TILE: usize = 64;

/// Occupancy grid split into tiles, each stamped with the map version in
/// which it last changed. A client that has seen version `v` needs only the
/// tiles newer than `v`; a geometry change (the grid grew) resends all.
pub struct MapState {
    pub version: u64,
    /// Version of the last geometry change (size, origin, resolution).
    pub geometry: u64,
    pub map_id: String,
    pub name: String,
    pub resolution: f32,
    pub width: usize,
    pub height: usize,
    pub origin_x: f32,
    pub origin_y: f32,
    pub cells: Vec<u8>,
    pub tiles_x: usize,
    pub tiles_y: usize,
    pub tile_version: Vec<u64>,
}

impl MapState {
    /// Fold a new grid into the previous state.
    /// `version` must grow monotonically across DhruvaSLAM reconnects, so a
    /// client's "tiles newer than v" never misses a restarted map.
    pub fn update(prev: Option<&MapState>, m: dhruva::CurrentMap, version: u64) -> Option<MapState> {
        let (w, h) = (m.width as usize, m.height as usize);
        if w == 0 || h == 0 || m.cells.len() != w * h {
            return None;
        }
        let (tiles_x, tiles_y) = (w.div_ceil(TILE), h.div_ceil(TILE));
        // A different map (enabled or renamed) counts as a geometry change:
        // clients get the new metadata and every tile.
        let same_geometry = prev.is_some_and(|p| {
            p.map_id == m.map_id
                && p.name == m.name
                && p.width == w
                && p.height == h
                && p.resolution == m.resolution
                && p.origin_x == m.origin_x
                && p.origin_y == m.origin_y
        });
        let (geometry, tile_version) = match prev {
            Some(p) if same_geometry => {
                let mut tv = p.tile_version.clone();
                for ty in 0..tiles_y {
                    for tx in 0..tiles_x {
                        if tile_differs(&p.cells, &m.cells, w, h, tx, ty) {
                            tv[ty * tiles_x + tx] = version;
                        }
                    }
                }
                (p.geometry, tv)
            }
            _ => (version, vec![version; tiles_x * tiles_y]),
        };
        Some(MapState {
            version,
            geometry,
            map_id: m.map_id,
            name: m.name,
            resolution: m.resolution,
            width: w,
            height: h,
            origin_x: m.origin_x,
            origin_y: m.origin_y,
            cells: m.cells,
            tiles_x,
            tiles_y,
            tile_version,
        })
    }

    pub fn meta_json(&self) -> Value {
        json!({
            "type": "map_meta",
            "version": self.version,
            "geometry": self.geometry,
            "map_id": self.map_id,
            "name": self.name,
            "resolution": self.resolution,
            "width": self.width,
            "height": self.height,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "tile": TILE,
        })
    }

    /// PGM (P5) of the whole grid, ROS map_server convention (row 0 at the top).
    pub fn pgm(&self) -> Vec<u8> {
        let mut out = format!("P5\n{} {}\n255\n", self.width, self.height).into_bytes();
        for y in (0..self.height).rev() {
            for &c in &self.cells[y * self.width..(y + 1) * self.width] {
                out.push(match c {
                    255 => 205,
                    v if v >= 50 => 0,
                    _ => 254,
                });
            }
        }
        out
    }
}

fn tile_differs(a: &[u8], b: &[u8], w: usize, h: usize, tx: usize, ty: usize) -> bool {
    let x0 = tx * TILE;
    let x1 = (x0 + TILE).min(w);
    (ty * TILE..((ty + 1) * TILE).min(h)).any(|y| a[y * w + x0..y * w + x1] != b[y * w + x0..y * w + x1])
}

