//! Map loader for PGM+YAML format (ROS standard)
//!
//! Loads occupancy grid maps with optional cliff mask layer.

use crate::error::{Error, Result};
use image::GrayImage;
use serde::Deserialize;
use std::path::Path;

/// Map metadata from YAML file (ROS standard format)
#[derive(Debug, Deserialize)]
pub struct MapMetadata {
    /// PGM image filename (relative to YAML file)
    pub image: String,

    /// Map resolution in meters per pixel
    pub resolution: f32,

    /// Origin of map [x, y, yaw] - world coordinates of bottom-left pixel
    pub origin: [f32; 3],

    /// Threshold for occupied cells (0.0-1.0, pixel > threshold*255 = occupied)
    #[serde(default = "default_occupied_thresh")]
    pub occupied_thresh: f32,

    /// Optional cliff mask file (same dimensions as main map)
    #[serde(default)]
    pub cliff_mask: Option<String>,

    /// Optional lidar mask file (same dimensions as main map): see [`SimulationMap`]
    #[serde(default)]
    pub lidar_mask: Option<String>,
}

fn default_occupied_thresh() -> f32 {
    0.65
}

/// Simulation map loaded from PGM+YAML
///
/// The occupancy grid is what the body runs into. The lidar sees the same, unless a lidar
/// mask (an extension of the ROS format) says otherwise: there a cell's darkness is the
/// chance that a beam crossing the cell returns from it, so it depends on the resolution.
/// Glass is occupied but clear in the mask; a curtain is free but a light gray in it, so
/// the beams stop somewhere among its folds.
pub struct SimulationMap {
    /// Occupancy grid pixels
    pixels: GrayImage,

    /// Optional cliff mask layer
    cliff_mask: Option<GrayImage>,

    /// Optional lidar mask layer
    lidar_mask: Option<GrayImage>,

    /// Resolution in meters per pixel
    resolution: f32,

    /// Origin (x, y) in world coordinates
    origin: (f32, f32),

    /// Pixel threshold for occupied (0-255)
    occupied_thresh: u8,
}

impl SimulationMap {
    /// A map from an occupancy image built in memory (black = occupied).
    pub fn from_image(pixels: GrayImage, resolution: f32, origin: (f32, f32)) -> Self {
        Self {
            pixels,
            cliff_mask: None,
            lidar_mask: None,
            resolution,
            origin,
            occupied_thresh: 128,
        }
    }

    /// The map with a lidar mask (the size of the map).
    pub fn with_lidar_mask(self, mask: GrayImage) -> Self {
        assert_eq!(
            mask.dimensions(),
            self.pixels.dimensions(),
            "lidar mask size"
        );
        Self {
            lidar_mask: Some(mask),
            ..self
        }
    }

    /// Load map from ROS-standard YAML + PGM files
    pub fn load<P: AsRef<Path>>(yaml_path: P) -> Result<Self> {
        let yaml_path = yaml_path.as_ref();

        // Read and parse YAML
        let yaml_content = std::fs::read_to_string(yaml_path)
            .map_err(|e| Error::Config(format!("Failed to read map YAML: {}", e)))?;

        let metadata: MapMetadata = serde_yaml::from_str(&yaml_content)
            .map_err(|e| Error::Config(format!("Failed to parse map YAML: {}", e)))?;

        // Determine base directory for relative paths
        let yaml_dir = yaml_path.parent().unwrap_or(Path::new("."));

        // Load main PGM image
        let pgm_path = yaml_dir.join(&metadata.image);
        let img = image::open(&pgm_path)
            .map_err(|e| {
                Error::Config(format!(
                    "Failed to load map image {}: {}",
                    pgm_path.display(),
                    e
                ))
            })?
            .into_luma8();

        // Optional masks, the size of the map
        let mask = |file: &Option<String>, what: &str| -> Result<Option<GrayImage>> {
            let Some(file) = file else {
                return Ok(None);
            };
            let path = yaml_dir.join(file);
            let mask = image::open(&path)
                .map_err(|e| {
                    Error::Config(format!("Failed to load {what} {}: {e}", path.display()))
                })?
                .into_luma8();
            if mask.dimensions() != img.dimensions() {
                return Err(Error::Config(format!(
                    "{what} dimensions {:?} don't match map dimensions {:?}",
                    mask.dimensions(),
                    img.dimensions()
                )));
            }
            Ok(Some(mask))
        };
        let cliff_mask = mask(&metadata.cliff_mask, "cliff mask")?;
        let lidar_mask = mask(&metadata.lidar_mask, "lidar mask")?;

        // Convert threshold to pixel value
        // occupied_thresh: pixels BELOW this are occupied (darker = occupied)
        let occupied_thresh = ((1.0 - metadata.occupied_thresh) * 255.0) as u8;

        Ok(Self {
            pixels: img,
            cliff_mask,
            lidar_mask,
            resolution: metadata.resolution,
            origin: (metadata.origin[0], metadata.origin[1]),
            occupied_thresh,
        })
    }

    /// Get map width in pixels
    pub fn width(&self) -> u32 {
        self.pixels.width()
    }

    /// Get map height in pixels
    pub fn height(&self) -> u32 {
        self.pixels.height()
    }

    /// Get map resolution in meters per pixel
    pub fn resolution(&self) -> f32 {
        self.resolution
    }

    /// Get map origin (world coordinates of bottom-left pixel)
    pub fn origin(&self) -> (f32, f32) {
        self.origin
    }

    /// Convert pixel coordinates to world coordinates
    ///
    /// Returns (x, y) in meters. The center of the pixel is returned.
    pub fn pixel_to_world(&self, px: u32, py: u32) -> (f32, f32) {
        let x = self.origin.0 + (px as f32 + 0.5) * self.resolution;
        // Y is inverted: bottom of image = origin, top of image = +Y
        let y =
            self.origin.1 + (self.pixels.height() as f32 - 1.0 - py as f32 + 0.5) * self.resolution;
        (x, y)
    }

    /// Convert world coordinates to pixel coordinates
    fn world_to_pixel(&self, x: f32, y: f32) -> Option<(u32, u32)> {
        let px = ((x - self.origin.0) / self.resolution).floor() as i32;
        // Y is inverted: bottom of image = origin, top of image = +Y
        let py = (self.pixels.height() as i32 - 1)
            - ((y - self.origin.1) / self.resolution).floor() as i32;

        if px >= 0
            && py >= 0
            && (px as u32) < self.pixels.width()
            && (py as u32) < self.pixels.height()
        {
            Some((px as u32, py as u32))
        } else {
            None
        }
    }

    /// Check if world coordinate is occupied
    pub fn is_occupied(&self, x: f32, y: f32) -> bool {
        match self.world_to_pixel(x, y) {
            Some((px, py)) => {
                let pixel = self.pixels.get_pixel(px, py).0[0];
                pixel < self.occupied_thresh // Darker = occupied
            }
            None => true, // Out of bounds = occupied (wall)
        }
    }

    /// Whether any occupied cell reaches within `radius` of the world point (x, y): each
    /// cell's nearest point counts, so the disc never overlaps a wall cell.
    pub fn disc_occupied(&self, x: f32, y: f32, radius: f32) -> bool {
        let res = self.resolution;
        let cells = |c: f32, o: f32| {
            let first = ((c - radius - o) / res).floor() as i32;
            let last = ((c + radius - o) / res).floor() as i32;
            (first..=last).map(move |i| o + (i as f32 + 0.5) * res)
        };
        let near = |c: f32, centre: f32| (c - centre).abs() - res / 2.0;
        cells(x, self.origin.0).any(|cx| {
            cells(y, self.origin.1).any(|cy| {
                let (dx, dy) = (near(x, cx).max(0.0), near(y, cy).max(0.0));
                dx.hypot(dy) <= radius && self.is_occupied(cx, cy)
            })
        })
    }

    /// Check if world coordinate is a cliff (from mask or map boundary)
    pub fn is_cliff(&self, x: f32, y: f32) -> bool {
        match self.world_to_pixel(x, y) {
            Some((px, py)) => {
                if let Some(mask) = &self.cliff_mask {
                    let pixel = mask.get_pixel(px, py).0[0];
                    pixel < 128 // Black = cliff, White = safe
                } else {
                    false // No mask, not a cliff
                }
            }
            None => true, // Out of bounds = cliff
        }
    }

    /// The chance (0..1) that a lidar beam crossing the cell at the world point (x, y)
    /// returns from it: the lidar mask's darkness, or else whether the cell is occupied.
    pub fn lidar_opacity(&self, x: f32, y: f32) -> f32 {
        match (&self.lidar_mask, self.world_to_pixel(x, y)) {
            (Some(mask), Some((px, py))) => 1.0 - f32::from(mask.get_pixel(px, py).0[0]) / 255.0,
            _ => f32::from(u8::from(self.is_occupied(x, y))),
        }
    }

    /// Distance from (ox, oy) along `angle` to where the ray enters the first cell that
    /// `hits` (given the cell's centre), or `max_range`. Exact: the cells are walked as
    /// the ray crosses them (Amanatides & Woo). The cell holding the origin is not tested:
    /// the lidar is never inside what it sees. Ranges come in steps of the resolution's
    /// fraction along the ray, so benches use fine (1 cm) maps.
    pub fn ray_cast(
        &self,
        ox: f32,
        oy: f32,
        angle: f32,
        max_range: f32,
        mut hits: impl FnMut(f32, f32) -> bool,
    ) -> f32 {
        if !(ox.is_finite() && oy.is_finite() && angle.is_finite()) {
            return max_range;
        }
        let res = self.resolution;
        let (dy, dx) = angle.sin_cos();
        // In cells from the origin of the map.
        let (gx, gy) = ((ox - self.origin.0) / res, (oy - self.origin.1) / res);
        let (mut i, mut j) = (gx.floor(), gy.floor());
        // Per axis: the cell step, the ray length per cell, and to the next cell border.
        let axis = |g: f32, cell: f32, d: f32| {
            if d == 0.0 {
                return (0.0, f32::INFINITY, f32::INFINITY);
            }
            let per_cell = res / d.abs();
            let to_border = if d > 0.0 { cell + 1.0 - g } else { g - cell };
            (d.signum(), per_cell, to_border * per_cell)
        };
        let (step_i, per_i, mut next_i) = axis(gx, i, dx);
        let (step_j, per_j, mut next_j) = axis(gy, j, dy);
        loop {
            let distance = if next_i < next_j {
                i += step_i;
                let distance = next_i;
                next_i += per_i;
                distance
            } else {
                j += step_j;
                let distance = next_j;
                next_j += per_j;
                distance
            };
            if distance >= max_range {
                return max_range;
            }
            let centre = (
                self.origin.0 + (i + 0.5) * res,
                self.origin.1 + (j + 0.5) * res,
            );
            if hits(centre.0, centre.1) {
                return distance;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_world_to_pixel() {
        // Create a simple 100x100 map with 0.1m resolution
        // Origin at (-5, -5), so map covers (-5,-5) to (5,5) in world coords
        let img = GrayImage::from_fn(100, 100, |_, _| image::Luma([255u8]));
        let map = SimulationMap::from_image(img, 0.1, (-5.0, -5.0));

        // Origin should map to bottom-left pixel
        assert_eq!(map.world_to_pixel(-5.0, -5.0), Some((0, 99)));

        // Center should map to center pixel
        assert_eq!(map.world_to_pixel(0.0, 0.0), Some((50, 49)));

        // Out of bounds should return None
        assert_eq!(map.world_to_pixel(-10.0, 0.0), None);
        assert_eq!(map.world_to_pixel(10.0, 0.0), None);
        assert_eq!(
            map.world_to_pixel(-5.05, 0.0),
            None,
            "under a cell past the edge"
        );
    }

    #[test]
    fn a_disc_reaches_a_cell_by_its_nearest_point() {
        // One occupied 1 cm cell, [0.50, 0.51) x [0.50, 0.51), on a map with its origin off zero.
        let img = GrayImage::from_fn(200, 200, |px, py| {
            image::Luma([if (px, py) == (150, 49) { 0 } else { 255 }])
        });
        let map = SimulationMap::from_image(img, 0.01, (-1.0, -1.0));
        assert!(map.is_occupied(0.505, 0.505));
        let r = 0.175;
        // Off the cell's left face, level with an off-grid point of it.
        let y = 0.5037;
        assert!(
            map.disc_occupied(0.5 - r + 0.004, y, r),
            "just over the face"
        );
        assert!(
            !map.disc_occupied(0.5 - r - 0.0001, y, r),
            "just short of it"
        );
    }

    /// A 2 x 1 m floor at 1 cm per cell; `column` paints both layers per pixel column.
    fn strip(column: impl Fn(u32) -> (u8, u8)) -> SimulationMap {
        let layer = |pick: fn((u8, u8)) -> u8| {
            GrayImage::from_fn(200, 100, |i, _| image::Luma([pick(column(i))]))
        };
        SimulationMap::from_image(layer(|c| c.0), 0.01, (0.0, 0.0)).with_lidar_mask(layer(|c| c.1))
    }

    fn opaque(map: &SimulationMap) -> impl FnMut(f32, f32) -> bool + '_ {
        |x, y| map.lidar_opacity(x, y) >= 1.0
    }

    #[test]
    fn a_ray_stops_at_the_first_hit() {
        let map = SimulationMap::from_image(
            GrayImage::from_fn(200, 100, |i, _| {
                image::Luma([if i >= 150 { 0 } else { 255 }])
            }),
            0.01,
            (0.0, 0.0),
        );
        let hit = map.ray_cast(0.5, 0.5, 0.0, 5.0, opaque(&map));
        assert!((hit - 1.0).abs() < 0.01, "hit {hit}");
        assert_eq!(map.ray_cast(0.5, 0.5, 0.0, 0.5, opaque(&map)), 0.5);
    }

    #[test]
    fn rays_are_exact_along_the_axes_and_diagonals() {
        // A 2 x 2 m box around the map origin, 10 cm walls; cast from the origin, which
        // lies on cell borders in both axes.
        let map = SimulationMap::from_image(
            GrayImage::from_fn(200, 200, |i, j| {
                let wall = !(10..190).contains(&i) || !(10..190).contains(&j);
                image::Luma([if wall { 0 } else { 255 }])
            }),
            0.01,
            (-1.0, -1.0),
        );
        let quarter = std::f32::consts::FRAC_PI_4;
        for (k, expected) in [
            (0, 0.9),
            (2, 0.9),
            (4, 0.9),
            (6, 0.9),
            (1, 0.9 * 2f32.sqrt()),
        ] {
            let angle = k as f32 * quarter;
            let hit = map.ray_cast(0.0, 0.0, angle, 5.0, opaque(&map));
            assert!(
                (hit - expected).abs() < 1e-4,
                "{angle}: {hit} vs {expected}"
            );
        }
        assert_eq!(map.ray_cast(f32::NAN, 0.0, 0.0, 5.0, opaque(&map)), 5.0);
    }

    #[test]
    fn glass_stops_the_body_not_the_beams() {
        // Glass at x in [1.0, 1.02): occupied, clear to the lidar; a wall beyond x = 1.5.
        let map = strip(|i| match i {
            100..102 => (0, 255),
            150.. => (0, 0),
            _ => (255, 255),
        });
        assert!(map.is_occupied(1.01, 0.5));
        let hit = map.ray_cast(0.5, 0.5, 0.0, 5.0, opaque(&map));
        assert!(
            (hit - 1.0).abs() < 0.01,
            "sees the wall behind the glass: {hit}"
        );
    }

    #[test]
    fn a_curtain_is_seen_and_passed() {
        let map = strip(|i| match i {
            100..120 => (255, 128),
            _ => (255, 255),
        });
        assert!(!map.is_occupied(1.1, 0.5));
        let opacity = map.lidar_opacity(1.1, 0.5);
        assert!(
            (opacity - 0.5).abs() < 0.01,
            "half the beams per step: {opacity}"
        );
    }
}
