//! The run's map cleaned into a map for the SangamIO mock: `sim_map.png` + `sim_map.yaml`,
//! the ROS map_server format the mock loads, in the run's frame (the mock starts the robot at
//! the run's first pose).
//!
//! The floor is what the robot's body can reach from where the run started: the free cells
//! within its radius of a cell its centre can drive to. Its outline is straightened into
//! polygons (the walls of a building are straight); the holes in it (table and chair legs)
//! stay cell for cell; everything else is wall. Space the lidar saw through a mirror or glass
//! but that is still joined to the floor is cut off with `--block` boxes (walls across where
//! it joins), until the plan is drawn by hand.

use std::path::Path;

use dhruva_slam::algorithms::mapping::{CellState, OccupancyGrid};
use image::{GrayImage, Luma};
use imageproc::contours::{BorderType, find_contours};
use imageproc::distance_transform::euclidean_squared_distance_transform;
use imageproc::drawing::draw_polygon_mut;
use imageproc::geometry::approximate_polygon_dp;
use imageproc::region_labelling::{Connectivity, connected_components};

const FLOOR: u8 = 254;
const WALL: u8 = 0;
/// How far a straightened wall may stray from the floor's outline (cells); the robot's
/// margin covers it.
const STRAIGHTEN: f64 = 1.5;

/// A box to make wall, in metres: x0, y0, x1, y1.
pub type Block = [f32; 4];

/// Parses `x0,y0,x1,y1` into a box with x0 ≤ x1 and y0 ≤ y1.
pub fn parse_block(s: &str) -> Result<Block, String> {
    let v: Vec<f32> = (s.split(','))
        .map(|n| n.trim().parse().map_err(|e| format!("{s}: {e}")))
        .collect::<Result<_, _>>()?;
    let [x0, y0, x1, y1]: Block = v
        .try_into()
        .map_err(|_| format!("{s}: expected x0,y0,x1,y1"))?;
    if [x0, y0, x1, y1].iter().any(|n| !n.is_finite()) {
        return Err(format!("{s}: not a number"));
    }
    Ok([x0.min(x1), y0.min(y1), x0.max(x1), y0.max(y1)])
}

/// Writes [`render`] as `<dir>/sim_map.{png,yaml}`.
pub fn write(
    dir: &Path,
    grid: &OccupancyGrid,
    start: (f32, f32),
    keep: f32,
    blocks: &[Block],
) -> Result<(), String> {
    let png = dir.join("sim_map.png");
    (render(grid, start, keep, blocks).save(&png))
        .map_err(|e| format!("{}: {e}", png.display()))?;
    let res = grid.resolution();
    let (ox, oy) = grid.origin();
    // `negate` and `free_thresh` are for other map_server readers; the mock ignores them.
    let yaml = format!(
        "image: sim_map.png\nresolution: {res}\norigin: [{ox}, {oy}, 0.0]\nnegate: 0\n\
         occupied_thresh: 0.65\nfree_thresh: 0.196\n"
    );
    let path = dir.join("sim_map.yaml");
    std::fs::write(&path, yaml).map_err(|e| format!("{}: {e}", path.display()))
}

/// The map of `grid` for a robot keeping `keep` m (radius and margin) from obstacles that
/// started at `start`, with the `blocks` walled off: image rows from the top (largest y) down.
pub fn render(grid: &OccupancyGrid, start: (f32, f32), keep: f32, blocks: &[Block]) -> GrayImage {
    let (w, h) = grid.dimensions();
    let res = grid.resolution();
    let blocked = blocked_cells(grid, blocks);
    // The blocks are walls, so floor reached only through them goes too.
    let mut clearance = grid.clearance();
    for (c, _) in clearance.iter_mut().zip(&blocked).filter(|(_, b)| **b) {
        *c = 0.0;
    }
    let reachable = grid.reachable(&clearance, keep, start);
    let centres = GrayImage::from_fn(w as u32, h as u32, |x, y| {
        Luma([u8::from(reachable[y as usize * w + x as usize])])
    });
    let to_centre = euclidean_squared_distance_transform(&centres);
    let floor = GrayImage::from_fn(w as u32, h as u32, |cx, cy| {
        let i = cy as usize * w + cx as usize;
        let body = to_centre.get_pixel(cx, cy)[0].sqrt() as f32 * res <= keep + res / 2.0;
        let free = grid.get_state(cx as usize, cy as usize) == CellState::Free;
        Luma([if free && body && !blocked[i] {
            FLOOR
        } else {
            WALL
        }])
    });

    // Straight outlines: the outer contours of the floor simplified into polygons.
    let mut outlined = GrayImage::from_pixel(w as u32, h as u32, Luma([WALL]));
    for contour in find_contours::<i32>(&floor) {
        if contour.border_type != BorderType::Outer {
            continue;
        }
        let mut corners = approximate_polygon_dp(&contour.points, STRAIGHTEN, true);
        if corners.len() > 1 && corners.first() == corners.last() {
            corners.pop();
        }
        if corners.len() >= 3 {
            draw_polygon_mut(&mut outlined, &corners, Luma([FLOOR]));
        }
    }
    // Holes: the parts of the rest not joined to the grid's edge, kept as they are.
    let rest = GrayImage::from_fn(w as u32, h as u32, |x, y| {
        Luma([u8::from(floor.get_pixel(x, y)[0] != FLOOR)])
    });
    let parts = connected_components(&rest, Connectivity::Four, Luma([0]));
    let mut outside = vec![false; w * h + 1];
    for (x, y, p) in parts.enumerate_pixels() {
        if x == 0 || y == 0 || x + 1 == w as u32 || y + 1 == h as u32 {
            outside[p[0] as usize] = true;
        }
    }
    GrayImage::from_fn(w as u32, h as u32, |x, row| {
        let cy = h as u32 - 1 - row;
        let part = parts.get_pixel(x, cy)[0] as usize;
        let hole = part > 0 && !outside[part];
        let is_floor = outlined.get_pixel(x, cy)[0] == FLOOR && !hole;
        Luma([if is_floor { FLOOR } else { WALL }])
    })
}

/// Per cell of `grid`, whether its centre is in one of the `blocks`.
fn blocked_cells(grid: &OccupancyGrid, blocks: &[Block]) -> Vec<bool> {
    let (w, h) = grid.dimensions();
    let mut blocked = vec![false; w * h];
    for &[x0, y0, x1, y1] in blocks {
        let (cx0, cy0) = grid.world_to_cell_signed(x0, y0);
        let (cx1, cy1) = grid.world_to_cell_signed(x1, y1);
        let xs = cx0.max(0) as usize..=(cx1.min(w as i32 - 1)).max(-1) as usize;
        for cy in cy0.max(0)..=cy1.min(h as i32 - 1) {
            for cx in xs.clone() {
                let (x, y) = grid.cell_to_world(cx, cy as usize);
                if (x0..=x1).contains(&x) && (y0..=y1).contains(&y) {
                    blocked[cy as usize * w + cx] = true;
                }
            }
        }
    }
    blocked
}

#[cfg(test)]
mod tests {
    use super::*;
    use dhruva_slam::algorithms::mapping::OccupancyGridConfig;

    /// A 1 m × 1 m grid at 5 cm centred on (0, 0), free below y = 0, unknown above.
    fn lower_half_free() -> OccupancyGrid {
        let mut grid = OccupancyGrid::new(OccupancyGridConfig {
            resolution: 0.05,
            initial_width: 1.0,
            initial_height: 1.0,
            ..Default::default()
        });
        assert_eq!(grid.origin(), (-0.5, -0.5));
        for (cx, cy) in (0..10).flat_map(|cy| (0..20).map(move |cx| (cx, cy))) {
            while grid.get_state(cx, cy) != CellState::Free {
                grid.update_cell(cx, cy, false);
            }
        }
        grid
    }

    /// The pixel of world point (x, y) in the map of [`lower_half_free`].
    fn at(map: &GrayImage, x: f32, y: f32) -> u8 {
        let cell = |v: f32| ((v + 0.5) / 0.05) as u32;
        map.get_pixel(cell(x), 19 - cell(y))[0]
    }

    #[test]
    fn the_bottom_image_row_is_the_lowest_y() {
        let map = render(&lower_half_free(), (0.0, -0.25), 0.1, &[]);
        assert_eq!(at(&map, 0.0, -0.25), FLOOR);
        assert_eq!(at(&map, 0.0, 0.25), WALL);
    }

    #[test]
    fn floor_reached_only_through_a_block_is_wall() {
        let block = [-0.05, -0.5, 0.05, 0.5];
        let map = render(&lower_half_free(), (-0.3, -0.25), 0.1, &[block]);
        assert_eq!(at(&map, -0.3, -0.25), FLOOR);
        assert_eq!(at(&map, 0.0, -0.25), WALL);
        assert_eq!(at(&map, 0.3, -0.25), WALL);
    }

    #[test]
    fn blocks_parse_in_either_corner_order() {
        assert_eq!(parse_block("1, 2,0,-1"), Ok([0.0, -1.0, 1.0, 2.0]));
        assert!(parse_block("1,2,3").is_err());
        assert!(parse_block("1,2,3,NaN").is_err());
    }
}
