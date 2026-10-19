//! Coverage of the floor plan: room by room, each room as an edge pass and then its floor
//! section by section, so the robot finishes one area before it moves on and never crosses
//! what it has cleaned.
//!
//! Everything runs over the cells the robot's centre can stand on: the room's free space shrunk
//! by the robot radius and a margin.
//!
//! - **Edge passes** follow the outlines of that space, with the wall or furniture on the right,
//!   where the side brush sweeps the strip the round body cannot reach.
//! - **Lanes** are rows spaced by the main roller's width, along the plan's wall axis or across
//!   it, whichever needs fewer of them. They start one lane spacing in from the edge passes, so
//!   lanes and edges overlap like neighbouring lanes do.
//! - **Sections** are the boustrophedon cell decomposition of the lanes (Choset, "Coverage of
//!   known spaces: the boustrophedon cellular decomposition", 2000): consecutive lanes stay in
//!   one section while each has a single run overlapping the next; furniture splitting the
//!   lanes starts new sections, and so does a section growing wider than `max_section_width`.
//!   Each section is one zigzag path, lanes joined by U-turns.
//! - **Order**: rooms nearest first, in a room the edge passes, then the sections through the
//!   section adjacency graph, a neighbour of the last section first.
//!
//! Hops between the paths are left to the path planner; a part of a room it cannot reach
//! (behind a sofa) is skipped by the executor.

use std::f32::consts::FRAC_PI_2;

use image::{GrayImage, Luma};
use imageproc::contours::{BorderType, find_contours};
use imageproc::filter::gaussian_blur_f32;
use imageproc::geometry::approximate_polygon_dp;
use imageproc::gradients::{horizontal_sobel, vertical_sobel};
use imageproc::region_labelling::{Connectivity, connected_components};

use crate::algorithms::mapping::{CellState, OccupancyGrid, Rooms};
use crate::core::types::Point2D;

/// How close a section must end to a furniture loop for the robot to drive the loop next (m).
const FURNITURE_REACH: f32 = 1.0;

/// The robot's cleaning geometry.
#[derive(Debug, Clone, Copy)]
pub struct CoverageConfig {
    /// Robot radius (m).
    pub robot_radius: f32,
    /// Extra distance the body keeps from walls and furniture (m), for map and path error.
    pub margin: f32,
    /// Distance between neighbouring lanes (m): the main roller's width less an overlap.
    pub lane_spacing: f32,
    /// Widest section across its lanes (m).
    pub max_section_width: f32,
}

impl Default for CoverageConfig {
    fn default() -> Self {
        Self {
            robot_radius: 0.18,
            margin: 0.03,
            // A ~0.18 m roller (to measure on the unit) less 3 cm of overlap.
            lane_spacing: 0.15,
            max_section_width: 3.0,
        }
    }
}

/// What a path of a cleaning run cleans.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathKind {
    /// The room's outline: a closed loop, counter-clockwise, wall on the right.
    Wall,
    /// Around furniture: a closed loop, clockwise, furniture on the right.
    Furniture,
    /// A section's floor: a zigzag of lanes joined by U-turns.
    Lanes,
}

/// A path to drive, in plan coordinates (closed loops end at their first point).
#[derive(Debug, Clone, PartialEq)]
pub struct CoveragePath {
    pub kind: PathKind,
    pub points: Vec<Point2D>,
}

/// How to clean one room: its paths in driving order, each starting near where the one
/// before ended.
#[derive(Debug, Clone, PartialEq)]
pub struct RoomCoverage {
    pub room: u16,
    pub paths: Vec<CoveragePath>,
}

/// Plans the coverage of the rooms of one floor plan.
pub struct CoveragePlanner<'a> {
    grid: &'a OccupancyGrid,
    rooms: &'a Rooms,
    config: CoverageConfig,
    clearance: Vec<f32>,
    axis: f32,
}

impl<'a> CoveragePlanner<'a> {
    /// A planner for `rooms`, segmented from `grid`.
    pub fn new(grid: &'a OccupancyGrid, rooms: &'a Rooms, config: CoverageConfig) -> Self {
        let config = CoverageConfig {
            lane_spacing: config.lane_spacing.max(grid.resolution()),
            ..config
        };
        Self {
            grid,
            rooms,
            config,
            clearance: grid.clearance(),
            axis: wall_axis(grid),
        }
    }

    /// All rooms for a robot at `start`: the nearest room first (the one it is in), then
    /// depth first through the doorways, the nearest unvisited neighbour of the latest room.
    /// Only the floor the robot can drive to from `start` is planned: space seen only through
    /// a gap it does not fit through is left out, and so are rooms with nothing left.
    pub fn plan(&self, start: Point2D) -> Vec<RoomCoverage> {
        let reachable = (self.grid).reachable(&self.clearance, self.keep(0.0), (start.x, start.y));
        let neighbours: Vec<Vec<usize>> = (self.rooms.neighbours().into_iter())
            .map(|list| list.into_iter().map(|r| r as usize - 1).collect())
            .collect();
        let mut at = start;
        walk(&neighbours, |candidates| {
            let distances: Vec<f32> = (candidates.iter())
                .map(|&r| self.distance(r as u16 + 1, at, &reachable))
                .collect();
            let i = (0..candidates.len())
                .min_by(|&a, &b| distances[a].total_cmp(&distances[b]))
                .unwrap_or(0);
            let plan = self.room(candidates[i] as u16 + 1, at, &reachable);
            if let Some(end) = plan.paths.last().and_then(|p| p.points.last()) {
                at = *end;
            }
            (i, plan)
        })
        .into_iter()
        .filter(|plan| !plan.paths.is_empty())
        .collect()
    }

    /// The coverage of the `reachable` part of `room` for a robot at `start`: its outline,
    /// then its sections depth first through the section adjacency graph, each followed by the
    /// loops around furniture it ended near.
    fn room(&self, room: u16, start: Point2D, reachable: &[bool]) -> RoomCoverage {
        let mut paths = Vec::new();
        let mut at = start;
        let (mut walls, mut furniture) = self.loops(&self.mask(room, 0.0, reachable));
        paths.extend(drive_loops(&mut walls, PathKind::Wall, f32::MAX, &mut at));

        // Lanes start one lane spacing in from the loops.
        let inner = self.mask(room, self.config.lane_spacing, reachable);
        let [along, across] = [self.axis, self.axis + FRAC_PI_2].map(|a| self.lanes(&inner, a));
        let lanes = if across.run_count() < along.run_count() {
            across
        } else {
            along
        };
        let (sections, neighbours) = lanes.decompose(self.max_section_lanes());
        paths.extend(
            walk(&neighbours, |candidates| {
                let (i, corner) = (0..candidates.len())
                    .flat_map(|i| Corner::ALL.map(|c| (i, c)))
                    .min_by(|&(i, c), &(j, d)| {
                        let hop = |s: usize, c| lanes.corner(&sections[s], c).distance_squared(&at);
                        hop(candidates[i], c).total_cmp(&hop(candidates[j], d))
                    })
                    .expect("walk passes candidates");
                let points = lanes.zigzag(&sections[candidates[i]], corner);
                at = *points.last().expect("a section has a lane");
                let kind = PathKind::Lanes;
                let passed = drive_loops(
                    &mut furniture,
                    PathKind::Furniture,
                    FURNITURE_REACH,
                    &mut at,
                );
                (i, [vec![CoveragePath { kind, points }], passed].concat())
            })
            .into_iter()
            .flatten(),
        );
        paths.extend(drive_loops(
            &mut furniture,
            PathKind::Furniture,
            f32::MAX,
            &mut at,
        ));
        RoomCoverage { room, paths }
    }

    /// Distance from `at` to the nearest `reachable` cell of `room` (m), `f32::MAX` for none.
    fn distance(&self, room: u16, at: Point2D, reachable: &[bool]) -> f32 {
        let (w, h) = self.grid.dimensions();
        (0..w * h)
            .filter(|&i| reachable[i] && self.rooms.room(i % w, i / w) == Some(room))
            .map(|i| self.cell_centre(i).distance(&at))
            .fold(f32::MAX, f32::min)
    }

    fn cell_centre(&self, i: usize) -> Point2D {
        let w = self.grid.dimensions().0;
        let (x, y) = self.grid.cell_to_world(i % w, i / w);
        Point2D::new(x, y)
    }

    /// How far the robot's centre stays from anything not free, `inset` m more than its
    /// radius and margin.
    fn keep(&self, inset: f32) -> f32 {
        self.config.robot_radius + self.config.margin + inset
    }

    fn max_section_lanes(&self) -> usize {
        let lanes = self.config.max_section_width / self.config.lane_spacing;
        ((lanes + 1e-3) as usize).max(1)
    }

    /// The smallest area worth a path (m²).
    fn min_area(&self) -> f32 {
        self.config.lane_spacing.powi(2)
    }

    /// The `reachable` cells of `room` where the robot's centre keeps `inset` m more than the
    /// robot radius and margin from anything not free, without fragments smaller than
    /// `min_area`.
    fn mask(&self, room: u16, inset: f32, reachable: &[bool]) -> GrayImage {
        let (w, h) = self.grid.dimensions();
        let keep = self.keep(inset);
        let mask = GrayImage::from_fn(w as u32, h as u32, |x, y| {
            let (x, y) = (x as usize, y as usize);
            let i = y * w + x;
            let inside =
                reachable[i] && self.rooms.room(x, y) == Some(room) && self.clearance[i] > keep;
            Luma([if inside { 255 } else { 0 }])
        });
        let parts = connected_components(&mask, Connectivity::Eight, Luma([0]));
        let part_count = parts.pixels().map(|p| p[0] as usize).max().unwrap_or(0);
        let mut sizes = vec![0usize; part_count + 1];
        for p in parts.pixels() {
            sizes[p[0] as usize] += 1;
        }
        let min_cells = (self.min_area() / self.grid.resolution().powi(2)).ceil() as usize;
        GrayImage::from_fn(w as u32, h as u32, |x, y| {
            let part = parts.get_pixel(x, y)[0] as usize;
            Luma([if part > 0 && sizes[part] >= min_cells {
                255
            } else {
                0
            }])
        })
    }

    /// The outlines of `mask`: the outer ones counter-clockwise (walls), the inner ones
    /// clockwise (furniture).
    fn loops(&self, mask: &GrayImage) -> (Vec<Vec<Point2D>>, Vec<Vec<Point2D>>) {
        let (mut walls, mut furniture) = (Vec::new(), Vec::new());
        for contour in find_contours::<u32>(mask) {
            // Within half a cell of the contour, so the clearance holds along the chords.
            let corners = approximate_polygon_dp(&contour.points, 0.5, true);
            let mut ring: Vec<Point2D> = corners
                .iter()
                .map(|p| {
                    let (x, y) = self.grid.cell_to_world(p.x as usize, p.y as usize);
                    Point2D::new(x, y)
                })
                .collect();
            let area = signed_area(&ring);
            if area.abs() < self.min_area() {
                continue;
            }
            // The room lies left of its outline driven counter-clockwise; furniture lies right
            // of its outline driven clockwise.
            let outer = contour.border_type == BorderType::Outer;
            if (area > 0.0) != outer {
                ring.reverse();
            }
            if outer { &mut walls } else { &mut furniture }.push(ring);
        }
        (walls, furniture)
    }

    /// The runs of the lanes along `axis` over the cells of `mask`.
    fn lanes(&self, mask: &GrayImage, axis: f32) -> Lanes {
        let (cos, sin) = (axis.cos(), axis.sin());
        // The mask's extent along (u) and across (v) the lanes.
        let (mut u0, mut u1, mut v0, mut v1) = (f32::MAX, f32::MIN, f32::MAX, f32::MIN);
        for (x, y, p) in mask.enumerate_pixels() {
            if p[0] > 0 {
                let (wx, wy) = self.grid.cell_to_world(x as usize, y as usize);
                let (u, v) = (cos * wx + sin * wy, -sin * wx + cos * wy);
                (u0, u1, v0, v1) = (u0.min(u), u1.max(u), v0.min(v), v1.max(v));
            }
        }
        let (resolution, spacing) = (self.grid.resolution(), self.config.lane_spacing);
        let mut lanes = Lanes {
            cos,
            sin,
            first: 0.0,
            spacing,
            runs: Vec::new(),
        };
        if u0 > u1 {
            return lanes;
        }
        // The outermost lanes on the mask's edges, a lane spacing in from the loops, and the
        // ones between evenly spread, no farther apart than the spacing. Half a cell inside
        // the edges, so a lane along a slanted wall does not graze the steps of its cells.
        let v0 = v0 + resolution / 2.0;
        let v1 = (v1 - resolution / 2.0).max(v0);
        let gaps = ((v1 - v0) / spacing - 1e-3).ceil().max(0.0) as usize;
        let count = gaps + 1;
        lanes.first = v0;
        if gaps > 0 {
            lanes.spacing = (v1 - v0) / gaps as f32;
        }

        // Each lane is cut into runs of samples in the mask; a one-cell miss, where a lane
        // grazes the staircase of a slanted wall, does not cut it. Runs shorter than a lane
        // spacing are left to the edge passes.
        let step = resolution / 2.0;
        let inside = |u: f32, v: f32| {
            let p = lanes.world(u, v);
            self.grid
                .world_to_cell(p.x, p.y)
                .is_some_and(|(x, y)| mask.get_pixel(x as u32, y as u32)[0] > 0)
        };
        let runs = (0..count)
            .map(|k| {
                let v = lanes.v(k);
                let mut runs = Vec::new();
                let mut run: Option<(f32, f32)> = None;
                for i in 0..=((u1 - u0) / step).ceil() as usize {
                    let u = u0 + i as f32 * step;
                    if inside(u, v) {
                        run = Some((run.map_or(u, |r| r.0), u));
                    } else if run.is_some_and(|(_, b)| u - b > resolution) {
                        runs.extend(run.take());
                    }
                }
                runs.extend(run);
                runs.retain(|&(a, b)| b - a >= self.config.lane_spacing);
                runs
            })
            .collect();
        lanes.runs = runs;
        lanes
    }
}

/// Where a section's zigzag starts.
#[derive(Debug, Clone, Copy)]
struct Corner {
    /// From its last lane rather than its first.
    last_lane: bool,
    /// From the high-u end of that lane rather than the low one.
    high_end: bool,
}

impl Corner {
    const ALL: [Corner; 4] = [
        Corner::new(false, false),
        Corner::new(false, true),
        Corner::new(true, false),
        Corner::new(true, true),
    ];

    const fn new(last_lane: bool, high_end: bool) -> Self {
        Self {
            last_lane,
            high_end,
        }
    }
}

/// Parallel lanes in their own frame: u along a lane, v across; lane k at v = first + k spacing.
struct Lanes {
    cos: f32,
    sin: f32,
    first: f32,
    spacing: f32,
    /// Per lane, its runs (u from, u to) in increasing u.
    runs: Vec<Vec<(f32, f32)>>,
}

/// A section: one run on each of the consecutive lanes `first..first + runs.len()`.
struct Section {
    first: usize,
    runs: Vec<(f32, f32)>,
}

impl Lanes {
    fn v(&self, lane: usize) -> f32 {
        self.first + lane as f32 * self.spacing
    }

    fn world(&self, u: f32, v: f32) -> Point2D {
        Point2D::new(self.cos * u - self.sin * v, self.sin * u + self.cos * v)
    }

    fn run_count(&self) -> usize {
        self.runs.iter().map(Vec::len).sum()
    }

    /// Splits the runs into sections (Choset's boustrophedon cells, at most `max_lanes` wide)
    /// and finds which sections touch.
    fn decompose(&self, max_lanes: usize) -> (Vec<Section>, Vec<Vec<usize>>) {
        let overlap = |(a, b): (f32, f32), (c, d): (f32, f32)| a <= d && c <= b;
        let mut sections: Vec<Section> = Vec::new();
        let mut touching = Vec::new();
        let mut section_of: Vec<Vec<usize>> = Vec::with_capacity(self.runs.len());
        for (k, lane) in self.runs.iter().enumerate() {
            let previous: &[(f32, f32)] = if k > 0 { &self.runs[k - 1] } else { &[] };
            let mut row = Vec::with_capacity(lane.len());
            for &run in lane {
                let below: Vec<usize> = (0..previous.len())
                    .filter(|&p| overlap(previous[p], run))
                    .collect();
                // Continue the section below if it is the only one this run touches, this run
                // the only one it touches, and it is not full.
                let continued = match below[..] {
                    [p] => {
                        let section = section_of[k - 1][p];
                        let above = lane.iter().filter(|&&r| overlap(previous[p], r)).count();
                        (above == 1 && sections[section].runs.len() < max_lanes).then_some(section)
                    }
                    _ => None,
                };
                let section = continued.unwrap_or_else(|| {
                    sections.push(Section {
                        first: k,
                        runs: Vec::new(),
                    });
                    sections.len() - 1
                });
                sections[section].runs.push(run);
                for &p in &below {
                    if section_of[k - 1][p] != section {
                        touching.push((section_of[k - 1][p], section));
                    }
                }
                row.push(section);
            }
            section_of.push(row);
        }
        let mut neighbours = vec![Vec::new(); sections.len()];
        for (a, b) in touching {
            neighbours[a].push(b);
            neighbours[b].push(a);
        }
        for list in &mut neighbours {
            list.sort_unstable();
            list.dedup();
        }
        (sections, neighbours)
    }

    /// Where the section's zigzag from `corner` starts.
    fn corner(&self, section: &Section, corner: Corner) -> Point2D {
        let lane = if corner.last_lane {
            section.runs.len() - 1
        } else {
            0
        };
        let (a, b) = section.runs[lane];
        let u = if corner.high_end { b } else { a };
        self.world(u, self.v(section.first + lane))
    }

    /// The section's lanes as one path from `corner`, lanes alternating direction, joined by
    /// U-turns at the end of the shorter lane, so a turn stays over both lanes (the longer one
    /// is driven to its end and back).
    fn zigzag(&self, section: &Section, corner: Corner) -> Vec<Point2D> {
        let mut lanes: Vec<(f32, (f32, f32))> = section
            .runs
            .iter()
            .enumerate()
            .map(|(i, &run)| (self.v(section.first + i), run))
            .collect();
        if corner.last_lane {
            lanes.reverse();
        }
        let mut high = corner.high_end;
        let mut path: Vec<Point2D> = Vec::with_capacity(lanes.len() * 4);
        let mut push = |p: Point2D| {
            if path.last().is_none_or(|q| q.distance_squared(&p) > 1e-6) {
                path.push(p);
            }
        };
        for (i, &(v, (a, b))) in lanes.iter().enumerate() {
            let (from, to) = if high { (b, a) } else { (a, b) };
            if i > 0 {
                let (pv, (pa, pb)) = lanes[i - 1];
                let turn = if high { pb.min(b) } else { pa.max(a) };
                push(self.world(turn, pv));
                push(self.world(turn, v));
            }
            push(self.world(from, v));
            push(self.world(to, v));
            high = !high;
        }
        path
    }
}

/// The direction of the plan's walls (rad, in [-π/4, π/4]): the mean of the wall normals
/// folded onto a quarter turn, so perpendicular walls agree.
fn wall_axis(grid: &OccupancyGrid) -> f32 {
    let (w, h) = grid.dimensions();
    let walls = GrayImage::from_fn(w as u32, h as u32, |x, y| {
        let wall = grid.get_state(x as usize, y as usize) == CellState::Occupied;
        Luma([if wall { 255 } else { 0 }])
    });
    // Blurred first, so the steps of a slanted wall do not pull its normal to the grid.
    let walls = gaussian_blur_f32(&walls, 2.0);
    let (gx, gy) = (horizontal_sobel(&walls), vertical_sobel(&walls));
    let (mut re, mut im) = (0.0f64, 0.0f64);
    for (dx, dy) in gx.pixels().zip(gy.pixels()) {
        let (dx, dy) = (f64::from(dx[0]), f64::from(dy[0]));
        let phi = 4.0 * dy.atan2(dx);
        let weight = dx.hypot(dy);
        (re, im) = (re + weight * phi.cos(), im + weight * phi.sin());
    }
    (im.atan2(re) / 4.0) as f32
}

/// Visits every node of a graph (`neighbours` per node) depth first, the robot's way of
/// finishing an area before it moves on: the next node is picked by `choose` among the
/// unvisited neighbours of the latest visited node that has any, or among all unvisited nodes
/// when none has (the first pick, or a part not connected to the rest). `choose` returns the
/// index of its pick among the candidates and what to record for it.
fn walk<T>(neighbours: &[Vec<usize>], mut choose: impl FnMut(&[usize]) -> (usize, T)) -> Vec<T> {
    let n = neighbours.len();
    let mut done = vec![false; n];
    let mut visited: Vec<usize> = Vec::with_capacity(n);
    let mut picked = Vec::with_capacity(n);
    while visited.len() < n {
        let open = |node: &usize| !done[*node];
        let candidates: Vec<usize> = (visited.iter().rev())
            .map(|&v| {
                neighbours[v]
                    .iter()
                    .copied()
                    .filter(open)
                    .collect::<Vec<_>>()
            })
            .find(|c| !c.is_empty())
            .unwrap_or_else(|| (0..n).filter(open).collect());
        let (i, item) = choose(&candidates);
        done[candidates[i]] = true;
        visited.push(candidates[i]);
        picked.push(item);
    }
    picked
}

/// Drives the loops of `rings` that start within `reach` of the robot `at`, nearest first,
/// moving it to each loop's start (and end).
fn drive_loops(
    rings: &mut Vec<Vec<Point2D>>,
    kind: PathKind,
    reach: f32,
    at: &mut Point2D,
) -> Vec<CoveragePath> {
    let mut paths = Vec::new();
    while !rings.is_empty() {
        let starts: Vec<Point2D> = (rings.iter())
            .map(|r| r[nearest(r.iter().copied(), *at)])
            .collect();
        let i = nearest(starts.iter().copied(), *at);
        if starts[i].distance(at) > reach {
            break;
        }
        let points = closed_from(rings.swap_remove(i), *at);
        *at = points[0];
        paths.push(CoveragePath { kind, points });
    }
    paths
}

/// `ring` as a closed loop from its corner nearest `at`.
fn closed_from(mut ring: Vec<Point2D>, at: Point2D) -> Vec<Point2D> {
    let first = nearest(ring.iter().copied(), at);
    ring.rotate_left(first);
    ring.push(ring[0]);
    ring
}

/// Index of the point nearest `to` (0 if there are none).
fn nearest(points: impl IntoIterator<Item = Point2D>, to: Point2D) -> usize {
    points
        .into_iter()
        .map(|p| p.distance_squared(&to))
        .enumerate()
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map_or(0, |(i, _)| i)
}

/// Shoelace area, positive for a counter-clockwise ring.
fn signed_area(ring: &[Point2D]) -> f32 {
    let n = ring.len();
    (0..n)
        .map(|i| {
            let (p, q) = (ring[i], ring[(i + 1) % n]);
            p.x * q.y - q.x * p.y
        })
        .sum::<f32>()
        / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algorithms::mapping::{OccupancyGridConfig, RoomsConfig};

    const RES: f32 = 0.05;
    /// Robot radius plus margin.
    const KEEP: f32 = 0.21;
    const SPACING: f32 = 0.15;

    /// A `w` x `h` m grid from the world origin whose cells are walls, free or unknown by
    /// `cell` at their centres.
    fn grid(w: f32, h: f32, cell: impl Fn(f32, f32) -> Option<bool>) -> OccupancyGrid {
        let config = OccupancyGridConfig {
            resolution: RES,
            ..OccupancyGridConfig::default()
        };
        let (cw, ch) = ((w / RES).round() as usize, (h / RES).round() as usize);
        let mut grid = OccupancyGrid::from_raw(config, vec![0.0; cw * ch], cw, ch, 0.0, 0.0);
        for cy in 0..ch {
            for cx in 0..cw {
                let (x, y) = grid.cell_to_world(cx, cy);
                if let Some(occupied) = cell(x, y) {
                    for _ in 0..10 {
                        grid.update_cell(cx, cy, occupied);
                    }
                }
            }
        }
        grid
    }

    /// A room `w` x `h` m inside 0.1 m walls, 1 m in from the grid's corner, rotated by `angle`
    /// about its centre, with a `chair` m square at the centre (0 for none).
    fn room(w: f32, h: f32, angle: f32, chair: f32) -> OccupancyGrid {
        let (cx, cy) = (w / 2.0 + 1.0, h / 2.0 + 1.0);
        grid(w + 2.0, h + 2.0, |x, y| {
            let (dx, dy) = (x - cx, y - cy);
            let (u, v) = (
                angle.cos() * dx + angle.sin() * dy,
                -angle.sin() * dx + angle.cos() * dy,
            );
            let (au, av) = (u.abs(), v.abs());
            if au.max(av) < chair / 2.0 {
                Some(true)
            } else if au < w / 2.0 && av < h / 2.0 {
                Some(false)
            } else if au < w / 2.0 + 0.1 && av < h / 2.0 + 0.1 {
                Some(true)
            } else {
                None
            }
        })
    }

    fn cover(grid: &OccupancyGrid) -> RoomCoverage {
        let rooms = Rooms::segment(grid, RoomsConfig::default());
        assert_eq!(rooms.count(), 1);
        let mut plan = CoveragePlanner::new(grid, &rooms, CoverageConfig::default())
            .plan(Point2D::new(1.2, 1.2));
        assert_eq!(plan.len(), 1);
        plan.remove(0)
    }

    /// The points of the paths of `kinds`, in driving order.
    fn of(coverage: &RoomCoverage, kinds: &[PathKind]) -> Vec<Vec<Point2D>> {
        let paths = coverage.paths.iter().filter(|p| kinds.contains(&p.kind));
        paths.map(|p| p.points.clone()).collect()
    }

    fn sections(coverage: &RoomCoverage) -> Vec<Vec<Point2D>> {
        of(coverage, &[PathKind::Lanes])
    }

    fn edges(coverage: &RoomCoverage) -> Vec<Vec<Point2D>> {
        of(coverage, &[PathKind::Wall, PathKind::Furniture])
    }

    fn segments(path: &[Point2D]) -> impl Iterator<Item = (Point2D, Point2D)> + '_ {
        path.windows(2).map(|w| (w[0], w[1]))
    }

    /// Every cell the robot's centre can stand on is under the roller (0.18 m wide) on some
    /// path, give or take half a cell.
    fn assert_covered(grid: &OccupancyGrid, coverage: &RoomCoverage) {
        let clearance = grid.clearance();
        let (w, h) = grid.dimensions();
        for cy in 0..h {
            for cx in 0..w {
                if clearance[cy * w + cx] <= KEEP + RES {
                    continue;
                }
                let (x, y) = grid.cell_to_world(cx, cy);
                let p = Point2D::new(x, y);
                let gap = coverage
                    .paths
                    .iter()
                    .flat_map(|path| segments(&path.points))
                    .map(|(a, b)| segment_distance(p, a, b))
                    .fold(f32::MAX, f32::min);
                assert!(gap <= 0.09 + RES / 2.0, "({x}, {y}) is {gap} m off");
            }
        }
    }

    fn segment_distance(p: Point2D, a: Point2D, b: Point2D) -> f32 {
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let len2 = (dx * dx + dy * dy).max(1e-9);
        let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len2).clamp(0.0, 1.0);
        (p.x - a.x - t * dx).hypot(p.y - a.y - t * dy)
    }

    #[test]
    fn an_empty_room_is_its_edge_then_one_zigzag() {
        let grid = room(3.0, 2.0, 0.0, 0.0);
        let coverage = cover(&grid);
        assert_eq!(edges(&coverage).len(), 1);
        assert_eq!(sections(&coverage).len(), 1);
        // 1.2 m between the outermost lanes, about a lane spacing in from the edge pass: 9
        // lanes, each a straight followed by a U-turn of one lane spacing.
        let path = &sections(&coverage)[0];
        assert_eq!(path.len(), 2 * 9, "{path:?}");
        for (i, (a, b)) in segments(path).enumerate() {
            if i % 2 == 0 {
                assert!(
                    (a.y - b.y).abs() < 1e-3 && (a.x - b.x).abs() > 2.0,
                    "lane {a:?}"
                );
            } else {
                let turn = (b.y - a.y).abs();
                assert!((a.x - b.x).abs() < 1e-3 && turn <= SPACING + 1e-3, "{turn}");
            }
        }
        // The first lane is about a lane spacing from the edge pass, and starts where it
        // ended.
        let edge = &edges(&coverage)[0];
        let inset = path[0].y - edge[0].y;
        assert!((inset - SPACING).abs() <= RES, "{inset}");
        assert!(path[0].distance(&edge[0]) < 0.3);
        assert_covered(&grid, &coverage);
    }

    #[test]
    fn a_corridor_is_cleaned_along_its_length() {
        let grid = room(1.2, 4.0, 0.0, 0.0);
        let coverage = cover(&grid);
        for (a, b) in sections(&coverage).iter().flat_map(|p| segments(p)) {
            assert!((a.x - b.x).abs() < SPACING + 1e-3, "along y: {a:?}-{b:?}");
        }
        assert_covered(&grid, &coverage);
    }

    #[test]
    fn lanes_follow_rotated_walls() {
        let grid = room(3.0, 2.0, 0.3, 0.0);
        let axis = wall_axis(&grid);
        assert!((axis - 0.3).abs() < 0.03, "{axis}");
        let coverage = cover(&grid);
        for (a, b) in sections(&coverage).iter().flat_map(|p| segments(p)) {
            if a.distance(&b) > 0.5 {
                let heading = (b.y - a.y).atan2(b.x - a.x);
                let off = (heading - 0.3).rem_euclid(std::f32::consts::PI);
                assert!(off.min(std::f32::consts::PI - off) < 0.03, "{heading}");
            }
        }
        assert_covered(&grid, &coverage);
    }

    #[test]
    fn a_chair_splits_the_floor_into_sections_cleaned_side_by_side() {
        // Wide enough around the chair not to read its sides as doors.
        let grid = room(4.0, 3.0, 0.0, 0.4);
        let coverage = cover(&grid);
        // Below, beside left, beside right and above the chair.
        assert_eq!(sections(&coverage).len(), 4, "{:?}", sections(&coverage));
        // No path, turns included, runs into the chair grown by the robot (less a cell).
        let chair = |p: Point2D| (p.x - 3.0).abs().max((p.y - 2.5).abs()) < 0.2 + KEEP - RES;
        for (a, b) in sections(&coverage).iter().flat_map(|p| segments(p)) {
            for i in 0..=20 {
                let t = i as f32 / 20.0;
                let p = Point2D::new(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
                assert!(!chair(p), "{a:?}-{b:?} crosses the chair");
            }
        }
        // Ringing an obstacle needs one hop back across it at most.
        let hops: f32 = sections(&coverage)
            .windows(2)
            .map(|pair| pair[0].last().unwrap().distance(&pair[1][0]))
            .sum();
        assert!(hops < 3.0, "hops of {hops} m");
        // An edge pass circles the chair clockwise.
        let edges = edges(&coverage);
        assert_eq!(edges.len(), 2);
        let around = edges.iter().find(|r| signed_area(r) < 0.0).unwrap();
        for p in around {
            let to_chair = (p.x - 3.0).abs().max((p.y - 2.5).abs()) - 0.2;
            assert!((to_chair - KEEP).abs() < 0.06, "{p:?}");
        }
        assert_covered(&grid, &coverage);
    }

    #[test]
    fn a_large_room_is_cleaned_in_sections() {
        let grid = room(5.0, 7.0, 0.0, 0.0);
        let coverage = cover(&grid);
        // Lanes along the 7 m walls, 4.4 m across them: two sections of at most 3 m.
        assert_eq!(sections(&coverage).len(), 2);
        for path in &sections(&coverage) {
            let xs = path.iter().map(|p| p.x);
            let width = xs.clone().fold(f32::MIN, f32::max) - xs.fold(f32::MAX, f32::min);
            assert!(width <= 3.0, "{width}");
        }
        assert_covered(&grid, &coverage);
    }

    #[test]
    fn the_edge_pass_keeps_the_wall_on_the_right_at_robot_radius() {
        let grid = room(3.0, 2.0, 0.0, 0.0);
        let coverage = cover(&grid);
        let ring = &edges(&coverage)[0];
        assert!(signed_area(ring) > 0.0, "counter-clockwise");
        // The 4 corners of the room shrunk by the robot radius and margin, nearest first, and
        // back to the first.
        assert_eq!(ring.len(), 5, "{ring:?}");
        assert_eq!(ring[0], ring[4]);
        assert!(ring[0].distance(&Point2D::new(1.0 + KEEP, 1.0 + KEEP)) < 0.06);
        for p in ring {
            let to_wall = (p.x - 1.0).min(4.0 - p.x).min(p.y - 1.0).min(3.0 - p.y);
            assert!((to_wall - KEEP).abs() < 0.04, "{p:?}");
        }
    }

    /// Two 2 x 2 m rooms side by side, the left one from x = 1 to 3 m, joined by a `door` m
    /// wide opening.
    fn two_rooms(door: f32) -> OccupancyGrid {
        grid(6.0, 4.0, |x, y| {
            let (inside_x, inside_y) = ((1.0..5.0).contains(&x), (1.0..3.0).contains(&y));
            let opening = (2.0 - door / 2.0..2.0 + door / 2.0).contains(&y);
            let wall = (2.95..3.05).contains(&x) && !opening;
            match (inside_x && inside_y, wall) {
                (true, false) => Some(false),
                _ if (0.9..5.1).contains(&x) && (0.9..3.1).contains(&y) => Some(true),
                _ => None,
            }
        })
    }

    #[test]
    fn a_room_the_robot_cannot_get_into_is_left_out() {
        // A 0.3 m gap: the lidar sees through it, the robot does not fit.
        let grid = two_rooms(0.3);
        let rooms = Rooms::segment(&grid, RoomsConfig::default());
        assert_eq!(rooms.count(), 2, "the gap is a doorway to segmentation");
        let planner = CoveragePlanner::new(&grid, &rooms, CoverageConfig::default());
        let plan = planner.plan(Point2D::new(4.5, 2.0));
        assert_eq!(plan.len(), 1);
        for p in plan[0].paths.iter().flat_map(|path| &path.points) {
            assert!(p.x > 3.05, "{p:?} is past the wall with the gap");
        }
    }

    #[test]
    fn rooms_are_cleaned_one_at_a_time_and_stay_in_place() {
        let grid = two_rooms(0.8);
        let rooms = Rooms::segment(&grid, RoomsConfig::default());
        assert_eq!(rooms.count(), 2);
        let planner = CoveragePlanner::new(&grid, &rooms, CoverageConfig::default());
        let plan = planner.plan(Point2D::new(4.5, 2.0));
        assert_eq!(plan.len(), 2);
        let right = rooms.room(80, 40).unwrap();
        assert_eq!(plan[0].room, right, "the room the robot is in first");
        for room in &plan {
            let left = room.room != right;
            for p in room.paths.iter().flat_map(|path| &path.points) {
                assert_eq!(p.x < 3.0, left, "{p:?} is outside room {}", room.room);
            }
        }
    }

    #[test]
    fn the_walk_finishes_a_branch_before_backtracking_and_jumps_to_unconnected_parts() {
        // 0 - 1 - 2, 1 - 3, and 4 alone; always pick the highest candidate.
        let neighbours = vec![vec![1], vec![0, 2, 3], vec![1], vec![1], vec![]];
        let mut offered = Vec::new();
        let order = walk(&neighbours, |candidates| {
            offered.push(candidates.to_vec());
            let i = candidates.len() - 1;
            (i, candidates[i])
        });
        // 4 has no neighbours: any node; then down 3 - 1 - 2, and back from 2 through 1 to 0.
        assert_eq!(order, vec![4, 3, 1, 2, 0]);
        let expected: [&[usize]; 5] = [&[0, 1, 2, 3, 4], &[0, 1, 2, 3], &[1], &[0, 2], &[0]];
        assert_eq!(offered, expected);
    }
}
