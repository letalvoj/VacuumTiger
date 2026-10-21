//! What a sweep left unswept, from the true path and the v8's brushes:
//! - the side brush sweeps a 7.8 cm circle 10.5 cm ahead and 11 cm right of the centre;
//! - the main brush is a 16 cm strip across the centre.
//!
//! The main term is the floor left along the edges. Each edge is sampled every centimetre,
//! and each sample measures how far in from the wall the swept floor starts, past the floor
//! no brush can reach (from a pose the body fits at, at any yaw); their sum is the area
//! missed. A sample with no reachable floor within `MAX_MISS` is not scored. Corners report
//! the distance from the corner to the nearest swept floor.
//!
//! Pockets are reachable unswept floor the path closed round, away from the room's sides,
//! too small for the body: inside a loop round a leg, say. Sweeping them later means coming
//! back for each, so they cost more than the open middle the lanes will sweep.
//!
//! Besides, not in the cost: how much of the edge the pass reached (the share of the lap it
//! made), how fast (edge reached per minute: a run that ran out of time is not charged for
//! the time it would have needed), and how much of the middle it swept straying from the
//! edges, past the straight drive from the start to the nearest wall.

use serde_json::{Value, json};

use crate::run::Run;
use crate::scene::{Point, Scene, distance_to_segment, inside};

/// Side brush centre (robot frame, m; right is -y) and the radius it sweeps (m).
pub const SIDE_BRUSH: Point = (0.105, -0.11);
pub const SIDE_BRUSH_RADIUS: f32 = 0.078;
/// Main brush: a strip this half-wide (m) across the centre.
pub const MAIN_BRUSH_HALF: f32 = 0.08;

/// Swept-floor grid cell (m), and the deepest miss counted at an edge sample (m).
const CELL: f32 = 0.01;
const MAX_MISS: f32 = 0.3;
/// Edges are sampled this far apart (m)…
const EDGE_STEP: f32 = 0.01;
/// …and a corner's miss is measured from this far inside it along the bisector (m), where
/// the floor actually is.
const CORNER_INSET: f32 = 0.005;
/// Pockets: gaps in a loop up to twice this many cells wide (6 cm) are closed; a wider
/// one joins the pocket to the middle…
const GAP: usize = 3;
/// …and an enclosed patch with less area than the body's (radius, m) is one.
const BODY_RADIUS: f32 = 0.175;
/// A path that does not leave a circle this big (m) for long loiters.
const LOITER_RADIUS: f32 = 1.0;
/// An edge sample is reached when the swept floor starts this near it (m), past what no
/// brush reaches: the corners' allowance.
const REACHED: f32 = 0.05;
/// Floor farther than a body's width from anything solid (m) is the middle, not the edge.
const MIDDLE: f32 = 2.0 * BODY_RADIUS;

/// A run's score. Lower is better throughout, but for `lap_share` and `rate`.
#[derive(Debug, Clone, Default)]
pub struct Score {
    pub completed: bool,
    pub seconds: f32,
    pub bumps: usize,
    /// Floor left unswept along the edges (m²), each sample capped at `MAX_MISS` deep…
    pub missed_m2: f32,
    /// …and per metre of edge scored (m): the mean depth of the miss.
    pub missed_depth: f32,
    /// Distance from each inner corner to the nearest swept floor (m), in edge order.
    pub corners: Vec<f32>,
    /// The pockets' cells (centres)…
    pub pockets: Vec<Point>,
    /// …and their area (m²).
    pub pockets_m2: f32,
    /// The swept cells (centres), for the render.
    pub swept: Vec<Point>,
    /// The longest time the path stayed within `LOITER_RADIUS` of a point (s).
    pub loiter: f32,
    /// Every edge sample scored: where it is, the inward normal, and how deep the miss is (m).
    pub misses: Vec<(Point, Point, f32)>,
    /// The edge scored (m)…
    pub edge_m: f32,
    /// …and what of it was reached at some point in the run (m), not necessarily in one go.
    pub reached_m: f32,
    /// The middle (m²)…
    pub middle_m2: f32,
    /// …and what of it was swept, but for the drive from the start to the nearest wall (m²).
    pub stray_m2: f32,
}

impl Score {
    pub fn worst_corner(&self) -> f32 {
        self.corners.iter().copied().fold(0.0, f32::max)
    }

    /// The share of the edge reached: of the lap, how much it made.
    pub fn lap_share(&self) -> f32 {
        if self.edge_m > 0.0 {
            self.reached_m / self.edge_m
        } else {
            0.0
        }
    }

    /// The edge reached per minute (m/min).
    pub fn rate(&self) -> f32 {
        if self.seconds > 0.0 {
            self.reached_m / self.seconds * 60.0
        } else {
            0.0
        }
    }

    /// The share of the middle swept straying from the edges.
    pub fn stray_share(&self) -> f32 {
        if self.middle_m2 > 0.0 {
            self.stray_m2 / self.middle_m2
        } else {
            0.0
        }
    }
}

/// The floor the brushes swept, on a `CELL` grid. Only floor (cells whose centre is in the
/// room and not in a leg or the glass): a brush reaching past a wall sweeps nothing.
struct Swept {
    origin: Point,
    width: usize,
    height: usize,
    /// Cells in the room…
    room: Vec<bool>,
    /// …its floor…
    floor: Vec<bool>,
    /// …and the floor a brush can reach: from a pose the body fits at, joined to where the
    /// path went, at any yaw.
    reach: Vec<bool>,
    cells: Vec<bool>,
}

impl Swept {
    fn new(scene: &Scene, run: &Run) -> Self {
        let (xs, ys): (Vec<f32>, Vec<f32>) = scene.room.iter().copied().unzip();
        let min = |v: &[f32]| v.iter().copied().fold(f32::INFINITY, f32::min);
        let max = |v: &[f32]| v.iter().copied().fold(f32::NEG_INFINITY, f32::max);
        let margin = 0.5;
        let origin = (min(&xs) - margin, min(&ys) - margin);
        let width = ((max(&xs) - origin.0 + margin) / CELL) as usize + 1;
        let height = ((max(&ys) - origin.1 + margin) / CELL) as usize + 1;
        let mut swept = Self {
            origin,
            width,
            height,
            room: Vec::new(),
            floor: Vec::new(),
            reach: Vec::new(),
            cells: vec![false; width * height],
        };
        let centres: Vec<Point> = (0..width * height).map(|k| swept.centre(k)).collect();
        swept.room = centres.iter().map(|&p| inside(&scene.room, p)).collect();
        swept.floor = centres.iter().map(|&p| !scene.solid(p)).collect();
        for &(x, y, theta) in &run.path {
            let (s, c) = theta.sin_cos();
            let brush = (
                x + SIDE_BRUSH.0 * c - SIDE_BRUSH.1 * s,
                y + SIDE_BRUSH.0 * s + SIDE_BRUSH.1 * c,
            );
            swept.disc(brush, SIDE_BRUSH_RADIUS);
            let mut across = -MAIN_BRUSH_HALF;
            while across <= MAIN_BRUSH_HALF {
                swept.mark((x - across * s, y + across * c));
                across += CELL;
            }
        }
        swept.reach = if run.path.is_empty() {
            swept.floor.clone()
        } else {
            swept.reachable(&run.path)
        };
        swept
    }

    fn index(&self, p: Point) -> Option<usize> {
        let i = ((p.0 - self.origin.0) / CELL).floor();
        let j = ((p.1 - self.origin.1) / CELL).floor();
        (i >= 0.0 && j >= 0.0 && (i as usize) < self.width && (j as usize) < self.height)
            .then(|| j as usize * self.width + i as usize)
    }

    fn centre(&self, k: usize) -> Point {
        let at = |n: usize, o: f32| o + (n as f32 + 0.5) * CELL;
        (
            at(k % self.width, self.origin.0),
            at(k / self.width, self.origin.1),
        )
    }

    /// The cells within `r` cells of `k` (a square).
    fn around(&self, k: usize, r: usize) -> impl Iterator<Item = usize> + '_ {
        let (i, j) = (k % self.width, k / self.width);
        let is = i.saturating_sub(r)..=(i + r).min(self.width - 1);
        let js = j.saturating_sub(r)..=(j + r).min(self.height - 1);
        js.flat_map(move |j| is.clone().map(move |i| j * self.width + i))
    }

    /// The cells whose centres are within `r` (m) of `k`'s.
    fn within(&self, k: usize, r: f32) -> impl Iterator<Item = usize> + '_ {
        let c = self.centre(k);
        self.around(k, (r / CELL).ceil() as usize)
            .filter(move |&m| {
                let p = self.centre(m);
                (p.0 - c.0).hypot(p.1 - c.1) <= r
            })
    }

    /// The 4-connected neighbours of `k`.
    fn neighbours(&self, k: usize) -> impl Iterator<Item = usize> {
        let (i, j, w) = (k % self.width, k / self.width, self.width);
        [
            (i > 0).then(|| k - 1),
            (i + 1 < w).then(|| k + 1),
            (j > 0).then(|| k - w),
            (j + 1 < self.height).then(|| k + w),
        ]
        .into_iter()
        .flatten()
    }

    /// The pockets' cells. Reachable unswept floor is split into patches (4-connected) with
    /// the swept floor grown by `GAP` first, which closes narrow gaps in a loop and drops
    /// specks; a patch is a pocket if it touches neither the room's sides nor floor out of
    /// reach (the edge, which is scored as missed) and has less area than the body (shrunk
    /// by `GAP` too). Its cells are the patch grown back by `GAP` over unswept floor.
    fn pockets(&self) -> Vec<usize> {
        let unswept = |k: usize| self.reach[k] && !self.cells[k];
        let body = std::f32::consts::PI * (BODY_RADIUS - GAP as f32 * CELL).powi(2);
        let open: Vec<bool> = (0..self.cells.len())
            .map(|k| unswept(k) && !self.around(k, GAP).any(|n| self.cells[n]))
            .collect();
        let mut seen = vec![false; self.cells.len()];
        let mut pocket = vec![false; self.cells.len()];
        for start in 0..self.cells.len() {
            if seen[start] || !open[start] {
                continue;
            }
            seen[start] = true;
            let (mut patch, mut todo, mut walled) = (Vec::new(), vec![start], false);
            while let Some(k) = todo.pop() {
                patch.push(k);
                for n in self.neighbours(k) {
                    walled |= !self.room[n] || (self.floor[n] && !self.reach[n]);
                    if !seen[n] && open[n] {
                        seen[n] = true;
                        todo.push(n);
                    }
                }
            }
            if !walled && (patch.len() as f32) * CELL * CELL < body {
                for k in patch {
                    for n in self.around(k, GAP) {
                        pocket[n] |= unswept(n);
                    }
                }
            }
        }
        (0..pocket.len()).filter(|&k| pocket[k]).collect()
    }

    /// The floor a brush can reach: from the centres the body fits at (no solid cell within
    /// `BODY_RADIUS`) joined (4-connected) to where `path` went, at any yaw, and what was
    /// swept. Like `mark`, it reaches through a wall thin enough.
    fn reachable(&self, path: &[(f32, f32, f32)]) -> Vec<bool> {
        let n = self.cells.len();
        // Solid cells next to floor bar the centres within the body's radius.
        let mut barred = vec![false; n];
        for k in (0..n).filter(|&k| !self.floor[k]) {
            if self.around(k, 1).any(|m| self.floor[m]) {
                for m in self.within(k, BODY_RADIUS) {
                    barred[m] = true;
                }
            }
        }
        let mut joined = vec![false; n];
        let mut todo = Vec::new();
        for k in path.iter().filter_map(|&(x, y, _)| self.index((x, y))) {
            if self.floor[k] && !joined[k] {
                joined[k] = true;
                todo.push(k);
            }
        }
        while let Some(k) = todo.pop() {
            for m in self.neighbours(k) {
                if !joined[m] && self.floor[m] && !barred[m] {
                    joined[m] = true;
                    todo.push(m);
                }
            }
        }
        // A point `brush_reach` from a centre inside the joined region is as near to one on
        // its rim, so only the rim is grown.
        let far = brush_reach();
        let mut reach: Vec<bool> = (0..n).map(|k| joined[k] || self.cells[k]).collect();
        for k in (0..n).filter(|&k| joined[k]) {
            if self.around(k, 1).any(|m| !joined[m]) {
                for m in self.within(k, far) {
                    reach[m] |= self.floor[m];
                }
            }
        }
        reach
    }

    /// Each cell's distance to the nearest solid cell (m): a two-pass chamfer transform,
    /// diagonals at √2 cells, so exact along the axes and up to 8% long in between.
    fn clearance(&self) -> Vec<f32> {
        let (w, n) = (self.width, self.cells.len());
        let diag = CELL * std::f32::consts::SQRT_2;
        let mut d: Vec<f32> = (self.floor.iter())
            .map(|&f| if f { f32::INFINITY } else { 0.0 })
            .collect();
        // The neighbours before cell `k` in raster order.
        let before = |k: usize| {
            let i = k % w;
            [
                (i > 0).then(|| (k - 1, CELL)),
                (k >= w).then(|| (k - w, CELL)),
                (k >= w && i > 0).then(|| (k - w - 1, diag)),
                (k >= w && i + 1 < w).then(|| (k - w + 1, diag)),
            ]
        };
        // Forwards, then backwards: the same pass over the grid reversed (`n - 1 - k`).
        for reversed in [false, true] {
            let at = |k: usize| if reversed { n - 1 - k } else { k };
            for k in 0..n {
                for (m, step) in before(k).into_iter().flatten() {
                    d[at(k)] = d[at(k)].min(d[at(m)] + step);
                }
            }
        }
        d
    }

    /// The middle (floor more than `MIDDLE` from anything solid), and what of it was swept but
    /// for what the brushes reach driving straight from `start` to the nearest wall, up to
    /// where the body touches it (m²).
    fn stray(&self, start: Point) -> (f32, f32) {
        let clearance = self.clearance();
        let middle = |k: usize| clearance[k] > MIDDLE;
        let solid = (0..self.cells.len()).filter(|&k| !self.floor[k]);
        let dist = |k: usize| {
            let p = self.centre(k);
            (p.0 - start.0).hypot(p.1 - start.1)
        };
        let seek = solid.min_by(|&a, &b| dist(a).total_cmp(&dist(b))).map(|k| {
            let (wall, d) = (self.centre(k), dist(k));
            // Already against it: no drive, only what the brushes reach from the start.
            let t = if d > BODY_RADIUS {
                (d - BODY_RADIUS) / d
            } else {
                0.0
            };
            let stop = (
                start.0 + t * (wall.0 - start.0),
                start.1 + t * (wall.1 - start.1),
            );
            (start, stop)
        });
        let seeking = |p: Point| seek.is_some_and(|s| distance_to_segment(p, s) <= brush_reach());
        let n = self.cells.len();
        let area = |cells: usize| cells as f32 * CELL * CELL;
        let stray = (0..n).filter(|&k| self.cells[k] && middle(k) && !seeking(self.centre(k)));
        (
            area(stray.count()),
            area((0..n).filter(|&k| middle(k)).count()),
        )
    }

    fn mark(&mut self, p: Point) {
        if let Some(i) = self.index(p) {
            self.cells[i] |= self.floor[i];
        }
    }

    fn disc(&mut self, c: Point, r: f32) {
        let n = (r / CELL).ceil() as i32;
        for i in -n..=n {
            for j in -n..=n {
                let (dx, dy) = (i as f32 * CELL, j as f32 * CELL);
                if dx.hypot(dy) <= r {
                    self.mark((c.0 + dx, c.1 + dy));
                }
            }
        }
    }

    fn swept(&self, p: Point) -> bool {
        self.index(p).is_some_and(|i| self.cells[i])
    }

    /// How far from `p` along `dir` (unit) the first floor that `cells` holds lies, up to
    /// `MAX_MISS`.
    fn first(&self, cells: &[bool], p: Point, dir: Point) -> f32 {
        let mut d = 0.0;
        while d < MAX_MISS {
            let at = (p.0 + d * dir.0, p.1 + d * dir.1);
            if self.index(at).is_some_and(|i| cells[i]) {
                return d;
            }
            d += CELL / 2.0;
        }
        MAX_MISS
    }

    /// How deep the floor left unswept from `p` along `dir` (unit) is, past the floor no
    /// brush can reach; `None` with no reachable floor within `MAX_MISS`.
    fn miss(&self, p: Point, dir: Point) -> Option<f32> {
        let from = self.first(&self.reach, p, dir);
        (from < MAX_MISS).then(|| self.first(&self.cells, p, dir) - from)
    }

    /// Distance from `p` to the nearest swept cell, up to `MAX_MISS`.
    fn nearest(&self, p: Point) -> f32 {
        let n = (MAX_MISS / CELL) as i32;
        let mut best = MAX_MISS;
        for i in -n..=n {
            for j in -n..=n {
                let (dx, dy) = (i as f32 * CELL, j as f32 * CELL);
                let d = dx.hypot(dy);
                if d < best && self.swept((p.0 + dx, p.1 + dy)) {
                    best = d;
                }
            }
        }
        best
    }
}

/// How far from the centre the brushes reach at some yaw (m): the side brush's far rim.
fn brush_reach() -> f32 {
    SIDE_BRUSH.0.hypot(SIDE_BRUSH.1) + SIDE_BRUSH_RADIUS
}

/// The longest time (s) the path stayed within `LOITER_RADIUS` of where it was: a stay
/// starts when the path last got that far from the previous start.
fn loiter(run: &Run) -> f32 {
    let path = &run.path;
    let (mut longest, mut anchor) = (0, 0);
    for (i, p) in path.iter().enumerate() {
        let a = path[anchor];
        if (p.0 - a.0).hypot(p.1 - a.1) > LOITER_RADIUS {
            anchor = i;
        }
        longest = longest.max(i - anchor);
    }
    longest as f32 * run.dt
}

/// Scores `run` in `scene`.
pub fn score(scene: &Scene, run: &Run) -> Score {
    let swept = Swept::new(scene, run);
    let edges = scene.edges();
    let (mut missed, mut length, mut reached) = (0.0, 0.0, 0.0);
    let mut misses = Vec::new();
    for &((ax, ay), (bx, by)) in &edges {
        let len = (bx - ax).hypot(by - ay);
        let (ux, uy) = ((bx - ax) / len, (by - ay) / len);
        let inward = (-uy, ux);
        for i in 0..(len / EDGE_STEP) as usize {
            let s = (i as f32 + 0.5) * EDGE_STEP;
            let at = (ax + s * ux, ay + s * uy);
            let Some(depth) = swept.miss(at, inward) else {
                continue;
            };
            missed += depth * EDGE_STEP;
            length += EDGE_STEP;
            if depth <= REACHED {
                reached += EDGE_STEP;
            }
            misses.push((at, inward, depth));
        }
    }
    // A corner is where an edge ends and the next starts, turning left (inner).
    let mut corners = Vec::new();
    for (i, &(a, b)) in edges.iter().enumerate() {
        let (_, d) = edges[(i + 1) % edges.len()];
        let u = (b.0 - a.0, b.1 - a.1);
        let w = (d.0 - b.0, d.1 - b.1);
        if u.0 * w.1 - u.1 * w.0 <= 0.0 {
            continue;
        }
        let (lu, lw) = (u.0.hypot(u.1), w.0.hypot(w.1));
        let bisector = (w.0 / lw - u.0 / lu, w.1 / lw - u.1 / lu);
        let lb = bisector.0.hypot(bisector.1);
        let inside = (
            b.0 + CORNER_INSET * bisector.0 / lb,
            b.1 + CORNER_INSET * bisector.1 / lb,
        );
        corners.push(swept.nearest(inside));
    }
    let (stray_m2, middle_m2) =
        (run.path.first()).map_or((0.0, 0.0), |&(x, y, _)| swept.stray((x, y)));
    let pockets: Vec<Point> = (swept.pockets().into_iter())
        .map(|k| swept.centre(k))
        .collect();
    Score {
        swept: (0..swept.cells.len())
            .filter(|&k| swept.cells[k])
            .map(|k| swept.centre(k))
            .collect(),
        pockets_m2: pockets.len() as f32 * CELL * CELL,
        pockets,
        completed: run.completed,
        seconds: run.seconds(),
        bumps: run.bumps.len(),
        missed_m2: missed,
        missed_depth: if length > 0.0 { missed / length } else { 0.0 },
        corners,
        loiter: loiter(run),
        misses,
        edge_m: length,
        reached_m: reached,
        middle_m2,
        stray_m2,
    }
}

/// The run, its scene and score as JSON, for `tools/wf_sim_plot.py`.
pub fn to_json(scene: &Scene, run: &Run, score: &Score) -> Value {
    let curtains: Vec<_> = scene.curtains.iter().map(|c| c.front).collect();
    let path: Vec<_> = run.path.iter().step_by(10).collect();
    let misses: Vec<_> = (score.misses.iter())
        .map(|&((x, y), (nx, ny), d)| (x, y, nx, ny, d))
        .collect();
    json!({
        "name": scene.name,
        "walls": scene.walls(),
        "glass": scene.glass,
        "curtains": curtains,
        "edges": scene.edges(),
        "path": path,
        "belief": run.belief,
        "bumps": run.bumps,
        "reversing": run.reversing,
        "seen": run.seen,
        "completed": run.completed,
        "ended": run.ended,
        "seconds": score.seconds,
        "bump_count": score.bumps,
        "missed_m2": score.missed_m2,
        "missed_depth": score.missed_depth,
        "corners": score.corners,
        "misses": misses,
        "pockets": score.pockets,
        "pockets_m2": score.pockets_m2,
        "lap_share": score.lap_share(),
        "rate": score.rate(),
        "middle_m2": score.middle_m2,
        "stray_m2": score.stray_m2,
        "swept": score.swept,
        "side_brush": SIDE_BRUSH,
        "side_brush_radius": SIDE_BRUSH_RADIUS,
        "main_brush_half": MAIN_BRUSH_HALF,
    })
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;

    use super::*;

    fn room(corners: &[Point]) -> Scene {
        Scene::new("test", corners, (0.0, 0.0, 0.0), 0.0)
    }

    /// A 2 m square room and a run of `path`, 10 ms apart.
    fn square(path: Vec<(f32, f32, f32)>) -> (Scene, Run) {
        let scene = room(&[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]);
        let run = Run {
            path,
            dt: 0.01,
            ..Run::default()
        };
        (scene, run)
    }

    /// Straight along y = `y`, heading +x, from x = 0.3 to 1.7, a step per mm.
    fn along(y: f32) -> Vec<(f32, f32, f32)> {
        (300..=1700)
            .map(|mm| (mm as f32 / 1000.0, y, 0.0))
            .collect()
    }

    #[test]
    fn floor_no_pose_can_reach_is_not_missed() {
        // A 40 cm block 25 cm off the bottom wall: the body (35 cm) can't get under it, and
        // the brushes reach 23 cm, so the floor under its middle is out of reach.
        let (mut scene, run) = square(along(1.0));
        scene.legs.push(((1.0, 0.45), 0.4));
        let s = score(&scene, &run);
        let under = |x: f32| {
            (s.misses.iter())
                .find(|((mx, my), _, _)| *my == 0.0 && (mx - x).abs() < 0.006)
                .map(|m| m.2)
        };
        assert_eq!(under(1.0), None, "under the block: not scored");
        // The open wall is reachable and the pass along y = 1 never came near it.
        assert_eq!(under(0.2), Some(MAX_MISS));
    }

    #[test]
    fn a_path_starting_against_a_wall_still_scores_the_edges() {
        // Centre 10 cm off the bottom wall: closer than the body fits.
        let (scene, run) = square(along(0.1));
        let s = score(&scene, &run);
        assert!(s.missed_depth > 0.0, "the far walls are missed");
        assert!(s.misses.iter().any(|((_, y), _, _)| *y == 0.0));
    }

    #[test]
    fn a_pass_along_a_wall_leaves_the_strip_its_brushes_do_not_reach() {
        // Heading +x the side brush is on the right, toward y = 0: it reaches down to
        // y - 0.11 - 0.078 = 0.012, so 1.2 cm along the bottom wall is left.
        let (scene, run) = square(along(0.2));
        let s = score(&scene, &run);
        let bottom: Vec<f32> = (s.misses.iter())
            .filter(|((x, y), _, _)| *y == 0.0 && (0.5..1.5).contains(x))
            .map(|m| m.2)
            .collect();
        assert!(!bottom.is_empty());
        assert!(
            bottom.iter().all(|d| (d - 0.012).abs() <= CELL),
            "{bottom:?}"
        );
        // Corners in edge order: (2, 0), 13 cm from the side brush's last disc (centre
        // (1.805, 0.09), radius 7.8 cm), then (2, 2), (0, 2) and (0, 0), out of reach.
        assert!((s.corners[0] - 0.13).abs() < 0.01, "{:?}", s.corners);
        assert_eq!(s.corners[1..], [MAX_MISS; 3]);
    }

    #[test]
    fn the_brushes_sweep_nothing_beyond_a_wall() {
        // Hugging the bottom wall: the side brush would reach below y = 0.
        let (scene, run) = square(along(0.1));
        let swept = Swept::new(&scene, &run);
        assert!(swept.swept((1.0, 0.005)));
        assert!(!swept.swept((1.0, -0.005)));
    }

    #[test]
    fn only_inner_corners_are_scored() {
        // An L: five inner corners and one outer at (1, 1).
        let scene = room(&[
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 1.0),
            (1.0, 1.0),
            (1.0, 2.0),
            (0.0, 2.0),
        ]);
        let run = Run {
            dt: 0.01,
            ..Run::default()
        };
        assert_eq!(score(&scene, &run).corners.len(), 5);
    }

    #[test]
    fn a_loop_round_a_leg_leaves_a_pocket_and_the_middle_is_not_one() {
        // Clockwise 0.3 m round a 4 cm leg at (1, 1), the side brush towards it: its centre
        // runs hypot(0.3 - 0.11, 0.105) = 21.7 cm from the leg, so it reaches in to 13.9 cm
        // and a disc that big less the leg is left, about 590 cm² (570 on the grid). The rest
        // of the room is
        // the middle.
        let (scene, run) = round_a_leg(0.3);
        let s = score(&scene, &run);
        assert!((0.054..0.061).contains(&s.pockets_m2), "{}", s.pockets_m2);
        let near = |p: &Point| (p.0 - 1.0).hypot(p.1 - 1.0) < 0.14;
        assert!(s.pockets.iter().all(near));
    }

    #[test]
    fn a_narrow_gap_in_the_loop_still_leaves_a_pocket() {
        // A 2 cm channel cut through the swept ring, out to the middle.
        let (scene, run) = round_a_leg(0.3);
        let mut swept = Swept::new(&scene, &run);
        for k in 0..swept.cells.len() {
            let (x, y) = swept.centre(k);
            if x > 1.0 && (1.0..1.02).contains(&y) {
                swept.cells[k] = false;
            }
        }
        let area = swept.pockets().len() as f32 * CELL * CELL;
        assert!((0.054..0.062).contains(&area), "{area}");
    }

    #[test]
    fn a_loop_the_body_fits_in_leaves_no_pocket() {
        let (scene, run) = round_a_leg(0.6);
        assert_eq!(score(&scene, &run).pockets_m2, 0.0);
    }

    /// Clockwise round a 4 cm leg at (1, 1), `r` off, a step per mm.
    fn round_a_leg(r: f32) -> (Scene, Run) {
        let (mut scene, mut run) = square(Vec::new());
        scene.legs.push(((1.0, 1.0), 0.04));
        let steps = (2.0 * std::f32::consts::PI * r * 1000.0) as usize;
        run.path = (0..=steps)
            .map(|k| {
                let a = -(k as f32) / (r * 1000.0);
                (1.0 + r * a.cos(), 1.0 + r * a.sin(), a - FRAC_PI_2)
            })
            .collect();
        (scene, run)
    }

    #[test]
    fn a_pass_along_one_wall_makes_that_share_of_the_lap_and_strays_nothing() {
        // 1.4 m along the bottom. The side brush (centre 10.5 cm ahead, 9 cm off the wall)
        // sweeps to within `REACHED` of it 6.7 cm either side of its centre: from x = 0.34
        // to 1.87, 1.53 m of the 8 m round the room.
        let (scene, run) = square(along(0.2));
        let s = score(&scene, &run);
        assert!((s.reached_m - 1.53).abs() < 0.02, "{}", s.reached_m);
        assert!((s.edge_m - 8.0).abs() < 0.01, "{}", s.edge_m);
        assert!((s.rate() - s.reached_m / run.seconds() * 60.0).abs() < 1e-3);
        assert_eq!(s.stray_m2, 0.0);
    }

    #[test]
    fn a_pass_through_the_middle_strays_but_for_the_drive_to_the_nearest_wall() {
        // From (0.3, 1) the nearest wall is x = 0: that drive (to x = 0.175) and its brushes'
        // 23 cm reach is not counted, out to x = 0.44–0.53 across the swept strip. The main
        // brush's 16 cm and the side brush's 10.8 cm below it are, on to x = 1.65 (35 cm off
        // the far wall): about 0.27 × 1.17 m.
        let (scene, run) = square(along(1.0));
        let s = score(&scene, &run);
        assert!((s.stray_m2 - 0.31).abs() < 0.015, "{}", s.stray_m2);
        // The middle is the room less 35 cm all round: 1.3 m square.
        assert!((s.middle_m2 - 1.69).abs() < 0.03, "{}", s.middle_m2);
    }

    #[test]
    fn starting_against_a_wall_there_is_no_drive_to_it() {
        // 10 cm off the bottom wall, then straight up the middle at x = 1: what the brushes
        // reach from the start is not counted, the rest of the climb through the middle is.
        let mut path = vec![(1.0, 0.1, FRAC_PI_2)];
        path.extend((100..=1000).map(|mm| (1.0, mm as f32 / 1000.0, FRAC_PI_2)));
        let (scene, run) = square(path);
        let s = score(&scene, &run);
        assert!(s.stray_m2.is_finite() && s.stray_m2 > 0.1, "{}", s.stray_m2);
    }

    #[test]
    fn clearance_is_the_distance_to_the_nearest_solid() {
        let (scene, run) = square(along(1.0));
        let swept = Swept::new(&scene, &run);
        let clearance = swept.clearance();
        for (p, want) in [((1.0, 1.0), 1.0), ((0.3, 1.0), 0.3), ((0.3, 0.2), 0.2)] {
            let got = clearance[swept.index(p).unwrap()];
            assert!((got - want).abs() < 0.015, "{p:?}: {got}");
        }
    }

    #[test]
    fn loitering_is_the_longest_stay_within_a_metre() {
        let mut path = vec![(0.0, 0.0, 0.0); 500];
        path.extend(vec![(5.0, 0.0, 0.0); 200]);
        let (_, run) = square(path);
        assert!((loiter(&run) - 4.99).abs() < 1e-3, "{}", loiter(&run));
    }
}
