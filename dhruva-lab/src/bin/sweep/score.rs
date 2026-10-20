//! What a sweep left unswept, from the true path and the v8's brushes:
//! - the side brush sweeps a 7.8 cm circle 10.5 cm ahead and 11 cm right of the centre;
//! - the main brush is a 16 cm strip across the centre.
//!
//! The main term is the floor left along the edges. Each edge is sampled every centimetre,
//! and each sample measures how far in from the wall the swept floor starts; their sum is
//! the area missed. Corners report the distance from the corner to the nearest swept floor.
//!
//! Pockets are unswept floor the path closed round, away from the room's sides, too small
//! for the body: inside a loop round a leg, say. Sweeping them later means coming back for
//! each, so they cost more than the open middle the lanes will sweep.

use serde_json::{Value, json};

use crate::run::Run;
use crate::scene::{Point, Scene, inside};

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

/// A run's score. Lower is better throughout.
#[derive(Debug, Clone, Default)]
pub struct Score {
    pub completed: bool,
    pub seconds: f32,
    pub bumps: usize,
    /// Floor left unswept along the edges (m²), each sample capped at `MAX_MISS` deep…
    pub missed_m2: f32,
    /// …and per metre of edge (m): the mean depth of the miss.
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
    /// Every edge sample: where it is, the inward normal, and how deep the miss is (m).
    pub misses: Vec<(Point, Point, f32)>,
}

impl Score {
    pub fn worst_corner(&self) -> f32 {
        self.corners.iter().copied().fold(0.0, f32::max)
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
    /// …and its floor.
    floor: Vec<bool>,
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

    /// The pockets' cells. Unswept floor is split into patches (4-connected) with the swept
    /// floor grown by `GAP` first, which closes narrow gaps in a loop and drops specks; a
    /// patch is a pocket if it does not touch the room's sides and has less area than the
    /// body (shrunk by `GAP` too). Its cells are the patch grown back by `GAP` over unswept
    /// floor.
    fn pockets(&self) -> Vec<usize> {
        let unswept = |k: usize| self.floor[k] && !self.cells[k];
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
                let (i, j) = (k % self.width, k / self.width);
                let neighbours = [
                    (i > 0).then(|| k - 1),
                    (i + 1 < self.width).then(|| k + 1),
                    (j > 0).then(|| k - self.width),
                    (j + 1 < self.height).then(|| k + self.width),
                ];
                for n in neighbours.into_iter().flatten() {
                    walled |= !self.room[n];
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

    /// How far from `p` along `dir` (unit) the swept floor starts, up to `MAX_MISS`.
    fn miss(&self, p: Point, dir: Point) -> f32 {
        let mut d = 0.0;
        while d < MAX_MISS {
            if self.swept((p.0 + d * dir.0, p.1 + d * dir.1)) {
                return d;
            }
            d += CELL / 2.0;
        }
        MAX_MISS
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
    let (mut missed, mut length) = (0.0, 0.0);
    let mut misses = Vec::new();
    for &((ax, ay), (bx, by)) in &edges {
        let len = (bx - ax).hypot(by - ay);
        let (ux, uy) = ((bx - ax) / len, (by - ay) / len);
        let inward = (-uy, ux);
        for i in 0..(len / EDGE_STEP) as usize {
            let s = (i as f32 + 0.5) * EDGE_STEP;
            let at = (ax + s * ux, ay + s * uy);
            let depth = swept.miss(at, inward);
            missed += depth * EDGE_STEP;
            misses.push((at, inward, depth));
        }
        length += len;
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
    fn loitering_is_the_longest_stay_within_a_metre() {
        let mut path = vec![(0.0, 0.0, 0.0); 500];
        path.extend(vec![(5.0, 0.0, 0.0); 200]);
        let (_, run) = square(path);
        assert!((loiter(&run) - 4.99).abs() < 1e-3, "{}", loiter(&run));
    }
}
