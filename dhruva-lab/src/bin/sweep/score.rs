//! What a sweep left unswept, from the true path and the v8's brushes:
//! - the side brush sweeps a 7.8 cm circle 10.5 cm ahead and 11 cm right of the centre;
//! - the main brush is a 16 cm strip across the centre.
//!
//! The main term is the floor left along the edges. Each edge is sampled every centimetre,
//! and each sample measures how far in from the wall the swept floor starts; their sum is
//! the area missed. Corners report the distance from the corner to the nearest swept floor.

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

/// The floor the brushes swept, on a `CELL` grid. Only the room's floor (cells whose centre
/// is in it): a brush reaching past a wall sweeps nothing.
struct Swept {
    origin: Point,
    width: usize,
    height: usize,
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
        let floor = (0..width * height)
            .map(|k| {
                let centre = |n: usize, o: f32| o + (n as f32 + 0.5) * CELL;
                inside(
                    &scene.room,
                    (centre(k % width, origin.0), centre(k / width, origin.1)),
                )
            })
            .collect();
        let mut swept = Self {
            origin,
            width,
            height,
            floor,
            cells: vec![false; width * height],
        };
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
    Score {
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

/// The run, its scene and score as JSON, for `tools/py/wf_sim_plot.py`.
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
        "completed": run.completed,
        "ended": run.ended,
        "seconds": score.seconds,
        "bump_count": score.bumps,
        "missed_m2": score.missed_m2,
        "missed_depth": score.missed_depth,
        "corners": score.corners,
        "misses": misses,
        "side_brush": SIDE_BRUSH,
        "side_brush_radius": SIDE_BRUSH_RADIUS,
        "main_brush_half": MAIN_BRUSH_HALF,
    })
}

#[cfg(test)]
mod tests {
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
    fn loitering_is_the_longest_stay_within_a_metre() {
        let mut path = vec![(0.0, 0.0, 0.0); 500];
        path.extend(vec![(5.0, 0.0, 0.0); 200]);
        let (_, run) = square(path);
        assert!((loiter(&run) - 4.99).abs() < 1e-3, "{}", loiter(&run));
    }
}
