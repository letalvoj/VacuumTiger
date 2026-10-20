//! The scenes the corner sweep is scored on, and their maps for the SangamIO mock: rooms
//! after what the real runs met (`captures/CATALOG.md`), and primitives, one move each.

use std::f32::consts::{FRAC_PI_2, PI};

use image::{GrayImage, Luma};
use sangamio::devices::mock::map_loader::SimulationMap;

pub type Point = (f32, f32);
/// A segment `(a, b)`. For an edge to sweep, the floor is on its left.
pub type Segment = (Point, Point);

/// Map cell (m): fine, since the mock's ranges come in steps of it. A cell is solid when
/// its centre is; one on the very edge of a leg is (`TIE`), so a leg spans whole cells
/// whatever the rounding, at most a cell wider than it is.
const RES: f32 = 0.01;
const TIE: f32 = 1e-4;
/// Wall around the room's bounding box in the map (m).
const MARGIN: f32 = 0.3;
/// A curtain's fabric is a band this thick (m)…
const FABRIC: f32 = 0.02;
/// …that folds every this far (m).
const FOLD_PITCH: f32 = 0.15;
/// Floor beside a fin (m): one lap sweeps a band about 26 cm wide (the side brush reaches
/// 18.5 cm to the wall's side, the main brush 8 cm to the other), so the pass along the
/// wall and the one along the fin overlap, and what is left unswept is the turn's. A wider
/// bay leaves a strip down its middle that one lap cannot sweep: the rooms kit scores that.
const BAY: f32 = 0.5;
/// The bay right of a fin is this much narrower (m): two alike, the lidar cannot tell them
/// apart from their corners, and SLAM jumps from one to the other.
const BAY_SKEW: f32 = 0.05;

/// A curtain: fabric hanging in folds behind `front` (towards its left), bulging up to
/// `depth` back. A lidar beam crossing it square is returned with probability `seen`
/// (one crossing the folds at a slant meets more fabric, and is returned more often); the
/// body drives through it.
#[derive(Debug, Clone, Copy)]
pub struct Curtain {
    pub front: Segment,
    pub depth: f32,
    pub seen: f32,
}

impl Curtain {
    /// Whether `p` is in the fabric.
    fn holds(&self, p: Point) -> bool {
        let ((ax, ay), (bx, by)) = self.front;
        let len = (bx - ax).hypot(by - ay);
        let u = ((bx - ax) / len, (by - ay) / len);
        let (dx, dy) = (p.0 - ax, p.1 - ay);
        let along = dx * u.0 + dy * u.1;
        let behind = dy * u.0 - dx * u.1;
        let fold = self.depth * 0.5 * (1.0 - (along * std::f32::consts::TAU / FOLD_PITCH).cos());
        (0.0..=len).contains(&along) && (behind - fold).abs() <= FABRIC / 2.0
    }

    /// The lidar mask's gray for the fabric: a beam crosses about `FABRIC / RES` cells of it.
    fn gray(&self) -> u8 {
        let opacity = 1.0 - (1.0 - self.seen).powf(RES / FABRIC);
        (255.0 * (1.0 - opacity)).round() as u8
    }
}

#[derive(Debug, Clone)]
pub struct Scene {
    pub name: &'static str,
    /// The floor's outline, counter-clockwise: its sides are the edges to sweep, and
    /// their joints the corners.
    pub room: Vec<Point>,
    /// Square legs: centre and side (m).
    pub legs: Vec<(Point, f32)>,
    /// Solid but not seen by the lidar: glass, or something below it.
    pub glass: Vec<Segment>,
    pub curtains: Vec<Curtain>,
    /// Where the robot starts: x, y (m), heading (rad).
    pub start: (f32, f32, f32),
    pub seconds: f32,
    /// People's legs per lidar turn (the mock's `lidar.noise.legs`), if not as `--sim` says.
    pub walkers: Option<f32>,
}

impl Scene {
    pub fn new(name: &'static str, room: &[Point], start: (f32, f32, f32), seconds: f32) -> Self {
        Self {
            name,
            room: room.to_vec(),
            legs: Vec::new(),
            glass: Vec::new(),
            curtains: Vec::new(),
            start,
            seconds,
            walkers: None,
        }
    }

    pub fn edges(&self) -> Vec<Segment> {
        polygon(&self.room)
    }

    /// Everything the lidar sees, as segments: the room's sides and the legs' outlines.
    pub fn walls(&self) -> Vec<Segment> {
        let legs = self.legs.iter().flat_map(|&((x, y), side)| {
            let h = side / 2.0;
            polygon(&[
                (x - h, y - h),
                (x + h, y - h),
                (x + h, y + h),
                (x - h, y + h),
            ])
        });
        self.edges().into_iter().chain(legs).collect()
    }

    /// The scene as the mock's map: wall outside the room, in the legs and along the
    /// glass; a lidar mask that is clear along the glass and gray in the curtains.
    pub fn map(&self) -> SimulationMap {
        let (xs, ys): (Vec<f32>, Vec<f32>) = self.room.iter().copied().unzip();
        let lo = |v: &[f32]| v.iter().copied().fold(f32::INFINITY, f32::min) - MARGIN;
        let hi = |v: &[f32]| v.iter().copied().fold(f32::NEG_INFINITY, f32::max) + MARGIN;
        let origin = (lo(&xs), lo(&ys));
        let w = ((hi(&xs) - origin.0) / RES).ceil() as u32;
        let h = ((hi(&ys) - origin.1) / RES).ceil() as u32;
        // Cell centres; image rows run from the top (largest y) down.
        let centre = |px: u32, py: u32| {
            (
                origin.0 + (px as f32 + 0.5) * RES,
                origin.1 + ((h - 1 - py) as f32 + 0.5) * RES,
            )
        };
        let occupancy = GrayImage::from_fn(w, h, |px, py| {
            Luma([if self.solid(centre(px, py)) { 0 } else { 255 }])
        });
        let mask = GrayImage::from_fn(w, h, |px, py| {
            let p = centre(px, py);
            let gray = if self.glass_at(p) {
                255
            } else if self.solid(p) {
                0
            } else {
                (self.curtains.iter().find(|c| c.holds(p))).map_or(255, Curtain::gray)
            };
            Luma([gray])
        });
        SimulationMap::from_image(occupancy, RES, origin).with_lidar_mask(mask)
    }

    /// Whether `p` is not floor: outside the room, in a leg or on the glass.
    pub fn solid(&self, p: Point) -> bool {
        !inside(&self.room, p)
            || self.glass_at(p)
            || (self.legs.iter()).any(|&(c, side)| {
                (p.0 - c.0).abs() <= side / 2.0 + TIE && (p.1 - c.1).abs() <= side / 2.0 + TIE
            })
    }

    /// Glass is the map cells it crosses or touches: one or two thick.
    fn glass_at(&self, p: Point) -> bool {
        self.glass.iter().any(|&g| distance_to_segment(p, g) < RES)
    }
}

/// A closed polygon's sides, `corners` in order.
pub fn polygon(corners: &[Point]) -> Vec<Segment> {
    (0..corners.len())
        .map(|i| (corners[i], corners[(i + 1) % corners.len()]))
        .collect()
}

pub fn distance_to_segment(p: Point, ((ax, ay), (bx, by)): Segment) -> f32 {
    let (dx, dy) = (bx - ax, by - ay);
    let t = (((p.0 - ax) * dx + (p.1 - ay) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
    (p.0 - ax - t * dx).hypot(p.1 - ay - t * dy)
}

/// Whether `p` is inside the polygon (even-odd rule).
pub fn inside(polygon: &[Point], p: Point) -> bool {
    let mut odd = false;
    for (a, b) in self::polygon(polygon) {
        if (a.1 > p.1) != (b.1 > p.1) && p.0 < a.0 + (p.1 - a.1) * (b.0 - a.0) / (b.1 - a.1) {
            odd = !odd;
        }
    }
    odd
}

const BOX: [Point; 4] = [(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (0.0, 2.0)];

/// A set of scenes.
#[derive(Clone, Copy, clap::ValueEnum)]
pub enum Kit {
    /// After the real runs: a whole lap each.
    Rooms,
    /// One move each, in a tiny room.
    Primitives,
}

impl Kit {
    pub fn scenes(self) -> Vec<Scene> {
        match self {
            Kit::Rooms => scenes(),
            Kit::Primitives => primitives(),
        }
    }
}

/// A fin `fin_w` wide and `fin_len` long (m) out of a room's bottom wall, `BAY` of floor
/// left of it and beyond its end, `BAY_SKEW` less right of it.
fn finned(fin_w: f32, fin_len: f32) -> Vec<Point> {
    let (a, b) = (BAY, BAY + fin_w);
    let (room_w, room_h) = (b + BAY - BAY_SKEW, fin_len + BAY);
    vec![
        (0.0, 0.0),
        (a, 0.0),
        (a, fin_len),
        (b, fin_len),
        (b, 0.0),
        (room_w, 0.0),
        (room_w, room_h),
        (0.0, room_h),
    ]
}

/// The moves of a sweep, one per scene, tuned one at a time: along a wall, round the end of
/// a thin one and of a thick one, round a leg; then along the wall with people about. No
/// one walks about but in `people`. Each starts by its bottom wall, facing along it.
pub fn primitives() -> Vec<Scene> {
    let quiet = |name, room: &[Point], start, seconds| Scene {
        walkers: Some(0.0),
        ..Scene::new(name, room, start, seconds)
    };
    let start = (0.35, 0.25, 0.0);
    let room = [(0.0, 0.0), (1.4, 0.0), (1.4, 1.0), (0.0, 1.0)];
    // Along 8 m of wall, a U-turn at each end (its corners are scored too). It starts in the
    // bottom left corner, so the lap's seam is there and not along the wall.
    let corridor = quiet(
        "wall",
        &[(0.0, 0.0), (8.0, 0.0), (8.0, 0.7), (0.0, 0.7)],
        (0.25, 0.22, 0.0),
        200.0,
    );
    let mut out = vec![corridor.clone()];

    // Round the end of a 2 cm partition (a desk side): a half turn about one point.
    // In the middle of the left bay, beside the fin, facing up along it.
    let in_bay = (BAY / 2.0, 0.3, FRAC_PI_2);
    out.push(quiet("fin_thin", &finned(0.02, 0.6), in_bay, 90.0));

    // Round a 40 cm block (a cabinet): two quarter turns, a short side between.
    out.push(quiet("fin_thick", &finned(0.4, 0.6), in_bay, 90.0));

    // A 3 cm leg too near the wall to pass behind: round it and back to the wall.
    let mut scene = quiet("leg", &room, start, 90.0);
    scene.legs.push(((0.7, 0.2), 0.03));
    out.push(scene);

    // A cabinet whose top overhangs its plinth by 4 cm: the lidar sees the plinth, the
    // shell meets the top. Bumps all along unless it is kept off.
    let cabinet = [
        (0.0, 0.0),
        (0.5, 0.0),
        (0.5, 0.4),
        (1.1, 0.4),
        (1.1, 0.0),
        (1.6, 0.0),
        (1.6, 1.0),
        (0.0, 1.0),
    ];
    let mut scene = quiet("cabinet", &cabinet, (0.3, 0.8, 0.0), 120.0);
    let lid = [(0.46, 0.0), (0.46, 0.44), (1.14, 0.44), (1.14, 0.0)];
    scene.glass.extend(lid.windows(2).map(|w| (w[0], w[1])));
    out.push(scene);

    // Chairs (39 x 37 cm outside, 2.5 cm legs) 14 cm off the bottom wall, as in sweep8:
    // the body cannot pass behind or between the legs, so round them as one thing. Bays
    // beside and above as by a fin.
    let chair = |x: f32| {
        [(0.0, 0.0), (0.365, 0.0), (0.0, 0.345), (0.365, 0.345)]
            .map(|(dx, dy)| ((x + dx, 0.15 + dy), 0.025))
    };
    let (left, top) = (BAY + 0.025, 0.52 + BAY);
    let room_to = |right: f32| [(0.0, 0.0), (right, 0.0), (right, top), (0.0, top)];
    let one = room_to(left + 0.39 + BAY - BAY_SKEW);
    let by_door = (0.3, top - 0.25, 0.0);
    let mut scene = quiet("lone_chair", &one, by_door, 120.0);
    scene.legs.extend(chair(left));
    out.push(scene);

    // The chair with a stretcher across its far side, below the lidar: seen open, bumped.
    let mut scene = quiet("chair_bar", &one, by_door, 120.0);
    scene.legs.extend(chair(left));
    scene.glass.push(((left + 0.365, 0.52), (left, 0.52)));
    out.push(scene);

    // Two chairs 22 cm apart: the gap is narrower than the body, round both as one.
    let next = left + 0.39 + 0.22;
    let two = room_to(next + 0.39 + BAY - BAY_SKEW);
    let mut scene = quiet("two_chairs", &two, by_door, 150.0);
    scene.legs.extend(chair(left));
    scene.legs.extend(chair(next));
    out.push(scene);

    let open = [(0.0, 0.0), (1.6, 0.0), (1.6, 1.3), (0.0, 1.3)];

    // A curtain 15 cm deep before a wall, its fabric mostly seen: into it to the wall.
    let mut scene = quiet("curtain_strip", &open, (1.3, 0.8, PI), 150.0);
    scene.curtains.push(Curtain {
        front: ((1.3, 0.18), (0.3, 0.18)),
        depth: 0.15,
        seen: 0.85,
    });
    out.push(scene);

    // The corridor with people walking past: not taking their legs for the wall.
    out.push(Scene {
        name: "people",
        walkers: Some(2.0),
        ..corridor
    });
    out
}

/// The scenes after the real runs, each a few metres and a few minutes.
pub fn scenes() -> Vec<Scene> {
    let start = (1.5, 1.0, 0.3);
    let mut out = Vec::new();

    // Walls and four inner corners.
    out.push(Scene::new("box", &BOX, start, 150.0));

    // An L: five inner corners and an outer one.
    let l = [
        (0.0, 0.0),
        (4.0, 0.0),
        (4.0, 2.0),
        (2.0, 2.0),
        (2.0, 3.0),
        (0.0, 3.0),
    ];
    out.push(Scene::new("l_room", &l, (1.5, 1.2, 0.3), 240.0));

    // Just undocked: the dock (35 x 12 cm) on the left wall behind, the robot 30 cm off it
    // facing away (sweeps 1, 4, 5 and 8 went wrong here).
    let dock = [
        (0.0, 0.0),
        (3.0, 0.0),
        (3.0, 2.0),
        (0.0, 2.0),
        (0.0, 1.18),
        (0.12, 1.18),
        (0.12, 0.83),
        (0.0, 0.83),
    ];
    out.push(Scene::new("dock", &dock, (0.6, 1.0, 0.0), 150.0));

    // sweep7: a curtain along the bottom wall, its folds 3-18 cm out, mostly seen by the
    // lidar; the robot drives through it up to the wall. A cupboard side ends it.
    let curtain = [
        (0.0, 0.0),
        (2.4, 0.0),
        (2.4, 0.3),
        (2.5, 0.3),
        (2.5, 0.0),
        (3.0, 0.0),
        (3.0, 2.0),
        (0.0, 2.0),
    ];
    let mut scene = Scene::new("curtain", &curtain, start, 150.0);
    scene.curtains.push(Curtain {
        front: ((2.4, 0.18), (0.4, 0.18)),
        depth: 0.15,
        seen: 0.85,
    });
    out.push(scene);

    // sweep6/7: legs 15 cm off the wall every half metre, too close to it to pass behind.
    let mut scene = Scene::new("legs_by_wall", &BOX, start, 150.0);
    scene.legs = (0..4)
        .map(|i| ((0.75 + 0.5 * i as f32, 0.15), 0.03))
        .collect();
    out.push(scene);

    // The dog's bench by the dock: 1.2 x 0.4 m on six 3 cm legs against the bottom wall,
    // high enough to drive under; the robot starts beside it.
    let mut scene = Scene::new("dog_bench", &BOX, (0.45, 0.6, -0.3), 200.0);
    for x in [0.9, 1.5, 2.1] {
        scene.legs.extend([((x, 0.03), 0.03), ((x, 0.4), 0.03)]);
    }
    out.push(scene);

    // sweep8: a chair (39 x 37 cm outside, 2.5 cm legs) 20 cm off the wall. Its legs are
    // 34 and 32 cm apart inside, too narrow for the 35 cm body: round it as one thing.
    let mut scene = Scene::new("chair", &BOX, (0.5, 1.2, 0.0), 200.0);
    for x in [1.3, 1.665] {
        scene.legs.extend([((x, 0.2), 0.025), ((x, 0.545), 0.025)]);
    }
    out.push(scene);

    // A wider chair, 42 cm inside between its legs, 20 cm off the wall: the body cannot
    // pass behind it but fits under it. Coming along the wall it enters from the left and
    // leaves on the right. With a second such chair 6 cm to the right and 20 cm farther
    // from the wall, its leg splits the way out into 18 cm gaps: a dead end to sweep and
    // back out of.
    let wide = |x: f32, y: f32| {
        [(0.0, 0.0), (0.445, 0.0), (0.0, 0.445), (0.445, 0.445)]
            .map(|(dx, dy)| ((x + dx, y + dy), 0.025))
    };
    let mut scene = Scene::new("chair_through", &BOX, (0.5, 1.2, 0.0), 200.0);
    scene.legs.extend(wide(1.3, 0.2125));
    out.push(scene);
    let mut scene = Scene::new("chair_dead_end", &BOX, (0.5, 1.2, 0.0), 200.0);
    scene.legs.extend(wide(1.3, 0.2125));
    scene.legs.extend(wide(1.3 + 0.445 + 0.06, 0.4125));
    out.push(scene);

    // sweep6: a round bed against the wall, a half disc of 35 cm radius: an outer curve
    // between two nooks.
    let mut bed = vec![(0.0, 0.0)];
    bed.extend((0..=12).map(|i| {
        let a = PI - i as f32 * PI / 12.0;
        (1.5 + 0.35 * a.cos(), 0.35 * a.sin())
    }));
    bed.extend([(3.0, 0.0), (3.0, 2.0), (0.0, 2.0)]);
    out.push(Scene::new("round_bed", &bed, start, 150.0));

    // A 0.6 m glass fin out of the left wall: felt, not seen.
    let mut scene = Scene::new("glass", &BOX, start, 200.0);
    scene.glass.push(((0.0, 1.2), (0.6, 1.2)));
    out.push(scene);

    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scene(name: &str) -> Scene {
        scenes().into_iter().find(|s| s.name == name).unwrap()
    }

    #[test]
    fn every_scene_is_named_once_wound_anticlockwise_and_starts_on_free_floor() {
        let all: Vec<Scene> = [Kit::Rooms, Kit::Primitives]
            .into_iter()
            .flat_map(Kit::scenes)
            .collect();
        for (i, s) in all.iter().enumerate() {
            // `--dump` names a run's file by its scene.
            assert!(
                all[..i].iter().all(|o| o.name != s.name),
                "{} twice",
                s.name
            );
            let area: f32 = (polygon(&s.room).iter())
                .map(|&((ax, ay), (bx, by))| ax * by - bx * ay)
                .sum();
            assert!(
                area > 0.0,
                "{}: the floor must be on the edges' left",
                s.name
            );
            // The body's rim, a little out: nothing solid under it.
            let (x, y, _) = s.start;
            let r = 0.175 + 0.01;
            for k in 0..64 {
                let a = k as f32 * PI / 32.0;
                let rim = (x + r * a.cos(), y + r * a.sin());
                assert!(
                    !s.solid((x, y)) && !s.solid(rim),
                    "{}: starts in a wall",
                    s.name
                );
            }
        }
    }

    #[test]
    fn walls_legs_and_glass_are_solid_and_the_room_is_free() {
        let map = scene("chair").map();
        assert!(!map.is_occupied(1.5, 1.0));
        assert!(map.is_occupied(1.5, -0.005), "beyond the bottom wall");
        // The leg at (1.3, 0.2) fills the 2 x 2 cells whose centres it covers.
        for (x, y) in [(1.296, 0.196), (1.304, 0.204)] {
            assert!(map.is_occupied(x, y), "a 2.5 cm leg at ({x}, {y})");
        }
        assert!(!map.is_occupied(1.316, 0.2), "next to it");

        let map = scene("glass").map();
        assert!(map.is_occupied(0.3, 1.2));
        assert_eq!(map.lidar_opacity(0.3, 1.2), 0.0, "glass is clear");
        assert_eq!(map.lidar_opacity(-0.1, 1.0), 1.0, "a wall is not");
    }

    #[test]
    fn a_curtain_is_free_and_stops_most_beams_crossing_it() {
        let scene = scene("curtain");
        let map = scene.map();
        // Down a column through the fabric: it is crossed once, about two cells deep.
        let x = 1.0;
        let pass: f32 = (0..30)
            .map(|i| 0.01 * i as f32 + 0.005)
            .inspect(|&y| assert!(!map.is_occupied(x, y)))
            .map(|y| 1.0 - map.lidar_opacity(x, y))
            .product();
        let seen = 1.0 - pass;
        assert!((seen - 0.85).abs() < 0.01, "seen {seen}");
    }
}
