//! Wall following: the corner sweep (scout).
//!
//! Follows a wall on one side (the side brush's) into every corner until the robot is back
//! where the lap started. It does not hold a distance: it steers slightly into the wall so
//! that, were the wall straight, it would touch it again after `bump_every` metres. Every
//! touch puts the body against the wall, so the brush reaches the edge even where the lidar
//! is wrong about the distance (glass, mirrors, dark surfaces); the lidar only gives the
//! wall's direction. Obstacles are the latest scan plus remembered contacts (bumps and
//! cliffs), so an obstacle the lidar misses is followed like a wall once it has been touched.
//!
//! The control logic is written for a wall on the right; a wall on the left mirrors the
//! robot-frame inputs (y → -y) and the turn direction.

use std::collections::VecDeque;
use std::f32::consts::{FRAC_PI_4, TAU};

use serde::Deserialize;

use crate::algorithms::matching::Line2D;
use crate::core::math::angle_diff;
use crate::core::types::{Point2D, PointCloud2D, Pose2D};
use crate::state::CurrentMapData;

use super::strategy::{ExplorationAction, ExplorationStrategy, HazardEvent};

/// Obstacle closer than this to the front of the body blocks the way (m).
const FRONT_CLEARANCE: f32 = 0.01;
/// The wall-side window: from this far behind the centre…
const SIDE_BEHIND: f32 = 0.05;
/// …to this far ahead of it (m)…
const SIDE_AHEAD: f32 = 0.30;
/// …and sideways from this fraction of the body radius (so a wall ahead is not mistaken
/// for the side wall)…
const SIDE_INNER: f32 = 0.8;
/// …out to this far from the body (m).
const SIDE_SLACK: f32 = 0.25;
/// Turn rate per radian of heading error.
const K_ANGLE: f32 = 1.2;
/// Smallest angle into the wall (rad): already touching, it still leans in.
const MIN_APPROACH: f32 = 0.01;
/// Travel over which the robot aims to close the gap when the next touch is due (m).
const MIN_APPROACH_RUN: f32 = 0.3;
/// Farther off the wall than this (m), close in within `CLOSE_RUN` rather than when the
/// next touch is due.
const MAX_GAP: f32 = 0.03;
/// Travel over which a gap wider than `MAX_GAP` is closed (m).
const CLOSE_RUN: f32 = 0.5;
/// Contacts (bumps, cliffs) remembered as obstacle points. They are kept in the world frame
/// and not moved by loop closures; only the ones near the robot matter.
const MAX_CONTACTS: usize = 64;
/// Obstacle points within this distance of a contact describe what was hit (m).
const CONTACT_RADIUS: f32 = 0.25;
/// Turn range after a contact (rad, away from the wall).
const MIN_CONTACT_TURN: f32 = 0.05;
const MAX_CONTACT_TURN: f32 = 1.6;
/// Reverse this far after a contact (m) at this speed (m/s).
const BACKOFF_DISTANCE: f32 = 0.02;
const BACKOFF_SPEED: f32 = 0.08;
/// A turn in place is done within this angle (rad).
const TURN_TOLERANCE: f32 = 0.06;
/// Ticks after which a back-off gives up (it reverses blind: 1 s at 10 Hz, ≤ 8 cm)…
const MAX_BACKOFF_TICKS: u32 = 10;
/// …and a turn in place (stuck guard, 10 s).
const MAX_TURN_TICKS: u32 = 100;
/// Pose steps longer than this are relocalisation jumps, not travel (m).
const MAX_POSE_STEP: f32 = 0.1;

/// Which side the wall is kept on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum WallSide {
    Left,
    #[default]
    Right,
}

/// Wall-following configuration (`[exploration.wall_follow]`).
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(default)]
pub struct WallFollowConfig {
    /// Side the wall is kept on: the side brush's.
    pub side: WallSide,
    /// Robot body radius (m); taken from `[navigation] robot_radius`.
    #[serde(skip)]
    pub robot_radius: f32,
    /// Steer into the wall so that, were it straight, the robot touches it again after
    /// this much travel (m).
    pub bump_every: f32,
    /// Forward speed (m/s).
    pub speed: f32,
    /// Turn rate in corners (rad/s).
    pub turn_rate: f32,
    /// A lap is closed back within this distance of where following started (m)…
    pub lap_radius: f32,
    /// …after at least this much travel (m).
    pub lap_min_distance: f32,
    /// Stop after this much travel in total (m).
    pub max_distance: f32,
}

impl Default for WallFollowConfig {
    fn default() -> Self {
        Self {
            side: WallSide::Right,
            robot_radius: 0.17,
            bump_every: 2.5,
            speed: 0.15,
            turn_rate: 0.6,
            lap_radius: 0.3,
            lap_min_distance: 3.0,
            max_distance: 300.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    /// Drive to the nearest obstacle.
    Seek,
    /// Follow the wall; `blind` is how far the robot has turned (rad) since it last saw it.
    Follow {
        blind: f32,
    },
    /// Reverse from `from` after a contact, then turn away from the wall by `turn` (rad).
    BackOff {
        from: Pose2D,
        turn: f32,
        ticks: u32,
    },
    /// Turn in place to `heading`.
    Turn {
        heading: f32,
        ticks: u32,
    },
    Done,
}

/// A wall seen from the robot: `angle` is the heading (relative to the robot's) that runs
/// along it with the wall on the robot's right; `distance` is from the robot centre (m).
#[derive(Debug, Clone, Copy)]
struct WallLine {
    angle: f32,
    distance: f32,
}

/// The wall through `points` (robot frame), from a total least-squares line fit.
fn wall_line(points: &[Point2D]) -> Option<WallLine> {
    if points.len() < 3 {
        return None;
    }
    let line = Line2D::fit(points)?;
    // Along the line is (-b, a); the robot (origin) is on its left when c < 0.
    let (ux, uy) = if line.c < 0.0 {
        (-line.b, line.a)
    } else {
        (line.b, -line.a)
    };
    Some(WallLine {
        angle: uy.atan2(ux),
        distance: line.c.abs(),
    })
}

/// Corner sweep: follow walls into every corner and around the room.
pub struct WallFollower {
    config: WallFollowConfig,
    phase: Phase,
    /// Contact points (world frame), oldest first.
    contacts: VecDeque<Point2D>,
    last_pose: Option<Pose2D>,
    travelled: f32,
    /// `travelled` by which the robot means to touch the wall again.
    touch_due: f32,
    /// Where following started (world frame, not moved by loop closures) and the distance
    /// travelled by then.
    lap_start: Option<(Point2D, f32)>,
}

impl WallFollower {
    pub fn new(config: WallFollowConfig) -> Self {
        Self {
            config,
            phase: Phase::Seek,
            contacts: VecDeque::new(),
            last_pose: None,
            travelled: 0.0,
            touch_due: 0.0,
            lap_start: None,
        }
    }

    /// +1 for a wall on the right (the frame the logic is written in), -1 for the left.
    fn mirror(&self) -> f32 {
        match self.config.side {
            WallSide::Right => 1.0,
            WallSide::Left => -1.0,
        }
    }

    /// A world point in the (mirrored) robot frame.
    fn to_robot(&self, pose: &Pose2D, x: f32, y: f32) -> Point2D {
        let p = pose.inverse().compose(&Pose2D::new(x, y, 0.0));
        Point2D::new(p.x, self.mirror() * p.y)
    }

    /// Scan points and remembered contacts, in the (mirrored) robot frame.
    fn obstacles(&self, pose: &Pose2D, scan: &PointCloud2D) -> Vec<Point2D> {
        let m = self.mirror();
        let scan_points = scan
            .xs
            .iter()
            .zip(&scan.ys)
            .map(|(&x, &y)| Point2D::new(x, m * y));
        let contacts = self.contacts.iter().map(|c| self.to_robot(pose, c.x, c.y));
        scan_points.chain(contacts).collect()
    }

    /// Updates the distance travelled; returns the heading change since the last tick.
    fn track(&mut self, pose: &Pose2D) -> f32 {
        let turned = match self.last_pose {
            Some(last) => {
                let step = (pose.x - last.x).hypot(pose.y - last.y);
                if step < MAX_POSE_STEP {
                    self.travelled += step;
                }
                angle_diff(last.theta, pose.theta)
            }
            None => 0.0,
        };
        self.last_pose = Some(*pose);
        turned
    }

    fn front_blocked(&self, points: &[Point2D]) -> bool {
        let r = self.config.robot_radius;
        points
            .iter()
            .any(|p| p.x > 0.0 && p.x < r + FRONT_CLEARANCE && p.y.abs() < r)
    }

    fn side_wall(&self, points: &[Point2D]) -> Option<WallLine> {
        let r = self.config.robot_radius;
        let (inner, outer) = (-SIDE_INNER * r, -(r + SIDE_SLACK));
        let window: Vec<Point2D> = points
            .iter()
            .filter(|p| p.x > -SIDE_BEHIND && p.x < SIDE_AHEAD && p.y < inner && p.y > outer)
            .copied()
            .collect();
        wall_line(&window)
    }

    /// How far to turn away from the wall after touching something at `contact`: parallel
    /// to the obstacle when the lidar or earlier contacts saw it, else by where it was hit.
    fn turn_after_contact(points: &[Point2D], contact: Point2D) -> f32 {
        let near: Vec<Point2D> = points
            .iter()
            .filter(|p| p.distance(&contact) < CONTACT_RADIUS)
            .copied()
            .collect();
        let turn = match wall_line(&near) {
            Some(line) => line.angle,
            None => contact.y.atan2(contact.x) + FRAC_PI_4,
        };
        turn.clamp(MIN_CONTACT_TURN, MAX_CONTACT_TURN)
    }

    fn lap_closed(&self, pose: &Pose2D) -> bool {
        self.lap_start.is_some_and(|(start, at)| {
            self.travelled - at > self.config.lap_min_distance
                && (pose.x - start.x).hypot(pose.y - start.y) < self.config.lap_radius
        })
    }

    /// The (mirrored) velocity command for this tick, moving through phase transitions.
    fn step(&mut self, pose: &Pose2D, points: &[Point2D], turned: f32) -> (f32, f32, &'static str) {
        let cfg = self.config;
        loop {
            match self.phase {
                Phase::Seek => {
                    if self.front_blocked(points) || self.side_wall(points).is_some() {
                        self.phase = Phase::Follow { blind: 0.0 };
                        continue;
                    }
                    let nearest = points
                        .iter()
                        .min_by(|a, b| a.x.hypot(a.y).total_cmp(&b.x.hypot(b.y)));
                    return match nearest {
                        None => (0.0, 0.0, "waiting for the lidar"),
                        Some(p) => {
                            let bearing = p.y.atan2(p.x);
                            if bearing.abs() > 0.4 {
                                (
                                    0.0,
                                    cfg.turn_rate * bearing.signum(),
                                    "turning to the nearest wall",
                                )
                            } else {
                                (cfg.speed, K_ANGLE * bearing, "driving to the nearest wall")
                            }
                        }
                    };
                }
                Phase::Follow { blind } => {
                    // The lap starts where following first does (from Seek or a bump).
                    let here = (Point2D::new(pose.x, pose.y), self.travelled);
                    self.lap_start.get_or_insert(here);
                    if self.lap_closed(pose) {
                        self.phase = Phase::Done;
                        continue;
                    }
                    if self.front_blocked(points) {
                        self.phase = Phase::Follow { blind: 0.0 };
                        return (0.0, cfg.turn_rate, "inner corner");
                    }
                    if let Some(wall) = self.side_wall(points) {
                        self.phase = Phase::Follow { blind: 0.0 };
                        // Aim into the wall to touch it when the touch is due: the angle
                        // steepens as that point comes closer. Too far off, close in now.
                        let gap = (wall.distance - cfg.robot_radius).max(0.0);
                        let due = if gap > MAX_GAP {
                            CLOSE_RUN
                        } else {
                            self.touch_due - self.travelled
                        };
                        let run = due.max(MIN_APPROACH_RUN);
                        let approach = gap.atan2(run).max(MIN_APPROACH);
                        let angular = (K_ANGLE * (wall.angle - approach))
                            .clamp(-cfg.turn_rate, cfg.turn_rate);
                        return (cfg.speed, angular, "following the wall");
                    }
                    let blind = blind + turned.abs();
                    if blind > TAU {
                        // Circled without finding the wall again: look for one.
                        self.phase = Phase::Seek;
                        continue;
                    }
                    self.phase = Phase::Follow { blind };
                    let angular = (cfg.speed / cfg.robot_radius).min(cfg.turn_rate);
                    return (cfg.speed, -angular, "outer corner");
                }
                Phase::BackOff { from, turn, ticks } => {
                    let backed = (pose.x - from.x).hypot(pose.y - from.y);
                    if backed >= BACKOFF_DISTANCE || ticks >= MAX_BACKOFF_TICKS {
                        let heading = from.theta + self.mirror() * turn;
                        self.phase = Phase::Turn { heading, ticks: 0 };
                        continue;
                    }
                    self.phase = Phase::BackOff {
                        from,
                        turn,
                        ticks: ticks + 1,
                    };
                    return (-BACKOFF_SPEED, 0.0, "backing off a contact");
                }
                Phase::Turn { heading, ticks } => {
                    let remaining = self.mirror() * angle_diff(pose.theta, heading);
                    if remaining < TURN_TOLERANCE || ticks >= MAX_TURN_TICKS {
                        self.phase = Phase::Follow { blind: 0.0 };
                        continue;
                    }
                    self.phase = Phase::Turn {
                        heading,
                        ticks: ticks + 1,
                    };
                    let angular = (2.0 * remaining).clamp(0.2, cfg.turn_rate);
                    return (0.0, angular, "turning after a contact");
                }
                Phase::Done => return (0.0, 0.0, "lap closed"),
            }
        }
    }
}

impl ExplorationStrategy for WallFollower {
    fn name(&self) -> &'static str {
        "wall_follow"
    }

    fn next_action(
        &mut self,
        _map: &CurrentMapData,
        current_pose: &Pose2D,
        scan: &PointCloud2D,
        hazard: Option<&HazardEvent>,
    ) -> ExplorationAction {
        let turned = self.track(current_pose);
        let contact = hazard.map(|h| Point2D::new(h.world_position.x, h.world_position.y));
        if let Some(at) = contact {
            if self.contacts.len() == MAX_CONTACTS {
                self.contacts.pop_front();
            }
            self.contacts.push_back(at);
            self.touch_due = self.travelled + self.config.bump_every;
        }
        let points = self.obstacles(current_pose, scan);
        if let Some(at) = contact {
            self.phase = Phase::BackOff {
                from: *current_pose,
                turn: Self::turn_after_contact(&points, self.to_robot(current_pose, at.x, at.y)),
                ticks: 0,
            };
        }
        if self.travelled > self.config.max_distance {
            self.phase = Phase::Done;
        }

        let (linear, angular, reason) = self.step(current_pose, &points, turned);
        if self.phase == Phase::Done {
            return ExplorationAction::Complete;
        }
        ExplorationAction::Drive {
            linear,
            angular: self.mirror() * angular,
            reason,
        }
    }

    fn reset(&mut self) {
        *self = Self::new(self.config);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exploration::HazardType;

    fn no_map() -> CurrentMapData {
        CurrentMapData {
            map_id: String::new(),
            name: String::new(),
            resolution: 0.05,
            width: 0,
            height: 0,
            origin_x: 0.0,
            origin_y: 0.0,
            cells: Vec::new(),
            explored_area_m2: 0.0,
        }
    }

    /// Points along a robot-frame segment, every 2 cm.
    fn segment_scan(scan: &mut PointCloud2D, a: (f32, f32), b: (f32, f32)) {
        let n = ((b.0 - a.0).hypot(b.1 - a.1) / 0.02) as usize;
        for i in 0..=n {
            let t = i as f32 / n as f32;
            scan.push_xy(a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1));
        }
    }

    /// A follower in the middle of a lap, just after a touch.
    fn following(side: WallSide) -> WallFollower {
        let mut follower = WallFollower::new(WallFollowConfig {
            side,
            ..WallFollowConfig::default()
        });
        follower.phase = Phase::Follow { blind: 0.0 };
        follower.lap_start = Some((Point2D::new(100.0, 100.0), 0.0));
        follower.touch_due = follower.config.bump_every;
        follower
    }

    fn drive(action: ExplorationAction) -> (f32, f32) {
        match action {
            ExplorationAction::Drive {
                linear, angular, ..
            } => (linear, angular),
            other => panic!("expected Drive, got {other:?}"),
        }
    }

    fn tick(f: &mut WallFollower, scan: &PointCloud2D) -> (f32, f32) {
        drive(f.next_action(&no_map(), &Pose2D::identity(), scan, None))
    }

    #[test]
    fn wall_line_is_directed_with_the_robot_on_its_left() {
        let right = [(0.0, -0.3), (0.2, -0.3), (0.4, -0.3)].map(|(x, y)| Point2D::new(x, y));
        let line = wall_line(&right).unwrap();
        assert!(line.angle.abs() < 1e-4 && (line.distance - 0.3).abs() < 1e-4);
        // A wall straight ahead: run along it to the left.
        let ahead = [(0.3, -0.2), (0.3, 0.0), (0.3, 0.2)].map(|(x, y)| Point2D::new(x, y));
        let line = wall_line(&ahead).unwrap();
        assert!((line.angle - std::f32::consts::FRAC_PI_2).abs() < 1e-4);
    }

    #[test]
    fn leans_into_a_parallel_wall_to_touch_it_again() {
        let mut f = following(WallSide::Right);
        let r = f.config.robot_radius;
        let wall_at = |gap: f32| {
            let mut scan = PointCloud2D::new();
            segment_scan(&mut scan, (-0.5, -r - gap), (1.0, -r - gap));
            scan
        };
        // Close: aim to touch it when the next touch is due, here 0.6 m on.
        f.touch_due = 0.6;
        let (v, w) = tick(&mut f, &wall_at(0.02));
        let approach = 0.02f32.atan2(0.6);
        assert!(
            v > 0.0 && (w + K_ANGLE * approach).abs() < 1e-3,
            "v {v} w {w}"
        );
        // Too far off: close in now.
        let (_, w) = tick(&mut f, &wall_at(0.1));
        let approach = 0.1f32.atan2(CLOSE_RUN);
        assert!((w + K_ANGLE * approach).abs() < 1e-3, "w {w}");

        // Touching it, it still leans in a little.
        let mut touching = PointCloud2D::new();
        segment_scan(&mut touching, (-0.5, -r), (1.0, -r));
        let (_, w) = tick(&mut f, &touching);
        assert!(w < 0.0 && w > -0.05, "w {w}");
    }

    #[test]
    fn turns_away_in_an_inner_corner_and_around_an_outer_one() {
        let mut f = following(WallSide::Right);
        let mut corner = PointCloud2D::new();
        segment_scan(&mut corner, (-0.5, -0.18), (0.175, -0.18));
        segment_scan(&mut corner, (0.175, -0.18), (0.175, 1.0));
        let (v, w) = tick(&mut f, &corner);
        assert!(v == 0.0 && w > 0.0, "inner corner: v {v} w {w}");

        let mut ended = PointCloud2D::new();
        segment_scan(&mut ended, (-1.0, -0.2), (-0.2, -0.2));
        let (v, w) = tick(&mut f, &ended);
        assert!(v > 0.0 && w < 0.0, "outer corner: v {v} w {w}");
    }

    #[test]
    fn a_left_wall_mirrors_the_turns() {
        let mut f = following(WallSide::Left);
        let y = f.config.robot_radius + 0.1;
        let mut far = PointCloud2D::new();
        segment_scan(&mut far, (-0.5, y), (1.0, y));
        let (_, w) = tick(&mut f, &far);
        assert!(w > 0.0, "leans into a left wall: {w}");
    }

    #[test]
    fn a_bump_turns_parallel_to_what_the_lidar_saw_there() {
        // A wall ahead at 45 degrees, rising to the left; bumped with the right bumper.
        let mut scan = PointCloud2D::new();
        segment_scan(&mut scan, (0.4, -0.4), (-0.2, 0.6));
        let contact = Point2D::new(0.17 * 0.3f32.cos(), -0.17 * 0.3f32.sin());
        let points: Vec<Point2D> = scan.iter().collect();
        let turn = WallFollower::turn_after_contact(&points, contact);
        let along = (0.6f32 + 0.4).atan2(-0.2 - 0.4); // direction (-0.6, 1.0)
        assert!(
            (turn - along.min(MAX_CONTACT_TURN)).abs() < 0.05,
            "turn {turn}"
        );

        // Nothing seen there (glass): turn by where it was hit, more for the far side.
        let blind_right = WallFollower::turn_after_contact(&[], contact);
        let blind_left = WallFollower::turn_after_contact(&[], Point2D::new(contact.x, -contact.y));
        assert!(blind_right < blind_left);
    }

    #[test]
    fn closes_the_lap_back_at_the_start() {
        let mut f = WallFollower::new(WallFollowConfig::default());
        let mut scan = PointCloud2D::new();
        segment_scan(&mut scan, (-0.5, -0.2), (1.0, -0.2));
        let map = no_map();
        let start = Pose2D::identity();
        assert!(matches!(
            f.next_action(&map, &start, &scan, None),
            ExplorationAction::Drive { .. }
        ));
        f.travelled = 10.0;
        assert!(matches!(
            f.next_action(&map, &start, &scan, None),
            ExplorationAction::Complete
        ));
    }

    #[test]
    fn the_lap_starts_wherever_following_does() {
        // E.g. after bumping glass while seeking: Follow is entered from a turn, not Seek.
        let mut f = WallFollower::new(WallFollowConfig::default());
        f.phase = Phase::Follow { blind: 0.0 };
        let at = Pose2D::new(1.0, 2.0, 0.0);
        f.next_action(&no_map(), &at, &PointCloud2D::new(), None);
        assert_eq!(f.lap_start.map(|(p, _)| (p.x, p.y)), Some((1.0, 2.0)));
    }

    // ---- Closed-loop simulation: a unicycle in a polygon room, lidar by ray casting ----

    type Segment = ((f32, f32), (f32, f32));

    fn distance_to_segment(p: (f32, f32), s: Segment) -> f32 {
        let ((ax, ay), (bx, by)) = s;
        let (dx, dy) = (bx - ax, by - ay);
        let t = (((p.0 - ax) * dx + (p.1 - ay) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
        (p.0 - ax - t * dx).hypot(p.1 - ay - t * dy)
    }

    fn ray_hit(origin: (f32, f32), dir: (f32, f32), s: Segment) -> Option<f32> {
        let ((ax, ay), (bx, by)) = s;
        let (ex, ey) = (bx - ax, by - ay);
        let denom = dir.0 * ey - dir.1 * ex;
        if denom.abs() < 1e-9 {
            return None;
        }
        let (wx, wy) = (ax - origin.0, ay - origin.1);
        let t = (wx * ey - wy * ex) / denom;
        let u = (wx * dir.1 - wy * dir.0) / denom;
        (t > 0.0 && (0.0..=1.0).contains(&u)).then_some(t)
    }

    struct Run {
        completed: bool,
        bumps: usize,
        path: Vec<(f32, f32)>,
    }

    /// Runs the follower at 10 Hz. `walls` are seen by the lidar and solid; `glass` is
    /// solid only.
    fn simulate(
        side: WallSide,
        walls: &[Segment],
        glass: &[Segment],
        start: Pose2D,
        seconds: f32,
    ) -> Run {
        let config = WallFollowConfig {
            side,
            ..WallFollowConfig::default()
        };
        let r = config.robot_radius;
        let mut follower = WallFollower::new(config);
        let map = no_map();
        let solid: Vec<Segment> = walls.iter().chain(glass).copied().collect();
        let mut pose = start;
        let mut hazard = None;
        let mut run = Run {
            completed: false,
            bumps: 0,
            path: Vec::new(),
        };
        let dt = 0.1;
        for _ in 0..(seconds / dt) as usize {
            let mut scan = PointCloud2D::new();
            for i in 0..180 {
                let a = i as f32 * TAU / 180.0;
                let world = (pose.theta + a).sin_cos();
                let hit = walls
                    .iter()
                    .filter_map(|&s| ray_hit((pose.x, pose.y), (world.1, world.0), s))
                    .fold(f32::INFINITY, f32::min);
                if hit < 5.0 {
                    scan.push_xy(hit * a.cos(), hit * a.sin());
                }
            }
            let action = follower.next_action(&map, &pose, &scan, hazard.as_ref());
            hazard = None;
            let (v, w) = match action {
                ExplorationAction::Drive {
                    linear, angular, ..
                } => (linear, angular),
                ExplorationAction::Complete => {
                    run.completed = true;
                    break;
                }
                other => panic!("unexpected {other:?}"),
            };
            let theta = pose.theta + w * dt;
            let next = (pose.x + v * dt * theta.cos(), pose.y + v * dt * theta.sin());
            let clearance = solid
                .iter()
                .map(|&s| distance_to_segment(next, s))
                .fold(f32::INFINITY, f32::min);
            if clearance < r {
                // Blocked: the bumper on the side of the contact fires; the robot only turns.
                let closest = solid
                    .iter()
                    .min_by(|a, b| {
                        distance_to_segment(next, **a).total_cmp(&distance_to_segment(next, **b))
                    })
                    .unwrap();
                let ((ax, ay), (bx, by)) = *closest;
                let mid = ((ax + bx) / 2.0, (ay + by) / 2.0);
                let probe = (0..=20)
                    .map(|i| {
                        let t = i as f32 / 20.0;
                        (ax + t * (bx - ax), ay + t * (by - ay))
                    })
                    .min_by(|p, q| {
                        let d = |z: &(f32, f32)| (z.0 - next.0).hypot(z.1 - next.1);
                        d(p).total_cmp(&d(q))
                    })
                    .unwrap_or(mid);
                let bearing = angle_diff(theta, (probe.1 - pose.y).atan2(probe.0 - pose.x));
                let kind = if bearing > 0.15 {
                    HazardType::BumperLeft
                } else if bearing < -0.15 {
                    HazardType::BumperRight
                } else {
                    HazardType::BumperBoth
                };
                pose = Pose2D::new(pose.x, pose.y, theta);
                hazard = Some(HazardEvent::new(kind, &pose));
                run.bumps += 1;
            } else {
                pose = Pose2D::new(next.0, next.1, theta);
            }
            run.path.push((pose.x, pose.y));
        }
        run
    }

    /// An L-shaped room: 4 x 3 m with the top-right 2 x 1 m cut out.
    fn l_room() -> Vec<Segment> {
        let corners = [
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, 2.0),
            (2.0, 2.0),
            (2.0, 3.0),
            (0.0, 3.0),
        ];
        (0..corners.len())
            .map(|i| (corners[i], corners[(i + 1) % corners.len()]))
            .collect()
    }

    fn closest_approach(path: &[(f32, f32)], to: (f32, f32)) -> f32 {
        path.iter()
            .map(|p| (p.0 - to.0).hypot(p.1 - to.1))
            .fold(f32::INFINITY, f32::min)
    }

    /// Asserts a lap of the L room (mirrored by `m` = ±1 in y) hugged its walls and corners.
    fn assert_swept(run: &Run, m: f32) {
        let mirrored = |(x, y): (f32, f32)| (x, m * y);
        assert!(run.completed, "lap not closed");
        assert!(run.bumps <= 25, "{} bumps", run.bumps);
        // Along the walls the body stays within 2 cm of them (here it rides a few
        // millimetres off without quite triggering the simulated bumper).
        let walls: Vec<Segment> = l_room()
            .into_iter()
            .map(|(a, b)| (mirrored(a), mirrored(b)))
            .collect();
        let mut gaps: Vec<f32> = run
            .path
            .iter()
            .map(|&p| {
                walls
                    .iter()
                    .map(|&s| distance_to_segment(p, s))
                    .fold(f32::INFINITY, f32::min)
            })
            .filter(|&d| d < 0.4)
            .map(|d| d - WallFollowConfig::default().robot_radius)
            .collect();
        gaps.sort_by(f32::total_cmp);
        let median = gaps[gaps.len() / 2];
        assert!(median < 0.02, "median gap to the wall {median:.3} m");
        for corner in [(0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (2.0, 3.0), (0.0, 3.0)] {
            let d = closest_approach(&run.path, mirrored(corner));
            assert!(d < 0.26, "corner {corner:?} missed by {d:.2} m");
        }
        assert!(
            closest_approach(&run.path, mirrored((2.0, 2.0))) < 0.3,
            "outer corner cut"
        );
    }

    #[test]
    fn sweeps_every_corner_of_an_l_shaped_room() {
        let start = Pose2D::new(1.5, 1.2, 0.3);
        let run = simulate(WallSide::Right, &l_room(), &[], start, 300.0);
        assert_swept(&run, 1.0);
    }

    #[test]
    fn sweeps_a_mirrored_room_with_the_wall_on_the_left() {
        let room: Vec<Segment> = l_room()
            .into_iter()
            .map(|((ax, ay), (bx, by))| ((ax, -ay), (bx, -by)))
            .collect();
        let start = Pose2D::new(1.5, -1.2, -0.3);
        let run = simulate(WallSide::Left, &room, &[], start, 300.0);
        assert_swept(&run, -1.0);
    }

    #[test]
    fn follows_glass_the_lidar_cannot_see() {
        // A 0.6 m glass fin out of the left wall.
        let glass = [((0.0, 1.5), (0.6, 1.5))];
        let start = Pose2D::new(1.5, 1.2, 0.3);
        let run = simulate(WallSide::Right, &l_room(), &glass, start, 400.0);
        assert!(run.completed, "lap not closed after {} bumps", run.bumps);
        assert!(run.bumps >= 1, "never touched the glass");
        // It went around the fin's tip rather than giving up on the wall.
        assert!(closest_approach(&run.path, (0.6, 1.5)) < 0.35);
        assert!(
            closest_approach(&run.path, (0.0, 3.0)) < 0.3,
            "missed the corner past the glass"
        );
    }
}
