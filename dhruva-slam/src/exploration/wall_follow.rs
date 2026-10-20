//! Wall following: the corner sweep (scout).
//!
//! Follows the edge of the room on one side (the side brush's) into every corner until the
//! robot is back where the lap started. The edge is the nearest obstacle point on that side
//! (or what blocks the way): the robot keeps it beside the body, `hover_gap` off, so one rule
//! runs along walls, turns in inner corners, and hugs outer corners and legs round (the point
//! followed is kept as it passes behind). From far away the same rule drives straight at the
//! nearest point on the wall side, which is how a sweep starts.
//!
//! The lidar steers; the bumper corrects. A bump on something the lidar sees backs off a
//! little and nudges away. What the bumper touches where the lidar sees nothing (glass) is a
//! wall ahead: kept as a short line across the contact for a while, it is turned alongside
//! and followed like a seen one; so are cliffs. A fence (virtual wall points, e.g. across a
//! room's doorways) is an edge too, though it is never touched. Contacts and the fence are in
//! the world frame of when they were made and do not move with loop closures.
//!
//! Known limit: a sweep that starts nearer a free-standing island than a wall follows the
//! island round and closes its lap there.
//!
//! Scenario tests (rooms, legs, chairs, curtains, glass) are the lab's sweep bench,
//! `dhruva-lab/src/bin/sweep`, closed loop through SangamIO's mock and the SLAM.
//!
//! The control logic is written for a wall on the right; a wall on the left mirrors the
//! robot-frame inputs (y → -y) and the turn direction.

use std::collections::VecDeque;
use std::f32::consts::FRAC_PI_2;

use serde::Deserialize;

use crate::core::math::angle_diff;
use crate::core::types::{Point2D, PointCloud2D, Pose2D};
use crate::state::CurrentMapData;

use super::strategy::{ExplorationAction, ExplorationStrategy, HazardEvent, HazardType};

/// Edge points are not behind the centre by more than this (m): the dock just left is.
const BEHIND: f32 = 0.05;
/// The edge point moves less than this per tick (m) while followed.
const EDGE_TRACK: f32 = 0.04;
/// The edge is the centroid of the points at most this much farther than the nearest (m),
/// about the lidar's range noise, and within this of it (m): one surface, not the next leg.
const EDGE_BAND: f32 = 0.02;
const EDGE_SPAN: f32 = 0.15;
/// Reverse after a bump at this speed (m/s).
const BACKOFF_SPEED: f32 = 0.08;
/// Contacts the lidar did not see (glass, cliffs) are obstacles for this much travel (m):
/// points every `CONTACT_STEP` (m) along a line across the contact, this far each side (m).
const CONTACT_KEEP: f32 = 1.5;
const CONTACT_STEP: f32 = 0.02;
const CONTACT_SPAN: f32 = 0.1;
/// A scan point this near a contact (m) is what was touched: the bumper's position is only
/// known to within its zone.
const EXPLAINED: f32 = 0.06;
/// Following has started once the body is this close to the edge (m).
const ON_EDGE: f32 = 0.05;
/// A turn in place is done within this angle (rad); it turns at `SPIN_GAIN` per radian
/// left (1/s), at least `MIN_SPIN` (rad/s).
const TURN_TOLERANCE: f32 = 0.06;
const SPIN_GAIN: f32 = 2.0;
const MIN_SPIN: f32 = 0.2;
/// Ticks after which a back-off gives up (blind: 1.5 s at 10 Hz)…
const MAX_BACKOFF_TICKS: u32 = 15;
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
    /// Forward speed (m/s).
    pub speed: f32,
    /// Turn rate on the spot (rad/s).
    pub turn_rate: f32,
    /// A lap is closed back within this distance of where following started (m)…
    pub lap_radius: f32,
    /// …after at least this much travel (m).
    pub lap_min_distance: f32,
    /// Stop after this much travel in total (m).
    pub max_distance: f32,
    /// The controller's gains, as tuned on the lab's sweep bench (`dhruva-lab/src/bin/sweep`).
    pub tuning: Tuning,
}

impl Default for WallFollowConfig {
    fn default() -> Self {
        Self {
            side: WallSide::Right,
            robot_radius: 0.17,
            speed: 0.17,
            turn_rate: 0.67,
            lap_radius: 0.3,
            lap_min_distance: 3.0,
            max_distance: 300.0,
            tuning: Tuning::default(),
        }
    }
}

/// How the edge is followed (`[exploration.wall_follow.tuning]`).
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(default)]
pub struct Tuning {
    /// Gap kept between the body and an edge the lidar sees where it is (m).
    pub hover_gap: f32,
    /// Bend towards the edge per metre of gap beyond the wanted one (rad/m)…
    pub k_gap: f32,
    /// …and at most this far away from it when too close (rad).
    pub max_bend_away: f32,
    /// Turn rate per radian of heading error while driving (1/s).
    pub k_turn: f32,
    /// A heading error above this (rad) is turned on the spot; below it the robot drives.
    pub spin_angle: f32,
    /// A point the body would hit within this much more travel (m) blocks the way and is the
    /// edge, on either side: a wall ahead takes over from the one alongside as the body
    /// reaches it, so the corner gets swept.
    pub front_look: f32,
    /// Reverse this far after a bump (m), room to turn on the spot…
    pub backoff_distance: f32,
    /// …then nudge away by this much (rad).
    pub nudge: f32,
}

impl Default for Tuning {
    fn default() -> Self {
        Self {
            hover_gap: 0.02,
            k_gap: 5.5,
            max_bend_away: 0.21,
            k_turn: 2.6,
            spin_angle: 0.45,
            front_look: 0.067,
            backoff_distance: 0.033,
            nudge: 0.6,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    /// Follow the edge (or drive to the nearest one).
    Edge,
    /// Reverse `distance` (m) from `from` after a bump, then turn by `turn` (rad, mirrored).
    BackOff {
        from: Pose2D,
        distance: f32,
        turn: f32,
        ticks: u32,
    },
    /// Turn in place to `heading`, then follow the edge.
    Turn {
        heading: f32,
        ticks: u32,
    },
    Done,
}

/// Corner sweep: follow the room's edge into every corner and around the room.
pub struct WallFollower {
    config: WallFollowConfig,
    phase: Phase,
    /// Contacts the lidar did not see (world frame) and `travelled` when made, oldest first.
    contacts: VecDeque<(Point2D, f32)>,
    last_pose: Option<Pose2D>,
    travelled: f32,
    /// Where following started (world frame, not moved by loop closures) and the distance
    /// travelled by then.
    lap_start: Option<(Point2D, f32)>,
    /// The edge point followed last tick (world frame).
    last_edge: Option<Point2D>,
    /// Virtual wall points (world frame), followed like real ones.
    fence: Vec<Point2D>,
}

impl WallFollower {
    pub fn new(config: WallFollowConfig) -> Self {
        Self::fenced(config, Vec::new())
    }

    /// A sweep that also follows the virtual wall through `fence` (world points), e.g. across
    /// the doorways of the room it sweeps.
    pub fn fenced(config: WallFollowConfig, fence: Vec<Point2D>) -> Self {
        Self {
            config,
            phase: Phase::Edge,
            contacts: VecDeque::new(),
            last_pose: None,
            travelled: 0.0,
            lap_start: None,
            last_edge: None,
            fence,
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
    fn to_robot(&self, pose: &Pose2D, p: &Point2D) -> Point2D {
        let p = pose.inverse().compose(&Pose2D::new(p.x, p.y, 0.0));
        Point2D::new(p.x, self.mirror() * p.y)
    }

    /// A (mirrored) robot-frame point in the world frame.
    fn to_world(&self, pose: &Pose2D, p: &Point2D) -> Point2D {
        let w = pose.compose(&Pose2D::new(p.x, self.mirror() * p.y, 0.0));
        Point2D::new(w.x, w.y)
    }

    /// Scan points in the (mirrored) robot frame.
    fn seen(&self, scan: &PointCloud2D) -> Vec<Point2D> {
        let m = self.mirror();
        (scan.xs.iter().zip(&scan.ys))
            .map(|(&x, &y)| Point2D::new(x, m * y))
            .collect()
    }

    /// Scan points, kept contacts and the fence, in the (mirrored) robot frame.
    fn obstacles(&self, pose: &Pose2D, scan: &PointCloud2D) -> Vec<Point2D> {
        let remembered = (self.contacts.iter().map(|(c, _)| c)).chain(&self.fence);
        let mut points = self.seen(scan);
        points.extend(remembered.map(|c| self.to_robot(pose, c)));
        points
    }

    /// Updates the distance travelled and forgets old contacts.
    fn track(&mut self, pose: &Pose2D) {
        if let Some(last) = self.last_pose {
            let step = (pose.x - last.x).hypot(pose.y - last.y);
            if step < MAX_POSE_STEP {
                self.travelled += step;
            }
        }
        self.last_pose = Some(*pose);
        while (self.contacts.front()).is_some_and(|&(_, at)| self.travelled - at > CONTACT_KEEP) {
            self.contacts.pop_front();
        }
    }

    /// Travel (m) before the body would hit `p` driving straight on, if it is in the way.
    fn free_travel(&self, p: &Point2D) -> Option<f32> {
        let r = self.config.robot_radius;
        (p.x > 0.0 && p.y.abs() < r).then(|| p.x - (r * r - p.y * p.y).sqrt())
    }

    /// The point that blocks the way within `front_look`, the nearest hit first.
    fn blocking(&self, points: &[Point2D]) -> Option<Point2D> {
        (points.iter())
            .filter_map(|p| self.free_travel(p).map(|s| (p, s)))
            .filter(|&(_, s)| s < self.config.tuning.front_look)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(p, _)| *p)
    }

    /// The edge point (robot frame, mirrored) and its distance from the centre: what blocks
    /// the way, else the nearest point on the wall side, not behind unless it is still the
    /// one followed (`last`, the previous edge point): an outer corner is hugged round.
    fn edge(&self, points: &[Point2D], last: Option<Point2D>) -> Option<(Point2D, f32)> {
        let near_last =
            |p: &Point2D| last.is_some_and(|l| (p.x - l.x).hypot(p.y - l.y) < EDGE_TRACK);
        let distance = |p: &&Point2D| p.x.hypot(p.y);
        let side = || {
            (points.iter())
                .filter(|p| (p.x > -BEHIND || near_last(p)) && p.y < 0.0)
                .min_by(|a, b| distance(a).total_cmp(&distance(b)))
                .copied()
        };
        let p = self.blocking(points).or_else(side)?;
        let c = Self::centroid_near(points, p);
        Some((c, c.x.hypot(c.y)))
    }

    /// The centroid of the points about as near as `p` and next to it. On a flat wall the
    /// nearest point's bearing jumps by tens of degrees with a centimetre of range noise. The
    /// near points around it lie symmetric about the foot of the perpendicular, so their
    /// centroid does not jump. `p` is one of `points`, so there is at least one.
    fn centroid_near(points: &[Point2D], p: Point2D) -> Point2D {
        let d = p.x.hypot(p.y);
        let near = (points.iter())
            .filter(|q| q.x.hypot(q.y) - d < EDGE_BAND && (q.x - p.x).hypot(q.y - p.y) < EDGE_SPAN);
        let (sx, sy, n) = near.fold((0.0, 0.0, 0), |(sx, sy, n), q| (sx + q.x, sy + q.y, n + 1));
        Point2D::new(sx / n as f32, sy / n as f32)
    }

    /// The turn rate on the spot for `error` (rad, mirrored): `SPIN_GAIN` per radian, at
    /// least `MIN_SPIN` so it does not stall, at most `turn_rate`.
    fn spin(&self, error: f32) -> f32 {
        let rate = (SPIN_GAIN * error.abs()).clamp(MIN_SPIN, self.config.turn_rate);
        rate * error.signum()
    }

    /// Turns by `error` (rad, mirrored): on the spot when it is large, else while driving.
    fn steer(&self, error: f32, reason: &'static str) -> (f32, f32, &'static str) {
        let cfg = self.config;
        if error.abs() > cfg.tuning.spin_angle {
            (0.0, self.spin(error), reason)
        } else {
            let angular = (cfg.tuning.k_turn * error).clamp(-cfg.turn_rate, cfg.turn_rate);
            (cfg.speed, angular, reason)
        }
    }

    /// Turns on the spot to `heading`; `None` once there.
    fn turn_to(&self, pose: &Pose2D, heading: f32) -> Option<(f32, f32, &'static str)> {
        let remaining = self.mirror() * angle_diff(pose.theta, heading);
        (remaining.abs() >= TURN_TOLERANCE)
            .then(|| (0.0, self.spin(remaining), "turning on the spot"))
    }

    fn lap_closed(&self, pose: &Pose2D) -> bool {
        self.lap_start.is_some_and(|(start, at)| {
            self.travelled - at > self.config.lap_min_distance
                && (pose.x - start.x).hypot(pose.y - start.y) < self.config.lap_radius
        })
    }

    /// Whether the scan has a point within `EXPLAINED` of the contact `at` (world frame).
    fn explains(&self, pose: &Pose2D, scan: &PointCloud2D, at: &Point2D) -> bool {
        let at = self.to_robot(pose, at);
        (self.seen(scan).iter()).any(|p| (p.x - at.x).hypot(p.y - at.y) < EXPLAINED)
    }

    /// Records a bump or cliff and starts backing off it.
    fn on_hazard(&mut self, hazard: &HazardEvent, pose: &Pose2D, scan: &PointCloud2D) {
        let at = Point2D::new(hazard.world_position.x, hazard.world_position.y);
        let nudge = self.config.tuning.nudge;
        let away_side = match self.config.side {
            WallSide::Right => HazardType::BumperLeft,
            WallSide::Left => HazardType::BumperRight,
        };
        let turn = if !hazard.hazard_type.is_cliff() && self.explains(pose, scan, &at) {
            // The lidar sees what was touched: nudge away from the wall, unless the bump is
            // on the side away from it.
            if hazard.hazard_type == away_side {
                -nudge
            } else {
                nudge
            }
        } else {
            // Glass, or a cliff: nothing the lidar sees is there. What the shell touched runs
            // along it there: a short line across the contact's bearing, a wall ahead, turned
            // alongside as one seen.
            let (s, c) = hazard.world_position.theta.sin_cos();
            let n = (CONTACT_SPAN / CONTACT_STEP).round() as i32;
            for k in -n..=n {
                let along = k as f32 * CONTACT_STEP;
                let p = Point2D::new(at.x - along * s, at.y + along * c);
                self.contacts.push_back((p, self.travelled));
            }
            let c = self.to_robot(pose, &at);
            c.y.atan2(c.x) + FRAC_PI_2
        };
        let here = (Point2D::new(pose.x, pose.y), self.travelled);
        self.lap_start.get_or_insert(here);
        self.phase = Phase::BackOff {
            from: *pose,
            distance: self.config.tuning.backoff_distance,
            turn,
            ticks: 0,
        };
    }

    /// The (mirrored) velocity command for this tick, moving through phase transitions.
    fn step(&mut self, pose: &Pose2D, points: &[Point2D]) -> (f32, f32, &'static str) {
        let cfg = self.config;
        let r = cfg.robot_radius;
        loop {
            match self.phase {
                Phase::Edge => {
                    if self.lap_closed(pose) {
                        self.phase = Phase::Done;
                        continue;
                    }
                    let last = self.last_edge.map(|w| self.to_robot(pose, &w));
                    let Some((p, d)) = self.edge(points, last) else {
                        self.last_edge = None;
                        if points.is_empty() {
                            return (0.0, 0.0, "waiting for the lidar");
                        }
                        return (cfg.speed, 0.0, "driving ahead to find a wall");
                    };
                    self.last_edge = Some(self.to_world(pose, &p));
                    let gap = d - r;
                    if gap < ON_EDGE {
                        let here = (Point2D::new(pose.x, pose.y), self.travelled);
                        self.lap_start.get_or_insert(here);
                    }
                    // Run along the edge point (it on the right), bent towards it when the
                    // gap is wider than wanted: from far away, straight at it.
                    let tuning = cfg.tuning;
                    let bend = (tuning.k_gap * (gap - tuning.hover_gap))
                        .clamp(-tuning.max_bend_away, FRAC_PI_2);
                    let error = p.y.atan2(p.x) + FRAC_PI_2 - bend;
                    if error > TURN_TOLERANCE && self.blocking(points).is_some() {
                        // In the way (a corner, a leg): one turn on the spot that puts it
                        // alongside, committed so it does not flip back to the old wall.
                        let heading = pose.theta + self.mirror() * error;
                        self.phase = Phase::Turn { heading, ticks: 0 };
                        continue;
                    }
                    let reason = if gap < ON_EDGE {
                        "following the wall"
                    } else {
                        "driving to the nearest wall"
                    };
                    return self.steer(error, reason);
                }
                Phase::BackOff {
                    from,
                    distance,
                    turn,
                    ticks,
                } => {
                    let backed = (pose.x - from.x).hypot(pose.y - from.y);
                    if backed >= distance || ticks >= MAX_BACKOFF_TICKS {
                        let heading = from.theta + self.mirror() * turn;
                        self.phase = Phase::Turn { heading, ticks: 0 };
                        continue;
                    }
                    self.phase = Phase::BackOff {
                        from,
                        distance,
                        turn,
                        ticks: ticks + 1,
                    };
                    return (-BACKOFF_SPEED, 0.0, "backing off a bump");
                }
                Phase::Turn { heading, ticks } => match self.turn_to(pose, heading) {
                    Some(command) if ticks < MAX_TURN_TICKS => {
                        self.phase = Phase::Turn {
                            heading,
                            ticks: ticks + 1,
                        };
                        return command;
                    }
                    _ => self.phase = Phase::Edge,
                },
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
        self.track(current_pose);
        let before = self.phase;
        if let Some(h) = hazard {
            self.on_hazard(h, current_pose, scan);
        }
        if self.travelled > self.config.max_distance {
            self.phase = Phase::Done;
        }
        let points = self.obstacles(current_pose, scan);
        let (linear, angular, reason) = self.step(current_pose, &points);
        if std::mem::discriminant(&self.phase) != std::mem::discriminant(&before) {
            log::debug!(
                "wall_follow at ({:.2}, {:.2}, {:.2}) travelled {:.2}: {:?} -> {:?} ({})",
                current_pose.x,
                current_pose.y,
                current_pose.theta,
                self.travelled,
                before,
                self.phase,
                reason
            );
        }
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
        *self = Self::fenced(self.config, std::mem::take(&mut self.fence));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

    /// A follower in the middle of a lap.
    fn following(side: WallSide) -> WallFollower {
        let mut follower = WallFollower::new(WallFollowConfig {
            side,
            ..WallFollowConfig::default()
        });
        follower.lap_start = Some((Point2D::new(100.0, 100.0), 0.0));
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

    fn wall_at(y: f32) -> PointCloud2D {
        let mut scan = PointCloud2D::new();
        segment_scan(&mut scan, (-0.5, y), (1.0, y));
        scan
    }

    #[test]
    fn runs_along_a_wall_at_the_hover_gap() {
        let mut f = following(WallSide::Right);
        let r = f.config.robot_radius;
        let (v, w) = tick(&mut f, &wall_at(-r - Tuning::default().hover_gap));
        assert!(v > 0.0 && w.abs() < 0.02, "v {v} w {w}");
        // A little off: driving on, it bends in.
        let (v, w) = tick(&mut f, &wall_at(-r - 0.05));
        assert!(v > 0.0 && w < 0.0, "v {v} w {w}");
        // Into it: it turns out on the spot.
        let (v, w) = tick(&mut f, &wall_at(-r + 0.01));
        assert!(v == 0.0 && w > 0.0, "v {v} w {w}");
    }

    #[test]
    fn a_fence_is_followed_like_a_wall() {
        let r = WallFollowConfig::default().robot_radius;
        let wall = wall_at(-r - 0.05);
        let fence = (wall.xs.iter().zip(&wall.ys)).map(|(&x, &y)| Point2D::new(x, y));
        let mut fenced = WallFollower::fenced(WallFollowConfig::default(), fence.collect());
        fenced.lap_start = following(WallSide::Right).lap_start;
        let along_fence = tick(&mut fenced, &PointCloud2D::new());
        let along_wall = tick(&mut following(WallSide::Right), &wall);
        assert_eq!(along_fence, along_wall);
        fenced.reset();
        assert_eq!(fenced.fence.len(), wall.xs.len(), "a reset keeps the fence");
    }

    #[test]
    fn turns_on_the_spot_in_an_inner_corner_and_hugs_an_outer_one() {
        let mut f = following(WallSide::Right);
        let r = f.config.robot_radius;
        let mut corner = wall_at(-r - Tuning::default().hover_gap);
        segment_scan(&mut corner, (r + 0.02, -0.5), (r + 0.02, 1.0));
        let (v, w) = tick(&mut f, &corner);
        assert!(v == 0.0 && w > 0.0, "inner corner: v {v} w {w}");

        // The wall just followed ends just behind the body: round its end, towards it.
        let mut f = following(WallSide::Right);
        let mut ended = PointCloud2D::new();
        segment_scan(
            &mut ended,
            (-1.0, -r - Tuning::default().hover_gap),
            (-0.03, -r - Tuning::default().hover_gap),
        );
        let (_, w) = tick(&mut f, &ended);
        assert!(w < 0.0, "outer corner: w {w}");
    }

    #[test]
    fn a_left_wall_mirrors_the_turns() {
        let mut f = following(WallSide::Left);
        let r = f.config.robot_radius;
        let mut far = PointCloud2D::new();
        segment_scan(&mut far, (-0.5, r + 0.05), (1.0, r + 0.05));
        let (_, w) = tick(&mut f, &far);
        assert!(w > 0.0, "bends into a left wall: {w}");
    }

    #[test]
    fn a_bump_backs_off_a_little_and_nudges_away() {
        let mut f = following(WallSide::Right);
        let pose = Pose2D::identity();
        let bump = HazardEvent::new(HazardType::BumperRight, &pose);
        // The wall touched, in front, seen.
        let r = f.config.robot_radius;
        let mut wall = PointCloud2D::new();
        segment_scan(&mut wall, (r, -0.3), (r, 0.3));
        let (v, w) = drive(f.next_action(&no_map(), &pose, &wall, Some(&bump)));
        assert!(v < 0.0 && w == 0.0, "backs off: {v} {w}");
        // The pose does not move here, so the back-off ends by its tick limit.
        for _ in 0..MAX_BACKOFF_TICKS {
            f.next_action(&no_map(), &pose, &wall, None);
        }
        let Phase::Turn { heading, .. } = f.phase else {
            panic!("turns after backing off: {:?}", f.phase);
        };
        let nudge = Tuning::default().nudge;
        assert!((heading - nudge).abs() < 1e-6, "a nudge left, not a U-turn");
        // On the side away from the wall, the nudge is towards it.
        let mut f = following(WallSide::Right);
        let bump = HazardEvent::new(HazardType::BumperLeft, &pose);
        f.next_action(&no_map(), &pose, &wall, Some(&bump));
        assert!(matches!(f.phase, Phase::BackOff { turn, .. } if turn == -nudge));
    }

    #[test]
    fn a_bump_on_nothing_seen_turns_it_alongside_and_remembers_it() {
        let mut f = following(WallSide::Right);
        let pose = Pose2D::identity();
        let bump = HazardEvent::new(HazardType::BumperBoth, &pose);
        f.next_action(&no_map(), &pose, &PointCloud2D::new(), Some(&bump));
        let Phase::BackOff { turn, .. } = f.phase else {
            panic!("backs off: {:?}", f.phase);
        };
        assert!(
            (turn - FRAC_PI_2).abs() < 1e-4,
            "dead ahead: a quarter turn left, {turn}"
        );
        // A line across the front, through the contact.
        let r = f.config.robot_radius;
        assert!(f.contacts.iter().all(|(p, _)| (p.x - r).abs() < 1e-4));
        let ys = f.contacts.iter().map(|(p, _)| p.y);
        assert!(ys.clone().fold(0.0, f32::min) < -0.05 && ys.fold(0.0, f32::max) > 0.05);
    }

    #[test]
    fn starts_for_the_nearest_wall_on_the_right_not_the_dock_behind() {
        let mut f = WallFollower::new(WallFollowConfig::default());
        // Just off the dock (behind, nearest of all and partly to the right), a wall ahead
        // right and a nearer one on the left.
        let scan = PointCloud2D {
            xs: vec![-0.25, -0.25, 0.6, 0.3],
            ys: vec![0.0, -0.3, -0.6, 0.5],
            ..Default::default()
        };
        let (v, w) = drive(f.next_action(&no_map(), &Pose2D::identity(), &scan, None));
        assert!(v == 0.0 && w < 0.0, "turns right on the spot: {v} {w}");
        // Facing it, it drives straight at it.
        let mut f = WallFollower::new(WallFollowConfig::default());
        let ahead = PointCloud2D {
            xs: vec![-0.25, 0.85],
            ys: vec![0.0, -0.01],
            ..Default::default()
        };
        let (v, w) = tick(&mut f, &ahead);
        assert!(v > 0.0 && w.abs() < 0.05, "then drives at it: {v} {w}");
    }

    #[test]
    fn closes_the_lap_back_at_the_start() {
        let mut f = WallFollower::new(WallFollowConfig::default());
        let scan = wall_at(-0.175);
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
    fn the_lap_starts_at_the_first_bump_too() {
        // E.g. glass the lidar never saw.
        let mut f = WallFollower::new(WallFollowConfig::default());
        let at = Pose2D::new(1.0, 2.0, 0.0);
        let bump = HazardEvent::new(HazardType::BumperBoth, &at);
        f.next_action(&no_map(), &at, &PointCloud2D::new(), Some(&bump));
        assert_eq!(f.lap_start.map(|(p, _)| (p.x, p.y)), Some((1.0, 2.0)));
    }
}
