//! One lap round the room: where it started, how far round it has turned, and when it is
//! back over its seam.

use std::f32::consts::{FRAC_PI_2, PI};

use crate::core::math::angle_diff;
use crate::core::types::{Point2D, Pose2D};

/// Going round a room turns the robot round once (±2π, the sign by the side it follows);
/// a lap closes only after this much of it. Not travel or distance from the start:
/// dithering by an unseen edge piles travel up and turns back and forth, and a small room
/// keeps the whole lap near its start.
const LAP_TURN: f32 = 1.5 * PI;

/// A lap round the room's edge. The fields are open to `edge` for the followers' tests.
#[derive(Debug, Clone, Copy)]
pub struct Lap {
    /// It closes back within this distance of where it started (m)…
    radius: f32,
    /// …followed on this far past the nearest it came to that start (m), over the seam.
    overlap: f32,
    /// Where it started (world frame, not moved by loop closures) and how far the robot has
    /// turned since `heading` was set (rad, + left): the turn from facing a wall it bumped
    /// into to running along it is not part of the lap.
    pub(super) start: Option<(Point2D, f32)>,
    /// The heading the first tick along the edge (rad): the lap closes going that way, not
    /// passing its start the other way (down the far side of a narrow bay). Not the heading
    /// at a bump that started the lap: into the wall, not along it. Unset, any way closes.
    pub(super) heading: Option<f32>,
    /// Once back at its start, the nearest it has come to it (m) and the travel then (m).
    pub(super) back: Option<(f32, f32)>,
}

impl Lap {
    pub fn new(radius: f32, overlap: f32) -> Self {
        Self {
            radius,
            overlap,
            start: None,
            heading: None,
            back: None,
        }
    }

    /// Whether the robot has run along the edge since the lap started.
    pub fn aligned(&self) -> bool {
        self.heading.is_some()
    }

    /// Starts the lap at `pose` unless it has started; whether it started now.
    pub fn start(&mut self, pose: &Pose2D) -> bool {
        let new = self.start.is_none();
        if new {
            self.start = Some((Point2D::new(pose.x, pose.y), 0.0));
        }
        new
    }

    /// The robot turned by `turn` (rad, + left).
    pub fn turned(&mut self, turn: f32) {
        if let Some((_, turned)) = &mut self.start {
            *turned += turn;
        }
    }

    /// The robot runs along the edge, facing `heading`: the first time, that is the way the
    /// lap closes, and what it turned so far was getting alongside.
    pub fn along(&mut self, heading: f32) {
        if self.heading.is_none() {
            self.heading = Some(heading);
            if let Some((_, turned)) = &mut self.start {
                *turned = 0.0;
            }
        }
    }

    /// Whether it has been round and back over the seam: back within `radius` of the start
    /// the way it started, and `overlap` on from the nearest it came to it, `travelled` being
    /// the distance driven (m): over the seam, not short of it (the start may lie in a
    /// corner still ahead).
    pub fn done(&mut self, pose: &Pose2D, travelled: f32) -> bool {
        let Some((start, turned)) = self.start else {
            return false;
        };
        let d = (pose.x - start.x).hypot(pose.y - start.y);
        let same_way = self
            .heading
            .is_none_or(|h| angle_diff(pose.theta, h).abs() < FRAC_PI_2);
        let back = turned.abs() >= LAP_TURN && d < self.radius && same_way;
        match &mut self.back {
            Some(nearest) if d < nearest.0 => *nearest = (d, travelled),
            None if back => self.back = Some((d, travelled)),
            _ => {}
        }
        self.back
            .is_some_and(|(_, at)| travelled - at >= self.overlap)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::math::normalize_angle;
    use crate::exploration::edge::Travel;

    #[test]
    fn a_lap_counts_the_turn_round_not_turns_back_and_forth() {
        let mut lap = Lap::new(0.3, 0.5);
        let mut travel = Travel::default();
        lap.start(&Pose2D::identity());
        let turned = |lap: &Lap| lap.start.unwrap().1;
        // Dithering by an unseen edge: left and right again, many times.
        for _ in 0..20 {
            for theta in [0.0, 0.8, 0.0, -0.8] {
                lap.turned(travel.update(&Pose2D::new(0.0, 0.0, theta)));
            }
        }
        assert!(turned(&lap).abs() < 1.0, "{}", turned(&lap));
        // Round a room: a quarter at each corner, across ±π.
        for k in 1..=8 {
            let theta = normalize_angle(k as f32 * FRAC_PI_2 / 2.0);
            lap.turned(travel.update(&Pose2D::new(0.0, 0.0, theta)));
        }
        assert!((turned(&lap) - 2.0 * PI).abs() < 1.0, "{}", turned(&lap));
    }

    #[test]
    fn passing_the_start_the_other_way_is_no_lap() {
        // Down the far side of a narrow bay, 12 cm from where following started going up.
        let (radius, overlap) = (0.3, 0.5);
        let mut lap = Lap::new(radius, overlap);
        let up = FRAC_PI_2;
        // Having been round the room.
        lap.start = Some((Point2D::new(0.0, 0.0), 2.0 * PI));
        lap.heading = Some(up);
        assert!(!lap.done(&Pose2D::new(0.12, 0.0, -up), 5.0));
        assert!(!lap.done(&Pose2D::new(0.12, -overlap, -up), 5.0 + overlap));
        assert!(lap.back.is_none(), "it never came back");
        // Round the bay and up the wall again: the lap.
        assert!(!lap.done(&Pose2D::new(0.0, 0.0, up), 7.0));
        assert!(lap.done(&Pose2D::new(0.0, overlap, up), 7.0 + overlap));
    }

    #[test]
    fn the_turn_alongside_is_not_counted() {
        let mut lap = Lap::new(0.3, 0.5);
        lap.start(&Pose2D::identity());
        lap.turned(FRAC_PI_2);
        lap.along(0.0);
        assert_eq!(lap.heading, Some(0.0));
        assert_eq!(lap.start.unwrap().1, 0.0);
        lap.turned(0.3);
        lap.along(1.0);
        assert_eq!(lap.heading, Some(0.0), "only the first time");
        assert_eq!(lap.start.unwrap().1, 0.3);
    }
}
