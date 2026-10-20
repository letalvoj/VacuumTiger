//! The stock follower's actions: each a control law with its own finish and fail tests
//! (`STOCK_SWEEP_LOGIC.md` §3–§5). The stock steps every 20 ms; its per-tick steps are
//! given here per second and scaled by the tick we run at.
//!
//! Everything is in the mirrored frame: the wall on the right, + is a left turn.

use crate::core::math::angle_diff;
use crate::core::types::Pose2D;
use crate::exploration::strategy::HazardType;

use super::gaps::{CLEAR, Gaps};

/// The stock's global speed (m/s) and Follow's and Arc's turn cap (rad/s).
pub const VMAX: f32 = 0.26;
pub const WMAX: f32 = 1.05;
/// Follow, Arc and Line ramp ω this fast (rad/s²: 0.8 per tick)…
const W_SLEW: f32 = 40.0;
/// …and v up this fast (m/s²: 0.02 per tick), and down this fast (0.32 per tick).
const V_UP: f32 = 1.0;
const V_DOWN: f32 = 16.0;

/// Follow sets off at least this fast (m/s).
const FOLLOW_START: f32 = 0.05;
/// The error is clamped to this (m), which is also the sectors' acceptance band.
const ERR_CLAMP: f32 = 0.04;
/// With no sector, both the gap and the setpoint (m): no error.
const NO_SECTOR_GAP: f32 = 0.045;
/// The front fan brakes within this gap (m), from this offset (m), down to this scale…
const BRAKE_WINDOW: f32 = 0.13;
const BRAKE_OFFSET: f32 = 0.04;
const MIN_SCALE: f32 = 0.3;
/// …and the speed never drops under this share of the cap.
const FLOOR: f32 = 0.4;
/// Turning slows it by this times ω's share of `WMAX`.
const TURN_SLOWING: f32 = 1.2;
// The gain (rad/s per m) by the setpoint: over 2 cm, over 0, else.
const GAIN_FAR: f32 = 10.0;
const GAIN_NEAR: f32 = 40.0;
const GAIN_BEHIND: f32 = 80.0;

/// Rotation: ω (rad/s), its ramp (rad/s²: 0.9 per tick), v braked to 0 (m/s²: 0.015 per
/// tick); done within this (rad); timeout (s).
const SPIN: f32 = 1.4;
const SPIN_SLEW: f32 = 45.0;
const SPIN_V_DOWN: f32 = 0.75;
const SPIN_DONE: f32 = 0.01;
const SPIN_TIMEOUT: f32 = 5.0;

/// Arc: the radius (m), so v = radius · `WMAX`; timeout (s).
const ARC_RADIUS: f32 = 0.1209;
const ARC_TIMEOUT: f32 = 10.0;
/// The angle a lost wall arcs through (rad, −75.6°)…
pub const ARC_ANGLE: f32 = -1.319_469;
/// …and the one after a Line (−π/2).
pub const ARC_ANGLE_AFTER_LINE: f32 = -std::f32::consts::FRAC_PI_2;

/// Line: speed cap (m/s), turn cap (rad/s); it brakes within this gap ahead (m) from this
/// offset (m) down to this scale; timeout (s).
const LINE_V: f32 = 0.182;
const LINE_W: f32 = 0.7;
const LINE_WINDOW: f32 = 0.15;
const LINE_BRAKE_OFFSET: f32 = 0.02;
const LINE_MIN_SCALE: f32 = 0.35;
const LINE_TIMEOUT: f32 = 5.0;
/// How hard a Line holds its heading (rad/s per rad). Not decoded: ours, as are the
/// Line's virtual-bumper fans and its braking fan.
const LINE_GAIN: f32 = 2.0;

/// BackAlign: speed (m/s), its ramp (m/s²: 0.03 per tick); timeout (s); it stops for
/// something this near behind (m).
const BACK_V: f32 = 0.104;
const BACK_ACCEL: f32 = 1.5;
const BACK_TIMEOUT: f32 = 1.0;
const REAR_GAP: f32 = 0.02;

/// What the actions sense each tick.
pub struct Sensed<'a> {
    pub gaps: &'a Gaps,
    /// The bumper pressed (mirrored: `BumperRight` is the wall side).
    pub bumper: Option<HazardType>,
    pub cliff: bool,
    /// The pose, mirrored with the rest: y and θ flip for a wall on the left.
    pub pose: &'a Pose2D,
    /// Tight baseboard mode, not loose (`STOCK_SWEEP_LOGIC.md` §9).
    pub tight: bool,
    /// The tick (s).
    pub dt: f32,
}

/// Why an action failed, in the order the stock checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cause {
    Cliff,
    /// A bumper pressed (mirrored).
    Bumper(HazardType),
    /// The lidar sees something too near ahead.
    VirtualBumper,
    /// Something behind stops a retract.
    Blocked,
    Timeout,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Status {
    Running,
    Finished,
    Failed(Cause),
}

/// The command, carried from one action into the next (m/s, rad/s).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Command {
    pub v: f32,
    pub w: f32,
}

/// A follow sector: beams, setpoint (tight, loose; m), speed cap (m/s), and whether its
/// band is closed (`≤`, S3–S5) or open (`<`).
struct Sector {
    beams: (usize, usize),
    setpoint: (f32, f32),
    cap: f32,
    closed: bool,
}

/// `filterFeedbackDistance`: the first that passes wins.
const SECTORS: [Sector; 5] = [
    Sector {
        beams: (148, 150),
        setpoint: (0.055, 0.08),
        cap: VMAX,
        closed: false,
    },
    Sector {
        beams: (144, 147),
        setpoint: (0.035, 0.06),
        cap: VMAX,
        closed: false,
    },
    Sector {
        beams: (135, 143),
        setpoint: (0.02, 0.02),
        cap: 0.7 * VMAX,
        closed: true,
    },
    Sector {
        beams: (126, 134),
        setpoint: (0.005, 0.005),
        cap: 0.7 * VMAX,
        closed: true,
    },
    Sector {
        beams: (120, 125),
        setpoint: (-0.01, -0.01),
        cap: 0.7 * VMAX,
        closed: true,
    },
];

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Action {
    /// `CActionRightFollowWallState`: never finishes. `sector` is the one followed last
    /// tick (`None`: none passed, or not run yet).
    Follow { ticks: u32, sector: Option<usize> },
    /// `CActionRotationState`: `left` to turn (rad), the way it turns (+1 left, −1 right).
    Rotation { left: f32, way: f32, ticks: u32 },
    /// `CActionArcState`: `left` to turn (rad, negative: right).
    Arc { left: f32, ticks: u32 },
    /// `CActionLineState`: straight on from `start`'s heading, `distance` (m) or, with
    /// `None`, until something is in the way (finding a wall; no timeout).
    Line {
        start: Pose2D,
        distance: Option<f32>,
        ticks: u32,
    },
    /// `CActionBackAlignState`: `left` (m) still to reverse.
    BackAlign { left: f32, ticks: u32 },
}

impl Action {
    pub fn follow() -> Self {
        Action::Follow {
            ticks: 0,
            sector: None,
        }
    }

    pub fn rotation(angle: f32) -> Self {
        Action::Rotation {
            left: angle,
            way: angle.signum(),
            ticks: 0,
        }
    }

    pub fn arc(angle: f32) -> Self {
        Action::Arc {
            left: angle,
            ticks: 0,
        }
    }

    pub fn line(start: &Pose2D, distance: Option<f32>) -> Self {
        Action::Line {
            start: *start,
            distance,
            ticks: 0,
        }
    }

    pub fn back_align(distance: f32) -> Self {
        Action::BackAlign {
            left: distance.abs(),
            ticks: 0,
        }
    }

    /// The robot turned by `turn` (rad, mirrored) since the last tick.
    pub fn turned(&mut self, turn: f32) {
        if let Action::Rotation { left, .. } | Action::Arc { left, .. } = self {
            *left -= turn;
        }
    }

    /// `CActionState::execute`: finished, else failed, else running.
    pub fn status(&self, s: &Sensed) -> Status {
        let timed_out = |ticks: u32, limit: f32| ticks as f32 * s.dt >= limit;
        let contact = || {
            if s.cliff {
                Some(Cause::Cliff)
            } else {
                s.bumper.map(Cause::Bumper)
            }
        };
        let failed = |cause: Option<Cause>| cause.map_or(Status::Running, Status::Failed);
        match *self {
            Action::Follow { .. } => failed(contact().or_else(|| {
                (s.gaps.bumper(160, 195, 0.04) || s.gaps.bumper(150, 195, 0.02))
                    .then_some(Cause::VirtualBumper)
            })),
            Action::Rotation { left, way, ticks } => {
                // Within reach of the target, or past it.
                if left * way <= SPIN_DONE {
                    return Status::Finished;
                }
                // The bumper on the side it turns away from may stay pressed.
                let away = if way > 0.0 {
                    HazardType::BumperRight
                } else {
                    HazardType::BumperLeft
                };
                let bumper = s.bumper.filter(|&k| k != away);
                let cause = if s.cliff {
                    Some(Cause::Cliff)
                } else {
                    bumper.map(Cause::Bumper)
                };
                failed(cause.or(timed_out(ticks, SPIN_TIMEOUT).then_some(Cause::Timeout)))
            }
            Action::Arc { left, ticks } => {
                if s.gaps.follow_ir(120, 143).0 <= 0.03 || left >= 0.0 {
                    return Status::Finished;
                }
                failed(contact().or_else(|| {
                    if s.gaps.bumper(155, 205, 0.04) {
                        Some(Cause::VirtualBumper)
                    } else {
                        timed_out(ticks, ARC_TIMEOUT).then_some(Cause::Timeout)
                    }
                }))
            }
            Action::Line {
                start,
                distance,
                ticks,
            } => {
                let along = (s.pose.x - start.x).hypot(s.pose.y - start.y);
                if distance.is_some_and(|d| along >= d) {
                    return Status::Finished;
                }
                failed(contact().or_else(|| {
                    if s.gaps.bumper(140, 220, 0.02) || s.gaps.bumper(155, 205, 0.04) {
                        Some(Cause::VirtualBumper)
                    } else {
                        (distance.is_some() && timed_out(ticks, LINE_TIMEOUT))
                            .then_some(Cause::Timeout)
                    }
                }))
            }
            Action::BackAlign { left, ticks } => {
                if left <= 1e-4 {
                    return Status::Finished;
                }
                let rear = s.gaps.bumper(5, 15, REAR_GAP) || s.gaps.bumper(345, 355, REAR_GAP);
                failed(if rear {
                    Some(Cause::Blocked)
                } else {
                    timed_out(ticks, BACK_TIMEOUT).then_some(Cause::Timeout)
                })
            }
        }
    }

    /// One tick of the action's law: moves `cmd` on.
    pub fn execute(&mut self, s: &Sensed, cmd: &mut Command) {
        let dt = s.dt;
        match self {
            Action::Follow { ticks, sector } => {
                if *ticks == 0 {
                    cmd.v = cmd.v.max(FOLLOW_START);
                }
                *ticks += 1;
                let (picked, setpoint, gap, cap) = feedback(s.gaps, s.tight);
                *sector = picked;
                let err = (setpoint - gap).clamp(-ERR_CLAMP, ERR_CLAMP);
                let gain = if setpoint > 0.02 {
                    GAIN_FAR
                } else if setpoint > 0.0 {
                    GAIN_NEAR
                } else {
                    GAIN_BEHIND
                };
                cmd.w = slew(cmd.w, gain * err, W_SLEW * dt).clamp(-WMAX, WMAX);
                let front = s.gaps.fused_sonar(160, 210);
                let fr = if front > BRAKE_WINDOW {
                    1.0
                } else {
                    ((front - BRAKE_OFFSET) / BRAKE_WINDOW).max(MIN_SCALE)
                };
                let ea = if cmd.w.abs() > WMAX {
                    0.0
                } else {
                    (1.0 - TURN_SLOWING * cmd.w.abs() / WMAX).max(MIN_SCALE)
                };
                let target = (ea * fr).max(FLOOR) * cap;
                cmd.v = ramp(cmd.v, target, dt).min(VMAX);
            }
            Action::Rotation { left, way, ticks } => {
                *ticks += 1;
                // The last tick slows so it lands on the target, not a tick past it.
                let w = *way * SPIN.min(*left * *way / dt);
                cmd.w = slew(cmd.w, w, SPIN_SLEW * dt);
                cmd.v = slew(cmd.v, 0.0, SPIN_V_DOWN * dt);
            }
            Action::Arc { ticks, .. } => {
                *ticks += 1;
                cmd.w = slew(cmd.w, -WMAX, W_SLEW * dt);
                cmd.v = ramp(cmd.v, ARC_RADIUS * WMAX, dt);
            }
            Action::Line { start, ticks, .. } => {
                *ticks += 1;
                let d = s.gaps.fused_sonar(150, 215);
                let scale = if d == CLEAR {
                    1.0
                } else {
                    ((d - LINE_BRAKE_OFFSET) / LINE_WINDOW).clamp(LINE_MIN_SCALE, 1.0)
                };
                cmd.v = ramp(cmd.v, scale * LINE_V, dt);
                let hold = LINE_GAIN * angle_diff(s.pose.theta, start.theta);
                cmd.w = slew(cmd.w, hold, W_SLEW * dt).clamp(-LINE_W, LINE_W);
            }
            Action::BackAlign { left, ticks } => {
                *ticks += 1;
                // Commanded, not measured: a few millimetres only release the bumper.
                let v = BACK_V.min(*left / dt);
                cmd.v = slew(cmd.v, -v, BACK_ACCEL * dt);
                cmd.w = 0.0;
                *left -= cmd.v.abs() * dt;
            }
        }
    }

    pub fn reason(&self) -> &'static str {
        match self {
            Action::Follow { .. } => "following the wall",
            Action::Rotation { .. } => "turning on the spot",
            Action::Arc { .. } => "arcing round to the wall",
            Action::Line { distance: None, .. } => "driving to the nearest wall",
            Action::Line { .. } => "driving straight",
            Action::BackAlign { .. } => "backing off a bump",
        }
    }
}

/// `filterFeedbackDistance`: the sector followed, its setpoint, the gap there (m) and the
/// speed cap (m/s).
fn feedback(gaps: &Gaps, tight: bool) -> (Option<usize>, f32, f32, f32) {
    SECTORS
        .iter()
        .enumerate()
        .find_map(|(i, sector)| {
            let gap = gaps.follow_ir(sector.beams.0, sector.beams.1).0;
            let setpoint = if tight {
                sector.setpoint.0
            } else {
                sector.setpoint.1
            };
            let off = gap - setpoint;
            let passes = gap != CLEAR
                && if sector.closed {
                    off <= ERR_CLAMP
                } else {
                    off < ERR_CLAMP
                };
            passes.then_some((Some(i), setpoint, gap, sector.cap))
        })
        .unwrap_or((None, NO_SECTOR_GAP, NO_SECTOR_GAP, 0.5 * VMAX))
}

/// `value` moved towards `target` by at most `step`.
fn slew(value: f32, target: f32, step: f32) -> f32 {
    value + (target - value).clamp(-step, step)
}

/// Speed towards `target`: up gently (`V_UP`), down at once (`V_DOWN`).
fn ramp(v: f32, target: f32, dt: f32) -> f32 {
    let rate = if target >= v { V_UP } else { V_DOWN };
    slew(v, target, rate * dt)
}

#[cfg(test)]
mod tests {
    use super::super::gaps::Shell;
    use super::super::gaps::tests::{ray_scan, right_wall};
    use super::*;

    /// Our lidar where the shell table has it (and no optical offset): gaps as in the spec.
    const LIDAR_X: f32 = -0.105;
    const R: f32 = 0.175;
    const DT: f32 = 0.1;

    fn sensed<'a>(gaps: &'a Gaps, pose: &'a Pose2D) -> Sensed<'a> {
        Sensed {
            gaps,
            bumper: None,
            cliff: false,
            pose,
            tight: true,
            dt: DT,
        }
    }

    fn gaps(points: &[crate::core::types::Point2D]) -> Gaps {
        Gaps::new(&Shell::new(LIDAR_X, 0.0), points)
    }

    #[test]
    fn on_the_tight_setpoint_it_cruises_straight_in_s1() {
        // S1 reads 5.5 cm at a clearance of −1.1 cm (§3).
        let gaps = gaps(&right_wall(LIDAR_X, R, -0.011));
        let pose = Pose2D::identity();
        let mut follow = Action::follow();
        let mut cmd = Command::default();
        for _ in 0..30 {
            assert_eq!(follow.status(&sensed(&gaps, &pose)), Status::Running);
            follow.execute(&sensed(&gaps, &pose), &mut cmd);
        }
        assert_eq!(
            follow,
            Action::Follow {
                ticks: 30,
                sector: Some(0)
            }
        );
        assert!(cmd.w.abs() < 0.01, "{cmd:?}");
        assert!((cmd.v - VMAX).abs() < 0.003, "{cmd:?}");
    }

    #[test]
    fn it_speeds_up_gently_and_brakes_at_once() {
        let pose = Pose2D::identity();
        let mut cmd = Command::default();
        let mut follow = Action::follow();
        let open = gaps(&right_wall(LIDAR_X, R, -0.011));
        follow.execute(&sensed(&open, &pose), &mut cmd);
        assert!((cmd.v - (FOLLOW_START + V_UP * DT)).abs() < 1e-5, "{cmd:?}");
        // A wall 5 cm ahead: the floor at once.
        let mut points = right_wall(LIDAR_X, R, -0.011);
        points.extend(ray_scan(LIDAR_X, &[((R + 0.05, -1.0), (R + 0.05, 1.0))]));
        let blocked = gaps(&points);
        cmd.v = VMAX;
        follow.execute(&sensed(&blocked, &pose), &mut cmd);
        assert!((cmd.v - FLOOR * VMAX).abs() < 1e-4, "{cmd:?}");
    }

    #[test]
    fn a_wall_ahead_within_4_cm_is_a_virtual_bumper() {
        let pose = Pose2D::identity();
        let ahead = gaps(&ray_scan(LIDAR_X, &[((R + 0.035, -1.0), (R + 0.035, 1.0))]));
        let follow = Action::follow();
        assert_eq!(
            follow.status(&sensed(&ahead, &pose)),
            Status::Failed(Cause::VirtualBumper)
        );
    }

    #[test]
    fn with_no_wall_it_drives_straight_at_half_speed() {
        let pose = Pose2D::identity();
        let none = gaps(&[]);
        let mut follow = Action::follow();
        let mut cmd = Command { v: VMAX, w: 0.5 };
        follow.execute(&sensed(&none, &pose), &mut cmd);
        assert_eq!(
            cmd,
            Command {
                v: 0.5 * VMAX,
                w: 0.0
            }
        );
    }

    #[test]
    fn a_rotation_spins_at_1_4_and_lands_on_its_target() {
        let pose = Pose2D::identity();
        let none = gaps(&[]);
        let mut rotation = Action::rotation(0.3);
        let mut cmd = Command { v: 0.1, w: 0.0 };
        rotation.execute(&sensed(&none, &pose), &mut cmd);
        assert_eq!(cmd.w, SPIN);
        assert!((cmd.v - (0.1 - SPIN_V_DOWN * DT)).abs() < 1e-6);
        rotation.turned(cmd.w * DT);
        rotation.execute(&sensed(&none, &pose), &mut cmd);
        assert_eq!(cmd.w, SPIN);
        rotation.turned(cmd.w * DT);
        assert_eq!(rotation.status(&sensed(&none, &pose)), Status::Running);
        rotation.execute(&sensed(&none, &pose), &mut cmd);
        assert!(
            (cmd.w - 0.2).abs() < 1e-4,
            "the last 0.02 rad in one tick: {cmd:?}"
        );
        rotation.turned(cmd.w * DT);
        assert_eq!(rotation.status(&sensed(&none, &pose)), Status::Finished);
    }

    #[test]
    fn a_rotation_ignores_the_bumper_it_turns_away_from() {
        let pose = Pose2D::identity();
        let none = gaps(&[]);
        let mut s = sensed(&none, &pose);
        s.bumper = Some(HazardType::BumperRight);
        assert_eq!(Action::rotation(0.5).status(&s), Status::Running);
        assert_eq!(
            Action::rotation(-0.5).status(&s),
            Status::Failed(Cause::Bumper(HazardType::BumperRight))
        );
        s.bumper = Some(HazardType::BumperLeft);
        assert_eq!(
            Action::rotation(0.5).status(&s),
            Status::Failed(Cause::Bumper(HazardType::BumperLeft))
        );
        assert_eq!(Action::rotation(-0.5).status(&s), Status::Running);
    }

    #[test]
    fn an_arc_ends_when_the_wall_is_back_or_its_angle_is_turned() {
        let pose = Pose2D::identity();
        let mut arc = Action::arc(ARC_ANGLE);
        let none = gaps(&[]);
        let mut cmd = Command::default();
        arc.execute(&sensed(&none, &pose), &mut cmd);
        assert_eq!(cmd.w, -WMAX);
        assert_eq!(arc.status(&sensed(&none, &pose)), Status::Running);
        let wall = gaps(&right_wall(LIDAR_X, R, 0.01));
        assert_eq!(arc.status(&sensed(&wall, &pose)), Status::Finished);
        arc.turned(ARC_ANGLE);
        assert_eq!(arc.status(&sensed(&none, &pose)), Status::Finished);
    }

    #[test]
    fn a_back_align_reverses_its_few_millimetres_and_stops() {
        let pose = Pose2D::identity();
        let none = gaps(&[]);
        let mut back = Action::back_align(0.005);
        let mut cmd = Command::default();
        back.execute(&sensed(&none, &pose), &mut cmd);
        assert!((cmd.v + 0.05).abs() < 1e-5, "5 mm in one tick: {cmd:?}");
        assert_eq!(back.status(&sensed(&none, &pose)), Status::Finished);
    }

    #[test]
    fn a_line_holds_its_heading_and_ends_at_its_distance() {
        let none = gaps(&[]);
        let start = Pose2D::identity();
        let mut line = Action::line(&start, Some(0.05));
        let skewed = Pose2D::new(0.02, 0.0, 0.1);
        let mut cmd = Command::default();
        line.execute(&sensed(&none, &skewed), &mut cmd);
        assert!(cmd.w < 0.0, "back to the start's heading: {cmd:?}");
        assert_eq!(line.status(&sensed(&none, &skewed)), Status::Running);
        let there = Pose2D::new(0.05, 0.0, 0.0);
        assert_eq!(line.status(&sensed(&none, &there)), Status::Finished);
    }
}
