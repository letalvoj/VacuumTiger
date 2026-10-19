//! One exploration session, tick by tick: everything the exploration thread decides, apart
//! from talking to SangamIO and the navigation thread. Benches run the very same decisions
//! against a simulated robot.

use std::time::{Duration, Instant};

use super::{ExplorationAction, ExplorationStrategy, HazardEvent, HazardType};
use crate::core::types::{Point2D, PointCloud2D, Pose2D, Timestamped};
use crate::io::motion_controller::{MotionConfig, compute_velocity_to_target};
use crate::state::{CliffState, CurrentMapData, PlacedScan, SharedState};

/// A SLAM scan (and so pose) older than this stops the robot: strategies steer by them.
const SCAN_TIMEOUT: Duration = Duration::from_secs(1);

/// Driven straight ahead off the dock (m) at this speed (m/s) when a session starts on it.
const UNDOCK_DISTANCE: f32 = 0.3;
const UNDOCK_SPEED: f32 = 0.1;

/// Leaving a circle of this radius (m) around where the robot was counts as progress.
pub const STUCK_RADIUS: f32 = 1.0;
/// A longer gap between two ticks was a pause (navigation active, waiting for SLAM), not
/// time spent stuck: the watch starts over.
const STUCK_PAUSE: Duration = Duration::from_secs(1);

/// What the session reads each tick: a snapshot of the shared state.
#[derive(Debug, Clone)]
pub struct Inputs {
    pub pose: Pose2D,
    pub map: Option<CurrentMapData>,
    pub scan: Timestamped<PlacedScan>,
    /// Bumpers pressed (left, right).
    pub bumper: (bool, bool),
    pub cliff: CliffState,
    /// On the dock's contacts.
    pub docked: bool,
}

impl Inputs {
    pub fn snapshot(state: &SharedState) -> Self {
        let bumper = &state.sensor_status.bumper;
        Self {
            pose: state.robot_status.pose,
            map: state.current_map.clone(),
            scan: state.sensor_status.scan_cloud.clone(),
            bumper: (bumper.left_pressed, bumper.right_pressed),
            cliff: state.sensor_status.cliff.clone(),
            docked: state.robot_status.is_docked,
        }
    }

    /// A bumper pressed or a cliff seen.
    pub fn contact(&self) -> bool {
        self.bumper.0 || self.bumper.1 || self.cliff.any_triggered()
    }
}

/// What to do until the next tick.
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    Drive {
        linear: f32,
        angular: f32,
    },
    /// Plan a path there and follow it (the navigation thread).
    NavigateTo(Pose2D),
    /// The session is over: `completed`, or stuck.
    Finish {
        completed: bool,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct Tick {
    /// Stop at once first: something is touched (the drive interlock lets the strategy
    /// back off or turn right after).
    pub halt: bool,
    pub command: Command,
    pub status: String,
}

impl Tick {
    fn hold(halt: bool, status: impl Into<String>) -> Self {
        Self {
            halt,
            command: Command::Drive {
                linear: 0.0,
                angular: 0.0,
            },
            status: status.into(),
        }
    }

    fn drive(halt: bool, (linear, angular): (f32, f32), status: impl Into<String>) -> Self {
        Self {
            halt,
            command: Command::Drive { linear, angular },
            status: status.into(),
        }
    }
}

/// Leaving the dock at the start of a session.
#[derive(Debug, Clone, Copy)]
enum Undock {
    /// Not decided yet: the first tick with fresh scans looks at the dock contacts.
    Pending,
    /// Driving straight off it from `from`, since `since`.
    Driving {
        from: Pose2D,
        since: Instant,
    },
    Done,
}

/// The first full turn in place, when there is no map yet.
#[derive(Debug, Clone, Copy, PartialEq)]
enum InitialScan {
    NotStarted,
    Rotating { since: Instant },
    Complete,
}

/// An exploration session: its strategy and the rules around it.
pub struct Session {
    strategy: Box<dyn ExplorationStrategy>,
    motion: MotionConfig,
    stuck: StuckWatch,
    undock: Undock,
    initial_scan: InitialScan,
    /// Contacts as of the last tick, to tell new ones.
    last_bumper: (bool, bool),
    last_cliff: CliffState,
    /// A contact the strategy has not heard of yet (e.g. while waiting for SLAM).
    pending_hazard: Option<HazardEvent>,
    /// The scan stamp last seen, and since when; `None` until one arrives after the start.
    scan_seen: (u64, Option<Instant>),
}

impl Session {
    /// `stuck_after`: staying within `STUCK_RADIUS` of one spot this long ends the session.
    pub fn new(
        strategy: Box<dyn ExplorationStrategy>,
        motion: MotionConfig,
        stuck_after: Duration,
    ) -> Self {
        Self {
            strategy,
            motion,
            stuck: StuckWatch::new(stuck_after),
            undock: Undock::Done,
            initial_scan: InitialScan::NotStarted,
            last_bumper: (false, false),
            last_cliff: CliffState::default(),
            pending_hazard: None,
            scan_seen: (0, None),
        }
    }

    pub fn strategy(&self) -> &dyn ExplorationStrategy {
        self.strategy.as_ref()
    }

    /// Navigation could not reach the target the strategy chose at (x, y).
    pub fn target_blocked(&mut self, x: f32, y: f32) {
        self.strategy.mark_target_blocked(x, y);
    }

    /// Navigation reached the target the strategy chose.
    pub fn target_reached(&mut self) {
        self.strategy.mark_target_reached();
    }

    /// Starts afresh (also after a session that completed). `scan_stamp` is the scan now:
    /// it is from before, so only a newer one lets the robot move.
    pub fn start(&mut self, scan_stamp: u64) {
        self.strategy.reset();
        self.stuck.reset();
        self.undock = Undock::Pending;
        self.initial_scan = InitialScan::NotStarted;
        // A contact held from before is news to this session.
        self.last_bumper = (false, false);
        self.last_cliff = CliffState::default();
        self.pending_hazard = None;
        self.scan_seen = (scan_stamp, None);
    }

    /// Whether SLAM has published a scan in the last `SCAN_TIMEOUT`, as of the last tick.
    pub fn scans_flowing(&self, now: Instant) -> bool {
        self.scan_seen
            .1
            .is_some_and(|seen| now.duration_since(seen) <= SCAN_TIMEOUT)
    }

    /// The decision for this tick, the session running. `now` is the caller's monotonic
    /// clock: wall time on the robot, simulated time in a bench. Contacts are noticed only
    /// in ticks: while navigation drives, it handles its own.
    pub fn tick(&mut self, inputs: &Inputs, now: Instant) -> Tick {
        let hazard = self.new_contact(inputs);
        let new_contact = hazard.is_some();
        self.pending_hazard = hazard.or(self.pending_hazard.take());
        let halt = inputs.contact();

        // SLAM publishes every scan it processes, also standing still: no new one is a
        // stall. Nothing moves before scans flow: SangamIO holds its command channel while
        // the lidar boots (~30 s), and commands sent meanwhile would all run late, at once.
        if inputs.scan.timestamp_us != self.scan_seen.0 {
            self.scan_seen = (inputs.scan.timestamp_us, Some(now));
        }
        if !self.scans_flowing(now) {
            return Tick::hold(halt, "Waiting for SLAM");
        }

        if let Some(tick) = self.undock(inputs, new_contact, now) {
            return tick;
        }

        let Some(map) = &inputs.map else {
            return self.initial_scan(halt, now);
        };
        self.initial_scan = InitialScan::Complete;

        if self.stuck.update(&inputs.pose, now) {
            let status = format!(
                "Stuck: did not leave {:.0} m around ({:.1}, {:.1}) in {:.0} s",
                STUCK_RADIUS,
                inputs.pose.x,
                inputs.pose.y,
                self.stuck.timeout.as_secs_f32()
            );
            return Tick {
                halt: true,
                command: Command::Finish { completed: false },
                status,
            };
        }

        let scan = inputs.scan.data.seen_from(&inputs.pose);
        self.act(map, &inputs.pose, &scan, halt)
    }

    /// A contact that was not there last tick, the most urgent first.
    fn new_contact(&mut self, inputs: &Inputs) -> Option<HazardEvent> {
        let (left, right) = inputs.bumper;
        let (was_left, was_right) = std::mem::replace(&mut self.last_bumper, inputs.bumper);
        let was = std::mem::replace(&mut self.last_cliff, inputs.cliff.clone());
        let cliff = &inputs.cliff;
        let (new_left, new_right) = (left && !was_left, right && !was_right);
        let kind = if new_left && new_right {
            HazardType::BumperBoth
        } else if new_left {
            HazardType::BumperLeft
        } else if new_right {
            HazardType::BumperRight
        } else if cliff.left_front && !was.left_front {
            HazardType::CliffLeftFront
        } else if cliff.right_front && !was.right_front {
            HazardType::CliffRightFront
        } else if cliff.left_side && !was.left_side {
            HazardType::CliffLeftSide
        } else if cliff.right_side && !was.right_side {
            HazardType::CliffRightSide
        } else {
            return None;
        };
        Some(HazardEvent::new(kind, &inputs.pose))
    }

    /// Started on the dock: straight off it before anything turns the robot (the dock is
    /// behind it). `None` once off; a new contact ends it, for the strategy to handle.
    fn undock(&mut self, inputs: &Inputs, new_contact: bool, now: Instant) -> Option<Tick> {
        if let Undock::Pending = self.undock {
            self.undock = if inputs.docked {
                log::info!("Starting on the dock: undocking");
                Undock::Driving {
                    from: inputs.pose,
                    since: now,
                }
            } else {
                Undock::Done
            };
        }
        let Undock::Driving { from, since } = self.undock else {
            return None;
        };
        let off = (inputs.pose.x - from.x).hypot(inputs.pose.y - from.y);
        let limit = Duration::from_secs_f32(2.0 * UNDOCK_DISTANCE / UNDOCK_SPEED);
        if off >= UNDOCK_DISTANCE || now.duration_since(since) >= limit || new_contact {
            log::info!("Undocked: {:.2} m off the dock", off);
            self.undock = Undock::Done;
            return None;
        }
        Some(Tick::drive(false, (UNDOCK_SPEED, 0.0), "Undocking"))
    }

    /// No map yet: a full turn in place builds one.
    fn initial_scan(&mut self, halt: bool, now: Instant) -> Tick {
        let turn = Duration::from_secs_f32(std::f32::consts::TAU / self.motion.angular_vel);
        match self.initial_scan {
            InitialScan::NotStarted => {
                log::info!("No map data yet, starting initial 360° scan rotation");
                self.initial_scan = InitialScan::Rotating { since: now };
                Tick::drive(halt, (0.0, self.motion.angular_vel), "Initial scan")
            }
            InitialScan::Rotating { since } if now.duration_since(since) < turn => {
                let progress = now.duration_since(since).as_secs_f32() / turn.as_secs_f32();
                let status = format!("Initial scan: {:.0}% complete", progress * 100.0);
                Tick::drive(halt, (0.0, self.motion.angular_vel), status)
            }
            _ => {
                self.initial_scan = InitialScan::Complete;
                Tick::hold(halt, "Waiting for map data")
            }
        }
    }

    /// The strategy's next action, as a command.
    fn act(
        &mut self,
        map: &CurrentMapData,
        pose: &Pose2D,
        scan: &PointCloud2D,
        halt: bool,
    ) -> Tick {
        let hazard = self.pending_hazard.take();
        let motion = &self.motion;
        match self.strategy.next_action(map, pose, scan, hazard.as_ref()) {
            ExplorationAction::MoveTo { target, reason } => {
                let velocity = compute_velocity_to_target(
                    pose.x, pose.y, pose.theta, target.x, target.y, motion,
                );
                Tick::drive(halt, velocity, format!("Recovery: {reason}"))
            }
            ExplorationAction::NavigateTo { target, reason } => Tick {
                halt,
                command: Command::NavigateTo(target),
                status: format!("Navigating: {reason}"),
            },
            ExplorationAction::Drive {
                linear,
                angular,
                reason,
            } => Tick::drive(halt, (linear, angular), reason),
            ExplorationAction::RotateInPlace { angle_rad } => Tick::drive(
                halt,
                (0.0, angle_rad.signum() * motion.angular_vel),
                "Scanning area",
            ),
            ExplorationAction::Complete => Tick {
                halt: true,
                command: Command::Finish { completed: true },
                status: "Exploration complete".into(),
            },
            ExplorationAction::Stuck { reason } => {
                log::warn!("Exploration stuck: {}", reason);
                Tick::drive(true, (0.0, motion.angular_vel), format!("Stuck: {reason}"))
            }
        }
    }
}

/// Notices a robot that stays in one spot while exploring: wedged with the wheels
/// spinning (the SLAM pose holds still while the odometry runs on), or circling among
/// chair legs. Time-based, so it also covers a strategy that keeps turning in place.
struct StuckWatch {
    timeout: Duration,
    /// Where the robot last left the circle around the previous anchor, and when.
    anchor: Option<(Point2D, Instant)>,
    /// The last tick it was told about.
    last: Option<Instant>,
}

impl StuckWatch {
    fn new(timeout: Duration) -> Self {
        Self {
            timeout,
            anchor: None,
            last: None,
        }
    }

    fn reset(&mut self) {
        self.anchor = None;
    }

    /// Notes the robot at `pose`; true once it has stayed within `STUCK_RADIUS` of one
    /// spot for `timeout`.
    fn update(&mut self, pose: &Pose2D, now: Instant) -> bool {
        if self
            .last
            .replace(now)
            .is_some_and(|last| now.duration_since(last) > STUCK_PAUSE)
        {
            self.reset();
        }
        let here = Point2D::new(pose.x, pose.y);
        match self.anchor {
            Some((anchor, since)) if anchor.distance(&here) <= STUCK_RADIUS => {
                now.duration_since(since) > self.timeout
            }
            _ => {
                self.anchor = Some((here, now));
                false
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    /// Ticks the watch at 10 Hz from `from` to `to` (s) along `path`; the first tick that
    /// reports stuck, in seconds.
    fn first_stuck(
        watch: &mut StuckWatch,
        t0: Instant,
        (from, to): (f32, f32),
        path: impl Fn(f32) -> Pose2D,
    ) -> Option<f32> {
        (0..)
            .map(|i| from + i as f32 * 0.1)
            .take_while(|&s| s <= to)
            .find(|&s| watch.update(&path(s), t0 + Duration::from_secs_f32(s)))
    }

    #[test]
    fn circling_in_one_spot_is_stuck_after_the_timeout() {
        let timeout = Duration::from_secs(180);
        let mut watch = StuckWatch::new(timeout);
        // 0.15 m/s around a 0.5 m circle: it never leaves the 1 m one.
        let circling = |s: f32| Pose2D::new(0.5 * (0.3 * s).cos(), 0.5 * (0.3 * s).sin(), 0.0);
        let stuck = first_stuck(&mut watch, Instant::now(), (0.0, 300.0), circling);
        assert!(
            stuck
                .is_some_and(|s| (timeout.as_secs_f32()..timeout.as_secs_f32() + 0.2).contains(&s)),
            "{stuck:?}"
        );
    }

    #[test]
    fn driving_on_or_pausing_restarts_the_clock() {
        let mut watch = StuckWatch::new(Duration::from_secs(120));
        let t0 = Instant::now();
        let still = |_| Pose2D::new(0.0, 0.0, 0.0);
        // A wall run: 0.15 m/s in a straight line, never stuck.
        let run = |s: f32| Pose2D::new(0.15 * s, 0.0, 0.0);
        assert_eq!(first_stuck(&mut watch, t0, (0.0, 600.0), run), None);
        // Still for 100 s, then a 60 s pause (navigation, SLAM stall), then still again:
        // the clock starts over after the pause.
        assert_eq!(first_stuck(&mut watch, t0, (600.0, 700.0), still), None);
        let stuck = first_stuck(&mut watch, t0, (760.0, 1000.0), still);
        assert!(stuck.is_some_and(|s| s > 880.0), "{stuck:?}");
    }

    /// What the test strategy was given: the last scan, and how many hazards; and whether it
    /// is to say it is done.
    #[derive(Default)]
    struct Seen {
        scan: Option<PointCloud2D>,
        hazards: usize,
        done: bool,
    }

    /// Drives ahead slowly until told it is done.
    #[derive(Default)]
    struct Ahead(Arc<Mutex<Seen>>);

    impl ExplorationStrategy for Ahead {
        fn name(&self) -> &'static str {
            "ahead"
        }

        fn next_action(
            &mut self,
            _map: &CurrentMapData,
            _pose: &Pose2D,
            scan: &PointCloud2D,
            hazard: Option<&HazardEvent>,
        ) -> ExplorationAction {
            let mut seen = self.0.lock().unwrap();
            seen.scan = Some(scan.clone());
            seen.hazards += usize::from(hazard.is_some());
            if seen.done {
                return ExplorationAction::Complete;
            }
            ExplorationAction::Drive {
                linear: 0.2,
                angular: 0.0,
                reason: "ahead",
            }
        }

        fn reset(&mut self) {}
    }

    fn session() -> Session {
        session_seeing().0
    }

    /// A started session driving `Ahead`, and what that is given.
    fn session_seeing() -> (Session, Arc<Mutex<Seen>>) {
        let seen = Arc::default();
        let mut session = Session::new(
            Box::new(Ahead(Arc::clone(&seen))),
            MotionConfig::default(),
            Duration::from_secs(180),
        );
        session.start(0);
        (session, seen)
    }

    /// The robot at `pose`, a map, and a scan stamped `stamp` taken at `scan_pose` with one
    /// point 1 m ahead.
    fn inputs(stamp: u64, pose: Pose2D, scan_pose: Pose2D) -> Inputs {
        let cloud = PointCloud2D::from_points(&[Point2D::new(1.0, 0.0)]);
        Inputs {
            pose,
            map: Some(CurrentMapData::default()),
            scan: Timestamped::new(
                PlacedScan {
                    cloud,
                    pose: scan_pose,
                },
                stamp,
            ),
            bumper: (false, false),
            cliff: CliffState::default(),
            docked: false,
        }
    }

    fn drive(tick: &Tick) -> (f32, f32) {
        match tick.command {
            Command::Drive { linear, angular } => (linear, angular),
            ref other => panic!("not driving: {other:?}"),
        }
    }

    #[test]
    fn nothing_moves_until_a_scan_newer_than_the_start() {
        let mut session = session();
        let t0 = Instant::now();
        let origin = Pose2D::identity();
        let stale = session.tick(&inputs(0, origin, origin), t0);
        assert_eq!(drive(&stale), (0.0, 0.0));
        assert_eq!(stale.status, "Waiting for SLAM");
        assert_eq!(
            drive(&session.tick(&inputs(1, origin, origin), t0)),
            (0.2, 0.0)
        );
        // The scans stop: after SCAN_TIMEOUT the robot holds.
        let later = t0 + SCAN_TIMEOUT + Duration::from_millis(100);
        assert_eq!(
            drive(&session.tick(&inputs(1, origin, origin), later)),
            (0.0, 0.0)
        );
    }

    #[test]
    fn the_strategy_sees_the_scan_from_where_the_robot_is_now() {
        let (mut session, seen) = session_seeing();
        // Taken at the origin; the robot has since moved 10 cm on and turned 90° left.
        let now = Pose2D::new(0.1, 0.0, std::f32::consts::FRAC_PI_2);
        session.tick(&inputs(1, now, Pose2D::identity()), Instant::now());
        let scan = seen.lock().unwrap().scan.clone().expect("a scan");
        // The point 1 m ahead of the origin is now 0.9 m to the right.
        assert!((scan.xs[0] - 0.0).abs() < 1e-5 && (scan.ys[0] + 0.9).abs() < 1e-5);
    }

    #[test]
    fn a_contact_halts_and_the_strategy_hears_of_it_once() {
        let (mut session, seen) = session_seeing();
        let t0 = Instant::now();
        let origin = Pose2D::identity();
        let mut pressed = inputs(1, origin, origin);
        pressed.bumper = (true, false);
        assert!(session.tick(&pressed, t0).halt);
        assert!(session.tick(&pressed, t0).halt, "halts while pressed");
        assert!(!session.tick(&inputs(1, origin, origin), t0).halt);
        assert_eq!(seen.lock().unwrap().hazards, 1);
    }

    #[test]
    fn starting_on_the_dock_drives_straight_off_it() {
        let mut session = session();
        let t0 = Instant::now();
        let mut docked = inputs(1, Pose2D::identity(), Pose2D::identity());
        docked.docked = true;
        let tick = session.tick(&docked, t0);
        assert_eq!(
            (drive(&tick), tick.status.as_str()),
            ((UNDOCK_SPEED, 0.0), "Undocking")
        );
        // Off the contacts, still undocking until UNDOCK_DISTANCE.
        let mut off = inputs(2, Pose2D::new(0.2, 0.0, 0.0), Pose2D::identity());
        assert_eq!(session.tick(&off, t0).status, "Undocking");
        off.pose.x = UNDOCK_DISTANCE;
        assert_eq!(session.tick(&off, t0).status, "ahead");
    }

    #[test]
    fn undocking_gives_up_after_twice_the_time_it_takes() {
        let mut session = session();
        let t0 = Instant::now();
        let mut docked = inputs(1, Pose2D::identity(), Pose2D::identity());
        docked.docked = true;
        session.tick(&docked, t0);
        let late = t0 + Duration::from_secs_f32(2.0 * UNDOCK_DISTANCE / UNDOCK_SPEED);
        docked.scan.timestamp_us = 2;
        assert_eq!(session.tick(&docked, late).status, "ahead");
    }

    #[test]
    fn a_contact_while_waiting_for_slam_reaches_the_strategy_later() {
        let (mut session, seen) = session_seeing();
        let t0 = Instant::now();
        let origin = Pose2D::identity();
        let mut pressed = inputs(0, origin, origin);
        pressed.bumper = (false, true);
        assert_eq!(session.tick(&pressed, t0).status, "Waiting for SLAM");
        session.tick(&inputs(1, origin, origin), t0);
        assert_eq!(seen.lock().unwrap().hazards, 1);
    }

    #[test]
    fn standing_in_one_spot_finishes_stuck() {
        let mut session = session();
        let t0 = Instant::now();
        let origin = Pose2D::identity();
        // Ticks at 10 Hz with fresh scans, never leaving the spot.
        let tick = (1..)
            .map(|i| {
                let now = t0 + Duration::from_millis(100 * i);
                session.tick(&inputs(i, origin, origin), now)
            })
            .find(|tick| {
                tick.command
                    != Command::Drive {
                        linear: 0.2,
                        angular: 0.0,
                    }
            })
            .expect("a tick that is not driving on");
        assert!(tick.halt);
        assert_eq!(tick.command, Command::Finish { completed: false });
        assert!(tick.status.starts_with("Stuck"), "{}", tick.status);
    }

    #[test]
    fn a_strategy_that_is_done_finishes_the_session() {
        let (mut session, seen) = session_seeing();
        let origin = Pose2D::identity();
        seen.lock().unwrap().done = true;
        let tick = session.tick(&inputs(1, origin, origin), Instant::now());
        assert!(tick.halt);
        assert_eq!(tick.command, Command::Finish { completed: true });
    }
}
