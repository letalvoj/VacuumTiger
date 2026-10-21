//! The stock firmware's edge pass, ported from its decoded `everest-server`
//! (`STOCK_SWEEP_LOGIC.md`, vendor RE kept outside the repo).
//!
//! - [`gaps`]: what it sees, the gap from the body per beam;
//! - [`actions`]: Follow, Rotation, Arc, Line and BackAlign, each a law with its own finish
//!   and fail tests;
//! - here: the transfer state machine over them (`CActionTransferMotionROFW`), the failure
//!   recipes, the tight/loose baseboard mode, the motion watchdogs and the lap.
//!
//! Where it differs from the stock, on purpose:
//! - the last [`OBSTACLE_MEMORY`] of scans stands in for the obstacle map under the gaps (its
//!   units are not decoded; SLAM's 5 cm grid is too coarse for 1 cm setpoints), and every gap
//!   query reads it; `getBumper` does not debounce over three layers;
//! - no rush (it needs a front sensor other than the lidar), no virtual walls or magnetic
//!   strip;
//! - a wall on the left mirrors the right follower;
//! - the lap is our [`Lap`], not the stock's chain of cells, and it starts along the wall,
//!   not facing it;
//! - a cliff always turns 63° (the stock turns 0 in a cliff state not mapped to our
//!   sensors);
//! - Line's heading gain is ours, and the line flags are a least-squares fit;
//! - it ticks at the exploration loop's rate, the stock's 20 ms steps scaled to it, and a
//!   Rotation slows on its last tick so as not to overshoot by a whole one;
//! - an arc counts towards the second −π/2 in a row however it ended (the decode does not
//!   say what breaks the row);
//! - started away from walls, tripped by a watchdog, or stopped by a second −π/2 arc in a
//!   row (`processArcInCircle`), it finds a wall: it faces the nearest return and drives to
//!   it (the stock's task-level escape is not decoded);
//! - of the motion watchdogs only stuck and circling run: wheel stall, back slip, a
//!   persisting cliff and the app's bumper flag have no signal here (the session halts on
//!   contacts itself);
//! - the decode shows no reset of the two baseboard counters but the mode change: the bumps
//!   add up while tight, the left-only updates while loose.
//!
//! `stock_follow_fix` is the same but for a wall the lidar sees short of where it is (an
//! overhang, a kick board): a bump on the wall side moves the wall-side gaps in by what they
//! still read, for [`BUMPED_REACH`] past the last such bump, and it follows loose meanwhile,
//! since rubbing that wall is a bump each time. And once bumps loosened it, it stays loose
//! for the run: what made it bump (the lidar's range bias, baseboards) does not stop at a
//! corner, and tight again it bumps its way loose round every one.

mod actions;
mod gaps;

use std::collections::VecDeque;
use std::f32::consts::{FRAC_PI_4, PI};

use serde::Deserialize;

use crate::algorithms::matching::Line2D;
use crate::core::math::angle_diff;
use crate::core::types::{Point2D, PointCloud2D, Pose2D};
use crate::exploration::strategy::{
    ExplorationAction, ExplorationStrategy, HazardEvent, HazardType,
};
use crate::state::CurrentMapData;

use super::{Lap, Travel, WallSide};
use actions::{ARC_ANGLE, ARC_ANGLE_AFTER_LINE, Action, Cause, Command, Sensed, Status};
use gaps::{Gaps, Shell, beam_angle};

// Rotation steps after each failure (rad, + left) and how often it may turn again still
// blocked (§7).
const CLIFF_TURN: f32 = 1.0996;
const BUMPER_OTHER_TURN: f32 = 1.4137;
const GRAZE_TURN: f32 = 0.2;
const NUDGE_TURN: f32 = 0.05;
const NUDGE_TRIES: u32 = 15;
/// The virtual bumper's turn when nothing is in its fan (rad).
const VIRTUAL_BUMPER_TURN: f32 = 0.18;
// Retracts (m).
const CLIFF_BACK: f32 = 0.02;
const BUMPER_BACK: f32 = 0.005;
// The Line after a clear turn (m), and after giving up on a cliff or a bumper.
const RELAUNCH: f32 = 0.05;
const CLIFF_GIVE_UP: f32 = 0.025;
const BUMPER_GIVE_UP: f32 = 0.01;
/// The first entry turns this far left with only the left-front fan blocked (rad).
const FIRST_ENTER_TURN: f32 = 1.4137;
/// It finds a wall first when nothing on the right is nearer than this (m): the arc
/// acquires one this near (§6).
const FIND_WALL_GAP: f32 = 0.1;

/// The line flags: returns nearer than this to the lidar (m)…
const LINE_RANGE: f32 = 0.5;
// …at least this many of them in a fan, this near a fitted line (RMS, m).
const LINE_POINTS: usize = 8;
const LINE_RMS: f32 = 0.01;
/// Loose after more than this many bumps along a line on the right…
const LOOSE_BUMPS: u32 = 2;
// …tight again after this many updates with a line on the left only, or a corner.
const LEFT_ONLY_UPDATES: u32 = 4;
const CORNER: f32 = FRAC_PI_4;
/// The arcs of `ARC_ANGLE_AFTER_LINE` in a row that stop it (`processArcInCircle`).
const QUARTER_ARCS: u32 = 2;
/// The lap starts only while following with nothing nearer than this ahead (m), in the
/// virtual bumper's fan: still facing the wall it just found, the lap's heading would point
/// into it and the spin alongside would count as going round.
const LAP_FRONT_GAP: f32 = 0.1;
/// The most actions handed over in one tick.
const HAND_OVERS: usize = 4;

// Stuck: over this long (s), moved less than this (m, in x and y) and turned less (rad).
const STUCK_WINDOW: f32 = 3.0;
const STUCK_MOVE: f32 = 0.05;
const STUCK_TURN: f32 = 0.05;
// Circling: turned this much (rad) before getting this far from where it started (m).
const CIRCLE_TURN: f32 = 3.0 * PI;
const CIRCLE_LEAVE: f32 = 0.5;
/// The scans remembered under the gaps (s). On the sweep bench (16 seeds) it halves the bumps
/// a room and misses less wall; a constant 1 cm off every gap halves them too, but misses
/// more. Mixed below 0.6 s.
const OBSTACLE_MEMORY: f32 = 2.0;
/// A pose moved this far in one tick (m) jumped (a loop closure): what was remembered is
/// dropped, not smeared. Driving covers at most 4 cm a tick.
const POSE_JUMP: f32 = 0.1;
/// `stock_follow_fix`: the wall-side beams a bump moves the wall in, and the most it moves
/// it (m): an overhang or a kick board, not glass the lidar sees nothing behind.
const BUMPED_FROM: usize = 90;
const BUMPED_TO: usize = 160;
const MAX_BUMPED: f32 = 0.1;
/// …and how far it holds past the last bump on it (m).
const BUMPED_REACH: f32 = 0.5;

/// The stock edge pass's configuration (`[exploration.stock_follow]`).
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(default)]
pub struct StockFollowConfig {
    pub side: WallSide,
    /// Stop after this much travel in total (m).
    pub max_distance: f32,
    /// The lap closes back within this distance of its start (m), this far over the seam.
    pub lap_radius: f32,
    pub lap_overlap: f32,
    /// Where our lidar sits ahead of the centre (m); from `[lidar] mounting_x`. This and the
    /// next two are set by `build_exploration`; their defaults are for tests.
    #[serde(skip)]
    pub lidar_x: f32,
    /// Its optical offset (m); from `[lidar] optical_offset`.
    #[serde(skip)]
    pub optical_offset: f32,
    /// The tick (s); from `[exploration] loop_rate_hz`.
    #[serde(skip)]
    pub tick: f32,
    /// `stock_follow_fix`: walls may stand in front of what the lidar sees (overhangs), so a
    /// bump on the wall side moves the wall to where the bumper found it (set by
    /// `build_exploration` from the strategy).
    #[serde(skip)]
    pub overhangs: bool,
    /// `stock_follow_fix`: once loose, loose for the run (set like `overhangs`).
    #[serde(skip)]
    pub stays_loose: bool,
}

impl Default for StockFollowConfig {
    fn default() -> Self {
        Self {
            side: WallSide::Right,
            max_distance: 300.0,
            lap_radius: 0.3,
            lap_overlap: 0.5,
            lidar_x: -0.105,
            optical_offset: 0.0,
            tick: 0.1,
            overhangs: false,
            stays_loose: false,
        }
    }
}

/// What the turns after a failure are for (the stock's `mode`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    VirtualBumper,
    Cliff,
    Bumper,
    /// Facing the nearest return to drive to it.
    FindWall,
}

/// The turns of one recovery: their step (rad), how many were still blocked, and how many
/// may be. Each failure starts a fresh one, which is the stock's "clear resets the
/// counters and the step to 0.18".
#[derive(Debug, Clone, Copy)]
struct Recovery {
    mode: Mode,
    step: f32,
    tries: u32,
    max_tries: u32,
}

impl Recovery {
    fn new(mode: Mode, step: f32, max_tries: u32) -> Self {
        Self {
            mode,
            step,
            tries: 0,
            max_tries,
        }
    }
}

/// Tight or loose (§9): loose widens S1's and S2's setpoints after bumps along a straight.
#[derive(Debug, Clone, Copy)]
struct Baseboard {
    tight: bool,
    /// Bumps while following, tight.
    bumps: u32,
    /// Updates with a line on the left only, loose.
    left_only: u32,
    /// The heading when it went loose (rad).
    heading: f32,
    /// `stock_follow_fix`: once loose, loose for the rest of the run.
    stays_loose: bool,
}

impl Baseboard {
    fn new(stays_loose: bool) -> Self {
        Self {
            tight: true,
            bumps: 0,
            left_only: 0,
            heading: 0.0,
            stays_loose,
        }
    }

    /// Tight again, the bumps forgotten; never if it stays loose.
    fn tighten(&mut self) {
        if !self.stays_loose {
            *self = Self::new(false);
        }
    }

    /// One update with the line flags (`updateState`); whether to restart following.
    /// Loose, a left-only line counts while `following`; any other tick checks for a corner.
    fn update(&mut self, right: bool, left: bool, heading: f32, following: bool) -> bool {
        if self.tight {
            if right && !left && self.bumps > LOOSE_BUMPS {
                *self = Self {
                    tight: false,
                    bumps: 0,
                    left_only: 0,
                    heading,
                    ..*self
                };
            }
            return false;
        }
        if left && !right && following {
            self.left_only += 1;
            if self.left_only > LEFT_ONLY_UPDATES {
                // Staying loose, it never left following: nothing to restart.
                self.tighten();
                return !self.stays_loose;
            }
        } else if angle_diff(self.heading, heading).abs() > CORNER {
            self.tighten();
        }
        false
    }
}

/// The motion watchdogs (§10): stuck in place, or circling.
#[derive(Debug, Clone, Copy, Default)]
struct Watchdogs {
    /// The pose the stuck window started at and its ticks so far.
    window: Option<(Pose2D, u32)>,
    /// Where the circling count started and the turn since (rad).
    circle: Option<(Point2D, f32)>,
}

impl Watchdogs {
    /// One tick at `pose`, having turned `turn`; which fired.
    fn update(&mut self, pose: &Pose2D, turn: f32, tick: f32) -> Option<&'static str> {
        let (from, ticks) = self.window.get_or_insert((*pose, 0));
        *ticks += 1;
        if *ticks as f32 * tick >= STUCK_WINDOW {
            let stuck = (pose.x - from.x).abs() < STUCK_MOVE
                && (pose.y - from.y).abs() < STUCK_MOVE
                && angle_diff(from.theta, pose.theta).abs() < STUCK_TURN;
            self.window = Some((*pose, 0));
            if stuck {
                return Some("stuck");
            }
        }
        let here = Point2D::new(pose.x, pose.y);
        let (anchor, turned) = self.circle.get_or_insert((here, 0.0));
        if (here.x - anchor.x).hypot(here.y - anchor.y) >= CIRCLE_LEAVE {
            self.circle = Some((here, 0.0));
        } else {
            *turned += turn;
            if turned.abs() > CIRCLE_TURN {
                self.circle = None;
                return Some("circling");
            }
        }
        None
    }
}

/// The stock edge pass.
pub struct StockFollower {
    config: StockFollowConfig,
    shell: Shell,
    action: Action,
    cmd: Command,
    recovery: Recovery,
    /// The angle the next Arc turns through (rad).
    arc_angle: f32,
    /// Whether the running Arc's angle is `ARC_ANGLE_AFTER_LINE`, and how many such arcs
    /// finished in a row.
    quarter_arc: bool,
    quarter_arcs: u32,
    baseboard: Baseboard,
    watchdogs: Watchdogs,
    travel: Travel,
    /// Started the first tick it follows a wall in S1 or S2.
    lap: Lap,
    started: bool,
    done: bool,
    /// The last `OBSTACLE_MEMORY` of scans (world frame, newest first) and the pose of the
    /// newest.
    remembered: VecDeque<PointCloud2D>,
    remembered_at: Option<Pose2D>,
    /// `stock_follow_fix`: the wall the bumper found in front of what the lidar sees.
    bumped: Option<Bumped>,
}

/// How far in front of what the lidar sees the bumper found the wall (m), and the distance
/// travelled at the last such bump (m).
#[derive(Debug, Clone, Copy)]
struct Bumped {
    by: f32,
    at: f32,
}

impl StockFollower {
    pub fn new(config: StockFollowConfig) -> Self {
        Self {
            config,
            shell: Shell::new(config.lidar_x, config.optical_offset),
            action: Action::follow(),
            cmd: Command::default(),
            recovery: Recovery::new(Mode::VirtualBumper, VIRTUAL_BUMPER_TURN, NUDGE_TRIES),
            arc_angle: ARC_ANGLE,
            quarter_arc: false,
            quarter_arcs: 0,
            baseboard: Baseboard::new(config.stays_loose),
            watchdogs: Watchdogs::default(),
            travel: Travel::default(),
            lap: Lap::new(config.lap_radius, config.lap_overlap),
            started: false,
            done: false,
            remembered: VecDeque::new(),
            remembered_at: None,
            bumped: None,
        }
    }

    /// Remembers `scan`, taken at `pose`, for `OBSTACLE_MEMORY` past this tick.
    fn remember(&mut self, scan: &PointCloud2D, pose: &Pose2D) {
        if self
            .remembered_at
            .is_some_and(|at| (pose.x - at.x).hypot(pose.y - at.y) > POSE_JUMP)
        {
            self.remembered.clear();
        }
        self.remembered.push_front(scan.transform(pose));
        let ticks = (OBSTACLE_MEMORY / self.config.tick).round() as usize;
        self.remembered.truncate(ticks + 1);
        self.remembered_at = Some(*pose);
    }

    /// The remembered scans seen from `pose`, mirrored: what the gaps take the least of
    /// (`getFollowWallIR` with the obstacle map under it).
    fn remembered_from(&self, pose: &Pose2D) -> Vec<Point2D> {
        let here = pose.inverse();
        (self.remembered.iter())
            .flat_map(|world| self.mirrored(&world.transform(&here)))
            .collect()
    }

    /// The scan in the mirrored robot frame: the wall on the right.
    fn mirrored(&self, scan: &PointCloud2D) -> Vec<Point2D> {
        let m = self.config.side.mirror();
        (scan.xs.iter().zip(&scan.ys))
            .map(|(&x, &y)| Point2D::new(x, m * y))
            .collect()
    }

    /// A contact in the mirrored frame: `BumperRight` is the wall side's.
    fn mirror_kind(&self, kind: HazardType) -> HazardType {
        match self.config.side {
            WallSide::Right => kind,
            WallSide::Left => kind.mirrored(),
        }
    }

    /// Whether the fan of beams `a..=b` holds a line of returns nearer than `LINE_RANGE`
    /// (`updataFollowWallLine`).
    fn fan_has_line(&self, points: &[Point2D], a: usize, b: usize) -> bool {
        let fan: Vec<Point2D> = (points.iter())
            .filter(|p| {
                let (i, range) = self.shell.beam(p);
                (a..=b).contains(&i) && range < LINE_RANGE
            })
            .copied()
            .collect();
        fan.len() >= LINE_POINTS
            && Line2D::fit(&fan).is_some_and(|line| {
                let sq: f32 = fan.iter().map(|p| line.distance(p).powi(2)).sum();
                (sq / fan.len() as f32).sqrt() <= LINE_RMS
            })
    }

    /// The start: the first entry's turn with only the left-front blocked, else to the
    /// wall nearby, else find one.
    fn first_action(&mut self, gaps: &Gaps, pose: &Pose2D) -> Action {
        // `processFirstEnter` as a virtual bumper's single turn: clear, it follows.
        if !gaps.bumper(123, 180, 0.075) && gaps.bumper(180, 237, 0.075) {
            self.recovery = Recovery::new(Mode::VirtualBumper, FIRST_ENTER_TURN, 1);
            return Action::rotation(FIRST_ENTER_TURN);
        }
        if gaps.follow_ir(90, 160).0 > FIND_WALL_GAP {
            return self.find_wall(gaps, pose);
        }
        Action::follow()
    }

    /// Face the nearest return, then drive to it (a Line until something is in the way).
    fn find_wall(&mut self, gaps: &Gaps, pose: &Pose2D) -> Action {
        self.recovery = Recovery::new(Mode::FindWall, 0.0, 1);
        match gaps.follow_ir(0, gaps::BEAMS - 1).1 {
            Some(i) => Action::rotation(beam_angle(i)),
            None => Action::line(pose, None),
        }
    }

    /// `processStateFail`: the recipe for `cause` (§7).
    fn on_failed(&mut self, cause: Cause, gaps: &Gaps, pose: &Pose2D) -> Action {
        match self.action {
            // BackAlign is not failed through: it goes on to its turn either way.
            Action::BackAlign { .. } => return Action::rotation(self.recovery.step),
            Action::Follow { .. } => {
                // A bump counts while tight; loose, it only tightens, which zeroes the bumps.
                if !self.baseboard.tight {
                    self.baseboard.tighten();
                } else if let Cause::Bumper(_) = cause {
                    self.baseboard.bumps += 1;
                }
                self.cmd = Command::default();
            }
            // failStop: Arc and Line stop dead too; Rotation carries on from its command.
            Action::Arc { .. } | Action::Line { .. } => self.cmd = Command::default(),
            Action::Rotation { .. } => {}
        }
        match cause {
            Cause::Cliff => {
                self.recovery = Recovery::new(Mode::Cliff, CLIFF_TURN, 1);
                Action::back_align(CLIFF_BACK)
            }
            Cause::Bumper(HazardType::BumperRight) => {
                // The wall is where the bumper is, this far in front of the side fan's gap.
                // More than `MAX_BUMPED` (or no return, `CLEAR`) is not a wall in front.
                let unseen = gaps.follow_ir(120, 143).0.max(0.0);
                let by = self.bumped.map_or(0.0, |b| b.by) + unseen;
                if self.config.overhangs && unseen <= MAX_BUMPED && by > 0.0 {
                    self.bumped = Some(Bumped {
                        by: by.min(MAX_BUMPED),
                        at: self.travel.distance,
                    });
                }
                // A graze: the side close and the front open turns once, a little more.
                let graze = gaps.follow_ir(123, 143).0 < 0.03 && gaps.follow_ir(144, 156).0 > 0.1;
                self.recovery = if graze {
                    Recovery::new(Mode::Bumper, GRAZE_TURN, 1)
                } else {
                    Recovery::new(Mode::Bumper, NUDGE_TURN, NUDGE_TRIES)
                };
                Action::back_align(BUMPER_BACK)
            }
            Cause::Bumper(_) => {
                self.recovery = Recovery::new(Mode::Bumper, BUMPER_OTHER_TURN, 1);
                Action::back_align(BUMPER_BACK)
            }
            Cause::VirtualBumper => {
                let step = virtual_bumper_turn(gaps);
                self.recovery = Recovery::new(Mode::VirtualBumper, step, NUDGE_TRIES);
                Action::rotation(step)
            }
            // Only a BackAlign is blocked, and it was handled above.
            Cause::Blocked | Cause::Timeout => {
                log::debug!("stock_follow: {:?} timed out at {pose:?}", self.action);
                Action::follow()
            }
        }
    }

    /// The transfer once the action is done.
    fn on_finished(&mut self, gaps: &Gaps, bumper: bool, pose: &Pose2D) -> Action {
        match self.action {
            Action::Follow { .. } => Action::follow(),
            Action::Arc { .. } => {
                self.quarter_arcs = if self.quarter_arc {
                    self.quarter_arcs + 1
                } else {
                    0
                };
                if self.quarter_arcs < QUARTER_ARCS {
                    return Action::follow();
                }
                log::debug!("stock_follow: arcing in a circle, finding a wall");
                self.quarter_arcs = 0;
                self.find_wall(gaps, pose)
            }
            Action::Line { .. } => {
                self.arc_angle = ARC_ANGLE_AFTER_LINE;
                Action::follow()
            }
            Action::BackAlign { .. } => Action::rotation(self.recovery.step),
            Action::Rotation { .. } => {
                let r = &mut self.recovery;
                if r.mode == Mode::FindWall {
                    return Action::line(pose, None);
                }
                if !gaps.bumper(160, 205, 0.05) && !bumper {
                    return match r.mode {
                        Mode::Cliff | Mode::Bumper => Action::line(pose, Some(RELAUNCH)),
                        Mode::VirtualBumper | Mode::FindWall => Action::follow(),
                    };
                }
                r.tries += 1;
                if r.tries < r.max_tries {
                    return Action::rotation(r.step);
                }
                match r.mode {
                    Mode::Cliff => Action::line(pose, Some(CLIFF_GIVE_UP)),
                    Mode::Bumper => Action::line(pose, Some(BUMPER_GIVE_UP)),
                    Mode::VirtualBumper | Mode::FindWall => Action::follow(),
                }
            }
        }
    }

    /// Moves the state machine on until an action runs this tick.
    fn transfer(&mut self, s: &Sensed) {
        // A few hand-overs at most (fail → retract → turn); bounded against a cycle.
        for _ in 0..HAND_OVERS {
            let next = match self.action.status(s) {
                Status::Running => match self.action {
                    Action::Follow { ticks, .. }
                        if ticks > 0 && s.gaps.follow_ir(120, 143).0 >= 0.03 =>
                    {
                        let angle = std::mem::replace(&mut self.arc_angle, ARC_ANGLE);
                        self.quarter_arc = angle == ARC_ANGLE_AFTER_LINE;
                        Action::arc(angle)
                    }
                    _ => return,
                },
                Status::Finished => self.on_finished(s.gaps, s.bumper.is_some(), s.pose),
                Status::Failed(cause) => self.on_failed(cause, s.gaps, s.pose),
            };
            log::debug!(
                "stock_follow at ({:.2}, {:.2}, {:.2}) mirrored: {} -> {}",
                s.pose.x,
                s.pose.y,
                s.pose.theta,
                self.action.reason(),
                next.reason()
            );
            self.action = next;
        }
    }
}

/// `processVirtualBumperFail`: the turn by where the nearest thing ahead is (rad).
fn virtual_bumper_turn(gaps: &Gaps) -> f32 {
    match gaps.follow_ir(160, 200).1 {
        None => VIRTUAL_BUMPER_TURN,
        Some(i) if i >= 180 => 60f32.to_radians(),
        Some(i) => (2.0 * (i as f32 - 135.0) - 30.0).to_radians(),
    }
}

impl ExplorationStrategy for StockFollower {
    fn name(&self) -> &'static str {
        if self.config.overhangs {
            "stock_follow_fix"
        } else {
            "stock_follow"
        }
    }

    fn next_action(
        &mut self,
        _map: &CurrentMapData,
        current_pose: &Pose2D,
        scan: &PointCloud2D,
        hazard: Option<&HazardEvent>,
        held: Option<HazardType>,
    ) -> ExplorationAction {
        let pose = current_pose;
        let m = self.config.side.mirror();
        let turn = self.travel.update(pose);
        self.lap.turned(turn);
        self.action.turned(m * turn);
        if self.done
            || self.lap.done(pose, self.travel.distance)
            || self.travel.distance > self.config.max_distance
        {
            self.done = true;
            return ExplorationAction::Complete;
        }

        let points = self.mirrored(scan);
        self.remember(scan, pose);
        let mut gaps = Gaps::new(&self.shell, &self.remembered_from(pose));
        // The actions' pose, mirrored with the scan; the lap and watchdogs keep the real one.
        let mirrored = Pose2D::new(pose.x, m * pose.y, m * pose.theta);
        // A wall the bumper found holds for `BUMPED_REACH` past the last bump on it.
        let travelled = self.travel.distance;
        self.bumped = self.bumped.filter(|b| travelled - b.at <= BUMPED_REACH);
        if let Some(b) = self.bumped {
            gaps.nearer(BUMPED_FROM, BUMPED_TO, b.by);
        }

        if let Some(why) = self.watchdogs.update(pose, turn, self.config.tick) {
            log::debug!("{} at {pose:?}: {why}, finding a wall", self.name());
            self.cmd = Command::default();
            self.action = self.find_wall(&gaps, &mirrored);
            self.bumped = None;
        }
        let right = self.fan_has_line(&points, 90, 160);
        let left = self.fan_has_line(&points, 191, 249);
        let following = matches!(self.action, Action::Follow { .. });
        if self.baseboard.update(right, left, pose.theta, following) {
            self.action = Action::follow();
        }
        if !self.started {
            self.started = true;
            self.action = self.first_action(&gaps, &mirrored);
        }

        let contacts = || hazard.map(|h| h.hazard_type).into_iter().chain(held);
        let bumper = contacts().find(|k| k.bumper_arc().is_some());
        let sensed = Sensed {
            gaps: &gaps,
            bumper: bumper.map(|k| self.mirror_kind(k)),
            cliff: contacts().any(|k| k.is_cliff()),
            pose: &mirrored,
            // Rubbing a wall the bumper found bumps it each time: loose off it.
            tight: self.baseboard.tight && self.bumped.is_none(),
            dt: self.config.tick,
        };
        self.transfer(&sensed);
        self.action.execute(&sensed, &mut self.cmd);
        if let Action::Follow {
            sector: Some(0 | 1),
            ..
        } = self.action
            && !gaps.bumper(160, 195, LAP_FRONT_GAP)
        {
            self.lap.start(pose);
            self.lap.along(pose.theta);
        }
        ExplorationAction::Drive {
            linear: self.cmd.v,
            angular: m * self.cmd.w,
            reason: self.action.reason(),
        }
    }

    fn reset(&mut self) {
        *self = Self::new(self.config);
    }
}

#[cfg(test)]
mod tests {
    use super::gaps::tests::{Segment, ray_scan, right_wall};
    use super::*;

    const R: f32 = 0.175;
    /// Our lidar where the shell table has it (and no optical offset): gaps as in the spec.
    const LIDAR_X: f32 = -0.105;
    const DT: f32 = 0.1;

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

    fn follower(side: WallSide) -> StockFollower {
        StockFollower::new(StockFollowConfig {
            side,
            ..StockFollowConfig::default()
        })
    }

    /// A follower already following, the wall at the tight equilibrium.
    fn following() -> StockFollower {
        let mut f = follower(WallSide::Right);
        tick(
            &mut f,
            &Pose2D::identity(),
            &right_wall(LIDAR_X, R, -0.011),
            None,
        );
        assert!(matches!(f.action, Action::Follow { .. }), "{:?}", f.action);
        f
    }

    fn tick(
        f: &mut StockFollower,
        pose: &Pose2D,
        points: &[Point2D],
        held: Option<HazardType>,
    ) -> (f32, f32) {
        let scan = PointCloud2D::from_points(points);
        let hazard = held.map(|k| HazardEvent::new(k, pose));
        match f.next_action(&no_map(), pose, &scan, hazard.as_ref(), held) {
            ExplorationAction::Drive {
                linear, angular, ..
            } => (linear, angular),
            other => panic!("expected Drive, got {other:?}"),
        }
    }

    /// A wall straight across, `gap` (m) ahead of the body.
    fn wall_ahead(gap: f32) -> Vec<Point2D> {
        ray_scan(LIDAR_X, &[((R + gap, -1.0), (R + gap, 1.0))])
    }

    #[test]
    fn a_graze_on_the_wall_side_retracts_millimetres_nudges_and_relaunches() {
        let mut f = following();
        let wall = right_wall(LIDAR_X, R, -0.011);
        let pose = Pose2D::identity();
        let (v, w) = tick(&mut f, &pose, &wall, Some(HazardType::BumperRight));
        assert!(
            matches!(f.action, Action::BackAlign { .. }),
            "{:?}",
            f.action
        );
        assert!(v < 0.0 && v > -0.06 && w == 0.0, "{v} {w}");
        assert_eq!(
            (f.recovery.mode, f.recovery.max_tries),
            (Mode::Bumper, NUDGE_TRIES)
        );
        assert_eq!(f.baseboard.bumps, 1);
        // Released: the nudge left…
        let (_, w) = tick(&mut f, &pose, &wall, None);
        assert!(
            matches!(f.action, Action::Rotation { .. }),
            "{:?}",
            f.action
        );
        assert!(w > 0.0 && w <= NUDGE_TURN / DT + 1e-4, "{w}");
        // …turned, the front clear: 5 cm straight, then following.
        let turned = Pose2D::new(0.0, 0.0, NUDGE_TURN);
        tick(&mut f, &turned, &wall, None);
        assert!(
            matches!(f.action, Action::Line { distance: Some(d), .. } if d == RELAUNCH),
            "{:?}",
            f.action
        );
        tick(&mut f, &Pose2D::new(RELAUNCH, 0.0, NUDGE_TURN), &wall, None);
        assert!(matches!(f.action, Action::Follow { .. }), "{:?}", f.action);
        assert_eq!(f.arc_angle, ARC_ANGLE_AFTER_LINE, "the next arc is seeded");
    }

    #[test]
    fn a_bump_elsewhere_turns_81_degrees_and_a_cliff_63_after_a_longer_retract() {
        for (kind, mode, turn) in [
            (HazardType::BumperLeft, Mode::Bumper, BUMPER_OTHER_TURN),
            (HazardType::BumperBoth, Mode::Bumper, BUMPER_OTHER_TURN),
            (HazardType::CliffRightFront, Mode::Cliff, CLIFF_TURN),
        ] {
            let mut f = following();
            tick(&mut f, &Pose2D::identity(), &[], Some(kind));
            assert!(matches!(f.action, Action::BackAlign { .. }), "{kind:?}");
            assert_eq!((f.recovery.mode, f.recovery.step), (mode, turn), "{kind:?}");
        }
    }

    #[test]
    fn a_wall_ahead_turns_part_of_the_corner_without_retracting() {
        let mut f = following();
        let mut corner = right_wall(LIDAR_X, R, -0.011);
        corner.extend(wall_ahead(0.03));
        let (v, w) = tick(&mut f, &Pose2D::identity(), &corner, None);
        assert_eq!(f.recovery.mode, Mode::VirtualBumper);
        assert_eq!(
            f.recovery.step,
            60f32.to_radians(),
            "the nearest beam is ahead"
        );
        assert!(
            matches!(f.action, Action::Rotation { .. }),
            "{:?}",
            f.action
        );
        assert!(v == 0.0 && w > 0.0, "failStop, then the spin: {v} {w}");
    }

    #[test]
    fn the_virtual_bumper_turns_by_where_the_nearest_return_is() {
        let shell = Shell::new(LIDAR_X, 0.0);
        assert_eq!(
            virtual_bumper_turn(&Gaps::new(&shell, &[])),
            VIRTUAL_BUMPER_TURN
        );
        // A post at beam 160: 20° left.
        let at = beam_angle(160);
        let post = Point2D::new(LIDAR_X + 0.3 * at.cos(), 0.3 * at.sin());
        let turn = virtual_bumper_turn(&Gaps::new(&shell, &[post]));
        assert!(
            (turn - 20f32.to_radians()).abs() < 1e-5,
            "{}",
            turn.to_degrees()
        );
    }

    #[test]
    fn still_blocked_it_turns_again_then_gives_up_and_follows() {
        let mut f = following();
        let blocked = wall_ahead(0.01);
        let mut heading = 0.0;
        tick(&mut f, &Pose2D::identity(), &blocked, None);
        let step = f.recovery.step;
        for tries in 1..NUDGE_TRIES {
            heading += step;
            // 15 turns of 60° in place would trip the circling watchdog first.
            f.watchdogs = Watchdogs::default();
            tick(&mut f, &Pose2D::new(0.0, 0.0, heading), &blocked, None);
            assert_eq!(f.recovery.tries, tries);
            assert!(
                matches!(f.action, Action::Rotation { .. }),
                "{:?}",
                f.action
            );
        }
        heading += step;
        tick(&mut f, &Pose2D::new(0.0, 0.0, heading), &blocked, None);
        // Given up: following, which the wall in front fails at once.
        assert_eq!(f.recovery.tries, 0, "a fresh recovery");
    }

    #[test]
    fn the_lap_starts_along_the_wall_not_facing_it() {
        let mut f = follower(WallSide::Right);
        f.started = true;
        let mut facing = right_wall(LIDAR_X, R, -0.011);
        facing.extend(wall_ahead(0.06));
        tick(&mut f, &Pose2D::identity(), &facing, None);
        assert!(
            matches!(
                f.action,
                Action::Follow {
                    sector: Some(0 | 1),
                    ..
                }
            ),
            "{:?}",
            f.action
        );
        assert!(f.lap.start.is_none() && !f.lap.aligned());
        // Turned alongside: the wall ahead gone, and out of the obstacle memory too (it would
        // take `OBSTACLE_MEMORY` of ticks).
        f.remembered.clear();
        tick(
            &mut f,
            &Pose2D::identity(),
            &right_wall(LIDAR_X, R, -0.011),
            None,
        );
        assert_eq!(f.lap.heading, Some(0.0), "along the wall");
    }

    #[test]
    fn the_fix_moves_the_wall_to_where_the_bumper_found_it_for_a_stretch() {
        // The lidar sees the wall 4 cm off; the bumper finds it at the body.
        let seen = right_wall(LIDAR_X, R, 0.04);
        let bump_at = |f: &mut StockFollower, wall: &[Point2D]| {
            f.action = Action::follow();
            f.remembered.clear();
            tick(f, &Pose2D::identity(), wall, Some(HazardType::BumperRight));
            f.bumped.map(|b| b.by)
        };
        let bump = |f: &mut StockFollower| bump_at(f, &seen);
        let fix = || {
            let mut f = StockFollower::new(StockFollowConfig {
                overhangs: true,
                ..StockFollowConfig::default()
            });
            tick(&mut f, &Pose2D::identity(), &seen, None);
            f
        };
        let mut stock = follower(WallSide::Right);
        tick(&mut stock, &Pose2D::identity(), &seen, None);
        assert_eq!(bump(&mut stock), None, "stock_follow ignores it");

        let mut f = fix();
        let unseen = Gaps::new(&f.shell, &seen).follow_ir(120, 143).0;
        let by = bump(&mut f).expect("the bumper found the wall");
        assert!((by - unseen).abs() < 1e-4, "{by} vs {unseen}");
        // Held on the next tick, then forgotten `BUMPED_REACH` further on.
        tick(&mut f, &Pose2D::identity(), &seen, None);
        assert!(f.bumped.is_some());
        f.travel.distance += BUMPED_REACH + 0.01;
        tick(&mut f, &Pose2D::identity(), &seen, None);
        assert!(f.bumped.is_none());

        // Bumped again at the same wall it reads no gap, and the wall stays put…
        let mut f = fix();
        bump(&mut f);
        assert_eq!(bump(&mut f), Some(by));
        // …but where it still reads one, it adds that, up to `MAX_BUMPED`.
        assert_eq!(
            bump_at(&mut f, &right_wall(LIDAR_X, R, 0.1)),
            Some(MAX_BUMPED)
        );
        // A bump on the far side moves nothing.
        let mut f = fix();
        f.action = Action::follow();
        tick(
            &mut f,
            &Pose2D::identity(),
            &seen,
            Some(HazardType::BumperLeft),
        );
        assert!(f.bumped.is_none());
    }

    #[test]
    fn bumps_along_a_straight_loosen_it_and_a_corner_tightens_it() {
        let mut f = following();
        let wall = right_wall(LIDAR_X, R, -0.011);
        for _ in 0..=LOOSE_BUMPS {
            f.action = Action::follow();
            tick(
                &mut f,
                &Pose2D::identity(),
                &wall,
                Some(HazardType::BumperRight),
            );
        }
        assert_eq!(f.baseboard.bumps, LOOSE_BUMPS + 1);
        tick(&mut f, &Pose2D::identity(), &wall, None);
        assert!(!f.baseboard.tight, "a line on the right only");
        tick(&mut f, &Pose2D::new(0.0, 0.0, 0.5), &wall, None);
        assert!(!f.baseboard.tight, "within 45°");
        tick(&mut f, &Pose2D::new(0.0, 0.0, 0.9), &wall, None);
        assert!(f.baseboard.tight, "a corner");
    }

    #[test]
    fn the_fix_stays_loose_round_corners_and_bumps() {
        let mut b = Baseboard {
            bumps: LOOSE_BUMPS + 1,
            ..Baseboard::new(true)
        };
        b.update(true, false, 0.0, true);
        assert!(!b.tight, "loose");
        b.update(false, false, 1.0, true);
        assert!(!b.tight, "past a corner");
        for _ in 0..=LEFT_ONLY_UPDATES {
            assert!(!b.update(false, true, 1.0, true), "no restart");
        }
        b.tighten();
        assert!(!b.tight, "a bump while loose");
    }

    #[test]
    fn left_only_lines_tighten_it_only_while_following() {
        let loose = Baseboard {
            tight: false,
            ..Baseboard::new(false)
        };
        let mut b = loose;
        for _ in 0..=LEFT_ONLY_UPDATES {
            assert!(!b.update(false, true, 0.0, false));
        }
        assert!(!b.tight && b.left_only == 0, "not following: not counted");
        let mut b = loose;
        for _ in 0..LEFT_ONLY_UPDATES {
            assert!(!b.update(false, true, 0.0, true));
        }
        assert!(b.update(false, true, 0.0, true), "following: restart");
        assert!(b.tight);
    }

    #[test]
    fn remembered_returns_stay_where_they_were_for_the_memory_then_go() {
        let mut f = follower(WallSide::Right);
        let leg = PointCloud2D::from_points(&[Point2D::new(0.3, -0.2)]);
        f.remember(&leg, &Pose2D::identity());
        // 5 cm on: still there, 5 cm nearer in the robot frame.
        let moved = Pose2D::new(0.05, 0.0, 0.0);
        let ticks = (OBSTACLE_MEMORY / DT).round() as usize;
        for _ in 0..ticks {
            f.remember(&PointCloud2D::new(), &moved);
        }
        let seen = f.remembered_from(&moved);
        assert_eq!(seen.len(), 1);
        assert!((seen[0].x - 0.25).abs() < 1e-5 && (seen[0].y + 0.2).abs() < 1e-5);
        f.remember(&PointCloud2D::new(), &moved);
        assert!(f.remembered_from(&moved).is_empty(), "forgotten");
    }

    #[test]
    fn a_pose_jump_forgets_what_was_remembered() {
        let mut f = follower(WallSide::Right);
        let leg = PointCloud2D::from_points(&[Point2D::new(0.3, -0.2)]);
        f.remember(&leg, &Pose2D::identity());
        f.remember(&PointCloud2D::new(), &Pose2D::new(0.2, 0.0, 0.0));
        assert!(f.remembered_from(&Pose2D::identity()).is_empty());
    }

    #[test]
    fn the_first_entry_turns_left_from_something_on_the_left_front() {
        let mut f = follower(WallSide::Right);
        // Within 7.5 cm of the body, beams 191–232 only.
        let left_front = ray_scan(LIDAR_X, &[((0.05, 0.2), (0.2, 0.06))]);
        tick(&mut f, &Pose2D::identity(), &left_front, None);
        assert!(
            matches!(f.action, Action::Rotation { left, .. } if left == FIRST_ENTER_TURN),
            "{:?}",
            f.action
        );
    }

    #[test]
    fn away_from_walls_it_faces_the_nearest_and_drives_to_it() {
        let mut f = follower(WallSide::Right);
        // A wall 1 m to the left.
        let far = ray_scan(LIDAR_X, &[((-3.0, 1.0), (3.0, 1.0))]);
        let (_, w) = tick(&mut f, &Pose2D::identity(), &far, None);
        assert_eq!(f.recovery.mode, Mode::FindWall);
        assert!(w > 0.0, "{w}");
        // Facing it: straight to it.
        tick(&mut f, &Pose2D::new(0.0, 0.0, PI / 2.0), &far, None);
        assert!(
            matches!(f.action, Action::Line { distance: None, .. }),
            "{:?}",
            f.action
        );
    }

    #[test]
    fn circling_in_place_trips_the_watchdog() {
        let mut dogs = Watchdogs::default();
        let mut theta = 0.0;
        let fired = (0..100).find_map(|_| {
            theta += 0.2;
            dogs.update(&Pose2D::new(0.0, 0.0, theta), 0.2, DT)
        });
        assert_eq!(fired, Some("circling"));
        let mut dogs = Watchdogs::default();
        let fired = (0..40).find_map(|_| dogs.update(&Pose2D::identity(), 0.0, DT));
        assert_eq!(fired, Some("stuck"));
    }

    #[test]
    fn a_wall_on_the_left_mirrors_the_right() {
        let mut right = following();
        let mut left = follower(WallSide::Left);
        let mirrored = |p: &[Point2D]| {
            p.iter()
                .map(|p| Point2D::new(p.x, -p.y))
                .collect::<Vec<_>>()
        };
        tick(
            &mut left,
            &Pose2D::identity(),
            &mirrored(&right_wall(LIDAR_X, R, -0.011)),
            None,
        );
        // Closer than the setpoint: away from the wall.
        let close = right_wall(LIDAR_X, R, -0.02);
        let (vr, wr) = tick(&mut right, &Pose2D::identity(), &close, None);
        let (vl, wl) = tick(&mut left, &Pose2D::identity(), &mirrored(&close), None);
        assert!(wr > 0.0, "{wr}");
        assert_eq!((vl, wl), (vr, -wr));
        let (_, _) = tick(
            &mut left,
            &Pose2D::identity(),
            &[],
            Some(HazardType::BumperLeft),
        );
        assert_eq!(
            left.recovery.max_tries, NUDGE_TRIES,
            "the wall side's bumper"
        );
    }

    #[test]
    fn a_line_on_the_left_holds_its_heading_too() {
        let mut f = follower(WallSide::Left);
        f.started = true;
        f.action = Action::line(&Pose2D::identity(), Some(1.0));
        let (_, w) = tick(&mut f, &Pose2D::new(0.0, 0.0, 0.1), &[], None);
        assert!(w < 0.0, "back right to the start's heading: {w}");
    }

    #[test]
    fn a_second_quarter_arc_in_a_row_stops_it_to_find_a_wall() {
        let far = ray_scan(LIDAR_X, &[((-3.0, 1.0), (3.0, 1.0))]);
        for (before, mode) in [(0, None), (1, Some(Mode::FindWall))] {
            let mut f = following();
            f.action = Action::Arc {
                left: 0.0,
                ticks: 5,
            };
            (f.quarter_arc, f.quarter_arcs) = (true, before);
            tick(&mut f, &Pose2D::identity(), &far, None);
            match mode {
                None => assert!(matches!(f.action, Action::Follow { .. }), "{:?}", f.action),
                Some(mode) => assert_eq!(f.recovery.mode, mode),
            }
        }
    }

    /// Drives a unicycle among walls, the lidar ray-cast; the body may go into them (no
    /// bumpers here). Returns whether the lap closed and the clearance each tick (m).
    fn drive(room: &[Segment], mut pose: Pose2D, ticks: usize) -> (bool, Vec<f32>) {
        let mut f = follower(WallSide::Right);
        let mut clearance = Vec::new();
        for _ in 0..ticks {
            let inv = pose.inverse();
            let local: Vec<_> = (room.iter())
                .map(|&(a, b)| {
                    let a = inv.compose(&Pose2D::new(a.0, a.1, 0.0));
                    let b = inv.compose(&Pose2D::new(b.0, b.1, 0.0));
                    ((a.x, a.y), (b.x, b.y))
                })
                .collect();
            let scan = PointCloud2D::from_points(&ray_scan(LIDAR_X, &local));
            match f.next_action(&no_map(), &pose, &scan, None, None) {
                ExplorationAction::Complete => return (true, clearance),
                ExplorationAction::Drive {
                    linear, angular, ..
                } => {
                    pose = Pose2D::new(
                        pose.x + linear * pose.theta.cos() * DT,
                        pose.y + linear * pose.theta.sin() * DT,
                        crate::core::math::normalize_angle(pose.theta + angular * DT),
                    );
                }
                other => panic!("{other:?}"),
            }
            let to_wall = |&((x1, y1), (x2, y2)): &Segment| {
                let (ex, ey) = (x2 - x1, y2 - y1);
                let s = (((pose.x - x1) * ex + (pose.y - y1) * ey) / (ex * ex + ey * ey))
                    .clamp(0.0, 1.0);
                (pose.x - x1 - s * ex).hypot(pose.y - y1 - s * ey) - R
            };
            clearance.push(room.iter().map(to_wall).fold(f32::INFINITY, f32::min));
        }
        (false, clearance)
    }

    #[test]
    fn on_a_straight_wall_it_settles_grazing_it_as_the_stock_does() {
        // §3: tight, S1 holds its 5.5 cm at a clearance of −1.1 cm, the body 1 cm into the
        // shell model. Started 3 cm off, parallel.
        let wall = [((-1.0, 0.0), (20.0, 0.0))];
        let (_, clearance) = drive(&wall, Pose2D::new(0.0, R + 0.03, 0.0), 150);
        let settled = &clearance[100..];
        let mean = settled.iter().sum::<f32>() / settled.len() as f32;
        assert!((mean + 0.011).abs() < 0.003, "settled at {mean}");
    }

    #[test]
    fn it_closes_a_lap_round_a_room_hugging_its_walls() {
        let (w, h) = (3.0, 2.5);
        let room = [
            ((0.0, 0.0), (w, 0.0)),
            ((w, 0.0), (w, h)),
            ((w, h), (0.0, h)),
            ((0.0, h), (0.0, 0.0)),
        ];
        // Mid-room: it finds a wall first.
        let (closed, clearance) = drive(&room, Pose2D::new(1.5, 1.2, 0.3), 3000);
        assert!(closed, "no lap");
        let least = clearance.iter().copied().fold(f32::INFINITY, f32::min);
        assert!(least > -0.03, "into the wall: {least}");
    }
}
