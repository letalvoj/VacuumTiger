//! Global relocalisation: where in the floor plan is the robot, starting from nothing.
//!
//! The first scan is matched everywhere the robot could stand: one coarse correlative search
//! (all headings) per tile of the plan's free space, so each region of the home keeps its
//! best explanation as a hypothesis instead of one global winner taking all. The best
//! hypotheses are refined, then followed through the robot's motion (from the session's own
//! SLAM poses) and re-scored on every scan. A symmetric room or a moved sofa makes several
//! hypotheses fit at first; commit only once one has led the others by a margin for a few
//! scans in a row (Cartographer's and AMCL's "late commit"). The result is the transform
//! from the session frame to the plan frame: what `FloorPlan::merge` takes, and the prior
//! the SLAM's localisation mode needs. It is as accurate as the plan's grid (a few cm); the
//! session's own tracking refines from there.
//!
//! The tiled search costs about a million score lookups per m² of home, seconds on the
//! robot's CPU: fine once per run. Cartographer's branch-and-bound global search
//! (`vastu-slam/src/matching/branch_bound`) would be the faster drop-in for the tiles.

use crate::algorithms::mapping::{CellState, OccupancyGrid};
use crate::algorithms::matching::{CorrelativeConfig, CorrelativeMatcher, ScanMatcher};
use crate::core::math::angle_diff;
use crate::core::types::{PointCloud2D, Pose2D};

/// Placement tuning.
#[derive(Debug, Clone)]
pub struct RelocaliserConfig {
    /// Side of the square regions searched separately (m).
    pub tile: f32,
    /// Position step of the coarse search (m); also its score grid's cell.
    pub coarse_step: f32,
    /// Heading step of the coarse search (rad).
    pub coarse_heading_step: f32,
    /// Distinct hypotheses kept after the coarse search.
    pub hypotheses: usize,
    /// A hypothesis whose running score drops below this is dropped (0-1, the matcher's
    /// mean smeared score).
    pub min_score: f32,
    /// To commit, the leader's running score must reach this...
    pub commit_score: f32,
    /// ...be this far ahead of the runner-up's...
    pub commit_margin: f32,
    /// ...for this many scans in a row.
    pub commit_scans: usize,
    /// A search that has not committed after this many scans counts as lost.
    pub give_up_scans: usize,
    /// Most scan points used by the coarse search (the scan is thinned evenly).
    pub coarse_points: usize,
}

impl Default for RelocaliserConfig {
    fn default() -> Self {
        Self {
            tile: 1.0,
            coarse_step: 0.1,
            coarse_heading_step: 5f32.to_radians(),
            hypotheses: 8,
            min_score: 0.55,
            commit_score: 0.7,
            commit_margin: 0.08,
            commit_scans: 5,
            give_up_scans: 300, // a minute of scans
            coarse_points: 120,
        }
    }
}

/// A candidate pose of the robot in the plan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hypothesis {
    pub pose: Pose2D,
    /// Running (exponentially weighted) match score.
    pub score: f32,
}

/// Where the search stands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Placement {
    /// Several hypotheses still fit.
    Searching,
    /// Found: the session-to-plan transform.
    Committed(Pose2D),
    /// Nothing in the plan explains the scans (another home, furniture moved a lot, or
    /// the robot was carried). The next scan starts the search afresh.
    Lost,
}

// Fixed by the matching rather than tuned per home: the running score's weight of the
// newest scan, and how close (m, rad) two hypotheses are to be the same.
const SCORE_GAIN: f32 = 0.3;
const SAME_POSITION: f32 = 0.2;
const SAME_HEADING: f32 = 0.2;

/// Searches the floor plan for the robot, one scan at a time.
pub struct Relocaliser {
    config: RelocaliserConfig,
    /// Plan walls, the matchers' target.
    walls: PointCloud2D,
    /// Centres of the tiles holding free plan space.
    tiles: Vec<(f32, f32)>,
    coarse: CorrelativeMatcher,
    fine: CorrelativeMatcher,
    hypotheses: Vec<Hypothesis>,
    /// Session pose at the previous scan.
    last_session_pose: Option<Pose2D>,
    /// Scans the current leader has led by the margin.
    lead: usize,
    /// Scans since the search (re)started.
    searched: usize,
    state: Placement,
}

impl Relocaliser {
    /// A search of `plan` (walls occupied, where the robot can stand free), e.g.
    /// `FloorPlan::to_grid`.
    pub fn new(plan: &OccupancyGrid, config: RelocaliserConfig) -> Self {
        let mut tiles: Vec<(i32, i32)> = Vec::new();
        let (w, h) = plan.dimensions();
        for cy in 0..h {
            for cx in 0..w {
                if plan.get_state(cx, cy) == CellState::Free {
                    let (x, y) = plan.cell_to_world(cx, cy);
                    tiles.push((
                        (x / config.tile).floor() as i32,
                        (y / config.tile).floor() as i32,
                    ));
                }
            }
        }
        tiles.sort_unstable();
        tiles.dedup();
        let half = config.tile / 2.0;
        let coarse = CorrelativeMatcher::new(CorrelativeConfig {
            search_window_x: half,
            search_window_y: half,
            search_window_theta: std::f32::consts::PI,
            linear_resolution: config.coarse_step,
            angular_resolution: config.coarse_heading_step,
            grid_resolution: config.coarse_step,
            min_score: 0.0,
        });
        // Refines a coarse pose and follows one through a scan's worth of motion, to the
        // SLAM front end's precision.
        let fine = CorrelativeMatcher::new(CorrelativeConfig {
            search_window_x: config.coarse_step,
            search_window_y: config.coarse_step,
            search_window_theta: config.coarse_heading_step,
            linear_resolution: 0.02,
            angular_resolution: 0.01,
            grid_resolution: 0.05,
            min_score: 0.0,
        });
        Self {
            walls: plan.as_pointcloud(),
            tiles: tiles
                .into_iter()
                .map(|(i, j)| {
                    (
                        (i as f32 + 0.5) * config.tile,
                        (j as f32 + 0.5) * config.tile,
                    )
                })
                .collect(),
            config,
            coarse,
            fine,
            hypotheses: Vec::new(),
            last_session_pose: None,
            lead: 0,
            searched: 0,
            state: Placement::Searching,
        }
    }

    /// The hypotheses still in play, best first (for showing the belief).
    pub fn hypotheses(&self) -> &[Hypothesis] {
        &self.hypotheses
    }

    /// Feeds one scan (robot frame) taken at `session_pose` (the session SLAM's pose).
    /// Once committed, the answer stays; make a new relocaliser to search again. The scan
    /// that (re)starts the search takes the whole tiled search, seconds on the robot: a
    /// live caller runs this off the SLAM thread.
    pub fn update(&mut self, scan: &PointCloud2D, session_pose: &Pose2D) -> Placement {
        if matches!(self.state, Placement::Committed(_)) || scan.is_empty() {
            return self.state;
        }
        if self.state == Placement::Lost {
            (self.last_session_pose, self.lead, self.searched) = (None, 0, 0);
            self.state = Placement::Searching;
        }
        match self.last_session_pose.replace(*session_pose) {
            None => self.search_everywhere(scan),
            Some(last) => self.follow(scan, &last.inverse().compose(session_pose)),
        }
        self.searched += 1;
        self.hypotheses.retain(|h| h.score >= self.config.min_score);
        self.hypotheses.sort_by(|a, b| b.score.total_cmp(&a.score));

        let Some(best) = self
            .hypotheses
            .first()
            .copied()
            .filter(|_| self.searched <= self.config.give_up_scans)
        else {
            self.state = Placement::Lost;
            return self.state;
        };
        // Running averages, except on the first scan.
        let leads = best.score >= self.config.commit_score
            && self
                .hypotheses
                .get(1)
                .is_none_or(|second| best.score - second.score >= self.config.commit_margin);
        self.lead = if leads { self.lead + 1 } else { 0 };
        if self.lead >= self.config.commit_scans {
            self.state = Placement::Committed(best.pose.compose(&session_pose.inverse()));
        }
        self.state
    }

    /// The best pose per tile; of those, the best few distinct ones refined.
    fn search_everywhere(&mut self, scan: &PointCloud2D) {
        let stride = scan.len().div_ceil(self.config.coarse_points).max(1);
        let thin = PointCloud2D::from_points(&scan.iter().step_by(stride).collect::<Vec<_>>());
        let mut found: Vec<Hypothesis> = self
            .tiles
            .iter()
            .map(|&(x, y)| {
                let r = self
                    .coarse
                    .match_scans(&thin, &self.walls, &Pose2D::new(x, y, 0.0));
                Hypothesis {
                    pose: r.transform,
                    score: r.score,
                }
            })
            .collect();
        // Neighbouring tiles often find the same pose: keep the best few distinct ones.
        found.sort_by(|a, b| b.score.total_cmp(&a.score));
        let mut distinct = Vec::new();
        for h in found {
            if distinct.len() == self.config.hypotheses {
                break;
            }
            add_distinct(&mut distinct, h);
        }
        self.hypotheses.clear();
        for h in distinct {
            let r = self.fine.match_scans(scan, &self.walls, &h.pose);
            let refined = Hypothesis {
                pose: r.transform,
                score: r.score,
            };
            add_distinct(&mut self.hypotheses, refined);
        }
    }

    /// Moves every hypothesis by `delta` (robot frame) and re-scores it on `scan`.
    fn follow(&mut self, scan: &PointCloud2D, delta: &Pose2D) {
        for h in std::mem::take(&mut self.hypotheses) {
            let r = self
                .fine
                .match_scans(scan, &self.walls, &h.pose.compose(delta));
            let moved = Hypothesis {
                pose: r.transform,
                score: h.score + SCORE_GAIN * (r.score - h.score),
            };
            add_distinct(&mut self.hypotheses, moved);
        }
    }
}

/// Adds `h` to `list` of distinct hypotheses; if one there is the same pose, the better
/// scoring of the two stays.
fn add_distinct(list: &mut Vec<Hypothesis>, h: Hypothesis) {
    let same = |o: &Hypothesis| {
        (o.pose.x - h.pose.x).hypot(o.pose.y - h.pose.y) < SAME_POSITION
            && angle_diff(o.pose.theta, h.pose.theta).abs() < SAME_HEADING
    };
    match list.iter_mut().find(|o| same(o)) {
        Some(o) if o.score < h.score => *o = h,
        Some(_) => {}
        None => list.push(h),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algorithms::mapping::OccupancyGridConfig;
    use crate::core::types::Point2D;

    type Wall = ((f32, f32), (f32, f32));

    /// An L-shaped flat: a 5 x 4 m room with a 2 x 2 m corner cut out, and a partition.
    const FLAT: [Wall; 7] = [
        ((0.0, 0.0), (5.0, 0.0)),
        ((5.0, 0.0), (5.0, 2.0)),
        ((5.0, 2.0), (3.0, 2.0)),
        ((3.0, 2.0), (3.0, 4.0)),
        ((3.0, 4.0), (0.0, 4.0)),
        ((0.0, 4.0), (0.0, 0.0)),
        ((1.5, 1.5), (1.5, 2.5)),
    ];

    /// A plain 4 x 3 m rectangle: every pose has a twin turned by 180°.
    const BOX: [Wall; 4] = [
        ((0.0, 0.0), (4.0, 0.0)),
        ((4.0, 0.0), (4.0, 3.0)),
        ((4.0, 3.0), (0.0, 3.0)),
        ((0.0, 3.0), (0.0, 0.0)),
    ];

    /// The plan of `walls`: walls occupied, what is in sight of `(2, 1)` free.
    fn plan(walls: &[Wall]) -> OccupancyGrid {
        plan_seen_from(walls, &[(2.0, 1.0)])
    }

    /// The plan of `walls`: walls occupied, what is in sight of any of `seeds` free.
    fn plan_seen_from(walls: &[Wall], seeds: &[(f32, f32)]) -> OccupancyGrid {
        let mut grid = OccupancyGrid::new(OccupancyGridConfig {
            resolution: 0.05,
            initial_width: 14.0,
            initial_height: 14.0,
            ..OccupancyGridConfig::default()
        });
        let mut set = |x: f32, y: f32, occupied: bool| {
            let (cx, cy) = grid.world_to_cell(x, y).unwrap();
            for _ in 0..10 {
                grid.update_cell(cx, cy, occupied);
            }
        };
        let in_sight = |x: f32, y: f32| {
            seeds
                .iter()
                .any(|&(sx, sy)| ray_hit(walls, (sx, sy), (x - sx, y - sy)).is_none_or(|t| t > 1.0))
        };
        for i in -120..120 {
            for j in -120..120 {
                let (x, y) = (i as f32 * 0.05 + 0.025, j as f32 * 0.05 + 0.025);
                let enclosed = |dx: f32, dy: f32| ray_hit(walls, (x, y), (dx, dy)).is_some();
                if in_sight(x, y) && enclosed(1.0, 0.0) && enclosed(-1.0, 0.0) {
                    set(x, y, false);
                }
            }
        }
        for &((ax, ay), (bx, by)) in walls {
            for k in 0..=100 {
                let t = k as f32 / 100.0;
                set(ax + (bx - ax) * t, ay + (by - ay) * t, true);
            }
        }
        grid
    }

    /// Distance along `dir` (in units of |dir|) from `from` to the nearest wall.
    fn ray_hit(walls: &[Wall], from: (f32, f32), dir: (f32, f32)) -> Option<f32> {
        walls
            .iter()
            .filter_map(|&((ax, ay), (bx, by))| {
                let (ex, ey) = (bx - ax, by - ay);
                let den = dir.0 * ey - dir.1 * ex;
                if den.abs() < 1e-9 {
                    return None;
                }
                let (wx, wy) = (ax - from.0, ay - from.1);
                let t = (wx * ey - wy * ex) / den;
                let u = (wx * dir.1 - wy * dir.0) / den;
                (t > 1e-4 && (0.0..=1.0).contains(&u)).then_some(t)
            })
            .min_by(f32::total_cmp)
    }

    /// A 360-ray scan from `pose`, in the robot frame.
    fn scan(walls: &[Wall], pose: Pose2D) -> PointCloud2D {
        let points: Vec<Point2D> = (0..360)
            .filter_map(|k| {
                let a = (k as f32).to_radians();
                let dir = ((pose.theta + a).cos(), (pose.theta + a).sin());
                let r = ray_hit(walls, (pose.x, pose.y), dir)?;
                (r < 8.0).then(|| Point2D::new(r * a.cos(), r * a.sin()))
            })
            .collect();
        PointCloud2D::from_points(&points)
    }

    /// Feeds scans along `path` (plan poses), reporting them in a session frame that is
    /// `to_plan` away from the plan; returns the outcome and how many scans it took.
    fn run(
        plan_walls: &[Wall],
        world: &[Wall],
        path: &[Pose2D],
        to_plan: Pose2D,
    ) -> (Placement, usize) {
        let mut reloc = Relocaliser::new(&plan(plan_walls), RelocaliserConfig::default());
        let to_session = to_plan.inverse();
        for (i, pose) in path.iter().enumerate() {
            let state = reloc.update(&scan(world, *pose), &to_session.compose(pose));
            if state != Placement::Searching {
                return (state, i + 1);
            }
        }
        (Placement::Searching, path.len())
    }

    fn standing(pose: Pose2D, scans: usize) -> Vec<Pose2D> {
        vec![pose; scans]
    }

    /// Asserts that `found` puts the robot, at plan pose `at`, within the plan and score
    /// grids' quantisation (5 cm each) and 1°.
    fn assert_places(found: Pose2D, to_plan: Pose2D, at: Pose2D) {
        let placed = found.compose(&to_plan.inverse().compose(&at));
        let off = (placed.x - at.x).hypot(placed.y - at.y);
        let turn = angle_diff(placed.theta, at.theta).abs();
        assert!(off < 0.08 && turn < 0.02, "{placed:?} vs {at:?}");
    }

    #[test]
    fn finds_the_robot_in_an_asymmetric_flat() {
        let truth = Pose2D::new(3.8, 0.9, 2.2);
        let to_plan = Pose2D::new(3.0, -1.0, 0.7); // where this session's frame sits
        match run(&FLAT, &FLAT, &standing(truth, 10), to_plan) {
            (Placement::Committed(found), scans) => {
                assert_places(found, to_plan, truth);
                assert_eq!(scans, RelocaliserConfig::default().commit_scans);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn follows_the_robot_while_it_moves() {
        let path: Vec<Pose2D> = (0..10)
            .map(|k| Pose2D::new(1.0 + 0.1 * k as f32, 0.8, 0.05 * k as f32))
            .collect();
        let to_plan = Pose2D::new(-2.0, 5.0, -1.2);
        match run(&FLAT, &FLAT, &path, to_plan) {
            (Placement::Committed(found), scans) => {
                assert_places(found, to_plan, path[scans - 1]);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn does_not_commit_while_a_symmetric_room_is_ambiguous() {
        let (state, _) = run(
            &BOX,
            &BOX,
            &standing(Pose2D::new(1.0, 1.0, 0.3), 20),
            Pose2D::identity(),
        );
        assert_eq!(state, Placement::Searching);
    }

    #[test]
    fn gives_up_in_a_home_the_plan_does_not_know() {
        let corridor: [Wall; 2] = [((-5.0, 0.0), (9.0, 0.0)), ((-5.0, 0.8), (9.0, 0.8))];
        let (state, _) = run(
            &FLAT,
            &corridor,
            &standing(Pose2D::new(2.0, 0.4, 0.0), 20),
            Pose2D::identity(),
        );
        assert_eq!(state, Placement::Lost);
    }

    #[test]
    fn does_not_commit_between_two_identical_flats() {
        // The flat and a copy 6 m to its left: the best distinct pose of each must survive
        // the coarse search, however many tiles find the first.
        let mut twins = FLAT.to_vec();
        twins.extend(FLAT.map(|((ax, ay), (bx, by))| ((ax - 6.0, ay), (bx - 6.0, by))));
        let mut reloc = Relocaliser::new(
            &plan_seen_from(&twins, &[(2.0, 1.0), (-4.0, 1.0)]),
            RelocaliserConfig::default(),
        );
        let pose = Pose2D::new(3.8, 0.9, 2.2);
        for _ in 0..20 {
            assert_eq!(
                reloc.update(&scan(&twins, pose), &pose),
                Placement::Searching
            );
        }
        let near = |x: f32| {
            reloc
                .hypotheses()
                .iter()
                .any(|h| (h.pose.x - x).abs() < 0.1)
        };
        assert!(
            near(pose.x) && near(pose.x - 6.0),
            "{:?}",
            reloc.hypotheses()
        );
    }

    #[test]
    fn searches_again_after_being_lost() {
        let corridor: [Wall; 2] = [((-5.0, 0.0), (9.0, 0.0)), ((-5.0, 0.8), (9.0, 0.8))];
        let mut reloc = Relocaliser::new(&plan(&FLAT), RelocaliserConfig::default());
        let lost = Pose2D::new(2.0, 0.4, 0.0);
        assert_eq!(reloc.update(&scan(&corridor, lost), &lost), Placement::Lost);
        // Carried home: the session pose jumps, but the search starts over.
        let home = Pose2D::new(3.8, 0.9, 2.2);
        let states: Vec<_> = (0..5)
            .map(|_| reloc.update(&scan(&FLAT, home), &lost))
            .collect();
        assert!(matches!(states[4], Placement::Committed(_)), "{states:?}");
    }

    #[test]
    fn gives_up_on_a_search_that_never_settles() {
        let config = RelocaliserConfig {
            give_up_scans: 10,
            ..RelocaliserConfig::default()
        };
        let mut reloc = Relocaliser::new(&plan(&BOX), config);
        let pose = Pose2D::new(1.0, 1.0, 0.3);
        let states: Vec<_> = (0..12)
            .map(|_| reloc.update(&scan(&BOX, pose), &pose))
            .collect();
        assert!(
            states[..10].iter().all(|s| *s == Placement::Searching),
            "{states:?}"
        );
        assert_eq!(states[10], Placement::Lost);
        assert_eq!(states[11], Placement::Searching, "and starts over");
    }
}
