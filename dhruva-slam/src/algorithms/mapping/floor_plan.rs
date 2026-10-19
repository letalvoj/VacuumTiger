//! The floor plan: what is structurally there, merged from many sessions.
//!
//! A session map (the SLAM occupancy grid of one run) also holds chairs, bags and people,
//! and through a mirror a whole room that is not there. The floor plan keeps, per cell, which
//! of the last `window` sessions that observed the cell saw it occupied, and calls it a wall
//! when at least `wall_votes` of them did (fewer while fewer sessions saw it) and they are
//! not outvoted by the sessions that saw it free. So furniture that moves stops being a wall,
//! a new wall becomes one after a few runs, a reflection seen free from other angles is
//! dropped, and a cell no session observed keeps its state (an occluded wall is not erased).
//! Each cell is two bytes, the plan grows only to what sessions observed and a merge that
//! would make it larger than a home is refused, so merging any number of runs cannot bloat it.
//!
//! Sessions must be aligned to the plan first (relocalisation); `merge` takes that
//! alignment as the session-to-plan transform.

use std::io::{Read, Write};
use std::path::Path;

use super::{CellState, OccupancyGrid, OccupancyGridConfig};
use crate::core::types::Pose2D;

/// How far from the world origin (where the first session started) the plan may reach
/// (m); a home is far smaller.
const MAX_REACH_M: f32 = 50.0;
/// Most cells in a plan (2 bytes each).
const MAX_CELLS: u64 = 4_000_000;
/// Most lookups along each axis of a plan cell when merging.
const MAX_SAMPLES: usize = 8;
/// File magic, ending in the format version.
const MAGIC: &[u8; 4] = b"DFP1";
/// Bytes before the cells: magic, resolution, window, wall_votes, offset, width, height.
const HEADER: usize = 4 + 4 + 1 + 1 + 4 + 4 + 4 + 4;

/// Floor-plan configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FloorPlanConfig {
    /// Cell size (m).
    pub resolution: f32,
    /// Number of most recent observing sessions remembered per cell (1..=8).
    pub window: u8,
    /// Sessions out of the window that must see a cell occupied for it to be a wall
    /// (1..=window).
    pub wall_votes: u8,
}

impl Default for FloorPlanConfig {
    fn default() -> Self {
        Self {
            resolution: 0.05,
            window: 5,
            wall_votes: 2,
        }
    }
}

impl FloorPlanConfig {
    fn validate(&self) -> Result<(), String> {
        let ok = self.resolution.is_finite()
            && self.resolution >= 0.005
            && (1..=8).contains(&self.window)
            && (1..=self.window).contains(&self.wall_votes);
        ok.then_some(())
            .ok_or_else(|| format!("invalid floor plan config {self:?}"))
    }

    /// Whether world cells `lo..=hi` are within `MAX_REACH_M` and `MAX_CELLS`.
    fn fits(&self, lo: (i64, i64), hi: (i64, i64)) -> bool {
        let reach = (MAX_REACH_M / self.resolution) as i64;
        let within = |v: i64| (-reach..=reach).contains(&v);
        [lo.0, lo.1, hi.0, hi.1].into_iter().all(within)
            && lo.0 <= hi.0
            && lo.1 <= hi.1
            && (hi.0 - lo.0 + 1) as u64 * (hi.1 - lo.1 + 1) as u64 <= MAX_CELLS
    }
}

/// What the plan says about a cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanCell {
    Unknown,
    Free,
    Wall,
}

/// One cell's evidence: bit i of `history` is whether the i-th most recent session that
/// observed the cell saw it occupied (bits beyond the window are cleared); `seen` counts
/// those sessions, up to the window.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct Evidence {
    history: u8,
    seen: u8,
}

/// The persistent floor plan. Its cells lie on the lattice through the world origin: cell
/// (i, j) of the plan is world cell (offset.0 + i, offset.1 + j), which spans
/// [k * resolution, (k + 1) * resolution) along each axis.
#[derive(Debug, Clone, PartialEq)]
pub struct FloorPlan {
    config: FloorPlanConfig,
    offset: (i32, i32),
    width: usize,
    height: usize,
    cells: Vec<Evidence>,
}

impl FloorPlan {
    pub fn new(config: FloorPlanConfig) -> Result<Self, String> {
        config.validate()?;
        Ok(Self {
            config,
            offset: (0, 0),
            width: 0,
            height: 0,
            cells: Vec::new(),
        })
    }

    /// Cells along x and y.
    pub fn dimensions(&self) -> (usize, usize) {
        (self.width, self.height)
    }

    /// Adds one session. `to_plan` maps session coordinates to plan coordinates. Refuses,
    /// leaving the plan as it was, a session that would take the plan beyond
    /// `MAX_REACH_M` or `MAX_CELLS` (a bad alignment).
    pub fn merge(&mut self, session: &OccupancyGrid, to_plan: &Pose2D) -> Result<(), String> {
        let Some((lo, hi)) = self.observed_bounds(session, to_plan)? else {
            return Ok(());
        };
        self.grow_to(lo, hi)?;

        // Each plan cell is looked up at a few points spaced finer than the session's
        // cells, so no session cell falls between them whatever the alignment; occupied
        // wins over free.
        let res = self.config.resolution;
        let n = ((res / session.resolution()).ceil() as usize + 1).min(MAX_SAMPLES);
        let to_session = to_plan.inverse();
        let window_mask = ((1u16 << self.config.window) - 1) as u8;
        for wy in lo.1..=hi.1 {
            for wx in lo.0..=hi.0 {
                let mut observed = None;
                for (i, j) in (0..n).flat_map(|i| (0..n).map(move |j| (i, j))) {
                    let x = (wx as f32 + (i as f32 + 0.5) / n as f32) * res;
                    let y = (wy as f32 + (j as f32 + 0.5) / n as f32) * res;
                    let p = to_session.compose(&Pose2D::new(x, y, 0.0));
                    let state = session
                        .world_to_cell(p.x, p.y)
                        .map_or(CellState::Unknown, |(cx, cy)| session.get_state(cx, cy));
                    match state {
                        CellState::Occupied => {
                            observed = Some(true);
                            break;
                        }
                        CellState::Free => observed = Some(false),
                        CellState::Unknown => {}
                    }
                }
                if let Some(occupied) = observed {
                    let i = self.index(wx, wy).expect("grown to hold it");
                    let cell = &mut self.cells[i];
                    cell.history = ((cell.history << 1) | u8::from(occupied)) & window_mask;
                    cell.seen = (cell.seen + 1).min(self.config.window);
                }
            }
        }
        Ok(())
    }

    /// The plan's verdict at a world position.
    pub fn cell_at(&self, x: f32, y: f32) -> PlanCell {
        let res = self.config.resolution;
        self.index((x / res).floor() as i32, (y / res).floor() as i32)
            .map_or(PlanCell::Unknown, |i| self.verdict(self.cells[i]))
    }

    /// The plan as an occupancy grid (walls occupied, free free, the rest unknown), for
    /// the code that reads maps: rendering, saving, scan matching.
    pub fn to_grid(&self) -> OccupancyGrid {
        let config = OccupancyGridConfig {
            resolution: self.config.resolution,
            ..OccupancyGridConfig::default()
        };
        let cells = self
            .cells
            .iter()
            .map(|&e| match self.verdict(e) {
                PlanCell::Wall => config.log_odds_max,
                PlanCell::Free => config.log_odds_min,
                PlanCell::Unknown => 0.0,
            })
            .collect();
        let res = self.config.resolution;
        let (ox, oy) = (self.offset.0 as f32 * res, self.offset.1 as f32 * res);
        OccupancyGrid::from_raw(config, cells, self.width, self.height, ox, oy)
    }

    fn verdict(&self, e: Evidence) -> PlanCell {
        if e.seen == 0 {
            return PlanCell::Unknown;
        }
        let occupied = e.history.count_ones() as u8;
        if occupied >= self.config.wall_votes.min(e.seen) && 2 * occupied >= e.seen {
            PlanCell::Wall
        } else {
            PlanCell::Free
        }
    }

    /// Index of world lattice cell (wx, wy), if the plan holds it.
    fn index(&self, wx: i32, wy: i32) -> Option<usize> {
        let (x, y) = (
            wx as i64 - self.offset.0 as i64,
            wy as i64 - self.offset.1 as i64,
        );
        let inside = x >= 0 && y >= 0 && (x as usize) < self.width && (y as usize) < self.height;
        inside.then(|| y as usize * self.width + x as usize)
    }

    /// The world lattice cells covering what `session` observed, once in the plan frame.
    #[allow(clippy::type_complexity)]
    fn observed_bounds(
        &self,
        session: &OccupancyGrid,
        to_plan: &Pose2D,
    ) -> Result<Option<((i32, i32), (i32, i32))>, String> {
        let (w, h) = session.dimensions();
        let (mut x0, mut y0, mut x1, mut y1) = (w, h, 0, 0);
        for cy in 0..h {
            for cx in 0..w {
                if session.get_state(cx, cy) != CellState::Unknown {
                    (x0, y0, x1, y1) = (x0.min(cx), y0.min(cy), x1.max(cx), y1.max(cy));
                }
            }
        }
        if x0 > x1 {
            return Ok(None);
        }
        let (ox, oy) = session.origin();
        let r = session.resolution();
        let (ax, ay) = (ox + x0 as f32 * r, oy + y0 as f32 * r);
        let (bx, by) = (ox + (x1 + 1) as f32 * r, oy + (y1 + 1) as f32 * r);
        let (mut lo, mut hi) = ((f32::MAX, f32::MAX), (f32::MIN, f32::MIN));
        for (x, y) in [(ax, ay), (bx, ay), (ax, by), (bx, by)] {
            let p = to_plan.compose(&Pose2D::new(x, y, 0.0));
            (lo, hi) = (
                (lo.0.min(p.x), lo.1.min(p.y)),
                (hi.0.max(p.x), hi.1.max(p.y)),
            );
        }
        let reach = -MAX_REACH_M..=MAX_REACH_M;
        if ![lo.0, lo.1, hi.0, hi.1].iter().all(|v| reach.contains(v)) {
            return Err(format!(
                "session spans ({:.1}, {:.1})..({:.1}, {:.1}) m in the plan, beyond its \
                 {MAX_REACH_M} m reach; misaligned?",
                lo.0, lo.1, hi.0, hi.1
            ));
        }
        let res = self.config.resolution;
        let cell = |v: f32| (v / res).floor() as i32;
        Ok(Some(((cell(lo.0), cell(lo.1)), (cell(hi.0), cell(hi.1)))))
    }

    /// Grows the plan to hold world cells `lo..=hi`, keeping what it has.
    fn grow_to(&mut self, lo: (i32, i32), hi: (i32, i32)) -> Result<(), String> {
        let (lo, hi) = if self.cells.is_empty() {
            (lo, hi)
        } else {
            let end = (
                self.offset.0 + self.width as i32 - 1,
                self.offset.1 + self.height as i32 - 1,
            );
            (
                (lo.0.min(self.offset.0), lo.1.min(self.offset.1)),
                (hi.0.max(end.0), hi.1.max(end.1)),
            )
        };
        let wide = |(x, y): (i32, i32)| (x as i64, y as i64);
        if !self.config.fits(wide(lo), wide(hi)) {
            return Err(format!(
                "session would grow the floor plan beyond {MAX_CELLS} cells; misaligned?"
            ));
        }
        let (width, height) = ((hi.0 - lo.0 + 1) as usize, (hi.1 - lo.1 + 1) as usize);
        if (lo, width, height) == (self.offset, self.width, self.height) {
            return Ok(());
        }
        let mut cells = vec![Evidence::default(); width * height];
        let (dx, dy) = (
            (self.offset.0 - lo.0) as usize,
            (self.offset.1 - lo.1) as usize,
        );
        for (y, row) in self.cells.chunks_exact(self.width.max(1)).enumerate() {
            let start = (y + dy) * width + dx;
            cells[start..start + self.width].copy_from_slice(row);
        }
        (self.offset, self.width, self.height, self.cells) = (lo, width, height, cells);
        Ok(())
    }

    /// Writes the plan, replacing `path` only once the new file is complete on disk.
    pub fn save(&self, path: &Path) -> std::io::Result<()> {
        let mut out = MAGIC.to_vec();
        out.extend(self.config.resolution.to_le_bytes());
        out.extend([self.config.window, self.config.wall_votes]);
        out.extend(self.offset.0.to_le_bytes());
        out.extend(self.offset.1.to_le_bytes());
        out.extend((self.width as u32).to_le_bytes());
        out.extend((self.height as u32).to_le_bytes());
        out.extend(self.cells.iter().flat_map(|e| [e.history, e.seen]));
        let mut tmp = path.as_os_str().to_owned();
        tmp.push(".tmp");
        let mut file = std::fs::File::create(&tmp)?;
        file.write_all(&out)?;
        file.sync_all()?;
        std::fs::rename(tmp, path)
    }

    pub fn load(path: &Path) -> std::io::Result<Self> {
        let invalid = |what: String| std::io::Error::new(std::io::ErrorKind::InvalidData, what);
        let mut bytes = Vec::new();
        std::fs::File::open(path)?.read_to_end(&mut bytes)?;
        if bytes.len() < HEADER || &bytes[..4] != MAGIC {
            return Err(invalid("not a floor plan".into()));
        }
        let word = |i: usize| <[u8; 4]>::try_from(&bytes[i..i + 4]).expect("4 bytes");
        let config = FloorPlanConfig {
            resolution: f32::from_le_bytes(word(4)),
            window: bytes[8],
            wall_votes: bytes[9],
        };
        config.validate().map_err(invalid)?;
        let offset = (i32::from_le_bytes(word(10)), i32::from_le_bytes(word(14)));
        let (width, height) = (u32::from_le_bytes(word(18)), u32::from_le_bytes(word(22)));
        let lo = (offset.0 as i64, offset.1 as i64);
        let hi = (lo.0 + width as i64 - 1, lo.1 + height as i64 - 1);
        let size = 2 * width as u64 * height as u64;
        let empty = size == 0 && offset == (0, 0);
        if !(empty || config.fits(lo, hi)) || bytes.len() as u64 != HEADER as u64 + size {
            return Err(invalid("truncated or corrupt floor plan".into()));
        }
        let cells = bytes[HEADER..]
            .as_chunks::<2>()
            .0
            .iter()
            .map(|&[history, seen]| {
                let seen = seen.min(config.window);
                let history = (u16::from(history) & ((1u16 << seen) - 1)) as u8;
                Evidence { history, seen }
            })
            .collect();
        Ok(Self {
            config,
            offset,
            width: width as usize,
            height: height as usize,
            cells,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(size: f32, resolution: f32) -> OccupancyGrid {
        OccupancyGrid::new(OccupancyGridConfig {
            resolution,
            initial_width: size,
            initial_height: size,
            ..OccupancyGridConfig::default()
        })
    }

    /// Observes a cell often enough to cross the grid's thresholds.
    fn observe(grid: &mut OccupancyGrid, x: f32, y: f32, occupied: bool) {
        let (cx, cy) = grid.world_to_cell(x, y).unwrap();
        for _ in 0..10 {
            grid.update_cell(cx, cy, occupied);
        }
    }

    /// A 4 x 3 m room seen from inside at the SLAM's 2 cm, a one-cell wall around it and
    /// the cells holding `extra` occupied.
    fn session(extra: &[(f32, f32)]) -> OccupancyGrid {
        let mut grid = grid(10.0, 0.02);
        let extra: Vec<_> = extra
            .iter()
            .map(|&(x, y)| grid.world_to_cell(x, y))
            .collect();
        for i in 0..=200 {
            for j in 0..=150 {
                let (x, y) = (i as f32 * 0.02 + 0.01, j as f32 * 0.02 + 0.01);
                let wall = i == 0 || i == 200 || j == 0 || j == 150;
                let occupied = wall || extra.contains(&grid.world_to_cell(x, y));
                observe(&mut grid, x, y, occupied);
            }
        }
        grid
    }

    fn merged(sessions: &[OccupancyGrid]) -> FloorPlan {
        let mut plan = FloorPlan::new(FloorPlanConfig::default()).unwrap();
        for s in sessions {
            plan.merge(s, &Pose2D::identity()).unwrap();
        }
        plan
    }

    const CHAIR: (f32, f32) = (2.01, 1.51);
    const LEFT_WALL: (f32, f32) = (0.01, 1.51);

    #[test]
    fn walls_seen_every_time_stay_and_a_moved_chair_does_not() {
        let plan = merged(&[session(&[CHAIR]), session(&[]), session(&[])]);
        assert_eq!(plan.cell_at(LEFT_WALL.0, LEFT_WALL.1), PlanCell::Wall);
        assert_eq!(plan.cell_at(CHAIR.0, CHAIR.1), PlanCell::Free);
        assert_eq!(plan.cell_at(1.01, 1.01), PlanCell::Free);
        assert_eq!(plan.cell_at(-3.0, 1.0), PlanCell::Unknown);
    }

    #[test]
    fn a_first_session_is_taken_as_it_is() {
        let plan = merged(&[session(&[CHAIR])]);
        assert_eq!(plan.cell_at(LEFT_WALL.0, LEFT_WALL.1), PlanCell::Wall);
        assert_eq!(plan.cell_at(CHAIR.0, CHAIR.1), PlanCell::Wall);
    }

    #[test]
    fn a_new_wall_appears_after_enough_sessions() {
        let plan = merged(&[session(&[]), session(&[CHAIR])]);
        assert_eq!(plan.cell_at(CHAIR.0, CHAIR.1), PlanCell::Free, "one of two");
        let plan = merged(&[session(&[]), session(&[CHAIR]), session(&[CHAIR])]);
        assert_eq!(
            plan.cell_at(CHAIR.0, CHAIR.1),
            PlanCell::Wall,
            "two of three"
        );
    }

    #[test]
    fn a_reflection_seen_free_more_often_is_dropped() {
        // A ghost wall seen through a mirror in two runs, free from other angles in three.
        let (ghost, clear) = (session(&[CHAIR]), session(&[]));
        let plan = merged(&[ghost.clone(), clear.clone(), ghost, clear.clone(), clear]);
        assert_eq!(plan.cell_at(CHAIR.0, CHAIR.1), PlanCell::Free);
    }

    #[test]
    fn a_removed_wall_ages_out_of_the_window() {
        let (wall, clear) = (session(&[CHAIR]), session(&[]));
        let mut sessions = vec![wall.clone(), wall.clone(), wall];
        sessions.extend(std::iter::repeat_n(clear, 3));
        assert_eq!(merged(&sessions).cell_at(CHAIR.0, CHAIR.1), PlanCell::Free);
    }

    #[test]
    fn unobserved_cells_keep_their_state_and_new_areas_extend_the_plan() {
        let mut plan = merged(&[session(&[]), session(&[])]);
        // A session that sees only a corridor beyond the room's right wall.
        let mut corridor = grid(20.0, 0.02);
        for i in 0..50 {
            observe(&mut corridor, 4.51 + i as f32 * 0.02, 1.51, false);
        }
        plan.merge(&corridor, &Pose2D::identity()).unwrap();
        assert_eq!(plan.cell_at(LEFT_WALL.0, LEFT_WALL.1), PlanCell::Wall);
        assert_eq!(plan.cell_at(5.0, 1.51), PlanCell::Free);
    }

    #[test]
    fn merging_the_same_room_again_does_not_grow_the_plan() {
        let mut plan = merged(&[session(&[])]);
        let size = plan.dimensions();
        for _ in 0..10 {
            plan.merge(&session(&[]), &Pose2D::identity()).unwrap();
        }
        assert_eq!(plan.dimensions(), size);
    }

    #[test]
    fn a_session_far_off_the_plan_is_refused_and_changes_nothing() {
        let mut plan = merged(&[session(&[])]);
        let before = plan.clone();
        for to_plan in [
            Pose2D::new(-150.0, 0.0, 0.0),
            Pose2D::new(0.0, 1e10, 0.0),
            Pose2D::new(f32::NAN, 0.0, 0.0),
        ] {
            assert!(plan.merge(&session(&[]), &to_plan).is_err(), "{to_plan:?}");
        }
        assert_eq!(plan, before);
        assert_eq!(plan.cell_at(1e10, -1e10), PlanCell::Unknown);
    }

    #[test]
    fn walls_stay_whole_through_any_alignment() {
        // The room seen from a frame shifted by a fraction of a cell and turned 0.3 rad.
        let to_plan = Pose2D::new(1.013, 2.027, 0.3);
        let mut plan = FloorPlan::new(FloorPlanConfig::default()).unwrap();
        for _ in 0..2 {
            plan.merge(&session(&[CHAIR]), &to_plan).unwrap();
        }
        let at = |x: f32, y: f32| {
            let p = to_plan.compose(&Pose2D::new(x, y, 0.0));
            plan.cell_at(p.x, p.y)
        };
        for k in 1..100 {
            let t = k as f32 / 100.0;
            assert_eq!(at(0.01, 3.0 * t), PlanCell::Wall, "left wall at {t}");
            assert_eq!(at(4.0 * t, 3.01), PlanCell::Wall, "top wall at {t}");
        }
        assert_eq!(at(CHAIR.0, CHAIR.1), PlanCell::Wall);
        assert_eq!(at(1.0, 1.0), PlanCell::Free);
    }

    #[test]
    fn saves_loads_and_converts_to_a_grid() {
        let plan = merged(&[session(&[CHAIR]), session(&[])]);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("plan.dfp");
        plan.save(&path).unwrap();
        assert_eq!(FloorPlan::load(&path).unwrap(), plan);
        assert_eq!(
            std::fs::read_dir(dir.path()).unwrap().count(),
            1,
            "no temp file left"
        );
        let empty = dir.path().join("empty.dfp");
        FloorPlan::new(FloorPlanConfig::default())
            .unwrap()
            .save(&empty)
            .unwrap();
        assert_eq!(FloorPlan::load(&empty).unwrap().dimensions(), (0, 0));

        let grid = plan.to_grid();
        let state = |(x, y)| {
            let (cx, cy) = grid.world_to_cell(x, y).unwrap();
            grid.get_state(cx, cy)
        };
        assert_eq!(state(LEFT_WALL), CellState::Occupied);
        assert_eq!(state(CHAIR), CellState::Free);

        let bytes = std::fs::read(&path).unwrap();
        let corrupt = |what: &str, edit: &dyn Fn(&mut Vec<u8>)| {
            let mut b = bytes.clone();
            edit(&mut b);
            std::fs::write(&path, &b).unwrap();
            assert!(FloorPlan::load(&path).is_err(), "{what}");
        };
        corrupt("zero wall votes", &|b| b[9] = 0);
        corrupt("more votes than window", &|b| b[9] = 6);
        corrupt("offset out of reach", &|b| {
            b[10..14].copy_from_slice(&i32::MAX.to_le_bytes())
        });
        corrupt("truncated", &|b| {
            b.pop();
        });
    }
}
