//! Rooms: the floor plan's free space split at its doors.
//!
//! The morphological segmentation of the IPA room segmentation (Bormann et al., "Room
//! segmentation: survey, implementation, and analysis", ICRA 2016), at a single scale:
//! shrink the free space by half a door's width, so doorways close and each room is left as a
//! separate core; then grow the cores back over the free space, nearest core first, so the
//! boundaries fall in the doorways. A corridor wider than a door keeps a core of its own and
//! becomes a room, as in the IPA method. Cores too small to be a room (a gap behind a
//! radiator) are dropped and their space joins a neighbour. Free space no core reaches (a
//! closet narrower than a door) becomes a room of its own, so every free cell is in a room.
//!
//! Room numbers follow the grid's raster order and are not stable when the plan changes.

use std::collections::VecDeque;

use image::{GrayImage, Luma};
use imageproc::region_labelling::{Connectivity, connected_components};

use super::OccupancyGrid;

/// Segmentation tuning.
#[derive(Debug, Clone, Copy)]
pub struct RoomsConfig {
    /// Widest opening that counts as a door between rooms (m): an opening of up to
    /// `door_width / resolution` cells closes when the free space shrinks.
    pub door_width: f32,
    /// Smallest core that makes a room (m², after the shrinking).
    pub min_core_area: f32,
}

impl Default for RoomsConfig {
    fn default() -> Self {
        Self {
            door_width: 1.0,
            min_core_area: 0.25,
        }
    }
}

/// A room label per cell of the grid it was made from: 0 outside any room, rooms 1..=count.
#[derive(Debug, Clone, PartialEq)]
pub struct Rooms {
    width: usize,
    height: usize,
    resolution: f32,
    labels: Vec<u16>,
    count: u16,
}

impl Rooms {
    /// Splits the free space of `plan` (walls occupied, e.g. `FloorPlan::to_grid`). Unknown
    /// cells bound the rooms like walls: the plan is what the robot has seen.
    pub fn segment(plan: &OccupancyGrid, config: RoomsConfig) -> Self {
        let (w, h) = plan.dimensions();
        let resolution = plan.resolution();
        let clearance = plan.clearance();
        let free: Vec<bool> = clearance.iter().map(|&c| c > 0.0).collect();
        let cores = GrayImage::from_fn(w as u32, h as u32, |x, y| {
            let i = y as usize * w + x as usize;
            Luma([u8::from(clearance[i] > config.door_width / 2.0)])
        });
        let components = connected_components(&cores, Connectivity::Four, Luma([0]));

        // Number the cores large enough to be rooms, 1.. in order of first appearance.
        let min_cells = (config.min_core_area / resolution.powi(2)).ceil() as usize;
        let core_count = components
            .pixels()
            .map(|p| p[0] as usize)
            .max()
            .unwrap_or(0);
        let mut sizes = vec![0usize; core_count + 1];
        for p in components.pixels() {
            sizes[p[0] as usize] += 1;
        }
        let mut rooms = Self {
            width: w,
            height: h,
            resolution,
            labels: vec![0; w * h],
            count: 0,
        };
        let mut room_of = vec![0u16; core_count + 1];
        let mut seeds = VecDeque::new();
        for (i, p) in components.pixels().enumerate() {
            let core = p[0] as usize;
            if core == 0 || sizes[core] < min_cells {
                continue;
            }
            if room_of[core] == 0 {
                let Some(room) = rooms.count.checked_add(1) else {
                    continue;
                };
                (rooms.count, room_of[core]) = (room, room);
            }
            rooms.labels[i] = room_of[core];
            seeds.push_back(i);
        }
        rooms.grow(seeds, &free);

        // Free space no core reached becomes rooms of its own.
        for i in 0..w * h {
            if free[i] && rooms.labels[i] == 0 {
                let Some(room) = rooms.count.checked_add(1) else {
                    break;
                };
                (rooms.count, rooms.labels[i]) = (room, room);
                rooms.grow(VecDeque::from([i]), &free);
            }
        }
        rooms
    }

    /// Grows the labelled `frontier` breadth first over unlabelled free cells, so each cell
    /// goes to the room fewest steps away.
    fn grow(&mut self, mut frontier: VecDeque<usize>, free: &[bool]) {
        let w = self.width;
        while let Some(i) = frontier.pop_front() {
            let (x, y) = (i % w, i / w);
            let neighbours = [
                (x > 0).then(|| i - 1),
                (x + 1 < w).then(|| i + 1),
                (y > 0).then(|| i - w),
                (y + 1 < self.height).then(|| i + w),
            ];
            for j in neighbours.into_iter().flatten() {
                if self.labels[j] == 0 && free[j] {
                    self.labels[j] = self.labels[i];
                    frontier.push_back(j);
                }
            }
        }
    }

    /// Number of rooms.
    pub fn count(&self) -> u16 {
        self.count
    }

    /// The room of cell (x, y) of the segmented grid, if it is in one.
    pub fn room(&self, x: usize, y: usize) -> Option<u16> {
        if x >= self.width || y >= self.height {
            return None;
        }
        Some(self.labels[y * self.width + x]).filter(|&r| r > 0)
    }

    /// The rooms each room shares a doorway with (free cells side by side), indexed by
    /// room - 1.
    pub fn neighbours(&self) -> Vec<Vec<u16>> {
        let mut neighbours = vec![Vec::new(); self.count as usize];
        let w = self.width;
        for (i, &a) in self.labels.iter().enumerate() {
            let right = (i % w + 1 < w).then(|| i + 1);
            let up = (i + w < self.labels.len()).then(|| i + w);
            for b in [right, up].into_iter().flatten().map(|j| self.labels[j]) {
                if a > 0 && b > 0 && a != b {
                    neighbours[a as usize - 1].push(b);
                    neighbours[b as usize - 1].push(a);
                }
            }
        }
        for list in &mut neighbours {
            list.sort_unstable();
            list.dedup();
        }
        neighbours
    }

    /// Floor area per room (m²), indexed by room - 1.
    pub fn areas(&self) -> Vec<f32> {
        let mut cells = vec![0usize; self.count as usize];
        for &r in self.labels.iter().filter(|&&r| r > 0) {
            cells[r as usize - 1] += 1;
        }
        let cell_area = self.resolution.powi(2);
        cells.into_iter().map(|n| n as f32 * cell_area).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algorithms::mapping::OccupancyGridConfig;

    const RES: f32 = 0.05;

    /// A plan drawn as text, one character per cell, top row first: `#` wall, `.` free,
    /// anything else unknown.
    fn plan(rows: &[String]) -> OccupancyGrid {
        let (w, h) = (rows[0].len(), rows.len());
        let mut grid = OccupancyGrid::new(OccupancyGridConfig {
            resolution: RES,
            initial_width: w as f32 * RES,
            initial_height: h as f32 * RES,
            ..OccupancyGridConfig::default()
        });
        for (row, line) in rows.iter().enumerate() {
            for (x, c) in line.chars().enumerate() {
                let y = h - 1 - row;
                let occupied = match c {
                    '#' => true,
                    '.' => false,
                    _ => continue,
                };
                for _ in 0..10 {
                    grid.update_cell(x, y, occupied);
                }
            }
        }
        grid
    }

    /// Two rooms side by side, `left` and `right` cells wide and `height` tall inside,
    /// joined through the middle of their shared wall by an opening `door` cells wide.
    fn two_rooms(left: usize, right: usize, height: usize, door: usize) -> Vec<String> {
        let wall = "#".repeat(left + right + 3);
        let mut rows = vec![wall.clone()];
        for y in 0..height {
            let opening = (height - door) / 2..(height + door) / 2;
            let middle = if opening.contains(&y) { '.' } else { '#' };
            rows.push(format!(
                "#{}{middle}{}#",
                ".".repeat(left),
                ".".repeat(right)
            ));
        }
        rows.push(wall);
        rows
    }

    fn segment(rows: &[String]) -> Rooms {
        Rooms::segment(&plan(rows), RoomsConfig::default())
    }

    /// Cells per room, indexed by room - 1.
    fn cells(rooms: &Rooms) -> Vec<usize> {
        let areas = rooms.areas();
        areas
            .iter()
            .map(|a| (a / (RES * RES)).round() as usize)
            .collect()
    }

    #[test]
    fn a_doorway_splits_two_rooms() {
        // 4 x 3 m and 3 x 3 m rooms, a 0.8 m door.
        let rows = two_rooms(80, 60, 60, 16);
        let rooms = segment(&rows);
        assert_eq!(rooms.count(), 2);
        let (left, right) = (rooms.room(20, 30).unwrap(), rooms.room(120, 30).unwrap());
        assert_ne!(left, right);
        assert_eq!(rooms.neighbours()[left as usize - 1], vec![right]);
        // Every free cell is in a room, split at the door.
        let cells = cells(&rooms);
        assert_eq!(cells.iter().sum::<usize>(), 80 * 60 + 60 * 60 + 16);
        let (a, b) = (cells[left as usize - 1], cells[right as usize - 1]);
        assert!(
            a.abs_diff(80 * 60) <= 16 && b.abs_diff(60 * 60) <= 16,
            "{cells:?}"
        );
    }

    #[test]
    fn an_opening_as_wide_as_a_door_splits_and_a_cell_wider_joins() {
        assert_eq!(segment(&two_rooms(80, 60, 60, 20)).count(), 2);
        assert_eq!(segment(&two_rooms(80, 60, 60, 21)).count(), 1);
    }

    #[test]
    fn rooms_without_a_door_stay_apart_and_walls_are_in_none() {
        let rooms = segment(&two_rooms(80, 60, 60, 0));
        assert_eq!(rooms.count(), 2);
        assert_eq!(rooms.neighbours(), vec![Vec::<u16>::new(); 2]);
        assert_eq!(rooms.room(0, 0), None);
        assert_eq!(rooms.room(81, 30), None, "the shared wall");
        assert_eq!(rooms.room(143, 30), None, "past the grid");
        assert_eq!(rooms.room(20, 62), None, "past the grid");
    }

    #[test]
    fn a_nook_too_small_for_a_room_joins_its_room() {
        // A 1.1 m square nook off a room through a 0.4 m gap: a core, but a tiny one.
        let mut rows = two_rooms(80, 22, 60, 8);
        for (_, row) in rows
            .iter_mut()
            .enumerate()
            .filter(|(r, _)| !(20..42).contains(r))
        {
            row.replace_range(82..104, &"#".repeat(22));
        }
        let rooms = segment(&rows);
        assert_eq!(rooms.count(), 1);
        assert_eq!(rooms.room(92, 30), rooms.room(20, 30));
    }

    #[test]
    fn a_closet_narrower_than_a_door_is_a_room_of_its_own() {
        // A closed-off 0.6 m wide closet has no core but its floor still needs cleaning.
        let rooms = segment(&two_rooms(80, 12, 60, 0));
        assert_eq!(rooms.count(), 2);
        assert_eq!(cells(&rooms).iter().sum::<usize>(), 80 * 60 + 12 * 60);
        assert_ne!(rooms.room(85, 30), rooms.room(20, 30));
    }

    #[test]
    fn an_unseen_plan_has_no_rooms() {
        let rooms = segment(&vec![" ".repeat(10); 10]);
        assert_eq!(rooms.count(), 0);
        assert_eq!(rooms.room(5, 5), None);
    }
}
