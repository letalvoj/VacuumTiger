//! What the stock follower sees: per one-degree beam of the lidar, the gap from the body to
//! the nearest return, `range − shell` (`STOCK_SWEEP_LOGIC.md` §2).
//!
//! Beam `i` looks `(i − 180)°` from the heading, so beams under 180 are on the right. The
//! points come in the (mirrored) robot frame: the wall side is always the right.
//!
//! The stock takes the lidar's ranges as they come, measured from its optical centre, and
//! its shell table is a constant. So the points are turned back into those raw ranges, and
//! the table is the decoded one, not one built from our idea of where the lidar sits.

use crate::core::types::Point2D;

/// What a query returns with nothing in its sector (m): the stock's 255.
pub const CLEAR: f32 = 255.0;
/// Shorter ranges are the robot itself (m).
const MIN_RANGE: f32 = 0.07;
/// `fused_sonar` counts beams within this gap (m)…
const SONAR_RANGE: f32 = 0.3;
/// …and this many of them, as `bumper` does under its gap.
const MIN_BEAMS: usize = 3;

pub const BEAMS: usize = 360;
/// The shell table at 0x009746d8 is exactly this circle (m) seen from this far ahead of
/// its centre (behind: negative): where the vendor put the lidar, a constant of the table,
/// not where ours is configured (`Shell::new`).
const SHELL_RADIUS: f32 = 0.175;
const SHELL_LIDAR_X: f32 = -0.105;

/// The stock's shell table, the range along each beam to the body's edge, and where our
/// lidar is: how far ahead of the centre (m), and its optical offset (m), which the points
/// carry and the raw ranges do not.
#[derive(Debug, Clone)]
pub struct Shell {
    lidar_x: f32,
    optical_offset: f32,
    range: [f32; BEAMS],
}

impl Shell {
    pub fn new(lidar_x: f32, optical_offset: f32) -> Self {
        let range = std::array::from_fn(|i| {
            let a = beam_angle(i);
            // |(x, 0) + t (cos a, sin a)| = radius, the root ahead.
            let b = SHELL_LIDAR_X * a.cos();
            let c = SHELL_LIDAR_X * SHELL_LIDAR_X - SHELL_RADIUS * SHELL_RADIUS;
            -b + (b * b - c).sqrt()
        });
        Self {
            lidar_x,
            optical_offset,
            range,
        }
    }

    /// The beam of a robot-frame point and its range as the lidar reports it.
    pub fn beam(&self, p: &Point2D) -> (usize, f32) {
        let (x, y) = (p.x - self.lidar_x, p.y);
        let deg = y.atan2(x).to_degrees().round() as i32 + 180;
        let range = x.hypot(y) - self.optical_offset;
        (deg.rem_euclid(BEAMS as i32) as usize, range)
    }
}

/// The bearing of beam `i` from the heading (rad, + left).
pub fn beam_angle(i: usize) -> f32 {
    (i as f32 - 180.0).to_radians()
}

/// The gap per beam, the nearest return's; `None` where none came back.
#[derive(Debug, Clone)]
pub struct Gaps([Option<f32>; BEAMS]);

impl Gaps {
    pub fn new(shell: &Shell, points: &[Point2D]) -> Self {
        let mut gaps = [None; BEAMS];
        for p in points {
            let (i, range) = shell.beam(p);
            if range < MIN_RANGE {
                continue;
            }
            let gap = range - shell.range[i];
            let g: &mut Option<f32> = &mut gaps[i];
            *g = Some(g.map_or(gap, |g| g.min(gap)));
        }
        Self(gaps)
    }

    /// Beams `a..=b` with a return read it `by` (m) nearer, below 0 if need be: something the
    /// lidar does not see stands that far in front of what it does.
    pub fn nearer(&mut self, a: usize, b: usize, by: f32) {
        for g in self.0[a..=b].iter_mut().flatten() {
            *g -= by;
        }
    }

    fn sector(&self, a: usize, b: usize) -> impl Iterator<Item = (usize, f32)> + '_ {
        (a..=b).filter_map(|i| self.0[i % BEAMS].map(|g| (i % BEAMS, g)))
    }

    /// `getFollowWallIR`: the least gap in beams `a..=b` and its beam; `CLEAR` and `None`
    /// with no return there.
    pub fn follow_ir(&self, a: usize, b: usize) -> (f32, Option<usize>) {
        self.sector(a, b).fold((CLEAR, None), |(best, at), (i, g)| {
            if g < best { (g, Some(i)) } else { (best, at) }
        })
    }

    /// `getFusedSonar`: the least gap in beams `a..=b` (not below 0) when `MIN_BEAMS` of
    /// them are within `SONAR_RANGE`; else `CLEAR`.
    pub fn fused_sonar(&self, a: usize, b: usize) -> f32 {
        let near = || {
            self.sector(a, b)
                .map(|(_, g)| g)
                .filter(|&g| g <= SONAR_RANGE)
        };
        if near().count() >= MIN_BEAMS {
            near().fold(CLEAR, f32::min).max(0.0)
        } else {
            CLEAR
        }
    }

    /// `getBumper`: `MIN_BEAMS` or more beams in `a..=b` under `gap`. The stock debounces
    /// it over three layers (lidar, obstacle map, grid map); on one scan they agree.
    pub fn bumper(&self, a: usize, b: usize, gap: f32) -> bool {
        self.sector(a, b).filter(|&(_, g)| g < gap).count() >= MIN_BEAMS
    }
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;

    /// A wall from one end to the other (m).
    pub type Segment = ((f32, f32), (f32, f32));

    /// Robot-frame points where each beam of a lidar at `lidar_x` meets the segments
    /// (robot frame), nearest first.
    pub fn ray_scan(lidar_x: f32, segments: &[Segment]) -> Vec<Point2D> {
        (0..BEAMS)
            .filter_map(|i| {
                let (uy, ux) = beam_angle(i).sin_cos();
                let t = segments
                    .iter()
                    .filter_map(|&((x1, y1), (x2, y2))| {
                        let (ex, ey) = (x2 - x1, y2 - y1);
                        let den = ux * ey - uy * ex;
                        if den.abs() < 1e-9 {
                            return None;
                        }
                        let t = ((x1 - lidar_x) * ey - y1 * ex) / den;
                        let s = ((x1 - lidar_x) * uy - y1 * ux) / den;
                        (t > 0.0 && (0.0..=1.0).contains(&s)).then_some(t)
                    })
                    .fold(f32::INFINITY, f32::min);
                t.is_finite()
                    .then(|| Point2D::new(lidar_x + t * ux, t * uy))
            })
            .collect()
    }

    /// A wall along the right side at `clearance` (m) from a body of `radius`.
    pub fn right_wall(lidar_x: f32, radius: f32, clearance: f32) -> Vec<Point2D> {
        let y = -(radius + clearance);
        ray_scan(lidar_x, &[((-3.0, y), (3.0, y))])
    }

    #[test]
    fn the_shell_is_the_body_seen_from_the_lidar() {
        let shell = Shell::new(-0.105, 0.0);
        assert!((shell.range[180] - 0.28).abs() < 1e-5, "ahead");
        assert!((shell.range[0] - 0.07).abs() < 1e-5, "behind");
        assert!(
            (shell.range[90] - 0.14).abs() < 1e-5,
            "right: √(0.175² − 0.105²)"
        );
    }

    #[test]
    fn gaps_are_raw_ranges_less_the_shell() {
        // The points carry the optical offset; the stock's ranges do not.
        let wall = ray_scan(-0.11, &[((0.195, -1.0), (0.195, 1.0))]);
        let gaps = Gaps::new(&Shell::new(-0.11, 0.025), &wall);
        let want = (0.195 + 0.11 - 0.025) - 0.28;
        assert!((gaps.follow_ir(180, 180).0 - want).abs() < 1e-4);
    }

    #[test]
    fn a_parallel_wall_reads_as_the_spec_table() {
        // STOCK_SWEEP_LOGIC.md §2: per clearance, S1 148–150, S3 135–143 and the arc test
        // 120–143.
        let shell = Shell::new(-0.105, 0.0);
        for (clearance, s1, s3, arc) in [
            (-0.01, 0.056, 0.001, -0.012),
            (0.0, 0.075, 0.015, 0.0),
            (0.025, 0.122, 0.050, 0.029),
            (0.03, 0.132, 0.057, 0.035),
        ] {
            let gaps = Gaps::new(&shell, &right_wall(-0.105, 0.175, clearance));
            for (got, want) in [
                (gaps.follow_ir(148, 150).0, s1),
                (gaps.follow_ir(135, 143).0, s3),
                (gaps.follow_ir(120, 143).0, arc),
            ] {
                assert!((got - want).abs() < 0.002, "{clearance}: {got} vs {want}");
            }
        }
    }

    #[test]
    fn queries_need_three_beams_and_say_clear_without_returns() {
        let shell = Shell::new(-0.105, 0.0);
        let gaps = Gaps::new(&shell, &[]);
        assert_eq!(gaps.follow_ir(0, 359), (CLEAR, None));
        assert_eq!(gaps.fused_sonar(160, 210), CLEAR);
        // One post 2 cm ahead of the body: one beam, no bumper, no sonar.
        let gaps = Gaps::new(&shell, &[Point2D::new(0.195, 0.0)]);
        assert!((gaps.follow_ir(160, 200).0 - 0.02).abs() < 1e-4);
        assert_eq!(gaps.follow_ir(160, 200).1, Some(180));
        assert!(!gaps.bumper(160, 195, 0.04));
        assert_eq!(gaps.fused_sonar(160, 210), CLEAR);
        // A wall 2 cm ahead: both.
        let wall = ray_scan(-0.105, &[((0.195, -1.0), (0.195, 1.0))]);
        let gaps = Gaps::new(&shell, &wall);
        assert!(gaps.bumper(160, 195, 0.04));
        assert!((gaps.fused_sonar(160, 210) - 0.02).abs() < 1e-3);
    }
}
