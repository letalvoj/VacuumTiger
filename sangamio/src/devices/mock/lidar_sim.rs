//! Lidar simulator with ray-casting
//!
//! Simulates the Delta-2D: a beam every `360 / num_rays` degrees, clockwise, one
//! revolution per scan, each beam cast from where the robot is when it is taken. The
//! points are published as the CRL-200S driver publishes them (see [`LidarConfig`]).

use super::config::LidarConfig;
use super::map_loader::SimulationMap;
use super::noise::NoiseGenerator;
use std::f32::consts::{PI, TAU};

/// A lidar scan: (angle_rad, distance_m, quality) per point, as the drivers publish it.
pub type Scan = Vec<(f32, f32, u8)>;

/// A robot pose in the map: (x, y, theta).
pub type Pose = (f32, f32, f32);

/// A person's leg's radius (m).
const LEG_RADIUS: f32 = 0.05;
/// How far from the lidar a leg may stand (m).
const LEG_RANGE: std::ops::Range<f32> = 0.3..2.5;

/// The distance along the ray from `origin` at `heading` (map frame) to the nearest leg
/// (centres in the map frame) in its way.
fn leg_in_the_way(legs: &[(f32, f32)], origin: (f32, f32), heading: f32) -> Option<f32> {
    let (s, c) = heading.sin_cos();
    (legs.iter())
        .filter_map(|&(x, y)| {
            let (dx, dy) = (x - origin.0, y - origin.1);
            let (along, off) = (dx * c + dy * s, dy * c - dx * s);
            (along > 0.0 && off.abs() < LEG_RADIUS)
                .then(|| along - (LEG_RADIUS * LEG_RADIUS - off * off).sqrt())
        })
        .min_by(f32::total_cmp)
}

/// Lidar simulator
pub struct LidarSimulator {
    config: LidarConfig,
    noise: NoiseGenerator,
    /// Share of the current revolution done (0..1), and the beams cast in it.
    turned: f32,
    beams: usize,
    points: Scan,
    /// People's legs in this revolution: their centres in the map frame.
    legs: Vec<(f32, f32)>,
}

impl LidarSimulator {
    /// Create new lidar simulator
    pub fn new(config: &LidarConfig, noise: NoiseGenerator) -> Self {
        Self {
            config: config.clone(),
            noise,
            turned: 0.0,
            beams: 0,
            points: Vec::with_capacity(config.num_rays),
            legs: Vec::new(),
        }
    }

    /// Places this revolution's legs round the lidar at `origin` (map frame): a Poisson
    /// number of them, `noise.legs` on average. Draws nothing when there are none, so the
    /// beams' noise is as it was without them.
    fn place_legs(&mut self, origin: (f32, f32)) {
        self.legs.clear();
        let mean = self.config.noise.legs;
        if mean <= 0.0 {
            return;
        }
        // Knuth: count uniforms until their product drops below e^-mean.
        let (floor, mut product) = ((-mean).exp(), self.noise.uniform());
        let span = LEG_RANGE.end - LEG_RANGE.start;
        while product > floor {
            let bearing = self.noise.uniform() * TAU;
            let range = LEG_RANGE.start + self.noise.uniform() * span;
            let (s, c) = bearing.sin_cos();
            self.legs.push((origin.0 + range * c, origin.1 + range * s));
            product *= self.noise.uniform();
        }
    }

    /// Stops the lidar: the revolution under way is dropped.
    pub fn stop(&mut self) {
        self.turned = 0.0;
        self.beams = 0;
        self.points.clear();
    }

    /// Turns the lidar for `dt` seconds while the robot moves from `from` to `to`, casting
    /// the beams that fall in that time from where the robot is when each is taken.
    /// Returns the scan when a revolution completes. `dt` is at most a revolution.
    pub fn sweep(&mut self, map: &SimulationMap, from: Pose, to: Pose, dt: f32) -> Option<Scan> {
        let start = self.turned;
        let turn = dt * self.config.scan_rate_hz;
        assert!(turn <= 1.0, "a sweep of {turn} revolutions");
        self.turned += turn;
        let n = self.config.num_rays;
        let due = (self.turned.min(1.0) * n as f32) as usize;
        let rotation = (to.2 - from.2 + PI).rem_euclid(TAU) - PI;
        let (mx, my) = (self.config.mounting_x, self.config.mounting_y);
        let mount = |(x, y, theta): Pose| {
            let (s, c) = theta.sin_cos();
            (x + c * mx - s * my, y + s * mx + c * my)
        };
        if self.beams == 0 && due > 0 {
            self.place_legs(mount(from));
        }
        while self.beams < due {
            let phase = self.beams as f32 / n as f32;
            let f = ((phase - start) / (self.turned - start)).clamp(0.0, 1.0);
            let (x, y, theta) = (
                from.0 + f * (to.0 - from.0),
                from.1 + f * (to.1 - from.1),
                from.2 + f * rotation,
            );
            let origin = mount((x, y, theta));
            let angle = self.config.sweep_start_angle - phase * TAU;
            self.beams += 1;
            if let Some(point) = self.beam(map, origin, theta, angle) {
                self.points.push(point);
            }
        }
        if self.turned < 1.0 {
            return None;
        }
        self.turned -= 1.0;
        self.beams = 0;
        Some(std::mem::take(&mut self.points))
    }

    /// One beam from the lidar at `origin` towards `angle` (robot frame), if it returns.
    fn beam(
        &mut self,
        map: &SimulationMap,
        origin: (f32, f32),
        theta: f32,
        angle: f32,
    ) -> Option<(f32, f32, u8)> {
        let leg = leg_in_the_way(&self.legs, origin, theta + angle);
        let cfg = &self.config;
        let noise = &cfg.noise;
        let rng = &mut self.noise;
        let hit = map.ray_cast(origin.0, origin.1, theta + angle, cfg.max_range, |x, y| {
            let opacity = map.lidar_opacity(x, y);
            opacity >= 1.0 || (opacity > 0.0 && rng.chance(opacity))
        });
        let hit = leg.map_or(hit, |leg| hit.min(leg));
        if hit >= cfg.max_range || self.noise.chance(noise.miss_rate) {
            return None;
        }
        let stddev = noise.range_stddev + noise.range_stddev_per_m * hit;
        let distance =
            hit - cfg.optical_offset + self.noise.biased_gaussian(noise.range_bias, stddev);
        if !(cfg.min_range..cfg.max_range).contains(&distance) {
            return None;
        }
        // In [0, 2π): `rem_euclid` rounds a tiny negative angle up to 2π itself.
        let angle = (angle + self.noise.gaussian(noise.angle_stddev)).rem_euclid(TAU);
        let angle = if angle < TAU { angle } else { 0.0 };
        let decay = noise.quality_distance_decay as f32 * distance;
        let quality = (noise.quality_base as f32 - decay).clamp(1.0, 255.0) as u8;
        Some((angle, distance, quality))
    }
}

#[cfg(test)]
mod tests {
    use super::super::config::LidarNoiseConfig;
    use super::*;

    /// A 4 x 4 m room with 10 cm walls, at 1 cm per cell.
    fn room() -> SimulationMap {
        let img = image::GrayImage::from_fn(400, 400, |i, j| {
            let wall = i < 10 || j < 10 || i >= 390 || j >= 390;
            image::Luma([if wall { 0 } else { 255 }])
        });
        SimulationMap::from_image(img, 0.01, (0.0, 0.0))
    }

    fn lidar(config: LidarConfig) -> LidarSimulator {
        let config = LidarConfig {
            noise: LidarNoiseConfig {
                range_stddev: 0.0,
                angle_stddev: 0.0,
                miss_rate: 0.0,
                ..LidarNoiseConfig::default()
            },
            ..config
        };
        LidarSimulator::new(&config, NoiseGenerator::new(1))
    }

    /// Sweeps standing at `at` until a scan completes, `dt` at a time.
    fn scan_at(lidar: &mut LidarSimulator, map: &SimulationMap, at: Pose, dt: f32) -> Scan {
        (0..10_000)
            .find_map(|_| lidar.sweep(map, at, at, dt))
            .expect("a scan")
    }

    #[test]
    fn a_scan_is_one_revolution_clockwise_from_the_start_angle() {
        let mut lidar = lidar(LidarConfig {
            sweep_start_angle: 0.2182,
            ..LidarConfig::default()
        });
        let map = room();
        let scans: Vec<Scan> = (0..100)
            .filter_map(|_| lidar.sweep(&map, (2.0, 2.0, 0.0), (2.0, 2.0, 0.0), 0.01))
            .collect();
        assert_eq!(scans.len(), 5, "one second at 5 Hz");
        let scan = &scans[0];
        assert_eq!(scan.len(), 360);
        assert!((scan[0].0 - 0.2182).abs() < 1e-5);
        let step = (scan[0].0 - scan[1].0).rem_euclid(TAU);
        assert!(
            (step - 1f32.to_radians()).abs() < 1e-4,
            "clockwise by a degree"
        );
    }

    #[test]
    fn ranges_are_measured_from_the_mount() {
        let mut lidar = lidar(LidarConfig {
            mounting_x: -0.1,
            optical_offset: 0.02,
            ..LidarConfig::default()
        });
        let scan = scan_at(&mut lidar, &room(), (2.0, 2.0, 0.0), 0.01);
        // Ahead: the wall at x = 3.9 from the lidar at x = 1.9, less the optical offset.
        let (_, ahead, _) = scan.iter().find(|p| p.0.abs() < 1e-3).expect("beam 0");
        assert!((ahead - 1.98).abs() < 1e-4, "ahead {ahead}");
    }

    #[test]
    fn ranges_are_exact_at_any_angle() {
        // The wall at x = 3.9 seen 30° off from (2, 2): 1.9 / cos 30°.
        let mut lidar = lidar(LidarConfig::default());
        let scan = scan_at(&mut lidar, &room(), (2.0, 2.0, 0.0), 0.01);
        let (_, range, _) = scan
            .iter()
            .find(|p| (p.0 - 30f32.to_radians()).abs() < 1e-3)
            .expect("the 30° beam");
        let expected = 1.9 / 30f32.to_radians().cos();
        assert!((range - expected).abs() < 2e-3, "{range} vs {expected}");
    }

    #[test]
    fn beams_stop_among_the_folds_of_a_curtain() {
        // A 10 cm curtain at x in [3, 3.1), gray in the lidar mask: 60% of the beams return
        // from each centimetre of it.
        let map = room().with_lidar_mask(image::GrayImage::from_fn(400, 400, |i, j| {
            let wall = i < 10 || j < 10 || i >= 390 || j >= 390;
            image::Luma([if wall {
                0
            } else if (300..310).contains(&i) {
                100
            } else {
                255
            }])
        }));
        let mut lidar = lidar(LidarConfig::default());
        let at = (2.0, 2.0, 0.0);
        let ahead: Vec<f32> = (0..1000)
            .filter_map(|_| lidar.sweep(&map, at, at, 0.01))
            .flat_map(|scan| scan.into_iter().filter(|p| p.0.abs() < 1e-3).map(|p| p.1))
            .collect();
        assert!(ahead.len() > 40);
        assert!(
            ahead.iter().all(|&r| (0.999..1.1).contains(&r)),
            "{ahead:?}"
        );
        let (near, far) = ahead
            .iter()
            .fold((f32::MAX, 0.0f32), |(lo, hi), &r| (lo.min(r), hi.max(r)));
        assert!(far - near > 0.01, "not all at the first fold: {ahead:?}");
    }

    #[test]
    fn a_leg_blocks_the_rays_through_it_at_its_near_side() {
        let legs = [(1.0, 0.0)];
        let at = |heading: f32| leg_in_the_way(&legs, (0.0, 0.0), heading);
        assert!((at(0.0).unwrap() - 0.95).abs() < 1e-5);
        assert!(at(0.049).is_some() && at(-0.049).is_some());
        assert_eq!(at(0.06), None);
        assert_eq!(at(PI), None, "not behind");
    }

    #[test]
    fn people_walking_about_block_a_few_beams_each_revolution() {
        let map = room();
        let at = (2.0, 2.0, 0.0);
        let clear = scan_at(&mut lidar(LidarConfig::default()), &map, at, 0.01);
        let mut people = lidar(LidarConfig::default());
        people.config.noise.legs = 2.0;
        let blocked: Vec<usize> = (0..20)
            .map(|_| {
                let scan = scan_at(&mut people, &map, at, 0.01);
                assert_eq!(scan.len(), clear.len());
                (scan.iter().zip(&clear))
                    .filter(|(p, q)| p.1 < q.1 - 0.01)
                    .count()
            })
            .collect();
        // A 10 cm leg spans 2-19 beams at 2.5-0.3 m (none when beyond the wall); two of
        // them, now here, now there.
        assert!(blocked.iter().all(|&n| n <= 40), "{blocked:?}");
        assert!(blocked.iter().sum::<usize>() > 20 * 4, "{blocked:?}");
        assert!(blocked.windows(2).any(|w| w[0] != w[1]), "{blocked:?}");
    }

    #[test]
    fn beams_are_cast_from_where_the_robot_is_when_taken() {
        // Turning counter-clockwise as fast as the lidar turns clockwise, every beam looks
        // along +x: all of them see the same wall 1.9 m away, also in coarse steps.
        let mut lidar = lidar(LidarConfig::default());
        let map = room();
        let dt = 0.02;
        let mut theta = 0.0f32;
        let scan = loop {
            let turned = theta + dt * 5.0 * TAU;
            if let Some(scan) = lidar.sweep(&map, (2.0, 1.0, theta), (2.0, 1.0, turned), dt) {
                break scan;
            }
            theta = turned;
        };
        assert!(scan.iter().all(|p| (p.1 - 1.9).abs() < 1e-3), "{scan:?}");
    }
}
