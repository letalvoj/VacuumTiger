//! Proximity loop closure detection (Karto / slam_toolbox style).
//!
//! When a new keyframe lands near keyframes recorded long ago, its scan is
//! matched against those old keyframes stitched in the world frame:
//!
//! 1. **Candidates**: keyframes at least `min_keyframe_gap` ids older than the
//!    query and within `search_radius` of its (drifted) pose.
//! 2. **Coarse match**: a wide multi-resolution correlative search, so drift of
//!    tens of centimetres and degrees is recovered without a good initial guess.
//! 3. **Refinement**: the SLAM's own fine matcher from the coarse pose; a
//!    closure is accepted only if both agree.
//! 4. **Drift gate**: the closure may move the query only as far as odometry can
//!    drift over the path since the old keyframes; a larger correction means it
//!    matched a look-alike place (two identical bays), not the same one. A sharp
//!    match there passes Karto's response checks; this is a cheap stand-in for
//!    gating against the odometry's covariance.
//!
//! The constraint links the nearest candidate to the query, so the pose graph
//! can pull the drifted chain of keyframes back onto the old ones.

use crate::algorithms::matching::{MultiResolutionConfig, MultiResolutionMatcher, ScanMatcher};
use crate::core::math::normalize_angle;
use crate::core::types::{PointCloud2D, Pose2D};
use crate::engine::slam::keyframe::Keyframe;

use super::pose_graph::Information2D;

/// A verified loop closure: `relative_pose` is `T_match⁻¹ · T_query`.
#[derive(Debug, Clone)]
pub struct LoopClosure {
    /// ID of the query keyframe (current).
    pub query_id: u64,

    /// ID of the old keyframe the query was matched to.
    pub match_id: u64,

    /// Pose of the query in the frame of the matched keyframe.
    pub relative_pose: Pose2D,

    /// Information matrix for the constraint.
    pub information: Information2D,

    /// Fine match score (0-1).
    pub confidence: f32,
}

/// Configuration for loop detection.
#[derive(Debug, Clone, serde::Deserialize)]
#[serde(default)]
pub struct LoopDetectorConfig {
    /// Only keyframes at least this many ids older than the query are candidates
    /// (ids, not metres: with v8 keyframes every 0.2 m / 0.2 rad, 30 ids is a few metres
    /// of travel or a turn on the spot).
    pub min_keyframe_gap: u64,

    /// Candidates must lie within this distance (m) of the query pose.
    pub search_radius: f32,

    /// At most this many nearest candidates are stitched into the match target.
    pub area_keyframes: usize,

    /// Coarse search half-width in x and y (m).
    pub search_window_xy: f32,

    /// Coarse search half-width in heading (rad).
    pub search_window_theta: f32,

    /// Minimum coarse and fine match score.
    pub min_score: f32,

    /// The fine match may differ from the coarse one by at most this much (m)...
    pub max_refine_translation: f32,

    /// ...and this much (rad).
    pub max_refine_rotation: f32,

    /// A closure may move the query by at most this (m)...
    pub max_correction: f32,

    /// ...plus this much per metre driven since the oldest keyframe it matched (m/m).
    pub max_drift_per_m: f32,

    /// Standard deviation of a closure constraint in x and y (m).
    pub std_xy: f32,

    /// Standard deviation of a closure constraint in heading (rad).
    pub std_theta: f32,
}

impl Default for LoopDetectorConfig {
    fn default() -> Self {
        Self {
            min_keyframe_gap: 30,
            search_radius: 1.5,
            area_keyframes: 10,
            search_window_xy: 0.5,
            search_window_theta: 0.35,
            min_score: 0.6,
            max_refine_translation: 0.1,
            max_refine_rotation: 0.05,
            max_correction: 0.05,
            max_drift_per_m: 0.05,
            std_xy: 0.03,
            std_theta: 0.015,
        }
    }
}

/// Proximity-based loop closure detector with correlative verification.
pub struct LoopDetector {
    config: LoopDetectorConfig,

    /// Wide coarse matcher.
    coarse: MultiResolutionMatcher,

    /// Candidate keyframes stitched in the world frame (reused buffer).
    area: PointCloud2D,
}

impl LoopDetector {
    /// Create a new loop detector.
    pub fn new(config: LoopDetectorConfig) -> Self {
        let coarse = MultiResolutionMatcher::new(MultiResolutionConfig {
            coarse_search_window_xy: config.search_window_xy,
            coarse_search_window_theta: config.search_window_theta,
            min_score: config.min_score,
            ..MultiResolutionConfig::default()
        });
        Self {
            config,
            coarse,
            area: PointCloud2D::new(),
        }
    }

    /// Look for a closure between `query` and the older `keyframes`.
    ///
    /// `refine` is the fine matcher used for the final alignment.
    pub fn detect(
        &mut self,
        query: &Keyframe,
        keyframes: &[Keyframe],
        refine: &mut impl ScanMatcher,
    ) -> Option<LoopClosure> {
        let cfg = &self.config;
        let dist2 =
            |kf: &Keyframe| (kf.pose.x - query.pose.x).powi(2) + (kf.pose.y - query.pose.y).powi(2);
        let mut candidates: Vec<&Keyframe> = keyframes
            .iter()
            .filter(|kf| kf.id + cfg.min_keyframe_gap <= query.id)
            .filter(|kf| dist2(kf) <= cfg.search_radius.powi(2))
            .collect();
        candidates.sort_by(|a, b| dist2(a).total_cmp(&dist2(b)));
        candidates.truncate(cfg.area_keyframes);
        // The anchor is only the edge endpoint; heading does not matter since the match
        // is against the stitched cloud.
        let anchor = *candidates.first()?;

        self.area.clear();
        for kf in &candidates {
            kf.push_world_points(&mut self.area);
        }

        let coarse = self
            .coarse
            .match_scans(&query.scan, &self.area, &query.pose);
        if coarse.score < cfg.min_score {
            return None;
        }
        let fine = refine.match_scans(&query.scan, &self.area, &coarse.transform);
        let shift = coarse.transform.inverse().compose(&fine.transform);
        if !fine.converged
            || fine.score < cfg.min_score
            || shift.x.hypot(shift.y) > cfg.max_refine_translation
            || normalize_angle(shift.theta).abs() > cfg.max_refine_rotation
        {
            return None;
        }
        // Matched against all candidates: the drift is since the oldest of them.
        let oldest = (candidates.iter())
            .min_by_key(|kf| kf.id)
            .unwrap_or(&anchor);
        let path = path_between(keyframes, oldest, query);
        let allowed = cfg.max_correction + cfg.max_drift_per_m * path;
        let correction = query.pose.inverse().compose(&fine.transform);
        let moved = correction.x.hypot(correction.y);
        if moved > allowed {
            log::debug!(
                "Loop closure {} -> {} rejected: moves {moved:.2} m > {allowed:.2} m after \
                 {path:.2} m (score {:.2})",
                anchor.id,
                query.id,
                fine.score
            );
            return None;
        }

        Some(LoopClosure {
            query_id: query.id,
            match_id: anchor.id,
            relative_pose: anchor.pose.inverse().compose(&fine.transform),
            information: Information2D::from_std_dev(cfg.std_xy, cfg.std_xy, cfg.std_theta),
            confidence: fine.score,
        })
    }
}

/// The path from `from` to `to` (m) through the keyframes between them (`keyframes` in id
/// order), as SLAM placed them: chords, and pruned keyframes skipped, so at most the path
/// driven.
fn path_between(keyframes: &[Keyframe], from: &Keyframe, to: &Keyframe) -> f32 {
    let between = (keyframes.iter()).filter(|kf| kf.id > from.id && kf.id < to.id);
    let (path, _) = (between.map(|kf| &kf.pose))
        .chain([&to.pose])
        .fold((0.0, &from.pose), |(path, last), p| {
            (path + (p.x - last.x).hypot(p.y - last.y), p)
        });
    path
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algorithms::matching::{PointToLineIcp, PointToLineIcpConfig};

    /// A 4 x 3 m room with a box in one corner, seen from `pose`.
    /// Points carry a little jitter: the k-d tree rejects many identical coordinates.
    fn room_scan(pose: &Pose2D) -> PointCloud2D {
        let mut world = PointCloud2D::new();
        for i in 0..=80 {
            let t = i as f32 / 80.0;
            let e = (i % 7) as f32 * 1e-3;
            world.push_xy(t * 4.0, e);
            world.push_xy(t * 4.0, 3.0 + e);
            world.push_xy(e, t * 3.0);
            world.push_xy(4.0 + e, t * 3.0);
            world.push_xy(3.0 + t * 0.5, 2.0 + e);
            world.push_xy(3.0 + e, 2.0 + t);
        }
        world.transform(&pose.inverse())
    }

    fn keyframe(id: u64, believed: Pose2D, truth: Pose2D) -> Keyframe {
        Keyframe::new(id, believed, room_scan(&truth), 0, 0)
    }

    #[test]
    fn closes_a_drifted_revisit() {
        let truth = Pose2D::new(2.0, 1.5, 0.0);
        let old = keyframe(0, truth, truth);
        // Back at the same spot, but odometry believes it is 0.3 m and 8° off.
        let drifted = Pose2D::new(2.25, 1.35, 0.14);
        let query = keyframe(100, drifted, truth);
        // Drifted that far over a long way round: ~13 m via a waypoint out of the search
        // radius, which allows a 0.7 m correction.
        let away = keyframe(50, Pose2D::new(2.0, 8.0, 0.0), Pose2D::new(2.0, 8.0, 0.0));

        let mut detector = LoopDetector::new(LoopDetectorConfig::default());
        let mut fine = PointToLineIcp::new(PointToLineIcpConfig::default());
        let closure = detector
            .detect(&query, &[old, away], &mut fine)
            .expect("revisit should close");

        assert_eq!((closure.match_id, closure.query_id), (0, 100));
        let r = closure.relative_pose;
        assert!(r.x.hypot(r.y) < 0.03, "relative translation {r:?}");
        assert!(r.theta.abs() < 0.02, "relative rotation {r:?}");
    }

    #[test]
    fn a_correction_odometry_cannot_drift_over_a_short_path_is_no_closure() {
        // The same 0.3 m and 8° off, but having driven only those 0.3 m: a look-alike place.
        let truth = Pose2D::new(2.0, 1.5, 0.0);
        let old = keyframe(0, truth, truth);
        let query = keyframe(100, Pose2D::new(2.25, 1.35, 0.14), truth);

        let mut detector = LoopDetector::new(LoopDetectorConfig::default());
        let mut fine = PointToLineIcp::new(PointToLineIcpConfig::default());
        assert!(detector.detect(&query, &[old], &mut fine).is_none());
    }

    #[test]
    fn ignores_recent_and_distant_keyframes() {
        let pose = Pose2D::new(2.0, 1.5, 0.0);
        let recent = keyframe(90, pose, pose);
        let far = keyframe(0, Pose2D::new(-5.0, 0.0, 0.0), pose);
        let query = keyframe(100, pose, pose);

        let mut detector = LoopDetector::new(LoopDetectorConfig::default());
        let mut fine = PointToLineIcp::new(PointToLineIcpConfig::default());
        assert!(detector.detect(&query, &[far, recent], &mut fine).is_none());
    }
}
