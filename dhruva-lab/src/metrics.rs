//! Scores against a reference trajectory (no ground truth on real floors; the
//! reference is e.g. Cartographer on the same recording, docker/ref-slam).
//!
//! Same definitions as `tools/py/slam_eval.py` (which scores against a mock's
//! ground truth instead): the run is aligned to the reference with the best rigid
//! 2D transform (Umeyama on positions, no scale), then
//! - `ate_m`: RMS position error after alignment,
//! - `heading_rmse_deg`, `heading_final_deg`, `heading_max_deg`: per-pose heading
//!   error after the same rotation, wrapped to ±180°.

use std::path::Path;

use crate::PoseRow;

struct RefPose {
    t_us: u64,
    x: f64,
    y: f64,
    /// Unwrapped, so that interpolation between two poses is right.
    theta: f64,
}

fn read_reference(path: &Path) -> Result<Vec<RefPose>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut out: Vec<RefPose> = Vec::new();
    for line in text.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        let num = |i: usize| -> Option<f64> { f.get(i)?.parse().ok() };
        let (Some(t), Some(x), Some(y), Some(th)) = (num(0), num(2), num(3), num(4)) else {
            continue;
        };
        let theta = match out.last() {
            Some(p) => p.theta + wrap(th - p.theta),
            None => th,
        };
        out.push(RefPose {
            t_us: t as u64,
            x,
            y,
            theta,
        });
    }
    if out.len() < 2 {
        return Err(format!("{}: fewer than 2 poses", path.display()));
    }
    Ok(out)
}

/// Reference pose at `t_us` (linear interpolation), `None` outside its span.
fn at(reference: &[RefPose], t_us: u64) -> Option<(f64, f64, f64)> {
    let i = reference.partition_point(|p| p.t_us < t_us);
    if i == 0 || i == reference.len() {
        return None;
    }
    let (a, b) = (&reference[i - 1], &reference[i]);
    let s = (t_us - a.t_us) as f64 / (b.t_us - a.t_us).max(1) as f64;
    Some((
        a.x + (b.x - a.x) * s,
        a.y + (b.y - a.y) * s,
        a.theta + (b.theta - a.theta) * s,
    ))
}

pub fn against_reference(rows: &[PoseRow], path: &Path) -> Result<serde_json::Value, String> {
    let reference = read_reference(path)?;
    let pairs: Vec<((f64, f64, f64), (f64, f64, f64))> = rows
        .iter()
        .filter_map(|r| {
            let ours = (r.slam.x as f64, r.slam.y as f64, r.slam.theta as f64);
            Some((ours, at(&reference, r.t_us)?))
        })
        .collect();
    let n = pairs.len();
    if n < 2 {
        return Err("no overlap with the reference in time".into());
    }

    // Umeyama in 2D: rotation from the cross-covariance of the centred positions.
    let (mut mo, mut mr) = ((0.0, 0.0), (0.0, 0.0));
    for (o, r) in &pairs {
        mo = (mo.0 + o.0 / n as f64, mo.1 + o.1 / n as f64);
        mr = (mr.0 + r.0 / n as f64, mr.1 + r.1 / n as f64);
    }
    let (mut sxx, mut sxy) = (0.0, 0.0);
    for (o, r) in &pairs {
        let (ox, oy, rx, ry) = (o.0 - mo.0, o.1 - mo.1, r.0 - mr.0, r.1 - mr.1);
        sxx += ox * rx + oy * ry;
        sxy += ox * ry - oy * rx;
    }
    let rot = sxy.atan2(sxx);
    let (c, s) = (rot.cos(), rot.sin());

    let (mut sq_pos, mut sq_head, mut max_head, mut last_head) = (0.0, 0.0, 0.0f64, 0.0);
    for (o, r) in &pairs {
        let (ox, oy) = (o.0 - mo.0, o.1 - mo.1);
        let (ax, ay) = (c * ox - s * oy + mr.0, s * ox + c * oy + mr.1);
        sq_pos += (ax - r.0).powi(2) + (ay - r.1).powi(2);
        let e = wrap(o.2 + rot - r.2).to_degrees();
        sq_head += e * e;
        max_head = max_head.max(e.abs());
        last_head = e;
    }
    Ok(serde_json::json!({
        "poses": n,
        "ate_m": (sq_pos / n as f64).sqrt(),
        "heading_rmse_deg": (sq_head / n as f64).sqrt(),
        "heading_final_deg": last_head,
        "heading_max_deg": max_head,
    }))
}

fn wrap(a: f64) -> f64 {
    a.sin().atan2(a.cos())
}
