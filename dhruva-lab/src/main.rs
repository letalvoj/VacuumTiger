//! dhruva-lab: replay a recording through dhruva-slam's pipeline one scan at a
//! time and record what the engine believes after each step.
//!
//! The SLAM code stays in `dhruva-slam`; this crate only drives it and looks at
//! it. Outputs in `--out`:
//! - `trace.mcap`: the step-by-step trace (see `trace.rs`), for Foxglove/Lichtblick
//! - `map.png`: final map with the SLAM path, odometry path and pose graph
//! - `traj.csv`: `t_us,ms,x,y,theta,score,keyframes,points` (DhruvaSLAM's
//!   `timing.csv` columns, so `tools/py/slam_eval.py` reads it)
//! - `metrics.json`: scores against `--reference` (another run's `traj.csv`)

mod metrics;
mod render;
mod trace;

use std::io::Write;
use std::path::PathBuf;
use std::time::Instant;

use clap::Parser;
use dhruva_slam::config::{Config, build_slam_thread_config};
use dhruva_slam::core::types::{LaserScan, Pose2D};
use dhruva_slam::engine::pipeline::SlamPipeline;
use dhruva_slam::engine::slam::{SlamEngine, SlamMode};
use dhruva_slam::io::recording::{Sample, for_each_sample};

#[derive(Parser)]
#[command(about = "Replay a recording through dhruva-slam and trace every step")]
struct Args {
    /// dhruva-slam TOML config (e.g. dhruva-slam/dhruva-v8.toml).
    #[arg(short, long)]
    config: PathBuf,
    /// Recording: an MCAP file, a run directory of MCAP segments, or SGIO1.
    recording: PathBuf,
    /// Output directory.
    #[arg(long, default_value = "lab_out")]
    out: PathBuf,
    /// Reference trajectory to score against (`traj.csv`/`timing.csv` columns
    /// t_us,_,x,y,theta; e.g. a Cartographer run from docker/ref-slam).
    #[arg(long)]
    reference: Option<PathBuf>,
    /// Data seconds between map snapshots in the trace (an unchanged map is skipped).
    #[arg(long, default_value_t = 5.0, value_parser = positive)]
    map_every: f64,
    /// Replay only part of the recording: A:B as fractions of its sensor time span
    /// (0.4:0.6 = the middle fifth), starting cold like a run that started there.
    /// Same cut as `tools/py/sgio2bag.py --window` makes for the reference SLAMs.
    #[arg(long, default_value = "0:1", value_parser = window)]
    window: (f64, f64),
}

fn window(s: &str) -> Result<(f64, f64), String> {
    let (a, b) = s.split_once(':').ok_or("expected A:B")?;
    let (a, b): (f64, f64) = (
        a.parse().map_err(|_| format!("{a}: not a number"))?,
        b.parse().map_err(|_| format!("{b}: not a number"))?,
    );
    if (0.0..b).contains(&a) && b <= 1.0 {
        Ok((a, b))
    } else {
        Err(format!("{s}: need 0 <= A < B <= 1"))
    }
}

fn positive(s: &str) -> Result<f64, String> {
    match s.parse::<f64>() {
        Ok(v) if v > 0.0 => Ok(v),
        _ => Err(format!("{s}: not a positive number")),
    }
}

/// One processed scan, for the outputs written at the end.
pub(crate) struct PoseRow {
    pub t_us: u64,
    pub ms: f64,
    pub slam: Pose2D,
    pub odom: Pose2D,
    pub score: f32,
    pub keyframes: usize,
    pub points: usize,
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    if let Err(e) = run(Args::parse()) {
        eprintln!("dhruva-lab: {e}");
        std::process::exit(1);
    }
}

fn run(args: Args) -> Result<(), String> {
    let config = Config::from_file(&args.config)?;
    let pipeline_config = build_slam_thread_config(&config).pipeline;
    std::fs::create_dir_all(&args.out).map_err(|e| format!("{}: {e}", args.out.display()))?;

    let mut pipeline = SlamPipeline::new(&pipeline_config);
    pipeline.slam_mut().set_mode(SlamMode::Mapping);
    let mut trace = trace::Trace::create(&args.out.join("trace.mcap"))?;
    let mut rows: Vec<PoseRow> = Vec::new();
    let map_every_us = (args.map_every * 1e6) as u64;
    let mut last_map_us = 0u64;
    let wall = Instant::now();

    let (lo, hi) = window_bounds_us(&args.recording, args.window)?;
    for_each_sample(&args.recording, |sample| {
        // Compared as f64 on purpose: the same cut as sgio2bag.py, see window_bounds_us.
        let t = sample.timestamp_us() as f64;
        if t < lo || t > hi {
            return Ok(());
        }
        match sample {
            Sample::Status(s) => {
                pipeline.on_odometry(
                    s.left_encoder,
                    s.right_encoder,
                    s.gyro_yaw_raw,
                    s.timestamp_us,
                );
            }
            Sample::Lidar(l) => {
                let t0 = Instant::now();
                let laser = LaserScan::from_lidar_scan(&l.points);
                let Some(step) = pipeline.on_scan(laser, l.timestamp_us, true) else {
                    return Ok(());
                };
                let ms = t0.elapsed().as_secs_f64() * 1e3;
                let Some(result) = &step.result else {
                    return Ok(());
                };
                let keyframes = pipeline.slam().status().num_keyframes;
                trace.scan(&step, result, pipeline.slam().pose_graph(), keyframes, ms);
                rows.push(PoseRow {
                    t_us: step.timestamp_us,
                    ms,
                    slam: result.pose,
                    odom: step.odom_pose,
                    score: result.match_score,
                    keyframes,
                    points: step.cloud.len(),
                });
                if step.timestamp_us.saturating_sub(last_map_us) >= map_every_us {
                    trace.map(step.timestamp_us, pipeline.slam_mut().global_map());
                    last_map_us = step.timestamp_us;
                }
            }
        }
        Ok(())
    })
    .map_err(|e| format!("{}: {e}", args.recording.display()))?;

    let wall_s = wall.elapsed().as_secs_f64();
    if let Some(last) = rows.last() {
        trace.map(last.t_us, pipeline.slam_mut().global_map());
    }
    trace.finish()?;

    write_traj(&args.out.join("traj.csv"), &rows)?;
    let grid = pipeline.slam_mut().global_map().clone();
    let graph = pipeline.slam().pose_graph();
    render::map_png(&args.out.join("map.png"), &grid, &rows, graph)?;

    let mut summary = serde_json::json!({
        "scans": rows.len(),
        "wall_s": wall_s,
        "mean_ms": rows.iter().map(|r| r.ms).sum::<f64>() / rows.len().max(1) as f64,
        "keyframes": rows.last().map_or(0, |r| r.keyframes),
        "graph_nodes": graph.num_nodes(),
        "loop_closures": graph.num_loop_closures(),
    });
    if let Some(reference) = &args.reference {
        let score = metrics::against_reference(&rows, reference)?;
        summary["reference"] = score;
    }
    let text = serde_json::to_string_pretty(&summary).map_err(|e| e.to_string())?;
    std::fs::write(args.out.join("metrics.json"), &text).map_err(|e| e.to_string())?;
    println!("{text}");
    println!(
        "-> {}/{{trace.mcap,map.png,traj.csv,metrics.json}}",
        args.out.display()
    );
    Ok(())
}

/// The inclusive data-time bounds (us) of a window given as fractions of the
/// recording's sensor time span.
///
/// Must cut exactly where `tools/py/sgio2bag.py --window` (`window_us`) cuts the
/// bags for the reference SLAMs, so it does the same IEEE arithmetic: f64 bounds
/// `t0 + f * (t1 - t0)`, compared with the f64 sample time (exact: < 2^53).
fn window_bounds_us(recording: &std::path::Path, (a, b): (f64, f64)) -> Result<(f64, f64), String> {
    if (a, b) == (0.0, 1.0) {
        return Ok((f64::NEG_INFINITY, f64::INFINITY));
    }
    let (mut t0, mut t1) = (u64::MAX, 0);
    for_each_sample(recording, |s| {
        t0 = t0.min(s.timestamp_us());
        t1 = t1.max(s.timestamp_us());
        Ok(())
    })
    .map_err(|e| format!("{}: {e}", recording.display()))?;
    if t0 > t1 {
        return Err(format!("{}: no sensor samples", recording.display()));
    }
    let at = |f: f64| t0 as f64 + f * (t1 - t0) as f64;
    Ok((at(a), at(b)))
}

fn write_traj(path: &std::path::Path, rows: &[PoseRow]) -> Result<(), String> {
    let err = |e: std::io::Error| format!("{}: {e}", path.display());
    let mut f = std::io::BufWriter::new(std::fs::File::create(path).map_err(err)?);
    writeln!(f, "t_us,ms,x,y,theta,score,keyframes,points").map_err(err)?;
    for r in rows {
        writeln!(
            f,
            "{},{:.3},{:.4},{:.4},{:.5},{:.3},{},{}",
            r.t_us, r.ms, r.slam.x, r.slam.y, r.slam.theta, r.score, r.keyframes, r.points
        )
        .map_err(err)?;
    }
    f.flush().map_err(err)
}
