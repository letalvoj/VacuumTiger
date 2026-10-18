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

    for_each_sample(&args.recording, |sample| {
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
