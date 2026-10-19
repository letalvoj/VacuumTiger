//! The corner-sweep bench: dhruva-slam's exploration and SLAM against SangamIO's mock of
//! the v8, closed loop and faster than real time, in small scenes after what the real runs
//! met. It prints a row per scene and seed and a total cost; `--climb` hill-climbs the
//! sweep's settings on that cost.
//!
//!     cargo run --release --bin sweep -- [--seeds N] [--only NAME] [--set K=V,...]
//!         [--climb TRIALS] [--dump DIR] [--reasons]
//!
//! - The robot is `--sim` (default `sangamio/mock_bench.toml`, the v8 as measured), the
//!   software `--config` (default `dhruva-slam/dhruva-v8.toml`, as on the robot). Each
//!   models its own side: dhruva-slam's gyro scale is what it assumes, the mock's what
//!   was measured, and the 2.5% between them is real.
//! - `--set` changes `[exploration.wall_follow]` settings before the runs (see `climb`).
//! - `--dump` writes `<DIR>/<scene>_<seed>.json` for `tools/py/wf_sim_plot.py`.

mod climb;
mod run;
mod scene;
mod score;

use std::path::{Path, PathBuf};

use clap::Parser;
use dhruva_slam::config::Config;
use sangamio::devices::mock::config::SimulationConfig;

use crate::run::Run;
use crate::scene::Scene;
use crate::score::Score;

/// The software as on the robot, and the robot as measured.
const CONFIG: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../dhruva-slam/dhruva-v8.toml");
const SIM: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../sangamio/mock_bench.toml");

#[derive(Parser)]
struct Args {
    /// dhruva-slam's configuration.
    #[arg(long, default_value = CONFIG)]
    config: PathBuf,
    /// SangamIO's configuration with the mock's `[device.simulation]`.
    #[arg(long, default_value = SIM)]
    sim: PathBuf,
    /// Runs per scene, each with its own noise.
    #[arg(long, default_value_t = 2)]
    seeds: u64,
    /// Only this scene.
    #[arg(long)]
    only: Option<String>,
    /// Settings to change first: KEY=VALUE[,KEY=VALUE...].
    #[arg(long, value_delimiter = ',')]
    set: Vec<String>,
    /// Hill-climb the settings for this many trials, then run the best.
    #[arg(long, default_value_t = 0)]
    climb: usize,
    /// Write each run as JSON here.
    #[arg(long)]
    dump: Option<PathBuf>,
    /// Print the seconds spent per session status.
    #[arg(long)]
    reasons: bool,
}

/// One scene and seed: its run and score.
pub struct Outcome {
    pub scene: &'static str,
    pub seed: u64,
    pub run: Run,
    pub score: Score,
}

/// Every scene and seed with `config`, in parallel.
pub fn evaluate(
    scenes: &[Scene],
    config: &Config,
    sim: &SimulationConfig,
    seeds: u64,
) -> Result<Vec<Outcome>, String> {
    std::thread::scope(|scope| {
        let handles: Vec<_> = (scenes.iter())
            .flat_map(|scene| (1..=seeds).map(move |seed| (scene, seed)))
            .map(|(scene, seed)| {
                scope.spawn(move || {
                    let run = run::run(scene, config, sim.clone(), seed)?;
                    let score = score::score(scene, &run);
                    Ok(Outcome {
                        scene: scene.name,
                        seed,
                        run,
                        score,
                    })
                })
            })
            .collect();
        (handles.into_iter())
            .map(|h| h.join().map_err(|_| "a run panicked".to_string())?)
            .collect()
    })
}

/// One number per run to minimise. A point is a centimetre of mean unswept depth along
/// the edges (the main term), or 3.3 cm of a corner missed beyond 5 cm, or 2 bumps, or
/// 30 s, or 10 s of loitering beyond 30 s; a lap not closed costs 10.
pub fn cost(s: &Score) -> f32 {
    let corners: f32 = s.corners.iter().map(|c| (c - 0.05).max(0.0) * 100.0).sum();
    let mut c = s.missed_depth * 100.0 + 0.3 * corners;
    c += s.seconds / 30.0;
    c += 0.5 * s.bumps as f32;
    c += (s.loiter - 30.0).max(0.0) / 10.0;
    if !s.completed {
        c += 10.0;
    }
    c
}

/// dhruva-slam's configuration and the mock's `[device.simulation]`.
pub fn settings(config: &Path, sim: &Path) -> Result<(Config, SimulationConfig), String> {
    let simulation = (sangamio::Config::load(sim)
        .map_err(|e| e.to_string())?
        .device
        .simulation)
        .ok_or_else(|| format!("{}: no [device.simulation]", sim.display()))?;
    Ok((Config::from_file(config)?, simulation))
}

pub fn total(outcomes: &[Outcome]) -> f32 {
    outcomes.iter().map(|o| cost(&o.score)).sum()
}

fn main() -> Result<(), String> {
    env_logger::init();
    let args = Args::parse();
    let (mut config, sim) = settings(&args.config, &args.sim)?;
    for kv in &args.set {
        let (key, value) = kv.split_once('=').ok_or(format!("--set {kv}: KEY=VALUE"))?;
        let value = value.parse().map_err(|e| format!("--set {kv}: {e}"))?;
        *climb::knob(key)?.at(&mut config.exploration.wall_follow) = value;
    }
    let scenes: Vec<Scene> = (scene::scenes().into_iter())
        .filter(|s| args.only.as_ref().is_none_or(|o| o == s.name))
        .collect();
    if scenes.is_empty() {
        return Err(format!("no scene {:?}", args.only));
    }
    if args.climb > 0 {
        climb::climb(&scenes, &mut config, &sim, args.seeds, args.climb)?;
    }

    let outcomes = evaluate(&scenes, &config, &sim, args.seeds)?;
    println!(
        "{:<16} {:>4} {:>6} {:>5} {:>8} {:>8} {:>7} {:>6}",
        "scene", "lap", "s", "bumps", "miss cm", "corner", "loiter", "cost"
    );
    for o in &outcomes {
        let s = &o.score;
        println!(
            "{:<16} {:>4} {:>6.0} {:>5} {:>8.1} {:>8.1} {:>7.0} {:>6.1}  {}",
            format!("{}#{}", o.scene, o.seed),
            if s.completed { "yes" } else { "NO" },
            s.seconds,
            s.bumps,
            s.missed_depth * 100.0,
            s.worst_corner() * 100.0,
            s.loiter,
            cost(s),
            o.run.ended.as_deref().unwrap_or_default()
        );
        if let Some(dir) = &args.dump {
            let scene = scenes
                .iter()
                .find(|s| s.name == o.scene)
                .expect("its scene");
            std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
            let path = dir.join(format!("{}_{}.json", o.scene, o.seed));
            std::fs::write(&path, score::to_json(scene, &o.run, s).to_string())
                .map_err(|e| format!("{}: {e}", path.display()))?;
        }
    }
    println!("total {:.1}", total(&outcomes));
    if args.reasons {
        print_reasons(&outcomes);
    }
    Ok(())
}

/// Seconds per session status over all runs, and the mean speed commanded in each.
fn print_reasons(outcomes: &[Outcome]) {
    let mut by: Vec<(&str, usize, f32)> = Vec::new();
    for o in outcomes {
        for (status, speed) in &o.run.ticks {
            match by.iter_mut().find(|r| r.0 == status) {
                Some(r) => {
                    r.1 += 1;
                    r.2 += speed;
                }
                None => by.push((status, 1, *speed)),
            }
        }
    }
    by.sort_by_key(|r| std::cmp::Reverse(r.1));
    println!("\n{:<40} {:>7} {:>8}", "status", "ticks", "mean v");
    for (status, ticks, speed) in by {
        println!("{status:<40} {ticks:>7} {:>8.3}", speed / ticks as f32);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// wall_follow's scenario tests: every scene as on the robot, with a seed the climb did
    /// not see (it climbs on the first few).
    #[test]
    fn the_v8_sweep_closes_every_lap_and_reaches_every_corner() {
        let (config, sim) = settings(CONFIG.as_ref(), SIM.as_ref()).unwrap();
        for scene in scene::scenes() {
            let run = run::run(&scene, &config, sim.clone(), 7).unwrap();
            let s = score::score(&scene, &run);
            assert!(s.completed, "{}: {:?}", scene.name, run.ended);
            // The body reaches 6 cm into a corner (0.1 m allows for noise); the curtain's
            // corners are behind the fabric.
            let reach = if scene.curtains.is_empty() { 0.1 } else { 0.2 };
            assert!(
                s.worst_corner() < reach,
                "{}: corners {:?}",
                scene.name,
                s.corners
            );
        }
    }
}
