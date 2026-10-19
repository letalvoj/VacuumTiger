//! Hill-climbing the sweep's `[exploration.wall_follow]` settings on the bench's cost.

use dhruva_slam::config::Config;
use dhruva_slam::exploration::WallFollowConfig;
use sangamio::devices::mock::config::SimulationConfig;

use crate::scene::Scene;
use crate::{evaluate, total};

/// A setting the climb may change: its name, where it is, its bounds, and the factor
/// (> 1) of a step.
pub struct Knob {
    name: &'static str,
    at: fn(&mut WallFollowConfig) -> &mut f32,
    min: f32,
    max: f32,
    factor: f32,
}

#[rustfmt::skip]
const KNOBS: &[Knob] = &[
    Knob { name: "speed", at: |c| &mut c.speed, min: 0.1, max: 0.3, factor: 1.15 },
    Knob { name: "turn_rate", at: |c| &mut c.turn_rate, min: 0.3, max: 1.2, factor: 1.2 },
];

impl Knob {
    pub fn at<'a>(&self, config: &'a mut WallFollowConfig) -> &'a mut f32 {
        (self.at)(config)
    }

    /// The two neighbours of `x` (down, up), within the bounds.
    fn steps(&self, x: f32) -> [f32; 2] {
        [x / self.factor, x * self.factor].map(|v| v.clamp(self.min, self.max))
    }
}

pub fn knob(name: &str) -> Result<&'static Knob, String> {
    let names: Vec<_> = KNOBS.iter().map(|k| k.name).collect();
    (KNOBS.iter().find(|k| k.name == name))
        .ok_or_else(|| format!("no setting {name} ({})", names.join(", ")))
}

/// A stochastic hill climb from `config`'s settings: each trial moves every knob a step
/// down, up or not at all, at random, and is kept if the total cost drops (the cost is
/// jumpy: a lap closes or not, a bump more or less, so moving one knob at a time stalls).
/// Leaves the best in `config` and prints it as TOML and as a `--set` line.
pub fn climb(
    scenes: &[Scene],
    config: &mut Config,
    sim: &SimulationConfig,
    seeds: u64,
    trials: usize,
) -> Result<(), String> {
    let mut best = total(&evaluate(scenes, config, sim, seeds)?);
    println!("start {best:.1}");
    let mut rng = 0x2545_F491_4F6C_DD1Du64;
    let mut roll = || {
        rng ^= rng << 13;
        rng ^= rng >> 7;
        rng ^= rng << 17;
        rng % 3
    };
    let mut kept = config.exploration.wall_follow;
    for trial in 0..trials {
        let mut candidate = kept;
        for knob in KNOBS {
            let x = *knob.at(&mut candidate);
            if let Some(&next) = knob.steps(x).get(roll() as usize) {
                *knob.at(&mut candidate) = next;
            }
        }
        config.exploration.wall_follow = candidate;
        let cost = total(&evaluate(scenes, config, sim, seeds)?);
        if cost < best {
            println!("trial {trial}: {best:.1} -> {cost:.1}");
            (best, kept) = (cost, candidate);
        }
    }
    config.exploration.wall_follow = kept;
    println!("\n[exploration.wall_follow]");
    let mut set = Vec::new();
    for knob in KNOBS {
        let value = *knob.at(&mut kept);
        println!("{} = {value:.3}", knob.name);
        set.push(format!("{}={value:.3}", knob.name));
    }
    println!("\n--set {}\n", set.join(","));
    Ok(())
}
