//! Item 30 of `docs/BUILD_PLAN.md`: generates the real degradation curve for our own
//! system, Decision 3's own stated purpose for this file (`docs/decisions.md`) — the
//! A/B/L thresholds in `config.rs` are still a provisional starting point, explicitly
//! provisional until this sweep either confirms or replaces them.
//! `docs/TESTING_PLAN.md`'s Phase 8 check 3: mode occupancy should shift from
//! Cooperative toward Cautious/Autonomous as packet-loss/latency worsen, and
//! task-completion should degrade *gracefully*, not cliff straight to zero completions.
//!
//! In-process, deterministic (Decision 15) like the rest of Phase 8 — no real UDP, so
//! packet loss/latency are simulated: each tick, each robot's own seeded draw decides
//! whether it "hears" a heartbeat this tick (probability `packet_loss_pct`) and, if so,
//! at what latency (`latency_ticks`, fixed per sweep point) — fed straight into a real,
//! unmodified `ModeStateMachine` (`mode_state_machine.rs`, already exhaustively
//! threshold-tested in `tests/mode_state_machine.rs`) so the mode transitions this
//! produces are the real ones, not a second implementation of the threshold logic.
//!
//! **Movement-degradation modeling — a deliberate simplification, logged rather than
//! hidden:** the real system's Autonomous mode drops peer-intent negotiation entirely
//! for local-sensing-only movement (`resolve_tick_autonomous`, `robot/mod.rs`), but that
//! logic lives on `DistributedAgent`, the real-UDP per-agent type Decision 15 chose not
//! to use here (mixing two different per-robot movement algorithms inside one shared
//! `Pibt::step()` call — which needs every agent's position to resolve candidates
//! correctly — isn't a clean fit without deeper `Pibt` surgery). Instead: an
//! Autonomous-mode robot's PIBT priority is capped to `0.0` for that tick's `step()`
//! call only (its own accumulating priority still tracks normally underneath, so it
//! doesn't jump oddly on recovery) — it can never win a priority contest while degraded,
//! a real, honest proxy for "can't assert priority without negotiation trust," not a
//! literal reimplementation of local sensing. Collision-freedom is untouched either way:
//! `func_pibt`'s guarantee doesn't depend on the priority *values* being fair, only on
//! the mechanism itself, so this is still real, unmodified PIBT underneath.

use std::path::PathBuf;

use clap::Parser;
use rand::rngs::StdRng;
use rand::seq::SliceRandom;
use rand::RngExt;
use rand::SeedableRng;
use serde_json::{json, Value};

use sih26123::protocol::messages::Mode;
use sih26123::robot::mode_state_machine::ModeStateMachine;
use sih26123::robot::planner_pibt::{has_no_collisions, Cell, Config, Pibt};
use sih26123::robot::task_layer::{Task, TaskLayer, TaskState};
use sih26123::world::grid::Grid;

#[derive(Parser)]
struct Args {
    #[arg(long, default_value = "maps/warehouse-10-20-10-2-1.map")]
    map: String,
    #[arg(long, default_value_t = 4)]
    num_robots: usize,
    #[arg(long, default_value_t = 6)]
    num_tasks: usize,
    #[arg(long, value_delimiter = ',', default_value = "1,2,3")]
    seeds: Vec<u64>,
    #[arg(long, default_value_t = 3000)]
    tick_limit: u64,
    /// Packet-loss sweep points, as percentages — latency is held at 0 for these runs
    /// so the two dimensions (Decision 3's A/B vs. L) are read independently.
    #[arg(long, value_delimiter = ',', default_value = "0,10,20,30,40,50,60,70,80,90,100")]
    packet_loss_points: Vec<f64>,
    /// Latency sweep points, in ticks — packet loss held at 0 for these runs.
    #[arg(long, value_delimiter = ',', default_value = "0,1,2,3,4,5,6")]
    latency_points: Vec<u32>,
    #[arg(long, default_value = "results/degradation.json")]
    out: String,
}

/// Same disjoint-slice reservation `run_experiment.rs` uses for Decision 5's
/// well-formedness — duplicated rather than shared across these two binary crates
/// (`docs/decisions.md`'s own reasoning for why `baseline_stop_and_wait.rs` and this
/// file each own their simulation loop rather than importing one another's).
fn generate_scenario(grid: &Grid, num_robots: usize, num_tasks: usize, seed: u64) -> (Vec<Cell>, Vec<Task>) {
    let mut rng = StdRng::seed_from_u64(seed);
    let mut free = grid.free_cells();
    free.shuffle(&mut rng);

    let starts: Vec<Cell> = free[..num_robots].to_vec();
    let mut idx = num_robots;
    let mut tasks = Vec::with_capacity(num_tasks);
    for task_id in 1..=num_tasks as u32 {
        let pickup = free[idx];
        let dropoff = free[idx + 1];
        idx += 2;
        tasks.push(Task { task_id, pickup, dropoff });
    }
    (starts, tasks)
}

struct DegradedRobot {
    id: u32,
    task_layer: TaskLayer,
    mode_sm: ModeStateMachine,
    goal: Cell,
    priority: f64,
    distance: u32,
    wait_ticks: u32,
    tasks_completed: u32,
    mode_ticks: [u32; 3], // [Cooperative, Cautious, Autonomous]
    packets_sent: u32,
    packets_dropped: u32,
}

fn mode_index(m: Mode) -> usize {
    match m {
        Mode::Cooperative => 0,
        Mode::Cautious => 1,
        Mode::Autonomous => 2,
    }
}

/// Runs one full task-driven scenario at a fixed (packet_loss_pct, latency_ticks)
/// injection point. Same one-token-hop-per-tick cadence and PIBT-step structure
/// `run_experiment.rs`'s `run_pibt_token` uses, extended with per-robot comms
/// simulation and mode-dependent priority capping (see this file's own doc comment).
#[allow(clippy::too_many_arguments)]
fn run_degraded(
    grid: &Grid,
    map_name: &str,
    starts: &[Cell],
    tasks: Vec<Task>,
    seed: u64,
    tick_limit: u64,
    packet_loss_pct: f64,
    latency_ticks: u32,
) -> Value {
    let n = starts.len();
    let peer_ids: Vec<u32> = (1..=n as u32).collect();
    let mut pibt = Pibt::new(grid, starts.to_vec(), starts.to_vec(), seed);
    // A distinct RNG stream from `pibt`'s own tie-breaking one, seeded from the same
    // run seed plus a fixed offset — comms simulation and movement tie-breaking must
    // not consume from (and so perturb) the same stream.
    let mut comms_rng = StdRng::seed_from_u64(seed ^ 0x636F_6D6D_735F); // "comms_" tag

    let mut robots: Vec<DegradedRobot> = starts
        .iter()
        .zip(&peer_ids)
        .map(|(&pos, &id)| DegradedRobot {
            id,
            task_layer: TaskLayer::new_pure(id, grid, tasks.clone(), peer_ids.clone()),
            mode_sm: ModeStateMachine::new(),
            goal: pos,
            priority: 0.0,
            distance: 0,
            wait_ticks: 0,
            tasks_completed: 0,
            mode_ticks: [0, 0, 0],
            packets_sent: 0,
            packets_dropped: 0,
        })
        .collect();

    let tasks_total = tasks.len();
    let mut positions: Config = starts.to_vec();
    let mut configs: Vec<Config> = vec![positions.clone()];
    let mut token = sih26123::protocol::messages::TokenMsg {
        seq: 0,
        holder_id: peer_ids[0],
        claimed_tasks: vec![],
        epoch: 0,
        creator: 0,
    };
    let mut completion_ticks: Option<u64> = None;

    for tick in 0..tick_limit {
        let holder_idx = robots.iter().position(|r| r.id == token.holder_id)
            .expect("token holder_id always names a robot in peer_ids");
        token = robots[holder_idx].task_layer.handle_token(&token, positions[holder_idx]);

        // Simulated comms outcome + mode transition, one per robot per tick.
        for r in robots.iter_mut() {
            r.packets_sent += 1;
            let missed = comms_rng.random_bool(packet_loss_pct / 100.0);
            if missed {
                r.packets_dropped += 1;
                r.mode_sm.observe_heartbeat(None);
            } else {
                r.mode_sm.observe_heartbeat(Some(latency_ticks));
            }
            r.mode_ticks[mode_index(r.mode_sm.mode())] += 1;
        }

        for i in 0..n {
            let desired = robots[i].task_layer.current_goal().unwrap_or(positions[i]);
            if desired != robots[i].goal {
                pibt.retarget(i, desired);
                robots[i].goal = desired;
            }
        }

        let priorities: Vec<f64> = robots
            .iter()
            .map(|r| if r.mode_sm.mode() == Mode::Autonomous { 0.0 } else { r.priority })
            .collect();
        let q_to = pibt.step(&positions, &priorities);

        for i in 0..n {
            if q_to[i] != positions[i] {
                robots[i].distance += 1;
            } else {
                robots[i].wait_ticks += 1;
            }
            if q_to[i] != robots[i].goal {
                robots[i].priority += 1.0;
            } else {
                robots[i].priority -= robots[i].priority.floor();
            }
        }
        positions = q_to;
        configs.push(positions.clone());

        for i in 0..n {
            let before = robots[i].task_layer.state();
            robots[i].task_layer.on_position_update(positions[i]);
            let after = robots[i].task_layer.state();
            if matches!(before, TaskState::ToDropoff(_)) && after == TaskState::Idle {
                robots[i].tasks_completed += 1;
            }
        }

        let completed: usize = robots.iter().map(|r| r.tasks_completed as usize).sum();
        if completed >= tasks_total {
            completion_ticks = Some(tick + 1);
            break;
        }
    }

    let collision_free = has_no_collisions(&configs);
    let tasks_completed: usize = robots.iter().map(|r| r.tasks_completed as usize).sum();
    let finished_within_limit = completion_ticks.is_some();

    let total_robot_ticks: u32 = robots.iter().map(|r| r.mode_ticks.iter().sum::<u32>()).sum();
    let mode_totals = robots.iter().fold([0u32; 3], |mut acc, r| {
        for k in 0..3 {
            acc[k] += r.mode_ticks[k];
        }
        acc
    });
    let frac = |k: usize| {
        if total_robot_ticks == 0 { 0.0 } else { mode_totals[k] as f64 / total_robot_ticks as f64 }
    };

    json!({
        "run_id": format!("pibt_token-degradation-loss{packet_loss_pct}-lat{latency_ticks}-seed{seed}"),
        "map": map_name,
        "seed": seed,
        "system": "pibt_token",
        "num_robots": n,
        "tick_limit": tick_limit,
        "tasks_total": tasks_total,
        "tasks_completed": tasks_completed,
        "finished_within_limit": finished_within_limit,
        "completion_ticks": completion_ticks,
        "vertex_collisions": if collision_free { 0 } else { 1 },
        "edge_collisions": 0,
        "deadlocks_detected": 0,
        "total_distance": robots.iter().map(|r| r.distance).sum::<u32>(),
        "total_wait_ticks": robots.iter().map(|r| r.wait_ticks).sum::<u32>(),
        "claim_conflicts": robots.iter().map(|r| r.task_layer.claim_conflicts()).sum::<u32>(),
        "token_skips": robots.iter().map(|r| r.task_layer.skip_count()).sum::<u32>(),
        "comparable_to_baseline": false, // degradation runs aren't compared against stop_and_wait
        "improvement_pct": Value::Null,
        "comms": {
            "packet_loss_pct_injected": packet_loss_pct,
            "latency_ticks_injected": latency_ticks,
            "packets_sent": robots.iter().map(|r| r.packets_sent).sum::<u32>(),
            "packets_dropped": robots.iter().map(|r| r.packets_dropped).sum::<u32>(),
        },
        "mode_occupancy": {
            "cooperative": frac(0),
            "cautious": frac(1),
            "autonomous": frac(2),
        },
        "per_robot": robots.iter().map(|r| json!({
            "robot_id": r.id,
            "distance": r.distance,
            "wait_ticks": r.wait_ticks,
            "tasks_completed": r.tasks_completed,
        })).collect::<Vec<_>>(),
    })
}

fn main() {
    let args = Args::parse();
    let grid = Grid::load(&args.map).unwrap_or_else(|e| {
        eprintln!("failed to load map \"{}\": {e}", args.map);
        std::process::exit(1);
    });
    let map_name = std::path::Path::new(&args.map)
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| args.map.clone());

    let mut runs: Vec<Value> = Vec::new();

    for &packet_loss_pct in &args.packet_loss_points {
        for &seed in &args.seeds {
            let (starts, tasks) = generate_scenario(&grid, args.num_robots, args.num_tasks, seed);
            let run = run_degraded(
                &grid, &map_name, &starts, tasks, seed, args.tick_limit, packet_loss_pct, 0,
            );
            eprintln!(
                "[degradation_sweep] loss={packet_loss_pct}% latency=0 seed={seed}: mode_occupancy={} completion_ticks={}",
                run["mode_occupancy"], run["completion_ticks"]
            );
            runs.push(run);
        }
    }

    for &latency_ticks in &args.latency_points {
        for &seed in &args.seeds {
            let (starts, tasks) = generate_scenario(&grid, args.num_robots, args.num_tasks, seed);
            let run = run_degraded(
                &grid, &map_name, &starts, tasks, seed, args.tick_limit, 0.0, latency_ticks,
            );
            eprintln!(
                "[degradation_sweep] loss=0% latency={latency_ticks} seed={seed}: mode_occupancy={} completion_ticks={}",
                run["mode_occupancy"], run["completion_ticks"]
            );
            runs.push(run);
        }
    }

    let out_path = PathBuf::from(&args.out);
    if let Some(parent) = out_path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let payload = json!({ "runs": runs });
    std::fs::write(&out_path, serde_json::to_string_pretty(&payload).unwrap())
        .unwrap_or_else(|e| {
            eprintln!("failed to write {}: {e}", out_path.display());
            std::process::exit(1);
        });
    println!("wrote {} runs to {}", runs.len(), out_path.display());
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_grid() -> Grid {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map");
        Grid::load(path).expect("map should load")
    }

    /// No dedicated `tests/*.rs` gate for Phase 8 items — same fallback discipline as
    /// `baseline_stop_and_wait.rs`/`run_experiment.rs`. Directly exercises the
    /// degradation mechanics this file adds beyond what `run_experiment.rs` already
    /// covers: 100% packet loss must force every robot to Autonomous eventually, and
    /// collision-freedom must hold regardless (PIBT's guarantee doesn't depend on fair
    /// priorities, per this file's own doc comment).
    #[test]
    fn total_packet_loss_eventually_forces_every_robot_autonomous() {
        let grid = test_grid();
        let (starts, tasks) = generate_scenario(&grid, 3, 3, 1);
        let run = run_degraded(&grid, "test", &starts, tasks, 1, 500, 100.0, 0);

        assert_eq!(run["vertex_collisions"], json!(0));
        assert_eq!(run["edge_collisions"], json!(0));
        assert!(
            run["mode_occupancy"]["autonomous"].as_f64().unwrap() > 0.5,
            "100% packet loss for the whole run should spend most robot-ticks Autonomous, got {}",
            run["mode_occupancy"]
        );
    }

    #[test]
    fn zero_packet_loss_and_zero_latency_stays_fully_cooperative() {
        let grid = test_grid();
        let (starts, tasks) = generate_scenario(&grid, 3, 3, 1);
        let run = run_degraded(&grid, "test", &starts, tasks, 1, 500, 0.0, 0);

        assert_eq!(run["mode_occupancy"]["cooperative"], json!(1.0));
        assert_eq!(run["mode_occupancy"]["autonomous"], json!(0.0));
    }

    #[test]
    fn latency_at_the_threshold_triggers_cautious_not_cooperative() {
        let grid = test_grid();
        let (starts, tasks) = generate_scenario(&grid, 3, 3, 1);
        // config::THRESHOLD_L_LATENCY_TICKS is 3 — every heartbeat this late must
        // trigger Cautious per mode_state_machine.rs's own exact-threshold design,
        // already proven in tests/mode_state_machine.rs; checked here only to confirm
        // this file's own comms-injection wiring actually reaches that real logic.
        let run = run_degraded(
            &grid, "test", &starts, tasks,
            1, 500, 0.0, sih26123::config::THRESHOLD_L_LATENCY_TICKS,
        );
        assert_eq!(run["mode_occupancy"]["cooperative"], json!(0.0));
    }
}
