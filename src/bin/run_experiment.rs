//! Item 28 of `docs/BUILD_PLAN.md`: seeded benchmark runs, both `pibt_token` (our
//! system) and `stop_and_wait` (the baseline), on the same map/scenario/tick budget, so
//! `improvement_pct` is a real apples-to-apples comparison rather than two independently
//! generated numbers. In-process, deterministic, tick-counted (Decision 15,
//! `docs/decisions.md`) — `pibt_token` reuses `Pibt::step`/`Pibt::retarget` and
//! `TaskLayer::handle_token` (`new_pure`) directly; `stop_and_wait` is invoked as a
//! subprocess of the already-built, independently-verified `baseline_stop_and_wait`
//! binary (`CARGO_BIN_EXE_baseline_stop_and_wait`) rather than duplicating its logic.
//!
//! Owns Decision 5's well-formedness enforcement (`docs/decisions.md`): this is the
//! "scenario-generation" concern that decision named as *this* file's job, not
//! `task_layer.rs`'s. `generate_scenario` reserves `num_robots` free cells as start
//! positions and a further `2 * num_tasks` as pickup/dropoff cells, all pairwise
//! disjoint by construction (one shuffled slice of the map's free cells, sliced into
//! non-overlapping chunks) — satisfying (a) a finite task set, (b) at least as many
//! non-task endpoints as robots, trivially; (c) (any two endpoints connected without
//! passing through a third) is already satisfied for free by Decision 1's map choice
//! being proven biconnected, a strictly stronger property.
//!
//! `--force-crossing` biases starts into the map's left half and every task's
//! pickup/dropoff into its right half, so every robot's path likely crosses through the
//! middle where other robots are also crossing — a simple, honest way to get the
//! "overlapping paths" scenarios `docs/TESTING_PLAN.md`'s Phase 8 speedup check
//! specifically asks for (§1.4), without hand-curating exact adversarial geometry per
//! pair (attempted once already for `baseline_stop_and_wait.rs`'s own deadlock test and
//! found genuinely fiddly — see that file's own entry in `docs/FILE_MAP.md`). Off by
//! default for general-coverage runs; on for the runs that actually back the PS's ≥20%
//! claim.
//!
//! Writes `harness/metrics_schema.json`-shaped output (default `results/metrics.json`).

use std::collections::HashSet;
use std::path::PathBuf;
use std::process::Command;

use clap::Parser;
use rand::rngs::StdRng;
use rand::seq::SliceRandom;
use rand::SeedableRng;
use serde_json::{json, Value};

use sih26123::robot::planner_pibt::{has_no_collisions, Cell, Config, Pibt};
use sih26123::robot::task_layer::{Task, TaskLayer, TaskState};
use sih26123::world::grid::Grid;

#[derive(Parser)]
struct Args {
    #[arg(long, default_value = "maps/warehouse-10-20-10-2-1.map")]
    map: String,
    #[arg(long, default_value_t = 3)]
    num_robots: usize,
    #[arg(long, default_value_t = 5)]
    num_tasks: usize,
    /// One run per seed, comma-separated — "multiple seeds"
    /// (`docs/PS_AND_ARCHITECTURE.md` §3.6) is a real requirement, not a nicety.
    #[arg(long, value_delimiter = ',', default_value = "1,2,3,4,5")]
    seeds: Vec<u64>,
    #[arg(long, default_value_t = 3000)]
    tick_limit: u64,
    #[arg(long)]
    force_crossing: bool,
    /// Explicit robot start cells, "x,y", one per robot, in robot-id order — bypasses
    /// `generate_scenario` entirely when given (along with `--task`), for reproducing a
    /// specific hand-picked scenario (e.g. a known-dense one) rather than a random
    /// well-formed one. Only meaningful with exactly one `--seeds` value, since the
    /// scenario itself is then fixed regardless of seed.
    #[arg(long = "explicit-start", value_delimiter = ' ')]
    explicit_starts: Vec<String>,
    /// Explicit task pool, "pickup_x,pickup_y,dropoff_x,dropoff_y", repeatable — see
    /// `--explicit-start`.
    #[arg(long = "explicit-task")]
    explicit_tasks: Vec<String>,
    #[arg(long, default_value = "results/metrics.json")]
    out: String,
    /// Path to the `baseline_stop_and_wait` binary. `CARGO_BIN_EXE_*` env vars are only
    /// set by Cargo for `tests/*.rs` integration binaries, not for another `src/bin/`
    /// target — Cargo places sibling binaries in the same directory as this one's own
    /// executable, so that's the default, overridable for an unusual layout.
    #[arg(long)]
    stop_and_wait_bin: Option<String>,
}

fn parse_cell(s: &str) -> Cell {
    let mut parts = s.split(',');
    let x: usize = parts.next().expect("missing x").trim().parse().expect("bad x");
    let y: usize = parts.next().expect("missing y").trim().parse().expect("bad y");
    (x, y)
}

fn parse_task(task_id: u32, s: &str) -> Task {
    let parts: Vec<&str> = s.split(',').collect();
    assert_eq!(parts.len(), 4, "task must be pickup_x,pickup_y,dropoff_x,dropoff_y");
    Task {
        task_id,
        pickup: parse_cell(&format!("{},{}", parts[0], parts[1])),
        dropoff: parse_cell(&format!("{},{}", parts[2], parts[3])),
    }
}

/// Reserves `num_robots + 2 * num_tasks` pairwise-disjoint free cells for a well-formed
/// scenario (Decision 5). `force_crossing` splits the map into left/right halves by `x`
/// and draws starts only from the left, task pickups/dropoffs only from the right —
/// falls back to `false`'s uniform draw if either half doesn't have enough free cells on
/// its own (a small/oddly-shaped map shouldn't hard-fail a run over this).
fn generate_scenario(
    grid: &Grid,
    num_robots: usize,
    num_tasks: usize,
    seed: u64,
    force_crossing: bool,
) -> (Vec<Cell>, Vec<Task>) {
    let mut rng = StdRng::seed_from_u64(seed);
    let all_free = grid.free_cells();

    let (mut start_pool, mut task_pool) = if force_crossing {
        let mid = grid.width() / 2;
        let left: Vec<Cell> = all_free.iter().copied().filter(|&(x, _)| x < mid).collect();
        let right: Vec<Cell> = all_free.iter().copied().filter(|&(x, _)| x >= mid).collect();
        if left.len() >= num_robots && right.len() >= 2 * num_tasks {
            (left, right)
        } else {
            (all_free.clone(), all_free.clone())
        }
    } else {
        (all_free.clone(), all_free.clone())
    };
    start_pool.shuffle(&mut rng);
    task_pool.shuffle(&mut rng);

    assert!(
        start_pool.len() >= num_robots,
        "map doesn't have enough free cells for {num_robots} robot start positions"
    );
    let starts: Vec<Cell> = start_pool[..num_robots].to_vec();
    let taken: HashSet<Cell> = starts.iter().copied().collect();

    // Draw pickup/dropoff cells from `task_pool`, skipping anything already used as a
    // start (relevant when `force_crossing` fell back to the shared uniform pool, or
    // when a start and a task cell happen to coincide even across disjoint halves at
    // the boundary column) — Decision 5(b) requires these sets to be disjoint outright.
    let mut task_cells = task_pool.into_iter().filter(|c| !taken.contains(c));
    let mut tasks = Vec::with_capacity(num_tasks);
    for task_id in 1..=num_tasks as u32 {
        let pickup = task_cells
            .next()
            .expect("map doesn't have enough free cells left for this many tasks");
        let dropoff = task_cells
            .next()
            .expect("map doesn't have enough free cells left for this many tasks");
        tasks.push(Task { task_id, pickup, dropoff });
    }
    (starts, tasks)
}

struct PibtRobot {
    id: u32,
    task_layer: TaskLayer,
    goal: Cell,
    priority: f64,
    distance: u32,
    wait_ticks: u32,
    tasks_completed: u32,
}

/// The in-process `pibt_token` benchmark run: one token-ring hop per tick (the same
/// cadence `baseline_stop_and_wait.rs` uses, so task-allocation timing isn't itself a
/// confound in the movement-algorithm comparison), driving `Pibt::step` tick-by-tick
/// with each robot's goal kept in sync with its `TaskLayer::current_goal()` via
/// `Pibt::retarget` whenever it changes. An idle robot's "goal" is wherever it currently
/// stands — it doesn't want to move, but (unlike `stop_and_wait`'s idle robots, which
/// never move regardless) `Pibt`'s own negotiation can still displace it if a busy
/// neighbor needs its cell, a real behavioral difference between the two systems.
fn run_pibt_token(
    grid: &Grid,
    map_name: &str,
    starts: &[Cell],
    tasks: Vec<Task>,
    seed: u64,
    tick_limit: u64,
) -> Value {
    let n = starts.len();
    let peer_ids: Vec<u32> = (1..=n as u32).collect();
    let mut pibt = Pibt::new(grid, starts.to_vec(), starts.to_vec(), seed);
    let mut robots: Vec<PibtRobot> = starts
        .iter()
        .zip(&peer_ids)
        .map(|(&pos, &id)| PibtRobot {
            id,
            task_layer: TaskLayer::new_pure(id, grid, tasks.clone(), peer_ids.clone()),
            goal: pos,
            priority: 0.0,
            distance: 0,
            wait_ticks: 0,
            tasks_completed: 0,
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

        for i in 0..n {
            let desired = robots[i].task_layer.current_goal().unwrap_or(positions[i]);
            if desired != robots[i].goal {
                pibt.retarget(i, desired);
                robots[i].goal = desired;
            }
        }

        let priorities: Vec<f64> = robots.iter().map(|r| r.priority).collect();
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

    // The PS's hard requirement (§1.4), checked for real on every run rather than
    // assumed from the proof — reuses the exact checker item 10's own test suite
    // already proved correct, not a second implementation of collision logic.
    let collision_free = has_no_collisions(&configs);
    let tasks_completed: usize = robots.iter().map(|r| r.tasks_completed as usize).sum();
    let finished_within_limit = completion_ticks.is_some();

    json!({
        "run_id": format!("pibt_token-seed{seed}"),
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
        "comparable_to_baseline": finished_within_limit,
        "improvement_pct": Value::Null,
        "mode_occupancy": { "cooperative": 1.0, "cautious": 0.0, "autonomous": 0.0 },
        "per_robot": robots.iter().map(|r| json!({
            "robot_id": r.id,
            "distance": r.distance,
            "wait_ticks": r.wait_ticks,
            "tasks_completed": r.tasks_completed,
        })).collect::<Vec<_>>(),
    })
}

fn run_stop_and_wait(
    bin_path: &str,
    map: &str,
    starts: &[Cell],
    tasks: &[Task],
    seed: u64,
    tick_limit: u64,
) -> Value {
    let mut cmd = Command::new(bin_path);
    cmd.arg("--map").arg(map);
    for &(x, y) in starts {
        cmd.arg("--start").arg(format!("{x},{y}"));
    }
    for t in tasks {
        cmd.arg("--task").arg(format!(
            "{},{},{},{}",
            t.pickup.0, t.pickup.1, t.dropoff.0, t.dropoff.1
        ));
    }
    cmd.arg("--seed").arg(seed.to_string());
    cmd.arg("--tick-limit").arg(tick_limit.to_string());

    let output = cmd.output().unwrap_or_else(|e| {
        eprintln!("failed to run baseline_stop_and_wait: {e}");
        std::process::exit(1);
    });
    if !output.status.success() {
        eprintln!(
            "baseline_stop_and_wait exited with {}: {}",
            output.status,
            String::from_utf8_lossy(&output.stderr)
        );
        std::process::exit(1);
    }
    serde_json::from_slice(&output.stdout).unwrap_or_else(|e| {
        eprintln!("failed to parse baseline_stop_and_wait output: {e}");
        std::process::exit(1);
    })
}

/// Cross-fills `comparable_to_baseline`/`improvement_pct` on both entries once both
/// systems' results for the same seed are known — Decision 4's "no invented percentage"
/// discipline: a run that didn't finish inside `tick_limit` (either side) can never
/// produce a fabricated speedup number.
fn cross_compare(pibt_token: &mut Value, stop_and_wait: &mut Value) {
    let pibt_finished = pibt_token["finished_within_limit"].as_bool().unwrap_or(false);
    let saw_finished = stop_and_wait["finished_within_limit"].as_bool().unwrap_or(false);
    let both_finished = pibt_finished && saw_finished;

    pibt_token["comparable_to_baseline"] = json!(both_finished);
    stop_and_wait["comparable_to_baseline"] = json!(both_finished);

    if both_finished {
        let pibt_ticks = pibt_token["completion_ticks"].as_f64().unwrap();
        let saw_ticks = stop_and_wait["completion_ticks"].as_f64().unwrap();
        let improvement = (saw_ticks - pibt_ticks) / saw_ticks * 100.0;
        pibt_token["improvement_pct"] = json!(improvement);
    } else {
        pibt_token["improvement_pct"] = Value::Null;
    }
    stop_and_wait["improvement_pct"] = Value::Null; // the baseline is what pibt_token is measured against, not the reverse
}

fn main() {
    let args = Args::parse();
    let grid = Grid::load(&args.map).unwrap_or_else(|e| {
        eprintln!("failed to load map \"{}\": {e}", args.map);
        std::process::exit(1);
    });

    let stop_and_wait_bin = args.stop_and_wait_bin.clone().unwrap_or_else(|| {
        let mut path = std::env::current_exe().unwrap_or_else(|e| {
            eprintln!("failed to locate this binary's own path: {e}");
            std::process::exit(1);
        });
        path.set_file_name("baseline_stop_and_wait");
        path.to_string_lossy().into_owned()
    });

    let map_name = std::path::Path::new(&args.map)
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| args.map.clone());

    let mut runs: Vec<Value> = Vec::new();
    for &seed in &args.seeds {
        let (starts, tasks) = if !args.explicit_starts.is_empty() {
            let starts: Vec<Cell> = args.explicit_starts.iter().map(|s| parse_cell(s)).collect();
            let tasks: Vec<Task> = args
                .explicit_tasks
                .iter()
                .enumerate()
                .map(|(i, s)| parse_task((i + 1) as u32, s))
                .collect();
            (starts, tasks)
        } else {
            generate_scenario(&grid, args.num_robots, args.num_tasks, seed, args.force_crossing)
        };

        let mut pibt_result =
            run_pibt_token(&grid, &map_name, &starts, tasks.clone(), seed, args.tick_limit);
        let mut saw_result =
            run_stop_and_wait(&stop_and_wait_bin, &args.map, &starts, &tasks, seed, args.tick_limit);
        cross_compare(&mut pibt_result, &mut saw_result);

        eprintln!(
            "[run_experiment] seed {seed}: pibt_token completed_ticks={:?} stop_and_wait completed_ticks={:?} improvement_pct={:?}",
            pibt_result["completion_ticks"], saw_result["completion_ticks"], pibt_result["improvement_pct"]
        );

        runs.push(pibt_result);
        runs.push(saw_result);
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

    /// No dedicated `tests/*.rs` file for this item (same Phase-8-has-no-per-file-gate
    /// reasoning as `baseline_stop_and_wait.rs`) — Decision 5's well-formedness
    /// enforcement is the one piece of real, non-obvious logic in this file's own scope
    /// (the two simulation runners are exercised for real by the manual verification
    /// noted in `docs/FILE_MAP.md`), so it gets direct in-file coverage.
    #[test]
    fn generated_scenario_is_well_formed_per_decision_5() {
        let grid = test_grid();
        let (starts, tasks) = generate_scenario(&grid, 4, 6, 42, false);

        assert_eq!(starts.len(), 4);
        assert_eq!(tasks.len(), 6);

        let start_set: HashSet<Cell> = starts.iter().copied().collect();
        assert_eq!(start_set.len(), starts.len(), "starts must be pairwise distinct");

        for t in &tasks {
            assert!(
                !start_set.contains(&t.pickup) && !start_set.contains(&t.dropoff),
                "Decision 5(b): a task endpoint must never coincide with a robot start"
            );
        }

        let mut all_cells: Vec<Cell> = starts.clone();
        for t in &tasks {
            all_cells.push(t.pickup);
            all_cells.push(t.dropoff);
        }
        let all_set: HashSet<Cell> = all_cells.iter().copied().collect();
        assert_eq!(all_set.len(), all_cells.len(), "every reserved cell must be pairwise distinct");

        for &c in &all_cells {
            assert!(grid.is_free(c.0, c.1), "every reserved cell must actually be free");
        }
    }

    #[test]
    fn force_crossing_keeps_starts_and_tasks_on_opposite_halves() {
        let grid = test_grid();
        let mid = grid.width() / 2;
        let (starts, tasks) = generate_scenario(&grid, 3, 3, 7, true);

        for &(x, _) in &starts {
            assert!(x < mid, "force_crossing starts must land in the left half");
        }
        for t in &tasks {
            assert!(t.pickup.0 >= mid && t.dropoff.0 >= mid, "force_crossing tasks must land in the right half");
        }
    }

    #[test]
    fn cross_compare_never_invents_a_percentage_when_either_side_fails() {
        let mut pibt_token = json!({
            "finished_within_limit": true,
            "completion_ticks": 100,
        });
        let mut stop_and_wait = json!({
            "finished_within_limit": false,
            "completion_ticks": Value::Null,
        });
        cross_compare(&mut pibt_token, &mut stop_and_wait);

        assert_eq!(pibt_token["comparable_to_baseline"], json!(false));
        assert_eq!(pibt_token["improvement_pct"], Value::Null);
        assert_eq!(stop_and_wait["comparable_to_baseline"], json!(false));
    }

    #[test]
    fn cross_compare_computes_a_real_percentage_when_both_finish() {
        let mut pibt_token = json!({
            "finished_within_limit": true,
            "completion_ticks": 80,
        });
        let mut stop_and_wait = json!({
            "finished_within_limit": true,
            "completion_ticks": 100,
        });
        cross_compare(&mut pibt_token, &mut stop_and_wait);

        assert_eq!(pibt_token["comparable_to_baseline"], json!(true));
        assert_eq!(pibt_token["improvement_pct"], json!(20.0));
    }
}
