//! Item 27 of `docs/BUILD_PLAN.md`, Phase 8's simplest comparison baseline: "stop and
//! wait" collision avoidance, the PS's own named comparison point (§1.4's hard "≥20%
//! reduction vs. stop-and-wait" criterion). No priority inheritance, no yielding, no
//! rerouting — each robot greedily steps toward its goal along the shortest BFS path
//! and, if its one chosen next cell is occupied this tick, simply waits. This is what
//! actually differentiates PIBT: stop-and-wait can genuinely deadlock (two robots
//! head-on in a corridor, each waiting forever for the other to move first), which
//! `deadlocks_detected` below measures directly rather than asserting.
//!
//! In-process, deterministic, tick-counted simulation (Decision 15, `docs/decisions.md`)
//! — no sockets, no wall-clock timing anywhere in the measurement, so many seeds/maps
//! can run fast and the comparison against `pibt_token` isn't contaminated by real
//! network jitter. Task allocation reuses `TaskLayer::handle_token` (`new_pure`,
//! Decision 15) unchanged from the real system — only the *movement* algorithm differs
//! here, which is the actual thing Phase 8 is supposed to be comparing.
//!
//! Standalone and independently runnable (`cargo run --release --bin
//! baseline_stop_and_wait -- --map ... --start ... --task ... --seed ... --tick-limit
//! ...`), and also what `run_experiment.rs` (item 28) invokes as a subprocess with a
//! matching scenario to get a directly comparable `pibt_token` vs. `stop_and_wait` run.
//! Well-formedness of the given scenario (Decision 5 — distinct non-task start
//! positions, etc.) is the *caller's* responsibility, same as `main.rs`'s `robot`
//! subcommand: this binary faithfully simulates whatever it's handed.
//!
//! Prints exactly one `harness/metrics_schema.json`-shaped `run` object as JSON to
//! stdout, `"system": "stop_and_wait"`.

use std::collections::{HashMap, HashSet};

use clap::Parser;
use rand::rngs::StdRng;
use rand::seq::SliceRandom;
use rand::SeedableRng;
use serde::Serialize;

use sih26123::robot::planner_pibt::Cell;
use sih26123::robot::task_layer::{Task, TaskLayer, TaskState};
use sih26123::world::grid::Grid;

/// How many consecutive ticks a robot can be genuinely blocked (has a goal, has a
/// strictly-distance-reducing candidate move, but that cell stayed occupied) before this
/// counts as one deadlock event, not transient contention from an ordinary crossing.
/// Deliberately not a `config.rs` constant: this is baseline-specific, not a property of
/// the deployed system `config.rs` otherwise describes.
const DEADLOCK_DETECTION_TICKS: u32 = 20;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    map: String,
    /// This scenario's robot starting cells, "x,y", one per robot, in robot-id order
    /// (ids are 1-based, assigned by position in this list).
    #[arg(long = "start", value_delimiter = ' ')]
    starts: Vec<String>,
    /// Shared task pool, "pickup_x,pickup_y,dropoff_x,dropoff_y", repeatable.
    #[arg(long = "task")]
    tasks: Vec<String>,
    #[arg(long)]
    seed: u64,
    #[arg(long)]
    tick_limit: u64,
    /// Identifies this run in the output JSON's `run_id` field; defaults to a
    /// map+seed-derived id if omitted.
    #[arg(long)]
    run_id: Option<String>,
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

#[derive(Serialize)]
struct PerRobot {
    robot_id: u32,
    distance: u32,
    wait_ticks: u32,
    tasks_completed: u32,
}

#[derive(Serialize)]
struct RunMetrics {
    run_id: String,
    map: String,
    seed: u64,
    system: &'static str,
    num_robots: usize,
    tick_limit: u64,
    tasks_total: usize,
    tasks_completed: usize,
    finished_within_limit: bool,
    completion_ticks: Option<u64>,
    vertex_collisions: u32,
    edge_collisions: u32,
    deadlocks_detected: u32,
    total_distance: u32,
    total_wait_ticks: u32,
    claim_conflicts: u32,
    token_skips: u32,
    comparable_to_baseline: bool,
    improvement_pct: Option<f64>,
    per_robot: Vec<PerRobot>,
}

struct RobotRuntime {
    id: u32,
    pos: Cell,
    task_layer: TaskLayer,
    dist_table: Option<(Cell, HashMap<Cell, u32>)>,
    distance: u32,
    wait_ticks: u32,
    consecutive_blocked_ticks: u32,
    tasks_completed: u32,
}

/// The pure simulation core — no I/O, directly unit-testable, same discipline every
/// other pure-core/thin-wrapper module in this project uses. Runs the given scenario for
/// up to `tick_limit` ticks (one token-ring hop per tick, same cadence a caller building
/// the `pibt_token` comparison run should use for a fair comparison), stopping early once
/// every task completes.
fn run_stop_and_wait(
    grid: &Grid,
    starts: &[Cell],
    tasks: Vec<Task>,
    seed: u64,
    tick_limit: u64,
) -> RunMetrics {
    let peer_ids: Vec<u32> = (1..=starts.len() as u32).collect();
    let mut robots: Vec<RobotRuntime> = starts
        .iter()
        .zip(&peer_ids)
        .map(|(&pos, &id)| RobotRuntime {
            id,
            pos,
            task_layer: TaskLayer::new_pure(id, grid, tasks.clone(), peer_ids.clone()),
            dist_table: None,
            distance: 0,
            wait_ticks: 0,
            consecutive_blocked_ticks: 0,
            tasks_completed: 0,
        })
        .collect();

    let tasks_total = tasks.len();
    let mut rng = StdRng::seed_from_u64(seed);
    let mut token = sih26123::protocol::messages::TokenMsg {
        seq: 0,
        holder_id: peer_ids[0],
        claimed_tasks: vec![],
        epoch: 0,
        creator: 0,
    };
    let mut deadlocks_detected = 0u32;
    let mut vertex_collisions = 0u32;
    let mut edge_collisions = 0u32;
    let mut completion_ticks: Option<u64> = None;
    let mut prev_positions: Vec<Cell> = robots.iter().map(|r| r.pos).collect();

    for tick in 0..tick_limit {
        // One token-ring hop per tick — deliberately the same cadence a `pibt_token`
        // in-process comparison run should use, so task-allocation timing isn't itself
        // a confound in the movement-algorithm comparison this baseline exists for.
        let holder_idx = robots.iter().position(|r| r.id == token.holder_id)
            .expect("token holder_id always names a robot in peer_ids");
        let holder_pos = robots[holder_idx].pos;
        token = robots[holder_idx].task_layer.handle_token(&token, holder_pos);

        let mut order: Vec<usize> = (0..robots.len()).collect();
        order.shuffle(&mut rng);

        let mut occupied: HashSet<Cell> = robots.iter().map(|r| r.pos).collect();

        for idx in order {
            // Process arrival at the *current* waypoint before deciding whether to
            // move this tick — a task claimed this same tick whose pickup or dropoff
            // happens to equal the robot's already-current position (e.g. an idle
            // robot standing exactly on a task's pickup cell when the token reaches
            // it) would otherwise never advance: `on_position_update` only fires after
            // a successful move below, and such a robot never needs to move at all.
            let my_pos = robots[idx].pos;
            let before = robots[idx].task_layer.state();
            robots[idx].task_layer.on_position_update(my_pos);
            let after = robots[idx].task_layer.state();
            if matches!(before, TaskState::ToDropoff(_)) && after == TaskState::Idle {
                robots[idx].tasks_completed += 1;
            }

            let Some(goal) = robots[idx].task_layer.current_goal() else {
                robots[idx].wait_ticks += 1;
                continue;
            };
            if robots[idx].pos == goal {
                continue;
            }

            if robots[idx].dist_table.as_ref().map(|(g, _)| *g) != Some(goal) {
                robots[idx].dist_table = Some((goal, grid.bfs_distance(goal)));
            }
            let (_, dist_table) = robots[idx].dist_table.as_ref().unwrap();
            let my_dist = dist_table.get(&my_pos).copied();

            let best = grid
                .neighbors(my_pos.0, my_pos.1)
                .into_iter()
                .filter_map(|c| dist_table.get(&c).map(|&d| (c, d)))
                .filter(|&(_, d)| Some(d) < my_dist)
                .min_by_key(|&(_, d)| d);

            match best {
                Some((next, _)) if !occupied.contains(&next) => {
                    occupied.remove(&my_pos);
                    occupied.insert(next);
                    robots[idx].pos = next;
                    robots[idx].distance += 1;
                    robots[idx].consecutive_blocked_ticks = 0;

                    let before = robots[idx].task_layer.state();
                    robots[idx].task_layer.on_position_update(next);
                    let after = robots[idx].task_layer.state();
                    if matches!(before, TaskState::ToDropoff(_)) && after == TaskState::Idle {
                        robots[idx].tasks_completed += 1;
                    }
                }
                Some(_) => {
                    robots[idx].wait_ticks += 1;
                    robots[idx].consecutive_blocked_ticks += 1;
                    if robots[idx].consecutive_blocked_ticks == DEADLOCK_DETECTION_TICKS {
                        deadlocks_detected += 1;
                    }
                }
                None => {
                    // No candidate strictly reduces distance (goal unreachable from
                    // here, or already adjacent-optimal) — waits, doesn't count as
                    // "blocked" toward deadlock detection since there's genuinely
                    // nothing better to try, not a contested cell.
                    robots[idx].wait_ticks += 1;
                }
            }
        }

        // Collision bookkeeping: the sequential occupied-set check above never lets a
        // robot move into a still-occupied cell, so both should stay 0 by construction
        // — checked directly anyway rather than assumed, same discipline
        // `has_no_collisions` embodies for the real system.
        let now_positions: Vec<Cell> = robots.iter().map(|r| r.pos).collect();
        let mut seen = HashSet::with_capacity(now_positions.len());
        for &p in &now_positions {
            if !seen.insert(p) {
                vertex_collisions += 1;
            }
        }
        for i in 0..robots.len() {
            for j in (i + 1)..robots.len() {
                if now_positions[i] == prev_positions[j] && now_positions[j] == prev_positions[i] {
                    edge_collisions += 1;
                }
            }
        }
        prev_positions = now_positions;

        let tasks_completed: usize = robots.iter().map(|r| r.tasks_completed as usize).sum();
        if tasks_completed >= tasks_total {
            completion_ticks = Some(tick + 1);
            break;
        }
    }

    let tasks_completed: usize = robots.iter().map(|r| r.tasks_completed as usize).sum();
    let finished_within_limit = completion_ticks.is_some();
    let total_distance = robots.iter().map(|r| r.distance).sum();
    let total_wait_ticks = robots.iter().map(|r| r.wait_ticks).sum();
    let claim_conflicts = robots.iter().map(|r| r.task_layer.claim_conflicts()).sum();
    let token_skips = robots.iter().map(|r| r.task_layer.skip_count()).sum();

    RunMetrics {
        run_id: String::new(), // filled in by main() once the map name is known
        map: String::new(),
        seed,
        system: "stop_and_wait",
        num_robots: robots.len(),
        tick_limit,
        tasks_total,
        tasks_completed,
        finished_within_limit,
        completion_ticks,
        vertex_collisions,
        edge_collisions,
        deadlocks_detected,
        total_distance,
        total_wait_ticks,
        claim_conflicts,
        token_skips,
        comparable_to_baseline: finished_within_limit,
        improvement_pct: None, // stop_and_wait is the baseline itself; only pibt_token computes this
        per_robot: robots
            .iter()
            .map(|r| PerRobot {
                robot_id: r.id,
                distance: r.distance,
                wait_ticks: r.wait_ticks,
                tasks_completed: r.tasks_completed,
            })
            .collect(),
    }
}

fn main() {
    let args = Args::parse();
    let grid = Grid::load(&args.map).unwrap_or_else(|e| {
        eprintln!("failed to load map \"{}\": {e}", args.map);
        std::process::exit(1);
    });
    let starts: Vec<Cell> = args.starts.iter().map(|s| parse_cell(s)).collect();
    let tasks: Vec<Task> = args
        .tasks
        .iter()
        .enumerate()
        .map(|(i, s)| parse_task((i + 1) as u32, s))
        .collect();

    let map_name = std::path::Path::new(&args.map)
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| args.map.clone());

    let mut metrics = run_stop_and_wait(&grid, &starts, tasks, args.seed, args.tick_limit);
    metrics.map = map_name.clone();
    metrics.run_id = args
        .run_id
        .unwrap_or_else(|| format!("stop_and_wait-{map_name}-seed{}", args.seed));

    println!("{}", serde_json::to_string(&metrics).expect("RunMetrics always serializes"));
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_grid() -> Grid {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map");
        Grid::load(path).expect("map should load")
    }

    /// No dedicated `tests/*.rs` file for this item — Phase 8 has no per-file gate in
    /// `docs/TESTING_PLAN.md` beyond the real benchmark numbers themselves — so the pure
    /// core gets thorough in-file coverage instead, same fallback discipline every other
    /// phase-without-a-gate item in this project has used.
    #[test]
    fn a_single_robot_completes_its_task_and_reaches_the_dropoff() {
        let grid = test_grid();
        let free = grid.free_cells();
        let start = free[0];
        let pickup = free[10];
        let dropoff = free[20];
        let tasks = vec![Task { task_id: 1, pickup, dropoff }];

        let metrics = run_stop_and_wait(&grid, &[start], tasks, 1, 2_000);

        assert!(metrics.finished_within_limit, "a lone robot should always finish");
        assert_eq!(metrics.tasks_completed, 1);
        assert_eq!(metrics.vertex_collisions, 0);
        assert_eq!(metrics.edge_collisions, 0);
        assert_eq!(metrics.deadlocks_detected, 0, "no peers, nothing to deadlock against");
    }

    /// The defining property this baseline exists to demonstrate: with no alternate
    /// route and no negotiation mechanism at all, a robot permanently blocked by
    /// another (even one that isn't actively contesting anything, just idle in the
    /// only path) never gets unstuck — unlike `pibt_token`, which resolves exactly this
    /// via priority inheritance. A single-file corridor with the blocking robot given
    /// no task keeps this deterministic without depending on the task pool's own
    /// "nearest first" tie-break (which would otherwise make constructing a guaranteed
    /// standoff surprisingly fiddly — a task whose pickup happens to coincide with a
    /// robot's own current position gets claimed instantly, so both tasks need to sit
    /// where neither robot already is).
    #[test]
    fn a_permanently_blocked_robot_never_finishes() {
        // A 1-wide, 3-tall vertical corridor with no side exits: robot 2 parks directly
        // in robot 1's only path and is given no task, so it never moves again.
        let map = "type octile\nheight 3\nwidth 1\nmap\n.\n.\n.\n";
        let path = std::env::temp_dir().join("baseline_stop_and_wait_corridor_test.map");
        std::fs::write(&path, map).expect("write test map");
        let grid = Grid::load(&path).expect("map should load");

        let starts = vec![(0, 0), (0, 1)];
        let tasks = vec![Task { task_id: 1, pickup: (0, 2), dropoff: (0, 2) }];

        let metrics = run_stop_and_wait(&grid, &starts, tasks, 7, 200);

        assert!(!metrics.finished_within_limit, "a permanently blocked robot must never finish");
        assert!(metrics.deadlocks_detected > 0, "expected at least one detected deadlock");
        assert_eq!(metrics.vertex_collisions, 0, "sequential move validation must prevent this");
        assert_eq!(metrics.edge_collisions, 0, "sequential move validation must prevent this");
        assert!(!metrics.comparable_to_baseline);
        assert!(metrics.improvement_pct.is_none());

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn an_unreachable_scenario_never_fabricates_completion() {
        let grid = test_grid();
        let free = grid.free_cells();
        let tasks = vec![Task { task_id: 1, pickup: free[5], dropoff: free[6] }];

        // tick_limit of 0: the run can't possibly finish; must report honestly.
        let metrics = run_stop_and_wait(&grid, &[free[0]], tasks, 1, 0);

        assert!(!metrics.finished_within_limit);
        assert_eq!(metrics.completion_ticks, None);
        assert!(!metrics.comparable_to_baseline);
        assert_eq!(metrics.improvement_pct, None);
    }
}
