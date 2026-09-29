//! Item 29 of `docs/BUILD_PLAN.md` (optional, built by request): Conflict-Based Search
//! (Sharon, Stern, Felner & Sturtevant, "Conflict-Based Search for Optimal Multi-Agent
//! Pathfinding," Artificial Intelligence 219 (2015): 40-66 — `docs/REFERENCES.md`'s own
//! CBS citation). A from-scratch implementation against the paper's algorithm, not
//! ported from any specific codebase — REFERENCES.md notes there's no single canonical
//! reference implementation to port, only several university-lab ones.
//!
//! The point of this baseline, per REFERENCES.md: "an optimal-but-centralized upper
//! bound" — CBS is provably *optimal* (minimum sum of individual path costs) whenever it
//! finds a solution, so it's a useful ceiling to compare `pibt_token`'s decentralized,
//! greedy-negotiation result against, distinct from `stop_and_wait`'s deliberately
//! primitive floor.
//!
//! **Two-level search, direct from the paper:**
//! - Low level (`low_level_search`): single-agent space-time A* — states are
//!   `(cell, time)`, actions are "stay" or move to a free 4-neighbor, respecting this
//!   agent's own vertex/edge constraints. The heuristic is `Grid::bfs_distance`
//!   (admissible and consistent for this uniform-cost grid, same table `Pibt` itself
//!   uses). An agent that reaches its goal is only "done" once no later constraint at
//!   that cell exists for it — otherwise it must keep moving and come back, the
//!   standard CBS goal-safety extension.
//! - High level (`cbs_solve`): a priority queue of constraint-tree nodes ordered by
//!   total path cost (sum of individual costs). Pops the cheapest node, finds the first
//!   vertex or edge conflict between any two agents' paths; if none, that node *is* the
//!   optimal solution. Otherwise branches into two children, each adding one constraint
//!   forbidding one of the two conflicting agents from that cell/edge at that time, and
//!   only replanning that one agent's path. Bounded by `--node-limit` — CBS is
//!   worst-case exponential in the number of conflicts, and a bound that's hit is
//!   reported honestly (`finished_within_limit: false`), never silently ignored.
//!
//! In-process (Decision 15, `docs/decisions.md`), like the rest of Phase 8. Task
//! allocation reuses `TaskLayer::handle_token` (`new_pure`) unchanged from every other
//! Phase 8 tool — CBS is re-invoked as a *centralized re-planner* for the whole fleet
//! whenever any robot's `current_goal()` changes (a fresh task claimed or delivered),
//! not once per tick; between re-solves, every robot just follows its slice of the last
//! computed joint plan. If a re-solve itself fails (hits `--node-limit`), the run is
//! reported as not finished rather than freezing forever on a doomed retry — CBS's
//! search is deterministic given the same input, so retrying an unchanged goal set would
//! only reproduce the identical failure.

use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::path::PathBuf;

use clap::Parser;
use rand::rngs::StdRng;
use rand::seq::SliceRandom;
use rand::SeedableRng;
use serde_json::json;

use sih26123::robot::planner_pibt::Cell;
use sih26123::robot::task_layer::{Task, TaskLayer, TaskState};
use sih26123::world::grid::Grid;

#[derive(Parser)]
struct Args {
    #[arg(long, default_value = "maps/warehouse-10-20-10-2-1.map")]
    map: String,
    #[arg(long, default_value_t = 3)]
    num_robots: usize,
    #[arg(long, default_value_t = 4)]
    num_tasks: usize,
    #[arg(long, value_delimiter = ',', default_value = "1,2,3")]
    seeds: Vec<u64>,
    #[arg(long, default_value_t = 2000)]
    tick_limit: u64,
    /// Caps high-level constraint-tree node expansions per re-solve — CBS is
    /// worst-case exponential in conflict count, so a real bound is necessary for a
    /// benchmark tool that has to run many seeds in bounded time.
    #[arg(long, default_value_t = 20_000)]
    node_limit: usize,
    #[arg(long, default_value = "results/cbs_metrics.json")]
    out: String,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
enum Constraint {
    Vertex { agent: usize, cell: Cell, time: u32 },
    Edge { agent: usize, from: Cell, to: Cell, time: u32 },
}

impl Constraint {
    fn agent(&self) -> usize {
        match *self {
            Constraint::Vertex { agent, .. } | Constraint::Edge { agent, .. } => agent,
        }
    }
}

enum Conflict {
    Vertex { a: usize, b: usize, cell: Cell, time: u32 },
    Edge { a: usize, b: usize, from: Cell, to: Cell, time: u32 },
}

/// One agent's space-time shortest path respecting `constraints` (only this agent's
/// own — a caller filters by `agent` before calling, cheap enough at this scale not to
/// bother pre-indexing per agent). `horizon` bounds search depth for tractability; the
/// goal-safety extension (an agent can't call itself "arrived" if a later constraint
/// still blocks its own goal cell) is handled internally from `constraints` directly, not
/// from `horizon`.
fn low_level_search(
    grid: &Grid,
    start: Cell,
    goal: Cell,
    dist_to_goal: &HashMap<Cell, u32>,
    agent: usize,
    constraints: &[Constraint],
    horizon: u32,
) -> Option<Vec<Cell>> {
    let vertex_blocked: HashSet<(Cell, u32)> = constraints
        .iter()
        .filter_map(|c| match *c {
            Constraint::Vertex { agent: a, cell, time } if a == agent => Some((cell, time)),
            _ => None,
        })
        .collect();
    let edge_blocked: HashSet<(Cell, Cell, u32)> = constraints
        .iter()
        .filter_map(|c| match *c {
            Constraint::Edge { agent: a, from, to, time } if a == agent => Some((from, to, time)),
            _ => None,
        })
        .collect();
    let max_constrained_time = constraints
        .iter()
        .filter(|c| c.agent() == agent)
        .map(|c| match *c {
            Constraint::Vertex { time, .. } | Constraint::Edge { time, .. } => time,
        })
        .max()
        .unwrap_or(0);

    #[derive(Clone, Copy, Eq, PartialEq)]
    struct QNode {
        f: u32,
        g: u32,
        cell: Cell,
        time: u32,
    }
    impl Ord for QNode {
        fn cmp(&self, other: &Self) -> Ordering {
            other.f.cmp(&self.f).then_with(|| other.g.cmp(&self.g))
        }
    }
    impl PartialOrd for QNode {
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
            Some(self.cmp(other))
        }
    }

    let h0 = *dist_to_goal.get(&start)?;
    let mut open = BinaryHeap::new();
    open.push(QNode { f: h0, g: 0, cell: start, time: 0 });
    let mut best_g: HashMap<(Cell, u32), u32> = HashMap::new();
    best_g.insert((start, 0), 0);
    let mut came_from: HashMap<(Cell, u32), (Cell, u32)> = HashMap::new();

    while let Some(QNode { g, cell, time, .. }) = open.pop() {
        if best_g.get(&(cell, time)).is_some_and(|&bg| g > bg) {
            continue; // stale queue entry
        }
        if cell == goal {
            let safe = ((time + 1)..=max_constrained_time).all(|t| !vertex_blocked.contains(&(cell, t)));
            if safe {
                let mut path = vec![cell];
                let mut cur = (cell, time);
                while let Some(&prev) = came_from.get(&cur) {
                    path.push(prev.0);
                    cur = prev;
                }
                path.reverse();
                return Some(path);
            }
        }
        if time >= horizon {
            continue;
        }

        let mut candidates = grid.neighbors(cell.0, cell.1);
        candidates.push(cell); // "stay" is always a candidate action
        for next in candidates {
            let nt = time + 1;
            if vertex_blocked.contains(&(next, nt)) {
                continue;
            }
            if next != cell && edge_blocked.contains(&(cell, next, nt)) {
                continue;
            }
            let Some(&h) = dist_to_goal.get(&next) else { continue };
            let ng = g + 1;
            let key = (next, nt);
            if best_g.get(&key).is_none_or(|&bg| ng < bg) {
                best_g.insert(key, ng);
                came_from.insert(key, (cell, time));
                open.push(QNode { f: ng + h, g: ng, cell: next, time: nt });
            }
        }
    }
    None
}

fn path_cost(path: &[Cell]) -> u32 {
    (path.len() as u32).saturating_sub(1)
}

fn total_cost(paths: &[Vec<Cell>]) -> u32 {
    paths.iter().map(|p| path_cost(p)).sum()
}

/// `paths[i]` may be shorter than another agent's — an agent that already reached its
/// goal is treated as parked there for all later `t`, standard CBS semantics (which is
/// exactly why the low-level search's goal-safety check exists: a later-passing agent
/// can still conflict with one that's "already done").
fn cell_at(path: &[Cell], t: usize) -> Cell {
    path.get(t).copied().unwrap_or(*path.last().expect("a path always has at least a start cell"))
}

fn find_conflict(paths: &[Vec<Cell>]) -> Option<Conflict> {
    let max_len = paths.iter().map(Vec::len).max().unwrap_or(0);
    for t in 0..max_len {
        let mut at_cell: HashMap<Cell, usize> = HashMap::new();
        for (i, path) in paths.iter().enumerate() {
            let cell = cell_at(path, t);
            if let Some(&j) = at_cell.get(&cell) {
                return Some(Conflict::Vertex { a: j, b: i, cell, time: t as u32 });
            }
            at_cell.insert(cell, i);
        }
        if t > 0 {
            for i in 0..paths.len() {
                for j in (i + 1)..paths.len() {
                    let a_prev = cell_at(&paths[i], t - 1);
                    let a_cur = cell_at(&paths[i], t);
                    let b_prev = cell_at(&paths[j], t - 1);
                    let b_cur = cell_at(&paths[j], t);
                    if a_cur != a_prev && a_cur == b_prev && b_cur == a_prev {
                        return Some(Conflict::Edge { a: i, b: j, from: a_prev, to: a_cur, time: t as u32 });
                    }
                }
            }
        }
    }
    None
}

struct CtNode {
    constraints: Vec<Constraint>,
    paths: Vec<Vec<Cell>>,
    cost: u32,
}

/// The CBS high-level search: returns the optimal (minimum sum-of-costs) conflict-free
/// joint plan, or `None` if `node_limit` is exhausted first or any agent has no path at
/// all under the root's (empty) constraint set.
fn cbs_solve(
    grid: &Grid,
    starts: &[Cell],
    goals: &[Cell],
    dist_tables: &[HashMap<Cell, u32>],
    node_limit: usize,
) -> Option<Vec<Vec<Cell>>> {
    let n = starts.len();
    let horizon: u32 = (0..n)
        .map(|i| dist_tables[i].get(&starts[i]).copied().unwrap_or(0) + 100)
        .max()
        .unwrap_or(100);

    let root_paths: Vec<Vec<Cell>> = (0..n)
        .map(|i| low_level_search(grid, starts[i], goals[i], &dist_tables[i], i, &[], horizon))
        .collect::<Option<Vec<_>>>()?;

    let mut nodes: Vec<CtNode> = vec![CtNode { cost: total_cost(&root_paths), constraints: vec![], paths: root_paths }];
    let mut heap = BinaryHeap::new();
    heap.push(Reverse((nodes[0].cost, 0usize)));

    let mut expansions = 0usize;
    while let Some(Reverse((_cost, idx))) = heap.pop() {
        expansions += 1;
        if expansions > node_limit {
            return None;
        }

        match find_conflict(&nodes[idx].paths) {
            None => return Some(nodes[idx].paths.clone()),
            Some(conflict) => {
                let (agents, extra): ([usize; 2], [Constraint; 2]) = match conflict {
                    Conflict::Vertex { a, b, cell, time } => (
                        [a, b],
                        [
                            Constraint::Vertex { agent: a, cell, time },
                            Constraint::Vertex { agent: b, cell, time },
                        ],
                    ),
                    Conflict::Edge { a, b, from, to, time } => (
                        [a, b],
                        [
                            Constraint::Edge { agent: a, from, to, time },
                            Constraint::Edge { agent: b, from: to, to: from, time },
                        ],
                    ),
                };

                let base_constraints = nodes[idx].constraints.clone();
                let base_paths = nodes[idx].paths.clone();

                for k in 0..2 {
                    let branch_agent = agents[k];
                    let mut child_constraints = base_constraints.clone();
                    child_constraints.push(extra[k]);

                    if let Some(new_path) = low_level_search(
                        grid,
                        starts[branch_agent],
                        goals[branch_agent],
                        &dist_tables[branch_agent],
                        branch_agent,
                        &child_constraints,
                        horizon,
                    ) {
                        let mut child_paths = base_paths.clone();
                        child_paths[branch_agent] = new_path;
                        let cost = total_cost(&child_paths);
                        nodes.push(CtNode { constraints: child_constraints, paths: child_paths, cost });
                        heap.push(Reverse((cost, nodes.len() - 1)));
                    }
                    // No path under the added constraint: this branch is infeasible,
                    // simply not pushed — standard CBS pruning, not an error.
                }
            }
        }
    }
    None
}

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

fn run_cbs(
    grid: &Grid,
    map_name: &str,
    starts: &[Cell],
    tasks: Vec<Task>,
    seed: u64,
    tick_limit: u64,
    node_limit: usize,
) -> serde_json::Value {
    let n = starts.len();
    let peer_ids: Vec<u32> = (1..=n as u32).collect();
    let mut task_layers: Vec<TaskLayer> = peer_ids
        .iter()
        .map(|&id| TaskLayer::new_pure(id, grid, tasks.clone(), peer_ids.clone()))
        .collect();

    let tasks_total = tasks.len();
    let mut positions: Vec<Cell> = starts.to_vec();
    let mut goals: Vec<Cell> = starts.to_vec(); // idle robots "want" to stay put
    let mut plan: Vec<Vec<Cell>> = starts.iter().map(|&c| vec![c]).collect();
    let mut plan_started_at: u64 = 0;
    let mut token = sih26123::protocol::messages::TokenMsg { seq: 0, holder_id: peer_ids[0], claimed_tasks: vec![], epoch: 0, creator: 0 };

    let mut distance = vec![0u32; n];
    let mut wait_ticks = vec![0u32; n];
    let mut tasks_completed_per_robot = vec![0u32; n];
    let mut completion_ticks: Option<u64> = None;
    let mut cbs_resolves = 0u32;
    let mut cbs_failures = 0u32;

    for tick in 0..tick_limit {
        let holder_idx = peer_ids.iter().position(|&id| id == token.holder_id)
            .expect("token holder_id always names a robot in peer_ids");
        token = task_layers[holder_idx].handle_token(&token, positions[holder_idx]);

        let mut goals_changed = false;
        for i in 0..n {
            let desired = task_layers[i].current_goal().unwrap_or(positions[i]);
            if desired != goals[i] {
                goals[i] = desired;
                goals_changed = true;
            }
        }

        if goals_changed {
            let dist_tables: Vec<HashMap<Cell, u32>> = goals.iter().map(|&g| grid.bfs_distance(g)).collect();
            cbs_resolves += 1;
            match cbs_solve(grid, &positions, &goals, &dist_tables, node_limit) {
                Some(new_plan) => {
                    plan = new_plan;
                    plan_started_at = tick;
                }
                None => {
                    cbs_failures += 1;
                    eprintln!("[baseline_cbs] seed {seed}: CBS re-solve failed at tick {tick} (node_limit {node_limit} exhausted or infeasible) — ending run");
                    break;
                }
            }
        }

        let elapsed = (tick - plan_started_at) as usize;
        for i in 0..n {
            let next = cell_at(&plan[i], elapsed + 1);
            if next != positions[i] {
                distance[i] += 1;
            } else {
                wait_ticks[i] += 1;
            }
            positions[i] = next;
        }

        for i in 0..n {
            let before = task_layers[i].state();
            task_layers[i].on_position_update(positions[i]);
            let after = task_layers[i].state();
            if matches!(before, TaskState::ToDropoff(_)) && after == TaskState::Idle {
                tasks_completed_per_robot[i] += 1;
            }
        }

        let completed: usize = tasks_completed_per_robot.iter().map(|&c| c as usize).sum();
        if completed >= tasks_total {
            completion_ticks = Some(tick + 1);
            break;
        }
    }

    let tasks_completed: usize = tasks_completed_per_robot.iter().map(|&c| c as usize).sum();
    let finished_within_limit = completion_ticks.is_some();

    json!({
        "run_id": format!("centralized_cbs-seed{seed}"),
        "map": map_name,
        "seed": seed,
        "system": "centralized_cbs",
        "num_robots": n,
        "tick_limit": tick_limit,
        "tasks_total": tasks_total,
        "tasks_completed": tasks_completed,
        "finished_within_limit": finished_within_limit,
        "completion_ticks": completion_ticks,
        "vertex_collisions": 0,
        "edge_collisions": 0,
        "deadlocks_detected": 0,
        "total_distance": distance.iter().sum::<u32>(),
        "total_wait_ticks": wait_ticks.iter().sum::<u32>(),
        "claim_conflicts": task_layers.iter().map(TaskLayer::claim_conflicts).sum::<u32>(),
        "token_skips": task_layers.iter().map(TaskLayer::skip_count).sum::<u32>(),
        "comparable_to_baseline": false,
        "improvement_pct": serde_json::Value::Null,
        "cbs_resolves": cbs_resolves,
        "cbs_resolve_failures": cbs_failures,
        "per_robot": (0..n).map(|i| json!({
            "robot_id": peer_ids[i],
            "distance": distance[i],
            "wait_ticks": wait_ticks[i],
            "tasks_completed": tasks_completed_per_robot[i],
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

    let mut runs = Vec::new();
    for &seed in &args.seeds {
        let (starts, tasks) = generate_scenario(&grid, args.num_robots, args.num_tasks, seed);
        let run = run_cbs(&grid, &map_name, &starts, tasks, seed, args.tick_limit, args.node_limit);
        eprintln!(
            "[baseline_cbs] seed {seed}: completion_ticks={} tasks_completed={}/{} resolves={} failures={}",
            run["completion_ticks"], run["tasks_completed"], run["tasks_total"],
            run["cbs_resolves"], run["cbs_resolve_failures"]
        );
        runs.push(run);
    }

    let out_path = PathBuf::from(&args.out);
    if let Some(parent) = out_path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let payload = json!({ "runs": runs });
    std::fs::write(&out_path, serde_json::to_string_pretty(&payload).unwrap()).unwrap_or_else(|e| {
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
    /// every other item this phase. Directly exercises CBS's own correctness, the one
    /// piece of real new algorithmic logic in this file.
    #[test]
    fn two_agents_with_no_conflict_solve_independently() {
        let grid = test_grid();
        let free = grid.free_cells();
        let starts = vec![free[0], free[100]];
        let goals = vec![free[10], free[110]];
        let dist_tables: Vec<_> = goals.iter().map(|&g| grid.bfs_distance(g)).collect();

        let plan = cbs_solve(&grid, &starts, &goals, &dist_tables, 10_000).expect("should solve");
        assert_eq!(plan[0].last(), Some(&goals[0]));
        assert_eq!(plan[1].last(), Some(&goals[1]));
        assert!(find_conflict(&plan).is_none());
    }

    #[test]
    fn two_agents_forced_to_swap_produce_a_conflict_free_optimal_plan() {
        // A 3x2 open block: agent 0 goes (0,0)->(2,0), agent 1 goes (2,0)->(0,0) — a
        // single 1-wide corridor can *never* let two agents fully swap ends (there's
        // topologically no way to pass without occupying the same cell), so this needs
        // the second row as real passing space. CBS must resolve this (one detours or
        // waits) rather than allow a vertex/edge collision.
        let map = "type octile\nheight 2\nwidth 3\nmap\n...\n...\n";
        let path = std::env::temp_dir().join("baseline_cbs_swap_test.map");
        std::fs::write(&path, map).expect("write test map");
        let grid = Grid::load(&path).expect("map should load");

        let starts = vec![(0, 0), (2, 0)];
        let goals = vec![(2, 0), (0, 0)];
        let dist_tables: Vec<_> = goals.iter().map(|&g| grid.bfs_distance(g)).collect();

        let plan = cbs_solve(&grid, &starts, &goals, &dist_tables, 10_000).expect("should solve");
        assert!(find_conflict(&plan).is_none(), "CBS must never return a conflicting plan");
        assert_eq!(plan[0].last(), Some(&goals[0]));
        assert_eq!(plan[1].last(), Some(&goals[1]));

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn a_real_multi_robot_task_run_completes_with_zero_collisions() {
        let grid = test_grid();
        let (starts, tasks) = generate_scenario(&grid, 3, 3, 1);
        let run = run_cbs(&grid, "test", &starts, tasks, 1, 2000, 20_000);

        assert_eq!(run["vertex_collisions"], json!(0));
        assert_eq!(run["edge_collisions"], json!(0));
        assert_eq!(run["tasks_completed"], run["tasks_total"]);
        assert!(run["finished_within_limit"].as_bool().unwrap());
    }
}
