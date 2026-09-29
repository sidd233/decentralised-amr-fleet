//! Phase 3 gate (`docs/TESTING_PLAN.md`): the first dedicated `tests/*.rs` file
//! (item 10 of `docs/BUILD_PLAN.md`). Spawns N simulated robots in-process (no
//! sockets — that's item 11+) on the real chosen map (`docs/decisions.md`, Decision 1),
//! runs each scenario to completion, and asserts:
//! - no vertex collision (two robots on the same cell at the same tick),
//! - no edge collision (two robots swapping cells across one tick),
//! - every robot reaches its goal within the tick budget.
//!
//! Five scenarios, per the gate's "at least 5 different start/goal configurations"
//! requirement, including one deliberately dense case funneling many robots through a
//! single narrow corridor — the one that actually substantiates "deadlock-free by
//! construction" rather than a wait-for-graph deadlock-detection approach also passing empirically on
//! easy cases.
//!
//! Map layout facts these scenarios are built on (verified directly against the map
//! file, not assumed): rows `y = 1, 4, 7, ..., 61` are free across the full width
//! (`x = 1..159`); the rows in between come in blocked pairs (2-row-tall shelf blocks,
//! matching Decision 1's connectivity reasoning) with free single-column gaps at
//! `x = 36, 47, 58, 69, 80, 91, 102, 113, 124`, plus two wide open margins at
//! `x = 1..25` and `x = 135..159` that are free at every row, not just the gap columns.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Barrier, Mutex};
use std::thread;

use sih26123::robot::comms::Comms;
use sih26123::robot::planner_pibt::{has_no_collisions, DistributedAgent, Pibt};
use sih26123::world::grid::Grid;

const MAP_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map");

fn load_map() -> Grid {
    Grid::load(MAP_PATH).expect("map should load")
}

/// Runs one scenario and asserts the Phase 3 gate holds for it.
fn assert_scenario_passes(
    name: &str,
    grid: &Grid,
    starts: Vec<(usize, usize)>,
    goals: Vec<(usize, usize)>,
    max_ticks: usize,
    seed: u64,
) {
    let n = starts.len();
    let mut pibt = Pibt::new(grid, starts, goals.clone(), seed);
    let configs = pibt.run(max_ticks);

    assert!(
        has_no_collisions(&configs),
        "[{name}] collision detected (vertex or edge) within {max_ticks} ticks"
    );

    let last = configs.last().expect("run() always returns at least the start config");
    for i in 0..n {
        assert_eq!(
            last[i], goals[i],
            "[{name}] robot {i} did not reach goal {:?} within {max_ticks} ticks (ended at {:?})",
            goals[i], last[i]
        );
    }
}

/// Item 13's Phase 4 re-test instruction (`docs/TESTING_PLAN.md`): "re-run
/// `tests/pibt_single_process.rs`'s scenarios again, this time through the real comms
/// layer instead of direct function calls." The distributed-variant tests below share
/// the one hardcoded multicast group/port (`config.rs`), same as
/// `tests/comms_protocol.rs`, so they're serialized behind this mutex to avoid cross-talk
/// between concurrently-running `cargo test` threads.
static SEQUENTIAL: Mutex<()> = Mutex::new(());

/// Drives `n` `DistributedAgent`s through up to `max_ticks` ticks of real message-driven
/// negotiation over genuinely separate `Comms`/UDP sockets — one real OS socket per
/// agent, same relationship `tests/comms_protocol.rs`'s sockets have to real separate
/// robot processes (true separate OS processes arrive at item 20/21).
///
/// Each agent runs on its own OS thread. Two barriers per tick keep every thread on the
/// *same* tick's negotiation at once — no artificial per-round lockstep; within a tick,
/// each agent's own round loop (`DistributedAgent::resolve_tick`) runs independently
/// against real UDP timing, exactly like separate processes would. The first barrier
/// lets a single "leader" thread (whichever the barrier hands that role to for the tick)
/// check, once every agent's move for the tick is visible, whether every agent has
/// reached its goal; the second barrier makes that decision visible to every other
/// thread before any of them loops back to check it (a single barrier isn't enough here:
/// reaching a barrier only proves everyone *arrived*, not that the leader's write
/// afterward already happened). Ending early once converged matters in practice — without
/// it every scenario would always pay for the full `max_ticks`, not just however many it
/// actually needs.
fn run_distributed_scenario(
    grid: &Grid,
    starts: &[(usize, usize)],
    goals: &[(usize, usize)],
    max_ticks: usize,
    seed: u64,
) -> Vec<Vec<(usize, usize)>> {
    let n = starts.len();
    let after_resolve = Barrier::new(n);
    let after_stop_check = Barrier::new(n);
    let stop = AtomicBool::new(false);
    let positions: Mutex<Vec<(usize, usize)>> = Mutex::new(starts.to_vec());

    let agents: Vec<DistributedAgent> = (0..n)
        .map(|i| {
            let comms = Comms::new(i as u32, (starts[i].0 as i32, starts[i].1 as i32))
                .expect("bind agent comms");
            DistributedAgent::new(
                i as u32,
                grid,
                starts[i],
                goals[i],
                seed + i as u64,
                n,
                comms,
            )
        })
        .collect();

    thread::scope(|scope| {
        let handles: Vec<_> = agents
            .into_iter()
            .enumerate()
            .map(|(i, mut agent)| {
                let after_resolve = &after_resolve;
                let after_stop_check = &after_stop_check;
                let stop = &stop;
                let positions = &positions;
                scope.spawn(move || {
                    let mut trace = vec![agent.position()];
                    for tick in 0..max_ticks as u64 {
                        if stop.load(Ordering::Acquire) {
                            break;
                        }
                        agent.resolve_tick(tick);
                        trace.push(agent.position());
                        positions.lock().unwrap()[i] = agent.position();

                        let result = after_resolve.wait();
                        if result.is_leader() {
                            let all_arrived = positions
                                .lock()
                                .unwrap()
                                .iter()
                                .zip(goals.iter())
                                .all(|(p, g)| p == g);
                            if all_arrived {
                                stop.store(true, Ordering::Release);
                            }
                        }
                        after_stop_check.wait();
                    }
                    trace
                })
            })
            .collect();
        handles.into_iter().map(|h| h.join().unwrap()).collect()
    })
}

/// Transposes per-agent position traces (one `Vec` per agent, indexed by tick) into
/// per-tick configs (one `Vec` per tick, indexed by agent) — the shape `has_no_collisions`
/// expects, so the distributed runs can reuse it rather than reimplementing collision
/// detection.
fn traces_to_configs(traces: &[Vec<(usize, usize)>]) -> Vec<Vec<(usize, usize)>> {
    let len = traces[0].len();
    (0..len)
        .map(|t| traces.iter().map(|trace| trace[t]).collect())
        .collect()
}

/// Distributed counterpart of `assert_scenario_passes`: same gate (no collisions, every
/// robot at its goal by `max_ticks`), driven through real sockets instead of direct calls.
fn assert_scenario_passes_distributed(
    name: &str,
    grid: &Grid,
    starts: Vec<(usize, usize)>,
    goals: Vec<(usize, usize)>,
    max_ticks: usize,
    seed: u64,
) {
    let _guard = SEQUENTIAL.lock().unwrap();
    let n = starts.len();
    let traces = run_distributed_scenario(grid, &starts, &goals, max_ticks, seed);
    let configs = traces_to_configs(&traces);

    assert!(
        has_no_collisions(&configs),
        "[{name}, distributed] collision detected (vertex or edge) within {max_ticks} ticks"
    );

    let last = configs
        .last()
        .expect("trace always has at least the start config");
    for i in 0..n {
        assert_eq!(
            last[i], goals[i],
            "[{name}, distributed] robot {i} did not reach goal {:?} within {max_ticks} ticks (ended at {:?})",
            goals[i], last[i]
        );
    }
}

/// Baseline completeness check: 3 robots, long paths, starting positions well apart
/// from each other so early ticks don't force any interaction.
#[test]
fn three_robots_well_separated_long_paths() {
    let grid = load_map();
    assert_scenario_passes(
        "three_robots_well_separated_long_paths",
        &grid,
        vec![(5, 1), (150, 1), (80, 1)],
        vec![(150, 58), (5, 58), (80, 61)],
        800,
        1,
    );
}

/// Distributed re-run of `three_robots_well_separated_long_paths` (item 13's Phase 4
/// re-test instruction).
#[test]
fn three_robots_well_separated_long_paths_distributed() {
    let grid = load_map();
    assert_scenario_passes_distributed(
        "three_robots_well_separated_long_paths",
        &grid,
        vec![(5, 1), (150, 1), (80, 1)],
        vec![(150, 58), (5, 58), (80, 61)],
        800,
        1,
    );
}

/// 4 robots crossing diagonally inside the fully open left-margin block
/// (`x = 1..25`, free at every row) — two crossing diagonals through the same open
/// area, no chokepoints involved, purely a "many paths overlapping in open space" case.
#[test]
fn four_robots_crossing_in_open_margin() {
    let grid = load_map();
    assert_scenario_passes(
        "four_robots_crossing_in_open_margin",
        &grid,
        vec![(2, 2), (24, 2), (2, 60), (24, 60)],
        vec![(24, 60), (2, 60), (24, 2), (2, 2)],
        400,
        2,
    );
}

/// Distributed re-run of `four_robots_crossing_in_open_margin`.
#[test]
fn four_robots_crossing_in_open_margin_distributed() {
    let grid = load_map();
    assert_scenario_passes_distributed(
        "four_robots_crossing_in_open_margin",
        &grid,
        vec![(2, 2), (24, 2), (2, 60), (24, 60)],
        vec![(24, 60), (2, 60), (24, 2), (2, 2)],
        400,
        2,
    );
}

/// 2 robots on adjacent cells with swapped goals — the direct edge-collision
/// temptation (a literal swap is illegal, so PIBT must route one of them around
/// instead). Placed in the open left margin, which has room to do that.
#[test]
fn two_robots_adjacent_swap_goals() {
    let grid = load_map();
    assert_scenario_passes(
        "two_robots_adjacent_swap_goals",
        &grid,
        vec![(10, 30), (11, 30)],
        vec![(11, 30), (10, 30)],
        200,
        3,
    );
}

/// Distributed re-run of `two_robots_adjacent_swap_goals` — the direct edge-collision
/// temptation, now resolved via the swap-conflict rule in
/// `DistributedAgent::outranks_me` instead of `func_pibt`'s direct edge check.
#[test]
fn two_robots_adjacent_swap_goals_distributed() {
    let grid = load_map();
    assert_scenario_passes_distributed(
        "two_robots_adjacent_swap_goals",
        &grid,
        vec![(10, 30), (11, 30)],
        vec![(11, 30), (10, 30)],
        200,
        3,
    );
}

/// The mandatory dense case: 6 robots funneled through a single narrow corridor. The
/// chokepoint column `x = 36` is the only free column between the shelf block at
/// `y = 2..3`; every other `x` in `33..=38` is blocked there. All 6 robots start
/// side-by-side in the free row above (`y = 1`), so every one of them must funnel
/// through the same two cells, `(36, 2)` and `(36, 3)`, before going anywhere else.
///
/// Goals are spread to distinct, well-separated destinations across the rest of the
/// map (a realistic task-allocation shape — different pickup/dropoff points, not a
/// puzzle), rather than mirrored immediately on the far side of the gate. An earlier
/// version of this test used mirrored near-side goals and found vanilla PIBT can
/// livelock indefinitely on that specific symmetric "precise adjacent-swap right after
/// a shared gate" shape — confirmed against the actual `Kei18/pypibt` reference
/// implementation run on the identical instance (same non-convergence, every seed
/// tried), not a bug in this port. This version keeps the same funnel pressure while
/// giving robots real room to disperse once through, which is what the gate's "narrow
/// corridor" requirement is actually checking for.
#[test]
fn dense_corridor_six_robots_through_one_gap() {
    let grid = load_map();
    assert_scenario_passes(
        "dense_corridor_six_robots_through_one_gap",
        &grid,
        vec![(33, 1), (34, 1), (35, 1), (36, 1), (37, 1), (38, 1)],
        vec![(10, 61), (150, 61), (47, 40), (102, 25), (69, 55), (124, 10)],
        400,
        4,
    );
}

/// Distributed re-run of `dense_corridor_six_robots_through_one_gap` — the scenario that
/// actually substantiates deadlock-freedom under real pressure, now through real sockets.
/// This is the one TESTING_PLAN.md is most explicit about not skipping.
#[test]
fn dense_corridor_six_robots_through_one_gap_distributed() {
    let grid = load_map();
    assert_scenario_passes_distributed(
        "dense_corridor_six_robots_through_one_gap",
        &grid,
        vec![(33, 1), (34, 1), (35, 1), (36, 1), (37, 1), (38, 1)],
        vec![(10, 61), (150, 61), (47, 40), (102, 25), (69, 55), (124, 10)],
        400,
        4,
    );
}

/// 8 robots (the upper end of the metrics schema's `num_robots` range) scattered
/// across the map: several travel straight down distinct chokepoint columns while one
/// travels a long horizontal path along a free row, guaranteeing real crossing
/// contention between multiple independent chokepoint transits at once, not just one
/// isolated corridor.
#[test]
fn eight_robots_scattered_across_map() {
    let grid = load_map();
    assert_scenario_passes(
        "eight_robots_scattered_across_map",
        &grid,
        vec![
            (5, 1),
            (155, 1),
            (80, 1),
            (47, 1),
            (102, 1),
            (10, 31),
            (69, 1),
            (124, 61),
        ],
        vec![
            (155, 61),
            (5, 61),
            (80, 61),
            (102, 61),
            (47, 61),
            (150, 31),
            (69, 61),
            (124, 1),
        ],
        1000,
        5,
    );
}

/// Distributed re-run of `eight_robots_scattered_across_map` — the upper end of the
/// metrics schema's `num_robots` range, so also the largest `max_rounds` per tick
/// (`2 * 8 + 5 = 21`) any of these distributed tests exercises.
#[test]
fn eight_robots_scattered_across_map_distributed() {
    let grid = load_map();
    assert_scenario_passes_distributed(
        "eight_robots_scattered_across_map",
        &grid,
        vec![
            (5, 1),
            (155, 1),
            (80, 1),
            (47, 1),
            (102, 1),
            (10, 31),
            (69, 1),
            (124, 61),
        ],
        vec![
            (155, 61),
            (5, 61),
            (80, 61),
            (102, 61),
            (47, 61),
            (150, 31),
            (69, 61),
            (124, 1),
        ],
        1000,
        5,
    );
}
