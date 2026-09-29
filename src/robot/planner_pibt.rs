//! PIBT (Priority Inheritance with Backtracking) planner.
//!
//! Phase 3, item 9 of `docs/BUILD_PLAN.md` — a direct port of `Kei18/pypibt`'s `PIBT`
//! class (`docs/REFERENCES.md`, Decision 2 in `docs/decisions.md`), run single-process:
//! `Pibt`/`func_pibt` below resolve conflicts via direct recursive function calls between
//! agents, still the reference implementation `tests/pibt_single_process.rs` validates.
//!
//! Item 13 (Phase 4) adds `DistributedAgent`, a message-driven counterpart that reaches
//! the same kind of decision using only `robot::comms::Comms` broadcasts — no shared
//! memory, no direct calls between agents, since real robots are separate OS processes.
//! A literal translation of `func_pibt`'s recursion doesn't work: priority inheritance
//! specifically pulls a low-priority agent's decision *out of normal priority order* the
//! instant a higher-priority agent wants its cell (this is what actually prevents
//! deadlocks, e.g. two agents facing off in a corridor) — a protocol that just resolves
//! conflicts in strict global-priority order does not reproduce that. And `func_pibt`'s
//! "every candidate blocked" fallback unconditionally overwrites whoever else's tentative
//! claim was sitting on the agent's own current cell, because nobody can be evicted from
//! a cell they're already occupying if they truly have nowhere else to go — that's not a
//! priority contest at all. `DistributedAgent::resolve_tick` reproduces both properties
//! with a round-based "broadcast intent, yield-on-outrank, unconditional exhausted
//! fallback" protocol; see its doc comment for the exact rules and the round-count bound.
//!
//! Two adaptations from the Python reference, semantics preserved, encoding made
//! idiomatic for Rust:
//! - `pypibt` uses sentinel values (`NIL = N`, `NIL_COORD = grid.shape`) for
//!   "unassigned"; this uses `Option<Cell>` / `Option<usize>` instead.
//! - `pypibt` uses full-grid NumPy arrays for `occupied_now`/`occupied_nxt`; this uses
//!   `HashMap<Cell, usize>`, since at most a handful of robots are ever occupying
//!   anything on a map with thousands of free cells.
//!
//! Goals are static per agent here, matching `docs/TESTING_PLAN.md`'s Phase 3 scope
//! ("static goals", no dynamic obstacles) — dynamic per-task goals arrive with the task
//! layer (item 15+).

use std::collections::{HashMap, HashSet};
use std::time::Duration;

use rand::rngs::StdRng;
use rand::seq::SliceRandom;
use rand::SeedableRng;

use crate::config::AUTONOMOUS_YIELD_TIMEOUT_TICKS;
use crate::protocol::messages::PoseIntent;
use crate::robot::comms::{Comms, Payload};
use crate::world::grid::Grid;

/// A grid cell, `(x, y)`.
pub type Cell = (usize, usize);

/// One tick's positions, one entry per agent, indexed by agent id.
pub type Config = Vec<Cell>;

/// PIBT solver state for a fixed set of agents with fixed goals.
pub struct Pibt {
    grid: Grid,
    starts: Vec<Cell>,
    goals: Vec<Cell>,
    /// `dist_tables[i]` is the BFS distance-to-goal table for agent `i`'s goal
    /// (`world::grid::Grid::bfs_distance`, item 6) — the same table
    /// `PS_AND_ARCHITECTURE.md` §3.3 describes the greedy step as consulting.
    dist_tables: Vec<HashMap<Cell, u32>>,
    rng: StdRng,
}

impl Pibt {
    /// Builds a solver for `starts[i]` -> `goals[i]` pairs on `grid`. `seed` makes tie
    /// breaking (and therefore the whole run) reproducible, per Phase 8's seeded-run
    /// requirement.
    pub fn new(grid: &Grid, starts: Vec<Cell>, goals: Vec<Cell>, seed: u64) -> Self {
        assert_eq!(
            starts.len(),
            goals.len(),
            "must have exactly one goal per start"
        );
        let dist_tables = goals.iter().map(|&g| grid.bfs_distance(g)).collect();
        Pibt {
            grid: grid.clone(),
            starts,
            goals,
            dist_tables,
            rng: StdRng::seed_from_u64(seed),
        }
    }

    /// Changes agent `i`'s goal and recomputes its distance table — item 28's
    /// in-process `pibt_token` benchmark run needs this for the same reason
    /// `DistributedAgent::retarget` (below) already exists: `TaskLayer::current_goal()`
    /// changes over a robot's lifetime (pickup, then dropoff, then whatever's claimed
    /// next), but `Pibt::run`'s single static `starts[i] -> goals[i]` pass doesn't fit a
    /// task-driven simulation at all — that orchestration is `run_experiment.rs`'s own
    /// tick-by-tick loop around `step`, not a new method on this type.
    pub fn retarget(&mut self, i: usize, goal: Cell) {
        self.dist_tables[i] = self.grid.bfs_distance(goal);
        self.goals[i] = goal;
    }

    /// Computes the next configuration from `q_from`, given each agent's current
    /// `priorities` (higher plans first). Direct port of `pypibt`'s `PIBT.step`.
    pub fn step(&mut self, q_from: &[Cell], priorities: &[f64]) -> Config {
        let n = q_from.len();
        let mut q_to: Vec<Option<Cell>> = vec![None; n];

        let mut occupied_now: HashMap<Cell, usize> = HashMap::with_capacity(n);
        for (i, &cell) in q_from.iter().enumerate() {
            occupied_now.insert(cell, i);
        }
        let mut occupied_nxt: HashMap<Cell, usize> = HashMap::with_capacity(n);

        // Highest priority plans first.
        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by(|&a, &b| priorities[b].partial_cmp(&priorities[a]).unwrap());

        for i in order {
            if q_to[i].is_none() {
                func_pibt(
                    &self.grid,
                    &self.dist_tables,
                    &mut self.rng,
                    q_from,
                    &mut q_to,
                    &occupied_now,
                    &mut occupied_nxt,
                    i,
                );
            }
        }

        q_to.into_iter()
            .map(|c| c.expect("every agent must be assigned a next cell by func_pibt"))
            .collect()
    }

    /// Runs until every agent reaches its goal or `max_ticks` elapses. Returns the full
    /// sequence of configurations, `configs[0] == starts`.
    ///
    /// Priority update rule (direct port of `pypibt`'s `PIBT.run`): an agent not yet at
    /// its goal has its priority incremented every tick — "priority rises the longer a
    /// robot has waited" (`PS_AND_ARCHITECTURE.md` §3.3) — and an agent that just
    /// arrived has its priority reset to its fractional part (dropping the accumulated
    /// integer part), so a fresh, small tie-breaking value is what's left once it's
    /// done "waiting."
    pub fn run(&mut self, max_ticks: usize) -> Vec<Config> {
        let n = self.starts.len();
        let cell_count = (self.grid.width() * self.grid.height()) as f64;
        let mut priorities: Vec<f64> = (0..n)
            .map(|i| {
                let d = self
                    .dist_tables[i]
                    .get(&self.starts[i])
                    .copied()
                    .unwrap_or(0) as f64;
                d / cell_count
            })
            .collect();

        let mut configs = vec![self.starts.clone()];
        for _ in 0..max_ticks {
            let q_from = configs.last().unwrap().clone();
            let q_to = self.step(&q_from, &priorities);

            let mut all_at_goal = true;
            for i in 0..n {
                if q_to[i] != self.goals[i] {
                    all_at_goal = false;
                    priorities[i] += 1.0;
                } else {
                    priorities[i] -= priorities[i].floor();
                }
            }
            configs.push(q_to);
            if all_at_goal {
                break;
            }
        }
        configs
    }
}

/// Core PIBT step for a single agent `i`: tries candidate next cells in order of
/// increasing distance-to-goal (ties broken randomly), securing the first one that
/// doesn't cause a vertex or edge collision — recursively asking a lower-priority
/// occupant to move out of the way (priority inheritance) when needed.
///
/// Direct port of `pypibt`'s `funcPIBT`. Returns `true` if agent `i` secured a move,
/// `false` if it had to stay in place because every candidate failed.
#[allow(clippy::too_many_arguments)]
fn func_pibt(
    grid: &Grid,
    dist_tables: &[HashMap<Cell, u32>],
    rng: &mut StdRng,
    q_from: &[Cell],
    q_to: &mut [Option<Cell>],
    occupied_now: &HashMap<Cell, usize>,
    occupied_nxt: &mut HashMap<Cell, usize>,
    i: usize,
) -> bool {
    // Candidates: stay in place, or move to a free 4-connected neighbor.
    let mut candidates = vec![q_from[i]];
    candidates.extend(grid.neighbors(q_from[i].0, q_from[i].1));

    // Randomized tie-breaking: shuffle first, then a *stable* sort by distance-to-goal
    // so cells with equal distance keep their shuffled relative order (matches
    // `pypibt`: `rng.shuffle(C)` then `sorted(C, key=...)`, and Python's `sorted` is
    // stable, same as Rust's `sort_by_key`).
    candidates.shuffle(rng);
    candidates.sort_by_key(|c| dist_tables[i].get(c).copied().unwrap_or(u32::MAX));

    for v in candidates {
        // Vertex-collision avoidance: someone already secured v this tick.
        if occupied_nxt.contains_key(&v) {
            continue;
        }

        let j = occupied_now.get(&v).copied();

        // Edge-collision avoidance: v's current occupant is about to swap into i's
        // current cell (a two-agent position swap across one tick).
        if let Some(j) = j {
            if q_to[j] == Some(q_from[i]) {
                continue;
            }
        }

        // Tentatively reserve v for i.
        q_to[i] = Some(v);
        occupied_nxt.insert(v, i);

        // Priority inheritance: if v is occupied and its occupant hasn't been assigned
        // a move yet, ask it to move out of the way. If that recursive attempt fails,
        // this candidate doesn't work for i either -- but note the tentative
        // reservation above is deliberately *not* undone before trying the next
        // candidate. This matches pypibt's funcPIBT exactly: v becomes a dead cell for
        // this tick (an unresolvable inheritance chain sits there), so no one else
        // re-triggers the same failed resolution while i tries something else.
        if let Some(j) = j {
            if q_to[j].is_none()
                && !func_pibt(
                    grid,
                    dist_tables,
                    rng,
                    q_from,
                    q_to,
                    occupied_now,
                    occupied_nxt,
                    j,
                )
            {
                continue;
            }
        }

        return true;
    }

    // No candidate worked: stay in place.
    q_to[i] = Some(q_from[i]);
    occupied_nxt.insert(q_from[i], i);
    false
}

/// True if `configs` (as produced by repeated `step`/`run` calls) has no vertex
/// collisions (two agents on the same cell at the same tick) and no edge collisions
/// (two agents swapping cells across one tick). Exposed so item 10's dedicated test
/// suite (`tests/pibt_single_process.rs`) can reuse this rather than reimplementing it.
pub fn has_no_collisions(configs: &[Config]) -> bool {
    for t in 0..configs.len() {
        let config = &configs[t];

        let mut seen: HashSet<Cell> = HashSet::with_capacity(config.len());
        for &cell in config {
            if !seen.insert(cell) {
                return false; // vertex collision
            }
        }

        if t > 0 {
            let prev = &configs[t - 1];
            for i in 0..config.len() {
                for j in (i + 1)..config.len() {
                    if config[i] == prev[j] && config[j] == prev[i] {
                        return false; // edge collision (swap)
                    }
                }
            }
        }
    }
    true
}

/// One round's listen-and-drain window. Real loopback delivery is sub-100-microsecond,
/// but this needs margin against real OS thread-scheduling jitter under load, not just
/// against transmission time — `tests/pibt_single_process.rs`'s distributed scenarios hit
/// a genuine, reproducible collision under a smaller value (`500µs`) on a loaded machine,
/// confirmed to be a timing-margin problem rather than a resolution-rule bug by hand
/// (working through `two_robots_adjacent_swap_goals_distributed`'s exact tie-broken
/// priorities shows both agents converging cleanly assuming reliable delivery). This is
/// also the dominant per-round cost (`drain_round` always blocks for the full window on
/// its last, confirming-nothing-left call), so it directly trades test runtime for
/// margin — `2ms` is the smallest value that survived repeated full-suite runs without a
/// failure during this item's own testing.
const ROUND_WINDOW: Duration = Duration::from_millis(2);

fn cell_to_wire(c: Cell) -> (i32, i32) {
    (c.0 as i32, c.1 as i32)
}

fn wire_to_cell(c: (i32, i32)) -> Cell {
    (c.0 as usize, c.1 as usize)
}

/// One robot's message-driven PIBT negotiation state — item 13's replacement for
/// `func_pibt`'s direct recursive calls, using only `robot::comms::Comms` broadcasts.
/// See this module's top-level doc comment for why a naive strict-priority-order
/// translation doesn't preserve `func_pibt`'s actual deadlock-avoidance guarantee.
pub struct DistributedAgent {
    robot_id: u32,
    grid: Grid,
    dist_table: HashMap<Cell, u32>,
    goal: Cell,
    pos: Cell,
    priority: f64,
    comms: Comms,
    rng: StdRng,
    max_rounds: usize,
    /// Consecutive ticks this agent's best candidate has been occupied by a
    /// locally-sensed peer while in Autonomous mode (`resolve_tick_autonomous`) — item
    /// 20/Decision 6's head-on tie-break timer. Irrelevant in Cooperative/Cautious mode.
    autonomous_block_ticks: u32,
}

impl DistributedAgent {
    /// `seed` drives only this agent's own randomized candidate tie-breaking (see
    /// `Pibt::new`). `fleet_size` is the total number of agents in this negotiation
    /// (not just how many are ever in range) — it sizes `max_rounds` per the bound
    /// `resolve_tick` derives: a forced yield can cascade at most one hop per round,
    /// through at most every other agent, so a worst-case cascade needs at most
    /// `fleet_size` rounds to fully propagate. `3 * fleet_size + 10` triples that plus
    /// adds a flat buffer — not just doubles it — because the bound above assumes every
    /// round's broadcast is actually received on time; the extra margin is slack against
    /// an occasional round being missed under real scheduling jitter rather than against
    /// cascade length itself (see `ROUND_WINDOW`'s doc comment for the failure this
    /// margin was added to fix).
    pub fn new(
        robot_id: u32,
        grid: &Grid,
        start: Cell,
        goal: Cell,
        seed: u64,
        fleet_size: usize,
        comms: Comms,
    ) -> Self {
        let dist_table = grid.bfs_distance(goal);
        let cell_count = (grid.width() * grid.height()) as f64;
        let priority = dist_table.get(&start).copied().unwrap_or(0) as f64 / cell_count;
        DistributedAgent {
            robot_id,
            grid: grid.clone(),
            dist_table,
            goal,
            pos: start,
            priority,
            comms,
            rng: StdRng::seed_from_u64(seed),
            max_rounds: 3 * fleet_size + 10,
            autonomous_block_ticks: 0,
        }
    }

    pub fn position(&self) -> Cell {
        self.pos
    }

    pub fn at_goal(&self) -> bool {
        self.pos == self.goal
    }

    /// Updates this agent's goal mid-run and recomputes the BFS distance table it plans
    /// against — needed for interoperability with `task_layer::TaskLayer`, whose
    /// `current_goal()` changes as a robot picks up and drops off tasks over its
    /// lifetime. Without this, an agent's goal would be fixed forever at construction,
    /// which was fine for Phase 3's static-goal scope but not once a task layer is
    /// driving the destination.
    pub fn retarget(&mut self, goal: Cell) {
        self.dist_table = self.grid.bfs_distance(goal);
        self.goal = goal;
    }

    /// Marks a cell blocked/free in this agent's own map (Decision 18's operator
    /// `BlockCell`) and recomputes its distance table for the current goal, since the old
    /// one is stale as soon as the grid changes.
    pub fn set_blocked(&mut self, x: usize, y: usize, blocked: bool) {
        self.grid.set_blocked(x, y, blocked);
        self.dist_table = self.grid.bfs_distance(self.goal);
    }

    /// Runs one tick's negotiation to completion and commits this agent's move for the
    /// tick, updating `position` and `priority` (same update rule as `Pibt::run`: rises
    /// while not at goal, resets to its fractional part on arrival). Returns the
    /// committed next cell.
    ///
    /// Protocol, run independently by every agent with no shared memory:
    /// 1. Rank candidates (stay + free neighbors) exactly as `func_pibt` does: shuffle,
    ///    then stable-sort by BFS distance-to-goal. `ptr` starts at the best (index 0).
    /// 2. Each round: broadcast `PoseIntent{intended_next: candidates[ptr], priority,
    ///    exhausted}`, then listen for `ROUND_WINDOW`, collecting the latest claim heard
    ///    from every other agent *for this same tick* (a stale claim from a slower
    ///    agent's previous tick is discarded, not compared against).
    /// 3. A heard claim conflicts with `candidates[ptr]` if it targets the same cell
    ///    (vertex) or if its sender currently occupies `candidates[ptr]` and intends to
    ///    move into this agent's own current cell (a swap). An `exhausted` conflicting
    ///    claim always wins outright; otherwise the higher `priority` wins, ties broken
    ///    by the lower `robot_id` — both sides compute this identically, so no arbiter is
    ///    needed to agree on the outcome. Losing advances `ptr`; walking off the end of
    ///    the candidate list marks this agent `exhausted`, after which it only ever
    ///    re-broadcasts staying at its own current cell, unconditionally.
    /// 4. After `max_rounds`, commit to whatever was broadcast in the *final* round —
    ///    not to a locally-updated-but-never-broadcast choice, which peers would have no
    ///    way to have seen or agreed with. `max_rounds` is sized with enough margin past
    ///    the worst-case convergence bound that the last several rounds are always
    ///    redundant confirmations in practice, so this is never actually a live cutoff.
    pub fn resolve_tick(&mut self, tick: u64) -> Cell {
        let mut candidates = vec![self.pos];
        candidates.extend(self.grid.neighbors(self.pos.0, self.pos.1));
        candidates.shuffle(&mut self.rng);
        candidates.sort_by_key(|c| self.dist_table.get(c).copied().unwrap_or(u32::MAX));

        let mut ptr = 0usize;
        let mut exhausted = false;
        let mut committed_target = self.pos;
        let mut committed_exhausted = false;

        self.comms.set_position(cell_to_wire(self.pos));

        for _round in 0..self.max_rounds {
            let target = if exhausted { self.pos } else { candidates[ptr] };
            committed_target = target;
            committed_exhausted = exhausted;

            let intent = PoseIntent {
                robot_id: self.robot_id,
                tick,
                position: cell_to_wire(self.pos),
                intended_next: cell_to_wire(target),
                priority: self.priority,
                exhausted,
            };
            // A transient send failure (e.g. a real Wi-Fi dead zone, `docs/
            // PS_AND_ARCHITECTURE.md` §1.1's own core motivation) must not crash the
            // whole robot — this is exactly the failure mode the project exists to
            // survive, not reproduce in its own control code. Log and keep going: the
            // next round (or next tick) gets another chance to broadcast.
            if let Err(e) = self.comms.send_pose_intent(intent) {
                eprintln!("[robot {}] failed to send pose intent: {e}", self.robot_id);
            }

            let heard = self.drain_round(tick);

            if exhausted {
                continue; // final; keep re-broadcasting so slower peers still see it
            }

            let outranked = heard.values().any(|other| self.outranks_me(target, other));
            if outranked {
                ptr += 1;
                if ptr >= candidates.len() {
                    exhausted = true;
                }
            }
        }

        let final_pos = if committed_exhausted {
            self.pos
        } else {
            committed_target
        };

        self.update_priority_after_move(final_pos);
        self.pos = final_pos;
        final_pos
    }

    /// Shared priority-update rule between `resolve_tick` and `resolve_tick_autonomous`:
    /// rises while not at goal (so a long-waiting agent eventually wins any negotiation
    /// it re-enters), resets to its fractional part on arrival. Applied even in
    /// Autonomous mode, where priority isn't used for anything right now, purely so a
    /// clean value is already waiting if the robot's mode later recovers to
    /// Cooperative/Cautious and priority starts mattering again.
    fn update_priority_after_move(&mut self, final_pos: Cell) {
        if final_pos == self.goal {
            self.priority -= self.priority.floor();
        } else {
            self.priority += 1.0;
        }
    }

    /// Autonomous-mode movement (item 20, Decision 6 — `docs/decisions.md`): no message
    /// negotiation at all, since Autonomous mode is defined as comms being unreliable
    /// enough that a PIBT round exchange can't be trusted. Instead: rank candidates
    /// exactly as `resolve_tick` does (stay + free neighbors, shuffled then sorted by
    /// BFS distance-to-goal), and take the best-ranked one that isn't currently occupied
    /// by a locally-sensed peer (`local_obstacles`, from `perception::PerceivedState::
    /// LocalOnly` — position only, never intent, since intent can't be trusted here).
    ///
    /// If the single *best* candidate is the one that's blocked, this is (or might be) a
    /// head-on standoff — `docs/decisions.md` Decision 6's exact concern, since neither
    /// robot has any negotiation channel to decide who yields. After
    /// `AUTONOMOUS_YIELD_TIMEOUT_TICKS` consecutive ticks of that same best candidate
    /// staying blocked (not immediately, so an ordinary momentary crossing doesn't
    /// misfire the tie-break), the lower `robot_id` yields: it backs off to the nearest
    /// other free, unoccupied candidate (not necessarily closer to its goal) rather than
    /// waiting indefinitely. The higher `robot_id` just keeps waiting at its current
    /// position until the way is clear — both sides compute this identically from purely
    /// local information (their own id, the blocker's id from `local_obstacles`, their
    /// own map), no message exchange needed.
    pub fn resolve_tick_autonomous(&mut self, local_obstacles: &[(u32, Cell)]) -> Cell {
        let mut candidates = vec![self.pos];
        candidates.extend(self.grid.neighbors(self.pos.0, self.pos.1));
        candidates.shuffle(&mut self.rng);
        candidates.sort_by_key(|c| self.dist_table.get(c).copied().unwrap_or(u32::MAX));

        let occupied_by: HashMap<Cell, u32> =
            local_obstacles.iter().map(|&(id, cell)| (cell, id)).collect();

        let best = candidates[0];
        let final_pos = if let Some(&blocker_id) = occupied_by.get(&best) {
            self.autonomous_block_ticks += 1;
            if self.autonomous_block_ticks >= AUTONOMOUS_YIELD_TIMEOUT_TICKS
                && self.robot_id < blocker_id
            {
                // I'm the lower id in a standing standoff: yield by backing off to any
                // *other* free, unoccupied candidate — clears the contested cell even
                // though it may move me away from my goal. Explicitly excludes
                // `self.pos` too, not just `best`: `candidates` always includes staying
                // in place as one of its entries, and "yielding" to your own current
                // cell is a no-op that would leave the standoff completely unresolved.
                self.autonomous_block_ticks = 0;
                candidates
                    .iter()
                    .copied()
                    .find(|&c| c != best && c != self.pos && !occupied_by.contains_key(&c))
                    .unwrap_or(self.pos) // nowhere safe to back off to: stay put
            } else {
                // Either the timeout hasn't elapsed yet, or I'm the higher id waiting
                // for the other robot to yield first.
                self.pos
            }
        } else {
            // `best` is by definition not in `occupied_by` here, and it's the first
            // (closest-to-goal) entry in the sorted candidate list — nothing ranked
            // ahead of it to prefer, so it's simply the move to take.
            self.autonomous_block_ticks = 0;
            best
        };

        self.update_priority_after_move(final_pos);
        self.pos = final_pos;
        final_pos
    }

    fn drain_round(&self, tick: u64) -> HashMap<u32, PoseIntent> {
        self.comms
            .set_read_timeout(Some(ROUND_WINDOW))
            .expect("set round window timeout");
        let mut heard = HashMap::new();
        loop {
            match self.comms.recv_filtered() {
                Ok(Some(received)) => {
                    if let Payload::Pose(intent) = received.payload {
                        if intent.tick == tick {
                            heard.insert(intent.robot_id, intent);
                        }
                    }
                }
                Ok(None) => continue, // recv_filtered resolves internally; defensive only
                Err(_) => break,      // window elapsed: nothing more queued right now
            }
        }
        heard
    }

    /// True if `other` beats this agent's current `target` proposal. Only meaningful
    /// when there's an actual vertex or swap conflict between them; see
    /// `resolve_tick`'s doc comment for the full resolution rule.
    fn outranks_me(&self, target: Cell, other: &PoseIntent) -> bool {
        let other_next = wire_to_cell(other.intended_next);
        let other_pos = wire_to_cell(other.position);

        let vertex_conflict = other_next == target;
        let swap_conflict = other_pos == target && other_next == self.pos;
        if !vertex_conflict && !swap_conflict {
            return false;
        }
        if other.exhausted {
            return true;
        }
        other.priority > self.priority
            || (other.priority == self.priority && other.robot_id < self.robot_id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Phase 3 gate fallback (`docs/TESTING_PLAN.md`): no dedicated `tests/*.rs` file
    /// exists until item 10, which is next. This is a lightweight smoke test, not
    /// item 10's fuller job (5 configurations including a dense corridor case) --
    /// two agents on the real warehouse map with non-conflicting start/goal pairs,
    /// confirming the port runs, both reach goal, and no collision occurs.
    #[test]
    fn two_agents_reach_goals_without_collision() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map");
        let grid = Grid::load(path).expect("map should load");

        let free = grid.free_cells();
        // Two well-separated free cells, taken from opposite ends of the free-cell
        // list (row-major order), so the pairs don't trivially start adjacent.
        let start_a = free[0];
        let goal_a = free[free.len() / 2];
        let start_b = free[free.len() - 1];
        let goal_b = free[free.len() / 2 + 1];

        let mut pibt = Pibt::new(&grid, vec![start_a, start_b], vec![goal_a, goal_b], 42);
        let configs = pibt.run(500);

        assert!(has_no_collisions(&configs), "collision detected in run");

        let last = configs.last().unwrap();
        assert_eq!(last[0], goal_a, "agent 0 did not reach its goal in time");
        assert_eq!(last[1], goal_b, "agent 1 did not reach its goal in time");
    }

    fn make_agent(robot_id: u32, start: Cell, goal: Cell) -> DistributedAgent {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map");
        let grid = Grid::load(path).expect("map should load");
        let comms = Comms::new(robot_id, cell_to_wire(start)).expect("bind comms");
        DistributedAgent::new(robot_id, &grid, start, goal, 42, 2, comms)
    }

    /// Item 20/Decision 6: with no locally-sensed obstacles at all, Autonomous mode
    /// should behave like ordinary greedy movement toward the goal.
    #[test]
    fn autonomous_mode_moves_toward_goal_when_unobstructed() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map");
        let grid = Grid::load(path).expect("map should load");
        let free = grid.free_cells();
        let start = free[0];
        let goal = free[10];

        let mut agent = make_agent(1, start, goal);
        for _ in 0..200 {
            if agent.at_goal() {
                break;
            }
            agent.resolve_tick_autonomous(&[]);
        }
        assert_eq!(agent.position(), goal, "should reach goal with nothing in the way");
    }

    /// Decision 6's exact scenario: a persistent head-on block. The lower `robot_id`
    /// must keep waiting for `AUTONOMOUS_YIELD_TIMEOUT_TICKS` ticks (not misfire on the
    /// very first blocked tick, which would misfire on any ordinary momentary crossing),
    /// then yield by moving to a different cell — proving the deadlock actually resolves
    /// rather than persisting forever, which is the entire point of the fix.
    #[test]
    fn lower_id_yields_after_timeout_on_persistent_head_on_block() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map");
        let grid = Grid::load(path).expect("map should load");
        let free = grid.free_cells();
        let start = free[0];
        // A goal one step past the best neighbor, so `candidates[0]` (after sort) is
        // stable and predictable: the single neighbor cell closest to `goal`.
        let goal = free[20];
        let mut agent = make_agent(1, start, goal); // lower id than the blocker below

        let best_neighbor = *grid
            .neighbors(start.0, start.1)
            .iter()
            .min_by_key(|c| {
                grid.bfs_distance(goal).get(c).copied().unwrap_or(u32::MAX)
            })
            .expect("map has a neighbor");
        let blocker = vec![(2u32, best_neighbor)]; // robot 2 > robot 1

        for i in 0..(AUTONOMOUS_YIELD_TIMEOUT_TICKS - 1) {
            let pos = agent.resolve_tick_autonomous(&blocker);
            assert_eq!(
                pos, start,
                "should still be waiting on tick {i}, not yet timed out"
            );
        }
        let after_timeout = agent.resolve_tick_autonomous(&blocker);
        assert_ne!(
            after_timeout, start,
            "lower id must yield (move away) once the timeout elapses"
        );
        assert_ne!(
            after_timeout, best_neighbor,
            "must not move into the cell the blocker is reported to occupy"
        );
    }

    /// The mirror case: the *higher* `robot_id` in a standoff must keep waiting
    /// indefinitely rather than ever yielding itself — otherwise both sides could yield
    /// simultaneously (or neither would), defeating the whole point of a tie-break.
    #[test]
    fn higher_id_keeps_waiting_indefinitely_in_a_head_on_block() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map");
        let grid = Grid::load(path).expect("map should load");
        let free = grid.free_cells();
        let start = free[0];
        let goal = free[20];
        let mut agent = make_agent(5, start, goal); // higher id than the blocker below

        let best_neighbor = *grid
            .neighbors(start.0, start.1)
            .iter()
            .min_by_key(|c| {
                grid.bfs_distance(goal).get(c).copied().unwrap_or(u32::MAX)
            })
            .expect("map has a neighbor");
        let blocker = vec![(1u32, best_neighbor)]; // robot 1 < robot 5

        for i in 0..20 {
            let pos = agent.resolve_tick_autonomous(&blocker);
            assert_eq!(pos, start, "higher id must keep waiting forever on tick {i}");
        }
    }

    /// Once the blocking peer is no longer sensed, movement resumes normally — the
    /// block timer isn't a one-way ratchet.
    #[test]
    fn resumes_normal_movement_once_the_obstacle_clears() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map");
        let grid = Grid::load(path).expect("map should load");
        let free = grid.free_cells();
        let start = free[0];
        let goal = free[20];
        let mut agent = make_agent(1, start, goal);

        let best_neighbor = *grid
            .neighbors(start.0, start.1)
            .iter()
            .min_by_key(|c| {
                grid.bfs_distance(goal).get(c).copied().unwrap_or(u32::MAX)
            })
            .expect("map has a neighbor");
        let blocker = vec![(2u32, best_neighbor)];

        // Block for one tick, well under the timeout, then clear it.
        agent.resolve_tick_autonomous(&blocker);
        let pos = agent.resolve_tick_autonomous(&[]);
        assert_eq!(pos, best_neighbor, "should resume toward goal once unblocked");
    }
}
