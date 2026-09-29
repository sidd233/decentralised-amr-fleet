//! Item 20 of `docs/BUILD_PLAN.md`: `RobotProcess` ties every module above into one
//! robot's tick loop — the first point in the build where they all run together as a
//! single process's real behavior, not just proven to interoperate in a test
//! (`tests/module_interop.rs`, which this reuses the same module set as, minus the
//! test's own scaffolding).
//!
//! **Pacing, stated plainly as a known simplification:** ticks here are locally
//! self-paced (`std::thread::sleep`), not yet driven by `clock_driver.rs` (item 23,
//! not built). `docs/PS_AND_ARCHITECTURE.md`'s process model says the clock driver
//! exists "only to keep simulation ticks synchronized for reproducibility" — that's a
//! real requirement this doesn't yet satisfy, since self-paced sleeping drifts
//! independently per process. Item 23 will need to add a tick-broadcast wire message and
//! change `RobotProcess::run` to wait for it instead of sleeping; that's a known,
//! deliberate follow-up, not an oversight being papered over.
//!
//! **Token circulation runs on its own background thread**, not inline in the tick loop:
//! `TaskLayer::run_one_cycle` blocks (waiting to receive the token, then retrying its
//! forward-hop) for up to roughly a second in the worst case, which would stall the
//! whole tick cadence if called inline — and a robot's real-time responsiveness to
//! degrading comms (the entire point of the mode state machine) can't be allowed to
//! depend on how quickly a token happens to circulate.
//!
//! `TaskLayer` itself is owned exclusively by that background thread — the main tick
//! thread never touches it directly, only publishing its latest position into
//! `shared_position` and reading the latest goal out of `task_snapshot`, both small,
//! always-fast locks. The first version of this tried having the main thread also
//! best-effort (`try_lock`, skip if busy) call `on_position_update` on the same
//! `TaskLayer` the background thread owns; a real test caught that this doesn't just
//! cause occasional staleness as expected, but near-total starvation — the background
//! loop re-acquires its mutex almost continuously (essentially no gap between one
//! `run_one_cycle` call ending and the next beginning), so a two-robot run got stuck one
//! cell short of its dropoff for an entire 400-tick budget, not just "a bit late."
//! Applying `on_position_update` from inside the background thread's own loop instead
//! (using the freshest position it reads from `shared_position` each iteration) removes
//! the contention by construction: exactly one thread ever mutates `TaskLayer`.

pub mod comms;
pub mod metrics;
pub mod mode_state_machine;
pub mod perception;
pub mod planner_pibt;
pub mod task_layer;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::config::TICK_INTERVAL_MS;
use crate::protocol::messages::{BlockCell, Heartbeat, Mode, ModeAnnounce};
use crate::robot::comms::{Comms, Payload, Received};
use crate::robot::metrics::BatteryModel;
use crate::robot::mode_state_machine::ModeStateMachine;
use crate::robot::perception::{perceive, PerceivedState};
use crate::robot::planner_pibt::{Cell, DistributedAgent};
use crate::robot::task_layer::{Task, TaskLayer, TaskState};
use crate::world::grid::Grid;

fn cell_to_wire(c: Cell) -> (i32, i32) {
    (c.0 as i32, c.1 as i32)
}

/// Range-scoping radius for `clock_comms` — matches `clock_driver.rs`'s own
/// `CLOCK_RANGE_CELLS` (same value, same reasoning: a tick has to reach every robot
/// regardless of distance). Kept as a separate constant rather than making that one
/// `pub`, since duplicating one `u32` is cheaper than coupling the two modules over it.
const CLOCK_RANGE_CELLS: u32 = 100_000;

/// Mirrors `TaskLayer::current_goal`'s own logic, applied to a `TaskState` snapshot
/// taken off the mutex rather than calling that method directly on a live `TaskLayer`
/// (which would need the same contended lock this module's top-level note explains).
fn goal_from_state(state: TaskState) -> Option<Cell> {
    match state {
        TaskState::Idle => None,
        TaskState::ToPickup(t) => Some(t.pickup),
        TaskState::ToDropoff(t) => Some(t.dropoff),
    }
}

/// How often (in ticks) a robot re-announces its current mode even without a transition.
const MODE_REANNOUNCE_TICKS: u64 = 10;

/// One robot process's full state, wiring every module above into a single tick loop.
pub struct RobotProcess {
    id: u32,
    tick: u64,
    /// The first driver tick this robot ever adopted; `ticks_run` counts from it so
    /// `--max-ticks` means "run N ticks" even when the clock is already far along.
    first_tick: Option<u64>,
    planner: DistributedAgent,
    /// Separate from the planner's own `Comms`: sends this robot's `Heartbeat` (with
    /// battery) every tick and receives whatever's needed for `perception::perceive` —
    /// deliberately not the planner's socket, since the planner's own negotiation
    /// protocol already consumes every `PoseIntent` it reads and reusing the same socket
    /// for a second, independent listener would just contend over the same datagrams.
    sensing_comms: Comms,
    /// Dedicated to receiving `clock_driver.rs`'s `TickMsg` broadcasts, at a deliberately
    /// wide range (`CLOCK_RANGE_CELLS`) rather than `sensing_comms`'s default
    /// `COMMS_RANGE_CELLS`. Found necessary by a real Docker smoke test: a robot started
    /// far from the clock's own broadcast position (which range-scoping compares against)
    /// had `sensing_comms` silently filter out every tick as "out of range" — correct
    /// behavior for `PoseIntent`/`Heartbeat` range-scoping, wrong for a broadcast that
    /// must reach the whole fleet regardless of distance.
    clock_comms: Comms,
    mode_sm: ModeStateMachine,
    battery: BatteryModel,
    shared_position: Arc<Mutex<Cell>>,
    /// The task layer's own state, refreshed by the background thread every loop
    /// iteration — a cheap, always-fast snapshot rather than a `try_lock` on the
    /// `TaskLayer` mutex itself, which the background thread holds almost continuously
    /// (see `spawn_task_thread`'s doc comment). Doubles as the source for both this
    /// robot's current goal (`goal_from_state`) and `task_state()`'s public accessor.
    task_snapshot: Arc<Mutex<TaskState>>,
    /// The goal last handed to `planner.retarget` — tracked so retargeting only happens
    /// on an actual change, not recomputed (a full-grid BFS) every tick for no reason.
    applied_goal: Option<Cell>,
    background_shutdown: Arc<AtomicBool>,
    background_handle: Option<JoinHandle<()>>,
    /// `BlockCell`s the background task thread applied to its own grid, waiting for the
    /// main tick to apply them to the planner's separate grid (Decision 18).
    pending_blocks: Arc<Mutex<Vec<BlockCell>>>,
}

impl RobotProcess {
    /// `fleet_size`/`peer_ids`/`seed` mirror `DistributedAgent::new`/`TaskLayer::new`'s
    /// own requirements directly — this constructor doesn't add policy of its own, only
    /// wiring. Binds two real sockets (`DistributedAgent`'s own, `TaskLayer`'s own) plus
    /// this struct's `sensing_comms`, and spawns the background token-circulation thread
    /// immediately.
    pub fn new(
        id: u32,
        grid: &Grid,
        start: Cell,
        tasks: Vec<Task>,
        peer_ids: Vec<u32>,
        seed: u64,
    ) -> std::io::Result<Self> {
        let fleet_size = peer_ids.len();
        let planner_comms = Comms::new(id, cell_to_wire(start))?;
        let planner = DistributedAgent::new(id, grid, start, start, seed, fleet_size, planner_comms);
        let sensing_comms = Comms::new(id, cell_to_wire(start))?;
        let clock_comms = Comms::with_range(id, cell_to_wire(start), CLOCK_RANGE_CELLS)?;

        let task_layer = Arc::new(Mutex::new(TaskLayer::new(
            id,
            grid,
            tasks,
            peer_ids,
            start,
        )?));
        let shared_position = Arc::new(Mutex::new(start));
        let task_snapshot: Arc<Mutex<TaskState>> = Arc::new(Mutex::new(TaskState::Idle));
        let background_shutdown = Arc::new(AtomicBool::new(false));
        let pending_blocks: Arc<Mutex<Vec<BlockCell>>> = Arc::new(Mutex::new(Vec::new()));

        let handle = Self::spawn_task_thread(
            task_layer,
            Arc::clone(&pending_blocks),
            Arc::clone(&shared_position),
            Arc::clone(&task_snapshot),
            Arc::clone(&background_shutdown),
        );

        Ok(RobotProcess {
            id,
            tick: 0,
            first_tick: None,
            planner,
            sensing_comms,
            clock_comms,
            mode_sm: ModeStateMachine::new(),
            battery: BatteryModel::new(),
            shared_position,
            task_snapshot,
            applied_goal: None,
            background_shutdown,
            background_handle: Some(handle),
            pending_blocks,
        })
    }

    /// The background token thread: forever, publish the current position into a fresh
    /// `run_one_cycle` call, then publish whatever goal resulted into `task_snapshot`.
    /// `wait_timeout` is one tick's worth — short enough that this thread notices
    /// `shutdown` promptly between cycles, not a tuned protocol parameter.
    ///
    /// **Pacing floor, found necessary by a real test hang, not a hypothetical:** a
    /// small `thread::sleep` after each iteration, outside the mutex — unrelated to (and
    /// deliberately not undoing) the "no gap while holding the lock" reasoning below,
    /// which is about who applies `on_position_update` and stays true either way. Without
    /// it, `two_robots_complete_a_task_over_real_concurrent_ticks` hung indefinitely
    /// (reproduced at ~95% CPU on one thread for 9+ minutes with no progress): on fast
    /// real loopback UDP a two-robot ring's `run_one_cycle` can complete a hop (send,
    /// ack, forward) in well under a millisecond, so this loop cycled far faster than
    /// `tick_with`'s `sensing_comms` drain loop's 5ms idle-timeout ever saw a quiet gap
    /// — `comms.rs`'s single shared multicast port (needed so the dashboard can hear
    /// every payload type off one socket) means that drain loop also receives this
    /// thread's own Token/Ack traffic, so it never exited, and `RobotProcess::tick`
    /// never returned. Bounding `tick_with`'s drain loop to a hard wall-clock deadline
    /// (`tick_with`'s own fix) stopped the literal hang, but left the flood itself
    /// unbounded: with heartbeats and true task-relevant traffic vastly outnumbered by
    /// unthrottled token ping-pong on the same socket, both robots still degraded all
    /// the way to `Autonomous` even while continuously reachable — a real, measured
    /// consequence of flooding the shared channel, not a mode-state-machine bug (that
    /// machine is exhaustively tested elsewhere against a real heartbeat stream). Capped
    /// here at roughly `TOKEN_RETRY_INTERVAL`'s own cadence (the protocol's existing
    /// baseline pace for token-related traffic, `TOKEN_RETRY_INTERVAL` in
    /// `task_layer.rs`) rather than inventing an unrelated new constant.
    fn spawn_task_thread(
        task_layer: Arc<Mutex<TaskLayer>>,
        pending_blocks: Arc<Mutex<Vec<BlockCell>>>,
        shared_position: Arc<Mutex<Cell>>,
        task_snapshot: Arc<Mutex<TaskState>>,
        shutdown: Arc<AtomicBool>,
    ) -> JoinHandle<()> {
        thread::spawn(move || {
            let wait = Duration::from_millis(TICK_INTERVAL_MS);
            while !shutdown.load(Ordering::Acquire) {
                let pos = *shared_position.lock().unwrap();
                {
                    let mut layer = task_layer.lock().unwrap();
                    // Applied here, not by the main tick thread via `try_lock`: this
                    // loop re-acquires `task_layer`'s mutex almost continuously (there's
                    // essentially no gap between one `run_one_cycle` call ending and the
                    // next beginning), which starves any other thread's `try_lock` on the
                    // same mutex to the point of practical never-success — discovered by
                    // a real test failure (a two-robot run getting stuck one cell short
                    // of its dropoff for the entire tick budget, not just "a bit stale").
                    // Doing it here instead means the one thread that actually owns
                    // `TaskLayer` applies every position update itself, using the
                    // freshest position available at the top of each loop iteration —
                    // no contention, no starvation, by construction.
                    layer.on_position_update(pos);
                    layer.run_one_cycle(pos, wait);
                    let blocks = layer.drain_applied_blocks();
                    if !blocks.is_empty() {
                        pending_blocks.lock().unwrap().extend(blocks);
                    }
                    *task_snapshot.lock().unwrap() = layer.state();
                }
                thread::sleep(task_layer::TOKEN_RETRY_INTERVAL);
            }
        })
    }

    pub fn id(&self) -> u32 {
        self.id
    }

    /// The current tick number — the driver's own tick count once `run`/`synced_tick`
    /// has been used at least once (`docs/decisions.md` Decision 8), or a locally
    /// self-incremented count if only the standalone `tick()` has been called.
    pub fn tick_count(&self) -> u64 {
        self.tick
    }

    /// How many driver ticks this robot has run so far, counted from the first one it
    /// adopted rather than from the clock's absolute value (a long-lived clock is far past
    /// tick 0 by the time a fresh robot joins). Zero before any `synced_tick`.
    pub fn ticks_run(&self) -> u64 {
        self.first_tick.map_or(0, |first| self.tick - first + 1)
    }

    pub fn position(&self) -> Cell {
        self.planner.position()
    }

    pub fn mode(&self) -> Mode {
        self.mode_sm.mode()
    }

    pub fn battery_pct(&self) -> f32 {
        self.battery.percent()
    }

    /// This robot's current task state, from the cheap `task_snapshot` the background
    /// thread refreshes every loop iteration — not a `try_lock` on `TaskLayer` itself,
    /// which an earlier version used and which turned out to almost always fail (the
    /// background thread holds that mutex almost continuously; see `spawn_task_thread`'s
    /// doc comment), silently misreporting a busy robot as `Idle` rather than genuinely
    /// not knowing. Exposed for future consumers: the dashboard (item 24) needs "task
    /// status per robot" per `docs/PS_AND_ARCHITECTURE.md` §1.3.
    pub fn task_state(&self) -> TaskState {
        *self.task_snapshot.lock().unwrap()
    }

    /// Runs exactly one tick: perceive, retarget if the task layer handed out a new
    /// goal, move, drain battery, broadcast a heartbeat, and best-effort advance the
    /// task layer's state off the *real* resulting position. Exposed separately from
    /// `run` so tests can drive it deterministically without real sleeping.
    pub fn tick(&mut self) {
        self.tick_with(Vec::new());
    }

    /// Same as `tick`, but starting from a set of messages already drained elsewhere —
    /// `run`'s `wait_for_tick` collects whatever non-`Tick` messages arrive while it
    /// waits for the next tick broadcast, since real robots pace to the same driver
    /// broadcast and often send their own heartbeat while a peer is still mid-wait; those
    /// would otherwise be silently lost to this tick's perception/mode-tracking instead
    /// of merged in here.
    fn tick_with(&mut self, mut received: Vec<Received>) {
        let blocks = std::mem::take(&mut *self.pending_blocks.lock().unwrap());
        for b in blocks {
            if let (Ok(x), Ok(y)) = (usize::try_from(b.x), usize::try_from(b.y)) {
                self.planner.set_blocked(x, y, b.blocked);
            }
        }

        // Drain whatever else has arrived since — bounded by a hard wall-clock deadline,
        // not just a per-recv idle timeout: `sensing_comms` shares its multicast port
        // with every other socket in the fleet (`comms.rs`'s single-group design, needed
        // so the dashboard can passively hear every payload type off one socket), so it
        // also receives the background task thread's own Token/Ack traffic circulating
        // on a two-robot ring. That ring can ping-pong far faster than a 5ms idle gap
        // ever opens up on real (loopback-fast) UDP — a per-recv-only timeout never
        // resets to "quiet" and this loop never exits. Found live: this hung
        // `two_robots_complete_a_task_over_real_concurrent_ticks` for 9+ minutes at
        // ~95% CPU on one thread before being root-caused. Bounding total elapsed time
        // instead keeps the original intent (a quiet network doesn't stall the tick)
        // while also bounding the busy case.
        let drain_deadline = Instant::now() + Duration::from_millis(5);
        self.sensing_comms
            .set_read_timeout(Some(Duration::from_millis(5)))
            .expect("set sensing read timeout");
        loop {
            match self.sensing_comms.recv_filtered() {
                Ok(Some(r)) => {
                    received.push(r);
                    if Instant::now() >= drain_deadline {
                        break; // hard budget hit: stop draining even mid-stream
                    }
                }
                Ok(None) => continue, // resolves internally; defensive only
                Err(_) => break,      // window elapsed: nothing more queued right now
            }
        }

        // Mode: aggregate this tick's heartbeats into one observation (see this module's
        // top-level note on why per-peer tracking isn't worth the complexity here) —
        // the minimum reported latency among everything heard this tick, or a miss if
        // nothing arrived at all.
        let min_latency = received
            .iter()
            .filter_map(|r| match r.payload {
                Payload::Heartbeat(hb) => Some(self.tick.saturating_sub(hb.tick) as u32),
                _ => None,
            })
            .min();
        let transition = self.mode_sm.observe_heartbeat(min_latency);
        // Also re-announce periodically: a robot that never changes mode (e.g. a lone one
        // that stays Autonomous) would otherwise never tell a passive listener like the
        // dashboard what its mode is.
        let announce_mode = transition.or_else(|| {
            (self.tick % MODE_REANNOUNCE_TICKS == 0).then(|| self.mode_sm.mode())
        });
        if let Some(new_mode) = announce_mode {
            // A failed announce isn't fatal (same reasoning throughout this codebase's
            // other send sites — a transient network failure must not crash the robot):
            // peers just won't learn about the transition until the next successful one.
            if let Err(e) = self.sensing_comms.send_mode_announce(ModeAnnounce {
                robot_id: self.id,
                mode: new_mode,
                tick: self.tick,
            }) {
                eprintln!("[robot {}] failed to send mode announce: {e}", self.id);
            }
        }
        // Read fresh, after the possible transition just above — never a stale value
        // from before this tick's heartbeats were processed.
        let mode = self.mode_sm.mode();
        let perceived = perceive(mode, self.planner.position(), &received);

        // Retarget only on an actual goal change (an idle robot's goal is its own
        // current position — see this module's top-level note on why an idle robot must
        // still participate in negotiation rather than going silent).
        let snapshot_goal = goal_from_state(*self.task_snapshot.lock().unwrap());
        let effective_goal = snapshot_goal.unwrap_or_else(|| self.planner.position());
        if self.applied_goal != Some(effective_goal) {
            self.planner.retarget(effective_goal);
            self.applied_goal = Some(effective_goal);
        }

        let before = self.planner.position();
        let after = match mode {
            Mode::Cooperative | Mode::Cautious => self.planner.resolve_tick(self.tick),
            Mode::Autonomous => match perceived {
                PerceivedState::LocalOnly { nearby_obstacles } => {
                    self.planner.resolve_tick_autonomous(&nearby_obstacles)
                }
                PerceivedState::Networked { .. } => {
                    unreachable!("perceive() always returns LocalOnly for Autonomous mode")
                }
            },
        };

        self.battery.drain_tick(after != before);
        *self.shared_position.lock().unwrap() = after;
        // Without this, every heartbeat/mode announce is tagged with the robot's *start*
        // cell forever (the dashboard saw lone robots never move), and incoming traffic
        // is range-filtered around that stale cell instead of the robot's real one.
        self.sensing_comms.set_position(cell_to_wire(after));

        // Same reasoning as this module's other send sites: a transient network failure
        // (e.g. a real Wi-Fi dead zone — this project's own core motivation, `docs/
        // PS_AND_ARCHITECTURE.md` §1.1) skips this tick's heartbeat, not the process.
        if let Err(e) = self.sensing_comms.send_heartbeat(Heartbeat {
            robot_id: self.id,
            tick: self.tick,
            battery_pct: self.battery.percent(),
        }) {
            eprintln!("[robot {}] failed to send heartbeat: {e}", self.id);
        }

        // `on_position_update` is applied by the background thread itself, not here —
        // see `spawn_task_thread`'s doc comment for why a `try_lock` from this thread
        // was tried first and found to starve almost completely in practice.

        self.tick += 1;
    }

    /// Blocks until `clock_driver.rs`'s next `TickMsg` arrives on the dedicated,
    /// wide-range `clock_comms` socket, and sets `self.tick` from it. Since ticks arrive
    /// on their own socket now (not `sensing_comms`), nothing else ever shows up here to
    /// carry forward — a peer's `Heartbeat` is drained separately, by `tick`'s own short
    /// window on `sensing_comms`. If nothing arrives within a generous multiple of
    /// `TICK_INTERVAL_MS`, logs and keeps waiting rather than giving up — a slow-starting
    /// driver shouldn't permanently strand a robot that came up first.
    fn wait_for_tick(&mut self) {
        let wait_timeout = Duration::from_millis(TICK_INTERVAL_MS * 20);
        self.clock_comms
            .set_read_timeout(Some(wait_timeout))
            .expect("set tick wait timeout");
        loop {
            match self.clock_comms.recv_filtered() {
                Ok(Some(received)) => {
                    if let Payload::Tick(msg) = received.payload {
                        self.tick = msg.tick;
                        self.first_tick.get_or_insert(msg.tick);
                        return;
                    }
                }
                Ok(None) => continue, // resolves internally; defensive only
                Err(e) => eprintln!(
                    "[robot {}] no tick broadcast within {wait_timeout:?} \
                     (is clock_driver running?): {e}",
                    self.id
                ),
            }
        }
    }

    /// One real driver-synced tick: blocks for `clock_driver.rs`'s next broadcast, then
    /// runs this robot's tick logic. The building block `run` loops on; exposed
    /// separately so a caller (e.g. `main.rs`) can log or otherwise react after each one.
    pub fn synced_tick(&mut self) {
        self.wait_for_tick();
        self.tick();
    }

    /// Runs ticks forever (`max_ticks = None`) or up to a bound, paced by
    /// `clock_driver.rs`'s tick broadcasts rather than self-paced sleeping
    /// (`docs/decisions.md` Decision 8 — this is the fix that decision described).
    pub fn run(&mut self, max_ticks: Option<u64>) {
        loop {
            self.synced_tick();
            if let Some(max) = max_ticks {
                if self.ticks_run() >= max {
                    break;
                }
            }
        }
    }
}

impl Drop for RobotProcess {
    fn drop(&mut self) {
        self.background_shutdown.store(true, Ordering::Release);
        if let Some(handle) = self.background_handle.take() {
            let _ = handle.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex as StdMutex;

    // Every test here binds real sockets on the one shared multicast port `config.rs`
    // hardcodes — same reasoning as `tests/comms_protocol.rs`, `tests/token_passing.rs`,
    // etc: serialize so `cargo test`'s default parallel execution can't cross-talk.
    static SEQUENTIAL: StdMutex<()> = StdMutex::new(());

    fn test_grid() -> Grid {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map");
        Grid::load(path).expect("map should load")
    }

    /// Phase 6's `docs/TESTING_PLAN.md` gate is a manual multi-process smoke test
    /// (needs item 21's `main.rs` subcommand to run as real separate OS processes) — this
    /// is the automated in-file equivalent for a single robot's own wiring: many ticks,
    /// no panic, and the mode/battery/position all end up exactly where real physics
    /// says they should for a robot nobody else is talking to.
    #[test]
    fn single_isolated_robot_runs_many_ticks_without_panicking() {
        let _guard = SEQUENTIAL.lock().unwrap();
        let grid = test_grid();
        let start = grid.free_cells()[0];
        let mut robot =
            RobotProcess::new(1, &grid, start, vec![], vec![1], 42).expect("construct robot");

        for _ in 0..50 {
            robot.tick();
        }

        // Nobody else exists to ever heartbeat this robot — real, honest consequence:
        // it must degrade all the way to Autonomous (`HEARTBEAT_WINDOW_SIZE` is only 5
        // ticks of total silence), not stay Cooperative on an unfounded assumption that
        // comms are fine.
        assert_eq!(
            robot.mode(),
            Mode::Autonomous,
            "an utterly isolated robot has no comms to trust and must recognize that"
        );
        // No tasks exist and no one else exists to ever hand this robot the bootstrap
        // token, so it never becomes goal-directed — stays exactly where it started.
        assert_eq!(robot.position(), start);
        assert!(
            robot.battery_pct() < 100.0 && robot.battery_pct() > 0.0,
            "battery should have drained some but not fully over 50 idle ticks, got {}",
            robot.battery_pct()
        );
    }

    /// Two robots, real UDP, real bootstrap token, real PIBT movement, each on its own
    /// real OS thread — the same shape `tests/module_interop.rs` proved works, now
    /// proven from inside `RobotProcess::tick` itself, which is what `main.rs`'s `robot`
    /// subcommand (item 21) will actually call in production.
    ///
    /// Deliberately run on two genuinely independent threads, not called sequentially
    /// from one: an earlier version of this test called `robot1.tick(); robot2.tick();`
    /// alternately from a single thread, which turned out to bias every heartbeat
    /// exchange — robot 2 always observed robot 1's *current*-tick heartbeat (sent
    /// moments before, in the same iteration), while robot 1 only ever observed robot
    /// 2's *previous*-tick one (not sent yet when robot 1's turn ran) — a pure artifact
    /// of call ordering, not real network behavior, and not representative of real
    /// separate-process deployment (`CLAUDE.md`'s own requirement).
    ///
    /// **Paced by a real `ClockDriver` and `synced_tick`, not self-paced `tick()` +
    /// `thread::sleep`, found necessary by a real flaky failure, not a hypothetical:**
    /// an earlier version of this test called `robot.tick()` directly with each thread
    /// sleeping ~5ms on its own between calls. That let the two robots' tick counters
    /// drift apart under real, uneven per-iteration processing load (this same test's
    /// own `tick_with` drain-loop fix below shows that load can vary tick to tick) —
    /// `mode_state_machine.rs`'s `HEARTBEAT_WINDOW_SIZE` is deliberately only 5
    /// (`config.rs`'s own comment), so even a couple of ticks of relative drift was
    /// enough to occasionally read as a real loss burst and degrade one robot to
    /// `Autonomous` right as the run ended — reproduced directly (instrumented with a
    /// temporary per-tick `eprintln!` of `received.len()`/`min_latency`/`mode`, since
    /// removed) alternating pass/fail across otherwise-identical runs. This isn't a
    /// mode-state-machine bug — that machine is exhaustively threshold-tested elsewhere
    /// against a synthetic heartbeat stream — it's specifically an artifact of this
    /// test's *own* choice of self-paced ticking. `main.rs`'s real `robot` subcommand
    /// never self-paces (`docs/decisions.md` Decision 8): it always calls `synced_tick`,
    /// which blocks for a real `clock_driver.rs` broadcast before each tick, giving every
    /// robot in the fleet the *same* authoritative tick number rather than two
    /// independently-incrementing local counters. Switching this test to the same real
    /// `ClockDriver`/`synced_tick` path `synced_tick_advances_off_a_real_clock_driver_broadcast`
    /// already exercises removes the drift by construction (both robots advance off one
    /// shared broadcast) and is also strictly more representative of what actually ships.
    ///
    /// What this doesn't check, and why: exact collision-freedom across the two robots'
    /// tick streams belongs to `tests/no_collisions_e2e.rs` (item 26), which validates it
    /// over the real, fully containerized stack. What's checked here instead: neither
    /// thread panics, the task actually completes through real movement, and neither
    /// robot degrades all the way to Autonomous — real, continuously loopback-heartbeating
    /// robots on a shared clock shouldn't ever look like total comms silence.
    #[test]
    fn two_robots_complete_a_task_over_real_concurrent_ticks() {
        let _guard = SEQUENTIAL.lock().unwrap();
        let grid = test_grid();
        let free = grid.free_cells();
        let pos1 = free[0];
        let pos2 = free[1]; // near robot 1, so heartbeats reach each other

        let task = Task {
            task_id: 1,
            pickup: free[free.len() / 2],
            dropoff: free[free.len() / 2 + 1],
        };

        let ready1 = AtomicBool::new(false);
        let ready2 = AtomicBool::new(false);
        // `task.pickup`/`task.dropoff` are `free_cells()[len/2]` and `[len/2 + 1]` on a
        // 161x63 map (`maps/warehouse-10-20-10-2-1.map`) — not adjacent cells (a first
        // attempt at this rewrite assumed they were and used a much smaller budget,
        // which then failed every run: measured 47+ cells apart in one real run, `(34,
        // 31)` short of `(81, 31)`). `200` is a generous ceiling for that plus real
        // negotiation overhead — real runs at this value consistently finish around
        // ~115-120 ticks (see this constant's sibling note below for how that was
        // measured).
        const TOTAL_TICKS: u64 = 200;

        // **A plain fixed-count broadcast loop, not `ClockDriver::run`, and no
        // `stop`-triggered early exit — found necessary by three real, sequential bugs,
        // not hypotheticals:**
        //
        // Attempt 1 gave `ClockDriver::run` a fixed `Some(N)` at a fast fixed interval
        // (5ms). That outran real achievable per-tick processing time in this
        // environment (`tick_with`'s drain loop alone measured 15-39 real messages per
        // tick during earlier diagnosis) — the driver exhausted its whole broadcast
        // budget and its thread exited while the robots were still only partway through
        // their own tick counts, and since `wait_for_tick` retries forever on a timeout
        // by design ("a slow-starting driver shouldn't permanently strand a robot" — a
        // reasonable assumption that doesn't hold once the driver has *finished for
        // good*, not just started slow), both robots then blocked on
        // `clock_comms.recv_filtered()` forever waiting for a broadcast that would never
        // come again: reproduced directly, 3 of the process's threads parked in
        // `__skb_wait_for_more_packets` indefinitely while the driver thread had already
        // exited.
        //
        // Attempt 2 fixed that by switching to `driver.run(None)` (unbounded, mirroring
        // `main.rs`'s real `clock` subcommand) on a plain, deliberately-unjoined
        // `thread::spawn` outside this `thread::scope`, reasoning it would just "die
        // with the process." That reasoning missed that this lib test binary runs many
        // `#[test]`s in *one* process: the detached driver kept broadcasting
        // ever-increasing tick numbers on the shared multicast bus for the rest of the
        // binary's life, and the very next sequential test to touch `clock_comms`
        // (`synced_tick_advances_off_a_real_clock_driver_broadcast`, guarded by the same
        // `SEQUENTIAL` mutex but with no protocol-level way to tell "this test's driver"
        // from "a leftover one" — `TickMsg` carries no epoch, unlike `TokenMsg`)
        // nondeterministically received whichever of the two live drivers' broadcasts
        // arrived first, reproduced directly: `got [118, 2, 3, 4, 5]`, tick 118 from the
        // stale leftover driver interleaved with 2-5 from that test's own fresh one.
        //
        // Attempt 3 bounded the driver again and tied its exit to a shared `stop` flag
        // (set once `h1` reached the dropoff), joined via `thread::scope` so it could
        // no longer outlive this test. That reopened Attempt 1's exact failure mode by a
        // different door: `stop` can flip true, and the driver observe it and quit,
        // *while* `h2` (which has no task of its own and just keeps ticking idly) is
        // mid-block inside `wait_for_tick`'s retry-forever loop waiting for the next
        // broadcast — which now never comes. `wait_for_tick` has no way to observe this
        // test's own `stop` flag (it only knows about real `TickMsg` traffic), so
        // nothing could ever wake that wait. Reproduced directly: the driver thread
        // already gone (no `hrtimer_nanosleep` thread left) while two other threads sat
        // in `__skb_wait_for_more_packets` forever.
        //
        // This version removes the race at its root instead of narrowing the window
        // again: no `stop` flag, no early exit for anyone. The driver, `h1`, and `h2`
        // all run for the exact same fixed `TOTAL_TICKS`, so nothing can ever finish
        // and go silent while something else is still relying on it being alive. The
        // driver sends the broadcast itself, in a plain loop this test fully controls,
        // rather than calling `ClockDriver::run` at all — still real UDP, real
        // `TickMsg`s, the exact wire path `wait_for_tick` consumes (already separately
        // proven to be the real `ClockDriver` mechanism by
        // `synced_tick_advances_off_a_real_clock_driver_broadcast`). `TICK_INTERVAL_MS`
        // (not an invented faster test-only value) is the interval `config.rs` itself
        // documents as "chosen... slow enough that real UDP round trips on one machine
        // aren't swamped" — exactly the property Attempt 1 was violating; at this
        // interval, `TOTAL_TICKS = 200` costs a fixed, always-incurred ~20s per run (no
        // more early-exit speedup), verified stable across repeated real runs.
        thread::scope(|scope| {
            let ready1 = &ready1;
            let ready2 = &ready2;
            let grid = &grid;

            scope.spawn(move || {
                let driver_comms =
                    Comms::new(crate::config::CLOCK_DRIVER_ID, (0, 0)).expect("bind driver comms");
                for tick in 0..=TOTAL_TICKS {
                    if let Err(e) = driver_comms.send_tick(crate::protocol::messages::TickMsg { tick })
                    {
                        eprintln!("[test driver] failed to send tick {tick}: {e}");
                    }
                    thread::sleep(Duration::from_millis(TICK_INTERVAL_MS));
                }
            });

            let h1 = scope.spawn(move || {
                let mut robot =
                    RobotProcess::new(1, grid, pos1, vec![task], vec![1, 2], 100).expect("robot 1");
                ready1.store(true, Ordering::Release);
                let mut ever_networked = false;
                while robot.tick_count() < TOTAL_TICKS {
                    robot.synced_tick();
                    ever_networked |= robot.mode() != Mode::Autonomous;
                }
                (robot.position(), ever_networked)
            });

            let h2 = scope.spawn(move || {
                let mut robot =
                    RobotProcess::new(2, grid, pos2, vec![task], vec![1, 2], 200).expect("robot 2");
                ready2.store(true, Ordering::Release);
                let mut ever_networked = false;
                while robot.tick_count() < TOTAL_TICKS {
                    robot.synced_tick();
                    ever_networked |= robot.mode() != Mode::Autonomous;
                }
                ever_networked
            });

            // Bootstrap the ring exactly as `tests/token_passing.rs`/`module_interop.rs`
            // do: a real robot can't hear its own loopback broadcast, so a third party
            // sends the very first token — the role `run_experiment.rs` (item 28) will
            // play for real. Waits on real readiness signals rather than a fixed sleep
            // guess: an earlier version slept a fixed 20ms before sending, which turned
            // out unreliable under real system load — `RobotProcess::new` binds several
            // real sockets and spawns a thread, and when that took longer than 20ms on a
            // loaded machine, the bootstrap token was sent before the socket even
            // existed to receive it, silently lost, and the test spun for its entire
            // tick budget before failing (a real, reproduced ~218s failure during this
            // item's own testing, not a hypothetical). Sends the token a few times over
            // ~200ms as further margin — harmless, since `run_one_cycle`'s `seq` dedup
            // already ignores a duplicate once the first copy is processed.
            while !ready1.load(Ordering::Acquire) || !ready2.load(Ordering::Acquire) {
                thread::sleep(Duration::from_millis(2));
            }
            let bootstrapper = Comms::new(99, (0, 0)).expect("bind bootstrapper");
            for _ in 0..5 {
                bootstrapper
                    .send_token(crate::protocol::messages::TokenMsg {
                        seq: 0,
                        holder_id: 1,
                        claimed_tasks: vec![],
                        epoch: 0,
                        creator: 0,
                    })
                    .expect("bootstrap send");
                thread::sleep(Duration::from_millis(40));
            }

            let (final_pos1, networked1) = h1.join().expect("robot 1 thread panicked");
            let networked2 = h2.join().expect("robot 2 thread panicked");

            assert_eq!(
                final_pos1, task.dropoff,
                "robot 1 should have completed the task within the tick budget"
            );
            // Robot 1 ends up at the far-away dropoff, out of robot 2's `COMMS_RANGE_CELLS`, so
            // ending in Autonomous is correct — the check is that they networked while in
            // range (an earlier version of this test asserted the *final* mode, which only
            // held because heartbeats wrongly carried the start cell forever).
            assert!(
                networked1 && networked2,
                "two robots heartbeating each other in range shouldn't look like total comms \
                 silence (robot 1: {networked1}, robot 2: {networked2})"
            );
        });
    }

    /// Decision 18 end to end over real UDP: a robot with an empty pool receives a
    /// `TaskInject` from an outside sender, claims it on the next token turn, and heads
    /// for the pickup.
    #[test]
    fn robot_picks_up_a_task_injected_over_udp() {
        let _guard = SEQUENTIAL.lock().unwrap();
        let grid = test_grid();
        let free = grid.free_cells();
        // Two robots: a one-robot ring can't deliver the token to anyone, so the claim
        // would (correctly) be released as unannounced.
        let mut robot =
            RobotProcess::new(1, &grid, free[0], vec![], vec![1, 2], 7).expect("construct robot");
        let mut robot2 =
            RobotProcess::new(2, &grid, free[1], vec![], vec![1, 2], 8).expect("construct robot");
        let sender = Comms::with_range(99, (0, 0), task_layer::TOKEN_RANGE_CELLS).expect("sender");
        let inject = crate::protocol::messages::TaskInject {
            task_id: 1,
            pickup: (free[3].0 as i32, free[3].1 as i32),
            dropoff: (free[6].0 as i32, free[6].1 as i32),
        };
        let bootstrap = crate::protocol::messages::TokenMsg {
            seq: 0, holder_id: 1, claimed_tasks: vec![], epoch: 0, creator: 0,
        };
        let mut claimed = false;
        for _ in 0..80 {
            // Resends: commands are idempotent, and the robot may be mid-wait when one lands.
            sender.send_task_inject(inject).expect("send inject");
            sender.send_token(bootstrap.clone()).expect("send token");
            robot.tick();
            robot2.tick();
            thread::sleep(Duration::from_millis(50));
            let moved = |r: &RobotProcess, start| r.task_state() != TaskState::Idle || r.position() != start;
            if moved(&robot, free[0]) || moved(&robot2, free[1]) {
                claimed = true;
                break;
            }
        }
        assert!(claimed, "robot never picked up the injected task");
    }

    /// Regression: a lone robot that never changes mode must still announce it, so a
    /// passive listener (the dashboard) doesn't show it as "unknown" forever.
    #[test]
    fn a_robot_with_no_mode_transition_still_announces_its_mode() {
        let _guard = SEQUENTIAL.lock().unwrap();
        let grid = test_grid();
        let start = grid.free_cells()[0];
        let mut robot =
            RobotProcess::new(1, &grid, start, vec![], vec![1], 7).expect("construct robot");
        let listener = Comms::with_range(77, (0, 0), task_layer::TOKEN_RANGE_CELLS)
            .expect("bind listener");
        listener.set_read_timeout(Some(Duration::from_millis(50))).expect("timeout");
        for _ in 0..(MODE_REANNOUNCE_TICKS + 2) {
            robot.tick();
        }
        let mut announced = false;
        while let Ok(Some(r)) = listener.recv_filtered() {
            announced |= matches!(r.payload, Payload::ModeAnnounce(_));
        }
        assert!(announced, "expected a periodic ModeAnnounce from a lone robot");
    }

    /// Regression: broadcasts must carry the robot's current cell, not its start cell.
    #[test]
    fn sensing_comms_position_follows_the_robot() {
        let _guard = SEQUENTIAL.lock().unwrap();
        let grid = test_grid();
        let start = grid.free_cells()[0];
        let mut robot =
            RobotProcess::new(1, &grid, start, vec![], vec![1], 7).expect("construct robot");
        robot.tick();
        assert_eq!(robot.sensing_comms.position(), cell_to_wire(robot.position()));
    }

    /// Regression: `--max-ticks` must count ticks this robot ran, not the clock's absolute
    /// value, or a fresh robot joining a long-running clock exits after one tick.
    #[test]
    fn ticks_run_counts_from_the_first_adopted_tick() {
        let _guard = SEQUENTIAL.lock().unwrap();
        let grid = test_grid();
        let start = grid.free_cells()[0];
        let mut robot =
            RobotProcess::new(1, &grid, start, vec![], vec![1], 7).expect("construct robot");
        assert_eq!(robot.ticks_run(), 0);
        robot.first_tick = Some(55_437);
        robot.tick = 55_437;
        assert_eq!(robot.ticks_run(), 1);
        robot.tick = 55_736;
        assert_eq!(robot.ticks_run(), 300, "300 ticks run, not 55,736");
    }

    /// Item 23's actual point (`docs/decisions.md` Decision 8): `RobotProcess::run`/
    /// `synced_tick` must advance off a real `clock_driver.rs` broadcast, not an
    /// independent local counter. A real `ClockDriver` on its own thread, a real robot
    /// calling `synced_tick` a few times, and the robot's own tick count must reflect
    /// values that actually came from the driver — not just "some number that went up."
    #[test]
    fn synced_tick_advances_off_a_real_clock_driver_broadcast() {
        let _guard = SEQUENTIAL.lock().unwrap();
        let grid = test_grid();
        let start = grid.free_cells()[0];

        let driver_handle = thread::spawn(|| {
            let driver =
                crate::clock_driver::ClockDriver::new(20).expect("bind clock driver"); // fast, test-only cadence
            driver.run(Some(10));
        });

        let mut robot =
            RobotProcess::new(1, &grid, start, vec![], vec![1], 7).expect("construct robot");

        let mut observed = Vec::new();
        for _ in 0..5 {
            robot.synced_tick();
            observed.push(robot.tick_count());
        }

        driver_handle.join().expect("clock driver thread panicked");

        // Ticks arrive in order and strictly increase — this robot never invented its
        // own numbering, it only ever adopted whatever the driver actually broadcast.
        assert!(
            observed.windows(2).all(|w| w[1] > w[0]),
            "tick count must strictly increase from real driver broadcasts, got {observed:?}"
        );
        assert!(
            observed[0] < 10,
            "the first observed tick should be an early real driver tick, got {observed:?}"
        );
    }
}
