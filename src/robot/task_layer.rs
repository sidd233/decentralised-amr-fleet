//! Task allocation: lifelong-MAPF token passing (Ma et al. AAMAS'17 Algorithm 1 — see
//! `docs/REFERENCES.md` for the exact pseudocode this ports). Items 15-17 of
//! `docs/BUILD_PLAN.md`, `docs/PS_AND_ARCHITECTURE.md` §3.4/§3.2's Task Layer.
//!
//! Same architectural split as `planner_pibt.rs`: a pure decision core (`handle_token`,
//! no I/O, directly unit-testable) plus a thin real-UDP wrapper (`run_one_cycle`) built
//! on `robot::comms::Comms` and `protocol::messages::TokenMsg`. Unlike the planner at
//! item 9, `comms.rs` (item 11) already existed when this module was written, so there's
//! no single-process-only stage to build first and convert later — this goes straight to
//! real UDP.
//!
//! One deliberate deviation from Algorithm 1: the paper passes the token to "a random
//! free agent"; this uses a fixed round-robin ring (`peer_ids`) instead, so
//! `docs/TESTING_PLAN.md`'s Phase 5 "reaches every idle robot in bounded time" gate is
//! provable exactly (bounded by fleet size) rather than probabilistically, and so seeded
//! benchmark runs (Phase 8) stay reproducible without needing a second RNG stream here.
//!
//! Item 17's re-pooling is folded directly into `handle_token` rather than being a
//! separate mechanism: whichever robot currently holds the token also re-validates its
//! *own* current task's reachability (using its own locally up-to-date `Grid`, which its
//! own perception has applied `set_blocked` calls to) before deciding whether to claim a
//! fresh one. This keeps `claimed_tasks` — the single source of truth for what's taken —
//! mutated only by whoever legitimately holds the token at that instant, with no separate
//! out-of-band release message needed. The tradeoff: a robot whose task just became
//! unreachable only actually releases it the next time the token reaches it, bounded by
//! the same ring-size bound as "no starvation" above — acceptable at this project's scale
//! and logged here rather than silently assumed instantaneous.
//!
//! **Fix 1 (`docs/decisions.md`'s explicit-ack decision entry): the retry/ack design
//! below replaced an earlier one that used "a higher `seq` observed on the wire" as the
//! only proof of delivery, sampling one arbitrary queued packet per retry attempt.**
//! Reproduced and root-caused live against the real Docker stack and real host-process
//! fleets: under load, that implicit signal gets buried under everyone else's own stale
//! retries (a positive-feedback congestion collapse — 99.7% of all wire traffic was
//! `TokenMsg`, every single hop exhausting its full retry budget), which was *also* the
//! proximate cause of the clock's `TickMsg` broadcasts almost never reaching
//! `RobotProcess::wait_for_tick` in time. This version uses an explicit `Ack`, a
//! draining (not sampling) wait, doubling backoff with a bounded budget, and an
//! unreachable-peer skip so a dead peer costs the fleet once, not forever.
//!
//! **Epoch design (`docs/decisions.md`'s epoch decision entry): a bare `seq` watermark
//! cannot tell two independently-numbered branches of the same ancestor apart — the
//! Step 1B fork proved a stale resend and a real forward can both survive indefinitely,
//! each incrementing `seq` on its own, doubling live traffic with no way to converge.**
//! `TokenMsg::epoch`/`creator` gives every lineage a totally-ordered identity; a
//! `deliver_with_backoff` proof is now also sender-checked (a robot only trusts progress
//! reported by the peer it actually addressed, not ambient higher-`seq` traffic from an
//! unrelated lineage — closing a real false-positive-ack finding from real-transport
//! testing). A watchdog regenerates the token if it's been silent long enough that it may
//! be genuinely lost (Amendment C).

use std::collections::{HashMap, HashSet};
use std::io;
use std::time::{Duration, Instant};

use crate::protocol::messages::{
    Ack, BlockCell, ClaimEntry, TaskInject, TaskRetarget, TokenMsg,
};
use crate::robot::comms::{Comms, Payload, Received};
use crate::robot::planner_pibt::Cell;
use crate::world::grid::Grid;

/// One pickup-and-deliver job. The full set of tasks for a run is static and known to
/// every robot up front (same "given, like the map" treatment as the map itself) — only
/// *claims* on tasks circulate, via `TokenMsg::claimed_tasks`, not task definitions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Task {
    pub task_id: u32,
    pub pickup: Cell,
    pub dropoff: Cell,
}

/// `docs/PS_AND_ARCHITECTURE.md` §3.2: "idle -> claim task -> pickup -> dropoff."
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaskState {
    Idle,
    ToPickup(Task),
    ToDropoff(Task),
}

/// The result of one `run_one_cycle` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TokenCycleOutcome {
    /// No `TokenMsg` naming this robot as holder arrived within the caller's
    /// `wait_timeout`, and the watchdog didn't fire either. Not necessarily an error — a
    /// robot with a long-running task may simply not have been due a turn yet.
    NoTokenArrived,
    /// The token was received, this robot's turn was processed, and the outgoing token
    /// was delivered (acked, or confirmed via the target's own higher-`seq`/`epoch`
    /// forward) to some peer — not necessarily the ring's immediate next one if a skip
    /// happened first this cycle.
    Handled { claimed: Option<Task> },
    /// This robot's turn was processed, but delivering the outgoing token required
    /// marking `unreachable_peer` unreachable and re-targeting the *same* `seq` (a new
    /// `epoch`, though — see the epoch decision entry) at the next reachable peer after
    /// its backoff budget was exhausted with no ack observed. `TaskLayer::skip_count` is
    /// the cumulative counter this increments, once per skip event.
    Skipped { unreachable_peer: u32 },
    /// No token of *any* epoch had been observed for the watchdog timeout, and this
    /// robot was the lowest id among itself and whoever it currently hears heartbeats
    /// from, so it regenerated the token from scratch (Amendment C) — a new `epoch` and
    /// `seq`, empty claims (owners re-assert their own on their next turn).
    Regenerated { claimed: Option<Task> },
}

/// Base retry interval and doubling backoff cap (Amendment B): a hop's total delivery
/// budget is `20 + 40 + 80 + 160 = 300ms` before its target is marked unreachable and
/// skipped — replaces the original fixed-count `MAX_TOKEN_RETRIES` (50 sends, no pacing
/// at all between them under load, root of the Fix 1a flood).
const TOKEN_RETRY_BACKOFF_MS: [u64; 4] = [20, 40, 80, 160];

/// Sum of `TOKEN_RETRY_BACKOFF_MS` — the worst-case wall-clock cost of one hop that ends
/// in a skip. Used to derive both the watchdog timeout and how long an `unreachable`
/// mark is trusted before it expires on its own (see each's own doc comment).
const TOKEN_SKIP_BUDGET_MS: u64 = TOKEN_RETRY_BACKOFF_MS[0]
    + TOKEN_RETRY_BACKOFF_MS[1]
    + TOKEN_RETRY_BACKOFF_MS[2]
    + TOKEN_RETRY_BACKOFF_MS[3];

/// How many ring circulations an `unreachable` mark is trusted for before it expires on
/// its own, even with no `Heartbeat` ever clearing it (Amendment B's follow-up: "a live
/// peer is never excluded forever"). Three, not one: a single circulation's worth of
/// margin risks re-marking a peer that was genuinely just slow for one lap.
const UNREACHABLE_EXPIRY_CIRCULATIONS: u32 = 3;

/// `bootstrap-token`'s own resend cadence (`main.rs`, Decision 13) — unrelated to the
/// backoff schedule above, kept as the simple fixed interval it always was, since that
/// short-lived out-of-ring process only ever sends the fleet's very first token a
/// handful of times, never enters this module's own retry/ack loop.
///
/// `pub` (not just `pub(crate)`) since Decision 13: `src/main.rs` is a separate crate
/// from this `lib.rs`-rooted one (`sih26123`'s bin/lib split), so its `bootstrap-token`
/// subcommand needs real crate-external visibility to reuse this same cadence, rather
/// than duplicating the magic number.
pub const TOKEN_RETRY_INTERVAL: Duration = Duration::from_millis(20);

/// Range-scoping radius for this module's own `Comms` instance. Token circulation is
/// fleet-wide by design (`docs/PS_AND_ARCHITECTURE.md` §3.4) — unlike `PoseIntent`'s
/// deliberate "nearby robots only" scoping (§3.2) — so this is set far larger than any
/// map this project uses. Not literally `u32::MAX`: `Comms::recv_filtered` squares the
/// range as `i64`, and `u32::MAX` squared overflows `i64::MAX`.
///
/// `pub` for the same cross-crate reason as `TOKEN_RETRY_INTERVAL` above: `main.rs`'s
/// `bootstrap-token` subcommand binds its own `Comms` at this same wide range, since it
/// also puts a `TokenMsg` on the fleet-wide bus.
pub const TOKEN_RANGE_CELLS: u32 = 100_000;

fn cell_to_wire(c: Cell) -> (i32, i32) {
    (c.0 as i32, c.1 as i32)
}

/// `true` for the payload types `run_one_cycle`'s waits must forward to the caller as
/// incidentals rather than silently discard — the exact bug Fix 1a found starving
/// `wait_for_tick`. Stray `Token`/`Ack` traffic that doesn't match what a given wait is
/// looking for is protocol-internal noise with no handler outside this module, so it's
/// dropped rather than forwarded.
fn is_forwardable(payload: &Payload) -> bool {
    matches!(payload, Payload::Tick(_) | Payload::Pose(_) | Payload::Heartbeat(_))
}

/// `true` if lineage `a` (`(epoch, creator)`) should be preferred over lineage `b`: a
/// strictly higher `epoch` always wins; at equal `epoch`, the lower `creator` wins (same
/// tie-break direction as claim reconciliation's lower-`robot_id`-wins convention, for
/// consistency). Total order over `(u32, u32)` pairs — exactly one side dominates the
/// other whenever they differ, so independently-applied comparisons always agree.
fn epoch_dominates(a: (u32, u32), b: (u32, u32)) -> bool {
    a.0 > b.0 || (a.0 == b.0 && a.1 < b.1)
}

/// One robot's task-allocation state: current job (if any), the known task pool, the
/// token-passing ring order, and its own dedicated `Comms` channel (see
/// `TOKEN_RANGE_CELLS` for why this is a separate instance from the planner's).
pub struct TaskLayer {
    robot_id: u32,
    grid: Grid,
    tasks: Vec<Task>,
    /// The fleet's token-passing order, sorted ascending, identical on every robot.
    /// Must contain `robot_id`.
    peer_ids: Vec<u32>,
    state: TaskState,
    /// `None` only for a `new_pure`-constructed instance (item 27's benchmark harness,
    /// `docs/decisions.md` Decision 15): `handle_token` — the only thing a pure,
    /// in-process simulation needs — never touches this field, so paying for a real
    /// socket bind per simulated robot across a sweep of many scenarios would be pure
    /// waste. `run_one_cycle` is the one method that needs it for real, and panics via
    /// `comms()` below if called on a pure instance — a genuine programming error, not a
    /// real runtime condition, same category as this file's other `.expect()`s.
    comms: Option<Comms>,
    /// Highest `(epoch, creator)` lineage this robot has ever adopted (epoch decision
    /// entry, `docs/decisions.md`): a token from a dominated lineage is dropped
    /// silently, regardless of its `seq`; a token from a strictly dominating lineage is
    /// always adopted as the new baseline, resetting `highest_seen_seq`.
    highest_seen_epoch: Option<(u32, u32)>,
    /// Highest `TokenMsg::seq` this robot has accepted as a genuinely new turn *within
    /// its current lineage* (Amendment A): a re-delivery exactly equal to it is re-acked
    /// without reprocessing (the sender's own retry, still in flight), and anything
    /// strictly older is dropped outright. Reset whenever a new lineage is adopted.
    highest_seen_seq: Option<u64>,
    /// Peers this robot has stopped attempting to deliver the token to directly, with
    /// the time each was marked (Amendment B). Cleared immediately on a `Heartbeat` from
    /// that peer, or after `UNREACHABLE_EXPIRY_CIRCULATIONS` worth of wall-clock time
    /// even with no `Heartbeat` at all — "a live peer is never excluded forever."
    unreachable: HashMap<u32, Instant>,
    /// Wall-clock time each peer's `Heartbeat` was last incidentally observed — the
    /// watchdog's "who I currently hear from" signal (Amendment C). Same imperfect
    /// visibility `unreachable`-clearing already accepts: only whatever lands on this
    /// module's own socket via `SO_REUSEPORT`, not every `Heartbeat` in the fleet.
    heartbeat_last_seen: HashMap<u32, Instant>,
    /// Wall-clock time a `TokenMsg` of *any* lineage, addressed to anyone, was last
    /// observed on the wire — the watchdog's "is the token alive somewhere" signal.
    last_token_seen: Instant,
    /// Cumulative count of skip events (a peer newly marked unreachable), once per
    /// event, not once per retry attempt — `docs/decisions.md`'s Fix 1 entry and Fix
    /// 1d's traffic measurement both read this.
    skip_count: u32,
    /// Cumulative count of claim-conflict resolutions (`docs/decisions.md`'s
    /// claim-reconciliation decision entry): once per `handle_token` call where this
    /// robot's own committed task and the incoming token's recorded claimant disagreed,
    /// whichever way the tie-break went.
    claim_conflicts: u32,
    /// Cumulative count of epoch bumps this robot has originated — either a skip-resend
    /// (Amendment A: same `seq`, new `epoch`) or a watchdog regeneration (new `seq` and
    /// `epoch`). `docs/decisions.md`'s epoch entry and Fix 1d's traffic measurement both
    /// read this.
    epoch_bumps: u32,
    /// Cumulative count of tokens dropped silently because their lineage was strictly
    /// dominated by one already adopted — the direct, measured cost of a fork that
    /// didn't win.
    lineages_dropped: u32,
    /// Cumulative count of a real, found gap (`docs/decisions.md`'s
    /// unannounced-claim-release entry): a task `handle_token` just freshly claimed
    /// (idle -> claimed this same cycle), whose resulting outgoing token then failed to
    /// reach *every* peer in the ring within this same `run_one_cycle` call — the whole
    /// point of the claim-conflict reconciliation logic above is that it fires on the
    /// *receiving* side of `handle_token`, but a claim that never successfully leaves
    /// this robot never puts itself in front of anyone to reconcile against, so it could
    /// otherwise stand forever unreconciled alongside another robot's legitimately
    /// circulated claim on the same task. Released back to `Idle` rather than kept —
    /// see `run_one_cycle`'s own comment at the release site for the full account of
    /// how this was found (a real, 100%-reproducible double-commitment in
    /// `tests/token_passing.rs`'s `real_transport_fork_reproduces_via_actual_skip_path_50_trials`).
    unannounced_claims_released: u32,
    /// Task ids cancelled by an operator command (Decision 18). `handle_token` strips any
    /// claim on them from the token so a cancelled task doesn't linger as somebody's
    /// "current task" on the wire (a *completed* task's claim must stay, or it would be
    /// re-claimed — cancelled ones are removed from `tasks`, so they never are).
    cancelled: HashSet<u32>,
    /// `BlockCell` commands applied to this layer's own grid since the last drain — the
    /// robot's planner keeps a separate `Grid`, so `RobotProcess` drains these and applies
    /// them there too.
    applied_blocks: Vec<BlockCell>,
}

impl TaskLayer {
    pub fn new(
        robot_id: u32,
        grid: &Grid,
        tasks: Vec<Task>,
        peer_ids: Vec<u32>,
        own_position: Cell,
    ) -> io::Result<Self> {
        assert!(
            peer_ids.contains(&robot_id),
            "peer_ids must include this robot's own id"
        );
        let comms = Comms::with_range(robot_id, cell_to_wire(own_position), TOKEN_RANGE_CELLS)?;
        Ok(TaskLayer {
            robot_id,
            grid: grid.clone(),
            tasks,
            peer_ids,
            state: TaskState::Idle,
            comms: Some(comms),
            highest_seen_epoch: None,
            highest_seen_seq: None,
            unreachable: HashMap::new(),
            heartbeat_last_seen: HashMap::new(),
            last_token_seen: Instant::now(),
            skip_count: 0,
            claim_conflicts: 0,
            epoch_bumps: 0,
            lineages_dropped: 0,
            unannounced_claims_released: 0,
            cancelled: HashSet::new(),
            applied_blocks: Vec::new(),
        })
    }

    /// A `TaskLayer` with no real socket at all — for a pure, in-process benchmark
    /// simulation (item 27, Decision 15) driving `handle_token` directly, many times
    /// over, with no wire traffic involved. `run_one_cycle` is unusable on the result;
    /// everything `handle_token` actually needs (`grid`/`tasks`/`peer_ids`/`state`) works
    /// identically either way. `unreachable` stays permanently empty here — a pure
    /// simulation never observes a `Heartbeat` or a backoff timeout, so `handle_token`'s
    /// peer selection reduces to the plain ring order, unchanged from before Fix 1.
    pub fn new_pure(robot_id: u32, grid: &Grid, tasks: Vec<Task>, peer_ids: Vec<u32>) -> Self {
        assert!(
            peer_ids.contains(&robot_id),
            "peer_ids must include this robot's own id"
        );
        TaskLayer {
            robot_id,
            grid: grid.clone(),
            tasks,
            peer_ids,
            state: TaskState::Idle,
            comms: None,
            highest_seen_epoch: None,
            highest_seen_seq: None,
            unreachable: HashMap::new(),
            heartbeat_last_seen: HashMap::new(),
            last_token_seen: Instant::now(),
            skip_count: 0,
            claim_conflicts: 0,
            epoch_bumps: 0,
            lineages_dropped: 0,
            unannounced_claims_released: 0,
            cancelled: HashSet::new(),
            applied_blocks: Vec::new(),
        }
    }

    fn comms(&self) -> &Comms {
        self.comms
            .as_ref()
            .expect("run_one_cycle needs a real TaskLayer::new instance, not new_pure")
    }

    pub fn state(&self) -> TaskState {
        self.state
    }

    pub fn robot_id(&self) -> u32 {
        self.robot_id
    }

    /// Cumulative skip-event count (Amendment B) — see the field's own doc comment.
    pub fn skip_count(&self) -> u32 {
        self.skip_count
    }

    /// Cumulative claim-conflict count — see the field's own doc comment.
    pub fn claim_conflicts(&self) -> u32 {
        self.claim_conflicts
    }

    /// Cumulative epoch-bump count (skip-resends + watchdog regenerations combined) —
    /// see the field's own doc comment.
    pub fn epoch_bumps(&self) -> u32 {
        self.epoch_bumps
    }

    /// Cumulative count of tokens dropped for belonging to a dominated lineage — see the
    /// field's own doc comment.
    pub fn lineages_dropped(&self) -> u32 {
        self.lineages_dropped
    }

    /// Cumulative count of freshly-claimed tasks released back to `Idle` because the
    /// claim never reached any peer — see the field's own doc comment.
    pub fn unannounced_claims_released(&self) -> u32 {
        self.unannounced_claims_released
    }

    /// The cell the planner should currently be steering toward: the task's pickup
    /// location if still `ToPickup`, its dropoff if `ToDropoff`, or `None` if idle.
    pub fn current_goal(&self) -> Option<Cell> {
        match self.state {
            TaskState::Idle => None,
            TaskState::ToPickup(t) => Some(t.pickup),
            TaskState::ToDropoff(t) => Some(t.dropoff),
        }
    }

    /// Call whenever this robot's actual position changes, to detect arrival at the
    /// current waypoint: `ToPickup` -> `ToDropoff` on reaching pickup, `ToDropoff` ->
    /// `Idle` (task complete) on reaching dropoff.
    pub fn on_position_update(&mut self, pos: Cell) {
        self.state = match self.state {
            TaskState::ToPickup(t) if pos == t.pickup => TaskState::ToDropoff(t),
            TaskState::ToDropoff(t) if pos == t.dropoff => TaskState::Idle,
            other => other,
        };
    }

    /// Marks a cell blocked/free in this robot's own local view of the map — e.g. when
    /// its perception (item 18) detects a newly blocked aisle. `handle_token`'s
    /// reachability check (item 17) reads from this same `Grid`, so calling this is what
    /// actually makes a stale task's pickup/dropoff "unreachable" from this robot's own
    /// point of view the next time it holds the token.
    pub fn set_blocked(&mut self, x: usize, y: usize, blocked: bool) {
        self.grid.set_blocked(x, y, blocked);
    }

    fn wire_cell(&self, c: (i32, i32)) -> Option<Cell> {
        let (x, y) = (usize::try_from(c.0).ok()?, usize::try_from(c.1).ok()?);
        self.grid.is_free(x, y).then_some((x, y))
    }

    /// Operator command (Decision 18): add a job to this robot's pool. Idempotent by
    /// `task_id`; ignored if either cell is out of bounds or blocked. Returns whether it
    /// was added.
    pub fn apply_task_inject(&mut self, m: TaskInject) -> bool {
        if self.tasks.iter().any(|t| t.task_id == m.task_id) || self.cancelled.contains(&m.task_id) {
            return false;
        }
        let (Some(pickup), Some(dropoff)) = (self.wire_cell(m.pickup), self.wire_cell(m.dropoff))
        else {
            return false;
        };
        self.tasks.push(Task { task_id: m.task_id, pickup, dropoff });
        true
    }

    /// Operator command (Decision 18): change an existing task's cells, or cancel it. A
    /// task this robot is currently working on is updated in place (keeping its
    /// pickup/dropoff phase) so its goal moves on the next tick; a cancelled one sends the
    /// robot back to `Idle`. Idempotent — applying the same command twice changes nothing.
    pub fn apply_task_retarget(&mut self, m: TaskRetarget) {
        if m.cancel {
            self.tasks.retain(|t| t.task_id != m.task_id);
            self.cancelled.insert(m.task_id);
            if self.current_task().is_some_and(|t| t.task_id == m.task_id) {
                self.state = TaskState::Idle;
            }
            return;
        }
        let (Some(pickup), Some(dropoff)) = (self.wire_cell(m.pickup), self.wire_cell(m.dropoff))
        else {
            return;
        };
        let Some(task) = self.tasks.iter_mut().find(|t| t.task_id == m.task_id) else {
            return;
        };
        task.pickup = pickup;
        task.dropoff = dropoff;
        let updated = *task;
        self.state = match self.state {
            TaskState::ToPickup(t) if t.task_id == m.task_id => TaskState::ToPickup(updated),
            TaskState::ToDropoff(t) if t.task_id == m.task_id => TaskState::ToDropoff(updated),
            other => other,
        };
    }

    /// Operator command (Decision 18): block/unblock a map cell in this robot's own view.
    /// `handle_token`'s reachability check then releases any task it makes unreachable.
    pub fn apply_block_cell(&mut self, m: BlockCell) {
        let (Ok(x), Ok(y)) = (usize::try_from(m.x), usize::try_from(m.y)) else {
            return;
        };
        self.set_blocked(x, y, m.blocked);
        self.applied_blocks.push(m);
    }

    /// Hands back (and clears) the `BlockCell`s applied since the last call.
    pub fn drain_applied_blocks(&mut self) -> Vec<BlockCell> {
        std::mem::take(&mut self.applied_blocks)
    }

    fn apply_command(&mut self, payload: &Payload) {
        match payload {
            Payload::TaskInject(m) => {
                self.apply_task_inject(*m);
            }
            Payload::TaskRetarget(m) => self.apply_task_retarget(*m),
            Payload::BlockCell(m) => self.apply_block_cell(*m),
            _ => {}
        }
    }

    fn current_task(&self) -> Option<Task> {
        match self.state {
            TaskState::Idle => None,
            TaskState::ToPickup(t) | TaskState::ToDropoff(t) => Some(t),
        }
    }

    fn next_waypoint(&self) -> Cell {
        match self.state {
            TaskState::ToPickup(t) => t.pickup,
            TaskState::ToDropoff(t) => t.dropoff,
            TaskState::Idle => unreachable!("next_waypoint called while Idle"),
        }
    }

    /// How long an `unreachable` mark is trusted before it expires on its own (item 4 of
    /// the epoch decision entry): `UNREACHABLE_EXPIRY_CIRCULATIONS` full ring
    /// circulations, each circulation costed at the worst case (every hop needing a full
    /// skip) — `peer_ids.len() * TOKEN_SKIP_BUDGET_MS` per circulation, same basis as
    /// `watchdog_timeout` below.
    fn unreachable_expiry(&self) -> Duration {
        Duration::from_millis(
            self.peer_ids.len() as u64 * TOKEN_SKIP_BUDGET_MS * u64::from(UNREACHABLE_EXPIRY_CIRCULATIONS),
        )
    }

    fn is_unreachable(&self, id: u32) -> bool {
        self.unreachable
            .get(&id)
            .is_some_and(|&marked_at| Instant::now().duration_since(marked_at) < self.unreachable_expiry())
    }

    /// How long this robot waits, in real time, without observing a `TokenMsg` of *any*
    /// lineage before concluding it may be genuinely lost and attempting to regenerate
    /// it (Amendment C). Formula: worst case, the token needs to traverse every peer in
    /// the ring, and each hop can cost up to `TOKEN_SKIP_BUDGET_MS` (a complete skip) —
    /// `peer_ids.len() * TOKEN_SKIP_BUDGET_MS` is the longest a *healthy* ring should
    /// ever plausibly go without producing some token traffic again, even in the worst
    /// case every single hop needs a full skip.
    fn watchdog_timeout(&self) -> Duration {
        Duration::from_millis(self.peer_ids.len() as u64 * TOKEN_SKIP_BUDGET_MS)
    }

    /// Ring-successor of `from`, skipping anyone currently in `unreachable` (and whose
    /// mark hasn't yet expired — see `unreachable_expiry`) — and always skipping
    /// `self.robot_id` itself, since a robot must never be asked to deliver the token to
    /// itself (`Comms::recv_filtered`'s self-loopback exclusion means it could never see
    /// its own send as a valid ack anyway, which would otherwise silently degenerate
    /// into `self.robot_id` marking *itself* unreachable — a real bug this exact check
    /// replaced, caught by `tests/token_passing.rs`'s retry test on a 2-robot ring, the
    /// smallest case where the ring-successor of "the only other peer, now unreachable"
    /// wraps straight back to the caller). Falls back to `from` itself once every other
    /// candidate has been excluded — a degenerate case (this robot's only peer, or every
    /// peer, is currently unreachable) — so callers can detect "no progress possible" by
    /// checking `next_reachable_peer(x) == x` and stop, rather than looping forever.
    fn next_reachable_peer(&self, from: u32) -> u32 {
        let idx = self
            .peer_ids
            .iter()
            .position(|&id| id == from)
            .expect("from must be a ring member");
        let n = self.peer_ids.len();
        for step in 1..=n {
            let candidate = self.peer_ids[(idx + step) % n];
            if candidate == self.robot_id {
                continue;
            }
            if candidate == from || !self.is_unreachable(candidate) {
                return candidate;
            }
        }
        from
    }

    /// The pure per-turn decision (Algorithm 1's body, plus item 17's re-pooling): given
    /// the `TokenMsg` this robot just received and its own current position, resolves
    /// any claim conflict against its own commitment, re-asserts its own claim if the
    /// incoming token dropped it entirely (a lineage switch — the epoch decision entry's
    /// "claims dropped with the losing branch get re-asserted by their owners"),
    /// releases its current task if it's become unreachable, claims the nearest
    /// reachable unclaimed task if idle, and returns the outgoing token (bumped `seq`,
    /// `holder_id` set to the next *reachable* peer in the ring, `claimed_tasks`
    /// updated, `epoch`/`creator` passed through unchanged — epoch is a transport-layer
    /// concern `run_one_cycle` manages, not this pure core). No I/O — directly
    /// unit-testable, same as `Pibt::step`. `unreachable` is always empty for a
    /// `new_pure` instance (no wire, no `Heartbeat`s, no backoff timeouts), so this is
    /// exactly the old plain-ring-order behavior there.
    pub fn handle_token(&mut self, token: &TokenMsg, current_pos: Cell) -> TokenMsg {
        let mut claimed = token.claimed_tasks.clone();
        if !self.cancelled.is_empty() {
            claimed.retain(|c| !self.cancelled.contains(&c.task_id));
        }

        // Claim-conflict reconciliation (`docs/decisions.md`'s claim-reconciliation
        // decision entry, closing Step 1B's fork): if I'm locally committed to a task
        // the incoming token attributes to someone else, resolve it right here,
        // deterministically, rather than let both commitments stand.
        //
        // `ClaimEntry::picked_up` is what makes the comparison symmetric: both sides
        // compute it from the *same* two `(picked_up, robot_id)` pairs — mine read live
        // from `self.state`, the rival's read directly from the entry — so whichever
        // side is asking reaches the same answer. `picked_up` wins outright; a tie
        // (neither has picked up yet) goes to the lower `robot_id`.
        if let Some(my_task) = self.current_task() {
            if let Some(entry_idx) =
                claimed.iter().position(|c| c.task_id == my_task.task_id && c.robot_id != self.robot_id)
            {
                self.claim_conflicts += 1;
                let rival = claimed[entry_idx];
                let my_picked_up = matches!(self.state, TaskState::ToDropoff(_));
                let i_win = match (my_picked_up, rival.picked_up) {
                    (true, false) => true,
                    (false, true) => false,
                    (_, _) => self.robot_id < rival.robot_id,
                };
                if i_win {
                    claimed[entry_idx] = ClaimEntry {
                        task_id: my_task.task_id,
                        robot_id: self.robot_id,
                        picked_up: my_picked_up,
                    };
                } else {
                    self.state = TaskState::Idle;
                }
            }
        }

        if let Some(task) = self.current_task() {
            let target = self.next_waypoint();
            let still_reachable = self.grid.bfs_distance(target).contains_key(&current_pos);
            if !still_reachable {
                claimed.retain(|c| !(c.task_id == task.task_id && c.robot_id == self.robot_id));
                self.state = TaskState::Idle;
            } else {
                // Keep the wire's record of my own claim in sync: refresh my phase if
                // it's changed since I last held the token, and — the epoch decision
                // entry's addition — re-insert it entirely if a lineage switch dropped
                // it (a fresh epoch starts with empty claims; nothing else will ever
                // re-assert my claim for me).
                let my_picked_up = matches!(self.state, TaskState::ToDropoff(_));
                match claimed.iter_mut().find(|c| c.task_id == task.task_id && c.robot_id == self.robot_id) {
                    Some(entry) => entry.picked_up = my_picked_up,
                    None => claimed.push(ClaimEntry {
                        task_id: task.task_id,
                        robot_id: self.robot_id,
                        picked_up: my_picked_up,
                    }),
                }
            }
        }

        if self.state == TaskState::Idle {
            let claimed_ids: HashSet<u32> = claimed.iter().map(|c| c.task_id).collect();
            let nearest = self
                .tasks
                .iter()
                .filter(|t| !claimed_ids.contains(&t.task_id))
                .filter_map(|t| {
                    let dist = self.grid.bfs_distance(t.pickup).get(&current_pos).copied()?;
                    Some((dist, *t))
                })
                .min_by_key(|&(dist, _)| dist);

            if let Some((_, task)) = nearest {
                claimed.push(ClaimEntry {
                    task_id: task.task_id,
                    robot_id: self.robot_id,
                    picked_up: false,
                });
                self.state = TaskState::ToPickup(task);
            }
        }

        TokenMsg {
            seq: token.seq + 1,
            holder_id: self.next_reachable_peer(self.robot_id),
            claimed_tasks: claimed,
            epoch: token.epoch,
            creator: token.creator,
        }
    }

    /// Sends `token` to whoever it currently names as holder (`token.holder_id`),
    /// retrying with doubling backoff (Amendment B: 20/40/80/160ms, ~300ms total budget)
    /// until proof of delivery is observed, or the budget is exhausted. Proof (the
    /// epoch decision entry, replacing the old "any higher `seq`, from anyone, counts"
    /// rule — found, by real-transport testing, to let a sender falsely conclude success
    /// from an unrelated lineage's ambient traffic): an `Ack` from the target, or a
    /// `TokenMsg` whose `sender_id` is *also* the target, with `epoch >= token.epoch`
    /// and `seq > token.seq` — nothing from any other sender counts, no matter how high
    /// its `seq`. Every backoff step *drains* (not samples) whatever's queued within its
    /// window, so the real ack can't get buried behind other traffic the way Fix 1a
    /// found it could; every `Tick`/`Pose`/`Heartbeat` pulled out along the way is
    /// forwarded via `incidentals` rather than discarded, and a `Heartbeat`
    /// additionally clears its sender from `self.unreachable` and refreshes
    /// `heartbeat_last_seen`, regardless of whose ack this call is waiting for.
    fn deliver_with_backoff(&mut self, token: &TokenMsg, incidentals: &mut Vec<Received>) -> bool {
        let target = token.holder_id;
        for &step_ms in &TOKEN_RETRY_BACKOFF_MS {
            if let Err(e) = self.comms().send_token(token.clone()) {
                eprintln!("[robot {}] failed to send token: {e}", self.robot_id);
            }
            let deadline = Instant::now() + Duration::from_millis(step_ms);
            loop {
                let remaining = deadline.saturating_duration_since(Instant::now());
                if remaining.is_zero() {
                    break;
                }
                self.comms()
                    .set_read_timeout(Some(remaining))
                    .expect("set retry interval");
                match self.comms().recv_filtered() {
                    Ok(Some(received)) => {
                        // Operator commands can land while waiting for an ack too.
                        self.apply_command(&received.payload);
                        match &received.payload {
                        Payload::Ack(ack)
                            if ack.seq == token.seq
                                && ack.to == self.robot_id
                                && received.sender_id == target =>
                        {
                            return true;
                        }
                        Payload::Token(t)
                            if received.sender_id == target
                                && t.epoch >= token.epoch
                                && t.seq > token.seq =>
                        {
                            return true;
                        }
                        Payload::Heartbeat(hb) => {
                            self.unreachable.remove(&hb.robot_id);
                            self.heartbeat_last_seen.insert(hb.robot_id, Instant::now());
                            incidentals.push(received);
                        }
                        payload if is_forwardable(payload) => incidentals.push(received),
                        _ => {} // protocol-internal noise (stray Token/Ack from elsewhere): drop
                        }
                    }
                    Ok(None) => continue, // recv_filtered resolves internally; defensive only
                    Err(_) => break,      // this backoff step's window elapsed
                }
            }
        }
        false
    }

    /// Checks the watchdog (Amendment C) and, if it fires, builds the fresh regenerated
    /// token (new `epoch`/`seq`, empty claims — owners re-assert their own on their next
    /// turn, via `handle_token`'s lineage-switch re-insertion above). Fires only when
    /// (a) no token of any lineage has been seen for `watchdog_timeout()`, *and* (b)
    /// this robot is the lowest id among itself and whoever it currently (within that
    /// same window) hears a `Heartbeat` from — so at most one robot in a healthy fleet
    /// ever regenerates for a given silence.
    fn maybe_build_watchdog_token(&mut self) -> Option<TokenMsg> {
        let now = Instant::now();
        let timeout = self.watchdog_timeout();
        if now.duration_since(self.last_token_seen) < timeout {
            return None;
        }
        let mut candidates: Vec<u32> = self
            .heartbeat_last_seen
            .iter()
            .filter(|&(_, &t)| now.duration_since(t) <= timeout)
            .map(|(&id, _)| id)
            .collect();
        candidates.push(self.robot_id);
        if *candidates.iter().min().expect("self is always present") != self.robot_id {
            return None;
        }

        let new_epoch = self.highest_seen_epoch.map_or(0, |(e, _)| e) + 1;
        let new_seq = self.highest_seen_seq.map_or(0, |s| s + 1);
        self.highest_seen_epoch = Some((new_epoch, self.robot_id));
        self.last_token_seen = now; // don't immediately re-trigger on the next check
        self.epoch_bumps += 1;

        Some(TokenMsg {
            seq: new_seq.wrapping_sub(1), // handle_token computes `token.seq + 1` == new_seq
            holder_id: self.robot_id,
            claimed_tasks: vec![],
            epoch: new_epoch,
            creator: self.robot_id,
        })
    }

    /// Real-transport wrapper around `handle_token`. Blocks (bounded by `wait_timeout`)
    /// until a `TokenMsg` naming this robot as holder is observed on the wire — sending
    /// an `Ack` immediately on first receipt, independent of how long processing then
    /// takes — then delivers the result via `deliver_with_backoff`, marking and skipping
    /// past any peer whose budget is exhausted (Amendment B), bumping the epoch on skip
    /// (Amendment A/the epoch decision entry — never a new `seq` for a skip, only a new
    /// `epoch`). If the wait times out and the watchdog fires, regenerates the token
    /// instead of returning empty-handed. Returns every incidental `Tick`/`Pose`/
    /// `Heartbeat` observed along the way so the caller can still fold them into its own
    /// normal per-tick handling.
    pub fn run_one_cycle(
        &mut self,
        current_pos: Cell,
        wait_timeout: Duration,
    ) -> (TokenCycleOutcome, Vec<Received>) {
        self.comms()
            .set_read_timeout(Some(wait_timeout))
            .expect("set wait timeout");
        let mut incidentals = Vec::new();
        let mut regenerated = false;

        let token = loop {
            match self.comms().recv_filtered() {
                Ok(Some(received)) => {
                    if let Payload::Token(t) = &received.payload {
                        self.last_token_seen = Instant::now();
                        if t.holder_id == self.robot_id {
                            let incoming_pair = (t.epoch, t.creator);
                            match self.highest_seen_epoch {
                                Some(cur) if epoch_dominates(cur, incoming_pair) => {
                                    // Strictly older lineage: drop silently, not even acked.
                                    self.lineages_dropped += 1;
                                    continue;
                                }
                                Some(cur) if cur == incoming_pair => {
                                    // Same lineage: the existing seq-based dedupe.
                                    let is_duplicate = self.highest_seen_seq == Some(t.seq);
                                    let is_stale = self.highest_seen_seq.is_some_and(|h| t.seq < h);
                                    if is_stale {
                                        continue; // older than our watermark: drop silently
                                    }
                                    if let Err(e) = self.comms().send_ack(Ack {
                                        seq: t.seq,
                                        from: self.robot_id,
                                        to: received.sender_id,
                                    }) {
                                        eprintln!("[robot {}] failed to send ack: {e}", self.robot_id);
                                    }
                                    if is_duplicate {
                                        // The sender's own retry, still in flight:
                                        // re-ack, but do not reprocess or forward again
                                        // — a second forward of the same hop would fork
                                        // the ring.
                                        continue;
                                    }
                                    break t.clone();
                                }
                                _ => {
                                    // `None`, or `incoming_pair` strictly dominates
                                    // `cur`: adopt this as the new baseline lineage.
                                    self.highest_seen_epoch = Some(incoming_pair);
                                    if let Err(e) = self.comms().send_ack(Ack {
                                        seq: t.seq,
                                        from: self.robot_id,
                                        to: received.sender_id,
                                    }) {
                                        eprintln!("[robot {}] failed to send ack: {e}", self.robot_id);
                                    }
                                    break t.clone();
                                }
                            }
                        }
                    }
                    self.apply_command(&received.payload);
                    if let Payload::Heartbeat(hb) = &received.payload {
                        self.unreachable.remove(&hb.robot_id);
                        self.heartbeat_last_seen.insert(hb.robot_id, Instant::now());
                    }
                    if is_forwardable(&received.payload) {
                        incidentals.push(received);
                    }
                }
                Ok(None) => continue, // recv_filtered resolves internally; defensive only
                Err(_) => {
                    if let Some(synthetic) = self.maybe_build_watchdog_token() {
                        regenerated = true;
                        break synthetic;
                    }
                    return (TokenCycleOutcome::NoTokenArrived, incidentals);
                }
            }
        };

        let before = self.current_task();
        let outgoing = self.handle_token(&token, current_pos);
        let after = self.current_task();
        let mut claimed = match (before, after) {
            (Some(b), Some(a)) if b.task_id == a.task_id => None,
            (_, Some(a)) => Some(a),
            _ => None,
        };
        // Whether `outgoing` just asserted *new* claim information the *incoming*
        // token didn't already carry — either a fresh claim (`claimed.is_some()` above
        // is a subset of this) or winning a claim conflict by overwriting a rival's
        // entry (same task, `claimed` stays `None` there since `before`/`after` share a
        // `task_id`, but the wire's record of *who* holds it just changed). Computed
        // purely from the two `claimed_tasks` lists `run_one_cycle` already has, not by
        // changing `handle_token`'s pure, no-I/O interface — see this cycle's own
        // release check below for why this distinction matters and how it was found.
        let my_newly_asserted_claim = after.filter(|task| {
            let incoming_claimant =
                token.claimed_tasks.iter().find(|c| c.task_id == task.task_id).map(|c| c.robot_id);
            incoming_claimant != Some(self.robot_id)
        });
        // The watermark tracks "what this robot was addressed with" for the *received*
        // case (`token.seq`); a regeneration has no received value in that sense — it's
        // self-originated — so the watermark becomes what actually went out
        // (`outgoing.seq`, which `handle_token` computed as `token.seq + 1`).
        self.highest_seen_seq = Some(if regenerated { outgoing.seq } else { token.seq });

        let mut to_send = outgoing.clone();
        let mut target = outgoing.holder_id;
        let mut skipped_peer = None;
        let mut delivered = false;
        // Bounded by ring size: a fully-unreachable ring must still terminate rather
        // than spin.
        for _attempt in 0..self.peer_ids.len() {
            to_send.holder_id = target;
            if self.deliver_with_backoff(&to_send, &mut incidentals) {
                delivered = true;
                break;
            }
            self.unreachable.insert(target, Instant::now());
            self.skip_count += 1;
            skipped_peer = Some(target);
            let next = self.next_reachable_peer(target);
            if next == target {
                break; // every other peer is currently marked unreachable
            }
            target = next;
            // Skip-resend: same seq, a fresh epoch (never mint a new seq for a skip —
            // Amendment A) so this branch is totally ordered against any other live one.
            let new_epoch = self.highest_seen_epoch.map_or(0, |(e, _)| e) + 1;
            to_send.epoch = new_epoch;
            to_send.creator = self.robot_id;
            self.highest_seen_epoch = Some((new_epoch, self.robot_id));
            self.epoch_bumps += 1;
        }

        // Found via a real, 100%-reproducible double-commitment
        // (`tests/token_passing.rs`'s `real_transport_fork_reproduces_via_actual_skip_path_50_trials`):
        // the claim-conflict reconciliation above only ever runs on the *receiving*
        // side of `handle_token`. If this cycle's outgoing token just asserted *new*
        // claim info (`my_newly_asserted_claim`, above) and that fails to reach *every*
        // peer, the correction never puts itself in front of anyone to reconcile
        // against — and the peers who rejected it also mark this robot unreachable,
        // excluding it from the ring for a real, non-negligible window
        // (`unreachable_expiry`). Left alone, the claim would sit here uncontested
        // indefinitely, alongside another robot's conflicting belief about the same
        // task. Releasing it back to `Idle` here is the fallback that's actually safe:
        // the task returns to the pool and gets re-claimed (by this robot or whoever
        // else's token reaches it) the next time it can genuinely participate, rather
        // than standing as a silent, unreconciled duplicate.
        //
        // Deliberately checks `my_newly_asserted_claim`, not just `claimed.is_some()`
        // (a first attempt at this fix used only the latter and still failed 50/50):
        // real-transport tracing found the actual failure wasn't only a *fresh* claim
        // going unheard, but *winning a claim conflict* against a rival's entry
        // (`before`/`after` share the same `task_id`, so `claimed` stays `None`, even
        // though the wire's record of the rightful claimant just changed) and then
        // failing to announce *that* correction either — traced directly via temporary
        // instrumentation: robot 2 won a conflict against robot 3's already-circulated
        // claim (lower id, per the tie-break), but its corrective token was rejected by
        // every peer, leaving robot 3 permanently unaware it had lost. A *stable*
        // re-circulation of an *already-known* claim (no fresh claim, no conflict, nothing
        // new to tell anyone) deliberately does *not* release just because one delivery
        // attempt failed — `my_newly_asserted_claim` is `None` in that case, since the
        // incoming token already showed this robot as the claimant.
        if !delivered {
            if let Some(task) = my_newly_asserted_claim {
                debug_assert_eq!(self.current_task(), Some(task));
                self.state = TaskState::Idle;
                self.unannounced_claims_released += 1;
                claimed = None;
            }
        }

        let outcome = match (regenerated, skipped_peer) {
            (true, _) => TokenCycleOutcome::Regenerated { claimed },
            (false, Some(peer)) => TokenCycleOutcome::Skipped { unreachable_peer: peer },
            (false, None) => TokenCycleOutcome::Handled { claimed },
        };
        (outcome, incidentals)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_grid() -> Grid {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map");
        Grid::load(path).expect("map should load")
    }

    fn token0() -> TokenMsg {
        TokenMsg { seq: 0, holder_id: 1, claimed_tasks: vec![], epoch: 0, creator: 0 }
    }

    fn wire(c: Cell) -> (i32, i32) {
        (c.0 as i32, c.1 as i32)
    }

    /// Decision 18: an injected job enters the pool (idempotently), gets claimed by an
    /// idle robot on its next token turn, and invalid cells are refused.
    #[test]
    fn injected_task_is_claimed_and_duplicates_or_bad_cells_are_ignored() {
        let grid = test_grid();
        let free = grid.free_cells();
        let mut layer = TaskLayer::new_pure(1, &grid, vec![], vec![1]);
        let m = TaskInject { task_id: 5, pickup: wire(free[10]), dropoff: wire(free[20]) };
        assert!(layer.apply_task_inject(m));
        assert!(!layer.apply_task_inject(m), "same id twice must be ignored");
        assert!(!layer.apply_task_inject(TaskInject { task_id: 6, pickup: (-1, 0), dropoff: wire(free[20]) }));
        assert!(!layer.apply_task_inject(TaskInject { task_id: 7, pickup: (0, 0), dropoff: wire(free[20]) }), "(0,0) is a tree/wall cell");
        let out = layer.handle_token(&token0(), free[0]);
        assert!(matches!(layer.state(), TaskState::ToPickup(t) if t.task_id == 5));
        assert_eq!(out.claimed_tasks.len(), 1);
    }

    /// Retargeting a task in progress moves the robot's goal; cancelling sends it Idle and
    /// strips the claim from the next outgoing token.
    #[test]
    fn retarget_moves_the_goal_and_cancel_releases_the_claim() {
        let grid = test_grid();
        let free = grid.free_cells();
        let task = Task { task_id: 1, pickup: free[10], dropoff: free[20] };
        let mut layer = TaskLayer::new_pure(1, &grid, vec![task], vec![1]);
        layer.handle_token(&token0(), free[0]);
        assert_eq!(layer.current_goal(), Some(free[10]));

        layer.apply_task_retarget(TaskRetarget {
            task_id: 1, pickup: wire(free[30]), dropoff: wire(free[40]), cancel: false,
        });
        assert_eq!(layer.current_goal(), Some(free[30]), "goal follows the new pickup");

        layer.apply_task_retarget(TaskRetarget {
            task_id: 1, pickup: (0, 0), dropoff: (0, 0), cancel: true,
        });
        assert_eq!(layer.state(), TaskState::Idle);
        let mut with_claim = token0();
        with_claim.claimed_tasks.push(ClaimEntry { task_id: 1, robot_id: 1, picked_up: false });
        let out = layer.handle_token(&with_claim, free[0]);
        assert!(out.claimed_tasks.is_empty(), "cancelled task's claim must leave the token");
        assert_eq!(layer.state(), TaskState::Idle, "and it must not be re-claimed");
    }

    /// Blocking the cell a task needs releases it (existing re-pooling path); the applied
    /// block is queued for the robot's planner grid.
    #[test]
    fn block_cell_makes_the_task_unreachable_and_is_queued_for_the_planner() {
        let grid = test_grid();
        let free = grid.free_cells();
        let far = free[free.len() - 1];
        let mut layer = TaskLayer::new_pure(1, &grid, vec![Task { task_id: 1, pickup: far, dropoff: far }], vec![1]);
        layer.handle_token(&token0(), free[0]);
        assert!(matches!(layer.state(), TaskState::ToPickup(_)));
        layer.apply_block_cell(BlockCell { x: far.0 as i32, y: far.1 as i32, blocked: true });
        let out = layer.handle_token(&token0(), free[0]);
        assert_eq!(layer.state(), TaskState::Idle, "pickup itself is blocked, so it's released");
        assert!(out.claimed_tasks.is_empty());
        assert_eq!(layer.drain_applied_blocks().len(), 1);
        assert!(layer.drain_applied_blocks().is_empty());
    }

    /// Lightweight smoke test for the pure core; the dedicated `tests/token_passing.rs`
    /// (item 16) covers the full Phase 5 gate.
    #[test]
    fn idle_robot_claims_nearest_task_and_forwards_token() {
        let grid = test_grid();
        let free = grid.free_cells();
        let start = free[0];
        let near_task_pickup = free[1];
        let far_task_pickup = free[free.len() - 1];

        let tasks = vec![
            Task {
                task_id: 1,
                pickup: far_task_pickup,
                dropoff: far_task_pickup,
            },
            Task {
                task_id: 2,
                pickup: near_task_pickup,
                dropoff: near_task_pickup,
            },
        ];
        let mut layer = TaskLayer::new(1, &grid, tasks, vec![1, 2], start).expect("build layer");

        let incoming = TokenMsg {
            seq: 0,
            holder_id: 1,
            claimed_tasks: vec![],
            epoch: 0,
            creator: 0,
        };
        let outgoing = layer.handle_token(&incoming, start);

        assert_eq!(outgoing.seq, 1);
        assert_eq!(outgoing.holder_id, 2, "should forward to the next peer in the ring");
        assert_eq!(
            outgoing.claimed_tasks,
            vec![ClaimEntry { task_id: 2, robot_id: 1, picked_up: false }],
            "should claim the nearer task"
        );
        assert_eq!(layer.state(), TaskState::ToPickup(tasks_task(2, near_task_pickup)));
    }

    /// Item 27 (Decision 15): `new_pure` must drive `handle_token` identically to a
    /// real, `Comms`-backed instance — the whole point is that a benchmark's simulated
    /// robots get the exact same task-allocation behavior real ones do, just without
    /// paying for a socket per instance.
    #[test]
    fn new_pure_drives_handle_token_the_same_as_a_real_instance() {
        let grid = test_grid();
        let free = grid.free_cells();
        let start = free[0];
        let pickup = free[1];

        let tasks = vec![Task { task_id: 1, pickup, dropoff: pickup }];
        let mut layer = TaskLayer::new_pure(1, &grid, tasks, vec![1, 2]);

        let incoming = TokenMsg { seq: 0, holder_id: 1, claimed_tasks: vec![], epoch: 0, creator: 0 };
        let outgoing = layer.handle_token(&incoming, start);

        assert_eq!(
            outgoing.claimed_tasks,
            vec![ClaimEntry { task_id: 1, robot_id: 1, picked_up: false }]
        );
        assert_eq!(layer.state(), TaskState::ToPickup(tasks_task(1, pickup)));
    }

    #[test]
    #[should_panic(expected = "run_one_cycle needs a real TaskLayer::new instance")]
    fn new_pure_instance_cannot_run_one_cycle() {
        let grid = test_grid();
        let free = grid.free_cells();
        let mut layer = TaskLayer::new_pure(1, &grid, vec![], vec![1, 2]);
        let _ = layer.run_one_cycle(free[0], Duration::from_millis(1));
    }

    fn tasks_task(task_id: u32, cell: Cell) -> Task {
        Task {
            task_id,
            pickup: cell,
            dropoff: cell,
        }
    }
}
