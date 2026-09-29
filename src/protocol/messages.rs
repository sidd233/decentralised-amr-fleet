//! Wire message structs, no logic. Phase 2, item 7 of `docs/BUILD_PLAN.md`.
//!
//! These are what gets broadcast over the UDP multicast bus starting at item 11
//! (`robot/comms.rs`). Coordinates and ids use fixed-width types (`i32`/`u32`/`u64`),
//! not `usize` like `world::grid::Grid` uses internally — `usize`'s width isn't
//! guaranteed across platforms, and these values cross a real network wire.

use serde::{Deserialize, Serialize};

/// A robot's current position and intended next move, broadcast every tick.
///
/// This is what makes PIBT's priority-inheritance conflict resolution a message
/// exchange rather than one process directly commanding another: a robot losing a
/// contested cell to a higher-priority neighbor needs to see that neighbor's `position`,
/// `intended_next`, and `priority` to know it lost and must yield
/// (`docs/PS_AND_ARCHITECTURE.md` §3.3).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PoseIntent {
    pub robot_id: u32,
    pub tick: u64,
    pub position: (i32, i32),
    pub intended_next: (i32, i32),
    /// Rises the longer this robot has waited; higher wins a priority-inheritance
    /// conflict, ties broken by the lower `robot_id`. `f64`, not `u32`: item 9's actual
    /// priority scheme (`Pibt::run`) starts every agent at a fractional value
    /// (`dist_to_goal / cell_count`, typically well under 1.0) and increments it by
    /// exactly `1.0` per waiting tick — an integer type would truncate nearly every
    /// agent's priority to `0` before item 13 ever had a wire value to compare. Dropping
    /// `Eq` from this struct's derive is the direct consequence (`f64` has no total
    /// order, only `PartialEq`/`PartialOrd`, because of `NaN`); nothing here ever
    /// produces `NaN`, so `PartialEq`/`partial_cmp` are exact for every value this field
    /// actually takes.
    pub priority: f64,
    /// Set once this robot has tried and lost every one of its ranked candidate cells
    /// for this tick (item 13's distributed fallback, mirroring `func_pibt`'s
    /// unconditional stay-in-place when every candidate is blocked). An `exhausted`
    /// claim always wins its conflicts outright, regardless of `priority` — nobody can
    /// be forced out of a cell they're already occupying if they truly have nowhere else
    /// to go. Without this flag, an ordinary "staying is my best-ranked option so far"
    /// claim (still an ordinary, yieldable candidate) would be indistinguishable on the
    /// wire from this final, unconditional one — both broadcast `intended_next ==
    /// position`.
    pub exhausted: bool,
}

/// One task's claim record as carried on the wire inside `TokenMsg::claimed_tasks`.
/// `picked_up` was added by `docs/decisions.md`'s claim-reconciliation decision entry
/// (closing Step 1B's token-fork finding): without a progress marker, a robot
/// reconciling a conflicting claim against its own commitment had no way to tell a
/// rival's genuine `ToDropoff` progress from a bare, possibly-stale `ToPickup` claim —
/// this field is exactly that marker, `true` once the claiming robot's own state has
/// reached `TaskState::ToDropoff` for this task.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClaimEntry {
    pub task_id: u32,
    pub robot_id: u32,
    pub picked_up: bool,
}

/// The lifelong-MAPF task-allocation token (Ma et al. AAMAS'17 Algorithm 1, "Token
/// Passing" — see `docs/REFERENCES.md`), wired up at item 15 (`robot/task_layer.rs`).
///
/// The one message type that isn't fire-and-forget: `docs/decisions.md`'s task-layer
/// design says a lost token stalls allocation for the whole fleet, so it gets ack/retry
/// keyed on `seq` rather than being dropped silently like `PoseIntent`/`Heartbeat`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TokenMsg {
    pub seq: u64,
    pub holder_id: u32,
    /// Claims already recorded in the token — visible to whoever holds the token next,
    /// per the Token Passing algorithm's requirement that a claim travel with the token
    /// itself. At most one entry per `task_id` at any time.
    pub claimed_tasks: Vec<ClaimEntry>,
    /// Lineage identity, added by the epoch-design decision entry in `docs/decisions.md`
    /// (closing the post-fork token-survival finding — a bare `seq` watermark can't tell
    /// two independently-numbered branches of the same ancestor apart, so it can't make
    /// exactly one of them win). `(epoch, creator)` orders lineages: a strictly higher
    /// `epoch` always wins; at equal `epoch`, the lower `creator` wins. The bootstrap
    /// token is `epoch: 0, creator: 0`. Bumped only at a skip-resend (same `seq`, new
    /// epoch) or a watchdog regeneration (new `seq` too) — never on an ordinary hop.
    pub epoch: u32,
    pub creator: u32,
}

/// Explicit acknowledgement of one `TokenMsg` hop, added by the Fix 1 token-flood
/// remediation (`docs/decisions.md`): sent immediately by whoever receives a `TokenMsg`
/// naming them holder, independent of how long that receiver then takes to process and
/// forward it. Replaces the original design's "a higher `seq` observed on the wire" as
/// the *only* proof of delivery (Fix 1a found that implicit signal gets buried once
/// several robots are concurrently retrying, since the sender was sampling one arbitrary
/// queued packet per attempt rather than searching for it) — the sender's retry loop
/// now still also accepts a higher-`seq` `TokenMsg` as proof (Amendment A: either counts),
/// since that's still valid independent evidence the hop succeeded even if this `Ack`
/// itself was the one that got lost.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Ack {
    pub seq: u64,
    /// The robot sending this ack (i.e. the token's receiver).
    pub from: u32,
    /// The robot this ack is for (i.e. the token's sender, who is waiting on it).
    pub to: u32,
}

/// Liveness ping. `mode_state_machine.rs` (item 14) measures heartbeat loss and latency
/// from a stream of these against Decision 3's thresholds (A, B, L).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Heartbeat {
    pub robot_id: u32,
    pub tick: u64,
    /// 0.0-100.0, from `robot::metrics::BatteryModel` (item 19, Decision 7) — carried on
    /// `Heartbeat` because it's already the one message every robot broadcasts every
    /// tick, rather than inventing a new message type just for this. `f32`, not an
    /// integer percent: the model drains by small fractional amounts each tick, and
    /// rounding here would make the dashboard's readout jump in visible whole-percent
    /// steps instead of draining smoothly. Dropping `Eq` from this struct's derive is the
    /// direct consequence — same reasoning already logged for `PoseIntent`'s `priority`
    /// field above (`f32`/`f64` have no total order because of `NaN`, though nothing here
    /// ever produces one).
    pub battery_pct: f32,
}

/// The clock driver's tick broadcast (`clock_driver.rs`, item 23,
/// `docs/PS_AND_ARCHITECTURE.md` §3.1's "clock-driver process... keeps simulation ticks
/// synchronized for reproducibility"). Carries no decision content — the driver "makes no
/// planning or allocation decisions" per that same section, only a tick number every
/// robot process advances to in lockstep. This is what `robot::RobotProcess::run`
/// replaces `std::thread::sleep`-based self-pacing with (`docs/decisions.md` Decision 8):
/// self-paced ticking made cross-robot tick comparison ill-defined, which this fixes by
/// construction — every robot sets its own tick counter from this message rather than
/// incrementing independently.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TickMsg {
    pub tick: u64,
}

/// The three degradation modes (`docs/PS_AND_ARCHITECTURE.md` §3.5).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Mode {
    Cooperative,
    Cautious,
    Autonomous,
}

/// Broadcast whenever a robot's mode state machine transitions, so peers know how much
/// to trust that robot's `PoseIntent` broadcasts — a `Cooperative` peer shares full
/// intents to plan around; an `Autonomous` peer should be treated as unpredictable
/// outside local sensing range.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct ModeAnnounce {
    pub robot_id: u32,
    pub mode: Mode,
    pub tick: u64,
}

/// Operator command (Decision 18): add a new pickup-and-deliver job to every robot's task
/// pool at runtime. Idempotent by `task_id` — a robot that already has that id ignores a
/// repeat, so the sender may resend for reliability over lossy UDP.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TaskInject {
    pub task_id: u32,
    pub pickup: (i32, i32),
    pub dropoff: (i32, i32),
}

/// Operator command (Decision 18): change an existing task's pickup/dropoff, or cancel it
/// outright (`cancel`, in which case the cells are ignored). A robot currently holding a
/// claim on the task releases it so the token re-pools it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct TaskRetarget {
    pub task_id: u32,
    pub pickup: (i32, i32),
    pub dropoff: (i32, i32),
    pub cancel: bool,
}

/// Operator command (Decision 18): block or unblock one map cell (a "blocked aisle"),
/// applied by every robot to its own copy of the grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct BlockCell {
    pub x: i32,
    pub y: i32,
    pub blocked: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn operator_commands_round_trip() {
        let inject = TaskInject { task_id: 9, pickup: (4, 5), dropoff: (6, 7) };
        let back: TaskInject = serde_json::from_str(&serde_json::to_string(&inject).unwrap()).unwrap();
        assert_eq!(inject, back);
        let retarget = TaskRetarget { task_id: 9, pickup: (1, 2), dropoff: (3, 4), cancel: true };
        let back: TaskRetarget =
            serde_json::from_str(&serde_json::to_string(&retarget).unwrap()).unwrap();
        assert_eq!(retarget, back);
        let block = BlockCell { x: 10, y: 11, blocked: true };
        let back: BlockCell = serde_json::from_str(&serde_json::to_string(&block).unwrap()).unwrap();
        assert_eq!(block, back);
    }

    /// Phase 2 gate fallback (`docs/TESTING_PLAN.md`): no dedicated `tests/*.rs` file
    /// exists until item 10, so this in-file round-trip check stands in for it. This
    /// also directly foreshadows part of item 12's `tests/comms_protocol.rs` gate
    /// ("encode/decode round-trip for every message type"), just without real sockets.
    #[test]
    fn pose_intent_round_trips() {
        let msg = PoseIntent {
            robot_id: 3,
            tick: 42,
            position: (5, 7),
            intended_next: (5, 8),
            priority: 12.5,
            exhausted: false,
        };
        let json = serde_json::to_string(&msg).unwrap();
        let back: PoseIntent = serde_json::from_str(&json).unwrap();
        assert_eq!(msg, back);
    }

    #[test]
    fn ack_round_trips() {
        let msg = Ack { seq: 7, from: 2, to: 1 };
        let json = serde_json::to_string(&msg).unwrap();
        let back: Ack = serde_json::from_str(&json).unwrap();
        assert_eq!(msg, back);
    }

    #[test]
    fn token_msg_round_trips() {
        let msg = TokenMsg {
            seq: 1,
            holder_id: 2,
            claimed_tasks: vec![
                ClaimEntry { task_id: 10, robot_id: 2, picked_up: false },
                ClaimEntry { task_id: 11, robot_id: 5, picked_up: true },
            ],
            epoch: 3,
            creator: 7,
        };
        let json = serde_json::to_string(&msg).unwrap();
        let back: TokenMsg = serde_json::from_str(&json).unwrap();
        assert_eq!(msg, back);
    }

    #[test]
    fn heartbeat_round_trips() {
        let msg = Heartbeat {
            robot_id: 4,
            tick: 100,
            battery_pct: 87.5,
        };
        let json = serde_json::to_string(&msg).unwrap();
        let back: Heartbeat = serde_json::from_str(&json).unwrap();
        assert_eq!(msg, back);
    }

    #[test]
    fn tick_msg_round_trips() {
        let msg = TickMsg { tick: 42 };
        let json = serde_json::to_string(&msg).unwrap();
        let back: TickMsg = serde_json::from_str(&json).unwrap();
        assert_eq!(msg, back);
    }

    #[test]
    fn mode_announce_round_trips_every_variant() {
        for mode in [Mode::Cooperative, Mode::Cautious, Mode::Autonomous] {
            let msg = ModeAnnounce {
                robot_id: 1,
                mode,
                tick: 7,
            };
            let json = serde_json::to_string(&msg).unwrap();
            let back: ModeAnnounce = serde_json::from_str(&json).unwrap();
            assert_eq!(msg, back);
        }
    }
}
