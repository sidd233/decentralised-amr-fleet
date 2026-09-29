//! Central configuration constants.
//!
//! Decision 3's thresholds (`docs/decisions.md`, originally slated for Phase 1 item 3)
//! and Phase 2 item 8's "remaining constants: tick rate, ports" both land in this one
//! file — `docs/BUILD_PLAN.md` names `src/config.rs` as home for both, and Phase 1 had
//! no code yet to carry them until now.

use std::net::Ipv4Addr;

/// Simulation tick interval in milliseconds. Drives `clock_driver.rs`'s broadcast
/// cadence (item 23) and every robot's tick loop (item 20).
///
/// 100ms (10 ticks/sec) is a default chosen for this item, not derived from any
/// external source: fast enough that Phase 6's "few hundred ticks" smoke-test gate
/// finishes quickly, slow enough that real UDP round trips on one machine aren't
/// swamped. Easy to retune later if Phase 8's benchmark timings want it different.
pub const TICK_INTERVAL_MS: u64 = 100;

/// Multicast group every robot process, the clock driver, and the dashboard backend
/// join — one shared bus, per `docs/PS_AND_ARCHITECTURE.md`'s architecture diagram, not
/// a separate channel per message type. Chosen from `239.0.0.0/8`, the
/// administratively-scoped range (RFC 2365) reserved for private use within one
/// organization/site — appropriate for a single-machine or single-LAN simulation.
pub const MULTICAST_ADDR: Ipv4Addr = Ipv4Addr::new(239, 255, 0, 1);

/// Shared UDP port every robot process binds via `SO_REUSEPORT` (item 11), so multiple
/// robot processes on one machine can all receive the same multicast traffic, per
/// `docs/BUILD_PLAN.md` item 11. The clock driver's tick broadcasts and the dashboard
/// backend's passive listening use this same port/group.
pub const MULTICAST_PORT: u16 = 7000;

/// The `sender_id` `clock_driver.rs` (item 23) uses on the shared multicast bus — a
/// reserved sentinel, never assigned to a real robot (CLI/test convention throughout this
/// project starts robot ids at 1), so `Comms::recv_filtered`'s self-loopback exclusion
/// can't accidentally collide with any real robot's own id.
pub const CLOCK_DRIVER_ID: u32 = 0;

/// The `sender_id` `main.rs`'s `bootstrap-token` subcommand uses when it originates the
/// fleet's very first `TokenMsg` — Decision 13 (`docs/decisions.md`): no real robot can
/// bootstrap its own token-passing ring by sending to itself (`Comms::recv_filtered`
/// drops a robot's own loopback by design), so a short-lived out-of-ring process has to
/// do it once at startup instead. `u32::MAX` rather than a small sentinel like
/// `CLOCK_DRIVER_ID`'s `0`, since — unlike the clock driver, which never shares the bus
/// with another sentinel — this only needs to be obviously outside the range of real
/// robot ids (which start at 1 by CLI/test convention, with no fixed upper bound), a
/// reserved sentinel, never assigned to a real robot, same convention as
/// `CLOCK_DRIVER_ID`.
pub const BOOTSTRAP_TOKEN_ID: u32 = u32::MAX;

/// The `sender_id` `dashboard::server`'s passive listener (item 24) binds under — it
/// never sends anything onto the bus (`docs/PS_AND_ARCHITECTURE.md` §3.6: "passive
/// WebSocket listener... can't become a hidden central coordinator"), so this only
/// matters as `Comms`'s own bookkeeping, not for any real self-loopback risk. Kept as a
/// third distinct reserved sentinel anyway, same never-assigned-to-a-real-robot
/// convention as `CLOCK_DRIVER_ID`/`BOOTSTRAP_TOKEN_ID`, rather than reusing either.
pub const DASHBOARD_LISTENER_ID: u32 = u32::MAX - 1;

/// HTTP/WebSocket port for the dashboard's own axum server (item 24) — distinct from
/// `MULTICAST_PORT` since it serves browser clients over TCP, not robot-to-robot UDP.
pub const DASHBOARD_HTTP_PORT: u16 = 8080;

/// Default range-scoping radius, in grid cells, beyond which a robot's comms layer
/// (item 11) discards an incoming broadcast rather than acting on it — part of
/// `docs/PS_AND_ARCHITECTURE.md` §3.2's "range-scoped to nearby robots only". Item 12's
/// test overrides this with an artificially small radius rather than relying on this
/// default, so this value only matters for real runs.
pub const COMMS_RANGE_CELLS: u32 = 20;

/// Item 18 (`robot/perception.rs`): the radius, in grid cells, a robot can still detect
/// a peer's *physical presence* by in Autonomous mode, once it's stopped trusting peer
/// *intent* entirely (`docs/PS_AND_ARCHITECTURE.md` §3.2/§3.5). Deliberately much smaller
/// than `COMMS_RANGE_CELLS`: that constant models radio reach, this one models a short
/// physical proximity sensor — the two aren't the same distance for the same reason a
/// robot's Wi-Fi range and its bump sensor's range aren't the same in real hardware.
pub const LOCAL_SENSING_RADIUS: u32 = 3;

/// Item 19 (`robot/metrics.rs`), Decision 7: percent drained from a robot's simulated
/// battery per tick while it stays in place. Simulation-illustrative, not derived from a
/// real battery spec — chosen only so a benchmark-length run (a few hundred to a few
/// thousand ticks, per `docs/TESTING_PLAN.md`'s smoke-test/Phase 8 scales) drains a
/// visible but not implausibly fast amount, giving the dashboard's battery readout
/// (§1.3) something real to show.
pub const BATTERY_IDLE_DRAIN_PCT_PER_TICK: f32 = 0.005;

/// Item 19, Decision 7: percent drained per tick while moving — deliberately larger than
/// the idle rate, mirroring a real motor drawing more current than a robot merely
/// holding position.
pub const BATTERY_MOVE_DRAIN_PCT_PER_TICK: f32 = 0.02;

/// Item 20, Decision 6: how many consecutive ticks a robot in Autonomous mode tolerates
/// its best move being blocked by a locally-sensed peer before treating it as a head-on
/// standoff and applying the lower-`robot_id`-yields tie-break, rather than reacting to
/// the very first tick a peer happens to be in the way (which would misfire on any
/// ordinary crossing, not just a genuine standoff). Short and fixed, per Decision 6's own
/// wording — this is a liveness fallback, not a tuned parameter.
pub const AUTONOMOUS_YIELD_TIMEOUT_TICKS: u32 = 3;

/// Decision 3 (`docs/decisions.md`) — **PROVISIONAL, not final.** Packet-loss threshold
/// (as a percentage) above which `mode_state_machine.rs` (item 14) transitions a robot
/// from Cooperative to Cautious. A reasonable starting value, not yet measured on our
/// own system — Phase 8's `degradation_sweep.rs` is what confirms or replaces this
/// number. Do not treat as final before then.
pub const THRESHOLD_A_PACKET_LOSS_PCT: f64 = 20.0;

/// Decision 3 — **PROVISIONAL.** Packet-loss threshold above which
/// `mode_state_machine.rs` transitions Cautious to Autonomous. Same provenance and
/// caveat as `THRESHOLD_A_PACKET_LOSS_PCT` above.
pub const THRESHOLD_B_PACKET_LOSS_PCT: f64 = 60.0;

/// Decision 3 — **PROVISIONAL.** Latency, in ticks, that also triggers a Cooperative ->
/// Cautious transition regardless of packet loss. Same provenance and caveat as
/// `THRESHOLD_A_PACKET_LOSS_PCT` above.
pub const THRESHOLD_L_LATENCY_TICKS: u32 = 3;

/// Item 14 (`robot/mode_state_machine.rs`): the number of most-recent heartbeat
/// outcomes (received/missed) the mode state machine keeps to compute a rolling
/// packet-loss percentage. Chosen as `5` deliberately, not arbitrarily: the only
/// percentages a 5-slot window can produce are multiples of 20 (`0, 20, 40, 60, 80,
/// 100`), which land exactly on `THRESHOLD_A_PACKET_LOSS_PCT` (20) and
/// `THRESHOLD_B_PACKET_LOSS_PCT` (60) with no floating-point rounding ambiguity —
/// what makes `docs/TESTING_PLAN.md`'s Phase 4 gate ("transition points are exact,
/// not roughly around there") actually provable rather than approximate.
pub const HEARTBEAT_WINDOW_SIZE: usize = 5;

#[cfg(test)]
mod tests {
    use super::*;

    /// Phase 2 gate fallback (`docs/TESTING_PLAN.md`): `config.rs` has no logic to
    /// unit-test, so this checks the constants are internally consistent instead of
    /// being a no-op placeholder.
    #[test]
    fn thresholds_and_ports_are_sane() {
        assert!(
            THRESHOLD_A_PACKET_LOSS_PCT < THRESHOLD_B_PACKET_LOSS_PCT,
            "Cooperative->Cautious threshold must trigger before Cautious->Autonomous"
        );
        assert!(THRESHOLD_L_LATENCY_TICKS > 0);
        assert!(TICK_INTERVAL_MS > 0);
        assert_ne!(
            MULTICAST_PORT, DASHBOARD_HTTP_PORT,
            "multicast and dashboard HTTP must not share a port"
        );
        assert!(MULTICAST_ADDR.is_multicast());
        assert!(
            LOCAL_SENSING_RADIUS < COMMS_RANGE_CELLS,
            "local proximity sensing must be shorter-range than radio, not equal or wider"
        );
        assert!(
            BATTERY_MOVE_DRAIN_PCT_PER_TICK > BATTERY_IDLE_DRAIN_PCT_PER_TICK,
            "moving must drain battery faster than idling"
        );
        assert_ne!(
            BOOTSTRAP_TOKEN_ID, CLOCK_DRIVER_ID,
            "the two reserved sentinel ids on the bus must not collide"
        );
        assert_ne!(
            DASHBOARD_LISTENER_ID, CLOCK_DRIVER_ID,
            "reserved sentinel ids on the bus must not collide"
        );
        assert_ne!(
            DASHBOARD_LISTENER_ID, BOOTSTRAP_TOKEN_ID,
            "reserved sentinel ids on the bus must not collide"
        );
    }
}
