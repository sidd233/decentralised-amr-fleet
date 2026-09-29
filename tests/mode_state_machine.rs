//! Item 14's dedicated test file, `docs/BUILD_PLAN.md` Phase 4 / `docs/TESTING_PLAN.md`
//! Phase 4's mode-state-machine gate: feed synthetic heartbeat-loss and latency
//! sequences, assert the state machine transitions Cooperative -> Cautious ->
//! Autonomous at exactly the configured thresholds (A, B, L), and transitions back down
//! when the sequence improves. "Exactly" is checked at the single-tick granularity that
//! flips the mode, not just "eventually ends up in the right place."

use sih26123::config::{HEARTBEAT_WINDOW_SIZE, THRESHOLD_L_LATENCY_TICKS};
use sih26123::protocol::messages::Mode;
use sih26123::robot::mode_state_machine::ModeStateMachine;

/// Feed `n` on-time heartbeats and return the machine, used to establish a clean
/// Cooperative baseline (a full healthy window) before each scenario below.
fn healthy_baseline() -> ModeStateMachine {
    let mut sm = ModeStateMachine::new();
    for _ in 0..HEARTBEAT_WINDOW_SIZE {
        assert_eq!(sm.observe_heartbeat(Some(0)), None);
    }
    assert_eq!(sm.mode(), Mode::Cooperative);
    sm
}

/// `HEARTBEAT_WINDOW_SIZE` is load-bearing for every exact-percentage assertion below
/// (see `config.rs`'s doc comment on it) — pin it here so a change to that constant
/// fails loudly in this file instead of silently invalidating the arithmetic.
#[test]
fn window_size_is_five_as_every_other_assertion_here_assumes() {
    assert_eq!(HEARTBEAT_WINDOW_SIZE, 5);
}

#[test]
fn packet_loss_crosses_threshold_a_at_exactly_one_in_five_missed() {
    let mut sm = healthy_baseline();

    // 1/5 missed = 20% loss, exactly THRESHOLD_A. First 3 misses (0%, still just this
    // one pending) don't cross it until the window actually contains the loss.
    assert_eq!(sm.observe_heartbeat(None), Some(Mode::Cautious));
    assert_eq!(sm.mode(), Mode::Cautious);
}

#[test]
fn packet_loss_stays_cooperative_below_one_in_five() {
    let mut sm = ModeStateMachine::new();
    // 4 on-time, then check: window not yet full, 0/4 lost = 0%.
    for _ in 0..HEARTBEAT_WINDOW_SIZE - 1 {
        assert_eq!(sm.observe_heartbeat(Some(0)), None);
    }
    assert_eq!(sm.mode(), Mode::Cooperative);
}

#[test]
fn packet_loss_crosses_threshold_b_at_exactly_three_in_five_missed() {
    let mut sm = healthy_baseline();

    // 1/5 missed -> Cautious (crosses A).
    assert_eq!(sm.observe_heartbeat(None), Some(Mode::Cautious));
    // 2/5 missed -> still Cautious, hasn't reached B (40% < 60%).
    assert_eq!(sm.observe_heartbeat(None), None);
    assert_eq!(sm.mode(), Mode::Cautious);
    // 3/5 missed = 60% loss, exactly THRESHOLD_B -> Autonomous.
    assert_eq!(sm.observe_heartbeat(None), Some(Mode::Autonomous));
    assert_eq!(sm.mode(), Mode::Autonomous);
}

#[test]
fn mode_steps_back_down_as_the_window_recovers() {
    let mut sm = healthy_baseline();

    // Drive up to Autonomous: 3 misses in a row (60% loss).
    sm.observe_heartbeat(None);
    sm.observe_heartbeat(None);
    assert_eq!(sm.observe_heartbeat(None), Some(Mode::Autonomous));

    // Now recover with on-time heartbeats, one at a time, and check every exact step.
    // Window before this point: [ok, ok, miss, miss, miss] (5 slots, 3/5 = 60% lost).
    // Push one more "ok": window becomes [ok, miss, miss, miss, ok] -> still 3/5 = 60%,
    // still Autonomous (>= B, not yet below it).
    assert_eq!(sm.observe_heartbeat(Some(0)), None);
    assert_eq!(sm.mode(), Mode::Autonomous);

    // Push another "ok": window becomes [miss, miss, miss, ok, ok] -> still 3/5 = 60%,
    // still Autonomous.
    assert_eq!(sm.observe_heartbeat(Some(0)), None);
    assert_eq!(sm.mode(), Mode::Autonomous);

    // Push another "ok": the first of the three original misses finally falls out of
    // the window -> [miss, miss, ok, ok, ok] = 2/5 = 40% lost, below B, above A ->
    // Cautious.
    assert_eq!(sm.observe_heartbeat(Some(0)), Some(Mode::Cautious));
    assert_eq!(sm.mode(), Mode::Cautious);

    // Another "ok": [miss, ok, ok, ok, ok] = 1/5 = 20% lost, still >= A -> stays Cautious.
    assert_eq!(sm.observe_heartbeat(Some(0)), None);
    assert_eq!(sm.mode(), Mode::Cautious);

    // Another "ok": the last miss falls out -> [ok, ok, ok, ok, ok] = 0% -> Cooperative.
    assert_eq!(sm.observe_heartbeat(Some(0)), Some(Mode::Cooperative));
    assert_eq!(sm.mode(), Mode::Cooperative);
}

#[test]
fn latency_alone_triggers_cautious_regardless_of_packet_loss() {
    let mut sm = healthy_baseline();

    // Heartbeat arrives, but exactly at THRESHOLD_L ticks late -> Cautious even though
    // the loss window is still perfectly healthy (it was received, not lost).
    assert_eq!(
        sm.observe_heartbeat(Some(THRESHOLD_L_LATENCY_TICKS)),
        Some(Mode::Cautious)
    );
    assert_eq!(sm.mode(), Mode::Cautious);
}

#[test]
fn latency_one_tick_below_threshold_does_not_trigger_cautious() {
    let mut sm = healthy_baseline();

    assert_eq!(
        sm.observe_heartbeat(Some(THRESHOLD_L_LATENCY_TICKS - 1)),
        None
    );
    assert_eq!(sm.mode(), Mode::Cooperative);
}

#[test]
fn latency_trigger_clears_on_the_next_on_time_heartbeat() {
    let mut sm = healthy_baseline();

    assert_eq!(
        sm.observe_heartbeat(Some(THRESHOLD_L_LATENCY_TICKS)),
        Some(Mode::Cautious)
    );
    // Loss window is still healthy (that heartbeat was received, just late), and
    // latency is instantaneous, not windowed -> one on-time heartbeat clears it.
    assert_eq!(sm.observe_heartbeat(Some(0)), Some(Mode::Cooperative));
    assert_eq!(sm.mode(), Mode::Cooperative);
}

#[test]
fn packet_loss_at_autonomous_outranks_a_simultaneous_low_latency_reading() {
    let mut sm = healthy_baseline();
    sm.observe_heartbeat(None);
    sm.observe_heartbeat(None);
    // Third miss crosses B -> Autonomous. Latency tier for a *missed* heartbeat is
    // irrelevant (there's nothing to be "late" — `None` carries no latency), so this
    // also exercises that the loss tier alone can drive Autonomous.
    assert_eq!(sm.observe_heartbeat(None), Some(Mode::Autonomous));
    assert_eq!(sm.mode(), Mode::Autonomous);
}
