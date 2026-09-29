//! The Cooperative/Cautious/Autonomous degradation state machine
//! (`docs/PS_AND_ARCHITECTURE.md` §3.5), item 14 of `docs/BUILD_PLAN.md`.
//!
//! Deliberately sans-I/O: it never touches a socket or a `protocol::messages::Heartbeat`
//! directly. A caller (the robot tick loop, item 20) decides per tick whether an expected
//! heartbeat from a peer arrived and how late it was, and reports just that outcome here.
//! Keeping this pure is what makes it unit-testable with synthetic sequences, per
//! `docs/TESTING_PLAN.md`'s Phase 4 gate, without any real networking involved.

use std::collections::VecDeque;

use crate::config::{
    HEARTBEAT_WINDOW_SIZE, THRESHOLD_A_PACKET_LOSS_PCT, THRESHOLD_B_PACKET_LOSS_PCT,
    THRESHOLD_L_LATENCY_TICKS,
};
use crate::protocol::messages::Mode;

/// Severity ordering used to combine the loss-tier and latency-tier signals below —
/// `Mode` itself derives no `Ord` (it's a wire enum, `protocol/messages.rs` keeps it
/// free of ordering semantics that don't belong on the wire), so this stays local.
fn severity(mode: Mode) -> u8 {
    match mode {
        Mode::Cooperative => 0,
        Mode::Cautious => 1,
        Mode::Autonomous => 2,
    }
}

fn more_severe(a: Mode, b: Mode) -> Mode {
    if severity(a) >= severity(b) {
        a
    } else {
        b
    }
}

/// Tracks one robot's current degradation `Mode` and the recent heartbeat history that
/// justifies it.
pub struct ModeStateMachine {
    mode: Mode,
    /// Most recent heartbeat outcomes, `true` = received, oldest at the front. Capped at
    /// `HEARTBEAT_WINDOW_SIZE`.
    window: VecDeque<bool>,
}

impl ModeStateMachine {
    /// Every robot starts Cooperative — full comms assumed until a real observation says
    /// otherwise.
    pub fn new() -> Self {
        Self {
            mode: Mode::Cooperative,
            window: VecDeque::with_capacity(HEARTBEAT_WINDOW_SIZE),
        }
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Record the outcome of one expected heartbeat interval from a peer.
    ///
    /// `Some(n)` — the heartbeat arrived, `n` ticks late (`0` = on time). `None` — it
    /// didn't arrive this interval at all, counted as a loss in the rolling window.
    ///
    /// Returns `Some(new_mode)` only on the observation that actually changes the mode
    /// (what a caller uses to decide when to fire `protocol::messages::ModeAnnounce`,
    /// rather than announcing every tick), `None` if the mode is unchanged.
    pub fn observe_heartbeat(&mut self, latency_ticks: Option<u32>) -> Option<Mode> {
        if self.window.len() == HEARTBEAT_WINDOW_SIZE {
            self.window.pop_front();
        }
        self.window.push_back(latency_ticks.is_some());

        let received = self.window.iter().filter(|&&ok| ok).count();
        let loss_pct = 100.0 * (self.window.len() - received) as f64 / self.window.len() as f64;

        let loss_tier = if loss_pct >= THRESHOLD_B_PACKET_LOSS_PCT {
            Mode::Autonomous
        } else if loss_pct >= THRESHOLD_A_PACKET_LOSS_PCT {
            Mode::Cautious
        } else {
            Mode::Cooperative
        };

        let latency_tier = match latency_ticks {
            Some(n) if n >= THRESHOLD_L_LATENCY_TICKS => Mode::Cautious,
            _ => Mode::Cooperative,
        };

        let classified = more_severe(loss_tier, latency_tier);

        if classified != self.mode {
            self.mode = classified;
            Some(classified)
        } else {
            None
        }
    }
}

impl Default for ModeStateMachine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Lightweight smoke test — the real exact-threshold assertions live in the
    /// dedicated `tests/mode_state_machine.rs` (item 14's other half), same split as
    /// `planner_pibt.rs`'s in-file smoke test vs. `tests/pibt_single_process.rs`.
    #[test]
    fn starts_cooperative_and_reports_no_transition_while_healthy() {
        let mut sm = ModeStateMachine::new();
        assert_eq!(sm.mode(), Mode::Cooperative);
        for _ in 0..HEARTBEAT_WINDOW_SIZE {
            assert_eq!(sm.observe_heartbeat(Some(0)), None);
        }
        assert_eq!(sm.mode(), Mode::Cooperative);
    }
}
