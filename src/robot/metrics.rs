//! Item 19 of `docs/BUILD_PLAN.md`. First slice: the per-robot `BatteryModel`
//! (Decision 7, `docs/decisions.md`) — a real gap found cross-checking the architecture
//! against the PS itself, not the papers: `docs/PS_AND_ARCHITECTURE.md` §1.3 lists
//! "real-time positions + **battery status**" as part of the required "expected
//! solution," and the dashboard (item 24) would have had nothing real to show for it.
//!
//! `PS_AND_ARCHITECTURE.md` §3.2's other half of this module — "Metrics Emitter — feeds
//! the benchmark harness" — grows into this file incrementally as later items give it
//! real events to log (task completions from `task_layer.rs`, collision counts from the
//! planner, etc.), same growth pattern `config.rs` and `protocol/messages.rs` have
//! already had rather than being speculatively built out now with nothing yet to log.

use crate::config::{BATTERY_IDLE_DRAIN_PCT_PER_TICK, BATTERY_MOVE_DRAIN_PCT_PER_TICK};

/// Synthetic per-robot battery level, 0.0-100.0. Drain rates (`config.rs`) are
/// simulation-illustrative, not derived from a real battery spec — per
/// `PS_AND_ARCHITECTURE.md` §1.3, this whole project is explicitly a simulation, not
/// physical hardware, so a plausible, gradually-declining number is all the PS actually
/// asks the dashboard to show.
pub struct BatteryModel {
    percent: f32,
}

impl BatteryModel {
    /// Every robot starts fully charged.
    pub fn new() -> Self {
        BatteryModel { percent: 100.0 }
    }

    pub fn percent(&self) -> f32 {
        self.percent
    }

    /// Call once per tick with whether the robot actually moved this tick (vs. held its
    /// position) — movement drains faster than idling, mirroring a real motor drawing
    /// more current than a robot merely holding position. Clamps at `0.0` rather than
    /// going negative.
    pub fn drain_tick(&mut self, moved: bool) {
        let drain = if moved {
            BATTERY_MOVE_DRAIN_PCT_PER_TICK
        } else {
            BATTERY_IDLE_DRAIN_PCT_PER_TICK
        };
        self.percent = (self.percent - drain).max(0.0);
    }
}

impl Default for BatteryModel {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starts_fully_charged() {
        assert_eq!(BatteryModel::new().percent(), 100.0);
    }

    #[test]
    fn moving_drains_faster_than_idling() {
        let mut moving = BatteryModel::new();
        let mut idle = BatteryModel::new();
        moving.drain_tick(true);
        idle.drain_tick(false);
        assert!(
            moving.percent() < idle.percent(),
            "moving ({}) should drain faster than idling ({})",
            moving.percent(),
            idle.percent()
        );
    }

    #[test]
    fn drains_by_exactly_the_configured_rate_per_tick() {
        let mut battery = BatteryModel::new();
        battery.drain_tick(false);
        assert_eq!(battery.percent(), 100.0 - BATTERY_IDLE_DRAIN_PCT_PER_TICK);

        let mut battery = BatteryModel::new();
        battery.drain_tick(true);
        assert_eq!(battery.percent(), 100.0 - BATTERY_MOVE_DRAIN_PCT_PER_TICK);
    }

    #[test]
    fn never_goes_negative() {
        let mut battery = BatteryModel::new();
        for _ in 0..1_000_000 {
            battery.drain_tick(true);
        }
        assert_eq!(battery.percent(), 0.0);
    }
}
