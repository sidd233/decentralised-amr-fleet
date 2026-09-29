//! Item 23 of `docs/BUILD_PLAN.md`, `docs/PS_AND_ARCHITECTURE.md` §3.1's clock-driver
//! process: "exists only to keep simulation ticks synchronized for reproducibility — it
//! makes no planning or allocation decisions, so it is not a 'central coordinator' in the
//! sense the PS is asking us to avoid." True to that: this broadcasts a bare tick number
//! and nothing else — no map knowledge, no robot state, no decisions.
//!
//! Closes the gap `docs/decisions.md` Decision 8 flagged: `robot::RobotProcess::run`
//! previously self-paced with `std::thread::sleep`, which made "did two robots occupy the
//! same cell at the same tick" ill-defined across independently-drifting processes. Every
//! robot now *sets* its own tick counter from this broadcast (`RobotProcess::wait_for_tick`)
//! rather than incrementing one independently — a shared authoritative tick number, not
//! just similarly-paced sleeping.

use std::thread;
use std::time::Duration;

use crate::config::CLOCK_DRIVER_ID;
use crate::protocol::messages::TickMsg;
use crate::robot::comms::Comms;

/// Range-scoping radius for the clock driver's own `Comms`. Tick broadcasts are
/// fleet-wide by design, same reasoning as `task_layer.rs`'s `TOKEN_RANGE_CELLS` (a
/// robot at the far end of the warehouse still needs every tick, not just nearby ones) —
/// not `u32::MAX`, for the same overflow reason documented there.
const CLOCK_RANGE_CELLS: u32 = 100_000;

/// Broadcasts one `TickMsg` per `tick_interval`, starting at tick 0 immediately (no
/// initial wait — robots can start on the very first broadcast rather than idling through
/// one interval first).
pub struct ClockDriver {
    comms: Comms,
    tick_interval: Duration,
}

impl ClockDriver {
    pub fn new(tick_interval_ms: u64) -> std::io::Result<Self> {
        let comms = Comms::with_range(CLOCK_DRIVER_ID, (0, 0), CLOCK_RANGE_CELLS)?;
        Ok(ClockDriver {
            comms,
            tick_interval: Duration::from_millis(tick_interval_ms),
        })
    }

    /// Runs forever (`max_ticks = None`) or up to a bound. Same log-and-continue
    /// treatment of a failed send as every other broadcast site in this codebase
    /// (`docs/decisions.md` Decision 10) — a transient network failure skips one tick's
    /// broadcast, it doesn't crash the driver and stall the whole fleet.
    pub fn run(&self, max_ticks: Option<u64>) {
        let mut tick = 0u64;
        loop {
            if let Err(e) = self.comms.send_tick(TickMsg { tick }) {
                eprintln!("[clock] failed to send tick {tick}: {e}");
            }
            if let Some(max) = max_ticks {
                if tick >= max {
                    break;
                }
            }
            thread::sleep(self.tick_interval);
            tick += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::comms::Payload;
    use std::sync::Mutex;

    // Real UDP on the one shared hardcoded multicast port — same serialization
    // convention as `tests/comms_protocol.rs` and this codebase's other socket tests.
    static SEQUENTIAL: Mutex<()> = Mutex::new(());

    #[test]
    fn broadcasts_tick_zero_through_max_ticks_in_order() {
        let _guard = SEQUENTIAL.lock().unwrap();
        let receiver = Comms::with_range(1, (0, 0), CLOCK_RANGE_CELLS).expect("bind receiver");
        receiver
            .set_read_timeout(Some(Duration::from_millis(500)))
            .unwrap();

        let driver = ClockDriver::new(5).expect("bind driver"); // fast cadence for a quick test
        driver.run(Some(3));

        let mut seen = Vec::new();
        for _ in 0..4 {
            let received = receiver
                .recv_filtered()
                .expect("recv should not error")
                .expect("expected a tick broadcast");
            match received.payload {
                Payload::Tick(msg) => seen.push(msg.tick),
                other => panic!("expected Payload::Tick, got {other:?}"),
            }
        }
        assert_eq!(seen, vec![0, 1, 2, 3], "ticks must arrive in order, starting at 0");
    }
}
