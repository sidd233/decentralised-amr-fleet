//! Mode-dependent visibility. Item 18 of `docs/BUILD_PLAN.md`,
//! `docs/PS_AND_ARCHITECTURE.md` §3.2's Perception module: "full map + peer intents
//! (Cooperative) down to local-sensing-radius only (Autonomous)."
//!
//! Deliberately sans-I/O, same discipline as `mode_state_machine.rs`: this never touches
//! a socket itself. A caller (the future robot tick loop, item 20) hands it whatever it
//! already drained from `robot::comms::Comms::recv_filtered` this tick, plus the robot's
//! current `Mode` (from `mode_state_machine.rs`) and position, and gets back a
//! `PerceivedState` describing what the robot is allowed to trust this tick — the seam
//! `docs/decisions.md` Decision 6 pointed at for wiring up Autonomous mode's
//! local-sensing-only visibility (needed for that decision's head-on-deadlock tie-break,
//! not yet implemented here — that lands in the planner at item 20).
//!
//! `LocalOnly`'s obstacle list is built from *any* message's `sender_position`
//! (`comms::Received` carries one on every payload type, not just `PoseIntent`), not
//! just intent broadcasts: real local proximity sensing wouldn't care what a robot is
//! broadcasting, only where it physically is — restricting this to `PoseIntent` alone
//! would be an arbitrary, unjustified narrowing of what "I can see something over there"
//! actually means.

use std::collections::HashMap;

use crate::config::LOCAL_SENSING_RADIUS;
use crate::protocol::messages::{Mode, PoseIntent};
use crate::robot::comms::{Payload, Received};
use crate::robot::planner_pibt::Cell;

fn wire_to_cell(c: (i32, i32)) -> Cell {
    (c.0 as usize, c.1 as usize)
}

fn within_local_sensing_radius(a: Cell, b: Cell) -> bool {
    let dx = a.0 as i64 - b.0 as i64;
    let dy = a.1 as i64 - b.1 as i64;
    let r = LOCAL_SENSING_RADIUS as i64;
    dx * dx + dy * dy <= r * r
}

/// What a robot is allowed to trust this tick, per its current degradation `Mode`.
#[derive(Debug, Clone, PartialEq)]
pub enum PerceivedState {
    /// Cooperative or Cautious: full trust in peer intent, unfiltered beyond whatever
    /// range-scoping `comms::Comms` itself already applied before delivering these.
    Networked { peer_intents: Vec<PoseIntent> },
    /// Autonomous: peer intent is ignored entirely (comms can't be trusted to have
    /// delivered it reliably) — only physically-nearby peers matter, at a much shorter
    /// radius than the comms range, and only their current position, never their intent.
    LocalOnly { nearby_obstacles: Vec<(u32, Cell)> },
}

/// Computes this tick's `PerceivedState` from whatever was already received over comms
/// this tick and the robot's current mode and position. No I/O — pure, unit-testable.
pub fn perceive(mode: Mode, own_position: Cell, received_this_tick: &[Received]) -> PerceivedState {
    match mode {
        Mode::Cooperative | Mode::Cautious => {
            let peer_intents = received_this_tick
                .iter()
                .filter_map(|r| match r.payload {
                    Payload::Pose(p) => Some(p),
                    _ => None,
                })
                .collect();
            PerceivedState::Networked { peer_intents }
        }
        Mode::Autonomous => {
            // Dedup by sender: a robot may have broadcast several messages this tick,
            // but it's one physical peer at one position, not several obstacles.
            let mut nearby: HashMap<u32, Cell> = HashMap::new();
            for r in received_this_tick {
                let sender_cell = wire_to_cell(r.sender_position);
                if within_local_sensing_radius(own_position, sender_cell) {
                    nearby.insert(r.sender_id, sender_cell);
                }
            }
            let mut nearby_obstacles: Vec<(u32, Cell)> = nearby.into_iter().collect();
            nearby_obstacles.sort_by_key(|&(id, _)| id); // deterministic order for callers/tests
            PerceivedState::LocalOnly { nearby_obstacles }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pose_from(sender_id: u32, sender_position: (i32, i32)) -> Received {
        Received {
            sender_id,
            sender_position,
            payload: Payload::Pose(PoseIntent {
                robot_id: sender_id,
                tick: 1,
                position: sender_position,
                intended_next: sender_position,
                priority: 0.0,
                exhausted: false,
            }),
        }
    }

    fn heartbeat_from(sender_id: u32, sender_position: (i32, i32)) -> Received {
        Received {
            sender_id,
            sender_position,
            payload: Payload::Heartbeat(crate::protocol::messages::Heartbeat {
                robot_id: sender_id,
                tick: 1,
                battery_pct: 100.0,
            }),
        }
    }

    #[test]
    fn cooperative_mode_trusts_every_peer_intent_regardless_of_distance() {
        let far = pose_from(2, (1000, 1000));
        let received = vec![pose_from(1, (5, 5)), far];
        // Distance is irrelevant here: `Comms` itself already range-scoped what reaches
        // this function, so a real "far" entry shouldn't exist in practice — but the
        // point of this test is that `perceive` doesn't re-filter and drop it either.
        let state = perceive(Mode::Cooperative, (0, 0), &received);
        match state {
            PerceivedState::Networked { peer_intents } => assert_eq!(peer_intents.len(), 2),
            other => panic!("expected Networked, got {other:?}"),
        }
    }

    #[test]
    fn cautious_mode_behaves_the_same_as_cooperative_for_visibility() {
        let received = vec![pose_from(1, (5, 5))];
        assert_eq!(
            perceive(Mode::Cooperative, (0, 0), &received),
            perceive(Mode::Cautious, (0, 0), &received),
            "the PS_AND_ARCHITECTURE.md table only distinguishes Cautious by margin/speed \
             policy, not visibility — perception itself should treat them identically"
        );
    }

    #[test]
    fn networked_mode_ignores_non_pose_payloads() {
        let received = vec![pose_from(1, (1, 1)), heartbeat_from(2, (2, 2))];
        let state = perceive(Mode::Cooperative, (0, 0), &received);
        match state {
            PerceivedState::Networked { peer_intents } => {
                assert_eq!(peer_intents.len(), 1, "a Heartbeat is not a peer intent")
            }
            other => panic!("expected Networked, got {other:?}"),
        }
    }

    #[test]
    fn autonomous_mode_ignores_intent_and_uses_short_range_local_sensing() {
        let own = (10usize, 10usize);
        // Within LOCAL_SENSING_RADIUS (3): should be visible.
        let near = pose_from(1, (11, 10));
        // Within comms range but outside local sensing radius: must be filtered out —
        // this is the entire point of Autonomous mode's shorter-range fallback.
        let far = pose_from(2, (18, 10));
        let received = vec![near, far];

        let state = perceive(Mode::Autonomous, own, &received);
        match state {
            PerceivedState::LocalOnly { nearby_obstacles } => {
                assert_eq!(nearby_obstacles, vec![(1, (11, 10))]);
            }
            other => panic!("expected LocalOnly, got {other:?}"),
        }
    }

    #[test]
    fn autonomous_mode_senses_any_payload_type_not_only_pose_intents() {
        let own = (0usize, 0usize);
        // A Heartbeat still reveals its sender's real position; local sensing should
        // pick it up exactly like it would a PoseIntent.
        let received = vec![heartbeat_from(9, (1, 0))];
        let state = perceive(Mode::Autonomous, own, &received);
        match state {
            PerceivedState::LocalOnly { nearby_obstacles } => {
                assert_eq!(nearby_obstacles, vec![(9, (1, 0))]);
            }
            other => panic!("expected LocalOnly, got {other:?}"),
        }
    }

    #[test]
    fn autonomous_mode_deduplicates_multiple_messages_from_the_same_sender() {
        let own = (0usize, 0usize);
        let received = vec![pose_from(1, (1, 0)), heartbeat_from(1, (1, 0))];
        let state = perceive(Mode::Autonomous, own, &received);
        match state {
            PerceivedState::LocalOnly { nearby_obstacles } => {
                assert_eq!(nearby_obstacles.len(), 1, "one physical peer, not one per message")
            }
            other => panic!("expected LocalOnly, got {other:?}"),
        }
    }

    #[test]
    fn autonomous_mode_boundary_is_inclusive_at_exactly_the_sensing_radius() {
        let own = (0usize, 0usize);
        // Exactly LOCAL_SENSING_RADIUS (3) cells away, axis-aligned.
        let at_boundary = pose_from(1, (3, 0));
        // One cell past it.
        let just_outside = pose_from(2, (4, 0));
        let received = vec![at_boundary, just_outside];

        let state = perceive(Mode::Autonomous, own, &received);
        match state {
            PerceivedState::LocalOnly { nearby_obstacles } => {
                assert_eq!(nearby_obstacles, vec![(1, (3, 0))]);
            }
            other => panic!("expected LocalOnly, got {other:?}"),
        }
    }
}
