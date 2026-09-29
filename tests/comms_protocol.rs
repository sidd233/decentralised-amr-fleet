//! `tests/comms_protocol.rs` — item 12 of `docs/BUILD_PLAN.md`, Phase 4's gate for
//! `robot/comms.rs` (item 11). Real `socket2`/`UdpSocket` multicast traffic over
//! loopback, not an in-memory stand-in (`CLAUDE.md`: no shortcutting real UDP sockets).
//!
//! Every test here binds to the one shared multicast group/port `config.rs` hardcodes
//! (item 11 doesn't do per-test ports — there's exactly one shared bus, by design), and
//! `cargo test` runs test functions on parallel threads within one process by default.
//! Without serializing, one test's broadcast could land in another concurrently-running
//! test's `recv_filtered` call and produce a flaky false pass or fail. `SEQUENTIAL` forces
//! these tests to run one at a time instead.

use sih26123::protocol::messages::{ClaimEntry, Heartbeat, Mode, ModeAnnounce, PoseIntent, TokenMsg};
use sih26123::robot::comms::{Comms, Payload};
use std::sync::Mutex;
use std::time::Duration;

static SEQUENTIAL: Mutex<()> = Mutex::new(());
const RECV_TIMEOUT: Duration = Duration::from_millis(500);

#[test]
fn pose_intent_round_trips_over_socket() {
    let _guard = SEQUENTIAL.lock().unwrap();
    let sender = Comms::new(1, (0, 0)).expect("bind sender");
    let receiver = Comms::new(2, (0, 0)).expect("bind receiver");
    receiver.set_read_timeout(Some(RECV_TIMEOUT)).unwrap();

    let msg = PoseIntent {
        robot_id: 1,
        tick: 5,
        position: (0, 0),
        intended_next: (0, 1),
        priority: 3.0,
        exhausted: false,
    };
    sender.send_pose_intent(msg).expect("send pose intent");

    let received = receiver
        .recv_filtered()
        .expect("recv_from should not error")
        .expect("expected a message, got None (filtered or undecodable)");
    assert_eq!(received.sender_id, 1);
    match received.payload {
        Payload::Pose(p) => assert_eq!(p, msg),
        other => panic!("expected Payload::Pose, got {other:?}"),
    }
}

#[test]
fn token_msg_round_trips_over_socket() {
    let _guard = SEQUENTIAL.lock().unwrap();
    let sender = Comms::new(1, (0, 0)).expect("bind sender");
    let receiver = Comms::new(2, (0, 0)).expect("bind receiver");
    receiver.set_read_timeout(Some(RECV_TIMEOUT)).unwrap();

    let msg = TokenMsg {
        seq: 7,
        holder_id: 1,
        claimed_tasks: vec![
            ClaimEntry { task_id: 3, robot_id: 1, picked_up: false },
            ClaimEntry { task_id: 4, robot_id: 2, picked_up: true },
        ],
        epoch: 0,
        creator: 0,
    };
    sender.send_token(msg.clone()).expect("send token");

    let received = receiver
        .recv_filtered()
        .expect("recv_from should not error")
        .expect("expected a message");
    assert_eq!(received.sender_id, 1);
    match received.payload {
        Payload::Token(t) => assert_eq!(t, msg),
        other => panic!("expected Payload::Token, got {other:?}"),
    }
}

#[test]
fn heartbeat_round_trips_over_socket() {
    let _guard = SEQUENTIAL.lock().unwrap();
    let sender = Comms::new(1, (0, 0)).expect("bind sender");
    let receiver = Comms::new(2, (0, 0)).expect("bind receiver");
    receiver.set_read_timeout(Some(RECV_TIMEOUT)).unwrap();

    let msg = Heartbeat {
        robot_id: 1,
        tick: 99,
        battery_pct: 100.0,
    };
    sender.send_heartbeat(msg).expect("send heartbeat");

    let received = receiver
        .recv_filtered()
        .expect("recv_from should not error")
        .expect("expected a message");
    assert_eq!(received.sender_id, 1);
    match received.payload {
        Payload::Heartbeat(h) => assert_eq!(h, msg),
        other => panic!("expected Payload::Heartbeat, got {other:?}"),
    }
}

#[test]
fn mode_announce_round_trips_over_socket() {
    let _guard = SEQUENTIAL.lock().unwrap();
    let sender = Comms::new(1, (0, 0)).expect("bind sender");
    let receiver = Comms::new(2, (0, 0)).expect("bind receiver");
    receiver.set_read_timeout(Some(RECV_TIMEOUT)).unwrap();

    let msg = ModeAnnounce {
        robot_id: 1,
        mode: Mode::Cautious,
        tick: 12,
    };
    sender.send_mode_announce(msg).expect("send mode announce");

    let received = receiver
        .recv_filtered()
        .expect("recv_from should not error")
        .expect("expected a message");
    assert_eq!(received.sender_id, 1);
    match received.payload {
        Payload::ModeAnnounce(m) => assert_eq!(m, msg),
        other => panic!("expected Payload::ModeAnnounce, got {other:?}"),
    }
}

/// TESTING_PLAN.md Phase 4: "Two processes on the same machine, bound via SO_REUSEPORT,
/// can send and receive a multicast message to each other." Two independently-bound
/// sockets (real `SO_REUSEPORT`, both actually joining the multicast group — the same
/// mechanism separate robot OS processes use) exchange messages in both directions.
#[test]
fn two_reuseport_sockets_exchange_messages_bidirectionally() {
    let _guard = SEQUENTIAL.lock().unwrap();
    let robot_a = Comms::new(1, (0, 0)).expect("bind robot A");
    let robot_b = Comms::new(2, (0, 0)).expect("bind robot B");
    robot_a.set_read_timeout(Some(RECV_TIMEOUT)).unwrap();
    robot_b.set_read_timeout(Some(RECV_TIMEOUT)).unwrap();

    robot_a
        .send_heartbeat(Heartbeat {
            robot_id: 1,
            tick: 1,
            battery_pct: 100.0,
        })
        .expect("A sends");
    let at_b = robot_b
        .recv_filtered()
        .expect("recv_from should not error")
        .expect("B should receive A's broadcast");
    assert_eq!(at_b.sender_id, 1);

    robot_b
        .send_heartbeat(Heartbeat {
            robot_id: 2,
            tick: 2,
            battery_pct: 100.0,
        })
        .expect("B sends");
    let at_a = robot_a
        .recv_filtered()
        .expect("recv_from should not error")
        .expect("A should receive B's broadcast");
    assert_eq!(at_a.sender_id, 2);
}

/// TESTING_PLAN.md Phase 4: "Range-scoping: a robot outside the configured radius does
/// not receive a broadcast (simulate by setting radius artificially small in the test)."
///
/// `recv_filtered` retries internally past anything it filters (self-loopback,
/// out-of-range senders — see its doc comment), so `Ok(None)` on its own no longer
/// distinguishes "filtered" from "nothing sent yet". Proven directly instead: queue the
/// far sender's datagram *and then* the near sender's, both before the receiver reads
/// anything. A single `recv_filtered` call must skip the far one and surface the near
/// one — which is only possible if the far datagram really was received off the wire and
/// discarded, not merely never sent.
#[test]
fn range_scoping_filters_far_sender_but_not_near_one() {
    let _guard = SEQUENTIAL.lock().unwrap();
    let far_sender = Comms::new(10, (100, 0)).expect("bind far sender");
    let near_sender = Comms::new(11, (3, 0)).expect("bind near sender");
    let receiver = Comms::with_range(12, (0, 0), 5).expect("bind receiver, range 5");
    receiver.set_read_timeout(Some(RECV_TIMEOUT)).unwrap();

    far_sender
        .send_heartbeat(Heartbeat {
            robot_id: 10,
            tick: 1,
            battery_pct: 100.0,
        })
        .expect("far sender sends");
    near_sender
        .send_heartbeat(Heartbeat {
            robot_id: 11,
            tick: 2,
            battery_pct: 100.0,
        })
        .expect("near sender sends");

    let received = receiver
        .recv_filtered()
        .expect("recv_from should not error")
        .expect("the near sender's message should still be delivered");
    assert_eq!(
        received.sender_id, 11,
        "far sender (100 cells away) must be filtered; only the near sender (3 cells) should surface"
    );
}

/// Regression for the Docker e2e stall (`tests/no_collisions_e2e.rs`): a robot far from
/// every peer drops each packet as out-of-range *inside* `recv_filtered`'s own loop, and
/// the socket read timeout only bounds each `recv_from`, not the whole call. Peers
/// sending faster than that timeout kept the call from ever returning (observed: one
/// call blocked 29.9s). The timeout must bound the call as a whole.
#[test]
fn recv_filtered_honors_timeout_under_out_of_range_flood() {
    let _guard = SEQUENTIAL.lock().unwrap();
    let far_sender = Comms::new(1, (1000, 1000)).expect("bind sender");
    let receiver = Comms::with_range(2, (0, 0), 5).expect("bind receiver");
    receiver.set_read_timeout(Some(Duration::from_millis(50))).unwrap();

    let flood_for = Duration::from_secs(2);
    let flooder = std::thread::spawn(move || {
        let end = std::time::Instant::now() + flood_for;
        while std::time::Instant::now() < end {
            let _ = far_sender.send_heartbeat(Heartbeat { robot_id: 1, tick: 0, battery_pct: 100.0 });
            std::thread::sleep(Duration::from_millis(1));
        }
    });

    std::thread::sleep(Duration::from_millis(50)); // let the flood start
    let started = std::time::Instant::now();
    let result = receiver.recv_filtered();
    let elapsed = started.elapsed();
    flooder.join().unwrap();

    assert!(result.is_err(), "nothing in range was sent, expected a timeout error, got {result:?}");
    assert!(
        elapsed < Duration::from_millis(400),
        "recv_filtered took {elapsed:?} with a 50ms read timeout; the timeout must bound the whole call"
    );
}
