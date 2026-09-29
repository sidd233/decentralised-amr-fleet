//! Items 15-17's dedicated test file, `docs/TESTING_PLAN.md` Phase 5's gate for
//! `robot/task_layer.rs`. Four required behaviors, each observed directly rather than
//! inferred from absence of errors:
//! 1. Token reaches every idle robot in bounded time (no starvation).
//! 2. A claimed task correctly transitions the claiming robot from idle to goal-directed.
//! 3. An unreachable task returns to the pool and is re-announced, not silently dropped.
//! 4. Token loss (simulated) triggers retry, not a stalled fleet.
//!
//! Behaviors 1-3 are checked against `TaskLayer::handle_token` directly — pure logic, no
//! sockets involved, same style as `tests/pibt_single_process.rs`'s original
//! single-process scenarios. Behavior 4 needs real UDP timing, so it uses
//! `run_one_cycle` over real sockets, serialized behind the same `Mutex` pattern
//! `tests/comms_protocol.rs` and `tests/pibt_single_process.rs`'s `_distributed` tests
//! use, since every test here shares the one hardcoded multicast bus.

use std::collections::HashSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use sih26123::protocol::messages::{Ack, ClaimEntry, TokenMsg};
use sih26123::robot::comms::{Comms, Payload};
use sih26123::robot::planner_pibt::Cell;
use sih26123::robot::task_layer::{Task, TaskLayer, TaskState, TokenCycleOutcome, TOKEN_RANGE_CELLS};
use sih26123::world::grid::Grid;

static SEQUENTIAL: Mutex<()> = Mutex::new(());

fn test_grid() -> Grid {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map");
    Grid::load(path).expect("map should load")
}

fn build_fleet(grid: &Grid, tasks: Vec<Task>, n: u32, positions: &[Cell]) -> Vec<TaskLayer> {
    let peer_ids: Vec<u32> = (1..=n).collect();
    peer_ids
        .iter()
        .enumerate()
        .map(|(i, &id)| {
            TaskLayer::new(id, grid, tasks.clone(), peer_ids.clone(), positions[i])
                .expect("bind task layer")
        })
        .collect()
}

fn bootstrap_token() -> TokenMsg {
    TokenMsg {
        seq: 0,
        holder_id: 1,
        claimed_tasks: vec![],
        epoch: 0,
        creator: 0,
    }
}

#[test]
fn token_reaches_every_idle_robot_within_one_ring_cycle() {
    let _guard = SEQUENTIAL.lock().unwrap();
    let grid = test_grid();
    let free = grid.free_cells();
    let n = 5;
    let positions: Vec<Cell> = (0..n).map(|i| free[i * 37 % free.len()]).collect();
    // No tasks at all: every robot stays idle, so this isolates pure circulation.
    let mut fleet = build_fleet(&grid, vec![], n as u32, &positions);

    let mut token = bootstrap_token();
    let mut visited = HashSet::new();
    for _ in 0..fleet.len() {
        let holder_idx = (token.holder_id - 1) as usize;
        visited.insert(token.holder_id);
        token = fleet[holder_idx].handle_token(&token, positions[holder_idx]);
    }

    assert_eq!(
        visited.len(),
        fleet.len(),
        "every robot must have held the token exactly once within one ring cycle"
    );
}

#[test]
fn claiming_a_task_transitions_robot_from_idle_to_goal_directed() {
    let _guard = SEQUENTIAL.lock().unwrap();
    let grid = test_grid();
    let free = grid.free_cells();
    let start = free[0];
    let task = Task {
        task_id: 1,
        pickup: free[10],
        dropoff: free[20],
    };
    let mut fleet = build_fleet(&grid, vec![task], 1, &[start]);

    assert_eq!(fleet[0].state(), TaskState::Idle);
    assert_eq!(fleet[0].current_goal(), None);

    let token = bootstrap_token();
    let outgoing = fleet[0].handle_token(&token, start);

    assert_eq!(
        outgoing.claimed_tasks,
        vec![ClaimEntry { task_id: 1, robot_id: 1, picked_up: false }]
    );
    assert_eq!(fleet[0].state(), TaskState::ToPickup(task));
    assert_eq!(
        fleet[0].current_goal(),
        Some(task.pickup),
        "goal-directed: planner should now steer toward the task's pickup cell"
    );

    // Arriving at pickup advances to ToDropoff; arriving at dropoff completes the task.
    fleet[0].on_position_update(task.pickup);
    assert_eq!(fleet[0].state(), TaskState::ToDropoff(task));
    fleet[0].on_position_update(task.dropoff);
    assert_eq!(
        fleet[0].state(),
        TaskState::Idle,
        "task complete, robot idle again"
    );
}

#[test]
fn unreachable_task_returns_to_pool_and_is_re_announced() {
    let _guard = SEQUENTIAL.lock().unwrap();
    let grid = test_grid();
    let free = grid.free_cells();
    let start = free[0];
    let task = Task {
        task_id: 1,
        pickup: free[10],
        dropoff: free[20],
    };

    let mut robot1 = TaskLayer::new(1, &grid, vec![task], vec![1, 2], start).expect("bind robot 1");

    // Claim the task first, exactly as behavior 2's test already proves.
    let outgoing = robot1.handle_token(&bootstrap_token(), start);
    assert_eq!(robot1.state(), TaskState::ToPickup(task));
    assert_eq!(
        outgoing.claimed_tasks,
        vec![ClaimEntry { task_id: 1, robot_id: 1, picked_up: false }]
    );

    // Simulate an aisle block making the pickup itself unreachable (item 17's "aisle
    // block -> grid.rs recompute" half): `Grid::bfs_distance` requires its goal cell to
    // be free, so blocking the pickup cell directly is the simplest faithful way to make
    // it unreachable — the same downstream effect a real surrounding block would have.
    robot1.set_blocked(task.pickup.0, task.pickup.1, true);

    // Next time robot 1 holds the token, it must release the now-unreachable task.
    let released_token = robot1.handle_token(&outgoing, start);
    assert_eq!(
        robot1.state(),
        TaskState::Idle,
        "task released back to idle once its pickup became unreachable"
    );
    assert!(
        !released_token
            .claimed_tasks
            .contains(&ClaimEntry { task_id: 1, robot_id: 1, picked_up: false }),
        "the released task's claim must be gone from the outgoing token, not silently kept: {:?}",
        released_token.claimed_tasks
    );
}

/// Token loss (a dropped/delayed forward-hop) triggers retry, not a stalled fleet. Real
/// UDP timing: robot A holds the token and forwards it to robot B; B deliberately
/// doesn't start listening until after several of A's retry attempts would already have
/// gone unanswered, standing in for lost delivery. A's own `run_one_cycle` (started by a
/// third, out-of-ring `Comms` that bootstraps the token to A) must keep re-broadcasting,
/// not give up after one try, for B to ever pick it up.
#[test]
fn dropped_forward_hop_is_retried_until_the_next_peer_picks_it_up() {
    let _guard = SEQUENTIAL.lock().unwrap();
    let grid = test_grid();
    let free = grid.free_cells();
    let pos_a = free[0];
    let pos_b = free[1];

    let mut robot_a = TaskLayer::new(1, &grid, vec![], vec![1, 2], pos_a).expect("bind A");
    let mut robot_b = TaskLayer::new(2, &grid, vec![], vec![1, 2], pos_b).expect("bind B");
    // A third party (not 1 or 2) originates the bootstrap token onto the wire — a real
    // robot can't hear its own broadcast (`Comms::recv_filtered` drops self-loopback by
    // design), so something else has to be the first sender, same as a harness would.
    let bootstrapper = Comms::new(99, (0, 0)).expect("bind bootstrapper");

    let handle_a = thread::spawn(move || {
        // Wait long enough for the bootstrapper's send below to land, then process and
        // forward — retrying every `TOKEN_RETRY_INTERVAL` (20ms) internally.
        robot_a.run_one_cycle(pos_a, Duration::from_secs(2))
    });

    let handle_b = thread::spawn(move || {
        // Deliberately arrive late: A's first several 20ms-spaced retries go out before
        // B ever starts listening, standing in for lost/delayed delivery rather than
        // instant first-try success.
        thread::sleep(Duration::from_millis(150));
        robot_b.run_one_cycle(pos_b, Duration::from_secs(2))
    });

    bootstrapper
        .send_token(TokenMsg {
            seq: 0,
            holder_id: 1,
            claimed_tasks: vec![],
            epoch: 0,
            creator: 0,
        })
        .expect("bootstrapper sends initial token to A");

    let (outcome_a, _incidentals_a) = handle_a.join().expect("robot A thread panicked");
    let (outcome_b, _incidentals_b) = handle_b.join().expect("robot B thread panicked");

    // A must have delivered to B within its backoff budget despite B's late start — the
    // actual behavior this test exists to prove.
    match outcome_a {
        TokenCycleOutcome::Handled { .. } => {}
        TokenCycleOutcome::NoTokenArrived => panic!("A never received the bootstrap token"),
        TokenCycleOutcome::Skipped { .. } => panic!(
            "A should have delivered to B within its backoff budget once B started listening, not skipped it"
        ),
        TokenCycleOutcome::Regenerated { .. } => {
            panic!("A should not have hit the watchdog in this short, healthy scenario")
        }
    }
    // B's own forward-hop (to A) is a separate matter: this test only calls
    // `run_one_cycle` once per robot, so A's socket is already gone by the time B tries
    // to deliver its own outgoing token back to A — `Skipped{unreachable_peer: 1}` is
    // therefore the expected, honest outcome here (Fix 1's whole point: a peer that
    // can't be reached within the backoff budget is reported as skipped, not silently
    // reported as delivered the way the pre-Fix-1 design did). What this test actually
    // needs from B is that it got far enough to *see* A's token at all — proven by
    // `Skipped`/`Handled` both requiring `token` to have been received and processed;
    // only `NoTokenArrived` would mean the retry/recovery this test targets failed.
    match outcome_b {
        TokenCycleOutcome::Handled { .. } | TokenCycleOutcome::Skipped { unreachable_peer: 1 } => {}
        TokenCycleOutcome::NoTokenArrived => panic!(
            "B never received the token despite A's retries — retry did not recover from the simulated drop"
        ),
        TokenCycleOutcome::Skipped { unreachable_peer } => {
            panic!("B unexpectedly skipped peer {unreachable_peer}, expected peer 1 (A) or none")
        }
        TokenCycleOutcome::Regenerated { .. } => {
            panic!("B should not have hit the watchdog in this short, healthy scenario")
        }
    }
}

fn cell_to_wire(c: Cell) -> (i32, i32) {
    (c.0 as i32, c.1 as i32)
}

/// Amendment A ("no token forking on skip"): the receiver's `Ack` gets lost, but the
/// receiver still processes and forwards the token normally (a real, narrower failure
/// than a fully dead peer — just the one small ack packet, not the peer, goes missing).
/// The sender must recover via the *other* proof of delivery Amendment A allows (a
/// `TokenMsg` with a higher `seq`, observed directly on the wire) rather than assuming
/// the hop failed and independently re-targeting the same `seq` at a different peer —
/// which is exactly what would fork the ring into two live tokens for the same seq.
///
/// Modeled with a hand-rolled stand-in for B rather than a real `TaskLayer::new`
/// instance specifically so the test can suppress *only* the `Ack` send while leaving
/// every other real-transport behavior (processing via the same `handle_token` core,
/// broadcasting the forward for real) intact — a real `TaskLayer` has no way to send an
/// ack any other way, so this is the one deliberate seam needed to reproduce "ack lost,
/// forward not" on purpose rather than leaving it to chance.
#[test]
fn ack_loss_recovered_via_higher_seq_forward_no_fork() {
    let _guard = SEQUENTIAL.lock().unwrap();
    let grid = test_grid();
    let free = grid.free_cells();
    let pos_a = free[0];
    let pos_b = free[1];
    let task = Task {
        task_id: 1,
        pickup: free[10],
        dropoff: free[20],
    };

    let mut robot_a =
        TaskLayer::new(1, &grid, vec![task], vec![1, 2, 3], pos_a).expect("bind A");
    let bootstrapper = Comms::new(99, (0, 0)).expect("bind bootstrapper");

    let handle_a = thread::spawn(move || robot_a.run_one_cycle(pos_a, Duration::from_secs(2)));

    let grid_for_b = grid.clone();
    let handle_b = thread::spawn(move || {
        let b_comms =
            Comms::with_range(2, cell_to_wire(pos_b), TOKEN_RANGE_CELLS).expect("bind B");
        b_comms
            .set_read_timeout(Some(Duration::from_secs(2)))
            .expect("set B read timeout");
        let mut b_layer = TaskLayer::new_pure(2, &grid_for_b, vec![task], vec![1, 2, 3]);
        loop {
            if let Ok(Some(received)) = b_comms.recv_filtered() {
                if let Payload::Token(t) = received.payload {
                    if t.holder_id == 2 {
                        // The simulated loss: no `send_ack` call at all, unlike real
                        // `run_one_cycle`. Everything else proceeds exactly as a real
                        // receiver would.
                        let outgoing = b_layer.handle_token(&t, pos_b);
                        b_comms.send_token(outgoing.clone()).expect("B forwards for real");
                        return outgoing;
                    }
                }
            }
        }
    });

    // A neutral observer with no ring role, watching everything either side puts on the
    // wire, to directly count how many distinct tokens actually circulated.
    let observer =
        Comms::with_range(50, (0, 0), TOKEN_RANGE_CELLS).expect("bind observer");
    observer
        .set_read_timeout(Some(Duration::from_millis(800)))
        .expect("set observer read timeout");

    bootstrapper
        .send_token(TokenMsg {
            seq: 0,
            holder_id: 1,
            claimed_tasks: vec![],
            epoch: 0,
            creator: 0,
        })
        .expect("bootstrapper sends initial token to A");

    let (outcome_a, _incidentals_a) = handle_a.join().expect("robot A thread panicked");
    let b_outgoing = handle_b.join().expect("robot B thread panicked");

    // The core claim: A must recover purely from observing B's real forward on the
    // wire, not by concluding the hop failed and re-targeting the same seq elsewhere —
    // that second path is exactly what would fork the ring.
    match outcome_a {
        TokenCycleOutcome::Handled { .. } => {}
        other => panic!(
            "A should have recovered via the higher-seq forward alone (no ack ever sent), \
             got {other:?} instead — this is the fork path Amendment A exists to prevent"
        ),
    }

    // No task claimed by two robots: exactly one (task_id, robot_id) entry for task 1,
    // not two (which double-claiming would produce), and not zero (which would mean the
    // claim was lost instead of forked — also wrong, just a different bug).
    let claims_of_task_1: Vec<_> =
        b_outgoing.claimed_tasks.iter().filter(|c| c.task_id == 1).collect();
    assert_eq!(
        claims_of_task_1.len(),
        1,
        "task 1 must be claimed by exactly one robot, got {:?}",
        b_outgoing.claimed_tasks
    );

    // Exactly one live token: drain whatever the observer saw and assert there is
    // exactly one distinct seq value for any TokenMsg addressed past B (holder_id == 3)
    // — if A had forked, a second, independently-numbered or independently-addressed
    // token would show up here too.
    let mut forwarded_to_c: HashSet<u64> = HashSet::new();
    while let Ok(Some(received)) = observer.recv_filtered() {
        if let Payload::Token(t) = received.payload {
            if t.holder_id == 3 {
                forwarded_to_c.insert(t.seq);
            }
        }
    }
    assert_eq!(
        forwarded_to_c,
        HashSet::from([b_outgoing.seq]),
        "exactly one live token should have reached the point of being forwarded to C"
    );
}

/// Amendment A's dedupe rule: a re-delivery of the exact seq this robot already handled
/// (e.g. the sender's own retry landing a second time before its first ack arrived) must
/// be re-acked, not reprocessed — no second claim/release decision, and no second
/// forwarded `TokenMsg` put on the wire.
#[test]
fn duplicate_token_delivery_causes_no_reprocessing_and_no_extra_traffic() {
    let _guard = SEQUENTIAL.lock().unwrap();
    let grid = test_grid();
    let free = grid.free_cells();
    let pos_a = free[0];
    let task = Task {
        task_id: 1,
        pickup: free[10],
        dropoff: free[20],
    };

    let mut robot_a = TaskLayer::new(1, &grid, vec![task], vec![1, 2], pos_a).expect("bind A");
    let sender = Comms::new(99, (0, 0)).expect("bind sender");

    // A permanent, ack-only stand-in for peer 2, so A's own outgoing forward always
    // gets acked and A never needs to skip/backoff — isolating just the behavior this
    // test targets: how A handles a *duplicate inbound* delivery, not peer reachability.
    let stop = Arc::new(AtomicBool::new(false));
    let stop_for_thread = Arc::clone(&stop);
    let acker = thread::spawn(move || {
        let b_comms =
            Comms::with_range(2, (0, 0), TOKEN_RANGE_CELLS).expect("bind ack-only B stand-in");
        b_comms
            .set_read_timeout(Some(Duration::from_millis(50)))
            .expect("set ack-only B read timeout");
        while !stop_for_thread.load(Ordering::Acquire) {
            if let Ok(Some(received)) = b_comms.recv_filtered() {
                if let Payload::Token(t) = received.payload {
                    if t.holder_id == 2 {
                        let _ = b_comms.send_ack(Ack {
                            seq: t.seq,
                            from: 2,
                            to: received.sender_id,
                        });
                    }
                }
            }
        }
    });

    // First delivery: a genuinely new token — A claims the task and forwards.
    sender
        .send_token(TokenMsg { seq: 0, holder_id: 1, claimed_tasks: vec![], epoch: 0, creator: 0 })
        .expect("send first delivery");
    let (outcome1, _inc1) = robot_a.run_one_cycle(pos_a, Duration::from_secs(1));
    let claimed1 = match outcome1 {
        TokenCycleOutcome::Handled { claimed } => claimed,
        other => panic!("expected A to handle the first delivery, got {other:?}"),
    };
    assert_eq!(claimed1, Some(task), "A should have claimed the task on first delivery");
    let state_after_first = robot_a.state();

    // A neutral observer, listening only from here on, so it never sees the first
    // delivery's own (expected) ack/forward traffic — only whatever the duplicate below
    // produces.
    let observer =
        Comms::with_range(51, (0, 0), TOKEN_RANGE_CELLS).expect("bind observer");
    observer
        .set_read_timeout(Some(Duration::from_millis(300)))
        .expect("set observer read timeout");

    // Re-deliver the exact same seq=0 token — simulating a duplicate/redelivered
    // packet, e.g. the original sender's own retry landing a second time.
    sender
        .send_token(TokenMsg { seq: 0, holder_id: 1, claimed_tasks: vec![], epoch: 0, creator: 0 })
        .expect("send duplicate delivery");

    // A duplicate must not register as a new hop: this call should simply time out,
    // having only re-acked in the background while it kept waiting for something new.
    let (outcome2, _inc2) = robot_a.run_one_cycle(pos_a, Duration::from_millis(300));
    assert_eq!(
        outcome2,
        TokenCycleOutcome::NoTokenArrived,
        "a duplicate of the already-handled seq must not trigger a second hop"
    );
    assert_eq!(
        robot_a.state(),
        state_after_first,
        "duplicate delivery must not change task state a second time"
    );

    stop.store(true, Ordering::Release);
    acker.join().expect("ack-only stand-in thread panicked");

    // No extra traffic: exactly one re-ack for the duplicate, and no second forwarded
    // TokenMsg (seq > 0) — reprocessing would have produced another one.
    let mut acks_for_seq_0 = 0;
    let mut forwarded_tokens = 0;
    while let Ok(Some(received)) = observer.recv_filtered() {
        match received.payload {
            Payload::Ack(ack) if ack.seq == 0 => acks_for_seq_0 += 1,
            Payload::Token(t) if t.seq > 0 => forwarded_tokens += 1,
            _ => {}
        }
    }
    assert_eq!(acks_for_seq_0, 1, "duplicate must be re-acked exactly once");
    assert_eq!(forwarded_tokens, 0, "duplicate must not trigger a second forward");
}

/// Step 1B: the *unrecoverable* fork path, deliberately not covered by
/// `ack_loss_recovered_via_higher_seq_forward_no_fork` above — that test only exercises
/// the case where A happens to overhear B's real forward and treats it as an implicit
/// ack. This constructs the harder case: A is made deaf to B specifically (never
/// receives B's `Ack` *or* B's forward), so A's backoff genuinely exhausts, marks B
/// unreachable, and re-sends the *same* `seq` — with B's-claim-free, stale claim set —
/// directly to C (`run_one_cycle`'s real skip behavior: same seq, only `holder_id`
/// changes). B's real forward (a *different* claim set, but the *same* next `seq`,
/// since both are computed as `received.seq + 1` off the same original token) is also
/// still alive and reaches C independently. Two delivery orders to C are tested,
/// deterministically (not raced) by controlling exactly when each message is sent and
/// blocking on C's real `run_one_cycle` between them.
///
/// Every message A/B "send" here is computed via the same real `handle_token` core
/// production code uses, and C is driven through its real `run_one_cycle` (so the real
/// ack/dedupe logic runs) — only the delivery *order* is deterministically controlled,
/// not the decision logic itself.
fn run_fork_scenario(a_resend_first: bool) -> (bool, bool, TokenCycleOutcome) {
    let _guard = SEQUENTIAL.lock().unwrap();
    let grid = test_grid();
    let free = grid.free_cells();
    let pos_a = free[0];
    let pos_b = free[1];
    let pos_c = free[2];
    let task1 = Task {
        task_id: 1,
        pickup: free[10],
        dropoff: free[20],
    };

    // A has no tasks of its own, so its first-step outgoing is *guaranteed* to carry an
    // empty claim set — "A's stale claims" below is unambiguous by construction, not
    // just probable given map distances.
    let mut a = TaskLayer::new(1, &grid, vec![], vec![1, 2, 3], pos_a).expect("bind A");
    let mut b = TaskLayer::new(2, &grid, vec![task1], vec![1, 2, 3], pos_b).expect("bind B");

    let token0 = TokenMsg { seq: 0, holder_id: 1, claimed_tasks: vec![], epoch: 0, creator: 0 };
    let t1 = a.handle_token(&token0, pos_a); // A's real first-step decision
    assert_eq!(t1.holder_id, 2);
    assert_eq!(t1.claimed_tasks, Vec::<ClaimEntry>::new(), "A has nothing of its own to claim");

    let t2 = b.handle_token(&t1, pos_b); // B's real decision on receiving T1
    assert_eq!(
        t2.claimed_tasks,
        vec![ClaimEntry { task_id: 1, robot_id: 2, picked_up: false }],
        "B should have claimed task1"
    );
    assert_eq!(b.state(), TaskState::ToPickup(task1));

    // A's real skip-on-exhaustion resend: identical seq/claims to T1, re-addressed to
    // C — exactly `run_one_cycle`'s own skip logic (`to_send.holder_id = target`), not
    // a hand-typed stand-in for it.
    let mut a_resend = t1.clone();
    a_resend.holder_id = 3;

    let mut c = TaskLayer::new(3, &grid, vec![task1], vec![1, 2, 3], pos_c).expect("bind C");
    let sender_a =
        Comms::with_range(1, cell_to_wire(pos_a), TOKEN_RANGE_CELLS).expect("bind A sender");
    let sender_b =
        Comms::with_range(2, cell_to_wire(pos_b), TOKEN_RANGE_CELLS).expect("bind B sender");

    // A lightweight ack-only stand-in for A (id 1), purely so C's own outgoing forwards
    // after each round get acked quickly instead of burning through backoff — this test
    // cares about C's *processing* outcome, not delivery speed to a robot this scenario
    // isn't otherwise exercising.
    let stop = Arc::new(AtomicBool::new(false));
    let stop_for_thread = Arc::clone(&stop);
    let acker = thread::spawn(move || {
        let a_stand_in =
            Comms::with_range(1, (0, 0), TOKEN_RANGE_CELLS).expect("bind A ack stand-in");
        a_stand_in
            .set_read_timeout(Some(Duration::from_millis(50)))
            .expect("set A ack stand-in read timeout");
        while !stop_for_thread.load(Ordering::Acquire) {
            if let Ok(Some(received)) = a_stand_in.recv_filtered() {
                if let Payload::Token(t) = received.payload {
                    if t.holder_id == 1 {
                        let _ = a_stand_in.send_ack(Ack {
                            seq: t.seq,
                            from: 1,
                            to: received.sender_id,
                        });
                    }
                }
            }
        }
    });

    // Round 1: whichever message is configured to arrive first — this call blocks
    // until C actually processes it.
    if a_resend_first {
        sender_a.send_token(a_resend.clone()).expect("send A's resend first");
    } else {
        sender_b.send_token(t2.clone()).expect("send B's forward first");
    }
    let (outcome1, _inc1) = c.run_one_cycle(pos_c, Duration::from_secs(2));
    assert!(
        matches!(outcome1, TokenCycleOutcome::Handled { .. }),
        "round 1 should be a genuinely new hop for C: {outcome1:?}"
    );

    // Round 2: the other message.
    if a_resend_first {
        sender_b.send_token(t2.clone()).expect("send B's forward second");
    } else {
        sender_a.send_token(a_resend.clone()).expect("send A's resend second");
    }
    let (outcome2, _inc2) = c.run_one_cycle(pos_c, Duration::from_millis(500));

    stop.store(true, Ordering::Release);
    acker.join().expect("A ack stand-in thread panicked");

    let b_holds_it = b.state() == TaskState::ToPickup(task1);
    let c_holds_it = c.state() == TaskState::ToPickup(task1);
    (b_holds_it, c_holds_it, outcome2)
}

/// Ordering (a): B's real forward reaches C first. C sees task1 already claimed (by B)
/// in that token's own claim set, so it never independently claims it; when A's stale
/// resend arrives second, C's watermark (`highest_seen_seq`) correctly recognizes it as
/// older than what it already processed and drops it silently. No fork.
#[test]
fn fork_scenario_b_forward_first_stays_safe() {
    let (b_holds_it, c_holds_it, _outcome2) = run_fork_scenario(false);
    assert!(b_holds_it, "B should hold task1 from its own real processing");
    assert!(
        !c_holds_it,
        "C should never have claimed task1: B's forward already showed it claimed"
    );
}

/// Ordering (b): A's stale resend reaches C first. Since C's watermark was still empty,
/// A's resend looks exactly like a genuinely new hop — C claims task1 for itself (A's
/// claim set doesn't show it taken yet). When B's real forward arrives second, its
/// `seq` is *not* higher than what C just set its watermark to (both A's resend and B's
/// forward compute `received.seq + 1` off the *same* original token, so they carry the
/// identical `seq` — highest-seq-wins has no tie-break here), and it *is* higher than
/// C's watermark from the resend it just(!) processed — wait: both messages carry the
/// *same* seq (`t1.seq + 1`), so C's second receipt is compared against its own
/// watermark, already sitting at exactly that seq. That makes it read as an exact
/// *duplicate* of what C just handled — except it superficially looks like a *forward*
/// from a *different* sender/claim-set, not a byte-identical resend, and this
/// implementation's dedupe check only compares `seq`, not content. Expected (per the
/// task's own prediction): this is exactly where the fork surfaces.
#[test]
fn fork_scenario_a_resend_first_may_double_claim() {
    let (b_holds_it, c_holds_it, outcome2) = run_fork_scenario(true);
    assert!(b_holds_it, "B should hold task1 from its own real processing");
    assert!(
        !c_holds_it,
        "double claim: both B and C locally believe they hold task1 — B via its own \
         real processing, C via {} A's stale resend as though it were a fresh, \
         independent hop from B. round-2 outcome was {outcome2:?}.",
        if matches!(outcome2, TokenCycleOutcome::Handled { .. }) {
            "reprocessing"
        } else {
            "not reprocessing (unexpected — investigate why)"
        }
    );
}

/// End-state check, added because the transient assertion above only proves the
/// reconciliation *resolved this one collision* — it says nothing about whether the
/// claim invariant (exactly one robot committed, no orphan) still holds after further
/// circulation, which is the actually load-bearing property. Reconstructs the same
/// ordering-(b) fork, deterministically, then continues circulating (`handle_token`
/// only, no further loss — the same pure-core continuation style Step 1B's own helper
/// and the property test both already use) for several more hops and checks the
/// invariant at that later point too, not just immediately after the first collision.
#[test]
fn fork_scenario_a_resend_first_end_state_after_circulation_is_consistent() {
    let grid = test_grid();
    let free = grid.free_cells();
    let pos_a = free[0];
    let pos_b = free[1];
    let pos_c = free[2];
    let task1 = Task { task_id: 1, pickup: free[10], dropoff: free[20] };

    let mut a = TaskLayer::new_pure(1, &grid, vec![], vec![1, 2, 3]);
    let mut b = TaskLayer::new_pure(2, &grid, vec![task1], vec![1, 2, 3]);
    let mut c = TaskLayer::new_pure(3, &grid, vec![task1], vec![1, 2, 3]);

    let token0 = TokenMsg { seq: 0, holder_id: 1, claimed_tasks: vec![], epoch: 0, creator: 0 };
    let t1 = a.handle_token(&token0, pos_a);
    let t2 = b.handle_token(&t1, pos_b); // B's real claim
    let mut a_resend = t1.clone();
    a_resend.holder_id = 3;

    // Ordering (b): A's stale resend reaches C first, then B's real forward — the
    // collision this test exists to check the aftermath of.
    let outgoing_r1 = c.handle_token(&a_resend, pos_c);
    let outgoing_r2 = c.handle_token(&t2, pos_c);
    assert!(
        c.state() != TaskState::ToPickup(task1),
        "sanity check: the transient collision should already be resolved by here, \
         same as fork_scenario_a_resend_first_may_double_claim proves"
    );

    // Continue circulating BOTH branches still alive after the collision (C's own
    // response to the resend, and its response to B's real forward) for several more
    // clean hops around the ring, checking the claim invariant after each one — not
    // just once, immediately after the first collision resolves.
    let positions = [pos_a, pos_b, pos_c];
    let mut queue: Vec<(TokenMsg, u32)> = vec![
        (outgoing_r1.clone(), outgoing_r1.holder_id),
        (outgoing_r2.clone(), outgoing_r2.holder_id),
    ];

    for _ in 0..12 {
        // 4 laps around a 3-robot ring
        if queue.is_empty() {
            break;
        }
        let (token, target) = queue.remove(0);
        let ti = (target - 1) as usize;
        let pos = positions[ti];
        let outgoing = match ti {
            0 => a.handle_token(&token, pos),
            1 => b.handle_token(&token, pos),
            _ => c.handle_token(&token, pos),
        };
        queue.push((outgoing.clone(), outgoing.holder_id));

        let committed: Vec<u32> = [(&a, 1u32), (&b, 2), (&c, 3)]
            .iter()
            .filter(|(layer, _)| layer.state() == TaskState::ToPickup(task1))
            .map(|&(_, id)| id)
            .collect();
        assert!(
            committed.len() <= 1,
            "end-state invariant violated mid-circulation: task1 committed by {committed:?}"
        );
    }

    let committed_at_end: Vec<u32> = [(&a, 1u32), (&b, 2), (&c, 3)]
        .iter()
        .filter(|(layer, _)| layer.state() == TaskState::ToPickup(task1))
        .map(|&(_, id)| id)
        .collect();
    assert_eq!(
        committed_at_end.len(),
        1,
        "exactly one robot must be committed to task1 at the end, got {committed_at_end:?}"
    );

    // No orphan: the final queued token's claim entry for task1 must name whoever is
    // actually committed.
    if let Some((last_token, _)) = queue.last() {
        for entry in &last_token.claimed_tasks {
            if entry.task_id == 1 {
                assert_eq!(
                    entry.robot_id, committed_at_end[0],
                    "orphan/mismatch: the token names {} as task1's claimant, but {:?} is who's \
                     actually committed",
                    entry.robot_id, committed_at_end
                );
            }
        }
    }
}

/// Test 2 (mirror case): same fork shape as `fork_scenario_a_resend_first_may_double_claim`
/// — one robot independently claims a task from a stale/empty-claims token, then later
/// sees a token naming a *different* robot as that task's claimant — but with the id
/// ordering deliberately flipped so the stale-claiming robot has the *lower* id. Per the
/// reconciliation rule ("ties go to the lower robot_id"), the lower-id side must win
/// this tie and the higher-id side must release — this is the only way to confirm the
/// tie-break is genuinely id-ordered rather than "whichever side happens to process
/// second loses" (the original fork scenario's id assignment can't distinguish the two,
/// since its stale-claiming robot (C=3) also happens to have the higher id).
/// Pure logic only (`handle_token`, no sockets) — the reconciliation rule itself is
/// transport-agnostic, same as Amendment A's dedupe.
#[test]
fn claim_conflict_tie_break_favors_lower_id_regardless_of_who_claimed_first() {
    let grid = test_grid();
    let free = grid.free_cells();
    let task1 = Task { task_id: 1, pickup: free[10], dropoff: free[20] };

    let mut robot_low = TaskLayer::new_pure(1, &grid, vec![task1], vec![1, 5]);
    let _ = robot_low.handle_token(
        &TokenMsg { seq: 1, holder_id: 1, claimed_tasks: vec![], epoch: 0, creator: 0 },
        free[0],
    );
    assert_eq!(robot_low.state(), TaskState::ToPickup(task1));

    let mut robot_high = TaskLayer::new_pure(5, &grid, vec![task1], vec![1, 5]);
    let _ = robot_high.handle_token(
        &TokenMsg { seq: 1, holder_id: 5, claimed_tasks: vec![], epoch: 0, creator: 0 },
        free[1],
    );
    assert_eq!(robot_high.state(), TaskState::ToPickup(task1));

    // Robot 1 (lower id) sees a token naming robot 5 as task1's claimant — both sides
    // still merely `ToPickup`, so `picked_up` is `false` on both, a genuine tie.
    let outgoing = robot_low.handle_token(
        &TokenMsg {
            seq: 2,
            holder_id: 1,
            claimed_tasks: vec![ClaimEntry { task_id: 1, robot_id: 5, picked_up: false }],
            epoch: 0,
            creator: 0,
        },
        free[0],
    );
    assert_eq!(
        robot_low.state(),
        TaskState::ToPickup(task1),
        "lower id (1) should win the tie and keep task1"
    );
    assert_eq!(
        outgoing.claimed_tasks,
        vec![ClaimEntry { task_id: 1, robot_id: 1, picked_up: false }],
        "the outgoing token must be rewritten to show the tie's actual winner"
    );
    assert_eq!(robot_low.claim_conflicts(), 1);

    // Robot 5 (higher id) must release once it sees the token confirm robot 1's win.
    let _ = robot_high.handle_token(
        &TokenMsg {
            seq: 3,
            holder_id: 5,
            claimed_tasks: vec![ClaimEntry { task_id: 1, robot_id: 1, picked_up: false }],
            epoch: 0,
            creator: 0,
        },
        free[1],
    );
    assert_eq!(
        robot_high.state(),
        TaskState::Idle,
        "higher id (5) should have released task1 after losing the tie"
    );
    assert_eq!(robot_high.claim_conflicts(), 1);
}

/// Test 3 (progress case): a robot that has made real physical progress (`ToDropoff`)
/// must keep the task even against a rival with a lower `robot_id` — progress beats the
/// id tie-break, not the other way around. Pure logic only, same reasoning as above.
#[test]
fn claim_conflict_progress_beats_lower_id() {
    let grid = test_grid();
    let free = grid.free_cells();
    let task1 = Task { task_id: 1, pickup: free[10], dropoff: free[20] };

    // Robot 9 (deliberately the *higher* id) claims task1 and makes real progress to
    // ToDropoff.
    let mut robot_high = TaskLayer::new_pure(9, &grid, vec![task1], vec![1, 9]);
    let _ = robot_high.handle_token(
        &TokenMsg { seq: 1, holder_id: 9, claimed_tasks: vec![], epoch: 0, creator: 0 },
        free[0],
    );
    assert_eq!(robot_high.state(), TaskState::ToPickup(task1));
    robot_high.on_position_update(task1.pickup);
    assert_eq!(
        robot_high.state(),
        TaskState::ToDropoff(task1),
        "must have genuinely arrived at pickup, not just claimed"
    );

    // Robot 1 (lower id) independently believes it holds the same task, at the mere
    // ToPickup stage — the stale-resend shape again.
    let mut robot_low = TaskLayer::new_pure(1, &grid, vec![task1], vec![1, 9]);
    let _ = robot_low.handle_token(
        &TokenMsg { seq: 1, holder_id: 1, claimed_tasks: vec![], epoch: 0, creator: 0 },
        free[1],
    );
    assert_eq!(robot_low.state(), TaskState::ToPickup(task1));

    // Robot 9 sees a token naming robot 1 as claimant (still merely `ToPickup`, so
    // `picked_up: false`) — real ToDropoff progress must win despite the higher id.
    let outgoing = robot_high.handle_token(
        &TokenMsg {
            seq: 2,
            holder_id: 9,
            claimed_tasks: vec![ClaimEntry { task_id: 1, robot_id: 1, picked_up: false }],
            epoch: 0,
            creator: 0,
        },
        task1.pickup,
    );
    assert_eq!(
        robot_high.state(),
        TaskState::ToDropoff(task1),
        "real progress (ToDropoff) must beat a rival's mere claim, even at a higher id"
    );
    assert_eq!(
        outgoing.claimed_tasks,
        vec![ClaimEntry { task_id: 1, robot_id: 9, picked_up: true }],
        "the rewritten entry must carry the winner's real phase too"
    );
    assert_eq!(robot_high.claim_conflicts(), 1);

    // Robot 1 must release once it sees the token confirm robot 9's win — crucially,
    // that confirmation must carry `picked_up: true` (what robot 9's own token above
    // just produced), or robot 1 (still merely `ToPickup` itself) would once again see
    // what looks like a tie and wrongly re-assert its own lower-id "win".
    let _ = robot_low.handle_token(
        &TokenMsg {
            seq: 3,
            holder_id: 1,
            claimed_tasks: vec![ClaimEntry { task_id: 1, robot_id: 9, picked_up: true }],
            epoch: 0,
            creator: 0,
        },
        free[1],
    );
    assert_eq!(robot_low.state(), TaskState::Idle);
    assert_eq!(robot_low.claim_conflicts(), 1);
}

/// Symmetric case: two robots, neither has picked up, each independently believes it
/// holds the same task, and each sees a token naming the *other* as claimant — a
/// genuinely simultaneous conflict, not one side resolving before the other's even
/// constructed. Both sides applying the identical rule from their own vantage point
/// must agree on exactly one winner: not both releasing (which would orphan the task —
/// nobody committed, even though it's still listed as claimed in the pool-exclusion
/// sense) and not both keeping it (the original fork).
#[test]
fn claim_conflict_symmetric_case_exactly_one_winner_not_orphaned() {
    let grid = test_grid();
    let free = grid.free_cells();
    let task1 = Task { task_id: 1, pickup: free[10], dropoff: free[20] };

    let mut robot_a = TaskLayer::new_pure(3, &grid, vec![task1], vec![3, 7]);
    let _ = robot_a.handle_token(
        &TokenMsg { seq: 1, holder_id: 3, claimed_tasks: vec![], epoch: 0, creator: 0 },
        free[0],
    );
    assert_eq!(robot_a.state(), TaskState::ToPickup(task1));

    let mut robot_b = TaskLayer::new_pure(7, &grid, vec![task1], vec![3, 7]);
    let _ = robot_b.handle_token(
        &TokenMsg { seq: 1, holder_id: 7, claimed_tasks: vec![], epoch: 0, creator: 0 },
        free[1],
    );
    assert_eq!(robot_b.state(), TaskState::ToPickup(task1));

    let out_a = robot_a.handle_token(
        &TokenMsg {
            seq: 2,
            holder_id: 3,
            claimed_tasks: vec![ClaimEntry { task_id: 1, robot_id: 7, picked_up: false }],
            epoch: 0,
            creator: 0,
        },
        free[0],
    );
    let out_b = robot_b.handle_token(
        &TokenMsg {
            seq: 2,
            holder_id: 7,
            claimed_tasks: vec![ClaimEntry { task_id: 1, robot_id: 3, picked_up: false }],
            epoch: 0,
            creator: 0,
        },
        free[1],
    );

    let a_holds = robot_a.state() == TaskState::ToPickup(task1);
    let b_holds = robot_b.state() == TaskState::ToPickup(task1);
    assert_ne!(
        a_holds, b_holds,
        "exactly one of the two must end up committed — not both (fork), not neither (orphaned)"
    );
    assert!(a_holds, "robot 3 (lower id) should have won the tie");
    let winner_entry = ClaimEntry { task_id: 1, robot_id: 3, picked_up: false };
    assert_eq!(out_a.claimed_tasks, vec![winner_entry]);
    assert_eq!(
        out_b.claimed_tasks,
        vec![winner_entry],
        "the loser's own outgoing token must also reflect the true winner, not drop the claim"
    );
}

/// Idempotence: running the *exact same* input token through `handle_token` twice must
/// not flip-flop the wire entry or the robot's own state — a real scenario a duplicate
/// redelivery could trigger (Amendment A's dedupe in `run_one_cycle` already prevents
/// this at the transport layer via `highest_seen_seq`, but `handle_token` itself has no
/// seq-awareness at all, so this checks the reconciliation rule's own idempotence
/// directly, independent of that transport-layer guard).
#[test]
fn claim_conflict_reconciliation_is_idempotent_on_repeated_input() {
    let grid = test_grid();
    let free = grid.free_cells();
    let task1 = Task { task_id: 1, pickup: free[10], dropoff: free[20] };

    // Winner-side idempotence: robot 2 (lower id) independently holds task1.
    let mut winner = TaskLayer::new_pure(2, &grid, vec![task1], vec![2, 8]);
    let _ = winner.handle_token(&TokenMsg { seq: 1, holder_id: 2, claimed_tasks: vec![], epoch: 0, creator: 0 }, free[0]);
    assert_eq!(winner.state(), TaskState::ToPickup(task1));

    let conflicting = TokenMsg {
        seq: 2,
        holder_id: 2,
        claimed_tasks: vec![ClaimEntry { task_id: 1, robot_id: 8, picked_up: false }],
        epoch: 0,
        creator: 0,
    };
    let out1 = winner.handle_token(&conflicting, free[0]);
    let (state1, conflicts1) = (winner.state(), winner.claim_conflicts());
    let out2 = winner.handle_token(&conflicting, free[0]); // exact same input again
    let (state2, conflicts2) = (winner.state(), winner.claim_conflicts());

    assert_eq!(state1, state2, "state must not flip-flop across repeated identical input");
    assert_eq!(out1.claimed_tasks, out2.claimed_tasks, "the wire entry must not flip-flop either");
    assert_eq!(state1, TaskState::ToPickup(task1), "robot 2 (lower id) should win both times");
    assert_eq!(
        conflicts2, conflicts1 + 1,
        "each call independently detects and resolves the same conflict once"
    );

    // Loser-side idempotence: a different mechanism entirely — once released, the
    // robot has no `current_task()` left, so reconciliation simply doesn't re-fire.
    let mut loser = TaskLayer::new_pure(8, &grid, vec![task1], vec![2, 8]);
    let _ = loser.handle_token(&TokenMsg { seq: 1, holder_id: 8, claimed_tasks: vec![], epoch: 0, creator: 0 }, free[1]);
    assert_eq!(loser.state(), TaskState::ToPickup(task1));

    let confirming = TokenMsg {
        seq: 3,
        holder_id: 8,
        claimed_tasks: vec![ClaimEntry { task_id: 1, robot_id: 2, picked_up: false }],
        epoch: 0,
        creator: 0,
    };
    let _ = loser.handle_token(&confirming, free[1]);
    assert_eq!(loser.state(), TaskState::Idle, "robot 8 (higher id) should release");
    let _ = loser.handle_token(&confirming, free[1]); // exact same input again
    assert_eq!(
        loser.state(),
        TaskState::Idle,
        "must not flip back to ToPickup on a repeat of the same confirming token"
    );
}

/// Property test (item 6): 6 robots, 200 seeded trials, randomly injected ack/forward
/// drops (Step 1B's exact fork-injection mechanism — a lost ack forces a stale resend
/// of the *pre-hop* token to the next-reachable peer, while the real recipient's
/// forward keeps circulating independently too) plus random reordering standing in for
/// delay, during a bounded "lossy" phase — followed by a "clean" phase with no further
/// injected loss, giving every still-live branch room to complete at least one full
/// circulation. Checks, after that clean phase:
/// 1. No task is locally committed (`ToPickup`/`ToDropoff`) by more than one robot.
/// 2. No task is listed as claimed in the final circulating token by a robot whose own
///    local state doesn't actually reflect holding it — a "phantom" claim that would
///    permanently block the task from being reclaimed by anyone while nobody is
///    actually working it (the "committed by zero robots while still unclaimed[-looking]
///    in the pool" failure mode).
///
/// Pure simulation, `handle_token` only — no real sockets, so this can run 200 trials in
/// well under a second, same reasoning as `run_experiment.rs`/`degradation_sweep.rs`'s
/// own in-process harness (Decision 15).
#[test]
fn claim_conflict_property_no_double_commitment_after_clean_circulation() {
    use rand::rngs::StdRng;
    use rand::{RngExt, SeedableRng};

    const NUM_ROBOTS: u32 = 6;
    const NUM_TRIALS: u64 = 200;
    const LOSSY_HOPS: usize = 40;
    /// Clean laps guaranteed *per live branch* in phase 2, not a fixed total — see that
    /// phase's own comment for why this replaced a fixed hop count.
    const CLEAN_LAPS: usize = 3;
    const P_LOSS: f64 = 0.35;

    let grid = test_grid();
    let free = grid.free_cells();
    let peer_ids: Vec<u32> = (1..=NUM_ROBOTS).collect();
    let tasks: Vec<Task> = (0..3u32)
        .map(|i| Task {
            task_id: i + 1,
            pickup: free[10 + i as usize * 5],
            dropoff: free[40 + i as usize * 5],
        })
        .collect();
    let positions: Vec<Cell> = (0..NUM_ROBOTS as usize).map(|i| free[i]).collect();

    let mut double_commitment_failures = 0u32;
    let mut orphan_failures = 0u32;
    let mut total_claim_conflicts = 0u64;
    let mut total_token_skips = 0u64;

    for trial in 0..NUM_TRIALS {
        let mut rng = StdRng::seed_from_u64(trial);
        let mut layers: Vec<TaskLayer> = peer_ids
            .iter()
            .map(|&id| TaskLayer::new_pure(id, &grid, tasks.clone(), peer_ids.clone()))
            .collect();

        // In-flight queue: (token, target_robot_id). Starts with the bootstrap hop.
        let mut queue: Vec<(TokenMsg, u32)> =
            vec![(TokenMsg { seq: 0, holder_id: 1, claimed_tasks: vec![], epoch: 0, creator: 0 }, 1)];
        let mut last_processed: Option<TokenMsg> = None;

        // Phase 1 (lossy): exactly LOSSY_HOPS hops, each with probability P_LOSS of
        // spawning a fork branch (Step 1B's mechanism). The queue only ever grows here
        // (each hop nets +0 or +1 entries), so its size at the end of this phase is the
        // real number of independently-live branches phase 2 has to converge.
        for _ in 0..LOSSY_HOPS {
            if queue.is_empty() {
                break;
            }
            // Random delay/reorder: pick a random queued entry rather than always the
            // front, standing in for delay-induced reordering on a real bus.
            let idx = rng.random_range(0..queue.len());
            let (token, target) = queue.remove(idx);
            let target_idx = (target - 1) as usize;
            let outgoing = layers[target_idx].handle_token(&token, positions[target_idx]);

            if rng.random_bool(P_LOSS) {
                // The real forward still happens physically — only the ack is lost
                // (Step 1B's exact mechanism) — so it keeps circulating...
                queue.push((outgoing.clone(), outgoing.holder_id));
                // ...while the sender independently falls back to a stale resend of
                // the *pre-hop* token, re-addressed to the next peer after the one it
                // gave up on (mirroring `run_one_cycle`'s real skip: same seq/claims,
                // only the addressee changes).
                let skip_to = peer_ids
                    .iter()
                    .cycle()
                    .skip_while(|&&id| id != outgoing.holder_id)
                    .nth(1)
                    .copied()
                    .unwrap_or(target);
                let mut stale_resend = token.clone();
                stale_resend.holder_id = skip_to;
                queue.push((stale_resend, skip_to));
            } else {
                queue.push((outgoing.clone(), outgoing.holder_id));
            }
            last_processed = Some(outgoing);
        }

        // Phase 2 (clean): no further loss injection. Budget scales with how many
        // branches phase 1 actually left live, not a fixed constant — "one full
        // loss-free circulation" means every live branch needs a real chance at a full
        // NUM_ROBOTS-hop lap, and a fixed small budget spread across a variable,
        // possibly-larger set of branches under-provisions exactly that (confirmed
        // empirically: a fixed 60-hop budget produced real convergence failures here
        // that a scaled budget does not — this constant, not the reconciliation logic
        // itself, was the gap).
        let branches_after_lossy_phase = queue.len().max(1);
        let clean_budget = branches_after_lossy_phase * NUM_ROBOTS as usize * CLEAN_LAPS;
        for _ in 0..clean_budget {
            if queue.is_empty() {
                break;
            }
            let idx = rng.random_range(0..queue.len());
            let (token, target) = queue.remove(idx);
            let target_idx = (target - 1) as usize;
            let outgoing = layers[target_idx].handle_token(&token, positions[target_idx]);
            last_processed = Some(outgoing.clone());
            queue.push((outgoing.clone(), outgoing.holder_id));
        }

        for l in &layers {
            total_claim_conflicts += u64::from(l.claim_conflicts());
            total_token_skips += u64::from(l.skip_count()); // always 0: no real transport in this pure sim
        }

        // Invariant 1: no double commitment.
        for t in &tasks {
            let holders = layers.iter().filter(|l| {
                matches!(l.state(), TaskState::ToPickup(x) | TaskState::ToDropoff(x) if x.task_id == t.task_id)
            }).count();
            if holders > 1 {
                double_commitment_failures += 1;
            }
        }

        // Invariant 2: no phantom/orphaned claim in the last token actually processed.
        if let Some(last_token) = last_processed {
            for entry in &last_token.claimed_tasks {
                let named = &layers[(entry.robot_id - 1) as usize];
                let named_holds = matches!(
                    named.state(),
                    TaskState::ToPickup(x) | TaskState::ToDropoff(x) if x.task_id == entry.task_id
                );
                if !named_holds {
                    orphan_failures += 1;
                }
            }
        }
    }

    println!(
        "[claim_conflict property test] trials={NUM_TRIALS} claim_conflicts_total={total_claim_conflicts} \
         token_skips_total={total_token_skips} double_commitment_failures={double_commitment_failures} \
         orphan_failures={orphan_failures}"
    );
    assert_eq!(
        double_commitment_failures, 0,
        "at least one trial ended with a task locally committed by more than one robot"
    );
    assert_eq!(
        orphan_failures, 0,
        "at least one trial ended with a claimed-but-unheld (orphaned) task"
    );
}

/// TEMPORARY DIAGNOSTIC (not a permanent test — answering a direct question about
/// post-fork token survival, to be removed after reporting). Reproduces Step 1B
/// ordering (b) exactly (same real `handle_token` calls A/B/C's genuine decisions come
/// from), tags each of the two branches created there with a distinct lineage id, then
/// runs 5 more full ring circulations (5 * 3 = 15 hops, 3-robot ring, no further loss
/// injection — "clean", matching the request), processing the queue strictly FIFO (not
/// randomly reordered, so "packets per circulation" has an unambiguous meaning). Reports
/// how many distinct lineages are still being forwarded at the end, and packet-per-hop
/// traffic before vs. after the fork.
#[test]
fn diag_post_fork_token_survival() {
    let grid = test_grid();
    let free = grid.free_cells();
    let pos_a = free[0];
    let pos_b = free[1];
    let pos_c = free[2];
    let task1 = Task { task_id: 1, pickup: free[10], dropoff: free[20] };

    let mut a = TaskLayer::new_pure(1, &grid, vec![], vec![1, 2, 3]);
    let mut b = TaskLayer::new_pure(2, &grid, vec![task1], vec![1, 2, 3]);
    let mut c = TaskLayer::new_pure(3, &grid, vec![task1], vec![1, 2, 3]);

    const NUM_ROBOTS: usize = 3;
    const CIRCULATIONS: usize = 5;
    let hops_per_lineage_target = NUM_ROBOTS * CIRCULATIONS; // 15: what 5 clean laps costs ONE lineage

    // Baseline, before the fork: a single lineage circulating cleanly for 5 laps costs
    // exactly `hops_per_lineage_target` packets — 1 packet per hop, by construction (no
    // branching yet).
    let token0 = TokenMsg { seq: 0, holder_id: 1, claimed_tasks: vec![], epoch: 0, creator: 0 };
    let t1 = a.handle_token(&token0, pos_a); // 1 packet, the pre-fork baseline hop
    let packets_before_fork_baseline_for_5_laps = hops_per_lineage_target as u32;

    // The fork event itself (Step 1B ordering (b)): B's real forward, and A's stale
    // resend of T1, both now independently alive, each tagged with a lineage id.
    let t2 = b.handle_token(&t1, pos_b); // B's real forward
    let mut a_resend = t1.clone();
    a_resend.holder_id = 3; // A's stale resend, re-addressed (not a new handle_token call)

    let mut queue: Vec<(TokenMsg, u32, u32)> = vec![
        (a_resend, 3, 1), // lineage 1: A's stale resend, heading to C
        (t2, 3, 2),        // lineage 2: B's real forward, heading to C
    ];

    let positions = [pos_a, pos_b, pos_c];
    let layers: [&mut TaskLayer; 3] = [&mut a, &mut b, &mut c];

    // Run, strict FIFO (no reordering, so this is reproducible and each pop is exactly
    // one real packet), until EVERY still-live lineage has individually accumulated
    // `hops_per_lineage_target` hops of its own — i.e., every lineage that's still
    // alive actually gets its full 5 clean laps, not just the queue as a whole.
    let mut hops_done_per_lineage: std::collections::HashMap<u32, usize> = std::collections::HashMap::new();
    let mut total_packets_after_fork = 0u32;
    let safety_cap = hops_per_lineage_target * 20; // generous; should never be hit
    for _ in 0..safety_cap {
        let all_done = queue
            .iter()
            .all(|&(_, _, lineage)| *hops_done_per_lineage.get(&lineage).unwrap_or(&0) >= hops_per_lineage_target);
        if queue.is_empty() || all_done {
            break;
        }
        let (token, target, lineage) = queue.remove(0);
        let ti = (target - 1) as usize;
        let pos = positions[ti];
        let outgoing = match ti {
            0 => layers[0].handle_token(&token, pos),
            1 => layers[1].handle_token(&token, pos),
            _ => layers[2].handle_token(&token, pos),
        };
        total_packets_after_fork += 1;
        *hops_done_per_lineage.entry(lineage).or_insert(0) += 1;
        queue.push((outgoing.clone(), outgoing.holder_id, lineage));
    }

    let live_lineages: std::collections::HashSet<u32> =
        queue.iter().map(|&(_, _, lineage)| lineage).collect();

    println!(
        "[diag] baseline (pre-fork, 1 lineage): {packets_before_fork_baseline_for_5_laps} packets \
         for {CIRCULATIONS} clean circulations"
    );
    println!(
        "[diag] AFTER the fork: {total_packets_after_fork} packets needed for every still-live \
         lineage to each individually get its own {CIRCULATIONS} clean circulations \
         -> {:.2}x the pre-fork baseline traffic",
        total_packets_after_fork as f64 / packets_before_fork_baseline_for_5_laps as f64
    );
    println!(
        "[diag] distinct live lineages still being forwarded at the end: {} -> {:?}",
        live_lineages.len(),
        queue.iter().map(|&(ref t, target, lineage)| (lineage, t.seq, target)).collect::<Vec<_>>()
    );
}

/// Real-transport confirmation (item 4): the property test's forks are synthesized —
/// `handle_token` calls with a hand-constructed stale resend, never touching
/// `deliver_with_backoff`/`run_one_cycle`'s real backoff/skip logic at all. This variant
/// reproduces the same fork shape through the *real* skip path: A is given a genuinely
/// deaf peer (B deliberately doesn't start polling until after A's full ~300ms backoff
/// budget to it has elapsed — the same "arrive late" technique
/// `dropped_forward_hop_is_retried_until_the_next_peer_picks_it_up` already uses, just
/// tuned past the budget instead of within it, so this time the skip genuinely fires),
/// so A's own real `run_one_cycle` marks B unreachable and resends for real. B's
/// eventual real `run_one_cycle` call still picks up A's earlier backlogged send (queued
/// in its already-bound kernel socket buffer) and forwards for real too — two real,
/// independently-arriving branches at C, not two hand-typed `TokenMsg`s.
#[test]
fn real_transport_fork_reproduces_via_actual_skip_path_50_trials() {
    const TRIALS: u32 = 50;
    let mut double_commitment_failures = 0u32;
    let mut orphan_failures = 0u32;
    let mut real_skips_observed = 0u32;

    for trial in 0..TRIALS {
        let _guard = SEQUENTIAL.lock().unwrap();
        let grid = test_grid();
        let free = grid.free_cells();
        let pos_a = free[0];
        let pos_b = free[1];
        let pos_c = free[2];
        let task1 = Task { task_id: 1, pickup: free[10], dropoff: free[20] };

        let mut a = TaskLayer::new(1, &grid, vec![], vec![1, 2, 3], pos_a).expect("bind A");
        let mut b = TaskLayer::new(2, &grid, vec![task1], vec![1, 2, 3], pos_b).expect("bind B");
        let mut c = TaskLayer::new(3, &grid, vec![task1], vec![1, 2, 3], pos_c).expect("bind C");
        let bootstrapper = Comms::new(99, (0, 0)).expect("bind bootstrapper");

        let (c_first_outcome, a_outcome) = thread::scope(|scope| {
            // A and B each stay real, continuing participants for a couple more rounds
            // after their first hop — not fake ack-only stand-ins — so that C's own
            // downstream forward (after resolving the fork) has someone real left to
            // reach. Without this, both A and B's one-shot threads have already exited
            // by the time C tries to forward, and C's own delivery spuriously exhausts
            // backoff against nobody — a test-harness gap, not evidence about the
            // reconciliation logic under test (confirmed by tracing it: C's first
            // result was `Skipped` only because *both* of C's would-be downstream
            // targets had already disconnected, not because reconciliation misbehaved).
            let h_a = scope.spawn(|| {
                let (first, _) = a.run_one_cycle(pos_a, Duration::from_secs(2));
                for _ in 0..10 {
                    let (o, _) = a.run_one_cycle(pos_a, Duration::from_millis(500));
                    if matches!(o, TokenCycleOutcome::NoTokenArrived) {
                        break;
                    }
                }
                first
            });
            let h_b = scope.spawn(|| {
                // Deliberately past A's full ~300ms backoff budget to B (20+40+80+160),
                // so A's real skip genuinely fires instead of succeeding within budget.
                thread::sleep(Duration::from_millis(350));
                for _ in 0..10 {
                    let (o, _) = b.run_one_cycle(pos_b, Duration::from_millis(500));
                    if matches!(o, TokenCycleOutcome::NoTokenArrived) {
                        break;
                    }
                }
            });
            let h_c = scope.spawn(|| {
                let (first, _) = c.run_one_cycle(pos_c, Duration::from_secs(2));
                for _ in 0..10 {
                    let (o, _) = c.run_one_cycle(pos_c, Duration::from_millis(500));
                    if matches!(o, TokenCycleOutcome::NoTokenArrived) {
                        break;
                    }
                }
                first
            });

            bootstrapper
                .send_token(TokenMsg { seq: 0, holder_id: 1, claimed_tasks: vec![], epoch: 0, creator: 0 })
                .expect("bootstrapper sends initial token");

            let a_outcome = h_a.join().expect("A thread panicked");
            h_b.join().expect("B thread panicked");
            let c_first_outcome = h_c.join().expect("C thread panicked");
            (c_first_outcome, a_outcome)
        });
        let b_state = b.state();

        real_skips_observed += u32::from(matches!(a_outcome, TokenCycleOutcome::Skipped { unreachable_peer: 2 }));
        assert!(
            matches!(a_outcome, TokenCycleOutcome::Skipped { unreachable_peer: 2 }),
            "trial {trial}: A should have genuinely exhausted backoff to B and skipped it \
             for real, got {a_outcome:?} — the deliberate delay may not be long enough \
             on this machine"
        );
        assert!(
            matches!(c_first_outcome, TokenCycleOutcome::Handled { .. }),
            "trial {trial}: C's first hop should have been a genuinely new one, got {c_first_outcome:?}"
        );

        let b_holds = b_state == TaskState::ToPickup(task1);
        let c_holds = c.state() == TaskState::ToPickup(task1);
        if b_holds && c_holds {
            double_commitment_failures += 1;
        }
        if !b_holds && !c_holds {
            orphan_failures += 1;
        }
    }

    println!(
        "[real-transport fork, {TRIALS} trials] real_skips_observed={real_skips_observed} \
         double_commitment_failures={double_commitment_failures} orphan_failures={orphan_failures}"
    );
    assert_eq!(real_skips_observed, TRIALS, "every trial should have exercised the real skip path");
    assert_eq!(double_commitment_failures, 0);
    assert_eq!(orphan_failures, 0);
}
