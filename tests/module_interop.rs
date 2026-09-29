//! Integration test proving `robot::comms`, `robot::planner_pibt` (`DistributedAgent`),
//! `robot::task_layer`, and `robot::mode_state_machine` actually interoperate when
//! combined, not just individually pass their own dedicated test files.
//!
//! Scope, stated plainly: this is **not** item 20's production tick loop
//! (`robot/mod.rs` doesn't exist yet). It doesn't attempt item 20's harder problem of
//! interleaving token circulation and PIBT movement concurrently, tick by tick, forever.
//! What it does exercise, all through real code and real UDP sockets (no shortcuts):
//! 1. A real token-passing bootstrap (`TaskLayer::run_one_cycle`) hands one robot a task.
//! 2. That robot's `DistributedAgent` is `retarget`ed to the task's pickup, then moves
//!    there through real PIBT negotiation over real sockets — collision-checked the same
//!    way `tests/pibt_single_process.rs`'s distributed scenarios are.
//! 3. `TaskLayer::on_position_update` is driven by the agent's *real*, PIBT-negotiated
//!    position each tick — proving the task state machine advances off actual movement,
//!    not a synthetic position jump — through pickup, then retargeted again to dropoff.
//! 4. `ModeStateMachine::observe_heartbeat` is advanced every tick alongside the other
//!    two, fed a synthetic healthy heartbeat, proving it coexists in the same loop
//!    without interfering with the real-time-sensitive PIBT round protocol.
//! 5. Each agent's `robot::metrics::BatteryModel` is drained every tick based on whether
//!    that agent's *real* PIBT-negotiated position actually changed, broadcast as a real
//!    `Heartbeat` over its own socket, and picked up by a third, passive, wide-range
//!    `Comms` standing in for the future dashboard (item 24) — proving the whole
//!    battery pipeline (Decision 7) actually works end to end: real movement -> real
//!    drain -> real wire message -> real delivery, not just `BatteryModel`'s own
//!    isolated unit tests.
//!
//! `retarget` (added to `DistributedAgent` for this) is the one piece of new glue this
//! test needed: without it, an agent's goal was fixed forever at construction, which was
//! fine for Phase 3's static-goal scope but not once `task_layer.rs` needs to redirect a
//! robot mid-run. Everything else here is existing, already-tested module code.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Barrier, Mutex};
use std::thread;
use std::time::Duration;

use sih26123::protocol::messages::{Heartbeat, Mode, TokenMsg};
use sih26123::robot::comms::{Comms, Payload};
use sih26123::robot::metrics::BatteryModel;
use sih26123::robot::mode_state_machine::ModeStateMachine;
use sih26123::robot::planner_pibt::{has_no_collisions, Cell, DistributedAgent};
use sih26123::robot::task_layer::{Task, TaskLayer, TaskState, TokenCycleOutcome};
use sih26123::world::grid::Grid;

static SEQUENTIAL: Mutex<()> = Mutex::new(());
const MAX_MOVE_TICKS: usize = 500;

fn test_grid() -> Grid {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map");
    Grid::load(path).expect("map should load")
}

fn traces_to_configs(traces: &[Vec<Cell>]) -> Vec<Vec<Cell>> {
    let len = traces[0].len();
    (0..len)
        .map(|t| traces.iter().map(|trace| trace[t]).collect())
        .collect()
}

/// Confirms the full battery pipeline (Decision 7) actually worked during a movement
/// phase: the observer received real readings from both real robots over real UDP
/// (proving delivery, not just that `BatteryModel` computed something nobody saw), every
/// reading is a sane percentage, and the mover (id 1) ends up with less battery than the
/// near-stationary robot (id 2) — proving `drain_tick`'s `moved` flag was threaded
/// through from *real* PIBT movement, not a hardcoded test value.
fn assert_battery_pipeline_worked(readings: &[(u32, f32)], phase_label: &str) {
    assert!(
        !readings.is_empty(),
        "{phase_label}: observer received no heartbeats at all — battery pipeline is not wired up"
    );
    for &(id, pct) in readings {
        assert!(
            (0.0..=100.0).contains(&pct),
            "{phase_label}: robot {id} reported an out-of-range battery_pct {pct}"
        );
    }

    let last_for = |id: u32| -> f32 {
        readings
            .iter()
            .rev()
            .find(|&&(rid, _)| rid == id)
            .unwrap_or_else(|| panic!("{phase_label}: observer never heard from robot {id}"))
            .1
    };
    let mover_final = last_for(1);
    let stationary_final = last_for(2);
    assert!(
        mover_final < stationary_final,
        "{phase_label}: the moving robot (battery {mover_final}) should have drained more \
         than the near-stationary one (battery {stationary_final})"
    );
}

/// Runs two real `DistributedAgent`s (real UDP, real PIBT negotiation) from
/// `(start1, start2)` toward `(goal1, goal2)`, stopping once both arrive or
/// `MAX_MOVE_TICKS` elapses. Also advances a single shared `ModeStateMachine` with a
/// synthetic healthy heartbeat once per tick, on the same thread that checks for
/// convergence — proving the mode machine can be driven from inside a real PIBT tick
/// loop without disrupting its timing — and drains each agent's own `BatteryModel` based
/// on whether its *real* position actually changed that tick, broadcasting the result as
/// a real `Heartbeat` picked up by a passive wide-range observer standing in for the
/// future dashboard. Returns both agents' position traces, the mode observed after each
/// tick, and every `(sender_id, battery_pct)` reading the observer actually received off
/// the wire (same trace shape `tests/pibt_single_process.rs`'s
/// `run_distributed_scenario`/`traces_to_configs` use, reused where the shapes match).
fn move_and_track(
    grid: &Grid,
    start1: Cell,
    goal1: Cell,
    start2: Cell,
    goal2: Cell,
) -> (Vec<Cell>, Vec<Cell>, Vec<Mode>, Vec<(u32, f32)>) {
    let after_resolve = Barrier::new(2);
    let after_stop_check = Barrier::new(2);
    let stop = AtomicBool::new(false);
    let positions: Mutex<Vec<Cell>> = Mutex::new(vec![start1, start2]);
    let mode_sm: Mutex<ModeStateMachine> = Mutex::new(ModeStateMachine::new());
    let mode_history: Mutex<Vec<Mode>> = Mutex::new(Vec::new());
    let battery_readings: Mutex<Vec<(u32, f32)>> = Mutex::new(Vec::new());
    let goals = [goal1, goal2];

    let comms1 = Comms::new(1, (start1.0 as i32, start1.1 as i32)).expect("bind agent1 comms");
    let comms2 = Comms::new(2, (start2.0 as i32, start2.1 as i32)).expect("bind agent2 comms");
    let agents_init = [(1u32, start1, goal1, comms1), (2u32, start2, goal2, comms2)];

    // Passive listener, deliberately wide-range (unlike the real per-robot
    // `COMMS_RANGE_CELLS`) — the future dashboard (item 24) is meant to see every
    // robot's heartbeat regardless of inter-robot distance, same as it listens
    // passively to the whole multicast bus rather than range-scoping itself.
    let observer = Comms::with_range(999, (0, 0), 1_000_000).expect("bind observer comms");
    observer
        .set_read_timeout(Some(Duration::from_millis(20)))
        .expect("set observer read timeout");

    thread::scope(|scope| {
        let after_resolve = &after_resolve;
        let after_stop_check = &after_stop_check;
        let stop = &stop;
        let positions = &positions;
        let mode_sm = &mode_sm;
        let mode_history = &mode_history;
        let battery_readings = &battery_readings;
        let goals = &goals;

        scope.spawn(move || {
            while !stop.load(Ordering::Acquire) {
                if let Ok(Some(received)) = observer.recv_filtered() {
                    if let Payload::Heartbeat(hb) = received.payload {
                        battery_readings.lock().unwrap().push((hb.robot_id, hb.battery_pct));
                    }
                }
            }
        });

        let handles: Vec<_> = agents_init
            .into_iter()
            .enumerate()
            .map(|(i, (id, start, goal, comms))| {
                let mut agent = DistributedAgent::new(id, grid, start, goal, 1000 + id as u64, 2, comms);
                let heartbeat_comms =
                    Comms::new(id, (start.0 as i32, start.1 as i32)).expect("bind heartbeat comms");
                scope.spawn(move || {
                    let mut battery = BatteryModel::new();
                    let mut trace = vec![agent.position()];
                    for tick in 0..MAX_MOVE_TICKS as u64 {
                        if stop.load(Ordering::Acquire) {
                            break;
                        }
                        let before = agent.position();
                        agent.resolve_tick(tick);
                        let after = agent.position();
                        trace.push(after);
                        positions.lock().unwrap()[i] = after;

                        battery.drain_tick(after != before);
                        heartbeat_comms
                            .send_heartbeat(Heartbeat {
                                robot_id: id,
                                tick,
                                battery_pct: battery.percent(),
                            })
                            .expect("send heartbeat");

                        let result = after_resolve.wait();
                        if result.is_leader() {
                            let mut sm = mode_sm.lock().unwrap();
                            sm.observe_heartbeat(Some(0)); // synthetic, always-healthy heartbeat
                            mode_history.lock().unwrap().push(sm.mode());
                            drop(sm);

                            let all_arrived = positions
                                .lock()
                                .unwrap()
                                .iter()
                                .zip(goals.iter())
                                .all(|(p, g)| p == g);
                            if all_arrived {
                                stop.store(true, Ordering::Release);
                            }
                        }
                        after_stop_check.wait();
                    }
                    trace
                })
            })
            .collect();

        let traces: Vec<Vec<Cell>> = handles.into_iter().map(|h| h.join().unwrap()).collect();
        let history = mode_history.lock().unwrap().clone();
        // Give the observer thread a moment past `stop` to drain whatever's still in
        // flight, then read out what it collected. It's a daemon-style thread scoped to
        // this call, so we don't join it explicitly — `thread::scope` still waits for it
        // before returning, since every spawned thread in a scope is joined at scope exit.
        thread::sleep(Duration::from_millis(50));
        let readings = battery_readings.lock().unwrap().clone();
        (traces[0].clone(), traces[1].clone(), history, readings)
    })
}

#[test]
fn task_layer_planner_and_mode_state_machine_interoperate_over_real_udp() {
    let _guard = SEQUENTIAL.lock().unwrap();
    let grid = test_grid();
    let free = grid.free_cells();
    let pos1 = free[0];
    let pos2 = free[free.len() - 1];
    let task = Task {
        task_id: 1,
        pickup: free[free.len() / 2],
        dropoff: free[free.len() / 2 + 1],
    };

    // --- Phase 1: real-UDP token bootstrap. Robot 1 is the only one that can claim the
    // single task; robot 2 stays idle. Same bootstrap pattern as
    // `tests/token_passing.rs`'s retry test (a third, out-of-ring `Comms` originates the
    // token, since a robot can't hear its own loopback broadcast).
    let mut layer1 =
        TaskLayer::new(1, &grid, vec![task], vec![1, 2], pos1).expect("bind task layer 1");
    let mut layer2 =
        TaskLayer::new(2, &grid, vec![task], vec![1, 2], pos2).expect("bind task layer 2");
    let bootstrapper = Comms::new(99, (0, 0)).expect("bind bootstrapper");

    let claim1 = thread::scope(|scope| {
        let h1 = scope.spawn(|| layer1.run_one_cycle(pos1, Duration::from_secs(2)));
        let h2 = scope.spawn(|| layer2.run_one_cycle(pos2, Duration::from_secs(2)));
        bootstrapper
            .send_token(TokenMsg {
                seq: 0,
                holder_id: 1,
                claimed_tasks: vec![],
                epoch: 0,
                creator: 0,
            })
            .expect("bootstrapper sends initial token");
        let (outcome1, _incidentals1) = h1.join().expect("robot 1 thread panicked");
        h2.join().expect("robot 2 thread panicked");
        outcome1
    });

    match claim1 {
        TokenCycleOutcome::Handled { claimed } => {
            assert_eq!(claimed, Some(task), "robot 1 should have claimed the only task");
        }
        TokenCycleOutcome::NoTokenArrived => panic!("robot 1 never received the bootstrap token"),
        TokenCycleOutcome::Skipped { unreachable_peer } => panic!(
            "robot 1 should have delivered to robot 2 directly, not skipped peer {unreachable_peer}"
        ),
        TokenCycleOutcome::Regenerated { .. } => {
            panic!("robot 1 should not have hit the watchdog in this short, healthy scenario")
        }
    }
    assert_eq!(layer1.state(), TaskState::ToPickup(task));
    assert_eq!(layer2.state(), TaskState::Idle, "robot 2 had nothing left to claim");

    // --- Phase 2: real-UDP PIBT movement toward pickup, mode machine advanced alongside.
    let pickup_goal = layer1.current_goal().expect("robot 1 should be goal-directed");
    assert_eq!(pickup_goal, task.pickup);
    let (trace1, trace2, modes, battery1) = move_and_track(&grid, pos1, pickup_goal, pos2, pos2);

    assert!(
        has_no_collisions(&traces_to_configs(&[trace1.clone(), trace2.clone()])),
        "collision detected while robot 1 moved to pickup"
    );
    assert_eq!(
        *trace1.last().unwrap(),
        task.pickup,
        "robot 1 must actually reach the pickup cell within the tick budget"
    );
    assert!(
        modes.iter().all(|&m| m == Mode::Cooperative),
        "mode should stay Cooperative throughout a healthy-heartbeat run, got {modes:?}"
    );
    assert_battery_pipeline_worked(&battery1, "pickup phase");

    // Task state must advance off the agent's *real* final position, not a shortcut.
    layer1.on_position_update(*trace1.last().unwrap());
    assert_eq!(
        layer1.state(),
        TaskState::ToDropoff(task),
        "arriving at pickup (via real PIBT movement) should advance the task state"
    );

    // --- Phase 3: retarget to dropoff, same real-UDP movement + mode tracking. ---
    let dropoff_goal = layer1.current_goal().expect("robot 1 should still be goal-directed");
    assert_eq!(dropoff_goal, task.dropoff);
    let (trace1b, trace2b, modes2, battery2) =
        move_and_track(&grid, task.pickup, dropoff_goal, pos2, pos2);

    assert!(
        has_no_collisions(&traces_to_configs(&[trace1b.clone(), trace2b.clone()])),
        "collision detected while robot 1 moved to dropoff"
    );
    assert_eq!(
        *trace1b.last().unwrap(),
        task.dropoff,
        "robot 1 must actually reach the dropoff cell within the tick budget"
    );
    assert!(
        modes2.iter().all(|&m| m == Mode::Cooperative),
        "mode should stay Cooperative throughout, got {modes2:?}"
    );
    assert_battery_pipeline_worked(&battery2, "dropoff phase");

    layer1.on_position_update(*trace1b.last().unwrap());
    assert_eq!(
        layer1.state(),
        TaskState::Idle,
        "arriving at dropoff (via real PIBT movement) should complete the task"
    );
}
