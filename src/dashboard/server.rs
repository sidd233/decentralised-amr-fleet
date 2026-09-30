//! Dashboard backend, item 24 of `docs/BUILD_PLAN.md`, `docs/PS_AND_ARCHITECTURE.md`
//! §3.6: "`axum` backend, passive WebSocket listener (never sends into the fleet, so it
//! can't become a hidden central coordinator)." Frontend is React + Vite (Decision 11),
//! served as a static build (`frontend/dist`) this same `axum` server hands out —
//! no separate Node process at runtime.
//!
//! Same pure-core/thin-I/O-wrapper split every other module in this project uses:
//! `apply` is the pure decision logic (no I/O, directly unit-testable — what
//! turns one `Received` message into an update to the fleet's known state), while
//! `spawn_listener_thread`/`ws_handler`/`DashboardServer::serve` are the real-UDP/
//! real-WebSocket wrapper around it.
//!
//! Populates §1.3's "real-time positions + battery status" and §3.6's fuller "position,
//! battery, mode, task status per robot" from whatever this passive listener has heard
//! on the shared multicast bus, without inventing any new wire message: position comes
//! from every `Envelope`'s own `sender_position` (`comms.rs`'s entry, `docs/FILE_MAP.md`),
//! battery from `Heartbeat::battery_pct` (Decision 7), mode from `ModeAnnounce::mode`, and
//! per-robot task assignment from `TokenMsg::claimed_tasks` — the token already carries
//! the fleet's one shared source of truth for "who's claimed what" (`task_layer.rs`), so
//! passively watching every token pass by is enough to reconstruct it without a new
//! broadcast. `TokenMsg::claimed_tasks` is the *full* current claim set each time, not a
//! delta, so `apply` refreshes every robot's `current_task` from it in one pass, clearing
//! stale claims exactly as readily as it sets new ones.

use std::collections::HashMap;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::{Path, State};
use axum::http::StatusCode;
use axum::response::{IntoResponse, Json};
use axum::routing::{delete, get, post, put};
use axum::Router;
use serde::{Deserialize, Serialize};
use tokio::sync::watch;
use tower_http::services::ServeDir;

use crate::config::{BOOTSTRAP_TOKEN_ID, CLOCK_DRIVER_ID, DASHBOARD_LISTENER_ID};
use crate::protocol::messages::Mode;
use crate::robot::comms::{Comms, Payload, Received};
use crate::dashboard::scenario::{Scenario, TaskView};
use crate::robot::task_layer::TOKEN_RANGE_CELLS;
use crate::world::grid::Grid;

/// One robot's latest known state, built up from whatever this passive listener has
/// observed so far — any field nothing has arrived for yet stays `None` rather than a
/// fabricated default, so the frontend can render "unknown" honestly instead of a fake
/// zero/idle.
#[derive(Debug, Clone, Serialize, Default, PartialEq)]
pub struct RobotSnapshot {
    pub position: Option<(i32, i32)>,
    /// The logical tick `position` was reported at (from whichever of `Heartbeat`/
    /// `ModeAnnounce`/`PoseIntent` carried it — `TokenMsg` has no tick of its own, so it
    /// never updates this or `position`; see `apply`). Item 26's `tests/
    /// no_collisions_e2e.rs` needs this to correlate every robot's position at the same
    /// logical tick, not just whichever arrived most recently in wall-clock time.
    pub tick: Option<u64>,
    pub battery_pct: Option<f32>,
    pub mode: Option<Mode>,
    pub current_task: Option<u32>,
    /// Whether `current_task` has been picked up yet (from the token's `ClaimEntry`); lets
    /// the operator console tell a finished job from an active one.
    pub task_picked_up: Option<bool>,
    /// Every claim this robot holds in the token. The token never drops a *finished*
    /// job's entry (or it would be claimed again), so a robot that has done several jobs
    /// lists all of them; the operator console works out which one is live.
    pub claims: Vec<ClaimInfo>,
}

#[derive(Debug, Clone, Copy, Serialize, PartialEq, Eq)]
pub struct ClaimInfo {
    pub task_id: u32,
    pub picked_up: bool,
}

/// The full fleet state broadcast to every connected dashboard client. Robot ids are
/// serialized as string keys (`HashMap<u32, _>` isn't valid JSON object-key territory
/// unless the caller uses a numeric-key-aware serializer) so this round-trips as plain
/// JSON to any browser without a custom deserializer on the frontend side.
#[derive(Debug, Clone, Serialize, Default, PartialEq)]
pub struct FleetSnapshot {
    pub robots: HashMap<String, RobotSnapshot>,
    /// Operator-console state (Decision 18), filled in by `AppState::compose` — the
    /// bus-listener's `apply` never touches these.
    pub tasks: Vec<TaskView>,
    pub blocked: Vec<(i32, i32)>,
    /// A dashboard-launched scenario is running.
    pub running: bool,
    /// This dashboard was started with `--manage-fleet`, so the controls work.
    pub can_control: bool,
}

/// `Heartbeat` is the only authoritative position source: it is sent once per tick *after*
/// the robot has moved, so its (position, tick) pair is consistent across the fleet.
/// `PoseIntent` and `ModeAnnounce` are sent mid-tick, *before* the move, and carry the same
/// tick number — mixing them in made a robot appear to step backwards and made two robots
/// look co-located at one tick. They only fill in a robot's first sighting.
fn seed_position(snapshot: &mut RobotSnapshot, position: (i32, i32), tick: u64) {
    if snapshot.position.is_none() {
        snapshot.position = Some(position);
        snapshot.tick = Some(tick);
    }
}

/// Applies one message this listener observed on the bus to the fleet's known state.
/// Pure and I/O-free by design (see this module's own doc comment) — every real-UDP/
/// real-WebSocket concern lives in the wrapper functions below instead.
///
/// Ignores the bus's own reserved sentinel senders (`CLOCK_DRIVER_ID`,
/// `BOOTSTRAP_TOKEN_ID`) outright — found by hand during a real smoke test: the clock
/// driver's `TickMsg` envelope carries a `sender_position` like any other message
/// (`comms.rs`'s `Envelope`, `docs/FILE_MAP.md`), so without this check it showed up in
/// the fleet snapshot as a fake "robot `0`", which §3.6's "position... per robot" was
/// never meant to include.
///
/// `position` and `tick` are always set together, from whichever payload carried them
/// (`Heartbeat`/`ModeAnnounce`/`PoseIntent` each have their own `tick`; `TokenMsg`
/// doesn't, so a token never updates either field) — item 26 added `tick` and this
/// pairing discipline together: setting `position` unconditionally for every payload
/// type (the original item-24 design) while only sometimes updating `tick` alongside it
/// would let a later, tick-less `TokenMsg` update leave a *newer* position paired with a
/// *stale* `tick`, exactly the kind of mismatch `tests/no_collisions_e2e.rs` needs to
/// not happen when it correlates positions across robots by tick.
fn apply(fleet: &mut FleetSnapshot, received: Received) {
    if received.sender_id == CLOCK_DRIVER_ID || received.sender_id == BOOTSTRAP_TOKEN_ID {
        return;
    }
    let sender_key = received.sender_id.to_string();

    match received.payload {
        Payload::Heartbeat(hb) => {
            let snapshot = fleet.robots.entry(sender_key).or_default();
            snapshot.position = Some(received.sender_position);
            snapshot.tick = Some(hb.tick);
            snapshot.battery_pct = Some(hb.battery_pct);
        }
        Payload::ModeAnnounce(ma) => {
            let snapshot = fleet.robots.entry(sender_key).or_default();
            snapshot.mode = Some(ma.mode);
            seed_position(snapshot, received.sender_position, ma.tick);
        }
        Payload::Pose(pose) => {
            let snapshot = fleet.robots.entry(sender_key).or_default();
            seed_position(snapshot, received.sender_position, pose.tick);
        }
        Payload::Token(token) => {
            let mut by_robot: HashMap<u32, Vec<ClaimInfo>> = HashMap::new();
            for c in &token.claimed_tasks {
                by_robot
                    .entry(c.robot_id)
                    .or_default()
                    .push(ClaimInfo { task_id: c.task_id, picked_up: c.picked_up });
            }
            for (&robot_id, claims) in &by_robot {
                let snapshot = fleet.robots.entry(robot_id.to_string()).or_default();
                let last = claims.last().expect("grouped from at least one entry");
                snapshot.current_task = Some(last.task_id);
                snapshot.task_picked_up = Some(last.picked_up);
                snapshot.claims = claims.clone();
            }
            for (id_str, snapshot) in fleet.robots.iter_mut() {
                if let Ok(robot_id) = id_str.parse::<u32>() {
                    if !by_robot.contains_key(&robot_id) {
                        snapshot.current_task = None;
                        snapshot.task_picked_up = None;
                        snapshot.claims.clear();
                    }
                }
            }
        }
        Payload::Tick(_) => {}
        Payload::Ack(_) => {}
        Payload::TaskInject(_) | Payload::TaskRetarget(_) | Payload::BlockCell(_) => {}
    }
}

/// Most frames kept for replay; the oldest are dropped past this.
const MAX_HISTORY_FRAMES: usize = 20_000;

/// How this dashboard may launch a fleet (`--manage-fleet`): the map it validates against
/// and hands to robot processes, and whether it also starts its own clock (standalone runs;
/// under docker compose the separate `clock` container already provides one).
pub struct LauncherConfig {
    pub map_path: String,
    pub spawn_clock: bool,
    pub grid: Grid,
}

/// Shared between the listener thread (writer), the axum handlers (readers/operators).
pub struct AppState {
    tx: watch::Sender<FleetSnapshot>,
    /// One `FleetSnapshot` per logical tick of the current run, for the Replay button.
    history: Mutex<Vec<FleetSnapshot>>,
    /// What the bus listener has heard, before the operator-console overlay.
    last_fleet: Mutex<FleetSnapshot>,
    scenario: Mutex<Option<Scenario>>,
    /// Robot/bootstrap (and optional clock) processes this dashboard launched.
    children: Mutex<Vec<Child>>,
    launcher: Option<LauncherConfig>,
    /// Command broadcaster; `None` when fleet control is off.
    sender: Option<Comms>,
    /// Set by stop/start so the listener thread drops its accumulated fleet state.
    reset_requested: AtomicBool,
}

impl AppState {
    /// The listener's fleet plus the operator overlay: task list with derived statuses,
    /// blocked cells, and flags. A finished or cancelled task no longer shows as any
    /// robot's `current_task`.
    fn compose(&self, fleet: &FleetSnapshot) -> FleetSnapshot {
        let mut view = fleet.clone();
        view.can_control = self.launcher.is_some();
        if let Some(scenario) = self.scenario.lock().unwrap().as_mut() {
            scenario.observe(fleet);
            view.running = true;
            view.tasks = scenario.task_views(fleet);
            view.blocked = scenario.blocked();
            // Show each robot's *live* job: a claim on a finished or cancelled task is
            // just a leftover token entry.
            for robot in view.robots.values_mut() {
                let live = robot.claims.iter().rev().find(|c| scenario.is_live(c.task_id));
                robot.current_task = live.map(|c| c.task_id);
                robot.task_picked_up = live.map(|c| c.picked_up);
            }
        }
        view
    }

    /// Records the listener's latest fleet, then sends the composed view to every client.
    fn publish(&self, fleet: &FleetSnapshot) {
        *self.last_fleet.lock().unwrap() = fleet.clone();
        let view = self.compose(fleet);
        record_frame(&mut self.history.lock().unwrap(), &view);
        // Errors only once every receiver has gone (no clients connected).
        let _ = self.tx.send(view);
    }

    /// Re-sends the current view after an operator action changed the overlay.
    fn republish(&self) {
        let fleet = self.last_fleet.lock().unwrap().clone();
        let _ = self.tx.send(self.compose(&fleet));
    }
}

type SharedState = Arc<AppState>;

// ---- Operator console (Decision 18) -------------------------------------------------

type ApiResponse = (StatusCode, Json<serde_json::Value>);

fn ok() -> ApiResponse {
    (StatusCode::OK, Json(serde_json::json!({ "ok": true })))
}

fn fail(status: StatusCode, msg: impl Into<String>) -> ApiResponse {
    (status, Json(serde_json::json!({ "error": msg.into() })))
}

fn bad_request(msg: impl Into<String>) -> ApiResponse {
    fail(StatusCode::BAD_REQUEST, msg)
}

/// Each command is broadcast this many times, `COMMAND_RESEND_GAP` apart: UDP is lossy and
/// a robot mid-wait can miss one, and every command is idempotent (Decision 18).
const COMMAND_RESENDS: usize = 5;
const COMMAND_RESEND_GAP: std::time::Duration = std::time::Duration::from_millis(120);

/// Sends `send` on the bus a few times, on its own thread so the HTTP reply isn't delayed.
fn broadcast<F>(state: &SharedState, send: F)
where
    F: Fn(&Comms) -> std::io::Result<()> + Send + 'static,
{
    let state = Arc::clone(state);
    thread::spawn(move || {
        let Some(comms) = state.sender.as_ref() else { return };
        for _ in 0..COMMAND_RESENDS {
            if let Err(e) = send(comms) {
                eprintln!("[dashboard] failed to broadcast operator command: {e}");
            }
            thread::sleep(COMMAND_RESEND_GAP);
        }
    });
}

fn require_control(state: &SharedState) -> Result<(), ApiResponse> {
    if state.launcher.is_some() {
        Ok(())
    } else {
        Err(fail(
            StatusCode::FORBIDDEN,
            "fleet control is off: start the dashboard with --manage-fleet",
        ))
    }
}

/// Runs `f` on the running scenario, or fails with "no scenario running".
fn with_scenario<T>(
    state: &SharedState,
    f: impl FnOnce(&mut Scenario) -> Result<T, String>,
) -> Result<T, ApiResponse> {
    let mut guard = state.scenario.lock().unwrap();
    let scenario = guard
        .as_mut()
        .ok_or_else(|| bad_request("no scenario is running: place robots and press Start"))?;
    f(scenario).map_err(bad_request)
}

#[derive(Deserialize)]
struct TaskReq {
    pickup: [i32; 2],
    dropoff: [i32; 2],
}

#[derive(Deserialize)]
struct StartReq {
    robots: Vec<[i32; 2]>,
    #[serde(default)]
    tasks: Vec<TaskReq>,
}

#[derive(Deserialize)]
struct BlockReq {
    cell: [i32; 2],
    blocked: bool,
}

/// Kills every process this dashboard launched and forgets the scenario and its history.
fn stop_scenario_inner(state: &SharedState) {
    for mut child in state.children.lock().unwrap().drain(..) {
        let _ = child.kill();
        let _ = child.wait();
    }
    *state.scenario.lock().unwrap() = None;
    state.history.lock().unwrap().clear();
    state.reset_requested.store(true, Ordering::Release);
    *state.last_fleet.lock().unwrap() = FleetSnapshot::default();
    state.republish();
}

fn spawn_process(args: &[String]) -> std::io::Result<Child> {
    Command::new(std::env::current_exe()?)
        .args(args)
        .stdout(Stdio::null())
        // Errors (a robot's failed send, a lost tick source...) land in the dashboard's log.
        .stderr(Stdio::inherit())
        .spawn()
}

async fn start_handler(State(state): State<SharedState>, Json(req): Json<StartReq>) -> ApiResponse {
    if let Err(e) = require_control(&state) {
        return e;
    }
    let launcher = state.launcher.as_ref().expect("checked by require_control");
    stop_scenario_inner(&state);

    let starts: Vec<(i32, i32)> = req.robots.iter().map(|c| (c[0], c[1])).collect();
    let mut scenario = match Scenario::new(&launcher.grid, starts.clone()) {
        Ok(s) => s,
        Err(e) => return bad_request(e),
    };
    let mut injects = Vec::new();
    for t in &req.tasks {
        match scenario.add_task((t.pickup[0], t.pickup[1]), (t.dropoff[0], t.dropoff[1])) {
            Ok(m) => injects.push(m),
            Err(e) => return bad_request(format!("task {}: {e}", injects.len() + 1)),
        }
    }

    let n = starts.len() as u32;
    let peers = (1..=n).map(|i| i.to_string()).collect::<Vec<_>>().join(",");
    let mut launched = Vec::new();
    let mut commands: Vec<Vec<String>> = Vec::new();
    if launcher.spawn_clock {
        commands.push(vec!["clock".into()]);
    }
    for (i, start) in starts.iter().enumerate() {
        let id = (i + 1) as u32;
        commands.push(vec![
            "robot".into(), "--id".into(), id.to_string(),
            "--map".into(), launcher.map_path.clone(),
            "--start".into(), format!("{},{}", start.0, start.1),
            "--peers".into(), peers.clone(),
            "--seed".into(), id.to_string(),
        ]);
    }
    for args in &commands {
        match spawn_process(args) {
            Ok(child) => launched.push(child),
            Err(e) => {
                for mut c in launched {
                    let _ = c.kill();
                    let _ = c.wait();
                }
                return fail(StatusCode::INTERNAL_SERVER_ERROR, format!("failed to launch {args:?}: {e}"));
            }
        }
    }
    *state.children.lock().unwrap() = launched;
    *state.scenario.lock().unwrap() = Some(scenario);
    state.republish();

    // Robots need a moment to bind their sockets before the first token / tasks are sent
    // (same reasoning as docker-compose.yml's `bootstrap-token` sleep, Decision 13).
    let bg = Arc::clone(&state);
    thread::spawn(move || {
        thread::sleep(std::time::Duration::from_millis(1500));
        if bg.scenario.lock().unwrap().is_none() {
            return; // stopped in the meantime
        }
        if let Ok(child) = spawn_process(&["bootstrap-token".into(), "--peers".into(), peers]) {
            bg.children.lock().unwrap().push(child);
        }
        for m in injects {
            broadcast(&bg, move |c| c.send_task_inject(m));
        }
    });
    ok()
}

async fn stop_handler(State(state): State<SharedState>) -> ApiResponse {
    if let Err(e) = require_control(&state) {
        return e;
    }
    stop_scenario_inner(&state);
    ok()
}

async fn add_task_handler(State(state): State<SharedState>, Json(req): Json<TaskReq>) -> ApiResponse {
    if let Err(e) = require_control(&state) {
        return e;
    }
    let msg = match with_scenario(&state, |s| {
        s.add_task((req.pickup[0], req.pickup[1]), (req.dropoff[0], req.dropoff[1]))
    }) {
        Ok(m) => m,
        Err(e) => return e,
    };
    state.republish();
    broadcast(&state, move |c| c.send_task_inject(msg));
    (StatusCode::OK, Json(serde_json::json!({ "ok": true, "task_id": msg.task_id })))
}

async fn retarget_task_handler(
    State(state): State<SharedState>,
    Path(id): Path<u32>,
    Json(req): Json<TaskReq>,
) -> ApiResponse {
    if let Err(e) = require_control(&state) {
        return e;
    }
    let msg = match with_scenario(&state, |s| {
        s.retarget_task(id, (req.pickup[0], req.pickup[1]), (req.dropoff[0], req.dropoff[1]))
    }) {
        Ok(m) => m,
        Err(e) => return e,
    };
    state.republish();
    broadcast(&state, move |c| c.send_task_retarget(msg));
    ok()
}

async fn cancel_task_handler(State(state): State<SharedState>, Path(id): Path<u32>) -> ApiResponse {
    if let Err(e) = require_control(&state) {
        return e;
    }
    let msg = match with_scenario(&state, |s| s.cancel_task(id)) {
        Ok(m) => m,
        Err(e) => return e,
    };
    state.republish();
    broadcast(&state, move |c| c.send_task_retarget(msg));
    ok()
}

async fn block_handler(State(state): State<SharedState>, Json(req): Json<BlockReq>) -> ApiResponse {
    if let Err(e) = require_control(&state) {
        return e;
    }
    let robot_cells: Vec<(i32, i32)> = state
        .last_fleet
        .lock()
        .unwrap()
        .robots
        .values()
        .filter_map(|r| r.position)
        .collect();
    let msg = match with_scenario(&state, |s| {
        s.set_block((req.cell[0], req.cell[1]), req.blocked, &robot_cells)
    }) {
        Ok(m) => m,
        Err(e) => return e,
    };
    state.republish();
    broadcast(&state, move |c| c.send_block_cell(msg));
    ok()
}


/// How far behind the newest robot a robot may fall before it's ignored as stalled/dead
/// when deciding whether a frame is complete.
const STALL_TICKS: u64 = 20;

/// The latest tick every *active* robot has reported: the minimum tick among robots within
/// `STALL_TICKS` of the newest. A frame is recorded when this advances, so it never mixes
/// robots from different ticks (a killed robot stops counting after `STALL_TICKS`).
fn newest_tick(fleet: &FleetSnapshot) -> Option<u64> {
    let newest = fleet.robots.values().filter_map(|r| r.tick).max()?;
    fleet
        .robots
        .values()
        .filter_map(|r| r.tick)
        .filter(|&t| t + STALL_TICKS >= newest)
        .min()
}

/// Pure replay-buffer logic: appends `fleet` as a new frame whenever the newest tick in
/// it has advanced past the last recorded frame's, and caps the buffer's length.
fn record_frame(history: &mut Vec<FleetSnapshot>, fleet: &FleetSnapshot) {
    let Some(tick) = newest_tick(fleet) else { return };
    if history.last().and_then(newest_tick).is_some_and(|last| last >= tick) {
        return;
    }
    history.push(fleet.clone());
    if history.len() > MAX_HISTORY_FRAMES {
        history.remove(0);
    }
}

async fn history_handler(State(state): State<SharedState>) -> Json<Vec<FleetSnapshot>> {
    Json(state.history.lock().unwrap().clone())
}

/// Binds a wide-range, receive-only `Comms` (same `TOKEN_RANGE_CELLS` reasoning as
/// `task_layer.rs`'s own instance: a dashboard watching the whole fleet shouldn't
/// range-scope itself the way a real robot's `PoseIntent` traffic does) and folds every
/// message it hears into `state` for as long as the process runs. Never calls any
/// `Comms::send_*` method — the passive-listener guarantee `docs/PS_AND_ARCHITECTURE.md`
/// §3.6 requires isn't just documentation, this thread has no path to violate it.
fn spawn_listener_thread(state: SharedState) {
    thread::spawn(move || {
        let comms = match Comms::with_range(DASHBOARD_LISTENER_ID, (0, 0), TOKEN_RANGE_CELLS) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("[dashboard] failed to bind passive listener: {e}");
                return;
            }
        };
        let mut fleet = FleetSnapshot::default();
        let mut last_bootstrap: Option<Instant> = None;
        loop {
            if state.reset_requested.swap(false, Ordering::AcqRel) {
                fleet = FleetSnapshot::default();
            }
            match comms.recv_filtered() {
                Ok(Some(received)) => {
                    // The bootstrap token is sent once at the start of every run, so it
                    // marks "replay from the start" — drop the previous run's history.
                    // It's resent a few times per run, so only a resend-free gap counts.
                    if received.sender_id == BOOTSTRAP_TOKEN_ID
                        && last_bootstrap.map_or(true, |t| t.elapsed() > Duration::from_secs(10))
                    {
                        fleet = FleetSnapshot::default();
                        state.history.lock().unwrap().clear();
                    }
                    if received.sender_id == BOOTSTRAP_TOKEN_ID {
                        last_bootstrap = Some(Instant::now());
                    }
                    apply(&mut fleet, received);
                    state.publish(&fleet);
                }
                Ok(None) => {
                    // No read timeout is set on this socket, so this arm is unreachable
                    // in practice — `recv_filtered` blocks until a message arrives.
                }
                Err(e) => {
                    eprintln!("[dashboard] recv failed, continuing: {e}");
                }
            }
        }
    });
}

async fn ws_handler(ws: WebSocketUpgrade, State(state): State<SharedState>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_socket(socket, state))
}

async fn handle_socket(mut socket: WebSocket, state: SharedState) {
    let mut rx = state.tx.subscribe();

    let Ok(initial) = serde_json::to_string(&*rx.borrow_and_update()) else {
        return;
    };
    if socket.send(Message::Text(initial.into())).await.is_err() {
        return;
    }

    loop {
        tokio::select! {
            changed = rx.changed() => {
                if changed.is_err() {
                    break;
                }
                let Ok(json) = serde_json::to_string(&*rx.borrow_and_update()) else {
                    continue;
                };
                if socket.send(Message::Text(json.into())).await.is_err() {
                    break;
                }
            }
            incoming = socket.recv() => {
                // Passive means passive both ways: this endpoint never acts on anything
                // a browser sends, it only needs to notice the connection closing.
                match incoming {
                    Some(Ok(_)) => continue,
                    _ => break,
                }
            }
        }
    }
}

/// The dashboard's `axum` app: `/ws` for live fleet state, everything else served as
/// static files from `frontend_dir` (Vite's `npm run build` output, per Decision 11).
pub struct DashboardServer {
    router: Router,
    state: SharedState,
}

impl DashboardServer {
    /// Spawns the bus listener and builds the router (`launcher` = `--manage-fleet`). Binding/serving doesn't
    /// happen until `serve` is called, so construction can't itself fail on a taken port.
    pub fn new(frontend_dir: &str, launcher: Option<LauncherConfig>) -> Self {
        let (tx, _rx) = watch::channel(FleetSnapshot::default());
        let sender = launcher.as_ref().and_then(|_| {
            Comms::with_range(DASHBOARD_LISTENER_ID, (0, 0), TOKEN_RANGE_CELLS)
                .map_err(|e| eprintln!("[dashboard] failed to bind command sender: {e}"))
                .ok()
        });
        let launcher = launcher.filter(|_| sender.is_some());
        let state: SharedState = Arc::new(AppState {
            tx,
            history: Mutex::new(Vec::new()),
            last_fleet: Mutex::new(FleetSnapshot::default()),
            scenario: Mutex::new(None),
            children: Mutex::new(Vec::new()),
            launcher,
            sender,
            reset_requested: AtomicBool::new(false),
        });

        // A browser connecting before any robot is on the bus would otherwise get the bare
        // default snapshot (`can_control: false`) and never see the console that launches
        // the fleet. `send_replace` because `send` doesn't store a value with no receivers.
        state.tx.send_replace(state.compose(&FleetSnapshot::default()));

        spawn_listener_thread(Arc::clone(&state));

        let router = Router::new()
            .route("/ws", get(ws_handler))
            .route("/history", get(history_handler))
            .route("/api/scenario/start", post(start_handler))
            .route("/api/scenario/stop", post(stop_handler))
            .route("/api/tasks", post(add_task_handler))
            .route("/api/tasks/{id}", put(retarget_task_handler))
            .route("/api/tasks/{id}", delete(cancel_task_handler))
            .route("/api/block", post(block_handler))
            .fallback_service(ServeDir::new(frontend_dir))
            .with_state(Arc::clone(&state));

        Self { router, state }
    }

    /// Binds `port` and serves forever (or until the process is killed) — matches
    /// `robot`/`clock`'s own "runs forever unless told otherwise" default.
    pub async fn serve(self, port: u16) -> std::io::Result<()> {
        let listener = tokio::net::TcpListener::bind(("0.0.0.0", port)).await?;
        axum::serve(listener, self.router).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::messages::{ClaimEntry, Heartbeat, ModeAnnounce, TokenMsg};

    /// The first snapshot a browser gets, before any robot has spoken, must already say
    /// whether the operator console is available (it is what launches the fleet).
    #[test]
    fn initial_snapshot_reports_can_control_before_any_robot_is_heard() {
        let grid = Grid::load("maps/warehouse-10-20-10-2-1.map").expect("map should load");
        let launcher = LauncherConfig {
            map_path: "maps/warehouse-10-20-10-2-1.map".into(),
            spawn_clock: false,
            grid,
        };
        let managed = DashboardServer::new("frontend/dist", Some(launcher));
        assert!(managed.state.tx.borrow().can_control);
        assert!(!managed.state.tx.borrow().running);

        let read_only = DashboardServer::new("frontend/dist", None);
        assert!(!read_only.state.tx.borrow().can_control);
    }

    fn received(sender_id: u32, position: (i32, i32), payload: Payload) -> Received {
        Received {
            sender_id,
            sender_position: position,
            payload,
        }
    }

    /// Phase 7 gate fallback (`docs/TESTING_PLAN.md`): no dedicated `tests/*.rs` file
    /// covers this in isolation yet (item 26's `no_collisions_e2e.rs` needs item 25's
    /// docker-compose stack first), so `apply` — the one piece of real decision logic in
    /// this module — gets thorough in-file coverage instead, same discipline as
    /// `perception.rs`/`metrics.rs` used for their own Phase-6-without-a-gate items.
    #[test]
    fn a_frame_waits_for_every_active_robot_and_ignores_a_stalled_one() {
        let hb = |id, tick| Payload::Heartbeat(Heartbeat { robot_id: id, tick, battery_pct: 90.0 });
        let mut fleet = FleetSnapshot::default();
        let mut history = Vec::new();
        for (id, tick) in [(1, 10), (2, 9), (2, 10), (1, 11), (2, 11)] {
            apply(&mut fleet, received(id, (tick as i32, 0), hb(id, tick)));
            record_frame(&mut history, &fleet);
        }
        // 1@10 alone -> min 10 recorded; 2@9 lowers nothing new; 2@10 -> min 10 (dup);
        // 1@11 -> min still 10; 2@11 -> min 11 recorded.
        let ticks: Vec<_> = history.iter().map(|f| newest_tick(f).unwrap()).collect();
        assert_eq!(ticks, vec![10, 11], "no frame until both robots reached the tick");

        // Robot 2 stalls; once it's over STALL_TICKS behind, robot 1 alone drives frames.
        apply(&mut fleet, received(1, (0, 0), hb(1, 11 + STALL_TICKS + 1)));
        record_frame(&mut history, &fleet);
        assert_eq!(history.len(), 3);
    }

    #[test]
    fn record_frame_appends_only_when_the_newest_tick_advances() {
        let hb = |tick| Payload::Heartbeat(Heartbeat { robot_id: 1, tick, battery_pct: 90.0 });
        let mut fleet = FleetSnapshot::default();
        let mut history = Vec::new();
        record_frame(&mut history, &fleet);
        assert!(history.is_empty(), "no ticks heard yet, nothing to record");
        for tick in [3, 3, 4, 2, 5] {
            apply(&mut fleet, received(1, (tick as i32, 0), hb(tick)));
            record_frame(&mut history, &fleet);
        }
        assert_eq!(history.len(), 3, "only ticks 3, 4 and 5 advance");
        assert_eq!(history[0].robots["1"].position, Some((3, 0)));
    }

    #[test]
    fn only_heartbeat_moves_a_known_robot_but_others_seed_a_first_sighting() {
        let pose = |tick, pos| {
            Payload::Pose(crate::protocol::messages::PoseIntent {
                robot_id: 1,
                tick,
                position: pos,
                intended_next: pos,
                priority: 0.0,
                exhausted: true,
            })
        };
        let mut fleet = FleetSnapshot::default();
        // First sighting via a mid-tick Pose seeds position and tick together.
        apply(&mut fleet, received(1, (9, 10), pose(2, (9, 10))));
        assert_eq!((fleet.robots["1"].position, fleet.robots["1"].tick), (Some((9, 10)), Some(2)));

        // A Heartbeat (post-move, once per tick) is authoritative.
        apply(
            &mut fleet,
            received(1, (5, 6), Payload::Heartbeat(Heartbeat { robot_id: 1, tick: 3, battery_pct: 90.0 })),
        );
        assert_eq!((fleet.robots["1"].position, fleet.robots["1"].tick), (Some((5, 6)), Some(3)));

        // Pre-move Pose / ModeAnnounce carrying the same tick must not overwrite it — that
        // mix made robots appear to step backwards and to share a cell.
        apply(&mut fleet, received(1, (7, 8), pose(4, (7, 8))));
        apply(
            &mut fleet,
            received(1, (7, 8), Payload::ModeAnnounce(ModeAnnounce { robot_id: 1, mode: Mode::Cautious, tick: 4 })),
        );
        assert_eq!((fleet.robots["1"].position, fleet.robots["1"].tick), (Some((5, 6)), Some(3)));
        assert_eq!(fleet.robots["1"].mode, Some(Mode::Cautious), "the announce still sets mode");
    }

    #[test]
    fn heartbeat_sets_battery_and_mode_announce_sets_mode_independently() {
        let mut fleet = FleetSnapshot::default();
        apply(
            &mut fleet,
            received(2, (0, 0), Payload::Heartbeat(Heartbeat { robot_id: 2, tick: 1, battery_pct: 77.5 })),
        );
        assert_eq!(fleet.robots["2"].battery_pct, Some(77.5));
        assert_eq!(fleet.robots["2"].mode, None, "mode shouldn't be inferred from a Heartbeat alone");

        apply(
            &mut fleet,
            received(2, (0, 0), Payload::ModeAnnounce(ModeAnnounce { robot_id: 2, mode: Mode::Autonomous, tick: 2 })),
        );
        assert_eq!(fleet.robots["2"].mode, Some(Mode::Autonomous));
        assert_eq!(fleet.robots["2"].battery_pct, Some(77.5), "a ModeAnnounce shouldn't clobber battery");
    }

    #[test]
    fn token_assigns_current_task_to_every_claimant_in_one_pass() {
        let mut fleet = FleetSnapshot::default();
        let token = TokenMsg {
            seq: 3,
            holder_id: 9,
            claimed_tasks: vec![
                ClaimEntry { task_id: 10, robot_id: 1, picked_up: false },
                ClaimEntry { task_id: 11, robot_id: 2, picked_up: false },
            ],
            epoch: 0,
            creator: 0,
        };
        apply(&mut fleet, received(9, (0, 0), Payload::Token(token)));

        assert_eq!(fleet.robots["1"].current_task, Some(10));
        assert_eq!(fleet.robots["2"].current_task, Some(11));
    }

    #[test]
    fn a_later_token_clears_a_released_task_for_its_old_claimant() {
        let mut fleet = FleetSnapshot::default();
        let first = TokenMsg {
            seq: 1,
            holder_id: 5,
            claimed_tasks: vec![ClaimEntry { task_id: 20, robot_id: 1, picked_up: false }],
            epoch: 0,
            creator: 0,
        };
        apply(&mut fleet, received(5, (0, 0), Payload::Token(first)));
        assert_eq!(fleet.robots["1"].current_task, Some(20));

        // Robot 1's task became unreachable and was released; the next token no longer
        // lists it — `claimed_tasks` is the full set each time, not a delta.
        let second = TokenMsg {
            seq: 2,
            holder_id: 6,
            claimed_tasks: vec![],
            epoch: 0,
            creator: 0,
        };
        apply(&mut fleet, received(6, (0, 0), Payload::Token(second)));
        assert_eq!(
            fleet.robots["1"].current_task, None,
            "a robot's stale claim must clear once a later token stops listing it"
        );
    }

    #[test]
    fn reserved_sentinel_senders_never_appear_as_robots() {
        let mut fleet = FleetSnapshot::default();
        apply(
            &mut fleet,
            received(
                CLOCK_DRIVER_ID,
                (0, 0),
                Payload::Tick(crate::protocol::messages::TickMsg { tick: 1 }),
            ),
        );
        apply(
            &mut fleet,
            received(
                BOOTSTRAP_TOKEN_ID,
                (0, 0),
                Payload::Token(TokenMsg { seq: 0, holder_id: 1, claimed_tasks: vec![], epoch: 0, creator: 0 }),
            ),
        );
        assert!(
            fleet.robots.is_empty(),
            "the clock driver and the bootstrap process are not robots"
        );
    }

    #[test]
    fn tick_payload_is_a_true_no_op() {
        // Only `clock_driver.rs` ever sends a `TickMsg`, always under `CLOCK_DRIVER_ID`
        // (already filtered out above `apply`'s match) — a `Tick` from any other sender
        // id is unrealistic, but `apply` must still not fabricate a robot entry for it.
        let mut fleet = FleetSnapshot::default();
        apply(
            &mut fleet,
            received(1, (1, 1), Payload::Tick(crate::protocol::messages::TickMsg { tick: 5 })),
        );
        assert!(
            fleet.robots.is_empty(),
            "a Tick payload carries no tick of its own robot's position and must not create an entry"
        );
    }

    #[test]
    fn a_token_never_updates_position_or_tick() {
        // Regression for the item-26 pairing fix: setting `position` unconditionally
        // for every payload type (the original item-24 design) while `tick` only
        // sometimes updated would let a tick-less `TokenMsg` leave a *newer* position
        // paired with a *stale* tick — exactly what `no_collisions_e2e.rs` cannot
        // tolerate when it correlates robots by tick.
        let mut fleet = FleetSnapshot::default();
        apply(
            &mut fleet,
            received(1, (5, 6), Payload::Heartbeat(Heartbeat { robot_id: 1, tick: 3, battery_pct: 90.0 })),
        );
        apply(
            &mut fleet,
            received(
                1,
                (99, 99),
                Payload::Token(TokenMsg { seq: 1, holder_id: 2, claimed_tasks: vec![], epoch: 0, creator: 0 }),
            ),
        );
        assert_eq!(
            fleet.robots["1"].position, Some((5, 6)),
            "a token from robot 1 must not overwrite its last real position"
        );
        assert_eq!(fleet.robots["1"].tick, Some(3));
    }
}
