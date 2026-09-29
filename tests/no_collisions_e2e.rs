//! Item 26 of `docs/BUILD_PLAN.md`, Phase 7's gate (`docs/TESTING_PLAN.md`): "full stack
//! via `docker-compose up`, 3+ robot containers, clock driver, dashboard — run for a
//! fixed number of ticks, then parse `results/metrics.json` or a dedicated test-mode
//! metrics dump, assert collision count is exactly 0."
//!
//! Drives the real `docker-compose.yml` (item 25) directly via `docker compose`, then
//! observes the real fleet not through a second raw multicast listener but through the
//! dashboard's own already-verified `/ws` feed (item 24) — its `8080:8080` port mapping
//! is already published to the host, so a plain WebSocket client here is enough; no new
//! way to reach into the compose network is needed. `FleetSnapshot`'s `tick` field
//! (added by this same item, `dashboard/server.rs`) is what lets this test correlate
//! every robot's position at the *same logical tick*, not just whichever update arrived
//! most recently in wall-clock time — collecting `(tick, robot_id) -> position` across
//! the run and feeding the result to `has_no_collisions` (the exact same checker item
//! 10's `tests/pibt_single_process.rs` already proved correct) is the real e2e test:
//! reusing that checker, not reimplementing collision logic a second time.
//!
//! **Ignored by default** — needs Docker, and rebuilds+runs the full stack (~1-2
//! minutes), an honest cost for testing the real thing rather than a shortcut around it
//! (same discipline `pibt_single_process.rs`'s distributed suite already established).
//! Run explicitly: `cargo test --test no_collisions_e2e -- --ignored --nocapture`.
//!
//! Writes a test-mode metrics dump to `results/no_collisions_e2e_last_run.json` rather
//! than `results/metrics.json` — the latter is `harness/metrics_schema.json`'s full
//! cross-system shape (baseline comparison, task-completion timing, mode occupancy),
//! which only `run_experiment.rs` (item 28, not yet built) actually has the data to
//! populate; `docs/TESTING_PLAN.md` explicitly allows "a dedicated test-mode metrics
//! dump" as the alternative at this stage.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;

use futures_util::StreamExt;
use serde::Deserialize;

use sih26123::robot::planner_pibt::{has_no_collisions, Config};

const DASHBOARD_WS_URL: &str = "ws://127.0.0.1:8080/ws";
const ROBOT_IDS: [u32; 6] = [1, 2, 3, 4, 5, 6];
const OVERALL_DEADLINE: Duration = Duration::from_secs(180);
const COMPOSE_POLL_INTERVAL: Duration = Duration::from_secs(3);
/// Minimum number of ticks every robot reported *the same* tick number for, before this
/// counts as a meaningful sample rather than a vacuous pass. Measured, not guessed: after
/// Decision 17's fix (a far-away robot's `recv_filtered` blocking ~30s on out-of-range
/// traffic) real runs of the six-container stack observed 73-79 such ticks; 30 leaves a
/// wide margin against CPU-contention variance while still failing loudly if any robot
/// stalls again. Not the plan's full 300 ticks: the dashboard's `/ws` feed samples
/// snapshots rather than relaying every tick, so this is a sample of the run, not all of it.
const MIN_COMPLETE_TICKS: usize = 30;

#[derive(Deserialize)]
struct RobotSnapshotView {
    position: Option<(i32, i32)>,
    tick: Option<u64>,
}

#[derive(Deserialize)]
struct FleetSnapshotView {
    robots: HashMap<String, RobotSnapshotView>,
}

/// Brings the real stack down on drop, including on a failed assertion/panic — a test
/// that leaves 6 containers running behind it is worse than one that fails cleanly.
struct ComposeDownGuard {
    dir: PathBuf,
}

impl Drop for ComposeDownGuard {
    fn drop(&mut self) {
        let _ = Command::new("docker")
            .args(["compose", "--profile", "smoke", "down", "--remove-orphans"])
            .current_dir(&self.dir)
            .status();
    }
}

/// `true` once every service except `dashboard`/`clock` (both deliberately run forever,
/// Decision 14 — `docker-compose.yml`'s own comment) has stopped — the signal that this
/// run's fixed per-robot `--max-ticks` budget has played out.
fn compose_workers_finished(dir: &Path) -> bool {
    const RUNS_FOREVER: [&str; 2] = ["dashboard", "clock"];
    let Ok(output) = Command::new("docker")
        .args(["compose", "--profile", "smoke", "ps", "--status", "running", "--format", "{{.Service}}"])
        .current_dir(dir)
        .output()
    else {
        return false;
    };
    String::from_utf8_lossy(&output.stdout)
        .lines()
        .filter(|service| !service.is_empty() && !RUNS_FOREVER.contains(service))
        .count()
        == 0
}

/// Connects to the real dashboard's `/ws` and folds every `FleetSnapshot` push into
/// `(robot_id -> tick -> position)`, until either the compose stack's workers finish or
/// `OVERALL_DEADLINE` is hit (whichever first — the latter is just a safety net against
/// a stack that never converges, not the expected exit path).
async fn collect_positions(compose_dir: PathBuf) -> HashMap<u32, HashMap<u64, (i32, i32)>> {
    let (ws_stream, _) = tokio_tungstenite::connect_async(DASHBOARD_WS_URL)
        .await
        .expect("dashboard's /ws should be reachable on the published host port");
    let (_write, mut read) = ws_stream.split();

    let mut per_robot: HashMap<u32, HashMap<u64, (i32, i32)>> = HashMap::new();
    let start = tokio::time::Instant::now();
    let mut last_poll = tokio::time::Instant::now();

    loop {
        if start.elapsed() > OVERALL_DEADLINE {
            eprintln!("[e2e] hit the overall deadline before compose reported finished");
            break;
        }

        match tokio::time::timeout(Duration::from_secs(5), read.next()).await {
            Ok(Some(Ok(tokio_tungstenite::tungstenite::Message::Text(text)))) => {
                if let Ok(snapshot) = serde_json::from_str::<FleetSnapshotView>(&text) {
                    for (id_str, robot) in snapshot.robots {
                        if let (Ok(id), Some(position), Some(tick)) =
                            (id_str.parse::<u32>(), robot.position, robot.tick)
                        {
                            per_robot.entry(id).or_default().insert(tick, position);
                        }
                    }
                }
            }
            Ok(Some(Ok(_))) => {}
            Ok(Some(Err(_))) | Ok(None) => break,
            Err(_) => {} // read timeout — fall through to the compose-status poll below
        }

        if last_poll.elapsed() > COMPOSE_POLL_INTERVAL {
            last_poll = tokio::time::Instant::now();
            if compose_workers_finished(&compose_dir) {
                // A short grace read for any last in-flight snapshot.
                let _ = tokio::time::timeout(Duration::from_secs(2), read.next()).await;
                break;
            }
        }
    }

    per_robot
}

fn write_metrics_dump(manifest_dir: &Path, complete_ticks: usize, runs: &[Vec<Config>], collisions_found: bool) {
    let results_dir = manifest_dir.join("results");
    let _ = std::fs::create_dir_all(&results_dir);
    let dump = serde_json::json!({
        "note": "test-mode dump from tests/no_collisions_e2e.rs, not harness/metrics_schema.json's full cross-system shape — see this file's own doc comment for why.",
        "ticks_observed_complete": complete_ticks,
        "consecutive_runs": runs.len(),
        "longest_run_ticks": runs.iter().map(Vec::len).max().unwrap_or(0),
        "vertex_or_edge_collisions_found": collisions_found,
    });
    if let Ok(json) = serde_json::to_string_pretty(&dump) {
        let _ = std::fs::write(results_dir.join("no_collisions_e2e_last_run.json"), json);
    }
}

#[test]
#[ignore = "needs Docker; run explicitly: cargo test --test no_collisions_e2e -- --ignored --nocapture"]
fn full_stack_docker_compose_run_has_zero_collisions() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));

    let status = Command::new("docker")
        .args(["compose", "--profile", "smoke", "up", "--build", "-d"])
        .current_dir(&manifest_dir)
        .status()
        .expect("`docker compose` should be installed and runnable");
    // Guard first: a failed or interrupted `up` can still leave containers behind, and
    // they'd break the next run's `up` (name/port conflicts) if not torn down.
    let _guard = ComposeDownGuard { dir: manifest_dir.clone() };
    assert!(status.success(), "docker compose up failed to bring the stack up");

    // The dashboard needs a moment to bind its HTTP port before a client can connect.
    std::thread::sleep(Duration::from_secs(2));

    let runtime = tokio::runtime::Runtime::new().expect("tokio runtime for the ws client");
    let per_robot = runtime.block_on(collect_positions(manifest_dir.clone()));

    let mut complete_ticks: Vec<u64> = per_robot
        .get(&ROBOT_IDS[0])
        .map(|by_tick| by_tick.keys().copied().collect())
        .unwrap_or_default();
    complete_ticks.retain(|tick| {
        ROBOT_IDS
            .iter()
            .all(|id| per_robot.get(id).is_some_and(|by_tick| by_tick.contains_key(tick)))
    });
    complete_ticks.sort_unstable();

    assert!(
        complete_ticks.len() >= MIN_COMPLETE_TICKS,
        "expected at least {MIN_COMPLETE_TICKS} ticks where every robot reported a position, got {} \
         (per-robot tick counts observed: {:?})",
        complete_ticks.len(),
        ROBOT_IDS
            .iter()
            .map(|id| (id, per_robot.get(id).map_or(0, HashMap::len)))
            .collect::<Vec<_>>()
    );

    let mut collision_found = false;
    let mut runs: Vec<Vec<Config>> = Vec::new();
    let mut current_run: Vec<Config> = Vec::new();
    let mut prev_tick: Option<u64> = None;

    for &tick in &complete_ticks {
        let config: Config = ROBOT_IDS
            .iter()
            .map(|id| {
                let (x, y) = per_robot[id][&tick];
                (x as usize, y as usize)
            })
            .collect();

        // Vertex collisions are meaningful for any single tick on its own, consecutive
        // or not — checked immediately regardless of how the run-grouping below goes.
        if !has_no_collisions(std::slice::from_ref(&config)) {
            collision_found = true;
        }

        match prev_tick {
            Some(p) if tick == p + 1 => current_run.push(config),
            _ => {
                if current_run.len() > 1 {
                    runs.push(std::mem::take(&mut current_run));
                } else {
                    current_run.clear();
                }
                current_run.push(config);
            }
        }
        prev_tick = Some(tick);
    }
    if current_run.len() > 1 {
        runs.push(current_run);
    }

    // Edge (swap) collisions only make sense across genuinely consecutive ticks —
    // `has_no_collisions` assumes adjacency between `configs[t-1]` and `configs[t]`.
    for run in &runs {
        if !has_no_collisions(run) {
            collision_found = true;
        }
    }

    write_metrics_dump(&manifest_dir, complete_ticks.len(), &runs, collision_found);

    assert!(
        !collision_found,
        "a real docker-compose run produced a vertex or edge collision — see \
         results/no_collisions_e2e_last_run.json"
    );
}
