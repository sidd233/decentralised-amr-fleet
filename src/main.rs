//! Binary entry point, items 21 and 23 of `docs/BUILD_PLAN.md`. `dashboard`/`bench` land
//! at items 24/27-30, each adding its own `Command` variant when its module exists to
//! call into.
//!
//! Since item 23, `robot` needs a `clock` process running somewhere on the same
//! multicast bus to make progress at all: `RobotProcess::run` paces off
//! `clock_driver.rs`'s tick broadcasts now, not self-paced sleeping
//! (`docs/decisions.md` Decision 8) — a `robot` started with no `clock` running will
//! just wait and periodically log that it hasn't heard one, exactly as intended.

use std::thread;

use clap::{Parser, Subcommand};

use sih26123::clock_driver::ClockDriver;
use sih26123::config::{BOOTSTRAP_TOKEN_ID, DASHBOARD_HTTP_PORT, TICK_INTERVAL_MS};
use sih26123::dashboard::server::DashboardServer;
use sih26123::protocol::messages::TokenMsg;
use sih26123::robot::comms::Comms;
use sih26123::robot::planner_pibt::Cell;
use sih26123::robot::task_layer::{Task, TOKEN_RANGE_CELLS, TOKEN_RETRY_INTERVAL};
use sih26123::robot::RobotProcess;
use sih26123::world::grid::Grid;

#[derive(Parser)]
#[command(name = "sih26123")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Run one robot as its own OS process. Every robot in a fleet gets its own
    /// invocation, sharing the same `--map`, `--peers`, and `--task` list — matching
    /// `task_layer.rs`'s assumption that the task pool and ring order are common
    /// knowledge, not negotiated at runtime. Needs a `clock` process running somewhere
    /// on the same multicast bus to make progress.
    Robot(RobotArgs),
    /// Run the clock driver: broadcasts a shared tick number over the multicast bus, no
    /// decisions of its own (`docs/PS_AND_ARCHITECTURE.md` §3.1). Exactly one instance
    /// per fleet — every `robot` process on the same bus paces off this same broadcast.
    Clock(ClockArgs),
    /// Originate the fleet's very first task-passing token (Decision 13,
    /// `docs/decisions.md`) and exit. No robot can bootstrap its own ring by sending to
    /// itself (`Comms::recv_filtered` drops a robot's own loopback by design), so this
    /// short-lived, out-of-ring process does it once instead. Run exactly once per fleet,
    /// any time after every `robot` in `--peers` has started listening.
    BootstrapToken(BootstrapTokenArgs),
    /// Run the dashboard: an `axum` server that listens on the multicast bus and serves
    /// both the built React frontend (Decision 11) and a `/ws` WebSocket of live fleet
    /// state. Read-only unless `--manage-fleet` (Decision 18) turns on the operator console.
    Dashboard(DashboardArgs),
}

#[derive(clap::Args)]
struct ClockArgs {
    /// Milliseconds between tick broadcasts.
    #[arg(long, default_value_t = TICK_INTERVAL_MS)]
    tick_interval_ms: u64,
    /// Stop after broadcasting this many ticks; runs forever if omitted.
    #[arg(long)]
    max_ticks: Option<u64>,
}

#[derive(clap::Args)]
struct RobotArgs {
    /// This robot's unique id within the fleet.
    #[arg(long)]
    id: u32,
    /// Path to the MovingAI `.map` file every robot in the fleet shares.
    #[arg(long)]
    map: String,
    /// This robot's starting cell, "x,y".
    #[arg(long)]
    start: String,
    /// Every robot id in the fleet's token-passing ring, comma-separated (e.g.
    /// "1,2,3") — must include this robot's own `--id` (`TaskLayer::new` asserts this).
    #[arg(long, value_delimiter = ',')]
    peers: Vec<u32>,
    /// One task, "pickup_x,pickup_y,dropoff_x,dropoff_y". Repeatable; task ids are
    /// assigned by position in this list (1-based). Pass the identical list to every
    /// robot in the fleet.
    #[arg(long = "task")]
    tasks: Vec<String>,
    /// This robot's own RNG seed, for PIBT's randomized candidate tie-breaking
    /// (`DistributedAgent::new`) — give every robot in a fleet a distinct seed for a
    /// reproducible-but-not-degenerate run.
    #[arg(long, default_value_t = 0)]
    seed: u64,
    /// Stop after this many ticks; runs forever if omitted (the normal case for a real
    /// deployment — a benchmark harness, item 28, is what will set this).
    #[arg(long)]
    max_ticks: Option<u64>,
}

#[derive(clap::Args)]
struct BootstrapTokenArgs {
    /// The fleet's token-passing ring, comma-separated, identical to the `--peers` list
    /// given to every `robot` in the fleet. The very first token is sent to the first id
    /// in this list.
    #[arg(long, value_delimiter = ',')]
    peers: Vec<u32>,
    /// How many times to re-broadcast the initial token before exiting — best-effort
    /// delivery (Decision 10's "log and continue" philosophy), not an acknowledged
    /// handshake: nothing else on the bus is listening for *this* process specifically.
    #[arg(long, default_value_t = 10)]
    resends: u32,
}

#[derive(clap::Args)]
struct DashboardArgs {
    /// HTTP/WebSocket port to serve on.
    #[arg(long, default_value_t = DASHBOARD_HTTP_PORT)]
    http_port: u16,
    /// Path to the built React frontend (Vite's `npm run build` output, Decision 11) —
    /// everything not matched by `/ws` is served from here as static files.
    #[arg(long, default_value = "frontend/dist")]
    frontend_dir: String,
    /// Operator console (Decision 18): let the dashboard launch robot processes and send
    /// operator commands (new jobs, retargets, blocked cells) into the fleet.
    #[arg(long)]
    manage_fleet: bool,
    /// Map the launched robots use (and the console validates clicks against).
    #[arg(long, default_value = "maps/warehouse-10-20-10-2-1.map")]
    map: String,
    /// With `--manage-fleet`: also start a clock process with each scenario (standalone
    /// runs; leave off when a separate `clock` is already on the bus, e.g. compose).
    #[arg(long)]
    spawn_clock: bool,
}

fn parse_cell(s: &str) -> Result<Cell, String> {
    let mut parts = s.split(',');
    let x = parts
        .next()
        .ok_or_else(|| format!("missing x in cell \"{s}\""))?
        .trim()
        .parse::<usize>()
        .map_err(|e| format!("bad x in cell \"{s}\": {e}"))?;
    let y = parts
        .next()
        .ok_or_else(|| format!("missing y in cell \"{s}\""))?
        .trim()
        .parse::<usize>()
        .map_err(|e| format!("bad y in cell \"{s}\": {e}"))?;
    if parts.next().is_some() {
        return Err(format!("too many components in cell \"{s}\""));
    }
    Ok((x, y))
}

fn parse_task(task_id: u32, s: &str) -> Result<Task, String> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 4 {
        return Err(format!(
            "task \"{s}\" must have exactly 4 comma-separated values: pickup_x,pickup_y,dropoff_x,dropoff_y"
        ));
    }
    let pickup = parse_cell(&format!("{},{}", parts[0], parts[1]))?;
    let dropoff = parse_cell(&format!("{},{}", parts[2], parts[3]))?;
    Ok(Task {
        task_id,
        pickup,
        dropoff,
    })
}

fn run_robot(args: RobotArgs) {
    let grid = Grid::load(&args.map).unwrap_or_else(|e| {
        eprintln!("failed to load map \"{}\": {e}", args.map);
        std::process::exit(1);
    });
    let start = parse_cell(&args.start).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(1);
    });
    let tasks: Vec<Task> = args
        .tasks
        .iter()
        .enumerate()
        .map(|(i, s)| {
            parse_task((i + 1) as u32, s).unwrap_or_else(|e| {
                eprintln!("{e}");
                std::process::exit(1);
            })
        })
        .collect();
    let task_pool_size = tasks.len();

    let mut robot = RobotProcess::new(args.id, &grid, start, tasks, args.peers, args.seed)
        .unwrap_or_else(|e| {
            eprintln!("robot {} failed to start: {e}", args.id);
            std::process::exit(1);
        });

    println!(
        "[robot {}] starting at {:?}, {task_pool_size} task(s) in the shared pool",
        args.id, start
    );

    loop {
        robot.synced_tick();
        println!(
            "[robot {}] tick={} pos={:?} mode={:?} battery={:.2}% task={:?}",
            args.id,
            robot.tick_count(),
            robot.position(),
            robot.mode(),
            robot.battery_pct(),
            robot.task_state()
        );
        if let Some(max) = args.max_ticks {
            if robot.ticks_run() >= max {
                break;
            }
        }
    }
}

fn run_bootstrap_token(args: BootstrapTokenArgs) {
    let holder_id = *args.peers.first().unwrap_or_else(|| {
        eprintln!("--peers must name at least one robot to hand the first token to");
        std::process::exit(1);
    });

    let comms = Comms::with_range(BOOTSTRAP_TOKEN_ID, (0, 0), TOKEN_RANGE_CELLS)
        .unwrap_or_else(|e| {
            eprintln!("bootstrap-token failed to bind: {e}");
            std::process::exit(1);
        });
    let token = TokenMsg {
        seq: 0,
        holder_id,
        claimed_tasks: vec![],
        epoch: 0,
        creator: 0,
    };

    for _ in 0..args.resends {
        if let Err(e) = comms.send_token(token.clone()) {
            eprintln!("[bootstrap-token] send failed, will retry: {e}");
        }
        thread::sleep(TOKEN_RETRY_INTERVAL);
    }
    println!("[bootstrap-token] sent the initial token to robot {holder_id}");
}

fn run_dashboard(args: DashboardArgs) {
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap_or_else(|e| {
            eprintln!("dashboard failed to start its async runtime: {e}");
            std::process::exit(1);
        });

    println!(
        "[dashboard] serving frontend from \"{}\" on http://0.0.0.0:{}, ws at /ws",
        args.frontend_dir, args.http_port
    );
    let launcher = args.manage_fleet.then(|| {
        let grid = Grid::load(&args.map).unwrap_or_else(|e| {
            eprintln!("dashboard --manage-fleet couldn't load map \"{}\": {e}", args.map);
            std::process::exit(1);
        });
        sih26123::dashboard::server::LauncherConfig {
            map_path: args.map.clone(),
            spawn_clock: args.spawn_clock,
            grid,
        }
    });
    let server = DashboardServer::new(&args.frontend_dir, launcher);
    if let Err(e) = runtime.block_on(server.serve(args.http_port)) {
        eprintln!("dashboard server failed: {e}");
        std::process::exit(1);
    }
}

fn run_clock(args: ClockArgs) {
    let driver = ClockDriver::new(args.tick_interval_ms).unwrap_or_else(|e| {
        eprintln!("clock driver failed to start: {e}");
        std::process::exit(1);
    });
    println!("[clock] broadcasting every {}ms", args.tick_interval_ms);
    driver.run(args.max_ticks);
}

fn main() {
    let cli = Cli::parse();
    match cli.command {
        Command::Robot(args) => run_robot(args),
        Command::Clock(args) => run_clock(args),
        Command::BootstrapToken(args) => run_bootstrap_token(args),
        Command::Dashboard(args) => run_dashboard(args),
    }
}
