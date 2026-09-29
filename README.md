# Decentralised AMR Fleet

Leaderless, deadlock-free coordination for warehouse autonomous mobile robots (AMRs), written
in Rust. Built for Smart India Hackathon 2026, problem statement 26123 (Edge-AI based
distributed fleet coordination).

Every robot is its own OS process. Robots share position, intent and battery over a
peer-to-peer UDP multicast bus; there is no central planner.

- **Collision-free planning:** PIBT (Priority Inheritance with Backtracking), so wait cycles
  never form.
- **Task allocation:** token passing; the robot holding the token claims the nearest reachable
  job, and jobs behind a blocked aisle return to the pool.
- **Graceful degradation:** Cooperative, then Cautious, then Autonomous (local sensing only) as
  radio quality falls.
- **Dashboard and operator console:** live positions, battery, mode and task; place robots,
  add, reassign or cancel jobs, block an aisle, and replay a run.
- **Edge-sized:** each robot runs within 1 CPU / 512 MB in Docker.

## Quick start (Docker)

```
docker compose up -d --build        # clock + dashboard (operator console)
# open http://localhost:8080, load a preset, press Start
docker compose down
```

A fixed 6-robot smoke fleet (used by the end-to-end collision test) is available with
`docker compose --profile smoke up --build`. Do not run it together with dashboard-launched
scenarios; both use robot ids 1-6.

## Build and test

```
cargo build --release
cargo test -- --test-threads=1                              # several tests share one multicast port
cargo test --test no_collisions_e2e -- --ignored --nocapture # needs Docker
```

Frontend (dev): `cd frontend && npm install && npm run dev` (proxies to a dashboard on :8080).

## Binaries

`sih26123` has subcommands `robot`, `clock`, `dashboard` (add `--manage-fleet` for the operator
console) and `bootstrap-token`. Benchmark tools live in `src/bin/`: `run_experiment`,
`baseline_stop_and_wait`, `baseline_cbs`, `degradation_sweep`. Their outputs are in `results/`.

## Layout

```
src/robot/       one robot: comms, PIBT planner, token-passing task layer, modes, battery
src/dashboard/   axum server, operator-console logic, replay history
src/protocol/    wire messages
src/world/       map loader and BFS distance tables
src/bin/         benchmark harness
frontend/        React + Vite dashboard
maps/            MovingAI warehouse-10-20-10-2-1 benchmark map
results/         generated benchmark output
tests/           integration and end-to-end tests
```

## References

- Okumura et al., PIBT, IJCAI 2019 / Artificial Intelligence 310 (2022)
- Ma, Li, Kumar, Koenig, Lifelong MAPF (token passing), AAMAS 2017
- Sharon et al., Conflict-Based Search, Artificial Intelligence 219 (2015)
- MovingAI MAPF benchmarks: https://movingai.com/benchmarks/mapf/index.html
