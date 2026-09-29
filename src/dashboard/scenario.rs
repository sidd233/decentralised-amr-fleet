//! Operator-console bookkeeping for the dashboard (Decision 18, `docs/decisions.md`): what
//! the person at the dashboard has asked for — the fleet's start cells, the jobs injected
//! so far, the cells they've blocked — plus the validation each request must pass before
//! it is broadcast to the fleet, and the derived per-task status the UI shows.
//!
//! Pure and I/O-free, like `server.rs`'s `apply`: every method either rejects a request
//! with a human-readable reason or returns the exact wire message for `server.rs` to
//! broadcast. Nothing here plans, claims or resolves anything — that stays with the robots.

use std::collections::BTreeSet;

use serde::Serialize;

use crate::dashboard::server::FleetSnapshot;
use crate::protocol::messages::{BlockCell, TaskInject, TaskRetarget};
use crate::world::grid::Grid;

/// Most robots one scenario may have. The token ring and PIBT rounds scale with fleet
/// size (Decision 1's benchmark scope is 3-8; runs at 10 were tested).
pub const MAX_ROBOTS: usize = 10;

/// Fewest robots one scenario may have. The task-passing ring needs a peer to hand the
/// token to: with one robot a claim can never be announced, so it's released again and
/// the robot never moves (found by trying it). The problem statement asks for 3+.
pub const MIN_ROBOTS: usize = 2;

/// One job the operator asked for, as the dashboard remembers it.
#[derive(Debug, Clone, PartialEq, Eq)]
struct TaskRecord {
    task_id: u32,
    pickup: (i32, i32),
    dropoff: (i32, i32),
    cancelled: bool,
    /// Sticky: set by `observe` the first time its robot is seen at the dropoff with the
    /// job picked up. The token keeps a finished job's claim forever, so "done" can't be
    /// re-derived from the claim once the robot drives away.
    done_by: Option<u32>,
}

/// One job as shown in the UI, derived from the operator's records and the fleet's
/// latest claims.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct TaskView {
    pub task_id: u32,
    pub pickup: (i32, i32),
    pub dropoff: (i32, i32),
    /// `"pending"` (nobody claimed it yet), `"active"`, `"done"` or `"cancelled"`.
    pub status: &'static str,
    /// The robot holding the claim, once there is one.
    pub robot: Option<u32>,
}

pub struct Scenario {
    grid: Grid,
    starts: Vec<(i32, i32)>,
    tasks: Vec<TaskRecord>,
    blocked: BTreeSet<(i32, i32)>,
    next_task_id: u32,
}

impl Scenario {
    /// `starts[i]` is robot `i + 1`'s start cell. Every start must be a free map cell and
    /// no two may share one.
    pub fn new(grid: &Grid, starts: Vec<(i32, i32)>) -> Result<Self, String> {
        if starts.len() < MIN_ROBOTS {
            return Err(format!("place at least {MIN_ROBOTS} robots (the task ring needs a peer)"));
        }
        if starts.len() > MAX_ROBOTS {
            return Err(format!("at most {MAX_ROBOTS} robots"));
        }
        let mut seen = BTreeSet::new();
        for &c in &starts {
            free_cell(grid, c).map_err(|e| format!("robot start {c:?}: {e}"))?;
            if !seen.insert(c) {
                return Err(format!("two robots share start cell {c:?}"));
            }
        }
        Ok(Scenario {
            grid: grid.clone(),
            starts,
            tasks: Vec::new(),
            blocked: BTreeSet::new(),
            next_task_id: 1,
        })
    }

    pub fn starts(&self) -> &[(i32, i32)] {
        &self.starts
    }

    pub fn blocked(&self) -> Vec<(i32, i32)> {
        self.blocked.iter().copied().collect()
    }

    fn check_pair(&self, pickup: (i32, i32), dropoff: (i32, i32)) -> Result<(), String> {
        let p = free_cell(&self.grid, pickup).map_err(|e| format!("pickup {pickup:?}: {e}"))?;
        free_cell(&self.grid, dropoff).map_err(|e| format!("dropoff {dropoff:?}: {e}"))?;
        if pickup == dropoff {
            return Err("pickup and dropoff must be different cells".into());
        }
        if !self.grid.bfs_distance(p).contains_key(&(dropoff.0 as usize, dropoff.1 as usize)) {
            return Err("dropoff can't be reached from pickup (blocked or walled off)".into());
        }
        Ok(())
    }

    /// A new job. Ids count up from 1 and are never reused, so a resent command can't be
    /// mistaken for a different job.
    pub fn add_task(&mut self, pickup: (i32, i32), dropoff: (i32, i32)) -> Result<TaskInject, String> {
        self.check_pair(pickup, dropoff)?;
        let task_id = self.next_task_id;
        self.next_task_id += 1;
        self.tasks.push(TaskRecord { task_id, pickup, dropoff, cancelled: false, done_by: None });
        Ok(TaskInject { task_id, pickup, dropoff })
    }

    fn live_task(&mut self, task_id: u32) -> Result<&mut TaskRecord, String> {
        match self.tasks.iter_mut().find(|t| t.task_id == task_id) {
            Some(t) if t.done_by.is_some() => Err(format!("task {task_id} is already finished")),
            Some(t) if !t.cancelled => Ok(t),
            Some(_) => Err(format!("task {task_id} was cancelled")),
            None => Err(format!("no task {task_id}")),
        }
    }

    pub fn retarget_task(
        &mut self,
        task_id: u32,
        pickup: (i32, i32),
        dropoff: (i32, i32),
    ) -> Result<TaskRetarget, String> {
        self.live_task(task_id)?;
        self.check_pair(pickup, dropoff)?;
        let t = self.live_task(task_id)?;
        t.pickup = pickup;
        t.dropoff = dropoff;
        Ok(TaskRetarget { task_id, pickup, dropoff, cancel: false })
    }

    pub fn cancel_task(&mut self, task_id: u32) -> Result<TaskRetarget, String> {
        let t = self.live_task(task_id)?;
        t.cancelled = true;
        Ok(TaskRetarget { task_id, pickup: t.pickup, dropoff: t.dropoff, cancel: true })
    }

    /// Block or unblock one cell. `robot_cells` are where robots are right now: blocking a
    /// cell a robot is standing on would trap it inside the wall.
    pub fn set_block(
        &mut self,
        cell: (i32, i32),
        blocked: bool,
        robot_cells: &[(i32, i32)],
    ) -> Result<BlockCell, String> {
        let (Ok(x), Ok(y)) = (usize::try_from(cell.0), usize::try_from(cell.1)) else {
            return Err(format!("cell {cell:?} is outside the map"));
        };
        if !self.grid.in_bounds(x, y) {
            return Err(format!("cell {cell:?} is outside the map"));
        }
        if blocked {
            if !self.grid.is_free(x, y) && !self.blocked.contains(&cell) {
                return Err(format!("cell {cell:?} is already a wall/shelf"));
            }
            if robot_cells.contains(&cell) {
                return Err(format!("a robot is standing on {cell:?}"));
            }
            self.blocked.insert(cell);
        } else {
            if !self.blocked.contains(&cell) {
                return Err(format!("cell {cell:?} isn't blocked by the operator"));
            }
            self.blocked.remove(&cell);
        }
        self.grid.set_blocked(x, y, blocked);
        Ok(BlockCell { x: cell.0, y: cell.1, blocked })
    }

    /// Whether a job is still in play (not finished, not cancelled). Unknown ids count as
    /// live so a robot's claim on a job this scenario never issued isn't hidden.
    pub fn is_live(&self, task_id: u32) -> bool {
        self.tasks
            .iter()
            .find(|t| t.task_id == task_id)
            .map_or(true, |t| !t.cancelled && t.done_by.is_none())
    }

    /// Notices jobs that just finished: claimed by a robot, picked up, and that robot is
    /// standing on the dropoff. Call with each fleet update; `done` then stays set.
    pub fn observe(&mut self, fleet: &FleetSnapshot) {
        for t in self.tasks.iter_mut().filter(|t| t.done_by.is_none() && !t.cancelled) {
            for (id, r) in &fleet.robots {
                let picked_up = r.claims.iter().any(|c| c.task_id == t.task_id && c.picked_up);
                if picked_up && r.position == Some(t.dropoff) {
                    t.done_by = id.parse().ok();
                    break;
                }
            }
        }
    }

    /// Every job with its status. `active` = some robot holds a claim on it.
    pub fn task_views(&self, fleet: &FleetSnapshot) -> Vec<TaskView> {
        self.tasks
            .iter()
            .map(|t| {
                let claimant = || {
                    fleet
                        .robots
                        .iter()
                        .find(|(_, r)| r.claims.iter().any(|c| c.task_id == t.task_id))
                        .and_then(|(id, _)| id.parse::<u32>().ok())
                };
                let (status, robot) = if t.cancelled {
                    ("cancelled", None)
                } else if let Some(id) = t.done_by {
                    ("done", Some(id))
                } else if let Some(id) = claimant() {
                    ("active", Some(id))
                } else {
                    ("pending", None)
                };
                TaskView { task_id: t.task_id, pickup: t.pickup, dropoff: t.dropoff, status, robot }
            })
            .collect()
    }
}

fn free_cell(grid: &Grid, c: (i32, i32)) -> Result<(usize, usize), String> {
    let (Ok(x), Ok(y)) = (usize::try_from(c.0), usize::try_from(c.1)) else {
        return Err("outside the map".into());
    };
    if !grid.in_bounds(x, y) {
        return Err("outside the map".into());
    }
    if !grid.is_free(x, y) {
        return Err("not a free cell (wall, shelf or blocked)".into());
    }
    Ok((x, y))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dashboard::server::RobotSnapshot;

    fn grid() -> Grid {
        Grid::load(concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map"))
            .expect("map should load")
    }

    fn w(c: (usize, usize)) -> (i32, i32) {
        (c.0 as i32, c.1 as i32)
    }

    #[test]
    fn starts_are_validated() {
        let g = grid();
        let free = g.free_cells();
        assert!(Scenario::new(&g, vec![]).is_err());
        assert!(Scenario::new(&g, vec![w(free[0])]).is_err(), "a lone robot can't run the ring");
        assert!(Scenario::new(&g, vec![(0, 0), w(free[1])]).is_err(), "(0,0) is a wall");
        assert!(Scenario::new(&g, vec![(-1, 3)]).is_err());
        assert!(Scenario::new(&g, vec![w(free[0]), w(free[0])]).is_err(), "shared start");
        assert!(Scenario::new(&g, vec![w(free[0]), w(free[1])]).is_ok());
        assert!(Scenario::new(&g, free.iter().take(MAX_ROBOTS + 1).map(|&c| w(c)).collect()).is_err());
    }

    #[test]
    fn task_ids_count_up_and_bad_cells_are_rejected() {
        let g = grid();
        let free = g.free_cells();
        let mut s = Scenario::new(&g, vec![w(free[0]), w(free[1])]).unwrap();
        let a = s.add_task(w(free[5]), w(free[9])).unwrap();
        let b = s.add_task(w(free[6]), w(free[9])).unwrap();
        assert_eq!((a.task_id, b.task_id), (1, 2));
        assert!(s.add_task(w(free[5]), w(free[5])).is_err(), "same cell");
        assert!(s.add_task((0, 0), w(free[9])).is_err(), "wall pickup");
        assert!(s.add_task(w(free[5]), (9999, 1)).is_err(), "out of map");
    }

    #[test]
    fn retarget_and_cancel_only_touch_live_tasks() {
        let g = grid();
        let free = g.free_cells();
        let mut s = Scenario::new(&g, vec![w(free[0]), w(free[1])]).unwrap();
        let t = s.add_task(w(free[5]), w(free[9])).unwrap();
        let r = s.retarget_task(t.task_id, w(free[7]), w(free[11])).unwrap();
        assert_eq!((r.pickup, r.cancel), (w(free[7]), false));
        assert!(s.retarget_task(99, w(free[7]), w(free[11])).is_err());
        assert!(s.cancel_task(t.task_id).unwrap().cancel);
        assert!(s.cancel_task(t.task_id).is_err(), "already cancelled");
        assert!(s.retarget_task(t.task_id, w(free[7]), w(free[11])).is_err());
    }

    #[test]
    fn blocking_rejects_walls_robots_and_double_unblock() {
        let g = grid();
        let free = g.free_cells();
        let mut s = Scenario::new(&g, vec![w(free[0]), w(free[1])]).unwrap();
        assert!(s.set_block((0, 0), true, &[]).is_err(), "already a wall");
        assert!(s.set_block(w(free[3]), true, &[w(free[3])]).is_err(), "robot standing there");
        assert!(s.set_block(w(free[3]), false, &[]).is_err(), "not blocked yet");
        let b = s.set_block(w(free[3]), true, &[]).unwrap();
        assert!(b.blocked);
        assert_eq!(s.blocked(), vec![w(free[3])]);
        assert!(s.add_task(w(free[3]), w(free[9])).is_err(), "can't target a blocked cell");
        assert!(!s.set_block(w(free[3]), false, &[]).unwrap().blocked);
        assert!(s.blocked().is_empty());
    }

    #[test]
    fn task_status_follows_the_fleets_claims_and_done_is_sticky() {
        use crate::dashboard::server::ClaimInfo;
        let g = grid();
        let free = g.free_cells();
        let mut s = Scenario::new(&g, vec![w(free[0]), w(free[1])]).unwrap();
        let t = s.add_task(w(free[5]), w(free[9])).unwrap();
        let mut fleet = FleetSnapshot::default();
        assert_eq!(s.task_views(&fleet)[0].status, "pending");
        assert!(s.is_live(t.task_id));

        let mut robot = RobotSnapshot::default();
        robot.claims = vec![ClaimInfo { task_id: t.task_id, picked_up: false }];
        robot.position = Some(w(free[2]));
        fleet.robots.insert("1".into(), robot.clone());
        s.observe(&fleet);
        let v = &s.task_views(&fleet)[0];
        assert_eq!((v.status, v.robot), ("active", Some(1)));

        robot.claims = vec![ClaimInfo { task_id: t.task_id, picked_up: true }];
        robot.position = Some(w(free[9]));
        fleet.robots.insert("1".into(), robot.clone());
        s.observe(&fleet);
        assert_eq!(s.task_views(&fleet)[0].status, "done");
        assert!(!s.is_live(t.task_id));

        // The robot drives off; its old claim entry is still in the token. Still done.
        robot.position = Some(w(free[20]));
        fleet.robots.insert("1".into(), robot);
        s.observe(&fleet);
        assert_eq!(s.task_views(&fleet)[0].status, "done");
        assert!(s.cancel_task(t.task_id).is_err(), "can't cancel a finished job");

        let t2 = s.add_task(w(free[5]), w(free[9])).unwrap();
        s.cancel_task(t2.task_id).unwrap();
        assert_eq!(s.task_views(&fleet)[1].status, "cancelled");
        assert!(!s.is_live(t2.task_id));
    }
}
