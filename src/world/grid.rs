//! Static occupancy grid: map loader + BFS distance-to-goal table.
//!
//! Phase 2, item 6 of `docs/BUILD_PLAN.md`. Pure logic — no networking dependencies,
//! just `std::fs` to read the map file. This is the module `robot/planner_pibt.rs`
//! (item 9) consults every tick for its greedy "move toward goal" rule; see
//! `docs/PS_AND_ARCHITECTURE.md` §3.3 and `docs/REFERENCES.md` (Kei18/pypibt) for why
//! that rule is BFS-distance-based rather than A*/Dijkstra.

use std::collections::{HashMap, VecDeque};
use std::fs;
use std::io;
use std::path::Path;

/// A static occupancy grid loaded from a MovingAI-format `.map` file (octile format).
///
/// Cells are addressed `(x, y)` with `x` in `[0, width)`, `y` in `[0, height)`.
#[derive(Debug, Clone)]
pub struct Grid {
    width: usize,
    height: usize,
    // true = blocked, false = free
    blocked: Vec<bool>,
}

impl Grid {
    /// Loads a grid from a MovingAI octile `.map` file.
    ///
    /// Parses permissively per `docs/REFERENCES.md`: only `.` counts as free, every
    /// other character in the grid body is treated as blocked (so it doesn't matter
    /// whether a given map source uses `T`, `@`, or something else for obstacles).
    pub fn load(path: impl AsRef<Path>) -> io::Result<Grid> {
        let text = fs::read_to_string(path)?;
        let mut lines = text.lines();

        let mut height = None;
        let mut width = None;
        loop {
            let line = lines.next().ok_or_else(|| {
                io::Error::new(io::ErrorKind::InvalidData, "map file ended before body")
            })?;
            let line = line.trim();
            if line == "map" {
                break;
            }
            if let Some(v) = line.strip_prefix("height ") {
                height = Some(v.trim().parse::<usize>().map_err(|e| {
                    io::Error::new(io::ErrorKind::InvalidData, format!("bad height: {e}"))
                })?);
            } else if let Some(v) = line.strip_prefix("width ") {
                width = Some(v.trim().parse::<usize>().map_err(|e| {
                    io::Error::new(io::ErrorKind::InvalidData, format!("bad width: {e}"))
                })?);
            }
            // "type octile" and any other header line is ignored.
        }

        let height = height
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "missing 'height' header"))?;
        let width = width
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "missing 'width' header"))?;

        let mut blocked = vec![false; width * height];
        for y in 0..height {
            let row = lines.next().ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("map body has fewer than {height} rows"),
                )
            })?;
            let row: Vec<char> = row.chars().collect();
            if row.len() < width {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("row {y} shorter than declared width {width}"),
                ));
            }
            for x in 0..width {
                blocked[y * width + x] = row[x] != '.';
            }
        }

        Ok(Grid {
            width,
            height,
            blocked,
        })
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    fn index(&self, x: usize, y: usize) -> usize {
        y * self.width + x
    }

    pub fn in_bounds(&self, x: usize, y: usize) -> bool {
        x < self.width && y < self.height
    }

    pub fn is_free(&self, x: usize, y: usize) -> bool {
        self.in_bounds(x, y) && !self.blocked[self.index(x, y)]
    }

    /// Marks a cell blocked or free — e.g. when an aisle becomes impassable mid-run
    /// (`docs/BUILD_PLAN.md` item 17). Any previously computed `bfs_distance` table is
    /// stale after this call; callers must recompute it.
    pub fn set_blocked(&mut self, x: usize, y: usize, blocked: bool) {
        if self.in_bounds(x, y) {
            let idx = self.index(x, y);
            self.blocked[idx] = blocked;
        }
    }

    /// The up-to-4 in-bounds, free, 4-connected neighbors of a cell. `pub` since
    /// `robot::planner_pibt`'s candidate-generation step (item 9) needs the same
    /// neighbor logic `bfs_distance` uses internally, not just the distance table.
    pub fn neighbors(&self, x: usize, y: usize) -> Vec<(usize, usize)> {
        let mut out = Vec::with_capacity(4);
        let deltas: [(i32, i32); 4] = [(0, -1), (0, 1), (-1, 0), (1, 0)];
        for (dx, dy) in deltas {
            let nx = x as i32 + dx;
            let ny = y as i32 + dy;
            if nx >= 0 && ny >= 0 {
                let (nx, ny) = (nx as usize, ny as usize);
                if self.is_free(nx, ny) {
                    out.push((nx, ny));
                }
            }
        }
        out
    }

    /// Computes, via single-source BFS from `goal`, the shortest-path distance (in
    /// grid steps) from every free cell reachable from `goal` to `goal` itself.
    ///
    /// Every move costs exactly 1 step, so BFS is exact here (Dijkstra would degenerate
    /// to this same result at extra cost; see the discussion logged for item 6 in this
    /// session). This gives every reachable cell's distance in one pass, reused as a
    /// plain lookup every tick until the goal changes or `set_blocked` invalidates it.
    ///
    /// Cells not reachable from `goal` (e.g. isolated by a blocked aisle) are simply
    /// absent from the returned map.
    pub fn bfs_distance(&self, goal: (usize, usize)) -> HashMap<(usize, usize), u32> {
        let mut dist = HashMap::new();
        if !self.is_free(goal.0, goal.1) {
            return dist;
        }
        let mut queue = VecDeque::new();
        dist.insert(goal, 0);
        queue.push_back(goal);
        while let Some((x, y)) = queue.pop_front() {
            let d = dist[&(x, y)];
            for n in self.neighbors(x, y) {
                if !dist.contains_key(&n) {
                    dist.insert(n, d + 1);
                    queue.push_back(n);
                }
            }
        }
        dist
    }

    /// All free cells in the grid, in row-major order. Used by tests and by callers
    /// that need to enumerate every free cell (e.g. picking start/goal pairs).
    pub fn free_cells(&self) -> Vec<(usize, usize)> {
        let mut out = Vec::new();
        for y in 0..self.height {
            for x in 0..self.width {
                if self.is_free(x, y) {
                    out.push((x, y));
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Phase 2 gate (`docs/TESTING_PLAN.md`): loads the chosen map and asserts the BFS
    /// distance table is non-empty and every reachable cell has a finite distance to at
    /// least one goal cell. Decision 1 (`docs/decisions.md`) already proved the whole
    /// free-space graph is one connected component with zero articulation points, so the
    /// strong, checkable claim here is that *every* free cell (not just "reachable
    /// ones") gets a finite distance from a single arbitrary goal.
    #[test]
    fn bfs_table_covers_every_free_cell() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/maps/warehouse-10-20-10-2-1.map");
        let grid = Grid::load(path).expect("map should load");

        // Cross-check against Decision 1's numbers so a bad map file or a parser
        // regression fails loudly here rather than silently corrupting every later phase.
        assert_eq!(grid.width(), 161);
        assert_eq!(grid.height(), 63);

        let free_cells = grid.free_cells();
        assert_eq!(free_cells.len(), 5699);

        let goal = free_cells[0];
        let dist = grid.bfs_distance(goal);
        assert!(!dist.is_empty());

        for cell in &free_cells {
            assert!(
                dist.contains_key(cell),
                "cell {cell:?} has no finite distance to goal {goal:?}"
            );
        }
    }
}
