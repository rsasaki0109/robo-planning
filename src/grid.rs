//! Occupancy grids and graph search (Dijkstra, A*, weighted A*).
//!
//! Fits holonomic and differential-drive robots that can turn in place: the
//! robot footprint is handled by inflating obstacles by its radius.

use crate::geometry::{Map2D, Vec2};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// Binary occupancy grid.
#[derive(Debug, Clone, PartialEq)]
pub struct OccupancyGrid {
    /// cells along x
    pub width: usize,
    /// cells along y
    pub height: usize,
    /// cell size [m]
    pub resolution_m: f64,
    /// world position of cell (0, 0)'s lower-left corner
    pub origin: Vec2,
    /// row-major occupancy, `true` = blocked
    pub occupied: Vec<bool>,
}

impl OccupancyGrid {
    /// Rasterises a map, inflating obstacles by `inflation_m` (robot radius + margin).
    pub fn from_map(map: &Map2D, resolution_m: f64, inflation_m: f64) -> Self {
        let width = ((map.max.x - map.min.x) / resolution_m).ceil() as usize;
        let height = ((map.max.y - map.min.y) / resolution_m).ceil() as usize;
        let mut g = Self {
            width,
            height,
            resolution_m,
            origin: map.min,
            occupied: vec![false; width * height],
        };
        for iy in 0..height {
            for ix in 0..width {
                let c = g.cell_center(ix, iy);
                g.occupied[iy * width + ix] = !map.is_free(c, inflation_m);
            }
        }
        g
    }
    /// World position of a cell centre.
    pub fn cell_center(&self, ix: usize, iy: usize) -> Vec2 {
        Vec2::new(
            self.origin.x + (ix as f64 + 0.5) * self.resolution_m,
            self.origin.y + (iy as f64 + 0.5) * self.resolution_m,
        )
    }
    /// Cell containing a world point, if inside the grid.
    pub fn cell_of(&self, p: Vec2) -> Option<(usize, usize)> {
        let fx = (p.x - self.origin.x) / self.resolution_m;
        let fy = (p.y - self.origin.y) / self.resolution_m;
        if fx < 0.0 || fy < 0.0 {
            return None;
        }
        let (ix, iy) = (fx as usize, fy as usize);
        (ix < self.width && iy < self.height).then_some((ix, iy))
    }
    /// Whether a cell is blocked (out of range counts as blocked).
    pub fn is_blocked(&self, ix: isize, iy: isize) -> bool {
        if ix < 0 || iy < 0 || ix as usize >= self.width || iy as usize >= self.height {
            return true;
        }
        self.occupied[iy as usize * self.width + ix as usize]
    }
}

/// Result of a grid search.
#[derive(Debug, Clone, PartialEq)]
pub struct GridPath {
    /// waypoints in world coordinates, start to goal
    pub points: Vec<Vec2>,
    /// path cost [m]
    pub cost_m: f64,
    /// number of expanded nodes
    pub expanded: usize,
}

#[derive(PartialEq)]
struct Node {
    f: f64,
    idx: usize,
}
impl Eq for Node {}
impl Ord for Node {
    fn cmp(&self, o: &Self) -> Ordering {
        o.f.partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
            .then_with(|| o.idx.cmp(&self.idx))
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

const NEIGHBOURS: [(isize, isize); 8] = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)];

/// Grid search with heuristic weight `w`: `0` is Dijkstra, `1` is A*, `>1` is weighted A*.
pub fn search(grid: &OccupancyGrid, start: Vec2, goal: Vec2, w: f64) -> Option<GridPath> {
    let (sx, sy) = grid.cell_of(start)?;
    let (gx, gy) = grid.cell_of(goal)?;
    if grid.is_blocked(sx as isize, sy as isize) || grid.is_blocked(gx as isize, gy as isize) {
        return None;
    }
    let n = grid.width * grid.height;
    let mut g = vec![f64::INFINITY; n];
    let mut parent = vec![usize::MAX; n];
    let mut closed = vec![false; n];
    let h = |ix: usize, iy: usize| {
        let dx = (ix as f64 - gx as f64).abs();
        let dy = (iy as f64 - gy as f64).abs();
        // octile distance
        (dx.max(dy) + (std::f64::consts::SQRT_2 - 1.0) * dx.min(dy)) * grid.resolution_m
    };
    let s = sy * grid.width + sx;
    let goal_idx = gy * grid.width + gx;
    g[s] = 0.0;
    let mut open = BinaryHeap::new();
    open.push(Node {
        f: w * h(sx, sy),
        idx: s,
    });
    let mut expanded = 0;
    while let Some(Node { idx, .. }) = open.pop() {
        if closed[idx] {
            continue;
        }
        closed[idx] = true;
        expanded += 1;
        if idx == goal_idx {
            break;
        }
        let (ix, iy) = ((idx % grid.width) as isize, (idx / grid.width) as isize);
        for (dx, dy) in NEIGHBOURS {
            let (nx, ny) = (ix + dx, iy + dy);
            if grid.is_blocked(nx, ny) {
                continue;
            }
            // no corner cutting
            if dx != 0 && dy != 0 && (grid.is_blocked(ix + dx, iy) || grid.is_blocked(ix, iy + dy)) {
                continue;
            }
            let ni = ny as usize * grid.width + nx as usize;
            let step = if dx != 0 && dy != 0 {
                std::f64::consts::SQRT_2
            } else {
                1.0
            } * grid.resolution_m;
            let ng = g[idx] + step;
            if ng < g[ni] {
                g[ni] = ng;
                parent[ni] = idx;
                open.push(Node {
                    f: ng + w * h(nx as usize, ny as usize),
                    idx: ni,
                });
            }
        }
    }
    if !g[goal_idx].is_finite() {
        return None;
    }
    let mut cells = vec![goal_idx];
    while *cells.last().unwrap() != s {
        cells.push(parent[*cells.last().unwrap()]);
    }
    cells.reverse();
    let mut points: Vec<Vec2> = cells
        .iter()
        .map(|&i| grid.cell_center(i % grid.width, i / grid.width))
        .collect();
    points[0] = start;
    *points.last_mut().unwrap() = goal;
    Some(GridPath {
        points,
        cost_m: g[goal_idx],
        expanded,
    })
}

/// Dijkstra (uninformed search).
pub fn dijkstra(grid: &OccupancyGrid, start: Vec2, goal: Vec2) -> Option<GridPath> {
    search(grid, start, goal, 0.0)
}

/// A* with the admissible octile heuristic.
pub fn astar(grid: &OccupancyGrid, start: Vec2, goal: Vec2) -> Option<GridPath> {
    search(grid, start, goal, 1.0)
}

/// Removes intermediate waypoints that have line of sight to each other on the map
/// (string pulling), keeping `margin_m` clearance.
pub fn prune_line_of_sight(map: &Map2D, path: &[Vec2], margin_m: f64) -> Vec<Vec2> {
    if path.len() < 3 {
        return path.to_vec();
    }
    let mut out = vec![path[0]];
    let mut i = 0;
    while i < path.len() - 1 {
        let mut j = path.len() - 1;
        while j > i + 1 && !map.segment_free(path[i], path[j], margin_m, 0.05) {
            j -= 1;
        }
        out.push(path[j]);
        i = j;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn map() -> Map2D {
        Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0)).rect(4.0, 0.0, 5.0, 8.0)
    }

    #[test]
    fn astar_matches_dijkstra_cost_and_expands_less() {
        let g = OccupancyGrid::from_map(&map(), 0.1, 0.2);
        let a = astar(&g, Vec2::new(1.0, 1.0), Vec2::new(9.0, 1.0)).unwrap();
        let d = dijkstra(&g, Vec2::new(1.0, 1.0), Vec2::new(9.0, 1.0)).unwrap();
        assert!((a.cost_m - d.cost_m).abs() < 1e-9);
        assert!(a.expanded < d.expanded);
        assert!(a.points.iter().all(|p| map().is_free(*p, 0.1)));
    }

    #[test]
    fn pruned_path_is_collision_free() {
        let g = OccupancyGrid::from_map(&map(), 0.1, 0.2);
        let a = astar(&g, Vec2::new(1.0, 1.0), Vec2::new(9.0, 1.0)).unwrap();
        let p = prune_line_of_sight(&map(), &a.points, 0.2);
        assert!(p.len() < 6);
        assert!(p.windows(2).all(|w| map().segment_free(w[0], w[1], 0.2, 0.05)));
    }
}
