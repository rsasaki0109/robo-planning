//! Coverage path planning (cleaning, mowing, inspection): boustrophedon sweeps
//! over the free space of an occupancy grid, linked by A* transfers.

use crate::geometry::{Map2D, Vec2};
use crate::grid::{astar, OccupancyGrid};

/// A coverage plan.
#[derive(Debug, Clone, PartialEq)]
pub struct CoveragePlan {
    /// full path, sweeps and transfers in order
    pub path: Vec<Vec2>,
    /// per path point: `true` when it belongs to a sweep (tool working), `false` on a transfer
    pub sweeping: Vec<bool>,
    /// number of sweep segments
    pub sweeps: usize,
}

/// Plans back-and-forth sweeps spaced `tool_width_m` apart over all free space
/// reachable by a robot of radius `robot_radius_m`, starting at `start`.
/// Sweeps run along y; segments are visited greedily by nearest end point.
pub fn boustrophedon(map: &Map2D, start: Vec2, robot_radius_m: f64, tool_width_m: f64) -> CoveragePlan {
    let res = (tool_width_m / 4.0).min(0.1);
    let grid = OccupancyGrid::from_map(map, res, robot_radius_m);
    // free vertical intervals along each sweep column
    let mut segs: Vec<(Vec2, Vec2)> = Vec::new();
    let mut x = map.min.x + robot_radius_m + 1e-3;
    while x < map.max.x - robot_radius_m {
        let mut run_start: Option<Vec2> = None;
        let mut y = map.min.y;
        let mut last_free = None;
        while y <= map.max.y {
            let p = Vec2::new(x, y);
            let free = grid
                .cell_of(p)
                .is_some_and(|(ix, iy)| !grid.is_blocked(ix as isize, iy as isize));
            if free {
                if run_start.is_none() {
                    run_start = Some(p);
                }
                last_free = Some(p);
            } else if let (Some(s), Some(e)) = (run_start.take(), last_free) {
                if e.y - s.y > res {
                    segs.push((s, e));
                }
            }
            y += res;
        }
        if let (Some(s), Some(e)) = (run_start, last_free) {
            if e.y - s.y > res {
                segs.push((s, e));
            }
        }
        x += tool_width_m;
    }
    let mut path = vec![start];
    let mut sweeping = vec![false];
    let mut used = vec![false; segs.len()];
    let mut cur = start;
    let mut sweeps = 0;
    for _ in 0..segs.len() {
        // nearest segment end (either direction), preferring short transfers
        let mut best: Option<(f64, usize, bool)> = None;
        for (i, (a, b)) in segs.iter().enumerate() {
            if used[i] {
                continue;
            }
            for (rev, p) in [(false, a), (true, b)] {
                let d = cur.dist(*p);
                if best.is_none_or(|(bd, _, _)| d < bd) {
                    best = Some((d, i, rev));
                }
            }
        }
        let Some((_, i, rev)) = best else { break };
        used[i] = true;
        let (a, b) = if rev { (segs[i].1, segs[i].0) } else { segs[i] };
        if cur.dist(a) > 1e-9 {
            let transfer = if map.segment_free(cur, a, robot_radius_m, 0.05) {
                vec![cur, a]
            } else if let Some(gp) = astar(&grid, cur, a) {
                gp.points
            } else {
                continue;
            };
            for p in transfer.into_iter().skip(1) {
                path.push(p);
                sweeping.push(false);
            }
        }
        path.push(b);
        sweeping.push(true);
        sweeps += 1;
        cur = b;
    }
    CoveragePlan { path, sweeping, sweeps }
}

/// Fraction of free cells (at `res_m`) within `tool_width_m / 2` of a sweeping part of the path.
pub fn coverage_ratio(map: &Map2D, plan: &CoveragePlan, robot_radius_m: f64, tool_width_m: f64, res_m: f64) -> f64 {
    let grid = OccupancyGrid::from_map(map, res_m, robot_radius_m);
    let (mut free, mut hit) = (0usize, 0usize);
    for iy in 0..grid.height {
        for ix in 0..grid.width {
            if grid.is_blocked(ix as isize, iy as isize) {
                continue;
            }
            free += 1;
            let c = grid.cell_center(ix, iy);
            let covered = plan.path.windows(2).zip(plan.sweeping.iter().skip(1)).any(|(w, &sw)| {
                sw && crate::geometry::point_segment_distance(c, w[0], w[1]) <= tool_width_m / 2.0 + 1e-9
            });
            if covered {
                hit += 1;
            }
        }
    }
    hit as f64 / free.max(1) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn covers_room_with_obstacle() {
        let map = Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(6.0, 4.0)).rect(2.5, 1.0, 3.5, 3.0);
        let plan = boustrophedon(&map, Vec2::new(0.5, 0.5), 0.2, 0.4);
        assert!(plan.sweeps > 10);
        assert!(plan.path.windows(2).all(|w| map.segment_free(w[0], w[1], 0.19, 0.05)));
        let r = coverage_ratio(&map, &plan, 0.2, 0.4, 0.1);
        assert!(r > 0.95, "coverage {r}");
    }
}
