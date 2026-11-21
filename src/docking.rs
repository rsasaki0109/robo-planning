//! Charging / docking: choose a charger by path cost, plan to a pre-dock pose
//! in front of it, then a straight, aligned final approach.

use crate::geometry::{Map2D, Pose2, Vec2};
use crate::grid::{astar, prune_line_of_sight, OccupancyGrid};
use crate::trajectory::{densify, spline_path, PathPoint};

/// A docking plan.
#[derive(Debug, Clone, PartialEq)]
pub struct DockPlan {
    /// index of the chosen charger
    pub charger: usize,
    /// pre-dock pose (on the charger axis, `standoff_m` in front of it)
    pub pre_dock: Pose2,
    /// smooth path to the pre-dock pose followed by the straight approach
    pub path: Vec<PathPoint>,
    /// index in `path` where the final approach starts
    pub approach_start: usize,
    /// cost of every charger considered [m] (`INFINITY` if unreachable)
    pub costs_m: Vec<f64>,
}

/// Pre-dock pose for a charger facing direction `dock.theta_rad` (robot docks facing the charger).
pub fn pre_dock_pose(dock: Pose2, standoff_m: f64) -> Pose2 {
    Pose2::new(
        dock.x - standoff_m * dock.theta_rad.cos(),
        dock.y - standoff_m * dock.theta_rad.sin(),
        dock.theta_rad,
    )
}

/// Plans to the cheapest reachable charger. `docks` are the robot's final poses at each charger.
pub fn plan_dock(map: &Map2D, start: Vec2, docks: &[Pose2], robot_radius_m: f64, standoff_m: f64) -> Option<DockPlan> {
    let grid = OccupancyGrid::from_map(map, 0.1, robot_radius_m);
    let mut costs = Vec::new();
    let mut best: Option<(usize, Vec<Vec2>)> = None;
    for (i, d) in docks.iter().enumerate() {
        let pre = pre_dock_pose(*d, standoff_m);
        let c = astar(&grid, start, pre.pos()).map(|p| prune_line_of_sight(map, &p.points, robot_radius_m));
        let cost = c
            .as_ref()
            .map_or(f64::INFINITY, |p| crate::geometry::path_length(p) + standoff_m);
        if let Some(c) = c {
            if best.as_ref().is_none_or(|(b, _)| cost < costs[*b]) {
                best = Some((i, c));
            }
        }
        costs.push(cost);
    }
    let (i, mut pts) = best?;
    let dock = docks[i];
    let pre = pre_dock_pose(dock, standoff_m);
    // add a lead-in point behind the pre-dock pose so the spline arrives aligned
    let lead = Vec2::new(
        pre.x - 0.5 * standoff_m * dock.theta_rad.cos(),
        pre.y - 0.5 * standoff_m * dock.theta_rad.sin(),
    );
    let n = pts.len();
    if n >= 2
        && map.segment_free(pts[n - 2], lead, robot_radius_m, 0.05)
        && map.segment_free(lead, pre.pos(), robot_radius_m, 0.05)
    {
        pts.insert(n - 1, lead);
    }
    let mut path = spline_path(&densify(&pts, 0.6), 0.05);
    let approach_start = path.len() - 1;
    let s0 = path.last().unwrap().s_m;
    let steps = (standoff_m / 0.05).ceil() as usize;
    for k in 1..=steps {
        let t = k as f64 / steps as f64;
        path.push(PathPoint {
            s_m: s0 + standoff_m * t,
            x: pre.x + (dock.x - pre.x) * t,
            y: pre.y + (dock.y - pre.y) * t,
            yaw_rad: dock.theta_rad,
            curvature: 0.0,
        });
    }
    Some(DockPlan {
        charger: i,
        pre_dock: pre,
        path,
        approach_start,
        costs_m: costs,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn picks_reachable_nearest_charger_and_ends_aligned() {
        let map = Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0)).rect(4.0, 0.0, 5.0, 9.0);
        let docks = [
            Pose2::new(9.5, 1.0, 0.0),
            Pose2::new(0.5, 9.5, std::f64::consts::FRAC_PI_2),
        ];
        let p = plan_dock(&map, Vec2::new(2.0, 2.0), &docks, 0.3, 1.0).unwrap();
        assert_eq!(p.charger, 1);
        let end = p.path.last().unwrap();
        assert!((end.x - 0.5).abs() < 1e-9 && (end.y - 9.5).abs() < 1e-9);
        assert!((end.yaw_rad - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }
}
