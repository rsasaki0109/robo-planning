//! Hybrid A*: lattice search over continuous (x, y, heading) for car-like robots,
//! with a holonomic-with-obstacles heuristic and Dubins analytic expansion.

use crate::geometry::{normalize_angle, Map2D, Pose2, Vec2};
use crate::grid::OccupancyGrid;
use crate::{dubins, reeds_shepp};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

/// Car and search parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HybridAStarConfig {
    /// minimum turning radius [m]
    pub min_turn_radius_m: f64,
    /// collision radius of the car footprint disc(s) [m]
    pub footprint_radius_m: f64,
    /// offset of the second footprint disc ahead of the rear axle [m] (0 = single disc)
    pub footprint_front_offset_m: f64,
    /// xy cell size of the closed set [m]
    pub xy_resolution_m: f64,
    /// heading bins
    pub heading_bins: usize,
    /// arc length of one motion primitive [m]
    pub step_m: f64,
    /// number of steering samples per side (total = 2n+1)
    pub steer_samples: usize,
    /// cost multiplier for steering
    pub steer_penalty: f64,
    /// try Dubins shot to goal every n expansions
    pub analytic_every: usize,
    /// expansion budget
    pub max_expansions: usize,
    /// allow reversing (Reeds-Shepp heuristic and analytic expansion)
    pub allow_reverse: bool,
    /// cost multiplier for reversing
    pub reverse_penalty: f64,
    /// cost added at each change of direction [m]
    pub switch_penalty_m: f64,
}

impl Default for HybridAStarConfig {
    fn default() -> Self {
        Self {
            min_turn_radius_m: 3.0,
            footprint_radius_m: 1.0,
            footprint_front_offset_m: 1.5,
            xy_resolution_m: 0.5,
            heading_bins: 72,
            step_m: 1.0,
            steer_samples: 2,
            steer_penalty: 0.1,
            analytic_every: 5,
            max_expansions: 50_000,
            allow_reverse: false,
            reverse_penalty: 2.0,
            switch_penalty_m: 5.0,
        }
    }
}

/// Planner result.
#[derive(Debug, Clone, PartialEq)]
pub struct HybridAStarResult {
    /// poses (rear-axle frame) from start to goal at roughly `step_m / 5` spacing
    pub poses: Vec<Pose2>,
    /// expansion count
    pub expanded: usize,
    /// explored primitive end points (for visualisation)
    pub explored: Vec<Vec2>,
    /// driving direction at each pose: +1 forward, -1 reverse
    pub directions: Vec<f64>,
}

/// Collision check of the car footprint at a pose.
pub fn car_free(map: &Map2D, cfg: &HybridAStarConfig, p: Pose2) -> bool {
    let c0 = p.pos();
    let c1 = c0 + Vec2::new(p.theta_rad.cos(), p.theta_rad.sin()) * cfg.footprint_front_offset_m;
    map.is_free(c0, cfg.footprint_radius_m) && map.is_free(c1, cfg.footprint_radius_m)
}

#[derive(PartialEq)]
struct Open {
    f: f64,
    id: usize,
}
impl Eq for Open {}
impl Ord for Open {
    fn cmp(&self, o: &Self) -> Ordering {
        o.f.partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
            .then_with(|| o.id.cmp(&self.id))
    }
}
impl PartialOrd for Open {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

/// Holonomic distance-to-goal field over a grid (Dijkstra from the goal).
fn heuristic_field(map: &Map2D, goal: Vec2, res: f64, inflation: f64) -> (OccupancyGrid, Vec<f64>) {
    let g = OccupancyGrid::from_map(map, res, inflation);
    let mut dist = vec![f64::INFINITY; g.width * g.height];
    if let Some((gx, gy)) = g.cell_of(goal) {
        let mut heap = BinaryHeap::new();
        let gi = gy * g.width + gx;
        dist[gi] = 0.0;
        heap.push(Open { f: 0.0, id: gi });
        while let Some(Open { f, id }) = heap.pop() {
            if f > dist[id] {
                continue;
            }
            let (ix, iy) = ((id % g.width) as isize, (id / g.width) as isize);
            for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)] {
                let (nx, ny) = (ix + dx, iy + dy);
                if g.is_blocked(nx, ny) {
                    continue;
                }
                let ni = ny as usize * g.width + nx as usize;
                let nd = f + if dx != 0 && dy != 0 {
                    std::f64::consts::SQRT_2
                } else {
                    1.0
                } * res;
                if nd < dist[ni] {
                    dist[ni] = nd;
                    heap.push(Open { f: nd, id: ni });
                }
            }
        }
    }
    (g, dist)
}

/// Plans a forward-driving path from `start` to `goal`.
pub fn plan(map: &Map2D, start: Pose2, goal: Pose2, cfg: &HybridAStarConfig) -> Option<HybridAStarResult> {
    if !car_free(map, cfg, start) || !car_free(map, cfg, goal) {
        return None;
    }
    let (hgrid, hfield) = heuristic_field(map, goal.pos(), cfg.xy_resolution_m, cfg.footprint_radius_m * 0.8);
    // analytic curve to the goal: Reeds-Shepp when reversing is allowed, else Dubins
    let shot = |p: Pose2| -> Option<(f64, Vec<(Pose2, f64)>)> {
        let ds = cfg.step_m / 5.0;
        if cfg.allow_reverse {
            reeds_shepp::shortest(p, goal, cfg.min_turn_radius_m).map(|r| (r.length_m(), r.sample_many(ds)))
        } else {
            dubins::shortest(p, goal, cfg.min_turn_radius_m)
                .map(|d| (d.length_m(), d.sample_many(ds).into_iter().map(|q| (q, 1.0)).collect()))
        }
    };
    let h = |p: Pose2| -> f64 {
        let curve = if cfg.allow_reverse {
            reeds_shepp::shortest(p, goal, cfg.min_turn_radius_m).map_or(0.0, |r| r.length_m())
        } else {
            dubins::shortest(p, goal, cfg.min_turn_radius_m).map_or(0.0, |d| d.length_m())
        };
        let hol = hgrid
            .cell_of(p.pos())
            .map_or(f64::INFINITY, |(ix, iy)| hfield[iy * hgrid.width + ix]);
        curve.max(hol)
    };
    let key = |p: Pose2, dir: f64| -> (i64, i64, i64, i8) {
        let bin = 2.0 * std::f64::consts::PI / cfg.heading_bins as f64;
        (
            (p.x / cfg.xy_resolution_m).floor() as i64,
            (p.y / cfg.xy_resolution_m).floor() as i64,
            (normalize_angle(p.theta_rad) / bin).round() as i64 % cfg.heading_bins as i64,
            if cfg.allow_reverse { dir as i8 } else { 1 },
        )
    };
    let mut poses = vec![start];
    let mut dirs = vec![1.0];
    let mut parent = vec![usize::MAX];
    let mut segs: Vec<Vec<(Pose2, f64)>> = vec![vec![(start, 1.0)]];
    let mut gcost = vec![0.0];
    let mut best: HashMap<(i64, i64, i64, i8), f64> = HashMap::new();
    let mut open = BinaryHeap::new();
    open.push(Open { f: h(start), id: 0 });
    let mut expanded = 0;
    let mut explored = Vec::new();
    let substeps = 5;
    let steers: Vec<f64> = (-(cfg.steer_samples as i64)..=cfg.steer_samples as i64)
        .map(|k| k as f64 / cfg.steer_samples.max(1) as f64)
        .collect();
    let directions: &[f64] = if cfg.allow_reverse { &[1.0, -1.0] } else { &[1.0] };
    while let Some(Open { id, .. }) = open.pop() {
        expanded += 1;
        if expanded > cfg.max_expansions {
            return None;
        }
        let cur = poses[id];
        // analytic expansion
        if expanded % cfg.analytic_every == 0 || cur.pos().dist(goal.pos()) < 4.0 * cfg.min_turn_radius_m {
            if let Some((_, samples)) = shot(cur) {
                if samples.iter().all(|(p, _)| car_free(map, cfg, *p)) {
                    let mut out = samples;
                    let mut k = id;
                    while k != usize::MAX {
                        let mut s = segs[k].clone();
                        s.extend(out.into_iter().skip(1));
                        out = s;
                        k = parent[k];
                    }
                    return Some(HybridAStarResult {
                        poses: out.iter().map(|(p, _)| *p).collect(),
                        directions: out.iter().map(|(_, d)| *d).collect(),
                        expanded,
                        explored,
                    });
                }
            }
        }
        for &dir in directions {
            for &u in &steers {
                let curvature = u / cfg.min_turn_radius_m;
                let mut p = cur;
                let mut seg = vec![(cur, dir)];
                let mut ok = true;
                let ds = dir * cfg.step_m / substeps as f64;
                for _ in 0..substeps {
                    p = if curvature.abs() < 1e-9 {
                        Pose2::new(p.x + ds * p.theta_rad.cos(), p.y + ds * p.theta_rad.sin(), p.theta_rad)
                    } else {
                        let dth = ds * curvature;
                        let r = 1.0 / curvature;
                        Pose2::new(
                            p.x + r * ((p.theta_rad + dth).sin() - p.theta_rad.sin()),
                            p.y - r * ((p.theta_rad + dth).cos() - p.theta_rad.cos()),
                            normalize_angle(p.theta_rad + dth),
                        )
                    };
                    if !car_free(map, cfg, p) {
                        ok = false;
                        break;
                    }
                    seg.push((p, dir));
                }
                if !ok {
                    continue;
                }
                let mut step_cost = cfg.step_m * (1.0 + cfg.steer_penalty * u.abs());
                if dir < 0.0 {
                    step_cost *= cfg.reverse_penalty;
                }
                if id != 0 && dirs[id] != dir {
                    step_cost += cfg.switch_penalty_m;
                }
                let ng = gcost[id] + step_cost;
                let k = key(p, dir);
                if best.get(&k).is_some_and(|&c| c <= ng) {
                    continue;
                }
                best.insert(k, ng);
                let hv = h(p);
                if !hv.is_finite() {
                    continue;
                }
                poses.push(p);
                dirs.push(dir);
                parent.push(id);
                // the pose that starts this primitive drives in its direction
                let mut s = seg[1..].to_vec();
                if let Some(first) = s.first_mut() {
                    first.1 = dir;
                }
                segs.push(s);
                gcost.push(ng);
                explored.push(p.pos());
                open.push(Open {
                    f: ng + 1.5 * hv,
                    id: poses.len() - 1,
                });
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plans_around_wall() {
        let map = Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(40.0, 30.0)).rect(18.0, 0.0, 22.0, 18.0);
        let cfg = HybridAStarConfig::default();
        let r = plan(
            &map,
            Pose2::new(5.0, 5.0, 0.0),
            Pose2::new(35.0, 5.0, -std::f64::consts::FRAC_PI_2),
            &cfg,
        )
        .unwrap();
        assert!(r.poses.iter().all(|p| car_free(&map, &cfg, *p)));
        let end = *r.poses.last().unwrap();
        assert!(end.pos().dist(Vec2::new(35.0, 5.0)) < 1e-6);
        // curvature bounded by the turning radius
        for w in r.poses.windows(2) {
            let ds = w[0].pos().dist(w[1].pos());
            if ds > 1e-6 {
                assert!(
                    normalize_angle(w[1].theta_rad - w[0].theta_rad).abs() / ds <= 1.0 / cfg.min_turn_radius_m + 1e-3
                );
            }
        }
    }

    #[test]
    fn backs_into_a_parallel_slot() {
        // kerb along y = 0, cars parked front and back; the slot fits only by reversing in
        let map = Map2D::new(Vec2::new(0.0, -1.0), Vec2::new(30.0, 12.0))
            .rect(0.0, -1.0, 30.0, 0.2)
            .rect(8.0, 0.2, 12.4, 2.2)
            .rect(19.6, 0.2, 24.0, 2.2);
        let cfg = HybridAStarConfig {
            min_turn_radius_m: 4.0,
            footprint_radius_m: 0.95,
            footprint_front_offset_m: 2.0,
            allow_reverse: true,
            xy_resolution_m: 0.3,
            ..Default::default()
        };
        let start = Pose2::new(6.0, 6.0, 0.0);
        let goal = Pose2::new(14.6, 1.2, 0.0);
        let r = plan(&map, start, goal, &cfg).expect("parking plan");
        assert!(r.poses.iter().all(|p| car_free(&map, &cfg, *p)));
        assert!(r.poses.last().unwrap().pos().dist(goal.pos()) < 1e-6);
        assert!(r.directions.iter().any(|d| *d < 0.0), "never reversed");
        assert_eq!(r.directions.len(), r.poses.len());
    }
}
