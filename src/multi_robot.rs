//! Multi-robot navigation: reactive reciprocal avoidance (sampling RVO) and
//! prioritized space-time A* on a grid.
//!
//! * [`RvoSim`]: every robot picks, from a fan of candidate velocities, the one
//!   closest to its preferred velocity that keeps a safe time-to-collision
//!   against the *reciprocal* velocity obstacle of each neighbour (assuming the
//!   neighbour takes half of the avoidance effort) and against static obstacles.
//!   Decentralised and reactive; no plan is shared.
//! * [`prioritized_plan`]: robots are planned one after another on a
//!   time-expanded grid; each plan reserves its cells (and the edges it
//!   traverses) so later robots route or wait around it. Centralised, collision
//!   free on the grid by construction, incomplete in general.

use crate::geometry::{Map2D, Vec2};
use crate::grid::OccupancyGrid;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

/// Disc robot for [`RvoSim`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Agent {
    /// position
    pub pos: Vec2,
    /// velocity
    pub vel: Vec2,
    /// goal
    pub goal: Vec2,
    /// radius [m]
    pub radius_m: f64,
    /// max speed [m/s]
    pub v_max_mps: f64,
}

/// Settings of [`RvoSim`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RvoConfig {
    /// time step [s]
    pub dt_s: f64,
    /// look-ahead for collisions [s]
    pub horizon_s: f64,
    /// candidate speeds (including zero)
    pub speed_samples: usize,
    /// candidate headings
    pub heading_samples: usize,
    /// weight of a collision inside the horizon (per 1/time-to-collision)
    pub w_collision: f64,
    /// extra clearance [m]
    pub margin_m: f64,
    /// preference for passing on the right (breaks symmetric deadlocks)
    pub w_right: f64,
}

impl Default for RvoConfig {
    fn default() -> Self {
        Self {
            dt_s: 0.05,
            horizon_s: 2.5,
            speed_samples: 6,
            heading_samples: 36,
            w_collision: 1.5,
            margin_m: 0.05,
            w_right: 0.3,
        }
    }
}

/// Time until two discs at relative position `p` moving at relative velocity
/// `v` touch (combined radius `r`), or infinity.
fn time_to_collision(p: Vec2, v: Vec2, r: f64) -> f64 {
    let c = p.dot(p) - r * r;
    if c < 0.0 {
        return 0.0;
    }
    let a = v.dot(v);
    let b = p.dot(v);
    if b >= 0.0 || a < 1e-12 {
        return f64::INFINITY;
    }
    let disc = b * b - a * c;
    if disc < 0.0 {
        return f64::INFINITY;
    }
    (-b - disc.sqrt()) / a
}

/// Decentralised reciprocal-avoidance simulation.
#[derive(Debug, Clone)]
pub struct RvoSim {
    /// robots
    pub agents: Vec<Agent>,
    /// static obstacles
    pub map: Map2D,
    /// settings
    pub cfg: RvoConfig,
}

impl RvoSim {
    /// Advances all robots by one step; returns the smallest pairwise gap
    /// (distance minus radii) after the step.
    #[allow(clippy::needless_range_loop)]
    pub fn step(&mut self) -> f64 {
        let n = self.agents.len();
        let mut new_vel = vec![Vec2::default(); n];
        for i in 0..n {
            let a = self.agents[i];
            let to_goal = a.goal - a.pos;
            let dist = to_goal.norm();
            let pref = if dist < 1e-6 {
                Vec2::default()
            } else {
                to_goal * (a.v_max_mps.min(dist / self.cfg.dt_s.max(0.5)) / dist)
            };
            let mut best = (f64::INFINITY, Vec2::default());
            let mut candidates = vec![pref, Vec2::default()];
            for si in 1..=self.cfg.speed_samples {
                let s = a.v_max_mps * si as f64 / self.cfg.speed_samples as f64;
                for hi in 0..self.cfg.heading_samples {
                    let th = 2.0 * std::f64::consts::PI * hi as f64 / self.cfg.heading_samples as f64;
                    candidates.push(Vec2::new(s * th.cos(), s * th.sin()));
                }
            }
            for v in candidates {
                let mut tc = f64::INFINITY;
                for (j, b) in self.agents.iter().enumerate() {
                    if j == i {
                        continue;
                    }
                    // reciprocal velocity obstacle: assume the other robot takes half the avoidance
                    let rel_v = v * 2.0 - a.vel - b.vel;
                    tc = tc.min(time_to_collision(
                        b.pos - a.pos,
                        rel_v * -1.0,
                        a.radius_m + b.radius_m + self.cfg.margin_m,
                    ));
                }
                // static obstacles: march along the candidate
                let steps = 10;
                for k in 1..=steps {
                    let t = self.cfg.horizon_s * k as f64 / steps as f64;
                    if !self.map.is_free(a.pos + v * t, a.radius_m + self.cfg.margin_m) {
                        tc = tc.min(t);
                        break;
                    }
                }
                let penalty = if tc < 2.0 * self.cfg.dt_s {
                    // imminent contact: only if nothing else is left
                    1.0e3 + self.cfg.w_collision / tc.max(1e-3)
                } else if tc < self.cfg.horizon_s {
                    self.cfg.w_collision / tc.max(1e-3)
                } else {
                    0.0
                };
                // passing on the right: penalise veering left of the preferred direction
                let left = if pref.norm() > 1e-9 {
                    (pref.x * v.y - pref.y * v.x) / (pref.norm() * a.v_max_mps)
                } else {
                    0.0
                };
                let cost = penalty + (v - pref).norm() + self.cfg.w_right * left.max(0.0);
                if cost < best.0 {
                    best = (cost, v);
                }
            }
            new_vel[i] = best.1;
        }
        for (a, v) in self.agents.iter_mut().zip(new_vel) {
            a.vel = v;
            a.pos = a.pos + v * self.cfg.dt_s;
        }
        let mut gap = f64::INFINITY;
        for i in 0..n {
            for j in i + 1..n {
                let (a, b) = (&self.agents[i], &self.agents[j]);
                gap = gap.min(a.pos.dist(b.pos) - a.radius_m - b.radius_m);
            }
        }
        gap
    }

    /// Whether every robot is within `tol_m` of its goal.
    pub fn arrived(&self, tol_m: f64) -> bool {
        self.agents.iter().all(|a| a.pos.dist(a.goal) < tol_m)
    }
}

/// A grid edge `(from, to)` traversed starting at a time step.
type EdgeAt = ((i64, i64), (i64, i64), usize);

/// Space-time path of one robot: cell centre per time step (waits repeat a cell).
#[derive(Debug, Clone, PartialEq)]
pub struct TimedPath {
    /// positions at t = 0, dt, 2 dt, ...
    pub points: Vec<Vec2>,
}

#[derive(PartialEq)]
struct Node {
    f: f64,
    g: f64,
    cell: (i64, i64),
    t: usize,
}
impl Eq for Node {}
impl Ord for Node {
    fn cmp(&self, o: &Self) -> Ordering {
        o.f.partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
            .then_with(|| self.t.cmp(&o.t))
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

/// Prioritized planning: robots are planned in the given order with space-time
/// A* (4-connected moves plus waiting, one cell per step) while respecting the
/// cells and edge swaps reserved by earlier robots. A robot that has arrived
/// keeps its goal cell reserved. Returns `None` if some robot finds no plan
/// within `max_t` steps.
pub fn prioritized_plan(
    map: &Map2D,
    starts: &[Vec2],
    goals: &[Vec2],
    radius_m: f64,
    cell_m: f64,
    max_t: usize,
) -> Option<Vec<TimedPath>> {
    let grid = OccupancyGrid::from_map(map, cell_m, radius_m);
    let cell = |p: Vec2| grid.cell_of(p).map(|(x, y)| (x as i64, y as i64));
    let mut vertex: HashSet<((i64, i64), usize)> = HashSet::new();
    let mut edge: HashSet<EdgeAt> = HashSet::new();
    let mut parked: HashMap<(i64, i64), usize> = HashMap::new(); // goal cell -> time from which it is occupied
    let mut out = Vec::new();
    for (s, g) in starts.iter().zip(goals) {
        let (sc, gc) = (cell(*s)?, cell(*g)?);
        let h = |c: (i64, i64)| ((c.0 - gc.0).abs() + (c.1 - gc.1).abs()) as f64;
        let blocked = |c: (i64, i64), t: usize| vertex.contains(&(c, t)) || parked.get(&c).is_some_and(|&t0| t >= t0);
        let mut open = BinaryHeap::new();
        let mut parent: HashMap<((i64, i64), usize), ((i64, i64), usize)> = HashMap::new();
        let mut seen: HashSet<((i64, i64), usize)> = HashSet::new();
        open.push(Node {
            f: h(sc),
            g: 0.0,
            cell: sc,
            t: 0,
        });
        seen.insert((sc, 0));
        let mut found = None;
        // the latest time any earlier robot still moves: arriving before it may get run over
        let last_reserved = vertex.iter().map(|(_, t)| *t).max().unwrap_or(0);
        while let Some(Node { g, cell: c, t, .. }) = open.pop() {
            if c == gc && t >= last_reserved {
                found = Some((c, t));
                break;
            }
            if t >= max_t {
                continue;
            }
            for d in [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)] {
                let n = (c.0 + d.0, c.1 + d.1);
                if grid.is_blocked(n.0 as isize, n.1 as isize) || blocked(n, t + 1) || edge.contains(&(n, c, t)) {
                    continue;
                }
                if seen.insert((n, t + 1)) {
                    parent.insert((n, t + 1), (c, t));
                    let ng = g + if d == (0, 0) && n == gc { 0.0 } else { 1.0 };
                    open.push(Node {
                        f: ng + h(n),
                        g: ng,
                        cell: n,
                        t: t + 1,
                    });
                }
            }
        }
        let (mut c, mut t) = found?;
        let mut cells = vec![(c, t)];
        while let Some(&(pc, pt)) = parent.get(&(c, t)) {
            cells.push((pc, pt));
            c = pc;
            t = pt;
        }
        cells.reverse();
        for w in cells.windows(2) {
            vertex.insert(w[1]);
            edge.insert((w[0].0, w[1].0, w[0].1));
        }
        vertex.insert(cells[0]);
        let (goal_cell, arrive) = *cells.last().unwrap();
        parked.insert(goal_cell, arrive);
        out.push(TimedPath {
            points: cells
                .iter()
                .map(|(c, _)| grid.cell_center(c.0 as usize, c.1 as usize))
                .collect(),
        });
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rvo_circle_swap_around_an_obstacle_without_contact() {
        let n = 10;
        let agents = (0..n)
            .map(|i| {
                let th = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
                let p = Vec2::new(6.0 + 5.0 * th.cos(), 6.0 + 5.0 * th.sin());
                Agent {
                    pos: p,
                    vel: Vec2::default(),
                    goal: Vec2::new(12.0, 12.0) - p,
                    radius_m: 0.35,
                    v_max_mps: 1.2,
                }
            })
            .collect();
        let map = Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(12.0, 12.0)).circle(6.0, 6.0, 0.8);
        let mut sim = RvoSim {
            agents,
            map,
            cfg: RvoConfig::default(),
        };
        let mut min_gap = f64::INFINITY;
        for _ in 0..1200 {
            min_gap = min_gap.min(sim.step());
            if sim.arrived(0.1) {
                break;
            }
        }
        assert!(sim.arrived(0.1), "not all arrived");
        assert!(min_gap > 0.0, "contact: {min_gap}");
    }

    #[test]
    fn prioritized_paths_never_share_a_cell() {
        let map = Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(6.0, 3.0))
            .rect(0.0, 1.5, 2.5, 3.0)
            .rect(3.5, 1.5, 6.0, 3.0);
        // two robots swap ends of a corridor with one side pocket
        let starts = [Vec2::new(0.25, 0.75), Vec2::new(5.75, 0.75)];
        let goals = [Vec2::new(5.75, 0.75), Vec2::new(0.25, 0.75)];
        let plans = prioritized_plan(&map, &starts, &goals, 0.2, 0.5, 200).expect("plans");
        let len = plans.iter().map(|p| p.points.len()).max().unwrap();
        for t in 0..len {
            let a = plans[0].points[t.min(plans[0].points.len() - 1)];
            let b = plans[1].points[t.min(plans[1].points.len() - 1)];
            assert!(a.dist(b) > 0.1, "same cell at t={t}");
        }
        assert!(plans[0].points.last().unwrap().dist(goals[0]) < 0.3);
        assert!(plans[1].points.last().unwrap().dist(goals[1]) < 0.3);
    }
}
