//! Sampling-based planners generic over a configuration space:
//! RRT, RRT*, RRT-Connect and PRM, plus shortcut smoothing.
//!
//! Implement [`StateSpace`] for a robot (point robot, arm joint space, ...) and
//! every planner here works for it.

use crate::rng::Rng;

/// Configuration space a sampling planner searches.
pub trait StateSpace {
    /// Configuration type.
    type State: Clone + std::fmt::Debug;
    /// Uniform random configuration (may be invalid).
    fn sample(&self, rng: &mut Rng) -> Self::State;
    /// Metric between configurations.
    fn distance(&self, a: &Self::State, b: &Self::State) -> f64;
    /// Interpolation, `t` in `[0, 1]`.
    fn interpolate(&self, a: &Self::State, b: &Self::State, t: f64) -> Self::State;
    /// Collision / limit check for one configuration.
    fn is_valid(&self, s: &Self::State) -> bool;
    /// Edge-checking resolution in metric units.
    fn resolution(&self) -> f64;
    /// Checks the straight edge between two configurations.
    fn motion_valid(&self, a: &Self::State, b: &Self::State) -> bool {
        let n = ((self.distance(a, b) / self.resolution()).ceil() as usize).max(1);
        (0..=n).all(|i| self.is_valid(&self.interpolate(a, b, i as f64 / n as f64)))
    }
}

/// Common planner settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SamplingConfig {
    /// maximum extension per step (metric units)
    pub step: f64,
    /// probability of sampling the goal
    pub goal_bias: f64,
    /// iteration budget
    pub max_iters: usize,
    /// goal tolerance (metric units)
    pub goal_tolerance: f64,
    /// RNG seed
    pub seed: u64,
}

impl Default for SamplingConfig {
    fn default() -> Self {
        Self {
            step: 0.5,
            goal_bias: 0.05,
            max_iters: 5000,
            goal_tolerance: 0.3,
            seed: 1,
        }
    }
}

/// A search tree, exposed for visualisation.
#[derive(Debug, Clone)]
pub struct Tree<S> {
    /// nodes
    pub nodes: Vec<S>,
    /// parent index per node (`usize::MAX` for the root)
    pub parent: Vec<usize>,
    /// cost-to-come per node
    pub cost: Vec<f64>,
}

impl<S: Clone> Tree<S> {
    fn new(root: S) -> Self {
        Self {
            nodes: vec![root],
            parent: vec![usize::MAX],
            cost: vec![0.0],
        }
    }
    fn path_to(&self, mut i: usize) -> Vec<S> {
        let mut p = vec![self.nodes[i].clone()];
        while self.parent[i] != usize::MAX {
            i = self.parent[i];
            p.push(self.nodes[i].clone());
        }
        p.reverse();
        p
    }
    /// Tree edges as (child, parent) pairs.
    pub fn edges(&self) -> Vec<(S, S)> {
        (1..self.nodes.len())
            .filter(|&i| self.parent[i] != usize::MAX)
            .map(|i| (self.nodes[i].clone(), self.nodes[self.parent[i]].clone()))
            .collect()
    }
}

/// Planner output.
#[derive(Debug, Clone)]
pub struct SamplingResult<S> {
    /// path from start to goal (empty when not found)
    pub path: Vec<S>,
    /// path cost in the space metric
    pub cost: f64,
    /// explored trees (one for RRT/RRT*, two for RRT-Connect, roadmap edges for PRM)
    pub trees: Vec<Tree<S>>,
    /// iterations used
    pub iterations: usize,
}

impl<S> SamplingResult<S> {
    /// Whether a path was found.
    pub fn found(&self) -> bool {
        !self.path.is_empty()
    }
}

fn nearest<P: StateSpace>(space: &P, nodes: &[P::State], q: &P::State) -> usize {
    let mut best = 0;
    let mut bd = f64::INFINITY;
    for (i, n) in nodes.iter().enumerate() {
        let d = space.distance(n, q);
        if d < bd {
            bd = d;
            best = i;
        }
    }
    best
}

fn steer<P: StateSpace>(space: &P, from: &P::State, to: &P::State, step: f64) -> P::State {
    let d = space.distance(from, to);
    if d <= step {
        to.clone()
    } else {
        space.interpolate(from, to, step / d)
    }
}

/// Path cost under the space metric.
pub fn path_cost<P: StateSpace>(space: &P, path: &[P::State]) -> f64 {
    path.windows(2).map(|w| space.distance(&w[0], &w[1])).sum()
}

/// Rapidly-exploring Random Tree.
pub fn rrt<P: StateSpace>(
    space: &P,
    start: &P::State,
    goal: &P::State,
    cfg: &SamplingConfig,
) -> SamplingResult<P::State> {
    let mut rng = Rng::new(cfg.seed);
    let mut tree = Tree::new(start.clone());
    for it in 0..cfg.max_iters {
        let q = if rng.uniform() < cfg.goal_bias {
            goal.clone()
        } else {
            space.sample(&mut rng)
        };
        let ni = nearest(space, &tree.nodes, &q);
        let new = steer(space, &tree.nodes[ni], &q, cfg.step);
        if !space.motion_valid(&tree.nodes[ni], &new) {
            continue;
        }
        let c = tree.cost[ni] + space.distance(&tree.nodes[ni], &new);
        tree.nodes.push(new.clone());
        tree.parent.push(ni);
        tree.cost.push(c);
        if space.distance(&new, goal) <= cfg.goal_tolerance && space.motion_valid(&new, goal) {
            let last = tree.nodes.len() - 1;
            let mut path = tree.path_to(last);
            path.push(goal.clone());
            let cost = path_cost(space, &path);
            return SamplingResult {
                path,
                cost,
                trees: vec![tree],
                iterations: it + 1,
            };
        }
    }
    SamplingResult {
        path: vec![],
        cost: f64::INFINITY,
        trees: vec![tree],
        iterations: cfg.max_iters,
    }
}

/// RRT* (asymptotically optimal). Runs the whole iteration budget and keeps improving.
/// `rewire_radius` bounds the neighbourhood used for parent choice and rewiring.
pub fn rrt_star<P: StateSpace>(
    space: &P,
    start: &P::State,
    goal: &P::State,
    cfg: &SamplingConfig,
    rewire_radius: f64,
) -> SamplingResult<P::State> {
    let mut rng = Rng::new(cfg.seed);
    let mut tree = Tree::new(start.clone());
    for _ in 0..cfg.max_iters {
        let q = if rng.uniform() < cfg.goal_bias {
            goal.clone()
        } else {
            space.sample(&mut rng)
        };
        let ni = nearest(space, &tree.nodes, &q);
        let new = steer(space, &tree.nodes[ni], &q, cfg.step);
        if !space.is_valid(&new) {
            continue;
        }
        let near: Vec<usize> = (0..tree.nodes.len())
            .filter(|&i| space.distance(&tree.nodes[i], &new) <= rewire_radius)
            .collect();
        let mut best_parent = usize::MAX;
        let mut best_cost = f64::INFINITY;
        for &i in near.iter().chain(std::iter::once(&ni)) {
            let c = tree.cost[i] + space.distance(&tree.nodes[i], &new);
            if c < best_cost && space.motion_valid(&tree.nodes[i], &new) {
                best_cost = c;
                best_parent = i;
            }
        }
        if best_parent == usize::MAX {
            continue;
        }
        tree.nodes.push(new.clone());
        tree.parent.push(best_parent);
        tree.cost.push(best_cost);
        let new_i = tree.nodes.len() - 1;
        for &i in &near {
            let c = best_cost + space.distance(&new, &tree.nodes[i]);
            if c + 1e-12 < tree.cost[i] && space.motion_valid(&new, &tree.nodes[i]) {
                let delta = tree.cost[i] - c;
                tree.parent[i] = new_i;
                // propagate the improvement to descendants
                let mut stack = vec![i];
                while let Some(k) = stack.pop() {
                    tree.cost[k] -= delta;
                    for (j, &p) in tree.parent.iter().enumerate() {
                        if p == k {
                            stack.push(j);
                        }
                    }
                }
            }
        }
    }
    let mut best = None;
    let mut best_cost = f64::INFINITY;
    for i in 0..tree.nodes.len() {
        let d = space.distance(&tree.nodes[i], goal);
        if d <= cfg.goal_tolerance && tree.cost[i] + d < best_cost && space.motion_valid(&tree.nodes[i], goal) {
            best_cost = tree.cost[i] + d;
            best = Some(i);
        }
    }
    match best {
        Some(i) => {
            let mut path = tree.path_to(i);
            path.push(goal.clone());
            SamplingResult {
                cost: path_cost(space, &path),
                path,
                trees: vec![tree],
                iterations: cfg.max_iters,
            }
        }
        None => SamplingResult {
            path: vec![],
            cost: f64::INFINITY,
            trees: vec![tree],
            iterations: cfg.max_iters,
        },
    }
}

/// RRT-Connect: two trees grown towards each other with greedy connection.
/// Very fast in high-dimensional spaces such as arm joint space.
pub fn rrt_connect<P: StateSpace>(
    space: &P,
    start: &P::State,
    goal: &P::State,
    cfg: &SamplingConfig,
) -> SamplingResult<P::State> {
    let mut rng = Rng::new(cfg.seed);
    let mut ta = Tree::new(start.clone());
    let mut tb = Tree::new(goal.clone());
    let mut swapped = false;
    for it in 0..cfg.max_iters {
        let q = space.sample(&mut rng);
        // extend A towards q
        let ni = nearest(space, &ta.nodes, &q);
        let new = steer(space, &ta.nodes[ni], &q, cfg.step);
        if space.motion_valid(&ta.nodes[ni], &new) {
            let c = ta.cost[ni] + space.distance(&ta.nodes[ni], &new);
            ta.nodes.push(new.clone());
            ta.parent.push(ni);
            ta.cost.push(c);
            let a_new = ta.nodes.len() - 1;
            // connect B towards new
            let mut bi = nearest(space, &tb.nodes, &new);
            loop {
                let target = steer(space, &tb.nodes[bi], &new, cfg.step);
                if !space.motion_valid(&tb.nodes[bi], &target) {
                    break;
                }
                let c = tb.cost[bi] + space.distance(&tb.nodes[bi], &target);
                tb.nodes.push(target.clone());
                tb.parent.push(bi);
                tb.cost.push(c);
                bi = tb.nodes.len() - 1;
                if space.distance(&target, &new) < 1e-9 {
                    let mut pa = ta.path_to(a_new);
                    let mut pb = tb.path_to(bi);
                    pb.reverse();
                    pa.extend(pb.into_iter().skip(1));
                    if swapped {
                        pa.reverse();
                        std::mem::swap(&mut ta, &mut tb);
                    }
                    let cost = path_cost(space, &pa);
                    return SamplingResult {
                        path: pa,
                        cost,
                        trees: vec![ta, tb],
                        iterations: it + 1,
                    };
                }
            }
        }
        std::mem::swap(&mut ta, &mut tb);
        swapped = !swapped;
    }
    if swapped {
        std::mem::swap(&mut ta, &mut tb);
    }
    SamplingResult {
        path: vec![],
        cost: f64::INFINITY,
        trees: vec![ta, tb],
        iterations: cfg.max_iters,
    }
}

/// Probabilistic Roadmap: `n_samples` valid nodes connected to neighbours within
/// `connect_radius`, then Dijkstra on the graph. Good for many queries in one map.
pub fn prm<P: StateSpace>(
    space: &P,
    start: &P::State,
    goal: &P::State,
    n_samples: usize,
    connect_radius: f64,
    seed: u64,
) -> SamplingResult<P::State> {
    let mut rng = Rng::new(seed);
    let mut nodes = vec![start.clone(), goal.clone()];
    let mut guard = 0;
    while nodes.len() < n_samples + 2 && guard < n_samples * 50 {
        guard += 1;
        let q = space.sample(&mut rng);
        if space.is_valid(&q) {
            nodes.push(q);
        }
    }
    let n = nodes.len();
    let mut adj: Vec<Vec<(usize, f64)>> = vec![vec![]; n];
    let mut roadmap = Tree {
        nodes: nodes.clone(),
        parent: vec![usize::MAX; n],
        cost: vec![0.0; n],
    };
    let mut edges_vis = Vec::new();
    for i in 0..n {
        for j in (i + 1)..n {
            let d = space.distance(&nodes[i], &nodes[j]);
            if d <= connect_radius && space.motion_valid(&nodes[i], &nodes[j]) {
                adj[i].push((j, d));
                adj[j].push((i, d));
                edges_vis.push((i, j));
            }
        }
    }
    // Dijkstra
    let mut dist = vec![f64::INFINITY; n];
    let mut par = vec![usize::MAX; n];
    let mut done = vec![false; n];
    dist[0] = 0.0;
    for _ in 0..n {
        let mut u = usize::MAX;
        let mut bd = f64::INFINITY;
        for k in 0..n {
            if !done[k] && dist[k] < bd {
                bd = dist[k];
                u = k;
            }
        }
        if u == usize::MAX || u == 1 {
            break;
        }
        done[u] = true;
        for &(v, w) in &adj[u] {
            if dist[u] + w < dist[v] {
                dist[v] = dist[u] + w;
                par[v] = u;
            }
        }
    }
    // Encode roadmap edges for visualisation as a star of 2-node trees in one Tree.
    let mut vis = Tree {
        nodes: Vec::new(),
        parent: Vec::new(),
        cost: Vec::new(),
    };
    for (i, j) in edges_vis {
        vis.nodes.push(nodes[i].clone());
        vis.parent.push(usize::MAX);
        vis.cost.push(0.0);
        vis.nodes.push(nodes[j].clone());
        vis.parent.push(vis.nodes.len() - 2);
        vis.cost.push(0.0);
    }
    roadmap.parent = par.clone();
    if !dist[1].is_finite() {
        return SamplingResult {
            path: vec![],
            cost: f64::INFINITY,
            trees: vec![vis],
            iterations: n,
        };
    }
    let mut path = vec![nodes[1].clone()];
    let mut k = 1;
    while par[k] != usize::MAX {
        k = par[k];
        path.push(nodes[k].clone());
    }
    path.reverse();
    let _ = roadmap;
    SamplingResult {
        cost: dist[1],
        path,
        trees: vec![vis],
        iterations: n,
    }
}

/// Randomised shortcut smoothing: repeatedly replaces sub-paths by straight edges when valid.
pub fn shortcut<P: StateSpace>(space: &P, path: &[P::State], iters: usize, seed: u64) -> Vec<P::State> {
    let mut p = path.to_vec();
    let mut rng = Rng::new(seed);
    for _ in 0..iters {
        if p.len() < 3 {
            break;
        }
        let i = (rng.uniform() * p.len() as f64) as usize % p.len();
        let j = (rng.uniform() * p.len() as f64) as usize % p.len();
        let (i, j) = (i.min(j), i.max(j));
        if j <= i + 1 {
            continue;
        }
        if space.motion_valid(&p[i], &p[j]) {
            p.drain(i + 1..j);
        }
    }
    p
}

/// Point robot (disc) moving in a [`crate::Map2D`].
#[derive(Debug, Clone)]
pub struct Point2Space<'a> {
    /// map
    pub map: &'a crate::Map2D,
    /// robot radius + safety margin [m]
    pub margin_m: f64,
}

impl StateSpace for Point2Space<'_> {
    type State = crate::Vec2;
    fn sample(&self, rng: &mut Rng) -> crate::Vec2 {
        crate::Vec2::new(
            rng.range(self.map.min.x, self.map.max.x),
            rng.range(self.map.min.y, self.map.max.y),
        )
    }
    fn distance(&self, a: &crate::Vec2, b: &crate::Vec2) -> f64 {
        a.dist(*b)
    }
    fn interpolate(&self, a: &crate::Vec2, b: &crate::Vec2, t: f64) -> crate::Vec2 {
        a.lerp(*b, t)
    }
    fn is_valid(&self, s: &crate::Vec2) -> bool {
        self.map.is_free(*s, self.margin_m)
    }
    fn resolution(&self) -> f64 {
        0.05
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Map2D, Vec2};

    fn map() -> Map2D {
        Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0))
            .rect(4.0, 0.0, 5.0, 7.0)
            .circle(7.0, 6.0, 1.0)
    }

    fn check(space: &Point2Space, r: &SamplingResult<Vec2>) {
        assert!(r.found());
        assert!(r.path.windows(2).all(|w| space.motion_valid(&w[0], &w[1])));
    }

    #[test]
    fn all_planners_find_valid_paths() {
        let m = map();
        let sp = Point2Space { map: &m, margin_m: 0.2 };
        let (s, g) = (Vec2::new(1.0, 1.0), Vec2::new(9.0, 1.0));
        let cfg = SamplingConfig::default();
        let a = rrt(&sp, &s, &g, &cfg);
        check(&sp, &a);
        let b = rrt_star(&sp, &s, &g, &SamplingConfig { max_iters: 2000, ..cfg }, 1.5);
        check(&sp, &b);
        assert!(b.cost <= a.cost + 1e-9);
        check(&sp, &rrt_connect(&sp, &s, &g, &cfg));
        check(&sp, &prm(&sp, &s, &g, 300, 2.0, 3));
        let sc = shortcut(&sp, &a.path, 200, 4);
        assert!(path_cost(&sp, &sc) <= a.cost + 1e-9);
    }

    #[test]
    fn deterministic() {
        let m = map();
        let sp = Point2Space { map: &m, margin_m: 0.2 };
        let cfg = SamplingConfig::default();
        let a = rrt(&sp, &Vec2::new(1.0, 1.0), &Vec2::new(9.0, 1.0), &cfg);
        let b = rrt(&sp, &Vec2::new(1.0, 1.0), &Vec2::new(9.0, 1.0), &cfg);
        assert_eq!(a.path, b.path);
    }
}
