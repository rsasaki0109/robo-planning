//! Planar serial arms: kinematics, inverse kinematics and a joint-space
//! [`StateSpace`](crate::sampling::StateSpace) with link-vs-obstacle collision checks.

use crate::geometry::{point_segment_distance, Map2D, Obstacle, Vec2};
use crate::rng::Rng;
use crate::sampling::StateSpace;

/// Planar revolute arm with its base at `base`.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanarArm {
    /// base position
    pub base: Vec2,
    /// link lengths [m]
    pub link_lengths_m: Vec<f64>,
    /// joint lower limits [rad]
    pub q_min_rad: Vec<f64>,
    /// joint upper limits [rad]
    pub q_max_rad: Vec<f64>,
}

impl PlanarArm {
    /// Arm with symmetric joint limits of `±limit_rad`.
    pub fn new(base: Vec2, link_lengths_m: Vec<f64>, limit_rad: f64) -> Self {
        let n = link_lengths_m.len();
        Self {
            base,
            link_lengths_m,
            q_min_rad: vec![-limit_rad; n],
            q_max_rad: vec![limit_rad; n],
        }
    }
    /// Degrees of freedom.
    pub fn dof(&self) -> usize {
        self.link_lengths_m.len()
    }
    /// Joint positions from the base to the end effector (`dof + 1` points).
    pub fn fk(&self, q: &[f64]) -> Vec<Vec2> {
        let mut pts = vec![self.base];
        let mut a = 0.0;
        let mut p = self.base;
        for (qi, l) in q.iter().zip(&self.link_lengths_m) {
            a += qi;
            p = p + Vec2::new(a.cos(), a.sin()) * *l;
            pts.push(p);
        }
        pts
    }
    /// End-effector position.
    pub fn end_effector(&self, q: &[f64]) -> Vec2 {
        *self.fk(q).last().unwrap()
    }
    /// Position Jacobian (2 x dof), column-major as `[dx/dq_i, dy/dq_i]`.
    pub fn jacobian(&self, q: &[f64]) -> Vec<[f64; 2]> {
        let pts = self.fk(q);
        let ee = *pts.last().unwrap();
        (0..self.dof()).map(|i| [-(ee.y - pts[i].y), ee.x - pts[i].x]).collect()
    }
    /// Damped-least-squares IK for the end-effector position from seed `q0`.
    /// Returns `None` if it does not converge within `tol_m`.
    pub fn ik(&self, target: Vec2, q0: &[f64], tol_m: f64) -> Option<Vec<f64>> {
        let mut q = q0.to_vec();
        let lambda2 = 1e-3;
        for _ in 0..500 {
            let e = target - self.end_effector(&q);
            if e.norm() < tol_m {
                return Some(q);
            }
            let j = self.jacobian(&q);
            // JJ^T + λ²I (2x2)
            let (mut a, mut b, mut d) = (lambda2, 0.0, lambda2);
            for c in &j {
                a += c[0] * c[0];
                b += c[0] * c[1];
                d += c[1] * c[1];
            }
            let det = a * d - b * b;
            let (wx, wy) = ((d * e.x - b * e.y) / det, (-b * e.x + a * e.y) / det);
            for (i, c) in j.iter().enumerate() {
                q[i] = (q[i] + 0.5 * (c[0] * wx + c[1] * wy)).clamp(self.q_min_rad[i], self.q_max_rad[i]);
            }
        }
        None
    }
}

/// Joint space of a [`PlanarArm`] among obstacles.
#[derive(Debug, Clone)]
pub struct ArmSpace<'a> {
    /// arm
    pub arm: &'a PlanarArm,
    /// workspace obstacles
    pub map: &'a Map2D,
    /// link half-thickness + margin [m]
    pub link_margin_m: f64,
}

impl ArmSpace<'_> {
    /// Minimum distance between any link segment and any obstacle.
    pub fn clearance(&self, q: &[f64]) -> f64 {
        let pts = self.arm.fk(q);
        let mut best = f64::INFINITY;
        for w in pts.windows(2) {
            for o in &self.map.obstacles {
                let d = match *o {
                    Obstacle::Circle { center, radius_m } => point_segment_distance(center, w[0], w[1]) - radius_m,
                    Obstacle::Rect { .. } => {
                        let n = ((w[0].dist(w[1]) / 0.02).ceil() as usize).max(1);
                        (0..=n)
                            .map(|k| o.distance(w[0].lerp(w[1], k as f64 / n as f64)))
                            .fold(f64::INFINITY, f64::min)
                    }
                };
                best = best.min(d);
            }
        }
        best
    }
}

impl StateSpace for ArmSpace<'_> {
    type State = Vec<f64>;
    fn sample(&self, rng: &mut Rng) -> Vec<f64> {
        (0..self.arm.dof())
            .map(|i| rng.range(self.arm.q_min_rad[i], self.arm.q_max_rad[i]))
            .collect()
    }
    fn distance(&self, a: &Vec<f64>, b: &Vec<f64>) -> f64 {
        a.iter().zip(b).map(|(x, y)| (x - y) * (x - y)).sum::<f64>().sqrt()
    }
    fn interpolate(&self, a: &Vec<f64>, b: &Vec<f64>, t: f64) -> Vec<f64> {
        a.iter().zip(b).map(|(x, y)| x + (y - x) * t).collect()
    }
    fn is_valid(&self, q: &Vec<f64>) -> bool {
        q.iter()
            .enumerate()
            .all(|(i, v)| *v >= self.arm.q_min_rad[i] && *v <= self.arm.q_max_rad[i])
            && self.clearance(q) > self.link_margin_m
    }
    fn resolution(&self) -> f64 {
        0.02
    }
}

impl ArmSpace<'_> {
    /// IK that also requires a collision-free configuration: tries `n_seeds`
    /// deterministic random seeds and returns the valid solution with the largest clearance.
    pub fn ik_valid(&self, target: Vec2, n_seeds: usize, seed: u64) -> Option<Vec<f64>> {
        let mut rng = Rng::new(seed);
        let mut best: Option<(f64, Vec<f64>)> = None;
        for _ in 0..n_seeds {
            let q0 = self.sample(&mut rng);
            if let Some(q) = self.arm.ik(target, &q0, 1e-4) {
                if self.is_valid(&q) {
                    let c = self.clearance(&q);
                    if best.as_ref().is_none_or(|(bc, _)| c > *bc) {
                        best = Some((c, q));
                    }
                }
            }
        }
        best.map(|(_, q)| q)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sampling::{rrt_connect, SamplingConfig};

    #[test]
    fn ik_reaches_target() {
        let arm = PlanarArm::new(Vec2::new(0.0, 0.0), vec![1.0, 0.8, 0.5], 2.8);
        let q = arm.ik(Vec2::new(1.2, 1.0), &[0.1, 0.1, 0.1], 1e-6).unwrap();
        assert!(arm.end_effector(&q).dist(Vec2::new(1.2, 1.0)) < 1e-6);
    }

    #[test]
    fn joint_space_plan_avoids_obstacle() {
        let arm = PlanarArm::new(Vec2::new(0.0, 0.0), vec![1.0, 0.8, 0.5], 2.8);
        let map = Map2D::new(Vec2::new(-3.0, -3.0), Vec2::new(3.0, 3.0)).circle(1.3, 0.3, 0.25);
        let sp = ArmSpace {
            arm: &arm,
            map: &map,
            link_margin_m: 0.05,
        };
        let s = vec![-0.6, 0.3, 0.2];
        let g = vec![0.9, -0.4, -0.3];
        assert!(sp.is_valid(&s) && sp.is_valid(&g));
        let r = rrt_connect(
            &sp,
            &s,
            &g,
            &SamplingConfig {
                step: 0.2,
                ..Default::default()
            },
        );
        assert!(r.found());
        assert!(r.path.windows(2).all(|w| sp.motion_valid(&w[0], &w[1])));
    }
}
