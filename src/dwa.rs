//! Dynamic Window Approach: reactive local planning for differential-drive
//! (unicycle) robots with velocity and acceleration limits.

use crate::geometry::{normalize_angle, Map2D, Pose2, Vec2};

/// DWA parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DwaConfig {
    /// max forward speed [m/s]
    pub v_max_mps: f64,
    /// min speed (negative allows reversing) [m/s]
    pub v_min_mps: f64,
    /// max yaw rate [rad/s]
    pub w_max_radps: f64,
    /// linear acceleration limit [m/s^2]
    pub a_max_mps2: f64,
    /// angular acceleration limit [rad/s^2]
    pub alpha_max_radps2: f64,
    /// control period [s]
    pub dt_s: f64,
    /// rollout horizon [s]
    pub horizon_s: f64,
    /// velocity samples
    pub v_samples: usize,
    /// yaw-rate samples
    pub w_samples: usize,
    /// robot radius [m]
    pub robot_radius_m: f64,
    /// weight on heading to goal
    pub w_heading: f64,
    /// weight on clearance
    pub w_clearance: f64,
    /// weight on speed
    pub w_speed: f64,
    /// weight on remaining distance to goal
    pub w_goal: f64,
}

impl Default for DwaConfig {
    fn default() -> Self {
        Self {
            v_max_mps: 1.0,
            v_min_mps: 0.0,
            w_max_radps: 1.5,
            a_max_mps2: 1.0,
            alpha_max_radps2: 3.0,
            dt_s: 0.1,
            horizon_s: 2.0,
            v_samples: 11,
            w_samples: 21,
            robot_radius_m: 0.3,
            w_heading: 0.3,
            w_clearance: 0.6,
            w_speed: 0.4,
            w_goal: 1.0,
        }
    }
}

/// Chosen command and its predicted rollout.
#[derive(Debug, Clone, PartialEq)]
pub struct DwaCommand {
    /// linear velocity [m/s]
    pub v_mps: f64,
    /// yaw rate [rad/s]
    pub w_radps: f64,
    /// predicted poses
    pub rollout: Vec<Pose2>,
}

/// Unicycle rollout with constant (v, w).
pub fn rollout(p: Pose2, v: f64, w: f64, dt: f64, steps: usize) -> Vec<Pose2> {
    let mut out = Vec::with_capacity(steps + 1);
    let mut s = p;
    out.push(s);
    for _ in 0..steps {
        s = Pose2::new(
            s.x + v * s.theta_rad.cos() * dt,
            s.y + v * s.theta_rad.sin() * dt,
            normalize_angle(s.theta_rad + w * dt),
        );
        out.push(s);
    }
    out
}

/// A moving disc obstacle, predicted at constant velocity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MovingDisc {
    /// position now
    pub pos: Vec2,
    /// velocity
    pub vel: Vec2,
    /// radius [m]
    pub radius_m: f64,
}

/// Picks the best (v, w) in the dynamic window around the current velocity.
pub fn plan(map: &Map2D, pose: Pose2, v_now: f64, w_now: f64, goal: Vec2, cfg: &DwaConfig) -> Option<DwaCommand> {
    plan_with_movers(map, &[], pose, v_now, w_now, goal, cfg)
}

/// [`plan`] among moving discs: the rollout pose at step `k` is checked
/// against each disc's predicted position at step `k`, so the robot can step
/// aside from someone walking towards it instead of freezing in their way.
pub fn plan_with_movers(
    map: &Map2D,
    movers: &[MovingDisc],
    pose: Pose2,
    v_now: f64,
    w_now: f64,
    goal: Vec2,
    cfg: &DwaConfig,
) -> Option<DwaCommand> {
    let v_lo = cfg.v_min_mps.max(v_now - cfg.a_max_mps2 * cfg.dt_s);
    let v_hi = cfg.v_max_mps.min(v_now + cfg.a_max_mps2 * cfg.dt_s);
    let w_lo = (-cfg.w_max_radps).max(w_now - cfg.alpha_max_radps2 * cfg.dt_s);
    let w_hi = cfg.w_max_radps.min(w_now + cfg.alpha_max_radps2 * cfg.dt_s);
    let steps = (cfg.horizon_s / cfg.dt_s).round() as usize;
    let mut best: Option<(f64, DwaCommand)> = None;
    for i in 0..cfg.v_samples {
        let v = v_lo + (v_hi - v_lo) * i as f64 / (cfg.v_samples - 1).max(1) as f64;
        for j in 0..cfg.w_samples {
            let w = w_lo + (w_hi - w_lo) * j as f64 / (cfg.w_samples - 1).max(1) as f64;
            let traj = rollout(pose, v, w, cfg.dt_s, steps);
            let clearance = traj
                .iter()
                .enumerate()
                .map(|(k, p)| {
                    let t = k as f64 * cfg.dt_s;
                    movers
                        .iter()
                        .map(|m| p.pos().dist(m.pos + m.vel * t) - m.radius_m)
                        .fold(map.clearance(p.pos()), f64::min)
                })
                .fold(f64::INFINITY, f64::min)
                - cfg.robot_radius_m;
            if clearance <= 0.0 {
                continue;
            }
            // must be able to stop before the closest obstacle
            if v > 0.0 && v * v / (2.0 * cfg.a_max_mps2) > clearance + 0.05 {
                continue;
            }
            let end = *traj.last().unwrap();
            let to_goal = goal - end.pos();
            let heading =
                1.0 - normalize_angle(to_goal.y.atan2(to_goal.x) - end.theta_rad).abs() / std::f64::consts::PI;
            let score =
                cfg.w_heading * heading + cfg.w_clearance * clearance.min(1.0) + cfg.w_speed * v / cfg.v_max_mps
                    - cfg.w_goal * to_goal.norm() / (cfg.v_max_mps * cfg.horizon_s).max(1e-6);
            if best.as_ref().is_none_or(|(b, _)| score > *b) {
                best = Some((
                    score,
                    DwaCommand {
                        v_mps: v,
                        w_radps: w,
                        rollout: traj,
                    },
                ));
            }
        }
    }
    best.map(|(_, c)| c)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steps_aside_from_an_oncoming_mover() {
        let map = Map2D::new(Vec2::new(0.0, -5.0), Vec2::new(20.0, 5.0));
        let cfg = DwaConfig {
            v_min_mps: -0.3,
            ..DwaConfig::default()
        };
        let goal = Vec2::new(18.0, 0.0);
        let (mut p, mut v, mut w) = (Pose2::new(1.0, 0.0, 0.0), 0.0, 0.0);
        let mut min_gap = f64::INFINITY;
        for k in 0..300 {
            let t = k as f64 * cfg.dt_s;
            // walks straight down the robot's line at 0.8 m/s
            let m = MovingDisc {
                pos: Vec2::new(12.0 - 0.8 * t, 0.0),
                vel: Vec2::new(-0.8, 0.0),
                radius_m: 0.3,
            };
            min_gap = min_gap.min(p.pos().dist(m.pos) - m.radius_m - cfg.robot_radius_m);
            let c = plan_with_movers(&map, &[m], p, v, w, goal, &cfg).expect("a command");
            (v, w) = (c.v_mps, c.w_radps);
            p = c.rollout[1];
        }
        assert!(min_gap > 0.0, "min gap {min_gap}");
        assert!(p.pos().dist(goal) < 1.0, "{p:?}");
    }

    #[test]
    fn reaches_goal_without_collision() {
        let map = Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0)).circle(5.0, 5.0, 1.0);
        let cfg = DwaConfig::default();
        let goal = Vec2::new(9.0, 9.0);
        let (mut p, mut v, mut w) = (Pose2::new(1.0, 1.0, 0.0), 0.0, 0.0);
        for _ in 0..600 {
            let c = plan(&map, p, v, w, goal, &cfg).expect("no admissible command");
            v = c.v_mps;
            w = c.w_radps;
            p = c.rollout[1];
            assert!(map.is_free(p.pos(), cfg.robot_radius_m));
            if p.pos().dist(goal) < 0.3 {
                return;
            }
        }
        panic!("goal not reached: {p:?}");
    }
}
