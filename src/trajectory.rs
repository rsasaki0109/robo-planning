//! Trajectory generation: splines, polynomials, velocity profiles and path
//! time-parameterisation. Planners produce geometric paths; controllers need
//! time-stamped references, and this module bridges the two.

use crate::geometry::{normalize_angle, Pose2, Vec2};

/// Natural cubic spline `y(x)` over strictly increasing knots.
#[derive(Debug, Clone, PartialEq)]
pub struct CubicSpline1D {
    x: Vec<f64>,
    a: Vec<f64>,
    b: Vec<f64>,
    c: Vec<f64>,
    d: Vec<f64>,
}

impl CubicSpline1D {
    /// Fits a natural cubic spline. Needs at least two knots.
    pub fn new(x: &[f64], y: &[f64]) -> Self {
        let n = x.len();
        assert!(n >= 2 && y.len() == n, "spline needs >= 2 matching knots");
        let h: Vec<f64> = x.windows(2).map(|w| w[1] - w[0]).collect();
        // tridiagonal system for c
        let mut c = vec![0.0; n];
        if n > 2 {
            let m = n - 2;
            let mut diag = vec![0.0; m];
            let mut rhs = vec![0.0; m];
            for i in 0..m {
                diag[i] = 2.0 * (h[i] + h[i + 1]);
                rhs[i] = 3.0 * ((y[i + 2] - y[i + 1]) / h[i + 1] - (y[i + 1] - y[i]) / h[i]);
            }
            // Thomas algorithm, sub/super diagonal = h[i+1]
            for i in 1..m {
                let w = h[i] / diag[i - 1];
                diag[i] -= w * h[i];
                rhs[i] -= w * rhs[i - 1];
            }
            let mut sol = vec![0.0; m];
            sol[m - 1] = rhs[m - 1] / diag[m - 1];
            for i in (0..m - 1).rev() {
                sol[i] = (rhs[i] - h[i + 1] * sol[i + 1]) / diag[i];
            }
            c[1..(m + 1)].copy_from_slice(&sol[..m]);
        }
        let mut b = vec![0.0; n - 1];
        let mut d = vec![0.0; n - 1];
        for i in 0..n - 1 {
            d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);
            b[i] = (y[i + 1] - y[i]) / h[i] - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0;
        }
        Self {
            x: x.to_vec(),
            a: y.to_vec(),
            b,
            c,
            d,
        }
    }
    fn seg(&self, t: f64) -> (usize, f64) {
        let n = self.x.len();
        let i = match self.x.binary_search_by(|v| v.partial_cmp(&t).unwrap()) {
            Ok(i) => i.min(n - 2),
            Err(i) => i.saturating_sub(1).min(n - 2),
        };
        (i, t - self.x[i])
    }
    /// Value.
    pub fn eval(&self, t: f64) -> f64 {
        let (i, dx) = self.seg(t);
        self.a[i] + self.b[i] * dx + self.c[i] * dx * dx + self.d[i] * dx * dx * dx
    }
    /// First derivative.
    pub fn d1(&self, t: f64) -> f64 {
        let (i, dx) = self.seg(t);
        self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx * dx
    }
    /// Second derivative.
    pub fn d2(&self, t: f64) -> f64 {
        let (i, dx) = self.seg(t);
        2.0 * self.c[i] + 6.0 * self.d[i] * dx
    }
}

/// Point on a geometric path.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PathPoint {
    /// arc length [m]
    pub s_m: f64,
    /// x [m]
    pub x: f64,
    /// y [m]
    pub y: f64,
    /// tangent heading [rad]
    pub yaw_rad: f64,
    /// curvature [1/m]
    pub curvature: f64,
}

/// Point on a time-parameterised path.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TrajPoint {
    /// time [s]
    pub t_s: f64,
    /// geometric data
    pub p: PathPoint,
    /// speed [m/s]
    pub v_mps: f64,
    /// tangential acceleration [m/s^2]
    pub a_mps2: f64,
}

/// Inserts points so no segment is longer than `max_seg_m` (keeps splines close to the polyline).
pub fn densify(path: &[Vec2], max_seg_m: f64) -> Vec<Vec2> {
    let mut out = Vec::new();
    for w in path.windows(2) {
        let n = ((w[0].dist(w[1]) / max_seg_m).ceil() as usize).max(1);
        for i in 0..n {
            out.push(w[0].lerp(w[1], i as f64 / n as f64));
        }
    }
    if let Some(l) = path.last() {
        out.push(*l);
    }
    out
}

/// Fits a 2D arc-length cubic spline through waypoints and samples it every `ds_m`.
pub fn spline_path(waypoints: &[Vec2], ds_m: f64) -> Vec<PathPoint> {
    let mut pts: Vec<Vec2> = Vec::new();
    for p in waypoints {
        if pts.last().is_none_or(|l: &Vec2| l.dist(*p) > 1e-6) {
            pts.push(*p);
        }
    }
    if pts.len() < 2 {
        return pts
            .iter()
            .map(|p| PathPoint {
                x: p.x,
                y: p.y,
                ..Default::default()
            })
            .collect();
    }
    let mut s = vec![0.0];
    for w in pts.windows(2) {
        s.push(s.last().unwrap() + w[0].dist(w[1]));
    }
    let sx = CubicSpline1D::new(&s, &pts.iter().map(|p| p.x).collect::<Vec<_>>());
    let sy = CubicSpline1D::new(&s, &pts.iter().map(|p| p.y).collect::<Vec<_>>());
    let total = *s.last().unwrap();
    let n = ((total / ds_m).ceil() as usize).max(1);
    let mut out: Vec<PathPoint> = (0..=n)
        .map(|i| {
            let t = total * i as f64 / n as f64;
            let (dx, dy, ddx, ddy) = (sx.d1(t), sy.d1(t), sx.d2(t), sy.d2(t));
            PathPoint {
                s_m: 0.0,
                x: sx.eval(t),
                y: sy.eval(t),
                yaw_rad: dy.atan2(dx),
                curvature: (dx * ddy - dy * ddx) / (dx * dx + dy * dy).powf(1.5).max(1e-9),
            }
        })
        .collect();
    // recompute true arc length
    for i in 1..out.len() {
        out[i].s_m = out[i - 1].s_m + (out[i].x - out[i - 1].x).hypot(out[i].y - out[i - 1].y);
    }
    out
}

/// Converts a pose sequence (e.g. from Hybrid A*) into path points with finite-difference curvature.
pub fn poses_to_path(poses: &[Pose2]) -> Vec<PathPoint> {
    let mut out: Vec<PathPoint> = Vec::with_capacity(poses.len());
    for p in poses {
        if let Some(l) = out.last() {
            if (p.x - l.x).hypot(p.y - l.y) < 1e-6 {
                continue;
            }
        }
        let s = out
            .last()
            .map_or(0.0, |l: &PathPoint| l.s_m + (p.x - l.x).hypot(p.y - l.y));
        out.push(PathPoint {
            s_m: s,
            x: p.x,
            y: p.y,
            yaw_rad: p.theta_rad,
            curvature: 0.0,
        });
    }
    for i in 1..out.len().saturating_sub(1) {
        let ds = out[i + 1].s_m - out[i - 1].s_m;
        out[i].curvature = normalize_angle(out[i + 1].yaw_rad - out[i - 1].yaw_rad) / ds.max(1e-9);
    }
    out
}

/// Speed limits for [`time_parameterize`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpeedLimits {
    /// max speed [m/s]
    pub v_max_mps: f64,
    /// max tangential acceleration / deceleration [m/s^2]
    pub a_max_mps2: f64,
    /// max lateral acceleration [m/s^2] (limits speed in curves)
    pub a_lat_max_mps2: f64,
}

/// Forward-backward pass speed profile that starts and ends at rest.
pub fn time_parameterize(path: &[PathPoint], lim: &SpeedLimits) -> Vec<TrajPoint> {
    let n = path.len();
    if n == 0 {
        return vec![];
    }
    let mut v: Vec<f64> = path
        .iter()
        .map(|p| {
            lim.v_max_mps
                .min((lim.a_lat_max_mps2 / p.curvature.abs().max(1e-6)).sqrt())
        })
        .collect();
    v[0] = 0.0;
    v[n - 1] = 0.0;
    for i in 1..n {
        let ds = path[i].s_m - path[i - 1].s_m;
        v[i] = v[i].min((v[i - 1] * v[i - 1] + 2.0 * lim.a_max_mps2 * ds).sqrt());
    }
    for i in (0..n - 1).rev() {
        let ds = path[i + 1].s_m - path[i].s_m;
        v[i] = v[i].min((v[i + 1] * v[i + 1] + 2.0 * lim.a_max_mps2 * ds).sqrt());
    }
    let mut out = Vec::with_capacity(n);
    let mut t = 0.0;
    for i in 0..n {
        if i > 0 {
            let ds = path[i].s_m - path[i - 1].s_m;
            t += 2.0 * ds / (v[i] + v[i - 1]).max(1e-6);
        }
        let a = if i + 1 < n {
            let ds = (path[i + 1].s_m - path[i].s_m).max(1e-9);
            (v[i + 1] * v[i + 1] - v[i] * v[i]) / (2.0 * ds)
        } else {
            0.0
        };
        out.push(TrajPoint {
            t_s: t,
            p: path[i],
            v_mps: v[i],
            a_mps2: a,
        });
    }
    out
}

/// Interpolates a trajectory at time `t_s` (clamped to its ends).
pub fn sample_traj(traj: &[TrajPoint], t_s: f64) -> TrajPoint {
    let last = *traj.last().expect("empty trajectory");
    if t_s >= last.t_s {
        return TrajPoint {
            t_s,
            v_mps: 0.0,
            a_mps2: 0.0,
            ..last
        };
    }
    if t_s <= traj[0].t_s {
        return traj[0];
    }
    let i = traj.partition_point(|p| p.t_s <= t_s) - 1;
    let (a, b) = (traj[i], traj[i + 1]);
    let u = (t_s - a.t_s) / (b.t_s - a.t_s).max(1e-12);
    let l = |x: f64, y: f64| x + (y - x) * u;
    TrajPoint {
        t_s,
        p: PathPoint {
            s_m: l(a.p.s_m, b.p.s_m),
            x: l(a.p.x, b.p.x),
            y: l(a.p.y, b.p.y),
            yaw_rad: a.p.yaw_rad + normalize_angle(b.p.yaw_rad - a.p.yaw_rad) * u,
            curvature: l(a.p.curvature, b.p.curvature),
        },
        v_mps: l(a.v_mps, b.v_mps),
        a_mps2: a.a_mps2,
    }
}

/// Quintic polynomial with position/velocity/acceleration boundary conditions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuinticPolynomial {
    c: [f64; 6],
    /// duration [s]
    pub duration_s: f64,
}

impl QuinticPolynomial {
    /// Solves for the coefficients.
    pub fn new(p0: f64, v0: f64, a0: f64, p1: f64, v1: f64, a1: f64, duration_s: f64) -> Self {
        let t = duration_s;
        let (t2, t3, t4, t5) = (t * t, t * t * t, t.powi(4), t.powi(5));
        let c0 = p0;
        let c1 = v0;
        let c2 = a0 / 2.0;
        let r0 = p1 - c0 - c1 * t - c2 * t2;
        let r1 = v1 - c1 - 2.0 * c2 * t;
        let r2 = a1 - 2.0 * c2;
        let c3 = (20.0 * r0 - 8.0 * r1 * t + r2 * t2) / (2.0 * t3);
        let c4 = (-30.0 * r0 + 14.0 * r1 * t - 2.0 * r2 * t2) / (2.0 * t4);
        let c5 = (12.0 * r0 - 6.0 * r1 * t + r2 * t2) / (2.0 * t5);
        Self {
            c: [c0, c1, c2, c3, c4, c5],
            duration_s,
        }
    }
    /// (position, velocity, acceleration) at `t` (clamped).
    pub fn eval(&self, t: f64) -> (f64, f64, f64) {
        let t = t.clamp(0.0, self.duration_s);
        let c = &self.c;
        let p = c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))));
        let v = c[1] + t * (2.0 * c[2] + t * (3.0 * c[3] + t * (4.0 * c[4] + t * 5.0 * c[5])));
        let a = 2.0 * c[2] + t * (6.0 * c[3] + t * (12.0 * c[4] + t * 20.0 * c[5]));
        (p, v, a)
    }
}

/// Trapezoidal (or triangular) velocity profile over a distance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrapezoidalProfile {
    /// distance (signed)
    pub distance: f64,
    /// cruise speed reached
    pub v_peak: f64,
    /// acceleration magnitude
    pub accel: f64,
    /// acceleration phase duration [s]
    pub t_acc_s: f64,
    /// total duration [s]
    pub duration_s: f64,
}

impl TrapezoidalProfile {
    /// Builds a rest-to-rest profile.
    pub fn new(distance: f64, v_max: f64, a_max: f64) -> Self {
        let d = distance.abs();
        let (v_peak, t_acc, total) = if d < v_max * v_max / a_max {
            let tp = (d / a_max).sqrt();
            (a_max * tp, tp, 2.0 * tp)
        } else {
            let ta = v_max / a_max;
            (v_max, ta, 2.0 * ta + (d - v_max * ta) / v_max)
        };
        Self {
            distance,
            v_peak,
            accel: a_max,
            t_acc_s: t_acc,
            duration_s: total,
        }
    }
    /// (position, velocity, acceleration) at `t`.
    pub fn eval(&self, t: f64) -> (f64, f64, f64) {
        let sg = self.distance.signum();
        let t = t.clamp(0.0, self.duration_s);
        let (ta, tt) = (self.t_acc_s, self.duration_s);
        let (p, v, a) = if t < ta {
            (0.5 * self.accel * t * t, self.accel * t, self.accel)
        } else if t < tt - ta {
            (0.5 * self.accel * ta * ta + self.v_peak * (t - ta), self.v_peak, 0.0)
        } else {
            let r = tt - t;
            (
                self.distance.abs() - 0.5 * self.accel * r * r,
                self.accel * r,
                -self.accel,
            )
        };
        (sg * p, sg * v, sg * a)
    }
}

/// Joint-space trajectory through via points: each segment is a synchronised
/// rest-to-rest quintic whose duration respects per-joint velocity/acceleration limits.
#[derive(Debug, Clone, PartialEq)]
pub struct JointTrajectory {
    /// via points
    pub waypoints: Vec<Vec<f64>>,
    /// segment start times [s]
    pub t_start_s: Vec<f64>,
    /// segment durations [s]
    pub durations_s: Vec<f64>,
}

impl JointTrajectory {
    /// Builds the trajectory.
    pub fn new(waypoints: Vec<Vec<f64>>, v_max: f64, a_max: f64) -> Self {
        let mut t_start_s = vec![];
        let mut durations_s = vec![];
        let mut t = 0.0;
        for w in waypoints.windows(2) {
            let dmax = w[0].iter().zip(&w[1]).map(|(a, b)| (b - a).abs()).fold(0.0, f64::max);
            // quintic rest-to-rest: v_peak = 1.875 d / T, a_peak = 5.7735 d / T^2
            let dur = (1.875 * dmax / v_max).max((5.7735 * dmax / a_max).sqrt()).max(0.05);
            t_start_s.push(t);
            durations_s.push(dur);
            t += dur;
        }
        Self {
            waypoints,
            t_start_s,
            durations_s,
        }
    }
    /// Total duration [s].
    pub fn duration_s(&self) -> f64 {
        self.t_start_s
            .last()
            .map_or(0.0, |s| s + self.durations_s.last().unwrap())
    }
    /// (q, qd, qdd) at time `t`.
    pub fn sample(&self, t: f64) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let dof = self.waypoints[0].len();
        if self.durations_s.is_empty() {
            return (self.waypoints[0].clone(), vec![0.0; dof], vec![0.0; dof]);
        }
        let k = self.t_start_s.partition_point(|&s| s <= t).saturating_sub(1);
        let tau = t - self.t_start_s[k];
        let (a, b) = (&self.waypoints[k], &self.waypoints[k + 1]);
        let s = QuinticPolynomial::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, self.durations_s[k]);
        let (sp, sv, sa) = s.eval(tau);
        let q = a.iter().zip(b).map(|(x, y)| x + (y - x) * sp).collect();
        let qd = a.iter().zip(b).map(|(x, y)| (y - x) * sv).collect();
        let qdd = a.iter().zip(b).map(|(x, y)| (y - x) * sa).collect();
        (q, qd, qdd)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quintic_boundaries() {
        let q = QuinticPolynomial::new(1.0, 0.5, 0.0, 4.0, -0.2, 0.1, 3.0);
        let (p0, v0, a0) = q.eval(0.0);
        let (p1, v1, a1) = q.eval(3.0);
        assert!((p0 - 1.0).abs() < 1e-12 && (v0 - 0.5).abs() < 1e-12 && a0.abs() < 1e-12);
        assert!((p1 - 4.0).abs() < 1e-9 && (v1 + 0.2).abs() < 1e-9 && (a1 - 0.1).abs() < 1e-9);
    }

    #[test]
    fn trapezoid_reaches_distance() {
        for d in [0.1, 5.0, -3.0] {
            let t = TrapezoidalProfile::new(d, 1.0, 2.0);
            let (p, v, _) = t.eval(t.duration_s);
            assert!((p - d).abs() < 1e-9 && v.abs() < 1e-9);
        }
    }

    #[test]
    fn spline_interpolates_and_profile_respects_limits() {
        let wp = [
            Vec2::new(0.0, 0.0),
            Vec2::new(5.0, 0.0),
            Vec2::new(5.0, 5.0),
            Vec2::new(10.0, 5.0),
        ];
        let p = spline_path(&densify(&wp, 1.0), 0.1);
        assert!((p.last().unwrap().x - 10.0).abs() < 1e-9);
        let lim = SpeedLimits {
            v_max_mps: 2.0,
            a_max_mps2: 1.0,
            a_lat_max_mps2: 1.0,
        };
        let tr = time_parameterize(&p, &lim);
        assert!(tr.iter().all(|t| t.v_mps <= 2.0 + 1e-9));
        assert!(tr.iter().all(|t| t.v_mps * t.v_mps * t.p.curvature.abs() <= 1.0 + 1e-6));
        assert_eq!(tr.last().unwrap().v_mps, 0.0);
        let mid = sample_traj(&tr, tr.last().unwrap().t_s / 2.0);
        assert!(mid.v_mps > 0.0);
    }

    #[test]
    fn joint_traj_limits() {
        let jt = JointTrajectory::new(vec![vec![0.0, 0.0], vec![1.0, -2.0], vec![0.5, 0.5]], 1.0, 2.0);
        let mut t = 0.0;
        while t < jt.duration_s() {
            let (_, qd, qdd) = jt.sample(t);
            assert!(qd.iter().all(|v| v.abs() <= 1.0 + 1e-6));
            assert!(qdd.iter().all(|a| a.abs() <= 2.0 + 1e-3));
            t += 0.01;
        }
        let (q, _, _) = jt.sample(jt.duration_s());
        assert!((q[1] - 0.5).abs() < 1e-9);
    }
}
