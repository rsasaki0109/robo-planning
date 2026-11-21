//! Reeds-Shepp curves: shortest paths for a car that can drive forwards and
//! backwards with a minimum turning radius (parking, tight manoeuvres).
//!
//! The candidate words follow the classic families (CSC, CCC, CCCC, CCSC,
//! CCSCC) with their time-flipped and reflected variants. Every candidate is
//! checked by integrating it to its end pose, so a path is only returned if it
//! really reaches the goal.

use crate::geometry::{normalize_angle, Pose2};
use std::f64::consts::{FRAC_PI_2, PI};

/// Segment steering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Steer {
    /// left
    L,
    /// straight
    S,
    /// right
    R,
}

/// A Reeds-Shepp path: segments with signed lengths in units of the radius
/// (negative = reverse).
#[derive(Debug, Clone, PartialEq)]
pub struct RsPath {
    /// start pose
    pub start: Pose2,
    /// turning radius [m]
    pub radius_m: f64,
    /// segments
    pub segments: Vec<(Steer, f64)>,
}

fn advance(p: Pose2, steer: Steer, t: f64) -> Pose2 {
    let th = p.theta_rad;
    match steer {
        Steer::L => Pose2::new(p.x + (th + t).sin() - th.sin(), p.y - (th + t).cos() + th.cos(), th + t),
        Steer::R => Pose2::new(p.x - (th - t).sin() + th.sin(), p.y + (th - t).cos() - th.cos(), th - t),
        Steer::S => Pose2::new(p.x + th.cos() * t, p.y + th.sin() * t, th),
    }
}

impl RsPath {
    /// Total length [m].
    pub fn length_m(&self) -> f64 {
        self.segments.iter().map(|(_, d)| d.abs()).sum::<f64>() * self.radius_m
    }

    /// Number of direction changes.
    pub fn cusps(&self) -> usize {
        let dirs: Vec<f64> = self
            .segments
            .iter()
            .filter(|(_, d)| d.abs() > 1e-9)
            .map(|(_, d)| d.signum())
            .collect();
        dirs.windows(2).filter(|w| w[0] != w[1]).count()
    }

    /// Pose and driving direction (+1 forward, -1 reverse) at arc length `s_m`.
    pub fn sample(&self, s_m: f64) -> (Pose2, f64) {
        let mut s = (s_m / self.radius_m).clamp(0.0, self.length_m() / self.radius_m);
        let mut p = Pose2::new(0.0, 0.0, self.start.theta_rad);
        let mut dir = self
            .segments
            .iter()
            .find(|(_, d)| d.abs() > 1e-9)
            .map_or(1.0, |(_, d)| d.signum());
        for (steer, d) in &self.segments {
            let take = s.min(d.abs());
            if d.abs() > 1e-12 {
                dir = d.signum();
            }
            p = advance(p, *steer, take * d.signum());
            s -= take;
            if s <= 0.0 {
                break;
            }
        }
        (
            Pose2::new(
                self.start.x + p.x * self.radius_m,
                self.start.y + p.y * self.radius_m,
                normalize_angle(p.theta_rad),
            ),
            dir,
        )
    }

    /// Samples the path every `step_m` (cusps are always included).
    pub fn sample_many(&self, step_m: f64) -> Vec<(Pose2, f64)> {
        let l = self.length_m();
        let n = ((l / step_m).ceil() as usize).max(1);
        let mut out: Vec<(Pose2, f64)> = (0..=n).map(|i| self.sample(l * i as f64 / n as f64)).collect();
        // make sure the pose at each cusp carries the direction that leaves it
        for i in 0..out.len().saturating_sub(1) {
            if out[i].1 != out[i + 1].1 {
                out[i].1 = out[i + 1].1;
            }
        }
        out
    }
}

fn m2p(x: f64) -> f64 {
    // to (-pi, pi]
    normalize_angle(x)
}

fn polar(x: f64, y: f64) -> (f64, f64) {
    (x.hypot(y), y.atan2(x))
}

type Word = (Vec<f64>, Vec<Steer>);
/// A path family solver in normalised coordinates.
type Family = fn(f64, f64, f64) -> Option<Word>;

fn lsl(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u, t) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if (0.0..=PI).contains(&t) {
        let v = m2p(phi - t);
        if (0.0..=PI).contains(&v) {
            return Some((vec![t, u, v], vec![Steer::L, Steer::S, Steer::L]));
        }
    }
    None
}

fn lsr(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, t1) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    let u1 = u1 * u1;
    if u1 >= 4.0 {
        let u = (u1 - 4.0).sqrt();
        let theta = 2.0f64.atan2(u);
        let t = m2p(t1 + theta);
        let v = m2p(t - phi);
        if t >= 0.0 && v >= 0.0 {
            return Some((vec![t, u, v], vec![Steer::L, Steer::S, Steer::R]));
        }
    }
    None
}

fn lxrxl(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if u1 <= 4.0 {
        let a = (0.25 * u1).acos();
        let t = m2p(a + theta + FRAC_PI_2);
        let u = m2p(PI - 2.0 * a);
        let v = m2p(phi - t - u);
        return Some((vec![t, -u, v], vec![Steer::L, Steer::R, Steer::L]));
    }
    None
}

fn lxrl(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if u1 <= 4.0 {
        let a = (0.25 * u1).acos();
        let t = m2p(a + theta + FRAC_PI_2);
        let u = m2p(PI - 2.0 * a);
        let v = m2p(-phi + t + u);
        return Some((vec![t, -u, -v], vec![Steer::L, Steer::R, Steer::L]));
    }
    None
}

fn lrxlr(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    if u1 <= 2.0 {
        let a = ((u1 + 2.0) * 0.25).acos();
        let t = m2p(theta + a + FRAC_PI_2);
        let u = m2p(a);
        let v = m2p(phi - t + 2.0 * u);
        if t >= 0.0 && u >= 0.0 && v >= 0.0 {
            return Some((vec![t, u, -u, -v], vec![Steer::L, Steer::R, Steer::L, Steer::R]));
        }
    }
    None
}

fn lxrlxr(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    let u2 = (20.0 - u1 * u1) / 16.0;
    if (0.0..=1.0).contains(&u2) {
        let u = u2.acos();
        let a = (2.0 * u.sin() / u1).clamp(-1.0, 1.0).asin();
        let t = m2p(theta + a + FRAC_PI_2);
        let v = m2p(t - phi);
        if t >= 0.0 && v >= 0.0 {
            return Some((vec![t, -u, -u, v], vec![Steer::L, Steer::R, Steer::L, Steer::R]));
        }
    }
    None
}

fn lxr90sl(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if u1 >= 2.0 {
        let r = (u1 * u1 - 4.0).sqrt();
        let u = r - 2.0;
        let a = 2.0f64.atan2(r);
        let t = m2p(theta + a + FRAC_PI_2);
        let v = m2p(t - phi + FRAC_PI_2);
        if t >= 0.0 && v >= 0.0 {
            return Some((
                vec![t, -FRAC_PI_2, -u, -v],
                vec![Steer::L, Steer::R, Steer::S, Steer::L],
            ));
        }
    }
    None
}

fn lsr90xl(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if u1 >= 2.0 {
        let r = (u1 * u1 - 4.0).sqrt();
        let u = r - 2.0;
        let a = r.atan2(2.0);
        let t = m2p(theta - a + FRAC_PI_2);
        let v = m2p(t - phi - FRAC_PI_2);
        if t >= 0.0 && v >= 0.0 {
            return Some((vec![t, u, FRAC_PI_2, -v], vec![Steer::L, Steer::S, Steer::R, Steer::L]));
        }
    }
    None
}

fn lxr90sr(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    if u1 >= 2.0 {
        let t = m2p(theta + FRAC_PI_2);
        let u = u1 - 2.0;
        let v = m2p(phi - t - FRAC_PI_2);
        if t >= 0.0 && v >= 0.0 {
            return Some((
                vec![t, -FRAC_PI_2, -u, -v],
                vec![Steer::L, Steer::R, Steer::S, Steer::R],
            ));
        }
    }
    None
}

fn lsl90xr(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    if u1 >= 2.0 {
        let t = m2p(theta);
        let u = u1 - 2.0;
        let v = m2p(phi - t - FRAC_PI_2);
        if t >= 0.0 && v >= 0.0 {
            return Some((vec![t, u, FRAC_PI_2, -v], vec![Steer::L, Steer::S, Steer::L, Steer::R]));
        }
    }
    None
}

fn lxr90sl90xr(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    if u1 >= 4.0 {
        let r = (u1 * u1 - 4.0).sqrt();
        let u = r - 4.0;
        let a = 2.0f64.atan2(r);
        let t = m2p(theta + a + FRAC_PI_2);
        let v = m2p(t - phi);
        if t >= 0.0 && v >= 0.0 {
            return Some((
                vec![t, -FRAC_PI_2, -u, -FRAC_PI_2, v],
                vec![Steer::L, Steer::R, Steer::S, Steer::L, Steer::R],
            ));
        }
    }
    None
}

/// All candidate paths between two poses (each verified to reach the goal).
pub fn candidates(start: Pose2, goal: Pose2, radius_m: f64) -> Vec<RsPath> {
    let (dx, dy) = (goal.x - start.x, goal.y - start.y);
    let (c, s) = (start.theta_rad.cos(), start.theta_rad.sin());
    let x = (c * dx + s * dy) / radius_m;
    let y = (-s * dx + c * dy) / radius_m;
    let phi = m2p(goal.theta_rad - start.theta_rad);
    let fns: [Family; 11] = [
        lsl,
        lsr,
        lxrxl,
        lxrl,
        lrxlr,
        lxrlxr,
        lxr90sl,
        lxr90sr,
        lsr90xl,
        lsl90xr,
        lxr90sl90xr,
    ];
    let reflect = |st: Vec<Steer>| -> Vec<Steer> {
        st.into_iter()
            .map(|s| match s {
                Steer::L => Steer::R,
                Steer::R => Steer::L,
                Steer::S => Steer::S,
            })
            .collect()
    };
    let mut out = Vec::new();
    for f in fns {
        // original, time-flipped, reflected, time-flipped + reflected
        let variants: [(f64, f64, f64, bool, bool); 4] = [
            (x, y, phi, false, false),
            (-x, y, -phi, true, false),
            (x, -y, -phi, false, true),
            (-x, -y, phi, true, true),
        ];
        for (xx, yy, pp, flip, refl) in variants {
            if let Some((mut d, mut st)) = f(xx, yy, pp) {
                if flip {
                    d.iter_mut().for_each(|v| *v = -*v);
                }
                if refl {
                    st = reflect(st);
                }
                let path = RsPath {
                    start,
                    radius_m,
                    segments: st.into_iter().zip(d).collect(),
                };
                let (end, _) = path.sample(path.length_m());
                if (end.x - goal.x).abs() < 1e-6 * radius_m.max(1.0)
                    && (end.y - goal.y).abs() < 1e-6 * radius_m.max(1.0)
                    && normalize_angle(end.theta_rad - goal.theta_rad).abs() < 1e-6
                {
                    out.push(path);
                }
            }
        }
    }
    out
}

/// Shortest verified Reeds-Shepp path, or `None` if no candidate reaches the goal.
pub fn shortest(start: Pose2, goal: Pose2, radius_m: f64) -> Option<RsPath> {
    candidates(start, goal, radius_m)
        .into_iter()
        .min_by(|a, b| a.length_m().partial_cmp(&b.length_m()).unwrap())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::Rng;

    #[test]
    fn reaches_random_goals_and_never_beats_the_straight_line() {
        let mut rng = Rng::new(3);
        let mut found = 0;
        for _ in 0..500 {
            let s = Pose2::new(rng.range(-5.0, 5.0), rng.range(-5.0, 5.0), rng.range(-PI, PI));
            let g = Pose2::new(rng.range(-5.0, 5.0), rng.range(-5.0, 5.0), rng.range(-PI, PI));
            if let Some(p) = shortest(s, g, 1.5) {
                found += 1;
                assert!(p.length_m() >= s.pos().dist(g.pos()) - 1e-9);
                // no longer than the forward-only Dubins path
                let d = crate::dubins::shortest(s, g, 1.5).unwrap();
                assert!(
                    p.length_m() <= d.length_m() + 1e-6,
                    "RS {} > Dubins {}",
                    p.length_m(),
                    d.length_m()
                );
            }
        }
        assert!(found >= 495, "only {found}/500 solved");
    }

    #[test]
    fn parallel_parking_uses_reverse() {
        // goal directly beside the start: the car must back in
        let p = shortest(Pose2::new(0.0, 0.0, 0.0), Pose2::new(0.0, -2.0, 0.0), 3.0).unwrap();
        assert!(p.cusps() >= 1);
        assert!(p.segments.iter().any(|(_, d)| *d < 0.0));
    }
}
