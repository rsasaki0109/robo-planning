//! Dubins curves: shortest forward-only paths for a car with a minimum turning radius.

use crate::geometry::{normalize_angle, Pose2};
use std::f64::consts::PI;

/// Segment type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Seg {
    /// left turn
    L,
    /// straight
    S,
    /// right turn
    R,
}

/// A Dubins path from `start` with turning radius `radius_m`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DubinsPath {
    /// start pose
    pub start: Pose2,
    /// turning radius [m]
    pub radius_m: f64,
    /// segment types
    pub word: [Seg; 3],
    /// segment lengths in units of the radius
    pub params: [f64; 3],
}

fn mod2pi(a: f64) -> f64 {
    a.rem_euclid(2.0 * PI)
}

impl DubinsPath {
    /// Total length [m].
    pub fn length_m(&self) -> f64 {
        self.params.iter().sum::<f64>() * self.radius_m
    }

    /// Pose at arc length `s_m` along the path.
    pub fn sample(&self, s_m: f64) -> Pose2 {
        let mut s = (s_m / self.radius_m).clamp(0.0, self.params.iter().sum());
        let mut p = Pose2::new(0.0, 0.0, self.start.theta_rad);
        for (seg, &len) in self.word.iter().zip(self.params.iter()) {
            let t = s.min(len);
            p = step(p, *seg, t);
            s -= t;
            if s <= 0.0 {
                break;
            }
        }
        Pose2::new(
            self.start.x + p.x * self.radius_m,
            self.start.y + p.y * self.radius_m,
            normalize_angle(p.theta_rad),
        )
    }

    /// Samples the path every `step_m`.
    pub fn sample_many(&self, step_m: f64) -> Vec<Pose2> {
        let l = self.length_m();
        let n = ((l / step_m).ceil() as usize).max(1);
        (0..=n).map(|i| self.sample(l * i as f64 / n as f64)).collect()
    }
}

fn step(p: Pose2, seg: Seg, t: f64) -> Pose2 {
    let th = p.theta_rad;
    match seg {
        Seg::L => Pose2::new(p.x + (th + t).sin() - th.sin(), p.y - (th + t).cos() + th.cos(), th + t),
        Seg::R => Pose2::new(p.x - (th - t).sin() + th.sin(), p.y + (th - t).cos() - th.cos(), th - t),
        Seg::S => Pose2::new(p.x + th.cos() * t, p.y + th.sin() * t, th),
    }
}

/// Shortest Dubins path between two poses, or `None` if numerically degenerate.
pub fn shortest(start: Pose2, goal: Pose2, radius_m: f64) -> Option<DubinsPath> {
    let dx = goal.x - start.x;
    let dy = goal.y - start.y;
    let d = dx.hypot(dy) / radius_m;
    let th = mod2pi(dy.atan2(dx));
    let a = mod2pi(start.theta_rad - th);
    let b = mod2pi(goal.theta_rad - th);
    let (sa, sb, ca, cb, cab) = (a.sin(), b.sin(), a.cos(), b.cos(), (a - b).cos());
    let mut best: Option<([Seg; 3], [f64; 3])> = None;
    let mut consider = |word: [Seg; 3], p: Option<[f64; 3]>| {
        if let Some(p) = p {
            let c = p[0] + p[1] + p[2];
            if best.is_none_or(|(_, bp)| c < bp[0] + bp[1] + bp[2]) {
                best = Some((word, p));
            }
        }
    };
    // LSL
    {
        let tmp = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sa - sb);
        consider(
            [Seg::L, Seg::S, Seg::L],
            (tmp >= 0.0).then(|| {
                let t0 = (cb - ca).atan2(d + sa - sb);
                [mod2pi(-a + t0), tmp.sqrt(), mod2pi(b - t0)]
            }),
        );
    }
    // RSR
    {
        let tmp = 2.0 + d * d - 2.0 * cab + 2.0 * d * (sb - sa);
        consider(
            [Seg::R, Seg::S, Seg::R],
            (tmp >= 0.0).then(|| {
                let t0 = (ca - cb).atan2(d - sa + sb);
                [mod2pi(a - t0), tmp.sqrt(), mod2pi(t0 - b)]
            }),
        );
    }
    // LSR
    {
        let tmp = -2.0 + d * d + 2.0 * cab + 2.0 * d * (sa + sb);
        consider(
            [Seg::L, Seg::S, Seg::R],
            (tmp >= 0.0).then(|| {
                let p = tmp.sqrt();
                let t0 = (-ca - cb).atan2(d + sa + sb) - (-2.0f64).atan2(p);
                [mod2pi(-a + t0), p, mod2pi(-mod2pi(b) + t0)]
            }),
        );
    }
    // RSL
    {
        let tmp = -2.0 + d * d + 2.0 * cab - 2.0 * d * (sa + sb);
        consider(
            [Seg::R, Seg::S, Seg::L],
            (tmp >= 0.0).then(|| {
                let p = tmp.sqrt();
                let t0 = (ca + cb).atan2(d - sa - sb) - 2.0f64.atan2(p);
                [mod2pi(a - t0), p, mod2pi(b - t0)]
            }),
        );
    }
    // RLR
    {
        let tmp = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sa - sb)) / 8.0;
        consider(
            [Seg::R, Seg::L, Seg::R],
            (tmp.abs() <= 1.0).then(|| {
                let p = mod2pi(2.0 * PI - tmp.acos());
                let t = mod2pi(a - (ca - cb).atan2(d - sa + sb) + p / 2.0);
                [t, p, mod2pi(a - b - t + p)]
            }),
        );
    }
    // LRL
    {
        let tmp = (6.0 - d * d + 2.0 * cab + 2.0 * d * (sb - sa)) / 8.0;
        consider(
            [Seg::L, Seg::R, Seg::L],
            (tmp.abs() <= 1.0).then(|| {
                let p = mod2pi(2.0 * PI - tmp.acos());
                let t = mod2pi(-a - (ca - cb).atan2(d + sa - sb) + p / 2.0);
                [t, p, mod2pi(mod2pi(b) - a - t + p)]
            }),
        );
    }
    best.map(|(word, params)| DubinsPath {
        start,
        radius_m,
        word,
        params,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endpoints_match() {
        let cases = [
            (Pose2::new(0.0, 0.0, 0.0), Pose2::new(10.0, 0.0, 0.0)),
            (Pose2::new(0.0, 0.0, 0.0), Pose2::new(0.0, 5.0, PI)),
            (Pose2::new(1.0, 2.0, 1.0), Pose2::new(-3.0, 4.0, -2.0)),
            (Pose2::new(0.0, 0.0, 0.0), Pose2::new(1.0, 1.0, PI / 2.0)),
            (Pose2::new(0.0, 0.0, 0.0), Pose2::new(0.5, 0.0, PI)),
        ];
        for (s, g) in cases {
            let p = shortest(s, g, 1.5).unwrap();
            let e = p.sample(p.length_m());
            assert!(
                (e.x - g.x).abs() < 1e-6 && (e.y - g.y).abs() < 1e-6,
                "{:?} {:?} {:?}",
                p.word,
                e,
                g
            );
            assert!(normalize_angle(e.theta_rad - g.theta_rad).abs() < 1e-6);
            assert!(p.length_m() >= s.pos().dist(g.pos()) - 1e-9);
        }
        let straight = shortest(cases[0].0, cases[0].1, 1.0).unwrap();
        assert!((straight.length_m() - 10.0).abs() < 1e-9);
    }
}
