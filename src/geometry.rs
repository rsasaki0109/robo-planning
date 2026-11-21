//! 2D geometry: vectors, poses, obstacles and a collision-checkable map.

use std::ops::{Add, Mul, Sub};

/// 2D vector / point in metres.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec2 {
    /// x [m]
    pub x: f64,
    /// y [m]
    pub y: f64,
}

impl Vec2 {
    /// Creates a vector.
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    /// Euclidean norm.
    pub fn norm(self) -> f64 {
        self.x.hypot(self.y)
    }
    /// Distance to another point.
    pub fn dist(self, o: Vec2) -> f64 {
        (self - o).norm()
    }
    /// Dot product.
    pub fn dot(self, o: Vec2) -> f64 {
        self.x * o.x + self.y * o.y
    }
    /// Linear interpolation.
    pub fn lerp(self, o: Vec2, t: f64) -> Vec2 {
        self + (o - self) * t
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x + o.x, self.y + o.y)
    }
}
impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x - o.x, self.y - o.y)
    }
}
impl Mul<f64> for Vec2 {
    type Output = Vec2;
    fn mul(self, s: f64) -> Vec2 {
        Vec2::new(self.x * s, self.y * s)
    }
}

/// Planar pose.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose2 {
    /// x [m]
    pub x: f64,
    /// y [m]
    pub y: f64,
    /// heading [rad]
    pub theta_rad: f64,
}

impl Pose2 {
    /// Creates a pose.
    pub const fn new(x: f64, y: f64, theta_rad: f64) -> Self {
        Self { x, y, theta_rad }
    }
    /// Position part.
    pub fn pos(self) -> Vec2 {
        Vec2::new(self.x, self.y)
    }
}

/// Wraps an angle to `(-pi, pi]`.
pub fn normalize_angle(a: f64) -> f64 {
    let two_pi = 2.0 * std::f64::consts::PI;
    let mut r = (a + std::f64::consts::PI).rem_euclid(two_pi) - std::f64::consts::PI;
    if r <= -std::f64::consts::PI {
        r += two_pi;
    }
    r
}

/// Static obstacle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Obstacle {
    /// Disc.
    Circle {
        /// centre
        center: Vec2,
        /// radius [m]
        radius_m: f64,
    },
    /// Axis-aligned rectangle.
    Rect {
        /// lower-left corner
        min: Vec2,
        /// upper-right corner
        max: Vec2,
    },
}

impl Obstacle {
    /// Signed-ish distance from a point to the obstacle boundary (negative inside for circles, 0 inside rects).
    pub fn distance(&self, p: Vec2) -> f64 {
        match *self {
            Obstacle::Circle { center, radius_m } => p.dist(center) - radius_m,
            Obstacle::Rect { min, max } => {
                let dx = (min.x - p.x).max(0.0).max(p.x - max.x);
                let dy = (min.y - p.y).max(0.0).max(p.y - max.y);
                if dx == 0.0 && dy == 0.0 {
                    -((p.x - min.x).min(max.x - p.x).min(p.y - min.y).min(max.y - p.y))
                } else {
                    dx.hypot(dy)
                }
            }
        }
    }
}

/// Bounded 2D world with static obstacles.
#[derive(Debug, Clone, PartialEq)]
pub struct Map2D {
    /// lower-left corner of the workspace
    pub min: Vec2,
    /// upper-right corner of the workspace
    pub max: Vec2,
    /// obstacles
    pub obstacles: Vec<Obstacle>,
}

impl Map2D {
    /// Creates an empty map.
    pub fn new(min: Vec2, max: Vec2) -> Self {
        Self {
            min,
            max,
            obstacles: Vec::new(),
        }
    }
    /// Adds a circle obstacle.
    pub fn circle(mut self, x: f64, y: f64, r: f64) -> Self {
        self.obstacles.push(Obstacle::Circle {
            center: Vec2::new(x, y),
            radius_m: r,
        });
        self
    }
    /// Adds a rectangle obstacle.
    pub fn rect(mut self, x0: f64, y0: f64, x1: f64, y1: f64) -> Self {
        self.obstacles.push(Obstacle::Rect {
            min: Vec2::new(x0.min(x1), y0.min(y1)),
            max: Vec2::new(x0.max(x1), y0.max(y1)),
        });
        self
    }
    /// Distance to the nearest obstacle or workspace boundary.
    pub fn clearance(&self, p: Vec2) -> f64 {
        let bound = (p.x - self.min.x)
            .min(self.max.x - p.x)
            .min(p.y - self.min.y)
            .min(self.max.y - p.y);
        self.obstacles.iter().map(|o| o.distance(p)).fold(bound, f64::min)
    }
    /// True when a disc of radius `margin_m` at `p` is inside the workspace and collision-free.
    pub fn is_free(&self, p: Vec2, margin_m: f64) -> bool {
        self.clearance(p) > margin_m
    }
    /// Checks a straight segment by sampling at `step_m`.
    pub fn segment_free(&self, a: Vec2, b: Vec2, margin_m: f64, step_m: f64) -> bool {
        let n = ((a.dist(b) / step_m).ceil() as usize).max(1);
        (0..=n).all(|i| self.is_free(a.lerp(b, i as f64 / n as f64), margin_m))
    }
}

/// Total length of a polyline.
pub fn path_length(path: &[Vec2]) -> f64 {
    path.windows(2).map(|w| w[0].dist(w[1])).sum()
}

/// Minimum distance from point `p` to segment `ab`.
pub fn point_segment_distance(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    let ab = b - a;
    let l2 = ab.dot(ab);
    if l2 == 0.0 {
        return p.dist(a);
    }
    let t = ((p - a).dot(ab) / l2).clamp(0.0, 1.0);
    p.dist(a + ab * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angle_wrap() {
        assert!((normalize_angle(3.0 * std::f64::consts::PI) - std::f64::consts::PI).abs() < 1e-12);
        assert!((normalize_angle(-0.5) + 0.5).abs() < 1e-12);
    }

    #[test]
    fn map_collision() {
        let m = Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0))
            .circle(5.0, 5.0, 1.0)
            .rect(1.0, 1.0, 2.0, 2.0);
        assert!(!m.is_free(Vec2::new(5.0, 5.5), 0.0));
        assert!(!m.is_free(Vec2::new(1.5, 1.5), 0.0));
        assert!(m.is_free(Vec2::new(8.0, 8.0), 0.5));
        assert!(!m.segment_free(Vec2::new(0.5, 5.0), Vec2::new(9.5, 5.0), 0.1, 0.05));
    }
}
