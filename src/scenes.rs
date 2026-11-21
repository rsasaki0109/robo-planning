//! Reusable benchmark scenes, one per robot type / task.

use crate::geometry::{Map2D, Pose2, Vec2};

/// A planning task: a map plus start and goal poses.
#[derive(Debug, Clone, PartialEq)]
pub struct Scene {
    /// short identifier
    pub name: &'static str,
    /// map
    pub map: Map2D,
    /// start pose
    pub start: Pose2,
    /// goal pose
    pub goal: Pose2,
}

/// Warehouse aisles for a differential-drive AMR (20 m x 12 m).
pub fn warehouse() -> Scene {
    let mut map = Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(20.0, 12.0));
    // shelf rows with cross-aisle gaps
    for (k, x) in [3.5, 7.0, 10.5, 14.0].iter().enumerate() {
        if k % 2 == 0 {
            map = map.rect(*x, 0.0, x + 1.2, 8.0);
        } else {
            map = map.rect(*x, 4.0, x + 1.2, 12.0);
        }
    }
    map = map.circle(17.5, 6.0, 0.6).circle(1.8, 8.0, 0.5);
    Scene {
        name: "warehouse",
        map,
        start: Pose2::new(1.0, 1.0, 0.0),
        goal: Pose2::new(18.8, 10.8, std::f64::consts::FRAC_PI_2),
    }
}

/// Parking lot for a car-like robot (50 m x 30 m) ending in a bay.
pub fn parking_lot() -> Scene {
    let mut map = Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(50.0, 30.0));
    // parked cars along the top and a central island
    for i in 0..7 {
        let x = 10.0 + i as f64 * 5.0;
        if i != 4 {
            map = map.rect(x, 24.0, x + 2.6, 30.0);
        }
    }
    map = map.rect(12.0, 10.0, 40.0, 14.0).circle(6.0, 20.0, 1.5);
    Scene {
        name: "parking_lot",
        map,
        start: Pose2::new(4.0, 4.0, 0.0),
        goal: Pose2::new(31.3, 26.2, std::f64::consts::FRAC_PI_2),
    }
}

/// Cluttered field for a planar quadrotor (x-z plane, 16 m x 8 m).
pub fn forest() -> Scene {
    let map = Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(16.0, 8.0))
        .rect(0.0, 0.0, 16.0, 0.3)
        .rect(4.0, 0.0, 4.8, 5.0)
        .rect(9.0, 3.0, 9.8, 8.0)
        .circle(6.8, 6.0, 0.7)
        .circle(12.3, 2.0, 0.8)
        .circle(13.5, 5.8, 0.6);
    Scene {
        name: "forest",
        map,
        start: Pose2::new(1.0, 1.5, 0.0),
        goal: Pose2::new(15.0, 6.5, 0.0),
    }
}

/// Work cell for a 3-link planar arm: base at the origin, a pillar between pick and place.
pub fn arm_cell() -> (crate::arm::PlanarArm, Map2D, Vec2, Vec2) {
    let arm = crate::arm::PlanarArm::new(Vec2::new(0.0, 0.0), vec![1.0, 0.8, 0.6], 2.9);
    let map = Map2D::new(Vec2::new(-2.6, -0.6), Vec2::new(2.6, 2.6))
        .rect(-0.25, 1.35, 0.25, 2.6)
        .rect(-2.6, -0.6, 2.6, -0.3)
        .circle(1.05, 1.45, 0.18);
    (arm, map, Vec2::new(1.7, 0.3), Vec2::new(-1.6, 0.6))
}
