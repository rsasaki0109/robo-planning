//! # robo-planning
//!
//! Motion planning algorithms grouped by the kind of robot and task they fit.
//!
//! | Robot / task | Module | Algorithms |
//! |---|---|---|
//! | Holonomic / differential drive on a map | [`grid`] | Dijkstra, A*, weighted A* |
//! | Any configuration space | [`sampling`] | RRT, RRT*, RRT-Connect, PRM, shortcut smoothing |
//! | Car-like (minimum turning radius) | [`dubins`], [`hybrid_astar`] | Dubins curves, Hybrid A* |
//! | Planar serial arm | [`arm`] | FK, analytic/numeric IK, joint-space collision checking |
//! | Differential drive, reactive | [`dwa`] | Dynamic Window Approach |
//! | All | [`trajectory`] | cubic spline, quintic polynomial, trapezoidal profile, path time-parameterisation |
//!
//! The crate has no dependencies, is deterministic (all randomness comes from a
//! seeded [`rng::Rng`]) and needs no renderer; [`svg`] writes plain SVG figures.

pub mod arm;
pub mod dubins;
pub mod dwa;
pub mod geometry;
pub mod grid;
pub mod hybrid_astar;
pub mod rng;
pub mod sampling;
pub mod scenes;
pub mod svg;
pub mod trajectory;

pub use geometry::{Map2D, Obstacle, Pose2, Vec2};
