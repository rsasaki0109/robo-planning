//! Multi-goal tours (patrol, delivery rounds): visiting order by nearest
//! neighbour + 2-opt over A* path costs, then the stitched path.

use crate::geometry::{Map2D, Vec2};
use crate::grid::{astar, OccupancyGrid};

/// A planned tour.
#[derive(Debug, Clone, PartialEq)]
pub struct Tour {
    /// visiting order as indices into the goal list
    pub order: Vec<usize>,
    /// stitched path through all goals (and back to the start if closed)
    pub path: Vec<Vec2>,
    /// path index at which each goal in `order` is reached
    pub arrivals: Vec<usize>,
    /// total cost [m]
    pub cost_m: f64,
}

/// Orders `goals` to minimise travel from `start`; `closed` returns to `start`.
/// Returns `None` if some goal is unreachable.
pub fn plan_tour(map: &Map2D, start: Vec2, goals: &[Vec2], robot_radius_m: f64, closed: bool) -> Option<Tour> {
    let grid = OccupancyGrid::from_map(map, 0.1, robot_radius_m);
    let pts: Vec<Vec2> = std::iter::once(start).chain(goals.iter().copied()).collect();
    let n = pts.len();
    let mut cost = vec![vec![0.0; n]; n];
    let mut paths = vec![vec![Vec::new(); n]; n];
    for i in 0..n {
        for j in (i + 1)..n {
            let p = astar(&grid, pts[i], pts[j])?;
            let pruned = crate::grid::prune_line_of_sight(map, &p.points, robot_radius_m);
            cost[i][j] = crate::geometry::path_length(&pruned);
            cost[j][i] = cost[i][j];
            let mut rev = pruned.clone();
            rev.reverse();
            paths[i][j] = pruned;
            paths[j][i] = rev;
        }
    }
    // nearest neighbour from the start (node 0)
    let mut seq = vec![0];
    let mut left: Vec<usize> = (1..n).collect();
    while !left.is_empty() {
        let cur = *seq.last().unwrap();
        let (k, _) = left
            .iter()
            .enumerate()
            .min_by(|a, b| cost[cur][*a.1].partial_cmp(&cost[cur][*b.1]).unwrap())
            .unwrap();
        seq.push(left.remove(k));
    }
    if closed {
        seq.push(0);
    }
    let total = |s: &[usize]| s.windows(2).map(|w| cost[w[0]][w[1]]).sum::<f64>();
    // 2-opt (keeps the start fixed, and the end fixed when closed)
    let last = if closed { seq.len() - 1 } else { seq.len() };
    let mut improved = true;
    while improved {
        improved = false;
        for i in 1..last.saturating_sub(1) {
            for j in (i + 1)..last {
                let mut cand = seq.clone();
                cand[i..=j].reverse();
                if total(&cand) + 1e-9 < total(&seq) {
                    seq = cand;
                    improved = true;
                }
            }
        }
    }
    let mut path = vec![start];
    let mut arrivals = Vec::new();
    for w in seq.windows(2) {
        path.extend(paths[w[0]][w[1]].iter().skip(1));
        if w[1] != 0 {
            arrivals.push(path.len() - 1);
        }
    }
    let order = seq.iter().filter(|&&k| k != 0).map(|k| k - 1).collect();
    Some(Tour {
        order,
        path,
        arrivals,
        cost_m: total(&seq),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tour_visits_all_and_beats_given_order() {
        let map = Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0)).rect(4.0, 2.0, 6.0, 8.0);
        let goals = [
            Vec2::new(9.0, 9.0),
            Vec2::new(1.0, 9.0),
            Vec2::new(9.0, 1.0),
            Vec2::new(1.0, 5.0),
        ];
        let t = plan_tour(&map, Vec2::new(1.0, 1.0), &goals, 0.3, true).unwrap();
        let mut o = t.order.clone();
        o.sort();
        assert_eq!(o, vec![0, 1, 2, 3]);
        assert_eq!(t.arrivals.len(), 4);
        assert!(t.path.windows(2).all(|w| map.segment_free(w[0], w[1], 0.29, 0.05)));
    }
}
