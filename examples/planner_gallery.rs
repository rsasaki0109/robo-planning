//! Runs every planner on the scene that fits it and writes SVG figures plus a
//! markdown summary to `out/planning/`.
//!
//! ```bash
//! cargo run --release --example planner_gallery
//! ```

use robo_planning::arm::ArmSpace;
use robo_planning::geometry::path_length;
use robo_planning::grid::{self, OccupancyGrid};
use robo_planning::sampling::{self, Point2Space, SamplingConfig};
use robo_planning::svg::Svg;
use robo_planning::{dubins, dwa, hybrid_astar, scenes, trajectory, Vec2};
use std::fmt::Write;

const OUT: &str = "out/planning";

fn main() -> std::io::Result<()> {
    let mut md = String::from("# Planner gallery\n\n| robot / task | planner | found | length [m] | work | figure |\n|---|---|---|---|---|---|\n");

    // ---- differential-drive AMR in a warehouse: grid + sampling planners
    let sc = scenes::warehouse();
    let radius = 0.35;
    let g = OccupancyGrid::from_map(&sc.map, 0.1, radius);
    let space = Point2Space {
        map: &sc.map,
        margin_m: radius,
    };
    let (s, goal) = (sc.start.pos(), sc.goal.pos());
    let colors = ["#d62728", "#1f77b4", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b"];
    let mut overlay = Svg::for_map(&sc.map, 40.0);
    overlay.map(&sc.map).title("warehouse: all planners");
    let mut row = |name: &str,
                   path: Vec<Vec2>,
                   work: String,
                   extra: &dyn Fn(&mut Svg),
                   idx: usize,
                   md: &mut String|
     -> std::io::Result<()> {
        let mut svg = Svg::for_map(&sc.map, 40.0);
        svg.map(&sc.map);
        extra(&mut svg);
        svg.polyline(&path, colors[idx], 3.0, false)
            .dot(s, 6.0, "green")
            .dot(goal, 6.0, "red")
            .title(&format!("warehouse: {name}"));
        let file = format!(
            "warehouse_{}.svg",
            name.to_lowercase().replace("*", "_star").replace([' ', '-'], "_")
        );
        svg.save(format!("{OUT}/{file}"))?;
        overlay
            .polyline(&path, colors[idx], 2.5, false)
            .legend(idx, name, colors[idx]);
        let _ = writeln!(
            md,
            "| diff-drive / warehouse | {name} | {} | {:.2} | {work} | [svg]({file}) |",
            !path.is_empty(),
            path_length(&path)
        );
        Ok(())
    };
    let d = grid::dijkstra(&g, s, goal).expect("dijkstra");
    row(
        "Dijkstra",
        d.points.clone(),
        format!("{} expanded", d.expanded),
        &|_| {},
        0,
        &mut md,
    )?;
    let a = grid::astar(&g, s, goal).expect("astar");
    row(
        "A*",
        a.points.clone(),
        format!("{} expanded", a.expanded),
        &|_| {},
        1,
        &mut md,
    )?;
    let cfg = SamplingConfig {
        step: 0.6,
        max_iters: 8000,
        goal_tolerance: 0.6,
        ..Default::default()
    };
    let r = sampling::rrt(&space, &s, &goal, &cfg);
    let edges = r.trees[0].edges();
    row(
        "RRT",
        r.path.clone(),
        format!("{} iters", r.iterations),
        &|svg| {
            svg.segments(&edges, "#bbb", 1.0);
        },
        2,
        &mut md,
    )?;
    let rs = sampling::rrt_star(&space, &s, &goal, &SamplingConfig { max_iters: 3000, ..cfg }, 2.0);
    let edges = rs.trees[0].edges();
    row(
        "RRT*",
        rs.path.clone(),
        format!("{} iters", rs.iterations),
        &|svg| {
            svg.segments(&edges, "#bbb", 1.0);
        },
        3,
        &mut md,
    )?;
    let rc = sampling::rrt_connect(&space, &s, &goal, &cfg);
    let (e0, e1) = (rc.trees[0].edges(), rc.trees[1].edges());
    row(
        "RRT-Connect",
        rc.path.clone(),
        format!("{} iters", rc.iterations),
        &|svg| {
            svg.segments(&e0, "#9ecae1", 1.0).segments(&e1, "#fdae6b", 1.0);
        },
        4,
        &mut md,
    )?;
    let pr = sampling::prm(&space, &s, &goal, 400, 2.0, 7);
    let edges = pr.trees[0].edges();
    row(
        "PRM",
        pr.path.clone(),
        format!("{} nodes", pr.iterations),
        &|svg| {
            svg.segments(&edges, "#ddd", 0.8);
        },
        5,
        &mut md,
    )?;
    overlay.dot(s, 6.0, "green").dot(goal, 6.0, "red");
    overlay.save(format!("{OUT}/warehouse_all.svg"))?;

    // smoothing pipeline: A* -> line-of-sight pruning -> spline -> speed profile
    let pruned = grid::prune_line_of_sight(&sc.map, &a.points, radius);
    let smooth = trajectory::spline_path(&trajectory::densify(&pruned, 0.8), 0.05);
    let traj = trajectory::time_parameterize(
        &smooth,
        &trajectory::SpeedLimits {
            v_max_mps: 1.2,
            a_max_mps2: 0.6,
            a_lat_max_mps2: 0.5,
        },
    );
    let pts: Vec<Vec2> = smooth.iter().map(|p| Vec2::new(p.x, p.y)).collect();
    let mut svg = Svg::for_map(&sc.map, 40.0);
    svg.map(&sc.map)
        .polyline(&a.points, "#999", 1.5, true)
        .polyline(&pruned, "#ff7f0e", 2.0, false)
        .polyline(&pts, "#1f77b4", 3.0, false)
        .animate_disc(&pts, radius, "#1f77b4", traj.last().unwrap().t_s)
        .legend(0, "A* raw", "#999")
        .legend(1, "line-of-sight pruned", "#ff7f0e")
        .legend(2, "spline (animated)", "#1f77b4")
        .title("warehouse: A* -> prune -> spline -> speed profile");
    svg.save(format!("{OUT}/warehouse_smoothing.svg"))?;
    let _ = writeln!(md, "| diff-drive / warehouse | A* + prune + spline | true | {:.2} | {:.1} s traverse | [svg](warehouse_smoothing.svg) |", smooth.last().unwrap().s_m, traj.last().unwrap().t_s);

    // DWA from start to goal (reactive, no global plan)
    let dcfg = dwa::DwaConfig {
        robot_radius_m: radius,
        ..Default::default()
    };
    let (mut p, mut v, mut w) = (sc.start, 0.0, 0.0);
    let mut trace = vec![p.pos()];
    // follow pruned A* waypoints as local goals
    let mut wp = 1;
    for _ in 0..3000 {
        if p.pos().dist(pruned[wp]) < 0.6 && wp + 1 < pruned.len() {
            wp += 1;
        }
        let Some(c) = dwa::plan(&sc.map, p, v, w, pruned[wp], &dcfg) else {
            break;
        };
        (v, w, p) = (c.v_mps, c.w_radps, c.rollout[1]);
        trace.push(p.pos());
        if p.pos().dist(goal) < 0.2 {
            break;
        }
    }
    let mut svg = Svg::for_map(&sc.map, 40.0);
    svg.map(&sc.map)
        .polyline(&pruned, "#ff7f0e", 1.5, true)
        .polyline(&trace, "#2ca02c", 3.0, false)
        .animate_disc(&trace, radius, "#2ca02c", trace.len() as f64 * dcfg.dt_s)
        .title("warehouse: DWA following pruned A* waypoints");
    svg.save(format!("{OUT}/warehouse_dwa.svg"))?;
    let _ = writeln!(
        md,
        "| diff-drive / warehouse | DWA (local) | {} | {:.2} | {:.1} s | [svg](warehouse_dwa.svg) |",
        trace.last().unwrap().dist(goal) < 0.2,
        path_length(&trace),
        trace.len() as f64 * dcfg.dt_s
    );

    // ---- car in a parking lot: Hybrid A* and Dubins
    let pk = scenes::parking_lot();
    let hcfg = hybrid_astar::HybridAStarConfig {
        min_turn_radius_m: 4.0,
        footprint_radius_m: 1.1,
        ..Default::default()
    };
    let hr = hybrid_astar::plan(&pk.map, pk.start, pk.goal, &hcfg).expect("hybrid A*");
    let pts: Vec<Vec2> = hr.poses.iter().map(|p| p.pos()).collect();
    let mut svg = Svg::for_map(&pk.map, 18.0);
    svg.map(&pk.map);
    for e in &hr.explored {
        svg.dot(*e, 1.2, "#bbb");
    }
    for p in hr.poses.iter().step_by(12) {
        svg.pose(*p, 1.5, "#9467bd");
    }
    svg.polyline(&pts, "#d62728", 3.0, false)
        .animate_disc(&pts, hcfg.footprint_radius_m, "#d62728", 8.0)
        .pose(pk.start, 2.0, "green")
        .pose(pk.goal, 2.0, "red")
        .title("parking lot: Hybrid A* (car, R_min = 4 m)");
    svg.save(format!("{OUT}/parking_hybrid_astar.svg"))?;
    let _ = writeln!(
        md,
        "| car / parking lot | Hybrid A* | true | {:.2} | {} expanded | [svg](parking_hybrid_astar.svg) |",
        path_length(&pts),
        hr.expanded
    );
    let db = dubins::shortest(pk.start, pk.goal, 4.0).unwrap();
    let dp: Vec<Vec2> = db.sample_many(0.2).iter().map(|p| p.pos()).collect();
    let mut svg = Svg::for_map(&pk.map, 18.0);
    svg.map(&pk.map)
        .polyline(&dp, "#1f77b4", 3.0, false)
        .pose(pk.start, 2.0, "green")
        .pose(pk.goal, 2.0, "red")
        .title(&format!("Dubins {:?} (ignores obstacles)", db.word));
    svg.save(format!("{OUT}/parking_dubins.svg"))?;
    let _ = writeln!(
        md,
        "| car / free space | Dubins | true | {:.2} | closed form | [svg](parking_dubins.svg) |",
        db.length_m()
    );

    // ---- quadrotor in a forest (x-z plane): RRT* + shortcut + spline
    let fo = scenes::forest();
    let fspace = Point2Space {
        map: &fo.map,
        margin_m: 0.35,
    };
    let fr = sampling::rrt_star(
        &fspace,
        &fo.start.pos(),
        &fo.goal.pos(),
        &SamplingConfig {
            step: 0.5,
            max_iters: 2500,
            goal_tolerance: 0.5,
            seed: 3,
            ..Default::default()
        },
        1.5,
    );
    let sc_path = sampling::shortcut(&fspace, &fr.path, 300, 5);
    let sp: Vec<Vec2> = trajectory::spline_path(&trajectory::densify(&sc_path, 0.7), 0.05)
        .iter()
        .map(|p| Vec2::new(p.x, p.y))
        .collect();
    let mut svg = Svg::for_map(&fo.map, 50.0);
    svg.map(&fo.map)
        .segments(&fr.trees[0].edges(), "#ccc", 0.8)
        .polyline(&fr.path, "#ff7f0e", 2.0, true)
        .polyline(&sp, "#d62728", 3.0, false)
        .animate_disc(&sp, 0.3, "#d62728", 6.0)
        .title("quadrotor (x-z): RRT* -> shortcut -> spline");
    svg.save(format!("{OUT}/forest_rrt_star.svg"))?;
    let _ = writeln!(
        md,
        "| quadrotor / forest | RRT* + shortcut | {} | {:.2} | {} iters | [svg](forest_rrt_star.svg) |",
        fr.found(),
        path_length(&sp),
        fr.iterations
    );

    // ---- planar arm: IK + RRT-Connect in joint space
    let (arm, amap, pick, place) = scenes::arm_cell();
    let aspace = ArmSpace {
        arm: &arm,
        map: &amap,
        link_margin_m: 0.06,
    };
    let qa = aspace.ik_valid(pick, 64, 1).expect("ik pick");
    let qb = aspace.ik_valid(place, 64, 2).expect("ik place");
    let ar = sampling::rrt_connect(
        &aspace,
        &qa,
        &qb,
        &SamplingConfig {
            step: 0.15,
            max_iters: 20000,
            ..Default::default()
        },
    );
    let ap = sampling::shortcut(&aspace, &ar.path, 300, 9);
    let mut svg = Svg::for_map(&amap, 140.0);
    svg.map(&amap);
    let jt = trajectory::JointTrajectory::new(ap.clone(), 1.5, 3.0);
    let n = 14;
    for k in 0..=n {
        let (q, _, _) = jt.sample(jt.duration_s() * k as f64 / n as f64);
        let shade = 200 - (k * 140 / n) as u8;
        svg.polyline(&arm.fk(&q), &format!("rgb({shade},{shade},255)"), 4.0, false);
    }
    let ee: Vec<Vec2> = (0..=200)
        .map(|k| arm.end_effector(&jt.sample(jt.duration_s() * k as f64 / 200.0).0))
        .collect();
    svg.polyline(&ee, "#d62728", 2.0, true)
        .dot(pick, 6.0, "green")
        .dot(place, 6.0, "red")
        .title("3-link arm: IK + joint-space RRT-Connect");
    svg.save(format!("{OUT}/arm_rrt_connect.svg"))?;
    let _ = writeln!(md, "| planar arm / pick-place | IK + RRT-Connect + shortcut | {} | {:.2} rad | {} iters, {} vias | [svg](arm_rrt_connect.svg) |", ar.found(), sampling::path_cost(&aspace, &ap), ar.iterations, ap.len());

    std::fs::write(format!("{OUT}/README.md"), &md)?;
    println!("{md}");
    Ok(())
}
