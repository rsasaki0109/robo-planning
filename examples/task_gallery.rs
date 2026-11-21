//! Animated GIFs of planning **by task**: goal reaching, charging, cleaning
//! coverage, patrol, moving-target following, parking and pick-and-place.
//!
//! ```bash
//! cargo run --release --features render --example task_gallery   # writes out/tasks/*.gif
//! ```

use robo_planning::arm::ArmSpace;
use robo_planning::geometry::{path_length, Map2D};
use robo_planning::grid::{self, OccupancyGrid};
use robo_planning::render::{colors, Canvas, GifWriter, Rgba, View};
use robo_planning::sampling::{self, Point2Space, SamplingConfig};
use robo_planning::trajectory::{self, sample_traj, SpeedLimits, TrajPoint};
use robo_planning::{coverage, docking, hybrid_astar, scenes, tour, Pose2, Vec2};

const OUT: &str = "out/tasks";
const DELAY: u16 = 6; // centiseconds per frame

fn title(c: &mut Canvas, s: &str) {
    c.rect_px(0.0, 0.0, c.width() as f32, 30.0, colors::INK);
    c.text(10.0, 8.0, s, 2.0, colors::WHITE);
}

fn traj_of(pts: &[Vec2], v: f64) -> Vec<TrajPoint> {
    let path = trajectory::spline_path(&trajectory::densify(pts, 0.6), 0.05);
    trajectory::time_parameterize(
        &path,
        &SpeedLimits {
            v_max_mps: v,
            a_max_mps2: 1.0,
            a_lat_max_mps2: 1.0,
        },
    )
}

fn pose_at(tr: &[TrajPoint], t: f64) -> Pose2 {
    let s = sample_traj(tr, t);
    Pose2::new(s.p.x, s.p.y, s.p.yaw_rad)
}

fn trail(tr: &[TrajPoint], t: f64) -> Vec<Vec2> {
    tr.iter()
        .take_while(|p| p.t_s <= t)
        .map(|p| Vec2::new(p.p.x, p.p.y))
        .collect()
}

/// Goal reaching: four planners side by side, search first, then the robot drives.
fn goal_reaching() -> std::io::Result<()> {
    let sc = scenes::warehouse();
    let r = 0.35;
    let g = OccupancyGrid::from_map(&sc.map, 0.1, r);
    let sp = Point2Space {
        map: &sc.map,
        margin_m: r,
    };
    let (s, goal) = (sc.start.pos(), sc.goal.pos());
    let dj = grid::dijkstra(&g, s, goal).unwrap();
    let a = grid::astar(&g, s, goal).unwrap();
    let cfg = SamplingConfig {
        step: 0.6,
        max_iters: 8000,
        goal_tolerance: 0.6,
        ..Default::default()
    };
    let rrt = sampling::rrt(&sp, &s, &goal, &cfg);
    let rs = sampling::rrt_star(&sp, &s, &goal, &SamplingConfig { max_iters: 2500, ..cfg }, 2.0);
    struct Panel {
        name: String,
        cells: Vec<Vec2>,
        edges: Vec<(Vec2, Vec2)>,
        path: Vec<Vec2>,
    }
    let panels = [
        Panel {
            name: format!("DIJKSTRA {} NODES", dj.expanded),
            cells: dj.expanded_cells.clone(),
            edges: vec![],
            path: dj.points.clone(),
        },
        Panel {
            name: format!("A* {} NODES", a.expanded),
            cells: a.expanded_cells.clone(),
            edges: vec![],
            path: a.points.clone(),
        },
        Panel {
            name: format!("RRT {:.1} M", rrt.cost),
            cells: vec![],
            edges: rrt.trees[0].edges(),
            path: rrt.path.clone(),
        },
        Panel {
            name: format!("RRT* {:.1} M", rs.cost),
            cells: vec![],
            edges: rs.trees[0].edges(),
            path: rs.path.clone(),
        },
    ];
    let trajs: Vec<Vec<TrajPoint>> = panels
        .iter()
        .map(|p| traj_of(&grid::prune_line_of_sight(&sc.map, &p.path, r), 2.5))
        .collect();
    let (w, h) = (840u32, 560u32);
    let mut gif = GifWriter::create(format!("{OUT}/goal_reaching.gif"), w, h)?;
    let search_frames = 45;
    let t_end = trajs.iter().map(|t| t.last().unwrap().t_s).fold(0.0, f64::max);
    let drive_frames = 70;
    for f in 0..(search_frames + drive_frames + 15) {
        let mut c = Canvas::new(w, h);
        title(&mut c, "GOAL REACHING: SEARCH, THEN DRIVE");
        for (k, p) in panels.iter().enumerate() {
            let (px, py) = ((k % 2) as f32 * 420.0, 34.0 + (k / 2) as f32 * 263.0);
            let v = View::fit(sc.map.min, sc.map.max, px + 6.0, py + 18.0, 408.0, 240.0);
            c.map(&v, &sc.map);
            let frac = ((f as f64 + 1.0) / search_frames as f64).min(1.0);
            let nc = (p.cells.len() as f64 * frac) as usize;
            for cell in &p.cells[..nc] {
                c.dot(&v, *cell, 1.6, Rgba::rgb(158, 202, 225));
            }
            let ne = (p.edges.len() as f64 * frac) as usize;
            c.segments(&v, &p.edges[..ne], colors::GREY, 1.0);
            if f >= search_frames {
                c.polyline(&v, &p.path, colors::series(k), 2.5);
                let t = (f - search_frames) as f64 / drive_frames as f64 * t_end;
                c.polyline(&v, &trail(&trajs[k], t), colors::INK, 1.5);
                c.disc_robot(&v, pose_at(&trajs[k], t), r, colors::series(k));
            }
            c.dot(&v, s, 4.0, colors::GREEN);
            c.dot(&v, goal, 4.0, colors::RED);
            c.text(px + 8.0, py + 4.0, &p.name, 1.5, colors::INK);
        }
        gif.frame(&c, DELAY)?;
    }
    Ok(())
}

/// Charging: compare chargers by path cost, drive, dock aligned, charge.
fn charging() -> std::io::Result<()> {
    let sc = scenes::warehouse();
    let r = 0.35;
    let docks = [
        Pose2::new(19.55, 1.5, 0.0),
        Pose2::new(1.5, 11.55, std::f64::consts::FRAC_PI_2),
    ];
    let start = Vec2::new(9.0, 2.0);
    let plan = docking::plan_dock(&sc.map, start, &docks, r, 1.2).expect("dock plan");
    let g = OccupancyGrid::from_map(&sc.map, 0.1, r);
    let alts: Vec<Vec<Vec2>> = docks
        .iter()
        .map(|d| {
            grid::prune_line_of_sight(
                &sc.map,
                &grid::astar(&g, start, docking::pre_dock_pose(*d, 1.2).pos())
                    .unwrap()
                    .points,
                r,
            )
        })
        .collect();
    let tr = trajectory::time_parameterize(
        &plan.path,
        &SpeedLimits {
            v_max_mps: 2.0,
            a_max_mps2: 1.0,
            a_lat_max_mps2: 1.0,
        },
    );
    let t_end = tr.last().unwrap().t_s;
    let (w, h) = (800u32, 520u32);
    let mut gif = GifWriter::create(format!("{OUT}/charging.gif"), w, h)?;
    let v = View::fit(sc.map.min, sc.map.max, 10.0, 40.0, 780.0, 470.0);
    let (n_drive, n_charge) = (90, 30);
    for f in 0..(10 + n_drive + n_charge) {
        let mut c = Canvas::new(w, h);
        title(&mut c, "CHARGING: PICK CHEAPEST CHARGER, DOCK ALIGNED");
        c.map(&v, &sc.map);
        for (i, d) in docks.iter().enumerate() {
            c.oriented_box(
                &v,
                Pose2::new(
                    d.x + 0.35 * d.theta_rad.cos(),
                    d.y + 0.35 * d.theta_rad.sin(),
                    d.theta_rad,
                ),
                0.3,
                1.0,
                colors::GREEN,
            );
            let (lx, ly) = v.px(d.pos());
            let cost = plan.costs_m[i];
            c.label(
                lx - if i == 0 { 120.0 } else { -10.0 },
                ly - if i == 0 { 40.0 } else { 0.0 },
                &format!("{cost:.1} M"),
                1.5,
                if i == plan.charger { colors::GREEN } else { colors::GREY },
            );
            c.polyline(
                &v,
                &alts[i],
                if i == plan.charger {
                    colors::GREEN.alpha(90)
                } else {
                    colors::GREY
                },
                2.0,
            );
        }
        let pts: Vec<Vec2> = plan.path.iter().map(|p| Vec2::new(p.x, p.y)).collect();
        c.polyline(&v, &pts, colors::GREEN, 2.5);
        c.arrow(&v, plan.pre_dock, 0.8, colors::PURPLE, 2.0);
        let t = ((f as f64 - 10.0).max(0.0) / n_drive as f64 * t_end).min(t_end);
        let battery = if f < 10 + n_drive {
            0.18 - 0.08 * t / t_end
        } else {
            0.10 + 0.9 * (f - 10 - n_drive) as f64 / n_charge as f64
        };
        c.polyline(&v, &trail(&tr, t), colors::INK, 1.5);
        c.disc_robot(&v, pose_at(&tr, t), r, colors::BLUE);
        // battery gauge
        let (bx, by) = (w as f32 - 190.0, 40.0);
        c.rect_px(bx, by, 170.0, 28.0, colors::WHITE);
        c.polyline_px(
            &[
                (bx, by),
                (bx + 170.0, by),
                (bx + 170.0, by + 28.0),
                (bx, by + 28.0),
                (bx, by),
            ],
            colors::INK,
            2.0,
        );
        let col = if battery < 0.2 { colors::RED } else { colors::GREEN };
        c.rect_px(bx + 3.0, by + 3.0, 164.0 * battery as f32, 22.0, col);
        c.text(
            bx + 55.0,
            by + 8.0,
            &format!("{:.0}%", battery * 100.0),
            2.0,
            colors::INK,
        );
        gif.frame(&c, DELAY)?;
    }
    Ok(())
}

/// Cleaning: boustrophedon coverage with a painted swath.
fn cleaning() -> std::io::Result<()> {
    let map = Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(12.0, 8.0))
        .rect(3.0, 2.5, 5.0, 5.0) // table
        .rect(8.5, 0.0, 12.0, 1.8) // kitchen counter
        .circle(9.0, 5.5, 0.6) // chair
        .rect(0.0, 6.8, 2.2, 8.0); // shelf
    let (r, tool) = (0.25, 0.5);
    let plan = coverage::boustrophedon(&map, Vec2::new(0.5, 0.5), r, tool);
    let ratio = coverage::coverage_ratio(&map, &plan, r, tool, 0.1);
    // constant-speed walk along the path
    let mut cum = vec![0.0];
    for w in plan.path.windows(2) {
        cum.push(cum.last().unwrap() + w[0].dist(w[1]));
    }
    let total = *cum.last().unwrap();
    let at = |s: f64| -> (Vec2, usize) {
        let i = cum.partition_point(|c| *c <= s).clamp(1, cum.len() - 1);
        let u = ((s - cum[i - 1]) / (cum[i] - cum[i - 1]).max(1e-9)).clamp(0.0, 1.0);
        (plan.path[i - 1].lerp(plan.path[i], u), i)
    };
    let (w, h) = (760u32, 560u32);
    let mut gif = GifWriter::create(format!("{OUT}/cleaning.gif"), w, h)?;
    let v = View::fit(map.min, map.max, 10.0, 40.0, 740.0, 510.0);
    let n = 140;
    for f in 0..=n + 15 {
        let s = total * (f.min(n) as f64 / n as f64);
        let (pos, idx) = at(s);
        let mut c = Canvas::new(w, h);
        title(
            &mut c,
            &format!("CLEANING: BOUSTROPHEDON COVERAGE {:.0}%", ratio * 100.0),
        );
        c.map(&v, &map);
        let swath = tool as f32 * v.scale();
        for k in 1..idx {
            if plan.sweeping[k] {
                c.polyline(&v, &[plan.path[k - 1], plan.path[k]], Rgba::rgb(160, 210, 240), swath);
            }
        }
        if plan.sweeping[idx] {
            c.polyline(&v, &[plan.path[idx - 1], pos], Rgba::rgb(160, 210, 240), swath);
        }
        c.map_obstacles_only(&v, &map);
        c.polyline(&v, &plan.path[..idx], colors::BLUE.alpha(120), 1.0);
        let heading = plan.path[idx] - plan.path[idx - 1];
        c.disc_robot(
            &v,
            Pose2::new(pos.x, pos.y, heading.y.atan2(heading.x)),
            r,
            colors::BLUE,
        );
        gif.frame(&c, DELAY)?;
    }
    Ok(())
}

/// Patrol: optimised visiting order over A* costs.
fn patrol() -> std::io::Result<()> {
    let sc = scenes::warehouse();
    let r = 0.35;
    let goals = [
        Vec2::new(18.8, 10.8),
        Vec2::new(2.5, 10.5),
        Vec2::new(12.3, 2.0),
        Vec2::new(8.7, 10.5),
        Vec2::new(5.2, 2.0),
        Vec2::new(18.8, 2.0),
    ];
    let start = Vec2::new(1.0, 1.0);
    let t = tour::plan_tour(&sc.map, start, &goals, r, true).expect("tour");
    let tr = traj_of(&t.path, 2.5);
    let t_end = tr.last().unwrap().t_s;
    // arrival time per goal: first time the trajectory gets close to it
    let arrival: Vec<f64> = t
        .order
        .iter()
        .map(|&k| {
            tr.iter()
                .find(|p| Vec2::new(p.p.x, p.p.y).dist(goals[k]) < 0.3)
                .map_or(t_end, |p| p.t_s)
        })
        .collect();
    let (w, h) = (800u32, 520u32);
    let mut gif = GifWriter::create(format!("{OUT}/patrol.gif"), w, h)?;
    let v = View::fit(sc.map.min, sc.map.max, 10.0, 40.0, 780.0, 470.0);
    let n = 120;
    for f in 0..=n + 10 {
        let time = t_end * f.min(n) as f64 / n as f64;
        let mut c = Canvas::new(w, h);
        title(&mut c, &format!("PATROL: NN + 2-OPT TOUR, {:.1} M", t.cost_m));
        c.map(&v, &sc.map);
        c.polyline(&v, &t.path, colors::GREY, 2.0);
        c.polyline(&v, &trail(&tr, time), colors::ORANGE, 2.5);
        for (rank, &k) in t.order.iter().enumerate() {
            let done = time >= arrival[rank];
            let (x, y) = v.px(goals[k]);
            c.circle_px(x, y, 11.0, if done { colors::GREEN } else { colors::RED });
            c.text(x - 4.0, y - 6.0, &format!("{}", rank + 1), 1.8, colors::WHITE);
        }
        c.disc_robot(&v, pose_at(&tr, time), r, colors::BLUE);
        gif.frame(&c, DELAY)?;
    }
    Ok(())
}

/// Following a moving target with periodic A* replanning.
fn follow() -> std::io::Result<()> {
    let sc = scenes::warehouse();
    let r = 0.35;
    let g = OccupancyGrid::from_map(&sc.map, 0.1, r);
    // the target walks a loop through the aisles
    let loop_pts = [
        Vec2::new(5.8, 9.0),
        Vec2::new(5.8, 1.0),
        Vec2::new(9.0, 1.0),
        Vec2::new(9.0, 11.0),
        Vec2::new(12.5, 11.0),
        Vec2::new(12.5, 1.0),
        Vec2::new(16.0, 1.0),
        Vec2::new(16.0, 10.5),
    ];
    let target_tr = traj_of(&loop_pts, 1.1);
    let t_end = target_tr.last().unwrap().t_s;
    let dt = 0.1;
    let mut robot = Pose2::new(1.0, 1.0, 0.0);
    let mut plan: Vec<Vec2> = vec![];
    let mut robot_trail = vec![robot.pos()];
    let (w, h) = (800u32, 520u32);
    let mut gif = GifWriter::create(format!("{OUT}/follow.gif"), w, h)?;
    let v = View::fit(sc.map.min, sc.map.max, 10.0, 40.0, 780.0, 470.0);
    let steps = (t_end / dt) as usize + 30;
    let mut replans = 0;
    for k in 0..steps {
        let t = k as f64 * dt;
        let target = pose_at(&target_tr, t).pos();
        if k % 5 == 0 {
            if let Some(p) = grid::astar(&g, robot.pos(), target) {
                plan = grid::prune_line_of_sight(&sc.map, &p.points, r);
                replans += 1;
            }
        }
        // move along the plan at up to 1.4 m/s, stopping 1 m short of the target
        let mut budget = if path_length(&plan) > 1.0 { 1.4 * dt } else { 0.0 };
        while budget > 0.0 && plan.len() >= 2 {
            let d = plan[0].dist(plan[1]);
            if d <= budget {
                budget -= d;
                plan.remove(0);
            } else {
                plan[0] = plan[0].lerp(plan[1], budget / d);
                budget = 0.0;
            }
        }
        if let Some(p) = plan.first() {
            let heading = plan.get(1).map_or(robot.theta_rad, |n| (n.y - p.y).atan2(n.x - p.x));
            robot = Pose2::new(p.x, p.y, heading);
        }
        robot_trail.push(robot.pos());
        if k % 2 == 0 {
            let mut c = Canvas::new(w, h);
            title(
                &mut c,
                &format!("FOLLOW A MOVING TARGET: A* REPLAN AT 2 HZ ({replans})"),
            );
            c.map(&v, &sc.map);
            c.polyline(&v, &trail(&target_tr, t), colors::RED.alpha(100), 2.0);
            c.polyline(&v, &robot_trail, colors::BLUE.alpha(120), 2.0);
            c.polyline(&v, &plan, colors::GREEN, 2.5);
            c.disc(&v, target, 0.3, colors::RED);
            c.disc_robot(&v, robot, r, colors::BLUE);
            gif.frame(&c, DELAY)?;
        }
    }
    Ok(())
}

/// Parking: Hybrid A* search, then the car drives into the bay.
fn parking() -> std::io::Result<()> {
    let pk = scenes::parking_lot();
    let cfg = hybrid_astar::HybridAStarConfig {
        min_turn_radius_m: 4.0,
        footprint_radius_m: 1.1,
        ..Default::default()
    };
    let res = hybrid_astar::plan(&pk.map, pk.start, pk.goal, &cfg).expect("hybrid A*");
    let path = trajectory::poses_to_path(&res.poses);
    let tr = trajectory::time_parameterize(
        &path,
        &SpeedLimits {
            v_max_mps: 4.0,
            a_max_mps2: 1.5,
            a_lat_max_mps2: 1.5,
        },
    );
    let t_end = tr.last().unwrap().t_s;
    let (w, h) = (800u32, 520u32);
    let mut gif = GifWriter::create(format!("{OUT}/parking.gif"), w, h)?;
    let v = View::fit(pk.map.min, pk.map.max, 10.0, 40.0, 780.0, 470.0);
    let (ns, nd) = (25, 80);
    let pts: Vec<Vec2> = res.poses.iter().map(|p| p.pos()).collect();
    for f in 0..(ns + nd + 12) {
        let mut c = Canvas::new(w, h);
        title(
            &mut c,
            &format!("PARKING: HYBRID A* (R_MIN 4 M), {} EXPANSIONS", res.expanded),
        );
        c.map(&v, &pk.map);
        let ne = (res.explored.len() * (f + 1).min(ns) / ns).min(res.explored.len());
        for e in &res.explored[..ne] {
            c.dot(&v, *e, 2.0, colors::GREY);
        }
        c.arrow(&v, pk.goal, 3.0, colors::RED, 2.5);
        if f >= ns {
            c.polyline(&v, &pts, colors::PURPLE, 2.5);
            let t = ((f - ns) as f64 / nd as f64 * t_end).min(t_end);
            let p = pose_at(&tr, t);
            // body centred 1.35 m ahead of the rear axle
            let centre = Pose2::new(
                p.x + 1.35 * p.theta_rad.cos(),
                p.y + 1.35 * p.theta_rad.sin(),
                p.theta_rad,
            );
            c.oriented_box(&v, centre, 4.4, 1.8, colors::BLUE);
            c.arrow(&v, p, 2.5, colors::WHITE, 2.0);
        }
        gif.frame(&c, DELAY)?;
    }
    Ok(())
}

/// Pick and place with a 3-link arm: collision-aware IK + joint-space RRT-Connect.
fn pick_place() -> std::io::Result<()> {
    let (arm, map, pick, place) = scenes::arm_cell();
    let sp = ArmSpace {
        arm: &arm,
        map: &map,
        link_margin_m: 0.06,
    };
    let home = sp.ik_valid(Vec2::new(0.9, 0.6), 64, 3).unwrap();
    let qa = sp.ik_valid(pick, 64, 1).unwrap();
    let qb = sp.ik_valid(place, 64, 2).unwrap();
    let cfg = SamplingConfig {
        step: 0.15,
        max_iters: 20000,
        ..Default::default()
    };
    let mut legs = Vec::new();
    for (a, b) in [(&home, &qa), (&qa, &qb), (&qb, &home)] {
        let r = sampling::rrt_connect(&sp, a, b, &cfg);
        legs.push(trajectory::JointTrajectory::new(
            sampling::shortcut(&sp, &r.path, 300, 9),
            1.5,
            3.0,
        ));
    }
    let (w, h) = (720u32, 520u32);
    let mut gif = GifWriter::create(format!("{OUT}/pick_place.gif"), w, h)?;
    let v = View::fit(map.min, map.max, 10.0, 40.0, 700.0, 470.0);
    let mut ee_trace = Vec::new();
    for (li, leg) in legs.iter().enumerate() {
        let n = 45;
        for k in 0..=n {
            let (q, _, _) = leg.sample(leg.duration_s() * k as f64 / n as f64);
            let pts = arm.fk(&q);
            let ee = *pts.last().unwrap();
            ee_trace.push(ee);
            let mut c = Canvas::new(w, h);
            title(
                &mut c,
                ["PICK: HOME -> PART", "PLACE: CARRY AROUND THE PILLAR", "RETURN HOME"][li],
            );
            c.map(&v, &map);
            c.polyline(&v, &ee_trace, colors::RED.alpha(140), 1.5);
            let box_at = |c: &mut Canvas, p: Vec2| {
                c.oriented_box(&v, Pose2::new(p.x, p.y - 0.08, 0.0), 0.2, 0.16, colors::ORANGE)
            };
            match li {
                0 => box_at(&mut c, pick),
                1 => box_at(&mut c, ee),
                _ => box_at(&mut c, place),
            }
            c.polyline(&v, &pts, colors::BLUE, 9.0);
            for p in &pts {
                c.dot(&v, *p, 5.0, colors::INK);
            }
            c.dot(&v, pick, 4.0, colors::GREEN);
            c.dot(&v, place, 4.0, colors::RED);
            gif.frame(&c, DELAY)?;
        }
    }
    Ok(())
}

/// Parallel parking: Hybrid A* with reversing (Reeds-Shepp shots).
fn parallel_parking() -> std::io::Result<()> {
    let map = Map2D::new(Vec2::new(0.0, -1.0), Vec2::new(30.0, 12.0))
        .rect(0.0, -1.0, 30.0, 0.2)
        .rect(3.0, 0.2, 7.4, 2.2)
        .rect(8.0, 0.2, 12.4, 2.2)
        .rect(18.6, 0.2, 23.0, 2.2)
        .rect(23.6, 0.2, 28.0, 2.2)
        .rect(0.0, 10.0, 30.0, 12.0);
    let cfg = hybrid_astar::HybridAStarConfig {
        min_turn_radius_m: 4.0,
        footprint_radius_m: 0.95,
        footprint_front_offset_m: 2.0,
        allow_reverse: true,
        xy_resolution_m: 0.3,
        ..Default::default()
    };
    let start = Pose2::new(4.0, 6.0, 0.0);
    let goal = Pose2::new(14.2, 1.2, 0.0);
    let res = hybrid_astar::plan(&map, start, goal, &cfg).expect("parking plan");
    let (w, h) = (840u32, 440u32);
    let mut gif = GifWriter::create(format!("{OUT}/parallel_parking.gif"), w, h)?;
    let v = View::fit(map.min, map.max, 10.0, 40.0, 820.0, 390.0);
    let pts: Vec<Vec2> = res.poses.iter().map(|p| p.pos()).collect();
    let cusps = res.directions.windows(2).filter(|w| w[0] != w[1]).count();
    let n = res.poses.len();
    let frames = 110;
    for f in 0..(frames + 15) {
        let k = (n - 1) * f.min(frames) / frames;
        let mut c = Canvas::new(w, h);
        title(
            &mut c,
            &format!("PARALLEL PARKING: HYBRID A* + REEDS-SHEPP, {cusps} CUSPS"),
        );
        c.map(&v, &map);
        for (i, w2) in pts.windows(2).enumerate() {
            let col = if res.directions[i] < 0.0 {
                colors::RED
            } else {
                colors::BLUE
            };
            c.polyline(&v, w2, col.alpha(160), 2.0);
        }
        c.arrow(&v, goal, 2.5, colors::GREEN, 2.5);
        let p = res.poses[k];
        let centre = Pose2::new(
            p.x + 1.0 * p.theta_rad.cos(),
            p.y + 1.0 * p.theta_rad.sin(),
            p.theta_rad,
        );
        let reversing = res.directions[k] < 0.0;
        c.oriented_box(&v, centre, 4.0, 1.8, if reversing { colors::RED } else { colors::BLUE });
        c.arrow(&v, p, 2.0, colors::WHITE, 2.0);
        c.label(
            w as f32 - 200.0,
            44.0,
            if reversing { "REVERSE" } else { "FORWARD" },
            2.5,
            if reversing { colors::RED } else { colors::BLUE },
        );
        gif.frame(&c, DELAY)?;
    }
    Ok(())
}

/// Ten robots swap places across a circle with reciprocal avoidance.
fn multi_robot_rvo() -> std::io::Result<()> {
    use robo_planning::multi_robot::{Agent, RvoConfig, RvoSim};
    let n = 10;
    let map = Map2D::new(Vec2::new(0.0, 0.0), Vec2::new(12.0, 12.0)).circle(6.0, 6.0, 0.8);
    let agents = (0..n)
        .map(|i| {
            let th = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
            let p = Vec2::new(6.0 + 5.0 * th.cos(), 6.0 + 5.0 * th.sin());
            Agent {
                pos: p,
                vel: Vec2::default(),
                goal: Vec2::new(12.0, 12.0) - p,
                radius_m: 0.35,
                v_max_mps: 1.2,
            }
        })
        .collect();
    let mut sim = RvoSim {
        agents,
        map: map.clone(),
        cfg: RvoConfig::default(),
    };
    let starts: Vec<Vec2> = sim.agents.iter().map(|a| a.pos).collect();
    let mut trails: Vec<Vec<Vec2>> = starts.iter().map(|p| vec![*p]).collect();
    let (w, h) = (560u32, 600u32);
    let mut gif = GifWriter::create(format!("{OUT}/multi_robot_rvo.gif"), w, h)?;
    let v = View::fit(map.min, map.max, 10.0, 40.0, 540.0, 550.0);
    let mut min_gap = f64::INFINITY;
    for k in 0..900 {
        min_gap = min_gap.min(sim.step());
        for (t, a) in trails.iter_mut().zip(&sim.agents) {
            t.push(a.pos);
        }
        let done = sim.arrived(0.05);
        if k % 4 == 0 || done {
            let mut c = Canvas::new(w, h);
            title(
                &mut c,
                &format!("10 ROBOTS SWAP: RECIPROCAL AVOIDANCE, GAP {:.2} M", min_gap.max(0.0)),
            );
            c.map(&v, &map);
            for (i, (t, a)) in trails.iter().zip(&sim.agents).enumerate() {
                c.polyline(&v, t, colors::series(i).alpha(120), 1.5);
                c.ring_px(
                    v.px(a.goal).0,
                    v.px(a.goal).1,
                    a.radius_m as f32 * v.scale(),
                    colors::series(i),
                    1.5,
                );
                c.disc_robot(
                    &v,
                    Pose2::new(a.pos.x, a.pos.y, a.vel.y.atan2(a.vel.x)),
                    a.radius_m,
                    colors::series(i),
                );
            }
            gif.frame(&c, DELAY)?;
        }
        if done {
            break;
        }
    }
    Ok(())
}

/// Four AMRs cross a warehouse with prioritized space-time A*.
fn multi_robot_prioritized() -> std::io::Result<()> {
    use robo_planning::multi_robot::prioritized_plan;
    let sc = scenes::warehouse();
    let starts = [
        Vec2::new(1.25, 1.25),
        Vec2::new(18.75, 10.75),
        Vec2::new(1.25, 10.75),
        Vec2::new(18.75, 1.25),
    ];
    let goals = [
        Vec2::new(18.75, 10.75),
        Vec2::new(1.25, 1.25),
        Vec2::new(18.75, 1.25),
        Vec2::new(1.25, 10.75),
    ];
    let plans = prioritized_plan(&sc.map, &starts, &goals, 0.3, 0.5, 0.5, 400).expect("plans");
    let t_end = plans.iter().map(|p| p.points.len()).max().unwrap();
    let (w, h) = (800u32, 520u32);
    let mut gif = GifWriter::create(format!("{OUT}/multi_robot_prioritized.gif"), w, h)?;
    let v = View::fit(sc.map.min, sc.map.max, 10.0, 40.0, 780.0, 470.0);
    let sub = 3; // interpolated frames per grid step
    for f in 0..(t_end * sub + 12) {
        let tf = (f as f64 / sub as f64).min((t_end - 1) as f64);
        let mut c = Canvas::new(w, h);
        title(&mut c, "4 AMRS: PRIORITIZED SPACE-TIME A* (ORDER 1-4)");
        c.map(&v, &sc.map);
        for (i, p) in plans.iter().enumerate() {
            c.polyline(&v, &p.points, colors::series(i).alpha(90), 2.0);
            let k = (tf as usize).min(p.points.len() - 1);
            let k2 = (k + 1).min(p.points.len() - 1);
            let u = tf - tf.floor();
            let pos = p.points[k].lerp(p.points[k2], if k2 > k { u } else { 0.0 });
            c.dot(&v, goals[i], 6.0, colors::series(i));
            c.disc(&v, pos, 0.3, colors::series(i));
            let (x, y) = v.px(pos);
            c.text(x - 4.0, y - 6.0, &format!("{}", i + 1), 1.6, colors::WHITE);
        }
        c.text(
            w as f32 - 150.0,
            46.0,
            &format!("STEP {:3}", tf as usize),
            1.6,
            colors::INK,
        );
        gif.frame(&c, DELAY)?;
    }
    Ok(())
}

fn main() -> std::io::Result<()> {
    type Job = fn() -> std::io::Result<()>;
    let jobs: [(&str, Job); 10] = [
        ("goal_reaching", goal_reaching),
        ("charging", charging),
        ("cleaning", cleaning),
        ("patrol", patrol),
        ("follow", follow),
        ("parking", parking),
        ("pick_place", pick_place),
        ("parallel_parking", parallel_parking),
        ("multi_robot_rvo", multi_robot_rvo),
        ("multi_robot_prioritized", multi_robot_prioritized),
    ];
    let only: Vec<String> = std::env::args().skip(1).collect();
    for (name, job) in jobs {
        if only.is_empty() || only.iter().any(|o| o == name) {
            job()?;
            println!("wrote {OUT}/{name}.gif");
        }
    }
    Ok(())
}
