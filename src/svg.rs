//! Minimal SVG writer for planning figures (maps, paths, trees, animated robots).

use crate::geometry::{Map2D, Obstacle, Pose2, Vec2};
use std::fmt::Write as _;

/// SVG canvas in world coordinates (y up).
#[derive(Debug, Clone)]
pub struct Svg {
    min: Vec2,
    max: Vec2,
    scale: f64,
    body: String,
    title: String,
}

impl Svg {
    /// Canvas covering `[min, max]` at `px_per_m`.
    pub fn new(min: Vec2, max: Vec2, px_per_m: f64) -> Self {
        Self {
            min,
            max,
            scale: px_per_m,
            body: String::new(),
            title: String::new(),
        }
    }
    /// Canvas covering a map.
    pub fn for_map(map: &Map2D, px_per_m: f64) -> Self {
        Self::new(map.min, map.max, px_per_m)
    }
    fn px(&self, p: Vec2) -> (f64, f64) {
        ((p.x - self.min.x) * self.scale, (self.max.y - p.y) * self.scale)
    }
    /// Sets a title drawn in the top-left corner.
    pub fn title(&mut self, t: &str) -> &mut Self {
        self.title = t.to_string();
        self
    }
    /// Draws the map border and obstacles.
    pub fn map(&mut self, map: &Map2D) -> &mut Self {
        let (x0, y0) = self.px(Vec2::new(map.min.x, map.max.y));
        let (x1, y1) = self.px(Vec2::new(map.max.x, map.min.y));
        let _ = writeln!(
            self.body,
            r##"<rect x="{x0:.1}" y="{y0:.1}" width="{:.1}" height="{:.1}" fill="#fafafa" stroke="#333"/>"##,
            x1 - x0,
            y1 - y0
        );
        for o in &map.obstacles {
            match *o {
                Obstacle::Circle { center, radius_m } => {
                    let (cx, cy) = self.px(center);
                    let _ = writeln!(
                        self.body,
                        r##"<circle cx="{cx:.1}" cy="{cy:.1}" r="{:.1}" fill="#555"/>"##,
                        radius_m * self.scale
                    );
                }
                Obstacle::Rect { min, max } => {
                    let (ax, ay) = self.px(Vec2::new(min.x, max.y));
                    let (bx, by) = self.px(Vec2::new(max.x, min.y));
                    let _ = writeln!(
                        self.body,
                        r##"<rect x="{ax:.1}" y="{ay:.1}" width="{:.1}" height="{:.1}" fill="#555"/>"##,
                        bx - ax,
                        by - ay
                    );
                }
            }
        }
        self
    }
    /// Polyline with colour, stroke width [px] and optional dash.
    pub fn polyline(&mut self, pts: &[Vec2], color: &str, width_px: f64, dashed: bool) -> &mut Self {
        if pts.is_empty() {
            return self;
        }
        let d: Vec<String> = pts
            .iter()
            .map(|p| {
                let (x, y) = self.px(*p);
                format!("{x:.1},{y:.1}")
            })
            .collect();
        let dash = if dashed { r#" stroke-dasharray="6,4""# } else { "" };
        let _ = writeln!(
            self.body,
            r#"<polyline points="{}" fill="none" stroke="{color}" stroke-width="{width_px}"{dash} stroke-linejoin="round"/>"#,
            d.join(" ")
        );
        self
    }
    /// Thin line segments (e.g. tree edges).
    pub fn segments(&mut self, segs: &[(Vec2, Vec2)], color: &str, width_px: f64) -> &mut Self {
        let mut d = String::new();
        for (a, b) in segs {
            let (ax, ay) = self.px(*a);
            let (bx, by) = self.px(*b);
            let _ = write!(d, "M{ax:.1},{ay:.1}L{bx:.1},{by:.1}");
        }
        let _ = writeln!(
            self.body,
            r#"<path d="{d}" stroke="{color}" stroke-width="{width_px}" fill="none" opacity="0.6"/>"#
        );
        self
    }
    /// Filled dot with radius in px.
    pub fn dot(&mut self, p: Vec2, r_px: f64, color: &str) -> &mut Self {
        let (x, y) = self.px(p);
        let _ = writeln!(
            self.body,
            r#"<circle cx="{x:.1}" cy="{y:.1}" r="{r_px}" fill="{color}"/>"#
        );
        self
    }
    /// Pose arrow of length `len_m`.
    pub fn pose(&mut self, p: Pose2, len_m: f64, color: &str) -> &mut Self {
        let tip = p.pos() + Vec2::new(p.theta_rad.cos(), p.theta_rad.sin()) * len_m;
        self.polyline(&[p.pos(), tip], color, 3.0, false)
            .dot(p.pos(), 4.0, color)
    }
    /// Text label at a world point.
    pub fn text(&mut self, p: Vec2, s: &str, color: &str) -> &mut Self {
        let (x, y) = self.px(p);
        let _ = writeln!(
            self.body,
            r#"<text x="{x:.1}" y="{y:.1}" font-family="sans-serif" font-size="13" fill="{color}">{s}</text>"#
        );
        self
    }
    /// Legend entry `index` in the top-right corner.
    pub fn legend(&mut self, index: usize, label: &str, color: &str) -> &mut Self {
        let w = (self.max.x - self.min.x) * self.scale;
        let y = 18.0 + 18.0 * index as f64;
        let _ = writeln!(
            self.body,
            r#"<rect x="{:.1}" y="{:.1}" width="14" height="4" fill="{color}"/><text x="{:.1}" y="{:.1}" font-family="sans-serif" font-size="12">{label}</text>"#,
            w - 170.0,
            y - 4.0,
            w - 150.0,
            y + 1.0
        );
        self
    }
    /// Disc of radius `r_m` that moves along `pts` over `duration_s`, looping.
    pub fn animate_disc(&mut self, pts: &[Vec2], r_m: f64, color: &str, duration_s: f64) -> &mut Self {
        if pts.len() < 2 {
            return self;
        }
        let d: Vec<String> = pts
            .iter()
            .map(|p| {
                let (x, y) = self.px(*p);
                format!("{x:.1},{y:.1}")
            })
            .collect();
        let _ = writeln!(
            self.body,
            r#"<circle r="{:.1}" fill="{color}" opacity="0.85"><animateMotion dur="{duration_s:.2}s" repeatCount="indefinite" path="M{}"/></circle>"#,
            (r_m * self.scale).max(3.0),
            d.join(" L")
        );
        self
    }
    /// Renders the document.
    pub fn render(&self) -> String {
        let w = (self.max.x - self.min.x) * self.scale;
        let h = (self.max.y - self.min.y) * self.scale;
        let title = if self.title.is_empty() {
            String::new()
        } else {
            format!(
                r#"<text x="8" y="18" font-family="sans-serif" font-size="15" font-weight="bold">{}</text>"#,
                self.title
            )
        };
        format!(
            r#"<svg xmlns="http://www.w3.org/2000/svg" width="{w:.0}" height="{h:.0}" viewBox="0 0 {w:.0} {h:.0}">
<rect width="100%" height="100%" fill="white"/>
{}{title}
</svg>
"#,
            self.body
        )
    }
    /// Writes the document to a file, creating parent directories.
    pub fn save(&self, path: impl AsRef<std::path::Path>) -> std::io::Result<()> {
        if let Some(parent) = path.as_ref().parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(path, self.render())
    }
}
