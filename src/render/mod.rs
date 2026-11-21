//! Raster rendering and animated GIF output (feature `render`).
//!
//! A [`Canvas`] is an RGBA image; a [`View`] maps a world rectangle (y up) onto a
//! pixel rectangle of it, so one frame can hold several panels. [`GifWriter`]
//! quantises frames to a fixed 216-colour palette, which keeps output
//! deterministic and fast.

mod font;

use crate::geometry::{Map2D, Obstacle, Pose2, Vec2};
use tiny_skia::{FillRule, Paint, PathBuilder, Pixmap, Rect, Stroke, Transform};

/// RGBA colour.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rgba(pub u8, pub u8, pub u8, pub u8);

impl Rgba {
    /// Opaque colour.
    pub const fn rgb(r: u8, g: u8, b: u8) -> Self {
        Self(r, g, b, 255)
    }
    /// Same colour with alpha.
    pub const fn alpha(self, a: u8) -> Self {
        Self(self.0, self.1, self.2, a)
    }
}

/// A small categorical palette.
pub mod colors {
    use super::Rgba;
    /// background
    pub const BG: Rgba = Rgba::rgb(250, 250, 250);
    /// white
    pub const WHITE: Rgba = Rgba::rgb(255, 255, 255);
    /// text / outlines
    pub const INK: Rgba = Rgba::rgb(34, 34, 34);
    /// obstacles
    pub const OBSTACLE: Rgba = Rgba::rgb(85, 85, 85);
    /// light grey
    pub const GREY: Rgba = Rgba::rgb(190, 190, 190);
    /// series colours
    pub const BLUE: Rgba = Rgba::rgb(31, 119, 180);
    /// series colours
    pub const ORANGE: Rgba = Rgba::rgb(255, 127, 14);
    /// series colours
    pub const GREEN: Rgba = Rgba::rgb(44, 160, 44);
    /// series colours
    pub const RED: Rgba = Rgba::rgb(214, 39, 40);
    /// series colours
    pub const PURPLE: Rgba = Rgba::rgb(148, 103, 189);
    /// series colours
    pub const BROWN: Rgba = Rgba::rgb(140, 86, 75);
    /// series colours
    pub const PINK: Rgba = Rgba::rgb(227, 119, 194);
    /// series colours
    pub const TEAL: Rgba = Rgba::rgb(23, 190, 207);
    /// Series colour `i` (cycles).
    pub fn series(i: usize) -> Rgba {
        [BLUE, ORANGE, GREEN, RED, PURPLE, BROWN, PINK, TEAL][i % 8]
    }
}

fn paint(c: Rgba) -> Paint<'static> {
    let mut p = Paint::default();
    p.set_color_rgba8(c.0, c.1, c.2, c.3);
    p.anti_alias = true;
    p
}

/// World-to-pixel mapping with uniform scale, centred in its pixel rectangle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct View {
    min: Vec2,
    max: Vec2,
    ox: f32,
    oy: f32,
    scale: f32,
}

impl View {
    /// Fits world rectangle `[min, max]` into pixel rectangle `(x, y, w, h)`.
    pub fn fit(min: Vec2, max: Vec2, x: f32, y: f32, w: f32, h: f32) -> Self {
        let sx = w / (max.x - min.x) as f32;
        let sy = h / (max.y - min.y) as f32;
        let scale = sx.min(sy);
        let ox = x + (w - scale * (max.x - min.x) as f32) / 2.0;
        let oy = y + (h - scale * (max.y - min.y) as f32) / 2.0;
        Self {
            min,
            max,
            ox,
            oy,
            scale,
        }
    }
    /// Pixels per world unit.
    pub fn scale(&self) -> f32 {
        self.scale
    }
    /// World point to pixel coordinates.
    pub fn px(&self, p: Vec2) -> (f32, f32) {
        (
            self.ox + (p.x - self.min.x) as f32 * self.scale,
            self.oy + (self.max.y - p.y) as f32 * self.scale,
        )
    }
}

/// RGBA raster image.
#[derive(Debug, Clone)]
pub struct Canvas {
    pixmap: Pixmap,
}

impl Canvas {
    /// New canvas filled with the background colour.
    pub fn new(width: u32, height: u32) -> Self {
        let mut c = Self {
            pixmap: Pixmap::new(width, height).expect("canvas size"),
        };
        c.clear(colors::BG);
        c
    }
    /// Width in px.
    pub fn width(&self) -> u32 {
        self.pixmap.width()
    }
    /// Height in px.
    pub fn height(&self) -> u32 {
        self.pixmap.height()
    }
    /// Raw RGBA bytes (premultiplied; opaque canvases are unaffected).
    pub fn data(&self) -> &[u8] {
        self.pixmap.data()
    }
    /// Fills the whole canvas.
    pub fn clear(&mut self, c: Rgba) {
        self.pixmap.fill(tiny_skia::Color::from_rgba8(c.0, c.1, c.2, c.3));
    }
    /// Filled pixel rectangle.
    pub fn rect_px(&mut self, x: f32, y: f32, w: f32, h: f32, c: Rgba) {
        if let Some(r) = Rect::from_xywh(x, y, w.max(0.5), h.max(0.5)) {
            self.pixmap.fill_rect(r, &paint(c), Transform::identity(), None);
        }
    }
    /// Polyline in pixel coordinates.
    pub fn polyline_px(&mut self, pts: &[(f32, f32)], c: Rgba, width_px: f32) {
        if pts.len() < 2 {
            return;
        }
        let mut pb = PathBuilder::new();
        pb.move_to(pts[0].0, pts[0].1);
        for p in &pts[1..] {
            pb.line_to(p.0, p.1);
        }
        if let Some(path) = pb.finish() {
            let st = Stroke {
                width: width_px,
                line_cap: tiny_skia::LineCap::Round,
                line_join: tiny_skia::LineJoin::Round,
                ..Default::default()
            };
            self.pixmap
                .stroke_path(&path, &paint(c), &st, Transform::identity(), None);
        }
    }
    /// Filled polygon in pixel coordinates.
    pub fn polygon_px(&mut self, pts: &[(f32, f32)], c: Rgba) {
        if pts.len() < 3 {
            return;
        }
        let mut pb = PathBuilder::new();
        pb.move_to(pts[0].0, pts[0].1);
        for p in &pts[1..] {
            pb.line_to(p.0, p.1);
        }
        pb.close();
        if let Some(path) = pb.finish() {
            self.pixmap
                .fill_path(&path, &paint(c), FillRule::Winding, Transform::identity(), None);
        }
    }
    /// Filled circle in pixel coordinates.
    pub fn circle_px(&mut self, x: f32, y: f32, r: f32, c: Rgba) {
        if let Some(path) = PathBuilder::from_circle(x, y, r.max(0.5)) {
            self.pixmap
                .fill_path(&path, &paint(c), FillRule::Winding, Transform::identity(), None);
        }
    }
    /// Circle outline in pixel coordinates.
    pub fn ring_px(&mut self, x: f32, y: f32, r: f32, c: Rgba, width_px: f32) {
        if let Some(path) = PathBuilder::from_circle(x, y, r.max(0.5)) {
            let st = Stroke {
                width: width_px,
                ..Default::default()
            };
            self.pixmap
                .stroke_path(&path, &paint(c), &st, Transform::identity(), None);
        }
    }
    /// Bitmap text with its top-left corner at `(x, y)`; `scale` 2 gives 10x14 px glyphs.
    pub fn text(&mut self, x: f32, y: f32, s: &str, scale: f32, c: Rgba) {
        let mut cx = x;
        for ch in s.chars() {
            let g = font::glyph(ch);
            for (row, bits) in g.iter().enumerate() {
                for col in 0..5 {
                    if bits & (0x10 >> col) != 0 {
                        self.rect_px(cx + col as f32 * scale, y + row as f32 * scale, scale, scale, c);
                    }
                }
            }
            cx += 6.0 * scale;
        }
    }
    /// Text width in px for [`Canvas::text`].
    pub fn text_width(s: &str, scale: f32) -> f32 {
        s.chars().count() as f32 * 6.0 * scale
    }
    /// Text on a translucent white box (labels over busy plots).
    pub fn label(&mut self, x: f32, y: f32, s: &str, scale: f32, c: Rgba) {
        self.rect_px(
            x - 3.0,
            y - 3.0,
            Self::text_width(s, scale) + 4.0,
            7.0 * scale + 6.0,
            colors::WHITE.alpha(220),
        );
        self.text(x, y, s, scale, c);
    }

    // ---- world-space helpers

    /// Draws a map: workspace, border and obstacles.
    pub fn map(&mut self, v: &View, map: &Map2D) {
        let (x0, y0) = v.px(Vec2::new(map.min.x, map.max.y));
        let (x1, y1) = v.px(Vec2::new(map.max.x, map.min.y));
        self.rect_px(x0, y0, x1 - x0, y1 - y0, colors::WHITE);
        for o in &map.obstacles {
            match *o {
                Obstacle::Circle { center, radius_m } => {
                    let (cx, cy) = v.px(center);
                    self.circle_px(cx, cy, radius_m as f32 * v.scale(), colors::OBSTACLE);
                }
                Obstacle::Rect { min, max } => {
                    let (ax, ay) = v.px(Vec2::new(min.x, max.y));
                    let (bx, by) = v.px(Vec2::new(max.x, min.y));
                    self.rect_px(ax, ay, bx - ax, by - ay, colors::OBSTACLE);
                }
            }
        }
        self.polyline_px(&[(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)], colors::INK, 1.5);
    }
    /// Draws only the obstacles of a map (to repaint them over other layers).
    pub fn map_obstacles_only(&mut self, v: &View, map: &Map2D) {
        let bare = Map2D {
            obstacles: map.obstacles.clone(),
            ..Map2D::new(map.min, map.max)
        };
        for o in &bare.obstacles {
            match *o {
                Obstacle::Circle { center, radius_m } => self.disc(v, center, radius_m, colors::OBSTACLE),
                Obstacle::Rect { min, max } => {
                    let (ax, ay) = v.px(Vec2::new(min.x, max.y));
                    let (bx, by) = v.px(Vec2::new(max.x, min.y));
                    self.rect_px(ax, ay, bx - ax, by - ay, colors::OBSTACLE);
                }
            }
        }
    }
    /// World polyline.
    pub fn polyline(&mut self, v: &View, pts: &[Vec2], c: Rgba, width_px: f32) {
        let p: Vec<(f32, f32)> = pts.iter().map(|p| v.px(*p)).collect();
        self.polyline_px(&p, c, width_px);
    }
    /// World segments (e.g. tree edges).
    pub fn segments(&mut self, v: &View, segs: &[(Vec2, Vec2)], c: Rgba, width_px: f32) {
        for (a, b) in segs {
            self.polyline_px(&[v.px(*a), v.px(*b)], c, width_px);
        }
    }
    /// World polygon.
    pub fn polygon(&mut self, v: &View, pts: &[Vec2], c: Rgba) {
        let p: Vec<(f32, f32)> = pts.iter().map(|p| v.px(*p)).collect();
        self.polygon_px(&p, c);
    }
    /// World disc of radius `r_m`.
    pub fn disc(&mut self, v: &View, p: Vec2, r_m: f64, c: Rgba) {
        let (x, y) = v.px(p);
        self.circle_px(x, y, r_m as f32 * v.scale(), c);
    }
    /// Dot of fixed pixel radius at a world point.
    pub fn dot(&mut self, v: &View, p: Vec2, r_px: f32, c: Rgba) {
        let (x, y) = v.px(p);
        self.circle_px(x, y, r_px, c);
    }
    /// Oriented box centred at `pose` (length along heading, width across), e.g. a car body.
    pub fn oriented_box(&mut self, v: &View, pose: Pose2, length_m: f64, width_m: f64, c: Rgba) {
        let (s, co) = pose.theta_rad.sin_cos();
        let corners = [(0.5, 0.5), (0.5, -0.5), (-0.5, -0.5), (-0.5, 0.5)].map(|(a, b)| {
            let (lx, ly) = (a * length_m, b * width_m);
            Vec2::new(pose.x + co * lx - s * ly, pose.y + s * lx + co * ly)
        });
        self.polygon(v, &corners, c);
    }
    /// Disc robot with a heading tick (differential drive).
    pub fn disc_robot(&mut self, v: &View, pose: Pose2, r_m: f64, c: Rgba) {
        self.disc(v, pose.pos(), r_m, c);
        let tip = pose.pos() + Vec2::new(pose.theta_rad.cos(), pose.theta_rad.sin()) * r_m;
        self.polyline(v, &[pose.pos(), tip], colors::WHITE, 2.0);
    }
    /// Pose arrow.
    pub fn arrow(&mut self, v: &View, pose: Pose2, len_m: f64, c: Rgba, width_px: f32) {
        let (s, co) = pose.theta_rad.sin_cos();
        let tip = pose.pos() + Vec2::new(co, s) * len_m;
        let back = len_m * 0.35;
        let l = tip + Vec2::new(-co * back - s * back * 0.6, -s * back + co * back * 0.6);
        let r = tip + Vec2::new(-co * back + s * back * 0.6, -s * back - co * back * 0.6);
        self.polyline(v, &[pose.pos(), tip], c, width_px);
        self.polygon(v, &[tip, l, r], c);
    }
}

/// Animated GIF writer.
pub struct GifWriter {
    encoder: gif::Encoder<std::io::BufWriter<std::fs::File>>,
    width: u16,
    height: u16,
    prev: Option<Vec<u8>>,
}

impl std::fmt::Debug for GifWriter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GifWriter")
            .field("width", &self.width)
            .field("height", &self.height)
            .finish()
    }
}

fn palette() -> Vec<u8> {
    let mut p = Vec::with_capacity(256 * 3);
    for r in 0..6u8 {
        for g in 0..6u8 {
            for b in 0..6u8 {
                p.extend_from_slice(&[r * 51, g * 51, b * 51]);
            }
        }
    }
    // greys for anti-aliased edges
    for i in 0..39u8 {
        let v = 6 + i * 6;
        p.extend_from_slice(&[v, v, v]);
    }
    // index 255 is reserved for "unchanged" (transparent) pixels in delta frames
    p.extend_from_slice(&[255, 0, 255]);
    p
}

impl GifWriter {
    /// Creates a looping GIF file (parent directories are created).
    pub fn create(path: impl AsRef<std::path::Path>, width: u32, height: u32) -> std::io::Result<Self> {
        if let Some(parent) = path.as_ref().parent() {
            std::fs::create_dir_all(parent)?;
        }
        let f = std::io::BufWriter::new(std::fs::File::create(path)?);
        let (w, h) = (width as u16, height as u16);
        let mut encoder = gif::Encoder::new(f, w, h, &palette()).map_err(std::io::Error::other)?;
        encoder
            .set_repeat(gif::Repeat::Infinite)
            .map_err(std::io::Error::other)?;
        Ok(Self {
            encoder,
            width: w,
            height: h,
            prev: None,
        })
    }
    /// Appends a frame shown for `delay_cs` hundredths of a second.
    pub fn frame(&mut self, c: &Canvas, delay_cs: u16) -> std::io::Result<()> {
        assert_eq!(
            (c.width() as u16, c.height() as u16),
            (self.width, self.height),
            "frame size mismatch"
        );
        let q = |v: u8| ((v as u16 + 25) / 51) as u8;
        let idx: Vec<u8> = c
            .data()
            .chunks_exact(4)
            .map(|px| {
                let (r, g, b) = (px[0], px[1], px[2]);
                let grey = r.abs_diff(g) < 8 && g.abs_diff(b) < 8 && r.abs_diff(b) < 8;
                if grey && r > 3 && r < 249 && (r as u16 + 25) % 51 > 10 && (r as u16 + 25) % 51 < 41 {
                    216 + ((r.saturating_sub(6) as u16 + 3) / 6).min(38) as u8
                } else {
                    q(r) * 36 + q(g) * 6 + q(b)
                }
            })
            .collect();
        // Delta encoding: only the bounding box of changed pixels is written and
        // unchanged pixels inside it are transparent, which compresses far better.
        let (w, h) = (self.width as usize, self.height as usize);
        let frame = match &self.prev {
            None => gif::Frame {
                width: self.width,
                height: self.height,
                buffer: std::borrow::Cow::Owned(idx.clone()),
                delay: delay_cs,
                ..Default::default()
            },
            Some(prev) => {
                let (mut x0, mut y0, mut x1, mut y1) = (w, h, 0usize, 0usize);
                for y in 0..h {
                    for x in 0..w {
                        if idx[y * w + x] != prev[y * w + x] {
                            x0 = x0.min(x);
                            y0 = y0.min(y);
                            x1 = x1.max(x);
                            y1 = y1.max(y);
                        }
                    }
                }
                if x0 > x1 {
                    (x0, y0, x1, y1) = (0, 0, 0, 0);
                }
                let mut buf = Vec::with_capacity((x1 - x0 + 1) * (y1 - y0 + 1));
                for y in y0..=y1 {
                    for x in x0..=x1 {
                        let k = y * w + x;
                        buf.push(if idx[k] == prev[k] { 255 } else { idx[k] });
                    }
                }
                gif::Frame {
                    left: x0 as u16,
                    top: y0 as u16,
                    width: (x1 - x0 + 1) as u16,
                    height: (y1 - y0 + 1) as u16,
                    buffer: std::borrow::Cow::Owned(buf),
                    delay: delay_cs,
                    transparent: Some(255),
                    dispose: gif::DisposalMethod::Keep,
                    ..Default::default()
                }
            }
        };
        self.prev = Some(idx);
        self.encoder.write_frame(&frame).map_err(std::io::Error::other)
    }
}
