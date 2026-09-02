//! Parameter-space trimming of curved faces.
//!
//! An `IfcAdvancedFace` on a curved surface is the region of that
//! surface bounded by its edge loops (face-orientation digest §2:
//! the outer bound runs counter-clockwise about the outward normal,
//! holes clockwise; `IfcFaceSurface.SameSense` relates the outward
//! normal to the surface's own). The loops are given as 3-D edge
//! curves, so the face is meshed by
//!
//! 1. inverting every loop vertex into the surface's `(u, v)`
//!    parameters, unwrapping periodic parameters continuously along
//!    the loop (a loop may wind around a cylinder), and orienting the
//!    loops so the face region lies on their **left** in `(u, v)`
//!    (reversing them when `SameSense` is FALSE, because the outward
//!    normal is then `−(∂S/∂u × ∂S/∂v)`);
//! 2. clipping the loops (and, for periodic parameters, their period-
//!    shifted copies) to one fundamental rectangle of the parameter
//!    domain — chains that enter and leave the rectangle are joined by
//!    walking its perimeter counter-clockwise, which introduces the
//!    seam / pole edges the region needs; loops entirely inside stay
//!    whole (counter-clockwise ones are outer boundaries, clockwise
//!    ones holes of the piece containing them);
//! 3. triangulating every resulting polygon-with-holes with the shared
//!    hole-bridging ear clipper, then refining the triangles by
//!    midpoint subdivision until no edge spans more than the surface's
//!    parameter step — new vertices on a boundary chord are
//!    interpolated on the 3-D chord (so the neighbouring face, which
//!    shares that chord, still matches), all others are evaluated on
//!    the surface — with every inserted midpoint recorded so the final
//!    pass re-splits any triangle edge a neighbour subdivided
//!    (no T-junctions inside the face);
//! 4. welding the new vertices by parameter (period images and pole
//!    points collapse to one mesh vertex) and registering the chord
//!    points with the Brep so a later pass can split the *other* face's
//!    triangles along the same chords ([`repair_t_junctions`]).
//!
//! The result is a closed, consistently oriented mesh when the input
//! Brep is; a face whose loops are inconsistently oriented (a file
//! with a wrong `SameSense` / `Orientation`) is retried with the
//! opposite orientation, since an empty or negative region cannot be
//! what the author meant.

use super::surfaces::{ParamSurface, Uv};
use super::{triangulate_profile, GeometryError, ProfileArea, VertexPool};
use std::collections::HashMap;

/// One loop vertex handed to the trimmer: its mesh vertex and world
/// position (its loop neighbours — the chords it belongs to — follow
/// from the loop order).
#[derive(Debug, Clone, Copy)]
pub(super) struct LoopVertex {
    pub(super) id: u32,
    pub(super) p: [f64; 3],
}

/// A vertex of a clipped parameter-space polygon.
#[derive(Debug, Clone, Copy)]
enum PVert {
    /// A loop vertex: mesh id plus its loop neighbours.
    Loop {
        id: u32,
        uv: Uv,
        prev: u32,
        next: u32,
    },
    /// A point on the chord `a → b` (mesh ids, `a < b`) at fraction `t`.
    Cross { a: u32, b: u32, t: f64, uv: Uv },
    /// A point defined by its parameters only (rectangle corners,
    /// subdivision midpoints).
    Grid { uv: Uv },
}

impl PVert {
    fn uv(&self) -> Uv {
        match *self {
            PVert::Loop { uv, .. } | PVert::Cross { uv, .. } | PVert::Grid { uv } => uv,
        }
    }
}

/// One loop in parameter space: its vertices plus the shift its first
/// vertex takes when the loop closes (non-zero when the loop winds
/// around a periodic direction — the closing edge then runs to the
/// period image of the first vertex, not back across the domain).
#[derive(Debug, Clone)]
struct ULoop {
    verts: Vec<PVert>,
    wrap: Uv,
}

impl ULoop {
    /// The polygon's edges as `(p, q)` with the closing edge wrapped.
    fn edges(&self, shift: Uv) -> impl Iterator<Item = (PVert, PVert)> + '_ {
        let n = self.verts.len();
        (0..n).map(move |i| {
            let p = shift_vert(&self.verts[i], shift);
            let q = if i + 1 < n {
                shift_vert(&self.verts[i + 1], shift)
            } else {
                shift_vert(
                    &self.verts[0],
                    [shift[0] + self.wrap[0], shift[1] + self.wrap[1]],
                )
            };
            (p, q)
        })
    }

    fn winds(&self) -> bool {
        self.wrap[0] != 0.0 || self.wrap[1] != 0.0
    }

    /// Reverse the traversal direction, re-unwrapping the parameters
    /// continuously in the new order (vertex 0 stays first).
    fn reverse(&mut self, period_u: Option<f64>, period_v: Option<f64>) {
        let n = self.verts.len();
        self.verts[1..].reverse();
        for v in self.verts.iter_mut() {
            if let PVert::Loop { prev, next, .. } = v {
                core::mem::swap(prev, next);
            }
        }
        let mut prev = self.verts[0].uv();
        for v in self.verts.iter_mut().skip(1) {
            let mut uv = v.uv();
            if let Some(p) = period_u {
                uv[0] = unwrap_near(uv[0], prev[0], p);
            }
            if let Some(p) = period_v {
                uv[1] = unwrap_near(uv[1], prev[1], p);
            }
            set_uv(v, uv);
            prev = uv;
        }
        let (f, l) = (self.verts[0].uv(), self.verts[n - 1].uv());
        self.wrap = [
            period_u.map_or(0.0, |p| unwrap_near(f[0], l[0], p) - f[0]),
            period_v.map_or(0.0, |p| unwrap_near(f[1], l[1], p) - f[1]),
        ];
    }

    /// Parameter bounding box including the wrapped closing point.
    fn bbox(&self) -> (Uv, Uv) {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        let first = self.verts[0].uv();
        for uv in self.verts.iter().map(PVert::uv).chain(core::iter::once([
            first[0] + self.wrap[0],
            first[1] + self.wrap[1],
        ])) {
            for k in 0..2 {
                lo[k] = lo[k].min(uv[k]);
                hi[k] = hi[k].max(uv[k]);
            }
        }
        (lo, hi)
    }
}

fn set_uv(v: &mut PVert, new: Uv) {
    match v {
        PVert::Loop { uv, .. } | PVert::Cross { uv, .. } | PVert::Grid { uv } => *uv = new,
    }
}

fn shift_vert(v: &PVert, shift: Uv) -> PVert {
    let s = |uv: Uv| [uv[0] + shift[0], uv[1] + shift[1]];
    match *v {
        PVert::Loop { id, uv, prev, next } => PVert::Loop {
            id,
            uv: s(uv),
            prev,
            next,
        },
        PVert::Cross { a, b, t, uv } => PVert::Cross { a, b, t, uv: s(uv) },
        PVert::Grid { uv } => PVert::Grid { uv: s(uv) },
    }
}

/// A closed region piece in the fundamental rectangle.
struct Piece {
    outer: Vec<PVert>,
    holes: Vec<Vec<PVert>>,
}

/// The parameter rectangle the loops are clipped to.
#[derive(Debug, Clone, Copy)]
struct Rect {
    u0: f64,
    u1: f64,
    v0: f64,
    v1: f64,
}

/// Largest number of triangles one face may produce.
const MAX_FACE_TRIANGLES: usize = 400_000;

/// Largest number of subdivision rounds.
const MAX_REFINE_ROUNDS: usize = 16;

/// Mesh a curved face into `triangles` against the shared `pool`.
/// `loops[0]` is the outer bound; every loop is already in its
/// effective direction (`IfcFaceBound.Orientation` applied) and
/// `same_sense` is the face's `IfcFaceSurface.SameSense`.
pub(super) fn tessellate_curved_face(
    surface: &ParamSurface,
    surface_key: u64,
    loops: &[Vec<LoopVertex>],
    same_sense: bool,
    pool: &mut VertexPool,
    triangles: &mut Vec<[u32; 3]>,
) -> Result<(), GeometryError> {
    let uv_loops = parameter_loops(surface, loops)?;
    // First attempt with the schema orientation; retry flipped when the
    // region comes out empty (inconsistent flags in the file).
    for attempt in 0..2 {
        let reversed = (!same_sense) ^ (attempt == 1);
        let mut oriented: Vec<ULoop> = uv_loops.clone();
        if reversed {
            for l in &mut oriented {
                l.reverse(surface.period_u(), surface.period_v());
            }
        }
        let rect = fundamental_rect(surface, &oriented);
        let pieces = clip_to_rect(&oriented, &rect, surface.period_u(), surface.period_v());
        // Net region area: outer pieces minus their holes.
        let area: f64 = pieces
            .iter()
            .map(|p| signed_area(&p.outer) + p.holes.iter().map(|h| signed_area(h)).sum::<f64>())
            .sum();
        if pieces.is_empty() || area <= 0.0 {
            continue;
        }
        let mut out: Vec<[u32; 3]> = Vec::new();
        let mut failed = false;
        for piece in &pieces {
            if mesh_piece(surface, surface_key, piece, reversed, pool, &mut out).is_err() {
                failed = true;
                break;
            }
        }
        if failed || out.is_empty() {
            continue;
        }
        triangles.extend(out);
        return Ok(());
    }
    Err(GeometryError::BadCoordinates)
}

/// Invert the loops into parameter space with continuous unwrapping;
/// pole vertices (degenerate `u`) are doubled so the boundary follows
/// the pole line between their neighbours' `u` values.
fn parameter_loops(
    surface: &ParamSurface,
    loops: &[Vec<LoopVertex>],
) -> Result<Vec<ULoop>, GeometryError> {
    let period_u = surface.period_u();
    let period_v = surface.period_v();
    let mut out = Vec::with_capacity(loops.len());
    for l in loops {
        let n = l.len();
        if n < 3 {
            return Err(GeometryError::BadCoordinates);
        }
        // Raw inversions.
        let raw: Vec<(Uv, bool)> = l.iter().map(|v| surface.inverse(v.p)).collect();
        // Unwrap continuously, starting from the first non-degenerate
        // vertex; degenerate ones take the running u.
        let mut uv: Vec<Uv> = Vec::with_capacity(n);
        let start = raw.iter().position(|(_, d)| !d).unwrap_or(0);
        let mut last: Option<Uv> = None;
        for k in 0..n {
            let i = (start + k) % n;
            let (mut p, degenerate) = raw[i];
            if let Some(prev) = last {
                if degenerate {
                    p[0] = prev[0];
                } else if let Some(pu) = period_u {
                    p[0] = unwrap_near(p[0], prev[0], pu);
                }
                if let Some(pv) = period_v {
                    p[1] = unwrap_near(p[1], prev[1], pv);
                }
            }
            uv.push(p);
            last = Some(p);
        }
        // The closing shift: where the first vertex's image lands when
        // the loop comes back around (a multiple of the period).
        let first = uv[0];
        let last_uv = uv[n - 1];
        let wrap = [
            period_u.map_or(0.0, |p| unwrap_near(first[0], last_uv[0], p) - first[0]),
            period_v.map_or(0.0, |p| unwrap_near(first[1], last_uv[1], p) - first[1]),
        ];
        // Rotate back to the loop's own vertex order.
        uv.rotate_right(start);
        let wrap = if start == 0 {
            wrap
        } else {
            // The unwrapping started mid-loop; recompute the closing
            // shift in the loop's own order.
            let (f, l) = (uv[0], uv[n - 1]);
            [
                period_u.map_or(0.0, |p| unwrap_near(f[0], l[0], p) - f[0]),
                period_v.map_or(0.0, |p| unwrap_near(f[1], l[1], p) - f[1]),
            ]
        };
        let mut verts: Vec<PVert> = Vec::with_capacity(n + 2);
        for i in 0..n {
            let prev = l[(i + n - 1) % n].id;
            let next = l[(i + 1) % n].id;
            let id = l[i].id;
            if raw[i].1 {
                // Pole: one copy at the previous vertex's u, one at the
                // next vertex's u.
                let u_prev = uv[(i + n - 1) % n][0];
                let u_next = uv[(i + 1) % n][0];
                verts.push(PVert::Loop {
                    id,
                    uv: [u_prev, uv[i][1]],
                    prev,
                    next,
                });
                verts.push(PVert::Loop {
                    id,
                    uv: [u_next, uv[i][1]],
                    prev,
                    next,
                });
            } else {
                verts.push(PVert::Loop {
                    id,
                    uv: uv[i],
                    prev,
                    next,
                });
            }
        }
        out.push(ULoop { verts, wrap });
    }
    Ok(out)
}

/// The image of `x` under period shifts closest to `reference`.
fn unwrap_near(x: f64, reference: f64, period: f64) -> f64 {
    let k = ((reference - x) / period).round();
    x + k * period
}

/// The rectangle the loops are clipped to: one period in a periodic
/// direction, the surface's fixed extent otherwise, or the loops' own
/// range (a cylinder's height is whatever the loops span).
fn fundamental_rect(surface: &ParamSurface, loops: &[ULoop]) -> Rect {
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for l in loops {
        let (llo, lhi) = l.bbox();
        for k in 0..2 {
            lo[k] = lo[k].min(llo[k]);
            hi[k] = hi[k].max(lhi[k]);
        }
    }
    let (u0, u1) = match (surface.period_u(), surface.u_extent()) {
        (Some(p), _) => {
            // Anchor the seam at the outer loop's lowest u so a loop
            // that does not wind is not cut at all.
            let a = loops.first().map_or(0.0, |l| l.bbox().0[0]);
            (a, a + p)
        }
        (None, Some(e)) => e,
        (None, None) => (lo[0], hi[0]),
    };
    let (v0, v1) = match (surface.period_v(), surface.v_extent()) {
        (Some(p), _) => {
            let a = loops.first().map_or(0.0, |l| l.bbox().0[1]);
            (a, a + p)
        }
        (None, Some(e)) => e,
        (None, None) => (lo[1], hi[1]),
    };
    // Guard against a flat range (a loop lying entirely on one line).
    let pad = |a: f64, b: f64| {
        if b - a <= 0.0 {
            (a - 1.0, b + 1.0)
        } else {
            (a, b)
        }
    };
    let (u0, u1) = pad(u0, u1);
    let (v0, v1) = pad(v0, v1);
    Rect { u0, u1, v0, v1 }
}

/// Signed area (shoelace) of a parameter polygon.
fn signed_area(poly: &[PVert]) -> f64 {
    let mut s = 0.0;
    for i in 0..poly.len() {
        let a = poly[i].uv();
        let b = poly[(i + 1) % poly.len()].uv();
        s += a[0] * b[1] - b[0] * a[1];
    }
    0.5 * s
}

/// Ray-casting point-in-polygon in parameter space.
fn point_in_polygon(p: Uv, poly: &[PVert]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let a = poly[i].uv();
        let b = poly[(i + 1) % n].uv();
        if (a[1] > p[1]) != (b[1] > p[1]) {
            let x = a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
            if p[0] < x {
                inside = !inside;
            }
        }
    }
    inside
}

/// One run of loop vertices inside the rectangle, entering at its
/// first vertex and leaving at its last (both on the perimeter).
struct Chain {
    verts: Vec<PVert>,
    t_in: f64,
    t_out: f64,
    /// Whether the chain's first / last vertex lies on the perimeter
    /// (a winding loop copy may start or end at an interior vertex;
    /// such chains are joined with their continuation from the
    /// neighbouring copy before the perimeter walk).
    closed_start: bool,
    closed_end: bool,
}

/// Clip the oriented loops (region on the left) to `rect`, joining
/// perimeter-crossing chains along the perimeter, and assemble the
/// pieces with their holes.
fn clip_to_rect(
    loops: &[ULoop],
    rect: &Rect,
    period_u: Option<f64>,
    period_v: Option<f64>,
) -> Vec<Piece> {
    let mut chains: Vec<Chain> = Vec::new();
    let mut inside_loops: Vec<Vec<PVert>> = Vec::new();
    for l in loops {
        let (lo, hi) = l.bbox();
        let shifts = |period: Option<f64>, lo: f64, hi: f64, r0: f64, r1: f64| -> Vec<f64> {
            match period {
                None => vec![0.0],
                Some(p) => {
                    let kmin = ((r0 - hi) / p).floor() as i64;
                    let kmax = ((r1 - lo) / p).ceil() as i64;
                    (kmin..=kmax)
                        .filter(|&k| {
                            let s = k as f64 * p;
                            hi + s >= r0 && lo + s <= r1
                        })
                        .map(|k| k as f64 * p)
                        .collect()
                }
            }
        };
        let su = shifts(period_u, lo[0], hi[0], rect.u0, rect.u1);
        let sv = shifts(period_v, lo[1], hi[1], rect.v0, rect.v1);
        for &du in &su {
            for &dv in &sv {
                clip_loop(l, [du, dv], rect, &mut chains, &mut inside_loops);
            }
        }
    }

    merge_open_chains(&mut chains);
    // A chain running along the perimeter *against* its counter-
    // clockwise direction has the region outside the rectangle (a
    // period image of a loop that bounds the region from the other
    // side): it contributes nothing here.
    chains.retain(|c| {
        if !c.verts.iter().all(|v| on_perimeter(rect, v.uv())) {
            return true;
        }
        let mut advance = 0.0;
        for w in c.verts.windows(2) {
            let (a, b) = (
                perimeter_param(rect, w[0].uv()),
                perimeter_param(rect, w[1].uv()),
            );
            let d = b - a;
            // Shortest signed step around the perimeter.
            advance += if d > 2.0 {
                d - 4.0
            } else if d < -2.0 {
                d + 4.0
            } else {
                d
            };
        }
        advance >= 0.0
    });
    let mut pieces: Vec<Piece> = Vec::new();
    // Perimeter-walk assembly of the crossing chains.
    let n = chains.len();
    let mut used = vec![false; n];
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| {
        chains[a]
            .t_in
            .partial_cmp(&chains[b].t_in)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    for &start in &order {
        if used[start] {
            continue;
        }
        let mut poly: Vec<PVert> = Vec::new();
        let mut cur = start;
        let mut guard = 0;
        loop {
            used[cur] = true;
            poly.extend(chains[cur].verts.iter().copied());
            let x = chains[cur].t_out;
            // Next chain: the smallest entry parameter after the exit,
            // cyclically; the current chain itself only if nothing
            // else lies between (it then closes on itself).
            let mut best: Option<(f64, usize)> = None;
            for (j, c) in chains.iter().enumerate() {
                let mut d = (c.t_in - x).rem_euclid(4.0);
                if j == cur && d < 1e-12 {
                    d = 4.0;
                }
                if match best {
                    Some((bd, _)) => d < bd,
                    None => true,
                } {
                    best = Some((d, j));
                }
            }
            let Some((d, nxt)) = best else { break };
            // Corners passed on the way.
            for c in 1..=4u32 {
                let tc = c as f64;
                let dc = (tc - x).rem_euclid(4.0);
                if dc > 1e-12 && dc < d - 1e-12 {
                    poly.push(PVert::Grid {
                        uv: perimeter_point(rect, tc),
                    });
                }
            }
            // Corners in increasing order of distance: sort the tail.
            let tail_start = poly.len() - corner_count(x, d);
            poly[tail_start..].sort_by(|a, b| {
                let ta = (perimeter_param(rect, a.uv()) - x).rem_euclid(4.0);
                let tb = (perimeter_param(rect, b.uv()) - x).rem_euclid(4.0);
                ta.partial_cmp(&tb).unwrap_or(core::cmp::Ordering::Equal)
            });
            if nxt == start || used[nxt] {
                break;
            }
            cur = nxt;
            guard += 1;
            if guard > n {
                break;
            }
        }
        dedup_uv(&mut poly);
        if poly.len() >= 3 && signed_area(&poly).abs() > 0.0 {
            pieces.push(Piece {
                outer: poly,
                holes: Vec::new(),
            });
        }
    }

    // Whole loops inside: counter-clockwise → own pieces, clockwise →
    // holes; only holes and no chains → the whole rectangle is the
    // region.
    let mut holes: Vec<Vec<PVert>> = Vec::new();
    for l in inside_loops {
        if signed_area(&l) > 0.0 {
            pieces.push(Piece {
                outer: l,
                holes: Vec::new(),
            });
        } else {
            holes.push(l);
        }
    }
    if pieces.is_empty() && !holes.is_empty() {
        pieces.push(Piece {
            outer: (0..4)
                .map(|c| PVert::Grid {
                    uv: perimeter_point(rect, c as f64),
                })
                .collect(),
            holes: Vec::new(),
        });
    }
    for h in holes {
        let probe = h[0].uv();
        if let Some(piece) = pieces
            .iter_mut()
            .find(|p| point_in_polygon(probe, &p.outer))
        {
            piece.holes.push(h);
        }
    }
    pieces
}

/// Number of rectangle corners strictly between perimeter parameters
/// `x` and `x + d` (cyclic).
fn corner_count(x: f64, d: f64) -> usize {
    (1..=4u32)
        .filter(|&c| {
            let dc = (c as f64 - x).rem_euclid(4.0);
            dc > 1e-12 && dc < d - 1e-12
        })
        .count()
}

/// Drop consecutive vertices with identical parameters.
fn dedup_uv(poly: &mut Vec<PVert>) {
    let same = |a: Uv, b: Uv| (a[0] - b[0]).abs() <= 1e-12 && (a[1] - b[1]).abs() <= 1e-12;
    poly.dedup_by(|b, a| same(a.uv(), b.uv()));
    while poly.len() > 1 && same(poly[0].uv(), poly[poly.len() - 1].uv()) {
        poly.pop();
    }
}

/// Perimeter parameter `t ∈ [0, 4)` of a point on the rectangle
/// boundary, counter-clockwise from the `(u0, v0)` corner.
fn perimeter_param(rect: &Rect, p: Uv) -> f64 {
    let w = (rect.u1 - rect.u0).max(f64::MIN_POSITIVE);
    let h = (rect.v1 - rect.v0).max(f64::MIN_POSITIVE);
    // Distances to the four sides.
    let d_bottom = (p[1] - rect.v0).abs();
    let d_right = (p[0] - rect.u1).abs();
    let d_top = (p[1] - rect.v1).abs();
    let d_left = (p[0] - rect.u0).abs();
    let m = d_bottom.min(d_right).min(d_top).min(d_left);
    if m == d_bottom {
        ((p[0] - rect.u0) / w).clamp(0.0, 1.0)
    } else if m == d_right {
        1.0 + ((p[1] - rect.v0) / h).clamp(0.0, 1.0)
    } else if m == d_top {
        2.0 + ((rect.u1 - p[0]) / w).clamp(0.0, 1.0)
    } else {
        3.0 + ((rect.v1 - p[1]) / h).clamp(0.0, 1.0)
    }
}

/// The boundary point at perimeter parameter `t`.
fn perimeter_point(rect: &Rect, t: f64) -> Uv {
    let t = t.rem_euclid(4.0);
    let w = rect.u1 - rect.u0;
    let h = rect.v1 - rect.v0;
    if t < 1.0 {
        [rect.u0 + t * w, rect.v0]
    } else if t < 2.0 {
        [rect.u1, rect.v0 + (t - 1.0) * h]
    } else if t < 3.0 {
        [rect.u1 - (t - 2.0) * w, rect.v1]
    } else {
        [rect.u0, rect.v1 - (t - 3.0) * h]
    }
}

/// Clip one (shifted) loop copy: whole-inside loops go to
/// `inside_loops`, perimeter-crossing runs to `chains`.
fn clip_loop(
    l: &ULoop,
    shift: Uv,
    rect: &Rect,
    chains: &mut Vec<Chain>,
    inside_loops: &mut Vec<Vec<PVert>>,
) {
    let edges: Vec<(PVert, PVert)> = l.edges(shift).collect();
    let n = edges.len();
    let size = (rect.u1 - rect.u0).abs().max((rect.v1 - rect.v0).abs());
    let eps = 1e-9 * size;
    let inside = |p: Uv| {
        p[0] >= rect.u0 - eps
            && p[0] <= rect.u1 + eps
            && p[1] >= rect.v0 - eps
            && p[1] <= rect.v1 + eps
    };
    // Start at a vertex outside the rectangle so chains are delimited
    // by crossings; a non-winding loop with none is entirely inside. A
    // winding loop with none starts at its first vertex, which then
    // sits on the seam (the rectangle is anchored on the loops).
    let start = match edges.iter().position(|(p, _)| !inside(p.uv())) {
        Some(i) => i,
        None if !l.winds() => {
            inside_loops.push(edges.iter().map(|(p, _)| *p).collect());
            return;
        }
        None => 0,
    };
    let mut cur: Option<Vec<PVert>> = None;
    let near = |a: Uv, b: Uv| (a[0] - b[0]).abs() <= eps && (a[1] - b[1]).abs() <= eps;
    let at = |p: Uv, q: Uv, t: f64| [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t];
    for k in 0..n {
        let (p, q) = edges[(start + k) % n];
        let Some((t0, t1)) = clip_segment(p.uv(), q.uv(), rect) else {
            continue;
        };
        if t0 > 0.0 {
            // Entering: a new chain starts at the entry point, snapped
            // to a segment end that lies (within tolerance) on the
            // boundary.
            let x = at(p.uv(), q.uv(), t0);
            let entry = if near(x, q.uv()) {
                q
            } else if near(x, p.uv()) {
                p
            } else {
                cross_vertex(&p, &q, t0)
            };
            cur = Some(vec![entry]);
        } else if cur.is_none() {
            cur = Some(vec![p]);
        }
        let chain = cur.as_mut().expect("chain open");
        if t1 < 1.0 {
            // Leaving before q.
            let x = at(p.uv(), q.uv(), t1);
            if near(x, q.uv()) {
                push_distinct(chain, q);
            } else if !near(x, p.uv()) {
                push_distinct(chain, cross_vertex(&p, &q, t1));
            }
            let verts = cur.take().expect("chain open");
            close_chain(verts, rect, chains);
        } else {
            push_distinct(chain, q);
        }
    }
    if let Some(verts) = cur.take() {
        close_chain(verts, rect, chains);
    }
}

fn push_distinct(chain: &mut Vec<PVert>, v: PVert) {
    if let Some(last) = chain.last() {
        let (a, b) = (last.uv(), v.uv());
        if (a[0] - b[0]).abs() <= 1e-12 && (a[1] - b[1]).abs() <= 1e-12 {
            return;
        }
    }
    chain.push(v);
}

fn close_chain(mut verts: Vec<PVert>, rect: &Rect, chains: &mut Vec<Chain>) {
    dedup_uv_open(&mut verts);
    if verts.len() < 2 {
        return;
    }
    let t_in = perimeter_param(rect, verts[0].uv());
    let t_out = perimeter_param(rect, verts[verts.len() - 1].uv());
    let closed_start = on_perimeter(rect, verts[0].uv());
    let closed_end = on_perimeter(rect, verts[verts.len() - 1].uv());
    chains.push(Chain {
        verts,
        t_in,
        t_out,
        closed_start,
        closed_end,
    });
}

/// Whether a parameter point lies on the rectangle boundary.
fn on_perimeter(rect: &Rect, p: Uv) -> bool {
    let size = (rect.u1 - rect.u0).abs().max((rect.v1 - rect.v0).abs());
    let eps = 1e-9 * size;
    (p[0] - rect.u0).abs() <= eps
        || (p[0] - rect.u1).abs() <= eps
        || (p[1] - rect.v0).abs() <= eps
        || (p[1] - rect.v1).abs() <= eps
}

/// Join chains that end at an interior vertex with the chain (from the
/// neighbouring period copy) that starts there; chains still open
/// afterwards are malformed and dropped.
fn merge_open_chains(chains: &mut Vec<Chain>) {
    let same = |a: Uv, b: Uv| (a[0] - b[0]).abs() <= 1e-9 && (a[1] - b[1]).abs() <= 1e-9;
    let mut guard = chains.len() + 1;
    while guard > 0 {
        guard -= 1;
        let Some(i) = chains.iter().position(|c| !c.closed_end) else {
            break;
        };
        let tail = chains[i].verts[chains[i].verts.len() - 1].uv();
        let Some(j) = chains
            .iter()
            .enumerate()
            .position(|(j, c)| j != i && !c.closed_start && same(c.verts[0].uv(), tail))
        else {
            // No continuation: drop the open chain.
            chains.remove(i);
            continue;
        };
        let mut next = chains.remove(j);
        let i = if j < i { i - 1 } else { i };
        let head = &mut chains[i];
        head.verts.extend(next.verts.drain(1..));
        head.t_out = next.t_out;
        head.closed_end = next.closed_end;
    }
    chains.retain(|c| c.closed_start && c.closed_end);
}

fn dedup_uv_open(poly: &mut Vec<PVert>) {
    let same = |a: Uv, b: Uv| (a[0] - b[0]).abs() <= 1e-12 && (a[1] - b[1]).abs() <= 1e-12;
    poly.dedup_by(|b, a| same(a.uv(), b.uv()));
}

/// The point at fraction `t` along the parameter segment `p → q`: a
/// chord point when both ends are loop vertices of one chord (or one
/// of them a chord point), a plain parameter point otherwise.
fn cross_vertex(p: &PVert, q: &PVert, t: f64) -> PVert {
    let (a, b) = (p.uv(), q.uv());
    let uv = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
    match chord_between(p, q) {
        Some((ca, cb, ta, tb)) => PVert::Cross {
            a: ca,
            b: cb,
            t: ta + (tb - ta) * t,
            uv,
        },
        None => PVert::Grid { uv },
    }
}

/// If `p` and `q` lie on one loop chord, that chord `(a, b)` (`a < b`)
/// with the chord fractions of `p` and `q`.
fn chord_between(p: &PVert, q: &PVert) -> Option<(u32, u32, f64, f64)> {
    let chords_of = |v: &PVert| -> Vec<(u32, u32, f64)> {
        match *v {
            PVert::Loop { id, prev, next, .. } => {
                let mut out = Vec::with_capacity(2);
                for other in [prev, next] {
                    if other == id {
                        continue;
                    }
                    let (a, b) = if id < other { (id, other) } else { (other, id) };
                    out.push((a, b, if id == a { 0.0 } else { 1.0 }));
                }
                out
            }
            PVert::Cross { a, b, t, .. } => vec![(a, b, t)],
            PVert::Grid { .. } => Vec::new(),
        }
    };
    let cp = chords_of(p);
    let cq = chords_of(q);
    for &(a, b, tp) in &cp {
        for &(a2, b2, tq) in &cq {
            if a == a2 && b == b2 {
                return Some((a, b, tp, tq));
            }
        }
    }
    None
}

/// Liang–Barsky clip of the segment `p → q` against the (closed)
/// rectangle: the parameter interval `[t0, t1] ⊆ [0, 1]` inside, or
/// `None`. Exact — the caller snaps crossings that fall within
/// tolerance of a segment end onto that end.
fn clip_segment(p: Uv, q: Uv, rect: &Rect) -> Option<(f64, f64)> {
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    let d = [q[0] - p[0], q[1] - p[1]];
    let checks = [
        (-d[0], p[0] - rect.u0),
        (d[0], rect.u1 - p[0]),
        (-d[1], p[1] - rect.v0),
        (d[1], rect.v1 - p[1]),
    ];
    for (pk, qk) in checks {
        if pk == 0.0 {
            if qk < 0.0 {
                return None;
            }
        } else {
            let r = qk / pk;
            if pk < 0.0 {
                if r > t1 {
                    return None;
                }
                if r > t0 {
                    t0 = r;
                }
            } else {
                if r < t0 {
                    return None;
                }
                if r < t1 {
                    t1 = r;
                }
            }
        }
    }
    Some((t0, t1))
}

/// The provenance of a local (per-piece) vertex during refinement.
#[derive(Debug, Clone, Copy)]
enum Local {
    Loop { id: u32, prev: u32, next: u32 },
    Cross { a: u32, b: u32, t: f64 },
    Grid,
}

/// Triangulate, refine and emit one piece.
fn mesh_piece(
    surface: &ParamSurface,
    surface_key: u64,
    piece: &Piece,
    reversed: bool,
    pool: &mut VertexPool,
    out: &mut Vec<[u32; 3]>,
) -> Result<(), GeometryError> {
    let (su, sv) = surface.metric();
    // Local vertex table.
    let mut kinds: Vec<Local> = Vec::new();
    let mut uvs: Vec<Uv> = Vec::new();
    let mut pos: Vec<[f64; 3]> = Vec::new();
    let add = |v: &PVert, kinds: &mut Vec<Local>, uvs: &mut Vec<Uv>, pos: &mut Vec<[f64; 3]>| {
        let uv = v.uv();
        let (kind, p) = match *v {
            PVert::Loop { id, prev, next, .. } => {
                (Local::Loop { id, prev, next }, pool.positions[id as usize])
            }
            PVert::Cross { a, b, t, .. } => {
                let (pa, pb) = (pool.positions[a as usize], pool.positions[b as usize]);
                (
                    Local::Cross { a, b, t },
                    [
                        pa[0] + (pb[0] - pa[0]) * t,
                        pa[1] + (pb[1] - pa[1]) * t,
                        pa[2] + (pb[2] - pa[2]) * t,
                    ],
                )
            }
            PVert::Grid { .. } => (Local::Grid, surface.eval(uv)),
        };
        kinds.push(kind);
        uvs.push(uv);
        pos.push(p);
    };
    let scaled = |uv: Uv| [uv[0] * su, uv[1] * sv];
    let mut outer2: Vec<Uv> = Vec::with_capacity(piece.outer.len());
    for v in &piece.outer {
        add(v, &mut kinds, &mut uvs, &mut pos);
        outer2.push(scaled(v.uv()));
    }
    let mut holes2: Vec<Vec<Uv>> = Vec::with_capacity(piece.holes.len());
    let mut index_table: Vec<u32> = (0..piece.outer.len() as u32).collect();
    for h in &piece.holes {
        // triangulate_profile wants counter-clockwise hole rings.
        let mut ring: Vec<Uv> = Vec::with_capacity(h.len());
        let mut ids: Vec<u32> = Vec::with_capacity(h.len());
        for v in h {
            ids.push(kinds.len() as u32);
            add(v, &mut kinds, &mut uvs, &mut pos);
            ring.push(scaled(v.uv()));
        }
        if super::signed_area_2x(&ring) < 0.0 {
            ring.reverse();
            ids.reverse();
        }
        holes2.push(ring);
        index_table.extend(ids);
    }
    let area = ProfileArea {
        outer: outer2,
        holes: holes2,
    };
    let cap = triangulate_profile(&area)?;
    let mut tris: Vec<[u32; 3]> = cap
        .iter()
        .map(|t| {
            [
                index_table[t[0] as usize],
                index_table[t[1] as usize],
                index_table[t[2] as usize],
            ]
        })
        .collect();

    // Midpoint refinement.
    let (step_u, step_v) = surface.step();
    let mut midpoints: HashMap<(u32, u32), u32> = HashMap::new();
    let too_long = |a: Uv, b: Uv| {
        step_u.is_some_and(|s| (a[0] - b[0]).abs() > s * 1.0001)
            || step_v.is_some_and(|s| (a[1] - b[1]).abs() > s * 1.0001)
    };
    for _ in 0..MAX_REFINE_ROUNDS {
        if tris.len() > MAX_FACE_TRIANGLES {
            break;
        }
        let mut next: Vec<[u32; 3]> = Vec::with_capacity(tris.len());
        let mut changed = false;
        for t in &tris {
            let long = [
                too_long(uvs[t[0] as usize], uvs[t[1] as usize]),
                too_long(uvs[t[1] as usize], uvs[t[2] as usize]),
                too_long(uvs[t[2] as usize], uvs[t[0] as usize]),
            ];
            if !long.iter().any(|&l| l) {
                next.push(*t);
                continue;
            }
            changed = true;
            let mut mid = |a: u32, b: u32| -> u32 {
                midpoint(
                    a,
                    b,
                    surface,
                    &mut midpoints,
                    &mut kinds,
                    &mut uvs,
                    &mut pos,
                )
            };
            let [a, b, c] = *t;
            match long {
                [true, true, true] => {
                    let (ab, bc, ca) = (mid(a, b), mid(b, c), mid(c, a));
                    next.extend([[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]]);
                }
                _ => {
                    // Rotate so the first edge is long.
                    let (a, b, c, l1, l2) = if long[0] {
                        (a, b, c, long[1], long[2])
                    } else if long[1] {
                        (b, c, a, long[2], long[0])
                    } else {
                        (c, a, b, long[0], long[1])
                    };
                    let ab = mid(a, b);
                    match (l1, l2) {
                        (false, false) => next.extend([[a, ab, c], [ab, b, c]]),
                        (true, false) => {
                            let bc = mid(b, c);
                            next.extend([[a, ab, c], [ab, bc, c], [ab, b, bc]]);
                        }
                        (false, true) => {
                            let ca = mid(c, a);
                            next.extend([[a, ab, ca], [ab, c, ca], [ab, b, c]]);
                        }
                        (true, true) => unreachable!(),
                    }
                }
            }
        }
        tris = next;
        if !changed {
            break;
        }
    }

    // Conform every edge to the midpoints its neighbours inserted.
    let mut conformed: Vec<[u32; 3]> = Vec::with_capacity(tris.len());
    let mut stack = tris;
    while let Some(t) = stack.pop() {
        let mut split = false;
        for i in 0..3 {
            let (x, y, z) = (t[i], t[(i + 1) % 3], t[(i + 2) % 3]);
            let key = if x < y { (x, y) } else { (y, x) };
            if let Some(&m) = midpoints.get(&key) {
                stack.push([x, m, z]);
                stack.push([m, y, z]);
                split = true;
                break;
            }
        }
        if !split {
            conformed.push(t);
        }
        if conformed.len() + stack.len() > MAX_FACE_TRIANGLES * 4 {
            return Err(GeometryError::BadCoordinates);
        }
    }

    // Emit with welding and chord registration.
    let mut mesh_id: Vec<Option<u32>> = vec![None; kinds.len()];
    let mut resolve = |i: usize, pool: &mut VertexPool| -> u32 {
        if let Some(id) = mesh_id[i] {
            return id;
        }
        let id = match kinds[i] {
            Local::Loop { id, .. } => id,
            Local::Cross { a, b, t } => {
                let key = (a, b, (t * 1e9).round() as i64);
                if let Some(&id) = pool.chord_weld.get(&key) {
                    id
                } else {
                    let id = pool.push_raw(pos[i]);
                    pool.chord_weld.insert(key, id);
                    pool.chord_points.entry((a, b)).or_default().push((t, id));
                    id
                }
            }
            Local::Grid => {
                let (ku, kv) = surface.weld_key(uvs[i]);
                let key = (surface_key, ku, kv);
                if let Some(&id) = pool.param_weld.get(&key) {
                    id
                } else {
                    let id = pool.push_raw(pos[i]);
                    pool.param_weld.insert(key, id);
                    id
                }
            }
        };
        mesh_id[i] = Some(id);
        id
    };
    for t in conformed {
        let a = resolve(t[0] as usize, pool);
        let b = resolve(t[1] as usize, pool);
        let c = resolve(t[2] as usize, pool);
        if a == b || b == c || c == a {
            continue; // collapsed onto a pole / seam point
        }
        if reversed {
            out.push([a, c, b]);
        } else {
            out.push([a, b, c]);
        }
    }
    Ok(())
}

/// The (shared) midpoint vertex of local edge `a–b`: on the 3-D chord
/// when both ends lie on one loop chord, on the surface otherwise.
#[allow(clippy::too_many_arguments)]
fn midpoint(
    a: u32,
    b: u32,
    surface: &ParamSurface,
    midpoints: &mut HashMap<(u32, u32), u32>,
    kinds: &mut Vec<Local>,
    uvs: &mut Vec<Uv>,
    pos: &mut Vec<[f64; 3]>,
) -> u32 {
    let key = if a < b { (a, b) } else { (b, a) };
    if let Some(&m) = midpoints.get(&key) {
        return m;
    }
    let (ua, ub) = (uvs[a as usize], uvs[b as usize]);
    // Where along the edge to split: the midpoint, unless the surface
    // prefers a breakpoint (a sampled-profile vertex) inside the edge.
    let f = surface.split_fraction(ua, ub);
    let uv = [ua[0] + (ub[0] - ua[0]) * f, ua[1] + (ub[1] - ua[1]) * f];
    let as_pvert = |k: Local, uv: Uv| match k {
        Local::Loop { id, prev, next } => PVert::Loop { id, uv, prev, next },
        Local::Cross { a, b, t } => PVert::Cross { a, b, t, uv },
        Local::Grid => PVert::Grid { uv },
    };
    let pa = as_pvert(kinds[a as usize], ua);
    let pb = as_pvert(kinds[b as usize], ub);
    // Two parameter images of one mesh vertex (a pole line): the
    // midpoint is that vertex again, so the triangles collapse.
    if let (PVert::Loop { id: ia, .. }, PVert::Loop { id: ib, .. }) = (&pa, &pb) {
        if ia == ib {
            let m = kinds.len() as u32;
            kinds.push(kinds[a as usize]);
            uvs.push(uv);
            pos.push(pos[a as usize]);
            midpoints.insert(key, m);
            return m;
        }
    }
    let (kind, p) = match chord_between(&pa, &pb) {
        Some((ca, cb, ta, tb)) => {
            let (qa, qb) = (pos[a as usize], pos[b as usize]);
            (
                Local::Cross {
                    a: ca,
                    b: cb,
                    t: ta + (tb - ta) * f,
                },
                [
                    qa[0] + (qb[0] - qa[0]) * f,
                    qa[1] + (qb[1] - qa[1]) * f,
                    qa[2] + (qb[2] - qa[2]) * f,
                ],
            )
        }
        None => (Local::Grid, surface.eval(uv)),
    };
    let m = kinds.len() as u32;
    kinds.push(kind);
    uvs.push(uv);
    pos.push(p);
    midpoints.insert(key, m);
    m
}

/// Split every triangle edge that runs along a loop chord on which
/// another face inserted vertices, so the faces meeting at that chord
/// share their vertices (no T-junctions across faces). Runs once per
/// Brep after all faces are meshed.
pub(super) fn repair_t_junctions(triangles: &mut Vec<[u32; 3]>, pool: &VertexPool) {
    if pool.chord_points.is_empty() {
        return;
    }
    // Inserted vertex → (chord, t).
    let mut on_chord: HashMap<u32, ((u32, u32), f64)> = HashMap::new();
    for (&chord, pts) in &pool.chord_points {
        for &(t, id) in pts {
            on_chord.insert(id, (chord, t));
        }
    }
    let position_on = |v: u32, chord: (u32, u32)| -> Option<f64> {
        if v == chord.0 {
            Some(0.0)
        } else if v == chord.1 {
            Some(1.0)
        } else {
            on_chord
                .get(&v)
                .and_then(|&(c, t)| if c == chord { Some(t) } else { None })
        }
    };
    // Points strictly between x and y on their common chord, ordered
    // from x to y.
    let between = |x: u32, y: u32| -> Option<Vec<u32>> {
        let chord = if let Some(&(c, _)) = on_chord.get(&x) {
            c
        } else if let Some(&(c, _)) = on_chord.get(&y) {
            c
        } else if x < y {
            (x, y)
        } else {
            (y, x)
        };
        let tx = position_on(x, chord)?;
        let ty = position_on(y, chord)?;
        let pts = pool.chord_points.get(&chord)?;
        let (lo, hi) = if tx < ty { (tx, ty) } else { (ty, tx) };
        let mut inner: Vec<(f64, u32)> = pts
            .iter()
            .filter(|&&(t, id)| t > lo && t < hi && id != x && id != y)
            .copied()
            .collect();
        if inner.is_empty() {
            return None;
        }
        inner.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(core::cmp::Ordering::Equal));
        if tx > ty {
            inner.reverse();
        }
        Some(inner.into_iter().map(|(_, id)| id).collect())
    };
    let mut out: Vec<[u32; 3]> = Vec::with_capacity(triangles.len());
    let mut stack: Vec<[u32; 3]> = core::mem::take(triangles);
    let mut budget = stack.len() * 64 + 1024;
    while let Some(t) = stack.pop() {
        let mut split = false;
        for i in 0..3 {
            let (x, y, z) = (t[i], t[(i + 1) % 3], t[(i + 2) % 3]);
            if let Some(pts) = between(x, y) {
                let mut prev = x;
                for &p in &pts {
                    stack.push([prev, p, z]);
                    prev = p;
                }
                stack.push([prev, y, z]);
                split = true;
                break;
            }
        }
        if !split {
            out.push(t);
        }
        budget -= 1;
        if budget == 0 {
            out.append(&mut stack);
            break;
        }
    }
    *triangles = out;
}
