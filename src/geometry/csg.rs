//! Mesh–mesh Boolean evaluation on closed triangle meshes.
//!
//! `IfcBooleanResult` composes two solids as point sets (half-space
//! clipping digest §5.1): `UNION` = A ∪ B, `INTERSECTION` = A ∩ B,
//! `DIFFERENCE` = A − B. With both operands given as closed, outward-
//! wound boundary meshes, the boundary of the result is assembled from
//! pieces of the operand boundaries:
//!
//! * ∂(A ∪ B) = (∂A outside B) ∪ (∂B outside A)
//! * ∂(A ∩ B) = (∂A inside B) ∪ (∂B inside A)
//! * ∂(A − B) = (∂A outside B) ∪ (∂B inside A, reversed)
//!
//! "Inside" / "outside" is decided with a binary space partition of the
//! *other* operand's faces: every face plane halves space, and a
//! polygon pushed down the tree is split wherever it spans a plane
//! until it lands in a leaf cell lying entirely inside or outside the
//! solid. A polygon coplanar with a partition plane is classified by
//! normal agreement — with the plane's normal it counts as facing out
//! of the solid (kept as "outside"), against it as facing in ("inside")
//! — which is what makes coincident faces resolve: two coincident
//! same-facing faces keep exactly one copy, two opposite-facing ones
//! (solids touching along a face) dissolve both.
//!
//! Polygons stay convex n-gons throughout (an operand triangle cut by
//! planes remains convex), so no single polygon ever carries a
//! T-junction; the seams between fragments of *different* polygons
//! along the intersection curve do — the two sides subdivide the
//! shared segment at different points. [`stitch`] closes them: the
//! vertices are welded by position and every vertex lying strictly
//! inside an unbalanced edge splits that edge, after which every edge
//! of the result pairs up with its reverse and the mesh is watertight
//! again (a test-pinned invariant, alongside the exact volume identity
//! vol(A − B) + vol(A ∩ B) = vol(A)).
//!
//! Everything is bounded: the total number of polygon fragments the
//! partition may create is capped, so a hostile operand pair (many
//! mutually spanning slivers) surfaces as an error instead of running
//! away.

use super::{GeometryError, TriMesh};

/// The three regularised set operations of `IfcBooleanOperator`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOperator {
    /// Points in either operand.
    Union,
    /// Points in both operands.
    Intersection,
    /// Points in the first operand but not the second.
    Difference,
}

/// Largest number of polygon fragments one evaluation may create
/// (input polygons plus every split). Beyond this the evaluation is
/// abandoned.
const MAX_CSG_POLYGONS: usize = 1 << 19;

/// Largest number of triangles the seam stitching may grow the result
/// to (relative to its input) before it stops splitting.
const MAX_STITCH_GROWTH: usize = 8;

/// `true` when every directed edge of `mesh` is balanced by its
/// reverse — the mesh is a closed (watertight) surface.
pub(super) fn is_closed(mesh: &TriMesh) -> bool {
    if mesh.is_empty() {
        return false;
    }
    let mut net: std::collections::HashMap<(u32, u32), i32> = std::collections::HashMap::new();
    for t in &mesh.triangles {
        for i in 0..3 {
            let a = t[i];
            let b = t[(i + 1) % 3];
            if a < b {
                *net.entry((a, b)).or_insert(0) += 1;
            } else {
                *net.entry((b, a)).or_insert(0) -= 1;
            }
        }
    }
    net.values().all(|&n| n == 0)
}

/// Largest absolute coordinate of both meshes (≥ 1), the scale every
/// tolerance is relative to.
fn scale_of(a: &TriMesh, b: &TriMesh) -> f64 {
    a.positions
        .iter()
        .chain(b.positions.iter())
        .map(|p| p[0].abs().max(p[1].abs()).max(p[2].abs()))
        .filter(|s| s.is_finite())
        .fold(1.0f64, f64::max)
}

// ---------------------------------------------------------------------
// Planes and polygons
// ---------------------------------------------------------------------

/// An oriented plane `n · p = w` with unit normal `n`.
#[derive(Debug, Clone, Copy)]
struct Plane {
    n: [f64; 3],
    w: f64,
}

impl Plane {
    fn through(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> Option<Self> {
        let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let n = [
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        ];
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if len <= 0.0 || !len.is_finite() {
            return None;
        }
        let n = [n[0] / len, n[1] / len, n[2] / len];
        Some(Self {
            n,
            w: n[0] * a[0] + n[1] * a[1] + n[2] * a[2],
        })
    }

    fn flip(&mut self) {
        self.n = [-self.n[0], -self.n[1], -self.n[2]];
        self.w = -self.w;
    }

    fn distance(&self, p: [f64; 3]) -> f64 {
        self.n[0] * p[0] + self.n[1] * p[1] + self.n[2] * p[2] - self.w
    }
}

/// A convex polygon with its supporting plane (normal = winding).
#[derive(Debug, Clone)]
struct Poly {
    verts: Vec<[f64; 3]>,
    plane: Plane,
}

impl Poly {
    fn flip(&mut self) {
        self.verts.reverse();
        self.plane.flip();
    }
}

const COPLANAR: u8 = 0;
const FRONT: u8 = 1;
const BACK: u8 = 2;
const SPANNING: u8 = 3;

/// Classify `poly` against `plane` and route it: coplanar polygons go
/// to `coplanar_front` / `coplanar_back` by normal agreement, polygons
/// entirely on one side to `front` / `back`, and a spanning polygon is
/// cut along the plane into one piece for each side.
fn split_poly(
    plane: &Plane,
    poly: Poly,
    eps: f64,
    coplanar_front: &mut Vec<Poly>,
    coplanar_back: &mut Vec<Poly>,
    front: &mut Vec<Poly>,
    back: &mut Vec<Poly>,
) -> usize {
    let n = poly.verts.len();
    let mut kinds: Vec<u8> = Vec::with_capacity(n);
    let mut poly_kind = COPLANAR;
    for &v in &poly.verts {
        let d = plane.distance(v);
        let k = if d < -eps {
            BACK
        } else if d > eps {
            FRONT
        } else {
            COPLANAR
        };
        poly_kind |= k;
        kinds.push(k);
    }
    match poly_kind {
        COPLANAR => {
            let agree = plane.n[0] * poly.plane.n[0]
                + plane.n[1] * poly.plane.n[1]
                + plane.n[2] * poly.plane.n[2];
            if agree > 0.0 {
                coplanar_front.push(poly);
            } else {
                coplanar_back.push(poly);
            }
            0
        }
        FRONT => {
            front.push(poly);
            0
        }
        BACK => {
            back.push(poly);
            0
        }
        _ => {
            let mut f: Vec<[f64; 3]> = Vec::with_capacity(n + 1);
            let mut b: Vec<[f64; 3]> = Vec::with_capacity(n + 1);
            for i in 0..n {
                let j = (i + 1) % n;
                let (ki, kj) = (kinds[i], kinds[j]);
                let (vi, vj) = (poly.verts[i], poly.verts[j]);
                if ki != BACK {
                    f.push(vi);
                }
                if ki != FRONT {
                    b.push(vi);
                }
                if (ki | kj) == SPANNING {
                    let e = [vj[0] - vi[0], vj[1] - vi[1], vj[2] - vi[2]];
                    let denom = plane.n[0] * e[0] + plane.n[1] * e[1] + plane.n[2] * e[2];
                    let t = if denom.abs() > 0.0 {
                        ((plane.w - (plane.n[0] * vi[0] + plane.n[1] * vi[1] + plane.n[2] * vi[2]))
                            / denom)
                            .clamp(0.0, 1.0)
                    } else {
                        0.5
                    };
                    let v = [vi[0] + t * e[0], vi[1] + t * e[1], vi[2] + t * e[2]];
                    f.push(v);
                    b.push(v);
                }
            }
            let mut made = 0usize;
            if f.len() >= 3 {
                front.push(Poly {
                    verts: f,
                    plane: poly.plane,
                });
                made += 1;
            }
            if b.len() >= 3 {
                back.push(Poly {
                    verts: b,
                    plane: poly.plane,
                });
                made += 1;
            }
            made.saturating_sub(1)
        }
    }
}

// ---------------------------------------------------------------------
// Binary space partition (arena-allocated, iterative walks)
// ---------------------------------------------------------------------

#[derive(Default)]
struct Node {
    plane: Option<Plane>,
    front: Option<usize>,
    back: Option<usize>,
    polys: Vec<Poly>,
}

struct Bsp {
    nodes: Vec<Node>,
}

/// A shared fragment budget across both partitions.
struct Budget {
    remaining: usize,
}

impl Budget {
    fn spend(&mut self, n: usize) -> Result<(), GeometryError> {
        if n > self.remaining {
            return Err(GeometryError::Unsupported("IFCBOOLEANRESULT".to_string()));
        }
        self.remaining -= n;
        Ok(())
    }
}

impl Bsp {
    fn new(polys: Vec<Poly>, eps: f64, budget: &mut Budget) -> Result<Self, GeometryError> {
        let mut bsp = Self {
            nodes: vec![Node::default()],
        };
        bsp.build(0, polys, eps, budget)?;
        Ok(bsp)
    }

    /// Insert `polys` under node `root`, growing the tree as needed.
    fn build(
        &mut self,
        root: usize,
        polys: Vec<Poly>,
        eps: f64,
        budget: &mut Budget,
    ) -> Result<(), GeometryError> {
        let mut stack: Vec<(usize, Vec<Poly>)> = vec![(root, polys)];
        while let Some((idx, polys)) = stack.pop() {
            if polys.is_empty() {
                continue;
            }
            if self.nodes[idx].plane.is_none() {
                self.nodes[idx].plane = Some(polys[0].plane);
            }
            let plane = self.nodes[idx].plane.expect("set above");
            let mut f: Vec<Poly> = Vec::new();
            let mut b: Vec<Poly> = Vec::new();
            let mut own: Vec<Poly> = std::mem::take(&mut self.nodes[idx].polys);
            let mut created = 0usize;
            for p in polys {
                // Coplanar polygons of either facing live at this node.
                let mut cb: Vec<Poly> = Vec::new();
                created += split_poly(&plane, p, eps, &mut own, &mut cb, &mut f, &mut b);
                own.extend(cb);
            }
            budget.spend(created)?;
            self.nodes[idx].polys = own;
            if !f.is_empty() {
                let child = match self.nodes[idx].front {
                    Some(c) => c,
                    None => {
                        self.nodes.push(Node::default());
                        let c = self.nodes.len() - 1;
                        self.nodes[idx].front = Some(c);
                        c
                    }
                };
                stack.push((child, f));
            }
            if !b.is_empty() {
                let child = match self.nodes[idx].back {
                    Some(c) => c,
                    None => {
                        self.nodes.push(Node::default());
                        let c = self.nodes.len() - 1;
                        self.nodes[idx].back = Some(c);
                        c
                    }
                };
                stack.push((child, b));
            }
        }
        Ok(())
    }

    /// Swap solid and empty space.
    fn invert(&mut self) {
        for node in &mut self.nodes {
            for p in &mut node.polys {
                p.flip();
            }
            if let Some(pl) = &mut node.plane {
                pl.flip();
            }
            core::mem::swap(&mut node.front, &mut node.back);
        }
    }

    /// Remove every part of `polys` that lies inside this solid.
    fn clip_polys(
        &self,
        polys: Vec<Poly>,
        eps: f64,
        budget: &mut Budget,
    ) -> Result<Vec<Poly>, GeometryError> {
        let mut out: Vec<Poly> = Vec::new();
        let mut stack: Vec<(usize, Vec<Poly>)> = vec![(0, polys)];
        while let Some((idx, polys)) = stack.pop() {
            let node = &self.nodes[idx];
            let Some(plane) = node.plane else {
                out.extend(polys);
                continue;
            };
            let mut f: Vec<Poly> = Vec::new();
            let mut b: Vec<Poly> = Vec::new();
            let mut created = 0usize;
            for p in polys {
                // Coplanar-front → front (outside), coplanar-back →
                // back (inside): the normal-agreement rule.
                let mut cf: Vec<Poly> = Vec::new();
                let mut cb: Vec<Poly> = Vec::new();
                created += split_poly(&plane, p, eps, &mut cf, &mut cb, &mut f, &mut b);
                f.extend(cf);
                b.extend(cb);
            }
            budget.spend(created)?;
            match node.front {
                Some(c) => stack.push((c, f)),
                None => out.extend(f),
            }
            if let Some(c) = node.back {
                stack.push((c, b));
            }
            // No back child: `b` is inside the solid — dropped.
        }
        Ok(out)
    }

    /// Remove every part of this tree's polygons inside `other`.
    fn clip_to(&mut self, other: &Bsp, eps: f64, budget: &mut Budget) -> Result<(), GeometryError> {
        for i in 0..self.nodes.len() {
            let polys = std::mem::take(&mut self.nodes[i].polys);
            self.nodes[i].polys = other.clip_polys(polys, eps, budget)?;
        }
        Ok(())
    }

    fn all_polys(&self) -> Vec<Poly> {
        self.nodes
            .iter()
            .flat_map(|n| n.polys.iter().cloned())
            .collect()
    }
}

/// The triangles of `mesh` as polygons (degenerate ones dropped).
fn polys_of(mesh: &TriMesh) -> Vec<Poly> {
    let mut out = Vec::with_capacity(mesh.triangles.len());
    for t in &mesh.triangles {
        let Some((a, b, c)) = (|| {
            Some((
                *mesh.positions.get(t[0] as usize)?,
                *mesh.positions.get(t[1] as usize)?,
                *mesh.positions.get(t[2] as usize)?,
            ))
        })() else {
            continue;
        };
        if let Some(plane) = Plane::through(a, b, c) {
            out.push(Poly {
                verts: vec![a, b, c],
                plane,
            });
        }
    }
    out
}

/// Fan-triangulate convex polygons into a mesh.
fn mesh_of(polys: &[Poly]) -> TriMesh {
    let mut mesh = TriMesh::default();
    for p in polys {
        let base = mesh.positions.len() as u32;
        mesh.positions.extend_from_slice(&p.verts);
        for k in 1..(p.verts.len() as u32 - 1) {
            mesh.triangles.push([base, base + k, base + k + 1]);
        }
    }
    mesh
}

/// Evaluate `a <op> b` on two closed, outward-wound meshes. The result
/// is stitched watertight (see the module documentation); an evaluation
/// exceeding the fragment budget is `Unsupported("IFCBOOLEANRESULT")`.
///
/// Open (non-closed) inputs are not rejected — the classification is
/// still well defined for the fragments that meet the other operand's
/// partition — but only closed inputs yield a closed result.
pub fn mesh_boolean(
    a: &TriMesh,
    b: &TriMesh,
    op: BooleanOperator,
) -> Result<TriMesh, GeometryError> {
    let scale = scale_of(a, b);
    let eps = 1e-9 * scale;
    let mut budget = Budget {
        remaining: MAX_CSG_POLYGONS,
    };
    let pa = polys_of(a);
    let pb = polys_of(b);
    budget.spend(pa.len() + pb.len())?;
    let mut ta = Bsp::new(pa, eps, &mut budget)?;
    let mut tb = Bsp::new(pb, eps, &mut budget)?;
    let polys = match op {
        BooleanOperator::Union => {
            // Keep ∂A outside B and ∂B outside A; the extra inverted
            // clip of B drops B's faces coincident with A's (which A
            // already contributes once).
            ta.clip_to(&tb, eps, &mut budget)?;
            tb.clip_to(&ta, eps, &mut budget)?;
            tb.invert();
            tb.clip_to(&ta, eps, &mut budget)?;
            tb.invert();
            ta.build(0, tb.all_polys(), eps, &mut budget)?;
            ta.all_polys()
        }
        BooleanOperator::Difference => {
            // A − B = ¬(¬A ∪ B).
            ta.invert();
            ta.clip_to(&tb, eps, &mut budget)?;
            tb.clip_to(&ta, eps, &mut budget)?;
            tb.invert();
            tb.clip_to(&ta, eps, &mut budget)?;
            tb.invert();
            ta.build(0, tb.all_polys(), eps, &mut budget)?;
            ta.invert();
            ta.all_polys()
        }
        BooleanOperator::Intersection => {
            // A ∩ B = ¬(¬A ∪ ¬B).
            ta.invert();
            tb.clip_to(&ta, eps, &mut budget)?;
            tb.invert();
            ta.clip_to(&tb, eps, &mut budget)?;
            tb.clip_to(&ta, eps, &mut budget)?;
            ta.build(0, tb.all_polys(), eps, &mut budget)?;
            ta.invert();
            ta.all_polys()
        }
    };
    let mut mesh = mesh_of(&polys);
    stitch(&mut mesh, 1e-8 * scale);
    Ok(mesh)
}

// ---------------------------------------------------------------------
// Seam stitching: position welding + T-junction splitting
// ---------------------------------------------------------------------

/// Weld coincident vertices (within `tol`), drop degenerate triangles,
/// and split every unbalanced edge at the vertices lying strictly
/// inside it, so fragments meeting along the intersection curve share
/// their vertices edge for edge.
pub(super) fn stitch(mesh: &mut TriMesh, tol: f64) {
    weld(mesh, tol);
    let limit = mesh
        .triangles
        .len()
        .saturating_mul(MAX_STITCH_GROWTH)
        .saturating_add(1024);
    for _ in 0..8 {
        let unbalanced = unbalanced_edges(mesh);
        if unbalanced.is_empty() {
            break;
        }
        if !split_t_junctions(mesh, &unbalanced, tol, limit) {
            break;
        }
    }
    // Splitting may have left slivers whose corners coincide.
    mesh.triangles
        .retain(|t| t[0] != t[1] && t[1] != t[2] && t[2] != t[0]);
}

/// Merge vertices closer than `tol`, re-indexing the triangles and
/// dropping the ones that collapse.
fn weld(mesh: &mut TriMesh, tol: f64) {
    use std::collections::HashMap;
    let cell = (tol * 4.0).max(f64::MIN_POSITIVE);
    let key_of = |p: [f64; 3]| -> [i64; 3] {
        [
            (p[0] / cell).floor() as i64,
            (p[1] / cell).floor() as i64,
            (p[2] / cell).floor() as i64,
        ]
    };
    let mut grid: HashMap<[i64; 3], Vec<u32>> = HashMap::new();
    let mut positions: Vec<[f64; 3]> = Vec::with_capacity(mesh.positions.len());
    let mut remap: Vec<u32> = Vec::with_capacity(mesh.positions.len());
    let tol2 = tol * tol;
    for &p in &mesh.positions {
        if !p.iter().all(|c| c.is_finite()) {
            // Keep a non-finite vertex unmerged; its triangles are
            // dropped below.
            remap.push(positions.len() as u32);
            positions.push(p);
            continue;
        }
        let k = key_of(p);
        let mut found: Option<u32> = None;
        'search: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(list) = grid.get(&[k[0] + dx, k[1] + dy, k[2] + dz]) {
                        for &i in list {
                            let q = positions[i as usize];
                            let d = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
                            if d[0] * d[0] + d[1] * d[1] + d[2] * d[2] <= tol2 {
                                found = Some(i);
                                break 'search;
                            }
                        }
                    }
                }
            }
        }
        let idx = match found {
            Some(i) => i,
            None => {
                let i = positions.len() as u32;
                positions.push(p);
                grid.entry(k).or_default().push(i);
                i
            }
        };
        remap.push(idx);
    }
    let finite = |i: u32| positions[i as usize].iter().all(|c| c.is_finite());
    let mut triangles: Vec<[u32; 3]> = Vec::with_capacity(mesh.triangles.len());
    for t in &mesh.triangles {
        let r = [
            remap[t[0] as usize],
            remap[t[1] as usize],
            remap[t[2] as usize],
        ];
        if r[0] == r[1] || r[1] == r[2] || r[2] == r[0] {
            continue;
        }
        if !(finite(r[0]) && finite(r[1]) && finite(r[2])) {
            continue;
        }
        triangles.push(r);
    }
    mesh.positions = positions;
    mesh.triangles = triangles;
}

/// The undirected edges whose directed traversals do not cancel.
fn unbalanced_edges(mesh: &TriMesh) -> std::collections::HashSet<(u32, u32)> {
    let mut net: std::collections::HashMap<(u32, u32), i32> = std::collections::HashMap::new();
    for t in &mesh.triangles {
        for i in 0..3 {
            let a = t[i];
            let b = t[(i + 1) % 3];
            if a < b {
                *net.entry((a, b)).or_insert(0) += 1;
            } else {
                *net.entry((b, a)).or_insert(0) -= 1;
            }
        }
    }
    net.into_iter()
        .filter(|&(_, n)| n != 0)
        .map(|(e, _)| e)
        .collect()
}

/// Split every triangle edge in `unbalanced` at the vertices (drawn
/// from the unbalanced edges' own endpoints) lying strictly inside it.
/// Returns whether anything changed.
fn split_t_junctions(
    mesh: &mut TriMesh,
    unbalanced: &std::collections::HashSet<(u32, u32)>,
    tol: f64,
    limit: usize,
) -> bool {
    // Candidate vertices sorted by x for range lookups.
    let mut candidates: Vec<u32> = unbalanced
        .iter()
        .flat_map(|&(a, b)| [a, b])
        .collect::<std::collections::HashSet<u32>>()
        .into_iter()
        .collect();
    candidates.sort_unstable_by(|&i, &j| {
        let (x, y) = (mesh.positions[i as usize][0], mesh.positions[j as usize][0]);
        x.partial_cmp(&y).unwrap_or(core::cmp::Ordering::Equal)
    });
    let xs: Vec<f64> = candidates
        .iter()
        .map(|&i| mesh.positions[i as usize][0])
        .collect();
    let positions = &mesh.positions;
    // Interior points of the segment a → b, as (fraction, vertex).
    let points_on = |a: u32, b: u32| -> Vec<(f64, u32)> {
        let (pa, pb) = (positions[a as usize], positions[b as usize]);
        let e = [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]];
        let len2 = e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
        if len2 <= 0.0 || !len2.is_finite() {
            return Vec::new();
        }
        let len = len2.sqrt();
        let (x0, x1) = (pa[0].min(pb[0]) - tol, pa[0].max(pb[0]) + tol);
        let start = xs.partition_point(|&x| x < x0);
        let mut out: Vec<(f64, u32)> = Vec::new();
        for k in start..candidates.len() {
            if xs[k] > x1 {
                break;
            }
            let v = candidates[k];
            if v == a || v == b {
                continue;
            }
            let p = positions[v as usize];
            let d = [p[0] - pa[0], p[1] - pa[1], p[2] - pa[2]];
            let t = (d[0] * e[0] + d[1] * e[1] + d[2] * e[2]) / len2;
            // Strictly interior by at least `tol` along the edge.
            if t * len <= tol || (1.0 - t) * len <= tol {
                continue;
            }
            let q = [d[0] - t * e[0], d[1] - t * e[1], d[2] - t * e[2]];
            if q[0] * q[0] + q[1] * q[1] + q[2] * q[2] <= tol * tol {
                out.push((t, v));
            }
        }
        out.sort_by(|p, q| p.0.partial_cmp(&q.0).unwrap_or(core::cmp::Ordering::Equal));
        out.dedup_by_key(|p| p.1);
        out
    };
    let mut changed = false;
    let mut out: Vec<[u32; 3]> = Vec::with_capacity(mesh.triangles.len());
    let mut stack: Vec<[u32; 3]> = mesh.triangles.iter().rev().copied().collect();
    while let Some(t) = stack.pop() {
        if out.len() + stack.len() > limit {
            // Give up splitting: emit what is left unchanged.
            out.push(t);
            out.append(&mut stack);
            break;
        }
        let mut split = false;
        for i in 0..3 {
            let (a, b, c) = (t[i], t[(i + 1) % 3], t[(i + 2) % 3]);
            let key = if a < b { (a, b) } else { (b, a) };
            if !unbalanced.contains(&key) {
                continue;
            }
            let pts = points_on(a, b);
            if pts.is_empty() {
                continue;
            }
            let mut prev = a;
            for &(_, v) in &pts {
                stack.push([prev, v, c]);
                prev = v;
            }
            stack.push([prev, b, c]);
            split = true;
            changed = true;
            break;
        }
        if !split {
            out.push(t);
        }
    }
    mesh.triangles = out;
    changed
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube(min: [f64; 3], max: [f64; 3]) -> TriMesh {
        super::super::box_mesh(min, max)
    }

    /// A 48-gon cylinder of radius `r`, axis z ∈ [−h/2, h/2].
    fn cylinder(r: f64, h: f64) -> TriMesh {
        let n = 48u32;
        let mut positions = Vec::new();
        for level in [-h / 2.0, h / 2.0] {
            for k in 0..n {
                let a = 2.0 * core::f64::consts::PI * (k as f64) / (n as f64);
                positions.push([r * a.cos(), r * a.sin(), level]);
            }
        }
        let mut triangles = Vec::new();
        for k in 0..n {
            let k1 = (k + 1) % n;
            triangles.push([k, k1, n + k1]);
            triangles.push([k, n + k1, n + k]);
        }
        for k in 1..(n - 1) {
            triangles.push([0, k + 1, k]);
            triangles.push([n, n + k, n + k + 1]);
        }
        TriMesh {
            positions,
            triangles,
        }
    }

    fn assert_closed(m: &TriMesh) {
        assert!(is_closed(m), "mesh is not watertight");
    }

    #[test]
    fn overlapping_cubes_conserve_volume() {
        let a = cube([0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
        let b = cube([1.0, 1.0, 1.0], [3.0, 3.0, 3.0]);
        let d = mesh_boolean(&a, &b, BooleanOperator::Difference).unwrap();
        let i = mesh_boolean(&a, &b, BooleanOperator::Intersection).unwrap();
        let u = mesh_boolean(&a, &b, BooleanOperator::Union).unwrap();
        assert_closed(&d);
        assert_closed(&i);
        assert_closed(&u);
        assert!(
            (d.signed_volume() - 7.0).abs() < 1e-9,
            "{}",
            d.signed_volume()
        );
        assert!(
            (i.signed_volume() - 1.0).abs() < 1e-9,
            "{}",
            i.signed_volume()
        );
        assert!(
            (u.signed_volume() - 15.0).abs() < 1e-9,
            "{}",
            u.signed_volume()
        );
    }

    #[test]
    fn coincident_cubes() {
        let a = cube([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let d = mesh_boolean(&a, &a, BooleanOperator::Difference).unwrap();
        assert!(d.is_empty(), "{} triangles", d.triangle_count());
        let i = mesh_boolean(&a, &a, BooleanOperator::Intersection).unwrap();
        assert_closed(&i);
        assert!((i.signed_volume() - 1.0).abs() < 1e-9);
        let u = mesh_boolean(&a, &a, BooleanOperator::Union).unwrap();
        assert_closed(&u);
        assert!((u.signed_volume() - 1.0).abs() < 1e-9);
        assert_eq!(u.triangle_count(), 12);
    }

    #[test]
    fn stacked_cubes_touching_along_a_face() {
        let a = cube([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = cube([0.0, 0.0, 1.0], [1.0, 1.0, 2.0]);
        let u = mesh_boolean(&a, &b, BooleanOperator::Union).unwrap();
        assert_closed(&u);
        assert!((u.signed_volume() - 2.0).abs() < 1e-9);
        // The shared face dissolved: no triangle lies in z = 1.
        assert!(u.triangles.iter().all(|t| {
            !t.iter()
                .all(|&i| (u.positions[i as usize][2] - 1.0).abs() < 1e-12)
        }));
        let d = mesh_boolean(&a, &b, BooleanOperator::Difference).unwrap();
        assert_closed(&d);
        assert!((d.signed_volume() - 1.0).abs() < 1e-9);
        let i = mesh_boolean(&a, &b, BooleanOperator::Intersection).unwrap();
        assert!(i.is_empty());
    }

    #[test]
    fn through_hole_in_a_slab() {
        // A 4×4×1 slab minus a 1×1 column through it: a genuine hole
        // (genus 1), watertight, volume 16 − 1.
        let a = cube([0.0, 0.0, 0.0], [4.0, 4.0, 1.0]);
        let b = cube([1.5, 1.5, -1.0], [2.5, 2.5, 2.0]);
        let d = mesh_boolean(&a, &b, BooleanOperator::Difference).unwrap();
        assert_closed(&d);
        assert!(
            (d.signed_volume() - 15.0).abs() < 1e-9,
            "{}",
            d.signed_volume()
        );
        let i = mesh_boolean(&a, &b, BooleanOperator::Intersection).unwrap();
        assert_closed(&i);
        assert!((i.signed_volume() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn disjoint_operands() {
        let a = cube([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = cube([5.0, 0.0, 0.0], [6.0, 1.0, 1.0]);
        let d = mesh_boolean(&a, &b, BooleanOperator::Difference).unwrap();
        assert!((d.signed_volume() - 1.0).abs() < 1e-9);
        let i = mesh_boolean(&a, &b, BooleanOperator::Intersection).unwrap();
        assert!(i.is_empty());
        let u = mesh_boolean(&a, &b, BooleanOperator::Union).unwrap();
        assert_closed(&u);
        assert!((u.signed_volume() - 2.0).abs() < 1e-9);
    }

    #[test]
    fn sphere_minus_cylinder_partitions_exactly() {
        // Curved operands (thousands of mutually spanning triangles):
        // A − B and A ∩ B stay watertight and partition A exactly.
        let sphere = super::super::sphere_mesh(3.0);
        let mut cyl = cylinder(1.0, 10.0);
        // Tilt the cylinder so no face is axis-aligned with the sphere's
        // pole fans.
        for p in &mut cyl.positions {
            let (y, z) = (p[1], p[2]);
            p[1] = 0.8 * y - 0.6 * z;
            p[2] = 0.6 * y + 0.8 * z;
        }
        let d = mesh_boolean(&sphere, &cyl, BooleanOperator::Difference).unwrap();
        let i = mesh_boolean(&sphere, &cyl, BooleanOperator::Intersection).unwrap();
        assert_closed(&d);
        assert_closed(&i);
        let total = d.signed_volume() + i.signed_volume();
        let want = sphere.signed_volume();
        assert!((total - want).abs() < 1e-9 * want, "{total} != {want}");
        assert!(i.signed_volume() > 0.9 * core::f64::consts::PI * 6.0);
    }

    #[test]
    fn stitch_closes_a_t_junction() {
        // Two squares meeting along an edge one of them subdivides.
        let mut m = TriMesh {
            positions: vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, -1.0, 0.0],
                [1.0, -1.0, 0.0],
                [0.5, 0.0, 0.0],
                [0.5, 0.0, 1e-10],
            ],
            triangles: vec![[0, 1, 2], [0, 2, 3], [4, 5, 1], [4, 1, 6], [4, 7, 0]],
        };
        stitch(&mut m, 1e-8);
        assert_eq!(m.vertex_count(), 7);
        // Every interior edge is now balanced; the outer boundary of
        // the 2×1 sheet remains open (6 boundary edges).
        let open = unbalanced_edges(&m);
        assert_eq!(open.len(), 6, "{open:?}");
    }
}
