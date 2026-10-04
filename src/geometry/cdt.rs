//! Constrained Delaunay triangulation of a polygon with holes.
//!
//! The textbook incremental construction (Lawson's flip algorithm, with
//! constraint recovery by edge flipping after Sloan, "A fast algorithm
//! for generating constrained Delaunay triangulations", 1993):
//!
//! 1. the points are scaled into the unit square, wrapped in a large
//!    super-triangle, and inserted one by one in Morton (z-curve) order
//!    — each located by walking from the previous one, splitting the
//!    triangle (or edge) it falls in, and restoring the Delaunay
//!    property by flipping the edges opposite it;
//! 2. every boundary segment is then forced into the triangulation:
//!    the edges it crosses are flipped away (a flip is only taken when
//!    the quadrilateral is strictly convex; others are retried) until
//!    the segment is an edge, which is then marked constrained; a
//!    segment passing exactly through another input point is split
//!    there;
//! 3. the triangles are classified by flood fill from the super-
//!    triangle: crossing a constrained edge toggles inside / outside, so
//!    the region is the odd-depth set (inside the outer ring, outside
//!    every hole).
//!
//! Duplicate input points are merged onto their first occurrence. All
//! loops are bounded (walk steps, flip budget), and any failure (self-
//! intersecting rings, a budget exhausted) is reported so the caller can
//! fall back to ear clipping. Unlike ear clipping with hole bridges this
//! is `O(n log n)` in practice and handles any number of holes, holes
//! touching each other or the outer ring at a vertex, and rings sharing
//! vertices without bridge selection.

use std::collections::HashMap;

const NONE: u32 = u32::MAX;

/// Why the triangulation could not be built (the caller falls back).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct CdtError;

struct Mesh {
    pts: Vec<[f64; 2]>,
    tri: Vec<[u32; 3]>,
    /// `adj[t][i]`: the triangle across edge `i` = (`tri[t][i]`,
    /// `tri[t][(i + 1) % 3]`), or `NONE`.
    adj: Vec<[u32; 3]>,
    /// Constrained flags per triangle edge.
    con: Vec<[bool; 3]>,
    /// One triangle incident to each vertex.
    vtri: Vec<u32>,
    last: u32,
    eps: f64,
}

fn orient(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// Positive when `d` is strictly inside the circumcircle of the
/// counter-clockwise triangle `a b c`, beyond a margin relative to the
/// configuration's size (cocircular points — a sampled circle, a grid —
/// never flip back and forth).
fn incircle(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> f64 {
    let (ax, ay) = (a[0] - d[0], a[1] - d[1]);
    let (bx, by) = (b[0] - d[0], b[1] - d[1]);
    let (cx, cy) = (c[0] - d[0], c[1] - d[1]);
    let (a2, b2, c2) = (ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy);
    let det = ax * (by * c2 - b2 * cy) - ay * (bx * c2 - b2 * cx) + a2 * (bx * cy - by * cx);
    let size2 = a2 + b2 + c2;
    det - 1e-10 * size2 * size2
}

/// Interleave the bits of two 16-bit grid coordinates.
fn morton(x: u32, y: u32) -> u64 {
    fn spread(mut v: u64) -> u64 {
        v &= 0xFFFF_FFFF;
        v = (v | (v << 16)) & 0x0000_FFFF_0000_FFFF;
        v = (v | (v << 8)) & 0x00FF_00FF_00FF_00FF;
        v = (v | (v << 4)) & 0x0F0F_0F0F_0F0F_0F0F;
        v = (v | (v << 2)) & 0x3333_3333_3333_3333;
        v = (v | (v << 1)) & 0x5555_5555_5555_5555;
        v
    }
    spread(x as u64) | (spread(y as u64) << 1)
}

impl Mesh {
    fn edge_index(&self, t: u32, a: u32, b: u32) -> Option<usize> {
        let v = self.tri[t as usize];
        (0..3).find(|&i| v[i] == a && v[(i + 1) % 3] == b)
    }

    /// Point the neighbour `n` of `t` across edge (a, b) back at `t`
    /// (through its edge (b, a)).
    fn link(&mut self, t: u32, a: u32, b: u32, n: u32) {
        if n == NONE {
            return;
        }
        if let Some(j) = self.edge_index(n, b, a) {
            self.adj[n as usize][j] = t;
        }
    }

    fn push(&mut self, v: [u32; 3]) -> u32 {
        let t = self.tri.len() as u32;
        self.tri.push(v);
        self.adj.push([NONE; 3]);
        self.con.push([false; 3]);
        for &x in &v {
            self.vtri[x as usize] = t;
        }
        t
    }

    fn write(&mut self, t: u32, v: [u32; 3]) {
        self.tri[t as usize] = v;
        for &x in &v {
            self.vtri[x as usize] = t;
        }
    }

    /// Locate the triangle containing `p` (walking), returning the
    /// triangle and, when `p` lies on one of its edges, that edge index.
    fn locate(&self, p: [f64; 2]) -> Result<(u32, Option<usize>), CdtError> {
        let mut t = self.last;
        let n = self.tri.len();
        let mut steps = 0usize;
        'walk: loop {
            steps += 1;
            if steps > 4 * n + 64 {
                break;
            }
            let v = self.tri[t as usize];
            let mut on: Option<usize> = None;
            for i in 0..3 {
                let (a, b) = (self.pts[v[i] as usize], self.pts[v[(i + 1) % 3] as usize]);
                let o = orient(a, b, p);
                if o < -self.eps {
                    let nb = self.adj[t as usize][i];
                    if nb == NONE {
                        return Err(CdtError);
                    }
                    t = nb;
                    continue 'walk;
                }
                if o <= self.eps {
                    on = Some(i);
                }
            }
            return Ok((t, on));
        }
        // Walk failed to settle (degenerate predicates): exhaustive scan.
        for (ti, v) in self.tri.iter().enumerate() {
            let mut inside = true;
            let mut on = None;
            for i in 0..3 {
                let o = orient(
                    self.pts[v[i] as usize],
                    self.pts[v[(i + 1) % 3] as usize],
                    p,
                );
                if o < -self.eps {
                    inside = false;
                    break;
                }
                if o <= self.eps {
                    on = Some(i);
                }
            }
            if inside {
                return Ok((ti as u32, on));
            }
        }
        Err(CdtError)
    }

    /// Insert vertex `p` (already in `pts`), then legalise.
    fn insert(&mut self, p: u32) -> Result<(), CdtError> {
        let pp = self.pts[p as usize];
        let (t, on) = self.locate(pp)?;
        let mut stack: Vec<(u32, usize)> = Vec::new();
        match on {
            None => {
                let [a, b, c] = self.tri[t as usize];
                let [na, nb, nc] = self.adj[t as usize];
                let [ca, cb, cc] = self.con[t as usize];
                // t → (a, b, p); t1 → (b, c, p); t2 → (c, a, p).
                self.write(t, [a, b, p]);
                let t1 = self.push([b, c, p]);
                let t2 = self.push([c, a, p]);
                self.adj[t as usize] = [na, t1, t2];
                self.adj[t1 as usize] = [nb, t2, t];
                self.adj[t2 as usize] = [nc, t, t1];
                self.con[t as usize] = [ca, false, false];
                self.con[t1 as usize] = [cb, false, false];
                self.con[t2 as usize] = [cc, false, false];
                self.link(t1, b, c, nb);
                self.link(t2, c, a, nc);
                stack.extend([(t, 0), (t1, 0), (t2, 0)]);
            }
            Some(i) => {
                // On edge i of t: split t and its neighbour u.
                let v = self.tri[t as usize];
                let (a, b, c) = (v[i], v[(i + 1) % 3], v[(i + 2) % 3]);
                let u = self.adj[t as usize][i];
                let constrained = self.con[t as usize][i];
                let n_bc = self.adj[t as usize][(i + 1) % 3];
                let n_ca = self.adj[t as usize][(i + 2) % 3];
                let c_bc = self.con[t as usize][(i + 1) % 3];
                let c_ca = self.con[t as usize][(i + 2) % 3];
                // t → (c, a, p), t1 → (b, c, p) (and the neighbour u
                // likewise into (a, d, p) + (d, b, p)).
                self.write(t, [c, a, p]);
                let t1 = self.push([b, c, p]);
                if u == NONE {
                    self.adj[t as usize] = [n_ca, NONE, t1];
                    self.adj[t1 as usize] = [n_bc, t, NONE];
                    self.con[t as usize] = [c_ca, constrained, false];
                    self.con[t1 as usize] = [c_bc, false, constrained];
                    self.link(t1, b, c, n_bc);
                    stack.extend([(t, 0), (t1, 0)]);
                } else {
                    let j = self.edge_index(u, b, a).ok_or(CdtError)?;
                    let w = self.tri[u as usize];
                    let d = w[(j + 2) % 3];
                    let n_ad = self.adj[u as usize][(j + 1) % 3];
                    let n_db = self.adj[u as usize][(j + 2) % 3];
                    let c_ad = self.con[u as usize][(j + 1) % 3];
                    let c_db = self.con[u as usize][(j + 2) % 3];
                    // u → (a, d, p); t3 → (d, b, p).
                    self.write(u, [a, d, p]);
                    let t3 = self.push([d, b, p]);
                    // Edges: t (c,a,p): [ca, a-p, p-c]
                    //        t1 (b,c,p): [bc, c-p, p-b]
                    //        u (a,d,p): [ad, d-p, p-a]
                    //        t3 (d,b,p): [db, b-p, p-d]
                    self.adj[t as usize] = [n_ca, u, t1];
                    self.adj[t1 as usize] = [n_bc, t, t3];
                    self.adj[u as usize] = [n_ad, t3, t];
                    self.adj[t3 as usize] = [n_db, t1, u];
                    self.con[t as usize] = [c_ca, constrained, false];
                    self.con[t1 as usize] = [c_bc, false, constrained];
                    self.con[u as usize] = [c_ad, false, constrained];
                    self.con[t3 as usize] = [c_db, constrained, false];
                    self.link(t1, b, c, n_bc);
                    self.link(t3, d, b, n_db);
                    stack.extend([(t, 0), (t1, 0), (u, 0), (t3, 0)]);
                }
            }
        }
        self.last = t;
        // Legalise the edges opposite p.
        let mut budget = 64 * self.tri.len() + 1024;
        while let Some((t, i)) = stack.pop() {
            budget = budget.checked_sub(1).ok_or(CdtError)?;
            if self.con[t as usize][i] {
                continue;
            }
            let u = self.adj[t as usize][i];
            if u == NONE {
                continue;
            }
            let v = self.tri[t as usize];
            let (a, b, c) = (v[i], v[(i + 1) % 3], v[(i + 2) % 3]);
            if c != p {
                continue;
            }
            let Some(j) = self.edge_index(u, b, a) else {
                continue;
            };
            let d = self.tri[u as usize][(j + 2) % 3];
            let (pa, pb, pc, pd) = (
                self.pts[a as usize],
                self.pts[b as usize],
                self.pts[c as usize],
                self.pts[d as usize],
            );
            if incircle(pa, pb, pc, pd) > 0.0 {
                let (t2, u2) = self.flip(t, i)?;
                // After the flip both triangles contain p; push the
                // edges opposite p.
                for tt in [t2, u2] {
                    let v = self.tri[tt as usize];
                    if let Some(k) = (0..3).find(|&k| v[(k + 2) % 3] == p) {
                        stack.push((tt, k));
                    }
                }
            }
        }
        Ok(())
    }

    /// Flip edge `i` of triangle `t` (shared with its neighbour): the
    /// pair (a b c) + (b a d) becomes (c a d) + (d b c), reusing both
    /// slots.
    fn flip(&mut self, t: u32, i: usize) -> Result<(u32, u32), CdtError> {
        let u = self.adj[t as usize][i];
        if u == NONE {
            return Err(CdtError);
        }
        let v = self.tri[t as usize];
        let (a, b, c) = (v[i], v[(i + 1) % 3], v[(i + 2) % 3]);
        let j = self.edge_index(u, b, a).ok_or(CdtError)?;
        let w = self.tri[u as usize];
        let d = w[(j + 2) % 3];
        let n_bc = self.adj[t as usize][(i + 1) % 3];
        let n_ca = self.adj[t as usize][(i + 2) % 3];
        let n_ad = self.adj[u as usize][(j + 1) % 3];
        let n_db = self.adj[u as usize][(j + 2) % 3];
        let c_bc = self.con[t as usize][(i + 1) % 3];
        let c_ca = self.con[t as usize][(i + 2) % 3];
        let c_ad = self.con[u as usize][(j + 1) % 3];
        let c_db = self.con[u as usize][(j + 2) % 3];
        // (a b c) + (b a d) → t = (c, a, d), u = (d, b, c).
        self.write(t, [c, a, d]);
        self.write(u, [d, b, c]);
        // t edges: c-a (n_ca), a-d (n_ad), d-c (u)
        // u edges: d-b (n_db), b-c (n_bc), c-d (t)
        self.adj[t as usize] = [n_ca, n_ad, u];
        self.adj[u as usize] = [n_db, n_bc, t];
        self.con[t as usize] = [c_ca, c_ad, false];
        self.con[u as usize] = [c_db, c_bc, false];
        self.link(t, a, d, n_ad);
        self.link(u, b, c, n_bc);
        Ok((t, u))
    }

    /// Triangles around vertex `a` (counter-clockwise walk, both ways
    /// when the fan is open).
    fn fan(&self, a: u32) -> Vec<u32> {
        let start = self.vtri[a as usize];
        if start == NONE {
            return Vec::new();
        }
        let mut out = vec![start];
        // Rotate: in triangle t with a at k, the next triangle CCW
        // around a is across edge (k + 2) (the edge ending at a).
        let mut t = start;
        for _ in 0..self.tri.len() {
            let v = self.tri[t as usize];
            let Some(k) = (0..3).find(|&k| v[k] == a) else {
                break;
            };
            let n = self.adj[t as usize][(k + 2) % 3];
            if n == NONE || n == start {
                if n == NONE {
                    // Open fan: walk the other way too.
                    let mut t2 = start;
                    for _ in 0..self.tri.len() {
                        let v = self.tri[t2 as usize];
                        let Some(k) = (0..3).find(|&k| v[k] == a) else {
                            break;
                        };
                        let n = self.adj[t2 as usize][k];
                        if n == NONE || n == start {
                            break;
                        }
                        out.push(n);
                        t2 = n;
                    }
                }
                break;
            }
            out.push(n);
            t = n;
        }
        out
    }

    /// Force segment (a, b) into the triangulation and mark it.
    fn constrain(&mut self, a: u32, b: u32, depth: usize) -> Result<(), CdtError> {
        if a == b {
            return Ok(());
        }
        if depth > 64 {
            return Err(CdtError);
        }
        let (pa, pb) = (self.pts[a as usize], self.pts[b as usize]);
        let mut budget = 16 * self.tri.len() + 4096;
        loop {
            budget = budget.checked_sub(1).ok_or(CdtError)?;
            // Already an edge?
            for t in self.fan(a) {
                let v = self.tri[t as usize];
                for i in 0..3 {
                    if (v[i] == a && v[(i + 1) % 3] == b) || (v[i] == b && v[(i + 1) % 3] == a) {
                        self.con[t as usize][i] = true;
                        let n = self.adj[t as usize][i];
                        if n != NONE {
                            if let Some(j) = self.edge_index(n, v[(i + 1) % 3], v[i]) {
                                self.con[n as usize][j] = true;
                            }
                        }
                        return Ok(());
                    }
                }
            }
            // A vertex exactly on the segment splits it.
            for t in self.fan(a) {
                let v = self.tri[t as usize];
                for &c in &v {
                    if c == a || c == b {
                        continue;
                    }
                    let pc = self.pts[c as usize];
                    let o = orient(pa, pb, pc);
                    if o.abs() <= self.eps {
                        let dot =
                            (pc[0] - pa[0]) * (pb[0] - pa[0]) + (pc[1] - pa[1]) * (pb[1] - pa[1]);
                        let len2 = (pb[0] - pa[0]).powi(2) + (pb[1] - pa[1]).powi(2);
                        if dot > 0.0 && dot < len2 {
                            self.constrain(a, c, depth + 1)?;
                            return self.constrain(c, b, depth + 1);
                        }
                    }
                }
            }
            // The triangle around a whose opposite edge the segment
            // crosses; flip the first crossed edge that can be flipped
            // (walking along the segment).
            let mut crossed: Option<(u32, usize)> = None;
            for t in self.fan(a) {
                let v = self.tri[t as usize];
                let k = (0..3).find(|&k| v[k] == a).ok_or(CdtError)?;
                let (x, y) = (v[(k + 1) % 3], v[(k + 2) % 3]);
                let (px, py) = (self.pts[x as usize], self.pts[y as usize]);
                if orient(pa, pb, px) < -self.eps && orient(pa, pb, py) > self.eps
                    || orient(pa, pb, px) > self.eps && orient(pa, pb, py) < -self.eps
                {
                    // The segment leaves t through edge (x, y) if it
                    // points into t's angle at a.
                    if orient(pa, px, pb) >= -self.eps && orient(pa, pb, py) >= -self.eps {
                        crossed = Some((t, (k + 1) % 3));
                        break;
                    }
                }
            }
            let (mut t, mut i) = crossed.ok_or(CdtError)?;
            // Walk along the segment flipping where convex.
            let mut progressed = false;
            for _ in 0..self.tri.len() + 8 {
                if self.con[t as usize][i] {
                    // Crossing another constraint: intersecting rings.
                    return Err(CdtError);
                }
                let u = self.adj[t as usize][i];
                if u == NONE {
                    return Err(CdtError);
                }
                let v = self.tri[t as usize];
                let (x, y) = (v[i], v[(i + 1) % 3]);
                let j = self.edge_index(u, y, x).ok_or(CdtError)?;
                let d = self.tri[u as usize][(j + 2) % 3];
                let c = v[(i + 2) % 3];
                let (px, py, pc, pd) = (
                    self.pts[x as usize],
                    self.pts[y as usize],
                    self.pts[c as usize],
                    self.pts[d as usize],
                );
                // Convex quad (x, d, y, c)? Then flip edge x–y.
                let convex = orient(pc, pd, px) * orient(pc, pd, py) < 0.0
                    && orient(px, py, pc) * orient(px, py, pd) < 0.0;
                if convex {
                    self.flip(t, i)?;
                    progressed = true;
                    break;
                }
                if d == b {
                    break;
                }
                // Continue to the next crossed edge of u.
                let w = self.tri[u as usize];
                let pdd = self.pts[d as usize];
                let od = orient(pa, pb, pdd);
                if od.abs() <= self.eps {
                    // The segment passes through d: split there.
                    self.constrain(a, d, depth + 1)?;
                    return self.constrain(d, b, depth + 1);
                }
                // Next edge: (x, d) or (d, y) of u, whichever the segment
                // crosses.
                let nx = if (orient(pa, pb, px) > 0.0) != (od > 0.0) {
                    (x, d)
                } else {
                    (d, y)
                };
                let k = (0..3)
                    .find(|&k| w[k] == nx.0 && w[(k + 1) % 3] == nx.1)
                    .ok_or(CdtError)?;
                t = u;
                i = k;
            }
            if !progressed {
                // No convex quad along the whole chain this pass: the
                // configuration is degenerate (or the rings intersect).
                return Err(CdtError);
            }
        }
    }
}

/// Triangulate `rings` (`rings[0]` the outer boundary, the rest holes;
/// any orientation) over the points `pts`. Rings are index lists into
/// `pts`. Returns counter-clockwise triangles over `pts` indices.
pub(super) fn triangulate(pts: &[[f64; 2]], rings: &[Vec<u32>]) -> Result<Vec<[u32; 3]>, CdtError> {
    let n = pts.len();
    if n < 3 || n > (u32::MAX / 4) as usize {
        return Err(CdtError);
    }
    if pts.iter().any(|p| !p[0].is_finite() || !p[1].is_finite()) {
        return Err(CdtError);
    }
    // Normalise into the unit square.
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in pts {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let size = (hi[0] - lo[0]).max(hi[1] - lo[1]);
    if size.is_nan() || size <= 0.0 {
        return Err(CdtError);
    }
    let norm: Vec<[f64; 2]> = pts
        .iter()
        .map(|p| [(p[0] - lo[0]) / size, (p[1] - lo[1]) / size])
        .collect();
    // Merge duplicates (exact after normalisation, or within 1e-13).
    let mut canon: Vec<u32> = (0..n as u32).collect();
    {
        let q = |v: f64| (v * 1e12).round() as i64;
        let mut seen: HashMap<(i64, i64), u32> = HashMap::with_capacity(n);
        for (i, p) in norm.iter().enumerate() {
            let key = (q(p[0]), q(p[1]));
            match seen.get(&key) {
                Some(&j) => canon[i] = j,
                None => {
                    seen.insert(key, i as u32);
                }
            }
        }
    }
    let mut m = Mesh {
        pts: norm.clone(),
        tri: Vec::with_capacity(2 * n + 8),
        adj: Vec::with_capacity(2 * n + 8),
        con: Vec::with_capacity(2 * n + 8),
        vtri: vec![NONE; n + 3],
        last: 0,
        eps: 1e-14,
    };
    // Super-triangle vertices n, n+1, n+2.
    m.pts.push([-20.0, -20.0]);
    m.pts.push([40.0, -20.0]);
    m.pts.push([-20.0, 40.0]);
    let (s0, s1, s2) = (n as u32, n as u32 + 1, n as u32 + 2);
    m.push([s0, s1, s2]);
    // Morton insertion order.
    let mut order: Vec<u32> = (0..n as u32).filter(|&i| canon[i as usize] == i).collect();
    order.sort_by_key(|&i| {
        let p = norm[i as usize];
        morton(
            (p[0].clamp(0.0, 1.0) * 65535.0) as u32,
            (p[1].clamp(0.0, 1.0) * 65535.0) as u32,
        )
    });
    for &i in &order {
        m.insert(i)?;
    }
    // Constraints.
    for ring in rings {
        let r: Vec<u32> = ring
            .iter()
            .map(|&i| canon.get(i as usize).copied().unwrap_or(NONE))
            .collect();
        if r.contains(&NONE) {
            return Err(CdtError);
        }
        for k in 0..r.len() {
            let (a, b) = (r[k], r[(k + 1) % r.len()]);
            m.constrain(a, b, 0)?;
        }
    }
    // Flood fill: depth parity across constrained edges.
    let nt = m.tri.len();
    let mut depth: Vec<i32> = vec![-1; nt];
    let seed = (0..nt)
        .find(|&t| m.tri[t].iter().any(|&v| v >= n as u32))
        .ok_or(CdtError)?;
    depth[seed] = 0;
    let mut queue: std::collections::VecDeque<usize> = std::collections::VecDeque::new();
    queue.push_back(seed);
    // Breadth-first by depth: process same-depth regions before crossing.
    let mut next: Vec<usize> = Vec::new();
    loop {
        while let Some(t) = queue.pop_front() {
            for i in 0..3 {
                let u = m.adj[t][i];
                if u == NONE || depth[u as usize] >= 0 {
                    continue;
                }
                if m.con[t][i] {
                    next.push(u as usize);
                } else {
                    depth[u as usize] = depth[t];
                    queue.push_back(u as usize);
                }
            }
        }
        if next.is_empty() {
            break;
        }
        for t in next.drain(..) {
            if depth[t] < 0 {
                // Its constrained neighbour that queued it had depth d;
                // find the minimal such.
                let d = (0..3)
                    .filter_map(|i| {
                        let u = m.adj[t][i];
                        (u != NONE && m.con[t][i] && depth[u as usize] >= 0)
                            .then(|| depth[u as usize])
                    })
                    .min()
                    .unwrap_or(0);
                depth[t] = d + 1;
                queue.push_back(t);
            }
        }
    }
    let mut out = Vec::with_capacity(nt);
    for (&d, &v) in depth.iter().zip(&m.tri) {
        if d % 2 == 1 {
            if v.iter().any(|&x| x >= n as u32) {
                return Err(CdtError);
            }
            out.push(v);
        }
    }
    if out.is_empty() {
        return Err(CdtError);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn area(pts: &[[f64; 2]], tris: &[[u32; 3]]) -> f64 {
        tris.iter()
            .map(|t| 0.5 * orient(pts[t[0] as usize], pts[t[1] as usize], pts[t[2] as usize]))
            .sum()
    }

    #[test]
    fn square_with_hole() {
        let pts = vec![
            [0.0, 0.0],
            [4.0, 0.0],
            [4.0, 4.0],
            [0.0, 4.0],
            [1.0, 1.0],
            [3.0, 1.0],
            [3.0, 3.0],
            [1.0, 3.0],
        ];
        let t = triangulate(&pts, &[vec![0, 1, 2, 3], vec![4, 5, 6, 7]]).unwrap();
        assert_eq!(t.len(), 8);
        assert!((area(&pts, &t) - 12.0).abs() < 1e-12);
        assert!(t.iter().all(|t| orient(
            pts[t[0] as usize],
            pts[t[1] as usize],
            pts[t[2] as usize]
        ) > 0.0));
    }

    #[test]
    fn concave_with_collinear_points_and_many_holes() {
        // A comb with collinear runs along its spine.
        let mut pts: Vec<[f64; 2]> = Vec::new();
        let mut outer = Vec::new();
        for i in 0..=20 {
            pts.push([i as f64, 0.0]);
            outer.push(pts.len() as u32 - 1);
        }
        for i in (0..=20).rev() {
            let y = if i % 2 == 0 { 10.0 } else { 5.0 };
            pts.push([i as f64, y]);
            outer.push(pts.len() as u32 - 1);
        }
        let mut rings = vec![outer];
        let mut hole_area = 0.0;
        for k in 0..9 {
            let x = 1.0 + 2.0 * k as f64;
            let base = pts.len() as u32;
            pts.extend([
                [x + 0.2, 1.0],
                [x + 0.8, 1.0],
                [x + 0.8, 2.0],
                [x + 0.2, 2.0],
            ]);
            rings.push(vec![base, base + 3, base + 2, base + 1]); // clockwise
            hole_area += 0.6;
        }
        let t = triangulate(&pts, &rings).unwrap();
        // Exact outer area by the shoelace formula.
        let o: Vec<[f64; 2]> = rings[0].iter().map(|&i| pts[i as usize]).collect();
        let mut s = 0.0;
        for i in 0..o.len() {
            let (a, b) = (o[i], o[(i + 1) % o.len()]);
            s += a[0] * b[1] - b[0] * a[1];
        }
        let want = s.abs() * 0.5 - hole_area;
        assert!(
            (area(&pts, &t) - want).abs() < 1e-9,
            "{} vs {want}",
            area(&pts, &t)
        );
    }

    #[test]
    fn large_circle_with_hole_is_fast() {
        let n = 20_000;
        let mut pts: Vec<[f64; 2]> = (0..n)
            .map(|i| {
                let a = 2.0 * core::f64::consts::PI * i as f64 / n as f64;
                [a.cos(), a.sin()]
            })
            .collect();
        let base = pts.len() as u32;
        pts.extend((0..n).map(|i| {
            let a = 2.0 * core::f64::consts::PI * i as f64 / n as f64;
            [0.5 * a.cos(), 0.5 * a.sin()]
        }));
        let outer: Vec<u32> = (0..n as u32).collect();
        let hole: Vec<u32> = (base..base + n as u32).collect();
        let t = triangulate(&pts, &[outer, hole]).unwrap();
        assert_eq!(t.len(), 2 * n);
        let want = 0.75 * n as f64 * 0.5 * (2.0 * core::f64::consts::PI / n as f64).sin();
        assert!((area(&pts, &t) - want).abs() < 1e-9);
    }

    /// Random star-shaped outer rings with random non-overlapping
    /// square holes: the triangulation always covers exactly the
    /// region (area), counter-clockwise, over input indices only.
    #[test]
    fn random_star_polygons_with_holes() {
        let mut seed: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut rnd = || {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((seed >> 11) as f64) / ((1u64 << 53) as f64)
        };
        for case in 0..200 {
            let n = 3 + (rnd() * 60.0) as usize;
            let mut pts: Vec<[f64; 2]> = Vec::new();
            for i in 0..n {
                let a = 2.0 * core::f64::consts::PI * (i as f64 + 0.5 * rnd()) / n as f64;
                let r = 5.0 + 5.0 * rnd();
                pts.push([r * a.cos(), r * a.sin()]);
            }
            let mut rings = vec![(0..n as u32).collect::<Vec<u32>>()];
            let mut want = 0.0;
            for i in 0..n {
                let (a, b) = (pts[i], pts[(i + 1) % n]);
                want += 0.5 * (a[0] * b[1] - b[0] * a[1]);
            }
            // Holes on a grid inside radius 4.2; with n ≥ 12 every chord
            // of the outer ring stays beyond radius 4.6, so they are
            // inside.
            let holes = if n >= 12 { (rnd() * 6.0) as usize } else { 0 };
            for h in 0..holes {
                let (cx, cy) = (-2.5 + (h % 3) as f64 * 2.5, if h < 3 { -1.5 } else { 1.5 });
                let s = 0.3 + 0.6 * rnd();
                let base = pts.len() as u32;
                pts.extend([
                    [cx - s, cy - s],
                    [cx - s, cy + s],
                    [cx + s, cy + s],
                    [cx + s, cy - s],
                ]);
                rings.push((base..base + 4).collect());
                want -= 4.0 * s * s;
            }
            let t = triangulate(&pts, &rings).unwrap_or_else(|e| panic!("case {case}: {e:?}"));
            let got = area(&pts, &t);
            assert!(
                (got - want).abs() < 1e-9 * want.abs().max(1.0),
                "case {case}: {got} vs {want}"
            );
            for tri in &t {
                assert!(tri.iter().all(|&v| (v as usize) < pts.len()));
                let o = orient(
                    pts[tri[0] as usize],
                    pts[tri[1] as usize],
                    pts[tri[2] as usize],
                );
                assert!(o > 0.0, "case {case}: clockwise or flat triangle");
            }
        }
    }

    #[test]
    fn self_intersecting_ring_is_an_error() {
        let pts = vec![[0.0, 0.0], [1.0, 1.0], [1.0, 0.0], [0.0, 1.0]];
        assert!(triangulate(&pts, &[vec![0, 1, 2, 3]]).is_err());
    }

    #[test]
    fn duplicate_points_merge() {
        let pts = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [1.0, 1.0], [0.0, 1.0]];
        let t = triangulate(&pts, &[vec![0, 1, 2, 3, 4]]).unwrap();
        assert!((area(&pts, &t) - 1.0).abs() < 1e-12);
    }
}
