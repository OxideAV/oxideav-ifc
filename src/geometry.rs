//! Phase 3: tessellated-geometry extraction.
//!
//! Phase 1 ([`crate::parser`]) gives a positional instance graph and
//! Phase 2 ([`crate::schema`]) names the attributes of the spatial /
//! product slice. This module turns the *geometric representation
//! items* a product points at into plain triangle meshes — the first
//! Phase-3 slice covers the two tessellation entities:
//!
//! * **`IfcTriangulatedFaceSet`** (`Coordinates`, `Normals`, `Closed`,
//!   `CoordIndex`, `PnIndex`) — a list of triangles, each a triple of
//!   one-based indices into a shared point list (ISO 16739 §8.8.3.47 /
//!   IFC4 EXPRESS `IfcTriangulatedFaceSet`).
//! * **`IfcPolygonalFaceSet`** (`Coordinates`, `Closed`, `Faces`,
//!   `PnIndex`) — a list of `IfcIndexedPolygonalFace` records, each a
//!   convex polygon of one-based indices; this module fan-triangulates
//!   every face (§8.8.3.39 / IFC4 EXPRESS `IfcPolygonalFaceSet`).
//!
//! Both face-set kinds carry their vertices in a shared
//! `IfcCartesianPointList3D` (`CoordList : LIST OF LIST [3:3] OF
//! IfcLengthMeasure`) reached through the supertype attribute
//! `IfcTessellatedFaceSet.Coordinates`. The optional `PnIndex` adds one
//! more one-based indirection: a `CoordIndex` value *i* selects
//! `PnIndex[i]`, which is then the one-based row of the point list
//! (§8.8.3.47, "Use of PnIndex").
//!
//! The output [`TriMesh`] is std-only (no framework dependency), so it
//! is available in `--no-default-features` builds. The `registry`
//! decoder lifts it into an `oxideav_mesh3d::Scene3D`.
//!
//! Alongside the two tessellation entities this module also evaluates
//! the **explicit faceted boundary-representation** family:
//!
//! * **`IfcFacetedBrep`** (`Outer : IfcClosedShell`) and
//!   **`IfcFacetedBrepWithVoids`** (`Outer`, `Voids : SET OF
//!   IfcClosedShell`) — a manifold solid whose faces are all planar
//!   bounded polygons (ISO 16739 §8.8.3.18 / IFC4 EXPRESS
//!   `IfcManifoldSolidBrep`).
//! * **`IfcFaceBasedSurfaceModel`** (`FbsmFaces : SET OF
//!   IfcConnectedFaceSet`) and **`IfcShellBasedSurfaceModel`**
//!   (`SbsmBoundary : SET OF IfcShell`) — open/closed shells of the same
//!   faces, used for surface (non-solid) bodies.
//!
//! Each shell is an `IfcConnectedFaceSet` (`CfsFaces : SET OF IfcFace`);
//! each `IfcFace` carries `Bounds : SET OF IfcFaceBound`, whose
//! `IfcFaceOuterBound` (or any bound when none is flagged outer) wraps an
//! `IfcPolyLoop(Polygon : LIST [3:?] OF IfcCartesianPoint)`. The outer
//! loop is fan-triangulated; the `IfcFaceBound.Orientation` flag reverses
//! the loop winding when `.F.`. Points are referenced directly (not
//! through a `IfcCartesianPointList`), so the extractor builds the mesh
//! vertex table on the fly, sharing a vertex across loops by its
//! `IfcCartesianPoint` `#id`.
//!
//! Inner face bounds (holes, and the `Voids` of a
//! `…BrepWithVoids`) are **not** subtracted in this slice — the outer
//! loop of every face is emitted. Swept solids
//! (`IfcExtrudedAreaSolid`), advanced Breps, boolean results, and mapped
//! items remain later Phase-3 work and are reported as
//! [`GeometryError::Unsupported`] rather than silently dropped.

use crate::parser::StepFile;
use crate::value::Value;

/// A flat, indexed triangle mesh in the local coordinate space of the
/// representation item it was extracted from.
///
/// Coordinates are `f64` (the wire carries `IfcLengthMeasure` reals);
/// the registry decoder narrows to `f32` when building a `Scene3D`.
/// `triangles` are **zero-based** triples into `positions` — the
/// one-based STEP indices have already been resolved (and `PnIndex`
/// indirection applied) during extraction.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TriMesh {
    /// Vertex positions, one `[x, y, z]` per point.
    pub positions: Vec<[f64; 3]>,
    /// Triangle connectivity: zero-based indices into `positions`.
    pub triangles: Vec<[u32; 3]>,
}

impl TriMesh {
    /// Number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.positions.len()
    }

    /// `true` when the mesh carries no triangles.
    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }
}

/// Why tessellation extraction could not produce a mesh.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GeometryError {
    /// The instance id does not exist in the file.
    MissingInstance(u64),
    /// A representation item is a geometry style this slice does not yet
    /// evaluate (swept solid, Brep, boolean, mapped item, …). Carries
    /// the offending entity keyword.
    Unsupported(String),
    /// A face-set's `Coordinates` reference is missing, not an
    /// `IfcCartesianPointList3D`, or otherwise malformed.
    BadCoordinates,
    /// A one-based index (in `CoordIndex` / a face / `PnIndex`) is zero
    /// or points past the end of the list it indexes.
    IndexOutOfRange,
    /// A coordinate row did not have three numeric components.
    BadCoordinate,
}

impl core::fmt::Display for GeometryError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::MissingInstance(id) => write!(f, "no instance #{id}"),
            Self::Unsupported(kw) => {
                write!(f, "unsupported geometry representation item `{kw}`")
            }
            Self::BadCoordinates => {
                f.write_str("face-set Coordinates is not a valid IfcCartesianPointList3D")
            }
            Self::IndexOutOfRange => f.write_str("tessellation index out of range"),
            Self::BadCoordinate => f.write_str("coordinate row is not three reals"),
        }
    }
}

impl std::error::Error for GeometryError {}

/// Resolve and tessellate one geometric-representation item by id.
///
/// Dispatches on the entity keyword: the two tessellation face sets
/// (`IFCTRIANGULATEDFACESET`, `IFCPOLYGONALFACESET`) and the explicit
/// faceted-Brep family (`IFCFACETEDBREP`, `IFCFACETEDBREPWITHVOIDS`,
/// `IFCFACEBASEDSURFACEMODEL`, `IFCSHELLBASEDSURFACEMODEL`) produce a
/// [`TriMesh`]; any other keyword is a [`GeometryError::Unsupported`].
/// This is the lowest-level entry — most callers want
/// [`mesh_from_shape_representation`] or the `Model`-level walk.
pub fn tessellate_item(step: &StepFile, id: u64) -> Result<TriMesh, GeometryError> {
    let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
    match inst.keyword.as_str() {
        "IFCTRIANGULATEDFACESET" => triangulated_face_set(step, &inst.args),
        "IFCPOLYGONALFACESET" => polygonal_face_set(step, &inst.args),
        // Explicit faceted boundary representation: a solid (one outer
        // closed shell, plus ignored void shells) or a surface model
        // (a set of connected face sets / shells). All reduce to a set
        // of `IfcFace`s walked by `faceted_brep`.
        "IFCFACETEDBREP" | "IFCFACETEDBREPWITHVOIDS" => {
            // IfcManifoldSolidBrep.Outer is the first attribute; the
            // optional `Voids` of the …WithVoids subtype follow but are
            // not subtracted in this slice.
            let outer = inst
                .args
                .first()
                .and_then(Value::as_reference)
                .ok_or(GeometryError::BadCoordinates)?;
            faceted_brep(step, &[outer])
        }
        "IFCFACEBASEDSURFACEMODEL" => {
            // FbsmFaces : SET OF IfcConnectedFaceSet (first attribute).
            let shells = inst
                .args
                .first()
                .and_then(Value::as_list)
                .ok_or(GeometryError::BadCoordinates)?;
            faceted_brep(step, &refs(shells)?)
        }
        "IFCSHELLBASEDSURFACEMODEL" => {
            // SbsmBoundary : SET OF IfcShell (IfcOpenShell / IfcClosedShell)
            // — both are IfcConnectedFaceSet (first attribute).
            let shells = inst
                .args
                .first()
                .and_then(Value::as_list)
                .ok_or(GeometryError::BadCoordinates)?;
            faceted_brep(step, &refs(shells)?)
        }
        other => Err(GeometryError::Unsupported(other.to_string())),
    }
}

/// Collect a list of `#id` references, erroring on any non-reference
/// element (an aggregate of shells / faces is always a reference list).
fn refs(list: &[Value]) -> Result<Vec<u64>, GeometryError> {
    list.iter()
        .map(|v| v.as_reference().ok_or(GeometryError::BadCoordinates))
        .collect()
}

/// Tessellate every supported item of an `IfcShapeRepresentation`,
/// merging the per-item meshes into one [`TriMesh`].
///
/// `IfcShapeRepresentation` serialises as
/// `(ContextOfItems, RepresentationIdentifier, RepresentationType,
/// Items)`; `Items` (argument index 3) is the list of
/// representation-item references. Items whose keyword is not a
/// supported tessellation are skipped (a representation commonly mixes
/// an axis/box item with the body mesh); if **no** item yielded
/// geometry the first unsupported keyword is surfaced as the error so
/// the caller can tell "empty" from "all-unsupported".
pub fn mesh_from_shape_representation(
    step: &StepFile,
    shape_rep_id: u64,
) -> Result<TriMesh, GeometryError> {
    let inst = step
        .get(shape_rep_id)
        .ok_or(GeometryError::MissingInstance(shape_rep_id))?;
    // Items is the 4th positional attribute (index 3).
    let items = inst
        .args
        .get(3)
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinates)?;

    let mut merged = TriMesh::default();
    let mut first_unsupported: Option<GeometryError> = None;
    let mut produced = false;

    for item in items {
        let Some(item_id) = item.as_reference() else {
            continue;
        };
        match tessellate_item(step, item_id) {
            Ok(mesh) => {
                append_mesh(&mut merged, mesh);
                produced = true;
            }
            Err(e @ GeometryError::Unsupported(_)) => {
                if first_unsupported.is_none() {
                    first_unsupported = Some(e);
                }
            }
            // A malformed *supported* item is a hard error.
            Err(other) => return Err(other),
        }
    }

    if produced {
        Ok(merged)
    } else {
        Err(first_unsupported.unwrap_or(GeometryError::BadCoordinates))
    }
}

/// Tessellate every shape representation reachable from a product's
/// `Representation`, merging the result into one [`TriMesh`].
///
/// The walk is `IfcProduct.Representation` →
/// `IfcProductDefinitionShape.Representations` (a list of
/// `IfcShapeRepresentation`) → each representation's `Items`. Shape
/// representations that contain only unsupported items contribute
/// nothing (an axis / box / swept-solid representation alongside the
/// tessellated body is the common case); the merged mesh is returned as
/// long as **some** representation produced geometry. If none did, the
/// first unsupported keyword encountered is surfaced.
///
/// `product_def_shape_id` is the `#id` of the `IfcProductDefinitionShape`
/// (i.e. the value of a product's `Representation` attribute — see
/// [`crate::TypedEntity::representation`]).
pub fn mesh_from_product_shape(
    step: &StepFile,
    product_def_shape_id: u64,
) -> Result<TriMesh, GeometryError> {
    let inst = step
        .get(product_def_shape_id)
        .ok_or(GeometryError::MissingInstance(product_def_shape_id))?;
    // IfcProductDefinitionShape: (Name, Description, Representations).
    // Representations is the 3rd attribute (index 2): a list of
    // IfcRepresentation (here IfcShapeRepresentation) references.
    let reps = inst
        .args
        .get(2)
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinates)?;

    let mut merged = TriMesh::default();
    let mut first_unsupported: Option<GeometryError> = None;
    let mut produced = false;

    for rep in reps {
        let Some(rep_id) = rep.as_reference() else {
            continue;
        };
        match mesh_from_shape_representation(step, rep_id) {
            Ok(mesh) => {
                append_mesh(&mut merged, mesh);
                produced = true;
            }
            Err(e @ GeometryError::Unsupported(_)) => {
                if first_unsupported.is_none() {
                    first_unsupported = Some(e);
                }
            }
            // A representation that exists but has no usable Items list
            // (BadCoordinates) is tolerated — it may be a non-body
            // representation (axis, footprint) we don't model.
            Err(GeometryError::BadCoordinates) => {}
            Err(other) => return Err(other),
        }
    }

    if produced {
        Ok(merged)
    } else {
        Err(first_unsupported.unwrap_or(GeometryError::BadCoordinates))
    }
}

impl TriMesh {
    /// Return a copy of this mesh with every vertex mapped through
    /// `xform` (triangle connectivity is unchanged).
    pub fn transformed(&self, xform: &Transform) -> TriMesh {
        TriMesh {
            positions: self.positions.iter().map(|p| xform.apply(*p)).collect(),
            triangles: self.triangles.clone(),
        }
    }

    /// Map every vertex through `xform` in place.
    pub fn transform(&mut self, xform: &Transform) {
        for p in &mut self.positions {
            *p = xform.apply(*p);
        }
    }
}

// =====================================================================
// IfcLocalPlacement world-positioning
//
// A product's `ObjectPlacement` is (for the common case) an
// `IfcLocalPlacement(PlacementRelTo, RelativePlacement)` where
// `RelativePlacement` is an `IfcAxis2Placement3D(Location, Axis,
// RefDirection)`. Each placement defines an affine map from the
// product's local space into the coordinate space of its parent
// placement (`PlacementRelTo`); chaining the maps from a leaf up to the
// root (where `PlacementRelTo` is absent) gives the world transform.
//
// The rotation columns are the three orthonormal axes derived by the
// EXPRESS `IfcBuildAxes(Axis, RefDirection)` function (IFC4 EXPRESS,
// `IfcGeometricConstraintResource` / `IfcGeometryResource`):
//   Z = normalise(Axis)                     (default [0,0,1])
//   X = first-proj-axis(Z, RefDirection)    (RefDirection made ⟂ to Z)
//   Y = normalise(Z × X)
// and the translation is the placement `Location`.
// =====================================================================

/// A 3-D affine transform: a 3×3 linear part (column-major: `cols[0]`
/// is the image of the local X axis, etc.) plus a translation.
///
/// Built from an `IfcAxis2Placement3D`, where the columns are the
/// orthonormal placement axes (`P[1]`, `P[2]`, `P[3]`) derived per the
/// EXPRESS `IfcBuildAxes` function and the translation is the placement
/// `Location`. Composition follows the `IfcLocalPlacement.PlacementRelTo`
/// chain.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    /// Column-major 3×3 linear part: `cols[i]` is the world-space image
    /// of local basis vector *i* (X, Y, Z).
    pub cols: [[f64; 3]; 3],
    /// Translation (the placement origin in parent space).
    pub translation: [f64; 3],
}

impl Default for Transform {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Transform {
    /// The identity transform (no rotation, no translation).
    pub const IDENTITY: Transform = Transform {
        cols: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        translation: [0.0, 0.0, 0.0],
    };

    /// Map a local point into this transform's parent space:
    /// `R · p + t`.
    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        let [cx, cy, cz] = self.cols;
        [
            cx[0] * p[0] + cy[0] * p[1] + cz[0] * p[2] + self.translation[0],
            cx[1] * p[0] + cy[1] * p[1] + cz[1] * p[2] + self.translation[1],
            cx[2] * p[0] + cy[2] * p[1] + cz[2] * p[2] + self.translation[2],
        ]
    }

    /// Compose two transforms: `self ∘ other` applies `other` first,
    /// then `self` (i.e. `self.compose(other).apply(p) ==
    /// self.apply(other.apply(p))`). Used to fold a child placement into
    /// its parent's coordinate space.
    pub fn compose(&self, other: &Transform) -> Transform {
        let mut cols = [[0.0; 3]; 3];
        for (j, col) in other.cols.iter().enumerate() {
            // The image of `other`'s j-th column under `self`'s linear
            // part (no translation — these are direction vectors).
            let [sx, sy, sz] = self.cols;
            cols[j] = [
                sx[0] * col[0] + sy[0] * col[1] + sz[0] * col[2],
                sx[1] * col[0] + sy[1] * col[1] + sz[1] * col[2],
                sx[2] * col[0] + sy[2] * col[1] + sz[2] * col[2],
            ];
        }
        Transform {
            cols,
            translation: self.apply(other.translation),
        }
    }
}

/// Resolve the **world** transform of an `IfcObjectPlacement` id by
/// folding its `IfcLocalPlacement.PlacementRelTo` chain from this
/// placement up to the (absolute) root.
///
/// `world = root ∘ … ∘ parent ∘ self`. A placement whose `PlacementRelTo`
/// is `$` is absolute (its transform is composed against the identity).
/// A self-referential or cyclic `PlacementRelTo` chain is broken at the
/// `max_depth` cap and the partial composition returned (the file is
/// malformed, but extraction stays bounded).
///
/// Only `IfcLocalPlacement` with an `IfcAxis2Placement3D` relative
/// placement is interpreted; any other placement kind (grid / linear /
/// 2-D) contributes identity, so its geometry stays in local space
/// rather than being dropped.
pub fn placement_transform(step: &StepFile, placement_id: u64) -> Result<Transform, GeometryError> {
    // Walk parent-ward collecting each link's local transform, then fold
    // root-first so the outermost placement is applied last.
    const MAX_DEPTH: usize = 4096;
    let mut chain: Vec<Transform> = Vec::new();
    let mut current = Some(placement_id);
    let mut depth = 0usize;
    while let Some(id) = current {
        if depth >= MAX_DEPTH {
            break;
        }
        depth += 1;
        let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
        match inst.keyword.as_str() {
            "IFCLOCALPLACEMENT" => {
                // (PlacementRelTo, RelativePlacement)
                let rel_id = inst
                    .args
                    .get(1)
                    .and_then(Value::as_reference)
                    .ok_or(GeometryError::BadCoordinates)?;
                chain.push(axis2_placement_3d(step, rel_id)?);
                current = inst.args.first().and_then(Value::as_reference);
            }
            // Non-local placement kinds are not interpreted in this slice;
            // treat as identity (geometry stays in its local frame).
            _ => break,
        }
    }
    // Fold root-first: world = root ∘ … ∘ leaf.
    let mut world = Transform::IDENTITY;
    for link in chain.iter().rev() {
        world = world.compose(link);
    }
    Ok(world)
}

/// Build the affine transform of one `IfcAxis2Placement3D`
/// (`Location`, `Axis`, `RefDirection`) per the EXPRESS `IfcBuildAxes`
/// derivation. `$`/absent `Axis` and `RefDirection` default to the
/// world Z and X directions.
fn axis2_placement_3d(step: &StepFile, id: u64) -> Result<Transform, GeometryError> {
    let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
    if inst.keyword != "IFCAXIS2PLACEMENT3D" {
        // A 2-D or other placement is out of this slice's scope — keep
        // the geometry in local space rather than mis-transforming it.
        return Ok(Transform::IDENTITY);
    }
    let location = cartesian_point(step, inst.args.first())?;
    let axis = direction(step, inst.args.get(1))?;
    let ref_dir = direction(step, inst.args.get(2))?;
    let [x, y, z] = build_axes(axis, ref_dir);
    Ok(Transform {
        cols: [x, y, z],
        translation: location,
    })
}

/// Resolve an `IfcCartesianPoint` reference to a 3-D coordinate,
/// zero-padding a 2-D point's Z to 0.
fn cartesian_point(step: &StepFile, arg: Option<&Value>) -> Result<[f64; 3], GeometryError> {
    let id = arg
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
    if inst.keyword != "IFCCARTESIANPOINT" {
        return Err(GeometryError::BadCoordinates);
    }
    let comps = inst
        .args
        .first()
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinate)?;
    if comps.len() < 2 || comps.len() > 3 {
        return Err(GeometryError::BadCoordinate);
    }
    let mut out = [0.0f64; 3];
    for (i, c) in comps.iter().enumerate() {
        out[i] = c.as_number().ok_or(GeometryError::BadCoordinate)?;
    }
    Ok(out)
}

/// Resolve an optional `IfcDirection` reference to a 3-D ratio vector
/// (`$`/absent → `None`), zero-padding a 2-D direction's Z to 0.
fn direction(step: &StepFile, arg: Option<&Value>) -> Result<Option<[f64; 3]>, GeometryError> {
    let id = match arg {
        None | Some(Value::Unset) => return Ok(None),
        Some(v) => v.as_reference().ok_or(GeometryError::BadCoordinates)?,
    };
    let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
    if inst.keyword != "IFCDIRECTION" {
        return Err(GeometryError::BadCoordinates);
    }
    let comps = inst
        .args
        .first()
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinate)?;
    if comps.len() < 2 || comps.len() > 3 {
        return Err(GeometryError::BadCoordinate);
    }
    let mut out = [0.0f64; 3];
    for (i, c) in comps.iter().enumerate() {
        out[i] = c.as_number().ok_or(GeometryError::BadCoordinate)?;
    }
    Ok(Some(out))
}

// --- EXPRESS placement-axis derivation (IfcBuildAxes) ---------------
//
// These mirror the EXPRESS functions IfcBuildAxes / IfcFirstProjAxis /
// IfcNormalise / IfcCrossProduct / IfcDotProduct, restricted to the 3-D
// real-vector case (no IfcVector magnitude wrapper — every input here is
// an IfcDirection).

/// `IfcBuildAxes(Axis, RefDirection)` → the orthonormal placement axes
/// `[X, Y, Z]` (returned as `[P1, P2, P3]`).
fn build_axes(axis: Option<[f64; 3]>, ref_dir: Option<[f64; 3]>) -> [[f64; 3]; 3] {
    // D1 = NVL(normalise(Axis), [0,0,1]); the default Z direction.
    let d1 = axis.and_then(normalise).unwrap_or([0.0, 0.0, 1.0]);
    let d2 = first_proj_axis(d1, ref_dir);
    // P2 = normalise(D1 × D2); D1 and D2 are already orthonormal so the
    // cross product has unit magnitude, but normalise for numerical
    // hygiene. P3 = D1 (the Z axis).
    let p2 = normalise(cross(d1, d2)).unwrap_or_else(|| cross(d1, d2));
    [d2, p2, d1]
}

/// `IfcFirstProjAxis(ZAxis, Arg)` → the placement X axis: `Arg` (or a
/// default) projected into the plane ⟂ `ZAxis`, then normalised.
fn first_proj_axis(z_axis: [f64; 3], arg: Option<[f64; 3]>) -> [f64; 3] {
    let z = normalise(z_axis).unwrap_or([0.0, 0.0, 1.0]);
    // Choose V: the given RefDirection (normalised), or a world axis not
    // parallel to Z when RefDirection is absent.
    let v = match arg.and_then(normalise) {
        Some(v) => v,
        None => {
            if z != [1.0, 0.0, 0.0] {
                [1.0, 0.0, 0.0]
            } else {
                [0.0, 1.0, 0.0]
            }
        }
    };
    // XVec = (V · Z) · Z ; XAxis = normalise(V − XVec).
    let vz = dot(v, z);
    let x_vec = [vz * z[0], vz * z[1], vz * z[2]];
    let diff = [v[0] - x_vec[0], v[1] - x_vec[1], v[2] - x_vec[2]];
    // If V is parallel to Z the difference is ~0; fall back to a world
    // axis ⟂ to Z so the basis stays well-formed.
    normalise(diff).unwrap_or_else(|| {
        let fallback = if z != [1.0, 0.0, 0.0] {
            [1.0, 0.0, 0.0]
        } else {
            [0.0, 1.0, 0.0]
        };
        let d = dot(fallback, z);
        let proj = [
            fallback[0] - d * z[0],
            fallback[1] - d * z[1],
            fallback[2] - d * z[2],
        ];
        normalise(proj).unwrap_or([1.0, 0.0, 0.0])
    })
}

/// `IfcNormalise` for a 3-D direction: unit vector, or `None` for a
/// zero-magnitude input.
fn normalise(v: [f64; 3]) -> Option<[f64; 3]> {
    let mag = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if mag > 0.0 {
        Some([v[0] / mag, v[1] / mag, v[2] / mag])
    } else {
        None
    }
}

/// `IfcDotProduct` of two 3-D directions (both normalised first, per the
/// EXPRESS definition).
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    let a = normalise(a).unwrap_or(a);
    let b = normalise(b).unwrap_or(b);
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// `IfcCrossProduct` orientation of two 3-D directions (both normalised
/// first, per the EXPRESS definition; the magnitude wrapper is dropped).
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    let a = normalise(a).unwrap_or(a);
    let b = normalise(b).unwrap_or(b);
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Append `src` onto `dst`, offsetting `src`'s triangle indices by the
/// vertex count already in `dst`.
fn append_mesh(dst: &mut TriMesh, src: TriMesh) {
    let base = dst.positions.len() as u32;
    dst.positions.extend(src.positions);
    dst.triangles.extend(
        src.triangles
            .into_iter()
            .map(|[a, b, c]| [a + base, b + base, c + base]),
    );
}

// ---------------------------------------------------------------------
// IfcTriangulatedFaceSet
//   args: Coordinates, Normals, Closed, CoordIndex, PnIndex
// ---------------------------------------------------------------------
fn triangulated_face_set(step: &StepFile, args: &[Value]) -> Result<TriMesh, GeometryError> {
    let positions = coordinates(step, args.first())?;
    let pn = pn_index(args.get(4))?;
    // CoordIndex: LIST OF LIST [3:3] OF positive-integer (index 3).
    let coord_index = args
        .get(3)
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinates)?;

    let n = positions.len();
    let mut triangles = Vec::with_capacity(coord_index.len());
    for tri in coord_index {
        let row = tri.as_list().ok_or(GeometryError::IndexOutOfRange)?;
        if row.len() != 3 {
            return Err(GeometryError::IndexOutOfRange);
        }
        let a = resolve_vertex(&row[0], &pn, n)?;
        let b = resolve_vertex(&row[1], &pn, n)?;
        let c = resolve_vertex(&row[2], &pn, n)?;
        triangles.push([a, b, c]);
    }
    Ok(TriMesh {
        positions,
        triangles,
    })
}

// ---------------------------------------------------------------------
// IfcPolygonalFaceSet
//   args: Coordinates, Closed, Faces, PnIndex
//   Faces: LIST OF IfcIndexedPolygonalFace (CoordIndex : LIST [3:?])
// ---------------------------------------------------------------------
fn polygonal_face_set(step: &StepFile, args: &[Value]) -> Result<TriMesh, GeometryError> {
    let positions = coordinates(step, args.first())?;
    let pn = pn_index(args.get(3))?;
    // Faces is the 3rd attribute (index 2).
    let faces = args
        .get(2)
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinates)?;

    let n = positions.len();
    let mut triangles = Vec::new();
    for face_ref in faces {
        let face_id = face_ref
            .as_reference()
            .ok_or(GeometryError::BadCoordinates)?;
        let face = step
            .get(face_id)
            .ok_or(GeometryError::MissingInstance(face_id))?;
        // IfcIndexedPolygonalFace (and …WithVoids): CoordIndex is the
        // first attribute (index 0). Inner void loops (the second
        // attribute of the …WithVoids subtype) are not subtracted in
        // this slice — the outer loop is fan-triangulated.
        let loop_indices = face
            .args
            .first()
            .and_then(Value::as_list)
            .ok_or(GeometryError::BadCoordinates)?;
        if loop_indices.len() < 3 {
            return Err(GeometryError::IndexOutOfRange);
        }
        // Fan-triangulate the (assumed convex / planar) polygon:
        // (v0,v1,v2), (v0,v2,v3), …
        let v0 = resolve_vertex(&loop_indices[0], &pn, n)?;
        for w in loop_indices[1..].windows(2) {
            let v1 = resolve_vertex(&w[0], &pn, n)?;
            let v2 = resolve_vertex(&w[1], &pn, n)?;
            triangles.push([v0, v1, v2]);
        }
    }
    Ok(TriMesh {
        positions,
        triangles,
    })
}

// ---------------------------------------------------------------------
// IfcFacetedBrep / IfcFaceBasedSurfaceModel / IfcShellBasedSurfaceModel
//
// Explicit faceted boundary representation. Each input is one or more
// `IfcConnectedFaceSet` (closed/open shell) #ids; every shell's faces are
// `IfcFace(Bounds : SET OF IfcFaceBound)`. The bound to triangulate is
// the `IfcFaceOuterBound` if present, else the single bound; its
// `Bound : IfcLoop` is an `IfcPolyLoop(Polygon : LIST OF
// IfcCartesianPoint)`. The polygon is fan-triangulated; a `.F.`
// `Orientation` reverses the winding. Points are referenced directly, so
// vertices are interned by `IfcCartesianPoint` #id into a shared table.
// ---------------------------------------------------------------------

/// Build a [`TriMesh`] from a set of `IfcConnectedFaceSet` (shell) ids —
/// the shared core of `IfcFacetedBrep` (one outer shell) and the
/// face-/shell-based surface models (a set of shells).
fn faceted_brep(step: &StepFile, shell_ids: &[u64]) -> Result<TriMesh, GeometryError> {
    // Intern points by #id so a vertex shared across faces is emitted once.
    let mut interner = PointInterner::default();
    let mut triangles: Vec<[u32; 3]> = Vec::new();

    for &shell_id in shell_ids {
        let shell = step
            .get(shell_id)
            .ok_or(GeometryError::MissingInstance(shell_id))?;
        // IfcConnectedFaceSet.CfsFaces is the first attribute (closed and
        // open shells add no serialised attributes of their own).
        let faces = shell
            .args
            .first()
            .and_then(Value::as_list)
            .ok_or(GeometryError::BadCoordinates)?;
        for face_ref in faces {
            let face_id = face_ref
                .as_reference()
                .ok_or(GeometryError::BadCoordinates)?;
            face_triangles(step, face_id, &mut interner, &mut triangles)?;
        }
    }

    Ok(TriMesh {
        positions: interner.positions,
        triangles,
    })
}

/// Fan-triangulate the outer bound of one `IfcFace`, appending triangles
/// (with interned vertices) to `out`.
fn face_triangles(
    step: &StepFile,
    face_id: u64,
    interner: &mut PointInterner,
    out: &mut Vec<[u32; 3]>,
) -> Result<(), GeometryError> {
    let face = step
        .get(face_id)
        .ok_or(GeometryError::MissingInstance(face_id))?;
    if face.keyword != "IFCFACE" && face.keyword != "IFCFACESURFACE" {
        return Err(GeometryError::Unsupported(face.keyword.clone()));
    }
    // IfcFace.Bounds : SET OF IfcFaceBound (first attribute).
    let bounds = face
        .args
        .first()
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinates)?;

    // Prefer the IfcFaceOuterBound; fall back to the sole bound when no
    // bound is flagged outer (a single-loop face). Inner bounds (holes)
    // are not subtracted in this slice.
    let mut outer: Option<u64> = None;
    let mut only: Option<u64> = None;
    let mut count = 0usize;
    for b in bounds {
        let bid = b.as_reference().ok_or(GeometryError::BadCoordinates)?;
        count += 1;
        only = Some(bid);
        let binst = step.get(bid).ok_or(GeometryError::MissingInstance(bid))?;
        if binst.keyword == "IFCFACEOUTERBOUND" {
            outer = Some(bid);
        }
    }
    let bound_id = match (outer, count) {
        (Some(id), _) => id,
        (None, 1) => only.ok_or(GeometryError::BadCoordinates)?,
        // Multiple bounds, none flagged outer: ambiguous which is the
        // outer loop — emit nothing rather than guess.
        (None, _) => return Ok(()),
    };

    let bound = step
        .get(bound_id)
        .ok_or(GeometryError::MissingInstance(bound_id))?;
    // IfcFaceBound: (Bound : IfcLoop, Orientation : IfcBoolean).
    let loop_id = bound
        .args
        .first()
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    // Orientation `.F.` reverses the polygon sense; absent / `.T.` keeps it.
    let reversed = matches!(bound.args.get(1).and_then(Value::as_enum), Some("F"));

    let loop_inst = step
        .get(loop_id)
        .ok_or(GeometryError::MissingInstance(loop_id))?;
    if loop_inst.keyword != "IFCPOLYLOOP" {
        // Vertex/edge loops are not modelled in this slice.
        return Err(GeometryError::Unsupported(loop_inst.keyword.clone()));
    }
    // IfcPolyLoop.Polygon : LIST [3:?] OF IfcCartesianPoint (first attr).
    let polygon = loop_inst
        .args
        .first()
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinates)?;
    if polygon.len() < 3 {
        return Err(GeometryError::IndexOutOfRange);
    }

    // Intern each polygon vertex to a mesh index.
    let mut idx: Vec<u32> = Vec::with_capacity(polygon.len());
    for p in polygon {
        let pid = p.as_reference().ok_or(GeometryError::BadCoordinates)?;
        idx.push(interner.intern(step, pid)?);
    }
    if reversed {
        idx.reverse();
    }
    // Fan-triangulate the (planar, assumed convex) loop.
    let v0 = idx[0];
    for w in idx[1..].windows(2) {
        out.push([v0, w[0], w[1]]);
    }
    Ok(())
}

/// Builds the mesh vertex table for a faceted Brep, mapping each
/// `IfcCartesianPoint` `#id` to a single vertex index so a point shared
/// across faces (every Brep vertex is, by topology) is emitted once.
#[derive(Default)]
struct PointInterner {
    positions: Vec<[f64; 3]>,
    by_id: std::collections::HashMap<u64, u32>,
}

impl PointInterner {
    /// Return the vertex index for an `IfcCartesianPoint` `#id`, resolving
    /// and appending it the first time it is seen.
    fn intern(&mut self, step: &StepFile, point_id: u64) -> Result<u32, GeometryError> {
        if let Some(&i) = self.by_id.get(&point_id) {
            return Ok(i);
        }
        let coord = cartesian_point(step, Some(&Value::Reference(point_id)))?;
        let i = self.positions.len() as u32;
        self.positions.push(coord);
        self.by_id.insert(point_id, i);
        Ok(i)
    }
}

/// Resolve the `Coordinates` reference (the first face-set attribute) to
/// the `IfcCartesianPointList3D` point rows.
fn coordinates(
    step: &StepFile,
    coords_arg: Option<&Value>,
) -> Result<Vec<[f64; 3]>, GeometryError> {
    let id = coords_arg
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
    if inst.keyword != "IFCCARTESIANPOINTLIST3D" {
        return Err(GeometryError::BadCoordinates);
    }
    // CoordList: LIST OF LIST [3:3] OF IfcLengthMeasure (index 0).
    let rows = inst
        .args
        .first()
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinates)?;
    let mut out = Vec::with_capacity(rows.len());
    for row in rows {
        let comps = row.as_list().ok_or(GeometryError::BadCoordinate)?;
        if comps.len() != 3 {
            return Err(GeometryError::BadCoordinate);
        }
        let x = comps[0].as_number().ok_or(GeometryError::BadCoordinate)?;
        let y = comps[1].as_number().ok_or(GeometryError::BadCoordinate)?;
        let z = comps[2].as_number().ok_or(GeometryError::BadCoordinate)?;
        out.push([x, y, z]);
    }
    Ok(out)
}

/// Parse the optional `PnIndex` attribute into a flat one-based table,
/// or `None` when absent (`$`).
///
/// `PnIndex : LIST OF IfcPositiveInteger` — each entry is itself a
/// one-based row number into the point list; a `CoordIndex` value *i*
/// then selects `PnIndex[i]` (ISO 16739 §8.8.3.47).
fn pn_index(arg: Option<&Value>) -> Result<Option<Vec<usize>>, GeometryError> {
    match arg {
        None | Some(Value::Unset) => Ok(None),
        Some(Value::List(items)) => {
            let mut out = Vec::with_capacity(items.len());
            for it in items {
                let v = it.as_integer().ok_or(GeometryError::IndexOutOfRange)?;
                if v < 1 {
                    return Err(GeometryError::IndexOutOfRange);
                }
                out.push(v as usize);
            }
            Ok(Some(out))
        }
        Some(_) => Err(GeometryError::IndexOutOfRange),
    }
}

/// Turn one wire index value (a positive integer, one-based) into a
/// zero-based vertex index into a point list of length `n`, applying the
/// optional `PnIndex` indirection.
fn resolve_vertex(value: &Value, pn: &Option<Vec<usize>>, n: usize) -> Result<u32, GeometryError> {
    let raw = value.as_integer().ok_or(GeometryError::IndexOutOfRange)?;
    if raw < 1 {
        return Err(GeometryError::IndexOutOfRange);
    }
    let one_based = match pn {
        // CoordIndex selects PnIndex[i] (both one-based); the result is
        // the one-based point-list row.
        Some(table) => *table
            .get((raw - 1) as usize)
            .ok_or(GeometryError::IndexOutOfRange)?,
        None => raw as usize,
    };
    if one_based < 1 || one_based > n {
        return Err(GeometryError::IndexOutOfRange);
    }
    Ok((one_based - 1) as u32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::parse_step;

    fn wrap(data: &str) -> String {
        format!(
            "ISO-10303-21;\nHEADER;\n\
             FILE_DESCRIPTION((''),'2;1');\n\
             FILE_NAME('t.ifc','2026-06-12T00:00:00',('a'),('o'),'p','s','auth');\n\
             FILE_SCHEMA(('IFC4'));\nENDSEC;\nDATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n"
        )
    }

    fn parse(data: &str) -> StepFile {
        parse_step(wrap(data).as_bytes()).expect("parse failed")
    }

    #[test]
    fn triangulated_unit_tetra() {
        // 4 points, 1 triangle (1-based indices into the point list).
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.),(0.,0.,1.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3),(1,3,4)),$);",
        );
        let m = tessellate_item(&f, 2).unwrap();
        assert_eq!(m.vertex_count(), 4);
        assert_eq!(m.triangle_count(), 2);
        // 1-based → 0-based.
        assert_eq!(m.triangles[0], [0, 1, 2]);
        assert_eq!(m.triangles[1], [0, 2, 3]);
        assert_eq!(m.positions[3], [0.0, 0.0, 1.0]);
    }

    #[test]
    fn triangulated_with_pn_index_indirection() {
        // PnIndex remaps CoordIndex values to point rows: CoordIndex
        // value 1 → PnIndex[1] = 3 → point row 3 (0-based vertex 2).
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((9.,9.,9.),(8.,8.,8.),(0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),(3,4,5));",
        );
        let m = tessellate_item(&f, 2).unwrap();
        // CoordIndex (1,2,3) → PnIndex rows (3,4,5) → 0-based (2,3,4).
        assert_eq!(m.triangles, vec![[2, 3, 4]]);
        assert_eq!(m.positions[2], [0.0, 0.0, 0.0]);
    }

    #[test]
    fn polygonal_quad_fan_triangulates() {
        // One quad face (4 indices) fan-triangulates into 2 triangles.
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(1.,1.,0.),(0.,1.,0.)));\n\
             #2=IFCINDEXEDPOLYGONALFACE((1,2,3,4));\n\
             #3=IFCPOLYGONALFACESET(#1,$,(#2),$);",
        );
        let m = tessellate_item(&f, 3).unwrap();
        assert_eq!(m.vertex_count(), 4);
        assert_eq!(m.triangle_count(), 2);
        assert_eq!(m.triangles[0], [0, 1, 2]);
        assert_eq!(m.triangles[1], [0, 2, 3]);
    }

    #[test]
    fn faceted_brep_single_triangle_face() {
        // One closed shell, one triangular face, outer poly-loop of three
        // directly-referenced cartesian points.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((0.,1.,0.));\n\
             #4=IFCPOLYLOOP((#1,#2,#3));\n\
             #5=IFCFACEOUTERBOUND(#4,.T.);\n\
             #6=IFCFACE((#5));\n\
             #7=IFCCLOSEDSHELL((#6));\n\
             #8=IFCFACETEDBREP(#7);",
        );
        let m = tessellate_item(&f, 8).unwrap();
        assert_eq!(m.vertex_count(), 3);
        assert_eq!(m.triangle_count(), 1);
        assert_eq!(m.triangles[0], [0, 1, 2]);
        assert_eq!(m.positions[1], [1.0, 0.0, 0.0]);
    }

    #[test]
    fn faceted_brep_shares_vertices_across_faces() {
        // Two faces of a tetrahedron sharing an edge (#1,#2): the shared
        // points are interned once → 4 verts, 2 triangles.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((0.,1.,0.));\n\
             #4=IFCCARTESIANPOINT((0.,0.,1.));\n\
             #10=IFCPOLYLOOP((#1,#2,#3));\n\
             #11=IFCFACEOUTERBOUND(#10,.T.);\n\
             #12=IFCFACE((#11));\n\
             #13=IFCPOLYLOOP((#1,#2,#4));\n\
             #14=IFCFACEOUTERBOUND(#13,.T.);\n\
             #15=IFCFACE((#14));\n\
             #16=IFCCLOSEDSHELL((#12,#15));\n\
             #17=IFCFACETEDBREP(#16);",
        );
        let m = tessellate_item(&f, 17).unwrap();
        assert_eq!(m.vertex_count(), 4);
        assert_eq!(m.triangle_count(), 2);
        // Face 1 → (0,1,2); face 2 shares #1,#2 (verts 0,1), adds #4 (3).
        assert_eq!(m.triangles[0], [0, 1, 2]);
        assert_eq!(m.triangles[1], [0, 1, 3]);
    }

    #[test]
    fn faceted_brep_quad_face_fans() {
        // A 4-vertex outer bound fan-triangulates into 2 triangles.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((1.,1.,0.));\n\
             #4=IFCCARTESIANPOINT((0.,1.,0.));\n\
             #5=IFCPOLYLOOP((#1,#2,#3,#4));\n\
             #6=IFCFACEOUTERBOUND(#5,.T.);\n\
             #7=IFCFACE((#6));\n\
             #8=IFCCLOSEDSHELL((#7));\n\
             #9=IFCFACETEDBREP(#8);",
        );
        let m = tessellate_item(&f, 9).unwrap();
        assert_eq!(m.triangle_count(), 2);
        assert_eq!(m.triangles[0], [0, 1, 2]);
        assert_eq!(m.triangles[1], [0, 2, 3]);
    }

    #[test]
    fn faceted_brep_orientation_false_reverses_winding() {
        // Orientation .F. reverses the loop sense (flips the normal).
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((0.,1.,0.));\n\
             #4=IFCPOLYLOOP((#1,#2,#3));\n\
             #5=IFCFACEBOUND(#4,.F.);\n\
             #6=IFCFACE((#5));\n\
             #7=IFCCLOSEDSHELL((#6));\n\
             #8=IFCFACETEDBREP(#7);",
        );
        let m = tessellate_item(&f, 8).unwrap();
        // Reversed [0,1,2] → [2,1,0]; fan v0=2 → (2,1,0).
        assert_eq!(m.triangles[0], [2, 1, 0]);
    }

    #[test]
    fn faceted_brep_falls_back_to_sole_bound_without_outer() {
        // A single IfcFaceBound (not flagged outer) is still triangulated.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((0.,1.,0.));\n\
             #4=IFCPOLYLOOP((#1,#2,#3));\n\
             #5=IFCFACEBOUND(#4,.T.);\n\
             #6=IFCFACE((#5));\n\
             #7=IFCCLOSEDSHELL((#6));\n\
             #8=IFCFACETEDBREP(#7);",
        );
        let m = tessellate_item(&f, 8).unwrap();
        assert_eq!(m.triangle_count(), 1);
        assert_eq!(m.triangles[0], [0, 1, 2]);
    }

    #[test]
    fn faceted_brep_with_voids_uses_outer_shell() {
        // …WithVoids: only the Outer shell (arg 0) is meshed; the Voids
        // set (arg 1) is ignored in this slice.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((0.,1.,0.));\n\
             #4=IFCPOLYLOOP((#1,#2,#3));\n\
             #5=IFCFACEOUTERBOUND(#4,.T.);\n\
             #6=IFCFACE((#5));\n\
             #7=IFCCLOSEDSHELL((#6));\n\
             #20=IFCCARTESIANPOINT((9.,9.,9.));\n\
             #21=IFCFACETEDBREPWITHVOIDS(#7,(#7));",
        );
        let m = tessellate_item(&f, 21).unwrap();
        assert_eq!(m.vertex_count(), 3);
        assert_eq!(m.triangle_count(), 1);
    }

    #[test]
    fn shell_based_surface_model_meshes_all_shells() {
        // A shell-based surface model with two single-face shells.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((0.,1.,0.));\n\
             #4=IFCPOLYLOOP((#1,#2,#3));\n\
             #5=IFCFACEOUTERBOUND(#4,.T.);\n\
             #6=IFCFACE((#5));\n\
             #7=IFCOPENSHELL((#6));\n\
             #8=IFCSHELLBASEDSURFACEMODEL((#7));",
        );
        let m = tessellate_item(&f, 8).unwrap();
        assert_eq!(m.triangle_count(), 1);
    }

    #[test]
    fn face_based_surface_model_meshes_faces() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((0.,1.,0.));\n\
             #4=IFCPOLYLOOP((#1,#2,#3));\n\
             #5=IFCFACEOUTERBOUND(#4,.T.);\n\
             #6=IFCFACE((#5));\n\
             #7=IFCCONNECTEDFACESET((#6));\n\
             #8=IFCFACEBASEDSURFACEMODEL((#7));",
        );
        let m = tessellate_item(&f, 8).unwrap();
        assert_eq!(m.triangle_count(), 1);
    }

    #[test]
    fn faceted_brep_degenerate_loop_rejected() {
        // A poly-loop with fewer than three points is malformed.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #4=IFCPOLYLOOP((#1,#2));\n\
             #5=IFCFACEOUTERBOUND(#4,.T.);\n\
             #6=IFCFACE((#5));\n\
             #7=IFCCLOSEDSHELL((#6));\n\
             #8=IFCFACETEDBREP(#7);",
        );
        assert_eq!(
            tessellate_item(&f, 8).unwrap_err(),
            GeometryError::IndexOutOfRange
        );
    }

    #[test]
    fn shape_representation_skips_unsupported_items() {
        // A representation mixing a (currently unsupported) extruded
        // solid with a triangulated body still yields the body mesh.
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);\n\
             #3=IFCEXTRUDEDAREASOLID(#9,#9,#9,1.0);\n\
             #4=IFCSHAPEREPRESENTATION(#8,'Body','Tessellation',(#3,#2));",
        );
        let m = mesh_from_shape_representation(&f, 4).unwrap();
        assert_eq!(m.triangle_count(), 1);
    }

    #[test]
    fn all_unsupported_surfaces_keyword() {
        let f = parse(
            "#3=IFCEXTRUDEDAREASOLID(#9,#9,#9,1.0);\n\
             #4=IFCSHAPEREPRESENTATION(#8,'Body','SweptSolid',(#3));",
        );
        let err = mesh_from_shape_representation(&f, 4).unwrap_err();
        assert_eq!(
            err,
            GeometryError::Unsupported("IFCEXTRUDEDAREASOLID".to_string())
        );
    }

    #[test]
    fn out_of_range_index_is_error() {
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);",
        );
        assert_eq!(
            tessellate_item(&f, 2).unwrap_err(),
            GeometryError::IndexOutOfRange
        );
    }

    #[test]
    fn zero_index_rejected() {
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((0,1,2)),$);",
        );
        assert_eq!(
            tessellate_item(&f, 2).unwrap_err(),
            GeometryError::IndexOutOfRange
        );
    }

    fn approx(a: [f64; 3], b: [f64; 3]) {
        for i in 0..3 {
            assert!((a[i] - b[i]).abs() < 1e-9, "axis {i}: {a:?} != {b:?}");
        }
    }

    #[test]
    fn default_axes_are_world_identity() {
        // Axis = $, RefDirection = $ → X=[1,0,0] Y=[0,1,0] Z=[0,0,1].
        let [x, y, z] = build_axes(None, None);
        approx(x, [1.0, 0.0, 0.0]);
        approx(y, [0.0, 1.0, 0.0]);
        approx(z, [0.0, 0.0, 1.0]);
    }

    #[test]
    fn explicit_z_and_x_axes_round_trip() {
        // Axis=[0,0,1] RefDirection=[1,0,0] is the canonical identity
        // basis, just written out explicitly.
        let [x, y, z] = build_axes(Some([0.0, 0.0, 1.0]), Some([1.0, 0.0, 0.0]));
        approx(x, [1.0, 0.0, 0.0]);
        approx(y, [0.0, 1.0, 0.0]);
        approx(z, [0.0, 0.0, 1.0]);
    }

    #[test]
    fn ref_direction_is_orthogonalised_against_axis() {
        // Z=[0,0,1], RefDirection=[1,1,0] (not unit, not ⟂ to Z but
        // already in-plane): X must be normalised [1,1,0], Y = Z×X.
        let [x, y, z] = build_axes(Some([0.0, 0.0, 1.0]), Some([1.0, 1.0, 0.0]));
        let s = 1.0 / 2f64.sqrt();
        approx(x, [s, s, 0.0]);
        approx(z, [0.0, 0.0, 1.0]);
        // Y = Z × X = [-s, s, 0].
        approx(y, [-s, s, 0.0]);
    }

    #[test]
    fn ref_direction_projected_out_of_plane_component() {
        // Z=[0,0,1], RefDirection=[1,0,1]: the Z component is projected
        // away, leaving X=[1,0,0].
        let [x, _y, z] = build_axes(Some([0.0, 0.0, 1.0]), Some([1.0, 0.0, 1.0]));
        approx(z, [0.0, 0.0, 1.0]);
        approx(x, [1.0, 0.0, 0.0]);
    }

    #[test]
    fn rotated_axis_builds_orthonormal_basis() {
        // Z pointing along world +X. Default RefDirection handling must
        // still yield an orthonormal right-handed basis.
        let [x, y, z] = build_axes(Some([1.0, 0.0, 0.0]), None);
        approx(z, [1.0, 0.0, 0.0]);
        // X ⟂ Z, Y = Z × X — verify orthonormality + handedness.
        assert!((dot(x, z)).abs() < 1e-9);
        assert!((dot(y, z)).abs() < 1e-9);
        assert!((dot(x, y)).abs() < 1e-9);
        approx(cross(z, x), y);
    }

    #[test]
    fn placement_transform_translates_to_location() {
        // A single absolute IfcLocalPlacement at (10,20,30), identity
        // rotation → a point at local origin maps to (10,20,30).
        let f = parse(
            "#1=IFCCARTESIANPOINT((10.,20.,30.));\n\
             #2=IFCAXIS2PLACEMENT3D(#1,$,$);\n\
             #3=IFCLOCALPLACEMENT($,#2);",
        );
        let t = placement_transform(&f, 3).unwrap();
        approx(t.apply([0.0, 0.0, 0.0]), [10.0, 20.0, 30.0]);
        approx(t.apply([1.0, 2.0, 3.0]), [11.0, 22.0, 33.0]);
    }

    #[test]
    fn placement_transform_composes_chain() {
        // Parent translates by (100,0,0); child translates by (0,5,0)
        // relative to parent → world (100,5,0) for the child origin.
        let f = parse(
            "#1=IFCCARTESIANPOINT((100.,0.,0.));\n\
             #2=IFCAXIS2PLACEMENT3D(#1,$,$);\n\
             #3=IFCLOCALPLACEMENT($,#2);\n\
             #4=IFCCARTESIANPOINT((0.,5.,0.));\n\
             #5=IFCAXIS2PLACEMENT3D(#4,$,$);\n\
             #6=IFCLOCALPLACEMENT(#3,#5);",
        );
        let t = placement_transform(&f, 6).unwrap();
        approx(t.apply([0.0, 0.0, 0.0]), [100.0, 5.0, 0.0]);
        approx(t.apply([1.0, 1.0, 1.0]), [101.0, 6.0, 1.0]);
    }

    #[test]
    fn placement_transform_rotation_then_translation() {
        // Z=+Y (90° rotation mapping local +Z onto world +Y) with a
        // translation: verify the rotation is applied to local axes
        // before the translation.
        // Axis (local Z) = world +Y; RefDirection (local X) = world +X.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCDIRECTION((0.,1.,0.));\n\
             #3=IFCDIRECTION((1.,0.,0.));\n\
             #4=IFCAXIS2PLACEMENT3D(#1,#2,#3);\n\
             #5=IFCLOCALPLACEMENT($,#4);",
        );
        let t = placement_transform(&f, 5).unwrap();
        // local +Z (0,0,1) → world +Y (0,1,0).
        approx(t.apply([0.0, 0.0, 1.0]), [0.0, 1.0, 0.0]);
        // local +X (1,0,0) → world +X (1,0,0).
        approx(t.apply([1.0, 0.0, 0.0]), [1.0, 0.0, 0.0]);
    }

    #[test]
    fn placement_cycle_is_bounded() {
        // A self-referential PlacementRelTo must not loop forever.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCAXIS2PLACEMENT3D(#1,$,$);\n\
             #3=IFCLOCALPLACEMENT(#3,#2);",
        );
        // Should return Ok within the depth cap (identity translation).
        let t = placement_transform(&f, 3).unwrap();
        approx(t.apply([7.0, 8.0, 9.0]), [7.0, 8.0, 9.0]);
    }

    #[test]
    fn trimesh_transform_applies_to_all_vertices() {
        let mut m = TriMesh {
            positions: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            triangles: vec![],
        };
        let t = Transform {
            cols: Transform::IDENTITY.cols,
            translation: [5.0, 0.0, 0.0],
        };
        let moved = m.transformed(&t);
        approx(moved.positions[0], [5.0, 0.0, 0.0]);
        approx(moved.positions[1], [6.0, 0.0, 0.0]);
        m.transform(&t);
        approx(m.positions[0], [5.0, 0.0, 0.0]);
    }

    #[test]
    fn merged_meshes_offset_indices() {
        let mut dst = TriMesh {
            positions: vec![[0.0; 3], [1.0; 3]],
            triangles: vec![[0, 1, 0]],
        };
        let src = TriMesh {
            positions: vec![[2.0; 3], [3.0; 3]],
            triangles: vec![[0, 1, 0]],
        };
        append_mesh(&mut dst, src);
        assert_eq!(dst.vertex_count(), 4);
        assert_eq!(dst.triangles, vec![[0, 1, 0], [2, 3, 2]]);
    }
}
