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
//! Alongside the index-based tessellations this module also evaluates
//! the **faceted boundary representation** family, whose faces are
//! explicit polygons of `IfcCartesianPoint` references rather than
//! indices into a shared list:
//!
//! * **`IfcFacetedBrep`** / **`IfcFacetedBrepWithVoids`** — a manifold
//!   solid whose `Outer` (and, for the …WithVoids subtype, each `Voids`)
//!   is an `IfcClosedShell` (§8.8.3.18 / IFC4 EXPRESS
//!   `IfcManifoldSolidBrep.Outer`).
//! * **`IfcFaceBasedSurfaceModel`** (`FbsmFaces : SET OF
//!   IfcConnectedFaceSet`) and **`IfcShellBasedSurfaceModel`**
//!   (`SbsmBoundary : SET OF IfcShell`, the SELECT of
//!   `IfcClosedShell` / `IfcOpenShell`) — collections of shells.
//!
//! Each shell (`IfcConnectedFaceSet.CfsFaces : SET OF IfcFace`) holds
//! `IfcFace`s; every face's outer `IfcFaceBound` (the `IfcFaceOuterBound`
//! when present, else the first bound) carries an `IfcPolyLoop` whose
//! `Polygon : LIST [3:?] OF IfcCartesianPoint` is fan-triangulated. The
//! shared vertex table is de-duplicated by `IfcCartesianPoint` id so a
//! point referenced by several loops contributes one mesh vertex
//! (§8.8.3.18: "each Cartesian point shall be referenced by at least
//! three poly loops"). `Voids` inner shells and per-bound `Orientation`
//! flags are not yet applied — the outer surface is meshed as authored.
//!
//! Beyond the face-set / Brep families this module also sweeps the
//! linear **swept area solid**:
//!
//! * **`IfcExtrudedAreaSolid`** (`SweptArea`, `Position`,
//!   `ExtrudedDirection`, `Depth`) — a 2-D profile area swept along a
//!   direction by a depth into a closed prism (§8.8.3.15). The profile is
//!   resolved to its outer ring from an `IfcArbitraryClosedProfileDef`
//!   (`IfcPolyline` outer curve) or an `IfcRectangleProfileDef`
//!   (centred `XDim`×`YDim`, with an optional 2-D `Position`), then a
//!   bottom cap, a `Depth · ExtrudedDirection`-offset top cap, and a
//!   side-wall quad per profile edge are emitted and re-placed by the
//!   solid's optional `Position` `IfcAxis2Placement3D`.
//!
//! It also resolves the **mapped item** instancing entity:
//!
//! * **`IfcMappedItem`** (`MappingSource`, `MappingTarget`) — the
//!   inserted instance of a source `IfcRepresentationMap`'s
//!   `MappedRepresentation`, meshed in its own frame, lifted into the
//!   map's `MappingOrigin` `IfcAxis2Placement`, then placed by the
//!   `MappingTarget` `IfcCartesianTransformationOperator`
//!   (2D / 3D / 3DnonUniform — orthonormal `IfcBaseAxis` columns scaled
//!   by `Scale`(`/Scale2`/`Scale3`), translated by `LocalOrigin`).
//!   Mapped items may nest (a source representation can contain further
//!   mapped items); recursion is bounded by a depth cap.
//!
//! Still later Phase-3 work (reported as [`GeometryError::Unsupported`]
//! rather than silently dropped): the other swept solids
//! (`IfcRevolvedAreaSolid`, `IfcSurfaceCurveSweptAreaSolid`, the tapered
//! extrusion), parameterised profiles beyond the rectangle, curved
//! (`IfcIndexedPolyCurve`/circle) profile curves, advanced/curved breps
//! (`IfcAdvancedBrep`, `IfcFaceSurface`), and boolean results.

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
    /// evaluate (revolved swept solid, advanced Brep, boolean result,
    /// …). Carries the offending entity keyword.
    Unsupported(String),
    /// A face-set's `Coordinates` reference is missing, not an
    /// `IfcCartesianPointList3D`, or otherwise malformed.
    BadCoordinates,
    /// A one-based index (in `CoordIndex` / a face / `PnIndex`) is zero
    /// or points past the end of the list it indexes.
    IndexOutOfRange,
    /// A coordinate row did not have three numeric components.
    BadCoordinate,
    /// A swept-area solid's profile (`SweptArea`) is malformed, of an
    /// unsupported profile kind, or yields fewer than three planar
    /// points.
    BadProfile,
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
            Self::BadProfile => f.write_str("swept-area solid profile is malformed or unsupported"),
        }
    }
}

impl std::error::Error for GeometryError {}

/// Resolve and tessellate one geometric-representation item by id.
///
/// Dispatches on the entity keyword: `IFCTRIANGULATEDFACESET` and
/// `IFCPOLYGONALFACESET` produce a [`TriMesh`]; any other keyword is a
/// [`GeometryError::Unsupported`]. This is the lowest-level entry —
/// most callers want [`mesh_from_shape_representation`] or the
/// `Model`-level walk.
pub fn tessellate_item(step: &StepFile, id: u64) -> Result<TriMesh, GeometryError> {
    tessellate_item_depth(step, id, 0)
}

/// The largest `IfcMappedItem` / `IfcShapeRepresentation` nesting depth
/// that [`tessellate_item`] will follow. Mapped items may reuse other
/// mapped items (nested blocks); this bound keeps a malformed self-
/// referential map (which the EXPRESS `ApplicableMappedRepr` informal
/// proposition forbids, but a file may still contain) from recursing
/// without end.
const MAX_MAP_DEPTH: usize = 64;

/// Depth-tracked core of [`tessellate_item`]. `depth` counts how many
/// `IfcMappedItem` indirections have been followed so far.
fn tessellate_item_depth(step: &StepFile, id: u64, depth: usize) -> Result<TriMesh, GeometryError> {
    let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
    match inst.keyword.as_str() {
        "IFCTRIANGULATEDFACESET" => triangulated_face_set(step, &inst.args),
        "IFCPOLYGONALFACESET" => polygonal_face_set(step, &inst.args),
        // Faceted boundary representation: Outer (and, for the …WithVoids
        // subtype, Voids) are IfcClosedShells of polygonal IfcFaces.
        "IFCFACETEDBREP" | "IFCFACETEDBREPWITHVOIDS" => faceted_brep(step, &inst.args),
        // Surface models: collections of IfcConnectedFaceSet / IfcShell.
        "IFCFACEBASEDSURFACEMODEL" | "IFCSHELLBASEDSURFACEMODEL" => surface_model(step, &inst.args),
        // Swept area solid: sweep a 2-D profile along a direction.
        "IFCEXTRUDEDAREASOLID" => extruded_area_solid(step, &inst.args),
        // Revolved area solid: revolve a 2-D profile about an axis line.
        "IFCREVOLVEDAREASOLID" => revolved_area_solid(step, &inst.args),
        // Mapped item: instance a source representation under a Cartesian
        // transformation operator.
        "IFCMAPPEDITEM" => mapped_item(step, &inst.args, depth),
        other => Err(GeometryError::Unsupported(other.to_string())),
    }
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
    mesh_from_shape_representation_depth(step, shape_rep_id, 0)
}

/// Depth-tracked core of [`mesh_from_shape_representation`]. `depth` is
/// the `IfcMappedItem` nesting depth carried through to
/// [`tessellate_item_depth`].
fn mesh_from_shape_representation_depth(
    step: &StepFile,
    shape_rep_id: u64,
    depth: usize,
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
        match tessellate_item_depth(step, item_id, depth) {
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
// Faceted boundary representation
//
//   IfcManifoldSolidBrep.Outer : IfcClosedShell        (attr index 0)
//   IfcFacetedBrep        : SUBTYPE OF IfcManifoldSolidBrep (no new attrs)
//   IfcFacetedBrepWithVoids(Outer, Voids : SET OF IfcClosedShell)
//   IfcConnectedFaceSet.CfsFaces : SET OF IfcFace       (attr index 0)
//   IfcFace.Bounds : SET OF IfcFaceBound                (attr index 0)
//   IfcFaceBound(Bound : IfcLoop, Orientation)          (Bound index 0)
//   IfcPolyLoop.Polygon : LIST [3:?] OF IfcCartesianPoint (attr index 0)
//
// The faces reference IfcCartesianPoints directly, so vertices are
// pooled into a shared table keyed by point id (a point shared by N
// loops becomes one mesh vertex) and each face's outer loop is
// fan-triangulated.
// ---------------------------------------------------------------------

/// A growing, point-id-deduplicated vertex pool used while walking a
/// Brep / surface-model face graph.
struct VertexPool {
    positions: Vec<[f64; 3]>,
    /// Map from `IfcCartesianPoint` instance id → its index in
    /// `positions`. A point referenced by several poly loops resolves to
    /// the same mesh vertex.
    index_of: std::collections::HashMap<u64, u32>,
}

impl VertexPool {
    fn new() -> Self {
        Self {
            positions: Vec::new(),
            index_of: std::collections::HashMap::new(),
        }
    }

    /// Resolve (or insert) the `IfcCartesianPoint` with id `point_id`,
    /// returning its zero-based index in the pool.
    fn intern(&mut self, step: &StepFile, point_id: u64) -> Result<u32, GeometryError> {
        if let Some(&idx) = self.index_of.get(&point_id) {
            return Ok(idx);
        }
        let p = cartesian_point(step, Some(&Value::Reference(point_id)))?;
        let idx = self.positions.len() as u32;
        self.positions.push(p);
        self.index_of.insert(point_id, idx);
        Ok(idx)
    }
}

/// Tessellate an `IfcFacetedBrep` / `IfcFacetedBrepWithVoids`. The
/// `Outer` closed shell (attribute index 0) is meshed; the optional
/// `Voids` shells (index 1, …WithVoids only) are appended as additional
/// surface — boolean subtraction is a later slice, but emitting the void
/// shells keeps their geometry visible rather than dropped.
fn faceted_brep(step: &StepFile, args: &[Value]) -> Result<TriMesh, GeometryError> {
    let outer = args
        .first()
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    let mut pool = VertexPool::new();
    let mut triangles = Vec::new();
    connected_face_set(step, outer, &mut pool, &mut triangles)?;
    // IfcFacetedBrepWithVoids.Voids : SET OF IfcClosedShell (attr index 1).
    if let Some(voids) = args.get(1).and_then(Value::as_list) {
        for v in voids {
            let Some(shell_id) = v.as_reference() else {
                continue;
            };
            connected_face_set(step, shell_id, &mut pool, &mut triangles)?;
        }
    }
    Ok(TriMesh {
        positions: pool.positions,
        triangles,
    })
}

/// Tessellate an `IfcFaceBasedSurfaceModel` (`FbsmFaces`) or
/// `IfcShellBasedSurfaceModel` (`SbsmBoundary`). Both carry their shells
/// as a SET in attribute index 0: a list of `IfcConnectedFaceSet`
/// (face-based) or `IfcShell` SELECT references (shell-based, the SELECT
/// resolving to `IfcClosedShell` / `IfcOpenShell`, both connected face
/// sets). Every shell's faces are merged into one mesh over a shared
/// vertex pool.
fn surface_model(step: &StepFile, args: &[Value]) -> Result<TriMesh, GeometryError> {
    let shells = args
        .first()
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinates)?;
    let mut pool = VertexPool::new();
    let mut triangles = Vec::new();
    for shell in shells {
        let shell_id = shell.as_reference().ok_or(GeometryError::BadCoordinates)?;
        connected_face_set(step, shell_id, &mut pool, &mut triangles)?;
    }
    Ok(TriMesh {
        positions: pool.positions,
        triangles,
    })
}

// ---------------------------------------------------------------------
// IfcExtrudedAreaSolid
//
//   IfcSweptAreaSolid.SweptArea : IfcProfileDef      (attr index 0)
//   IfcSweptAreaSolid.Position  : OPTIONAL IfcAxis2Placement3D (index 1)
//   IfcExtrudedAreaSolid.ExtrudedDirection : IfcDirection (index 2)
//   IfcExtrudedAreaSolid.Depth  : IfcPositiveLengthMeasure (index 3)
//
// The profile (a closed 2-D area) is swept along ExtrudedDirection by
// Depth. Both ExtrudedDirection and the profile are expressed in the
// solid's `Position` coordinate system (ISO 16739 §8.8.3.15); the whole
// result is then mapped into the representation's local space by the
// `Position` IfcAxis2Placement3D affine.
//
// The mesh built here is a closed prism over the (assumed convex,
// planar, CCW) profile ring: a bottom cap, a top cap offset by
// `Depth · ExtrudedDirection`, and a quad side wall per profile edge.
// Profile inner boundaries (holes) and the tapered subtype are not yet
// applied — the outer ring is meshed as authored.
// ---------------------------------------------------------------------
fn extruded_area_solid(step: &StepFile, args: &[Value]) -> Result<TriMesh, GeometryError> {
    // SweptArea (profile) — attribute index 0.
    let profile_id = args
        .first()
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadProfile)?;
    let ring = profile_ring(step, profile_id)?;
    if ring.len() < 3 {
        return Err(GeometryError::BadProfile);
    }

    // ExtrudedDirection (index 2) and Depth (index 3), both in the
    // Position coordinate system.
    let dir = direction(step, args.get(2))?.ok_or(GeometryError::BadCoordinates)?;
    let depth = args
        .get(3)
        .and_then(Value::as_number)
        .ok_or(GeometryError::BadCoordinate)?;
    // The sweep vector: Depth times the (normalised) extruded direction.
    let d = normalise(dir).unwrap_or(dir);
    let sweep = [d[0] * depth, d[1] * depth, d[2] * depth];

    let n = ring.len();
    let mut positions: Vec<[f64; 3]> = Vec::with_capacity(n * 2);
    // Bottom ring (profile plane, local z = 0).
    for &[x, y] in &ring {
        positions.push([x, y, 0.0]);
    }
    // Top ring (bottom + sweep).
    for &[x, y] in &ring {
        positions.push([x + sweep[0], y + sweep[1], sweep[2]]);
    }

    let mut triangles: Vec<[u32; 3]> = Vec::with_capacity((n - 2) * 2 + n * 2);
    // Bottom cap (fan from vertex 0), wound to face away from the sweep
    // (reversed relative to the top cap).
    for i in 1..(n - 1) {
        triangles.push([0, (i + 1) as u32, i as u32]);
    }
    // Top cap (fan from the first top vertex), normal winding.
    let top = n as u32;
    for i in 1..(n - 1) {
        triangles.push([top, top + i as u32, top + (i + 1) as u32]);
    }
    // Side walls: one quad (two triangles) per profile edge i → i+1.
    for i in 0..n {
        let i_next = (i + 1) % n;
        let b0 = i as u32;
        let b1 = i_next as u32;
        let t0 = top + i as u32;
        let t1 = top + i_next as u32;
        triangles.push([b0, b1, t1]);
        triangles.push([b0, t1, t0]);
    }

    let mut mesh = TriMesh {
        positions,
        triangles,
    };

    // Position: OPTIONAL IfcAxis2Placement3D (index 1). When present it
    // re-places the whole swept solid; absent → local identity.
    if let Some(pos_id) = args.get(1).and_then(Value::as_reference) {
        let xform = axis2_placement_3d(step, pos_id)?;
        mesh.transform(&xform);
    }
    Ok(mesh)
}

// ---------------------------------------------------------------------
// IfcRevolvedAreaSolid (IfcSweptAreaSolid + Axis : IfcAxis1Placement,
//   Angle : IfcPlaneAngleMeasure)
//   inherited args: SweptArea (0), Position (1); own: Axis (2), Angle (3)
//
// The 2-D profile (in the Position XY-plane, z = 0) is revolved about the
// `Axis` line by `Angle` radians (right-hand rule about the axis
// direction). Per the EXPRESS WHERE rules `AxisStartInXY` /
// `AxisDirectionInXY`, both the axis Location and its direction lie in the
// XY-plane. We approximate the revolution by stepping the profile ring
// through a fan of intermediate angular positions, emitting a ring of
// vertices at each step; the side walls stitch adjacent rings, and (for a
// partial, non-2π revolution) the first/last profile rings cap the open
// ends. A full 2π revolution wraps closed with no end caps.
//
// The angular resolution is a fixed segment count — a faithful tessellated
// approximation of the analytic surface of revolution (the spec defines
// the exact swept surface; the mesh density is an extraction choice).
// ---------------------------------------------------------------------

/// Number of angular segments per full 2π of revolution. The actual
/// segment count for a partial sweep is scaled by `Angle / 2π`, with at
/// least one segment.
const REVOLVE_SEGMENTS_PER_TURN: usize = 48;

fn revolved_area_solid(step: &StepFile, args: &[Value]) -> Result<TriMesh, GeometryError> {
    // SweptArea (index 0).
    let profile_id = args
        .first()
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadProfile)?;
    let ring = profile_ring(step, profile_id)?;
    if ring.len() < 3 {
        return Err(GeometryError::BadProfile);
    }

    // Axis : IfcAxis1Placement (index 2) — its Location and direction.
    let axis_id = args
        .get(2)
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    let (axis_origin, axis_dir) = axis1_placement(step, axis_id)?;

    // Angle : IfcPlaneAngleMeasure (index 3), radians.
    let angle = args
        .get(3)
        .and_then(Value::as_number)
        .ok_or(GeometryError::BadCoordinate)?;
    if angle == 0.0 {
        return Err(GeometryError::BadProfile);
    }

    // Segment count proportional to the swept angle (≥1).
    let frac = (angle.abs() / (2.0 * core::f64::consts::PI)).min(1.0);
    let segments = ((REVOLVE_SEGMENTS_PER_TURN as f64 * frac).ceil() as usize).max(1);
    let two_pi = 2.0 * core::f64::consts::PI;
    // A full turn (within tolerance) wraps closed: no end caps, and the
    // last ring coincides with the first.
    let full_turn = (angle.abs() - two_pi).abs() < 1e-9 || angle.abs() > two_pi;
    let ring_count = if full_turn { segments } else { segments + 1 };

    let n = ring.len();
    let mut positions: Vec<[f64; 3]> = Vec::with_capacity(n * ring_count);
    for s in 0..ring_count {
        // Angular position of this ring (the last ring of a full turn is
        // not emitted separately — it reuses ring 0).
        let theta = angle * (s as f64) / (segments as f64);
        for &[x, y] in &ring {
            let p = [x, y, 0.0];
            positions.push(rotate_about_axis(p, axis_origin, axis_dir, theta));
        }
    }

    let mut triangles: Vec<[u32; 3]> = Vec::new();
    // Side walls: stitch each adjacent pair of rings into quads.
    for s in 0..segments {
        let a = ((s % ring_count) * n) as u32;
        let b = (((s + 1) % ring_count) * n) as u32;
        for i in 0..n {
            let i_next = ((i + 1) % n) as u32;
            let i = i as u32;
            // Quad (a+i, a+i_next, b+i_next, b+i).
            triangles.push([a + i, a + i_next, b + i_next]);
            triangles.push([a + i, b + i_next, b + i]);
        }
    }
    // End caps for a partial revolution: fan-triangulate the first and
    // last profile rings (the open ends of the swept volume).
    if !full_turn {
        let first = 0u32;
        for i in 1..(n - 1) {
            triangles.push([first, first + (i + 1) as u32, first + i as u32]);
        }
        let last = (segments * n) as u32;
        for i in 1..(n - 1) {
            triangles.push([last, last + i as u32, last + (i + 1) as u32]);
        }
    }

    let mut mesh = TriMesh {
        positions,
        triangles,
    };

    // Position: OPTIONAL IfcAxis2Placement3D (index 1) re-places the solid.
    if let Some(pos_id) = args.get(1).and_then(Value::as_reference) {
        let xform = axis2_placement_3d(step, pos_id)?;
        mesh.transform(&xform);
    }
    Ok(mesh)
}

/// Resolve an `IfcAxis1Placement(Location, Axis)` to its `(origin,
/// direction)` pair. `Axis` is `OPTIONAL`; the EXPRESS derived `Z`
/// defaults it to world +Z (`[0,0,1]`).
fn axis1_placement(step: &StepFile, id: u64) -> Result<([f64; 3], [f64; 3]), GeometryError> {
    let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
    if inst.keyword != "IFCAXIS1PLACEMENT" {
        return Err(GeometryError::BadCoordinates);
    }
    let origin = cartesian_point(step, inst.args.first())?;
    let dir = direction(step, inst.args.get(1))?
        .and_then(normalise)
        .unwrap_or([0.0, 0.0, 1.0]);
    Ok((origin, dir))
}

/// Rotate point `p` about the line through `origin` with unit direction
/// `axis` by `theta` radians (right-hand rule), via Rodrigues' rotation
/// formula applied to the offset `v = p − origin`.
fn rotate_about_axis(p: [f64; 3], origin: [f64; 3], axis: [f64; 3], theta: f64) -> [f64; 3] {
    let k = normalise(axis).unwrap_or([0.0, 0.0, 1.0]);
    let v = [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]];
    let c = theta.cos();
    let s = theta.sin();
    let kv = dot_raw(k, v);
    let kxv = cross_raw(k, v);
    // v_rot = v·cosθ + (k×v)·sinθ + k·(k·v)·(1−cosθ).
    let r = [
        v[0] * c + kxv[0] * s + k[0] * kv * (1.0 - c),
        v[1] * c + kxv[1] * s + k[1] * kv * (1.0 - c),
        v[2] * c + kxv[2] * s + k[2] * kv * (1.0 - c),
    ];
    [r[0] + origin[0], r[1] + origin[1], r[2] + origin[2]]
}

/// Raw dot product (no re-normalisation — unlike [`dot`], which the
/// EXPRESS `IfcDotProduct` definition normalises its inputs first).
fn dot_raw(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Raw cross product (no input re-normalisation — unlike [`cross`]).
fn cross_raw(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

// =====================================================================
// IfcMappedItem  (MappingSource, MappingTarget)
//
// A mapped item is the inserted instance of a *source* representation
// (an IfcRepresentationMap, "block / cell / macro definition") placed by
// a Cartesian transformation operator (MappingTarget). It lets one
// representation reuse another — and mapped items may themselves nest
// (a source representation can contain further IfcMappedItems).
//
//   IfcMappedItem
//     MappingSource : IfcRepresentationMap
//       MappingOrigin       : IfcAxis2Placement   (the source frame)
//       MappedRepresentation: IfcShapeRepresentation
//     MappingTarget : IfcCartesianTransformationOperator(2D|3D[nonUniform])
//       Axis1, Axis2, LocalOrigin, Scale [, Axis3] [, Scale2, Scale3]
//
// The source representation's geometry is authored about the
// MappingOrigin placement; the target operator maps that frame to its
// destination. The effective placement of the mapped geometry is
//   target_operator ∘ mapping_origin
// (both are affine frames; we mesh the source items in their own local
// space, lift them into the source MappingOrigin frame, then apply the
// target operator). The Axis2-default case — MappingOrigin at the world
// origin and an identity operator — leaves the source geometry exactly
// where it was authored, which is the common IFC case.
// =====================================================================

fn mapped_item(step: &StepFile, args: &[Value], depth: usize) -> Result<TriMesh, GeometryError> {
    if depth >= MAX_MAP_DEPTH {
        // A self-referential / over-deep map chain: stop following it
        // (the file violates the ApplicableMappedRepr informal
        // proposition) rather than recurse without bound.
        return Err(GeometryError::Unsupported("IFCMAPPEDITEM".to_string()));
    }
    // MappingSource (index 0) : IfcRepresentationMap.
    let map_id = args
        .first()
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    let map = step
        .get(map_id)
        .ok_or(GeometryError::MissingInstance(map_id))?;
    if map.keyword != "IFCREPRESENTATIONMAP" {
        return Err(GeometryError::Unsupported(map.keyword.clone()));
    }
    // IfcRepresentationMap(MappingOrigin, MappedRepresentation).
    let origin_id = map
        .args
        .first()
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    let mapped_rep_id = map
        .args
        .get(1)
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;

    // Mesh the source representation's items (recursing one level deeper
    // so nested mapped items are bounded). Vertices come out in the
    // source representation's own local frame.
    let mut mesh = mesh_from_shape_representation_depth(step, mapped_rep_id, depth + 1)?;

    // MappingOrigin : IfcAxis2Placement (2D or 3D); fold it in first so
    // the source geometry sits in the map's reference frame.
    let origin = axis2_placement_3d(step, origin_id)?;
    if origin != Transform::IDENTITY {
        mesh.transform(&origin);
    }

    // MappingTarget (index 1) : IfcCartesianTransformationOperator.
    let target_id = args
        .get(1)
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    let target = transformation_operator(step, target_id)?;
    mesh.transform(&target);

    Ok(mesh)
}

/// Resolve an `IfcCartesianTransformationOperator` (2D / 3D /
/// 3DnonUniform) to a [`Transform`].
///
/// Common attributes (`IfcCartesianTransformationOperator`):
/// `(Axis1, Axis2, LocalOrigin, Scale)`; the 3-D subtype adds `Axis3`
/// (index 4) and the non-uniform 3-D subtype `Scale2`, `Scale3`
/// (indices 5, 6).
///
/// The operator's axes are derived by the EXPRESS `IfcBaseAxis`
/// function: it orthonormalises the supplied `Axis1`/`Axis2`(/`Axis3`),
/// defaulting any absent axis to the corresponding world axis. We reuse
/// the placement [`build_axes`] derivation (`Axis2`→ref-X, `Axis3`→Z is
/// not the operator convention, so the operator orders its columns
/// directly): column *i* is `U[i] · Scl[i]`, translation is
/// `LocalOrigin`. A `$`/absent `Scale` is 1.0 (the non-uniform `Scale2`
/// / `Scale3` default to that uniform `Scale`).
fn transformation_operator(step: &StepFile, id: u64) -> Result<Transform, GeometryError> {
    let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
    let is_3d = matches!(
        inst.keyword.as_str(),
        "IFCCARTESIANTRANSFORMATIONOPERATOR3D" | "IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM"
    );
    let is_2d = matches!(
        inst.keyword.as_str(),
        "IFCCARTESIANTRANSFORMATIONOPERATOR2D" | "IFCCARTESIANTRANSFORMATIONOPERATOR2DNONUNIFORM"
    );
    if !is_3d && !is_2d {
        return Err(GeometryError::Unsupported(inst.keyword.clone()));
    }
    let args = &inst.args;
    // Axis1, Axis2 (both OPTIONAL IfcDirection); Axis3 only on the 3-D
    // subtype (index 4).
    let axis1 = direction(step, args.first())?;
    let axis2 = direction(step, args.get(1))?;
    let axis3 = if is_3d {
        direction(step, args.get(4))?
    } else {
        None
    };
    // LocalOrigin : IfcCartesianPoint (index 2).
    let local_origin = cartesian_point(step, args.get(2))?;
    // Scale : OPTIONAL IfcReal (index 3), default 1.0.
    let scale = args.get(3).and_then(Value::as_number).unwrap_or(1.0);

    // U = IfcBaseAxis: orthonormal [U1, U2, U3]. Axis1 seeds X, Axis2
    // seeds Y; the third is their cross product. We derive it from the
    // shared build_axes machinery by treating Axis1 as the reference X
    // direction in the plane ⟂ to the Z built from Axis1×Axis2 — but the
    // operator's convention is simpler: U1 = normalise(Axis1) (default
    // world X), U2 = the component of Axis2 ⟂ U1 (default world Y), U3 =
    // U1 × U2 (3-D only).
    let u = base_axes(axis1, axis2, axis3, is_3d);

    // Per-column scale: uniform Scale for the 2-D / uniform-3-D operators;
    // Scale2 / Scale3 (indices 5, 6) override columns 2 / 3 on the
    // non-uniform 3-D subtype (default to the uniform Scale).
    let (s1, s2, s3) = if inst.keyword == "IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM" {
        let s2 = args.get(5).and_then(Value::as_number).unwrap_or(scale);
        let s3 = args.get(6).and_then(Value::as_number).unwrap_or(scale);
        (scale, s2, s3)
    } else if inst.keyword == "IFCCARTESIANTRANSFORMATIONOPERATOR2DNONUNIFORM" {
        // 2-D non-uniform: Scale2 at index 4.
        let s2 = args.get(4).and_then(Value::as_number).unwrap_or(scale);
        (scale, s2, scale)
    } else {
        (scale, scale, scale)
    };

    Ok(Transform {
        cols: [
            [u[0][0] * s1, u[0][1] * s1, u[0][2] * s1],
            [u[1][0] * s2, u[1][1] * s2, u[1][2] * s2],
            [u[2][0] * s3, u[2][1] * s3, u[2][2] * s3],
        ],
        translation: local_origin,
    })
}

/// `IfcBaseAxis(Dim, Axis1, Axis2[, Axis3])` → orthonormal axes
/// `[U1, U2, U3]`.
///
/// The operator convention (distinct from `IfcBuildAxes`, which is
/// driven by a Z `Axis`): `U1` is `normalise(Axis1)` defaulting to world
/// X; `U2` is the part of `Axis2` orthogonal to `U1`, defaulting to
/// world Y; `U3 = U1 × U2` for the 3-D case (world Z when both inputs
/// default). For the 2-D case `U3` is left as world Z (unused by the
/// 2-D column write since the source geometry is planar, but kept so the
/// returned basis is well-formed).
fn base_axes(
    axis1: Option<[f64; 3]>,
    axis2: Option<[f64; 3]>,
    axis3: Option<[f64; 3]>,
    is_3d: bool,
) -> [[f64; 3]; 3] {
    let u1 = axis1.and_then(normalise).unwrap_or([1.0, 0.0, 0.0]);
    // U2: Axis2 made ⟂ to U1, or world Y projected ⟂ to U1.
    let raw2 = axis2.and_then(normalise).unwrap_or([0.0, 1.0, 0.0]);
    let d = dot(raw2, u1);
    let proj = [
        raw2[0] - d * u1[0],
        raw2[1] - d * u1[1],
        raw2[2] - d * u1[2],
    ];
    let u2 = normalise(proj).unwrap_or_else(|| {
        // Axis2 parallel to U1: pick a world axis ⟂ to U1.
        let fallback = if u1 != [0.0, 1.0, 0.0] {
            [0.0, 1.0, 0.0]
        } else {
            [0.0, 0.0, 1.0]
        };
        let d = dot(fallback, u1);
        normalise([
            fallback[0] - d * u1[0],
            fallback[1] - d * u1[1],
            fallback[2] - d * u1[2],
        ])
        .unwrap_or([0.0, 1.0, 0.0])
    });
    let u3 = if is_3d {
        // Axis3 (if supplied) is orthonormalised against the U1/U2 plane;
        // otherwise U1 × U2.
        match axis3.and_then(normalise) {
            Some(a3) => {
                let d1 = dot(a3, u1);
                let d2 = dot(a3, u2);
                let proj = [
                    a3[0] - d1 * u1[0] - d2 * u2[0],
                    a3[1] - d1 * u1[1] - d2 * u2[1],
                    a3[2] - d1 * u1[2] - d2 * u2[2],
                ];
                normalise(proj)
                    .unwrap_or_else(|| normalise(cross(u1, u2)).unwrap_or([0.0, 0.0, 1.0]))
            }
            None => normalise(cross(u1, u2)).unwrap_or([0.0, 0.0, 1.0]),
        }
    } else {
        [0.0, 0.0, 1.0]
    };
    [u1, u2, u3]
}

/// Number of segments a full circle / ellipse profile boundary is
/// approximated with (matches [`REVOLVE_SEGMENTS_PER_TURN`], so a
/// revolved circle and a circle profile have consistent density).
const CIRCLE_SEGMENTS: usize = 48;

/// Resolve an `IfcProfileDef` to its outer-boundary ring as a list of
/// 2-D `[x, y]` points (no duplicated closing vertex).
///
/// Supported profile kinds:
/// * `IfcArbitraryClosedProfileDef(ProfileType, ProfileName, OuterCurve)`
///   — the `OuterCurve` (attribute index 2) is a supported planar curve
///   (see [`curve_points_2d`]).
/// * `IfcRectangleProfileDef(ProfileType, ProfileName, Position, XDim,
///   YDim)` — a rectangle centred on its 2-D `Position` origin with full
///   widths `XDim` (index 3) / `YDim` (index 4); the optional `Position`
///   `IfcAxis2Placement2D` offsets/rotates it in the profile plane.
/// * `IfcCircleProfileDef(ProfileType, ProfileName, Position, Radius)` —
///   a circle of `Radius` (index 3) centred on its `Position` origin
///   (IFC4 EXPRESS `IfcCircleProfileDef`; the parameterised-profile
///   `Position` is inherited from `IfcParameterizedProfileDef`).
/// * `IfcEllipseProfileDef(ProfileType, ProfileName, Position,
///   SemiAxis1, SemiAxis2)` — an ellipse with semi-axis `SemiAxis1`
///   (index 3) along the profile X axis and `SemiAxis2` (index 4) along
///   Y (IFC4 EXPRESS `IfcEllipseProfileDef`).
///
/// Circular boundaries are approximated with [`CIRCLE_SEGMENTS`]
/// counter-clockwise segments. Any other profile keyword is
/// [`GeometryError::Unsupported`].
fn profile_ring(step: &StepFile, profile_id: u64) -> Result<Vec<[f64; 2]>, GeometryError> {
    let inst = step
        .get(profile_id)
        .ok_or(GeometryError::MissingInstance(profile_id))?;
    match inst.keyword.as_str() {
        "IFCARBITRARYCLOSEDPROFILEDEF" | "IFCARBITRARYPROFILEDEFWITHVOIDS" => {
            // OuterCurve : IfcCurve — attribute index 2. Inner voids (the
            // …WithVoids subtype) are not subtracted in this slice.
            let curve_id = inst
                .args
                .get(2)
                .and_then(Value::as_reference)
                .ok_or(GeometryError::BadProfile)?;
            let ring = curve_points_2d(step, curve_id)?;
            Ok(close_ring(ring))
        }
        "IFCRECTANGLEPROFILEDEF" => {
            // (ProfileType, ProfileName, Position, XDim, YDim).
            let xdim = inst
                .args
                .get(3)
                .and_then(Value::as_number)
                .ok_or(GeometryError::BadProfile)?;
            let ydim = inst
                .args
                .get(4)
                .and_then(Value::as_number)
                .ok_or(GeometryError::BadProfile)?;
            let (hx, hy) = (xdim / 2.0, ydim / 2.0);
            // Centred rectangle, counter-clockwise from the −x/−y corner.
            let ring = vec![[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]];
            positioned_profile_ring(step, inst.args.get(2), ring)
        }
        "IFCCIRCLEPROFILEDEF" => {
            // (ProfileType, ProfileName, Position, Radius).
            let radius = inst
                .args
                .get(3)
                .and_then(Value::as_number)
                .ok_or(GeometryError::BadProfile)?;
            if radius <= 0.0 {
                return Err(GeometryError::BadProfile);
            }
            positioned_profile_ring(step, inst.args.get(2), ellipse_ring(radius, radius))
        }
        "IFCELLIPSEPROFILEDEF" => {
            // (ProfileType, ProfileName, Position, SemiAxis1, SemiAxis2).
            let a = inst
                .args
                .get(3)
                .and_then(Value::as_number)
                .ok_or(GeometryError::BadProfile)?;
            let b = inst
                .args
                .get(4)
                .and_then(Value::as_number)
                .ok_or(GeometryError::BadProfile)?;
            if a <= 0.0 || b <= 0.0 {
                return Err(GeometryError::BadProfile);
            }
            positioned_profile_ring(step, inst.args.get(2), ellipse_ring(a, b))
        }
        other => Err(GeometryError::Unsupported(other.to_string())),
    }
}

/// Apply a parameterised profile's optional 2-D `Position` placement
/// (an `IfcAxis2Placement2D` reference, or `$`) to every ring point.
fn positioned_profile_ring(
    step: &StepFile,
    pos_arg: Option<&Value>,
    mut ring: Vec<[f64; 2]>,
) -> Result<Vec<[f64; 2]>, GeometryError> {
    if let Some(pos_id) = pos_arg.and_then(Value::as_reference) {
        let pl = axis2_placement_2d(step, pos_id)?;
        for p in &mut ring {
            *p = pl.apply(*p);
        }
    }
    Ok(ring)
}

/// A counter-clockwise ellipse ring centred on the origin with semi-axis
/// `a` along X and `b` along Y, approximated with [`CIRCLE_SEGMENTS`]
/// segments (no duplicated closing point). `a == b` gives a circle.
fn ellipse_ring(a: f64, b: f64) -> Vec<[f64; 2]> {
    let mut ring = Vec::with_capacity(CIRCLE_SEGMENTS);
    for i in 0..CIRCLE_SEGMENTS {
        let theta = 2.0 * core::f64::consts::PI * (i as f64) / (CIRCLE_SEGMENTS as f64);
        ring.push([a * theta.cos(), b * theta.sin()]);
    }
    ring
}

/// Resolve a bounded planar curve to a list of 2-D `[x, y]` points (a
/// third component, if present, is dropped — a closed profile curve is
/// planar).
///
/// Supported curve kinds:
/// * `IfcPolyline` (`Points : LIST OF IfcCartesianPoint`, attribute
///   index 0) — the points as authored.
/// * `IfcCircle` (`Position : IfcAxis2Placement` index 0, `Radius`
///   index 1) — a full circle in the profile plane, approximated with
///   [`CIRCLE_SEGMENTS`] segments about its 2-D `Position` (IFC4 EXPRESS
///   `IfcCircle` / `IfcConic.Position`).
///
/// Trimmed / composite / indexed curves are surfaced as `Unsupported`
/// with their keyword so callers can tell why.
fn curve_points_2d(step: &StepFile, id: u64) -> Result<Vec<[f64; 2]>, GeometryError> {
    let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
    match inst.keyword.as_str() {
        "IFCPOLYLINE" => {
            let points = inst
                .args
                .first()
                .and_then(Value::as_list)
                .ok_or(GeometryError::BadProfile)?;
            let mut out = Vec::with_capacity(points.len());
            for p in points {
                let pt = cartesian_point(step, Some(p))?;
                out.push([pt[0], pt[1]]);
            }
            Ok(out)
        }
        "IFCCIRCLE" => {
            // IfcConic(Position) + IfcCircle(Radius).
            let radius = inst
                .args
                .get(1)
                .and_then(Value::as_number)
                .ok_or(GeometryError::BadProfile)?;
            if radius <= 0.0 {
                return Err(GeometryError::BadProfile);
            }
            positioned_profile_ring(step, inst.args.first(), ellipse_ring(radius, radius))
        }
        other => Err(GeometryError::Unsupported(other.to_string())),
    }
}

/// Drop a duplicated closing vertex from a profile ring (an `IfcPolyline`
/// outer curve commonly repeats its first point as the last), so the
/// extrusion side-wall loop does not generate a degenerate quad.
fn close_ring(mut ring: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    if ring.len() >= 2 {
        let first = ring[0];
        let last = ring[ring.len() - 1];
        let eq = (first[0] - last[0]).abs() < 1e-12 && (first[1] - last[1]).abs() < 1e-12;
        if eq {
            ring.pop();
        }
    }
    ring
}

/// A 2-D affine placement (the profile-plane analogue of [`Transform`]):
/// an origin plus the orthonormal `x_axis` / `y_axis` of an
/// `IfcAxis2Placement2D`.
struct Placement2D {
    origin: [f64; 2],
    x_axis: [f64; 2],
    y_axis: [f64; 2],
}

impl Placement2D {
    /// Map a local 2-D point into the placement's parent plane.
    fn apply(&self, p: [f64; 2]) -> [f64; 2] {
        [
            self.origin[0] + self.x_axis[0] * p[0] + self.y_axis[0] * p[1],
            self.origin[1] + self.x_axis[1] * p[0] + self.y_axis[1] * p[1],
        ]
    }
}

/// Build the 2-D affine of an `IfcAxis2Placement2D(Location,
/// RefDirection)`. The X axis is `RefDirection` normalised (default world
/// +X); Y is the 90° counter-clockwise rotation of X (EXPRESS
/// `IfcBuild2Axes`).
fn axis2_placement_2d(step: &StepFile, id: u64) -> Result<Placement2D, GeometryError> {
    let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
    if inst.keyword != "IFCAXIS2PLACEMENT2D" {
        return Err(GeometryError::BadProfile);
    }
    let loc = cartesian_point(step, inst.args.first())?;
    let origin = [loc[0], loc[1]];
    // RefDirection (index 1) → X axis; default world +X.
    let x_axis = match direction(step, inst.args.get(1))? {
        Some(d) => {
            let mag = (d[0] * d[0] + d[1] * d[1]).sqrt();
            if mag > 0.0 {
                [d[0] / mag, d[1] / mag]
            } else {
                [1.0, 0.0]
            }
        }
        None => [1.0, 0.0],
    };
    // Y = 90° CCW rotation of X (EXPRESS IfcBuild2Axes: [-X.y, X.x]).
    let y_axis = [-x_axis[1], x_axis[0]];
    Ok(Placement2D {
        origin,
        x_axis,
        y_axis,
    })
}

/// Walk an `IfcConnectedFaceSet` (or its `IfcClosedShell` / `IfcOpenShell`
/// subtypes), triangulating each member `IfcFace` into `triangles`
/// against the shared `pool`. `CfsFaces` is attribute index 0.
fn connected_face_set(
    step: &StepFile,
    shell_id: u64,
    pool: &mut VertexPool,
    triangles: &mut Vec<[u32; 3]>,
) -> Result<(), GeometryError> {
    let inst = step
        .get(shell_id)
        .ok_or(GeometryError::MissingInstance(shell_id))?;
    match inst.keyword.as_str() {
        "IFCCLOSEDSHELL" | "IFCOPENSHELL" | "IFCCONNECTEDFACESET" => {}
        other => return Err(GeometryError::Unsupported(other.to_string())),
    }
    let faces = inst
        .args
        .first()
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinates)?;
    for face_ref in faces {
        let face_id = face_ref
            .as_reference()
            .ok_or(GeometryError::BadCoordinates)?;
        face(step, face_id, pool, triangles)?;
    }
    Ok(())
}

/// Triangulate one `IfcFace`: pick its outer bound (the
/// `IfcFaceOuterBound` if any, else the first `IfcFaceBound`), resolve
/// that bound's `IfcPolyLoop`, and fan-triangulate the loop polygon.
/// `Bounds` is attribute index 0; `IfcFaceBound.Bound` (the loop) is its
/// attribute index 0. Inner bounds (holes) are not subtracted in this
/// slice.
fn face(
    step: &StepFile,
    face_id: u64,
    pool: &mut VertexPool,
    triangles: &mut Vec<[u32; 3]>,
) -> Result<(), GeometryError> {
    let inst = step
        .get(face_id)
        .ok_or(GeometryError::MissingInstance(face_id))?;
    // IfcFace (and IfcFaceSurface subtype) carry Bounds at index 0;
    // IfcFaceSurface adds attributes *after* it, so the index is stable.
    if inst.keyword != "IFCFACE" && inst.keyword != "IFCFACESURFACE" {
        return Err(GeometryError::Unsupported(inst.keyword.clone()));
    }
    let bounds = inst
        .args
        .first()
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinates)?;
    if bounds.is_empty() {
        return Err(GeometryError::BadCoordinates);
    }

    // Prefer the IfcFaceOuterBound; fall back to the first bound. (A face
    // may carry just one untyped IfcFaceBound for its outer loop.)
    let mut chosen: Option<u64> = None;
    let mut first: Option<u64> = None;
    for b in bounds {
        let Some(bid) = b.as_reference() else {
            continue;
        };
        if first.is_none() {
            first = Some(bid);
        }
        let bk = step.get(bid).ok_or(GeometryError::MissingInstance(bid))?;
        if bk.keyword == "IFCFACEOUTERBOUND" {
            chosen = Some(bid);
            break;
        }
    }
    let bound_id = chosen.or(first).ok_or(GeometryError::BadCoordinates)?;
    let bound = step
        .get(bound_id)
        .ok_or(GeometryError::MissingInstance(bound_id))?;
    // IfcFaceBound(Bound : IfcLoop, Orientation : IfcBoolean): Bound is
    // attribute index 0. Orientation (index 1) is not applied here.
    let loop_id = bound
        .args
        .first()
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    poly_loop(step, loop_id, pool, triangles)
}

/// Fan-triangulate one `IfcPolyLoop` (`Polygon : LIST [3:?] OF
/// IfcCartesianPoint`, attribute index 0) into `triangles`, interning
/// each polygon vertex through the shared `pool`. Edge / vertex loops
/// (`IfcEdgeLoop` / `IfcVertexLoop`) are not polygonal and are surfaced
/// as `Unsupported`.
fn poly_loop(
    step: &StepFile,
    loop_id: u64,
    pool: &mut VertexPool,
    triangles: &mut Vec<[u32; 3]>,
) -> Result<(), GeometryError> {
    let inst = step
        .get(loop_id)
        .ok_or(GeometryError::MissingInstance(loop_id))?;
    if inst.keyword != "IFCPOLYLOOP" {
        return Err(GeometryError::Unsupported(inst.keyword.clone()));
    }
    let polygon = inst
        .args
        .first()
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinates)?;
    if polygon.len() < 3 {
        return Err(GeometryError::IndexOutOfRange);
    }
    // Intern each polygon point, then fan from the first vertex.
    let v0_id = polygon[0]
        .as_reference()
        .ok_or(GeometryError::BadCoordinates)?;
    let v0 = pool.intern(step, v0_id)?;
    for w in polygon[1..].windows(2) {
        let a_id = w[0].as_reference().ok_or(GeometryError::BadCoordinates)?;
        let b_id = w[1].as_reference().ok_or(GeometryError::BadCoordinates)?;
        let a = pool.intern(step, a_id)?;
        let b = pool.intern(step, b_id)?;
        triangles.push([v0, a, b]);
    }
    Ok(())
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
    fn shape_representation_skips_unsupported_items() {
        // A representation mixing a (still unsupported) surface-curve
        // swept solid with a triangulated body still yields the body mesh.
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);\n\
             #3=IFCSURFACECURVESWEPTAREASOLID(#9,#9,#9,#9,#9,#9);\n\
             #4=IFCSHAPEREPRESENTATION(#8,'Body','Tessellation',(#3,#2));",
        );
        let m = mesh_from_shape_representation(&f, 4).unwrap();
        assert_eq!(m.triangle_count(), 1);
    }

    #[test]
    fn all_unsupported_surfaces_keyword() {
        let f = parse(
            "#3=IFCSURFACECURVESWEPTAREASOLID(#9,#9,#9,#9,#9,#9);\n\
             #4=IFCSHAPEREPRESENTATION(#8,'Body','SweptSolid',(#3));",
        );
        let err = mesh_from_shape_representation(&f, 4).unwrap_err();
        assert_eq!(
            err,
            GeometryError::Unsupported("IFCSURFACECURVESWEPTAREASOLID".to_string())
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
    fn faceted_brep_tetra_dedups_shared_points() {
        // A 4-point tetrahedron as an IfcFacetedBrep: 4 triangular faces,
        // each an IfcFaceOuterBound over an IfcPolyLoop. The 4 points are
        // shared across faces and must pool to exactly 4 vertices.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((0.,1.,0.));\n\
             #4=IFCCARTESIANPOINT((0.,0.,1.));\n\
             #10=IFCPOLYLOOP((#1,#2,#3));\n\
             #11=IFCPOLYLOOP((#1,#2,#4));\n\
             #12=IFCPOLYLOOP((#1,#3,#4));\n\
             #13=IFCPOLYLOOP((#2,#3,#4));\n\
             #20=IFCFACEOUTERBOUND(#10,.T.);\n\
             #21=IFCFACEOUTERBOUND(#11,.T.);\n\
             #22=IFCFACEOUTERBOUND(#12,.T.);\n\
             #23=IFCFACEOUTERBOUND(#13,.T.);\n\
             #30=IFCFACE((#20));\n\
             #31=IFCFACE((#21));\n\
             #32=IFCFACE((#22));\n\
             #33=IFCFACE((#23));\n\
             #40=IFCCLOSEDSHELL((#30,#31,#32,#33));\n\
             #41=IFCFACETEDBREP(#40);",
        );
        let m = tessellate_item(&f, 41).unwrap();
        // 4 unique cartesian points → 4 pooled vertices (not 12).
        assert_eq!(m.vertex_count(), 4);
        // 4 triangular faces → 4 triangles.
        assert_eq!(m.triangle_count(), 4);
        // First face (#1,#2,#3) pools to vertices 0,1,2 in encounter order.
        assert_eq!(m.triangles[0], [0, 1, 2]);
        assert_eq!(m.positions[0], [0.0, 0.0, 0.0]);
        assert_eq!(m.positions[3], [0.0, 0.0, 1.0]);
    }

    #[test]
    fn faceted_brep_quad_face_fan_triangulates() {
        // A single 4-point face fan-triangulates into 2 triangles, and an
        // untyped IfcFaceBound (no IfcFaceOuterBound) is accepted as the
        // outer loop.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((1.,1.,0.));\n\
             #4=IFCCARTESIANPOINT((0.,1.,0.));\n\
             #10=IFCPOLYLOOP((#1,#2,#3,#4));\n\
             #20=IFCFACEBOUND(#10,.T.);\n\
             #30=IFCFACE((#20));\n\
             #40=IFCCLOSEDSHELL((#30));\n\
             #41=IFCFACETEDBREP(#40);",
        );
        let m = tessellate_item(&f, 41).unwrap();
        assert_eq!(m.vertex_count(), 4);
        assert_eq!(m.triangle_count(), 2);
        assert_eq!(m.triangles[0], [0, 1, 2]);
        assert_eq!(m.triangles[1], [0, 2, 3]);
    }

    #[test]
    fn faceted_brep_outer_bound_preferred_over_inner() {
        // A face listing an inner IfcFaceBound first then an
        // IfcFaceOuterBound must mesh the *outer* loop.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((4.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((0.,4.,0.));\n\
             #5=IFCCARTESIANPOINT((1.,1.,0.));\n\
             #6=IFCCARTESIANPOINT((2.,1.,0.));\n\
             #7=IFCCARTESIANPOINT((1.,2.,0.));\n\
             #10=IFCPOLYLOOP((#5,#6,#7));\n\
             #11=IFCPOLYLOOP((#1,#2,#3));\n\
             #20=IFCFACEBOUND(#10,.T.);\n\
             #21=IFCFACEOUTERBOUND(#11,.T.);\n\
             #30=IFCFACE((#20,#21));\n\
             #40=IFCCLOSEDSHELL((#30));\n\
             #41=IFCFACETEDBREP(#40);",
        );
        let m = tessellate_item(&f, 41).unwrap();
        // Only the outer loop (3 points) is meshed in this slice.
        assert_eq!(m.vertex_count(), 3);
        assert_eq!(m.triangle_count(), 1);
        assert_eq!(m.positions[0], [0.0, 0.0, 0.0]);
        assert_eq!(m.positions[1], [4.0, 0.0, 0.0]);
    }

    #[test]
    fn faceted_brep_with_voids_includes_inner_shell() {
        // IfcFacetedBrepWithVoids(Outer, Voids): both shells contribute.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((0.,1.,0.));\n\
             #4=IFCCARTESIANPOINT((0.,0.,1.));\n\
             #5=IFCCARTESIANPOINT((0.,0.,2.));\n\
             #10=IFCPOLYLOOP((#1,#2,#3));\n\
             #11=IFCPOLYLOOP((#1,#2,#4));\n\
             #20=IFCFACEOUTERBOUND(#10,.T.);\n\
             #21=IFCFACEOUTERBOUND(#11,.T.);\n\
             #30=IFCFACE((#20));\n\
             #31=IFCFACE((#21));\n\
             #40=IFCCLOSEDSHELL((#30));\n\
             #41=IFCCLOSEDSHELL((#31));\n\
             #42=IFCFACETEDBREPWITHVOIDS(#40,(#41));",
        );
        let m = tessellate_item(&f, 42).unwrap();
        // Outer (#1,#2,#3) + void (#1,#2,#4): 4 pooled points, 2 triangles.
        assert_eq!(m.vertex_count(), 4);
        assert_eq!(m.triangle_count(), 2);
    }

    #[test]
    fn face_based_surface_model_merges_face_sets() {
        // IfcFaceBasedSurfaceModel(FbsmFaces : SET OF IfcConnectedFaceSet).
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((0.,1.,0.));\n\
             #10=IFCPOLYLOOP((#1,#2,#3));\n\
             #20=IFCFACEOUTERBOUND(#10,.T.);\n\
             #30=IFCFACE((#20));\n\
             #40=IFCCONNECTEDFACESET((#30));\n\
             #41=IFCFACEBASEDSURFACEMODEL((#40));",
        );
        let m = tessellate_item(&f, 41).unwrap();
        assert_eq!(m.vertex_count(), 3);
        assert_eq!(m.triangle_count(), 1);
    }

    #[test]
    fn shell_based_surface_model_meshes_open_shell() {
        // IfcShellBasedSurfaceModel(SbsmBoundary : SET OF IfcShell);
        // the IfcShell SELECT resolves to an IfcOpenShell here.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((1.,1.,0.));\n\
             #4=IFCCARTESIANPOINT((0.,1.,0.));\n\
             #10=IFCPOLYLOOP((#1,#2,#3,#4));\n\
             #20=IFCFACEOUTERBOUND(#10,.T.);\n\
             #30=IFCFACE((#20));\n\
             #40=IFCOPENSHELL((#30));\n\
             #41=IFCSHELLBASEDSURFACEMODEL((#40));",
        );
        let m = tessellate_item(&f, 41).unwrap();
        assert_eq!(m.vertex_count(), 4);
        assert_eq!(m.triangle_count(), 2);
    }

    #[test]
    fn brep_degenerate_loop_is_error() {
        // A poly loop with fewer than 3 points is malformed.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #10=IFCPOLYLOOP((#1,#2));\n\
             #20=IFCFACEOUTERBOUND(#10,.T.);\n\
             #30=IFCFACE((#20));\n\
             #40=IFCCLOSEDSHELL((#30));\n\
             #41=IFCFACETEDBREP(#40);",
        );
        assert_eq!(
            tessellate_item(&f, 41).unwrap_err(),
            GeometryError::IndexOutOfRange
        );
    }

    #[test]
    fn brep_via_shape_representation() {
        // A Brep body reached through an IfcShapeRepresentation, the path
        // the registry decoder takes.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #3=IFCCARTESIANPOINT((0.,1.,0.));\n\
             #10=IFCPOLYLOOP((#1,#2,#3));\n\
             #20=IFCFACEOUTERBOUND(#10,.T.);\n\
             #30=IFCFACE((#20));\n\
             #40=IFCCLOSEDSHELL((#30));\n\
             #41=IFCFACETEDBREP(#40);\n\
             #50=IFCSHAPEREPRESENTATION(#8,'Body','Brep',(#41));",
        );
        let m = mesh_from_shape_representation(&f, 50).unwrap();
        assert_eq!(m.triangle_count(), 1);
    }

    #[test]
    fn extruded_rectangle_is_a_box() {
        // A 2×4 rectangle profile extruded +Z by 3 → a closed box.
        let f = parse(
            "#1=IFCRECTANGLEPROFILEDEF(.AREA.,$,$,2.,4.);\n\
             #2=IFCDIRECTION((0.,0.,1.));\n\
             #3=IFCEXTRUDEDAREASOLID(#1,$,#2,3.);",
        );
        let m = tessellate_item(&f, 3).unwrap();
        // 4 ring points → 8 vertices (bottom + top).
        assert_eq!(m.vertex_count(), 8);
        // 2 cap fans (2 tris each) + 4 side quads (2 tris each) = 12.
        assert_eq!(m.triangle_count(), 12);
        // Rectangle is centred: x in [-1,1], y in [-2,2].
        for p in &m.positions[..4] {
            assert!((p[0].abs() - 1.0).abs() < 1e-9);
            assert!((p[1].abs() - 2.0).abs() < 1e-9);
            approx(*p, [p[0], p[1], 0.0]);
        }
        // Top ring lifted to z = 3.
        for p in &m.positions[4..] {
            assert!((p[2] - 3.0).abs() < 1e-9);
        }
    }

    #[test]
    fn extruded_arbitrary_polyline_profile() {
        // A triangle profile (closing point repeated) extruded +Z by 2.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.));\n\
             #2=IFCCARTESIANPOINT((10.,0.));\n\
             #3=IFCCARTESIANPOINT((0.,10.));\n\
             #4=IFCPOLYLINE((#1,#2,#3,#1));\n\
             #5=IFCARBITRARYCLOSEDPROFILEDEF(.AREA.,$,#4);\n\
             #6=IFCDIRECTION((0.,0.,1.));\n\
             #7=IFCEXTRUDEDAREASOLID(#5,$,#6,2.);",
        );
        let m = tessellate_item(&f, 7).unwrap();
        // Duplicated closing point dropped → 3 ring points → 6 vertices.
        assert_eq!(m.vertex_count(), 6);
        // 2 cap fans (1 tri each) + 3 side quads (2 each) = 8 triangles.
        assert_eq!(m.triangle_count(), 8);
        approx(m.positions[0], [0.0, 0.0, 0.0]);
        approx(m.positions[3], [0.0, 0.0, 2.0]);
        approx(m.positions[4], [10.0, 0.0, 2.0]);
    }

    #[test]
    fn extruded_solid_position_replaces_result() {
        // A unit square extruded +Z by 1, then repositioned by a
        // Position whose Location is (100, 0, 0).
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.));\n\
             #2=IFCCARTESIANPOINT((1.,0.));\n\
             #3=IFCCARTESIANPOINT((1.,1.));\n\
             #4=IFCCARTESIANPOINT((0.,1.));\n\
             #5=IFCPOLYLINE((#1,#2,#3,#4,#1));\n\
             #6=IFCARBITRARYCLOSEDPROFILEDEF(.AREA.,$,#5);\n\
             #7=IFCDIRECTION((0.,0.,1.));\n\
             #8=IFCCARTESIANPOINT((100.,0.,0.));\n\
             #9=IFCAXIS2PLACEMENT3D(#8,$,$);\n\
             #10=IFCEXTRUDEDAREASOLID(#6,#9,#7,1.);",
        );
        let m = tessellate_item(&f, 10).unwrap();
        assert_eq!(m.vertex_count(), 8);
        // Every vertex shifted +100 in X by the Position.
        approx(m.positions[0], [100.0, 0.0, 0.0]);
        approx(m.positions[2], [101.0, 1.0, 0.0]);
        // Top ring: z = 1, still +100 in X.
        approx(m.positions[4], [100.0, 0.0, 1.0]);
    }

    #[test]
    fn extruded_oblique_direction_shears_top() {
        // A non-axis-aligned ExtrudedDirection shears the top cap.
        let f = parse(
            "#1=IFCRECTANGLEPROFILEDEF(.AREA.,$,$,2.,2.);\n\
             #2=IFCDIRECTION((1.,0.,1.));\n\
             #3=IFCEXTRUDEDAREASOLID(#1,$,#2,2.);",
        );
        let m = tessellate_item(&f, 3).unwrap();
        // Sweep = 2 * normalise([1,0,1]) = [√2, 0, √2].
        let s = 2f64.sqrt();
        // Bottom ring at z=0, top ring offset by [√2,0,√2].
        approx(m.positions[0], [-1.0, -1.0, 0.0]);
        approx(m.positions[4], [-1.0 + s, -1.0, s]);
    }

    #[test]
    fn extruded_rectangle_with_2d_position_offsets_ring() {
        // The rectangle's 2-D Position translates the centred ring.
        let f = parse(
            "#1=IFCCARTESIANPOINT((5.,7.));\n\
             #2=IFCAXIS2PLACEMENT2D(#1,$);\n\
             #3=IFCRECTANGLEPROFILEDEF(.AREA.,$,#2,2.,2.);\n\
             #4=IFCDIRECTION((0.,0.,1.));\n\
             #5=IFCEXTRUDEDAREASOLID(#3,$,#4,1.);",
        );
        let m = tessellate_item(&f, 5).unwrap();
        // Centred unit-half rectangle around (5,7): corner at (4,6).
        approx(m.positions[0], [4.0, 6.0, 0.0]);
        approx(m.positions[2], [6.0, 8.0, 0.0]);
    }

    #[test]
    fn extruded_unsupported_profile_surfaces_keyword() {
        // A trapezium profile is out of this slice → Unsupported(keyword).
        let f = parse(
            "#1=IFCTRAPEZIUMPROFILEDEF(.AREA.,$,$,3.,2.,1.,0.5);\n\
             #2=IFCDIRECTION((0.,0.,1.));\n\
             #3=IFCEXTRUDEDAREASOLID(#1,$,#2,1.);",
        );
        assert_eq!(
            tessellate_item(&f, 3).unwrap_err(),
            GeometryError::Unsupported("IFCTRAPEZIUMPROFILEDEF".to_string())
        );
    }

    #[test]
    fn extruded_circle_profile_is_a_cylinder() {
        // A circle profile of radius 3 extruded +Z by 5 → a closed
        // 48-segment cylinder.
        let f = parse(
            "#1=IFCCIRCLEPROFILEDEF(.AREA.,$,$,3.);\n\
             #2=IFCDIRECTION((0.,0.,1.));\n\
             #3=IFCEXTRUDEDAREASOLID(#1,$,#2,5.);",
        );
        let m = tessellate_item(&f, 3).unwrap();
        // 48 ring points → 96 vertices (bottom + top).
        assert_eq!(m.vertex_count(), 96);
        // 2 cap fans (46 tris each) + 48 side quads (2 each) = 188.
        assert_eq!(m.triangle_count(), 188);
        // The ring starts at theta = 0: (radius, 0).
        approx(m.positions[0], [3.0, 0.0, 0.0]);
        // Quarter turn (12 of 48 segments): (0, radius).
        approx(m.positions[12], [0.0, 3.0, 0.0]);
        // Every bottom vertex sits on the radius-3 circle at z = 0, every
        // top vertex at z = 5.
        for p in &m.positions[..48] {
            assert!(((p[0] * p[0] + p[1] * p[1]).sqrt() - 3.0).abs() < 1e-9);
            assert!(p[2].abs() < 1e-12);
        }
        for p in &m.positions[48..] {
            assert!((p[2] - 5.0).abs() < 1e-9);
        }
    }

    #[test]
    fn extruded_circle_profile_position_offsets_centre() {
        // The parameterised profile's 2-D Position shifts the circle
        // centre to (10, 20).
        let f = parse(
            "#1=IFCCARTESIANPOINT((10.,20.));\n\
             #2=IFCAXIS2PLACEMENT2D(#1,$);\n\
             #3=IFCCIRCLEPROFILEDEF(.AREA.,$,#2,2.);\n\
             #4=IFCDIRECTION((0.,0.,1.));\n\
             #5=IFCEXTRUDEDAREASOLID(#3,$,#4,1.);",
        );
        let m = tessellate_item(&f, 5).unwrap();
        approx(m.positions[0], [12.0, 20.0, 0.0]);
        for p in &m.positions[..48] {
            let (dx, dy) = (p[0] - 10.0, p[1] - 20.0);
            assert!(((dx * dx + dy * dy).sqrt() - 2.0).abs() < 1e-9);
        }
    }

    #[test]
    fn extruded_ellipse_profile_semi_axes() {
        // SemiAxis1 = 4 along X, SemiAxis2 = 1 along Y.
        let f = parse(
            "#1=IFCELLIPSEPROFILEDEF(.AREA.,$,$,4.,1.);\n\
             #2=IFCDIRECTION((0.,0.,1.));\n\
             #3=IFCEXTRUDEDAREASOLID(#1,$,#2,2.);",
        );
        let m = tessellate_item(&f, 3).unwrap();
        assert_eq!(m.vertex_count(), 96);
        // theta = 0 → (4, 0); quarter turn → (0, 1).
        approx(m.positions[0], [4.0, 0.0, 0.0]);
        approx(m.positions[12], [0.0, 1.0, 0.0]);
        // Every bottom vertex satisfies the ellipse equation.
        for p in &m.positions[..48] {
            let e = (p[0] / 4.0).powi(2) + p[1].powi(2);
            assert!((e - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn arbitrary_profile_with_circle_outer_curve() {
        // An IfcArbitraryClosedProfileDef whose OuterCurve is an
        // IfcCircle centred at (5, 0) with radius 1.
        let f = parse(
            "#1=IFCCARTESIANPOINT((5.,0.));\n\
             #2=IFCAXIS2PLACEMENT2D(#1,$);\n\
             #3=IFCCIRCLE(#2,1.);\n\
             #4=IFCARBITRARYCLOSEDPROFILEDEF(.AREA.,$,#3);\n\
             #5=IFCDIRECTION((0.,0.,1.));\n\
             #6=IFCEXTRUDEDAREASOLID(#4,$,#5,1.);",
        );
        let m = tessellate_item(&f, 6).unwrap();
        assert_eq!(m.vertex_count(), 96);
        approx(m.positions[0], [6.0, 0.0, 0.0]);
        for p in &m.positions[..48] {
            let (dx, dy) = (p[0] - 5.0, p[1]);
            assert!(((dx * dx + dy * dy).sqrt() - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn revolved_circle_profile_makes_torus() {
        // A radius-1 circle profile centred at x = 5, revolved a full
        // turn about the Y axis through the origin → a torus: every
        // vertex is 1 away from the circle of radius 5 about the Y axis.
        let f = parse(
            "#1=IFCCARTESIANPOINT((5.,0.));\n\
             #2=IFCAXIS2PLACEMENT2D(#1,$);\n\
             #3=IFCCIRCLEPROFILEDEF(.AREA.,$,#2,1.);\n\
             #4=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #5=IFCDIRECTION((0.,1.,0.));\n\
             #6=IFCAXIS1PLACEMENT(#4,#5);\n\
             #7=IFCREVOLVEDAREASOLID(#3,$,#6,6.283185307179586);",
        );
        let m = tessellate_item(&f, 7).unwrap();
        // 48 rings of 48 points, wrapped closed (no end caps).
        assert_eq!(m.vertex_count(), 48 * 48);
        assert_eq!(m.triangle_count(), 48 * 48 * 2);
        for p in &m.positions {
            // Distance from the Y axis in the XZ plane, paired with the
            // Y offset: (r - 5)² + y² = 1².
            let r = (p[0] * p[0] + p[2] * p[2]).sqrt();
            let d = ((r - 5.0).powi(2) + p[1] * p[1]).sqrt();
            assert!((d - 1.0).abs() < 1e-9, "vertex {p:?} off the torus");
        }
    }

    #[test]
    fn extruded_via_product_shape_walk() {
        // The full product → shape → extruded-solid path the registry
        // decoder takes for a swept-solid body.
        let f = parse(
            "#1=IFCRECTANGLEPROFILEDEF(.AREA.,$,$,2.,2.);\n\
             #2=IFCDIRECTION((0.,0.,1.));\n\
             #3=IFCEXTRUDEDAREASOLID(#1,$,#2,5.);\n\
             #4=IFCSHAPEREPRESENTATION(#8,'Body','SweptSolid',(#3));\n\
             #5=IFCPRODUCTDEFINITIONSHAPE($,$,(#4));",
        );
        let m = mesh_from_product_shape(&f, 5).unwrap();
        assert_eq!(m.vertex_count(), 8);
        assert_eq!(m.triangle_count(), 12);
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

    // --- IfcMappedItem ------------------------------------------------

    /// A source representation map (origin at the world origin) containing
    /// one triangulated body, instanced by an identity 3-D operator,
    /// reproduces the body unchanged.
    #[test]
    fn mapped_item_identity_operator_reproduces_source() {
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);\n\
             #3=IFCSHAPEREPRESENTATION(#8,'Body','Tessellation',(#2));\n\
             #10=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #11=IFCAXIS2PLACEMENT3D(#10,$,$);\n\
             #12=IFCREPRESENTATIONMAP(#11,#3);\n\
             #20=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #21=IFCCARTESIANTRANSFORMATIONOPERATOR3D($,$,#20,$,$);\n\
             #22=IFCMAPPEDITEM(#12,#21);",
        );
        let m = tessellate_item(&f, 22).unwrap();
        assert_eq!(m.vertex_count(), 3);
        assert_eq!(m.triangle_count(), 1);
        approx(m.positions[0], [0.0, 0.0, 0.0]);
        approx(m.positions[1], [1.0, 0.0, 0.0]);
        approx(m.positions[2], [0.0, 1.0, 0.0]);
    }

    /// The target operator's `LocalOrigin` translates the whole instance.
    #[test]
    fn mapped_item_operator_translation() {
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);\n\
             #3=IFCSHAPEREPRESENTATION(#8,'Body','Tessellation',(#2));\n\
             #11=IFCAXIS2PLACEMENT3D(#10,$,$);\n\
             #10=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #12=IFCREPRESENTATIONMAP(#11,#3);\n\
             #20=IFCCARTESIANPOINT((10.,20.,30.));\n\
             #21=IFCCARTESIANTRANSFORMATIONOPERATOR3D($,$,#20,$,$);\n\
             #22=IFCMAPPEDITEM(#12,#21);",
        );
        let m = tessellate_item(&f, 22).unwrap();
        approx(m.positions[0], [10.0, 20.0, 30.0]);
        approx(m.positions[1], [11.0, 20.0, 30.0]);
        approx(m.positions[2], [10.0, 21.0, 30.0]);
    }

    /// A uniform `Scale` on the operator scales the instanced geometry.
    #[test]
    fn mapped_item_uniform_scale() {
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(2.,0.,0.),(0.,2.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);\n\
             #3=IFCSHAPEREPRESENTATION(#8,'Body','Tessellation',(#2));\n\
             #10=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #11=IFCAXIS2PLACEMENT3D(#10,$,$);\n\
             #12=IFCREPRESENTATIONMAP(#11,#3);\n\
             #20=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #21=IFCCARTESIANTRANSFORMATIONOPERATOR3D($,$,#20,3.,$);\n\
             #22=IFCMAPPEDITEM(#12,#21);",
        );
        let m = tessellate_item(&f, 22).unwrap();
        // Each axis scaled by 3: (2,0,0) → (6,0,0); (0,2,0) → (0,6,0).
        approx(m.positions[1], [6.0, 0.0, 0.0]);
        approx(m.positions[2], [0.0, 6.0, 0.0]);
    }

    /// The non-uniform 3-D operator applies a distinct scale per axis.
    #[test]
    fn mapped_item_nonuniform_scale() {
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.),(0.,0.,1.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3),(1,2,4)),$);\n\
             #3=IFCSHAPEREPRESENTATION(#8,'Body','Tessellation',(#2));\n\
             #10=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #11=IFCAXIS2PLACEMENT3D(#10,$,$);\n\
             #12=IFCREPRESENTATIONMAP(#11,#3);\n\
             #20=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #21=IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM($,$,#20,2.,$,5.,7.);\n\
             #22=IFCMAPPEDITEM(#12,#21);",
        );
        let m = tessellate_item(&f, 22).unwrap();
        // Scale=2 on X, Scale2=5 on Y, Scale3=7 on Z.
        approx(m.positions[1], [2.0, 0.0, 0.0]);
        approx(m.positions[2], [0.0, 5.0, 0.0]);
        approx(m.positions[3], [0.0, 0.0, 7.0]);
    }

    /// An explicit `Axis1`/`Axis2` operator rotates the basis: Axis1=+Y,
    /// Axis2=−X gives a 90° rotation about Z (local +X → world +Y).
    #[test]
    fn mapped_item_rotated_axes() {
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);\n\
             #3=IFCSHAPEREPRESENTATION(#8,'Body','Tessellation',(#2));\n\
             #10=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #11=IFCAXIS2PLACEMENT3D(#10,$,$);\n\
             #12=IFCREPRESENTATIONMAP(#11,#3);\n\
             #20=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #30=IFCDIRECTION((0.,1.,0.));\n\
             #31=IFCDIRECTION((-1.,0.,0.));\n\
             #21=IFCCARTESIANTRANSFORMATIONOPERATOR3D(#30,#31,#20,$,$);\n\
             #22=IFCMAPPEDITEM(#12,#21);",
        );
        let m = tessellate_item(&f, 22).unwrap();
        // U1 = +Y, U2 = −X: local +X (point #2 = (1,0,0)) → world +Y.
        approx(m.positions[1], [0.0, 1.0, 0.0]);
        // local +Y (point #3 = (0,1,0)) → world −X.
        approx(m.positions[2], [-1.0, 0.0, 0.0]);
    }

    /// The `MappingOrigin` placement frame is folded in before the target
    /// operator: a non-identity origin translation shifts the source.
    #[test]
    fn mapped_item_mapping_origin_applied() {
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);\n\
             #3=IFCSHAPEREPRESENTATION(#8,'Body','Tessellation',(#2));\n\
             #10=IFCCARTESIANPOINT((100.,0.,0.));\n\
             #11=IFCAXIS2PLACEMENT3D(#10,$,$);\n\
             #12=IFCREPRESENTATIONMAP(#11,#3);\n\
             #20=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #21=IFCCARTESIANTRANSFORMATIONOPERATOR3D($,$,#20,$,$);\n\
             #22=IFCMAPPEDITEM(#12,#21);",
        );
        let m = tessellate_item(&f, 22).unwrap();
        // MappingOrigin at (100,0,0) translates the whole source.
        approx(m.positions[0], [100.0, 0.0, 0.0]);
        approx(m.positions[1], [101.0, 0.0, 0.0]);
    }

    /// Mapped items may nest: an outer map whose source representation
    /// contains another mapped item, each with its own operator
    /// translation, accumulates both offsets.
    #[test]
    fn mapped_item_nested_composes() {
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);\n\
             #3=IFCSHAPEREPRESENTATION(#8,'Body','Tessellation',(#2));\n\
             #10=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #11=IFCAXIS2PLACEMENT3D(#10,$,$);\n\
             #12=IFCREPRESENTATIONMAP(#11,#3);\n\
             #20=IFCCARTESIANPOINT((1.,0.,0.));\n\
             #21=IFCCARTESIANTRANSFORMATIONOPERATOR3D($,$,#20,$,$);\n\
             #22=IFCMAPPEDITEM(#12,#21);\n\
             #30=IFCSHAPEREPRESENTATION(#8,'Body','MappedRepresentation',(#22));\n\
             #31=IFCREPRESENTATIONMAP(#11,#30);\n\
             #40=IFCCARTESIANPOINT((0.,10.,0.));\n\
             #41=IFCCARTESIANTRANSFORMATIONOPERATOR3D($,$,#40,$,$);\n\
             #42=IFCMAPPEDITEM(#31,#41);",
        );
        let m = tessellate_item(&f, 42).unwrap();
        // Inner op +X then outer op +10Y: origin point (0,0,0) → (1,10,0).
        approx(m.positions[0], [1.0, 10.0, 0.0]);
    }

    /// A mapped item flows through the `IfcShapeRepresentation` walk
    /// alongside (skipped) unsupported items.
    #[test]
    fn mapped_item_via_shape_representation() {
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);\n\
             #3=IFCSHAPEREPRESENTATION(#8,'Body','Tessellation',(#2));\n\
             #10=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #11=IFCAXIS2PLACEMENT3D(#10,$,$);\n\
             #12=IFCREPRESENTATIONMAP(#11,#3);\n\
             #20=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #21=IFCCARTESIANTRANSFORMATIONOPERATOR3D($,$,#20,$,$);\n\
             #22=IFCMAPPEDITEM(#12,#21);\n\
             #50=IFCSHAPEREPRESENTATION(#8,'Body','MappedRepresentation',(#22));",
        );
        let m = mesh_from_shape_representation(&f, 50).unwrap();
        assert_eq!(m.triangle_count(), 1);
    }

    /// A self-referential mapped-item chain is bounded by the depth cap
    /// and surfaces `Unsupported` rather than recursing without end.
    #[test]
    fn mapped_item_self_reference_is_bounded() {
        let f = parse(
            "#10=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #11=IFCAXIS2PLACEMENT3D(#10,$,$);\n\
             #20=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #21=IFCCARTESIANTRANSFORMATIONOPERATOR3D($,$,#20,$,$);\n\
             #30=IFCSHAPEREPRESENTATION(#8,'Body','MappedRepresentation',(#42));\n\
             #31=IFCREPRESENTATIONMAP(#11,#30);\n\
             #42=IFCMAPPEDITEM(#31,#21);",
        );
        // #42 maps a representation (#30) whose only item is #42 itself.
        assert_eq!(
            tessellate_item(&f, 42).unwrap_err(),
            GeometryError::Unsupported("IFCMAPPEDITEM".to_string())
        );
    }

    /// The bare `transformation_operator` resolver: a 2-D operator with a
    /// scale yields a planar scaled basis with the `LocalOrigin`
    /// translation.
    #[test]
    fn transformation_operator_2d_scaled() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((4.,5.));\n\
             #2=IFCCARTESIANTRANSFORMATIONOPERATOR2D($,$,#1,2.);",
        );
        let t = transformation_operator(&f, 2).unwrap();
        approx(t.translation, [4.0, 5.0, 0.0]);
        approx(t.apply([1.0, 0.0, 0.0]), [6.0, 5.0, 0.0]);
        approx(t.apply([0.0, 1.0, 0.0]), [4.0, 7.0, 0.0]);
    }

    // --- IfcRevolvedAreaSolid -----------------------------------------

    /// `rotate_about_axis`: a 90° turn about the world Z axis through the
    /// origin maps +X → +Y, and is unaffected by Z.
    #[test]
    fn rotate_about_z_axis_quarter_turn() {
        let q = core::f64::consts::FRAC_PI_2;
        approx(
            rotate_about_axis([1.0, 0.0, 5.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], q),
            [0.0, 1.0, 5.0],
        );
        // Rotation about an offset axis line (through (2,0,0)).
        approx(
            rotate_about_axis([3.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 0.0, 1.0], q),
            [2.0, 1.0, 0.0],
        );
    }

    /// `IfcAxis1Placement` with an absent `Axis` defaults its direction to
    /// world +Z; an explicit direction is normalised.
    #[test]
    fn axis1_placement_defaults_and_explicit() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((1.,2.,0.));\n\
             #2=IFCAXIS1PLACEMENT(#1,$);\n\
             #3=IFCDIRECTION((0.,2.,0.));\n\
             #4=IFCAXIS1PLACEMENT(#1,#3);",
        );
        let (o, d) = axis1_placement(&f, 2).unwrap();
        approx(o, [1.0, 2.0, 0.0]);
        approx(d, [0.0, 0.0, 1.0]);
        let (_, d2) = axis1_placement(&f, 4).unwrap();
        approx(d2, [0.0, 1.0, 0.0]);
    }

    /// A full 2π revolution of a unit square (offset from the axis) about
    /// world Z wraps closed: `segments` rings, no end caps, side walls
    /// only. With 48 segments × 4 profile verts that is 192 verts and
    /// 48·4·2 = 384 triangles.
    #[test]
    fn revolved_full_turn_wraps_closed() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((2.,0.));\n\
             #2=IFCCARTESIANPOINT((3.,0.));\n\
             #3=IFCCARTESIANPOINT((3.,1.));\n\
             #4=IFCCARTESIANPOINT((2.,1.));\n\
             #5=IFCPOLYLINE((#1,#2,#3,#4,#1));\n\
             #6=IFCARBITRARYCLOSEDPROFILEDEF(.AREA.,$,#5);\n\
             #10=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #11=IFCDIRECTION((0.,0.,1.));\n\
             #12=IFCAXIS1PLACEMENT(#10,#11);\n\
             #20=IFCREVOLVEDAREASOLID(#6,$,#12,6.283185307179586);",
        );
        let m = tessellate_item(&f, 20).unwrap();
        assert_eq!(m.vertex_count(), 48 * 4);
        assert_eq!(m.triangle_count(), 48 * 4 * 2);
        // The profile lies in the XY-plane (z = 0) and is revolved about
        // the world Z axis, so every vertex stays at z = 0 and its
        // distance from the Z axis equals its in-plane radius — between
        // the profile's nearest corner (2,0)→r=2 and farthest (3,1)→
        // r=√10≈3.162.
        for p in &m.positions {
            let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
            assert!(r > 1.99 && r < 3.17, "radius {r}");
            assert!(p[2].abs() < 1e-9, "z {}", p[2]);
        }
    }

    /// A quarter turn (π/2) is a *partial* revolution: `segments + 1`
    /// rings plus two fan end caps. The first ring stays in the profile
    /// plane (y = 0); the last ring is rotated 90° (x → y).
    #[test]
    fn revolved_quarter_turn_has_end_caps() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((2.,0.));\n\
             #2=IFCCARTESIANPOINT((3.,0.));\n\
             #3=IFCCARTESIANPOINT((3.,1.));\n\
             #4=IFCCARTESIANPOINT((2.,1.));\n\
             #5=IFCPOLYLINE((#1,#2,#3,#4,#1));\n\
             #6=IFCARBITRARYCLOSEDPROFILEDEF(.AREA.,$,#5);\n\
             #10=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #11=IFCDIRECTION((0.,0.,1.));\n\
             #12=IFCAXIS1PLACEMENT(#10,#11);\n\
             #20=IFCREVOLVEDAREASOLID(#6,$,#12,1.5707963267948966);",
        );
        let m = tessellate_item(&f, 20).unwrap();
        // 48 * (π/2)/(2π) = 12 segments → 13 rings × 4 verts = 52.
        assert_eq!(m.vertex_count(), 52);
        // 12 segments side walls (12·4·2 = 96) + 2 end caps (2·2 = 4) = 100.
        assert_eq!(m.triangle_count(), 12 * 4 * 2 + 2 * 2);
        // First ring in the profile plane (y = 0).
        approx(m.positions[0], [2.0, 0.0, 0.0]);
        // Last ring rotated 90° about Z: (2,0,0) → (0,2,0).
        approx(m.positions[48], [0.0, 2.0, 0.0]);
    }

    /// A zero `Angle` is a degenerate revolution → `BadProfile`.
    #[test]
    fn revolved_zero_angle_rejected() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((2.,0.));\n\
             #2=IFCCARTESIANPOINT((3.,0.));\n\
             #3=IFCCARTESIANPOINT((3.,1.));\n\
             #5=IFCPOLYLINE((#1,#2,#3,#1));\n\
             #6=IFCARBITRARYCLOSEDPROFILEDEF(.AREA.,$,#5);\n\
             #10=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #12=IFCAXIS1PLACEMENT(#10,$);\n\
             #20=IFCREVOLVEDAREASOLID(#6,$,#12,0.);",
        );
        assert_eq!(
            tessellate_item(&f, 20).unwrap_err(),
            GeometryError::BadProfile
        );
    }

    /// The revolved solid flows through the product-shape walk the
    /// registry decoder takes.
    #[test]
    fn revolved_via_product_shape_walk() {
        let f = parse(
            "#1=IFCRECTANGLEPROFILEDEF(.AREA.,$,#30,2.,2.);\n\
             #30=IFCAXIS2PLACEMENT2D(#31,$);\n\
             #31=IFCCARTESIANPOINT((5.,0.));\n\
             #10=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #11=IFCDIRECTION((0.,0.,1.));\n\
             #12=IFCAXIS1PLACEMENT(#10,#11);\n\
             #20=IFCREVOLVEDAREASOLID(#1,$,#12,3.141592653589793);\n\
             #40=IFCSHAPEREPRESENTATION(#8,'Body','SweptSolid',(#20));\n\
             #41=IFCPRODUCTDEFINITIONSHAPE($,$,(#40));",
        );
        let m = mesh_from_product_shape(&f, 41).unwrap();
        // Half turn: 24 segments → 25 rings × 4 verts = 100.
        assert_eq!(m.vertex_count(), 100);
        assert!(m.triangle_count() > 0);
    }
}
