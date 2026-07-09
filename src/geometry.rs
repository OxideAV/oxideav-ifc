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
//! `Polygon : LIST [3:?] OF IfcCartesianPoint` is triangulated — convex
//! single-bound loops by a fan, concave loops and faces with inner
//! (hole) bounds by projecting onto the face plane (Newell normal) and
//! ear-clipping hole-aware, so face holes stay open. The shared vertex
//! table is de-duplicated by `IfcCartesianPoint` id so a point
//! referenced by several loops contributes one mesh vertex (§8.8.3.18:
//! "each Cartesian point shall be referenced by at least three poly
//! loops"). Per-bound `Orientation` flags are not applied — loops are
//! meshed as authored.
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
//! Swept-solid profiles are resolved as full areas ([`ProfileArea`]):
//! arbitrary closed profiles (polyline or full-circle outer curves,
//! with or without voids), rectangles (plain and hollow), circles
//! (plain and hollow), and ellipses. Caps are triangulated hole-aware
//! (hole bridging + ear clipping), so concave and holed profiles mesh
//! correctly; hole side walls are emitted for hollow / voided profiles.
//!
//! Boolean results (`IfcBooleanResult` / `IfcBooleanClippingResult`)
//! compose at the surface-mesh level: UNION merges the operand
//! boundaries; DIFFERENCE emits the first operand's boundary as
//! authored (half-space carving is pending the
//! `IfcHalfSpaceSolid.AgreementFlag` side-convention documentation);
//! INTERSECTION is surfaced as `Unsupported`.
//!
//! Still later Phase-3 work (reported as [`GeometryError::Unsupported`]
//! rather than silently dropped): the other swept solids
//! (`IfcSurfaceCurveSweptAreaSolid`, the tapered subtypes), trimmed /
//! composite curves and `IfcArcIndex` poly-curve segments, the named
//! parameterised profiles (I/L/T/U/Z/C shapes), advanced/curved breps
//! (`IfcAdvancedBrep`, `IfcFaceSurface`), boolean intersection, and
//! actual boolean subtraction.

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
        // Boolean composition of two solid operands.
        "IFCBOOLEANRESULT" | "IFCBOOLEANCLIPPINGRESULT" => boolean_result(step, &inst.args, depth),
        other => Err(GeometryError::Unsupported(other.to_string())),
    }
}

// =====================================================================
// IfcBooleanResult (Operator, FirstOperand, SecondOperand)
//
// The result of a regularised Boolean operation on two solid operands
// (ISO 16739 §8.8.3.5): UNION is the set of points in either operand,
// INTERSECTION the points in both, DIFFERENCE the points in the first
// but not the second. IfcBooleanClippingResult restricts the operator
// to DIFFERENCE with a half-space second operand (the common
// "wall clipped by a plane" case).
//
// This slice composes the operand *surface meshes*:
// * UNION — the merged operand boundaries (a boundary superset of the
//   regularised union; overlapping interior surface is kept, not
//   dissolved).
// * DIFFERENCE — the first operand's boundary as authored. The
//   subtracted volume is NOT yet carved out: half-space clipping needs
//   the IfcHalfSpaceSolid.AgreementFlag side convention, whose
//   normative description is not in the staged documentation set. The
//   un-clipped body is emitted (visible rather than dropped, matching
//   the Brep `Voids` policy) until that lands.
// * INTERSECTION — no boundary-level approximation is defensible;
//   surfaced as Unsupported.
//
// Operands may be any meshable solid, including nested boolean results
// (clipping chains); recursion shares the mapped-item depth cap.
// =====================================================================
fn boolean_result(step: &StepFile, args: &[Value], depth: usize) -> Result<TriMesh, GeometryError> {
    if depth >= MAX_MAP_DEPTH {
        // A cyclic operand chain (malformed file): stop rather than
        // recurse without end.
        return Err(GeometryError::Unsupported("IFCBOOLEANRESULT".to_string()));
    }
    // Operator : IfcBooleanOperator (index 0) — .UNION. / .INTERSECTION.
    // / .DIFFERENCE.
    let op = args
        .first()
        .and_then(Value::as_enum)
        .ok_or(GeometryError::BadCoordinates)?;
    let first = args
        .get(1)
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    let second = args
        .get(2)
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadCoordinates)?;
    match op {
        "UNION" => {
            // Merge both operand boundaries; tolerate one unsupported
            // operand as long as the other produced geometry.
            let a = tessellate_item_depth(step, first, depth + 1);
            let b = tessellate_item_depth(step, second, depth + 1);
            match (a, b) {
                (Ok(mut m), Ok(other)) => {
                    append_mesh(&mut m, other);
                    Ok(m)
                }
                (Ok(m), Err(GeometryError::Unsupported(_)))
                | (Err(GeometryError::Unsupported(_)), Ok(m)) => Ok(m),
                (Err(e), _) | (_, Err(e)) => Err(e),
            }
        }
        "DIFFERENCE" => tessellate_item_depth(step, first, depth + 1),
        "INTERSECTION" => Err(GeometryError::Unsupported(
            "IFCBOOLEANRESULT(.INTERSECTION.)".to_string(),
        )),
        _ => Err(GeometryError::BadCoordinates),
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

/// Tessellate every supported representation item of a product shape
/// **individually**, returning one `(item id, mesh)` pair per item that
/// produced geometry.
///
/// Same walk as [`mesh_from_product_shape`], but the per-item
/// granularity is preserved so callers can attach per-item presentation
/// (surface styles, colour maps) to each mesh. Items with unsupported
/// geometry styles are skipped; if **no** item produced geometry the
/// first unsupported keyword is surfaced.
pub fn meshed_items_from_product_shape(
    step: &StepFile,
    product_def_shape_id: u64,
) -> Result<Vec<(u64, TriMesh)>, GeometryError> {
    let inst = step
        .get(product_def_shape_id)
        .ok_or(GeometryError::MissingInstance(product_def_shape_id))?;
    // IfcProductDefinitionShape.Representations (index 2).
    let reps = inst
        .args
        .get(2)
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadCoordinates)?;

    let mut out: Vec<(u64, TriMesh)> = Vec::new();
    let mut first_unsupported: Option<GeometryError> = None;
    for rep in reps {
        let Some(rep_id) = rep.as_reference() else {
            continue;
        };
        let Some(rep_inst) = step.get(rep_id) else {
            return Err(GeometryError::MissingInstance(rep_id));
        };
        // IfcShapeRepresentation.Items (index 3); a representation with
        // no usable Items list is tolerated (axis / footprint styles).
        let Some(items) = rep_inst.args.get(3).and_then(Value::as_list) else {
            continue;
        };
        for item in items {
            let Some(item_id) = item.as_reference() else {
                continue;
            };
            match tessellate_item(step, item_id) {
                Ok(mesh) if !mesh.is_empty() => out.push((item_id, mesh)),
                Ok(_) => {}
                Err(e @ GeometryError::Unsupported(_)) => {
                    if first_unsupported.is_none() {
                        first_unsupported = Some(e);
                    }
                }
                Err(other) => return Err(other),
            }
        }
    }
    if out.is_empty() {
        Err(first_unsupported.unwrap_or(GeometryError::BadCoordinates))
    } else {
        Ok(out)
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
// The mesh built here is a closed prism over the profile area: a bottom
// cap, a top cap offset by `Depth · ExtrudedDirection`, and a quad side
// wall per boundary edge (outer ring and hole rings alike). The caps are
// ear-clipped through `triangulate_profile`, so concave outer boundaries
// and profile holes (hollow / voided profile kinds) triangulate
// correctly. The tapered subtype is not yet applied.
// ---------------------------------------------------------------------
fn extruded_area_solid(step: &StepFile, args: &[Value]) -> Result<TriMesh, GeometryError> {
    // SweptArea (profile) — attribute index 0.
    let profile_id = args
        .first()
        .and_then(Value::as_reference)
        .ok_or(GeometryError::BadProfile)?;
    let areas = profile_areas(step, profile_id)?;

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

    // One closed prism per profile area (a composite profile is the
    // union of its component areas), merged into a single mesh.
    let mut mesh = TriMesh::default();
    for area in &areas {
        append_mesh(&mut mesh, extrude_area(area, sweep)?);
    }

    // Position: OPTIONAL IfcAxis2Placement3D (index 1). When present it
    // re-places the whole swept solid; absent → local identity.
    if let Some(pos_id) = args.get(1).and_then(Value::as_reference) {
        let xform = axis2_placement_3d(step, pos_id)?;
        mesh.transform(&xform);
    }
    Ok(mesh)
}

/// Build the closed prism of one [`ProfileArea`] swept by `sweep`.
fn extrude_area(area: &ProfileArea, sweep: [f64; 3]) -> Result<TriMesh, GeometryError> {
    let total = area.point_count();
    let mut positions: Vec<[f64; 3]> = Vec::with_capacity(total * 2);
    // Bottom rings (profile plane, local z = 0): outer, then each hole.
    for ring in area.rings() {
        for &[x, y] in ring {
            positions.push([x, y, 0.0]);
        }
    }
    // Top rings (bottom + sweep), same order.
    for ring in area.rings() {
        for &[x, y] in ring {
            positions.push([x + sweep[0], y + sweep[1], sweep[2]]);
        }
    }

    // Caps: the hole-aware profile triangulation (indices address the
    // concatenated rings in the same order as `positions`).
    let cap = triangulate_profile(area)?;
    let top = total as u32;
    let mut triangles: Vec<[u32; 3]> = Vec::with_capacity(cap.len() * 2 + total * 2);
    // Bottom cap wound to face away from the sweep (reversed), top cap
    // in profile (CCW) winding.
    for &[a, b, c] in &cap {
        triangles.push([a, c, b]);
    }
    for &[a, b, c] in &cap {
        triangles.push([top + a, top + b, top + c]);
    }
    // Side walls: one quad (two triangles) per ring edge. Hole walls are
    // wound opposite the outer wall so their faces look into the hole.
    let mut offset = 0u32;
    for (ri, ring) in area.rings().enumerate() {
        let k = ring.len();
        for i in 0..k {
            let b0 = offset + i as u32;
            let b1 = offset + ((i + 1) % k) as u32;
            let t0 = top + b0;
            let t1 = top + b1;
            if ri == 0 {
                triangles.push([b0, b1, t1]);
                triangles.push([b0, t1, t0]);
            } else {
                triangles.push([b1, b0, t0]);
                triangles.push([b1, t0, t1]);
            }
        }
        offset += k as u32;
    }

    Ok(TriMesh {
        positions,
        triangles,
    })
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
    let areas = profile_areas(step, profile_id)?;

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

    // One surface of revolution per profile area (a composite profile is
    // the union of its component areas), merged into a single mesh.
    let mut mesh = TriMesh::default();
    for area in &areas {
        append_mesh(&mut mesh, revolve_area(area, axis_origin, axis_dir, angle)?);
    }

    // Position: OPTIONAL IfcAxis2Placement3D (index 1) re-places the solid.
    if let Some(pos_id) = args.get(1).and_then(Value::as_reference) {
        let xform = axis2_placement_3d(step, pos_id)?;
        mesh.transform(&xform);
    }
    Ok(mesh)
}

/// Build the tessellated surface of revolution of one [`ProfileArea`]
/// about the axis line through `axis_origin` with direction `axis_dir`,
/// swept by `angle` radians.
fn revolve_area(
    area: &ProfileArea,
    axis_origin: [f64; 3],
    axis_dir: [f64; 3],
    angle: f64,
) -> Result<TriMesh, GeometryError> {
    // Segment count proportional to the swept angle (≥1).
    let frac = (angle.abs() / (2.0 * core::f64::consts::PI)).min(1.0);
    let segments = ((REVOLVE_SEGMENTS_PER_TURN as f64 * frac).ceil() as usize).max(1);
    let two_pi = 2.0 * core::f64::consts::PI;
    // A full turn (within tolerance) wraps closed: no end caps, and the
    // last ring coincides with the first.
    let full_turn = (angle.abs() - two_pi).abs() < 1e-9 || angle.abs() > two_pi;
    let ring_count = if full_turn { segments } else { segments + 1 };

    // One "slice" per angular step: the concatenated profile rings
    // (outer, then each hole) rotated into position.
    let n = area.point_count();
    let mut positions: Vec<[f64; 3]> = Vec::with_capacity(n * ring_count);
    for s in 0..ring_count {
        // Angular position of this slice (the last slice of a full turn
        // is not emitted separately — it reuses slice 0).
        let theta = angle * (s as f64) / (segments as f64);
        for ring in area.rings() {
            for &[x, y] in ring {
                let p = [x, y, 0.0];
                positions.push(rotate_about_axis(p, axis_origin, axis_dir, theta));
            }
        }
    }

    let mut triangles: Vec<[u32; 3]> = Vec::new();
    // Side walls: stitch each adjacent pair of slices, ring by ring.
    // Hole-ring walls are wound opposite the outer wall so their faces
    // look into the revolved hole channel.
    for s in 0..segments {
        let a = ((s % ring_count) * n) as u32;
        let b = (((s + 1) % ring_count) * n) as u32;
        let mut offset = 0u32;
        for (ri, ring) in area.rings().enumerate() {
            let k = ring.len();
            for i in 0..k {
                let i_next = offset + ((i + 1) % k) as u32;
                let i = offset + i as u32;
                if ri == 0 {
                    // Quad (a+i, a+i_next, b+i_next, b+i).
                    triangles.push([a + i, a + i_next, b + i_next]);
                    triangles.push([a + i, b + i_next, b + i]);
                } else {
                    triangles.push([a + i_next, a + i, b + i]);
                    triangles.push([a + i_next, b + i, b + i_next]);
                }
            }
            offset += k as u32;
        }
    }
    // End caps for a partial revolution: the hole-aware profile
    // triangulation applied to the first and last slices (the open ends
    // of the swept volume).
    if !full_turn {
        let cap = triangulate_profile(area)?;
        let last = (segments * n) as u32;
        for &[a, b, c] in &cap {
            triangles.push([a, c, b]);
            triangles.push([last + a, last + b, last + c]);
        }
    }

    Ok(TriMesh {
        positions,
        triangles,
    })
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
            // OuterCurve : IfcCurve — attribute index 2. (The …WithVoids
            // inner curves are handled by [`profile_area`].)
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
        "IFCINDEXEDPOLYCURVE" => {
            // IfcIndexedPolyCurve(Points : IfcCartesianPointList,
            // Segments : OPTIONAL LIST OF IfcSegmentIndexSelect,
            // SelfIntersect). With `$` Segments the curve is the point
            // list in order (IFC4 EXPRESS `IfcIndexedPolyCurve`); with
            // Segments each `IfcLineIndex` contributes its indexed
            // points in sequence (the EXPRESS `Consecutive` WHERE rule
            // makes adjacent segments share their junction point, which
            // is emitted once). `IfcArcIndex` segments are curved and
            // remain `Unsupported` in this slice.
            let points_id = inst
                .args
                .first()
                .and_then(Value::as_reference)
                .ok_or(GeometryError::BadProfile)?;
            let pts = cartesian_point_list_2d(step, points_id)?;
            match inst.args.get(1) {
                None | Some(Value::Unset) => Ok(pts),
                Some(Value::List(segments)) => {
                    let mut out: Vec<[f64; 2]> = Vec::new();
                    let mut last_idx: Option<usize> = None;
                    for seg in segments {
                        let (kw, sargs) = seg.as_typed().ok_or(GeometryError::BadProfile)?;
                        if kw != "IFCLINEINDEX" {
                            // IFCARCINDEX (three-point arc) or another
                            // SELECT member: not evaluated here.
                            return Err(GeometryError::Unsupported(kw.to_string()));
                        }
                        let idxs = sargs
                            .first()
                            .and_then(Value::as_list)
                            .ok_or(GeometryError::BadProfile)?;
                        for v in idxs {
                            let raw = v.as_integer().ok_or(GeometryError::IndexOutOfRange)?;
                            if raw < 1 || raw as usize > pts.len() {
                                return Err(GeometryError::IndexOutOfRange);
                            }
                            let idx = raw as usize;
                            // Consecutive segments share their junction
                            // point — skip the repeat.
                            if last_idx == Some(idx) {
                                continue;
                            }
                            out.push(pts[idx - 1]);
                            last_idx = Some(idx);
                        }
                    }
                    Ok(out)
                }
                Some(_) => Err(GeometryError::BadProfile),
            }
        }
        other => Err(GeometryError::Unsupported(other.to_string())),
    }
}

/// Resolve an `IfcCartesianPointList2D` (`CoordList : LIST OF LIST
/// [2:2] OF IfcLengthMeasure`) — or, leniently, a 3-D point list whose
/// Z is dropped — to 2-D points.
fn cartesian_point_list_2d(step: &StepFile, id: u64) -> Result<Vec<[f64; 2]>, GeometryError> {
    let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
    if inst.keyword != "IFCCARTESIANPOINTLIST2D" && inst.keyword != "IFCCARTESIANPOINTLIST3D" {
        return Err(GeometryError::BadProfile);
    }
    let rows = inst
        .args
        .first()
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadProfile)?;
    let mut out = Vec::with_capacity(rows.len());
    for row in rows {
        let comps = row.as_list().ok_or(GeometryError::BadCoordinate)?;
        if comps.len() < 2 || comps.len() > 3 {
            return Err(GeometryError::BadCoordinate);
        }
        let x = comps[0].as_number().ok_or(GeometryError::BadCoordinate)?;
        let y = comps[1].as_number().ok_or(GeometryError::BadCoordinate)?;
        out.push([x, y]);
    }
    Ok(out)
}

/// A 2-D profile area: one outer boundary ring plus zero or more hole
/// rings, all counter-clockwise, none with a duplicated closing vertex.
///
/// The mesh vertex order of a swept solid concatenates the rings
/// (outer first, then each hole), so a ring point's index is its ring
/// offset plus its position within the ring.
struct ProfileArea {
    outer: Vec<[f64; 2]>,
    holes: Vec<Vec<[f64; 2]>>,
}

impl ProfileArea {
    /// Total number of boundary points across all rings.
    fn point_count(&self) -> usize {
        self.outer.len() + self.holes.iter().map(Vec::len).sum::<usize>()
    }

    /// The rings in mesh-vertex order: outer first, then each hole.
    fn rings(&self) -> impl Iterator<Item = &Vec<[f64; 2]>> {
        core::iter::once(&self.outer).chain(self.holes.iter())
    }
}

/// Resolve an `IfcProfileDef` to its full [`ProfileArea`] — the outer
/// ring plus any inner (hole) rings.
///
/// Beyond the single-ring kinds of [`profile_ring`] this resolves:
/// * `IfcArbitraryProfileDefWithVoids(…, OuterCurve, InnerCurves)` — the
///   `InnerCurves : SET [1:?] OF IfcCurve` (attribute index 3) become
///   hole rings (IFC4 EXPRESS `IfcArbitraryProfileDefWithVoids`).
/// * `IfcCircleHollowProfileDef(…, Position, Radius, WallThickness)` —
///   an annulus: outer circle of `Radius` (index 3), inner circle of
///   `Radius − WallThickness` (index 4); the EXPRESS WR1 requires
///   `WallThickness < Radius`.
/// * `IfcRectangleHollowProfileDef(…, Position, XDim, YDim,
///   WallThickness, InnerFilletRadius, OuterFilletRadius)` — a
///   rectangular tube: the inner rectangle is `XDim − 2·WallThickness` ×
///   `YDim − 2·WallThickness` (`WallThickness` at index 5); the optional
///   fillet radii (indices 6/7) are not applied in this slice — corners
///   stay square.
///
/// Ring orientation is normalised: every returned ring is
/// counter-clockwise (holes are re-oriented during cap triangulation).
fn profile_area(step: &StepFile, profile_id: u64) -> Result<ProfileArea, GeometryError> {
    let inst = step
        .get(profile_id)
        .ok_or(GeometryError::MissingInstance(profile_id))?;
    let mut area = match inst.keyword.as_str() {
        "IFCARBITRARYPROFILEDEFWITHVOIDS" => {
            let outer = profile_ring(step, profile_id)?;
            // InnerCurves : SET [1:?] OF IfcCurve (attribute index 3).
            let inner = inst
                .args
                .get(3)
                .and_then(Value::as_list)
                .ok_or(GeometryError::BadProfile)?;
            let mut holes = Vec::with_capacity(inner.len());
            for c in inner {
                let cid = c.as_reference().ok_or(GeometryError::BadProfile)?;
                let ring = close_ring(curve_points_2d(step, cid)?);
                if ring.len() < 3 {
                    return Err(GeometryError::BadProfile);
                }
                holes.push(ring);
            }
            ProfileArea { outer, holes }
        }
        "IFCCIRCLEHOLLOWPROFILEDEF" => {
            // (ProfileType, ProfileName, Position, Radius, WallThickness).
            let radius = inst
                .args
                .get(3)
                .and_then(Value::as_number)
                .ok_or(GeometryError::BadProfile)?;
            let wall = inst
                .args
                .get(4)
                .and_then(Value::as_number)
                .ok_or(GeometryError::BadProfile)?;
            if radius <= 0.0 || wall <= 0.0 || wall >= radius {
                return Err(GeometryError::BadProfile);
            }
            let pos = inst.args.get(2);
            let outer = positioned_profile_ring(step, pos, ellipse_ring(radius, radius))?;
            let r_in = radius - wall;
            let hole = positioned_profile_ring(step, pos, ellipse_ring(r_in, r_in))?;
            ProfileArea {
                outer,
                holes: vec![hole],
            }
        }
        "IFCRECTANGLEHOLLOWPROFILEDEF" => {
            // (ProfileType, ProfileName, Position, XDim, YDim,
            //  WallThickness, InnerFilletRadius, OuterFilletRadius).
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
            let wall = inst
                .args
                .get(5)
                .and_then(Value::as_number)
                .ok_or(GeometryError::BadProfile)?;
            // EXPRESS ValidWallThickness: WallThickness < XDim/2 ∧ < YDim/2.
            if wall <= 0.0 || wall >= xdim / 2.0 || wall >= ydim / 2.0 {
                return Err(GeometryError::BadProfile);
            }
            let rect = |hx: f64, hy: f64| vec![[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]];
            let pos = inst.args.get(2);
            let outer = positioned_profile_ring(step, pos, rect(xdim / 2.0, ydim / 2.0))?;
            let hole =
                positioned_profile_ring(step, pos, rect(xdim / 2.0 - wall, ydim / 2.0 - wall))?;
            ProfileArea {
                outer,
                holes: vec![hole],
            }
        }
        // Every single-ring profile kind (and the unsupported-keyword
        // error path) comes from profile_ring.
        _ => ProfileArea {
            outer: profile_ring(step, profile_id)?,
            holes: Vec::new(),
        },
    };
    if area.outer.len() < 3 {
        return Err(GeometryError::BadProfile);
    }
    // Normalise every ring counter-clockwise so wall winding and cap
    // triangulation see a consistent orientation regardless of how the
    // file authored its curves.
    make_ccw(&mut area.outer);
    for h in &mut area.holes {
        make_ccw(h);
    }
    Ok(area)
}

/// Resolve an `IfcProfileDef` to one or more [`ProfileArea`]s.
///
/// `IfcCompositeProfileDef(ProfileType, ProfileName, Profiles, Label)`
/// is the union of its component profiles (`Profiles : SET [2:?] OF
/// IfcProfileDef`, attribute index 2; the EXPRESS `NoRecursion` WHERE
/// rule forbids nested composites) — each component becomes its own
/// area, swept independently and merged. Every other profile kind
/// resolves to a single [`profile_area`].
fn profile_areas(step: &StepFile, profile_id: u64) -> Result<Vec<ProfileArea>, GeometryError> {
    let inst = step
        .get(profile_id)
        .ok_or(GeometryError::MissingInstance(profile_id))?;
    if inst.keyword != "IFCCOMPOSITEPROFILEDEF" {
        return Ok(vec![profile_area(step, profile_id)?]);
    }
    let profiles = inst
        .args
        .get(2)
        .and_then(Value::as_list)
        .ok_or(GeometryError::BadProfile)?;
    if profiles.len() < 2 {
        return Err(GeometryError::BadProfile);
    }
    let mut areas = Vec::with_capacity(profiles.len());
    for p in profiles {
        let pid = p.as_reference().ok_or(GeometryError::BadProfile)?;
        // A nested composite falls through profile_area → profile_ring,
        // which surfaces it as Unsupported (the NoRecursion WHERE rule
        // forbids it anyway).
        areas.push(profile_area(step, pid)?);
    }
    Ok(areas)
}

/// Twice the signed area of a 2-D ring (positive when
/// counter-clockwise) — the shoelace sum.
fn signed_area_2x(ring: &[[f64; 2]]) -> f64 {
    let mut sum = 0.0;
    for i in 0..ring.len() {
        let a = ring[i];
        let b = ring[(i + 1) % ring.len()];
        sum += a[0] * b[1] - b[0] * a[1];
    }
    sum
}

/// Reverse a ring in place if it is clockwise.
fn make_ccw(ring: &mut [[f64; 2]]) {
    if signed_area_2x(ring) < 0.0 {
        ring.reverse();
    }
}

/// 2-D cross product of `(b − a)` × `(c − a)` — positive when `c` lies
/// to the left of the directed line `a → b`.
fn cross2(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// Triangulate a [`ProfileArea`] cap — an outer polygon with holes — by
/// bridging each hole into the outer boundary and ear-clipping the
/// resulting simple polygon. Returned triangle indices address the
/// concatenated ring points (outer first, then each hole, matching
/// [`ProfileArea::rings`]) and wind counter-clockwise in the profile
/// plane.
///
/// This is plain computational geometry, not IFC semantics: the EXPRESS
/// schema defines the profile *area*; how the planar caps are decomposed
/// into triangles is an extraction choice. Ear clipping handles concave
/// outer boundaries (which the previous cap fan did not).
fn triangulate_profile(area: &ProfileArea) -> Result<Vec<[u32; 3]>, GeometryError> {
    // Working polygon: (original concatenated index, position). Outer is
    // CCW; holes are walked clockwise when merged so the bridged polygon
    // stays consistently oriented.
    let mut poly: Vec<(u32, [f64; 2])> = area
        .outer
        .iter()
        .enumerate()
        .map(|(i, &p)| (i as u32, p))
        .collect();

    // All rings' edges (for bridge-visibility tests) as point pairs.
    let mut all_edges: Vec<([f64; 2], [f64; 2])> = Vec::new();
    for ring in area.rings() {
        for i in 0..ring.len() {
            all_edges.push((ring[i], ring[(i + 1) % ring.len()]));
        }
    }

    let mut offset = area.outer.len() as u32;
    for hole in &area.holes {
        // Hole vertices clockwise (the ring is stored CCW), tagged with
        // their concatenated-index positions.
        let hverts: Vec<(u32, [f64; 2])> = hole
            .iter()
            .enumerate()
            .rev()
            .map(|(i, &p)| (offset + i as u32, p))
            .collect();
        merge_hole(&mut poly, &hverts, &all_edges)?;
        offset += hole.len() as u32;
    }

    ear_clip(poly)
}

/// Splice one hole (given clockwise) into the outer polygon through a
/// mutually visible vertex pair, duplicating the two bridge vertices.
///
/// Visibility is brute force: candidate (outer, hole) vertex pairs are
/// tried nearest-first, accepting the first bridge segment that crosses
/// no ring edge. Ring sizes are small (profile boundaries), so the
/// quadratic scan is cheap and avoids the corner cases of ray-casting
/// approaches.
fn merge_hole(
    poly: &mut Vec<(u32, [f64; 2])>,
    hole: &[(u32, [f64; 2])],
    all_edges: &[([f64; 2], [f64; 2])],
) -> Result<(), GeometryError> {
    let mut pairs: Vec<(f64, usize, usize)> = Vec::with_capacity(poly.len() * hole.len());
    for (pi, &(_, pp)) in poly.iter().enumerate() {
        for (hi, &(_, hp)) in hole.iter().enumerate() {
            let d2 = (pp[0] - hp[0]).powi(2) + (pp[1] - hp[1]).powi(2);
            pairs.push((d2, pi, hi));
        }
    }
    pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(core::cmp::Ordering::Equal));

    for &(_, pi, hi) in &pairs {
        let a = poly[pi].1;
        let b = hole[hi].1;
        if bridge_is_clear(a, b, all_edges) {
            // poly[..=pi] ++ hole[hi..] ++ hole[..=hi] ++ poly[pi..]:
            // walk the whole hole cycle starting (and ending, duplicated)
            // at hi, then return to the duplicated outer vertex pi.
            let mut spliced: Vec<(u32, [f64; 2])> = Vec::with_capacity(poly.len() + hole.len() + 2);
            spliced.extend_from_slice(&poly[..=pi]);
            spliced.extend_from_slice(&hole[hi..]);
            spliced.extend_from_slice(&hole[..=hi]);
            spliced.extend_from_slice(&poly[pi..]);
            *poly = spliced;
            return Ok(());
        }
    }
    Err(GeometryError::BadProfile)
}

/// `true` when the open segment `a–b` crosses none of `edges` (touching
/// an edge exactly at `a` or `b` is allowed — the bridge ends on ring
/// vertices).
fn bridge_is_clear(a: [f64; 2], b: [f64; 2], edges: &[([f64; 2], [f64; 2])]) -> bool {
    for &(p, q) in edges {
        if segments_cross(a, b, p, q) {
            return false;
        }
    }
    true
}

/// Segment intersection test for the bridge scan: `true` when segment
/// `a–b` and segment `p–q` intersect anywhere other than at a shared
/// endpoint of `a`/`b`.
fn segments_cross(a: [f64; 2], b: [f64; 2], p: [f64; 2], q: [f64; 2]) -> bool {
    let eps = 1e-12
        * [a, b, p, q]
            .iter()
            .map(|v| v[0].abs().max(v[1].abs()))
            .fold(1.0f64, f64::max)
            .powi(2);
    let same =
        |u: [f64; 2], v: [f64; 2]| (u[0] - v[0]).abs() < 1e-12 && (u[1] - v[1]).abs() < 1e-12;
    // Edges sharing a bridge endpoint never disqualify the bridge.
    if same(a, p) || same(a, q) || same(b, p) || same(b, q) {
        return false;
    }
    let d1 = cross2(a, b, p);
    let d2 = cross2(a, b, q);
    let d3 = cross2(p, q, a);
    let d4 = cross2(p, q, b);
    if ((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps))
        && ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps))
    {
        return true;
    }
    // Collinear touch: an endpoint of one segment lying on the other.
    let on_segment = |u: [f64; 2], v: [f64; 2], w: [f64; 2], d: f64| {
        d.abs() <= eps
            && w[0] >= u[0].min(v[0]) - 1e-12
            && w[0] <= u[0].max(v[0]) + 1e-12
            && w[1] >= u[1].min(v[1]) - 1e-12
            && w[1] <= u[1].max(v[1]) + 1e-12
    };
    on_segment(a, b, p, d1)
        || on_segment(a, b, q, d2)
        || on_segment(p, q, a, d3)
        || on_segment(p, q, b, d4)
}

/// Ear-clip a (weakly) simple counter-clockwise polygon into triangles
/// over the vertices' original indices. Bridge splices duplicate
/// vertices, so containment tests skip points coincident with the ear's
/// corners. A degenerate pass (no strict ear found) drops the flattest
/// vertex to guarantee termination on collinear spurs.
fn ear_clip(mut poly: Vec<(u32, [f64; 2])>) -> Result<Vec<[u32; 3]>, GeometryError> {
    if poly.len() < 3 {
        return Err(GeometryError::BadProfile);
    }
    // Scale-relative epsilon for area / orientation comparisons.
    let scale = poly
        .iter()
        .map(|(_, p)| p[0].abs().max(p[1].abs()))
        .fold(1.0f64, f64::max);
    let eps = 1e-12 * scale * scale;

    let mut tris: Vec<[u32; 3]> = Vec::with_capacity(poly.len().saturating_sub(2));
    while poly.len() > 3 {
        let n = poly.len();
        let mut clipped = false;
        for i in 0..n {
            let prev = poly[(i + n - 1) % n];
            let cur = poly[i];
            let next = poly[(i + 1) % n];
            let cr = cross2(prev.1, cur.1, next.1);
            if cr <= eps {
                continue; // reflex or flat corner: not an ear
            }
            if any_point_inside(&poly, prev.1, cur.1, next.1, eps) {
                continue;
            }
            tris.push([prev.0, cur.0, next.0]);
            poly.remove(i);
            clipped = true;
            break;
        }
        if !clipped {
            // No strict ear (collinear spur / duplicated bridge run):
            // drop the flattest corner to make progress; emit it only if
            // it has real area.
            let n = poly.len();
            let (mut flat_i, mut flat_cr) = (0usize, f64::INFINITY);
            for i in 0..n {
                let cr = cross2(poly[(i + n - 1) % n].1, poly[i].1, poly[(i + 1) % n].1).abs();
                if cr < flat_cr {
                    flat_cr = cr;
                    flat_i = i;
                }
            }
            if flat_cr > eps {
                // Nothing flat and no ear: the polygon is malformed
                // (self-intersecting profile) — stop rather than loop.
                return Err(GeometryError::BadProfile);
            }
            poly.remove(flat_i);
            if poly.len() < 3 {
                break;
            }
        }
    }
    if poly.len() == 3 {
        let cr = cross2(poly[0].1, poly[1].1, poly[2].1);
        if cr.abs() > eps {
            tris.push([poly[0].0, poly[1].0, poly[2].0]);
        }
    }
    if tris.is_empty() {
        return Err(GeometryError::BadProfile);
    }
    Ok(tris)
}

/// `true` when any polygon vertex other than the ear's own corners lies
/// inside (or on the boundary of) triangle `a b c`.
fn any_point_inside(
    poly: &[(u32, [f64; 2])],
    a: [f64; 2],
    b: [f64; 2],
    c: [f64; 2],
    eps: f64,
) -> bool {
    let same =
        |u: [f64; 2], v: [f64; 2]| (u[0] - v[0]).abs() < 1e-12 && (u[1] - v[1]).abs() < 1e-12;
    for &(_, p) in poly {
        // Skip the corners themselves and their bridge duplicates.
        if same(p, a) || same(p, b) || same(p, c) {
            continue;
        }
        let d1 = cross2(a, b, p);
        let d2 = cross2(b, c, p);
        let d3 = cross2(c, a, p);
        if d1 >= -eps && d2 >= -eps && d3 >= -eps {
            return true;
        }
    }
    false
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
/// `IfcFaceOuterBound` if any, else the first `IfcFaceBound`), gather
/// the remaining bounds as inner (hole) loops, and triangulate the
/// polygon-with-holes. `Bounds` is attribute index 0;
/// `IfcFaceBound.Bound` (the loop) is its attribute index 0.
///
/// A single convex outer loop takes the fan fast path (the historical
/// behaviour); a concave loop or a face with inner bounds is projected
/// onto its own plane (Newell normal) and triangulated hole-aware
/// through the shared [`triangulate_profile`] machinery, so face holes
/// are left open instead of being covered. Per-bound `Orientation`
/// flags are not applied — the loops are meshed as authored.
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

    // Prefer the IfcFaceOuterBound as the outer loop; fall back to the
    // first bound. (A face may carry just one untyped IfcFaceBound for
    // its outer loop.) Every other bound is an inner (hole) loop.
    let mut bound_ids: Vec<u64> = Vec::with_capacity(bounds.len());
    for b in bounds {
        let Some(bid) = b.as_reference() else {
            continue;
        };
        // Validate existence early for a precise error.
        step.get(bid).ok_or(GeometryError::MissingInstance(bid))?;
        bound_ids.push(bid);
    }
    if bound_ids.is_empty() {
        return Err(GeometryError::BadCoordinates);
    }
    let outer_pos = bound_ids
        .iter()
        .position(|&bid| {
            step.get(bid)
                .is_some_and(|b| b.keyword == "IFCFACEOUTERBOUND")
        })
        .unwrap_or(0);
    let outer_bid = bound_ids.remove(outer_pos);

    // IfcFaceBound(Bound : IfcLoop, Orientation : IfcBoolean): Bound is
    // attribute index 0.
    let bound_loop = |bid: u64| -> Result<u64, GeometryError> {
        step.get(bid)
            .ok_or(GeometryError::MissingInstance(bid))?
            .args
            .first()
            .and_then(Value::as_reference)
            .ok_or(GeometryError::BadCoordinates)
    };

    // Intern the outer loop (encounter order — this keeps the pooled
    // vertex numbering identical to the historical fan path).
    let outer = interned_loop(step, bound_loop(outer_bid)?, pool)?;

    // Inner bounds → hole loops.
    let mut holes: Vec<Vec<(u32, [f64; 3])>> = Vec::with_capacity(bound_ids.len());
    for bid in bound_ids {
        holes.push(interned_loop(step, bound_loop(bid)?, pool)?);
    }

    if holes.is_empty() && convex_loop(&outer) {
        // Fan from the first vertex — the planar convex fast path.
        for w in outer[1..].windows(2) {
            triangles.push([outer[0].0, w[0].0, w[1].0]);
        }
        return Ok(());
    }

    // Project the face onto its own plane and triangulate hole-aware.
    let pts3: Vec<[f64; 3]> = outer.iter().map(|&(_, p)| p).collect();
    let n = newell_normal(&pts3);
    let Some(n) = normalise(n) else {
        return Err(GeometryError::BadCoordinates);
    };
    // In-plane orthonormal basis (u, v) with v = n × u, so the outer
    // loop projects counter-clockwise (the Newell normal is the side the
    // loop winds counter-clockwise around).
    let seed = if n[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let d = dot_raw(seed, n);
    let u = normalise([seed[0] - d * n[0], seed[1] - d * n[1], seed[2] - d * n[2]])
        .unwrap_or([1.0, 0.0, 0.0]);
    let v = cross_raw(n, u);
    let origin = pts3[0];
    let project = |p: [f64; 3]| -> [f64; 2] {
        let r = [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]];
        [dot_raw(r, u), dot_raw(r, v)]
    };

    // Build the profile area; the pool indices are carried in a parallel
    // concatenated table so triangulator output maps back to the mesh.
    let mut index_table: Vec<u32> = outer.iter().map(|&(i, _)| i).collect();
    let outer_2d: Vec<[f64; 2]> = outer.iter().map(|&(_, p)| project(p)).collect();
    let mut hole_2d: Vec<Vec<[f64; 2]>> = Vec::with_capacity(holes.len());
    for hole in &holes {
        let mut ring: Vec<[f64; 2]> = hole.iter().map(|&(_, p)| project(p)).collect();
        let mut ids: Vec<u32> = hole.iter().map(|&(i, _)| i).collect();
        // triangulate_profile expects counter-clockwise hole rings.
        if signed_area_2x(&ring) < 0.0 {
            ring.reverse();
            ids.reverse();
        }
        hole_2d.push(ring);
        index_table.extend(ids);
    }
    let area = ProfileArea {
        outer: outer_2d,
        holes: hole_2d,
    };
    let cap = triangulate_profile(&area)?;
    for [a, b, c] in cap {
        triangles.push([
            index_table[a as usize],
            index_table[b as usize],
            index_table[c as usize],
        ]);
    }
    Ok(())
}

/// Resolve one `IfcPolyLoop` (`Polygon : LIST [3:?] OF
/// IfcCartesianPoint`, attribute index 0) into `(pooled vertex index,
/// position)` pairs, interning each point through the shared `pool`.
/// Edge / vertex loops (`IfcEdgeLoop` / `IfcVertexLoop`) are not
/// polygonal and are surfaced as `Unsupported`.
fn interned_loop(
    step: &StepFile,
    loop_id: u64,
    pool: &mut VertexPool,
) -> Result<Vec<(u32, [f64; 3])>, GeometryError> {
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
    let mut out = Vec::with_capacity(polygon.len());
    for p in polygon {
        let pid = p.as_reference().ok_or(GeometryError::BadCoordinates)?;
        let idx = pool.intern(step, pid)?;
        out.push((idx, pool.positions[idx as usize]));
    }
    Ok(out)
}

/// Newell's method: the (unnormalised) plane normal of a closed 3-D
/// polygon — robust for slightly non-planar loops, and oriented so the
/// loop winds counter-clockwise when viewed from the normal's side.
fn newell_normal(pts: &[[f64; 3]]) -> [f64; 3] {
    let mut n = [0.0f64; 3];
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    n
}

/// `true` when a 3-D loop is convex: every consecutive edge pair's
/// cross product points along the loop's Newell normal (within a
/// scale-relative tolerance, collinear corners allowed).
fn convex_loop(pts: &[(u32, [f64; 3])]) -> bool {
    let pts3: Vec<[f64; 3]> = pts.iter().map(|&(_, p)| p).collect();
    let n = newell_normal(&pts3);
    let scale = pts3
        .iter()
        .map(|p| p[0].abs().max(p[1].abs()).max(p[2].abs()))
        .fold(1.0f64, f64::max);
    let eps = 1e-12 * scale * scale;
    let len = pts3.len();
    for i in 0..len {
        let a = pts3[i];
        let b = pts3[(i + 1) % len];
        let c = pts3[(i + 2) % len];
        let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let e2 = [c[0] - b[0], c[1] - b[1], c[2] - b[2]];
        if dot_raw(cross_raw(e1, e2), n) < -eps {
            return false;
        }
    }
    true
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

    /// Sum of the 3-D areas of every triangle in the mesh.
    fn surface_area(m: &TriMesh) -> f64 {
        m.triangles
            .iter()
            .map(|t| {
                let a = m.positions[t[0] as usize];
                let b = m.positions[t[1] as usize];
                let c = m.positions[t[2] as usize];
                let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
                let x = [
                    e1[1] * e2[2] - e1[2] * e2[1],
                    e1[2] * e2[0] - e1[0] * e2[2],
                    e1[0] * e2[1] - e1[1] * e2[0],
                ];
                0.5 * (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]).sqrt()
            })
            .sum()
    }

    #[test]
    fn faceted_brep_inner_bound_is_a_hole() {
        // A face listing an inner IfcFaceBound (a triangle of area 0.5)
        // ahead of its IfcFaceOuterBound (a triangle of area 8): the
        // outer loop is identified by keyword regardless of order and
        // the inner loop is left open, so the meshed area is 8 − 0.5.
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
        // Outer (3 points, interned first) + hole (3 points).
        assert_eq!(m.vertex_count(), 6);
        assert_eq!(m.positions[0], [0.0, 0.0, 0.0]);
        assert_eq!(m.positions[1], [4.0, 0.0, 0.0]);
        assert!(
            (surface_area(&m) - 7.5).abs() < 1e-9,
            "{}",
            surface_area(&m)
        );
        // No triangle centroid falls inside the open hole.
        for t in &m.triangles {
            let ps: Vec<[f64; 3]> = t.iter().map(|&v| m.positions[v as usize]).collect();
            let cx = (ps[0][0] + ps[1][0] + ps[2][0]) / 3.0;
            let cy = (ps[0][1] + ps[1][1] + ps[2][1]) / 3.0;
            // Inside-hole test for the (1,1)-(2,1)-(1,2) triangle.
            let inside = cx > 1.0 && cy > 1.0 && (cx - 1.0) + (cy - 1.0) < 1.0;
            assert!(!inside, "cap triangle centroid ({cx},{cy}) in the hole");
        }
    }

    #[test]
    fn faceted_brep_concave_face_covers_exact_area() {
        // A concave L-shaped face (2×2 square minus its 1×1 top-right
        // corner, area 3) on the z = 5 plane: the fan fast path would
        // spill outside the notch; the projected ear clip must not.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,5.));\n\
             #2=IFCCARTESIANPOINT((2.,0.,5.));\n\
             #3=IFCCARTESIANPOINT((2.,1.,5.));\n\
             #4=IFCCARTESIANPOINT((1.,1.,5.));\n\
             #5=IFCCARTESIANPOINT((1.,2.,5.));\n\
             #6=IFCCARTESIANPOINT((0.,2.,5.));\n\
             #10=IFCPOLYLOOP((#1,#2,#3,#4,#5,#6));\n\
             #20=IFCFACEOUTERBOUND(#10,.T.);\n\
             #30=IFCFACE((#20));\n\
             #40=IFCCLOSEDSHELL((#30));\n\
             #41=IFCFACETEDBREP(#40);",
        );
        let m = tessellate_item(&f, 41).unwrap();
        assert_eq!(m.vertex_count(), 6);
        assert_eq!(m.triangle_count(), 4);
        assert!((surface_area(&m) - 3.0).abs() < 1e-9);
    }

    #[test]
    fn faceted_brep_holed_face_on_tilted_plane() {
        // A rectangular face with a rectangular hole, lying in the
        // tilted x = z plane (spanning the (1,0,1)/√2 and Y directions).
        // Projection through the Newell normal must recover the exact
        // in-plane areas: outer 10√2 × 10√2 = 200, hole 2√2 × 2 = 4√2.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #2=IFCCARTESIANPOINT((10.,0.,10.));\n\
             #3=IFCCARTESIANPOINT((10.,14.142135623730951,10.));\n\
             #4=IFCCARTESIANPOINT((0.,14.142135623730951,0.));\n\
             #5=IFCCARTESIANPOINT((4.,5.,4.));\n\
             #6=IFCCARTESIANPOINT((6.,5.,6.));\n\
             #7=IFCCARTESIANPOINT((6.,7.,6.));\n\
             #8=IFCCARTESIANPOINT((4.,7.,4.));\n\
             #10=IFCPOLYLOOP((#1,#2,#3,#4));\n\
             #11=IFCPOLYLOOP((#5,#6,#7,#8));\n\
             #20=IFCFACEOUTERBOUND(#10,.T.);\n\
             #21=IFCFACEBOUND(#11,.T.);\n\
             #30=IFCFACE((#20,#21));\n\
             #40=IFCCLOSEDSHELL((#30));\n\
             #41=IFCFACETEDBREP(#40);",
        );
        let m = tessellate_item(&f, 41).unwrap();
        assert_eq!(m.vertex_count(), 8);
        let outer = 200.0f64;
        let hole = 2.0f64 * 2f64.sqrt() * 2.0;
        assert!(
            (surface_area(&m) - (outer - hole)).abs() < 1e-6,
            "{}",
            surface_area(&m)
        );
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

    /// Sum of the (absolute) areas of every triangle whose three
    /// vertices all sit in the z = 0 plane — i.e. the bottom cap of a
    /// +Z extrusion.
    fn cap_area_z0(m: &TriMesh) -> f64 {
        m.triangles
            .iter()
            .filter(|t| t.iter().all(|&v| m.positions[v as usize][2].abs() < 1e-9))
            .map(|t| {
                let a = m.positions[t[0] as usize];
                let b = m.positions[t[1] as usize];
                let c = m.positions[t[2] as usize];
                0.5 * ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs()
            })
            .sum()
    }

    #[test]
    fn extruded_concave_profile_caps_cover_exact_area() {
        // An L-shaped (concave) profile: a 2×2 square missing its 1×1
        // top-right corner (area 3). A naive cap fan from vertex 0 would
        // spill outside the notch; the ear-clipped cap must cover
        // exactly 3 area units.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.));\n\
             #2=IFCCARTESIANPOINT((2.,0.));\n\
             #3=IFCCARTESIANPOINT((2.,1.));\n\
             #4=IFCCARTESIANPOINT((1.,1.));\n\
             #5=IFCCARTESIANPOINT((1.,2.));\n\
             #6=IFCCARTESIANPOINT((0.,2.));\n\
             #7=IFCPOLYLINE((#1,#2,#3,#4,#5,#6,#1));\n\
             #8=IFCARBITRARYCLOSEDPROFILEDEF(.AREA.,$,#7);\n\
             #9=IFCDIRECTION((0.,0.,1.));\n\
             #10=IFCEXTRUDEDAREASOLID(#8,$,#9,1.);",
        );
        let m = tessellate_item(&f, 10).unwrap();
        assert_eq!(m.vertex_count(), 12);
        // 4 cap triangles per cap + 6 side quads = 4 + 4 + 12 = 20.
        assert_eq!(m.triangle_count(), 20);
        assert!((cap_area_z0(&m) - 3.0).abs() < 1e-9, "{}", cap_area_z0(&m));
    }

    #[test]
    fn extruded_rectangle_hollow_profile_is_a_tube() {
        // A 4×4 rectangle with wall thickness 1: outer ring 4 points,
        // inner hole ring 4 points (2×2), cap area 16 − 4 = 12.
        let f = parse(
            "#1=IFCRECTANGLEHOLLOWPROFILEDEF(.AREA.,$,$,4.,4.,1.,$,$);\n\
             #2=IFCDIRECTION((0.,0.,1.));\n\
             #3=IFCEXTRUDEDAREASOLID(#1,$,#2,2.);",
        );
        let m = tessellate_item(&f, 3).unwrap();
        // 8 profile points × 2 (bottom + top).
        assert_eq!(m.vertex_count(), 16);
        // The hole ring occupies indices 4..8: 2×2 centred rectangle.
        for p in &m.positions[4..8] {
            assert!((p[0].abs() - 1.0).abs() < 1e-9);
            assert!((p[1].abs() - 1.0).abs() < 1e-9);
        }
        assert!((cap_area_z0(&m) - 12.0).abs() < 1e-9);
        // Every cap triangle avoids the hole: its centroid is outside
        // the open inner square.
        for t in &m.triangles {
            let ps: Vec<[f64; 3]> = t.iter().map(|&v| m.positions[v as usize]).collect();
            if ps.iter().any(|p| p[2].abs() > 1e-9) {
                continue; // side wall / top cap
            }
            let cx = (ps[0][0] + ps[1][0] + ps[2][0]) / 3.0;
            let cy = (ps[0][1] + ps[1][1] + ps[2][1]) / 3.0;
            assert!(
                cx.abs() > 1.0 - 1e-9 || cy.abs() > 1.0 - 1e-9,
                "cap triangle centroid ({cx},{cy}) inside the hole"
            );
        }
    }

    #[test]
    fn extruded_circle_hollow_profile_is_an_annulus() {
        // Radius 5, wall 2 → outer ring r=5, hole ring r=3; the cap area
        // is the 48-gon annulus area.
        let f = parse(
            "#1=IFCCIRCLEHOLLOWPROFILEDEF(.AREA.,$,$,5.,2.);\n\
             #2=IFCDIRECTION((0.,0.,1.));\n\
             #3=IFCEXTRUDEDAREASOLID(#1,$,#2,1.);",
        );
        let m = tessellate_item(&f, 3).unwrap();
        assert_eq!(m.vertex_count(), 48 * 2 * 2);
        // Bottom vertices sit on one of the two circles.
        for p in &m.positions[..96] {
            let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
            assert!((r - 5.0).abs() < 1e-9 || (r - 3.0).abs() < 1e-9);
        }
        // Regular 48-gon area = ½·n·r²·sin(2π/n); annulus = outer − inner.
        let n = 48.0f64;
        let gon = |r: f64| 0.5 * n * r * r * (2.0 * core::f64::consts::PI / n).sin();
        assert!((cap_area_z0(&m) - (gon(5.0) - gon(3.0))).abs() < 1e-6);
    }

    #[test]
    fn extruded_arbitrary_profile_with_voids() {
        // A 4×4 polyline square with a 1×1 polyline void at its centre:
        // cap area 16 − 1 = 15.
        let f = parse(
            "#1=IFCCARTESIANPOINT((0.,0.));\n\
             #2=IFCCARTESIANPOINT((4.,0.));\n\
             #3=IFCCARTESIANPOINT((4.,4.));\n\
             #4=IFCCARTESIANPOINT((0.,4.));\n\
             #5=IFCPOLYLINE((#1,#2,#3,#4,#1));\n\
             #6=IFCCARTESIANPOINT((1.5,1.5));\n\
             #7=IFCCARTESIANPOINT((2.5,1.5));\n\
             #8=IFCCARTESIANPOINT((2.5,2.5));\n\
             #9=IFCCARTESIANPOINT((1.5,2.5));\n\
             #10=IFCPOLYLINE((#6,#7,#8,#9,#6));\n\
             #11=IFCARBITRARYPROFILEDEFWITHVOIDS(.AREA.,$,#5,(#10));\n\
             #12=IFCDIRECTION((0.,0.,1.));\n\
             #13=IFCEXTRUDEDAREASOLID(#11,$,#12,1.);",
        );
        let m = tessellate_item(&f, 13).unwrap();
        assert_eq!(m.vertex_count(), 16);
        assert!((cap_area_z0(&m) - 15.0).abs() < 1e-9);
    }

    #[test]
    fn revolved_hollow_profile_quarter_turn_counts() {
        // A 2×2/wall-0.5 hollow rectangle centred at x = 5, revolved 90°
        // about the Y axis: 12 segments → 13 slices of 8 points, walls
        // for both rings, hole-aware end caps.
        let f = parse(
            "#1=IFCCARTESIANPOINT((5.,0.));\n\
             #2=IFCAXIS2PLACEMENT2D(#1,$);\n\
             #3=IFCRECTANGLEHOLLOWPROFILEDEF(.AREA.,$,#2,2.,2.,0.5,$,$);\n\
             #4=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #5=IFCDIRECTION((0.,1.,0.));\n\
             #6=IFCAXIS1PLACEMENT(#4,#5);\n\
             #7=IFCREVOLVEDAREASOLID(#3,$,#6,1.5707963267948966);",
        );
        let m = tessellate_item(&f, 7).unwrap();
        assert_eq!(m.vertex_count(), 13 * 8);
        // Walls: 12 segments × 8 edges × 2 tris; caps: 8 tris × 2 ends.
        assert_eq!(m.triangle_count(), 12 * 8 * 2 + 16);
    }

    #[test]
    fn extruded_indexed_polycurve_without_segments() {
        // An IfcIndexedPolyCurve with `$` Segments is the point list in
        // order: a unit square from an IfcCartesianPointList2D.
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST2D(((0.,0.),(1.,0.),(1.,1.),(0.,1.)));\n\
             #2=IFCINDEXEDPOLYCURVE(#1,$,.F.);\n\
             #3=IFCARBITRARYCLOSEDPROFILEDEF(.AREA.,$,#2);\n\
             #4=IFCDIRECTION((0.,0.,1.));\n\
             #5=IFCEXTRUDEDAREASOLID(#3,$,#4,2.);",
        );
        let m = tessellate_item(&f, 5).unwrap();
        assert_eq!(m.vertex_count(), 8);
        assert_eq!(m.triangle_count(), 12);
        assert!((cap_area_z0(&m) - 1.0).abs() < 1e-9);
        approx(m.positions[0], [0.0, 0.0, 0.0]);
        approx(m.positions[6], [1.0, 1.0, 2.0]);
    }

    #[test]
    fn extruded_indexed_polycurve_line_segments_share_junctions() {
        // Two IfcLineIndex segments (1,2,3) + (3,4,1): the shared
        // junction 3 and the closing 1 are emitted once → a 4-point
        // square ring.
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST2D(((0.,0.),(2.,0.),(2.,2.),(0.,2.)));\n\
             #2=IFCINDEXEDPOLYCURVE(#1,(IFCLINEINDEX((1,2,3)),IFCLINEINDEX((3,4,1))),.F.);\n\
             #3=IFCARBITRARYCLOSEDPROFILEDEF(.AREA.,$,#2);\n\
             #4=IFCDIRECTION((0.,0.,1.));\n\
             #5=IFCEXTRUDEDAREASOLID(#3,$,#4,1.);",
        );
        let m = tessellate_item(&f, 5).unwrap();
        assert_eq!(m.vertex_count(), 8);
        assert!((cap_area_z0(&m) - 4.0).abs() < 1e-9);
    }

    #[test]
    fn indexed_polycurve_arc_segment_unsupported() {
        // IfcArcIndex (a three-point arc segment) is not evaluated yet;
        // the SELECT keyword is surfaced.
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST2D(((0.,0.),(1.,1.),(2.,0.)));\n\
             #2=IFCINDEXEDPOLYCURVE(#1,(IFCARCINDEX((1,2,3))),.F.);\n\
             #3=IFCARBITRARYCLOSEDPROFILEDEF(.AREA.,$,#2);\n\
             #4=IFCDIRECTION((0.,0.,1.));\n\
             #5=IFCEXTRUDEDAREASOLID(#3,$,#4,1.);",
        );
        assert_eq!(
            tessellate_item(&f, 5).unwrap_err(),
            GeometryError::Unsupported("IFCARCINDEX".to_string())
        );
    }

    #[test]
    fn extruded_composite_profile_unions_components() {
        // Two 1×1 rectangles side by side (centres at x = ±2) → two
        // separate prisms in one mesh: 16 vertices, 24 triangles, total
        // bottom-cap area 2.
        let f = parse(
            "#1=IFCCARTESIANPOINT((-2.,0.));\n\
             #2=IFCAXIS2PLACEMENT2D(#1,$);\n\
             #3=IFCRECTANGLEPROFILEDEF(.AREA.,$,#2,1.,1.);\n\
             #4=IFCCARTESIANPOINT((2.,0.));\n\
             #5=IFCAXIS2PLACEMENT2D(#4,$);\n\
             #6=IFCRECTANGLEPROFILEDEF(.AREA.,$,#5,1.,1.);\n\
             #7=IFCCOMPOSITEPROFILEDEF(.AREA.,$,(#3,#6),$);\n\
             #8=IFCDIRECTION((0.,0.,1.));\n\
             #9=IFCEXTRUDEDAREASOLID(#7,$,#8,3.);",
        );
        let m = tessellate_item(&f, 9).unwrap();
        assert_eq!(m.vertex_count(), 16);
        assert_eq!(m.triangle_count(), 24);
        assert!((cap_area_z0(&m) - 2.0).abs() < 1e-9);
        // Component prisms stay centred on their own Positions.
        assert!(m.positions[..8].iter().all(|p| p[0] < 0.0));
        assert!(m.positions[8..].iter().all(|p| p[0] > 0.0));
    }

    #[test]
    fn boolean_union_merges_operand_boundaries() {
        // UNION of two triangulated face sets: both boundaries appear in
        // the result (a boundary superset of the regularised union).
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);\n\
             #3=IFCCARTESIANPOINTLIST3D(((5.,0.,0.),(6.,0.,0.),(5.,1.,0.)));\n\
             #4=IFCTRIANGULATEDFACESET(#3,$,.T.,((1,2,3)),$);\n\
             #5=IFCBOOLEANRESULT(.UNION.,#2,#4);",
        );
        let m = tessellate_item(&f, 5).unwrap();
        assert_eq!(m.vertex_count(), 6);
        assert_eq!(m.triangle_count(), 2);
        approx(m.positions[0], [0.0, 0.0, 0.0]);
        approx(m.positions[3], [5.0, 0.0, 0.0]);
        // The second operand's triangle is re-indexed past the first's
        // vertices.
        assert_eq!(m.triangles[1], [3, 4, 5]);
    }

    #[test]
    fn boolean_clipping_result_emits_first_operand() {
        // A wall body (extruded box) clipped by a plane half-space: the
        // subtraction is not yet carved, so the result is the box as
        // authored (visible rather than dropped).
        let f = parse(
            "#1=IFCRECTANGLEPROFILEDEF(.AREA.,$,$,2.,4.);\n\
             #2=IFCDIRECTION((0.,0.,1.));\n\
             #3=IFCEXTRUDEDAREASOLID(#1,$,#2,3.);\n\
             #4=IFCCARTESIANPOINT((0.,0.,2.));\n\
             #5=IFCAXIS2PLACEMENT3D(#4,$,$);\n\
             #6=IFCPLANE(#5);\n\
             #7=IFCHALFSPACESOLID(#6,.F.);\n\
             #8=IFCBOOLEANCLIPPINGRESULT(.DIFFERENCE.,#3,#7);",
        );
        let m = tessellate_item(&f, 8).unwrap();
        let plain = tessellate_item(&f, 3).unwrap();
        assert_eq!(m, plain);
        assert_eq!(m.vertex_count(), 8);
        assert_eq!(m.triangle_count(), 12);
    }

    #[test]
    fn boolean_clipping_chains_nest() {
        // Clipping results chain (the first operand of a clipping may
        // itself be a clipping): two levels resolve to the base solid.
        let f = parse(
            "#1=IFCRECTANGLEPROFILEDEF(.AREA.,$,$,1.,1.);\n\
             #2=IFCDIRECTION((0.,0.,1.));\n\
             #3=IFCEXTRUDEDAREASOLID(#1,$,#2,1.);\n\
             #4=IFCCARTESIANPOINT((0.,0.,0.5));\n\
             #5=IFCAXIS2PLACEMENT3D(#4,$,$);\n\
             #6=IFCPLANE(#5);\n\
             #7=IFCHALFSPACESOLID(#6,.T.);\n\
             #8=IFCBOOLEANCLIPPINGRESULT(.DIFFERENCE.,#3,#7);\n\
             #9=IFCBOOLEANCLIPPINGRESULT(.DIFFERENCE.,#8,#7);",
        );
        let m = tessellate_item(&f, 9).unwrap();
        assert_eq!(m.vertex_count(), 8);
        assert_eq!(m.triangle_count(), 12);
    }

    #[test]
    fn boolean_intersection_is_unsupported() {
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);\n\
             #3=IFCBOOLEANRESULT(.INTERSECTION.,#2,#2);",
        );
        assert_eq!(
            tessellate_item(&f, 3).unwrap_err(),
            GeometryError::Unsupported("IFCBOOLEANRESULT(.INTERSECTION.)".to_string())
        );
    }

    #[test]
    fn boolean_cyclic_operand_chain_is_bounded() {
        // A self-referential first operand must terminate at the depth
        // cap instead of recursing without end.
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);\n\
             #3=IFCBOOLEANRESULT(.DIFFERENCE.,#3,#2);",
        );
        assert_eq!(
            tessellate_item(&f, 3).unwrap_err(),
            GeometryError::Unsupported("IFCBOOLEANRESULT".to_string())
        );
    }

    #[test]
    fn boolean_union_via_shape_representation() {
        // A CSG body representation carrying a boolean result flows
        // through the representation walk.
        let f = parse(
            "#1=IFCCARTESIANPOINTLIST3D(((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\n\
             #2=IFCTRIANGULATEDFACESET(#1,$,.T.,((1,2,3)),$);\n\
             #3=IFCBOOLEANRESULT(.UNION.,#2,#2);\n\
             #4=IFCSHAPEREPRESENTATION(#8,'Body','CSG',(#3));",
        );
        let m = mesh_from_shape_representation(&f, 4).unwrap();
        assert_eq!(m.triangle_count(), 2);
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
