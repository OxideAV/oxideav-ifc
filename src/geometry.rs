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
//! Only the tessellation styles are handled here; swept solids
//! (`IfcExtrudedAreaSolid`), Breps (`IfcFacetedBrep`), boolean results,
//! and mapped items are later Phase-3 work and are reported as
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
/// Dispatches on the entity keyword: `IFCTRIANGULATEDFACESET` and
/// `IFCPOLYGONALFACESET` produce a [`TriMesh`]; any other keyword is a
/// [`GeometryError::Unsupported`]. This is the lowest-level entry —
/// most callers want [`mesh_from_shape_representation`] or the
/// `Model`-level walk.
pub fn tessellate_item(step: &StepFile, id: u64) -> Result<TriMesh, GeometryError> {
    let inst = step.get(id).ok_or(GeometryError::MissingInstance(id))?;
    match inst.keyword.as_str() {
        "IFCTRIANGULATEDFACESET" => triangulated_face_set(step, &inst.args),
        "IFCPOLYGONALFACESET" => polygonal_face_set(step, &inst.args),
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
