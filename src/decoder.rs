//! Framework integration (`registry` feature): a [`Mesh3DDecoder`]
//! implementation plus the [`Mesh3DRegistry`] registration helper.
//!
//! Phase 3 (this release) extracts tessellated geometry
//! (`IFCTRIANGULATEDFACESET` / `IFCPOLYGONALFACESET`) and faceted
//! boundary representations (`IFCFACETEDBREP`(`WITHVOIDS`),
//! `IFCFACEBASEDSURFACEMODEL`, `IFCSHELLBASEDSURFACEMODEL`) into a
//! [`Scene3D`]: the decoder probes the `ISO-10303-21;` magic, fully
//! parses + validates the exchange structure, then walks every
//! `IfcProductDefinitionShape` to its supported body items and emits
//! one scene node + mesh per shape. Each shape's vertices are then
//! positioned in **world space** by resolving the owning product's
//! `IfcLocalPlacement` chain (`placement_transform`); a shape with no
//! discoverable product placement stays in its local frame. Swept-solid
//! / advanced-brep / boolean / mapped-item geometry styles remain later
//! Phase-3 slices.

use oxideav_core::Error as CoreError;
use oxideav_mesh3d::{
    Indices, Mesh, Mesh3DDecoder, Mesh3DRegistry, Node, Primitive, Scene3D, Topology,
};

use crate::geometry::{mesh_from_product_shape, placement_transform, GeometryError, TriMesh};
use crate::parser::{parse_step_with_limits, probe_step, StepFile, StepLimits};
use crate::Error;

/// IFC decoder front-end for the OxideAV 3D-format registry.
#[derive(Debug, Clone, Default)]
pub struct IfcDecoder {
    limits: StepLimits,
}

impl IfcDecoder {
    /// Decoder with default [`StepLimits`].
    pub fn new() -> Self {
        Self::default()
    }

    /// Decoder with caller-supplied DoS caps.
    pub fn with_limits(limits: StepLimits) -> Self {
        Self { limits }
    }
}

impl Mesh3DDecoder for IfcDecoder {
    fn decode(&mut self, bytes: &[u8]) -> oxideav_mesh3d::Result<Scene3D> {
        if !probe_step(bytes) {
            return Err(CoreError::InvalidData(
                "not an ISO 10303-21 exchange structure (missing `ISO-10303-21;` magic)".into(),
            ));
        }
        let step = parse_step_with_limits(bytes, &self.limits).map_err(|e| match e {
            Error::LimitExceeded(msg) => CoreError::ResourceExhausted(msg),
            other => CoreError::InvalidData(other.to_string()),
        })?;
        build_scene(&step)
    }
}

/// Build a [`Scene3D`] from a parsed IFC exchange structure by
/// tessellating every `IfcProductDefinitionShape` with a supported
/// (tessellated) body representation.
///
/// Each shape that yields geometry becomes one [`Mesh`] (a single
/// `Triangles` primitive with a `U32` index buffer) plus one [`Node`]
/// (named after the shape's `#id`) parented under a single scene root.
/// Before meshing, the shape's vertices are positioned in world space by
/// the owning product's `IfcLocalPlacement` chain (see
/// [`shape_world_transform`]); a shape with no discoverable placement
/// stays in its local frame. Shapes whose representations are all
/// unsupported geometry styles are skipped. If no shape produced any
/// geometry the decode fails with a clear [`CoreError::Unsupported`] so
/// callers can distinguish "parsed but no tessellation" from "parse
/// error".
fn build_scene(step: &StepFile) -> oxideav_mesh3d::Result<Scene3D> {
    let mut scene = Scene3D::new();
    let root = scene.add_node(Node::new().with_name("IfcProject"));
    scene.add_root(root);

    let mut emitted = 0usize;
    let mut last_unsupported: Option<String> = None;

    // Walk every IfcProductDefinitionShape: it is the `Representation`
    // target of any geometric product, so this catches tessellated
    // bodies regardless of whether the owning element is in the typed
    // schema slice.
    for inst in step.instances.values() {
        if inst.keyword != "IFCPRODUCTDEFINITIONSHAPE" {
            continue;
        }
        match mesh_from_product_shape(step, inst.id) {
            Ok(mut tri) if !tri.is_empty() => {
                // Position the body in world space via the owning
                // product's IfcLocalPlacement chain (identity when none
                // is discoverable / interpretable).
                let xform = shape_world_transform(step, inst.id);
                tri.transform(&xform);
                let mesh_id = scene.add_mesh(tri_to_mesh(&tri, inst.id));
                let node = Node::new()
                    .with_name(format!("#{}", inst.id))
                    .with_mesh(mesh_id);
                let node_id = scene.add_node(node);
                scene
                    .node_mut(root)
                    .expect("root just inserted")
                    .children
                    .push(node_id);
                emitted += 1;
            }
            Ok(_) => {}
            Err(GeometryError::Unsupported(kw)) => last_unsupported = Some(kw),
            // A shape with no usable representations is not fatal — other
            // shapes in the file may still tessellate.
            Err(_) => {}
        }
    }

    if emitted == 0 {
        let detail = match last_unsupported {
            Some(kw) => format!(
                "no tessellated geometry: only unsupported representation styles present \
                 (e.g. `{kw}`); swept-solid / Brep extraction is a later Phase-3 slice"
            ),
            None => "no geometric representations present in the model".to_string(),
        };
        return Err(CoreError::Unsupported(detail));
    }
    Ok(scene)
}

/// Resolve the world transform for an `IfcProductDefinitionShape` by
/// finding the product that references it and following that product's
/// `ObjectPlacement` chain.
///
/// IFC products carry their geometry by reference: a product's
/// `Representation` attribute (inherited from `IfcProduct`) points at the
/// shape, and its `ObjectPlacement` — declared immediately before
/// `Representation` on `IfcProduct` — points at the `IfcLocalPlacement`
/// that positions it. The shape does not back-reference its product, so
/// this scans for the (typically unique) product whose argument list
/// holds a `#shape_id` reference; the `ObjectPlacement` is then the
/// nearest preceding argument that references an `IfcLocalPlacement`.
///
/// Resolving by entity adjacency rather than a fixed positional index is
/// deliberate: product subtypes interleave their own attributes
/// (`IfcObject.ObjectType`, `IfcColumn.Tag`/`PredefinedType`, …) so the
/// `ObjectPlacement` / `Representation` pair sits at a subtype-dependent
/// offset, and the geometry-bearing product may be outside the typed
/// schema slice (e.g. `IfcBuildingElementProxy`).
///
/// Returns identity when no owning product is found, the product has no
/// `IfcLocalPlacement`, or the placement chain cannot be interpreted (a
/// non-`IfcLocalPlacement` / non-3-D placement); the body then stays in
/// its local frame rather than being dropped or mis-placed.
fn shape_world_transform(step: &StepFile, shape_id: u64) -> crate::geometry::Transform {
    use crate::geometry::Transform;

    for inst in step.instances.values() {
        // `Representation` is the reference to this shape.
        let Some(rep_idx) = inst
            .args
            .iter()
            .position(|a| a.as_reference() == Some(shape_id))
        else {
            continue;
        };
        // `ObjectPlacement` precedes `Representation` on IfcProduct; take
        // the nearest preceding argument that references an
        // IfcLocalPlacement (skipping subtype attributes such as Tag).
        let placement_id = inst.args[..rep_idx].iter().rev().find_map(|a| {
            let id = a.as_reference()?;
            (step.get(id)?.keyword == "IFCLOCALPLACEMENT").then_some(id)
        });
        let Some(placement_id) = placement_id else {
            continue;
        };
        // A malformed placement chain leaves the body in local space.
        return placement_transform(step, placement_id).unwrap_or(Transform::IDENTITY);
    }
    Transform::IDENTITY
}

/// Convert a crate-local [`TriMesh`] into an `oxideav_mesh3d::Mesh`: one
/// `Triangles` primitive with `f32` positions and a flattened `U32`
/// index buffer.
fn tri_to_mesh(tri: &TriMesh, shape_id: u64) -> Mesh {
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = tri
        .positions
        .iter()
        .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32])
        .collect();
    let mut idx = Vec::with_capacity(tri.triangles.len() * 3);
    for t in &tri.triangles {
        idx.push(t[0]);
        idx.push(t[1]);
        idx.push(t[2]);
    }
    prim.indices = Some(Indices::U32(idx));
    Mesh::new(Some(format!("#{shape_id}"))).with_primitive(prim)
}

/// Direct (registry-free) constructor — the conventional `make_`
/// entry point alongside [`register_mesh3d`].
pub fn make_decoder() -> IfcDecoder {
    IfcDecoder::new()
}

/// Register the IFC decoder into a [`Mesh3DRegistry`] under format id
/// `"ifc"` with the `.ifc` file extension.
pub fn register_mesh3d(registry: &mut Mesh3DRegistry) {
    registry.register_decoder("ifc", &["ifc"], Box::new(|| Box::new(IfcDecoder::new())));
}
