//! Framework integration (`registry` feature): a [`Mesh3DDecoder`]
//! implementation plus the [`Mesh3DRegistry`] registration helper.
//!
//! The decoder probes the `ISO-10303-21;` magic, fully parses +
//! validates the exchange structure, then walks every
//! `IfcProductDefinitionShape` to its supported body items —
//! tessellated face sets, faceted Breps / surface models, extruded and
//! revolved swept solids, boolean results, and mapped items — and emits
//! one scene node + mesh per shape, with **one primitive per
//! representation item**. Each shape's vertices are positioned in
//! **world space** by resolving the owning product's
//! `IfcLocalPlacement` chain (`placement_transform`); a shape with no
//! discoverable product placement stays in its local frame.
//!
//! Presentation is carried over where the file styles its items: an
//! `IfcStyledItem` → `IfcSurfaceStyle` shading colour becomes the
//! primitive's [`Material`] (`base_color`, deduplicated per style), and
//! an `IfcIndexedColourMap` on a triangulated face set becomes
//! per-vertex colours (vertices split per face so each triangle keeps
//! its flat colour). Advanced (curved) breps and boolean intersections
//! remain later Phase-3 slices.

use std::collections::HashMap;

use oxideav_core::Error as CoreError;
use oxideav_mesh3d::{
    Indices, Material, MaterialId, Mesh, Mesh3DDecoder, Mesh3DRegistry, Node, Primitive, Scene3D,
    Topology,
};

use crate::geometry::{
    meshed_items_from_product_shape, placement_transform, GeometryError, TriMesh,
};
use crate::parser::{parse_step_with_limits, probe_step, StepFile, StepLimits};
use crate::value::Value;
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
/// body representation.
///
/// Each shape that yields geometry becomes one [`Mesh`] — one
/// `Triangles` [`Primitive`] per representation item, carrying that
/// item's surface-style material and/or per-vertex colour map — plus
/// one [`Node`] (named after the shape's `#id`) parented under a single
/// scene root. Before meshing, the shape's vertices are positioned in
/// world space by the owning product's `IfcLocalPlacement` chain (see
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
    // Styled-item materials are deduplicated per IfcSurfaceStyle id.
    let mut material_cache: HashMap<u64, MaterialId> = HashMap::new();
    // Semantic (IfcRelAssociatesMaterial) fallback materials are
    // deduplicated per IfcMaterialSelect id.
    let mut semantic_cache: HashMap<u64, MaterialId> = HashMap::new();
    // The typed model folds the material / type relationships.
    let model = crate::schema::Model::from_step(step);

    // Walk every IfcProductDefinitionShape: it is the `Representation`
    // target of any geometric product, so this catches tessellated
    // bodies regardless of whether the owning element is in the typed
    // schema slice.
    for inst in step.instances.values() {
        if inst.keyword != "IFCPRODUCTDEFINITIONSHAPE" {
            continue;
        }
        match meshed_items_from_product_shape(step, inst.id) {
            Ok(items) => {
                // Position the body in world space via the owning
                // product's IfcLocalPlacement chain (identity when none
                // is discoverable / interpretable).
                let xform = shape_world_transform(step, inst.id);
                let mut mesh = Mesh::new(Some(format!("#{}", inst.id)));
                for (item_id, mut tri) in items {
                    tri.transform(&xform);
                    let mut prim = tri_to_primitive(&tri, step, item_id);
                    // Surface style (IfcStyledItem → IfcSurfaceStyle →
                    // shading colour) becomes the primitive material.
                    if let Some((style_id, name, rgba)) = surface_style_of_item(step, item_id) {
                        let mat_id = *material_cache.entry(style_id).or_insert_with(|| {
                            let mut mat = Material::new().with_base_color(rgba);
                            mat.name = name;
                            scene.add_material(mat)
                        });
                        prim.material = Some(mat_id);
                    }
                    mesh.primitives.push(prim);
                }
                if mesh.primitives.is_empty() {
                    continue;
                }
                // Primitives left unstyled fall back to the product's
                // semantic material association (IfcRelAssociatesMaterial,
                // occurrence-overrides-type): a named, colourless
                // Material so the substance ("Ceramic") reaches the
                // scene graph.
                if mesh.primitives.iter().any(|p| p.material.is_none()) {
                    if let Some((select_id, name)) = semantic_material_name(step, &model, inst.id) {
                        let mat_id = *semantic_cache.entry(select_id).or_insert_with(|| {
                            let mut mat = Material::new();
                            mat.name = Some(name);
                            scene.add_material(mat)
                        });
                        for prim in mesh.primitives.iter_mut() {
                            if prim.material.is_none() {
                                prim.material = Some(mat_id);
                            }
                        }
                    }
                }
                let mesh_id = scene.add_mesh(mesh);
                let node = Node::new()
                    .with_name(shape_node_name(step, inst.id))
                    .with_mesh(mesh_id);
                let node_id = scene.add_node(node);
                scene
                    .node_mut(root)
                    .expect("root just inserted")
                    .children
                    .push(node_id);
                emitted += 1;
            }
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
                 (e.g. `{kw}`); advanced-brep / boolean-intersection extraction is a later \
                 Phase-3 slice"
            ),
            None => "no geometric representations present in the model".to_string(),
        };
        return Err(CoreError::Unsupported(detail));
    }
    Ok(scene)
}

/// Resolve the surface colour a representation item is styled with, if
/// any: `(IfcSurfaceStyle id, style name, RGBA)`.
///
/// The presentation chain (IFC4 EXPRESS,
/// `IfcPresentationAppearanceResource`): an `IfcStyledItem(Item, Styles,
/// Name)` back-references the representation item; each of its `Styles`
/// is an `IfcStyleAssignmentSelect` — either an `IfcPresentationStyle`
/// directly or the (deprecated but common) wrapper
/// `IfcPresentationStyleAssignment(Styles)`. An `IfcSurfaceStyle(Name,
/// Side, Styles)` among them carries up to one shading member
/// (`IfcSurfaceStyleShading(SurfaceColour, Transparency)` or its
/// `IfcSurfaceStyleRendering` subtype, which shares those two leading
/// attributes); its `SurfaceColour` is an `IfcColourRgb(Name, Red,
/// Green, Blue)` and the optional `Transparency` (0 = opaque) maps to
/// alpha `1 − Transparency`.
fn surface_style_of_item(step: &StepFile, item_id: u64) -> Option<(u64, Option<String>, [f32; 4])> {
    for styled in step.instances.values() {
        if styled.keyword != "IFCSTYLEDITEM"
            || styled.args.first().and_then(Value::as_reference) != Some(item_id)
        {
            continue;
        }
        // Styles : SET [1:?] OF IfcStyleAssignmentSelect (index 1).
        let selects = styled.args.get(1).and_then(Value::as_list)?;
        // Flatten one level of IfcPresentationStyleAssignment.
        let mut style_ids: Vec<u64> = Vec::new();
        for sel in selects {
            let Some(sid) = sel.as_reference() else {
                continue;
            };
            let Some(sel_inst) = step.get(sid) else {
                continue;
            };
            if sel_inst.keyword == "IFCPRESENTATIONSTYLEASSIGNMENT" {
                if let Some(inner) = sel_inst.args.first().and_then(Value::as_list) {
                    style_ids.extend(inner.iter().filter_map(Value::as_reference));
                }
            } else {
                style_ids.push(sid);
            }
        }
        for style_id in style_ids {
            let Some(style) = step.get(style_id) else {
                continue;
            };
            if style.keyword != "IFCSURFACESTYLE" {
                continue;
            }
            // IfcPresentationStyle(Name) + IfcSurfaceStyle(Side, Styles).
            let name = style
                .args
                .first()
                .and_then(Value::as_str)
                .map(str::to_string);
            let members = style.args.get(2).and_then(Value::as_list)?;
            for member in members {
                let Some(mid) = member.as_reference() else {
                    continue;
                };
                let Some(m) = step.get(mid) else {
                    continue;
                };
                if m.keyword != "IFCSURFACESTYLESHADING" && m.keyword != "IFCSURFACESTYLERENDERING"
                {
                    continue;
                }
                // SurfaceColour (index 0) → IfcColourRgb(Name, R, G, B);
                // Transparency (index 1) optional.
                let colour_id = m.args.first().and_then(Value::as_reference)?;
                let colour = step.get(colour_id)?;
                if colour.keyword != "IFCCOLOURRGB" {
                    continue;
                }
                let r = colour.args.get(1).and_then(Value::as_number)?;
                let g = colour.args.get(2).and_then(Value::as_number)?;
                let b = colour.args.get(3).and_then(Value::as_number)?;
                let alpha = 1.0 - m.args.get(1).and_then(Value::as_number).unwrap_or(0.0);
                return Some((style_id, name, [r as f32, g as f32, b as f32, alpha as f32]));
            }
        }
    }
    None
}

/// Resolve the `IfcIndexedColourMap` attached to a tessellated face
/// set, if any: `(optional opacity, colour rows, one-based per-face
/// colour indices)`.
///
/// `IfcIndexedColourMap(MappedTo, Opacity, Colours, ColourIndex)` —
/// `MappedTo` back-references the face set, `Colours` is an
/// `IfcColourRgbList(ColourList)` and `ColourIndex` holds one one-based
/// row per face (IFC4 EXPRESS `IfcIndexedColourMap`).
type ColourMap = (Option<f64>, Vec<[f32; 3]>, Vec<usize>);
fn indexed_colour_map_of(step: &StepFile, item_id: u64) -> Option<ColourMap> {
    for map in step.instances.values() {
        if map.keyword != "IFCINDEXEDCOLOURMAP"
            || map.args.first().and_then(Value::as_reference) != Some(item_id)
        {
            continue;
        }
        let opacity = map.args.get(1).and_then(Value::as_number);
        let colours_id = map.args.get(2).and_then(Value::as_reference)?;
        let colours_inst = step.get(colours_id)?;
        if colours_inst.keyword != "IFCCOLOURRGBLIST" {
            continue;
        }
        let rows = colours_inst.args.first().and_then(Value::as_list)?;
        let mut colours = Vec::with_capacity(rows.len());
        for row in rows {
            let c = row.as_list()?;
            if c.len() != 3 {
                return None;
            }
            colours.push([
                c[0].as_number()? as f32,
                c[1].as_number()? as f32,
                c[2].as_number()? as f32,
            ]);
        }
        let idx = map.args.get(3).and_then(Value::as_list)?;
        let indices: Vec<usize> = idx
            .iter()
            .filter_map(Value::as_integer)
            .filter(|&v| v >= 1)
            .map(|v| v as usize)
            .collect();
        return Some((opacity, colours, indices));
    }
    None
}

/// The semantic material of the product owning a shape:
/// `(IfcMaterialSelect id, headline name)` from the
/// `IfcRelAssociatesMaterial` fold — a directly associated material
/// wins, else the material of the product's type object
/// ([`Model::material_of`](crate::schema::Model::material_of)).
/// `None` when the owner has no association or the assignment carries
/// no name.
fn semantic_material_name(
    step: &StepFile,
    model: &crate::schema::Model<'_>,
    shape_id: u64,
) -> Option<(u64, String)> {
    let owner = shape_owner(step, shape_id)?;
    let select_id = model.material_of(owner)?;
    let assignment = crate::material::material_assignment(step, select_id)?;
    Some((select_id, assignment.name()?.to_string()))
}

/// The product owning a shape: the rooted instance (string `GlobalId`
/// first argument) referencing `#shape_id` — the same scan
/// [`shape_node_name`] labels nodes with.
fn shape_owner(step: &StepFile, shape_id: u64) -> Option<u64> {
    step.instances.values().find_map(|inst| {
        (inst.keyword.starts_with("IFC")
            && inst.args.iter().any(|a| a.as_reference() == Some(shape_id))
            && inst.args.first().and_then(Value::as_str).is_some())
        .then_some(inst.id)
    })
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

/// Human-readable label for the product owning a shape: the product's
/// `IfcRoot.Name` (attribute index 2 on every rooted entity) when
/// present, else `KEYWORD#id`. Falls back to the shape's own `#id` when
/// no owning product is found.
fn shape_node_name(step: &StepFile, shape_id: u64) -> String {
    for inst in step.instances.values() {
        // The owning product references the shape as its Representation
        // and carries an ObjectPlacement / GlobalId, i.e. is a rooted
        // product — a plain reference scan suffices here because a shape
        // is referenced by exactly its product(s).
        if inst.keyword.starts_with("IFC")
            && inst.args.iter().any(|a| a.as_reference() == Some(shape_id))
            && inst.args.first().and_then(Value::as_str).is_some()
        {
            // IfcRoot(GlobalId, OwnerHistory, Name, Description).
            if let Some(name) = inst.args.get(2).and_then(Value::as_str) {
                if !name.is_empty() {
                    return name.to_string();
                }
            }
            return format!("{}#{}", inst.keyword, inst.id);
        }
    }
    format!("#{shape_id}")
}

/// Convert a crate-local [`TriMesh`] into one `Triangles` [`Primitive`].
///
/// The common case is an indexed primitive (`f32` positions + a
/// flattened `U32` index buffer). When the source item is an
/// `IfcTriangulatedFaceSet` carrying an `IfcIndexedColourMap` (whose
/// `ColourIndex` assigns one colour row per triangle), the primitive is
/// instead emitted **non-indexed with per-vertex colours**: vertices are
/// split per triangle so each face can carry its own flat colour (a
/// shared vertex may belong to differently-coloured faces). Faces
/// beyond the end of `ColourIndex`, or rows out of range, fall back to
/// white; the map's optional `Opacity` supplies alpha.
fn tri_to_primitive(tri: &TriMesh, step: &StepFile, item_id: u64) -> Primitive {
    let mut prim = Primitive::new(Topology::Triangles);

    let is_triangulated = step
        .get(item_id)
        .is_some_and(|i| i.keyword == "IFCTRIANGULATEDFACESET");
    if is_triangulated {
        if let Some((opacity, colours, colour_index)) = indexed_colour_map_of(step, item_id) {
            let alpha = 1.0f32.min(opacity.unwrap_or(1.0) as f32).max(0.0);
            let mut positions = Vec::with_capacity(tri.triangles.len() * 3);
            let mut vert_colours = Vec::with_capacity(tri.triangles.len() * 3);
            for (face, t) in tri.triangles.iter().enumerate() {
                let rgb = colour_index
                    .get(face)
                    .and_then(|&one_based| colours.get(one_based - 1))
                    .copied()
                    .unwrap_or([1.0, 1.0, 1.0]);
                for &v in t {
                    let p = tri.positions[v as usize];
                    positions.push([p[0] as f32, p[1] as f32, p[2] as f32]);
                    vert_colours.push([rgb[0], rgb[1], rgb[2], alpha]);
                }
            }
            prim.positions = positions;
            prim.colors = vec![vert_colours];
            return prim;
        }
    }

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
    prim
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
