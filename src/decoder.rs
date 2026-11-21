//! Framework integration (`registry` feature): a [`Mesh3DDecoder`]
//! implementation plus the [`Mesh3DRegistry`] registration helper.
//!
//! The decoder probes the `ISO-10303-21;` magic, fully parses +
//! validates the exchange structure and builds a [`Scene3D`]:
//!
//! * **Hierarchy** — the project is the scene root; the spatial
//!   structure (`IfcRelAggregates`: site → building → storey → space)
//!   and element decomposition become nested nodes, and every product
//!   hangs under the structure element containing it
//!   (`IfcRelContainedInSpatialStructure`). Nodes carry the product's
//!   `Name`, its `GlobalId` and entity class (`extras`).
//! * **Geometry** — every product shape is flattened to its
//!   representation items (mapped items are followed into their shared
//!   source representation, so each source item is tessellated **once**
//!   however many products map it), one primitive per item, positioned
//!   through the product's `IfcLocalPlacement` chain. Non-body
//!   representations (`Box`, `Clearance`, `Lighting`, `CoG`,
//!   `Reference`) are not drawn and a `Body-FallBack` representation
//!   only stands in for a `Body` that produced nothing.
//! * **Openings** — the bodies of the opening elements voiding a host
//!   (`IfcRelVoidsElement`) are subtracted from the host's closed body
//!   items, bounding boxes first; the opening elements themselves are
//!   not drawn.
//! * **Presentation** — `IfcStyledItem` → `IfcSurfaceStyle` shading /
//!   rendering becomes the primitive's [`Material`] (colour,
//!   transparency, two-sidedness, reflectance), an `IfcIndexedColourMap`
//!   on a tessellated face set per-face vertex colours, and unstyled
//!   items fall back to the product's associated material (its own
//!   surface style when it has one).
//! * **Model facts** — the scene is Z-up, declares the project length
//!   unit, and carries the unit scale, map conversion and site
//!   geolocation in its `extras`.
//!
//! Work that is independent per item or per host runs on the caller's
//! thread budget ([`IfcDecoder::with_threads`]); the decoder is serial
//! by default and the scene is identical at every budget.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use oxideav_core::Error as CoreError;
use oxideav_core::ExecutionContext;
use oxideav_mesh3d::{
    AlphaMode, Axis, Indices, Material, MaterialId, Mesh, Mesh3DDecoder, Mesh3DRegistry, MeshId,
    Node, NodeId, Primitive, Scene3D, Topology, Unit,
};

use crate::Error;
use crate::geometry::session::{Session, Shared};
use crate::geometry::{
    BooleanOperator, Transform, TriMesh, curve_points, is_closed_mesh, mapped_item_transform,
    mesh_boolean, placement_transform, tessellate_item, tessellated_face_indices,
};
use crate::parser::{ParsedInstance, StepFile, StepLimits, parse_step_with_limits, probe_step};
use crate::schema::Model;
use crate::value::Value;

/// IFC decoder front-end for the OxideAV 3D-format registry.
#[derive(Debug, Clone)]
pub struct IfcDecoder {
    limits: StepLimits,
    threads: usize,
    instancing: bool,
    subtract_openings: bool,
    curves: bool,
}

impl Default for IfcDecoder {
    fn default() -> Self {
        Self {
            limits: StepLimits::default(),
            threads: 1,
            instancing: false,
            subtract_openings: true,
            curves: false,
        }
    }
}

impl IfcDecoder {
    /// Decoder with default [`StepLimits`]: serial, world-space meshes,
    /// openings subtracted from their hosts.
    pub fn new() -> Self {
        Self::default()
    }

    /// Decoder with caller-supplied DoS caps.
    pub fn with_limits(limits: StepLimits) -> Self {
        Self {
            limits,
            ..Self::default()
        }
    }

    /// Allow up to `threads` worker threads for the per-item
    /// tessellation and the per-host opening subtraction (values below
    /// 1 read as 1 — serial, the default). The scene is identical at
    /// every budget.
    #[must_use]
    pub fn with_threads(mut self, threads: usize) -> Self {
        self.threads = threads.max(1);
        self
    }

    /// Take the thread budget from the caller's execution context.
    #[must_use]
    pub fn with_execution_context(self, ctx: &ExecutionContext) -> Self {
        self.with_threads(ctx.threads)
    }

    /// Share meshes between products (default off).
    ///
    /// Off, every product's vertices are written in world space and
    /// each product owns its mesh. On, meshes stay in the product's
    /// local frame, the product node carries the placement as its
    /// transform, and products whose flattened representation is the
    /// same list of shared source items reference **one** mesh — a
    /// model of many mapped occurrences then costs one mesh per type
    /// instead of one per occurrence. Hosts carved by openings keep
    /// their own mesh.
    #[must_use]
    pub fn with_instancing(mut self, instancing: bool) -> Self {
        self.instancing = instancing;
        self
    }

    /// Subtract opening elements from the elements they void and leave
    /// the openings themselves undrawn (default on). Off, hosts stay
    /// as authored and every opening is drawn as its own solid.
    #[must_use]
    pub fn with_opening_subtraction(mut self, subtract: bool) -> Self {
        self.subtract_openings = subtract;
        self
    }

    /// Draw curve items as line strips (default off): the axis and
    /// footprint curves of products, the members of geometric curve
    /// sets, alignment curves. Products represented only by curves —
    /// an alignment — then appear in the scene.
    #[must_use]
    pub fn with_curves(mut self, curves: bool) -> Self {
        self.curves = curves;
        self
    }

    /// Build the scene of an already parsed exchange structure.
    pub fn decode_step(&self, step: &StepFile) -> oxideav_mesh3d::Result<Scene3D> {
        build_scene(step, self)
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
        build_scene(&step, self)
    }
}

/// Deepest `IfcMappedItem` nesting the flattening follows.
const MAX_MAPPED_DEPTH: usize = 64;

/// Most items one product shape may flatten to (nested mapped items
/// multiply; a hostile file must not make them explode).
const MAX_SHAPE_ITEMS: usize = 1 << 16;

/// Deepest ancestor chain followed when hanging a node in the tree.
const MAX_TREE_DEPTH: usize = 256;

/// The back-references the scene walk needs, gathered in one pass over
/// the instance graph (each used to be a scan per product).
#[derive(Debug, Default)]
struct Index {
    /// `IfcProductDefinitionShape` → the product it represents (lowest
    /// id wins).
    owner_of_shape: HashMap<u64, u64>,
    /// Representation item → the `IfcStyledItem` styling it.
    styled: HashMap<u64, u64>,
    /// Tessellated face set → its `IfcIndexedColourMap`.
    colour_map: HashMap<u64, u64>,
    /// `IfcMaterial` → its `IfcMaterialDefinitionRepresentation`.
    material_representation: HashMap<u64, u64>,
    /// Curves are drawn: geometric sets are opened into their members.
    open_sets: bool,
}

impl Index {
    fn build(step: &StepFile) -> Self {
        let mut index = Index::default();
        for inst in step.instances.values() {
            match inst.keyword.as_str() {
                "IFCSTYLEDITEM" => {
                    if let Some(item) = inst.args.first().and_then(Value::as_reference) {
                        index.styled.entry(item).or_insert(inst.id);
                    }
                }
                "IFCINDEXEDCOLOURMAP" => {
                    if let Some(item) = inst.args.first().and_then(Value::as_reference) {
                        index.colour_map.entry(item).or_insert(inst.id);
                    }
                }
                // (Name, Description, Representations, RepresentedMaterial).
                "IFCMATERIALDEFINITIONREPRESENTATION" => {
                    if let Some(material) = inst.args.get(3).and_then(Value::as_reference) {
                        index
                            .material_representation
                            .entry(material)
                            .or_insert(inst.id);
                    }
                }
                // A rooted instance (string GlobalId) referencing a
                // product definition shape is the product owning it.
                kw if kw.starts_with("IFC")
                    && inst.args.first().and_then(Value::as_str).is_some() =>
                {
                    for arg in &inst.args {
                        if let Some(target) = arg.as_reference()
                            && step
                                .get(target)
                                .is_some_and(|t| t.keyword == "IFCPRODUCTDEFINITIONSHAPE")
                        {
                            index.owner_of_shape.entry(target).or_insert(inst.id);
                        }
                    }
                }
                _ => {}
            }
        }
        index
    }
}

/// One representation item of a flattened product shape.
#[derive(Debug, Clone)]
struct Leaf {
    /// The item to tessellate.
    item: u64,
    /// Its frame within the product (the mapped items above it).
    xform: Transform,
    /// The nearest enclosing mapped item carrying a style of its own.
    inherited_style: Option<u64>,
    /// Part of a `Body-FallBack` representation.
    fallback: bool,
}

/// One product shape to draw.
#[derive(Debug)]
struct ShapePlan {
    shape: u64,
    owner: Option<u64>,
    leaves: Vec<Leaf>,
}

fn plan_shape(step: &StepFile, index: &Index, shape: &ParsedInstance) -> Vec<Leaf> {
    let mut leaves = Vec::new();
    // IfcProductDefinitionShape(Name, Description, Representations).
    let Some(reps) = shape.args.get(2).and_then(Value::as_list) else {
        return leaves;
    };
    for rep in reps {
        let Some(rep_inst) = step.resolve(rep) else {
            continue;
        };
        // IfcShapeRepresentation(ContextOfItems, RepresentationIdentifier,
        // RepresentationType, Items).
        let identifier = rep_inst.args.get(1).and_then(Value::as_str);
        if matches!(
            identifier,
            Some("Box" | "Clearance" | "Lighting" | "CoG" | "Reference")
        ) {
            continue;
        }
        let fallback = identifier == Some("Body-FallBack");
        let Some(items) = rep_inst.args.get(3).and_then(Value::as_list) else {
            continue;
        };
        for item in items {
            if let Some(id) = item.as_reference() {
                plan_item(
                    step,
                    index,
                    id,
                    Transform::IDENTITY,
                    None,
                    fallback,
                    0,
                    &mut leaves,
                );
            }
        }
    }
    leaves
}

#[allow(clippy::too_many_arguments)]
fn plan_item(
    step: &StepFile,
    index: &Index,
    id: u64,
    xform: Transform,
    inherited_style: Option<u64>,
    fallback: bool,
    depth: usize,
    leaves: &mut Vec<Leaf>,
) {
    if leaves.len() >= MAX_SHAPE_ITEMS {
        return;
    }
    let Some(inst) = step.get(id) else {
        return;
    };
    if index.open_sets
        && matches!(
            inst.keyword.as_str(),
            "IFCGEOMETRICSET" | "IFCGEOMETRICCURVESET"
        )
        && depth < MAX_MAPPED_DEPTH
    {
        // IfcGeometricSet(Elements): each member is drawn by itself.
        for element in inst.args.first().and_then(Value::as_list).unwrap_or(&[]) {
            // Members are points, curves and surfaces — never sets or
            // mapped items, so this does not nest.
            if let Some(member) = element.as_reference()
                && step.get(member).is_some_and(|m| {
                    !matches!(
                        m.keyword.as_str(),
                        "IFCGEOMETRICSET" | "IFCGEOMETRICCURVESET" | "IFCMAPPEDITEM"
                    )
                })
            {
                plan_item(
                    step,
                    index,
                    member,
                    xform,
                    inherited_style,
                    fallback,
                    depth + 1,
                    leaves,
                );
            }
        }
        return;
    }
    if inst.keyword != "IFCMAPPEDITEM" {
        leaves.push(Leaf {
            item: id,
            xform,
            inherited_style,
            fallback,
        });
        return;
    }
    if depth >= MAX_MAPPED_DEPTH {
        return;
    }
    // IfcMappedItem(MappingSource, MappingTarget) over
    // IfcRepresentationMap(MappingOrigin, MappedRepresentation).
    let Some(map) = inst.args.first().and_then(|v| step.resolve(v)) else {
        return;
    };
    if map.keyword != "IFCREPRESENTATIONMAP" {
        return;
    }
    let (Some(origin), Some(source), Some(target)) = (
        map.args.first().and_then(Value::as_reference),
        map.args.get(1).and_then(|v| step.resolve(v)),
        inst.args.get(1).and_then(Value::as_reference),
    ) else {
        return;
    };
    let Ok(mapped) = mapped_item_transform(step, origin, target) else {
        return;
    };
    let xform = xform.compose(&mapped);
    let inherited_style = if index.styled.contains_key(&id) {
        Some(id)
    } else {
        inherited_style
    };
    let Some(items) = source.args.get(3).and_then(Value::as_list) else {
        return;
    };
    for item in items {
        if let Some(inner) = item.as_reference() {
            plan_item(
                step,
                index,
                inner,
                xform,
                inherited_style,
                fallback,
                depth + 1,
                leaves,
            );
        }
    }
}

/// Run `f` over `items` on up to `threads` workers; results come back
/// in item order whatever the interleaving.
fn parallel_map<T: Sync, R: Send>(
    threads: usize,
    step: &StepFile,
    shared: &Arc<Shared>,
    items: &[T],
    f: impl Fn(&T) -> R + Sync,
) -> Vec<R> {
    let workers = threads.min(items.len()).max(1);
    if workers == 1 {
        return items.iter().map(&f).collect();
    }
    let next = AtomicUsize::new(0);
    let batches: Vec<Vec<(usize, R)>> = std::thread::scope(|scope| {
        let handles: Vec<_> = (0..workers)
            .map(|_| {
                scope.spawn(|| {
                    let _session = Session::join(step, shared);
                    let mut out = Vec::new();
                    loop {
                        let i = next.fetch_add(1, Ordering::Relaxed);
                        let Some(item) = items.get(i) else {
                            break;
                        };
                        out.push((i, f(item)));
                    }
                    out
                })
            })
            .collect();
        handles
            .into_iter()
            .map(|h| h.join().unwrap_or_else(|e| std::panic::resume_unwind(e)))
            .collect()
    });
    let mut slots: Vec<Option<R>> = (0..items.len()).map(|_| None).collect();
    for (i, r) in batches.into_iter().flatten() {
        slots[i] = Some(r);
    }
    slots
        .into_iter()
        .map(|r| r.expect("every item was claimed by one worker"))
        .collect()
}

/// An axis-aligned box: its low and high corners.
type Aabb = ([f64; 3], [f64; 3]);

/// Axis-aligned bounds of a mesh under `xform`.
fn bounds(mesh: &TriMesh, xform: &Transform) -> Aabb {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in &mesh.positions {
        let q = xform.apply(*p);
        for k in 0..3 {
            lo[k] = lo[k].min(q[k]);
            hi[k] = hi[k].max(q[k]);
        }
    }
    (lo, hi)
}

fn boxes_overlap(a: &Aabb, b: &Aabb) -> bool {
    (0..3).all(|k| a.0[k] <= b.1[k] && b.0[k] <= a.1[k])
}

/// The inverse of an affine transform (`None` when singular).
fn inverse(t: &Transform) -> Option<Transform> {
    let [a, b, c] = t.cols;
    let cross = |u: [f64; 3], v: [f64; 3]| {
        [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ]
    };
    let dot = |u: [f64; 3], v: [f64; 3]| u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    let (bc, ca, ab) = (cross(b, c), cross(c, a), cross(a, b));
    let det = dot(a, bc);
    if !det.is_finite() || det.abs() < 1e-300 {
        return None;
    }
    // Rows of the inverse are the reciprocal basis.
    let rows = [
        [bc[0] / det, bc[1] / det, bc[2] / det],
        [ca[0] / det, ca[1] / det, ca[2] / det],
        [ab[0] / det, ab[1] / det, ab[2] / det],
    ];
    let cols = [
        [rows[0][0], rows[1][0], rows[2][0]],
        [rows[0][1], rows[1][1], rows[2][1]],
        [rows[0][2], rows[1][2], rows[2][2]],
    ];
    let tr = t.translation;
    let translation = [-dot(rows[0], tr), -dot(rows[1], tr), -dot(rows[2], tr)];
    Some(Transform { cols, translation })
}

/// The world transform of the product owning a shape: the nearest
/// `IfcLocalPlacement` argument preceding the `Representation`
/// reference (product subtypes interleave their own attributes, so the
/// pair is found by adjacency rather than a fixed index). Identity
/// when the product has no interpretable placement — the body then
/// stays in its local frame.
fn owner_world(step: &StepFile, owner: Option<u64>, shape: u64) -> Transform {
    let Some(inst) = owner.and_then(|id| step.get(id)) else {
        return Transform::IDENTITY;
    };
    let Some(rep_idx) = inst
        .args
        .iter()
        .position(|a| a.as_reference() == Some(shape))
    else {
        return Transform::IDENTITY;
    };
    inst.args[..rep_idx]
        .iter()
        .rev()
        .find_map(|a| {
            let id = a.as_reference()?;
            (step.get(id)?.keyword == "IFCLOCALPLACEMENT").then_some(id)
        })
        .and_then(|id| placement_transform(step, id).ok())
        .unwrap_or(Transform::IDENTITY)
}

/// A surface style resolved to material terms.
#[derive(Debug, Clone)]
struct SurfaceLook {
    style: u64,
    name: Option<String>,
    rgba: [f32; 4],
    double_sided: bool,
    metallic: f32,
    roughness: Option<f32>,
}

/// Resolve the surface style an `IfcStyledItem` assigns.
///
/// `IfcStyledItem(Item, Styles, Name)`: each of `Styles` is an
/// `IfcPresentationStyle` or the (older) wrapper
/// `IfcPresentationStyleAssignment(Styles)`. An `IfcSurfaceStyle(Name,
/// Side, Styles)` carries at most one shading member —
/// `IfcSurfaceStyleShading(SurfaceColour, Transparency)` or its
/// `IfcSurfaceStyleRendering` subtype, which adds `DiffuseColour`
/// (index 2: a colour, or a factor on the surface colour),
/// `SpecularHighlight` (7: an `IfcSpecularRoughness` reads as the
/// roughness) and `ReflectanceMethod` (8: `.METAL.` / `.MIRROR.` read
/// as metallic). `Transparency` (0 = opaque) maps to alpha
/// `1 − Transparency`; `Side` `.BOTH.` makes the material two-sided.
fn surface_look(step: &StepFile, styled_item: u64) -> Option<SurfaceLook> {
    let styled = step.get(styled_item)?;
    let selects = styled.args.get(1).and_then(Value::as_list)?;
    let mut style_ids: Vec<u64> = Vec::new();
    for sel in selects {
        let Some(sel_inst) = step.resolve(sel) else {
            continue;
        };
        if sel_inst.keyword == "IFCPRESENTATIONSTYLEASSIGNMENT" {
            if let Some(inner) = sel_inst.args.first().and_then(Value::as_list) {
                style_ids.extend(inner.iter().filter_map(Value::as_reference));
            }
        } else {
            style_ids.push(sel_inst.id);
        }
    }
    let colour_of = |v: &Value| -> Option<[f64; 3]> {
        let colour = step.resolve(v)?;
        if colour.keyword != "IFCCOLOURRGB" {
            return None;
        }
        Some([
            colour.args.get(1).and_then(Value::as_number)?,
            colour.args.get(2).and_then(Value::as_number)?,
            colour.args.get(3).and_then(Value::as_number)?,
        ])
    };
    for style_id in style_ids {
        let Some(style) = step.get(style_id) else {
            continue;
        };
        if style.keyword != "IFCSURFACESTYLE" {
            continue;
        }
        let name = style
            .args
            .first()
            .and_then(Value::as_str)
            .map(str::to_string);
        let double_sided = style.args.get(1).and_then(Value::as_enum) == Some("BOTH");
        let members = style.args.get(2).and_then(Value::as_list)?;
        for member in members {
            let Some(m) = step.resolve(member) else {
                continue;
            };
            let rendering = m.keyword == "IFCSURFACESTYLERENDERING";
            if m.keyword != "IFCSURFACESTYLESHADING" && !rendering {
                continue;
            }
            let Some(mut rgb) = m.args.first().and_then(colour_of) else {
                continue;
            };
            let alpha = 1.0 - m.args.get(1).and_then(Value::as_number).unwrap_or(0.0);
            let mut metallic = 0.0;
            let mut roughness = None;
            if rendering {
                match m.args.get(2) {
                    Some(v @ Value::Reference(_)) => {
                        if let Some(diffuse) = colour_of(v) {
                            rgb = diffuse;
                        }
                    }
                    Some(Value::Typed { args, .. }) => {
                        if let Some(k) = args.first().and_then(Value::as_number) {
                            rgb = [rgb[0] * k, rgb[1] * k, rgb[2] * k];
                        }
                    }
                    _ => {}
                }
                if let Some(("IFCSPECULARROUGHNESS", args)) =
                    m.args.get(7).and_then(Value::as_typed)
                {
                    roughness = args.first().and_then(Value::as_number).map(|r| r as f32);
                }
                if matches!(
                    m.args.get(8).and_then(Value::as_enum),
                    Some("METAL" | "MIRROR")
                ) {
                    metallic = 1.0;
                }
            }
            return Some(SurfaceLook {
                style: style_id,
                name,
                rgba: [rgb[0] as f32, rgb[1] as f32, rgb[2] as f32, alpha as f32],
                double_sided,
                metallic,
                roughness,
            });
        }
    }
    None
}

fn material_of_look(look: &SurfaceLook) -> Material {
    let mut mat = Material::new().with_base_color(look.rgba);
    mat.name = look.name.clone();
    mat.double_sided = look.double_sided;
    // IFC surface colours are diffuse reflectances: a dielectric unless
    // the style says metal.
    mat.metallic = look.metallic;
    if let Some(r) = look.roughness {
        mat.roughness = r.clamp(0.0, 1.0);
    }
    if look.rgba[3] < 1.0 {
        mat.alpha_mode = AlphaMode::Blend;
    }
    mat
}

/// The surface style of an associated material, through its
/// `IfcMaterialDefinitionRepresentation` → `IfcStyledRepresentation`
/// → `IfcStyledItem` chain. `select` is the associated
/// `IfcMaterialSelect`; layer / profile / constituent sets and their
/// usages are searched for their first styled material.
fn material_look(step: &StepFile, index: &Index, select: u64) -> Option<SurfaceLook> {
    // Breadth-first over the material definition records only, a few
    // levels deep (usage → set → layer → material).
    let mut frontier = vec![select];
    for _ in 0..4 {
        let mut next = Vec::new();
        for &id in &frontier {
            let Some(inst) = step.get(id) else {
                continue;
            };
            if !inst.keyword.starts_with("IFCMATERIAL") {
                continue;
            }
            if inst.keyword == "IFCMATERIAL"
                && let Some(&repr) = index.material_representation.get(&id)
                && let Some(look) = material_representation_look(step, repr)
            {
                return Some(look);
            }
            next.extend(step.references_of(id));
        }
        frontier = next;
        if frontier.is_empty() {
            break;
        }
    }
    None
}

fn material_representation_look(step: &StepFile, repr: u64) -> Option<SurfaceLook> {
    // IfcMaterialDefinitionRepresentation.Representations (index 2) are
    // IfcStyledRepresentations whose Items (index 3) are styled items.
    let reps = step.get(repr)?.args.get(2).and_then(Value::as_list)?;
    for rep in reps {
        let Some(items) = step
            .resolve(rep)
            .and_then(|r| r.args.get(3).and_then(Value::as_list))
        else {
            continue;
        };
        for item in items {
            if let Some(id) = item.as_reference()
                && step.get(id).is_some_and(|i| i.keyword == "IFCSTYLEDITEM")
                && let Some(look) = surface_look(step, id)
            {
                return Some(look);
            }
        }
    }
    None
}

/// `(optional opacity, colour rows, one-based per-face colour indices)`
/// of an `IfcIndexedColourMap(MappedTo, Opacity, Colours, ColourIndex)`.
type ColourMap = (Option<f64>, Vec<[f32; 3]>, Vec<usize>);

fn indexed_colour_map(step: &StepFile, map_id: u64) -> Option<ColourMap> {
    let map = step.get(map_id)?;
    let opacity = map.args.get(1).and_then(Value::as_number);
    let colours_inst = map.args.get(2).and_then(|v| step.resolve(v))?;
    if colours_inst.keyword != "IFCCOLOURRGBLIST" {
        return None;
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
    Some((opacity, colours, indices))
}

/// An indexed `Triangles` primitive of `mesh` under `xform`.
fn indexed_primitive(mesh: &TriMesh, xform: &Transform) -> Primitive {
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = mesh
        .positions
        .iter()
        .map(|p| {
            let q = xform.apply(*p);
            [q[0] as f32, q[1] as f32, q[2] as f32]
        })
        .collect();
    let mut idx = Vec::with_capacity(mesh.triangles.len() * 3);
    for t in &mesh.triangles {
        idx.extend_from_slice(t);
    }
    prim.indices = Some(Indices::U32(idx));
    prim
}

/// A non-indexed primitive with per-vertex colours: vertices are split
/// per triangle so each authored face keeps its flat colour (a shared
/// vertex may belong to differently coloured faces). Faces beyond the
/// end of `ColourIndex`, or rows out of range, fall back to white; the
/// map's optional `Opacity` supplies alpha.
fn face_coloured_primitive(
    mesh: &TriMesh,
    face_of: &[u32],
    xform: &Transform,
    (opacity, colours, colour_index): &ColourMap,
) -> Primitive {
    let mut prim = Primitive::new(Topology::Triangles);
    let alpha = 1.0f32.min(opacity.unwrap_or(1.0) as f32).max(0.0);
    let mut positions = Vec::with_capacity(mesh.triangles.len() * 3);
    let mut vert_colours = Vec::with_capacity(mesh.triangles.len() * 3);
    for (t, &face) in mesh.triangles.iter().zip(face_of) {
        let rgb = colour_index
            .get(face as usize)
            .and_then(|&one_based| colours.get(one_based - 1))
            .copied()
            .unwrap_or([1.0, 1.0, 1.0]);
        for &v in t {
            let q = xform.apply(mesh.positions[v as usize]);
            positions.push([q[0] as f32, q[1] as f32, q[2] as f32]);
            vert_colours.push([rgb[0], rgb[1], rgb[2], alpha]);
        }
    }
    prim.positions = positions;
    prim.colors = vec![vert_colours];
    prim
}

/// `Name` of a rooted instance, else `KEYWORD#id`.
fn object_name(inst: &ParsedInstance) -> String {
    match inst.args.get(2).and_then(Value::as_str) {
        Some(name) if !name.is_empty() => name.to_string(),
        _ => format!("{}#{}", inst.keyword, inst.id),
    }
}

fn object_node(inst: &ParsedInstance) -> Node {
    let mut node = Node::new().with_name(object_name(inst));
    if let Some(guid) = inst.args.first().and_then(Value::as_str) {
        node.extras
            .insert("ifc:global_id".to_string(), guid.to_string().into());
    }
    node.extras
        .insert("ifc:class".to_string(), inst.keyword.clone().into());
    node
}

/// The scene tree under construction: one node per rooted object,
/// hung under what aggregates or contains it.
struct Tree<'m, 's> {
    step: &'s StepFile,
    model: &'m Model<'s>,
    root: NodeId,
    nodes: HashMap<u64, NodeId>,
}

impl Tree<'_, '_> {
    /// The node of object `id`, created (with its ancestors) on first
    /// use.
    fn node(&mut self, scene: &mut Scene3D, id: u64) -> NodeId {
        // Collect the not-yet-created chain upward, then create it
        // top-down.
        let mut chain: Vec<u64> = Vec::new();
        let mut parent_node = self.root;
        let mut current = Some(id);
        while let Some(cur) = current {
            if let Some(&known) = self.nodes.get(&cur) {
                parent_node = known;
                break;
            }
            if chain.len() >= MAX_TREE_DEPTH || chain.contains(&cur) {
                break;
            }
            chain.push(cur);
            current = self
                .model
                .aggregating_parent(cur)
                .or_else(|| self.model.container_of(cur));
        }
        for &cur in chain.iter().rev() {
            let node = match self.step.get(cur) {
                Some(inst) => object_node(inst),
                None => Node::new().with_name(format!("#{cur}")),
            };
            let node_id = scene.add_node(node);
            if let Some(parent) = scene.node_mut(parent_node) {
                parent.children.push(node_id);
            }
            self.nodes.insert(cur, node_id);
            parent_node = node_id;
        }
        parent_node
    }
}

/// The carved body items of one host: `(leaf index, mesh in the
/// product's local frame)` for every item an opening changed.
type Carved = Vec<(usize, TriMesh)>;

fn matrix_of(t: &Transform) -> [[f32; 4]; 4] {
    let mut m = [[0.0f32; 4]; 4];
    for (r, row) in m.iter_mut().enumerate().take(3) {
        for (c, cell) in row.iter_mut().enumerate().take(3) {
            *cell = t.cols[c][r] as f32;
        }
        row[3] = t.translation[r] as f32;
    }
    m[3][3] = 1.0;
    m
}

/// A hashable image of a transform.
fn transform_bits(t: &Transform) -> [u64; 12] {
    let mut bits = [0u64; 12];
    for (slot, v) in bits
        .iter_mut()
        .zip(t.cols.iter().flatten().chain(t.translation.iter()))
    {
        *slot = v.to_bits();
    }
    bits
}

fn build_scene(step: &StepFile, options: &IfcDecoder) -> oxideav_mesh3d::Result<Scene3D> {
    let session = Session::enter(step);
    let shared = session.shared();
    let model = Model::from_step(step);
    let mut index = Index::build(step);
    index.open_sets = options.curves;

    // ---- Plan: flatten every product shape to its items. ------------
    let plans: Vec<ShapePlan> = step
        .instances
        .values()
        .filter(|inst| inst.keyword == "IFCPRODUCTDEFINITIONSHAPE")
        .map(|inst| ShapePlan {
            shape: inst.id,
            owner: index.owner_of_shape.get(&inst.id).copied(),
            leaves: plan_shape(step, &index, inst),
        })
        .collect();

    // ---- Tessellate every distinct item once. -----------------------
    let mut slot_of: HashMap<u64, usize> = HashMap::new();
    let mut distinct: Vec<u64> = Vec::new();
    for leaf in plans.iter().flat_map(|p| p.leaves.iter()) {
        slot_of.entry(leaf.item).or_insert_with(|| {
            distinct.push(leaf.item);
            distinct.len() - 1
        });
    }
    let meshes: Vec<Result<Arc<TriMesh>, Option<String>>> = parallel_map(
        options.threads,
        step,
        &shared,
        &distinct,
        |&id| match tessellate_item(step, id) {
            Ok(mesh) if !mesh.is_empty() => Ok(Arc::new(mesh)),
            Ok(_) => Err(None),
            Err(crate::geometry::GeometryError::Unsupported(kw)) => Err(Some(kw)),
            Err(_) => Err(None),
        },
    );
    let mesh_of = |leaf: &Leaf| meshes[slot_of[&leaf.item]].as_ref().ok();
    // Items that are curves, sampled when curves are drawn.
    let polylines: Vec<Option<Arc<Vec<[f64; 3]>>>> = if options.curves {
        let pending: Vec<(usize, u64)> = distinct
            .iter()
            .enumerate()
            .filter(|(slot, _)| matches!(meshes[*slot], Err(Some(_))))
            .map(|(slot, &id)| (slot, id))
            .collect();
        let sampled = parallel_map(options.threads, step, &shared, &pending, |&(_, id)| {
            curve_points(step, id).ok().filter(|pts| pts.len() >= 2)
        });
        let mut polylines = vec![None; distinct.len()];
        for ((slot, _), pts) in pending.into_iter().zip(sampled) {
            polylines[slot] = pts.map(Arc::new);
        }
        polylines
    } else {
        Vec::new()
    };

    // ---- Openings: carve each host's closed body items. -------------
    let plan_of_owner: HashMap<u64, usize> = plans
        .iter()
        .enumerate()
        .filter_map(|(i, p)| Some((p.owner?, i)))
        .collect();
    let hosts: Vec<usize> = if options.subtract_openings {
        plans
            .iter()
            .enumerate()
            .filter(|(_, p)| p.owner.is_some_and(|o| !model.openings_of(o).is_empty()))
            .map(|(i, _)| i)
            .collect()
    } else {
        Vec::new()
    };
    let carve = |&host: &usize| -> Carved {
        let plan = &plans[host];
        let Some(owner) = plan.owner else {
            return Vec::new();
        };
        let Some(to_host) = inverse(&owner_world(step, plan.owner, plan.shape)) else {
            return Vec::new();
        };
        // The opening bodies in the host's local frame, with bounds.
        let mut tools: Vec<(TriMesh, Aabb)> = Vec::new();
        for &opening in model.openings_of(owner) {
            let Some(&oplan) = plan_of_owner.get(&opening) else {
                continue;
            };
            let oplan = &plans[oplan];
            let rel = to_host.compose(&owner_world(step, oplan.owner, oplan.shape));
            for leaf in &oplan.leaves {
                let Some(mesh) = mesh_of(leaf) else {
                    continue;
                };
                if !is_closed_mesh(mesh) {
                    continue;
                }
                let tool = mesh.transformed(&rel.compose(&leaf.xform));
                let bb = bounds(&tool, &Transform::IDENTITY);
                tools.push((tool, bb));
            }
        }
        let mut carved: Carved = Vec::new();
        for (i, leaf) in plan.leaves.iter().enumerate() {
            let Some(mesh) = mesh_of(leaf) else {
                continue;
            };
            let mut bb = bounds(mesh, &leaf.xform);
            if !tools.iter().any(|(_, tb)| boxes_overlap(&bb, tb)) || !is_closed_mesh(mesh) {
                continue;
            }
            let mut body = mesh.transformed(&leaf.xform);
            let mut changed = false;
            for (tool, tb) in &tools {
                if !boxes_overlap(&bb, tb) {
                    continue;
                }
                // A tool the evaluator cannot take leaves the body as
                // it stands.
                if let Ok(cut) = mesh_boolean(&body, tool, BooleanOperator::Difference) {
                    body = cut;
                    bb = bounds(&body, &Transform::IDENTITY);
                    changed = true;
                }
            }
            if changed {
                carved.push((i, body));
            }
        }
        carved
    };
    let carved: HashMap<usize, Carved> = hosts
        .iter()
        .copied()
        .zip(parallel_map(options.threads, step, &shared, &hosts, carve))
        .collect();

    // ---- Assemble the scene. ----------------------------------------
    let mut scene = Scene3D::new();
    scene.up_axis = Axis::PosZ;
    scene.front_axis = Axis::NegY;
    let root_node = match model.project_id().and_then(|id| step.get(id)) {
        Some(project) => {
            let mut node = object_node(project);
            if project
                .args
                .get(2)
                .and_then(Value::as_str)
                .is_none_or(str::is_empty)
            {
                node.name = Some("IfcProject".to_string());
            }
            node
        }
        None => Node::new().with_name("IfcProject"),
    };
    let root = scene.add_node(root_node);
    scene.add_root(root);
    let mut tree = Tree {
        step,
        model: &model,
        root,
        nodes: HashMap::new(),
    };
    if let Some(project) = model.project_id() {
        tree.nodes.insert(project, root);
    }
    // The spatial structure is part of the scene whether or not it
    // holds drawable products.
    let spatial: Vec<u64> = model.spatial_elements().map(|e| e.id()).collect();
    for id in spatial {
        tree.node(&mut scene, id);
    }

    let mut emitted = 0usize;
    let mut last_unsupported: Option<String> = None;
    let mut style_materials: HashMap<u64, MaterialId> = HashMap::new();
    let mut semantic_materials: HashMap<u64, Option<MaterialId>> = HashMap::new();
    type InstanceKey = (Vec<(u64, [u64; 12], Option<u64>)>, Option<u64>);
    let mut shared_meshes: HashMap<InstanceKey, MeshId> = HashMap::new();

    for (plan_idx, plan) in plans.iter().enumerate() {
        let owner_inst = plan.owner.and_then(|id| step.get(id));
        // An opening voiding a host is a hole, not a solid.
        if options.subtract_openings
            && plan
                .owner
                .is_some_and(|o| model.voided_element_of(o).is_some())
        {
            continue;
        }
        // Body items; a fall-back representation only when the body
        // proper produced nothing.
        let mut any = false;
        for leaf in &plan.leaves {
            match &meshes[slot_of[&leaf.item]] {
                Ok(_) => any |= !leaf.fallback,
                Err(Some(kw)) => last_unsupported = Some(kw.clone()),
                Err(None) => {}
            }
        }
        let drawn: Vec<(usize, &Leaf, &Arc<TriMesh>)> = plan
            .leaves
            .iter()
            .enumerate()
            .filter(|(_, leaf)| leaf.fallback != any)
            .filter_map(|(i, leaf)| Some((i, leaf, mesh_of(leaf)?)))
            .collect();
        let lines: Vec<(&Leaf, &Arc<Vec<[f64; 3]>>)> = plan
            .leaves
            .iter()
            .filter_map(|leaf| Some((leaf, polylines.get(slot_of[&leaf.item])?.as_ref()?)))
            .collect();
        if drawn.is_empty() && lines.is_empty() {
            continue;
        }
        let world = owner_world(step, plan.owner, plan.shape);
        let carved_here = carved.get(&plan_idx).filter(|c| !c.is_empty());

        // The look of every drawn item.
        let looks: Vec<Option<u64>> = drawn
            .iter()
            .map(|(_, leaf, _)| {
                index
                    .styled
                    .get(&leaf.item)
                    .copied()
                    .or(leaf.inherited_style)
            })
            .collect();
        let semantic_select = plan.owner.and_then(|o| model.material_of(o));

        let instance_key: Option<InstanceKey> =
            (options.instancing && carved_here.is_none() && lines.is_empty()).then(|| {
                (
                    drawn
                        .iter()
                        .zip(&looks)
                        .map(|((_, leaf, _), look)| (leaf.item, transform_bits(&leaf.xform), *look))
                        .collect(),
                    semantic_select,
                )
            });
        let reused = instance_key
            .as_ref()
            .and_then(|key| shared_meshes.get(key).copied());
        let mesh_id = match reused {
            Some(id) => id,
            None => {
                let mut mesh = Mesh::new(Some(format!("#{}", plan.shape)));
                for ((i, leaf, tri), styled) in drawn.iter().zip(&looks) {
                    let cut = carved_here
                        .and_then(|c| c.iter().find(|(leaf_idx, _)| leaf_idx == i))
                        .map(|(_, m)| m);
                    // Vertex frame: world when baking, the product's
                    // own frame when instancing.
                    let frame = |local: &Transform| {
                        if options.instancing {
                            *local
                        } else {
                            world.compose(local)
                        }
                    };
                    let mut prim = match cut {
                        Some(body) => indexed_primitive(body, &frame(&Transform::IDENTITY)),
                        None => {
                            let xform = frame(&leaf.xform);
                            let coloured = index.colour_map.get(&leaf.item).and_then(|&map| {
                                let map = indexed_colour_map(step, map)?;
                                let (mesh, face_of) = tessellated_face_indices(step, leaf.item)?;
                                Some(face_coloured_primitive(&mesh, &face_of, &xform, &map))
                            });
                            coloured.unwrap_or_else(|| indexed_primitive(tri, &xform))
                        }
                    };
                    if let Some(look) = styled.and_then(|s| surface_look(step, s)) {
                        let mat_id = *style_materials
                            .entry(look.style)
                            .or_insert_with(|| scene.add_material(material_of_look(&look)));
                        prim.material = Some(mat_id);
                    }
                    mesh.primitives.push(prim);
                }
                // Unstyled items take the product's associated material:
                // its own surface style when it has one, else a named
                // colourless material so the substance ("Ceramic")
                // reaches the scene graph.
                if mesh.primitives.iter().any(|p| p.material.is_none())
                    && let Some(select) = semantic_select
                {
                    let mat_id = *semantic_materials.entry(select).or_insert_with(|| {
                        let assignment = crate::material::material_assignment(step, select);
                        let name = assignment.as_ref().and_then(|a| a.name()).map(String::from);
                        match material_look(step, &index, select) {
                            Some(look) => {
                                let mut mat = material_of_look(&look);
                                if name.is_some() {
                                    mat.name = name;
                                }
                                Some(scene.add_material(mat))
                            }
                            None => name.map(|name| {
                                let mut mat = Material::new();
                                mat.name = Some(name);
                                scene.add_material(mat)
                            }),
                        }
                    });
                    if let Some(mat_id) = mat_id {
                        for prim in mesh.primitives.iter_mut() {
                            if prim.material.is_none() {
                                prim.material = Some(mat_id);
                            }
                        }
                    }
                }
                // Curve items as line strips (unstyled).
                for (leaf, points) in &lines {
                    let xform = if options.instancing {
                        leaf.xform
                    } else {
                        world.compose(&leaf.xform)
                    };
                    let mut prim = Primitive::new(Topology::LineStrip);
                    prim.positions = points
                        .iter()
                        .map(|p| {
                            let q = xform.apply(*p);
                            [q[0] as f32, q[1] as f32, q[2] as f32]
                        })
                        .collect();
                    mesh.primitives.push(prim);
                }
                let id = scene.add_mesh(mesh);
                if let Some(key) = instance_key {
                    shared_meshes.insert(key, id);
                }
                id
            }
        };

        // The product's node: its place in the tree, or a bare node
        // under the root for a shape nothing owns.
        let node_id = match owner_inst {
            Some(inst) => tree.node(&mut scene, inst.id),
            None => {
                let id = scene.add_node(Node::new().with_name(format!("#{}", plan.shape)));
                if let Some(root_node) = scene.node_mut(root) {
                    root_node.children.push(id);
                }
                id
            }
        };
        if let Some(node) = scene.node_mut(node_id) {
            if node.mesh.is_some() {
                // A product drawn through a second shape: keep the first.
                continue;
            }
            node.mesh = Some(mesh_id);
            if options.instancing {
                node.transform = oxideav_mesh3d::Transform::Matrix(matrix_of(&world));
            }
        }
        emitted += 1;
    }

    if emitted == 0 {
        let detail = match last_unsupported {
            Some(kw) => format!(
                "no tessellated geometry: only unsupported representation styles present \
                 (e.g. `{kw}`)"
            ),
            None => "no geometric representations present in the model".to_string(),
        };
        return Err(CoreError::Unsupported(detail));
    }

    describe_model(step, &mut scene);
    Ok(scene)
}

/// Record the model-level facts on the scene: length unit, map
/// conversion and site geolocation.
fn describe_model(step: &StepFile, scene: &mut Scene3D) {
    if let Some(scale) = crate::schema::length_unit_scale(step) {
        let near = |unit: f64| (scale - unit).abs() <= 1e-9 * unit;
        scene.unit = if near(0.01) {
            Unit::Centimetres
        } else if near(0.001) {
            Unit::Millimetres
        } else if near(0.0254) {
            Unit::Inches
        } else if near(0.3048) {
            Unit::Feet
        } else if near(0.9144) {
            Unit::Yards
        } else {
            Unit::Metres
        };
        // The exact factor (a decimetre model has no `Unit` of its own).
        scene
            .extras
            .insert("ifc:metres_per_unit".to_string(), scale.into());
    }
    if let Some(conversion) = crate::geo::map_conversion(step) {
        let mut record = serde_json::Map::new();
        record.insert("eastings".into(), conversion.eastings.into());
        record.insert("northings".into(), conversion.northings.into());
        record.insert(
            "orthogonal_height".into(),
            conversion.orthogonal_height.into(),
        );
        record.insert("rotation".into(), conversion.rotation_angle().into());
        if let Some(scale) = conversion.scale {
            record.insert("scale".into(), scale.into());
        }
        if let Some(crs) = conversion.target_crs.as_ref().and_then(|crs| crs.name) {
            record.insert("crs".into(), crs.to_string().into());
        }
        scene
            .extras
            .insert("ifc:map_conversion".to_string(), record.into());
    }
    if let Some(site) = crate::geo::site_geolocation(step) {
        let mut record = serde_json::Map::new();
        record.insert("latitude".into(), site.latitude.into());
        record.insert("longitude".into(), site.longitude.into());
        if let Some(elevation) = site.elevation {
            record.insert("elevation".into(), elevation.into());
        }
        scene
            .extras
            .insert("ifc:site_geolocation".to_string(), record.into());
    }
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
