# oxideav-ifc

Pure-Rust **IFC** (Industry Foundation Classes, **ISO 16739**) reader
for the OxideAV framework — clean-room, implemented from the ISO
10303-21 exchange-structure grammar and the buildingSMART IFC schema
documentation only.

IFC is the open BIM (Building Information Modeling) interchange
format: a complete building model — spatial hierarchy (site →
building → storey → space), classified elements (walls, doors,
windows, beams, columns, slabs), property sets, and per-element
geometric representations — serialised by default as a **STEP
physical file** (ISO 10303-21 clear-text encoding) with the `.ifc`
extension.

## Status / phases

| Phase | Scope | Status |
|-------|-------|--------|
| **1** | STEP physical-file (ISO 10303-21) parser: HEADER + DATA instance graph, full parameter grammar, reference resolver, DoS caps | ✅ landed |
| **2** | EXPRESS-schema-aware typing: named attribute resolution per the IFC 4 EXPRESS inheritance chains, spatial-structure traversal | ✅ this release (core entity slice) |
| **3** | Geometry extraction into `oxideav-mesh3d::Scene3D`: `IFCTRIANGULATEDFACESET` / `IFCPOLYGONALFACESET` tessellations + `IfcLocalPlacement` world-positioning | ✅ this release (tessellation + placement slices); swept solids / Breps later |

## Phase 1 surface

```rust
let step = oxideav_ifc::parse_step(&bytes)?;
assert_eq!(step.header.file_schema, ["IFC4"]);
for wall in step.instances_of("IfcWall") {
    println!("#{} {} args", wall.id, wall.args.len());
}
```

* `probe_step(bytes)` — cheap `ISO-10303-21;` magic probe.
* `parse_step(bytes) -> Result<StepFile>` / `parse_step_with_limits`.
* `StepFile { header, instances }` — typed `FILE_DESCRIPTION` /
  `FILE_NAME` / `FILE_SCHEMA` header plus the `#id → ParsedInstance
  { keyword, args }` instance graph.
* Complete ISO 10303-21 §10 parameter grammar: `$` unset, `*`
  derived, integers, reals (`1.`, `.5`, `-2.7E-3`), strings, `.ENUM.`
  literals, `"hex"` binaries, typed/SELECT parameters
  (`IFCLABEL('x')`), nested aggregates, and `#id` references —
  forward references included (the instance map is built before any
  resolution).
* String escape decoding per §6.4.3: `''` quote doubling, `\\`,
  `\X\HH`, `\X2\…\X0\` / `\X4\…\X0\` codepoint runs (terminator
  omissible at the closing quote), `\S\c` page-A shorthand, `\P?\`
  page selection. Raw UTF-8 in strings is tolerated (a common writer
  deviation), with Latin-1 fallback for stray high bytes.
* `/* … */` comments, multi-line records, multiple `DATA` sections.
* Graph utilities: `resolve`, `references_of`, cycle-safe
  `reachable_from`, `dangling_references`.
* DoS hardening via `StepLimits`: input-size, instance-count,
  aggregate-nesting-depth, and string-length caps.

Known Phase-1 limits: external-mapping (multi-keyword complex
entity) records — which IFC writers do not emit — are rejected with
a clear error.

## Phase 2 surface — typed schema layer

`oxideav_ifc::schema` layers the IFC 4 EXPRESS schema over the
positional instance graph for the core entity slice. Each entity's
attribute order is its inheritance chain concatenated **parent-first**
(EXPRESS digest §7), hand-transcribed from the staged
`IFC4_ADD2.exp` declarations.

```rust
let step = oxideav_ifc::parse_step(&bytes)?;
let model = oxideav_ifc::Model::from_step(&step);

let project = model.project().unwrap();          // the IfcProject root
for site in model.aggregated_children(project.id()) {
    for child in model.aggregated_children(*site) {     // building
        for storey in model.aggregated_children(*child) {
            for elem in model.contained_elements(*storey) {
                let e = model.typed(*elem).unwrap();
                println!("{} {:?}", e.keyword(), e.name());
            }
        }
    }
}
```

* `TypedEntity::new(&ParsedInstance)` — a borrowing view that resolves
  attribute **names** to positional `Value`s: `attr("GlobalId")`,
  plus `global_id()`, `name()`, `description()`,
  `object_placement()`, `representation()`, `predefined_type()`, and
  an in-order `attrs()` iterator. Truncated records (trailing
  optionals omitted) treat the missing slots as absent.
* `Model::from_step(&StepFile)` — classifies instances and folds
  `IfcRelAggregates` (composition: project→site→building→storey→space)
  + `IfcRelContainedInSpatialStructure` (element containment) into a
  navigable tree: `project()`, `aggregated_children(id)`,
  `contained_elements(id)`, `spatial_elements()`, `products()`.
* Typed slice: `IfcProject`, `IfcSite`/`IfcBuilding`/
  `IfcBuildingStorey`/`IfcSpace`, the common building elements
  (`IfcWall`(+`StandardCase`)/`IfcColumn`/`IfcBeam`/`IfcSlab`/
  `IfcDoor`/`IfcWindow`/`IfcOpeningElement`), `IfcLocalPlacement`,
  `IfcProductDefinitionShape`, `IfcGeometricRepresentationContext`,
  and the two structural relationships. Keywords outside the slice
  resolve to `None` — the positional Phase-1 view is always available.

## Phase 3 surface — tessellated geometry

`oxideav_ifc::geometry` turns the tessellation representation items a
product points at into plain triangle meshes. It is std-only (available
in `--no-default-features` builds); the `registry` decoder lifts the
result into a `Scene3D`.

```rust
let step = oxideav_ifc::parse_step(&bytes)?;
let mesh = oxideav_ifc::tessellate_item(&step, face_set_id)?;
println!("{} verts, {} tris", mesh.vertex_count(), mesh.triangle_count());
```

* `TriMesh { positions: Vec<[f64;3]>, triangles: Vec<[u32;3]> }` — a
  flat indexed mesh in the representation item's local coordinate space.
  Triangles are **zero-based** (the one-based STEP `CoordIndex` plus any
  optional `PnIndex` indirection are resolved during extraction).
* `tessellate_item(step, id)` — one `IfcTriangulatedFaceSet`
  (`Coordinates`, `Normals`, `Closed`, `CoordIndex`, `PnIndex`) or
  `IfcPolygonalFaceSet` (`Coordinates`, `Closed`, `Faces`, `PnIndex`,
  with each `IfcIndexedPolygonalFace` fan-triangulated) → a `TriMesh`.
  Both read their vertices from the shared `IfcCartesianPointList3D`
  reached through the `IfcTessellatedFaceSet.Coordinates` supertype
  attribute. Any other keyword → `GeometryError::Unsupported`.
* `mesh_from_shape_representation` / `mesh_from_product_shape` — the walk
  from a product's `Representation` down through
  `IfcProductDefinitionShape.Representations` →
  `IfcShapeRepresentation.Items`, merging the supported items and
  skipping unsupported styles (an axis/box/swept-solid representation
  alongside the tessellated body is the common case).
* `placement_transform(step, id)` → a [`Transform`] (a 3×3 linear part +
  translation) for an `IfcObjectPlacement`, folding the
  `IfcLocalPlacement.PlacementRelTo` chain from a leaf up to the absolute
  root. Each `IfcAxis2Placement3D(Location, Axis, RefDirection)` becomes
  an affine map whose rotation columns are the orthonormal placement axes
  derived by the EXPRESS `IfcBuildAxes` function (Z = normalise(`Axis`),
  X = `RefDirection` projected ⟂ to Z and normalised, Y = Z × X; absent
  `Axis`/`RefDirection` default to world Z/X). `TriMesh::transformed` /
  `TriMesh::transform` apply it to a mesh; cyclic `PlacementRelTo` chains
  are bounded by a depth cap.

With the `registry` feature, `IfcDecoder` walks every
`IfcProductDefinitionShape`, tessellates its supported body items, and
positions the result in **world space** via the owning product's
`IfcLocalPlacement` chain — one `Scene3D` node + mesh per tessellated
body. The product owning a shape is found by back-scanning for the
instance whose `Representation` references the shape (so geometry-bearing
products outside the typed schema slice, e.g. `IfcBuildingElementProxy`,
are still placed). The five fixture models decode to 8/24-vertex boxes
(cube proxy, column, colour cube) and the dense basin mesh; the column
body lands at its placed origin `(432, 288, 48)`. The swept-solid wall
model reports `Unsupported` (no tessellation present).

Still later in Phase 3: swept solids (`IfcExtrudedAreaSolid`), Breps
(`IfcFacetedBrep`), boolean results, mapped items, and EXPRESS WHERE-rule
validation.

## Cargo features

* `registry` *(default)* — pulls `oxideav-core` + `oxideav-mesh3d`
  and exposes `IfcDecoder` (a `Mesh3DDecoder`), the `make_decoder()`
  direct constructor, and `register_mesh3d(&mut Mesh3DRegistry)`
  (format id `"ifc"`, extension `.ifc`). The decoder probes the magic,
  fully parses + validates the exchange structure, and extracts every
  tessellated product shape into the `Scene3D`; a model with no
  tessellation (only swept solids / Breps) decodes to `Unsupported`.
* `--no-default-features` — standalone STEP parser only, std types,
  zero dependencies.

## Fixtures

`tests/fixtures/*.ifc` are five small IFC 4 sample models from the
buildingSMART **Sample-Test-Files** repository, licensed **CC-BY
4.0** by buildingSMART International (attribution preserved here);
they exercise tessellated face sets, a column, and a wall with
opening + window over the full spatial hierarchy.

## License

MIT — see [LICENSE](LICENSE). © 2026 Karpelès Lab Inc.
