# oxideav-ifc

[![CI](https://github.com/OxideAV/oxideav-ifc/actions/workflows/ci.yml/badge.svg)](https://github.com/OxideAV/oxideav-ifc/actions/workflows/ci.yml) [![crates.io](https://img.shields.io/crates/v/oxideav-ifc.svg)](https://crates.io/crates/oxideav-ifc) [![docs.rs](https://docs.rs/oxideav-ifc/badge.svg)](https://docs.rs/oxideav-ifc) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

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
| **3** | Geometry extraction into `oxideav-mesh3d::Scene3D`: tessellations (incl. face voids + colour maps), faceted Breps (incl. face holes), face/shell surface models, swept solids over a wide profile family (arbitrary ± voids, rectangle ± hollow, circle ± hollow, ellipse, composite; polyline / circle / indexed-poly-curve boundaries), boolean composition, mapped-item instancing, `IfcLocalPlacement` world-positioning, and surface-style materials | ✅ this release; advanced (curved) breps, boolean carving/intersection, arc segments + named profiles later |

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
* Geometric-representation-item primitives (`EntityKind::Geometry`):
  `IfcCartesianPoint` (`coordinates()`), `IfcDirection`
  (`direction_ratios()`), `IfcAxis2Placement2D`/`IfcAxis2Placement3D`
  (`location()`, `axis()`, `ref_direction()`), `IfcPolyline`
  (`points()`), and `IfcShapeRepresentation` (`context_of_items()`,
  `representation_identifier()`, `representation_type()`, `items()`).
  Each attribute order is the EXPRESS inheritance chain (IfcPoint /
  IfcPlacement / IfcCurve / IfcBoundedCurve / IfcRepresentation /
  IfcShapeModel supertypes add no serialised attributes). These let the
  product → shape → placement → point/direction chain be walked entirely
  by attribute name; they are typed but stay out of the spatial-model
  `products()` / `spatial_elements()` enumerations.
* Mapped-item instancing entities are likewise typed
  (`EntityKind::Geometry`): `IfcMappedItem` (`mapping_source()`,
  `mapping_target()`), `IfcRepresentationMap` (`mapping_origin()`,
  `mapped_representation()`), and `IfcCartesianTransformationOperator3D`
  (`/3DnonUniform`), so the mapped-item → representation-map →
  source-representation chain resolves by attribute name. The swept-solid
  helpers `IfcRevolvedAreaSolid` (`SweptArea`, `Position`, `Axis`,
  `Angle`) and `IfcAxis1Placement` (`Location`, `Axis`) are typed too.

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
* Tessellated faces may carry holes: `IfcIndexedPolygonalFaceWithVoids`
  inner loops are left open, triangulated hole-aware on the face plane.
* `tessellate_item` additionally evaluates the **faceted boundary
  representation** family, whose faces are explicit polygons of
  `IfcCartesianPoint` references rather than indices into a shared list:
  `IfcFacetedBrep` / `IfcFacetedBrepWithVoids` (`Outer` + optional
  `Voids` `IfcClosedShell`s), `IfcFaceBasedSurfaceModel` (`FbsmFaces`),
  and `IfcShellBasedSurfaceModel` (`SbsmBoundary`, the `IfcShell` SELECT
  of `IfcClosedShell` / `IfcOpenShell`). Each shell
  (`IfcConnectedFaceSet.CfsFaces`) is walked to its `IfcFace`s; the
  `IfcFaceOuterBound` loop is the boundary and every other
  `IfcFaceBound` is an inner **hole** — the loops are projected onto the
  face plane (Newell normal) and triangulated hole-aware (bridge + ear
  clip), so concave faces and face holes mesh exactly; convex
  single-bound faces keep the fan fast path. The shared vertex table is
  de-duplicated by `IfcCartesianPoint` id, so a point referenced by
  several loops (the §8.8.3.18 invariant guarantees at least three)
  becomes one mesh vertex. Per-bound `Orientation` flags are not
  applied; advanced (curved) breps and `IfcFaceSurface` faces remain
  `Unsupported`.
* `tessellate_item` also sweeps the **extruded area solid**
  `IfcExtrudedAreaSolid` (`SweptArea`, `Position`, `ExtrudedDirection`,
  `Depth`) into a closed prism — hole-aware caps (ear-clipped, so
  concave profiles are exact), one side-wall quad per boundary edge
  (outer and hole rings alike), the optional `Position`
  `IfcAxis2Placement3D` re-placing the whole solid — and revolves the
  **revolved area solid** `IfcRevolvedAreaSolid` (`SweptArea`,
  `Position`, `Axis`, `Angle`) about its `IfcAxis1Placement` line by
  `Angle` radians (Rodrigues' rotation, 48 segments per full turn; a
  full 2π wraps closed, a partial sweep gets hole-aware end caps).
* The shared **profile family** behind both sweeps
  (`profile_area`/`profile_areas`): `IfcArbitraryClosedProfileDef` and
  `IfcArbitraryProfileDefWithVoids` (inner curves become hole rings),
  `IfcRectangleProfileDef` / `IfcRectangleHollowProfileDef`
  (`WallThickness` inset; fillet radii not yet applied),
  `IfcCircleProfileDef` / `IfcCircleHollowProfileDef` (48-segment
  circles / annuli), `IfcEllipseProfileDef`, and
  `IfcCompositeProfileDef` (each component swept independently and
  merged). Boundary curves may be an `IfcPolyline` (duplicated closing
  point dropped), a full `IfcCircle`, or an `IfcIndexedPolyCurve` over
  an `IfcCartesianPointList2D` (`$` segments or `IfcLineIndex` runs
  with shared junctions; `IfcArcIndex` remains `Unsupported`). All
  rings are normalised counter-clockwise; parameterised profiles apply
  their optional 2-D `Position`.
* `tessellate_item` composes **boolean results** (`IfcBooleanResult` /
  `IfcBooleanClippingResult`, §8.8.3.5) at the surface-mesh level:
  `UNION` merges the operand boundaries, `DIFFERENCE` emits the first
  operand as authored (half-space carving pends the
  `IfcHalfSpaceSolid.AgreementFlag` side-convention documentation), and
  `INTERSECTION` surfaces as `Unsupported`. Operands recurse (clipping
  chains nest) under a shared depth cap.
* `tessellate_item` also evaluates the **mapped item**
  `IfcMappedItem` (`MappingSource`, `MappingTarget`) — the inserted
  instance of a reusable source representation. `MappingSource` is an
  `IfcRepresentationMap(MappingOrigin, MappedRepresentation)`: the source
  `IfcShapeRepresentation` is meshed in its own frame, lifted into the
  `MappingOrigin` `IfcAxis2Placement`, then placed by the `MappingTarget`
  `IfcCartesianTransformationOperator` (2D / 2DnonUniform / 3D /
  3DnonUniform). The operator's column basis is the EXPRESS `IfcBaseAxis`
  derivation (`U1` = normalise(`Axis1`), `U2` = `Axis2` ⟂ `U1`, `U3` =
  `U1`×`U2`, world-axis defaults), each column scaled by its uniform or
  per-axis `Scale`(`/Scale2`/`Scale3`) and translated by `LocalOrigin`.
  Mapped items may **nest** (a source representation can contain further
  mapped items); recursion is bounded by a depth cap and a
  self-referential map surfaces `Unsupported`.
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
`IfcProductDefinitionShape`, tessellates its supported body items —
**one primitive per representation item** — and positions the result in
**world space** via the owning product's `IfcLocalPlacement` chain. The
scene node is named after the owning product's `Name` (the column
fixture decodes as "Column #1"); the product is found by back-scanning
for the instance whose `Representation` references the shape, so
geometry-bearing products outside the typed schema slice
(e.g. `IfcBuildingElementProxy`) are still placed and named.
Presentation carries over: an `IfcStyledItem` → `IfcSurfaceStyle`
shading colour becomes the primitive's `Material` (`base_color` with
alpha `1 − Transparency`, named from the style, deduplicated per
style), and an `IfcIndexedColourMap` on a triangulated face set becomes
per-vertex colours (vertices split per face so each triangle keeps its
flat colour — the individual-colors fixture decodes with its
red/green/yellow faces). The five fixture models decode to their boxes
and the dense basin mesh; the column body lands at its placed origin
`(432, 288, 48)` and the wall model decodes to its wall body, opening
and window prisms.

`length_unit_scale(&StepFile)` resolves metres-per-model-unit from the
project's `IfcUnitAssignment` (`IfcSIUnit` prefixes and
`IfcConversionBasedUnit` factors — the wall fixture is millimetres, the
column fixture inches); the decoder itself keeps raw model units.

Still later in Phase 3: the remaining swept solids
(`IfcSurfaceCurveSweptAreaSolid` / the tapered subtypes /
`IfcSweptDiskSolid`), `IfcArcIndex` poly-curve segments + named
parameterised profiles (I/L/T/U/Z/C), advanced (curved) breps
(`IfcAdvancedBrep` / `IfcFaceSurface`), boolean half-space carving +
intersection, and EXPRESS WHERE-rule validation.

## Cargo features

* `registry` *(default)* — pulls `oxideav-core` + `oxideav-mesh3d`
  and exposes `IfcDecoder` (a `Mesh3DDecoder`), the `make_decoder()`
  direct constructor, and `register_mesh3d(&mut Mesh3DRegistry)`
  (format id `"ifc"`, extension `.ifc`). The decoder probes the magic,
  fully parses + validates the exchange structure, and extracts every
  tessellated / faceted-Brep / swept-solid / boolean-result /
  mapped-item product shape into the `Scene3D` — product-named nodes,
  per-item primitives, surface-style materials, per-face colour maps;
  a model with no extractable geometry (only curved/advanced breps)
  decodes to `Unsupported`.
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
