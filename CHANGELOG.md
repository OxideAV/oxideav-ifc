# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added

- Phase 3 (faceted-Brep slice): `geometry::tessellate_item` now evaluates
  the explicit faceted boundary-representation family in addition to the
  two tessellated face sets.
  - `IfcFacetedBrep` / `IfcFacetedBrepWithVoids` (via
    `IfcManifoldSolidBrep.Outer`), `IfcFaceBasedSurfaceModel`
    (`FbsmFaces`) and `IfcShellBasedSurfaceModel` (`SbsmBoundary`) →
    `TriMesh`. Each `IfcConnectedFaceSet` shell's `IfcFace`s are
    triangulated from the `IfcFaceOuterBound` (falling back to a sole
    `IfcFaceBound`) `IfcPolyLoop`, fan-triangulated with the
    `IfcFaceBound.Orientation` `.F.` flag reversing the winding.
  - Directly-referenced `IfcCartesianPoint`s are interned by `#id` so a
    Brep vertex shared across faces is emitted once. Attribute layouts
    transcribed from the staged `IFC4_ADD2.exp`
    (`IfcManifoldSolidBrep`, `IfcConnectedFaceSet`, `IfcFace`,
    `IfcFaceBound`, `IfcPolyLoop`, face-/shell-based surface models).
  - Inner face bounds (holes) and `IfcFacetedBrepWithVoids.Voids` are not
    subtracted in this slice. Through the unchanged
    `mesh_from_shape_representation` / product-shape walk, the `registry`
    decoder now lifts faceted-Brep bodies into the `Scene3D`.

- Phase 2 (geometry-primitive slice): the core geometric-representation-
  item entities are now in the typed schema layer
  (`oxideav_ifc::schema`).
  - New `EntityKind::Geometry` plus `SCHEMA` entries for
    `IfcCartesianPoint` (`Coordinates`), `IfcDirection`
    (`DirectionRatios`), `IfcAxis2Placement2D` (`Location`,
    `RefDirection`), `IfcAxis2Placement3D` (`Location`, `Axis`,
    `RefDirection`), `IfcPolyline` (`Points`), and
    `IfcShapeRepresentation` (`ContextOfItems`,
    `RepresentationIdentifier`, `RepresentationType`, `Items`).
    Attribute orders transcribed from the staged `IFC4_ADD2.exp`
    inheritance chains (IfcPoint / IfcPlacement / IfcCurve /
    IfcBoundedCurve / IfcRepresentation / IfcShapeModel supertypes add
    no serialised attributes).
  - `TypedEntity` accessors over the new slice: `coordinates`,
    `direction_ratios`, `location`, `axis`, `ref_direction`, `points`,
    `items`, `context_of_items`, `representation_identifier`,
    `representation_type` (integer literals where REAL is expected are
    widened via `Value::as_number`; `$` slots and missing attributes
    yield `None`).
  - Geometry-kind entities are typed but never enter the spatial-model
    `products()` / `spatial_elements()` enumerations.
  - 6 new schema unit tests (point/direction typing, 2-D + 3-D axis
    placements, polyline point lists, shape-representation resolution,
    spatial-model exclusion) + 2 fixture integration tests walking the
    column's product→shape→placement chain to its placed origin
    `(432, 288, 48)` and the wall's `Axis` polyline — all through the
    typed layer.
- Phase 3 (placement slice): `IfcLocalPlacement` world-positioning.
  - `Transform { cols, translation }` — a 3-D affine map (column-major
    3×3 linear part + translation) with `apply` / `compose` and an
    `IDENTITY` constant.
  - `placement_transform(step, id)` — folds an `IfcObjectPlacement`'s
    `IfcLocalPlacement.PlacementRelTo` chain (leaf → absolute root) into
    one world transform. Each `IfcAxis2Placement3D(Location, Axis,
    RefDirection)` resolves its orthonormal rotation columns through the
    EXPRESS `IfcBuildAxes` derivation (Z = normalise(`Axis`),
    X = `RefDirection` projected ⟂ to Z and normalised, Y = Z × X;
    `IfcFirstProjAxis` / `IfcNormalise` / `IfcCrossProduct` /
    `IfcDotProduct` implemented for the 3-D direction case). Absent
    `Axis`/`RefDirection` default to world Z/X; cyclic `PlacementRelTo`
    chains are bounded by a depth cap.
  - `TriMesh::transformed` / `TriMesh::transform` — map a mesh's
    vertices through a `Transform`.
  - `registry` decoder now positions each tessellated body in world
    space: the owning product (found by back-scanning for the instance
    whose `Representation` references the shape, so proxy products
    outside the typed slice are covered) supplies the placement chain.
    The column fixture's body lands at its placed origin `(432,288,48)`.
  - 11 placement unit tests (default + explicit + rotated axes,
    `RefDirection` orthogonalisation, chain composition, rotation-then-
    translation, cycle bounding, `TriMesh` transform) + a fixture test
    asserting the column's world coordinates + a decoder test asserting
    the placed scene extents.
- Phase 3 (tessellation slice): geometry extraction
  (`oxideav_ifc::geometry`) turning the tessellation representation
  items a product points at into plain triangle meshes. Std-only, so
  available in `--no-default-features` builds.
  - `TriMesh { positions, triangles }` — flat indexed mesh in local
    coordinate space; triangles are zero-based with the one-based STEP
    `CoordIndex` and any optional `PnIndex` indirection resolved.
  - `tessellate_item` — one `IfcTriangulatedFaceSet`
    (`Coordinates`, `Normals`, `Closed`, `CoordIndex`, `PnIndex`) or
    `IfcPolygonalFaceSet` (`Coordinates`, `Closed`, `Faces`, `PnIndex`,
    each `IfcIndexedPolygonalFace` fan-triangulated) → `TriMesh`, both
    reading the shared `IfcCartesianPointList3D` via the
    `IfcTessellatedFaceSet.Coordinates` supertype attribute. Other
    keywords → `GeometryError::Unsupported`.
  - `mesh_from_shape_representation` / `mesh_from_product_shape` — walk
    a product's `Representation` →
    `IfcProductDefinitionShape.Representations` →
    `IfcShapeRepresentation.Items`, merging supported items and
    skipping unsupported geometry styles.
  - `registry` decoder now builds a real `Scene3D`: one node + mesh per
    tessellated `IfcProductDefinitionShape`. Vertices are emitted in
    local space (placement transforms are a later slice); a model with
    no tessellation reports `Unsupported`.
  - 8 geometry unit tests (incl. `PnIndex` indirection, fan
    triangulation, out-of-range / zero-index rejection) + 5 fixture
    integration tests over the real buildingSMART meshes + updated
    registry-decoder tests asserting populated scenes.
- Phase 2: EXPRESS schema typing (`oxideav_ifc::schema`) over the
  Phase-1 positional instance graph, for the core IFC 4 entity slice.
  - `EntitySchema` / `SCHEMA` / `schema_of` — static table mapping
    each entity keyword to its EXPRESS-serialisation attribute names
    (inheritance chain concatenated parent-first), hand-transcribed
    from the staged `IFC4_ADD2.exp` declarations.
  - `TypedEntity` — borrowing view resolving attribute names to
    positional `Value`s (`attr`, `attrs`, `global_id`, `name`,
    `description`, `object_placement`, `representation`,
    `predefined_type`), tolerant of truncated trailing optionals.
  - `Model` — folds `IfcRelAggregates` +
    `IfcRelContainedInSpatialStructure` into a navigable
    project → site → building → storey → space → element tree
    (`project`, `aggregated_children`, `contained_elements`,
    `spatial_elements`, `products`).
  - Slice: `IfcProject`, the four spatial-structure elements, the
    common building elements (wall/column/beam/slab/door/window/
    opening), `IfcLocalPlacement`, `IfcProductDefinitionShape`,
    `IfcGeometricRepresentationContext`, and the two relationships.
  - 11 schema unit tests + 3 typed-model integration tests over the
    staged IFC 4 fixtures (full spatial hierarchy, column-in-site,
    direct containment).
- Phase 1: clean-room STEP physical-file (ISO 10303-21) parser —
  `probe_step` / `parse_step` / `parse_step_with_limits` producing a
  typed `StepFile` (typed `FILE_DESCRIPTION` / `FILE_NAME` /
  `FILE_SCHEMA` header + `#id → ParsedInstance` instance graph).
- Full Part 21 parameter grammar: `$` / `*` placeholders, integers,
  reals, strings with the §6.4.3 escape directives (`''`, `\\`,
  `\X\`, `\X2\`/`\X4\` runs, `\S\`, `\P?\`), `.ENUM.` literals,
  `"hex"` binaries, typed/SELECT parameters, nested aggregates, and
  forward-referencing `#id` entity references.
- Graph utilities: `resolve`, `references_of`, cycle-safe
  `reachable_from`, `dangling_references`.
- DoS hardening (`StepLimits`): input-size, instance-count,
  nesting-depth, and string-length caps.
- `registry` feature (default-on): `IfcDecoder` implementing
  `oxideav_mesh3d::Mesh3DDecoder` (magic probe + structure
  validation), `make_decoder()`, and `register_mesh3d()` under format
  id `"ifc"` / extension `.ifc`.
- Test suite: 31 unit tests + 10 integration tests, including the
  five buildingSMART IFC 4 sample fixtures (CC-BY 4.0) parsed with
  exact instance counts, schema assertions, and spot-checked
  entities.
