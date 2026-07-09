# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added

- Phase 2 (unit resolution): new public `length_unit_scale(&StepFile)`
  — metres per model length unit, walked from
  `IfcProject.UnitsInContext` → `IfcUnitAssignment.Units` (§8.11.3.11:
  at most one unit per unit type) → the `.LENGTHUNIT.` unit. Supports
  `IfcSIUnit` (`.METRE.` with the full ISO 80000 decimal-prefix
  multiplier table) and `IfcConversionBasedUnit`(`WithOffset`) through
  its `IfcMeasureWithUnit` factor over a recursively resolved SI base.
  Returns `None` when unresolvable; the decoder keeps raw model units
  (no silent rescale). Typed entries for `IfcUnitAssignment`,
  `IfcSIUnit`, `IfcConversionBasedUnit`, `IfcMeasureWithUnit`. 4 new
  tests: wall fixture millimetres (10⁻³), column fixture inches
  (0.0254 via the real conversion-based chain), synthetic
  foot-through-metre resolution, unitless model → `None`.

- Phase 3 (polygonal-face voids + product-named nodes):
  - `IfcIndexedPolygonalFaceWithVoids.InnerCoordIndices` (§8.8.3.39:
    each inner list is one hole loop of the planar face) is now
    evaluated — polygonal face-set faces with voids leave their holes
    open through the shared plane-projection + bridge/ear-clip
    triangulator (extracted as `triangulate_face_3d`, shared with the
    Brep `IfcFace` walk). Concave plain polygonal faces are ear-clipped
    too (fan spill fixed); convex faces keep the fan output unchanged.
  - The registry decoder names each scene node after the owning
    product's `IfcRoot.Name` (else `KEYWORD#id`, else the shape's
    `#id`), so a decoded `Scene3D` reads as "Column #1" rather than
    anonymous shape ids.
  - 3 new tests (voided polygonal face area 12 + void-avoidance
    centroids, concave polygonal L-face area 3, product-named node on
    the column fixture).

- Phase 3 (Brep face holes + concave faces): `IfcFace` inner bounds are
  now real holes. The outer bound is identified by keyword
  (`IfcFaceOuterBound`, else the first bound); every remaining
  `IfcFaceBound` loop is projected — together with the outer loop —
  onto the face plane (Newell normal, robust for slightly non-planar
  loops) and triangulated hole-aware through the shared bridge +
  ear-clip machinery, so the hole area stays open instead of being
  covered. Convex single-bound faces keep the historical fan fast path
  (identical output); concave single-bound faces are ear-clipped,
  fixing fan spill outside the boundary. Per-bound `Orientation` flags
  remain unapplied (their normative description is not in the staged
  set). 3 new geometry tests with exact area-sum assertions (holed
  face 8 − 0.5 with hole-avoidance centroids, concave L-face area 3,
  holed face on a tilted x = z plane with in-plane areas 200 − 4√2).

- Phase 3 (presentation styling): the registry decoder now emits **one
  primitive per representation item** (new public
  `meshed_items_from_product_shape`) and carries the file's styling
  onto each primitive:
  - `IfcStyledItem` → `IfcSurfaceStyle` (directly or through the
    `IfcPresentationStyleAssignment` wrapper) →
    `IfcSurfaceStyleShading`/`IfcSurfaceStyleRendering.SurfaceColour`
    (`IfcColourRgb`) becomes the primitive's `Material.base_color`,
    with alpha `1 − Transparency`; materials are deduplicated per
    `IfcSurfaceStyle` and named from the style `Name`.
  - `IfcIndexedColourMap` on an `IfcTriangulatedFaceSet` becomes
    per-vertex colours: vertices are split per face (non-indexed
    primitive) so each triangle keeps its flat `ColourIndex` colour;
    the optional `Opacity` supplies alpha and out-of-range / missing
    rows fall back to white. The tessellation-with-individual-colors
    fixture now decodes with its red/green/yellow faces.
  - 2 new end-to-end registry tests (colour-map fixture with per-face
    colour assertions incl. the 11-of-12 `ColourIndex` fallback;
    synthetic styled extruded box asserting material name, RGB and
    alpha 0.75).

- Phase 3 (boolean composition): `tessellate_item` now evaluates
  `IfcBooleanResult` / `IfcBooleanClippingResult` (`Operator`,
  `FirstOperand`, `SecondOperand`, ISO 16739 §8.8.3.5) at the
  surface-mesh level, so CSG / Clipping body representations flow
  through the product-shape walk instead of failing as unsupported.
  - `UNION` merges the two operand boundary meshes (a boundary superset
    of the regularised union); one unsupported operand is tolerated if
    the other produced geometry.
  - `DIFFERENCE` emits the first operand's boundary as authored —
    the subtracted volume is **not yet carved**: exact half-space
    clipping needs the `IfcHalfSpaceSolid.AgreementFlag` side
    convention, which is not in the staged documentation set (docs gap
    filed). Clipping chains (a clipping result as first operand of
    another) nest, bounded by the shared recursion depth cap.
  - `INTERSECTION` is surfaced as `Unsupported` — no boundary-level
    approximation is defensible.
  - 6 new geometry tests (union merge + re-indexing, clipping emits
    first operand bit-identical, two-level clipping chain, intersection
    surfacing, cyclic-operand termination, CSG shape-representation
    walk); Phase-2 typed entries for `IfcBooleanResult`,
    `IfcBooleanClippingResult`, `IfcHalfSpaceSolid`,
    `IfcPolygonalBoundedHalfSpace`, `IfcPlane`.

- Phase 3 (indexed poly curves + composite profiles): profile boundary
  curves may now be `IfcIndexedPolyCurve` over an
  `IfcCartesianPointList2D` — either the whole point list in order (`$`
  `Segments`) or `IfcLineIndex` segments whose shared junction points
  are emitted once (the EXPRESS `Consecutive` WHERE rule); `IfcArcIndex`
  (three-point arc) segments remain `Unsupported` pending their
  semantics doc. `IfcCompositeProfileDef` resolves to the union of its
  component profiles, each swept independently and merged (extrusion and
  revolution both; the EXPRESS `NoRecursion` rule falls out naturally as
  `Unsupported`). 4 new geometry tests (segment-less polycurve,
  junction-sharing line segments, arc surfacing, two-component composite
  with per-component position checks); Phase-2 typed entries for
  `IfcIndexedPolyCurve`, `IfcCartesianPointList2D`,
  `IfcCompositeProfileDef`.

- Phase 3 (profile holes + hole-aware caps): swept-solid profiles now
  resolve to a full `ProfileArea` (outer ring + hole rings), and the
  extrusion / revolution caps are triangulated by hole bridging + ear
  clipping instead of a convex fan.
  - New profile kinds: `IfcArbitraryProfileDefWithVoids` (`InnerCurves`
    become hole rings), `IfcCircleHollowProfileDef` (annulus of `Radius`
    / `Radius − WallThickness`), and `IfcRectangleHollowProfileDef`
    (rectangular tube, `WallThickness` inset; the optional fillet radii
    are not yet applied). Attribute orders transcribed from
    `IFC4_ADD2.exp`; EXPRESS wall-thickness WHERE bounds enforced.
  - Hole side walls are emitted with inward winding for both extrusion
    and revolution; ring orientation is normalised counter-clockwise
    regardless of how the file authored its curves.
  - Bug fix: concave arbitrary profiles previously fan-triangulated
    their caps from vertex 0, spilling triangles outside the profile
    boundary; the ear-clipped caps now cover exactly the profile area
    (regression-tested with an L-profile whose cap area is asserted).
  - 5 new geometry unit tests with vertex-level / area-sum assertions
    (L-profile exact cap area, rectangular tube cap area 12 +
    hole-avoidance centroids, 48-gon annulus area, voided arbitrary
    profile area 15, hollow quarter-revolution counts). Also typed in
    the Phase-2 schema layer (`IfcArbitraryProfileDefWithVoids`,
    `IfcCircleHollowProfileDef`, `IfcRectangleHollowProfileDef`).

- Phase 3 (parameterised-profile widening): `profile_ring` now resolves
  `IfcCircleProfileDef` (`Radius`), `IfcEllipseProfileDef` (`SemiAxis1`
  along X, `SemiAxis2` along Y), and `IfcArbitraryClosedProfileDef`
  outer curves that are a full `IfcCircle` (`IfcConic.Position` +
  `Radius`), so cylinders / elliptic prisms / tori extrude and revolve
  through the same swept-solid paths. Attribute orders transcribed from
  `IFC4_ADD2.exp` (`IfcParameterizedProfileDef.Position` is the shared
  optional `IfcAxis2Placement2D`, applied to every generated ring
  point). Circular boundaries are approximated with 48
  counter-clockwise segments (`CIRCLE_SEGMENTS`, matching the revolve
  density). 5 new geometry unit tests (cylinder counts/radius check,
  positioned circle, ellipse equation, circle outer curve, full-turn
  torus with a vertex-level torus-membership assertion). Also typed in
  the Phase-2 schema layer (`IfcExtrudedAreaSolid`,
  `IfcArbitraryClosedProfileDef`, `IfcRectangleProfileDef`,
  `IfcCircleProfileDef`, `IfcEllipseProfileDef`, `IfcCircle`).

- Phase 3 (revolved-swept-solid slice): `tessellate_item` now revolves
  `IfcRevolvedAreaSolid` (`SweptArea`, `Position`, `Axis`, `Angle`) into
  a tessellated surface of revolution, so revolved bodies flow through
  the same `mesh_from_product_shape` / registry-decoder path into a
  `Scene3D`.
  - The 2-D profile ring (in the `Position` XY-plane, z = 0) is stepped
    through a fan of angular positions about the `Axis`
    `IfcAxis1Placement(Location, Axis)` line by `Angle` radians, via
    Rodrigues' rotation (`rotate_about_axis`). A full 2π revolution wraps
    closed (side walls only); a partial sweep adds fan end caps on the
    open first/last rings. Angular resolution is
    `REVOLVE_SEGMENTS_PER_TURN`(48) scaled by `Angle / 2π`. The optional
    `Position` `IfcAxis2Placement3D` re-places the solid. Attribute and
    derived-axis (`IfcAxis1Placement.Z` default world +Z) orders
    transcribed from `IFC4_ADD2.exp`.
  - Reuses the existing `profile_ring` (so arbitrary-polyline and
    rectangle profiles revolve); `IfcRevolvedAreaSolidTapered` /
    non-rectangle parameterised + curved-curve profiles remain
    `Unsupported`.
  - 6 new geometry unit tests (axis rotation, axis-1 placement defaults,
    full-turn closed wrap, quarter-turn end caps, zero-angle rejection,
    product-shape walk). Also typed in the Phase-2 schema layer
    (`IfcRevolvedAreaSolid`, `IfcAxis1Placement`).
- Phase 3 (mapped-item slice): `tessellate_item` now evaluates
  `IfcMappedItem` (`MappingSource`, `MappingTarget`), so reused /
  instanced representations flow through the same
  `mesh_from_shape_representation` / `mesh_from_product_shape` /
  registry-decoder path into a `Scene3D`.
  - `MappingSource` → `IfcRepresentationMap(MappingOrigin,
    MappedRepresentation)`: the source `IfcShapeRepresentation` is meshed
    in its own frame, lifted into the `MappingOrigin` `IfcAxis2Placement`,
    then placed by `MappingTarget`.
  - `MappingTarget` → `transformation_operator`: resolves
    `IfcCartesianTransformationOperator2D` / `…2DnonUniform` / `…3D` /
    `…3DnonUniform` (`Axis1`, `Axis2`, `LocalOrigin`, `Scale`[, `Axis3`]
    [, `Scale2`, `Scale3`]) to a `Transform`. The orthonormal axis basis
    is the EXPRESS `IfcBaseAxis` derivation (`base_axes`: `U1` =
    normalise(`Axis1`) default world X, `U2` = `Axis2` ⟂ `U1` default
    world Y, `U3` = `U1`×`U2`); each column is scaled by its (uniform or
    per-axis) `Scale` and translated by `LocalOrigin`. Attribute orders
    transcribed from `IFC4_ADD2.exp`.
  - Mapped items may nest (a source representation can itself contain
    `IfcMappedItem`s); recursion is bounded by `MAX_MAP_DEPTH` and a
    self-referential map surfaces `Unsupported` rather than looping.
  - 10 new geometry unit tests (identity / translation / uniform +
    non-uniform scale / rotated axes / mapping-origin fold / nested
    composition / shape-rep walk / self-reference bounding / 2-D
    operator).
- Phase 2 (mapped-item typing): `IfcMappedItem`, `IfcRepresentationMap`,
  and the 3-D Cartesian transformation operators are now in the typed
  schema layer (`EntityKind::Geometry`), with `TypedEntity` accessors
  `mapping_source` / `mapping_target` / `mapping_origin` /
  `mapped_representation`. Attribute orders transcribed from
  `IFC4_ADD2.exp`; 1 new schema unit test walking the mapped-item chain
  by attribute name.
- Phase 3 (extruded-swept-solid slice): `tessellate_item` now sweeps
  `IfcExtrudedAreaSolid` (`SweptArea`, `Position`, `ExtrudedDirection`,
  `Depth`) into a closed prism, so extruded bodies flow through the same
  `mesh_from_product_shape` / registry-decoder path into a `Scene3D`.
  - The 2-D profile is resolved to its outer ring from
    `IfcArbitraryClosedProfileDef` (`OuterCurve` an `IfcPolyline`; a
    duplicated closing point is dropped) or `IfcRectangleProfileDef`
    (centred `XDim`×`YDim`, optional 2-D `Position` applied via the
    EXPRESS `IfcBuild2Axes` derivation). Attribute orders transcribed
    from the staged `IFC4_ADD2.exp` declarations.
  - The ring is swept along `Depth · normalise(ExtrudedDirection)` into a
    bottom cap, an offset top cap, and one side-wall quad per profile
    edge; the optional `Position` `IfcAxis2Placement3D` re-places the
    whole solid (§8.8.3.15: the direction and profile are in the position
    coordinate system).
  - The wall fixture (body / opening / window are polyline-profile
    extrusions) now decodes to three 8-vertex boxes instead of reporting
    `Unsupported`. Revolved / surface-curve / tapered solids,
    non-rectangle parameterised + curved-curve profiles, and `Voids`
    (profile-hole) subtraction remain `Unsupported`.
  - New `GeometryError::BadProfile` variant; 9 new geometry unit tests +
    2 wall-fixture tests; the registry wall test now asserts a 3-box
    scene.
- Phase 3 (faceted-Brep slice): `tessellate_item` now evaluates the
  faceted boundary-representation family in addition to the index-based
  tessellations, so faceted-Brep bodies flow through the same
  `mesh_from_product_shape` / registry-decoder path into a `Scene3D`.
  - `IfcFacetedBrep` / `IfcFacetedBrepWithVoids` (`IfcManifoldSolidBrep.
    Outer : IfcClosedShell` + optional `Voids`), `IfcFaceBasedSurfaceModel`
    (`FbsmFaces`) and `IfcShellBasedSurfaceModel` (`SbsmBoundary`, the
    `IfcShell` SELECT of `IfcClosedShell` / `IfcOpenShell`).
  - Each shell (`IfcConnectedFaceSet.CfsFaces : SET OF IfcFace`) is walked
    to its faces; every face's outer `IfcFaceBound` / `IfcFaceOuterBound`
    resolves to an `IfcPolyLoop` (`Polygon : LIST [3:?] OF
    IfcCartesianPoint`) which is fan-triangulated. Vertices are pooled
    in a `HashMap`-deduplicated table keyed by `IfcCartesianPoint` id, so
    a point shared by several loops (§8.8.3.18 guarantees ≥3) becomes one
    mesh vertex. Attribute orders transcribed from `IFC4_ADD2.exp`.
  - Per-bound `Orientation` and `Voids` boolean subtraction are not yet
    applied (outer surface meshed as authored); advanced (curved) breps
    (`IfcAdvancedBrep` / `IfcFaceSurface`) stay `Unsupported`.
  - 9 new geometry unit tests (tetra point-dedup, quad fan, outer-bound
    preference, `…WithVoids`, face/shell surface models, degenerate-loop
    rejection, Brep-via-`IfcShapeRepresentation`).
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
