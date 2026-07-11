# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added

- Phase 4 (georeferencing): new [`geo`] module.
  - `map_conversion(step)` — the model's `IfcMapConversion` binding
    (the one whose `SourceCRS` is the
    `IfcGeometricRepresentationContext`), with the `IfcProjectedCRS`
    target resolved (EPSG name, datums, projection/zone, `MapUnit`
    resolvable through `named_unit_scale`). `MapConversion::to_map`
    applies the planar similarity the attributes describe — rotation
    by the normalised (`XAxisAbscissa`, `XAxisOrdinate`) x-axis
    direction, `Scale` on the planar components, translation to
    (`Eastings`, `Northings`, `OrthogonalHeight`); heights translate
    **unscaled** (the staged schema text states no height-scaling
    rule). CRS-to-CRS conversions resolve by id but are not the model
    binding.
  - `site_geolocation(step)` — the first `IfcSite` with
    `RefLatitude`/`RefLongitude`, converted from
    `IfcCompoundPlaneAngleMeasure` (degrees, minutes, seconds[,
    millionths] with consistent sign) to decimal degrees via the
    public `compound_angle_degrees`; `RefElevation` carried along.
  - Schema entries for `IfcMapConversion` / `IfcProjectedCRS`; 5 unit
    tests (3-4-5 rotation + origin/step mapping, identity + planar
    scale defaults, non-model-binding filter, compound-angle
    conversions incl. negative measures, ungeoreferenced model).

- Phase 4 (material associations): new [`material`] module resolving
  the full `IfcMaterialSelect` family.
  - `Model` folds `IfcRelAssociatesMaterial` (object →
    `RelatingMaterial`, first edge wins): `material_of(id)` — a
    directly associated material wins, else the material associated
    with the object's **type** applies (occurrence-overrides-type) —
    and `material_assignment(id)` for the resolved view.
  - `material_assignment(step, id)` → `MaterialAssignment`:
    `IfcMaterial` (name/description/category), `IfcMaterialList`,
    `IfcMaterialLayer(WithOffsets)` + `IfcMaterialLayerSet` (typed
    layers with material / thickness / IsVentilated / category /
    priority and the derived `total_thickness()` per
    `IfcMlsTotalThickness`) + `IfcMaterialLayerSetUsage` (direction /
    sense / offset), `IfcMaterialProfile(WithOffsets)` +
    `IfcMaterialProfileSet` (profile-def `#id`s into the geometry
    layer's profile family) + `IfcMaterialProfileSetUsage(Tapering)`
    (cardinal point), and `IfcMaterialConstituent(Set)` (fractions).
    `MaterialAssignment::name()` gives the headline display name.
  - Schema entries for the 14 material entities +
    `IfcRelAssociatesMaterial`, transcribed from `IFC4_ADD2.exp`.
  - 6 unit tests (plain material, cavity-wall layer-set usage with
    total-thickness and ventilation flags, list + constituent set,
    profile-set usage, occurrence-beats-type precedence, non-material
    target) + a basin-fixture integration test (the occurrence
    inherits Ceramic through its sanitary-terminal type).

- Phase 4 (unit engine): the §8.11.3.11 project-unit walk is
  generalised over a per-dimension table, adding public
  `area_unit_scale` (m²), `volume_unit_scale` (m³), `mass_unit_scale`
  (**kilograms** — the SI name is `.GRAM.`, so an unprefixed gram
  model yields 10⁻³) and `time_unit_scale` (seconds) alongside the
  existing length / plane-angle scales, plus `named_unit_scale(step,
  unit_id, unit_type)` for resolving one `IfcSIUnit` /
  `IfcConversionBasedUnit(WithOffset)` chain directly.
  - A **prefixed** `.SQUARE_METRE.` / `.CUBIC_METRE.` SI unit resolves
    to `None`: whether the prefix scales the base length (mm² = 10⁻⁶
    m²) or the derived unit (10⁻³ m²) is not stated by the staged
    schema text, so no guess is made (docs gap noted); conversion-based
    area/volume chains resolve normally.
  - `Quantity::si_scale` / `Quantity::si_value` — the per-quantity
    `IfcPhysicalSimpleQuantity.Unit` override wins (validated against
    the dimension the quantity kind's WHERE rule requires — a
    mismatched override refuses rather than mis-scales); otherwise the
    model default applies; counts are dimensionless (scale 1).
  - 4 new unit tests (model-default scaling across all six kinds,
    override-beats-default + WR21 mismatch refusal, gram/pound mass
    anchoring, prefixed-area refusal + square-foot conversion chain).

- Phase 4 (property + quantity sets): new [`props`] module — the typed
  semantic-data surface over the Phase-2 schema layer.
  - `Model` now folds `IfcRelDefinesByProperties` (object →
    property-set definitions; single definitions and
    `IfcPropertySetDefinitionSet` aggregates both) and
    `IfcRelDefinesByType` (occurrence → type object, first edge wins
    per the `SET [0:1]` Types inverse): `defined_property_sets(id)`,
    `type_of(id)`, and `property_set_ids(id)` — the applicable sets
    with type-level `HasPropertySets` inherited and **occurrence sets
    shadowing same-named type sets**. `HasPropertySets` reads
    positionally at index 5, valid for every `IfcTypeObject` subtype.
  - `property_set(step, id)` → `PropertySet` (IfcRoot header + named
    members, `property("IsExternal")` lookup); the whole
    `IfcSimpleProperty` family resolves: `IfcPropertySingleValue`,
    `IfcPropertyEnumeratedValue` (+ `IfcPropertyEnumeration`
    reference), `IfcPropertyBoundedValue` (upper/lower/set-point),
    `IfcPropertyListValue`, `IfcPropertyTableValue` (paired columns +
    units + interpolation), `IfcPropertyReferenceValue`, and nested
    `IfcComplexProperty` groups (depth-capped, self-reference dropped).
  - `element_quantity(step, id)` → `ElementQuantity` with the six
    `IfcPhysicalSimpleQuantity` kinds (`Length`/`Area`/`Volume`/
    `Count`/`Weight`/`Time`, each with the optional per-quantity
    named-unit override and `Formula`) and nested
    `IfcPhysicalComplexQuantity` groups.
  - `IfcValue` — the SELECT-typed leaf: keeps the defined-type wrapper
    (`IFCBOOLEAN(.T.)` → `type_name` + payload) with `as_number` /
    `as_str` / `as_bool` / `as_enum` accessors; plain literals accepted.
  - `Model::property_sets(id)` / `Model::element_quantities(id)` —
    resolved convenience walks.
  - Schema entries for the 12 property/quantity entities, the two
    relationships, and the fixture type-object slice (`IfcWallType`,
    `IfcWindowType`, `IfcSanitaryTerminalType`), transcribed from
    `IFC4_ADD2.exp`.
  - 13 unit tests + a new `property_sets` integration suite over the
    wall fixture (`Pset_WallCommon` 10 members with boolean/measure
    assertions, `Pset_WindowCommon` + window-type link) and the basin
    fixture (type link, `HasPropertySets` unset).

- Phase 3 (sectioned solids): `IfcSectionedSolidHorizontal`
  (`Directrix`, `CrossSections`, `CrossSectionPositions`) — the IFC 4.3
  infrastructure/alignment solid — now tessellates per the swept-disk
  digest §2.
  - Each `IfcAxis2PlacementLinear` station resolves its
    `IfcPointByDistanceExpression` (length-measure `DistanceAlong` +
    optional `OffsetLateral`/`OffsetVertical`; a longitudinal offset
    violates `NoLongitudinalOffsets` and is rejected; `IfcParameterValue`
    distances and explicit `Axis`/`RefDirection` tilt overrides surface
    `Unsupported`).
  - The **Horizontal** convention keeps every section LEVEL: profile +y
    maps to global +z and +x to the horizontal lateral direction
    (ẑ × tangent). Sub-stations are interpolated (profile rings and
    offsets blended linearly) at every directrix vertex between
    authored stations, so the loft follows a curved directrix instead
    of jumping station-to-station; profiles may change shape
    station-to-station (ring structure must stay congruent). End caps
    are the hole-aware profile triangulations.
  - 4 new tests: constant-section box (exact 20), tapered 2×1→4×1
    ruled loft (exact 30 = mean-area × length), quarter-circle
    directrix with level-section assertions (volume = area ×
    tessellated path length within 1%), and lateral+vertical station
    offsets (displaced bbox, volume preserved). Phase-2 typed entries
    for `IfcSectionedSolidHorizontal`, `IfcAxis2PlacementLinear`,
    `IfcPointByDistanceExpression`.

- Phase 3 (face orientation): the Brep face walk now applies
  `IfcFaceBound.Orientation` per the staged face-orientation digest —
  a bound flagged FALSE contributes its `IfcPolyLoop` in **reverse**
  of the stored vertex order (the shared-loop reuse case), so the
  effective winding, Newell normal and triangle orientation match the
  face sense and a well-formed `IfcClosedShell` tessellates with
  consistently outward normals (positive `signed_volume`).
  `IfcFaceSurface.SameSense` is documented as relating the face normal
  to the *surface* normal (digest §2.3) — the bound winding is already
  face-relative, so the planar tessellation applies `Orientation` only.
  4 new tests: shared loop under .T./.F. bounds (reversed triangle),
  mixed-orientation tetrahedron closing to +1/6 volume, `SameSense`
  no-flip on a planar `IfcFaceSurface`, and a .F.-flagged inner bound
  still opening its hole (area 16 − 4).

- Phase 3 (CSG primitives + convex Boolean tools): the parametric
  `IfcCsgPrimitive3D` family now tessellates with the per-primitive
  anchoring recorded in the swept-disk digest §3 (the EXPRESS schema
  does not state it): `IfcBlock` and `IfcRectangularPyramid` anchored
  by a base **corner** growing +x/+y/+z, `IfcRightCircularCone`
  standing on its base-circle centre with the apex at +z,
  `IfcRightCircularCylinder` **centred** (axis z ∈ [−H/2, +H/2]),
  `IfcSphere` centred (48×24 lat-long). `IfcCsgSolid` evaluates its
  `TreeRootExpression` (depth-capped).
  - Boolean DIFFERENCE / INTERSECTION now carve with **any closed
    convex solid tool**, not just half-spaces: the tool mesh's
    deduplicated face planes form a convex region (verified closed +
    convex) driving the same plane-splitting/re-capping path — so an
    extruded circular/rectangular tool genuinely cuts an opening.
    Non-convex tools keep the authored-boundary fallback (DIFFERENCE)
    / `Unsupported` (INTERSECTION).
  - 8 new exact-volume tests (corner-anchored block with placed bbox,
    pyramid X·Y·H/3, cone on base, centred cylinder, sphere ≥ 98% of
    (4/3)πR³ with on-sphere vertices, `IfcCsgSolid` block-notched
    cylinder, extruded-circle wall opening 60 − A₄₈(0.5)·3, two-box
    INTERSECTION overlap) + non-convex fallback regression. Phase-2
    typed entries for the five primitives and `IfcCsgSolid`.

- Phase 3 (swept-disk solids): `tessellate_item` now sweeps
  `IfcSweptDiskSolid` / `IfcSweptDiskSolidPolygonal` (`Directrix`,
  `Radius`, `InnerRadius`, `StartParam`, `EndParam`) into a watertight
  tube per the staged swept-disk digest — pipes, rods, railings,
  reinforcement bars.
  - 3-D directrix evaluation (`curve_points_3d`): `IfcPolyline`,
    `IfcIndexedPolyCurve` over `IfcCartesianPointList3D` with
    `IfcLineIndex` and three-point `IfcArcIndex` segments (the 2-D
    circumcircle construction carried out in the plane of the three
    points), `IfcTrimmedCurve` over a 3-D conic, full `IfcCircle` /
    `IfcEllipse`, and `IfcCompositeCurve` chains. `StartParam` /
    `EndParam` are honoured on a full-conic directrix (conic angle in
    the model plane-angle unit); a closed directrix (first ≈ last
    point) wraps seamlessly with no end caps.
  - Ring frames by parallel transport (the minimal rotation carrying
    each tangent to the next — no twist), with **exact elliptical mitre
    junctions**: at each path corner the ring plane is the mitre
    (bisector-normal) plane and the transported ring is stretched by
    1/cos(half-bend) within the bend plane, so a mitred polyline tube's
    volume is exactly cross-section-area × path-length. Solid rods get
    fan caps, `InnerRadius` pipes annular caps + a reversed inner wall.
  - 7 new tests: straight rod and hollow pipe (exact 48-gon volumes +
    radius membership), full-circle directrix torus (closed, volume
    within tessellation bounds, mitre-aware membership band), the
    digest's 3-D arc-directrix hollow pipe against Pappus, quarter-arc
    `StartParam`/`EndParam` trim, exact right-angle mitre volume, and a
    swept-disk first operand of an `IfcBooleanClippingResult`
    (documenting the schema's DISC/DISK WHERE-literal typo — the WHERE
    is deliberately not enforced). Phase-2 typed entries for
    `IfcSweptDiskSolid(Polygonal)`, `IfcTrimmedCurve`,
    `IfcCompositeCurve(Segment)`, `IfcEllipse`, `IfcLine`, `IfcVector`.

- Phase 3 (arcs + trimmed curves): profile boundary curves now cover
  the arc family per the staged arcs/trimmed-curves digest.
  - `IfcTrimmedCurve` over an `IfcCircle` / `IfcEllipse` / `IfcLine`
    basis: Cartesian trims are inverted through the conic
    parameterisation (`u` from `atan2(y/b, x/a)` in the conic frame),
    parameter trims are scaled by the model's plane-angle unit, and
    `MasterRepresentation` picks the authoritative form of a dual trim
    (`CARTESIAN` preferred for `UNSPECIFIED` — the parameter form is
    the digest's degree/radian interoperability hazard).
    `SenseAgreement` TRUE runs counter-clockwise (increasing `u`) from
    Trim1 to Trim2, FALSE clockwise; endpoints never swap. Arc
    tessellation density is `CIRCLE_SEGMENTS`·(swept/2π), ≥ 1.
  - `IfcArcIndex` (start / on-arc mid / end) segments of
    `IfcIndexedPolyCurve` fit the circumscribed circle through the
    three points, direction disambiguated by the mid point; junction
    points between segments are emitted once (position-based, shared
    with `IfcLineIndex` runs).
  - `IfcCompositeCurve` of `IfcCompositeCurveSegment`s (incl. the
    reparametrised subtype): each parent curve's points, reversed when
    `SameSense` is FALSE, junctions deduplicated; nesting bounded by a
    depth cap. Full `IfcEllipse` outer curves are also accepted.
  - New public `plane_angle_unit_scale(&StepFile)` — radians per model
    plane-angle unit, the `.PLANEANGLEUNIT.` analogue of
    `length_unit_scale` (SI radian prefixes + conversion-based degree
    chains). `IfcRevolvedAreaSolid.Angle` is now scaled by it too.
  - 8 new tests with exact inscribed-polygon volume assertions
    (three-point-arc half disc, quarter/long-way circle segments,
    Cartesian-master override of degree-polluted dual trims, degree
    model parameter trims, ellipse quarter segment, full-ellipse
    profile, four-segment stadium composite with a reversed
    `SameSense` segment) + a unit test for the degree/radian scale.

- Phase 3 (half-space clipping): `IfcBooleanResult` /
  `IfcBooleanClippingResult` DIFFERENCE with a half-space tool now
  genuinely **carves** the first operand instead of emitting it as
  authored, per the staged half-space clipping digest.
  - The `IfcHalfSpaceSolid.AgreementFlag` side convention: TRUE → the
    solid (removed) region is the negative side of the `IfcPlane` base
    surface's normal, FALSE → the positive side.
  - `IfcPolygonalBoundedHalfSpace` restricts the cut to the infinite
    prism of its closed 2-D `PolygonalBoundary` in its `Position` frame
    (a concave boundary is ear-clipped into convex prism pieces
    subtracted in sequence); `IfcBoxedHalfSpace` restricts it to its
    `Enclosure` `IfcBoundingBox`.
  - Mechanism: the operand's closed mesh is split by each region plane
    (Sutherland–Hodgman per triangle with shared-edge-canonical cut
    points) and every cut cross-section is **re-capped watertight** —
    boundary edges on the cut plane are chained into loops (leftmost-
    turn successor at pinch vertices, deterministic), grouped into
    outer/hole rings by winding, and triangulated hole-aware, so a cut
    through a hollow body caps as an annulus. The result is a union of
    closed pieces whose internal shared walls cancel exactly in the new
    public `TriMesh::signed_volume()` (divergence-theorem volume).
  - INTERSECTION with a plain / boxed half-space clips the operand to
    the solid side; a non-half-space DIFFERENCE tool still falls back
    to the unmodified first operand (mesh–mesh CSG is a later slice).
  - 9 new geometry tests with exact volume + watertightness (balanced
    directed edges) assertions: agreement-flag both ways, tilted-plane
    half-cube, clipping chains, polygonal-bounded (rectangular and
    concave-L footprints), boxed enclosure limit, half-space
    intersection, full-body clip → empty mesh, non-half-space tool
    fallback. Phase-2 typed entries for `IfcBoxedHalfSpace` and
    `IfcBoundingBox`.

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
