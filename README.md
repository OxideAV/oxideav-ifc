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
| **3** | Geometry extraction into `oxideav-mesh3d::Scene3D`: tessellations (incl. face voids + colour maps), faceted Breps (face holes + bound orientation, void shells as cavities, poly-loops and **edge loops** with line / conic / bounded-curve edge geometry), **advanced (curved) Breps** — cylindrical / spherical / toroidal / B-spline / revolved / extruded faces trimmed in parameter space, watertight across seams, poles and shared chords — face/shell surface models, bounded-surface sheets (curve-bounded planes, rectangular trims of any supported surface, **p-curve-bounded surfaces** with implicit outers), swept solids (extruded / revolved + their **tapered** subtypes, **directrix sweeps** with a fixed-reference or reference-surface frame) over the full profile family — arbitrary curves with arc and **B-spline** boundaries (trimmed conics, three-point arcs, composite curves, NURBS), **named parameterised sections** (I / asymmetric I / L/T/U/Z/C, rounded rectangle, trapezium, with fillet / edge radii and slopes), hollow, derived / mirrored and composite profiles — swept-disk tubes (polygonal subtype with **filleted** corners), sectioned (alignment) solids, sectioned surfaces and sectioned spines, triangulated irregular networks, CSG primitives and CSG trees, **real mesh–mesh booleans** (UNION / INTERSECTION / DIFFERENCE with non-convex tools, half-space clipping, watertight stitching), mapped-item instancing, `IfcLocalPlacement` world-positioning, surface-style materials, and EXPRESS WHERE-rule validation for the swept-solid / profile slice | ✅ this release |
| **4** | Semantic data layer: property sets (`IfcPropertySet` — the full `IfcSimpleProperty` family + complex groups), quantity sets (`IfcElementQuantity` with SI scaling), type-object inheritance (`IfcRelDefinesByType` + `HasPropertySets` shadowing), material associations (`IfcRelAssociatesMaterial` — layer / profile / constituent sets), classification + document references, groups / systems / zones, void/fill opening graph, georeferencing (`IfcMapConversion` (+ `Scaled`) / `IfcRigidOperation` / `IfcProjectedCRS`, site lat/long), extended unit engine (area / volume / mass / time, prefixed-derived-unit policy) | ✅ this release |
| **Scene** | Scene assembly: spatial hierarchy, openings subtracted from hosts, shared representations tessellated once (optionally instanced), surface-style / material looks, model units and georeferencing, caller-granted thread budget; load linear in the model size | ✅ this release |

## Phase 1 surface

```rust
# let bytes = std::fs::read("model.ifc")?;
let step = oxideav_ifc::parse_step(&bytes)?;
assert_eq!(step.header.file_schema, ["IFC4"]);
for wall in step.instances_of("IfcWall") {
    println!("#{} {} args", wall.id, wall.args.len());
}
# Ok::<(), Box<dyn std::error::Error>>(())
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

* External-mapping (complex entity) records `#id = (A(…) B(…) …);`
  — which IFC writers do not emit but STEP application protocols use
  heavily — parse into `ParsedInstance::parts` (`EntityPart` per
  partial record; `part` / `has_part` / `keywords` accessors). The
  instance `keyword` is the partial keywords joined by `+`.
* User-defined keywords (`!NAME`) keep their `!` prefix.

## Phase 2 surface — typed schema layer

`oxideav_ifc::schema` layers the IFC 4 EXPRESS schema over the
positional instance graph for the core entity slice. Each entity's
attribute order is its inheritance chain concatenated **parent-first**
(EXPRESS digest §7), hand-transcribed from the staged
`IFC4_ADD2.exp` declarations.

```rust
# let bytes = std::fs::read("model.ifc")?;
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
# Ok::<(), Box<dyn std::error::Error>>(())
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
# let bytes = std::fs::read("model.ifc")?;
# let face_set_id: u64 = 42; // e.g. an IfcTriangulatedFaceSet instance id
let step = oxideav_ifc::parse_step(&bytes)?;
let mesh = oxideav_ifc::tessellate_item(&step, face_set_id)?;
println!("{} verts, {} tris", mesh.vertex_count(), mesh.triangle_count());
# Ok::<(), Box<dyn std::error::Error>>(())
```

* `TriMesh { positions: Vec<[f64;3]>, triangles: Vec<[u32;3]> }` — a
  flat indexed mesh in the representation item's local coordinate space.
  Triangles are **zero-based** (the one-based STEP `CoordIndex` plus any
  optional `PnIndex` indirection are resolved during extraction).
  `TriMesh::signed_volume()` reports the enclosed volume by the
  divergence theorem (positive for outward-wound watertight meshes).
* `tessellate_item(step, id)` — one `IfcTriangulatedFaceSet` (or `IfcTriangulatedIrregularNetwork`)
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
  becomes one mesh vertex. Per-bound `Orientation` flags are applied —
  a FALSE bound contributes its loop **reversed**, so a well-formed
  `IfcClosedShell` tessellates with consistently outward normals
  (positive `signed_volume`); planar `IfcFaceSurface` faces are
  accepted (`SameSense` relates the face to the *surface* normal and
  does not flip the loop-derived winding).
* **Edge loops** — `IfcEdgeLoop(EdgeList : LIST OF IfcOrientedEdge)` —
  bound faces too: each `IfcOrientedEdge(EdgeElement, Orientation)`
  contributes the sampled run of its element, reversed for
  `Orientation` FALSE, and consecutive edges chain head-to-tail. Edge
  runs are cached per `IfcEdge` id so the two faces sharing an edge
  share its vertices (watertight). `IfcEdgeCurve(EdgeStart, EdgeEnd,
  EdgeGeometry, SameSense)` samples the geometry between its
  `IfcVertexPoint`s: an `IfcLine` adds no interior points, an
  unbounded `IfcCircle` / `IfcEllipse` is swept from the start vertex's
  conic parameter to the end vertex's in the `SameSense` direction
  (coincident vertices = the full conic), and any bounded 3-D curve
  (polyline, trimmed / composite / indexed curve, B-spline) is sampled
  whole and re-ended on the vertices. Bare `IfcEdge`s are straight and
  `IfcSubedge`s clip their parent's run. **`IfcAdvancedBrep` /
  `IfcAdvancedBrepWithVoids`** walk the same shells; a planar
  `IfcAdvancedFace` (`FaceSurface` an `IfcPlane`) takes this polygon
  path.
* **Curved advanced faces** — `IfcAdvancedFace` / `IfcFaceSurface` on an
  `IfcCylindricalSurface`, `IfcSphericalSurface`, `IfcToroidalSurface`,
  `IfcBSplineSurfaceWithKnots` or `IfcRationalBSplineSurfaceWithKnots`
  — are meshed by **parameter-space trimming**: every loop vertex is
  inverted into the surface's `(u, v)` (analytically for the
  elementary surfaces, by a sampled guess + Gauss–Newton for B-spline
  patches), periodic parameters are unwrapped continuously along the
  loop (loops may wind around the axis), the loops are oriented so the
  face lies on their left (`SameSense` FALSE reverses them: the outward
  normal is then `−(∂S/∂u × ∂S/∂v)`), clipped — with their period
  images — to one fundamental rectangle whose perimeter supplies the
  seam / pole edges, ear-clipped hole-aware, and refined by midpoint
  subdivision to the circle density (angular parameters) or 1/24 of
  the knot domain (B-spline) — points inserted on a boundary chord
  stay on the 3-D chord, all others are evaluated on the surface.
  Period images and sphere poles weld to single mesh vertices, and
  every chord point is registered with the Brep so the neighbouring
  face's triangles are re-split along the same chord (no T-junctions):
  a closed cylinder, a sphere from two hemispheres, a torus from two
  half tubes and a box with a B-spline top all mesh **watertight**
  with consistently outward normals. A face whose flags leave the
  region empty (inconsistent `SameSense` / `Orientation` in the file)
  is retried with the opposite orientation. The **swept surfaces**
  `IfcSurfaceOfRevolution(SweptCurve, Position, AxisPosition)` and
  `IfcSurfaceOfLinearExtrusion(SweptCurve, Position, ExtrudedDirection,
  Depth)` join the same path: their `SweptCurve` (an
  `IfcArbitraryOpenProfileDef` curve, or any closed profile kind) is
  sampled in the `Position` xy-plane and parameterised by fractional
  sample index (periodic for a closed extrusion profile) — refinement
  splits edges *at* the profile samples, so meshed chords never cut a
  profile corner — with the revolution angle / extrusion distance as
  the other parameter; a profile on either side of the revolution
  axis inverts correctly, and on-axis profile points weld like poles.
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
  The **tapered** subtypes `IfcExtrudedAreaSolidTapered` /
  `IfcRevolvedAreaSolidTapered` loft the section linearly from
  `SweptArea` to `EndSweptArea` (the `CorrectProfileAssignment`
  same-type / derived pairing gives the point correspondence; holes
  taper with the outer ring, a tapered full turn keeps both caps).
* The shared **profile family** behind the sweeps
  (`profile_area`/`profile_areas`): `IfcArbitraryClosedProfileDef` and
  `IfcArbitraryProfileDefWithVoids` (inner curves become hole rings),
  `IfcRectangleProfileDef` / `IfcRectangleHollowProfileDef`
  (`WallThickness` inset, inner / outer fillet radii rounded),
  `IfcCircleProfileDef` / `IfcCircleHollowProfileDef` (48-segment
  circles / annuli), `IfcEllipseProfileDef`, the **named parameterised
  sections** `IfcIShapeProfileDef` / `IfcAsymmetricIShapeProfileDef`
  (unequal flanges centred on the web; IFC4 and the 12-attribute IFC 2x3
  layout; an omitted `TopFlangeThickness` reads as the bottom one) /
  `IfcLShapeProfileDef` / `IfcTShapeProfileDef` / `IfcUShapeProfileDef`
  / `IfcZShapeProfileDef` / `IfcCShapeProfileDef` /
  `IfcRoundedRectangleProfileDef` / `IfcTrapeziumProfileDef` (contours
  built centred on the profile's
  bounding box per the parameterised-profiles digest — Z overall width
  `2·FlangeWidth − WebThickness`, trapezium centred on its
  `BottomXDim × YDim` rectangle — with tangent-arc fillet / edge radii
  at the documented corners, `FlangeSlope` / `WebSlope` / `LegSlope`
  tilting the inner faces about the `b/4` thickness station, omitted
  radii read as sharp, and the entity WHERE rules enforced),
  `IfcDerivedProfileDef` / `IfcMirroredProfileDef` (parent area mapped
  by the 2-D operator / x-mirror), and `IfcCompositeProfileDef` (each
  component swept independently and merged). All rings are normalised
  counter-clockwise; parameterised profiles apply their optional 2-D
  `Position`.
* Boundary **curves** cover the arc family: `IfcPolyline` (duplicated
  closing point dropped), full `IfcCircle` / `IfcEllipse`,
  `IfcTrimmedCurve` over a circle / ellipse / line basis (Cartesian
  trims inverted through the conic parameterisation, parameter trims
  scaled by the model **plane-angle unit** — `plane_angle_unit_scale`;
  `MasterRepresentation` picks the authoritative dual-trim form,
  `SenseAgreement` the traversal direction), `IfcIndexedPolyCurve`
  with `IfcLineIndex` runs and three-point `IfcArcIndex` arcs
  (circumscribed circle, mid-point disambiguated),
  `IfcCompositeCurve` chains with per-segment `SameSense` reversal, and
  the **B-spline family** — `IfcBSplineCurveWithKnots` /
  `IfcRationalBSplineCurveWithKnots` (IFC4+) and the IFC 2x3
  `IfcBezierCurve` / `IfcRationalBezierCurve` — evaluated by de Boor's
  recurrence in homogeneous coordinates over the expanded
  `Knots` × `KnotMultiplicities` vector (the `IfcConstraintsParamBSpline`
  invariants enforced, Bézier knots synthesised), sampled per knot span
  at the circle density and capped in total (a rational quadratic
  circle meshes exactly on the circle at 48 segments).
* **`IfcSweptDiskSolid`** / `…Polygonal` sweeps a disk or annulus along
  a 3-D directrix (polyline, indexed poly-curve incl. 3-D arcs,
  trimmed / full circle, composite, B-spline): parallel-transported
  ring frames with **exact elliptical mitre junctions** (a mitred
  polyline tube is exactly area × length), `StartParam`/`EndParam` on
  conic directrices (the angle) and B-spline directrices (their own
  knot parameter), closed-path wrap, disk / annulus end caps, reversed
  inner walls.
* The **directrix-swept area solids** `IfcFixedReferenceSweptAreaSolid`
  (and the IFC 4.3 `IfcDirectrixDerivedReferenceSweptAreaSolid`, same
  attributes) and `IfcSurfaceCurveSweptAreaSolid` (IFC 2x3 / 4 / 4.3
  attribute order alike) sweep a profile area along any supported 3-D
  directrix. **Profile frame convention** (this crate's reading of the
  EXPRESS text — the staged digests do not spell it out; flagged as a
  docs ask): the profile plane is ⟂ the directrix tangent (profile +z
  → tangent), profile **+x follows the reference** — the
  `FixedReference` direction, or the `ReferenceSurface` normal at the
  directrix point (`IfcPlane` / `IfcCylindricalSurface` /
  `IfcSphericalSurface` / `IfcToroidalSurface`; other surfaces surface
  `Unsupported`) — projected ⟂ the tangent, and +y = tangent × x.
  Directrix and reference are read in the solid's `Position` frame,
  which then places the whole solid. Corners of the sampled directrix
  are **mitred** (the outgoing prism cut by the bisector plane), so a
  polyline sweep is exactly area × length, an arc sweep of a centred
  profile follows Pappus to the chord error, and a closed directrix
  wraps without caps. `StartParam` / `EndParam` accept
  `IfcCurveMeasureSelect` — `IFCPARAMETERVALUE` (conic angle, B-spline
  knot parameter) or `IFCLENGTHMEASURE` distance along the directrix
  (any curve); a parameter trim on a curve without a documented
  parameterisation (polylines, composites) surfaces `Unsupported`.
* **`IfcSectionedSolidHorizontal`** (IFC 4.3 alignment solid) lofts
  profiles at `IfcAxis2PlacementLinear` distance-along stations
  (lateral / vertical offsets honoured, longitudinal rejected per the
  WHERE rule): sections stay **level** (profile +y → global +z) and
  sub-stations interpolate the rings at every directrix vertex so the
  loft follows curved alignments.
* **Bounded surfaces as representation items** (open sheets, e.g. in a
  `Surface` representation): `IfcCurveBoundedPlane(BasisSurface,
  OuterBoundary, InnerBoundaries)` — boundaries read in the plane's
  2-D system (lifted through its placement) or as 3-D curves on the
  plane, triangulated hole-aware with the normal along the plane's +z;
  `IfcRectangularTrimmedSurface(BasisSurface, U1, V1, U2, V2, Usense,
  Vsense)` over a plane, cylinder, sphere, torus or B-spline patch —
  the parameters address the basis surface's own parameterisation
  (lengths on the plane, angles from the local +x axis scaled by the
  model plane-angle unit, knot values), `Usense` / `Vsense` FALSE runs
  a periodic parameter the other way round, and the sheet's normal
  follows the traversal senses. WHERE rules `U1AndU2Different`,
  `V1AndV2Different`, `UsenseCompatible`, `VsenseCompatible`.
* **`IfcSectionedSpine`** (`SpineCurve`, `CrossSections`,
  `CrossSectionPositions` — IFC 2x3 / 4 / 4.3 alike) places each profile
  by its own `IfcAxis2Placement3D` and lofts consecutive sections; the
  placements' origins are projected onto the sampled spine and a
  sub-section is blended in at every spine vertex between them
  (rings, placement offset and frame axes interpolated), so the loft
  follows a curved spine. Same-structure sections are required (a
  hollow-to-solid pair is refused); WHERE rules
  `CorrespondingSectionPositions` / `ConsistentProfileTypes`.
* The **CSG primitives** `IfcBlock` / `IfcRectangularPyramid` /
  `IfcRightCircularCone` / `IfcRightCircularCylinder` / `IfcSphere`
  mesh with their per-primitive anchoring (corner vs base vs centred),
  and `IfcCsgSolid` evaluates its `TreeRootExpression`.
* `tessellate_item` evaluates **boolean results** (`IfcBooleanResult` /
  `IfcBooleanClippingResult`, §8.8.3.5) with real carving:
  - a plain `IfcHalfSpaceSolid` tool on an `IfcPlane` (the
    `AgreementFlag` side convention) splits the operand's closed mesh
    along its base plane and **re-caps the cut watertight**
    (deterministic loop chaining, hole-aware annulus caps); a
    half-space on a **cylindrical, spherical or toroidal** base (its
    inside for `AgreementFlag` TRUE — the negative side of the outward
    surface normal — its outside for FALSE) is materialised as a finite solid
    over the operand's extent and carved by the evaluator below;
  - every other tool goes through the **mesh–mesh Boolean evaluator**
    (`mesh_boolean(a, b, op)`, also public): a binary space partition of
    each operand's faces classifies the other operand's polygons —
    split where they span a plane, coplanar ones by normal agreement,
    so coincident faces keep one copy and touching solids dissolve
    their shared face — and the seams are **stitched watertight**
    (position welding + T-junction splitting), with the exact identity
    vol(A − B) + vol(A ∩ B) = vol(A) pinned. Tools may be **non-convex**:
    any closed solid — extruded / revolved / swept, Breps, tessellated
    face sets, CSG primitives, nested Boolean results — or the bounded
    half-spaces `IfcPolygonalBoundedHalfSpace` (footprint prism;
    concave boundaries ear-clipped) and `IfcBoxedHalfSpace`
    (`Enclosure` box), materialised as finite solids over the
    operand's extent.
  - `UNION` of two closed operands is the regularised union (an open
    operand merges boundaries as authored); `INTERSECTION` /
    `DIFFERENCE` carve; a `DIFFERENCE` tool that cannot be meshed
    leaves the first operand as authored. Operands recurse (clipping
    chains and CSG trees nest) under a shared depth cap, and the
    fragment count is capped.
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

### Scene decoding (`registry` feature)

`IfcDecoder` builds a `Scene3D` from the whole model:

* **Hierarchy.** The project is the scene root; the spatial structure
  (`IfcRelAggregates`: site → building → storey → space) and element
  decomposition are nested nodes, and each product hangs under the
  structure element containing it
  (`IfcRelContainedInSpatialStructure`). Nodes are named after the
  object's `Name` and carry `ifc:global_id` / `ifc:class` extras;
  `Model::aggregating_parent` / `Model::container_of` expose the back
  edges.
* **Items.** Every product shape is flattened to its representation
  items — mapped items are followed into their shared source
  representation (nested, any `IfcCartesianTransformationOperator`
  incl. non-uniform scales and mirrors) — **one primitive per item**,
  positioned through the product's `IfcLocalPlacement` chain. Each
  distinct item is tessellated **once** per model however many
  products map it. Representations identified `Box`, `Clearance`,
  `Lighting`, `CoG` and `Reference` are not drawn, and `Body-FallBack`
  only stands in for a `Body` that produced nothing.
* **Openings.** The bodies of the opening elements voiding a host
  (`IfcRelVoidsElement`) are subtracted from the host's closed body
  items — bounding boxes first, in the host's local frame — and the
  openings themselves are not drawn
  (`with_opening_subtraction(false)` keeps the as-authored solids).
* **Looks.** `IfcStyledItem` → `IfcSurfaceStyle` shading / rendering
  becomes the primitive's `Material`: colour, `Transparency` → blended
  alpha, `Side` `.BOTH.` → two-sided, diffuse colour or factor,
  specular roughness, metal reflectance (surface colours are
  dielectric otherwise), deduplicated per style. A source item keeps
  its own style under a mapped item. An `IfcIndexedColourMap` on a
  triangulated **or polygonal** face set becomes per-vertex colours
  (vertices split per face). Unstyled items take the surface style of
  the product's associated material
  (`IfcMaterialDefinitionRepresentation`), else a named material
  (the basin fixture decodes with `"Ceramic"`). Texture maps are not
  read.
* **Model facts.** The scene is Z-up and declares the project length
  unit (`Scene3D::unit`, exact factor in `ifc:metres_per_unit` —
  coordinates stay in model units); `ifc:map_conversion` (eastings /
  northings / height / rotation / scale / CRS name) and
  `ifc:site_geolocation` carry the georeferencing.

Options (builder methods on `IfcDecoder`; defaults in bold):

| Method | Effect |
|---|---|
| `with_threads(n)` / `with_execution_context(&ctx)` | item tessellation and opening subtraction on up to `n` threads (**1**); the scene is identical at every budget |
| `with_instancing(bool)` | **off**: world-space vertices, one mesh per product. On: meshes in the product frame, placement as the node transform, and products mapping the same source items share **one** mesh |
| `with_opening_subtraction(bool)` | **on**; off draws hosts and openings as authored |
| `with_curves(bool)` | **off**; on draws curve items (axis / footprint curves, geometric curve set members, alignment curves) as line strips, so curve-only products appear |
| `decode_step(&StepFile)` | build the scene of an already parsed file |

`length_unit_scale(&StepFile)` resolves metres-per-model-unit from the
project's `IfcUnitAssignment` (`IfcSIUnit` prefixes and
`IfcConversionBasedUnit` factors — the wall fixture is millimetres, the
column fixture inches); `plane_angle_unit_scale(&StepFile)` resolves
radians-per-model-angle-unit the same way (degree models scale their
conic trim parameters and revolution angles). The decoder itself keeps
raw model units.

`oxideav_ifc::rules` validates the **EXPRESS WHERE rules** of this
slice — `where_rule_violations(step, id)` names the failed rule labels
of one instance (`None` when the entity has no transcribed rules),
`model_where_rule_violations(step)` sweeps the model: the named /
hollow / composite profiles, the swept area solids (`SweptAreaType`,
`ValidExtrusionDirection`, the revolution-axis rules, tapered
`CorrectProfileAssignment`, directrix-sweep `DirectrixBounded`), swept
disk solids (+ the polygonal subtype's `CorrectRadii` /
`DirectrixIsPolyline`), Boolean results and clipping results
(`FirstOperandClosed` / `SecondOperandClosed` / `SameDim`,
`OperatorType` / `FirstOperandType` / `SecondOperandType`), the
half-space subtypes (`BoundaryDim` / `BoundaryType`,
`UnboundedSurface`), sectioned solids / surfaces, triangulated
irregular networks, curves on surfaces, sectioned spines, advanced Breps / faces / edge loops /
oriented edges (`HasAdvancedFaces`, `VoidsHaveAdvancedFaces`,
`ApplicableEdgeCurves`, `ApplicableSurface`, `RequiresEdgeCurve`,
`IsClosed`, `IsContinuous`, `EdgeElementNotOriented`), the toroidal /
swept / B-spline surfaces (`MajorLargerMinor`, `SweptCurveType`,
`DepthGreaterZero`, the knot-list and weight rules) and rectangular
trimmed surfaces, B-spline curves
(`ConsistentBSpline` via the transcribed `IfcConstraintsParamBSpline`,
`CorrespondingKnotLists`, rational `SameNumOfWeightsAndPoints` /
`WeightsGreaterZero`), `IfcMapConversion` and `IfcRigidOperation`.

* **`IfcCurveBoundedSurface`** (`BasisSurface`, `Boundaries`,
  `ImplicitOuter`) — a region of any parameterised basis surface
  (plane / cylinder / sphere / torus / B-spline patch) bounded by
  curves in the surface's **own parameter space**: each
  `IfcBoundaryCurve` / `IfcOuterBoundaryCurve` is an
  `IfcCompositeCurveOnSurface` whose segments' parents are
  `IfcPcurve(BasisSurface, ReferenceCurve)` records (the 2-D reference
  curve read as (u, v), angular parameters scaled by the plane-angle
  unit) or `IfcSurfaceCurve` / `IfcIntersectionCurve` / `IfcSeamCurve`
  records (their p-curve on this basis when present, else the 3-D
  `Curve3D` inverted onto the surface). Loops are taken as authored in
  parameter space (a loop closing on the period image of its start
  winds around the seam), `ImplicitOuter` closes a periodic basis with
  winding rim loops at its parameter extents, and the sheet is meshed
  by the same parameter-space trimmer as the curved advanced faces —
  watertight across seams and poles. On a basis closed in **both**
  parameters (a torus) the implicit outer boundary is empty: the whole
  surface is the region and every listed curve a hole, with both seams
  placed in the widest parameter band no hole reaches (a hole
  straddling `u = 0` or `v = 0` is not cut, no hole edge lies on a
  seam, no sliver triangles). The same placement meshes an advanced
  face whose bound is an `IfcVertexLoop` — a whole sphere or torus,
  optionally with edge-loop holes — closed through the seams and the
  sphere's poles. WHERE rules `DimIs2D`,
  `CurveIs3D` / `CurveIsNotPcurve` / `TwoPCurves` / `SameSurface` /
  `DistinctSurfaces` (surface curves, transcribing
  `IfcGetBasisSurface` / `IfcAssociatedSurface`), `SameSurface` /
  `IsClosed` (composite curves on surfaces).

* **`IfcSectionedSurface`** (`Directrix`, `CrossSectionPositions`,
  `CrossSections` — note the order) lofts open `CURVE`-typed profiles
  (`IfcArbitraryOpenProfileDef`; a closed profile gives an open-ended
  tube) at `IfcAxis2PlacementLinear` stations into a sheet with the
  same station expansion as the horizontal solid — level sections,
  no caps, offsets refused (`NoOffsets`). Both sectioned entities now
  honour a placement's explicit `Axis` / `RefDirection` (Z = `Axis`,
  X = `RefDirection` ⟂ Z, Y = Z × X; frames blend by rotation between
  stations). WHERE rules `DirectrixIs3D`, `SectionsSameType`,
  `CorrespondingSectionPositions`, `AreaProfileTypes` / `NoOffsets`
  (surface), `ConsistentProfileTypes` / `NoLongitudinalOffsets`
  (solid).
* **`IfcTriangulatedIrregularNetwork`** (`Flags` after `PnIndex`)
  reads as its `IfcTriangulatedFaceSet`; WHERE rule `NotClosed`.

The synthetic fixture `tests/fixtures/synthetic-advanced-brep.ifc`
(authored for this crate, IFC 4) places an advanced-Brep cylinder, a
two-hemisphere sphere, a tapered sectioned spine and a fixed-reference
arc sweep as four proxies on one storey; the decoder test pins their
world-space bounding boxes and the geometry test their volumes and
watertightness.

### Fuzzing

`fuzz/` is a `cargo-fuzz` crate (its own workspace; needs a nightly
toolchain) with two targets, run daily by the `Fuzz` workflow:

* `tessellate` — every instance of an arbitrary STEP file is run
  through `tessellate_item` and `curve_points` under tight `StepLimits`, so the face-set /
  Brep / sweep / Boolean / curved-face paths must never panic or run
  away on hostile input. `fuzz/seeds/` holds the seed corpus (the STEP
  snippets of the geometry tests).
* `mesh_boolean` — hostile operand pairs (concatenations of boxes,
  tetrahedra and wedges, degenerate or inverted) through every operator
  of the mesh–mesh Boolean evaluator; on two well-formed boxes the
  partition identities and the watertightness of every result are
  asserted, so a classification or stitching bug is a finding. One
  input in eight also meets a dense sphere both ways round — the
  window-localised classifier path.

```sh
cd fuzz && cargo fuzz run tessellate seeds -- -max_total_time=300
cd fuzz && cargo fuzz run mesh_boolean -- -max_total_time=300
```

Bounded foreground runs of this release: `tessellate` 33 418 runs /
100 s and `mesh_boolean` 20 388 runs / 110 s, no findings — after one
earlier in the round: sampling curves exposed a per-path walk of
reference lattices in the curve-dimension query, now bounded and
pinned by a test.

Hostile-geometry work is bounded throughout: curve samples, B-spline
control nets, sweep sections, per-face triangles, refinement rounds,
Delaunay flips, the cross-face chord repair and the Boolean fragment
count all carry hard caps.

### Alignment curves (IFC 4.3)

`curve_points(step, id)` samples any bounded curve item into a 3-D
polyline, including the alignment family:

* `IfcCompositeCurve` of **`IfcCurveSegment`s** — each segment is the
  run of its `ParentCurve` from `SegmentStart` over `SegmentLength`
  (lengths along the parent; negative runs against its sense),
  inserted with its start on `Placement.Location` heading along
  `Placement.RefDirection`. It is evaluated by integrating the
  parent's curvature over arc length from that point and heading
  (Simpson heading, three-point Gauss–Legendre position):
  `IfcLine` (0), `IfcCircle` (1/R), **`IfcClothoid`**
  (`s / (A·|A|)`, a negative constant turning right) and
  **`IfcCosineSpiral`** (`1/ConstantTerm + cos(π·s/L)/CosineTerm`).
  The clothoid and cosine laws are this crate's reading of the
  attribute lists; they reproduce the transition end points, headings
  and radii of the Annex E alignment listings to 1e-6.
* **`IfcGradientCurve`** — the base curve lifted to the height of its
  elevation profile (segments in the distance-along / height plane).
* **`IfcOffsetCurveByDistances`** — lateral (left of travel) and
  vertical offsets interpolated between distances along the basis.

Every sample carries its true distance along the curve (arc length,
not the chord sum; for a gradient curve the base curve's), which
`IfcAxis2PlacementLinear` stations and length-valued sweep trims use
— so `IfcSectionedSolidHorizontal`, `IfcSectionedSurface` and the
directrix sweeps follow an alignment by its stationing. Alignment
segments are sampled at a quarter of the circle step, sixteen samples
at least on a curved run.

`IfcAnnotationFillArea` meshes as a planar region with inner
boundaries; an `IfcGeometricSet` / `IfcGeometricCurveSet` meshes its
surface members.

## Representation item coverage

Every entity under `IfcGeometricRepresentationItem` and
`IfcTopologicalRepresentationItem` in the staged IFC4 ADD2 and
IFC4X3 ADD2 schemas (153 entities; ³ = IFC 4.3 only). Of the 126
instantiable ones, 35 tessellate as items, 10 sample as curves, 58
are evaluated as parts or operands of those, 3 are typed only and 20
are refused — 12 of them carrying no surface or curve of their own
(lights, text, fill patterns, bare topology) and 8 for want of a
staged definition: 106 of the 114 geometry-bearing entities (93 %)
are covered.

| Family | Entities | Status | Notes |
|---|---|---|---|
| Tessellated face sets | `IfcTriangulatedFaceSet`, `IfcPolygonalFaceSet`, `IfcTriangulatedIrregularNetwork`³ | **tessellated** | indexed triangles / polygons (`PnIndex`, voids); per-face `IfcIndexedColourMap` colours on both face-set kinds; TIN `Flags` carry no geometry |
| Tessellated faces | `IfcIndexedPolygonalFace`, `IfcIndexedPolygonalFaceWithVoids` | evaluated in context | faces of a polygonal face set (inner loops are holes) |
| Boundary representations | `IfcFacetedBrep`, `IfcFacetedBrepWithVoids`, `IfcAdvancedBrep`, `IfcAdvancedBrepWithVoids`, `IfcFaceBasedSurfaceModel`, `IfcShellBasedSurfaceModel` | **tessellated** | planar faces by polygon triangulation, curved faces by parameter-space trimming; void shells are cavities |
| Topology | `IfcClosedShell`, `IfcOpenShell`, `IfcConnectedFaceSet`, `IfcFace`, `IfcFaceSurface`, `IfcAdvancedFace`, `IfcFaceBound`, `IfcFaceOuterBound`, `IfcPolyLoop`, `IfcEdgeLoop`, `IfcVertexLoop`, `IfcOrientedEdge`, `IfcEdgeCurve`, `IfcEdge`, `IfcSubedge`, `IfcVertexPoint` | evaluated in context | walked from their Brep / surface model; a vertex loop bounds a face covering a whole closed surface (sphere, torus) |
| Topology | `IfcVertex`, `IfcLoop`, `IfcPath` | **refused** | a bare `IfcVertex` / `IfcLoop` carries no geometry; `IfcPath` is referenced by no meshed entity |
| Swept area solids | `IfcExtrudedAreaSolid`, `IfcExtrudedAreaSolidTapered`, `IfcRevolvedAreaSolid`, `IfcRevolvedAreaSolidTapered`, `IfcFixedReferenceSweptAreaSolid`, `IfcDirectrixDerivedReferenceSweptAreaSolid`³, `IfcSurfaceCurveSweptAreaSolid` | **tessellated** | full profile family; directrix sweeps along any sampled 3-D curve incl. alignment curves; reference surfaces: plane / cylinder / sphere / torus |
| Other solids | `IfcSweptDiskSolid`, `IfcSweptDiskSolidPolygonal`, `IfcSectionedSolidHorizontal`³, `IfcSectionedSpine`, `IfcCsgSolid`, `IfcBooleanResult`, `IfcBooleanClippingResult` | **tessellated** | mitred tubes, station lofts, real mesh–mesh Booleans |
| CSG primitives | `IfcBlock`, `IfcRectangularPyramid`, `IfcRightCircularCone`, `IfcRightCircularCylinder`, `IfcSphere` | **tessellated** |  |
| Half-spaces | `IfcHalfSpaceSolid`, `IfcBoxedHalfSpace`, `IfcPolygonalBoundedHalfSpace` | evaluated in context | Boolean operands on a plane, cylinder, sphere or **torus** base (unbounded on their own); a swept / B-spline base is refused — no closed-form side test |
| Bounded surfaces | `IfcCurveBoundedPlane`, `IfcCurveBoundedSurface`, `IfcRectangularTrimmedSurface`, `IfcSectionedSurface`³ | **tessellated** | open sheets; `ImplicitOuter` on every basis incl. the doubly periodic torus |
| Basis surfaces | `IfcPlane`, `IfcCylindricalSurface`, `IfcSphericalSurface`, `IfcToroidalSurface`, `IfcBSplineSurfaceWithKnots`, `IfcRationalBSplineSurfaceWithKnots`, `IfcSurfaceOfLinearExtrusion`, `IfcSurfaceOfRevolution` | evaluated in context | face / trim / half-space / reference surfaces; unbounded (or untrimmed) as items of their own |
| Annotation and sets | `IfcAnnotationFillArea`, `IfcGeometricSet`, `IfcGeometricCurveSet` | **tessellated** | planar fill with inner boundaries; a set meshes its surface members, its curves are drawn by `with_curves` |
| Bounded curves | `IfcPolyline`, `IfcIndexedPolyCurve`, `IfcTrimmedCurve`, `IfcCompositeCurve`, `IfcBSplineCurveWithKnots`, `IfcRationalBSplineCurveWithKnots`, `IfcGradientCurve`³, `IfcOffsetCurveByDistances`³, `IfcCircle`, `IfcEllipse` | **sampled** (curve) | `curve_points`; profile boundaries, directrices, edge geometry; drawn as line strips by `with_curves` |
| Curve parts and parents | `IfcLine`, `IfcClothoid`³, `IfcCosineSpiral`³, `IfcCurveSegment`³, `IfcCompositeCurveSegment`, `IfcReparametrisedCompositeCurveSegment` | evaluated in context | `IfcLine` as trim basis / edge geometry / segment parent; the spirals as `IfcCurveSegment` parents (a segment alone is sampled too) |
| Curves on surfaces | `IfcPcurve`, `IfcSurfaceCurve`, `IfcIntersectionCurve`, `IfcSeamCurve`, `IfcCompositeCurveOnSurface`, `IfcBoundaryCurve`, `IfcOuterBoundaryCurve` | evaluated in context | boundaries of `IfcCurveBoundedSurface` in the basis parameter space |
| Spirals | `IfcSineSpiral`³, `IfcSecondOrderPolynomialSpiral`³, `IfcThirdOrderPolynomialSpiral`³, `IfcSeventhOrderPolynomialSpiral`³ | **refused** | the staged schema lists their terms but no page gives the curvature law / term normalisation (docs gap) |
| Other curves | `IfcPolynomialCurve`³, `IfcSegmentedReferenceCurve`³, `IfcOffsetCurve2D`, `IfcOffsetCurve3D` | **refused** | polynomial: parameter range and its relation to a segment length are not staged; segmented reference curve: the cant rules are only outlined; offset curves: the side a positive distance lies on is not staged (docs gaps) |
| Placements, points, directions | `IfcAxis1Placement`, `IfcAxis2Placement2D`, `IfcAxis2Placement3D`, `IfcAxis2PlacementLinear`³, `IfcCartesianPoint`, `IfcCartesianPointList2D`, `IfcCartesianPointList3D`, `IfcDirection`, `IfcVector`, `IfcPointByDistanceExpression`³, `IfcCartesianTransformationOperator2D`, `IfcCartesianTransformationOperator2DnonUniform`, `IfcCartesianTransformationOperator3D`, `IfcCartesianTransformationOperator3DnonUniform` | evaluated in context | frames, coordinates, stations; mapped-item / derived-profile operators incl. non-uniform scales and mirrors |
| Extents | `IfcBoundingBox`, `IfcPlanarExtent`, `IfcPlanarBox` | typed only | the enclosure of a boxed half-space; a `Box` representation is not drawn |
| Points on geometry | `IfcPointOnCurve`, `IfcPointOnSurface` | evaluated in context | wherever a point is read (vertex geometry, loop corners, locations): on a line, circle, ellipse or B-spline curve; on a plane, cylinder, sphere, torus or B-spline surface |
| Lights | `IfcLightSourceAmbient`, `IfcLightSourceDirectional`, `IfcLightSourceGoniometric`, `IfcLightSourcePositional`, `IfcLightSourceSpot` | **refused** | not geometry; not mapped to scene lights |
| Text and fill styles | `IfcTextLiteral`, `IfcTextLiteralWithExtent`, `IfcFillAreaStyleHatching`, `IfcFillAreaStyleTiles` | **refused** | text needs a font; fill areas are drawn solid, hatch / tile patterns are not applied |

The 27 abstract supertypes (`IfcBSplineCurve`, `IfcBSplineSurface`,
`IfcBoundedCurve`, `IfcBoundedSurface`, `IfcCartesianPointList`,
`IfcCartesianTransformationOperator`, `IfcConic`, `IfcCsgPrimitive3D`,
`IfcCurve`, `IfcDirectrixCurveSweptAreaSolid`³, `IfcElementarySurface`,
`IfcGeometricRepresentationItem`, `IfcLightSource`,
`IfcManifoldSolidBrep`, `IfcOffsetCurve`³, `IfcPlacement`, `IfcPoint`,
`IfcSectionedSolid`³, `IfcSegment`³, `IfcSolidModel`, `IfcSpiral`³,
`IfcSurface`, `IfcSweptAreaSolid`, `IfcSweptSurface`,
`IfcTessellatedFaceSet`, `IfcTessellatedItem`,
`IfcTopologicalRepresentationItem`) have no instances.

## Fixtures: what loads

Default decoder options; "open items" counts triangle primitives with
an unbalanced edge after welding equal positions, "non-manifold" edges
used by more than two triangles. Before = the previous release.

| Model | Products with a shape | Drawn before → after | Triangles before → after | Open items | Non-manifold edges |
|---|---|---|---|---|---|
| `ifc4-basin-tessellation` | 1 | 1 → 1 | 234 → 234 | 0 → 0 | 0 → 0 |
| `ifc4-column-straight-rectangle-tessellation` | 1 | 1 → 1 | 12 → 12 | 0 → 0 | 0 → 0 |
| `ifc4-tessellated-item` | 1 | 1 → 1 | 12 → 12 | 0 → 0 | 0 → 0 |
| `ifc4-tessellation-with-individual-colors` | 1 | 1 → 1 | 12 → 12 | 0 → 0 | 0 → 0 |
| `ifc4-wall-with-opening-and-window` | 3 (1 voiding) | 3 → 2 (wall carved, opening a void) | 36 → 76 | 0 → 0 | 0 → 0 |
| `synthetic-advanced-brep` | 4 | 4 → 4 | 18 916 → 18 916 | 0 → 0 | 0 → 0 |
| Annex E listing, fixed-reference swept area solid | 10 | no scene → 1 (8 with `with_curves`) | — → 300 | — → 0 | — → 0 |
| Annex E listing, sectioned solid horizontal | 10 | no scene → 1 (8 with `with_curves`) | — → 300 | — → 0 | — → 0 |
| Annex E listing, segmented reference curve | 4 | no scene → no solid (3 curve products with `with_curves`) | — | — | — |

The two Annex E solids close with volumes 2 699.99 and 2 699.85
against the exact 9 × 300 (the 9-unit section over 300 of alignment).
The products still undrawn there are the two zero-length terminating
alignment segments and, in the third listing, the cant curve
(`IfcSegmentedReferenceCurve`, refused). The listings are read from
the staged specification pages and are not redistributed here.

## Performance

`cargo run --release --example ifc_load -- …` (one aarch64 laptop,
medians of three runs, other builds running alongside).

**Load → scene, 100 000 products sharing 8 mapped representations**
(`--synthetic 100000`: 37.9 MiB, 703 787 instances, plus 100 walls
with 300 openings; 30.2 M output triangles when baked):

| | Before | After |
|---|---|---|
| Parse | 207 MiB/s | 195 MiB/s (same code path; 212 → 214 MiB/s on the 2.0 MiB staged STEP fixture) |
| Parse + scene, serial | 1 056 s | **0.55 s** (0.20 s parse + 0.35 s scene) |
| … 16 000 products | 19.8 s | 0.08 s |
| Peak resident memory, world-space meshes | 1 205 MiB | 1 260 MiB (the 30 M-triangle output dominates; the scene now also carries the hierarchy and node identities) |
| Peak resident memory, `with_instancing(true)` | — | **528 MiB** (108 meshes; the parsed file alone is 396 MiB, down from 438) |

What changed: product ownership, styled items, colour maps and
material representations are indexed in one pass over the instance
graph (each was a scan of the whole file per product — the old cost
was quadratic); placement chains and the model's plane-angle unit are
resolved once per model; each distinct representation item is
tessellated once and written per occurrence straight into the output
vertex format. Parsed-file memory fell 10 % (exact-capacity keywords
and argument lists). Baking 100 000 occurrences into world-space
vertices costs what its output weighs; instancing is the form that
scales, and stays opt-in because it changes the scene's shape (local
meshes + node transforms).

**Thread budget** (`--synthetic 20000 --unique`: 20 000 products each
with its own filleted I-section extrusion, 4.7 M triangles): scene
build 0.43 s serial → 0.27 s (2 threads) → 0.18 s (4) → 0.13 s (8);
the scene is identical at every budget (pinned by a test).

**Boolean cost** (`--boolean 256`: a 65 024-triangle sphere minus a
box straddling its surface):

| | Before | After |
|---|---|---|
| Time | 15.6 s | **0.07 s** |
| Output triangles | 67 078 | 65 038 (same volume) |

Polygons whose box misses the other operand's box are kept or dropped
unclassified and unsplit, and a large closed operand met by a small
one is classified through the part of it inside a window around the
small one (six plane cuts with re-capped sections, linear in the
mesh) instead of a partition of every face. A curved half-space
selecting the inside of its base now costs one Boolean instead of two.

## Phase 4 surface — semantic data layer

The typed model resolves the definition / association relationships
into a queryable surface (all std-only, `--no-default-features`
included):

```rust
# let bytes = std::fs::read("model.ifc")?;
# let wall_id: u64 = 42; // an IfcWall instance id
let step = oxideav_ifc::parse_step(&bytes)?;
let model = oxideav_ifc::Model::from_step(&step);

for pset in model.property_sets(wall_id) {
    println!("{}", pset.name.unwrap_or("?"));
    if let Some(p) = pset.property("IsExternal") {
        println!("  external: {:?}", p.nominal().and_then(|v| v.as_bool()));
    }
}
let name = model.material_assignment(wall_id).and_then(|m| m.name().map(String::from));
# Ok::<(), Box<dyn std::error::Error>>(())
```

* **Property sets** (`oxideav_ifc::props`): `Model` folds
  `IfcRelDefinesByProperties` (single definitions and definition-set
  aggregates) and `IfcRelDefinesByType`; `property_set_ids(id)` merges
  the occurrence's sets with the type's `HasPropertySets`, an
  occurrence set **shadowing** a same-named type set, and a type
  object queried directly answers with its own sets.
  `property_set(step, id)` resolves the whole `IfcSimpleProperty`
  family — single (`nominal()` with the measure-type wrapper kept:
  `IFCBOOLEAN(.T.)` → `type_name` + `as_bool`), enumerated (+
  `IfcPropertyEnumeration` reference), bounded (upper / lower /
  set-point), list, table (paired columns + units + interpolation),
  reference — plus nested `IfcComplexProperty` groups (depth-capped).
* **Quantity sets**: `element_quantity(step, id)` resolves
  `IfcElementQuantity` into length / area / volume / count / weight /
  time quantities (+ nested `IfcPhysicalComplexQuantity` groups),
  each with `Formula` and the optional per-quantity named-unit
  override. `Quantity::si_value` converts to SI reference units —
  the override wins (dimension-checked per the WHERE rules), else the
  model default.
* **Unit engine**: `area_unit_scale` (m²) / `volume_unit_scale` (m³) /
  `mass_unit_scale` (kg — the SI name is `.GRAM.`) /
  `time_unit_scale` (s) join the length / plane-angle scales, all via
  one per-dimension §8.11.3.11 walk; `named_unit_scale(step, id,
  unit_type)` resolves one `IfcSIUnit` / conversion-based chain
  directly. A prefix on a **named derived** SI unit (`.MILLI.` +
  `.SQUARE_METRE.`) is undefined by the specification (the conforming
  form is an `IfcDerivedUnit` over a prefixed base unit), so the
  default functions resolve it to `None`; the `*_with(…,
  PrefixedDerivedUnit::PrefixScalesLength)` variants read it as mm² =
  10⁻⁶ m² under explicit opt-in and flag the assumption in the
  returned `UnitScale`.
* **Materials** (`oxideav_ifc::material`): `Model::material_of(id)`
  (occurrence association wins, else the type's) →
  `material_assignment` resolving the full `IfcMaterialSelect`:
  plain `IfcMaterial`, `IfcMaterialList`, layer sets (+ usage;
  `total_thickness()` per `IfcMlsTotalThickness`), profile sets
  (+ usage incl. tapering; profile-def ids into the Phase-3 profile
  family), and constituent sets — with a headline `name()` accessor.
* **Openings**: `openings_of(wall)` / `voided_element_of(opening)`
  (`IfcRelVoidsElement`), `fillers_of(opening)` /
  `filled_opening_of(window)` (`IfcRelFillsElement`), and the
  flattened `hosted_fillers(wall)` walk.
* **Type objects**: `EntityKind::TypeObject` classifies the common
  element-type slice (wall / window / column / beam / slab / door
  (+style) / covering / member / plate / railing / roof / stair /
  furniture / proxy / sanitary-terminal types);
  `Model::type_objects()` enumerates, `is_type_object(id)` also
  recognises `RelatingType` targets outside the slice.
* **External references** (`oxideav_ifc::external`):
  `classifications_of(id)` / `documents_of(id)` fold the remaining
  `IfcRelAssociates` family (multi-valued, occurrence + type merged);
  `classification_assignment` walks an `IfcClassificationReference`'s
  `ReferencedSource` chain to its system root (`code()` gives the
  Uniclass/OmniClass-style identifier), `document_assignment` resolves
  `IfcDocumentInformation` / `IfcDocumentReference` metadata.
* **Groups / systems / zones**: `group_members(group)` /
  `groups_of(object)` over `IfcRelAssignsToGroup`(`ByFactor`),
  `serviced_buildings(system)` over `IfcRelServicesBuildings`, and the
  `groups()` enumeration (`IfcGroup` / `IfcSystem` / `IfcZone` /
  distribution + building systems).
* **Georeferencing** (`oxideav_ifc::geo`): `map_conversion(step)` —
  the `IfcMapConversion` bound to the geometric representation
  context (or the IFC 4.3 `IfcMapConversionScaled`, with its
  per-axis `factors`), with its `IfcProjectedCRS` target (EPSG name,
  datums, map unit); `MapConversion::to_map` / `from_map` /
  `transform()` apply the documented operation in order — one `Scale`
  on all three axes, the anti-clockwise z-rotation
  `atan2(XAxisOrdinate, XAxisAbscissa)` (`rotation_angle()`), then the
  unscaled E/N/H translation — so a placed `TriMesh` carries into map
  coordinates. `rigid_operations` exposes `IfcRigidOperation` (length
  or plane-angle coordinate pairs). `site_geolocation(step)` converts
  `IfcSite` compound-measure lat/long to decimal degrees.

The registry decoder uses the layer too: primitives with no surface
style fall back to the product's associated material as a named
`Material` (the basin fixture decodes with `"Ceramic"`).

## Shared ISO 10303-42 kernel

`oxideav_ifc::kernel` exposes the geometry engine over plain data so
other ISO 10303 readers (the STEP AP203 / AP214 / AP242 reader
`oxideav-step`) resolve their own entity layouts onto it:

* `BSplineCurve` / `BSplineSurface` — (rational) B-splines from control
  points + distinct knots + multiplicities.
* `Surface` — plane, cylinder, cone, sphere, torus, B-spline, surface of
  revolution / linear extrusion of a sampled 3-D curve, offset surface;
  `point_at` / `inverse` / `normal_at`; refinement density from
  `with_chordal_tolerance(tol, max_angle, size_hint)` or explicit
  `with_param_steps`.
* `FaceMesher` — shared vertex pool + triangles: `add_planar_face`,
  `add_surface_face` (boundary loops inverted into parameter space,
  clipped to the fundamental domain, triangulated and refined),
  `add_parameter_face`; faces sharing boundary vertex ids come out
  watertight, `finish` repairs the refinement's T-junctions.
* `triangulate_polygon` — constrained Delaunay triangulation of a 2-D
  polygon with holes (ear clipping with hole bridges as the fallback).

## Cargo features

* `registry` *(default)* — pulls `oxideav-core` + `oxideav-mesh3d`
  and exposes `IfcDecoder` (a `Mesh3DDecoder`), the `make_decoder()`
  direct constructor, and `register_mesh3d(&mut Mesh3DRegistry)`
  (format id `"ifc"`, extension `.ifc`). The decoder probes the magic,
  fully parses + validates the exchange structure, and extracts every
  tessellated / faceted-Brep / swept-solid / boolean-result /
  mapped-item product shape into the `Scene3D` — product-named nodes,
  per-item primitives, surface-style materials, per-face colour maps;
  a model with no drawable geometry decodes to `Unsupported`.
* `--no-default-features` — standalone STEP parser only, std types,
  zero dependencies.

## Fixtures

`tests/fixtures/*.ifc` are five small IFC 4 sample models from the
buildingSMART **Sample-Test-Files** repository, licensed **CC-BY
4.0** by buildingSMART International (attribution preserved here);
they exercise tessellated face sets, a column, and a wall with
opening + window over the full spatial hierarchy.
`tests/named_profiles.rs` adds a synthetic IFC 4 steelwork model
authored from the staged schema — an I-section beam laid along +x, a
tapered U-section column, millimetre units and `IfcMapConversion`
georeferencing — pinned through the std-only API and the registry
decoder.

## License

MIT — see [LICENSE](LICENSE). © 2026 Karpelès Lab Inc.
