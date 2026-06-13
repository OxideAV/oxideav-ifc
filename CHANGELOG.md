# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added

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
  validation; geometry extraction reports unsupported until
  Phase 3), `make_decoder()`, and `register_mesh3d()` under format
  id `"ifc"` / extension `.ifc`.
- Test suite: 31 unit tests + 10 integration tests, including the
  five buildingSMART IFC 4 sample fixtures (CC-BY 4.0) parsed with
  exact instance counts, schema assertions, and spot-checked
  entities.
