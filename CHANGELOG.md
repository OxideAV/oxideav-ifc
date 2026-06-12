# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added

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
