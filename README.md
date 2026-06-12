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
| **1** | STEP physical-file (ISO 10303-21) parser: HEADER + DATA instance graph, full parameter grammar, reference resolver, DoS caps | ✅ this release |
| **2** | EXPRESS-schema-aware typing: attribute names/arity per `IFC2X3` / `IFC4` / `IFC4X3_ADD2`, spatial-structure traversal | planned |
| **3** | Geometry extraction into `oxideav-mesh3d::Scene3D`: `IFCTRIANGULATEDFACESET` / `IFCPOLYGONALFACESET` tessellations first, swept solids / Breps later | planned |

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
a clear error; EXPRESS-level validation (attribute arity, WHERE
rules) is Phase 2.

## Cargo features

* `registry` *(default)* — pulls `oxideav-core` + `oxideav-mesh3d`
  and exposes `IfcDecoder` (a `Mesh3DDecoder`), the `make_decoder()`
  direct constructor, and `register_mesh3d(&mut Mesh3DRegistry)`
  (format id `"ifc"`, extension `.ifc`). The Phase-1 decoder probes
  the magic, fully parses + validates the exchange structure, and
  reports geometry extraction as unsupported until Phase 3.
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
