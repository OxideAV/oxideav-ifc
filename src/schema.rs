//! Phase 2: EXPRESS schema typing over the Phase-1 STEP parser.
//!
//! Phase 1 ([`crate::parser`]) turns a `.ifc` file into a positional
//! instance graph: every `#id = KEYWORD(args);` record is a
//! [`ParsedInstance`](crate::ParsedInstance) whose `args` are an
//! ordered list of [`Value`]s with no attribute names attached. The
//! wire format carries position only — the IFC EXPRESS schema is what
//! gives each positional slot a *name* and a *type*.
//!
//! This module supplies that schema layer for the core IFC 4 entity
//! slice (the spatial-structure backbone plus the common building
//! elements, placements, and representation references). It does two
//! things:
//!
//! * [`EntitySchema`] — a static table mapping an entity keyword to its
//!   ordered attribute names. The order is the EXPRESS serialisation
//!   order: a subtype's argument list is its inheritance chain's
//!   attributes concatenated **parent-first** (ISO 10303-11 / EXPRESS
//!   digest §7). So `IFCWALL` reads `IfcRoot` → `IfcObject` →
//!   `IfcProduct` → `IfcElement` → `IfcWall` attributes in that order.
//!   The table is hand-transcribed from the staged IFC 4 EXPRESS schema
//!   (`docs/3d/ifc/IFC4_ADD2.exp`) entity declarations.
//!
//! * [`TypedEntity`] — a borrowing view of one [`ParsedInstance`] that
//!   resolves attribute *names* to their positional [`Value`] via the
//!   schema, plus convenience typed-accessors for the IFC-common
//!   attributes (GlobalId, Name, ObjectPlacement reference, …).
//!
//! Geometry extraction (turning representation items into meshes) stays
//! Phase 3. This layer stops at typed *attribute resolution* and the
//! spatial-structure graph ([`Model`]).

use std::collections::BTreeMap;

use crate::parser::{ParsedInstance, StepFile};
use crate::value::Value;

/// The ordered attribute names of one IFC entity, in EXPRESS
/// serialisation order (inheritance chain concatenated parent-first).
///
/// `kind` classifies the entity into the small set of structural roles
/// the typed layer cares about; `attrs` are the positional attribute
/// names matching the wire argument order one-for-one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EntitySchema {
    /// Upper-cased entity keyword (`"IFCWALL"`).
    pub keyword: &'static str,
    /// Structural role of the entity.
    pub kind: EntityKind,
    /// Ordered attribute names; index *i* names argument *i*.
    pub attrs: &'static [&'static str],
}

/// Structural classification of a typed entity — the coarse role the
/// spatial-model builder routes on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityKind {
    /// `IfcProject` — the single model root context.
    Project,
    /// A spatial-structure element: site / building / storey / space.
    Spatial(SpatialKind),
    /// A physical product placed in the spatial structure (wall, door,
    /// window, column, slab, beam, opening, …).
    Product,
    /// `IfcRelAggregates` — composition (project→site→building→storey).
    RelAggregates,
    /// `IfcRelContainedInSpatialStructure` — element containment.
    RelContained,
    /// `IfcRelDefinesByProperties` — property-set / quantity-set
    /// assignment to occurrence objects.
    RelDefinesByProperties,
    /// `IfcRelDefinesByType` — type-object assignment to occurrences
    /// (the occurrence inherits the type's `HasPropertySets`).
    RelDefinesByType,
    /// `IfcRelAssociatesMaterial` — material assignment to objects
    /// (occurrences and type objects).
    RelAssociatesMaterial,
    /// `IfcRelVoidsElement` — an opening carved out of an element.
    RelVoids,
    /// `IfcRelFillsElement` — an element (door / window) filling an
    /// opening.
    RelFills,
    /// An `IfcTypeObject` subtype — the type-level definition an
    /// occurrence inherits property sets (and materials) from.
    TypeObject,
    /// A placement entity (`IfcLocalPlacement`).
    Placement,
    /// A representation / context entity referenced by a product.
    Representation,
    /// A geometric-representation-item primitive — the core point /
    /// direction / placement / curve set a representation is built from
    /// (`IfcCartesianPoint`, `IfcDirection`, `IfcAxis2Placement2D`,
    /// `IfcAxis2Placement3D`, `IfcPolyline`).
    Geometry,
    /// Any other typed entity in the table with named attributes but no
    /// special structural role.
    Other,
}

/// The four IFC spatial-structure levels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpatialKind {
    /// `IfcSite`.
    Site,
    /// `IfcBuilding`.
    Building,
    /// `IfcBuildingStorey`.
    Storey,
    /// `IfcSpace`.
    Space,
}

// ---------------------------------------------------------------------
// Attribute chains (parent-first), transcribed from IFC4_ADD2.exp.
//
// Common ancestor segments are named once and concatenated per entity
// so the inheritance order is auditable against the schema text.
// ---------------------------------------------------------------------

/// `IfcRoot` — GlobalId, OwnerHistory, Name, Description.
const ROOT: &[&str] = &["GlobalId", "OwnerHistory", "Name", "Description"];

macro_rules! chain {
    ($($seg:expr),+ $(,)?) => {{
        // Build a flat &'static [&str] at compile time by const concat.
        const N: usize = 0 $(+ $seg.len())+;
        const A: [&str; N] = {
            let mut out = [""; N];
            let mut i = 0;
            $(
                let seg = $seg;
                let mut j = 0;
                while j < seg.len() {
                    out[i] = seg[j];
                    i += 1;
                    j += 1;
                }
            )+
            out
        };
        &A
    }};
}

// IfcRoot → IfcObjectDefinition(none) → IfcObject(ObjectType) →
//   IfcProduct(ObjectPlacement, Representation)
const OBJECT_TAIL: &[&str] = &["ObjectType"];
const PRODUCT_TAIL: &[&str] = &["ObjectPlacement", "Representation"];
// IfcSpatialElement(LongName) → IfcSpatialStructureElement(CompositionType)
const SPATIAL_TAIL: &[&str] = &["LongName"];
const SPATIAL_STRUCT_TAIL: &[&str] = &["CompositionType"];
// IfcElement(Tag)
const ELEMENT_TAIL: &[&str] = &["Tag"];
// IfcContext(ObjectType, LongName, Phase, RepresentationContexts, UnitsInContext)
const CONTEXT_TAIL: &[&str] = &[
    "ObjectType",
    "LongName",
    "Phase",
    "RepresentationContexts",
    "UnitsInContext",
];
// IfcTypeObject(ApplicableOccurrence, HasPropertySets) →
//   IfcTypeProduct(RepresentationMaps, Tag) → IfcElementType(ElementType)
const TYPE_OBJECT_TAIL: &[&str] = &["ApplicableOccurrence", "HasPropertySets"];
const TYPE_PRODUCT_TAIL: &[&str] = &["RepresentationMaps", "Tag"];
const ELEMENT_TYPE_TAIL: &[&str] = &["ElementType"];
// IfcProperty(Name, Description) — the shared property header; note
// IfcPropertyAbstraction adds no serialised attributes.
const PROPERTY_HEAD: &[&str] = &["Name", "Description"];
// IfcPhysicalQuantity(Name, Description) — the shared quantity header.
const QUANTITY_HEAD: &[&str] = &["Name", "Description"];
// IfcPhysicalSimpleQuantity(Unit) — every simple quantity carries an
// optional per-quantity named-unit override before its value slot.
const SIMPLE_QUANTITY_UNIT: &[&str] = &["Unit"];

/// Master schema table for the core IFC 4 entity slice.
///
/// Every entry's `attrs` length equals the EXPRESS serialised argument
/// count for that entity; the [`schema_of`] lookup is exact (no
/// supertype fallback) so an unknown keyword resolves to `None` and the
/// caller keeps the positional view.
pub const SCHEMA: &[EntitySchema] = &[
    // ---- Project / context ----
    EntitySchema {
        keyword: "IFCPROJECT",
        kind: EntityKind::Project,
        // IfcRoot + IfcContext.
        attrs: chain!(ROOT, CONTEXT_TAIL),
    },
    // ---- Spatial structure ----
    EntitySchema {
        keyword: "IFCSITE",
        kind: EntityKind::Spatial(SpatialKind::Site),
        // IfcRoot + ObjectType + ObjectPlacement,Representation + LongName
        // + CompositionType + IfcSite(5).
        attrs: chain!(
            ROOT,
            OBJECT_TAIL,
            PRODUCT_TAIL,
            SPATIAL_TAIL,
            SPATIAL_STRUCT_TAIL,
            &[
                "RefLatitude",
                "RefLongitude",
                "RefElevation",
                "LandTitleNumber",
                "SiteAddress"
            ]
        ),
    },
    EntitySchema {
        keyword: "IFCBUILDING",
        kind: EntityKind::Spatial(SpatialKind::Building),
        attrs: chain!(
            ROOT,
            OBJECT_TAIL,
            PRODUCT_TAIL,
            SPATIAL_TAIL,
            SPATIAL_STRUCT_TAIL,
            &[
                "ElevationOfRefHeight",
                "ElevationOfTerrain",
                "BuildingAddress"
            ]
        ),
    },
    EntitySchema {
        keyword: "IFCBUILDINGSTOREY",
        kind: EntityKind::Spatial(SpatialKind::Storey),
        attrs: chain!(
            ROOT,
            OBJECT_TAIL,
            PRODUCT_TAIL,
            SPATIAL_TAIL,
            SPATIAL_STRUCT_TAIL,
            &["Elevation"]
        ),
    },
    EntitySchema {
        keyword: "IFCSPACE",
        kind: EntityKind::Spatial(SpatialKind::Space),
        attrs: chain!(
            ROOT,
            OBJECT_TAIL,
            PRODUCT_TAIL,
            SPATIAL_TAIL,
            SPATIAL_STRUCT_TAIL,
            &["PredefinedType", "ElevationWithFlooring"]
        ),
    },
    // ---- Building elements (IfcRoot+Object+Product+Element + own) ----
    EntitySchema {
        keyword: "IFCWALL",
        kind: EntityKind::Product,
        attrs: chain!(
            ROOT,
            OBJECT_TAIL,
            PRODUCT_TAIL,
            ELEMENT_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCWALLSTANDARDCASE",
        kind: EntityKind::Product,
        // IfcWallStandardCase adds no attributes over IfcWall.
        attrs: chain!(
            ROOT,
            OBJECT_TAIL,
            PRODUCT_TAIL,
            ELEMENT_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCCOLUMN",
        kind: EntityKind::Product,
        attrs: chain!(
            ROOT,
            OBJECT_TAIL,
            PRODUCT_TAIL,
            ELEMENT_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCBEAM",
        kind: EntityKind::Product,
        attrs: chain!(
            ROOT,
            OBJECT_TAIL,
            PRODUCT_TAIL,
            ELEMENT_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCSLAB",
        kind: EntityKind::Product,
        attrs: chain!(
            ROOT,
            OBJECT_TAIL,
            PRODUCT_TAIL,
            ELEMENT_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCDOOR",
        kind: EntityKind::Product,
        attrs: chain!(
            ROOT,
            OBJECT_TAIL,
            PRODUCT_TAIL,
            ELEMENT_TAIL,
            &[
                "OverallHeight",
                "OverallWidth",
                "PredefinedType",
                "OperationType",
                "UserDefinedOperationType"
            ]
        ),
    },
    EntitySchema {
        keyword: "IFCWINDOW",
        kind: EntityKind::Product,
        attrs: chain!(
            ROOT,
            OBJECT_TAIL,
            PRODUCT_TAIL,
            ELEMENT_TAIL,
            &[
                "OverallHeight",
                "OverallWidth",
                "PredefinedType",
                "PartitioningType",
                "UserDefinedPartitioningType"
            ]
        ),
    },
    EntitySchema {
        keyword: "IFCOPENINGELEMENT",
        kind: EntityKind::Product,
        // IfcFeatureElement/Subtraction add no attributes; own:
        // PredefinedType.
        attrs: chain!(
            ROOT,
            OBJECT_TAIL,
            PRODUCT_TAIL,
            ELEMENT_TAIL,
            &["PredefinedType"]
        ),
    },
    // ---- Relationships ----
    EntitySchema {
        keyword: "IFCRELAGGREGATES",
        kind: EntityKind::RelAggregates,
        // IfcRoot + IfcRelDecomposes(none) + RelatingObject, RelatedObjects.
        attrs: chain!(ROOT, &["RelatingObject", "RelatedObjects"]),
    },
    EntitySchema {
        keyword: "IFCRELCONTAINEDINSPATIALSTRUCTURE",
        kind: EntityKind::RelContained,
        // IfcRoot + IfcRelConnects(none) + RelatedElements, RelatingStructure.
        attrs: chain!(ROOT, &["RelatedElements", "RelatingStructure"]),
    },
    // ---- Placement ----
    EntitySchema {
        keyword: "IFCLOCALPLACEMENT",
        kind: EntityKind::Placement,
        attrs: chain!(&["PlacementRelTo", "RelativePlacement"]),
    },
    // ---- Representation / context ----
    EntitySchema {
        keyword: "IFCPRODUCTDEFINITIONSHAPE",
        kind: EntityKind::Representation,
        attrs: chain!(&["Name", "Description", "Representations"]),
    },
    EntitySchema {
        keyword: "IFCGEOMETRICREPRESENTATIONCONTEXT",
        kind: EntityKind::Representation,
        // IfcRepresentationContext(ContextIdentifier, ContextType) +
        // IfcGeometricRepresentationContext(4).
        attrs: chain!(
            &["ContextIdentifier", "ContextType"],
            &[
                "CoordinateSpaceDimension",
                "Precision",
                "WorldCoordinateSystem",
                "TrueNorth"
            ]
        ),
    },
    EntitySchema {
        keyword: "IFCSHAPEREPRESENTATION",
        kind: EntityKind::Representation,
        // IfcRepresentation(ContextOfItems, RepresentationIdentifier,
        // RepresentationType, Items); IfcShapeModel /
        // IfcShapeRepresentation add no serialised attributes.
        attrs: chain!(&[
            "ContextOfItems",
            "RepresentationIdentifier",
            "RepresentationType",
            "Items"
        ]),
    },
    // ---- Geometric representation items (core primitive slice) ----
    EntitySchema {
        keyword: "IFCCARTESIANPOINT",
        kind: EntityKind::Geometry,
        // IfcPoint adds nothing; IfcCartesianPoint(Coordinates).
        attrs: chain!(&["Coordinates"]),
    },
    EntitySchema {
        keyword: "IFCDIRECTION",
        kind: EntityKind::Geometry,
        attrs: chain!(&["DirectionRatios"]),
    },
    EntitySchema {
        keyword: "IFCAXIS2PLACEMENT2D",
        kind: EntityKind::Geometry,
        // IfcPlacement(Location) + IfcAxis2Placement2D(RefDirection).
        attrs: chain!(&["Location", "RefDirection"]),
    },
    EntitySchema {
        keyword: "IFCAXIS2PLACEMENT3D",
        kind: EntityKind::Geometry,
        // IfcPlacement(Location) + IfcAxis2Placement3D(Axis, RefDirection).
        attrs: chain!(&["Location", "Axis", "RefDirection"]),
    },
    EntitySchema {
        keyword: "IFCPOLYLINE",
        kind: EntityKind::Geometry,
        // IfcCurve / IfcBoundedCurve add nothing; IfcPolyline(Points).
        attrs: chain!(&["Points"]),
    },
    // ---- Mapped-item instancing (representation reuse) ----
    EntitySchema {
        keyword: "IFCMAPPEDITEM",
        kind: EntityKind::Geometry,
        // IfcRepresentationItem adds nothing;
        // IfcMappedItem(MappingSource, MappingTarget).
        attrs: chain!(&["MappingSource", "MappingTarget"]),
    },
    EntitySchema {
        keyword: "IFCREPRESENTATIONMAP",
        kind: EntityKind::Geometry,
        // IfcRepresentationMap(MappingOrigin, MappedRepresentation).
        attrs: chain!(&["MappingOrigin", "MappedRepresentation"]),
    },
    EntitySchema {
        keyword: "IFCCARTESIANTRANSFORMATIONOPERATOR3D",
        kind: EntityKind::Geometry,
        // IfcGeometricRepresentationItem adds nothing;
        // IfcCartesianTransformationOperator(Axis1, Axis2, LocalOrigin,
        // Scale) + IfcCartesianTransformationOperator3D(Axis3).
        attrs: chain!(&["Axis1", "Axis2", "LocalOrigin", "Scale", "Axis3"]),
    },
    EntitySchema {
        keyword: "IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM",
        kind: EntityKind::Geometry,
        // …Operator(Axis1, Axis2, LocalOrigin, Scale) + 3D(Axis3) +
        // 3DnonUniform(Scale2, Scale3).
        attrs: chain!(&[
            "Axis1",
            "Axis2",
            "LocalOrigin",
            "Scale",
            "Axis3",
            "Scale2",
            "Scale3"
        ]),
    },
    // ---- Swept-area solids ----
    EntitySchema {
        keyword: "IFCAXIS1PLACEMENT",
        kind: EntityKind::Geometry,
        // IfcPlacement(Location) + IfcAxis1Placement(Axis).
        attrs: chain!(&["Location", "Axis"]),
    },
    EntitySchema {
        keyword: "IFCREVOLVEDAREASOLID",
        kind: EntityKind::Geometry,
        // IfcSweptAreaSolid(SweptArea, Position) +
        // IfcRevolvedAreaSolid(Axis, Angle).
        attrs: chain!(&["SweptArea", "Position", "Axis", "Angle"]),
    },
    EntitySchema {
        keyword: "IFCEXTRUDEDAREASOLID",
        kind: EntityKind::Geometry,
        // IfcSweptAreaSolid(SweptArea, Position) +
        // IfcExtrudedAreaSolid(ExtrudedDirection, Depth).
        attrs: chain!(&["SweptArea", "Position", "ExtrudedDirection", "Depth"]),
    },
    // ---- Profile definitions (swept-area cross-sections) ----
    EntitySchema {
        keyword: "IFCARBITRARYCLOSEDPROFILEDEF",
        kind: EntityKind::Geometry,
        // IfcProfileDef(ProfileType, ProfileName) +
        // IfcArbitraryClosedProfileDef(OuterCurve).
        attrs: chain!(&["ProfileType", "ProfileName", "OuterCurve"]),
    },
    EntitySchema {
        keyword: "IFCRECTANGLEPROFILEDEF",
        kind: EntityKind::Geometry,
        // IfcProfileDef(2) + IfcParameterizedProfileDef(Position) +
        // IfcRectangleProfileDef(XDim, YDim).
        attrs: chain!(&["ProfileType", "ProfileName", "Position", "XDim", "YDim"]),
    },
    EntitySchema {
        keyword: "IFCCIRCLEPROFILEDEF",
        kind: EntityKind::Geometry,
        // IfcProfileDef(2) + IfcParameterizedProfileDef(Position) +
        // IfcCircleProfileDef(Radius).
        attrs: chain!(&["ProfileType", "ProfileName", "Position", "Radius"]),
    },
    EntitySchema {
        keyword: "IFCELLIPSEPROFILEDEF",
        kind: EntityKind::Geometry,
        // IfcProfileDef(2) + IfcParameterizedProfileDef(Position) +
        // IfcEllipseProfileDef(SemiAxis1, SemiAxis2).
        attrs: chain!(&[
            "ProfileType",
            "ProfileName",
            "Position",
            "SemiAxis1",
            "SemiAxis2"
        ]),
    },
    EntitySchema {
        keyword: "IFCCIRCLE",
        kind: EntityKind::Geometry,
        // IfcConic(Position) + IfcCircle(Radius).
        attrs: chain!(&["Position", "Radius"]),
    },
    EntitySchema {
        keyword: "IFCELLIPSE",
        kind: EntityKind::Geometry,
        // IfcConic(Position) + IfcEllipse(SemiAxis1, SemiAxis2).
        attrs: chain!(&["Position", "SemiAxis1", "SemiAxis2"]),
    },
    EntitySchema {
        keyword: "IFCLINE",
        kind: EntityKind::Geometry,
        // IfcLine(Pnt, Dir).
        attrs: chain!(&["Pnt", "Dir"]),
    },
    EntitySchema {
        keyword: "IFCVECTOR",
        kind: EntityKind::Geometry,
        // IfcVector(Orientation, Magnitude).
        attrs: chain!(&["Orientation", "Magnitude"]),
    },
    EntitySchema {
        keyword: "IFCTRIMMEDCURVE",
        kind: EntityKind::Geometry,
        // IfcBoundedCurve adds nothing; IfcTrimmedCurve(BasisCurve,
        // Trim1, Trim2, SenseAgreement, MasterRepresentation).
        attrs: chain!(&[
            "BasisCurve",
            "Trim1",
            "Trim2",
            "SenseAgreement",
            "MasterRepresentation"
        ]),
    },
    EntitySchema {
        keyword: "IFCCOMPOSITECURVE",
        kind: EntityKind::Geometry,
        // IfcCompositeCurve(Segments, SelfIntersect).
        attrs: chain!(&["Segments", "SelfIntersect"]),
    },
    EntitySchema {
        keyword: "IFCCOMPOSITECURVESEGMENT",
        kind: EntityKind::Geometry,
        // IfcSegment(Transition) + IfcCompositeCurveSegment(SameSense,
        // ParentCurve).
        attrs: chain!(&["Transition", "SameSense", "ParentCurve"]),
    },
    EntitySchema {
        keyword: "IFCSWEPTDISKSOLID",
        kind: EntityKind::Geometry,
        // IfcSweptDiskSolid(Directrix, Radius, InnerRadius, StartParam,
        // EndParam).
        attrs: chain!(&[
            "Directrix",
            "Radius",
            "InnerRadius",
            "StartParam",
            "EndParam"
        ]),
    },
    EntitySchema {
        keyword: "IFCSWEPTDISKSOLIDPOLYGONAL",
        kind: EntityKind::Geometry,
        // IfcSweptDiskSolid(5) + IfcSweptDiskSolidPolygonal(FilletRadius).
        attrs: chain!(&[
            "Directrix",
            "Radius",
            "InnerRadius",
            "StartParam",
            "EndParam",
            "FilletRadius"
        ]),
    },
    EntitySchema {
        keyword: "IFCARBITRARYPROFILEDEFWITHVOIDS",
        kind: EntityKind::Geometry,
        // IfcArbitraryClosedProfileDef(3) +
        // IfcArbitraryProfileDefWithVoids(InnerCurves).
        attrs: chain!(&["ProfileType", "ProfileName", "OuterCurve", "InnerCurves"]),
    },
    EntitySchema {
        keyword: "IFCCIRCLEHOLLOWPROFILEDEF",
        kind: EntityKind::Geometry,
        // IfcCircleProfileDef(4) + IfcCircleHollowProfileDef(WallThickness).
        attrs: chain!(&[
            "ProfileType",
            "ProfileName",
            "Position",
            "Radius",
            "WallThickness"
        ]),
    },
    EntitySchema {
        keyword: "IFCCOMPOSITEPROFILEDEF",
        kind: EntityKind::Geometry,
        // IfcProfileDef(2) + IfcCompositeProfileDef(Profiles, Label).
        attrs: chain!(&["ProfileType", "ProfileName", "Profiles", "Label"]),
    },
    EntitySchema {
        keyword: "IFCINDEXEDPOLYCURVE",
        kind: EntityKind::Geometry,
        // IfcBoundedCurve adds nothing; IfcIndexedPolyCurve(Points,
        // Segments, SelfIntersect).
        attrs: chain!(&["Points", "Segments", "SelfIntersect"]),
    },
    EntitySchema {
        keyword: "IFCCARTESIANPOINTLIST2D",
        kind: EntityKind::Geometry,
        // IfcCartesianPointList adds nothing; (CoordList).
        attrs: chain!(&["CoordList"]),
    },
    EntitySchema {
        keyword: "IFCRECTANGLEHOLLOWPROFILEDEF",
        kind: EntityKind::Geometry,
        // IfcRectangleProfileDef(5) + IfcRectangleHollowProfileDef(
        // WallThickness, InnerFilletRadius, OuterFilletRadius).
        attrs: chain!(&[
            "ProfileType",
            "ProfileName",
            "Position",
            "XDim",
            "YDim",
            "WallThickness",
            "InnerFilletRadius",
            "OuterFilletRadius"
        ]),
    },
    // ---- Boolean results / half spaces ----
    EntitySchema {
        keyword: "IFCBOOLEANRESULT",
        kind: EntityKind::Geometry,
        // IfcGeometricRepresentationItem adds nothing;
        // IfcBooleanResult(Operator, FirstOperand, SecondOperand).
        attrs: chain!(&["Operator", "FirstOperand", "SecondOperand"]),
    },
    EntitySchema {
        keyword: "IFCBOOLEANCLIPPINGRESULT",
        kind: EntityKind::Geometry,
        // IfcBooleanClippingResult adds no serialised attributes.
        attrs: chain!(&["Operator", "FirstOperand", "SecondOperand"]),
    },
    EntitySchema {
        keyword: "IFCHALFSPACESOLID",
        kind: EntityKind::Geometry,
        // IfcHalfSpaceSolid(BaseSurface, AgreementFlag).
        attrs: chain!(&["BaseSurface", "AgreementFlag"]),
    },
    EntitySchema {
        keyword: "IFCPOLYGONALBOUNDEDHALFSPACE",
        kind: EntityKind::Geometry,
        // IfcHalfSpaceSolid(2) + IfcPolygonalBoundedHalfSpace(Position,
        // PolygonalBoundary).
        attrs: chain!(&[
            "BaseSurface",
            "AgreementFlag",
            "Position",
            "PolygonalBoundary"
        ]),
    },
    EntitySchema {
        keyword: "IFCSECTIONEDSOLIDHORIZONTAL",
        kind: EntityKind::Geometry,
        // IfcSectionedSolid(Directrix, CrossSections) +
        // IfcSectionedSolidHorizontal(CrossSectionPositions).
        attrs: chain!(&["Directrix", "CrossSections", "CrossSectionPositions"]),
    },
    EntitySchema {
        keyword: "IFCAXIS2PLACEMENTLINEAR",
        kind: EntityKind::Geometry,
        // IfcPlacement(Location) + IfcAxis2PlacementLinear(Axis,
        // RefDirection).
        attrs: chain!(&["Location", "Axis", "RefDirection"]),
    },
    EntitySchema {
        keyword: "IFCPOINTBYDISTANCEEXPRESSION",
        kind: EntityKind::Geometry,
        // IfcPoint adds nothing; IfcPointByDistanceExpression(
        // DistanceAlong, OffsetLateral, OffsetVertical,
        // OffsetLongitudinal, BasisCurve).
        attrs: chain!(&[
            "DistanceAlong",
            "OffsetLateral",
            "OffsetVertical",
            "OffsetLongitudinal",
            "BasisCurve"
        ]),
    },
    EntitySchema {
        keyword: "IFCCSGSOLID",
        kind: EntityKind::Geometry,
        // IfcCsgSolid(TreeRootExpression).
        attrs: chain!(&["TreeRootExpression"]),
    },
    EntitySchema {
        keyword: "IFCBLOCK",
        kind: EntityKind::Geometry,
        // IfcCsgPrimitive3D(Position) + IfcBlock(XLength, YLength,
        // ZLength).
        attrs: chain!(&["Position", "XLength", "YLength", "ZLength"]),
    },
    EntitySchema {
        keyword: "IFCRECTANGULARPYRAMID",
        kind: EntityKind::Geometry,
        // IfcCsgPrimitive3D(Position) + IfcRectangularPyramid(XLength,
        // YLength, Height).
        attrs: chain!(&["Position", "XLength", "YLength", "Height"]),
    },
    EntitySchema {
        keyword: "IFCRIGHTCIRCULARCONE",
        kind: EntityKind::Geometry,
        // IfcCsgPrimitive3D(Position) + IfcRightCircularCone(Height,
        // BottomRadius).
        attrs: chain!(&["Position", "Height", "BottomRadius"]),
    },
    EntitySchema {
        keyword: "IFCRIGHTCIRCULARCYLINDER",
        kind: EntityKind::Geometry,
        // IfcCsgPrimitive3D(Position) + IfcRightCircularCylinder(Height,
        // Radius).
        attrs: chain!(&["Position", "Height", "Radius"]),
    },
    EntitySchema {
        keyword: "IFCSPHERE",
        kind: EntityKind::Geometry,
        // IfcCsgPrimitive3D(Position) + IfcSphere(Radius).
        attrs: chain!(&["Position", "Radius"]),
    },
    EntitySchema {
        keyword: "IFCBOXEDHALFSPACE",
        kind: EntityKind::Geometry,
        // IfcHalfSpaceSolid(2) + IfcBoxedHalfSpace(Enclosure).
        attrs: chain!(&["BaseSurface", "AgreementFlag", "Enclosure"]),
    },
    EntitySchema {
        keyword: "IFCBOUNDINGBOX",
        kind: EntityKind::Geometry,
        // IfcBoundingBox(Corner, XDim, YDim, ZDim).
        attrs: chain!(&["Corner", "XDim", "YDim", "ZDim"]),
    },
    EntitySchema {
        keyword: "IFCPLANE",
        kind: EntityKind::Geometry,
        // IfcElementarySurface(Position); IfcPlane adds nothing.
        attrs: chain!(&["Position"]),
    },
    // ---- Property / quantity definition relationships ----
    EntitySchema {
        keyword: "IFCRELDEFINESBYPROPERTIES",
        kind: EntityKind::RelDefinesByProperties,
        // IfcRoot + IfcRelDefines(none) + RelatedObjects,
        // RelatingPropertyDefinition (an IfcPropertySetDefinitionSelect:
        // one definition or a set of definitions).
        attrs: chain!(ROOT, &["RelatedObjects", "RelatingPropertyDefinition"]),
    },
    EntitySchema {
        keyword: "IFCRELDEFINESBYTYPE",
        kind: EntityKind::RelDefinesByType,
        // IfcRoot + IfcRelDefines(none) + RelatedObjects, RelatingType.
        attrs: chain!(ROOT, &["RelatedObjects", "RelatingType"]),
    },
    // ---- Property sets ----
    EntitySchema {
        keyword: "IFCPROPERTYSET",
        kind: EntityKind::Other,
        // IfcRoot + IfcPropertyDefinition(none) +
        // IfcPropertySetDefinition(none) + IfcPropertySet(HasProperties).
        attrs: chain!(ROOT, &["HasProperties"]),
    },
    EntitySchema {
        keyword: "IFCPROPERTYSINGLEVALUE",
        kind: EntityKind::Other,
        // IfcProperty(Name, Description) + IfcSimpleProperty(none) +
        // IfcPropertySingleValue(NominalValue, Unit).
        attrs: chain!(PROPERTY_HEAD, &["NominalValue", "Unit"]),
    },
    EntitySchema {
        keyword: "IFCPROPERTYENUMERATEDVALUE",
        kind: EntityKind::Other,
        // IfcProperty(2) + IfcPropertyEnumeratedValue(EnumerationValues,
        // EnumerationReference).
        attrs: chain!(
            PROPERTY_HEAD,
            &["EnumerationValues", "EnumerationReference"]
        ),
    },
    EntitySchema {
        keyword: "IFCPROPERTYENUMERATION",
        kind: EntityKind::Other,
        // IfcPropertyAbstraction(none) + IfcPropertyEnumeration(Name,
        // EnumerationValues, Unit).
        attrs: chain!(&["Name", "EnumerationValues", "Unit"]),
    },
    EntitySchema {
        keyword: "IFCPROPERTYBOUNDEDVALUE",
        kind: EntityKind::Other,
        // IfcProperty(2) + IfcPropertyBoundedValue(UpperBoundValue,
        // LowerBoundValue, Unit, SetPointValue).
        attrs: chain!(
            PROPERTY_HEAD,
            &[
                "UpperBoundValue",
                "LowerBoundValue",
                "Unit",
                "SetPointValue"
            ]
        ),
    },
    EntitySchema {
        keyword: "IFCPROPERTYLISTVALUE",
        kind: EntityKind::Other,
        // IfcProperty(2) + IfcPropertyListValue(ListValues, Unit).
        attrs: chain!(PROPERTY_HEAD, &["ListValues", "Unit"]),
    },
    EntitySchema {
        keyword: "IFCPROPERTYTABLEVALUE",
        kind: EntityKind::Other,
        // IfcProperty(2) + IfcPropertyTableValue(DefiningValues,
        // DefinedValues, Expression, DefiningUnit, DefinedUnit,
        // CurveInterpolation).
        attrs: chain!(
            PROPERTY_HEAD,
            &[
                "DefiningValues",
                "DefinedValues",
                "Expression",
                "DefiningUnit",
                "DefinedUnit",
                "CurveInterpolation"
            ]
        ),
    },
    EntitySchema {
        keyword: "IFCPROPERTYREFERENCEVALUE",
        kind: EntityKind::Other,
        // IfcProperty(2) + IfcPropertyReferenceValue(UsageName,
        // PropertyReference).
        attrs: chain!(PROPERTY_HEAD, &["UsageName", "PropertyReference"]),
    },
    EntitySchema {
        keyword: "IFCCOMPLEXPROPERTY",
        kind: EntityKind::Other,
        // IfcProperty(2) + IfcComplexProperty(UsageName, HasProperties).
        attrs: chain!(PROPERTY_HEAD, &["UsageName", "HasProperties"]),
    },
    // ---- Quantity sets ----
    EntitySchema {
        keyword: "IFCELEMENTQUANTITY",
        kind: EntityKind::Other,
        // IfcRoot + IfcQuantitySet(none) +
        // IfcElementQuantity(MethodOfMeasurement, Quantities).
        attrs: chain!(ROOT, &["MethodOfMeasurement", "Quantities"]),
    },
    EntitySchema {
        keyword: "IFCQUANTITYLENGTH",
        kind: EntityKind::Other,
        attrs: chain!(
            QUANTITY_HEAD,
            SIMPLE_QUANTITY_UNIT,
            &["LengthValue", "Formula"]
        ),
    },
    EntitySchema {
        keyword: "IFCQUANTITYAREA",
        kind: EntityKind::Other,
        attrs: chain!(
            QUANTITY_HEAD,
            SIMPLE_QUANTITY_UNIT,
            &["AreaValue", "Formula"]
        ),
    },
    EntitySchema {
        keyword: "IFCQUANTITYVOLUME",
        kind: EntityKind::Other,
        attrs: chain!(
            QUANTITY_HEAD,
            SIMPLE_QUANTITY_UNIT,
            &["VolumeValue", "Formula"]
        ),
    },
    EntitySchema {
        keyword: "IFCQUANTITYCOUNT",
        kind: EntityKind::Other,
        attrs: chain!(
            QUANTITY_HEAD,
            SIMPLE_QUANTITY_UNIT,
            &["CountValue", "Formula"]
        ),
    },
    EntitySchema {
        keyword: "IFCQUANTITYWEIGHT",
        kind: EntityKind::Other,
        attrs: chain!(
            QUANTITY_HEAD,
            SIMPLE_QUANTITY_UNIT,
            &["WeightValue", "Formula"]
        ),
    },
    EntitySchema {
        keyword: "IFCQUANTITYTIME",
        kind: EntityKind::Other,
        attrs: chain!(
            QUANTITY_HEAD,
            SIMPLE_QUANTITY_UNIT,
            &["TimeValue", "Formula"]
        ),
    },
    EntitySchema {
        keyword: "IFCPHYSICALCOMPLEXQUANTITY",
        kind: EntityKind::Other,
        // IfcPhysicalQuantity(2) + IfcPhysicalComplexQuantity(
        // HasQuantities, Discrimination, Quality, Usage).
        attrs: chain!(
            QUANTITY_HEAD,
            &["HasQuantities", "Discrimination", "Quality", "Usage"]
        ),
    },
    // ---- Type objects (fixture slice; HasPropertySets is index 5 for
    // every IfcTypeObject subtype since ApplicableOccurrence /
    // HasPropertySets follow IfcRoot directly) ----
    EntitySchema {
        keyword: "IFCWALLTYPE",
        kind: EntityKind::TypeObject,
        // IfcRoot + IfcTypeObject(2) + IfcTypeProduct(2) +
        // IfcElementType(1) + IfcWallType(PredefinedType).
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCWINDOWTYPE",
        kind: EntityKind::TypeObject,
        // … + IfcWindowType(PredefinedType, PartitioningType,
        // ParameterTakesPrecedence, UserDefinedPartitioningType).
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &[
                "PredefinedType",
                "PartitioningType",
                "ParameterTakesPrecedence",
                "UserDefinedPartitioningType"
            ]
        ),
    },
    EntitySchema {
        keyword: "IFCSANITARYTERMINALTYPE",
        kind: EntityKind::TypeObject,
        // IfcFlowTerminalType / IfcDistributionFlowElementType add no
        // serialised attributes over IfcElementType.
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCCOLUMNTYPE",
        kind: EntityKind::TypeObject,
        // IfcElementType chain + own PredefinedType.
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCBEAMTYPE",
        kind: EntityKind::TypeObject,
        // IfcElementType chain + own PredefinedType.
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCSLABTYPE",
        kind: EntityKind::TypeObject,
        // IfcElementType chain + own PredefinedType.
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCBUILDINGELEMENTPROXYTYPE",
        kind: EntityKind::TypeObject,
        // IfcElementType chain + own PredefinedType.
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCCOVERINGTYPE",
        kind: EntityKind::TypeObject,
        // IfcElementType chain + own PredefinedType.
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCMEMBERTYPE",
        kind: EntityKind::TypeObject,
        // IfcElementType chain + own PredefinedType.
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCPLATETYPE",
        kind: EntityKind::TypeObject,
        // IfcElementType chain + own PredefinedType.
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCRAILINGTYPE",
        kind: EntityKind::TypeObject,
        // IfcElementType chain + own PredefinedType.
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCROOFTYPE",
        kind: EntityKind::TypeObject,
        // IfcElementType chain + own PredefinedType.
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCSTAIRTYPE",
        kind: EntityKind::TypeObject,
        // IfcElementType chain + own PredefinedType.
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &["PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCDOORTYPE",
        kind: EntityKind::TypeObject,
        // … + IfcDoorType(PredefinedType, OperationType,
        // ParameterTakesPrecedence, UserDefinedOperationType).
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &[
                "PredefinedType",
                "OperationType",
                "ParameterTakesPrecedence",
                "UserDefinedOperationType"
            ]
        ),
    },
    EntitySchema {
        keyword: "IFCFURNITURETYPE",
        kind: EntityKind::TypeObject,
        // IfcFurnishingElementType adds nothing over IfcElementType;
        // IfcFurnitureType(AssemblyPlace, PredefinedType).
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            ELEMENT_TYPE_TAIL,
            &["AssemblyPlace", "PredefinedType"]
        ),
    },
    EntitySchema {
        keyword: "IFCDOORSTYLE",
        kind: EntityKind::TypeObject,
        // IfcDoorStyle is a direct IfcTypeProduct subtype (no
        // IfcElementType): + (OperationType, ConstructionType,
        // ParameterTakesPrecedence, Sizeable).
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            &[
                "OperationType",
                "ConstructionType",
                "ParameterTakesPrecedence",
                "Sizeable"
            ]
        ),
    },
    EntitySchema {
        keyword: "IFCWINDOWSTYLE",
        kind: EntityKind::TypeObject,
        // IfcWindowStyle is a direct IfcTypeProduct subtype: +
        // (ConstructionType, OperationType, ParameterTakesPrecedence,
        // Sizeable).
        attrs: chain!(
            ROOT,
            TYPE_OBJECT_TAIL,
            TYPE_PRODUCT_TAIL,
            &[
                "ConstructionType",
                "OperationType",
                "ParameterTakesPrecedence",
                "Sizeable"
            ]
        ),
    },
    // ---- Openings (voids / fills) ----
    EntitySchema {
        keyword: "IFCRELVOIDSELEMENT",
        kind: EntityKind::RelVoids,
        // IfcRoot + IfcRelDecomposes(none) + IfcRelVoidsElement(
        // RelatingBuildingElement, RelatedOpeningElement).
        attrs: chain!(ROOT, &["RelatingBuildingElement", "RelatedOpeningElement"]),
    },
    EntitySchema {
        keyword: "IFCRELFILLSELEMENT",
        kind: EntityKind::RelFills,
        // IfcRoot + IfcRelConnects(none) + IfcRelFillsElement(
        // RelatingOpeningElement, RelatedBuildingElement).
        attrs: chain!(ROOT, &["RelatingOpeningElement", "RelatedBuildingElement"]),
    },
    // ---- Material associations ----
    EntitySchema {
        keyword: "IFCRELASSOCIATESMATERIAL",
        kind: EntityKind::RelAssociatesMaterial,
        // IfcRoot + IfcRelAssociates(RelatedObjects) +
        // IfcRelAssociatesMaterial(RelatingMaterial).
        attrs: chain!(ROOT, &["RelatedObjects", "RelatingMaterial"]),
    },
    EntitySchema {
        keyword: "IFCMATERIAL",
        kind: EntityKind::Other,
        // IfcMaterialDefinition adds nothing; IfcMaterial(Name,
        // Description, Category).
        attrs: chain!(&["Name", "Description", "Category"]),
    },
    EntitySchema {
        keyword: "IFCMATERIALLIST",
        kind: EntityKind::Other,
        attrs: chain!(&["Materials"]),
    },
    EntitySchema {
        keyword: "IFCMATERIALLAYER",
        kind: EntityKind::Other,
        // IfcMaterialLayer(Material, LayerThickness, IsVentilated,
        // Name, Description, Category, Priority).
        attrs: chain!(&[
            "Material",
            "LayerThickness",
            "IsVentilated",
            "Name",
            "Description",
            "Category",
            "Priority"
        ]),
    },
    EntitySchema {
        keyword: "IFCMATERIALLAYERWITHOFFSETS",
        kind: EntityKind::Other,
        // IfcMaterialLayer(7) + (OffsetDirection, OffsetValues).
        attrs: chain!(&[
            "Material",
            "LayerThickness",
            "IsVentilated",
            "Name",
            "Description",
            "Category",
            "Priority",
            "OffsetDirection",
            "OffsetValues"
        ]),
    },
    EntitySchema {
        keyword: "IFCMATERIALLAYERSET",
        kind: EntityKind::Other,
        // IfcMaterialLayerSet(MaterialLayers, LayerSetName, Description).
        attrs: chain!(&["MaterialLayers", "LayerSetName", "Description"]),
    },
    EntitySchema {
        keyword: "IFCMATERIALLAYERSETUSAGE",
        kind: EntityKind::Other,
        // IfcMaterialUsageDefinition adds nothing;
        // IfcMaterialLayerSetUsage(ForLayerSet, LayerSetDirection,
        // DirectionSense, OffsetFromReferenceLine, ReferenceExtent).
        attrs: chain!(&[
            "ForLayerSet",
            "LayerSetDirection",
            "DirectionSense",
            "OffsetFromReferenceLine",
            "ReferenceExtent"
        ]),
    },
    EntitySchema {
        keyword: "IFCMATERIALPROFILE",
        kind: EntityKind::Other,
        // IfcMaterialProfile(Name, Description, Material, Profile,
        // Priority, Category).
        attrs: chain!(&[
            "Name",
            "Description",
            "Material",
            "Profile",
            "Priority",
            "Category"
        ]),
    },
    EntitySchema {
        keyword: "IFCMATERIALPROFILEWITHOFFSETS",
        kind: EntityKind::Other,
        // IfcMaterialProfile(6) + (OffsetValues).
        attrs: chain!(&[
            "Name",
            "Description",
            "Material",
            "Profile",
            "Priority",
            "Category",
            "OffsetValues"
        ]),
    },
    EntitySchema {
        keyword: "IFCMATERIALPROFILESET",
        kind: EntityKind::Other,
        // IfcMaterialProfileSet(Name, Description, MaterialProfiles,
        // CompositeProfile).
        attrs: chain!(&[
            "Name",
            "Description",
            "MaterialProfiles",
            "CompositeProfile"
        ]),
    },
    EntitySchema {
        keyword: "IFCMATERIALPROFILESETUSAGE",
        kind: EntityKind::Other,
        // IfcMaterialProfileSetUsage(ForProfileSet, CardinalPoint,
        // ReferenceExtent).
        attrs: chain!(&["ForProfileSet", "CardinalPoint", "ReferenceExtent"]),
    },
    EntitySchema {
        keyword: "IFCMATERIALPROFILESETUSAGETAPERING",
        kind: EntityKind::Other,
        // IfcMaterialProfileSetUsage(3) + (ForProfileEndSet,
        // CardinalEndPoint).
        attrs: chain!(&[
            "ForProfileSet",
            "CardinalPoint",
            "ReferenceExtent",
            "ForProfileEndSet",
            "CardinalEndPoint"
        ]),
    },
    EntitySchema {
        keyword: "IFCMATERIALCONSTITUENT",
        kind: EntityKind::Other,
        // IfcMaterialConstituent(Name, Description, Material, Fraction,
        // Category).
        attrs: chain!(&["Name", "Description", "Material", "Fraction", "Category"]),
    },
    EntitySchema {
        keyword: "IFCMATERIALCONSTITUENTSET",
        kind: EntityKind::Other,
        // IfcMaterialConstituentSet(Name, Description,
        // MaterialConstituents).
        attrs: chain!(&["Name", "Description", "MaterialConstituents"]),
    },
    // ---- Georeferencing ----
    EntitySchema {
        keyword: "IFCPROJECTEDCRS",
        kind: EntityKind::Other,
        // IfcCoordinateReferenceSystem(Name, Description,
        // GeodeticDatum, VerticalDatum) + IfcProjectedCRS(
        // MapProjection, MapZone, MapUnit).
        attrs: chain!(&[
            "Name",
            "Description",
            "GeodeticDatum",
            "VerticalDatum",
            "MapProjection",
            "MapZone",
            "MapUnit"
        ]),
    },
    EntitySchema {
        keyword: "IFCMAPCONVERSION",
        kind: EntityKind::Other,
        // IfcCoordinateOperation(SourceCRS, TargetCRS) +
        // IfcMapConversion(Eastings, Northings, OrthogonalHeight,
        // XAxisAbscissa, XAxisOrdinate, Scale).
        attrs: chain!(&[
            "SourceCRS",
            "TargetCRS",
            "Eastings",
            "Northings",
            "OrthogonalHeight",
            "XAxisAbscissa",
            "XAxisOrdinate",
            "Scale"
        ]),
    },
    // ---- Units ----
    EntitySchema {
        keyword: "IFCUNITASSIGNMENT",
        kind: EntityKind::Other,
        attrs: chain!(&["Units"]),
    },
    EntitySchema {
        keyword: "IFCSIUNIT",
        kind: EntityKind::Other,
        // IfcNamedUnit(Dimensions, UnitType) + IfcSIUnit(Prefix, Name);
        // Dimensions is re-derived on IfcSIUnit so the wire carries `*`.
        attrs: chain!(&["Dimensions", "UnitType", "Prefix", "Name"]),
    },
    EntitySchema {
        keyword: "IFCCONVERSIONBASEDUNIT",
        kind: EntityKind::Other,
        // IfcNamedUnit(2) + IfcConversionBasedUnit(Name, ConversionFactor).
        attrs: chain!(&["Dimensions", "UnitType", "Name", "ConversionFactor"]),
    },
    EntitySchema {
        keyword: "IFCMEASUREWITHUNIT",
        kind: EntityKind::Other,
        attrs: chain!(&["ValueComponent", "UnitComponent"]),
    },
];

/// Look up the [`EntitySchema`] for an entity keyword
/// (case-insensitive). Returns `None` for keywords outside the typed
/// core slice — the caller keeps the positional view in that case.
pub fn schema_of(keyword: &str) -> Option<&'static EntitySchema> {
    let want = keyword.to_ascii_uppercase();
    SCHEMA.iter().find(|s| s.keyword == want)
}

/// One resolvable unit dimension: the `IfcUnitEnum` selector, the SI
/// unit name that anchors it, the factor from that SI unit to the
/// reference unit the public functions promise, and whether an SI
/// prefix is meaningful on the anchor.
///
/// `prefix_ok` is `false` for the exponent-carrying SI names
/// (`SQUARE_METRE`, `CUBIC_METRE`): whether a prefix there scales the
/// base length (mm² = 10⁻⁶ m²) or the derived unit (10⁻³ m²) is not
/// stated by the staged schema text, so a prefixed area/volume SI unit
/// resolves to `None` instead of guessing.
struct UnitDimension {
    unit_type: &'static str,
    si_name: &'static str,
    si_to_reference: f64,
    prefix_ok: bool,
}

const LENGTH_DIMENSION: UnitDimension = UnitDimension {
    unit_type: "LENGTHUNIT",
    si_name: "METRE",
    si_to_reference: 1.0,
    prefix_ok: true,
};
const PLANE_ANGLE_DIMENSION: UnitDimension = UnitDimension {
    unit_type: "PLANEANGLEUNIT",
    si_name: "RADIAN",
    si_to_reference: 1.0,
    prefix_ok: true,
};
const AREA_DIMENSION: UnitDimension = UnitDimension {
    unit_type: "AREAUNIT",
    si_name: "SQUARE_METRE",
    si_to_reference: 1.0,
    prefix_ok: false,
};
const VOLUME_DIMENSION: UnitDimension = UnitDimension {
    unit_type: "VOLUMEUNIT",
    si_name: "CUBIC_METRE",
    si_to_reference: 1.0,
    prefix_ok: false,
};
/// The SI mass name is GRAM (a kilogram is `.KILO.` + `.GRAM.`), and
/// the reference the public function promises is the kilogram.
const MASS_DIMENSION: UnitDimension = UnitDimension {
    unit_type: "MASSUNIT",
    si_name: "GRAM",
    si_to_reference: 1e-3,
    prefix_ok: true,
};
const TIME_DIMENSION: UnitDimension = UnitDimension {
    unit_type: "TIMEUNIT",
    si_name: "SECOND",
    si_to_reference: 1.0,
    prefix_ok: true,
};

/// Look up the [`UnitDimension`] for an `IfcUnitEnum` literal.
fn dimension_of(unit_type: &str) -> Option<&'static UnitDimension> {
    Some(match unit_type {
        "LENGTHUNIT" => &LENGTH_DIMENSION,
        "PLANEANGLEUNIT" => &PLANE_ANGLE_DIMENSION,
        "AREAUNIT" => &AREA_DIMENSION,
        "VOLUMEUNIT" => &VOLUME_DIMENSION,
        "MASSUNIT" => &MASS_DIMENSION,
        "TIMEUNIT" => &TIME_DIMENSION,
        _ => return None,
    })
}

/// The §8.11.3.11 walk: `IfcProject.UnitsInContext` →
/// `IfcUnitAssignment.Units : SET [1:?] OF IfcUnit` → the (at most
/// one) unit of the requested dimension, resolved to reference units.
fn project_unit_scale(step: &StepFile, dim: &UnitDimension) -> Option<f64> {
    // IfcProject: IfcRoot(4) + IfcContext(ObjectType, LongName, Phase,
    // RepresentationContexts, UnitsInContext) → UnitsInContext index 8.
    let project = step
        .instances
        .values()
        .find(|i| i.keyword == "IFCPROJECT")?;
    let assignment_id = project.args.get(8).and_then(Value::as_reference)?;
    let assignment = step.get(assignment_id)?;
    if assignment.keyword != "IFCUNITASSIGNMENT" {
        return None;
    }
    let units = assignment.args.first().and_then(Value::as_list)?;
    for unit in units {
        let Some(uid) = unit.as_reference() else {
            continue;
        };
        if let Some(scale) = resolve_named_unit(step, uid, dim) {
            return Some(scale);
        }
    }
    None
}

/// Resolve one named unit to reference units if it is of the requested
/// dimension.
fn resolve_named_unit(step: &StepFile, unit_id: u64, dim: &UnitDimension) -> Option<f64> {
    let unit = step.get(unit_id)?;
    match unit.keyword.as_str() {
        // IfcSIUnit(Dimensions [derived, `*` on the wire], UnitType,
        // Prefix, Name).
        "IFCSIUNIT" => {
            if unit.args.get(1).and_then(Value::as_enum) != Some(dim.unit_type)
                || unit.args.get(3).and_then(Value::as_enum) != Some(dim.si_name)
            {
                return None;
            }
            let prefix = match unit.args.get(2).and_then(Value::as_enum) {
                Some(prefix) => {
                    if !dim.prefix_ok {
                        // Prefix-on-exponent semantics undocumented in
                        // the staged set — refuse rather than guess.
                        return None;
                    }
                    si_prefix_multiplier(prefix)?
                }
                None => 1.0,
            };
            Some(prefix * dim.si_to_reference)
        }
        // IfcConversionBasedUnit(Dimensions, UnitType, Name,
        // ConversionFactor : IfcMeasureWithUnit).
        "IFCCONVERSIONBASEDUNIT" | "IFCCONVERSIONBASEDUNITWITHOFFSET" => {
            if unit.args.get(1).and_then(Value::as_enum) != Some(dim.unit_type) {
                return None;
            }
            let mwu_id = unit.args.get(3).and_then(Value::as_reference)?;
            let mwu = step.get(mwu_id)?;
            if mwu.keyword != "IFCMEASUREWITHUNIT" {
                return None;
            }
            // ValueComponent : IfcValue — a plain real or a typed
            // measure wrapper (e.g. IFCRATIOMEASURE(0.3048)).
            let value = match mwu.args.first()? {
                v @ (Value::Real(_) | Value::Integer(_)) => v.as_number()?,
                Value::Typed { args, .. } => args.first().and_then(Value::as_number)?,
                _ => return None,
            };
            let base_id = mwu.args.get(1).and_then(Value::as_reference)?;
            Some(value * resolve_named_unit(step, base_id, dim)?)
        }
        _ => None,
    }
}

/// Resolve one named unit (`IfcSIUnit` / `IfcConversionBasedUnit`
/// chain) to reference units, given the `IfcUnitEnum` dimension
/// literal it must declare (`"LENGTHUNIT"`, `"AREAUNIT"`,
/// `"VOLUMEUNIT"`, `"MASSUNIT"`, `"TIMEUNIT"`, `"PLANEANGLEUNIT"`).
///
/// The reference units are those of the per-dimension model functions:
/// metres / m² / m³ / **kilograms** / seconds / radians. This is the
/// per-instance analogue of [`length_unit_scale`] — quantity sets use
/// it for the optional `IfcPhysicalSimpleQuantity.Unit` override.
/// Returns `None` for unknown dimensions, mismatched unit types, and
/// the undocumented prefixed-area/volume SI forms.
pub fn named_unit_scale(step: &StepFile, unit_id: u64, unit_type: &str) -> Option<f64> {
    resolve_named_unit(step, unit_id, dimension_of(unit_type)?)
}

/// Metres per model length unit, resolved from the project's global
/// unit assignment.
///
/// The walk is `IfcProject.UnitsInContext` →
/// `IfcUnitAssignment.Units : SET [1:?] OF IfcUnit` → the unit whose
/// `UnitType` is `.LENGTHUNIT.` (the §8.11.3.11 WHERE rule guarantees
/// at most one per assignment):
///
/// * `IfcSIUnit(Dimensions*, UnitType, Prefix, Name)` with Name
///   `.METRE.` — the scale is the SI prefix multiplier (`.MILLI.` →
///   10⁻³, absent prefix → 1.0).
/// * `IfcConversionBasedUnit(Dimensions, UnitType, Name,
///   ConversionFactor)` — the factor's
///   `IfcMeasureWithUnit(ValueComponent, UnitComponent)` value times
///   the (recursively resolved) SI unit it is expressed in.
///
/// Returns `None` when the model has no resolvable length unit (the
/// caller keeps raw model units — the decoder does not rescale).
pub fn length_unit_scale(step: &StepFile) -> Option<f64> {
    project_unit_scale(step, &LENGTH_DIMENSION)
}

/// Resolve the model's plane-angle unit to **radians per model angle
/// unit** from the project's `IfcUnitAssignment` — the same §8.11.3.11
/// walk as [`length_unit_scale`], selecting the `.PLANEANGLEUNIT.`
/// entry instead.
///
/// Returns `Some(1.0)` for a plain SI radian, the conversion factor
/// for an `IfcConversionBasedUnit` (a degree-based model yields
/// ≈ 0.017453…), and `None` when the model declares no resolvable
/// plane-angle unit (callers should then assume radians — the
/// EXPRESS default parameterisation). Conic trim parameters
/// (`IfcParameterValue` on an `IfcTrimmedCurve`) and revolution angles
/// are expressed in this unit.
pub fn plane_angle_unit_scale(step: &StepFile) -> Option<f64> {
    project_unit_scale(step, &PLANE_ANGLE_DIMENSION)
}

/// Square metres per model area unit (`.AREAUNIT.`), from the
/// project's unit assignment — the area analogue of
/// [`length_unit_scale`].
///
/// A **prefixed** `.SQUARE_METRE.` SI unit resolves to `None`: whether
/// the prefix scales the base length or the derived unit is not stated
/// by the staged schema text, so no guess is made. Conversion-based
/// area chains (e.g. square feet over an SI base) resolve normally.
pub fn area_unit_scale(step: &StepFile) -> Option<f64> {
    project_unit_scale(step, &AREA_DIMENSION)
}

/// Cubic metres per model volume unit (`.VOLUMEUNIT.`) — the volume
/// analogue of [`length_unit_scale`], with the same prefixed-SI-form
/// refusal as [`area_unit_scale`].
pub fn volume_unit_scale(step: &StepFile) -> Option<f64> {
    project_unit_scale(step, &VOLUME_DIMENSION)
}

/// **Kilograms** per model mass unit (`.MASSUNIT.`). The SI unit name
/// is `.GRAM.` (a kilogram model declares `.KILO.` + `.GRAM.`), so an
/// unprefixed gram model yields 10⁻³.
pub fn mass_unit_scale(step: &StepFile) -> Option<f64> {
    project_unit_scale(step, &MASS_DIMENSION)
}

/// Seconds per model time unit (`.TIMEUNIT.`).
pub fn time_unit_scale(step: &StepFile) -> Option<f64> {
    project_unit_scale(step, &TIME_DIMENSION)
}

/// SI prefix multiplier (ISO 80000 decimal prefixes, as enumerated by
/// the EXPRESS `IfcSIPrefix` type).
fn si_prefix_multiplier(prefix: &str) -> Option<f64> {
    Some(match prefix {
        "EXA" => 1e18,
        "PETA" => 1e15,
        "TERA" => 1e12,
        "GIGA" => 1e9,
        "MEGA" => 1e6,
        "KILO" => 1e3,
        "HECTO" => 1e2,
        "DECA" => 1e1,
        "DECI" => 1e-1,
        "CENTI" => 1e-2,
        "MILLI" => 1e-3,
        "MICRO" => 1e-6,
        "NANO" => 1e-9,
        "PICO" => 1e-12,
        "FEMTO" => 1e-15,
        "ATTO" => 1e-18,
        _ => return None,
    })
}

/// A schema-typed view of one parsed instance: positional [`Value`]s
/// resolved to attribute names per the IFC 4 EXPRESS schema.
///
/// Construct via [`TypedEntity::new`] (returns `None` when the keyword
/// is outside the typed slice). The view borrows the instance; cloning
/// it is cheap (two references).
#[derive(Debug, Clone, Copy)]
pub struct TypedEntity<'a> {
    inst: &'a ParsedInstance,
    schema: &'static EntitySchema,
}

impl<'a> TypedEntity<'a> {
    /// Build a typed view, or `None` if the instance's keyword has no
    /// schema entry in the core slice.
    pub fn new(inst: &'a ParsedInstance) -> Option<Self> {
        schema_of(&inst.keyword).map(|schema| Self { inst, schema })
    }

    /// The underlying parsed instance.
    pub fn instance(&self) -> &'a ParsedInstance {
        self.inst
    }

    /// The instance id (`#id`).
    pub fn id(&self) -> u64 {
        self.inst.id
    }

    /// The entity keyword (upper-cased, `"IFCWALL"`).
    pub fn keyword(&self) -> &'a str {
        &self.inst.keyword
    }

    /// The entity's structural role.
    pub fn kind(&self) -> EntityKind {
        self.schema.kind
    }

    /// Resolve an attribute by EXPRESS name (case-sensitive — names
    /// match the schema declaration exactly, e.g. `"GlobalId"`).
    ///
    /// Returns `None` when the name is not an attribute of this entity,
    /// or when the instance's argument list is shorter than the schema
    /// (a truncated record — the trailing optional attributes are then
    /// treated as absent).
    pub fn attr(&self, name: &str) -> Option<&'a Value> {
        let idx = self.schema.attrs.iter().position(|a| *a == name)?;
        self.inst.args.get(idx)
    }

    /// Iterate `(attribute_name, value)` pairs in serialisation order.
    /// Stops at the shorter of the schema length and the argument count
    /// (so a truncated record yields only the attributes it carries).
    pub fn attrs(&self) -> impl Iterator<Item = (&'static str, &'a Value)> {
        self.schema.attrs.iter().copied().zip(self.inst.args.iter())
    }

    /// The `GlobalId` (IfcRoot.GlobalId) as a string, when present.
    pub fn global_id(&self) -> Option<&'a str> {
        self.attr("GlobalId")?.as_str()
    }

    /// The `Name` (IfcRoot.Name) as a string, when present and set.
    pub fn name(&self) -> Option<&'a str> {
        self.attr("Name")?.as_str()
    }

    /// The `Description` (IfcRoot.Description) as a string, when set.
    pub fn description(&self) -> Option<&'a str> {
        self.attr("Description")?.as_str()
    }

    /// The `#id` of the entity's `ObjectPlacement`, when it carries one
    /// (products / spatial elements). `None` for entities without the
    /// attribute or when it is `$`.
    pub fn object_placement(&self) -> Option<u64> {
        self.attr("ObjectPlacement")?.as_reference()
    }

    /// The `#id` of the entity's shape `Representation`
    /// (`IfcProductDefinitionShape`), when present and set.
    pub fn representation(&self) -> Option<u64> {
        self.attr("Representation")?.as_reference()
    }

    /// The `PredefinedType` enum literal, when the entity declares one
    /// and it is set (e.g. `"OPENING"` on an opening element).
    pub fn predefined_type(&self) -> Option<&'a str> {
        self.attr("PredefinedType")?.as_enum()
    }

    // ---- Geometric-primitive accessors -----------------------------
    //
    // These read the `EntityKind::Geometry` slice (IfcCartesianPoint /
    // IfcDirection / IfcAxis2Placement2D/3D / IfcPolyline). Each returns
    // `None` when called on an entity that does not carry the attribute,
    // so a non-geometry view simply yields nothing.

    /// The numeric values of an aggregate attribute (a `LIST OF
    /// IfcLengthMeasure` / `IfcReal`), e.g. an `IfcCartesianPoint`'s
    /// `Coordinates` or an `IfcDirection`'s `DirectionRatios`.
    ///
    /// Each element is read through [`Value::as_number`] so an integer
    /// literal where the schema says REAL is accepted (a common writer
    /// deviation). Returns `None` when the attribute is missing or not an
    /// aggregate; non-numeric members are skipped.
    fn number_list(&self, name: &str) -> Option<Vec<f64>> {
        let list = self.attr(name)?.as_list()?;
        Some(list.iter().filter_map(Value::as_number).collect())
    }

    /// An `IfcCartesianPoint`'s `Coordinates` as a numeric vector
    /// (length 2 or 3 per the EXPRESS `LIST [1:3]`), when present.
    pub fn coordinates(&self) -> Option<Vec<f64>> {
        self.number_list("Coordinates")
    }

    /// An `IfcDirection`'s `DirectionRatios` as a numeric vector
    /// (length 2 or 3 per the EXPRESS `LIST [2:3]`), when present.
    pub fn direction_ratios(&self) -> Option<Vec<f64>> {
        self.number_list("DirectionRatios")
    }

    /// The `#id` of an `IfcPlacement`'s `Location`
    /// (`IfcAxis2Placement2D` / `IfcAxis2Placement3D` → an
    /// `IfcCartesianPoint`), when present and set.
    pub fn location(&self) -> Option<u64> {
        self.attr("Location")?.as_reference()
    }

    /// The `#id` of an `IfcAxis2Placement3D`'s `Axis`
    /// (an `IfcDirection`), when present and set.
    pub fn axis(&self) -> Option<u64> {
        self.attr("Axis")?.as_reference()
    }

    /// The `#id` of an `IfcAxis2Placement2D/3D`'s `RefDirection`
    /// (an `IfcDirection`), when present and set.
    pub fn ref_direction(&self) -> Option<u64> {
        self.attr("RefDirection")?.as_reference()
    }

    /// The `#id`s of an `IfcPolyline`'s `Points` (a `LIST OF
    /// IfcCartesianPoint`), in serialisation order, when present.
    pub fn points(&self) -> Option<Vec<u64>> {
        self.reference_list("Points")
    }

    /// The `#id`s of an `IfcShapeRepresentation`'s `Items` (a `SET OF
    /// IfcRepresentationItem`), when present.
    pub fn items(&self) -> Option<Vec<u64>> {
        self.reference_list("Items")
    }

    /// The `RepresentationIdentifier` label of an
    /// `IfcShapeRepresentation` (e.g. `"Body"`, `"Axis"`), when set.
    pub fn representation_identifier(&self) -> Option<&'a str> {
        self.attr("RepresentationIdentifier")?.as_str()
    }

    /// The `RepresentationType` label of an `IfcShapeRepresentation`
    /// (e.g. `"Tessellation"`, `"Curve2D"`), when set.
    pub fn representation_type(&self) -> Option<&'a str> {
        self.attr("RepresentationType")?.as_str()
    }

    /// The `#id` of an `IfcShapeRepresentation`'s `ContextOfItems`
    /// (an `IfcRepresentationContext`), when present and set.
    pub fn context_of_items(&self) -> Option<u64> {
        self.attr("ContextOfItems")?.as_reference()
    }

    /// The `#id` of an `IfcMappedItem`'s `MappingSource`
    /// (an `IfcRepresentationMap`), when present and set.
    pub fn mapping_source(&self) -> Option<u64> {
        self.attr("MappingSource")?.as_reference()
    }

    /// The `#id` of an `IfcMappedItem`'s `MappingTarget`
    /// (an `IfcCartesianTransformationOperator`), when present and set.
    pub fn mapping_target(&self) -> Option<u64> {
        self.attr("MappingTarget")?.as_reference()
    }

    /// The `#id` of an `IfcRepresentationMap`'s `MappingOrigin`
    /// (an `IfcAxis2Placement`), when present and set.
    pub fn mapping_origin(&self) -> Option<u64> {
        self.attr("MappingOrigin")?.as_reference()
    }

    /// The `#id` of an `IfcRepresentationMap`'s `MappedRepresentation`
    /// (an `IfcShapeRepresentation`), when present and set.
    pub fn mapped_representation(&self) -> Option<u64> {
        self.attr("MappedRepresentation")?.as_reference()
    }

    /// Resolve an aggregate-of-references attribute to the ordered list
    /// of referenced `#id`s, skipping any non-reference members.
    fn reference_list(&self, name: &str) -> Option<Vec<u64>> {
        let list = self.attr(name)?.as_list()?;
        Some(list.iter().filter_map(Value::as_reference).collect())
    }
}

/// A resolved spatial-structure model: the typed entity graph with the
/// composition + containment relationships followed into a navigable
/// tree.
///
/// Built by [`Model::from_step`]. It does not copy the instance graph —
/// it indexes into the borrowed [`StepFile`], recording the parent /
/// child edges the two structural relationships imply.
#[derive(Debug, Clone)]
pub struct Model<'a> {
    step: &'a StepFile,
    /// The `#id` of the `IfcProject` root, if exactly one is present.
    project: Option<u64>,
    /// `parent -> children` composition edges from `IfcRelAggregates`
    /// (project→site→building→storey→space).
    aggregates: BTreeMap<u64, Vec<u64>>,
    /// `spatial-structure -> contained products` edges from
    /// `IfcRelContainedInSpatialStructure`.
    contains: BTreeMap<u64, Vec<u64>>,
    /// `object -> property-set definitions` edges from
    /// `IfcRelDefinesByProperties` (property sets and quantity sets).
    defines: BTreeMap<u64, Vec<u64>>,
    /// `occurrence -> type object` edges from `IfcRelDefinesByType`
    /// (the EXPRESS `Types` inverse is `SET [0:1]` — first edge wins).
    typed_by: BTreeMap<u64, u64>,
    /// `object -> IfcMaterialSelect` edges from
    /// `IfcRelAssociatesMaterial` (first edge wins).
    materials: BTreeMap<u64, u64>,
    /// `element -> opening elements` edges from `IfcRelVoidsElement`.
    voids: BTreeMap<u64, Vec<u64>>,
    /// `opening -> voided element` back edges (one relationship per
    /// opening — `HasFillings`' dual; first edge wins).
    voided: BTreeMap<u64, u64>,
    /// `opening -> filler elements` edges from `IfcRelFillsElement`.
    fills: BTreeMap<u64, Vec<u64>>,
    /// `filler element -> opening` back edges (first edge wins).
    filler_of: BTreeMap<u64, u64>,
}

impl<'a> Model<'a> {
    /// Resolve the typed spatial model from a parsed STEP file.
    ///
    /// Classifies instances via [`schema_of`], picks the single
    /// `IfcProject` (if exactly one exists), and folds every
    /// `IfcRelAggregates` / `IfcRelContainedInSpatialStructure` record
    /// into parent→children edges. Relationship records that reference
    /// missing ids are skipped (the dangling edge is dropped, not an
    /// error — use [`StepFile::dangling_references`] for validation).
    pub fn from_step(step: &'a StepFile) -> Self {
        let mut project = None;
        let mut aggregates: BTreeMap<u64, Vec<u64>> = BTreeMap::new();
        let mut contains: BTreeMap<u64, Vec<u64>> = BTreeMap::new();
        let mut defines: BTreeMap<u64, Vec<u64>> = BTreeMap::new();
        let mut typed_by: BTreeMap<u64, u64> = BTreeMap::new();
        let mut materials: BTreeMap<u64, u64> = BTreeMap::new();
        let mut voids: BTreeMap<u64, Vec<u64>> = BTreeMap::new();
        let mut voided: BTreeMap<u64, u64> = BTreeMap::new();
        let mut fills: BTreeMap<u64, Vec<u64>> = BTreeMap::new();
        let mut filler_of: BTreeMap<u64, u64> = BTreeMap::new();

        for inst in step.instances.values() {
            let Some(view) = TypedEntity::new(inst) else {
                continue;
            };
            match view.kind() {
                EntityKind::Project => {
                    // Exactly-one semantics: a second project clears the
                    // slot to `None` (ambiguous root).
                    project = match project {
                        None => Some(inst.id),
                        Some(_) => None,
                    };
                }
                EntityKind::RelAggregates => {
                    if let (Some(rel), Some(items)) =
                        (view.attr("RelatingObject"), view.attr("RelatedObjects"))
                    {
                        if let Some(parent) = rel.as_reference() {
                            let kids = aggregates.entry(parent).or_default();
                            push_refs(items, kids);
                        }
                    }
                }
                EntityKind::RelContained => {
                    if let (Some(items), Some(structure)) =
                        (view.attr("RelatedElements"), view.attr("RelatingStructure"))
                    {
                        if let Some(parent) = structure.as_reference() {
                            let kids = contains.entry(parent).or_default();
                            push_refs(items, kids);
                        }
                    }
                }
                EntityKind::RelDefinesByProperties => {
                    // RelatedObjects : SET OF IfcObjectDefinition;
                    // RelatingPropertyDefinition : one definition or a
                    // set of definitions (IfcPropertySetDefinitionSelect).
                    if let (Some(objects), Some(definition)) = (
                        view.attr("RelatedObjects"),
                        view.attr("RelatingPropertyDefinition"),
                    ) {
                        let mut object_ids = Vec::new();
                        push_refs(objects, &mut object_ids);
                        for object in object_ids {
                            let sets = defines.entry(object).or_default();
                            push_refs(definition, sets);
                        }
                    }
                }
                EntityKind::RelDefinesByType => {
                    if let (Some(objects), Some(ty)) =
                        (view.attr("RelatedObjects"), view.attr("RelatingType"))
                    {
                        if let Some(type_id) = ty.as_reference() {
                            let mut object_ids = Vec::new();
                            push_refs(objects, &mut object_ids);
                            for object in object_ids {
                                typed_by.entry(object).or_insert(type_id);
                            }
                        }
                    }
                }
                EntityKind::RelAssociatesMaterial => {
                    if let (Some(objects), Some(material)) =
                        (view.attr("RelatedObjects"), view.attr("RelatingMaterial"))
                    {
                        if let Some(material_id) = material.as_reference() {
                            let mut object_ids = Vec::new();
                            push_refs(objects, &mut object_ids);
                            for object in object_ids {
                                materials.entry(object).or_insert(material_id);
                            }
                        }
                    }
                }
                EntityKind::RelVoids => {
                    if let (Some(element), Some(opening)) = (
                        view.attr("RelatingBuildingElement")
                            .and_then(Value::as_reference),
                        view.attr("RelatedOpeningElement")
                            .and_then(Value::as_reference),
                    ) {
                        voids.entry(element).or_default().push(opening);
                        voided.entry(opening).or_insert(element);
                    }
                }
                EntityKind::RelFills => {
                    if let (Some(opening), Some(filler)) = (
                        view.attr("RelatingOpeningElement")
                            .and_then(Value::as_reference),
                        view.attr("RelatedBuildingElement")
                            .and_then(Value::as_reference),
                    ) {
                        fills.entry(opening).or_default().push(filler);
                        filler_of.entry(filler).or_insert(opening);
                    }
                }
                _ => {}
            }
        }

        Self {
            step,
            project,
            aggregates,
            contains,
            defines,
            typed_by,
            materials,
            voids,
            voided,
            fills,
            filler_of,
        }
    }

    /// The backing parsed file.
    pub fn step(&self) -> &'a StepFile {
        self.step
    }

    /// The `#id` of the `IfcProject` root, when the file has exactly
    /// one.
    pub fn project_id(&self) -> Option<u64> {
        self.project
    }

    /// A typed view of the project root.
    pub fn project(&self) -> Option<TypedEntity<'a>> {
        let id = self.project?;
        TypedEntity::new(self.step.get(id)?)
    }

    /// Typed view of any instance by id, when it is in the typed slice.
    pub fn typed(&self, id: u64) -> Option<TypedEntity<'a>> {
        TypedEntity::new(self.step.get(id)?)
    }

    /// The composition children of `id` (the `IfcRelAggregates`
    /// decomposition: site under project, building under site, …).
    /// Empty when `id` aggregates nothing.
    pub fn aggregated_children(&self, id: u64) -> &[u64] {
        self.aggregates.get(&id).map(Vec::as_slice).unwrap_or(&[])
    }

    /// The products contained directly in the spatial structure `id`
    /// via `IfcRelContainedInSpatialStructure`. Empty when nothing is
    /// contained there.
    pub fn contained_elements(&self, id: u64) -> &[u64] {
        self.contains.get(&id).map(Vec::as_slice).unwrap_or(&[])
    }

    /// Every spatial-structure element (site / building / storey /
    /// space) in the model, as typed views, in ascending id order.
    pub fn spatial_elements(&self) -> impl Iterator<Item = TypedEntity<'a>> + '_ {
        self.step.instances.values().filter_map(|inst| {
            let view = TypedEntity::new(inst)?;
            matches!(view.kind(), EntityKind::Spatial(_)).then_some(view)
        })
    }

    /// The property-set / quantity-set definition `#id`s assigned
    /// **directly** to the object `id` via `IfcRelDefinesByProperties`
    /// (type-inherited sets are not included — see
    /// [`Model::property_set_ids`]). Empty when nothing is assigned.
    pub fn defined_property_sets(&self, id: u64) -> &[u64] {
        self.defines.get(&id).map(Vec::as_slice).unwrap_or(&[])
    }

    /// The `#id` of the `IfcTypeObject` the occurrence `id` is typed by
    /// (`IfcRelDefinesByType.RelatingType`), when one is assigned. The
    /// EXPRESS `Types` inverse is `SET [0:1]`, so the first relationship
    /// encountered wins if a malformed file assigns several.
    pub fn type_of(&self, id: u64) -> Option<u64> {
        self.typed_by.get(&id).copied()
    }

    /// The `#id` of the `IfcMaterialSelect` associated with the object
    /// `id` (`IfcRelAssociatesMaterial.RelatingMaterial`) — a directly
    /// associated material wins; otherwise the material associated with
    /// the object's type (`IfcRelDefinesByType`) applies, per the
    /// occurrence-overrides-type convention.
    pub fn material_of(&self, id: u64) -> Option<u64> {
        if let Some(mid) = self.materials.get(&id) {
            return Some(*mid);
        }
        let type_id = self.type_of(id)?;
        self.materials.get(&type_id).copied()
    }

    /// The resolved
    /// [`MaterialAssignment`](crate::material::MaterialAssignment) for
    /// the object `id`, following [`Model::material_of`].
    pub fn material_assignment(&self, id: u64) -> Option<crate::material::MaterialAssignment<'a>> {
        crate::material::material_assignment(self.step, self.material_of(id)?)
    }

    /// The opening elements voiding the element `id`
    /// (`IfcRelVoidsElement`, the `HasOpenings` inverse). Empty when
    /// nothing is carved out.
    pub fn openings_of(&self, id: u64) -> &[u64] {
        self.voids.get(&id).map(Vec::as_slice).unwrap_or(&[])
    }

    /// The element the opening `id` is carved out of
    /// (`IfcRelVoidsElement.RelatingBuildingElement`, the
    /// `VoidsElements` inverse).
    pub fn voided_element_of(&self, id: u64) -> Option<u64> {
        self.voided.get(&id).copied()
    }

    /// The elements filling the opening `id` (`IfcRelFillsElement`,
    /// the `HasFillings` inverse — doors / windows). Empty when the
    /// opening stays open.
    pub fn fillers_of(&self, id: u64) -> &[u64] {
        self.fills.get(&id).map(Vec::as_slice).unwrap_or(&[])
    }

    /// The opening the element `id` fills
    /// (`IfcRelFillsElement.RelatingOpeningElement`, the
    /// `FillsVoids` inverse).
    pub fn filled_opening_of(&self, id: u64) -> Option<u64> {
        self.filler_of.get(&id).copied()
    }

    /// Every element hosted in the element `id` through an opening —
    /// the openings' fillers flattened (the windows and doors of a
    /// wall), in opening order.
    pub fn hosted_fillers(&self, id: u64) -> Vec<u64> {
        self.openings_of(id)
            .iter()
            .flat_map(|opening| self.fillers_of(*opening).iter().copied())
            .collect()
    }

    /// Every property-set / quantity-set definition `#id` that applies
    /// to the object `id`: its directly assigned sets
    /// (`IfcRelDefinesByProperties`) plus the sets inherited from its
    /// type object's `HasPropertySets` (`IfcRelDefinesByType`).
    ///
    /// An occurrence set **shadows** a type set with the same
    /// `IfcRoot.Name` — the occurrence overrides the type-level default
    /// — so at most one set per name survives, occurrence-first.
    /// `HasPropertySets` is read positionally at index 5, which holds
    /// for every `IfcTypeObject` subtype (ApplicableOccurrence and
    /// HasPropertySets directly follow the IfcRoot attributes).
    pub fn property_set_ids(&self, id: u64) -> Vec<u64> {
        let mut out: Vec<u64> = self.defined_property_sets(id).to_vec();
        // A type object queried directly answers with its own
        // HasPropertySets (occurrences cannot be assigned to it — the
        // NoRelatedTypeObject WHERE rule bars direct assignment).
        if self.is_type_object(id) {
            if let Some(inst) = self.step.get(id) {
                if let Some(Value::List(sets)) = inst.args.get(5) {
                    for set in sets {
                        if let Some(sid) = set.as_reference() {
                            if !out.contains(&sid) {
                                out.push(sid);
                            }
                        }
                    }
                }
            }
            return out;
        }
        let Some(type_id) = self.type_of(id) else {
            return out;
        };
        let Some(type_inst) = self.step.get(type_id) else {
            return out;
        };
        let occurrence_names: Vec<&str> = out
            .iter()
            .filter_map(|sid| self.step.get(*sid))
            .filter_map(|inst| inst.args.get(2).and_then(Value::as_str))
            .collect();
        let Some(Value::List(sets)) = type_inst.args.get(5) else {
            return out;
        };
        for set in sets {
            let Some(sid) = set.as_reference() else {
                continue;
            };
            if out.contains(&sid) {
                continue;
            }
            let shadowed = self
                .step
                .get(sid)
                .and_then(|inst| inst.args.get(2).and_then(Value::as_str))
                .is_some_and(|name| occurrence_names.contains(&name));
            if !shadowed {
                out.push(sid);
            }
        }
        out
    }

    /// Whether `id` is a type object: typed as
    /// [`EntityKind::TypeObject`], or the target of any
    /// `IfcRelDefinesByType` (covering type subtypes outside the typed
    /// slice).
    pub fn is_type_object(&self, id: u64) -> bool {
        if self
            .step
            .get(id)
            .and_then(TypedEntity::new)
            .is_some_and(|view| view.kind() == EntityKind::TypeObject)
        {
            return true;
        }
        self.typed_by.values().any(|t| *t == id)
    }

    /// Every type object in the typed slice
    /// ([`EntityKind::TypeObject`]), as typed views, in ascending id
    /// order.
    pub fn type_objects(&self) -> impl Iterator<Item = TypedEntity<'a>> + '_ {
        self.step.instances.values().filter_map(|inst| {
            let view = TypedEntity::new(inst)?;
            matches!(view.kind(), EntityKind::TypeObject).then_some(view)
        })
    }

    /// Every physical product (`EntityKind::Product`) in the model, as
    /// typed views, in ascending id order.
    pub fn products(&self) -> impl Iterator<Item = TypedEntity<'a>> + '_ {
        self.step.instances.values().filter_map(|inst| {
            let view = TypedEntity::new(inst)?;
            matches!(view.kind(), EntityKind::Product).then_some(view)
        })
    }

    /// Every resolved [`PropertySet`](crate::props::PropertySet) that
    /// applies to the object `id` — directly assigned and
    /// type-inherited sets per [`Model::property_set_ids`], with
    /// non-`IfcPropertySet` definitions (quantity sets) skipped.
    pub fn property_sets(&self, id: u64) -> Vec<crate::props::PropertySet<'a>> {
        self.property_set_ids(id)
            .into_iter()
            .filter_map(|sid| crate::props::property_set(self.step, sid))
            .collect()
    }

    /// Every resolved
    /// [`ElementQuantity`](crate::props::ElementQuantity) that applies
    /// to the object `id` — the quantity-set complement of
    /// [`Model::property_sets`].
    pub fn element_quantities(&self, id: u64) -> Vec<crate::props::ElementQuantity<'a>> {
        self.property_set_ids(id)
            .into_iter()
            .filter_map(|sid| crate::props::element_quantity(self.step, sid))
            .collect()
    }
}

/// Append every `#id` referenced by `value` (a single reference, or the
/// elements of an aggregate of references) to `out`.
fn push_refs(value: &Value, out: &mut Vec<u64>) {
    match value {
        Value::Reference(id) => out.push(*id),
        Value::List(items) => {
            for item in items {
                if let Value::Reference(id) = item {
                    out.push(*id);
                }
            }
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::parse_step;

    /// Wrap a DATA-section body in a minimal valid IFC4 exchange
    /// structure.
    fn wrap(data: &str) -> String {
        format!(
            "ISO-10303-21;\nHEADER;\n\
             FILE_DESCRIPTION((''),'2;1');\n\
             FILE_NAME('t.ifc','2026-06-12T00:00:00',('a'),('o'),'p','s','auth');\n\
             FILE_SCHEMA(('IFC4'));\nENDSEC;\nDATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n"
        )
    }

    fn parse(data: &str) -> StepFile {
        parse_step(wrap(data).as_bytes()).expect("parse failed")
    }

    #[test]
    fn schema_lookup_is_case_insensitive() {
        assert_eq!(schema_of("IfcWall").unwrap().keyword, "IFCWALL");
        assert_eq!(schema_of("IFCWALL").unwrap().kind, EntityKind::Product);
        assert!(schema_of("IFCNOSUCHENTITY").is_none());
    }

    #[test]
    fn attr_chain_lengths_match_schema_text() {
        // The serialised argument counts read off IFC4_ADD2.exp
        // inheritance chains.
        let lens = [
            ("IFCPROJECT", 9), // IfcRoot(4) + IfcContext(5)
            ("IFCSITE", 14),   // Root4 + Object1 + Product2 + Spatial1 + Struct1 + Site5
            ("IFCBUILDING", 12),
            ("IFCBUILDINGSTOREY", 10),
            ("IFCSPACE", 11),
            ("IFCWALL", 9), // Root4 + Object1 + Product2 + Element1 + Wall1
            ("IFCCOLUMN", 9),
            ("IFCDOOR", 13),   // ...Element1 + Door5
            ("IFCWINDOW", 13), // ...Element1 + Window5
            ("IFCOPENINGELEMENT", 9),
            ("IFCRELAGGREGATES", 6),                  // Root4 + 2
            ("IFCRELCONTAINEDINSPATIALSTRUCTURE", 6), // Root4 + 2
            ("IFCLOCALPLACEMENT", 2),
            ("IFCGEOMETRICREPRESENTATIONCONTEXT", 6),
            ("IFCSHAPEREPRESENTATION", 4), // IfcRepresentation(4)
            ("IFCCARTESIANPOINT", 1),
            ("IFCDIRECTION", 1),
            ("IFCAXIS2PLACEMENT2D", 2),  // Location + RefDirection
            ("IFCAXIS2PLACEMENT3D", 3),  // Location + Axis + RefDirection
            ("IFCPOLYLINE", 1),          // Points
            ("IFCMAPPEDITEM", 2),        // MappingSource + MappingTarget
            ("IFCREPRESENTATIONMAP", 2), // MappingOrigin + MappedRepresentation
            ("IFCCARTESIANTRANSFORMATIONOPERATOR3D", 5), // A1,A2,LocalOrigin,Scale,A3
            ("IFCCARTESIANTRANSFORMATIONOPERATOR3DNONUNIFORM", 7), // +Scale2,Scale3
            ("IFCAXIS1PLACEMENT", 2),    // Location + Axis
            ("IFCREVOLVEDAREASOLID", 4), // SweptArea,Position,Axis,Angle
            ("IFCEXTRUDEDAREASOLID", 4), // SweptArea,Position,Direction,Depth
            ("IFCARBITRARYCLOSEDPROFILEDEF", 3), // Type,Name,OuterCurve
            ("IFCRECTANGLEPROFILEDEF", 5), // Type,Name,Position,XDim,YDim
            ("IFCCIRCLEPROFILEDEF", 4),  // Type,Name,Position,Radius
            ("IFCELLIPSEPROFILEDEF", 5), // Type,Name,Position,SemiAxis1/2
            ("IFCCIRCLE", 2),            // Position + Radius
            ("IFCARBITRARYPROFILEDEFWITHVOIDS", 4), // + InnerCurves
            ("IFCCIRCLEHOLLOWPROFILEDEF", 5), // + WallThickness
            ("IFCRECTANGLEHOLLOWPROFILEDEF", 8), // + Wall,fillet radii
            ("IFCCOMPOSITEPROFILEDEF", 4), // Type,Name,Profiles,Label
            ("IFCINDEXEDPOLYCURVE", 3),  // Points,Segments,SelfIntersect
            ("IFCCARTESIANPOINTLIST2D", 1), // CoordList
            ("IFCBOOLEANRESULT", 3),     // Operator + operands
            ("IFCBOOLEANCLIPPINGRESULT", 3), // same serialised attrs
            ("IFCHALFSPACESOLID", 2),    // BaseSurface + AgreementFlag
            ("IFCPOLYGONALBOUNDEDHALFSPACE", 4), // + Position,Boundary
            ("IFCPLANE", 1),             // Position
            ("IFCRELDEFINESBYPROPERTIES", 6), // Root4 + 2
            ("IFCRELDEFINESBYTYPE", 6),  // Root4 + 2
            ("IFCPROPERTYSET", 5),       // Root4 + HasProperties
            ("IFCPROPERTYSINGLEVALUE", 4), // Name,Desc,NominalValue,Unit
            ("IFCPROPERTYENUMERATEDVALUE", 4), // Name,Desc,Values,Reference
            ("IFCPROPERTYENUMERATION", 3), // Name,Values,Unit
            ("IFCPROPERTYBOUNDEDVALUE", 6), // Name,Desc,Upper,Lower,Unit,SetPoint
            ("IFCPROPERTYLISTVALUE", 4), // Name,Desc,Values,Unit
            ("IFCPROPERTYTABLEVALUE", 8), // Name,Desc + 6
            ("IFCPROPERTYREFERENCEVALUE", 4), // Name,Desc,UsageName,Reference
            ("IFCCOMPLEXPROPERTY", 4),   // Name,Desc,UsageName,HasProperties
            ("IFCELEMENTQUANTITY", 6),   // Root4 + Method,Quantities
            ("IFCQUANTITYLENGTH", 5),    // Name,Desc,Unit,Value,Formula
            ("IFCQUANTITYAREA", 5),
            ("IFCQUANTITYVOLUME", 5),
            ("IFCQUANTITYCOUNT", 5),
            ("IFCQUANTITYWEIGHT", 5),
            ("IFCQUANTITYTIME", 5),
            ("IFCPHYSICALCOMPLEXQUANTITY", 6), // Name,Desc + 4
            ("IFCWALLTYPE", 10),               // Root4 + Type2 + Product2 + Elem1 + 1
            ("IFCWINDOWTYPE", 13),             // Root4 + Type2 + Product2 + Elem1 + 4
            ("IFCSANITARYTERMINALTYPE", 10),
            ("IFCRELASSOCIATESMATERIAL", 6), // Root4 + 2
            ("IFCMATERIAL", 3),              // Name,Desc,Category
            ("IFCMATERIALLAYER", 7),
            ("IFCMATERIALLAYERWITHOFFSETS", 9),
            ("IFCMATERIALLAYERSET", 3),
            ("IFCMATERIALLAYERSETUSAGE", 5),
            ("IFCMATERIALPROFILE", 6),
            ("IFCMATERIALPROFILESET", 4),
            ("IFCMATERIALPROFILESETUSAGE", 3),
            ("IFCMATERIALPROFILESETUSAGETAPERING", 5),
            ("IFCMATERIALCONSTITUENT", 5),
            ("IFCMATERIALCONSTITUENTSET", 3),
            ("IFCCOLUMNTYPE", 10),
            ("IFCBEAMTYPE", 10),
            ("IFCSLABTYPE", 10),
            ("IFCBUILDINGELEMENTPROXYTYPE", 10),
            ("IFCCOVERINGTYPE", 10),
            ("IFCMEMBERTYPE", 10),
            ("IFCPLATETYPE", 10),
            ("IFCRAILINGTYPE", 10),
            ("IFCROOFTYPE", 10),
            ("IFCSTAIRTYPE", 10),
            ("IFCDOORTYPE", 13),       // + Door4
            ("IFCFURNITURETYPE", 11),  // + AssemblyPlace,PredefinedType
            ("IFCDOORSTYLE", 12),      // TypeProduct(8) + 4
            ("IFCWINDOWSTYLE", 12),    // TypeProduct(8) + 4
            ("IFCRELVOIDSELEMENT", 6), // Root4 + 2
            ("IFCRELFILLSELEMENT", 6), // Root4 + 2
            ("IFCPROJECTEDCRS", 7),    // CRS(4) + 3
            ("IFCMAPCONVERSION", 8),   // Operation(2) + 6
        ];
        for (kw, want) in lens {
            assert_eq!(
                schema_of(kw).unwrap().attrs.len(),
                want,
                "attr count for {kw}"
            );
        }
    }

    #[test]
    fn root_attributes_always_lead() {
        for s in SCHEMA {
            if !matches!(
                s.kind,
                EntityKind::Project
                    | EntityKind::Spatial(_)
                    | EntityKind::Product
                    | EntityKind::RelAggregates
                    | EntityKind::RelContained
            ) {
                continue; // not an IfcRoot subtype
            }
            assert_eq!(
                &s.attrs[..4],
                ROOT,
                "{} should lead with IfcRoot",
                s.keyword
            );
        }
    }

    #[test]
    fn typed_entity_resolves_named_attributes() {
        // IfcWall: GlobalId, OwnerHistory, Name, Description, ObjectType,
        // ObjectPlacement, Representation, Tag, PredefinedType.
        let f = parse(
            "#1=IFCWALL('3ZYW',#2,'My Wall','desc',$,#46,#48,$,.STANDARD.);\n\
             #2=IFCOWNERHISTORY();\n#46=IFCLOCALPLACEMENT($,#47);",
        );
        let w = TypedEntity::new(f.get(1).unwrap()).unwrap();
        assert_eq!(w.kind(), EntityKind::Product);
        assert_eq!(w.global_id(), Some("3ZYW"));
        assert_eq!(w.name(), Some("My Wall"));
        assert_eq!(w.description(), Some("desc"));
        assert_eq!(w.object_placement(), Some(46));
        assert_eq!(w.representation(), Some(48));
        assert_eq!(w.predefined_type(), Some("STANDARD"));
        // OwnerHistory is a reference, ObjectType is $.
        assert_eq!(w.attr("OwnerHistory").unwrap().as_reference(), Some(2));
        assert!(w.attr("ObjectType").unwrap().is_unset());
        assert!(w.attr("NoSuchAttr").is_none());
    }

    #[test]
    fn typed_entity_iterates_in_serialisation_order() {
        let f = parse("#1=IFCBUILDINGSTOREY('g',$,'Ground',$,$,$,$,$,.ELEMENT.,0.);");
        let s = TypedEntity::new(f.get(1).unwrap()).unwrap();
        let names: Vec<&str> = s.attrs().map(|(n, _)| n).collect();
        assert_eq!(
            names,
            [
                "GlobalId",
                "OwnerHistory",
                "Name",
                "Description",
                "ObjectType",
                "ObjectPlacement",
                "Representation",
                "LongName",
                "CompositionType",
                "Elevation",
            ]
        );
        assert_eq!(
            s.attr("CompositionType").unwrap().as_enum(),
            Some("ELEMENT")
        );
        assert_eq!(s.attr("Elevation").unwrap().as_number(), Some(0.0));
    }

    #[test]
    fn model_resolves_spatial_hierarchy() {
        // project → site → building → storey, with a wall contained in
        // the storey, linked through IfcRelAggregates +
        // IfcRelContainedInSpatialStructure.
        let f = parse(
            "#1=IFCPROJECT('p',$,'Proj',$,$,$,$,(#20),$);\n\
             #20=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,1.E-5,$,$);\n\
             #31=IFCSITE('s',$,'Site',$,$,$,$,$,.ELEMENT.,$,$,$,$,$);\n\
             #34=IFCBUILDING('b',$,'Bldg',$,$,$,$,$,.ELEMENT.,$,$,$);\n\
             #38=IFCBUILDINGSTOREY('f',$,'Floor',$,$,$,$,$,.ELEMENT.,0.);\n\
             #45=IFCWALL('w',$,'Wall',$,$,$,$,$,$);\n\
             #41=IFCRELAGGREGATES('a1',$,$,$,#34,(#38));\n\
             #42=IFCRELAGGREGATES('a2',$,$,$,#31,(#34));\n\
             #43=IFCRELAGGREGATES('a3',$,$,$,#1,(#31));\n\
             #44=IFCRELCONTAINEDINSPATIALSTRUCTURE('c',$,$,$,(#45),#38);",
        );
        let m = Model::from_step(&f);
        assert_eq!(m.project_id(), Some(1));
        assert_eq!(m.project().unwrap().name(), Some("Proj"));
        assert_eq!(m.aggregated_children(1), &[31]);
        assert_eq!(m.aggregated_children(31), &[34]);
        assert_eq!(m.aggregated_children(34), &[38]);
        assert_eq!(m.contained_elements(38), &[45]);
        // Walk from project to the wall through both relationship kinds.
        let site = m.aggregated_children(1)[0];
        let bldg = m.aggregated_children(site)[0];
        let storey = m.aggregated_children(bldg)[0];
        let wall = m.contained_elements(storey)[0];
        assert_eq!(m.typed(wall).unwrap().global_id(), Some("w"));
        // Enumerations.
        assert_eq!(m.spatial_elements().count(), 3); // site/building/storey
        assert_eq!(m.products().count(), 1); // the wall
        let kinds: Vec<_> = m.spatial_elements().map(|e| e.kind()).collect();
        assert!(kinds.contains(&EntityKind::Spatial(SpatialKind::Site)));
        assert!(kinds.contains(&EntityKind::Spatial(SpatialKind::Building)));
        assert!(kinds.contains(&EntityKind::Spatial(SpatialKind::Storey)));
    }

    #[test]
    fn product_contained_directly_in_site() {
        // A product can be contained directly in any spatial element,
        // not only a storey (the column-in-site fixture shape).
        let f = parse(
            "#37=IFCPROJECT('p',$,'Project',$,$,$,$,(#40),$);\n\
             #40=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Plan',3,$,$,$);\n\
             #44=IFCSITE('s',$,'Site #1',$,$,$,$,$,.ELEMENT.,$,$,$,$,$);\n\
             #71=IFCCOLUMN('col',$,'Column #1',$,$,#121,#111,$,.COLUMN.);\n\
             #45=IFCRELAGGREGATES('a',$,$,$,#37,(#44));\n\
             #116=IFCRELCONTAINEDINSPATIALSTRUCTURE('c',$,$,$,(#71),#44);",
        );
        let m = Model::from_step(&f);
        assert_eq!(m.contained_elements(44), &[71]);
        let col = m.typed(71).unwrap();
        assert_eq!(col.kind(), EntityKind::Product);
        assert_eq!(col.predefined_type(), Some("COLUMN"));
        assert_eq!(col.object_placement(), Some(121));
        assert_eq!(col.representation(), Some(111));
    }

    #[test]
    fn ambiguous_double_project_clears_root() {
        let f = parse(
            "#1=IFCPROJECT('p1',$,'A',$,$,$,$,$,$);\n\
             #2=IFCPROJECT('p2',$,'B',$,$,$,$,$,$);",
        );
        let m = Model::from_step(&f);
        assert_eq!(m.project_id(), None);
    }

    #[test]
    fn untyped_keyword_yields_no_view() {
        // IfcOwnerHistory is outside the typed slice — no view, and it is
        // neither a product nor a spatial element.
        let f = parse("#1=IFCOWNERHISTORY();");
        assert!(TypedEntity::new(f.get(1).unwrap()).is_none());
        let m = Model::from_step(&f);
        assert!(m.typed(1).is_none());
        assert_eq!(m.products().count(), 0);
    }

    #[test]
    fn cartesian_point_and_direction_are_typed() {
        let f = parse(
            "#1=IFCCARTESIANPOINT((1.,2.,3.));\n\
             #2=IFCDIRECTION((0.,0.,1.));\n\
             #3=IFCDIRECTION((1,0));", // integer ratios + 2-D vector
        );
        let p = TypedEntity::new(f.get(1).unwrap()).unwrap();
        assert_eq!(p.kind(), EntityKind::Geometry);
        assert_eq!(p.coordinates(), Some(vec![1.0, 2.0, 3.0]));
        // A point carries no direction ratios.
        assert_eq!(p.direction_ratios(), None);

        let d = TypedEntity::new(f.get(2).unwrap()).unwrap();
        assert_eq!(d.kind(), EntityKind::Geometry);
        assert_eq!(d.direction_ratios(), Some(vec![0.0, 0.0, 1.0]));

        // Integer-valued ratios widen to f64; 2-D direction stays 2-D.
        let d2 = TypedEntity::new(f.get(3).unwrap()).unwrap();
        assert_eq!(d2.direction_ratios(), Some(vec![1.0, 0.0]));
    }

    #[test]
    fn axis2placement3d_resolves_location_axis_refdir() {
        // The common writer form: only Location set, Axis/RefDirection $.
        let f = parse(
            "#8=IFCAXIS2PLACEMENT3D(#9,$,$);\n\
             #9=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #10=IFCAXIS2PLACEMENT3D(#9,#11,#12);\n\
             #11=IFCDIRECTION((0.,0.,1.));\n\
             #12=IFCDIRECTION((1.,0.,0.));",
        );
        let a = TypedEntity::new(f.get(8).unwrap()).unwrap();
        assert_eq!(a.kind(), EntityKind::Geometry);
        assert_eq!(a.location(), Some(9));
        assert_eq!(a.axis(), None); // $ → None
        assert_eq!(a.ref_direction(), None);

        let b = TypedEntity::new(f.get(10).unwrap()).unwrap();
        assert_eq!(b.location(), Some(9));
        assert_eq!(b.axis(), Some(11));
        assert_eq!(b.ref_direction(), Some(12));
    }

    #[test]
    fn axis2placement2d_has_location_then_refdirection() {
        let f = parse(
            "#1=IFCAXIS2PLACEMENT2D(#2,#3);\n\
             #2=IFCCARTESIANPOINT((0.,0.));\n\
             #3=IFCDIRECTION((1.,0.));",
        );
        let a = TypedEntity::new(f.get(1).unwrap()).unwrap();
        let names: Vec<&str> = a.attrs().map(|(n, _)| n).collect();
        assert_eq!(names, ["Location", "RefDirection"]);
        assert_eq!(a.location(), Some(2));
        assert_eq!(a.ref_direction(), Some(3));
        // 2-D placement has no Axis attribute at all.
        assert_eq!(a.axis(), None);
        assert!(a.attr("Axis").is_none());
    }

    #[test]
    fn polyline_lists_point_references() {
        let f = parse(
            "#67=IFCPOLYLINE((#68,#69,#70));\n\
             #68=IFCCARTESIANPOINT((0.,0.));\n\
             #69=IFCCARTESIANPOINT((1.,0.));\n\
             #70=IFCCARTESIANPOINT((1.,1.));",
        );
        let pl = TypedEntity::new(f.get(67).unwrap()).unwrap();
        assert_eq!(pl.kind(), EntityKind::Geometry);
        assert_eq!(pl.points(), Some(vec![68, 69, 70]));
        // Each point resolves and carries 2-D coordinates.
        let first = TypedEntity::new(f.get(68).unwrap()).unwrap();
        assert_eq!(first.coordinates(), Some(vec![0.0, 0.0]));
    }

    #[test]
    fn shape_representation_resolves_context_and_items() {
        let f = parse(
            "#154=IFCSHAPEREPRESENTATION(#41,'Body','Tessellation',(#288,#289));\n\
             #41=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,$,$,$);",
        );
        let r = TypedEntity::new(f.get(154).unwrap()).unwrap();
        assert_eq!(r.kind(), EntityKind::Representation);
        assert_eq!(r.context_of_items(), Some(41));
        assert_eq!(r.representation_identifier(), Some("Body"));
        assert_eq!(r.representation_type(), Some("Tessellation"));
        assert_eq!(r.items(), Some(vec![288, 289]));
        let names: Vec<&str> = r.attrs().map(|(n, _)| n).collect();
        assert_eq!(
            names,
            [
                "ContextOfItems",
                "RepresentationIdentifier",
                "RepresentationType",
                "Items",
            ]
        );
    }

    #[test]
    fn mapped_item_chain_resolves_by_name() {
        // IfcMappedItem → IfcRepresentationMap → MappedRepresentation /
        // MappingOrigin, plus the transformation-operator attributes,
        // walked entirely by attribute name through the typed layer.
        let f = parse(
            "#22=IFCMAPPEDITEM(#12,#21);\n\
             #12=IFCREPRESENTATIONMAP(#11,#3);\n\
             #11=IFCAXIS2PLACEMENT3D(#10,$,$);\n\
             #10=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #20=IFCCARTESIANPOINT((1.,2.,3.));\n\
             #21=IFCCARTESIANTRANSFORMATIONOPERATOR3D($,$,#20,2.,$);\n\
             #3=IFCSHAPEREPRESENTATION(#41,'Body','Tessellation',(#288));\n\
             #41=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,$,$,$);",
        );
        let mi = TypedEntity::new(f.get(22).unwrap()).unwrap();
        assert_eq!(mi.kind(), EntityKind::Geometry);
        assert_eq!(mi.mapping_source(), Some(12));
        assert_eq!(mi.mapping_target(), Some(21));

        let map = TypedEntity::new(f.get(12).unwrap()).unwrap();
        assert_eq!(map.mapping_origin(), Some(11));
        assert_eq!(map.mapped_representation(), Some(3));

        let op = TypedEntity::new(f.get(21).unwrap()).unwrap();
        // Axis1 / Axis2 are `$`; LocalOrigin resolves; Scale = 2.
        assert_eq!(op.attr("Axis1").and_then(Value::as_reference), None);
        assert_eq!(
            op.attr("LocalOrigin").and_then(Value::as_reference),
            Some(20)
        );
        assert_eq!(op.attr("Scale").and_then(Value::as_number), Some(2.0));
        let names: Vec<&str> = op.attrs().map(|(n, _)| n).collect();
        assert_eq!(names, ["Axis1", "Axis2", "LocalOrigin", "Scale", "Axis3"]);
    }

    #[test]
    fn geometry_primitives_stay_out_of_spatial_model() {
        // Geometry-kind entities are typed but never enter the
        // product / spatial enumerations the spatial model exposes.
        let f = parse(
            "#1=IFCPROJECT('p',$,'P',$,$,$,$,$,$);\n\
             #2=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #3=IFCDIRECTION((0.,0.,1.));\n\
             #4=IFCAXIS2PLACEMENT3D(#2,$,$);",
        );
        let m = Model::from_step(&f);
        assert_eq!(m.products().count(), 0);
        assert_eq!(m.spatial_elements().count(), 0);
        // …but each is individually typed.
        assert_eq!(m.typed(2).unwrap().kind(), EntityKind::Geometry);
        assert_eq!(m.typed(4).unwrap().location(), Some(2));
    }

    #[test]
    fn truncated_record_treats_trailing_optionals_as_absent() {
        // A writer that omits trailing optional attributes: only 5 args.
        let f = parse("#1=IFCWALL('g',$,'Short',$,$);");
        let w = TypedEntity::new(f.get(1).unwrap()).unwrap();
        assert_eq!(w.global_id(), Some("g"));
        assert_eq!(w.name(), Some("Short"));
        // ObjectPlacement / PredefinedType not serialised → None.
        assert_eq!(w.object_placement(), None);
        assert_eq!(w.predefined_type(), None);
        assert_eq!(w.attrs().count(), 5);
    }

    #[test]
    fn voids_and_fills_fold_with_multiple_fillers() {
        // One wall, two openings; the second opening is double-filled
        // (door + side light). Back edges keep the first relationship.
        let f = parse(
            "#10=IFCWALL('w',$,'Wall',$,$,$,$,$,$);\n\
             #20=IFCOPENINGELEMENT('o1',$,'Op1',$,$,$,$,$,.OPENING.);\n\
             #21=IFCOPENINGELEMENT('o2',$,'Op2',$,$,$,$,$,.OPENING.);\n\
             #30=IFCDOOR('d',$,'Door',$,$,$,$,$,$,$,$,$,$);\n\
             #31=IFCWINDOW('g',$,'Light',$,$,$,$,$,$,$,$,$,$);\n\
             #40=IFCRELVOIDSELEMENT('v1',$,$,$,#10,#20);\n\
             #41=IFCRELVOIDSELEMENT('v2',$,$,$,#10,#21);\n\
             #50=IFCRELFILLSELEMENT('f1',$,$,$,#21,#30);\n\
             #51=IFCRELFILLSELEMENT('f2',$,$,$,#21,#31);",
        );
        let m = Model::from_step(&f);
        assert_eq!(m.openings_of(10), &[20, 21]);
        assert_eq!(m.voided_element_of(20), Some(10));
        assert_eq!(m.voided_element_of(21), Some(10));
        assert!(m.fillers_of(20).is_empty());
        assert_eq!(m.fillers_of(21), &[30, 31]);
        assert_eq!(m.filled_opening_of(30), Some(21));
        assert_eq!(m.filled_opening_of(31), Some(21));
        assert_eq!(m.hosted_fillers(10), vec![30, 31]);
    }

    #[test]
    fn dangling_relationship_edges_are_skipped() {
        let f = parse(
            "#1=IFCPROJECT('p',$,'P',$,$,$,$,$,$);\n\
             #41=IFCRELAGGREGATES('a',$,$,$,#1,(#999));",
        );
        let m = Model::from_step(&f);
        // The edge is still recorded (id-level), but resolving #999 is
        // None — the model does not invent the target.
        assert_eq!(m.aggregated_children(1), &[999]);
        assert!(m.typed(999).is_none());
    }
}
