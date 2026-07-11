//! Phase 4: typed material-association extraction.
//!
//! `IfcRelAssociatesMaterial` hands a set of objects (occurrences
//! **or** type objects) an `IfcMaterialSelect`: a material definition
//! (`IfcMaterial`, layer / profile / constituent sets), an
//! `IfcMaterialList`, or a usage definition binding a layer / profile
//! set to the element geometry. [`Model`](crate::Model) folds the
//! relationship ([`Model::material_of`](crate::Model::material_of),
//! with the occurrence-overrides-type fallback); this module resolves
//! the select target into a typed [`MaterialAssignment`].
//!
//! Layer thicknesses are in the model **length unit**
//! ([`length_unit_scale`](crate::schema::length_unit_scale) converts);
//! profile references stay `#id`s into the profile-definition family
//! the geometry layer consumes.

use crate::parser::StepFile;
use crate::schema::TypedEntity;
use crate::value::Value;

/// A plain `IfcMaterial`: the named homogeneous substance.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Material<'a> {
    /// The `#id` of the `IfcMaterial` instance.
    pub id: u64,
    /// `IfcMaterial.Name` (required by the schema).
    pub name: Option<&'a str>,
    /// `IfcMaterial.Description`, when set.
    pub description: Option<&'a str>,
    /// `IfcMaterial.Category` (e.g. `"concrete"`, `"steel"`), when set.
    pub category: Option<&'a str>,
}

/// One `IfcMaterialLayer` of a layer set.
#[derive(Debug, Clone, PartialEq)]
pub struct MaterialLayer<'a> {
    /// The `#id` of the layer instance.
    pub id: u64,
    /// The layer's material, when set (an unset material models an
    /// air gap).
    pub material: Option<Material<'a>>,
    /// `LayerThickness` in model length units (a non-negative measure;
    /// 0 models an infinitesimal membrane).
    pub thickness: f64,
    /// `IsVentilated` — `Some(true)` for an air gap with ventilation,
    /// `None` when unknown / unset.
    pub is_ventilated: Option<bool>,
    /// The optional layer name.
    pub name: Option<&'a str>,
    /// The optional layer description.
    pub description: Option<&'a str>,
    /// The optional layer category (e.g. `"insulation"`).
    pub category: Option<&'a str>,
    /// The optional joining priority (0–100 per the WHERE rule).
    pub priority: Option<i64>,
}

/// An `IfcMaterialLayerSet`: ordered layers through the element.
#[derive(Debug, Clone, PartialEq)]
pub struct MaterialLayerSet<'a> {
    /// The `#id` of the layer-set instance.
    pub id: u64,
    /// `LayerSetName`, when set.
    pub name: Option<&'a str>,
    /// `Description`, when set.
    pub description: Option<&'a str>,
    /// The layers, in the set's list order.
    pub layers: Vec<MaterialLayer<'a>>,
}

impl MaterialLayerSet<'_> {
    /// The derived `TotalThickness` — the sum of the layer
    /// thicknesses (the EXPRESS `IfcMlsTotalThickness` function), in
    /// model length units.
    pub fn total_thickness(&self) -> f64 {
        self.layers.iter().map(|l| l.thickness).sum()
    }
}

/// One `IfcMaterialProfile` of a profile set.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MaterialProfile<'a> {
    /// The `#id` of the material-profile instance.
    pub id: u64,
    /// The optional profile name.
    pub name: Option<&'a str>,
    /// The optional description.
    pub description: Option<&'a str>,
    /// The profile's material, when set.
    pub material: Option<Material<'a>>,
    /// The `#id` of the `IfcProfileDef` giving the cross-section
    /// geometry (the profile family the geometry layer sweeps).
    pub profile: Option<u64>,
    /// The optional joining priority (0–100).
    pub priority: Option<i64>,
    /// The optional category.
    pub category: Option<&'a str>,
}

/// An `IfcMaterialProfileSet`: the cross-section make-up of a
/// parametric member (beam / column).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MaterialProfileSet<'a> {
    /// The `#id` of the profile-set instance.
    pub id: u64,
    /// The optional set name.
    pub name: Option<&'a str>,
    /// The optional description.
    pub description: Option<&'a str>,
    /// The member profiles, in list order.
    pub profiles: Vec<MaterialProfile<'a>>,
    /// The `#id` of the optional `IfcCompositeProfileDef` combining
    /// the member profiles.
    pub composite_profile: Option<u64>,
}

/// One `IfcMaterialConstituent` of a constituent set.
#[derive(Debug, Clone, PartialEq)]
pub struct MaterialConstituent<'a> {
    /// The `#id` of the constituent instance.
    pub id: u64,
    /// The optional constituent name (e.g. `"Frame"`, `"Glazing"`).
    pub name: Option<&'a str>,
    /// The optional description.
    pub description: Option<&'a str>,
    /// The constituent's material (required by the schema).
    pub material: Option<Material<'a>>,
    /// The optional normalised fraction of the total (0–1).
    pub fraction: Option<f64>,
    /// The optional category.
    pub category: Option<&'a str>,
}

/// An `IfcMaterialConstituentSet`: named parts with distinct
/// materials (window frame vs glazing).
#[derive(Debug, Clone, PartialEq)]
pub struct MaterialConstituentSet<'a> {
    /// The `#id` of the constituent-set instance.
    pub id: u64,
    /// The optional set name.
    pub name: Option<&'a str>,
    /// The optional description.
    pub description: Option<&'a str>,
    /// The member constituents.
    pub constituents: Vec<MaterialConstituent<'a>>,
}

/// A resolved `IfcMaterialSelect` — what an
/// `IfcRelAssociatesMaterial` assigned to an object.
#[derive(Debug, Clone, PartialEq)]
pub enum MaterialAssignment<'a> {
    /// A single `IfcMaterial`.
    Material(Material<'a>),
    /// An `IfcMaterialList` (unordered alternative to a constituent
    /// set; deprecated upstream but still written).
    MaterialList(Vec<Material<'a>>),
    /// A bare `IfcMaterialLayer` (a material select member through
    /// `IfcMaterialDefinition`).
    Layer(MaterialLayer<'a>),
    /// An `IfcMaterialLayerSet` associated directly.
    LayerSet(MaterialLayerSet<'a>),
    /// An `IfcMaterialLayerSetUsage` — the layer set plus how it hangs
    /// on the element's reference line.
    LayerSetUsage {
        /// The used layer set (`ForLayerSet`), when resolvable.
        layer_set: Option<MaterialLayerSet<'a>>,
        /// `LayerSetDirection` — the `IfcLayerSetDirectionEnum`
        /// literal (`"AXIS1"`/`"AXIS2"`/`"AXIS3"`).
        direction: Option<&'a str>,
        /// `DirectionSense` (`"POSITIVE"`/`"NEGATIVE"`).
        direction_sense: Option<&'a str>,
        /// `OffsetFromReferenceLine` in model length units.
        offset_from_reference_line: Option<f64>,
        /// The optional `ReferenceExtent`.
        reference_extent: Option<f64>,
    },
    /// A bare `IfcMaterialProfile`.
    Profile(MaterialProfile<'a>),
    /// An `IfcMaterialProfileSet` associated directly.
    ProfileSet(MaterialProfileSet<'a>),
    /// An `IfcMaterialProfileSetUsage` (the tapering subtype resolves
    /// here too, through its start `ForProfileSet`).
    ProfileSetUsage {
        /// The used profile set (`ForProfileSet`), when resolvable.
        profile_set: Option<MaterialProfileSet<'a>>,
        /// The optional `IfcCardinalPointReference` (an integer code
        /// locating the profile on the member axis).
        cardinal_point: Option<i64>,
        /// The optional `ReferenceExtent`.
        reference_extent: Option<f64>,
    },
    /// A bare `IfcMaterialConstituent`.
    Constituent(MaterialConstituent<'a>),
    /// An `IfcMaterialConstituentSet`.
    ConstituentSet(MaterialConstituentSet<'a>),
}

impl<'a> MaterialAssignment<'a> {
    /// The headline material name for display: the single material's
    /// name, the set/list name when it has one, or the first named
    /// member.
    pub fn name(&self) -> Option<&'a str> {
        match self {
            Self::Material(m) => m.name,
            Self::MaterialList(list) => list.iter().find_map(|m| m.name),
            Self::Layer(l) => l.name.or_else(|| l.material.as_ref().and_then(|m| m.name)),
            Self::LayerSet(set) => set
                .name
                .or_else(|| set.layers.iter().find_map(|l| l.material.as_ref()?.name)),
            Self::LayerSetUsage { layer_set, .. } => layer_set.as_ref().and_then(|set| {
                set.name
                    .or_else(|| set.layers.iter().find_map(|l| l.material.as_ref()?.name))
            }),
            Self::Profile(p) => p.name.or_else(|| p.material.as_ref().and_then(|m| m.name)),
            Self::ProfileSet(set) => set
                .name
                .or_else(|| set.profiles.iter().find_map(|p| p.material.as_ref()?.name)),
            Self::ProfileSetUsage { profile_set, .. } => profile_set.as_ref().and_then(|set| {
                set.name
                    .or_else(|| set.profiles.iter().find_map(|p| p.material.as_ref()?.name))
            }),
            Self::Constituent(c) => c.name.or_else(|| c.material.as_ref().and_then(|m| m.name)),
            Self::ConstituentSet(set) => set.name.or_else(|| {
                set.constituents
                    .iter()
                    .find_map(|c| c.material.as_ref()?.name)
            }),
        }
    }
}

/// An optional string attribute (`$` → `None`).
fn opt_str<'a>(entity: &TypedEntity<'a>, name: &str) -> Option<&'a str> {
    entity.attr(name)?.as_str()
}

/// An optional numeric attribute, seeing through a typed measure
/// wrapper.
fn opt_number(entity: &TypedEntity<'_>, name: &str) -> Option<f64> {
    match entity.attr(name)? {
        Value::Typed { args, .. } => args.first().and_then(Value::as_number),
        other => other.as_number(),
    }
}

/// Resolve one `IfcMaterial` reference.
fn resolve_material(step: &StepFile, id: u64) -> Option<Material<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCMATERIAL" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    Some(Material {
        id,
        name: opt_str(&view, "Name"),
        description: opt_str(&view, "Description"),
        category: opt_str(&view, "Category"),
    })
}

/// Resolve one `IfcMaterialLayer` (or `…WithOffsets`) reference.
fn resolve_layer(step: &StepFile, id: u64) -> Option<MaterialLayer<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCMATERIALLAYER" && inst.keyword != "IFCMATERIALLAYERWITHOFFSETS" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    Some(MaterialLayer {
        id,
        material: view
            .attr("Material")
            .and_then(Value::as_reference)
            .and_then(|mid| resolve_material(step, mid)),
        thickness: opt_number(&view, "LayerThickness")?,
        is_ventilated: view.attr("IsVentilated").and_then(|v| match v.as_enum()? {
            "T" => Some(true),
            "F" => Some(false),
            _ => None,
        }),
        name: opt_str(&view, "Name"),
        description: opt_str(&view, "Description"),
        category: opt_str(&view, "Category"),
        priority: view.attr("Priority").and_then(Value::as_integer),
    })
}

/// Resolve one `IfcMaterialLayerSet` reference.
fn resolve_layer_set(step: &StepFile, id: u64) -> Option<MaterialLayerSet<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCMATERIALLAYERSET" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    let layers = view
        .attr("MaterialLayers")
        .and_then(Value::as_list)
        .map(|items| {
            items
                .iter()
                .filter_map(Value::as_reference)
                .filter_map(|lid| resolve_layer(step, lid))
                .collect()
        })
        .unwrap_or_default();
    Some(MaterialLayerSet {
        id,
        name: opt_str(&view, "LayerSetName"),
        description: opt_str(&view, "Description"),
        layers,
    })
}

/// Resolve one `IfcMaterialProfile` (or `…WithOffsets`) reference.
fn resolve_profile(step: &StepFile, id: u64) -> Option<MaterialProfile<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCMATERIALPROFILE" && inst.keyword != "IFCMATERIALPROFILEWITHOFFSETS" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    Some(MaterialProfile {
        id,
        name: opt_str(&view, "Name"),
        description: opt_str(&view, "Description"),
        material: view
            .attr("Material")
            .and_then(Value::as_reference)
            .and_then(|mid| resolve_material(step, mid)),
        profile: view.attr("Profile").and_then(Value::as_reference),
        priority: view.attr("Priority").and_then(Value::as_integer),
        category: opt_str(&view, "Category"),
    })
}

/// Resolve one `IfcMaterialProfileSet` reference.
fn resolve_profile_set(step: &StepFile, id: u64) -> Option<MaterialProfileSet<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCMATERIALPROFILESET" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    let profiles = view
        .attr("MaterialProfiles")
        .and_then(Value::as_list)
        .map(|items| {
            items
                .iter()
                .filter_map(Value::as_reference)
                .filter_map(|pid| resolve_profile(step, pid))
                .collect()
        })
        .unwrap_or_default();
    Some(MaterialProfileSet {
        id,
        name: opt_str(&view, "Name"),
        description: opt_str(&view, "Description"),
        profiles,
        composite_profile: view.attr("CompositeProfile").and_then(Value::as_reference),
    })
}

/// Resolve one `IfcMaterialConstituent` reference.
fn resolve_constituent(step: &StepFile, id: u64) -> Option<MaterialConstituent<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCMATERIALCONSTITUENT" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    Some(MaterialConstituent {
        id,
        name: opt_str(&view, "Name"),
        description: opt_str(&view, "Description"),
        material: view
            .attr("Material")
            .and_then(Value::as_reference)
            .and_then(|mid| resolve_material(step, mid)),
        fraction: opt_number(&view, "Fraction"),
        category: opt_str(&view, "Category"),
    })
}

/// Resolve an `IfcMaterialSelect` target (the `RelatingMaterial` of an
/// `IfcRelAssociatesMaterial`) into a typed [`MaterialAssignment`].
/// Returns `None` when `id` is missing or not a material entity.
pub fn material_assignment(step: &StepFile, id: u64) -> Option<MaterialAssignment<'_>> {
    let inst = step.get(id)?;
    let view = TypedEntity::new(inst);
    Some(match inst.keyword.as_str() {
        "IFCMATERIAL" => MaterialAssignment::Material(resolve_material(step, id)?),
        "IFCMATERIALLIST" => {
            let view = view?;
            let materials = view
                .attr("Materials")
                .and_then(Value::as_list)
                .map(|items| {
                    items
                        .iter()
                        .filter_map(Value::as_reference)
                        .filter_map(|mid| resolve_material(step, mid))
                        .collect()
                })
                .unwrap_or_default();
            MaterialAssignment::MaterialList(materials)
        }
        "IFCMATERIALLAYER" | "IFCMATERIALLAYERWITHOFFSETS" => {
            MaterialAssignment::Layer(resolve_layer(step, id)?)
        }
        "IFCMATERIALLAYERSET" => MaterialAssignment::LayerSet(resolve_layer_set(step, id)?),
        "IFCMATERIALLAYERSETUSAGE" => {
            let view = view?;
            MaterialAssignment::LayerSetUsage {
                layer_set: view
                    .attr("ForLayerSet")
                    .and_then(Value::as_reference)
                    .and_then(|sid| resolve_layer_set(step, sid)),
                direction: view.attr("LayerSetDirection").and_then(Value::as_enum),
                direction_sense: view.attr("DirectionSense").and_then(Value::as_enum),
                offset_from_reference_line: opt_number(&view, "OffsetFromReferenceLine"),
                reference_extent: opt_number(&view, "ReferenceExtent"),
            }
        }
        "IFCMATERIALPROFILE" | "IFCMATERIALPROFILEWITHOFFSETS" => {
            MaterialAssignment::Profile(resolve_profile(step, id)?)
        }
        "IFCMATERIALPROFILESET" => MaterialAssignment::ProfileSet(resolve_profile_set(step, id)?),
        "IFCMATERIALPROFILESETUSAGE" | "IFCMATERIALPROFILESETUSAGETAPERING" => {
            let view = view?;
            MaterialAssignment::ProfileSetUsage {
                profile_set: view
                    .attr("ForProfileSet")
                    .and_then(Value::as_reference)
                    .and_then(|sid| resolve_profile_set(step, sid)),
                cardinal_point: view.attr("CardinalPoint").and_then(Value::as_integer),
                reference_extent: opt_number(&view, "ReferenceExtent"),
            }
        }
        "IFCMATERIALCONSTITUENT" => MaterialAssignment::Constituent(resolve_constituent(step, id)?),
        "IFCMATERIALCONSTITUENTSET" => {
            let view = view?;
            let constituents = view
                .attr("MaterialConstituents")
                .and_then(Value::as_list)
                .map(|items| {
                    items
                        .iter()
                        .filter_map(Value::as_reference)
                        .filter_map(|cid| resolve_constituent(step, cid))
                        .collect()
                })
                .unwrap_or_default();
            MaterialAssignment::ConstituentSet(MaterialConstituentSet {
                id,
                name: opt_str(&view, "Name"),
                description: opt_str(&view, "Description"),
                constituents,
            })
        }
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::parse_step;
    use crate::schema::Model;

    fn wrap(data: &str) -> String {
        format!(
            "ISO-10303-21;\nHEADER;\n\
             FILE_DESCRIPTION((''),'2;1');\n\
             FILE_NAME('t.ifc','2026-07-11T00:00:00',('a'),('o'),'p','s','auth');\n\
             FILE_SCHEMA(('IFC4'));\nENDSEC;\nDATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n"
        )
    }

    fn parse(data: &str) -> StepFile {
        parse_step(wrap(data).as_bytes()).expect("parse failed")
    }

    #[test]
    fn plain_material_resolves() {
        let f = parse(
            "#10=IFCWALL('w',$,'Wall',$,$,$,$,$,$);\n\
             #1=IFCMATERIAL('Concrete','cast in place','concrete');\n\
             #2=IFCRELASSOCIATESMATERIAL('r',$,$,$,(#10),#1);",
        );
        let m = Model::from_step(&f);
        assert_eq!(m.material_of(10), Some(1));
        let a = m.material_assignment(10).unwrap();
        assert_eq!(a.name(), Some("Concrete"));
        let MaterialAssignment::Material(mat) = a else {
            panic!("expected material");
        };
        assert_eq!(mat.description, Some("cast in place"));
        assert_eq!(mat.category, Some("concrete"));
        // No association elsewhere.
        assert_eq!(m.material_of(1), None);
    }

    #[test]
    fn layer_set_usage_resolves_layers_and_total_thickness() {
        let f = parse(
            "#10=IFCWALL('w',$,'Wall',$,$,$,$,$,$);\n\
             #1=IFCMATERIAL('Brick',$,$);\n\
             #2=IFCMATERIAL('Insulation',$,$);\n\
             #3=IFCMATERIALLAYER(#1,110.,$,'Outer',$,$,$);\n\
             #4=IFCMATERIALLAYER(#2,80.,.T.,'Cavity',$,'insulation',10);\n\
             #5=IFCMATERIALLAYERSET((#3,#4),'CavityWall',$);\n\
             #6=IFCMATERIALLAYERSETUSAGE(#5,.AXIS2.,.POSITIVE.,-95.,$);\n\
             #7=IFCRELASSOCIATESMATERIAL('r',$,$,$,(#10),#6);",
        );
        let m = Model::from_step(&f);
        let a = m.material_assignment(10).unwrap();
        assert_eq!(a.name(), Some("CavityWall"));
        let MaterialAssignment::LayerSetUsage {
            layer_set,
            direction,
            direction_sense,
            offset_from_reference_line,
            reference_extent,
        } = a
        else {
            panic!("expected layer-set usage");
        };
        assert_eq!(direction, Some("AXIS2"));
        assert_eq!(direction_sense, Some("POSITIVE"));
        assert_eq!(offset_from_reference_line, Some(-95.0));
        assert_eq!(reference_extent, None);
        let set = layer_set.unwrap();
        assert_eq!(set.layers.len(), 2);
        assert!((set.total_thickness() - 190.0).abs() < 1e-12);
        let cavity = &set.layers[1];
        assert_eq!(cavity.material.as_ref().unwrap().name, Some("Insulation"));
        assert_eq!(cavity.is_ventilated, Some(true));
        assert_eq!(cavity.category, Some("insulation"));
        assert_eq!(cavity.priority, Some(10));
        assert_eq!(set.layers[0].is_ventilated, None);
    }

    #[test]
    fn material_list_and_constituent_set_resolve() {
        let f = parse(
            "#1=IFCMATERIAL('Steel',$,$);\n\
             #2=IFCMATERIAL('Glass',$,$);\n\
             #3=IFCMATERIALLIST((#1,#2));\n\
             #4=IFCMATERIALCONSTITUENT('Frame',$,#1,0.3,$);\n\
             #5=IFCMATERIALCONSTITUENT('Glazing',$,#2,0.7,$);\n\
             #6=IFCMATERIALCONSTITUENTSET('WindowParts',$,(#4,#5));",
        );
        let MaterialAssignment::MaterialList(list) = material_assignment(&f, 3).unwrap() else {
            panic!("expected list");
        };
        assert_eq!(list.len(), 2);
        assert_eq!(list[1].name, Some("Glass"));

        let a = material_assignment(&f, 6).unwrap();
        assert_eq!(a.name(), Some("WindowParts"));
        let MaterialAssignment::ConstituentSet(set) = a else {
            panic!("expected constituent set");
        };
        assert_eq!(set.constituents.len(), 2);
        assert_eq!(set.constituents[0].name, Some("Frame"));
        assert_eq!(set.constituents[0].fraction, Some(0.3));
        assert_eq!(
            set.constituents[1].material.as_ref().unwrap().name,
            Some("Glass")
        );
    }

    #[test]
    fn profile_set_usage_resolves() {
        let f = parse(
            "#1=IFCMATERIAL('S355',$,'steel');\n\
             #2=IFCRECTANGLEPROFILEDEF(.AREA.,'R',$,0.2,0.4);\n\
             #3=IFCMATERIALPROFILE('Web',$,#1,#2,$,$);\n\
             #4=IFCMATERIALPROFILESET('Column',$,(#3),$);\n\
             #5=IFCMATERIALPROFILESETUSAGE(#4,5,$);",
        );
        let a = material_assignment(&f, 5).unwrap();
        assert_eq!(a.name(), Some("Column"));
        let MaterialAssignment::ProfileSetUsage {
            profile_set,
            cardinal_point,
            ..
        } = a
        else {
            panic!("expected profile-set usage");
        };
        assert_eq!(cardinal_point, Some(5));
        let set = profile_set.unwrap();
        assert_eq!(set.profiles.len(), 1);
        assert_eq!(set.profiles[0].profile, Some(2));
        assert_eq!(
            set.profiles[0].material.as_ref().unwrap().name,
            Some("S355")
        );
    }

    #[test]
    fn occurrence_material_beats_type_material() {
        // The type carries Ceramic; the occurrence overrides with
        // Steel. An occurrence without its own association falls back
        // to the type's.
        let f = parse(
            "#10=IFCWALL('w1',$,'A',$,$,$,$,$,$);\n\
             #11=IFCWALL('w2',$,'B',$,$,$,$,$,$);\n\
             #30=IFCWALLTYPE('t',$,'WT',$,$,$,$,$,$,.SOLIDWALL.);\n\
             #1=IFCMATERIAL('Ceramic',$,$);\n\
             #2=IFCMATERIAL('Steel',$,$);\n\
             #40=IFCRELASSOCIATESMATERIAL('r1',$,$,$,(#30),#1);\n\
             #41=IFCRELASSOCIATESMATERIAL('r2',$,$,$,(#10),#2);\n\
             #50=IFCRELDEFINESBYTYPE('rt1',$,$,$,(#10,#11),#30);",
        );
        let m = Model::from_step(&f);
        assert_eq!(m.material_of(10), Some(2)); // own wins
        assert_eq!(m.material_of(11), Some(1)); // inherited from type
        assert_eq!(m.material_assignment(11).unwrap().name(), Some("Ceramic"));
    }

    #[test]
    fn non_material_target_is_none() {
        let f = parse("#1=IFCWALL('w',$,'W',$,$,$,$,$,$);");
        assert!(material_assignment(&f, 1).is_none());
        assert!(material_assignment(&f, 99).is_none());
    }
}
