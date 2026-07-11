//! Phase 4: typed property-set and quantity-set extraction.
//!
//! IFC attaches its non-geometric data to objects through
//! `IfcRelDefinesByProperties`: each relationship hands a set of
//! occurrence objects an `IfcPropertySetDefinition` — either an
//! `IfcPropertySet` (named [`Property`] values: the `Pset_WallCommon`
//! family) or an `IfcElementQuantity` (measured [`Quantity`] values:
//! the `Qto_…` base-quantity sets). Type objects
//! (`IfcRelDefinesByType.RelatingType`) contribute their
//! `HasPropertySets` to every occurrence they type.
//!
//! [`Model`](crate::Model) folds both relationships
//! ([`Model::defined_property_sets`](crate::Model::defined_property_sets),
//! [`Model::type_of`](crate::Model::type_of),
//! [`Model::property_set_ids`](crate::Model::property_set_ids)); this
//! module resolves the definitions those edges point at into a typed
//! surface:
//!
//! * [`property_set`] — one `IfcPropertySet` → a [`PropertySet`] of
//!   named [`Property`] values covering the whole `IfcSimpleProperty`
//!   family (single / enumerated / bounded / list / table / reference
//!   values) plus nested `IfcComplexProperty` groups.
//! * [`element_quantity`] — one `IfcElementQuantity` → an
//!   [`ElementQuantity`] of [`Quantity`] values (length / area /
//!   volume / count / weight / time and nested
//!   `IfcPhysicalComplexQuantity` groups).
//! * [`IfcValue`] — a SELECT-typed leaf value with the measure-type
//!   wrapper kept (`IFCBOOLEAN(.T.)` → type name + payload).
//!
//! Everything borrows from the parsed [`StepFile`]; unresolvable or
//! malformed members are skipped rather than failing the whole set
//! (use [`StepFile::dangling_references`] for validation).

use crate::parser::StepFile;
use crate::schema::TypedEntity;
use crate::value::Value;

/// Recursion bound for nested `IfcComplexProperty` /
/// `IfcPhysicalComplexQuantity` groups (a self-referential group
/// terminates instead of looping).
const MAX_NEST_DEPTH: usize = 16;

/// A leaf IFC value: the payload of an `IfcValue` SELECT slot, with
/// the defined-type wrapper kept when the file spelled one.
///
/// On the wire a SELECT-typed attribute is either a typed parameter —
/// `IFCBOOLEAN(.T.)`, `IFCLENGTHMEASURE(2.4)` — or (for non-SELECT
/// contexts) a plain literal. Both arrive here: [`IfcValue::type_name`]
/// carries the wrapper keyword when present, and the payload accessors
/// see through it either way.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IfcValue<'a> {
    type_name: Option<&'a str>,
    raw: &'a Value,
}

impl<'a> IfcValue<'a> {
    /// Wrap one parsed value; `$` / `*` slots yield `None`. A typed
    /// parameter is unwrapped to its (single) payload argument.
    pub fn of(value: &'a Value) -> Option<Self> {
        match value {
            Value::Unset | Value::Derived => None,
            Value::Typed { keyword, args } => Some(Self {
                type_name: Some(keyword),
                raw: args.first()?,
            }),
            other => Some(Self {
                type_name: None,
                raw: other,
            }),
        }
    }

    /// The defined-type wrapper keyword (`"IFCBOOLEAN"`,
    /// `"IFCLENGTHMEASURE"`, …), when the file used the typed form.
    pub fn type_name(&self) -> Option<&'a str> {
        self.type_name
    }

    /// The raw payload value.
    pub fn raw(&self) -> &'a Value {
        self.raw
    }

    /// Numeric payload widened to `f64` (REAL or INTEGER literal).
    pub fn as_number(&self) -> Option<f64> {
        self.raw.as_number()
    }

    /// Integer payload.
    pub fn as_integer(&self) -> Option<i64> {
        self.raw.as_integer()
    }

    /// String payload (labels, identifiers, text).
    pub fn as_str(&self) -> Option<&'a str> {
        self.raw.as_str()
    }

    /// Enumeration payload without the delimiting dots.
    pub fn as_enum(&self) -> Option<&'a str> {
        self.raw.as_enum()
    }

    /// Boolean payload: the LOGICAL / BOOLEAN literals `.T.` / `.F.`
    /// (`.U.` — LOGICAL unknown — yields `None`).
    pub fn as_bool(&self) -> Option<bool> {
        match self.raw.as_enum()? {
            "T" => Some(true),
            "F" => Some(false),
            _ => None,
        }
    }
}

/// Collect an optional `LIST OF IfcValue` attribute into wrapped leaf
/// values, skipping `$` members.
fn value_list<'a>(entity: &TypedEntity<'a>, name: &str) -> Vec<IfcValue<'a>> {
    entity
        .attr(name)
        .and_then(Value::as_list)
        .map(|items| items.iter().filter_map(IfcValue::of).collect())
        .unwrap_or_default()
}

/// An optional reference attribute (`$` → `None`).
fn opt_ref(entity: &TypedEntity<'_>, name: &str) -> Option<u64> {
    entity.attr(name)?.as_reference()
}

/// An optional string attribute (`$` → `None`).
fn opt_str<'a>(entity: &TypedEntity<'a>, name: &str) -> Option<&'a str> {
    entity.attr(name)?.as_str()
}

/// One property inside an `IfcPropertySet` (or a nested complex
/// group): the shared `IfcProperty` header plus the subtype payload.
#[derive(Debug, Clone, PartialEq)]
pub struct Property<'a> {
    /// The `#id` of the property instance.
    pub id: u64,
    /// `IfcProperty.Name` — the property identifier within its set.
    pub name: Option<&'a str>,
    /// `IfcProperty.Description`, when set.
    pub description: Option<&'a str>,
    /// The subtype payload.
    pub value: PropertyValue<'a>,
}

impl<'a> Property<'a> {
    /// The single nominal value, when this is a
    /// [`PropertyValue::Single`] with a set value — the overwhelmingly
    /// common case, so worth a direct accessor.
    pub fn nominal(&self) -> Option<&IfcValue<'a>> {
        match &self.value {
            PropertyValue::Single {
                value: Some(value), ..
            } => Some(value),
            _ => None,
        }
    }
}

/// The subtype payload of one [`Property`] — the `IfcSimpleProperty`
/// family plus `IfcComplexProperty` (ISO 16739 §8.11: property
/// resource).
#[derive(Debug, Clone, PartialEq)]
pub enum PropertyValue<'a> {
    /// `IfcPropertySingleValue(NominalValue, Unit)`.
    Single {
        /// The nominal value (`$` when the writer left it open — the
        /// buildingSMART fixtures serialise empty labels instead).
        value: Option<IfcValue<'a>>,
        /// The `#id` of the optional `IfcUnit` override.
        unit: Option<u64>,
    },
    /// `IfcPropertyEnumeratedValue(EnumerationValues,
    /// EnumerationReference)`.
    Enumerated {
        /// The selected enumeration values (all drawn from the
        /// reference's `EnumerationValues` per the WHERE rule).
        values: Vec<IfcValue<'a>>,
        /// The `#id` of the optional `IfcPropertyEnumeration` listing
        /// the allowed values.
        enumeration: Option<u64>,
    },
    /// `IfcPropertyBoundedValue(UpperBoundValue, LowerBoundValue,
    /// Unit, SetPointValue)`.
    Bounded {
        /// The upper bound, when set.
        upper: Option<IfcValue<'a>>,
        /// The lower bound, when set.
        lower: Option<IfcValue<'a>>,
        /// The set point (the operating value inside the bounds).
        set_point: Option<IfcValue<'a>>,
        /// The `#id` of the optional `IfcUnit` override.
        unit: Option<u64>,
    },
    /// `IfcPropertyListValue(ListValues, Unit)`.
    List {
        /// The ordered member values.
        values: Vec<IfcValue<'a>>,
        /// The `#id` of the optional `IfcUnit` override.
        unit: Option<u64>,
    },
    /// `IfcPropertyTableValue(DefiningValues, DefinedValues,
    /// Expression, DefiningUnit, DefinedUnit, CurveInterpolation)` —
    /// paired defining → defined value rows.
    Table {
        /// The defining (input) column.
        defining: Vec<IfcValue<'a>>,
        /// The defined (output) column, index-paired with `defining`.
        defined: Vec<IfcValue<'a>>,
        /// The optional expression text relating the columns.
        expression: Option<&'a str>,
        /// The `#id` of the optional defining-column `IfcUnit`.
        defining_unit: Option<u64>,
        /// The `#id` of the optional defined-column `IfcUnit`.
        defined_unit: Option<u64>,
        /// The `IfcCurveInterpolationEnum` literal, when set.
        interpolation: Option<&'a str>,
    },
    /// `IfcPropertyReferenceValue(UsageName, PropertyReference)` — a
    /// reference to a resource-level object rather than a leaf value.
    Reference {
        /// The optional usage label.
        usage_name: Option<&'a str>,
        /// The `#id` of the referenced resource object.
        reference: Option<u64>,
    },
    /// `IfcComplexProperty(UsageName, HasProperties)` — a named group
    /// of nested properties.
    Complex {
        /// The group usage identifier.
        usage_name: Option<&'a str>,
        /// The nested properties (recursion bounded; a self-referential
        /// group terminates).
        properties: Vec<Property<'a>>,
    },
}

/// A resolved `IfcPropertySet`: the `IfcRoot` header plus its named
/// properties.
#[derive(Debug, Clone, PartialEq)]
pub struct PropertySet<'a> {
    /// The `#id` of the `IfcPropertySet` instance.
    pub id: u64,
    /// `IfcRoot.GlobalId`.
    pub global_id: Option<&'a str>,
    /// `IfcRoot.Name` — required by the `ExistsName` WHERE rule
    /// (`"Pset_WallCommon"`, …); `None` only for malformed files.
    pub name: Option<&'a str>,
    /// `IfcRoot.Description`, when set.
    pub description: Option<&'a str>,
    /// The member properties, in serialisation order. Property names
    /// are unique within the set (`UniquePropertyNames`).
    pub properties: Vec<Property<'a>>,
}

impl<'a> PropertySet<'a> {
    /// Find a member property by `Name` (exact, case-sensitive — IFC
    /// property identifiers are case-sensitive).
    pub fn property(&self, name: &str) -> Option<&Property<'a>> {
        self.properties.iter().find(|p| p.name == Some(name))
    }
}

/// Resolve one `IfcPropertySet` instance into a [`PropertySet`].
/// Returns `None` when `id` is missing or not an `IfcPropertySet`.
/// Member properties that fail to resolve are skipped.
pub fn property_set(step: &StepFile, id: u64) -> Option<PropertySet<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCPROPERTYSET" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    let properties = view
        .attr("HasProperties")
        .and_then(Value::as_list)
        .map(|items| {
            items
                .iter()
                .filter_map(Value::as_reference)
                .filter_map(|pid| resolve_property(step, pid, MAX_NEST_DEPTH))
                .collect()
        })
        .unwrap_or_default();
    Some(PropertySet {
        id,
        global_id: view.global_id(),
        name: view.name(),
        description: view.description(),
        properties,
    })
}

/// Resolve one `IfcProperty` instance (any `IfcSimpleProperty` subtype
/// or `IfcComplexProperty`) into a [`Property`].
pub fn property(step: &StepFile, id: u64) -> Option<Property<'_>> {
    resolve_property(step, id, MAX_NEST_DEPTH)
}

fn resolve_property(step: &StepFile, id: u64, depth: usize) -> Option<Property<'_>> {
    let inst = step.get(id)?;
    let view = TypedEntity::new(inst)?;
    let value = match inst.keyword.as_str() {
        "IFCPROPERTYSINGLEVALUE" => PropertyValue::Single {
            value: view.attr("NominalValue").and_then(IfcValue::of),
            unit: opt_ref(&view, "Unit"),
        },
        "IFCPROPERTYENUMERATEDVALUE" => PropertyValue::Enumerated {
            values: value_list(&view, "EnumerationValues"),
            enumeration: opt_ref(&view, "EnumerationReference"),
        },
        "IFCPROPERTYBOUNDEDVALUE" => PropertyValue::Bounded {
            upper: view.attr("UpperBoundValue").and_then(IfcValue::of),
            lower: view.attr("LowerBoundValue").and_then(IfcValue::of),
            set_point: view.attr("SetPointValue").and_then(IfcValue::of),
            unit: opt_ref(&view, "Unit"),
        },
        "IFCPROPERTYLISTVALUE" => PropertyValue::List {
            values: value_list(&view, "ListValues"),
            unit: opt_ref(&view, "Unit"),
        },
        "IFCPROPERTYTABLEVALUE" => PropertyValue::Table {
            defining: value_list(&view, "DefiningValues"),
            defined: value_list(&view, "DefinedValues"),
            expression: opt_str(&view, "Expression"),
            defining_unit: opt_ref(&view, "DefiningUnit"),
            defined_unit: opt_ref(&view, "DefinedUnit"),
            interpolation: view.attr("CurveInterpolation").and_then(Value::as_enum),
        },
        "IFCPROPERTYREFERENCEVALUE" => PropertyValue::Reference {
            usage_name: opt_str(&view, "UsageName"),
            reference: opt_ref(&view, "PropertyReference"),
        },
        "IFCCOMPLEXPROPERTY" => {
            let properties = if depth == 0 {
                Vec::new()
            } else {
                view.attr("HasProperties")
                    .and_then(Value::as_list)
                    .map(|items| {
                        items
                            .iter()
                            .filter_map(Value::as_reference)
                            .filter(|nested| *nested != id)
                            .filter_map(|pid| resolve_property(step, pid, depth - 1))
                            .collect()
                    })
                    .unwrap_or_default()
            };
            PropertyValue::Complex {
                usage_name: opt_str(&view, "UsageName"),
                properties,
            }
        }
        _ => return None,
    };
    Some(Property {
        id,
        name: view.name(),
        description: view.description(),
        value,
    })
}

/// The measured payload of one [`Quantity`] — the
/// `IfcPhysicalSimpleQuantity` family plus
/// `IfcPhysicalComplexQuantity` groups.
#[derive(Debug, Clone, PartialEq)]
pub enum QuantityValue<'a> {
    /// `IfcQuantityLength.LengthValue` (model length unit unless the
    /// quantity carries a `Unit` override).
    Length(f64),
    /// `IfcQuantityArea.AreaValue`.
    Area(f64),
    /// `IfcQuantityVolume.VolumeValue`.
    Volume(f64),
    /// `IfcQuantityCount.CountValue` — dimensionless.
    Count(f64),
    /// `IfcQuantityWeight.WeightValue` (a mass measure).
    Weight(f64),
    /// `IfcQuantityTime.TimeValue`.
    Time(f64),
    /// `IfcPhysicalComplexQuantity(HasQuantities, Discrimination,
    /// Quality, Usage)` — a named group of nested quantities.
    Complex {
        /// The discrimination label (what distinguishes the group —
        /// e.g. a layer name).
        discrimination: Option<&'a str>,
        /// The optional quality label.
        quality: Option<&'a str>,
        /// The optional usage label.
        usage: Option<&'a str>,
        /// The nested quantities (recursion bounded; a self-referential
        /// group terminates).
        quantities: Vec<Quantity<'a>>,
    },
}

/// One quantity inside an `IfcElementQuantity`: the shared
/// `IfcPhysicalQuantity` header plus the measured payload.
#[derive(Debug, Clone, PartialEq)]
pub struct Quantity<'a> {
    /// The `#id` of the quantity instance.
    pub id: u64,
    /// `IfcPhysicalQuantity.Name` — unique within the set.
    pub name: Option<&'a str>,
    /// `IfcPhysicalQuantity.Description`, when set.
    pub description: Option<&'a str>,
    /// The `#id` of the optional per-quantity `IfcNamedUnit` override
    /// (`IfcPhysicalSimpleQuantity.Unit`); `None` on complex groups
    /// and when the model default applies.
    pub unit: Option<u64>,
    /// The optional formula label recording how the value was derived.
    pub formula: Option<&'a str>,
    /// The measured payload.
    pub value: QuantityValue<'a>,
}

impl Quantity<'_> {
    /// The scalar payload for the six simple kinds (`None` for a
    /// complex group).
    pub fn scalar(&self) -> Option<f64> {
        match self.value {
            QuantityValue::Length(v)
            | QuantityValue::Area(v)
            | QuantityValue::Volume(v)
            | QuantityValue::Count(v)
            | QuantityValue::Weight(v)
            | QuantityValue::Time(v) => Some(v),
            QuantityValue::Complex { .. } => None,
        }
    }

    /// The `IfcUnitEnum` dimension literal the WHERE rules require of
    /// this quantity's unit (`None` for counts — dimensionless — and
    /// complex groups).
    fn unit_type(&self) -> Option<&'static str> {
        match self.value {
            QuantityValue::Length(_) => Some("LENGTHUNIT"),
            QuantityValue::Area(_) => Some("AREAUNIT"),
            QuantityValue::Volume(_) => Some("VOLUMEUNIT"),
            QuantityValue::Weight(_) => Some("MASSUNIT"),
            QuantityValue::Time(_) => Some("TIMEUNIT"),
            QuantityValue::Count(_) | QuantityValue::Complex { .. } => None,
        }
    }

    /// The factor from this quantity's unit to SI reference units
    /// (metres / m² / m³ / kilograms / seconds).
    ///
    /// The optional per-quantity `IfcPhysicalSimpleQuantity.Unit`
    /// override wins (resolved by
    /// [`named_unit_scale`](crate::schema::named_unit_scale) against
    /// the dimension the quantity kind requires); otherwise the model
    /// default of that dimension applies
    /// ([`length_unit_scale`](crate::schema::length_unit_scale) and
    /// friends). Counts scale by 1; complex groups and unresolvable
    /// units yield `None`.
    pub fn si_scale(&self, step: &StepFile) -> Option<f64> {
        if matches!(self.value, QuantityValue::Count(_)) {
            return Some(1.0);
        }
        let unit_type = self.unit_type()?;
        match self.unit {
            Some(uid) => crate::schema::named_unit_scale(step, uid, unit_type),
            None => match self.value {
                QuantityValue::Length(_) => crate::schema::length_unit_scale(step),
                QuantityValue::Area(_) => crate::schema::area_unit_scale(step),
                QuantityValue::Volume(_) => crate::schema::volume_unit_scale(step),
                QuantityValue::Weight(_) => crate::schema::mass_unit_scale(step),
                QuantityValue::Time(_) => crate::schema::time_unit_scale(step),
                QuantityValue::Count(_) | QuantityValue::Complex { .. } => None,
            },
        }
    }

    /// The scalar payload converted to SI reference units —
    /// [`Quantity::scalar`] × [`Quantity::si_scale`].
    pub fn si_value(&self, step: &StepFile) -> Option<f64> {
        Some(self.scalar()? * self.si_scale(step)?)
    }
}

/// A resolved `IfcElementQuantity`: the `IfcRoot` header plus its
/// measured quantities.
#[derive(Debug, Clone, PartialEq)]
pub struct ElementQuantity<'a> {
    /// The `#id` of the `IfcElementQuantity` instance.
    pub id: u64,
    /// `IfcRoot.GlobalId`.
    pub global_id: Option<&'a str>,
    /// `IfcRoot.Name` (`"Qto_WallBaseQuantities"`, …).
    pub name: Option<&'a str>,
    /// `IfcRoot.Description`, when set.
    pub description: Option<&'a str>,
    /// The optional method-of-measurement label.
    pub method_of_measurement: Option<&'a str>,
    /// The member quantities, in serialisation order. Quantity names
    /// are unique within the set (`UniqueQuantityNames`).
    pub quantities: Vec<Quantity<'a>>,
}

impl<'a> ElementQuantity<'a> {
    /// Find a member quantity by `Name` (exact, case-sensitive).
    pub fn quantity(&self, name: &str) -> Option<&Quantity<'a>> {
        self.quantities.iter().find(|q| q.name == Some(name))
    }
}

/// Resolve one `IfcElementQuantity` instance into an
/// [`ElementQuantity`]. Returns `None` when `id` is missing or not an
/// `IfcElementQuantity`. Member quantities that fail to resolve are
/// skipped.
pub fn element_quantity(step: &StepFile, id: u64) -> Option<ElementQuantity<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCELEMENTQUANTITY" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    let quantities = view
        .attr("Quantities")
        .and_then(Value::as_list)
        .map(|items| {
            items
                .iter()
                .filter_map(Value::as_reference)
                .filter_map(|qid| resolve_quantity(step, qid, MAX_NEST_DEPTH))
                .collect()
        })
        .unwrap_or_default();
    Some(ElementQuantity {
        id,
        global_id: view.global_id(),
        name: view.name(),
        description: view.description(),
        method_of_measurement: opt_str(&view, "MethodOfMeasurement"),
        quantities,
    })
}

/// Resolve one `IfcPhysicalQuantity` instance (any simple kind or a
/// complex group) into a [`Quantity`].
pub fn quantity(step: &StepFile, id: u64) -> Option<Quantity<'_>> {
    resolve_quantity(step, id, MAX_NEST_DEPTH)
}

fn resolve_quantity(step: &StepFile, id: u64, depth: usize) -> Option<Quantity<'_>> {
    let inst = step.get(id)?;
    let view = TypedEntity::new(inst)?;
    // The value slot is a defined-type measure — a plain REAL on the
    // wire (writers may spell the typed form; both are accepted).
    let measure = |attr: &str| -> Option<f64> {
        match view.attr(attr)? {
            Value::Typed { args, .. } => args.first().and_then(Value::as_number),
            other => other.as_number(),
        }
    };
    let value = match inst.keyword.as_str() {
        "IFCQUANTITYLENGTH" => QuantityValue::Length(measure("LengthValue")?),
        "IFCQUANTITYAREA" => QuantityValue::Area(measure("AreaValue")?),
        "IFCQUANTITYVOLUME" => QuantityValue::Volume(measure("VolumeValue")?),
        "IFCQUANTITYCOUNT" => QuantityValue::Count(measure("CountValue")?),
        "IFCQUANTITYWEIGHT" => QuantityValue::Weight(measure("WeightValue")?),
        "IFCQUANTITYTIME" => QuantityValue::Time(measure("TimeValue")?),
        "IFCPHYSICALCOMPLEXQUANTITY" => {
            let quantities = if depth == 0 {
                Vec::new()
            } else {
                view.attr("HasQuantities")
                    .and_then(Value::as_list)
                    .map(|items| {
                        items
                            .iter()
                            .filter_map(Value::as_reference)
                            .filter(|nested| *nested != id)
                            .filter_map(|qid| resolve_quantity(step, qid, depth - 1))
                            .collect()
                    })
                    .unwrap_or_default()
            };
            QuantityValue::Complex {
                discrimination: opt_str(&view, "Discrimination"),
                quality: opt_str(&view, "Quality"),
                usage: opt_str(&view, "Usage"),
                quantities,
            }
        }
        _ => return None,
    };
    Some(Quantity {
        id,
        name: view.name(),
        description: view.description(),
        unit: opt_ref(&view, "Unit"),
        formula: opt_str(&view, "Formula"),
        value,
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
    fn single_value_property_set_resolves() {
        let f = parse(
            "#1=IFCPROPERTYSET('gid',$,'Pset_WallCommon','desc',(#2,#3,#4));\n\
             #2=IFCPROPERTYSINGLEVALUE('IsExternal','ext',IFCBOOLEAN(.T.),$);\n\
             #3=IFCPROPERTYSINGLEVALUE('ThermalTransmittance',$,\
             IFCTHERMALTRANSMITTANCEMEASURE(2.4E-1),$);\n\
             #4=IFCPROPERTYSINGLEVALUE('Reference',$,IFCIDENTIFIER('W-1'),#9);\n\
             #9=IFCSIUNIT(*,.LENGTHUNIT.,$,.METRE.);",
        );
        let pset = property_set(&f, 1).unwrap();
        assert_eq!(pset.name, Some("Pset_WallCommon"));
        assert_eq!(pset.global_id, Some("gid"));
        assert_eq!(pset.description, Some("desc"));
        assert_eq!(pset.properties.len(), 3);

        let ext = pset.property("IsExternal").unwrap();
        assert_eq!(ext.description, Some("ext"));
        let v = ext.nominal().unwrap();
        assert_eq!(v.type_name(), Some("IFCBOOLEAN"));
        assert_eq!(v.as_bool(), Some(true));

        let tt = pset.property("ThermalTransmittance").unwrap();
        assert_eq!(tt.nominal().unwrap().as_number(), Some(0.24));
        assert_eq!(
            tt.nominal().unwrap().type_name(),
            Some("IFCTHERMALTRANSMITTANCEMEASURE")
        );

        let re = pset.property("Reference").unwrap();
        assert_eq!(re.nominal().unwrap().as_str(), Some("W-1"));
        assert!(matches!(
            re.value,
            PropertyValue::Single { unit: Some(9), .. }
        ));

        // Absent name → None; non-pset id → None.
        assert!(pset.property("NoSuch").is_none());
        assert!(property_set(&f, 2).is_none());
        assert!(property_set(&f, 999).is_none());
    }

    #[test]
    fn unset_nominal_value_is_none() {
        let f = parse(
            "#1=IFCPROPERTYSET('g',$,'P',$,(#2));\n\
             #2=IFCPROPERTYSINGLEVALUE('Open',$,$,$);",
        );
        let pset = property_set(&f, 1).unwrap();
        let p = pset.property("Open").unwrap();
        assert!(p.nominal().is_none());
        assert_eq!(
            p.value,
            PropertyValue::Single {
                value: None,
                unit: None
            }
        );
    }

    #[test]
    fn enumerated_value_resolves_values_and_reference() {
        let f = parse(
            "#1=IFCPROPERTYENUMERATEDVALUE('Fire','r',(IFCLABEL('F30'),IFCLABEL('F60')),#2);\n\
             #2=IFCPROPERTYENUMERATION('FireRatings',(IFCLABEL('F30'),IFCLABEL('F60'),\
             IFCLABEL('F90')),$);",
        );
        let p = property(&f, 1).unwrap();
        let PropertyValue::Enumerated {
            values,
            enumeration,
        } = &p.value
        else {
            panic!("expected enumerated, got {:?}", p.value);
        };
        assert_eq!(enumeration, &Some(2));
        let labels: Vec<_> = values.iter().filter_map(IfcValue::as_str).collect();
        assert_eq!(labels, ["F30", "F60"]);
    }

    #[test]
    fn bounded_value_resolves_bounds_and_set_point() {
        let f = parse(
            "#1=IFCPROPERTYBOUNDEDVALUE('Temp',$,IFCTHERMODYNAMICTEMPERATUREMEASURE(26.),\
             IFCTHERMODYNAMICTEMPERATUREMEASURE(18.),$,\
             IFCTHERMODYNAMICTEMPERATUREMEASURE(21.));",
        );
        let p = property(&f, 1).unwrap();
        let PropertyValue::Bounded {
            upper,
            lower,
            set_point,
            unit,
        } = &p.value
        else {
            panic!("expected bounded");
        };
        assert_eq!(upper.unwrap().as_number(), Some(26.0));
        assert_eq!(lower.unwrap().as_number(), Some(18.0));
        assert_eq!(set_point.unwrap().as_number(), Some(21.0));
        assert_eq!(*unit, None);
    }

    #[test]
    fn list_and_table_values_resolve() {
        let f = parse(
            "#1=IFCPROPERTYLISTVALUE('Sizes',$,(IFCLENGTHMEASURE(10.),\
             IFCLENGTHMEASURE(20.)),$);\n\
             #2=IFCPROPERTYTABLEVALUE('Flow',$,(IFCREAL(1.),IFCREAL(2.)),\
             (IFCREAL(10.),IFCREAL(40.)),'q = 10*p*p',$,$,.LINEAR.);",
        );
        let list = property(&f, 1).unwrap();
        let PropertyValue::List { values, unit } = &list.value else {
            panic!("expected list");
        };
        assert_eq!(*unit, None);
        let nums: Vec<_> = values.iter().filter_map(IfcValue::as_number).collect();
        assert_eq!(nums, [10.0, 20.0]);

        let table = property(&f, 2).unwrap();
        let PropertyValue::Table {
            defining,
            defined,
            expression,
            interpolation,
            ..
        } = &table.value
        else {
            panic!("expected table");
        };
        assert_eq!(defining.len(), 2);
        assert_eq!(defined.len(), 2);
        assert_eq!(defined[1].as_number(), Some(40.0));
        assert_eq!(*expression, Some("q = 10*p*p"));
        assert_eq!(*interpolation, Some("LINEAR"));
    }

    #[test]
    fn reference_value_and_complex_nesting_resolve() {
        let f = parse(
            "#1=IFCCOMPLEXPROPERTY('Layer',$,'Pad',(#2,#3));\n\
             #2=IFCPROPERTYSINGLEVALUE('Thickness',$,IFCLENGTHMEASURE(12.5),$);\n\
             #3=IFCPROPERTYREFERENCEVALUE('Mat',$,'usage',#4);\n\
             #4=IFCMATERIAL('Concrete',$,$);",
        );
        let p = property(&f, 1).unwrap();
        let PropertyValue::Complex {
            usage_name,
            properties,
        } = &p.value
        else {
            panic!("expected complex");
        };
        assert_eq!(*usage_name, Some("Pad"));
        assert_eq!(properties.len(), 2);
        assert_eq!(properties[0].nominal().unwrap().as_number(), Some(12.5));
        let PropertyValue::Reference {
            usage_name,
            reference,
        } = &properties[1].value
        else {
            panic!("expected reference");
        };
        assert_eq!(*usage_name, Some("usage"));
        assert_eq!(*reference, Some(4));
    }

    #[test]
    fn self_referential_complex_property_terminates() {
        // A malformed complex property containing itself must not loop.
        let f = parse(
            "#1=IFCCOMPLEXPROPERTY('Loop',$,'u',(#1,#2));\n\
             #2=IFCPROPERTYSINGLEVALUE('Ok',$,IFCREAL(1.),$);",
        );
        let p = property(&f, 1).unwrap();
        let PropertyValue::Complex { properties, .. } = &p.value else {
            panic!("expected complex");
        };
        // The self edge is dropped; the sibling still resolves.
        assert_eq!(properties.len(), 1);
        assert_eq!(properties[0].name, Some("Ok"));
    }

    #[test]
    fn element_quantity_resolves_all_simple_kinds() {
        let f = parse(
            "#1=IFCELEMENTQUANTITY('g',$,'Qto_WallBaseQuantities',$,'BaseQuantities',\
             (#2,#3,#4,#5,#6,#7));\n\
             #2=IFCQUANTITYLENGTH('Width',$,$,0.3,$);\n\
             #3=IFCQUANTITYAREA('NetSideArea','side',$,11.4,$);\n\
             #4=IFCQUANTITYVOLUME('NetVolume',$,$,3.42,'w*h*t');\n\
             #5=IFCQUANTITYCOUNT('Openings',$,$,2.,$);\n\
             #6=IFCQUANTITYWEIGHT('Mass',$,$,8208.,$);\n\
             #7=IFCQUANTITYTIME('Cure',$,$,86400.,$);",
        );
        let eq = element_quantity(&f, 1).unwrap();
        assert_eq!(eq.name, Some("Qto_WallBaseQuantities"));
        assert_eq!(eq.method_of_measurement, Some("BaseQuantities"));
        assert_eq!(eq.quantities.len(), 6);
        assert_eq!(
            eq.quantity("Width").unwrap().value,
            QuantityValue::Length(0.3)
        );
        let area = eq.quantity("NetSideArea").unwrap();
        assert_eq!(area.value, QuantityValue::Area(11.4));
        assert_eq!(area.description, Some("side"));
        let vol = eq.quantity("NetVolume").unwrap();
        assert_eq!(vol.value, QuantityValue::Volume(3.42));
        assert_eq!(vol.formula, Some("w*h*t"));
        assert_eq!(
            eq.quantity("Openings").unwrap().value,
            QuantityValue::Count(2.0)
        );
        assert_eq!(
            eq.quantity("Mass").unwrap().value,
            QuantityValue::Weight(8208.0)
        );
        assert_eq!(
            eq.quantity("Cure").unwrap().value,
            QuantityValue::Time(86400.0)
        );
        assert_eq!(eq.quantity("Width").unwrap().scalar(), Some(0.3));
        // Not an element quantity → None.
        assert!(element_quantity(&f, 2).is_none());
    }

    #[test]
    fn quantity_unit_override_and_typed_measure_accepted() {
        let f = parse(
            "#1=IFCQUANTITYLENGTH('W',$,#8,IFCLENGTHMEASURE(300.),$);\n\
             #8=IFCSIUNIT(*,.LENGTHUNIT.,.MILLI.,.METRE.);",
        );
        let q = quantity(&f, 1).unwrap();
        assert_eq!(q.value, QuantityValue::Length(300.0));
        assert_eq!(q.unit, Some(8));
    }

    #[test]
    fn complex_quantity_groups_nested_quantities() {
        let f = parse(
            "#1=IFCPHYSICALCOMPLEXQUANTITY('Layers','d',(#2,#3),'ByLayer','q','u');\n\
             #2=IFCQUANTITYLENGTH('Core',$,$,0.2,$);\n\
             #3=IFCQUANTITYLENGTH('Insulation',$,$,0.1,$);",
        );
        let q = quantity(&f, 1).unwrap();
        assert_eq!(q.scalar(), None);
        let QuantityValue::Complex {
            discrimination,
            quality,
            usage,
            quantities,
        } = &q.value
        else {
            panic!("expected complex");
        };
        assert_eq!(*discrimination, Some("ByLayer"));
        assert_eq!(*quality, Some("q"));
        assert_eq!(*usage, Some("u"));
        assert_eq!(quantities.len(), 2);
        assert_eq!(quantities[1].value, QuantityValue::Length(0.1));
    }

    #[test]
    fn model_folds_rel_defines_by_properties() {
        let f = parse(
            "#10=IFCWALL('w',$,'Wall',$,$,$,$,$,$);\n\
             #1=IFCPROPERTYSET('g1',$,'Pset_A',$,(#2));\n\
             #2=IFCPROPERTYSINGLEVALUE('X',$,IFCREAL(1.),$);\n\
             #5=IFCELEMENTQUANTITY('g2',$,'Qto_B',$,$,(#6));\n\
             #6=IFCQUANTITYAREA('A',$,$,4.,$);\n\
             #20=IFCRELDEFINESBYPROPERTIES('r1',$,$,$,(#10),#1);\n\
             #21=IFCRELDEFINESBYPROPERTIES('r2',$,$,$,(#10),#5);",
        );
        let m = Model::from_step(&f);
        assert_eq!(m.defined_property_sets(10), &[1, 5]);
        let psets = m.property_sets(10);
        assert_eq!(psets.len(), 1);
        assert_eq!(psets[0].name, Some("Pset_A"));
        let quants = m.element_quantities(10);
        assert_eq!(quants.len(), 1);
        assert_eq!(quants[0].quantity("A").unwrap().scalar(), Some(4.0));
        // Nothing assigned elsewhere.
        assert!(m.defined_property_sets(1).is_empty());
    }

    #[test]
    fn rel_defines_accepts_definition_set_aggregate() {
        // RelatingPropertyDefinition may be an IfcPropertySetDefinitionSet
        // (an aggregate of definitions) per the SELECT.
        let f = parse(
            "#10=IFCWALL('w',$,'Wall',$,$,$,$,$,$);\n\
             #1=IFCPROPERTYSET('g1',$,'Pset_A',$,(#2));\n\
             #2=IFCPROPERTYSINGLEVALUE('X',$,IFCREAL(1.),$);\n\
             #3=IFCPROPERTYSET('g3',$,'Pset_B',$,(#2));\n\
             #20=IFCRELDEFINESBYPROPERTIES('r1',$,$,$,(#10),(#1,#3));",
        );
        let m = Model::from_step(&f);
        assert_eq!(m.defined_property_sets(10), &[1, 3]);
    }

    #[test]
    fn type_property_sets_inherit_with_occurrence_shadowing() {
        // The occurrence's own Pset_Common shadows the type-level set
        // of the same name; the type-only set is inherited.
        let f = parse(
            "#10=IFCWALL('w',$,'Wall',$,$,$,$,$,$);\n\
             #30=IFCWALLTYPE('t',$,'WT',$,$,(#40,#42),$,$,$,.SOLIDWALL.);\n\
             #40=IFCPROPERTYSET('g40',$,'Pset_Common',$,(#41));\n\
             #41=IFCPROPERTYSINGLEVALUE('IsExternal',$,IFCBOOLEAN(.F.),$);\n\
             #42=IFCPROPERTYSET('g42',$,'Pset_TypeOnly',$,(#43));\n\
             #43=IFCPROPERTYSINGLEVALUE('Rating',$,IFCLABEL('A'),$);\n\
             #50=IFCPROPERTYSET('g50',$,'Pset_Common',$,(#51));\n\
             #51=IFCPROPERTYSINGLEVALUE('IsExternal',$,IFCBOOLEAN(.T.),$);\n\
             #60=IFCRELDEFINESBYPROPERTIES('r',$,$,$,(#10),#50);\n\
             #61=IFCRELDEFINESBYTYPE('rt',$,$,$,(#10),#30);",
        );
        let m = Model::from_step(&f);
        assert_eq!(m.type_of(10), Some(30));
        // Occurrence set first, then only the non-shadowed type set.
        assert_eq!(m.property_set_ids(10), vec![50, 42]);
        let psets = m.property_sets(10);
        assert_eq!(psets.len(), 2);
        // The surviving Pset_Common is the occurrence's (.T.).
        let common = psets
            .iter()
            .find(|p| p.name == Some("Pset_Common"))
            .unwrap();
        assert_eq!(
            common
                .property("IsExternal")
                .unwrap()
                .nominal()
                .unwrap()
                .as_bool(),
            Some(true)
        );
        assert!(psets.iter().any(|p| p.name == Some("Pset_TypeOnly")));
        // An untyped object inherits nothing.
        assert_eq!(m.property_set_ids(30), Vec::<u64>::new());
    }

    #[test]
    fn quantities_scale_to_si_with_model_defaults() {
        // A millimetre model with explicit (unprefixed) SI area/volume
        // units, kilogram mass, second time.
        let f = parse(
            "#1=IFCSIUNIT(*,.LENGTHUNIT.,.MILLI.,.METRE.);\n\
             #2=IFCSIUNIT(*,.AREAUNIT.,$,.SQUARE_METRE.);\n\
             #3=IFCSIUNIT(*,.VOLUMEUNIT.,$,.CUBIC_METRE.);\n\
             #4=IFCSIUNIT(*,.MASSUNIT.,.KILO.,.GRAM.);\n\
             #5=IFCSIUNIT(*,.TIMEUNIT.,$,.SECOND.);\n\
             #6=IFCUNITASSIGNMENT((#1,#2,#3,#4,#5));\n\
             #7=IFCPROJECT('x',$,$,$,$,$,$,$,#6);\n\
             #10=IFCQUANTITYLENGTH('W',$,$,300.,$);\n\
             #11=IFCQUANTITYAREA('A',$,$,2.5,$);\n\
             #12=IFCQUANTITYVOLUME('V',$,$,0.75,$);\n\
             #13=IFCQUANTITYWEIGHT('M',$,$,5.,$);\n\
             #14=IFCQUANTITYTIME('T',$,$,60.,$);\n\
             #15=IFCQUANTITYCOUNT('N',$,$,2.,$);",
        );
        use crate::schema;
        assert_eq!(schema::length_unit_scale(&f), Some(1e-3));
        assert_eq!(schema::area_unit_scale(&f), Some(1.0));
        assert_eq!(schema::volume_unit_scale(&f), Some(1.0));
        assert_eq!(schema::mass_unit_scale(&f), Some(1.0)); // kilo+gram = kg
        assert_eq!(schema::time_unit_scale(&f), Some(1.0));

        let close = |a: Option<f64>, b: f64| (a.unwrap() - b).abs() < 1e-12;
        assert!(close(quantity(&f, 10).unwrap().si_value(&f), 0.3));
        assert!(close(quantity(&f, 11).unwrap().si_value(&f), 2.5));
        assert!(close(quantity(&f, 12).unwrap().si_value(&f), 0.75));
        assert!(close(quantity(&f, 13).unwrap().si_value(&f), 5.0));
        assert!(close(quantity(&f, 14).unwrap().si_value(&f), 60.0));
        // Counts are dimensionless — scale 1 even with model units.
        assert!(close(quantity(&f, 15).unwrap().si_value(&f), 2.0));
    }

    #[test]
    fn quantity_unit_override_beats_model_default() {
        // Model default is metres; the quantity carries a millimetre
        // override, which wins.
        let f = parse(
            "#1=IFCSIUNIT(*,.LENGTHUNIT.,$,.METRE.);\n\
             #6=IFCUNITASSIGNMENT((#1));\n\
             #7=IFCPROJECT('x',$,$,$,$,$,$,$,#6);\n\
             #8=IFCSIUNIT(*,.LENGTHUNIT.,.MILLI.,.METRE.);\n\
             #10=IFCQUANTITYLENGTH('W',$,#8,300.,$);\n\
             #11=IFCQUANTITYLENGTH('X',$,$,2.,$);\n\
             #12=IFCQUANTITYLENGTH('Bad',$,#13,1.,$);\n\
             #13=IFCSIUNIT(*,.AREAUNIT.,$,.SQUARE_METRE.);",
        );
        let q = quantity(&f, 10).unwrap();
        assert!((q.si_value(&f).unwrap() - 0.3).abs() < 1e-12);
        // No override → model metre.
        assert!((quantity(&f, 11).unwrap().si_value(&f).unwrap() - 2.0).abs() < 1e-12);
        // A dimension-mismatched override (the WR21 violation) refuses
        // to scale rather than mis-scaling.
        assert_eq!(quantity(&f, 12).unwrap().si_scale(&f), None);
    }

    #[test]
    fn mass_units_anchor_on_gram() {
        // An unprefixed gram model yields 10⁻³ kg per unit; a pound
        // resolves through a conversion chain over the gram base.
        let f = parse(
            "#1=IFCSIUNIT(*,.MASSUNIT.,$,.GRAM.);\n\
             #6=IFCUNITASSIGNMENT((#1));\n\
             #7=IFCPROJECT('x',$,$,$,$,$,$,$,#6);",
        );
        assert_eq!(crate::schema::mass_unit_scale(&f), Some(1e-3));

        let f = parse(
            "#1=IFCSIUNIT(*,.MASSUNIT.,.KILO.,.GRAM.);\n\
             #2=IFCMEASUREWITHUNIT(IFCRATIOMEASURE(0.45359237),#1);\n\
             #3=IFCCONVERSIONBASEDUNIT(*,.MASSUNIT.,'POUND',#2);\n\
             #6=IFCUNITASSIGNMENT((#3));\n\
             #7=IFCPROJECT('x',$,$,$,$,$,$,$,#6);",
        );
        let s = crate::schema::mass_unit_scale(&f).unwrap();
        assert!((s - 0.45359237).abs() < 1e-12);
    }

    #[test]
    fn prefixed_area_and_volume_si_units_are_refused() {
        // Whether .MILLI. on SQUARE_METRE scales the base length or
        // the derived unit is not stated by the staged schema text —
        // the scale resolves to None instead of guessing.
        let f = parse(
            "#1=IFCSIUNIT(*,.AREAUNIT.,.MILLI.,.SQUARE_METRE.);\n\
             #2=IFCSIUNIT(*,.VOLUMEUNIT.,.CENTI.,.CUBIC_METRE.);\n\
             #6=IFCUNITASSIGNMENT((#1,#2));\n\
             #7=IFCPROJECT('x',$,$,$,$,$,$,$,#6);",
        );
        assert_eq!(crate::schema::area_unit_scale(&f), None);
        assert_eq!(crate::schema::volume_unit_scale(&f), None);

        // A conversion-based area chain over an unprefixed SI base
        // still resolves (square foot in m²).
        let f = parse(
            "#1=IFCSIUNIT(*,.AREAUNIT.,$,.SQUARE_METRE.);\n\
             #2=IFCMEASUREWITHUNIT(IFCRATIOMEASURE(0.09290304),#1);\n\
             #3=IFCCONVERSIONBASEDUNIT(*,.AREAUNIT.,'SQUARE FOOT',#2);\n\
             #6=IFCUNITASSIGNMENT((#3));\n\
             #7=IFCPROJECT('x',$,$,$,$,$,$,$,#6);",
        );
        let s = crate::schema::area_unit_scale(&f).unwrap();
        assert!((s - 0.09290304).abs() < 1e-12);
    }
}
