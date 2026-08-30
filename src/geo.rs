//! Phase 4: georeferencing extraction — `IfcProjectedCRS` /
//! `IfcMapConversion` (the IFC 4 map-coordinate binding) and the
//! `IfcSite` latitude / longitude compound measures.
//!
//! An IFC model's engineering coordinates become map coordinates
//! through an `IfcMapConversion` whose `SourceCRS` is the model's
//! `IfcGeometricRepresentationContext` (the
//! `IfcCoordinateReferenceSystemSelect` alternative to a CRS) and
//! whose `TargetCRS` is an `IfcProjectedCRS` naming the projected
//! coordinate reference system (`"EPSG:…"`). The conversion carries
//! the map position of the local origin (`Eastings`, `Northings`,
//! `OrthogonalHeight`), the direction of the local +x axis in map
//! coordinates (`XAxisAbscissa` / `XAxisOrdinate` — the schema names
//! its easting and northing components), and an optional `Scale`.
//!
//! [`MapConversion::to_map`] applies the transformation the
//! `IfcMapConversion` page states, in its order (units digest §1): a
//! scaling of **all three** axes by the same `Scale` (a unit
//! conversion, default 1), then an anti-clockwise rotation about z by
//! `θ = atan2(XAxisOrdinate, XAxisAbscissa)`, then the translation
//! (`Eastings`, `Northings`, `OrthogonalHeight`) — which is *not*
//! scaled. The IFC 4.3 `IfcMapConversionScaled` subtype adds per-axis
//! `FactorX/Y/Z` that scale coordinates (not units) on top of `Scale`,
//! before the rotation; plain conversions never get anisotropic
//! scaling. `IfcRigidOperation` (the other `IfcCoordinateOperation`)
//! is exposed as [`RigidOperation`].

use crate::parser::StepFile;
use crate::schema::TypedEntity;
use crate::value::Value;

/// A resolved `IfcProjectedCRS` — the projected coordinate reference
/// system a map conversion targets.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProjectedCrs<'a> {
    /// The `#id` of the `IfcProjectedCRS` instance.
    pub id: u64,
    /// `Name` — the CRS designation, conventionally an EPSG code
    /// (`"EPSG:25832"`).
    pub name: Option<&'a str>,
    /// `Description`, when set.
    pub description: Option<&'a str>,
    /// The optional geodetic datum identifier.
    pub geodetic_datum: Option<&'a str>,
    /// The optional vertical datum identifier.
    pub vertical_datum: Option<&'a str>,
    /// The optional map-projection identifier.
    pub map_projection: Option<&'a str>,
    /// The optional map-zone identifier.
    pub map_zone: Option<&'a str>,
    /// The `#id` of the optional `MapUnit` (an `IfcNamedUnit`; the
    /// `IsLengthUnit` WHERE rule requires a length unit —
    /// [`named_unit_scale`](crate::schema::named_unit_scale) with
    /// `"LENGTHUNIT"` resolves it).
    pub map_unit: Option<u64>,
}

/// A resolved `IfcMapConversion` — the engineering-to-map coordinate
/// operation.
#[derive(Debug, Clone, PartialEq)]
pub struct MapConversion<'a> {
    /// The `#id` of the `IfcMapConversion` instance.
    pub id: u64,
    /// The `#id` of the `SourceCRS` — for the model binding this is
    /// the `IfcGeometricRepresentationContext` the conversion
    /// georeferences.
    pub source: Option<u64>,
    /// The resolved `TargetCRS`, when it is an `IfcProjectedCRS`.
    pub target_crs: Option<ProjectedCrs<'a>>,
    /// The easting of the local origin in the target CRS.
    pub eastings: f64,
    /// The northing of the local origin in the target CRS.
    pub northings: f64,
    /// The height of the local origin in the target CRS.
    pub orthogonal_height: f64,
    /// The easting component of the local +x axis direction in map
    /// coordinates, when set.
    pub x_axis_abscissa: Option<f64>,
    /// The northing component of the local +x axis direction in map
    /// coordinates, when set.
    pub x_axis_ordinate: Option<f64>,
    /// The optional scale factor from local to map lengths (a unit
    /// conversion applied to x, y and z alike).
    pub scale: Option<f64>,
    /// `IfcMapConversionScaled.FactorX / FactorY / FactorZ` — per-axis
    /// coordinate factors (IFC 4.3), `None` for a plain
    /// `IfcMapConversion`.
    pub factors: Option<[f64; 3]>,
}

impl MapConversion<'_> {
    /// The normalised (cos θ, sin θ) of the rotation carrying the
    /// local +x axis onto its map direction — (`XAxisAbscissa`,
    /// `XAxisOrdinate`) normalised, defaulting to (1, 0) (no rotation)
    /// when unset or degenerate.
    pub fn rotation(&self) -> (f64, f64) {
        let a = self.x_axis_abscissa.unwrap_or(1.0);
        let o = self.x_axis_ordinate.unwrap_or(0.0);
        let len = (a * a + o * o).sqrt();
        if len <= f64::EPSILON {
            (1.0, 0.0)
        } else {
            (a / len, o / len)
        }
    }

    /// The rotation angle θ in radians, anti-clockwise about z, of the
    /// local x-axis in the map plane: `atan2(XAxisOrdinate,
    /// XAxisAbscissa)` (zero when the direction is unset or degenerate).
    pub fn rotation_angle(&self) -> f64 {
        let (cos, sin) = self.rotation();
        sin.atan2(cos)
    }

    /// Map a local engineering point to (easting, northing, height) —
    /// scale all three axes by `Scale` (default 1; times the per-axis
    /// `FactorX/Y/Z` of an `IfcMapConversionScaled`), rotate about z by
    /// [`MapConversion::rotation`], then translate by (`Eastings`,
    /// `Northings`, `OrthogonalHeight`).
    pub fn to_map(&self, point: [f64; 3]) -> [f64; 3] {
        let (cos, sin) = self.rotation();
        let s = self.scale.unwrap_or(1.0);
        let f = self.factors.unwrap_or([1.0; 3]);
        let [x, y, z] = point;
        let (x, y, z) = (s * f[0] * x, s * f[1] * y, s * f[2] * z);
        [
            self.eastings + (x * cos - y * sin),
            self.northings + (x * sin + y * cos),
            self.orthogonal_height + z,
        ]
    }

    /// The inverse of [`MapConversion::to_map`]: a map point back to
    /// local engineering coordinates. `None` when a scale factor is
    /// zero.
    pub fn from_map(&self, point: [f64; 3]) -> Option<[f64; 3]> {
        let (cos, sin) = self.rotation();
        let s = self.scale.unwrap_or(1.0);
        let f = self.factors.unwrap_or([1.0; 3]);
        let [e, n, h] = point;
        let (dx, dy, dz) = (
            e - self.eastings,
            n - self.northings,
            h - self.orthogonal_height,
        );
        // Undo the rotation (transpose), then the scaling.
        let (rx, ry) = (dx * cos + dy * sin, -dx * sin + dy * cos);
        let div = |v: f64, k: f64| if k == 0.0 { None } else { Some(v / k) };
        Some([div(rx, s * f[0])?, div(ry, s * f[1])?, div(dz, s * f[2])?])
    }
}

/// The kind of coordinate pair an `IfcRigidOperation` carries — the
/// `SameCoordinateType` WHERE rule requires both to be lengths or both
/// plane angles.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RigidCoordinateKind {
    /// Both coordinates are `IfcLengthMeasure` (a projected source).
    Length,
    /// Both coordinates are `IfcPlaneAngleMeasure` (a geographic
    /// source: latitude / longitude).
    PlaneAngle,
    /// Untyped or mixed wrappers — the file violates the WHERE rule;
    /// the values are still exposed.
    Unknown,
}

/// A resolved `IfcRigidOperation` (IFC 4.3) — a coordinate operation
/// that is a pure translation of the source CRS origin to
/// (`FirstCoordinate`, `SecondCoordinate`[, `Height`]) in the target.
#[derive(Debug, Clone, PartialEq)]
pub struct RigidOperation<'a> {
    /// The `#id` of the `IfcRigidOperation` instance.
    pub id: u64,
    /// The `#id` of the `SourceCRS`.
    pub source: Option<u64>,
    /// The `#id` of the `TargetCRS` (any `IfcCoordinateReferenceSystem`).
    pub target: Option<u64>,
    /// The resolved target when it is an `IfcProjectedCRS`.
    pub target_crs: Option<ProjectedCrs<'a>>,
    /// `FirstCoordinate` (easting or latitude, by `kind`).
    pub first: f64,
    /// `SecondCoordinate` (northing or longitude, by `kind`).
    pub second: f64,
    /// The optional `Height`.
    pub height: Option<f64>,
    /// Which measure type the pair was written with.
    pub kind: RigidCoordinateKind,
}

/// Resolve one `IfcRigidOperation` instance.
pub fn rigid_operation_by_id(step: &StepFile, id: u64) -> Option<RigidOperation<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCRIGIDOPERATION" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    let measure = |name: &str| -> Option<(f64, Option<&str>)> {
        match view.attr(name)? {
            Value::Typed { keyword, args } => {
                Some((args.first().and_then(Value::as_number)?, Some(keyword)))
            }
            other => Some((other.as_number()?, None)),
        }
    };
    let (first, kf) = measure("FirstCoordinate")?;
    let (second, ks) = measure("SecondCoordinate")?;
    let kind = match (kf, ks) {
        (Some("IFCLENGTHMEASURE"), Some("IFCLENGTHMEASURE")) => RigidCoordinateKind::Length,
        (Some("IFCPLANEANGLEMEASURE"), Some("IFCPLANEANGLEMEASURE")) => {
            RigidCoordinateKind::PlaneAngle
        }
        _ => RigidCoordinateKind::Unknown,
    };
    let target = view.attr("TargetCRS").and_then(Value::as_reference);
    Some(RigidOperation {
        id,
        source: view.attr("SourceCRS").and_then(Value::as_reference),
        target,
        target_crs: target.and_then(|cid| projected_crs(step, cid)),
        first,
        second,
        height: view.attr("Height").and_then(|v| match v {
            Value::Typed { args, .. } => args.first().and_then(Value::as_number),
            other => other.as_number(),
        }),
        kind,
    })
}

/// Every `IfcRigidOperation` in the model, in ascending id order.
pub fn rigid_operations(step: &StepFile) -> Vec<RigidOperation<'_>> {
    step.instances
        .values()
        .filter(|inst| inst.keyword == "IFCRIGIDOPERATION")
        .filter_map(|inst| rigid_operation_by_id(step, inst.id))
        .collect()
}

/// Resolve one `IfcProjectedCRS` instance.
pub fn projected_crs(step: &StepFile, id: u64) -> Option<ProjectedCrs<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCPROJECTEDCRS" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    let s = |name: &str| view.attr(name).and_then(Value::as_str);
    Some(ProjectedCrs {
        id,
        name: s("Name"),
        description: s("Description"),
        geodetic_datum: s("GeodeticDatum"),
        vertical_datum: s("VerticalDatum"),
        map_projection: s("MapProjection"),
        map_zone: s("MapZone"),
        map_unit: view.attr("MapUnit").and_then(Value::as_reference),
    })
}

/// Resolve one `IfcMapConversion` (or `IfcMapConversionScaled`)
/// instance.
pub fn map_conversion_by_id(step: &StepFile, id: u64) -> Option<MapConversion<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCMAPCONVERSION" && inst.keyword != "IFCMAPCONVERSIONSCALED" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    let num = |name: &str| -> Option<f64> {
        match view.attr(name)? {
            Value::Typed { args, .. } => args.first().and_then(Value::as_number),
            other => other.as_number(),
        }
    };
    let factors = if inst.keyword == "IFCMAPCONVERSIONSCALED" {
        Some([num("FactorX")?, num("FactorY")?, num("FactorZ")?])
    } else {
        None
    };
    Some(MapConversion {
        id,
        source: view.attr("SourceCRS").and_then(Value::as_reference),
        target_crs: view
            .attr("TargetCRS")
            .and_then(Value::as_reference)
            .and_then(|cid| projected_crs(step, cid)),
        eastings: num("Eastings")?,
        northings: num("Northings")?,
        orthogonal_height: num("OrthogonalHeight")?,
        x_axis_abscissa: num("XAxisAbscissa"),
        x_axis_ordinate: num("XAxisOrdinate"),
        scale: num("Scale"),
        factors,
    })
}

/// The model's georeferencing: the first `IfcMapConversion` whose
/// `SourceCRS` is an `IfcGeometricRepresentationContext` (the
/// engineering-model binding), in ascending id order. `None` when the
/// model is not georeferenced.
pub fn map_conversion(step: &StepFile) -> Option<MapConversion<'_>> {
    step.instances
        .values()
        .filter(|inst| {
            inst.keyword == "IFCMAPCONVERSION" || inst.keyword == "IFCMAPCONVERSIONSCALED"
        })
        .filter_map(|inst| map_conversion_by_id(step, inst.id))
        .find(|conv| {
            conv.source
                .and_then(|sid| step.get(sid))
                .is_some_and(|src| src.keyword == "IFCGEOMETRICREPRESENTATIONCONTEXT")
        })
}

/// Convert an `IfcCompoundPlaneAngleMeasure` value — the `LIST [3:4]
/// OF INTEGER` (degrees, minutes, seconds[, millionths of a second])
/// with consistent sign — to decimal degrees.
pub fn compound_angle_degrees(value: &Value) -> Option<f64> {
    let parts = value.as_list()?;
    if !(3..=4).contains(&parts.len()) {
        return None;
    }
    let mut nums = [0f64; 4];
    for (slot, part) in nums.iter_mut().zip(parts) {
        *slot = part.as_number()?;
    }
    Some(nums[0] + nums[1] / 60.0 + nums[2] / 3600.0 + nums[3] / 3.6e9)
}

/// The `IfcSite.RefLatitude` / `RefLongitude` of one site, in decimal
/// degrees (WGS84 per the schema note), plus the optional
/// `RefElevation` in model length units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SiteGeolocation {
    /// The `#id` of the `IfcSite`.
    pub site: u64,
    /// Latitude in decimal degrees (positive north).
    pub latitude: f64,
    /// Longitude in decimal degrees (positive east).
    pub longitude: f64,
    /// `RefElevation`, when set.
    pub elevation: Option<f64>,
}

/// The first `IfcSite` carrying both `RefLatitude` and `RefLongitude`,
/// converted to decimal degrees. `None` when no site is geolocated.
pub fn site_geolocation(step: &StepFile) -> Option<SiteGeolocation> {
    step.instances
        .values()
        .filter(|inst| inst.keyword == "IFCSITE")
        .find_map(|inst| {
            let view = TypedEntity::new(inst)?;
            let latitude = compound_angle_degrees(view.attr("RefLatitude")?)?;
            let longitude = compound_angle_degrees(view.attr("RefLongitude")?)?;
            let elevation = view.attr("RefElevation").and_then(Value::as_number);
            Some(SiteGeolocation {
                site: inst.id,
                latitude,
                longitude,
                elevation,
            })
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::parse_step;

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
    fn map_conversion_resolves_crs_and_attributes() {
        let f = parse(
            "#10=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,1.E-5,#11,$);\n\
             #11=IFCAXIS2PLACEMENT3D(#12,$,$);\n\
             #12=IFCCARTESIANPOINT((0.,0.,0.));\n\
             #20=IFCPROJECTEDCRS('EPSG:25832','ETRS89 / UTM 32N','ETRS89',\
             'DHHN92','UTM','32N',#21);\n\
             #21=IFCSIUNIT(*,.LENGTHUNIT.,$,.METRE.);\n\
             #30=IFCMAPCONVERSION(#10,#20,400000.,5600000.,110.,0.8,0.6,1.);",
        );
        let conv = map_conversion(&f).expect("map conversion");
        assert_eq!(conv.id, 30);
        assert_eq!(conv.source, Some(10));
        assert_eq!(conv.eastings, 400000.0);
        assert_eq!(conv.northings, 5600000.0);
        assert_eq!(conv.orthogonal_height, 110.0);
        let crs = conv.target_crs.as_ref().expect("crs");
        assert_eq!(crs.name, Some("EPSG:25832"));
        assert_eq!(crs.geodetic_datum, Some("ETRS89"));
        assert_eq!(crs.vertical_datum, Some("DHHN92"));
        assert_eq!(crs.map_projection, Some("UTM"));
        assert_eq!(crs.map_zone, Some("32N"));
        assert_eq!(
            crate::schema::named_unit_scale(&f, crs.map_unit.unwrap(), "LENGTHUNIT"),
            Some(1.0)
        );

        // (0.8, 0.6) normalises to the 3-4-5 rotation.
        let (cos, sin) = conv.rotation();
        assert!((cos - 0.8).abs() < 1e-12 && (sin - 0.6).abs() < 1e-12);
        // The local origin lands at (E, N, H).
        assert_eq!(conv.to_map([0.0, 0.0, 0.0]), [400000.0, 5600000.0, 110.0]);
        // A unit +x step moves by the rotated axis direction.
        let p = conv.to_map([10.0, 0.0, 2.0]);
        assert!((p[0] - 400008.0).abs() < 1e-9);
        assert!((p[1] - 5600006.0).abs() < 1e-9);
        assert!((p[2] - 112.0).abs() < 1e-12);
    }

    #[test]
    fn map_conversion_defaults_and_scale() {
        // Unset rotation defaults to identity; Scale multiplies all
        // three axes (a millimetre model into a metre CRS).
        let f = parse(
            "#10=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,$,$,$);\n\
             #20=IFCPROJECTEDCRS('EPSG:3857',$,$,$,$,$,$);\n\
             #30=IFCMAPCONVERSION(#10,#20,100.,200.,10.,$,$,0.001);",
        );
        let conv = map_conversion(&f).expect("map conversion");
        assert_eq!(conv.rotation(), (1.0, 0.0));
        assert_eq!(conv.rotation_angle(), 0.0);
        assert_eq!(conv.factors, None);
        let p = conv.to_map([3000.0, -1000.0, 5000.0]);
        assert!((p[0] - 103.0).abs() < 1e-12);
        assert!((p[1] - 199.0).abs() < 1e-12);
        // Height is scaled too; the translation is not.
        assert!((p[2] - 15.0).abs() < 1e-12);
        // Round trip.
        let back = conv.from_map(p).unwrap();
        assert!((back[0] - 3000.0).abs() < 1e-9);
        assert!((back[1] + 1000.0).abs() < 1e-9);
        assert!((back[2] - 5000.0).abs() < 1e-9);
    }

    #[test]
    fn rotation_angle_is_atan2_of_the_axis_vector() {
        // A pure-northing x-axis (abscissa 0) is a quarter turn — no
        // division by zero; a non-unit vector gives the same angle.
        let f = parse(
            "#10=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,$,$,$);\n\
             #20=IFCPROJECTEDCRS('EPSG:3857',$,$,$,$,$,$);\n\
             #30=IFCMAPCONVERSION(#10,#20,0.,0.,0.,0.,5.,$);",
        );
        let conv = map_conversion(&f).unwrap();
        assert!((conv.rotation_angle() - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
        let p = conv.to_map([1.0, 0.0, 0.0]);
        assert!(p[0].abs() < 1e-12 && (p[1] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn map_conversion_scaled_applies_per_axis_factors() {
        // Factors scale coordinates on top of Scale, before rotation.
        let f = parse(
            "#10=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,$,$,$);\n\
             #20=IFCPROJECTEDCRS('EPSG:25832',$,$,$,$,$,$);\n\
             #30=IFCMAPCONVERSIONSCALED(#10,#20,1000.,2000.,30.,0.,1.,2.,\
             1.5,0.5,3.);",
        );
        let conv = map_conversion(&f).expect("scaled binding found");
        assert_eq!(conv.id, 30);
        assert_eq!(conv.factors, Some([1.5, 0.5, 3.0]));
        // (10, 20, 1): scale 2 → (20, 40, 2); factors → (30, 20, 6);
        // rotate 90° → (−20, 30, 6); translate → (980, 2030, 36).
        let p = conv.to_map([10.0, 20.0, 1.0]);
        assert!((p[0] - 980.0).abs() < 1e-9);
        assert!((p[1] - 2030.0).abs() < 1e-9);
        assert!((p[2] - 36.0).abs() < 1e-9);
        let back = conv.from_map(p).unwrap();
        assert!((back[0] - 10.0).abs() < 1e-9 && (back[1] - 20.0).abs() < 1e-9);
        assert!((back[2] - 1.0).abs() < 1e-9);
        assert_eq!(
            map_conversion_by_id(&f, 30).unwrap().factors,
            Some([1.5, 0.5, 3.0])
        );
    }

    #[test]
    fn rigid_operation_resolves_typed_pairs() {
        let f = parse(
            "#20=IFCPROJECTEDCRS('EPSG:25832',$,$,$,$,$,$);\n\
             #21=IFCGEOGRAPHICCRS('EPSG:4326',$,$,$,$);\n\
             #30=IFCRIGIDOPERATION(#20,#21,IFCPLANEANGLEMEASURE(49.5),\
             IFCPLANEANGLEMEASURE(8.25),IFCLENGTHMEASURE(120.));\n\
             #31=IFCRIGIDOPERATION(#21,#20,IFCLENGTHMEASURE(400000.),\
             IFCLENGTHMEASURE(5600000.),$);\n\
             #32=IFCRIGIDOPERATION(#21,#20,1.,2.,$);",
        );
        let ops = rigid_operations(&f);
        assert_eq!(ops.len(), 3);
        assert_eq!(ops[0].id, 30);
        assert_eq!(ops[0].kind, RigidCoordinateKind::PlaneAngle);
        assert_eq!((ops[0].first, ops[0].second), (49.5, 8.25));
        assert_eq!(ops[0].height, Some(120.0));
        assert_eq!(ops[0].source, Some(20));
        assert_eq!(ops[0].target, Some(21));
        assert!(ops[0].target_crs.is_none()); // geographic, not projected
        assert_eq!(ops[1].kind, RigidCoordinateKind::Length);
        assert_eq!(ops[1].target_crs.as_ref().unwrap().name, Some("EPSG:25832"));
        assert_eq!(ops[1].height, None);
        assert_eq!(ops[2].kind, RigidCoordinateKind::Unknown);
        assert!(rigid_operation_by_id(&f, 20).is_none());
        // Rigid operations are not the map binding.
        assert!(map_conversion(&f).is_none());
    }

    #[test]
    fn conversion_without_context_source_is_not_the_model_binding() {
        // A conversion between two CRSs (source is a CRS, not the
        // geometric context) is not returned as the model binding.
        let f = parse(
            "#20=IFCPROJECTEDCRS('EPSG:25832',$,$,$,$,$,$);\n\
             #21=IFCPROJECTEDCRS('EPSG:3857',$,$,$,$,$,$);\n\
             #30=IFCMAPCONVERSION(#20,#21,0.,0.,0.,$,$,$);",
        );
        assert!(map_conversion(&f).is_none());
        // …but it still resolves by id.
        let conv = map_conversion_by_id(&f, 30).expect("by id");
        assert_eq!(conv.source, Some(20));
        assert_eq!(conv.target_crs.as_ref().unwrap().name, Some("EPSG:3857"));
    }

    #[test]
    fn compound_plane_angle_converts_to_decimal_degrees() {
        // 49° 8' 33.6" — with and without the millionth-second part.
        let f = parse(
            "#1=IFCSITE('s',$,'Site',$,$,$,$,$,.ELEMENT.,(49,8,33,600000),\
             (8,-30,0),100.,$,$);",
        );
        let geo = site_geolocation(&f).expect("geolocated site");
        assert_eq!(geo.site, 1);
        assert!((geo.latitude - (49.0 + 8.0 / 60.0 + 33.6 / 3600.0)).abs() < 1e-12);
        // Mixed-sign parts still sum arithmetically (the schema's
        // ConsistentSign rule forbids this file, but the sum is what
        // the list denotes).
        assert!((geo.longitude - (8.0 - 30.0 / 60.0)).abs() < 1e-12);
        assert_eq!(geo.elevation, Some(100.0));

        // Sign-consistent negative measure (southern hemisphere).
        let v = Value::List(vec![
            Value::Integer(-33),
            Value::Integer(-52),
            Value::Integer(-12),
        ]);
        let d = compound_angle_degrees(&v).unwrap();
        assert!((d - (-33.0 - 52.0 / 60.0 - 12.0 / 3600.0)).abs() < 1e-12);

        // Too short / not a list → None.
        assert_eq!(compound_angle_degrees(&Value::Integer(5)), None);
        assert_eq!(
            compound_angle_degrees(&Value::List(vec![Value::Integer(1)])),
            None
        );
    }

    #[test]
    fn ungeoreferenced_model_yields_none() {
        let f = parse(
            "#1=IFCSITE('s',$,'Site',$,$,$,$,$,.ELEMENT.,$,$,$,$,$);\n\
             #10=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,$,$,$);",
        );
        assert!(map_conversion(&f).is_none());
        assert!(site_geolocation(&f).is_none());
    }
}
