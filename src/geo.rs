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
//! [`MapConversion::to_map`] applies the planar similarity those
//! attributes describe: the local x/y plane is rotated by the
//! normalised x-axis direction, scaled, and translated to
//! (`Eastings`, `Northings`); the local origin's height lands at
//! `OrthogonalHeight`. The staged schema text gives no rule for
//! scaling heights, so `Scale` is applied to the planar components
//! only and `z` translates unscaled.

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
    /// The optional scale factor from local to map lengths.
    pub scale: Option<f64>,
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

    /// Map a local engineering point to (easting, northing, height):
    /// the planar components are rotated by [`MapConversion::rotation`],
    /// scaled by `Scale` (default 1), and translated to
    /// (`Eastings`, `Northings`); the height translates by
    /// `OrthogonalHeight` unscaled (the staged schema text states no
    /// height-scaling rule).
    pub fn to_map(&self, point: [f64; 3]) -> [f64; 3] {
        let (cos, sin) = self.rotation();
        let s = self.scale.unwrap_or(1.0);
        let [x, y, z] = point;
        [
            self.eastings + s * (x * cos - y * sin),
            self.northings + s * (x * sin + y * cos),
            self.orthogonal_height + z,
        ]
    }
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

/// Resolve one `IfcMapConversion` instance.
pub fn map_conversion_by_id(step: &StepFile, id: u64) -> Option<MapConversion<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCMAPCONVERSION" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    let num = |name: &str| -> Option<f64> {
        match view.attr(name)? {
            Value::Typed { args, .. } => args.first().and_then(Value::as_number),
            other => other.as_number(),
        }
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
    })
}

/// The model's georeferencing: the first `IfcMapConversion` whose
/// `SourceCRS` is an `IfcGeometricRepresentationContext` (the
/// engineering-model binding), in ascending id order. `None` when the
/// model is not georeferenced.
pub fn map_conversion(step: &StepFile) -> Option<MapConversion<'_>> {
    step.instances
        .values()
        .filter(|inst| inst.keyword == "IFCMAPCONVERSION")
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
        // Unset rotation defaults to identity; Scale multiplies the
        // planar components only.
        let f = parse(
            "#10=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,$,$,$);\n\
             #20=IFCPROJECTEDCRS('EPSG:3857',$,$,$,$,$,$);\n\
             #30=IFCMAPCONVERSION(#10,#20,100.,200.,10.,$,$,0.001);",
        );
        let conv = map_conversion(&f).expect("map conversion");
        assert_eq!(conv.rotation(), (1.0, 0.0));
        let p = conv.to_map([3000.0, -1000.0, 5.0]);
        assert!((p[0] - 103.0).abs() < 1e-12);
        assert!((p[1] - 199.0).abs() < 1e-12);
        // Height translates unscaled.
        assert!((p[2] - 15.0).abs() < 1e-12);
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
