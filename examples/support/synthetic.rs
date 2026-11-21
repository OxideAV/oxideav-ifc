//! A synthetic IFC 4 model generator for load / instancing / Boolean
//! measurements: `products` proxy elements laid out on a grid, each
//! placing one of a handful of shared representation maps through an
//! `IfcMappedItem`, plus one wall with three openings per thousand
//! products (`IfcRelVoidsElement`), all contained in one storey.
//!
//! Authored for this crate from the staged schema; shared by the
//! `ifc_load` example and the integration tests.

use std::fmt::Write;

/// Number of distinct shared representation maps.
pub const SHARED_MAPS: usize = 8;

/// A 22-character IFC GlobalId derived from a counter.
fn guid(n: u64) -> String {
    const ALPHABET: &[u8; 64] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_$";
    let mut out = String::with_capacity(22);
    let mut v = n.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0x5DEE_CE66_D1CE_4E5B;
    for k in 0..22 {
        // The first character carries two bits only.
        let digit = if k == 0 { v & 3 } else { v & 63 };
        out.push(ALPHABET[digit as usize] as char);
        v = v.rotate_right(6) ^ n.wrapping_add(k);
    }
    out
}

/// Generate the model text. `products` mapped proxies; walls with
/// openings are added at one per thousand products (at least one).
// Shared source: the tests call this form, the example the other.
#[allow(dead_code)]
pub fn synthetic_model(products: usize) -> String {
    synthetic_model_with(products, true)
}

/// [`synthetic_model`] with a choice of sharing: `shared` FALSE gives
/// every product a filleted I-section extrusion of its own (nothing to
/// instance — the per-item tessellation cost in isolation).
pub fn synthetic_model_with(products: usize, shared: bool) -> String {
    let mut s = String::with_capacity(products * 420 + 16_384);
    s.push_str(
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('ViewDefinition [ReferenceView_V1.2]'),'2;1');\n\
         FILE_NAME('synthetic.ifc','2026-01-01T00:00:00',(''),(''),'oxideav-ifc','oxideav-ifc','');\n\
         FILE_SCHEMA(('IFC4'));\nENDSEC;\nDATA;\n",
    );
    // Units, context, spatial structure.
    s.push_str(
        "#1=IFCSIUNIT(*,.LENGTHUNIT.,.MILLI.,.METRE.);\n\
         #2=IFCSIUNIT(*,.PLANEANGLEUNIT.,$,.RADIAN.);\n\
         #3=IFCUNITASSIGNMENT((#1,#2));\n\
         #4=IFCCARTESIANPOINT((0.,0.,0.));\n\
         #5=IFCAXIS2PLACEMENT3D(#4,$,$);\n\
         #6=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,1.E-5,#5,$);\n\
         #7=IFCPROJECT('0000000000000000000001',$,'Synthetic',$,$,$,$,(#6),#3);\n\
         #8=IFCLOCALPLACEMENT($,#5);\n\
         #9=IFCSITE('0000000000000000000002',$,'Site',$,$,#8,$,$,.ELEMENT.,$,$,$,$,$);\n\
         #10=IFCBUILDING('0000000000000000000003',$,'Building',$,$,#8,$,$,.ELEMENT.,$,$,$);\n\
         #11=IFCBUILDINGSTOREY('0000000000000000000004',$,'Storey',$,$,#8,$,$,.ELEMENT.,0.);\n\
         #12=IFCRELAGGREGATES('0000000000000000000005',$,$,$,#7,(#9));\n\
         #13=IFCRELAGGREGATES('0000000000000000000006',$,$,$,#9,(#10));\n\
         #14=IFCRELAGGREGATES('0000000000000000000007',$,$,$,#10,(#11));\n\
         #15=IFCDIRECTION((0.,0.,1.));\n\
         #16=IFCDIRECTION((1.,0.,0.));\n\
         #17=IFCCARTESIANTRANSFORMATIONOPERATOR3D($,$,#4,$,$);\n\
         #18=IFCAXIS1PLACEMENT(#4,#16);\n",
    );
    // Shared representation maps #100 + 20k .. : profile, solid,
    // style, representation, map (#100 + 20k + 9).
    for k in 0..SHARED_MAPS {
        let b = 100 + 20 * k as u64;
        let (r, g, bl) = (
            0.2 + 0.1 * k as f64,
            0.9 - 0.1 * k as f64,
            0.3 + 0.05 * k as f64,
        );
        let solid = match k % 4 {
            // A filleted I section extruded 3 m.
            0 => format!(
                "#{b}=IFCISHAPEPROFILEDEF(.AREA.,$,$,200.,{}.,12.,18.,15.,$,$);\n\
                 #{}=IFCEXTRUDEDAREASOLID(#{b},$,#15,3000.);\n",
                300 + 20 * k,
                b + 1
            ),
            // A pipe: hollow circle extruded.
            1 => format!(
                "#{b}=IFCCIRCLEHOLLOWPROFILEDEF(.AREA.,$,$,{}.,10.);\n\
                 #{}=IFCEXTRUDEDAREASOLID(#{b},$,#15,2500.);\n",
                80 + 10 * k,
                b + 1
            ),
            // A revolved rectangle (a ring), profile offset from the axis.
            2 => format!(
                "#{}=IFCCARTESIANPOINT((0.,{}.));\n#{}=IFCAXIS2PLACEMENT2D(#{},$);\n\
                 #{b}=IFCRECTANGLEPROFILEDEF(.AREA.,$,#{},120.,60.);\n\
                 #{}=IFCREVOLVEDAREASOLID(#{b},$,#18,6.283185307179586);\n",
                b + 10,
                300 + 10 * k,
                b + 11,
                b + 10,
                b + 11,
                b + 1
            ),
            // A rounded rectangle extruded.
            _ => format!(
                "#{b}=IFCROUNDEDRECTANGLEPROFILEDEF(.AREA.,$,$,400.,{}.,40.);\n\
                 #{}=IFCEXTRUDEDAREASOLID(#{b},$,#15,600.);\n",
                200 + 20 * k,
                b + 1
            ),
        };
        s.push_str(&solid);
        let _ = write!(
            s,
            "#{}=IFCCOLOURRGB($,{r:.3},{g:.3},{bl:.3});\n\
             #{}=IFCSURFACESTYLERENDERING(#{},{},$,$,$,$,$,$,.FLAT.);\n\
             #{}=IFCSURFACESTYLE('Style {k}',.BOTH.,(#{}));\n\
             #{}=IFCSTYLEDITEM(#{},(#{}),$);\n\
             #{}=IFCSHAPEREPRESENTATION(#6,'Body','SweptSolid',(#{}));\n\
             #{}=IFCREPRESENTATIONMAP(#5,#{});\n",
            b + 2,
            b + 3,
            b + 2,
            if k == 3 { "0.5" } else { "0." },
            b + 4,
            b + 3,
            b + 5,
            b + 1,
            b + 4,
            b + 8,
            b + 1,
            b + 9,
            b + 8
        );
    }
    let mut next: u64 = 1000;
    let mut contained: Vec<u64> = Vec::with_capacity(products + products / 1000 + 1);
    let side = (products as f64).sqrt().ceil().max(1.0) as usize;
    for i in 0..products {
        let k = i % SHARED_MAPS;
        let map = 100 + 20 * k as u64 + 9;
        let (x, y) = ((i % side) as f64 * 1500.0, (i / side) as f64 * 1500.0);
        let b = next;
        next += 7;
        if !shared {
            // Own profile + solid in place of the mapped item (#b+3).
            let _ = write!(
                s,
                "#{b}=IFCCARTESIANPOINT(({x:.1},{y:.1},0.));\n\
                 #{}=IFCAXIS2PLACEMENT3D(#{b},$,$);\n\
                 #{}=IFCLOCALPLACEMENT(#8,#{});\n\
                 #{}=IFCISHAPEPROFILEDEF(.AREA.,$,$,200.,{}.,12.,18.,15.,$,$);\n\
                 #{}=IFCEXTRUDEDAREASOLID(#{},$,#15,3000.);\n\
                 #{}=IFCPRODUCTDEFINITIONSHAPE($,$,(#{}));\n\
                 #{}=IFCBUILDINGELEMENTPROXY('{}',$,'Proxy {i}',$,$,#{},#{},$,$);\n",
                b + 1,
                b + 2,
                b + 1,
                b + 3,
                300 + (i % 97),
                b + 4,
                b + 3,
                b + 5,
                next,
                b + 6,
                guid(b),
                b + 2,
                b + 5
            );
            let _ = writeln!(
                s,
                "#{next}=IFCSHAPEREPRESENTATION(#6,'Body','SweptSolid',(#{}));",
                b + 4
            );
            next += 1;
            contained.push(b + 6);
            continue;
        }
        let _ = write!(
            s,
            "#{b}=IFCCARTESIANPOINT(({x:.1},{y:.1},0.));\n\
             #{}=IFCAXIS2PLACEMENT3D(#{b},$,$);\n\
             #{}=IFCLOCALPLACEMENT(#8,#{});\n\
             #{}=IFCMAPPEDITEM(#{map},#17);\n\
             #{}=IFCSHAPEREPRESENTATION(#6,'Body','MappedRepresentation',(#{}));\n\
             #{}=IFCPRODUCTDEFINITIONSHAPE($,$,(#{}));\n\
             #{}=IFCBUILDINGELEMENTPROXY('{}',$,'Proxy {i}',$,$,#{},#{},$,$);\n",
            b + 1,
            b + 2,
            b + 1,
            b + 3,
            b + 4,
            b + 3,
            b + 5,
            b + 4,
            b + 6,
            guid(b),
            b + 2,
            b + 5
        );
        contained.push(b + 6);
    }
    // Walls with three openings each.
    let walls = (products / 1000).max(1);
    for w in 0..walls {
        let b = next;
        next += 40;
        let y = -5000.0 - 2000.0 * w as f64;
        let _ = write!(
            s,
            "#{b}=IFCCARTESIANPOINT((0.,{y:.1},0.));\n\
             #{}=IFCAXIS2PLACEMENT3D(#{b},$,$);\n\
             #{}=IFCLOCALPLACEMENT(#8,#{});\n\
             #{}=IFCCARTESIANPOINT((3000.,150.));\n\
             #{}=IFCAXIS2PLACEMENT2D(#{},$);\n\
             #{}=IFCRECTANGLEPROFILEDEF(.AREA.,$,#{},6000.,300.);\n\
             #{}=IFCEXTRUDEDAREASOLID(#{},$,#15,3000.);\n\
             #{}=IFCSHAPEREPRESENTATION(#6,'Body','SweptSolid',(#{}));\n\
             #{}=IFCPRODUCTDEFINITIONSHAPE($,$,(#{}));\n\
             #{}=IFCWALL('{}',$,'Wall {w}',$,$,#{},#{},$,$);\n",
            b + 1,
            b + 2,
            b + 1,
            b + 3,
            b + 4,
            b + 3,
            b + 5,
            b + 4,
            b + 6,
            b + 5,
            b + 7,
            b + 6,
            b + 8,
            b + 7,
            b + 9,
            guid(b),
            b + 2,
            b + 8
        );
        contained.push(b + 9);
        for o in 0..3u64 {
            let ob = b + 10 + 10 * o;
            let ox = 1000.0 + 1800.0 * o as f64;
            let _ = write!(
                s,
                "#{ob}=IFCCARTESIANPOINT(({ox:.1},150.,900.));\n\
                 #{}=IFCAXIS2PLACEMENT3D(#{ob},$,$);\n\
                 #{}=IFCLOCALPLACEMENT(#{},#{});\n\
                 #{}=IFCRECTANGLEPROFILEDEF(.AREA.,$,$,1000.,400.);\n\
                 #{}=IFCEXTRUDEDAREASOLID(#{},$,#15,1200.);\n\
                 #{}=IFCSHAPEREPRESENTATION(#6,'Body','SweptSolid',(#{}));\n\
                 #{}=IFCPRODUCTDEFINITIONSHAPE($,$,(#{}));\n\
                 #{}=IFCOPENINGELEMENT('{}',$,'Opening',$,$,#{},#{},$,.OPENING.);\n\
                 #{}=IFCRELVOIDSELEMENT('{}',$,$,$,#{},#{});\n",
                ob + 1,
                ob + 2,
                b + 2,
                ob + 1,
                ob + 3,
                ob + 4,
                ob + 3,
                ob + 5,
                ob + 4,
                ob + 6,
                ob + 5,
                ob + 7,
                guid(ob),
                ob + 2,
                ob + 6,
                ob + 8,
                guid(ob + 8),
                b + 9,
                ob + 7
            );
        }
    }
    let _ = write!(
        s,
        "#{next}=IFCRELCONTAINEDINSPATIALSTRUCTURE('{}',$,$,$,(",
        guid(next)
    );
    for (i, id) in contained.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        let _ = write!(s, "#{id}");
    }
    s.push_str("),#11);\nENDSEC;\nEND-ISO-10303-21;\n");
    s
}
