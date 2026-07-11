//! Phase 4: external-reference associations — classifications
//! (`IfcRelAssociatesClassification`) and documents
//! (`IfcRelAssociatesDocument`).
//!
//! A classification assignment tags objects with an entry of an
//! external classification system (Uniclass, OmniClass, …): either
//! the `IfcClassification` root itself or — the common case — an
//! `IfcClassificationReference` naming one code
//! (`Identification`), chained through `ReferencedSource` up to the
//! root system. A document assignment attaches an external document,
//! either as bare `IfcDocumentInformation` metadata or through an
//! `IfcDocumentReference` into it.
//!
//! [`Model`](crate::Model) folds both relationships
//! ([`Model::classifications_of`](crate::Model::classifications_of),
//! [`Model::documents_of`](crate::Model::documents_of) — each merging
//! the occurrence's associations with its type object's); this module
//! resolves the select targets.

use crate::parser::StepFile;
use crate::schema::TypedEntity;
use crate::value::Value;

/// Recursion bound for `ReferencedSource` chains (a self-referential
/// chain terminates instead of looping).
const MAX_CHAIN_DEPTH: usize = 16;

/// An `IfcClassification` — the root of an external classification
/// system.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Classification<'a> {
    /// The `#id` of the `IfcClassification` instance.
    pub id: u64,
    /// The publishing `Source` organisation, when set.
    pub source: Option<&'a str>,
    /// The `Edition` label, when set.
    pub edition: Option<&'a str>,
    /// The `EditionDate` (an `IfcDate` string), when set.
    pub edition_date: Option<&'a str>,
    /// `Name` — the system name (required by the schema).
    pub name: Option<&'a str>,
    /// `Description`, when set.
    pub description: Option<&'a str>,
    /// The `Location` URI of the system, when set.
    pub location: Option<&'a str>,
}

/// An `IfcClassificationReference` — one entry (code) of a
/// classification system.
#[derive(Debug, Clone, PartialEq)]
pub struct ClassificationReference<'a> {
    /// The `#id` of the reference instance.
    pub id: u64,
    /// The `Location` URI, when set.
    pub location: Option<&'a str>,
    /// `Identification` — the classification code (`"EF_25_10"`).
    pub identification: Option<&'a str>,
    /// The human-readable `Name` of the entry, when set.
    pub name: Option<&'a str>,
    /// `Description`, when set.
    pub description: Option<&'a str>,
    /// The `Sort` key, when set.
    pub sort: Option<&'a str>,
    /// The `ReferencedSource` parent — a further reference (a
    /// facet/group level) or the classification root. `None` when
    /// unset or unresolvable.
    pub source: Option<Box<ClassificationParent<'a>>>,
}

impl<'a> ClassificationReference<'a> {
    /// The classification system at the root of the
    /// `ReferencedSource` chain, when reachable.
    pub fn system(&self) -> Option<&Classification<'a>> {
        match self.source.as_deref()? {
            ClassificationParent::Classification(c) => Some(c),
            ClassificationParent::Reference(r) => r.system(),
        }
    }
}

/// The `ReferencedSource` of a classification reference.
#[derive(Debug, Clone, PartialEq)]
pub enum ClassificationParent<'a> {
    /// The classification system root.
    Classification(Classification<'a>),
    /// An intermediate reference level.
    Reference(ClassificationReference<'a>),
}

/// A resolved `IfcClassificationSelect` — what an
/// `IfcRelAssociatesClassification` assigned.
#[derive(Debug, Clone, PartialEq)]
pub enum ClassificationAssignment<'a> {
    /// The whole system was associated.
    Classification(Classification<'a>),
    /// One entry (code) was associated.
    Reference(ClassificationReference<'a>),
}

impl<'a> ClassificationAssignment<'a> {
    /// The headline code or name: a reference's `Identification`
    /// (else its `Name`), or the system `Name`.
    pub fn code(&self) -> Option<&'a str> {
        match self {
            Self::Classification(c) => c.name,
            Self::Reference(r) => r.identification.or(r.name),
        }
    }
}

/// `IfcDocumentInformation` metadata (the identifying subset —
/// actors and validity dates stay accessible positionally).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DocumentInformation<'a> {
    /// The `#id` of the information instance.
    pub id: u64,
    /// `Identification` — the document id (required).
    pub identification: Option<&'a str>,
    /// `Name` (required).
    pub name: Option<&'a str>,
    /// `Description`, when set.
    pub description: Option<&'a str>,
    /// The `Location` URI, when set.
    pub location: Option<&'a str>,
    /// `Purpose`, when set.
    pub purpose: Option<&'a str>,
    /// `Revision` label, when set.
    pub revision: Option<&'a str>,
    /// `ElectronicFormat` (a MIME type), when set.
    pub electronic_format: Option<&'a str>,
    /// The `Confidentiality` enum literal, when set.
    pub confidentiality: Option<&'a str>,
    /// The `Status` enum literal (`"DRAFT"`, `"FINAL"`, …), when set.
    pub status: Option<&'a str>,
}

/// An `IfcDocumentReference` — a pointer into (a part of) a document.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DocumentReference<'a> {
    /// The `#id` of the reference instance.
    pub id: u64,
    /// The `Location` URI, when set.
    pub location: Option<&'a str>,
    /// `Identification` — the referenced fragment id, when set.
    pub identification: Option<&'a str>,
    /// `Name`, when set.
    pub name: Option<&'a str>,
    /// `Description`, when set.
    pub description: Option<&'a str>,
    /// The `ReferencedDocument` metadata, when resolvable.
    pub document: Option<DocumentInformation<'a>>,
}

/// A resolved `IfcDocumentSelect` — what an
/// `IfcRelAssociatesDocument` assigned.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DocumentAssignment<'a> {
    /// Bare document metadata.
    Information(DocumentInformation<'a>),
    /// A reference into a document.
    Reference(DocumentReference<'a>),
}

impl<'a> DocumentAssignment<'a> {
    /// The headline document name: the information `Name`, or the
    /// reference's `Name` (else the referenced document's).
    pub fn name(&self) -> Option<&'a str> {
        match self {
            Self::Information(i) => i.name,
            Self::Reference(r) => r.name.or_else(|| r.document.as_ref().and_then(|d| d.name)),
        }
    }
}

/// An optional string attribute (`$` → `None`).
fn opt_str<'a>(entity: &TypedEntity<'a>, name: &str) -> Option<&'a str> {
    entity.attr(name)?.as_str()
}

/// Resolve one `IfcClassification` instance.
pub fn classification(step: &StepFile, id: u64) -> Option<Classification<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCCLASSIFICATION" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    Some(Classification {
        id,
        source: opt_str(&view, "Source"),
        edition: opt_str(&view, "Edition"),
        edition_date: opt_str(&view, "EditionDate"),
        name: opt_str(&view, "Name"),
        description: opt_str(&view, "Description"),
        location: opt_str(&view, "Location"),
    })
}

/// Resolve one `IfcClassificationReference` instance, following its
/// `ReferencedSource` chain (depth-capped).
pub fn classification_reference(step: &StepFile, id: u64) -> Option<ClassificationReference<'_>> {
    resolve_reference(step, id, MAX_CHAIN_DEPTH)
}

fn resolve_reference(
    step: &StepFile,
    id: u64,
    depth: usize,
) -> Option<ClassificationReference<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCCLASSIFICATIONREFERENCE" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    let source = view
        .attr("ReferencedSource")
        .and_then(Value::as_reference)
        .filter(|parent| *parent != id && depth > 0)
        .and_then(|parent| {
            if let Some(c) = classification(step, parent) {
                Some(ClassificationParent::Classification(c))
            } else {
                resolve_reference(step, parent, depth - 1).map(ClassificationParent::Reference)
            }
        })
        .map(Box::new);
    Some(ClassificationReference {
        id,
        location: opt_str(&view, "Location"),
        identification: opt_str(&view, "Identification"),
        name: opt_str(&view, "Name"),
        description: opt_str(&view, "Description"),
        sort: opt_str(&view, "Sort"),
        source,
    })
}

/// Resolve an `IfcClassificationSelect` target.
pub fn classification_assignment(step: &StepFile, id: u64) -> Option<ClassificationAssignment<'_>> {
    let inst = step.get(id)?;
    match inst.keyword.as_str() {
        "IFCCLASSIFICATION" => Some(ClassificationAssignment::Classification(classification(
            step, id,
        )?)),
        "IFCCLASSIFICATIONREFERENCE" => Some(ClassificationAssignment::Reference(
            classification_reference(step, id)?,
        )),
        _ => None,
    }
}

/// Resolve one `IfcDocumentInformation` instance.
pub fn document_information(step: &StepFile, id: u64) -> Option<DocumentInformation<'_>> {
    let inst = step.get(id)?;
    if inst.keyword != "IFCDOCUMENTINFORMATION" {
        return None;
    }
    let view = TypedEntity::new(inst)?;
    Some(DocumentInformation {
        id,
        identification: opt_str(&view, "Identification"),
        name: opt_str(&view, "Name"),
        description: opt_str(&view, "Description"),
        location: opt_str(&view, "Location"),
        purpose: opt_str(&view, "Purpose"),
        revision: opt_str(&view, "Revision"),
        electronic_format: opt_str(&view, "ElectronicFormat"),
        confidentiality: view.attr("Confidentiality").and_then(Value::as_enum),
        status: view.attr("Status").and_then(Value::as_enum),
    })
}

/// Resolve an `IfcDocumentSelect` target.
pub fn document_assignment(step: &StepFile, id: u64) -> Option<DocumentAssignment<'_>> {
    let inst = step.get(id)?;
    match inst.keyword.as_str() {
        "IFCDOCUMENTINFORMATION" => Some(DocumentAssignment::Information(document_information(
            step, id,
        )?)),
        "IFCDOCUMENTREFERENCE" => {
            let view = TypedEntity::new(inst)?;
            Some(DocumentAssignment::Reference(DocumentReference {
                id,
                location: opt_str(&view, "Location"),
                identification: opt_str(&view, "Identification"),
                name: opt_str(&view, "Name"),
                description: opt_str(&view, "Description"),
                document: view
                    .attr("ReferencedDocument")
                    .and_then(Value::as_reference)
                    .and_then(|did| document_information(step, did)),
            }))
        }
        _ => None,
    }
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
    fn classification_reference_chain_resolves_to_system() {
        let f = parse(
            "#10=IFCWALL('w',$,'Wall',$,$,$,$,$,$);\n\
             #1=IFCCLASSIFICATION('CSI','4.0','2026-01-01','Uniclass','uc',\
             'https://example.invalid/uniclass',$);\n\
             #2=IFCCLASSIFICATIONREFERENCE('https://example.invalid/ef',\
             'EF_25','Walls group',#1,$,$);\n\
             #3=IFCCLASSIFICATIONREFERENCE($,'EF_25_10','Walls',#2,'desc','10');\n\
             #20=IFCRELASSOCIATESCLASSIFICATION('r',$,$,$,(#10),#3);",
        );
        let m = Model::from_step(&f);
        assert_eq!(m.classifications_of(10), vec![3]);
        let a = classification_assignment(&f, 3).unwrap();
        assert_eq!(a.code(), Some("EF_25_10"));
        let ClassificationAssignment::Reference(r) = a else {
            panic!("expected reference");
        };
        assert_eq!(r.name, Some("Walls"));
        assert_eq!(r.sort, Some("10"));
        // Chain: #3 → #2 → #1 (the system).
        let system = r.system().expect("system root");
        assert_eq!(system.name, Some("Uniclass"));
        assert_eq!(system.source, Some("CSI"));
        assert_eq!(system.edition, Some("4.0"));
        let Some(parent) = r.source.as_deref() else {
            panic!("parent level");
        };
        let ClassificationParent::Reference(level) = parent else {
            panic!("intermediate level");
        };
        assert_eq!(level.identification, Some("EF_25"));
    }

    #[test]
    fn self_referential_classification_chain_terminates() {
        let f = parse("#3=IFCCLASSIFICATIONREFERENCE($,'X',$,#3,$,$);");
        let r = classification_reference(&f, 3).unwrap();
        assert!(r.source.is_none());
        assert!(r.system().is_none());
    }

    #[test]
    fn classifications_merge_occurrence_and_type() {
        let f = parse(
            "#10=IFCWALL('w',$,'Wall',$,$,$,$,$,$);\n\
             #30=IFCWALLTYPE('t',$,'WT',$,$,$,$,$,$,.SOLIDWALL.);\n\
             #1=IFCCLASSIFICATION($,$,$,'SystemA',$,$,$);\n\
             #2=IFCCLASSIFICATION($,$,$,'SystemB',$,$,$);\n\
             #20=IFCRELASSOCIATESCLASSIFICATION('r1',$,$,$,(#10),#1);\n\
             #21=IFCRELASSOCIATESCLASSIFICATION('r2',$,$,$,(#30),#2);\n\
             #22=IFCRELASSOCIATESCLASSIFICATION('r3',$,$,$,(#30),#1);\n\
             #40=IFCRELDEFINESBYTYPE('rt',$,$,$,(#10),#30);",
        );
        let m = Model::from_step(&f);
        // Occurrence first, then the type's (already-present ids not
        // duplicated).
        assert_eq!(m.classifications_of(10), vec![1, 2]);
        assert_eq!(m.classifications_of(30), vec![2, 1]);
        assert_eq!(
            classification_assignment(&f, 1).unwrap().code(),
            Some("SystemA")
        );
    }

    #[test]
    fn document_assignment_resolves_reference_and_information() {
        let f = parse(
            "#10=IFCWALL('w',$,'Wall',$,$,$,$,$,$);\n\
             #1=IFCDOCUMENTINFORMATION('DOC-7','Fire strategy','fs',\
             'https://example.invalid/fs.pdf','safety',$,$,'B',$,$,$,$,\
             'application/pdf',$,$,.CONFIDENTIAL.,.FINALDRAFT.);\n\
             #2=IFCDOCUMENTREFERENCE('https://example.invalid/fs.pdf#p3',\
             'p3','Fire strategy p3',$,#1);\n\
             #20=IFCRELASSOCIATESDOCUMENT('r',$,$,$,(#10),#2);",
        );
        let m = Model::from_step(&f);
        assert_eq!(m.documents_of(10), vec![2]);
        let a = document_assignment(&f, 2).unwrap();
        assert_eq!(a.name(), Some("Fire strategy p3"));
        let DocumentAssignment::Reference(r) = a else {
            panic!("expected reference");
        };
        assert_eq!(r.identification, Some("p3"));
        let info = r.document.expect("referenced document");
        assert_eq!(info.identification, Some("DOC-7"));
        assert_eq!(info.revision, Some("B"));
        assert_eq!(info.electronic_format, Some("application/pdf"));
        assert_eq!(info.confidentiality, Some("CONFIDENTIAL"));
        assert_eq!(info.status, Some("FINALDRAFT"));

        // Bare information association.
        let a = document_assignment(&f, 1).unwrap();
        assert_eq!(a.name(), Some("Fire strategy"));
        // A non-document target is None.
        assert!(document_assignment(&f, 10).is_none());
    }
}
