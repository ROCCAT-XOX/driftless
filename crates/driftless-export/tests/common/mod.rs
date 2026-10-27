//! Helpers shared by the test binaries: foreign E57 files written with the `e57` crate and
//! edited afterwards.

#![allow(dead_code)]

use e57::{
    E57Writer, ImageFormat, Record, RecordDataType, RecordName, RecordValue,
    VisualReferenceImageProperties,
};
use std::path::Path;

/// Writes a foreign file with the `e57` crate: one cloud named `name` with `prototype` and
/// `rows`, optionally with a pose and a preview image.
pub fn foreign(
    path: &Path,
    name: Option<&str>,
    prototype: Vec<Record>,
    rows: &[Vec<RecordValue>],
    pose: Option<e57::Transform>,
    image: bool,
) {
    let mut w = E57Writer::from_file(path, "fremd-datei").unwrap();
    let mut pc = w.add_pointcloud("fremd-scan", prototype).unwrap();
    pc.set_name(name.map(str::to_owned));
    pc.set_transform(pose);
    for row in rows {
        pc.add_point(row.clone()).unwrap();
    }
    pc.finalize().unwrap();
    if image {
        // A 1×1 PNG (only its bytes matter here, nothing decodes it).
        let png: &[u8] = b"\x89PNG\r\n\x1a\n";
        let mut img = w.add_image("fremd-bild").unwrap();
        img.add_visual_reference(
            ImageFormat::Png,
            &mut &png[..],
            VisualReferenceImageProperties {
                width: 1,
                height: 1,
            },
            None,
        )
        .unwrap();
        img.finalize().unwrap();
    }
    w.finalize().unwrap();
}

pub fn double(name: RecordName) -> Record {
    Record {
        name,
        data_type: RecordDataType::Double {
            min: None,
            max: None,
        },
    }
}

/// CRC-32C (Castagnoli), as on every E57 page.
fn crc32c(data: &[u8]) -> u32 {
    !data.iter().fold(!0u32, |crc, &b| {
        (0..8).fold(crc ^ u32::from(b), |c, _| {
            if c & 1 == 1 {
                (c >> 1) ^ 0x82F6_3B78
            } else {
                c >> 1
            }
        })
    })
}

/// Rewrites the XML section of the E57 file at `path` through `edit` and lays the file out
/// in checked 1024-byte pages again (the XML is the last section).
pub fn edit_xml(path: &Path, edit: impl FnOnce(&str) -> String) {
    const PAGE: usize = 1024;
    const DATA: usize = PAGE - 4;
    let bytes = std::fs::read(path).unwrap();
    assert_eq!(bytes.len() % PAGE, 0);
    let mut logical: Vec<u8> = bytes
        .chunks(PAGE)
        .flat_map(|p| &p[..DATA])
        .copied()
        .collect();
    let u64_at = |b: &[u8], at: usize| u64::from_le_bytes(b[at..at + 8].try_into().unwrap());
    let phys = u64_at(&logical, 24) as usize;
    let xml_len = u64_at(&logical, 32) as usize;
    let start = phys / PAGE * DATA + phys % PAGE;
    let xml = std::str::from_utf8(&logical[start..start + xml_len])
        .unwrap()
        .to_string();
    let new_xml = edit(&xml);
    assert_ne!(new_xml, xml, "the edit changed nothing");
    logical.truncate(start);
    logical.extend_from_slice(new_xml.as_bytes());
    logical.resize(logical.len().div_ceil(DATA) * DATA, 0);
    let pages = logical.len() / DATA;
    logical[16..24].copy_from_slice(&((pages * PAGE) as u64).to_le_bytes());
    logical[32..40].copy_from_slice(&(new_xml.len() as u64).to_le_bytes());
    let mut out = Vec::with_capacity(pages * PAGE);
    for page in logical.chunks(DATA) {
        out.extend_from_slice(page);
        out.extend_from_slice(&crc32c(page).to_be_bytes());
    }
    std::fs::write(path, out).unwrap();
}

pub fn integer(name: RecordName, min: i64, max: i64) -> Record {
    Record {
        name,
        data_type: RecordDataType::Integer { min, max },
    }
}
