//! Topic selection for the commands that read sensor streams (`odom`, `slam`, `export-glim`).

use driftless_io::bag::{BagError, BagReader};
use std::collections::BTreeSet;
use std::io::{Read, Seek};
use std::path::Path;

/// The topic of type `msg_type` to read from this bag.
///
/// With `wanted`, that topic if the bag has it with this type. Without, the only topic of
/// this type; several are an error naming them and the option (`option`) to choose one.
/// `Ok(None)`: the bag has no matching topic.
pub fn pick_topic<R: Read + Seek>(
    reader: &BagReader<R>,
    msg_type: &str,
    wanted: Option<&str>,
    option: &str,
) -> Result<Option<String>, String> {
    let mut topics: Vec<&str> = reader
        .connections()
        .iter()
        .filter(|c| c.msg_type == msg_type)
        .map(|c| c.topic.as_str())
        .collect();
    topics.sort_unstable();
    topics.dedup();
    match (wanted, topics.as_slice()) {
        (Some(w), _) => Ok(topics.contains(&w).then(|| w.to_string())),
        (None, []) => Ok(None),
        (None, [one]) => Ok(Some((*one).to_string())),
        (None, many) => Err(format!(
            "mehrere {msg_type}-Topics ({}): mit {option} eins wählen",
            many.join(", ")
        )),
    }
}

/// Topic choice across all bags of a project. With an explicit topic name that no bag has,
/// [`TopicPicker::finish`] fails and names the topics of that type that do exist.
pub struct TopicPicker<'a> {
    msg_type: &'a str,
    wanted: Option<&'a str>,
    option: &'a str,
    found: bool,
    available: BTreeSet<String>,
}

impl<'a> TopicPicker<'a> {
    pub fn new(msg_type: &'a str, wanted: Option<&'a str>, option: &'a str) -> Self {
        TopicPicker {
            msg_type,
            wanted,
            option,
            found: false,
            available: BTreeSet::new(),
        }
    }

    /// The topic to read from this bag (see [`pick_topic`]).
    pub fn pick<R: Read + Seek>(
        &mut self,
        reader: &BagReader<R>,
    ) -> Result<Option<String>, String> {
        self.available.extend(
            reader
                .connections()
                .iter()
                .filter(|c| c.msg_type == self.msg_type)
                .map(|c| c.topic.clone()),
        );
        let topic = pick_topic(reader, self.msg_type, self.wanted, self.option)?;
        self.found |= topic.is_some();
        Ok(topic)
    }

    /// Call after all bags: an explicitly requested topic must exist in at least one bag.
    pub fn finish(self) -> Result<(), String> {
        match self.wanted {
            Some(w) if !self.found => {
                let have: Vec<&str> = self.available.iter().map(String::as_str).collect();
                Err(if have.is_empty() {
                    format!(
                        "{}: Topic {w} nicht gefunden, die Bags haben kein {}-Topic",
                        self.option, self.msg_type
                    )
                } else {
                    format!(
                        "{}: Topic {w} nicht gefunden; vorhandene {}-Topics: {}",
                        self.option,
                        self.msg_type,
                        have.join(", ")
                    )
                })
            }
            _ => Ok(()),
        }
    }
}

/// A read error ends the bag: a truncated copy only with a warning (everything before the
/// cut is kept), anything else (unsupported compression, malformed or unreadable data) fails.
pub fn bag_read_error(bag: &Path, e: BagError) -> Result<(), String> {
    let name = file_name(bag);
    match e {
        BagError::Truncated { .. } => {
            eprintln!("Warnung: {name}: {e}");
            Ok(())
        }
        e => Err(format!("{name}: {e}")),
    }
}

/// File name for messages (the full path is noise and may be personal).
pub fn file_name(p: &Path) -> String {
    p.file_name().map_or_else(
        || p.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    )
}
