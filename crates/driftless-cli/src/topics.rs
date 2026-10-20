//! Topic selection for the commands that read sensor streams (`odom`, `export-glim`).

use driftless_io::bag::BagReader;
use std::io::{Read, Seek};

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
