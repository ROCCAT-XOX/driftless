//! `objekte.toml`: the objects of a map or model – sockets, connections, cables and pipes set
//! by hand, and the measurements kept –, in the folder of the map (`karte-NNN`, or the run
//! folder for a map the command line wrote there) or model (`modelle/modell-NNN`). Built like
//! `stellen.toml` ([`crate::marks`]). They belong to the map or model and not to a floor plan,
//! so a plan computed anew keeps them; the file names no paths and moves with the project.
//!
//! ```toml
//! next_nr = 3
//! next_seq = 4
//!
//! [[objekt]]
//! nr = 1
//! art = "anschluss"
//! punkte = [[2.5, 0.75, 0.3]]
//! name = "Küche"
//! notiz = "hinter dem Schrank"
//! anschluss = "wasser"
//! quelle = "hand"
//! seq = 3
//!
//! [objekt.eigenschaften]
//! hersteller = "Gira"
//!
//! [[objekt]]
//! nr = 2
//! art = "rohr"
//! punkte = [[0.0, 0.0, 2.4], [3.0, 0.0, 2.4]]
//! durchmesser_mm = 50.0
//! quelle = "hand"
//! seq = 2
//! ```
//!
//! Points are in metres in the frame of the map or model (absolute, not relative to the view).
//! `seq` orders the changes of the whole file: an object carries the one of its latest change,
//! and [`Objects::undo`] takes back the highest. `next_nr` and `next_seq` go on counting after
//! an undo or a removal, so no number and no `seq` is used twice, also after a restart.
//!
//! What ↶ can bring back is kept under `geloescht`, with the `seq` of the change, the object as
//! it was before it: removed („Löschen“, then it is in `objekt` no more) or changed (renamed, a
//! new note, then it still is). A removal of all objects of a kind is one change, all of them
//! with one `seq`:
//!
//! ```toml
//! [[geloescht]]
//! seq = 5
//!
//! [geloescht.objekt]
//! nr = 2
//! art = "rohr"
//! # …
//! ```
//!
//! A key the program does not know in an object (added by hand) is read and written back as it
//! is, also in one under `geloescht`. Any other unknown key, a missing `nr`, `art` or `punkte`
//! or a value of the wrong type makes the file unreadable instead of being dropped silently, as
//! does an object against the rules of [`Objects::add`] or a number used twice: everything that
//! reads the file can rely on them. An object written by hand needs no more than those three
//! keys. Without `quelle` it counts as `hand`; without `seq` (0) no ↶ takes it away.

use crate::StepError;
use driftless_io::lock::FileLock;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;
use std::fs;
use std::io;
use std::mem;
use std::path::Path;

/// File of the objects, in the folder of the map or model.
pub const OBJECTS_FILE: &str = "objekte.toml";

/// [`Object::source`] of an object set by hand.
pub const SOURCE_HAND: &str = "hand";

/// Comment at the top of the file.
const HEADER: &str = "# Driftless-Objekte: Steckdosen, Anschlüsse, Kabel, Rohre und Messungen \
dieser Karte bzw. dieses Modells.\n# Punkte in Metern im Rahmen der Karte bzw. des Modells; \
Rückgängig nimmt die Änderung mit der höchsten seq.\n\n";

/// What an object is.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum ObjectKind {
    /// A socket: one point.
    #[serde(rename = "steckdose")]
    Socket,
    /// A connection of the kind [`Object::connection`]: one point.
    #[serde(rename = "anschluss")]
    Connection,
    /// A cable: a line of at least two points.
    #[serde(rename = "kabel")]
    Cable,
    /// A pipe of [`Object::diameter_mm`]: a line of at least two points.
    #[serde(rename = "rohr")]
    Pipe,
    /// A measurement between two points.
    #[serde(rename = "messung")]
    Measure,
}

impl ObjectKind {
    /// The name in the label of an object.
    pub fn label(self) -> &'static str {
        match self {
            ObjectKind::Socket => "Steckdose",
            ObjectKind::Connection => "Anschluss",
            ObjectKind::Cable => "Kabel",
            ObjectKind::Pipe => "Rohr",
            ObjectKind::Measure => "Messung",
        }
    }
}

/// What a connection supplies.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConnectionKind {
    #[serde(rename = "strom")]
    Power,
    #[serde(rename = "wasser")]
    Water,
    #[serde(rename = "abwasser")]
    Drain,
    #[serde(rename = "heizung")]
    Heating,
    #[serde(rename = "gas")]
    Gas,
    #[serde(rename = "netzwerk")]
    Network,
    #[serde(rename = "sonstiges")]
    Other,
}

/// An [`Object`] before it is added: [`Objects::add`] gives it its number and `seq`.
#[derive(Clone, Debug, PartialEq)]
pub struct ObjectInput {
    pub kind: ObjectKind,
    pub points: Vec<[f64; 3]>,
    pub name: String,
    pub note: String,
    /// For a [`ObjectKind::Connection`].
    pub connection: Option<ConnectionKind>,
    /// For a [`ObjectKind::Pipe`].
    pub diameter_mm: Option<f32>,
    pub properties: BTreeMap<String, String>,
}

/// An object of a map or model.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Object {
    /// Its number, from 1, in the order the objects were added.
    pub nr: u32,
    #[serde(rename = "art")]
    pub kind: ObjectKind,
    /// In metres in the frame of the map or model, in the order set.
    #[serde(rename = "punkte")]
    pub points: Vec<[f64; 3]>,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub name: String,
    #[serde(rename = "notiz", default, skip_serializing_if = "String::is_empty")]
    pub note: String,
    #[serde(rename = "anschluss", default, skip_serializing_if = "Option::is_none")]
    pub connection: Option<ConnectionKind>,
    #[serde(
        default,
        skip_serializing_if = "Option::is_none",
        rename = "durchmesser_mm"
    )]
    pub diameter_mm: Option<f32>,
    /// Where it comes from: [`SOURCE_HAND`], so that a later automatic detection can write the
    /// same objects and tell its own.
    #[serde(rename = "quelle", default = "source_hand")]
    pub source: String,
    /// Free properties: for now set in the file only, not in the program.
    #[serde(
        rename = "eigenschaften",
        default,
        skip_serializing_if = "BTreeMap::is_empty"
    )]
    pub properties: BTreeMap<String, String>,
    /// Its latest change, in the order of the changes of the file; 0 if it was written by hand
    /// without one, then [`Objects::undo`] does not take it away.
    #[serde(default)]
    pub seq: u64,
    /// Keys the program does not know (added by hand), kept so that saving the file does not
    /// drop them.
    #[serde(flatten)]
    pub unknown: toml::Table,
}

fn source_hand() -> String {
    SOURCE_HAND.to_string()
}

impl Object {
    /// „Steckdose 3“, with a name „Steckdose 3 · Küche“.
    pub fn label(&self) -> String {
        let label = format!("{} {}", self.kind.label(), self.nr);
        match self.name.trim() {
            "" => label,
            name => format!("{label} · {name}"),
        }
    }
}

/// What [`Objects::undo`] took back.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Undone {
    /// That object `.0` was added: it is gone.
    Added(u32),
    /// That object `.0` was removed: it is back. Of a removal of a whole kind the first of the
    /// objects that came back.
    Removed(u32),
    /// A new name or note of object `.0`: it has the one from before.
    Changed(u32),
}

/// An object as it was before a change or removal, for [`Objects::undo`].
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(deny_unknown_fields)]
struct Removed {
    /// The change, in the order of the changes of the file.
    seq: u64,
    /// The object as it was before it, with the keys the program does not know.
    objekt: Object,
}

/// Why the objects could not be read, written or changed; the message is German, for display.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ObjectsError {
    /// The disk: the file could not be read or written.
    Io(String),
    /// `objekte.toml` is no TOML, has an unknown or missing key or a value of the wrong type.
    Format(String),
    /// An object against the rules of [`Objects::add`]: given to it, or found in the file.
    Invalid(String),
    /// The lock of the file could not be taken (another window or program holds it).
    Lock(String),
}

impl fmt::Display for ObjectsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ObjectsError::Io(m)
            | ObjectsError::Format(m)
            | ObjectsError::Invalid(m)
            | ObjectsError::Lock(m) => f.write_str(m),
        }
    }
}

impl std::error::Error for ObjectsError {}

impl From<ObjectsError> for StepError {
    fn from(e: ObjectsError) -> Self {
        StepError::Failed(e.to_string())
    }
}

/// The rules of an object; why it breaks one, for display.
fn check(
    kind: ObjectKind,
    points: &[[f64; 3]],
    connection: Option<ConnectionKind>,
    diameter_mm: Option<f32>,
) -> Result<(), &'static str> {
    let (fits, needs) = match kind {
        ObjectKind::Socket => (
            points.len() == 1,
            "Eine Steckdose braucht genau einen Punkt.",
        ),
        ObjectKind::Connection => (
            points.len() == 1,
            "Ein Anschluss braucht genau einen Punkt.",
        ),
        ObjectKind::Cable => (
            points.len() >= 2,
            "Ein Kabel braucht mindestens zwei Punkte.",
        ),
        ObjectKind::Pipe => (
            points.len() >= 2,
            "Ein Rohr braucht mindestens zwei Punkte.",
        ),
        ObjectKind::Measure => (points.len() == 2, "Eine Messung braucht genau zwei Punkte."),
    };
    if !fits {
        return Err(needs);
    }
    if points.iter().flatten().any(|c| !c.is_finite()) {
        return Err("Die Lage eines Punkts ist keine gültige Zahl.");
    }
    if kind == ObjectKind::Connection && connection.is_none() {
        return Err("Ein Anschluss braucht eine Sorte (Strom, Wasser, …).");
    }
    if kind == ObjectKind::Pipe && !diameter_mm.is_some_and(|d| d.is_finite() && d > 0.0) {
        return Err("Ein Rohr braucht einen Durchmesser über 0 mm.");
    }
    Ok(())
}

/// The objects of a map or model, in the order added. An unknown key beside `objekt` and
/// `geloescht` is an error instead of being dropped silently.
#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Objects {
    /// The number of the next new object.
    #[serde(default)]
    next_nr: u32,
    /// The `seq` of the next change.
    #[serde(default)]
    next_seq: u64,
    #[serde(rename = "objekt", default, skip_serializing_if = "Vec::is_empty")]
    objects: Vec<Object>,
    /// The objects as they were before a change or removal, in the order done: what ↶ can
    /// bring back.
    #[serde(rename = "geloescht", default, skip_serializing_if = "Vec::is_empty")]
    removed: Vec<Removed>,
}

impl Objects {
    /// The objects of the map or model in `owner_dir`; a missing file (or folder) means none
    /// yet.
    pub fn load(owner_dir: &Path) -> Result<Objects, ObjectsError> {
        Objects::read(&owner_dir.join(OBJECTS_FILE))
    }

    /// Writes them to the folder of the map or model `owner_dir`, under the lock of the file
    /// and through a file next to it, so that nobody finds half a file. To change the objects
    /// as they are on disk (another window may have saved meanwhile) use [`Objects::change`]:
    /// the lock is not taken twice.
    pub fn save(&self, owner_dir: &Path) -> Result<(), ObjectsError> {
        let path = owner_dir.join(OBJECTS_FILE);
        let _lock = lock(&path)?;
        self.write(&path)
    }

    /// Changes the objects of `owner_dir` with `f` under the lock of the file, on the objects
    /// as they are on disk now, so that what another window or program saved meanwhile is
    /// kept; saves them if `f` succeeded and changed them. Returns the objects as they are now
    /// and what `f` returned.
    pub fn change<T>(
        owner_dir: &Path,
        f: impl FnOnce(&mut Objects) -> Result<T, ObjectsError>,
    ) -> Result<(Objects, T), ObjectsError> {
        let path = owner_dir.join(OBJECTS_FILE);
        let _lock = lock(&path)?;
        let mut objects = Objects::read(&path)?;
        let before = objects.clone();
        let out = f(&mut objects)?;
        if objects != before {
            objects.write(&path)?;
        }
        Ok((objects, out))
    }

    /// Adds an object with a new number, from the hand ([`SOURCE_HAND`]); returns the number.
    /// A socket and a connection have exactly one point, a connection also its kind, a cable
    /// and a pipe at least two, a pipe also a diameter above 0, a measurement exactly two; all
    /// coordinates are finite. Refused otherwise ([`ObjectsError::Invalid`]), nothing changed.
    /// A kind of connection or a diameter given to another kind of object is left out: the
    /// tool keeps its settings while another kind is set.
    pub fn add(&mut self, input: ObjectInput) -> Result<u32, ObjectsError> {
        let ObjectInput {
            kind,
            points,
            name,
            note,
            connection,
            diameter_mm,
            properties,
        } = input;
        check(kind, &points, connection, diameter_mm)
            .map_err(|why| ObjectsError::Invalid(why.to_string()))?;
        let seq = self.take_seq();
        let nr = self.take_nr();
        self.objects.push(Object {
            nr,
            kind,
            points,
            name,
            note,
            connection: connection.filter(|_| kind == ObjectKind::Connection),
            diameter_mm: diameter_mm.filter(|_| kind == ObjectKind::Pipe),
            source: source_hand(),
            properties,
            seq,
            unknown: toml::Table::new(),
        });
        Ok(nr)
    }

    /// Gives object `nr` the name `name`, one change for [`Objects::undo`]; whether it is
    /// there. The name it has already is no change.
    pub fn rename(&mut self, nr: u32, name: &str) -> bool {
        self.set_text(nr, |o| &mut o.name, name)
    }

    /// Gives object `nr` the note `note`, one change for [`Objects::undo`]; whether it is
    /// there. The note it has already is no change.
    pub fn set_note(&mut self, nr: u32, note: &str) -> bool {
        self.set_text(nr, |o| &mut o.note, note)
    }

    /// Removes object `nr` („Löschen“); whether it was there. [`Objects::undo`] brings it back
    /// while the removal is the youngest change.
    pub fn remove(&mut self, nr: u32) -> bool {
        let Some(index) = self.index(nr) else {
            return false;
        };
        let seq = self.take_seq();
        let objekt = self.objects.remove(index);
        self.removed.push(Removed { seq, objekt });
        true
    }

    /// Removes all objects of `kind` as one change, so one [`Objects::undo`] brings them back;
    /// how many there were. None is no change.
    pub fn remove_kind(&mut self, kind: ObjectKind) -> usize {
        if !self.objects.iter().any(|o| o.kind == kind) {
            return 0;
        }
        let seq = self.take_seq();
        let (gone, kept): (Vec<_>, Vec<_>) = mem::take(&mut self.objects)
            .into_iter()
            .partition(|o| o.kind == kind);
        self.objects = kept;
        let count = gone.len();
        self.removed
            .extend(gone.into_iter().map(|objekt| Removed { seq, objekt }));
        count
    }

    /// Takes back the youngest change of the file, the one with the highest `seq`: a removed
    /// object comes back in its place, a changed one is as it was before, an added one goes.
    /// `None` if there is nothing to take back.
    pub fn undo(&mut self) -> Option<Undone> {
        let seq = self.seqs().max()?;
        let (back, kept): (Vec<_>, Vec<_>) = mem::take(&mut self.removed)
            .into_iter()
            .partition(|r| r.seq == seq);
        self.removed = kept;
        if back.is_empty() {
            let index = self.objects.iter().position(|o| o.seq == seq)?;
            return Some(Undone::Added(self.objects.remove(index).nr));
        }
        let mut undone = None;
        for Removed { objekt, .. } in back {
            let nr = objekt.nr;
            let this = if let Some(index) = self.index(nr) {
                self.objects[index] = objekt;
                Undone::Changed(nr)
            } else {
                // the objects are in the order added, which is the order of their numbers
                let at = self
                    .objects
                    .iter()
                    .position(|o| o.nr > nr)
                    .unwrap_or(self.objects.len());
                self.objects.insert(at, objekt);
                Undone::Removed(nr)
            };
            undone.get_or_insert(this);
        }
        undone
    }

    /// Whether [`Objects::undo`] has something to take back.
    pub fn can_undo(&self) -> bool {
        self.seqs().next().is_some()
    }

    /// The objects, in the order added.
    pub fn all(&self) -> &[Object] {
        &self.objects
    }

    /// Object `nr`, if it is there.
    pub fn get(&self, nr: u32) -> Option<&Object> {
        self.index(nr).map(|i| &self.objects[i])
    }

    /// The index of object `nr`.
    fn index(&self, nr: u32) -> Option<usize> {
        self.objects.iter().position(|o| o.nr == nr)
    }

    /// Sets a text of object `nr` as one change, unless it has that text already.
    fn set_text(&mut self, nr: u32, field: fn(&mut Object) -> &mut String, text: &str) -> bool {
        let Some(index) = self.index(nr) else {
            return false;
        };
        if field(&mut self.objects[index]) == text {
            return true;
        }
        let seq = self.take_seq();
        let objekt = self.objects[index].clone();
        self.removed.push(Removed { seq, objekt });
        let object = &mut self.objects[index];
        *field(object) = text.to_string();
        object.seq = seq;
        true
    }

    /// The `seq` of everything [`Objects::undo`] can take back: of the latest change of each
    /// object, unless it was written by hand without one, and of the changes and removals
    /// (not what the objects kept for them hold: they come back as a whole).
    fn seqs(&self) -> impl Iterator<Item = u64> + '_ {
        let objects = self.objects.iter().map(|o| o.seq).filter(|&s| s != 0);
        objects.chain(self.removed.iter().map(|r| r.seq))
    }

    /// The `seq` of a new change: above all in the file, also one added by hand.
    fn take_seq(&mut self) -> u64 {
        let above = self.seqs().max().map_or(1, |s| s.saturating_add(1));
        let seq = self.next_seq.max(above);
        self.next_seq = seq.saturating_add(1);
        seq
    }

    /// The number of a new object: above all in the file, also one added by hand or removed
    /// (it may come back).
    fn take_nr(&mut self) -> u32 {
        let removed = self.removed.iter().map(|r| r.objekt.nr);
        let top = self.objects.iter().map(|o| o.nr).chain(removed).max();
        let nr = self.next_nr.max(top.map_or(1, |n| n.saturating_add(1)));
        self.next_nr = nr.saturating_add(1);
        nr
    }

    /// Whether what was read keeps the rules: every object of the list and every one ↶ can
    /// bring back, and no number twice in the list. Why not, for display.
    fn check_read(&self) -> Result<(), String> {
        let mut numbers = BTreeSet::new();
        for o in &self.objects {
            if !numbers.insert(o.nr) {
                return Err(format!("Die Nummer {} steht zweimal in der Datei.", o.nr));
            }
        }
        let removed = self.removed.iter().map(|r| &r.objekt);
        for o in self.objects.iter().chain(removed) {
            check(o.kind, &o.points, o.connection, o.diameter_mm)
                .map_err(|why| format!("Objekt {}: {why}", o.nr))?;
        }
        Ok(())
    }

    /// Reads `path`; a missing file means no objects.
    fn read(path: &Path) -> Result<Objects, ObjectsError> {
        let unreadable = |e: &dyn fmt::Display| format!("{} nicht lesbar: {e}", path.display());
        let text = match fs::read_to_string(path) {
            Ok(t) => t,
            Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(Objects::default()),
            Err(e) => return Err(ObjectsError::Io(unreadable(&e))),
        };
        let objects: Objects =
            toml::from_str(&text).map_err(|e| ObjectsError::Format(unreadable(&e)))?;
        objects
            .check_read()
            .map_err(|why| ObjectsError::Invalid(unreadable(&why)))?;
        Ok(objects)
    }

    /// Writes `path` through a file next to it, so an interrupted write never leaves half a
    /// file behind. The caller holds the lock.
    fn write(&self, path: &Path) -> Result<(), ObjectsError> {
        let unwritable = |e: &dyn fmt::Display| format!("{} nicht schreibbar: {e}", path.display());
        let body = toml::to_string(self).map_err(|e| ObjectsError::Format(unwritable(&e)))?;
        driftless_io::atomic_write(path, format!("{HEADER}{body}").as_bytes())
            .map_err(|e| ObjectsError::Io(unwritable(&e)))
    }
}

/// The lock of `path` while it is read, changed and written: another window or program may
/// change the objects at the same time ([`FileLock`]).
fn lock(path: &Path) -> Result<FileLock, ObjectsError> {
    FileLock::acquire(path).map_err(|e| ObjectsError::Lock(e.to_string()))
}
