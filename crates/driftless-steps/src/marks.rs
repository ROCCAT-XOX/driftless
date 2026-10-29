//! `stellen.toml`: the marks („Stellen“) painted by hand on the surfaces of a map or model,
//! each with its touch-up steps, in the folder of the map (`karte-NNN`, or the run folder for
//! a map the command line wrote there) or model (`modelle/modell-NNN`). All surfaces of the
//! map or model share them: a new surface of it takes them over, a new map or model starts
//! without.
//!
//! ```toml
//! next_nr = 2
//! next_seq = 4
//!
//! [[stelle]]
//! nr = 1
//!
//! [[stelle.tupfer]]
//! centre = [2.5, 0.75, 1.4]
//! normal = [1.0, 0.0, 0.0]
//! radius = 0.2
//! erase = false
//! seq = 1
//!
//! [[stelle.tupfer]]
//! centre = [2.5, 0.8, 1.4]
//! normal = [1.0, 0.0, 0.0]
//! radius = 0.2
//! erase = false
//! seq = 1
//!
//! [[stelle.schritt]]
//! kind = "glaetten"
//! level = "eben"
//! seq = 2
//!
//! [[stelle.schritt]]
//! kind = "bilder_neu"
//! variant = 1
//! seq = 3
//! ```
//!
//! A mark is the dabs of its brush strokes, centres in metres in the frame of the map or
//! model (absolute, not relative to the view), and its steps in the order they were done.
//! `seq` orders the strokes and steps of the whole file as they were made: all dabs of one
//! stroke (press to release) share one, and [`Marks::undo`] takes back the highest, a whole
//! stroke or one step. `next_nr` and `next_seq` go on counting after an undo or a removal, so
//! no number and no `seq` is used twice, also after a restart.
//!
//! „Löschen“ ([`Marks::remove`]) is a change like a stroke: the mark goes from `stelle` to
//! `geloescht` with the `seq` of its removal, as it was, and [`Marks::undo`] brings it back in
//! its place with its dabs and steps while that removal is the youngest change:
//!
//! ```toml
//! [[geloescht]]
//! seq = 5
//!
//! [geloescht.stelle]
//! nr = 2
//!
//! [[geloescht.stelle.tupfer]]
//! # …
//! ```
//!
//! A file without `geloescht` (as written before it existed) reads as before.
//!
//! A key the program does not know in a mark (a note added by hand) is read and written back
//! as it is, also in a removed one. Any other unknown key (beside `stelle` and `geloescht`, in
//! a removal, a dab or a step), a missing key or a value of the wrong type makes the file
//! unreadable instead of being dropped silently.

use crate::map::hex;
use crate::StepError;
use driftless_io::lock::FileLock;
use driftless_traj::nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::fs;
use std::io;
use std::path::Path;

/// File of the marks, in the folder of the map or model.
pub const MARKS_FILE: &str = "stellen.toml";

/// Comment at the top of the file.
const HEADER: &str = "# Driftless-Stellen: angemalte Bereiche der Oberflächen dieser Karte bzw. \
dieses Modells und ihre Nacharbeit.\n# Tupfer in Metern im Rahmen der Karte bzw. des Modells; \
Rückgängig nimmt den Zug oder Schritt mit der höchsten seq.\n\n";

/// |z| of the mean normal from which a mark lies on the floor or the ceiling (within about 46°
/// of level).
const LEVEL_Z: f64 = 0.7;
/// |z| of the mean normal up to which a mark lies on a wall (within about 17° of upright).
const UPRIGHT_Z: f64 = 0.3;

/// One dab of the brush: the faces of the surface it touches go into its mark, or out of it.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Dab {
    /// Centre on the surface, in metres in the frame of the map or model.
    pub centre: [f64; 3],
    /// Normal of the surface there (unit length).
    pub normal: [f32; 3],
    /// Radius of the brush (m).
    pub radius: f32,
    /// Takes faces out of the mark instead of adding them („radieren“).
    pub erase: bool,
    /// The stroke: all dabs of one share it.
    pub seq: u64,
}

/// A [`Dab`] before it is painted: [`Marks::paint`] gives all dabs of a stroke one `seq`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DabInput {
    pub centre: [f64; 3],
    pub normal: [f32; 3],
    pub radius: f32,
    pub erase: bool,
}

/// How strongly „Glätten“ smooths.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub enum SmoothLevel {
    /// „leicht“.
    #[serde(rename = "leicht")]
    Light,
    /// „stark“.
    #[serde(rename = "stark")]
    Strong,
    /// „ganz eben“: onto a plane, for walls, floors and ceilings.
    #[serde(rename = "eben")]
    Flat,
}

/// A touch-up step of a mark.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
#[serde(tag = "kind", deny_unknown_fields)]
pub enum EditStep {
    /// „Glätten“.
    #[serde(rename = "glaetten")]
    Smooth { level: SmoothLevel, seq: u64 },
    /// „Bilder neu einspielen“ with the photos of `variant` ([`Mark::next_variant`]).
    #[serde(rename = "bilder_neu")]
    Retexture { variant: u32, seq: u64 },
}

impl EditStep {
    /// Its place in the order of the strokes and steps of the file.
    pub fn seq(&self) -> u64 {
        match *self {
            EditStep::Smooth { seq, .. } | EditStep::Retexture { seq, .. } => seq,
        }
    }
}

/// An [`EditStep`] before it is added: [`Marks::add_step`] gives it its `seq`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepInput {
    Smooth { level: SmoothLevel },
    Retexture { variant: u32 },
}

impl StepInput {
    fn with_seq(self, seq: u64) -> EditStep {
        match self {
            StepInput::Smooth { level } => EditStep::Smooth { level, seq },
            StepInput::Retexture { variant } => EditStep::Retexture { variant, seq },
        }
    }
}

/// Where a mark lies ([`Mark::placement`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Placement {
    Wall,
    Floor,
    Ceiling,
    /// Neither: sloping, or without a direction.
    Surface,
}

impl Placement {
    /// The name in the label of a mark.
    pub fn label(self) -> &'static str {
        match self {
            Placement::Wall => "Wand",
            Placement::Floor => "Boden",
            Placement::Ceiling => "Decke",
            Placement::Surface => "Fläche",
        }
    }
}

/// An area painted by hand on the surfaces of a map or model, with its touch-up.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct Mark {
    /// Its number, from 1, in the order the marks were begun.
    pub nr: u32,
    /// The dabs of its strokes, in the order painted.
    #[serde(rename = "tupfer", default, skip_serializing_if = "Vec::is_empty")]
    pub dabs: Vec<Dab>,
    /// Its steps, in the order done.
    #[serde(rename = "schritt", default, skip_serializing_if = "Vec::is_empty")]
    pub steps: Vec<EditStep>,
    /// Keys the program does not know (a note added by hand), kept so that saving the file
    /// does not drop them.
    #[serde(flatten)]
    pub unknown: toml::Table,
}

impl Mark {
    fn new(nr: u32) -> Mark {
        Mark {
            nr,
            dabs: Vec::new(),
            steps: Vec::new(),
            unknown: toml::Table::new(),
        }
    }

    /// Where it lies, from the mean normal of its painting dabs (z up): on the floor or the
    /// ceiling from |z| 0.7, on a wall up to |z| 0.3, else on a sloping surface, as also
    /// without a painting dab or with normals that cancel out.
    pub fn placement(&self) -> Placement {
        let sum: Vector3<f64> = self
            .dabs
            .iter()
            .filter(|d| !d.erase)
            .map(|d| Vector3::from(d.normal.map(f64::from)))
            .sum();
        // without a direction NaN, which none of the limits takes
        let z = sum.try_normalize(0.0).map_or(f64::NAN, |n| n.z);
        if z >= LEVEL_Z {
            Placement::Floor
        } else if z <= -LEVEL_Z {
            Placement::Ceiling
        } else if z.abs() <= UPRIGHT_Z {
            Placement::Wall
        } else {
            Placement::Surface
        }
    }

    /// „Stelle 1 · Wand“.
    pub fn label(&self) -> String {
        format!("Stelle {} · {}", self.nr, self.placement().label())
    }

    /// The variant for the next „Bilder neu einspielen“: 0 (the best photos) for the first,
    /// one more for each further one. A smoothing bakes the images of the mark anew with
    /// variant 0 itself, so after it the count goes on from 1.
    pub fn next_variant(&self) -> u32 {
        self.steps.iter().fold(0, |n, s| match s {
            EditStep::Smooth { .. } => 1,
            EditStep::Retexture { .. } => n + 1,
        })
    }
}

/// What [`Marks::undo`] took back.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Undone {
    /// A whole stroke of mark `mark`, with its `dabs` dabs.
    Stroke { mark: u32, dabs: usize },
    /// A step of mark `mark`.
    Step { mark: u32, step: EditStep },
    /// The removal of mark `mark` („Löschen“): it is back, with its dabs and steps.
    Removal { mark: u32 },
}

/// A mark taken out with „Löschen“ ([`Marks::remove`]), kept as it was so that
/// [`Marks::undo`] can bring it back.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(deny_unknown_fields)]
struct Removed {
    /// The removal, in the order of the strokes and steps of the file.
    seq: u64,
    /// The mark as it was, with its dabs, steps and the keys the program does not know.
    stelle: Mark,
}

/// Why the marks could not be read, written or changed; the message is German, for display.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MarksError {
    /// `stellen.toml` could not be read: no TOML, an unknown or missing key, a value of the
    /// wrong type, or the disk.
    Unreadable(String),
    /// It could not be written, or its lock not taken (another window or program holds it).
    Unwritable(String),
    /// There is no mark with this number (any more).
    NoMark(u32),
}

impl fmt::Display for MarksError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            MarksError::Unreadable(m) | MarksError::Unwritable(m) => f.write_str(m),
            MarksError::NoMark(nr) => write!(f, "Stelle {nr} gibt es nicht mehr"),
        }
    }
}

impl std::error::Error for MarksError {}

impl From<MarksError> for StepError {
    fn from(e: MarksError) -> Self {
        StepError::Failed(e.to_string())
    }
}

/// The marks of a map or model, in the order begun. An unknown key beside `stelle` and
/// `geloescht` is an error instead of being dropped silently.
#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct Marks {
    /// The number of the next new mark.
    #[serde(default)]
    next_nr: u32,
    /// The `seq` of the next stroke or step.
    #[serde(default)]
    next_seq: u64,
    #[serde(rename = "stelle", default, skip_serializing_if = "Vec::is_empty")]
    pub marks: Vec<Mark>,
    /// The marks taken out with „Löschen“, in the order removed: what ↶ can bring back.
    #[serde(rename = "geloescht", default, skip_serializing_if = "Vec::is_empty")]
    removed: Vec<Removed>,
}

impl Marks {
    /// The marks of the map or model in `owner_dir`; a missing file (or folder) means none
    /// yet.
    pub fn load(owner_dir: &Path) -> Result<Marks, MarksError> {
        Marks::read(&owner_dir.join(MARKS_FILE))
    }

    /// Writes them to the folder of the map or model `owner_dir`, under the lock of the file
    /// and through a file next to it, so that nobody finds half a file. To change the marks as
    /// they are on disk (another window may have saved meanwhile) use [`Marks::change`]: the
    /// lock is not taken twice.
    pub fn save(&self, owner_dir: &Path) -> Result<(), MarksError> {
        let path = owner_dir.join(MARKS_FILE);
        let _lock = lock(&path)?;
        self.write(&path)
    }

    /// Changes the marks of `owner_dir` with `change` under the lock of the file, on the marks
    /// as they are on disk now, so that what another window or program saved meanwhile is
    /// kept; saves them if `change` succeeded and changed them. Returns the marks as they are
    /// now and what `change` returned.
    pub fn change<T>(
        owner_dir: &Path,
        change: impl FnOnce(&mut Marks) -> Result<T, MarksError>,
    ) -> Result<(Marks, T), MarksError> {
        let path = owner_dir.join(MARKS_FILE);
        let _lock = lock(&path)?;
        let mut marks = Marks::read(&path)?;
        let before = marks.clone();
        let out = change(&mut marks)?;
        if marks != before {
            marks.write(&path)?;
        }
        Ok((marks, out))
    }

    /// Paints the stroke `dabs` (press to release) into mark `mark`, or into a new mark if
    /// `mark` is `None` or not there (any more); all its dabs get one new `seq`. Returns the
    /// number of the mark. A stroke without dabs changes nothing and returns the mark, or 0
    /// without one.
    pub fn paint(&mut self, mark: Option<u32>, dabs: &[DabInput]) -> u32 {
        let index = mark.and_then(|nr| self.index(nr));
        if dabs.is_empty() {
            return index.map_or(0, |i| self.marks[i].nr);
        }
        let seq = self.take_seq();
        let index = index.unwrap_or_else(|| {
            let nr = self.take_nr();
            self.marks.push(Mark::new(nr));
            self.marks.len() - 1
        });
        let mark = &mut self.marks[index];
        mark.dabs.extend(dabs.iter().map(|d| Dab {
            centre: d.centre,
            normal: d.normal,
            radius: d.radius,
            erase: d.erase,
            seq,
        }));
        mark.nr
    }

    /// Adds `step` with a new `seq` to mark `mark`.
    pub fn add_step(&mut self, mark: u32, step: StepInput) -> Result<(), MarksError> {
        let index = self.index(mark).ok_or(MarksError::NoMark(mark))?;
        let seq = self.take_seq();
        self.marks[index].steps.push(step.with_seq(seq));
        Ok(())
    }

    /// Takes back the youngest stroke, step or removal of the file, the one with the highest
    /// `seq`: all dabs of the stroke, or the step; a removed mark comes back in its place, as
    /// it was. A mark left without dabs and steps goes as well. `None` if there is nothing to
    /// take back.
    pub fn undo(&mut self) -> Option<Undone> {
        let seq = self.seqs().max()?;
        if let Some(i) = self.removed.iter().rposition(|r| r.seq == seq) {
            let mark = self.removed.remove(i).stelle;
            let nr = mark.nr;
            // the marks are in the order begun, which is the order of their numbers
            let at = self
                .marks
                .iter()
                .position(|m| m.nr > nr)
                .unwrap_or(self.marks.len());
            self.marks.insert(at, mark);
            return Some(Undone::Removal { mark: nr });
        }
        // a step before a stroke of the same `seq`, which only a hand edit makes
        let step = self.marks.iter().enumerate().find_map(|(i, m)| {
            let j = m.steps.iter().rposition(|s| s.seq() == seq)?;
            Some((i, j))
        });
        let (index, undone) = if let Some((i, j)) = step {
            let nr = self.marks[i].nr;
            let step = self.marks[i].steps.remove(j);
            (i, Undone::Step { mark: nr, step })
        } else {
            let of_stroke = |m: &Mark| m.dabs.iter().any(|d| d.seq == seq);
            let i = self.marks.iter().position(of_stroke)?;
            let mark = &mut self.marks[i];
            let before = mark.dabs.len();
            mark.dabs.retain(|d| d.seq != seq);
            let (nr, dabs) = (mark.nr, before - mark.dabs.len());
            (i, Undone::Stroke { mark: nr, dabs })
        };
        let mark = &self.marks[index];
        if mark.dabs.is_empty() && mark.steps.is_empty() {
            self.marks.remove(index);
        }
        Some(undone)
    }

    /// Removes mark `mark` with its dabs and steps („Löschen“); whether it was there. The
    /// removal is a change with a new `seq`: [`Marks::undo`] brings the mark back while it is
    /// the youngest.
    pub fn remove(&mut self, mark: u32) -> bool {
        let Some(index) = self.index(mark) else {
            return false;
        };
        // above everything of the mark itself, so that it comes back before its strokes go
        let seq = self.take_seq();
        let stelle = self.marks.remove(index);
        self.removed.push(Removed { seq, stelle });
        true
    }

    /// Takes the step `seq` out of mark `mark` as if it had never been done – a step that
    /// changed nothing (refused by the touch-up) –, not into the history: [`Marks::undo`] does
    /// not bring it back. Whether it was there.
    pub fn drop_step(&mut self, mark: u32, seq: u64) -> bool {
        let Some(index) = self.index(mark) else {
            return false;
        };
        let steps = &mut self.marks[index].steps;
        let before = steps.len();
        steps.retain(|s| s.seq() != seq);
        steps.len() != before
    }

    /// Whether there are no marks and nothing [`Marks::undo`] could bring back.
    pub fn is_empty(&self) -> bool {
        self.marks.is_empty() && self.removed.is_empty()
    }

    /// Whether [`Marks::undo`] has something to take back: a stroke, a step or a removal.
    pub fn can_undo(&self) -> bool {
        self.seqs().next().is_some()
    }

    /// The key of a cache made from the marks, hashed like the input ids of the steps
    /// ([`hex`]): the marks in their order with their numbers, dabs and steps, numbers
    /// written exactly (`{:?}`). The counters, unknown keys and removed marks are no part of
    /// it, so the key stays the same through saving and loading and with a note added by hand,
    /// and an undone change (also a removal) brings back the key from before it.
    pub fn fingerprint(&self) -> String {
        let mut id = String::new();
        for m in &self.marks {
            id += &format!("stelle {}\n", m.nr);
            for d in &m.dabs {
                id += &format!("{d:?}\n");
            }
            for s in &m.steps {
                id += &format!("{s:?}\n");
            }
        }
        hex(&id)
    }

    /// The index of mark `nr`.
    fn index(&self, nr: u32) -> Option<usize> {
        self.marks.iter().position(|m| m.nr == nr)
    }

    /// The `seq` of all dabs, steps and removals (not of what the removed marks hold: they
    /// come back as a whole).
    fn seqs(&self) -> impl Iterator<Item = u64> + '_ {
        let marks = self.marks.iter().flat_map(|m| {
            let dabs = m.dabs.iter().map(|d| d.seq);
            dabs.chain(m.steps.iter().map(EditStep::seq))
        });
        marks.chain(self.removed.iter().map(|r| r.seq))
    }

    /// The `seq` of a new stroke, step or removal: above all in the file, also one added by
    /// hand.
    fn take_seq(&mut self) -> u64 {
        let above = self.seqs().max().map_or(1, |s| s.saturating_add(1));
        let seq = self.next_seq.max(above);
        self.next_seq = seq.saturating_add(1);
        seq
    }

    /// The number of a new mark: above all in the file, also one added by hand or removed
    /// (it may come back).
    fn take_nr(&mut self) -> u32 {
        let removed = self.removed.iter().map(|r| r.stelle.nr);
        let top = self.marks.iter().map(|m| m.nr).chain(removed).max();
        let nr = self.next_nr.max(top.map_or(1, |n| n.saturating_add(1)));
        self.next_nr = nr.saturating_add(1);
        nr
    }

    /// Reads `path`; a missing file means no marks.
    fn read(path: &Path) -> Result<Marks, MarksError> {
        let fail = |e: &dyn fmt::Display| {
            MarksError::Unreadable(format!("{} nicht lesbar: {e}", path.display()))
        };
        let text = match fs::read_to_string(path) {
            Ok(t) => t,
            Err(e) if e.kind() == io::ErrorKind::NotFound => return Ok(Marks::default()),
            Err(e) => return Err(fail(&e)),
        };
        toml::from_str(&text).map_err(|e| fail(&e))
    }

    /// Writes `path` through a file next to it, so an interrupted write never leaves half a
    /// file behind. The caller holds the lock.
    fn write(&self, path: &Path) -> Result<(), MarksError> {
        let fail = |e: &dyn fmt::Display| {
            MarksError::Unwritable(format!("{} nicht schreibbar: {e}", path.display()))
        };
        let body = toml::to_string(self).map_err(|e| fail(&e))?;
        driftless_io::atomic_write(path, format!("{HEADER}{body}").as_bytes()).map_err(|e| fail(&e))
    }
}

/// The lock of `path` while it is read, changed and written: another window or program may
/// change the marks at the same time ([`FileLock`]).
fn lock(path: &Path) -> Result<FileLock, MarksError> {
    FileLock::acquire(path).map_err(|e| MarksError::Unwritable(e.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;

    const WALL: [f32; 3] = [1.0, 0.0, 0.0];

    fn dab(normal: [f32; 3], erase: bool) -> DabInput {
        DabInput {
            centre: [1.0, 2.0, 0.5],
            normal,
            radius: 0.2,
            erase,
        }
    }

    #[test]
    fn numbers_and_seqs_are_not_used_twice() {
        let mut marks = Marks::default();
        assert_eq!(marks.paint(None, &[dab(WALL, false)]), 1);
        assert_eq!(marks.undo(), Some(Undone::Stroke { mark: 1, dabs: 1 }));
        assert!(marks.marks.is_empty());
        // after an undo and a removal (a change with a seq of its own) the next mark and
        // stroke count on
        assert_eq!(marks.paint(None, &[dab(WALL, false)]), 2);
        assert!(marks.remove(2));
        assert_eq!(marks.paint(None, &[dab(WALL, false)]), 3);
        assert_eq!(marks.marks[0].dabs[0].seq, 4);
        assert_eq!((marks.next_nr, marks.next_seq), (4, 5));
        let back: Marks = toml::from_str(&toml::to_string(&marks).unwrap()).unwrap();
        assert_eq!((back.next_nr, back.next_seq), (4, 5));
    }

    #[test]
    fn counters_follow_entries_written_by_hand() {
        let entry = "[[stelle]]\nnr = 4\n\n[[stelle.tupfer]]\ncentre = [0.0, 0.0, 0.0]\n\
                     normal = [1.0, 0.0, 0.0]\nradius = 0.2\nerase = false\nseq = 7\n";
        // without counters, and with counters behind the entries
        for counters in ["", "next_nr = 2\nnext_seq = 3\n"] {
            let mut marks: Marks = toml::from_str(&format!("{counters}{entry}")).unwrap();
            assert_eq!(marks.paint(None, &[dab(WALL, false)]), 5);
            assert_eq!(marks.marks[1].dabs[0].seq, 8);
            assert_eq!(marks.undo(), Some(Undone::Stroke { mark: 5, dabs: 1 }));
            assert_eq!(marks.undo(), Some(Undone::Stroke { mark: 4, dabs: 1 }));
            assert_eq!(marks.undo(), None);
        }
    }

    #[test]
    fn paint_into_a_mark_that_is_gone_begins_a_new_one() {
        let mut marks = Marks::default();
        assert_eq!(marks.paint(Some(7), &[dab(WALL, false)]), 1);
        assert_eq!(marks.paint(Some(1), &[dab(WALL, true)]), 1);
        assert_eq!(marks.marks.len(), 1);
        assert_eq!(marks.marks[0].dabs.len(), 2);
        assert_ne!(marks.marks[0].dabs[0].seq, marks.marks[0].dabs[1].seq);
    }

    #[test]
    fn empty_stroke_changes_nothing() {
        let mut marks = Marks::default();
        assert_eq!(marks.paint(None, &[]), 0);
        assert_eq!(marks, Marks::default());
        let nr = marks.paint(None, &[dab(WALL, false)]);
        let before = marks.clone();
        assert_eq!(marks.paint(Some(nr), &[]), nr);
        assert_eq!(marks.paint(Some(nr + 1), &[]), 0);
        assert_eq!(marks, before);
    }

    #[test]
    fn step_for_a_missing_mark_is_refused() {
        let mut marks = Marks::default();
        let flat = StepInput::Smooth {
            level: SmoothLevel::Flat,
        };
        assert_eq!(marks.add_step(1, flat), Err(MarksError::NoMark(1)));
        assert_eq!(marks, Marks::default());
        let nr = marks.paint(None, &[dab(WALL, false)]);
        marks.add_step(nr, flat).unwrap();
        assert_eq!(
            marks.marks[0].steps,
            [EditStep::Smooth {
                level: SmoothLevel::Flat,
                seq: 2
            }]
        );
    }

    #[test]
    fn placement_is_the_mean_of_the_painting_normals() {
        let mut marks = Marks::default();
        // an erasing dab on the floor does not count
        let nr = marks.paint(None, &[dab(WALL, false), dab([0.0, 0.0, 1.0], true)]);
        assert_eq!(marks.marks[0].placement(), Placement::Wall);
        // two walls facing each other give no direction, nor do erasing dabs alone
        marks.paint(Some(nr), &[dab([-1.0, 0.0, 0.0], false)]);
        assert_eq!(marks.marks[0].placement(), Placement::Surface);
        let mut erasing = marks.marks[0].clone();
        erasing.dabs.retain(|d| d.erase);
        assert_eq!(erasing.placement(), Placement::Surface);
        // nor a normal that is no number
        erasing.dabs[0].erase = false;
        erasing.dabs[0].normal = [f32::NAN, 0.0, 1.0];
        assert_eq!(erasing.placement(), Placement::Surface);
        assert_eq!(erasing.label(), "Stelle 1 · Fläche");
    }

    #[test]
    fn levels_and_steps_in_the_file() {
        for (level, name) in [
            (SmoothLevel::Light, "leicht"),
            (SmoothLevel::Strong, "stark"),
            (SmoothLevel::Flat, "eben"),
        ] {
            assert_eq!(toml::Value::try_from(level).unwrap().as_str(), Some(name));
        }
        let mut marks = Marks::default();
        let nr = marks.paint(None, &[dab(WALL, true)]);
        let light = StepInput::Smooth {
            level: SmoothLevel::Light,
        };
        marks.add_step(nr, light).unwrap();
        marks
            .add_step(nr, StepInput::Retexture { variant: 1 })
            .unwrap();
        assert_eq!(
            toml::to_string(&marks).unwrap(),
            "next_nr = 2\nnext_seq = 4\n\n[[stelle]]\nnr = 1\n\n[[stelle.tupfer]]\n\
             centre = [1.0, 2.0, 0.5]\nnormal = [1.0, 0.0, 0.0]\nradius = 0.2\nerase = true\n\
             seq = 1\n\n[[stelle.schritt]]\nkind = \"glaetten\"\nlevel = \"leicht\"\nseq = 2\n\n\
             [[stelle.schritt]]\nkind = \"bilder_neu\"\nvariant = 1\nseq = 3\n"
        );
    }

    #[test]
    fn fingerprint_is_the_same_everywhere() {
        // FNV-1a of nothing, as for the input ids
        assert_eq!(Marks::default().fingerprint(), "cbf29ce484222325");
        // the key of a cache made on another computer or by another build
        let mut marks = Marks::default();
        let floor = DabInput {
            centre: [1.5, -2.25, 0.125],
            normal: [0.0, 0.6, 0.8],
            radius: 0.2,
            erase: false,
        };
        let nr = marks.paint(None, &[floor]);
        let flat = StepInput::Smooth {
            level: SmoothLevel::Flat,
        };
        marks.add_step(nr, flat).unwrap();
        marks
            .add_step(nr, StepInput::Retexture { variant: 1 })
            .unwrap();
        assert_eq!(marks.fingerprint(), "283f4379c1c83d2f");
    }

    #[test]
    fn undo_brings_a_removed_mark_back() {
        let mut marks = Marks::default();
        let wall = marks.paint(None, &[dab(WALL, false), dab(WALL, true)]);
        let strong = StepInput::Smooth {
            level: SmoothLevel::Strong,
        };
        marks.add_step(wall, strong).unwrap();
        let floor = marks.paint(None, &[dab([0.0, 0.0, 1.0], false)]);
        marks.marks[0]
            .unknown
            .insert("notiz".into(), "Fenster".into());
        let before = marks.clone();
        let key = marks.fingerprint();

        // „Löschen“ takes it out with its dabs and its step, and is the youngest change
        assert!(marks.remove(wall));
        assert!(!marks.remove(wall));
        assert_eq!(marks.marks.len(), 1);
        assert_eq!(marks.marks[0].nr, floor);
        assert_ne!(marks.fingerprint(), key);
        assert!(!marks.is_empty() && marks.can_undo());
        assert_eq!(marks.add_step(wall, strong), Err(MarksError::NoMark(wall)));

        // it lasts through saving and loading, with what was added by hand
        let text = toml::to_string(&marks).unwrap();
        assert!(text.contains("[[geloescht]]"), "{text}");
        assert!(text.contains("notiz = \"Fenster\""), "{text}");
        let mut marks: Marks = toml::from_str(&text).unwrap();

        // ↶ brings it back in its place, as it was: the key from before as well
        assert_eq!(marks.undo(), Some(Undone::Removal { mark: wall }));
        assert_eq!(marks.marks, before.marks);
        assert_eq!(marks.fingerprint(), key);
        // then the stroke on the floor, the step, the strokes on the wall
        assert_eq!(
            marks.undo(),
            Some(Undone::Stroke {
                mark: floor,
                dabs: 1
            })
        );
        assert_eq!(
            marks.undo(),
            Some(Undone::Step {
                mark: wall,
                step: EditStep::Smooth {
                    level: SmoothLevel::Strong,
                    seq: 2
                }
            })
        );
        assert_eq!(
            marks.undo(),
            Some(Undone::Stroke {
                mark: wall,
                dabs: 2
            })
        );
        assert_eq!(marks.undo(), None);
        assert!(marks.is_empty() && !marks.can_undo());
    }

    #[test]
    fn removed_marks_keep_their_numbers() {
        // also in a file without counters: a new Stelle never gets the number of one ↶ can
        // bring back, nor a seq below its removal
        let mut marks = Marks::default();
        let a = marks.paint(None, &[dab(WALL, false)]);
        let b = marks.paint(None, &[dab(WALL, false)]);
        assert!(marks.remove(b));
        assert!(marks.remove(a));
        let mut text = toml::to_string(&marks).unwrap();
        assert!(text.starts_with("next_nr = 3\nnext_seq = 5\n"), "{text}");
        text = text.replace("next_nr = 3\nnext_seq = 5\n", "");
        let mut marks: Marks = toml::from_str(&text).unwrap();
        assert!(marks.marks.is_empty() && !marks.is_empty());
        let c = marks.paint(None, &[dab(WALL, false)]);
        assert_eq!(c, 3);
        assert_eq!(marks.marks[0].dabs[0].seq, 5);
        assert_eq!(marks.undo(), Some(Undone::Stroke { mark: c, dabs: 1 }));
        assert_eq!(marks.undo(), Some(Undone::Removal { mark: a }));
        assert_eq!(marks.undo(), Some(Undone::Removal { mark: b }));
        assert_eq!(marks.marks.iter().map(|m| m.nr).collect::<Vec<_>>(), [a, b]);
    }

    #[test]
    fn a_refused_step_goes_without_a_trace() {
        let mut marks = Marks::default();
        let nr = marks.paint(None, &[dab(WALL, false)]);
        let flat = StepInput::Smooth {
            level: SmoothLevel::Flat,
        };
        marks.add_step(nr, flat).unwrap();
        let seq = marks.marks[0].steps[0].seq();
        assert!(!marks.drop_step(nr, seq + 1));
        assert!(!marks.drop_step(nr + 1, seq));
        assert!(marks.drop_step(nr, seq));
        assert!(marks.marks[0].steps.is_empty());
        // not into the history: ↶ takes back the stroke, nothing brings the step back
        assert_eq!(marks.undo(), Some(Undone::Stroke { mark: nr, dabs: 1 }));
        assert_eq!(marks.undo(), None);
        // the next seq is not that of the step again
        let again = marks.paint(None, &[dab(WALL, false)]);
        assert_eq!(marks.marks[0].nr, again);
        assert!(marks.marks[0].dabs[0].seq > seq);
    }

    #[test]
    fn errors_are_german_and_step_errors() {
        let gone = MarksError::NoMark(3);
        assert_eq!(gone.to_string(), "Stelle 3 gibt es nicht mehr");
        assert_eq!(
            StepError::from(gone),
            StepError::Failed("Stelle 3 gibt es nicht mehr".into())
        );
        let dir = driftless_io::testscan::tmp("steps-marks-locked");
        // a lock another window holds: saving waits, then gives up and writes nothing
        let held = FileLock::acquire(&dir.join(MARKS_FILE)).unwrap();
        let mut marks = Marks::default();
        marks.paint(None, &[dab(WALL, false)]);
        let err = marks.save(&dir).unwrap_err();
        assert!(
            matches!(&err, MarksError::Unwritable(m) if m.contains("anderen Fenster")),
            "{err}"
        );
        assert!(!dir.join(MARKS_FILE).exists());
        drop(held);
        marks.save(&dir).unwrap();
        assert_eq!(Marks::load(&dir).unwrap(), marks);
    }
}
