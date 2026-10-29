//! The tool „Pinsel“: Stellen painted by hand on the surface of a map or model, to touch them
//! up (`stellen.toml`, [`driftless_steps::marks`]; the touch-up itself in
//! `driftless_map::surface::edit`).
//!
//! - The left button paints, with ⌥ (Windows: Alt) it erases; Shift + left and the middle
//!   button move the view, the right one turns it (in fly mode: looks around). A press paints
//!   at once, without waiting to tell a click from a drag.
//! - A stroke (press to release) lays a dab every quarter of the radius along the way the
//!   pointer went over the surface: where the ray through the pointer first meets the surface
//!   (from either side, through the height cut, as drawn), with the normal of the surface
//!   around it ([`dab_normal`]). Between two positions of the pointer the way is followed on the
//!   surface, so a quick stroke leaves no gaps.
//! - Painting without a Stelle chosen begins a new one; with one chosen it paints into it
//!   ([`BrushState::active`]). A new Stelle is chosen once painted; Esc or a click on it in the
//!   list chooses none again. Erasing without a Stelle chosen erases from the one under the
//!   pointer, else it does nothing.
//! - While painting, the view marks the Stelle as the touch-up will see it: the faces of all its
//!   dabs ([`select_faces`], which also closes small holes), selected anew every frame that
//!   brought new dabs. At the release the stroke becomes one [`Action::Paint`]: it shows in
//!   [`AppState::marks`] at once and is saved in the background, one change of `stellen.toml`
//!   under its lock ([`MarkChange`]); ↶ / ⌘Z (Windows: Strg+Z) takes back the youngest stroke
//!   or step.
//! - Every Stelle of the surface shown is marked in the view in its colour, the chosen one
//!   stronger. The list „Markierte Stellen“ in „Bilder“ ([`super::steps`]) shows the Stellen, a
//!   click frames one ([`Action::FrameMark`]).
//! - The bar over the tools has two rows. Above, the steps for the chosen Stelle: „Glätten:
//!   leicht · stark · ganz eben“ ([`Action::Smooth`]), „Bilder neu einspielen“
//!   ([`Action::Retexture`]; off while the raw data are gone, [`retexture_blocked`]) and
//!   „Melden …“ ([`Action::OpenFeedback`]). Below, the size (5–100 cm, at first 20 cm, the
//!   radius), „Vorher“ / „Nachher“ ([`Action::ShowEdit`]) and ↶. A step goes into
//!   `stellen.toml` like a stroke ([`MarkChange`]); then the window computes the touch-up as a
//!   step of its own, with its progress and „Abbrechen“ ([`Action::Touchup`]). The steps are
//!   off while one runs or waits ([`steps_blocked`]).
//! - „Nachher“ draws the surface with its touch-up; the marks then lie where the touch-up moved
//!   the corners of their faces, „Vorher“ where they were computed ([`Brush::set_moved`],
//!   [`Brush::set_after`]), so they lie on the surface drawn either way.
//!
//! The panels see [`BrushState`], [`TouchupState`] and the marks in [`AppState`]; the window
//! holds the [`Brush`]: the welded surface the loader built ([`SurfacePick`]), the stroke, the
//! faces of every Stelle and the ring under the pointer.

use super::steps::parse_decimal;
use super::theme::{icon, DARK};
use super::tools::{self, bar_button, float, Tool};
use super::view3d::{effective_show, target_size, BrushInput};
use super::{text, Action, AppState};
use crate::camera::Camera;
use crate::pick::{project, relative_to};
use crate::project::ScanState;
use crate::render::mesh::{mark_color, mark_triangles, MarkTriangle};
use crate::render::ShowMode;
use crate::surface::SurfacePick;
use driftless_map::surface::edit::{dab_normal, select_faces, RayHit, SelectDab};
use driftless_steps::marks::{Dab, DabInput, Mark, Marks, SmoothLevel, StepInput, MARKS_FILE};
use eframe::egui::{self, Align2, Context, Key, KeyboardShortcut, Modifiers, Rect, Ui};
use std::collections::{BTreeMap, VecDeque};
use std::path::{Path, PathBuf};
use std::sync::Arc;

/// The radius of the brush at first (m), and its range on the slider.
pub const DEFAULT_RADIUS: f32 = 0.20;
pub const MIN_RADIUS: f32 = 0.05;
pub const MAX_RADIUS: f32 = 1.00;
/// The dabs of a stroke lie this share of the radius apart.
pub const DAB_SPACING: f64 = 0.25;
/// The alpha (of 255) of the chosen Stelle and of the one being painted: stronger than the
/// others' ([`crate::render::mesh::MARK_TINT`]).
pub const ACTIVE_TINT: u8 = 166;
/// [`Action::FrameMark`] shows a sphere this many times the size of the Stelle: it fills
/// about half the view, with its surroundings.
const FRAME_ROOM: f32 = 2.0;
/// The way of the pointer within one frame is followed in at most this many places.
const MAX_SAMPLES: usize = 4096;
/// … halving it at most this often between two of them …
const MAX_DEPTH: u32 = 16;
/// … down to this distance on the screen (physical pixels) where it reaches or leaves the
/// surface, …
const EDGE_PX: f32 = 0.5;
/// … and to this one where it misses the surface at both ends (something small may lie
/// between).
const MISS_PX: f32 = 8.0;
/// The ring under the pointer: corners, and its width with the dark rim around it (points).
const RING_SEGMENTS: usize = 64;
const RING_WIDTH: f32 = 1.5;
const RING_RIM: f32 = 3.5;
/// The width of the size slider in the bar (points).
const SIZE_SLIDER_WIDTH: f32 = 120.0;
/// ⌘Z (Windows: Strg+Z).
const UNDO: KeyboardShortcut = KeyboardShortcut::new(Modifiers::COMMAND, Key::Z);

/// The Stellen of a map or model as read from its `stellen.toml`, or why it cannot be read
/// (German).
pub type MarksRead = Result<Marks, String>;

/// The brush as the panels see it.
#[derive(Debug, Clone, PartialEq)]
pub struct BrushState {
    /// Its radius (m), [`MIN_RADIUS`] to [`MAX_RADIUS`].
    pub radius: f32,
    /// The Stelle it paints into, of the map or model of the surface shown; `None`: the next
    /// stroke begins a new one.
    pub active: Option<u32>,
    /// A stroke is being painted (as of this frame): ↶ waits for its end.
    pub painting: bool,
}

impl Default for BrushState {
    fn default() -> Self {
        BrushState {
            radius: DEFAULT_RADIUS,
            active: None,
            painting: false,
        }
    }
}

/// The touch-up of the Stellen as the panels see it.
#[derive(Debug, Clone, PartialEq)]
pub struct TouchupState {
    /// „Nachher“ (the default): the view draws the surface shown with its touch-up; „Vorher“
    /// (`false`): as computed.
    pub after: bool,
    /// The touched-up state the view holds of the surface shown: the key of the patches it was
    /// made from ([`crate::surface::SurfaceEdit::key`]); `None`: none.
    pub shown: Option<u64>,
    /// That state differs from the surface as computed: „Vorher“ / „Nachher“ can be told apart.
    pub edited: bool,
    /// Counts the surfaces shown: a touched-up state made for one shown before is dropped.
    pub load: u64,
    /// The touch-ups waiting to run, in order: the surface folder, and the step just added
    /// (Stelle, seq) whose outcome is told ([`Action::Touchup`]). One per surface.
    pub waiting: VecDeque<(PathBuf, Option<(u32, u64)>)>,
    /// What the last touch-up of the surface shown said of the images it baked, by Stelle and
    /// step (seq): the variant of how many („Variante 2 von 5“).
    pub variants: BTreeMap<(u32, u64), (u32, u32)>,
}

impl Default for TouchupState {
    fn default() -> Self {
        TouchupState {
            after: true,
            shown: None,
            edited: false,
            load: 0,
            waiting: VecDeque::new(),
            variants: BTreeMap::new(),
        }
    }
}

/// The Stellen of the maps and models in the folders `owners` that have any, or whose
/// `stellen.toml` cannot be read (with why). Reads the files: call it with the tree, off the
/// UI thread.
pub fn read_marks(owners: impl IntoIterator<Item = PathBuf>) -> BTreeMap<PathBuf, MarksRead> {
    owners
        .into_iter()
        .filter(|dir| dir.join(MARKS_FILE).exists())
        .filter_map(|dir| match Marks::load(&dir) {
            Ok(marks) if marks.is_empty() => None,
            Ok(marks) => Some((dir, Ok(marks))),
            Err(e) => Some((dir, Err(e.to_string()))),
        })
        .collect()
}

/// The folder of the map or model the surface file `glb` belongs to: the one its surface
/// folder lies in (`karte-NNN/oberflaeche-NNN/oberflaeche.glb`).
pub fn owner_of(glb: &Path) -> Option<&Path> {
    glb.parent()?.parent()
}

/// The folder of the map or model whose surface is shown, if the surface is one of theirs in
/// the tree of the project (not a file opened from elsewhere).
pub fn shown_owner(state: &AppState) -> Option<&Path> {
    let glb = &state.surface.as_ref()?.path;
    let folder = glb.parent()?;
    let owner = owner_of(glb)?;
    let listed =
        |surfaces: &[crate::project::SurfaceInfo]| surfaces.iter().any(|s| s.dir == folder);
    let of_map = state
        .map(owner)
        .is_some_and(|(_, _, m)| listed(&m.surfaces));
    let of_model = state.model(owner).is_some_and(|m| listed(&m.surfaces));
    (of_map || of_model).then_some(owner)
}

/// The Stellen of the surface shown – of its map or model –, if it is one of the project's;
/// `None` in it: none painted yet.
pub fn shown_marks(state: &AppState) -> Option<(&Path, Option<&Marks>)> {
    let owner = shown_owner(state)?;
    let marks = state.marks.get(owner).and_then(|m| m.as_ref().ok());
    Some((owner, marks))
}

/// The folder of the surface shown, if it is one of a map or model of the project
/// ([`shown_owner`]).
pub fn shown_surface(state: &AppState) -> Option<&Path> {
    shown_owner(state)?;
    state.surface.as_ref()?.path.parent()
}

/// Whether the Stellen of the map or model of `marks` have a step: the surfaces of the map or
/// model have a touch-up.
pub fn has_steps(marks: &Marks) -> bool {
    marks.marks.iter().any(|m| !m.steps.is_empty())
}

/// The Stellen of `marks` that have a step: what the touch-up is computed from. A change that
/// leaves them as they were needs no touch-up.
pub fn stepped(marks: &Marks) -> Vec<&Mark> {
    marks.marks.iter().filter(|m| !m.steps.is_empty()).collect()
}

/// Whether the touch-up of the surface shown runs or waits.
pub fn touchup_busy(state: &AppState) -> bool {
    let Some(dir) = shown_surface(state) else {
        return false;
    };
    let running = matches!(&state.running, Some(Action::Touchup { surface, .. }) if surface == dir);
    running || state.touchup.waiting.iter().any(|(s, _)| s == dir)
}

/// Why the steps of the bar (glätten, Bilder neu einspielen) cannot be done now for the chosen
/// Stelle: none is chosen, or [`mark_steps_blocked`]. `None` if they can.
pub fn steps_blocked(state: &AppState) -> Option<String> {
    let nr = state.brush.active;
    match nr.map(|nr| mark_steps_blocked(state, nr)) {
        Some(why) => why,
        None => blocked(state)
            .or_else(|| {
                state
                    .brush
                    .painting
                    .then(|| text::STEPS_WAIT_PAINTING.into())
            })
            .or_else(|| Some(text::STEPS_NEED_MARK.into())),
    }
}

/// Why a step of Stelle `nr` of the surface shown cannot be done now: the brush cannot paint
/// ([`blocked`]), a stroke is being painted, the Stelle is gone, its touch-up runs or waits,
/// another step runs. `None` if it can.
pub fn mark_steps_blocked(state: &AppState, nr: u32) -> Option<String> {
    if let Some(why) = blocked(state) {
        return Some(why);
    }
    if state.brush.painting {
        return Some(text::STEPS_WAIT_PAINTING.into());
    }
    let there = shown_marks(state)
        .and_then(|(_, marks)| marks)
        .is_some_and(|m| m.marks.iter().any(|k| k.nr == nr));
    if !there {
        return Some(text::mark_gone(nr));
    }
    if touchup_busy(state) {
        return Some(text::TOUCHUP_RUNS.into());
    }
    state.job.as_ref().map(|job| text::busy(&job.label))
}

/// Why „Bilder neu einspielen“ cannot be done on the surface shown, besides
/// [`steps_blocked`]: it was computed without photos, the raw data of its scan (of a scan of its
/// model) are gone – a drive unplugged, a folder renamed, as the tree last found them –, or no
/// scan of it has camera images. `None` if it can.
pub fn retexture_blocked(state: &AppState) -> Option<String> {
    let owner = shown_owner(state)?;
    let dir = shown_surface(state)?;
    let surface = state.all_surfaces().find(|s| s.dir == dir);
    if surface
        .and_then(|s| s.settings.as_ref())
        .is_some_and(|s| !s.texture)
    {
        return Some(text::RETEXTURE_NO_PHOTOS.into());
    }
    let scans: Vec<&super::ScanNode> = match state.map(owner) {
        Some((scan, _, _)) => vec![scan],
        None => state.model(owner).map_or_else(Vec::new, |m| {
            let members = m.report.members.iter().filter(|m| m.included);
            members.filter_map(|m| state.scan_at(&m.scan)).collect()
        }),
    };
    if scans
        .iter()
        .any(|s| matches!(s.state, ScanState::Missing(_)))
    {
        return Some(text::RETEXTURE_NO_RAW_DATA.into());
    }
    // nothing to bake from: no scan of it has camera images
    match scans.first() {
        Some(first) if scans.iter().all(|s| s.no_color.is_some()) => first.no_color.clone(),
        _ => None,
    }
}

/// Why the brush cannot paint now – no surface shown, a surface not of the project, only the
/// points shown, its `stellen.toml` unreadable –; `None` if it can.
pub fn blocked(state: &AppState) -> Option<String> {
    if state.surface.is_none() {
        return Some(text::BRUSH_NEEDS_SURFACE.into());
    }
    let Some(owner) = shown_owner(state) else {
        return Some(text::BRUSH_NOT_IN_PROJECT.into());
    };
    if effective_show(state.view.show, true, state.cloud.is_some()) == ShowMode::Cloud {
        return Some(text::BRUSH_NEEDS_SHOWN.into());
    }
    match state.marks.get(owner) {
        Some(Err(e)) => Some(e.clone()),
        _ => None,
    }
}

/// The keys of the brush, while it is chosen and no text field has the keyboard: ⌘Z (Windows:
/// Strg+Z; not with Shift, which is „redo“ elsewhere) takes back ([`Action::UndoMark`]), Esc
/// chooses no Stelle. Not while a stroke is being painted.
pub fn keys(ctx: &Context, state: &mut AppState) -> Option<Action> {
    if state.tool != Tool::Brush || state.brush.painting || !tools::keys_free(ctx) {
        return None;
    }
    if ctx.input(|i| i.key_pressed(Key::Escape)) {
        state.brush.active = None;
    }
    let undo = ctx.input_mut(|i| !i.modifiers.shift && i.consume_shortcut(&UNDO));
    undo.then_some(Action::UndoMark)
}

/// The bar of the brush over the tools (its bottom at `bottom`), in two rows: the steps for the
/// chosen Stelle above; the size, „Vorher“ / „Nachher“ and ↶ below. Returns its rectangle and
/// what was asked for.
pub fn bar(ctx: &Context, view: Rect, bottom: f32, state: &mut AppState) -> (Rect, Option<Action>) {
    let mut action = None;
    let pos = egui::pos2(view.center().x, bottom);
    let rect = float(ctx, "pinsel", view, pos, Align2::CENTER_BOTTOM, |ui| {
        ui.horizontal(|ui| {
            action = step_buttons(ui, state);
        });
        ui.horizontal(|ui| {
            size_slider(ui, &mut state.brush.radius);
            ui.separator();
            action = edit_buttons(ui, state).or(action.take());
            ui.separator();
            action = mark_buttons(ui, state).or(action.take());
        });
    });
    (rect, action)
}

/// A button of the bar that is on unless `off` says why, with `hint` (else that reason) on
/// hover.
fn step_button(ui: &mut Ui, label: &str, name: &str, hint: &str, off: Option<&str>) -> bool {
    let r = ui
        .add_enabled_ui(off.is_none(), |ui| {
            bar_button(ui, None, Some(label), name, None)
        })
        .inner;
    let r = match off {
        Some(why) => r.on_disabled_hover_text(why),
        None => r.on_hover_text(hint),
    };
    r.clicked()
}

/// The steps for the chosen Stelle: „Glätten: leicht · stark · ganz eben“, „Bilder neu
/// einspielen“, „Melden …“. Off with the reason while they cannot be done ([`steps_blocked`],
/// [`retexture_blocked`]); „Melden …“ is always on (without a Stelle chosen it reports none).
fn step_buttons(ui: &mut Ui, state: &AppState) -> Option<Action> {
    let mut action = None;
    let off = steps_blocked(state);
    let mark = state.brush.active;
    ui.label(egui::RichText::new(text::SMOOTH_LEVELS).color(DARK.text_dim));
    let levels = [
        (
            SmoothLevel::Light,
            text::SMOOTH_LIGHT,
            text::SMOOTH_LIGHT_NAME,
            text::SMOOTH_LIGHT_HINT,
        ),
        (
            SmoothLevel::Strong,
            text::SMOOTH_STRONG,
            text::SMOOTH_STRONG_NAME,
            text::SMOOTH_STRONG_HINT,
        ),
        (
            SmoothLevel::Flat,
            text::SMOOTH_FLAT,
            text::SMOOTH_FLAT_NAME,
            text::SMOOTH_FLAT_HINT,
        ),
    ];
    for (level, label, name, hint) in levels {
        if step_button(ui, label, name, hint, off.as_deref()) {
            action = mark.map(|mark| Action::Smooth { mark, level });
        }
    }
    ui.separator();
    let off = off.or_else(|| retexture_blocked(state));
    let (label, hint) = (text::RETEXTURE, text::RETEXTURE_HINT);
    if step_button(ui, label, label, hint, off.as_deref()) {
        action = mark.map(|mark| Action::Retexture { mark });
    }
    ui.separator();
    let (label, hint) = (text::FEEDBACK_BUTTON, text::feedback_button_hint());
    if step_button(ui, label, label, &hint, None) {
        action = Some(Action::OpenFeedback { mark });
    }
    action
}

/// „Vorher“ / „Nachher“: the surface shown as computed, or with its touch-up; the one drawn is
/// filled. Off while the view holds no touch-up of it that changes anything.
fn edit_buttons(ui: &mut Ui, state: &AppState) -> Option<Action> {
    let edited = state.touchup.edited;
    let mut action = None;
    let sides = [
        (false, text::EDIT_BEFORE, text::EDIT_BEFORE_HINT),
        (true, text::EDIT_AFTER, text::EDIT_AFTER_HINT),
    ];
    for (after, label, hint) in sides {
        let on = edited && state.touchup.after == after;
        let r = ui
            .add_enabled_ui(edited, |ui| {
                bar_button(ui, None, Some(label), label, Some(on))
            })
            .inner;
        let r = r
            .on_hover_text(hint)
            .on_disabled_hover_text(text::EDIT_NONE);
        if r.clicked() {
            action = Some(Action::ShowEdit(after));
        }
    }
    action
}

/// „Größe“ and its slider in centimetres (the radius).
fn size_slider(ui: &mut Ui, radius: &mut f32) {
    ui.spacing_mut().slider_width = SIZE_SLIDER_WIDTH;
    let label = ui
        .label(egui::RichText::new(text::BRUSH_SIZE).color(DARK.text_dim))
        .on_hover_text(text::BRUSH_SIZE_HINT);
    let slider = egui::Slider::new(radius, MIN_RADIUS..=MAX_RADIUS)
        .step_by(0.01)
        .custom_formatter(|v, _| text::centimetres(v as f32))
        .custom_parser(|s| {
            parse_decimal(s.trim().trim_end_matches("cm").trim()).map(|cm| cm / 100.0)
        });
    ui.add(slider)
        .labelled_by(label.id)
        .on_hover_text(text::BRUSH_SIZE_HINT);
}

/// ↶ for the Stellen of the surface shown: takes back the youngest stroke, step or „Löschen“
/// (off with nothing to take back and while painting).
fn mark_buttons(ui: &mut Ui, state: &AppState) -> Option<Action> {
    let any = shown_marks(state).is_some_and(|(_, m)| m.is_some_and(Marks::can_undo));
    let can = any && !state.brush.painting;
    let undo = ui
        .add_enabled_ui(can, |ui| {
            bar_button(ui, Some(icon::UNDO), None, text::BRUSH_UNDO, None)
        })
        .inner;
    let hint = text::brush_undo_hint();
    let undo = undo
        .on_hover_text(hint.as_str())
        .on_disabled_hover_text(hint.as_str());
    undo.clicked().then_some(Action::UndoMark)
}

/// A change of the Stellen of a map or model, made at once on [`AppState::marks`] and then in
/// its `stellen.toml` in the background ([`Marks::change`]), in the order asked for: the file
/// ends as the window shows it, unless another window changed it meanwhile (then the file's
/// state is shown when the change is saved).
#[derive(Debug, Clone, PartialEq)]
pub enum MarkChange {
    /// A stroke ([`Action::Paint`]).
    Paint {
        mark: Option<u32>,
        dabs: Vec<DabInput>,
    },
    /// ↶ ([`Action::UndoMark`]).
    Undo,
    /// „Löschen“ ([`Action::DeleteMark`]): into the history, ↶ brings it back.
    Delete(u32),
    /// „Glätten“ of a Stelle ([`Action::Smooth`]).
    Smooth { mark: u32, level: SmoothLevel },
    /// „Bilder neu einspielen“ of a Stelle ([`Action::Retexture`]), with the next variant of
    /// the Stelle as the marks are when the change is made ([`Mark::next_variant`]).
    Retexture { mark: u32 },
    /// A step that changed nothing (Stelle, seq) taken out again, not into the history.
    DropStep { mark: u32, seq: u64 },
}

impl MarkChange {
    /// Makes the change on `marks`; with a stroke the number of the Stelle painted into. A step
    /// for a Stelle that is gone changes nothing.
    pub fn apply(&self, marks: &mut Marks) -> Option<u32> {
        match self {
            MarkChange::Paint { mark, dabs } => return Some(marks.paint(*mark, dabs)),
            MarkChange::Undo => {
                marks.undo();
            }
            MarkChange::Delete(nr) => {
                marks.remove(*nr);
            }
            &MarkChange::Smooth { mark, level } => {
                let _ = marks.add_step(mark, StepInput::Smooth { level });
            }
            &MarkChange::Retexture { mark } => {
                let variant = marks.marks.iter().find(|m| m.nr == mark);
                if let Some(variant) = variant.map(Mark::next_variant) {
                    let _ = marks.add_step(mark, StepInput::Retexture { variant });
                }
            }
            &MarkChange::DropStep { mark, seq } => {
                marks.drop_step(mark, seq);
            }
        }
        None
    }
}

/// A Stelle's dabs as the selection takes them.
fn select_dabs(dabs: &[Dab]) -> Vec<SelectDab> {
    dabs.iter()
        .map(|d| select_dab(d.centre, d.normal, d.radius, d.erase))
        .collect()
}

fn select_dab(centre: [f64; 3], normal: [f32; 3], radius: f32, erase: bool) -> SelectDab {
    SelectDab {
        centre,
        normal,
        radius: f64::from(radius),
        erase,
    }
}

/// Whether `dabs` are what `selected` was made from.
fn same_dabs(selected: &[SelectDab], dabs: &[Dab]) -> bool {
    selected.len() == dabs.len()
        && selected
            .iter()
            .zip(dabs)
            .all(|(s, d)| *s == select_dab(d.centre, d.normal, d.radius, d.erase))
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|k| (a[k] - b[k]).powi(2)).sum::<f64>().sqrt()
}

/// The surface shown, as the brush hits it.
struct Hits {
    /// Its file (the surface shown must be this one).
    glb: PathBuf,
    /// The origin it is drawn relative to (whole metres): the marks are made relative to it.
    origin: [f64; 3],
    pick: Arc<SurfacePick>,
    /// Where its touch-up moved welded vertices, by vertex: „Nachher“ draws them there.
    moved: BTreeMap<u32, [f64; 3]>,
    /// The marks lie on „Nachher“ (at the moved places), else on the surface as computed.
    after: bool,
}

impl Hits {
    /// Where vertex `v` is drawn: moved by the touch-up in „Nachher“, else as computed.
    fn place(&self, v: u32) -> [f64; 3] {
        let moved = self.after.then(|| self.moved.get(&v)).flatten();
        moved
            .copied()
            .unwrap_or_else(|| self.pick.surface.positions()[v as usize])
    }

    /// The triangles of the faces `faces` as marks in `color`, at the places drawn, made
    /// relative to the origin exactly as the pages are, so they lie on them bit for bit.
    fn triangles(&self, faces: &[u32], color: [u8; 4]) -> Vec<MarkTriangle> {
        let s = &self.pick.surface;
        let corners: Vec<[u32; 3]> = faces.iter().map(|&f| s.faces()[f as usize]).collect();
        let origin = self.origin;
        let pos = |v: u32| relative_to(origin, self.place(v));
        mark_triangles(&corners, pos, color)
    }

    /// What a Stelle of `dabs` covers, marked in its colour.
    fn select(&self, nr: u32, dabs: Vec<SelectDab>) -> Selected {
        let faces = select_faces(&self.pick.surface, &self.pick.bvh, &dabs);
        let tris = self.triangles(&faces, mark_color(nr));
        Selected { dabs, faces, tris }
    }
}

/// What a Stelle covers: the dabs it was selected from, its faces and their marks (in its
/// colour).
#[derive(Clone, Default)]
struct Selected {
    dabs: Vec<SelectDab>,
    faces: Vec<u32>,
    tris: Vec<MarkTriangle>,
}

/// A stroke being painted.
struct Stroke {
    /// The map or model whose Stellen it changes.
    owner: PathBuf,
    /// The Stelle it paints into; `None`: a new one, …
    mark: Option<u32>,
    /// … which gets this number (the colour it is shown in meanwhile).
    nr: u32,
    erase: bool,
    /// Metres.
    radius: f32,
    /// The dabs of the Stelle before the stroke.
    before: Vec<SelectDab>,
    /// Its dabs so far.
    dabs: Vec<DabInput>,
    /// The last place of the pointer (physical pixels in the view) and what it hit there.
    last: Option<Sample>,
    /// The centre of the last dab.
    last_dab: Option<[f64; 3]>,
    /// The Stelle as it is with the stroke so far.
    shown: Selected,
}

/// A place of the pointer on the screen and where its ray meets the surface.
#[derive(Clone, Copy)]
struct Sample {
    px: [f32; 2],
    hit: Option<RayHit>,
}

/// Where the pointer is on the surface: the middle and the normal of the ring drawn there.
struct Ring {
    centre: [f64; 3],
    normal: [f32; 3],
    erase: bool,
}

/// How a frame sees the surface: through the camera, in the frame of the view.
struct Sight<'a> {
    pick: &'a SurfacePick,
    camera: &'a Camera,
    viewport: [f32; 2],
    /// The origin of the view (whole metres): the camera is relative to it.
    origin: [f64; 3],
    /// The height cut, absolute.
    cut: Option<[f64; 2]>,
}

impl Sight<'_> {
    /// Where the ray through the pixel `px` (physical, from the top left of the view) first
    /// meets the surface, through the height cut.
    fn hit(&self, px: [f32; 2]) -> Option<RayHit> {
        let ray = self.camera.ray(px, self.viewport);
        let from = [0, 1, 2].map(|k| self.origin[k] + f64::from(ray.origin[k]));
        let dir = [0, 1, 2].map(|k| f64::from(ray.dir[k]));
        self.pick
            .bvh
            .raycast(&self.pick.surface, from, dir, self.cut)
    }

    fn sample(&self, px: [f32; 2]) -> Sample {
        Sample {
            px,
            hit: self.hit(px),
        }
    }

    /// The places between `a` and `b` (without `a`, ending with `b`) where the way of the
    /// pointer is looked at: halved until two neighbours on the surface lie at most half a dab
    /// spacing apart, or the screen distance is too small to matter.
    fn way(&self, a: Sample, b: Sample, spacing: f64, depth: u32, out: &mut Vec<Sample>) {
        let gap = ((b.px[0] - a.px[0]).powi(2) + (b.px[1] - a.px[1]).powi(2)).sqrt();
        let split = match (a.hit, b.hit) {
            (Some(x), Some(y)) => distance(x.pos, y.pos) > spacing / 2.0 && gap > EDGE_PX,
            (None, None) => gap > MISS_PX,
            _ => gap > EDGE_PX,
        };
        if split && depth < MAX_DEPTH && out.len() < MAX_SAMPLES {
            let mid = self.sample([(a.px[0] + b.px[0]) / 2.0, (a.px[1] + b.px[1]) / 2.0]);
            self.way(a, mid, spacing, depth + 1, out);
            self.way(mid, b, spacing, depth + 1, out);
        } else {
            out.push(b);
        }
    }
}

impl Stroke {
    /// Follows the pointer to `px`: a dab wherever the way on the surface has gone a spacing
    /// past the last dab (the first one where it first meets the surface). Whether it laid one.
    fn walk(&mut self, sight: &Sight<'_>, px: [f32; 2]) -> bool {
        let spacing = f64::from(self.radius) * DAB_SPACING;
        let to = sight.sample(px);
        let mut samples = Vec::new();
        match self.last {
            Some(from) => sight.way(from, to, spacing, 0, &mut samples),
            None => samples.push(to),
        }
        let mut laid = false;
        for hit in samples.iter().filter_map(|s| s.hit) {
            if self
                .last_dab
                .is_none_or(|c| distance(c, hit.pos) >= spacing)
            {
                let normal = dab_normal(
                    &sight.pick.surface,
                    &sight.pick.bvh,
                    &hit,
                    f64::from(self.radius),
                );
                self.dabs.push(DabInput {
                    centre: hit.pos,
                    normal,
                    radius: self.radius,
                    erase: self.erase,
                });
                self.last_dab = Some(hit.pos);
                laid = true;
            }
        }
        self.last = samples.last().copied();
        laid
    }

    /// The dabs of the Stelle with the stroke so far.
    fn all_dabs(&self) -> Vec<SelectDab> {
        let stroke = self
            .dabs
            .iter()
            .map(|d| select_dab(d.centre, d.normal, d.radius, d.erase));
        self.before.iter().copied().chain(stroke).collect()
    }
}

/// The brush at work (held by the window, see the module docs).
#[derive(Default)]
pub struct Brush {
    hits: Option<Hits>,
    stroke: Option<Stroke>,
    /// What every Stelle of the surface shown covers, by number.
    selected: BTreeMap<u32, Selected>,
    /// The marks changed since they were last handed to the view …
    dirty: bool,
    /// … with this Stelle chosen.
    shown_active: Option<u32>,
    ring: Option<Ring>,
}

impl Brush {
    /// The surface shown from now on – its file, the origin it is drawn relative to and what
    /// the loader built to hit it –, or none. A stroke on the one before is dropped.
    pub fn set_surface(&mut self, surface: Option<(PathBuf, [f64; 3], Arc<SurfacePick>)>) {
        self.hits = surface.map(|(glb, origin, pick)| Hits {
            glb,
            origin,
            pick,
            moved: BTreeMap::new(),
            after: true,
        });
        self.stroke = None;
        self.selected.clear();
        self.ring = None;
        self.dirty = true;
    }

    /// Where the touch-up of the surface shown moved its corners (welded vertex, new place), as
    /// the touched-up state the view holds has them; none: nothing moved, or no touch-up.
    pub fn set_moved(&mut self, moved: &[(u32, [f64; 3])]) {
        let Some(hits) = &mut self.hits else {
            return;
        };
        let moved: BTreeMap<u32, [f64; 3]> = moved.iter().copied().collect();
        if hits.moved != moved {
            hits.moved = moved;
            self.reshape();
        }
    }

    /// Whether the view draws „Nachher“: the marks then lie at the places the touch-up moved
    /// corners to.
    pub fn set_after(&mut self, after: bool) {
        let Some(hits) = &mut self.hits else {
            return;
        };
        if hits.after != after {
            hits.after = after;
            if !hits.moved.is_empty() {
                self.reshape();
            }
        }
    }

    /// The marks of every Stelle (and of the stroke) made anew from their faces, at the places
    /// drawn now.
    fn reshape(&mut self) {
        let Some(hits) = &self.hits else {
            return;
        };
        for (nr, selected) in &mut self.selected {
            selected.tris = hits.triangles(&selected.faces, mark_color(*nr));
        }
        if let Some(stroke) = &mut self.stroke {
            stroke.shown.tris = hits.triangles(&stroke.shown.faces, mark_color(stroke.nr));
        }
        self.dirty = true;
    }

    /// Whether a stroke is being painted.
    pub fn painting(&self) -> bool {
        self.stroke.is_some()
    }

    /// The surface shown, if the brush can hit it: the one in `state`, loaded with what the
    /// brush needs.
    fn hits_of(&self, state: &AppState) -> Option<&Hits> {
        let shown = &state.surface.as_ref()?.path;
        self.hits.as_ref().filter(|h| h.glb == *shown)
    }

    /// Follows the Stellen of the surface shown in `state`: selects the faces of each that is
    /// new or changed, forgets those that are gone, and chooses none if the chosen one is gone.
    /// Returns the marks for the view when they changed ([`Brush::take_marks`]).
    pub fn sync(&mut self, state: &mut AppState) -> Option<Vec<MarkTriangle>> {
        let marks: &[Mark] = shown_marks(state)
            .and_then(|(_, m)| m)
            .map_or(&[], |m| &m.marks);
        let painted = self.stroke.as_ref().map(|s| s.nr);
        let active = state
            .brush
            .active
            .filter(|nr| Some(*nr) == painted || marks.iter().any(|m| m.nr == *nr));
        let shown = state.surface.as_ref().map(|s| &s.path);
        let hits = self.hits.as_ref().filter(|h| Some(&h.glb) == shown);
        let before = self.selected.len();
        match hits {
            None => self.selected.clear(),
            Some(_) => self
                .selected
                .retain(|nr, _| marks.iter().any(|m| m.nr == *nr)),
        }
        let mut changed = self.selected.len() != before;
        if let Some(hits) = hits {
            for mark in marks {
                let fresh = self
                    .selected
                    .get(&mark.nr)
                    .is_some_and(|s| same_dabs(&s.dabs, &mark.dabs));
                if !fresh {
                    let selected = hits.select(mark.nr, select_dabs(&mark.dabs));
                    self.selected.insert(mark.nr, selected);
                    changed = true;
                }
            }
        }
        self.dirty |= changed;
        state.brush.active = active;
        self.take_marks(active)
    }

    /// The marks for the view if they changed since last handed over, or the chosen Stelle
    /// `active` did: every Stelle in its colour, the chosen one and the one being painted over
    /// the others and stronger ([`ACTIVE_TINT`]).
    pub fn take_marks(&mut self, active: Option<u32>) -> Option<Vec<MarkTriangle>> {
        if !self.dirty && self.shown_active == active {
            return None;
        }
        self.dirty = false;
        self.shown_active = active;
        let painted = self.stroke.as_ref().map(|s| s.nr);
        let stronger = |t: &MarkTriangle| MarkTriangle {
            color: [t.color[0], t.color[1], t.color[2], ACTIVE_TINT],
            ..*t
        };
        let mut tris: Vec<MarkTriangle> = Vec::new();
        for (nr, s) in &self.selected {
            if Some(*nr) != active && Some(*nr) != painted {
                tris.extend_from_slice(&s.tris);
            }
        }
        if let Some(s) = active
            .filter(|nr| Some(*nr) != painted)
            .and_then(|nr| self.selected.get(&nr))
        {
            tris.extend(s.tris.iter().map(stronger));
        }
        if let Some(stroke) = &self.stroke {
            tris.extend(stroke.shown.tris.iter().map(stronger));
        }
        Some(tris)
    }

    /// One frame of the pointer, after the view took it (`input`, seen by `camera` in the frame
    /// at `view_origin`): the ring under it, and the stroke – begun, followed, ended. While the
    /// brush cannot paint (another tool, nothing to paint on, the surface still going to the
    /// GPU) a stroke is dropped unsaved. Returns the stroke as [`Action::Paint`] once it ended.
    pub fn input(
        &mut self,
        state: &mut AppState,
        input: &BrushInput,
        camera: &Camera,
        view_origin: [f64; 3],
        uploading: bool,
    ) -> Option<Action> {
        self.ring = None;
        let owner = shown_owner(state).map(Path::to_path_buf);
        let usable = state.tool == Tool::Brush && !uploading && blocked(state).is_none();
        let shown = state.surface.as_ref().map(|s| &s.path);
        let pick = self
            .hits
            .as_ref()
            .filter(|h| Some(&h.glb) == shown)
            .map(|h| Arc::clone(&h.pick));
        let (Some(owner), Some(pick), true) = (owner, pick, usable) else {
            if self.stroke.take().is_some() {
                self.dirty = true;
            }
            return None;
        };
        let cut = state.view.cut.map(|[lo, hi]| {
            let z = view_origin[2];
            [z + f64::from(lo), z + f64::from(hi)]
        });
        let sight = Sight {
            pick: &pick,
            camera,
            viewport: input.viewport,
            origin: view_origin,
            cut,
        };
        let radius = state.brush.radius.clamp(MIN_RADIUS, MAX_RADIUS);
        if let Some(hit) = input.hover.and_then(|px| sight.hit(px)) {
            let normal = dab_normal(&pick.surface, &pick.bvh, &hit, f64::from(radius));
            self.ring = Some(Ring {
                centre: hit.pos,
                normal,
                erase: input.erase,
            });
        }
        if input.pressed && self.stroke.is_none() {
            let first = input.path.first().and_then(|&px| sight.hit(px));
            self.begin(state, owner, input.erase, radius, first);
        }
        if let Some(stroke) = &mut self.stroke {
            let mut laid = false;
            for &px in &input.path {
                laid |= stroke.walk(&sight, px);
            }
            if laid {
                if let Some(hits) = &self.hits {
                    stroke.shown = hits.select(stroke.nr, stroke.all_dabs());
                }
                self.dirty = true;
            }
        }
        if input.released {
            return self.finish();
        }
        None
    }

    /// Begins a stroke of the map or model `owner`: into the chosen Stelle, else a new one;
    /// erasing without a Stelle chosen into the one under the pointer (`first`), chosen from now
    /// on – or no stroke at all where there is none.
    fn begin(
        &mut self,
        state: &mut AppState,
        owner: PathBuf,
        erase: bool,
        radius: f32,
        first: Option<RayHit>,
    ) {
        let marks = state
            .marks
            .get(&owner)
            .and_then(|m| m.as_ref().ok())
            .cloned()
            .unwrap_or_default();
        let has = |nr: u32| marks.marks.iter().any(|k| k.nr == nr);
        let mut mark = state.brush.active.filter(|&nr| has(nr));
        if erase && mark.is_none() {
            mark = first.and_then(|hit| self.mark_at(hit.face));
            if mark.is_none() {
                return;
            }
            state.brush.active = mark;
        }
        let nr = mark.unwrap_or_else(|| {
            // the number the new Stelle will get, for its colour while it is painted
            let anywhere = DabInput {
                centre: [0.0; 3],
                normal: [0.0; 3],
                radius,
                erase: false,
            };
            marks.clone().paint(None, &[anywhere])
        });
        let before = mark
            .and_then(|nr| marks.marks.iter().find(|m| m.nr == nr))
            .map(|m| select_dabs(&m.dabs))
            .unwrap_or_default();
        // until its first dab the Stelle shows as it was
        let shown = mark
            .and_then(|nr| self.selected.get(&nr))
            .cloned()
            .unwrap_or_default();
        self.stroke = Some(Stroke {
            owner,
            mark,
            nr,
            erase,
            radius,
            before,
            dabs: Vec::new(),
            last: None,
            last_dab: None,
            shown,
        });
        self.dirty = true;
    }

    /// The Stelle that covers face `face`: the youngest of those that do.
    fn mark_at(&self, face: u32) -> Option<u32> {
        self.selected
            .iter()
            .rev()
            .find(|(_, s)| s.faces.binary_search(&face).is_ok())
            .map(|(nr, _)| *nr)
    }

    /// Ends the stroke: [`Action::Paint`] with its dabs, `None` if it laid none. What it covers
    /// is kept as the Stelle's, so it is not selected again once the Stelle shows with it.
    fn finish(&mut self) -> Option<Action> {
        let stroke = self.stroke.take()?;
        self.dirty = true;
        if stroke.dabs.is_empty() {
            return None;
        }
        let shown = Selected {
            dabs: stroke.all_dabs(),
            ..stroke.shown
        };
        self.selected.insert(stroke.nr, shown);
        Some(Action::Paint {
            owner: stroke.owner,
            mark: stroke.mark,
            dabs: stroke.dabs,
        })
    }

    /// Where Stelle `nr` of the surface shown lies, to look at it: its middle (absolute), its
    /// normal and its size (m, from the middle to its farthest point) – from its faces, area
    /// weighted, else from its painting dabs; `None` if it covers nothing.
    pub fn place_of(&self, state: &AppState, nr: u32) -> Option<([f64; 3], [f32; 3], f32)> {
        if let (Some(hits), Some(s)) = (self.hits_of(state), self.selected.get(&nr)) {
            if let Some(place) = faces_place(&hits.pick, &s.faces) {
                return Some(place);
            }
        }
        let (_, marks) = shown_marks(state)?;
        let mark = marks?.marks.iter().find(|m| m.nr == nr)?;
        let dabs: Vec<&Dab> = mark.dabs.iter().filter(|d| !d.erase).collect();
        if dabs.is_empty() {
            return None;
        }
        let n = dabs.len() as f64;
        let centre = [0, 1, 2].map(|k| dabs.iter().map(|d| d.centre[k]).sum::<f64>() / n);
        let normal = [0, 1, 2].map(|k| dabs.iter().map(|d| d.normal[k]).sum::<f32>());
        let size = dabs
            .iter()
            .map(|d| distance(d.centre, centre) + f64::from(d.radius))
            .fold(0.0, f64::max);
        Some((centre, normal, size as f32))
    }

    /// Draws the ring of the brush where the pointer is on the surface, in `rect` (the view),
    /// seen by `camera` in the frame at `view_origin`: a circle of the radius lying on the
    /// surface, white (erasing: in the colour of a warning) with a dark rim.
    pub fn paint_ring(
        &self,
        ui: &Ui,
        rect: Rect,
        camera: &Camera,
        view_origin: [f64; 3],
        radius: f32,
    ) {
        let Some(ring) = &self.ring else {
            return;
        };
        let ppp = ui.ctx().pixels_per_point();
        let px = target_size(rect.size(), ppp);
        let viewport = [px[0] as f32, px[1] as f32];
        let vp = camera.view_proj(viewport[0] / viewport[1]);
        let n = ring.normal.map(f64::from);
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        let n = if len > 1e-9 {
            n.map(|c| c / len)
        } else {
            [0.0, 0.0, 1.0]
        };
        // two directions across the normal
        let other = if n[2].abs() < 0.9 {
            [0.0, 0.0, 1.0]
        } else {
            [1.0, 0.0, 0.0]
        };
        let cross = |a: [f64; 3], b: [f64; 3]| {
            [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
        };
        let u = cross(n, other);
        let ul = (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt();
        let u = u.map(|c| c / ul);
        let v = cross(n, u);
        let r = f64::from(radius);
        let mut points = Vec::with_capacity(RING_SEGMENTS);
        for k in 0..RING_SEGMENTS {
            let t = std::f64::consts::TAU * k as f64 / RING_SEGMENTS as f64;
            let (s, c) = t.sin_cos();
            let p = [0, 1, 2].map(|i| ring.centre[i] + r * (c * u[i] + s * v[i]));
            let Some(q) = project(&vp, relative_to(view_origin, p), viewport) else {
                return;
            };
            // a ring reaching behind the eye is not drawn
            if q[2] > 1.0 {
                return;
            }
            points.push(rect.min + egui::vec2(q[0], q[1]) / ppp);
        }
        let color = if ring.erase {
            DARK.warn
        } else {
            egui::Color32::WHITE
        };
        let painter = ui.painter_at(rect);
        let rim = egui::Color32::from_black_alpha(160);
        painter.add(egui::Shape::closed_line(
            points.clone(),
            egui::Stroke::new(RING_RIM, rim),
        ));
        painter.add(egui::Shape::closed_line(
            points,
            egui::Stroke::new(RING_WIDTH, color),
        ));
    }
}

/// [`Brush::place_of`] from the faces of a Stelle.
fn faces_place(pick: &SurfacePick, faces: &[u32]) -> Option<([f64; 3], [f32; 3], f32)> {
    let s = &pick.surface;
    let (mut area, mut middle, mut normal) = (0.0, [0.0; 3], [0.0; 3]);
    for &f in faces {
        let cross = s.face_cross(f);
        let a = (cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]).sqrt() / 2.0;
        let c = s.face_centroid(f);
        for k in 0..3 {
            middle[k] += a * c[k];
            normal[k] += cross[k];
        }
        area += a;
    }
    if area.is_nan() || area <= 0.0 {
        return None;
    }
    let middle = middle.map(|m| m / area);
    let size = faces
        .iter()
        .flat_map(|&f| s.face_corners(f))
        .map(|p| distance(p, middle))
        .fold(0.0, f64::max);
    Some((middle, normal.map(|c| c as f32), size as f32))
}

/// The radius to frame a Stelle of `size` with: room around it.
pub fn frame_radius(size: f32) -> f32 {
    size * FRAME_ROOM
}
