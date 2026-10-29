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
//!   stronger. The bar over the tools holds the size (5–100 cm, at first 20 cm, the radius) and
//!   ↶; the list „Markierte Stellen“ in „Bilder“ ([`super::steps`]) shows the Stellen, a click
//!   frames one ([`Action::FrameMark`]).
//!
//! The panels see [`BrushState`] and the marks in [`AppState`]; the window holds the [`Brush`]:
//! the welded surface the loader built ([`SurfacePick`]), the stroke, the faces of every Stelle
//! and the ring under the pointer.

use super::steps::parse_decimal;
use super::theme::{icon, DARK};
use super::tools::{self, bar_button, float, Tool};
use super::view3d::{effective_show, target_size, BrushInput};
use super::{text, Action, AppState};
use crate::camera::Camera;
use crate::pick::{project, relative_to};
use crate::render::mesh::{mark_color, mark_triangles, MarkTriangle};
use crate::render::ShowMode;
use crate::surface::SurfacePick;
use driftless_map::surface::edit::{dab_normal, select_faces, RayHit, SelectDab};
use driftless_steps::marks::{Dab, DabInput, Mark, Marks, MARKS_FILE};
use eframe::egui::{self, Align2, Context, Key, KeyboardShortcut, Modifiers, Rect, Ui};
use std::collections::BTreeMap;
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

/// The Stellen of the maps and models in the folders `owners` that have any, or whose
/// `stellen.toml` cannot be read (with why). Reads the files: call it with the tree, off the
/// UI thread.
pub fn read_marks(owners: impl IntoIterator<Item = PathBuf>) -> BTreeMap<PathBuf, MarksRead> {
    owners
        .into_iter()
        .filter(|dir| dir.join(MARKS_FILE).exists())
        .filter_map(|dir| match Marks::load(&dir) {
            Ok(marks) if marks.marks.is_empty() => None,
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

/// The bar of the brush over the tools (its bottom at `bottom`): the size, then the buttons for
/// the Stellen ([`mark_buttons`]). Returns its rectangle and what was asked for.
pub fn bar(ctx: &Context, view: Rect, bottom: f32, state: &mut AppState) -> (Rect, Option<Action>) {
    let mut action = None;
    let pos = egui::pos2(view.center().x, bottom);
    let rect = float(ctx, "pinsel", view, pos, Align2::CENTER_BOTTOM, |ui| {
        ui.horizontal(|ui| {
            size_slider(ui, &mut state.brush.radius);
            ui.separator();
            action = mark_buttons(ui, state);
        });
    });
    (rect, action)
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

/// The buttons for the Stellen of the surface shown: ↶ takes back the youngest stroke or step
/// (off with nothing to take back and while painting). The touch-up steps of the chosen Stelle
/// take their place before it.
fn mark_buttons(ui: &mut Ui, state: &AppState) -> Option<Action> {
    let any = shown_marks(state).is_some_and(|(_, m)| m.is_some_and(|m| !m.marks.is_empty()));
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
    /// „Löschen“ ([`Action::DeleteMark`]).
    Delete(u32),
}

impl MarkChange {
    /// Makes the change on `marks`; with a stroke the number of the Stelle painted into.
    pub fn apply(&self, marks: &mut Marks) -> Option<u32> {
        match self {
            MarkChange::Paint { mark, dabs } => Some(marks.paint(*mark, dabs)),
            MarkChange::Undo => {
                marks.undo();
                None
            }
            MarkChange::Delete(nr) => {
                marks.remove(*nr);
                None
            }
        }
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
}

impl Hits {
    /// The triangles of the faces `faces` as marks in `color`, made relative to the origin
    /// exactly as the pages are, so they lie on them bit for bit.
    fn triangles(&self, faces: &[u32], color: [u8; 4]) -> Vec<MarkTriangle> {
        let s = &self.pick.surface;
        let corners: Vec<[u32; 3]> = faces.iter().map(|&f| s.faces()[f as usize]).collect();
        let origin = self.origin;
        let pos = |v: u32| relative_to(origin, s.positions()[v as usize]);
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
        self.hits = surface.map(|(glb, origin, pick)| Hits { glb, origin, pick });
        self.stroke = None;
        self.selected.clear();
        self.ring = None;
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
