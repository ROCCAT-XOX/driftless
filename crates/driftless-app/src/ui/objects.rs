//! The tool „Objekt“ of the 3D view: sockets, connections, cables, pipes, doors and windows set
//! by hand on what is shown, with the bar above the tools ([`bar`]) and the keys Enter,
//! Backspace, Esc and ⌘Z. A click hits the points, like „Messen“, while only the surface is
//! shown the surface. A door or window takes two clicks, its two edges, and is finished then.
//!
//! The objects live with the map or model ([`view_owner`]), not with a floor plan: in its
//! `objekte.toml`, read with the tree into [`AppState::objects`], changed there at once and
//! written in the background ([`ObjectChange`], the window's `HelperTask::Objects`), as the
//! Stellen of the brush are. [`sync`] keeps what hangs on them current frame by frame: the
//! plan shown and the list of measurements.
//!
//! The measurements of the tool „Messen“ are objects of the kind „Messung“ of the same file.
//! [`AppState::measurements`] lists those of [`measure_owner`]; on a cloud from elsewhere they
//! stay in memory, as they always did ([`MeasureView`]).

use super::plan;
use super::steps::parse_decimal;
use super::theme::{icon, DARK};
use super::tools::{bar_button, float, keys_free, Measurement, Tool};
use super::view3d::{effective_show, target_size};
use super::{brush, text, Action, AppState};
use crate::camera::Camera;
use crate::pick::{project, relative_to};
use crate::render::ShowMode;
use driftless_steps::merge::MODEL_LAS;
use driftless_steps::objects::{
    ConnectionKind, ObjectInput, ObjectKind, Objects, ObjectsError, OBJECTS_FILE,
};
use eframe::egui::{self, Align2, Context, Key, Modifiers, Popup, Rect, RichText, Stroke, Ui};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// The diameter a pipe starts with (mm).
const DEFAULT_DIAMETER: f32 = 50.0;
/// The largest diameter the bar takes (mm).
const MAX_DIAMETER: f64 = 2000.0;
/// The height a door starts with (metres, from the floor).
const DEFAULT_DOOR_HEIGHT: f64 = 2.0;
/// The largest height or sill the bar takes (metres).
const MAX_OPENING_HEIGHT: f64 = 10.0;
/// The width of the view from which the settings of a kind fit beside the kinds in the bar
/// (points); in a narrower window they go into a second row.
const ONE_ROW_WIDTH: f32 = 660.0;
/// Radius of a point object in the view and of the corners of a line (points), and the width
/// of a line.
const DOT_RADIUS: f32 = 5.0;
const CORNER_RADIUS: f32 = 3.0;
const LINE_WIDTH: f32 = 2.0;
/// The ring around the object marked in the list (points).
const MARK_RADIUS: f32 = 9.0;
/// The least radius of the view „In 3D zeigen“ goes to (m): a point object has none of its own.
const MIN_FRAME_RADIUS: f64 = 0.5;

/// The kinds the tool sets, in the order of the bar.
const KINDS: [ObjectKind; 6] = [
    ObjectKind::Socket,
    ObjectKind::Connection,
    ObjectKind::Cable,
    ObjectKind::Pipe,
    ObjectKind::Door,
    ObjectKind::Window,
];
/// The sorts of connection, in the order of the bar.
const CONNECTIONS: [ConnectionKind; 7] = [
    ConnectionKind::Power,
    ConnectionKind::Water,
    ConnectionKind::Drain,
    ConnectionKind::Heating,
    ConnectionKind::Gas,
    ConnectionKind::Network,
    ConnectionKind::Other,
];

/// What the objects of a map or model read as: why not, if `objekte.toml` cannot be read.
pub type ObjectsRead = Result<Objects, String>;

/// The tool „Objekt“: what is set next, and a line under way (or the first edge of a door or
/// window).
#[derive(Debug, Clone, PartialEq)]
pub struct ObjectTool {
    pub kind: ObjectKind,
    /// The sort of a connection.
    pub connection: ConnectionKind,
    /// The diameter of a pipe.
    pub diameter_mm: f32,
    /// The height of a door (metres); 0: not known.
    pub door_height_m: f64,
    /// The height of a window (metres); 0: not known.
    pub window_height_m: f64,
    /// The sill of a window above the floor (metres); 0: not known.
    pub sill_m: f64,
    /// The points of a cable or pipe being set, absolute; the first edge of a door or window.
    pub line: Vec<[f64; 3]>,
    /// Why the last click or Enter set nothing (German).
    pub message: Option<String>,
    /// The map or model the line is being set on: it does not cross to another one.
    pub line_owner: Option<PathBuf>,
}

impl Default for ObjectTool {
    fn default() -> Self {
        ObjectTool {
            kind: ObjectKind::Socket,
            connection: ConnectionKind::Power,
            diameter_mm: DEFAULT_DIAMETER,
            door_height_m: DEFAULT_DOOR_HEIGHT,
            window_height_m: 0.0,
            sill_m: 0.0,
            line: Vec::new(),
            message: None,
            line_owner: None,
        }
    }
}

impl ObjectTool {
    /// The object of the tool with `points`.
    fn input(&self, points: Vec<[f64; 3]>) -> ObjectInput {
        let known = |v: f64| (v > 0.0).then_some(v);
        let (height, sill) = match self.kind {
            ObjectKind::Door => (known(self.door_height_m), None),
            ObjectKind::Window => (known(self.window_height_m), known(self.sill_m)),
            _ => (None, None),
        };
        ObjectInput {
            kind: self.kind,
            points,
            name: String::new(),
            note: String::new(),
            connection: (self.kind == ObjectKind::Connection).then_some(self.connection),
            diameter_mm: (self.kind == ObjectKind::Pipe).then_some(self.diameter_mm),
            height_m: height,
            sill_m: sill,
            properties: BTreeMap::new(),
        }
    }

    fn sets_a_line(&self) -> bool {
        matches!(self.kind, ObjectKind::Cable | ObjectKind::Pipe)
    }

    /// Chooses the kind set next. A line under way stays while the next one is a line too; the
    /// first edge of a door or window does not become the start of something else.
    fn choose(&mut self, kind: ObjectKind) {
        let was_a_line = self.sets_a_line();
        self.kind = kind;
        self.message = None;
        if !(was_a_line && self.sets_a_line()) {
            self.line.clear();
        }
    }
}

/// Which measurements [`AppState::measurements`] holds.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MeasureView {
    /// The map or model they are of; `None`: made on a cloud from elsewhere, in memory only.
    pub owner: Option<PathBuf>,
    /// Those made in memory, kept aside while the ones of a map or model are shown.
    pub memory: Vec<Measurement>,
}

/// A change of the objects of a map or model: made at once on those shown, then the same on the
/// file ([`Objects::change`]).
#[derive(Debug, Clone, PartialEq)]
pub enum ObjectChange {
    Add(ObjectInput),
    Remove(u32),
    Rename(u32, String),
    Note(u32, String),
    Undo,
    /// „Messungen löschen“: all of them, one step back for ↶.
    ClearMeasurements,
}

impl ObjectChange {
    pub fn apply(&self, objects: &mut Objects) -> Result<(), ObjectsError> {
        match self {
            ObjectChange::Add(input) => {
                objects.add(input.clone())?;
            }
            ObjectChange::Remove(nr) => {
                objects.remove(*nr);
            }
            ObjectChange::Rename(nr, name) => {
                objects.rename(*nr, name);
            }
            ObjectChange::Note(nr, note) => {
                objects.set_note(*nr, note);
            }
            ObjectChange::Undo => {
                objects.undo();
            }
            ObjectChange::ClearMeasurements => {
                objects.remove_kind(ObjectKind::Measure);
            }
        }
        Ok(())
    }
}

/// The objects of the maps and models in the folders `owners` that have an `objekte.toml`, or
/// why it cannot be read (in plain words: where it is damaged). Reads the files: call it with
/// the tree, off the UI thread.
pub fn read_objects(owners: impl IntoIterator<Item = PathBuf>) -> BTreeMap<PathBuf, ObjectsRead> {
    owners
        .into_iter()
        .filter(|dir| dir.join(OBJECTS_FILE).exists())
        .map(|dir| {
            let read = Objects::load(&dir).map_err(|e| unreadable(&dir, &e.to_string()));
            (dir, read)
        })
        .collect()
}

/// Why the `objekte.toml` of the folder `dir` cannot be read: where it is damaged, as for the
/// other files of the project ([`text::damaged_toml`]); else `said`, what reading it said.
fn unreadable(dir: &Path, said: &str) -> String {
    let file = dir.join(OBJECTS_FILE);
    let Ok(content) = std::fs::read_to_string(&file) else {
        return said.to_string();
    };
    match toml::from_str::<Objects>(&content) {
        Err(e) => text::damaged_toml(&file, &content, &e),
        Ok(_) => said.to_string(),
    }
}

/// The objects of the map or model in `owner`; `None` while its file cannot be read.
pub fn of<'a>(state: &'a AppState, owner: &Path) -> Option<&'a Objects> {
    static NONE: std::sync::OnceLock<Objects> = std::sync::OnceLock::new();
    match state.objects.get(owner) {
        Some(Ok(objects)) => Some(objects),
        Some(Err(_)) => None,
        None => Some(NONE.get_or_init(Objects::default)),
    }
}

/// The map or model of the cloud shown, if it is one of the project: a cloud of a map, or the
/// `modell.las` of a model.
fn cloud_owner(state: &AppState) -> Option<PathBuf> {
    let path = &state.cloud.as_ref()?.path;
    let dir = path.parent()?;
    let of_map = state
        .map(dir)
        .is_some_and(|(_, _, m)| m.clouds.iter().any(|c| c == path));
    let of_model = state.model(dir).is_some() && path.file_name() == Some(MODEL_LAS.as_ref());
    (of_map || of_model).then(|| dir.to_path_buf())
}

/// The map or model of what a click hits now, only from the project: of the surface while it
/// is shown alone, else of the cloud.
pub fn view_owner(state: &AppState) -> Option<PathBuf> {
    let shown = effective_show(
        state.view.show,
        state.surface.is_some(),
        state.cloud.is_some(),
    );
    if shown == ShowMode::Surface {
        return brush::shown_owner(state).map(Path::to_path_buf);
    }
    cloud_owner(state)
}

/// The map or model the measurements are of: of the plan on the screen, else of the cloud
/// („Messen“ hits the points, also while only the surface is seen), and without a cloud of the
/// surface shown.
pub fn measure_owner(state: &AppState) -> Option<PathBuf> {
    if let Some(owner) = plan_owner(state) {
        return Some(owner);
    }
    match state.cloud {
        Some(_) => cloud_owner(state),
        None => view_owner(state),
    }
}

/// The map or model of the plan on the screen.
fn plan_owner(state: &AppState) -> Option<PathBuf> {
    if !plan::on_screen(state) {
        return None;
    }
    state
        .plan
        .shown
        .as_ref()?
        .dir
        .parent()
        .map(Path::to_path_buf)
}

/// The map or model whose objects the field „Planung“ lists: of the plan on the screen, else
/// of what is shown.
pub fn listed_owner(state: &AppState) -> Option<PathBuf> {
    plan_owner(state).or_else(|| view_owner(state))
}

/// Why „Objekt“ cannot be chosen now: nothing of a map or model of the project shown, or its
/// `objekte.toml` unreadable; `None` if it can.
pub fn blocked(state: &AppState) -> Option<String> {
    let Some(owner) = view_owner(state) else {
        return Some(text::OBJECTS_NEED_OWNER.into());
    };
    match state.objects.get(&owner) {
        Some(Err(e)) => Some(e.clone()),
        _ => None,
    }
}

/// Whether ↶ has something to take back of the objects listed now.
pub fn can_undo(state: &AppState) -> bool {
    listed_owner(state).is_some_and(|o| of(state, &o).is_some_and(Objects::can_undo))
}

/// Whether the measurements listed are kept with a map or model (else in memory only).
pub fn measurements_saved(state: &AppState) -> bool {
    measure_owner(state).is_some_and(|o| of(state, &o).is_some())
}

fn measurements_of(objects: &Objects) -> impl Iterator<Item = Measurement> + '_ {
    objects
        .all()
        .iter()
        .filter(|o| o.kind == ObjectKind::Measure)
        .filter_map(|o| match o.points[..] {
            [a, b] => Some(Measurement { a, b }),
            _ => None,
        })
}

/// Every frame: the list of measurements follows the map or model shown (those of a cloud from
/// elsewhere wait in memory), a line under way is dropped when its map or model is left, and
/// the plan shown takes the objects as they are now.
pub fn sync(state: &mut AppState) {
    let owner = measure_owner(state);
    if owner != state.measure_view.owner {
        if state.measure_view.owner.is_none() {
            state.measure_view.memory = std::mem::take(&mut state.measurements);
        }
        if owner.is_none() {
            state.measurements = std::mem::take(&mut state.measure_view.memory);
        }
        state.measure_view.owner = owner.clone();
        state.measure_a = None;
    }
    if let Some(objects) = owner.as_deref().and_then(|o| of(state, o)) {
        let same = measurements_of(objects).eq(state.measurements.iter().copied());
        if !same {
            state.measurements = measurements_of(objects).collect();
        }
    }
    let here = view_owner(state);
    let tool = &mut state.object_tool;
    if !tool.line.is_empty() && tool.line_owner != here {
        tool.line.clear();
        tool.message = None;
    }
    sync_plan(state);
}

/// The plan shown takes the objects of its map or model as they are now; an object marked that
/// is gone is marked no more.
fn sync_plan(state: &mut AppState) {
    let Some(shown) = &state.plan.shown else {
        return;
    };
    let Some(objects) = shown.dir.parent().and_then(|o| of(state, o)) else {
        return;
    };
    if *objects != shown.objects {
        let objects = objects.clone();
        plan::set_objects(&mut state.plan, objects);
    }
    let gone = |nr: u32| {
        state
            .plan
            .shown
            .as_ref()
            .is_some_and(|s| s.objects.get(nr).is_none())
    };
    if state.plan.selected_object.is_some_and(gone) {
        state.plan.selected_object = None;
    }
}

/// A click of the tool on the point `position` (absolute): a socket or connection is set
/// ([`Action::AddObject`]), a cable or pipe gets a point – the same point again (a double click)
/// is no new one –, a door or window its first edge, then with the second it is set. Why not in
/// [`ObjectTool::message`]; a second edge that is refused leaves the first one, to try again.
pub fn point_clicked(state: &mut AppState, position: [f64; 3]) -> Option<Action> {
    if let Some(why) = blocked(state) {
        state.object_tool.message = Some(why);
        return None;
    }
    let owner = view_owner(state)?;
    let tool = &mut state.object_tool;
    tool.message = None;
    if tool.kind.is_opening() {
        return opening_clicked(tool, owner, position);
    }
    if !tool.sets_a_line() {
        let input = tool.input(vec![position]);
        return Some(Action::AddObject { owner, input });
    }
    if tool.line.last() != Some(&position) {
        tool.line.push(position);
        tool.line_owner = Some(owner);
    }
    None
}

/// A click of the tool on `position` while it sets a door or window.
fn opening_clicked(tool: &mut ObjectTool, owner: PathBuf, position: [f64; 3]) -> Option<Action> {
    let Some(&first) = tool.line.first() else {
        tool.line.push(position);
        tool.line_owner = Some(owner);
        return None;
    };
    if first == position {
        return None;
    }
    let input = tool.input(vec![first, position]);
    if let Err(why) = input.check() {
        tool.message = Some(why.to_string());
        return None;
    }
    tool.line.clear();
    Some(Action::AddObject { owner, input })
}

/// Ends the cable or pipe under way (Enter, double click): set with its points, if it has at
/// least two and a pipe its diameter; else the line stays and [`ObjectTool::message`] says why.
pub fn finish_line(state: &mut AppState) -> Option<Action> {
    let owner = view_owner(state)?;
    let tool = &mut state.object_tool;
    if !tool.sets_a_line() || tool.line.is_empty() {
        return None;
    }
    if tool.line.len() < 2 {
        tool.message = Some(text::object_line_short(tool.kind).to_string());
        return None;
    }
    if tool.kind == ObjectKind::Pipe && !(tool.diameter_mm.is_finite() && tool.diameter_mm > 0.0) {
        tool.message = Some(text::OBJECT_PIPE_NEEDS_DIAMETER.to_string());
        return None;
    }
    let points = std::mem::take(&mut tool.line);
    tool.message = None;
    let input = tool.input(points);
    Some(Action::AddObject { owner, input })
}

/// Backspace: takes back the last point of the line under way.
pub fn undo_point(state: &mut AppState) {
    state.object_tool.line.pop();
    state.object_tool.message = None;
}

/// Esc: drops the line under way.
pub fn cancel_line(state: &mut AppState) {
    state.object_tool.line.clear();
    state.object_tool.message = None;
}

/// After a new measurement (the last of [`AppState::measurements`]): keeps it with its map or
/// model. `None` on a cloud from elsewhere: there it stays in memory.
pub fn measured(state: &mut AppState) -> Option<Action> {
    let owner = measure_owner(state)?;
    let last = state.measurements.last()?;
    let input = ObjectInput {
        kind: ObjectKind::Measure,
        points: vec![last.a, last.b],
        name: String::new(),
        note: String::new(),
        connection: None,
        diameter_mm: None,
        height_m: None,
        sill_m: None,
        properties: BTreeMap::new(),
    };
    Some(Action::AddObject { owner, input })
}

/// The keys of the tool, while no text field has the keyboard and nothing else waits for them:
/// Enter ends a line, Backspace takes its last point back, Esc drops it (the first edge of a
/// door or window as well).
pub fn keys(ctx: &Context, state: &mut AppState) -> Option<Action> {
    if state.tool != Tool::Object || plan::on_screen(state) || !keys_free(ctx) {
        return None;
    }
    if state.object_tool.line.is_empty() || state.ask.is_some() || Popup::is_any_open(ctx) {
        return None;
    }
    let (enter, back, esc) = ctx.input_mut(|i| {
        (
            i.consume_key(Modifiers::NONE, Key::Enter),
            i.consume_key(Modifiers::NONE, Key::Backspace),
            i.consume_key(Modifiers::NONE, Key::Escape),
        )
    });
    if back {
        undo_point(state);
    }
    if esc {
        cancel_line(state);
    }
    enter.then(|| finish_line(state)).flatten()
}

/// The bar of the tool over the tools (its bottom at `bottom`): the kind, for a connection the
/// sort, for a pipe the diameter, for a door or window its height, for a window also the sill,
/// and ↶; below them why the last click set nothing. Returns its rectangle and what was asked
/// for.
pub fn bar(ctx: &Context, view: Rect, bottom: f32, state: &mut AppState) -> (Rect, Option<Action>) {
    let mut undo = false;
    let can_undo = can_undo(state);
    let pos = egui::pos2(view.center().x, bottom);
    let tool = &mut state.object_tool;
    let one_row = view.width() >= ONE_ROW_WIDTH;
    let rect = float(ctx, "objekt", view, pos, Align2::CENTER_BOTTOM, |ui| {
        ui.horizontal(|ui| {
            for kind in KINDS {
                let label = kind.label();
                if bar_button(ui, None, Some(label), label, Some(tool.kind == kind)).clicked() {
                    tool.choose(kind);
                }
            }
            if one_row && has_settings(tool.kind) {
                ui.separator();
                settings(ui, tool);
            }
            ui.separator();
            let r = ui
                .add_enabled_ui(can_undo, |ui| {
                    bar_button(ui, Some(icon::UNDO), None, text::BRUSH_UNDO, None)
                })
                .inner
                .on_hover_text(text::OBJECT_UNDO_HINT);
            undo = r.clicked();
        });
        if !one_row && has_settings(tool.kind) {
            ui.horizontal(|ui| settings(ui, tool));
        }
        if let Some(message) = &tool.message {
            ui.add(egui::Label::new(RichText::new(message).color(DARK.warn)).wrap());
        }
    });
    (rect, undo.then_some(Action::UndoObject))
}

/// Whether the kind has settings in the bar.
fn has_settings(kind: ObjectKind) -> bool {
    matches!(
        kind,
        ObjectKind::Connection | ObjectKind::Pipe | ObjectKind::Door | ObjectKind::Window
    )
}

/// The settings of the kind chosen: the sort of a connection, the diameter of a pipe, the
/// height of a door or window and the sill of a window.
fn settings(ui: &mut Ui, tool: &mut ObjectTool) {
    match tool.kind {
        ObjectKind::Connection => {
            ui.label(text::OBJECT_CONNECTION);
            egui::ComboBox::from_id_salt("objekt_sorte")
                .selected_text(tool.connection.label())
                .show_ui(ui, |ui| {
                    for c in CONNECTIONS {
                        ui.selectable_value(&mut tool.connection, c, c.label());
                    }
                });
        }
        ObjectKind::Pipe => {
            let label = ui.label(text::OBJECT_DIAMETER);
            let field = egui::DragValue::new(&mut tool.diameter_mm)
                .range(0.0..=MAX_DIAMETER)
                .speed(1.0)
                .max_decimals(1)
                .suffix(" mm");
            ui.add(field).labelled_by(label.id);
        }
        ObjectKind::Door => length_field(ui, text::OBJECT_HEIGHT, &mut tool.door_height_m),
        ObjectKind::Window => {
            length_field(ui, text::OBJECT_HEIGHT, &mut tool.window_height_m);
            length_field(ui, text::OBJECT_SILL, &mut tool.sill_m);
        }
        _ => {}
    }
}

/// A height or sill of a door or window in the bar, in metres with a comma; 0 means not known
/// and shows „–“, an empty field is that too.
fn length_field(ui: &mut Ui, name: &str, value: &mut f64) {
    let label = ui.label(name);
    let field = egui::DragValue::new(value)
        .range(0.0..=MAX_OPENING_HEIGHT)
        .speed(0.01)
        .custom_formatter(|v, _| {
            if v > 0.0 {
                text::meters(v)
            } else {
                text::OBJECT_LENGTH_UNKNOWN.to_string()
            }
        })
        .custom_parser(|s| match s.trim_end_matches('m').trim() {
            "" | text::OBJECT_LENGTH_UNKNOWN => Some(0.0),
            number => parse_decimal(number),
        });
    ui.add(field)
        .labelled_by(label.id)
        .on_hover_text(text::OBJECT_LENGTH_HINT);
}

/// Draws the objects of the map or model shown over the view in `rect`, seen by `camera` in the
/// frame at `origin` – in the step „Planung“ and while the tool is chosen, each only while its
/// layer is on – and the line under way with a dashed piece to the pointer. The marked object
/// gets a ring.
pub fn paint(ui: &mut Ui, rect: Rect, camera: &Camera, origin: [f64; 3], state: &AppState) {
    let line = &state.object_tool.line;
    let wanted = state.step == super::Step::Planung || state.tool == Tool::Object;
    let owner = view_owner(state).filter(|_| wanted);
    let objects = owner.as_deref().and_then(|o| of(state, o));
    if objects.is_none() && line.is_empty() {
        return;
    }
    let ppp = ui.ctx().pixels_per_point();
    let px = target_size(rect.size(), ppp);
    let viewport = [px[0] as f32, px[1] as f32];
    let vp = camera.view_proj(viewport[0] / viewport[1]);
    // on the screen and in front of the near plane
    let screen = |p: [f64; 3]| {
        let q = project(&vp, relative_to(origin, p), viewport)?;
        (q[2] <= 1.0).then(|| rect.min + egui::vec2(q[0], q[1]) / ppp)
    };
    let painter = ui.painter_at(rect);
    for object in objects.iter().flat_map(|o| o.all()) {
        if object.kind == ObjectKind::Measure || state.plan.hidden.contains(object.kind.label()) {
            continue;
        }
        let colour = plan::layer_colour(object.kind.label());
        let stroke = Stroke::new(LINE_WIDTH, colour);
        let at: Vec<Option<egui::Pos2>> = object.points.iter().copied().map(screen).collect();
        let marked = state.plan.selected_object == Some(object.nr);
        for piece in at.windows(2) {
            if let [Some(a), Some(b)] = piece {
                painter.line_segment([*a, *b], stroke);
            }
        }
        for p in at.iter().flatten() {
            let radius = if object.points.len() == 1 {
                DOT_RADIUS
            } else {
                CORNER_RADIUS
            };
            painter.circle_filled(*p, radius, colour);
            if marked {
                painter.circle_stroke(*p, MARK_RADIUS, Stroke::new(LINE_WIDTH, DARK.accent));
            }
        }
        if let Some(mid) = middle(&at).filter(|m| rect.contains(*m)) {
            tag(ui, &painter, mid, object.label());
        }
    }
    let stroke = Stroke::new(LINE_WIDTH, DARK.measure);
    let at: Vec<egui::Pos2> = line.iter().copied().filter_map(screen).collect();
    for piece in at.windows(2) {
        painter.line_segment([piece[0], piece[1]], stroke);
    }
    for p in &at {
        painter.circle_filled(*p, CORNER_RADIUS, DARK.measure);
    }
    if let (Some(&last), Some(hover)) = (
        at.last(),
        ui.ctx().pointer_hover_pos().filter(|p| rect.contains(*p)),
    ) {
        painter.extend(egui::Shape::dashed_line(&[last, hover], stroke, 6.0, 4.0));
    }
}

/// Where the label of an object goes: at its point, on a line at the corner in the middle.
fn middle(at: &[Option<egui::Pos2>]) -> Option<egui::Pos2> {
    match at {
        [point] => *point,
        _ => at[at.len() / 2].or_else(|| at.iter().flatten().next().copied()),
    }
}

/// `label` beside `at` on a small plate, as a widget, so that it can be read out.
fn tag(ui: &mut Ui, painter: &egui::Painter, at: egui::Pos2, label: String) {
    let font = egui::TextStyle::Body.resolve(ui.style());
    let size = ui.fonts_mut(|f| f.layout_no_wrap(label.clone(), font, DARK.text).size());
    let centre = at + egui::vec2(0.0, -16.0);
    let plate = Rect::from_center_size(centre, size + egui::vec2(10.0, 4.0));
    painter.rect_filled(plate, egui::CornerRadius::same(5), DARK.float_fill);
    let widget = egui::Label::new(RichText::new(label).color(DARK.text)).selectable(false);
    ui.put(plate, widget);
}

/// The centre of object `points` and the radius of a view around them that shows it all.
pub fn frame_of(points: &[[f64; 3]]) -> Option<([f64; 3], f64)> {
    let n = points.len() as f64;
    if n == 0.0 {
        return None;
    }
    let centre = [0, 1, 2].map(|k| points.iter().map(|p| p[k]).sum::<f64>() / n);
    let distance = |p: &[f64; 3]| {
        (0..3)
            .map(|k| (p[k] - centre[k]).powi(2))
            .sum::<f64>()
            .sqrt()
    };
    let radius = points.iter().map(distance).fold(0.0, f64::max);
    Some((centre, radius.max(MIN_FRAME_RADIUS)))
}
