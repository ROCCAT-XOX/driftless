//! The dialog „Melden“ and the window „Meldungen auf diesem Rechner“ (☰ → „Meldungen …“); what
//! is written and read is in [`crate::feedback`].
//!
//! - „Melden“ ([`show_dialog`]) is a modal dialog over the window: „Fehler“ or „Wunsch“, a text
//!   field (it has the keyboard when the dialog opens), ticked „Bild des Fensters anhängen“
//!   (with a small picture of it) and „Markierte Stellen anhängen (n)“ – the Stellen of the
//!   surface shown –, the Stelle the report is about if it came from the bar of the brush, and
//!   „Abbrechen“ / „Melden“ („Melden“ waits for a text). Esc closes it without saving like
//!   „Abbrechen“; a click beside it does not, so the text typed stays. The window fills in what
//!   comes along without the user ([`FeedbackDialog::context`]).
//! - The list ([`show_list`]) shows the reports of this computer, the newest first, each with
//!   its kind (a red or green stripe), time, project, Stelle, text, „Stand“ (orange while
//!   „offen“) and „Antwort“, and „Ordner öffnen“ / „Zur Ansicht“. The window reads the folder
//!   anew each time it opens it.

use super::theme::DARK;
use super::{text, Action, AppState, Selection, Step};
use crate::feedback::{Feedback, FeedbackKind, FeedbackSelection};
use driftless_steps::marks::Marks;
use eframe::egui::{
    self, Align, Color32, ColorImage, Context, CornerRadius, Frame, Id, Key, Layout, Margin,
    Modifiers, Rect, RichText, TextureHandle, TextureOptions, Ui,
};
use std::path::{Path, PathBuf};
use std::sync::Arc;

/// The width of the dialog (points).
const WIDTH: f32 = 360.0;
/// The small picture of the window in the dialog: its width (points) and that of its texture
/// (pixels, sharp on a display of two pixels per point).
const THUMB_WIDTH: f32 = 84.0;
const THUMB_PX: usize = 168;
/// The size of the list at first (points), and the most its reports take before they scroll.
const LIST_WIDTH: f32 = 440.0;
const LIST_HEIGHT: f32 = 520.0;
/// The stripe of the kind at the left edge of a report in the list (points).
const STRIPE: f32 = 3.0;

/// The kinds of a selection in `[auswahl]` of a report.
const SCAN: &str = "scan";
const RUN: &str = "lauf";
const MAP: &str = "karte";
const CLOUD: &str = "punktwolke";
const SURFACE: &str = "oberflaeche";
const STATE: &str = "stand";
const MODEL: &str = "modell";

/// The small picture of the window on the GPU (egui's handle has no `Debug`).
#[derive(Clone)]
struct Thumbnail(TextureHandle);

impl std::fmt::Debug for Thumbnail {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Thumbnail")
    }
}

/// The dialog „Melden“ while it is open ([`AppState::feedback_dialog`]).
#[derive(Debug, Clone)]
pub struct FeedbackDialog {
    /// „Fehler“ (at first) or „Wunsch“.
    pub kind: FeedbackKind,
    pub text: String,
    /// „Bild des Fensters anhängen“: ticked if there is a picture.
    pub attach_image: bool,
    /// „Markierte Stellen anhängen (n)“: ticked if there are Stellen.
    pub attach_marks: bool,
    /// The picture of the window as it was before the dialog opened; `None`: none came (no
    /// GPU).
    pub image: Option<Arc<ColorImage>>,
    /// The Stellen of the map or model of the surface shown, as they were when it opened.
    pub marks: Option<Marks>,
    /// The Stelle the report is about, with its name („Stelle 1 · Wand“).
    pub mark: Option<(u32, String)>,
    /// What comes along without the user: version, system, graphics, project, step, selection,
    /// camera.
    pub context: Feedback,
    /// The text field takes the keyboard in the next frame.
    focus: bool,
    thumbnail: Option<Thumbnail>,
}

impl FeedbackDialog {
    /// The dialog with `context`, the picture of the window, the Stellen of the surface shown
    /// and the Stelle it is about; both boxes ticked where there is something to attach.
    pub fn new(
        context: Feedback,
        image: Option<Arc<ColorImage>>,
        marks: Option<Marks>,
        mark: Option<(u32, String)>,
    ) -> FeedbackDialog {
        let marks = marks.filter(|m| !m.marks.is_empty());
        FeedbackDialog {
            kind: FeedbackKind::default(),
            text: String::new(),
            attach_image: image.is_some(),
            attach_marks: marks.is_some(),
            image,
            marks,
            mark,
            context,
            focus: true,
            thumbnail: None,
        }
    }

    /// Whether „Melden“ can save it: something is written.
    pub fn can_send(&self) -> bool {
        !self.text.trim().is_empty()
    }

    /// The report as it is saved now, made at `created`.
    pub fn report(&self, created: String) -> Feedback {
        let mut f = self.context.clone();
        f.kind = self.kind;
        f.text = self.text.trim().to_string();
        f.created = created;
        f.mark = self.mark.as_ref().map(|(nr, _)| *nr);
        f
    }

    /// The picture to attach (ticked).
    pub fn image_to_attach(&self) -> Option<Arc<ColorImage>> {
        self.image.clone().filter(|_| self.attach_image)
    }

    /// The Stellen to attach (ticked).
    pub fn marks_to_attach(&self) -> Option<Marks> {
        self.marks.clone().filter(|_| self.attach_marks)
    }
}

/// The window „Meldungen auf diesem Rechner“ while it is open ([`AppState::feedback_list`]).
#[derive(Debug, Clone, Default)]
pub struct ReportList {
    /// The folder of the reports is being read.
    pub reading: bool,
    /// The reports, the newest first, with their folders.
    pub reports: Vec<(PathBuf, Feedback)>,
    /// Why there is nothing to read (no folder for the reports).
    pub problem: Option<String>,
}

/// „Fehler“, „Wunsch“.
pub fn kind_label(kind: FeedbackKind) -> &'static str {
    match kind {
        FeedbackKind::Bug => text::FEEDBACK_BUG,
        FeedbackKind::Wish => text::FEEDBACK_WISH,
    }
}

/// A picture of at most [`THUMB_PX`] pixels in width, each pixel the mean of four samples of
/// the block of `image` it stands for; `None` for an empty one.
fn thumbnail_of(image: &ColorImage) -> Option<ColorImage> {
    let [w, h] = image.size;
    if w == 0 || h == 0 || image.pixels.len() != w * h {
        return None;
    }
    let tw = THUMB_PX.min(w);
    let th = (h * tw / w).max(1);
    let mut pixels = Vec::with_capacity(tw * th);
    for y in 0..th {
        for x in 0..tw {
            let mut sum = [0u32; 4];
            for (dx, dy) in [(1, 1), (3, 1), (1, 3), (3, 3)] {
                let sx = ((x * 4 + dx) * w / (tw * 4)).min(w - 1);
                let sy = ((y * 4 + dy) * h / (th * 4)).min(h - 1);
                let c = image.pixels[sy * w + sx].to_array();
                for k in 0..4 {
                    sum[k] += u32::from(c[k]);
                }
            }
            let [r, g, b, a] = sum.map(|v| (v / 4) as u8);
            pixels.push(Color32::from_rgba_premultiplied(r, g, b, a));
        }
    }
    Some(ColorImage::new([tw, th], pixels))
}

/// The dialog „Melden“ while [`AppState::feedback_dialog`] holds it. Returns
/// [`Action::SendFeedback`] when „Melden“ was clicked; „Abbrechen“ and Esc close it here.
pub fn show_dialog(ctx: &Context, state: &mut AppState) -> Option<Action> {
    let dialog = state.feedback_dialog.as_mut()?;
    if dialog.thumbnail.is_none() {
        if let Some(thumb) = dialog.image.as_deref().and_then(thumbnail_of) {
            let handle = ctx.load_texture("melden_bild", thumb, TextureOptions::LINEAR);
            dialog.thumbnail = Some(Thumbnail(handle));
        }
    }
    let mut action = None;
    let mut cancel = false;
    let modal = egui::Modal::new(Id::new("melden")).show(ctx, |ui| {
        ui.set_width(WIDTH);
        ui.label(RichText::new(text::FEEDBACK_TITLE).strong().size(15.0));
        ui.horizontal(|ui| {
            let kinds = [
                (FeedbackKind::Bug, text::FEEDBACK_BUG),
                (FeedbackKind::Wish, text::FEEDBACK_WISH),
            ];
            for (kind, label) in kinds {
                let r = ui.add(egui::Button::selectable(dialog.kind == kind, label));
                if r.clicked() {
                    dialog.kind = kind;
                }
            }
        });
        let field = egui::TextEdit::multiline(&mut dialog.text)
            .hint_text(text::FEEDBACK_HINT)
            .desired_rows(4)
            .desired_width(f32::INFINITY);
        let r = ui.add(field);
        if std::mem::take(&mut dialog.focus) {
            r.request_focus();
        }
        ui.horizontal(|ui| {
            if let Some(Thumbnail(handle)) = &dialog.thumbnail {
                let [w, h] = handle.size();
                let size = egui::vec2(THUMB_WIDTH, THUMB_WIDTH * h as f32 / w.max(1) as f32);
                ui.add(egui::Image::new((handle.id(), size)));
            }
            ui.vertical(|ui| attachments(ui, dialog));
        });
        ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
            let label = RichText::new(text::FEEDBACK_SEND).color(DARK.on_accent);
            let send = egui::Button::new(label).fill(DARK.accent);
            let r = ui.add_enabled(dialog.can_send(), send);
            if r.clicked() {
                action = Some(Action::SendFeedback);
            }
            r.on_disabled_hover_text(text::FEEDBACK_NEEDS_TEXT);
            if ui.button(text::CANCEL).clicked() {
                cancel = true;
            }
        });
    });
    // Esc closes it without saving (taken, so nothing below reacts to it as well)
    let esc = modal.is_top_modal
        && !modal.any_popup_open
        && ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::Escape));
    if cancel || esc {
        state.feedback_dialog = None;
    }
    action
}

/// The two boxes, the Stelle reported, and what comes along by itself.
fn attachments(ui: &mut Ui, dialog: &mut FeedbackDialog) {
    let has_image = dialog.image.is_some();
    let image = egui::Checkbox::new(&mut dialog.attach_image, text::FEEDBACK_ATTACH_IMAGE);
    ui.add_enabled(has_image, image)
        .on_disabled_hover_text(text::FEEDBACK_NO_IMAGE);
    let n = dialog.marks.as_ref().map_or(0, |m| m.marks.len());
    let marks = egui::Checkbox::new(&mut dialog.attach_marks, text::feedback_attach_marks(n));
    ui.add_enabled(n > 0, marks)
        .on_disabled_hover_text(text::FEEDBACK_NO_MARKS);
    if let Some((_, label)) = &dialog.mark {
        ui.label(text::feedback_about_mark(label));
    }
    ui.label(
        RichText::new(text::FEEDBACK_CONTEXT)
            .small()
            .color(DARK.text_dim),
    );
}

/// The window „Meldungen auf diesem Rechner“ while [`AppState::feedback_list`] holds it; its ×
/// closes it. Returns „Ordner öffnen“ or „Zur Ansicht“ of a report.
pub fn show_list(ctx: &Context, state: &mut AppState) -> Option<Action> {
    let list = state.feedback_list.as_ref()?;
    let mut open = true;
    let mut action = None;
    egui::Window::new(text::REPORTS_TITLE)
        .id(Id::new("meldungen"))
        .open(&mut open)
        .collapsible(false)
        .resizable(true)
        .default_width(LIST_WIDTH)
        .show(ctx, |ui| {
            if let Some(problem) = &list.problem {
                ui.colored_label(DARK.error, problem);
            } else if list.reading {
                ui.label(text::REPORTS_READING);
            } else if list.reports.is_empty() {
                ui.label(text::reports_none());
            } else {
                egui::ScrollArea::vertical()
                    .id_salt("meldungen_liste")
                    .max_height(LIST_HEIGHT)
                    .auto_shrink([false, true])
                    .show(ui, |ui| {
                        for (folder, report) in &list.reports {
                            action = report_card(ui, folder, report).or(action.take());
                        }
                    });
            }
            ui.add_space(4.0);
            ui.label(
                RichText::new(text::REPORTS_FOOTER)
                    .small()
                    .color(DARK.text_dim),
            );
        });
    if !open {
        state.feedback_list = None;
    }
    action
}

/// A report in the list: kind, time, project and Stelle, its text, „Stand“ and „Antwort“, and
/// its two buttons.
fn report_card(ui: &mut Ui, folder: &Path, f: &Feedback) -> Option<Action> {
    let mut action = None;
    let frame = Frame::new()
        .fill(DARK.surface)
        .corner_radius(CornerRadius::same(4))
        .inner_margin(Margin {
            left: 11,
            right: 8,
            top: 8,
            bottom: 8,
        });
    let shown = frame.show(ui, |ui| {
        ui.set_width(ui.available_width());
        ui.spacing_mut().item_spacing.y = 4.0;
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            ui.label(RichText::new(kind_label(f.kind)).strong());
            ui.label(RichText::new(text::REPORT_DOT).color(DARK.text_dim));
            ui.label(text::report_time(&f.created));
            if let Some(project) = &f.project {
                ui.label(RichText::new(text::REPORT_DOT).color(DARK.text_dim));
                ui.label(text::report_project(&project.name));
            }
            if let Some(nr) = f.mark {
                ui.label(RichText::new(text::REPORT_DOT).color(DARK.text_dim));
                ui.label(text::report_mark(nr));
            }
        });
        if !f.text.is_empty() {
            ui.label(&f.text);
        }
        let color = if f.is_open() { DARK.warn } else { DARK.ok };
        ui.label(RichText::new(&f.status).small().color(color));
        if !f.answer.trim().is_empty() {
            ui.label(&f.answer);
        }
        ui.horizontal(|ui| {
            if ui.button(text::REPORT_OPEN_FOLDER).clicked() {
                action = Some(Action::OpenReportFolder(folder.to_path_buf()));
            }
            let view = f.project.is_some() || f.camera.is_some();
            let r = ui.add_enabled(view, egui::Button::new(text::REPORT_TO_VIEW));
            if r.clicked() {
                action = Some(Action::ReportView(folder.to_path_buf()));
            }
            r.on_hover_text(text::REPORT_TO_VIEW_HINT)
                .on_disabled_hover_text(text::REPORT_NO_VIEW);
        });
    });
    let rect = shown.response.rect;
    let stripe = Rect::from_min_size(rect.min, egui::vec2(STRIPE, rect.height()));
    let color = match f.kind {
        FeedbackKind::Bug => DARK.error,
        FeedbackKind::Wish => DARK.ok,
    };
    let corners = CornerRadius {
        nw: 4,
        sw: 4,
        ne: 0,
        se: 0,
    };
    ui.painter().rect_filled(stripe, corners, color);
    action
}

/// What is selected in the field, for a report: its kind, how the field names it, its folder
/// (a cloud: its file), a scan's id, a state's number; `None` without a selection.
pub fn selection_record(state: &AppState) -> Option<FeedbackSelection> {
    let named = |p: &Path| {
        p.file_name().map_or_else(
            || p.display().to_string(),
            |n| n.to_string_lossy().into_owned(),
        )
    };
    let mut record = match &state.selection {
        Selection::None => return None,
        Selection::Scan(id) => {
            let name = state
                .scan_node(id)
                .map_or_else(|| id.clone(), |s| s.name.clone());
            let path = state
                .project
                .as_ref()
                .and_then(|p| p.scan(id))
                .map(|e| e.path.clone());
            let mut record = FeedbackSelection::new(SCAN, &name, path);
            record.scan = Some(id.clone());
            record
        }
        Selection::Run(dir) => {
            let name = run_text(state, dir).unwrap_or_else(|| named(dir));
            FeedbackSelection::new(RUN, &name, Some(dir.clone()))
        }
        Selection::Map(dir) => {
            let name = state.map(dir).map_or_else(
                || named(dir),
                |(s, r, m)| text::joined_name(&[&m.label, &r.label, &s.name]),
            );
            FeedbackSelection::new(MAP, &name, Some(dir.clone()))
        }
        Selection::Cloud(path) => FeedbackSelection::new(CLOUD, &named(path), Some(path.clone())),
        Selection::Surface(dir) => {
            let name = surface_text(state, dir).unwrap_or_else(|| named(dir));
            FeedbackSelection::new(SURFACE, &name, Some(dir.clone()))
        }
        Selection::History { run, nr } => {
            let of = run_text(state, run).unwrap_or_else(|| named(run));
            let name = text::joined_name(&[&text::state_name(*nr), &of]);
            let mut record = FeedbackSelection::new(STATE, &name, Some(run.clone()));
            record.nr = Some(*nr);
            record
        }
        Selection::Model(dir) => {
            let name = state
                .model(dir)
                .map_or_else(|| named(dir), |m| m.label.clone());
            FeedbackSelection::new(MODEL, &name, Some(dir.clone()))
        }
    };
    record.text = record.text.trim().to_string();
    Some(record)
}

/// „Lauf … · Halle“.
fn run_text(state: &AppState, dir: &Path) -> Option<String> {
    let (scan, run) = state.run(dir)?;
    Some(text::joined_name(&[&run.label, &scan.name]))
}

/// „Oberfläche 001 · Karte 001 · Halle“, „Oberfläche 001 · Modell 001“.
fn surface_text(state: &AppState, dir: &Path) -> Option<String> {
    let of_map = state.scans.iter().find_map(|scan| {
        let map = scan
            .runs
            .iter()
            .flat_map(|r| &r.maps)
            .find(|m| m.surfaces.iter().any(|s| s.dir == dir))?;
        let surface = map.surfaces.iter().find(|s| s.dir == dir)?;
        Some(text::joined_name(&[&surface.label, &map.label, &scan.name]))
    });
    of_map.or_else(|| {
        state.models.iter().find_map(|model| {
            let surface = model.surfaces.iter().find(|s| s.dir == dir)?;
            Some(text::joined_name(&[&surface.label, &model.label]))
        })
    })
}

/// The steps as `schritt` of a report names them (they stay, whatever the window calls them).
const STEP_WORDS: [(Step, &str); 3] = [
    (Step::Daten, "Daten"),
    (Step::Bilder, "Bilder"),
    (Step::Planung, "Planung"),
];

/// `schritt` of a report: „Daten“, „Bilder“, „Planung“.
pub fn step_word(step: Step) -> &'static str {
    STEP_WORDS
        .iter()
        .find(|(s, _)| *s == step)
        .map_or("Daten", |(_, w)| w)
}

/// The step `schritt` of a report names; `None` for another word.
pub fn step_of(word: &str) -> Option<Step> {
    STEP_WORDS
        .iter()
        .find(|(_, w)| *w == word.trim())
        .map(|(s, _)| *s)
}

/// The selection a report kept, as the field selects it; `None` for a kind it does not know or
/// one without what it needs (a path, a scan id, a number).
pub fn selection_of(record: &FeedbackSelection) -> Option<Selection> {
    let path = || record.path.clone();
    Some(match record.kind.as_str() {
        SCAN => Selection::Scan(record.scan.clone()?),
        RUN => Selection::Run(path()?),
        MAP => Selection::Map(path()?),
        CLOUD => Selection::Cloud(path()?),
        SURFACE => Selection::Surface(path()?),
        STATE => Selection::History {
            run: path()?,
            nr: record.nr?,
        },
        MODEL => Selection::Model(path()?),
        _ => return None,
    })
}

/// Whether the tree of the project shown has `selection`: a report may name a folder that was
/// deleted since.
pub fn selection_known(state: &AppState, selection: &Selection) -> bool {
    match selection {
        Selection::None => true,
        Selection::Scan(id) => state.scan_node(id).is_some(),
        Selection::Run(dir) => state.run(dir).is_some(),
        Selection::Map(dir) => state.map(dir).is_some(),
        Selection::Cloud(path) => {
            let of_run = state.scans.iter().flat_map(|s| &s.runs).any(|r| {
                r.cloud.as_ref() == Some(path) || r.maps.iter().any(|m| m.clouds.contains(path))
            });
            of_run
                || state
                    .models
                    .iter()
                    .any(|m| m.dir.join(driftless_steps::merge::MODEL_LAS) == *path)
        }
        Selection::Surface(dir) => state.all_surfaces().any(|s| s.dir == *dir),
        Selection::History { run, nr } => state
            .run(run)
            .is_some_and(|(_, r)| r.history.iter().any(|h| h.nr == *nr)),
        Selection::Model(dir) => state.model(dir).is_some(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn feedback_thumbnail_keeps_the_aspect() {
        let pixels = vec![Color32::from_rgb(200, 100, 50); 1400 * 900];
        let image = ColorImage::new([1400, 900], pixels);
        let thumb = thumbnail_of(&image).unwrap();
        assert_eq!(thumb.size, [THUMB_PX, THUMB_PX * 900 / 1400]);
        assert_eq!(thumb.pixels[0], Color32::from_rgb(200, 100, 50));
        // a small one is not made larger
        let small = ColorImage::new([8, 4], vec![Color32::WHITE; 32]);
        assert_eq!(thumbnail_of(&small).unwrap().size, [8, 4]);
        assert!(thumbnail_of(&ColorImage::new([0, 0], Vec::new())).is_none());
    }

    #[test]
    fn feedback_selection_goes_back_and_forth() {
        let cases = [
            Selection::Scan("20261020173822".into()),
            Selection::Run(PathBuf::from("/p/scans/a/slam-001")),
            Selection::Map(PathBuf::from("/p/scans/a/slam-001/karte-001")),
            Selection::Cloud(PathBuf::from("/p/scans/a/slam-001/karte-001/color.las")),
            Selection::Surface(PathBuf::from(
                "/p/scans/a/slam-001/karte-001/oberflaeche-001",
            )),
            Selection::History {
                run: PathBuf::from("/p/scans/a/slam-001"),
                nr: 3,
            },
            Selection::Model(PathBuf::from("/p/modelle/modell-001")),
        ];
        for selection in cases {
            let state = AppState {
                selection: selection.clone(),
                ..AppState::default()
            };
            let record = selection_record(&state).unwrap();
            assert!(!record.text.is_empty(), "{record:?}");
            assert_eq!(selection_of(&record), Some(selection));
        }
        assert_eq!(selection_record(&AppState::default()), None);
        let unknown = FeedbackSelection::new("bild", "x", Some(PathBuf::from("/x")));
        assert_eq!(selection_of(&unknown), None);
        let no_path = FeedbackSelection::new(SURFACE, "x", None);
        assert_eq!(selection_of(&no_path), None);
    }

    #[test]
    fn feedback_steps_have_their_words() {
        for step in [Step::Daten, Step::Bilder, Step::Planung] {
            assert_eq!(step_of(step_word(step)), Some(step));
        }
        assert_eq!(step_word(Step::Bilder), "Bilder");
        assert_eq!(step_of("Ansicht"), None);
    }
}
