//! What a touch-up step changes on a surface ([`SurfacePatch`]), applied in order to the
//! computed surface and kept next to it.
//!
//! A patch moves welded vertices („Glätten“), takes faces off the page that holds them and
//! adds pages that hold them anew („Bilder neu einspielen“). Applied in order, each patch
//! 1. moves every copy of its vertices: on the pages of the computed surface and on the pages
//!    earlier patches added;
//! 2. takes its faces off the page that holds them at that point: the computed page, or the
//!    page of an earlier patch – so where two patches take the same face, the later one wins;
//! 3. appends its pages after those of the computed surface and of the earlier patches.
//!
//! Nothing is packed anew. A computed page keeps its corners, texture coordinates and image:
//! it only loses the triangles a patch took off (their corners stay, unused) and its moved
//! corners get their new places – if a triangle it keeps uses one. A page whose moved
//! corners are all unused (the faces around them went to an added page: smoothed, then
//! re-textured) looks as before with its own corners, so it keeps them and only its triangles
//! change; the view then swaps its triangles only. A page's image is the same shared bytes,
//! never decoded or encoded, so `write_glb` writes an untouched page byte for byte as before.
//! An added page that a later patch took every face off (the same Stelle re-textured again)
//! is left out. [`apply_patches`] gives the touched-up surface (for the export);
//! [`page_edits`] gives only what changed (for the view, which swaps the buffers of the
//! touched pages and adds the new ones).
//!
//! The patches of a surface are kept in one file, [`PATCHES_FILE`], in a folder of its own
//! ([`write_patches`], [`read_patches`]). It is written as `<name>.tmp` and then renamed, so
//! there is never half a file, and the same patches give the same bytes. Little-endian:
//!
//! ```text
//! "DLTU"  u32 format version  u64 patch count
//! per patch:  u64 n, n × (u32 vertex, f64 x y z)               moved
//!             u64 n, n × u32 face                               removed_faces
//!             u64 n, per page:  u64 n, n × (f64 x y z, f32 u v)   corners
//!                               u64 n, n × (u32 a b c, u32 face)  triangles
//!                               u64 n, n bytes                    JPEG
//! u64 FNV-1a of all bytes before
//! ```
//!
//! Every count is checked against the bytes left before anything is allocated, and every
//! patch read must be consistent in itself (see [`SurfacePatch`]).

use super::{EditError, WeldedSurface};
use crate::content_hash;
use driftless_export::mesh::{ImageKind, Mesh, MeshPage, PageImage};
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;
use std::sync::Arc;

/// The file of the patches in the folder [`write_patches`] writes into.
pub const PATCHES_FILE: &str = "aenderungen.bin";

/// The first bytes of [`PATCHES_FILE`].
const MAGIC: &[u8; 4] = b"DLTU";

/// Version of the layout of [`PATCHES_FILE`]. What the steps compute has a version of its
/// own, [`super::EDIT_VERSION`].
const FORMAT: u32 = 1;

/// Bytes of a moved vertex, a corner and a triangle in the file.
const MOVED_BYTES: usize = 4 + 3 * 8;
const CORNER_BYTES: usize = 3 * 8 + 2 * 4;
const TRIANGLE_BYTES: usize = 4 * 4;
/// The fewest bytes of a patch and of a page in the file: their three counts.
const PATCH_MIN_BYTES: usize = 3 * 8;
const PAGE_MIN_BYTES: usize = 3 * 8;

/// No welded vertex.
const NONE: u32 = u32::MAX;

/// The changes of one touch-up step. Ids are those of the [`WeldedSurface`] of the computed
/// surface (faces and vertices keep them through every step). A patch is consistent in
/// itself: `moved` sorted by vertex and `removed_faces` sorted, each once, positions finite;
/// every page with one texture coordinate per corner, triangles within its corners, a face
/// per triangle and a JPEG; every face its pages hold among `removed_faces`, and held once.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SurfacePatch {
    /// Welded vertices and their new places, as [`super::smooth_region`] gives them.
    pub moved: Vec<(u32, [f64; 3])>,
    /// Welded faces taken off the page that holds them.
    pub removed_faces: Vec<u32>,
    /// New pages, holding faces of `removed_faces` anew, their corners at the places after
    /// `moved` (the patch's own moves do not reach them).
    pub pages: Vec<PatchPage>,
}

/// A page a patch adds. Triangle `t` is the welded face `faces[t]`: its corner `k` is a copy
/// of the face's corner `k` (same order, same winding), so a later patch that moves the
/// vertex moves this copy too.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PatchPage {
    /// Z up, metres, as [`MeshPage::positions`].
    pub positions: Vec<[f64; 3]>,
    /// One per corner; `v` points down, as [`MeshPage::uvs`].
    pub uvs: Vec<[f32; 2]>,
    pub indices: Vec<[u32; 3]>,
    /// The welded face of every triangle.
    pub faces: Vec<u32>,
    /// The page's image, encoded.
    pub jpeg: Arc<[u8]>,
}

/// How the patches changed one page of the computed surface ([`page_edits`]).
#[derive(Clone, Debug, PartialEq)]
pub struct PageEdit {
    /// The page of the computed surface.
    pub page: u32,
    /// All corners of the page, if a patch moved one that a triangle the page keeps uses.
    pub positions: Option<Vec<[f64; 3]>>,
    /// The triangles of the page (indices into its `indices`) the patches took off, sorted.
    pub removed: Vec<u32>,
}

impl SurfacePatch {
    /// Whether the patch is consistent in itself (see [`SurfacePatch`]); if not, why (German).
    fn check(&self) -> Result<(), String> {
        if !self.moved.windows(2).all(|w| w[0].0 < w[1].0) {
            return Err("verschobene Punkte nicht aufsteigend".into());
        }
        if !self.moved.iter().all(|(_, p)| finite(p)) {
            return Err("ungültige Lage eines Punkts".into());
        }
        if !self.removed_faces.windows(2).all(|w| w[0] < w[1]) {
            return Err("Flächen nicht aufsteigend".into());
        }
        let mut held = Vec::new();
        for (i, page) in self.pages.iter().enumerate() {
            page.check(&self.removed_faces)
                .map_err(|why| format!("Seite {i}: {why}"))?;
            held.extend_from_slice(&page.faces);
        }
        held.sort_unstable();
        match held.windows(2).find(|w| w[0] == w[1]) {
            Some(w) => Err(format!("Fläche {} zweimal auf den Seiten", w[0])),
            None => Ok(()),
        }
    }
}

impl PatchPage {
    /// Whether the page is consistent in itself and holds only faces of `removed` (sorted).
    fn check(&self, removed: &[u32]) -> Result<(), String> {
        let n = self.positions.len();
        if u32::try_from(n).is_err() {
            return Err(format!("{n} Ecken sind zu viele"));
        }
        if self.uvs.len() != n {
            return Err(format!(
                "{} Texturkoordinaten für {n} Ecken",
                self.uvs.len()
            ));
        }
        if !self.positions.iter().all(finite) {
            return Err("ungültige Lage einer Ecke".into());
        }
        if !self.uvs.iter().flatten().all(|c| c.is_finite()) {
            return Err("ungültige Texturkoordinate".into());
        }
        if self.faces.len() != self.indices.len() {
            return Err(format!(
                "{} Flächen für {} Dreiecke",
                self.faces.len(),
                self.indices.len()
            ));
        }
        if let Some(t) = self
            .indices
            .iter()
            .position(|tri| tri.iter().any(|&c| c as usize >= n))
        {
            return Err(format!(
                "Dreieck {t} verweist auf eine Ecke, die es nicht gibt"
            ));
        }
        if let Some(f) = self
            .faces
            .iter()
            .find(|f| removed.binary_search(f).is_err())
        {
            return Err(format!("Fläche {f} wird nicht ersetzt"));
        }
        if ImageKind::sniff(&self.jpeg) != Some(ImageKind::Jpeg) {
            return Err("Bild ist kein JPEG".into());
        }
        Ok(())
    }
}

fn finite(p: &[f64; 3]) -> bool {
    p.iter().all(|c| c.is_finite())
}

/// Whether the `patches` can be applied to the surface welded as `s` ([`apply_patches`],
/// [`page_edits`]): each is consistent in itself (see [`SurfacePatch`]), names only vertices
/// and faces of `s`, and every corner of a page it adds is a copy of one vertex of `s` (corner
/// `k` of a triangle is one of its face's corner `k`). Patches computed for this surface always
/// are; a file of another surface, or a crafted one, need not be, also with a valid checksum
/// ([`read_patches`] checks each patch in itself only): its patches are not this surface's.
pub fn patches_fit(s: &WeldedSurface, patches: &[SurfacePatch]) -> bool {
    let (vertices, faces) = (s.positions().len(), s.face_count());
    patches.iter().all(|p| {
        p.check().is_ok()
            && p.moved.last().is_none_or(|&(v, _)| (v as usize) < vertices)
            && p.removed_faces.last().is_none_or(|&f| (f as usize) < faces)
            && p.pages.iter().all(|page| {
                // the faces of a page are among the removed ones (checked): faces of `s`
                let mut vertex = vec![NONE; page.positions.len()];
                page.indices.iter().zip(&page.faces).all(|(tri, &f)| {
                    tri.iter().zip(&s.faces()[f as usize]).all(|(&c, &v)| {
                        let slot = &mut vertex[c as usize];
                        let one = *slot == NONE || *slot == v;
                        *slot = v;
                        one
                    })
                })
            })
    })
}

/// Panics unless every patch is consistent in itself.
fn check_all(patches: &[SurfacePatch]) {
    for (i, patch) in patches.iter().enumerate() {
        if let Err(why) = patch.check() {
            panic!("patch {i}: {why}");
        }
    }
}

/// The computed surface `mesh` with the `patches` applied in order (see the module docs): its
/// pages, changed where the patches changed them, then the pages the patches added that still
/// hold a face. `s` is the welded surface of `mesh` (its positions do not matter: those of
/// the patches count).
///
/// # Panics
/// If `s` is not the welded surface of `mesh`, or the patches do not fit it ([`patches_fit`]:
/// a patch is not consistent in itself, names a vertex or face `s` does not have, or a corner
/// of an added page is a copy of two vertices).
pub fn apply_patches(mesh: &Mesh, s: &WeldedSurface, patches: &[SurfacePatch]) -> Mesh {
    let (edits, added) = page_edits(s, mesh, patches);
    let mut edits = edits.into_iter().peekable();
    let mut pages = Vec::with_capacity(mesh.pages.len() + added.len());
    for (p, page) in mesh.pages.iter().enumerate() {
        pages.push(match edits.next_if(|e| e.page as usize == p) {
            None => page.clone(),
            Some(edit) => MeshPage {
                positions: edit.positions.unwrap_or_else(|| page.positions.clone()),
                uvs: page.uvs.clone(),
                indices: kept(&page.indices, &edit.removed).copied().collect(),
                image: page.image.clone(),
                color: page.color,
            },
        });
    }
    pages.extend(added);
    Mesh { pages }
}

/// The triangles of `indices` that are not `removed` (sorted), in their order.
fn kept<'a>(
    indices: &'a [[u32; 3]],
    removed: &'a [u32],
) -> impl Iterator<Item = &'a [u32; 3]> + 'a {
    let mut removed = removed.iter().copied().peekable();
    indices
        .iter()
        .enumerate()
        .filter(move |&(t, _)| removed.next_if_eq(&(t as u32)).is_none())
        .map(|(_, tri)| tri)
}

/// Whether a triangle of `page` that is not `removed` (sorted) uses a corner whose place in
/// `positions` differs from the page's own.
fn moved_in_sight(page: &MeshPage, positions: &[[f64; 3]], removed: &[u32]) -> bool {
    let moved: Vec<bool> = page
        .positions
        .iter()
        .zip(positions)
        .map(|(a, b)| a.map(f64::to_bits) != b.map(f64::to_bits))
        .collect();
    kept(&page.indices, removed)
        .flatten()
        .any(|&c| moved[c as usize])
}

/// What the `patches` change on the computed surface `mesh` (welded: `s`), for the view: the
/// pages they changed, by page, and the pages they added that still hold a face, in order.
/// [`apply_patches`] gives the same pages: a changed page with `positions` (if given) and
/// without its `removed` triangles, then the added pages.
///
/// # Panics
/// As [`apply_patches`].
pub fn page_edits(
    s: &WeldedSurface,
    mesh: &Mesh,
    patches: &[SurfacePatch],
) -> (Vec<PageEdit>, Vec<MeshPage>) {
    Applied::new(mesh, s, patches).finish()
}

/// The patches applied one after the other, kept as changes to the computed pages and as
/// added pages (see the module docs).
struct Applied<'a> {
    mesh: &'a Mesh,
    s: &'a WeldedSurface,
    /// The computed pages with a moved corner: all their corners.
    positions: BTreeMap<u32, Vec<[f64; 3]>>,
    /// The triangles taken off the computed pages, in the order taken.
    removed: BTreeMap<u32, Vec<u32>>,
    /// The faces that left their computed page: the added page and triangle holding them now,
    /// or none.
    elsewhere: BTreeMap<u32, Option<(u32, u32)>>,
    /// The pages the patches added, in order.
    added: Vec<Added>,
}

/// A page a patch added, as the later patches leave it.
struct Added {
    page: MeshPage,
    /// The welded vertex every corner is a copy of ([`NONE`]: no triangle uses the corner).
    vertex: Vec<u32>,
    /// Whether each triangle still holds its face.
    kept: Vec<bool>,
}

impl<'a> Applied<'a> {
    fn new(mesh: &'a Mesh, s: &'a WeldedSurface, patches: &[SurfacePatch]) -> Applied<'a> {
        let other = "the welded surface of another mesh";
        assert_eq!(mesh.triangles(), s.face_count(), "{other}");
        let mut first = 0;
        for (p, page) in mesh.pages.iter().enumerate() {
            if !page.indices.is_empty() {
                assert_eq!(s.face_origin(first), (p as u32, 0), "{other}");
            }
            first += page.indices.len() as u32;
        }
        check_all(patches);
        let mut applied = Applied {
            mesh,
            s,
            positions: BTreeMap::new(),
            removed: BTreeMap::new(),
            elsewhere: BTreeMap::new(),
            added: Vec::new(),
        };
        for patch in patches {
            applied.apply(patch);
        }
        applied
    }

    fn apply(&mut self, patch: &SurfacePatch) {
        let (mesh, s) = (self.mesh, self.s);
        if let Some(&(v, _)) = patch.moved.last() {
            let n = s.positions().len();
            assert!((v as usize) < n, "vertex {v} of {n}");
        }
        if let Some(&f) = patch.removed_faces.last() {
            let n = s.face_count();
            assert!((f as usize) < n, "face {f} of {n}");
        }

        // 1. Every copy of the moved vertices: on the computed pages, then on the added ones.
        for &(v, p) in &patch.moved {
            for &(page, corner) in s.copies(v) {
                let positions = self
                    .positions
                    .entry(page)
                    .or_insert_with(|| mesh.pages[page as usize].positions.clone());
                positions[corner as usize] = p;
            }
        }
        if !patch.moved.is_empty() {
            for added in &mut self.added {
                for (corner, &v) in added.vertex.iter().enumerate() {
                    if let Ok(m) = patch.moved.binary_search_by_key(&v, |m| m.0) {
                        added.page.positions[corner] = patch.moved[m].1;
                    }
                }
            }
        }

        // 2. The faces off the page that holds them now.
        for &f in &patch.removed_faces {
            match self.elsewhere.insert(f, None) {
                None => {
                    let (page, t) = s.face_origin(f);
                    self.removed.entry(page).or_default().push(t);
                }
                Some(Some((page, t))) => self.added[page as usize].kept[t as usize] = false,
                Some(None) => {}
            }
        }

        // 3. The new pages after all others; they hold their faces now.
        for page in &patch.pages {
            let at = self.added.len() as u32;
            let mut vertex = vec![NONE; page.positions.len()];
            for (t, (tri, &f)) in page.indices.iter().zip(&page.faces).enumerate() {
                for (&corner, &v) in tri.iter().zip(&s.faces()[f as usize]) {
                    let slot = &mut vertex[corner as usize];
                    assert!(
                        *slot == NONE || *slot == v,
                        "corner {corner} of an added page is a copy of vertices {} and {v}",
                        *slot
                    );
                    *slot = v;
                }
                self.elsewhere.insert(f, Some((at, t as u32)));
            }
            self.added.push(Added {
                page: MeshPage {
                    positions: page.positions.clone(),
                    uvs: page.uvs.clone(),
                    indices: page.indices.clone(),
                    image: Some(PageImage {
                        kind: ImageKind::Jpeg,
                        bytes: Arc::clone(&page.jpeg),
                    }),
                    // As a page with an image reads back from a glb.
                    color: [255; 3],
                },
                vertex,
                kept: vec![true; page.indices.len()],
            });
        }
    }

    /// The changed computed pages, by page, and the added pages as the patches left them,
    /// without those that hold no face any more.
    fn finish(self) -> (Vec<PageEdit>, Vec<MeshPage>) {
        let Applied {
            mesh,
            mut positions,
            removed,
            added,
            ..
        } = self;
        let mut edits: Vec<PageEdit> = removed
            .into_iter()
            .map(|(page, mut removed)| {
                removed.sort_unstable();
                PageEdit {
                    page,
                    positions: positions.remove(&page),
                    removed,
                }
            })
            .collect();
        edits.extend(positions.into_iter().map(|(page, positions)| PageEdit {
            page,
            positions: Some(positions),
            removed: Vec::new(),
        }));
        edits.sort_unstable_by_key(|e| e.page);
        // Moved corners that no kept triangle uses do not show: such a page keeps its own.
        for edit in &mut edits {
            let page = &mesh.pages[edit.page as usize];
            if let Some(positions) = &edit.positions {
                if !moved_in_sight(page, positions, &edit.removed) {
                    edit.positions = None;
                }
            }
        }
        edits.retain(|e| e.positions.is_some() || !e.removed.is_empty());
        let added = added
            .into_iter()
            .map(Added::into_page)
            .filter(|page| !page.indices.is_empty())
            .collect();
        (edits, added)
    }
}

impl Added {
    /// The page without the triangles that lost their face.
    fn into_page(self) -> MeshPage {
        let mut page = self.page;
        if self.kept.contains(&false) {
            page.indices = page
                .indices
                .iter()
                .zip(&self.kept)
                .filter(|&(_, &kept)| kept)
                .map(|(&tri, _)| tri)
                .collect();
        }
        page
    }
}

/// Writes `patches` as [`PATCHES_FILE`] into the folder `dir` (made if missing), through
/// `<name>.tmp` (see the module docs). The same patches give the same bytes.
///
/// # Errors
/// [`EditError::Unwritable`] if the folder or the file cannot be written; no `.tmp` stays.
///
/// # Panics
/// If a patch is not consistent in itself (see [`SurfacePatch`]).
pub fn write_patches(dir: &Path, patches: &[SurfacePatch]) -> Result<(), EditError> {
    check_all(patches);
    let unwritable = |path: &Path| {
        let path = path.to_path_buf();
        move |source| EditError::Unwritable { path, source }
    };
    fs::create_dir_all(dir).map_err(unwritable(dir))?;
    let path = dir.join(PATCHES_FILE);
    driftless_io::atomic_write(&path, &encode(patches)).map_err(unwritable(&path))
}

/// Reads the patches [`write_patches`] wrote into `dir`.
///
/// # Errors
/// [`EditError::Unreadable`] if the file is missing or cannot be read, is no file of
/// patches or of another version, damaged, or holds a patch that is not consistent in itself.
pub fn read_patches(dir: &Path) -> Result<Vec<SurfacePatch>, EditError> {
    let path = dir.join(PATCHES_FILE);
    fs::read(&path)
        .map_err(|e| e.to_string())
        .and_then(|bytes| decode(&bytes))
        .map_err(|why| EditError::Unreadable { path, why })
}

/// The bytes of [`PATCHES_FILE`] for `patches` (see the module docs), not checked.
fn encode(patches: &[SurfacePatch]) -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(MAGIC);
    out.extend_from_slice(&FORMAT.to_le_bytes());
    put_count(&mut out, patches.len());
    for patch in patches {
        put_count(&mut out, patch.moved.len());
        for (v, p) in &patch.moved {
            out.extend_from_slice(&v.to_le_bytes());
            put_point(&mut out, p);
        }
        put_count(&mut out, patch.removed_faces.len());
        for f in &patch.removed_faces {
            out.extend_from_slice(&f.to_le_bytes());
        }
        put_count(&mut out, patch.pages.len());
        for page in &patch.pages {
            put_count(&mut out, page.positions.len());
            for (p, uv) in page.positions.iter().zip(&page.uvs) {
                put_point(&mut out, p);
                for c in uv {
                    out.extend_from_slice(&c.to_le_bytes());
                }
            }
            put_count(&mut out, page.indices.len());
            for (tri, f) in page.indices.iter().zip(&page.faces) {
                for c in tri.iter().chain([f]) {
                    out.extend_from_slice(&c.to_le_bytes());
                }
            }
            put_count(&mut out, page.jpeg.len());
            out.extend_from_slice(&page.jpeg);
        }
    }
    let sum = content_hash(&out);
    out.extend_from_slice(&sum.to_le_bytes());
    out
}

fn put_count(out: &mut Vec<u8>, n: usize) {
    out.extend_from_slice(&(n as u64).to_le_bytes());
}

fn put_point(out: &mut Vec<u8>, p: &[f64; 3]) {
    for c in p {
        out.extend_from_slice(&c.to_le_bytes());
    }
}

/// The patches in the bytes of a [`PATCHES_FILE`]; if there are none, why (German).
fn decode(bytes: &[u8]) -> Result<Vec<SurfacePatch>, String> {
    if !bytes.starts_with(MAGIC) {
        return Err("keine Driftless-Nacharbeit".into());
    }
    let cut = || "abgeschnitten".to_string();
    let version: [u8; 4] = bytes
        .get(4..8)
        .ok_or_else(cut)?
        .try_into()
        .expect("4 bytes");
    let version = u32::from_le_bytes(version);
    if version != FORMAT {
        return Err(format!("Version {version}, erwartet {FORMAT}"));
    }
    // Magic, version, patch count and the sum at the end.
    if bytes.len() < 4 + 4 + 8 + 8 {
        return Err(cut());
    }
    let (body, sum) = bytes.split_last_chunk::<8>().expect("8 bytes");
    if content_hash(body) != u64::from_le_bytes(*sum) {
        return Err("beschädigt (Prüfsumme stimmt nicht)".into());
    }

    let mut r = Reader { rest: &body[8..] };
    let n = r.count(PATCH_MIN_BYTES)?;
    let mut patches = Vec::with_capacity(n);
    for _ in 0..n {
        let n = r.count(MOVED_BYTES)?;
        let mut moved = Vec::with_capacity(n);
        for _ in 0..n {
            moved.push((r.u32()?, r.point()?));
        }
        let n = r.count(4)?;
        let mut removed_faces = Vec::with_capacity(n);
        for _ in 0..n {
            removed_faces.push(r.u32()?);
        }
        let n = r.count(PAGE_MIN_BYTES)?;
        let mut pages = Vec::with_capacity(n);
        for _ in 0..n {
            let corners = r.count(CORNER_BYTES)?;
            let mut positions = Vec::with_capacity(corners);
            let mut uvs = Vec::with_capacity(corners);
            for _ in 0..corners {
                positions.push(r.point()?);
                uvs.push([r.f32()?, r.f32()?]);
            }
            let triangles = r.count(TRIANGLE_BYTES)?;
            let mut indices = Vec::with_capacity(triangles);
            let mut faces = Vec::with_capacity(triangles);
            for _ in 0..triangles {
                indices.push([r.u32()?, r.u32()?, r.u32()?]);
                faces.push(r.u32()?);
            }
            let len = r.count(1)?;
            let jpeg = r.bytes(len)?.into();
            pages.push(PatchPage {
                positions,
                uvs,
                indices,
                faces,
                jpeg,
            });
        }
        patches.push(SurfacePatch {
            moved,
            removed_faces,
            pages,
        });
    }
    if !r.rest.is_empty() {
        return Err("beschädigt (Daten nach dem Ende)".into());
    }
    for patch in &patches {
        patch.check().map_err(|why| format!("beschädigt ({why})"))?;
    }
    Ok(patches)
}

/// Reads the body of a [`PATCHES_FILE`] front to back.
struct Reader<'a> {
    rest: &'a [u8],
}

impl<'a> Reader<'a> {
    fn bytes(&mut self, n: usize) -> Result<&'a [u8], String> {
        if n > self.rest.len() {
            return Err("abgeschnitten".into());
        }
        let (head, rest) = self.rest.split_at(n);
        self.rest = rest;
        Ok(head)
    }

    fn array<const N: usize>(&mut self) -> Result<[u8; N], String> {
        Ok(self.bytes(N)?.try_into().expect("N bytes"))
    }

    fn u32(&mut self) -> Result<u32, String> {
        Ok(u32::from_le_bytes(self.array()?))
    }

    fn f32(&mut self) -> Result<f32, String> {
        Ok(f32::from_le_bytes(self.array()?))
    }

    fn f64(&mut self) -> Result<f64, String> {
        Ok(f64::from_le_bytes(self.array()?))
    }

    fn point(&mut self) -> Result<[f64; 3], String> {
        Ok([self.f64()?, self.f64()?, self.f64()?])
    }

    /// A count of records of at least `size` bytes each, which must fit into the bytes left.
    fn count(&mut self, size: usize) -> Result<usize, String> {
        let n = u64::from_le_bytes(self.array()?);
        match n.checked_mul(size as u64) {
            Some(bytes) if bytes <= self.rest.len() as u64 => Ok(n as usize),
            _ => Err("abgeschnitten".into()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::surface::testutil::{grid, paged};
    use driftless_export::mesh::{write_glb, ImageKind, PageImage};
    use driftless_io::progress::Progress;
    use driftless_io::testdir::TestDir;
    use image::codecs::jpeg::JpegEncoder;
    use image::ExtendedColorType;
    use std::collections::{BTreeMap, BTreeSet};
    use std::fs;

    /// Squares of the test surface (m).
    const STEP: f64 = 0.05;

    fn temp_dir(name: &str) -> TestDir {
        TestDir::new(format!(
            "driftless-edit-patch-{}-{name}",
            std::process::id()
        ))
    }

    fn f32_exact(p: [f64; 3]) -> [f64; 3] {
        p.map(|c| f64::from(c as f32))
    }

    fn bits(p: [f64; 3]) -> [u64; 3] {
        p.map(f64::to_bits)
    }

    /// Vertex `(i, j)` of the test surface.
    fn at(i: usize, j: usize) -> [f64; 3] {
        f32_exact([
            i as f64 * STEP,
            j as f64 * STEP,
            0.002 * ((i * j) % 7) as f64,
        ])
    }

    /// An 8 × 8 JPEG in one colour.
    fn jpeg(rgb: [u8; 3]) -> Arc<[u8]> {
        let data: Vec<u8> = (0..64).flat_map(|_| rgb).collect();
        let mut out = Vec::new();
        JpegEncoder::new_with_quality(&mut out, 90)
            .encode(&data, 8, 8, ExtendedColorType::Rgb8)
            .unwrap();
        out.into()
    }

    /// A gently bumpy floor of 24 × 10 squares of 5 cm, paged as the surface step does it:
    /// charts of 40 faces (two rows of squares), dealt to 3 pages, every page with texture
    /// coordinates and an image of its own. A vertex between the two rows of a chart lies on
    /// one page, one between two charts on two.
    fn surface() -> (Mesh, WeldedSurface) {
        let (positions, indices) = grid([24, 10], at);
        let mut mesh = paged(&positions, &indices, 40, 3);
        for (p, page) in mesh.pages.iter_mut().enumerate() {
            page.uvs = page
                .positions
                .iter()
                .map(|q| [q[0] as f32, 2.0 * q[1] as f32])
                .collect();
            page.image = Some(PageImage {
                kind: ImageKind::Jpeg,
                bytes: jpeg([60 * p as u8, 120, 200]),
            });
            page.color = [255; 3];
        }
        let s = WeldedSurface::from_mesh(&mesh);
        (mesh, s)
    }

    /// The welded vertex at `at(i, j)`.
    fn vertex(s: &WeldedSurface, i: usize, j: usize) -> u32 {
        s.positions().iter().position(|&p| p == at(i, j)).unwrap() as u32
    }

    /// The faces whose centroid lies within `r` of `centre`, sorted.
    fn near(s: &WeldedSurface, centre: [f64; 3], r: f64) -> Vec<u32> {
        (0..s.face_count() as u32)
            .filter(|&f| {
                let c = s.face_centroid(f);
                (0..3).map(|k| (c[k] - centre[k]).powi(2)).sum::<f64>() <= r * r
            })
            .collect()
    }

    /// A page holding `faces` (sorted) anew, as the bake makes one: their corners copied once
    /// per vertex in order of first use, at the vertices' places in `s`.
    fn page_of(s: &WeldedSurface, faces: &[u32], jpeg: Arc<[u8]>) -> PatchPage {
        let mut page = PatchPage {
            jpeg,
            ..PatchPage::default()
        };
        let mut copy: BTreeMap<u32, u32> = BTreeMap::new();
        for &f in faces {
            let tri = s.faces()[f as usize].map(|v| {
                *copy.entry(v).or_insert_with(|| {
                    let c = page.positions.len();
                    page.positions.push(s.positions()[v as usize]);
                    page.uvs
                        .push([(c % 16) as f32 / 16.0, (c / 16 % 16) as f32 / 16.0]);
                    c as u32
                })
            });
            page.indices.push(tri);
            page.faces.push(f);
        }
        page
    }

    /// A patch that puts `faces` (sorted) onto one new page with the image `jpeg`.
    fn retexture(s: &WeldedSurface, faces: &[u32], jpeg: Arc<[u8]>) -> SurfacePatch {
        SurfacePatch {
            moved: Vec::new(),
            removed_faces: faces.to_vec(),
            pages: vec![page_of(s, faces, jpeg)],
        }
    }

    /// The bits of a triangle's corners, sorted: the same for a face on whichever page.
    fn key(corners: [[f64; 3]; 3]) -> [[u64; 3]; 3] {
        let mut k = corners.map(bits);
        k.sort();
        k
    }

    /// For every face of `s` (its vertices at `positions`) the pages of `mesh` holding it.
    fn holders(s: &WeldedSurface, positions: &[[f64; 3]], mesh: &Mesh) -> Vec<Vec<u32>> {
        let face: BTreeMap<[[u64; 3]; 3], u32> = (0..s.face_count())
            .map(|f| (key(s.faces()[f].map(|v| positions[v as usize])), f as u32))
            .collect();
        assert_eq!(face.len(), s.face_count(), "faces with the same corners");
        let mut held = vec![Vec::new(); s.face_count()];
        for (p, page) in mesh.pages.iter().enumerate() {
            for tri in &page.indices {
                let k = key(tri.map(|c| page.positions[c as usize]));
                let f = face.get(&k).expect("a triangle that is no face");
                held[*f as usize].push(p as u32);
            }
        }
        held
    }

    fn image(page: &MeshPage) -> &Arc<[u8]> {
        &page.image.as_ref().expect("a page with an image").bytes
    }

    /// A vertex on the border of two pages moves on both, bit for bit, and on a page an earlier
    /// patch added; welded again, the surface has as many vertices and the same faces.
    #[test]
    fn move_keeps_watertight() {
        let (mesh, s) = surface();
        let v = vertex(&s, 4, 5);
        let pages: BTreeSet<u32> = s.copies(v).iter().map(|c| c.0).collect();
        assert!(pages.len() >= 2, "{pages:?}");
        let old = s.positions()[v as usize];
        let p = f32_exact([old[0] + 0.004, old[1] - 0.003, old[2] + 0.02]);
        let moved = SurfacePatch {
            moved: vec![(v, p)],
            ..SurfacePatch::default()
        };
        // The faces around the vertex, put onto a page of their own first.
        let around: Vec<u32> = (0..s.face_count() as u32)
            .filter(|&f| s.faces()[f as usize].contains(&v))
            .collect();
        assert_eq!(around.len(), 6);
        let earlier = retexture(&s, &around, jpeg([1, 2, 3]));
        let mut after = s.positions().to_vec();
        after[v as usize] = p;

        for patches in [vec![moved.clone()], vec![earlier.clone(), moved.clone()]] {
            let out = apply_patches(&mesh, &s, &patches);
            let added: usize = patches.iter().map(|p| p.pages.len()).sum();
            assert_eq!(out.pages.len(), mesh.pages.len() + added);
            // On the computed pages every copy a triangle uses, and nothing else.
            for (pg, (a, b)) in mesh.pages.iter().zip(&out.pages).enumerate() {
                let used: BTreeSet<u32> = b.indices.iter().flatten().copied().collect();
                for (c, (&x, &y)) in a.positions.iter().zip(&b.positions).enumerate() {
                    if used.contains(&(c as u32)) {
                        let copy = s.copies(v).contains(&(pg as u32, c as u32));
                        let want = if copy { p } else { x };
                        assert_eq!(bits(y), bits(want), "page {pg}, corner {c}");
                    }
                }
            }
            if patches.len() == 1 {
                // All copies are in use: all moved.
                for &(pg, c) in s.copies(v) {
                    let y = out.pages[pg as usize].positions[c as usize];
                    assert_eq!(bits(y), bits(p), "page {pg}, corner {c}");
                }
            }
            // Every copy on the added page.
            if let [first, _] = &patches[..] {
                let (page, added) = (&first.pages[0], &out.pages[mesh.pages.len()]);
                let mut copies = 0;
                for (tri, &f) in page.indices.iter().zip(&page.faces) {
                    for (&c, &w) in tri.iter().zip(&s.faces()[f as usize]) {
                        copies += usize::from(w == v);
                        let want = after[w as usize];
                        assert_eq!(bits(added.positions[c as usize]), bits(want), "{c}");
                    }
                }
                assert_eq!(copies, around.len());
            }
            // Still closed.
            let again = WeldedSurface::from_mesh(&out);
            assert_eq!(again.positions().len(), s.positions().len());
            assert_eq!(again.face_count(), s.face_count());
            assert!(holders(&s, &after, &out).iter().all(|h| h.len() == 1));
            let mut want: Vec<_> = s
                .faces()
                .iter()
                .map(|t| key(t.map(|w| after[w as usize])))
                .collect();
            let mut got: Vec<_> = (0..again.face_count() as u32)
                .map(|f| key(again.face_corners(f)))
                .collect();
            want.sort();
            got.sort();
            assert_eq!(got, want);
        }
    }

    /// Faces put onto a new page are held there and nowhere else; the computed pages keep
    /// their corners and the very bytes of their images, the untouched page all of it.
    #[test]
    fn retexture_replaces_faces() {
        let (mesh, s) = surface();
        // Rows 4 to 7 of squares: on pages 2 and 0.
        let faces = near(&s, [0.3, 0.25, 0.0], 0.11);
        let touched: BTreeSet<u32> = faces.iter().map(|&f| s.face_origin(f).0).collect();
        assert_eq!(touched, BTreeSet::from([0, 2]));
        let patch = retexture(&s, &faces, jpeg([9, 99, 199]));
        let out = apply_patches(&mesh, &s, std::slice::from_ref(&patch));
        let n = mesh.pages.len();
        assert_eq!(out.pages.len(), n + 1);

        for (f, held) in holders(&s, s.positions(), &out).iter().enumerate() {
            let f = f as u32;
            let want = match faces.binary_search(&f) {
                Ok(_) => n as u32,
                Err(_) => s.face_origin(f).0,
            };
            assert_eq!(held, &[want], "face {f}");
        }
        for (p, (a, b)) in mesh.pages.iter().zip(&out.pages).enumerate() {
            assert!(Arc::ptr_eq(image(a), image(b)), "page {p}");
            assert_eq!((&a.positions, &a.uvs), (&b.positions, &b.uvs), "page {p}");
            let gone = faces
                .iter()
                .filter(|&&f| s.face_origin(f).0 == p as u32)
                .count();
            assert_eq!(b.indices.len(), a.indices.len() - gone, "page {p}");
            if !touched.contains(&(p as u32)) {
                assert_eq!(a, b, "page {p}");
            }
        }
        let added = &out.pages[n];
        let page = &patch.pages[0];
        assert!(Arc::ptr_eq(image(added), &page.jpeg));
        assert_eq!(added.image.as_ref().unwrap().kind, ImageKind::Jpeg);
        assert_eq!(
            (&added.positions, &added.uvs, &added.indices),
            (&page.positions, &page.uvs, &page.indices)
        );

        // Without patches: the computed surface.
        assert_eq!(apply_patches(&mesh, &s, &[]), mesh);
    }

    /// Two patches over the same faces: the later one holds them, the earlier keeps the rest.
    #[test]
    fn later_patch_wins() {
        let (mesh, s) = surface();
        let first = near(&s, [0.3, 0.2, 0.0], 0.11);
        let second = near(&s, [0.3, 0.3, 0.0], 0.11);
        let both = first
            .iter()
            .filter(|f| second.binary_search(f).is_ok())
            .count();
        assert!(
            both > 10 && both < first.len() && both < second.len(),
            "{both}"
        );
        let a = retexture(&s, &first, jpeg([200, 0, 0]));
        let b = retexture(&s, &second, jpeg([0, 200, 0]));
        let n = mesh.pages.len() as u32;
        for (order, earlier, later) in [([&a, &b], &first, &second), ([&b, &a], &second, &first)] {
            let patches: Vec<SurfacePatch> = order.into_iter().cloned().collect();
            let out = apply_patches(&mesh, &s, &patches);
            assert_eq!(out.pages.len(), n as usize + 2);
            for (f, held) in holders(&s, s.positions(), &out).iter().enumerate() {
                let f = f as u32;
                let want = if later.binary_search(&f).is_ok() {
                    n + 1
                } else if earlier.binary_search(&f).is_ok() {
                    n
                } else {
                    s.face_origin(f).0
                };
                assert_eq!(held, &[want], "face {f}");
            }
            // The earlier page lost only triangles; its corners stay.
            let page = &patches[0].pages[0];
            assert_eq!(out.pages[n as usize].positions, page.positions);
            assert_eq!(out.pages[n as usize].indices.len(), earlier.len() - both);
            assert_eq!(
                out.pages[n as usize + 1].indices,
                patches[1].pages[0].indices
            );
        }
    }

    /// The same Stelle re-textured again („Bilder neu einspielen“ pressed twice): the new page
    /// holds all its faces, and the old page, holding none, is left out – the view uploads no
    /// image for it, the OBJ export writes none.
    #[test]
    fn same_stelle_again_leaves_out_the_old_page() {
        let (mesh, s) = surface();
        let faces = near(&s, [0.3, 0.2, 0.0], 0.11);
        let first = retexture(&s, &faces, jpeg([200, 0, 0]));
        let again = retexture(&s, &faces, jpeg([0, 0, 200]));
        let patches = [first, again];
        let out = apply_patches(&mesh, &s, &patches);
        let n = mesh.pages.len();
        assert_eq!(out.pages.len(), n + 1);
        assert!(Arc::ptr_eq(image(&out.pages[n]), &patches[1].pages[0].jpeg));
        for (f, held) in holders(&s, s.positions(), &out).iter().enumerate() {
            let f = f as u32;
            let want = match faces.binary_search(&f) {
                Ok(_) => n as u32,
                Err(_) => s.face_origin(f).0,
            };
            assert_eq!(held, &[want], "face {f}");
        }
        let (_, added) = page_edits(&s, &mesh, &patches);
        assert_eq!(added, out.pages[n..]);
    }

    /// The edits for the view are what [`apply_patches`] applies: only the changed pages, new
    /// corners only where one moved, the removed triangles sorted; the added pages as they end
    /// up (later moves and removals applied).
    #[test]
    fn page_edits_match_apply() {
        let (mesh, s) = surface();
        // Two Stellen on rows 4 to 7 of squares (pages 2 and 0), overlapping.
        let first = near(&s, [0.3, 0.2, 0.0], 0.11);
        let second = near(&s, [0.3, 0.3, 0.0], 0.11);
        // A vertex inside the first Stelle, moved after it got its page, and one on page 1.
        let inner = vertex(&s, 6, 4);
        let on_one = vertex(&s, 3, 5);
        assert!(s.copies(on_one).iter().all(|c| c.0 == 1));
        let lift = |v: u32| (v, f32_exact(s.positions()[v as usize].map(|c| c + 0.01)));
        let mut moved = vec![lift(inner), lift(on_one)];
        moved.sort_unstable_by_key(|m| m.0);
        let patches = vec![
            retexture(&s, &first, jpeg([200, 0, 0])),
            SurfacePatch {
                moved,
                ..SurfacePatch::default()
            },
            retexture(&s, &second, jpeg([0, 200, 0])),
        ];
        let (edits, added) = page_edits(&s, &mesh, &patches);
        let out = apply_patches(&mesh, &s, &patches);
        assert_eq!(edits.iter().map(|e| e.page).collect::<Vec<_>>(), [0, 1, 2]);
        for edit in &edits {
            let page = &mesh.pages[edit.page as usize];
            let mut rebuilt = page.clone();
            if let Some(positions) = &edit.positions {
                rebuilt.positions.clone_from(positions);
            }
            assert!(edit.removed.windows(2).all(|w| w[0] < w[1]));
            rebuilt.indices = (0..page.indices.len() as u32)
                .filter(|t| edit.removed.binary_search(t).is_err())
                .map(|t| page.indices[t as usize])
                .collect();
            assert_eq!(rebuilt, out.pages[edit.page as usize], "page {}", edit.page);
        }
        // Page 1 has a moved corner in use but lost no triangle. Pages 0 and 2 lost triangles;
        // the inner vertex has copies on them, but no triangle they keep uses one: they keep
        // their own corners.
        assert!(edits[1].positions.is_some() && edits[1].removed.is_empty());
        for e in [&edits[0], &edits[2]] {
            assert!(
                e.positions.is_none() && e.removed.len() > 10,
                "page {}",
                e.page
            );
            assert!(s.copies(inner).iter().any(|c| c.0 == e.page));
        }
        assert_eq!(added, out.pages[mesh.pages.len()..]);
        assert_eq!(added.len(), 2);
        let corner = patches[0].pages[0]
            .positions
            .iter()
            .position(|&q| q == s.positions()[inner as usize])
            .unwrap();
        let lifted = patches[1].moved.iter().find(|m| m.0 == inner).unwrap().1;
        assert_eq!(bits(added[0].positions[corner]), bits(lifted));

        // Smoothed, then re-textured, as a touch-up does it: the computed pages only lose
        // triangles, the moved vertex is on the added page.
        let mut smoothed = s.clone();
        smoothed.apply_moves(&[lift(inner)]);
        let patches = vec![
            SurfacePatch {
                moved: vec![lift(inner)],
                ..SurfacePatch::default()
            },
            retexture(&smoothed, &first, jpeg([200, 0, 0])),
        ];
        let (edits, added) = page_edits(&s, &mesh, &patches);
        assert_eq!(edits.iter().map(|e| e.page).collect::<Vec<_>>(), [0, 2]);
        assert!(edits.iter().all(|e| e.positions.is_none()));
        assert_eq!(added[0].positions[corner], lift(inner).1);
        let out = apply_patches(&mesh, &s, &patches);
        assert_eq!(out.pages[0].positions, mesh.pages[0].positions);
        assert_eq!(out.pages[2].positions, mesh.pages[2].positions);

        // Nothing applied: nothing changed, nothing added.
        assert_eq!(page_edits(&s, &mesh, &[]), (Vec::new(), Vec::new()));
    }

    /// `s` must be the welded surface of the mesh, paged alike.
    #[test]
    #[should_panic(expected = "the welded surface of another mesh")]
    fn patches_need_the_welded_surface_of_the_mesh() {
        let (mesh, s) = surface();
        // As many triangles, paged otherwise.
        let mut other = mesh.clone();
        let moved = other.pages[0].indices.pop().unwrap();
        other.pages[1].indices.push(moved);
        page_edits(&s, &other, &[]);
    }

    /// Written twice, and into another folder, the patches give the same bytes; read back,
    /// the same patches, bit for bit; no `.tmp` stays.
    #[test]
    fn patches_roundtrip_and_bytes() {
        let (_, s) = surface();
        let first = near(&s, [0.3, 0.2, 0.0], 0.12);
        let (low, high) = first.split_at(first.len() / 2);
        let v = vertex(&s, 7, 3);
        let mut moved: Vec<(u32, [f64; 3])> = vec![
            (vertex(&s, 0, 0), [-0.0, 0.0, -1.5]),
            (v, f32_exact([0.351, 0.15, 0.02])),
        ];
        moved.sort_unstable_by_key(|m| m.0);
        let patches = vec![
            SurfacePatch {
                moved: moved.clone(),
                ..SurfacePatch::default()
            },
            retexture(&s, &near(&s, [0.9, 0.3, 0.0], 0.1), jpeg([1, 2, 3])),
            SurfacePatch {
                moved,
                removed_faces: first.clone(),
                pages: vec![
                    page_of(&s, low, jpeg([4, 5, 6])),
                    page_of(&s, high, jpeg([7, 8, 9])),
                ],
            },
            SurfacePatch::default(),
        ];
        let dir = temp_dir("roundtrip");
        let path = dir.join(PATCHES_FILE);
        write_patches(&dir, &patches).unwrap();
        let bytes = fs::read(&path).unwrap();
        write_patches(&dir, &patches).unwrap();
        assert_eq!(fs::read(&path).unwrap(), bytes);
        let other = dir.join("anderswo");
        write_patches(&other, &patches).unwrap();
        assert_eq!(fs::read(other.join(PATCHES_FILE)).unwrap(), bytes);
        let names: BTreeSet<String> = fs::read_dir(&dir)
            .unwrap()
            .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
            .collect();
        assert_eq!(
            names,
            BTreeSet::from([PATCHES_FILE.into(), "anderswo".into()])
        );

        let back = read_patches(&dir).unwrap();
        assert_eq!(back, patches);
        // −0.0 stays −0.0.
        assert_eq!(bits(back[0].moved[0].1), bits([-0.0, 0.0, -1.5]));
        assert_eq!(encode(&back), bytes);

        write_patches(&other, &[]).unwrap();
        assert_eq!(read_patches(&other).unwrap(), Vec::new());
    }

    /// A missing, foreign, cut off, damaged or inconsistent file is refused with a reason.
    #[test]
    fn damaged_patches_are_refused() {
        let (_, s) = surface();
        let patches = vec![retexture(
            &s,
            &near(&s, [0.3, 0.2, 0.0], 0.12),
            jpeg([1, 2, 3]),
        )];
        let dir = temp_dir("damaged");
        let path = dir.join(PATCHES_FILE);
        let refused = |bytes: Option<&[u8]>, why: &str| {
            if let Some(bytes) = bytes {
                fs::write(&path, bytes).unwrap();
            }
            match read_patches(&dir) {
                Err(EditError::Unreadable { path: p, why: w }) => {
                    assert_eq!(p, path);
                    assert!(w.contains(why), "{w}");
                }
                other => panic!("{other:?}"),
            }
        };
        refused(None, "");
        write_patches(&dir, &patches).unwrap();
        let good = fs::read(&path).unwrap();

        let mut flipped = good.clone();
        flipped[good.len() / 2] ^= 0x10;
        refused(Some(&flipped), "beschädigt (Prüfsumme stimmt nicht)");
        refused(
            Some(&good[..good.len() - 1]),
            "beschädigt (Prüfsumme stimmt nicht)",
        );
        refused(Some(&good[..12]), "abgeschnitten");
        refused(Some(b"glTF\x02\0\0\0"), "keine Driftless-Nacharbeit");
        let mut newer = good.clone();
        newer[4..8].copy_from_slice(&2u32.to_le_bytes());
        refused(Some(&newer), "Version 2, erwartet 1");
        // Consistent bytes around a patch that is not: its page holds a face it does not take.
        let mut odd = patches.clone();
        let f = odd[0].removed_faces.pop().unwrap();
        refused(
            Some(&encode(&odd)),
            &format!("beschädigt (Seite 0: Fläche {f} wird nicht ersetzt)"),
        );
        // Bytes after the last patch.
        let mut longer = good[..good.len() - 8].to_vec();
        longer.push(0);
        let sum = crate::content_hash(&longer);
        longer.extend_from_slice(&sum.to_le_bytes());
        refused(Some(&longer), "beschädigt (Daten nach dem Ende)");

        // Nothing was left beside the file.
        assert_eq!(fs::read_dir(&dir).unwrap().count(), 1);
        assert_eq!(read_patches(&dir).ok(), None);
        fs::write(&path, &good).unwrap();
        assert_eq!(read_patches(&dir).unwrap(), patches);
    }

    /// A patch whose page holds a face it does not take off would leave the face twice.
    #[test]
    #[should_panic(expected = "wird nicht ersetzt")]
    fn inconsistent_patch_is_refused() {
        let (mesh, s) = surface();
        let mut patch = retexture(&s, &near(&s, [0.3, 0.2, 0.0], 0.12), jpeg([1, 2, 3]));
        patch.removed_faces.remove(0);
        apply_patches(&mesh, &s, &[patch]);
    }

    /// [`patches_fit`] holds for patches of the surface and not where applying them would
    /// panic: a vertex or face it does not have, a patch not consistent in itself, a corner of
    /// an added page that is a copy of two vertices (a crafted file with a valid checksum).
    #[test]
    fn patches_fit_the_surface_they_are_of() {
        let (mesh, s) = surface();
        let faces = near(&s, [0.3, 0.2, 0.0], 0.12);
        let v = vertex(&s, 7, 3);
        let good = vec![
            SurfacePatch {
                moved: vec![(v, f32_exact([0.351, 0.15, 0.02]))],
                ..SurfacePatch::default()
            },
            retexture(&s, &faces, jpeg([1, 2, 3])),
        ];
        assert!(patches_fit(&s, &good));
        assert!(patches_fit(&s, &[]));
        let unfit = |what: &str, change: &dyn Fn(&mut Vec<SurfacePatch>)| {
            let mut patches = good.clone();
            change(&mut patches);
            assert!(!patches_fit(&s, &patches), "{what}");
            let mesh = mesh.clone();
            let s = s.clone();
            let applied = std::panic::catch_unwind(move || apply_patches(&mesh, &s, &patches));
            assert!(applied.is_err(), "{what}: applying it would panic");
        };
        let n = s.positions().len() as u32;
        unfit("a vertex of another surface", &|p| p[0].moved[0].0 = n);
        let faces_n = s.face_count() as u32;
        unfit("a face of another surface", &|p| {
            p.push(SurfacePatch {
                removed_faces: vec![faces_n],
                ..SurfacePatch::default()
            })
        });
        unfit("not consistent in itself", &|p| {
            p[1].removed_faces.remove(0);
        });
        // corner 0 of a triangle made a copy of the vertex of another triangle's corner 0
        unfit("a corner of two vertices", &|p| {
            let page = &mut p[1].pages[0];
            let first = s.faces()[page.faces[0] as usize][0];
            let t = (1..page.faces.len())
                .find(|&t| s.faces()[page.faces[t] as usize][0] != first)
                .unwrap();
            page.indices[t][0] = page.indices[0][0];
        });
    }

    /// A patch that touches only page 1 leaves the data of page 0 in the glb as it was, byte
    /// for byte; page 2 and the image of page 1 too.
    #[test]
    fn glb_after_apply_keeps_old_pages() {
        let (mesh, s) = surface();
        // A vertex inside a chart of page 1, and the faces of another chart of page 1.
        let v = vertex(&s, 3, 5);
        assert!(s.copies(v).iter().all(|c| c.0 == 1));
        let faces: Vec<u32> = (0..s.face_count() as u32)
            .filter(|&f| s.face_origin(f).0 == 1)
            .filter(|&f| (0.4..0.5).contains(&s.face_centroid(f)[0]))
            .collect();
        assert_eq!(faces.len(), 40);
        let mut patch = retexture(&s, &faces, jpeg([5, 6, 7]));
        let old = s.positions()[v as usize];
        patch.moved = vec![(v, f32_exact([old[0], old[1], old[2] + 0.03]))];
        let out = apply_patches(&mesh, &s, &[patch]);
        assert_eq!(out.pages[0], mesh.pages[0]);
        assert_eq!(out.pages[2], mesh.pages[2]);

        let dir = temp_dir("glb");
        let (before, after) = (dir.join("vorher.glb"), dir.join("nachher.glb"));
        write_glb(&before, &mesh, &Progress::new()).unwrap();
        write_glb(&after, &out, &Progress::new()).unwrap();
        let (b, a) = (glb_parts(&before), glb_parts(&after));
        assert_eq!(a.len(), b.len() + 1);
        assert_eq!(a[0], b[0], "page 0");
        assert_eq!(a[2], b[2], "page 2");
        // Page 1: the same texture coordinates and image, moved corners, fewer triangles.
        assert_eq!((&a[1][1], &a[1][3]), (&b[1][1], &b[1][3]), "page 1");
        assert_ne!(a[1][0], b[1][0]);
        assert_eq!(a[1][2].len(), b[1][2].len() - 40 * 12);
        // Page 0 comes first in the BIN chunk: the same bytes at the same place.
        let bin = |path: &Path| {
            let bytes = fs::read(path).unwrap();
            gltf::Gltf::from_slice(&bytes).unwrap().blob.unwrap()
        };
        let page0: usize = b[0][..3].iter().map(Vec::len).sum();
        assert_eq!(bin(&after)[..page0], bin(&before)[..page0]);
    }

    /// Time to apply a Stelle of 1 m², lifted and then put onto a page of its own, to a
    /// surface of Büro size (1818² squares of 3.5 cm: 6.6 M triangles on 28 pages), and to
    /// write and read its patches.
    /// `cargo test -p driftless-map --release patch:: -- --ignored --nocapture`
    #[test]
    #[ignore = "Messung"]
    fn timing_buero_size() {
        use crate::surface::edit::{smooth_region, Smoothing};
        use crate::surface::testutil::Lcg;
        use std::time::Instant;
        let (n, step) = (1818, 0.035);
        let mut rng = Lcg::new(1);
        let (positions, indices) = grid([n, n], |i, j| {
            f32_exact([i as f64 * step, j as f64 * step, 0.005 * rng.normal()])
        });
        let mut mesh = paged(&positions, &indices, 18, 28);
        drop((positions, indices));
        for page in &mut mesh.pages {
            page.uvs = vec![[0.5; 2]; page.positions.len()];
            page.image = Some(PageImage {
                kind: ImageKind::Jpeg,
                bytes: jpeg([128; 3]),
            });
        }
        let s = WeldedSurface::from_mesh(&mesh);
        let middle = n as f64 * step / 2.0;
        let faces = near(
            &s,
            [middle, middle, 0.0],
            (1.0 / std::f64::consts::PI).sqrt(),
        );
        // Smoothed, then re-textured on the smoothed surface, as a touch-up does it.
        let moved = smooth_region(&s, &faces, Smoothing::Light).unwrap();
        let mut smoothed = s.clone();
        smoothed.apply_moves(&moved);
        let patches = vec![
            SurfacePatch {
                moved,
                ..SurfacePatch::default()
            },
            retexture(&smoothed, &faces, jpeg([200, 100, 0])),
        ];

        let t = Instant::now();
        let (edits, added) = page_edits(&s, &mesh, &patches);
        let edits_time = t.elapsed();
        let t = Instant::now();
        let out = apply_patches(&mesh, &s, &patches);
        let apply_time = t.elapsed();
        assert_eq!(out.pages.len(), mesh.pages.len() + added.len());
        let dir = temp_dir("timing");
        let t = Instant::now();
        write_patches(&dir, &patches).unwrap();
        let write_time = t.elapsed();
        let t = Instant::now();
        assert_eq!(read_patches(&dir).unwrap(), patches);
        let read_time = t.elapsed();
        // Only triangles leave the computed pages: no page needs new corners.
        assert!(edits.iter().all(|e| e.positions.is_none()));
        println!(
            "{} Flächen, {} Punkte verschoben: page_edits {edits_time:?} ({} Seiten verlieren \
             Dreiecke), apply_patches {apply_time:?}, write_patches {write_time:?} ({} Bytes), \
             read_patches {read_time:?}",
            faces.len(),
            patches[0].moved.len(),
            edits.len(),
            fs::metadata(dir.join(PATCHES_FILE)).unwrap().len()
        );
    }

    /// Per primitive of a glb: the bytes of its positions, texture coordinates, triangles and
    /// image in the BIN chunk.
    fn glb_parts(path: &Path) -> Vec<[Vec<u8>; 4]> {
        let bytes = fs::read(path).unwrap();
        let gltf = gltf::Gltf::from_slice(&bytes).unwrap();
        let blob = gltf.blob.as_deref().expect("binary chunk");
        let part = |view: gltf::buffer::View<'_>| {
            blob[view.offset()..view.offset() + view.length()].to_vec()
        };
        let mesh = gltf.meshes().next().expect("one mesh");
        mesh.primitives()
            .map(|prim| {
                let of = |semantic| part(prim.get(&semantic).unwrap().view().unwrap());
                let texture = prim
                    .material()
                    .pbr_metallic_roughness()
                    .base_color_texture()
                    .unwrap()
                    .texture();
                let gltf::image::Source::View { view, .. } = texture.source().source() else {
                    panic!("image not embedded");
                };
                [
                    of(gltf::Semantic::Positions),
                    of(gltf::Semantic::TexCoords(0)),
                    part(prim.indices().unwrap().view().unwrap()),
                    part(view),
                ]
            })
            .collect()
    }
}
