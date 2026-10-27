//! OBJ with `.mtl` and page images: writer (moved from `driftless-map/src/surface/write.rs`,
//! bytes unchanged for Driftless surfaces) and reader for own and simple foreign files.
//!
//! Written: Z up, metres (Blender: import with "Z up"); texture coordinates with `v` up, so
//! `1 − v` is written. One material `seite_NN` per page: with an image `map_Kd
//! <stem>_seite_NN.jpg|png`, without one `Kd` = the page colour. Positions are written in
//! the shortest form of `f32` when that holds them exactly (every Driftless surface), else of
//! `f64`. Every file goes to `<name>.tmp` first; all are renamed after the last one is
//! written, the `.obj` last: once it is there, the rest is too.
//!
//! Read: `v`, `vt`, `f` with polygons (as fans), negative indices and `v//vn`; `usemtl` and
//! `mtllib` with `Kd` and `map_Kd` (relative to the `.mtl`, JPEG or PNG). One page per
//! material, in the order of first use. A missing `.mtl` or image gives a gray page and a
//! warning. Read as Z up, as Driftless and Blender ("Z up") write it.
//!
//! A foreign OBJ is untrusted: the names in `mtllib` and `map_Kd` are only looked up inside
//! the OBJ's folder (and below it). An absolute name, a Windows drive or share name, a `..`
//! and a link that leads outside count as missing, so the file never makes the program read
//! elsewhere or contact a share.

use super::glb::write_temp;
use super::{check_mesh, page_name, ImageKind, Mesh, MeshPage, PageImage, MISSING_GRAY};
use crate::{check_cancel, ExportError, PHASE_READ};
use driftless_io::progress::Progress;
use std::collections::HashMap;
use std::ffi::OsString;
use std::io::Write;
use std::path::{Path, PathBuf};

/// The files [`write_obj`] writes for `mesh` at `path` (`<stem>.obj`): the `.obj`, the
/// `.mtl` and one `<stem>_seite_NN.jpg|png` per page with an image, in this order.
pub fn obj_files(path: &Path, mesh: &Mesh) -> Vec<PathBuf> {
    let mut files = vec![path.to_path_buf(), path.with_extension("mtl")];
    files.extend(
        mesh.pages
            .iter()
            .enumerate()
            .filter_map(|(p, page)| Some(image_path(path, p, page.image.as_ref()?.kind))),
    );
    files
}

/// `<stem>_seite_NN.<ext>` next to `path`.
fn image_path(path: &Path, p: usize, kind: ImageKind) -> PathBuf {
    let mut name: OsString = path.file_stem().unwrap_or_default().to_os_string();
    name.push(format!("_{}.{}", page_name(p), kind.extension()));
    path.with_file_name(name)
}

/// File name of `path` as text, for the references inside `.obj` and `.mtl`.
fn file_name(path: &Path) -> String {
    path.file_name()
        .unwrap_or_default()
        .to_string_lossy()
        .into_owned()
}

/// Writes `mesh` as `path` (`<stem>.obj`), `<stem>.mtl` and its page images next to it (see
/// the module docs). An error leaves no `.tmp`; the old files stay until the renames.
pub fn write_obj(path: &Path, mesh: &Mesh) -> Result<(), ExportError> {
    check_mesh(path, mesh)?;
    let mtl_path = path.with_extension("mtl");
    let mtl_name = file_name(&mtl_path);
    let mut temps = Vec::new();
    for (p, page) in mesh.pages.iter().enumerate() {
        if let Some(image) = &page.image {
            let file = image_path(path, p, image.kind);
            temps.push(write_temp(&file, |w| w.write_all(&image.bytes))?);
        }
    }
    temps.push(write_temp(&mtl_path, |w| {
        writeln!(
            w,
            "# Driftless surface: one material per texture page, unlit photos"
        )?;
        for (p, page) in mesh.pages.iter().enumerate() {
            writeln!(w, "\nnewmtl {}", page_name(p))?;
            match &page.image {
                Some(image) => {
                    writeln!(w, "Ka 1 1 1\nKd 1 1 1\nKs 0 0 0\nillum 0")?;
                    let file = image_path(path, p, image.kind);
                    writeln!(w, "map_Kd {}", file_name(&file))?;
                }
                None => {
                    let [r, g, b] = page.color.map(|c| f32::from(c) / 255.0);
                    writeln!(w, "Ka 1 1 1\nKd {r} {g} {b}\nKs 0 0 0\nillum 0")?;
                }
            }
        }
        Ok(())
    })?);
    temps.push(write_temp(path, |w| {
        // OBJ keeps Driftless' axes: Z up, metres (import into Blender with "Z up").
        writeln!(w, "# Driftless surface: metres, Z up")?;
        writeln!(w, "mtllib {mtl_name}")?;
        writeln!(w, "o oberflaeche")?;
        let (mut base, mut uv_base) = (1usize, 1usize);
        for (p, page) in mesh.pages.iter().enumerate() {
            for v in &page.positions {
                let [x, y, z] = v.map(Short);
                writeln!(w, "v {x} {y} {z}")?;
            }
            for t in &page.uvs {
                // OBJ texture coordinates have v up.
                writeln!(w, "vt {} {}", t[0], 1.0 - t[1])?;
            }
            writeln!(w, "usemtl {}", page_name(p))?;
            for tri in &page.indices {
                let [a, b, c] = tri.map(|i| i as usize + base);
                if page.uvs.is_empty() {
                    writeln!(w, "f {a} {b} {c}")?;
                } else {
                    let [at, bt, ct] = tri.map(|i| i as usize + uv_base);
                    writeln!(w, "f {a}/{at} {b}/{bt} {c}/{ct}")?;
                }
            }
            base += page.positions.len();
            uv_base += page.uvs.len();
        }
        Ok(())
    })?);
    // All written: rename, the .obj last. A failed rename drops (removes) the other .tmp.
    for temp in temps {
        let target = temp.path().to_path_buf();
        temp.commit().map_err(|e| ExportError::io(&target, e))?;
    }
    Ok(())
}

/// A coordinate in the shortest form of `f32` if that is exact, else of `f64`.
struct Short(f64);

impl std::fmt::Display for Short {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let v = self.0;
        if f64::from(v as f32) == v {
            write!(f, "{}", v as f32)
        } else {
            write!(f, "{v}")
        }
    }
}

/// A material of a `.mtl`.
#[derive(Default)]
struct Material {
    kd: Option<[f32; 3]>,
    /// `map_Kd` as written, and the `.mtl`'s folder it is relative to.
    map: Option<(String, PathBuf)>,
}

/// A page while it is read: its corners by (position, texture coordinate) index.
struct Building {
    material: String,
    corners: HashMap<(usize, usize), u32>,
    positions: Vec<[f64; 3]>,
    uvs: Vec<[f32; 2]>,
    has_uvs: bool,
    indices: Vec<[u32; 3]>,
}

/// Bytes of the file between two progress steps.
const STEP_BYTES: usize = 1 << 20;

/// Reads an OBJ with its materials (see the module docs). Progress: phase [`PHASE_READ`]
/// over the bytes of the `.obj`, a cancel check per MiB.
pub(crate) fn read_obj(path: &Path, progress: &Progress) -> Result<Mesh, ExportError> {
    let bytes = std::fs::read(path).map_err(|e| ExportError::io(path, e))?;
    let text = String::from_utf8_lossy(&bytes);
    progress.phase(PHASE_READ, Some(text.len() as u64));
    let fail = |line: usize, msg: String| {
        ExportError::format(path, format!("OBJ nicht lesbar: Zeile {line}: {msg}"))
    };

    let mut v: Vec<[f64; 3]> = Vec::new();
    let mut vt: Vec<[f32; 2]> = Vec::new();
    let mut libs: Vec<String> = Vec::new();
    let mut pages: Vec<Building> = Vec::new();
    let mut page_of: HashMap<String, usize> = HashMap::new();
    let mut current = String::new();
    let mut since = 0usize;
    for (n, raw) in text.split('\n').enumerate() {
        let line_no = n + 1;
        since += raw.len() + 1;
        if since >= STEP_BYTES {
            check_cancel(progress)?;
            progress.advance(since as u64);
            since = 0;
        }
        let line = raw.trim();
        let (key, rest) = match line.split_once(|c: char| c.is_whitespace()) {
            Some((k, r)) => (k, r.trim()),
            None => (line, ""),
        };
        match key {
            "v" => {
                let mut c = rest.split_whitespace().map(str::parse::<f64>);
                let mut next = || match c.next() {
                    Some(Ok(x)) if x.is_finite() => Ok(x),
                    _ => Err(fail(line_no, "Punkt braucht drei Zahlen".into())),
                };
                v.push([next()?, next()?, next()?]);
            }
            "vt" => {
                let c: Vec<f32> = rest
                    .split_whitespace()
                    .map(str::parse::<f32>)
                    .collect::<Result<_, _>>()
                    .map_err(|_| fail(line_no, "Texturkoordinate ist keine Zahl".into()))?;
                let u = c.first().copied().unwrap_or(0.0);
                let w = c.get(1).copied().unwrap_or(0.0);
                // OBJ has v up, Driftless v down.
                vt.push([u, 1.0 - w]);
            }
            "f" => {
                let corners = rest
                    .split_whitespace()
                    .map(|c| corner(c, v.len(), vt.len()))
                    .collect::<Result<Vec<_>, String>>()
                    .map_err(|m| fail(line_no, m))?;
                if corners.len() < 3 {
                    return Err(fail(line_no, "Fläche braucht mindestens drei Ecken".into()));
                }
                let p = *page_of.entry(current.clone()).or_insert_with(|| {
                    pages.push(Building {
                        material: current.clone(),
                        corners: HashMap::new(),
                        positions: Vec::new(),
                        uvs: Vec::new(),
                        has_uvs: false,
                        indices: Vec::new(),
                    });
                    pages.len() - 1
                });
                let page = &mut pages[p];
                let mut local = Vec::with_capacity(corners.len());
                for (vi, ti) in corners {
                    let key = (vi, ti.unwrap_or(usize::MAX));
                    let next = page.positions.len();
                    let id = *page.corners.entry(key).or_insert_with(|| next as u32);
                    if id as usize == next {
                        if next == u32::MAX as usize {
                            return Err(fail(line_no, "zu viele Ecken".into()));
                        }
                        page.positions.push(v[vi]);
                        page.uvs.push(ti.map_or([0.0; 2], |t| vt[t]));
                        page.has_uvs |= ti.is_some();
                    }
                    local.push(id);
                }
                // A polygon as a fan around its first corner.
                for k in 1..local.len() - 1 {
                    page.indices.push([local[0], local[k], local[k + 1]]);
                }
            }
            "usemtl" => current = rest.to_string(),
            "mtllib" if !rest.is_empty() => libs.push(rest.to_string()),
            _ => {}
        }
    }
    progress.advance(since as u64);

    let dir = match path.parent() {
        Some(d) if !d.as_os_str().is_empty() => d,
        _ => Path::new("."),
    };
    // Every name must stay inside this folder; without it nothing is found.
    let root = std::fs::canonicalize(dir).ok();
    let root = root.as_deref();
    let mut images: HashMap<PathBuf, PageImage> = HashMap::new();
    let mut warned: Vec<String> = Vec::new();
    let mut warn = |text: String| {
        if !warned.contains(&text) {
            progress.warn(text.clone());
            warned.push(text);
        }
    };
    let mut materials: HashMap<String, Material> = HashMap::new();
    for lib in &libs {
        match root.and_then(|root| read_mtl(root, lib)) {
            Some(found) => {
                for (name, m) in found {
                    materials.entry(name).or_insert(m);
                }
            }
            None => warn(format!("Textur fehlt: {lib}")),
        }
    }

    let mut out = Vec::with_capacity(pages.len());
    for b in pages {
        check_cancel(progress)?;
        let (image, color) = match materials.get(&b.material) {
            None => (None, MISSING_GRAY),
            Some(m) => {
                let kd =
                    m.kd.map(|kd| kd.map(|c| (c.clamp(0.0, 1.0) * 255.0).round() as u8));
                match &m.map {
                    None => (None, kd.unwrap_or(MISSING_GRAY)),
                    Some((name, base)) => match load_image(root, base, name, &mut images) {
                        Ok(image) => (Some(image), kd.unwrap_or([255; 3])),
                        Err(text) => {
                            warn(text);
                            (None, MISSING_GRAY)
                        }
                    },
                }
            }
        };
        let mut uvs = b.uvs;
        if !b.has_uvs {
            uvs.clear();
        }
        if image.is_some() && uvs.is_empty() {
            uvs = vec![[0.5; 2]; b.positions.len()];
        }
        out.push(MeshPage {
            positions: b.positions,
            uvs,
            indices: b.indices,
            image,
            color,
        });
    }
    Ok(Mesh { pages: out })
}

/// One corner `v`, `v/vt`, `v//vn` or `v/vt/vn` of a face: 0-based (position, texture
/// coordinate); negative indices count back from the last one read.
fn corner(token: &str, nv: usize, nt: usize) -> Result<(usize, Option<usize>), String> {
    let mut parts = token.split('/');
    let resolve = |s: &str, n: usize, what: &str| -> Result<usize, String> {
        let i: i64 = s
            .parse()
            .map_err(|_| format!("Ecke „{token}“ ist keine Zahl"))?;
        let k = if i > 0 { i - 1 } else { n as i64 + i };
        if i == 0 || k < 0 || k >= n as i64 {
            return Err(format!("Ecke „{token}“ verweist auf keine{what}"));
        }
        Ok(k as usize)
    };
    let vi = resolve(parts.next().unwrap_or(""), nv, "n Punkt")?;
    let ti = match parts.next() {
        None | Some("") => None,
        Some(t) => Some(resolve(t, nt, " Texturkoordinate")?),
    };
    Ok((vi, ti))
}

/// The materials of the `.mtl` `name` (relative to the OBJ's canonical folder `root`);
/// `None` if it cannot be read or lies outside.
fn read_mtl(root: &Path, name: &str) -> Option<Vec<(String, Material)>> {
    let path = find(root, root, name)?;
    let bytes = std::fs::read(&path).ok()?;
    let base = path.parent().unwrap_or(Path::new("")).to_path_buf();
    let mut out: Vec<(String, Material)> = Vec::new();
    for raw in String::from_utf8_lossy(&bytes).lines() {
        let line = raw.trim();
        let (key, rest) = match line.split_once(|c: char| c.is_whitespace()) {
            Some((k, r)) => (k, r.trim()),
            None => (line, ""),
        };
        match key {
            "newmtl" => out.push((rest.to_string(), Material::default())),
            "Kd" => {
                if let Some((_, m)) = out.last_mut() {
                    let c: Vec<f32> = rest
                        .split_whitespace()
                        .filter_map(|x| x.parse().ok())
                        .collect();
                    if c.len() >= 3 {
                        m.kd = Some([c[0], c[1], c[2]]);
                    }
                }
            }
            "map_Kd" => {
                if let Some((_, m)) = out.last_mut() {
                    let file = strip_map_options(rest);
                    if !file.is_empty() {
                        m.map = Some((file.to_string(), base.clone()));
                    }
                }
            }
            _ => {}
        }
    }
    Some(out)
}

/// The file name of a `map_Kd` value, after its options (`-s 1 1 1`, `-clamp on`, …).
fn strip_map_options(mut rest: &str) -> &str {
    while rest.starts_with('-') {
        let (opt, after) = rest
            .split_once(|c: char| c.is_whitespace())
            .unwrap_or((rest, ""));
        rest = after.trim_start();
        // Arguments: numbers, on/off, or the one channel letter of -imfchan.
        let mut first = true;
        loop {
            let (arg, after) = rest
                .split_once(|c: char| c.is_whitespace())
                .unwrap_or((rest, ""));
            let takes = arg.parse::<f64>().is_ok()
                || arg == "on"
                || arg == "off"
                || (first && opt == "-imfchan" && arg.len() == 1)
                || (first && opt == "-type");
            if arg.is_empty() || !takes || after.is_empty() {
                break;
            }
            rest = after.trim_start();
            first = false;
        }
    }
    rest.trim()
}

/// The file `name` relative to `base`, if it exists inside `root` (the OBJ's canonical
/// folder; `base` lies in it); a Windows path with `\` is tried with `/` as well. `None` for
/// a name that is absolute, has a drive or share prefix or a `..`, and for one whose real
/// place (links followed) is outside `root`.
fn find(root: &Path, base: &Path, name: &str) -> Option<PathBuf> {
    let escapes = name.starts_with(['/', '\\'])
        || name.contains(':')
        || Path::new(name).has_root()
        || Path::new(name).is_absolute()
        || name.split(['/', '\\']).any(|part| part == "..");
    if escapes {
        return None;
    }
    [base.join(name), base.join(name.replace('\\', "/"))]
        .into_iter()
        .filter_map(|p| std::fs::canonicalize(p).ok())
        .find(|p| p.starts_with(root) && p.is_file())
}

/// The page image `name` relative to `base` (see [`find`]) if it exists and is a JPEG or PNG,
/// else the warning. An image read once is shared by every page that names it.
fn load_image(
    root: Option<&Path>,
    base: &Path,
    name: &str,
    images: &mut HashMap<PathBuf, PageImage>,
) -> Result<PageImage, String> {
    let missing = || format!("Textur fehlt: {name}");
    let file = root
        .and_then(|root| find(root, base, name))
        .ok_or_else(missing)?;
    if let Some(image) = images.get(&file) {
        return Ok(image.clone());
    }
    let bytes = std::fs::read(&file).map_err(|_| missing())?;
    match ImageKind::sniff(&bytes) {
        Some(kind) => {
            let image = PageImage {
                kind,
                bytes: bytes.into(),
            };
            images.insert(file, image.clone());
            Ok(image)
        }
        None => Err(format!(
            "Texturformat nicht unterstützt (nur JPEG und PNG): {name}"
        )),
    }
}
