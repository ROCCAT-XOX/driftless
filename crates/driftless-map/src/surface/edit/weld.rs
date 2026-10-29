//! Welding: the pages of a surface as one mesh with shared vertices.
//!
//! The surface step copies the vertices of every chart onto its page (`bake`), so a vertex on
//! a chart border is there once per chart, on one page or several, every copy with the same
//! bits. [`WeldedSurface::from_mesh`] joins them again, as the PLY export does: positions
//! with the same bits are one vertex (+0.0 and −0.0 count as the same place), numbered in the
//! order the triangles first use them, page by page, so the numbering is the PLY export's.
//! Corners no triangle uses are left out.
//!
//! Face `f` is the `f`-th triangle counted over the pages in order; it remembers where it came
//! from (page, triangle). Every vertex keeps its copies (page, corner), so an edit can move all
//! of them alike and the surface stays closed. The copies are one flat list with an offset per
//! vertex, not a list per vertex: the Büro surface has 5.4 M corners on its pages.

use super::{cross, sub, unit};
use driftless_export::mesh::Mesh;
use std::collections::HashMap;

/// No welded vertex (yet).
const NONE: u32 = u32::MAX;

/// A surface's pages as one mesh with shared vertices (see the module docs).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct WeldedSurface {
    /// Every vertex as its first copy has it.
    positions: Vec<[f64; 3]>,
    faces: Vec<[u32; 3]>,
    /// First face of every page; a page without triangles shares it with the next page.
    page_first: Vec<u32>,
    /// The copies of vertex `v` are `copy_list[copy_start[v]..copy_start[v + 1]]`.
    copy_start: Vec<u32>,
    /// (page, corner) of every copy; a vertex's copies by page, then corner.
    copy_list: Vec<(u32, u32)>,
}

impl WeldedSurface {
    /// Welds the pages of `mesh` (see the module docs).
    ///
    /// # Panics
    /// If a triangle names a corner its page does not have (`read_mesh` never gives such a
    /// mesh), or the pages hold 2³² − 1 corners or 2³² triangles or more.
    pub fn from_mesh(mesh: &Mesh) -> WeldedSurface {
        let corners: usize = mesh.pages.iter().map(|p| p.positions.len()).sum();
        assert!(corners < NONE as usize, "{corners} corners are too many");
        let triangles = mesh.triangles();
        assert!(
            u32::try_from(triangles).is_ok(),
            "{triangles} triangles are too many"
        );

        // Vertex by the bits of its position; only looked up, never iterated.
        let mut welded: HashMap<[u64; 3], u32> = HashMap::with_capacity(corners / 2);
        let mut positions: Vec<[f64; 3]> = Vec::new();
        let mut faces = Vec::with_capacity(triangles);
        let mut page_first = Vec::with_capacity(mesh.pages.len());
        // Per page, the vertex of each corner (NONE: no triangle uses it).
        let mut maps: Vec<Vec<u32>> = Vec::with_capacity(mesh.pages.len());
        for page in &mesh.pages {
            page_first.push(faces.len() as u32);
            let mut map = vec![NONE; page.positions.len()];
            for tri in &page.indices {
                faces.push(tri.map(|i| {
                    let slot = &mut map[i as usize];
                    if *slot == NONE {
                        let p = page.positions[i as usize];
                        // +0.0 and −0.0 are the same place.
                        let key = p.map(|c| (c + 0.0).to_bits());
                        let next = positions.len() as u32;
                        *slot = *welded.entry(key).or_insert(next);
                        if *slot == next {
                            positions.push(p);
                        }
                    }
                    *slot
                }));
            }
            maps.push(map);
        }
        drop(welded);
        // Kept for the whole session: no room to grow.
        positions.shrink_to_fit();

        // The copies of every vertex: counted, then filled page by page, corner by corner, so
        // each vertex's list comes out sorted.
        let n = positions.len();
        let mut copy_start = vec![0u32; n + 1];
        for &v in maps.iter().flatten() {
            if v != NONE {
                copy_start[v as usize + 1] += 1;
            }
        }
        for v in 0..n {
            copy_start[v + 1] += copy_start[v];
        }
        let mut copy_list = vec![(0u32, 0u32); copy_start[n] as usize];
        let mut fill = copy_start[..n].to_vec();
        for (p, map) in maps.iter().enumerate() {
            for (i, &v) in map.iter().enumerate() {
                if v != NONE {
                    let at = &mut fill[v as usize];
                    copy_list[*at as usize] = (p as u32, i as u32);
                    *at += 1;
                }
            }
        }

        WeldedSurface {
            positions,
            faces,
            page_first,
            copy_start,
            copy_list,
        }
    }

    /// Number of faces: the triangles of all pages.
    pub fn face_count(&self) -> usize {
        self.faces.len()
    }

    /// The welded vertices (metres, the frame of the surface).
    pub fn positions(&self) -> &[[f64; 3]] {
        &self.positions
    }

    /// The corners of every face, as welded vertices; the page's winding.
    pub fn faces(&self) -> &[[u32; 3]] {
        &self.faces
    }

    /// Where face `face` came from: (page, triangle on the page).
    pub fn face_origin(&self, face: u32) -> (u32, u32) {
        assert!(
            (face as usize) < self.faces.len(),
            "face {face} of {}",
            self.faces.len()
        );
        // The last page starting at or before the face: an empty page shares its start with
        // the page after it.
        let page = self.page_first.partition_point(|&first| first <= face) - 1;
        (page as u32, face - self.page_first[page])
    }

    /// Unit normal of a face by its winding; zero if it has no area.
    pub fn face_normal(&self, face: u32) -> [f64; 3] {
        unit(self.face_cross(face)).unwrap_or([0.0; 3])
    }

    /// Every copy of a vertex: (page, corner), by page, then corner.
    pub fn copies(&self, vertex: u32) -> &[(u32, u32)] {
        let v = vertex as usize;
        &self.copy_list[self.copy_start[v] as usize..self.copy_start[v + 1] as usize]
    }

    /// The positions of a face's corners.
    pub fn face_corners(&self, face: u32) -> [[f64; 3]; 3] {
        self.faces[face as usize].map(|v| self.positions[v as usize])
    }

    /// `(b − a) × (c − a)` of a face's corners `a, b, c`: its normal times twice its area.
    pub fn face_cross(&self, face: u32) -> [f64; 3] {
        let [a, b, c] = self.face_corners(face);
        cross(sub(b, a), sub(c, a))
    }

    /// The mean of a face's corners.
    pub fn face_centroid(&self, face: u32) -> [f64; 3] {
        let [a, b, c] = self.face_corners(face);
        [0, 1, 2].map(|k| (a[k] + b[k] + c[k]) / 3.0)
    }

    /// FNV-1a over every byte (tests: the same surface, byte for byte).
    #[cfg(test)]
    pub(super) fn digest(&self) -> u64 {
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        let mut eat = |bytes: &[u8]| {
            for &b in bytes {
                h = (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3);
            }
        };
        for p in &self.positions {
            for c in p {
                eat(&c.to_bits().to_le_bytes());
            }
        }
        let words = self.faces.iter().flatten();
        let words = words.chain(&self.page_first).chain(&self.copy_start);
        let words = words.chain(self.copy_list.iter().flat_map(|(p, i)| [p, i]));
        for w in words {
            eat(&w.to_le_bytes());
        }
        h
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::surface::testutil::{grid, paged};
    use driftless_export::mesh::MeshPage;

    fn page(positions: Vec<[f64; 3]>, indices: Vec<[u32; 3]>) -> MeshPage {
        MeshPage {
            positions,
            indices,
            ..MeshPage::default()
        }
    }

    /// Two pages share the edge (1, 0, 0)–(0, 1, 0); the second writes one of its corners
    /// with −0.0, has a corner no triangle uses and an empty page before it.
    #[test]
    fn weld_joins_copies() {
        let mesh = Mesh {
            pages: vec![
                page(
                    vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                    vec![[0, 1, 2]],
                ),
                page(vec![], vec![]),
                page(
                    vec![
                        [9.0, 9.0, 9.0],
                        [1.0, -0.0, 0.0],
                        [1.0, 1.0, 0.0],
                        [0.0, 1.0, 0.0],
                    ],
                    vec![[1, 2, 3]],
                ),
            ],
        };
        let s = WeldedSurface::from_mesh(&mesh);
        assert_eq!(s.face_count(), 2);
        assert_eq!(s.faces(), &[[0, 1, 2], [1, 3, 2]]);
        assert_eq!(
            s.positions(),
            &[
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [1.0, 1.0, 0.0]
            ]
        );
        // The shared corners have a copy on each page, the others one, the unused corner none.
        assert_eq!(s.copies(1), &[(0, 1), (2, 1)]);
        assert_eq!(s.copies(2), &[(0, 2), (2, 3)]);
        assert_eq!(s.copies(0), &[(0, 0)]);
        assert_eq!(s.copies(3), &[(2, 2)]);
        assert_eq!(s.face_origin(0), (0, 0));
        assert_eq!(s.face_origin(1), (2, 0));
        assert_eq!(s.face_normal(0), [0.0, 0.0, 1.0]);
        assert_eq!(s.face_normal(1), [0.0, 0.0, 1.0]);
        assert_eq!(s.face_cross(1), [0.0, 0.0, 1.0]);
        assert_eq!(s.face_corners(1)[1], [1.0, 1.0, 0.0]);

        // One bit apart is another place.
        let mut apart = mesh.clone();
        apart.pages[2].positions[1][0] = 1f64.next_up();
        let s = WeldedSurface::from_mesh(&apart);
        assert_eq!(s.positions().len(), 5);
        assert_eq!(s.faces(), &[[0, 1, 2], [3, 4, 2]]);
        assert_eq!(s.copies(1), &[(0, 1)]);
        assert_eq!(s.copies(2), &[(0, 2), (2, 3)]);
    }

    /// A mesh with shared vertices, paged as the surface step does it and welded, comes back:
    /// as many vertices, the same corners per face, every copy at its vertex.
    #[test]
    fn weld_undoes_paging() {
        let (positions, indices) = grid([30, 20], |i, j| {
            [i as f64 * 0.1, j as f64 * 0.1, ((i * j) % 7) as f64 * 0.01]
        });
        let mesh = paged(&positions, &indices, 7, 3);
        let corners: usize = mesh.pages.iter().map(|p| p.positions.len()).sum();
        assert!(corners > positions.len() * 3 / 2, "{corners}");
        let s = WeldedSurface::from_mesh(&mesh);
        assert_eq!(s.positions().len(), positions.len());
        assert_eq!(s.face_count(), indices.len());

        let mut copies = 0;
        for v in 0..s.positions().len() as u32 {
            let c = s.copies(v);
            assert!(!c.is_empty());
            assert!(c.windows(2).all(|w| w[0] < w[1]), "{c:?}");
            for &(p, i) in c {
                assert_eq!(
                    mesh.pages[p as usize].positions[i as usize],
                    s.positions()[v as usize]
                );
            }
            copies += c.len();
        }
        assert_eq!(copies, corners);

        let bits = |p: [f64; 3]| p.map(f64::to_bits);
        let mut want: Vec<[[u64; 3]; 3]> = indices
            .iter()
            .map(|t| {
                let mut c = t.map(|v| bits(positions[v as usize]));
                c.sort();
                c
            })
            .collect();
        let mut got: Vec<[[u64; 3]; 3]> = (0..s.face_count() as u32)
            .map(|f| {
                let (p, t) = s.face_origin(f);
                let page = &mesh.pages[p as usize];
                let on_page = page.indices[t as usize].map(|i| page.positions[i as usize]);
                assert_eq!(on_page, s.face_corners(f));
                let mut c = s.face_corners(f).map(bits);
                c.sort();
                c
            })
            .collect();
        want.sort();
        got.sort();
        assert_eq!(got, want);
    }

    #[test]
    fn empty_and_degenerate() {
        let s = WeldedSurface::from_mesh(&Mesh::default());
        assert_eq!(s.face_count(), 0);
        assert!(s.positions().is_empty() && s.faces().is_empty());
        // A triangle with a repeated corner and one with three corners on a line have no normal.
        let mesh = Mesh {
            pages: vec![page(
                vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
                vec![[0, 1, 1], [0, 1, 2]],
            )],
        };
        let s = WeldedSurface::from_mesh(&mesh);
        assert_eq!(s.faces(), &[[0, 1, 1], [0, 1, 2]]);
        assert_eq!(s.face_normal(0), [0.0; 3]);
        assert_eq!(s.face_normal(1), [0.0; 3]);
        assert_eq!(s.face_centroid(1), [1.0, 0.0, 0.0]);
    }
}
