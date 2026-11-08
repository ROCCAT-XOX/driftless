//! Dimensions of a plan as lines and text, not as CAD dimension objects.

use serde::{Deserialize, Serialize};

use crate::{Room, WallFace, WallPair};

/// Edges shorter than this are not dimensioned (metres).
const MIN_EDGE: f64 = 0.30;
/// The dimension line runs this far inside the room (metres).
const LINE_OFFSET: f64 = 0.30;

/// A dimension: what was measured (`a`, `b`), where its line lies and what it says.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Dim {
    pub a: [f64; 2],
    pub b: [f64; 2],
    pub line_a: [f64; 2],
    pub line_b: [f64; 2],
    pub text: String,
}

/// A length as it is written in the plan: „8,30 m“.
pub fn metres(x: f64) -> String {
    let hundredths = (x * 100.0).round();
    // Without this, a very small negative length would read „-0,00 m“.
    let rounded = if hundredths == 0.0 {
        0.0
    } else {
        hundredths / 100.0
    };
    format!("{rounded:.2} m").replace('.', ",")
}

/// One dimension per outline edge of at least 30 cm and one thickness per wall with two faces.
pub(crate) fn dimensions(rooms: &[Room], faces: &[WallFace], pairs: &[WallPair]) -> Vec<Dim> {
    let mut dims = Vec::new();
    for room in rooms {
        let n = room.outline.len();
        for i in 0..n {
            let (a, b) = (room.outline[i], room.outline[(i + 1) % n]);
            let len = (b[0] - a[0]).hypot(b[1] - a[1]);
            if len < MIN_EDGE {
                continue;
            }
            // The outline runs counterclockwise: the room is on its left.
            let inward = [-(b[1] - a[1]) / len, (b[0] - a[0]) / len];
            let shifted = |p: [f64; 2]| {
                [
                    p[0] + inward[0] * LINE_OFFSET,
                    p[1] + inward[1] * LINE_OFFSET,
                ]
            };
            dims.push(Dim {
                a,
                b,
                line_a: shifted(a),
                line_b: shifted(b),
                text: metres(len),
            });
        }
    }
    dims.extend(pairs.iter().filter_map(|pair| thickness(pair, faces)));
    dims
}

/// The thickness of a wall, in the middle of the stretch where both faces overlap.
fn thickness(pair: &WallPair, faces: &[WallFace]) -> Option<Dim> {
    let (front, back) = (&faces[pair.front], &faces[pair.back]);
    let len = (front.b[0] - front.a[0]).hypot(front.b[1] - front.a[1]);
    if len <= 0.0 {
        return None;
    }
    let d = [
        (front.b[0] - front.a[0]) / len,
        (front.b[1] - front.a[1]) / len,
    ];
    let on_front = |p: [f64; 2]| (p[0] - front.a[0]) * d[0] + (p[1] - front.a[1]) * d[1];
    let (t0, t1) = (on_front(back.a), on_front(back.b));
    let (from, to) = (t0.min(t1).max(0.0), t0.max(t1).min(len));
    if to <= from {
        return None;
    }
    let t = (from + to) / 2.0;
    let a = [front.a[0] + d[0] * t, front.a[1] + d[1] * t];
    let b = [
        a[0] - front.inward[0] * pair.thickness,
        a[1] - front.inward[1] * pair.thickness,
    ];
    Some(Dim {
        a,
        b,
        line_a: a,
        line_b: b,
        text: metres(pair.thickness),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn metres_are_written_with_comma_and_unit() {
        assert_eq!(metres(8.3), "8,30 m");
        assert_eq!(metres(0.0), "0,00 m");
        assert_eq!(metres(0.126), "0,13 m");
        assert_eq!(metres(-0.001), "0,00 m");
        assert_eq!(metres(12.0), "12,00 m");
    }

    #[test]
    fn dimensions_skip_short_edges_and_lie_inside_the_room() {
        let room = Room {
            nr: 1,
            outline: vec![[0.0, 0.0], [4.0, 0.0], [4.0, 0.2], [0.0, 0.2]],
            area: 0.8,
            height: None,
            open: false,
            label_at: [2.0, 0.1],
        };
        let dims = dimensions(&[room], &[], &[]);
        assert_eq!(dims.len(), 2, "{dims:?}");
        assert_eq!(dims[0].text, "4,00 m");
        assert_eq!(dims[0].line_a, [0.0, 0.3]);
    }
}
