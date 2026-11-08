//! The whole plan of a storey: storey, walls, openings, rooms and dimensions, turned into the
//! frame of the plan, and the picture from above.
//!
//! In the frame of the plan the main direction is x, the lower left corner of the walls is
//! (0, 0) and the floor is at height 0.

use driftless_io::progress::Progress;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::dims::dimensions;
use crate::levels::{check, comma};
use crate::rooms::find_rooms;
use crate::top_view::top_view;
use crate::{
    find_openings, find_storeys, find_walls, level_for, storey_points, Dim, Leveling, Opening,
    OpeningKind, PlanConfig, PlanError, PlanInput, Room, Storey, StoreyPoints, TopView, WallFace,
    WallPair, Walls,
};

/// Two openings on the two faces of a wall are one if they overlap on this share of the
/// narrower one.
const SAME_OPENING: f64 = 0.5;

/// How the plan lies in the map.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PlanFrame {
    /// The main direction as a unit vector in the map, x of the plan.
    pub axis: [f64; 2],
    /// The point of the plan (0, 0, 0) in the levelled map.
    pub origin: [f64; 3],
    pub leveling: Leveling,
}

impl PlanFrame {
    /// From the map into the plan. The turn uses `axis` only, no angle function.
    pub fn to_plan(&self, map: [f64; 3]) -> [f64; 3] {
        let p = self.leveling.apply(map);
        let [x, y, z] = [
            p[0] - self.origin[0],
            p[1] - self.origin[1],
            p[2] - self.origin[2],
        ];
        [
            self.axis[0] * x + self.axis[1] * y,
            -self.axis[1] * x + self.axis[0] * y,
            z,
        ]
    }

    /// From the plan into the map.
    pub fn to_map(&self, plan: [f64; 3]) -> [f64; 3] {
        let [u, v, z] = plan;
        self.leveling.undo([
            self.axis[0] * u - self.axis[1] * v + self.origin[0],
            self.axis[1] * u + self.axis[0] * v + self.origin[1],
            z + self.origin[2],
        ])
    }
}

/// A finished plan; everything but [`Self::storey`] and [`Self::storeys`] is in the frame of
/// the plan.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FloorPlan {
    pub storey: Storey,
    pub storeys: Vec<Storey>,
    pub frame: PlanFrame,
    pub faces: Vec<WallFace>,
    pub pairs: Vec<WallPair>,
    /// An opening in a wall with two faces stands here twice, see [`Self::distinct_openings`].
    pub openings: Vec<Opening>,
    pub rooms: Vec<Room>,
    pub dims: Vec<Dim>,
    /// In German, for the reader of the plan.
    pub hints: Vec<String>,
    /// Edge of the grid cell the walls were found on (metres).
    pub cell: f64,
}

impl FloorPlan {
    /// Indices into [`Self::openings`], one per real opening. Of the two entries of an opening
    /// in a wall with two faces the surer one stays: a door or window before an unclear one,
    /// otherwise the first.
    pub fn distinct_openings(&self) -> Vec<usize> {
        let sure = |i: usize| self.openings[i].kind != OpeningKind::Unclear;
        (0..self.openings.len())
            .filter(|&i| {
                !(0..self.openings.len()).any(|j| {
                    j != i && self.same_opening(i, j) && (sure(j), j < i) > (sure(i), i < j)
                })
            })
            .collect()
    }

    /// Whether two entries are the same opening seen from the two faces of a wall.
    fn same_opening(&self, i: usize, j: usize) -> bool {
        let (first, second) = (&self.openings[i], &self.openings[j]);
        let paired = self.pairs.iter().any(|p| {
            (p.front, p.back) == (first.face, second.face)
                || (p.back, p.front) == (first.face, second.face)
        });
        if !paired {
            return false;
        }
        let face = &self.faces[first.face];
        let other = &self.faces[second.face];
        let (Some((d, _)), Some((e, _))) = (unit(face.a, face.b), unit(other.a, other.b)) else {
            return false;
        };
        let on_face = |metres: f64| {
            let p = [other.a[0] + e[0] * metres, other.a[1] + e[1] * metres];
            d[0] * (p[0] - face.a[0]) + d[1] * (p[1] - face.a[1])
        };
        let (t0, t1) = (on_face(second.from), on_face(second.to));
        let overlap = t0.max(t1).min(first.to) - t0.min(t1).max(first.from);
        overlap >= SAME_OPENING * (first.to - first.from).min(second.to - second.from)
    }
}

fn unit(a: [f64; 2], b: [f64; 2]) -> Option<([f64; 2], f64)> {
    let len = (b[0] - a[0]).hypot(b[1] - a[1]);
    (len > 0.0).then(|| ([(b[0] - a[0]) / len, (b[1] - a[1]) / len], len))
}

/// What came out of a cloud.
#[derive(Debug, Clone, PartialEq)]
pub enum Analysis {
    Plan(Box<FloorPlan>, TopView),
    /// Nothing is guessed: the map has several storeys and one has to be chosen.
    SeveralStoreys(Vec<Storey>),
}

/// Computes the plan of a cloud, or says which storeys there are to choose from.
pub fn analyze(
    input: &PlanInput,
    cfg: &PlanConfig,
    progress: &Progress,
) -> Result<Analysis, PlanError> {
    let search = find_storeys(input, progress)?;
    let found = search.storeys.len();
    let index = match cfg.storey {
        None if found > 1 => return Ok(Analysis::SeveralStoreys(search.storeys)),
        None => 0,
        Some(nr) if (1..=found).contains(&nr) => nr - 1,
        Some(asked) => return Err(PlanError::NoSuchStorey { asked, found }),
    };
    let storey = search.storeys[index];
    let leveling = level_for(input, &storey);
    check(progress)?;
    let mut points = storey_points(input, &storey, &leveling);

    let walls = find_walls(&points, cfg, progress)?;
    // One wall, even measured from both sides, is no plan.
    if walls.faces.len() < 2 || (walls.faces.len() == 2 && walls.pairs.len() == 1) {
        return Err(PlanError::NoWalls);
    }
    let openings = find_openings(&points, &walls, progress)?;

    progress.phase("Räume", Some(3));
    let (frame, shift) = plan_frame(&points, &walls, leveling);
    let faces = turn_into_plan(&mut points, &walls, frame.axis, shift);
    progress.advance(1);
    check(progress)?;

    let rooms = find_rooms(&points, &faces, &walls.pairs, progress)?;
    let dims = dimensions(&rooms, &faces, &walls.pairs);
    progress.advance(1);
    check(progress)?;
    let view = top_view(&points, &faces, cfg.cell);
    progress.advance(1);

    let mut plan = FloorPlan {
        storey,
        storeys: search.storeys,
        frame,
        faces,
        pairs: walls.pairs,
        openings,
        rooms,
        dims,
        hints: storey_hints(&search.hints, index + 1),
        cell: walls.cell,
    };
    add_hints(&mut plan, cfg);
    Ok(Analysis::Plan(Box::new(plan), view))
}

/// The frame of the plan, and how far the points move once turned onto the main direction.
fn plan_frame(points: &StoreyPoints, walls: &Walls, leveling: Leveling) -> (PlanFrame, [f64; 2]) {
    let [ax, ay] = walls.axis;
    let mut low = [f64::MAX; 2];
    for p in walls.faces.iter().flat_map(|f| [f.a, f.b]) {
        low = [
            low[0].min(ax * p[0] + ay * p[1]),
            low[1].min(-ay * p[0] + ax * p[1]),
        ];
    }
    let frame = PlanFrame {
        axis: walls.axis,
        origin: [
            points.origin[0] + ax * low[0] - ay * low[1],
            points.origin[1] + ay * low[0] + ax * low[1],
            points.origin[2],
        ],
        leveling,
    };
    (frame, low)
}

/// Turns the points and the faces into the frame of the plan. The points are turned in place,
/// so that the cloud is not held twice.
fn turn_into_plan(
    points: &mut StoreyPoints,
    walls: &Walls,
    axis: [f64; 2],
    shift: [f64; 2],
) -> Vec<WallFace> {
    let [ax, ay] = axis;
    let turn = |x: f64, y: f64| [ax * x + ay * y, -ay * x + ax * y];
    let place = |p: [f64; 2]| {
        let [u, v] = turn(p[0], p[1]);
        [u - shift[0], v - shift[1]]
    };
    let move_point = |p: &mut [f32; 3]| {
        let [x, y] = place([p[0] as f64, p[1] as f64]);
        p[0] = x as f32;
        p[1] = y as f32;
    };
    points.xyz.par_iter_mut().for_each(move_point);
    points.poses.iter_mut().for_each(move_point);
    walls
        .faces
        .iter()
        .map(|face| WallFace {
            a: place(face.a),
            b: place(face.b),
            inward: turn(face.inward[0], face.inward[1]),
            seen_m: face.seen_m,
        })
        .collect()
}

/// The hints of the storey search that concern storey `nr`.
fn storey_hints(all: &[String], nr: usize) -> Vec<String> {
    let own = format!("Stockwerk {nr}: ");
    all.iter()
        .filter_map(|hint| match hint.strip_prefix(own.as_str()) {
            Some(rest) => Some(rest.to_string()),
            None if hint.starts_with("Stockwerk ") => None,
            None => Some(hint.clone()),
        })
        .collect()
}

/// Adds the hints on what the plan says: tilt, coarser grid, unclear openings, open rooms.
fn add_hints(plan: &mut FloorPlan, cfg: &PlanConfig) {
    if plan.frame.leveling.tilt_deg > 0.0 {
        plan.hints.push(format!(
            "Die Karte steht {}° schief und wurde gerade gestellt.",
            comma(plan.frame.leveling.tilt_deg, 1)
        ));
    }
    if plan.cell > cfg.cell + 1e-9 {
        plan.hints.push(format!(
            "Die Fläche ist sehr groß: Der Plan rechnet darum mit einem Raster von {} cm statt \
             {} cm.",
            comma(plan.cell * 100.0, 1),
            comma(cfg.cell * 100.0, 1)
        ));
    }
    if plan.rooms.is_empty() {
        plan.hints.push(
            "Kein Raum gefunden: Unter der Laufbahn wurde nicht genug Boden gemessen.".to_string(),
        );
    }
    let unclear = plan
        .distinct_openings()
        .into_iter()
        .filter(|&i| plan.openings[i].kind == OpeningKind::Unclear)
        .count();
    match unclear {
        0 => {}
        1 => plan
            .hints
            .push("Eine Öffnung ist unklar, sie steht mit dem Grund im Plan.".to_string()),
        n => plan.hints.push(format!(
            "{n} Öffnungen sind unklar, sie stehen mit dem Grund im Plan."
        )),
    }
    let open: Vec<u32> = plan.rooms.iter().filter(|r| r.open).map(|r| r.nr).collect();
    for nr in open {
        plan.hints.push(format!(
            "Raum {nr} ist offen: An mehr als 30 % des Umfangs fehlt die Wand."
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::{Scene, TestData};
    use crate::OpeningKind;

    const FLOOR: f64 = -1.3;
    const WIDE: f64 = 8.3;
    const DEEP: f64 = 7.9;
    const AREA: f64 = WIDE * DEEP;

    fn walk() -> [[f64; 3]; 5] {
        [
            [1.0, 1.0, 0.0],
            [7.0, 1.0, 0.0],
            [7.0, 6.5, 0.0],
            [1.0, 6.5, 0.0],
            [1.0, 1.0, 0.0],
        ]
    }

    fn room() -> Scene {
        Scene::new()
            .room([0.0, 0.0], [WIDE, DEEP], FLOOR, 2.6)
            .walk(&walk())
    }

    fn run(data: &TestData) -> (FloorPlan, TopView) {
        match analyze(&data.input(), &PlanConfig::default(), &Progress::new()).unwrap() {
            Analysis::Plan(plan, view) => (*plan, view),
            Analysis::SeveralStoreys(found) => panic!("mehrere Stockwerke: {found:?}"),
        }
    }

    fn plan_of(scene: Scene) -> (FloorPlan, TopView) {
        run(&scene.build())
    }

    fn error_of(data: &TestData, storey: Option<usize>) -> PlanError {
        let cfg = PlanConfig {
            storey,
            ..PlanConfig::default()
        };
        match analyze(&data.input(), &cfg, &Progress::new()) {
            Err(error) => error,
            Ok(_) => panic!("Es hätte keinen Plan geben dürfen"),
        }
    }

    fn near(what: &str, got: f64, expected: f64, tol: f64) {
        assert!(
            (got - expected).abs() <= tol,
            "{what}: {got} statt {expected}"
        );
    }

    fn side_lengths(room: &Room) -> Vec<f64> {
        let n = room.outline.len();
        (0..n)
            .map(|i| {
                let (a, b) = (room.outline[i], room.outline[(i + 1) % n]);
                (b[0] - a[0]).hypot(b[1] - a[1])
            })
            .collect()
    }

    fn assert_box(room: &Room, expected: [f64; 2], tol: f64) {
        assert_eq!(room.outline.len(), 4, "{:?}", room.outline);
        let mut sides = side_lengths(room);
        sides.sort_by(f64::total_cmp);
        near("kurze Seite", sides[0], expected[1], tol);
        near("kurze Seite", sides[1], expected[1], tol);
        near("lange Seite", sides[2], expected[0], tol);
        near("lange Seite", sides[3], expected[0], tol);
    }

    fn has_hint(plan: &FloorPlan, word: &str) -> bool {
        plan.hints.iter().any(|h| h.contains(word))
    }

    #[test]
    fn one_room_plan() {
        let (plan, _) = plan_of(room());
        assert_eq!(plan.rooms.len(), 1, "{:?}", plan.rooms);
        let room = &plan.rooms[0];
        assert_eq!(room.nr, 1);
        assert_box(room, [WIDE, DEEP], 0.01);
        near("Fläche", room.area, AREA, 0.3);
        near("Höhe", room.height.unwrap(), 2.6, 0.02);
        assert!(!room.open);
        assert!(!has_hint(&plan, "offen"), "{:?}", plan.hints);
        let [x, y] = room.label_at;
        near("Beschriftung x", x, WIDE / 2.0, 0.1);
        near("Beschriftung y", y, DEEP / 2.0, 0.1);
    }

    #[test]
    fn two_rooms_split_at_the_door() {
        let scene = Scene::new()
            .room([0.0, 0.0], [4.0, 4.0], FLOOR, 2.6)
            .door(1, 1.5, 0.9, 2.05)
            .room([4.12, 0.0], [9.12, 4.0], FLOOR, 2.6)
            .door(3, 1.6, 0.9, 2.05)
            .walk(&[
                [1.0, 1.0, 0.0],
                [1.0, 3.0, 0.0],
                [2.0, 1.95, 0.0],
                [6.0, 1.95, 0.0],
                [8.0, 3.0, 0.0],
                [8.0, 1.0, 0.0],
            ]);
        let (plan, _) = plan_of(scene);
        assert_eq!(plan.rooms.len(), 2, "{:?}", plan.rooms);
        let (big, small) = (&plan.rooms[0], &plan.rooms[1]);
        assert_eq!((big.nr, small.nr), (1, 2));
        near("große Fläche", big.area, 20.0, 0.3);
        near("kleine Fläche", small.area, 16.0, 0.3);
        assert!(big.label_at[0] > small.label_at[0]);
        assert!(!big.open && !small.open);

        // One door in the building, listed once per face.
        let doors: Vec<usize> = plan
            .distinct_openings()
            .into_iter()
            .filter(|&i| plan.openings[i].kind == OpeningKind::Door)
            .collect();
        assert_eq!(doors.len(), 1, "{:?}", plan.openings);
        assert_eq!(plan.openings.len(), 2, "{:?}", plan.openings);
    }

    #[test]
    fn l_shaped_room_keeps_its_corners() {
        let corners = [
            [0.0, 0.0],
            [6.0, 0.0],
            [6.0, 3.0],
            [3.0, 3.0],
            [3.0, 6.0],
            [0.0, 6.0],
        ];
        let scene = Scene::new().polygon(&corners, FLOOR, 2.6).walk(&[
            [1.0, 1.0, 0.0],
            [5.0, 1.0, 0.0],
            [1.5, 1.5, 0.0],
            [1.5, 5.0, 0.0],
        ]);
        let (plan, _) = plan_of(scene);
        assert_eq!(plan.rooms.len(), 1, "{:?}", plan.rooms);
        assert_eq!(
            plan.rooms[0].outline.len(),
            6,
            "{:?}",
            plan.rooms[0].outline
        );
        near("Fläche", plan.rooms[0].area, 27.0, 0.3);
        assert!(!plan.rooms[0].open);
    }

    #[test]
    fn slanted_wall_stays_slanted_in_the_outline() {
        let corners = [[0.0, 0.0], [6.0, 0.0], [6.0, 4.0], [0.0, 2.0]];
        let scene = Scene::new().polygon(&corners, FLOOR, 2.6).walk(&[
            [1.0, 0.7, 0.0],
            [5.0, 0.7, 0.0],
            [5.0, 2.5, 0.0],
            [1.0, 1.5, 0.0],
        ]);
        let (plan, _) = plan_of(scene);
        assert_eq!(plan.rooms.len(), 1, "{:?}", plan.rooms);
        assert_eq!(
            plan.rooms[0].outline.len(),
            4,
            "{:?}",
            plan.rooms[0].outline
        );
        near("Fläche", plan.rooms[0].area, 18.0, 0.3);
    }

    #[test]
    fn noisy_rotated_room_keeps_its_corners() {
        let (plan, _) = plan_of(room().noise(0.005).spacing(0.02).yaw(37.3));
        assert_eq!(plan.rooms.len(), 1, "{:?}", plan.rooms);
        assert_box(&plan.rooms[0], [WIDE, DEEP], 0.03);
        near("Fläche", plan.rooms[0].area, AREA, 0.4);
        near("Höhe", plan.rooms[0].height.unwrap(), 2.6, 0.03);
        assert!(!plan.rooms[0].open);
    }

    #[test]
    fn a_step_in_the_floor_is_a_hint() {
        let (plan, _) = plan_of(
            Scene::new()
                .room([0.0, 0.0], [WIDE, DEEP], FLOOR, 2.6)
                .slab([1.0, 3.0], [4.0, 6.0], FLOOR + 0.2)
                .walk(&walk()),
        );
        assert!(has_hint(&plan, "Absatz"), "{:?}", plan.hints);
        assert_eq!(plan.rooms.len(), 1, "{:?}", plan.rooms);
    }

    #[test]
    fn the_hints_of_other_storeys_stay_out() {
        let all = [
            "Stockwerk 1: Absatz unten".to_string(),
            "Stockwerk 2: Absatz oben".to_string(),
            "Eine große Fläche".to_string(),
        ];
        assert_eq!(
            storey_hints(&all, 2),
            vec!["Absatz oben".to_string(), "Eine große Fläche".to_string()]
        );
    }

    #[test]
    fn furniture_does_not_split_the_room() {
        let furnished = room()
            .block([2.0, 0.0, FLOOR], [5.0, 0.6, FLOOR + 0.8])
            .block([3.0, 3.0, FLOOR], [5.0, 4.0, FLOOR + 0.5]);
        let (plan, _) = plan_of(furnished);
        assert_eq!(plan.rooms.len(), 1, "{:?}", plan.rooms);
        assert_box(&plan.rooms[0], [WIDE, DEEP], 0.02);
        near("Fläche", plan.rooms[0].area, AREA, 0.3);

        // A sideboard from wall to wall, the trajectory on both sides of it.
        let divided = Scene::new()
            .room([0.0, 0.0], [WIDE, DEEP], FLOOR, 2.6)
            .block([0.0, 3.5, FLOOR], [WIDE, 4.1, FLOOR + 0.9])
            .walk(&[
                [1.0, 1.0, 0.0],
                [7.0, 1.0, 0.0],
                [7.0, 2.5, 0.0],
                [1.0, 2.5, 0.0],
            ])
            .walk(&[
                [1.0, 5.5, 0.0],
                [7.0, 5.5, 0.0],
                [7.0, 6.5, 0.0],
                [1.0, 6.5, 0.0],
            ]);
        let (plan, _) = plan_of(divided);
        assert_eq!(plan.rooms.len(), 1, "{:?}", plan.rooms);
        near("Fläche", plan.rooms[0].area, AREA, 0.3);
    }

    #[test]
    fn open_room() {
        let scene = Scene::new()
            .room([0.0, 0.0], [6.0, 3.0], FLOOR, 2.6)
            .no_wall(2)
            .walk(&[
                [1.0, 1.0, 0.0],
                [5.0, 1.0, 0.0],
                [5.0, 2.2, 0.0],
                [1.0, 2.2, 0.0],
            ]);
        let (plan, _) = plan_of(scene);
        assert_eq!(plan.rooms.len(), 1, "{:?}", plan.rooms);
        assert!(plan.rooms[0].open, "{:?}", plan.rooms[0]);
        near("Fläche", plan.rooms[0].area, 18.0, 0.4);
        assert!(has_hint(&plan, "offen"), "{:?}", plan.hints);
    }

    #[test]
    fn plan_frame_is_aligned() {
        let data = room().yaw(23.0).build();
        let (plan, _) = run(&data);
        let (mut min_x, mut min_y) = (f64::MAX, f64::MAX);
        for face in &plan.faces {
            let (dx, dy) = ((face.b[0] - face.a[0]).abs(), (face.b[1] - face.a[1]).abs());
            assert!(dx < 1e-6 || dy < 1e-6, "{face:?}");
            for p in [face.a, face.b] {
                min_x = min_x.min(p[0]);
                min_y = min_y.min(p[1]);
            }
        }
        assert!(min_x.abs() < 1e-3 && min_y.abs() < 1e-3, "{min_x} {min_y}");
        assert_box(&plan.rooms[0], [WIDE, DEEP], 0.02);

        // The corner (0, 0) of the scene is the corner (0, 0) of the plan.
        let corner = plan.frame.to_plan([0.0, 0.0, FLOOR]);
        assert!(
            corner[0].abs() < 0.02 && corner[1].abs() < 0.02,
            "{corner:?}"
        );
        near("Boden", corner[2], 0.0, 0.01);

        let tilted = room().tilt(1.0).yaw(23.0).build();
        let (plan, _) = run(&tilted);
        assert!(plan.frame.leveling.tilt_deg > 0.5);
        assert!(has_hint(&plan, "schief"), "{:?}", plan.hints);
        for p in [[1.5, -3.25, 0.7], [-12.0, 40.0, 3.0], [0.0, 0.0, 0.0]] {
            let back = plan.frame.to_map(plan.frame.to_plan(p));
            for k in 0..3 {
                near("zurück", back[k], p[k], 1e-6);
            }
        }
    }

    #[test]
    fn dimensions() {
        let (plan, _) = plan_of(room());
        assert_eq!(plan.dims.len(), 4, "{:?}", plan.dims);
        let long: Vec<&Dim> = plan
            .dims
            .iter()
            .filter(|d| (d.b[0] - d.a[0]).hypot(d.b[1] - d.a[1]) > 8.0)
            .collect();
        assert_eq!(long.len(), 2, "{:?}", plan.dims);
        for dim in long {
            assert!(
                ["8,29 m", "8,30 m", "8,31 m"].contains(&dim.text.as_str()),
                "{}",
                dim.text
            );
            // 30 cm into the room.
            near(
                "Maßlinie",
                (dim.line_a[0] - dim.a[0]).hypot(dim.line_a[1] - dim.a[1]),
                0.30,
                1e-9,
            );
            assert!((0.2..DEEP - 0.2).contains(&dim.line_a[1]), "{dim:?}");
        }
    }

    #[test]
    fn wall_thickness_is_dimensioned_once() {
        let scene = Scene::new()
            .room([0.0, 0.0], [4.0, 4.0], FLOOR, 2.6)
            .door(1, 1.5, 0.9, 2.05)
            .room([4.2, 0.0], [8.2, 4.0], FLOOR, 2.6)
            .door(3, 1.6, 0.9, 2.05)
            .walk(&[
                [1.0, 1.0, 0.0],
                [3.0, 1.0, 0.0],
                [3.0, 1.95, 0.0],
                [5.2, 1.95, 0.0],
                [7.0, 1.95, 0.0],
                [7.0, 3.0, 0.0],
            ]);
        let (plan, _) = plan_of(scene);
        assert_eq!(plan.pairs.len(), 1, "{:?}", plan.pairs);
        let thin: Vec<&Dim> = plan
            .dims
            .iter()
            .filter(|d| ["0,19 m", "0,20 m", "0,21 m"].contains(&d.text.as_str()))
            .collect();
        assert_eq!(thin.len(), 1, "{:?}", plan.dims);
    }

    #[test]
    fn unclear_opening_in_a_shared_wall_counts_once() {
        let scene = Scene::new()
            .room([0.0, 0.0], [4.0, 4.0], FLOOR, 2.6)
            .closed_door(1, 1.5, 0.9, 2.05, 0.04)
            .room([4.2, 0.0], [8.2, 4.0], FLOOR, 2.6)
            .closed_door(3, 1.6, 0.9, 2.05, 0.04)
            .walk(&[[2.0, 1.0, 0.0], [2.5, 3.0, 0.0], [1.5, 2.0, 0.0]])
            .walk(&[[6.0, 1.0, 0.0], [5.7, 3.0, 0.0], [6.7, 2.0, 0.0]]);
        let (plan, _) = plan_of(scene);
        let unclear = plan
            .openings
            .iter()
            .filter(|o| o.kind == OpeningKind::Unclear)
            .count();
        assert_eq!(unclear, 2, "{:?}", plan.openings);
        assert!(
            plan.hints
                .iter()
                .any(|h| h.contains("Eine Öffnung ist unklar")),
            "{:?}",
            plan.hints
        );
        assert_eq!(plan.faces.len(), 6, "{:?}", plan.faces);
        let thin: Vec<&Dim> = plan
            .dims
            .iter()
            .filter(|d| d.text.starts_with("0,"))
            .collect();
        assert_eq!(thin.len(), 1, "{:?}", plan.dims);
        assert!(
            ["0,19 m", "0,20 m", "0,21 m"].contains(&thin[0].text.as_str()),
            "{:?}",
            thin
        );
    }

    #[test]
    fn of_two_entries_the_surer_one_stays() {
        let face = |a: [f64; 2], b: [f64; 2], inward: [f64; 2]| WallFace {
            a,
            b,
            inward,
            seen_m: 4.0,
        };
        let opening = |kind: OpeningKind, face: usize, from: f64, to: f64| Opening {
            kind,
            face,
            from,
            to,
            sill: 0.0,
            head: 2.0,
            crossed: false,
            note: String::new(),
        };
        let storey = Storey {
            floor_z: 0.0,
            ceiling_z: None,
            pose_share: 1.0,
            floor_area: 16.0,
        };
        let plan = FloorPlan {
            storey,
            storeys: vec![storey],
            frame: PlanFrame {
                axis: [1.0, 0.0],
                origin: [0.0; 3],
                leveling: Leveling::NONE,
            },
            faces: vec![
                face([0.0, 0.0], [4.0, 0.0], [0.0, -1.0]),
                face([4.0, 0.2], [0.0, 0.2], [0.0, 1.0]),
            ],
            pairs: vec![WallPair {
                front: 0,
                back: 1,
                thickness: 0.2,
            }],
            openings: vec![
                opening(OpeningKind::Unclear, 0, 1.0, 2.0),
                opening(OpeningKind::Unclear, 0, 3.0, 3.5),
                // Same place as the first, seen from the other face; `from` runs the other way.
                opening(OpeningKind::Door, 1, 2.0, 3.0),
            ],
            rooms: Vec::new(),
            dims: Vec::new(),
            hints: Vec::new(),
            cell: 0.02,
        };
        assert_eq!(plan.distinct_openings(), vec![1, 2]);
    }

    #[test]
    fn several_storeys_ask_first() {
        let upper = walk().map(|p| [p[0], p[1], 2.9]);
        let data = Scene::new()
            .room([0.0, 0.0], [WIDE, DEEP], FLOOR, 2.6)
            .room([0.0, 0.0], [WIDE, DEEP], 1.6, 2.6)
            .walk(&walk())
            .walk(&upper)
            .build();

        match analyze(&data.input(), &PlanConfig::default(), &Progress::new()).unwrap() {
            Analysis::SeveralStoreys(found) => assert_eq!(found.len(), 2, "{found:?}"),
            Analysis::Plan(..) => panic!("Das Programm darf nicht raten"),
        }

        let cfg = PlanConfig {
            storey: Some(2),
            ..PlanConfig::default()
        };
        match analyze(&data.input(), &cfg, &Progress::new()).unwrap() {
            Analysis::Plan(plan, _) => {
                near("Boden oben", plan.storey.floor_z, 1.6, 0.02);
                assert_eq!(plan.storeys.len(), 2);
                assert_eq!(plan.rooms.len(), 1, "{:?}", plan.rooms);
                near("Fläche", plan.rooms[0].area, AREA, 0.3);
            }
            Analysis::SeveralStoreys(_) => panic!("Stockwerk 2 war gewählt"),
        }

        assert_eq!(
            error_of(&data, Some(3)),
            PlanError::NoSuchStorey { asked: 3, found: 2 }
        );
        assert_eq!(
            error_of(&data, Some(0)),
            PlanError::NoSuchStorey { asked: 0, found: 2 }
        );
    }

    #[test]
    fn far_stray_points_do_not_blow_up_the_grid() {
        let mut data = room().build();
        let (base, _) = run(&data);
        // A small dense patch of reflections, 80 m away.
        data.xyz.extend(
            (0..500).map(|i| [80.0 + (i % 25) as f64 * 0.01, 80.0, (i / 25) as f64 * 0.02]),
        );
        let (plan, view) = run(&data);
        assert!(view.width <= 1000, "{}", view.width);
        assert!(view.height <= 1000, "{}", view.height);
        assert_eq!(plan.rooms.len(), 1, "{:?}", plan.rooms);
        near("Fläche", plan.rooms[0].area, base.rooms[0].area, 0.05);
        for (p, q) in plan.rooms[0].outline.iter().zip(&base.rooms[0].outline) {
            near("Ecke x", p[0], q[0], 0.01);
            near("Ecke y", p[1], q[1], 0.01);
        }
        assert_eq!(plan.dims.len(), base.dims.len());
        for (a, b) in plan.dims.iter().zip(&base.dims) {
            near("Maß", a.a[0], b.a[0], 0.01);
            near("Maß", a.b[1], b.b[1], 0.01);
        }
    }

    #[test]
    fn huge_extent_coarsens_the_cell() {
        let scene = Scene::new()
            .spacing(0.02)
            .room([0.0, 0.0], [6.0, 5.0], FLOOR, 2.6)
            .room([300.0, 0.0], [306.0, 5.0], FLOOR, 2.6)
            .walk(&[
                [1.0, 1.0, 0.0],
                [5.0, 1.0, 0.0],
                [5.0, 4.0, 0.0],
                [1.0, 4.0, 0.0],
            ])
            .walk(&[
                [301.0, 1.0, 0.0],
                [305.0, 1.0, 0.0],
                [305.0, 4.0, 0.0],
                [301.0, 4.0, 0.0],
            ]);
        let (plan, view) = plan_of(scene);
        assert!(plan.cell > 0.02, "{}", plan.cell);
        assert!(has_hint(&plan, "Raster"), "{:?}", plan.hints);
        assert!(view.width <= 4096 && view.height <= 4096);
        assert_eq!(plan.rooms.len(), 2, "{:?}", plan.rooms);
    }

    #[test]
    fn top_view_shows_the_floor() {
        let (_, view) = plan_of(room());
        let (w, h) = (view.width as usize, view.height as usize);
        assert_eq!(view.rgba.len(), w * h * 4);
        let alpha = |x: usize, y: usize| view.rgba[(y * w + x) * 4 + 3];
        // Row of a height in the plan: row 0 is the largest y.
        let row = |y: f64| h - 1 - ((y - view.origin[1]) / view.cell) as usize;
        let col = |x: f64| ((x - view.origin[0]) / view.cell) as usize;

        let middle = row(DEEP / 2.0);
        let opaque = (0..w).filter(|&x| alpha(x, middle) != 0).count();
        near("Breite", opaque as f64, WIDE / view.cell, 2.0);

        assert_eq!(alpha(col(WIDE / 2.0), middle), 255);
        assert_eq!(alpha(col(WIDE + 0.1), middle), 0);
        assert_eq!(alpha(col(WIDE / 2.0), row(DEEP + 0.1)), 0);
    }

    #[test]
    fn top_view_uses_the_colour_of_the_highest_point() {
        let mut data = room()
            .block([3.0, 3.0, FLOOR], [4.0, 4.0, FLOOR + 1.0])
            .build();
        let rgb: Vec<[u16; 3]> = data
            .xyz
            .iter()
            .map(|p| {
                if p[2] > FLOOR + 0.5 {
                    [65535, 0, 0]
                } else {
                    [0, 0, 65535]
                }
            })
            .collect();
        data.xyz.truncate(rgb.len());
        let input = PlanInput {
            xyz: &data.xyz,
            rgb: Some(&rgb),
            poses: &data.poses,
        };
        let Analysis::Plan(_, view) =
            analyze(&input, &PlanConfig::default(), &Progress::new()).unwrap()
        else {
            panic!("mehrere Stockwerke");
        };
        let at = |x: f64, y: f64| {
            let c = ((x - view.origin[0]) / view.cell) as usize;
            let r = view.height as usize - 1 - ((y - view.origin[1]) / view.cell) as usize;
            let i = (r * view.width as usize + c) * 4;
            [
                view.rgba[i],
                view.rgba[i + 1],
                view.rgba[i + 2],
                view.rgba[i + 3],
            ]
        };
        assert_eq!(at(3.5, 3.5), [255, 0, 0, 255]);
        assert_eq!(at(6.0, 6.0), [0, 0, 255, 255]);
    }

    #[test]
    fn single_wall_is_no_plan() {
        let data = Scene::new()
            .room([0.0, 0.0], [8.0, 6.0], FLOOR, 2.6)
            .no_wall(1)
            .no_wall(2)
            .no_wall(3)
            .walk(&[[1.0, 1.0, 0.0], [7.0, 1.0, 0.0], [7.0, 5.0, 0.0]])
            .build();
        assert_eq!(error_of(&data, None), PlanError::NoWalls);
    }

    #[test]
    fn only_a_floor_is_no_plan() {
        let data = Scene::new()
            .slab([0.0, 0.0], [8.0, 6.0], FLOOR)
            .walk(&[[1.0, 1.0, 0.0], [7.0, 1.0, 0.0], [7.0, 5.0, 0.0]])
            .build();
        assert_eq!(error_of(&data, None), PlanError::NoWalls);
    }

    #[test]
    fn empty_cloud() {
        let data = TestData {
            xyz: Vec::new(),
            poses: vec![[0.0, 0.0, 1.0]],
        };
        assert_eq!(error_of(&data, None), PlanError::TooFewPoints);
    }

    #[test]
    fn same_input_same_result() {
        let data = room()
            .spacing(0.02)
            .noise(0.005)
            .door(0, 3.0, 0.9, 2.05)
            .window(1, 2.0, 1.2, 0.9, 1.3)
            .build();
        let first = run(&data);
        let second = run(&data);
        assert!(first == second);
    }

    #[test]
    fn cancel_stops() {
        let data = room().spacing(0.05).build();
        let progress = Progress::new();
        progress.cancel();
        assert_eq!(
            analyze(&data.input(), &PlanConfig::default(), &progress),
            Err(PlanError::Cancelled)
        );
    }
}
