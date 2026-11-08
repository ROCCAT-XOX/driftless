//! Synthetic rooms for tests (here and, with the feature `test-util`, in other crates).
//!
//! A [`Scene`] is a list of rooms, furniture and free surfaces that [`Scene::build`] turns into
//! points on a regular grid. Noise comes from a fixed LCG and is added as the sum of twelve
//! uniform numbers, so a scene is the same on every system. The pose ([`Scene::yaw`],
//! [`Scene::tilt`]) is applied last, to the points and to the walk.

use crate::PlanInput;

/// A hole in a wall, or (with `recess`) a closed door set back behind the wall surface.
#[derive(Debug, Clone, Copy)]
struct Hole {
    wall: usize,
    at: f64,
    width: f64,
    sill: f64,
    height: f64,
    recess: Option<f64>,
}

#[derive(Debug, Clone)]
struct Room {
    corners: Vec<[f64; 2]>,
    floor: f64,
    height: f64,
    holes: Vec<Hole>,
    missing_walls: Vec<usize>,
    open_top: bool,
}

/// Builds a synthetic scan. See the module description.
#[derive(Debug, Clone)]
pub struct Scene {
    rooms: Vec<Room>,
    blocks: Vec<([f64; 3], [f64; 3])>,
    slabs: Vec<([f64; 2], [f64; 2], f64)>,
    behind: Vec<([f64; 3], [f64; 3])>,
    way: Vec<[f64; 3]>,
    spacing: f64,
    noise: f64,
    yaw: f64,
    tilt: f64,
}

/// The points and the trajectory of a [`Scene`].
#[derive(Debug, Clone, PartialEq)]
pub struct TestData {
    pub xyz: Vec<[f64; 3]>,
    pub poses: Vec<[f64; 3]>,
}

impl TestData {
    /// The scan as an input of the plan (without colours).
    pub fn input(&self) -> PlanInput<'_> {
        PlanInput {
            xyz: &self.xyz,
            rgb: None,
            poses: &self.poses,
        }
    }
}

impl Default for Scene {
    fn default() -> Self {
        Scene::new()
    }
}

impl Scene {
    /// An empty scene: points every centimetre, no noise, no walk, level and unrotated.
    pub fn new() -> Scene {
        Scene {
            rooms: Vec::new(),
            blocks: Vec::new(),
            slabs: Vec::new(),
            behind: Vec::new(),
            way: Vec::new(),
            spacing: 0.01,
            noise: 0.0,
            yaw: 0.0,
            tilt: 0.0,
        }
    }

    /// A rectangular room; wall 0 is the south side, then counterclockwise (east, north, west).
    pub fn room(self, min: [f64; 2], max: [f64; 2], floor: f64, height: f64) -> Scene {
        let corners = [min, [max[0], min[1]], max, [min[0], max[1]]];
        self.polygon(&corners, floor, height)
    }

    /// A room with floor, ceiling and walls; wall `i` runs from `corners[i]` to `corners[i + 1]`.
    pub fn polygon(mut self, corners: &[[f64; 2]], floor: f64, height: f64) -> Scene {
        self.rooms.push(Room {
            corners: corners.to_vec(),
            floor,
            height,
            holes: Vec::new(),
            missing_walls: Vec::new(),
            open_top: false,
        });
        self
    }

    fn last_room(&mut self) -> &mut Room {
        self.rooms
            .last_mut()
            .expect("Scene: first a room, then its openings")
    }

    /// An open doorway in `wall` of the last room, `at` metres from the start of the wall.
    pub fn door(mut self, wall: usize, at: f64, width: f64, height: f64) -> Scene {
        self.last_room().holes.push(Hole {
            wall,
            at,
            width,
            sill: 0.0,
            height,
            recess: None,
        });
        self
    }

    /// A window in `wall` of the last room: `sill` above the floor, `height` tall.
    pub fn window(mut self, wall: usize, at: f64, width: f64, sill: f64, height: f64) -> Scene {
        self.last_room().holes.push(Hole {
            wall,
            at,
            width,
            sill,
            height,
            recess: None,
        });
        self
    }

    /// A closed door in `wall` of the last room, its leaf `recess` behind the wall surface.
    pub fn closed_door(
        mut self,
        wall: usize,
        at: f64,
        width: f64,
        height: f64,
        recess: f64,
    ) -> Scene {
        self.last_room().holes.push(Hole {
            wall,
            at,
            width,
            sill: 0.0,
            height,
            recess: Some(recess),
        });
        self
    }

    /// The last room has no points on `wall`.
    pub fn no_wall(mut self, wall: usize) -> Scene {
        self.last_room().missing_walls.push(wall);
        self
    }

    /// The last room has no ceiling.
    pub fn open_top(mut self) -> Scene {
        self.last_room().open_top = true;
        self
    }

    /// Furniture: top and sides of a box. Hides the floor below it and the walls behind it
    /// when it touches them or stands at most 10 cm in front of them.
    pub fn block(mut self, min: [f64; 3], max: [f64; 3]) -> Scene {
        self.blocks.push((min, max));
        self
    }

    /// A horizontal surface at height `z`.
    pub fn slab(mut self, min: [f64; 2], max: [f64; 2], z: f64) -> Scene {
        self.slabs.push((min, max, z));
        self
    }

    /// Points behind an opening: the faces of a box, or one sheet if the box is flat.
    pub fn behind(mut self, min: [f64; 3], max: [f64; 3]) -> Scene {
        self.behind.push((min, max));
        self
    }

    /// Adds a walk along the way, a pose every 10 cm. Several walks follow one another.
    pub fn walk(mut self, way: &[[f64; 3]]) -> Scene {
        for leg in way.windows(2) {
            let (a, b) = (leg[0], leg[1]);
            let len = dist3(a, b);
            let n = ((len / 0.1).ceil() as usize).max(1);
            for i in 0..n {
                let t = i as f64 / n as f64;
                self.way.push([
                    a[0] + (b[0] - a[0]) * t,
                    a[1] + (b[1] - a[1]) * t,
                    a[2] + (b[2] - a[2]) * t,
                ]);
            }
        }
        if let Some(&last) = way.last() {
            self.way.push(last);
        }
        self
    }

    /// Distance of the points on every surface (metres).
    pub fn spacing(mut self, m: f64) -> Scene {
        self.spacing = m;
        self
    }

    /// Standard deviation of the noise on every coordinate (metres).
    pub fn noise(mut self, sigma: f64) -> Scene {
        self.noise = sigma;
        self
    }

    /// Turns the whole scan about Z, counterclockwise.
    pub fn yaw(mut self, deg: f64) -> Scene {
        self.yaw = deg;
        self
    }

    /// Tilts the whole scan about X.
    pub fn tilt(mut self, deg: f64) -> Scene {
        self.tilt = deg;
        self
    }

    /// Generates the points.
    pub fn build(self) -> TestData {
        let mut xyz = Vec::new();
        for room in &self.rooms {
            self.emit_room(room, &mut xyz);
        }
        for &(min, max, z) in &self.slabs {
            self.emit_slab(min, max, z, &mut xyz);
        }
        for &(min, max) in &self.blocks {
            self.emit_block(min, max, &mut xyz);
        }
        for &(min, max) in &self.behind {
            self.emit_box(min, max, &mut xyz);
        }
        if self.noise > 0.0 {
            let mut lcg = Lcg(0x5EED_CAFE_F00D_0001);
            for p in &mut xyz {
                for c in p.iter_mut() {
                    *c += lcg.gauss() * self.noise;
                }
            }
        }
        let mut poses = self.way.clone();
        self.place(&mut xyz);
        self.place(&mut poses);
        TestData { xyz, poses }
    }

    fn place(&self, points: &mut [[f64; 3]]) {
        if self.yaw != 0.0 {
            let (s, c) = self.yaw.to_radians().sin_cos();
            for p in points.iter_mut() {
                *p = [c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]];
            }
        }
        if self.tilt != 0.0 {
            let (s, c) = self.tilt.to_radians().sin_cos();
            for p in points.iter_mut() {
                *p = [p[0], c * p[1] - s * p[2], s * p[1] + c * p[2]];
            }
        }
    }

    fn emit_room(&self, room: &Room, out: &mut Vec<[f64; 3]>) {
        let (min, max) = bounds(&room.corners);
        grid(max[0] - min[0], max[1] - min[1], self.spacing, |u, v| {
            let q = [min[0] + u, min[1] + v];
            if !inside(&room.corners, q) {
                return;
            }
            if !self.floor_hidden(q, room.floor) {
                out.push([q[0], q[1], room.floor]);
            }
            if !room.open_top {
                out.push([q[0], q[1], room.floor + room.height]);
            }
        });
        for wall in 0..room.corners.len() {
            if !room.missing_walls.contains(&wall) {
                self.emit_wall(room, wall, out);
            }
        }
    }

    fn emit_wall(&self, room: &Room, wall: usize, out: &mut Vec<[f64; 3]>) {
        let a = room.corners[wall];
        let b = room.corners[(wall + 1) % room.corners.len()];
        let len = dist2(a, b);
        let dir = [(b[0] - a[0]) / len, (b[1] - a[1]) / len];
        let outward = if signed_area(&room.corners) >= 0.0 {
            [dir[1], -dir[0]]
        } else {
            [-dir[1], dir[0]]
        };
        grid(len, room.height, self.spacing, |t, h| {
            let at = [a[0] + dir[0] * t, a[1] + dir[1] * t];
            let hole = room.holes.iter().find(|o| {
                o.wall == wall
                    && (o.at..=o.at + o.width).contains(&t)
                    && (o.sill..=o.sill + o.height).contains(&h)
            });
            match hole {
                Some(Hole {
                    recess: Some(r), ..
                }) => out.push([
                    at[0] + outward[0] * r,
                    at[1] + outward[1] * r,
                    room.floor + h,
                ]),
                Some(_) => {}
                None => {
                    if !self.wall_hidden(at, room.floor + h) {
                        out.push([at[0], at[1], room.floor + h]);
                    }
                }
            }
        });
    }

    fn emit_slab(&self, min: [f64; 2], max: [f64; 2], z: f64, out: &mut Vec<[f64; 3]>) {
        grid(max[0] - min[0], max[1] - min[1], self.spacing, |u, v| {
            out.push([min[0] + u, min[1] + v, z]);
        });
    }

    fn emit_block(&self, min: [f64; 3], max: [f64; 3], out: &mut Vec<[f64; 3]>) {
        self.emit_slab([min[0], min[1]], [max[0], max[1]], max[2], out);
        let sides = [
            ([min[0], min[1]], [max[0], min[1]]),
            ([max[0], min[1]], [max[0], max[1]]),
            ([max[0], max[1]], [min[0], max[1]]),
            ([min[0], max[1]], [min[0], min[1]]),
        ];
        for (a, b) in sides {
            let middle = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
            // A side standing against a wall cannot be seen.
            if self.near_a_wall(middle) {
                continue;
            }
            let len = dist2(a, b);
            grid(len, max[2] - min[2], self.spacing, |t, h| {
                let s = t / len;
                out.push([
                    a[0] + (b[0] - a[0]) * s,
                    a[1] + (b[1] - a[1]) * s,
                    min[2] + h,
                ]);
            });
        }
    }

    fn emit_box(&self, min: [f64; 3], max: [f64; 3], out: &mut Vec<[f64; 3]>) {
        let size = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
        for axis in 0..3 {
            let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
            if size[u] < 1e-9 || size[v] < 1e-9 {
                continue;
            }
            let levels: &[f64] = if size[axis] < 1e-9 {
                &[min[axis]]
            } else {
                &[min[axis], max[axis]]
            };
            for &level in levels {
                grid(size[u], size[v], self.spacing, |a, b| {
                    let mut p = [0.0; 3];
                    p[axis] = level;
                    p[u] = min[u] + a;
                    p[v] = min[v] + b;
                    out.push(p);
                });
            }
        }
    }

    fn floor_hidden(&self, q: [f64; 2], floor: f64) -> bool {
        self.blocks.iter().any(|&(min, max)| {
            min[2] <= floor + 0.10 && footprint_distance(q, min, max) <= 0.0 && max[2] > floor
        })
    }

    fn wall_hidden(&self, q: [f64; 2], z: f64) -> bool {
        self.blocks.iter().any(|&(min, max)| {
            (min[2]..=max[2]).contains(&z) && footprint_distance(q, min, max) <= 0.10
        })
    }

    fn near_a_wall(&self, q: [f64; 2]) -> bool {
        self.rooms.iter().any(|room| {
            let n = room.corners.len();
            (0..n).any(|i| {
                !room.missing_walls.contains(&i)
                    && segment_distance(q, room.corners[i], room.corners[(i + 1) % n]) <= 0.10
            })
        })
    }
}

/// Calls `f(u, v)` on a regular grid of about `spacing` over `0..du` × `0..dv`, cell centres.
fn grid(du: f64, dv: f64, spacing: f64, mut f: impl FnMut(f64, f64)) {
    let (nu, nv) = (steps(du, spacing), steps(dv, spacing));
    for i in 0..nu {
        let u = (i as f64 + 0.5) * du / nu as f64;
        for j in 0..nv {
            f(u, (j as f64 + 0.5) * dv / nv as f64);
        }
    }
}

fn steps(len: f64, spacing: f64) -> usize {
    ((len / spacing).round() as usize).max(1)
}

fn dist2(a: [f64; 2], b: [f64; 2]) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1])
}

fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

fn bounds(corners: &[[f64; 2]]) -> ([f64; 2], [f64; 2]) {
    let mut min = corners[0];
    let mut max = corners[0];
    for c in corners {
        min = [min[0].min(c[0]), min[1].min(c[1])];
        max = [max[0].max(c[0]), max[1].max(c[1])];
    }
    (min, max)
}

fn signed_area(corners: &[[f64; 2]]) -> f64 {
    let n = corners.len();
    (0..n)
        .map(|i| {
            let (a, b) = (corners[i], corners[(i + 1) % n]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum::<f64>()
        / 2.0
}

fn inside(corners: &[[f64; 2]], q: [f64; 2]) -> bool {
    let n = corners.len();
    let mut odd = false;
    for i in 0..n {
        let (a, b) = (corners[i], corners[(i + 1) % n]);
        if (a[1] > q[1]) != (b[1] > q[1])
            && q[0] < a[0] + (q[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0])
        {
            odd = !odd;
        }
    }
    odd
}

/// Distance in the plan from `q` to the footprint of a box, 0 inside it.
fn footprint_distance(q: [f64; 2], min: [f64; 3], max: [f64; 3]) -> f64 {
    let dx = (min[0] - q[0]).max(q[0] - max[0]).max(0.0);
    let dy = (min[1] - q[1]).max(q[1] - max[1]).max(0.0);
    dx.hypot(dy)
}

fn segment_distance(q: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let t = ((q[0] - a[0]) * d[0] + (q[1] - a[1]) * d[1]) / (d[0] * d[0] + d[1] * d[1]);
    let t = t.clamp(0.0, 1.0);
    dist2(q, [a[0] + d[0] * t, a[1] + d[1] * t])
}

/// Fixed linear congruential generator: the same noise on every run and system.
struct Lcg(u64);

impl Lcg {
    fn uniform(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 33) as f64 / (1u64 << 31) as f64
    }

    /// Nearly normal (mean 0, deviation 1), without a transcendental function.
    fn gauss(&mut self) -> f64 {
        (0..12).map(|_| self.uniform()).sum::<f64>() - 6.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn small_room() -> Scene {
        Scene::new()
            .spacing(0.05)
            .room([0.0, 0.0], [4.0, 3.0], 0.0, 2.5)
    }

    fn near(p: &[f64; 3], x: f64, y: f64, z: f64, tol: f64) -> bool {
        (p[0] - x).abs() <= tol && (p[1] - y).abs() <= tol && (p[2] - z).abs() <= tol
    }

    #[test]
    fn room_has_floor_ceiling_and_walls() {
        let data = small_room().build();
        let floor = data.xyz.iter().filter(|p| p[2] == 0.0).count();
        let ceiling = data.xyz.iter().filter(|p| p[2] == 2.5).count();
        assert_eq!(floor, 80 * 60);
        assert_eq!(ceiling, 80 * 60);
        assert_eq!(data.xyz.len(), 2 * 80 * 60 + 2 * (80 + 60) * 50);
    }

    #[test]
    fn door_makes_a_hole_and_closed_door_sits_behind_the_wall() {
        let open = small_room().door(0, 1.0, 1.0, 2.0).build();
        let in_door = |p: &&[f64; 3]| {
            (1.1..1.9).contains(&p[0]) && p[1].abs() < 0.2 && (0.1..1.9).contains(&p[2])
        };
        assert_eq!(open.xyz.iter().filter(in_door).count(), 0);

        let closed = small_room().closed_door(0, 1.0, 1.0, 2.0, 0.04).build();
        let leaf: Vec<_> = closed.xyz.iter().filter(in_door).collect();
        assert!(!leaf.is_empty());
        assert!(leaf.iter().all(|p| (p[1] + 0.04).abs() < 1e-9));
    }

    #[test]
    fn block_hides_the_wall_behind_it_but_not_above_it() {
        let data = small_room().block([1.0, 0.0, 0.0], [2.0, 0.6, 2.0]).build();
        let on_wall = |low: f64, high: f64| {
            data.xyz
                .iter()
                .filter(|p| {
                    (1.2..1.8).contains(&p[0]) && p[1] == 0.0 && (low..high).contains(&p[2])
                })
                .count()
        };
        assert_eq!(on_wall(0.1, 1.9), 0);
        assert!(on_wall(2.1, 2.4) > 0);
        // The side against the wall is not measured, the front is.
        assert!(data
            .xyz
            .iter()
            .any(|p| (p[1] - 0.6).abs() < 1e-9 && (1.2..1.8).contains(&p[0])));
    }

    #[test]
    fn no_wall_and_open_top_drop_their_points() {
        let data = small_room().no_wall(1).open_top().build();
        assert!(data.xyz.iter().all(|p| p[2] < 2.5));
        assert!(!data.xyz.iter().any(|p| p[0] > 3.999 && p[2] > 0.5));
    }

    #[test]
    fn walks_are_sampled_and_follow_one_another() {
        let data = small_room()
            .walk(&[[0.5, 0.5, 1.5], [1.5, 0.5, 1.5]])
            .walk(&[[3.0, 2.0, 1.5]])
            .build();
        assert_eq!(data.poses.len(), 12);
        assert!(near(&data.poses[10], 1.5, 0.5, 1.5, 1e-9));
        assert!(near(&data.poses[11], 3.0, 2.0, 1.5, 1e-9));
    }

    #[test]
    fn pose_moves_points_and_walk() {
        let data = Scene::new()
            .spacing(0.5)
            .slab([1.0, 0.0], [1.5, 0.5], 0.0)
            .walk(&[[1.0, 0.0, 0.0]])
            .yaw(90.0)
            .build();
        assert!(near(&data.xyz[0], -0.25, 1.25, 0.0, 1e-9));
        assert!(near(&data.poses[0], 0.0, 1.0, 0.0, 1e-9));

        let tilted = Scene::new().walk(&[[0.0, 10.0, 0.0]]).tilt(1.0).build();
        assert!((tilted.poses[0][2] - 0.17452).abs() < 1e-4);
    }

    #[test]
    fn noise_has_the_asked_size_and_is_repeatable() {
        let scene = Scene::new()
            .spacing(0.02)
            .slab([0.0, 0.0], [4.0, 4.0], 0.0)
            .noise(0.005);
        let (a, b) = (scene.clone().build(), scene.build());
        assert_eq!(a, b);
        let var = a.xyz.iter().map(|p| p[2] * p[2]).sum::<f64>() / a.xyz.len() as f64;
        assert!((var.sqrt() - 0.005).abs() < 2e-4, "{}", var.sqrt());
    }

    #[test]
    fn polygon_keeps_its_shape_and_sheets_have_one_face() {
        let l = [
            [0.0, 0.0],
            [4.0, 0.0],
            [4.0, 2.0],
            [2.0, 2.0],
            [2.0, 4.0],
            [0.0, 4.0],
        ];
        let data = Scene::new().spacing(0.1).polygon(&l, 0.0, 2.0).build();
        assert!(!data
            .xyz
            .iter()
            .any(|p| p[0] > 2.05 && p[1] > 2.05 && p[2] < 1e-9));

        let sheet = Scene::new()
            .spacing(0.1)
            .behind([0.0, 1.0, 0.0], [1.0, 1.0, 1.0])
            .build();
        assert_eq!(sheet.xyz.len(), 100);
        assert!(sheet.xyz.iter().all(|p| p[1] == 1.0));
    }
}
