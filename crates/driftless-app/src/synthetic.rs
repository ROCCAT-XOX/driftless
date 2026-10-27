//! A synthetic cloud for tests and for trying the 3D view without a scan
//! (`--synthetic N`): rooms made of plates (floor, ceiling, four walls) on a grid, from a
//! fixed seed.

use crate::cloud::PointSet;

/// A small seeded random generator (SplitMix64): the same seed gives the same numbers on
/// every platform.
#[derive(Debug, Clone)]
pub struct Rng {
    state: u64,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng { state: seed }
    }

    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in `[0, 1)`.
    pub fn next_f32(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }

    /// Uniform in `0..n` (`n > 0`).
    pub fn below(&mut self, n: u64) -> u64 {
        ((u128::from(self.next_u64()) * u128::from(n)) >> 64) as u64
    }
}

/// Size of one room (m).
const ROOM: [f32; 3] = [6.0, 5.0, 3.0];
/// Points per room, roughly, so that larger clouds get more rooms, not denser ones.
const POINTS_PER_ROOM: usize = 250_000;
/// Scan duration the point times are spread over (s).
const DURATION: f32 = 1200.0;

const PLATE_COLORS: [[u8; 3]; 6] = [
    [150, 120, 90],  // floor
    [235, 235, 230], // ceiling
    [200, 190, 170],
    [170, 185, 200],
    [190, 200, 170],
    [210, 170, 160],
];

/// `n` points on the plates of rooms on a square grid, with colours per plate, intensity,
/// times in scan order, a quality per room and every 50th point flagged as without colour.
pub fn synthetic_cloud(n: usize, seed: u64) -> PointSet {
    let rooms = n.div_ceil(POINTS_PER_ROOM).max(1);
    let side = (rooms as f64).sqrt().ceil() as usize;
    let mut rng = Rng::new(seed);
    let mut xyz = Vec::with_capacity(n);
    let mut rgb = Vec::with_capacity(n);
    let mut intensity = Vec::with_capacity(n);
    let mut t = Vec::with_capacity(n);
    let mut quality = Vec::with_capacity(n);
    let mut flags = Vec::with_capacity(n);
    // Plate areas: floor and ceiling x·y, walls x·z and y·z (twice each).
    let [sx, sy, sz] = ROOM;
    let areas = [sx * sy, sx * sy, sx * sz, sx * sz, sy * sz, sy * sz];
    let total_area: f32 = areas.iter().sum();
    for i in 0..n {
        // Rooms in scan order, as someone walks through them.
        let room = i * rooms / n.max(1);
        let base = [(room % side) as f32 * sx, (room / side) as f32 * sy, 0.0];
        let mut pick = rng.next_f32() * total_area;
        let mut plate = 0;
        while plate < areas.len() - 1 && pick >= areas[plate] {
            pick -= areas[plate];
            plate += 1;
        }
        let (a, b) = (rng.next_f32(), rng.next_f32());
        let noise = (rng.next_f32() - 0.5) * 0.01;
        let local = match plate {
            0 => [a * sx, b * sy, noise],
            1 => [a * sx, b * sy, sz + noise],
            2 => [a * sx, noise, b * sz],
            3 => [a * sx, sy + noise, b * sz],
            4 => [noise, a * sy, b * sz],
            _ => [sx + noise, a * sy, b * sz],
        };
        xyz.push([0, 1, 2].map(|k| base[k] + local[k]));
        let shade = rng.below(40) as u8;
        rgb.push(PLATE_COLORS[plate].map(|c| c.saturating_sub(shade)));
        intensity.push(60 + (plate as u8) * 25 + shade);
        t.push(i as f32 / n as f32 * DURATION);
        quality.push(((room * 97 + 128) % 256) as u8);
        flags.push(u8::from(i % 50 == 0));
    }
    PointSet {
        origin: [0.0; 3],
        xyz,
        rgb: Some(rgb),
        intensity,
        t,
        quality,
        quality_known: true,
        flags,
        source: Vec::new(),
    }
}
