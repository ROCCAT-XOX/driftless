//! How much room a project takes and how much its drive has left ([`driftless_app::disk`]).

use driftless_app::disk::{folder_size, free_space, need, total_size, NeedKind};
use driftless_io::testscan::tmp;
use std::fs;

#[test]
fn sizes_of_folders_and_of_the_whole() {
    let dir = tmp("platz-ordner");
    fs::write(dir.join("a.bin"), vec![0u8; 100]).unwrap();
    fs::create_dir_all(dir.join("karte-001")).unwrap();
    fs::write(dir.join("karte-001/b.bin"), vec![0u8; 50]).unwrap();
    assert_eq!(folder_size(&dir), 150);
    assert_eq!(folder_size(&dir.join("karte-001")), 50);
    assert_eq!(folder_size(&dir.join("fehlt")), 0);
    assert_eq!(total_size(&dir), 150);
    // a stage linked into another map (as a new map takes it over): the folder holds it, the
    // whole counts it once
    #[cfg(unix)]
    {
        fs::create_dir_all(dir.join("karte-002")).unwrap();
        fs::hard_link(dir.join("karte-001/b.bin"), dir.join("karte-002/b.bin")).unwrap();
        assert_eq!(folder_size(&dir.join("karte-002")), 50);
        assert_eq!(folder_size(&dir), 200);
        assert_eq!(total_size(&dir), 150);
    }
    let _ = fs::remove_dir_all(&dir);
}

#[test]
fn free_space_of_the_drive() {
    let dir = tmp("platz-frei");
    let free = free_space(&dir).expect("the system says");
    assert!(free > 0);
    assert!(free_space(&dir.join("gibt-es-nicht")).is_none());
}

/// The estimate grows with the raw data and with the fineness, and a full scan at the defaults
/// needs well under its raw data.
#[test]
fn what_a_step_needs() {
    let raw = 4_000_000_000;
    let map = need(NeedKind::Map, raw, 0.01);
    assert!(need(NeedKind::Map, raw, 0.005) > map);
    assert!(need(NeedKind::Map, raw, 0.02) < map);
    assert!(need(NeedKind::Map, 2 * raw, 0.01) > map);
    let surface = need(NeedKind::Surface, raw, 0.02);
    assert!(need(NeedKind::Surface, raw, 0.01) > surface);
    let whole = need(NeedKind::Slam, raw, 0.01) + map + surface;
    assert!(whole < raw, "{whole}");
    // the office: map 1 cm about 0.6 GB of 7 GB raw, the estimate above it
    assert!(need(NeedKind::Map, 7_000_000_000, 0.01) > 600_000_000);
}

#[test]
fn sizes_read_as_people_say_them() {
    use driftless_app::ui::text::size;
    assert_eq!(size(0), "0 KB");
    assert_eq!(size(3_000), "3 KB");
    assert_eq!(size(3_400_000), "3,4 MB");
    assert_eq!(size(640_000_000), "640 MB");
    assert_eq!(size(1_234_000_000), "1,2 GB");
    assert_eq!(size(85_000_000_000), "85 GB");
    assert_eq!(size(1_200_000_000_000), "1200 GB");
}
