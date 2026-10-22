//! Manual corrections of a SLAM run in `<lauf>/corrections.toml`. Entries are only appended
//! or switched off and on, never removed; a reuse of the run applies them in seconds.

use crate::error::{failed, StepError};
use crate::slam::{CACHE_FILE, CORRECTIONS_FILE};
use driftless_slam::{match_manual_loops, read_cache, CacheError, Corrections, ManualLoop};
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};

/// The corrections of the run folder `run`; the folder must exist.
pub fn load(run: &Path) -> Result<Corrections, StepError> {
    if !run.is_dir() {
        return Err(format!(
            "Zielordner {} fehlt; zuerst `driftless slam <ordner> -o {}` laufen lassen",
            run.display(),
            run.display()
        )
        .into());
    }
    Ok(Corrections::load(&run.join(CORRECTIONS_FILE))?)
}

/// Checks a new loop together with the active ones against the submaps of `submaps.bin`
/// (times inside the scan, two different submaps, no second loop between the same pair).
/// Without a cache only the times themselves and exact repeats are checked.
pub fn check_new_loop(run: &Path, c: &Corrections, a: f64, b: f64) -> Result<(), StepError> {
    for t in [a, b] {
        if !t.is_finite() || t < 0.0 {
            return Err(format!("Zeit {t} s liegt außerhalb des Scans (ab 0 s)").into());
        }
    }
    let mut loops = c.manual_loops();
    let new = ManualLoop { a_t: a, b_t: b };
    if let Some(old) = loops
        .iter()
        .find(|l| (l.a_t, l.b_t) == (a, b) || (l.a_t, l.b_t) == (b, a))
    {
        return Err(format!(
            "doppelte Schleife: {} s ↔ {} s ist schon eingetragen",
            old.a_t, old.b_t
        )
        .into());
    }
    loops.push(new);
    match read_cache(&run.join(CACHE_FILE)) {
        Ok(cache) => {
            match_manual_loops(&cache.submaps, &loops).map_err(failed)?;
            Ok(())
        }
        Err(CacheError::Missing(_)) => Ok(()),
        Err(e) => Err(failed(e)),
    }
}

/// Checks and appends a manual loop between the places at times `a` and `b` (seconds since
/// the start of the scan) and saves the file; returns its number (from 1).
pub fn add_loop(run: &Path, a: f64, b: f64, note: &str) -> Result<usize, StepError> {
    add_loop_at(run, a, b, note, &utc_now())
}

/// [`add_loop`] with the time stamp `added` (for tests).
#[doc(hidden)]
pub fn add_loop_at(
    run: &Path,
    a: f64,
    b: f64,
    note: &str,
    added: &str,
) -> Result<usize, StepError> {
    let mut c = load(run)?;
    check_new_loop(run, &c, a, b)?;
    let nr = c.add_loop(a, b, note, added);
    c.save(&run.join(CORRECTIONS_FILE))?;
    Ok(nr)
}

/// Switches entry `nr` (from 1) on or off and saves the file; returns whether it changed.
/// Switching an entry back on is checked like a new loop against the active ones.
pub fn set_enabled(run: &Path, nr: usize, enabled: bool) -> Result<bool, StepError> {
    let mut c = load(run)?;
    if let Some(l) = c
        .loops
        .get(nr.wrapping_sub(1))
        .filter(|l| enabled && !l.enabled)
    {
        check_new_loop(run, &c, l.a, l.b)
            .map_err(|e| format!("Eintrag {nr} nicht eingeschaltet: {e}"))?;
    }
    let changed = c.set_enabled(nr, enabled)?;
    if changed {
        c.save(&run.join(CORRECTIONS_FILE))?;
    }
    Ok(changed)
}

/// The status of an entry as the list and the messages name it.
pub fn status_text(enabled: bool) -> &'static str {
    if enabled {
        "aktiv"
    } else {
        "deaktiviert"
    }
}

/// What `driftless correct … loop` answers after [`add_loop`] returned `nr`.
pub fn added_message(nr: usize, a: f64, b: f64) -> String {
    format!("Eintrag {nr} angehängt: Schleife {a} s ↔ {b} s")
}

/// What `driftless correct … enable|disable` answers after [`set_enabled`] (`changed`: its
/// result).
pub fn switched_message(nr: usize, enabled: bool, changed: bool) -> String {
    let state = status_text(enabled);
    if changed {
        format!("Eintrag {nr} ist jetzt {state}")
    } else {
        format!("Eintrag {nr} ist schon {state}; nichts geändert")
    }
}

/// The corrections as `driftless correct <lauf> list` prints them (`path`: their file).
pub fn render_list(c: &Corrections, path: &Path) -> String {
    if c.loops.is_empty() {
        return format!("keine Korrekturen in {}\n", path.display());
    }
    let mut s =
        String::from("Nr  Status       Art       a (s)      b (s)  hinzugefügt           Notiz\n");
    for (i, l) in c.loops.iter().enumerate() {
        s += &format!(
            "{:<3} {:<12} {:<8} {:>6.1} {:>10.1}  {:<21} {}\n",
            i + 1,
            status_text(l.enabled),
            "Schleife",
            l.a,
            l.b,
            l.added,
            l.note
        );
    }
    s
}

/// Current time as RFC 3339 in UTC, to the second.
pub fn utc_now() -> String {
    utc(SystemTime::now())
}

/// Seconds since 1970 of an RFC 3339 time such as `added` (`2026-10-21T14:03:12Z`, also with
/// fractional seconds or an offset like `+02:00`); `None` if it is not one.
pub fn parse_stamp(s: &str) -> Option<f64> {
    let s = s.trim();
    let num = |r: std::ops::Range<usize>| -> Option<i64> {
        let part = s.get(r)?;
        part.bytes()
            .all(|b| b.is_ascii_digit())
            .then(|| part.parse().ok())?
    };
    let sep = |i: usize, c: u8| s.as_bytes().get(i) == Some(&c);
    if !(sep(4, b'-')
        && sep(7, b'-')
        && matches!(s.as_bytes().get(10), Some(b'T' | b't' | b' '))
        && sep(13, b':')
        && sep(16, b':'))
    {
        return None;
    }
    let (year, month, day) = (num(0..4)?, num(5..7)?, num(8..10)?);
    let (hour, minute, second) = (num(11..13)?, num(14..16)?, num(17..19)?);
    if !(1..=12).contains(&month) || !(1..=31).contains(&day) || hour > 23 || minute > 59 {
        return None;
    }
    let mut rest = &s[19..];
    let mut frac = 0.0;
    if let Some(r) = rest.strip_prefix('.') {
        let digits = r.bytes().take_while(u8::is_ascii_digit).count();
        frac = format!("0.{}", r.get(..digits).filter(|d| !d.is_empty())?)
            .parse()
            .ok()?;
        rest = &r[digits..];
    }
    let offset = match rest {
        "Z" | "z" => 0,
        _ => {
            let sign = match rest.as_bytes().first()? {
                b'+' => 1,
                b'-' => -1,
                _ => return None,
            };
            let off = rest.get(1..)?;
            if off.len() != 5 || off.as_bytes()[2] != b':' {
                return None;
            }
            let h: i64 = off[..2].parse().ok()?;
            let m: i64 = off[3..].parse().ok()?;
            sign * (h * 3600 + m * 60)
        }
    };
    // days since 1970-01-01 of a civil date (H. Hinnant's algorithm)
    let y = if month <= 2 { year - 1 } else { year };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (month + 9) % 12;
    let doy = (153 * mp + 2) / 5 + day - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146_097 + doe - 719_468;
    let secs = days * 86_400 + hour * 3600 + minute * 60 + second - offset;
    Some(secs as f64 + frac)
}

/// `time` as RFC 3339 in UTC, to the second (the format of `added`).
pub fn utc(time: SystemTime) -> String {
    let secs = time.duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs());
    let (days, rest) = (secs / 86_400, secs % 86_400);
    // Civil date from days since 1970-01-01 (H. Hinnant's algorithm).
    let z = days as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02}T{:02}:{:02}:{:02}Z",
        rest / 3600,
        rest % 3600 / 60,
        rest % 60
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn stamps_parse_as_times() {
        let z = parse_stamp("2026-10-21T12:00:00Z").unwrap();
        assert_eq!(parse_stamp("2026-10-21T14:00:00+02:00"), Some(z));
        assert_eq!(parse_stamp("2026-10-21T07:30:00-04:30"), Some(z));
        assert_eq!(parse_stamp("2026-10-21T12:00:00.25Z"), Some(z + 0.25));
        // round trip with the format written
        let t = UNIX_EPOCH + Duration::from_secs(1_792_000_000);
        assert_eq!(parse_stamp(&utc(t)), Some(1_792_000_000.0));
        assert_eq!(parse_stamp("1970-01-01T00:00:00Z"), Some(0.0));
        for bad in [
            "",
            "gestern",
            "2026-10-21",
            "2026-13-01T00:00:00Z",
            "2026-10-21T12:00:00",
        ] {
            assert_eq!(parse_stamp(bad), None, "{bad}");
        }
    }
}
