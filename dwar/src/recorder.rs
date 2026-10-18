//! Records the SangamIO stream dwar receives as `.sgio.gz` (SGIO1: the
//! `b"SGIO1\n"` header, then `u64 recv_us | u32 len | protobuf Message` per
//! frame, little-endian), the format of `tools/v8rec sgio` and
//! `tools/py/sangam_client.py`, so recordings replay with
//! `dhruva-slam --replay` unchanged.
//!
//! On the robot the directory is RAM (/tmp), so both a single recording and
//! the directory as a whole are capped.

use crate::config::Recordings;
use flate2::Compression;
use flate2::write::GzEncoder;
use serde::Serialize;
use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::sync::Mutex;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

const SUFFIXES: &[&str] = &[".sgio.gz", ".sgio", ".v8t.gz"];

#[derive(Clone, Debug, Serialize)]
pub struct Info {
    pub name: String,
    pub frames: u64,
    pub raw_bytes: u64,
    pub seconds: f64,
}

struct Active {
    name: String,
    out: GzEncoder<BufWriter<File>>,
    started: Instant,
    frames: u64,
    raw_bytes: u64,
}

pub struct Recorder {
    dir: PathBuf,
    /// Per recording, uncompressed (gzip gets ~3×, so files are smaller).
    max_bytes: u64,
    /// All files in the directory together.
    max_total_bytes: u64,
    active: Mutex<Option<Active>>,
    /// Why the last recording stopped by itself (reported once).
    stopped_reason: Mutex<Option<String>>,
}

#[derive(Serialize)]
pub struct Entry {
    pub name: String,
    pub bytes: u64,
    pub modified_s: u64,
}

impl Recorder {
    pub fn new(cfg: &Recordings) -> Self {
        Self {
            dir: PathBuf::from(&cfg.dir),
            max_bytes: cfg.max_mb * 1024 * 1024,
            max_total_bytes: cfg.max_total_mb * 1024 * 1024,
            active: Mutex::new(None),
            stopped_reason: Mutex::new(None),
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Option<Active>> {
        self.active.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn start(&self, tag: Option<&str>) -> Result<Info, String> {
        let mut a = self.lock();
        if let Some(cur) = a.as_ref() {
            return Err(format!("already recording {}", cur.name));
        }
        // Room for one more full recording (worst case: incompressible).
        let used: u64 = self.list().iter().map(|e| e.bytes).sum();
        if used + self.max_bytes > self.max_total_bytes {
            return Err(format!(
                "recordings use {} MB of {} MB: delete some first",
                used >> 20,
                self.max_total_bytes >> 20
            ));
        }
        std::fs::create_dir_all(&self.dir).map_err(|e| format!("{}: {}", self.dir.display(), e))?;
        let tag: String = tag
            .unwrap_or("web")
            .chars()
            .filter(|c| c.is_ascii_alphanumeric() || *c == '-' || *c == '_')
            .take(32)
            .collect();
        let name = format!("{}_{}.sgio.gz", tag, unix_ms());
        let path = self.dir.join(&name);
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .map_err(|e| format!("{}: {}", path.display(), e))?;
        let mut out = GzEncoder::new(BufWriter::new(file), Compression::fast());
        out.write_all(b"SGIO1\n").map_err(|e| e.to_string())?;
        let act = Active {
            name,
            out,
            started: Instant::now(),
            frames: 0,
            raw_bytes: 0,
        };
        let i = info(&act);
        *a = Some(act);
        Ok(i)
    }

    pub fn stop(&self) -> Option<Info> {
        let active = self.lock().take()?;
        let i = info(&active);
        if let Err(e) = active.out.finish().and_then(|mut w| w.flush()) {
            log::warn!("recording {}: {}", i.name, e);
        }
        Some(i)
    }

    pub fn info(&self) -> Option<Info> {
        self.lock().as_ref().map(info)
    }

    /// Why the last recording stopped by itself, once.
    pub fn take_stopped_reason(&self) -> Option<String> {
        self.stopped_reason.lock().unwrap_or_else(|e| e.into_inner()).take()
    }

    /// Append one SangamIO frame (the protobuf `Message` bytes). Cheap no-op
    /// when not recording; stops at the size cap or on a write error.
    pub fn write(&self, recv_us: u64, frame: &[u8]) {
        let mut a = self.lock();
        let Some(act) = a.as_mut() else { return };
        let ok = act.out.write_all(&recv_us.to_le_bytes()).is_ok()
            && act.out.write_all(&(frame.len() as u32).to_le_bytes()).is_ok()
            && act.out.write_all(frame).is_ok();
        act.frames += 1;
        act.raw_bytes += frame.len() as u64 + 12;
        if ok && act.raw_bytes <= self.max_bytes {
            return;
        }
        let reason = format!(
            "recording {} stopped: {}",
            act.name,
            if ok { "size cap" } else { "write error" }
        );
        drop(a);
        self.stop();
        log::warn!("{}", reason);
        *self.stopped_reason.lock().unwrap_or_else(|e| e.into_inner()) = Some(reason);
    }

    pub fn list(&self) -> Vec<Entry> {
        let mut out = Vec::new();
        if let Ok(rd) = std::fs::read_dir(&self.dir) {
            for e in rd.flatten() {
                let name = e.file_name().to_string_lossy().to_string();
                if !SUFFIXES.iter().any(|s| name.ends_with(s)) {
                    continue;
                }
                if let Ok(m) = e.metadata() {
                    let modified_s = m
                        .modified()
                        .ok()
                        .and_then(|t| t.duration_since(UNIX_EPOCH).ok())
                        .map_or(0, |d| d.as_secs());
                    out.push(Entry {
                        name,
                        bytes: m.len(),
                        modified_s,
                    });
                }
            }
        }
        out.sort_by_key(|e| std::cmp::Reverse(e.modified_s));
        out
    }

    /// Path of a recording: a plain file name with a recording suffix in the
    /// recordings directory, nothing else.
    pub fn path_of(&self, name: &str) -> Option<PathBuf> {
        let plain = !name.contains('/') && !name.contains("..") && !name.starts_with('.');
        if !plain || !SUFFIXES.iter().any(|s| name.ends_with(s)) {
            return None;
        }
        let p = self.dir.join(name);
        // symlink_metadata: a link planted in the directory is not followed.
        std::fs::symlink_metadata(&p).is_ok_and(|m| m.is_file()).then_some(p)
    }

    pub fn delete(&self, name: &str) -> Result<(), String> {
        if self.info().is_some_and(|i| i.name == name) {
            return Err("recording in progress".into());
        }
        let p = self.path_of(name).ok_or("no such recording")?;
        std::fs::remove_file(&p).map_err(|e| e.to_string())
    }

    pub fn dir(&self) -> &std::path::Path {
        &self.dir
    }
}

fn info(a: &Active) -> Info {
    Info {
        name: a.name.clone(),
        frames: a.frames,
        raw_bytes: a.raw_bytes,
        seconds: a.started.elapsed().as_secs_f64(),
    }
}

fn unix_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_millis())
}

pub fn now_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| d.as_micros() as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn recorder(dir: &std::path::Path) -> Recorder {
        Recorder::new(&Recordings {
            dir: dir.display().to_string(),
            max_mb: 1,
            max_total_mb: 4,
        })
    }

    #[test]
    fn records_replayable_sgio_and_refuses_odd_names() {
        let dir = std::env::temp_dir().join(format!("dwar-rec-{}", unix_ms()));
        let r = recorder(&dir);
        let i = r.start(Some("t/../x")).unwrap();
        assert!(!i.name.contains('/'));
        r.write(1, b"abc");
        let done = r.stop().unwrap();
        assert_eq!(done.frames, 1);
        let path = r.path_of(&done.name).unwrap();
        let mut raw = Vec::new();
        std::io::Read::read_to_end(&mut flate2::read::GzDecoder::new(File::open(path).unwrap()), &mut raw).unwrap();
        assert_eq!(&raw[..6], b"SGIO1\n");
        assert_eq!(&raw[6 + 12..], b"abc");
        assert!(r.path_of("../etc/passwd").is_none());
        assert!(r.path_of("dwar.toml").is_none());
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn stops_at_the_size_cap() {
        let dir = std::env::temp_dir().join(format!("dwar-cap-{}", unix_ms()));
        let r = recorder(&dir);
        r.start(None).unwrap();
        let frame = vec![0u8; 64 * 1024];
        for _ in 0..20 {
            r.write(0, &frame);
        }
        assert!(r.info().is_none());
        assert!(r.take_stopped_reason().unwrap().contains("size cap"));
        let _ = std::fs::remove_dir_all(dir);
    }
}
