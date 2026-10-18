//! Records the SangamIO stream dwar receives as MCAP (https://mcap.dev), the
//! log format of Foxglove and ROS 2 bags, written with the Foxglove SDK.
//!
//! Channel `/sangamio/message`: every frame as received, protobuf
//! `sangamio.Message` with its schema embedded, log time = receive time.
//! zstd in 256 KiB chunks: on the R16 that is ~5× smaller than raw for 0.3 %
//! CPU, at worst ~20 ms per chunk on this (single) runtime thread; 1 MiB
//! chunks stalled 70 ms. A file cut short by a crash loses its last chunk
//! (~8 s) and is restored with `mcap recover`.
//!
//! Names sort by time and never repeat: `<UTC start>_<tag>_<6 random hex>.mcap`,
//! e.g. `20261018T193116Z_web_3fa9c1.mcap`.
//!
//! On the robot the directory is RAM (/tmp), so both a single recording and
//! the directory as a whole are capped, in bytes on disk.

use crate::config::Recordings;
use foxglove::{Context, McapCompression, McapWriteOptions, McapWriterHandle, PartialMetadata, RawChannel, Schema};
use serde::Serialize;
use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufWriter, Read, Seek, SeekFrom, Write};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

/// Recordings we list and serve; `.sgio.gz` / `.v8t.gz` are older captures.
const SUFFIXES: &[&str] = &[".mcap", ".sgio.gz", ".sgio", ".v8t.gz"];
const CHUNK_BYTES: u64 = 256 * 1024;

#[derive(Clone, Debug, Serialize)]
pub struct Info {
    pub name: String,
    pub frames: u64,
    /// Size on disk so far (lags by up to one chunk).
    pub bytes: u64,
    pub seconds: f64,
}

struct Active {
    name: String,
    mcap: McapWriterHandle<TrackedFile>,
    file: Arc<FileState>,
    started: Instant,
    frames: u64,
}

pub struct Recorder {
    dir: PathBuf,
    /// Per recording, on disk.
    max_bytes: u64,
    /// All files in the directory together.
    max_total_bytes: u64,
    /// Our own Foxglove context: its only sink is the active recording.
    context: Arc<Context>,
    sangamio: Arc<RawChannel>,
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
    pub fn new(cfg: &Recordings) -> Result<Self, String> {
        let context = Context::new();
        let sangamio = context
            .channel_builder("/sangamio/message")
            .message_encoding("protobuf")
            .schema(Schema::new("sangamio.Message", "protobuf", crate::proto::DESCRIPTORS))
            .build_raw()
            .map_err(|e| format!("recorder: {}", e))?;
        Ok(Self {
            dir: PathBuf::from(&cfg.dir),
            max_bytes: cfg.max_mb * 1024 * 1024,
            max_total_bytes: cfg.max_total_mb * 1024 * 1024,
            context,
            sangamio,
            active: Mutex::new(None),
            stopped_reason: Mutex::new(None),
        })
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Option<Active>> {
        self.active.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Start a recording. Its `run` metadata record is `metadata` plus the
    /// tag and the start time.
    pub fn start(&self, tag: Option<&str>, metadata: &BTreeMap<String, String>) -> Result<Info, String> {
        let mut a = self.lock();
        if let Some(cur) = a.as_ref() {
            return Err(format!("already recording {}", cur.name));
        }
        // Room for one more full recording.
        let used: u64 = self.list().iter().map(|e| e.bytes).sum();
        if used + self.max_bytes > self.max_total_bytes {
            return Err(format!(
                "recordings use {} MB of {} MB: delete some first",
                used >> 20,
                self.max_total_bytes >> 20
            ));
        }
        std::fs::create_dir_all(&self.dir).map_err(|e| format!("{}: {}", self.dir.display(), e))?;
        // No `_`: it separates the fields of the name.
        let tag: String = tag
            .unwrap_or("web")
            .chars()
            .filter(|c| c.is_ascii_alphanumeric() || *c == '-')
            .take(32)
            .collect();
        let now = SystemTime::now();
        let name = format!("{}_{}_{}.mcap", compact_utc(now), tag, random_hex()?);
        let path = self.dir.join(&name);
        let state = Arc::new(FileState::default());
        let file = File::create_new(&path).map_err(|e| format!("{}: {}", path.display(), e))?;
        let tracked = TrackedFile {
            file: BufWriter::new(file),
            pos: 0,
            state: Arc::clone(&state),
        };
        let options = McapWriteOptions::new()
            .compression(Some(McapCompression::Zstd))
            .chunk_size(Some(CHUNK_BYTES));
        let mcap = self
            .context
            .mcap_writer_with_options(options)
            .create(tracked)
            .map_err(|e| format!("{}: {}", path.display(), e))?;
        let mut run = metadata.clone();
        run.insert("tag".into(), tag);
        run.insert("started".into(), humantime::format_rfc3339_millis(now).to_string());
        if let Err(e) = mcap.write_metadata("run", run) {
            drop(mcap);
            let _ = std::fs::remove_file(&path);
            return Err(e.to_string());
        }
        let act = Active {
            name,
            mcap,
            file: state,
            started: Instant::now(),
            frames: 0,
        };
        let i = info(&act);
        *a = Some(act);
        Ok(i)
    }

    pub fn stop(&self) -> Option<Info> {
        let active = self.lock().take()?;
        let i = info(&active);
        if let Err(e) = active.mcap.close() {
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
        self.sangamio
            .log_with_meta(frame, PartialMetadata::with_log_time(recv_us * 1000));
        act.frames += 1;
        let why = if act.file.failed.load(Ordering::Relaxed) {
            "write error"
        } else if act.file.bytes.load(Ordering::Relaxed) > self.max_bytes {
            "size cap"
        } else {
            return;
        };
        let reason = format!("recording {} stopped: {}", act.name, why);
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
        bytes: a.file.bytes.load(Ordering::Relaxed),
        seconds: a.started.elapsed().as_secs_f64(),
    }
}

/// What the recorder needs to know about the file under the MCAP writer. The
/// Foxglove SDK only logs write errors, so the file remembers them itself.
#[derive(Default)]
struct FileState {
    /// File size so far.
    bytes: AtomicU64,
    failed: AtomicBool,
}

struct TrackedFile {
    file: BufWriter<File>,
    pos: u64,
    state: Arc<FileState>,
}

impl TrackedFile {
    fn track<T>(&mut self, r: std::io::Result<T>) -> std::io::Result<T> {
        if r.is_err() {
            self.state.failed.store(true, Ordering::Relaxed);
        }
        self.state.bytes.fetch_max(self.pos, Ordering::Relaxed);
        r
    }
}

impl Write for TrackedFile {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let r = self.file.write(buf);
        if let Ok(n) = r {
            self.pos += n as u64;
        }
        self.track(r)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        let r = self.file.flush();
        self.track(r)
    }
}

impl Seek for TrackedFile {
    fn seek(&mut self, to: SeekFrom) -> std::io::Result<u64> {
        let r = self.file.seek(to);
        if let Ok(p) = r {
            self.pos = p;
        }
        self.track(r)
    }
}

/// `20261018T193116Z`: sorts by time, no characters a file system or URL minds.
fn compact_utc(t: SystemTime) -> String {
    humantime::format_rfc3339_seconds(t)
        .to_string()
        .chars()
        .filter(|c| *c != '-' && *c != ':')
        .collect()
}

/// Six hex digits from the OS: two runs started in the same second (or after
/// a reboot with a wrong clock) never get the same name.
fn random_hex() -> Result<String, String> {
    let mut b = [0u8; 4];
    File::open("/dev/urandom")
        .and_then(|mut f| f.read_exact(&mut b[1..]))
        .map_err(|e| format!("/dev/urandom: {}", e))?;
    Ok(format!("{:06x}", u32::from_be_bytes(b)))
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
        .unwrap()
    }

    fn temp_dir(what: &str) -> PathBuf {
        std::env::temp_dir().join(format!("dwar-{}-{}", what, random_hex().unwrap()))
    }

    #[test]
    fn records_mcap_with_schema_and_metadata() {
        let dir = temp_dir("rec");
        let r = recorder(&dir);
        let meta = BTreeMap::from([("robot".to_string(), "test".to_string())]);
        let i = r.start(Some("t/../x_y"), &meta).unwrap();
        let parts: Vec<&str> = i.name.trim_end_matches(".mcap").split('_').collect();
        assert_eq!(parts.len(), 3, "{}", i.name);
        assert_eq!(parts[1], "txy");
        r.write(1_000, b"abc");
        r.write(2_000, b"de");
        let done = r.stop().unwrap();
        assert_eq!(done.frames, 2);

        let bytes = std::fs::read(r.path_of(&done.name).unwrap()).unwrap();
        let msgs: Vec<_> = mcap::MessageStream::new(&bytes).unwrap().map(|m| m.unwrap()).collect();
        assert_eq!(msgs.len(), 2);
        assert_eq!(msgs[0].channel.topic, "/sangamio/message");
        assert_eq!(msgs[0].log_time, 1_000_000);
        assert_eq!(&msgs[1].data[..], b"de");
        let schema = msgs[0].channel.schema.as_ref().unwrap();
        assert_eq!((schema.name.as_str(), schema.encoding.as_str()), ("sangamio.Message", "protobuf"));

        let summary = mcap::Summary::read(&bytes).unwrap().unwrap();
        let idx = summary.metadata_indexes.iter().find(|m| m.name == "run").unwrap();
        let run = mcap::read::metadata(&bytes, idx).unwrap();
        assert_eq!(run.metadata["robot"], "test");
        assert_eq!(run.metadata["tag"], "txy");
        assert!(run.metadata.contains_key("started"));
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn refuses_odd_names_and_never_reuses_one() {
        let dir = temp_dir("names");
        let r = recorder(&dir);
        let a = r.start(None, &BTreeMap::new()).unwrap();
        r.stop();
        let b = r.start(None, &BTreeMap::new()).unwrap();
        r.stop();
        assert_ne!(a.name, b.name);
        assert!(r.path_of(&a.name).is_some());
        assert!(r.path_of("../etc/passwd").is_none());
        assert!(r.path_of("dwar.toml").is_none());
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn stops_at_the_size_cap_with_a_readable_file() {
        let dir = temp_dir("cap");
        let r = recorder(&dir);
        r.start(None, &BTreeMap::new()).unwrap();
        // Incompressible, so the file outgrows the 1 MB cap.
        let mut frame = vec![0u8; 64 * 1024];
        File::open("/dev/urandom").unwrap().read_exact(&mut frame).unwrap();
        let mut n = 0;
        while r.info().is_some() && n < 100 {
            r.write(0, &frame);
            n += 1;
        }
        assert!(r.info().is_none(), "never stopped");
        assert!(r.take_stopped_reason().unwrap().contains("size cap"));
        let name = &r.list()[0].name;
        let bytes = std::fs::read(r.path_of(name).unwrap()).unwrap();
        assert_eq!(mcap::MessageStream::new(&bytes).unwrap().count(), n);
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn a_failing_file_is_noticed() {
        let state = Arc::new(FileState::default());
        let mut t = TrackedFile {
            // Read-only: every flush fails.
            file: BufWriter::new(File::open("/dev/null").unwrap()),
            pos: 0,
            state: Arc::clone(&state),
        };
        let _ = t.write(b"x");
        assert!(t.flush().is_err());
        assert!(state.failed.load(Ordering::Relaxed));
    }

    #[test]
    fn compact_utc_sorts_and_is_plain() {
        let t = UNIX_EPOCH + std::time::Duration::from_secs(1_792_351_876);
        assert_eq!(compact_utc(t), "20261018T193116Z");
    }
}
