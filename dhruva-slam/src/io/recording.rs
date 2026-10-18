//! Recorded SangamIO streams, for `--replay`.
//!
//! - MCAP (https://mcap.dev), as dwar records them: the `/sangamio/message`
//!   channel. A directory is one run split into segments, read in name order
//!   (dwar zero-pads the segment number).
//! - `SGIO1` (older captures from tools/v8rec and tools/py/sangam_client.py):
//!   the `b"SGIO1\n"` header, then `u64 recv_us | u32 len | protobuf Message`
//!   per frame, little-endian.
//!
//! Frames come in file order, which is arrival order (one writer). A file cut
//! short by a crash, or still being written, ends at its last whole chunk
//! (MCAP) or frame (SGIO1) with a warning; any other read error is fatal.

use std::io::{BufRead, BufReader, Read};
use std::path::{Path, PathBuf};

/// The channel dwar records the raw SangamIO stream on.
const SANGAMIO_TOPIC: &str = "/sangamio/message";

/// Call `f` with every recorded SangamIO frame (protobuf `Message` bytes), in
/// order. An error from `f` stops the reading and is returned.
pub fn for_each_frame(
    path: &Path,
    mut f: impl FnMut(&[u8]) -> std::io::Result<()>,
) -> std::io::Result<()> {
    if path.is_dir() {
        for segment in mcap_files(path)? {
            mcap_frames(&segment, &mut f)?;
        }
        return Ok(());
    }
    let mut magic = [0u8; 8];
    let n = std::fs::File::open(path)?.read(&mut magic)?;
    if magic[..n] == *mcap::MAGIC {
        mcap_frames(path, &mut f)
    } else {
        sgio_frames(path, &mut f)
    }
}

fn mcap_files(dir: &Path) -> std::io::Result<Vec<PathBuf>> {
    let mut files = Vec::new();
    for entry in std::fs::read_dir(dir)? {
        let path = entry?.path();
        if path.extension().is_some_and(|e| e == "mcap") {
            files.push(path);
        }
    }
    if files.is_empty() {
        return Err(std::io::Error::other(format!(
            "no .mcap files in {}",
            dir.display()
        )));
    }
    files.sort();
    Ok(files)
}

/// One MCAP file, read whole: dwar's segments are ~8 MB, and mapping a file
/// that may still be growing is asking for trouble.
fn mcap_frames(
    path: &Path,
    f: &mut impl FnMut(&[u8]) -> std::io::Result<()>,
) -> std::io::Result<()> {
    let bytes = std::fs::read(path)?;
    let bad = |e: mcap::McapError| std::io::Error::other(format!("{}: {}", path.display(), e));
    let mut frames = 0usize;
    for msg in mcap::MessageStream::new(&bytes).map_err(bad)? {
        let msg = match msg {
            Ok(m) => m,
            Err(mcap::McapError::UnexpectedEof) => {
                log::warn!(
                    "{}: cut short (crash or still recording), using its first {} frames",
                    path.display(),
                    frames
                );
                break;
            }
            Err(e) => return Err(bad(e)),
        };
        if msg.channel.topic == SANGAMIO_TOPIC {
            f(&msg.data)?;
            frames += 1;
        }
    }
    Ok(())
}

fn sgio_frames(
    path: &Path,
    f: &mut impl FnMut(&[u8]) -> std::io::Result<()>,
) -> std::io::Result<()> {
    let mut rd = BufReader::new(std::fs::File::open(path)?);
    let mut magic = Vec::new();
    rd.read_until(b'\n', &mut magic)?;
    if magic != b"SGIO1\n" {
        return Err(std::io::Error::other(format!(
            "{}: neither MCAP nor SGIO1",
            path.display()
        )));
    }
    let mut hdr = [0u8; 12];
    let mut body = Vec::new();
    while rd.read_exact(&mut hdr).is_ok() {
        let n = u32::from_le_bytes([hdr[8], hdr[9], hdr[10], hdr[11]]) as usize;
        body.resize(n, 0);
        if rd.read_exact(&mut body).is_err() {
            log::warn!("{}: cut short mid-frame", path.display());
            break;
        }
        f(&body)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn frames(path: &Path) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        for_each_frame(path, |b| {
            out.push(b.to_vec());
            Ok(())
        })
        .unwrap();
        out
    }

    /// An MCAP like dwar's: zstd chunks, plus a channel replay must skip.
    fn write_mcap(path: &Path, payloads: &[Vec<u8>]) {
        let file = std::io::BufWriter::new(std::fs::File::create(path).unwrap());
        let mut w = mcap::WriteOptions::new()
            .chunk_size(Some(4096))
            .create(file)
            .unwrap();
        let ch = w
            .add_channel(0, SANGAMIO_TOPIC, "protobuf", &Default::default())
            .unwrap();
        let other = w
            .add_channel(0, "/other", "json", &Default::default())
            .unwrap();
        for (i, p) in payloads.iter().enumerate() {
            let header = mcap::records::MessageHeader {
                channel_id: ch,
                sequence: i as u32,
                log_time: i as u64,
                publish_time: i as u64,
            };
            w.write_to_known_channel(&header, p).unwrap();
            let header = mcap::records::MessageHeader {
                channel_id: other,
                ..header
            };
            w.write_to_known_channel(&header, b"{}").unwrap();
        }
        w.finish().unwrap();
    }

    fn payloads(range: std::ops::Range<u32>) -> Vec<Vec<u8>> {
        range.map(|i| i.to_le_bytes().repeat(64)).collect()
    }

    #[test]
    fn reads_mcap_files_and_run_directories_in_order() {
        let dir = tempfile::tempdir().unwrap();
        write_mcap(&dir.path().join("run_0002.mcap"), &payloads(2..3));
        write_mcap(&dir.path().join("run_0001.mcap"), &payloads(0..2));
        // Detected by content, not by name.
        let renamed = dir.path().join("first");
        std::fs::copy(dir.path().join("run_0001.mcap"), &renamed).unwrap();
        assert_eq!(frames(&renamed), payloads(0..2));
        assert_eq!(frames(dir.path()), payloads(0..3));
    }

    #[test]
    fn reads_mcap_up_to_the_last_whole_chunk() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("run_0001.mcap");
        write_mcap(&path, &payloads(0..1000));
        let bytes = std::fs::read(&path).unwrap();
        std::fs::write(&path, &bytes[..bytes.len() * 2 / 3]).unwrap();
        let got = frames(&path);
        assert!(!got.is_empty() && got.len() < 1000, "{} frames", got.len());
        assert_eq!(got, payloads(0..got.len() as u32));
    }

    #[test]
    fn an_empty_run_directory_is_an_error() {
        let dir = tempfile::tempdir().unwrap();
        assert!(for_each_frame(dir.path(), |_| Ok(())).is_err());
    }

    #[test]
    fn reads_sgio_up_to_the_last_whole_frame() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("x.sgio");
        let mut f = std::fs::File::create(&path).unwrap();
        f.write_all(b"SGIO1\n").unwrap();
        for body in [&b"abc"[..], b"de"] {
            f.write_all(&7u64.to_le_bytes()).unwrap();
            f.write_all(&(body.len() as u32).to_le_bytes()).unwrap();
            f.write_all(body).unwrap();
        }
        f.write_all(&[1, 2, 3]).unwrap(); // torn header
        drop(f);
        assert_eq!(frames(&path), vec![b"abc".to_vec(), b"de".to_vec()]);
    }
}
