//! Recorded SangamIO streams, for `--replay` and offline tools (dhruva-lab):
//! raw frames (`for_each_frame`) or decoded sensor samples (`for_each_sample`).
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

use crate::io::sangam_udp_receiver::{LidarData, SangamUdpReceiver, SensorStatusData};

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

/// A recorded sensor sample, decoded for the SLAM pipeline.
#[derive(Debug, Clone)]
pub enum Sample {
    /// Wheel encoders, gyro, bumpers, cliffs (the `sensor_status` group).
    Status(SensorStatusData),
    /// One lidar revolution (the `lidar` group).
    Lidar(LidarData),
}

impl Sample {
    /// Data time of the sample.
    pub fn timestamp_us(&self) -> u64 {
        match self {
            Sample::Status(d) => d.timestamp_us,
            Sample::Lidar(d) => d.timestamp_us,
        }
    }
}

/// Call `f` with every `sensor_status` and `lidar` group of a recording, decoded,
/// in order. Other groups (kinematics, versions) and commands are skipped;
/// frames that don't decode, or sensor groups that don't parse, are skipped and
/// counted in one warning at the end.
pub fn for_each_sample(
    path: &Path,
    mut f: impl FnMut(Sample) -> std::io::Result<()>,
) -> std::io::Result<()> {
    use crate::io::sangam_client::proto::sangamio;
    use prost::Message as _;

    let mut malformed = 0usize;
    let result = for_each_frame(path, |body| {
        let Ok(msg) = sangamio::Message::decode(body) else {
            malformed += 1;
            return Ok(());
        };
        let Some(sangamio::message::Payload::SensorGroup(sg)) = msg.payload else {
            return Ok(());
        };
        let sample = match sg.group_id.as_str() {
            "sensor_status" => SangamUdpReceiver::parse_sensor_status(&sg).map(Sample::Status),
            "lidar" => SangamUdpReceiver::parse_lidar(&sg).map(Sample::Lidar),
            _ => return Ok(()),
        };
        match sample {
            Some(s) => f(s),
            None => {
                malformed += 1;
                Ok(())
            }
        }
    });
    if malformed > 0 {
        log::warn!("{}: skipped {} malformed frames", path.display(), malformed);
    }
    result
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

    fn group(id: &str, values: Vec<(&str, sangamio::sensor_value::Value)>) -> Vec<u8> {
        use prost::Message as _;
        let sg = sangamio::SensorGroup {
            group_id: id.into(),
            timestamp_us: 7,
            values: values
                .into_iter()
                .map(|(k, v)| (k.to_string(), sangamio::SensorValue { value: Some(v) }))
                .collect(),
        };
        sangamio::Message {
            topic: format!("sensors/{id}"),
            payload: Some(sangamio::message::Payload::SensorGroup(sg)),
        }
        .encode_to_vec()
    }

    use crate::io::sangam_client::proto::sangamio;
    use sangamio::sensor_value::Value;

    #[test]
    fn samples_are_decoded_in_order_and_the_rest_skipped() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("run_0001.mcap");
        // One bin at 0 rad, 1 m (4000 x 0.25 mm), quality 9.
        let scan = [&1u16.to_le_bytes()[..], &4000u16.to_le_bytes(), &[9]].concat();
        write_mcap(
            &path,
            &[
                group("kinematics", vec![("wheel_track_m", Value::F32Val(0.238))]),
                group(
                    "sensor_status",
                    vec![
                        ("wheel_left", Value::U32Val(1)),
                        ("wheel_right", Value::U32Val(2)),
                    ],
                ),
                b"not protobuf".to_vec(),
                group("lidar", vec![("scan_packed", Value::BytesVal(scan))]),
            ],
        );
        let mut got = Vec::new();
        for_each_sample(&path, |s| {
            got.push(s);
            Ok(())
        })
        .unwrap();
        assert_eq!(got.len(), 2);
        assert!(
            matches!(&got[0], Sample::Status(d) if (d.left_encoder, d.right_encoder) == (1, 2))
        );
        assert!(matches!(&got[1], Sample::Lidar(d) if d.points == vec![(0.0, 1.0, 9)]));
        assert_eq!(got[1].timestamp_us(), 7);

        // An error from the callback stops the read and is returned.
        let mut calls = 0;
        let r = for_each_sample(&path, |_| {
            calls += 1;
            Err(std::io::Error::other("stop"))
        });
        assert!(r.is_err() && calls == 1);
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
