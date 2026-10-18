//! Binary WebSocket frames (little-endian; the first byte is the kind).
//! Decoded by `web/src/protocol.ts`; documented in README.md.

use crate::map::{MapState, TILE};
use crate::proto::sangamio;
use axum::body::Bytes;

pub const FRAME_SCAN: u8 = 1;
pub const FRAME_TILE: u8 = 2;

/// Scan: `u8 1 | 3×0 | u32 seq | u64 t_us | f32 x | f32 y | f32 theta | packed scan`.
/// The pose is the latest SLAM pose when the scan arrived (NaN without SLAM),
/// so the client can draw the scan on the map.
pub fn scan_frame(seq: u32, t_us: u64, pose: Option<(f32, f32, f32)>, packed: &[u8]) -> Bytes {
    let (x, y, th) = pose.unwrap_or((f32::NAN, f32::NAN, f32::NAN));
    let mut b = Vec::with_capacity(28 + packed.len());
    b.extend_from_slice(&[FRAME_SCAN, 0, 0, 0]);
    b.extend_from_slice(&seq.to_le_bytes());
    b.extend_from_slice(&t_us.to_le_bytes());
    for v in [x, y, th] {
        b.extend_from_slice(&v.to_le_bytes());
    }
    b.extend_from_slice(packed);
    Bytes::from(b)
}

/// Tile: `u8 2 | 0 | u16 tx | u16 ty | u16 w | u16 h | u16 0 | u32 geometry | cells[w·h]`.
/// Cells are row-major from the grid origin: 0 free, 100 occupied, 255 unknown.
pub fn tile_frame(m: &MapState, tx: usize, ty: usize) -> Bytes {
    let x0 = tx * TILE;
    let y0 = ty * TILE;
    let tw = TILE.min(m.width - x0);
    let th = TILE.min(m.height - y0);
    let mut b = Vec::with_capacity(16 + tw * th);
    b.extend_from_slice(&[FRAME_TILE, 0]);
    for v in [tx, ty, tw, th, 0] {
        b.extend_from_slice(&(v as u16).to_le_bytes());
    }
    b.extend_from_slice(&(m.geometry as u32).to_le_bytes());
    for y in y0..y0 + th {
        b.extend_from_slice(&m.cells[y * m.width + x0..y * m.width + x0 + tw]);
    }
    Bytes::from(b)
}

/// The lidar scan as SangamIO's packed blob (`u16 n | n × (u16 range·4000 m, u8 q)`,
/// bin i at i·2π/n CCW from forward). The mock device sends a full point cloud
/// instead (the SLAM benchmarks use every ray); for display it is binned here the same way.
pub fn packed_scan(sg: &sangamio::SensorGroup) -> Option<Vec<u8>> {
    use sangamio::sensor_value::Value as V;
    if let Some(V::BytesVal(b)) = sg.values.get("scan_packed").and_then(|v| v.value.as_ref()) {
        let n = u16::from_le_bytes([*b.first()?, *b.get(1)?]) as usize;
        return (b.len() >= 2 + 3 * n).then(|| b.clone());
    }
    let Some(V::PointcloudVal(pc)) = sg.values.get("scan").and_then(|v| v.value.as_ref()) else {
        return None;
    };
    const BINS: usize = 360;
    let mut dist = [0u16; BINS];
    let mut qual = [0u8; BINS];
    for p in &pc.points {
        let a = p.angle_rad.rem_euclid(std::f32::consts::TAU);
        let bin = (a / std::f32::consts::TAU * BINS as f32) as usize % BINS;
        dist[bin] = (p.distance_m * 4000.0).round().clamp(0.0, u16::MAX as f32) as u16;
        qual[bin] = p.quality.min(255) as u8;
    }
    let mut b = Vec::with_capacity(2 + BINS * 3);
    b.extend_from_slice(&(BINS as u16).to_le_bytes());
    for i in 0..BINS {
        b.extend_from_slice(&dist[i].to_le_bytes());
        b.push(qual[i]);
    }
    Some(b)
}

/// Bins with a return in a packed scan.
pub fn packed_points(b: &[u8]) -> usize {
    b.get(2..)
        .unwrap_or_default()
        .as_chunks::<3>()
        .0
        .iter()
        .filter(|c| c[0] != 0 || c[1] != 0)
        .count()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scan_frame_layout() {
        let packed = [2u8, 0, 0xA0, 0x0F, 7, 0, 0, 0];
        let f = scan_frame(5, 9, Some((1.0, 2.0, 3.0)), &packed);
        assert_eq!(f[0], FRAME_SCAN);
        assert_eq!(u32::from_le_bytes(f[4..8].try_into().unwrap()), 5);
        assert_eq!(f32::from_le_bytes(f[20..24].try_into().unwrap()), 2.0);
        assert_eq!(&f[28..], &packed);
        assert_eq!(packed_points(&packed), 1);
    }

    #[test]
    fn short_packed_scan_is_rejected() {
        use sangamio::{SensorGroup, SensorValue, sensor_value::Value};
        let mut sg = SensorGroup::default();
        sg.values.insert(
            "scan_packed".into(),
            SensorValue { value: Some(Value::BytesVal(vec![10, 0, 1, 2])) },
        );
        assert!(packed_scan(&sg).is_none());
    }
}
