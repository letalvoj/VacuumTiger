//! The step-by-step trace: what the engine believed after every scan, as MCAP
//! with Foxglove schemas (open it in Foxglove or Lichtblick and press play).
//!
//! | Topic | Schema | What |
//! | --- | --- | --- |
//! | `/tf` | FrameTransforms | REP-105: `odom → base_link` (raw odometry), `map → odom` (the SLAM correction) |
//! | `/scan` | PointCloud in `base_link` | the preprocessed scan the engine matched; set a decay time to see the history fade |
//! | `/belief` | SceneUpdate | pose uncertainty (2σ ellipse) at the SLAM pose |
//! | `/path` | SceneUpdate | trajectory so far: SLAM (blue), odometry (orange) |
//! | `/graph` | SceneUpdate | pose graph (loop closure on): sequential edges (grey), loop closures (red) |
//! | `/map` | Grid (RGBA) | the global occupancy map, when it changed, at most every `--map-every` |
//! | `/step` | JSON | per-scan numbers for plots: match score, refinement, keyframes, closures |

use std::fs::File;
use std::io::BufWriter;
use std::path::Path;
use std::sync::Arc;

use dhruva_slam::algorithms::mapping::{CellState, OccupancyGrid};
use dhruva_slam::core::types::Pose2D;
use dhruva_slam::engine::graph::{EdgeType, PoseGraph};
use dhruva_slam::engine::pipeline::ScanStep;
use dhruva_slam::engine::slam::SlamResult;
use foxglove::messages::{
    Color, CylinderPrimitive, FrameTransform, FrameTransforms, Grid, LinePrimitive,
    PackedElementField, Point3, PointCloud, Pose, Quaternion, SceneEntity, SceneUpdate, Timestamp,
    Vector2, Vector3, line_primitive, packed_element_field::NumericType,
};
use foxglove::{
    Channel, Context, McapCompression, McapWriteOptions, McapWriterHandle, PartialMetadata,
    RawChannel, Schema,
};

const MAP_FRAME: &str = "map";
const ODOM_FRAME: &str = "odom";
const ROBOT_FRAME: &str = "base_link";
/// Trajectory lines are re-sent every this many scans (they grow with the run).
const PATH_EVERY_SCANS: usize = 10;

/// JSON schema of `/step`; keep in step with `Trace::scan`.
const STEP_SCHEMA: &str = r#"{"type":"object","properties":{
"score":{"type":"number"},"refined":{"type":"boolean"},"keyframe":{"type":"boolean"},
"keyframes":{"type":"integer"},"loop_closures":{"type":"integer"},"points":{"type":"integer"},
"odom_dtheta_deg":{"type":"number"},"slam_dtheta_deg":{"type":"number"},"ms":{"type":"number"}}}"#;

/// Writes the trace while the replay runs.
pub struct Trace {
    /// Owns the channels: they close when it is dropped.
    _context: Arc<Context>,
    mcap: McapWriterHandle<BufWriter<File>>,
    tf: Channel<FrameTransforms>,
    scan: Channel<PointCloud>,
    belief: Channel<SceneUpdate>,
    path: Channel<SceneUpdate>,
    graph: Channel<SceneUpdate>,
    map: Channel<Grid>,
    step: Arc<RawChannel>,
    slam_path: Vec<Point3>,
    odom_path: Vec<Point3>,
    /// Size and pose checksum of the last `/graph` sent.
    graph_signature: (usize, usize, u64),
    /// Checksum of the last `/map` sent.
    map_signature: u64,
    last_pose: Option<Pose2D>,
    last_t_us: u64,
}

impl Trace {
    pub fn create(path: &Path) -> Result<Self, String> {
        let context = Context::new();
        let file = File::create(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let mcap = context
            .mcap_writer_with_options(
                McapWriteOptions::new().compression(Some(McapCompression::Zstd)),
            )
            .create(BufWriter::new(file))
            .map_err(|e| format!("{}: {e}", path.display()))?;
        let step = context
            .channel_builder("/step")
            .message_encoding("json")
            .schema(Schema::new(
                "dhruva_lab.Step",
                "jsonschema",
                STEP_SCHEMA.as_bytes(),
            ))
            .build_raw()
            .map_err(|e| format!("/step: {e}"))?;
        Ok(Self {
            tf: context.channel_builder("/tf").build(),
            scan: context.channel_builder("/scan").build(),
            belief: context.channel_builder("/belief").build(),
            path: context.channel_builder("/path").build(),
            graph: context.channel_builder("/graph").build(),
            map: context.channel_builder("/map").build(),
            step,
            _context: context,
            mcap,
            slam_path: Vec::new(),
            odom_path: Vec::new(),
            graph_signature: (0, 0, 0),
            map_signature: 0,
            last_pose: None,
            last_t_us: 0,
        })
    }

    /// Record one processed scan and the engine state after it.
    pub fn scan(
        &mut self,
        step: &ScanStep,
        result: &SlamResult,
        graph: &PoseGraph,
        keyframes: usize,
        ms: f64,
    ) {
        let t = step.timestamp_us;
        let meta = || PartialMetadata::with_log_time(t * 1000);
        let pose = result.pose;
        self.last_t_us = t;

        // REP-105: odom -> base_link is the raw odometry, map -> odom the
        // correction SLAM applies to it.
        let correction = pose.compose(&step.odom_pose.inverse());
        self.tf.log_with_meta(
            &FrameTransforms {
                transforms: vec![
                    transform(t, MAP_FRAME, ODOM_FRAME, &correction),
                    transform(t, ODOM_FRAME, ROBOT_FRAME, &step.odom_pose),
                ],
            },
            meta(),
        );

        let mut data = Vec::with_capacity(step.cloud.len() * 8);
        for (x, y) in step.cloud.xs.iter().zip(&step.cloud.ys) {
            data.extend_from_slice(&x.to_le_bytes());
            data.extend_from_slice(&y.to_le_bytes());
        }
        self.scan.log_with_meta(
            &PointCloud {
                timestamp: Some(timestamp(t)),
                frame_id: ROBOT_FRAME.into(),
                pose: Some(pose3(&Pose2D::identity())),
                point_stride: 8,
                fields: vec![field("x", 0), field("y", 4)],
                data: data.into(),
            },
            meta(),
        );

        self.belief
            .log_with_meta(&scene("belief", t, vec![], vec![ellipse(result)]), meta());

        self.slam_path.push(point(pose.x, pose.y));
        self.odom_path
            .push(point(step.odom_pose.x, step.odom_pose.y));
        if self.slam_path.len() % PATH_EVERY_SCANS == 1 {
            self.log_path();
        }

        // Re-sent when it grows and when an optimisation moves its nodes.
        let signature = (
            graph.num_nodes(),
            graph.num_edges(),
            graph
                .nodes()
                .iter()
                .map(|n| (n.pose.x + n.pose.y + n.pose.theta) as f64)
                .sum::<f64>()
                .to_bits(),
        );
        if signature != self.graph_signature {
            self.graph_signature = signature;
            self.graph
                .log_with_meta(&scene("graph", t, graph_lines(graph), vec![]), meta());
        }

        let slam_dtheta = self
            .last_pose
            .map(|p| angle_diff(pose.theta, p.theta))
            .unwrap_or(0.0);
        self.last_pose = Some(pose);
        let json = serde_json::json!({
            "score": result.match_score,
            "refined": result.pose_refined,
            "keyframe": result.keyframe_created,
            "keyframes": keyframes,
            "loop_closures": graph.num_loop_closures(),
            "points": step.cloud.len(),
            "odom_dtheta_deg": step.odom_delta.theta.to_degrees(),
            "slam_dtheta_deg": slam_dtheta.to_degrees(),
            "ms": ms,
        });
        self.step.log_with_meta(json.to_string().as_bytes(), meta());
    }

    /// Record the global occupancy map, unless it is unchanged since the last one.
    pub fn map(&mut self, t_us: u64, grid: &OccupancyGrid) {
        let (w, h) = grid.dimensions();
        let mut data = Vec::with_capacity(w * h * 4);
        for cy in 0..h {
            for cx in 0..w {
                data.extend_from_slice(match grid.get_state(cx, cy) {
                    CellState::Occupied => &[20, 20, 20, 255],
                    CellState::Free => &[235, 235, 235, 160],
                    CellState::Unknown => &[0, 0, 0, 0],
                });
            }
        }
        let (ox, oy) = grid.origin();
        let signature =
            checksum(&data) ^ (w as u64) << 32 ^ ox.to_bits() as u64 ^ oy.to_bits() as u64;
        if signature == self.map_signature {
            return;
        }
        self.map_signature = signature;
        let res = grid.resolution() as f64;
        let fields = ["red", "green", "blue", "alpha"]
            .iter()
            .enumerate()
            .map(|(i, n)| PackedElementField {
                name: (*n).into(),
                offset: i as u32,
                r#type: NumericType::Uint8 as i32,
            })
            .collect();
        self.map.log_with_meta(
            &Grid {
                timestamp: Some(timestamp(t_us)),
                frame_id: MAP_FRAME.into(),
                pose: Some(pose3(&Pose2D::new(ox, oy, 0.0))),
                column_count: w as u32,
                cell_size: Some(Vector2 { x: res, y: res }),
                row_stride: (w * 4) as u32,
                cell_stride: 4,
                fields,
                data: data.into(),
            },
            PartialMetadata::with_log_time(t_us * 1000),
        );
    }

    pub fn finish(mut self) -> Result<(), String> {
        self.log_path();
        self.mcap
            .close()
            .map(|_| ())
            .map_err(|e| format!("trace: {e}"))
    }

    fn log_path(&mut self) {
        let lines = vec![
            line(
                line_primitive::Type::LineStrip,
                self.slam_path.clone(),
                rgba(0.2, 0.5, 1.0, 1.0),
            ),
            line(
                line_primitive::Type::LineStrip,
                self.odom_path.clone(),
                rgba(1.0, 0.6, 0.1, 0.8),
            ),
        ];
        let t = self.last_t_us;
        self.path.log_with_meta(
            &scene("path", t, lines, vec![]),
            PartialMetadata::with_log_time(t * 1000),
        );
    }
}

/// 2σ ellipse of the position covariance, oriented along its eigenvectors.
fn ellipse(result: &SlamResult) -> CylinderPrimitive {
    let c = result.covariance.as_slice();
    let (sxx, sxy, syy) = (c[0] as f64, c[1] as f64, c[4] as f64);
    let mean = (sxx + syy) / 2.0;
    let d = (((sxx - syy) / 2.0).powi(2) + sxy * sxy).sqrt();
    let (l1, l2) = (mean + d, (mean - d).max(0.0));
    let angle = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    let p = result.pose;
    CylinderPrimitive {
        pose: Some(pose3(&Pose2D::new(p.x, p.y, angle as f32))),
        // Cylinder size is a diameter: 2 x 2σ.
        size: Some(Vector3 {
            x: 4.0 * l1.sqrt().max(0.005),
            y: 4.0 * l2.sqrt().max(0.005),
            z: 0.01,
        }),
        bottom_scale: 1.0,
        top_scale: 1.0,
        color: Some(rgba(0.2, 0.6, 1.0, 0.35)),
    }
}

fn graph_lines(graph: &PoseGraph) -> Vec<LinePrimitive> {
    let (mut seq, mut closures) = (Vec::new(), Vec::new());
    for e in graph.edges() {
        let (Some(a), Some(b)) = (graph.get_node(e.from), graph.get_node(e.to)) else {
            continue;
        };
        let pts = [point(a.pose.x, a.pose.y), point(b.pose.x, b.pose.y)];
        match e.edge_type {
            EdgeType::LoopClosure => closures.extend(pts),
            _ => seq.extend(pts),
        }
    }
    vec![
        line(
            line_primitive::Type::LineList,
            seq,
            rgba(0.6, 0.6, 0.6, 0.8),
        ),
        line(
            line_primitive::Type::LineList,
            closures,
            rgba(1.0, 0.1, 0.1, 1.0),
        ),
    ]
}

fn scene(
    id: &str,
    t_us: u64,
    lines: Vec<LinePrimitive>,
    cylinders: Vec<CylinderPrimitive>,
) -> SceneUpdate {
    SceneUpdate {
        deletions: vec![],
        entities: vec![SceneEntity {
            timestamp: Some(timestamp(t_us)),
            frame_id: MAP_FRAME.into(),
            id: id.into(),
            lines,
            cylinders,
            ..Default::default()
        }],
    }
}

fn line(kind: line_primitive::Type, points: Vec<Point3>, color: Color) -> LinePrimitive {
    LinePrimitive {
        r#type: kind as i32,
        pose: Some(pose3(&Pose2D::identity())),
        thickness: 2.0,
        scale_invariant: true,
        points,
        color: Some(color),
        ..Default::default()
    }
}

fn field(name: &str, offset: u32) -> PackedElementField {
    PackedElementField {
        name: name.into(),
        offset,
        r#type: NumericType::Float32 as i32,
    }
}

fn transform(t_us: u64, parent: &str, child: &str, p: &Pose2D) -> FrameTransform {
    let pose = pose3(p);
    FrameTransform {
        timestamp: Some(timestamp(t_us)),
        parent_frame_id: parent.into(),
        child_frame_id: child.into(),
        translation: pose.position,
        rotation: pose.orientation,
    }
}

fn pose3(p: &Pose2D) -> Pose {
    let half = p.theta as f64 / 2.0;
    Pose {
        position: Some(Vector3 {
            x: p.x as f64,
            y: p.y as f64,
            z: 0.0,
        }),
        orientation: Some(Quaternion {
            x: 0.0,
            y: 0.0,
            z: half.sin(),
            w: half.cos(),
        }),
    }
}

fn point(x: f32, y: f32) -> Point3 {
    Point3 {
        x: x as f64,
        y: y as f64,
        z: 0.0,
    }
}

fn rgba(r: f64, g: f64, b: f64, a: f64) -> Color {
    Color { r, g, b, a }
}

fn timestamp(t_us: u64) -> Timestamp {
    Timestamp::new(
        (t_us / 1_000_000) as u32,
        ((t_us % 1_000_000) * 1000) as u32,
    )
}

fn angle_diff(a: f32, b: f32) -> f32 {
    let d = a - b;
    d.sin().atan2(d.cos())
}

/// FNV-1a, to skip resending an unchanged map.
fn checksum(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf29ce484222325, |h, b| {
        (h ^ *b as u64).wrapping_mul(0x100000001b3)
    })
}
