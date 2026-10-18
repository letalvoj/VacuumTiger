//! REST API, `/api/v1`. JSON in and out; anything continuous is on the
//! WebSocket (`ws.rs`).

use crate::hub::{Action, Hub};
use crate::proto::dhruva::{self, dhruva_command::Command as Dc};
use crate::proto::sangamio::sensor_value::Value as V;
use axum::body::Body;
use axum::extract::{Path, Request, State};
use axum::http::{HeaderMap, HeaderValue, Method, StatusCode, header};
use axum::middleware::Next;
use axum::response::{IntoResponse, Response};
use axum::routing::{delete, get, post};
use axum::{Json, Router};
use serde::Deserialize;
use serde_json::{Value, json};
use std::sync::Arc;

type St = State<Arc<Hub>>;
type ApiError = (StatusCode, Json<Value>);
type ApiResult = Result<Json<Value>, ApiError>;


pub fn router(hub: Arc<Hub>) -> Router {
    Router::new()
        .route("/robot", get(robot))
        .route("/status", get(status))
        .route("/state", get(state))
        .route("/capabilities", get(capabilities))
        .route("/stop", post(stop))
        .route("/lease", post(take_lease).delete(release_lease))
        .route("/actuators/{id}", post(actuator))
        .route("/slam/{action}", post(slam))
        .route("/maps", get(maps))
        .route("/maps/current.pgm", get(map_pgm))
        .route("/maps/{id}/enable", post(map_enable))
        .route("/maps/{id}", delete(map_delete))
        .route("/maps/{id}/segments", get(segments))
        .route("/recordings", get(recordings))
        .route("/recordings/start", post(rec_start))
        .route("/recordings/stop", post(rec_stop))
        .route("/recordings/{name}", get(rec_download).delete(rec_delete))
        .route("/ws", get(crate::ws::upgrade))
        .fallback(|| async { err(StatusCode::NOT_FOUND, "no such endpoint") })
        .layer(axum::middleware::from_fn_with_state(Arc::clone(&hub), guard))
        .with_state(hub)
}

/// Same-origin check and the optional token.
///
/// - A browser on another site must not drive the robot through the user's
///   browser (CSRF / cross-site WebSocket): state-changing requests and the
///   WebSocket upgrade are refused when `Origin` names another host.
/// - `[http] token`: `Authorization: Bearer <token>`, or `?token=` (a browser
///   can't put headers on a WebSocket or a download link).
async fn guard(State(hub): St, req: Request, next: Next) -> Response {
    let headers = req.headers();
    let mutating = req.method() != Method::GET || headers.contains_key(header::UPGRADE);
    if mutating && !same_origin(headers) {
        return err(StatusCode::FORBIDDEN, "cross-origin request refused").into_response();
    }
    let Some(token) = hub.cfg.http.token.as_deref() else {
        if !trusted_host(headers, &hub.cfg.http.allowed_hosts) {
            return err(StatusCode::FORBIDDEN, "unknown Host (set [http] allowed_hosts or a token)").into_response();
        }
        return next.run(req).await;
    };
    let bearer = headers
        .get(header::AUTHORIZATION)
        .and_then(|v| v.to_str().ok())
        .and_then(|v| v.strip_prefix("Bearer "));
    let query = req.uri().query().and_then(|q| {
        form_urlencoded::parse(q.as_bytes()).find_map(|(k, v)| (k == "token").then(|| v.into_owned()))
    });
    let matches = |t: Option<&str>| t.is_some_and(|t| constant_time_eq(t.as_bytes(), token.as_bytes()));
    if !matches(bearer) && !matches(query.as_deref()) {
        return err(StatusCode::UNAUTHORIZED, "token required").into_response();
    }
    next.run(req).await
}

/// Without a token, the Host must be an IP literal, localhost, a `.local`
/// name or explicitly allowed: a DNS-rebinding page names its own domain.
fn trusted_host(headers: &HeaderMap, allowed: &[String]) -> bool {
    let Some(host) = headers.get(header::HOST).and_then(|v| v.to_str().ok()) else {
        return false;
    };
    let name = match host.rsplit_once(':') {
        Some((n, port)) if port.bytes().all(|b| b.is_ascii_digit()) => n,
        _ => host,
    };
    let name = name.trim_start_matches('[').trim_end_matches(']');
    name.parse::<std::net::IpAddr>().is_ok()
        || name == "localhost"
        || name.ends_with(".local")
        || allowed.iter().any(|a| a.eq_ignore_ascii_case(name))
}

fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    a.len() == b.len() && a.iter().zip(b).fold(0u8, |acc, (x, y)| acc | (x ^ y)) == 0
}

/// True unless an `Origin` header names a different host than `Host`.
/// Non-browser clients (curl, scripts) send no `Origin`.
fn same_origin(headers: &HeaderMap) -> bool {
    let Some(origin) = headers.get(header::ORIGIN).and_then(|v| v.to_str().ok()) else {
        return true;
    };
    let host = headers.get(header::HOST).and_then(|v| v.to_str().ok());
    let origin_host = origin.split_once("://").map(|(_, rest)| rest);
    origin_host.is_some() && origin_host == host
}

fn err(code: StatusCode, msg: impl Into<String>) -> ApiError {
    (code, Json(json!({ "error": msg.into() })))
}

fn ok(v: Value) -> ApiResult {
    Ok(Json(v))
}

async fn robot(State(hub): St) -> Json<Value> {
    let s = hub.shared();
    Json(json!({
        "robot": hub.cfg.robot,
        "firmware": "VacuumTiger",
        "dwar_version": env!("CARGO_PKG_VERSION"),
        "versions": s.version,
    }))
}

async fn status(State(hub): St) -> Json<Value> {
    Json(hub.status_json())
}

async fn state(State(hub): St) -> Json<Value> {
    let s = hub.shared();
    Json(json!({
        "t_us": s.sensors_t_us,
        "values": s.sensors,
        "pose": s.pose.map(|(x, y, theta)| json!({"x": x, "y": y, "theta": theta})),
        "slam": s.robot_status,
    }))
}

/// Feature flags the UI renders from, so a UI never offers what this robot
/// or configuration can't do.
async fn capabilities(State(hub): St) -> Json<Value> {
    let c = &hub.cfg.control;
    let mut caps = vec!["sensors", "lidar", "stop", "recordings"];
    if hub.cfg.dhruva.enabled {
        caps.extend(["slam", "map"]);
    }
    if c.allow_drive {
        caps.push("teleop");
    }
    if c.allow_actuators {
        caps.push("actuators");
    }
    if c.allow_drive || c.allow_actuators {
        caps.push("lease");
    }
    Json(json!({ "capabilities": caps }))
}

async fn stop(State(hub): St) -> ApiResult {
    match hub.stop_all("REST /stop") {
        Ok(()) => ok(json!({ "ok": true })),
        Err(e) => Err(err(StatusCode::SERVICE_UNAVAILABLE, e)),
    }
}

#[derive(Deserialize, Default)]
struct LeaseReq {
    #[serde(default)]
    force: bool,
}

fn lease_allowed(hub: &Hub) -> Result<(), ApiError> {
    let c = &hub.cfg.control;
    if c.allow_drive || c.allow_actuators {
        Ok(())
    } else {
        Err(err(StatusCode::FORBIDDEN, "lease control disabled (dwar [control])"))
    }
}

/// Take SangamIO's control lease for dwar (`force` takes it from another
/// client such as a SLAM controller; SangamIO zeroes the drive on takeover).
/// SangamIO sends no reply, so this only reports that the request was queued.
async fn take_lease(State(hub): St, body: Option<Json<LeaseReq>>) -> ApiResult {
    lease_allowed(&hub)?;
    let force = body.is_some_and(|b| b.force);
    if force && !hub.cfg.control.allow_drive {
        // Taking the lease from a SLAM controller zeroes its drive mid-run.
        return Err(err(StatusCode::FORBIDDEN, "force needs [control] allow_drive"));
    }
    if !hub.sangam("lease", Action::Enable, &[("force", V::BoolVal(force))]) {
        return Err(err(StatusCode::SERVICE_UNAVAILABLE, "SangamIO command queue full"));
    }
    hub.event("info", format!("control lease requested (force={})", force));
    ok(json!({ "queued": true, "force": force }))
}

async fn release_lease(State(hub): St) -> ApiResult {
    lease_allowed(&hub)?;
    hub.sangam("lease", Action::Disable, &[]);
    ok(json!({ "queued": true }))
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ActuatorReq {
    enabled: bool,
    /// 0-100 % for vacuum/brushes/pump.
    speed: Option<u8>,
    /// LED pattern 0-18.
    state: Option<u8>,
}

async fn actuator(State(hub): St, Path(id): Path<String>, Json(req): Json<ActuatorReq>) -> ApiResult {
    let speedy = crate::control::CLEANING.contains(&id.as_str());
    let allowed = id == "lidar" || (hub.cfg.control.allow_actuators && (speedy || id == "led"));
    if !allowed {
        return Err(err(StatusCode::FORBIDDEN, format!("'{}' not allowed (dwar [control] allow_actuators)", id)));
    }
    if !hub.shared().sangam_connected {
        return Err(err(StatusCode::SERVICE_UNAVAILABLE, "SangamIO offline"));
    }
    if req.speed.is_some() && !speedy {
        return Err(err(StatusCode::BAD_REQUEST, format!("'{}' has no speed", id)));
    }
    if req.speed.is_some_and(|s| s > 100) {
        return Err(err(StatusCode::BAD_REQUEST, "speed is 0-100"));
    }
    let queued = match (id.as_str(), req.enabled) {
        ("led", on) => {
            let st = if on { req.state.unwrap_or(1) } else { 0 };
            if st > 18 {
                return Err(err(StatusCode::BAD_REQUEST, "led state is 0-18"));
            }
            hub.sangam("led", Action::Configure, &[("state", V::U32Val(st as u32))])
        }
        (_, false) => hub.sangam(&id, Action::Disable, &[]),
        (_, true) => {
            let mut q = hub.sangam(&id, Action::Enable, &[]);
            if let Some(speed) = req.speed {
                q &= hub.sangam(&id, Action::Configure, &[("speed", V::U32Val(speed as u32))]);
            }
            q
        }
    };
    if !queued {
        return Err(err(StatusCode::SERVICE_UNAVAILABLE, "SangamIO command queue full"));
    }
    let v = json!({ "enabled": req.enabled, "speed": req.speed, "state": req.state });
    hub.shared().actuators.insert(id.clone(), v.clone());
    hub.event("info", format!("{} -> {}", id, v));
    ok(json!({ "queued": true, "id": id, "requested": v }))
}

#[derive(Deserialize, Default)]
struct SlamReq {
    name: Option<String>,
    save: Option<bool>,
}

async fn slam(State(hub): St, Path(action): Path<String>, body: Option<Json<SlamReq>>) -> ApiResult {
    if !hub.cfg.dhruva.enabled {
        return Err(err(StatusCode::SERVICE_UNAVAILABLE, "SLAM disabled in dwar config"));
    }
    let req = body.map(|b| b.0).unwrap_or_default();
    let cmd = match action.as_str() {
        "start_mapping" => Dc::StartMapping(dhruva::StartMappingCommand {
            map_name: req.name.unwrap_or_default(),
        }),
        "stop_mapping" => Dc::StopMapping(dhruva::StopMappingCommand {
            save: req.save.unwrap_or(true),
        }),
        "clear_map" => Dc::ClearMap(dhruva::ClearMapCommand {}),
        _ => return Err(err(StatusCode::NOT_FOUND, format!("unknown SLAM action '{}'", action))),
    };
    dhruva_cmd(&hub, cmd)
}

fn dhruva_cmd(hub: &Hub, cmd: Dc) -> ApiResult {
    match hub.dhruva(cmd) {
        // The response arrives asynchronously as an event on the WebSocket.
        Ok(id) => ok(json!({ "queued": true, "request_id": id })),
        Err(e) => Err(err(StatusCode::SERVICE_UNAVAILABLE, e)),
    }
}

async fn maps(State(hub): St) -> Json<Value> {
    let maps = hub.shared().maps.clone();
    let current = hub.map.borrow().as_ref().map(|m| m.meta_json());
    Json(json!({ "maps": maps, "current": current }))
}

async fn map_pgm(State(hub): St) -> Response {
    let Some(m) = hub.map.borrow().clone() else {
        return err(StatusCode::NOT_FOUND, "no map yet").into_response();
    };
    ([(header::CONTENT_TYPE, "image/x-portable-graymap")], m.pgm()).into_response()
}

async fn map_enable(State(hub): St, Path(id): Path<String>) -> ApiResult {
    dhruva_cmd(&hub, Dc::EnableMap(dhruva::EnableMapCommand { map_id: id }))
}

async fn map_delete(State(hub): St, Path(id): Path<String>) -> ApiResult {
    dhruva_cmd(&hub, Dc::DeleteMap(dhruva::DeleteMapCommand { map_id: id }))
}

/// Reserved for room segmentation (the "see rooms, clean a room" UI).
async fn segments(Path(_id): Path<String>) -> ApiError {
    err(StatusCode::NOT_IMPLEMENTED, "map segmentation not implemented yet")
}

async fn recordings(State(hub): St) -> Json<Value> {
    Json(json!({
        "dir": hub.recorder.dir().display().to_string(),
        "active": hub.recorder.info(),
        "files": hub.recorder.list(),
    }))
}

#[derive(Deserialize, Default)]
struct RecReq {
    tag: Option<String>,
}

async fn rec_start(State(hub): St, body: Option<Json<RecReq>>) -> ApiResult {
    let tag = body.and_then(|b| b.0.tag);
    match hub.recorder.start(tag.as_deref()) {
        Ok(info) => {
            hub.event("info", format!("recording {}", info.name));
            ok(json!({ "ok": true, "recording": info }))
        }
        Err(e) => Err(err(StatusCode::CONFLICT, e)),
    }
}

async fn rec_stop(State(hub): St) -> ApiResult {
    let info = hub.recorder.stop();
    if let Some(i) = &info {
        hub.event("info", format!("recording {} stopped: {} frames, {:.0} s", i.name, i.frames, i.seconds));
    }
    ok(json!({ "ok": true, "recording": info }))
}

async fn rec_download(State(hub): St, Path(name): Path<String>) -> Response {
    let Some(path) = hub.recorder.path_of(&name) else {
        return err(StatusCode::NOT_FOUND, "no such recording").into_response();
    };
    let file = match tokio::fs::File::open(&path).await {
        Ok(f) => f,
        Err(e) => return err(StatusCode::INTERNAL_SERVER_ERROR, e.to_string()).into_response(),
    };
    let mut resp = Response::new(Body::from_stream(tokio_util::io::ReaderStream::new(file)));
    let h = resp.headers_mut();
    h.insert(header::CONTENT_TYPE, HeaderValue::from_static("application/octet-stream"));
    if let Ok(v) = HeaderValue::from_str(&format!("attachment; filename=\"{}\"", name)) {
        h.insert(header::CONTENT_DISPOSITION, v);
    }
    resp
}

async fn rec_delete(State(hub): St, Path(name): Path<String>) -> ApiResult {
    hub.recorder
        .delete(&name)
        .map(|_| Json(json!({ "ok": true })))
        .map_err(|e| err(StatusCode::NOT_FOUND, e))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn headers(origin: Option<&str>, host: &str) -> HeaderMap {
        let mut h = HeaderMap::new();
        h.insert(header::HOST, host.parse().unwrap());
        if let Some(o) = origin {
            h.insert(header::ORIGIN, o.parse().unwrap());
        }
        h
    }

    #[test]
    fn host_check() {
        let ok = |host: &str| trusted_host(&headers(None, host), &["robot.lan".into()]);
        assert!(ok("192.168.1.83:8080") && ok("[::1]:8080") && ok("localhost") && ok("vacuum.local:8080"));
        assert!(ok("Robot.lan:8080"), "allowed_hosts");
        assert!(!ok("attacker.example:8080"), "DNS rebinding");
    }

    #[test]
    fn origin_check() {
        assert!(same_origin(&headers(None, "robot:8080")), "curl");
        assert!(same_origin(&headers(Some("http://robot:8080"), "robot:8080")));
        assert!(!same_origin(&headers(Some("https://evil.example"), "robot:8080")));
        assert!(!same_origin(&headers(Some("null"), "robot:8080")));
    }
}
