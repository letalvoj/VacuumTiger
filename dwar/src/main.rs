//! dwar ("gate"): the web gateway of VacuumTiger.
//!
//! A client of SangamIO (hardware, TCP 5555) and DhruvaSLAM (SLAM, TCP 5557)
//! that serves a REST + WebSocket API (`/api/v1`) and the web UI embedded in
//! the binary. It never touches hardware directly, so a slow phone or a crash
//! here can't stall the HAL, and the UI keeps working with SLAM down.
//!
//! Usage: `dwar [config.toml]` (all settings have defaults; see dwar.toml).

mod api;
mod config;
mod control;
mod dhruva;
mod hub;
mod map;
mod proto;
mod recorder;
mod sangam;
mod web;
mod wire;
mod ws;

use axum::Router;
use axum::extract::ws::Message;
use std::sync::Arc;
use std::time::Duration;

#[tokio::main(flavor = "current_thread")]
async fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let path = std::env::args().nth(1);
    let cfg = match config::Config::load(path.as_deref()) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("dwar: config: {}", e);
            std::process::exit(2);
        }
    };
    let bind = cfg.http.bind.clone();
    let loopback = bind.parse::<std::net::SocketAddr>().is_ok_and(|a| a.ip().is_loopback());
    if cfg.http.token.is_none() && !loopback {
        log::warn!("no [http] token: anyone on this network can use the API");
    }
    let init = hub::Hub::build(cfg);
    let hub = init.hub;

    tokio::spawn(sangam::run(Arc::clone(&hub), init.sangam_rx));
    if hub.cfg.dhruva.enabled {
        tokio::spawn(dhruva::run(Arc::clone(&hub), init.dhruva_rx));
    }
    tokio::spawn(control::deadman_task(Arc::clone(&hub)));
    tokio::spawn(status_ticker(Arc::clone(&hub)));

    let app = Router::new()
        .nest("/api/v1", api::router(Arc::clone(&hub)))
        .fallback(web::asset);

    let listener = match tokio::net::TcpListener::bind(&bind).await {
        Ok(l) => l,
        Err(e) => {
            eprintln!("dwar: bind {}: {}", bind, e);
            std::process::exit(1);
        }
    };
    log::info!("dwar listening on http://{}", bind);
    if let Err(e) = axum::serve(listener, app).with_graceful_shutdown(shutdown_signal()).await {
        log::error!("server: {}", e);
    }
    // Leave the robot still and the recording readable.
    hub.recorder.stop();
    let _ = hub.stop_all("dwar shutting down");
    tokio::time::sleep(Duration::from_millis(200)).await; // let the writer send it
}

async fn shutdown_signal() {
    use tokio::signal::unix::{SignalKind, signal};
    let term = async {
        match signal(SignalKind::terminate()) {
            Ok(mut s) => {
                s.recv().await;
            }
            Err(_) => std::future::pending().await,
        }
    };
    tokio::select! {
        _ = tokio::signal::ctrl_c() => {}
        _ = term => {}
    }
}

/// Publishes the status topic once a second (it aggregates slow state).
async fn status_ticker(hub: Arc<hub::Hub>) {
    let mut tick = tokio::time::interval(Duration::from_secs(1));
    loop {
        tick.tick().await;
        if let Some(reason) = hub.recorder.take_stopped_reason() {
            hub.event("warn", reason);
        }
        let text = hub.status_json().to_string();
        hub.status.send_replace(Some(Message::Text(text.into())));
    }
}
