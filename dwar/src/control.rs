//! Motion control: who may drive, the dead-man, and STOP. Owned by the hub so
//! every transport (the WebSocket today, Valetudo's manual-control API later)
//! shares one set of rules, and the dead-man runs on its own timer, not inside
//! a client session that a vanished phone could stall.
//!
//! - One teleop holder at a time (a web session); taking it takes SangamIO's
//!   lease (SangamIO zeroes the drive on the handover).
//! - Drive commands are clamped to `[control]` limits. No command for
//!   `deadman_ms` -> zero velocity. SangamIO's heartbeat dead-man
//!   (`deadman_timeout_ms`) stays as the layer below.
//! - STOP (anyone) zeroes everything and revokes teleop: the holder must take
//!   it again, so a joystick still held can't undo a STOP.

use crate::hub::{Action, Hub};
use crate::proto::sangamio::sensor_value::Value as V;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Cleaning actuators with a 0-100 % speed.
pub const CLEANING: &[&str] = &["vacuum", "main_brush", "side_brush", "water_pump"];

/// Components switched off by STOP (SangamIO accepts their `Disable` from any client).
pub fn stop_components() -> impl Iterator<Item = &'static str> {
    std::iter::once("drive").chain(CLEANING.iter().copied())
}

pub type SessionId = u64;

#[derive(Default)]
pub struct Teleop {
    holder: Option<SessionId>,
    last_drive: Option<Instant>,
    /// Non-zero velocity was commanded and not yet zeroed.
    moving: bool,
    /// `drive Enable` sent for the current holder.
    enabled: bool,
}

impl Hub {
    fn teleop(&self) -> std::sync::MutexGuard<'_, Teleop> {
        self.teleop.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn teleop_holder(&self) -> Option<SessionId> {
        self.teleop().holder
    }

    pub fn take_teleop(&self, session: SessionId) -> Result<(), String> {
        if !self.cfg.control.allow_drive {
            return Err("teleop disabled (dwar [control] allow_drive = false)".into());
        }
        if !self.shared().sangam_connected {
            return Err("SangamIO offline".into());
        }
        let mut t = self.teleop();
        if t.holder == Some(session) {
            return Ok(());
        }
        let prev = t.holder.replace(session);
        t.moving = false;
        t.enabled = false;
        t.last_drive = None;
        drop(t);
        // Take SangamIO's lease from whoever drives now (SLAM controller, another
        // tool); SangamIO zeroes the drive on the handover.
        self.sangam("lease", Action::Enable, &[("force", V::BoolVal(true))]);
        self.event("warn", format!("teleop taken by web session #{} (was {:?})", session, prev));
        Ok(())
    }

    pub fn release_teleop(&self, session: SessionId, why: &str) {
        let mut t = self.teleop();
        if t.holder != Some(session) {
            return;
        }
        *t = Teleop::default();
        drop(t);
        self.sangam("drive", Action::Disable, &[]);
        self.sangam("lease", Action::Disable, &[]);
        self.event("info", format!("teleop released by web session #{} ({})", session, why));
    }

    pub fn drive(&self, session: SessionId, linear: f32, angular: f32) -> Result<(), String> {
        let c = &self.cfg.control;
        let clamp = |v: f32, max: f32| if v.is_finite() { v.clamp(-max, max) } else { 0.0 };
        let (linear, angular) = (clamp(linear, c.max_linear), clamp(angular, c.max_angular));
        let mut t = self.teleop();
        if t.holder != Some(session) {
            return Err("take teleop first".into());
        }
        t.last_drive = Some(Instant::now());
        let still = linear == 0.0 && angular == 0.0;
        if still && !t.moving {
            return Ok(());
        }
        // State changes only for commands that were queued; a dropped one is
        // retried by the next drive message (or the dead-man).
        if !t.enabled {
            if !self.sangam("drive", Action::Enable, &[]) {
                return Err("SangamIO command queue full".into());
            }
            t.enabled = true;
        }
        if !self.sangam(
            "drive",
            Action::Configure,
            &[("linear", V::F32Val(linear)), ("angular", V::F32Val(angular))],
        ) {
            return Err("SangamIO command queue full".into());
        }
        t.moving = !still;
        Ok(())
    }

    /// Emergency stop: zero the drive, cleaning actuators off, revoke teleop.
    /// SangamIO accepts these from any client, lease or not.
    pub fn stop_all(&self, why: &str) -> Result<(), String> {
        let holder = std::mem::take(&mut *self.teleop()).holder;
        let mut queued = true;
        for id in stop_components() {
            queued &= self.sangam(id, Action::Disable, &[]);
        }
        {
            let mut s = self.shared();
            for id in stop_components() {
                s.actuators.remove(id);
            }
        }
        self.event("warn", format!("STOP ({}){}", why, holder.map_or(String::new(), |h| format!(", teleop of session #{} revoked", h))));
        if !self.shared().sangam_connected {
            return Err("SangamIO offline: STOP not delivered".into());
        }
        if queued { Ok(()) } else { Err("command queue full: STOP may not have been delivered".into()) }
    }

    /// SangamIO went away: it zeroed the drive and freed the lease itself, so
    /// forget teleop and the commanded actuator state.
    pub fn sangam_lost(&self) {
        let holder = std::mem::take(&mut *self.teleop()).holder;
        self.shared().actuators.clear();
        if let Some(h) = holder {
            self.event("warn", format!("teleop of session #{} ended: SangamIO disconnected", h));
        }
    }

    /// Zero the velocity when the holder went quiet.
    fn deadman_check(&self) {
        let timeout = Duration::from_millis(self.cfg.control.deadman_ms);
        let mut t = self.teleop();
        if t.moving && t.last_drive.is_none_or(|l| l.elapsed() > timeout) {
            let zero = [("linear", V::F32Val(0.0)), ("angular", V::F32Val(0.0))];
            // If the zero can't be queued, stay "moving": the next tick retries.
            if self.sangam("drive", Action::Configure, &zero) {
                t.moving = false;
                let holder = t.holder;
                drop(t);
                self.event("warn", format!("teleop dead-man: session {:?} went quiet", holder));
            }
        }
    }
}

/// The dead-man timer, independent of any client session.
pub async fn deadman_task(hub: Arc<Hub>) {
    let mut tick = tokio::time::interval(Duration::from_millis(50));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tick.tick().await;
        hub.deadman_check();
    }
}
