//! Socket options for client connections.

use socket2::{SockRef, TcpKeepalive};
use std::net::TcpStream;
use std::time::Duration;

/// Idle time before the first keepalive probe, probe interval, probe count:
/// a silent peer is declared dead after 5 + 4 × 2 = 13 s.
const KEEPALIVE_IDLE: Duration = Duration::from_secs(5);
const KEEPALIVE_INTERVAL: Duration = Duration::from_secs(2);
const KEEPALIVE_PROBES: u32 = 4;
/// Unacknowledged data (TCP telemetry to a vanished peer) times out after this.
#[cfg(target_os = "linux")]
const USER_TIMEOUT: Duration = Duration::from_secs(15);

/// Detect a client that vanished without closing (Wi-Fi drop, power loss) in
/// ~15 s, so it leaves the hub and a held lease is freed with the drive zeroed.
pub fn enable_dead_peer_detection(stream: &TcpStream) {
    let sock = SockRef::from(stream);
    let keepalive = TcpKeepalive::new()
        .with_time(KEEPALIVE_IDLE)
        .with_interval(KEEPALIVE_INTERVAL)
        .with_retries(KEEPALIVE_PROBES);
    if let Err(e) = sock.set_tcp_keepalive(&keepalive) {
        log::warn!("TCP keepalive: {}", e);
    }
    #[cfg(target_os = "linux")]
    if let Err(e) = sock.set_tcp_user_timeout(Some(USER_TIMEOUT)) {
        log::warn!("TCP_USER_TIMEOUT: {}", e);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    #[test]
    fn options_are_set() {
        let l = TcpListener::bind("127.0.0.1:0").unwrap();
        let s = TcpStream::connect(l.local_addr().unwrap()).unwrap();
        enable_dead_peer_detection(&s);
        let sock = SockRef::from(&s);
        assert!(sock.keepalive().unwrap());
        assert_eq!(sock.keepalive_time().unwrap(), KEEPALIVE_IDLE);
        assert_eq!(sock.keepalive_retries().unwrap(), KEEPALIVE_PROBES);
    }
}
