//! Socket options for client connections.

use std::net::TcpStream;
use std::os::fd::AsRawFd;

/// Idle time before the first keepalive probe, probe interval, probe count:
/// a silent peer is declared dead after 5 + 4 × 2 = 13 s.
const KEEPALIVE_IDLE_S: libc::c_int = 5;
const KEEPALIVE_INTERVAL_S: libc::c_int = 2;
const KEEPALIVE_PROBES: libc::c_int = 4;
/// Unacknowledged data (TCP telemetry to a vanished peer) times out after this.
#[cfg(target_os = "linux")]
const USER_TIMEOUT_MS: libc::c_int = 15_000;

#[cfg(target_os = "linux")]
const TCP_KEEPIDLE: libc::c_int = libc::TCP_KEEPIDLE;
#[cfg(not(target_os = "linux"))]
const TCP_KEEPIDLE: libc::c_int = libc::TCP_KEEPALIVE;

/// Detect a client that vanished without closing (Wi-Fi drop, power loss) in
/// ~15 s, so it leaves the hub and a held lease is freed with the drive zeroed.
pub fn enable_dead_peer_detection(stream: &TcpStream) {
    let fd = stream.as_raw_fd();
    let set = |name: &str, level, opt, value: libc::c_int| {
        // SAFETY: valid fd; pointer and length describe one c_int.
        let rc = unsafe {
            libc::setsockopt(
                fd,
                level,
                opt,
                &value as *const libc::c_int as *const libc::c_void,
                std::mem::size_of::<libc::c_int>() as libc::socklen_t,
            )
        };
        if rc != 0 {
            log::warn!("setsockopt {}: {}", name, std::io::Error::last_os_error());
        }
    };
    set("SO_KEEPALIVE", libc::SOL_SOCKET, libc::SO_KEEPALIVE, 1);
    set(
        "TCP_KEEPIDLE",
        libc::IPPROTO_TCP,
        TCP_KEEPIDLE,
        KEEPALIVE_IDLE_S,
    );
    set(
        "TCP_KEEPINTVL",
        libc::IPPROTO_TCP,
        libc::TCP_KEEPINTVL,
        KEEPALIVE_INTERVAL_S,
    );
    set(
        "TCP_KEEPCNT",
        libc::IPPROTO_TCP,
        libc::TCP_KEEPCNT,
        KEEPALIVE_PROBES,
    );
    #[cfg(target_os = "linux")]
    set(
        "TCP_USER_TIMEOUT",
        libc::IPPROTO_TCP,
        libc::TCP_USER_TIMEOUT,
        USER_TIMEOUT_MS,
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    fn get(stream: &TcpStream, level: libc::c_int, opt: libc::c_int) -> libc::c_int {
        let mut v: libc::c_int = 0;
        let mut len = std::mem::size_of::<libc::c_int>() as libc::socklen_t;
        // SAFETY: valid fd; v/len describe one c_int.
        let rc = unsafe {
            libc::getsockopt(
                stream.as_raw_fd(),
                level,
                opt,
                &mut v as *mut _ as *mut libc::c_void,
                &mut len,
            )
        };
        assert_eq!(rc, 0);
        v
    }

    #[test]
    fn options_are_set() {
        let l = TcpListener::bind("127.0.0.1:0").unwrap();
        let s = TcpStream::connect(l.local_addr().unwrap()).unwrap();
        enable_dead_peer_detection(&s);
        assert_ne!(get(&s, libc::SOL_SOCKET, libc::SO_KEEPALIVE), 0);
        assert_eq!(get(&s, libc::IPPROTO_TCP, TCP_KEEPIDLE), KEEPALIVE_IDLE_S);
        assert_eq!(
            get(&s, libc::IPPROTO_TCP, libc::TCP_KEEPCNT),
            KEEPALIVE_PROBES
        );
    }
}
