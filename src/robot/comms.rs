//! UDP multicast transport. Item 11 of `docs/BUILD_PLAN.md`, Phase 4.
//!
//! Every robot process joins the same multicast group (`config::MULTICAST_ADDR` /
//! `MULTICAST_PORT`) and binds it with `SO_REUSEPORT` so multiple robot processes can run
//! on one machine and each still receive every broadcast — this is what makes "one OS
//! process per robot, real UDP sockets" (`CLAUDE.md`, `docs/BUILD_PLAN.md` item 11) actual
//! rather than simulated. `planner_pibt.rs`'s conflict resolution moves from direct
//! function calls to this message exchange at item 13.

use crate::config::{COMMS_RANGE_CELLS, MULTICAST_ADDR, MULTICAST_PORT};
use crate::protocol::messages::{
    Ack, BlockCell, Heartbeat, ModeAnnounce, PoseIntent, TaskInject, TaskRetarget, TickMsg, TokenMsg,
};
use serde::{Deserialize, Serialize};
use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use std::io;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

/// Every message type in `protocol::messages`, wrapped for the wire. Only `PoseIntent`
/// carries a `position` field of its own; range-scoping (`Comms::recv_filtered`) needs
/// the sender's position for every message type, not just that one, so `Envelope` carries
/// it once at the transport layer instead of duplicating a `position` field onto
/// `TokenMsg`/`Heartbeat`/`ModeAnnounce`, which don't otherwise need one.
#[derive(Debug, Clone, Serialize, Deserialize)]
struct Envelope {
    sender_id: u32,
    sender_position: (i32, i32),
    payload: Payload,
}

/// The decoded contents of one `Envelope`, exposed to callers via `Received::payload`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Payload {
    Pose(PoseIntent),
    Token(TokenMsg),
    Ack(Ack),
    Heartbeat(Heartbeat),
    ModeAnnounce(ModeAnnounce),
    Tick(TickMsg),
    TaskInject(TaskInject),
    TaskRetarget(TaskRetarget),
    BlockCell(BlockCell),
}

/// One incoming message that passed range-scoping, returned by `Comms::recv_filtered`.
#[derive(Debug, Clone)]
pub struct Received {
    pub sender_id: u32,
    pub sender_position: (i32, i32),
    pub payload: Payload,
}

/// This robot's UDP multicast socket: send any message type to the shared group, and
/// receive others' broadcasts filtered by range.
pub struct Comms {
    socket: UdpSocket,
    robot_id: u32,
    position: (i32, i32),
    range_cells: u32,
    group: SocketAddr,
    /// The read timeout last set via `set_read_timeout`, in nanoseconds (`0` = none).
    /// `recv_filtered` needs it to bound the whole call, not just each `recv_from`.
    read_timeout_nanos: AtomicU64,
}

impl Comms {
    /// Binds the shared multicast group/port from `config.rs` with `SO_REUSEPORT`, using
    /// the default range-scoping radius (`COMMS_RANGE_CELLS`).
    pub fn new(robot_id: u32, position: (i32, i32)) -> io::Result<Self> {
        Self::with_range(robot_id, position, COMMS_RANGE_CELLS)
    }

    /// Same as `new`, but with an explicit range-scoping radius — item 12's test uses
    /// this to set an artificially small radius rather than relying on the real default.
    pub fn with_range(robot_id: u32, position: (i32, i32), range_cells: u32) -> io::Result<Self> {
        let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;
        socket.set_reuse_address(true)?;
        #[cfg(unix)]
        socket.set_reuse_port(true)?;

        let bind_addr: SocketAddr = SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, MULTICAST_PORT).into();
        socket.bind(&SockAddr::from(bind_addr))?;
        socket.join_multicast_v4(&MULTICAST_ADDR, &Ipv4Addr::UNSPECIFIED)?;
        // Multiple robot processes run on one machine in simulation mode (CLAUDE.md), so
        // loopback delivery must stay on for them to hear each other at all.
        socket.set_multicast_loop_v4(true)?;

        Ok(Self {
            socket: socket.into(),
            robot_id,
            position,
            range_cells,
            group: SocketAddrV4::new(MULTICAST_ADDR, MULTICAST_PORT).into(),
            read_timeout_nanos: AtomicU64::new(0),
        })
    }

    pub fn position(&self) -> (i32, i32) {
        self.position
    }

    /// Updates the position used both to tag this robot's own outgoing broadcasts and to
    /// range-filter incoming ones. Callers update this once per tick as the robot moves.
    pub fn set_position(&mut self, position: (i32, i32)) {
        self.position = position;
    }

    /// `None` blocks forever in `recv_filtered`; `Some(d)` bounds the wait. Tests set a
    /// short timeout so a scenario with nothing left to receive doesn't hang.
    pub fn set_read_timeout(&self, timeout: Option<Duration>) -> io::Result<()> {
        self.socket.set_read_timeout(timeout)?;
        let nanos = timeout.map_or(0, |d| d.as_nanos().min(u64::MAX as u128) as u64);
        self.read_timeout_nanos.store(nanos, Ordering::Relaxed);
        Ok(())
    }

    fn send(&self, payload: Payload) -> io::Result<()> {
        let envelope = Envelope {
            sender_id: self.robot_id,
            sender_position: self.position,
            payload,
        };
        let json = serde_json::to_vec(&envelope)
            .expect("Envelope wraps only derive(Serialize) message types, never fails");
        self.socket.send_to(&json, self.group)?;
        Ok(())
    }

    pub fn send_pose_intent(&self, msg: PoseIntent) -> io::Result<()> {
        self.send(Payload::Pose(msg))
    }

    pub fn send_token(&self, msg: TokenMsg) -> io::Result<()> {
        self.send(Payload::Token(msg))
    }

    pub fn send_ack(&self, msg: Ack) -> io::Result<()> {
        self.send(Payload::Ack(msg))
    }

    pub fn send_heartbeat(&self, msg: Heartbeat) -> io::Result<()> {
        self.send(Payload::Heartbeat(msg))
    }

    pub fn send_mode_announce(&self, msg: ModeAnnounce) -> io::Result<()> {
        self.send(Payload::ModeAnnounce(msg))
    }

    pub fn send_tick(&self, msg: TickMsg) -> io::Result<()> {
        self.send(Payload::Tick(msg))
    }

    pub fn send_task_inject(&self, msg: TaskInject) -> io::Result<()> {
        self.send(Payload::TaskInject(msg))
    }

    pub fn send_task_retarget(&self, msg: TaskRetarget) -> io::Result<()> {
        self.send(Payload::TaskRetarget(msg))
    }

    pub fn send_block_cell(&self, msg: BlockCell) -> io::Result<()> {
        self.send(Payload::BlockCell(msg))
    }

    /// Returns the next deliverable message: decodes incoming datagrams and applies
    /// range-scoping (`docs/PS_AND_ARCHITECTURE.md` §3.2), retrying past any that don't
    /// qualify — undecodable data, this robot's own broadcast looped back to itself (see
    /// below), or a sender farther than `range_cells` away (Euclidean, not grid-path
    /// length: range-scoping models a physical radio's circular reach, not how far apart
    /// two cells are by the map's corridors) — rather than returning `None` for the first
    /// datagram that fails one of those checks. `Ok(None)` therefore only ever means the
    /// read genuinely timed out (`set_read_timeout`) with nothing left to check.
    ///
    /// Self-loopback happens because `IP_MULTICAST_LOOP` is a send-side, not sender-only,
    /// setting: turning it off would stop *every* local process from receiving that
    /// send, not just the sender, which would break same-machine delivery between robot
    /// processes entirely. Leaving it on is correct, but it means this robot's own sends
    /// land in its own receive queue too, and a caller doing one `recv_from` per expected
    /// peer message would silently desync behind its own stale loopback traffic — this
    /// loop is what keeps that invisible to callers.
    ///
    /// The read timeout bounds this call as a whole, not just each `recv_from`: a robot
    /// out of range of every peer drops each packet here, and peers sending faster than
    /// the timeout would otherwise keep it looping indefinitely (seen live: one call
    /// blocked ~30s in the Docker e2e run). Returns `WouldBlock`, like a plain socket
    /// timeout, once the budget is spent — so a call can overshoot by at most one
    /// per-`recv_from` timeout.
    pub fn recv_filtered(&self) -> io::Result<Option<Received>> {
        let timeout_nanos = self.read_timeout_nanos.load(Ordering::Relaxed);
        let deadline = (timeout_nanos > 0).then(|| Instant::now() + Duration::from_nanos(timeout_nanos));
        loop {
            let mut buf = [0u8; 2048];
            let (n, _from) = self.socket.recv_from(&mut buf)?;
            let out_of_budget = || deadline.is_some_and(|d| Instant::now() >= d);
            let Ok(envelope) = serde_json::from_slice::<Envelope>(&buf[..n]) else {
                if out_of_budget() {
                    return Err(io::ErrorKind::WouldBlock.into());
                }
                continue;
            };
            let dx = (envelope.sender_position.0 - self.position.0) as i64;
            let dy = (envelope.sender_position.1 - self.position.1) as i64;
            let range = self.range_cells as i64;
            if envelope.sender_id == self.robot_id || dx * dx + dy * dy > range * range {
                if out_of_budget() {
                    return Err(io::ErrorKind::WouldBlock.into());
                }
                continue;
            }
            return Ok(Some(Received {
                sender_id: envelope.sender_id,
                sender_position: envelope.sender_position,
                payload: envelope.payload,
            }));
        }
    }
}
