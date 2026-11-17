#![no_std]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

// must go first!
mod fmt;

use core::cell::RefCell;
use core::mem::MaybeUninit;
use core::task::{Context, Poll};

pub use embassy_net_driver as driver;
use embassy_net_driver::{Capabilities, LinkState, LocalLoss, LossTag};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::waitqueue::WakerRegistration;
use embassy_sync::zerocopy_channel;

/// Channel state.
///
/// Holds a buffer of packets with size MTU, for both TX and RX.
pub struct State<const MTU: usize, const N_RX: usize, const N_TX: usize> {
    rx: [PacketBuf<MTU>; N_RX],
    tx: [PacketBuf<MTU>; N_TX],
    inner: MaybeUninit<StateInner<'static, MTU>>,
}

impl<const MTU: usize, const N_RX: usize, const N_TX: usize> State<MTU, N_RX, N_TX> {
    /// Create a new channel state.
    pub const fn new() -> Self {
        Self {
            rx: [const { PacketBuf::new() }; N_RX],
            tx: [const { PacketBuf::new() }; N_TX],
            inner: MaybeUninit::uninit(),
        }
    }
}

struct StateInner<'d, const MTU: usize> {
    rx: zerocopy_channel::Channel<'d, NoopRawMutex, PacketBuf<MTU>>,
    tx: zerocopy_channel::Channel<'d, NoopRawMutex, PacketBuf<MTU>>,
    shared: Mutex<NoopRawMutex, RefCell<Shared>>,
}

struct Shared {
    link_state: LinkState,
    waker: WakerRegistration,
    hardware_address: driver::HardwareAddress,
    losses: Losses,
    loss_waker: WakerRegistration,
}

/// How many local-loss reports wait for the stack; when full, the oldest goes.
const LOSSES: usize = 4;

/// The local-loss reports waiting for the stack, oldest first.
struct Losses {
    reports: [Option<LocalLoss>; LOSSES],
    next: usize,
    len: usize,
}

impl Losses {
    const fn new() -> Self {
        Self {
            reports: [None; LOSSES],
            next: 0,
            len: 0,
        }
    }

    fn push(&mut self, loss: LocalLoss) {
        if self.len == LOSSES {
            self.next = (self.next + 1) % LOSSES;
            self.len -= 1;
        }
        self.reports[(self.next + self.len) % LOSSES] = Some(loss);
        self.len += 1;
    }

    fn pop(&mut self) -> Option<LocalLoss> {
        if self.len == 0 {
            return None;
        }
        let loss = self.reports[self.next].take();
        self.next = (self.next + 1) % LOSSES;
        self.len -= 1;
        loss
    }
}

impl Shared {
    fn report_local_loss(&mut self, loss: LocalLoss) {
        self.losses.push(loss);
        self.loss_waker.wake();
    }
}

/// Channel runner.
///
/// Holds the shared state and the lower end of channels for inbound and outbound packets.
pub struct Runner<'d, const MTU: usize> {
    tx_chan: zerocopy_channel::Receiver<'d, NoopRawMutex, PacketBuf<MTU>>,
    rx_chan: zerocopy_channel::Sender<'d, NoopRawMutex, PacketBuf<MTU>>,
    shared: &'d Mutex<NoopRawMutex, RefCell<Shared>>,
}

/// State runner.
///
/// Holds the shared state of the channel such as link state.
#[derive(Clone, Copy)]
pub struct StateRunner<'d> {
    shared: &'d Mutex<NoopRawMutex, RefCell<Shared>>,
}

/// RX runner.
///
/// Holds the lower end of the channel for passing inbound packets up the stack.
pub struct RxRunner<'d, const MTU: usize> {
    rx_chan: zerocopy_channel::Sender<'d, NoopRawMutex, PacketBuf<MTU>>,
}

/// TX runner.
///
/// Holds the lower end of the channel for passing outbound packets down the stack.
pub struct TxRunner<'d, const MTU: usize> {
    tx_chan: zerocopy_channel::Receiver<'d, NoopRawMutex, PacketBuf<MTU>>,
}

impl<'d, const MTU: usize> Runner<'d, MTU> {
    /// Split the runner into separate runners for controlling state, rx and tx.
    pub fn split(self) -> (StateRunner<'d>, RxRunner<'d, MTU>, TxRunner<'d, MTU>) {
        (
            StateRunner { shared: self.shared },
            RxRunner { rx_chan: self.rx_chan },
            TxRunner { tx_chan: self.tx_chan },
        )
    }

    /// Split the runner into separate runners for controlling state, rx and tx borrowing the underlying state.
    pub fn borrow_split(&mut self) -> (StateRunner<'_>, RxRunner<'_, MTU>, TxRunner<'_, MTU>) {
        (
            StateRunner { shared: self.shared },
            RxRunner {
                rx_chan: self.rx_chan.borrow(),
            },
            TxRunner {
                tx_chan: self.tx_chan.borrow(),
            },
        )
    }

    /// Create a state runner sharing the state channel.
    pub fn state_runner(&self) -> StateRunner<'d> {
        StateRunner { shared: self.shared }
    }

    /// Set the link state.
    pub fn set_link_state(&mut self, state: LinkState) {
        self.shared.lock(|s| {
            let s = &mut *s.borrow_mut();
            s.link_state = state;
            s.waker.wake();
        });
    }

    /// Set the hardware address.
    pub fn set_hardware_address(&mut self, address: driver::HardwareAddress) {
        self.shared.lock(|s| {
            let s = &mut *s.borrow_mut();
            s.hardware_address = address;
            s.waker.wake();
        });
    }

    /// Wait until there is space for more inbound packets and return a slice they can be copied into.
    pub async fn rx_buf(&mut self) -> &mut [u8] {
        let p = self.rx_chan.send().await;
        &mut p.buf
    }

    /// Check if there is space for more inbound packets right now.
    pub fn try_rx_buf(&mut self) -> Option<&mut [u8]> {
        let p = self.rx_chan.try_send()?;
        Some(&mut p.buf)
    }

    /// Polling the inbound channel if there is space for packets.
    pub fn poll_rx_buf(&mut self, cx: &mut Context) -> Poll<&mut [u8]> {
        match self.rx_chan.poll_send(cx) {
            Poll::Ready(p) => Poll::Ready(&mut p.buf),
            Poll::Pending => Poll::Pending,
        }
    }

    /// Mark packet of len bytes as pushed to the inbound channel.
    pub fn rx_done(&mut self, len: usize) {
        let p = self.rx_chan.try_send().unwrap();
        p.len = len;
        self.rx_chan.send_done();
    }

    /// Wait until there is space for more outbound packets and return a slice they can be copied into.
    pub async fn tx_buf(&mut self) -> &mut [u8] {
        let p = self.tx_chan.receive().await;
        &mut p.buf[..p.len]
    }

    /// Check if there is space for more outbound packets right now.
    pub fn try_tx_buf(&mut self) -> Option<&mut [u8]> {
        let p = self.tx_chan.try_receive()?;
        Some(&mut p.buf[..p.len])
    }

    /// Polling the outbound channel if there is space for packets.
    pub fn poll_tx_buf(&mut self, cx: &mut Context) -> Poll<&mut [u8]> {
        match self.tx_chan.poll_receive(cx) {
            Poll::Ready(p) => Poll::Ready(&mut p.buf[..p.len]),
            Poll::Pending => Poll::Pending,
        }
    }

    /// Wait for an outbound packet, and return it with its local-loss tag, if the stack gave
    /// it one.
    pub async fn tx_buf_with_tag(&mut self) -> (&mut [u8], Option<LossTag>) {
        let p = self.tx_chan.receive().await;
        (&mut p.buf[..p.len], p.tag)
    }

    /// Mark outbound packet as copied.
    pub fn tx_done(&mut self) {
        self.tx_chan.receive_done();
    }

    /// Report a tagged packet lost before it left the link layer.
    pub fn report_local_loss(&mut self, loss: LocalLoss) {
        self.shared.lock(|s| s.borrow_mut().report_local_loss(loss));
    }
}

impl<'d> StateRunner<'d> {
    /// Set link state.
    pub fn set_link_state(&self, state: LinkState) {
        self.shared.lock(|s| {
            let s = &mut *s.borrow_mut();
            s.link_state = state;
            s.waker.wake();
        });
    }

    /// Set the hardware address.
    pub fn set_hardware_address(&self, address: driver::HardwareAddress) {
        self.shared.lock(|s| {
            let s = &mut *s.borrow_mut();
            s.hardware_address = address;
            s.waker.wake();
        });
    }

    /// Report a tagged packet lost before it left the link layer.
    pub fn report_local_loss(&self, loss: LocalLoss) {
        self.shared.lock(|s| s.borrow_mut().report_local_loss(loss));
    }
}

impl<'d, const MTU: usize> RxRunner<'d, MTU> {
    /// Wait until there is space for more inbound packets and return a slice they can be copied into.
    pub async fn rx_buf(&mut self) -> &mut [u8] {
        let p = self.rx_chan.send().await;
        &mut p.buf
    }

    /// Check if there is space for more inbound packets right now.
    pub fn try_rx_buf(&mut self) -> Option<&mut [u8]> {
        let p = self.rx_chan.try_send()?;
        Some(&mut p.buf)
    }

    /// Polling the inbound channel if there is space for packets.
    pub fn poll_rx_buf(&mut self, cx: &mut Context) -> Poll<&mut [u8]> {
        match self.rx_chan.poll_send(cx) {
            Poll::Ready(p) => Poll::Ready(&mut p.buf),
            Poll::Pending => Poll::Pending,
        }
    }

    /// Mark packet of len bytes as pushed to the inbound channel.
    pub fn rx_done(&mut self, len: usize) {
        let p = self.rx_chan.try_send().unwrap();
        p.len = len;
        self.rx_chan.send_done();
    }
}

impl<'d, const MTU: usize> TxRunner<'d, MTU> {
    /// Wait until there is space for more outbound packets and return a slice they can be copied into.
    pub async fn tx_buf(&mut self) -> &mut [u8] {
        let p = self.tx_chan.receive().await;
        &mut p.buf[..p.len]
    }

    /// Check if there is space for more outbound packets right now.
    pub fn try_tx_buf(&mut self) -> Option<&mut [u8]> {
        let p = self.tx_chan.try_receive()?;
        Some(&mut p.buf[..p.len])
    }

    /// Polling the outbound channel if there is space for packets.
    pub fn poll_tx_buf(&mut self, cx: &mut Context) -> Poll<&mut [u8]> {
        match self.tx_chan.poll_receive(cx) {
            Poll::Ready(p) => Poll::Ready(&mut p.buf[..p.len]),
            Poll::Pending => Poll::Pending,
        }
    }

    /// Wait for an outbound packet, and return it with its local-loss tag, if the stack gave
    /// it one.
    pub async fn tx_buf_with_tag(&mut self) -> (&mut [u8], Option<LossTag>) {
        let p = self.tx_chan.receive().await;
        (&mut p.buf[..p.len], p.tag)
    }

    /// Mark outbound packet as copied.
    pub fn tx_done(&mut self) {
        self.tx_chan.receive_done();
    }
}

/// Create a channel.
///
/// Returns a pair of handles for interfacing with the peripheral and the networking stack.
///
/// The runner is interfacing with the peripheral at the lower part of the stack.
/// The device is interfacing with the networking stack on the layer above.
pub fn new<'d, const MTU: usize, const N_RX: usize, const N_TX: usize>(
    state: &'d mut State<MTU, N_RX, N_TX>,
    hardware_address: driver::HardwareAddress,
) -> (Runner<'d, MTU>, Device<'d, MTU>) {
    let mut caps = Capabilities::default();
    caps.max_transmission_unit = MTU;

    // safety: this is a self-referential struct, however:
    // - it can't move while the `'d` borrow is active.
    // - when the borrow ends, the dangling references inside the MaybeUninit will never be used again.
    let state_uninit: *mut MaybeUninit<StateInner<'d, MTU>> =
        (&mut state.inner as *mut MaybeUninit<StateInner<'static, MTU>>).cast();
    let state = unsafe { &mut *state_uninit }.write(StateInner {
        rx: zerocopy_channel::Channel::new(&mut state.rx[..]),
        tx: zerocopy_channel::Channel::new(&mut state.tx[..]),
        shared: Mutex::new(RefCell::new(Shared {
            link_state: LinkState::Down,
            hardware_address,
            waker: WakerRegistration::new(),
            losses: Losses::new(),
            loss_waker: WakerRegistration::new(),
        })),
    });

    let (rx_sender, rx_receiver) = state.rx.split();
    let (tx_sender, tx_receiver) = state.tx.split();

    (
        Runner {
            tx_chan: tx_receiver,
            rx_chan: rx_sender,
            shared: &state.shared,
        },
        Device {
            caps,
            shared: &state.shared,
            rx: rx_receiver,
            tx: tx_sender,
        },
    )
}

/// Represents a packet of size MTU.
pub struct PacketBuf<const MTU: usize> {
    len: usize,
    tag: Option<LossTag>,
    buf: [u8; MTU],
}

impl<const MTU: usize> PacketBuf<MTU> {
    /// Create a new packet buffer.
    pub const fn new() -> Self {
        Self {
            len: 0,
            tag: None,
            buf: [0; MTU],
        }
    }
}

/// Channel device.
///
/// Holds the shared state and upper end of channels for inbound and outbound packets.
pub struct Device<'d, const MTU: usize> {
    rx: zerocopy_channel::Receiver<'d, NoopRawMutex, PacketBuf<MTU>>,
    tx: zerocopy_channel::Sender<'d, NoopRawMutex, PacketBuf<MTU>>,
    shared: &'d Mutex<NoopRawMutex, RefCell<Shared>>,
    caps: Capabilities,
}

impl<'d, const MTU: usize> embassy_net_driver::Driver for Device<'d, MTU> {
    type RxToken<'a>
        = RxToken<'a, MTU>
    where
        Self: 'a;
    type TxToken<'a>
        = TxToken<'a, MTU>
    where
        Self: 'a;

    fn receive(&mut self, cx: &mut Context) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        if self.rx.poll_receive(cx).is_ready() && self.tx.poll_send(cx).is_ready() {
            Some((
                RxToken { rx: self.rx.borrow() },
                TxToken {
                    tx: self.tx.borrow(),
                    tag: None,
                },
            ))
        } else {
            None
        }
    }

    /// Construct a transmit token.
    fn transmit(&mut self, cx: &mut Context) -> Option<Self::TxToken<'_>> {
        if self.tx.poll_send(cx).is_ready() {
            Some(TxToken {
                tx: self.tx.borrow(),
                tag: None,
            })
        } else {
            None
        }
    }

    /// Get a description of device capabilities.
    fn capabilities(&self) -> Capabilities {
        self.caps.clone()
    }

    fn hardware_address(&self) -> driver::HardwareAddress {
        self.shared.lock(|s| s.borrow().hardware_address)
    }

    fn link_state(&mut self, cx: &mut Context) -> LinkState {
        self.shared.lock(|s| {
            let s = &mut *s.borrow_mut();
            s.waker.register(cx.waker());
            s.link_state
        })
    }

    fn poll_local_loss(&mut self, cx: &mut Context) -> Option<LocalLoss> {
        self.shared.lock(|s| {
            let s = &mut *s.borrow_mut();
            let loss = s.losses.pop();
            if loss.is_none() {
                s.loss_waker.register(cx.waker());
            }
            loss
        })
    }
}

/// A rx token.
///
/// Holds inbound receive channel and interfaces with embassy-net-driver.
pub struct RxToken<'a, const MTU: usize> {
    rx: zerocopy_channel::Receiver<'a, NoopRawMutex, PacketBuf<MTU>>,
}

impl<'a, const MTU: usize> embassy_net_driver::RxToken for RxToken<'a, MTU> {
    fn consume<R, F>(mut self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        // NOTE(unwrap): we checked the queue wasn't full when creating the token.
        let pkt = unwrap!(self.rx.try_receive());
        let r = f(&mut pkt.buf[..pkt.len]);
        self.rx.receive_done();
        r
    }
}

/// A tx token.
///
/// Holds outbound transmit channel and interfaces with embassy-net-driver.
pub struct TxToken<'a, const MTU: usize> {
    tx: zerocopy_channel::Sender<'a, NoopRawMutex, PacketBuf<MTU>>,
    tag: Option<LossTag>,
}

impl<'a, const MTU: usize> embassy_net_driver::TxToken for TxToken<'a, MTU> {
    fn consume<R, F>(mut self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        // NOTE(unwrap): we checked the queue wasn't full when creating the token.
        let pkt = unwrap!(self.tx.try_send());
        let r = f(&mut pkt.buf[..len]);
        pkt.len = len;
        pkt.tag = self.tag;
        self.tx.send_done();
        r
    }

    fn set_tag(&mut self, tag: LossTag) {
        self.tag = Some(tag);
    }
}

#[cfg(test)]
mod tests {
    use core::task::Waker;

    use embassy_net_driver::{Driver, LossReason, TxToken as _};

    use super::*;

    fn loss(id: u32) -> LocalLoss {
        LocalLoss {
            tag: LossTag { transport: 6, id },
            reason: LossReason::NoAck,
        }
    }

    #[test]
    fn a_tag_goes_down_with_its_packet_and_a_report_comes_back() {
        let mut state = State::<16, 1, 2>::new();
        let (mut runner, mut device) = new(&mut state, driver::HardwareAddress::Ip);
        let mut cx = Context::from_waker(Waker::noop());
        let mut tx = device.transmit(&mut cx).unwrap();
        tx.set_tag(LossTag { transport: 6, id: 7 });
        tx.consume(3, |b| b.copy_from_slice(&[1, 2, 3]));
        device.transmit(&mut cx).unwrap().consume(1, |b| b[0] = 9);

        let p = runner.tx_chan.try_receive().unwrap();
        assert_eq!(
            (&p.buf[..p.len], p.tag),
            (&[1, 2, 3][..], Some(LossTag { transport: 6, id: 7 }))
        );
        runner.tx_done();
        let p = runner.tx_chan.try_receive().unwrap();
        assert_eq!(p.tag, None);
        runner.tx_done();

        assert_eq!(device.poll_local_loss(&mut cx), None);
        runner.report_local_loss(loss(7));
        assert_eq!(device.poll_local_loss(&mut cx), Some(loss(7)));
        assert_eq!(device.poll_local_loss(&mut cx), None);
    }

    #[test]
    fn a_full_report_queue_drops_its_oldest() {
        let mut losses = Losses::new();
        for id in 0..6 {
            losses.push(loss(id));
        }
        for id in 2..6 {
            assert_eq!(losses.pop(), Some(loss(id)));
        }
        assert_eq!(losses.pop(), None);
    }
}
