#![macro_use]

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{fence, Ordering};
use core::task::{Context, Poll};

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use super::word::{Word, WordSize};
use super::{AnyChannel, Channel, Dir, Request, STATE};
use crate::interrupt::typelevel::Interrupt;
use crate::interrupt::Priority;
use crate::pac;
use crate::pac::gpdma::vals;

pub(crate) struct ChannelInfo {
    pub(crate) dma: pac::gpdma::Gpdma,
    pub(crate) num: usize,
    #[cfg(feature = "_dual-core")]
    pub(crate) irq: pac::Interrupt,
}

/// GPDMA transfer options.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TransferOptions {
    pub trigger_source: Option<TriggerSource>,
    pub trigger_mode: TriggerMode,
    pub trigger_polarity: TriggerPolarity,
}

impl Default for TransferOptions {
    fn default() -> Self {
        Self {
            trigger_source: None,
            trigger_mode: TriggerMode::Block,
            trigger_polarity: TriggerPolarity::None,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TriggerSource {
    ExtiLine0 = 0,
    ExtiLine1 = 1,
    ExtiLine2 = 2,
    ExtiLine3 = 3,
    ExtiLine4 = 4,
    ExtiLine5 = 5,
    ExtiLine6 = 6,
    ExtiLine7 = 7,
    TampTrg1 = 8,
    TampTrg2 = 9,
    TampTrg3 = 10,
    Lptim1Ch1 = 11,
    Lptim1Ch2 = 12,
    Lptim2Ch1 = 13,
    Lptim2Ch2 = 14,
    Lptim4Out = 15,
    Comp1Out = 16,
    Comp2Out = 17,
    RtcAlraTrg = 18,
    RtcAlrbTrg = 19,
    RtcWutTrg = 20,
    Gpdma1Ch0Tcf = 22,
    Gpdma1Ch1Tcf = 23,
    Gpdma1Ch2Tcf = 24,
    Gpdma1Ch3Tcf = 25,
    Gpdma1Ch4Tcf = 26,
    Gpdma1Ch5Tcf = 27,
    Gpdma1Ch6Tcf = 28,
    Gpdma1Ch7Tcf = 29,
    Gpdma1Ch8Tcf = 30,
    Gpdma1Ch9Tcf = 31,
    Gpdma1Ch10Tcf = 32,
    Gpdma1Ch11Tcf = 33,
    Gpdma1Ch12Tcf = 34,
    Gpdma1Ch13Tcf = 35,
    Gpdma1Ch14Tcf = 36,
    Gpdma1Ch15Tcf = 37,
    Lpdma1Ch0Tcf = 38,
    Lpdma1Ch1Tcf = 39,
    Lpdma1Ch2Tcf = 40,
    Lpdma1Ch3Tcf = 41,
    Tim2Trgo = 42,
    Tim15Trgo = 43,
    Tim3Trgo = 44,
    Tim4Trgo = 45,
    Tim5Trgo = 46,
    LtdcLi = 47,
    DsiTe = 48,
    DsiEr = 49,
    Adc4Awd1 = 57,
    Adc1Awd1 = 58,
    Dma2dTc = 50,
    Dma2dCtc = 51,
    Dma2dTw = 52,
    Gpu2dFlag0 = 53,
    Gpu2dFlag1 = 54,
    Gpu2dFlag2 = 55,
    Gpu2dFlag3 = 56,
    GfxtimEvt3 = 59,
    GfxtimEvt2 = 60,
    GfxtimEvt1 = 61,
    GfxtimEvt0 = 62,
    JpegEoc = 63,
    JpegIfnf = 64,
    JpegIft = 65,
    JpegOfne = 66,
    JpegOft = 67,
}

impl From<TriggerSource> for u8 {
    fn from(value: TriggerSource) -> Self {
        value as u8
    }
}

/// These bits define the polarity of the selected trigger source.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TriggerPolarity {
    /// No trigger (masked trigger event).
    None,
    /// Trigger on the rising edge.
    RisingEdge,
    /// Trigger on the falling edge.
    FallingEdge,
}

impl From<TriggerPolarity> for vals::Trigpol {
    fn from(value: TriggerPolarity) -> Self {
        match value {
            TriggerPolarity::None => vals::Trigpol::NONE,
            TriggerPolarity::RisingEdge => vals::Trigpol::RISINGEDGE,
            TriggerPolarity::FallingEdge => vals::Trigpol::FALLINGEDGE,
        }
    }
}

/// Trigger mode
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TriggerMode {
    /// Trigger at block level: the first burst read of each
    /// block transfer is conditioned by one hit trigger.
    Block,
    /// Same as block level trigger channel but at 2D/repeated block level.
    /// The first burst read of a 2D/repeated block transfer is conditioned by one hit trigger.
    Block2D,
    /// At link level: a LLI link transfer is conditioned by one hit trigger. The LLI data transfer
    /// (if any) is not conditioned.
    LinkedListItem,
    /// Trigger at programmed burst level: If SWREQ = 1, each programmed burst read is conditioned by
    /// one hit trigger. If SWREQ = 0, each programmed burst that is requested by the selected
    /// peripheral, is conditioned by one hit trigger.
    Burst,
}

impl From<TriggerMode> for vals::Trigm {
    fn from(value: TriggerMode) -> Self {
        match value {
            TriggerMode::Block => Self::BLOCK,
            TriggerMode::Burst => Self::BURST,
            TriggerMode::LinkedListItem => Self::LINKEDLISTITEM,
            TriggerMode::Block2D => Self::_2DBLOCK,
        }
    }
}

impl From<WordSize> for vals::Dw {
    fn from(raw: WordSize) -> Self {
        match raw {
            WordSize::OneByte => Self::BYTE,
            WordSize::TwoBytes => Self::HALFWORD,
            WordSize::FourBytes => Self::WORD,
        }
    }
}

pub(crate) struct ChannelState {
    waker: AtomicWaker,
}

impl ChannelState {
    pub(crate) const NEW: Self = Self {
        waker: AtomicWaker::new(),
    };
}

/// safety: must be called only once
pub(crate) unsafe fn init(cs: critical_section::CriticalSection, irq_priority: Priority) {
    foreach_interrupt! {
        ($peri:ident, gpdma, $block:ident, $signal_name:ident, $irq:ident) => {
            crate::interrupt::typelevel::$irq::set_priority_with_cs(cs, irq_priority);
            #[cfg(not(feature = "_dual-core"))]
            crate::interrupt::typelevel::$irq::enable();
        };
    }
    crate::_generated::init_gpdma();
}

impl AnyChannel {
    /// Safety: Must be called with a matching set of parameters for a valid dma channel
    pub(crate) unsafe fn on_irq(&self) {
        let info = self.info();
        #[cfg(feature = "_dual-core")]
        {
            use embassy_hal_internal::interrupt::InterruptExt as _;
            info.irq.enable();
        }

        let state = &STATE[self.id as usize];

        let ch = info.dma.ch(info.num);
        let sr = ch.sr().read();

        if sr.dtef() {
            panic!(
                "DMA: data transfer error on DMA@{:08x} channel {}",
                info.dma.as_ptr() as u32,
                info.num
            );
        }
        if sr.usef() {
            panic!(
                "DMA: user settings error on DMA@{:08x} channel {}",
                info.dma.as_ptr() as u32,
                info.num
            );
        }

        if sr.suspf() || sr.tcf() {
            // disable all xxIEs to prevent the irq from firing again.
            ch.cr().write(|_| {});

            // Wake the future. It'll look at tcf and see it's set.
            state.waker.wake();
        }
    }
}

/// DMA transfer.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Transfer<'a> {
    channel: PeripheralRef<'a, AnyChannel>,
}

impl<'a> Transfer<'a> {
    /// Create a new read DMA transfer (peripheral to memory).
    pub unsafe fn new_read<W: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        peri_addr: *mut W,
        buf: &'a mut [W],
        options: TransferOptions,
    ) -> Self {
        Self::new_read_raw(channel, request, peri_addr, buf, options)
    }

    /// Create a new read DMA transfer (peripheral to memory), using raw pointers.
    pub unsafe fn new_read_raw<W: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        peri_addr: *mut W,
        buf: *mut [W],
        options: TransferOptions,
    ) -> Self {
        into_ref!(channel);

        Self::new_inner(
            channel.map_into(),
            request,
            Dir::PeripheralToMemory,
            peri_addr as *const u32,
            buf as *mut W as *mut u32,
            buf.len(),
            true,
            W::size(),
            options,
        )
    }

    /// Create a new write DMA transfer (memory to peripheral).
    pub unsafe fn new_write<W: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        buf: &'a [W],
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Self {
        Self::new_write_raw(channel, request, buf, peri_addr, options)
    }

    /// Create a new write DMA transfer (memory to peripheral), using raw pointers.
    pub unsafe fn new_write_raw<W: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        buf: *const [W],
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Self {
        into_ref!(channel);

        Self::new_inner(
            channel.map_into(),
            request,
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            buf as *const W as *mut u32,
            buf.len(),
            true,
            W::size(),
            options,
        )
    }

    /// Create a new write DMA transfer (memory to peripheral), writing the same value repeatedly.
    pub unsafe fn new_write_repeated<W: Word>(
        channel: impl Peripheral<P = impl Channel> + 'a,
        request: Request,
        repeated: &'a W,
        count: usize,
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Self {
        into_ref!(channel);

        Self::new_inner(
            channel.map_into(),
            request,
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            repeated as *const W as *mut u32,
            count,
            false,
            W::size(),
            options,
        )
    }

    unsafe fn new_inner(
        channel: PeripheralRef<'a, AnyChannel>,
        request: Request,
        dir: Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: bool,
        data_size: WordSize,
        options: TransferOptions,
    ) -> Self {
        // BNDT is specified as bytes, not as number of transfers.
        let Ok(bndt) = (mem_len * data_size.bytes()).try_into() else {
            panic!("DMA transfers may not be larger than 65535 bytes.");
        };

        let info = channel.info();
        let ch = info.dma.ch(info.num);

        // "Preceding reads and writes cannot be moved past subsequent writes."
        fence(Ordering::SeqCst);

        let this = Self { channel };

        ch.cr().write(|w| w.set_reset(true));
        ch.fcr().write(|w| w.0 = 0xFFFF_FFFF); // clear all irqs
        ch.llr().write(|_| {}); // no linked list
        ch.tr1().write(|w| {
            w.set_sdw(data_size.into());
            w.set_ddw(data_size.into());
            w.set_sinc(dir == Dir::MemoryToPeripheral && incr_mem);
            w.set_dinc(dir == Dir::PeripheralToMemory && incr_mem);
        });
        ch.tr2().write(|w| {
            w.set_dreq(match dir {
                Dir::MemoryToPeripheral => vals::Dreq::DESTINATIONPERIPHERAL,
                Dir::PeripheralToMemory => vals::Dreq::SOURCEPERIPHERAL,
            });
            w.set_reqsel(request);
            w.set_trigsel(options.trigger_source.map(u8::from).unwrap_or(0));
            w.set_trigpol(options.trigger_polarity.into());
            w.set_trigm(options.trigger_mode.into());
        });
        ch.tr3().write(|_| {}); // no address offsets.
        ch.br1().write(|w| w.set_bndt(bndt));

        match dir {
            Dir::MemoryToPeripheral => {
                ch.sar().write_value(mem_addr as _);
                ch.dar().write_value(peri_addr as _);
            }
            Dir::PeripheralToMemory => {
                ch.sar().write_value(peri_addr as _);
                ch.dar().write_value(mem_addr as _);
            }
        }

        ch.cr().write(|w| {
            // Enable interrupts
            w.set_tcie(true);
            w.set_useie(true);
            w.set_dteie(true);
            w.set_suspie(true);

            // Start it
            w.set_en(true);
        });

        this
    }

    /// Request the transfer to stop.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_stop(&mut self) {
        let info = self.channel.info();
        let ch = info.dma.ch(info.num);

        ch.cr().modify(|w| w.set_susp(true))
    }

    /// Return whether this transfer is still running.
    ///
    /// If this returns `false`, it can be because either the transfer finished, or
    /// it was requested to stop early with [`request_stop`](Self::request_stop).
    pub fn is_running(&mut self) -> bool {
        let info = self.channel.info();
        let ch = info.dma.ch(info.num);

        let sr = ch.sr().read();
        !sr.tcf() && !sr.suspf()
    }

    /// Gets the total remaining transfers for the channel
    /// Note: this will be zero for transfers that completed without cancellation.
    pub fn get_remaining_transfers(&self) -> u16 {
        let info = self.channel.info();
        let ch = info.dma.ch(info.num);

        ch.br1().read().bndt()
    }

    /// Blocking wait until the transfer finishes.
    pub fn blocking_wait(mut self) {
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);

        core::mem::forget(self);
    }
}

impl<'a> Drop for Transfer<'a> {
    fn drop(&mut self) {
        self.request_stop();
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }
}

impl<'a> Unpin for Transfer<'a> {}
impl<'a> Future for Transfer<'a> {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let state = &STATE[self.channel.id as usize];
        state.waker.register(cx.waker());

        if self.is_running() {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }
}
