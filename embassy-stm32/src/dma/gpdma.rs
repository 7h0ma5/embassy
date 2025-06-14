#![macro_use]

use core::future::{poll_fn, Future};
use core::pin::Pin;
use core::sync::atomic::{fence, AtomicUsize, Ordering};
use core::task::{Context, Poll, Waker};

use embassy_hal_internal::Peri;
use embassy_sync::waitqueue::AtomicWaker;

use super::ringbuffer::{DmaCtrl, Error, ReadableDmaRingBuffer, WritableDmaRingBuffer};
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
    /// Priority of the transfer.
    pub priority: TransferPriority,
    /// Trigger source for the transfer. The number of the source is dependent
    /// on the selected microcontroller and is listed its the reference manual.
    pub trigger_source: u8,
    /// The trigger mode for the transfer.
    pub trigger_mode: TriggerMode,
    /// The transfer is triggered on the selected edge polarity.
    /// When the polarity is set to None, the trigger is disabled.
    pub trigger_polarity: TriggerPolarity,
    /// Amount of data to transfer in a single burst from the source.
    pub src_burst_len: u8,
    /// Amount of data to transfer in a single burst to the destination.
    pub dst_burst_len: u8,
}

impl Default for TransferOptions {
    fn default() -> Self {
        Self {
            priority: TransferPriority::LowWithLowWeight,
            trigger_source: 0,
            trigger_mode: TriggerMode::Block,
            trigger_polarity: TriggerPolarity::None,
            src_burst_len: 1,
            dst_burst_len: 1,
        }
    }
}

/// Defines the priority of a DMA transfer.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TransferPriority {
    /// Low priority with low weight.
    LowWithLowWeight,
    /// Low priority with medium weight.
    LowWithMidWeight,
    /// Low priority with high weight.
    LowWithHighWeigt,
    /// High priority.
    High,
}

impl From<TransferPriority> for vals::Prio {
    fn from(value: TransferPriority) -> Self {
        match value {
            TransferPriority::LowWithLowWeight => Self::LOW_WITH_MID_WEIGHT,
            TransferPriority::LowWithMidWeight => Self::LOW_WITH_MID_WEIGHT,
            TransferPriority::LowWithHighWeigt => Self::LOW_WITH_HIGH_WEIGHT,
            TransferPriority::High => Self::HIGH,
        }
    }
}

/// Polarity of the transfer trigger.
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
            TriggerPolarity::None => Self::NONE,
            TriggerPolarity::RisingEdge => Self::RISING_EDGE,
            TriggerPolarity::FallingEdge => Self::FALLING_EDGE,
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
            TriggerMode::LinkedListItem => Self::LINKED_LIST_ITEM,
            TriggerMode::Block2D => Self::_2DBLOCK,
        }
    }
}

impl From<WordSize> for vals::Dw {
    fn from(raw: WordSize) -> Self {
        match raw {
            WordSize::OneByte => Self::BYTE,
            WordSize::TwoBytes => Self::HALF_WORD,
            WordSize::FourBytes => Self::WORD,
        }
    }
}

pub(crate) struct ChannelState {
    waker: AtomicWaker,
    circular_address: AtomicUsize,
    complete_count: AtomicUsize,
}

impl ChannelState {
    pub(crate) const NEW: Self = Self {
        waker: AtomicWaker::new(),
        circular_address: AtomicUsize::new(0),
        complete_count: AtomicUsize::new(0),
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

        if sr.htf() {
            //clear the flag for the half transfer complete
            ch.fcr().modify(|w| w.set_htf(true));
            state.waker.wake();
        }

        if sr.tcf() {
            //clear the flag for the transfer complete
            ch.fcr().modify(|w| w.set_tcf(true));
            state.complete_count.fetch_add(1, Ordering::Release);
            state.waker.wake();
        }
        
        if sr.suspf() {
            ch.fcr().modify(|w| w.set_suspf(true));

            // disable all xxIEs to prevent the irq from firing again.
            ch.cr().modify(|w| {
                w.set_tcie(false);
                w.set_useie(false);
                w.set_dteie(false);
                w.set_suspie(false);
                w.set_htie(false);
            });

            state.waker.wake();
        }
    }
}

/// DMA transfer.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Transfer<'a> {
    channel: Peri<'a, AnyChannel>,
}

impl<'a> Transfer<'a> {
    /// Create a new read DMA transfer (peripheral to memory).
    pub unsafe fn new_read<W: Word>(
        channel: Peri<'a, impl Channel>,
        request: Request,
        peri_addr: *mut W,
        buf: &'a mut [W],
        options: TransferOptions,
    ) -> Self {
        Self::new_read_raw(channel, request, peri_addr, buf, options)
    }

    /// Create a new read DMA transfer (peripheral to memory), using raw pointers.
    pub unsafe fn new_read_raw<W: Word>(
        channel: Peri<'a, impl Channel>,
        request: Request,
        peri_addr: *mut W,
        buf: *mut [W],
        options: TransferOptions,
    ) -> Self {
        Self::new_inner(
            channel.into(),
            request,
            Dir::PeripheralToMemory,
            peri_addr as *const u32,
            buf as *mut W as *mut u32,
            buf.len(),
            true,
            W::size(),
            W::size(),
            options,
        )
    }

    /// Create a new write DMA transfer (memory to peripheral).
    pub unsafe fn new_write<MW: Word, PW: Word>(
        channel: Peri<'a, impl Channel>,
        request: Request,
        buf: &'a [MW],
        peri_addr: *mut PW,
        options: TransferOptions,
    ) -> Self {
        Self::new_write_raw(channel, request, buf, peri_addr, options)
    }

    /// Create a new write DMA transfer (memory to peripheral), using raw pointers.
    pub unsafe fn new_write_raw<MW: Word, PW: Word>(
        channel: Peri<'a, impl Channel>,
        request: Request,
        buf: *const [MW],
        peri_addr: *mut PW,
        options: TransferOptions,
    ) -> Self {
        Self::new_inner(
            channel.into(),
            request,
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            buf as *const MW as *mut u32,
            buf.len(),
            true,
            MW::size(),
            PW::size(),
            options,
        )
    }

    /// Create a new write DMA transfer (memory to peripheral), writing the same value repeatedly.
    pub unsafe fn new_write_repeated<MW: Word, PW: Word>(
        channel: Peri<'a, impl Channel>,
        request: Request,
        repeated: &'a MW,
        count: usize,
        peri_addr: *mut PW,
        options: TransferOptions,
    ) -> Self {
        Self::new_inner(
            channel.into(),
            request,
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            repeated as *const MW as *mut u32,
            count,
            false,
            MW::size(),
            PW::size(),
            options,
        )
    }

    unsafe fn new_inner(
        channel: Peri<'a, AnyChannel>,
        request: Request,
        dir: Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: bool,
        data_size: WordSize,
        dst_size: WordSize,
        options: TransferOptions,
    ) -> Self {
        // BNDT is specified as bytes, not as number of transfers.
        let Ok(bndt) = (mem_len * data_size.bytes()).try_into() else {
            panic!("DMA transfers may not be larger than 65535 bytes.");
        };

        if !(1..=63).contains(&options.src_burst_len) || !(1..=63).contains(&options.dst_burst_len) {
            panic!("DMA transfer burst length must lie between 1 and 63.");
        };

        let state: &ChannelState = &STATE[channel.id as usize];

        let info = channel.info();
        let ch = info.dma.ch(info.num);

        // "Preceding reads and writes cannot be moved past subsequent writes."
        fence(Ordering::SeqCst);

        let this = Self { channel };

        state.complete_count.swap(0, Ordering::Release);

        ch.cr().write(|w| w.set_reset(true));
        ch.fcr().write(|w| w.0 = 0xFFFF_FFFF); // clear all irqs
        ch.llr().write(|_| {}); // no linked list
        ch.tr1().write(|w| {
            w.set_sdw(data_size.into());
            w.set_ddw(dst_size.into());
            w.set_sinc(dir == Dir::MemoryToPeripheral && incr_mem);
            w.set_dinc(dir == Dir::PeripheralToMemory && incr_mem);
            w.set_sbl_1(options.src_burst_len - 1);
            w.set_dbl_1(options.dst_burst_len - 1);

            match dir {
                Dir::MemoryToPeripheral => {
                    w.set_sap(vals::Ap::PORT1);
                    w.set_dap(vals::Ap::PORT0);
                }
                Dir::PeripheralToMemory => {
                    w.set_sap(vals::Ap::PORT0);
                    w.set_dap(vals::Ap::PORT1);
                }
            }
        });
        ch.tr2().write(|w| {
            w.set_dreq(match dir {
                Dir::MemoryToPeripheral => vals::Dreq::DESTINATION_PERIPHERAL,
                Dir::PeripheralToMemory => vals::Dreq::SOURCE_PERIPHERAL,
            });
            w.set_reqsel(request);
            w.set_trigm(options.trigger_mode.into());
            w.set_trigsel(options.trigger_source);
            w.set_trigpol(options.trigger_polarity.into());
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
            // Set the priority
            w.set_prio(options.priority.into());

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
        let state = &STATE[self.channel.id as usize];

        let sr = ch.sr().read();
        let tcf = state.complete_count.load(Ordering::Acquire) != 0;

        !sr.idlef() && !tcf && !sr.suspf()
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

struct DmaCtrlImpl<'a> {
    channel: Peri<'a, AnyChannel>,
    word_size: WordSize,
}

impl<'a> DmaCtrl for DmaCtrlImpl<'a> {
    fn get_remaining_transfers(&self) -> usize {
        let info = self.channel.info();
        let ch = info.dma.ch(info.num);
        (ch.br1().read().bndt() / self.word_size.bytes() as u16) as usize
    }

    fn reset_complete_count(&mut self) -> usize {
        STATE[self.channel.id as usize].complete_count.swap(0, Ordering::AcqRel)
    }

    fn set_waker(&mut self, waker: &Waker) {
        STATE[self.channel.id as usize].waker.register(waker);
    }
}

struct RingBuffer {}

impl RingBuffer {
    fn configure<'a, W: Word>(
        ch: &pac::gpdma::Channel,
        channel_index: usize,
        request: Request,
        dir: Dir,
        peri_addr: *mut W,
        buffer: &'a mut [W],
        _options: TransferOptions,
    ) {
        let state = &STATE[channel_index];

        // "Preceding reads and writes cannot be moved past subsequent writes."
        fence(Ordering::SeqCst);

        let mem_addr = buffer.as_ptr() as usize;
        let mem_len = buffer.len();

        ch.cr().write(|w| w.set_reset(true));
        ch.fcr().write(|w| w.0 = 0xFFFF_FFFF); // clear all irqs

        if mem_addr & 0b11 != 0 {
            panic!("circular address must be 4-byte aligned");
        }

        state.circular_address.store(mem_addr, Ordering::Release);
        let lli = state.circular_address.as_ptr() as u32;
        ch.llr().write(|w| {
            match dir {
                Dir::MemoryToPeripheral => w.set_usa(true),
                Dir::PeripheralToMemory => w.set_uda(true),
            }
            // lower 16 bits of the memory address
            w.set_la(((lli >> 2usize) & 0x3fff) as u16);
        });
        ch.lbar().write(|w| {
            // upper 16 bits of the address of lli1
            w.set_lba((lli >> 16usize) as u16);
        });

        let data_size = W::size();
        ch.tr1().write(|w| {
            w.set_sdw(data_size.into());
            w.set_ddw(data_size.into());
            w.set_sinc(dir == Dir::MemoryToPeripheral);
            w.set_dinc(dir == Dir::PeripheralToMemory);
        });
        ch.tr2().write(|w| {
            w.set_dreq(match dir {
                Dir::MemoryToPeripheral => vals::Dreq::DESTINATION_PERIPHERAL,
                Dir::PeripheralToMemory => vals::Dreq::SOURCE_PERIPHERAL,
            });
            w.set_reqsel(request);
        });
        ch.br1().write(|w| {
            // BNDT is specified as bytes, not as number of transfers.
            w.set_bndt((mem_len * data_size.bytes()) as u16)
        });

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
    }

    fn clear_irqs(ch: &pac::gpdma::Channel) {
        ch.fcr().modify(|w| {
            w.set_htf(true);
            w.set_tcf(true);
            w.set_suspf(true);
        });
    }

    fn is_running(ch: &pac::gpdma::Channel) -> bool {
        !ch.sr().read().tcf()
    }

    fn request_suspend(ch: &pac::gpdma::Channel) {
        ch.cr().modify(|w| {
            w.set_susp(true);
        });
    }

    async fn stop(ch: &pac::gpdma::Channel, set_waker: &mut dyn FnMut(&Waker)) {
        use core::sync::atomic::compiler_fence;

        Self::request_suspend(ch);

        //wait until cr.susp reads as true
        poll_fn(|cx| {
            set_waker(cx.waker());

            compiler_fence(Ordering::SeqCst);

            let cr = ch.cr().read();
            if cr.susp() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }

    fn start(ch: &pac::gpdma::Channel) {
        Self::clear_irqs(ch);
        ch.cr().modify(|w| {
            w.set_susp(false);
            w.set_en(true);
            w.set_tcie(true);
            w.set_useie(true);
            w.set_dteie(true);
            w.set_suspie(true);
            w.set_htie(true);
        });
    }
}

/// This is a Readable ring buffer. It reads data from a peripheral into a buffer. The reads happen in circular mode.
/// There are interrupts on complete and half complete. You should read half the buffer on every read.
pub struct ReadableRingBuffer<'a, W: Word> {
    channel: Peri<'a, AnyChannel>,
    ringbuf: ReadableDmaRingBuffer<'a, W>,
}

impl<'a, W: Word> ReadableRingBuffer<'a, W> {
    /// Create a new Readable ring buffer.
    pub unsafe fn new(
        channel: Peri<'a, impl Channel>,
        request: Request,
        peri_addr: *mut W,
        buffer: &'a mut [W],
        options: TransferOptions,
    ) -> Self {
        let channel: Peri<'a, AnyChannel> = channel.into();

        let info = channel.info();

        RingBuffer::configure(
            &info.dma.ch(info.num),
            channel.id as usize,
            request,
            Dir::PeripheralToMemory,
            peri_addr,
            buffer,
            options,
        );

        Self {
            channel,
            ringbuf: ReadableDmaRingBuffer::new(buffer),
        }
    }

    /// Start reading the peripheral in ciccular mode.
    pub fn start(&mut self) {
        let info = self.channel.info();
        let ch = &info.dma.ch(info.num);
        RingBuffer::start(ch);
    }

    /// Request the transfer to stop. Use is_running() to see when the transfer is complete.
    pub fn request_stop(&mut self) {
        let info = self.channel.info();
        RingBuffer::request_suspend(&info.dma.ch(info.num));
    }
    /// Await until the stop completes. This is not used with request_stop(). Just call and await.
    /// It will stop when the current transfer is complete.
    pub async fn stop(&mut self) {
        let info = self.channel.info();
        RingBuffer::stop(&info.dma.ch(info.num), &mut |waker| self.set_waker(waker)).await
    }

    /// Clear the buffers internal pointers.
    pub fn clear(&mut self) {
        self.ringbuf.reset(&mut DmaCtrlImpl {
            channel: self.channel.reborrow(),
            word_size: W::size(),
        });
    }

    /// Read elements from the ring buffer
    /// Return a tuple of the length read and the length remaining in the buffer
    /// If not all of the elements were read, then there will be some elements in the buffer remaining
    /// The length remaining is the capacity, ring_buf.len(), less the elements remaining after the read
    /// OverrunError is returned if the portion to be read was overwritten by the DMA controller.
    pub fn read(&mut self, buf: &mut [W]) -> Result<(usize, usize), Error> {
        self.ringbuf.read(
            &mut DmaCtrlImpl {
                channel: self.channel.reborrow(),
                word_size: W::size(),
            },
            buf,
        )
    }

    /// Read an exact number of elements from the ringbuffer.
    ///
    /// Returns the remaining number of elements available for immediate reading.
    /// OverrunError is returned if the portion to be read was overwritten by the DMA controller.
    ///
    /// Async/Wake Behavior:
    /// The underlying DMA peripheral only can wake us when its buffer pointer has reached the halfway point,
    /// and when it wraps around. This means that when called with a buffer of length 'M', when this
    /// ring buffer was created with a buffer of size 'N':
    /// - If M equals N/2 or N/2 divides evenly into M, this function will return every N/2 elements read on the DMA source.
    /// - Otherwise, this function may need up to N/2 extra elements to arrive before returning.
    pub async fn read_exact(&mut self, buffer: &mut [W]) -> Result<usize, Error> {
        self.ringbuf
            .read_exact(
                &mut DmaCtrlImpl {
                    channel: self.channel.reborrow(),
                    word_size: W::size(),
                },
                buffer,
            )
            .await
    }

    /// The capacity of the ringbuffer
    pub const fn cap(&self) -> usize {
        self.ringbuf.cap()
    }

    /// Set the waker for the DMA controller.
    pub fn set_waker(&mut self, waker: &Waker) {
        DmaCtrlImpl {
            channel: self.channel.reborrow(),
            word_size: W::size(),
        }
        .set_waker(waker);
    }

    /// Return whether this transfer is still running.
    pub fn is_running(&mut self) -> bool {
        let info = self.channel.info();
        RingBuffer::is_running(&info.dma.ch(info.num))
    }
}

impl<'a, W: Word> Drop for ReadableRingBuffer<'a, W> {
    fn drop(&mut self) {
        self.request_stop();
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }
}

/// This is a Writable ring buffer. It writes data from a buffer to a peripheral. The writes happen in circular mode.
pub struct WritableRingBuffer<'a, W: Word> {
    #[allow(dead_code)] //this is only read by the DMA controller
    channel: Peri<'a, AnyChannel>,
    ringbuf: WritableDmaRingBuffer<'a, W>,
}

impl<'a, W: Word> WritableRingBuffer<'a, W> {
    /// Create a new Writable ring buffer.
    pub unsafe fn new(
        channel: Peri<'a, impl Channel>,
        request: Request,
        peri_addr: *mut W,
        buffer: &'a mut [W],
        options: TransferOptions,
    ) -> Self {
        let channel: Peri<'a, AnyChannel> = channel.into();

        let info = channel.info();

        RingBuffer::configure(
            &info.dma.ch(info.num),
            channel.id as usize,
            request,
            Dir::MemoryToPeripheral,
            peri_addr,
            buffer,
            options,
        );

        Self {
            channel,
            ringbuf: WritableDmaRingBuffer::new(buffer),
        }
    }

    /// Start writing to the peripheral in circular mode.
    pub fn start(&mut self) {
        let info = self.channel.info();
        RingBuffer::start(&info.dma.ch(info.num));
    }

    /// Await until the stop completes. This is not used with request_stop(). Just call and await.
    pub async fn stop(&mut self) {
        let info = self.channel.info();
        RingBuffer::stop(&info.dma.ch(info.num), &mut |waker| self.set_waker(waker)).await
    }

    /// Request the transfer to stop. Use is_running() to see when the transfer is complete.
    pub fn request_stop(&mut self) {
        // reads can be stopped by disabling the enable flag
        let info = self.channel.info();
        let ch = &info.dma.ch(info.num);
        ch.cr().modify(|w| w.set_en(false));
    }

    /// Write elements directly to the raw buffer.
    /// This can be used to fill the buffer before starting the DMA transfer.
    pub fn write_immediate(&mut self, buf: &[W]) -> Result<(usize, usize), Error> {
        self.ringbuf.write_immediate(buf)
    }

    /// Clear the buffers internal pointers.
    pub fn clear(&mut self) {
        self.ringbuf.reset(&mut DmaCtrlImpl {
            channel: self.channel.reborrow(),
            word_size: W::size(),
        });
    }

    /// Write elements from the ring buffer
    /// Return a tuple of the length written and the length remaining in the buffer
    pub fn write(&mut self, buf: &[W]) -> Result<(usize, usize), Error> {
        self.ringbuf.write(
            &mut DmaCtrlImpl {
                channel: self.channel.reborrow(),
                word_size: W::size(),
            },
            buf,
        )
    }

    /// Write an exact number of elements to the ringbuffer.
    pub async fn write_exact(&mut self, buffer: &[W]) -> Result<usize, Error> {
        self.ringbuf
            .write_exact(
                &mut DmaCtrlImpl {
                    channel: self.channel.reborrow(),
                    word_size: W::size(),
                },
                buffer,
            )
            .await
    }

    /// The capacity of the ringbuffer
    pub const fn cap(&self) -> usize {
        self.ringbuf.cap()
    }

    /// Set the waker for the DMA controller.
    pub fn set_waker(&mut self, waker: &Waker) {
        DmaCtrlImpl {
            channel: self.channel.reborrow(),
            word_size: W::size(),
        }
        .set_waker(waker);
    }

    /// Return whether this transfer is still running.
    pub fn is_running(&mut self) -> bool {
        let info = self.channel.info();
        RingBuffer::is_running(&info.dma.ch(info.num))
    }
}

impl<'a, W: Word> Drop for WritableRingBuffer<'a, W> {
    fn drop(&mut self) {
        self.request_stop();
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }
}
