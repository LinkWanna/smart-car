//! Quadrature encoder inputs with CHANGE interrupts.
//!
//! Wiring (Arduino AKA-00):
//! L: ENC_A=GPIO7, ENC_B=GPIO10. R: ENC_A=GPIO5, ENC_B=GPIO6.
//!
//! Ownership: this module owns the ISR state. `main` builds the `Input`s,
//! hands them to [`Encoders::install`] (one-time boot init), and registers
//! [`gpio_handler`]. The tick task only uses [`Encoders::read`]/[`Encoders::zero`].
//!
//! [`Encoder`] holds one motor's decode state (pins + counter); [`Encoders`]
//! holds both motors, so every operation goes through a struct method. Pins
//! are plain fields (no `Option`): the instance only exists after `install`,
//! so "not yet installed" is unrepresentable by construction.

use core::cell::RefCell;

use critical_section::Mutex;
use embassy_sync::once_lock::OnceLock;
use esp_hal::gpio::Input;
use portable_atomic::{AtomicI32, Ordering};

/// One motor's quadrature decode state.
struct Encoder {
    a: Input<'static>,
    b: Input<'static>,
    count: AtomicI32,
}

impl Encoder {
    fn new(a: Input<'static>, b: Input<'static>) -> Self {
        Self {
            a,
            b,
            count: AtomicI32::new(0),
        }
    }

    fn on_interrupt(&mut self) {
        let fa = self.a.is_interrupt_set();
        let fb = self.b.is_interrupt_set();
        if fa || fb {
            let la = self.a.is_high();
            let lb = self.b.is_high();
            if fa {
                Self::edge(&self.count, la, lb, false);
                self.a.clear_interrupt();
            }
            if fb {
                Self::edge(&self.count, la, lb, true);
                self.b.clear_interrupt();
            }
        }
    }

    fn edge(count: &AtomicI32, a_high: bool, b_high: bool, b_edge: bool) {
        let delta = if b_edge {
            if a_high == b_high { 1 } else { -1 }
        } else {
            if a_high != b_high { 1 } else { -1 }
        };
        count.fetch_add(delta, Ordering::Relaxed);
    }

    fn read(&self) -> i32 {
        self.count.load(Ordering::Relaxed)
    }

    fn zero(&self) {
        self.count.store(0, Ordering::Relaxed);
    }
}

/// Both motors' decode state.
pub(crate) struct Encoders {
    m0: Encoder,
    m1: Encoder,
}

static ENCODERS: OnceLock<Mutex<RefCell<Encoders>>> = OnceLock::new();

impl Encoders {
    pub(crate) fn install(
        e0a: Input<'static>,
        e0b: Input<'static>,
        e1a: Input<'static>,
        e1b: Input<'static>,
    ) {
        ENCODERS
            .init(Mutex::new(RefCell::new(Self {
                m0: Encoder::new(e0a, e0b),
                m1: Encoder::new(e1a, e1b),
            })))
            .ok()
            .expect("encoders installed twice (boot order bug)");
    }

    fn get() -> &'static Mutex<RefCell<Encoders>> {
        ENCODERS
            .try_get()
            .expect("encoders used before install (boot order bug)")
    }

    pub(crate) fn read() -> (i32, i32) {
        critical_section::with(|cs| {
            let enc = Self::get().borrow_ref(cs);
            (enc.m0.read(), enc.m1.read())
        })
    }

    pub(crate) fn zero() {
        critical_section::with(|cs| {
            let enc = Self::get().borrow_ref(cs);
            enc.m0.zero();
            enc.m1.zero();
        })
    }
}

#[esp_hal::handler]
pub(crate) fn gpio_handler() {
    critical_section::with(|cs| {
        let mut enc = Encoders::get().borrow_ref_mut(cs);
        enc.m0.on_interrupt();
        enc.m1.on_interrupt();
    })
}
