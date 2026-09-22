//! UART0 command link (RX=GPIO20 / TX=GPIO21, 115200 8N1).
//!
//! Ownership: this module owns the peripheral. The control loop never touches
//! it directly — both directions go through channels:
//! - [`UART_TX`]: frames queued by producers ([`crate::service::link::Link`]),
//!   uploaded by the TX half of [`uart_task`].
//! - [`UART_RX`]: bytes collected by the RX half, drained by the control loop.

use embassy_futures::join::join;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Receiver, Sender},
};
use esp_hal::{
    Async,
    peripherals::{GPIO20, GPIO21, UART0},
    uart::{Config as UartConfig, DataBits, Parity, StopBits, Uart, UartRx, UartTx},
};

use protocol::Frame;

/// Frames queued for UART0 upload.
pub static UART_TX: Channel<CriticalSectionRawMutex, Frame, 16> = Channel::new();
/// Bytes received from UART0, drained by the control loop.
pub static UART_RX: Channel<CriticalSectionRawMutex, u8, 256> = Channel::new();

/// Sender half of [`UART_TX`].
pub type UartTxSender = Sender<'static, CriticalSectionRawMutex, Frame, 16>;
/// Receiver half of [`UART_TX`].
pub type UartTxReceiver = Receiver<'static, CriticalSectionRawMutex, Frame, 16>;

/// Own the UART0 peripheral; run RX and TX loops side by side.
#[embassy_executor::task]
pub async fn uart_task(device: UART0<'static>, tx_pin: GPIO21<'static>, rx_pin: GPIO20<'static>) {
    let uart_cfg = UartConfig::default()
        .with_baudrate(115_200)
        .with_data_bits(DataBits::_8)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1);
    let uart = Uart::new(device, uart_cfg)
        .unwrap()
        .with_tx(tx_pin)
        .with_rx(rx_pin)
        .into_async();
    let (rx, tx) = uart.split();
    join(rx_task(rx), tx_task(UART_TX.receiver(), tx)).await;
}

/// Collect received bytes into [`UART_RX`] (best effort, like BLE RX).
async fn rx_task(mut rx: UartRx<'static, Async>) {
    loop {
        let mut byte = [0u8; 1];
        match rx.read_async(&mut byte).await {
            Ok(1) => {
                let _ = UART_RX.try_send(byte[0]);
            }
            Ok(_) => {}
            Err(_) => {}
        }
    }
}

/// Upload queued frames to UART0, one frame at a time.
async fn tx_task(rx: UartTxReceiver, mut tx: UartTx<'static, Async>) {
    loop {
        let frame = rx.receive().await;
        let data = frame.as_bytes();
        let mut off = 0;
        while off < data.len() {
            match tx.write_async(&data[off..]).await {
                Ok(0) => break,
                Ok(k) => off += k,
                Err(_) => break,
            }
        }
        let _ = tx.flush_async().await;
    }
}
