//! Command link framing: device-to-host responses and host-to-device bytes.
//!
//! Command handlers never touch the wire: they build a [`Response`] and enqueue
//! it. The upload task of the compiled-in link ([`crate::service::uart`] or
//! [`crate::service::ble`]) drains the queue independently, so a slow or
//! absent host cannot stall the command loop.
//!
//! Exactly one link is compiled in (see the `uart` / `ble` cargo features):
//! - UART always drains, so frames are pushed with back-pressure (a full queue
//!   blocks the producer instead of dropping responses).
//! - BLE only drains while connected, so it stays best effort.

use crate::drivers::motor::Pid;
#[cfg(feature = "ble")]
use crate::service::ble;
use crate::service::control::State;
#[cfg(feature = "uart")]
use crate::service::uart;
use protocol::{ErrorCode, PidData, RequestType, Response, Status};

/// Handle to the compiled-in command link: inbound byte stream and outbound
/// frame queue. Cheap `Copy`; owns no peripheral.
#[derive(Clone, Copy)]
pub(crate) struct Link {
    #[cfg(feature = "uart")]
    uart: uart::UartTxSender,
    #[cfg(feature = "ble")]
    ble: ble::BleTxSender,
}

impl Link {
    /// Attach to the compiled-in link's channels.
    pub(crate) fn new() -> Self {
        Self {
            #[cfg(feature = "uart")]
            uart: uart::UART_TX.sender(),
            #[cfg(feature = "ble")]
            ble: ble::BLE_TX.sender(),
        }
    }

    /// Wait for the next inbound byte from the host.
    pub(crate) async fn receive(&self) -> u8 {
        #[cfg(feature = "uart")]
        return uart::UART_RX.receive().await;
        #[cfg(feature = "ble")]
        return ble::BLE_RX.receive().await;
    }

    /// Encode and enqueue one response; encoding a response never fails in practice.
    pub(crate) async fn send_response(&self, response: Response) {
        match response.to_frame() {
            Ok(frame) => {
                #[cfg(feature = "uart")]
                self.uart.send(frame).await;
                #[cfg(feature = "ble")]
                let _ = self.ble.send(frame).await;
            }
            Err(err) => defmt::warn!("link: response encode failed ({:?})", err),
        }
    }

    pub(crate) async fn send_ack(&self, cmd: RequestType) {
        self.send_response(Response::Ack { cmd: cmd.as_u8() }).await;
    }

    pub(crate) async fn send_nack(&self, cmd: u8, error: ErrorCode) {
        self.send_response(Response::Nack { cmd, error }).await;
    }

    pub(crate) async fn send_status(&self, st: &State) {
        self.send_response(Response::Status(Status {
            sys: st.sys,
            rpm: [st.rpm0, st.rpm1],
            dist_active: st.odometry.active,
            dist_result: st.odometry.result,
        }))
        .await;
    }

    pub(crate) async fn send_pid(&self, pid: &Pid) {
        self.send_response(Response::PidData(PidData {
            kp: (pid.kp * 100.0) as i16,
            ki: (pid.ki * 100.0) as i16,
            kd: (pid.kd * 100.0) as i16,
        }))
        .await;
    }
}
