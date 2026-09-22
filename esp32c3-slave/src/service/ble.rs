//! BLE command link (NUS byte pipe).
//!
//! One GATT service (`car`): a transparent byte pipe carrying the existing
//! `0xAA55` protocol frames in both directions (RX write / TX notify).
//!
//! The host stack is `trouble-host` on top of the `esp-radio` BLE controller.

use portable_atomic::{AtomicBool, Ordering};

use embassy_futures::join::join;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Channel, Sender as ChannelSender},
};
use embassy_time::Timer;
use esp_hal::efuse::{self, InterfaceMacAddress};
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

use crate::protocol::Frame;

/// Payload size that fits a notification at the minimum (23 byte) ATT MTU.
pub const CHUNK: usize = 20;
/// Advertised device name.
pub const DEVICE_NAME: &str = "ESP32C3-CAR";

/// Maximum number of simultaneous connections.
pub const CONNECTIONS_MAX: usize = 1;
/// Maximum number of L2CAP channels (signal + att).
pub const L2CAP_CHANNELS_MAX: usize = 2;

/// NUS service UUID in little endian byte order (scan response data).
const NUS_SERVICE_UUID_LE: [u8; 16] = [
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e,
];

/// Frames from the car to the host (best effort).
pub static BLE_TX: Channel<CriticalSectionRawMutex, Frame, 16> = Channel::new();
/// Raw bytes received from the host, drained by the command loop.
pub static BLE_RX: Channel<CriticalSectionRawMutex, u8, 256> = Channel::new();
/// Set when a central disconnects so the control loop can stop the car.
pub static LINK_LOST: AtomicBool = AtomicBool::new(false);

/// Sender half of [`BLE_TX`].
pub type BleTxSender = ChannelSender<'static, CriticalSectionRawMutex, Frame, 16>;

#[gatt_server]
struct Server {
    car: CarService,
}

/// Nordic UART Service style byte pipe.
#[gatt_service(uuid = "6e400001-b5a3-f393-e0a9-e50e24dcca9e")]
struct CarService {
    /// Host -> car: command frame bytes.
    #[characteristic(
        uuid = "6e400002-b5a3-f393-e0a9-e50e24dcca9e",
        write,
        write_without_response
    )]
    rx: [u8; CHUNK],
    /// Car -> host: response frame bytes.
    #[characteristic(uuid = "6e400003-b5a3-f393-e0a9-e50e24dcca9e", notify)]
    tx: [u8; CHUNK],
}

/// Static random address derived from the chip MAC.
fn local_address() -> Address {
    let mac = efuse::interface_mac_address(InterfaceMacAddress::Bluetooth);
    let mut bytes = [0u8; 6];
    bytes.copy_from_slice(&mac.as_bytes()[..6]);
    // Static random addresses require the two most significant bits to be set.
    bytes[5] |= 0xC0;
    Address::random(bytes)
}

/// Init the controller and run the BLE stack forever.
#[embassy_executor::task]
pub async fn ble_task(device: esp_hal::peripherals::BT<'static>) -> ! {
    let connector = BleConnector::new(device, Default::default()).expect("BLE init failed");
    let controller: ExternalController<_, 1> = ExternalController::new(connector);

    let mut resources: HostResources<_, DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(local_address())
        .build();

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: DEVICE_NAME,
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .expect("BLE gap config failed");

    let mut peripheral = stack.peripheral();
    let runner = stack.runner();

    defmt::info!("ble: advertising as {}", DEVICE_NAME);

    join(
        async move {
            let mut runner = runner;
            loop {
                if let Err(e) = runner.run().await {
                    defmt::warn!("ble: runner error {:?}", e);
                    Timer::after_millis(100).await;
                }
            }
        },
        async move {
            loop {
                match advertise(&mut peripheral, &server).await {
                    Ok(conn) => {
                        defmt::info!("ble: connected");
                        join(rx_task(&server, &conn), tx_task(&server, &conn)).await;
                        LINK_LOST.store(true, Ordering::Relaxed);
                        defmt::info!("ble: disconnected");
                    }
                    Err(e) => {
                        defmt::warn!("ble: advertise error {:?}", e);
                    }
                }
                Timer::after_millis(200).await;
            }
        },
    )
    .await;

    unreachable!("BLE stack terminated")
}

/// Advertise and wait for a connection (re-created for every connection).
async fn advertise<'values, 'server, C: Controller>(
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(DEVICE_NAME.as_bytes()),
        ],
        &mut adv_data[..],
    )?;

    let mut scan_data = [0; 31];
    let scan_len = AdStructure::encode_slice(
        &[AdStructure::CompleteServiceUuids128(&[NUS_SERVICE_UUID_LE])],
        &mut scan_data[..],
    )?;

    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..len],
                scan_data: &scan_data[..scan_len],
            },
        )
        .await?;

    advertiser
        .accept()
        .await?
        .with_attribute_server(server)
        .map_err(BleHostError::from)
}

/// Handle GATT events until the connection drops.
async fn rx_task<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) {
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                defmt::info!("ble: link lost ({:?})", reason);
                break;
            }
            GattConnectionEvent::Gatt { event } => {
                if let GattEvent::Write(write) = &event
                    && write.handle() == server.car.rx.handle
                {
                    write.with_data(|offset, data| {
                        if offset == 0 {
                            for &b in data {
                                let _ = BLE_RX.try_send(b);
                            }
                        } else {
                            defmt::warn!("ble: unexpected write offset {}", offset);
                        }
                    });
                }
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(_) => defmt::warn!("ble: gatt reply failed"),
                }
            }
            _ => {}
        }
    }
}

/// Forward queued frames to the host, chunked to fit the ATT MTU.
async fn tx_task<P: PacketPool>(server: &Server<'_>, conn: &GattConnection<'_, '_, P>) {
    loop {
        let frame = BLE_TX.receive().await;
        let data = frame.as_bytes();
        let mut off = 0;
        while off < data.len() {
            let n = (data.len() - off).min(CHUNK);
            if server
                .car
                .tx
                .notify_raw(conn, &data[off..off + n], false)
                .await
                .is_err()
            {
                return;
            }
            off += n;
        }
    }
}
