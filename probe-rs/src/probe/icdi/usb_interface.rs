use hex;
use lazy_static::lazy_static;
use rusb::{Context, DeviceHandle, Error, UsbContext};
use std::num::Wrapping;
use std::time::Duration;

use crate::probe::icdi::IcdiError;

use std::collections::HashMap;

use super::tools::{is_icdi_device, read_serial_number};
use crate::{
    probe::{DebugProbeError, ProbeCreationError},
    DebugProbeSelector,
};

/// The USB Command packet size.
const CMD_LEN: usize = 16;

/// The USB VendorID.
pub const USB_VID: u16 = 0x1cbe; // Luminary Micro Inc.

pub const TIMEOUT: Duration = Duration::from_millis(1000);

lazy_static! {
    /// Map of USB PID to firmware version name and device endpoints.
    pub static ref USB_PID_EP_MAP: HashMap<u16, ICDIInfo> = {
        let mut m = HashMap::new();
        m.insert(0x00fd, ICDIInfo::new("Unknown-Version",    0x00fd, 0x02,   0x83));
        // m.insert(0x374b, STLinkInfo::new("V2-1",  0x374b, 0x01,   0x81,   0x82));
        // m.insert(0x374a, STLinkInfo::new("V2-1",  0x374a, 0x01,   0x81,   0x82));  // Audio
        // m.insert(0x3742, STLinkInfo::new("V2-1",  0x3742, 0x01,   0x81,   0x82));  // No MSD
        // m.insert(0x3752, STLinkInfo::new("V2-1",  0x3752, 0x01,   0x81,   0x82));  // Unproven
        // m.insert(0x374e, STLinkInfo::new("V3",    0x374e, 0x01,   0x81,   0x82));
        // m.insert(0x374f, STLinkInfo::new("V3",    0x374f, 0x01,   0x81,   0x82));  // Bridge
        // m.insert(0x3753, STLinkInfo::new("V3",    0x3753, 0x01,   0x81,   0x82));  // 2VCP
        m
    };
}

/// A helper struct to match STLink deviceinfo.
#[derive(Clone, Debug, Default)]
pub struct ICDIInfo {
    pub version_name: String,
    pub usb_pid: u16,
    ep_out: u8,
    ep_in: u8,
    // ep_swo: u8,
}

impl ICDIInfo {
    pub fn new<V: Into<String>>(
        version_name: V,
        usb_pid: u16,
        ep_out: u8,
        ep_in: u8,
        // ep_swo: u8,
    ) -> Self {
        Self {
            version_name: version_name.into(),
            usb_pid,
            ep_out,
            ep_in,
            // ep_swo,
        }
    }
}

pub(crate) struct ICDIUSBDevice {
    device_handle: DeviceHandle<rusb::Context>,
    info: ICDIInfo,
}

impl std::fmt::Debug for ICDIUSBDevice {
    fn fmt(&self, fmt: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        fmt.debug_struct("ICDIUSBDevice")
            .field("device_handle", &"DeviceHandle<rusb::Context>")
            .field("info", &self.info)
            .finish()
    }
}

pub trait IcdiUsb: std::fmt::Debug {
    fn write(
        &mut self,
        write_data: &[u8],
        read_data: &mut [u8],
        timeout: Duration,
    ) -> Result<(usize, usize), DebugProbeError>;

    fn write_remote(
        &mut self,
        write_data: &[u8],
        read_data: &mut [u8],
        timeout: Duration,
    ) -> Result<(usize, usize), DebugProbeError>;

    /// Reset the USB device. This can be used to recover when the
    /// STLink does not respond to USB requests.
    fn reset(&mut self) -> Result<(), DebugProbeError>;

    // fn read_swo(
    //     &mut self,
    //     read_data: &mut [u8],
    //     timeout: Duration,
    // ) -> Result<usize, DebugProbeError>;
}

impl ICDIUSBDevice {
    /// Creates and initializes a new USB device.
    pub fn new_from_selector(
        selector: impl Into<DebugProbeSelector>,
    ) -> Result<Self, ProbeCreationError> {
        log::debug!("starting");
        let selector = selector.into();

        let context = Context::new()?;

        log::debug!("Acquired libusb context.");

        let device = context
            .devices()?
            .iter()
            .filter(is_icdi_device)
            .find_map(|device| {
                let descriptor = device.device_descriptor().ok()?;
                // First match the VID & PID.
                if selector.vendor_id == descriptor.vendor_id()
                    && selector.product_id == descriptor.product_id()
                {
                    // If the VID & PID match, match the serial if one was given.
                    if let Some(serial) = &selector.serial_number {
                        let sn_str = read_serial_number(&device, &descriptor).ok();
                        if sn_str.as_ref() == Some(serial) {
                            Some(device)
                        } else {
                            None
                        }
                    } else {
                        // If no serial was given, the VID & PID match is enough; return the device.
                        Some(device)
                    }
                } else {
                    None
                }
            })
            .map_or(Err(ProbeCreationError::NotFound), Ok)?;

        let mut device_handle = device.open()?;

        log::debug!("Aquired handle for probe");

        let config = device.active_config_descriptor()?;

        log::debug!("Active config descriptor: {:?}", &config);

        let descriptor = device.device_descriptor()?;

        log::debug!("Device descriptor: {:?}", &descriptor);

        let info = USB_PID_EP_MAP[&descriptor.product_id()].clone();

        log::debug!("Going to claim interface");

        device_handle.claim_interface(2).unwrap();

        log::debug!("Claimed interface 2 of USB device.");

        let mut endpoint_out = false;
        let mut endpoint_in = false;

        if let Some(interface) = config.interfaces().nth(2) {
            if let Some(descriptor) = interface.descriptors().next() {
                for endpoint in descriptor.endpoint_descriptors() {
                    log::debug!("EP {:?}", endpoint.address());

                    if endpoint.address() == info.ep_out {
                        endpoint_out = true;
                    } else if endpoint.address() == info.ep_in {
                        endpoint_in = true;
                    }
                }
            }
        }
        log::debug!("EP found {} {}.", endpoint_in, endpoint_out);
        if !endpoint_out {
            return Err(IcdiError::EndpointNotFound.into());
        }

        if !endpoint_in {
            return Err(IcdiError::EndpointNotFound.into());
        }

        let usb_icdi = Self {
            device_handle,
            info,
        };

        log::debug!("Succesfully attached to ICDI.");

        Ok(usb_icdi)
    }

    /// Closes the USB interface gracefully.
    /// Internal helper.
    fn close(&mut self) -> Result<(), Error> {
        self.device_handle.release_interface(0)
    }
}

fn create_packet(payload: &[u8]) -> Vec<u8> {
    /* calculate checksum - offset start of packet */
    let mut cksum = 0u8;
    for c in payload {
        cksum = c.wrapping_add(cksum);
    }

    [
        "$".as_bytes(),
        payload,
        format!("#{:02x}", cksum).as_bytes(),
    ]
    .concat()
}

fn verify_packet(packet: &[u8]) -> Result<(), DebugProbeError> {
    let mut cksum = 0u8;

    for c in &packet[1..packet.len() - 3] {
        cksum = c.wrapping_add(cksum);
    }

    match hex::decode(&packet[packet.len() - 2..]) {
        Ok(v) => {
            if cksum != v[0] {
                Err(IcdiError::InvalidCheckSum {
                    is: v[0],
                    should: cksum,
                }
                .into())
            } else {
                Ok(())
            }
        }
        _ => Err(IcdiError::InvalidCheckSum {
            is: 0,
            should: cksum,
        }
        .into()),
    }
}

impl IcdiUsb for ICDIUSBDevice {
    /// Writes to the out EP and reads back data if needed.
    /// First the `cmd` is sent.
    /// In a second step `write_data` is transmitted.
    /// And lastly, data will be read back until `read_data` is filled.
    fn write(
        &mut self,
        write_data: &[u8],
        read_data: &mut [u8],
        timeout: Duration,
    ) -> Result<(usize, usize), DebugProbeError> {
        log::trace!("Sending something to ICDI, timeout: {:?}", timeout);

        let ep_out = self.info.ep_out;
        let ep_in = self.info.ep_in;

        let mut written_bytes = 0;

        for retry in 0..3 {
            if !write_data.is_empty() {
                log::debug!(
                    "Will write {} bytes (+3 bytes for CHKSUM)",
                    write_data.len()
                );

                let packet = create_packet(write_data);

                log::debug!(" --> really write {} bytes", packet.len());
                log::debug!(" --> {:x?}", packet);
                log::debug!(" --> {:?}", std::str::from_utf8(&packet).unwrap());
                let written_bytes = self
                    .device_handle
                    .write_bulk(ep_out, &packet, timeout)
                    .map_err(|e| DebugProbeError::USB(Some(Box::new(e))))?;

                if written_bytes != packet.len() {
                    return Err(IcdiError::NotEnoughBytesRead {
                        is: written_bytes,
                        should: packet.len(),
                    }
                    .into());
                }
                log::debug!("write step done");
                let mut ack = [0u8];
                let read_bytes = self
                    .device_handle
                    .read_bulk(ep_in, &mut ack, timeout)
                    .map_err(|e| DebugProbeError::USB(Some(Box::new(e))))?;
                if read_bytes == 1 && ack[0] == '-' as u8 {
                    log::debug!("looping, write not ACKed : {}", ack[0]);
                } else if read_bytes == 1 && ack[0] != '+' as u8 {
                    // unexpected
                    return Err(IcdiError::FixMeError(line!()).into());
                } else {
                    log::debug!("write ACKed");
                    break;
                }
            }
        }

        let mut read_bytes = 0;

        // Optional data in phase.
        if !read_data.is_empty() {
            let mut read_data_pkt = vec![0u8; read_data.len() + 4];
            log::debug!("Will read {} bytes", read_data_pkt.len());
            read_bytes = self
                .device_handle
                .read_bulk(ep_in, &mut read_data_pkt, timeout)
                .map_err(|e| DebugProbeError::USB(Some(Box::new(e))))?;
            // log::debug!("read finished {} bytes", read_bytes);
            // log::debug!("read finished {:?} bytes", read_data_pkt);
            // if read_bytes != read_data_pkt.len() {
            //     return Err(IcdiError::NotEnoughBytesRead {
            //         is: read_bytes,
            //         should: read_data_pkt.len(),
            //     }
            //     .into());
            // }
            log::debug!(" --> {:?}", std::str::from_utf8(&read_data_pkt[..read_bytes]).unwrap());
            if read_bytes < 4 || read_data_pkt[read_bytes - 3] != '#' as u8 {
                return Err(IcdiError::InvalidProtocol.into());
            }

            verify_packet(&read_data_pkt[..read_bytes])?;

            &read_data[..read_bytes - 4].copy_from_slice(&read_data_pkt[1..read_bytes - 3]);
            log::debug!("read step done, checksum OK");
        }
        Ok((written_bytes, read_bytes))
    }

    fn write_remote(
        &mut self,
        //        cmd: &[u8],
        write_data: &[u8],
        read_data: &mut [u8],
        timeout: Duration,
    ) -> Result<(usize, usize), DebugProbeError> {
        log::debug!("write remote");
        log::debug!("data to send : {:?}", write_data);
        log::debug!(
            "data to send hexified : {:?}",
            hex::encode(write_data).as_bytes()
        );
        let data = ["qRcmd,".as_bytes(), hex::encode(write_data).as_bytes()].concat();
        self.write(&data, read_data, timeout)
    }

    // fn read_swo(
    //     &mut self,
    //     read_data: &mut [u8],
    //     timeout: Duration,
    // ) -> Result<usize, DebugProbeError> {
    //     log::trace!(
    //         "Reading {:?} SWO bytes to ICDI, timeout: {:?}",
    //         read_data.len(),
    //         timeout
    //     );

    //     let ep_swo = self.info.ep_swo;

    //     if read_data.is_empty() {
    //         Ok(0)
    //     } else {
    //         let read_bytes = self
    //             .device_handle
    //             .read_bulk(ep_swo, read_data, timeout)
    //             .map_err(|e| DebugProbeError::USB(Some(Box::new(e))))?;
    //         Ok(read_bytes)
    //     }
    // }

    /// Reset the USB device. This can be used to recover when the
    /// STLink does not respond to USB requests.
    fn reset(&mut self) -> Result<(), DebugProbeError> {
        log::debug!("Resetting USB device of ICDI");
        self.device_handle
            .reset()
            .map_err(|e| DebugProbeError::USB(Some(Box::new(e))))
    }
}

impl Drop for ICDIUSBDevice {
    fn drop(&mut self) {
        // We ignore the error case as we can't do much about it anyways.
        let _ = self.close();
    }
}
