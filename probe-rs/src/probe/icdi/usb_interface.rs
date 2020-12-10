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

const DEFAULT_BUF_SIZE: usize = 128;

/// The USB VendorID.
pub const USB_VID: u16 = 0x1cbe; // Luminary Micro Inc.

pub const TIMEOUT: Duration = Duration::from_millis(1000);

lazy_static! {
    /// Map of USB PID to firmware version name and device endpoints.
    pub static ref USB_PID_EP_MAP: HashMap<u16, ICDIInfo> = {
        let mut m = HashMap::new();
        m.insert(0x00fd, ICDIInfo::new("Unknown-Version",    0x00fd, 0x02,   0x83));
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
    read_buffer : Vec<u8>,
    read_bytes: usize,
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
            read_buffer: vec![0u8;DEFAULT_BUF_SIZE],
            read_bytes : 0,
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

    [b"$", payload, format!("#{:02x}", cksum).as_bytes()].concat()
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
        self.read_bytes = 0;
        self.read_buffer = vec![0; 128 as usize];

        if !write_data.is_empty() {
            let mut write_acked = false;

            for retry in 0..3 {
                log::debug!(
                    "Will write {} bytes (+1byte for $ and +3 bytes for CHKSUM #xx)",
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
                self.read_bytes = self
                    .device_handle
                    .read_bulk(ep_in, &mut self.read_buffer, timeout)
                    .map_err(|e| DebugProbeError::USB(Some(Box::new(e)))).unwrap();

                log::debug!("read something to see if ack: {:?} : {:?}", self.read_bytes, self.read_buffer);

                if self.read_bytes > 0 && self.read_buffer[0] == '-' as u8 {
                    log::debug!("looping, write not ACKed with '-'");
                } else if (self.read_bytes > 0 && self.read_buffer[0] != b'+') || self.read_bytes == 0  {
                    log::debug!("Unexpected answer {} / '{}'", self.read_buffer[0], self.read_buffer[0] as char);
                    // unexpected
                    return Err(IcdiError::FixMeError(line!()).into());
                } else if self.read_bytes > 0 {
                    log::debug!("write ACKed");
                    self.read_buffer.remove(0); // shift the leading +
                    self.read_bytes -= 1;
                    write_acked = true;
                    break;
                }
            }

            if !write_acked {
                log::debug!("write not ACKed");
                return Err(IcdiError::InvalidProtocol.into());
            } else {
                log::debug!("write has been ACKed");
            }
        }

        // Optional data in phase.

        // add 4 bytes: $OK: <data> #xx
        //            let mut read_data_pkt = vec![0u8; read_data.len() + 7];
        log::debug!("Will read data from endpoint (already {} in buffer)", self.read_bytes);
        self.read_bytes += self
            .device_handle
            .read_bulk(ep_in, &mut self.read_buffer[self.read_bytes..], timeout)
            .map_err(|e| DebugProbeError::USB(Some(Box::new(e))))?;
        log::debug!("Buffer is now {}", self.read_bytes);
        log::debug!(" --> {:?}", &self.read_buffer[..self.read_bytes]);


        if self.read_bytes < 4 || self.read_buffer[self.read_bytes - 3] != b'#' {
            return Err(IcdiError::InvalidProtocol.into());
        }

        if self.read_bytes > 5 && self.read_buffer[3] == b':' {
            verify_packet(&self.read_buffer[3..self.read_bytes])?;
            // &read_data[..self.read_bytes - (1 + 3 + 3)]
            //     .copy_from_slice(&self.read_buffer[4..self.read_bytes - 3]);
        } else {
            verify_packet(&self.read_buffer[..self.read_bytes])?;
            // &read_data[..self.read_bytes - 4].copy_from_slice(&self.read_buffer[1..self.read_bytes - 3]);
        }
        log::debug!("read step done, checksum OK");


        if !read_data.is_empty() {
            log::debug!("Copying read data ({} bytes total) to caller buffer ({} bytes)", self.read_bytes, read_data.len());
            if self.read_bytes > 5 && self.read_buffer[3] == b':' {
                log::debug!("Skipping the 'OK:' prefix");
                &read_data[..self.read_bytes - (1 + 3 + 3)]
                    .copy_from_slice(&self.read_buffer[4..self.read_bytes - 3]);
            } else {
                log::debug!("Not skipping anything");
                &read_data[..self.read_bytes - 4].copy_from_slice(&self.read_buffer[1..self.read_bytes - 3]);
            }
        } else {
            log::debug!("Discarding read data");
        }

        Ok((written_bytes, self.read_bytes))
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
