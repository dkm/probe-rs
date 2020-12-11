use rusb::Device;
use rusb::UsbContext;

use crate::probe::{DebugProbeInfo, DebugProbeType};

use super::usb_interface::USB_PID_EP_MAP;
use super::usb_interface::USB_VID;
use std::time::Duration;

pub(super) fn is_icdi_device<T: UsbContext>(device: &Device<T>) -> bool {
    // Check the VID/PID.
    if let Ok(descriptor) = device.device_descriptor() {
        (descriptor.vendor_id() == USB_VID)
            && (USB_PID_EP_MAP.contains_key(&descriptor.product_id()))
    } else {
        false
    }
}

pub fn list_icdi_devices() -> Vec<DebugProbeInfo> {
    if let Ok(context) = rusb::Context::new() {
        if let Ok(devices) = context.devices() {
            devices
                .iter()
                .filter(is_icdi_device)
                .filter_map(|device| {
                    let descriptor = device.device_descriptor().ok()?;

                    let sn_str = match read_serial_number(&device, &descriptor) {
                        Ok(serial_number) => Some(serial_number),
                        Err(e) => {
                            // Reading the serial number can fail, e.g. if the driver for the probe
                            // is not installed. In this case we can still list the probe,
                            // just without serial number.
                            log::debug!(
                                "Failed to read serial number of device {:#06x}:{:#06x} : {}",
                                descriptor.product_id(),
                                descriptor.vendor_id(),
                                e
                            );
                            log::debug!("This might be happening because of a missing driver.");
                            None
                        }
                    };

                    Some(DebugProbeInfo::new(
                        format!(
                            "ICDI {}",
                            &USB_PID_EP_MAP[&descriptor.product_id()].version_name
                        ),
                        descriptor.vendor_id(),
                        descriptor.product_id(),
                        sn_str,
                        DebugProbeType::ICDI,
                    ))
                })
                .collect::<Vec<_>>()
        } else {
            vec![]
        }
    } else {
        vec![]
    }
}

/// Try to read the serial number of a USB device.
pub(super) fn read_serial_number<T: rusb::UsbContext>(
    device: &rusb::Device<T>,
    descriptor: &rusb::DeviceDescriptor,
) -> Result<String, rusb::Error> {
    let timeout = Duration::from_millis(100);

    let handle = device.open()?;
    let language = handle
        .read_languages(timeout)?
        .get(0)
        .cloned()
        .ok_or(rusb::Error::BadDescriptor)?;
    let sn = handle.read_serial_number_string(language, &descriptor, timeout);
    sn.map(|s| {
        if s.len() < 24 {
            // Some STLink (especially V2) have their serial number stored as a 12 bytes binary string
            // containing non printable characters, so convert to a hex string to make them printable.
            s.as_bytes().iter().map(|b| format!("{:02X}", b)).collect()
        } else {
            // Other STlink (especially V2-1) have their serial number already stored as a 24 characters
            // hex string so they don't need to ba converted
            s
        }
    })
}
