pub mod constants;
pub mod tools;
mod usb_interface;

use self::usb_interface::{ICDIUSBDevice, IcdiUsb};
use super::{DAPAccess, DebugProbe, DebugProbeError, PortType, ProbeCreationError, WireProtocol};
use crate::{
    architecture::arm::{
        ap::{
            valid_access_ports, APAccess, APClass, APRegister, AccessPort, BaseaddrFormat,
            GenericAP, MemoryAP, BASE, BASE2, CSW, IDR,
        },
        communication_interface::{ArmCommunicationInterfaceState, ArmProbeInterface},
        dp::{DPAccess, DPBankSel, DPRegister, DebugPortError, Select},
        memory::Component,
        ApInformation, ArmChipInfo, SwoAccess, SwoConfig,
    },
    DebugProbeSelector, Error as ProbeRsError, Memory, MemoryInterface, Probe,
};
use constants::{commands, JTagFrequencyToDivider, Mode, Status, SwdFrequencyToDelayCount};
use scroll::{Pread, Pwrite, BE, LE};
use std::{cmp::Ordering, convert::TryInto, time::Duration};
use thiserror::Error;
use usb_interface::TIMEOUT;

#[derive(Debug)]
pub struct ICDI<D: IcdiUsb> {
    device: D,
    hw_version: u8,
    jtag_version: u8,
    //    protocol: WireProtocol,
    speed_khz: u32,

    icdi_version: u32,
    packet_size: usize,

    /// List of opened APs
    openend_aps: Vec<u8>,
}

impl DebugProbe for ICDI<ICDIUSBDevice> {
    fn new_from_selector(
        selector: impl Into<DebugProbeSelector>,
    ) -> Result<Box<Self>, DebugProbeError> {
        let mut icdi = Self {
            device: ICDIUSBDevice::new_from_selector(selector)?,
            hw_version: 0,
            jtag_version: 0,
            //          protocol: WireProtocol::Swd,
            //            swd_speed_khz: 1_800,
            speed_khz: 1_120,
            icdi_version: 0,
            packet_size: 0,

            openend_aps: vec![],
        };

        icdi.init()?;

        Ok(Box::new(icdi))
    }

    fn get_name(&self) -> &str {
        "ICDI"
    }

    fn speed(&self) -> u32 {
        self.speed_khz
    }

    fn set_speed(&mut self, speed_khz: u32) -> Result<u32, DebugProbeError> {
        Err(DebugProbeError::UnsupportedSpeed(speed_khz))
    }

    fn attach(&mut self) -> Result<(), DebugProbeError> {
        log::debug!("attach");
        Ok(())
        //        Err(IcdiError::FixMeError(line!()).into())
        // self.enter_idle()?;

        // log::debug!("Switching protocol to JTAG");
        // let param = commands::JTAG_ENTER_JTAG_NO_CORE_RESET;

        // let mut buf = [0; 2];
        // self.send_jtag_command(
        //     &[commands::JTAG_COMMAND, commands::JTAG_ENTER2, param, 0],
        //     &[],
        //     &mut buf,
        //     TIMEOUT,
        // )?;

        // log::debug!("Successfully initialized SWD.");

        // // If the speed is not manually set, the probe will
        // // use whatever speed has been configured before.
        // //
        // // To ensure the default speed is used if not changed,
        // // we set the speed again here.
        // self.set_speed(self.speed_khz)?;

        // Ok(())
    }

    fn detach(&mut self) -> Result<(), DebugProbeError> {
        log::debug!("Detaching from ICDI.");
        Ok(())
    }

    fn target_reset(&mut self) -> Result<(), DebugProbeError> {
        self.device
            .write_remote(b"debug hreset", &mut [], TIMEOUT)?;

        self.device
            .write_remote(b"set vectorcatch 0", &mut [], TIMEOUT)?;

        self.device
            .write_remote(b"debug disable", &mut [], TIMEOUT)?;

        Ok(())
    }

    fn target_reset_assert(&mut self) -> Result<(), DebugProbeError> {
        log::error!("ICDI target_reset_assert");
        unimplemented!()
    }

    fn target_reset_deassert(&mut self) -> Result<(), DebugProbeError> {
        log::error!("ICDI target_reset_assert");
        unimplemented!()
    }

    fn select_protocol(&mut self, protocol: WireProtocol) -> Result<(), DebugProbeError> {
        if protocol != WireProtocol::Jtag {
            Err(DebugProbeError::UnsupportedProtocol(protocol))
        } else {
            Ok(())
        }
    }

    fn get_swo_interface(&self) -> Option<&dyn SwoAccess> {
        None
    }

    fn get_swo_interface_mut(&mut self) -> Option<&mut dyn SwoAccess> {
        None
    }

    fn get_arm_interface<'probe>(
        self: Box<Self>,
    ) -> Result<Option<Box<dyn ArmProbeInterface + 'probe>>, DebugProbeError> {
        //unimplemented!();
        let interface = IcdiArmDebug::new(self)?;

        Ok(Some(Box::new(interface)))
    }

    fn has_arm_interface(&self) -> bool {
        true
    }
}

impl<'a> AsRef<dyn DebugProbe + 'a> for ICDI<ICDIUSBDevice> {
    fn as_ref(&self) -> &(dyn DebugProbe + 'a) {
        self
    }
}

impl<'a> AsMut<dyn DebugProbe + 'a> for ICDI<ICDIUSBDevice> {
    fn as_mut(&mut self) -> &mut (dyn DebugProbe + 'a) {
        self
    }
}

impl<D: IcdiUsb> Drop for ICDI<D> {
    fn drop(&mut self) {
        // We ignore the error cases as we can't do much about it anyways.
        // if self.swo_enabled {
        //     let _ = self.disable_swo();
        // }
        //        let _ = self.enter_idle();
    }
}

fn check_result(result: &str) -> Result<(), DebugProbeError> {
    if result.starts_with("OK") {
        Ok(())
    } else if result.starts_with('E') {
        let num = result[1..].parse::<u32>().unwrap();
        Err(IcdiError::ProtocolError(num).into())
    } else {
        Err(IcdiError::InvalidProtocol.into())
    }
}

impl<D: IcdiUsb> ICDI<D> {
    /// Maximum number of bytes to send or receive for 32- and 16- bit transfers.
    ///
    /// 8-bit transfers have a maximum size of the maximum USB packet size (64 bytes for full speed).
    const _MAXIMUM_TRANSFER_SIZE: u32 = 1024;

    /// Minimum required STLink firmware version.
    const MIN_JTAG_VERSION: u8 = 26;

    /// Minimum required STLink V3 firmware version.
    ///
    /// Version 2 of the firmware (V3J2M1) has problems switching communication protocols.
    const MIN_JTAG_VERSION_V3: u8 = 3;

    /// Firmware version that adds multiple AP support.
    const MIN_JTAG_VERSION_MULTI_AP: u8 = 28;

    /// Reads the target voltage.
    /// For the china fake variants this will always read a nonzero value!
    pub fn get_target_voltage(&mut self) -> Result<f32, DebugProbeError> {
        Err(IcdiError::FixMeError(line!()).into())
        // let mut buf = [0; 8];
        // match self
        //     .device
        //     .write(&[commands::GET_TARGET_VOLTAGE], &[], &mut buf, TIMEOUT)
        // {
        //     Ok(_) => {
        //         // The next two unwraps are safe!
        //         let a0 = (&buf[0..4]).pread_with::<u32>(0, LE).unwrap() as f32;
        //         let a1 = (&buf[4..8]).pread_with::<u32>(0, LE).unwrap() as f32;
        //         if a0 != 0.0 {
        //             Ok((2.0 * a1 * 1.2 / a0) as f32)
        //         } else {
        //             // Should never happen
        //             Err(IcdiError::VoltageDivisionByZero.into())
        //         }
        //     }
        //     Err(e) => Err(e),
        // }
    }

    /// Get the current mode of the ST-Link
    fn get_current_mode(&mut self) -> Result<Mode, DebugProbeError> {
        log::trace!("Getting current mode of device...");
        Err(IcdiError::FixMeError(line!()).into())
        // self.device
        //     .write(&[commands::GET_CURRENT_MODE], &[], &mut buf, TIMEOUT)?;

        // use Mode::*;

        // let mode = match buf[0] {
        //     0 => Dfu,
        //     1 => MassStorage,
        //     2 => Jtag,
        //     3 => Swim,
        //     _ => return Err(IcdiError::UnknownMode.into()),
        // };

        // log::debug!("Current device mode: {:?}", mode);

        // Ok(mode)
    }

    /// Commands the ST-Link to enter idle mode.
    /// Internal helper.
    fn enter_idle(&mut self) -> Result<(), DebugProbeError> {
        log::debug!("Will enter idle");
        Err(IcdiError::FixMeError(line!()).into())
        // let mode = self.get_current_mode()?;

        // log::debug!("Mode is {:?}", mode);
        // match mode {
        //     Mode::Dfu => self.device.write(
        //         &[commands::DFU_COMMAND, commands::DFU_EXIT],
        //         &[],
        //         &mut [],
        //         TIMEOUT,
        //     ),
        //     Mode::Swim => self.device.write(
        //         &[commands::SWIM_COMMAND, commands::SWIM_EXIT],
        //         &[],
        //         &mut [],
        //         TIMEOUT,
        //     ),
        //     _ => Ok(()),
        // }
    }

    /// Reads the ST-Links version.
    /// Returns a tuple (hardware version, firmware version).
    /// This method stores the version data on the struct to make later use of it.
    fn get_version(&mut self) -> Result<u32, DebugProbeError> {
        // const HW_VERSION_SHIFT: u8 = 12;
        // const HW_VERSION_MASK: u8 = 0x0F;
        // const JTAG_VERSION_SHIFT: u8 = 6;
        // const JTAG_VERSION_MASK: u8 = 0x3F;

        log::debug!("Get version");

        let mut version = [0u8; 10];
        self.device
            .write_remote(b"version", &mut version, TIMEOUT)?;
        let vs = hex::decode(version).unwrap();

        let v = std::str::from_utf8(&vs[..vs.len() - 1]).unwrap();
        log::debug!("version {:?}", v);
        let vint = v.parse::<u32>().unwrap();

        Ok(vint)
        //            icdi_send_remote_cmd(handle, "version");

        // GET_VERSION response structure:
        //   Byte 0-1:
        //     [15:12] Major/HW version
        //     [11:6]  JTAG/SWD version
        //     [5:0]   SWIM or MSC version
        //   Byte 2-3: ST_VID
        //   Byte 4-5: STLINK_PID

        // let mut buf = [0; 6];
        // match self
        //     .device
        //     .write(&[commands::GET_VERSION], &[], &mut buf, TIMEOUT)
        // {
        //     Ok(_) => {
        //         let version: u16 = (&buf[0..2]).pread_with(0, BE).unwrap();
        //         self.hw_version = (version >> HW_VERSION_SHIFT) as u8 & HW_VERSION_MASK;
        //         self.jtag_version = (version >> JTAG_VERSION_SHIFT) as u8 & JTAG_VERSION_MASK;
        //     }
        //     Err(e) => return Err(e),
        // }

        // // For the STLinkV3 we must use the extended get version command.
        // if self.hw_version >= 3 {
        //     // GET_VERSION_EXT response structure (byte offsets)
        //     //  0: HW version
        //     //  1: SWIM version
        //     //  2: JTAG/SWD version
        //     //  3: MSC/VCP version
        //     //  4: Bridge version
        //     //  5-7: reserved
        //     //  8-9: ST_VID
        //     //  10-11: STLINK_PID
        //     let mut buf = [0; 12];
        //     match self
        //         .device
        //         .write(&[commands::GET_VERSION_EXT], &[], &mut buf, TIMEOUT)
        //     {
        //         Ok(_) => {
        //             let version: u8 = (&buf[2..3]).pread_with(0, LE).unwrap();
        //             self.jtag_version = version;
        //         }
        //         Err(e) => return Err(e),
        //     }
        // }

        // // Make sure everything is okay with the firmware we use.
        // if self.jtag_version == 0 {
        //     return Err(IcdiError::JTAGNotSupportedOnProbe.into());
        // }
        // if self.hw_version < 3 && self.jtag_version < Self::MIN_JTAG_VERSION {
        //     return Err(DebugProbeError::ProbeFirmwareOutdated);
        // }
        // if self.hw_version == 3 && self.jtag_version < Self::MIN_JTAG_VERSION_V3 {
        //     return Err(DebugProbeError::ProbeFirmwareOutdated);
        // }

        // Ok((self.hw_version, self.jtag_version))
    }

    fn set_extended_mode(&mut self) -> Result<(), DebugProbeError> {
        log::debug!("Get supported");

        let mut reply = [0u8; 128];
        self.device.write(b"!", &mut reply, TIMEOUT)?;
        let reply = std::str::from_utf8(&reply).unwrap();
        log::debug!("result '{}'", reply);

        check_result(reply)

    }
    fn get_supported(&mut self) -> Result<usize, DebugProbeError> {
        log::debug!("Get supported");

        let mut version = [0u8; 256];
        self.device.write(b"qSupported", &mut version, TIMEOUT)?;
        let version_str = std::str::from_utf8(&version[..version.len() - 1]).unwrap();

        if let Some(start_pkt_sz) = version_str.find("PacketSize=") {
            if let Some(end_pkt_sz) = version_str[start_pkt_sz..].find(";") {
                let packet_size = version_str[start_pkt_sz + "PacketSize=".len()..end_pkt_sz]
                    .parse::<usize>()
                    .unwrap();
                log::debug!("packet size {}", packet_size);
                return Ok(packet_size);
            }
        }

        Err(IcdiError::InvalidProtocol.into())
    }

    /// Opens the ST-Link USB device and tries to identify the ST-Links version and its target voltage.
    /// Internal helper.
    fn init(&mut self) -> Result<(), DebugProbeError> {
        log::debug!("Initializing ICDI...");

        // if let Err(e) = self.enter_idle() {
        //     match e {
        //         DebugProbeError::USB(_) => {
        //             // Reset the device, and try to enter idle mode again
        //             self.device.reset()?;

        //             self.enter_idle()?;
        //         }
        //         // Other error occured, return it
        //         _ => return Err(e),
        //     }
        // }

        log::debug!("Will get version");
        self.icdi_version = self.get_version()?;
        log::debug!("ICDI version: {:?}", self.icdi_version);
        self.packet_size = self.get_supported()?;
        log::debug!("ICDI packet size: {:?}", self.packet_size);
        self.set_extended_mode()?;

        Ok(())
    }

    /// sets the SWD frequency.
    // pub fn set_swd_frequency(
    //     &mut self,
    //     frequency: SwdFrequencyToDelayCount,
    // ) -> Result<(), DebugProbeError> {
    //     let mut buf = [0; 2];
    //     self.send_jtag_command(
    //         &[
    //             commands::JTAG_COMMAND,
    //             commands::SWD_SET_FREQ,
    //             frequency as u8,
    //         ],
    //         &[],
    //         &mut buf,
    //         TIMEOUT,
    //     )
    // }

    /// Sets the JTAG frequency.
    // pub fn set_jtag_frequency(
    //     &mut self,
    //     frequency: JTagFrequencyToDivider,
    // ) -> Result<(), DebugProbeError> {
    //     let mut buf = [0; 2];
    //     self.send_jtag_command(
    //         &[
    //             commands::JTAG_COMMAND,
    //             commands::JTAG_SET_FREQ,
    //             frequency as u8,
    //         ],
    //         &[],
    //         &mut buf,
    //         TIMEOUT,
    //     )
    // }

    /// Sets the communication frequency (V3 only)
    // fn set_communication_frequency(
    //     &mut self,
    //     protocol: WireProtocol,
    //     frequency_khz: u32,
    // ) -> Result<(), DebugProbeError> {
    //     if self.hw_version != 3 {
    //         return Err(DebugProbeError::CommandNotSupportedByProbe);
    //     }

    //     let cmd_proto = match protocol {
    //         WireProtocol::Swd => 0,
    //         WireProtocol::Jtag => 1,
    //     };

    //     let mut command = vec![commands::JTAG_COMMAND, commands::SET_COM_FREQ, cmd_proto, 0];
    //     command.extend_from_slice(&frequency_khz.to_le_bytes());

    //     let mut buf = [0; 8];
    //     self.send_jtag_command(&command, &[], &mut buf, TIMEOUT)
    // }

    /// Returns the current and available communication frequencies (V3 only)
    // fn get_communication_frequencies(
    //     &mut self,
    //     protocol: WireProtocol,
    // ) -> Result<(Vec<u32>, u32), DebugProbeError> {
    //     if self.hw_version != 3 {
    //         return Err(DebugProbeError::CommandNotSupportedByProbe);
    //     }

    //     let cmd_proto = match protocol {
    //         WireProtocol::Swd => 0,
    //         WireProtocol::Jtag => 1,
    //     };

    //     let mut buf = [0; 52];
    //     self.send_jtag_command(
    //         &[commands::JTAG_COMMAND, commands::GET_COM_FREQ, cmd_proto],
    //         &[],
    //         &mut buf,
    //         TIMEOUT,
    //     )?;

    //     let mut values = (&buf)
    //         .chunks(4)
    //         .map(|chunk| chunk.pread_with::<u32>(0, LE).unwrap())
    //         .collect::<Vec<u32>>();

    //     let current = values[1];
    //     let n = core::cmp::min(values[2], 10) as usize;

    //     values.rotate_left(3);
    //     values.truncate(n);

    //     Ok((values, current))
    // }

    /// Select an AP to use
    ///
    /// On newer ST-Links (JTAG Version >= 28), multiple APs are supported.
    /// To switch between APs, dedicated commands have to be used. For older
    /// ST-Links, we can only use AP 0. If an AP other than 0 is used on these
    /// probes, an error is returned.
    fn select_ap(&mut self, ap: u8) -> Result<(), DebugProbeError> {
        // Check if we can use APs other an AP 0.
        // Older versions of the ST-Link software don't support this.
        if self.hw_version < 3 && self.jtag_version < Self::MIN_JTAG_VERSION_MULTI_AP {
            if ap == 0 {
                return Ok(());
            } else {
                return Err(DebugProbeError::ProbeFirmwareOutdated);
            }
        }

        if !self.openend_aps.contains(&ap) {
            log::debug!("Opening AP {}", ap);
            self.open_ap(ap)?;
            self.openend_aps.push(ap)
        }

        Ok(())
    }

    /// Open a specific AP, which will be used for all future commands.
    ///
    /// This is only supported on ST-Link V3, or older ST-Links with
    /// a JTAG version >= `MIN_JTAG_VERSION_MULTI_AP`.
    fn open_ap(&mut self, apsel: u8) -> Result<(), DebugProbeError> {
        // Ensure this command is actually supported
        if self.hw_version < 3 && self.jtag_version < Self::MIN_JTAG_VERSION_MULTI_AP {
            return Err(DebugProbeError::CommandNotSupportedByProbe);
        }

        let mut buf = [0; 2];
        log::trace!("JTAG_INIT_AP {}", apsel);
        self.send_jtag_command(
            &[commands::JTAG_COMMAND, commands::JTAG_INIT_AP, apsel],
            &[],
            &mut buf,
            TIMEOUT,
        )
    }

    /// Close a specific AP, which was opened with `open_ap`.
    ///
    /// This is only supported on ST-Link V3, or older ST-Links with
    /// a JTAG version >= `MIN_JTAG_VERSION_MULTI_AP`.
    fn _close_ap(&mut self, apsel: u8) -> Result<(), DebugProbeError> {
        // Ensure this command is actually supported
        if self.hw_version < 3 && self.jtag_version < Self::MIN_JTAG_VERSION_MULTI_AP {
            return Err(DebugProbeError::CommandNotSupportedByProbe);
        }

        let mut buf = [0; 2];
        log::trace!("JTAG_CLOSE_AP {}", apsel);
        self.send_jtag_command(
            &[commands::JTAG_COMMAND, commands::JTAG_CLOSE_AP_DBG, apsel],
            &[],
            &mut buf,
            TIMEOUT,
        )
    }

    fn send_jtag_command(
        &mut self,
        cmd: &[u8],
        write_data: &[u8],
        read_data: &mut [u8],
        timeout: Duration,
    ) -> Result<(), DebugProbeError> {
        // for attempt in 0..13 {
        //     self.device.write(cmd, write_data, read_data, timeout)?;

        //     match Status::from(read_data[0]) {
        //         Status::JtagOk => return Ok(()),
        //         Status::SwdDpWait => {
        //             log::warn!("send_jtag_command {} got SwdDpWait, retrying", cmd[0])
        //         }
        //         Status::SwdApWait => {
        //             log::warn!("send_jtag_command {} got SwdApWait, retrying", cmd[0])
        //         }
        //         status => {
        //             log::warn!("send_jtag_command {} failed: {:?}", cmd[0], status);
        //             return Err(From::from(IcdiError::CommandFailed(status)));
        //         }
        //     }

        //     // Sleep with exponential backoff.
        //     std::thread::sleep(Duration::from_micros(100 << attempt));
        // }

        // log::warn!("too many retries, giving up");

        // // Return the last error (will be SwdDpWait or SwdApWait)
        // let status = Status::from(read_data[0]);
        // return Err(From::from(IcdiError::CommandFailed(status)));
        Err(IcdiError::FixMeError(line!()).into())
    }

    pub fn start_trace_reception(&mut self, config: &SwoConfig) -> Result<(), DebugProbeError> {
        let mut buf = [0; 2];
        let bufsize = 4096u16.to_le_bytes();
        let baud = config.baud().to_le_bytes();
        let mut command = vec![commands::JTAG_COMMAND, commands::SWO_START_TRACE_RECEPTION];
        command.extend_from_slice(&bufsize);
        command.extend_from_slice(&baud);

        self.send_jtag_command(&command, &[], &mut buf, TIMEOUT)?;

        // self.swo_enabled = true;

        Ok(())
    }

    pub fn stop_trace_reception(&mut self) -> Result<(), DebugProbeError> {
        let mut buf = [0; 2];

        self.send_jtag_command(
            &[commands::JTAG_COMMAND, commands::SWO_STOP_TRACE_RECEPTION],
            &[],
            &mut buf,
            TIMEOUT,
        )?;

        // self.swo_enabled = false;

        Ok(())
    }

    /// Gets the SWO count from the ST-Link probe.
    fn read_swo_available_byte_count(&mut self) -> Result<usize, DebugProbeError> {
        Err(IcdiError::FixMeError(line!()).into())
        // let mut buf = [0; 2];
        // self.device.write(
        //     &[
        //         commands::JTAG_COMMAND,
        //         commands::SWO_GET_TRACE_NEW_RECORD_NB,
        //     ],
        //     &[],
        //     &mut buf,
        //     TIMEOUT,
        // )?;
        // Ok(buf.pread::<u16>(0).unwrap() as usize)
    }

    /// Reads the actual data from the SWO buffer on the ST-Link.
    // fn read_swo_data(&mut self, timeout: Duration) -> Result<Vec<u8>, DebugProbeError> {
    //     // The byte count always needs to be polled first, otherwise
    //     // the ST-Link won't return any data.
    //     let mut buf = vec![0; self.read_swo_available_byte_count()?];
    //     let bytes_read = self.device.read_swo(&mut buf, timeout)?;
    //     buf.truncate(bytes_read);
    //     Ok(buf)
    // }

    fn get_last_rw_status(&mut self) -> Result<(), DebugProbeError> {
        let mut receive_buffer = [0u8; 12];
        self.send_jtag_command(
            &[commands::JTAG_COMMAND, commands::JTAG_GETLASTRWSTATUS2],
            &[],
            &mut receive_buffer,
            TIMEOUT,
        )
    }

    fn read_mem_32bit(
        &mut self,
        address: u32,
        length: u16
    ) -> Result<Vec<u32>, DebugProbeError> {
        log::debug!(
            "Read mem 32 bit, address={:08x}, length={}",
            address,
            length
        );

        // // Maximum supported read length is 2^16 bytes.
        // assert!(
        //     length < (u16::MAX / 4),
        //     "Maximum read length for STLink is 16'384 words"
        // );

        // if address % 4 != 0 {
        //     todo!("Should return an error here");
        // }

        let byte_length = length * 4;

        // Longest answer possible:
        // $OK#CC}x}x}x}x}x}x#CC
        // with :
        //  - CC the 2 checksum
        //  - }x an escaped byte payload (occupies 2 bytes)
        //        let mut receive_buffer = vec![0u8; (6 + byte_length*2 + 3) as usize];

        let mut recv_ok = vec![0u8; 2];
        self.device.write(
            &format!("x{:08x},{:x}", address, byte_length).into_bytes(),
            &mut recv_ok,
            TIMEOUT,
        )?;

        let reply = std::str::from_utf8(&recv_ok).unwrap();
        log::debug!("result '{}'", reply);

        check_result(reply)?;

        // self.get_last_rw_status()?;

        let mut recv_payload = vec![0u8; byte_length as usize];
        self.device.write(
            &[],
            &mut recv_payload,
            TIMEOUT,
        )?;

        let words: Vec<u32> = recv_payload
            .chunks_exact(4)
            .map(|chunk| u32::from_le_bytes(chunk.try_into().unwrap()))
            .collect();

        Ok(words)
    }

    fn read_mem_8bit(
        &mut self,
        address: u32,
        length: u16,
        apsel: u8,
    ) -> Result<Vec<u8>, DebugProbeError> {
        log::debug!("Read mem 8 bit, address={:08x}, length={}", address, length);
        Err(IcdiError::FixMeError(line!()).into())
        // let mut receive_buffer = vec![0u8; length as usize];

        // self.device.write(
        //     &[
        //         commands::JTAG_COMMAND,
        //         commands::JTAG_READMEM_8BIT,
        //         address as u8,
        //         (address >> 8) as u8,
        //         (address >> 16) as u8,
        //         (address >> 24) as u8,
        //         length as u8,
        //         (length >> 8) as u8,
        //         apsel,
        //     ],
        //     &[],
        //     &mut receive_buffer,
        //     TIMEOUT,
        // )?;

        // self.get_last_rw_status()?;

        // Ok(receive_buffer)
    }

    fn write_mem_32bit(
        &mut self,
        address: u32,
        data: &[u32],
        apsel: u8,
    ) -> Result<(), DebugProbeError> {
        log::trace!("write_mem_32bit");
        Err(IcdiError::FixMeError(line!()).into())
        // let length = data.len();

        // // Maximum supported read length is 2^16 bytes.
        // assert!(
        //     length < (u16::MAX / 4) as usize,
        //     "Maximum write length for STLink is 16'384 words"
        // );

        // if address % 4 != 0 {
        //     todo!("Should return an error here");
        // }

        // let byte_length = length * 4;

        // let mut tx_buffer = vec![0u8; byte_length];

        // let mut offset = 0;

        // for word in data {
        //     tx_buffer
        //         .gwrite(word, &mut offset)
        //         .expect("Failed to write into tx_buffer");
        // }

        // self.device.write(
        //     &[
        //         commands::JTAG_COMMAND,
        //         commands::JTAG_WRITEMEM_32BIT,
        //         address as u8,
        //         (address >> 8) as u8,
        //         (address >> 16) as u8,
        //         (address >> 24) as u8,
        //         byte_length as u8,
        //         (byte_length >> 8) as u8,
        //         apsel,
        //     ],
        //     &tx_buffer,
        //     &mut [],
        //     TIMEOUT,
        // )?;

        // self.get_last_rw_status()?;

        // Ok(())
    }

    fn write_mem_8bit(
        &mut self,
        address: u32,
        data: &[u8],
        apsel: u8,
    ) -> Result<(), DebugProbeError> {
        log::trace!("write_mem_8bit");
        Err(IcdiError::FixMeError(line!()).into())
        // let byte_length = data.len();

        // if self.hw_version < 3 {
        //     assert!(
        //         byte_length <= 64,
        //         "8-Bit writes are limited to 64 bytes on ST-Link v2"
        //     );
        // } else {
        //     assert!(
        //         byte_length <= 512,
        //         "8-Bit writes are limited to 512 bytes on ST-Link v3"
        //     );
        // }

        // self.device.write(
        //     &[
        //         commands::JTAG_COMMAND,
        //         commands::JTAG_WRITEMEM_8BIT,
        //         address as u8,
        //         (address >> 8) as u8,
        //         (address >> 16) as u8,
        //         (address >> 24) as u8,
        //         byte_length as u8,
        //         (byte_length >> 8) as u8,
        //         apsel,
        //     ],
        //     data,
        //     &mut [],
        //     TIMEOUT,
        // )?;

        // self.get_last_rw_status()?;

        // Ok(())
    }

    fn _read_debug_reg(&mut self, address: u32) -> Result<u32, DebugProbeError> {
        log::trace!("Read debug reg {:08x}", address);
        let mut buff = [0u8; 8];

        self.send_jtag_command(
            &[
                commands::JTAG_COMMAND,
                commands::JTAG_READ_DEBUG_REG,
                address as u8,
                (address >> 8) as u8,
                (address >> 16) as u8,
                (address >> 24) as u8,
            ],
            &[],
            &mut buff,
            TIMEOUT,
        )?;

        let response_val: u32 = buff.pread(4).unwrap();

        Ok(response_val)
    }

    fn _write_debug_reg(&mut self, address: u32, value: u32) -> Result<(), DebugProbeError> {
        log::trace!("Write debug reg {:08x}", address);
        let mut buff = [0u8; 2];

        let mut cmd = [0u8; 2 + 4 + 4];
        cmd[0] = commands::JTAG_COMMAND;
        cmd[1] = commands::JTAG_WRITE_DEBUG_REG;

        cmd.pwrite_with(address, 2, LE).unwrap();
        cmd.pwrite_with(value, 6, LE).unwrap();

        self.send_jtag_command(&cmd, &[], &mut buff, TIMEOUT)
    }

    fn _read_core_reg(&mut self, index: u32) -> Result<u32, DebugProbeError> {
        log::trace!("Read core reg {:08x}", index);
        let mut buff = [0u8; 8];

        let mut cmd = [0u8; 2 + 1];
        cmd[0] = commands::JTAG_COMMAND;
        cmd[1] = commands::JTAG_READ_CORE_REG;

        assert!(index < (u8::MAX as u32));

        cmd[2] = index as u8;

        self.send_jtag_command(&cmd, &[], &mut buff, TIMEOUT)?;

        let response = buff.pread_with(4, LE).unwrap();

        Ok(response)
    }

    fn _write_core_reg(&mut self, index: u32, value: u32) -> Result<(), DebugProbeError> {
        log::trace!("Write core reg {:08x}", index);
        let mut buff = [0u8; 2];

        let mut cmd = [0u8; 2 + 1 + 4];
        cmd[0] = commands::JTAG_COMMAND;
        cmd[1] = commands::JTAG_WRITE_CORE_REG;

        assert!(index < (u8::MAX as u32));

        cmd[2] = index as u8;

        cmd.pwrite_with(value, 3, LE).unwrap();

        self.send_jtag_command(&cmd, &[], &mut buff, TIMEOUT)
    }
}

#[derive(Error, Debug)]
pub(crate) enum IcdiError {
    #[error("Invalid voltage values returned by probe.")]
    VoltageDivisionByZero,
    #[error("Probe is an unknown mode.")]
    UnknownMode,
    #[error("Blank values are not allowed on DebugPort writes.")]
    BlanksNotAllowedOnDPRegister,
    #[error("Not enough bytes read.")]
    NotEnoughBytesRead { is: usize, should: usize },
    #[error("USB endpoint not found.")]
    EndpointNotFound,
    #[error("Command failed with status {0:?}")]
    CommandFailed(Status),
    #[error("JTAG not supported on Probe")]
    JTAGNotSupportedOnProbe,
    #[error("Invalid checksum")]
    InvalidCheckSum { is: u8, should: u8 },
    #[error("ICDI error {0}")]
    ProtocolError(u32),
    #[error("Invalid protocol")]
    InvalidProtocol,
    #[error("Temporary placeholder at {0:?}, FIXME")]
    FixMeError(u32),
    #[error("Mancehster-coded SWO mode not supported")]
    ManchesterSwoNotSupported,
}

impl From<IcdiError> for DebugProbeError {
    fn from(e: IcdiError) -> Self {
        DebugProbeError::ProbeSpecific(Box::new(e))
    }
}

impl From<IcdiError> for ProbeCreationError {
    fn from(e: IcdiError) -> Self {
        ProbeCreationError::ProbeSpecific(Box::new(e))
    }
}

// #[derive(Debug)]
// struct IcdiMemoryInterface<'probe> {
//     probe: &'probe mut ICDI<ICDIUSBDevice>,
//     access_port: MemoryAP,
// }

impl<D: IcdiUsb> MemoryInterface for ICDI<D> {
    fn read_word_32(&mut self, address: u32) -> Result<u32, ProbeRsError> {
//        self.probe.select_ap(self.access_port)?;

        // let mut buff = [0];
        // let cmd = format!("x{x},{x}", address, len)
        // self.probe.

        Ok(0)
    }

    fn read_word_8(&mut self, address: u32) -> Result<u8, ProbeRsError> {
        // self.probe.select_ap(self.access_port)?;

        // let mut buff = [0u8];
        // self.read_8(address, &mut buff)?;

        Ok(0)
    }

    fn read_32(&mut self, address: u32, data: &mut [u32]) -> Result<(), ProbeRsError> {
        // self.probe.select_ap(self.access_port)?;

        // let received_words = self.probe.probe.read_mem_32bit(
        //     address,
        //     data.len() as u16,
        //     self.access_port.port_number(),
        // )?;

        // data.copy_from_slice(&received_words);

        Ok(())
    }

    fn read_8(&mut self, address: u32, data: &mut [u8]) -> Result<(), ProbeRsError> {
        // self.probe.select_ap(self.access_port)?;

        // let received_words = self.probe.probe.read_mem_8bit(
        //     address,
        //     data.len() as u16,
        //     self.access_port.port_number(),
        // )?;

        // data.copy_from_slice(&received_words);

        Ok(())
    }

    fn write_word_32(&mut self, address: u32, data: u32) -> Result<(), ProbeRsError> {
        // self.probe.select_ap(self.access_port)?;

        // self.write_32(address, &[data])?;

        Ok(())
    }

    fn write_word_8(&mut self, address: u32, data: u8) -> Result<(), ProbeRsError> {
        // self.probe.select_ap(self.access_port)?;

        self.write_8(address, &[data])
    }

    fn write_32(&mut self, address: u32, data: &[u32]) -> Result<(), ProbeRsError> {
        // self.probe.select_ap(self.access_port)?;

        // self.probe
        //     .probe
        //     .write_mem_32bit(address, data, self.access_port.port_number())?;
        Ok(())
    }

    fn write_8(&mut self, address: u32, data: &[u8]) -> Result<(), ProbeRsError> {
        // self.probe.select_ap(self.access_port)?;

        // let chunk_size = if self.probe.probe.hw_version < 3 {
        //     64
        // } else {
        //     512
        // };

        // // If we write less than 64 bytes, just write it directly
        // if data.len() < chunk_size {
        //     log::trace!("write_8: small - direct 8 bit write to {:08x}", address);
        //     self.probe
        //         .probe
        //         .write_mem_8bit(address, data, self.access_port.port_number())?;
        // } else {
        //     // Handle unaligned data in the beginning.
        //     let bytes_beginning = if address % 4 == 0 {
        //         0
        //     } else {
        //         (4 - address % 4) as usize
        //     };

        //     let mut current_address = address;

        //     if bytes_beginning > 0 {
        //         log::warn!(
        //             "write_8: at_begin - unaligned write of {} bytes to address {:08x}",
        //             bytes_beginning,
        //             current_address,
        //         );
        //         self.probe.probe.write_mem_8bit(
        //             current_address,
        //             &data[..bytes_beginning],
        //             self.access_port.port_number(),
        //         )?;

        //         current_address += bytes_beginning as u32;
        //     }

        //     // Address has to be aligned here.
        //     assert!(current_address % 4 == 0);

        //     // Convert bytes to u32
        //     let mut chunks_iter = (&data[bytes_beginning..]).chunks_exact(4);

        //     let words: Vec<u32> = (&mut chunks_iter)
        //         .map(|c| u32::from_le_bytes(c.try_into().unwrap()))
        //         .collect();

        //     log::trace!(
        //         "write_8: aligned write of {} bytes to address {:08x}",
        //         words.len() * 4,
        //         current_address,
        //     );

        //     self.probe.probe.write_mem_32bit(
        //         current_address,
        //         &words,
        //         self.access_port.port_number(),
        //     )?;

        //     current_address += (words.len() * 4) as u32;

        //     let remaining_bytes = chunks_iter.remainder();

        //     if !remaining_bytes.is_empty() {
        //         log::trace!(
        //             "write_8: at_end -unaligned write of {} bytes to address {:08x}",
        //             bytes_beginning,
        //             current_address,
        //         );
        //         self.probe.probe.write_mem_8bit(
        //             current_address,
        //             remaining_bytes,
        //             self.access_port.port_number(),
        //         )?;
        //     }
        // }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), ProbeRsError> {
//        self.probe.probe.flush()?;

        Ok(())
    }
}


#[derive(Debug)]
struct IcdiArmDebug {
    probe: Box<ICDI<ICDIUSBDevice>>,
    state: ArmCommunicationInterfaceState,
}

impl IcdiArmDebug {
    fn new(probe: Box<ICDI<ICDIUSBDevice>>) -> Result<Self, DebugProbeError> {
        let state = ArmCommunicationInterfaceState::new();

        // Determine the number and type of available APs.

        let mut interface = Self { probe, state };
        Ok(interface)
    }
}

impl<'probe> ArmProbeInterface for IcdiArmDebug {
    fn memory_interface(&mut self, access_port: MemoryAP) -> Result<Memory<'_>, ProbeRsError> {
        Ok(Memory::new(self))
    }

    fn ap_information(
        &self,
        access_port: crate::architecture::arm::ap::GenericAP,
    ) -> Option<&crate::architecture::arm::communication_interface::ApInformation> {
        Some(&crate::architecture::arm::communication_interface::ApInformation::MemoryAp {
            port_number: 0,
            only_32bit_data_size: false,
            debug_base_address: 0
        })
    }

    fn read_from_rom_table(
        &mut self,
    ) -> Result<Option<crate::architecture::arm::ArmChipInfo>, ProbeRsError> {
//        unimplemented!();
        Ok(None)
    }

    fn num_access_ports(&self) -> usize {
        1
        //        self.state.ap_information.len()
    }

    fn close(self: Box<Self>) -> Probe {
        Probe::from_attached_probe(self.probe)
    }
}

impl<'a> AsRef<dyn DebugProbe + 'a> for IcdiArmDebug {
    fn as_ref(&self) -> &(dyn DebugProbe + 'a) {
        self.probe.as_ref()
    }
}

impl<'a> AsMut<dyn DebugProbe + 'a> for IcdiArmDebug {
    fn as_mut(&mut self) -> &mut (dyn DebugProbe + 'a) {
        self.probe.as_mut()
    }
}

impl SwoAccess for IcdiArmDebug {
    fn enable_swo(&mut self, config: &SwoConfig) -> Result<(), ProbeRsError> {
        unimplemented!();
        //        self.probe.enable_swo(config)
    }

    fn disable_swo(&mut self) -> Result<(), ProbeRsError> {
        unimplemented!();
//        self.probe.disable_swo()
    }

    fn read_swo_timeout(&mut self, timeout: Duration) -> Result<Vec<u8>, ProbeRsError> {
        unimplemented!();
//        self.probe.read_swo_timeout(timeout)
    }
}

impl MemoryInterface for IcdiArmDebug {
    fn read_word_32(&mut self, address: u32) -> Result<u32, ProbeRsError> {
        log::trace!("read_word_32 {:08x}", address);

        let received_words = self.probe.read_mem_32bit(address, 1)?;

        Ok(received_words[0])

//        unimplemented!();
    }

    fn read_word_8(&mut self, address: u32) -> Result<u8, ProbeRsError> {
        log::trace!("read_word_8 {:08x}", address);
        unimplemented!();
    }

    fn read_32(&mut self, address: u32, data: &mut [u32]) -> Result<(), ProbeRsError> {
        log::trace!("read_32 {:08x} len {}", address, data.len());
        unimplemented!();
    }

    fn read_8(&mut self, address: u32, data: &mut [u8]) -> Result<(), ProbeRsError> {
        log::trace!("read_8 {:08x} len {}", address, data.len());
        unimplemented!();
    }

    fn write_word_32(&mut self, address: u32, data: u32) -> Result<(), ProbeRsError> {
        log::trace!("write_word_32 {:08x} => {:08x}", address, data);
        unimplemented!();
    }

    fn write_word_8(&mut self, address: u32, data: u8) -> Result<(), ProbeRsError> {
        unimplemented!();
    }

    fn write_32(&mut self, address: u32, data: &[u32]) -> Result<(), ProbeRsError> {
        unimplemented!();
    }

    fn write_8(&mut self, address: u32, data: &[u8]) -> Result<(), ProbeRsError> {
        unimplemented!();
    }

    fn flush(&mut self) -> Result<(), ProbeRsError> {
        unimplemented!();
    }
}


// #[cfg(test)]
// mod test {

//     use super::{constants::commands, usb_interface::StLinkUsb, STLink};
//     use crate::{DebugProbeError, WireProtocol};

//     use scroll::Pwrite;

//     #[derive(Debug)]
//     struct MockUsb {
//         hw_version: u8,
//         jtag_version: u8,
//         swim_version: u8,

//         target_voltage_a0: f32,
//         target_voltage_a1: f32,
//     }

//     impl MockUsb {
//         fn build(self) -> STLink<MockUsb> {
//             STLink {
//                 device: self,
//                 hw_version: 0,
//                 protocol: WireProtocol::Swd,
//                 jtag_version: 0,
//                 swd_speed_khz: 0,
//                 jtag_speed_khz: 0,
//                 swo_enabled: false,
//                 openend_aps: vec![],
//             }
//         }
//     }

//     impl StLinkUsb for MockUsb {
//         fn write(
//             &mut self,
//             cmd: &[u8],
//             _write_data: &[u8],
//             read_data: &mut [u8],
//             _timeout: std::time::Duration,
//         ) -> Result<(), crate::DebugProbeError> {
//             match cmd[0] {
//                 commands::GET_VERSION => {
//                     // GET_VERSION response structure:
//                     //   Byte 0-1:
//                     //     [15:12] Major/HW version
//                     //     [11:6]  JTAG/SWD version
//                     //     [5:0]   SWIM or MSC version
//                     //   Byte 2-3: ST_VID
//                     //   Byte 4-5: STLINK_PID

//                     let version: u16 = ((self.hw_version as u16) << 12)
//                         | ((self.jtag_version as u16) << 6)
//                         | ((self.swim_version as u16) << 0);

//                     read_data[0] = (version >> 8) as u8;
//                     read_data[1] = version as u8;

//                     Ok(())
//                 }
//                 commands::GET_TARGET_VOLTAGE => {
//                     read_data.pwrite(self.target_voltage_a0, 0).unwrap();
//                     read_data.pwrite(self.target_voltage_a0, 4).unwrap();
//                     Ok(())
//                 }
//                 commands::JTAG_COMMAND => {
//                     // Return a status of OK for JTAG commands
//                     read_data[0] = 0x80;

//                     Ok(())
//                 }
//                 _ => Ok(()),
//             }
//         }
//         fn reset(&mut self) -> Result<(), crate::DebugProbeError> {
//             Ok(())
//         }

//         fn read_swo(
//             &mut self,
//             _read_data: &mut [u8],
//             _timeout: std::time::Duration,
//         ) -> Result<usize, DebugProbeError> {
//             unimplemented!("Not implemented for MockUSB")
//         }
//     }

//     #[test]
//     fn detect_old_firmware() {
//         // Test that the init function detects old, unsupported firmware.

//         let usb_mock = MockUsb {
//             hw_version: 2,
//             jtag_version: 20,
//             swim_version: 0,

//             target_voltage_a0: 1.0,
//             target_voltage_a1: 2.0,
//         };

//         let mut probe = usb_mock.build();

//         let init_result = probe.init();

//         match init_result.unwrap_err() {
//             DebugProbeError::ProbeFirmwareOutdated => (),
//             other => panic!("Expected firmware outdated error, got {}", other),
//         }
//     }

//     #[test]
//     fn firmware_without_multiple_ap_support() {
//         // Test that firmware with only support for a single AP works,
//         // as long as only AP 0 is selected

//         let usb_mock = MockUsb {
//             hw_version: 2,
//             jtag_version: 26,
//             swim_version: 0,
//             target_voltage_a0: 1.0,
//             target_voltage_a1: 2.0,
//         };

//         let mut probe = usb_mock.build();

//         probe.init().expect("Init function failed");

//         // Selecting AP 0 should still work
//         probe.select_ap(0).expect("Select AP 0 failed.");

//         probe
//             .select_ap(1)
//             .expect_err("Selecting AP other than AP 0 should fail");
//     }

//     #[test]
//     fn firmware_with_multiple_ap_support() {
//         // Test that firmware with only support for a single AP works,
//         // as long as only AP 0 is selected

//         let usb_mock = MockUsb {
//             hw_version: 2,
//             jtag_version: 30,
//             swim_version: 0,
//             target_voltage_a0: 1.0,
//             target_voltage_a1: 2.0,
//         };

//         let mut probe = usb_mock.build();

//         probe.init().expect("Init function failed");

//         // Selecting AP 0 should still work
//         probe.select_ap(0).expect("Select AP 0 failed.");

//         probe
//             .select_ap(1)
//             .expect("Selecting AP other than AP 0 should work");
//     }
// }
