pub mod constants;
pub mod tools;
mod usb_interface;

use self::usb_interface::{ICDIUSBDevice, IcdiUsb};
use super::{DAPAccess, DebugProbe, DebugProbeError, PortType, ProbeCreationError, WireProtocol};
use crate::{
    architecture::arm::communication_interface::MemoryApInformation,
    architecture::arm::{
        ap::{
            valid_access_ports, APAccess, APClass, APRegister, AccessPort, BaseaddrFormat,
            GenericAP, MemoryAP, BASE, BASE2, CSW, IDR,
        },
        communication_interface::{ArmCommunicationInterfaceState, ArmProbeInterface},
        dp::{DPAccess, DPBankSel, DPRegister, DebugPortError, Select},
        memory::{adi_v5_memory_interface::ArmProbe, Component},

        ApInformation, ArmChipInfo, SwoAccess, SwoConfig,
    },
    DebugProbeSelector, Error as ProbeRsError, Memory, MemoryInterface, Probe,
};
use constants::{commands, JTagFrequencyToDivider, Mode, Status, SwdFrequencyToDelayCount};
use scroll::{Pread, Pwrite, BE, LE};
use std::{cmp::Ordering, convert::TryInto, time::Duration};
use thiserror::Error;
use usb_interface::TIMEOUT;

// ICDI protocol reversed from openocd and lm4flash tools.
//
// All commands starts by $ and finish by # and a 2 bytes checksum
// example : $foo#xx
//
//
// Abstract:
// Each packet has the form:
// $ <command-and-payload> # <checksum>
//
// The normal reply has the same format:
// $ <reply> # <checksum>
//
// Reply can also be a NACK:
// -
//
// Read memory command:
// -> "x<AAAAAAAA>,<SZ>" : Read SZ bytes starting from address AAAAAAAA
// <- "-" : NACK, may retry
// <- "OK:<BBBBBBBBB>" : Reply with bytes in BBBBBBBBBB. Checksum on B only.
// <- "E<XX>" : In case of error XX
//
// Write memory command:
// -> "X<AAAAAAAA>,<SZ>:<BBBBB>" : Write SZ bytes in BBBBB starting from address AAAAAAAA
// <- "+" : Acknowledge
//
// Continue execution
// -> "c"
//
// Halt execution
// -> "?"
//
// Step execution
// -> "s"
//
// Read register
// -> "p<regno>"
// <- "+$<4-bytes-value>"
//
// Write register
// -> "P<regno>=<4-bytes-val>"
// <- "+$OK" or "+$" ?

// qSupported
//
// Flash erase:
// -> vFlashErase:<start>,<end>
//
// Flash write:
// -> vFlashWrite:<addr-08x>:<escaped-bytes>

// Set extended mode
// -> "!": Set extended mode
// <- "OK": Success
// <- "E<XX>": Error XX
//
// Send remote command
// -> "qRcmd,<CCCCCCCCC>": Send remote command CCCCCC. CCCCCC must be hex-encoded
// -> "-": NACK, may retry
// ->

// lm4flash:
// qRcmd,version -> +$
//
// openocd:
// - icdi_send_cmd : retry send on '-', error on '+'. Sends command and reads anything avail from read_ep. Error if not ending by #xx.
//   - checks result with icdi_get_cmd_result:
//     - $OK -> return OK without looking deeper
//     - $Exx -> return errors
//
// - icdi_send_remote_cmd: serialize then icdi_send_cmd (ie. does not check for OK/Exx, only -/+)
//   - expects reply of the form : +$<payload>
//

#[derive(Debug)]
pub struct ICDI<D: IcdiUsb> {
    device: D,

    icdi_version: u32,
    packet_size: usize,

}

impl DebugProbe for ICDI<ICDIUSBDevice> {
    fn new_from_selector(
        selector: impl Into<DebugProbeSelector>,
    ) -> Result<Box<Self>, DebugProbeError> {
        let mut icdi = Self {
            device: ICDIUSBDevice::new_from_selector(selector)?,
            icdi_version: 0,
            packet_size: 0,
        };

        icdi.init()?;

        Ok(Box::new(icdi))
    }

    fn get_name(&self) -> &str {
        "ICDI"
    }

    fn speed(&self) -> u32 {
        log::debug!("speed");
        100
    }

    fn set_speed(&mut self, speed_khz: u32) -> Result<u32, DebugProbeError> {
        log::debug!("set_speed");
        Ok(100)
    }

    fn attach(&mut self) -> Result<(), DebugProbeError> {
        log::debug!("attach");
        Ok(())
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
        log::error!("ICDI target_reset_deassert");
        unimplemented!()
    }

    fn select_protocol(&mut self, protocol: WireProtocol) -> Result<(), DebugProbeError> {
        // if protocol != WireProtocol::Jtag {
        //     Err(DebugProbeError::UnsupportedProtocol(protocol))
        // } else {
            Ok(())
        // }
    }
    /// Check if the probe offers an interface to debug ARM chips.
    fn has_arm_interface(&self) -> bool {
        true
    }

    /// Get the dedicated interface to debug ARM chips. Ensure that the
    /// probe actually supports this by calling `has_arm_interface` first.
    fn get_arm_interface<'probe>(
        self: Box<Self>,
    ) -> Result<Option<Box<dyn ArmProbeInterface + 'probe>>, DebugProbeError> {
        let interface = IcdiArmDebug::new(self)?;

        Ok(Some(Box::new(interface)))
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

    fn get_version(&mut self) -> Result<u32, DebugProbeError> {

        log::debug!("Get version");

        let mut version = [0u8; 10];
        self.device
            .write_remote(b"version", &mut version, TIMEOUT)?;
        let vs = hex::decode(version).unwrap();

        let v = std::str::from_utf8(&vs[..vs.len() - 1]).unwrap();
        log::debug!("version {:?}", v);
        let vint = v.parse::<u32>().unwrap();

        Ok(vint)
    }

    fn set_extended_mode(&mut self) -> Result<(), DebugProbeError> {
        log::debug!("Get supported");

        let mut reply = [0u8; 2];
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

    fn init(&mut self) -> Result<(), DebugProbeError> {
        log::debug!("Initializing ICDI...");

        log::debug!("First empty read queue from device, if any...");
        let mut purge_buffer = [0u8; 256];
        if let Err(e) = self.device.write(&[], &mut purge_buffer, TIMEOUT) {
            match e {
                DebugProbeError::USB(_) => (),
                _ => return Err(e)
            }
        }

        log::debug!("Will get version");
        self.icdi_version = self.get_version()?;
        log::debug!("ICDI version: {:?}", self.icdi_version);
        self.packet_size = self.get_supported()?;
        log::debug!("ICDI packet size: {:?}", self.packet_size);
        self.set_extended_mode()?;

        Ok(())
    }

    fn read_mem_32bit(
        &mut self,
        address: u32,
        data: &mut [u8],
    ) -> Result<(), DebugProbeError> {
        log::debug!(
            "Read mem 32 bit, address={:08x}, length={}",
            address,
            data.len()
        );

//    fn read_mem_32bit(&mut self, address: u32, length: u16) -> Result<Vec<u32>, DebugProbeError> {

        // // Maximum supported read length is 2^16 bytes.
        // assert!(
        //     length < (u16::MAX / 4),
        //     "Maximum read length for STLink is 16'384 words"
        // );

        // if address % 4 != 0 {
        //     todo!("Should return an error here");
        // }

        //let byte_length = data.len() * 4;

        // Longest answer possible:
        // $OK#CC}x}x}x}x}x}x#CC
        // with :
        //  - CC the 2 checksum
        //  - }x an escaped byte payload (occupies 2 bytes)
        //        let mut receive_buffer = vec![0u8; (6 + byte_length*2 + 3) as usize];

//        let mut recv_data = vec![0u8; byte_length as usize];

        self.device.write(
            &format!("x{:08x},{:x}", address, data.len()).into_bytes(),
            data,
            TIMEOUT,
        )?;

        //        let reply = std::str::from_utf8(&recv_data).unwrap();
        log::debug!("result  {:x?}", data);
        //        check_result(reply)?;

        // self.get_last_rw_status()?;

        // let mut recv_payload = vec![0u8; byte_length as usize];
        // self.device.write(
        //     &[],
        //     &mut recv_payload,
        //     TIMEOUT,
        // )?;

        // let words: Vec<u32> = recv_data
        //     .chunks_exact(4)
        //     .map(|chunk| u32::from_le_bytes(chunk.try_into().unwrap()))
        //     .collect();

        Ok(())
    }

    fn read_mem_8bit(
        &mut self,
        address: u32,
        length: u16,
        apsel: u8,
    ) -> Result<Vec<u8>, DebugProbeError> {
        log::debug!("Read mem 8 bit, address={:08x}, length={}", address, length);
        Err(IcdiError::FixMeError(line!()).into())
    }

    fn write_mem_32bit(&mut self, address: u32, data: &[u32]) -> Result<(), DebugProbeError> {
        log::trace!("write_mem_32bit");
        let length = data.len();

        let byte_length = length * 4;

        let mut tx_buffer = vec![0u8; byte_length];

        let mut offset = 0;

        for word in data {
            tx_buffer
                .gwrite(word, &mut offset)
                .expect("Failed to write into tx_buffer");
        }

        let pkt_data = [
            format!("X{:08x},{:x}:", address, byte_length).into_bytes(),
            tx_buffer,
        ]
        .concat();

        self.device.write(&pkt_data, &mut [], TIMEOUT)?;

        Ok(())
    }

    fn write_mem_8bit(
        &mut self,
        address: u32,
        data: &[u8],
        apsel: u8,
    ) -> Result<(), DebugProbeError> {
        log::trace!("write_mem_8bit");
        Err(IcdiError::FixMeError(line!()).into())
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

        // Read needs to be chunked into chunks with appropiate max length (see STLINK_MAX_READ_LEN).
        for (index, chunk) in data.chunks_mut(32 / 4).enumerate() {
//        for (index, chunk) in data.chunks_mut(STLINK_MAX_READ_LEN / 4).enumerate() {
            let mut buff = vec![0u8; 4 * chunk.len()];

            let received_words = self.read_mem_32bit(
                address,
                &mut buff
            )?;

            // self.probe.probe.read_mem_32bit(
            //     address + (index * STLINK_MAX_READ_LEN) as u32,
            //     &mut buff,
            //     ap.port_number(),
            // )?;

            for (index, word) in buff.chunks_exact(4).enumerate() {
                chunk[index] = u32::from_le_bytes(word.try_into().unwrap());
            }
        }

//        data.copy_from_slice(&received_words);

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

        // FIXME: get somehow 2 AccessPort :
        // - one for memory
        // - one for registers
        // for ap in valid_access_ports(&mut interface) {
        //     let ap_state = interface.read_ap_information(ap)?;

        //     log::debug!("AP {}: {:?}", ap.port_number(), ap_state);

        interface.state.ap_information.push(ApInformation::MemoryAp(MemoryApInformation {
            port_number:0,
            only_32bit_data_size: true,
            supports_hnonsec: false,
            debug_base_address: 0
        }));
        interface.state.ap_information.push(ApInformation::Other {port_number:1});

        // }

        Ok(interface)
    }

    fn read_ap_information(
        &mut self,
        access_port: GenericAP,
    ) -> Result<ApInformation, DebugProbeError> {
        if access_port.port_number() == 0 {
            Ok(ApInformation::MemoryAp (MemoryApInformation {
                port_number:0,
                only_32bit_data_size: true,
                supports_hnonsec: false,
                debug_base_address: 0,
            }))
        } else {
            Ok(ApInformation::Other {port_number:1})
        }
    }

}

impl<'probe> ArmProbeInterface for IcdiArmDebug {
    fn memory_interface(&mut self, access_port: MemoryAP) -> Result<Memory<'_>, ProbeRsError> {
        let interface = IcdiMemoryInterface {probe: self};
        Ok(Memory::new(interface, access_port))
    }

    fn ap_information(
        &self,
        access_port: crate::architecture::arm::ap::GenericAP,
    ) -> Option<&crate::architecture::arm::communication_interface::ApInformation> {
        self.state
            .ap_information
            .get(access_port.port_number() as usize)
    }

    fn read_from_rom_table(
        &mut self,
    ) -> Result<Option<crate::architecture::arm::ArmChipInfo>, ProbeRsError> {
        //        unimplemented!();
        Ok(None)
    }

    fn num_access_ports(&self) -> usize {
        2
    }

    fn close(self: Box<Self>) -> Probe {
        Probe::from_attached_probe(self.probe)
    }
}

// impl<AP, R> APAccess<AP, R> for IcdiArmDebug
// where
//     R: APRegister<AP> + Clone,
//     AP: AccessPort,
// {
//     type Error = DebugProbeError;

//     fn read_ap_register(&mut self, port: impl Into<AP>, register: R) -> Result<R, Self::Error> {
//         unimplemented!();
//         self.read_ap_register(port, register)
//     }

//     fn write_ap_register(&mut self, port: impl Into<AP>, register: R) -> Result<(), Self::Error> {
//         unimplemented!();
//         self.write_ap_register(port, register)
//     }

//     fn write_ap_register_repeated(
//         &mut self,
//         port: impl Into<AP> + Clone,
//         register: R,
//         values: &[u32],
//     ) -> Result<(), Self::Error> {
//         unimplemented!();
//         self.write_ap_register_repeated(port, register, values)
//     }

//     fn read_ap_register_repeated(
//         &mut self,
//         port: impl Into<AP> + Clone,
//         register: R,
//         values: &mut [u32],
//     ) -> Result<(), Self::Error> {
//         unimplemented!();
//         self.read_ap_register_repeated(port, register, values)
//     }
// }

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


// This is forced but currently have no way to be implemented.
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

// impl MemoryInterface for IcdiArmDebug {
//     fn read_word_32(&mut self, address: u32) -> Result<u32, ProbeRsError> {
//         log::trace!("read_word_32 {:08x}", address);

//         let received_words = self.probe.read_mem_32bit(address, 1)?;

//         Ok(received_words[0])

//         //        unimplemented!();
//     }

//     fn read_word_8(&mut self, address: u32) -> Result<u8, ProbeRsError> {
//         log::trace!("read_word_8 {:08x}", address);
//         unimplemented!();
//     }

//     fn read_32(&mut self, address: u32, data: &mut [u32]) -> Result<(), ProbeRsError> {
//         log::trace!("read_32 {:08x} len {}", address, data.len());
//         let received_words = self.probe.read_mem_32bit(
//             address,
//             data.len() as u16
//         )?;

//         data.copy_from_slice(&received_words);

//         Ok(())
//     }

//     fn read_8(&mut self, address: u32, data: &mut [u8]) -> Result<(), ProbeRsError> {
//         log::trace!("read_8 {:08x} len {}", address, data.len());
//         unimplemented!();
//     }

//     fn write_word_32(&mut self, address: u32, data: u32) -> Result<(), ProbeRsError> {
//         log::trace!("write_word_32 {:08x} => {:08x}", address, data);

//         self.probe.write_mem_32bit(address, &[data])?;
//         Ok(())
//     }

//     fn write_word_8(&mut self, address: u32, data: u8) -> Result<(), ProbeRsError> {
//         unimplemented!();
//     }

//     fn write_32(&mut self, address: u32, data: &[u32]) -> Result<(), ProbeRsError> {
//         unimplemented!();
//     }

//     fn write_8(&mut self, address: u32, data: &[u8]) -> Result<(), ProbeRsError> {
//         unimplemented!();
//     }

//     fn flush(&mut self) -> Result<(), ProbeRsError> {
//         unimplemented!();
//     }
// }


#[derive(Debug)]
struct IcdiMemoryInterface<'probe> {
    probe: &'probe mut IcdiArmDebug,
}

impl ArmProbe for IcdiMemoryInterface<'_> {
    fn read_32(
        &mut self,
        ap: MemoryAP,
        address: u32,
        data: &mut [u32],
    ) -> Result<(), ProbeRsError> {
        log::trace!("read_32 {:08x} x {}", address, data.len());

        self.probe.probe.read_32(address, data);

        Ok(())
    }

    fn read_8(&mut self, ap: MemoryAP, address: u32, data: &mut [u8]) -> Result<(), ProbeRsError> {
        log::trace!("read_8 {:08x} x {}", address, data.len());
        Ok(())
    }

    fn write_32(&mut self, ap: MemoryAP, address: u32, data: &[u32]) -> Result<(), ProbeRsError> {
        log::trace!("write_32 @{:08x} x {}", address, data.len());
        Ok(())
    }

    fn write_8(&mut self, ap: MemoryAP, address: u32, data: &[u8]) -> Result<(), ProbeRsError> {
        log::trace!("write_8 @{:08x} x {}", address, data.len());
        Ok(())
    }

    fn flush(&mut self) -> Result<(), ProbeRsError> {
        log::trace!("flush");
        Ok(())
    }

    fn read_core_reg(
        &mut self,
        _ap: MemoryAP,
        addr: crate::CoreRegisterAddress,
    ) -> Result<u32, ProbeRsError> {
        log::trace!("read_core_reg");
        let value = 0;

        Ok(value)
    }

    fn write_core_reg(
        &mut self,
        _ap: MemoryAP,
        addr: crate::CoreRegisterAddress,
        value: u32,
    ) -> Result<(), ProbeRsError> {
        log::trace!("write_core_reg");
        Ok(())
    }
}
