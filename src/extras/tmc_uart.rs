// Helper code for communicating with TMC stepper drivers via UART
// (port of klippy/extras/tmc_uart.py)
//
// Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use crate::configfile::{ConfigWrapper, Dflt, Lim};
use crate::error::{KError, KResult};
use crate::extras::tmc::{FieldHelper, RawRead, TmcMcu};
use crate::mcu::{get_printer_mcu, CommandQueryWrapper, CommandWrapper, Mcu};
use crate::msgproto::Val;
use crate::pins::{PinsObject, SharedPinParams};
use crate::printer::{downcast_rc, Printer, PrinterObject};
use crate::reactor::ReactorMutex;
use crate::chelper::serialqueue::CommandQueue;
use indexmap::IndexMap;
use std::any::Any;
use std::cell::RefCell;
use std::rc::Rc;

fn same_rc<A: ?Sized, B: ?Sized>(a: &Rc<A>, b: &Rc<B>) -> bool {
    Rc::as_ptr(a) as *const () == Rc::as_ptr(b) as *const ()
}

/// the mcu a pin is on (pin_params["chip"])
fn pin_mcu(printer: &Printer, pin_params: &SharedPinParams) -> KResult<Rc<Mcu>> {
    let chip_name = pin_params.borrow().chip_name.clone();
    get_printer_mcu(printer, &chip_name)
}

// ----------------------------------------------------------------------
// TMC uart analog mux support
// ----------------------------------------------------------------------

pub struct McuAnalogMux {
    mcu: Rc<Mcu>,
    oids: Vec<i64>,
    pins: Vec<String>,
    pin_values: RefCell<Vec<i64>>,
    update_pin_cmd: RefCell<Option<CommandWrapper>>,
}

impl McuAnalogMux {
    fn new(mcu: &Rc<Mcu>, cmd_queue: &CommandQueue, select_pins_desc: &[String]) -> KResult<Rc<Self>> {
        let printer = mcu.get_printer();
        let ppins = printer.lookup::<PinsObject>("pins")?;
        let mut pins = vec![];
        for spd in select_pins_desc {
            let pp = ppins.0.borrow_mut().lookup_pin(spd, true, false, None).map_err(|e| KError::Pins(e.0))?;
            pins.push(pp.borrow().pin.clone());
        }
        let oids: Vec<i64> = pins.iter().map(|_| mcu.create_oid()).collect();
        for (oid, pin) in oids.iter().zip(&pins) {
            mcu.add_config_cmd(&format!("config_digital_out oid={} pin={} value=0 default_value=0 max_duration=0", oid, pin), false, false);
        }
        let m = Rc::new(McuAnalogMux {
            mcu: mcu.clone(),
            pin_values: RefCell::new(vec![-1; pins.len()]),
            oids,
            pins,
            update_pin_cmd: RefCell::new(None),
        });
        let (mm, cq) = (m.clone(), cmd_queue.clone());
        mcu.register_config_callback(Rc::new(move || {
            *mm.update_pin_cmd.borrow_mut() = Some(mm.mcu.lookup_command("update_digital_out oid=%c value=%c", Some(cq.clone()))?);
            Ok(())
        }));
        Ok(m)
    }

    fn get_instance_id(&self, select_pins_desc: &[String]) -> KResult<Vec<i64>> {
        let printer = self.mcu.get_printer();
        let ppins = printer.lookup::<PinsObject>("pins")?;
        let mut params = vec![];
        for spd in select_pins_desc {
            params.push(ppins.0.borrow().parse_pin(spd, true, false).map_err(|e| KError::Pins(e.0))?);
        }
        for pp in &params {
            if !same_rc(&pp.chip, &self.mcu) {
                return Err(KError::config("TMC mux pins must be on the same mcu"));
            }
        }
        let pins: Vec<String> = params.iter().map(|pp| pp.pin.clone()).collect();
        if pins != self.pins {
            return Err(KError::config("All TMC mux instances must use identical pins"));
        }
        Ok(params.iter().map(|pp| (pp.invert == 0) as i64).collect())
    }

    fn activate(&self, instance_id: &[i64]) -> KResult<()> {
        let old_values = self.pin_values.borrow().clone();
        for ((oid, old), new) in self.oids.iter().zip(&old_values).zip(instance_id) {
            if old != new {
                if let Some(cmd) = self.update_pin_cmd.borrow().as_ref() {
                    cmd.send(&[Val::Int(*oid), Val::Int(*new)], 0, 0)?;
                }
            }
        }
        *self.pin_values.borrow_mut() = instance_id.to_vec();
        Ok(())
    }
}

// ----------------------------------------------------------------------
// TMC uart communication
// ----------------------------------------------------------------------

// Share mutexes so only one active tmc_uart command on a single mcu at
// a time. This helps limit cpu usage on slower micro-controllers.
pub struct PrinterTMCUartMutexes {
    mcu_to_mutex: RefCell<Vec<(Rc<Mcu>, Rc<ReactorMutex>)>>,
}

impl PrinterObject for PrinterTMCUartMutexes {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

fn lookup_tmc_uart_mutex(mcu: &Rc<Mcu>) -> KResult<Rc<ReactorMutex>> {
    let printer = mcu.get_printer();
    let pmutexes = match printer.lookup_object_opt("tmc_uart").and_then(downcast_rc::<PrinterTMCUartMutexes>) {
        Some(p) => p,
        None => {
            let p = Rc::new(PrinterTMCUartMutexes { mcu_to_mutex: RefCell::new(vec![]) });
            printer.add_object("tmc_uart", p.clone())?;
            p
        }
    };
    if let Some((_, m)) = pmutexes.mcu_to_mutex.borrow().iter().find(|(m, _)| Rc::ptr_eq(m, mcu)) {
        return Ok(m.clone());
    }
    let mutex = printer.get_reactor().mutex(false);
    pmutexes.mcu_to_mutex.borrow_mut().push((mcu.clone(), mutex.clone()));
    Ok(mutex)
}

const TMC_BAUD_RATE: f64 = 40000.0;
const TMC_BAUD_RATE_AVR: f64 = 9000.0;

/// the analog mux instance id (None without select_pins)
type InstanceId = Option<Vec<i64>>;

// Code for sending messages on a TMC uart
pub struct McuTmcUartBitbang {
    mcu: Rc<Mcu>,
    mutex: Rc<ReactorMutex>,
    pullup: i64,
    rx_pin: String,
    tx_pin: String,
    oid: i64,
    analog_mux: Option<Rc<McuAnalogMux>>,
    instances: RefCell<Vec<(InstanceId, i64)>>,
    tmcuart_send_cmd: RefCell<Option<CommandQueryWrapper>>,
}

impl McuTmcUartBitbang {
    fn new(mcu: &Rc<Mcu>, rx_pin_params: &SharedPinParams, tx_pin_params: &SharedPinParams, select_pins_desc: Option<&[String]>) -> KResult<Rc<Self>> {
        let mutex = lookup_tmc_uart_mutex(mcu)?;
        let oid = mcu.create_oid();
        let cmd_queue = mcu.alloc_command_queue();
        let analog_mux = match select_pins_desc {
            Some(spd) => Some(McuAnalogMux::new(mcu, &cmd_queue, spd)?),
            None => None,
        };
        let u = Rc::new(McuTmcUartBitbang {
            mcu: mcu.clone(),
            mutex,
            pullup: rx_pin_params.borrow().pullup,
            rx_pin: rx_pin_params.borrow().pin.clone(),
            tx_pin: tx_pin_params.borrow().pin.clone(),
            oid,
            analog_mux,
            instances: RefCell::new(vec![]),
            tmcuart_send_cmd: RefCell::new(None),
        });
        let uu = u.clone();
        mcu.register_config_callback(Rc::new(move || uu.build_config(&cmd_queue)));
        Ok(u)
    }

    fn build_config(&self, cmd_queue: &CommandQueue) -> KResult<()> {
        let mut baud = TMC_BAUD_RATE;
        let consts = self.mcu.get_constants();
        let mcu_type = consts.get("MCU").and_then(|v| v.as_str()).unwrap_or("");
        if mcu_type.starts_with("atmega") || mcu_type.starts_with("at90usb") {
            baud = TMC_BAUD_RATE_AVR;
        }
        let bit_ticks = self.mcu.seconds_to_clock(1.0 / baud);
        self.mcu.add_config_cmd(
            &format!(
                "config_tmcuart oid={} rx_pin={} pull_up={} tx_pin={} bit_time={}",
                self.oid, self.rx_pin, self.pullup, self.tx_pin, bit_ticks
            ),
            false,
            false,
        );
        *self.tmcuart_send_cmd.borrow_mut() = Some(self.mcu.lookup_query_command(
            "tmcuart_send oid=%c write=%*s read=%c",
            "tmcuart_response oid=%c read=%*s",
            Some(self.oid),
            Some(cmd_queue.clone()),
            true,
        )?);
        Ok(())
    }

    fn register_instance(
        &self,
        rx_pin_params: &SharedPinParams,
        tx_pin_params: &SharedPinParams,
        select_pins_desc: Option<&[String]>,
        addr: i64,
    ) -> KResult<InstanceId> {
        if rx_pin_params.borrow().pin != self.rx_pin
            || tx_pin_params.borrow().pin != self.tx_pin
            || select_pins_desc.is_none() != self.analog_mux.is_none()
        {
            return Err(KError::config("Shared TMC uarts must use the same pins"));
        }
        let instance_id = match (&self.analog_mux, select_pins_desc) {
            (Some(m), Some(spd)) => Some(m.get_instance_id(spd)?),
            _ => None,
        };
        let key = (instance_id.clone(), addr);
        if self.instances.borrow().contains(&key) {
            return Err(KError::config("Shared TMC uarts need unique address or select_pins polarity"));
        }
        self.instances.borrow_mut().push(key);
        Ok(instance_id)
    }

    fn calc_crc8(data: &[u8]) -> u8 {
        // Generate a CRC8-ATM value for a bytearray
        let mut crc: u8 = 0;
        for &b in data {
            let mut b = b;
            for _ in 0..8 {
                if (crc >> 7) ^ (b & 0x01) != 0 {
                    crc = (crc << 1) ^ 0x07;
                } else {
                    crc <<= 1;
                }
                b >>= 1;
            }
        }
        crc
    }

    fn add_serial_bits(data: &[u8]) -> Vec<u8> {
        // Add serial start and stop bits to a message in a bytearray
        let mut out: u128 = 0;
        let mut pos = 0;
        for &d in data {
            let b = ((d as u128) << 1) | 0x200;
            out |= b << pos;
            pos += 10;
        }
        (0..(pos + 7) / 8).map(|i| ((out >> (i * 8)) & 0xFF) as u8).collect()
    }

    fn encode_read(sync: u8, addr: u8, reg: u8) -> Vec<u8> {
        // Generate a uart read register message
        let mut msg = vec![sync, addr, reg];
        msg.push(Self::calc_crc8(&msg));
        Self::add_serial_bits(&msg)
    }

    fn encode_write(sync: u8, addr: u8, reg: u8, val: i64) -> Vec<u8> {
        // Generate a uart write register message
        let mut msg = vec![sync, addr, reg, (val >> 24) as u8, (val >> 16) as u8, (val >> 8) as u8, val as u8];
        msg.push(Self::calc_crc8(&msg));
        Self::add_serial_bits(&msg)
    }

    fn decode_read(reg: u8, data: &[u8]) -> Option<i64> {
        // Extract a uart read response message
        if data.len() != 10 {
            return None;
        }
        // Convert data into a long integer for easy manipulation
        let mut mval: u128 = 0;
        for (i, &d) in data.iter().enumerate() {
            mval |= (d as u128) << (i * 8);
        }
        // Extract register value
        let val = ((((mval >> 31) & 0xFF) << 24) | (((mval >> 41) & 0xFF) << 16) | (((mval >> 51) & 0xFF) << 8) | ((mval >> 61) & 0xFF)) as i64;
        // Verify start/stop bits and crc
        if data != Self::encode_write(0x05, 0xFF, reg, val).as_slice() {
            return None;
        }
        Some(val)
    }

    fn send_cmd(&self, data: &[Val], minclock: u64) -> KResult<crate::msgproto::Params> {
        let cmd = self.tmcuart_send_cmd.borrow();
        let cmd = cmd.as_ref().ok_or_else(|| KError::Other("'NoneType' object has no attribute 'send'".into()))?;
        cmd.send(data, minclock, 0, true)
    }

    fn reg_read(&self, instance_id: &InstanceId, addr: i64, reg: i64) -> KResult<(Option<i64>, f64)> {
        if let (Some(m), Some(id)) = (&self.analog_mux, instance_id) {
            m.activate(id)?;
        }
        let msg = Self::encode_read(0xF5, addr as u8, reg as u8);
        let params = self.send_cmd(&[Val::Int(self.oid), Val::Bytes(msg), Val::Int(10)], 0)?;
        let data = match params.get("read") {
            Some(Val::Bytes(b)) => Self::decode_read(reg as u8, b),
            _ => None,
        };
        let receive_time = match params.get("#receive_time") {
            Some(Val::Float(t)) => *t,
            _ => 0.0,
        };
        Ok((data, receive_time))
    }

    fn reg_write(&self, instance_id: &InstanceId, addr: i64, reg: i64, val: i64, print_time: Option<f64>) -> KResult<()> {
        let mut minclock = 0;
        if let Some(pt) = print_time {
            minclock = self.mcu.print_time_to_clock(pt);
        }
        if let (Some(m), Some(id)) = (&self.analog_mux, instance_id) {
            m.activate(id)?;
        }
        let msg = Self::encode_write(0xF5, addr as u8, (reg | 0x80) as u8, val);
        self.send_cmd(&[Val::Int(self.oid), Val::Bytes(msg), Val::Int(0)], minclock as u64)?;
        Ok(())
    }
}

// Lookup a (possibly shared) tmc uart
fn lookup_tmc_uart_bitbang(config: &ConfigWrapper, max_addr: i64) -> KResult<(InstanceId, i64, Rc<McuTmcUartBitbang>)> {
    let printer = config.get_printer();
    let ppins = printer.lookup::<PinsObject>("pins")?;
    let uart_pin = config.get("uart_pin", Dflt::Required)?;
    let rx_pin_params = ppins.0.borrow_mut().lookup_pin(&uart_pin, false, true, Some("tmc_uart_rx")).map_err(|e| KError::Pins(e.0))?;
    let tx_pin_params = match config.get_opt("tx_pin")? {
        None => rx_pin_params.clone(),
        Some(d) => ppins.0.borrow_mut().lookup_pin(&d, false, false, Some("tmc_uart_tx")).map_err(|e| KError::Pins(e.0))?,
    };
    if !same_rc(&rx_pin_params.borrow().chip, &tx_pin_params.borrow().chip) {
        return Err(KError::Pins("TMC uart rx and tx pins must be on the same mcu".into()));
    }
    let select_pins_desc = config.getlist("select_pins", Dflt::None, ",", None)?;
    let addr = config.getint("uart_address", 0, Lim::new().min(0).max(max_addr))?;
    let existing = rx_pin_params.borrow().class.clone();
    let mcu_uart = match existing {
        Some(c) => c.downcast::<McuTmcUartBitbang>().map_err(|_| KError::Other("tmc uart pin class has an unexpected type".into()))?,
        None => {
            let mcu = pin_mcu(&printer, &rx_pin_params)?;
            let u = McuTmcUartBitbang::new(&mcu, &rx_pin_params, &tx_pin_params, select_pins_desc.as_deref())?;
            rx_pin_params.borrow_mut().class = Some(u.clone());
            u
        }
    };
    let instance_id = mcu_uart.register_instance(&rx_pin_params, &tx_pin_params, select_pins_desc.as_deref(), addr)?;
    Ok((instance_id, addr, mcu_uart))
}

// Helper code for communicating via TMC uart
pub struct McuTmcUart {
    printer: Rc<Printer>,
    name: String,
    name_to_reg: IndexMap<String, i64>,
    fields: Rc<FieldHelper>,
    ifcnt: RefCell<Option<i64>>,
    instance_id: InstanceId,
    addr: i64,
    mcu_uart: Rc<McuTmcUartBitbang>,
    tmc_frequency: f64,
}

impl McuTmcUart {
    pub fn new(
        config: &ConfigWrapper,
        name_to_reg: &[(&str, i64)],
        fields: Rc<FieldHelper>,
        max_addr: i64,
        tmc_frequency: f64,
    ) -> KResult<Rc<Self>> {
        let (instance_id, addr, mcu_uart) = lookup_tmc_uart_bitbang(config, max_addr)?;
        Ok(Rc::new(McuTmcUart {
            printer: config.get_printer(),
            name: config.get_name().split_whitespace().last().unwrap_or("").to_string(),
            name_to_reg: name_to_reg.iter().map(|(k, v)| (k.to_string(), *v)).collect(),
            fields,
            ifcnt: RefCell::new(None),
            instance_id,
            addr,
            mcu_uart,
            tmc_frequency,
        }))
    }

    fn reg(&self, reg_name: &str) -> KResult<i64> {
        self.name_to_reg.get(reg_name).copied().ok_or_else(|| KError::Other(crate::pyfmt::py_str_repr(reg_name)))
    }

    fn do_get_register(&self, reg_name: &str) -> KResult<RawRead> {
        let reg = self.reg(reg_name)?;
        if self.printer.start_arg("debugoutput").is_some() {
            return Ok(RawRead { data: 0, receive_time: 0.0, spi_status: None });
        }
        for _retry in 0..5 {
            let (data, receive_time) = self.mcu_uart.reg_read(&self.instance_id, self.addr, reg)?;
            if let Some(data) = data {
                return Ok(RawRead { data, receive_time, spi_status: None });
            }
        }
        Err(KError::command(format!("Unable to read tmc uart '{}' register {}", self.name, reg_name)))
    }

    fn with_mutex<R>(&self, f: impl FnOnce() -> KResult<R>) -> KResult<R> {
        self.mcu_uart.mutex.lock()?;
        let r = f();
        self.mcu_uart.mutex.unlock();
        r
    }
}

impl TmcMcu for McuTmcUart {
    fn get_fields(&self) -> Rc<FieldHelper> {
        self.fields.clone()
    }

    fn get_register_raw(&self, reg_name: &str) -> KResult<RawRead> {
        self.with_mutex(|| self.do_get_register(reg_name))
    }

    fn set_register(&self, reg_name: &str, val: i64, print_time: Option<f64>) -> KResult<()> {
        let reg = self.reg(reg_name)?;
        if self.printer.start_arg("debugoutput").is_some() {
            return Ok(());
        }
        self.with_mutex(|| {
            for _retry in 0..5 {
                // (the cached value, not the borrow: the None arm stores into the cell)
                let cached = *self.ifcnt.borrow();
                let ifcnt = match cached {
                    Some(i) => i,
                    None => {
                        let i = self.do_get_register("IFCNT")?.data;
                        *self.ifcnt.borrow_mut() = Some(i);
                        i
                    }
                };
                self.mcu_uart.reg_write(&self.instance_id, self.addr, reg, val, print_time)?;
                let new = self.do_get_register("IFCNT")?.data;
                *self.ifcnt.borrow_mut() = Some(new);
                if new == (ifcnt + 1) & 0xFF {
                    return Ok(());
                }
            }
            Err(KError::command(format!("Unable to write tmc uart '{}' register {}", self.name, reg_name)))
        })
    }

    fn get_tmc_frequency(&self) -> Option<f64> {
        Some(self.tmc_frequency)
    }

    fn get_mcu(&self) -> Rc<Mcu> {
        self.mcu_uart.mcu.clone()
    }

    fn name_to_reg(&self, reg_name: &str) -> i64 {
        self.name_to_reg.get(reg_name).copied().unwrap_or(0)
    }
}

#[cfg(test)]
mod tests {
    use super::McuTmcUartBitbang as U;

    #[test]
    fn uart_encoding_roundtrip() {
        // a write message decodes back as a read reply of the same value
        let msg = U::encode_write(0x05, 0xFF, 0x6C, 0x12345678);
        assert_eq!(msg.len(), 10);
        assert_eq!(U::decode_read(0x6C, &msg), Some(0x12345678));
        // reference bytes from tmc_uart.py
        assert_eq!(U::encode_write(0xF5, 3, 0x90, 0x00071703), [234, 27, 8, 50, 128, 14, 186, 104, 32, 172]);
        assert_eq!(U::encode_read(0xF5, 1, 0x02), [234, 11, 72, 32, 191]);
    }
}
